#include "serialmanger.h"
#include <QDebug>

// --- Must match your STM32 code (F429/F407) ---
static constexpr quint8 FRAME_STX        = 0xAA;  // <-- set to your FRAME_STX
static constexpr quint8 FRAME_VERSION    = 0x01;  // <-- set to your FRAME_VERSION
static constexpr int    FRAME_PAYLOAD_LEN = 16;   // temp_in, temp_out, press, rpm, flags, alarm, uptime
static constexpr int    FRAME_SIZE        = 19;

static quint8 calc_checksum(const quint8 *data, int len)
{
    quint16 sum = 0;
    for (int i = 0; i < len; ++i) {
        sum += data[i];
    }
    return static_cast<quint8>(sum & 0xFF);
}

SerialManger::SerialManger(QObject *parent)
    : QObject{parent}
{
    // When data is available on the serial line:
    connect(&m_port, &QSerialPort::readyRead,
            this, &SerialManger::handleReadyRead);

    // When an error occurs:
    connect(&m_port, &QSerialPort::errorOccurred,
            this, &SerialManger::handleError);
}

void SerialManger::openPort(const QString &portName)
{
    if (m_port.isOpen())
        m_port.close();

    m_port.setPortName(portName);
    m_port.setBaudRate(QSerialPort::Baud115200);
    m_port.setDataBits(QSerialPort::Data8);
    m_port.setParity(QSerialPort::NoParity);
    m_port.setStopBits(QSerialPort::OneStop);
    m_port.setFlowControl(QSerialPort::NoFlowControl);

    if (!m_port.open(QIODevice::ReadWrite)) {
        emit errorOccurred(tr("Failed to open port: %1").arg(m_port.errorString()));
        return;
    }

    qDebug() << "Serial port opened:" << portName;
    emit connectedChanged();
}

void SerialManger::closePort()
{
    if (m_port.isOpen()) {
        m_port.close();
        qDebug() << "Serial port closed";
        emit connectedChanged();
    }
}

bool SerialManger::isConnected() const
{
    return m_port.isOpen();
}

void SerialManger::handleReadyRead()
{
    // Append all newly arrived bytes to our buffer
    m_rxBuffer.append(m_port.readAll());

    // Try to extract complete frames
    while (m_rxBuffer.size() >= FRAME_SIZE) {
        // Look for STX
        int stxIndex = m_rxBuffer.indexOf(char(FRAME_STX));
        if (stxIndex < 0) {
            // No STX at all, drop buffer
            m_rxBuffer.clear();
            return;
        }

        if (stxIndex > 0) {
            // Drop everything before STX
            m_rxBuffer.remove(0, stxIndex);
            if (m_rxBuffer.size() < FRAME_SIZE)
                return; // Wait for more bytes
        }

        if (m_rxBuffer.size() < FRAME_SIZE)
            return; // Not enough data yet

        // Now we have at least one full frame starting at index 0
        QByteArray frame = m_rxBuffer.left(FRAME_SIZE);
        m_rxBuffer.remove(0, FRAME_SIZE);

        if (!parseFrame(frame)) {
            qWarning() << "Invalid frame received, discarding";
            // Continue trying next possible frame in buffer
            continue;
        }
    }
}

bool SerialManger::parseFrame(const QByteArray &frame)
{
    if (frame.size() != FRAME_SIZE)
        return false;

    const quint8 *buf = reinterpret_cast<const quint8 *>(frame.constData());

    // 1) Basic header checks
    if (buf[0] != FRAME_STX)
        return false;

    if (buf[1] != FRAME_PAYLOAD_LEN)  // LEN = payload length
        return false;

    if (buf[2] != FRAME_VERSION)
        return false;

    // 2) Checksum
    quint8 expected = calc_checksum(buf, FRAME_SIZE - 1);
    quint8 actual   = buf[FRAME_SIZE - 1];
    if (expected != actual) {
        qWarning() << "Checksum mismatch. Expected" << expected << "got" << actual;
        return false;
    }

    // 3) Unpack fields (little-endian)
    int idx = 3;  // start of payload

    qint16  temp_in_q1  = static_cast<qint16>(buf[idx] | (buf[idx+1] << 8));
    idx += 2;

    qint16  temp_out_q1 = static_cast<qint16>(buf[idx] | (buf[idx+1] << 8));
    idx += 2;

    quint16 press_q2    = static_cast<quint16>(buf[idx] | (buf[idx+1] << 8));
    idx += 2;

    quint16 rpm_u16     = static_cast<quint16>(buf[idx] | (buf[idx+1] << 8));
    idx += 2;

    quint8  flags       = buf[idx++];

    quint16 alarm       = static_cast<quint16>(buf[idx] | (buf[idx+1] << 8));
    idx += 2;

    quint32 uptime      =  static_cast<quint32>(buf[idx])
                     | (static_cast<quint32>(buf[idx+1]) << 8)
                     | (static_cast<quint32>(buf[idx+2]) << 16)
                     | (static_cast<quint32>(buf[idx+3]) << 24);
    idx += 4;

    Q_UNUSED(idx); // silence warnings if not used further

    // 4) Convert to engineering units
    MachineState newState;
    newState.tempInlet   = static_cast<float>(temp_in_q1)  / 10.0f;
    newState.tempOutlet  = static_cast<float>(temp_out_q1) / 10.0f;
    newState.pressureBar = static_cast<float>(press_q2)    / 100.0f;
    newState.motorRpm    = static_cast<float>(rpm_u16);

    newState.estopActive = (flags & (1 << 0)) != 0;
    newState.doorClosed  = (flags & (1 << 1)) != 0;
    newState.alarmActive = (flags & (1 << 2)) != 0;

    newState.alarmCode   = alarm;
    newState.uptimeMs    = uptime;

    m_state = newState;
    emit machineStateChanged();

    qDebug() << "Frame OK:"
             << "T_in" << m_state.tempInlet
             << "T_out" << m_state.tempOutlet
             << "P" << m_state.pressureBar
             << "RPM" << m_state.motorRpm
             << "EStop" << m_state.estopActive
             << "Door" << m_state.doorClosed
             << "Alarm" << m_state.alarmActive
             << "Code" << m_state.alarmCode
             << "Uptime" << m_state.uptimeMs;

    return true;
}

void SerialManger::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError)
        return;

    emit errorOccurred(tr("Serial error: %1").arg(m_port.errorString()));
    qWarning() << "Serial error:" << m_port.errorString();
}
