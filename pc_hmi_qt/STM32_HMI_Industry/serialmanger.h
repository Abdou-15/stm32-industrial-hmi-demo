#ifndef SERIALMANGER_H
#define SERIALMANGER_H

#pragma once

#include <QObject>
#include <QSerialPort>
#include <QByteArray>

class SerialManger : public QObject
{
    Q_OBJECT

    // Connection state
    Q_PROPERTY(bool connected READ isConnected NOTIFY connectedChanged)

    // Machine state properties exposed to QML
    Q_PROPERTY(float tempInlet   READ tempInlet   NOTIFY machineStateChanged)
    Q_PROPERTY(float tempOutlet  READ tempOutlet  NOTIFY machineStateChanged)
    Q_PROPERTY(float pressureBar READ pressureBar NOTIFY machineStateChanged)
    Q_PROPERTY(float motorRpm    READ motorRpm    NOTIFY machineStateChanged)

    Q_PROPERTY(bool estopActive  READ estopActive  NOTIFY machineStateChanged)
    Q_PROPERTY(bool doorClosed   READ doorClosed   NOTIFY machineStateChanged)
    Q_PROPERTY(bool alarmActive  READ alarmActive  NOTIFY machineStateChanged)

    Q_PROPERTY(quint16 alarmCode READ alarmCode NOTIFY machineStateChanged)
    Q_PROPERTY(quint32 uptimeMs  READ uptimeMs  NOTIFY machineStateChanged)

public:
    explicit SerialManger(QObject *parent = nullptr);

    // Invokable from QML
    Q_INVOKABLE void openPort(const QString &portName);
    Q_INVOKABLE void closePort();

    bool isConnected() const;

    // Getters used by Q_PROPERTY
    float  tempInlet()   const { return m_state.tempInlet; }
    float  tempOutlet()  const { return m_state.tempOutlet; }
    float  pressureBar() const { return m_state.pressureBar; }
    float  motorRpm()    const { return m_state.motorRpm; }

    bool   estopActive() const { return m_state.estopActive; }
    bool   doorClosed()  const { return m_state.doorClosed; }
    bool   alarmActive() const { return m_state.alarmActive; }

    quint16 alarmCode()  const { return m_state.alarmCode; }
    quint32 uptimeMs()   const { return m_state.uptimeMs; }

signals:
    void connectedChanged();
    void machineStateChanged();
    void errorOccurred(const QString &message);

private slots:
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

private:
    // Same logical content as MachineState_t on STM32
    struct MachineState {
        float  tempInlet   = 0.0f;
        float  tempOutlet  = 0.0f;
        float  pressureBar = 0.0f;
        float  motorRpm    = 0.0f;

        bool   estopActive = false;
        bool   doorClosed  = false;
        bool   alarmActive = false;

        quint16 alarmCode  = 0;
        quint32 uptimeMs   = 0;
    };

    bool parseFrame(const QByteArray &frame);

    QSerialPort m_port;
    QByteArray  m_rxBuffer;
    MachineState m_state;
};

#endif // SERIALMANGER_H
