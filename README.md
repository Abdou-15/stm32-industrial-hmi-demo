
# Industrial HMI Demo – STM32F429 + STM32F407 + Qt/QML

This repository contains a complete industrial-style **machine simulation + gateway + HMI system**, designed to demonstrate embedded communication, binary protocols, Qt/QML visualization, and multi-MCU data handling.

It consists of **three coordinated projects**:

1. **STM32F429I-DISC1** — Machine Simulator  
2. **STM32F407G-DISC1** — Gateway + Frame Parser + USB CDC  
3. **Qt/QML Desktop App** — Industrial HMI Dashboard

Together they form a real-world embedded-to-desktop industrial data pipeline.

---

## 🚀 Overview

### 🟦 STM32F429I-DISC1 — Machine Simulation (`machine_f429/`)
- Generates synthetic process values:
  - Inlet/Outlet temperature  
  - Pressure  
  - Motor RPM  
  - Status flags (door, E-stop, alarm)  
- Packs them into a **19-byte binary frame**  
- Sends frames continuously to the F407 via UART (115200 baud)  
- Acts as the “machine” or “equipment” in this demo

### 🟩 STM32F407G-DISC1 — Gateway + Protocol Bridge (`master_f409/`)
- Receives frames from F429 (UART2)
- Validates:
  - Start byte  
  - Length  
  - Version  
  - Checksum  
- Unpacks fields into a `MachineState_t`
- Drives LEDs to visualize door/E-stop/alarm flags
- Forwards the **raw 19-byte binary frame** to the PC over USB CDC
- Creates a virtual COM port that the Qt app can open

### 🟧 Qt/QML HMI — Desktop Dashboard (`pc_hmi_qt/`)
- Cross-platform Qt 6 application (macOS/Windows/Linux)
- Reads binary frames using `QSerialPort`
- Reconstructs and decodes the 19-byte machine frame
- Exposes data to QML using `Q_PROPERTY`
- Displays:
  - Temperatures  
  - Pressure  
  - Motor RPM  
  - Flags (Door, E-Stop, Alarm)  
  - Alarm code  
  - Uptime  
- Clean, modern UI using Qt Quick
- Designed as an **industrial-style dashboard**

---

## 🧩 Binary Frame Format (19 bytes)

This protocol is jointly used by F429, F407, and the Qt HMI.
+--------------------------------------------------------+
| Byte | Description |
+--------------------------------------------------------+
| 0 | STX (Start byte) |
| 1 | LEN (payload length = 16) |
| 2 | VERSION (protocol version) |
| 3-4 | temp_in_q1 (int16, 0.1°C units) |
| 5-6 | temp_out_q1 (int16, 0.1°C units) |
| 7-8 | press_q2 (uint16, 0.01 bar units) |
| 9-10 | rpm_u16 (uint16, RPM) |
| 11 | flags (bit0 = estop, bit1 = door, bit2 = alarm) |
| 12-13| alarmCode (uint16) |
| 14-17| uptime_ms (uint32) |
| 18 | checksum (sum of bytes 0–17, low 8 bits) |
+--------------------------------------------------------+
Total size = 19 bytes

---

## 📡 Data Flow Architecture

[F429 Machine Sim] --> UART --> [F407 Gateway] --> USB CDC -->
  --> Virtual COM Port --> [Qt/QML Desktop HMI]

F429 generates real-time measurements
F407 validates & forwards binary frames
Qt HMI parses and displays live machine data

---

## 🛠 Build & Flash Instructions

1. STM32F429 (machine_f429/)
Open project in STM32CubeIDE
Configure correct UART pins
Build & flash to the F429 board
The board starts sending binary frames automatically
2. STM32F407 (master_f409/)
Open project in STM32CubeIDE
Ensure UART pins match F429 TX/RX
Build & flash
Board appears as USB CDC serial port
macOS: /dev/cu.usbmodemXXXX
Windows: COMx
3. Qt/QML Desktop HMI (pc_hmi_qt/)
Requires Qt 6.6+ (Qt Quick + SerialPort)
Build using CMake:
cd pc_hmi_qt
mkdir build && cd build
cmake ..
cmake --build .
Run the app → click Connect → select your USB serial port.
Live values will update in real time.

---

## 📊 HMI Features (Qt/QML)

Real-time dashboard:
Temperature inlet/outlet
Pressure
Motor RPM
Door status
E-stop
Alarm
Uptime
Smooth UI updates via QML bindings
Connection control (Connect/Disconnect)
Designed to be extended with:
Trends
Alarm list
Maintenance page
Predictive health indicators

---

## 📁 Repository Structure

Industrial_HMI_Demo/
├─ machine_f429/      # STM32F429: machine simulator (UART)
├─ master_f409/       # STM32F407: gateway (UART->USB)
├─ pc_hmi_qt/         # Qt/QML dashboard
├─ .gitignore
└─ README.md
Each subfolder contains its own project source files.

---

## 📄 License

This project is provided under the MIT License.
Feel free to modify and use it for learning, research, or integration.
