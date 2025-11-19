
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
