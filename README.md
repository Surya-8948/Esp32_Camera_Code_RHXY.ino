# 📷 ESP32-CAM RHYX M21-45 Control Center

<p align="center">
  <img src="https://img.shields.io/badge/ESP32-CAM-blue?style=for-the-badge">
  <img src="https://img.shields.io/badge/Arduino_Core-2.x_|_3.x-success?style=for-the-badge">
  <img src="https://img.shields.io/badge/Single_File-Firmware-orange?style=for-the-badge">
  <img src="https://img.shields.io/badge/License-MIT-red?style=for-the-badge">
</p>

> A production-ready **single-file ESP32-CAM firmware** featuring an offline web dashboard, AP + STA mode, live MJPEG streaming, OTA updates, software JPEG fallback for RHYX M21-45 (GC2145), REST API, and complete camera controls.

---

## ✨ Features

- 📷 Live MJPEG video streaming
- 📸 One-click image capture
- 🌐 Built-in Offline Web Dashboard (HTML/CSS/JS)
- 📡 AP + STA Wi-Fi Mode
- 🔄 Automatic Wi-Fi Reconnection
- 💾 Preferences stored in NVS
- 💡 Flash LED Brightness Control
- ⚙ Complete Camera Settings
- 🔐 Optional HTTP Authentication
- 📲 OTA Firmware Update
- 📊 Live System Monitoring
- 🌍 mDNS Support (`http://suryacam.local`)
- 🚀 Single `.ino` File
- ✅ Compatible with Arduino ESP32 Core 2.x & 3.x

---

# 📸 Supported Camera Modules

| Camera | Status |
|---------|--------|
| OV2640 | ✅ Supported |
| OV3660 | ✅ Supported |
| OV5640 | ✅ Supported |
| RHYX M21-45 (GC2145) | ✅ Supported |
| Other ESP32 Camera Sensors | ⚠ Depends on Driver |

---

# 🔥 RHYX M21-45 Automatic Fallback

Unlike the stock CameraWebServer example, this firmware automatically detects the RHYX M21-45 camera.

If hardware JPEG is unavailable, it automatically switches to:

```
RGB565
      ↓
Software JPEG Encoding
      ↓
Live Stream & Snapshot
```

No code modifications are required.

---

# 🚀 Features Overview

- AP + STA Mode
- Live Video Streaming
- Snapshot Download
- Flash LED Control
- PWM Brightness Control
- Camera Resolution Selection
- JPEG Quality Adjustment
- FPS Control
- Brightness
- Contrast
- Saturation
- White Balance
- Exposure
- Gain Control
- Mirror
- Flip
- Color Effects
- Auto Flash
- Camera Reset
- OTA Update
- Factory Reset
- System Information
- Network Diagnostics

---

# 🖥 Dashboard

The firmware hosts its own modern dashboard.

### Camera

- Live Stream
- Snapshot
- Flash
- Auto Flash
- Resolution
- Quality
- FPS

### Network

- Wi-Fi Status
- STA/AP Information
- RSSI
- Connected Clients
- IP Address
- Reconnect
- Forget Network

### System

- Firmware Version
- Sensor Name
- Chip Information
- Heap Usage
- PSRAM Usage
- Uptime
- Stream URL
- Restart
- Factory Reset

---

# 📦 Hardware Required

- ESP32-CAM AI Thinker
- RHYX M21-45 Camera Module
- USB to TTL Programmer
- 5V / 2A Power Supply
- Jumper Wires

---

# ⚙ Arduino IDE Settings

| Setting | Value |
|----------|------|
| Board | AI Thinker ESP32-CAM |
| Flash Mode | QIO |
| PSRAM | Enabled |
| Partition Scheme | Huge APP |
| Upload Speed | 115200 |
| Core Version | 2.x or 3.x |

---

# 🚀 Installation

### 1. Install ESP32 Board Package

Install **Espressif ESP32** board package from Arduino Boards Manager.

---

### 2. Create New Sketch

Delete the default code.

---

### 3. Paste Firmware

Paste the complete firmware into the sketch.

---

### 4. Select Board

```
AI Thinker ESP32-CAM
```

---

### 5. Upload

Connect GPIO0 to GND.

Press RESET.

Upload.

Remove GPIO0 from GND.

Press RESET again.

---

# 📶 Default Access Point

```
SSID : SuryaCAM-ESP32

Password :
surya12345
```

Open

```
http://192.168.4.1
```

---

# 🌐 REST API

| Endpoint | Method | Description |
|----------|---------|------------|
| `/` | GET | Dashboard |
| `/status` | GET | System Status |
| `/capture` | GET | Capture Image |
| `/control` | GET | Camera Settings |
| `/config` | POST | Save Wi-Fi |
| `/reboot` | GET | Restart Device |
| `/factory` | GET | Factory Reset |
| `:81/stream` | GET | MJPEG Stream |
| `:81/frame` | GET | Single JPEG |

---

# 📈 Performance

| Resolution | FPS |
|------------|-----|
| QVGA | 12–18 FPS |
| VGA | 6–10 FPS |
| SVGA | 4–6 FPS |
| UXGA Snapshot | 1–2 sec/frame |

---

# 🛠 Troubleshooting

### Camera Init Failed (0x106)

The firmware automatically switches to Software JPEG mode.

---

### Wi-Fi Not Connecting

- Verify SSID & Password
- Use a 2.4GHz Wi-Fi Network
- Disable WPA3-only Mode
- Ensure Stable Power Supply

---

### Brownout / Random Restart

Use:

- 5V / 2A Supply
- Short USB Cable
- Enable PSRAM

---

### Stream Not Opening

Open:

```
http://BOARD_IP:81/stream
```

---

# 📁 Project Structure

```text
ESP32_CAM_RHYX_M21_45/
│
├── ESP32_CAM_RHYX_M21_45_Surya.ino
├── README.md
├── LICENSE
├── assets/
│   ├── dashboard.png
│   ├── stream.gif
│   └── wiring.png
└── screenshots/
```

---

# 📷 Preview

You can add your screenshots here.

```
assets/dashboard.png
assets/stream.gif
assets/network.png
assets/system.png
```

---

# 🤝 Contributing

Contributions, bug reports, feature requests, and pull requests are always welcome.

If you find this project useful, please consider giving it a ⭐ on GitHub.

---

# 👨‍💻 Author

**Surya Bajpai**

Electronics Engineer • Embedded Systems • IoT • Robotics • PCB Design



GitHub: **https://github.com/Surya-8948**

---

# 📄 License

This project is licensed under the **MIT License**.

---

## ⭐ Support

If this project helped you:

⭐ Star the repository

🍴 Fork the project

💬 Share it with others

Happy Coding! 🚀
