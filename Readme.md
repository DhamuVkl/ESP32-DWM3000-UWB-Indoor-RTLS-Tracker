# 🚀 ESP32 UWB Indoor Tracker

A real-time **Ultra-Wideband (UWB) indoor positioning system** using **ESP32-WROOM** boards and **Qorvo DWM3000** modules.  
This project performs **two-way ranging** between a moving tag and three fixed anchors, calculates the position via **trilateration**, and visualizes it live on a **Python-based floor plan UI**.

---

## 📌 Features
- 📍 **Real-time indoor tracking** with centimeter-level accuracy  
- 📡 **Double-Sided Two-Way Ranging (DS-TWR)** protocol using Qorvo DWM3000  
- 🌐 Wi-Fi communication between Tag and PC server  
- 🖥 **Python GUI visualization** with floor plan and path tracking  
- 📶 Live RSSI (signal strength) display for each anchor  
- 🔄 Median filtering for smoother distance measurements  

---

## 🛠 Components Required
| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32-WROOM Board | 4 | 1 for Tag, 3 for Anchors |
| Qorvo DWM3000 UWB Module | 4 | Each paired with an ESP32 |
| USB Cables | 4 | For programming & power |
| Breadboard / PCB | As needed | For wiring stability |
| Power supply | Optional | Anchors can be USB powered |
| Wi-Fi network | 1 | Common network for all devices |
| PC / Laptop | 1 | Runs Python visualization script |

---

## 🔌 Hardware Setup
- **Each Anchor** has its own ESP32-WROOM + DWM3000 connected via SPI.
- **Tag** is mobile and also uses ESP32-WROOM + DWM3000.
- All devices connect to the **same Wi-Fi network**.
- SPI Pin Mapping (for ESP32-WROOM + DWM3000):
