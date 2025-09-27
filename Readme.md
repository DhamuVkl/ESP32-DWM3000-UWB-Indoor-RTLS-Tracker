# Real-Time Indoor Position Tracking System

> Ultra-precise indoor positioning using ESP32 & Qorvo DWM3000 UWB modules with centimeter-level accuracy

[![Hardware](https://img.shields.io/badge/Hardware-ESP32%20%2B%20DWM3000-blue)](https://github.com)

Article Link: [How to Build a Real-Time Indoor Position Tracking System using ESP32 & Qorvo DWM3000 UWB Modules](https://circuitdigest.com/)

## 🚀 Overview

This project implements a Real-Time Location System (RTLS) using Ultra-Wideband (UWB) technology to achieve **sub-10cm positioning accuracy** indoors. Unlike GPS, Wi-Fi, or Bluetooth-based solutions that struggle with indoor precision, this UWB system provides reliable, fast, and highly accurate position tracking even in challenging indoor environments.

Perfect for applications like:
- 🏭 **Warehouse automation** and robot navigation
- 🏪 **Asset tracking** in industrial facilities  
- ✈️ **Indoor navigation** systems (airports, malls)
- 🥽 **AR/VR** applications requiring precise spatial tracking
- 🏥 **Healthcare** equipment and personnel tracking

## ✨ Key Features

- **🎯 High Precision**: 10cm accuracy using Double-Sided Two-Way Ranging (DS-TWR)
- **⚡ Real-Time**: Live position updates with minimal latency
- **🛡️ Multipath Resistant**: UWB technology filters out wall reflections
- **📊 Live Visualization**: Real-time Python plotting with signal strength monitoring
- **🔧 Easy Setup**: Plug-and-play hardware with detailed documentation
- **📡 Wireless**: Tag communicates via Wi-Fi to central processing unit
- **🔒 Secure**: IEEE 802.15.4z compliance with anti-spoofing features

## 🛠️ Hardware Requirements

| Component | Quantity | Purpose |
|-----------|----------|---------|
| **Qorvo DWM3000 UWB Module** | 4 | 1 Tag + 3 Anchors for trilateration |
| **ESP32-WROOM Development Board** | 4 | SPI communication and processing |
| **Micro-USB Cables** | 4 | Programming and power |
| **Breadboards/PCBs** | 4 | Module mounting and connections |
| **5V USB Power Sources** | 4 | Powering ESP32 modules |

## 📋 Technical Specifications

### DWM3000 Module Capabilities
- **Operating Bands**: Channel 5 (6.5 GHz) and Channel 9 (8 GHz)
- **Data Rates**: 850 kbps and 6.8 Mbps
- **Standards**: IEEE 802.15.4z / FiRa Consortium compliant
- **Security**: Scrambled Timestamp Sequences (STS) for anti-spoofing
- **Interface**: High-speed SPI (up to 38 MHz)
- **Form Factor**: Compact 24 × 16 mm with integrated antenna

### System Performance
- **Position Accuracy**: ±10 cm or better
- **Update Rate**: Real-time (depends on ranging cycle configuration)
- **Range**: Optimized for indoor environments
- **Multi-tag Support**: Expandable architecture

## 🔌 Pin Connections

Each ESP32-DWM3000 pair uses the following SPI connections:

| DWM3000 Pin | ESP32 Pin | Function |
|-------------|-----------|----------|
| VCC | 3.3V | ⚠️ **Power Supply (3.3V ONLY)** |
| GND | GND | Ground |
| SCK | GPIO18 | SPI Clock |
| MOSI | GPIO23 | SPI Data (ESP32 → DWM3000) |
| MISO | GPIO19 | SPI Data (DWM3000 → ESP32) |
| CS | GPIO4 | SPI Chip Select |
| RST | GPIO27 | Hardware Reset |
| IRQ | GPIO34 | Interrupt (optional) |

> ⚠️ **Warning**: The DWM3000 operates at 3.3V only. Higher voltages will damage the module!

## 📐 Room Setup

Position three anchors at known coordinates for optimal trilateration:

```
Anchor Layout Example:
A3 (165, 625) ────────●────────── Room boundary
              │                │
              │                │
              │        TAG     │
              │         ●      │
              │                │
A1 (15, 5) ●──┼────────────────┼──● A2 (290, 5)
```

Update the `ANCHOR_POSITIONS` array in your Python script to match your room layout.

## 🚀 Quick Start

### 1. Hardware Assembly
1. Connect each DWM3000 module to an ESP32 using the pin mapping above
2. Ensure all connections are secure and power is 3.3V
3. Position anchors at known coordinates in your tracking area

### 2. Firmware Installation
```bash
# Clone the repository
https://github.com/Circuit-Digest/ESP32-DWM3000-UWB-Indoor-RTLS-Tracker.git
cd ESP32-DWM3000-UWB-Indoor-RTLS-Tracker

# Flash anchor firmware to 3 ESP32s (set unique ANCHOR_ID for each)
# Upload tag firmware to 1 ESP32
# Configure Wi-Fi credentials in both firmwares
```

### 3. Python Environment Setup
```bash
# Install required packages
pip install numpy scipy matplotlib json socket

# Run the visualization script
python floor_view.py
```

### 4. System Operation
1. Power on all anchor nodes first
2. Power on the tag node
3. The tag will automatically begin ranging with anchors
4. Position data appears in real-time on your Python visualization

## 📊 Data Format

The system transmits position data via JSON over Wi-Fi:

```json
{
        "tag_id": 10,
        "A1": {
            "distance": 144.51,
            "raw": 146.38,
            "rssi": -79.63,
            "fp_rssi": -85.71,
            "round_time": 64870950,
            "reply_time": 78282714,
            "clock_offset": -0.000004
        },
        "A2": {
            "distance": 137.00,
            "raw": 137.47,
            "rssi": -71.19,
            "fp_rssi": -75.68,
            "round_time": 69129089,
            "reply_time": 72260735,
            "clock_offset": -0.000001
        },
        "A3": {
            "distance": 134.89,
            "raw": 138.88,
            "rssi": -74.64,
            "fp_rssi": -82.12,
            "round_time": 71333162,
            "reply_time": 76495574,
            "clock_offset": -0.000001
        }
```

## 🔬 How It Works

### Ultra-Wideband Technology
UWB measures the time-of-flight of radio pulses between devices with nanosecond precision. The DWM3000 uses **Double-Sided Two-Way Ranging (DS-TWR)** to eliminate clock synchronization errors:

1. **Poll**: Tag sends poll message to anchor
2. **Response**: Anchor replies after known delay
3. **Final**: Tag sends final message with timing data
4. **Calculation**: Both devices compute precise distance

### Trilateration Mathematics
With distances to three anchors, the system solves:

```
(x - x₁)² + (y - y₁)² = d₁²
(x - x₂)² + (y - y₂)² = d₂²
(x - x₃)² + (y - y₃)² = d₃²
```

Using least-squares optimization to handle measurement noise and find the best-fit position.

### Multipath Mitigation
UWB's short pulse duration and wide frequency spectrum make it highly resistant to multipath interference from walls and obstacles, maintaining accuracy in complex indoor environments.


## 🔧 Configuration

### Firmware Settings
- **Channel**: UWB Channel 5 (6.5 GHz) - globally available
- **Data Rate**: 6.8 Mbps for faster ranging cycles
- **Antenna Delay**: Calibrated for each module (see calibration guide)
- **Wi-Fi**: Configure SSID/password for your network

### Python Visualization
- **Room Dimensions**: Update in `floor_view.py`
- **Anchor Positions**: Match your physical setup
- **Update Rate**: Adjustable refresh rate
- **RSSI Monitoring**: Live signal strength display

## 🎯 Performance Tips

1. **Antenna Calibration**: Perform antenna delay calibration for best accuracy
2. **Line-of-Sight**: Maintain clear paths between tag and anchors when possible
3. **Anchor Placement**: Position anchors to avoid geometric dilution of precision
4. **Environment**: UWB performs best in spaces without excessive metal reflections
5. **Power Supply**: Use clean, stable power sources to minimize noise

## 🐛 Troubleshooting

### Common Issues
- **No ranging data**: Check SPI connections and power supply
- **Poor accuracy**: Verify antenna delay calibration
- **Intermittent connection**: Ensure stable Wi-Fi and power
- **High noise**: Check for electrical interference sources


## 🔮 Future Enhancements

- [ ] **Multi-tag Support**: Track multiple objects simultaneously
- [ ] **3D Positioning**: Add vertical dimension with 4+ anchors
- [ ] **Mobile App**: Smartphone interface for configuration
- [ ] **Cloud Integration**: Remote monitoring and data storage
- [ ] **Machine Learning**: Predictive tracking and anomaly detection
- [ ] **Mesh Networking**: Extended coverage with anchor relaying

## 🤝 Contributing

Contributions are welcome! Please read our contributing guidelines:

---

**⭐ Star this project if you find it useful!**

*Built with ❤️ for the maker and robotics community*
