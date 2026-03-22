Here’s a clean, professional **GitHub README.md** for your drone project using your wiring diagram and supporting images 👇

---

# 🛰 Drone Development Package

An advanced **custom drone development system** built using Raspberry Pi, Pico, sensors, and real-time communication modules for intelligent flight control and monitoring.

> ⚡ Includes full hardware integration, sensor fusion, and software pipeline
> 📜 **Note:** This project is backed by a **patent**, showcasing its innovation and uniqueness

---

## 📸 Project Overview

### 🔌 Wiring Diagram

![Drone Wiring Diagram]<img width="3884" height="1968" alt="V4_Wiring_Diagram" src="https://github.com/user-attachments/assets/0611c46f-88ae-44c7-841f-89b684077e93" />

---

## 🧠 System Architecture

* 🖥️ **Raspberry Pi Zero 2W** – High-level processing & communication
* ⚙️ **Raspberry Pi Pico** – Flight controller
* 🧭 **MPU6050** – Gyroscope + Accelerometer
* 🧲 **QMC5883** – Magnetometer (Compass)
* 📡 **GPS Module** – Location tracking
* ⚡ **ESCs (4x)** – Motor control
* 🚀 **Brushless Motors** – Drone propulsion

---

## 🧩 Sensor Orientation Guide

![Sensor Orientation]<img width="950" height="572" alt="Sensors Placement Orientation Guide" src="https://github.com/user-attachments/assets/a3b7c147-8843-482a-b43a-e5c03e9b7a55" />


📌 Proper orientation is critical for accurate flight stabilization.

* X-axis → Right
* Y-axis → Forward
* Align sensors with **front direction of drone**

(Refer to guide: )

---

## 🛠️ Hardware Setup

### 🔧 Key Components

* Raspberry Pi Pico
* Raspberry Pi Zero 2W
* MPU6050
* QMC5883
* GPS Module
* ESC (4x)
* Brushless Motors (2212 920KV)
* LiPo Battery (3S 11.1V)
* Drone Frame (F450)

---

## 🔌 Wiring Summary

* ESC signals → Connected to Pico GPIO pins
* Sensors (MPU6050 + QMC5883) → I2C (SDA, SCL)
* GPS → UART (TX, RX)
* Power → Shared GND with regulated 5V supply

---

## 💻 Software Setup

### 🖥️ Raspberry Pi Zero Setup

Follow these steps (summarized):

1. Install **Raspberry Pi OS Lite (32-bit)**
2. Enable:

   * SSH
   * UART
   * Camera
3. Configure:

   * `config.txt → dtoverlay=dwc2`
   * `cmdline.txt → modules-load=dwc2,g_ether`
4. Connect via SSH (`raspberrypi.local`)
5. Install dependencies:

   ```bash
   sudo apt-get update
   sudo apt-get upgrade
   sudo apt-get install cmake
   ```

📄 Full detailed steps available here: 

---

## ⚙️ Flight Controller Programming

* Flash `.uf2` firmware to Raspberry Pi Pico
* Connect via BOOTSEL mode
* Drag & drop firmware

---

## 📡 Communication System

* Drone connects via **mobile hotspot**
* Uses IP-based communication
* Supports:

  * 📷 Live video streaming
  * 📊 Telemetry data

---

## 📱 Mobile App Integration

* Install control app (V4 APK)
* Enter Raspberry Pi IP
* Connect and control drone
* View real-time video

---

## 🚀 Features

* 🔄 Real-time flight control
* 📡 Live video streaming
* 🧠 Sensor fusion (IMU + Compass)
* 📍 GPS tracking
* 🔗 IoT-based communication
* ⚡ Scalable architecture

---

## 🧪 Real Hardware Implementation

![Drone Hardware]

https://github.com/user-attachments/assets/6d194287-a56f-4777-ab25-8e48ed73afbc



---

## 📌 How to Run

```bash
# Start communication modules
./tcp_uart_c &
./yct &
```

---

## 📄 License

This project is protected and associated with a **patent**.
Usage, replication, or modification may require permission.

