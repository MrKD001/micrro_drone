# 🚁 ESP32 Drone Flight Controller

This is an experimental DIY drone flight controller powered by an **ESP32**, designed to control **coreless motors** using real-time sensor data from the **MPU6050 IMU**. The system implements a basic **Kalman Filter**, **PID control**, and a **Wi-Fi based control interface** for manual flight via mobile app or joystick.

## 🧠 Features

- ESP32 dual-core microcontroller
- 6-axis motion tracking with MPU6050
- Kalman filter for pitch/roll estimation
- PID controller for stable flight
- PWM control for coreless motors
- Wi-Fi AP mode for app-based control
- Real-time control loop with performance tuning
- Basic task scheduling (RTOS-ready)

## 🛠️ Hardware Used

- ESP32-WROOM-32 Dev Board  
- MPU6050 (Gyroscope + Accelerometer)  
- 4x Coreless Motors (8520)  
- Motor driver (e.g., MOSFET-based or driver IC like DRV8833)  
- Li-Po Battery (1S/2S, 3.7V - 7.4V)  
- Propellers (65mm for 8520 motors)  
- Lightweight frame or custom 3D printed chassis  

## 📦 Project Structure

```plaintext
├── src/
│   ├── main.cpp             # Core logic: sensor read, Kalman, PID, motor control
│   ├── imu.cpp              # MPU6050 interface
│   ├── kalman.cpp           # Kalman filter implementation
│   ├── pid.cpp              # PID controller logic
│   └── motor.cpp            # PWM output for motor control
├── lib/
│   └── WiFiServer           # WiFi or BLE communication module
├── data/
│   └── PID_Tuning.csv       # Experimental tuning logs
└── README.md
