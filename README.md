# UWB Ranging Tag

A UWB-based ranging node built on a custom STM32L4 board, developed as part of a Master's thesis project. The system uses a DW3000 UWB transceiver for precise distance measurement and an LSM6DSV IMU for sensor fusion.

***

## Hardware

| Component | Part | Role |
|-----------|------|------|
| MCU | STM32L4 | Main controller |
| UWB Transceiver | Qorvo DW3000 (DWM3000 module) | UWB ranging |
| IMU | ST LSM6DSV | Accelerometer + gyroscope, sensor fusion |
| Board | Custom PCB | — |

***

## Software Stack

- **RTOS:** FreeRTOS
- **Build system:** CMake
- **Driver:** Qorvo DW3xxx Device Driver API v4.x
- **Language:** C (C11)

***

## Project Structure

```
├── CMakeLists.txt
├── Core/
│   ├── Inc/
│   └── Src/
│       └── main.c
├── Drivers/
│   ├── DW3XXX/          # Qorvo DW3000 device driver
│   └── LSM6DSV/         # ST IMU driver
├── App/
│   ├── dwm3000.c/.h     # DW3000 init, self-test, ranging
│   └── imu.c/.h         # LSM6DSV sensor fusion
├── .gitignore
└── README.md
```

