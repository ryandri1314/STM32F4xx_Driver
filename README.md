# STM32F4 Low-Level Driver Development

> Bare-metal peripheral driver development for STM32F4 — register-level understanding and implementation.

## 🧠 Project Overview

This project is a hands-on exploration of writing low-level drivers for the STM32F4 microcontroller family. The drivers are implemented **without using HAL or CMSIS high-level libraries**, directly manipulating peripheral registers based on the STM32 reference manual.

## 🔧 Implemented Drivers

| Module | Description |
|--------|-------------|
| GPIO   | Configure pins as input/output, set pull-up/pull-down, handle external interrupts |
| SPI    | Master/slave setup, data transfer, clock phase and polarity control |
| I2C    | Bus initialization, address transmission, state-based communication |
| UART   | Baud rate config, transmit/receive using polling method |
| RCC    | System and peripheral clock configuration, enable/disable modules |

## 📁 Project Structure

- `Src/`: Contains all `*.c` files used for testing driver functionality (e.g., GPIO, SPI, I2C, UART tests).
- `drivers/`: Contains the custom low-level driver implementation.
  - `Inc/`: Header files (`*.h`) with macro definitions, register mappings, and function declarations.
  - `Src/`: Source files (`*.c`) that implement the driver APIs and handle direct register-level interactions.

## 📈 Development Process

1. **Read the STM32F4 Reference Manual** to understand peripheral behavior and register maps.
2. **Define and access peripheral registers** using memory-mapped addresses.
3. **Write modular drivers** in C with clear interface functions.
4. **Test each driver** with real hardware and a logic analyzer (24MHz, 8 channels).

## 🧪 Testing Environment

- **Board:** STM32F4 (e.g., STM32F401, STM32F411)
- **Analyzer:** 8-channel 24MHz logic analyzer for signal verification
- **Test files** for each module to verify register-level functionality

## 📚 Reference documents
- STM32F4 Reference Manual (RM0090)
