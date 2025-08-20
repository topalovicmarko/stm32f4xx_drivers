# Bare-Metal STM32 Driver Development (Embedded C)

## Overview
This repository contains bare-metal drivers for STM32F4xx microcontrollers written in Embedded C. The drivers provide low-level access to peripherals without relying on HAL or other abstraction layers.

## Features
- **GPIO Driver**: Pin configuration, digital I/O, interrupt handling
- **Register-level access**: Direct hardware control
- **Minimal dependencies**: No external libraries required

## Project Structure

```text
stm32f4xx_drivers/
│
├── 📁 drivers/                  # Peripheral driver implementations
│   │
│   ├── 📁 inc/                  # Header files (.h)
│   │   ├── 🔴 stm32f401xx_gpio_driver.h
│   │   ├── 🔴 stm32f401xx.h           # MCU header file
│   │   ├── 🔴 stm32f401xx_i2c_driver.h
│   │   └── 🔴 stm32f401xx_spi_driver.h
│   │
│   └── 📁 src/                  # Source implementations (.c)
│       ├── 🟣 stm32f401xx_gpio_driver.c
│       ├── 🟣 stm32f401xx_i2c_driver.c
│       └── 🟣 stm32f401xx_spi_driver.c
│
├── 📁 examples/                 # Example applications
│   ├── 001_led_toggle.c             # Basic GPIO output
│   ├── 002_led_button.c             # GPIO input/output
│   ├── 003_led_button_ext.c         # External button
│   ├── 004_button_interrupt.c       # Interrupt example
│   ├── 005_spi_tx_testing.c         # SPI transmit test
│   ├── 006_spi_txonly_arduino.c     # SPI TX only to Arduino
│   ├── 007_spi_cmd_handling.c       # SPI command handling
│   └── 008_spi_message_rcv_it.c     # SPI receive via interrupt
│
└── 📄 README.md                 # Project documentation
```

## Getting Started

### Prerequisites
- ARM GCC Toolchain (arm-none-eabi)
- STM32 reference manual (RM0368 for STM32F401)
- ST-Link or compatible programmer
- Make or CMake build system
