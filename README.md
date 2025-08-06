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
│   │   ├── 🟣 stm32f401xx_gpio_driver.h
│   │   └── 🔴 stm32f401xx.h     # MCU header file
│   │
│   └── 📁 src/                  # Source implementations (.c)
│       └── 🟣 stm32f401xx_gpio_driver.c
│
├── 📁 src/                      # Example applications
│   ├── 1️⃣ 001_led_toggle/       # Basic GPIO output
│   ├── 2️⃣ 002_led_button/        # GPIO input/output
│   ├── 3️⃣ 003_led_button_ext/    # External interrupt
│   └── 4️⃣ 004_button_interrupt/  # NVIC interrupt handling
│
└── 📄 README.md                 # Project documentation

## Getting Started

### Prerequisites
- ARM GCC Toolchain (arm-none-eabi)
- STM32 reference manual (RM0368 for STM32F401)
- ST-Link or compatible programmer
- Make or CMake build system
