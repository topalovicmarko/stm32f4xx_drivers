# Bare-Metal STM32 Driver Development (Embedded C)

## Overview
This repository contains bare-metal drivers for STM32F4xx microcontrollers written in Embedded C. The drivers provide low-level access to peripherals without relying on HAL or other abstraction layers.

## Features
- **GPIO Driver**: Pin configuration, digital I/O, interrupt handling
- **I2C Driver**: Master/slave modes, interrupt/DMA support  
- **SPI Driver**: Full-duplex communication, various modes
- **USART Driver**: Asynchronous/synchronous communication
- **Register-level access**: Direct hardware control
- **Minimal dependencies**: No external libraries required

## Project Structure

stm32f4xx_drivers/
│
├── 📂 drivers/ # Peripheral driver implementations
│ ├── 📂 inc/ # Header files (.h)
│ │ ├── stm32f401xx_gpio_driver.h
│ │ ├── stm32f401xx_i2c_driver.h
│ │ ├── stm32f401xx_spi_driver.h
│ │ ├── stm32f401xx_usart_driver.h
│ │ └── stm32f401xx.h # MCU header
│ │
│ └── 📂 src/ # Source files (.c)
│ ├── stm32f401xx_gpio_driver.c
│ ├── stm32f401xx_i2c_driver.c
│ ├── stm32f401xx_spi_driver.c
│ └── stm32f401xx_usart_driver.c
│
├── 📂 src/ # Example applications
│ ├── 001_led_toggle/ # Basic GPIO output
│ ├── 002_led_button/ # GPIO input/output
│ ├── 003_led_button_ext/ # External interrupt
│ └── 004_button_interrupt/ # NVIC interrupt

## Getting Started

### Prerequisites
- ARM GCC Toolchain (arm-none-eabi)
- STM32 reference manual (RM0368 for STM32F401)
- ST-Link or compatible programmer
- Make or CMake build system
