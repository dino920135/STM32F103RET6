# STM32F103RET6 UART Communication

A simple UART communication project for the STM32F103RET6 microcontroller. This project demonstrates how to send an incrementing counter value through USART1.

## Hardware Requirements

- STM32F103RET6 development board
- USB-TTL converter (for receiving UART data)
- Connection cables

## Pin Configuration

- PA9 - USART1_TX
- GND - Ground

## Communication Parameters

- Baud Rate: 115200
- Word Length: 8 bits
- Stop Bits: 1
- Parity: None
- Mode: Transmit only

## Project Setup

1. Clone this repository
2. Open STM32CubeMX and load the `STM32F103RET6.ioc` file
3. Generate the code to get required drivers
4. Open the project in Keil MDK-ARM (or your preferred IDE)
5. Build and flash to your device

## Connection Setup

1. Connect PA9 (USART1_TX) to your USB-TTL converter's RX pin
2. Connect GND to your USB-TTL converter's GND pin
3. Open a serial terminal on your computer with the following settings:
   - Baud Rate: 115200
   - Data: 8 bit
   - Parity: None
   - Stop bits: 1
   - Flow control: None

## Expected Output

The device will continuously send an incrementing counter value every second in the format:
```
Counter: 0
Counter: 1
Counter: 2
...
```

## Development Environment

- STM32CubeMX
- Keil MDK-ARM
- STM32F1 HAL Drivers

## License

This project is released under the MIT License. 