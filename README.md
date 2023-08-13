# ESP-UARTArduinoFirmwareUpdate
A library to be used to use an ESP32 to flash a firmware file on an Arduino compatible MCU via the UART / Serial interface using the STK500 protocol.

## use cases
This library is intended to be used to update the firmware of an Atmega MUC running Optiboot using an ESP32 that is connected to the Atmega via UART / Serial interface.
For example if you are running an ESP32 and a Atmega as "co processor" on the same board and you want to provide Over the Air Updates to the Atmega, you can download the new firmware hex file to the SPIFFS of the ESP32 and then flash it to the connected Atmega automatically. 
Also it could be used to develop an independent (without PC) flashing station for Atmega MCUs by using a standalone ESP32 that can be connected to Arduino Boards or basically any Atmega running the Optiboot bootloader via Serial interface and flash the firmware at a press of a button on the ESP32.

## Requirements
- [ESP-IntelHexParser](https://github.com/pkerspe/ESP-IntelHex-Parser) library needs to be installed in your IDE
- free UART (RX/TX) pins from ESP32 (this example uses the second serial port of the ESP32 with its default pins, you can also use the first one if needed)
- GND connection between Atmega and ESP32
- connection between a free GPIO pin of the ESP32 to the reset pin of the Atmega
- Atmega needs to have Optiboot bootloader installed (standard bootloader nowadays for most Arduino models, you can burn the bootloader in the Arduino IDE)
 
> [!NOTE]
> IMPORTANT NOTE: if you run the Atmega at 5V you need a simple voltage divider as level shifter on RX pin of the ESP to shift the 5V signals from the Atmega to ~3.3V! This can be done with two resistors e.g. with 1k Ohm (from Atmega TX pin to second resistor) and 2k Ohm (from first resistor to ground)

## Example usage
see examples folder in this repo e.g. [https://github.com/pkerspe/ESP-UARTArduinoFirmwareUpdate/tree/main/examples/01_SerialArduinoFirmwareFlashing](https://github.com/pkerspe/ESP-UARTArduinoFirmwareUpdate/tree/main/examples/01_SerialArduinoFirmwareFlashing)

## Additional Information
The following pages provide good information if you want to know more about the involved file formate, protocols and mechanics on flashing an Atmega via serial, the STK500 protocol and the Optiboot bootloader:

**MightyCore:**
Optiboot/Ardino core for non--arduino-standard Atmega MCUs (e.g. ATmega8535, ATmega16(A), ATmega32(A), ATmega164, ATmega324, ATmega644 and ATmega1284) 
[https://github.com/MCUdude/MightyCore]([https://github.com/MCUdude/MightyCore)

**STK500 protocol and flow of firmware update on Arduino**
[https://mischianti.org/forums/topic/flash-bin-file-from-spiffs-of-esp8266-via-serial-to-arduino-uno/](https://mischianti.org/forums/topic/flash-bin-file-from-spiffs-of-esp8266-via-serial-to-arduino-uno/)

**Optiboot source ‚code and STK500 commands**: [https://github.com/Optiboot/optiboot/blob/master/optiboot/bootloaders/optiboot/stk500.h](https://github.com/Optiboot/optiboot/blob/master/optiboot/bootloaders/optiboot/stk500.h)

**Similar implementation for flashing via SPI**: [https://github.com/lbernstone/ESP_AVRISP/blob/master/src/ESP_AVRISP.cpp](https://github.com/lbernstone/ESP_AVRISP/blob/master/src/ESP_AVRISP.cpp)