#include <SPIFFS.h>
#include <IntelHexParser.h>
#include <UARTArduinoFirmwareUpdater.h>

/**
 * Example for using the ESP-UARTArduinoFirmwareUpdate library to write a new firmware onto an Arduino connected to the ESP32 via Serial port.
 * This specific example flashes a provided HEX firmware file (in the example for an Atmega32(A) but can be used to flash any firmware on any ATMEGA that runs Optiboot bootloader)
 *
 * Requirements:
 * - [ESP-IntelHexParser](https://github.com/pkerspe/ESP-IntelHex-Parser) library needs to be installed in your IDE
 * - free UART (RX/TX) pins from ESP32 (this example uses the second serial port of the ESP32 with its default pins, you can also use the first one if needed)
 * - gnd connection between Atmega and ESP32
 * - connection between a free GPIO pin of the ESP32 to the reset pin of the Atmega
 * - Atmega needs to have Optiboot bootloader installed (standard bootloader nowadays for most Arduino models, you can burn the bootloader in the Arduino IDE)
 *
 * IMPORTANT NOTE: if you run the ATMEGA at 5V you need a simple voltage divider as level shifter on RX pin of the ESP to shift the 5V signals from the ATMEGA to 3.3V!
 *
 * You need to upload a hex file containing the firmware you want to flash to SPIFFS on your ESP in order to execute this example.
 * You can use the firmware hex file in the data subfolder of this example, it contains a simple blink firmware for an Atmega32(A) as an example
 */

#define RESET_PIN 15 // adapt this to whatever ESP32 pin you use for resetting the Atmega
#define SERIAL_FLASHING_BAUD_RATE 38400
#define FIRMWARE_SPIFFS_PATH "/firmware_1.hex"

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting ESP Atmega firmware flasher via serial");

    // Initialize SPIFFS
    if (!SPIFFS.begin(true))
    {
        Serial.println("SPIFFS Mount Failed");
        return;
    }

    // create instance of the flasher class using the parameters for Serial port, baud rate and reset pin
    // use Serial2 (Second ESP32 UART interface in this case, it works also with the first one, but than you cannot output debug data on the serial port while flashing)
    // define baud rate to communicate with the bootloader on the Atmega, this depend on the Clock Speed of the Atmega which baud rate works best
    // this example has been tested with an Atmega32A running on the internal oscillator at 8 MHz, thus a baud rate of 38400 has been selected.
    // see also https://github.com/MCUdude/MightyCore for supported/recommended clock speeds
    // The reset pin is the ESP32 pin that is connected to the reset pin of the Atmega to be flashed
    UARTArduinoFirmwareUpdater flasher(&Serial2, RESET_PIN, SERIAL_FLASHING_BAUD_RATE);

    Serial.printf("Attempting to flash firmware from file %s to Atmega via serial port\n", FIRMWARE_SPIFFS_PATH);
    // start flashing the firmware by specifying the path in SPIFFS to the file. You can also use the function flashFirmware(File *file) instead an provide a pointer to an instance of File if you do not want to use SPIFFS
    flashResult result = flasher.flashFirmwareFromSpiffs(FIRMWARE_SPIFFS_PATH);

    // now the flash results are container in the struct "result"
    // the return code should be 0 to indicate successful flashing.
    // see macro definitions in UARTArduinoFirmwareUpdater.h starting with "FLASH_RETURN_CODE_" for list of possible return values
    Serial.printf("Firmware flashing process completed. A total of %u bytes have been written. Return code was %i\n", result.bytesFlashed, result.returnCode);
}

void loop()
{
    // do whatever you want here
}
