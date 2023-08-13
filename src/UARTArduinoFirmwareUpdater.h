#ifndef UARTArduinoFirmwareUpdater_H
#define UARTArduinoFirmwareUpdater_H

#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include <IntelHexParser.h>

#define FLASH_RETURN_CODE_SUCCESS 0
#define FLASH_RETURN_CODE_INVALID_FILE_POSITION 1
#define FLASH_RETURN_CODE_FILE_NOT_FOUND 2
#define FLASH_RETURN_CODE_FILE_READ_FAILED 3
#define FLASH_RETURN_CODE_STK_FAILED_SYNC 4
#define FLASH_RETURN_CODE_STK_FAILED_ADDRESS 5
#define FLASH_RETURN_CODE_STK_FAILED_PAGE_WRITE 6
#define FLASH_RETURN_CODE_STK_FAILED_LEAVING_PROG_MODE 7

#define BOOTLOADER_RESPONSE_TIMEOUT 1000UL
#define STK_FLASH_START 0x46 // 'F'
#define STK_GET_SYNC 0x30
#define STK_CRC_EOP 0x20
// list of available codes see also https://github.com/Optiboot/optiboot/blob/master/optiboot/bootloaders/optiboot/stk500.h#L21
#define STK_ENTER_PROGMODE 0x50
#define STK_LEAVE_PROGMODE 0x51
#define STK_LOAD_ADDRESS 0x55
#define STK_PROG_PAGE 0x64

// https://mischianti.org/forums/topic/flash-bin-file-from-spiffs-of-esp8266-via-serial-to-arduino-uno/
// https://github.com/JayLooi/RemoteArduino/blob/master/ESP8266_Interface/ArduinoOTAFirmwareUpdater/ArduinoOTAFirmwareUpdater.cpp
// https://github.com/Optiboot/optiboot
struct flashResult
{
    byte returnCode = FLASH_RETURN_CODE_SUCCESS;
    u_int32_t bytesFlashed = 0;
};

class UARTArduinoFirmwareUpdater
{
public:
    UARTArduinoFirmwareUpdater(HardwareSerial *serialPort, u_int8_t reset_pin, u_int32_t baudRate);
    flashResult flashFirmware(File *fileToFlash);
    flashResult flashFirmwareFromSpiffs(String filePathInSpiffs);

private:
    void sendByte(uint8_t data);
    void resetFlashTarget();
    bool executeStkSync();
    bool awaitOptibootResponse();
    bool sendCommandWithResponseCheck(byte cmd);
    bool setFlashAddress(uint16_t address);
    flashResult writePages(File *fileToFlash);
    flashResult writeRecords(File *fileToFlash);

    HardwareSerial *_serialPort;
    u_int8_t _resetPin;
    u_int32_t _prevBaudRate;
};

#endif