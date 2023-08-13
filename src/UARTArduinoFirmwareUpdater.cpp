#include "UARTArduinoFirmwareUpdater.h"

// #define ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED

UARTArduinoFirmwareUpdater::UARTArduinoFirmwareUpdater(HardwareSerial *serialPort, u_int8_t reset_pin, u_int32_t baudRate)
{
    this->_serialPort = serialPort;
    this->_prevBaudRate = serialPort->baudRate();
    serialPort->begin(baudRate, SERIAL_8N1);

    this->_resetPin = reset_pin;
    pinMode(this->_resetPin, OUTPUT);
    digitalWrite(this->_resetPin, HIGH);
}

flashResult UARTArduinoFirmwareUpdater::flashFirmwareFromSpiffs(String filePathInSpiffs)
{
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.println("Starting flashFirmwareFromSpiffs");
#endif
    flashResult result;
    result.bytesFlashed = 0;
    if (!SPIFFS.exists(filePathInSpiffs))
    {
        result.returnCode = FLASH_RETURN_CODE_FILE_NOT_FOUND;
        return result;
    }

    File file = SPIFFS.open(filePathInSpiffs, "r");
    if (!file)
    {
        result.returnCode = FLASH_RETURN_CODE_FILE_READ_FAILED;
        return result;
    }

    result = this->flashFirmware(&file);
    file.close();
    return result;
}

flashResult UARTArduinoFirmwareUpdater::flashFirmware(File *fileToFlash)
{
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.println("Starting firmware flashing process");
#endif
    flashResult result;
    result.bytesFlashed = 0;
    if (fileToFlash->position() > 0 || !fileToFlash->available())
    {
        result.returnCode = FLASH_RETURN_CODE_INVALID_FILE_POSITION;
        this->_serialPort->begin(this->_prevBaudRate);
        return result;
    }

    // reset connected MCU via reset pin
    this->resetFlashTarget();
    // clear RX buffer to flush old values
    while (this->_serialPort->available())
    {
        this->_serialPort->read();
    }

    if (!this->executeStkSync())
    {
        result.returnCode = FLASH_RETURN_CODE_STK_FAILED_SYNC;
        this->_serialPort->begin(this->_prevBaudRate);
        return result;
    }

    sendCommandWithResponseCheck(STK_ENTER_PROGMODE); // this command might even be ignored by Optiboot

    result = this->writePages(fileToFlash);
    // result = this->writeRecords(fileToFlash);

// Send exit programming command
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.println("exiting programming mode");
#endif
    if (!sendCommandWithResponseCheck(STK_LEAVE_PROGMODE))
    {
        result.returnCode = FLASH_RETURN_CODE_STK_FAILED_LEAVING_PROG_MODE;
        this->_serialPort->begin(this->_prevBaudRate);
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
        Serial.println("Error during exiting programming mode");
#endif
        return result;
    }

#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.printf("Resetting serial port to %i baud\n", this->_prevBaudRate);
#endif
    this->_serialPort->begin(this->_prevBaudRate);

    return result;
}

void UARTArduinoFirmwareUpdater::sendByte(uint8_t data)
{
    this->_serialPort->write(data);
}

void UARTArduinoFirmwareUpdater::resetFlashTarget()
{
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.println("resetting target MCU");
#endif
    digitalWrite(this->_resetPin, LOW);
    delay(200);
    digitalWrite(this->_resetPin, HIGH);
    delay(300);
}

bool UARTArduinoFirmwareUpdater::executeStkSync()
{
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.println("attempting to sync with target MCU");
#endif
    return sendCommandWithResponseCheck(STK_GET_SYNC);
}

bool UARTArduinoFirmwareUpdater::awaitOptibootResponse()
{
    unsigned long startTime = millis();
    bool timeOut = false;
    while ((this->_serialPort->available() < 2) && !(timeOut = ((millis() - startTime) > BOOTLOADER_RESPONSE_TIMEOUT)))
        ;

    if (timeOut)
    {
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
        Serial.println("timeout while waiting for MCU response");
#endif
        return false;
    }

    byte firstByte = this->_serialPort->read();
    if (firstByte == 0x00)
    { // skip first zero byte if any
        firstByte = this->_serialPort->read();
    }
    byte secondByte = this->_serialPort->read();
    bool success = (firstByte == 0x14 && secondByte == 0x10);
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    if (!success)
    {
        Serial.printf("Unexpected response from MCU: 0x%02X  0x%02X (expected 0x14 0x10)\n", firstByte, secondByte);
    }
#endif
    return success;
}

bool UARTArduinoFirmwareUpdater::sendCommandWithResponseCheck(byte cmd)
{
    this->sendByte(cmd);
    this->sendByte(STK_CRC_EOP);
    delay(100);
    return this->awaitOptibootResponse();
}

bool UARTArduinoFirmwareUpdater::setFlashAddress(uint16_t address)
{
    sendByte(STK_LOAD_ADDRESS);
    sendByte(address & 0xFF);
    sendByte(address >> 8);
    sendByte(STK_CRC_EOP);
    return awaitOptibootResponse();
}

flashResult UARTArduinoFirmwareUpdater::writeRecords(File *fileToFlash)
{
    flashResult result;
    IntelHexParser hexParser(fileToFlash);
    byte bufferSize = 50;
    byte buffer[bufferSize];
    _recordDetailsStruct recordDetails = hexParser.getNextRecordToWrite(buffer, bufferSize);
    while (recordDetails.errorCode == 0 && recordDetails.endOfFileReached == false)
    {
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
        Serial.printf("Writing %i bytes to address %u\n", recordDetails.dataLength, recordDetails.address);
#endif
        if (!setFlashAddress(recordDetails.address))
        {
            result.returnCode = FLASH_RETURN_CODE_STK_FAILED_ADDRESS;
            this->_serialPort->begin(this->_prevBaudRate);
            return result;
        }

        sendByte(STK_PROG_PAGE);
        sendByte(0x00);
        sendByte(recordDetails.dataLength);
        sendByte(STK_FLASH_START); // could basically send anything, since optiboot defaults to Flash by default (https://github.com/Optiboot/optiboot/blob/55d1e6b36922e4b8e3a32e6cea8ec03127ed65bf/optiboot/bootloaders/optiboot/optiboot.c#L1469)
        for (int i = 0; i < recordDetails.dataLength; i++)
        {
            sendByte(buffer[i]);
            result.bytesFlashed++;
        }
        sendByte(STK_CRC_EOP);

        if (!awaitOptibootResponse())
        {
            result.returnCode = FLASH_RETURN_CODE_STK_FAILED_PAGE_WRITE;
            this->_serialPort->begin(this->_prevBaudRate);
            return result;
        }
        recordDetails = hexParser.getNextRecordToWrite(buffer, bufferSize);
    }
    return result;
}

flashResult UARTArduinoFirmwareUpdater::writePages(File *fileToFlash)
{
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.println("Starting to write pages to flash");
#endif
    flashResult result;
    result.bytesFlashed = 0;

    byte pageSize = 128; // this must be equal to the page size of the ATMEGA (=128 bytes for ATMEGA32A, with total of 256 pages equal to 32768 bytes of total flash)
    IntelHexParser hexParser(fileToFlash, pageSize);
    uint16_t address = 0x00;
    uint8_t buffer[pageSize];
    uint16_t pageCounter = 0;
    while (hexParser.getNextPage(buffer))
    {
        pageCounter++;
        if (!setFlashAddress(address))
        {
            result.returnCode = FLASH_RETURN_CODE_STK_FAILED_ADDRESS;
            this->_serialPort->begin(this->_prevBaudRate);
            return result;
        }

        sendByte(STK_PROG_PAGE);
        sendByte(0x00);
        sendByte(pageSize);
        sendByte(STK_FLASH_START); // could basically send anything, since optiboot defaults to Flash by default (https://github.com/Optiboot/optiboot/blob/55d1e6b36922e4b8e3a32e6cea8ec03127ed65bf/optiboot/bootloaders/optiboot/optiboot.c#L1469)
        for (int i = 0; i < pageSize; i++)
        {
            sendByte(buffer[i]);
            result.bytesFlashed++;
        }
        sendByte(STK_CRC_EOP);

        if (!awaitOptibootResponse())
        {
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
            Serial.println("Failed to write page");
#endif
            result.returnCode = FLASH_RETURN_CODE_STK_FAILED_PAGE_WRITE;
            this->_serialPort->begin(this->_prevBaudRate);
            return result;
        }
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
        Serial.printf("page written successfully to address 0x%x (total bytes written so far: %ld)\n", address, result.bytesFlashed);
#endif
        address += (pageSize / 2); // flash word size is two bytes
    }
#ifdef ARDUINO_SERIAL_FLASHER_DEBUG_ENABLED
    Serial.printf("Completed writing %i pages\n", pageCounter);
#endif
    return result;
}