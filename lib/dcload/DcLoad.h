#ifndef DcLoad_h
#define DcLoad_h

#include <Arduino.h>
#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include "Adafruit_MCP4725.h"
#include "MCP342x.h"

class DcLoad {
  public:
    DcLoad (uint8_t lcdI2cAddress, uint8_t dacI2cAddress,
            uint8_t adcI2cAddress,byte rotaryEncoderPinA,
            byte rotaryEncoderPinB, byte cursorPositionPin,
            byte loadTogglePin,byte currentPin,
            byte resistancePin, byte powerPin,
            byte userSettingPin, byte fanPin,
            byte temperatureAddr, int fanTempMin,
            int fanTempMax);
    void setup(int initialOperatingMode, int welcomeDisplayMs);
    void run();
    void setRotaryEncoderPosition();
  private:
    // setup methods
    void _setupLcd();
    void _setupDac();
    void _setupAdc();
    void _setupPins();
    //dcLoad control methods
    void _setOperatingMode(int operatingMode);
    void _getAdcVolatgeAndCurrent();
    void _setDacControlVoltage();
    void _fanControl();
    // event handlers
    void _switchOperatingMode();
    void _setCursorPosition();
    void _toggleLoad();
    // lcd methods
    void _lcdWelcome(int displayMs);
    void _lcdOperatingMode();
    void _lcdUpdateEncoderReading();
    void _lcdUpdateAdcReading();
    void _lcdLoadStatus();
    void _lcdTemperatureStatus();
    // helper methods
    float _convertAdcVoltageOrCurrent(long inputValue, int multiplier);
    // PRIVATE PROPERTIES
    // lcd, dac and adc
    LiquidCrystal_I2C* _lcd;
    Adafruit_MCP4725 _dac;
    MCP342x _adc;
    // i2c addresses
    uint8_t _lcdI2cAddress;
    uint8_t _dacI2cAddress;
    uint8_t _adcI2cAddress;
    // arduino pins
    byte _rotaryEncoderPinA;
    byte _rotaryEncoderPinB;
    byte _cursorPositionPin;
    byte _loadTogglePin;
    byte _currentPin;
    byte _resistancePin;
    byte _powerPin;
    byte _userSettingPin;
    byte _fanPin;
    byte _temperatureAddr;
    // cursor/encoder control properties
    int _cursorPosition;
    volatile unsigned long _cursorFactor;
    volatile unsigned long _encoderMax;
    volatile unsigned long _encoderPosition;
    volatile unsigned int _encoderReading;
    volatile unsigned int _encoderPreviousReading;
    // fan control properties
    int _fanTemp;
    int _fanSpeed;
    int _fanTempMax;
    int _fanTempMin;
    int _fanStatus;
    // voltage and current properties
    long _current;
    long _voltage;
    float _actualVoltage;
    float _actualCurrent;
    float _actualPower;
    unsigned long _dacControlVoltage;
    float _currentCalibrationFactor;
    float _dacInputRange;
    float _dacVref;
    // the status of the DC load (true = on, false = off)
    boolean _loadStatus;
    // properties to store the operating mode
    //and assiociated values
    int _operatingMode;
    String _operatingModeDisplay;
    String _operatingModeUnit;
    String _operatingModeSetting;

};

#endif
