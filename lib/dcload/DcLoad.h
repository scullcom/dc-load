#ifndef DcLoad_h
#define DcLoad_h

#include <Arduino.h>
#include <Wire.h>
#include "LiquidCrystal_I2C.h"
#include "Adafruit_MCP4725.h"
#include "MCP342x.h"
#include "LM35.h"
#include "FanController.h"
#include "MCP79410_Timer.h"

class DcLoad {
  public:
    DcLoad (uint8_t lcdI2cAddress, uint8_t dacI2cAddress,
            uint8_t adcI2cAddress, uint8_t timerAddress, byte rotaryEncoderPinA,
            byte rotaryEncoderPinB, byte cursorPositionPin,
            byte loadTogglePin,byte currentPin,
            byte resistancePin, byte powerPin,
            byte battaryPin, byte fanPin,
            byte temperatureAddr, int fanTempMin,
            int fanTempMax);
    void setup(int initialOperatingMode, int welcomeDisplayMs);
    void run();
    void setRotaryEncoderPosition();
    void setSafetyLimits(float voltage, float current, float power, 
                         float temperature, float batterCurrent);
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
    void _calculateBatteryLife();
    void _safteyCheck();
    // event handlers
    void _switchOperatingMode();
    void _setCursorPosition();
    void _toggleLoad();
    void _toggleTimer();
    void _loadOn();
    void _loadOff();
    // lcd methods
    void _lcdWelcome(int displayMs);
    void _lcdOperatingMode();
    void _lcdUpdateEncoderReading();
    void _lcdUpdateAdcReading();
    void _lcdLoadStatus();
    void _lcdTemperatureStatus();
    void _lcdSetCursor();
    void _lcdDisplayBatteryLife();
    void _lcdDisplayTime();
    void _lcdSafteyStatus();
    void _lcdClearLine(int lineNumber);
    void _lcdClearBatteryWarning();
    // helper methods
    float _convertAdcVoltageOrCurrent(long inputValue, int multiplier);
    // PRIVATE PROPERTIES
    // lcd, dac and adc
    LiquidCrystal_I2C* _lcd;
    Adafruit_MCP4725 _dac;
    MCP342x _adc;
    LM35 _tempSensor;
    FanController _fanController;
    MCP79410_Timer _timer;
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
    byte _battaryPin;
    byte _fanPin;
    byte _temperatureAddr;
    // cursor/encoder control properties
    int _cursorPosition;
    int _cursorPositionPrevious;
    volatile unsigned long _cursorFactor;
    volatile unsigned long _encoderMax;
    volatile float _encoderPosition;
    float _encoderReading;
    volatile unsigned int _encoderPositionPrevious;
    // voltage and current properties
    long _current;
    long _voltage;
    float _actualVoltage;
    float _actualCurrent;
    float _actualPower;
    float _dacControlVoltage;
    float _currentCalibrationFactor;
    float _dacInputRange;
    float _dacVref;
    // protection properties
    float _tempMax;
    float _powerMax;
    float _voltsMax;
    float _currentMax;
    float _batteryMaxCurrent;
    int _safetyStatus;
    int _safetyStatusPervious;
    // the status of the DC load (true = on, false = off)
    boolean _loadStatus;
    // properties to store the operating mode
    //and assiociated values
    int _operatingMode;
    String _operatingModeOn;
    String _operatingModeOff;
    String _operatingModeDisplay;
    String _operatingModeUnit;
    String _operatingModeSetting;
    //battery capacity values;
    float _batteryCutOff;
    float _batteryLife;
    float _batteryLifePrevious;
};

#endif
