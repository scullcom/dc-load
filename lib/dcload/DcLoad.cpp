#include "DcLoad.h"

DcLoad::DcLoad(uint8_t lcdI2cAddress, uint8_t dacI2cAddress,
        uint8_t adcI2cAddress,byte rotaryEncoderPinA,
        byte rotaryEncoderPinB, byte cursorPositionPin,
        byte loadTogglePin,byte currentPin,
        byte resistancePin, byte powerPin,
        byte userSettingPin, byte fanPin,
        byte temperatureAddr, int fanTempMin,
        int fanTempMax){
  // store the inputs as private properities
  _lcdI2cAddress = lcdI2cAddress;
  _dacI2cAddress = dacI2cAddress;
  _adcI2cAddress = adcI2cAddress;
  // create instances of the LCD, DAC and ADC for use by DcLoad
  _lcd = new LiquidCrystal_I2C(_lcdI2cAddress,2,1,0,4,5,6,7);
  _dac = Adafruit_MCP4725();
  _adc = MCP342x(_adcI2cAddress);
  // store the pin configurations
  _rotaryEncoderPinA = rotaryEncoderPinA;
  _rotaryEncoderPinB = rotaryEncoderPinB;
  _cursorPositionPin = cursorPositionPin;
  _loadTogglePin = loadTogglePin;
  _currentPin = currentPin;
  _resistancePin = resistancePin;
  _powerPin = powerPin;
  _userSettingPin = userSettingPin;
  _fanPin = fanPin;
  _temperatureAddr = temperatureAddr;
  // set up start values
  _fanTempMax = fanTempMax; // fan temps in Celcius
  _fanTempMin = fanTempMin;
  // set cursor/encoder control properties
  _cursorFactor = 0;
  _cursorPosition = 8;
  _encoderPosition = 0;
  _encoderMax = 9000;
  _encoderReading = 0;
  _encoderPreviousReading = 0;
  // set voltage and current properties
  _current = 0;
  _voltage = 0;
  _dacControlVoltage = 0;
  _dacInputRange = 5000;
  _dacVref = 4096;
  _currentCalibrationFactor = 1;
  _operatingMode = 0;
  // set load status
  _loadStatus = false;
}

// Public methods
void DcLoad::setup(int initialOperatingMode, int welcomeDisplayMs){
  Wire.begin();
  // setup the ADC
  _setupAdc();
  // setup the DAC
  _setupDac();
  // setup the LCD
  _setupLcd();
  // set up the pins
  _setupPins();
  // display the welcome message for 3 seconds
  _lcdWelcome(welcomeDisplayMs);
  // set the operating mode to CC(1)
  _setOperatingMode(initialOperatingMode);
  // update the display to show the operating mode
  _lcdOperatingMode();
  // display the load status
  _lcdLoadStatus();
  // set the DAC control voltage and send it to the DAC
  _setDacControlVoltage();
  // check the temperature, start fan if required and output to display
  _fanControl();
}

void DcLoad::run(){
  // check if the operating mode has been changed and update
  // the display if required
  _switchOperatingMode();
  // update the encoder reading
  _lcdUpdateEncoderReading();
  // get the voltage and current readings from the ADC
  _getAdcVolatgeAndCurrent();
  // update the voltage, current and power values on the display
  _lcdUpdateAdcReading();
  // set the cursor position for the rotary encoderMax
  _setCursorPosition();
  // check the load status and toggle if required
  _toggleLoad();
  // check the temperature, start fan if required and output to display
  _fanControl();
}

void DcLoad::setRotaryEncoderPosition(){
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 5) {  //
    if (digitalRead(_rotaryEncoderPinB) == LOW)
    {
      _encoderPosition = _encoderPosition - _cursorFactor;
    } else {
      _encoderPosition = _encoderPosition + _cursorFactor;
    }
    // sets maximum range of rotary encoder if it exceeds the max value
    _encoderPosition = min(_encoderMax, max(0, _encoderPosition));
    lastInterruptTime = interruptTime;
  }
}

// PRIVATE - setup methods
void DcLoad::_setupLcd(){
  _lcd->begin(20, 4);
  _lcd->setBacklightPin(3,POSITIVE);
  _lcd->setBacklight(HIGH);
  _lcd->clear();
}

void DcLoad::_setupDac(){
  // the DAC I2C address with MCP4725 pin A0 set high
  _dac.begin(_dacI2cAddress);
  // reset DAC to zero for no output current set at switch on
  _dac.setVoltage(0,false);
}

void DcLoad::_setupAdc(){
  _adc.generalCallReset();
  delay(1);
}

void DcLoad::_setupPins(){
  // set pin modes
  pinMode (_rotaryEncoderPinA, INPUT);
  pinMode (_rotaryEncoderPinB, INPUT);
  pinMode (_loadTogglePin, INPUT_PULLUP);
  pinMode (_cursorPositionPin, INPUT_PULLUP);
  pinMode (_currentPin, INPUT_PULLUP);
  pinMode (_powerPin, INPUT_PULLUP);
  pinMode (_resistancePin, INPUT_PULLUP);
  pinMode (_userSettingPin, INPUT_PULLUP);
  pinMode (_fanPin, OUTPUT);
  pinMode (_temperatureAddr, INPUT);
}

// private dcLoad control methods
/*
 * void _setOperatingMode
 * sets if the dc load is operating in CC(1), CP(2) or CR(3) mode.
 */
void DcLoad::_setOperatingMode(int operatingMode) {
  _operatingMode = operatingMode;
  switch (_operatingMode){
    case 1:
      _operatingModeDisplay = "CC";
      _operatingModeUnit = "mA";
      _operatingModeSetting = "Set I = ";
      break;
    case 2:
      _operatingModeDisplay = "CP";
      _operatingModeUnit = "mW";
      _operatingModeSetting = "Set W = ";
      break;
    case 3:
      _operatingModeDisplay = "CR";
      _operatingModeUnit = ((char)0xF4);
      _operatingModeSetting = "Set R = ";
      break;
  }
}

/*
 * void _getAdcVoltageAndCurrent
 * gets the voltage and current read by the ADC and store the values in
 * _voltage and _current
 */
void DcLoad::_getAdcVolatgeAndCurrent() {
  MCP342x::Config status;
  // Initiate a conversion; convertAndRead() will wait until it can be read
  _adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot,
                    MCP342x::resolution16, MCP342x::gain1,
                    1000000, _voltage, status);

  // Initiate a conversion; convertAndRead() will wait until it can be read
  _adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot,
                    MCP342x::resolution16, MCP342x::gain1,
                    1000000, _current, status);
}

void DcLoad::_setDacControlVoltage(){
  if (_loadStatus){
    float tempCurrent = 0;
    switch (_operatingMode) {
      case 1:
        tempCurrent = _encoderPosition;
        _dacControlVoltage = (tempCurrent / (_dacInputRange/_dacVref) ) *
                              _currentCalibrationFactor;
        break;
      case 2:
        tempCurrent = _encoderPosition / _actualVoltage;
        _dacControlVoltage = (tempCurrent / (_dacInputRange/_dacVref) ) *
                              _currentCalibrationFactor;
        break;
      case 3:
        tempCurrent = (_actualVoltage * 1000) / _encoderPosition;
        _dacControlVoltage = (tempCurrent / (_dacInputRange/_dacVref) ) *
                              _currentCalibrationFactor;
        break;
    }
  } else {
    _dacControlVoltage = 0;
  }
  // send the control voltage to the DAC
  _dac.setVoltage(_dacControlVoltage,false);
}

/*
 * void fanControl
 * reads the temperature of the heatsink and controls the fan speed
 */
void DcLoad::_fanControl() {
  // read the temperature
  _fanTemp = analogRead(_temperatureAddr);
  // convert to Celsius
  _fanTemp = _fanTemp * 0.107421875;
  if (_fanTemp < _fanTempMin) {
    // if temperature is lower than the minimum temperature, turn the fan off
    _fanSpeed = 0;
    digitalWrite(_fanPin, LOW);
    _fanStatus = 0;
  } else if ((_fanTemp >= _fanTempMin) && (_fanTemp <= _fanTempMax)){
    // if temperature is between the min and max temp
    // map the fan to a suitable speed
    _fanSpeed = map(_fanTemp, _fanTempMin, _fanTempMax, 100, 255);
    analogWrite(_fanPin, _fanSpeed);
    _fanStatus = 1;
  } else {
    // if the max temp is exceeded, run the fan at max speed
    _fanSpeed = 255;
    analogWrite(_fanPin, _fanSpeed);
    _fanStatus = 2;
  }
  _lcdTemperatureStatus();
}

// private event handlers
/*
 * void setOperatingMode
 * sets the operating mode of the DC Load
 * 1 = CC (Constant Current)
 * 2 = CP (Constant Power)
 * 3 = CR (Constant Resistance)
 */
void DcLoad::_switchOperatingMode(){
  // check if Constant Current button pressed
  if(digitalRead(_currentPin) == LOW) {
    _setOperatingMode(1);
  }
  // check if Constant Power button pressed
  if(digitalRead(_powerPin) == LOW) {
    _setOperatingMode(2);
  }
  // check if Constant Resistance button pressed
  if(digitalRead(_resistancePin) == LOW) {
    _setOperatingMode(3);
  }
  _lcdOperatingMode();
}

void DcLoad::_setCursorPosition() {
  if (digitalRead(_cursorPositionPin) == LOW) {
    delay(10);  //simple key bounce delay
    _cursorPosition = _cursorPosition + 1;
  }
  if (_cursorPosition > 11){
    _cursorPosition = 8;
  }
  if (_cursorPosition == 11){
    _cursorFactor = 1;
  }
  if (_cursorPosition == 10) {
    _cursorFactor = 10;
  }
  if (_cursorPosition == 9){
    _cursorFactor = 100;
  }
  if (_cursorPosition == 8) {
    _cursorFactor = 1000;
  }
}

void DcLoad::_toggleLoad(){
  if (digitalRead(_loadTogglePin) == LOW) {
      _loadStatus = !_loadStatus;
      _lcdLoadStatus();
  }
}

// private LCD display methods
void DcLoad::_lcdWelcome(int displayMs){
  _lcd->setCursor(6,0);
  _lcd->print("SCULLCOM");
  _lcd->setCursor(1,1);
  _lcd->print("Hobby Electronics");
  _lcd->setCursor(1,2);
  _lcd->print("DC Electronic Load"); //
  _lcd->setCursor(0,3);
  _lcd->print("Software Version 1.0"); //
  // mSec delay for intro display
  delay(displayMs);
  _lcd->clear();
}

void DcLoad::_lcdOperatingMode(){
  _lcd->setCursor(0,1);
  _lcd->print("                ");
  _lcd->setCursor(17,1);
  _lcd->print(_operatingModeDisplay);
  _lcd->setCursor(0,1);
  _lcd->print(_operatingModeSetting);
  _lcd->setCursor(12,1);
  _lcd->print(_operatingModeUnit);
}

void DcLoad::_lcdUpdateEncoderReading(){
  // only update the display if the _encoderReading has changed from it's
  // last setting
  if (_encoderReading != _encoderPreviousReading) {
    // start position of setting cusor position (indicates which digit will change)
    _lcd->setCursor(8,1);
    // ensure leading zero's are displayed
    if (_encoderReading < 10) {
      _lcd->print("000");
    } else if(_encoderReading < 100 ) {
      _lcd->print("00");
    } else if (_encoderReading < 1000) {
      _lcd->print("0");
    }
    //
    _lcd->print (_encoderReading);
    // store the new value of _encoderReading in _encoderPreviousReading
    // for the next check
    _encoderPreviousReading = _encoderReading;
  }
}

void DcLoad::_lcdUpdateAdcReading(){
  // get the readable values for current and voltage from the DC load class
  _actualCurrent = _convertAdcVoltageOrCurrent(_current,4);
  _actualVoltage = _convertAdcVoltageOrCurrent(_voltage,50);
  // calculate the power in watts
  _actualPower = _actualCurrent * _actualVoltage;
  // set the number of decimal places for the voltage
  int voltsDecimalPlaces = 2;
  if (_actualVoltage < 10) {
    voltsDecimalPlaces = 3;
  }
  _lcd->setCursor(0,0);
  _lcd->print(_actualCurrent,3);
  _lcd->print("A");
  _lcd->print(" "); // two spaces between actual current and voltage readings
  _lcd->print(_actualVoltage,voltsDecimalPlaces);
  _lcd->print("V");
  //lcd.setCursor(0,1);
  _lcd->print(" ");
  _lcd->print(_actualPower,2);
  _lcd->print("W");
  _lcd->print(" ");
}

void DcLoad::_lcdLoadStatus(){
  if (_loadStatus){
    _lcd->setCursor(0,3);
    _lcd->print("LOAD ON ");
  } else {
    _lcd->setCursor(0,3);
    _lcd->print("LOAD OFF");
  }
}

void DcLoad::_lcdTemperatureStatus(){
  _lcd->setCursor(0,2);
  _lcd->print(_fanTemp);
  _lcd->print((char)0xDF);
  _lcd->print("C");
  _lcd->setCursor(11,2);
  switch(_fanStatus){
    case 0:
      _lcd->print("FAN: OFF");
      break;
    case 1:
      _lcd->print("FAN:  ON");
      break;
    case 2:
      _lcd->print("FAN: MAX");
      break;
  }

}

// helper methods
/*
 * float _convertAdcVoltageOrCurrent
 * takes a voltage or current reading from the ADC and converts it to a human
 * readable value
 */
float DcLoad::_convertAdcVoltageOrCurrent(long inputValue, int multiplier){
  return (((inputValue * 2.048) / 32767) * multiplier);
}
