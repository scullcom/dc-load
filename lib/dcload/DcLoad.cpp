#include "DcLoad.h"

DcLoad::DcLoad(uint8_t lcdI2cAddress, uint8_t dacI2cAddress,
        uint8_t adcI2cAddress, uint8_t timerAddress, byte rotaryEncoderPinA,
        byte rotaryEncoderPinB, byte cursorPositionPin,
        byte loadTogglePin,byte currentPin,
        byte resistancePin, byte powerPin,
        byte battaryPin, byte fanPin,
        byte temperatureAddr, int fanTMin, int fanTMax){
  // store the inputs as private properities
  _dacI2cAddress = dacI2cAddress;
  // create instances of the LCD, DAC, ADC, LM35 and FanController for use by DcLoad
  _lcd = new LiquidCrystal_I2C(lcdI2cAddress,2,1,0,4,5,6,7);
  _dac = Adafruit_MCP4725();
  _adc = MCP342x(adcI2cAddress);
  _timer = MCP79410_Timer(timerAddress);
  _tempSensor = LM35(temperatureAddr);
  _fanController = FanController(fanPin, fanTMin, fanTMax);

  // store the pin configurations
  _rotaryEncoderPinA = rotaryEncoderPinA;
  _rotaryEncoderPinB = rotaryEncoderPinB;
  _cursorPositionPin = cursorPositionPin;
  _loadTogglePin = loadTogglePin;
  _currentPin = currentPin;
  _resistancePin = resistancePin;
  _powerPin = powerPin;
  _battaryPin = battaryPin;

  // set cursor/encoder control properties
  _cursorFactor = 0;
  _cursorPosition = 8;
  _cursorPositionPrevious = 8;
  _encoderPosition = 0;
  _encoderReading = 0;
  _encoderMax = 50000;
  _encoderPositionPrevious = -1;
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
  // battery values
  _batteryCutOff = 3.0;
  _batteryLife = 0;
  _batteryLifePrevious = 0;
  // safety values
  _tempMax = 35.0;
  _powerMax = 5.0;
  _batteryMaxCurrent = 1.0;
  _voltsMax = 15.0;
  _currentMax = 3.0;
  _safetyStatus = 0;
  _safetyStatusPervious = 0;
}

// Public methods
void DcLoad::setup(int initialOperatingMode, int welcomeDisplayMs){
  // set the analogReference
  analogReference(INTERNAL);
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
  //encoder readings
  _lcdUpdateEncoderReading();
  //set the cursor position
  _lcdSetCursor();
  // display the load status
  _lcdLoadStatus();
  // set the DAC control voltage and send it to the DAC
  _setDacControlVoltage();
  // check the temperature, start fan if required and output to display
  _fanControl();
  //reset the timer
  _timer.reset();
}

void DcLoad::run(){
  // check if the operating mode has been changed and update
  // the display if required
  _switchOperatingMode();
  // update the encoder reading
  _lcdUpdateEncoderReading();
  //set the cursor position
  _lcdSetCursor();
  // get the voltage and current readings from the ADC
  _getAdcVolatgeAndCurrent();
  // update the voltage, current and power values on the display
  _lcdUpdateAdcReading();
  // set the cursor position for the rotary encoderMax
  _setCursorPosition();
  // check the load status and toggle if required
  _toggleLoad();
  // check that the DC load is operating in safe limits
  _safteyCheck();
  if (_safetyStatus == 0) {
    // Operating within safety parameters
    // Update the safety status if required
    _lcdSafteyStatus();
    // Batter Capacity specific actions
    if (_operatingMode == 4){
      //Clear the battery wanring from the lcd if required
      _lcdClearBatteryWarning();
      // turn off the load if the battery has reached the cut off voltage
      if (_actualVoltage <= _batteryCutOff){
        _loadOff();
      }
      //start and stop the timer according to the load
      _toggleTimer();
      // print the timer
      _lcdDisplayTime();
      //calculate the mAh
      _calculateBatteryLife();
      // display battery life
      _lcdDisplayBatteryLife();
    }
  } else{
    // DC load is outside of safe limits, turn off and display warning
    _loadOff();
    _lcdSafteyStatus();
  }
  // store the last safteyStatus for comparison
  _safetyStatusPervious = _safetyStatus;
  // send the control voltage to the DAC
  _setDacControlVoltage();
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

void DcLoad::setSafetyLimits(float voltage, float current, float power,
                     float temperature, float batterCurrent){
   _tempMax = temperature;
   _powerMax = power;
   _batteryMaxCurrent = batterCurrent;
   _voltsMax = voltage;
   _currentMax = current;
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
  pinMode (_battaryPin, INPUT_PULLUP);
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
      _operatingModeOn = "DC LOAD ON ";
      _operatingModeOff = "DC LOAD OFF";
      break;
    case 2:
      _operatingModeDisplay = "CP";
      _operatingModeUnit = "mW";
      _operatingModeSetting = "Set W = ";
      _operatingModeOn = "DC LOAD ON ";
      _operatingModeOff = "DC LOAD OFF";
      break;
    case 3:
      _operatingModeDisplay = "CR";
      _operatingModeUnit = ((char)0xF4);
      _operatingModeSetting = "Set R = ";
      _operatingModeOn = "DC LOAD ON ";
      _operatingModeOff = "DC LOAD OFF";
      break;
      case 4:
        _operatingModeDisplay = "BC";
        _operatingModeUnit = "mA";
        _operatingModeSetting = "Set I = ";
        _operatingModeOn = "BATTERY ON ";
        _operatingModeOff = "BATTERY OFF";
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
      case 4:
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
        tempCurrent = (_actualVoltage / _encoderPosition * 1000.0) * 1000.0;
        _dacControlVoltage = (tempCurrent / (_dacInputRange/_dacVref) ) *
                              _currentCalibrationFactor;
        break;
    }
  } else {
    _dacControlVoltage = 0;
  }
  unsigned long dacValue = _dacControlVoltage;
  // send the control voltage to the DAC
  _dac.setVoltage(dacValue,false);
}


/*
 * void fanControl
 * reads the temperature of the heatsink and controls the fan speed
 */
void DcLoad::_fanControl() {
  // read the temperature
  _fanController.adjustToTemperature(_tempSensor.celsius());
  _lcdTemperatureStatus();
}

void DcLoad::_calculateBatteryLife(){
  float secondsElapsed = _timer.getTotalSeconds();
  _batteryLife = round((_actualCurrent * 1000)*(secondsElapsed/3600));
}

void DcLoad::_safteyCheck(){
  if (_actualPower > _powerMax){
    _safetyStatus = 1;
  } else if (_tempSensor.celsius() > _tempMax) {
    _safetyStatus = 2;
  } else if (_operatingMode == 4 && _encoderReading > _batteryMaxCurrent ) {
    _safetyStatus = 3;
  } else if(_actualVoltage > _voltsMax) {
    _safetyStatus = 4;
  } else if(_actualCurrent > _currentMax) {
    _safetyStatus = 5;
  } else {
    _safetyStatus = 0;
  }
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
  boolean changed = false;
  if(digitalRead(_currentPin) == LOW) {
    _setOperatingMode(1);
    changed=true;
  }
  // check if Constant Power button pressed
  if(digitalRead(_powerPin) == LOW) {
    _setOperatingMode(2);
    changed=true;
  }
  // check if Constant Resistance button pressed
  if(digitalRead(_resistancePin) == LOW) {
    _setOperatingMode(3);
    changed=true;
  }
  // check if Battery test button pressed
  if(digitalRead(_battaryPin) == LOW) {
    delay(200);
    _setOperatingMode(4);
    /*
    if (_operatingMode == 4) {
      _operatingMode = 1;
    } else {
      _operatingMode++;
    }
    _setOperatingMode(_operatingMode);
    */
    changed=true;
  }
  if (changed){
      _timer.reset();
      _encoderPosition = 0;
      _encoderPositionPrevious = -1;
      _batteryLife = 0;
      _batteryLifePrevious = 0;
      _loadOff();
      _lcdOperatingMode();
  }
}

void DcLoad::_setCursorPosition() {
  if (digitalRead(_cursorPositionPin) == LOW) {
    delay(200);  //simple key bounce delay
    _cursorPositionPrevious = _cursorPosition;
    _cursorPosition++;
    if (_cursorPosition==10){
      _cursorPosition++;
    }
  }
  if (_cursorPosition > 13){
    _cursorPosition = 8;
  }
  if (_cursorPosition == 13){
    _cursorFactor = 1;
  }
  if (_cursorPosition == 12) {
    _cursorFactor = 10;
  }
  if (_cursorPosition == 11){
    _cursorFactor = 100;
  }
  if (_cursorPosition == 9) {
    _cursorFactor = 1000;
  }
  if (_cursorPosition == 8) {
    _cursorFactor = 10000;
  }
}

void DcLoad::_toggleLoad(){
  if (digitalRead(_loadTogglePin) == LOW) {
      _loadStatus = !_loadStatus;
      _lcdLoadStatus();
  }
}

void DcLoad::_toggleTimer(){
  if (_loadStatus){
    _timer.start();
  } else {
    _timer.stop();
  }
}

void DcLoad::_loadOff(){
  _loadStatus = false;
  _lcdLoadStatus();
}

void DcLoad::_loadOn(){
  _loadStatus = true;
  _lcdLoadStatus();
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
  //_lcd->setCursor(0,2);
  //_lcd->print("                ");
  _lcdClearLine(2);
  if (_safetyStatus != _safetyStatusPervious && _safetyStatus == 0){
    _lcdClearLine(3);
  }
  _lcd->setCursor(18,2);
  _lcd->print(_operatingModeDisplay);
  _lcd->setCursor(0,2);
  _lcd->print(_operatingModeSetting);
  _lcd->setCursor(14,2);
  _lcd->print(_operatingModeUnit);
  //display extra info for battery test
  if (_operatingMode == 4){
    _lcd->setCursor(0,3);
    _lcd->print("00:00:00");
    _lcd->setCursor(13,3);
    _lcd->print("0000mAh");
  } else {
    _lcd->setCursor(0,3);
    _lcd->print("                    ");
  }
}

void DcLoad::_lcdUpdateEncoderReading(){

  // only update the display if the _encoderReading has changed from it's
  // last setting
  if (_encoderPosition != _encoderPositionPrevious) {
    _encoderReading = _encoderPosition / 1000;
    // start position of setting cusor position (indicates which digit will change)
    _lcd->setCursor(8,2);
    // ensure leading zero's are displayed
    if (_encoderReading < 10) {
      _lcd->print("0");
    }

    _lcd->print (_encoderReading,3);
    // store the new value of _encoderReading in _encoderPreviousReading
    // for the next check
    _encoderPositionPrevious = _encoderPosition;
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
  _lcd->setCursor(0,1);
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
    _lcd->setCursor(0,0);
    _lcd->print(_operatingModeOn);
  } else {
    _lcd->setCursor(0,0);
    _lcd->print(_operatingModeOff);
  }
}

void DcLoad::_lcdTemperatureStatus(){
  _lcd->setCursor(16,0);
  _lcd->print(_tempSensor.celsius(),0);
  _lcd->print((char)0xDF);
  _lcd->print("C");
  _lcd->setCursor(11,2);
  /*
  switch(_fanController.status()){
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
  */

}

void DcLoad::_lcdSetCursor(){
    _lcd->setCursor(_cursorPosition,2);
    _lcd->cursor();
}

void DcLoad::_lcdDisplayTime(){
  _lcd->setCursor(0, 3);
  _lcd->print(_timer.getTime());
  _lcd->print("     ");
}

void DcLoad::_lcdDisplayBatteryLife(){
  //only update LCD (mAh) if BatteryLife has increased
  if(_batteryLife >= _batteryLifePrevious && _safetyStatus == 0){
    //add a 3 leading zero to display if reading less than 10
    _lcd->setCursor(13,3);
    if (_batteryLife < 10) {
      _lcd->print("000");
    }
    //add a 2 leading zero to display
    if (_batteryLife >= 10 && _batteryLife <100){
      _lcd->print("00");
    }
    //add a 1 leading zero to display
    if (_batteryLife >= 100 && _batteryLife <1000){
      _lcd->print("0");
    }
      _lcd->print(_batteryLife,0);

    _batteryLifePrevious = _batteryLife;
  }
}

void DcLoad::_lcdSafteyStatus(){
  if (_safetyStatus != _safetyStatusPervious){
    _lcdClearLine(3);
    switch (_safetyStatus) {
      case 1:
        _lcd->setCursor(0,3);
        _lcd->print("Max power exceeded");
        delay(2000);
        break;
      case 2:
        _lcd->setCursor(0,3);
        _lcd->print("Max temp exceeded");
        break;
      case 3:
        _lcd->setCursor(0,3);
        _lcd->print("Bat current exceeded");
        break;
      case 4:
        _lcd->setCursor(0,3);
        _lcd->print("Max voltage exceeded");
        break;
        case 5:
          _lcd->setCursor(0,3);
          _lcd->print("Max current exceeded");
          delay(2000);
          break;
    }
  }
}

void DcLoad::_lcdClearLine(int lineNumber){
  _lcd->setCursor(0,lineNumber);
  _lcd->print("                    ");
}

void DcLoad::_lcdClearBatteryWarning(){
  if (_safetyStatus != _safetyStatusPervious && _safetyStatus == 0) {
    _lcd->setCursor(8, 3);
    _lcd->print("     ");
    _lcd->setCursor(17, 3);
    _lcd->print("mAh");
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
