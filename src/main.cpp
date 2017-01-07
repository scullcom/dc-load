#include "DcLoad.h"

// Constants
const byte ROTARY_ENCODER_PIN_A = 3;  // pin connected to pin A of the rotary encoder
const byte ROTARY_ENCODER_PIN_B = 4;  // pin connected to pin B of the rotary encoder

const byte CURSOR_POSITION_PIN = 5;  // digital pin 5 used to set cursor position
const byte LOAD_TOGGLE_PIN = 6;      // digital pin 6 used to toggle load on or off
const byte CURRENT_PIN = 7;          // digital pin 7 used to select constant current mode
const byte RESISTANCE_PIN = 8;       // digital pin 8 to select constant resistance mode
const byte POWER_PIN = 9;            // digital pin 9 to select constant power mode
const byte USER_SETTING_PIN = 10;    // unused pin?

const byte FAN_PIN = 2;              // digital pin 2 for fan control output
const byte TEMPERATURE_ADDR = A0;    // address of the temperature output from LM35
const int FAN_TEMP_MIN = 20;         // temperature at which to start the fan
const int FAN_TEMP_MAX = 50;         // maximum temperature when fan speed at 100%

const uint8_t ADC_I2C_ADDRESS = 0x68; // the I2C address of the ADC (MCP3426: 0x68 is the default)
const uint8_t DAC_I2C_ADDRESS = 0x61; // the I2C address of the DAC (Adafruit_MCP4725)
const uint8_t LCD_I2C_ADDRESS = 0x27; // the I2C address of the LCD (0x27 is the default)

DcLoad dcLoad = DcLoad(LCD_I2C_ADDRESS, DAC_I2C_ADDRESS, ADC_I2C_ADDRESS,
                       ROTARY_ENCODER_PIN_A, ROTARY_ENCODER_PIN_B,
                       CURSOR_POSITION_PIN, LOAD_TOGGLE_PIN,
                       CURRENT_PIN, RESISTANCE_PIN, POWER_PIN,
                       USER_SETTING_PIN, FAN_PIN, TEMPERATURE_ADDR,
                       FAN_TEMP_MIN, FAN_TEMP_MAX);

void isr(){
  dcLoad.setRotaryEncoderPosition();
}

void setup() {
  Serial.begin(9600);
  // attach the intrupt sub routine to ROTARY_ENCODER_PIN_A
  attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_PIN_A), isr, LOW);
  // setup the dc load and set the initial mode to Constant Current(1)
  dcLoad.setup(1,3000);
}

void loop() {
  dcLoad.run();
}
