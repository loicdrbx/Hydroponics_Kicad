/*
HYDROPONICS CODE REV 0.1:
List of things to do:
Switch functions from all using global variables to passing by pointer,
Move graphics/menu structure code to it's own header file
Optimize dose pump functions to use less memory (one function rather than two separate ones per pump)
Move static strings into PROGMEM to reduce RAM consumption


*/



#include <Arduino.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <SoftwareSerial.h>


//pin definitions
#define DISPLAY_RESET 13
#define MAIN_PUMP_OUT 12
#define DOSING_PUMP_2 11
#define DOSING_PUMP_1 10
#define LED_STRIP_OUT 9
#define AUTO_TOPOFF_OUT 8
#define FLOAT_SWITCH_IN 7
#define AUX_BTTN_IN 6
#define ENCODER_SW_IN 5
#define ENCODER_B_IN 4
#define ENCODER_A_IN 3
#define RGB_LED_DOUT 2
#define TEMP_SENSE_IN A1
#define PH_IN A0

#define RTC_READ_ADDR 0x51
#define RTC_WRITE_ADDR 0x51
#define RTC_SECODS_REG 0x02
#define RTC_HOURS_REG 0x04

#define DEBUG

#define TOPLEVEL_MENU 1
#define PUMP_SETTINGS_MENU 2
#define LIGHTING_MENU 3
#define PH_MENU 4
#define CLOCK_MENU 5
#define SETTINGS_MENU 6
#define MAIN_PUMP_MENU 11
#define AUTO_TOPOFF_MENU 12
#define DOSE_PUMP_1_MENU 13
#define DOSE_PUMP_2_MENU 14
#define DOSE_PUMP_CALIBRATION_MENU 15

#define LOOP_COUNT_1_SECOND 27000     //approximate number of main loop cycles for one second
#define REFERENCE_PH 7
#define DOSE_VOLUME_TEST 100            //in 1/10? of a milliliter

#define HOURS_IN_WEEK 168
#define MINUTES_IN_WEEK 10080

char parseSerialInput(void);          //gets user input from the keyboard
char parseEncoderInput(void);         //translates encoder pulses into menu commands
uint8_t bcdToDec(uint8_t);            //converts the BCD format output of the RTC to decimal
uint8_t decToBCD(uint8_t);            //converts the decimal time format to BCD to set the RTC
uint8_t getLSB(uint16_t);             //gets the LSB of a given 16 bit number
uint8_t getMSB(uint16_t);             //gets the MSB of a given 16 bit number

void refreshDisplay(void);            //refreshes the display, this should be called every cycle
void navigateMenu(char);              //parses navigation commands for the menu structure
void parseCursorPos(char,uint8_t, uint8_t);
int changeVariable(char, uint32_t, uint16_t, uint16_t); // circularly increments or decrements a variable, with a set min and max
void changeTime(char, uint16_t*);                        //changes hours then minutes...

void displayToplevelMenu(uint8_t);
void displayStatusMenu(void);
void displayPumpMenu(uint8_t);        //show top level pump menu
void displayLightingMenu(uint8_t);    //show lighting settings menu
void displayClockMenu(uint8_t);       //show the clock settings menu
void displayPHMenu(uint8_t);          //show the PH settings menu
void displayMainPumpMenu(uint8_t);    //show the main pump settings menu
void displayDosePump1Menu(uint8_t);   //show dosing pump 1 settings menu
void displayDosePump2Menu(uint8_t);   //show dosign pump 2 settings menu
//void displaySettingsMenu(uint8_t);  //show the settings menu (save settings, reset to default)
void displayDosePumpCalibrationMenu(uint8_t); //Display the calibration menu for the dosing pumps

void refreshDisplay(uint8_t,uint8_t); //refreshes the display, given inputs are current menu and cursor position
void periodicRefresh(void);           //updates variables (ph, time, temperature), and refreshes the display

void getTimeofDay(void);              //retreive the current time of day from the RTC
void setTimeofDay(uint8_t, uint8_t);  //set the current time of day from the RTC
void setTimeofWeek(uint8_t);          //set the day of the week

void readTemperature(void);           //reads the temperature from the DS18B20, stores it in two 8-bit variables, one for the integer part, and one for one decimal place
void readPH(void);                    //reads the current PH from the sensor, stores it in two 8-bit variables, one for the integer part, and one for one decimal place

void setupIO(void);                   //pinMode stuff

void resetVariables(void);
void readEEPROMVariables(void);       //get any variables from memory that have been stored during a power outage
void writeEEPROMVariables(void);      //set values of variables in memory to keep data in the event of a power outage

void updateLights(void);              //turns the lights on or off, changes Brightness
void updateMainPump(void);            //turns the main pump on or off based on the current time_minutes
void updateDose1(void);               //delivers a dose to the system if one is required
void updateDose1Interval(void);       //updates the dosing volume and interval for dose pump 1
void updateDose2(void);               //delivers a dose to the system if one is require
void updateDose2Interval(void);       //updates the dosing volume and interval for dose pump 2
void setupDoses(void);                //sets the remaining dose volume for the day at the start of every day
void manualDose1(uint16_t);           //manually dose a certain volume
void changeInterval(char,uint16_t);   //calculates allowable intervals based on dose volume
void calibrateDosePumps(void);        //calibrates the dosing pump by timing the duration taken to drain 100ml through one of the pumps

void deliverDose(uint8_t,uint16_t);   //delivers a dose of a specified volume to a specified pump

void calibratePH(void);               //calibrates the PH of the current solution

char* getTimeString(uint16_t);        //returns a HH MM 24 hour time format string when given hours and minutes
void printVars(void);                 //used for debug

void debug(void);                     //does debug things...

//DISPLAY OBJECT CONSTRUCTOR
//SSD1305: Display controller
//128x64: Resolution
//ADAFRUIT: Vendor
//1 : framebuffer size (1=128bytes pagebuffer, 2=256bytes pagebuffer, F=full 512bytes framebuffer)
//U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C(rotation, [reset [, clock, data]])
U8G2_SSD1305_128X64_ADAFRUIT_2_HW_I2C display(U8G2_R0, DISPLAY_RESET);

//neopixel(single)
Adafruit_NeoPixel rgbLed = Adafruit_NeoPixel(1,RGB_LED_DOUT,NEO_GRB + NEO_KHZ800);

//temperature sensor and onewire initialize
OneWire oneWire(A1);
DallasTemperature tempSensor(&oneWire);

//GLOBAL VARIABLES:
uint16_t seconds = 0;          //time, seconds
uint16_t minutes = 0;          //time, minutes
uint16_t hours = 0;            //time, hours
uint8_t days = 0;              //time, days
uint16_t time_minutes = 720;   //maximum of 1440, represents the time of day in minutes only
uint8_t hours_ctr = 0;
uint16_t minutes_ctr1 = 0;     //increments every minute, used to keep track of when to dose pump1
uint16_t minutes_ctr2 = 0;     //increments every minute, used to keep track of when to dose pump 2

uint16_t temperature = 512;    //temperature represented as an int
uint8_t temperature_int = 0;   //the integer part of the temperature variable
uint8_t temperature_frac = 0;  //the fractional part of the temperature variable

uint16_t ph = 0;               //maximum of 1024. PH represented as 0->0, 14->1024
uint8_t ph_int = 0;
uint8_t ph_frac = 0;
int16_t ph_offset = 0;         //offset calibration value for the output offset voltage of the amplifier
uint8_t cal_solution_ph = 7;   //PH of the calibration solution

//variables related to the encoder
uint8_t encoder_state;
uint8_t encoder_last_state;
uint8_t encoder_a;
uint8_t encoder_b;
uint8_t encoder_aLast;
uint8_t encoder_bLast;
uint8_t encoder_button;
int8_t encoder_pulsectr = 0;   //counts the pulses from the encoder to determine whether the command is up or down (encoder does two pulses per detent)
uint8_t buttonPressed = 0;     //command to tell if button has been pressed by user

//LIGHT variables
uint8_t led_brightness = 100;
uint16_t led_on_time = 360;    //time of day that the LED strip turns on
uint16_t led_off_time = 1080;  //time of day that the LED strip turns off
uint8_t led_enabled = 1;       //LED strip enable

//Main Pump Variables:
uint8_t main_pump_on_interval = 1;    //ON-time of the main pump in minutes
uint8_t main_pump_off_interval = 1;   //OFF-time of the main pump in minutes
uint8_t main_pump_enabled = 1;        //is the main pump enabled?
uint16_t main_pump_last_time;         //records last time that the pump state changed
uint8_t pumpState = 0;                //current pump state (on/off)

//auto topoff pump:
uint8_t auto_topoff_enabled = 1;

uint8_t doses_have_been_set = 0;      //have the doses for the day been scheduled yet?

//dose pump 1 settings:
uint16_t dose1_volume = 15;    //total volume to be dosed for the first dosing Pumpuint16
uint16_t dose1_vol_per_dose;     //volume per dose (in 1/10 of a ml)
uint16_t dose1_interval = 0;   //time in minutes between doses
uint8_t dose1_enabled = 1;    //is dosing pump 1 enabled?
uint16_t dose1_last_time;     //when was the last dose delivered (hours)

//dose pump 2 settings:
uint16_t dose2_volume = 15; //total volume to be dosed for the first dosing Pump
uint16_t dose2_vol_per_dose;     //volume per dose (in 1/10 of a ml)
uint16_t dose2_interval = 0;  //time in minutes between doses
uint8_t dose2_enabled = 1;    //is dosing pump 1 enabled?
uint16_t dose2_last_time;     //when was the last dose delivered (hours)

uint8_t dosePumpCalState = 0;     //calibration state for the dosing pump (is the pump ON or OFF)
uint8_t dose_pump_scale_factor = 89; //scaling factor for dosing pump dose length calibration

//Menu navigation global variables:
uint8_t cursor = 0;                     //used for menu navigation horizontally
uint8_t menuLevel = 1;                  //used for menu navigation vertically
uint8_t editingVariable = 0;            //used to keep track of navigation button context (edit a variable or change menu selection)
uint8_t currentMenu = TOPLEVEL_MENU;    //for keeping track of the current menu that is being occupied
char command = '\0';                    //command variable for parsing the user input to the controller
char lastCommand = '\0';

char stringBuffer[50];    //used as an intermediary for the sprintf function

int8_t position = 0;  //test var for encoder position
uint64_t loopCtr = 0;

uint8_t i;
int t;
int deltat;

void setup()
{
  Wire.begin();           //initialize I2C bus
  display.begin();        //start the display
  Serial.begin(9600);     //start serial port
  refreshDisplay(currentMenu,cursor); //display the main screen
  //setTimeofDay(0,0);
  getTimeofDay();
  rgbLed.begin();
  rgbLed.setBrightness(1);
  rgbLed.show();
  setupIO();
  readEEPROMVariables();
  main_pump_last_time = time_minutes;
  updateDose1Interval();
  updateDose2Interval();
}

//LOOP TIME BENCHMARKING:
//refreshDisplay takes approximately 125ms to update
void loop()
{
  lastCommand = command;
  command = parseEncoderInput();
  //command = parseSerialInput();
  debug();
  if(command)
  {
    //Serial.print(command);
    navigateMenu(command);
    refreshDisplay(currentMenu,cursor);
    updateLights();
    loopCtr = 0;
  }
  command = '\0';
  if(loopCtr>=(uint64_t)2*LOOP_COUNT_1_SECOND) //updates main variables and pump status every 2 seconds
  {
    periodicRefresh();
    loopCtr = 0;
  }
  loopCtr++;
}

//FUNCTIONS (most of these use global variables, I would like to clean that up in the future if given time)

void debug(void)
{
  char input = Serial.read();
  if(input=='+')
    minutes_ctr1++;
  if(input=='-')
    minutes_ctr1--;
}

void periodicRefresh() //refreshes the display, reads variables from the sensors (PH/)
{
  readTemperature();
  readPH();
  getTimeofDay();
  refreshDisplay(currentMenu,cursor);
  updateLights();
  updateMainPump();
  updateDose1();
  updateDose2();
  //sprintf(stringBuffer,"dd:%i, hh:%i, mm:%i, ss:%i",days, hours, minutes, seconds);
  //`Serial.println(stringBuffer);
}

//gets the temperature and stores it in the corresponding global variables (up to the first decimal place)
void readTemperature(void)
{
  tempSensor.requestTemperatures();
  float floatTemp = tempSensor.getTempCByIndex(0);
  temperature_int = (uint8_t)floor(floatTemp);
  temperature_frac = (uint8_t)floor(10*(floatTemp-temperature_int)); //get only the first decimal place
}

void readPH(void) //read the PH values, and map the output from 0-1024, to one integer place, and one decimal place
{
  uint16_t ph_raw = analogRead(PH_IN);
  ph_raw = ph_raw + ph_offset;
  //dividing 1024 into 14 sections (ph goes from 0 to 14)
  int16_t vprobe = map(ph_raw,0,1023,-2500,2500);
  float ph_float = REFERENCE_PH + ((float)(-vprobe)*0.01708389);
  ph_int = (uint8_t)floor(ph_float);
  ph_frac = (uint8_t)floor(10*(ph_float-ph_int)); //get only the first decimal place
}

void calibratePH(void)
{
  uint16_t raw_input = analogRead(PH_IN); //the input voltage should be exactly zero for a PH of 7
  ph_offset = 512-raw_input;    //thus, any value that we get here will be the offset that we need to null
}

void getTimeofDay()
{
  Wire.beginTransmission(0x51); //begin a transmission with the RTC, move the register pointer to VL_seconds
  Wire.write(0x02);
  Wire.endTransmission();
  Wire.requestFrom(0x51,5);                     //request the first 5 bits from the RTC (seconds, minutes, hours, day of month, day of week)
  seconds = bcdToDec(Wire.read()&0b01111111);   //seconds
  minutes = bcdToDec(Wire.read()&0b01111111);   //minutes
  hours =   bcdToDec(Wire.read()&0b00111111);   //hours
  Wire.read();                                  //day of month (don't care)
  days =    bcdToDec(Wire.read()&0b00000111);   //day of week
  if(time_minutes != 60*hours+minutes)
  {
    minutes_ctr1++;
    minutes_ctr2++;
  }
  time_minutes = 60*hours + minutes;
  //sprintf(stringBuffer,"Minutes1: %i, Interval1: %i",minutes_ctr1,dose1_interval);
  //Serial.println(stringBuffer);

}

void setTimeofDay(uint8_t hours, uint8_t minutes)
{
  Wire.beginTransmission(RTC_WRITE_ADDR);
  Wire.write(0x02);
  Wire.write(decToBCD(0));        //we don't care about seconds
  Wire.write(decToBCD(minutes));  //write minutes
  Wire.write(decToBCD(hours));    //write hours
  Wire.endTransmission();
}

void setDayofWeek(uint8_t dayOfWeek)
{
  Wire.beginTransmission(RTC_WRITE_ADDR);
  Wire.write(0x06);
  Wire.write(dayOfWeek);
  Wire.endTransmission();
}

void updateLights()
{
  if((time_minutes >= led_on_time)&&led_enabled)
  {
    analogWrite(LED_STRIP_OUT,map(led_brightness,0,100,0,255));
  }
  if((time_minutes >= led_off_time)||!led_enabled)
  {
    analogWrite(LED_STRIP_OUT,0);
  }
}

void updateMainPump()
{
  if(pumpState) //if pump is currently on
  {
    if(time_minutes - main_pump_last_time >= main_pump_on_interval) //if the elapsed interval has passed
    {
      pumpState = 0;
      main_pump_last_time = time_minutes;
    }
  }
  else if(!pumpState) //if pump is currently OFF
  {
    if((time_minutes - main_pump_last_time >= main_pump_off_interval)) //if the elapsed off interval has passed, and the main pump is enabled
    {
      pumpState = 1;
      main_pump_last_time = time_minutes;
    }
  }
  if(main_pump_enabled)
    digitalWrite(MAIN_PUMP_OUT,pumpState);
  else
    digitalWrite(MAIN_PUMP_OUT,0);
}

void updateDose1(void)
{
  //Serial.print("checking ");
  if(minutes_ctr1 == dose1_interval)
  {
    deliverDose(1,dose1_vol_per_dose);
    minutes_ctr1 = 0;
    #ifdef DEBUG
      Serial.print("Dosing:  ");
      Serial.print(dose1_vol_per_dose);
      Serial.print("Time: ");
      getTimeString(time_minutes);
      Serial.print(stringBuffer);
      Serial.print("Interval: ");
      Serial.print(dose1_interval);
    #endif
  }
}


void updateDose1Interval(void)
{
  if(dose1_volume <=70 )
  {
    dose1_interval = 1440; //24 hours
  }
  if(dose1_volume>70 && dose1_volume<=80)
  {
    dose1_interval = 1260; //21 hours
  }
  if(dose1_volume>80 && dose1_volume<=120)
  {
    dose1_interval = 840;  //14 hours
  }
  if(dose1_volume>120 && dose1_volume <=140)
  {
    dose1_interval = 720;  //12 hours
  }
  if(dose1_volume>140 && dose1_volume<=210)
  {
    dose1_interval = 480; //8 hours
  }
  if(dose1_volume>210 && dose1_volume<=240)
  {
    dose1_interval = 420; //7 hours
  }
  if(dose1_volume>240 && dose1_volume<=280)
  {
    dose1_interval = 360; //6 hours
  }
  if(dose1_volume>280 && dose1_volume<=420)
  {
    dose1_interval = 240; //4 hours
  }
  if(dose1_volume>420 && dose1_volume<=560)
  {
    dose1_interval = 180; //3 hours
  }
  if(dose1_volume>560 && dose1_volume<=840)
  {
    dose1_interval = 120; //2 hours
  }
  if(dose1_volume>840)
  {
    dose1_interval = 60; // 1 hour
  }
  dose1_vol_per_dose = round((float)((10*dose1_volume)/(MINUTES_IN_WEEK/dose1_interval)));
  #ifdef DEBUG
  Serial.print("Dose1 Vol/Dose: ");
  Serial.print(dose1_vol_per_dose);
  Serial.print(" Interval: ");
  Serial.print(dose1_interval);
  Serial.print(" True Volume: ");
  Serial.println((MINUTES_IN_WEEK/dose1_interval)*dose1_vol_per_dose);
  #endif
}


void manualDose1(uint16_t volume) //volume is given in 1/10 of a ml
{
  uint16_t scaleNum = dose_pump_scale_factor;
  uint8_t scaleDen = 1;
  uint32_t doseTime = ((uint32_t)volume * scaleNum)/scaleDen;
  //Serial.print("Dosing: ");
  //Serial.println(doseTime);
  digitalWrite(DOSING_PUMP_1,1);
  delay(doseTime);
  digitalWrite(DOSING_PUMP_1,0);
}

void manualDose2(uint16_t volume)
{
  uint16_t scaleNum = dose_pump_scale_factor;
  uint8_t scaleDen = 1;
  uint32_t doseTime = (volume * scaleNum)/scaleDen;
  digitalWrite(DOSING_PUMP_2,1);
  delay(doseTime);
  digitalWrite(DOSING_PUMP_2,0);
}

void updateDose2Interval(void)
{
  if(dose2_volume <=70 )
  {
    dose2_interval = 1440; //24 hours
  }
  if(dose2_volume>70 && dose2_volume<=80)
  {
    dose2_interval = 1260; //21 hours
  }
  if(dose2_volume>80 && dose2_volume<=120)
  {
    dose2_interval = 840;  //14 hours
  }
  if(dose2_volume>120 && dose2_volume <=140)
  {
    dose2_interval = 720;  //12 hours
  }
  if(dose2_volume>140 && dose2_volume<=210)
  {
    dose2_interval = 480; //8 hours
  }
  if(dose2_volume>210 && dose2_volume<=240)
  {
    dose2_interval = 420; //7 hours
  }
  if(dose2_volume>240 && dose2_volume<=280)
  {
    dose2_interval = 360; //6 hours
  }
  if(dose2_volume>280 && dose2_volume<=420)
  {
    dose2_interval = 240; //4 hours
  }
  if(dose2_volume>420 && dose2_volume<=560)
  {
    dose2_interval = 180; //3 hours
  }
  if(dose2_volume>560 && dose2_volume<=840)
  {
    dose2_interval = 120; //2 hours
  }
  if(dose2_volume>840)
  {
    dose2_interval = 60; // 1 hour
  }
  dose2_vol_per_dose = round((float)((10*dose2_volume)/(MINUTES_IN_WEEK/dose2_interval)));
  #ifdef DEBUG
  Serial.print("Dose2 Vol/Dose: ");
  Serial.print(dose2_vol_per_dose);
  Serial.print(" Interval: ");
  Serial.print(dose2_interval);
  Serial.print(" True Volume: ");
  Serial.println((MINUTES_IN_WEEK/dose2_interval)*dose2_vol_per_dose);
  #endif
}

void updateDose2(void)
{
  //Serial.print("checking ");
  if(minutes_ctr2 == dose2_interval)
  {
    deliverDose(1,dose2_vol_per_dose);
    minutes_ctr2 = 0;
    //Serial.print("Dosing... ");
  }
}

void deliverDose(uint8_t pump, uint16_t volume) //volume is in 1/10 of a ml, so 10 would be 1ml, 15 would be 1.5ml, etc.
{
  uint16_t scaleNum = dose_pump_scale_factor;
  uint8_t scaleDen = 1;
  uint16_t doseTime = (volume * scaleNum)/scaleDen;

  if(pump==1&&dose1_enabled)
  {
    digitalWrite(DOSING_PUMP_1,1);
    delay(doseTime);
    digitalWrite(DOSING_PUMP_1,0);
  }

  if(pump==2&&dose2_enabled)
  {
    digitalWrite(DOSING_PUMP_2,1);
    delay(doseTime);
    digitalWrite(DOSING_PUMP_2,0);
  }
}

void calibrateDosePumps()
{
  command = '\0';
  dosePumpCalState = 1;
  refreshDisplay(currentMenu,cursor);
  uint32_t initialTime = millis();
  digitalWrite(DOSING_PUMP_1,1);
  while(command != 'E')
    command = parseEncoderInput();      //wait in this loop until the user presses the button again
  digitalWrite(DOSING_PUMP_1,0);
  uint32_t finalTime = millis();
  refreshDisplay(currentMenu,cursor);
  uint32_t deltaTime = finalTime - initialTime;
  #ifdef DEBUG
    Serial.println(deltaTime);
  #endif
  dose_pump_scale_factor = deltaTime / 1000; //deltaTime is the # of ms taken to dose 100ml, our scale factor is the # of ms to dose 0.1ml.
  #ifdef DEBUG
    Serial.println(dose_pump_scale_factor);
  #endif
  command = '\0';
}

void readEEPROMVariables()
{
  uint16_t address = 0;
  led_brightness = EEPROM.read(address);
  address++;
  led_on_time = EEPROM.read(address); //get the lsb
  address++;
  led_on_time = led_on_time + (EEPROM.read(address)<<8); //get the MSB
  address++;
  led_off_time = EEPROM.read(address); //get the lsb
  address++;
  led_off_time = led_off_time + (EEPROM.read(address)<<8); //get the MSB
  address++;
  led_enabled = EEPROM.read(address);
  address++;

  main_pump_on_interval = EEPROM.read(address);
  address++;
  main_pump_off_interval = EEPROM.read(address);
  address++;
  main_pump_enabled = EEPROM.read(address);
  address++;

  //minutes_ctr = EEPROM.read(address);
  //address++;
  //minutes_ctr = minutes_ctr + (EEPROM.read(address)<<8);
  //address++;

  dose1_volume = EEPROM.read(address); //get the lsb
  address++;
  dose1_volume = dose1_volume + (EEPROM.read(address)<<8); //get the MSB
  address++;
  dose1_interval = EEPROM.read(address); //get the LSB
  address++;
  dose1_interval = dose1_interval + (EEPROM.read(address)<<8); //get the MSB
  address++;
  dose1_enabled = EEPROM.read(address);
  address++;
  dose1_last_time = EEPROM.read(address);
  address++;
  dose1_last_time = dose1_last_time + (EEPROM.read(address)<<8);
  address++;

  dose2_volume = EEPROM.read(address); //get the lsb
  address++;
  dose2_volume = dose2_volume + (EEPROM.read(address)<<8); //get the MSB
  address++;
  dose2_interval = EEPROM.read(address);
  address++;
  dose2_interval = dose2_interval + (EEPROM.read(address)<<8); //get the MSB
  address++;
  dose2_enabled = EEPROM.read(address);
  address++;
  dose2_last_time = EEPROM.read(address);
  address++;
  dose2_last_time = dose2_last_time + (EEPROM.read(address)<<8);
  address++;

}

void writeEEPROMVariables()
{

  uint16_t address = 0;
  //LED variables
  EEPROM.write(address,led_brightness);
  address++;
  EEPROM.write(address,getLSB(led_on_time));
  address++;
  EEPROM.write(address,getMSB(led_on_time));
  address++;
  EEPROM.write(address,getLSB(led_off_time));
  address++;
  EEPROM.write(address,getMSB(led_off_time));
  address++;
  EEPROM.write(address,led_enabled);
  address++;

  //main pump variables
  EEPROM.write(address,main_pump_on_interval);
  address++;
  EEPROM.write(address,main_pump_off_interval);
  address++;
  EEPROM.write(address,main_pump_enabled);
  address++;

  //EEPROM.write(address,getLSB(minutes_ctr));
  //address++;
  //EEPROM.write(address,getMSB(minutes_ctr));
  //address++;

  //dose pump 1 variables:
  EEPROM.write(address,getLSB(dose1_volume));
  address++;
  EEPROM.write(address,getMSB(dose1_volume));
  address++;
  EEPROM.write(address,getLSB(dose1_interval));
  address++;
  EEPROM.write(address,getMSB(dose1_interval));
  address++;
  EEPROM.write(address,dose1_enabled);
  address++;
  EEPROM.write(address,getLSB(dose1_last_time));
  address++;
  EEPROM.write(address,getMSB(dose1_last_time));
  address++;

  //dose pump 2 variables:
  EEPROM.write(address,getLSB(dose2_volume));
  address++;
  EEPROM.write(address,getMSB(dose2_volume));
  address++;
  EEPROM.write(address,getLSB(dose2_interval));
  address++;
  EEPROM.write(address,getMSB(dose2_interval));
  address++;
  EEPROM.write(address,dose2_enabled);
  address++;
  EEPROM.write(address,getLSB(dose2_last_time));
  address++;
  EEPROM.write(address,getMSB(dose2_last_time));


}

void resetVariables(void)
{
  //code this later
}

uint8_t bcdToDec(uint8_t num)
{
  num = num/16*10+num%16;
  return(num);
}

uint8_t decToBCD(uint8_t num)
{
  num= num/10*16+num%10;
  return(num);
}

uint8_t getLSB(uint16_t num)
{
  uint8_t lsb = num&0b0000000011111111;
  return(lsb);
}

uint8_t getMSB(uint16_t num)
{
  uint8_t msb = ((num&0b1111111100000000)>>8); //mask and right shift by 8
  return(msb);
}

char parseSerialInput(void)     //parses the input at the serial monitor
{
  char command='\0';
  char input = Serial.read();
  //Serial.print("echo input is: ");
  //Serial.print(input);
  //Serial.print(", command is: ");
  if(input == '5')
  {
    command = 'E';
    //Serial.print("ENTER");
  }
  else if(input == '2')
  {
    command = 'D';
    //Serial.print("DOWN");
  }
  else if(input == '8')
  {
    command = 'U';
    //Serial.print("UP");
  }
  //Serial.print("\n");
  return(command);
}

void setupIO(void)
{
  pinMode(MAIN_PUMP_OUT,OUTPUT);
  pinMode(DOSING_PUMP_1,OUTPUT);
  pinMode(DOSING_PUMP_2,OUTPUT);
  pinMode(AUTO_TOPOFF_OUT,OUTPUT);
  pinMode(FLOAT_SWITCH_IN,INPUT);
}

char parseEncoderInput(void)
{
  char command = '\0';
  encoder_last_state = encoder_state;
  encoder_a = digitalRead(ENCODER_A_IN);        //read the state of the phases
  encoder_b = digitalRead(ENCODER_B_IN);
  encoder_state = (encoder_a<<1) + encoder_b;   //LSB of encoder_state is b, MSB is a
  if(encoder_state != encoder_last_state)       //if there has been a change of state, then update according to which direction the encoder has been turned
  {
    if(encoder_last_state==1)
    {
      if(encoder_state==3)
        encoder_pulsectr--;
      if(encoder_state==0)
        encoder_pulsectr++;
    }

    if(encoder_last_state==0)
    {
      if(encoder_state==1)
        encoder_pulsectr--;
      if(encoder_state==2)
        encoder_pulsectr++;
    }

    if(encoder_last_state==2)
    {
      if(encoder_state==0)
        encoder_pulsectr--;
      if(encoder_state==3)
        encoder_pulsectr++;
    }

    if(encoder_last_state==3)
    {
      if(encoder_state==2)
        encoder_pulsectr--;
      if(encoder_state==1)
        encoder_pulsectr++;
    }
  }
  if(encoder_pulsectr==2)         //since the encoder has two pulses per detent, we need to count two full pulses in either direction before issuing a command to the system
  {
    command = 'U';
    encoder_pulsectr = 0;
  }
  if(encoder_pulsectr==-2)        //-2 for the other direction
  {
    command='D';
    encoder_pulsectr = 0;
  }

  if(digitalRead(ENCODER_SW_IN))  //if the button has been pressed, debounce, and register a button press as the command. Button press will override a cursor movement
  {
    delay(200);
    command = 'E';
  }
  return(command);
}

void printVars(void)
{
  sprintf(stringBuffer,"A=%i,B=%i,SW=%i,Ap=%i,Bp=%i",encoder_a,encoder_b,encoder_button,encoder_aLast,encoder_bLast);
  Serial.println(stringBuffer);
}

char* getTimeString(uint16_t timeinminutes)  //returns a HH MM 24 hour time format string when time in minutes (0-1440)
{
  uint8_t hours = timeinminutes/60;
  uint8_t minutes = timeinminutes%60;
  if(minutes>9)
  {
    sprintf(stringBuffer,"%i:%i",hours,minutes);
  }
  else
    sprintf(stringBuffer,"%i:0%i",hours,minutes);
  return(stringBuffer);
}

void parseCursorPos(char command,uint8_t min, uint8_t max) //increments/decrements the cursor position, while keeping it within min/max boundary positions
{
  if(command=='U')  //circular cursor behaviour, will loop from max to min/min to max on increment/decrement
    cursor++;
  if(command=='D')
  {
    if(!(cursor-min)) //if at minimum position, and command is decrement, roll over to maximum value
      cursor=max-1;
    else
      cursor--;
  }
  cursor = cursor%max; //if at maximum value, and command is increment, roll over back to minimum value
}

int changeVariable(char command, uint32_t variable, uint16_t min, uint16_t max) //returns a value that will circularly add/subtract and stay within the min/max bounds
{
  if(command=='U')
  {
    variable++;
    if(variable>max)
      variable = min;
  }
  if(command=='D')
  {
    if(!(variable-min)) //if variable is at minimum value, and command is decrement, roll over to maximum value
      variable=max;
    else
      variable--;
  }
  return(variable);
}

void changeTime(char command, uint16_t *variable) //this is kind of a hack... re-do it later if given the opportunity
{
  uint8_t finished = 0;
  //*variable = 1440/2;
  uint8_t editingHours = 1;
  command = '\0';
  while(!finished)                       //change hours first and then minutes
  {
    //Serial.println("Loop");
    if(command == 'U')
    {                     //increment number of minutes by 60 (editing hours)
      if(editingHours)
        *variable = (*variable + 60);
      else
        *variable = (*variable + 1);
      if(*variable>=1440)
        *variable = *variable - 1440;
      refreshDisplay(currentMenu, cursor);
      command = '\0';
    }
    if(command == 'D')
    {
      if(editingHours)
      {
        if(*variable>=60)
          *variable = (*variable - 60);
        else
          *variable = *variable + 1380;
      }
      else
        if(*variable > 1)
          *variable = (*variable - 1);
      refreshDisplay(currentMenu, cursor);
      command = '\0';
    }
    if(command == 'E')
    {
      if(editingHours)
        editingHours = 0;
      else
        finished = 1;
    }
    command = parseEncoderInput();
  }
  editingVariable = 0;
}


void refreshDisplay(uint8_t current_menu, uint8_t cursorPos)  //update the information on the display. Call this as sparingly as possible, as it is a relatively slow funtion
{
  switch(current_menu)
  {
    case TOPLEVEL_MENU:
      displayToplevelMenu(cursorPos);
    break;
    case PUMP_SETTINGS_MENU:
      displayPumpMenu(cursorPos);
    break;
    case LIGHTING_MENU:
      displayLightingMenu(cursorPos);
    break;
    case PH_MENU:
      displayPHMenu(cursorPos);
    break;
    case CLOCK_MENU:
      displayClockMenu(cursorPos);
    break;
    case SETTINGS_MENU:
      //displaySettingsMenu(cursorPos);
    break;
    case MAIN_PUMP_MENU:
      displayMainPumpMenu(cursorPos);
    break;
    case AUTO_TOPOFF_MENU:
      displayMainPumpMenu(cursorPos);
    break;
    case DOSE_PUMP_1_MENU:
      displayDosePump1Menu(cursorPos);
    break;
    case DOSE_PUMP_2_MENU:
      displayDosePump2Menu(cursorPos);
    break;
    case DOSE_PUMP_CALIBRATION_MENU:
      displayDosePumpCalibrationMenu(cursorPos);
    break;
  }
}

void navigateMenu(char command) //master menu navigation function
{
  buttonPressed = 0;    //command to tell if button has been pressed by user

  if(command=='E')  //command was enter
  {
    buttonPressed = 1;
  }

  if(currentMenu==TOPLEVEL_MENU)  //top menu level
  {
    parseCursorPos(command,0,5);  //rotate cursor through values 0 to 5
    switch(cursor)
    {
      case 0:                     //status menu is being hovered over
        displayStatusMenu();
      break;
      case 1:                     //pump menu is being hovered over
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU; //if button is pressed, enter the pump settings menu
          cursor = 0;
        }
      break;
      case 2:                     //Lighting Menu is being hovered over
        if(buttonPressed)
        {
          currentMenu = LIGHTING_MENU; //if button is pressed, enter the lighting menu
          cursor = 0;
        }
      break;
      case 3:                     //PH menu is being hovered over
        if(buttonPressed)
        {
          currentMenu = PH_MENU;  //if button is pressed, enter the PH menu
          cursor = 0;
        }
      break;
      case 4:                     //Clock menu is being hovered over
        if(buttonPressed)
        {
          currentMenu = CLOCK_MENU;  //if button is pressed, enter the clock settings menu
          cursor = 0;
        }
      break;
      case 5:
        if(buttonPressed)
        {
          currentMenu = SETTINGS_MENU;
          cursor = 0;
        }
    }
    buttonPressed = 0;              //reset button push
  }

  if(currentMenu == PUMP_SETTINGS_MENU) //if the current menu is the pump settings menu
  {
    parseCursorPos(command,0,5);  //rotate cursor through 5 positions
    switch(cursor)
    {
      case 0: //hovering over Main pump settings menu
        if(buttonPressed)
        {
          currentMenu = MAIN_PUMP_MENU; //if button is pressed, enter the main pump settings menu
          cursor = 0;
        }
      break;
      case 1: //hovering over auto topup pump menu
        if(buttonPressed)
        {
          currentMenu = MAIN_PUMP_MENU; //will be auto topoff once implemented
          cursor = 0;
        }
      break;
      case 2: //hovering over dosing pump 1 menu
        if(buttonPressed)
        {
          currentMenu = DOSE_PUMP_1_MENU; //if button is pressed, enter the dosing pump 1 settings menu
          cursor = 0;
        }
      break;
      case 3: //hovering over dosing pump 2 menu
        if(buttonPressed)
        {
          currentMenu = DOSE_PUMP_2_MENU; //if butto is pressed, enter the dosing pump 2 settings menu
          cursor = 0;
        }
      break;
      case 4: //hovering over back button
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;    //return to main menu if back button is pressed, reset cursor position
          cursor = 1;
        }
      break;

    }
    buttonPressed = 0;
  }

  if(currentMenu == LIGHTING_MENU) //if current menu is led lighting settings menu
  {
    if(!editingVariable)            //if we are not currently editing a variable, move the cursor if given the command
      parseCursorPos(command,0,5);
    switch(cursor)
    {
      case 0:                       // hovering over the LED brightness button
          if(buttonPressed)         //if the button is pressed, either enter editing mode, or exit editing mode
            editingVariable=!editingVariable;
          if(editingVariable)       //if we are in editing mode, change the brightness value
            led_brightness = changeVariable(command,led_brightness,0,100);
      break;
      case 1:                       //hovering over the LED on-time button
        if(buttonPressed)           //if the button is pressed, enter or exit editing mode
          editingVariable=!editingVariable;
        if(editingVariable)         //if we are in editing mode, change the  led_on_time variable
          changeTime(command, &led_on_time);
      break;
      case 2:                       //hovering over the LED off-time button
        if(buttonPressed)           //if the button is pressed, enter or exit editing mode
          editingVariable=!editingVariable;
        if(editingVariable)         //if we are in editing mode, change the led_off_time variable
          changeTime(command, &led_off_time);
      break;
      case 3:                       //if we are in editing mode, change the led_enabled variable
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          led_enabled = changeVariable(command,led_enabled,0,1);
      break;
      case 4:
        if(buttonPressed)           //if we are hovering over the back button, and the button is pressed, return to the top level menu
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 2;
          writeEEPROMVariables(); //save changes
        }
      break;
    }
  }

  if(currentMenu == PH_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,3);
    switch(cursor)
    {
      case 0:
      break;
      case 1:
      if(buttonPressed)
      {
        calibratePH();
      }
      break;
      case 2:
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 3;
          writeEEPROMVariables();
        }
      break;
    }
  }

  if(currentMenu == CLOCK_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,3);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)           //if the button is pressed, enter or exit editing mode
          editingVariable=!editingVariable;
        if(editingVariable)         //if we are in editing mode, change the led_off_time variable
        {
          changeTime(command, &time_minutes);
          minutes = time_minutes%60;
          hours = time_minutes/60;
          setTimeofDay(hours,minutes);
        }
      break;
      case 1:
        if(buttonPressed)
          editingVariable = !editingVariable;
        if(editingVariable)
          days = changeVariable(command,days,0,6);
          setDayofWeek(days);
      break;
      case 2:
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 4;
          writeEEPROMVariables();
        }
      break;
    }
  }
/*
  if(currentMenu == SETTINGS_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,3);
    switch(cursor)
    {
      case 0:               //save current settings
        if(buttonPressed)
        {
          writeEEPROMVariables();
        }
      break;
      case 1:               //reset to default
        if(buttonPressed)
        {
          resetVariables();
        }
      break;
      case 2:               //back button
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 3;
          writeEEPROMVariables();
        }
      break;
    }
  }*/

  if(currentMenu == MAIN_PUMP_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,4);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
        {
          main_pump_on_interval = changeVariable(command,main_pump_on_interval,1,255);
        }
      break;
      case 1:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
        {
          main_pump_off_interval = changeVariable(command,main_pump_off_interval,1,255);
        }
      break;
      case 2:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
        {
          main_pump_enabled = changeVariable(command,main_pump_enabled,0,1);
          updateMainPump();
        }
      break;
      case 3:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 0;
          writeEEPROMVariables();
        }
      break;
    }
  }

  if(currentMenu == AUTO_TOPOFF_MENU) //STILL NEEDS TO BE IMPLEMENTED
  {
    parseCursorPos(command,0,2);
    switch(cursor)
    {
      case 0:
      break;
      case 1:
      break;
      case 2:
      break;
      case 3:
      break;
      case 4:
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 2;
        }
      break;
    }
  }

  if(currentMenu == DOSE_PUMP_1_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,5);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)
        {
          editingVariable=!editingVariable;
          updateDose1Interval();
        }
        if(editingVariable)
          dose1_volume = changeVariable(command,dose1_volume,10,1000);
      break;
      case 1:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose1_enabled = changeVariable(command,dose1_enabled,0,1);
      break;
      case 2:
      if(buttonPressed)
        {
          manualDose1(DOSE_VOLUME_TEST);
        }
      break;
      case 3:
      if(buttonPressed)
      {
        currentMenu = DOSE_PUMP_CALIBRATION_MENU;
        cursor = 0;
        buttonPressed = 0;
      }
      break;
      case 4:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 2;
          writeEEPROMVariables();
        }
      break;
    }
  }


  if(currentMenu == DOSE_PUMP_2_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,5);
    switch(cursor)
    {
      case 0:
      if(buttonPressed)
      {
        editingVariable=!editingVariable;
        updateDose2Interval();
      }
      if(editingVariable)
        dose2_volume = changeVariable(command,dose2_volume,10,1000);
      break;
      case 1:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose2_enabled = changeVariable(command,dose2_enabled,0,1);
      break;
      case 2:
      if(buttonPressed)
        {
          manualDose2(dose2_volume);
        }
      break;
      case 3:
      if(buttonPressed)
      {

      }
      break;
      case 4:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 2;
          writeEEPROMVariables();
        }
      break;
    }
  }

  if(currentMenu == DOSE_PUMP_CALIBRATION_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,2);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)
          calibrateDosePumps();
      break;
      case 1:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 3;
          writeEEPROMVariables();
        }
      break;
    }
  }
}

void displayToplevelMenu(uint8_t cursorPos)
{
  switch(cursorPos)
  {
    case 0:                     //status menu is being hovered over
      displayStatusMenu();
    break;
    case 1:                     //pump menu is being hovered over
      displayPumpMenu(254);
    break;
    case 2:                     //Lighting Menu is being hovered over
      displayLightingMenu(254);
    break;
    case 3:                     //PH menu is being hovered over
      displayPHMenu(254);
    break;
    case 4:                     //Clock menu is being hovered over
      displayClockMenu(254);
    break;
    /*case 5:
      displaySettingsMenu(254); //settings menu is being hovered over
    break;*/
  }
}

void displayStatusMenu() //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Grow Wave V1.0";
  //variables to be displayed

  //graphics position data
  //uint8_t cursorPos = 0;
  //uint8_t backx = 54;
  //uint8_t backy = 60;
  uint8_t optx = 12;
  uint8_t opty = 25;
  uint8_t optyspace = 11;
  //uint8_t boxw = 100;
  //uint8_t boxh = 10;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(18,11,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Time (24h): ");
    getTimeString(time_minutes);
    display.drawStr(optx+63,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"Temperature: ");
    sprintf(stringBuffer,"%i.%i C",temperature_int,temperature_frac);
    display.drawStr(optx+63,opty+optyspace,stringBuffer);
    display.drawStr(optx,opty+2*optyspace,"Current PH: ");
    sprintf(stringBuffer,"%i.%i",ph_int,ph_frac);
    display.drawStr(optx+63,opty+2*optyspace,stringBuffer);
    //display.drawStr(backx,backy,"Back");            //draw the back button (not needed for this menu context)
    //display.setDrawColor(2);
    display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
    //display.setDrawColor(1);
  } while(display.nextPage() );
}


void displayPumpMenu(uint8_t cursorPos)
{
  char menuTitle[] = "Pump Settings";

  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 10;
  uint8_t opty = 19;
  uint8_t optyspace = 10;
  uint8_t boxw = 100;
  uint8_t boxh = 9;

  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(28,10,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Main Pump ");
    display.drawStr(optx,opty+optyspace,"Auto Top-off");
    display.drawStr(optx,opty+2*optyspace,"Dose Pump 1");
    display.drawStr(optx,opty+3*optyspace,"Dose Pump 2");

    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,opty-7,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-7,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+2*optyspace)-7,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty+3*optyspace)-7,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-7,23,boxh);
        break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}


void displayLightingMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Lighting Menu";
  //variables to be displayed
  char state[10];

  if(led_enabled)
    strcpy(state,"Enabled");
  else
    strcpy(state,"Disabled");

  //graphics position data
  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 10;
  uint8_t opty = 20;
  uint8_t optyspace = 10;
  uint8_t boxw = 100;
  uint8_t boxh = 9;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(29,11,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Brightness: ");
    sprintf(stringBuffer,"%i",led_brightness);
    display.drawStr(optx+65,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"Turn on at: ");
    getTimeString(led_on_time);
    display.drawStr(optx+65,opty+optyspace,stringBuffer);
    getTimeString(led_off_time);
    display.drawStr(optx+65,opty+2*optyspace,stringBuffer);
    display.drawStr(optx,opty+2*optyspace,"Turn off at: ");
    display.drawStr(optx,opty+3*optyspace,"Status:");
    display.drawStr(optx+55,opty+3*optyspace,state);
    display.drawStr(backx,backy,"Back");            //draw the back button (not needed for this menu context)
    display.setDrawColor(2);
    //display.setDrawColor(1);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,opty-7,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-7,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+2*optyspace)-7,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty+3*optyspace)-7,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-7,23,boxh);
        break;
    }
  } while(display.nextPage() );
}

void displayClockMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Clock Menu";

  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 15;
  uint8_t opty = 25;
  uint8_t optyspace = 11;
  uint8_t boxw = 100;
  uint8_t boxh = 10;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(29,10,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Clock Time:");
    getTimeString(time_minutes);
    display.drawStr(optx+63,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"Day:");
    sprintf(stringBuffer,"%i",days);
    display.drawStr(optx+63,opty+optyspace,stringBuffer);
    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
      break;
      case 0:
        display.drawBox(optx-2,(opty)-8,boxw,boxh);
      break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
      break;
      case 2:
        display.drawBox(backx-2,backy-8,23,boxh);
      break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}

void displayPHMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "PH Probe Settings";

  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 15;
  uint8_t opty = 25;
  uint8_t optyspace = 11;
  uint8_t boxw = 100;
  uint8_t boxh = 10;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(15,10,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Current PH:");
    sprintf(stringBuffer,"%i.%i",ph_int,ph_frac);
    display.drawStr(optx+60,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"Calibrate Sensor");

    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,opty-8,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
        break;
      case 2:
        display.drawBox(backx-2,backy-8,23,boxh);
        break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}

void displayMainPumpMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Main Pump Settings";
  //variables to be displayed
  char state[10];

  if(main_pump_enabled)
    strcpy(state,"ENABLED");
  else
    strcpy(state,"DISABLED");

  //graphics position data
  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 10;
  uint8_t opty = 20;
  uint8_t optyspace = 10;
  uint8_t boxw = 100;
  uint8_t boxh = 9;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(10,11,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"ON-time(mins): ");
    sprintf(stringBuffer,"%i",main_pump_on_interval);
    display.drawStr(optx+80,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"OFF-time(mins): ");
    sprintf(stringBuffer,"%i",main_pump_off_interval);
    display.drawStr(optx+80,opty+optyspace,stringBuffer);
    display.drawStr(optx,opty+2*optyspace,"Status:");
    display.drawStr(optx+55,opty+2*optyspace,state);
    display.drawStr(backx,backy,"Back");            //draw the back button (not needed for this menu context)
    display.setDrawColor(2);
    //display.setDrawColor(1);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,opty-7,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-7,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+2*optyspace)-7,boxw,boxh);
        break;
      case 3:
        display.drawBox(backx-2,backy-7,23,boxh);
        break;
    }
  } while(display.nextPage() );
}

void displayDosePump1Menu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Dose Pump 1";
  //variables to be displayed
  char state[10];

  if(dose1_enabled)
    strcpy(state,"ENABLED");
  else
    strcpy(state,"DISABLED");

  //graphics position data
  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 10;
  uint8_t opty = 20;
  uint8_t optyspace = 10;
  uint8_t boxw = 105;
  uint8_t boxh = 9;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(27,11,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Volume/wk: ");
    sprintf(stringBuffer,"%i",dose1_volume);
    display.drawStr(optx+69,opty,stringBuffer);
    display.drawStr(optx+90,opty,"ml");
    display.drawStr(optx,opty+optyspace,"Status:");
    display.drawStr(optx+55,opty+optyspace,state);
    display.drawStr(optx,opty+2*optyspace,"Force Dose");
    display.drawStr(optx,opty+3*optyspace,"Calibrate Pump");
    display.drawStr(backx,backy,"Back");            //draw the back button (not needed for this menu context)
    display.setDrawColor(2);
    //display.setDrawColor(1);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,(opty)-7,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-7,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+2*optyspace)-7,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty + 3*optyspace)-7,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-7,23,boxh);
        break;
    }
  } while(display.nextPage() );
}

void displayDosePump2Menu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Dose Pump 2";
  //variables to be displayed
  char state[10];

  if(dose2_enabled)
    strcpy(state,"ENABLED");
  else
    strcpy(state,"DISABLED");

  //graphics position data
  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 10;
  uint8_t opty = 20;
  uint8_t optyspace = 10;
  uint8_t boxw = 105;
  uint8_t boxh = 9;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(27,11,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Volume/wk: ");
    sprintf(stringBuffer,"%i",dose2_volume);
    display.drawStr(optx+69,opty,stringBuffer);
    display.drawStr(optx+90,opty,"ml");
    display.drawStr(optx,opty+optyspace,"Status:");
    display.drawStr(optx+55,opty+optyspace,state);
    display.drawStr(optx,opty+2*optyspace,"Force Dose");
    display.drawStr(optx,opty+3*optyspace,"Calibrate Pump");
    display.drawStr(backx,backy,"Back");            //draw the back button (not needed for this menu context)
    display.setDrawColor(2);
    //display.setDrawColor(1);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,(opty)-7,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-7,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+2*optyspace)-7,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty+3*optyspace)-7,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-7,23,boxh);
        break;
    }
  } while(display.nextPage() );
}

void displayDosePumpCalibrationMenu(uint8_t cursorPos)
{

  char stateStr[5];
  if(dosePumpCalState)
    sprintf(stateStr,"STOP");
  else
    sprintf(stateStr,"START");

  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 10;
  uint8_t opty = 20;
  uint8_t optyspace = 10;
  uint8_t boxw = 105;
  uint8_t boxh = 9;

  display.firstPage();
  do
  {
    display.drawStr(backx,backy,"Back");
    display.drawStr(3,11,"1) Press to start pump");
    display.drawStr(3,11+1*optyspace,"2) Press again once 100");
    display.drawStr(3,11+2*optyspace,"   mL has been drained");
    display.drawStr(52,11+3*optyspace+5,stateStr);

    switch(cursorPos)
    {
    case 0:
      display.drawBox(50-1,11+3*optyspace-3,29,9);
    break;
    case 1:
      display.drawBox(backx-2,backy-7,23,boxh);
    break;
    }

  }while(display.nextPage() );

}

/*
void displaySettingsMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Settings Menu";

  uint8_t backx = 54;
  uint8_t backy = 60;
  uint8_t optx = 15;
  uint8_t opty = 25;
  uint8_t optyspace = 11;
  uint8_t boxw = 100;
  uint8_t boxh = 10;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(15,10,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Save Settings");
    display.drawStr(optx,opty+optyspace,"Reset to Default");

    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,opty-8,boxw,boxh);
        break;
      case 1:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
        break;
      case 2:
        display.drawBox(backx-2,backy-8,23,boxh);
        break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}*/
