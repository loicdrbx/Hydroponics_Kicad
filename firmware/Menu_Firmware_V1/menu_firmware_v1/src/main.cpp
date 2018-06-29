#include <Arduino.h>
#include <U8g2lib.h>
#include <string.h>

#define DISPLAY_RESET 9
#define DEBUG

#define TOPLEVEL_MENU 1
#define PUMP_SETTINGS_MENU 2
#define LIGHTING_MENU 3
#define PH_MENU 4
#define CLOCK_MENU 5
#define MAIN_PUMP_MENU 11
#define AUTO_TOPOFF_MENU 12
#define DOSE_PUMP_1_MENU 13
#define DOSE_PUMP_2_MENU 14

char parseSerialInput(void);  //gets user input from the keyboard
char parseEncoderInput(void); //translates encoder pulses into menu commands

void refreshDisplay(void);    //refreshes the display, this should be called every cycle
void navigateMenu(char);      //parses navigation commands for the menu structure
void parseCursorPos(char,uint8_t, uint8_t);
int changeVariable(char, uint32_t, uint16_t, uint16_t); // circularly increments or decrements a variable, with a set min and max

void displayStatusMenu(void);
void displayPumpMenu(uint8_t);        //show top level pump menu
void displayLightingMenu(uint8_t);    //show lighting settings menu
void displayClockMenu(uint8_t);       //show the clock settings menu
void displayPHMenu(uint8_t);          //show the PH settings menu
void displayMainPumpMenu(uint8_t);    //show the main pump settings menu
void displayDosePump1Menu(uint8_t);   //show dosing pump 1 settings menu
void displayDosePump2Menu(uint8_t);   //show dosign pump 2 settings menu

void getTimeofDay(void);              //retreive the current time of day from the RTC
void setTimeofDay(void);              //set the current time of day from the RTC

void readEEPROMVariables(void);       //get any variables from memory that have been stored during a power outage
void writeEEPROMVariables(void);      //set values of variables in memory to keep data in the event of a power outage

void updateLights(void);              //turns the lights on or off, changes Brightness
void updateMainPump(void);            //turns the main pump on or off based on the current time_minutes
void updateDose1(void);               //delivers a dose to the system if one is required
void updateDose2(void);               //delivers a dose to the system if one is required

void deliverDose(uint8_t,uint16_t);   //delivers a dose of a specified volume to a specified pump

void calibratePH(void);               //calibrates the PH of the current solution

char* getTimeString(uint16_t);        //returns a HH MM 24 hour time format string when given hours and minutes


//DISPLAY OBJECT CONSTRUCTOR
//SSD1305: Display controller
//128x64: Resolution
//ADAFRUIT: Vendor
//1 : framebuffer size (1=128bytes pagebuffer, 2=256bytes pagebuffer, F=full 512bytes framebuffer)
//U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C(rotation, [reset [, clock, data]])
U8G2_SSD1305_128X64_ADAFRUIT_2_HW_I2C display(U8G2_R0, DISPLAY_RESET);

//GLOBAL VARIABLES:
uint16_t time_minutes = 720;  //maximum of 1440, represents the time of day in minutes
uint16_t ph = 512;            //maximum of 1024. PH represented as 0->0, 14->1024
uint16_t temperature = 512;   //temperature represented as an int

//variables related to the encoder
uint8_t a;
uint8_t b;
uint8_t aLast;
uint8_t bLast;
uint8_t button;

//LIGHT variables
uint8_t led_brightness = 100;
uint16_t led_on_time = 360;   //time of day that the LED strip turns on
uint16_t led_off_time = 1080; //time of day that the LED strip turns off
uint8_t led_enabled = 1;      //LED strip enable

//Main Pump Variables:
uint8_t main_pump_on_interval = 10;   //ON-time of the main pump in minutes
uint8_t main_pump_off_interval = 45;  //OFF-time of the main pump in minutes
uint8_t main_pump_enabled = 1;        //is the main pump enabled?
uint16_t main_pump_last_time;         //records last time that the pump state changed
uint8_t pumpState = 0;                //current pump state (on/off)

//auto topoff pump:
uint8_t auto_topoff_enabled = 1;

//dose pump 1 settings:
uint16_t dose1_volume = 1300; //total volume to be dosed for the first dosing Pumpuint16
uint16_t dose1_remaining;     //total remaining volume to be dosed
uint8_t dose1_interval = 60;  //time in minutes between doses
uint8_t dose1_enabled = 1;    //is dosing pump 1 enabled?

//dose pump 2 settings:
uint16_t dose2_volume = 1300; //total volume to be dosed for the first dosing Pump
uint16_t dose2_remaining;     //total remaining volume to be dosed
uint8_t dose2_interval = 60;  //time in minutes between doses
uint8_t dose2_enabled = 1;    //is dosing pump 1 enabled?

//Menu navigation global variables:
uint8_t cursor = 0;                     //used for menu navigation horizontally
uint8_t menuLevel = 1;                  //used for menu navigation vertically
uint8_t editingVariable = 0;            //used to keep track of navigation button context (edit a variable or change menu selection)
uint8_t currentMenu = TOPLEVEL_MENU;    //for keeping track of the current menu that is being occupied
char command = '\0';                    //command variable for parsing the user input to the controller

char stringBuffer[50];    //used as an intermediary for the sprintf function


void setup()
{
  display.begin();        //start the display
  Serial.begin(9600);     //start serial port
}

void loop()
{
  command = parseSerialInput();
  navigateMenu(command);


  command = '\0';
}

//FUNCTIONS:

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
    variable++;
  if(command=='D')
  {
    if(!(variable-min)) //if variable is at minimum value, and command is decrement, roll over to maximum value
      variable=max;
    else
      variable--;
  }
  variable = variable%(max+1); //if variable is at maximum, and command is increment, roll over to minimum value
  return(variable);
}

void updateLights()
{
  if(time_minutes <= led_on_time)
  {
    //put analog stuff here
  }
  if(time_minutes >= led_off_time)
  {
    //put analog stuff here
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
    if(time_minutes - main_pump_last_time >= main_pump_off_interval) //if the elapsed off interval has passed
    {
      pumpState = 1;
      main_pump_last_time = time_minutes;
    }
  }
  //turn on output pin here
}

void deliverDose(uint8_t pump, uint16_t volume)   //delivers a dose of the specified volume to the system
{
  if(pump==1)
  {
    //deliver dose
  }
  if(pump==2)
  {
    //deliver dose
  }
}

void readEEPROMVariables()
{

}

void writeEEPROMVariables()
{

}

void navigateMenu(char command) //master menu navigation function
{
  uint8_t buttonPressed = 0;    //command to tell if button has been pressed by user

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
        displayPumpMenu(254);
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU; //if button is pressed, enter the pump settings menu
          cursor = 0;
        }
      break;
      case 2:                     //Lighting Menu is being hovered over
        displayLightingMenu(254);
        if(buttonPressed)
        {
          currentMenu = LIGHTING_MENU; //if button is pressed, enter the lighting menu
          cursor = 0;
        }
      break;
      case 3:                     //PH menu is being hovered over
        displayPHMenu(254);
        if(buttonPressed)
        {
          currentMenu = PH_MENU;  //if button is pressed, enter the PH menu
          cursor = 0;
        }
      break;
      case 4:                     //Clock menu is being hovered over
        displayClockMenu(254);
        if(buttonPressed)
        {
          currentMenu = CLOCK_MENU;  //if button is pressed, enter the clock settings menu
          cursor = 0;
        }
      break;
    }
    buttonPressed = 0;              //reset button push
  }

  if(currentMenu == PUMP_SETTINGS_MENU) //if the current menu is the pump settings menu
  {
    parseCursorPos(command,0,5);  //rotate cursor through 5 positions
    displayPumpMenu(cursor);
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
    displayLightingMenu(cursor);    //display the lighting menu for the current position
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
          led_on_time = changeVariable(command,led_on_time,0,1440);
      break;
      case 2:                       //hovering over the LED off-time button
        if(buttonPressed)           //if the button is pressed, enter or exit editing mode
          editingVariable=!editingVariable;
        if(editingVariable)         //if we are in editing mode, change the led_off_time variable
          led_off_time = changeVariable(command,led_off_time,0,1440);
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
        }
      break;
    }
  }

  if(currentMenu == PH_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,3);
    displayPHMenu(cursor);
    switch(cursor)
    {
      case 0:
      break;
      case 1:
      break;
      case 2:
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 3;
        }
      break;
    }
  }

  if(currentMenu == CLOCK_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,2);
    displayClockMenu(cursor);
    switch(cursor)
    {
      case 0:
      break;
      case 1:
        if(buttonPressed)
        {
          currentMenu = TOPLEVEL_MENU;
          cursor = 4;
        }
      break;
    }
  }

  if(currentMenu == MAIN_PUMP_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,4);
    displayMainPumpMenu(cursor);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          main_pump_on_interval = changeVariable(command,main_pump_on_interval,0,255);
      break;
      case 1:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          main_pump_off_interval = changeVariable(command,main_pump_off_interval,0,255);
      break;
      case 2:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          main_pump_enabled = changeVariable(command,main_pump_enabled,0,1);
      break;
      case 3:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 0;
        }
      break;
    }
  }

  if(currentMenu == AUTO_TOPOFF_MENU) //STILL NEEDS TO BE IMPLEMENTED
  {
    parseCursorPos(command,0,2);
    displayMainPumpMenu(cursor);
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
    displayDosePump1Menu(cursor);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose1_volume = changeVariable(command,dose1_volume,0,2500);
      break;
      case 1:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose1_interval = changeVariable(command,dose1_interval,0,1440);
      break;
      case 2:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose1_enabled = changeVariable(command,dose1_enabled,0,1);
      break;
      case 3:
      break;
      case 4:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 2;
        }
      break;
    }
  }

  if(currentMenu == DOSE_PUMP_2_MENU)
  {
    if(!editingVariable)
      parseCursorPos(command,0,5);
    displayDosePump2Menu(cursor);
    switch(cursor)
    {
      case 0:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose2_volume = changeVariable(command,dose2_volume,0,2500);
      break;
      case 1:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose2_interval = changeVariable(command,dose2_interval,0,1440);
      break;
      case 2:
        if(buttonPressed)
          editingVariable=!editingVariable;
        if(editingVariable)
          dose2_enabled = changeVariable(command,dose2_enabled,0,1);
      break;
      case 3:
      break;
      case 4:
        if(buttonPressed)
        {
          currentMenu = PUMP_SETTINGS_MENU;
          cursor = 2;
        }
      break;
    }
  }

}

void displayStatusMenu() //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[] = "Grow Wave V1.0";
  //variables to be displayed
  uint8_t tempint = 22;
  uint8_t tempfrac = 2;
  uint8_t phint = 6;
  uint8_t phfrac = 3;

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
    sprintf(stringBuffer,"%i.%i C",tempint,tempfrac);
    display.drawStr(optx+63,opty+optyspace,stringBuffer);
    display.drawStr(optx,opty+2*optyspace,"Current PH: ");
    sprintf(stringBuffer,"%i.%i",phint,phfrac);
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
    display.drawStr(optx,opty+optyspace,"Clock Time:");
    getTimeString(time_minutes);
    display.drawStr(optx+63,opty+optyspace,stringBuffer);
    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 254:   //no highlight
        display.drawFrame(0,0,126,63);                //draw box around whole frame to indicate selection
        break;
      case 0:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
        break;
      case 1:
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
    sprintf(stringBuffer,"%i",ph);
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
    display.drawStr(optx,opty,"Volume(daily): ");
    sprintf(stringBuffer,"%i",dose1_volume);
    display.drawStr(optx+69,opty,stringBuffer);
    display.drawStr(optx+90,opty,"ml");
    display.drawStr(optx,opty+optyspace,"Interval: ");
    sprintf(stringBuffer,"%i",dose1_interval);
    display.drawStr(optx+48,opty+optyspace,stringBuffer);
    display.drawStr(optx+58,opty+optyspace,"mins");
    display.drawStr(optx,opty+2*optyspace,"Status:");
    display.drawStr(optx+55,opty+2*optyspace,state);
    display.drawStr(optx,opty+3*optyspace,"Force Dose");
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
    display.drawStr(optx,opty,"Volume(daily): ");
    sprintf(stringBuffer,"%i",dose2_volume);
    display.drawStr(optx+69,opty,stringBuffer);
    display.drawStr(optx+90,opty,"ml");
    display.drawStr(optx,opty+optyspace,"Interval: ");
    sprintf(stringBuffer,"%i",dose2_interval);
    display.drawStr(optx+48,opty+optyspace,stringBuffer);
    display.drawStr(optx+58,opty+optyspace,"mins");
    display.drawStr(optx,opty+2*optyspace,"Status:");
    display.drawStr(optx+55,opty+2*optyspace,state);
    display.drawStr(optx,opty+3*optyspace,"Force Dose");
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
