#include <Arduino.h>
#include <U8g2lib.h>
#include <string.h>

#define DISPLAY_RESET 9
#define DEBUG

//function prototypes
void displaySplashScreen(void); //displays the splash screen
void displayGenericMenu(void); //displays a generic menu
char parseSerialInput(void);
void parseMenuInput(char);
void refreshDisplay(void);
void displayTestMenu(void);
void displayItemMenu(uint8_t);
void displayValueMenu(uint8_t);
void displayStatusMenu(uint8_t);
void displayPumpMenu(uint8_t);



//display object constructor
//SSD1305: Display controller
//128x64: Resolution
//ADAFRUIT: Vendor
//1 : framebuffer size (1=128bytes pagebuffer, 2=256bytes pagebuffer, F=full 512bytes framebuffer)
//U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C(rotation, [reset [, clock, data]])
U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C display(U8G2_R0, DISPLAY_RESET);



uint8_t num = 0;
char stringBuffer[50];
char command = '\0';

uint8_t menuLevel = 0; //two variables for keeping track of the current menu depth
uint8_t menuItem = 0;  //(0,0) is the splashscreen, each increment along the depth will take the user deeper into a menu chain, each increment for item will be a horizontal translation instead
uint8_t inMenu = 0;
uint8_t cursorPosition = 0; //position of the virtual "cursor" that navigates the menus.

void setup()
{
    display.begin();
    Serial.begin(9600);

}

void loop()
{
  /*if(Serial.available())  //parse input from keyboard (will be replaced with encoder input later on)
    parseMenuInput(parseSerialInput());
  refreshDisplay();
  delay(20);*/
  if(1)
  {
    displaySplashScreen();
    delay(2000);
    displayItemMenu(1);
    delay(500);
    displayItemMenu(2);
    delay(500);
    displayItemMenu(3);
    delay(500);
    displayItemMenu(4);
    delay(500);
    displayStatusMenu(0);
    delay(2000);
    displayPumpMenu(0);
    delay(250);
    displayPumpMenu(1);
    delay(250);
    displayPumpMenu(2);
    delay(250);
    displayPumpMenu(3);
    delay(250);
    displayPumpMenu(4);
    delay(250);
  }
  if(0)
  {
    displayPumpMenu(0);
    delay(250);
    displayPumpMenu(1);
    delay(250);
    displayPumpMenu(2);
    delay(250);
    displayPumpMenu(3);
    delay(250);
    displayPumpMenu(4);
    delay(250);
  }

}


char parseSerialInput(void)
{
  char command;
  char input = Serial.read();
  Serial.print("echo input is: ");
  Serial.print(input);
  Serial.print(", command is: ");
  if(input == '5')
  {
    command = 'E';
    Serial.print("ENTER");
  }
  else if(input == '2')
  {
    command = 'D';
    Serial.print("DOWN");
  }
  else if(input == '8')
  {
    command = 'U';
    Serial.print("UP");
  }
  else
    Serial.print("Unrecognized Command");
  Serial.print("\n");
  return(command);
}

void parseMenuInput(char command) //spaghetti...
{
  if(menuLevel==0)      //if we're on the splashscreen, press enter to enter the menu system
  {
    if(command=='E')
    {
      menuLevel++;      //go to the first menu level
      command='\0';     //reset command
    }
  }

  if(menuLevel>=1)
  {
    if(command=='U')
    {
      menuItem++;
      command='\0';     //reset command
    }
    if(command=='D')
    {
      menuItem--;
      command='\0';     //reset command
    }
    if(command=='E')
    {
      menuLevel++;
      command='\0';     //reset command
    }
  }
}



void refreshDisplay()
{
  if(menuLevel==0)
  {
    displaySplashScreen();
  }
  if(menuLevel>=1)
  {
    displayTestMenu();
  }
}




void displaySplashScreen()
{
  display.firstPage();
  do
  {
    display.setFont(u8g2_font_ncenB10_tr); //set the font to title font
    display.drawStr(20,15,"Grow-Wave");    //title text
    display.drawStr(10, 31, "Splashscreen"); //splashscreen text
    display.setFont(u8g2_font_profont10_mr);
    display.drawStr(15,50, "Press 5 to continue");
  } while ( display.nextPage() ); //display.nextpage will return 0 once the contents of the loop are done being written to the display
}

void displayGenericMenu(int uiLevel)
{
  display.firstPage();
  do
  {
    display.setFont(u8g2_font_profont12_mr);
    sprintf(stringBuffer,"Generic Menu lvl: %i",uiLevel); //generic title with ui level inbuilt
    display.drawStr(1,10,stringBuffer);

    display.setFont(u8g2_font_profont10_mr); //items
    display.drawStr(5,25,"ITEM1");
    display.drawStr(5,35,"ITEM1");
    display.drawStr(5,45,"ITEM1");
    display.drawStr(1,60,"BACK");
  } while ( display.nextPage() ); //display.nextpage will return 0 once the contents of the loop are done being written to the display
}

void displayTestMenu()
{
  display.firstPage();
  do
  {
    display.setFont(u8g2_font_profont12_mr);
    sprintf(stringBuffer,"Generic Menu lvl: %i",menuLevel); //generic title with ui level inbuilt
    display.drawStr(1,10,stringBuffer);
    sprintf(stringBuffer,"Menu Item is: %i",menuItem);
    display.drawStr(5,20,stringBuffer);
  } while ( display.nextPage() ); //display.nextpage will return 0 once the contents of the loop are done being written to the display
}

void displayValueMenu(uint8_t cursorPos)
{
  uint8_t backx = 58;
  uint8_t backy = 60;
  uint8_t valx = 5;
  uint8_t valy = 30;
  uint8_t boxh = 10;
  uint8_t boxw = 90;
  uint16_t value = 100;
  char menuTitle[25] = "Value Change Menu";
  char prefix[15] = "Value: ";
  char unit[10] = "Units";

  do
  {
    display.setDrawColor(1);                    //set draw color to solid
    display.setFont(u8g2_font_profont12_mr);    //draw the title block
    display.drawStr(15,10,menuTitle);
    display.setFont(u8g2_font_profont10_mr);

    display.drawStr(valx,valy,prefix);
    sprintf(stringBuffer,"%i",value);           //convert the int value to a string to print to the display
    display.drawStr(valx+30,valy,stringBuffer);
    display.drawStr(valx+50,valy,unit);

    display.drawStr(backx,backy,"Back");            //draw the back button

    display.setDrawColor(2);                    //set draw color to XOR
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 0:   //no highlight
        break;
      case 1:   //highlight on the Value
        display.drawBox(valx-2,valy-8,boxw,boxh);
        break;
      case 2:
        display.drawBox(backx-2,backy-8,23,boxh);
        break;

    }
    display.setDrawColor(1);  //reset draw color to 1
  } while(display.nextPage() );
}




void displayItemMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[15] = "Itembox Menu";
  char item1[15] = "Option 1";
  char item2[15] = "Option 2";
  char item3[15] = "Option 3";

  uint8_t backx = 58;
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
    display.drawStr(optx,opty,item1);
    display.drawStr(optx,opty+optyspace,item2);
    display.drawStr(optx,opty+2*optyspace,item3);

    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 0:   //no highlight
        break;
      case 1:
        display.drawBox(optx-2,opty-8,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty+2*optyspace)-8,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-8,23,boxh);
        break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}

void displayStatusMenu(uint8_t cursorPos) //4 cursor positions none, 1,2,3, and back.
{
  char menuTitle[15] = "Status Menu";

  uint8_t temp = 25;
  uint8_t ph = 6;
  char waterLevel[10] = "Good";

  uint8_t backx = 58;
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
    display.drawStr(optx,opty,"Temperature: ");
    sprintf(stringBuffer,"%i C",temp);
    display.drawStr(optx+60,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"Water Level: ");
    display.drawStr(optx+60,opty+optyspace,waterLevel);
    display.drawStr(optx,opty+2*optyspace,"Current PH: ");
    sprintf(stringBuffer,"%i",ph);
    display.drawStr(optx+60,opty+2*optyspace,stringBuffer);

    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 0:   //no highlight
        break;
      case 1:
        display.drawBox(optx-2,opty-8,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty+2*optyspace)-8,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-8,23,boxh);
        break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}

void displayPumpMenu(uint8_t cursorPos)
{
  char menuTitle[25] = "Pump Settings Menu";

  uint16_t pump1time = 500;
  uint16_t pump2time = 1200;
  uint8_t dosetime = 5;

  uint8_t backx = 58;
  uint8_t backy = 60;
  uint8_t optx = 7;
  uint8_t opty = 25;
  uint8_t optyspace = 11;
  uint8_t boxw = 115;
  uint8_t boxh = 10;
  display.firstPage();
  do
  {
    display.setDrawColor(1);                 //set draw color to solid
    display.setFont(u8g2_font_profont12_mr); //draw the title block
    display.drawStr(10,10,menuTitle);
    display.setFont(u8g2_font_profont10_mr); //draw the item menus
    display.drawStr(optx,opty,"Pump1 ON time: ");
    sprintf(stringBuffer,"%i ms", pump1time);
    display.drawStr(optx+75,opty,stringBuffer);
    display.drawStr(optx,opty+optyspace,"Pump2 ON Time: ");
    sprintf(stringBuffer,"%i ms", pump2time);
    display.drawStr(optx+75,opty+optyspace,stringBuffer);
    display.drawStr(optx,opty+2*optyspace,"Dose Time: ");
    sprintf(stringBuffer,"%i:00 h", dosetime);
    display.drawStr(optx+75,opty+2*optyspace,stringBuffer);

    display.drawStr(backx,backy,"Back");            //draw the back button
    display.setDrawColor(2);
    switch(cursorPos) //draw a highlight over the selected cursor position
    {
      case 0:   //no highlight
        break;
      case 1:
        display.drawBox(optx-2,opty-8,boxw,boxh);
        break;
      case 2:
        display.drawBox(optx-2,(opty+optyspace)-8,boxw,boxh);
        break;
      case 3:
        display.drawBox(optx-2,(opty+2*optyspace)-8,boxw,boxh);
        break;
      case 4:
        display.drawBox(backx-2,backy-8,23,boxh);
        break;
    }
    //display.setDrawColor(1);
  } while(display.nextPage() );
}

//object oriented approach, not used.
/*
class menu
{
  public:
  uint8_t level;      //current UI level (depth)
  uint8_t item;       //current UI item (lateral menu movement)
  uint8_t cursorPos;  //cursor position

  menu()         //default constructor, set menu position to (0,0) and cursor position to 0
  {
    level = 0;
    item = 0;
    cursorPos = 0;
  }

  //DIAGRAM FOR MENU HEIRARCHY

  level = 0:                                               SPLASHSCREEN
  level = 1:                SELECTIONMENU1(item=0)                 MENU1(item=1)       MENU2(item=2)       one cursor position per option, up/down scrolls to next option
  level = 2:  MENU0 OPTION0   MENU0 OPTION1   MENU0 OPTION2
                (item=0)        (item=1)        (item=2)


  void parseInput(char command)
  {
    if(level==0 && command == 'E') //go to next level past splashscreen if enter is pressed
    {
      level=1;
    }

    if(level==1 && command=='E')   //enter the menu options for the selected menu
    {

    }
  }

  void refreshDisplay()
  {

  }

};*/
