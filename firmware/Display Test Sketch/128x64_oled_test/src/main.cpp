#include <Arduino.h>
#include <U8g2lib.h>
#include <string.h>

#define DISPLAY_RESET 9

//function prototypes
void displaySplashScreen(void); //displays the splash screen
void refreshDisplay(void);
void parseInputs(char);
void displayGenericMenu(void); //displays a generic menu

//display object constructor
//SSD1305: Display controller
//128x64: Resolution
//ADAFRUIT: Vendor
//1 : framebuffer size (1=128bytes pagebuffer, 2=256bytes pagebuffer, F=full 512bytes framebuffer)
//U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C(rotation, [reset [, clock, data]])

U8G2_SSD1305_128X64_ADAFRUIT_1_HW_I2C display(U8G2_R0, DISPLAY_RESET);

uint8_t num = 0;
char input = '0';
char command = '0';

char stringBuffer[25];


uint8_t uiLevel = 0; //two variables for keeping track of the current menu depth and horizontal position
uint8_t uiItem = 0;  //(0,0) is the splashscreen, each increment along the depth will take the user deeper into a menu chain
uint8_t inMenu = 0;
uint8_t cursorPosition = 0; //position of the virtual "cursor" that navigates the menus.


void setup() {
    display.begin();
    Serial.begin(9600);

}

void loop()
{
  if(Serial.available())  //parse input from keyboard (will be replaced with encoder input later on)
  {
    input = Serial.read();
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
  }

  if(uiLevel==0)
  {
    displaySplashScreen();
    if(command=='E')
    {
      uiLevel = 1;
      command = '\0';
    }
  }

  if(uiLevel>0)
  {
    displayGenericMenu();
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

void displayGenericMenu()
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


void refreshDisplay()             //this function defines what the OLED should display, based on the uiLevel and uiItem variables
{
  if(uiLevel==0)                  //TOP UI LEVEL, SPLASHSCREEN
  {

  }

  if(uiLevel==1)                  //FIRST LEVEL OF SUBMENUS
  {
    if(uiItem==0)                 //FIRST LEVEL, ITEM 1
    {

    }

    if(uiItem==1)                 //FIRST LEVEL, ITEM 2
    {

    }

  }




}

void parseInput(char command)
{

}

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
  /*
  level = 0:                                               SPLASHSCREEN
  level = 1:                MENU0(item=0)                 MENU1(item=1)       MENU2(item=2)       one cursor position per option, up/down scrolls to next option
  level = 2:  MENU0 OPTION0   MENU0 OPTION1   MENU0 OPTION2
                (item=0)        (item=1)        (item=2)
*/

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

};
