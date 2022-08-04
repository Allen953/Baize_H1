//***********************************************************************
// Hexapod PS2 Controller Test Program
// Code for Arduino Mega
// by Mark W
//***********************************************************************

//***********************************************************************
//Important Usage and troubleshooting notes:
//***********************************************************************
// For this test program, the arduino and the remote receiver will be powered 
// by the USB cable from your PC for this testing purpose - you do not want to 
// connect the LiPo battery that powers the hexapod for this.  Load the test 
// sketch and you should see the remote joystick positions and the button presses 
// on the arduino Serial Monitor window and verify that the remote control is 
// working properly.
//
// The guy who developed the PS2 library also has a troubleshooting page at:
// http://www.billporter.info/2011/03/27/arduino-playstation-2-controller-library-troubleshooting-guide/
//
// Here are a couple other things to try if your controller is not working.
//   1. Find the file named "PS2X_lib.h"  Mine is in the path Documents\Arduino\libraries\PS2X_lib.  
//   Edit it with a text editor (WordPad works well) and find these lines:
//      #define CTRL_CLK        4
//      #define CTRL_BYTE_DELAY 3
//   Change both values to 16:
//      #define CTRL_CLK        16
//      #define CTRL_BYTE_DELAY 16
//   Save the file and close WordPad. 
//
//   2. In the same folder as the above, find the file named "PS2X_lib.cpp".  
//   Edit it with the text editor and find these lines:
//      pinMode(dat, INPUT);
//      digitalWrite(dat, HIGH); //enable pull-up
//   Change the first one to INPUT_PULLUP and comment out the second line like this:
//      pinMode(dat, INPUT_PULLUP);
//      //digitalWrite(dat, HIGH); //enable pull-up
//   Save the file and close WordPad.


//***********************************************************************
// Includes
//***********************************************************************
#include <PS2X_lib.h>   //reference: http://www.billporter.info/
#include <math.h>


//***********************************************************************
// Constant Declarations
//***********************************************************************
const int PS2_DAT = 2;                //gamepad port definitions
const int PS2_ATT = 3;
const int PS2_CMD = 4;
const int PS2_CLK = 5;
const int RUMBLE = true;
const int PRESSURES = false;

const int FRAME_TIME_MS = 200;         //frame time (200msec = 5Hz)


//***********************************************************************
// Variable Declarations
//***********************************************************************
int gamepad_error;                    //gamepad variables
byte gamepad_type;
byte gamepad_vibrate;

unsigned long currentTime;            //frame timer variables
unsigned long previousTime;


//***********************************************************************
// Object Declarations
//***********************************************************************
PS2X ps2x;              //PS2 gamepad controller


//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  //start serial
  Serial.begin(115200);
  
  //connect the gamepad
  gamepad_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, PRESSURES, RUMBLE);
  if(gamepad_error == 0)      Serial.println("Controller attached");
  else if(gamepad_error == 1) Serial.println("No controller found");
  else if(gamepad_error == 2) Serial.println("Controller found but not accepting commands");
  else if(gamepad_error == 3) Serial.println("Controller refusing to enter Pressures mode");

  //verify the gamepad type
  gamepad_type = ps2x.readType(); 
  if(gamepad_type == 0)      Serial.println("Unknown Controller type found");
  else if(gamepad_type == 1) Serial.println("DualShock Controller found");
  else if(gamepad_type == 2) Serial.println("GuitarHero Controller found");
  else if(gamepad_type == 3) Serial.println("Wireless Sony DualShock Controller found");

  //turn off gamepad vibration
  gamepad_vibrate = 0;
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop() 
{
  //exit if no controller found or GuitarHero controller
  if((gamepad_error == 1) || (gamepad_type == 2))
  {
    Serial.println("Invalid Controller!");
    return; 
  }

  //set up frame time
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime; 

    //read controller and process inputs
    ps2x.read_gamepad(false, gamepad_vibrate);      
    Serial.print(ps2x.Analog(PSS_LX));
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LY));
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RX));
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY));
    Serial.print("   ");
    process_gamepad();
    Serial.print("\n");
  }
}


//***********************************************************************
// Process gamepad controller inputs
//***********************************************************************
void process_gamepad()
{
  if(ps2x.ButtonPressed(PSB_PAD_DOWN))
    Serial.print("Down");
  if(ps2x.ButtonPressed(PSB_PAD_LEFT)) 
    Serial.print("Left");
  if(ps2x.ButtonPressed(PSB_PAD_UP))   
    Serial.print("Up");
  if(ps2x.ButtonPressed(PSB_PAD_RIGHT))
    Serial.print("Right");
  if(ps2x.ButtonPressed(PSB_TRIANGLE))
    Serial.print("Triangle");
  if(ps2x.ButtonPressed(PSB_SQUARE))
    Serial.print("Square");
  if(ps2x.ButtonPressed(PSB_CIRCLE))
    Serial.print("Circle");
  if(ps2x.ButtonPressed(PSB_CROSS))
    Serial.print("Cross");
  if(ps2x.ButtonPressed(PSB_START))
    Serial.print("Start");
  if(ps2x.ButtonPressed(PSB_SELECT))
    Serial.print("Select");
  if(ps2x.ButtonPressed(PSB_L1))
    Serial.print("L1");
  if(ps2x.ButtonPressed(PSB_R1))
    Serial.print("R1");
  if(ps2x.ButtonPressed(PSB_L2))
    Serial.print("L2");
  if(ps2x.ButtonPressed(PSB_R2))
    Serial.print("R2");
}
