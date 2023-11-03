/*
  This code is intended for the Mini Droid builds by Matt Zwarts
  It incorporates the use of a PS3 style controller, ESP32 microcontroller and MX1508 motor drivers or similar
  I use a 11.1v battery and step down buck converter to run at 5-6 Volts for the servos
  You can do the same with a 7.4 Volt battery as well or 4 AA batteries

  You will need to install the Ps3Controller, ESP32Servo and DFPlayerMini_Fast libraries from the Tools/Manage Libraries selection for the code to compile

  You will need to find out your PS3 or name brand controller MAC address and enter it into the Ps3.begin setup function below in the code
  //Ps3.begin("00:00:00:00:00:00");

  This code is based on the amazing work from Bill at DroneBot Workshop, please review the video below for more details on wiring and connecting the PS3 controller
  https://dronebotworkshop.com/ps3-esp32/

  Connnections to ESP32:
  //motor driver pins to MX1508 motor driver for steering
  IN1_PIN 5
  IN2_PIN 18
  IN3_PIN 19
  IN4_PIN 21

  //motor driver pins to MX1508 for dome motor
  IN5_PIN 2
  IN6_PIN 4

  Connections to servos, 5volt from a constant supply and common GND
  servo2 26 front arm 1 on PS3 L2
  servo3 27 front arm 2 on PS3 R2

  connections to dfplayer mini, again this needs 5Volt and common GND, check pin outs for the component online or on the build instructions
  TX2 - 17
  RX2 - 16

must apply the below fix to ESPAsyncWebServer
https://github.com/philbowles/ESPAsyncWebServer/issues/3
*/

// Include required libraries
#include <math.h>
#include <DFPlayerMini_Fast.h>
#include <Adafruit_PWMServoDriver.h>
#include "r2PWMoutput.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "SPIFFS.h"
#include <Adafruit_Fingerprint.h>
#include "r2HTML.h"
#include <AsyncElegantOTA.h>

// #include <ArduinoWebsockets.h>
//  Can be replaced with PS4 (https://github.com/aed3/PS4-esp32) or PS5 (https://github.com/rodneybakiskan/ps5-esp32) Library you just need to adjust the Notify for the available buttons and change references o PS3
#include <Ps3Controller.h>

// Set to true to serial debug messages
const bool sDEBUG = false;

const char *ssid = "R2D2-Mini";
const char *password = "12345678";
// enter your PS3 controller MAC address here, you can use a usb cable on PC with sixaxis tool to discover the MAC address
String MACaddress = "00:00:00:00:00:00";

int player = 0;
int battery = 0;

const char *PARAM_MESSAGE = "message";

AsyncEventSource events("/events");

const int deadzone = 10;

/// Define motor driver pins MX1508
#define IN1_PIN 13
#define IN2_PIN 12
#define IN3_PIN 14
#define IN4_PIN 27
#define IN5_PIN 26
#define IN6_PIN 25

#define SER1_TX 4
#define SER1_RX 2

// Serial2 Pins
// TX - 17
// RX - 16

// IC2 Pins
// SDA - 21
// SCL - 22

// Create Web Server
AsyncWebServer server(80);
// PWMBoard
// Call boards for i2c
// #define PWM1 1
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40);
// #define PWM2 2
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

#define NA -1

// PWM Definitions.  After first run you have to a clear on preferences
// Servos
r2PWMoutput xLeftArm("Left_Arm", pwm1, 0, r2PWMoutput::Servo, 30, 170, 110, 200, r2PWMoutput::Min, r2PWMoutput::Manual);
r2PWMoutput xRightArm("Right_Arm", pwm1, 1, r2PWMoutput::Servo, 30, 170, 0, 90, r2PWMoutput::Min, r2PWMoutput::Manual);
r2PWMoutput xCenterLift("Center_Lift", pwm1, 3, r2PWMoutput::Servo, 28, 125, NA, NA, r2PWMoutput::Min, r2PWMoutput::Manual);
r2PWMoutput xTilt("Tilt", pwm1, 4, r2PWMoutput::Servo, 40, 135, NA, NA, r2PWMoutput::Min, r2PWMoutput::Manual);

r2PWMoutput xDomeLED1("Dome_LED1", pwm2, 0, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xDomeLED2("Dome_LED2", pwm2, 1, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xDomeLED3("Dome_LED3", pwm2, 2, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xDomeLED4("Dome_LED4", pwm2, 3, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xHolo1("Holo1", pwm2, 4, r2PWMoutput::ContinuousServo, 150, 600, 2, 10, 80, 5, 20, r2PWMoutput::Off);
r2PWMoutput xHolo2("Holo2", pwm2, 5, r2PWMoutput::ContinuousServo, 150, 600, 2, 10, 80, 5, 20, r2PWMoutput::Off);
r2PWMoutput xHolo3("Holo3", pwm2, 6, r2PWMoutput::ContinuousServo, 150, 600, 2, 10, 80, 5, 20, r2PWMoutput::Off);
r2PWMoutput xDomeLED5("Dome_LED5", pwm2, 7, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xPeriscope("Periscope", pwm2, 10, r2PWMoutput::Servo, 25, 135, NA, NA, r2PWMoutput::Min, r2PWMoutput::Manual);
r2PWMoutput xPeriscopeLED("Periscope_LED", pwm2, 11, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xHolo1LED("Holo1_LED", pwm2, 13, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xHolo2LED("Holo2_LED", pwm2, 14, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);
r2PWMoutput xHolo3LED("Holo3_LED", pwm2, 15, r2PWMoutput::LED, 2, 10, 10, 60, r2PWMoutput::Off);

// time
unsigned long currentMillis; // time current

//  Define channels for each motor
const uint8_t motorAChannel1 = 3;
const uint8_t motorAChannel2 = 6;
const uint8_t motorBChannel1 = 4;
const uint8_t motorBChannel2 = 7;
// dome motor
const uint8_t motorCChannel1 = 9;
const uint8_t motorCChannel2 = 11;

// Dome speed is slowed down on the dome motor rotation by this multiplying value
const float dome_speed = 0.75; // dome rotation speed multiplier, increase or decrease value to speed up dome or slow down, max value is 1.0

// Pins used for DFPlayerMini are Serial1 on GPIO-17 TX and GPIO-16 RX
// DF Player mini variables
int setsoundvolume = 25;        ////////////////////////// Change this value from 0 to 30 to adjust the DF player volume///////////////////////////
const int dfPlayerDelay = 5000; ///////////////////////// Change this value to increase or decrease the delay in between sounds, time is in milliseconds, i.e. 3 seconds = 3000

DFPlayerMini_Fast myDFPlayer;

AsyncWebSocket ws("/ws");

/// @brief Serial debuging message handler
/// @param msg the message to print to Serial if debug is true
void SerialDebug(const String msg)
{
    if (sDEBUG)
    {
        Serial.println(msg);
    }
}

bool SingleConnection()
{
    if ((!Ps3.isConnected() && ws.count() != 1) || (Ps3.isConnected() && ws.count() == 1))
        return false;
    else
        return true;
}

/////////////////////////////////////////////////////////////////////////////
// Motor and Servo Motion Events
#pragma region Server_Motor_Move
/// @brief Toggle the center leg up/down
void liftmechanism()
{
    if (xTilt.PWMState == 1)
    {                                  // check the Upright before lifting leg
        if (xCenterLift.PWMState == 0) // down
        {
            xCenterLift.servoMin();
            xCenterLift.PWMState = 1;
        }
        else if (xCenterLift.PWMState == 1) // up
        {
            xCenterLift.servoMax();
            xCenterLift.PWMState = 0;
        }
    }
}

/// @brief Toggle the leg tilt tipped/upright
void tiltmechanism()
{
    if (xCenterLift.PWMState == 1)
    { // check the leg is down before tilting the body
        if (xTilt.PWMState == 0)
        {
            xTilt.servoMax();
            xTilt.PWMState = 1;
        }
        else if (xTilt.PWMState == 1)
        {
            xTilt.servoMin();
            xTilt.PWMState = 0;
        }
    }
}

/// @brief Toggle periscope up/down
void periscopemotion()
{
    if (xPeriscope.PWMState == 0)
    {
        xPeriscope.PWMState = 1;
        xPeriscope.servoMax();
    }
    else if (xPeriscope.PWMState == 1)
    {
        xPeriscope.PWMState = 0;
        xPeriscope.servoMin();
    }
}

/// @brief Move the Leg/Dome motor
/// @param motorPWM PWM Speed
/// @param motorChannel1 Channel 1
/// @param motorChannel2 Channel 2
void moveMotor(int motorPWM, const uint8_t motorChannel1, const uint8_t motorChannel2)
{
    Serial.println(motorPWM);
    constrain(motorPWM, -255, 255);
    if (motorPWM <= -deadzone)
    {
        motorPWM = abs(motorPWM);
        ledcWrite(motorChannel1, motorPWM);
        ledcWrite(motorChannel2, 0);
    }
    else if (motorPWM >= deadzone)
    {
        ledcWrite(motorChannel2, motorPWM);
        ledcWrite(motorChannel1, 0);
    }
    else
    {
        ledcWrite(motorChannel1, 0);
        ledcWrite(motorChannel2, 0);
    }
}

/// @brief Move dome
/// @param speedZ Speed to be converted to PWM includes direction
void moveDome(const int speedZ)
{
    if (SingleConnection())
    {
        int motorCPWM = 0;
        // head turn
        motorCPWM = speedZ * dome_speed;
        moveMotor(motorCPWM, motorCChannel1, motorCChannel2);
    }
}

/// @brief Move Leg Motors
/// @param speedX Speed left Leg
/// @param speedY Speed right Leg
void moveLegs(const int speedX, const int speedY)
{
    if (SingleConnection())
    {
        // Variables for Motor PWM values
        int motorAPWM = 0;
        int motorBPWM = 0;

        // Convert both stick values for steering from cartesian to Polar coordinates
        // hypotenuse
        float radius = sqrt(speedX * speedX + speedY * speedY);
        // angle in radians
        float theta = acos(abs(speedY) / radius);
        // cater for NaN values
        if (isnan(theta) == true)
        {
            theta = 0;
        }
        // angle in degrees
        float angle = theta * 180 / 3.1415;
        float tcoeff = -1 + (angle / 90) * 2;
        float turn = tcoeff * abs(abs(speedX) - abs(speedY));
        turn = round(turn * 100) / 100;
        float mov = max(abs(speedX), abs(speedY));
        // First and third quadrant
        float rawLeft;
        float rawRight;
        if ((speedY >= 0 && speedX >= 0) || (speedY < 0 && speedX < 0))
        {
            rawLeft = mov;
            rawRight = turn;
        }
        else
        {
            rawRight = mov;
            rawLeft = turn;
        }
        // Reverse polarity
        if (speedX < 0)
        {
            rawLeft = 0 - rawLeft;
            rawRight = 0 - rawRight;
        }

        // Map the values onto the defined rang
        motorAPWM = map(rawLeft, -255, 255, -255, 255);
        motorBPWM = map(rawRight, -255, 255, -255, 255);

        // Set the motor directions
        moveMotor(motorAPWM, motorAChannel1, motorAChannel2);
        moveMotor(motorBPWM, motorBChannel1, motorBChannel2);
    }
}
#pragma endregion Server_Motor_Move

/// @brief Turn Periscope LED on/off (random)
void periscopeLED()
{
    if (xPeriscope.PWMState == HIGH)
    { // blink the periscope LED
        xPeriscopeLED.RandomTrigger(currentMillis);
    }
    else if (xPeriscope.PWMState == LOW)
    {
        if (xPeriscopeLED.PWMState == LOW)
        {
            xPeriscopeLED.OFF();
        }
    }
}

// PS3 Controller Functions
// Callback Function
void PS3notify()
{
    int speedX;
    int speedY;
    int speedZ;

    int rightX;
    int rightY;
    int leftX;
    int leftY;

    // Analog Inputs
    //  Get Left Joystick value
    if (abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2)
    {
        leftX = (Ps3.data.analog.stick.lx);
        leftY = (Ps3.data.analog.stick.ly);
    }
    // Get Right Joystick value
    if (abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2)
    {
        rightX = (Ps3.data.analog.stick.rx);
        rightY = (Ps3.data.analog.stick.ry);
    }
    // front arm 1
    if (abs(Ps3.event.analog_changed.button.r2))
    {
        // Right Trigger - right arm
        xRightArm.setPWM(0, int(Ps3.data.analog.button.r2), true);
    }
    // front arm 2
    if (abs(Ps3.event.analog_changed.button.l2))
    {
        // Left Trigger - Left arm
        xLeftArm.setPWM(0, int(Ps3.data.analog.button.l2), true);
    }

    //--- Digital cross/square/triangle/circle button events ---
    // Cross button - LED1 momentary control
    if (Ps3.event.button_down.cross)
    {
        SerialDebug("Cross pressed");
    }
    if (Ps3.event.button_up.cross)
    {
        SerialDebug("Cross released");
    }

    // Triangle Button - LED2 toggle control
    if (Ps3.event.button_down.triangle)
    {
        SerialDebug("Triangle presssed");
    }
    if (Ps3.event.button_up.triangle)
        SerialDebug("Released the triangle button");
    if (Ps3.event.button_down.square)
        SerialDebug("Started pressing the square button");
    if (Ps3.event.button_up.square)
        SerialDebug("Released the square button");

    // Circle Button - play sound
    if (Ps3.event.button_down.circle)
    {
        SerialDebug("circle presssed");
        myDFPlayer.playNext();
    }
    if (Ps3.event.button_up.circle)
        SerialDebug("Released the circle button");

    //--------------- Digital D-pad button events --------------
    if (Ps3.event.button_down.up)
    {
        if (setsoundvolume <= 29)
        {
            setsoundvolume++;
        }
    }
    if (Ps3.event.button_up.up)
        SerialDebug("Released the up button");

    if (Ps3.event.button_down.right)
    {
        tiltmechanism();
        liftmechanism();
    }
    if (Ps3.event.button_up.right)
        SerialDebug("Released the right button");

    if (Ps3.event.button_down.down)
    {
        if (setsoundvolume >= 2)
        {
            setsoundvolume--;
        }
    }
    if (Ps3.event.button_up.down)
        SerialDebug("Released the down button");

    if (Ps3.event.button_down.left)
    {
        SerialDebug("Pressed the left button");
    }
    if (Ps3.event.button_up.left)
        SerialDebug("Released the left button");

    //------------- Digital shoulder button events -------------
    if (Ps3.event.button_down.l1)
        SerialDebug("Started pressing the left shoulder button");
    if (Ps3.event.button_up.l1)
        SerialDebug("Released the left shoulder button");

    if (Ps3.event.button_down.r1)
        SerialDebug("Started pressing the right shoulder button");
    if (Ps3.event.button_up.r1)
        SerialDebug("Released the right shoulder button");

    //-------------- Digital trigger button events -------------
    if (Ps3.event.button_down.l2)
        SerialDebug("Started pressing the left trigger button");
    if (Ps3.event.button_up.l2)
        SerialDebug("Released the left trigger button");

    if (Ps3.event.button_down.r2)
        SerialDebug("Started pressing the right trigger button");
    if (Ps3.event.button_up.r2)
        SerialDebug("Released the right trigger button");

    //--------------- Digital stick button events --------------
    if (Ps3.event.button_down.l3)
        SerialDebug("Started pressing the left stick button");
    if (Ps3.event.button_up.l3)
        SerialDebug("Released the left stick button");

    if (Ps3.event.button_down.r3)
        SerialDebug("Started pressing the right stick button");
    if (Ps3.event.button_up.r3)
        SerialDebug("Released the right stick button");

    //---------- Digital select/start/ps button events ---------
    if (Ps3.event.button_down.select)
        SerialDebug("Started pressing the select button");
    if (Ps3.event.button_up.select)
        SerialDebug("Released the select button");

    if (Ps3.event.button_down.start)
        SerialDebug("Started pressing the start button");
    if (Ps3.event.button_up.start)
        SerialDebug("Released the start button");

    if (Ps3.event.button_down.ps)
        SerialDebug("Started pressing the Playstation button");
    if (Ps3.event.button_up.ps)
        SerialDebug("Released the Playstation button");

    // DFPlayermini sounds

    // set volume dpad up and down

    myDFPlayer.volume(setsoundvolume);

    switch (player)
    {
    case 0:
        speedX = map(leftX, 128, -128, -255, 255);
        speedY = map(leftY, 128, -128, -255, 255);
        speedZ = map(rightX, -128, 128, -255, 255);
        break;
    case 1:
        speedX = map(rightX, 128, -128, -255, 255);
        speedY = map(rightY, 128, -128, -255, 255);
        speedZ = map(leftX, -128, 128, -255, 255);
        break;
    case 2:
        break;
    case 3:
        break;
    case 4:
        break;
    case 5:
        break;
    }

    moveLegs(speedX, speedY);
    moveDome(speedZ);
}

// On Connection function
void PS3onConnect()
{
    // Print to Serial Monitor
    Serial.println("PS3 Controller Connected.....");
}
/////////////////////////////////////////////////////////////////////////////

#pragma region Servo_Motor_Setup
// Servo Board 1 (Body) Setup
void servoSetup1()
{
    xLeftArm.pwmInitialise();
    xRightArm.pwmInitialise();
    pwm1.setPWM(2, 0, 0); // Not in Use
    xCenterLift.pwmInitialise();
    xTilt.pwmInitialise();
    pwm1.setPWM(5, 0, 0);  // Not in Use
    pwm1.setPWM(6, 0, 0);  // Not in Use
    pwm1.setPWM(7, 0, 0);  // Not in Use
    pwm1.setPWM(8, 0, 0);  // Not in Use
    pwm1.setPWM(9, 0, 0);  // Not in Use
    pwm1.setPWM(10, 0, 0); // Not in Use
    pwm1.setPWM(11, 0, 0); // Not in Use
    pwm1.setPWM(12, 0, 0); // Not in Use
    pwm1.setPWM(13, 0, 0); // Not in Use
    pwm1.setPWM(14, 0, 0); // Not in Use
    pwm1.setPWM(15, 0, 0); // Not in Use
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// Dome PCA9685 module, lights and Holo proector servos, board will require the A0 tab to be bridged with solder
// Pulse ranges for the servo positions to the PCA9685 board 0-180 to 150-600
// Servo Board 2 (Dome) Setup
void servoSetup2()
{
    xDomeLED1.pwmInitialise();
    xDomeLED2.pwmInitialise();
    xDomeLED3.pwmInitialise();
    xDomeLED4.pwmInitialise();
    xHolo1.pwmInitialise();
    xHolo2.pwmInitialise();
    xHolo3.pwmInitialise();
    xDomeLED5.pwmInitialise();
    pwm2.setPWM(8, 0, 0); // Not in Use
    pwm2.setPWM(9, 0, 0); // Not in Use
    xPeriscope.pwmInitialise();
    xPeriscopeLED.pwmInitialise();
    pwm2.setPWM(12, 0, 0); //  Not in Use
    xHolo1LED.pwmInitialise();
    xHolo2LED.pwmInitialise();
    xHolo3LED.pwmInitialise();
}

// Motor Settings

/// @brief Initialise the motor settings
/// @param in_pin controller pin
/// @param motorChannel Motor Channel
void setupMotor(const uint8_t in_pin, const uint8_t motorChannel)
{
    const uint32_t motorFreq = 1000;
    const uint8_t motorResolution = 8;
    pinMode(in_pin, OUTPUT);
    digitalWrite(in_pin, LOW);
    ledcSetup(motorChannel, motorFreq, motorResolution);
    ledcAttachPin(in_pin, motorChannel);
    ledcWrite(motorChannel, 0);
}

#pragma endregion Servo_Motor_Setup

// SPIFFS File Operations
#pragma region SPIFFS_Operations

const String pathRoot = "/";
const String SettingsExtension = ".json";

/// @brief Read file contents to String
/// @param fs File System object (SPIFFS)
/// @param path Path to file in SPIFFS
/// @return String Contents of specified file
String readFile(fs::FS &fs, const char *path)
{
    File file = fs.open(path);
    if (!file || file.isDirectory())
    {
        SerialDebug("- failed to open file for reading - " + String(path));
        return "";
    }

    SerialDebug("- read from file:");
    return file.readString();
    file.close();
}

/// @brief Wite string to file
/// @param fs File System object (SPIFFS)
/// @param path Path to file in SPIFFS
/// @param message String Contents to write to specified file
void writeFile(fs::FS &fs, const char *path, String message)
{
    File file = fs.open(path, FILE_WRITE);
    if (!file)
    {
        SerialDebug("- failed to open file for writing");
        return;
    }
    if (file.print(message))
    {
        SerialDebug("- file written");
    }
    else
    {
        SerialDebug("- write failed");
    }
    file.close();
}

/// @brief List files in Directory
/// @param fs File System object (SPIFFS)
/// @param dirname Path to file in SPIFFS
/// @param levels how many directory levels to walk.
void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    File root = fs.open(dirname);
    if (!root)
    {
        SerialDebug("- failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        SerialDebug(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            SerialDebug(file.name());
            if (levels)
            {
                listDir(fs, file.path(), levels - 1);
            }
        }
        else
        {
            SerialDebug("  FILE: ");
            SerialDebug(file.name());
            SerialDebug("\tSIZE: ");
            SerialDebug(String(file.size()));
        }
        file = root.openNextFile();
    }
}

// PWMOutput Settings persistance

/// @brief Read settings from file to PWMOutput object
/// @param pPWMOP r2PWMoutput object to update settings.
void readSettingsFromFilePWMOutput(r2PWMoutput &pPWMOP)
{
    r2PWMoutput *PWMoutput = &pPWMOP;
    String x = PWMoutput->PWMName;
    SerialDebug(x);

    String path = pathRoot + x + SettingsExtension;
    SerialDebug(path);
    int str_len = path.length() + 1;
    char char_array[str_len];
    path.toCharArray(char_array, str_len);
    String lJson = readFile(SPIFFS, char_array);
    if (lJson.length() > 1)
    {
        PWMoutput->setJSON(lJson);
    }
}

/// @brief  Write settings from file to PWMOutput object
/// @param pPWMOP r2PWMoutput object to update settings
void writeSettingsToFilePWMOutput(r2PWMoutput &pPWMOP)
{
    r2PWMoutput *PWMoutput = &pPWMOP;
    String x = PWMoutput->PWMName;
    SerialDebug(x);
    String path = pathRoot + x + SettingsExtension;
    SerialDebug(path);
    int str_len = path.length() + 1;
    char char_array[str_len];
    path.toCharArray(char_array, str_len);
    String Json = PWMoutput->getJson();
    int str_len1 = Json.length() + 1;
    char char_array1[str_len1];
    Json.toCharArray(char_array1, str_len1);

    writeFile(SPIFFS, char_array, char_array1);
}

/// @brief Read settings from file to all  PWMOutput objects
void readAllPWMOutputSettings()
{
    readSettingsFromFilePWMOutput(xLeftArm);
    readSettingsFromFilePWMOutput(xRightArm);
    readSettingsFromFilePWMOutput(xCenterLift);
    readSettingsFromFilePWMOutput(xTilt);
    readSettingsFromFilePWMOutput(xDomeLED1);
    readSettingsFromFilePWMOutput(xDomeLED2);
    readSettingsFromFilePWMOutput(xDomeLED3);
    readSettingsFromFilePWMOutput(xDomeLED4);
    readSettingsFromFilePWMOutput(xHolo1);
    readSettingsFromFilePWMOutput(xHolo2);
    readSettingsFromFilePWMOutput(xHolo3);
    readSettingsFromFilePWMOutput(xDomeLED5);
    readSettingsFromFilePWMOutput(xPeriscope);
    readSettingsFromFilePWMOutput(xPeriscopeLED);
    readSettingsFromFilePWMOutput(xHolo1LED);
    readSettingsFromFilePWMOutput(xHolo2LED);
    readSettingsFromFilePWMOutput(xHolo3LED);
}

/// @brief Save settings to file for all PWMOutput objects
void writeAllPWMOutputSettings()
{

    SerialDebug("WriteSettings");

    writeSettingsToFilePWMOutput(xLeftArm);
    writeSettingsToFilePWMOutput(xRightArm);
    writeSettingsToFilePWMOutput(xCenterLift);
    writeSettingsToFilePWMOutput(xTilt);
    writeSettingsToFilePWMOutput(xDomeLED1);
    writeSettingsToFilePWMOutput(xDomeLED2);
    writeSettingsToFilePWMOutput(xDomeLED3);
    writeSettingsToFilePWMOutput(xDomeLED4);
    writeSettingsToFilePWMOutput(xHolo1);
    writeSettingsToFilePWMOutput(xHolo2);
    writeSettingsToFilePWMOutput(xHolo3);
    writeSettingsToFilePWMOutput(xDomeLED5);
    writeSettingsToFilePWMOutput(xPeriscope);
    writeSettingsToFilePWMOutput(xPeriscopeLED);
    writeSettingsToFilePWMOutput(xHolo1LED);
    writeSettingsToFilePWMOutput(xHolo2LED);
    writeSettingsToFilePWMOutput(xHolo3LED);
}

#pragma endregion SPIFFS_Operations

/// @brief Update PWMOutput settings from web interface and save file
/// @param pwmName Output Name
/// @param pwmSetting Setting to update
/// @param value Value of setting to set
void setValuePWM(const String pwmName, const String pwmSetting, const int value)
{
    r2PWMoutput *PWMOutput = &getPWMByName(pwmName);
    writeSettingsToPWMOutput(*PWMOutput, pwmSetting, value);
    writeSettingsToFilePWMOutput(*PWMOutput);
}

/// @brief Update PWMOutput settings from web interface
/// @param pPWMOP PWM Output Object
/// @param pwmSetting Setting to update
/// @param value  Value of setting to set
void writeSettingsToPWMOutput(r2PWMoutput &pPWMOP, const String pwmSetting, const int value)
{
    r2PWMoutput *PWMoutput = &pPWMOP;
    PWMoutput->setValue(pwmSetting, value);
}

// Web server functions
String cbSelected = "Left_Arm";
const char *input_parameter1 = "output";
const char *input_parameter2 = "state";

/// @brief Retrieve the PWMOutput object based on its name.
/// @param var Name of the PWMoutput object
/// @return PWMOutupt Identified by Name
r2PWMoutput &getPWMByName(const String var)
{
    if (var == "Left_Arm")
        return xLeftArm;
    if (var == "Right_Arm")
        return xRightArm;
    if (var == "Center_Lift")
        return xCenterLift;
    if (var == "Tilt")
        return xTilt;
    if (var == "Holo1")
        return xHolo1;
    if (var == "Holo2")
        return xHolo2;
    if (var == "Holo3")
        return xHolo3;
    if (var == "Periscope")
        return xPeriscope;
    if (var == "Holo1_LED")
        return xHolo1LED;
    if (var == "Holo2_LED")
        return xHolo2LED;
    if (var == "Holo3_LED")
        return xHolo3LED;
    if (var == "Dome_LED1")
        return xDomeLED1;
    if (var == "Dome_LED2")
        return xDomeLED2;
    if (var == "Dome_LED3")
        return xDomeLED3;
    if (var == "Dome_LED4")
        return xDomeLED4;
    if (var == "Dome_LED5")
        return xDomeLED5;
    if (var == "Periscope_LED")
        return xPeriscopeLED;
}

#pragma region Web_Page_Code
/// @brief Get A html String for the Settings for the Named PWMOutput Object
/// @param var PWMOutput Name
/// @return HTML String
String getDisplaySettings(const String var)
{
    r2PWMoutput *PWMOutput = &getPWMByName(var);
    String html = PWMOutput->getSettingsHTML();
    return html;
}

/// @brief
/// @param var
void pwmOutputInit(const String var)
{
    r2PWMoutput *PWMOutput = &getPWMByName(var);
    PWMOutput->pwmInitialise();
}

/// @brief
/// @param var
/// @return
String htmlPWMProcessor(const String var)
{
    if (var == "Combo")
    {
        return "<select name=\"size\" id=\"size\" style=\"font-size: 22px;\" onchange=\"cb(this)\" >"
               "<option value=\"" +
               xLeftArm.PWMName + "\"> " + xLeftArm.PWMName + "   </option>"
                                                              "<option value=\"" +
               xRightArm.PWMName + "\"> " + xRightArm.PWMName + " </option>"
                                                                "<option value=\"" +
               xCenterLift.PWMName + "\"> " + xCenterLift.PWMName + " </option>"
                                                                    "<option value=\"" +
               xTilt.PWMName + "\"> " + xTilt.PWMName + " </option>"
                                                        "<option value=\"" +
               xHolo1.PWMName + "\"> " + xHolo1.PWMName + " </option>"
                                                          "<option value=\"" +
               xHolo2.PWMName + "\"> " + xHolo2.PWMName + " </option>"
                                                          "<option value=\"" +
               xHolo3.PWMName + "\"> " + xHolo3.PWMName + " </option>"
                                                          "<option value=\"" +
               xPeriscope.PWMName + "\"> " + xPeriscope.PWMName + " </option>"
                                                                  "<option value=\"" +
               xHolo1LED.PWMName + "\"> " + xHolo1LED.PWMName + " </option>"
                                                                "<option value=\"" +
               xHolo2LED.PWMName + "\"> " + xHolo2LED.PWMName + " </option>"
                                                                "<option value=\"" +
               xHolo3LED.PWMName + "\"> " + xHolo3LED.PWMName + " </option>"
                                                                "<option value=\"" +
               xDomeLED1.PWMName + "\"> " + xDomeLED1.PWMName + " </option>"
                                                                "<option value=\"" +
               xDomeLED2.PWMName + "\"> " + xDomeLED2.PWMName + " </option>"
                                                                "<option value=\"" +
               xDomeLED3.PWMName + "\"> " + xDomeLED3.PWMName + " </option>"
                                                                "<option value=\"" +
               xDomeLED4.PWMName + "\"> " + xDomeLED4.PWMName + " </option>"
                                                                "<option value=\"" +
               xDomeLED5.PWMName + "\"> " + xDomeLED5.PWMName + " </option>"
                                                                "<option value=\"" +
               xPeriscopeLED.PWMName + "\"> " + xPeriscopeLED.PWMName + " </option>"
                                                                        "</select>";
    }
    if (var == "data")
    {
        return getDisplaySettings(cbSelected);
    }
    return String();
}
/// @brief
/// @param var
/// @return
String htmlControllerProcessor(const String var)
{
    if (var == "Combo")
    {
        return "";
    }
    if (var == "data")
    {
        String Settings = MACaddress;

        Settings = "PS3 Mac Address <input type=\"text\" id=\"mac\" value=\"" + Settings + "\"><br>";
        Settings += "<button type='button' id= \"save\" value=\"home\" onclick=\"btn(this)\">Save</button>";
        return Settings;
    }
    return "";
}

/// @brief
void http()
{

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", String(), false); });
    server.on("/Outputs", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/servos.html", String(), false, htmlPWMProcessor); });
    server.on("/Controller", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/psController.html", String(), false, htmlControllerProcessor); });
}

size_t content_len;
#define U_PART U_SPIFFS

/// @brief
void httpget()
{
    server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        String message;
        if (request->hasParam(PARAM_MESSAGE)) {
            message = request->getParam(PARAM_MESSAGE)->value();
        } else {
            message = "No message sent";
        }
        request->send(200, "text/plain", "Hello, GET: " + message); });
}

/// @brief
void setupHTMLTemplates()
{
    writeFile(SPIFFS, "/servos.html", R2D2HTML.getPWMOutputSetupHTML());

    writeFile(SPIFFS, "/index.html", R2D2HTML.getIndexHTML());

    writeFile(SPIFFS, "/psController.html", R2D2HTML.getpsControllerHTML());
}

void tbPost()
{
    String inputMessage1;
    String inputMessage2;

    server.on("/updateTextBox", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/updateTextBox?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(input_parameter1) && request->hasParam(input_parameter2)) {
      inputMessage1 = request->getParam(input_parameter1)->value();
      inputMessage2 = request->getParam(input_parameter2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    SerialDebug(cbSelected);
    SerialDebug("Value: ");
    SerialDebug(inputMessage1);
    SerialDebug(" - Set to: ");
    SerialDebug(inputMessage2);
    //cbSelected = inputMessage2;
    setValuePWM(cbSelected,inputMessage1,inputMessage2.toInt());
    request->send(200, "text/plain", "OK"); });
}
void comboPost()
{
    String inputMessage1;
    String inputMessage2;

    /// Combo Update Code
    server.on("/updatecombo", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(input_parameter1) && request->hasParam(input_parameter2)) {
      inputMessage1 = request->getParam(input_parameter1)->value();
      inputMessage2 = request->getParam(input_parameter2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    SerialDebug("Combo Updated: ");
    
    SerialDebug(inputMessage1);
    SerialDebug(" - Set to: ");
    SerialDebug(inputMessage2);
    cbSelected = inputMessage2;
    request->send(200, "text/plain", "OK"); 
    events.send(String(getDisplaySettings(inputMessage2)).c_str(),"pwmoutput",millis()); });
}

void btnPost()
{
    String inputMessage1;
    String inputMessage2;

    // home
    server.on("/home", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(input_parameter1) && request->hasParam(input_parameter2)) {
      inputMessage1 = request->getParam(input_parameter1)->value();
      inputMessage2 = request->getParam(input_parameter2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }

    
    SerialDebug("Home For: ");
    SerialDebug(inputMessage1);
    SerialDebug(" - Set to: ");
    SerialDebug(inputMessage2);
    request->send(200, "text/plain", "OK"); 
    
    if(inputMessage2=="home")
    {   
        SerialDebug("Home:"+cbSelected);
         pwmOutputInit(cbSelected); 
    } });
}

void macPost()
{
    String inputMessage1;
    String inputMessage2;

    /// Slider Update Code
    server.on("/savemac", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  String inputMessage1;
                  String inputMessage2;
                  // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
                  if (request->hasParam(input_parameter1) && request->hasParam(input_parameter2))
                  {
                      inputMessage1 = request->getParam(input_parameter1)->value();
                      inputMessage2 = request->getParam(input_parameter2)->value();
                      // digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
                  }
                  else
                  {
                      inputMessage1 = "No message sent";
                      inputMessage2 = "No message sent";
                  }
                  SerialDebug("MAC Updated: ");
                  SerialDebug(inputMessage1);
                  SerialDebug(" - Set to: ");
                  SerialDebug(inputMessage2);
                  request->send(200, "text/plain", "OK");
                  MACaddress = inputMessage2;
                  writeFile(SPIFFS, "/mac.txt", MACaddress);
                  ESP.restart(); });
}

void sliderPost()

{
    String inputMessage1;
    String inputMessage2;

    server.on("/updateSlider", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(input_parameter1) && request->hasParam(input_parameter2)) {
      inputMessage1 = request->getParam(input_parameter1)->value();
      inputMessage2 = request->getParam(input_parameter2)->value();
      //digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    SerialDebug("Slider Updated: ");
    
    SerialDebug(inputMessage1);
    SerialDebug(" - Set to: ");
    SerialDebug(inputMessage2);
    request->send(200, "text/plain", "OK"); 
    r2PWMoutput *PWMOutput = &getPWMByName(cbSelected);
    PWMOutput->setPWM(0, inputMessage2.toInt(), true); });
}
/// @brief
void httppost()
{
    tbPost();
    comboPost();
    sliderPost();
    btnPost();
    macPost();
}

int lastL = 0;
int lastR = 0;

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    if (SingleConnection())
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
        {
            data[len] = 0;
            String msg = String((char *)data);
            int commaIndex = msg.indexOf(',');
            int LValue = msg.substring(0, commaIndex).toInt();
            int RValue = msg.substring(commaIndex + 1).toInt();
            if (lastL != LValue && lastR != RValue)
            {
                lastL = LValue;
                lastR = RValue;
                moveLegs(RValue, LValue);
            }
        }
    }
}

// i

#pragma endregion Web_Page_Code

HardwareSerial SerialPort(1); // use UART1

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}

void setup()
{
    //  Setup Serial Monitor for testing
    Serial.begin(115200);
    delay(2000);

    // Initialise SPIFFS
    if (!SPIFFS.begin(true))
    {
        SerialDebug("SPIFFS Mount Failed");
        return;
    }
    SerialDebug("SPIFFS Mounted");

    // Read Saved PWM Outut Settings from File System
    readAllPWMOutputSettings();
    String tMACAddress = readFile(SPIFFS, "/mac.txt");
    if (tMACAddress != "")
    {
        MACaddress = tMACAddress;
    }

    SerialDebug("Settings Read");

    // Start the Web Server
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    Serial.println("Mini Droid..... HTTP:)");

    // Write the latest HTML to the SPIFFS
    setupHTMLTemplates();
    // List All Files in the SPIFFS
    listDir(SPIFFS, "/", 0);

    // Setup the Web Server Callbacks
    http(); //
    httpget();
    httppost();
    // Web server async events for Settings change and other commands
    server.addHandler(&events);
    // Start the web Server
    AsyncElegantOTA.begin(&server);
    server.begin();
    ws.onEvent(onEvent);
    server.addHandler(&ws);

    SerialDebug("WebServer started");

    ///////Serial1 for dfplayermini communication
    SerialPort.begin(15200, SERIAL_8N1, 4, 2);
    Serial2.begin(9600);

    // Setup the DFPlayerMini
    myDFPlayer.begin(Serial2);
    myDFPlayer.volume(setsoundvolume); // initial start up volume
    myDFPlayer.playNext();             // Play the first mp3

    // setup the PS3 Controller
    //  Define Callback Function
    Ps3.attach(PS3notify);
    // Define On Connection Function
    Ps3.attachOnConnect(PS3onConnect);
    // Emulate console as specific MAC address (change as required)
    int str_len = MACaddress.length() + 1;
    char char_array[str_len];
    MACaddress.toCharArray(char_array, str_len);
    Ps3.begin(char_array);

    // Init Servo Boards
    pwm1.begin();
    pwm1.setPWMFreq(50); // standard for analog servos
    pwm2.begin();
    pwm2.setPWMFreq(50);

    // Set motor controller pins as outputs
    setupMotor(IN1_PIN, motorAChannel1);
    setupMotor(IN2_PIN, motorAChannel2);
    setupMotor(IN3_PIN, motorBChannel1);
    setupMotor(IN4_PIN, motorBChannel2);
    setupMotor(IN5_PIN, motorCChannel1);
    setupMotor(IN6_PIN, motorCChannel2);

    // Set Servos amd LED's
    servoSetup1();
    servoSetup2();
    // Print to Serial Monitor
    Serial.println("Mini Droid..... :)");
    Serial.println(password);
    Serial.println(MACaddress);
}

/////////////////////////////////////////////////////////////////////////////

void loop()
{
    currentMillis = millis();
    xDomeLED1.RandomTrigger(currentMillis);
    xDomeLED2.RandomTrigger(currentMillis);
    xDomeLED3.RandomTrigger(currentMillis);
    xDomeLED4.RandomTrigger(currentMillis);
    xDomeLED5.RandomTrigger(currentMillis);
    xHolo1LED.RandomTrigger(currentMillis);
    xHolo2LED.RandomTrigger(currentMillis);
    xHolo3LED.RandomTrigger(currentMillis);
    xHolo1.RandomTrigger(currentMillis);
    xHolo2.RandomTrigger(currentMillis);
    xHolo3.RandomTrigger(currentMillis);

    periscopeLED();

    bool tester = true;
    ws.cleanupClients();
    if (!SingleConnection())
    {
        Serial.println("Connect PS3 or a Single Web Controller..... :)");
        // if no connection or multiple connections turn motors off.
        // validation on moveDome and MoveLegs will prevent conflicting commands being sent
        ledcWrite(motorAChannel1, 0);
        ledcWrite(motorAChannel2, 0);
        ledcWrite(motorBChannel1, 0);
        ledcWrite(motorBChannel2, 0);
        ledcWrite(motorCChannel1, 0);
        ledcWrite(motorCChannel2, 0);
        delay(1000);
    }
    else if (Ps3.isConnected())
    {
        Serial.println("PS3 Controller Connected..... :)");
        delay(1000);
    }
    else if (ws.count() == 1)
    {
        Serial.println("Web Controller Connected..... :)");
        delay(1000);
    }
}
/////////////////////////////////////////////////////////////////////////////
