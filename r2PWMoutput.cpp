#include "r2PWMoutput.h"

/// @brief Instantiate the PWM Output
/// @param pPWMName Name of the PWM Output
/// @param pPWMBoard PWM Servo Driver Board Reference
/// @param pPWMAddress Address of the object on the board
/// @param pPWMType Servo/LED/Continous Servos
/// @param pPWMMin Min Servo Position
/// @param pPWMMax Max Servo Position
/// @param pPWMMin2 Constrain Min
/// @param pPWMMax2 Constrain Max
/// @param pPWMHome Home/Initial Setting
/// @param PWMMode Random/Manual
r2PWMoutput::r2PWMoutput(String pPWMName, Adafruit_PWMServoDriver &pPWMBoard, int pPWMAddress, PWMEType pPWMType, int pPWMMin, int pPWMMax, int pPWMMin2, int pPWMMax2, PWMEInitialise pPWMHome, PWMEMode pPWMMode)
{
    PWMName = pPWMName;
    PWMBoard = &pPWMBoard;
    PWMAddress = pPWMAddress;
    PWMType = pPWMType;
    PWMMin = pPWMMin;
    PWMMax = pPWMMax;
    PWMMin2 = pPWMMin2;
    PWMMax2 = pPWMMax2;
    PWMHome = pPWMHome;
    PWMMode = pPWMMode;
}

/// @brief Instantiate the PWM Output
/// @param pPWMName Name of the PWM Output
/// @param pPWMBoard PWM Servo Driver Board Reference
/// @param pPWMAddress Address of the object on the board
/// @param pPWMType Servo/LED/Continous Servos
/// @param pPWMMin Min Servo Position
/// @param pPWMMax Max Servo Position
/// @param pPWMMin2 Constrain Min
/// @param pPWMMax2 Constrain Max
/// @param PWMOff
/// @param PWMOFFTimeMin
/// @param PWMOFFTimeMax
/// @param PWMHome Home/Initial Setting
/// @param PWMMode Random/Manual
r2PWMoutput::r2PWMoutput(String pPWMName, Adafruit_PWMServoDriver &pPWMBoard, int pPWMAddress, PWMEType pPWMType, int pPWMMin, int pPWMMax, int pPWMMin2, int pPWMMax2, int pPWMOff, int pPWMOFFTimeMin, int pPWMOFFTimeMax, PWMEInitialise pPWMHome, PWMEMode pPWMMode)
{
    PWMName = pPWMName;
    PWMBoard = &pPWMBoard;
    PWMAddress = pPWMAddress;
    PWMType = pPWMType;
    PWMMin = pPWMMin;
    PWMMax = pPWMMax;
    PWMMin2 = pPWMMin2;
    PWMMax2 = pPWMMax2;
    PWMOff = pPWMOff;
    PWMHome = pPWMHome;
    PWMOFFTimeMin = pPWMOFFTimeMin;
    PWMOFFTimeMax = pPWMOFFTimeMax;
    PWMMode = pPWMMode;
}

/// @brief Convert Degrees to Pulses
/// @param degree Location in Degrees
/// @return pwm Pulses
int r2PWMoutput::degtopulse(int degree)
{
    return (((450 / 180) * degree) + 150);
}
/// @brief Drive Servo to MIN position
void r2PWMoutput::servoMin()
{
    setPWM(0, degtopulse(PWMMin));
}
/// @brief Drive Servo to Max position
void r2PWMoutput::servoMax()
{
    setPWM(0, degtopulse(PWMMax));
}

/// @brief Turn LED On Full
void r2PWMoutput::ON()
{
    PWMState = LOW;
    setPWM(4096, 0);
}
/// @brief Turn LED/Continuous Servo Off
void r2PWMoutput::OFF()
{
    switch (PWMType)
    {
    case LED:
        PWMState = HIGH;
        setPWM(0, 4096);
        break;
    case ContinuousServo:
        setPWM(0, degtopulse(PWMOff));
        break;
    }
}
/// @brief Random Function to Move Servo or turn LED on/off
/// @param pCurentmillis Current Time in Milliseconds
void r2PWMoutput::RandomTrigger(int pCurentmillis)
{
    if (pCurentmillis - previousMillis >= LEDInterval)
    {
        previousMillis = pCurentmillis;
        switch (PWMMode)
        {
        case Random:
        {
            switch (PWMType)
            {
            case LED:

                if (PWMState == LOW)
                {
                    LEDInterval = random(PWMMin * 1000, PWMMax * 1000); // set a random time delay
                    OFF();
                }
                else
                {
                    LEDInterval = random(PWMMin2 * 1000, PWMMax2 * 1000); // set a random time delay
                    ON();
                }

                break;
            case ContinuousServo:
                if (PWMState == LOW)
                {
                    LEDInterval = random(PWMMin2 * 1000, PWMMax2 * 1000); // set a random time delay
                    long pos1 = random(PWMMin, PWMMax);
                    setPWM(0, pos1);
                    PWMState = HIGH;
                }
                else
                {
                    LEDInterval = random(PWMOFFTimeMin * 1000, PWMOFFTimeMax * 1000); // set a random time delay
                    OFF();
                    PWMState = LOW;
                }
                break;
            case Servo:
                /* code */
                break;
            case RGBLED:
                /* code */
                break;

            default:
                break;
            }
            break;
        }
        default:
            break;
        }
    }
}

/// @brief Initialise LED/Servo to "Home" Setting
void r2PWMoutput::pwmInitialise()
{

    switch (PWMHome)
    {
    case Min:
        servoMin();
        break;
    case Max:
        servoMax();
        break;
    case On:
        ON();
        break;
    case Off:
        OFF();
        break;
    }
}
/// @brief Set value of PWMSettings Based on Setting name
/// @param pwmSeting
/// @param value
void r2PWMoutput::setValue(const String pwmSeting, const int value)
{
    if (pwmSeting == "PWMMin")
        PWMMin = value;
    else if (pwmSeting == "PWMMax")
        PWMMax = value;
    else if (pwmSeting == "PWMMin2")
        PWMMin2 = value;
    else if (pwmSeting == "PWMMax2")
        PWMMax2 = value;
    else if (pwmSeting == "PWMHome")
        PWMHome = PWMEInitialise(value);
    else if (pwmSeting == "PWMOff")
        PWMOff = value;
    else if (pwmSeting == "PWMOFFTimeMin")
        PWMOFFTimeMin = value;
    else if (pwmSeting == "PWMOFFTimeMax")
        PWMOFFTimeMax = value;
    else if (pwmSeting == "PWMMode")
        PWMMode = PWMEMode(value);
}
/// @brief Set The PWM Pulses to the specified output
/// @param iOn On Value
/// @param iOff Off Value
/// @param bConstrain constrain based on constraint settings?
void r2PWMoutput::setPWM(const int iOn, int iOff, const bool bConstrain)
{
    if (PWMType == Servo && bConstrain)
    {
        int servoPos;
        int sMax = degtopulse(PWMMax);
        int sMin = degtopulse(PWMMin);
        servoPos = constrain(iOff, PWMMin2, PWMMax2);
        servoPos = map(servoPos, PWMMin2, PWMMax2, sMin, sMax);
        if (servoPos < sMax || servoPos > sMin)
        {
            servoPos = constrain(servoPos, sMin, sMax);
            iOff = servoPos;
        }
        else
        {
            iOff = sMin;
        }
    }
    else if (PWMType == LED)
    {
    }

    PWMBoard->setPWM(PWMAddress, iOn, iOff);
}

/// @brief Generate a number input box for the website
/// @param label Input Box Label
/// @param pmin Minimum number
/// @param pmax MAximum Number
/// @param Value current Value
/// @return HTML String
String r2PWMoutput::inputBox(const String label, const String pmin, const String pmax, String ID, int Value)
{
    if (Value == -1)
    {
        return "";
    }
    else
    {
        return label + "<input type=\"number\" min=\"" + pmin + "\" max=\"" + pmax + "\" onchange=\"tc(this)\" id=\"" + ID + "\" value=\"" + Value + "\"><br>";
    }
}
/// @brief Generate a Slider  for the website
/// @param label Input Box Label
/// @param pmin Minimum number
/// @param pmax MAximum Number
/// @param Value current Value
/// @return HTML String
String r2PWMoutput::inputSlider(const String label, const String pmin, const String pmax, String ID, int Value)
{
    if (PWMMax2 == -1||PWMMin2 == -1)
    {
        return "";
    }
    else
    {
        return label + "<input id=\"" + ID + "\" type=\"range\" min=\""+pmin+"\" max=\""+pmax+"\" step=\"1\" value=\""+Value+"\" onchange=\"sl(this)\">";
    }
}

/// @brief Generat the Dropdown List for the Home Settings
/// @return HTML String
String r2PWMoutput::getHomeHTML()
{
    String str1 = "";
    String str2 = "";
    String sReturn = "";
    switch (PWMHome)
    {
    case Min:
        str1 = "selected";
        break;
    case Max:
        str2 = "selected";
        break;
    case On:
        str1 = "selected";
        break;
    case Off:
        str2 = "selected";
        break;
    }

    switch (PWMType)
    {
    case Servo:
        sReturn += "Home<select name=\"home\" id=\"PWMHome\" onchange=\"tc(this)\">";
        sReturn = sReturn + "<option value=\"" + Min + "\" " + str1 + ">Min</option>";
        sReturn = sReturn + "<option value=\"" + Max + "\" " + str2 + ">Max</option>";
        sReturn += "</select>";
        break;
    case LED:
        sReturn += "Home<select name=\"home\" id=\"PWMHome\" onchange=\"tc(this)\">";
        sReturn = sReturn + "<option value=\"" + On + "\" " + str1 + ">On</option>";
        sReturn = sReturn + "<option value=\"" + Off + "\" " + str2 + ">Off</option>";
        sReturn += "</select>";
        break;

    default:

        break;
    }
    return sReturn;
}

/// @brief Generate the HTML For all Settins for this Object
/// @return HTML String
String r2PWMoutput::getValueSettingsHTML()
{
    String Settings = "";
    const String sMin = "0";
    const String sMax = "180";
    const String tMin = "0";
    const String tMax = "600";

    switch (PWMType)
    {
    case Servo:
        Settings += inputBox("Min:  ", sMin, sMax, "PWMMin", PWMMin);
        Settings += inputBox("Max:  ", sMin, sMax, "PWMMax", PWMMax);
        Settings += inputBox("Constrain Min:  ", sMin, sMax, "PWMMin2", PWMMin2);
        Settings += inputBox("Constrain Max:  ", sMin, sMax, "PWMMax2", PWMMax2);
        switch (PWMHome)
        {
        case Min:
            Settings += inputSlider("Servo Location","0","255",PWMName,0);
            break;
        
        case Max:
            Settings += inputSlider("Servo Location","0","255",PWMName,255);
            break;
        }
        break;
    case ContinuousServo:
        Settings += inputBox("PWM Off:  ", sMin, sMax, "PWMOff", PWMOff);
        Settings += inputBox("PWM Min(speed):  ", sMin, sMax, "PWMMin", PWMMin);
        Settings += inputBox("PWM Max(speed):  ", sMin, sMax, "PWMMax", PWMMax);
        Settings += inputBox("On Time(s) Min:  ", tMin, tMax, "PWMMin2", PWMMin2);
        Settings += inputBox("On Time(s) Max:  ", tMin, tMax, "PWMMax2", PWMMax2);
        Settings += inputBox("Off Time(s) Min:  ", tMin, tMax, "PWMOFFTimeMin", PWMOFFTimeMin);
        Settings += inputBox("Off Time(s) Max:  ", tMin, tMax, "PWMOFFTimeMax", PWMOFFTimeMax);
        break;
    case LED:
        Settings += inputBox("On Time(s) Min:  ", sMin, sMax, "PWMMin", PWMMin);
        Settings += inputBox("On Time(s) Max:  ", sMin, sMax, "PWMMax", PWMMax);
        Settings += inputBox("Off Time(s) Min:  ", tMin, tMax, "PWMMin2", PWMMin2);
        Settings += inputBox("Off Time(s) Max:  ", tMin, tMax, "PWMMax2", PWMMax2);
        break;

    default:
        break;
    }
    Settings += getHomeHTML();
    return Settings;
}

/// @brief Generate the HTML Page For all Settins for this Object
/// @return HTML String
String r2PWMoutput::getSettingsHTML()
{

    String settingsHTML = "<h4>Output - " + PWMName + "</h4>";

    settingsHTML += getValueSettingsHTML();

    settingsHTML += "<button type='button' id= \"1\" value=\"home\" onclick=\"btn(this)\">Home</button>";
   

    return settingsHTML;
}
/// @brief Return the Name String as a Char Array
/// @return char array contaning the value from PWMName
char *r2PWMoutput::getName()
{
    // Length (with one extra character for the null terminator)
    int str_len = PWMName.length() + 1;

    // Prepare the character array (the buffer)
    char char_array[str_len];

    // Copy it over
    PWMName.toCharArray(char_array, str_len);
    return char_array;
}
/// @brief Retrieve values from object as a JSON String
/// @return JSON String
String r2PWMoutput::getJson()
{
    String destination;
    StaticJsonDocument<100> PWMDOC;
    PWMDOC["PWMMin"] = PWMMin;
    PWMDOC["PWMMax"] = PWMMax;
    PWMDOC["PWMMin2"] = PWMMin2;
    PWMDOC["PWMMax2"] = PWMMax2;
    PWMDOC["PWMOff"] = PWMOff;
    PWMDOC["PWMHome"] = PWMHome;
    PWMDOC["PWMMode"] = PWMMode;
    PWMDOC["PWMOFFTimeMin"] = PWMOFFTimeMin;
    PWMDOC["PWMOFFTimeMax"] = PWMOFFTimeMax;
    serializeJson(PWMDOC, destination);
    return destination;
}
/// @brief Set on object based on the JSON String provided
/// @param json JSON String
void r2PWMoutput::setJSON(String json)
{
    StaticJsonDocument<200> PWMDOC;
    DeserializationError error = deserializeJson(PWMDOC, json);
    PWMMin = PWMDOC["PWMMin"];
    PWMMax = PWMDOC["PWMMax"];
    PWMMin2 = PWMDOC["PWMMin2"];
    PWMMax2 = PWMDOC["PWMMax2"];
    PWMOff = PWMDOC["PWMOff"];
    PWMHome = PWMDOC["PWMHome"];
    PWMOFFTimeMin = PWMDOC["PWMOFFTimeMin"];
    PWMOFFTimeMax = PWMDOC["PWMOFFTimeMax"];
    PWMMode = PWMDOC["PWMMode"];
}
