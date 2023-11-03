#ifndef ARDUINO_R2PWMOUTPUT_H
#define ARDUINO_R2PWMOUTPUT_H
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>

/// @brief Class that stores the Valid data for the PWMOutputs
class r2PWMoutput
{
private:
	int degtopulse(int degree);
	int LEDInterval = 0;
	int PWMAddress;
	unsigned long previousMillis = 0;
	String getValueSettingsHTML();
	String getContServoSettingsHTML();
	String getLEDSettingsHTML();
	String inputBox(const String label, const String pmin, const String pmax, String ID, int Value);
	String inputSlider(const String label, const String pmin, const String pmax, String ID, int Value);
	String getHomeHTML();
	Adafruit_PWMServoDriver *PWMBoard;

public:
	enum PWMEType
	{
		/// @brief 180 Deg positionable Servo
		Servo,
		/// @brief 360 Deg Continuous Rotation Servo
		ContinuousServo,
		/// @brief Standard 5mm LED
		LED,
		/// @brief TBA
		RGBLED,
		/// @brief Disabled PWM Output.
		Disabled
	};
	enum PWMEInitialise
	{
		/// @brief Servo PWMMin Value
		Min,
		/// @brief Servo PWMMax Value
		Max,
		/// @brief LED/Continuous Servo ON
		On,
		/// @brief LED/Continuous Servo OFF
		Off
	};
	enum PWMEMode
	{
		/// @brief Toggle with controller btn
		Manual,
		/// @brief Random motion time based
		Random
	};

	String PWMName;
	PWMEType PWMType;
	int PWMMin = -1;
	int PWMMax = -1;
	/// @brief
	int PWMMin2 = -1;
	/// @brief
	int PWMMax2 = -1;
	int PWMOff = -1;
	int PWMState = LOW;
	int PWMOFFTimeMin = -1;
	int PWMOFFTimeMax = -1;
	PWMEMode PWMMode = Random;
	PWMEInitialise PWMHome;
	void servoMin();
	void servoMax();

	void ON();
	void OFF();
	void RandomTrigger(int pCurrentMillis);
	void pwmInitialise();
	r2PWMoutput(String pPWMName, Adafruit_PWMServoDriver &PWMBoard, int PWMAddress, PWMEType PWMType, int PWMMin, int PWMMax, int PWMMin2, int PWMMax2, PWMEInitialise PWMHome, PWMEMode PWMMode = Random);
	r2PWMoutput(String pPWMName, Adafruit_PWMServoDriver &PWMBoard, int PWMAddress, PWMEType PWMType, int PWMMin, int PWMMax, int PWMMin2, int PWMMax2, int PWMOff, int PWMOFFTimeMin, int PWMOFFTimeMax, PWMEInitialise PWMHome, PWMEMode PWMMode = Random);
	void setPWM(int iOn, int iOff, bool bConstrain = false);
	void setValue(String pwmSeting, int value);
	char *getName();
	String getJson();
	String getSettingsHTML();
	void setJSON(String doc);
};
#endif
