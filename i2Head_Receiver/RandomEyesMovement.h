#ifndef RandomEyesMovement_h
#define RandomEyesMovement_h

#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>

#include "eyes_random_moves.h"
#include "Servo_Min_Max.h"
#include "ServoConnectionToPwm.h"

#define lookUpDown    1
#define lookLeftRight 2
#define lidLowerLeft  3
#define lidUpperLeft  4
#define lidLowerRight 5
#define lidUpperRight 6


class RandomEyesMovement{
	public:
	int UpDownState;
	int LeftRightState;
	int lidMod;
	int uplidpulse;
	int lolidpulse;
	int altuplidpulse;
	int altlolidpulse;

	long REM_interval;
	long REM_pose;
	
	RandomEyesMovement();
  //Stream *theStream
	void begin(Adafruit_PWMServoDriver *thePwm);
	
  //Stream *_stream;
  Adafruit_PWMServoDriver * pPwm;

  void moveEyesRandomly(unsigned long currentMillis);
	void lookAtRandomDirection(bool generateRandomDirection, long minUpDown, long maxUpDown, String textToShow);
	void blink (int time);
	
	bool lookUpDown_write(byte servo_angle);
	bool lookLeftRight_write(byte servo_angle);
	bool lidLowerLeft_write(byte servo_angle);
	bool lidUpperLeft_write( byte servo_angle);
	bool lidLowerRight_write(byte servo_angle);
	bool lidUpperRight_write(byte servo_angle);
	
	bool servoSender_write(byte servo_angle, byte servoGroup);

};
#endif
