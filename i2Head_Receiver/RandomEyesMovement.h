#ifndef RandomEyesMovement_h
#define RandomEyesMovement_h

#include "Arduino.h"
#include <Adafruit_PWMServoDriver.h>

//#ifdef USE_DISPLAY_ST7735
  //#ifndef ST7735_h
    //#define ST7735_h
    #include <ST7735.h>
  //#endif
//#endif

#include "eyes_random_moves.h"
#include "Servo_Min_Max.h"
#include "ServoConnectionToPwm.h"
#include "colors.h"
#include "WritePulsesToDisplay.h"
#include "ServoMinMidMaxValues.h"


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

	unsigned long previousRemMillis;
  long REM_interval;
	long REM_pose;
  uint8_t * left_arrow_step;
  
	
	RandomEyesMovement();
  //Stream *theStream
	//void begin(Adafruit_PWMServoDriver *thePwm, ST7735 *theTft, int[] );
  void begin(Adafruit_PWMServoDriver *thePwm, WritePulsesToDisplay *theWritePulsesToDisplay, ServoMinMidMaxValues *theServoMinMidMaxValues);
	//void beginDisplay(ST7735 *theTft );
	
  //Stream *_stream;
  Adafruit_PWMServoDriver * pPwm;
  ServoMinMidMaxValues *servoMinMidMaxValues;

  // ST7735 * tft;
  WritePulsesToDisplay *writePulsesToDisplay;
  
  //int servoLimits[];
  int localServoLimits[64];

  void moveEyesRandomly(unsigned long currentMillis, String textToShow);
	void lookAtRandomDirection(bool generateRandomDirection, long minUpDown, long maxUpDown, String textToShow);
	void blink (int time);
	
	bool lookUpDown_write(byte servo_angle);
	bool lookLeftRight_write(byte servo_angle);
	bool lidLowerLeft_write(byte servo_angle);
	bool lidUpperLeft_write( byte servo_angle);
	bool lidLowerRight_write(byte servo_angle);
	bool lidUpperRight_write(byte servo_angle);
	
	bool servoSender_write(byte servo_angle, byte servoGroup);
 
  //void writePulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MIN, uint16_t servo_Pwm, uint16_t SERVO_MAX);
  
  //void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MIN);
  //void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm);
  //void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MAX);

  void writeOneFieldToDisplay_innerPart (uint8_t xPos, uint16_t _chr_point_shift_x, uint8_t yPos, uint16_t _char_height_y, uint16_t form_label_Min_Mid_Max, uint16_t servo_Pwm,uint16_t chanelNum, bool showDebug);
  void writeOneFieldToDisplay (uint8_t chanelNum,uint8_t form_label_Min_Mid_Max, uint16_t servo_Pwm, bool showDebug);

  void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug);
  void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm);
  void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug);
  void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm);
  void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug);
  void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm);
  void writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug);
  void writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm);


};
#endif
