#ifndef WritePulsesToDisplay_h
#define WritePulsesToDisplay_h

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


#define lookUpDown    1
#define lookLeftRight 2
#define lidLowerLeft  3
#define lidUpperLeft  4
#define lidLowerRight 5
#define lidUpperRight 6


class WritePulsesToDisplay{
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
  uint8_t * left_arrow_step;
	
  //ST7735 * tft;
  
  //#define OLED_RESET 4
  #define DISP_CS    6 //CS   -CS
  #define DISP_RS    7 //A0   -RS
  #define DISP_RST   8 //RESET-RST
  #define DISP_SID   4 //SDA  -SDA
  #define DISP_SCLK  5 //SCK  -SCK
  //#define LEFT_ARROW_SIZE  2
  //#define LEFT_ARROW_STEP  2 //moved to colors.h

    //           ST7735(uint8_t CS, uint8_t RS, uint8_t SID, uint8_t SCLK, uint8_t RST);
    ST7735 tft = ST7735(   DISP_CS,    DISP_RS,    DISP_SID,    DISP_SCLK,    DISP_RST); 
  //ST7735 tft = ST7735(         6,          7,          11,           13,           8); 
    //           ST7735(uint8_t CS, uint8_t RS, uint8_t RST);
  //ST7735 tft = ST7735(6, 7, 8);    


uint8_t spacing = 8;
uint8_t yPos = 2;
uint8_t servoNum = 0;

//char servo[]="S";
//char colon[]=":";

	WritePulsesToDisplay();
  //Stream *theStream

	void begin();//ST7735 *theTft);//, int[] );
	
  //Stream *_stream;
  


  //int localServoLimits[48];

  //void WritePulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MIN, uint16_t servo_Pwm, uint16_t SERVO_MAX);
  
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
  void writeArrow_activeServoSet (byte activeServoSet);
  void drawString(uint8_t x, uint8_t y, char *c, uint16_t color, uint8_t size=1);


};
#endif
