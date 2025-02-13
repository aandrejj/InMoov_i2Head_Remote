#ifndef ServoMinMidMaxValues_h
#define ServoMinMidMaxValues_h

#include "Arduino.h"
#include "Servo_Min_Max.h"
//#include "i2Head_Receiver.h"

#define SERVOS_COUNT 19

class ServoMinMidMaxValues{
 public:
  ServoMinMidMaxValues();

  int servoLimits[SERVOS_COUNT*4]={
  SERVO_MIN_eyeLeftUD        ,
  SERVO_MIN_eyeLeftLR        ,
  SERVO_MIN_eyeRightUD       ,
  SERVO_MIN_eyeRightLR       ,
  SERVO_MIN_eyelidLeftUpper  ,
  SERVO_MIN_eyelidLeftLower  ,
  SERVO_MIN_eyelidRightUpper ,
  SERVO_MIN_eyelidRightLower ,
  SERVO_MIN_eyebrowRight     ,
  SERVO_MIN_eyebrowLeft      ,

  SERVO_MIN_cheekRight_Upper,
  SERVO_MIN_cheekLeft_Upper ,

  SERVO_MIN_cheekRight_Lower,
  SERVO_MIN_cheekLeft_Lower ,

  SERVO_MIN_upperLip         ,
  SERVO_MIN_forheadRight     ,
  SERVO_MIN_forheadLeft      ,
  SERVO_MIN_Jaw_UpDown       ,
    SERVO_MID_eyeLeftUD       ,
    SERVO_MID_eyeLeftLR       ,
    SERVO_MID_eyeRightUD      ,
    SERVO_MID_eyeRightLR      ,
    SERVO_MID_eyelidLeftUpper ,
    SERVO_MID_eyelidLeftLower ,
    SERVO_MID_eyelidRightUpper,
    SERVO_MID_eyelidRightLower,
    SERVO_MID_eyebrowRight    ,
    SERVO_MID_eyebrowLeft     ,

    SERVO_MID_cheekRight_Upper,
    SERVO_MID_cheekLeft_Upper ,

    SERVO_MID_cheekRight_Lower,
    SERVO_MID_cheekLeft_Lower ,

    SERVO_MID_upperLip        ,
    SERVO_MID_forheadRight    ,
    SERVO_MID_forheadLeft     ,
    SERVO_MID_Jaw_UpDown      ,

  SERVO_MAX_eyeLeftUD       ,
  SERVO_MAX_eyeLeftLR       ,
  SERVO_MAX_eyeRightUD      ,
  SERVO_MAX_eyeRightLR      ,
  SERVO_MAX_eyelidLeftUpper ,
  SERVO_MAX_eyelidLeftLower ,
  SERVO_MAX_eyelidRightUpper,
  SERVO_MAX_eyelidRightLower,
  SERVO_MAX_eyebrowRight    ,
  SERVO_MAX_eyebrowLeft     ,
  
  SERVO_MAX_cheekRight_Upper,
  SERVO_MAX_cheekLeft_Upper ,

  SERVO_MAX_cheekRight_Lower,
  SERVO_MAX_cheekLeft_Lower ,

  SERVO_MAX_upperLip        ,
  SERVO_MAX_forheadRight    ,
  SERVO_MAX_forheadLeft     ,
  SERVO_MAX_Jaw_UpDown      ,

    SERVO_MID_eyeLeftUD       ,
    SERVO_MID_eyeLeftLR       ,
    SERVO_MID_eyeRightUD      ,
    SERVO_MID_eyeRightLR      ,
    SERVO_MID_eyelidLeftUpper ,
    SERVO_MID_eyelidLeftLower ,
    SERVO_MID_eyelidRightUpper,
    SERVO_MID_eyelidRightLower,
    SERVO_MID_eyebrowRight    ,
    SERVO_MID_eyebrowLeft     ,

    SERVO_MID_cheekRight_Upper,
    SERVO_MID_cheekLeft_Upper ,

    SERVO_MID_cheekRight_Lower,
    SERVO_MID_cheekLeft_Lower ,

    SERVO_MID_upperLip        ,
    SERVO_MID_forheadRight    ,
    SERVO_MID_forheadLeft     ,
    SERVO_MID_Jaw_UpDown      
};

int prevServoLimits[SERVOS_COUNT*4]={
    SERVO_MIN_eyeLeftUD        ,
    SERVO_MIN_eyeLeftLR        ,
    SERVO_MIN_eyeRightUD       ,
    SERVO_MIN_eyeRightLR       ,
    SERVO_MIN_eyelidLeftUpper  ,
    SERVO_MIN_eyelidLeftLower  ,
    SERVO_MIN_eyelidRightUpper ,
    SERVO_MIN_eyelidRightLower ,
    SERVO_MIN_eyebrowRight     ,
    SERVO_MIN_eyebrowLeft      ,

    SERVO_MIN_cheekRight_Upper,
    SERVO_MIN_cheekLeft_Upper ,

    SERVO_MIN_cheekRight_Lower,
    SERVO_MIN_cheekLeft_Lower ,

    SERVO_MIN_upperLip         ,
    SERVO_MIN_forheadRight     ,
    SERVO_MIN_forheadLeft      ,
    SERVO_MIN_Jaw_UpDown       ,

   SERVO_MID_eyeLeftUD       ,
   SERVO_MID_eyeLeftLR       ,
   SERVO_MID_eyeRightUD      ,
   SERVO_MID_eyeRightLR      ,
   SERVO_MID_eyelidLeftUpper ,
   SERVO_MID_eyelidLeftLower ,
   SERVO_MID_eyelidRightUpper,
   SERVO_MID_eyelidRightLower,
   SERVO_MID_eyebrowRight    ,
   SERVO_MID_eyebrowLeft     ,

    SERVO_MID_cheekRight_Upper,
    SERVO_MID_cheekLeft_Upper ,

    SERVO_MID_cheekRight_Lower,
    SERVO_MID_cheekLeft_Lower ,

   SERVO_MID_upperLip        ,
   SERVO_MID_forheadRight    ,
   SERVO_MID_forheadLeft     ,
   SERVO_MID_Jaw_UpDown      ,

    SERVO_MAX_eyeLeftUD       ,
    SERVO_MAX_eyeLeftLR       ,
    SERVO_MAX_eyeRightUD      ,
    SERVO_MAX_eyeRightLR      ,
    SERVO_MAX_eyelidLeftUpper ,
    SERVO_MAX_eyelidLeftLower ,
    SERVO_MAX_eyelidRightUpper,
    SERVO_MAX_eyelidRightLower,
    SERVO_MAX_eyebrowRight    ,
    SERVO_MAX_eyebrowLeft     ,

    SERVO_MAX_cheekRight_Upper,
    SERVO_MAX_cheekLeft_Upper ,

    SERVO_MAX_cheekRight_Lower,
    SERVO_MAX_cheekLeft_Lower ,

    SERVO_MAX_upperLip        ,
    SERVO_MAX_forheadRight    ,
    SERVO_MAX_forheadLeft     ,
    SERVO_MAX_Jaw_UpDown      ,

   SERVO_MID_eyeLeftUD       ,
   SERVO_MID_eyeLeftLR       ,
   SERVO_MID_eyeRightUD      ,
   SERVO_MID_eyeRightLR      ,
   SERVO_MID_eyelidLeftUpper ,
   SERVO_MID_eyelidLeftLower ,
   SERVO_MID_eyelidRightUpper,
   SERVO_MID_eyelidRightLower,
   SERVO_MID_eyebrowRight    ,
   SERVO_MID_eyebrowLeft     ,

    SERVO_MID_cheekRight_Upper,
    SERVO_MID_cheekLeft_Upper ,

    SERVO_MID_cheekRight_Lower,
    SERVO_MID_cheekLeft_Lower ,

   SERVO_MID_upperLip        ,
   SERVO_MID_forheadRight    ,
   SERVO_MID_forheadLeft     ,
   SERVO_MID_Jaw_UpDown      
  };


};
#endif