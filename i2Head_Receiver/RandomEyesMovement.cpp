#include "Arduino.h"
#include "RandomEyesMovement.h"

RandomEyesMovement::RandomEyesMovement() {

}

void RandomEyesMovement::begin(Adafruit_PWMServoDriver *thePwm) {
    pPwm = thePwm;
    randomSeed(analogRead(A7));
}

void RandomEyesMovement::moveEyesRandomly(unsigned long currentMillis) {
  Serial.print("REM: Start");
  REM_interval = random(20,2000);
  //Serial.print(", REM_interval = "+String(REM_interval));
  REM_pose = random (0,3);
  //if (REM_pose>2) {REM_pose =2;}
  //Serial.println(", REM_pose = "+String(REM_pose));
  //delay(100);
  
  switch (REM_pose){
    case 0:
      Serial.println("REM:0.start'blink'1");
      RandomEyesMovement::blink(80);
      //Serial.println("REM:0.'blink' end");
      
      Serial.print("REM:0.  starting lookAtDirection(true,..) ");
      RandomEyesMovement::lookAtRandomDirection(true, 50, 130, "REM:0, ");
      Serial.println(" OK");
      
    break;
    case 1:
      Serial.print("REM:1. starting lookAtDirection(true,..)");      
      RandomEyesMovement::lookAtRandomDirection(true, 30, 130, "REM:1, ");
      Serial.println(" OK");
    break;
    case 2:
      Serial.println("REM:2.  starting 'blink'2....");
      RandomEyesMovement::blink(60);
      //Serial.println("REM:2.  back from 'blink'");
      
      Serial.print("REM:2.  starting lookAtDirection(false,...)");
      RandomEyesMovement::lookAtRandomDirection(false, 30, 130, "REM:2, ");
      Serial.println("");
      
    break;
  }
  
  delay(REM_interval);
}

void RandomEyesMovement::lookAtRandomDirection(bool generateRandomDirection, long minUpDown, long maxUpDown, String textToShow)
{
    if(generateRandomDirection == true) {
        UpDownState = random(minUpDown, maxUpDown);
        LeftRightState = random(30, 220);
        lidMod = ( 60 - UpDownState)/2;
        Serial.print("UD= "+String(UpDownState)+" LR= "+String(LeftRightState)+" lidMod="+String(lidMod)+" ");
        RandomEyesMovement::lookUpDown_write(UpDownState);
        RandomEyesMovement::lookLeftRight_write(LeftRightState);
    }

     RandomEyesMovement::lidUpperLeft_write(120+lidMod);// 70+lidMod);
     RandomEyesMovement::lidUpperRight_write(90+lidMod);//110-lidMod);

     RandomEyesMovement::lidLowerLeft_write(80+lidMod);//160+lidMod);
     RandomEyesMovement::lidLowerRight_write(80+lidMod);// 30-lidMod);
}

void RandomEyesMovement::blink (int time)
{
      RandomEyesMovement::lidUpperLeft_write(255);//ClosedEyes_eyelidLeftUpper_Angle);//400  //pwm.setPWM(2, 0, 400);
      RandomEyesMovement::lidLowerLeft_write(255);//ClosedEyes_eyelidLeftLower_Angle);//240 //pwm.setPWM(3, 0, 240);
      RandomEyesMovement::lidUpperRight_write(255);//ClosedEyes_eyelidRightUpper_Angle);//pwm.setPWM(4, 0, 240);
      RandomEyesMovement::lidUpperRight_write(255);//ClosedEyes_eyelidRightLower_Angle);//pwm.setPWM(5, 0, 400);
      delay(time);
      RandomEyesMovement::lookAtRandomDirection(false, 0, 0,"blink");
}
bool RandomEyesMovement::lookUpDown_write(byte servo_angle)   {return servoSender_write(servo_angle, lookUpDown   );}
bool RandomEyesMovement::lookLeftRight_write(byte servo_angle){return servoSender_write(servo_angle, lookLeftRight);}
bool RandomEyesMovement::lidLowerLeft_write(byte servo_angle) {return servoSender_write(servo_angle, lidLowerLeft );}
bool RandomEyesMovement::lidUpperLeft_write( byte servo_angle){return servoSender_write(servo_angle, lidUpperLeft );}
bool RandomEyesMovement::lidLowerRight_write(byte servo_angle){return servoSender_write(servo_angle, lidLowerRight);}
bool RandomEyesMovement::lidUpperRight_write(byte servo_angle){return servoSender_write(servo_angle, lidUpperRight);}

bool RandomEyesMovement::servoSender_write(byte servo_angle, byte servoGroup) {
	uint8_t chanelNum1;
	uint8_t chanelNum2;
	
	uint16_t  SERVO1_MIN;
	uint16_t  SERVO1_MID;
	uint16_t  SERVO1_MAX;
	uint16_t  SERVO2_MIN;
	uint16_t  SERVO2_MID;
	uint16_t  SERVO2_MAX;
	
	if (servoGroup == lookUpDown) {
		chanelNum1 = i01_head_eyeLeftUD;
		chanelNum2 = i01_head_eyeRightUD;
		
		SERVO1_MIN = SERVO_MAX_eyeLeftUD;  //invertovane 255=hore
		SERVO1_MID = SERVO_MID_eyeLeftUD;
		SERVO1_MAX = SERVO_MIN_eyeLeftUD;  //0 = dole
		SERVO2_MIN = SERVO_MIN_eyeRightUD;
		SERVO2_MID = SERVO_MID_eyeRightUD;
		SERVO2_MAX = SERVO_MAX_eyeRightUD;
	} 
	else if (servoGroup == lookLeftRight) {
		chanelNum1 = i01_head_eyeLeftLR;
		chanelNum2 = i01_head_eyeRightLR;
		
 		SERVO1_MIN = SERVO_MIN_eyeLeftLR;
		SERVO1_MID = SERVO_MID_eyeLeftLR;
		SERVO1_MAX = SERVO_MAX_eyeLeftLR;
		SERVO2_MIN = SERVO_MIN_eyeRightLR;
		SERVO2_MID = SERVO_MID_eyeRightLR;
		SERVO2_MAX = SERVO_MAX_eyeRightLR;
	}
	else if (servoGroup == lidLowerLeft) {
		chanelNum1 = i01_head_eyelidLeftLower;
		chanelNum2 = 99;
		
 		SERVO1_MIN = SERVO_MAX_eyelidLeftLower;
		SERVO1_MID = SERVO_MID_eyelidLeftLower;
		SERVO1_MAX = SERVO_MIN_eyelidLeftLower;
		SERVO2_MIN = 0;
		SERVO2_MID = 0;
		SERVO2_MAX = 0;
	}
	else if (servoGroup == lidUpperLeft) {
		chanelNum1 = i01_head_eyelidLeftUpper;
		chanelNum2 = 99;
		
 		SERVO1_MIN = SERVO_MIN_eyelidLeftUpper;
		SERVO1_MID = SERVO_MID_eyelidLeftUpper;
		SERVO1_MAX = SERVO_MAX_eyelidLeftUpper;
		SERVO2_MIN = 0;
		SERVO2_MID = 0;
		SERVO2_MAX = 0;
	}
	else if (servoGroup == lidLowerRight) {
		chanelNum1 = i01_head_eyelidRightLower;
		chanelNum2 = 99;
		
 		SERVO1_MIN = SERVO_MIN_eyelidRightLower;
		SERVO1_MID = SERVO_MID_eyelidRightLower;
		SERVO1_MAX = SERVO_MAX_eyelidRightLower;
		SERVO2_MIN = 0;
		SERVO2_MID = 0;
		SERVO2_MAX = 0;
	}
	else if (servoGroup == lidUpperRight) {
		chanelNum1 = i01_head_eyelidRightUpper;
		chanelNum2 = 99;
		
 		SERVO1_MIN = SERVO_MAX_eyelidRightUpper;//invertovane 255 =zatvorene
		SERVO1_MID = SERVO_MID_eyelidRightUpper;
		SERVO1_MAX = SERVO_MIN_eyelidRightUpper; //0 = otvorene
		SERVO2_MIN = 0;
		SERVO2_MID = 0;
		SERVO2_MAX = 0;
	} else {
    return false;
  }

    servo_angle = constrain(servo_angle, 0, 255);
	
    uint16_t  servo1_Pwm = (servo_angle < 128 ? map(servo_angle, 0, 127, SERVO1_MIN, SERVO1_MID ) : map(servo_angle, 128, 255, SERVO1_MID , SERVO1_MAX));
    uint16_t  servo2_Pwm = (servo_angle < 128 ? map(servo_angle, 0, 127, SERVO2_MIN, SERVO2_MID ) : map(servo_angle, 128, 255, SERVO2_MID , SERVO2_MAX));

    if(chanelNum1<99) {
      pPwm->setPWM( chanelNum1, 0, servo1_Pwm);
    }
    if(chanelNum2<99) {
      pPwm->setPWM( chanelNum2, 0, servo2_Pwm);
    }
  return true;
}
