/* i2Head_Receiver  RF NANO Prímacovy kod 
 // https://blog.laskakit.cz/projekt-rc-arduino/
*/
 


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include "Servo_Min_Max.h"
#include "i2Head_Receiver.h"

//Servo PWM2;
//Servo PWM3;
//Servo PWM4;
//Servo PWM5;
//Méžeme mať až 32 kanalov
struct MyData {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
  byte ch6;
  byte ch7;
  byte ch8;
};
MyData data;
const uint64_t pipeIn = 0x0022;   //Tento isty kod musi mať aj primač
RF24 radio(10,9);  //zapojenie CE a CSN pinov
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long previousMillis = 0;
const long interval = 20;


void resetData()
{
//Definujeme iniciálnu hodnotu každého vstupu údajov
// potenciometre budú v strednej pozícii, takže 127 je v centre od 254
  data.ch1 = 127;
  data.ch2 = 127;
  data.ch3 = 127;
  data.ch4 = 127;
  data.ch5 = 127;
  data.ch6 = 127;
  data.ch7 = 127;
  data.ch8 = 127;

}

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println();
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);

  Serial.println("setup:: Servo Initialization started");
	delay(200);
  pwm.begin(); //pwm.begin(0);   0 = driver_ID
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	Serial.println("setup: Servos on PCA9685  attached");
  center_points_initialized = false;
  delay(20);
  
  //konfiguracia NRF24 
  resetData();
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipeIn);
    //začneme s rádiokomunikáciou
  radio.startListening();
}
unsigned long lastRecvTime = 0;
void recvData()
{
  while ( radio.available() ) {
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis(); //tu dostávame údaje
  }
}
void loop()
{
  //recvData();
  //unsigned long now = millis();
  //Tu skontrolujeme, či sme stratili signál, ak ano, tak by sme resetovali hodnoty
  //if ( now - lastRecvTime > 1200 ) {
  //  //Stratený signál?
  //  resetData();
  //}
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
    previousMillis = currentMillis;
  
    recvData();

    ch_1_constrained = constrain(data.ch1, 0, 1023);
    ch_2_constrained = constrain(data.ch2, 0, 1023);
    ch_3_constrained = constrain(data.ch3, 0, 1023);
    ch_4_constrained = constrain(data.ch4, 0, 1023);
    ch_5_constrained = constrain(data.ch5, 0, 1023);
    ch_6_constrained = constrain(data.ch6, 0, 1023);
    ch_7_constrained = constrain(data.ch7, 0, 1023);
    ch_8_constrained = constrain(data.ch8, 0, 1023);
    if(center_points_initialized == false) {
      if(data.ch1 != 127 || data.ch2 != 127  || data.ch3 != 127  || data.ch4 != 127  || data.ch5 != 127  || data.ch6 != 127  || data.ch7 != 127  || data.ch8 != 127 ) {
        ch_1_center = data.ch1;
        ch_2_center = data.ch2;
        ch_3_center = data.ch3;
        ch_4_center = data.ch4;
        ch_5_center = data.ch5;
        ch_6_center = data.ch6;
        ch_7_center = data.ch7;
        ch_8_center = data.ch8;
        center_points_initialized = true;
        Serial.println("@0 Chanels, Center points:"+String(data.ch1)+", "+String(data.ch2)+" | "+String(data.ch3)+", "+String(data.ch4)+" || "+String(data.ch5)+", "+String(data.ch6)+" | "+String(data.ch7)+", "+String(data.ch8));
      }else {
        Serial.println("@0 Chanels, waiting for received data. Center points not set.");
      }
    }

    //Serial.println("@1 Chanels:"+String(data.ch1)+", "+String(data.ch2)+" | "+String(data.ch3)+", "+String(data.ch4)+" || "+String(data.ch5)+", "+String(data.ch6)+" | "+String(data.ch7)+", "+String(data.ch8));
    //Serial.println("@2 Chanels:"+String(ch_1_constrained)+", "+String(ch_2_constrained)+" | "+String(ch_3_constrained)+", "+String(ch_4_constrained)+" || "+String(ch_5_constrained)+", "+String(ch_6_constrained)+" | "+String(ch_7_constrained)+", "+String(ch_8_constrained));

    if(center_points_initialized==true) {
      //Nastavenie koncovych a stred polôh
      ch_1 = (ch_1_constrained < ch_1_center ? map(ch_1_constrained,  110, ch_1_center, 1023, 512) : map(ch_1_constrained,  ch_1_center, 222, 512,    0));  //PWM vystup digital pin D1 čierny  //198
      ch_2 = (ch_2_constrained < ch_2_center ? map(ch_2_constrained,   78, ch_2_center, 1023, 512) : map(ch_2_constrained,  ch_2_center, 255, 512,    0)); //2000);  //PWM vystup digital pin D2 žltý    //116
      ch_3 = (ch_3_constrained < ch_3_center ? map(ch_3_constrained,  104, ch_3_center,    0, 512) : map(ch_3_constrained,  ch_3_center, 240, 512, 1023)); //2000);  //PWM vystup digital pin D3 modrý   //135
      ch_4 = (ch_4_constrained < ch_4_center ? map(ch_4_constrained,   80, ch_4_center, 1023, 512) : map(ch_4_constrained,  ch_4_center, 240, 512, 1023)); //2000);  //PWM vystup digital pin D4 červeny //115

      ch_5 = (ch_5_constrained < ch_5_center ? map(ch_5_constrained,  110, ch_5_center, 1023, 512) : map(ch_5_constrained,  ch_5_center, 240, 512,    0)); //2000);  //PWM vystup digital pin D5 čierny  //197
      ch_6 = (ch_6_constrained < ch_6_center ? map(ch_6_constrained,   78, ch_6_center, 1023, 512) : map(ch_6_constrained,  ch_6_center, 240, 512,    0)); //2000);  //PWM vystup digital pin D6 žltý    //113
      ch_7 = (ch_7_constrained < ch_7_center ? map(ch_7_constrained,  104, ch_7_center, 0   , 512) : map(ch_7_constrained,  ch_7_center, 240, 512, 1023)); //2000);  //PWM vystup digital pin D7 modrý   //128
      ch_8 = (ch_8_constrained < ch_8_center ? map(ch_8_constrained,   80, ch_8_center, 1023, 512) : map(ch_8_constrained,  ch_8_center, 240, 512,    0)); //2000);  //PWM vystup digital pin D8 červeny //113

      //Serial.println("@3 Chanels:"+String(ch_1)+"("+String(data.ch1)+", c "+String(ch_1_center)+"), "+String(ch_2)+"("+String(data.ch2)+", c "+String(ch_2_center)+") | "+String(ch_3)+"("+String(data.ch3)+", c "+String(ch_3_center)+"), "+String(ch_4)+"("+String(data.ch4)+", c "+String(ch_4_center)+") || "+String(ch_5)+"("+String(data.ch5)+", c "+String(ch_5_center)+"), "+String(ch_6)+"("+String(data.ch6)+", c "+String(ch_6_center)+") | "+String(ch_7)+"("+String(data.ch7)+", c "+String(ch_7_center)+"), "+String(ch_8)+"("+String(data.ch8)+", c "+String(ch_8_center)+")");

      servo_eyeLeftUD_Angle       = ch_1;
      servo_eyeLeftLR_Angle       = ch_2;
      servo_eyeRightUD_Angle      = ch_1;
      servo_eyeRightLR_Angle      = ch_2;

      servo_eyelidLeftUpper_Angle = ch_1 + (ch_3 - 512) + (ch_4 - 512);
      servo_eyelidLeftLower_Angle = ch_2 - (ch_3 - 512) - (ch_4 - 512);
      servo_eyelidRightUpper_Angle= ch_1 + (ch_3 - 512) - (ch_4 - 512);
      servo_eyelidRightLower_Angle= ch_2 - (ch_3 - 512) + (ch_4 - 512);

      servo_eyebrowRight_Angle    = ch_5 + (ch_6 - 512);
      servo_eyebrowLeft_Angle     = ch_5 - (ch_6 - 512);
      
      servo_cheekRight_Angle      = ch_7 + (ch_8 - 512);
      servo_cheekLeft_Angle       = ch_7 - (ch_8 - 512);
      
      servo_upperLip_Angle        = 0;
      
      servo_forheadRight_Angle    = ch_6 + (ch_5 - 512);
      servo_forheadLeft_Angle     = ch_6 - (ch_5 - 512);
      servo_Jaw_UpDown_Angle      = 0;


      servo_eyeLeftUD_Angle       =map(servo_eyeLeftUD_Angle       , 0, 1023, 1023, 0); 
      servo_eyeLeftLR_Angle       =map(servo_eyeLeftLR_Angle       , 0, 1023, 0, 1023); 
      servo_eyeRightUD_Angle      =map(servo_eyeRightUD_Angle      , 0, 1023, 0, 1023); 
      servo_eyeRightLR_Angle      =map(servo_eyeRightLR_Angle      , 0, 1023, 0, 1023);
      servo_eyelidLeftUpper_Angle =map(servo_eyelidLeftUpper_Angle , 0, 1023, 0, 1023); 
      servo_eyelidLeftLower_Angle =map(servo_eyelidLeftLower_Angle , 0, 1023, 0, 1023); 
      servo_eyelidRightUpper_Angle=map(servo_eyelidRightUpper_Angle, 0, 1023, 0, 1023); 
      servo_eyelidRightLower_Angle=map(servo_eyelidRightLower_Angle, 0, 1023, 0, 1023);
      servo_eyebrowRight_Angle    =map(servo_eyebrowRight_Angle    , 0, 1023, 0, 1023); 
      servo_eyebrowLeft_Angle     =map(servo_eyebrowLeft_Angle     , 0, 1023, 0, 1023);
      servo_cheekRight_Angle      =map(servo_cheekRight_Angle      , 0, 1023, 0, 1023); 
      servo_cheekLeft_Angle       =map(servo_cheekLeft_Angle       , 0, 1023, 0, 1023);
      servo_upperLip_Angle        =map(servo_upperLip_Angle        , 0, 1023, 0, 1023);
      servo_forheadRight_Angle    =map(servo_forheadRight_Angle    , 0, 1023, 0, 1023);
      servo_forheadLeft_Angle     =map(servo_forheadLeft_Angle     , 0, 1023, 0, 1023);
      servo_Jaw_UpDown_Angle      =map(servo_Jaw_UpDown_Angle      , 0, 1023, 0, 1023); 

      servo_eyeLeftUD_Pwm       = map(servo_eyeLeftUD_Angle       , 0, 1023, SERVO_MIN_eyeLeftUD ,       SERVO_MAX_eyeLeftUD);
      servo_eyeLeftLR_Pwm       = map(servo_eyeLeftLR_Angle       , 0, 1023, SERVO_MIN_eyeLeftLR ,       SERVO_MAX_eyeLeftLR);
      servo_eyeRightUD_Pwm      = map(servo_eyeRightUD_Angle      , 0, 1023, SERVO_MIN_eyeRightUD,       SERVO_MAX_eyeRightUD);
      servo_eyeRightLR_Pwm      = map(servo_eyeRightLR_Angle      , 0, 1023, SERVO_MIN_eyeRightLR,       SERVO_MAX_eyeRightLR); 
      servo_eyelidLeftUpper_Pwm = map(servo_eyelidLeftUpper_Angle , 0, 1023, SERVO_MIN_eyelidLeftUpper,  SERVO_MAX_eyelidLeftUpper);
      servo_eyelidLeftLower_Pwm = map(servo_eyelidLeftLower_Angle , 0, 1023, SERVO_MIN_eyelidLeftLower,  SERVO_MAX_eyelidLeftLower);
      servo_eyelidRightUpper_Pwm= map(servo_eyelidRightUpper_Angle, 0, 1023, SERVO_MIN_eyelidRightUpper, SERVO_MAX_eyelidRightUpper);
      servo_eyelidRightLower_Pwm= map(servo_eyelidRightLower_Angle, 0, 1023, SERVO_MIN_eyelidRightLower, SERVO_MAX_eyelidRightLower);
      servo_eyebrowRight_Pwm    = map(servo_eyebrowRight_Angle    , 0, 1023, SERVO_MIN_eyebrowRight,     SERVO_MAX_eyebrowRight);
      servo_eyebrowLeft_Pwm     = map(servo_eyebrowLeft_Angle     , 0, 1023, SERVO_MIN_eyebrowLeft,      SERVO_MAX_eyebrowLeft);
      servo_cheekRight_Pwm      = map(servo_cheekRight_Angle      , 0, 1023, SERVO_MIN_cheekRight,       SERVO_MAX_cheekRight);
      servo_cheekLeft_Pwm       = map(servo_cheekLeft_Angle       , 0, 1023, SERVO_MIN_cheekLeft,        SERVO_MAX_cheekLeft);
      servo_upperLip_Pwm        = map(servo_upperLip_Angle        , 0, 1023, SERVO_MIN_upperLip,         SERVO_MAX_upperLip);
      servo_forheadRight_Pwm    = map(servo_forheadRight_Angle    , 0, 1023, SERVO_MIN_forheadRight,     SERVO_MAX_forheadRight);
      servo_forheadLeft_Pwm     = map(servo_forheadLeft_Angle     , 0, 1023, SERVO_MIN_forheadLeft,      SERVO_MAX_forheadLeft);
      servo_Jaw_UpDown_Pwm      = map(servo_Jaw_UpDown_Angle      , 0, 1023, SERVO_MIN_Jaw_UpDown,       SERVO_MAX_Jaw_UpDown);

      pwm.setPWM( i01_head_eyeLeftUD       , 0, servo_eyeLeftUD_Pwm);
      pwm.setPWM( i01_head_eyeLeftLR       , 0, servo_eyeLeftLR_Pwm);
      
      pwm.setPWM( i01_head_eyeRightUD      , 0, servo_eyeRightUD_Pwm);
      pwm.setPWM( i01_head_eyeRightLR      , 0, servo_eyeRightLR_Pwm);
      /*
      pwm.setPWM( i01_head_eyelidLeftUpper , 0, servo_eyelidLeftUpper_Pwm);
      pwm.setPWM( i01_head_eyelidLeftLower , 0, servo_eyelidLeftLower_Pwm);
      pwm.setPWM( i01_head_eyelidRightUpper, 0, servo_eyelidRightUpper_Pwm);
      pwm.setPWM( i01_head_eyelidRightLower, 0, servo_eyelidRightLower_Pwm);

      pwm.setPWM( i01_head_eyebrowRight    , 0, servo_eyebrowRight_Pwm);
      pwm.setPWM( i01_head_eyebrowLeft     , 0, servo_eyebrowLeft_Pwm);

      pwm.setPWM( i01_head_cheekRight      , 0, servo_cheekRight_Pwm);
      pwm.setPWM( i01_head_cheekLeft       , 0, servo_cheekLeft_Pwm);

      pwm.setPWM( i01_head_upperLip        , 0, servo_upperLip_Pwm);

      pwm.setPWM( i01_head_forheadRight    , 0, servo_forheadRight_Pwm);
      pwm.setPWM( i01_head_forheadLeft     , 0, servo_forheadLeft_Pwm);

      pwm.setPWM( Jaw_UpDown               , 0, servo_Jaw_UpDown_Pwm);
      */
    //Serial.println("@4 Chanels:data.ch1 = "+String(data.ch1)+", ch_1 = "+String(ch_1)+", servo_eyeLeftUD_Angle = "+String(servo_eyeLeftUD_Angle)+", servo_eyeLeftUD_Pwm = "+String(servo_eyeLeftUD_Pwm)+", || data.ch2 = "+String(data.ch2)+", ch_2 = "+String(ch_2)+", servo_eyeLeftLR_Angle = "+String(servo_eyeLeftLR_Angle)+", servo_eyeLeftLR_Pwm = "+String(servo_eyeLeftLR_Pwm)+".");
    Serial.println("@5 Chanels:data.ch1 = "+String(data.ch1)+", ch_1 = "+String(ch_1)+", servo_eyeRightUD_Angle = "+String(servo_eyeRightUD_Angle)+", servo_eyeRightUD_Pwm = "+String(servo_eyeRightUD_Pwm)+", || data.ch2 = "+String(data.ch2)+", ch_2 = "+String(ch_2)+", servo_eyeRightLR_Angle = "+String(servo_eyeRightLR_Angle)+", servo_eyeRightLR_Pwm = "+String(servo_eyeRightLR_Pwm)+".");

    }
  }

//Teraz napíšeme signál PWM pomocou funkcie servo
//PWM2.writeMicroseconds(ch_2);
//PWM3.writeMicroseconds(ch_3);
//PWM4.writeMicroseconds(ch_4);
//PWM5.writeMicroseconds(ch_5);

}//Koniec slučky