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
  delay(20);

  //Set the pins for each PWM signal
  //PWM2.attach(2);
  //PWM3.attach(3);
  //PWM4.attach(4);
  //PWM5.attach(5);
  
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



    //Nastavenie koncovych a stred polôh
    ch_1 = map(ch_1_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D1 čierny
    ch_2 = map(ch_2_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D2 žltý
    ch_3 = map(ch_3_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D3 modrý
    ch_4 = map(ch_4_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D4 červeny

    ch_5 = map(ch_5_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D5 čierny
    ch_6 = map(ch_6_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D6 žltý
    ch_7 = map(ch_7_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D7 modrý
    ch_8 = map(ch_8_constrained,  100, 255, 0, 1023); //2000);  //PWM vystup digital pin D8 červeny

    Serial.println("Chanels:"+String(ch_1)+", "+String(ch_2)+" | "+String(ch_3)+", "+String(ch_4)+" || "+String(ch_5)+", "+String(ch_6)+" | "+String(ch_7)+", "+String(ch_8));

    servo_eyeLeftUD_Angle       = ch_1_constrained;
    servo_eyeLeftLR_Angle       = ch_2_constrained;
    servo_eyeRightUD_Angle      = ch_1_constrained;
    servo_eyeRightLR_Angle      = ch_2_constrained;

    servo_eyelidLeftUpper_Angle = ch_1_constrained + (ch_3_constrained - 512) + (ch_4_constrained - 512);
    servo_eyelidLeftLower_Angle = ch_2_constrained - (ch_3_constrained - 512) - (ch_4_constrained - 512);
    servo_eyelidRightUpper_Angle= ch_1_constrained + (ch_3_constrained - 512) - (ch_4_constrained - 512);
    servo_eyelidRightLower_Angle= ch_2_constrained - (ch_3_constrained - 512) + (ch_4_constrained - 512);

    servo_eyebrowRight_Angle    = ch_5_constrained + (ch_6_constrained - 512);
    servo_eyebrowLeft_Angle     = ch_5_constrained - (ch_6_constrained - 512);
    
    servo_cheekRight_Angle      = ch_7_constrained + (ch_8_constrained - 512);
    servo_cheekLeft_Angle       = ch_7_constrained - (ch_8_constrained - 512);
    
    servo_upperLip_Angle        = 0;
    
    servo_forheadRight_Angle    = ch_6_constrained + (ch_5_constrained - 512);
    servo_forheadLeft_Angle     = ch_6_constrained - (ch_5_constrained - 512);
    servo_Jaw_UpDown_Angle      = 0;


    servo_eyeLeftUD_Angle       =map(servo_eyeLeftUD_Angle       , 0, 1023, 0, 1023); 
    servo_eyeLeftLR_Angle       =map(servo_eyeLeftLR_Angle       , 0, 1023, 0, 1023); 
    servo_eyeRightUD_Angle      =map(servo_eyeRightUD_Angle      , 0, 1023, 0, 1023); 
    servo_eyeRightLR_Angle      =map(servo_eyeRightLR_Angle      , 0, 1023, 0, 1023);     //------------
    servo_eyelidLeftUpper_Angle =map(servo_eyelidLeftUpper_Angle , 0, 1023, 0, 1023); 
    servo_eyelidLeftLower_Angle =map(servo_eyelidLeftLower_Angle , 0, 1023, 0, 1023); 
    servo_eyelidRightUpper_Angle=map(servo_eyelidRightUpper_Angle, 0, 1023, 0, 1023); 
    servo_eyelidRightLower_Angle=map(servo_eyelidRightLower_Angle, 0, 1023, 0, 1023);     //------------
    servo_eyebrowRight_Angle    =map(servo_eyebrowRight_Angle    , 0, 1023, 0, 1023); 
    servo_eyebrowLeft_Angle     =map(servo_eyebrowLeft_Angle     , 0, 1023, 0, 1023);     //------------
    servo_cheekRight_Angle      =map(servo_cheekRight_Angle      , 0, 1023, 0, 1023); 
    servo_cheekLeft_Angle       =map(servo_cheekLeft_Angle       , 0, 1023, 0, 1023);     //------------
    servo_upperLip_Angle        =map(servo_upperLip_Angle        , 0, 1023, 0, 1023);     //------------
    servo_forheadRight_Angle    =map(servo_forheadRight_Angle    , 0, 1023, 0, 1023);
    servo_forheadLeft_Angle     =map(servo_forheadLeft_Angle     , 0, 1023, 0, 1023);     //------------
    servo_Jaw_UpDown_Angle      =map(servo_Jaw_UpDown_Angle      , 0, 1023, 0, 1023); 

    pwm.setPWM( i01_head_eyeLeftUD       , 0, map(servo_eyeLeftUD_Angle       , 0, 1023, SERVO_MIN_eyeLeftUD ,       SERVO_MAX_eyeLeftUD));
    pwm.setPWM( i01_head_eyeLeftLR       , 0, map(servo_eyeLeftLR_Angle       , 0, 1023, SERVO_MIN_eyeLeftLR ,       SERVO_MAX_eyeLeftLR));
    pwm.setPWM( i01_head_eyeRightUD      , 0, map(servo_eyeRightUD_Angle      , 0, 1023, SERVO_MIN_eyeRightUD,       SERVO_MAX_eyeRightUD));
    pwm.setPWM( i01_head_eyeRightLR      , 0, map(servo_eyeRightLR_Angle      , 0, 1023, SERVO_MIN_eyeRightLR,       SERVO_MAX_eyeRightLR));

    pwm.setPWM( i01_head_eyelidLeftUpper , 0, map(servo_eyelidLeftUpper_Angle , 0, 1023, SERVO_MIN_eyelidLeftUpper,  SERVO_MAX_eyelidLeftUpper));
    pwm.setPWM( i01_head_eyelidLeftLower , 0, map(servo_eyelidLeftLower_Angle , 0, 1023, SERVO_MIN_eyelidLeftLower,  SERVO_MAX_eyelidLeftLower));
    pwm.setPWM( i01_head_eyelidRightUpper, 0, map(servo_eyelidRightUpper_Angle, 0, 1023, SERVO_MIN_eyelidRightUpper, SERVO_MAX_eyelidRightUpper));
    pwm.setPWM( i01_head_eyelidRightLower, 0, map(servo_eyelidRightLower_Angle, 0, 1023, SERVO_MIN_eyelidRightLower, SERVO_MAX_eyelidRightLower));

    pwm.setPWM( i01_head_eyebrowRight    , 0, map(servo_eyebrowRight_Angle    , 0, 1023, SERVO_MIN_eyebrowRight,     SERVO_MAX_eyebrowRight));
    pwm.setPWM( i01_head_eyebrowLeft     , 0, map(servo_eyebrowLeft_Angle     , 0, 1023, SERVO_MIN_eyebrowLeft,      SERVO_MAX_eyebrowLeft));

    pwm.setPWM( i01_head_cheekRight      , 0, map(servo_cheekRight_Angle      , 0, 1023, SERVO_MIN_cheekRight,       SERVO_MAX_cheekRight));
    pwm.setPWM( i01_head_cheekLeft       , 0, map(servo_cheekLeft_Angle       , 0, 1023, SERVO_MIN_cheekLeft,        SERVO_MAX_cheekLeft));

    pwm.setPWM( i01_head_upperLip        , 0, map(servo_upperLip_Angle        , 0, 1023, SERVO_MIN_upperLip,         SERVO_MAX_upperLip));

    pwm.setPWM( i01_head_forheadRight    , 0, map(servo_forheadRight_Angle    , 0, 1023, SERVO_MIN_forheadRight,     SERVO_MAX_forheadRight));
    pwm.setPWM( i01_head_forheadLeft     , 0, map(servo_forheadLeft_Angle     , 0, 1023, SERVO_MIN_forheadLeft,      SERVO_MAX_forheadLeft));

    pwm.setPWM( Jaw_UpDown               , 0, map(servo_Jaw_UpDown_Angle      , 0, 1023, SERVO_MIN_Jaw_UpDown,       SERVO_MAX_Jaw_UpDown));
  }

//Teraz napíšeme signál PWM pomocou funkcie servo
//PWM2.writeMicroseconds(ch_2);
//PWM3.writeMicroseconds(ch_3);
//PWM4.writeMicroseconds(ch_4);
//PWM5.writeMicroseconds(ch_5);

}//Koniec slučky