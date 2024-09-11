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

#include "EasyTransfer.h"
#include "SoftwareSerial.h"
#include "TxRx_dataStructures.h"

//#define HIGHSPEED 
#define SERIAL_OUTPUT_LINE_RX 2  // Bluetooth RX -> Arduino D9
#define SERIAL_OUTPUT_LINE_TX 3 // Bluetooth TX -> Arduino D10

#ifdef HIGHSPEED
  #define Baud 38400   // Serial monitor
  #define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  #define BTBaud 9600  // HM-10, HM-19 etc
#endif

int16_t mode;
int count;
int noDataCount;


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

long previousSafetyMillis;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

unsigned long previousMillis_SerialLine = 0;
const long interval_SerialLine = 150;

SoftwareSerial serialOutputLine(SERIAL_OUTPUT_LINE_TX, SERIAL_OUTPUT_LINE_RX);
//create object
EasyTransfer serialLine; // send serial
//EasyTransfer ET1;   // send serial
//EasyTransfer ET2;   // rec serial

RX_DATA_STRUCTURE mydata_received;
TX_DATA_STRUCTURE mydata_remote;


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
  //all_center_points_initialized = false;
  delay(20);
  
  //konfiguracia NRF24 
  resetData();
  all_center_points_initialized = true;
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  
  radio.openReadingPipe(1,pipeIn);
    //začneme s rádiokomunikáciou
  radio.startListening();

  serialOutputLine.begin(BTBaud);

  serialLine.begin(details(mydata_received), &serialOutputLine);
  //ET1.begin(details(mydata_send), &serialOutputLine);
  //ET2.begin(details(mydata_remote), &serialOutputLine);

  Serial.println("setup:done. setup END.");

}



unsigned long lastRecvTime = 0;
bool recvData()
{
  bool dataReceived = false;
  while ( radio.available() ) {
    radio.read(&data, sizeof(MyData));
    lastRecvTime = millis(); //tu dostávame údaje
    dataReceived = true;
  }
  return dataReceived;
}

bool tmp_all_centers_initialized = false;
void setCenterPoints() {
    if(all_center_points_initialized == false) {

      tmp_all_centers_initialized = true;

      for (short i=0; i<=7; i++) {
        if(center_point_initialized[i]==false){
          if(abs(ch_constrained[i] - 127) < 2 ) {
            Serial.println("setCenterPoints: i="+String(i)+".");
            ch_center[i] = ch_constrained[i];
            center_point_initialized[i]=true;
          }
        }
        tmp_all_centers_initialized = tmp_all_centers_initialized && center_point_initialized[i];
      }//endfor
      if(tmp_all_centers_initialized == true) {
        Serial.println("setCenterPoints:all center points set. ");
        all_center_points_initialized = true;
      }

      if(all_center_points_initialized == true) {
        Serial.println("setCenterPoints: All center points set.  They are:  ----------------------------------------------------------------------");
        for (short i=0; i<=7; i++) {
          Serial.println("setCenterPoints: center_point_initialized["+String(i)+"] = "+String(center_point_initialized[i])+", ch_center["+String(i)+"] = "+String(ch_center[i])+".");
        }
        Serial.println("setCenterPoints: All center points set. End ----------------------------------------------------------------------");
      }
    } else {
        Serial.println("setCenterPoints: waiting for received data. Center points not set.");
    }
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
bool data_changed = false;
bool dataReceived = false;
void loop()
{
  dataReceived = false;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
    previousMillis = currentMillis;
  
    dataReceived = recvData();
    if(dataReceived) {
      ch_constrained[0] = constrain(data.ch1, 0, 255);
      ch_constrained[1] = constrain(data.ch2, 0, 255);
      ch_constrained[2] = constrain(data.ch3, 0, 255);
      ch_constrained[3] = constrain(data.ch4, 0, 255);
      ch_constrained[4] = constrain(data.ch5, 0, 255);
      ch_constrained[5] = constrain(data.ch6, 0, 255);
      ch_constrained[6] = constrain(data.ch7, 0, 255);
      ch_constrained[7] = constrain(data.ch8, 0, 255);
    } else {
      loop_ReadFromSerialLine(currentMillis);
    }


    if(all_center_points_initialized == false) {
      setCenterPoints();
    } else {

      ch[1] = ch_constrained[0];  //PWM vystup digital pin D1 čierny  //198
      ch[2] = ch_constrained[1];  //PWM vystup digital pin D2 žltý    //116
      ch[3] = ch_constrained[2];  //PWM vystup digital pin D3 modrý   //135
      ch[4] = ch_constrained[3];  //PWM vystup digital pin D4 červeny //115
      ch[5] = ch_constrained[4];  //PWM vystup digital pin D5 čierny  //197
      ch[6] = ch_constrained[5];  //PWM vystup digital pin D6 žltý    //113
      ch[7] = ch_constrained[6];  //PWM vystup digital pin D7 modrý   //128
      ch[8] = ch_constrained[7];  //PWM vystup digital pin D8 červeny //113

      data_changed = false;
      for (short i=0; i<=8; i++) {
        if(abs (prev_ch[i] - ch[i]) > 1 ) {
          Serial.print("loop: @3 ----This chanel has changed: Chanel: ");
          Serial.println("ch["+String(i)+"]:"+String(ch[i])+".");
          data_changed = true;
        }
        prev_ch[i] = ch[i];
      }
      // Serial.println("@3 Chanels:"+String(ch_1)+"("+String(data.ch1)+", c "+String(ch_1_center)+"), "+String(ch_2)+"("+String(data.ch2)+", c "+String(ch_2_center)+") | "+String(ch_3)+"("+String(data.ch3)+", c "+String(ch_3_center)+"), "+String(ch_4)+"("+String(data.ch4)+", c "+String(ch_4_center)+") || "+String(ch_5)+"("+String(data.ch5)+", c "+String(ch_5_center)+"), "+String(ch_6)+"("+String(data.ch6)+", c "+String(ch_6_center)+") | "+String(ch_7)+"("+String(data.ch7)+", c "+String(ch_7_center)+"), "+String(ch_8)+"("+String(data.ch8)+", c "+String(ch_8_center)+")");
      
      servo_eyeLeftUD_Angle       = 255 - ch[1];
      servo_eyeLeftLR_Angle       = ch[2];
      servo_eyeRightUD_Angle      = ch[1];
      servo_eyeRightLR_Angle      = ch[2];

      servo_eyelidLeftUpper_Angle = (127 - (eyeToLip_Scale * (ch[1] - 127))) + (ch5ToLip_Scale * (127 - ch[5])) + (ch6ToLip_Scale * (127 - ch[6]));
      servo_eyelidLeftLower_Angle = (127 - (eyeToLip_Scale * (ch[1] - 127))) - (ch5ToLip_Scale * (127 - ch[5])) + (ch6ToLip_Scale * (127 - ch[6]));
      servo_eyelidRightUpper_Angle= (eyeToLip_Scale * (ch[1] - 127))  + 127  - (ch5ToLip_Scale * (127 - ch[5])) - (ch6ToLip_Scale * (127 - ch[6]));
      servo_eyelidRightLower_Angle= (eyeToLip_Scale * (ch[1] - 127))  + 127  + (ch5ToLip_Scale * (127 - ch[5])) - (ch6ToLip_Scale * (127 - ch[6]));

      servo_eyebrowRight_Angle    = ch[4] + (127 - ch[3]);
      servo_eyebrowLeft_Angle     = ch[4] - (127 - ch[3]);
      
      servo_cheekRight_Angle      = ch[7] + (127 - ch[8]);
      servo_cheekLeft_Angle       = ch[7] - (127 - ch[8]);
      
      servo_upperLip_Angle        = 0;
      
      servo_forheadRight_Angle    = ch[6] + (127 - ch[5]);
      servo_forheadLeft_Angle     = ch[6] - (127 - ch[5]);
      servo_Jaw_UpDown_Angle      = 0;



      servo_eyeLeftUD_Angle        = constrain(servo_eyeLeftUD_Angle       , 0, 255);
      servo_eyeLeftLR_Angle        = constrain(servo_eyeLeftLR_Angle       , 0, 255);
      servo_eyeRightUD_Angle       = constrain(servo_eyeRightUD_Angle      , 0, 255);
      servo_eyeRightLR_Angle       = constrain(servo_eyeRightLR_Angle      , 0, 255);
      servo_eyelidLeftUpper_Angle  = constrain(servo_eyelidLeftUpper_Angle , 0, 255);
      servo_eyelidLeftLower_Angle  = constrain(servo_eyelidLeftLower_Angle , 0, 255);
      servo_eyelidRightUpper_Angle = constrain(servo_eyelidRightUpper_Angle, 0, 255);
      servo_eyelidRightLower_Angle = constrain(servo_eyelidRightLower_Angle, 0, 255);
      servo_eyebrowRight_Angle     = constrain(servo_eyebrowRight_Angle    , 0, 255);
      servo_eyebrowLeft_Angle      = constrain(servo_eyebrowLeft_Angle     , 0, 255);
      servo_cheekRight_Angle       = constrain(servo_cheekRight_Angle      , 0, 255);
      servo_cheekLeft_Angle        = constrain(servo_cheekLeft_Angle       , 0, 255);
      servo_upperLip_Angle         = constrain(servo_upperLip_Angle        , 0, 255);
      servo_forheadRight_Angle     = constrain(servo_forheadRight_Angle    , 0, 255);
      servo_forheadLeft_Angle      = constrain(servo_forheadLeft_Angle     , 0, 255);
      servo_Jaw_UpDown_Angle       = constrain(servo_Jaw_UpDown_Angle      , 0, 255);

      servo_eyeLeftUD_Pwm       = (servo_eyeLeftUD_Angle        < 128 ? map(servo_eyeLeftUD_Angle        , 0, 127, SERVO_MIN_eyeLeftUD,        SERVO_MID_eyeLeftUD )      : map(servo_eyeLeftUD_Angle,        128, 255, SERVO_MID_eyeLeftUD ,       SERVO_MAX_eyeLeftUD ));
      servo_eyeLeftLR_Pwm       = (servo_eyeLeftLR_Angle        < 128 ? map(servo_eyeLeftLR_Angle        , 0, 127, SERVO_MIN_eyeLeftLR,        SERVO_MID_eyeLeftLR )      : map(servo_eyeLeftLR_Angle,        128, 255, SERVO_MID_eyeLeftLR ,       SERVO_MAX_eyeLeftLR ));
      servo_eyeRightUD_Pwm      = (servo_eyeRightUD_Angle       < 128 ? map(servo_eyeRightUD_Angle       , 0, 127, SERVO_MIN_eyeRightUD,       SERVO_MID_eyeRightUD)      : map(servo_eyeRightUD_Angle,       128, 255, SERVO_MID_eyeRightUD,       SERVO_MAX_eyeRightUD));
      servo_eyeRightLR_Pwm      = (servo_eyeRightLR_Angle       < 128 ? map(servo_eyeRightLR_Angle       , 0, 127, SERVO_MIN_eyeRightLR,       SERVO_MID_eyeRightLR)      : map(servo_eyeRightLR_Angle,       128, 255, SERVO_MID_eyeRightLR,       SERVO_MAX_eyeRightLR)); 
      servo_eyelidLeftUpper_Pwm = (servo_eyelidLeftUpper_Angle  < 128 ?  map(servo_eyelidLeftUpper_Angle , 0, 127, SERVO_MIN_eyelidLeftUpper,  SERVO_MID_eyelidLeftUpper) : map(servo_eyelidLeftUpper_Angle , 128, 255, SERVO_MID_eyelidLeftUpper,  SERVO_MAX_eyelidLeftUpper));
      servo_eyelidLeftLower_Pwm = (servo_eyelidLeftLower_Angle  < 128 ?  map(servo_eyelidLeftLower_Angle , 0, 127, SERVO_MIN_eyelidLeftLower,  SERVO_MID_eyelidLeftLower) : map(servo_eyelidLeftLower_Angle , 128, 255, SERVO_MID_eyelidLeftLower,  SERVO_MAX_eyelidLeftLower));
      servo_eyelidRightUpper_Pwm= (servo_eyelidRightUpper_Angle < 128 ?  map(servo_eyelidRightUpper_Angle, 0, 127, SERVO_MIN_eyelidRightUpper, SERVO_MID_eyelidRightUpper): map(servo_eyelidRightUpper_Angle, 128, 255, SERVO_MID_eyelidRightUpper, SERVO_MAX_eyelidRightUpper));
      servo_eyelidRightLower_Pwm= (servo_eyelidRightLower_Angle < 128 ?  map(servo_eyelidRightLower_Angle, 0, 127, SERVO_MIN_eyelidRightLower, SERVO_MID_eyelidRightLower): map(servo_eyelidRightLower_Angle, 128, 255, SERVO_MID_eyelidRightLower, SERVO_MAX_eyelidRightLower));
      servo_eyebrowRight_Pwm    = (servo_eyebrowRight_Angle     < 128 ? map(servo_eyebrowRight_Angle     , 0, 127, SERVO_MIN_eyebrowRight,     SERVO_MID_eyebrowRight)    : map(servo_eyebrowRight_Angle    , 128, 255, SERVO_MID_eyebrowRight,     SERVO_MAX_eyebrowRight));
      servo_eyebrowLeft_Pwm     = (servo_eyebrowLeft_Angle      < 128 ? map(servo_eyebrowLeft_Angle      , 0, 127, SERVO_MIN_eyebrowLeft,      SERVO_MID_eyebrowLeft)     : map(servo_eyebrowLeft_Angle     , 128, 255, SERVO_MID_eyebrowLeft,      SERVO_MAX_eyebrowLeft));
      servo_cheekRight_Pwm      = (servo_cheekRight_Angle       < 128 ? map(servo_cheekRight_Angle       , 0, 127, SERVO_MIN_cheekRight,       SERVO_MID_cheekRight)      : map(servo_cheekRight_Angle      , 128, 255, SERVO_MID_cheekRight,       SERVO_MAX_cheekRight));
      servo_cheekLeft_Pwm       = (servo_cheekLeft_Angle        < 128 ? map(servo_cheekLeft_Angle        , 0, 127, SERVO_MIN_cheekLeft,        SERVO_MID_cheekLeft)       : map(servo_cheekLeft_Angle       , 128, 255, SERVO_MID_cheekLeft,        SERVO_MAX_cheekLeft));
      servo_upperLip_Pwm        = (servo_upperLip_Angle         < 128 ?  map(servo_upperLip_Angle        , 0, 127, SERVO_MIN_upperLip,         SERVO_MID_upperLip)        : map(servo_upperLip_Angle        , 128, 255, SERVO_MID_upperLip,         SERVO_MAX_upperLip));
      servo_forheadRight_Pwm    = (servo_forheadRight_Angle     < 128 ? map(servo_forheadRight_Angle     , 0, 127, SERVO_MIN_forheadRight,     SERVO_MID_forheadRight)    : map(servo_forheadRight_Angle    , 128, 255, SERVO_MID_forheadRight,     SERVO_MAX_forheadRight));
      servo_forheadLeft_Pwm     = (servo_forheadLeft_Angle      < 128 ? map(servo_forheadLeft_Angle      , 0, 127, SERVO_MIN_forheadLeft,      SERVO_MID_forheadLeft)     : map(servo_forheadLeft_Angle     , 128, 255, SERVO_MID_forheadLeft,      SERVO_MAX_forheadLeft));
      servo_Jaw_UpDown_Pwm      = (servo_Jaw_UpDown_Angle       < 128 ? map(servo_Jaw_UpDown_Angle       , 0, 127, SERVO_MIN_Jaw_UpDown,       SERVO_MID_Jaw_UpDown)      : map(servo_Jaw_UpDown_Angle      , 128, 255, SERVO_MID_Jaw_UpDown,       SERVO_MAX_Jaw_UpDown));

      pwm.setPWM( i01_head_eyeLeftUD       , 0, servo_eyeLeftUD_Pwm);
      pwm.setPWM( i01_head_eyeLeftLR       , 0, servo_eyeLeftLR_Pwm);
      pwm.setPWM( i01_head_eyeRightUD      , 0, servo_eyeRightUD_Pwm);
      pwm.setPWM( i01_head_eyeRightLR      , 0, servo_eyeRightLR_Pwm);
      
      pwm.setPWM( i01_head_eyelidLeftUpper , 0, servo_eyelidLeftUpper_Pwm);
      pwm.setPWM( i01_head_eyelidLeftLower , 0, servo_eyelidLeftLower_Pwm);
      pwm.setPWM( i01_head_eyelidRightUpper, 0, servo_eyelidRightUpper_Pwm);
      pwm.setPWM( i01_head_eyelidRightLower, 0, servo_eyelidRightLower_Pwm);
      /*
      pwm.setPWM( i01_head_eyebrowRight    , 0, servo_eyebrowRight_Pwm);
      pwm.setPWM( i01_head_eyebrowLeft     , 0, servo_eyebrowLeft_Pwm);

      pwm.setPWM( i01_head_cheekRight      , 0, servo_cheekRight_Pwm);
      pwm.setPWM( i01_head_cheekLeft       , 0, servo_cheekLeft_Pwm);

      pwm.setPWM( i01_head_upperLip        , 0, servo_upperLip_Pwm);

      pwm.setPWM( i01_head_forheadRight    , 0, servo_forheadRight_Pwm);
      pwm.setPWM( i01_head_forheadLeft     , 0, servo_forheadLeft_Pwm);

      pwm.setPWM( Jaw_UpDown               , 0, servo_Jaw_UpDown_Pwm);
      */

      if(data_changed == true) {
        ////Serial.println("loop: @4 Chanels:data.ch1 = "+String(data.ch1)+", ch_1 = "+String(ch_1)+", servo_eyeLeftUD_Angle = "+String(servo_eyeLeftUD_Angle)+", servo_eyeLeftUD_Pwm = "+String(servo_eyeLeftUD_Pwm)+", || data.ch2 = "+String(data.ch2)+", ch_2 = "+String(ch_2)+", servo_eyeLeftLR_Angle = "+String(servo_eyeLeftLR_Angle)+", servo_eyeLeftLR_Pwm = "+String(servo_eyeLeftLR_Pwm)+".");
        //Serial.println("loop @5 data changed.  Chanels:data.ch1 = "+String(data.ch1)+", ch_1 = "+String(ch_1)+", servo_eyeRightUD_Angle = "+String(servo_eyeRightUD_Angle)+", servo_eyeRightUD_Pwm = "+String(servo_eyeRightUD_Pwm)+", || data.ch2 = "+String(data.ch2)+", ch_2 = "+String(ch_2)+", servo_eyeRightLR_Angle = "+String(servo_eyeRightLR_Angle)+", servo_eyeRightLR_Pwm = "+String(servo_eyeRightLR_Pwm)+".");
        Serial.print("loop @5 changed. ");
        if ((prev_servo_eyeLeftUD_Pwm        != servo_eyeLeftUD_Pwm      ) || (prev_servo_eyeRightUD_Pwm       != servo_eyeRightUD_Pwm      )) 
          {
            Serial.print(" ch[1] ="+String(ch[1])+", ");
          }
        if ((prev_servo_eyeLeftLR_Pwm        != servo_eyeLeftLR_Pwm      ) || (prev_servo_eyeRightLR_Pwm       != servo_eyeRightLR_Pwm      )) 
          {
            Serial.print(" ch[2] ="+String(ch[2])+", ");
          }
        if ((prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) || (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) || (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) || (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm))
          {
            //Serial.print(" <ch[1], ch[5], ch[6]> = ");
            //Serial.print("<"+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+">, ");
          }

        //if (prev_servo_eyeLeftUD_Pwm        != servo_eyeLeftUD_Pwm       ) {Serial.print("S_eyeLeftUD_Pwm:"+String(servo_eyeLeftUD_Pwm       )+", ");}
        //if (prev_servo_eyeLeftLR_Pwm        != servo_eyeLeftLR_Pwm       ) {Serial.print("S_eyeLeftLR_Pwm:"+String(servo_eyeLeftLR_Pwm       )+", ");}
        //if (prev_servo_eyeRightUD_Pwm       != servo_eyeRightUD_Pwm      ) {Serial.print("S_eyeRightUD_Pwm:"+String(servo_eyeRightUD_Pwm      )+", ");}
        //if (prev_servo_eyeRightLR_Pwm       != servo_eyeRightLR_Pwm      ) {Serial.print("S_eyeRightLR_Pwm:"+String(servo_eyeRightLR_Pwm      )+", ");}
        if (prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) {Serial.print("S_eyelidLeftUpper_Pwm:"+String(servo_eyelidLeftUpper_Pwm )+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        if (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) {Serial.print("S_eyelidLeftLower_Pwm:"+String(servo_eyelidLeftLower_Pwm )+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        if (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) {Serial.print("S_eyelidRightUpper_Pwm:"+String(servo_eyelidRightUpper_Pwm)+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        if (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm) {Serial.print("S_eyelidRightLower_Pwm:"+String(servo_eyelidRightLower_Pwm)+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        //if (prev_servo_eyebrowRight_Pwm     != servo_eyebrowRight_Pwm    ) {Serial.print("S_eyebrowRight_Pwm:"+String(servo_eyebrowRight_Pwm    )+", ");}
        //if (prev_servo_eyebrowLeft_Pwm      != servo_eyebrowLeft_Pwm     ) {Serial.print("S_eyebrowLeft_Pwm:"+String(servo_eyebrowLeft_Pwm     )+", ");}
        //if (prev_servo_cheekRight_Pwm       != servo_cheekRight_Pwm      ) {Serial.print("S_cheekRight_Pwm:"+String(servo_cheekRight_Pwm      )+", ");}
        //if (prev_servo_cheekLeft_Pwm        != servo_cheekLeft_Pwm       ) {Serial.print("S_cheekLeft_Pwm:"+String(servo_cheekLeft_Pwm       )+", ");}
        //if (prev_servo_upperLip_Pwm         != servo_upperLip_Pwm        ) {Serial.print("S_upperLip_Pwm:"+String(servo_upperLip_Pwm        )+", ");}
        //if (prev_servo_forheadRight_Pwm     != servo_forheadRight_Pwm    ) {Serial.print("S_forheadRight_Pwm:"+String(servo_forheadRight_Pwm    )+", ");}
        //if (prev_servo_forheadLeft_Pwm      != servo_forheadLeft_Pwm     ) {Serial.print("S_forheadLeft_Pwm:"+String(servo_forheadLeft_Pwm     )+", ");}
        //if (prev_servo_Jaw_UpDown_Pwm       != servo_Jaw_UpDown_Pwm      ) {Serial.print("S_Jaw_UpDown_Pwm:"+String(servo_Jaw_UpDown_Pwm      )+", ");}
        Serial.println(".");

        if (prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) {Serial.print("servo_eyelidLeftUpper_Angle:"+String(servo_eyelidLeftUpper_Angle )+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        if (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) {Serial.print("servo_eyelidLeftLower_Angle:"+String(servo_eyelidLeftLower_Angle )+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        if (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) {Serial.print("servo_eyelidRightUpper_Angle:"+String(servo_eyelidRightUpper_Angle)+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        if (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm) {Serial.print("servo_eyelidRightLower_Angle:"+String(servo_eyelidRightLower_Angle)+", ");} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
        Serial.println(".");

      }

      prev_servo_eyeLeftUD_Pwm        = servo_eyeLeftUD_Pwm       ;
      prev_servo_eyeLeftLR_Pwm        = servo_eyeLeftLR_Pwm       ;
      prev_servo_eyeRightUD_Pwm       = servo_eyeRightUD_Pwm      ;
      prev_servo_eyeRightLR_Pwm       = servo_eyeRightLR_Pwm      ;
      prev_servo_eyelidLeftUpper_Pwm  = servo_eyelidLeftUpper_Pwm ;
      prev_servo_eyelidLeftLower_Pwm  = servo_eyelidLeftLower_Pwm ;
      prev_servo_eyelidRightUpper_Pwm = servo_eyelidRightUpper_Pwm;
      prev_servo_eyelidRightLower_Pwm = servo_eyelidRightLower_Pwm;
      prev_servo_eyebrowRight_Pwm     = servo_eyebrowRight_Pwm    ;
      prev_servo_eyebrowLeft_Pwm      = servo_eyebrowLeft_Pwm     ;
      prev_servo_cheekRight_Pwm       = servo_cheekRight_Pwm      ;
      prev_servo_cheekLeft_Pwm        = servo_cheekLeft_Pwm       ;
      prev_servo_upperLip_Pwm         = servo_upperLip_Pwm        ;
      prev_servo_forheadRight_Pwm     = servo_forheadRight_Pwm    ;
      prev_servo_forheadLeft_Pwm      = servo_forheadLeft_Pwm     ;
      prev_servo_Jaw_UpDown_Pwm       = servo_Jaw_UpDown_Pwm      ;

    }
  }
}

void loop_ReadFromSerialLine(unsigned long currentMillis) {
  if (currentMillis - previousMillis_SerialLine >= interval_SerialLine) {  // start timed event for read and send
    previousMillis_SerialLine = currentMillis;

    if(serialLine.receiveData()){ //ET2.receiveData())            // main data receive
      previousSafetyMillis = currentMillis; 
      //mydata_send.mode = mode;
      //mydata_send.count = count;
      //ToDo here
      Serial.println("loop_ReadFromSerialLine:mydata_received = 0:" + String(mydata_received.s00) +", 1:" + String(mydata_received.s01) +", 2:" + String(mydata_received.s02) +", 3:" + String(mydata_received.s03));
      count = count + 1;                                              // update count for remote monitoring
    } else if(currentMillis - previousSafetyMillis > 200) {         // safeties
      noDataCount = noDataCount + 1;                                  // update count for remote monitoring
      Serial.println("!"+String(noDataCount)+"! No Data ");
    }
  }  // end of timed event Receive/Send

  if (currentMillis - previousServoMillis >= servoInterval) {  // start timed event for Servos  (200 ms)
	previousServoMillis = currentMillis;
	//ToDo here
  }
	  
}