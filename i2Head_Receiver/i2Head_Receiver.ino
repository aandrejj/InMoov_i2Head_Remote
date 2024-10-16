/* i2Head_Receiver  RF NANO Prímacovy kod 

*/
 

#define USE_RF_REMOTE


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include "Servo_Min_Max.h"
#include "i2Head_Receiver.h"

#include "TxRx_dataStructures.h"

#define RANDOM_EYES_MOVEMENT

#ifdef RANDOM_EYES_MOVEMENT
  #include "eyes_random_moves.h"
  //#include "ServoSender.h"

  #define lookUpDown    1
  #define lookLeftRight 2
  #define lidLowerLeft  3
  #define lidUpperLeft  4
  #define lidLowerRight 5
  #define lidUpperRight 6

#endif

#define HIGHSPEED 

#ifdef HIGHSPEED
  #define Baud 19200   // Serial monitor
#else
  #define Baud 9600    // Serial monitor
#endif

int16_t mode;
int count;
int noDataCount = 0;


const uint64_t pipeIn = 0x0022;   //Tento isty kod musi mať aj primač
RF24 radio(10,9);  //zapojenie CE a CSN pinov
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long previousMillis = 0;
const long interval = 20;

long previousSafetyMillis;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

RX_DATA_STRUCTURE mydata_received;
RX_DATA_STRUCTURE prev_mydata;
//TX_DATA_STRUCTURE mydata_remote;

#ifdef RANDOM_EYES_MOVEMENT
  int UpDownState;
  int LeftRightState;
  int lidMod;
  int uplidpulse;
  int lolidpulse;
  int altuplidpulse;
  int altlolidpulse;

  long REM_interval;
  long REM_pose;
  /*
  ServoSender lookUpDown     = ServoSender(i01_head_eyeLeftUD       , i01_head_eyeRightUD, SERVO_MIN_eyeLeftUD       , SERVO_MID_eyeLeftUD       , SERVO_MAX_eyeLeftUD       , SERVO_MIN_eyeRightUD, SERVO_MID_eyeRightUD, SERVO_MAX_eyeRightUD);
  ServoSender lookLeftRight  = ServoSender(i01_head_eyeLeftLR       , i01_head_eyeRightLR, SERVO_MIN_eyeLeftLR       , SERVO_MID_eyeLeftLR       , SERVO_MAX_eyeLeftLR       , SERVO_MIN_eyeRightLR, SERVO_MID_eyeRightLR, SERVO_MAX_eyeRightLR);
  ServoSender lidLowerLeft   = ServoSender(i01_head_eyelidLeftLower ,                  99, SERVO_MIN_eyelidLeftLower , SERVO_MID_eyelidLeftLower , SERVO_MAX_eyelidLeftLower ,                    0,                    0,                    0);
  ServoSender lidUpperLeft   = ServoSender(i01_head_eyelidLeftUpper ,                  99, SERVO_MIN_eyelidLeftUpper , SERVO_MID_eyelidLeftUpper , SERVO_MAX_eyelidLeftUpper ,                    0,                    0,                    0);
  ServoSender lidLowerRight  = ServoSender(i01_head_eyelidRightLower,                  99, SERVO_MIN_eyelidRightLower, SERVO_MID_eyelidRightLower, SERVO_MAX_eyelidRightLower,                    0,                    0,                    0);
  ServoSender lidUpperRight  = ServoSender(i01_head_eyelidRightUpper,                  99, SERVO_MIN_eyelidRightUpper, SERVO_MID_eyelidRightUpper, SERVO_MAX_eyelidRightUpper,                    0,                    0,                    0);
  */
#endif

void resetData()
{

  mydata_received.s00 = 127;
  mydata_received.s01 = 127;
  mydata_received.s02 = 127;
  mydata_received.s03 = 127;
  mydata_received.s04 = 127;
  mydata_received.s05 = 127;
  mydata_received.s06 = 127;
  mydata_received.s07 = 127;
  mydata_received.s08 = 127;
  mydata_received.s09 = 127;
  mydata_received.s10 = 127;
  mydata_received.s11 = 127;
  mydata_received.s12 = 127;
  mydata_received.s13 = 127;
  mydata_received.s14 = 127;
  mydata_received.s15 = 127;
}

void setup()
{
  Serial.begin(Baud);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
	
  Serial.println("setup:: @1 Servo Initialization started");
  pwm.begin(); //pwm.begin(0);   0 = driver_ID
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	Serial.println("setup: @2 Servos on PCA9685  attached");
  //all_center_points_initialized = false;
  delay(200);

  #ifdef RANDOM_EYES_MOVEMENT
    randomSeed(analogRead(A7));
    //lookUpDown.begin(pwm);
    //lookLeftRight.begin(pwm);
    //lidLowerLeft.begin(pwm);
    //lidUpperLeft.begin(pwm);
    //lidLowerRight.begin(pwm);
    //lidUpperRight.begin(pwm);
  #endif

  //konfiguracia NRF24 
  resetData();
  //all_center_points_initialized = true;
  #ifdef USE_RF_REMOTE
  	Serial.println("setup: @3 radio.begin()..");
    radio.begin();
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);  
    radio.openReadingPipe(1,pipeIn);
    
    radio.startListening();
    Serial.println("setup: @4 rf-radio started");
  #endif

  delay(500);

  Serial.println("setup: @8 done. setup END.");
}

unsigned long lastRecvTime = 0;
bool recvData()
{
  bool dataReceived = false;
  while ( radio.available() ) {
    //radio.read(&data, sizeof(MyData));
    radio.read(&mydata_received, sizeof(RX_DATA_STRUCTURE));
    lastRecvTime = millis(); //tu dostávame údaje
    dataReceived = true;
  }
  return dataReceived;
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
bool RF_data_changed = false;
bool serial_data_changed = false;
bool remoteDataReceived = false;
bool serialDataReceived = false;
void loop()
{
  remoteDataReceived = false;
  unsigned long currentMillis = millis();
  
  #ifdef USE_RF_REMOTE
  // -------------------  data from RF--------------------------------------------------
  if (currentMillis - previousMillis >= interval) 
  {  // start timed event for read and send
    previousMillis = currentMillis;
    //Serial.print("@1.1 started");
    //----------------------------------------recvData-------------------------------------
    remoteDataReceived = recvData();
    if(remoteDataReceived == true) {
      previousSafetyMillis = currentMillis; 
      //Serial.println("@1.2 remoteDataReceived = "+String(remoteDataReceived));
      if (mydata_received.mode == 0) // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 3 = ?
      {
        constrain_RfData_0_255();
        RF_data_changed = RfData_changed();
        compute_fromRfData_toAngleData();
      }
      else if (mydata_received.mode == 1)
      {
          serial_data_changed = serialData_changed();
          if(serial_data_changed == true) {
            compute_from_SerialData_toAngleData();
            //Serial.println("@1.40 started.");
            constrain_allServoAngles_0_255();
            reset_SerialDataChanged();
          }
      }
    } else  if(currentMillis - previousSafetyMillis > 1000) {         // safeties
      //noDataCount = noDataCount+1;                                              // update count for remote monitoring
      //Serial.println("!"+String(noDataCount)+"! No Data ");
      //ToDo Add RandomEyesMovement here
      #ifdef RANDOM_EYES_MOVEMENT
        randomEyesMovement(currentMillis);
      #endif
    }
    //Serial.println(" @1.2 end");
  }
  #endif
  // -----------  end of RF-data---------------------------------------------------------------

  //-------------------Servos  handling-----------------------------------------------------------------
  //if (currentMillis - previousServoMillis >= servoInterval) 
  //{  // start timed event for Servos  (200 ms)
	  //previousServoMillis = currentMillis;
    //Serial.println("@3.1 started serialDataReceived = "+String(serialDataReceived)+", remoteDataReceived = "+String(remoteDataReceived));
    if((serialDataReceived==true) || (remoteDataReceived == true)) {
      //Serial.println("@3.2 DataReceived changed");
      constrain_allServoAngles_0_255();

      convert_allAngle_to_Pwm_Min_Center_Max();

      sendData_toPwmDriver();
      //Serial.println("@3.3 sendData_toPwmDriver");

      if(RF_data_changed == true || serial_data_changed == true) {
        //show_ChangedData_toDebug();

        show_PwmData_toDebug();

        //show_eyeLidsData_PwmAndAngle_toDebug();
        
      }

      copyActualPwmData_toPreviousPwmData();
      
    }
    //Serial.println(" @3.2 end.");
  //}
  //------------- end of Servo handling---------------------------------
  //Serial.println("loop: end of loop");
}// end of  loop()
//------------------------------end of  loop()----------------------------------
//------------------------------end of  loop()----------------------------------
//------------------------------end of  loop()----------------------------------

#ifdef RANDOM_EYES_MOVEMENT
void randomEyesMovement(unsigned long currentMillis){
  Serial.print("REM: Start");
  REM_interval = random(20,2000);
  Serial.print(", REM_interval = "+String(REM_interval));
  REM_pose = random (0,3);
  //if (REM_pose>2) {REM_pose =2;}
  Serial.println(", REM_pose = "+String(REM_pose));
  //delay(100);
  
  switch (REM_pose){
    case 0:
      Serial.println("REM:0.start'blink'");
      blink(80);
      Serial.println("REM:0.'blink' end");
      
      //Serial.println("REM:0.  starting lookAtDirection(true,..)");
      lookAtRandomDirection(true, 50, 130, "REM:0, ");
      //Serial.println("REM:0.  back from lookAtDirection()");
      
    break;
    case 1:
      Serial.println("REM:1. starting lookAtDirection(true,..)");
      
      lookAtRandomDirection(true, 30, 130, "REM:1, ");
      //Serial.println("REM:1.  back from lookAtDirection()");
      
    break;
    case 2:
      Serial.println("REM:2.  starting 'blink'....");
      blink(60);
      Serial.println("REM:2.  back from 'blink'");
      
      //Serial.println("REM:2.  starting lookAtDirection(false,...)");
      lookAtRandomDirection(false, 30, 130, "REM:2, ");
      //Serial.println("REM:2.  back from lookAtDirection()");      
      
    break;
  }
  
  delay(REM_interval);
}
void lookAtRandomDirection(bool generateRandomDirection, long minUpDown, long maxUpDown, String textToShow)
{
    if(generateRandomDirection == true) {
        UpDownState = random(minUpDown, maxUpDown);
        LeftRightState = random(30, 220);
        lidMod = ( 60 - UpDownState)/2;
         lookUpDown_write(UpDownState);
      lookLeftRight_write(LeftRightState);
    }

     lidUpperLeft_write(120+lidMod);// 70+lidMod);
    lidUpperRight_write(90+lidMod);//110-lidMod);

     lidLowerLeft_write(80+lidMod);//160+lidMod);
    lidLowerRight_write(80+lidMod);// 30-lidMod);


}

void blink (int time) 
{
      lidUpperLeft_write(255);//ClosedEyes_eyelidLeftUpper_Angle);//400  //pwm.setPWM(2, 0, 400);
      lidLowerLeft_write(255);//ClosedEyes_eyelidLeftLower_Angle);//240 //pwm.setPWM(3, 0, 240);
      lidUpperRight_write(255);//ClosedEyes_eyelidRightUpper_Angle);//pwm.setPWM(4, 0, 240);
      lidUpperRight_write(255);//ClosedEyes_eyelidRightLower_Angle);//pwm.setPWM(5, 0, 400);
      delay(time);
      lookAtRandomDirection(false, 0, 0,"blink");
      //lidUpperLeft_write( 70+lidMod);//pwm.setPWM(2, 0, uplidpulse);
      //lidLowerLeft_write(160+lidMod);//pwm.setPWM(3, 0, lolidpulse);
      //lidLowerRight_write( 30-lidMod);//pwm.setPWM(4, 0, altuplidpulse);
      //lidUpperRight_write(110-lidMod);//pwm.setPWM(5, 0, altlolidpulse);
    //----------------------------------
}
#endif

bool RfData_changed(){
  bool RF_data_changed = false;
  for (short i=0; i<=8; i++) {
    if(abs (prev_ch[i] - ch[i]) > 1 ) {
      Serial.print("loop: @3 ----DataChanged.@RF This chanel has changed: i="+String(i)+" Chanel: ");
      Serial.println("ch["+String(i)+"]:"+String(ch[i])+".");
      RF_data_changed = true;
    }
    prev_ch[i] = ch[i];
  }
  return RF_data_changed;
}
void compute_from_SerialData_toAngleData()
{
  servo_eyeLeftUD_Angle        = constrain(mydata_received.s00, 0, 255);
  servo_eyeLeftLR_Angle        = constrain(mydata_received.s01, 0, 255);
  servo_eyeRightUD_Angle       = constrain(mydata_received.s02, 0, 255);
  servo_eyeRightLR_Angle       = constrain(mydata_received.s03, 0, 255);
  servo_eyelidLeftUpper_Angle  = constrain(mydata_received.s04, 0, 255);
  servo_eyelidLeftLower_Angle  = constrain(mydata_received.s05, 0, 255);
  servo_eyelidRightUpper_Angle = constrain(mydata_received.s06, 0, 255);
  servo_eyelidRightLower_Angle = constrain(mydata_received.s07, 0, 255);
  servo_eyebrowRight_Angle     = constrain(mydata_received.s08, 0, 255);
  servo_eyebrowLeft_Angle      = constrain(mydata_received.s09, 0, 255);
  servo_cheekRight_Angle       = constrain(mydata_received.s10, 0, 255);
  servo_cheekLeft_Angle        = constrain(mydata_received.s11, 0, 255);
  servo_upperLip_Angle         = constrain(mydata_received.s12, 0, 255);
  servo_forheadRight_Angle     = constrain(mydata_received.s13, 0, 255);
  servo_forheadLeft_Angle      = constrain(mydata_received.s14, 0, 255);
  servo_Jaw_UpDown_Angle       = constrain(mydata_received.s15, 0, 255);

}

void compute_fromRfData_toAngleData()
{
  servo_eyeLeftUD_Angle       = ch[1];
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
}

void constrain_RfData_0_255() {
  if (mydata_received.mode == 0) // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 3 = ?
  {
    ch[1] = constrain(mydata_received.s00, 0, 255);
    ch[2] = constrain(mydata_received.s01, 0, 255);
    ch[3] = constrain(mydata_received.s02, 0, 255);
    ch[4] = constrain(mydata_received.s03, 0, 255);
    ch[5] = constrain(mydata_received.s04, 0, 255);
    ch[6] = constrain(mydata_received.s05, 0, 255);
    ch[7] = constrain(mydata_received.s06, 0, 255);
    ch[8] = constrain(mydata_received.s07, 0, 255);
  }
}

void constrain_allServoAngles_0_255() {
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
}

void convert_allAngle_to_Pwm_Min_Center_Max(){
      servo_eyeLeftUD_Pwm       = (servo_eyeLeftUD_Angle        < 128 ? map(servo_eyeLeftUD_Angle        , 0, 127, SERVO_MAX_eyeLeftUD,        SERVO_MID_eyeLeftUD )      : map(servo_eyeLeftUD_Angle,        128, 255, SERVO_MID_eyeLeftUD ,       SERVO_MIN_eyeLeftUD )); //invertovane
      servo_eyeLeftLR_Pwm       = (servo_eyeLeftLR_Angle        < 128 ? map(servo_eyeLeftLR_Angle        , 0, 127, SERVO_MIN_eyeLeftLR,        SERVO_MID_eyeLeftLR )      : map(servo_eyeLeftLR_Angle,        128, 255, SERVO_MID_eyeLeftLR ,       SERVO_MAX_eyeLeftLR )); 
      servo_eyeRightUD_Pwm      = (servo_eyeRightUD_Angle       < 128 ? map(servo_eyeRightUD_Angle       , 0, 127, SERVO_MIN_eyeRightUD,       SERVO_MID_eyeRightUD)      : map(servo_eyeRightUD_Angle,       128, 255, SERVO_MID_eyeRightUD,       SERVO_MAX_eyeRightUD)); 
      servo_eyeRightLR_Pwm      = (servo_eyeRightLR_Angle       < 128 ? map(servo_eyeRightLR_Angle       , 0, 127, SERVO_MIN_eyeRightLR,       SERVO_MID_eyeRightLR)      : map(servo_eyeRightLR_Angle,       128, 255, SERVO_MID_eyeRightLR,       SERVO_MAX_eyeRightLR)); 
      servo_eyelidLeftUpper_Pwm = (servo_eyelidLeftUpper_Angle  < 128 ?  map(servo_eyelidLeftUpper_Angle , 0, 127, SERVO_MIN_eyelidLeftUpper,  SERVO_MID_eyelidLeftUpper) : map(servo_eyelidLeftUpper_Angle , 128, 255, SERVO_MID_eyelidLeftUpper,  SERVO_MAX_eyelidLeftUpper));
      servo_eyelidLeftLower_Pwm = (servo_eyelidLeftLower_Angle  < 128 ?  map(servo_eyelidLeftLower_Angle , 0, 127, SERVO_MAX_eyelidLeftLower,  SERVO_MID_eyelidLeftLower) : map(servo_eyelidLeftLower_Angle , 128, 255, SERVO_MID_eyelidLeftLower,  SERVO_MIN_eyelidLeftLower));  //invert
      servo_eyelidRightUpper_Pwm= (servo_eyelidRightUpper_Angle < 128 ?  map(servo_eyelidRightUpper_Angle, 0, 127, SERVO_MAX_eyelidRightUpper, SERVO_MID_eyelidRightUpper): map(servo_eyelidRightUpper_Angle, 128, 255, SERVO_MID_eyelidRightUpper, SERVO_MIN_eyelidRightUpper)); //invertovane
      servo_eyelidRightLower_Pwm= (servo_eyelidRightLower_Angle < 128 ?  map(servo_eyelidRightLower_Angle, 0, 127, SERVO_MIN_eyelidRightLower, SERVO_MID_eyelidRightLower): map(servo_eyelidRightLower_Angle, 128, 255, SERVO_MID_eyelidRightLower, SERVO_MAX_eyelidRightLower));
      servo_eyebrowRight_Pwm    = (servo_eyebrowRight_Angle     < 128 ? map(servo_eyebrowRight_Angle     , 0, 127, SERVO_MIN_eyebrowRight,     SERVO_MID_eyebrowRight)    : map(servo_eyebrowRight_Angle    , 128, 255, SERVO_MID_eyebrowRight,     SERVO_MAX_eyebrowRight));
      servo_eyebrowLeft_Pwm     = (servo_eyebrowLeft_Angle      < 128 ? map(servo_eyebrowLeft_Angle      , 0, 127, SERVO_MIN_eyebrowLeft,      SERVO_MID_eyebrowLeft)     : map(servo_eyebrowLeft_Angle     , 128, 255, SERVO_MID_eyebrowLeft,      SERVO_MAX_eyebrowLeft));
      servo_cheekRight_Pwm      = (servo_cheekRight_Angle       < 128 ? map(servo_cheekRight_Angle       , 0, 127, SERVO_MIN_cheekRight,       SERVO_MID_cheekRight)      : map(servo_cheekRight_Angle      , 128, 255, SERVO_MID_cheekRight,       SERVO_MAX_cheekRight));
      servo_cheekLeft_Pwm       = (servo_cheekLeft_Angle        < 128 ? map(servo_cheekLeft_Angle        , 0, 127, SERVO_MIN_cheekLeft,        SERVO_MID_cheekLeft)       : map(servo_cheekLeft_Angle       , 128, 255, SERVO_MID_cheekLeft,        SERVO_MAX_cheekLeft));
      servo_upperLip_Pwm        = (servo_upperLip_Angle         < 128 ?  map(servo_upperLip_Angle        , 0, 127, SERVO_MIN_upperLip,         SERVO_MID_upperLip)        : map(servo_upperLip_Angle        , 128, 255, SERVO_MID_upperLip,         SERVO_MAX_upperLip));
      servo_forheadRight_Pwm    = (servo_forheadRight_Angle     < 128 ? map(servo_forheadRight_Angle     , 0, 127, SERVO_MIN_forheadRight,     SERVO_MID_forheadRight)    : map(servo_forheadRight_Angle    , 128, 255, SERVO_MID_forheadRight,     SERVO_MAX_forheadRight));
      servo_forheadLeft_Pwm     = (servo_forheadLeft_Angle      < 128 ? map(servo_forheadLeft_Angle      , 0, 127, SERVO_MIN_forheadLeft,      SERVO_MID_forheadLeft)     : map(servo_forheadLeft_Angle     , 128, 255, SERVO_MID_forheadLeft,      SERVO_MAX_forheadLeft));
      servo_Jaw_UpDown_Pwm      = (servo_Jaw_UpDown_Angle       < 128 ? map(servo_Jaw_UpDown_Angle       , 0, 127, SERVO_MIN_Jaw_UpDown,       SERVO_MID_Jaw_UpDown)      : map(servo_Jaw_UpDown_Angle      , 128, 255, SERVO_MID_Jaw_UpDown,       SERVO_MAX_Jaw_UpDown));
}

void sendData_toPwmDriver() {
  pwm.setPWM( i01_head_eyeLeftUD       , 0, servo_eyeLeftUD_Pwm);
  pwm.setPWM( i01_head_eyeLeftLR       , 0, servo_eyeLeftLR_Pwm);
  pwm.setPWM( i01_head_eyeRightUD      , 0, servo_eyeRightUD_Pwm);
  pwm.setPWM( i01_head_eyeRightLR      , 0, servo_eyeRightLR_Pwm);
  
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
  
}

void show_PwmData_toDebug() {
  bool dataShown = false;
  if (prev_servo_eyeLeftUD_Pwm        != servo_eyeLeftUD_Pwm       ) {Serial.print("S_eyeLeftUD_Pwm:"+String(servo_eyeLeftUD_Pwm       )+", "); dataShown = true;}
  if (prev_servo_eyeLeftLR_Pwm        != servo_eyeLeftLR_Pwm       ) {Serial.print("S_eyeLeftLR_Pwm:"+String(servo_eyeLeftLR_Pwm       )+", "); dataShown = true;}
  if (prev_servo_eyeRightUD_Pwm       != servo_eyeRightUD_Pwm      ) {Serial.print("S_eyeRightUD_Pwm:"+String(servo_eyeRightUD_Pwm      )+", "); dataShown = true;}
  if (prev_servo_eyeRightLR_Pwm       != servo_eyeRightLR_Pwm      ) {Serial.print("S_eyeRightLR_Pwm:"+String(servo_eyeRightLR_Pwm      )+", "); dataShown = true;}
  if (prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) {Serial.print("S_eyelidLeftUpper_Pwm:"+String(servo_eyelidLeftUpper_Pwm )+", "); dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) {Serial.print("S_eyelidLeftLower_Pwm:"+String(servo_eyelidLeftLower_Pwm )+", "); dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) {Serial.print("S_eyelidRightUpper_Pwm:"+String(servo_eyelidRightUpper_Pwm)+", "); dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm) {Serial.print("S_eyelidRightLower_Pwm:"+String(servo_eyelidRightLower_Pwm)+", "); dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyebrowRight_Pwm     != servo_eyebrowRight_Pwm    ) {Serial.print("S_eyebrowRight_Pwm:"+String(servo_eyebrowRight_Pwm    )+", "); dataShown = true;}
  if (prev_servo_eyebrowLeft_Pwm      != servo_eyebrowLeft_Pwm     ) {Serial.print("S_eyebrowLeft_Pwm:"+String(servo_eyebrowLeft_Pwm     )+", "); dataShown = true;}
  if (prev_servo_cheekRight_Pwm       != servo_cheekRight_Pwm      ) {Serial.print("S_cheekRight_Pwm:"+String(servo_cheekRight_Pwm      )+", "); dataShown = true;}
  if (prev_servo_cheekLeft_Pwm        != servo_cheekLeft_Pwm       ) {Serial.print("S_cheekLeft_Pwm:"+String(servo_cheekLeft_Pwm       )+", "); dataShown = true;}
  if (prev_servo_upperLip_Pwm         != servo_upperLip_Pwm        ) {Serial.print("S_upperLip_Pwm:"+String(servo_upperLip_Pwm        )+", "); dataShown = true;}
  if (prev_servo_forheadRight_Pwm     != servo_forheadRight_Pwm    ) {Serial.print("S_forheadRight_Pwm:"+String(servo_forheadRight_Pwm    )+", "); dataShown = true;}
  if (prev_servo_forheadLeft_Pwm      != servo_forheadLeft_Pwm     ) {Serial.print("S_forheadLeft_Pwm:"+String(servo_forheadLeft_Pwm     )+", "); dataShown = true;}
  if (prev_servo_Jaw_UpDown_Pwm       != servo_Jaw_UpDown_Pwm      ) {Serial.print("S_Jaw_UpDown_Pwm:"+String(servo_Jaw_UpDown_Pwm      )+", "); dataShown = true;}
  if( dataShown == true){Serial.println(".");}
}

void show_eyeLidsData_PwmAndAngle_toDebug() {
  bool dataShown = false;
  if (prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) {Serial.print("servo_eyelidLeftUpper_Angle:"+String(servo_eyelidLeftUpper_Angle )+", ") ; dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) {Serial.print("servo_eyelidLeftLower_Angle:"+String(servo_eyelidLeftLower_Angle )+", ") ; dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) {Serial.print("servo_eyelidRightUpper_Angle:"+String(servo_eyelidRightUpper_Angle)+", "); dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm) {Serial.print("servo_eyelidRightLower_Angle:"+String(servo_eyelidRightLower_Angle)+", "); dataShown = true;} // +": (ch[1], ch[5], ch[6]) =("+String(ch[1])+", "+String(ch[5])+", "+String(ch[6])+"), ");}
  if( dataShown == true){Serial.println(",");}
}

void show_ChangedData_toDebug()
{
  ////Serial.println("loop: @6 Chanels:data.ch1 = "+String(data.ch1)+", ch_1 = "+String(ch_1)+", servo_eyeLeftUD_Angle = "+String(servo_eyeLeftUD_Angle)+", servo_eyeLeftUD_Pwm = "+String(servo_eyeLeftUD_Pwm)+", || data.ch2 = "+String(data.ch2)+", ch_2 = "+String(ch_2)+", servo_eyeLeftLR_Angle = "+String(servo_eyeLeftLR_Angle)+", servo_eyeLeftLR_Pwm = "+String(servo_eyeLeftLR_Pwm)+".");
  //Serial.println("loop @7 data changed.  Chanels:data.ch1 = "+String(data.ch1)+", ch_1 = "+String(ch_1)+", servo_eyeRightUD_Angle = "+String(servo_eyeRightUD_Angle)+", servo_eyeRightUD_Pwm = "+String(servo_eyeRightUD_Pwm)+", || data.ch2 = "+String(data.ch2)+", ch_2 = "+String(ch_2)+", servo_eyeRightLR_Angle = "+String(servo_eyeRightLR_Angle)+", servo_eyeRightLR_Pwm = "+String(servo_eyeRightLR_Pwm)+".");
  Serial.print("loop @8 changed. ");
  #ifdef USE_RF_REMOTE
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
  #endif

}

void copyActualPwmData_toPreviousPwmData() {
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

bool serialData_changed() {
  bool data_changed = false;
  if(mydata_received.s00 != prev_mydata.s00){data_changed = true; s00_changed = true; Serial.print("s00 = "+ String(mydata_received.s00)+" ");}
  if(mydata_received.s01 != prev_mydata.s01){data_changed = true; s01_changed = true; Serial.print("s01 = "+ String(mydata_received.s01)+" ");}
  if(mydata_received.s02 != prev_mydata.s02){data_changed = true; s02_changed = true; Serial.print("s02 = "+ String(mydata_received.s02)+" ");}
  if(mydata_received.s03 != prev_mydata.s03){data_changed = true; s03_changed = true; Serial.print("s03 = "+ String(mydata_received.s03)+" ");}
  if(mydata_received.s04 != prev_mydata.s04){data_changed = true; s04_changed = true; Serial.print("s04 = "+ String(mydata_received.s04)+" ");}
  if(mydata_received.s05 != prev_mydata.s05){data_changed = true; s05_changed = true; Serial.print("s05 = "+ String(mydata_received.s05)+" ");}
  if(mydata_received.s06 != prev_mydata.s06){data_changed = true; s06_changed = true; Serial.print("s06 = "+ String(mydata_received.s06)+" ");}
  if(mydata_received.s07 != prev_mydata.s07){data_changed = true; s07_changed = true; Serial.print("s07 = "+ String(mydata_received.s07)+" ");}
  if(mydata_received.s08 != prev_mydata.s08){data_changed = true; s08_changed = true; Serial.print("s08 = "+ String(mydata_received.s08)+" ");}
  if(mydata_received.s09 != prev_mydata.s09){data_changed = true; s09_changed = true; Serial.print("s09 = "+ String(mydata_received.s09)+" ");}
  if(mydata_received.s10 != prev_mydata.s10){data_changed = true; s10_changed = true; Serial.print("s10 = "+ String(mydata_received.s10)+" ");}
  if(mydata_received.s11 != prev_mydata.s11){data_changed = true; s11_changed = true; Serial.print("s11 = "+ String(mydata_received.s11)+" ");}
  if(mydata_received.s12 != prev_mydata.s12){data_changed = true; s12_changed = true; Serial.print("s12 = "+ String(mydata_received.s12)+" ");}
  if(mydata_received.s13 != prev_mydata.s13){data_changed = true; s13_changed = true; Serial.print("s13 = "+ String(mydata_received.s13)+" ");}
  if(mydata_received.s14 != prev_mydata.s14){data_changed = true; s14_changed = true; Serial.print("s14 = "+ String(mydata_received.s14)+" ");}
  if(mydata_received.s15 != prev_mydata.s15){data_changed = true; s15_changed = true; Serial.print("s15 = "+ String(mydata_received.s15)+" ");}
  if(data_changed == true){
      //Serial.println(" mydata_received_changed ----");
    }
  return data_changed;
}

void reset_SerialDataChanged()
{
  prev_mydata.s00 = mydata_received.s00; 
  prev_mydata.s01 = mydata_received.s01; 
  prev_mydata.s02 = mydata_received.s02; 
  prev_mydata.s03 = mydata_received.s03; 
  prev_mydata.s04 = mydata_received.s04; 
  prev_mydata.s05 = mydata_received.s05; 
  prev_mydata.s06 = mydata_received.s06; 
  prev_mydata.s07 = mydata_received.s07; 
  prev_mydata.s08 = mydata_received.s08; 
  prev_mydata.s09 = mydata_received.s09; 
  prev_mydata.s10 = mydata_received.s10; 
  prev_mydata.s11 = mydata_received.s11; 
  prev_mydata.s12 = mydata_received.s12; 
  prev_mydata.s13 = mydata_received.s13; 
  prev_mydata.s14 = mydata_received.s14; 
  prev_mydata.s15 = mydata_received.s15;

  s00_changed = false;
  s01_changed = false;
  s02_changed = false;
  s03_changed = false;
  s04_changed = false;
  s05_changed = false;
  s06_changed = false;
  s07_changed = false;
  s08_changed = false;
  s09_changed = false;
  s10_changed = false;
  s11_changed = false;
  s12_changed = false;
  s13_changed = false;
  s14_changed = false;
  s15_changed = false;
}

//----------------------------Random eyes movement-------------------------------------------------
//----------------------------Random eyes movement-------------------------------------------------
//----------------------------Random eyes movement-------------------------------------------------
//----------------------------Random eyes movement-------------------------------------------------
//----------------------------Random eyes movement-------------------------------------------------
//----------------------------Random eyes movement-------------------------------------------------
#ifdef RANDOM_EYES_MOVEMENT
bool lookUpDown_write(byte servo_angle)   {return servoSender_write(servo_angle, lookUpDown   );}
bool lookLeftRight_write(byte servo_angle){return servoSender_write(servo_angle, lookLeftRight);}
bool lidLowerLeft_write(byte servo_angle) {return servoSender_write(servo_angle, lidLowerLeft );}
bool lidUpperLeft_write( byte servo_angle){return servoSender_write(servo_angle, lidUpperLeft );}
bool lidLowerRight_write(byte servo_angle){return servoSender_write(servo_angle, lidLowerRight);}
bool lidUpperRight_write(byte servo_angle){return servoSender_write(servo_angle, lidUpperRight);}

bool servoSender_write(byte servo_angle, byte servoGroup) {
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
      pwm.setPWM( chanelNum1, 0, servo1_Pwm);
    }
    if(chanelNum2<99) {
      pwm.setPWM( chanelNum2, 0, servo2_Pwm);
    }
  return true;
}
#endif
