/* i2Head_Receiver  RF NANO Prímacovy kod 

*/

#define USE_RF_REMOTE

#define USE_DISPLAY_ST7735
//#define SERVOPULSE_CONVERSION_NEEDED
//#define  SEND_FROM_0_TO_1023
#define  SEND_FROM_0_TO_255

#include "version_num.h"
#include "build_defs.h"

#ifdef USE_DISPLAY_ST7735
  #ifndef ST7735_h
    #define ST7735_h
    #include <ST7735.h>
  #endif
#endif

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
#include "Servo_Min_Max.h"
#include "i2Head_Receiver.h"
#include "ServoConnectionToPwm.h"
#include <math.h>
#include "colors.h"

#include "TxRx_dataStructures.h"

#define RANDOM_EYES_MOVEMENT

#ifdef RANDOM_EYES_MOVEMENT
  #include "RandomEyesMovement.h"
#endif

// want something like: 0.2.20241124.1502
const unsigned char completeVersion[] =
{
    VERSION_MAJOR_INIT,
    '.',
    VERSION_MINOR_INIT,
    //'-', 'V', '-',
    '.',
    BUILD_YEAR_CH0, BUILD_YEAR_CH1, BUILD_YEAR_CH2, BUILD_YEAR_CH3,
    //'-',
    BUILD_MONTH_CH0, BUILD_MONTH_CH1,
    //'-',
    BUILD_DAY_CH0, BUILD_DAY_CH1,
    //'T',
      '.',
    BUILD_HOUR_CH0, BUILD_HOUR_CH1,
    //':',
    BUILD_MIN_CH0, BUILD_MIN_CH1,
    //':',
    //BUILD_SEC_CH0, BUILD_SEC_CH1,
    '\0'
};

//#include <stdio.h>

#define HIGHSPEED 

#ifdef HIGHSPEED
  #define Baud 19200   // Serial monitor
#else
  #define Baud 9600    // Serial monitor
#endif

#ifdef USE_DISPLAY_ST7735

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

char servo[]="S";//"Servo ";
char colon[]=":";//": ";

#endif


int16_t mode;
int count;
int noDataCount = 0;

#ifdef RANDOM_EYES_MOVEMENT
  RandomEyesMovement randomEyesMovement; //  = new RandomEyesMovement();
#endif

const uint64_t pipeIn = 0x0022;   //Tento isty kod musi mať aj primač
/*
Arduion RF NANO   pinout
CE   D10
CSN  D09
SCK  D13
MOSI D11
MISO D12
 from: RF-Nano-Schematic.pdf

Arduion MEGA  NRF24L01 PA/LNA   pinout
CE   D10
CSN  D09
SCK  D52
MOSI D51
MISO D50

 SCK, MOSI, MISO and CS (or SS) pins. Those pins are 52, 51, 50 and 53 (defalut) on a Mega.
from: https://forum.arduino.cc/t/pin-connection/613444/4  

Hardware SPI Pins:
 * Arduino Uno   SCK=13, SDA=11
 * Arduino Nano  SCK=13, SDA=11
 * Arduino Due   SCK=76, SDA=75
 * Arduino Mega  SCK=52, SDA=51

*/


RF24 radio(10,9);  //zapojenie CE a CSN pinov   //RF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed = RF24_SPI_SPEED);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


unsigned long previousMillis = 0;
const long interval = 20;

long previousSafetyMillis;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

RX_DATA_STRUCTURE mydata_received;
RX_DATA_STRUCTURE prev_mydata;
//TX_DATA_STRUCTURE mydata_remote;
RX_SERIAL_DATA_STRUCTURE my_serial_data_received;
RX_SERIAL_DATA_STRUCTURE prev_my_serial_data;

byte previousServoSet = 0;

//#ifdef RANDOM_EYES_MOVEMENT
//#endif

void resetData()
{

  mydata_received.s1min = 0;
  mydata_received.s1mid = 127;
  mydata_received.s1curr= 127;
  mydata_received.s1max = 255;

  //mydata_received.s2min = 0;
  //mydata_received.s2mid = 127;
  //mydata_received.s2curr= 127;
  //mydata_received.s2max = 255;

  mydata_received.devType =  0; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
  //mydata_received.flags = 0;

  mydata_received.switchPos = 0;
  mydata_received.fireBtn1 = 0;

}

void setup()
{
  Serial.begin(Baud);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("  Sketch: ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
  Serial.println("------------------------------------------");
  Serial.print("Version: ");   Serial.write(completeVersion, strlen(completeVersion));
  Serial.println("");
  Serial.println("------------------------------------------");

  //printf("%s\n", completeVersion);
	
  #ifdef USE_DISPLAY_ST7735
    Serial.println("setup: tft.initR()...");
    tft.initR();
    //tft.initR(INITR_BLACKTAB); 

    //tft.pushColor(uint16_t color)
    //tft.pushColor(tft.Color565(RED,GREEN,BLUE));
    //tft.fillScreen(BLACK);
    //Set background colour
    Serial.println("setup: tft.fillScreen(BLACK)");
    tft.fillScreen(BLACK);
    Serial.println("setup: BLACK =done");

    prepareServoForm();
  #endif

  Serial.println("setup: @1 Servo Initialization started");
  pwm.begin(); //pwm.begin(0);   0 = driver_ID
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	Serial.println("setup: @2 Servos on PCA9685  attached");
  //all_center_points_initialized = false;
  delay(200);

  #ifdef RANDOM_EYES_MOVEMENT
    uint8_t _left_arrow_step = LEFT_ARROW_STEP;
    randomEyesMovement.begin(&pwm, &tft, servoLimits);
    //randomEyesMovement.beginDisplay(&tft);
  #endif

  //konfiguracia NRF24 
  resetData();
  //all_center_points_initialized = true;
  #ifdef USE_RF_REMOTE
  	Serial.println("setup: @3 radio.begin()..");
    radio.begin();
    //radio.setAutoAck(false);
    radio.setAutoAck(true);
    
    //Serial.println("setup: radio.setDataRate(RF24_250KBPS)");
    //radio.setDataRate(RF24_250KBPS);
    Serial.println("setup: radio.openReadingPipe(1,pipeIn)");
    radio.openReadingPipe(1,pipeIn);
    
    Serial.println("setup: radio.setPALevel(RF24_PA_MIN)");
    radio.setPALevel(RF24_PA_MIN);
   
    radio.startListening();
    Serial.println("setup: @4 rf-radio started");
  #endif

  //initMinMidMax_values();

  delay(600);

  Serial.println("setup: @8 done. setup END.");
}

#ifdef USE_RF_REMOTE
  //unsigned long lastRecvTime = 0;
  bool recvData()
  {
    bool dataReceived = false;
    while ( radio.available() ) {
      //radio.read(&data, sizeof(MyData));
      radio.read(&mydata_received, sizeof(RX_DATA_STRUCTURE));
      //lastRecvTime = millis(); //tu dostávame údaje
      dataReceived = true;
    }
    return dataReceived;
  }
#endif

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
bool RF_data_changed = false;
bool serial_data_changed = false;
bool RF_Serial_data_changed = false;
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
      //Serial.println("s1min: "+String(mydata_received.s1min)+", s1curr: "+String(mydata_received.s1curr)+", s1mid: "+String(mydata_received.s1mid)+", s1max: "+String(mydata_received.s1max)+", servoSet: "+String(mydata_received.servoSet)+", devType:"+String(mydata_received.devType));
      if (mydata_received.devType == 0) // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
      {
        constrain_RfData_0_255();
        RF_data_changed = RfData_changed();
        compute_fromRfData_toAngleData();
      }
      else if (mydata_received.devType == 1)
      {
        serial_data_changed = serialData_changed();
        if(serial_data_changed == true) {
          compute_from_SerialData_toAngleData();
          //Serial.println("@1.40 started.");
          constrain_allServoAngles_0_255();
          reset_SerialDataChanged();
        }
      }else if (mydata_received.devType == 2) {
        RF_Serial_data_changed = RfSerial_Data_changed();
          if(RF_Serial_data_changed == true) {
            //compute_from_SerialData_toMinMax();
            #ifdef RANDOM_EYES_MOVEMENT
              //randomEyesMovement.moveEyesRandomly(currentMillis);
            #endif
          }
          //reset_RfSerialData();
      }
    } else  if(currentMillis - previousSafetyMillis > 1000) {         // safeties
      #ifdef RANDOM_EYES_MOVEMENT
        randomEyesMovement.moveEyesRandomly(currentMillis);
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
      //Serial.println("@3.2 DataReceived changed, serialDataReceived="+String(serialDataReceived)+", remoteDataReceived = "+String(remoteDataReceived));
      #ifdef SERVOPULSE_CONVERSION_NEEDED
        constrain_allServoAngles_0_255();
        convert_allAngle_to_Pwm_Min_Center_Max();
      #else
        convert_constrain_allServoAngles_0_1023();
      #endif

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
String i_str ="";
void prepareServoForm(){
  Serial.println("prepareServoForm: Write servo numbers 1.for {for{}} start");
//Write servo numbers 
  for (uint8_t count = 0; count <= ((16/LEFT_ARROW_STEP) - 1); count ++){ 
    for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
      char numRead[2];
      char combined[30]= {0};
      dtostrf(servoNum, 1, 0, numRead);
      strcat(combined, servo);
      strcat(combined, numRead);
      tft.drawString(0, yPos, combined, WHITE);
      //Serial.println("setup: y:"+String(yPos)+", combined:"+String(combined)+", colon:"+String(colon)+"count:"+String(count)+", i:"+String(i)+".");
      tft.drawString((((strlen(servo) + 1)) * 8), yPos, colon, WHITE);    
      servoNum ++;
      yPos += spacing;    
      }
      yPos += (2*LEFT_ARROW_STEP); //8;
    }
Serial.println("sprepareServoForm: 1.for {for{}} done");

Serial.println("prepareServoForm: Write initial servo positions (350 to start with)  2.for {for{}} started");
//Write initial servo positions (350 to start with)  
  servoNum = 0;
  yPos = 2;
  //servo ="S".....
  for (uint8_t count = 0; count <= ((16/LEFT_ARROW_STEP) - 1); count ++){ 
    for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
        //char numRead[4];
        //dtostrf(servoLimits[servoNum], 4, 0, numRead);
        //tft.drawString((((strlen(servo) + 2)) * 8), yPos, numRead, YELLOW);
        writeMINPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoLimits[servoNum], true);

        //char numRead2[4];
        //dtostrf(servoLimits[servoNum + 16], 4, 0, numRead2);
        //tft.drawString((((strlen(servo) + 2 + 4)) * 8), yPos, numRead2, YELLOW);
        writeMIDPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoLimits[servoNum+16], true);

        writeCurrPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoLimits[servoNum+48], true);

        //char numRead3[4];
        //dtostrf(servoLimits[servoNum + 32], 4, 0, numRead3);
        //tft.drawString((((strlen(servo) + 2 + 8)) * 8), yPos, numRead3, YELLOW);
        writeMAXPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoLimits[servoNum+32], true);
        //Serial.print("prepareServoForm: y:"+String(yPos)+", count:"+String(count)+", i:"+String(i)+".");
      i_str = String(i);
      servoNum ++;
      yPos += spacing;    
      }
    yPos += (2*LEFT_ARROW_STEP); //8;
  }
  Serial.println("prepareServoForm: 2.for {for{}} done");

   tft.drawString((128-(LEFT_ARROW_SIZE*8)), 3, "<", WHITE, LEFT_ARROW_SIZE);
}


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


void compute_from_SerialData_toMinMax()
{
  servoLimits[ mydata_received.servoSet           ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s1curr, 0, 255), 0, 255, mydata_received.s1min, mydata_received.s1max);
  servoLimits[(mydata_received.servoSet + 16 )    ] = map(constrain(mydata_received.s1mid, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x00, 0, 255), 0, 255, 0, 1023);
  servoLimits[(mydata_received.servoSet + 16 + 16)] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x00, 0, 255), 0, 255, 0, 1023);
}

void compute_from_SerialData_toAngleData()
{
  servo_eyeLeftUD_Angle        = constrain(my_serial_data_received.s00, 0, 255);
  servo_eyeLeftLR_Angle        = constrain(my_serial_data_received.s01, 0, 255);
  servo_eyeRightUD_Angle       = constrain(my_serial_data_received.s02, 0, 255);
  servo_eyeRightLR_Angle       = constrain(my_serial_data_received.s03, 0, 255);
  servo_eyelidLeftUpper_Angle  = constrain(my_serial_data_received.s04, 0, 255);
  servo_eyelidLeftLower_Angle  = constrain(my_serial_data_received.s05, 0, 255);
  servo_eyelidRightUpper_Angle = constrain(my_serial_data_received.s06, 0, 255);
  servo_eyelidRightLower_Angle = constrain(my_serial_data_received.s07, 0, 255);
  servo_eyebrowRight_Angle     = constrain(my_serial_data_received.s08, 0, 255);
  servo_eyebrowLeft_Angle      = constrain(my_serial_data_received.s09, 0, 255);
  servo_cheekRight_Angle       = constrain(my_serial_data_received.s10, 0, 255);
  servo_cheekLeft_Angle        = constrain(my_serial_data_received.s11, 0, 255);
  servo_upperLip_Angle         = constrain(my_serial_data_received.s12, 0, 255);
  servo_forheadRight_Angle     = constrain(my_serial_data_received.s13, 0, 255);
  servo_forheadLeft_Angle      = constrain(my_serial_data_received.s14, 0, 255);
  servo_Jaw_UpDown_Angle       = constrain(my_serial_data_received.s15, 0, 255);

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
  if (my_serial_data_received.mode == 0) //...devType   mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 3 = ?
  {
    ch[1] = constrain(my_serial_data_received.s00, 0, 255);
    ch[2] = constrain(my_serial_data_received.s01, 0, 255);
    ch[3] = constrain(my_serial_data_received.s02, 0, 255);
    ch[4] = constrain(my_serial_data_received.s03, 0, 255);
    ch[5] = constrain(my_serial_data_received.s04, 0, 255);
    ch[6] = constrain(my_serial_data_received.s05, 0, 255);
    ch[7] = constrain(my_serial_data_received.s06, 0, 255);
    ch[8] = constrain(my_serial_data_received.s07, 0, 255);
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

void convert_constrain_allServoAngles_0_1023() {
  servo_eyeLeftUD_Pwm        = constrain(servo_eyeLeftUD_Angle       , 0, 1023);
  servo_eyeLeftLR_Pwm        = constrain(servo_eyeLeftLR_Angle       , 0, 1023);
  servo_eyeRightUD_Pwm       = constrain(servo_eyeRightUD_Angle      , 0, 1023);
  servo_eyeRightLR_Pwm       = constrain(servo_eyeRightLR_Angle      , 0, 1023);
  servo_eyelidLeftUpper_Pwm  = constrain(servo_eyelidLeftUpper_Angle , 0, 1023);
  servo_eyelidLeftLower_Pwm  = constrain(servo_eyelidLeftLower_Angle , 0, 1023);
  servo_eyelidRightUpper_Pwm = constrain(servo_eyelidRightUpper_Angle, 0, 1023);
  servo_eyelidRightLower_Pwm = constrain(servo_eyelidRightLower_Angle, 0, 1023);
  servo_eyebrowRight_Pwm     = constrain(servo_eyebrowRight_Angle    , 0, 1023);
  servo_eyebrowLeft_Pwm      = constrain(servo_eyebrowLeft_Angle     , 0, 1023);
  servo_cheekRight_Pwm       = constrain(servo_cheekRight_Angle      , 0, 1023);
  servo_cheekLeft_Pwm        = constrain(servo_cheekLeft_Angle       , 0, 1023);
  servo_upperLip_Pwm         = constrain(servo_upperLip_Angle        , 0, 1023);
  servo_forheadRight_Pwm     = constrain(servo_forheadRight_Angle    , 0, 1023);
  servo_forheadLeft_Pwm      = constrain(servo_forheadLeft_Angle     , 0, 1023);
  servo_Jaw_UpDown_Pwm       = constrain(servo_Jaw_UpDown_Angle      , 0, 1023);
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

bool RfSerial_Data_Changed_innerPart(int16_t servoIndex, int16_t displayChanelNumber, uint16_t mydata_received_value,uint8_t form_label_Min_Mid_Max) //, String ServoIndexLabel)
{
  bool data_changed = false;

  #ifdef SERVOPULSE_CONVERSION_NEEDED
    servoLimits[servoIndex] = map(constrain(mydata_received_value, 0, 255), 0, 255, 0, 1023);
  #else
    servoLimits[servoIndex] = constrain(mydata_received_value, 0, 1023);
  #endif

  //map(constrain(mydata_received.s1curr, 0, 255), 0, 255, mydata_received.s1min, mydata_received.s1max);
  if(servoLimits[servoIndex] != prevServoLimits[servoIndex])
  {
    data_changed = true; 
    //if(abs(servoLimits[servoIndex] - prevServoLimits[servoIndex])>=2)
    //{
      Serial.println("RfSerial_Data_Changed_innerPart: servoIndex="+String(servoIndex)+", displayChanelNumber="+ String(displayChanelNumber)+ ", mydata_received_value ="+ String(mydata_received_value)+", form_label_Min_Mid_Max="+String(form_label_Min_Mid_Max)+" ,servoLimits["+String(servoIndex)+"] = "+ String(servoLimits[servoIndex])+". ");
    ////Serial.println(" servoLimits["+String(servoIndex)+"] = "+ String(servoLimits[servoIndex])+" ");
    //}
    if(form_label_Min_Mid_Max == LABEL_FORM_MIN) {
      writeMINPulsesToDisplay( displayChanelNumber, servoLimits[servoIndex]);
    }
    else if(form_label_Min_Mid_Max == LABEL_FORM_MID) {
      writeMIDPulsesToDisplay( displayChanelNumber, servoLimits[servoIndex]);
    }
    else if(form_label_Min_Mid_Max == LABEL_FORM_MAX) {
      writeMAXPulsesToDisplay( displayChanelNumber, servoLimits[servoIndex]);
    }
    else {
      writeCurrPulsesToDisplay( displayChanelNumber, servoLimits[servoIndex]);
    }
    //writeOneFieldToDisplay(displayChanelNumber, form_label_Min_Mid_Max, servoLimits[servoIndex]); 
    prevServoLimits[servoIndex] = servoLimits[servoIndex];
  }

  return data_changed;
}
#define USE_RF_SERIAL_D_CH_INNER_PART
bool RfSerial_Data_changed() {
  bool data_changed = false;

  //#ifdef USE_RF_SERIAL_D_CH_INNER_PART
    if(mydata_received.servoSet != previousServoSet) {
      Serial.println("RfSerial_Data_changed: mydata_received.servoSet CHANGED to  "+String(mydata_received.servoSet)+".");
      writeArrow_activeServoSet(mydata_received.servoSet);
      previousServoSet = mydata_received.servoSet;
    }
    servoPositionChanged[(mydata_received.servoSet+ 0)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+ 0), mydata_received.servoSet, mydata_received.s1min,  LABEL_FORM_MIN);
    servoPositionChanged[(mydata_received.servoSet+16)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+16), mydata_received.servoSet, mydata_received.s1mid,  LABEL_FORM_MID);
    servoPositionChanged[(mydata_received.servoSet+32)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+32), mydata_received.servoSet, mydata_received.s1max,  LABEL_FORM_MAX);
    
    #ifdef SEND_FROM_0_TO_255
      uint16_t extrapolatedCurrentValue =
          (mydata_received.s1curr)<128 ? 
            (map(mydata_received.s1curr,   0, 127, servoLimits[(mydata_received.servoSet)           ], servoLimits[(mydata_received.servoSet) + (16 * 1)]) ) 
          : (map(mydata_received.s1curr, 128, 255, servoLimits[(mydata_received.servoSet) + (16 * 1)], servoLimits[(mydata_received.servoSet) + (16 * 2)]) );

      servoPositionChanged[(mydata_received.servoSet+48)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+48), mydata_received.servoSet, extrapolatedCurrentValue, LABEL_FORM_MAX+1);
    #else
      servoPositionChanged[(mydata_received.servoSet+48)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+48), mydata_received.servoSet, mydata_received.s1curr, LABEL_FORM_MAX+1);
    #endif

    if(servoPositionChanged[mydata_received.servoSet   ] == true ||
       servoPositionChanged[mydata_received.servoSet+16] == true ||
       servoPositionChanged[mydata_received.servoSet+32] == true ||
       servoPositionChanged[mydata_received.servoSet+48] == true ) 
    {
      data_changed = true; 
    }

  if(data_changed == true){
      //Serial.println(" my_serial_data_received_changed ----");
    }
  return data_changed;
}

void reset_RfSerialData() {
  for (short i=0; i<=47; i++) {
    prevServoLimits[i] = servoLimits[i];
  }
}



bool serialData_changed() {
  bool data_changed = false;
  if(my_serial_data_received.s00 != prev_my_serial_data.s00){data_changed = true; s00_changed = true; writeMINPulsesToDisplay( 0,my_serial_data_received.s00); Serial.print("s00 = "+ String(my_serial_data_received.s00)+" ");}
  if(my_serial_data_received.s01 != prev_my_serial_data.s01){data_changed = true; s01_changed = true; writeMINPulsesToDisplay( 1,my_serial_data_received.s01); Serial.print("s01 = "+ String(my_serial_data_received.s01)+" ");}
  if(my_serial_data_received.s02 != prev_my_serial_data.s02){data_changed = true; s02_changed = true; writeMINPulsesToDisplay( 2,my_serial_data_received.s02); Serial.print("s02 = "+ String(my_serial_data_received.s02)+" ");}
  if(my_serial_data_received.s03 != prev_my_serial_data.s03){data_changed = true; s03_changed = true; writeMINPulsesToDisplay( 3,my_serial_data_received.s03); Serial.print("s03 = "+ String(my_serial_data_received.s03)+" ");}
  if(my_serial_data_received.s04 != prev_my_serial_data.s04){data_changed = true; s04_changed = true; writeMINPulsesToDisplay( 4,my_serial_data_received.s04); Serial.print("s04 = "+ String(my_serial_data_received.s04)+" ");}
  if(my_serial_data_received.s05 != prev_my_serial_data.s05){data_changed = true; s05_changed = true; writeMINPulsesToDisplay( 5,my_serial_data_received.s05); Serial.print("s05 = "+ String(my_serial_data_received.s05)+" ");}
  if(my_serial_data_received.s06 != prev_my_serial_data.s06){data_changed = true; s06_changed = true; writeMINPulsesToDisplay( 6,my_serial_data_received.s06); Serial.print("s06 = "+ String(my_serial_data_received.s06)+" ");}
  if(my_serial_data_received.s07 != prev_my_serial_data.s07){data_changed = true; s07_changed = true; writeMINPulsesToDisplay( 7,my_serial_data_received.s07); Serial.print("s07 = "+ String(my_serial_data_received.s07)+" ");}
  if(my_serial_data_received.s08 != prev_my_serial_data.s08){data_changed = true; s08_changed = true; writeMINPulsesToDisplay( 8,my_serial_data_received.s08); Serial.print("s08 = "+ String(my_serial_data_received.s08)+" ");}
  if(my_serial_data_received.s09 != prev_my_serial_data.s09){data_changed = true; s09_changed = true; writeMINPulsesToDisplay( 9,my_serial_data_received.s09); Serial.print("s09 = "+ String(my_serial_data_received.s09)+" ");}
  if(my_serial_data_received.s10 != prev_my_serial_data.s10){data_changed = true; s10_changed = true; writeMINPulsesToDisplay(10,my_serial_data_received.s10); Serial.print("s10 = "+ String(my_serial_data_received.s10)+" ");}
  if(my_serial_data_received.s11 != prev_my_serial_data.s11){data_changed = true; s11_changed = true; writeMINPulsesToDisplay(11,my_serial_data_received.s11); Serial.print("s11 = "+ String(my_serial_data_received.s11)+" ");}
  if(my_serial_data_received.s12 != prev_my_serial_data.s12){data_changed = true; s12_changed = true; writeMINPulsesToDisplay(12,my_serial_data_received.s12); Serial.print("s12 = "+ String(my_serial_data_received.s12)+" ");}
  if(my_serial_data_received.s13 != prev_my_serial_data.s13){data_changed = true; s13_changed = true; writeMINPulsesToDisplay(13,my_serial_data_received.s13); Serial.print("s13 = "+ String(my_serial_data_received.s13)+" ");}
  if(my_serial_data_received.s14 != prev_my_serial_data.s14){data_changed = true; s14_changed = true; writeMINPulsesToDisplay(14,my_serial_data_received.s14); Serial.print("s14 = "+ String(my_serial_data_received.s14)+" ");}
  if(my_serial_data_received.s15 != prev_my_serial_data.s15){data_changed = true; s15_changed = true; writeMINPulsesToDisplay(15,my_serial_data_received.s15); Serial.print("s15 = "+ String(my_serial_data_received.s15)+" ");}
 
  if(my_serial_data_received.x00 != prev_my_serial_data.x00){data_changed = true; x00_changed = true; writeMAXPulsesToDisplay(32,my_serial_data_received.x00); Serial.print("x00 = "+ String(my_serial_data_received.x00)+" ");}
  if(my_serial_data_received.x01 != prev_my_serial_data.x01){data_changed = true; x01_changed = true; writeMAXPulsesToDisplay(33,my_serial_data_received.x01); Serial.print("x01 = "+ String(my_serial_data_received.x01)+" ");}
  if(my_serial_data_received.x02 != prev_my_serial_data.x02){data_changed = true; x02_changed = true; writeMAXPulsesToDisplay(34,my_serial_data_received.x02); Serial.print("x02 = "+ String(my_serial_data_received.x02)+" ");}
  if(my_serial_data_received.x03 != prev_my_serial_data.x03){data_changed = true; x03_changed = true; writeMAXPulsesToDisplay(35,my_serial_data_received.x03); Serial.print("x03 = "+ String(my_serial_data_received.x03)+" ");}
  if(my_serial_data_received.x04 != prev_my_serial_data.x04){data_changed = true; x04_changed = true; writeMAXPulsesToDisplay(36,my_serial_data_received.x04); Serial.print("x04 = "+ String(my_serial_data_received.x04)+" ");}
  if(my_serial_data_received.x05 != prev_my_serial_data.x05){data_changed = true; x05_changed = true; writeMAXPulsesToDisplay(37,my_serial_data_received.x05); Serial.print("x05 = "+ String(my_serial_data_received.x05)+" ");}
  if(my_serial_data_received.x06 != prev_my_serial_data.x06){data_changed = true; x06_changed = true; writeMAXPulsesToDisplay(38,my_serial_data_received.x06); Serial.print("x06 = "+ String(my_serial_data_received.x06)+" ");}
  if(my_serial_data_received.x07 != prev_my_serial_data.x07){data_changed = true; x07_changed = true; writeMAXPulsesToDisplay(39,my_serial_data_received.x07); Serial.print("x07 = "+ String(my_serial_data_received.x07)+" ");}
  if(my_serial_data_received.x08 != prev_my_serial_data.x08){data_changed = true; x08_changed = true; writeMAXPulsesToDisplay(40,my_serial_data_received.x08); Serial.print("x08 = "+ String(my_serial_data_received.x08)+" ");}
  if(my_serial_data_received.x09 != prev_my_serial_data.x09){data_changed = true; x09_changed = true; writeMAXPulsesToDisplay(41,my_serial_data_received.x09); Serial.print("x09 = "+ String(my_serial_data_received.x09)+" ");}
  if(my_serial_data_received.x10 != prev_my_serial_data.x10){data_changed = true; x10_changed = true; writeMAXPulsesToDisplay(42,my_serial_data_received.x10); Serial.print("x10 = "+ String(my_serial_data_received.x10)+" ");}
  if(my_serial_data_received.x11 != prev_my_serial_data.x11){data_changed = true; x11_changed = true; writeMAXPulsesToDisplay(43,my_serial_data_received.x11); Serial.print("x11 = "+ String(my_serial_data_received.x11)+" ");}
  if(my_serial_data_received.x12 != prev_my_serial_data.x12){data_changed = true; x12_changed = true; writeMAXPulsesToDisplay(44,my_serial_data_received.x12); Serial.print("x12 = "+ String(my_serial_data_received.x12)+" ");}
  if(my_serial_data_received.x13 != prev_my_serial_data.x13){data_changed = true; x13_changed = true; writeMAXPulsesToDisplay(45,my_serial_data_received.x13); Serial.print("x13 = "+ String(my_serial_data_received.x13)+" ");}
  if(my_serial_data_received.x14 != prev_my_serial_data.x14){data_changed = true; x14_changed = true; writeMAXPulsesToDisplay(46,my_serial_data_received.x14); Serial.print("x14 = "+ String(my_serial_data_received.x14)+" ");}
  if(my_serial_data_received.x15 != prev_my_serial_data.x15){data_changed = true; x15_changed = true; writeMAXPulsesToDisplay(47,my_serial_data_received.x15); Serial.print("x15 = "+ String(my_serial_data_received.x15)+" ");}

  if(data_changed == true){
      //Serial.println(" my_serial_data_received_changed ----");
    }
  return data_changed;
}

void reset_SerialDataChanged()
{
  prev_my_serial_data.s00 = my_serial_data_received.s00; 
  prev_my_serial_data.s01 = my_serial_data_received.s01; 
  prev_my_serial_data.s02 = my_serial_data_received.s02; 
  prev_my_serial_data.s03 = my_serial_data_received.s03; 
  prev_my_serial_data.s04 = my_serial_data_received.s04; 
  prev_my_serial_data.s05 = my_serial_data_received.s05; 
  prev_my_serial_data.s06 = my_serial_data_received.s06; 
  prev_my_serial_data.s07 = my_serial_data_received.s07; 
  prev_my_serial_data.s08 = my_serial_data_received.s08; 
  prev_my_serial_data.s09 = my_serial_data_received.s09; 
  prev_my_serial_data.s10 = my_serial_data_received.s10; 
  prev_my_serial_data.s11 = my_serial_data_received.s11; 
  prev_my_serial_data.s12 = my_serial_data_received.s12; 
  prev_my_serial_data.s13 = my_serial_data_received.s13; 
  prev_my_serial_data.s14 = my_serial_data_received.s14; 
  prev_my_serial_data.s15 = my_serial_data_received.s15;

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

void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, servo_Pwm, showDebug);  
}
void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, servo_Pwm, false);  
}


void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm, showDebug);
}
void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm, false);
}

void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  writeOneFieldToDisplay (chanelNum, (LABEL_FORM_MAX+1) , servo_Pwm, showDebug);
}

void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MAX+1, servo_Pwm, false);
}

void writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  writeOneFieldToDisplay (chanelNum, 2, servo_Pwm, showDebug);
}

void writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  writeOneFieldToDisplay (chanelNum, 2, servo_Pwm, false);
}


#define char_width_x 8

#define char_height_y 8  
#define char_shift_x  2
#define chr_point_shift_x  1

void writeOneFieldToDisplay (uint8_t chanelNum,uint8_t form_label_Min_Mid_Max, uint16_t servo_Pwm, bool showDebug){
  uint8_t modulo = chanelNum % LEFT_ARROW_STEP;
  uint8_t div_result =chanelNum / LEFT_ARROW_STEP;
  uint8_t yPos = 2 + (div_result * ((LEFT_ARROW_STEP*8)+2)) + (modulo*8);

  if(showDebug == true) {
    Serial.print("writePulsesToDisplay: ");
    Serial.print("chanelNum:"+String(chanelNum)+", form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", servo_Pwm:"+String(servo_Pwm)+",  ");
    Serial.print("div_result = "+String(div_result)+", modulo = "+String(modulo)+", ");
    Serial.println("yPos:"+String(yPos)+", ");
  } else {
    //Serial.println("writeOneFieldToDisplay: yPos:"+String(yPos)+", chanelNum:"+String(chanelNum)+", form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", servo_Pwm:"+String(servo_Pwm)+", servoPulseIndex:"+String(servoPulseIndex));
  }

  uint8_t xPos = (((char_shift_x + (form_label_Min_Mid_Max*3)) * char_width_x));
  writeOneFieldToDisplay_innerPart(xPos, chr_point_shift_x, yPos, char_height_y, form_label_Min_Mid_Max, servo_Pwm, chanelNum, showDebug);
}

void writeOneFieldToDisplay_innerPart (uint8_t xPos, uint16_t _chr_point_shift_x, uint8_t yPos, uint16_t _char_height_y, uint16_t form_label_Min_Mid_Max, uint16_t servo_Pwm,uint16_t chanelNum, bool showDebug)
{
  //uint8_t modulo2 = (chanelNum + form_label_Min_Mid_Max)%2;
    //if(modulo2 ==0) 
    //{
      tft.fillRect((xPos + _chr_point_shift_x), yPos, (3*char_width_x)-_chr_point_shift_x, _char_height_y, BLACK);
    //} else {
    //    tft.fillRect((xPos + _chr_point_shift_x), yPos, (3*char_width_x)-_chr_point_shift_x, _char_height_y, WHITE);
    //}
    char numRead3[4];
    dtostrf(servo_Pwm, 4, 0, numRead3);
    tft.drawString(xPos, yPos, numRead3, YELLOW);
}

void writeArrow_activeServoSet (byte activeServoSet) {
      tft.fillRect((128-(LEFT_ARROW_SIZE*8)), 0, (LEFT_ARROW_SIZE*8), 160, BLACK);
      tft.drawString((128-(LEFT_ARROW_SIZE*8)), ((activeServoSet * ((2+8) * LEFT_ARROW_STEP))+3), "<", WHITE, LEFT_ARROW_SIZE);
}

