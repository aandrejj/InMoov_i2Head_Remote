/* i2Head_Receiver  RF NANO Prímacovy kod 

*/

#define USE_RF_REMOTE

#define USE_DISPLAY_ST7735

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

#define HIGHSPEED 

#ifdef HIGHSPEED
  #define Baud 19200   // Serial monitor
#else
  #define Baud 9600    // Serial monitor
#endif

#ifdef USE_DISPLAY_ST7735

  //#define OLED_RESET 4
  #define DISP_CS    6
  #define DISP_RS    7
  #define DISP_RST   8
  #define DISP_SID   4
  #define DISP_SCLK  5
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
RX_SERIAL_DATA_STRUCTURE my_serial_data_received;
RX_SERIAL_DATA_STRUCTURE prev_my_serial_data;

//#ifdef RANDOM_EYES_MOVEMENT
//#endif

void resetData()
{

  mydata_received.s1min = 0;
  mydata_received.s1mid = 127;
  mydata_received.s1curr= 127;
  mydata_received.s1max = 255;

  mydata_received.s2min = 0;
  mydata_received.s2mid = 127;
  mydata_received.s2curr= 127;
  mydata_received.s2max = 255;

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
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);
	
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

  Serial.println("setup:: @1 Servo Initialization started");
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
    radio.setAutoAck(false);
    radio.setDataRate(RF24_250KBPS);  
    radio.openReadingPipe(1,pipeIn);
    
    radio.startListening();
    Serial.println("setup: @4 rf-radio started");
  #endif

  //initMinMidMax_values();

  delay(500);

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
          reset_RfSerialData();
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
      Serial.println("@3.2 DataReceived changed, serialDataReceived="+String(serialDataReceived)+", remoteDataReceived = "+String(remoteDataReceived));
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

void prepareServoForm(){
  Serial.println("setup: Write servo numbers 1.for {for{}} start");
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
Serial.println("setup: 1.for {for{}} done");

Serial.println("setup: Write initial servo positions (350 to start with)  2.for {for{}} started");
//Write initial servo positions (350 to start with)  
  servoNum = 0;
  yPos = 2;
  for (uint8_t count = 0; count <= ((16/LEFT_ARROW_STEP) - 1); count ++){ 
    for (uint8_t i = 0; i <=(LEFT_ARROW_STEP - 1); i ++){
      if(LEFT_ARROW_STEP>2) {
        char numRead[4];
        dtostrf(servoLimits[servoNum], 4, 0, numRead);
        tft.drawString((((strlen(servo) + 2)) * 8), yPos, numRead, YELLOW);
      } else {
        char numRead[4];
        dtostrf(servoLimits[servoNum], 4, 0, numRead);
        tft.drawString((((strlen(servo) + 2)) * 8), yPos, numRead, YELLOW);

        char numRead2[4];
        dtostrf(servoLimits[servoNum + 16], 4, 0, numRead2);
        tft.drawString((((strlen(servo) + 2 + 4)) * 8), yPos, numRead2, YELLOW);

        char numRead3[4];
        dtostrf(servoLimits[servoNum + 32], 4, 0, numRead3);
        tft.drawString((((strlen(servo) + 2 + 8)) * 8), yPos, numRead3, YELLOW);

      }
      //Serial.println("setup: y:"+String(yPos)+", numRead:"+String(numRead)+", count:"+String(count)+", i:"+String(i)+".");
      servoNum ++;
      yPos += spacing;    
      }
    yPos += (2*LEFT_ARROW_STEP); //8;
  }
  Serial.println("setup: 2.for {for{}} done");

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
  switch (mydata_received.servoSet) {
    case 0: 
        servoLimits[LBL_SRV_MIN_eyeLeftUD        ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s1curr, 0, 255), 0, 255, mydata_received.s1min, mydata_received.s1max);
        servoLimits[LBL_SRV_MIN_eyeLeftLR        ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s01, 0, 255), 0, 255, 0, 1023);
      break;
    case 1: 
        servoLimits[LBL_SRV_MIN_eyeRightUD       ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s02, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyeRightLR       ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s03, 0, 255), 0, 255, 0, 1023);
      break;
    case 2: 
        servoLimits[LBL_SRV_MIN_eyelidLeftUpper  ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s04, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyelidLeftLower  ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s05, 0, 255), 0, 255, 0, 1023);
      break;
    case 3: 
        servoLimits[LBL_SRV_MIN_eyelidRightUpper ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s06, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyelidRightLower ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s07, 0, 255), 0, 255, 0, 1023);
      break;
    case 4: 
        servoLimits[LBL_SRV_MIN_eyebrowRight     ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s08, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyebrowLeft      ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s09, 0, 255), 0, 255, 0, 1023);
      break;
    case 5: 
        servoLimits[LBL_SRV_MIN_cheekRight       ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s10, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_cheekLeft        ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s11, 0, 255), 0, 255, 0, 1023);
      break;
    case 6: 
        servoLimits[LBL_SRV_MIN_upperLip         ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s12, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_forheadRight     ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s13, 0, 255), 0, 255, 0, 1023);
      break;
    case 7: 
        servoLimits[LBL_SRV_MIN_forheadLeft      ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s14, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_Jaw_UpDown       ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s15, 0, 255), 0, 255, 0, 1023);
      break;
  }

  switch (mydata_received.servoSet) {
    case 0: 
        servoLimits[LBL_SRV_MAX_eyeLeftUD        ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x00, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyeLeftLR        ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x01, 0, 255), 0, 255, 0, 1023);
      break;
    case 1: 
        servoLimits[LBL_SRV_MAX_eyeRightUD       ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x02, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyeRightLR       ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x03, 0, 255), 0, 255, 0, 1023);
      break;
    case 2: 
        servoLimits[LBL_SRV_MAX_eyelidLeftUpper  ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x04, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyelidLeftLower  ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x05, 0, 255), 0, 255, 0, 1023);
      break;
    case 3: 
        servoLimits[LBL_SRV_MAX_eyelidRightUpper ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x06, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyelidRightLower ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x07, 0, 255), 0, 255, 0, 1023);
      break;
    case 4: 
        servoLimits[LBL_SRV_MAX_eyebrowRight     ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x08, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyebrowLeft      ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x09, 0, 255), 0, 255, 0, 1023);
      break;
    case 5: 
        servoLimits[LBL_SRV_MAX_cheekRight       ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x10, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_cheekLeft        ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x11, 0, 255), 0, 255, 0, 1023);
      break;
    case 6: 
        servoLimits[LBL_SRV_MAX_upperLip         ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x12, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_forheadRight     ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x13, 0, 255), 0, 255, 0, 1023);
      break;
    case 7: 
        servoLimits[LBL_SRV_MAX_forheadLeft      ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x14, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_Jaw_UpDown       ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x15, 0, 255), 0, 255, 0, 1023);
      break;
    }

  switch (mydata_received.servoSet) {
    case 0: 
        servoLimits[LBL_SRV_MID_eyeLeftUD        ] =  round((servoLimits[LBL_SRV_MAX_eyeLeftUD         ] + servoLimits[LBL_SRV_MIN_eyeLeftUD       ])/2);
        servoLimits[LBL_SRV_MID_eyeLeftLR        ] =  round((servoLimits[LBL_SRV_MAX_eyeLeftLR         ] + servoLimits[LBL_SRV_MIN_eyeLeftLR       ])/2);
      break;
    case 1: 
        servoLimits[LBL_SRV_MID_eyeRightUD       ] =  round((servoLimits[LBL_SRV_MAX_eyeRightUD        ] + servoLimits[LBL_SRV_MIN_eyeRightUD      ])/2);
        servoLimits[LBL_SRV_MID_eyeRightLR       ] =  round((servoLimits[LBL_SRV_MAX_eyeRightLR        ] + servoLimits[LBL_SRV_MIN_eyeRightLR      ])/2);
      break;
    case 2: 
        servoLimits[LBL_SRV_MID_eyelidLeftUpper  ] =  round((servoLimits[LBL_SRV_MAX_eyelidLeftUpper   ] + servoLimits[LBL_SRV_MIN_eyelidLeftUpper ])/2);
        servoLimits[LBL_SRV_MID_eyelidLeftLower  ] =  round((servoLimits[LBL_SRV_MAX_eyelidLeftLower   ] + servoLimits[LBL_SRV_MIN_eyelidLeftLower ])/2);
      break;
    case 3: 
        servoLimits[LBL_SRV_MID_eyelidRightUpper ] =  round((servoLimits[LBL_SRV_MAX_eyelidRightUpper  ] + servoLimits[LBL_SRV_MIN_eyelidRightUpper])/2);
        servoLimits[LBL_SRV_MID_eyelidRightLower ] =  round((servoLimits[LBL_SRV_MAX_eyelidRightLower  ] + servoLimits[LBL_SRV_MIN_eyelidRightLower])/2);
      break;
    case 4: 
        servoLimits[LBL_SRV_MID_eyebrowRight     ] =  round((servoLimits[LBL_SRV_MAX_eyebrowRight      ] + servoLimits[LBL_SRV_MIN_eyebrowRight    ])/2);
        servoLimits[LBL_SRV_MID_eyebrowLeft      ] =  round((servoLimits[LBL_SRV_MAX_eyebrowLeft       ] + servoLimits[LBL_SRV_MIN_eyebrowLeft     ])/2);
      break;
    case 5: 
        servoLimits[LBL_SRV_MID_cheekRight       ] =  round((servoLimits[LBL_SRV_MAX_cheekRight        ] + servoLimits[LBL_SRV_MIN_cheekRight      ])/2);
        servoLimits[LBL_SRV_MID_cheekLeft        ] =  round((servoLimits[LBL_SRV_MAX_cheekLeft         ] + servoLimits[LBL_SRV_MIN_cheekLeft       ])/2);
      break;
    case 6: 
        servoLimits[LBL_SRV_MID_upperLip         ] =  round((servoLimits[LBL_SRV_MAX_upperLip          ] + servoLimits[LBL_SRV_MIN_upperLip        ])/2);
        servoLimits[LBL_SRV_MID_forheadRight     ] =  round((servoLimits[LBL_SRV_MAX_forheadRight      ] + servoLimits[LBL_SRV_MIN_forheadRight    ])/2);
      break;
    case 7: 
        servoLimits[LBL_SRV_MID_forheadLeft      ] =  round((servoLimits[LBL_SRV_MAX_forheadLeft       ] + servoLimits[LBL_SRV_MIN_forheadLeft     ])/2);
        servoLimits[LBL_SRV_MID_Jaw_UpDown       ] =  round((servoLimits[LBL_SRV_MAX_Jaw_UpDown        ] + servoLimits[LBL_SRV_MIN_Jaw_UpDown      ])/2);
      break;
  }
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

bool RfSerial_Data_changed() {
  bool data_changed = false;

  /*
  if(my_serial_data_received.s1min != prev_my_serial_data.s1min){data_changed = true; s1min_changed = true; writeMINPulsesToDisplay( 0, my_serial_data_received.s1min); Serial.print("s1min = "+ String(my_serial_data_received.s1min)+" ");}
  if(my_serial_data_received.s1mid != prev_my_serial_data.s1mid){data_changed = true; s1mid_changed = true; writeMIDPulsesToDisplay( 0, my_serial_data_received.s1mid); Serial.print("s1mid = "+ String(my_serial_data_received.s1mid)+" ");}
  if(my_serial_data_received.s1max != prev_my_serial_data.s1max){data_changed = true; s1max_changed = true; writeMINPulsesToDisplay( 0, my_serial_data_received.s1max); Serial.print("s1max = "+ String(my_serial_data_received.s1max)+" ");}

  if(my_serial_data_received.s2min != prev_my_serial_data.s2min){data_changed = true; s2min_changed = true; writeMINPulsesToDisplay( 1, my_serial_data_received.s2min); Serial.print("s2min = "+ String(my_serial_data_received.s2min)+" ");}
  if(my_serial_data_received.s2mid != prev_my_serial_data.s2mid){data_changed = true; s2mid_changed = true; writeMIDPulsesToDisplay( 1, my_serial_data_received.s2mid); Serial.print("s2mid = "+ String(my_serial_data_received.s2mid)+" ");}
  if(my_serial_data_received.s2max != prev_my_serial_data.s2max){data_changed = true; s2max_changed = true; writeMINPulsesToDisplay( 1, my_serial_data_received.s2max); Serial.print("s2max = "+ String(my_serial_data_received.s2max)+" ");}

  //  byte fireBtn1;

  //  byte switchPos;
  */
  switch (mydata_received.servoSet) {
    case 0: 
        servoLimits[LBL_SRV_MIN_eyeLeftUD] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s1curr, 0, 255), 0, 255, mydata_received.s1min, mydata_received.s1max);
        servoLimits[LBL_SRV_MIN_eyeLeftLR] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s01, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_eyeLeftUD] != prevServoLimits[LBL_SRV_MIN_eyeLeftUD]){data_changed = true; SRV_MIN_eyeLeftUD_changed = true; writeMINPulsesToDisplay( 0, servoLimits[LBL_SRV_MIN_eyeLeftUD]); Serial.print("servoLimits[LBL_SRV_MIN_eyeLeftUD] = "+ String(servoLimits[LBL_SRV_MIN_eyeLeftUD])+" ");}
        if(servoLimits[LBL_SRV_MIN_eyeLeftLR] != prevServoLimits[LBL_SRV_MIN_eyeLeftLR]){data_changed = true; SRV_MIN_eyeLeftLR_changed = true; writeMINPulsesToDisplay( 1, servoLimits[LBL_SRV_MIN_eyeLeftLR]); Serial.print("servoLimits[LBL_SRV_MIN_eyeLeftLR] = " + String(servoLimits[LBL_SRV_MIN_eyeLeftLR])+" ");}
      break;
    case 1: 
        servoLimits[LBL_SRV_MIN_eyeRightUD] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s02, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyeRightLR] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s03, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_eyeRightUD] != prevServoLimits[LBL_SRV_MIN_eyeRightUD]){data_changed = true; SRV_MIN_eyeRightUD_changed = true; writeMINPulsesToDisplay( 2, servoLimits[LBL_SRV_MIN_eyeRightUD]); Serial.print("servoLimits[LBL_SRV_MIN_eyeRightUD] = "+ String(servoLimits[LBL_SRV_MIN_eyeRightUD])+" ");}
        if(servoLimits[LBL_SRV_MIN_eyeRightLR] != prevServoLimits[LBL_SRV_MIN_eyeRightLR]){data_changed = true; SRV_MIN_eyeRightLR_changed = true; writeMINPulsesToDisplay( 3, servoLimits[LBL_SRV_MIN_eyeRightLR]); Serial.print("servoLimits[LBL_SRV_MIN_eyeRightLR] = "+ String(servoLimits[LBL_SRV_MIN_eyeRightLR])+" ");}
      break;
    case 2: 
        servoLimits[LBL_SRV_MIN_eyelidLeftUpper  ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s04, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyelidLeftLower  ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s05, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_eyelidLeftUpper] != prevServoLimits[LBL_SRV_MIN_eyelidLeftUpper]){data_changed = true; SRV_MIN_eyelidLeftUpper_changed = true; writeMINPulsesToDisplay( 4, servoLimits[LBL_SRV_MIN_eyelidLeftUpper]); Serial.print("servoLimits[LBL_SRV_MIN_eyelidLeftUpper] = "+ String(servoLimits[LBL_SRV_MIN_eyelidLeftUpper])+" ");}
        if(servoLimits[LBL_SRV_MIN_eyelidLeftLower] != prevServoLimits[LBL_SRV_MIN_eyelidLeftLower]){data_changed = true; SRV_MIN_eyelidLeftLower_changed = true; writeMINPulsesToDisplay( 5, servoLimits[LBL_SRV_MIN_eyelidLeftLower]); Serial.print("servoLimits[LBL_SRV_MIN_eyelidLeftLower] = "+ String(servoLimits[LBL_SRV_MIN_eyelidLeftLower])+" ");}
      break;
    case 3: 
        servoLimits[LBL_SRV_MIN_eyelidRightUpper ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s06, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyelidRightLower ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s07, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_eyelidRightUpper] != prevServoLimits[LBL_SRV_MIN_eyelidRightUpper]){data_changed = true; SRV_MIN_eyelidRightUpper_changed = true; writeMINPulsesToDisplay( 6, servoLimits[LBL_SRV_MIN_eyelidRightUpper]); Serial.print("servoLimits[LBL_SRV_MIN_eyelidRightUpper] = "+ String(servoLimits[LBL_SRV_MIN_eyelidRightUpper])+" ");}
        if(servoLimits[LBL_SRV_MIN_eyelidRightLower] != prevServoLimits[LBL_SRV_MIN_eyelidRightLower]){data_changed = true; SRV_MIN_eyelidRightLower_changed = true; writeMINPulsesToDisplay( 7, servoLimits[LBL_SRV_MIN_eyelidRightLower]); Serial.print("servoLimits[LBL_SRV_MIN_eyelidRightLower] = "+ String(servoLimits[LBL_SRV_MIN_eyelidRightLower])+" ");}
      break;
    case 4: 
        servoLimits[LBL_SRV_MIN_eyebrowRight     ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s08, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_eyebrowLeft      ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s09, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_eyebrowRight] != prevServoLimits[LBL_SRV_MIN_eyebrowRight]){data_changed = true; SRV_MIN_eyebrowRight_changed = true; writeMINPulsesToDisplay( 8, servoLimits[LBL_SRV_MIN_eyebrowRight]); Serial.print("servoLimits[LBL_SRV_MIN_eyebrowRight] = "+ String(servoLimits[LBL_SRV_MIN_eyebrowRight])+" ");}
        if(servoLimits[LBL_SRV_MIN_eyebrowLeft ] != prevServoLimits[LBL_SRV_MIN_eyebrowLeft ]){data_changed = true; SRV_MIN_eyebrowLeft_changed  = true; writeMINPulsesToDisplay( 9, servoLimits[LBL_SRV_MIN_eyebrowLeft ]); Serial.print("servoLimits[LBL_SRV_MIN_eyebrowLeft ] = "+ String(servoLimits[LBL_SRV_MIN_eyebrowLeft ])+" ");}
      break;
    case 5: 
        servoLimits[LBL_SRV_MIN_cheekRight       ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s10, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_cheekLeft        ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s11, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_cheekRight] != prevServoLimits[LBL_SRV_MIN_cheekRight]){data_changed = true; SRV_MIN_cheekRight_changed = true; writeMINPulsesToDisplay( 10, servoLimits[LBL_SRV_MIN_cheekRight]); Serial.print("servoLimits[LBL_SRV_MIN_cheekRight] = "+ String(servoLimits[LBL_SRV_MIN_cheekRight])+" ");}
        if(servoLimits[LBL_SRV_MIN_cheekLeft ] != prevServoLimits[LBL_SRV_MIN_cheekLeft ]){data_changed = true; SRV_MIN_cheekLeft_changed  = true; writeMINPulsesToDisplay( 11, servoLimits[LBL_SRV_MIN_cheekLeft ]); Serial.print("servoLimits[LBL_SRV_MIN_cheekLeft ] = "+ String(servoLimits[LBL_SRV_MIN_cheekLeft ])+" ");}
      break;
    case 6: 
        servoLimits[LBL_SRV_MIN_upperLip         ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s12, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_forheadRight     ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s13, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_upperLip    ] != prevServoLimits[LBL_SRV_MIN_upperLip    ]){data_changed = true; SRV_MIN_upperLip_changed     = true; writeMINPulsesToDisplay( 12, servoLimits[LBL_SRV_MIN_upperLip    ]); Serial.print("servoLimits[LBL_SRV_MIN_upperLip    ] = "+ String(servoLimits[LBL_SRV_MIN_upperLip    ])+" ");}
        if(servoLimits[LBL_SRV_MIN_forheadRight] != prevServoLimits[LBL_SRV_MIN_forheadRight]){data_changed = true; SRV_MIN_forheadRight_changed = true; writeMINPulsesToDisplay( 13, servoLimits[LBL_SRV_MIN_forheadRight]); Serial.print("servoLimits[LBL_SRV_MIN_forheadRight] = "+ String(servoLimits[LBL_SRV_MIN_forheadRight])+" ");}
      break;
    case 7: 
        servoLimits[LBL_SRV_MIN_forheadLeft      ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s14, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MIN_Jaw_UpDown       ] = map(constrain(mydata_received.s2min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s15, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MIN_forheadLeft] != prevServoLimits[LBL_SRV_MIN_forheadLeft]){data_changed = true; SRV_MIN_forheadLeft_changed = true; writeMINPulsesToDisplay( 14, servoLimits[LBL_SRV_MIN_forheadLeft]); Serial.print("servoLimits[LBL_SRV_MIN_forheadLeft] = "+ String(servoLimits[LBL_SRV_MIN_forheadLeft])+" ");}
        if(servoLimits[LBL_SRV_MIN_Jaw_UpDown ] != prevServoLimits[LBL_SRV_MIN_Jaw_UpDown ]){data_changed = true; SRV_MIN_Jaw_UpDown_changed  = true; writeMINPulsesToDisplay( 15, servoLimits[LBL_SRV_MIN_Jaw_UpDown ]); Serial.print("servoLimits[LBL_SRV_MIN_Jaw_UpDown ] = "+ String(servoLimits[LBL_SRV_MIN_Jaw_UpDown ])+" ");}
      break;
  }

  switch (mydata_received.servoSet) {
    case 0: 
        servoLimits[LBL_SRV_MAX_eyeLeftUD        ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x00, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyeLeftLR        ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x01, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_eyeLeftUD] != prevServoLimits[LBL_SRV_MAX_eyeLeftUD]){data_changed = true; SRV_MAX_eyeLeftUD_changed = true; writeMINPulsesToDisplay( 0, servoLimits[LBL_SRV_MAX_eyeLeftUD]); Serial.print("servoLimits[LBL_SRV_MAX_eyeLeftUD] = " + String(servoLimits[LBL_SRV_MAX_eyeLeftUD])+" ");}
        if(servoLimits[LBL_SRV_MAX_eyeLeftLR] != prevServoLimits[LBL_SRV_MAX_eyeLeftLR]){data_changed = true; SRV_MAX_eyeLeftLR_changed = true; writeMINPulsesToDisplay( 1, servoLimits[LBL_SRV_MAX_eyeLeftLR]); Serial.print("servoLimits[LBL_SRV_MAX_eyeLeftLR] = " + String(servoLimits[LBL_SRV_MAX_eyeLeftLR])+" ");}
      break;
    case 1: 
        servoLimits[LBL_SRV_MAX_eyeRightUD       ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x02, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyeRightLR       ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x03, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_eyeRightUD] != prevServoLimits[LBL_SRV_MAX_eyeRightUD]){data_changed = true; SRV_MAX_eyeRightUD_changed = true; writeMINPulsesToDisplay( 2, servoLimits[LBL_SRV_MAX_eyeRightUD]); Serial.print("servoLimits[LBL_SRV_MAX_eyeRightUD] = "+ String(servoLimits[LBL_SRV_MAX_eyeRightUD])+" ");}
        if(servoLimits[LBL_SRV_MAX_eyeRightLR] != prevServoLimits[LBL_SRV_MAX_eyeRightLR]){data_changed = true; SRV_MAX_eyeRightLR_changed = true; writeMINPulsesToDisplay( 3, servoLimits[LBL_SRV_MAX_eyeRightLR]); Serial.print("servoLimits[LBL_SRV_MAX_eyeRightLR] = "+ String(servoLimits[LBL_SRV_MAX_eyeRightLR])+" ");}
      break;
    case 2: 
        servoLimits[LBL_SRV_MAX_eyelidLeftUpper  ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x04, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyelidLeftLower  ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x05, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_eyelidLeftUpper] != prevServoLimits[LBL_SRV_MAX_eyelidLeftUpper]){data_changed = true; SRV_MAX_eyelidLeftUpper_changed = true; writeMINPulsesToDisplay( 4, servoLimits[LBL_SRV_MAX_eyelidLeftUpper]); Serial.print("servoLimits[LBL_SRV_MAX_eyelidLeftUpper] = "+ String(servoLimits[LBL_SRV_MAX_eyelidLeftUpper])+" ");}
        if(servoLimits[LBL_SRV_MAX_eyelidLeftLower] != prevServoLimits[LBL_SRV_MAX_eyelidLeftLower]){data_changed = true; SRV_MAX_eyelidLeftLower_changed = true; writeMINPulsesToDisplay( 5, servoLimits[LBL_SRV_MAX_eyelidLeftLower]); Serial.print("servoLimits[LBL_SRV_MAX_eyelidLeftLower] = "+ String(servoLimits[LBL_SRV_MAX_eyelidLeftLower])+" ");}
      break;
    case 3: 
        servoLimits[LBL_SRV_MAX_eyelidRightUpper ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x06, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyelidRightLower ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x07, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_eyelidRightUpper] != prevServoLimits[LBL_SRV_MAX_eyelidRightUpper]){data_changed = true; SRV_MAX_eyelidRightUpper_changed = true; writeMINPulsesToDisplay( 6, servoLimits[LBL_SRV_MAX_eyelidRightUpper]); Serial.print("servoLimits[LBL_SRV_MAX_eyelidRightUpper] = "+ String(servoLimits[LBL_SRV_MAX_eyelidRightUpper])+" ");}
        if(servoLimits[LBL_SRV_MAX_eyelidRightLower] != prevServoLimits[LBL_SRV_MAX_eyelidRightLower]){data_changed = true; SRV_MAX_eyelidRightLower_changed = true; writeMINPulsesToDisplay( 7, servoLimits[LBL_SRV_MAX_eyelidRightLower]); Serial.print("servoLimits[LBL_SRV_MAX_eyelidRightLower] = "+ String(servoLimits[LBL_SRV_MAX_eyelidRightLower])+" ");}
      break;
    case 4: 
        servoLimits[LBL_SRV_MAX_eyebrowRight     ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x08, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_eyebrowLeft      ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x09, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_eyebrowRight] != prevServoLimits[LBL_SRV_MAX_eyebrowRight]){data_changed = true; SRV_MAX_eyebrowRight_changed = true; writeMINPulsesToDisplay( 8, servoLimits[LBL_SRV_MAX_eyebrowRight]); Serial.print("servoLimits[LBL_SRV_MAX_eyebrowRight] = "+ String(servoLimits[LBL_SRV_MAX_eyebrowRight])+" ");}
        if(servoLimits[LBL_SRV_MAX_eyebrowLeft ] != prevServoLimits[LBL_SRV_MAX_eyebrowLeft ]){data_changed = true; SRV_MAX_eyebrowLeft_changed  = true; writeMINPulsesToDisplay( 9, servoLimits[LBL_SRV_MAX_eyebrowLeft ]); Serial.print("servoLimits[LBL_SRV_MAX_eyebrowLeft ] = "+ String(servoLimits[LBL_SRV_MAX_eyebrowLeft ])+" ");}
      break;
    case 5: 
        servoLimits[LBL_SRV_MAX_cheekRight       ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x10, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_cheekLeft        ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x11, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_cheekRight] != prevServoLimits[LBL_SRV_MAX_cheekRight]){data_changed = true; SRV_MAX_cheekRight_changed = true; writeMINPulsesToDisplay( 10, servoLimits[LBL_SRV_MAX_cheekRight]); Serial.print("servoLimits[LBL_SRV_MAX_cheekRight] = "+ String(servoLimits[LBL_SRV_MAX_cheekRight])+" ");}
        if(servoLimits[LBL_SRV_MAX_cheekLeft ] != prevServoLimits[LBL_SRV_MAX_cheekLeft ]){data_changed = true; SRV_MAX_cheekLeft_changed  = true; writeMINPulsesToDisplay( 11, servoLimits[LBL_SRV_MAX_cheekLeft ]); Serial.print("servoLimits[LBL_SRV_MAX_cheekLeft ] = "+ String(servoLimits[LBL_SRV_MAX_cheekLeft ])+" ");}
      break;
    case 6: 
        servoLimits[LBL_SRV_MAX_upperLip         ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x12, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_forheadRight     ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x13, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_upperLip    ] != prevServoLimits[LBL_SRV_MAX_upperLip    ]){data_changed = true; SRV_MAX_upperLip_changed     = true; writeMINPulsesToDisplay( 12, servoLimits[LBL_SRV_MAX_upperLip    ]); Serial.print("servoLimits[LBL_SRV_MAX_upperLip    ] = "+ String(servoLimits[LBL_SRV_MAX_upperLip    ])+" ");}
        if(servoLimits[LBL_SRV_MAX_forheadRight] != prevServoLimits[LBL_SRV_MAX_forheadRight]){data_changed = true; SRV_MAX_forheadRight_changed = true; writeMINPulsesToDisplay( 13, servoLimits[LBL_SRV_MAX_forheadRight]); Serial.print("servoLimits[LBL_SRV_MAX_forheadRight] = "+ String(servoLimits[LBL_SRV_MAX_forheadRight])+" ");}
      break;
    case 7: 
        servoLimits[LBL_SRV_MAX_forheadLeft      ] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x14, 0, 255), 0, 255, 0, 1023);
        servoLimits[LBL_SRV_MAX_Jaw_UpDown       ] = map(constrain(mydata_received.s2max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x15, 0, 255), 0, 255, 0, 1023);
        if(servoLimits[LBL_SRV_MAX_forheadLeft] != prevServoLimits[LBL_SRV_MAX_forheadLeft]){data_changed = true; SRV_MAX_forheadLeft_changed = true; writeMINPulsesToDisplay( 14, servoLimits[LBL_SRV_MAX_forheadLeft]); Serial.print("servoLimits[LBL_SRV_MAX_forheadLeft] = "+ String(servoLimits[LBL_SRV_MAX_forheadLeft])+" ");}
        if(servoLimits[LBL_SRV_MAX_Jaw_UpDown ] != prevServoLimits[LBL_SRV_MAX_Jaw_UpDown ]){data_changed = true; SRV_MAX_Jaw_UpDown_changed  = true; writeMINPulsesToDisplay( 15, servoLimits[LBL_SRV_MAX_Jaw_UpDown ]); Serial.print("servoLimits[LBL_SRV_MAX_Jaw_UpDown ] = "+ String(servoLimits[LBL_SRV_MAX_Jaw_UpDown ])+" ");}
      break;
    }

  switch (mydata_received.servoSet) {
    case 0: 
        servoLimits[LBL_SRV_MID_eyeLeftUD        ] =  round((servoLimits[LBL_SRV_MAX_eyeLeftUD         ] + servoLimits[LBL_SRV_MIN_eyeLeftUD       ])/2);
        servoLimits[LBL_SRV_MID_eyeLeftLR        ] =  round((servoLimits[LBL_SRV_MAX_eyeLeftLR         ] + servoLimits[LBL_SRV_MIN_eyeLeftLR       ])/2);
      break;
    case 1: 
        servoLimits[LBL_SRV_MID_eyeRightUD       ] =  round((servoLimits[LBL_SRV_MAX_eyeRightUD        ] + servoLimits[LBL_SRV_MIN_eyeRightUD      ])/2);
        servoLimits[LBL_SRV_MID_eyeRightLR       ] =  round((servoLimits[LBL_SRV_MAX_eyeRightLR        ] + servoLimits[LBL_SRV_MIN_eyeRightLR      ])/2);
      break;
    case 2: 
        servoLimits[LBL_SRV_MID_eyelidLeftUpper  ] =  round((servoLimits[LBL_SRV_MAX_eyelidLeftUpper   ] + servoLimits[LBL_SRV_MIN_eyelidLeftUpper ])/2);
        servoLimits[LBL_SRV_MID_eyelidLeftLower  ] =  round((servoLimits[LBL_SRV_MAX_eyelidLeftLower   ] + servoLimits[LBL_SRV_MIN_eyelidLeftLower ])/2);
      break;
    case 3: 
        servoLimits[LBL_SRV_MID_eyelidRightUpper ] =  round((servoLimits[LBL_SRV_MAX_eyelidRightUpper  ] + servoLimits[LBL_SRV_MIN_eyelidRightUpper])/2);
        servoLimits[LBL_SRV_MID_eyelidRightLower ] =  round((servoLimits[LBL_SRV_MAX_eyelidRightLower  ] + servoLimits[LBL_SRV_MIN_eyelidRightLower])/2);
      break;
    case 4: 
        servoLimits[LBL_SRV_MID_eyebrowRight     ] =  round((servoLimits[LBL_SRV_MAX_eyebrowRight      ] + servoLimits[LBL_SRV_MIN_eyebrowRight    ])/2);
        servoLimits[LBL_SRV_MID_eyebrowLeft      ] =  round((servoLimits[LBL_SRV_MAX_eyebrowLeft       ] + servoLimits[LBL_SRV_MIN_eyebrowLeft     ])/2);
      break;
    case 5: 
        servoLimits[LBL_SRV_MID_cheekRight       ] =  round((servoLimits[LBL_SRV_MAX_cheekRight        ] + servoLimits[LBL_SRV_MIN_cheekRight      ])/2);
        servoLimits[LBL_SRV_MID_cheekLeft        ] =  round((servoLimits[LBL_SRV_MAX_cheekLeft         ] + servoLimits[LBL_SRV_MIN_cheekLeft       ])/2);
      break;
    case 6: 
        servoLimits[LBL_SRV_MID_upperLip         ] =  round((servoLimits[LBL_SRV_MAX_upperLip          ] + servoLimits[LBL_SRV_MIN_upperLip        ])/2);
        servoLimits[LBL_SRV_MID_forheadRight     ] =  round((servoLimits[LBL_SRV_MAX_forheadRight      ] + servoLimits[LBL_SRV_MIN_forheadRight    ])/2);
      break;
    case 7: 
        servoLimits[LBL_SRV_MID_forheadLeft      ] =  round((servoLimits[LBL_SRV_MAX_forheadLeft       ] + servoLimits[LBL_SRV_MIN_forheadLeft     ])/2);
        servoLimits[LBL_SRV_MID_Jaw_UpDown       ] =  round((servoLimits[LBL_SRV_MAX_Jaw_UpDown        ] + servoLimits[LBL_SRV_MIN_Jaw_UpDown      ])/2);
      break;
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

void writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MIN){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, SERVO_MIN);  
  /*
  uint8_t modulo = chanelNum % LEFT_ARROW_STEP;
  uint8_t div_result =chanelNum / LEFT_ARROW_STEP;
  //Serial.print("writePulsesToDisplay:");// div_result = "+String(div_result)+", modulo = "+String(modulo));
  uint8_t yPos = 2 + (div_result * ((LEFT_ARROW_STEP*8)+4)) + (modulo*8);
  Serial.println("writePulsesToDisplay: yPos="+String(yPos)+", ["+String(chanelNum)+"]->"+String(servo_Pwm));
  tft->fillRect((((4) * 8)-2), yPos, 20, 8, BLACK);
  char numRead[4];
  dtostrf(SERVO_MIN, 4, 0, numRead);
  tft->drawString(((3) * 8), yPos, numRead, YELLOW);
  ///tft->fillRect((((4 + 4) * 8)-2), yPos, 20, 8, BLACK);
  //char numRead2[4];
  //dtostrf( (servo_Pwm) , 4, 0, numRead2);
  ////dtostrf(chanelNum, 4, 0, numRead2);
  //tft->drawString(((3 + 4) * 8), yPos, numRead2, YELLOW);
  //tft->fillRect((((4 + 8) * 8)-2), yPos, 20, 8, BLACK);
  //char numRead3[4];
  //dtostrf(SERVO_MAX, 4, 0, numRead3);
  //tft->drawString(((3 + 8) * 8), yPos, numRead3, YELLOW);
  //Serial.print(" loop_writePulsesToDisplay: yPos:"+String(yPos)+" , inChar:"+String(inChar)+". ");
  //Serial.println("RandomEyesMovement::writePulsesToDisplay End.");
  */
}
void writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm);
  /*
  uint8_t modulo = chanelNum % LEFT_ARROW_STEP;
  uint8_t div_result =chanelNum / LEFT_ARROW_STEP;
  //Serial.print("writePulsesToDisplay:");// div_result = "+String(div_result)+", modulo = "+String(modulo));
  uint8_t yPos = 2 + (div_result * ((LEFT_ARROW_STEP*8)+4)) + (modulo*8);
  Serial.println("writePulsesToDisplay: yPos="+String(yPos)+", ["+String(chanelNum)+"]->"+String(servo_Pwm));
  //tft->fillRect((((4) * 8)-2), yPos, 20, 8, BLACK);
  //char numRead[4];
  //dtostrf(SERVO_MIN, 4, 0, numRead);
  //tft->drawString(((3) * 8), yPos, numRead, YELLOW);
  tft->fillRect((((4 + 4) * 8)-2), yPos, 20, 8, BLACK);
  char numRead2[4];
  dtostrf( (servo_Pwm) , 4, 0, numRead2);
  //dtostrf(chanelNum, 4, 0, numRead2);
  tft->drawString(((3 + 4) * 8), yPos, numRead2, YELLOW);
  //tft->fillRect((((4 + 8) * 8)-2), yPos, 20, 8, BLACK);
  //char numRead3[4];
  //dtostrf(SERVO_MAX, 4, 0, numRead3);
  //tft->drawString(((3 + 8) * 8), yPos, numRead3, YELLOW);
  //Serial.print(" loop_writePulsesToDisplay: yPos:"+String(yPos)+" , inChar:"+String(inChar)+". ");
  //Serial.println("RandomEyesMovement::writePulsesToDisplay End.");
  */
}

void writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t SERVO_MAX){
  writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, SERVO_MAX);
}
  
void writeOneFieldToDisplay (uint8_t chanelNum,uint8_t form_label_Min_Mid_Max, uint16_t servo_Pwm){
  uint8_t modulo = chanelNum % LEFT_ARROW_STEP;
  uint8_t div_result =chanelNum / LEFT_ARROW_STEP;
  //Serial.print("writePulsesToDisplay:");// div_result = "+String(div_result)+", modulo = "+String(modulo));
  uint8_t yPos = 2 + (div_result * ((LEFT_ARROW_STEP*8)+4)) + (modulo*8);
  Serial.println("writeOneFieldToDisplay: yPos="+String(yPos)+", ["+String(chanelNum)+","+String(form_label_Min_Mid_Max)+"]->"+String(servo_Pwm));

  if(form_label_Min_Mid_Max == LABEL_FORM_MIN) 
  {
    tft.fillRect((((4) * 8)-2), yPos, 20, 8, BLACK);
    char numRead[4];
    dtostrf(servo_Pwm, 4, 0, numRead);
    tft.drawString(((3) * 8), yPos, numRead, YELLOW);
  } 
  else if (form_label_Min_Mid_Max == LABEL_FORM_MID) 
  {
    tft.fillRect((((4 + 4) * 8)-2), yPos, 20, 8, BLACK);
    char numRead2[4];
    dtostrf( (servo_Pwm) , 4, 0, numRead2);
    tft.drawString(((3 + 4) * 8), yPos, numRead2, YELLOW);
  } 
  else if (form_label_Min_Mid_Max == LABEL_FORM_MAX) 
  {
    tft.fillRect((((4 + 8) * 8)-2), yPos, 20, 8, BLACK);
    char numRead3[4];
    dtostrf(servo_Pwm, 4, 0, numRead3);
    tft.drawString(((3 + 8) * 8), yPos, numRead3, YELLOW);
  }
}

