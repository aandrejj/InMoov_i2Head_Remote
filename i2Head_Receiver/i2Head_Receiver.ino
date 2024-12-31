/* i2Head_Receiver  RF NANO Prímacovy kod 

*/

#define USE_RF_REMOTE

//#define SERVOPULSE_CONVERSION_NEEDED
//#define  SEND_FROM_0_TO_1023
#define  SEND_FROM_0_TO_255

#include "version_num.h"
#include "build_defs.h"

  #ifndef ST7735_h
    #define ST7735_h
    #include <ST7735.h>
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

#include "WritePulsesToDisplay.h"

#include "TxRx_dataStructures.h"

#include "ServoMinMidMaxValues.h"

#include <EEPROM.h>

#define RANDOM_EYES_MOVEMENT

#ifdef RANDOM_EYES_MOVEMENT
  #include "RandomEyesMovement.h"
#endif

#include "completeVersion.h"

#define HIGHSPEED 

#ifdef HIGHSPEED
  #define Baud 19200   // Serial monitor
#else
  #define Baud 9600    // Serial monitor
#endif

uint8_t spacing = 8;
uint8_t yPos = 2;
uint8_t servoNum = 0;

char servo[]="S";//"Servo ";
char colon[]=":";//": ";


int16_t mode;
int count;
int noDataCount = 0;

WritePulsesToDisplay writePulsesToDisplay;

#ifdef RANDOM_EYES_MOVEMENT
  RandomEyesMovement randomEyesMovement;
#endif

const uint64_t pipeIn = 0x0022;
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

RF24 radio(10,9);  //RF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed = RF24_SPI_SPEED);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ServoMinMidMaxValues servoMinMidMaxValues;

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

//RX_SERIAL_DATA_STRUCTURE prev_my_serial_data;

SERVO_EEPROM_CONFIGURATION servoEepromConfig;

byte previousServoSet;
byte previousFireBtn1;
byte previousSwitchPos;

bool RF_data_changed = false;
bool serial_data_changed = false;
bool RF_Serial_data_changed = false;
bool remoteDataReceived = false;
bool serialDataReceived = false;

String i_str ="";

//-------------------------------setup------------------------------------
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

  Serial.println("setup: @1 getconfiguration  started");
  //servoEepromConfig = getConfiguation();
  getConfiguation();
  Serial.println("setup: @1 getconfiguration End.Ok");

  Serial.println("setup: @2 ShowConfiguration started");
  //ShowConfiguration(servoEepromConfig);
  ShowConfiguration();
  Serial.println("setup: @2 ShowConfiguration End.Ok");



  writePulsesToDisplay.begin();//&tft);//, servoMinMidMaxValues.servoLimits);
  previousServoSet  = 0;
  previousFireBtn1  = 1;
  previousSwitchPos = 1;

  prepareServoForm();

  Serial.println("setup: @3 Servo Initialization started");
  pwm.begin(); //pwm.begin(0);   0 = driver_ID
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	Serial.println("setup: @4 Servos on PCA9685  attached");
  delay(200);


  #ifdef RANDOM_EYES_MOVEMENT
    uint8_t _left_arrow_step = LEFT_ARROW_STEP;
    randomEyesMovement.begin(&pwm, &writePulsesToDisplay, &servoMinMidMaxValues);
  #endif

  resetData();
  //config for NRF24 
  #ifdef USE_RF_REMOTE
  	Serial.println("setup: @5 radio.begin()..");
    radio.begin();
    //radio.setAutoAck(false);
    radio.setAutoAck(true);
    
    //Serial.println("setup: radio.setDataRate(RF24_250KBPS)");
    //radio.setDataRate(RF24_250KBPS);
    Serial.println("setup: @5radio.openReadingPipe(1,pipeIn)");
    radio.openReadingPipe(1,pipeIn);
    
    Serial.println("setup: @5radio.setPALevel(RF24_PA_MIN)");
    radio.setPALevel(RF24_PA_MIN);
   
    radio.startListening();
    Serial.println("setup: @6 rf-radio started");
  #endif

  //initMinMidMax_values();

  delay(600);
  Serial.println("setup: @7 done. setup END.");
}

//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
//-------------------------loop------------------------------------------------
void loop()
{
  remoteDataReceived = false;
  unsigned long currentMillis = millis();
  
  #ifdef USE_RF_REMOTE
  // -------------------  data from RF--------------------------------------------------
  if (currentMillis - previousMillis >= interval) 
  {  // start timed event for read and send
    previousMillis = currentMillis;
    //----------------------------------------recvData-------------------------------------
    remoteDataReceived = recvData();
    if(remoteDataReceived == true) {
      previousSafetyMillis = currentMillis; 
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
          constrain_allServoAngles_0_255();
          reset_SerialDataChanged();
        }
      }else if (mydata_received.devType == 2) {
        RF_Serial_data_changed = RfSerial_Data_changed();
          if(RF_Serial_data_changed == true) {
            String helpText = "="+String(previousServoSet)+" "+String(previousFireBtn1)+" "+ String(previousSwitchPos)+".";

            //compute_from_SerialData_toMinMax();
            #ifdef RANDOM_EYES_MOVEMENT
            if(mydata_received.fireBtn1 == 0) {

              Serial.println("loop: fireBtn pressed");

              Serial.println("writeConfiguation started.");
              writeConfiguation();
              Serial.println("writeConfiguation End.Ok.");
              
              //Serial.println("getconfiguration  started");
              //getConfiguation();
              //Serial.println("getconfiguration End.Ok");

              //Serial.println("ShowConfiguration started");
              //ShowConfiguration();
              //Serial.println("ShowConfiguration End.Ok");
              
              println_AllServoLimits_Values();

            }
            #endif
          } else {
            if(mydata_received.switchPos == 2) {
              //Serial.println("loop: switchPos ==2 -> starting randomEyesMovement.moveEyesRandomly...");
              randomEyesMovement.moveEyesRandomly(currentMillis,"With RF");
            }
          } 
          //reset_RfSerialData();
      }
    } else  if(currentMillis - previousSafetyMillis > 1000) {         // safeties
      #ifdef RANDOM_EYES_MOVEMENT
        randomEyesMovement.moveEyesRandomly(currentMillis,"No RF");
      #endif
    }
    //Serial.println(" @1.2 end");
  }
  #endif
  // -----------  end of RF-data---------------------------------------------------------------

  if((serialDataReceived==true) || (remoteDataReceived == true) || (RF_Serial_data_changed == true)) {
    //Serial.println("@3.2 DataReceived changed, serialDataReceived="+String(serialDataReceived)+", remoteDataReceived = "+String(remoteDataReceived));
    #ifdef SERVOPULSE_CONVERSION_NEEDED
      constrain_allServoAngles_0_255();
      convert_allAngle_to_Pwm_Min_Center_Max();
    #else
      convert_constrain_allServoAngles_0_1023();
    #endif

    if((RF_Serial_data_changed == true)) {
      copy_RF_Data_toPwmData();
    } 

    sendData_toPwmDriver();

    if(RF_data_changed == true || serial_data_changed == true) {
      //show_ChangedData_toDebug();
      show_PwmData_toDebug();
      //show_eyeLidsData_PwmAndAngle_toDebug();
    }
    copyActualPwmData_toPreviousPwmData();
  }
}// end of  loop()
//------------------------------end of  loop()----------------------------------
//------------------------------end of  loop()----------------------------------
//------------------------------end of  loop()----------------------------------

void writeCounterToEeprom()
{
  for (int i = 0; i < 48; i++)
    EEPROM.write(i, i);
}

void getConfiguation()
{
  int eeAddress = 0; 
  EEPROM.get( eeAddress, servoEepromConfig );
  for (int i = 0; i < 16; i++){
    servoMinMidMaxValues.servoLimits[i]    = servoEepromConfig.servoMinMidMax[i];
    servoMinMidMaxValues.servoLimits[i+16] = servoEepromConfig.servoMinMidMax[i+16];
    servoMinMidMaxValues.servoLimits[i+32] = servoEepromConfig.servoMinMidMax[i+32];
  }
}


void ShowConfiguration(){
  Serial.println("ShowConfiguration: ");
  for (int i = 0; i < 48; i++){
      //Serial.println("ServoEepromConfig.servoMinMidMax["+String(i)+"]" + String(tmpServoEepromConfig.servoMinMidMax[i])+"]");
      Serial.println("ServoEepromConfig.servoMinMidMax["+String(i)+"]=" + String(servoEepromConfig.servoMinMidMax[i])+".");
  }
}

/*
void createDemoConfiguration(){
  for (int i = 0; i < 48; i++){
    servoEepromConfig.servoMinMidMax[i]=i;
  }
}
*/

void writeConfiguation()
{
  for (int i = 0; i < 16; i++){
    servoEepromConfig.servoMinMidMax[i]=servoMinMidMaxValues.servoLimits[i];
    servoEepromConfig.servoMinMidMax[i+16]=servoMinMidMaxValues.servoLimits[i+16];
    servoEepromConfig.servoMinMidMax[i+32]=servoMinMidMaxValues.servoLimits[i+32];
  }

  int eeAddress = 0;   //Location we want the data to be put.

  EEPROM.put(eeAddress, servoEepromConfig);
  Serial.print("Written custom data type! \n\nView the example sketch eeprom_get to see how you can retrieve the values!");

}


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
      writePulsesToDisplay.drawString(0, yPos, combined, WHITE);
      //writePulsesToDisplay.drawString((((strlen(servo) + 1)) * 8), yPos, colon, WHITE);
      
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
        writePulsesToDisplay.writeMINPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoMinMidMaxValues.servoLimits[servoNum]);//, true);
        writePulsesToDisplay.writeMIDPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoMinMidMaxValues.servoLimits[servoNum+16]);//, true);
        writePulsesToDisplay.writeCurrPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoMinMidMaxValues.servoLimits[servoNum+48], true);
        writePulsesToDisplay.writeMAXPulsesToDisplay((count*LEFT_ARROW_STEP)+i, servoMinMidMaxValues.servoLimits[servoNum+32]);//, true);
        i_str = String(i);
        servoNum ++;
        yPos += spacing;    
      }
    yPos += (2*LEFT_ARROW_STEP); //8;
  }
  Serial.println("prepareServoForm: 2.for {for{}} done");
  writePulsesToDisplay.writeArrow_activeServoSet (0);
  //tft.drawString((128-(LEFT_ARROW_SIZE*8)), 3, "<", WHITE, LEFT_ARROW_SIZE);
}

void resetData()
{
  mydata_received.s1min = 0;
  mydata_received.s1mid = 127;
  mydata_received.s1curr= 127;
  mydata_received.s1max = 255;

  mydata_received.devType =  0; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)

  mydata_received.switchPos = 0;
  mydata_received.fireBtn1 = 0;
}


#ifdef USE_RF_REMOTE
  bool recvData()
  {
    bool dataReceived = false;
    while ( radio.available() ) {
      radio.read(&mydata_received, sizeof(RX_DATA_STRUCTURE));
      dataReceived = true;
    }
    return dataReceived;
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


void compute_from_SerialData_toMinMax()
{
  servoMinMidMaxValues.servoLimits[ mydata_received.servoSet           ] = map(constrain(mydata_received.s1min, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.s1curr, 0, 255), 0, 255, mydata_received.s1min, mydata_received.s1max);
  servoMinMidMaxValues.servoLimits[(mydata_received.servoSet + 16 )    ] = map(constrain(mydata_received.s1mid, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x00, 0, 255), 0, 255, 0, 1023);
  servoMinMidMaxValues.servoLimits[(mydata_received.servoSet + 16 + 16)] = map(constrain(mydata_received.s1max, 0, 255), 0, 255, 0, 1023);//map(constrain(mydata_received.x00, 0, 255), 0, 255, 0, 1023);
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

void send_RF_Data_toPwmDriver() {
  pwm.setPWM( i01_head_eyeLeftUD       , 0, servoMinMidMaxValues.servoLimits[i01_head_eyeLeftUD       +48]);
  pwm.setPWM( i01_head_eyeLeftLR       , 0, servoMinMidMaxValues.servoLimits[i01_head_eyeLeftLR       +48]);
  pwm.setPWM( i01_head_eyeRightUD      , 0, servoMinMidMaxValues.servoLimits[i01_head_eyeRightUD      +48]);
  pwm.setPWM( i01_head_eyeRightLR      , 0, servoMinMidMaxValues.servoLimits[i01_head_eyeRightLR      +48]);
  pwm.setPWM( i01_head_eyelidLeftUpper , 0, servoMinMidMaxValues.servoLimits[i01_head_eyelidLeftUpper +48]);
  pwm.setPWM( i01_head_eyelidLeftLower , 0, servoMinMidMaxValues.servoLimits[i01_head_eyelidLeftLower +48]);
  pwm.setPWM( i01_head_eyelidRightUpper, 0, servoMinMidMaxValues.servoLimits[i01_head_eyelidRightUpper+48]);
  pwm.setPWM( i01_head_eyelidRightLower, 0, servoMinMidMaxValues.servoLimits[i01_head_eyelidRightLower+48]);
  pwm.setPWM( i01_head_eyebrowRight    , 0, servoMinMidMaxValues.servoLimits[i01_head_eyebrowRight    +48]);
  pwm.setPWM( i01_head_eyebrowLeft     , 0, servoMinMidMaxValues.servoLimits[i01_head_eyebrowLeft     +48]);
  pwm.setPWM( i01_head_cheekRight      , 0, servoMinMidMaxValues.servoLimits[i01_head_cheekRight      +48]);
  pwm.setPWM( i01_head_cheekLeft       , 0, servoMinMidMaxValues.servoLimits[i01_head_cheekLeft       +48]);
  pwm.setPWM( i01_head_upperLip        , 0, servoMinMidMaxValues.servoLimits[i01_head_upperLip        +48]);
  pwm.setPWM( i01_head_forheadRight    , 0, servoMinMidMaxValues.servoLimits[i01_head_forheadRight    +48]);
  pwm.setPWM( i01_head_forheadLeft     , 0, servoMinMidMaxValues.servoLimits[i01_head_forheadLeft     +48]);
  pwm.setPWM( Jaw_UpDown               , 0, servoMinMidMaxValues.servoLimits[Jaw_UpDown               +48]);
}

void copy_RF_Data_toPwmData() {
  servo_eyeLeftUD_Pwm       = servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftUD       +48)];
  servo_eyeLeftLR_Pwm       = servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftLR       +48)];
  servo_eyeRightUD_Pwm      = servoMinMidMaxValues.servoLimits[(i01_head_eyeRightUD      +48)];
  servo_eyeRightLR_Pwm      = servoMinMidMaxValues.servoLimits[(i01_head_eyeRightLR      +48)];
  servo_eyelidLeftUpper_Pwm = servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftUpper +48)];
  servo_eyelidLeftLower_Pwm = servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftLower +48)];
  servo_eyelidRightUpper_Pwm= servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightUpper+48)];
  servo_eyelidRightLower_Pwm= servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightLower+48)];
  servo_eyebrowRight_Pwm    = servoMinMidMaxValues.servoLimits[(i01_head_eyebrowRight    +48)];
  servo_eyebrowLeft_Pwm     = servoMinMidMaxValues.servoLimits[(i01_head_eyebrowLeft     +48)];
  servo_cheekRight_Pwm      = servoMinMidMaxValues.servoLimits[(i01_head_cheekRight      +48)];
  servo_cheekLeft_Pwm       = servoMinMidMaxValues.servoLimits[(i01_head_cheekLeft       +48)];
  servo_upperLip_Pwm        = servoMinMidMaxValues.servoLimits[(i01_head_upperLip        +48)];
  servo_forheadRight_Pwm    = servoMinMidMaxValues.servoLimits[(i01_head_forheadRight    +48)];
  servo_forheadLeft_Pwm     = servoMinMidMaxValues.servoLimits[(i01_head_forheadLeft     +48)];
  servo_Jaw_UpDown_Pwm      = servoMinMidMaxValues.servoLimits[(Jaw_UpDown               +48)];
}

void show_PwmData_toDebug() {
  bool dataShown = false;
  if (prev_servo_eyeLeftUD_Pwm        != servo_eyeLeftUD_Pwm       ) {Serial.print("S_eyeLeftUD_Pwm:"+String(servo_eyeLeftUD_Pwm       )+", "); dataShown = true;}
  if (prev_servo_eyeLeftLR_Pwm        != servo_eyeLeftLR_Pwm       ) {Serial.print("S_eyeLeftLR_Pwm:"+String(servo_eyeLeftLR_Pwm       )+", "); dataShown = true;}
  if (prev_servo_eyeRightUD_Pwm       != servo_eyeRightUD_Pwm      ) {Serial.print("S_eyeRightUD_Pwm:"+String(servo_eyeRightUD_Pwm      )+", "); dataShown = true;}
  if (prev_servo_eyeRightLR_Pwm       != servo_eyeRightLR_Pwm      ) {Serial.print("S_eyeRightLR_Pwm:"+String(servo_eyeRightLR_Pwm      )+", "); dataShown = true;}
  if (prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) {Serial.print("S_eyelidLeftUpper_Pwm:"+String(servo_eyelidLeftUpper_Pwm )+", "); dataShown = true;}
  if (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) {Serial.print("S_eyelidLeftLower_Pwm:"+String(servo_eyelidLeftLower_Pwm )+", "); dataShown = true;}
  if (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) {Serial.print("S_eyelidRightUpper_Pwm:"+String(servo_eyelidRightUpper_Pwm)+", "); dataShown = true;}
  if (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm) {Serial.print("S_eyelidRightLower_Pwm:"+String(servo_eyelidRightLower_Pwm)+", "); dataShown = true;}
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
  if (prev_servo_eyelidLeftUpper_Pwm  != servo_eyelidLeftUpper_Pwm ) {Serial.print("servo_eyelidLeftUpper_Angle:"+String(servo_eyelidLeftUpper_Angle )+", ") ; dataShown = true;}
  if (prev_servo_eyelidLeftLower_Pwm  != servo_eyelidLeftLower_Pwm ) {Serial.print("servo_eyelidLeftLower_Angle:"+String(servo_eyelidLeftLower_Angle )+", ") ; dataShown = true;}
  if (prev_servo_eyelidRightUpper_Pwm != servo_eyelidRightUpper_Pwm) {Serial.print("servo_eyelidRightUpper_Angle:"+String(servo_eyelidRightUpper_Angle)+", "); dataShown = true;}
  if (prev_servo_eyelidRightLower_Pwm != servo_eyelidRightLower_Pwm) {Serial.print("servo_eyelidRightLower_Angle:"+String(servo_eyelidRightLower_Angle)+", "); dataShown = true;}
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
    servoMinMidMaxValues.servoLimits[servoIndex] = map(constrain(mydata_received_value, 0, 255), 0, 255, 0, 1023);
  #else
    servoMinMidMaxValues.servoLimits[servoIndex] = constrain(mydata_received_value, 0, 1023);
  #endif

  if(servoMinMidMaxValues.servoLimits[servoIndex] != servoMinMidMaxValues.prevServoLimits[servoIndex])
  {
    data_changed = true; 
    Serial.print("RfSerial_Data_Changed_innerPart: servoIndex="+String(servoIndex)+", ");
    Serial.print("displayChanelNumber="+ String(displayChanelNumber)+ ", ");
    Serial.print("mydata_received_value ="+ String(mydata_received_value)+", ");
    Serial.print("form_label_Min_Mid_Max="+String(form_label_Min_Mid_Max)+" , ");
    Serial.print("servoMinMidMaxValues.prevServoLimits["+String(servoIndex)+"] = "+ String(servoMinMidMaxValues.prevServoLimits[servoIndex])+" , ");
    Serial.print("servoMinMidMaxValues.servoLimits["+String(servoIndex)+"] = "+ String(servoMinMidMaxValues.servoLimits[servoIndex]));
    Serial.println(".");
    if(form_label_Min_Mid_Max == LABEL_FORM_MIN) {
      writePulsesToDisplay.writeMINPulsesToDisplay( displayChanelNumber, servoMinMidMaxValues.servoLimits[servoIndex]);
    }
    else if(form_label_Min_Mid_Max == LABEL_FORM_MID) {
      writePulsesToDisplay.writeMIDPulsesToDisplay( displayChanelNumber, servoMinMidMaxValues.servoLimits[servoIndex]);
    }
    else if(form_label_Min_Mid_Max == LABEL_FORM_MAX) {
      writePulsesToDisplay.writeMAXPulsesToDisplay( displayChanelNumber, servoMinMidMaxValues.servoLimits[servoIndex]);
    }
    else {
      writePulsesToDisplay.writeCurrPulsesToDisplay( displayChanelNumber, servoMinMidMaxValues.servoLimits[servoIndex]);
    }
    //writeOneFieldToDisplay(displayChanelNumber, form_label_Min_Mid_Max, servoMinMidMaxValues.servoLimits[servoIndex]); 
    servoMinMidMaxValues.prevServoLimits[servoIndex] = servoMinMidMaxValues.servoLimits[servoIndex];
  }
  return data_changed;
}

bool RfSerial_Data_changed() {
  bool data_changed = false;
    if(abs(mydata_received.fireBtn1 - previousFireBtn1)>0){
      Serial.println("RfSerial_Data_changed: mydata_received.fireBtn1 CHANGED from "+String(previousFireBtn1)+" to  "+String(mydata_received.fireBtn1)+" ");
      data_changed = true;
      previousFireBtn1 = mydata_received.fireBtn1;
    }
    if(abs(mydata_received.switchPos - previousSwitchPos)>0){
      Serial.println("RfSerial_Data_changed: mydata_received.switchPos CHANGED from "+String(previousSwitchPos)+" to  "+String(mydata_received.switchPos)+" ");
      data_changed = true;
      previousSwitchPos = mydata_received.switchPos;
    }

    if(abs(mydata_received.servoSet - previousServoSet)>0) {
      Serial.println("RfSerial_Data_changed: mydata_received.servoSet CHANGED from "+String(previousServoSet)+" to  "+String(mydata_received.servoSet)+" ");
      writePulsesToDisplay.writeArrow_activeServoSet(mydata_received.servoSet);
      data_changed = true; 
      previousServoSet = mydata_received.servoSet;
    }
    servoPositionChanged[(mydata_received.servoSet+ 0)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+ 0), mydata_received.servoSet, mydata_received.s1min,  LABEL_FORM_MIN);
    servoPositionChanged[(mydata_received.servoSet+16)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+16), mydata_received.servoSet, mydata_received.s1mid,  LABEL_FORM_MID);
    servoPositionChanged[(mydata_received.servoSet+32)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+32), mydata_received.servoSet, mydata_received.s1max,  LABEL_FORM_MAX);
    
    if(mydata_received.switchPos != 2){
      #ifdef SEND_FROM_0_TO_255
        uint16_t extrapolatedCurrentValue =
            (mydata_received.s1curr)<128 ? 
              (map(mydata_received.s1curr,   0, 127, servoMinMidMaxValues.servoLimits[(mydata_received.servoSet)           ], servoMinMidMaxValues.servoLimits[(mydata_received.servoSet) + (16 * 1)]) ) 
            : (map(mydata_received.s1curr, 128, 255, servoMinMidMaxValues.servoLimits[(mydata_received.servoSet) + (16 * 1)], servoMinMidMaxValues.servoLimits[(mydata_received.servoSet) + (16 * 2)]) );

        servoPositionChanged[(mydata_received.servoSet+48)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+48), mydata_received.servoSet, extrapolatedCurrentValue, LABEL_FORM_MAX+1);
      #else
        servoPositionChanged[(mydata_received.servoSet+48)] = RfSerial_Data_Changed_innerPart((mydata_received.servoSet+48), mydata_received.servoSet, mydata_received.s1curr, LABEL_FORM_MAX+1);
      #endif
    }

    if(servoPositionChanged[mydata_received.servoSet   ] == true ||
       servoPositionChanged[mydata_received.servoSet+16] == true ||
       servoPositionChanged[mydata_received.servoSet+32] == true ||
       servoPositionChanged[mydata_received.servoSet+48] == true ) 
    {
      data_changed = true; 
    }

  if(data_changed == true){
      //Serial.println(" RfSerial_Data_changed:@End previousServoSet = "+String(previousServoSet)+".");
      //Serial.println(" my_serial_data_received_changed ----");
    }
  return data_changed;
}

void reset_RfSerialData() {
  for (short i=0; i<=47; i++) {
    servoMinMidMaxValues.prevServoLimits[i] = servoMinMidMaxValues.servoLimits[i];
  }
}



bool serialData_changed() {
  bool data_changed = false;
  if(my_serial_data_received.s00 != prev_my_serial_data.s00){data_changed = true; s00_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 0,my_serial_data_received.s00); Serial.print("s00 = "+ String(my_serial_data_received.s00)+" ");}
  if(my_serial_data_received.s01 != prev_my_serial_data.s01){data_changed = true; s01_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 1,my_serial_data_received.s01); Serial.print("s01 = "+ String(my_serial_data_received.s01)+" ");}
  if(my_serial_data_received.s02 != prev_my_serial_data.s02){data_changed = true; s02_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 2,my_serial_data_received.s02); Serial.print("s02 = "+ String(my_serial_data_received.s02)+" ");}
  if(my_serial_data_received.s03 != prev_my_serial_data.s03){data_changed = true; s03_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 3,my_serial_data_received.s03); Serial.print("s03 = "+ String(my_serial_data_received.s03)+" ");}
  if(my_serial_data_received.s04 != prev_my_serial_data.s04){data_changed = true; s04_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 4,my_serial_data_received.s04); Serial.print("s04 = "+ String(my_serial_data_received.s04)+" ");}
  if(my_serial_data_received.s05 != prev_my_serial_data.s05){data_changed = true; s05_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 5,my_serial_data_received.s05); Serial.print("s05 = "+ String(my_serial_data_received.s05)+" ");}
  if(my_serial_data_received.s06 != prev_my_serial_data.s06){data_changed = true; s06_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 6,my_serial_data_received.s06); Serial.print("s06 = "+ String(my_serial_data_received.s06)+" ");}
  if(my_serial_data_received.s07 != prev_my_serial_data.s07){data_changed = true; s07_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 7,my_serial_data_received.s07); Serial.print("s07 = "+ String(my_serial_data_received.s07)+" ");}
  if(my_serial_data_received.s08 != prev_my_serial_data.s08){data_changed = true; s08_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 8,my_serial_data_received.s08); Serial.print("s08 = "+ String(my_serial_data_received.s08)+" ");}
  if(my_serial_data_received.s09 != prev_my_serial_data.s09){data_changed = true; s09_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay( 9,my_serial_data_received.s09); Serial.print("s09 = "+ String(my_serial_data_received.s09)+" ");}
  if(my_serial_data_received.s10 != prev_my_serial_data.s10){data_changed = true; s10_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay(10,my_serial_data_received.s10); Serial.print("s10 = "+ String(my_serial_data_received.s10)+" ");}
  if(my_serial_data_received.s11 != prev_my_serial_data.s11){data_changed = true; s11_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay(11,my_serial_data_received.s11); Serial.print("s11 = "+ String(my_serial_data_received.s11)+" ");}
  if(my_serial_data_received.s12 != prev_my_serial_data.s12){data_changed = true; s12_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay(12,my_serial_data_received.s12); Serial.print("s12 = "+ String(my_serial_data_received.s12)+" ");}
  if(my_serial_data_received.s13 != prev_my_serial_data.s13){data_changed = true; s13_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay(13,my_serial_data_received.s13); Serial.print("s13 = "+ String(my_serial_data_received.s13)+" ");}
  if(my_serial_data_received.s14 != prev_my_serial_data.s14){data_changed = true; s14_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay(14,my_serial_data_received.s14); Serial.print("s14 = "+ String(my_serial_data_received.s14)+" ");}
  if(my_serial_data_received.s15 != prev_my_serial_data.s15){data_changed = true; s15_changed = true; writePulsesToDisplay.writeMINPulsesToDisplay(15,my_serial_data_received.s15); Serial.print("s15 = "+ String(my_serial_data_received.s15)+" ");}
 
  if(my_serial_data_received.x00 != prev_my_serial_data.x00){data_changed = true; x00_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(32,my_serial_data_received.x00); Serial.print("x00 = "+ String(my_serial_data_received.x00)+" ");}
  if(my_serial_data_received.x01 != prev_my_serial_data.x01){data_changed = true; x01_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(33,my_serial_data_received.x01); Serial.print("x01 = "+ String(my_serial_data_received.x01)+" ");}
  if(my_serial_data_received.x02 != prev_my_serial_data.x02){data_changed = true; x02_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(34,my_serial_data_received.x02); Serial.print("x02 = "+ String(my_serial_data_received.x02)+" ");}
  if(my_serial_data_received.x03 != prev_my_serial_data.x03){data_changed = true; x03_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(35,my_serial_data_received.x03); Serial.print("x03 = "+ String(my_serial_data_received.x03)+" ");}
  if(my_serial_data_received.x04 != prev_my_serial_data.x04){data_changed = true; x04_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(36,my_serial_data_received.x04); Serial.print("x04 = "+ String(my_serial_data_received.x04)+" ");}
  if(my_serial_data_received.x05 != prev_my_serial_data.x05){data_changed = true; x05_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(37,my_serial_data_received.x05); Serial.print("x05 = "+ String(my_serial_data_received.x05)+" ");}
  if(my_serial_data_received.x06 != prev_my_serial_data.x06){data_changed = true; x06_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(38,my_serial_data_received.x06); Serial.print("x06 = "+ String(my_serial_data_received.x06)+" ");}
  if(my_serial_data_received.x07 != prev_my_serial_data.x07){data_changed = true; x07_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(39,my_serial_data_received.x07); Serial.print("x07 = "+ String(my_serial_data_received.x07)+" ");}
  if(my_serial_data_received.x08 != prev_my_serial_data.x08){data_changed = true; x08_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(40,my_serial_data_received.x08); Serial.print("x08 = "+ String(my_serial_data_received.x08)+" ");}
  if(my_serial_data_received.x09 != prev_my_serial_data.x09){data_changed = true; x09_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(41,my_serial_data_received.x09); Serial.print("x09 = "+ String(my_serial_data_received.x09)+" ");}
  if(my_serial_data_received.x10 != prev_my_serial_data.x10){data_changed = true; x10_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(42,my_serial_data_received.x10); Serial.print("x10 = "+ String(my_serial_data_received.x10)+" ");}
  if(my_serial_data_received.x11 != prev_my_serial_data.x11){data_changed = true; x11_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(43,my_serial_data_received.x11); Serial.print("x11 = "+ String(my_serial_data_received.x11)+" ");}
  if(my_serial_data_received.x12 != prev_my_serial_data.x12){data_changed = true; x12_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(44,my_serial_data_received.x12); Serial.print("x12 = "+ String(my_serial_data_received.x12)+" ");}
  if(my_serial_data_received.x13 != prev_my_serial_data.x13){data_changed = true; x13_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(45,my_serial_data_received.x13); Serial.print("x13 = "+ String(my_serial_data_received.x13)+" ");}
  if(my_serial_data_received.x14 != prev_my_serial_data.x14){data_changed = true; x14_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(46,my_serial_data_received.x14); Serial.print("x14 = "+ String(my_serial_data_received.x14)+" ");}
  if(my_serial_data_received.x15 != prev_my_serial_data.x15){data_changed = true; x15_changed = true; writePulsesToDisplay.writeMAXPulsesToDisplay(47,my_serial_data_received.x15); Serial.print("x15 = "+ String(my_serial_data_received.x15)+" ");}

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

void println_AllServoLimits_Values()
{
  Serial.println("-------------------------------------------------");
  Serial.println("#define SERVO_MIN_eyeLeftUD         "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftUD       + 0)])+"");
  Serial.println("#define SERVO_MID_eyeLeftUD         "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftUD       +16)])+"");
  Serial.println("#define SERVO_MAX_eyeLeftUD         "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftUD       +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyeLeftLR         "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftLR       + 0)])+"");
  Serial.println("#define SERVO_MID_eyeLeftLR         "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftLR       +16)])+"");
  Serial.println("#define SERVO_MAX_eyeLeftLR         "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeLeftLR       +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyeRightUD        "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeRightUD      + 0)])+"");
  Serial.println("#define SERVO_MID_eyeRightUD        "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeRightUD      +16)])+"");
  Serial.println("#define SERVO_MAX_eyeRightUD        "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeRightUD      +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyeRightLR        "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeRightLR      + 0)])+"");
  Serial.println("#define SERVO_MID_eyeRightLR        "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeRightLR      +16)])+"");
  Serial.println("#define SERVO_MAX_eyeRightLR        "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyeRightLR      +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyelidLeftUpper   "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftUpper + 0)])+"");
  Serial.println("#define SERVO_MID_eyelidLeftUpper   "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftUpper +16)])+"");
  Serial.println("#define SERVO_MAX_eyelidLeftUpper   "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftUpper +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyelidLeftLower   "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftLower + 0)])+"");
  Serial.println("#define SERVO_MID_eyelidLeftLower   "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftLower +16)])+"");
  Serial.println("#define SERVO_MAX_eyelidLeftLower   "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidLeftLower +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyelidRightUpper  "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightUpper+ 0)])+"");
  Serial.println("#define SERVO_MID_eyelidRightUpper  "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightUpper+16)])+"");
  Serial.println("#define SERVO_MAX_eyelidRightUpper  "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightUpper+32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyelidRightLower  "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightLower+ 0)])+"");
  Serial.println("#define SERVO_MID_eyelidRightLower  "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightLower+16)])+"");
  Serial.println("#define SERVO_MAX_eyelidRightLower  "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyelidRightLower+32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyebrowRight      "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyebrowRight    + 0)])+"");
  Serial.println("#define SERVO_MID_eyebrowRight      "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyebrowRight    +16)])+"");
  Serial.println("#define SERVO_MAX_eyebrowRight      "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyebrowRight    +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_eyebrowLeft       "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyebrowLeft     + 0)])+"");
  Serial.println("#define SERVO_MID_eyebrowLeft       "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyebrowLeft     +16)])+"");
  Serial.println("#define SERVO_MAX_eyebrowLeft       "+String(servoMinMidMaxValues.servoLimits[(i01_head_eyebrowLeft     +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_cheekRight        "+String(servoMinMidMaxValues.servoLimits[(i01_head_cheekRight      + 0)])+"");
  Serial.println("#define SERVO_MID_cheekRight        "+String(servoMinMidMaxValues.servoLimits[(i01_head_cheekRight      +16)])+"");
  Serial.println("#define SERVO_MAX_cheekRight        "+String(servoMinMidMaxValues.servoLimits[(i01_head_cheekRight      +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_cheekLeft         "+String(servoMinMidMaxValues.servoLimits[(i01_head_cheekLeft       + 0)])+"");
  Serial.println("#define SERVO_MID_cheekLeft         "+String(servoMinMidMaxValues.servoLimits[(i01_head_cheekLeft       +16)])+"");
  Serial.println("#define SERVO_MAX_cheekLeft         "+String(servoMinMidMaxValues.servoLimits[(i01_head_cheekLeft       +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_upperLip          "+String(servoMinMidMaxValues.servoLimits[(i01_head_upperLip        + 0)])+"");
  Serial.println("#define SERVO_MID_upperLip          "+String(servoMinMidMaxValues.servoLimits[(i01_head_upperLip        +16)])+"");
  Serial.println("#define SERVO_MAX_upperLip          "+String(servoMinMidMaxValues.servoLimits[(i01_head_upperLip        +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_forheadRight      "+String(servoMinMidMaxValues.servoLimits[(i01_head_forheadRight    + 0)])+"");
  Serial.println("#define SERVO_MID_forheadRight      "+String(servoMinMidMaxValues.servoLimits[(i01_head_forheadRight    +16)])+"");
  Serial.println("#define SERVO_MAX_forheadRight      "+String(servoMinMidMaxValues.servoLimits[(i01_head_forheadRight    +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_forheadLeft       "+String(servoMinMidMaxValues.servoLimits[(i01_head_forheadLeft     + 0)])+"");
  Serial.println("#define SERVO_MID_forheadLeft       "+String(servoMinMidMaxValues.servoLimits[(i01_head_forheadLeft     +16)])+"");
  Serial.println("#define SERVO_MAX_forheadLeft       "+String(servoMinMidMaxValues.servoLimits[(i01_head_forheadLeft     +32)])+"");
  Serial.println(" ");
  Serial.println("#define SERVO_MIN_Jaw_UpDown        "+String(servoMinMidMaxValues.servoLimits[(Jaw_UpDown               + 0)])+"");
  Serial.println("#define SERVO_MID_Jaw_UpDown        "+String(servoMinMidMaxValues.servoLimits[(Jaw_UpDown               +16)])+"");
  Serial.println("#define SERVO_MAX_Jaw_UpDown        "+String(servoMinMidMaxValues.servoLimits[(Jaw_UpDown               +32)])+"");
  Serial.println("-------------------------------------------------");
}