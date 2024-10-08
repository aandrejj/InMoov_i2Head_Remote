/* Kod vysielača pre RF NANO
Kod vysielača pre 4kanaly ,datove piny A0, A1, A2, A3
// https://blog.laskakit.cz/projekt-rc-arduino/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define minAcceptedMaxDifference   40

int analogReadA[8]          = { 127, 127, 127, 127, 127, 127, 127, 127};

int previousAnalogReadA[8]  = {   0,   0,   0,   0,   0,   0,   0,   0};

int previous2AnalogReadA[8] = {   0,   0,   0,   0,   0,   0,   0,   0};

int centerAnalogReadA[8]    = {   0,   0,   0,   0,   0,   0,   0,   0};

bool center_point_initialized[8] = {false, false, false, false, false, false, false, false};
bool all_center_points_initialized = false;

int minAnalogReadA[8]       = {1023,1023,1023,1023,1023,1023,1023,1023};

int maxAnalogReadA[8]       = {   0,   0,   0,   0,   0,   0,   0,   0};

byte sent_data_ch[8]        = {   0,   0,   0,   0,   0,   0,   0,   0};
byte prev_data_ch[8]        = {   0,   0,   0,   0,   0,   0,   0,   0};

bool minMaxValues_initialized[8] = {false, false, false, false, false, false, false, false};
bool all_minMaxValues_initialized = false;

byte prev_ch1 = 0;
byte prev_ch2 = 0;
byte prev_ch3 = 0;
byte prev_ch4 = 0;
byte prev_ch5 = 0;
byte prev_ch6 = 0;
byte prev_ch7 = 0;
byte prev_ch8 = 0;

const uint64_t my_radio_pipe = 0x0022; //toto istý kod musí mať aj primač
RF24 radio(10, 9);  //zapojenie CE a CSN pinov
//maximalne 32 kanalov
struct Data_to_be_sent {
  byte ch1;
  byte ch2;
  byte ch3;
  byte ch4;
  byte ch5;
  byte ch6;
  byte ch7;
  byte ch8;
 };
Data_to_be_sent sent_data;

void ReadAnalogData() 
{
  analogReadA[0] = analogRead(A0);
  analogReadA[1] = analogRead(A1);
  analogReadA[2] = analogRead(A2);
  analogReadA[3] = analogRead(A3);
  analogReadA[4] = analogRead(A4);
  analogReadA[5] = analogRead(A5);
  analogReadA[6] = analogRead(A6);
  analogReadA[7] = analogRead(A7);

  analogReadA[0] = map(analogReadA[0], 0, 1023, 1023, 0);  //Inverted
  analogReadA[1] = map(analogReadA[1], 0, 1023, 0, 1023);  
  analogReadA[2] = map(analogReadA[2], 0, 1023, 1023, 0);  //Inverted
  analogReadA[3] = map(analogReadA[3], 0, 1023, 0, 1023);  
  analogReadA[4] = map(analogReadA[4], 0, 1023, 1023, 0);  //Inverted
  analogReadA[5] = map(analogReadA[5], 0, 1023, 0, 1023);  
  analogReadA[6] = map(analogReadA[6], 0, 1023, 1023, 0);  //Inverted
  analogReadA[7] = map(analogReadA[7], 0, 1023, 0, 1023);

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
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(my_radio_pipe);
  //Resetujte každú hodnotu kanála
  sent_data.ch1 = 127;
  sent_data.ch2 = 127;
  sent_data.ch3 = 127;
  sent_data.ch4 = 127;
  sent_data.ch5 = 127;
  sent_data.ch6 = 127;
  sent_data.ch7 = 127;
  sent_data.ch8 = 127;

}

void setPreviousValues() {
  for (short i=0; i<=7; i++) {
    if(center_point_initialized[i] == false){
      previous2AnalogReadA[i] = previousAnalogReadA[i];
      previousAnalogReadA[i] = analogReadA[i];
    }
  }
}

bool tmp_all_centers_initialized = false;

void calibrateCenterPoints() {
  tmp_all_centers_initialized = true;
  for (short i=0; i<=7; i++) {
    if(center_point_initialized[i] == false){
      if( previous2AnalogReadA[i] == previousAnalogReadA[i] && previousAnalogReadA[i] == analogReadA[i])
      {
        Serial.println("calibrateCenterPoints: i="+String(i)+".");
        centerAnalogReadA[i] = analogReadA[i];
        center_point_initialized[i] = true;
      }
    }
    tmp_all_centers_initialized = tmp_all_centers_initialized && center_point_initialized[i];
  }
  all_center_points_initialized = tmp_all_centers_initialized;
  if(all_center_points_initialized == true) {
    Serial.println("calibrateCenterPoints: All center points set.");
    for (short i=0; i<=7; i++) {
      Serial.println("calibrateCenterPoints: centerAnalogReadA["+String(i)+"] = "+String(centerAnalogReadA[i])+".");
    }
  }
}

bool tmp_all_minMaxValues_initialized = false;

void checkMaxMinValues() {
  tmp_all_minMaxValues_initialized = true;
  bool minMaxValuesChanged = false;
  for (short i=0; i<=7; i++) {
    if( analogReadA[i] > maxAnalogReadA[i]){
      maxAnalogReadA[i] = analogReadA[i];
      Serial.println("checkMaxMinValues:                                       maxAnalogReadA["+String(i)+"] changed to "+String(maxAnalogReadA[i]));
      minMaxValuesChanged = true;
      if((maxAnalogReadA[i] - centerAnalogReadA[i] > minAcceptedMaxDifference) && (centerAnalogReadA[i] - minAnalogReadA[i] > minAcceptedMaxDifference)) {
        minMaxValues_initialized[i] = true;
      }
    }
    if( analogReadA[i] < minAnalogReadA[i]){
      minAnalogReadA[i] = analogReadA[i] ;
      Serial.println("checkMaxMinValues: minAnalogReadA["+String(i)+"] changed to "+String(minAnalogReadA[i]));
      minMaxValuesChanged = true;
      if((maxAnalogReadA[i] - centerAnalogReadA[i] > minAcceptedMaxDifference) && (centerAnalogReadA[i] - minAnalogReadA[i] > minAcceptedMaxDifference)) {
        minMaxValues_initialized[i] = true;
      }
    }
    tmp_all_minMaxValues_initialized = tmp_all_minMaxValues_initialized && minMaxValues_initialized[i];
  }
  all_minMaxValues_initialized = tmp_all_minMaxValues_initialized;
  /*
  if((all_minMaxValues_initialized == true) &&(minMaxValuesChanged == true)){
    for (short i=0; i<=7; i++) {
      Serial.println("checkMaxMinValues: minAnalogReadA["+String(i)+"] = "+String(minAnalogReadA[i])+", maxAnalogReadA["+String(i)+"] = "+String(maxAnalogReadA[i])+".");
    }
    Serial.println("checkMaxMinValues:OK");
  }
  */
}

void loop()
{
/*Nastavenie Kanalov /velkosť vychiliek a reverz serv/
  Normal:    data.ch1 = map( analogRead(A0), 0, 1024, 0, 255);
  Reverz:  data.ch1 = map( analogRead(A0), 0, 1024, 255, 0);  */
  if(all_center_points_initialized == false) {
    setPreviousValues();
    ReadAnalogData();
    calibrateCenterPoints();
    radio.write(&sent_data, sizeof(Data_to_be_sent));
  }
  else
  {
      ReadAnalogData();
      checkMaxMinValues();

      for (short i=0; i<=7; i++) {
        if(minMaxValues_initialized[i] == true) {
          sent_data_ch[i] = (analogReadA[i] < centerAnalogReadA[i] ? map( analogReadA[i], minAnalogReadA[i], centerAnalogReadA[i], 0, 127) : map( analogReadA[i], centerAnalogReadA[i], maxAnalogReadA[i], 127, 255));  //žlta
        } else {
          sent_data_ch[i] = (analogReadA[i] < centerAnalogReadA[i] ? map( analogReadA[i], 0, centerAnalogReadA[i], 0, 127) : map( analogReadA[i], centerAnalogReadA[i], 1023, 127, 255));  //žlta
        }

        if( abs(sent_data_ch[i] - 127) < 2 ) {
          sent_data_ch[i] = 127;
        }
      }

      for (short i=0; i<=7; i++) {
        if(  abs(prev_data_ch[i] - sent_data_ch[i])>2 ) {
          Serial.println("Chanel["+String(i)+"]:"+String(sent_data_ch[i])+"("+String(analogReadA[i])+")");
        }
      }

      for (short i=0; i<=7; i++) {
        prev_data_ch[i] =sent_data_ch[i];
      }
      sent_data.ch1 = sent_data_ch[0];
      sent_data.ch2 = sent_data_ch[1];
      sent_data.ch3 = sent_data_ch[2];
      sent_data.ch4 = sent_data_ch[3];
      sent_data.ch5 = sent_data_ch[4];
      sent_data.ch6 = sent_data_ch[5];
      sent_data.ch7 = sent_data_ch[6];
      sent_data.ch8 = sent_data_ch[7];
      
      
      //Serial.println("Chanels:"+String(sent_data.ch1)+"("+String(analogReadA[0])+"), "+String(sent_data.ch2)+"("+String(analogReadA[1])+") | "+String(sent_data.ch3)+"("+String(analogReadA[2])+"), "+String(sent_data.ch4)+"("+String(analogReadA[3])+") || "+String(sent_data.ch5)+"("+String(analogReadA[4])+"), "+String(sent_data.ch6)+"("+String(analogReadA[5])+") | "+String(sent_data.ch7)+"("+String(analogReadA[6])+"), "+String(sent_data.ch8)+"("+String(analogReadA[7])+")");
      
    radio.write(&sent_data, sizeof(Data_to_be_sent));
  }  
}