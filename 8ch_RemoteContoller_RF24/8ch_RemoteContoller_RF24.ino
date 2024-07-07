/* Kod vysielača pre RF NANO
Kod vysielača pre 4kanaly ,datove piny A0, A1, A2, A3
// https://blog.laskakit.cz/projekt-rc-arduino/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
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
void loop()
{
/*Nastavenie Kanalov /velkosť vychiliek a reverz serv/
  Normal:    data.ch1 = map( analogRead(A0), 0, 1024, 0, 255);
  Reverz:  data.ch1 = map( analogRead(A0), 0, 1024, 255, 0);  */
  
  sent_data.ch1 = map( analogRead(A0), 0, 1000, 110, 220); //čierny
  sent_data.ch2 = map( analogRead(A1), 0, 1124, 255, 60);  //žlta
  sent_data.ch3 = map( analogRead(A2), 0, 1124, 240, 90); //modra
  sent_data.ch4 = map( analogRead(A3), 0, 1020, 235, 80); //červeny
  sent_data.ch5 = map( analogRead(A4), 0, 1000, 110, 220); //čierny
  sent_data.ch6 = map( analogRead(A5), 0, 1124, 255, 60);  //žlta
  sent_data.ch7 = map( analogRead(A6), 0, 1124, 240, 90); //modra
  sent_data.ch8 = map( analogRead(A7), 0, 1020, 235, 80); //červeny
  Serial.println("Chanels:"+String(sent_data.ch1)+", "+String(sent_data.ch2)+" | "+String(sent_data.ch3)+", "+String(sent_data.ch4)+" || "+String(sent_data.ch5)+", "+String(sent_data.ch6)+" | "+String(sent_data.ch7)+", "+String(sent_data.ch8));

    radio.write(&sent_data, sizeof(Data_to_be_sent));
    
}