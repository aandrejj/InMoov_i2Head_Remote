/*
* Arduino Wireless Communication Tutorial
*       Example 1 - Receiver Code
*                
* by Dejan Nedelkovski, www.HowToMechatronics.com
* https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
* Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "TxRx_dataStructures.h"


RX_DATA_STRUCTURE mydata;

RF24 radio(10, 9); // CE, CSN

//const byte address[6] = "00001";
const uint64_t my_radio_pipe = 0x0022;

void setup() {
  Serial.begin(9600);
  delay(200);
  
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);

  radio.begin();
  //radio.setAutoAck(true);
  radio.openReadingPipe(1, my_radio_pipe);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("setup: @4 rf-radio started");
}

void loop() {
  if (radio.available()) {
    char text[32] = "";
    //radio.read(&text, sizeof(text));
    radio.read(&mydata, sizeof(RX_DATA_STRUCTURE));
    //Serial.println(text);
    Serial.println("s1min: "+String(mydata.s1min)+", s1curr: "+String(mydata.s1curr)+", s1mid: "+String(mydata.s1mid)+", s1max: "+String(mydata.s1max)+", servoSet: "+String(mydata.servoSet)+", devType:"+String(mydata.devType));
  }
}