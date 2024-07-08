/* RF NANO Prímacovy kod 
 * PWM vystup kanalov D2,D3,D4,D5,
 // https://blog.laskakit.cz/projekt-rc-arduino/
 */
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
//Define widths
int ch_1 = 0;
int ch_2 = 0;
int ch_3 = 0;
int ch_4 = 0;
int ch_5 = 0;
int ch_6 = 0;
int ch_7 = 0;
int ch_8 = 0;

Servo PWM2;
Servo PWM3;
Servo PWM4;
Servo PWM5;
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
  recvData();
  unsigned long now = millis();
  //Tu skontrolujeme, či sme stratili signál, ak ano, tak by sme resetovali hodnoty
  if ( now - lastRecvTime > 1200 ) {
    //Stratený signál?
    resetData();
  }
  //Nastavenie koncovych a stred polôh
ch_1 = map(data.ch1,  110, 210, 200, 2000);  //PWM vystup digital pin D1 čierny
ch_2 = map(data.ch2,  100, 255, 200, 2000);  //PWM vystup digital pin D2 žltý
ch_3 = map(data.ch3,  120, 245, 200, 2000);  //PWM vystup digital pin D3 modrý
ch_4 = map(data.ch4,  100, 245, 200, 2000);  //PWM vystup digital pin D4 červeny

ch_5 = map(data.ch5,  115, 210, 200, 2000);  //PWM vystup digital pin D5 čierny
ch_6 = map(data.ch6,  100, 255, 200, 2000);  //PWM vystup digital pin D6 žltý
ch_7 = map(data.ch7,  120, 245, 200, 2000);  //PWM vystup digital pin D7 modrý
ch_8 = map(data.ch8,  100, 240, 200, 2000);  //PWM vystup digital pin D8 červeny

Serial.println("Chanels:"+String(ch_1)+", "+String(ch_2)+" | "+String(ch_3)+", "+String(ch_4)+" || "+String(ch_5)+", "+String(ch_6)+" | "+String(ch_7)+", "+String(ch_8));
//Teraz napíšeme signál PWM pomocou funkcie servo
//PWM2.writeMicroseconds(ch_2);
//PWM3.writeMicroseconds(ch_3);
//PWM4.writeMicroseconds(ch_4);
//PWM5.writeMicroseconds(ch_5);

}//Koniec slučky