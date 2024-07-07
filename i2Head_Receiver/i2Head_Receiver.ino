#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;
int ch_width_7 = 0;
int ch_width_8 = 0;

Servo ch1;
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;
Servo ch7;
Servo ch8;

struct Signal {
byte throttle_a;
byte pitch_a;
byte roll_a;
byte yaw_a;
byte throttle_b;
byte pitch_b;
byte roll_b;
byte yaw_b;
};

Signal data;

const uint64_t pipeIn = 0xE9E8F0F0E1LL;
RF24 radio(7, 8);

void ResetData()
{
// Define the inicial value of each data input.
// The middle position for Potenciometers. (254/2=127)
data.throttle_a = 127; // Motor Stop
data.pitch_a = 127; // Center
data.roll_a = 127; // Center
data.yaw_a = 127; // Center
data.throttle_b = 127; // Center
data.pitch_b = 127; // Center
data.roll_b = 127; // Center
data.yaw_b = 127; // Center
}

void setup()
{
//Set the pins for each PWM signal
ch1.attach(2);
ch2.attach(3);
ch3.attach(4);
ch4.attach(5);
ch5.attach(1);
ch6.attach(6);
ch7.attach(9);
ch8.attach(10);

//Configure the NRF24 module
ResetData();
radio.begin();
radio.openReadingPipe(1,pipeIn);

radio.startListening(); //start the radio comunication for receiver
}

unsigned long lastRecvTime = 0;

void recvData()
{
while ( radio.available() ) {
radio.read(&data, sizeof(Signal));
lastRecvTime = millis();   // receive the data | 
}
}

void loop()
{
recvData();
unsigned long now = millis();
if ( now - lastRecvTime > 1000 ) {
ResetData(); // Signal lost.. Reset data | 
}

ch_width_1 = map(data.throttle_a, 0, 255, 1000, 2000);
ch_width_2 = map(data.pitch_a, 0, 255, 1000, 2000);
ch_width_3 = map(data.roll_a, 0, 255, 1000, 2000);
ch_width_4 = map(data.yaw_a, 0, 255, 1000, 2000);
ch_width_5 = map(data.throttle_b, 0, 255, 1000, 2000);
ch_width_6 = map(data.pitch_b, 0, 255, 1000, 2000);
ch_width_7 = map(data.roll_b, 0, 255, 1000, 2000);
ch_width_8 = map(data.yaw_b, 0, 255, 1000, 2000);

// Write the PWM signal
ch1.writeMicroseconds(ch_width_1);
ch2.writeMicroseconds(ch_width_2);
ch3.writeMicroseconds(ch_width_3);
ch4.writeMicroseconds(ch_width_4);
ch5.writeMicroseconds(ch_width_5);
ch6.writeMicroseconds(ch_width_6);
ch7.writeMicroseconds(ch_width_7);
ch8.writeMicroseconds(ch_width_8);
}