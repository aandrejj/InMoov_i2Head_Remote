
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


const uint64_t pipeOut = 0xE9E8F0F0E1LL; //IMPORTANT: The same as in the receiver 0xE9E8F0F0E1LL
RF24 radio(7, 8); // select CE,CSN pin |

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

void ResetData()
{
data.throttle_a = 127; // Motor Stop (254/2=127)(Signal lost position)
data.pitch_a = 127; // Center (Signal lost position)
data.roll_a = 127; // Center(Signal lost position)
data.yaw_a = 127; // Center (Signal lost position )
data.throttle_b = 127; // Center (Signal lost position )
data.pitch_b = 127; // Center (Signal lost position )
data.roll_b = 127; // Center (Signal lost position )
data.yaw_b = 127; // Center (Signal lost position )
}

void setup()
{
//Start everything up

radio.begin();
radio.openWritingPipe(pipeOut);
radio.stopListening(); //start the radio comunication for Transmitter
ResetData();
}

// Joystick center and its borders

int mapJoystickValues(int val, int lower, int middle, int upper, bool reverse)
{
val = constrain(val, lower, upper);
if ( val < middle )
val = map(val, lower, middle, 0, 128);
else
val = map(val, middle, upper, 128, 255);
return ( reverse ? 255 - val : val );
}


void loop()
{
// Control Stick Calibration
// Setting may be required for the correct values of the control levers.

data.throttle_a = mapJoystickValues( analogRead(A0), 524, 524, 1015, false );
data.roll_a = mapJoystickValues( analogRead(A1), 12, 524, 1020, true ); // "true" or "false" for servo direction
data.pitch_a = mapJoystickValues( analogRead(A2), 12, 524, 1020, true ); // "true" or "false" for servo direction
data.yaw_a = mapJoystickValues( analogRead(A3), 12, 524, 1020, true ); // "true" or "false" for servo direction
data.throttle_b = mapJoystickValues( analogRead(A4), 12, 524, 1020, true ); // "true" or "false" for servo direction
data.roll_b = mapJoystickValues( analogRead(A5), 12, 524, 1020, true ); // "true" or "false" for servo direction
data.pitch_b = mapJoystickValues( analogRead(A6), 12, 524, 1020, true ); // "true" or "false" for servo direction
data.yaw_b = mapJoystickValues( analogRead(A7), 12, 524, 1020, true ); // "true" or "false" for servo direction

radio.write(&data, sizeof(Signal));
}