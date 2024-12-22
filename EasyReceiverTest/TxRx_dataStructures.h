struct RX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t devType; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
    
    byte servoSet;

    byte s1min;
    byte s1mid;
    byte s1curr;
    byte s1max;

    byte fireBtn1;

    byte switchPos;

    //byte flags;

};

struct RX_SERIAL_DATA_STRUCTURE{
 int16_t mode; // mode:  0 = fourSticksController (8 chanels) ,   1 = ServoConfigurator (16 chanels) , 2 = MinMaxServoConfig (min max for 2 chanels)
    
    byte s00;
    byte s01;
    byte s02;
    byte s03;
    byte s04;
    byte s05;
    byte s06;
    byte s07;
    byte s08;
    byte s09;
    byte s10;
    byte s11;
    byte s12;
    byte s13;
    byte s14;
    byte s15;

    byte x00;
    byte x01;
    byte x02;
    byte x03;
    byte x04;
    byte x05;
    byte x06;
    byte x07;
    byte x08;
    byte x09;
    byte x10;
    byte x11;
    byte x12;
    byte x13;
    byte x14;
    byte x15;

    byte flags;
};

struct TX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

