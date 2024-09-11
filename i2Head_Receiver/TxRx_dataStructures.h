struct RX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  
    int16_t mode;  

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
};

struct TX_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int16_t mode;
  int16_t count;
};

