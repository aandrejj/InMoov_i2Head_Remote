#include "Arduino.h"
#include "WritePulsesToDisplay.h"

#define char_width_x 8

#define char_height_y 8  
#define char_shift_x  2
#define chr_point_shift_x  1

WritePulsesToDisplay::WritePulsesToDisplay() {
  //Serial.println("WritePulsesToDisplay  ctor.");
  //servo[]="S";
  //colon[]=":";
}

void WritePulsesToDisplay::begin()//ST7735 *theTft)//, int servo_Limits[]) 
{
  Serial.println("WritePulsesToDisplay:begin");
    //tft = theTft;


    Serial.println("setup: tft.initR()...");
    tft.initR();
    delay(500);
    //tft.initR(INITR_BLACKTAB); 

    //tft.pushColor(uint16_t color)
    //tft.pushColor(tft.Color565(RED,GREEN,BLUE));
    //tft.fillScreen(BLACK);
    //Set background colour
    Serial.println("setup: tft.fillScreen(BLACK)");
    tft.fillScreen(BLACK);
    Serial.println("setup: BLACK =done");

    Serial.println("WritePulsesToDisplay:End of begin().");
}

//---------------------------------------------------------------------
void WritePulsesToDisplay::writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, servo_Pwm, showDebug);  
}
void WritePulsesToDisplay::writeMINPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, LABEL_FORM_MIN, servo_Pwm, false);  
}


void WritePulsesToDisplay::writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm, showDebug);
}
void WritePulsesToDisplay::writeMIDPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, LABEL_FORM_MID, servo_Pwm, false);
}

void WritePulsesToDisplay::writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, (LABEL_FORM_MAX+1) , servo_Pwm, showDebug);
}

void WritePulsesToDisplay::writeMAXPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, LABEL_FORM_MAX+1, servo_Pwm, false);
}

void WritePulsesToDisplay::writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm, bool showDebug){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, 2, servo_Pwm, showDebug);
}

void WritePulsesToDisplay::writeCurrPulsesToDisplay (uint8_t chanelNum, uint16_t servo_Pwm){
  WritePulsesToDisplay::writeOneFieldToDisplay (chanelNum, 2, servo_Pwm, false);
}

void WritePulsesToDisplay::writeOneFieldToDisplay (uint8_t chanelNum,uint8_t form_label_Min_Mid_Max, uint16_t servo_Pwm, bool showDebug){
  uint8_t modulo = chanelNum % LEFT_ARROW_STEP;
  uint8_t div_result =chanelNum / LEFT_ARROW_STEP;
  uint8_t yPos = 2 + (div_result * ((LEFT_ARROW_STEP*8)+2)) + (modulo*8);

  if(showDebug == true) {
    Serial.print("WritePulsesToDisplay: ");
    Serial.print("chanelNum:"+String(chanelNum)+", form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", servo_Pwm:"+String(servo_Pwm)+",  ");
    Serial.print("div_result = "+String(div_result)+", modulo = "+String(modulo)+", ");
    Serial.println("yPos:"+String(yPos)+", ");
  } else {
    //Serial.println("writeOneFieldToDisplay: yPos:"+String(yPos)+", chanelNum:"+String(chanelNum)+", form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", servo_Pwm:"+String(servo_Pwm)+", servoPulseIndex:"+String(servoPulseIndex));
  }

  uint8_t xPos = (((char_shift_x + (form_label_Min_Mid_Max*3)) * char_width_x));
  WritePulsesToDisplay::writeOneFieldToDisplay_innerPart(xPos, chr_point_shift_x, yPos, char_height_y, form_label_Min_Mid_Max, servo_Pwm, chanelNum, showDebug);
}

void WritePulsesToDisplay::writeOneFieldToDisplay_innerPart (uint8_t xPos, uint16_t _chr_point_shift_x, uint8_t yPos, uint16_t _char_height_y, uint16_t form_label_Min_Mid_Max, uint16_t servo_Pwm,uint16_t chanelNum, bool showDebug)
{
  
  if(showDebug == true) {
    Serial.print("writeOneFieldToDisplay_innerPart: ");
    Serial.print("xPos:"+String(xPos)+", ");
    Serial.print("_chr_point_shift_x:"+String(_chr_point_shift_x)+", ");
    Serial.print("yPos:"+String(yPos)+", ");
    Serial.print("_char_height_y:"+String(_char_height_y)+", ");
    Serial.print("form_label_Min_Mid_Max:"+String(form_label_Min_Mid_Max)+", ");
    Serial.print("servo_Pwm:"+String(servo_Pwm)+", ");
    Serial.print("chanelNum:"+String(chanelNum)+", ");
    Serial.println("");
  }
  
  tft.fillRect((xPos + _chr_point_shift_x), yPos, (3*char_width_x)-_chr_point_shift_x, _char_height_y, BLACK);
  /*
  if(showDebug == true) {
    Serial.println("writeOneFieldToDisplay_innerPart: Part2");
  }
  */

  char numRead3[4];
  dtostrf(servo_Pwm, 4, 0, numRead3);
  tft.drawString(xPos, yPos, numRead3, YELLOW);

  /*
  if(showDebug == true) {
    Serial.print("writeOneFieldToDisplay_innerPart: End.OK");
  }
  */
}

void WritePulsesToDisplay::writeArrow_activeServoSet (byte activeServoSet) {
      tft.fillRect((128-(LEFT_ARROW_SIZE*8)), 0, (LEFT_ARROW_SIZE*8), 160, BLACK);
      tft.drawString((128-(LEFT_ARROW_SIZE*8)), ((activeServoSet * ((2+8) * LEFT_ARROW_STEP))+3), "<", WHITE, LEFT_ARROW_SIZE);
}

void WritePulsesToDisplay::drawString(uint8_t x, uint8_t y, char *c, uint16_t color, uint8_t size=1) {
  //Serial.println("WritePulsesToDisplay::drawString: x="+String(x)+", y="+String(y)+", *c="+String(c)+", color="+String(color)+", size="+String(size)+".");
  tft.drawString(x, y, c, color, size);
}
