
#define eyeToLip_Scale  0.5 
#define ch5ToLip_Scale  0.45 
#define ch6ToLip_Scale  0.45 


int ch_1 = 0;
int ch_2 = 0;
int ch_3 = 0;
int ch_4 = 0;
int ch_5 = 0;
int ch_6 = 0;
int ch_7 = 0;
int ch_8 = 0;
byte ch[9]    =      {   0,   0,   0,   0,   0,   0,   0,   0,  0};
byte prev_ch[9]    = {   0,   0,   0,   0,   0,   0,   0,   0,  0};

bool center_point_initialized[8] = {false, false, false, false, false, false, false, false};
bool all_center_points_initialized = false;
//bool center_points_initialized;

byte ch_1_center = 0;
byte ch_2_center = 0;
byte ch_3_center = 0;
byte ch_4_center = 0;
byte ch_5_center = 0;
byte ch_6_center = 0;
byte ch_7_center = 0;
byte ch_8_center = 0;

byte ch_center[8]    = {   0,   0,   0,   0,   0,   0,   0,   0};


//byte ch_1_constrained = 0;
//byte ch_2_constrained = 0;
//byte ch_3_constrained = 0;
//byte ch_4_constrained = 0;
//byte ch_5_constrained = 0;
//byte ch_6_constrained = 0;
//byte ch_7_constrained = 0;
//byte ch_8_constrained = 0;
byte ch_constrained[8]        = {   0,   0,   0,   0,   0,   0,   0,   0};
//byte prev_ch_constrained[8]        = {   0,   0,   0,   0,   0,   0,   0,   0};

int servo_eyeLeftUD_Angle       ;
int servo_eyeLeftLR_Angle       ;
int servo_eyeRightUD_Angle      ;
int servo_eyeRightLR_Angle      ; 
int servo_eyelidLeftUpper_Angle ;
int servo_eyelidLeftLower_Angle ;
int servo_eyelidRightUpper_Angle;
int servo_eyelidRightLower_Angle;
int servo_eyebrowRight_Angle    ;
int servo_eyebrowLeft_Angle     ;
int servo_cheekRight_Angle      ;
int servo_cheekLeft_Angle       ;
int servo_upperLip_Angle        ;
int servo_forheadRight_Angle    ;
int servo_forheadLeft_Angle     ;
int servo_Jaw_UpDown_Angle      ;

int servo_eyeLeftUD_Pwm       ;
int servo_eyeLeftLR_Pwm       ;
int servo_eyeRightUD_Pwm      ;
int servo_eyeRightLR_Pwm      ; 
int servo_eyelidLeftUpper_Pwm ;
int servo_eyelidLeftLower_Pwm ;
int servo_eyelidRightUpper_Pwm;
int servo_eyelidRightLower_Pwm;
int servo_eyebrowRight_Pwm    ;
int servo_eyebrowLeft_Pwm     ;
int servo_cheekRight_Pwm      ;
int servo_cheekLeft_Pwm       ;
int servo_upperLip_Pwm        ;
int servo_forheadRight_Pwm    ;
int servo_forheadLeft_Pwm     ;
int servo_Jaw_UpDown_Pwm      ;

int prev_servo_eyeLeftUD_Pwm       ;
int prev_servo_eyeLeftLR_Pwm       ;
int prev_servo_eyeRightUD_Pwm      ;
int prev_servo_eyeRightLR_Pwm      ;
int prev_servo_eyelidLeftUpper_Pwm ;
int prev_servo_eyelidLeftLower_Pwm ;
int prev_servo_eyelidRightUpper_Pwm;
int prev_servo_eyelidRightLower_Pwm;
int prev_servo_eyebrowRight_Pwm    ;
int prev_servo_eyebrowLeft_Pwm     ;
int prev_servo_cheekRight_Pwm      ;
int prev_servo_cheekLeft_Pwm       ;
int prev_servo_upperLip_Pwm        ;
int prev_servo_forheadRight_Pwm    ;
int prev_servo_forheadLeft_Pwm     ;
int prev_servo_Jaw_UpDown_Pwm      ;


bool s00_changed = false;
bool s01_changed = false;
bool s02_changed = false;
bool s03_changed = false;
bool s04_changed = false;
bool s05_changed = false;
bool s06_changed = false;
bool s07_changed = false;
bool s08_changed = false;
bool s09_changed = false;
bool s10_changed = false;
bool s11_changed = false;
bool s12_changed = false;
bool s13_changed = false;
bool s14_changed = false;
bool s15_changed = false;

//int servoLimits[17,3]={};
int servoLimits[48]={
  SERVO_MIN_eyeLeftUD        ,
  SERVO_MIN_eyeLeftLR        ,
  SERVO_MIN_eyeRightUD       ,
  SERVO_MIN_eyeRightLR       ,
  SERVO_MIN_eyelidLeftUpper  ,
  SERVO_MIN_eyelidLeftLower  ,
  SERVO_MIN_eyelidRightUpper ,
  SERVO_MIN_eyelidRightLower ,
  SERVO_MIN_eyebrowRight     ,
  SERVO_MIN_eyebrowLeft      ,
  SERVO_MIN_cheekRight       ,
  SERVO_MIN_cheekLeft        ,
  SERVO_MIN_upperLip         ,
  SERVO_MIN_forheadRight     ,
  SERVO_MIN_forheadLeft      ,
  SERVO_MIN_Jaw_UpDown       ,
   SERVO_MID_eyeLeftUD       ,
   SERVO_MID_eyeLeftLR       ,
   SERVO_MID_eyeRightUD      ,
   SERVO_MID_eyeRightLR      ,
   SERVO_MID_eyelidLeftUpper ,
   SERVO_MID_eyelidLeftLower ,
   SERVO_MID_eyelidRightUpper,
   SERVO_MID_eyelidRightLower,
   SERVO_MID_eyebrowRight    ,
   SERVO_MID_eyebrowLeft     ,
   SERVO_MID_cheekRight      ,
   SERVO_MID_cheekLeft       ,
   SERVO_MID_upperLip        ,
   SERVO_MID_forheadRight    ,
   SERVO_MID_forheadLeft     ,
   SERVO_MID_Jaw_UpDown      ,
   SERVO_MAX_eyeLeftUD       ,
   SERVO_MAX_eyeLeftLR       ,
   SERVO_MAX_eyeRightUD      ,
   SERVO_MAX_eyeRightLR      ,
   SERVO_MAX_eyelidLeftUpper ,
   SERVO_MAX_eyelidLeftLower ,
   SERVO_MAX_eyelidRightUpper,
   SERVO_MAX_eyelidRightLower,
   SERVO_MAX_eyebrowRight    ,
   SERVO_MAX_eyebrowLeft     ,
   SERVO_MAX_cheekRight      ,
   SERVO_MAX_cheekLeft       ,
   SERVO_MAX_upperLip        ,
   SERVO_MAX_forheadRight    ,
   SERVO_MAX_forheadLeft     ,
   SERVO_MAX_Jaw_UpDown      
};

/*  int servo_eyeLeftUD_Min_Angle        = SERVO_MIN_eyeLeftUD       ;
  int servo_eyeLeftLR_Min_Angle        = SERVO_MIN_eyeLeftLR       ;
  int servo_eyeRightUD_Min_Angle       = SERVO_MIN_eyeRightUD      ;
  int servo_eyeRightLR_Min_Angle       = SERVO_MIN_eyeRightLR      ;
  int servo_eyelidLeftUpper_Min_Angle  = SERVO_MIN_eyelidLeftUpper ;
  int servo_eyelidLeftLower_Min_Angle  = SERVO_MIN_eyelidLeftLower ;
  int servo_eyelidRightUpper_Min_Angle = SERVO_MIN_eyelidRightUpper;
  int servo_eyelidRightLower_Min_Angle = SERVO_MIN_eyelidRightLower;
  int servo_eyebrowRight_Min_Angle     = SERVO_MIN_eyebrowRight    ;
  int servo_eyebrowLeft_Min_Angle      = SERVO_MIN_eyebrowLeft     ;
  int servo_cheekRight_Min_Angle       = SERVO_MIN_cheekRight      ;
  int servo_cheekLeft_Min_Angle        = SERVO_MIN_cheekLeft       ;
  int servo_upperLip_Min_Angle         = SERVO_MIN_upperLip        ;
  int servo_forheadRight_Min_Angle     = SERVO_MIN_forheadRight    ;
  int servo_forheadLeft_Min_Angle      = SERVO_MIN_forheadLeft     ;
  int servo_Jaw_UpDown_Min_Angle       = SERVO_MIN_Jaw_UpDown      ;

  int servo_eyeLeftUD_Max_Angle         = SERVO_MID_eyeLeftUD       ;
  int servo_eyeLeftLR_Max_Angle         = SERVO_MID_eyeLeftLR       ;
  int servo_eyeRightUD_Max_Angle        = SERVO_MID_eyeRightUD      ;
  int servo_eyeRightLR_Max_Angle        = SERVO_MID_eyeRightLR      ;
  int servo_eyelidLeftUpper_Max_Angle   = SERVO_MID_eyelidLeftUpper ;
  int servo_eyelidLeftLower_Max_Angle   = SERVO_MID_eyelidLeftLower ;
  int servo_eyelidRightUpper_Max_Angle  = SERVO_MID_eyelidRightUpper;
  int servo_eyelidRightLower_Max_Angle  = SERVO_MID_eyelidRightLower;
  int servo_eyebrowRight_Max_Angle      = SERVO_MID_eyebrowRight    ;
  int servo_eyebrowLeft_Max_Angle       = SERVO_MID_eyebrowLeft     ;
  int servo_cheekRight_Max_Angle        = SERVO_MID_cheekRight      ;
  int servo_cheekLeft_Max_Angle         = SERVO_MID_cheekLeft       ;
  int servo_upperLip_Max_Angle          = SERVO_MID_upperLip        ;
  int servo_forheadRight_Max_Angle      = SERVO_MID_forheadRight    ;
  int servo_forheadLeft_Max_Angle       = SERVO_MID_forheadLeft     ;
  int servo_Jaw_UpDown_Max_Angle        = SERVO_MID_Jaw_UpDown      ;

  int servo_eyeLeftUD_Mid_Angle         = SERVO_MAX_eyeLeftUD       ;
  int servo_eyeLeftLR_Mid_Angle         = SERVO_MAX_eyeLeftLR       ;
  int servo_eyeRightUD_Mid_Angle        = SERVO_MAX_eyeRightUD      ;
  int servo_eyeRightLR_Mid_Angle        = SERVO_MAX_eyeRightLR      ;
  int servo_eyelidLeftUpper_Mid_Angle   = SERVO_MAX_eyelidLeftUpper ;
  int servo_eyelidLeftLower_Mid_Angle   = SERVO_MAX_eyelidLeftLower ;
  int servo_eyelidRightUpper_Mid_Angle  = SERVO_MAX_eyelidRightUpper;
  int servo_eyelidRightLower_Mid_Angle  = SERVO_MAX_eyelidRightLower;
  int servo_eyebrowRight_Mid_Angle      = SERVO_MAX_eyebrowRight    ;
  int servo_eyebrowLeft_Mid_Angle       = SERVO_MAX_eyebrowLeft     ;
  int servo_cheekRight_Mid_Angle        = SERVO_MAX_cheekRight      ;
  int servo_cheekLeft_Mid_Angle         = SERVO_MAX_cheekLeft       ;
  int servo_upperLip_Mid_Angle          = SERVO_MAX_upperLip        ;
  int servo_forheadRight_Mid_Angle      = SERVO_MAX_forheadRight    ;
  int servo_forheadLeft_Mid_Angle       = SERVO_MAX_forheadLeft     ;
  int servo_Jaw_UpDown_Mid_Angle        = SERVO_MAX_Jaw_UpDown      ;
*/
