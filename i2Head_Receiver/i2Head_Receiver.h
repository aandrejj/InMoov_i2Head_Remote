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
