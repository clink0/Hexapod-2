//***********************************************************************
// Hexapod Program
// Code for Arduino Mega
// Serial command control (USB) - PS2 controller removed
// by Mark W, modified for serial control
//***********************************************************************

//***********************************************************************
// IK and Hexapod gait references:
//  https://www.projectsofdan.com/?cat=4
//  http://www.gperco.com/2015/06/hex-inverse-kinematics.html
//  http://virtual-shed.blogspot.com/2012/12/hexapod-inverse-kinematics-part-1.html
//  http://virtual-shed.blogspot.com/2013/01/hexapod-inverse-kinematics-part-2.html
//  https://www.robotshop.com/community/forum/t/inverse-kinematic-equations-for-lynxmotion-3dof-legs/21336
//  http://arduin0.blogspot.com/2012/01/inverse-kinematics-ik-implementation.html?utm_source=rb-community&utm_medium=forum&utm_campaign=inverse-kinematic-equations-for-lynxmotion-3dof-legs
//***********************************************************************

//***********************************************************************
// Serial Command Reference:
//
//  MODE <n>     - Set mode: 0=idle, 1=walk, 2=translate, 3=rotate,
//                           4=one_leg_lift, 5=scan, 6=navigate, 7=sensor_cal, 99=set_all_90
//  GAIT <n>     - Select gait: 0=tripod, 1=wave, 2=ripple, 3=tetrapod
//                 (also stops movement and resets position)
//  SPEED <n>    - Set gait speed: 0=fast, 1=slow
//  JOY <RX> <RY> <LX> <LY>
//               - Set joystick axes, all values 0-255 (128 = center/neutral)
//                 RX/RY = right stick (walk Y/X, translate Y/X, rotate pitch/roll)
//                 LX/LY = left stick  (walk rotation, translate Z, rotate yaw/Z)
//  CAPTURE      - Capture/lock current offsets (like L1/R1)
//  CLEAR        - Clear all offsets and reset step height (like L2/R2)
//  HOME         - Reset all legs to home position
//  STATUS       - Print current mode, gait, speed, joystick, and heading
//  COMPASS      - Print current compass heading and locked target heading
//
// Example Python usage (pyserial):
//   ser.write(b"MODE 1\n")          # enter walk mode
//   ser.write(b"GAIT 0\n")          # select tripod gait
//   ser.write(b"JOY 128 50 128 128\n")  # walk forward
//   ser.write(b"JOY 128 128 128 128\n") # stop (all centered)
//***********************************************************************

//***********************************************************************
// Includes
//***********************************************************************
#include <Servo.h>
#include <math.h>
#include <Wire.h>
#include "ICM_20948.h"
#include "MagCal.h"


//***********************************************************************
// Constant Declarations
//***********************************************************************
const int BATT_VOLTAGE = 0;

const int TRIG_RIGHT = 2;             //HC-SR04 right sensor (front-right leg)
const int ECHO_RIGHT = 3;
const int TRIG_LEFT  = 4;             //HC-SR04 left sensor (front-left leg)
const int ECHO_LEFT  = 5;

const int SW_BIT0    = A3;            //mode switchboard: 3-bit binary input
const int SW_BIT1    = A4;            //  A3=bit0 (LSB), A4=bit1, A5=bit2 (MSB)
const int SW_BIT2    = A5;            //  LOW = switch closed (INPUT_PULLUP)
const int SW_BUTTON  = A2;            //push-button trigger (A6/A7 are analog-only on Mega; use A0-A5)

const int COXA1_SERVO  = 19;
const int FEMUR1_SERVO = 8;
const int TIBIA1_SERVO = 23;
const int COXA2_SERVO  = 25;
const int FEMUR2_SERVO = 27;
const int TIBIA2_SERVO = 29;
const int COXA3_SERVO  = 31;
const int FEMUR3_SERVO = 33;
const int TIBIA3_SERVO = 35;
const int COXA4_SERVO  = 37;
const int FEMUR4_SERVO = 39;
const int TIBIA4_SERVO = 41;
const int COXA5_SERVO  = 43;
const int FEMUR5_SERVO = 45;
const int TIBIA5_SERVO = 47;
const int COXA6_SERVO  = 49;
const int FEMUR6_SERVO = 51;
const int TIBIA6_SERVO = 53;

const int RED_LED1   = 22;            //LED port definitions
const int GREEN_LED1 = 24;
const int RED_LED2   = 26;
const int GREEN_LED2 = 28;
const int RED_LED3   = 30;
const int GREEN_LED3 = 32;
const int RED_LED4   = 34;
const int GREEN_LED4 = 36;
const int RED_LED5   = 38;
const int GREEN_LED5 = 40;
const int RED_LED6   = 42;
const int GREEN_LED6 = 44;
const int RED_LED7   = 46;
const int GREEN_LED7 = 48;
const int RED_LED8   = 50;
const int GREEN_LED8 = 52;

const int COXA_LENGTH = 51;           //leg part lengths
const int FEMUR_LENGTH = 65;
const int TIBIA_LENGTH = 121;

const int TRAVEL = 30;                //translate and rotate travel limit constant

const float SCAN_STANCE_Z    = -120.0; //support leg Z during scan/navigate tall stance (HOME_Z - 40)
const float SCAN_RAISED_Z    =  -50.0; //scanning leg Z while raised
const float SCAN_RADIUS      =  100.0; //foot arc radius during scan (mm from body center)
const int   SCAN_LIFT_TICKS  =  25;    //frames to lift or lower a scanning leg (~0.5s)
const int   SCAN_SWEEP_TICKS =  100;   //frames for a full 90-degree sweep (~2s)

const long A12DEG = 209440;           //12 degrees in radians x 1,000,000
const long A30DEG = 523599;           //30 degrees in radians x 1,000,000

const int FRAME_TIME_MS = 20;         //frame time (20msec = 50Hz)

const float HOME_X[6] = {  82.0,   0.0, -82.0,  -82.0,    0.0,  82.0};  //coxa-to-toe home positions
const float HOME_Y[6] = {  82.0, 116.0,  82.0,  -82.0, -116.0, -82.0};
const float HOME_Z[6] = { -80.0, -80.0, -80.0,  -80.0,  -80.0, -80.0};

const float BODY_X[6] = { 110.4,   0.0, -110.4, -110.4,    0.0, 110.4};  //body center-to-coxa servo distances
const float BODY_Y[6] = {  58.4, 90.8,   58.4,  -58.4,  -90.8, -58.4};
const float BODY_Z[6] = {   0.0,   0.0,    0.0,    0.0,    0.0,   0.0};

const int COXA_CAL[6]  = {7, -2, 0, 7, 10, 2};                       //servo calibration constants
const int FEMUR_CAL[6] = {0, 0,  0, 0,  0,  0};
const int TIBIA_CAL[6] = {0, 0, 0, 0, 0, 0};

//negative is cw

//***********************************************************************
// Variable Declarations
//***********************************************************************

// --- Virtual joystick axes (0-255, 128 = center/neutral) ---
int joy_RX = 128;
int joy_RY = 128;
int joy_LX = 128;
int joy_LY = 128;

// --- Serial input buffer ---
String serialBuffer = "";

unsigned long currentTime;            //frame timer variables
unsigned long previousTime;

int temp;                             //mode and control variables
int mode;
int gait;
int gait_speed;
int gait_LED_color;
int reset_position;
int capture_offsets;

int batt_LEDs;                        //battery monitor variables
int batt_voltage;
int batt_voltage_index;
int batt_voltage_array[50];
long batt_voltage_sum;

float L0, L3;                         //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control; //leg lift mode variables
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int scan_phase;                       //scan mode variables (0-5 state machine)
int scan_tick;

int auto_phase;                       //navigate mode variables
int auto_timer;
int auto_scan_min_right;
int auto_scan_min_left;

int sensor_right_offset = 0;         //calibration offset applied to right sensor in navigate mode
int cal_count;                        //0 = burst not yet run, 1 = done
int vfh_left[7];                      // distances at 90°,75°,60°,45°,30°,15°,0° (left side)
int vfh_right[7];                     // distances at 90°,75°,60°,45°,30°,15°,0° (right side)
int auto_turn_dir;                    //-1=turn left, 0=straight, 1=turn right
int auto_straight_streak;             //consecutive straight decisions
int auto_walk_ticks      = 100;       //current walk duration in frames (grows with streak)
int auto_turn_ticks      = 75;        //current turn duration in frames (set each DECIDE)
int auto_turn_joy        = 128;       //joy_LX value for turn (set each DECIDE)

int leg_num;                          //positioning and walking variables
int z_height_LED_color;
int totalX, totalY, totalZ;
int tick, duration, numTicks;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1,2,1,2,1,2};     //for tripod gait walking
int ripple_case[6]   = {2,6,4,1,3,5};     //for ripple gait
int wave_case[6]     = {1,2,3,4,5,6};     //for wave gait
int tetrapod_case[6] = {1,3,2,1,2,3};     //for tetrapod gait


//***********************************************************************
// Compass / IMU
//***********************************************************************
ICM_20948_I2C myICM;
bool  imu_ready      = false;
MagCal magCal;
float compass_target  = -1.0;  // -1 = not locked; 0-360 = target heading (degrees)
float compass_sin_acc =  0.0;  // low-pass filter accumulators (sin/cos form handles wrap)
float compass_cos_acc =  1.0;
bool  compass_filter_init = false;

// --- Mode switchboard ---
int switch_pending_mode = -1;  // mode waiting to start after home countdown
int switch_countdown    =  0;  // frames remaining in 2-second home pause

//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo;      //18 servos
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;


//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  Serial.begin(115200);
  Serial.println("Hexapod Serial Control Ready");
  Serial.println("Send STATUS for command reference.");

  //attach servos
  coxa1_servo.attach(COXA1_SERVO,610,2400);
  femur1_servo.attach(FEMUR1_SERVO,610,2400);
  tibia1_servo.attach(TIBIA1_SERVO,610,2400);
  coxa2_servo.attach(COXA2_SERVO,610,2400);
  femur2_servo.attach(FEMUR2_SERVO,610,2400);
  tibia2_servo.attach(TIBIA2_SERVO,610,2400);
  coxa3_servo.attach(COXA3_SERVO,610,2400);
  femur3_servo.attach(FEMUR3_SERVO,610,2400);
  tibia3_servo.attach(TIBIA3_SERVO,610,2400);
  coxa4_servo.attach(COXA4_SERVO,610,2400);
  femur4_servo.attach(FEMUR4_SERVO,610,2400);
  tibia4_servo.attach(TIBIA4_SERVO,610,2400);
  coxa5_servo.attach(COXA5_SERVO,610,2400);
  femur5_servo.attach(FEMUR5_SERVO,610,2400);
  tibia5_servo.attach(TIBIA5_SERVO,610,2400);
  coxa6_servo.attach(COXA6_SERVO,610,2400);
  femur6_servo.attach(FEMUR6_SERVO,610,2400);
  tibia6_servo.attach(TIBIA6_SERVO,610,2400);

  //set up LED pins as outputs
  for(int i=0; i<8; i++)
  {
    pinMode((RED_LED1+(4*i)),OUTPUT);
    pinMode((GREEN_LED1+(4*i)),OUTPUT);
  }

  //set up ultrasonic sensor pins
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);

  //set up mode switchboard pins (internal pull-ups; LOW = switch closed)
  pinMode(SW_BIT0,   INPUT_PULLUP);
  pinMode(SW_BIT1,   INPUT_PULLUP);
  pinMode(SW_BIT2,   INPUT_PULLUP);
  pinMode(SW_BUTTON, INPUT_PULLUP);

  //set up battery monitor average array
  for(batt_voltage_index=0; batt_voltage_index<50; batt_voltage_index++)
    batt_voltage_array[batt_voltage_index] = 0;
  batt_voltage_sum = 0;
  batt_voltage_index = 0;

  //clear offsets
  for(leg_num=0; leg_num<6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
  capture_offsets = false;
  step_height_multiplier = 1.0;

  //initialize mode and gait variables
  mode = 0;
  gait = 0;
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;

  //initialize ICM-20948 compass
  Wire.begin();
  Wire.setClock(400000);
  myICM.begin(Wire, 1);   // AD0 high → 0x69; change to 0 for 0x68
  if(myICM.status == ICM_20948_Stat_Ok) {
    imu_ready = true;
    if(magCalLoad(magCal)) {
      Serial.println("Compass ready (calibration loaded from EEPROM).");
    } else {
      magCal.offsetX = 0; magCal.offsetY = 0; magCal.scaleX = 1; magCal.scaleY = 1;
      Serial.println("Compass ready (NO calibration — run IMU_Calibrate.ino first).");
    }
  } else {
    Serial.println("ICM-20948 not found — compass heading lock disabled.");
  }
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop()
{
  currentTime = millis();
  if((currentTime - previousTime) > FRAME_TIME_MS)
  {
    previousTime = currentTime;

    //read and process any incoming serial commands
    process_serial();
    process_switches();

    if(reset_position == true)
    {
      for(leg_num=0; leg_num<6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    if(mode < 99)
    {
      for(leg_num=0; leg_num<6; leg_num++)
        leg_IK(leg_num, current_X[leg_num]+offset_X[leg_num],
                        current_Y[leg_num]+offset_Y[leg_num],
                        current_Z[leg_num]+offset_Z[leg_num]);
    }

    if(mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    battery_monitor();
    //print_debug();

    if(mode == 1)
    {
      if(gait == 0) tripod_gait();
      if(gait == 1) wave_gait();
      if(gait == 2) ripple_gait();
      if(gait == 3) tetrapod_gait();
    }
    if(mode == 2) translate_control();
    if(mode == 3) rotate_control();
    if(mode == 4) one_leg_lift();
    if(mode == 5) scan_mode();
    if(mode == 6) auto_navigate();
    if(mode == 7) calibrate_sensors();
    if(mode == 99) set_all_90();
  }
}


//***********************************************************************
// Mode Helpers
//***********************************************************************

// Shared mode-entry logic used by both serial commands and the switchboard.
void apply_mode(int new_mode)
{
  mode = new_mode;
  reset_position = true;
  if(mode == 5) { scan_phase = 0; scan_tick = 0; }
  if(mode == 6) {
    auto_phase = 0; auto_timer = 0; scan_tick = 0; tick = 0;
    auto_straight_streak = 0; auto_walk_ticks = 100;
    auto_turn_dir = 0; auto_turn_ticks = 75; auto_turn_joy = 128;
    memset(vfh_left,  0, sizeof(vfh_left));
    memset(vfh_right, 0, sizeof(vfh_right));
    int t[6] = {1,2,1,2,1,2}; memcpy(tripod_case, t, sizeof(t));
  }
  if(mode == 7) {
    cal_count = 0;
    Serial.println("CAL_START — place both sensors the same distance from a flat surface.");
  }
  if(mode == 1 && imu_ready) {
    float h = read_compass_heading();
    if(h >= 0) { compass_target = h; Serial.print("COMPASS_LOCK "); Serial.println(h, 1); }
  }
  if(mode == 6 && imu_ready) {
    float h = read_compass_heading();
    if(h >= 0) { compass_target = h; Serial.print("NAV_HEADING_LOCK "); Serial.println(h, 1); }
  }
  if(mode != 1 && mode != 6) compass_target = -1.0;
}

// Reads the 3 binary switches (A3/A4/A5) and returns mode 0-7.
// Triggers on falling edge of the push-button (A6).
// Holds home (mode 0) for 2 seconds, then enters the selected mode.
void process_switches()
{
  static int button_prev = HIGH;
  int button_now = digitalRead(SW_BUTTON);

  // Falling edge = button pressed
  if(button_prev == HIGH && button_now == LOW)
  {
    // Read each pin and show its raw state so wiring can be verified
    int b0 = digitalRead(SW_BIT0);   // A3
    int b1 = digitalRead(SW_BIT1);   // A4
    int b2 = digitalRead(SW_BIT2);   // A5

    Serial.print("SWITCH_PINS A3="); Serial.print(b0);
    Serial.print(" A4="); Serial.print(b1);
    Serial.print(" A5="); Serial.println(b2);

    // Build mode: open (HIGH=1) → bit is 1, GND (LOW=0) → bit is 0
    // This reads the pin value directly as the binary digit.
    int sw = 0;
    if(b0 == HIGH) sw |= 1;   // A3 open = bit 0 set
    if(b1 == HIGH) sw |= 2;   // A4 open = bit 1 set
    if(b2 == HIGH) sw |= 4;   // A5 open = bit 2 set

    Serial.print("SWITCH mode="); Serial.print(sw);
    Serial.print(" (binary "); Serial.print(b2); Serial.print(b1); Serial.print(b0);
    Serial.println(") standing 2s...");

    switch_pending_mode = sw;
    switch_countdown    = 100;   // 100 frames × 20ms = 2 seconds
    mode = 0;
    reset_position = true;
    joy_RX = 128; joy_RY = 128; joy_LX = 128; joy_LY = 128;
    compass_target = -1.0;
  }
  button_prev = button_now;

  // Count down home-standing period
  if(switch_countdown > 0)
  {
    mode = 0;
    switch_countdown--;
    if(switch_countdown == 0)
    {
      Serial.print("SWITCH go mode="); Serial.println(switch_pending_mode);
      apply_mode(switch_pending_mode);
      switch_pending_mode = -1;
    }
  }
}

//***********************************************************************
// Serial Processing
//***********************************************************************
void process_serial()
{
  while(Serial.available() > 0)
  {
    char c = (char)Serial.read();
    if(c == '\n' || c == '\r')
    {
      if(serialBuffer.length() > 0)
      {
        parse_command(serialBuffer);
        serialBuffer = "";
      }
    }
    else
    {
      serialBuffer += c;
    }
  }
}

void parse_command(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  if(cmd.startsWith("MODE"))
  {
    apply_mode(cmd.substring(5).toInt());
    Serial.print("OK MODE "); Serial.println(mode);
  }
  else if(cmd.startsWith("GAIT"))
  {
    gait = constrain(cmd.substring(5).toInt(), 0, 3);
    mode = 0;
    reset_position = true;
    int t[6] = {1,2,1,2,1,2}; memcpy(tripod_case,   t, sizeof(t));
    int r[6] = {2,6,4,1,3,5}; memcpy(ripple_case,   r, sizeof(r));
    int w[6] = {1,2,3,4,5,6}; memcpy(wave_case,     w, sizeof(w));
    int p[6] = {1,3,2,1,2,3}; memcpy(tetrapod_case, p, sizeof(p));
    tick = 0;
    Serial.print("OK GAIT "); Serial.println(gait);
  }
  else if(cmd.startsWith("SPEED"))
  {
    gait_speed = constrain(cmd.substring(6).toInt(), 0, 1);
    Serial.print("OK SPEED "); Serial.println(gait_speed);
  }
  else if(cmd.startsWith("JOY"))
  {
    String args = cmd.substring(4); args.trim();
    int v[4] = {128,128,128,128};
    int idx = 0; int start = 0;
    for(int i=0; i<=(int)args.length() && idx<4; i++)
    {
      if(i == (int)args.length() || args[i] == ' ')
      {
        if(i > start) v[idx++] = constrain(args.substring(start, i).toInt(), 0, 255);
        start = i + 1;
      }
    }
    joy_RX = v[0]; joy_RY = v[1]; joy_LX = v[2]; joy_LY = v[3];
    Serial.println("OK JOY");
  }
  else if(cmd == "CAPTURE")
  {
    capture_offsets = true;
    Serial.println("OK CAPTURE");
  }
  else if(cmd == "CLEAR")
  {
    for(leg_num=0; leg_num<6; leg_num++) { offset_X[leg_num] = 0; offset_Y[leg_num] = 0; offset_Z[leg_num] = 0; }
    leg1_IK_control = true; leg6_IK_control = true; step_height_multiplier = 1.0;
    Serial.println("OK CLEAR");
  }
  else if(cmd == "HOME")
  {
    mode = 0; reset_position = true;
    joy_RX = 128; joy_RY = 128; joy_LX = 128; joy_LY = 128;
    tick = 0;
    Serial.println("OK HOME");
  }
  else if(cmd == "STATUS")
  {
    Serial.println("--- Hexapod Status ---");
    Serial.print("Mode: "); Serial.println(mode);
    Serial.print("Gait: "); Serial.println(gait);
    Serial.print("Batt: "); Serial.println(float(batt_voltage)/100.0);
    if(imu_ready) {
      float h = read_compass_heading();
      Serial.print("Heading: "); Serial.println(h, 1);
      if(compass_target >= 0) { Serial.print("TargetHdg: "); Serial.println(compass_target, 1); }
    }
  }
  else if(cmd == "COMPASS")
  {
    if(!imu_ready) { Serial.println("Compass not available."); return; }
    float h = read_compass_heading();
    Serial.print("HEADING "); Serial.println(h, 1);
    if(compass_target >= 0) { Serial.print("TARGET  "); Serial.println(compass_target, 1); }
  }
}

//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  if((L3 < (TIBIA_LENGTH+FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH-FEMUR_LENGTH)))
  {
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3))/(2*FEMUR_LENGTH*TIBIA_LENGTH));
    theta_tibia = phi_tibia*RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia,0.0,180.0);

    gamma_femur = atan2(Z,L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH))/(2*FEMUR_LENGTH*L3));
    theta_femur = (phi_femur + gamma_femur)*RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur,0.0,180.0);

    theta_coxa = atan2(X,Y)*RAD_TO_DEG + COXA_CAL[leg_number];

    switch(leg_number)
    {
      case 0:
        if(leg1_IK_control == true) {
          theta_coxa = constrain(theta_coxa + 45.0, 0.0, 180.0);
          coxa1_servo.write(int(theta_coxa)); femur1_servo.write(int(theta_femur)); tibia1_servo.write(int(theta_tibia));
        }
        break;
      case 1:
        theta_coxa = constrain(theta_coxa + 90.0, 0.0, 180.0);
        coxa2_servo.write(int(theta_coxa)); femur2_servo.write(int(theta_femur)); tibia2_servo.write(int(theta_tibia));
        break;
      case 2:
        theta_coxa = constrain(theta_coxa + 135.0, 0.0, 180.0);
        coxa3_servo.write(int(theta_coxa)); femur3_servo.write(int(theta_femur)); tibia3_servo.write(int(theta_tibia));
        break;
      case 3:
        theta_coxa = (theta_coxa < 0) ? theta_coxa + 225.0 : theta_coxa - 135.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa4_servo.write(int(theta_coxa)); femur4_servo.write(int(theta_femur)); tibia4_servo.write(int(theta_tibia));
        break;
      case 4:
        theta_coxa = (theta_coxa < 0) ? theta_coxa + 270.0 : theta_coxa - 90.0;
        theta_coxa = constrain(theta_coxa, 0.0, 180.0);
        coxa5_servo.write(int(theta_coxa)); femur5_servo.write(int(theta_femur)); tibia5_servo.write(int(theta_tibia));
        break;
      case 5:
        if(leg6_IK_control == true) {
          theta_coxa = (theta_coxa < 0) ? theta_coxa + 315.0 : theta_coxa - 45.0;
          theta_coxa = constrain(theta_coxa, 0.0, 180.0);
          coxa6_servo.write(int(theta_coxa)); femur6_servo.write(int(theta_femur)); tibia6_servo.write(int(theta_tibia));
        }
        break;
    }
  }
}

//***********************************************************************
// Gaits
//***********************************************************************
void tripod_gait()
{
  commandedX = map(joy_RY, 0, 255, 127, -127);
  commandedY = map(joy_RX, 0, 255, -127, 127);
  commandedR = map(joy_LX, 0, 255, 127, -127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tripod_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tripod_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) tripod_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}

void wave_gait()
{
  commandedX = map(joy_RY, 0, 255, 127, -127);
  commandedY = map(joy_RX, 0, 255, -127, 127);
  commandedR = map(joy_LX, 0, 255, 127, -127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(wave_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) wave_case[leg_num] = 6;
          break;
        default:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) wave_case[leg_num]--;
          if(wave_case[leg_num] < 1) wave_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}

void ripple_gait()
{
  commandedX = map(joy_RY, 0, 255, 127, -127);
  commandedY = map(joy_RX, 0, 255, -127, 127);
  commandedR = map(joy_LX, 0, 255, 127, -127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(ripple_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 2;
          break;
        case 2:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*(numTicks+tick)/(numTicks*2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*(numTicks+tick)/(numTicks*2));
          if(tick >= numTicks-1) ripple_case[leg_num] = 3;
          break;
        default:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks/2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks/2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) ripple_case[leg_num]++;
          if(ripple_case[leg_num] > 6) ripple_case[leg_num] = 1;
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}

void tetrapod_gait()
{
  commandedX = map(joy_RY, 0, 255, 127, -127);
  commandedY = map(joy_RX, 0, 255, -127, 127);
  commandedR = map(joy_LX, 0, 255, 127, -127);

  if((abs(commandedX) > 15) || (abs(commandedY) > 15) || (abs(commandedR) > 15) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0);
    for(leg_num=0; leg_num<6; leg_num++)
    {
      compute_amplitudes();
      switch(tetrapod_case[leg_num])
      {
        case 1:
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX*cos(M_PI*tick/numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY*cos(M_PI*tick/numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ)*sin(M_PI*tick/numTicks);
          if(tick >= numTicks-1) tetrapod_case[leg_num] = 2;
          break;
        default:
          current_X[leg_num] = current_X[leg_num] - amplitudeX/numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY/numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if(tick >= numTicks-1) { tetrapod_case[leg_num]++; if(tetrapod_case[leg_num]>3) tetrapod_case[leg_num]=1; }
          break;
      }
    }
    if(tick < numTicks-1) tick++; else tick = 0;
  }
}

//***********************************************************************
// Compass Heading
//***********************************************************************
// Returns heading in degrees [0, 360), or -1 if IMU not ready.
// Applies hard-iron/soft-iron calibration and a low-pass filter.
// Uses sin/cos accumulation so the filter handles the 0°/360° wrap correctly.
// ALPHA = 0.1 → ~130 ms time constant at 50 Hz (tune lower for more smoothing).
float read_compass_heading()
{
  if(!imu_ready) return -1;
  myICM.getAGMT();
  float mx = (myICM.magX() - magCal.offsetX) * magCal.scaleX;
  float my = (myICM.magY() - magCal.offsetY) * magCal.scaleY;

  float raw_rad = atan2(my, mx);
  const float ALPHA = 0.1f;

  if(!compass_filter_init) {
    compass_sin_acc   = sin(raw_rad);
    compass_cos_acc   = cos(raw_rad);
    compass_filter_init = true;
  } else {
    compass_sin_acc = ALPHA * sin(raw_rad) + (1.0f - ALPHA) * compass_sin_acc;
    compass_cos_acc = ALPHA * cos(raw_rad) + (1.0f - ALPHA) * compass_cos_acc;
  }

  float heading = atan2(compass_sin_acc, compass_cos_acc) * RAD_TO_DEG;
  if(heading < 0) heading += 360.0f;
  return heading;
}

//***********************************************************************
// Calculation Helpers
//***********************************************************************
void compute_strides()
{
  strideX = 90*commandedX/127;
  strideY = 90*commandedY/127;
  strideR = 35*commandedR/127;

  // Compass heading lock: P-controller keeps robot on its locked heading.
  if(imu_ready && compass_target >= 0)
  {
    float cur = read_compass_heading();
    if(cur >= 0)
    {
      if(mode == 6)
      {
        // Auto-navigate: apply correction only while walking straight (phase 0/1).
        // During obstacle-avoidance turns (phase 10) let the VFH steer freely.
        // Never update compass_target — always try to return to the original heading.
        if(auto_phase != 10)
        {
          float err = cur - compass_target;
          if(err >  180) err -= 360;
          if(err < -180) err += 360;
          strideR = constrain(strideR - err * 0.15f, -35.0f, 35.0f);
        }
      }
      else
      {
        // Manual walk (mode 1): track heading while user rotates, lock when released.
        bool user_rotating = (abs(joy_LX - 128) > 20);
        if(user_rotating)
        {
          compass_target = cur;
        }
        else
        {
          float err = cur - compass_target;
          if(err >  180) err -= 360;
          if(err < -180) err += 360;
          strideR = constrain(strideR - err * 0.15f, -35.0f, 35.0f);
        }
      }
    }
  }

  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));
  duration = (gait_speed == 2) ? 720 : (gait_speed == 0) ? 1080 : 3240;
}

void compute_amplitudes()
{
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
  rotOffsetX = totalY*sinRotZ + totalX*cosRotZ - totalX;
  rotOffsetY = totalY*cosRotZ - totalX*sinRotZ - totalY;
  amplitudeX = constrain(((strideX + rotOffsetX)/2.0), -50, 50);
  amplitudeY = constrain(((strideY + rotOffsetY)/2.0), -50, 50);
  if(abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}

//***********************************************************************
// Controls
//***********************************************************************
void translate_control()
{
  translateX = map(joy_RY, 0, 255, -2*TRAVEL, 2*TRAVEL);
  translateY = map(joy_RX, 0, 255, 2*TRAVEL, -2*TRAVEL);
  translateZ = (joy_LY > 127) ? map(joy_LY, 128, 255, 0, TRAVEL) : map(joy_LY, 0, 127, -3*TRAVEL, 0);

  for(leg_num=0; leg_num<6; leg_num++) {
    current_X[leg_num] = HOME_X[leg_num] + translateX;
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;
  }

  if(capture_offsets == true) {
    for(leg_num=0; leg_num<6; leg_num++) {
      offset_X[leg_num] += translateX; offset_Y[leg_num] += translateY; offset_Z[leg_num] += translateZ;
      current_X[leg_num] = HOME_X[leg_num]; current_Y[leg_num] = HOME_Y[leg_num]; current_Z[leg_num] = HOME_Z[leg_num];
    }
    capture_offsets = false; mode = 0;
  }
}

void rotate_control()
{
  sinRotX = sin((map(joy_RX, 0,255, A12DEG,-A12DEG))/1000000.0);
  cosRotX = cos((map(joy_RX, 0,255, A12DEG,-A12DEG))/1000000.0);
  sinRotY = sin((map(joy_RY, 0,255, A12DEG,-A12DEG))/1000000.0);
  cosRotY = cos((map(joy_RY, 0,255, A12DEG,-A12DEG))/1000000.0);
  sinRotZ = sin((map(joy_LX, 0,255, -A30DEG,A30DEG))/1000000.0);
  cosRotZ = cos((map(joy_LX, 0,255, -A30DEG,A30DEG))/1000000.0);
  translateZ = (joy_LY > 127) ? map(joy_LY, 128,255, 0, TRAVEL) : map(joy_LY, 0,127, -3*TRAVEL, 0);

  for(int ln=0; ln<6; ln++) {
    totalX = HOME_X[ln] + BODY_X[ln]; totalY = HOME_Y[ln] + BODY_Y[ln]; totalZ = HOME_Z[ln] + BODY_Z[ln];
    rotOffsetX =  totalX*cosRotY*cosRotZ + totalY*sinRotX*sinRotY*cosRotZ + totalY*cosRotX*sinRotZ - totalZ*cosRotX*sinRotY*cosRotZ + totalZ*sinRotX*sinRotZ - totalX;
    rotOffsetY = -totalX*cosRotY*sinRotZ - totalY*sinRotX*sinRotY*sinRotZ + totalY*cosRotX*cosRotZ + totalZ*cosRotX*sinRotY*sinRotZ + totalZ*sinRotX*cosRotZ - totalY;
    rotOffsetZ =  totalX*sinRotY - totalY*sinRotX*cosRotY + totalZ*cosRotX*cosRotY - totalZ;

    current_X[ln] = HOME_X[ln] + rotOffsetX;
    current_Y[ln] = HOME_Y[ln] + rotOffsetY;
    current_Z[ln] = HOME_Z[ln] + rotOffsetZ + translateZ;

    if(capture_offsets == true) {
      offset_X[ln] += rotOffsetX; offset_Y[ln] += rotOffsetY; offset_Z[ln] += (rotOffsetZ + translateZ);
      current_X[ln] = HOME_X[ln]; current_Y[ln] = HOME_Y[ln]; current_Z[ln] = HOME_Z[ln];
    }
  }
  if(capture_offsets == true) { capture_offsets = false; mode = 0; }
}

void one_leg_lift()
{
  if(leg1_IK_control == true) {
    leg1_coxa = coxa1_servo.read(); leg1_femur = femur1_servo.read(); leg1_tibia = tibia1_servo.read();
    leg1_IK_control = false;
  }
  if(leg6_IK_control == true) {
    leg6_coxa = coxa6_servo.read(); leg6_femur = femur6_servo.read(); leg6_tibia = tibia6_servo.read();
    leg6_IK_control = false;
  }

  temp = map(joy_RX, 0, 255, 45, -45);
  coxa1_servo.write(constrain(int(leg1_coxa+temp), 45, 135));
  if(joy_RY < 117) {
    temp = map(joy_RY, 116, 0, 0, 24);
    femur1_servo.write(constrain(int(leg1_femur+temp), 0, 170));
    tibia1_servo.write(constrain(int(leg1_tibia+4*temp), 0, 170));
  } else {
    z_height_right = map(constrain(joy_RY, 140, 255), 140, 255, 1, 8);
  }

  temp = map(joy_LX, 0, 255, 45, -45);
  coxa6_servo.write(constrain(int(leg6_coxa+temp), 45, 135));
  if(joy_LY < 117) {
    temp = map(joy_LY, 116, 0, 0, 24);
    femur6_servo.write(constrain(int(leg6_femur+temp), 0, 170));
    tibia6_servo.write(constrain(int(leg6_tibia+4*temp), 0, 170));
  } else {
    z_height_left = map(constrain(joy_LY, 140, 255), 140, 255, 1, 8);
  }

  int final_z = (z_height_left > z_height_right) ? z_height_left : z_height_right;
  LED_Bar((batt_LEDs > 3 ? 0 : 1), final_z);

  if(capture_offsets == true) {
    step_height_multiplier = 1.0 + ((final_z - 1.0) / 3.0);
    capture_offsets = false;
  }
}

void auto_navigate()
{
  // State machine:
  //  0  = walk forward (tripod)   1  = stop/settle (tripod winds down)
  //  2  = lift left leg  3  = sweep left   4  = lower left
  //  5  = lift right leg 6  = sweep right  7  = lower right
  //  8  = decide         9  = lower body   10 = turn (wave, arcing)
  //
  // Walk duration grows with consecutive straight decisions (auto_straight_streak).
  // Turn duration and sharpness scale with obstacle proximity.
  // After any turn the streak resets, so the first walk back is always short.

  const int   AUTO_OBSTACLE_MM  = 400;  // obstacle threshold (mm)
  const int   BASE_WALK_TICKS   = 100;  // minimum walk duration (~2s at 50Hz)
  const int   WALK_INCREMENT    = 25;   // extra frames per consecutive straight (~0.5s each)
  const int   MAX_WALK_TICKS    = 400;  // cap walk duration (~8s)
  const int   MIN_TURN_TICKS    = 40;   // shortest turn (~0.8s, obstacle near threshold)
  const int   MAX_TURN_TICKS    = 150;  // longest turn (~3s, obstacle very close)
  const int   MIN_JOY_OFFSET    = 28;   // gentlest rotation (obstacle near threshold)
  const int   MAX_JOY_OFFSET    = 88;   // sharpest rotation (obstacle very close, ~100mm)
  const float NAV_WALK_Z_OFFSET = 20.0; // extra body height (mm) during navigate walking for obstacle clearance

  float alpha, sweep_angle;
  int   dist;

  switch(auto_phase)
  {
    //--- Walk forward using fast tripod gait -------------------------
    case 0:
      joy_RY = 50; joy_RX = 128; joy_LX = 128;  // joy_RY=50 → near-max stride (~50mm)
      gait_speed = 2;                             // sprint: 720ms cycle vs normal 1080ms
      tripod_gait();
      for(int ln=0; ln<6; ln++) current_Z[ln] -= NAV_WALK_Z_OFFSET;
      if(++auto_timer >= auto_walk_ticks) { auto_timer = 0; auto_phase = 1; }
      break;

    //--- Let tripod gait finish its cycle, then reset ----------------
    case 1:
      joy_RY = 128; joy_RX = 128; joy_LX = 128;
      gait_speed = 0;                             // restore normal speed for wave gait turns
      tripod_gait();
      for(int ln=0; ln<6; ln++) current_Z[ln] -= NAV_WALK_Z_OFFSET;
      if(tick == 0)
      {
        reset_position = true;
        scan_tick      = 0;
        memset(vfh_left,  0, sizeof(vfh_left));
        memset(vfh_right, 0, sizeof(vfh_right));
        auto_phase     = 2;
      }
      break;

    //--- Left leg scan (leg 5) phases 2-4 ----------------------------
    case 2:  // lift left leg to sweep start
      for(int ln=0; ln<6; ln++) { current_X[ln]=HOME_X[ln]; current_Y[ln]=HOME_Y[ln]; current_Z[ln]=SCAN_STANCE_Z; }
      alpha        = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[5] = HOME_X[5]     + alpha * (0.0          - HOME_X[5]);
      current_Y[5] = HOME_Y[5]     + alpha * (-SCAN_RADIUS  - HOME_Y[5]);
      current_Z[5] = SCAN_STANCE_Z + alpha * (SCAN_RAISED_Z - SCAN_STANCE_Z);
      if(scan_tick >= SCAN_LIFT_TICKS-1) { scan_tick=0; auto_phase=3; } else scan_tick++;
      break;

    case 3:  // sweep left leg from left (90°) to front (0°)
      for(int ln=0; ln<5; ln++) { current_X[ln]=HOME_X[ln]; current_Y[ln]=HOME_Y[ln]; current_Z[ln]=SCAN_STANCE_Z; }
      sweep_angle  = (M_PI/2.0) * (1.0 - (float)scan_tick / (SCAN_SWEEP_TICKS-1));
      current_X[5] =  SCAN_RADIUS * cos(sweep_angle);
      current_Y[5] = -SCAN_RADIUS * sin(sweep_angle);
      current_Z[5] = SCAN_RAISED_Z;
      if(scan_tick == 0 || scan_tick == 16 || scan_tick == 33 || scan_tick == 50 ||
         scan_tick == 66 || scan_tick == 83 || scan_tick == SCAN_SWEEP_TICKS-1) {
        int vfh_idx = (int)((float)scan_tick / (SCAN_SWEEP_TICKS-1) * 6.0 + 0.5);
        dist = read_ultrasonic(TRIG_LEFT, ECHO_LEFT);
        vfh_left[vfh_idx] = dist;
        Serial.print("SCAN_L "); Serial.print(int(degrees(sweep_angle))); Serial.print(" "); Serial.println(dist);
      }
      if(scan_tick >= SCAN_SWEEP_TICKS-1) { scan_tick=0; auto_phase=4; } else scan_tick++;
      break;

    case 4:  // lower left leg back to tall stance
      for(int ln=0; ln<5; ln++) { current_X[ln]=HOME_X[ln]; current_Y[ln]=HOME_Y[ln]; current_Z[ln]=SCAN_STANCE_Z; }
      alpha        = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[5] = SCAN_RADIUS  + alpha * (HOME_X[5] - SCAN_RADIUS);
      current_Y[5] = 0.0          + alpha * (HOME_Y[5] - 0.0);
      current_Z[5] = SCAN_RAISED_Z + alpha * (SCAN_STANCE_Z - SCAN_RAISED_Z);
      if(scan_tick >= SCAN_LIFT_TICKS-1) { scan_tick=0; auto_phase=5; } else scan_tick++;
      break;

    //--- Right leg scan (leg 0) phases 5-7 ---------------------------
    case 5:  // lift right leg to sweep start
      for(int ln=0; ln<6; ln++) { current_X[ln]=HOME_X[ln]; current_Y[ln]=HOME_Y[ln]; current_Z[ln]=SCAN_STANCE_Z; }
      alpha        = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[0] = HOME_X[0]     + alpha * (0.0           - HOME_X[0]);
      current_Y[0] = HOME_Y[0]     + alpha * (SCAN_RADIUS   - HOME_Y[0]);
      current_Z[0] = SCAN_STANCE_Z + alpha * (SCAN_RAISED_Z - SCAN_STANCE_Z);
      if(scan_tick >= SCAN_LIFT_TICKS-1) { scan_tick=0; auto_phase=6; } else scan_tick++;
      break;

    case 6:  // sweep right leg from right (90°) to front (0°)
      for(int ln=1; ln<6; ln++) { current_X[ln]=HOME_X[ln]; current_Y[ln]=HOME_Y[ln]; current_Z[ln]=SCAN_STANCE_Z; }
      sweep_angle  = (M_PI/2.0) * (1.0 - (float)scan_tick / (SCAN_SWEEP_TICKS-1));
      current_X[0] = SCAN_RADIUS * cos(sweep_angle);
      current_Y[0] = SCAN_RADIUS * sin(sweep_angle);
      current_Z[0] = SCAN_RAISED_Z;
      if(scan_tick == 0 || scan_tick == 16 || scan_tick == 33 || scan_tick == 50 ||
         scan_tick == 66 || scan_tick == 83 || scan_tick == SCAN_SWEEP_TICKS-1) {
        int vfh_idx = (int)((float)scan_tick / (SCAN_SWEEP_TICKS-1) * 6.0 + 0.5);
        dist = read_ultrasonic(TRIG_RIGHT, ECHO_RIGHT);
        if(dist > 0) dist = constrain(dist + sensor_right_offset, 1, 9999);
        vfh_right[vfh_idx] = dist;
        Serial.print("SCAN_R "); Serial.print(int(degrees(sweep_angle))); Serial.print(" "); Serial.println(dist);
      }
      if(scan_tick >= SCAN_SWEEP_TICKS-1) { scan_tick=0; auto_phase=7; } else scan_tick++;
      break;

    case 7:  // lower right leg back to tall stance
      for(int ln=1; ln<6; ln++) { current_X[ln]=HOME_X[ln]; current_Y[ln]=HOME_Y[ln]; current_Z[ln]=SCAN_STANCE_Z; }
      alpha        = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[0] = SCAN_RADIUS  + alpha * (HOME_X[0] - SCAN_RADIUS);
      current_Y[0] = 0.0          + alpha * (HOME_Y[0] - 0.0);
      current_Z[0] = SCAN_RAISED_Z + alpha * (SCAN_STANCE_Z - SCAN_RAISED_Z);
      if(scan_tick >= SCAN_LIFT_TICKS-1) { scan_tick=0; auto_timer=0; auto_phase=8; } else scan_tick++;
      break;

    //--- VFH-lite: build polar histogram, pick clearest direction ----
    case 8:
      {
        // --- Raw scan data dump ---
        Serial.print("RAW_L mm:");
        for(int i = 0; i < 7; i++) { Serial.print(" "); Serial.print(vfh_left[i]); }
        Serial.println();
        Serial.print("RAW_R mm:");
        for(int i = 0; i < 7; i++) { Serial.print(" "); Serial.print(vfh_right[i]); }
        Serial.println();

        // Polar certainty histogram: 13 slots, 15° each, covering -90° to +90°
        // slot 0=-90°(hard left) ... slot 6=0°(forward) ... slot 12=+90°(hard right)
        // vfh_left[i]:  i=0→90°L(slot 0), i=1→75°L(slot 1), ..., i=6→0°(slot 6)
        // vfh_right[i]: i=0→90°R(slot 12), i=1→75°R(slot 11), ..., i=6→0°(slot 6)
        // Certainty = 0 (clear) to 1.0 (fully blocked); dist<=0 treated as clear (no echo=far)
        float hist[13];
        for(int i = 0; i < 13; i++) hist[i] = 0.0;

        // --- Build histogram and print cert values slot by slot ---
        Serial.println("CERT  slot  dist_L  cert_L  dist_R  cert_R");
        for(int i = 0; i < 7; i++) {
          float cert_l = (vfh_left[i]  > 0 && vfh_left[i]  < AUTO_OBSTACLE_MM) ?
                          (float)(AUTO_OBSTACLE_MM - vfh_left[i])  / AUTO_OBSTACLE_MM : 0.0;
          float cert_r = (vfh_right[i] > 0 && vfh_right[i] < AUTO_OBSTACLE_MM) ?
                          (float)(AUTO_OBSTACLE_MM - vfh_right[i]) / AUTO_OBSTACLE_MM : 0.0;

          Serial.print("CERT  i="); Serial.print(i);
          Serial.print("  L="); Serial.print(vfh_left[i]);
          Serial.print(" cL="); Serial.print(int(cert_l * 100));
          Serial.print("  R="); Serial.print(vfh_right[i]);
          Serial.print(" cR="); Serial.println(int(cert_r * 100));

          if(i == 6) {
            hist[6] = (cert_l + cert_r) / 2.0;     // forward: average both sensors
          } else {
            hist[i]      = cert_l;                  // left side:  slots 0,1,2,3,4,5
            hist[12 - i] = cert_r;                  // right side: slots 12,11,10,9,8,7
          }
        }

        // Smooth over a 3-slot (45°) window; find slot with lowest total certainty.
        // Seed with the forward slot (6) so ties always prefer going straight.
        float best_score = hist[5] + hist[6] + hist[7];
        int   best_slot  = 6;
        for(int i = 1; i <= 11; i++) {
          float score = hist[i-1] + hist[i] + hist[i+1];
          if(score < best_score) { best_score = score; best_slot = i; }
        }

        // Convert best slot to signed angle (-90° to +90°, negative=left, positive=right)
        int best_angle_deg = (best_slot - 6) * 15;

        // Update direction and streak
        if(best_angle_deg == 0) {
          auto_turn_dir = 0;
          auto_straight_streak = min(auto_straight_streak + 1, 12);
        } else {
          auto_turn_dir = (best_angle_deg < 0) ? -1 : 1;
          auto_straight_streak = 0;
        }

        // Adaptive walk length
        auto_walk_ticks = constrain(BASE_WALK_TICKS + auto_straight_streak * WALK_INCREMENT,
                                    BASE_WALK_TICKS, MAX_WALK_TICKS);

        // Turn parameters scale with angle magnitude
        if(auto_turn_dir != 0) {
          int abs_angle = constrain(abs(best_angle_deg), 15, 90);
          auto_turn_ticks = map(abs_angle, 15, 90, MIN_TURN_TICKS, MAX_TURN_TICKS);
          int joy_offset  = map(abs_angle, 15, 90, MIN_JOY_OFFSET, MAX_JOY_OFFSET);
          auto_turn_joy   = (auto_turn_dir == -1) ? (128 - joy_offset) : (128 + joy_offset);
        }

        // Print histogram (as 0-100 integers) and decision
        // Format: L=left side, [FWD], R=right side
        Serial.print("HIST L[");
        for(int i = 0; i <= 5; i++) { Serial.print(int(hist[i]*100)); if(i<5) Serial.print(","); }
        Serial.print("] FWD["); Serial.print(int(hist[6]*100));
        Serial.print("] R[");
        for(int i = 7; i <= 12; i++) { Serial.print(int(hist[i]*100)); if(i<12) Serial.print(","); }
        Serial.println("]");

        Serial.print("DECIDE best=slot"); Serial.print(best_slot);
        Serial.print("("); Serial.print(best_angle_deg); Serial.print("deg)");
        Serial.print(" dir=");
        Serial.print(auto_turn_dir == 0 ? "STRAIGHT" : (auto_turn_dir == -1 ? "LEFT" : "RIGHT"));
        if(auto_turn_dir != 0) {
          Serial.print(" joy_LX="); Serial.print(auto_turn_joy);
          Serial.print(" ticks="); Serial.print(auto_turn_ticks);
        }
        Serial.print(" streak="); Serial.print(auto_straight_streak);
        Serial.print(" nextWalk="); Serial.print(auto_walk_ticks * FRAME_TIME_MS / 1000.0, 1);
        Serial.println("s");

        auto_timer = 0;
        auto_phase = 9;
      }
      break;

    //--- Lower body back to walk height, reset appropriate gait ------
    case 9:
      {
        alpha = (float)auto_timer / SCAN_LIFT_TICKS;
        for(int ln=0; ln<6; ln++) {
          current_X[ln] = HOME_X[ln];
          current_Y[ln] = HOME_Y[ln];
          current_Z[ln] = SCAN_STANCE_Z + alpha * ((HOME_Z[ln] - NAV_WALK_Z_OFFSET) - SCAN_STANCE_Z);
        }
        if(++auto_timer >= SCAN_LIFT_TICKS) {
          reset_position = true;
          tick           = 0;
          auto_timer     = 0;
          if(auto_turn_dir == 0) {
            int t[6] = {1,2,1,2,1,2}; memcpy(tripod_case, t, sizeof(t));
            auto_phase = 0;
          } else {
            int w[6] = {1,2,3,4,5,6}; memcpy(wave_case, w, sizeof(w));
            auto_phase = 10;
          }
        }
      }
      break;

    //--- Arc turn: wave gait with slight forward to curve around obstacle
    case 10:
      joy_RY = 105;                                        // ~15% forward so robot arcs, not spins
      joy_RX = 128;
      joy_LX = auto_turn_joy;
      wave_gait();
      for(int ln=0; ln<6; ln++) current_Z[ln] -= NAV_WALK_Z_OFFSET;
      if(++auto_timer >= auto_turn_ticks) {
        joy_LX = 128; joy_RY = 128;
        auto_timer = 0;
        tick       = 0;
        int t[6] = {1,2,1,2,1,2}; memcpy(tripod_case, t, sizeof(t));
        auto_phase = 0;
      }
      break;
  }
}

void calibrate_sensors()
{
  // Tall stance with both front legs (0 and 5) at their 0-degree scan position:
  // X=SCAN_RADIUS, Y=0, Z=SCAN_RAISED_Z — exactly where the sweep ends pointing forward.
  // Support legs use SCAN_STANCE_Z (same as navigate mode).
  for(int ln = 0; ln < 6; ln++)
  {
    current_X[ln] = HOME_X[ln];
    current_Y[ln] = HOME_Y[ln];
    current_Z[ln] = SCAN_STANCE_Z;
  }
  current_X[0] = SCAN_RADIUS;  current_Y[0] = 0.0;  current_Z[0] = SCAN_RAISED_Z;
  current_X[5] = SCAN_RADIUS;  current_Y[5] = 0.0;  current_Z[5] = SCAN_RAISED_Z;

  if(cal_count > 0) return;   // burst already completed, waiting for mode to reset
  cal_count = 1;              // mark as running so we only burst once

  // Take 20 readings per sensor; discard first 10 (settling), average last 10.
  const int BURST = 20;
  const int SKIP  = 10;
  long sum_l = 0, sum_r = 0;
  int  valid_l = 0, valid_r = 0;

  for(int n = 0; n < BURST; n++)
  {
    int dl = read_ultrasonic(TRIG_LEFT,  ECHO_LEFT);
    int dr = read_ultrasonic(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("CAL_L "); Serial.print(n); Serial.print(" "); Serial.println(dl);
    Serial.print("CAL_R "); Serial.print(n); Serial.print(" "); Serial.println(dr);

    if(n >= SKIP)
    {
      if(dl > 0) { sum_l += dl; valid_l++; }
      if(dr > 0) { sum_r += dr; valid_r++; }
    }

    delay(30);  // 30ms between pings — HC-SR04 needs ~60ms per sensor but we alternate L/R
  }

  int avg_left  = (valid_l > 0) ? (int)(sum_l / valid_l) : -1;
  int avg_right = (valid_r > 0) ? (int)(sum_r / valid_r) : -1;

  Serial.println("CAL_DONE");
  Serial.print("CAL_AVG_L "); Serial.println(avg_left);
  Serial.print("CAL_AVG_R "); Serial.println(avg_right);

  if(avg_left > 0 && avg_right > 0)
  {
    sensor_right_offset = avg_left - avg_right;
    Serial.print("CAL_OFFSET_R "); Serial.println(sensor_right_offset);
    Serial.println("Right sensor offset saved. Navigate mode will apply it automatically.");
  }
  else
  {
    Serial.println("CAL_ERROR: one or both sensors returned no valid readings.");
  }

  mode = 0;
}

// Returns distance in mm, or -1 on timeout (no echo within ~2.5m)
int read_ultrasonic(int trig_pin, int echo_pin)
{
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  long duration = pulseIn(echo_pin, HIGH, 15000UL);  // 15ms timeout
  if(duration == 0) return -1;
  return int(duration * 0.1715);  // microseconds to mm (speed of sound / 2)
}

void scan_mode()
{
  // All positions are passed through IK - no direct servo writes.
  // Leg 0 (front-right) sweeps right-side to front; leg 2 (front-left) mirrors it.
  // Serial output during sweep: "SCAN_R <angle_deg> <distance>"
  //                             "SCAN_L <angle_deg> <distance>"
  // Replace the 0 placeholder with your ultrasonic sensor read.

  float alpha, sweep_angle;

  // Stand tall: support legs extended down so body rises; scanning leg overridden per phase
  for(int ln = 0; ln < 6; ln++)
  {
    current_X[ln] = HOME_X[ln];
    current_Y[ln] = HOME_Y[ln];
    current_Z[ln] = SCAN_STANCE_Z;
  }

  switch(scan_phase)
  {
    case 0:  // lift right leg (leg 0) from tall stance to right-side sweep start (X=0, Y=RADIUS)
      alpha = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[0] = HOME_X[0]  + alpha * (0.0           - HOME_X[0]);
      current_Y[0] = HOME_Y[0]  + alpha * (SCAN_RADIUS   - HOME_Y[0]);
      current_Z[0] = SCAN_STANCE_Z + alpha * (SCAN_RAISED_Z - SCAN_STANCE_Z);
      if(scan_tick >= SCAN_LIFT_TICKS - 1) { scan_tick = 0; scan_phase = 1; }
      else scan_tick++;
      break;

    case 1:  // sweep right leg from right (90 deg) to front (0 deg)
      sweep_angle  = (M_PI / 2.0) * (1.0 - (float)scan_tick / (SCAN_SWEEP_TICKS - 1));
      current_X[0] = SCAN_RADIUS * cos(sweep_angle);
      current_Y[0] = SCAN_RADIUS * sin(sweep_angle);
      current_Z[0] = SCAN_RAISED_Z;
      // sample at 90°,75°,60°,45°,30°,15°,0° (ticks 0,16,33,50,66,83,99)
      if(scan_tick == 0 || scan_tick == 16 || scan_tick == 33 || scan_tick == 50 ||
         scan_tick == 66 || scan_tick == 83 || scan_tick == SCAN_SWEEP_TICKS-1) {
        Serial.print("SCAN_R ");
        Serial.print(int(degrees(sweep_angle)));
        Serial.print(" ");
        Serial.println(read_ultrasonic(TRIG_RIGHT, ECHO_RIGHT));
      }
      if(scan_tick >= SCAN_SWEEP_TICKS - 1) { scan_tick = 0; scan_phase = 2; }
      else scan_tick++;
      break;

    case 2:  // lower right leg from sweep end back to tall stance position
      alpha = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[0] = SCAN_RADIUS  + alpha * (HOME_X[0] - SCAN_RADIUS);
      current_Y[0] = 0.0          + alpha * (HOME_Y[0] - 0.0);
      current_Z[0] = SCAN_RAISED_Z + alpha * (SCAN_STANCE_Z - SCAN_RAISED_Z);
      if(scan_tick >= SCAN_LIFT_TICKS - 1) { scan_tick = 0; scan_phase = 3; }
      else scan_tick++;
      break;

    case 3:  // lift left leg (leg 5) from tall stance to left-side sweep start (X=0, Y-)
      alpha = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[5] = HOME_X[5]  + alpha * (0.0           - HOME_X[5]);
      current_Y[5] = HOME_Y[5]  + alpha * (-SCAN_RADIUS   - HOME_Y[5]);
      current_Z[5] = SCAN_STANCE_Z + alpha * (SCAN_RAISED_Z - SCAN_STANCE_Z);
      if(scan_tick >= SCAN_LIFT_TICKS - 1) { scan_tick = 0; scan_phase = 4; }
      else scan_tick++;
      break;

    case 4:  // sweep left leg from left (Y-) to front (X+) - Y-mirror of right sweep
      sweep_angle  = (M_PI / 2.0) * (1.0 - (float)scan_tick / (SCAN_SWEEP_TICKS - 1));
      current_X[5] =  SCAN_RADIUS * cos(sweep_angle);
      current_Y[5] = -SCAN_RADIUS * sin(sweep_angle);
      current_Z[5] = SCAN_RAISED_Z;
      // sample at 90°,75°,60°,45°,30°,15°,0° (ticks 0,16,33,50,66,83,99)
      if(scan_tick == 0 || scan_tick == 16 || scan_tick == 33 || scan_tick == 50 ||
         scan_tick == 66 || scan_tick == 83 || scan_tick == SCAN_SWEEP_TICKS-1) {
        Serial.print("SCAN_L ");
        Serial.print(int(degrees(sweep_angle)));
        Serial.print(" ");
        Serial.println(read_ultrasonic(TRIG_LEFT, ECHO_LEFT));
      }
      if(scan_tick >= SCAN_SWEEP_TICKS - 1) { scan_tick = 0; scan_phase = 5; }
      else scan_tick++;
      break;

    case 5:  // lower left leg from sweep end (X=RADIUS, Y=0) back to home
      alpha = (float)scan_tick / SCAN_LIFT_TICKS;
      current_X[5] = SCAN_RADIUS  + alpha * (HOME_X[5] - SCAN_RADIUS);
      current_Y[5] = 0.0          + alpha * (HOME_Y[5] - 0.0);
      current_Z[5] = SCAN_RAISED_Z + alpha * (SCAN_STANCE_Z - SCAN_RAISED_Z);
      if(scan_tick >= SCAN_LIFT_TICKS - 1)
      {
        Serial.println("SCAN_DONE");
        scan_tick = 0; scan_phase = 0;  // loop back for continuous scanning
      }
      else scan_tick++;
      break;
  }
}

void set_all_90()
{
  coxa1_servo.write(90+COXA_CAL[0]); femur1_servo.write(90+FEMUR_CAL[0]); tibia1_servo.write(90+TIBIA_CAL[0]);
  coxa2_servo.write(90+COXA_CAL[1]); femur2_servo.write(90+FEMUR_CAL[1]); tibia2_servo.write(90+TIBIA_CAL[1]);
  coxa3_servo.write(90+COXA_CAL[2]); femur3_servo.write(90+FEMUR_CAL[2]); tibia3_servo.write(90+TIBIA_CAL[2]);
  coxa4_servo.write(90+COXA_CAL[3]); femur4_servo.write(90+FEMUR_CAL[3]); tibia4_servo.write(90+TIBIA_CAL[3]);
  coxa5_servo.write(90+COXA_CAL[4]); femur5_servo.write(90+FEMUR_CAL[4]); tibia5_servo.write(90+TIBIA_CAL[4]);
  coxa6_servo.write(90+COXA_CAL[5]); femur6_servo.write(90+FEMUR_CAL[5]); tibia6_servo.write(90+TIBIA_CAL[5]);
}

//***********************************************************************
// Battery and Utility
//***********************************************************************
void battery_monitor()
{
  batt_voltage_sum -= batt_voltage_array[batt_voltage_index];
  batt_voltage_array[batt_voltage_index] = map(analogRead(BATT_VOLTAGE), 0, 1023, 0, 1497);
  batt_voltage_sum += batt_voltage_array[batt_voltage_index];
  batt_voltage_index = (batt_voltage_index + 1) % 50;
  batt_voltage = batt_voltage_sum / 50;
  batt_LEDs = map(constrain(batt_voltage, 1020, 1230), 1020, 1230, 1, 8);
  LED_Bar((batt_LEDs > 3 ? 1 : 0), batt_LEDs);
}

void LED_Bar(int color, int count)
{
  for(int i=0; i<8; i++) {
    if(i < count) {
      digitalWrite((RED_LED1+(4*i)), color == 0 ? HIGH : LOW);
      digitalWrite((GREEN_LED1+(4*i)), color == 1 ? HIGH : LOW);
    } else {
      digitalWrite((RED_LED1+(4*i)), LOW); digitalWrite((GREEN_LED1+(4*i)), LOW);
    }
  }
}

void print_debug()
{
  Serial.print(millis() - previousTime);
  Serial.print(",");
  Serial.println(float(batt_voltage)/100.0);
}