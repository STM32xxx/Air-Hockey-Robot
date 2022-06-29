// AHR AIR HOCKEY ROBOT PROJECT

// USER CONFIGURATIONS HERE
// ROBOT DIMENSIONS, MAX SPEED, MAX ACCELERATION, CALIBRATION

// THIS VALUES DEPENDS ON THE VOLTAGE, MOTORS, PULLEYS AND ROBOT CONSTRUCTION
// RECOMMENDED VALUES FOR 12V POWER SUPPLY
#define MIN_ACCEL_X 60
#define MAX_ACCEL_X 320   //360 //300//320      // Maximun motor acceleration in (steps/seg2)/1000
#define MIN_ACCEL_Y 60
#define MAX_ACCEL_Y 180    //140//220 
#define MAX_SPEED_X 32000     //max 25000 for 12V   // Maximun speed in steps/seg
#define MAX_SPEED_Y 28000

// This is for the Accel ramp implementation (to smooth the intial acceleration), simplified S-profile
#define ACCEL_RAMP_MIN 2500  // The S profile is generated up to this speed
#define ACCEL_RAMP_MAX 10000

// UNCOMMENT THIS LINES TO INVERT MOTORS
#define INVERT_X_AXIS 1
//#define INVERT_Y_AXIS 1  //Y-LEFT
//#define INVERT_Z_AXIS 1  //Y_RIGHT

// Geometric calibration.
// This depends on the pulley teeth. For 42 teeth GT2 => 19, for 40 teeth GT2 => 20, for 16 teeth T5 => 20
#define X_AXIS_STEPS_PER_UNIT 40    // With 42 teeth GT2 pulley and 1/8 microstepping on drivers
#define Y_AXIS_STEPS_PER_UNIT 40    // 200*8 = 1600 steps/rev = 1600/42teeth*2mm = 19.047, using 19 is an error of 1mm every 40cm not too much! and we use int operations...

// Absolute Min and Max robot positions in mm (measured from center of robot pusher)
#define ROBOT_MIN_X 0
#define ROBOT_MIN_Y 0
#define ROBOT_MAX_X 322
#define ROBOT_MAX_Y 400

// This is the center of the table. All units in milimeters
#define ROBOT_CENTER_X 161   // Center of robot. The table is 600x1000mm, so center is 300,500
#define ROBOT_CENTER_Y 0/////////////////////////////////////

// Robot defense and attack lines
#define ROBOT_DEFENSE_POSITION 20 //（单位：mm）
#define ROBOT_DEFENSE_ATTACK_POSITION 110 //最大180，（单位：mm）

//修正电动机的缺失步骤
//如果你不想做修正的话，将宏定义改为0
#define CORRECT_MISSING_STEPS_X 1
#define CORRECT_MISSING_STEPS_Y 0

#define MISSING_STEPS_MAX_ERROR_X 8
#define MISSING_STEPS_MAX_ERROR_Y 10
#define ROBOT_POSITION_CAMERA_CORRECTION_Y -20 // Correction of the position of the camera because the camera point of view and mark position

// AIR HOCKEY TABLE FANS SPEED
// USE 255 for FULL SPEED (if you use 15V power supply 180 is ok)
#define FAN1_SPEED 255  //180
#define FAN2_SPEED 255  //180

// Utils (don´t modify)
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535

