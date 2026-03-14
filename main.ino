#include <Wire.h>
#include <math.h>
#include <Servo.h>

/* 
 * MISSION: Orient to wall -> Short Straight Drive -> 90 deg CCW Turn -> Long Straight Drive -> Release Pucks -> Close Puck Gate
 * -> Reverse Long Straight Drive -> 90 deg CW Turn -> Reverse Short Straight Drive -> Wait 3s for reload -> Repeat Short Straight Drive
 * Autopilot uses IMU snapshots to correct drift during both straight drives.
 */

 /*---------------TUNING PARAMETERS (Adjust here)-----------------*/
const bool  USE_IMU_STEERING         = true;  // Toggle Autopilot
enum SensorMode { USE_CM1, USE_CM2, USE_AVG, USE_MAX };
const SensorMode DISTANCE_MODE       = USE_MAX; // Selection for stop-trigger

// Distance Targets (cm) - Driving AWAY from the rear wall
const float DRIVE_STOP_DIST_SHORT           = 35.0;  
const float DRIVE_STOP_RELOAD_DIST_SHORT    = 25.0;  
const float DRIVE_STOP_DIST_LONG            = 164.0; // from 165 
const unsigned long MAX_DRIVE_TIME          = 12000; // 60s Safety Timeout
const float DRIVE_STOP_RELOAD_DIST_LONG     = 25.0;  // reverse until 20cm from back wall

// Timing & Motor Power
const unsigned long PRINT_INTERVAL   = 5000;  // ms
const unsigned long WAIT_PAUSE_MS    = 500;   
const int   ORIENT_SPEED_L           = 190;   
const int   ORIENT_SPEED_R           = 195;   
const int   TURN_MIN_PWM             = 190;   
const int   DRIVE_SPEED_PWM          = 195;   // from 200
const float TURN_BALANCE_OFFSET      = 1.05;   // Motor 2 Bias (Rotation)
const float DRIVE_BALANCE_OFFSET     = 1.05;   // Motor 2 Bias (Straight)
const unsigned long RELOAD_PAUSE_MS    = 3000; // 3s pause
const unsigned long PUCKS_RELEASE_PAUSE_MS = 2000; // 3s pause
const unsigned long ORIENT_MIN_MS       = 3000;  // 6s minimum of orienting first for motors to stabilize

// Thresholds & PID
const float US_TARGET_CM             = 20.0;  
const float US_MATCH_TOL_CM          = 1.0; 
const float US_MATCH_SIDE_CM         = 1.0;  
const float TOF_RIGHT_LIMIT          = 16.0;  
const float TURN_TOLERANCE_DEG       = 1.5;   
const float RELATIVE_TURN_ANGLE      = 93.0;  // adding some bias; from 94
const float TURN_KP                  = 2.0;   
const float DRIVE_KP                 = 2.0;   

// Moving Average Settings
const int   MA_N                     = 2;     
const long  MAX_DISTANCE_CM          = 1000;  

/* --------------- Hardware Pins --------------- */
#define PIN_PWM_MOTOR_1 3
#define IN1_PIN 6
#define IN2_PIN 7
#define PIN_PWM_MOTOR_2 4
#define IN1_PIN2 8
#define IN2_PIN2 9

#define PIN_SERVO 32

/* --------------- State Definitions --------------- */
typedef enum {
    STATE_ORIENTING,
    STATE_SETTLING,
    STATE_DRIVING,
    STATE_TURNING,
    STATE_PUCKS,
    STATE_STOPPED
} RobotState;

typedef enum {
    TASK_ORIENT,
    TASK_SHORT_WAIT,
    TASK_SHORT_FWD,
    TASK_TURN_CCW_90,
    TASK_PRELONG_WAIT,
    TASK_LONG_FWD,
    TASK_PUCK_RELEASE,
    TASK_LONG_REV,
    TASK_TURN_BACK_TO_RELOAD,
    TASK_SHORT_REV,
    TASK_RELOAD_WAIT,
    TASK_DONE
} RobotTask; 

enum DriveDirection { FWD, REV };
enum StopCondition  { STOP_GE, STOP_LE }; // stop when measurement >= target OR <= target

struct DriveContext {
  DriveDirection direction;
  StopCondition  stop;
  float    targetDist; // cm
  float    headingNominal;     // latched on DRIVE entry
  int      basePWM; // 0-255
};

struct TurnContext {
  float targetHeading;         // absolute (0..360)
};

struct SettleContext {
  unsigned long duration; // ms
};

struct PucksContext {
  unsigned long duration; // ms
  bool opened; // gate
};

DriveContext    drive;
TurnContext     turn;
SettleContext   settle;
PucksContext    pucks;
Servo           myServo;  // create servo object to control a servo

/* --------------- Module Variables --------------- */
RobotState state = STATE_ORIENTING;
RobotState prevState = STATE_STOPPED;
RobotTask nextTask = TASK_ORIENT;
float originalHeading = 0; // Pre-turn straight target / home position
float targetHeading = 0;   // Post-turn straight target
float h, p, r;             
float cm1, cm2, cm3, cm4;  
int puckRounds = 0; // NEW: needs to stop running after unloading 3 rounds of pucks
const int MAX_ROUNDS = 3; // NEW

unsigned long stateStartTime = 0;   
unsigned long lastPrintMs = 0;

int servoPos = 0;    // variable to store the servo position

long buf1[MA_N] = {0}, buf2[MA_N] = {0};
long buf3[MA_N] = {0}, buf4[MA_N] = {0};
long sum1 = 0, sum2 = 0, sum3 = 0, sum4 = 0;
int idx = 0, count = 0;
int idx34 = 0, count34 = 0;
int lastLeftPwr = 0, lastRightPwr = 0;
float lastSteerTarget = NAN;
float lastDriftError  = NAN;   // wrapped (-180..180)
int   lastCorrection  = 0;
float longRunHeading = 0.0f;

/* --------------- Main Arduino Functions --------------- */
void setup() {
    Serial.begin(115200);
    Wire.begin();

    setupImu();
//   Serial.println("IMU OK");

    setupUltrasonic();
//   Serial.println("US OK");

    setupMotor();
//   Serial.println("Motors OK");

    myServo.attach(PIN_SERVO);
//   Serial.println("Servo OK");

  // ---- inits ---- //
    prevState = STATE_STOPPED;
    nextTask = TASK_ORIENT;
    changeState(STATE_ORIENTING);
    Serial.println("INIT: ORIENT");

    // Set defaults
    drive.basePWM = DRIVE_SPEED_PWM;
    pucks.duration = PUCKS_RELEASE_PAUSE_MS;
    settle.duration = WAIT_PAUSE_MS;
}

void loop() {
    pollImu(h, p, r);

    float raw1, raw2, raw3, raw4;
    pollUltrasonic(raw1, raw2, raw3, raw4);
    updateMovingAvg(raw1, raw2);
    updateMovingAvg34(raw3, raw4);
    cm1 = getAvg1();
    cm2 = getAvg2();
    cm3 = getAvg3();
    cm4 = getAvg4();

    switch(state) {
        case STATE_ORIENTING:   handleOrienting();    break;
        case STATE_SETTLING:    handleSettling();     break;
        case STATE_DRIVING:     handleDrive();        break;
        case STATE_TURNING:     handleIMUTurn();      break;
        case STATE_PUCKS: // gate already opened bc changeState called releasePucks()      
            if (millis() - stateStartTime >= pucks.duration) {
                closePuckDoor(); 
  
                nextStep();    
            }
            break;
        case STATE_STOPPED:     stopMotors();         break;
    }

    if (millis() - lastPrintMs >= PRINT_INTERVAL) {
        lastPrintMs = millis();
        Serial.print("H: "); Serial.print(h);
        Serial.print(" | cm1: "); Serial.print(cm1);
        Serial.print(" cm2: "); Serial.print(cm2);
        Serial.print(" | cm3: "); Serial.print(cm3);
        Serial.print(" cm4: "); Serial.print(cm4);
        Serial.print(" SteerTarget: "); Serial.print(lastSteerTarget);
        Serial.print(" DriftErr: "); Serial.print(lastDriftError);
        Serial.print("");
        Serial.print(" corr: "); Serial.print(lastCorrection);
        Serial.print(" | PWM L: "); Serial.print(lastLeftPwr);
        Serial.print(" R: "); Serial.print(lastRightPwr);
        Serial.print(" | State: "); Serial.println(state);
    }
}

/* State Switches */
void nextStep() {
    if (nextTask == TASK_RELOAD_WAIT && puckRounds >= MAX_ROUNDS) {
        nextTask = TASK_DONE;
        changeState(STATE_STOPPED);
        return;
    }

    switch(nextTask){
        case TASK_ORIENT:              nextTask = TASK_SHORT_WAIT; break;
        case TASK_SHORT_WAIT:          nextTask = TASK_SHORT_FWD; break;
        case TASK_SHORT_FWD:           nextTask = TASK_TURN_CCW_90; break;
        case TASK_TURN_CCW_90:         nextTask = TASK_PRELONG_WAIT; break;
        case TASK_PRELONG_WAIT:        nextTask = TASK_LONG_FWD; break;
        case TASK_LONG_FWD:            nextTask = TASK_PUCK_RELEASE; break;
        case TASK_PUCK_RELEASE:        nextTask = TASK_LONG_REV; break;
        case TASK_LONG_REV:            nextTask = TASK_TURN_BACK_TO_RELOAD; break;
        case TASK_TURN_BACK_TO_RELOAD: nextTask = TASK_SHORT_REV; break;
        case TASK_SHORT_REV:           nextTask = TASK_RELOAD_WAIT; break;
        case TASK_RELOAD_WAIT:         nextTask = TASK_SHORT_FWD; break;   // repeat
        case TASK_DONE:      /* stay */ break;
        default:                       nextTask = TASK_DONE; break;
    }
    planNextStep();
}

    


/* --------------- State Routines --------------- */
// void handleOrienting() {
//   commandRotateCW(ORIENT_SPEED_L, ORIENT_SPEED_R);

//   bool rearMatch  = (cm1 < US_TARGET_CM && fabs(cm1 - cm2) <= US_MATCH_TOL_CM);
//   bool rightMatch = (cm3 <= TOF_RIGHT_LIMIT && fabs(cm3 - cm4) <= US_MATCH_SIDE_CM);

//   if (rearMatch && rightMatch) {
//     stopMotors();
//     originalHeading = h;               // “home pose”
//     nextTask = TASK_SHORT_WAIT;      // next
//     planNextStep();              // config next action + changeState(...)
//   }
// }

void handleOrienting() {
  commandRotateCW(ORIENT_SPEED_L, ORIENT_SPEED_R);

  // must spin at least 6s before allowing “done”
  if (millis() - stateStartTime < ORIENT_MIN_MS) return;

  bool rearMatch  = (cm1 < US_TARGET_CM && fabs(cm1 - cm2) <= US_MATCH_TOL_CM);
  bool rightMatch = (cm3 <= TOF_RIGHT_LIMIT && fabs(cm3 - cm4) <= US_MATCH_SIDE_CM);

  if (rearMatch && rightMatch) {
    stopMotors();
    originalHeading = h; // keep this setpoint to reuse whenever we do shorter dist nav
    longRunHeading = wrap360(originalHeading - RELATIVE_TURN_ANGLE); // keep this setpoint to reuse whenever we do the longer dist nav
    nextTask = TASK_SHORT_WAIT;
    planNextStep();
  }
}


void handleSettling() {
    stopMotors();
    if (millis() - stateStartTime >= settle.duration) {
        nextStep();  // go into DRIVE or TURN or PUCKS depending on missionStep
    }
}

void handleIMUTurn() {
    float error = turn.targetHeading - h;
    if (error > 180)  error -= 360; // wrap to [-180,180]
    if (error < -180) error += 360; 

    if (abs(error) < TURN_TOLERANCE_DEG) {
        stopMotors();
        nextStep();
        return;
    } 
    int u = constrain(abs(error * TURN_KP), TURN_MIN_PWM, 255);
    if (error < 0) commandRotateCCW(u, (int)(u * TURN_BALANCE_OFFSET));
    else           commandRotateCW(u, (int)(u * TURN_BALANCE_OFFSET));
}


void handleDrive() {
    // --- compute heading correction (using drive.headingNominal)
    lastSteerTarget = drive.headingNominal;
    float driftError = drive.headingNominal - h;
    if (driftError > 180) driftError -= 360; // wrap to [-180,180]
    if (driftError < -180) driftError += 360;
    int correction = (int)(driftError * DRIVE_KP);

    // int rawL = drive.basePWM + correction;
    // int rawR = drive.basePWM - correction;
    if (drive.direction == REV) correction = -correction;  // corrected this
    int rawL = drive.basePWM + correction;
    int rawR = drive.basePWM - correction;

    int leftPwr  = constrain(rawL, 180, 255);
    int rightPwr = constrain((int)(rawR * DRIVE_BALANCE_OFFSET), 180, 255);

    // direction
    const bool FWD_IN1  = true;
    const bool FWD_IN2  = false;
    const bool FWD_IN3  = true;
    const bool FWD_IN4  = false;

    const bool REV_IN1  = false;
    const bool REV_IN2  = true;
    const bool REV_IN3  = false;
    const bool REV_IN4  = true;
    if (drive.direction == FWD) setMotorDirPins(FWD_IN1, FWD_IN2, FWD_IN3, FWD_IN4);
    else            setMotorDirPins(REV_IN1, REV_IN2, REV_IN3, REV_IN4);
    
    lastLeftPwr = leftPwr;
    lastRightPwr = rightPwr;
    analogWrite(PIN_PWM_MOTOR_1, leftPwr);
    analogWrite(PIN_PWM_MOTOR_2, rightPwr);

    // --- distance measure (diff modes)
    float currentMeasure = 0;
    switch (DISTANCE_MODE) {
        case USE_CM1: currentMeasure = cm1; break;
        case USE_CM2: currentMeasure = cm2; break;
        case USE_AVG: currentMeasure = (cm1 + cm2) / 2.0; break;
        case USE_MAX: currentMeasure = max(cm1,cm2); break;
    }

    bool reached = false;
    if (!isnan(currentMeasure)) {
        reached = (drive.stop == STOP_GE) ? (currentMeasure >= drive.targetDist)
                                        : (currentMeasure <= drive.targetDist);
    }

    bool timeoutReached = (millis() - stateStartTime >= MAX_DRIVE_TIME);

    if (reached || timeoutReached) {
        stopMotors();
        nextStep();
    }
}

void planNextStep() {
  switch (nextTask) {

    case TASK_SHORT_FWD:
        drive.headingNominal = originalHeading; // NEW: to hold same holding durign repeats
        drive.direction = FWD;
        drive.stop = STOP_GE;              
        drive.targetDist = DRIVE_STOP_DIST_SHORT;
        drive.basePWM = DRIVE_SPEED_PWM;
        changeState(STATE_DRIVING);
        break;
    
    case TASK_SHORT_WAIT:
        settle.duration = WAIT_PAUSE_MS;
        changeState(STATE_SETTLING);
        break;

    case TASK_PRELONG_WAIT:
        settle.duration = WAIT_PAUSE_MS;
        changeState(STATE_SETTLING);
        break;

    case TASK_TURN_CCW_90:
        // CCW 90 relative to current heading / relative to originalHeading;
        // turn.targetHeading = wrap360(h - RELATIVE_TURN_ANGLE);  // NEW: bug that was ran during competition
        turn.targetHeading = longRunHeading; // NEW: same setpt on repeats
        changeState(STATE_TURNING);
        break;

    case TASK_LONG_FWD:
        drive.headingNominal = longRunHeading; // NEW: same setpt on repeats
        drive.direction = FWD;
        drive.stop = STOP_GE;
        drive.targetDist = DRIVE_STOP_DIST_LONG; 
        drive.basePWM = DRIVE_SPEED_PWM;
        changeState(STATE_DRIVING);
        break;

    case TASK_PUCK_RELEASE:
        pucks.duration = PUCKS_RELEASE_PAUSE_MS;
        changeState(STATE_PUCKS);
        break;

    case TASK_LONG_REV:
        drive.headingNominal = longRunHeading; // NEW: same setpt on repeats
        drive.direction = REV;
        drive.stop = STOP_LE;              // reverse until <= x cm
        drive.targetDist = DRIVE_STOP_RELOAD_DIST_LONG;
        drive.basePWM = DRIVE_SPEED_PWM;
        changeState(STATE_DRIVING);
        break;

    case TASK_TURN_BACK_TO_RELOAD:
        turn.targetHeading = originalHeading;
        changeState(STATE_TURNING);
        break;
      
    case TASK_SHORT_REV:
        drive.headingNominal = originalHeading; // NEW: same setpt on repeats
        drive.direction = REV;
        drive.stop = STOP_LE;
        drive.targetDist = DRIVE_STOP_RELOAD_DIST_SHORT;
        drive.basePWM = DRIVE_SPEED_PWM;
        changeState(STATE_DRIVING);
        break;

    case TASK_RELOAD_WAIT:
        settle.duration = RELOAD_PAUSE_MS;
        changeState(STATE_SETTLING);
        break;

    case TASK_DONE:
        // repeat
        changeState(STATE_STOPPED);
        break;

    default:
        changeState(STATE_STOPPED);
        break;
  }
}

/* --------------- Helper Functions --------------- */

void setupMotor() {
    pinMode(PIN_PWM_MOTOR_1, OUTPUT);
    pinMode(IN1_PIN, OUTPUT);
    pinMode(IN2_PIN, OUTPUT);
    pinMode(PIN_PWM_MOTOR_2, OUTPUT);
    pinMode(IN1_PIN2, OUTPUT);
    pinMode(IN2_PIN2, OUTPUT);
    stopMotors();
}

static inline bool validCm(long cm) {
    return (cm > 0 && cm <= MAX_DISTANCE_CM);
}

void updateMovingAvg(long cm1_raw, long cm2_raw) {
    if (!validCm(cm1_raw) || !validCm(cm2_raw)) return;
    sum1 -= buf1[idx]; sum2 -= buf2[idx];
    buf1[idx] = cm1_raw; buf2[idx] = cm2_raw;
    sum1 += buf1[idx]; sum2 += buf2[idx];
    idx = (idx + 1) % MA_N;
    if (count < MA_N) count++;
}

void updateMovingAvg34(long cm3_raw, long cm4_raw) {
    if (!validCm(cm3_raw) || !validCm(cm4_raw)) return;
    sum3 -= buf3[idx34]; sum4 -= buf4[idx34];
    buf3[idx34] = cm3_raw; buf4[idx34] = cm4_raw;
    sum3 += buf3[idx34]; sum4 += buf4[idx34];
    idx34 = (idx34 + 1) % MA_N;
    if (count34 < MA_N) count34++;
}

float getAvg1() { return (count == 0) ? NAN : (float)sum1 / (float)count; }
float getAvg2() { return (count == 0) ? NAN : (float)sum2 / (float)count; }
float getAvg3() { return (count34 == 0) ? NAN : (float)sum3 / (float)count34; }
float getAvg4() { return (count34 == 0) ? NAN : (float)sum4 / (float)count34; }

void changeState(RobotState newState) {
    RobotState old = state;
    state = newState;
    stateStartTime = millis();

    Serial.print("NEW STATE: ");
    Serial.print(old);
    Serial.print("->");
    Serial.println(newState);

    switch(state){
        // case STATE_DRIVING:     drive.headingNominal = h; break; // NEW: bug that was ran during competition, want to instead use same 2 headings setpts on repeats
        case STATE_DRIVING:     break; // NEW: use same 2 headings setps on repeats
        case STATE_PUCKS:       releasePucks(); break;
        case STATE_SETTLING:    stopMotors(); break;
        default: break;
    }
}

void stopMotors() {
    analogWrite(PIN_PWM_MOTOR_1, 0);
    analogWrite(PIN_PWM_MOTOR_2, 0);
}

void commandRotateCW(int pwmLeft, int pwmRight) {
    setMotorDirPins(true, false, false, true);
    analogWrite(PIN_PWM_MOTOR_1, pwmLeft);
    analogWrite(PIN_PWM_MOTOR_2, pwmRight);
}

void commandRotateCCW(int pwmLeft, int pwmRight) {
    setMotorDirPins(false, true, true, false);
    analogWrite(PIN_PWM_MOTOR_1, pwmLeft);
    analogWrite(PIN_PWM_MOTOR_2, pwmRight);
}

void setMotorDirPins(bool in1, bool in2, bool in3, bool in4) {
    digitalWrite(IN1_PIN, in1); digitalWrite(IN2_PIN, in2);
    digitalWrite(IN1_PIN2, in3); digitalWrite(IN2_PIN2, in4);
}

float wrap360(float deg) {
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    return deg;
}

float wrapTo180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void releasePucks() {
    puckRounds++; // NEW
    for (servoPos = 70; servoPos >= 0; servoPos -= 1) { // goes from 180 degrees to 0 degrees
        myServo.write(servoPos);              // tell servo to go to position in variable 'pos'
        // delay(100);                       // waits 15ms for the servo to reach the position

    }
}

void closePuckDoor() {
    for (servoPos = 0; servoPos <= 70; servoPos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myServo.write(servoPos);              // tell servo to go to position in variable 'pos'
    // delay(100);                       // waits 15ms for the servo to reach the position
    }
}

