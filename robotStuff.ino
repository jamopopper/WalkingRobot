//  Robot controlling program for quad legged robot

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>

//#include <BLEDevice.h>
//#include <BLEUtils.h>
//#include <BLEScan.h>
//#include <BLEAdvertisedDevice.h>

#include "math.h"

#define USMIN  350 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2300 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define PI_Value 3.1416
#define Base_Length 113.1

#define Coxa_Length 82.25
#define Femur_Length 82.25
#define Tibia_Length 82.25

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire1);

//custom math
double robotDimensions[] = {113.137, 82.25, 82.25, 82.25}; //{coxa joint to adjacent coxa joint length, coxa length, femur length, tibia length} in mm from joint center to joint center
double legLocation[4][3] = {{ -175, 175, 0}, {175, 175, 0}, { -175, -175, 0}, {175, -175, 0}}; //{leg selected, {foot x, foot y, foot z}} in mm from robot center
double torsoLocation[] = {0, 0, 0}; //{center x, center y, center z} in mm from center of robot on the plane passing through all joints when perpendicular
double torsoOrientation[] = {0, 0, 0}; //{roll, pitch, yaw} in degrees of offset
double servoPositions[4][3] = {{90, 90, 90}, {90, 90, 90}, {90, 90, 90}, {90, 90, 90}}; //
const double servoScale = double(USMAX - USMIN) / 180;
boolean servoState = false;
boolean servoToggle = false;

int testable;

void setup() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  Serial.begin(115200);
  pinMode(3, OUTPUT);
  pinMode(10, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(3, LOW);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  servoCalc();
  //servoSend();
  if (!servoToggle && digitalRead(10)) {
    servoState = !servoState;
  }
  servoToggle = digitalRead(10);
  digitalWrite(3, servoState);
  digitalWrite(LED_BUILTIN, servoState);
  delay(75);
}
void servoCalc() {
  float jointOffset[4][2] = {{-1, 1}, {1, 1}, {-1, -1}, {1, -1}};
  // implement heading rotation using the coordinate rotation math from calc class
  for (uint_fast8_t i = 0U; i < 4U; i++) {
    // variable calulations
    double legHold[2] = {legLocation[i][0] - torsoLocation[0] - (jointOffset[i][0] * robotDimensions[0] / 2), legLocation[i][1] - torsoLocation[1] - (jointOffset[i][1] * robotDimensions[0] / 2)};
    double legLength = sqrt(sq(legLocation[i][2] - torsoLocation[2]) + sq(sqrt(sq(legHold[0]) + sq(legHold[1])) - robotDimensions[1]));
    double legAngle[3] = {0, 0, acos((sq(robotDimensions[2]) + sq(robotDimensions[3]) - sq(legLength))/(2 * robotDimensions[2] * robotDimensions[3]))};
    if (legHold[0] != 0) legAngle[0] = atan(legHold[1]/legHold[0]); //angle from horizontal
    else legAngle[0] = 90;
    // warnings for invalid inputs
    if (legLength > (robotDimensions[2] + robotDimensions[3])) {
      Serial.print(i);
      Serial.print(" is over-extended");
      Serial.print("\t");
      break;
    }
    if (legLength < abs(robotDimensions[2] - robotDimensions[3])) {
      Serial.print(i);
      Serial.print(" is under-extended");
      Serial.print("\t");
      break;
    }
    if ((i % 3) && (legAngle[0] < 0 || legAngle[0] > HALF_PI)) {
      Serial.print(i);
      Serial.print(" coxa servo can't extend that far");
      Serial.print("\t");
      break;
    } else if (!(i % 3) && (legAngle[0] > 0 || legAngle[0] < -HALF_PI)) {
      Serial.print(i);
      Serial.print(" coxa servo can't extend that far");
      Serial.print("\t");
      break;
    }
    // setting servo angles
    if (i % 3) servoPositions[i][0] = 45 + (legAngle[0] * (180 / PI));
    else servoPositions[i][0] = 135 + (legAngle[0] * (180 / PI));
    servoPositions[i][1] = (sqrt(sq(legHold[0]) + sq(legHold[1])) - robotDimensions[1]);
    servoPositions[i][2] = atan((legLocation[i][2] - torsoLocation[2])/((180/PI) * legAngle[2]));
    for (uint_fast8_t j = 0U; j < 3U; j++) {
      Serial.print(servoPositions[i][j]);
      Serial.print("\t");
    }
  }
  Serial.println();
}
void robotAim(int heading) {
  //moving leg designation
  /*
     use joystick to tell the goal position for moving feet
     further joystick push = longer strides
     update location at end of step
  */
}
void servoSend() {
  for (uint_fast8_t i = 0U; i < 4U; i++) {
    for (uint_fast8_t j = 0U; j < 3U; j++) {
      pwm.writeMicroseconds(((i * 3) + j), USMIN + (servoPositions[i][j] * servoScale));
      Serial.print((i * 3) + j);
      Serial.print(" at ");
      Serial.print(USMIN + (servoPositions[i][j] * servoScale));
      Serial.print("\t");
    }
  }
  Serial.println();
}
