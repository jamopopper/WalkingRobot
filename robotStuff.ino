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
float robotDimensions[] = {113.137, 82.25, 82.25, 82.25}; //{coxa joint to adjacent coxa joint length, coxa length, femur length, tibia length} in mm from joint center to joint center
float legLocation[4][3] = {{ -150, 250, 0}, {150, 250, 0}, { -150, -250, 0}, {150, -250, 0}}; //{leg selected, {foot x, foot y, foot z}} in mm from robot center
float torsoLocation[] = {0, 0, 0}; //{center x, center y, center z} in mm from center of robot on the plane passing through all joints when perpendicular
float torsoOrientation[] = {0, 0, 0}; //{roll, pitch, yaw} in degrees of offset
const double servoScale = double(USMAX - USMIN) / 180;
float servoPositions[4][3] = {{90, 90, 90}, {90, 90, 90}, {90, 90, 90}, {90, 90, 90}};
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
  for (uint_fast8_t i = 0U; i < 4U; i++) {
    float legLength = 0;
    float legAngle = 0;
    float legHold[2] = {legLocation[i][0] - torsoLocation[0] - (jointOffset[i][0] * robotDimensions[0] / 2), legLocation[i][1] - torsoLocation[1] - (jointOffset[i][1] * robotDimensions[0] / 2)};
    legHold[0] = ((legHold[0]) * (legHold[0])) + ((legHold[1]) * (legHold[1]));
    legLength = sqrt((legHold[0]) + ((legLocation[i][2] - torsoLocation[2]) * (legLocation[i][2] - torsoLocation[2]))); //leg extention
    legHold[1] = sqrt(legHold[0]);
    if (legHold[0] != 0) legAngle = asin((legLocation[i][1] - torsoLocation[1] - (jointOffset[i][1] * robotDimensions[0] / 2)) / legHold[1]); //angle from perpendicular
    else legAngle = 0;
    Serial.print(i);
    Serial.print(" is at ");
    if ((i % 2)) Serial.print(90 - (legAngle * (180 / PI)));
    else Serial.print(90 + (legAngle * (180 / PI)));
    Serial.print("\t");
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
