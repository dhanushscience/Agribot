#include <Wire.h>
#include <PCF8574.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

/* =======================
   PCF8574 PIN MAP
   ======================= */
PCF8574 pcf(0x20);

// P0 – Right ultrasonic TRIG
// P1 – Front ultrasonic TRIG
// P2 – Back ultrasonic TRIG
// P3 – Pump Relay
// P4 – L Motor A IN1
// P5 – L Motor A IN2
// P6 – L Motor B IN1
// P7 – L Motor B IN2

/* =======================
   UNO PINS
   ======================= */
#define STBY 4

// PWM for TB6612
#define LMA_PWM 5
#define LMB_PWM 6
#define RMA_PWM 9
#define RMB_PWM 10

// Ultrasonic ECHO pins
#define RIGHT_ECHO A0
#define FRONT_ECHO A1
#define BACK_ECHO A2

// Moisture sensor
#define MOISTURE A3

// Servo
#define SERVO_PIN 3

/* =======================
   SERVOS & OBJECTS
   ======================= */
Servo arm;
MPU6050 mpu6050(Wire);

float targetYaw = 0;
float Kp_heading = 2.0;

/* =======================
   MOTION CONSTANTS
   ======================= */
int SLOW_SPEED = 90;
int NORMAL_SPEED = 150;

unsigned long TIME_1M_SLOW = 6000;
unsigned long TIME_10M = 25000;
unsigned long TIME_0_5M = 3000;

/* =======================
   STATES
   ======================= */
enum State {
  SLOW_FORWARD,
  FORWARD,
  SHIFT_LEFT,
  BACKWARD,
  STOPPED
} state;

unsigned long stateStart;

/* =======================
   ULTRASONIC FUNCTION
   ======================= */
long readUltra(int trigPinPCF, int echoPin) {
  pcf.write(trigPinPCF, LOW);
  delayMicroseconds(2);
  pcf.write(trigPinPCF, HIGH);
  delayMicroseconds(10);
  pcf.write(trigPinPCF, LOW);

  long dur = pulseIn(echoPin, HIGH, 30000);
  if (dur == 0) return 999;
  return dur / 58;
}

/* =======================
   MOTOR CONTROL
   ======================= */
void motorLeft(int speed, bool forward) {
  pcf.write(4, forward ? HIGH : LOW);
  pcf.write(5, forward ? LOW : HIGH);
  analogWrite(LMA_PWM, speed);

  pcf.write(6, forward ? HIGH : LOW);
  pcf.write(7, forward ? LOW : HIGH);
  analogWrite(LMB_PWM, speed);
}

void motorRight(int speed, bool forward) {
  // For right motors, we assume direction controlled by second PCF if available
  // For UNO case, we'll invert PWM direction only (simple model)
  analogWrite(RMA_PWM, forward ? speed : 0);
  analogWrite(RMB_PWM, forward ? speed : 0);
}

void stopAll() {
  analogWrite(LMA_PWM, 0);
  analogWrite(LMB_PWM, 0);
  analogWrite(RMA_PWM, 0);
  analogWrite(RMB_PWM, 0);
}

/* =======================
   DRIVE STRAIGHT w/ MPU
   ======================= */
void driveStraight(int baseSpeed, bool forward) {
  mpu6050.update();
  float yaw = mpu6050.getAngleZ();
  float error = yaw - targetYaw;
  int correction = Kp_heading * error;

  int leftSpeed = constrain(baseSpeed - correction, 0, 255);
  int rightSpeed = constrain(baseSpeed + correction, 0, 255);

  motorLeft(leftSpeed, forward);
  motorRight(rightSpeed, forward);
}

/* =======================
   WATERING
   ======================= */
void waterPlant() {
  stopAll();
  arm.write(80);
  delay(700);

  int val = analogRead(MOISTURE);
  if (val > 500) {
    pcf.write(3, HIGH);
    delay(2000);
    pcf.write(3, LOW);
  }

  arm.write(0);
  delay(500);
}

/* =======================
   SETUP
   ======================= */
void setup() {
  Serial.begin(9600);
  Wire.begin();

  pcf.begin();

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(RIGHT_ECHO, INPUT);
  pinMode(FRONT_ECHO, INPUT);
  pinMode(BACK_ECHO, INPUT);

  arm.attach(SERVO_PIN);
  arm.write(0);

  // MPU init
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.update();
  targetYaw = mpu6050.getAngleZ();

  state = SLOW_FORWARD;
  stateStart = millis();

  Serial.println("BOT READY");
}

/* =======================
   LOOP
   ======================= */
void loop() {
  long right = readUltra(0, RIGHT_ECHO);
  long front = readUltra(1, FRONT_ECHO);
  long back  = readUltra(2, BACK_ECHO);

  if (right < 20) waterPlant();
  if (front < 20 || back < 20) { stopAll(); return; }

  unsigned long t = millis();

  switch (state) {
    case SLOW_FORWARD:
      driveStraight(SLOW_SPEED, true);
      if (t - stateStart > TIME_1M_SLOW) {
        state = FORWARD;
        stateStart = t;
        targetYaw = mpu6050.getAngleZ();
      }
      break;

    case FORWARD:
      driveStraight(NORMAL_SPEED, true);
      if (t - stateStart > TIME_10M) {
        state = SHIFT_LEFT;
        stateStart = t;
        stopAll();
      }
      break;

    case SHIFT_LEFT:
      // Simple shift: turn left, move, turn back
      stopAll();
      delay(300);
      // turn left
      motorLeft(100, false);
      motorRight(100, true);
      delay(700);

      stopAll();
      delay(200);

      driveStraight(NORMAL_SPEED, true);
      delay(TIME_0_5M);

      stopAll();
      delay(200);

      // turn right back
      motorLeft(100, true);
      motorRight(100, false);
      delay(700);

      stopAll();
      delay(300);

      targetYaw = mpu6050.getAngleZ();
      state = BACKWARD;
      stateStart = millis();
      break;

    case BACKWARD:
      driveStraight(NORMAL_SPEED, false);
      if (t - stateStart > TIME_10M) {
        stopAll();
        state = STOPPED;
      }
      break;

    case STOPPED:
      stopAll();
      break;
  }
}
