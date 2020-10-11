#include <Arduino.h>
#include <CheapStepper.h>
#include <OneButton.h>

#define STEPPER_IN1 16
#define STEPPER_IN2 17
#define STEPPER_IN3 18
#define STEPPER_IN4 19
#define BUTTON_RED 15
#define BUTTON_GREEN 2
#define BUTTON_BLUE 4
#define LED_RED 14
#define LED_GREEN 12
#define LED_BLUE 13
#define POT_LEFT 36
#define POT_RIGHT 39
#define MOTOR_IN1 27
#define MOTOR_IN2 26
#define MOTOR_ENABLE1 25
#define MOTOR_ENABLE1_PWM_CHANNEL 1
#define MOTOR_PWM_FREQUENCY 5000
#define MOTOR_STATE_STOPPED 0
#define MOTOR_STATE_FORWARD 1
#define MOTOR_STATE_REVERSE -1

#define SERVO_LOWER 21
#define SERVO_UPPER 22
#define SERVO_LOWER_PWM_CHANNEL 2
#define SERVO_UPPER_PWM_CHANNEL 3
#define SERVO_PWM_FREQUENCY 50
#define SERVO_POS_MIN ((3.5 * 4096) / 100)
#define SERVO_POS_MID ((7.5 * 4096) / 100)
#define SERVO_POS_MAX ((11.5 * 4096) / 100)

#define MODE_NONE 0
#define MODE_MOTOR 1
#define MODE_SERVO_ZERO 2
#define MODE_SERVO_POT 3
#define MAX_MODES 4
#define UPDATE_DELAY 50

CheapStepper stepper(STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);
OneButton bluebtn(BUTTON_BLUE, false);
int mode = 0;
unsigned long lastServoUpdate = 0;
unsigned long lastMotorUpdate = 0;
int lastMotorState = 0;

void on_blue_button();

void setup()
{
  Serial.begin(115200);

  pinMode(BUTTON_RED, INPUT_PULLDOWN);
  pinMode(BUTTON_GREEN, INPUT_PULLDOWN);
  pinMode(BUTTON_BLUE, INPUT_PULLDOWN);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(POT_LEFT, INPUT);
  pinMode(POT_RIGHT, INPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENABLE1, OUTPUT);

  // Setup PWM for the Motor Speed
  ledcSetup(MOTOR_ENABLE1_PWM_CHANNEL, MOTOR_PWM_FREQUENCY, 12);
  ledcAttachPin(MOTOR_ENABLE1, MOTOR_ENABLE1_PWM_CHANNEL);

  // Setup PWM for the Servos
  ledcSetup(SERVO_LOWER_PWM_CHANNEL, SERVO_PWM_FREQUENCY, 12);
  ledcSetup(SERVO_UPPER_PWM_CHANNEL, SERVO_PWM_FREQUENCY, 12);
  ledcAttachPin(SERVO_LOWER, SERVO_LOWER_PWM_CHANNEL);
  ledcAttachPin(SERVO_UPPER, SERVO_UPPER_PWM_CHANNEL);

  bluebtn.attachClick(on_blue_button);

  stepper.setRpm(20);
}

void on_blue_button()
{
  mode = (mode + 1) % 4;
  switch (mode)
  {
  case MODE_NONE:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    break;
  case MODE_MOTOR:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    break;
  case MODE_SERVO_ZERO:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    ledcWrite(SERVO_LOWER_PWM_CHANNEL, SERVO_POS_MID);
    ledcWrite(SERVO_UPPER_PWM_CHANNEL, SERVO_POS_MID);
    break;
  case MODE_SERVO_POT:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
    break;
  }
}

void setMotorSpeed()
{
  unsigned long now = millis();
  if((now - lastMotorUpdate) > UPDATE_DELAY)
  {
    int value = analogRead(POT_RIGHT);
    Serial.println(value);
    ledcWrite(MOTOR_ENABLE1_PWM_CHANNEL, map(value, 0, 4096, 2048, 4096));
    lastMotorUpdate = now;
  }
}

void driveMotor()
{
  if (digitalRead(BUTTON_RED) == HIGH)
  {
    if(lastMotorState != MOTOR_STATE_FORWARD)
    {
      digitalWrite(MOTOR_IN1, HIGH);
      digitalWrite(MOTOR_IN2, LOW);
      lastMotorState = MOTOR_STATE_FORWARD;
    }
    setMotorSpeed();
  }
  else if (digitalRead(BUTTON_GREEN) == HIGH)
  {
    if(lastMotorState != MOTOR_STATE_REVERSE)
    {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, HIGH);
      lastMotorState = MOTOR_STATE_REVERSE;
    }
    setMotorSpeed();
  }
  else
  {
    if(lastMotorState != MOTOR_STATE_STOPPED)
    {
      digitalWrite(MOTOR_IN1, LOW);
      digitalWrite(MOTOR_IN2, LOW);
      ledcWrite(MOTOR_ENABLE1_PWM_CHANNEL, 0);
      lastMotorState = MOTOR_STATE_STOPPED;
    }
  }
}

void driveStepper()
{
  if (digitalRead(BUTTON_RED) == HIGH)
  {
    stepper.step(true);
  }
  else if (digitalRead(BUTTON_GREEN) == HIGH)
  {
    stepper.step(false);
  }
}

void driveServos()
{
  unsigned long now = millis();
  if ((now - lastServoUpdate) > UPDATE_DELAY)
  {
    int value = analogRead(POT_LEFT);
    ledcWrite(SERVO_LOWER_PWM_CHANNEL, map(value, 0, 4096, SERVO_POS_MIN, SERVO_POS_MAX));
    Serial.println(value);
    value = analogRead(POT_RIGHT);
    ledcWrite(SERVO_UPPER_PWM_CHANNEL, map(value, 0, 4096, SERVO_POS_MIN, SERVO_POS_MAX));
    Serial.println(value);
    lastServoUpdate = now;
  }
}

void loop()
{
  bluebtn.tick();

  if (mode == MODE_MOTOR)
  {
    driveMotor();
  }
  else
  {
    driveStepper();
    if (mode == MODE_SERVO_POT)
    {
      driveServos();
    }
  }
}