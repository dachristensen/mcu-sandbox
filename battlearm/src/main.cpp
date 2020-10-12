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
#define LED_RED 23
#define LED_GREEN 22
#define LED_BLUE 21
#define MOTOR_RELAY 26
#define MOTOR_STATE_STOPPED 0
#define MOTOR_STATE_FORWARD 1
#define MOTOR_STATE_REVERSE -1

#define SERVO_LOWER 12
#define SERVO_UPPER 13
#define SERVO_LOWER_PWM_CHANNEL 2
#define SERVO_UPPER_PWM_CHANNEL 3
#define SERVO_PWM_FREQUENCY 50
#define SERVO_POS_MIN ((3.5 * 4096) / 100)
#define SERVO_POS_MID ((7.5 * 4096) / 100)
#define SERVO_POS_MAX ((11.5 * 4096) / 100)
#define SERVO_STEP ((SERVO_POS_MAX - SERVO_POS_MIN) / 100)

#define MODE_STEPPER 0
#define MODE_MOTOR 1
#define MODE_SERVO_LOWER 2
#define MODE_SERVO_UPPER 3
#define MAX_MODES 4
#define UPDATE_DELAY 50

CheapStepper stepper(STEPPER_IN1, STEPPER_IN2, STEPPER_IN3, STEPPER_IN4);
OneButton modebtn(BUTTON_BLUE, false);
int mode = MODE_STEPPER;
unsigned long lastServoUpdate = 0;
unsigned long lastMotorUpdate = 0;
int lastMotorState = 0;
bool motorOn = false;

void on_mode_button();

/**
 * Setup all inputs, outputs and PWM
 */
void setup()
{
  Serial.begin(115200);

  pinMode(BUTTON_RED, INPUT_PULLDOWN);
  pinMode(BUTTON_GREEN, INPUT_PULLDOWN);
  pinMode(BUTTON_BLUE, INPUT_PULLDOWN);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(MOTOR_RELAY, OUTPUT);

  // Setup PWM for the Servos
  ledcSetup(SERVO_LOWER_PWM_CHANNEL, SERVO_PWM_FREQUENCY, 12);
  ledcSetup(SERVO_UPPER_PWM_CHANNEL, SERVO_PWM_FREQUENCY, 12);
  ledcAttachPin(SERVO_LOWER, SERVO_LOWER_PWM_CHANNEL);
  ledcAttachPin(SERVO_UPPER, SERVO_UPPER_PWM_CHANNEL);
  ledcWrite(SERVO_LOWER_PWM_CHANNEL, SERVO_POS_MID);
  ledcWrite(SERVO_UPPER_PWM_CHANNEL, SERVO_POS_MID);

  modebtn.attachClick(on_mode_button);

  stepper.setRpm(20);
}

/**
 * Update the LED's in resonse to the mode button being pressed.
 */
void on_mode_button()
{
  mode = (mode + 1) % 4;
  switch (mode)
  {
  case MODE_STEPPER:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    break;
  case MODE_MOTOR:
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    break;
  case MODE_SERVO_LOWER:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    break;
  case MODE_SERVO_UPPER:
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
    break;
  }
}

/**
 * Step the motor clockwise or counter-clockwise depending on whether the red
 *  or green button is pressed.
 */
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

/**
 * Decrease the angle if the red button is pressed or increase the angle 
 *  if the green button is pressed. Reset both servos to mid position if 
 *  both buttons are pressed.
 */
void driveServo(int channel)
{
  unsigned long now = millis();
  if ((now - lastServoUpdate) > UPDATE_DELAY)
  {
    int value = ledcRead(channel);
    bool red = (digitalRead(BUTTON_RED) == HIGH);
    bool green = (digitalRead(BUTTON_GREEN) == HIGH);
    
    if(red && green)
    {
      ledcWrite(SERVO_LOWER_PWM_CHANNEL, SERVO_POS_MID);
      ledcWrite(SERVO_UPPER_PWM_CHANNEL, SERVO_POS_MID);
    }
    else if(red)
    {
      if(value > SERVO_POS_MIN){
        value -= SERVO_STEP;
        ledcWrite(channel, value);
      }
    }
    else if(green)
    {
      if(value < SERVO_POS_MAX){
        value += SERVO_STEP;
        ledcWrite(channel, value);
      }
    }
    lastServoUpdate = now;
  }
}

/**
 * Run the main loop. Response to buttons to drive the stepper,
 *  servos and the motor.
 */
void loop()
{
  modebtn.tick();

  if (mode == MODE_MOTOR && digitalRead(BUTTON_RED) == HIGH)
  {
    if(!motorOn)
    {
      digitalWrite(MOTOR_RELAY, HIGH);
      motorOn = true;
    }
  }
  else if(motorOn)
  {
    digitalWrite(MOTOR_RELAY, LOW);
    motorOn = false;
  }

  switch(mode)
  {
    case MODE_STEPPER:
      driveStepper();
      break;
    case MODE_SERVO_LOWER:
      driveServo(SERVO_LOWER_PWM_CHANNEL);
      break;
    case MODE_SERVO_UPPER:
      driveServo(SERVO_UPPER_PWM_CHANNEL);
      break;
  }
}