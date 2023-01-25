#include <MPU6050.h>  // install this from the library manager (MPU-6050 by Electric cats)
#include "math.h"     // include this library for the atan() function
// #include "I2CDev.h"

// definitions of the PIC constants
#define Kp  50
#define Kd  2
#define Ki  40

#define sampleTime  0.005
#define targetAngle 0

#define RAD_TO_DEG 57.2958

// define pins for motors
#define MOTOR_DIR_PIN_1 3 // direction pins
#define MOTOR_DIR_PIN_2 4
#define MOTOR_PWM_PIN   5 // motor power pin

// create obj for sensors
MPU6050 mpu;

// volatile global variables that will be used in ISR
volatile int motorPower;
volatile float accAngle; 
volatile float currentAngle, prevAngle=0;
volatile float error, prevError=0, errorSum=0;

int16_t accX, accZ; // accel reading variables

void setup() {
  
  // init interupts
  init_interrupt();
  // TODO: initialize all input/output pins required
  pinMode(MOTOR_DIR_PIN_1, OUTPUT);
  pinMode(MOTOR_DIR_PIN_2, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);

  // TODO: initialize sensor
  mpu.initialize();       // turn on the accel/gyro
  Serial.begin(9600);     // start serial communication

}

void loop() {
  // measure the acc outputs
  accX = mpu.getAccelerationX();  // read the X acceleration
  accZ = mpu.getAccelerationZ();  // read the Z acceleration

  // set motor speeds/direction
  set_motor_speed(motorPower);  
}

void init_interrupt()
{
    cli();                      // disable global interrupts
    TCCR1A = 0;                 // set entire TCCR1A register to 0
    TCCR1B = 0;                 // same for TCCR1B    
    OCR1A = 9999;               // set timer for 5s
    TCCR1B |= (1 << WGM12);     // turn on CTC mode
    TCCR1B |= (1 << CS11);      // Set CS11 bit for prescaling by 8
    TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
    sei();                      // enable global interrupts
}

ISR(TIMER1_COMPA_vect)
{
    // calculate the angle of inclination
    accAngle = atan2(accX, accZ)*RAD_TO_DEG;
    
    currentAngle = accAngle;
    // calculate error
    error = currentAngle - targetAngle;
    errorSum = errorSum + error;  
    errorSum = constrain(errorSum, -100, 100);

    //calculate output from Kp, Ki and Kd values
    motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
    if ( currentAngle >= -7.5 && currentAngle <= 7.5){
    motorPower = 0;
  }
    prevAngle = currentAngle;
}

// TODO: add motor speed setting functions
void set_motor_speed(int pwm){
  // set motors in forward direction
  if(prevAngle >= 0){
    digitalWrite(MOTOR_DIR_PIN_1, HIGH);
    digitalWrite(MOTOR_DIR_PIN_2, LOW);
  } else {
    digitalWrite(MOTOR_DIR_PIN_1, LOW);
    digitalWrite(MOTOR_DIR_PIN_2, HIGH);
  }
 
  
  // set motor speed to half speed
  if(pwm < 0){
    pwm *= -1;
  }
  pwm = constrain(pwm,0,255);
  Serial.println(pwm);
  analogWrite(MOTOR_PWM_PIN, pwm);
}