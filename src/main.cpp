#include <Arduino.h>
#include <Wire.h>
#include <Kalman.h> 
#include <DigitalToggle.h>
#include <PID_v1.h>
#include <Servo.h>
#include <functions.h>



#define PID_OUT_LIMIT 30

#define CALIBRATE_ESC 
// #define TUNNING_PID
#define SHOW_ANGLE_DATA 
#define AUTOTUNE_PID 

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Servo ESC1;
Servo ESC2;
Servo ESC3;
Servo ESC4;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double kalAngleX, kalAngleY; 

uint32_t timer;
uint8_t i2cData[14]; 

double dt;

double roll;
double pitch;
double gyroXrate;
double gyroYrate;

int esc1_assigned_value;
int esc3_assigned_value;
int esc2_assigned_value;
int esc4_assigned_value;

int esc1_output_mapped;
int esc2_output_mapped;
int esc3_output_mapped;
int esc4_output_mapped;

int hovering_throttle = 40;

int joystick_roll_pwm_value;
int joystick_pitch_pwm_value;
int joystick_throttle_pwm_value;

double roll_setpoint, roll_input, pid_roll_output;
double pitch_setpoint, pitch_input, pid_pitch_output;

//Define the aggressive and conservative Tuning Parameters
#ifdef AUTOTUNE_PID
  double aggKp=4, aggKi=0.2, aggKd=1;
  double consKp=1, consKi=0.05, consKd=0.25;
  PID roll_PID(&roll_input, &pid_roll_output, &roll_setpoint, consKp, consKi, consKd, DIRECT);
  PID pitch_PID(&pitch_input, &pid_pitch_output, &pitch_setpoint, consKp, consKi, consKd, DIRECT);
#endif
#ifndef AUTOTUNE_PID
  double Kp=2.2, Ki=0, Kd=1.6;
  PID roll_PID(&roll_input, &pid_roll_output, &roll_setpoint, Kp, Ki, Kd, DIRECT);
  PID pitch_PID(&pitch_input, &pid_pitch_output, &pitch_setpoint, Kp, Ki, Kd, DIRECT);
#endif

volatile int prev_time = 0;

char str[30];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  // pinMode(9, OUTPUT);
  // pinMode(10, OUTPUT);
  ESC1.attach(7);
  ESC3.attach(6);
  ESC2.attach(9);
  ESC4.attach(10);
  
  pinMode(14, OUTPUT);

  pinMode(15, INPUT);
  pinMode(16, INPUT);

  roll_PID.SetOutputLimits(-PID_OUT_LIMIT, PID_OUT_LIMIT);
  pitch_PID.SetOutputLimits(-PID_OUT_LIMIT, PID_OUT_LIMIT);

  roll_setpoint = 0;
  pitch_setpoint = 0;

  roll_PID.SetMode(AUTOMATIC);
  pitch_PID.SetMode(AUTOMATIC);

  config_i2cData();

  kalman_set_initial_angle();

  #ifdef CALIBRATE_ESC
  delay(1000);
  int i;
  for (i = 1900; i > 1000; i--)
  {
    ESC1.writeMicroseconds(i);
    ESC3.writeMicroseconds(i);
    ESC2.writeMicroseconds(i);
    ESC4.writeMicroseconds(i);
    delay(6);
  }
  delay(1000);
  ESC1.writeMicroseconds(1000);
  delay(500);
  ESC2.writeMicroseconds(1000);
  delay(500);
  ESC3.writeMicroseconds(1000);
  delay(500);
  ESC4.writeMicroseconds(1000);
  delay(500);
  #endif
    timer = micros();
}

void loop() {

#ifndef AUTOTUNE_PID 
  while (Serial.available())
  {
    if (Serial.available() >0)
    {
      char c = Serial.read();  //gets one byte from serial buffer
      switch(c)
        {
        case 'q':
          {
            Kp += 0.1;
          }
        break;
        case 'a':
          {
            Kp -= 0.1;
          }
        break;
        case 'w':
          {
            Ki += 0.1;
          }
        break;
        case 's':
          {
            Ki -= 0.1;
          }
        break;
        case 'e':
          {
            Kd += 0.1;
          }
        break;
        case 'd':
          {
            Kd -= 0.1;
          }
        break;
        case 'r':
          {
           hovering_throttle += 1;
          }
        break;
        case 'f':
          {
            hovering_throttle -= 1;
          }
        break;
        }
    }
  }

#endif
    #ifdef TUNNING_PID
     Serial.print(Kp);Serial.print("\t");
     Serial.print(Ki);Serial.print("\t");
     Serial.print(Kd);Serial.print("\t");
     Serial.print(hovering_throttle);Serial.print("\t");
    #endif
	// digitalToggle(14);
  // joystick_roll_pwm_value = pulseIn(16, HIGH);
  // Serial.print(joystick_roll_pwm_value);Serial.print("\t");

  // joystick_pitch_pwm_value = pulseIn(15, HIGH);
  // Serial.print(joystick_pitch_pwm_value);Serial.print("\t");

  // joystick_pitch_pwm_value = pulseIn(15, HIGH);
  // Serial.print(joystick_throttle_pwm_value);Serial.print("\t");

  /* Update all the values */
  get_IMU_data();

  update_time();

  compute_roll_pitch();

  check_and_fix_transition();

  roll_input = -kalAngleX;
  pitch_input = -kalAngleY;

  #ifdef SHOW_ANGLE_DATA
    Serial.print(kalAngleX); Serial.print("\t");
    Serial.print(roll); Serial.print("\t");
    Serial.print(kalAngleY); Serial.print("\t");
    Serial.print(pitch); Serial.print("\t");
  #endif

  roll_PID.Compute();
  pitch_PID.Compute();

  // esc1_assigned_value = hovering_throttle + pid_pitch_output;
  // esc3_assigned_value = hovering_throttle - pid_pitch_output;
  // esc2_assigned_value = hovering_throttle + pid_roll_output;
  // esc4_assigned_value = hovering_throttle - pid_roll_output;

  esc1_assigned_value = hovering_throttle + pid_roll_output;
  esc3_assigned_value = hovering_throttle - pid_pitch_output;
  esc2_assigned_value = hovering_throttle + pid_pitch_output;
  esc4_assigned_value = hovering_throttle - pid_roll_output;


  // For debug
  // Serial.print(esc1_assigned_value); Serial.print("\t");
  // Serial.print(esc3_assigned_value); Serial.print("\t");
  // Serial.print(esc2_assigned_value); Serial.print("\t");
  // Serial.print(esc4_assigned_value); Serial.print("\t");


  esc1_output_mapped = map(esc1_assigned_value, 0, 130, 1000, 2000);
  // Serial.print(esc1_output_mapped); Serial.print("\t");

  esc3_output_mapped = map(esc3_assigned_value, 0, 130, 1000, 2000);
  // Serial.print(esc3_output_mapped); Serial.print("\t");

  esc2_output_mapped = map(esc2_assigned_value, 0, 130, 1000, 2000);
  // Serial.print(esc2_output_mapped); Serial.print("\t");

  esc4_output_mapped = map(esc4_assigned_value, 0, 130, 1000, 2000);
  // Serial.print(esc4_output_mapped); Serial.print("\t");

  ESC1.writeMicroseconds(esc1_output_mapped);
  ESC3.writeMicroseconds(esc3_output_mapped);
  ESC2.writeMicroseconds(esc2_output_mapped);
  ESC4.writeMicroseconds(esc4_output_mapped);

  Serial.print("\r\n");
  //delay(2);
}


