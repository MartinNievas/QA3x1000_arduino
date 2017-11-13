#include <Arduino.h>
#include <Wire.h>
#include <I2C.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <DigitalToggle.h>
#include <PID_v1.h>


extern Kalman kalmanX; // Create the Kalman instances
extern Kalman kalmanY;

extern uint8_t i2cData[14]; // Buffer for I2C data
extern double accX, accY, accZ;
extern double gyroX, gyroY, gyroZ;
extern int16_t tempRaw;
extern double accX, accY, accZ;
extern double gyroX, gyroY, gyroZ;
extern int16_t tempRaw;

extern double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

extern uint32_t timer;
extern double dt;

extern double roll;
extern double pitch;
extern double gyroXrate;
extern double gyroYrate;

extern volatile int prev_time;

extern int joystick_roll_pwm_value;
extern int joystick_pitch_pwm_value;
extern int joystick_throttle_pwm_value;

extern char str[30];

void config_i2cData(void){
  i2cData[0] = 7;    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));        

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

}


void kalman_set_initial_angle(void){

  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;

  kalmanX.setAngle(roll); 
  kalmanY.setAngle(pitch);
}


void get_IMU_data(void){
  digitalToggle(8);
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
}

void update_time(void){

  dt = (double)(micros() - timer) / 1000000; 
  timer = micros();

}

void compute_roll_pitch(void){

  roll  = atan2(accY, accZ) * RAD_TO_DEG;
  pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  gyroXrate = gyroX / 131.0; // Convert to deg/s
  gyroYrate = gyroY / 131.0; // Convert to deg/s
}

void check_and_fix_transition(void){
// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
  } 
  else{
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  }
  if (abs(kalAngleX) > 90){
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  }

  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

}

