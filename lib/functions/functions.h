#ifndef functions_h
#define functions_h



void config_i2cData(void);

void kalman_set_initial_angle(void);

void get_IMU_data(void);

void update_time(void);

void compute_roll_pitch(void);

void check_and_fix_transition(void);


#endif
