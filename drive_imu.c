/*******************************************************************************
* drive_simple.c
*
* 
*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <getopt.h>

#include <lcm/lcm.h>
#include "lcmtypes/mbot_encoder_t.h"
#include "lcmtypes/mbot_imu_t.h"
#include "lcmtypes/simple_motor_command_t.h"

#include <rc/start_stop.h>
#include <rc/encoder_eqep.h>
#include <rc/mpu.h>
#include <rc/time.h>
#include <rc/motor.h>

//LCM
lcm_t * lcm;
#define MBOT_ENCODER_CHANNEL                "MBOT_ENCODERS"
#define MBOT_MOTOR_COMMAND_SIMPLE_CHANNEL   "MBOT_MOTOR_COMMAND_SIMPLE"
#define ENCODER_CONVERSION		    78
#define MBOT_IMU_CHANNEL		    "MBOT_IMU"

#define P_gain 2.5
#define I_gain 25
#define D_gain 0.015

typedef struct PID_type {
    double Kp;
    double Ki;
    double Kd;
    double prev_error;
    double integral;
    double dt;
    double lower_saturation;
    double upper_saturation;
    uint8_t saturation;
    double prev_time;
} PID;

void PID_Init(PID *pid, double Kp, double Ki, double Kd);
void PID_EnableSaturation(PID *pid, double lower, double upper);
double PID_March(PID *pid, double error);

//global watchdog_timer to cut off motors if no lcm messages recieved
float watchdog_timer;

//functions
void publish_imu_data();
void publish_encoder_msg();
void print_answers();
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char* channel, const simple_motor_command_t * msg, void * user);
void drive_forward();
void change_pwm_for_drive();

// For testing PID - comment out the line connecting simple_motor_controller to here.
double left_goal = 0;
double right_goal = 0;

// Get current forward and angular velocities and accelerations for PD loop
double left_current = 0;
double right_current = 0;

int mot_l_pol;
int mot_r_pol;
int enc_l_pol;
int enc_r_pol;

PID left_pid;
PID right_pid;

rc_mpu_data_t imu_data;

/*******************************************************************************
* int main() 
*
*******************************************************************************/
int main(int argc, char *argv[]){
    //check args
    if( argc != 5 ) {
        printf("Usage: test_simple {left motor polarity} {right motor polarity} {left encoder polarity} {right encoder polarity} P I D\n");
        printf("Example: test_simple -1 1 -1 1 0.1 0.001 0.001\n");
        return 0;
    }
    
    mot_l_pol = atoi(argv[1]);
    mot_r_pol = atoi(argv[2]);
    enc_l_pol = atoi(argv[3]);
    enc_r_pol = atoi(argv[4]);

    if( ((mot_l_pol != 1)&(mot_l_pol != -1)) |
        ((mot_r_pol != 1)&(mot_r_pol != -1)) |
        ((enc_l_pol != 1)&(enc_l_pol != -1)) |
        ((enc_r_pol != 1)&(enc_r_pol != -1))){
        printf("Usage: polarities must be -1 or 1\n");
        return 0;
    }

	// make sure another instance isn't running
    if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
    if(rc_enable_signal_handler()==-1){
    	fprintf(stderr,"ERROR: failed to start signal handler\n");
        return -1;
    }

	if(rc_motor_init()<0){
        fprintf(stderr,"ERROR: failed to initialze motors\n");
        return -1;
    }

	rc_mpu_config_t imu_config = rc_mpu_default_config();
	imu_config.dmp_sample_rate = 50;
	imu_config.dmp_fetch_accel_gyro=1;

	if(rc_mpu_initialize_dmp(&imu_data, imu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU! Exiting.\n");
		return -1;
	}

	rc_mpu_set_dmp_callback(publish_imu_data);

	//rc_mpu_calibrate_accel_routine(rc_mpu_default_config());
	//rc_mpu_calibrate_gyro_routine(rc_mpu_default_config());

    lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");

    // Initialize the PID controllers
    PID_Init(&left_pid, P_gain, I_gain, D_gain);
    PID_Init(&right_pid, P_gain, I_gain, D_gain);

    // Enable saturation to make sure PWM does not exceed max
    PID_EnableSaturation(&left_pid, -1, 1);
    PID_EnableSaturation(&right_pid, -1, 1);

    // make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	// rc_make_pid_file();

	// done initializing so set state to RUNNING
    rc_encoder_eqep_init();
    rc_set_state(RUNNING);
    
    watchdog_timer = 0.0;
    
    // Uncomment this line to enable forward velocity from pi
    simple_motor_command_t_subscribe(lcm, "MBOT_MOTOR_COMMAND_SIMPLE", &simple_motor_command_handler, NULL);
    printf("Running...\n");
	while(rc_get_state()==RUNNING){
        watchdog_timer += 0.01;
        if(watchdog_timer >= 0.25)
        {
	    left_goal = 0;
	    right_goal = 0;
            printf("timeout...\r");
        }
		// define a timeout (for erroring out) and the delay time
        publish_encoder_msg();
        change_pwm_for_drive();
        lcm_handle_timeout(lcm, 1);
        rc_nanosleep(1E9 / 100); //handle at 10Hz
	}
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
	rc_mpu_power_off();
    lcm_destroy(lcm);
	// rc_remove_pid_file();   // remove pid file LAST

    return 0;
}


void change_pwm_for_drive()
{
    // Define proportional error for each motor
    double left_error = left_goal - left_current;
    double right_error = right_goal - right_current;

    // Calculate error
    double left_pwm = PID_March(&left_pid, left_error);
    double right_pwm = PID_March(&right_pid, right_error);

    //printf("left out = %f, right out = %f\n", left_pwm, right_pwm);

    // Set PWM values
    rc_motor_set(1, mot_l_pol * left_pwm);
    rc_motor_set(2, mot_r_pol * right_pwm);
}


/*******************************************************************************
*  simple_motor_command_handler()
*
*  sets motor PWMS from incoming lcm message
*
*******************************************************************************/
//////////////////////////////////////////////////////////////////////////////
/// TODO: Create a handler that receives lcm message simple_motor_command_t and
/// sets motor PWM according to the recieved message.
/// command the motor using the command: rc_motor_set(channel, polarity * pwm);
/// for now the pwm value should be proportional to the velocity you send, 
/// the sign of the velocity should indicate direction, and angular velocity 
//  indicates turning rate. 
//////////////////////////////////////////////////////////////////////////////
void simple_motor_command_handler(const lcm_recv_buf_t *rbuf, const char* channel, const simple_motor_command_t * msg, void * user) 
{
	watchdog_timer = 0.0;
	left_goal = msg->forward_velocity - 0.055 * msg->angular_velocity;
	right_goal = msg->forward_velocity + 0.055 * msg->angular_velocity;
	
}

void publish_imu_data() {
	mbot_imu_t imu_msg;
	imu_msg.utime = rc_nanos_since_epoch();
	
	imu_msg.accel[0] = imu_data.accel[0];
	imu_msg.accel[1] = imu_data.accel[1];
	imu_msg.accel[2] = imu_data.accel[2];

	imu_msg.gyro[0] = imu_data.gyro[0] * DEG_TO_RAD;
	imu_msg.gyro[1] = imu_data.gyro[1] * DEG_TO_RAD;
	imu_msg.gyro[2] = imu_data.gyro[2] * DEG_TO_RAD;

    	mbot_imu_t_publish(lcm, MBOT_IMU_CHANNEL, &imu_msg);
}
/*******************************************************************************
* void publish_encoder_msg()
*
* publishes LCM message of encoder reading
* 
*******************************************************************************/
void publish_encoder_msg(){

    static int64_t prev_leftticks = 0, prev_rightticks = 0;
    static uint64_t lasttime = 0;

    mbot_encoder_t encoder_msg;
    encoder_msg.utime = rc_nanos_since_epoch();
    encoder_msg.leftticks = enc_l_pol * rc_encoder_eqep_read(1);
    encoder_msg.rightticks = enc_r_pol * rc_encoder_eqep_read(2);
    encoder_msg.left_delta = encoder_msg.leftticks - prev_leftticks;
    encoder_msg.right_delta = encoder_msg.rightticks - prev_rightticks;

    double time_delta = (double)(encoder_msg.utime - lasttime) / 1000000000.0;
    lasttime = encoder_msg.utime;
    prev_leftticks = encoder_msg.leftticks;
    prev_rightticks = encoder_msg.rightticks;

    double forward_vel = ((encoder_msg.left_delta + encoder_msg.right_delta) / 2.0) * (2 * 3.14159265 * 0.042) / (20 * ENCODER_CONVERSION);
    double angular_vel = ((encoder_msg.right_delta - encoder_msg.left_delta) / 0.11) * ((2 * 3.14159265 * 0.042) / (20 * ENCODER_CONVERSION));
    forward_vel = (forward_vel / (float)(time_delta));
    angular_vel = (angular_vel / (float)(time_delta));
    
    // Estimate left and right forward velocities
    left_current = ((2.0*3.1415*0.042)/(20*ENCODER_CONVERSION))*(encoder_msg.left_delta)*(1.0/(float)(time_delta));
    right_current = ((2.0*3.1415*0.042)/(20*ENCODER_CONVERSION))*(encoder_msg.right_delta)*(1.0/(float)(time_delta));

    // printf(" ENC: %lld | %lld  - v: %f | w: %f \r", encoder_msg.leftticks, encoder_msg.rightticks, forward_vel, angular_vel);
    
    //printf("Forward velocity: %f, Angular velocity: %f \r", forward_vel, angular_vel);
    
    //mbot_encoder_t_publish(lcm, MBOT_ENCODER_CHANNEL, &encoder_msg);
}

void PID_Init(PID *pid, double Kp, double Ki, double Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->prev_time = rc_nanos_since_boot() / 1.0e9;
    pid->saturation = 0;
}

void PID_EnableSaturation(PID *pid, double lower, double upper) {
    pid->saturation = 1;
    pid->lower_saturation = lower;
    pid->upper_saturation = upper;
}

double PID_March(PID *pid, double error) {
    double curr_time = rc_nanos_since_boot() / 1.0e9;
    pid->integral += error * (curr_time - pid->prev_time);
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * (error - pid->prev_error) / (curr_time - pid->prev_time);
    pid->prev_error = error;
    pid->prev_time = curr_time;

    if (pid->saturation) {
        output = (output > pid->upper_saturation) ? pid->upper_saturation : output;
        output = (output < pid->lower_saturation) ? pid->lower_saturation : output;
    }
    
    return output;
}
