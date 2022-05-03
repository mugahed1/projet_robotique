#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include "sensors/VL53L0X/VL53L0X.h"

#include <main.h>
#include <motors.h>
#include <motor_configuration.h>

#define COLOR_THRESHOLD	16
static uint16_t distance=0;

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

void set_speed(void)
{
	int speed =0;
	distance = VL53L0X_get_dist_mm();
	chprintf((BaseSequentialStream *)&SD3, "distance = %d \n",distance);
	if(distance<200)
	{
		speed=0;
	}else{speed = 1000;}
	right_motor_set_speed(speed);
	left_motor_set_speed(speed);
}

void color_detection(void)
{
	//Takes pixels 288 to 352 of the line 216 to 264 (48 lines)
	po8030_advanced_config(FORMAT_RGB565, 288, 216, 352, 48, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

	uint8_t image[3072] = {0};
	uint8_t *img_buff_ptr;
	uint32_t mean = 0;

	dcmi_capture_start(); //starts a capture
	wait_image_ready();	//waits for the capture to be done
	img_buff_ptr = dcmi_get_last_image_ptr();

	//Extracts only the red pixels
	for(uint16_t i = 0 ; i < (2 * 3072) ; i+=2)
	{
		//extracts first 5bits of the first byte
		//takes nothing from the second byte
		image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		mean += image[i/2];
	}
	mean /= 3072;
	if(mean> COLOR_THRESHOLD)
	{
		chprintf((BaseSequentialStream *)&SD3, "mean = %d \n",mean);
		//robot movement for black color
	}
	else
	{
		chprintf((BaseSequentialStream *)&SD3, "mean = %d \n",mean);
		//robot movement for white color
	}
}
