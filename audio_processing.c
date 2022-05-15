#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];

static float angle_direction;
static float angle_direction_old;
static int16_t freq_index;

#define MIN_VALUE_THRESHOLD	10000 
#define MIN_FREQ			10	//we don't analyze before this index to not use resources for nothing
#define MAX_FREQ			30	//we don't analyze after this index to not use resources for nothing

#define ANGLE_THRESHOLD 	1
#define CONSTANTE_CONVERT	2.76

//==================================================================================================================
//PRIVATE FUNCTIONS
//==================================================================================================================

// extracts the highest frequency index.
uint16_t frequence_max (float* micro_left_fft, float* micro_right_fft){

	float max_norm_left = MIN_VALUE_THRESHOLD;
	float max_norm_right = MIN_VALUE_THRESHOLD;

	int16_t max_norm_index_left = -1;
	int16_t max_norm_index_right = -1;

	// max index for left micro.
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(micro_left_fft[i] > max_norm_left){
			max_norm_left = micro_left_fft[i];
			max_norm_index_left = i;
		}
	}

	// max index for right micro.
		for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
			if(micro_right_fft[i] > max_norm_right){
				max_norm_right = micro_right_fft[i];
				max_norm_index_right = i;
			}
		}

	//check that the indices are the same for the micro.
	if((max_norm_index_left == max_norm_index_right) && (max_norm_right > MIN_VALUE_THRESHOLD )
			&& (max_norm_left > MIN_VALUE_THRESHOLD )){
		return max_norm_index_left;
	}
	else {
		return 0;
	}
}

//Calculation of the angle direction.
void angle_calculation(uint16_t freq_max){

	float phase_left  = 0;
	float phase_right = 0;

	phase_left = atan2f(micLeft_cmplx_input[2*freq_max+1],micLeft_cmplx_input[2*freq_max]);
	phase_right = atan2f(micRight_cmplx_input[2*freq_max+1],micRight_cmplx_input[2*freq_max]);

	angle_direction = (phase_right - phase_left) * CONSTANTE_CONVERT ;

	//check if the angle is similar to the old one.
	if((angle_direction > angle_direction_old + ANGLE_THRESHOLD)||(angle_direction < angle_direction_old - ANGLE_THRESHOLD)) {
		angle_direction = angle_direction_old;
	}
	else {
		angle_direction_old = angle_direction;
	}
}

//==================================================================================================================
//PUBLIC FUNCTIONS
//==================================================================================================================

//inits the audio angles.
void audio_init(void){
	angle_direction = 0;
	angle_direction_old = 0;
}

void processAudioData(int16_t *data, uint16_t num_samples){

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){

		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);


		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;

		// stock the index of the perceived frequency
		freq_index = frequence_max(micLeft_output, micRight_output);
		//update angle direction
		angle_calculation(freq_index);
	}
}

//returns the angle direction
float get_angle(void){
	return angle_direction;
}

//returns the frequency index
int16_t get_freq_index(void){
	return freq_index;
}

