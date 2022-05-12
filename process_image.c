#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>

#define	COLOR_TRESHOLD 			16
#define IMAGE_BUFFER_SIZE		640

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static bool capture= false;
static bool butine= false;
static uint16_t mean  = 0;

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}
static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	//po8030_advanced_config(FORMAT_RGB565, 288, 216, 351, 48, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
    	if(capture)//capture)
    	{
			//starts a capture
			dcmi_capture_start();
			//waits for the capture to be done
			wait_image_ready();
			//signals an image has been captured
			chBSemSignal(&image_ready_sem);
    	}
    	chThdSleepMilliseconds(100);
    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint16_t var  = 0;
	bool send_to_computer = true;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	//bool butine = false;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			//var = (uint8_t)img_buff_ptr[i]&0xF8;
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
			var += image[i/2];
			if (var > 200)
			{
				//fleur(true);
				butine = true;
				chprintf((BaseSequentialStream *)&SD3, "var  =  %d\n", var);
			}

			//image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		mean = var/IMAGE_BUFFER_SIZE;

		if(send_to_computer){
					//sends to the computer the image
					SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
				}
				//invert the bool
				send_to_computer = !send_to_computer;



		capture=false;
		chThdSleepMilliseconds(100);
    }
}


void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}

void set_capture(void)
{
	capture=true;
}
void set_butine(void)
{
	butine = false;
}
bool fleur (void ){
	if(butine){
		chprintf((BaseSequentialStream *)&SD3, "true");
	}
	else{
		chprintf((BaseSequentialStream *)&SD3, "false");
	}

	return butine;
}
