#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define ROTATION_THRESHOLD		30
#define ROTATION_COEFF			2
#define GOAL_ANGLE 				0.0f
#define ERROR_THRESHOLD			0.1	//[de l'erreur d'angle
#define KP						150.0f
#define KI 						0.05f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/10)
#define LED_FRONT 			0
#define LED_BACK			2
#define LED_LEFT			3
#define LED_RIGHT			1
/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
