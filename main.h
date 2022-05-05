#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera
#define KP						800.0f
#define KI 						3.5f	//must not be zero
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define GOAL_DISTANCE 			10.0f
#define IMAGE_BUFFER_SIZE		640

#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_ANGLE 				0.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1	//[de l'erreur d'angle
#define KP						150.0f
#define KI 						0.05f	//must not be zero
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/10) //KI)

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
