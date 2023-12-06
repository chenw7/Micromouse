/*
 * pid.c
 */

#include "main.h"
#include "motors.h"
#include "encoders.h"

int angleError = 0;
int oldAngleError = 0;
float distanceError = 0;
float oldDistanceError = 0;
float kPw = 0.0015;
float kDw = 0.0015;
float kPx = 0.0013;
float kDx = 0.0013;

float goalDistance = 0;
float goalAngle = 0;

float angleCorrection, distanceCorrection;
float LPWM = 0;
float RPWM = 0;
int ticksElapsed = 0;

void resetPID() {
	/*
	 * For assignment 3.1: This function does not need to do anything
	 * For assignment 3.2: This function should reset all the variables you define in this file to help with PID to their default
	 *  values. You should also reset your motors and encoder counts (if you tell your rat to turn 90 degrees, there will be a big
	 * difference in encoder counts after it turns. If you follow that by telling your rat to drive straight without first
	 * resetting the encoder counts, your rat is going to see a huge angle error and be very unhappy).
	 *
	 * You should additionally set your distance and error goal values (and your oldDistanceError and oldAngleError) to zero.
	 */
	angleError = 0;
	oldAngleError = 0;
	distanceError = 0;
	oldDistanceError = 0;
	angleCorrection = 0;
	distanceCorrection = 0;
	resetEncoders();
	resetMotors();

	goalDistance = 0;
	goalAngle = 0;
	ticksElapsed = 0;
	LPWM = 0;
	RPWM = 0;
}

void updatePID() {
	/*
	 * This function will do the heavy lifting PID logic. It should do the following: read the encoder counts to determine an error,
	 * use that error along with some PD constants you determine in order to determine how to set the motor speeds, and then actually
	 * set the motor speeds.
	 *
	 * For assignment 3.1: implement this function to get your rat to drive forwards indefinitely in a straight line. Refer to pseudocode
	 * example document on the google drive for some pointers
	 *
	 * TIPS (assignment 3.1): Create kPw and kDw variables, and use a variable to store the previous error for use in computing your
	 * derivative term. You may get better performance by having your kDw term average the previous handful of error values instead of the
	 * immediately previous one, or using a stored error from 10-15 cycles ago (stored in an array?). This is because systick calls so frequently
	 * that the error change may be very small and hard to operate on.
	 *
	 * For assignment 3.2: implement this function so it calculates distanceError as the difference between your goal distance and the average of
	 * your left and right encoder counts. Calculate angleError as the difference between your goal angle and the difference between your left and
	 * right encoder counts. Refer to pseudocode example document on the google drive for some pointers.
	 */
	 angleError = goalAngle - (getRightEncoderCounts() - getLeftEncoderCounts());
	 angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
	 oldAngleError = angleError;

	 distanceError = goalDistance - (getRightEncoderCounts() + getLeftEncoderCounts())/2;
	 distanceCorrection = kPx * distanceError + kDx * (distanceError - oldDistanceError);
	 oldDistanceError = distanceError;

	 LPWM = distanceCorrection - angleCorrection;
	 RPWM = distanceCorrection + angleCorrection;

	 if(LPWM < 0.3 && LPWM > 0){
		 LPWM = 0.35;
	 }
	 else if(LPWM > -0.3 && LPWM < 0){
		 LPWM = -0.35;
	 }

	 if(RPWM < 0.3 && RPWM > 0){
	 	RPWM = 0.35;
	 }
	 else if(RPWM > -0.3 && RPWM < 0){
	 	RPWM = -0.35;
	 }

	 setMotorLPWM(LPWM);
	 setMotorRPWM(RPWM);
}

void setPIDGoalD(int16_t distance) {
	/*
	 * For assignment 3.1: this function does not need to do anything.
	 * For assignment 3.2: this function should set a variable that stores the goal distance.
	 */
	goalDistance = distance;
}

void setPIDGoalA(int16_t angle) {
	/*
	 * For assignment 3.1: this function does not need to do anything
	 * For assignment 3.2: This function should set a variable that stores the goal angle.
	 */
	 goalAngle = angle;
}

int8_t PIDdone(void){
	if(angleError <= 10 && angleError >= -10 && distanceError <= 10 && distanceError >= -10) {
		ticksElapsed++;
	}
	else{
		ticksElapsed = 0;
	}

	return ticksElapsed == 500000;
}
