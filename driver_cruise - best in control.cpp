/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>

#define PI 3.141592653589793238462643383279

char *track_s;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;

	char line[60];
	FILE *f;
	//string dir = GetLocalDir();
	f = fopen("config//raceman//cybercruise.xml", "rt");
	int flag = 0;
	char *tmp;
	while (fgets(line, 60, f) != NULL)
	{
		char *ptr = strstr(line, "Tracks");
		if (ptr != NULL) {
			flag = 1;
			continue;
		}
		if (strstr(line, "name=\"category\"") != NULL && flag == 1) {
			//strstr(line, "name=\"name\"");
			char *delim = "val=";
			char *start;
			start = strstr(line, delim) + 5;
			char *end;
			end = strstr(line, "\"/>");
			strncpy(tmp, start, end - start);
			tmp[end - start] = '\0';
			flag = 0;
			break;
		}
	}
	printf("%s\n", tmp);
	//strcpy(track_s, tmp);
	//fclose(f);
	//free(f);
	delete(f);
	track_s = tmp;
	printf("--------------------------------------------------------\n");
	printf("%s\n", track_s);
	printf("--------------------------------------------------------\n");
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}


/*
WARNING!

DO NOT MODIFY CODES ABOVE!
*/

//**********Global variables for vehicle states*********//
static double radius[21];
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
														//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


														//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
														//******************************************************//

														//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //
				// Direction Control Variables						         //
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum = 0;//sum of direction error(Integration)      //
					// Speed Control Variables								     //
circle c;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum = 0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta = 20;												 //
															 //***********************************************************//

															 //*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag = true;											//
double offset = 0;										//
double Tmp = 0;
//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int *cmdGear);													//
																				// Function constrain:															//
																				//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary, double input);		//
																				// Function getR:																//
																				//		Given three points ahead, outputs a struct circle.						//
																				//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);		//
																				//******************************************************************************//

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */
	int j = 0;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	
	while (j < 21) {
		c = getR(_midline[10 * j][0], _midline[10 * j][1], _midline[10 * (j + 1)][0], _midline[10 * (j + 1)][1], _midline[10 * (j + 2)][0], _midline[10 * (j + 2)][1]);
		radius[j] = c.r;
		j++;
	}
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	if (parameterSet == false)		// Initialization Part
	{
		PIDParamSetter();
	}
	else
	{
		// Speed Control
		/*
		You can modify the limited speed in this module
		Enjoy  -_-
		*/
		//printf("%s\n", track_s);
		
		if (strcmp(track_s, "dirt") == 0) {
			if (_speed<20)
			{
				expectedSpeed = 45;
				*cmdAcc = 1.0;
				*cmdBrake = 0.0;
				*cmdSteer = 0.0;
			}
			if (_speed < 130) {
				startPoint = _speed*0.3;
			}
			else {
				startPoint = _speed * 0.4;
			}
			c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
			printf("in dirt c : %f, radius: %f\n", c.r, radius[0]);
			if (c.r<40 && radius[0] < 40)
			{
				printf("1 \n");
				expectedSpeed = constrain(30, 50, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
			}
			else if (c.r<80 && radius[0] < 80)
			{
				printf("2 \n");
				expectedSpeed = constrain(45, 55, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
			}
			else if (c.r < 400 && fabs(radius[0]) > 490)
			{
				if (c.r < 80) {
					printf("3 \n");
					expectedSpeed = constrain(30, 50, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
				}
				else
				{
					printf("4 \n");
					expectedSpeed = constrain(50, 100, c.r*0.9);
				}

				//expectedSpeed = constrain(40,79,c.r*0.9);
			}
			else if (c.r > 450 && fabs(radius[0]) < 400)
			{
				if (radius[0] < 50) {
					printf("5 \n");
					expectedSpeed = constrain(30, 50, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
				}
				else {
					printf("6 \n");
					expectedSpeed = constrain(60, 110, c.r*0.9);
				}

				//expectedSpeed = constrain(40,79,c.r*0.9);
			}
			else if (c.r > 490 && fabs(radius[0]) > 490) {
				printf("gogogogo \n");
				expectedSpeed = constrain(100, 230, c.r*1.35);
			}
			else if (c.r<120&&radius[0]<120)
			{
				printf("7 \n");
				//expectedSpeed = constrain(40, 100, c.r*0.9);
				expectedSpeed = constrain(40,79,c.r*0.9);
			}
			else if (c.r<350 && c.r > 270 && radius[0] < 350 && radius[0] > 270 )
			{
				printf("8 \n");
				//expectedSpeed = constrain(40, 100, c.r*0.9);
				expectedSpeed = constrain(60,200,c.r*0.9);
			}
			else if (c.r<270 && c.r > 100 && radius[0] < 270 && radius[0] > 100 && fabs(c.r-radius[0])<60)
			{
				printf("9 new \n");
				//expectedSpeed = constrain(40, 100, c.r*0.9);
				expectedSpeed = constrain(60, 180, c.r*0.9);
			}
			else
			{
				printf("else \n");
				//expectedSpeed = constrain(50, 145, c.r*1.35);
				expectedSpeed = constrain(50,80,c.r*1.35);
			}
			curSpeedErr = expectedSpeed - _speed;
			speedErrSum = 0.1 * speedErrSum + curSpeedErr;
			if (curSpeedErr > 0)
			{

				if (abs(*cmdSteer)<0.25)
				{
					*cmdAcc = constrain(0.0, 0.8, kp_s * curSpeedErr + ki_s * speedErrSum + 0.2 * offset);
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer)<0.5)
				{
					*cmdAcc = constrain(0.0, 0.4, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else
				{
					*cmdAcc = constrain(0.0, 0.2, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}

			}
			else if (curSpeedErr < 0)
			{
				
					*cmdBrake = constrain(0.0, 1.0, -kp_s *curSpeedErr / 5 - offset / 3);
					*cmdAcc = 0;
				

			}

			updateGear(cmdGear);

			/******************************************Modified by Yuan Wei********************************************/
			/*
			Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
			Once you have chose the error model , you can rectify the value of PID to improve your control performance.
			Enjoy  -_-
			*/
			// Direction Control		
			//set the param of PID controller
			kp_d = 1.4;
			ki_d = 0;
			kd_d = 0;

			//get the error 
			offset = _midline[3][0];
			D_err = _yaw - 5 * atan2(_midline[3][0], _midline[3][1]) + 2.0* atan(20 * offset / _speed);//only track the aiming point on the middle line

																								  //the differential and integral operation 
			D_errDiff = D_err - Tmp;
			D_errSum = D_errSum + D_err;
			Tmp = D_err;

			//set the error and get the cmdSteer
			*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);

			/******************************************End by Yuan Wei********************************************/
		}
		else {
			if (_speed<20)
			{
				expectedSpeed = 45;
				*cmdAcc = 1.0;
				*cmdBrake = 0.0;
				*cmdSteer = 0.0;
			}
			//ki_s = 0.01;
			if (_speed > 200) {
				startPoint = _speed * 0.5;
			}
			else
			{
				startPoint = _speed * 0.4;
			}
			
			c = getR(_midline[startPoint][0], _midline[startPoint][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
			//c2 = getR(_midline[startPoint*2][0], _midline[startPoint*2][1], _midline[startPoint + delta][0], _midline[startPoint + delta][1], _midline[startPoint + 2 * delta][0], _midline[startPoint + 2 * delta][1]);
			printf("in road   c : %f, far target radius: %f \n", c.r, radius[0]);
			if (c.r<40 && radius[0]< 40)
			{
				printf("gogogogo1 \n");
				expectedSpeed = constrain(25, 40, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
			}
			else if (c.r<70 && radius[0]< 70)
			{
				printf("gogogogo2 \n");
				expectedSpeed = constrain(45, 65, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
			}
			else if (c.r<120 && radius[0] < 70)
			{
				printf("gogogogo3 \n");
				expectedSpeed = constrain(40, 100, c.r*0.9);
				//expectedSpeed = constrain(40,79,c.r*0.9);
			}
			else if (c.r < 300 && fabs(radius[0]) > 450)
			{
				if (c.r < 50) {
					printf("gogogogo3.5 \n");
					expectedSpeed = constrain(25, 40, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
				}
				else
				{
					printf("gogogogo4 \n");
					expectedSpeed = constrain(70, 135, c.r*0.9);
				}
				
				//expectedSpeed = constrain(40,79,c.r*0.9);
			}
			else if (c.r > 450 && fabs(radius[0]) < 400 )
			{
				if (radius[0] < 50) {
					printf("gogogogo5 \n");
					expectedSpeed = constrain(25, 60, c.r*c.r*(-0.046) + c.r*5.3 - 59.66);
				}
				else {
					printf("gogogogo6 \n");
					expectedSpeed = constrain(80, 140, c.r*0.9);
				}
				
				//expectedSpeed = constrain(40,79,c.r*0.9);
			}
			else if (c.r > 490 && fabs(radius[0]) > 490) {
				printf("gogogogo \n");
				expectedSpeed = constrain(100, 300, c.r*1.35);
			}
			else if (c.r >= 70 && radius[0]>=70 && fabs(c.r-radius[0] < 20) ){
				printf("gogogogo6.5 \n");
				expectedSpeed = constrain(100, 180, c.r*1.35);
			}
			else if (c.r<250 && c.r > 100 &&  radius[0] < 100)
			{
				printf("gogogogo7 \n");
				//expectedSpeed = constrain(40, 100, c.r*0.9);
				expectedSpeed = constrain(50, 80, c.r*0.9);
			}
			else
			{
				printf("gogogogo \n");
				expectedSpeed = constrain(70, 150, c.r*1.35);
				//expectedSpeed = constrain(50,115,c.r*1.35);
			}
			curSpeedErr = expectedSpeed - _speed;
			speedErrSum = 0.1 * speedErrSum + curSpeedErr;
			if (curSpeedErr > 0)
			{

				if (abs(*cmdSteer)<0.25)
				{
					*cmdAcc = constrain(0.0, 0.8, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else if (abs(*cmdSteer)<0.5)
				{
					*cmdAcc = constrain(0.0, 0.4, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}
				else
				{
					*cmdAcc = constrain(0.0, 0.2, kp_s * curSpeedErr + ki_s * speedErrSum + offset);
					*cmdBrake = 0;
				}

			}
			else if (curSpeedErr < 0)
			{
				
					*cmdBrake = constrain(0.0, 1.0, -kp_s *curSpeedErr / 5 - offset / 3);
					*cmdAcc = 0;
				
				
			}

			updateGear(cmdGear);

			/******************************************Modified by Yuan Wei********************************************/
			/*
			Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
			Once you have chose the error model , you can rectify the value of PID to improve your control performance.
			Enjoy  -_-
			*/
			// Direction Control		
			//set the param of PID controller
			kp_d = 1.4;
			ki_d = 0;
			kd_d = 0;

			//get the error 
			offset = _midline[3][0];
			D_err = _yaw - 5 * atan2(_midline[3][0], _midline[3][1]) + atan(20 * offset / _speed);//only track the aiming point on the middle line

																								  //the differential and integral operation 
			D_errDiff = D_err - Tmp;
			D_errSum = D_errSum + D_err;
			Tmp = D_err;

			//set the error and get the cmdSteer
			*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);

			/******************************************End by Yuan Wei********************************************/
		}
		
	}
}

void PIDParamSetter()
{

	kp_s = 0.02;
	ki_s = 0;
	kd_s = 0;
	kp_d = 1.35;
	ki_d = 0.151;
	kd_d = 0.10;
	parameterSet = true;

}

void updateGear(int *cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear >1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear >2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear >3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear >4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear >5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2*x2 + y2*y2 - x1*x1 - y1*y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3*x3 + y3*y3 - x2*x2 - y2*y2;
	x = (b*f - e*c) / (b*d - e*a);
	y = (d*c - a*f) / (b*d - e*a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x>0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}

