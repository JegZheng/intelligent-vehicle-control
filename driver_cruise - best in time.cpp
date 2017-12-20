/*
WARNING !

DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

char* find_track(char *tmp) {
	char line[60];
	FILE *f;
	//string dir = GetLocalDir();
	f = fopen("config//raceman//cybercruise.xml", "rt");
	int flag = 0;
	//char *tmp;
	while (fgets(line, 60, f) != NULL)
	{
		char *ptr = strstr(line, "Tracks");
		if (ptr != NULL) {
			flag = 1;
			continue;
		}
		if (strstr(line, "name=\"name\"") != NULL && flag == 1) {
			//strstr(line, "name=\"name\"");
			char *delim = "val=";
			char *start;
			start = strstr(line, delim) + 5;
			char *end;
			end = strstr(line, "\"/>");
			strncpy(tmp, start, end - start);
			tmp[end - start] = '\0';
			flag = 0;
			printf("%ld\n", end - start);
			fclose(f);
			break;
		}
	}
	printf("%s\n", tmp);
	return tmp;
}

//char *track_s;
//char *track = find_track(track_s);

#define PI 3.141592653589793238462643383279

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
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;//
static double radius[21];
static int roadTypeJudge = 0;
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
int delta = 20;//


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
	//track = find_track(track_s);
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
	float k, t;
	k = 0;
	t = 0;
	int t_point;
	if (_width > 16) _width = 16;
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
		if (_speed < 20) { *cmdGear = 1; *cmdAcc = 1; *cmdBrake = 0; *cmdSteer = _yaw / 10; ++roadTypeJudge; }
		else {
			if (roadTypeJudge <= 91) {
				if (_speed < 50) { t_point = (0, 15, _speed * 0.3); }
				else { t_point = constrain(15, 200, 0.01*_speed*(_width - sqrt(abs(_width - 10)) + 20)); } // 这两个经验公式里面的参数可以调一下试试
				kp_d = 1.4;
				ki_d = 0;
				kd_d = 0;

				//get the error 

				D_err = _yaw - 5 * atan2((_midline[t_point][0]), _midline[t_point][1]);//only track the aiming point on the middle line

																					   //the differential and integral operation 
				D_errDiff = D_err - Tmp;
				D_errSum = D_errSum + D_err;
				Tmp = D_err;


				//set the error and get the cmdSteer
				*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
				//updateGear(cmdGear);
				if (_rpm > 700 && (_speed > 58 * *cmdGear - *cmdGear**cmdGear) && *cmdGear <= 6)    *cmdGear = *cmdGear + 1;
				if ((_speed < 50 * (*cmdGear - 1)) && *cmdGear >= 2)   *cmdGear = *cmdGear - 1;

				startPoint = int(_speed*_speed*0.00015);//speed target
				if (_speed>300) startPoint = int(_speed*_speed*0.00016);
				startPoint = constrain(0, 20, startPoint);
				for (int j = 0; j < startPoint + 1; j++) { if (t < 1 / fabs(radius[j]))  t = 1 / fabs(radius[j]); } //approximation of 1/radius
				for (int j = 1; j < startPoint + 1; j++) { if (k < 1 / fabs(radius[j]))  k = 1 / fabs(radius[j]); }
				if (_speed < 40) { *cmdAcc = 1; *cmdBrake = 0; *cmdSteer = *cmdSteer / 3; }
				else
				{
					if (_width < 10) { expectedSpeed = k * _speed * _speed / sqrt(_width); }
					else {
						expectedSpeed = k * _speed * _speed / (sqrt(1.5*sqrt(_width - 10) + 10));
					}

					if (expectedSpeed < 110)
					{
						*cmdAcc = constrain(0, 1, 1 - *cmdSteer * *cmdSteer* _speed* t * (0.0002*_speed*_speed*abs(atan2(_midline[t_point][0], _midline[t_point][1])) + 15 * t));
						*cmdBrake = 0;

					}
					else
					{
						*cmdAcc = 0; *cmdBrake = 1;
					}

				}
				//updateGear(cmdGear);

			}
			else { // dirt
				if (_speed < 50) { t_point = constrain(0, 20, _speed * 0.3); }
				else { t_point = constrain(50, 200, 0.01*_speed*(_width - sqrt(abs(_width - 15)) + 30)); } // 这两个经验公式里面的参数可以调一下试试
				kp_d = 1;
				ki_d = 0;
				kd_d = 0;

				//get the error 

				D_err = _yaw - 8 * atan2((_midline[t_point][0]), _midline[t_point][1]);//only track the aiming point on the middle line

																					   //the differential and integral operation 
				D_errDiff = D_err - Tmp;
				D_errSum = D_errSum + D_err;
				Tmp = D_err;


				//set the error and get the cmdSteer
				*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
				if (_rpm > 700 && (_speed > 53 * *cmdGear) && *cmdGear <= 4)    *cmdGear = *cmdGear + 1;//gear
				if ((_speed < 48 * (*cmdGear - 1)) && *cmdGear >= 2)   *cmdGear = *cmdGear - 1;
				//updateGear(cmdGear);
				if (_speed < 50) { *cmdAcc = 1; *cmdBrake = 0; }
				else
				{
					startPoint = int(_speed*_speed*0.00009);//target
					startPoint = constrain(0, 6, startPoint);
					for (int j = 0; j < startPoint + 1; j++) { if (t < 1 / fabs(radius[3 * j]))  t = 1 / fabs(radius[3 * j]); }
					expectedSpeed = t * _speed * _speed * _speed / (-sqrt(abs(_width - 15)) + _width);
					if (expectedSpeed < 2800) { *cmdAcc = 1 - *cmdSteer**cmdSteer; *cmdBrake = 0; }
					else { *cmdAcc = 0; *cmdBrake = 1; }

				}
				//updateGear(cmdGear);

			}


		}


		//print some useful info on the terminal
		printf("steer %f radius %f  acc %f brake %f roadTypeJudge%d\n", *cmdSteer, radius[0], *cmdAcc, *cmdBrake, roadTypeJudge);
		/******************************************End by Yuan Wei********************************************/
	}
}

void PIDParamSetter()
{

	kp_s = 0.02;
	ki_s = 0;
	kd_s = 0;
	kp_d = 1;
	ki_d = 0;
	kd_d = 0;
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
	r = constrain(1.0, 5000000000.0, r);
	int sign = (x>0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}

