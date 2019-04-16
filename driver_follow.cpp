/***************************************************************************

				 file : driver_cruise.cpp
	description : user module for CyberFollow

 ***************************************************************************/

 /*
	  WARNING !

	  DO NOT MODIFY CODES BELOW!
 */

#ifdef _WIN32
#include <windows.h>
#include <cmath>
#endif

#include "driver_follow.h"

#define PI acos(-1)

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_follow";	// name of the module (short).
	modInfo[0].desc = "user module for CyberFollower";	// Description of the module (can be long).
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

/*
	define your variables here.
	following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;

static double Leader_speed, Leader_acc;//头车速度与加速度，暂时只用了速度
const int topGear = 6;
double kp_s, ki_s, kd_s;
double kp_d, ki_d, kd_d;
double D_err, D_errDiff = 0, D_errSum = 0;
double Tmp;
//下列参数计算前车x方向的加速度和速度，y方向的速度和加速度
//由于前车x,y坐标 是在以后车为参考点的参考系 得到，所以其计算得到的(所有)速度和加速度均为相对值
double dis_previous_x = 0;
double dis_previous_y = 20;
double speed_x = 0;
double speed_y = 0;
double speed_previous_x = 0;
double speed_previous_y = 0;
double acc_x = 0;
double acc_y = 0;
double expect_speed = 0;
double dis;
double curspeed_err = 0;
double speed_errsum = 0;
double t1, t2;
double s;
double kp_b;
double ki_b;
double kd_b;
double p_brake;
double i_brake = 0;
double d_brake = 0;
double temp_brake = 0;
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;
circle c;
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);
static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	speed_x = (_Leader_X - dis_previous_x)*180;
	speed_y = (_Leader_Y - dis_previous_y) * 180;
	acc_x = (speed_x - speed_previous_x) * 50;
	acc_y = (speed_y - speed_previous_y) * 50;
	/* you can modify the print code here to show what you want */
	dis_previous_x = _Leader_X;
	dis_previous_y = _Leader_Y;
	speed_previous_x = speed_x;
	speed_previous_y = speed_y;
	printf("speed %.3fLeaderXY(%.3f, %.3f)", _speed, _Leader_X, _Leader_Y);
}
void updateGear(int *cmdGear);
double constrain(double lowerBoundary, double upperBoundary, double input);
int sgn(float a);

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	/* write your own code here */
	//纵向y控制
	//速度控制
	updateGear(cmdGear);
	kp_s = 1.75;
	ki_s = 0.01;
	kd_s = 0.2;
	kp_b = 1.75;
	ki_b = 0.1;
	kd_b = 0.2;
	dis = _Leader_Y - 11;
	expect_speed = ((_speed + speed_y) + acc_y) / (cos(atan2(speed_x, speed_y)))/1000 + dis * sgn(dis)/10;
	curspeed_err = expect_speed - _speed / 1000 * (cos(atan2(speed_x, speed_y)));
	speed_errsum = 0.1 * speed_errsum + curspeed_err;
	s = sgn(speed_y)*abs((speed_y)*(speed_y) / (2 * acc_y));

	p_brake = abs(acc_y) / 20;

	d_brake = p_brake - temp_brake;

	i_brake += 0.1*d_brake;

	temp_brake = p_brake;
	

	if (_Leader_Y + s > 10.2)
	{
		if (_Leader_Y > 12)
			kd_s = 1;
		*cmdAcc = constrain(0, 1, kp_s*expect_speed + ki_s * speed_errsum + kd_s * curspeed_err);
		*cmdBrake = 0;
	}
	else
	{
		*cmdAcc = 0;
		*cmdBrake = constrain(0.1, 1, kp_b*p_brake + ki_b * i_brake + kd_b * d_brake);
	}

	{//计算方向PID
		kp_d = 50;							//拐弯不行  还需改进
		ki_d = 0.1;
		kd_d = 100;
		D_err = -atan2(_Leader_X, _Leader_Y) / 10 * PI;
		D_errDiff = D_err - Tmp;
		D_errSum = D_errSum + D_err;
		Tmp = D_err;
		*cmdSteer = constrain(-1.0, 1.0, kp_d * D_err + ki_d * D_errSum + kd_d * D_errDiff);
	}

	printf("*cmdAcc%.3f*cmdsteer%.3f*cmdbrake%.3fwatch1:%0.3fwatch2:%.3fwatch3:%.3f\n", *cmdAcc, *cmdSteer, *cmdBrake, i_brake, acc_y, speed_y);

}
void updateGear(int *cmdGear)
{
	if (*cmdGear == 1)
	{
		if (_speed >= 60 && topGear > 1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (*cmdGear == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 105 && topGear > 2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (*cmdGear == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 145 && topGear > 3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (*cmdGear == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 187 && topGear > 4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (*cmdGear == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 234 && topGear > 5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (*cmdGear == 6)
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
int sgn(float a)
{
	if (a >= 0) return 1;
	else return -1;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b*f - e * c) / (b*d - e * a);
	y = (d*c - a * f) / (b*d - e * a);
	r = sqrt((x - x1)*(x - x1) + (y - y1)*(y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}