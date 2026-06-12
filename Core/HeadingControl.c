#include "HeadingControl.h"
#include "CarControl.h"
#include "Motors.h"

#include <math.h>

void Set_Straight(void)
{
	Motor_ResetSpeedScale();
	is_straight = 1;
	is_turn = 0;
	Org_Yaw = New_Yaw;
	HeadingPID_Reset(&headingPID);
	headingPID.CrossTrackEnable = 1;
	Odometry_Reset();
}

void keep_straight(void)
{
	float error = Org_Yaw - New_Yaw;
	float pOut;
	float iOut;
	float dError;
	float dOut;
	float correction;
	float target_angle;

	if(error > 180.0f) error -= 360.0f;
	if(error < -180.0f) error += 360.0f;

	if(fabs(error) < headingPID.Deadband)
	{
		error = 0.0f;
	}

	pOut = headingPID.Kp * error;

	if(fabs(error) < 8.0f && fabs(error) > 0.0f)
	{
		headingPID.Integral += headingPID.Ki * error;
	}
	else if(fabs(error) >= 8.0f)
	{
		headingPID.Integral *= 0.95f;
	}

	if(headingPID.Integral > headingPID.MaxI) headingPID.Integral = headingPID.MaxI;
	if(headingPID.Integral < -headingPID.MaxI) headingPID.Integral = -headingPID.MaxI;
	iOut = headingPID.Integral;

	if(headingPID.FirstRun)
	{
		headingPID.LastError = error;
		headingPID.FirstRun = 0;
	}
	dError = error - headingPID.LastError;
	headingPID.dV = (1.0f - headingPID.D_Alpha) * dError * headingPID.Kd
	               + headingPID.D_Alpha * headingPID.dV;
	dOut = headingPID.dV;
	headingPID.LastError = error;

	correction = pOut + iOut + dOut;

	if(is_up == -1)
	{
		correction = -correction;
	}

	if(headingPID.CrossTrackEnable)
	{
		Odometry_t snapshot;
		float cross_correction;

		Odometry_GetSnapshot(&snapshot);
		cross_correction = headingPID.CrossTrackKp * snapshot.x;
		if(is_up == -1)
		{
			cross_correction = -cross_correction;
		}
		correction += cross_correction;
	}

	if(correction > headingPID.MaxOut) correction = headingPID.MaxOut;
	if(correction < -headingPID.MaxOut) correction = -headingPID.MaxOut;

	target_angle = 90.0f + correction;
	headingPID.SmoothedAngle = headingPID.SmoothAlpha * target_angle
	                           + (1.0f - headingPID.SmoothAlpha) * headingPID.SmoothedAngle;

	Angle = headingPID.SmoothedAngle;
	SetSteeringAngle(Angle);
}
