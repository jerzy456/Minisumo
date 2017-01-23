#include "pid.h"
#include "math.h"

#define ERR_SUM_MAX		1000




void PID_init(float kp, float ki, float kd,float maxOutput, pid_params *pPid_params)
{

	pPid_params->kp = kp;
	pPid_params->ki = ki;
	pPid_params->kd = kd;
	pPid_params->err = 0;
	pPid_params->err_sum = 0;
	pPid_params->err_last = 0;
	pPid_params->maxOutput=maxOutput;
}

float PID_calculate(float set_val, float read_val, pid_params *pPid_params)
{
	float err_d, u;

	pPid_params->err = set_val - read_val;
	pPid_params->err_sum += pPid_params->err;

	if (pPid_params->err_sum > ERR_SUM_MAX) {
		pPid_params->err_sum = ERR_SUM_MAX;
	} else if (pPid_params->err_sum < -ERR_SUM_MAX) {
		pPid_params->err_sum = -ERR_SUM_MAX;
	}

	err_d = pPid_params->err_last - pPid_params->err;
	u = pPid_params->kp * pPid_params->err + pPid_params->ki * pPid_params->err_sum
			+ pPid_params->kd * err_d;
	if(u>pPid_params->maxOutput)u=pPid_params->maxOutput;
	if(u<-pPid_params->maxOutput)u=-pPid_params->maxOutput;


	return u;
}
