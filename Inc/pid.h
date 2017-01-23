#ifndef PID_H_
#define PID_H_

typedef struct pid_params
{
	float kp;
	float ki;
	float kd;
	float err;
	float err_sum;
	float err_last;
	 float maxOutput;
}pid_params;




void PID_init(float kp, float ki, float kd,float maxOutput,pid_params *pid_params);
float PID_calculate(float set_val, float read_val, pid_params *pPid_params);

#endif /* PID_H_ */
