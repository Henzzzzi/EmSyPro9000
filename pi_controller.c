//	EmSyPro9000

//#include <stdlib.h>

#define PI_SATURATION 1000

// Re-entrant PI controller
double PI(double u_ref, double u_act, double* pui_prev, double Ki, double Kp){
	//static double ui_prev = 0;
	double error, up, ui, ui_prev, u_out;
	ui_prev = *pui_prev;
	error = u_ref - u_act;
	up = Kp * error;
	ui = Ki * error*0.00002 + ui_prev;
	ui_prev = ui;
	u_out = up + ui;

	if (u_out > PI_SATURATION){		// Saturation
		u_out = PI_SATURATION;
		ui_prev = 0;				// Anti wind-up
	}
	else if (u_out < 0){
		u_out = 0;
	}
	*pui_prev = ui_prev;			// Update integrator value to pointer address

	return u_out;
}
