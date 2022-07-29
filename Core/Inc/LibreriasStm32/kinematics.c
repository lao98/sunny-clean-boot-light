#include "include/kinematics.h"
/**
 * @brief important to add variables to print jut when you need to debug,
 * the idea is not print in execution
 *
 */

#define DEBUG_KIN 0

limits_t lim = { .min_driver = DRIVER_MINIMO, .max_driver = DRIVER_MAXIMO,
		.max_vel = MAX_VEL_LINEAL, .min_wheel = MIN_WHEEL, .max_wheel =
				MAX_WHEEL, .min_speed = LINEAR_MIN_VELOCITY, .max_speed =
				LINEAR_MAX_VELOCITY };

void cmddriver(cal_variable_t *device, double* cmd) {

	device->driver[0] = 0;
	device->driver[1] = 0;
	if (cmd[0] != 0 && cmd[1] != 0) {
		for (size_t i = 0; i < 2; i++) {
			device->driver[i] = round(
					(lim.max_driver / lim.max_vel) * (fabs(cmd[i])));
			if (cmd[i] < 0) {
				device->driver[i] = -device->driver[i];
			}
		}
	}
#if DEBUG_KIN
	printf("comandos = %f  %f\n", device->driver[0], device->driver[1]);
#endif
}

void driverspeed(cal_variable_t *device, double* driver) {
	for (size_t i = 0; i < 2; i++) {
		{
			if (driver[i] > 0) {
				device->speed[i] = 0.0
						+ ((driver[i] - lim.min_driver)
								* (lim.max_wheel - 0.0)
								/ (lim.max_driver - lim.min_driver));
			} else {
				device->speed[i] = lim.min_wheel
						+ ((driver[i] - (-1) * lim.max_driver)
								* (0.0 - lim.min_wheel)
								/ (lim.max_driver - lim.min_driver));
			}
		}
	}
}

void Motorsvel(cal_variable_t *device, double* driver) {

	driverspeed(device, driver);

	// v=w*r velocidad angular * RADIUS
	device->Vrl[0] = device->speed[0] * RADIUS;
	device->Vrl[1] = device->speed[1] * RADIUS;


	 //W= (Vr + Vl )/l   l = SEPARATION;
	device->W[0]=(device->Vrl[0]-device->Vrl[1])/SEPARATION;
	device->V[0]= (device->Vrl[0]+ device->Vrl[1])/2;


	// *** rad/s a rpm**
	device->Wl[0] = (device->speed[0] * 60) / (2.0 * M_PI);
	device->Wl[1] = (device->speed[1] * 60) / (2.0 * M_PI);

#if DEBUG_KIN
	printf("Wl = %f  Wr= %f\n", device->Wl[0], device->Wl[1]);
	printf("calculo : %f %f\n", device->cal[0], device->cal[1]);
	printf("velocidad  : %f  %f\n", device->Vrl[0], device->Vrl[1]);
	printf("W = %lf  V = %lf\n ",device->W[0],device->V[0]);
#endif

}

void controlvel(cal_variable_t *device, float velocidad_lineal,
		float velocidad_angular)

{
	// vt=vr+vl /2 promedio velocidad total
	// si vr>vl gira en el sentido del menor vl
	// si Wr = -Wl sin avance;
	//W= (Vr - Vl )/l   l = SEPARATION;
	//velocidad_lineal=velocidad_lineal*lim.max_speed;
	double Vrl[2] = {0.0, 0.0};
	Vrl[0] =-(velocidad_lineal - ((velocidad_angular * SEPARATION) / 2));
	Vrl[1] = velocidad_lineal + ((velocidad_angular * SEPARATION) / 2);
#if DEBUG_KIN
	printf("velocidad %f %f\n",device->Vrl[0],device->Vrl[1]);
#endif
	cmddriver(device, &Vrl[0]);
}
