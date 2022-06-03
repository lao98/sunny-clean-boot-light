/*
 *
 *  Created on: feb 18, 2022
 *      Author: leo
 */

#include "include/kinematics.h"



limits_t lim={.min_driver=DRIVER_MINIMO,.max_driver=DRIVER_MAXIMO,.max_vel=MAX_VEL_LINEAL,.min_wheel=MIN_WHEEL, .max_wheel=MAX_WHEEL, .min_speed=LINEAR_MIN_VELOCITY,.max_speed=LINEAR_MAX_VELOCITY};





double cmddriver (cal_variable_t *device, double Vrl )
{
  device->cmd[0]=device->Vrl[0];device->cmd[1]=- device->Vrl[1];
  device->driver[0]=0;device->driver[1]=0;
  if (device->cmd[0] !=0 && device->cmd[1] !=0){
	  for (size_t i = 0; i < 2; i++){
	     device->driver[i]=round((lim.max_driver/lim.max_vel)*( fabs(device->cmd[i])));
	     if (device->cmd[i] < 0)
	     {
	       device->driver[i]= - device->driver[i];
	     }
	   }
  }
  //printf("comandos = %f  %f\n",device->driver[0],device->driver[1]);
  return device->driver[0];//device->driver[1];
}
double drivercmd(cal_variable_t *device )
{
  for (size_t i =0; i<2; i++){
    {
      if (device->driver[i] > 0)
      {
        device->cmd[i] = 0.0 + ((device->driver[0] - lim.min_driver) * (lim.max_wheel - 0.0)
        /(lim.max_driver -lim.min_driver));
        printf("cmd : % f\n",device->cmd[i]);
      }     
      else
      {
        device->cmd[i] = lim.min_wheel + ((device->driver[0] - (-1) * lim.max_driver) * (0.0 -
        lim.min_wheel) /(lim.max_driver- lim.min_driver));
      }
    }
  }
  return device->cmd[0],device->cmd[1];
}
double Motorsvel(cal_variable_t *device)
{

    // w=2*pi/60
    device->cal[0]=device->driver[0]*2.0*M_PI/60.0;
    device->cal[1]=device->driver[1]*2*M_PI/60.0;
    printf("calculo : %f %f\n",device->cal[0],device->cal[1]);
    // v=w*r velocidad angular * radius
    device->Vrl[0]=device->cal[0]*radius;device->Vrl[0]=device->cal[1]*radius;
    printf("velocidad  : %f  %f\n",device->Vrl[0],device->Vrl[1]);
    /*
    W= (Vr + Vl )/l   l = separation;
    device->W=(Vrl[0]-Vrl[1])/separation;
    double V= (Vrl[0]+ Vrl[1])/2;
    double *Vt=&V;
    printf("W = %lf  V = %lf\n ",Wt[0],Vt[0]);
    */
    //**** rad/s a rpm**
    device->Wl[0]= (device->cal[0]*60)/ (2.0*M_PI);device->Wl[1]=(device->cal[1]*60)/(2.0*M_PI);
    printf("Wl = %f  Wr= %f\n",device->Wl[0],device->Wl[1]);
}




double controlvel(cal_variable_t*device, float velocidad_lineal,float velocidad_angular)

{
    // vt=vr+vl /2 promedio velocidad total
	// si vr>vl gira en el sentido del menor vl
    // si Wr = -Wl sin avance;
	 //W= (Vr - Vl )/l   l = separation;
	velocidad_lineal=velocidad_lineal*lim.max_speed;
	device->Vrl[0]=velocidad_lineal - ((velocidad_angular*separation)/2);
	device->Vrl[1]=velocidad_lineal +((velocidad_angular*separation)/2);
    //printf("velocidad %f %f\n",device->Vrl[0],device->Vrl[1]);
    cmddriver(device,device->Vrl[0]);
    return device->W[0];
}
