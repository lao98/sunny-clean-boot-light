#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdlib.h>
#include <fenv.h>
#include "stm32f7xx_hal.h"


#define radius 0.035845
#define separation 0.2065

/*
#define separation 0.2065
#define separation_multiplier 0.980
#define left_radius_multiplier 1.066
#define right_radius_multiplier 1.066
#define error_wheel_right 0.00005
#define error_wheel_left 0.00005
#define DEFAULT_LINEAR_MIN_VELOCITY  -0.621
#define DEFAULT_LINEAR_MAX_VELOCITY  0.621
*/
#define Rpm_max 300
//
#define MIN_WHEEL -31.41  // 300 rpm max-> angular speed max [rad/s]
#define MAX_WHEEL 31.41   // -300 rpm min->angular speed min[rad/s]
#define DRIVER_MINIMO 1000  // comando driver max
#define DRIVER_MAXIMO -1000 // comando  driver min
#define LINEAR_MIN_VELOCITY  -0.621
#define LINEAR_MAX_VELOCITY  0.621
#define MAX_VEL_LINEAL 1.1261

typedef struct
{   
  double min_driver;   // Min value 
  double max_driver;   // Max value
  double min_wheel; 
  double max_wheel;
  double min_speed;
  double max_speed;
  double max_vel;
   
} limits_t;
/*
typedef struct{
	float r;
  float l;
  float W;
  float V;
}value_t;
*/
typedef struct{
	double driver[2];
	double cmd[2];
	double cal[2];
	double Vrl[2];
	double Wl[2];
	double vel_w[1];
	float W[1];
}cal_variable_t;
/**
   * \brief Maps the driver values to command values
   * \param cmd Command values to map
 */
double cmddriver(cal_variable_t*device, double vel_w);
/**
   * \brief Maps the command values to driver values
   * \param cmd Command values to map
*/
double drivercmd(cal_variable_t*device);
/**
   * \converted driver to W y vl
*/
double MotorsVel(cal_variable_t*device);
/**
   * \converted control to vel
*/
double controlvel(cal_variable_t*device ,float velocidad_lineal,float velocidad_angular);



