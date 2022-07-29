/*
 *
 *  Created on: feb 18, 2022
 *      Author: leo
 */

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


#define RADIUS 0.035845
#define SEPARATION 0.2065

/*
#define SEPARATION 0.2065
#define SEPARATION_multiplier 0.980
#define left_RADIUS_multiplier 1.066
#define right_RADIUS_multiplier 1.066
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
#define LINEAR_MIN_VELOCITY  -0.5//0.621
#define LINEAR_MAX_VELOCITY  0.5//0.621

#define LINEAR_MEDIO_MIN_VELOCITY  -0.4
#define LINEAR_MEDIO_MAX_VELOCITY  0.4

#define LINEAR_MINIMO_MIN_VELOCITY  -0.3
#define LINEAR_MINIMO_MAX_VELOCITY  0.3

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
	double driver[2]; //cmds in values to driver
	double speed[2]; //speed read from the drive in rad/s
	double Vrl[2];
	double Wl[2]; //speed read from the driver in rpm
	double V[1]; //linear velocity calculated from the speeds read from driver
	float W[1];//angular velocity calculated from the speeds read from driver
}cal_variable_t;

/**
   * \brief Maps the command values to driver values
   * \param cmd Command values to map
 */
void cmddriver(cal_variable_t *device, double* cmd);

/**
   * \brief Maps the driver values to speed values
   * \param cmd Command values to map
*/
void driverspeed(cal_variable_t*device, double* driver);
/**
   * \converted driver to W y vl
*/
void MotorsVel(cal_variable_t*device, double* driver);
/**
   * \converted control to vel
*/
void controlvel(cal_variable_t*device ,float velocidad_lineal,float velocidad_angular);
