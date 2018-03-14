#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
* @file    MyRobot.h
* @brief   A simple example using the odometry with encoders.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>
#include <webots/Compass.hpp>
#include "Goal.h"
#include "Odometry.h"

using namespace std;
using namespace webots;

// Starting position
#define START_X 0
#define START_Z -9
#define START_TETA 0

// Number of points in the trajectory
#define TARGET_POINTS_SIZE 1

class MyRobot : public DifferentialWheels {
public:
  /**
  * @brief Empty constructor of the class.
  */
  MyRobot();

  /**
  * @brief Destructor of the class.
  */
  ~MyRobot();

  /**
  * @brief Function with the logic of the controller.
  * @param
  * @return
  */
  void run();

private:

  /**
  * @brief Function for converting bearing vector from compass to angle (in radians).
  * @param in_vector Input vector with the values to convert.
  * @return The value in radians.
  */
  double convert_bearing_to_rad(const double* in_vector);

  /**
  * @brief Function for calculing the modulus of a number.
  * @param a : Value of number.
  * @param m : Value of modulus.
  * @return The value of modulus.
  */
  double modulus_double(double a, double m);

  /**
  * @brief Function for converting the angle to the coordinate systeme of the world.
  * @param Value of angle (degree).
  * @return Value of angle in the coordinate system of the world.
  */
  double convert_angle_to_coordinate_systeme (double angle);

  int _time_step;

  // velocities
  double _left_speed, _right_speed;

  // sensors
  Compass * _my_compass;

  // position
  Odometry* position;
  // objective
  Goal* objective;

};

#endif
