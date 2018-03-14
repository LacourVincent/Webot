#ifndef ODOMETRY_H_
#define ODOMETRY_H_

/**
* @file    Odometry.h
* @brief   Calculate the current position of the robot with odometry.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include <iostream>
#include <cmath>

using namespace std;

// Number of increments per tour for a wheel. Depend on encoderResolution.
#define INCREMENTS_PER_TOUR  628
// Wheel radius of the robot.
#define WHEEL_RADIUS 0.0825

class Odometry{
public:
  /**
  * @brief Constructor of the class.
  */
  Odometry(double start_x, double start_z, double start_teta);

  /**
  * @brief Destructor of the class.
  */
  ~Odometry();

  /**
  * @brief Update the position of the robot.
  * @param encoder_right : The value of the right encoder.
  * @param encoder_left : The value of the left encoder.
  * @param angle : The angle of the robot.
  * @return
  */
  void update_position(int encoder_right ,int encoder_left, double angle);

  // Position of the robot.
  double x;
  double z;
  double teta;

private:

  /**
  * @brief Calculate the distance traveled by a wheel.
  * @param count : The number of ticks of an encoder.
  * @return The distance traveled by a wheel.
  */
  double estimate_distance_wheel(const int count);

  int right_count;
  int left_count;

  int old_count_right;
  int old_count_left;
};

#endif
