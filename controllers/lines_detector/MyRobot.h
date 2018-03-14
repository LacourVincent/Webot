#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
* @file    MyRobot.h
* @brief   Controller using the spherical camera of the robot in order to detect yellow lines.
*
* @warning Please set the field "FieldOfView" of the sperical camara at 2.2.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include <iostream>
#include <webots/DifferentialWheels.hpp>
#include <webots/Camera.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED  100

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
  int _time_step;

  // velocities
  double _left_speed, _right_speed;

  // Camera
  Camera *_spherical_camera;
};

#endif
