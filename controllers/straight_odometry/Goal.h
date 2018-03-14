#ifndef GOAL_H_
#define GOAL_H_

/**
* @file    Goal.h
* @brief   Control the value of the motors to reach a target.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include <iostream>
#include <cmath>
#include "Odometry.h"

using namespace std;

// Distance tolerance (meter)
#define DISTANCE_TOLERANCE 0.5

// Angle tolerance (degree)
#define ANGLE_TOLERANCE 2

#define MAX_SPEED 100

class Goal {
public:

  /**
  * @brief Empty constructor of the class.
  */
  Goal();

  /**
  * @brief Destructor of the class.
  */
  ~Goal();

  /**
  * @brief Define the objective.
  * @param current_position : The current position of the robot.
  * @param x : The value x of the objective.
  * @param z : The value z of the objective.
  * @param teta : The value teta of the objective.
  * @return
  */
  void set_goal(Odometry* current_position,double x, double z,double teta);

  /**
  * @brief Define the value of the motors.
  * @param position : The current position of the robot.
  * @return
  */
  void goto_step(Odometry* position);

  double left_speed;
  double right_speed;
  bool at_goal;

private:

  /**
  * @brief Function for converting an angle in degree.
  * @param angle : The value of angle in radians.
  * @return The value of angle in degree.
  */
  double convert_angle_to_deg(const double angle);

  /**
  * @brief Function for calculating the value of angle between the initial position and the objective thanks to vectors.
  * @param teta : Orientation of the robot.
  * @param x1 : Value x of the position of robot.
  * @param z1 : Value z of the position of robot.
  * @param x2 : Value x of the objective.
  * @param z2 : Value z of the objective.
  * @return The value of angle between the initial position and the objective.
  */
  double calculation_teta_start(double teta,double x1 , double x2, double z1 , double z2);

  /**
  * @brief Function for calculating the norm of a vector(u,v).
  * @param u_1 : Component u of the vector.
  * @param v_1 : Component v of the vector.
  * @return The norm of the vector.
  */
  double norm_vector(double u_1, double v_1);

  /**
  * @brief Function for calculating the product of two vectors.
  * @param u_1 : Component u of the first vector.
  * @param v_1 : Component v of the first vector.
  * @param u_2 : Component u of the second vector.
  * @param v_2 : Component v of the second vector.
  * @return The product of two vectors.
  */
  double product_vector(double u_1 , double v_1 , double u_2, double v_2);

  /**
  * @brief Function for calculating the determinant of a couple of vectors.
  * @param u_1 : Component u of the first vector.
  * @param v_1 : Component v of the first vector.
  * @param u_2 : Component u of the second vector.
  * @param v_2 : Component v of the second vector.
  * @return The determinant of a couple of vectors.
  */
  double det_vector(double u_1, double v_1 , double u_2 , double v_2);

  double goal_x;
  double goal_z;
  double goal_teta;

  double start_teta;
};

#endif
