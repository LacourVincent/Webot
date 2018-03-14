/**
* @file    Odometry.cpp
* @brief   Calculate the current position of the robot with odometry.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include "Odometry.h"

//////////////////////////////////////////////

Odometry::Odometry(double start_x, double start_z, double start_teta){

  x = start_x;
  z = start_z;
  teta = start_teta;

  right_count = 0;
  left_count = 0;
  old_count_right = 0;
  old_count_left = 0;
}

Odometry::~Odometry(){
    //Empty
}

//////////////////////////////////////////////

void Odometry::update_position(int encoder_right ,int encoder_left, double angle){

    // Manage value of encoders.
    right_count = encoder_right - old_count_right ;
    left_count = encoder_left - old_count_left ;
    old_count_right = encoder_right;
    old_count_left = encoder_left;

    // Estimation of the distance traveled by the right wheel.
    double distance_right = estimate_distance_wheel(right_count);
    // Estimation of the distance traveled by the left wheel.
    double distance_left = estimate_distance_wheel(left_count);
    // Total distance
    double total_distance = (distance_right + distance_left)/2;

    // New position of the robot.
    teta = angle;
    x = x + total_distance*sinf(teta);
    z = z + total_distance*cosf(teta);
  }

//////////////////////////////////////////////

double Odometry::estimate_distance_wheel(const int count){
  double distance = (count*2*WHEEL_RADIUS*M_PI) / INCREMENTS_PER_TOUR;
  return distance;
}

//////////////////////////////////////////////
