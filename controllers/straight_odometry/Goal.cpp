/**
* @file    Goal.cpp
* @brief   Control the value of the motors to reach a target.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include "Goal.h"

Goal::Goal()
{
  goal_x = 0;
  goal_z = 0;
  at_goal = false;
  start_teta = 0;

  left_speed = 0;
  right_speed = 0;
}


Goal::~Goal()
{
  //Empty
}

//////////////////////////////////////////////

void Goal::set_goal(Odometry* current_position, double x, double z,double teta){
  goal_x = x;
  goal_z = z;
  goal_teta = teta;
  at_goal = false;
  start_teta = calculation_teta_start(current_position->teta, current_position->x , goal_x , current_position->z , goal_z);
}

void Goal::goto_step(Odometry* current_position){

  double dx = goal_x - current_position->x ;
  double dz = goal_z -current_position->z ;
  double distance_goal = sqrt(dx*dx + dz*dz);
  double diff_angle = start_teta - convert_angle_to_deg(current_position->teta);

  if ( distance_goal < DISTANCE_TOLERANCE) {
    double diff_angle_goal = goal_teta - convert_angle_to_deg(current_position->teta);
    
    if (diff_angle_goal > ANGLE_TOLERANCE ){
      left_speed = -MAX_SPEED/10;
      right_speed = MAX_SPEED/10;
    }
    else if (diff_angle_goal < -ANGLE_TOLERANCE ){
      left_speed = MAX_SPEED/10;
      right_speed = -MAX_SPEED /10 ;
    }
    else {
      at_goal = true;
      cout << "sub target reached : x = " << goal_x << " z = "  << goal_z << " teta = " << goal_teta  << "°" << endl;
      left_speed = 0;
      right_speed = 0;
    }
  }

  //Forward left
  else if ((diff_angle > ANGLE_TOLERANCE)&&(abs(diff_angle) < 15)) {
    left_speed = MAX_SPEED - 15;
    right_speed = MAX_SPEED;
  }

  //Forward right
  else if ((diff_angle < -ANGLE_TOLERANCE)&&(abs(diff_angle) < 15)) {
    left_speed = MAX_SPEED;
    right_speed = MAX_SPEED -15;
  }

  //Turn left
  else if ((diff_angle > ANGLE_TOLERANCE)&&(abs(diff_angle) > 15)){
    left_speed = - MAX_SPEED/10;
    right_speed = MAX_SPEED/10;
  }

  //Turn right
  else if ((diff_angle < -ANGLE_TOLERANCE)&&(abs(diff_angle) > 15) ){
    left_speed = MAX_SPEED/10;
    right_speed = -MAX_SPEED /10 ;
  }

  // Forward
  else{
    left_speed = MAX_SPEED ;
    right_speed = MAX_SPEED;
  }

}

//////////////////////////////////////////////

double Goal::convert_angle_to_deg(const double rad){
  return  (rad*180)/M_PI;
}


double Goal::calculation_teta_start(double teta ,double x1 , double x2, double z1 , double z2){
  double u_x = sin(teta);
  double u_z = cos(teta);
  double v_x = x2 - x1;
  double v_z = z2 - z1 ;
  double angle = acos( product_vector(u_x,v_x,u_z,v_z)/(norm_vector(u_x,u_z)*norm_vector(v_x,v_z)));
  if (det_vector(u_z,u_x,v_z,v_x) < 0){
    angle = - angle;
  }
  return convert_angle_to_deg(teta + angle); // unit = deg
}


double Goal::norm_vector(double u_1, double v_1){
  return sqrt(u_1*u_1 + v_1*v_1);
}


double Goal::product_vector(double u_1 , double v_1 , double u_2, double v_2){
  return (u_1*v_1 + u_2*v_2);
}


double Goal::det_vector(double u_1, double v_1 , double u_2 , double v_2){
  return (u_1*v_2 - v_1*u_2);
}
