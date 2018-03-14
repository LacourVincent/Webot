/**
* @file    MyRobot.cpp
* @brief   A simple example using the odometry with encoders.
*
* @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
* @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
* @date    2017-11
*/

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels() {

  _time_step = 64;

  _left_speed = 0;
  _right_speed = 0;

  // get and enable the encoders
  enableEncoders(_time_step);
  setEncoders(0,0);

  // get and enable the compass device
  _my_compass = getCompass("compass");
  _my_compass-> enable(_time_step);

  // starting position
  position = new Odometry(START_X,START_Z,START_TETA);

}

MyRobot::~MyRobot(){
  // disable devices
  _my_compass->disable();
  disableEncoders();
}

//////////////////////////////////////////////

void MyRobot::run(){

    // List of points of the trajectory and the final orientation.
    const double targets[TARGET_POINTS_SIZE][3] = {
    { 0, 8, 0}};

   objective = new Goal();

  int current_target_index = 0;
  objective->set_goal(position,targets[current_target_index][0], targets[current_target_index][1], targets[current_target_index][2]); // First Target

  while (step(_time_step) != -1) {

    // Read the sensors
    const double *compass_val = _my_compass->getValues();
    double teta = modulus_double(convert_bearing_to_rad(compass_val),2.0*M_PI); // convert compass bearing vector to angle, in radian
    teta = convert_angle_to_coordinate_systeme(teta); //Convert angle to the coordinate system of the world
    int right_count = getRightEncoder(); //read value of right encoder
    int left_count = getLeftEncoder(); //read value of left encoder


    // Update de position of the robot
    position->update_position(right_count,left_count,teta);

    cout << "Encoders droit : " << right_count << endl;
    cout << "Encoders gauche : " << left_count << endl;
    cout << "X = " << position->x << endl;
    cout << "Z = " << position->z << endl;

    // Current target
    if (objective->at_goal == false) {
      objective->goto_step(position);
      setSpeed(objective->left_speed, objective->right_speed);
    }

    // Next target
    else {
      if (current_target_index < TARGET_POINTS_SIZE -1){
          current_target_index++;
          cout << "current_target_index : " << current_target_index << endl;
          objective->set_goal(position,targets[current_target_index][0], targets[current_target_index][1], targets[current_target_index][2]);
      }
      else{
        cout << "End of program : final target reached" << endl;
        break;
      }
    }
  }
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_rad(const double* in_vector){
  double rad = atan2(in_vector[0],in_vector[2]) + (135*M_PI/180);
  return rad;
}

double MyRobot::modulus_double(double a, double m) {
  int div_i = (int) (a/m);
  double div_d = (double) div_i;
  double r = a - div_d * m;
  if (r<0.0)
    r += m;
  return r;
}

double MyRobot::convert_angle_to_coordinate_systeme (double angle){
  return -1*(angle- M_PI);
}
