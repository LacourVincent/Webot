/**
 * @file    MyRobot.cpp
 * @brief   Avoid obstacles with the distance sensors.
 *
 * @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
 * @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
 * @date    2017-11
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
  // init default values
  _time_step = 64;

  _left_speed = 0;
  _right_speed = 0;

  _mode = STOP;

  // get distance sensor array and enable each one
  for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
    stringstream ds;
    ds << "ds" << i ;
    _distance_sensor[i] = getDistanceSensor(ds.str());
    _distance_sensor[i]->enable(_time_step);
  }
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
  // disable devices
  for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
    _distance_sensor[i]->disable();
  }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // get the sensor values and use local boolean variables refered to the principal sensors of each detection zone.
  double *sensorvalue;
  sensorvalue = new double[NUM_DISTANCE_SENSOR];
  bool front_right , front_left , side_right , side_left;

  while (step(_time_step) != -1) {

    // read the sensors
    for (int i = 0 ; i < NUM_DISTANCE_SENSOR ; i++){
      sensorvalue[i] = _distance_sensor[i]->getValue();
    }

    // control logic of the robot
    front_right = (sensorvalue[14] > DISTANCE_LIMIT);
    front_left =  (sensorvalue[1] > DISTANCE_LIMIT);
    side_right = (sensorvalue[11] > DISTANCE_LIMIT);
    side_left = (sensorvalue[4] > DISTANCE_LIMIT);

    // routines associated to "_mode" function.
    if ((!front_right)&&(front_left)&&(!side_right)&&(!side_left)){
      _mode = ROTATION_RIGHT;
    }
    else if((front_right)&&(front_left)&&(side_right)&&(side_left)){
      _mode = ROTATION_LEFT;
    }
    else if((front_right)&&(!front_left)&&(!side_right)&&(!side_left)){
      _mode = ROTATION_LEFT;
    }
    else if((front_right)&&(front_left)&&(!side_right)&&(!side_left)){
      _mode = BACKWARD; // * Change */
    }
    else if((front_right)&&(front_left)&&(side_right)&&(!side_left)){
      _mode = ROTATION_LEFT;
    }
    else if((front_right)&&(front_left)&&(!side_right)&&(side_left)){
      _mode = ROTATION_RIGHT;
    }
    else if((front_right)&&(!front_left)&&(side_right)&&(!side_left)){
      _mode = ROTATION_LEFT;
    }
    else if((!front_right)&&(front_left)&&(!side_right)&&(side_left)){
      _mode = ROTATION_RIGHT;
    }
    else if((!front_right)&&(!front_left)&&(side_right)&&(!side_left)){
      _mode = FORWARD_RIGHT;
    }
    else if((!front_right)&&(!front_left)&&(!side_right)&&(side_left)){
      _mode = FORWARD_LEFT;
    }
    else{
      _mode = FORWARD;
    }

    // send actuators commands according to the mode
    switch (_mode){
      case STOP:
      _left_speed = 0;
      _right_speed = 0;
      break;
      case FORWARD:
      _left_speed = MAX_SPEED/2;
      _right_speed = MAX_SPEED/2;
      break;
      case BACKWARD:
      _left_speed = -15  ;
      _right_speed = 2.5;
      break;
      case ROTATION_RIGHT:
      _left_speed =  MAX_SPEED/8;
      _right_speed = -MAX_SPEED/8;
      break;
      case ROTATION_LEFT:
      _left_speed = -MAX_SPEED/8;
      _right_speed = MAX_SPEED/8;
      break;
      case FORWARD_LEFT:
      _left_speed = 55;
      _right_speed = 75;
      break;
      case FORWARD_RIGHT:
      _left_speed = 75;
      _right_speed = 55;
      break;
      default:
      break;
    }

    // set the motor speeds
    setSpeed(_left_speed, _right_speed);
  }
}

//////////////////////////////////////////////
