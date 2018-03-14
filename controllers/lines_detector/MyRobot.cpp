/**
* @file    MyRobot.cpp
* @brief   Controller using the spherical camera of the robot in order to detect yellow lines.
*
* @warning Please set the field "FieldOfView" of the sperical camara at 2.2.
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

  // get spherical camera and enable it.
  _spherical_camera = getCamera("camera_s");
  _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
  // disable camera devices
  _spherical_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
  // initialize the local variables.
  int sum = 0;
  unsigned char green = 0, red = 0, blue = 0;
  double percentage_yellow = 0.0;

  // get size of images for spherical camera
  int image_width_s = _spherical_camera->getWidth();
  int image_height_s = _spherical_camera->getHeight();
  cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;

  while (step(_time_step) != -1) {
    sum = 0;

    // get current image from forward camera
    const unsigned char *image_s = _spherical_camera->getImage();

    // count number of pixels that are yellow in the picture
    for (int x = 0; x < image_width_s; x++) {
      for (int y = 0; y < image_height_s; y++) {
        green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
        red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
        blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);

        if ((green > 163) && (red > 163) && (blue < 5)) {
          sum = sum + 1;
        }
      }
    }

    percentage_yellow = (sum / (float) (image_width_s * image_height_s)) * 100;
    if (percentage_yellow > 1.0){
      cout << "Yellow line detected : rate = "<< percentage_yellow << endl;
    }
    else{
      cout << "Yellow line not detected." << endl;
    }

    // go straight away
    _left_speed = MAX_SPEED / 10;
    _right_speed = MAX_SPEED / 10;

    // set the motor speeds
    setSpeed(_left_speed, _right_speed);
  }
}

//////////////////////////////////////////////
