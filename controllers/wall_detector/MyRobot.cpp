/**
 * @file    MyRobot.cpp
 * @brief   Controller using the front camera of the robot in order to detect walls.
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

    // get camera and enable it.
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // disable devices
    _forward_camera->disable();
}

//////////////////////////////////////////////

void MyRobot::run()
{
    // initialize the local variables.
    int yellow = 0, wall = 0, floor = 0, total = 0;
    unsigned char green = 0, red = 0, blue = 0;
    double percentage_yellow = 0.0, percentage_wall = 0.0, percentage_floor = 0.0;

    // get size of images for forward camera, limit the y axis to 137 to use the useful camera vision.
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = 137;
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    while (step(_time_step) != -1) {
        yellow = 0, wall = 0, floor = 0, total=0;

        // get current image from forward camera
        const unsigned char *image_f = _forward_camera->getImage();

        // count number of pixels that are white
        // (here assumed to have pixel value > 245 out of 255 for all color components)
        for (int x = 0; x < image_width_f; x++) {
            for (int y = 0; y < image_height_f; y++) {
                green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
                red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
                blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

                if ((green > 163) && (red > 163) && (blue < 5)) {
                    yellow = yellow + 2;
                }
                else if ((green <= 44) && (red <= 44) && (blue <= 44)) {
                    floor = floor + 1;
                }
                else if ((green == blue) && (red == blue)) {
                    wall = wall + 1;
                }
            }
        }

        // get the % of each element to analize by the data received.
        percentage_yellow = (yellow / (float) (image_width_f * image_height_f)) * 100;
        percentage_wall = (wall / (float) (image_width_f * image_height_f)) * 100;
        percentage_floor = (floor / (float) (image_width_f * image_height_f)) * 100;
        total = percentage_yellow+percentage_wall+percentage_floor;

        cout << "Percentage in forward camera image of:: yellow: " << percentage_yellow << "| Wall : " << percentage_wall << "| Floor : " << percentage_floor <<"| Total% : " << total << endl;

        // turn around slowly
        _left_speed = MAX_SPEED/20;
        _right_speed = -MAX_SPEED/20;

        // set the motor speeds
        setSpeed(_left_speed, _right_speed);
    }
}

//////////////////////////////////////////////
