/**
 * @file    straight_odometry.cpp
 * @brief   A simple example using the odometry with encoders.
 *
 * @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
 * @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
 * @date    2017-11
 */

#include "MyRobot.h"

/**
 * @brief Main program.
 */

int main(int argc, char **argv)
{
    MyRobot* my_robot = new MyRobot();
    my_robot->run();
    delete my_robot;
    return 0;
}
