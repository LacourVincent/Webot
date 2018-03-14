#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   A simple example for maintaining a straight line with the compass.
 *
 * @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
 * @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
 * @date    2017-11
 */

#include <iostream>
#include <cmath>
#include <webots/DifferentialWheels.hpp>
#include <webots/Compass.hpp>

using namespace std;
using namespace webots;

#define MAX_SPEED       100
#define DESIRED_ANGLE   45

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

        /**
         * @brief Function for converting bearing vector from compass to angle (in degrees).
         * @param in_vector Input vector with the values to convert.
         * @return The value in degrees.
         */
        double convert_bearing_to_degrees(const double* in_vector);

    private:
        int _time_step;
        
        // velocities
        double _left_speed, _right_speed;

        // sensors
        Compass * _my_compass;
};

#endif

