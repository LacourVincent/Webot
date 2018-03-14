#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   Read the values of all distance sensors.
 *
 * @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
 * @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
 * @date    2017-11
 */

#include <iostream>
#include <cmath> 
#include <webots/DistanceSensor.hpp>
#include <webots/DifferentialWheels.hpp>

using namespace std;
using namespace webots;

#define NUM_DISTANCE_SENSOR 16
#define MAX_SPEED       0

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
         * @brief Convert a value of distance sensor in centimeter.
         * @param valor Value of distance sensor.
         * @return Value in centimeter.
         */
    double convert_value_to_centimeters(const double valor);

private:
    int _time_step;

    // velocities
    double _left_speed, _right_speed;

    // distance sensors
    DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];

};

#endif

