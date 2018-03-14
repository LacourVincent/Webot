/**
 * @file    MyRobot.cpp
 * @brief   Read the values of all distance sensors.
 *
 * @author  LACOUR Vincent <100378196@alumnos.uc3m.es>
 * @author  JOFRE Rodrigo <100378110@alumnos.uc3m.es>
 * @date    2017-11
 */

#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : DifferentialWheels()
{
    // init default values
    _time_step = 64;
    _left_speed = 0;
    _right_speed = 0;
    
    // get distance sensor array and enable each one
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        ostringstream os;
        os << "ds"<<i;
        std::string res = os.str();
        _distance_sensor[i] = getDistanceSensor(res);
        _distance_sensor[i]->enable(_time_step);
    }
}


//////////////////////////////////////////////

MyRobot::~MyRobot()
{  
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
}

//////////////////////////////////////////////

void MyRobot::run()
{
    double ir_val[NUM_DISTANCE_SENSOR];
    while (step(_time_step) != -1) {

        // print sensor values to console
        for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
            ir_val[i] = _distance_sensor[i]->getValue();
            cout <<" DS"<< i <<": "<< convert_value_to_centimeters((ir_val[i]));
        }
        cout << endl;
    }
}

//////////////////////////////////////////////////////////

double MyRobot::convert_value_to_centimeters(const double valor){
    double result = 0;
    if ((valor < 1000) && (valor > 50)){
        result = (0.0000001253*valor*valor -0.0003421*valor + 0.3178)*100 ;
    }
    return result;
}
