#ifndef MOTORS_H
#define MOTORS_H

#include <iostream>
#include <math.h>

#include "InputSystem.h"

#include "MyVector.h"
#include "quaternion.h"

class Motors {
    private:

    float alpha = 0.45;

    // motors parameters
    float motorThrust = 8.0;
    float ThrustToTorque = 4.0;

    void Q4Mixer (float pitch, float roll, float yaw, float throttle);

    float motors[4] = {0,0,0,0};
    float lastMotorsState[4] = {0,0,0,0};

    public:

    void UpdateMotors(Tools::GM_Inputs input);
    
    float vertical_thrust;
    Tools::Vector3 momentums;

    Motors(); // default constructor

    //destructor
    ~Motors();

};



#endif