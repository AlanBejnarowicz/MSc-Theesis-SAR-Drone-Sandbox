#ifndef vIMU_H
#define vIMU_H


#include <iostream>
#include <math.h>
#include <vector>

#include "MyVector.h"
#include "quaternion.h"

class vIMU {

    private:

    Tools::Vector3 GRAV = {0,-9.81,0};
    Tools::Vector3 last_vel = {0,0,0};


    public:
    Tools::Quaternion last_rotation;
    

    vIMU(); 
    vIMU(Tools::Quaternion initial_rotation); 
    ~vIMU();


    // methods
    Tools::Vector3 SimulateGyro(Tools::Quaternion rotation, float dT);
    Tools::Vector3 SimulateAccelerometer(Tools::Quaternion rotation, Tools::Vector3 vel, float dT);

};


#endif