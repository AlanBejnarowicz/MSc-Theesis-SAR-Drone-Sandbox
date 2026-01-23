#ifndef CONTROLLER_H
#define CONTROLLER_H


#include <iostream>
#include <math.h>
#include <vector>

#include "MyVector.h"
#include "quaternion.h"
#include "InputSystem.h"


class PIDController{

    private:

        Tools::Vector3 MaxAngularSpeed = {7,7,7};

        float MaxIntegral = 0.2;

        // Rate controller
        Tools::Vector3 RateI;
        Tools::Vector3 RateD = {0,0,0};
        Tools::Vector3 RateLastError;


    public:

        PIDController();
        ~PIDController();

        float DtermAlpha = 0.7;

        //Rate PID values
        Tools::Vector3 Kp_Rate = {0.3,0.3,0.3};
        Tools::Vector3 Ki_Rate = {0.01,0.01,0.01};
        Tools::Vector3 Kd_Rate = {0.01, 0.01, 0.01};


        //methods

        Tools::Vector3 RatePID(Tools::GM_Inputs GMinput, Tools::Vector3 gyro, float dT);


};







class QuaternionController {
    private:


    public:

        double Kp = 0.4;                   // proportional gain
        double Kd = 0.2;                   // derivative (rate) gain

        QuaternionController();
        ~QuaternionController();

        //methods
        Tools::Vector3 QControllerUpdate(Tools::Quaternion q_ref, Tools::Quaternion rotation, Tools::Vector3 omega, float dT);


};


#endif