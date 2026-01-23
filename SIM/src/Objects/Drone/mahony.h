#ifndef MAHONY_H
#define MAHONY_H


#include <stdio.h>
#include <math.h>

#include "MyVector.h"
#include "quaternion.h"

#define GRAVITY_NED 9.80665


class Mahony {

    private:


    public:

    float ki, kp;
    float imu_hz, imu_dt;
    Tools::Vector3 acc, gyr, mag;
    Tools::Vector3 gyro_bias;
    Tools::Quaternion est_quat;


    Mahony(); 
    ~Mahony();

    Tools::Vector3 quat2euler (Tools::Quaternion q);

    Tools::Quaternion perform_mahony_fusion_6DOF(Tools::Vector3 accel, Tools::Vector3 gyro, Tools::Quaternion est_quat, float dt);
    
    

};



#endif

