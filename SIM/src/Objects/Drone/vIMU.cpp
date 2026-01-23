#include "vIMU.h"

vIMU::vIMU(){

}


vIMU::vIMU(Tools::Quaternion initial_rotation){

    // take initial value
    last_rotation = initial_rotation;

}

vIMU::~vIMU(){

}


Tools::Vector3 vIMU::SimulateGyro(Tools::Quaternion rotation, float dT){

    // calculating difference between Quaternions
    Tools::Quaternion qDelta = rotation * last_rotation.inverse();
    //qDelta = qDelta.normalize();


    last_rotation = rotation.normalize();  // Ensure unit quaternion


    // extract w and v
    float w = qDelta.w;
    Tools::Vector3 v(qDelta.x, qDelta.y, qDelta.z);


    float epsilon = 1e-8f;
    float cosHalfAngle = std::max(-1.0f, std::min(w, 1.0f));

    float angle = 2.0f * (float)acos(cosHalfAngle);
    // 4) If angle is very small, use small-angle approximation for axis
    float sinHalfAngle = (float)sqrt(1.0f - cosHalfAngle*cosHalfAngle);


    Tools::Vector3 axis;
    if (fabs(sinHalfAngle) > epsilon) {
        axis = v / sinHalfAngle;
    } else {
        axis = {1.0f, 0, 0}; // arbitrary
    }

    // 5) Angular velocity = (angle / dt) * axis
    //    in the same coordinate frame that qDelta represents
    float rate = angle / dT;
    Tools::Vector3 omega = axis * rate;

    // Convert to drone's local frame
    Tools::Vector3 localOmega = rotation * omega * rotation.inverse();

    localOmega.z = -localOmega.z;
    
    // convert to (roll pitch yaw)  from (pitch, yaw, roll)
    //Tools::Vector3 RPYlocalOmega = {localOmega.x, localOmega.y, -localOmega.z};


    return localOmega;
}


Tools::Vector3 vIMU::SimulateAccelerometer(Tools::Quaternion rotation, Tools::Vector3 vel, float dT){

    Tools::Vector3 acc_global = (vel - last_vel) / dT;
    last_vel = vel;

    //add gravity
    acc_global = acc_global - GRAV;

    //convert acc_global to local frame
    Tools::Vector3 acc_local = acc_global * rotation.inverse();

    return acc_local;
}
