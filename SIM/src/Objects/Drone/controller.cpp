#include "controller.h"




PIDController::PIDController(){


}


PIDController::~PIDController(){


}



Tools::Vector3 PIDController::RatePID(Tools::GM_Inputs GMinput, Tools::Vector3 gyro, float dT){

        Tools::Vector3 input = {-GMinput.pitch, GMinput.yaw, GMinput.roll};

        // aplly max angular speeds
        input = input * MaxAngularSpeed;

        //calculate error
        Tools::Vector3 error = input - gyro;

        // calculate Integral term
        RateI = RateI + ((error * Ki_Rate) * dT);

        // D term with alphaBeta Lowpass filter
        Tools::Vector3 errorDerivative = (error - RateLastError) / dT;
        RateLastError = error;

        // lowpass
        //RateD = (errorDerivative * DtermAlpha) + (RateD * (1 - DtermAlpha));
        RateD = RateD * Kd_Rate;

        
        // anti wind-up
        if (RateI.x > MaxIntegral) RateI.x = MaxIntegral;
        if (RateI.x < -MaxIntegral) RateI.x = -MaxIntegral;
        if (RateI.y > MaxIntegral) RateI.y = MaxIntegral;
        if (RateI.y < -MaxIntegral) RateI.y = -MaxIntegral;
        if (RateI.z > MaxIntegral) RateI.z = MaxIntegral;
        if (RateI.z < -MaxIntegral) RateI.z = -MaxIntegral;


        // zero integral when throttle is low
        if(GMinput.throttle < 0.2){
            RateI = {0,0,0};
        }

        Tools::Vector3 omega = (error * Kp_Rate) + RateI + RateD;

        return omega;

}













//////////////////////////////////////
// QUATERNIONS
//////////////////////////////////////


QuaternionController::QuaternionController(){


}


QuaternionController::~QuaternionController(){


}


Tools::Vector3 QuaternionController::QControllerUpdate(Tools::Quaternion q_ref, Tools::Quaternion rotation, Tools::Vector3 omega, float dT) {

    Tools::Vector3 tr_omega(omega.x, omega.y, omega.z);

    //std::cout << "Rotation: " << rotation << std::endl;

    // 1. Compute error in BODY frame (conj(q_meas) * q_ref)
    Tools::Quaternion q_err = rotation.conjugate() * q_ref;

    q_err = q_err.normalize();

    Tools::AxisAngle err = q_err.ToAxisAngle();  // yields axis=(ex,ey,ez), angle=theta
    float theta = err.a;

    // If axis is not normalized inside .ToAxisAngle(), then normalize it here.
    Tools::Vector3 axis(-err.x, -err.y, err.z);
    
    
    // PD torque in body frame:
    Tools::Vector3 tau_p = axis * (-Kp * theta );
    Tools::Vector3 tau_d = omega * -Kd; 
    Tools::Vector3 tau   = tau_p + tau_d;

    std::cout << "      TauP: " << tau_p << std::endl;
    std::cout << "      TauD: " << tau_d << std::endl;

    return tau;
}