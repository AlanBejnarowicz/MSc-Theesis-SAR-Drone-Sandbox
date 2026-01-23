
#include "quadcopter.h"



Quadcopter::Quadcopter()// Initialize gamepad to 0 or the desired index
{
    Start();

}

Quadcopter::Quadcopter(Tools::GM_Inputs* gmpd)// Initialize gamepad to 0 or the desired index
{
    gm_input = gmpd;
    Start();
}

Quadcopter::~Quadcopter()
{

}



void Quadcopter::Start(void){

        // ensure zero start conditions
        position = {0,0,0};
        last_possition = position;
        rotation = {0, 0, 0, 1};
        rotation = rotation.normalize();
        velocity = {0,0,0};
        
        //precompute diagonal inverse inertia
        //!!!! DIAG ELEMENTS CANNOT BE ZERO !!!!
    
        invInertiaDiag.x = 1 / InertiaDiag.x;
        invInertiaDiag.y = 1 / InertiaDiag.y;
        invInertiaDiag.z = 1 / InertiaDiag.z;

        //!!!! DIAG ELEMENTS CANNOT BE ZERO !!!!

        // init IMU
        virtualIMU.last_rotation = rotation;
        

        // load mesh for drone
        droneMesh = GenMeshCube(1.0f, 0.25f, 1.0f);


        //init Mahony Filter
        



}


// apply deadband to inputs preventing noise
void Quadcopter::apply_deadband(){

    // copy values from gm_input to local variable gm_input_deadband
    if (gm_input != nullptr) {
        gm_input_deadband.throttle = gm_input->throttle;
        gm_input_deadband.yaw = gm_input->yaw;
        gm_input_deadband.roll = gm_input->roll;
        gm_input_deadband.pitch = gm_input->pitch;

        // apply deadband
        if(abs(gm_input_deadband.throttle) <= JoystickDeadband)
            gm_input_deadband.throttle = 0;

        if(abs(gm_input_deadband.yaw) <= JoystickDeadband)
            gm_input_deadband.yaw = 0;

        if(abs(gm_input_deadband.roll) <= JoystickDeadband)
            gm_input_deadband.roll = 0;

        if(abs(gm_input_deadband.pitch) <= JoystickDeadband)
            gm_input_deadband.pitch = 0;
    } else {
        std::cout << "Gamepad is nullptr !!!" << std::endl;
    }
}


void Quadcopter::UpdateDronePhysics(float dT){

    Tools::Vector3 FG = gravity * mass;

    //std::cout << "Thrust: "<< Q4Motors.vertical_thrust << std::endl; 

    // aerodynamics drag
    Tools::Vector3 aeroDrag = velocity * (-0.5f * CX_coef * velocity.magnitude());

    Tools::Vector3 thrustBody = {0 ,Q4Motors.vertical_thrust ,0 };
    //std::cout << "Thrust: "<< thrustBody << std::endl;

    // calculate thrust in local coordinates
    
    Tools::Vector3 thrustGlobal = thrustBody * rotation.inverse();
  
    //std::cout << "Drone Rotation: "<< rotation << std::endl;

    Tools::Vector3 totalF = FG + thrustGlobal + aeroDrag;
    //std::cout << "TotalF: "<< totalF << std::endl;
    Tools::Vector3 acceleration = totalF / mass;

    // integrate acceleration velocity to possition
    velocity = velocity + acceleration * dT;
    position = position + velocity * dT;

    // Rotation
    Tools::Vector3 omegaBody = angularMomentumBody * invInertiaDiag;

    //cross produvt of omegaBody and angularMomentumBody
    Tools::Vector3 crossTerm = omegaBody.cross(angularMomentumBody);

    //Tools::Vector3 dLdT = (Q4Motors.momentums * rotation.inverse()) - crossTerm;
    Tools::Vector3 dLdT = Q4Motors.momentums - crossTerm;

    // integrate angular momentum
    angularMomentumBody = angularMomentumBody + (dLdT * dT);

    // update omega body
    omegaBody = angularMomentumBody * invInertiaDiag;

    Tools::Quaternion qDot = rotation.derivative(omegaBody);

    //integrate rotation
    rotation = rotation + (qDot * dT);
    rotation = rotation.normalize();

}






// GameObject methods
void Quadcopter::Update(float dt) {
    
    apply_deadband();

    

    #ifndef QUATERNION_CONTROLLER


        Tools::Vector3 localOmega = virtualIMU.SimulateGyro(rotation, dt);

        std::cout << "  Controller " << std::endl << std::endl;

        //std::cout << "Local Omega: " << localOmega << std::endl;
        //std::cout << "Local Acc: " << localAcc << "    magnitude: "<<localAcc.magnitude() << std::endl;

        //est_rot = mahony_filter.perform_mahony_fusion_6DOF(localAcc, localOmega, est_rot, dt);
        std::cout << "True Rotation est Eulers: " <<rotation.ToEuler() * 180 / PI << std::endl;

        Tools::Vector3 PID_out = PID.RatePID(gm_input_deadband, localOmega, dt);

        gm_input_deadband.pitch = PID_out.x;
        gm_input_deadband.yaw = PID_out.y;
        gm_input_deadband.roll = PID_out.z;


        Q4Motors.UpdateMotors(gm_input_deadband);

    #else

        Tools::Vector3 localOmega = virtualIMU.SimulateGyro(rotation, dt);
        Tools::Vector3 localAcc = virtualIMU.SimulateAccelerometer(rotation, velocity, dt);

        //std::cout << "Local Omega: " << localOmega << std::endl;
        //std::cout << "Local Acc: " << localAcc << "    magnitude: "<<localAcc.magnitude() << std::endl;

        //est_rot = mahony_filter.perform_mahony_fusion_6DOF(localAcc, localOmega, est_rot, dt);
        std::cout << "True Rotation est Eulers: " <<rotation.ToEuler() * 180 / PI << std::endl;

        double ref_pitch = gm_input_deadband.pitch * 2;
        double ref_yaw   = gm_input_deadband.yaw;
        double ref_roll  = gm_input_deadband.roll * 2;
        
        //Tools::Quaternion q_ref(-ref_roll, ref_yaw, -ref_pitch); 

        // Convert gamepad angles to a local offset quaternion:
        Tools::Quaternion q_offset(-ref_roll, ref_yaw, -ref_pitch);  

        // Compose the new reference orientation via multiplication (not addition):
        Tools::Quaternion q_ref = rotation * q_offset;  

        // gamepad_ref_angles.x = -ref_roll;
        // gamepad_ref_angles.y += -ref_yaw;
        // gamepad_ref_angles.z = -ref_pitch;

        //Tools::Quaternion q_ref(gamepad_ref_angles.x, gamepad_ref_angles.y, gamepad_ref_angles.z); 

        std::cout <<"Qref: " << q_ref << std::endl;

        Tools::Vector3 QCntrl_Out = QCntrl.QControllerUpdate(q_ref, rotation, localOmega, dt);

        std::cout <<"Quaternion Controller Out: " << QCntrl_Out << std::endl;


        gm_input_deadband.pitch = QCntrl_Out.x;
        gm_input_deadband.yaw = QCntrl_Out.y;
        gm_input_deadband.roll = QCntrl_Out.z;

        Q4Motors.UpdateMotors(gm_input_deadband);

    #endif

    UpdateDronePhysics(dt);

    if(position.y <= 1){
        position.y = 1;
        velocity.y = 0;
    }


    // position.x = 0;
    // position.y = 5;
    // position.z = 0;


}


void Quadcopter::DrawDrone3D (Tools::Vector3 pos, Tools::Quaternion rotation, Tools::Vector3 size ){

    
    // Apply rotation and move it to position
    Matrix transform = MatrixMultiply(rotation.RotationMatrix(), MatrixTranslate(pos.x, pos.y, pos.z));
    
    // Draw the cube mesh with red color
    Material material = LoadMaterialDefault();
    material.maps[MATERIAL_MAP_DIFFUSE].color = RED;
    DrawMesh(droneMesh, material, transform);


}


void Quadcopter::Draw() {

    Vector3 r_pos = {position.x, position.y, position.z};
    //DrawCubeWires(r_pos, 1.0, 0.2, 1.0 , GREEN);

    Tools::Vector3 size = {1,1,1};
    DrawDrone3D(position,rotation,size);

}



void Quadcopter::Draw2D() {

    //Draw drone velocity

    float HUD_y = GetScreenHeight() - 50;
    
    DrawText(("Velocity: " + std::to_string(velocity.magnitude())).c_str(), 10, HUD_y, 25, GREEN);
    DrawText(("Attitude: " + std::to_string(position.y )).c_str(), 10, HUD_y - 25, 25, GREEN);

  

}