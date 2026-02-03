#include "rigidbody.h"


Rigidbody::Rigidbody(){

    //precompute diagonal inverse inertia
    //!!!! DIAG ELEMENTS CANNOT BE ZERO !!!!

    invInertiaDiag.x = 1 / InertiaDiag.x;
    invInertiaDiag.y = 1 / InertiaDiag.y;
    invInertiaDiag.z = 1 / InertiaDiag.z;



}


Rigidbody::~Rigidbody(){


}



void Rigidbody::Update(float dt){



    // update GameObject possition and rotation
    if(rotation != nullptr && position != nullptr) {
        temp_position = *position;
        temp_rotation = *rotation;
    } else {
        std::cout << "Rigidbody not binded to GameObject! \r\n";
    }




    Tools::Vector3 total_forces(0,0,0);
    Tools::Vector3 total_momentums(0,0,0);


    // count all forces and momentums
    for(int i = 0; i < forces.size(); i++){
        total_forces = total_forces + forces[i];

        Tools::Vector3 momentum (0,0,0);
        momentum = forces[i].cross(forces_pos[i] - COR);
        total_momentums = total_momentums + momentum;
    }


    // Clear previous frame's debug lines
    debugLines.clear();



    for (size_t i = 0; i < forces.size(); i++) {
        // 1. Convert the local application point to world space
        // worldPos = (Rotation * localPos) + Translation
        Tools::Vector3 worldStart = ((forces_pos[i] + temp_position )  ) ;
        
        // 2. The force vector itself (usually defined in world space or 
        // transformed to world space)
        float vectror_lenght = 3.5f; // Adjust this so lines aren't miles long
        Tools::Vector3 worldEnd = worldStart + (forces[i].normalize() * vectror_lenght);

        // Store for the Draw() call
        debugLines.push_back({ worldStart, worldEnd, YELLOW });
    }



    // clear arrays
    forces.clear();
    forces_pos.clear();


    Tools::Vector3 acceleration = total_forces / mass;

    if(isKinematic == false){
        velocity = {0,0,0};
        acceleration = {0,0,0};
    }

    // integrate acceleration velocity to possition
    velocity = velocity + acceleration * dt;
    temp_position = temp_position + velocity * dt;



    // Rotation
    Tools::Vector3 omegaBody = angularMomentumBody * invInertiaDiag;
    Tools::Vector3 dragTorque = omegaBody * (-angularDragCoeff);

    //cross produvt of omegaBody and angularMomentumBody
    Tools::Vector3 crossTerm = omegaBody.cross(angularMomentumBody);
    Tools::Vector3 dLdT =  (total_momentums + dragTorque) - crossTerm;

    // integrate angular momentum
    angularMomentumBody = angularMomentumBody + (dLdT * dt);

    // update omega body
    omegaBody = angularMomentumBody * invInertiaDiag;

    Tools::Quaternion qDot = temp_rotation.derivative(omegaBody);

    //integrate rotation
    temp_rotation = temp_rotation + (qDot * dt);
    temp_rotation = temp_rotation.normalize();





    // update GameObject possition and rotation
    if(rotation != nullptr && position != nullptr) {
        *position = temp_position;
        *rotation = temp_rotation;
    } else {
        std::cout << "Rigidbody not binded to GameObject! \r\n";
    }


}



void Rigidbody::Draw(){

    for (const auto& line : debugLines) {
        DrawLine3D(line.start, line.end, line.color);
        
        // Optional: Draw a small sphere at the point of application
        DrawSphere(line.end, 0.05f, YELLOW);
    }

}


// Link the Rigidbody to the GameObject data
void Rigidbody::BindTransform(Tools::Vector3& pos, Tools::Quaternion& rot) {
    position = &pos;
    rotation = &rot;
}


void Rigidbody::AddForce(Tools::Vector3 pos, Tools::Vector3 force) {

    Tools::Vector3 local_pos = pos * temp_rotation.inverse();

    forces.push_back(force);
    forces_pos.push_back(local_pos);

}


void Rigidbody::AddLocalForce(Tools::Vector3 pos, Tools::Vector3 force) {

    Tools::Vector3 local_force = force * temp_rotation.inverse();
    Tools::Vector3 local_pos = pos * temp_rotation.inverse();

    forces.push_back(local_force);
    forces_pos.push_back(local_pos);

}





Tools::Vector3 Rigidbody::GetVelocityAtPoint(Tools::Vector3 localPoint) {
    // 1. Calculate Angular Velocity in the body frame
    // (using your element-wise multiplication or define a specific transform)
    Tools::Vector3 OB(
        angularMomentumBody.x * invInertiaDiag.x,
        angularMomentumBody.y * invInertiaDiag.y,
        angularMomentumBody.z * invInertiaDiag.z
    );

    // 2. Calculate the distance from the Center of Rotation to the point in local space
    Tools::Vector3 r = localPoint - COR;

    // 3. Transform Local Omega and Local R into World Space
    // Your library uses: Vector3 * Quaternion (which handles conjugate/sandwich product)
    Tools::Vector3 worldOmega = OB * (rotation->inverse()); 
    Tools::Vector3 worldR = r * (rotation->inverse());

    // 4. V_point = V_linear + (Omega_world x R_world)
    // 'velocity' is assumed to be world-space linear velocity
    return velocity + worldOmega.cross(worldR);
}

