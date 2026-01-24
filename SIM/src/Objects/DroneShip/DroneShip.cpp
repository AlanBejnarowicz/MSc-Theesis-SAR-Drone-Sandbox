
#include "DroneShip.h"



DroneShip::DroneShip()
{
    ship_model = LoadModel("rsc/ONR_SHIP/ONR_TUMBLEHOME.obj");
    if (ship_model.meshCount > 0) {
        std::cout << "DroneShip model loaded successfully!" << std::endl;
    } else {
        std::cerr << "Failed to load DroneShip model!" << std::endl;
    }




}

DroneShip::~DroneShip()
{

}



// GameObject methods
void DroneShip::Update(float dt) {

    // Apply rotation and move it to position
    Matrix transform = MatrixMultiply(rotation.RotationMatrix(), MatrixTranslate(position.x, position.y, position.z));
    transform = transform * MatrixScale(ship_length, ship_length, ship_length);
    // Correct model orientation
    transform = MatrixMultiply(model_correction.RotationMatrix(), transform);
    ship_model.transform = transform;


}

void DroneShip::Draw() {

    // Draw the ship model with proper transformation
    DrawModel(ship_model, {0, 0, 0}, 1.0, WHITE);
    
    DrawVectors();

   
    

}

void DroneShip::Draw2D() {
   

}


