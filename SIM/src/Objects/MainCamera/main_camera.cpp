
#include "main_camera.h"



MainCamera::MainCamera()
{

    
    camera.position = { 0.0f, 5.0f, -10.0f }; // Like Unity (Camera starts behind)
    camera.target = { 0.0f, 2.0f, 0.0f };   // Looking towards +Z
    camera.up = { 0.0f, 1.0f, 0.0f };       // Keep Y-up (same as Unity)
    camera.fovy = 90.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    //CameraMode(camera, CAMERA_FREE); // Allows movement without fixed target

}

MainCamera::~MainCamera()
{

}

// GameObject methods

void MainCamera::Start(){



}


void MainCamera::Update(float dt) {


    #ifndef TEST_CAM_ORBIT
    //set camera to possition of drone, as FPV camera
    // Assuming the first game object is the drone

    UpdateCamera(&camera, CAMERA_FREE);



    #endif

    #ifdef TEST_CAM_ORBIT

    static float angle = 0.0f;
    //angle += -0.1f * dt; // Adjust the speed of the orbit here

    // Calculate the new camera position
    camera.position.x = 10.0f * cos(angle);
    camera.position.z = 10.0f * sin(angle);
    camera.position.y = 5;

    camera.position += CamObbitPos;

    // Keep the camera looking at the origin
    camera.target = CamObbitPos;

    // Update the camera
    UpdateCamera(&camera, CAMERA_ORBITAL);
    #endif



    // calculate FPS
    true_FPS = 1.0 / dt;

}

void MainCamera::Draw() {
    

}

void MainCamera::Draw2D() {


    std::string fpsText = "FPS: " + std::to_string(true_FPS);
    DrawText(fpsText.c_str(), 10, 50, 20, DARKGRAY);
   

}