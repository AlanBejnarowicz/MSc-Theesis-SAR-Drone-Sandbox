
#include "main_camera.h"



MainCamera::MainCamera(GameObject* parent)// Initialize gamepad to 0 or the desired index
{

    parrent_obj = parent;
    
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
void MainCamera::Update(float dt) {


    #ifndef TEST_CAM_ORBIT
    //set camera to possition of drone, as FPV camera
    // Assuming the first game object is the drone

    if (parrent_obj) {
        Vector3 dronePosition = parrent_obj->position;
        Tools::Quaternion droneRotation = parrent_obj->rotation;

        // Set camera position to drone's position
        camera.position = dronePosition;

        // Forward direction for FPV camera is negative Z
        Tools::Vector3 v = {0,0,1};
        Tools::Vector3 camforward = v * droneRotation.inverse();
        // Update camera target
        camera.target = Vector3Add(camera.position, camforward);

        // up vector (positive Y) rotated by drone quaternion
        Tools::Vector3 up = Tools::Vector3(0, 1, 0) * droneRotation.inverse();
        camera.up = up;
    }


    // Update
    UpdateCamera(&camera, CAMERA_FIRST_PERSON);

    #endif

    #ifdef TEST_CAM_ORBIT

    static float angle = 0.0f;
    angle += -0.1f * dt; // Adjust the speed of the orbit here

    // Calculate the new camera position
    camera.position.x = 15.0f * cos(angle);
    camera.position.z = 15.0f * sin(angle);
    camera.position.y = 10;

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