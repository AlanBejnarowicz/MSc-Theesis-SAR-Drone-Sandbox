#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"

//#include <SDL2/SDL.h>
#include <omp.h>      


#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>
#include <memory>
#include <iostream>


// include tools
#include "Tools/quaternion.h"
#include "Tools/MyVector.h"


// include Engine Tools
#include "GameObjectRegistry.h"
#include "ModelRegistry.h"


// include Objects
#include "GameObject.h"
#include "Objects/MainCamera/main_camera.h"




#include "DroneShip/DroneShip.h"
#include "SEA_model/sea.h"




bool grid_enabled = true;


int main() {
    SetTraceLogLevel(LOG_WARNING);
    // Camera and window initialization

    const int screenWidth = 2000;
    const int screenHeight = 1200;

    SetConfigFlags(FLAG_MSAA_4X_HINT);      // Enable Multi Sampling Anti Aliasing 4x (if available)    
    InitWindow(screenWidth, screenHeight, "MarineSIM - SAR Drone Sandbox");
    SetTargetFPS(120);

    GameObjectManager GO_Manager;
    ModelManager model_manager;
    MainCamera mainCamera;



    // #########################
    //  $$$$$ Init Objects $$$$$

    GO_Manager.PreStart();
    model_manager.LoadImportantModels();

    GO_Manager.model_manager = &model_manager;

    GO_Manager.Spawn<SEA>("MainSea");

    float spacing = 10.0;

    for(int k = 0; k < 30; k++){

        for(int i = 0; i < 30; i++){
            Tools::Vector3 spawn_pos = {k * spacing, 0, i * spacing};
            GO_Manager.Spawn<DroneShip>("Drone_01", spawn_pos);
        }  
        
    }


    



    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        GO_Manager.Handle_Game_Objects();


        GO_Manager.Update_GameObjects(dt);
        mainCamera.Update(dt);
        

        BeginDrawing();
        ClearBackground(GRAY);
        BeginMode3D(mainCamera.camera);

        if(grid_enabled) DrawGrid(50000, 5.0f);

            GO_Manager.Draw_GameObjects();

        EndMode3D();
            



        // 2D UI
        GO_Manager.Draw2D_GameObjects();


        // draw FPS number
        if (dt != 0)
        {
            DrawText(TextFormat("FPS: %i", (int)(1.0f/dt)), GetScreenWidth() - 220, 40, 20, GREEN);
        }

        EndDrawing();

    }



    CloseWindow(); // Close window and OpenGL context
    return 0;
}