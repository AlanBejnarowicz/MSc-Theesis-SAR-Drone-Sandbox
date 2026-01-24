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


// include Objects
#include "Objects/GameObject.h"
#include "Objects/MainCamera/main_camera.h"

#include "Objects/DroneShip/DroneShip.h"
#include "Objects/SEA_model/sea.h"


// main collection of GameObjects
std::vector<std::unique_ptr<GameObject>> gameObjects;




int main() {
    
    // Camera and window initialization

    const int screenWidth = 2000;
    const int screenHeight = 1200;

    SetConfigFlags(FLAG_MSAA_4X_HINT);      // Enable Multi Sampling Anti Aliasing 4x (if available)    
    InitWindow(screenWidth, screenHeight, "MarineSIM - SAR Drone Sandbox");
    SetTargetFPS(120);




    // #########################
    //  $$$$$ Init Objects $$$$$

    gameObjects.push_back(std::make_unique<DroneShip>());
    gameObjects.push_back(std::make_unique<SEA>());




    MainCamera mainCamera(nullptr);


    while (!WindowShouldClose()) {
        float dt = GetFrameTime();


        // Parallel update
        #pragma omp parallel for
        for (int i = 0; i < static_cast<int>(gameObjects.size()); i++) {
            gameObjects[i]->Update(dt);
        }

        mainCamera.Update(dt);
        

        // Draw
        BeginDrawing();


        //ClearBackground(RAYWHITE);
        ClearBackground(GRAY);

        
        BeginMode3D(mainCamera.camera);


        DrawGrid(5000, 1.25f);

        for (auto& obj : gameObjects) {
            obj->Draw();
        }


        EndMode3D();


        for (auto& obj : gameObjects) {
            obj->Draw2D();
        }

        mainCamera.Draw2D();

        EndDrawing();


    }



    CloseWindow(); // Close window and OpenGL context
    return 0;
}