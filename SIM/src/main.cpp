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

//#include "Objects/HUD/HUD.h"


// main collection of GameObjects
std::vector<std::unique_ptr<GameObject>> gameObjects;




int main() {
    
    // Camera and window initialization

    const int screenWidth = 2000;
    const int screenHeight = 1200;

    SetConfigFlags(FLAG_MSAA_4X_HINT);      // Enable Multi Sampling Anti Aliasing 4x (if available)

    
    InitWindow(screenWidth, screenHeight, "RayDroneSim");

    SetTargetFPS(120);


    // ############### GAMEPAD FIX ############### 

    Tools::InputSystem Input_System;

    // Get pointer to GMInputs
    Tools::GM_Inputs* GMInputs = Input_System.GetGM_Inputs_ptr();


    // ############### END GAMEPAD FIX ###############




    // #########################
    //  $$$$$ Init Objects $$$$$

    // gameObjects.push_back(std::make_unique<VGamepad>(GMInputs));
    // gameObjects.push_back(std::make_unique<Quadcopter>(GMInputs));
    // gameObjects.push_back(std::make_unique<HUD>(gameObjects[1].get()));
    //gameObjects.push_back(std::make_unique<GameWorld>());

    //MainCamera mainCamera(nullptr);


    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        // Handle SDL input for gamepad
        // Input_System.HandleGamepadInput();

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




    //SDL_GameControllerClose(gamepad);

    CloseWindow(); // Close window and OpenGL context
    return 0;
}