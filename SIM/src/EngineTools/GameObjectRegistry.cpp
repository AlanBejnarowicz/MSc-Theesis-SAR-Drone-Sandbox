#include "GameObjectRegistry.h"


GameObjectManager::GameObjectManager(){



}


GameObjectManager::~GameObjectManager(){


}

void GameObjectManager::PreStart(void){

    lightingInstanced = LoadShader("rsc/LIGHT_SHADERS/lighting_instancing.vs", "rsc/LIGHT_SHADERS/lighting.fs");

}



void GameObjectManager::Handle_Game_Objects() {

   for (auto& obj : objects_registry) {

        if(obj->start_done == false) {
            obj->SetRegistry(&objects_registry);
            obj->SetRenderQueue(&renderQueue);
            obj->Start(); 
            obj->start_done = true;
        }
    }

}





void GameObjectManager::Update_GameObjects(float dt) {

    // Parallel update
    #pragma omp parallel for
    for (int i = 0; i < static_cast<int>(objects_registry.size()); i++) {
        objects_registry[i]->Update(dt);
    }

}


void GameObjectManager::Draw_GameObjects() {


    for (auto& pair : renderQueue) {
        Model* masterModel = pair.first;
        std::vector<Matrix>& transforms = pair.second;

        if (transforms.empty()) continue;

        // Assign this shader to every material in your master model
        for (int i = 0; i < masterModel->materialCount; i++) {
            masterModel->materials[i].shader = lightingInstanced;
        }

        for (int i = 0; i < masterModel->meshCount; i++) {            
            // Raylib sends the 'transforms' array directly to the vertex shader
            DrawMeshInstanced(
                masterModel->meshes[i], 
                masterModel->materials[masterModel->meshMaterial[i]], 
                transforms.data(), 
                (int)transforms.size()
            );
        }
        // Clear for the next frame
        transforms.clear();
    }


    for (auto& obj : objects_registry) {

            obj->Draw(); 
    }

}


void GameObjectManager::Draw2D_GameObjects() {

        for (auto& obj : objects_registry) {

            obj->Draw2D(); 
    }

}




