#include "GameObjectRegistry.h"


GameObjectManager::GameObjectManager(){

}


GameObjectManager::~GameObjectManager(){

}



void GameObjectManager::Handle_Game_Objects() {

   for (auto& obj : objects_registry) {

        if(obj->start_done == false) {
            obj->SetRegistry(&objects_registry);
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

    for (auto& obj : objects_registry) {

            obj->Draw(); 
    }

}


void GameObjectManager::Draw2D_GameObjects() {

        for (auto& obj : objects_registry) {

            obj->Draw2D(); 
    }

}




