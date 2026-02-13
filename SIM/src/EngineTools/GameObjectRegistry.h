#pragma once


#include <iostream>
#include <math.h>


#include <thread>
#include <atomic>
#include <chrono>
#include <mutex>
#include <vector>
#include <memory>
#include <iostream>




// include Objects
#include "GameObject.h"
#include "ModelRegistry.h"

#include "DroneShip/DroneShip.h"
#include "SEA_model/sea.h"




class GameObjectManager  {

    private:

    // main collection of GameObjects
    std::vector<std::unique_ptr<GameObject>> objects_registry;    

    
    public:

    GameObjectManager(); // default constructor
    ~GameObjectManager();


    ModelManager *model_manager = nullptr;



    void Update_GameObjects(float dt);
    void Draw_GameObjects();
    void Draw2D_GameObjects();
    void PreStart(void);

    int id_counter = 0;


template<typename T, typename... Args> 
T* Spawn(std::string name, Args&&... args) {
    auto newObj = std::make_unique<T>(std::forward<Args>(args)...);
    
    T* ptr = newObj.get();
    ptr->GameObjectName = name;
    ptr->GameObjectID = id_counter++;
    ptr->model_manager = model_manager;

    objects_registry.push_back(std::move(newObj));
    return ptr;
}



    void Handle_Game_Objects();
    

};


