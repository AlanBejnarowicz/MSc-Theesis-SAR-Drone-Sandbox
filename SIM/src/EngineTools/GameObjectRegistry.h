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



// include Engine Tools
#include "GameObjectRegistry.h"


// include Objects
#include "GameObject.h"

#include "DroneShip/DroneShip.h"
#include "SEA_model/sea.h"




class GameObjectManager  {

    private:

    // main collection of GameObjects
    std::vector<std::unique_ptr<GameObject>> objects_registry;    

    public:

    GameObjectManager(); // default constructor
    ~GameObjectManager();


    void Update_GameObjects(float dt);
    void Draw_GameObjects();
    void Draw2D_GameObjects();

    int id_counter = 0;


    template<typename T> T* Spawn(std::string name) {
        auto newObj = std::make_unique<T>();
        T* ptr = newObj.get();
        ptr->GameObjectName = name;
        ptr->GameObjectID = id_counter ++;
        objects_registry.push_back(std::move(newObj));
        return ptr;
    }



    void Handle_Game_Objects();
    

};


