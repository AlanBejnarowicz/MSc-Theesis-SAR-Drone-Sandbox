
#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H


#include <vector>
#include <memory>
#include <raylib.h>
#include <string>


#include "MyVector.h"
#include "quaternion.h"
#include "ModelRegistry.h"



// Abstract base class (interface)
class GameObject {
    public:

        GameObject()
        {

        }



        virtual void Start() = 0;  // Must be implemented in subclasses
        virtual void Update(float dt) = 0;  // Pure virtual method (must be implemented)
        virtual void Draw() = 0;  // Must be implemented in subclasses
        virtual void Draw2D() = 0;  // Must be implemented in subclasses
        

        virtual ~GameObject() {}  // Virtual destructor for proper cleanup


        void SetRegistry(std::vector<std::unique_ptr<GameObject>>* globalList) {
            registry = globalList;
        }


        std::string GameObjectName = "NewGameObject";
        int GameObjectID = 0;

        ModelManager *model_manager = nullptr;

        // set true if Start() was already called
        bool start_done = false;
        

    protected:
        void DrawVectors(bool local = false);

        Tools::Vector3 position;
        Tools::Quaternion rotation;
        std::vector<std::unique_ptr<GameObject>>* registry = nullptr;

        



        template<typename T> 
        T* GetGameObjectByName(std::string GO_name) {
            if (!registry) return nullptr;
            for (auto& obj : *registry) {
                if (obj->GameObjectName == GO_name) {
                    T* casted = dynamic_cast<T*>(obj.get());
                    if (casted) return casted;
                }
            }
            return nullptr;
        }

        
        
};



#endif