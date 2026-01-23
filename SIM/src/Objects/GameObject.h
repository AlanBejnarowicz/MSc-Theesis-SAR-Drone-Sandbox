
#ifndef GAMEOBJECT_H
#define GAMEOBJECT_H


#include <vector>
#include <memory>
#include <raylib.h>
#include <string>


#include "MyVector.h"
#include "quaternion.h"


// Abstract base class (interface)
class GameObject {
    public:
        virtual void Update(float dt) = 0;  // Pure virtual method (must be implemented)
        virtual void Draw() = 0;  // Must be implemented in subclasses
        virtual void Draw2D() = 0;  // Must be implemented in subclasses

        virtual ~GameObject() {}  // Virtual destructor for proper cleanup

        Tools::Vector3 position;
        Tools::Quaternion rotation;

        std::string name = "NewGameObject";
        
};



#endif