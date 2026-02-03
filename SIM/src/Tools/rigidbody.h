#pragma once


#include <vector>
#include <memory>
#include <raylib.h>
#include <string>


#include "MyVector.h"
#include "quaternion.h"


struct DebugForce {
    Tools::Vector3 start;
    Tools::Vector3 end;
    Color color;
};


// Abstract base class (interface)
class Rigidbody {

    private:
        // Pointers to the GameObject's actual data
        Tools::Vector3* position = nullptr;
        Tools::Quaternion* rotation = nullptr;

        Tools::Vector3 temp_position;
        Tools::Quaternion temp_rotation;


        std::vector<Tools::Vector3> forces;
        std::vector<Tools::Vector3> forces_pos;

        std::vector<DebugForce> debugLines;

    public:

        Rigidbody(); // default constructor
        ~Rigidbody();   


        void Update(float dt);
        void Draw();

        void BindTransform(Tools::Vector3& pos, Tools::Quaternion& rot);


        void AddForce(Tools::Vector3 pos, Tools::Vector3 force);
        void AddLocalForce(Tools::Vector3 pos, Tools::Vector3 force);


        Tools::Vector3 GetVelocityAtPoint (Tools::Vector3 localPoint);


        Tools::Vector3 InertiaDiag = {0.084,0.084,0.168};
        Tools::Vector3 invInertiaDiag;
        Tools::Vector3 angularMomentumBody;

        // centre of rotation ~COG
        Tools::Vector3 COR = {0,0,0};


        Tools::Vector3 velocity;
        float mass = 1.0;
        float angularDragCoeff = 0.5f;


        // if set to false possition and rotation wont be updated
        bool isKinematic = true;


    


        
};