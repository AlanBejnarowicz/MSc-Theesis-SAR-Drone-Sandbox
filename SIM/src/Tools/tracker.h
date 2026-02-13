#pragma once


#include <vector>
#include <memory>
#include <raylib.h>
#include <string>


#include "MyVector.h"
#include "quaternion.h"


#include "SEA_model/sea.h"


// Abstract base class (interface)
class Tracker {

    private:
        // Pointers to the GameObject's actual data
        Tools::Vector3* position = nullptr;
        Tools::Quaternion* rotation = nullptr;

        Tools::Vector3 last_position;

        std::vector<Tools::Vector3> dots_vector;


        
        Color color = YELLOW;

        SEA* _sea_model = nullptr;


    public:

        Tracker(float marker_distance = 1.0f); // default constructor
        ~Tracker();   

        void BindTransform(Tools::Vector3& pos, Tools::Quaternion& rot);
        void BindSea(SEA& sea);


        void Update(float dt);
        void Draw();


        float mark_dist = 1.0f;
        int max_points = 100;



    


        
};