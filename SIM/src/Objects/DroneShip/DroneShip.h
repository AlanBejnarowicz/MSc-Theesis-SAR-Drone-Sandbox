
#ifndef DRONESHIP_H
#define DRONESHIP_H

#include <iostream>
#include <math.h>
#include <raylib.h>
#include "raymath.h"
#include <vector>


#include "MyVector.h"
#include "GameObject.h"
#include "rigidbody.h"
#include "tracker.h"


#include "SEA_model/sea.h"


class DroneShip : public GameObject {

    private:
        Tools::Vector3 InertiaDiag = {0.084,0.084,0.168};
        Tools::Vector3 invInertiaDiag;
        Tools::Vector3 angularMomentumBody;
        float ship_mass = 1.0;

        // Rotate model to align with forward direction
        Tools::Quaternion model_correction = Tools::Quaternion(M_PI, 0.0f, M_PI/2); 

        SEA* _sea_model = nullptr;





        void FloatPhysics(float dt);


    public:
        Model ship_model;

        Rigidbody rb;
        Tracker tr;


        float ship_length = 5.0f;
        float ship_width = ship_length / 5.0f;
        float ship_CG_possition = -0.8f;
        float ship_IM_correction = -0.5f;

        Tools::Vector3 Ship_CG;

        // buoyancy points 
        std::vector<Tools::Vector3> buoyancy_points;





        DroneShip(); // default constructor
        DroneShip(Tools::Vector3 pos); // possition constructor
        ~DroneShip();


        void Start() override;
        void Update(float dt) override;
        void Draw() override;
        void Draw2D() override;




};


#endif