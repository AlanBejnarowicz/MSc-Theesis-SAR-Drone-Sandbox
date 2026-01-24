
#ifndef DRONESHIP_H
#define DRONESHIP_H

#include <iostream>
#include <math.h>
#include <SDL2/SDL.h>
#include <raylib.h>
#include "raymath.h"


#include "MyVector.h"
#include "GameObject.h"


class DroneShip : public GameObject {

    private:
    Tools::Vector3 InertiaDiag = {0.084,0.084,0.168};
    Tools::Vector3 invInertiaDiag;
    Tools::Vector3 angularMomentumBody;
    float mass = 1.0;

    // Rotate model to align with forward direction
    Tools::Quaternion model_correction = Tools::Quaternion(M_PI, 0.0f, M_PI/2); 


    public:
    Model ship_model;
    float ship_length = 5.0f;



    







    DroneShip(); // default constructor
    ~DroneShip();

    void Update(float dt) override;
    void Draw() override;
    void Draw2D() override;
    

};


#endif