
#ifndef MAINCAM_H
#define MAINCAM_H

#include <iostream>
#include <math.h>

#include "raylib.h"
#include "raymath.h"
#include "rcamera.h"

#include "MyVector.h"
#include "quaternion.h"
#include "GameObject.h"


#define TEST_CAM_ORBIT



class MainCamera : public GameObject {

    private:
        float true_FPS = 0;
    

    public:

        MainCamera(); // default constructor
        ~MainCamera();

        Tools::Vector3 CamObbitPos = {0,0,0};
        Camera3D camera = { 0 };


        void Start() override;
        void Update(float dt) override;
        void Draw() override;
        void Draw2D() override;
    

};


#endif