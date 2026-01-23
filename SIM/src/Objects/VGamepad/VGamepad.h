
#ifndef VGAMEPAD_H
#define VGAMEPAD_H

#include <iostream>
#include <math.h>



#include "MyVector.h"
#include "GameObject.h"
#include "InputSystem.h"


class VGamepad : public GameObject {

    private:

    float VGM_square_size = 0.15;
    float VGM_square_Y_pos = 0.8;
    float VGM_box_separation = 0.05;

    Color fillColor = {5,5,5, 80};
    Color extColor = {0,0,0,255};
    Color dotColor = {255,0,0,200};


    Tools::Vector3 axis_1;
    Tools::Vector3 axis_2;

    public:

    //int gamepad = 1; // which gamepad to display
    Tools::GM_Inputs* gm_input = nullptr;


    VGamepad(); // default constructor
    VGamepad(Tools::GM_Inputs* gmpd); // default constructor
    ~VGamepad();

    void Update(float dt) override;
    void Draw() override;
    void Draw2D() override;
    


};


#endif