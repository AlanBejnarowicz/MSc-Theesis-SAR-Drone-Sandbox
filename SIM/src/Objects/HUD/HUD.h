#ifndef HUD_H
#define HUD_H


#include <iostream>
#include <math.h>
#include <SDL2/SDL.h>

#include "MyVector.h"
#include "GameObject.h"

class HUD : public GameObject {

    private:
    GameObject* parrent_obj;
    float screen_centre_height = 0;

    float screen_width = 0;
    float screen_height = 0;

    float pitch_scale = 11.0f;

    void DrawAftHorizon();

    public:
        HUD(GameObject* parent);
        ~HUD();

    void Update(float dt) override;
    void Draw() override;
    void Draw2D() override;


    Color HUD_color = {0,200,0,200};

};


#endif