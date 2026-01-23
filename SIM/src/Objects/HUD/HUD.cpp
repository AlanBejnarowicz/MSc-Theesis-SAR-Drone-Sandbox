#include "HUD.h"


HUD::HUD(GameObject* parent){

    parrent_obj = parent;

    screen_centre_height = GetScreenHeight() / 2;
    screen_width = GetScreenWidth();
    screen_height = GetScreenHeight();

}


HUD::~HUD(){


}


// GameObject methods
void HUD::Update(float dt) {

    rotation = parrent_obj->rotation;
    position = parrent_obj->position;

}

void HUD::Draw() {
    

}

void HUD::Draw2D() {

    DrawAftHorizon();

}






void HUD::DrawAftHorizon(){

    Tools::Vector3 rt = rotation.ToEuler();

    // convert to deg
    rt = rt * 180 / PI;

    Vector2 p1 = {250, screen_centre_height};
    Vector2 p2 = {500, screen_centre_height};

    DrawLineEx(p1, p2, 5 , HUD_color);

    p1 = {screen_width - 250, screen_centre_height};
    p2 = {screen_width - 500, screen_centre_height};

    DrawLineEx(p1, p2, 5 , HUD_color);



    p1 = {550, screen_centre_height};
    p2 = {screen_width - 550, screen_centre_height};

    p1.y = p1.y + rt.x * pitch_scale;
    p2.y = p2.y + rt.x * pitch_scale;

    DrawLineEx(p1, p2, 4 , HUD_color);

    // DrawLine(18, screen_centre_height, 500, screen_centre_height, GREEN);
    // DrawLine(18, screen_centre_height - 5, 500, screen_centre_height - 5, GREEN);
    // DrawLine(18, screen_centre_height + 5, 500, screen_centre_height + 5, GREEN);


}