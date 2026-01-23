
#include "VGamepad.h"



VGamepad::VGamepad()// Initialize gamepad to 0 or the desired index
{

}

VGamepad::VGamepad(Tools::GM_Inputs* gmpd) // Initialize gamepad to 0 or the desired index
{
    gm_input = gmpd;
}

VGamepad::~VGamepad()
{

}

// GameObject methods
void VGamepad::Update(float dt) {

    axis_1.x = gm_input->roll;
    axis_1.y = -gm_input->pitch;
    axis_2.x = gm_input->yaw;
    axis_2.y = -(gm_input->throttle * 2 - 1);
    
}

void VGamepad::Draw() {
    

}

void VGamepad::Draw2D() {
    // to optimise

    int screen_w = GetScreenWidth();
    int screen_h = GetScreenHeight();

    float s = (screen_h * VGM_square_size);

    float box_x = (screen_w / 2);
    float box_y = (screen_h * VGM_square_Y_pos);

    float separation = (s * VGM_box_separation);

    Rectangle r1 = {box_x + separation, box_y, s, s};
    Rectangle r2 = {box_x - s - separation, box_y, s, s};

    Vector2 c1 = {r1.x + (r1.width/2), r1.y + (r1.height/2)};
    Vector2 c2 = {r2.x + (r2.width/2), r2.y + (r2.height/2)};

    // to optimise, can be run only one window resized


    DrawRectangleRounded(r1, 0.1f, 10, fillColor);
    DrawRectangleRounded(r2, 0.1f, 10, fillColor);

    DrawRectangleRoundedLinesEx(r1, 0.1f, 10, 2.0, extColor);
    DrawRectangleRoundedLinesEx(r2, 0.1f, 10, 2.0, extColor);

    
    Tools::Vector3 c1_ax1(r1.x + (r1.width/2), r1.y + (r1.height/2), 0);
    Tools::Vector3 c2_ax2(r2.x + (r2.width/2), r2.y + (r2.height/2),0);

    c1_ax1 = c1_ax1 + (axis_1 * s / 2);
    c2_ax2 = c2_ax2 + (axis_2 * s / 2);

    Vector2 v1 = {c1_ax1.x, c1_ax1.y};
    Vector2 v2 = {c2_ax2.x, c2_ax2.y};

    DrawCircleV(v1, 5, dotColor);
    DrawCircleV(v2, 5, dotColor);

    //const char* gamepadText = TextFormat("GP%d: %s", gamepad, GetGamepadName(gamepad));
    //int textWidth = MeasureText(gamepadText, 20);
    //DrawText(gamepadText, box_x - (textWidth / 2), box_y + s + separation, 20, BLACK);



}