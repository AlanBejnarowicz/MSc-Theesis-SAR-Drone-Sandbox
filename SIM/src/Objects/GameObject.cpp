#include "GameObject.h"



void GameObject::DrawVectors(bool local) {

    float thickness = 0.03f;  // Radius of the cylinder
    float length = 5.0f;
    
    // X axis (red)
    Tools::Vector3 startX = position;
    Tools::Vector3 endX = {position.x + length, position.y, position.z};
    DrawCylinderEx(startX, endX, thickness, thickness, 2, RED);
    
    // Y axis (green)
    Tools::Vector3 startY = position;
    Tools::Vector3 endY = {position.x, position.y + length, position.z};
    DrawCylinderEx(startY, endY, thickness, thickness, 2, GREEN);
    
    // Z axis (blue)
    Tools::Vector3 startZ = position;
    Tools::Vector3 endZ = {position.x, position.y, position.z + length};
    DrawCylinderEx(startZ, endZ, thickness, thickness, 2, BLUE);
    
    // Add sphere caps for better appearance
    DrawSphere(endX, thickness * 2.0f, RED);
    DrawSphere(endY, thickness * 2.0f, GREEN);
    DrawSphere(endZ, thickness * 2.0f, BLUE);

}





