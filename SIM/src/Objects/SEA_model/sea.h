
#ifndef SEA_MODEL_H
#define SEA_MODEL_H

#include <iostream>
#include <math.h>


#include "MyVector.h"
#include "GameObject.h"


class SEA : public GameObject {


    private:

        Mesh mesh;
        Model model;
        Texture2D texture;

        //shaders
        Shader sea_shader;
        int secondsLoc;


    float spacing = 1.0f; // Distance between vertices
    int width = 200;     // Number of vertices along width
    int depth = 200;     // Number of vertices along depth




    public:

        SEA(); // default constructor
        ~SEA();

        void Update(float dt) override;
        void Draw() override;
        void Draw2D() override;

    

};


#endif


