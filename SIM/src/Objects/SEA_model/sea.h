
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

        float wave_amplitude_scale = 3.0f;

        float spacing = 10.0f; // Distance between vertices
        int SEA_vertices_width = 50;     // Number of vertices along width
        int SEA_vertices_depth = 50;     // Number of vertices along depth

        //camera possition
        Tools::Vector3 cam_possition;

        float timer;

        float calculateWave(Tools::Vector3 pos, float freq, float speed, float amp);
        
        void update_wave(void);


    public:

        SEA(); // default constructor
        ~SEA();


        void Start() override;
        void Update(float dt) override;
        void Draw() override;
        void Draw2D() override;



        Tools::Vector3 get_wave_normal(Tools::Vector3 pos);
        float calculate_total_wave_height(Tools::Vector3 pos);



    

};


#endif


