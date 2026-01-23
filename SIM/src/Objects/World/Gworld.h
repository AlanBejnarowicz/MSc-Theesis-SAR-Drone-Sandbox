#ifndef GWORLD_H
#define GWORLD_H

#include "raylib.h"
#include "rlgl.h"       
#include "raymath.h" 


#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <cstring>

#include <algorithm>
#include <execution>


#include "MyVector.h"
#include "GameObject.h"

class GameWorld : public GameObject{

    private:
    Texture2D def_texture;
    Material def_material;



    // prebaked mesh

    std::vector <Model> world_chunks;
    std::vector <Tools::Vector3> chunkWorldPosition;
    bool meshGenerated = false;


    int chunk_size = 32;
    int world_size_chunks_squared = 16;
    float voxel_size = 0.25;


    public:

    GameWorld();
    ~GameWorld();


    Mesh GenCombinedVoxelMesh(const std::vector<Vector3>& voxels);


    void Update(float dt) override;
    void Draw() override;
    void Draw2D() override;


};



#endif