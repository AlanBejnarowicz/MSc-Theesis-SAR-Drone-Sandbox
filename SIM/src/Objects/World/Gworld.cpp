#include "Gworld.h"



GameWorld::GameWorld() {

    def_texture = LoadTexture("../Resources/Textures/grass.png");

    // generate map
    Tools::Vector3 GlobalVoxelPos = {0,-5,0}; 
    unsigned long long totalVoxels = 0;

    

    for (int chunks_x = 0; chunks_x < world_size_chunks_squared; chunks_x++) {
        for (int chunks_z = 0; chunks_z < world_size_chunks_squared; chunks_z++) {
            
            std::vector<Vector3> localVoxels;
            Tools::Vector3 voxelPos = {0,0,0}; 

            float lvx = 0;
            float lvz = 0;
            
            for (int c_x = 0; c_x < chunk_size; c_x++) {
                lvx += voxel_size;
                for (int c_z = 0; c_z < chunk_size; c_z++) {
                    lvz += voxel_size;

                    float x = lvx + GlobalVoxelPos.x;
                    float z = lvz + GlobalVoxelPos.z;
    
                    float height = sin(0.05f*x + 0.03f*z) 
                                   + cos(-0.022f*x + 0.033f*z) 
                                   + sin(0.04f*x - 0.03f*z);

                    height = height * 5.0;

                    // round to the nearest voxelsize

                   
    
                    Vector3 voxelPos;
                    voxelPos.x = lvx;
                    voxelPos.y = height;
                    voxelPos.z = lvz;
                    
                    totalVoxels++;
                    localVoxels.push_back(voxelPos);
                }
                lvz = 0;
            }
    
            // std::cout << "Chunk at x=" << chunks_x 
            //           << ", z=" << chunks_z
            //           << " has " << localVoxels.size() << " voxels\n";
    
            // 2) Generate mesh from localVoxels (NOT from box_map)
            Mesh voxelWorldMesh = GenCombinedVoxelMesh(localVoxels); 
            Model voxelWorldModel = LoadModelFromMesh(voxelWorldMesh);
            voxelWorldModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = def_texture;
    
            // 3) Push to the chunks list
            world_chunks.push_back(voxelWorldModel);
            chunkWorldPosition.push_back(GlobalVoxelPos);

            GlobalVoxelPos.z += voxel_size * chunk_size;

        }

        GlobalVoxelPos.z = 0;
        GlobalVoxelPos.x += voxel_size * chunk_size;
    
    }
    

    std::cout<<"Geneerated Voxels: " << totalVoxels << std::endl;



}

GameWorld::~GameWorld(){

}


// GameObject methods
void GameWorld::Update(float dt) {


}

void GameWorld::Draw() {

    for(int i = 0; i < world_chunks.size(); i++) {

        DrawModel(world_chunks[i], chunkWorldPosition[i], 1.0f, WHITE);
    }

}

void GameWorld::Draw2D() {
   

}





Mesh GameWorld::GenCombinedVoxelMesh(const std::vector<Vector3>& voxels)
{
    Mesh mesh = { 0 };

    const int cubeVertexCount = 24;
    const int cubeIndexCount  = 36; // not used directly but good reference

    int totalVoxels = (int)voxels.size();

    // Each voxel has 24 vertices -> total = totalVoxels * 24
    mesh.vertexCount   = totalVoxels * cubeVertexCount;
    // Each voxel has 12 triangles (36 indices / 3) -> total = totalVoxels * 12
    mesh.triangleCount = totalVoxels * 12;

    // Allocate arrays
    mesh.vertices  = (float*)RL_MALLOC(mesh.vertexCount*3*sizeof(float));
    mesh.texcoords = (float*)RL_MALLOC(mesh.vertexCount*2*sizeof(float));
    mesh.normals   = (float*)RL_MALLOC(mesh.vertexCount*3*sizeof(float));
    mesh.indices   = (unsigned short*)RL_MALLOC(mesh.triangleCount*3*sizeof(unsigned short));

    int vCounter = 0;
    int iCounter = 0;
    float half = voxel_size * 0.5f;


    // Loop through every voxel position
    for (int i = 0; i < totalVoxels; i++)
    {
        // Current voxel center
        float x = voxels[i].x;
        float y = voxels[i].y;
        float z = voxels[i].z;

        // ---------------------------------------------------------
        // We'll define 6 faces, each with 4 vertices & 6 indices
        // We'll replicate the same texture coords + normals
        // as in your DrawCubeTexture() function.
        // ---------------------------------------------------------

        // --------------------- FRONT FACE -----------------------
        // Normal: +Z
        // Texcoords: (0,0), (1,0), (1,1), (0,1)
        // Positions: see your code -> (x±half, y±half, z+half)
        {
            int faceBase = vCounter; // The first vertex of this face

            // Vertex 0
            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 1.0f;
            vCounter++;

            // Vertex 1
            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 1.0f;
            vCounter++;

            // Vertex 2
            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 1.0f;
            vCounter++;

            // Vertex 3
            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 1.0f;
            vCounter++;

            // 2 triangles for this face
            mesh.indices[iCounter++] = faceBase + 0;
            mesh.indices[iCounter++] = faceBase + 1;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 3;
            mesh.indices[iCounter++] = faceBase + 0;
        }

        // --------------------- BACK FACE ------------------------
        // Normal: -Z
        // Texcoords: (1,0), (1,1), (0,1), (0,0)  (matching your DrawCubeTexture)
        {
            int faceBase = vCounter;

            // Vertex 0
            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = -1.0f;
            vCounter++;

            // Vertex 1
            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = -1.0f;
            vCounter++;

            // Vertex 2
            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = -1.0f;
            vCounter++;

            // Vertex 3
            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = -1.0f;
            vCounter++;

            // Indices
            mesh.indices[iCounter++] = faceBase + 0;
            mesh.indices[iCounter++] = faceBase + 1;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 3;
            mesh.indices[iCounter++] = faceBase + 0;
        }

        // --------------------- TOP FACE -------------------------
        // Normal: +Y
        // Texcoords: (0,1), (0,0), (1,0), (1,1)
        {
            int faceBase = vCounter;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = 1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.indices[iCounter++] = faceBase + 0;
            mesh.indices[iCounter++] = faceBase + 1;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 3;
            mesh.indices[iCounter++] = faceBase + 0;
        }

        // --------------------- BOTTOM FACE -----------------------
        // Normal: -Y
        // Texcoords: (1,1), (0,1), (0,0), (1,0)
        {
            int faceBase = vCounter;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = -1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = -1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = -1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 0.0f;
            mesh.normals[(vCounter*3) + 1]   = -1.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.indices[iCounter++] = faceBase + 0;
            mesh.indices[iCounter++] = faceBase + 1;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 3;
            mesh.indices[iCounter++] = faceBase + 0;
        }

        // --------------------- RIGHT FACE ------------------------
        // Normal: +X
        // Texcoords: (1,0), (1,1), (0,1), (0,0)
        {
            int faceBase = vCounter;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = 1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x + half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = 1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.indices[iCounter++] = faceBase + 0;
            mesh.indices[iCounter++] = faceBase + 1;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 3;
            mesh.indices[iCounter++] = faceBase + 0;
        }

        // --------------------- LEFT FACE -------------------------
        // Normal: -X
        // Texcoords: (0,0), (1,0), (1,1), (0,1)
        {
            int faceBase = vCounter;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = -1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y - half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 0.0f;
            mesh.normals[(vCounter*3) + 0]   = -1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z + half;
            mesh.texcoords[(vCounter*2) + 0] = 1.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = -1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.vertices[(vCounter*3) + 0] = x - half;
            mesh.vertices[(vCounter*3) + 1] = y + half;
            mesh.vertices[(vCounter*3) + 2] = z - half;
            mesh.texcoords[(vCounter*2) + 0] = 0.0f;
            mesh.texcoords[(vCounter*2) + 1] = 1.0f;
            mesh.normals[(vCounter*3) + 0]   = -1.0f;
            mesh.normals[(vCounter*3) + 1]   = 0.0f;
            mesh.normals[(vCounter*3) + 2]   = 0.0f;
            vCounter++;

            mesh.indices[iCounter++] = faceBase + 0;
            mesh.indices[iCounter++] = faceBase + 1;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 2;
            mesh.indices[iCounter++] = faceBase + 3;
            mesh.indices[iCounter++] = faceBase + 0;
        }
    } // end for all voxels

    // Now upload CPU arrays to GPU
    UploadMesh(&mesh, false);

    return mesh;
}



