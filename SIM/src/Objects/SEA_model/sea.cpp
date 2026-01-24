
#include "sea.h"



SEA::SEA() {
    // Create mesh with initial data
    mesh = GenMeshPlane(width * spacing, depth * spacing, width - 1, depth - 1);
    
    // Upload mesh to GPU once
    UploadMesh(&mesh, true);  // Dynamic = true for frequent updates
    
    // Create model
    model = LoadModelFromMesh(mesh);
    
    // Simple water material
    model.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = 
        Color{30, 144, 255, 180};  // Dodger blue with transparency
}

SEA::~SEA()
{
    UnloadModel(model);
    UnloadTexture(texture);
    UnloadMesh(mesh);
}

// GameObject methods
void SEA::Update(float dt) {

}

void SEA::Draw() {
    
    DrawModel(model, Vector3{0, 0, 0}, 1.0f, WHITE);

}

void SEA::Draw2D() {
   

}






