
#include "sea.h"



SEA::SEA() {
    // Create mesh with initial data
    mesh = GenMeshPlane(width * spacing, depth * spacing, width - 1, depth - 1);
    
    // Upload mesh to GPU once
    //UploadMesh(&mesh, true);  // Dynamic = true for frequent updates
    
    // Create model
    model = LoadModelFromMesh(mesh);


    // load shader
    sea_shader = LoadShader("rsc/SEA_SHADER/sea_vertex.glsl", "rsc/SEA_SHADER/sea_flatness.glsl");
    // Get the location of the "seconds" variable in the shader
    secondsLoc = GetShaderLocation(sea_shader, "seconds");
    
    // Assign shader to the model's material
    model.materials[0].shader = sea_shader;
}

SEA::~SEA()
{
    UnloadModel(model);
    UnloadTexture(texture);
    UnloadMesh(mesh);
    UnloadShader(sea_shader);
}



// GameObject methods
void SEA::Update(float dt) {
    static float timer = 0.0f;
    timer += dt;

    SetShaderValue(sea_shader, secondsLoc, &timer, SHADER_UNIFORM_FLOAT);

    if (IsKeyPressed(KEY_R)) {
        UnloadShader(sea_shader); // Clean up the old one
        sea_shader = LoadShader("rsc/SEA_SHADER/sea_vertex.glsl", "rsc/SEA_SHADER/sea_flatness.glsl");
        model.materials[0].shader = sea_shader;
        secondsLoc = GetShaderLocation(sea_shader, "seconds");
        TraceLog(LOG_INFO, "Shaders Reloaded!");
    }
}

void SEA::Draw() {
    
    DrawModel(model, Vector3{0, 0, 0}, 1.0f, WHITE);

}

void SEA::Draw2D() {
   

}






