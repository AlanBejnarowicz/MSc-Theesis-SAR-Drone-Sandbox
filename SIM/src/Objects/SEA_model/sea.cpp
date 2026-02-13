
#include "sea.h"



SEA::SEA() {
    timer = 0.0f;
}

SEA::~SEA()
{
    UnloadModel(model);
    UnloadTexture(texture);
    UnloadMesh(mesh);
    UnloadShader(sea_shader);
}






// GameObject methods

void SEA::Start(){

    // Create mesh with initial data
    mesh = GenMeshPlane(SEA_vertices_width * spacing, SEA_vertices_depth * spacing, SEA_vertices_width - 1, SEA_vertices_depth - 1);


    model = LoadModelFromMesh(mesh);

    //move model center to (0,0,0)

    // load shader
    //sea_shader = LoadShader("rsc/SEA_SHADER/sea_vertex.glsl", "rsc/SEA_SHADER/sea_flatness.glsl");
    sea_shader = LoadShader("rsc/SEA_SHADER/sea.vs", "rsc/SEA_SHADER/sea.fs");
    // Get the location of the "seconds" variable in the shader
    secondsLoc = GetShaderLocation(sea_shader, "seconds");
    
    // Assign shader to the model's material
    model.materials[0].shader = sea_shader;


    std::cout << " SEA object name:  " << GameObjectName << "       ID: " << GameObjectID  << "\r\n";


}  






void SEA::Update(float dt) {
    timer += dt;

    // pass time to shader
    SetShaderValue(sea_shader, secondsLoc, &timer, SHADER_UNIFORM_FLOAT);

    if (IsKeyPressed(KEY_R)) {
        UnloadShader(sea_shader); // Clean up the old one
        sea_shader = LoadShader("rsc/SEA_SHADER/sea.vs", "rsc/SEA_SHADER/sea.fs");
        model.materials[0].shader = sea_shader;
        secondsLoc = GetShaderLocation(sea_shader, "seconds");
        TraceLog(LOG_INFO, "Shaders Reloaded!");
    }




    update_wave ();
    UpdateMeshBuffer(mesh, 0, mesh.vertices, mesh.vertexCount * 3 * sizeof(float), 0);


}

void SEA::Draw() {
    
    DrawModel(model, Vector3{0, 0, 0}, 1.0f, WHITE);


    // // draw debug vector for wave
    // Tools::Vector3 norm_pos(0,0,0);
    // Tools::Vector3 norm_vect = get_wave_normal(norm_pos);

    // Vector3 start = { norm_pos.x, norm_pos.y, norm_pos.z };
    // Vector3 end = { 
    //     norm_pos.x + norm_vect.x * 20.0f, // Scale by 2.0 to make it visible
    //     norm_pos.y + norm_vect.y * 20.0f, 
    //     norm_pos.z + norm_vect.z * 20.0f 
    // };

    // DrawLine3D(start, end, RED);
    // DrawSphere(start, 0.1f, RED); // Small dot at the base of the normal



}

void SEA::Draw2D() {
   

}




float SEA::calculateWave(Tools::Vector3 pos, float freq, float speed, float amp) {
    // Adding X and Z together with different offsets prevents "grid" look
    return sin(pos.x * freq + timer * speed) * cos(pos.z * freq * 0.8 + timer * speed) * amp;
}





float SEA::calculate_total_wave_height(Tools::Vector3 pos) {
    // Large Swells (Low frequency, high amplitude)
    float w1 = calculateWave(pos, 0.02, 1.2, 0.8 * wave_amplitude_scale); 
    float w2 = calculateWave(pos, 0.04, 1.5, 0.5 * wave_amplitude_scale);

    // Mid-level Turbulence (Medium frequency)
    float w3 = calculateWave(pos, 0.10, 2.2, 0.2 * wave_amplitude_scale);
    float w4 = calculateWave(pos, 0.18, 1.8, 0.15 * wave_amplitude_scale);

    // Surface Micro-Ripples (High frequency, low amplitude)
    // These give the water that "shimmering" look
    float w5 = calculateWave(pos, 0.45, 3.0, 0.05 * wave_amplitude_scale);
    float w6 = calculateWave(pos, 0.80, 4.2, 0.02 * wave_amplitude_scale);

    // Summing with a slight decay or scaling can help
    return w1 + w2 + w3 + w4 + w5 + w6;
}






void SEA::update_wave(void){

    for (int i = 0; i < SEA_vertices_width * SEA_vertices_depth; i++) {
        float x = mesh.vertices[i * 3];
        float z = mesh.vertices[i * 3 + 2];

        Tools::Vector3 pos(x,0,z);

        float totalWave = calculate_total_wave_height(pos);
        
        // Example wave math: y = sin(x + time)
        mesh.vertices[i * 3 + 1] = totalWave;
    }

}


Tools::Vector3 SEA::get_wave_normal(Tools::Vector3 pos){

    float epsilon = 0.1f; 

    // 1. Get the height at the current point and two neighbor points
    float h = calculate_total_wave_height(pos);
    
    // Sample slightly offset on X axis
    Tools::Vector3 posStepX = { pos.x + epsilon, 0, pos.z };
    float hX = calculate_total_wave_height(posStepX);
    
    // Sample slightly offset on Z axis
    Tools::Vector3 posStepZ = { pos.x, 0, pos.z + epsilon };
    float hZ = calculate_total_wave_height(posStepZ);

    // 2. Construct two tangent vectors
    // Tangent along X: (epsilon, change_in_height, 0)
    Tools::Vector3 tangentX = { epsilon, hX - h, 0.0f };
    // Tangent along Z: (0, change_in_height, epsilon)
    Tools::Vector3 tangentZ = { 0.0f, hZ - h, epsilon };

    // 3. The Normal is the Cross Product of these two tangents
    Tools::Vector3 normal = tangentZ.cross(tangentX); 
    
    return normal.normalize();
}



