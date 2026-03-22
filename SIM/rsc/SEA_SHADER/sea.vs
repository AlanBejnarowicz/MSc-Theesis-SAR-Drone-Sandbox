#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;

uniform mat4 mvp;
uniform mat4 matModel; // We need this to get world position
uniform float seconds;

out float vHeight;

// Match your C++ math exactly
float calculateWave(vec2 pos, float freq, float speed, float amp) {
    return sin(pos.x * freq + seconds * speed) * cos(pos.y * freq * 0.8 + seconds * speed) * amp;
}



void main() {
    // 1. Calculate World Position
    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);
    
    // 2. Calculate Height using World X and Z
    float w1 = calculateWave(worldPos.xz, 0.02, 1.2, 0.8);
    float w2 = calculateWave(worldPos.xz, 0.04, 1.5, 0.5);
    float w3 = calculateWave(worldPos.xz, 0.10, 2.2, 0.2);
    float w4 = calculateWave(worldPos.xz, 0.18, 1.8, 0.15);
    float w5 = calculateWave(worldPos.xz, 0.45, 3.0, 0.05);
    float w6 = calculateWave(worldPos.xz, 0.80, 4.2, 0.02);
    
    float totalHeight = w1 + w2 + w3 + w4 + w5 + w6;

    // 3. Displace the vertex
    vec3 displacedPos = vertexPosition;
    displacedPos.y += totalHeight;

    vHeight = totalHeight; // Pass to fragment shader for shading/coloring
    gl_Position = mvp * vec4(displacedPos, 1.0);
}