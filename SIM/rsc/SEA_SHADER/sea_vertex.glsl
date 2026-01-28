#version 330

in vec3 vertexPosition;
in vec2 vertexTexCoord;

uniform mat4 mvp;
uniform float seconds;

out float height;
out vec2 fragTexCoord;

// Helper function to create a wave layer
float calculateWave(vec3 pos, float freq, float speed, float amp) {
    // Adding X and Z together with different offsets prevents "grid" look
    return sin(pos.x * freq + seconds * speed) * cos(pos.z * freq * 0.8 + seconds * speed) * amp;
}

void main() {
    fragTexCoord = vertexTexCoord;
    vec3 pos = vertexPosition;

    // Layer 1: Large, slow swells
    float w1 = calculateWave(pos, 0.02, 1.0, 0.8);
    
    // Layer 2: Medium, faster ripples
    float w2 = calculateWave(pos, 0.05, 1.8, 0.3);
    
    // Layer 3: Tiny, high-frequency "noise"
    float w3 = calculateWave(pos, 0.12, 1.5, 0.1);

    float totalWave = w1 + w2 + w3;
    
    height = totalWave; // Pass this to the fragment shader for shading
    pos.y += totalWave;

    gl_Position = mvp * vec4(pos, 1.0);
}