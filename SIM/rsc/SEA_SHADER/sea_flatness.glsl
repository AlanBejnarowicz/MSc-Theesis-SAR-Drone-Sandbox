#version 330

in float height; 
in vec2 fragTexCoord;
out vec4 finalColor;

void main() {
    // Deep water is dark navy, shallow is turquoise
    vec3 deepBlue = vec3(0.02, 0.04, 0.15);
    vec3 shallowBlue = vec3(0.1, 0.5, 0.7);
    vec3 crestColor = vec3(0.3, 0.4, 1.0); // Bright highlight for peaks

    // Normalize height (assuming total wave is around -1.2 to 1.2)
    float normHeight = (height + 1.2) / 2.4;
    normHeight = clamp(normHeight, 0.0, 1.0);

    // Color mixing
    vec3 color = mix(deepBlue, shallowBlue, normHeight);
    
    // Add "Foam" or "Crest" highlights on the very top of waves
    if (normHeight > 0.8) {
        float foamFactor = (normHeight - 0.8) * 5.0;
        color = mix(color, crestColor, foamFactor);
    }

    // Add a slight "glint" based on the texture coordinates
    float glint = sin(fragTexCoord.x * 100.0 + fragTexCoord.y * 100.0) * 0.05;
    color += glint * (normHeight);

    finalColor = vec4(color, 0.85); 
}