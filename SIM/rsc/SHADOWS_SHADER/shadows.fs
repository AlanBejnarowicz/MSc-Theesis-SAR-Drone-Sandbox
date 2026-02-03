#version 330
in vec2 fragTexCoord;
in vec4 fragPosLightSpace;
out vec4 finalColor;

uniform sampler2D shadowMap;
uniform vec4 colDiffuse;

float CalculateShadow(vec4 shadowPos) {
    // Perform perspective division
    vec3 projCoords = shadowPos.xyz / shadowPos.w;
    projCoords = projCoords * 0.5 + 0.5; // Transform to [0,1] range
    
    if (projCoords.z > 1.0) return 0.0;
    
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;
    
    // Simple bias to prevent shadow acne
    float bias = 0.005;
    return currentDepth - bias > closestDepth ? 0.5 : 0.0;
}

void main() {
    float shadow = CalculateShadow(fragPosLightSpace);
    vec3 color = colDiffuse.rgb * (1.0 - shadow);
    finalColor = vec4(color, 1.0);
}