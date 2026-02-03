#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
in vec3 vertexNormal;

uniform mat4 mvp;
uniform mat4 matrixModel; // Changed from matModel (Raylib fills this automatically)
uniform mat4 matLightVP; 

out vec2 fragTexCoord;
out vec4 fragPosLightSpace;

void main() {
    fragTexCoord = vertexTexCoord;
    
    // Position in world space, then into Light Space
    vec4 worldPos = matrixModel * vec4(vertexPosition, 1.0);
    fragPosLightSpace = matLightVP * worldPos;
    
    gl_Position = mvp * vec4(vertexPosition, 1.0);
}