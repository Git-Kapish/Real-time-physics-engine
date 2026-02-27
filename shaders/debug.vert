#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
uniform vec3 uColor;
out vec3 vColor;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = uColor;
}
