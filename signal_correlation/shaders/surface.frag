#version 330 core
in vec3 surfaceVtxColor;
out vec4 color;

void main()
{    
    color = vec4(surfaceVtxColor, 1.0);
}

