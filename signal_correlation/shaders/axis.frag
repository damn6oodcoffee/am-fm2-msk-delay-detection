#version 330 core
in vec3 axisColor;
out vec4 color;

void main()
{    
    color = vec4(axisColor, 1.0);
}

