#version 430 core
uniform mat4 mvMatrix;
uniform mat4 projMatrix;
in vec4 vPosition;
out vec4 fPosition;

void
main()
{
    fPosition = projMatrix * mvMatrix * vPosition;
    gl_Position = fPosition;
}

