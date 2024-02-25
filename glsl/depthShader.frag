#version 430 core
in vec4 fPosition;
out vec4 fragmentdepth;

void
main()
{
    vec3 fragCoord = (fPosition.xyz / fPosition.w);
    fragmentdepth.xyz = 0.5 * fragCoord.zzz + 0.5;
    fragmentdepth.w = 1.0;
}
