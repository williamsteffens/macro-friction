#version 430 core
layout(binding=1) uniform sampler2D mapNormal;
uniform vec4 texIncrement;
in vec3 fNormal;
in vec3 fTangent;
in vec3 fBinormal;
in vec2 fUV;
out vec4 fColor;

#define TC_XMINUS_YMINUS(coord) (coord - texIncrement.xy)
#define TC_XPLUS_YPLUS(coord) (coord + texIncrement.xy)
#define TC_XCENTER_YMINUS(coord) (coord - texIncrement.zy)
#define TC_XCENTER_YPLUS(coord) (coord + texIncrement.zy)
#define TC_XMINUS_YCENTER(coord) (coord - texIncrement.xz)
#define TC_XPLUS_YCENTER(coord) (coord + texIncrement.xz)
#define TC_XMINUS_YPLUS(coord) (coord - texIncrement.xw)
#define TC_XPLUS_YMINUS(coord) (coord + texIncrement.xw)

void
main()
{
    vec3 rgb_avg = (texture(mapNormal, TC_XMINUS_YMINUS(fUV)).rgb +
                   texture(mapNormal, TC_XPLUS_YPLUS(fUV)).rgb +
                   texture(mapNormal, TC_XCENTER_YMINUS(fUV)).rgb +
                   texture(mapNormal, TC_XCENTER_YPLUS(fUV)).rgb +
                   texture(mapNormal, fUV).rgb +
                   texture(mapNormal, TC_XMINUS_YCENTER(fUV)).rgb +
                   texture(mapNormal, TC_XPLUS_YCENTER(fUV)).rgb +
                   texture(mapNormal, TC_XMINUS_YPLUS(fUV)).rgb +
                   texture(mapNormal, TC_XPLUS_YMINUS(fUV)).rgb ) / 9.0f;
    vec3 normal_avg = normalize(2.0f * rgb_avg - 1.0f);
    vec3 normal = normalize(2.0f * texture(mapNormal, fUV).rgb - 1.0f);
    float w = 1.0f - abs(dot(normal_avg, normal));
    fColor = vec4(w, w, w, 1.0f);
}
