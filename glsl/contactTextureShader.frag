#version 430 core
layout(binding=1) uniform sampler2D mapNormal;
uniform vec3 ncontact;
uniform vec3 d0;
uniform vec3 d1;
uniform float sigma;
uniform vec4 texIncrement;
in vec3 fNormal;
in vec3 fTangent;
in vec3 fBinormal;
in vec2 fUV;
out vec4 fColor;

void
main()
{
    vec3 nfNormal = normalize(fNormal);
    mat3 tbn = mat3(normalize(fTangent), normalize(fBinormal), nfNormal);
    vec3 normalFromTexture = texture(mapNormal, fUV).rgb * 2.0f - vec3(1.0f);
    vec3 mfNormal = normalize(tbn * normalFromTexture);

    // Compute contact normal direction
    vec3 contactNormal = sigma*ncontact;
    // filter if normal doesn't align with contact normal (viewing direction)
    if ( abs( dot(ncontact, nfNormal)) < 0.995 ) {
           fColor = vec4(0,0,0,0.01);
    } else {
        // Tangent direction of the microfacet
        vec3 dir = normalize(mfNormal - dot(mfNormal, contactNormal) * contactNormal);

        float dp = max(-1.0f, min(1.0f, dot(contactNormal, mfNormal) ));
        float ang = acos(dp);
        float tang = tan( ang );
        vec3 d2 = -d0;
        vec3 d3 = -d1;
        float mu0 = max(0.0f, dot(dir,d0)) * tang;
        float mu1 = max(0.0f, dot(dir,d1)) * tang;
        float mu2 = max(0.0f, dot(dir,d2)) * tang;
        float mu3 = max(0.0f, dot(dir,d3)) * tang;
        fColor.r = mu0;
        fColor.g = mu1;
        fColor.b = mu2;
        fColor.a = mu3;
    }
}
