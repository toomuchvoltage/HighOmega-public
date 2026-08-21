#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define M_PI 3.1415926535

layout (location = 0) in vec4 inPosCol;
layout (location = 1) in vec4 inUVNorm;

struct InstanceProps
{
	vec4 attribs;
	vec4 attribs2;
};

layout (set = 1, binding = 5) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

layout (location = 0) out vec2 outUV;
layout (location = 1) out vec3 outPos;
layout (location = 2) out vec3 outVNorm;
layout (location = 3) out vec3 outTangent;
layout (location = 4) flat out float outFlipFlag;
layout (location = 5) flat out vec4 outInstanceFlags;
layout (location = 6) flat out vec4 outInstanceFlags2;

out gl_PerVertex 
{
	vec4 gl_Position;
};

float getHasNrmMap (uint packedFlags)
{
	return ((packedFlags & 0x00000001) != 0) ? 1.0 : 0.0;
}

float getHasRghMap (uint packedFlags)
{
	return ((packedFlags & 0x00000002) != 0) ? 1.0 : 0.0;
}

float getSmooth (uint packedFlags)
{
	return ((packedFlags & 0x00000004) != 0) ? 1.0 : 0.0;
}

float getDiElectric (uint packedFlags)
{
	return ((packedFlags & 0x00000040) != 0) ? 1.0 : 0.0;
}

void packObjectFlags (float particleStrength)
{
	uint packedFlags = floatBitsToUint (instanceInfo.props[gl_InstanceIndex].attribs.x);
	outInstanceFlags = outInstanceFlags2 = vec4 (0.0);
	outInstanceFlags.x  = getHasNrmMap (packedFlags);
	outInstanceFlags.z  = getDiElectric (packedFlags);
	outInstanceFlags2.x = getSmooth (packedFlags);
	outInstanceFlags2.y = getHasRghMap (packedFlags);
	outInstanceFlags2.z = instanceInfo.props[gl_InstanceIndex].attribs.z;
	if ( particleStrength > 0.0 ) outInstanceFlags2.z *= particleStrength; // particle strength...
	outInstanceFlags2.w = instanceInfo.props[gl_InstanceIndex].attribs.w;
}

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
}

vec3 unpackColor(float inpPack)
{
	uint packInt = floatBitsToUint (inpPack);
	vec3 color;
	color.x = float((packInt & 0xFF000000) >> 24) / 255.0;
	color.y = float((packInt & 0x00FF0000) >> 16) / 255.0;
	color.z = float((packInt & 0x0000FF00) >> 8) / 255.0;
	return color;
}

void unpackRasterVertex(out vec3 vPos, out vec3 vCol, out vec2 vUV, out vec3 vNorm, in vec4 rv1, in vec4 rv2)
{
	vPos = rv1.xyz;
	vCol = unpackColor(rv1.w);

	vUV = rv2.xy;
	vNorm = toVec3(rv2.zw);
}

void main() 
{
	vec3 vColor;
	unpackRasterVertex (outPos, vColor, outUV, outVNorm, inPosCol, inUVNorm);
	packObjectFlags (vColor.x);
	outTangent = normalize (cross (outVNorm, outVNorm + vec3 (0.1)));
	outFlipFlag = 0.0;

    uint vertIndexMod = gl_VertexIndex % 3;
    if ( vertIndexMod == 0 )
        gl_Position = vec4 (-1.0, 1.0, 0.0, 1.0);
    else if ( vertIndexMod == 1 )
        gl_Position = vec4 (1.0, 1.0, 0.0, 1.0);
    else
        gl_Position = vec4 (-1.0, -1.0, 0.0, 1.0);
}