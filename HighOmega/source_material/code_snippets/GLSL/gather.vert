#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define M_PI 3.1415926535

layout (location = 0) in vec4 inPosCol;
layout (location = 1) in vec4 inTanSpaceUV;

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} frameMVP;

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
layout (location = 7) flat out vec4 outPackedTessFlags;
layout (location = 8) out vec3 outGeomNorm;

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

float getDoubleSided (uint packedFlags)
{
	return ((packedFlags & 0x00000008) != 0) ? 1.0 : 0.0;
}

float getDiElectric (uint packedFlags)
{
	return ((packedFlags & 0x00000040) != 0) ? 1.0 : 0.0;
}

void packObjectFlags ()
{
	uint packedFlags = floatBitsToUint (instanceInfo.props[gl_InstanceIndex].attribs.x);
	outInstanceFlags = outInstanceFlags2 = vec4 (0.0);
	outInstanceFlags.x  = getHasNrmMap (packedFlags);
	outInstanceFlags.y  = getDoubleSided (packedFlags);
	outInstanceFlags.z  = getDiElectric (packedFlags);
	outInstanceFlags2.x = getSmooth (packedFlags);
	outInstanceFlags2.y = getHasRghMap (packedFlags);
	outInstanceFlags2.z = instanceInfo.props[gl_InstanceIndex].attribs.z;
	outInstanceFlags2.w = instanceInfo.props[gl_InstanceIndex].attribs.w;
}

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
}

vec2 unpackSpherical(float inpPack)
{
	uint packInt = floatBitsToUint(inpPack);
	vec2 retVal;
	retVal.x = float((packInt & 0xFFFF0000) >> 16) / 65535.0;
	retVal.y = float(packInt & 0x0000FFFF) / 65535.0;
	return retVal;
}

void unpackColorFlipBiTan(float inpPack, out vec3 color, out bool flipFlag)
{
	uint packInt = floatBitsToUint(inpPack);
	color = vec3 (float((packInt & 0xFF000000) >> 24) / 255.0, float((packInt & 0x00FF0000) >> 16) / 255.0, float((packInt & 0x0000FF00) >>  8) / 255.0);
	flipFlag = ((packInt & 0x00000001) == 0x00000001) ? true : false;
}

void main() 
{
	packObjectFlags ();
	outUV = inTanSpaceUV.zw;
	outPackedTessFlags = instanceInfo.props[gl_InstanceIndex].attribs2;
	outPos = inPosCol.xyz;
	vec3 vColor;
	bool vFlipFlag;
	unpackColorFlipBiTan(inPosCol.w, vColor, vFlipFlag);
	gl_Position = frameMVP.projectionViewMatrix * vec4(outPos, 1.0);
	outGeomNorm = outVNorm = toVec3 (unpackSpherical(inTanSpaceUV.x));
	outTangent = toVec3 (unpackSpherical(inTanSpaceUV.y));
	outFlipFlag = vFlipFlag ? 1.0 : 0.0;
}
