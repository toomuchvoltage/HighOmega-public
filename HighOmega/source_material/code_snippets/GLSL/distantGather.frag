#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define M_PI 3.1415926535

#define hasNrmMap (inInstanceFlags.x == 1.0)

layout (set = 1, binding = 0) uniform sampler2DArray diffuseSampler;
layout (set = 1, binding = 1) uniform sampler2DArray normalSampler;
layout (set = 1, binding = 2) uniform sampler2DArray roughnessSampler;
layout (set = 1, binding = 4) uniform sampler2DArray specularSampler;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inVNorm;
layout (location = 3) in vec3 inTangent;
layout (location = 4) flat in float inFlipFlag;
layout (location = 5) flat in vec4 inInstanceFlags;
layout (location = 6) flat in vec4 inInstanceFlags2;

layout (location = 0) out vec4 worldPosAlbedoOutput;
layout (location = 1) out vec4 normalOutput;

vec4 sampleDiffuse ()
{
	return texture (diffuseSampler, vec3 (inUV.xy, 0.0));
}

vec3 sampleNormal ()
{
	return 2.0 * texture (normalSampler, vec3 (inUV.xy, 0.0)).rgb - vec3 (1.0);
}

mat3 reOrthoNormalize (mat3 inpBasis)
{
	mat3 outMat;
	
	outMat[2] = normalize (inpBasis[2]);
	
	vec3 newBiTan = normalize (cross (outMat[2], inpBasis[0]));
	outMat[1] = newBiTan * sign(dot (inpBasis[1], newBiTan));

	vec3 newTan = cross (outMat[2], outMat[1]);
	outMat[0] = newTan * sign(dot (inpBasis[0], newTan));
	
	return outMat;
}

vec3 getFragmentNormal ()
{
	vec3 vertNorm = normalize (inVNorm);
	vec3 curTan = normalize (inTangent);
	vec3 curBiTan = cross (vertNorm, curTan);
	if ( inFlipFlag == 1.0 ) curBiTan = -curBiTan;
	mat3 tanSpace;

	tanSpace[2] = vertNorm;
	tanSpace[0] = curTan;
	tanSpace[1] = curBiTan;
	tanSpace = reOrthoNormalize (tanSpace);

	if (hasNrmMap)
		return normalize (tanSpace * sampleNormal ());
	else
		return tanSpace[2];
}

vec2 toSpherical(vec3 inpVec)
{
	vec2 inpVecOnXZ = ((inpVec.xz == vec2 (0.0)) ? vec2 (0.0) : normalize (inpVec.xz));
	float phi = acos(clamp (inpVecOnXZ.x, -1.0, 1.0));
	if (inpVecOnXZ.y < 0.0) phi = (2.0 * M_PI) - phi;
	float theta = acos(clamp (inpVec.y, -1.0, 1.0));
	return vec2(phi / (2.0*M_PI), theta / M_PI);
}

void main()
{
	worldPosAlbedoOutput = vec4 (inPos.xyz, uintBitsToFloat (packUnorm4x8 (sampleDiffuse ())));
	normalOutput = vec4 (toSpherical(getFragmentNormal ()), 0.0, 1.0);
}