#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define hasNrmMap (inInstanceFlags.x == 1.0)
#define doubleSided (inInstanceFlags.y == 1.0)
#define isDiElectric (inInstanceFlags.z == 1.0)
#define hasRghMap (inInstanceFlags2.y == 1.0)
#define inEmissivity inInstanceFlags2.z
#define inRefractiveIndex inInstanceFlags2.w

#define M_PI 3.1415926535

layout (set = 1, binding = 0) uniform sampler2DArray diffuseSampler;
layout (set = 1, binding = 1) uniform sampler2DArray normalSampler;
layout (set = 1, binding = 2) uniform sampler2DArray roughnessSampler;
layout (set = 1, binding = 4) uniform sampler2DArray specularSampler;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPosW;
layout (location = 2) in vec3 inVNorm;
layout (location = 3) in vec3 inTangent;
layout (location = 4) flat in float inFlipFlag;
layout (location = 5) flat in vec4 inInstanceFlags;
layout (location = 6) flat in vec4 inInstanceFlags2;
layout (location = 8) in vec3 inGeomNorm;

layout (location = 0) out vec4 materialOutput;
layout (location = 1) out vec4 worldPosOutput;
layout (location = 2) out vec4 normalOutput;

vec3 sampleTerrainWeights ()
{
	vec3 terrainMap = texture (diffuseSampler, vec3 (inUV, 3.0)).rgb;
	terrainMap /= (terrainMap.x + terrainMap.y + terrainMap.z);
	return terrainMap;
}

vec4 sampleDiffuse (vec3 inpWeights, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (inPosW.xz * 33.33333)/vec2 (textureSize (diffuseSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return vec4 (retVal.xyz, 1.0);
	}
	return texture (diffuseSampler, vec3 (inUV, 0.0));
}

vec4 sampleSpecular (vec3 inpWeights, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (inPosW.xz * 33.33333)/vec2 (textureSize (diffuseSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (specularSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return vec4 (retVal.xyz, 1.0);
	}
	return texture (specularSampler, vec3 (inUV, 0.0));
}

vec3 sampleNormal (vec3 inpWeights, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (inPosW.xz * 33.33333)/vec2 (textureSize (diffuseSampler, 0).xy);
		vec3 retVal = vec3 (0.0);
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 2.0)).rgb - vec3 (1.0)) * inpWeights.r;
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 1.0)).rgb - vec3 (1.0)) * inpWeights.g;
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 0.0)).rgb - vec3 (1.0)) * inpWeights.b;
		return normalize (retVal);
	}
	return 2.0 * texture (normalSampler, vec3 (inUV, 0.0)).rgb - vec3 (1.0);
}

vec4 sampleRoughness (vec3 inpWeights, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (inPosW.xz * 33.33333)/vec2 (textureSize (diffuseSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (roughnessSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (roughnessSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (roughnessSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return vec4 (retVal.xyz, 1.0);
	}
	return texture (roughnessSampler, vec3 (inUV, 0.0));
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

vec3 getFragmentNormal (vec3 inpWeights, bool isTerrain, inout mat3 tanSpace)
{
	vec3 vertNorm = normalize (inVNorm);
	vec3 curTan = normalize (inTangent);
	vec3 curBiTan = cross (vertNorm, curTan);
	if ( inFlipFlag == 1.0 ) curBiTan = -curBiTan;
	
	tanSpace[2] = vertNorm;
	tanSpace[0] = curTan;
	tanSpace[1] = curBiTan;
	tanSpace = reOrthoNormalize (tanSpace);

	if ( hasNrmMap )
		return normalize (tanSpace * sampleNormal (inpWeights, isTerrain));
	else
		return tanSpace[2];
}

float packEmissivityRefractiveIndexDielectric ()
{
	float retVal = round (inRefractiveIndex * 1000.0) + fract (inEmissivity / 255.0);
	if ( isDiElectric ) retVal = -retVal;
	return retVal;
}

vec2 toSpherical(vec3 inpVec)
{
	vec2 inpVecOnXZ = ((inpVec.xz == vec2 (0.0)) ? vec2 (0.0) : normalize (inpVec.xz));
	float phi = acos(clamp (inpVecOnXZ.x, -1.0, 1.0));
	if (inpVecOnXZ.y < 0.0) phi = (2.0 * M_PI) - phi;
	float theta = acos(clamp (inpVec.y, -1.0, 1.0));
	return vec2(phi / (2.0*M_PI), theta / M_PI);
}

float packSpherical(vec2 inpSph)
{
	uint retVal = ((uint(inpSph.x * 65535.0) & 0x0000FFFF) << 16) | (uint(inpSph.y * 65535.0) & 0x0000FFFF);
	return uintBitsToFloat(retVal);
}

void main()
{
	vec3 terrainSampleWeights = vec3 (0.0);
	bool isTerrain = textureSize (diffuseSampler, 0).z > 1;
	if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights ();

	vec4 diffFetch = sampleDiffuse (terrainSampleWeights, isTerrain);
	if ( diffFetch.a < 0.9 ) discard;
	vec4 specFetch = sampleSpecular (terrainSampleWeights, isTerrain);
	vec4 roughFetch = vec4 (1.0, 1.0, 0.0, 0.0);
	if ( hasRghMap ) roughFetch = sampleRoughness (terrainSampleWeights, isTerrain);
	float MaterialHint = max (texture (diffuseSampler, vec3 (0.1, 0.1, 0.0), 0.0).r + texture (diffuseSampler, vec3 (0.5, 0.5, 0.0), 0.0).g + texture (diffuseSampler, vec3 (0.9, 0.9, 0.0), 0.0).b, 0.0001);
	if ( doubleSided ) MaterialHint = -MaterialHint;

	materialOutput = vec4 (uintBitsToFloat (packUnorm4x8 (diffFetch)), uintBitsToFloat (packUnorm4x8 (specFetch)), uintBitsToFloat (packUnorm4x8 (roughFetch)), packEmissivityRefractiveIndexDielectric ());
	worldPosOutput = vec4 (inPosW, MaterialHint);
	mat3 tanSpace;
	vec3 fragNorm = getFragmentNormal(terrainSampleWeights, isTerrain, tanSpace);
	normalOutput = vec4 (packSpherical (toSpherical (fragNorm)),
						 packSpherical (toSpherical (tanSpace[2])),
						 packSpherical (toSpherical (tanSpace[0])),
						 packSpherical (toSpherical (tanSpace[1])));
}