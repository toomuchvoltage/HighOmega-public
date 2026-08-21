#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define M_PI 3.1415926535

#define isDiElectric (inInstanceFlags.z == 1.0)
#define hasRghMap (inInstanceFlags2.y == 1.0)
#define inEmissivity inInstanceFlags2.z
#define inRefractiveIndex inInstanceFlags2.w
#define isDynamic (voxelizedInfo.mapMaxDynamicFlag.a == 1.0)

layout (set = 1, binding = 0) uniform sampler2DArray diffuseSampler;
layout (set = 1, binding = 1) uniform sampler2DArray normalSampler;
layout (set = 1, binding = 2) uniform sampler2DArray roughnessAndSpecSampler;
layout (set = 1, binding = 3) uniform sampler2DArray heightSampler;
layout (set = 1, binding = 4) uniform sampler2DArray specularSampler;

layout (binding = 0, rgba8) uniform image3D sceneVoxelized;

layout (binding = 1) uniform VoxelizedInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxDynamicFlag;
	vec4 invMapDims;
	vec4 gridSize;
} voxelizedInfo;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inVNorm;
layout (location = 3) in vec3 inTangent;
layout (location = 4) flat in float inFlipFlag;
layout (location = 5) flat in vec4 inInstanceFlags;
layout (location = 6) flat in vec4 inInstanceFlags2;

vec2 toSpherical(vec3 inpVec)
{
	vec2 inpVecOnXZ = ((inpVec.xz == vec2 (0.0)) ? vec2 (0.0) : normalize (inpVec.xz));
	float phi = acos(clamp (inpVecOnXZ.x, -1.0, 1.0));
	if (inpVecOnXZ.y < 0.0) phi = (2.0 * M_PI) - phi;
	float theta = acos(clamp (inpVec.y, -1.0, 1.0));
	return vec2(phi / (2.0*M_PI), theta / M_PI);
}

vec3 getFragmentNormal ()
{
	return normalize (inVNorm);
}

float encodeTanBitan (vec3 inpNorm, vec3 inpTan, vec3 inpBiTan)
{
	vec3 aBasisX = normalize (cross (inpNorm, inpNorm + vec3 (0.1)));
	vec3 aBasisY = cross (inpNorm,aBasisX);
	float shadowFor8Bit = round(((dot (aBasisX, inpTan) + 1.0) * 0.5) * 63.0);
	if ( dot (aBasisY, inpTan) < 0.0 ) shadowFor8Bit += 64.0;
	if ( dot (aBasisY, inpBiTan) < 0.0 ) shadowFor8Bit += 128.0;
	return shadowFor8Bit/255.0;
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

vec4 encodeNormal(vec3 roughnessAndSpec)
{
	vec3 vertNorm = normalize (inVNorm);
	vec2 sphCoord = toSpherical (vertNorm);

	vec3 curTan = normalize (inTangent);
	vec3 curBiTan = cross (vertNorm, curTan);
	if ( inFlipFlag == 1.0 ) curBiTan = -curBiTan;
	mat3 tanSpace;
	
	tanSpace[2] = vertNorm;
	tanSpace[0] = curTan;
	tanSpace[1] = curBiTan;
	tanSpace = reOrthoNormalize (tanSpace);

	return vec4 (sphCoord.x, sphCoord.y, encodeTanBitan (tanSpace[2], tanSpace[0], tanSpace[1]), roughnessAndSpec.z);
}

vec4 encodeEmissivityDielectricRefractiveIndexAndRoughness(vec3 roughnessAndSpec)
{
	float encodedEmissivity = clamp (inEmissivity / 25.5, 0.0, 1.0);
	float refractiveIndexDielectric8Bit = clamp (inRefractiveIndex, 0.0, 4.0) * 31.75;
	if ( isDiElectric ) refractiveIndexDielectric8Bit += 128.0;
	return vec4 (encodedEmissivity, refractiveIndexDielectric8Bit / 255.0, roughnessAndSpec.x, roughnessAndSpec.y);
}

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
		vec2 curTerrainUV = (inPos.xz * 33.33333)/vec2 (textureSize (diffuseSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return retVal;
	}
	return texture (diffuseSampler, vec3 (inUV, 0.0));
}

vec4 sampleSpecular (vec3 inpWeights, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (inPos.xz * 33.33333)/vec2 (textureSize (specularSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (specularSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return retVal;
	}
	return texture (specularSampler, vec3 (inUV, 0.0));
}

vec4 sampleRoughness (vec3 inpWeights, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (inPos.xz * 33.33333)/vec2 (textureSize (roughnessAndSpecSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (roughnessAndSpecSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (roughnessAndSpecSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (roughnessAndSpecSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return retVal;
	}
	return texture (roughnessAndSpecSampler, vec3 (inUV, 0.0));
}

void main()
{
	vec3 boxUVW = (inPos - voxelizedInfo.mapMinVoxelSize.xyz) * voxelizedInfo.invMapDims.xyz;
	if ( boxUVW == clamp (boxUVW, vec3(0.0), vec3(0.999999)) )
	{
		vec3 terrainSampleWeights = vec3 (0.0);
		bool isTerrain = textureSize (diffuseSampler, 0).z > 1;
		if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights ();

		vec3 roughnessAndSpec = vec3 (1.0, 1.0, 0.0);
		if (hasRghMap) roughnessAndSpec = sampleRoughness (terrainSampleWeights, isTerrain).rgb;

		vec4 diffStore = sampleDiffuse (terrainSampleWeights, isTerrain);
		if ( diffStore.a < 1.0 ) discard;
		vec4 normalStore = encodeNormal(roughnessAndSpec);
		vec4 emissivityDielectricRefractiveIndexAndRoughnessStore = encodeEmissivityDielectricRefractiveIndexAndRoughness (roughnessAndSpec);
		vec4 specularStore = sampleSpecular (terrainSampleWeights, isTerrain);

		specularStore.a = 0.0;
		if ( isDynamic ) specularStore.a = 128.0 / 255.0;

		ivec3 uvwLoc = ivec3(boxUVW * voxelizedInfo.gridSize.xyz);

		if ( !isDynamic || (isDynamic && imageLoad(sceneVoxelized, ivec3 (uvwLoc.x*4, uvwLoc.yz)) == vec4 (0.0)) )
		{
			imageStore(sceneVoxelized, ivec3 (uvwLoc.x*4    , uvwLoc.yz), diffStore);
			imageStore(sceneVoxelized, ivec3 (uvwLoc.x*4 + 1, uvwLoc.yz), normalStore);
			imageStore(sceneVoxelized, ivec3 (uvwLoc.x*4 + 2, uvwLoc.yz), emissivityDielectricRefractiveIndexAndRoughnessStore);
			imageStore(sceneVoxelized, ivec3 (uvwLoc.x*4 + 3, uvwLoc.yz), specularStore);
		}
	}
}