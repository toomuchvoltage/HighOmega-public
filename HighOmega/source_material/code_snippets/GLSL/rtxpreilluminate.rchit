#version 460
#extension GL_NV_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require

#define VOXEL_DIM_SIZE gridInfo.center.w
#define M_PI 3.1415926535

#define diffuseSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5)]
#define normalSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 1)]
#define roughnessSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 2)]
#define specularSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 4)]
#define curInstInfo instanceBuffers[nonuniformEXT(gl_InstanceCustomIndexNV)]
#define curTriBuf scene[nonuniformEXT(gl_InstanceCustomIndexNV)]

layout(location = 0) rayPayloadInNV struct {
	vec3 radiance;
} hitValues;
hitAttributeNV vec3 hitBaryCoord;

layout (binding = 0, set = 0) uniform accelerationStructureNV topLevelAS;

layout (binding = 5, rgba16f) uniform image3D preIlluminateImage;

layout (binding = 6) uniform GridInfoUBO
{
	vec4 gridMin;
	vec4 gridMax;
	vec4 center;
	vec4 mapMin;
	vec4 mapMax;
} gridInfo;

layout(set = 1, binding = 0) buffer InstanceInfo
{
	mat4 trans;
	mat4 transIT;
	vec4 attribs;
	vec4 attribs2;
} instanceBuffers[];

struct RTXTriangle
{
	vec4 e1u1;
	vec4 e2v1;
	vec4 e3u2;
	vec4 normv2;
	vec4 tanu3;
	vec4 bitanv3;
};

layout(set = 2, binding = 0) buffer CompleteTriBuffer
{
	RTXTriangle tri[];
} scene[];

layout(set = 3, binding = 0) uniform sampler2DArray textures[];

vec3 getUVW (vec3 worldPos)
{
	vec3 boxMax = gridInfo.center.xyz + gridInfo.gridMax.xyz;
	vec3 boxMin = gridInfo.center.xyz + gridInfo.gridMin.xyz;
	vec3 boxLen = boxMax - boxMin;
    vec3 boxUVW = (worldPos - boxMin)/boxLen;
	if ( boxUVW == clamp (boxUVW, vec3(0.0), vec3(1.0)) )
	{
		return boxUVW;
	}
	return vec3 (2.0);
}

float schlicks (float n1, float n2, vec3 norm, vec3 light)
{
	float R0 = (n1 - n2)/(n1 + n2);
	R0 *= R0;
	float _1_minus_dot = 1.0 - min(abs (dot (norm, light)), 1.0);
	_1_minus_dot = _1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot;
	return R0 + (1 - R0) * _1_minus_dot;
}

ivec3 getImageLoc (vec3 uvwLoc)
{
	vec3 boxMax = gridInfo.center.xyz + gridInfo.gridMax.xyz;
	vec3 boxMin = gridInfo.center.xyz + gridInfo.gridMin.xyz;
	vec3 boxLen = boxMax - boxMin;
	return ivec3 (round((uvwLoc * boxLen) / VOXEL_DIM_SIZE));
}

float getHasNrmMap ()
{
	return floor (curInstInfo.attribs.x);
}

bool getIsTerrain ()
{
	return textureSize (diffuseSampler, 0).z > 1;
}

vec3 sampleTerrainWeights (vec2 inUV)
{
	vec3 terrainMap = texture (diffuseSampler, vec3 (inUV, 3.0)).rgb;
	terrainMap /= (terrainMap.x + terrainMap.y + terrainMap.z);
	return terrainMap;
}

vec4 sampleDiffuse (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (posAtSample.xz * 33.33333)/vec2 (textureSize (diffuseSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return retVal;
	}
	return texture (diffuseSampler, vec3 (inUV, 0.0));
}

vec4 sampleSpecular (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (posAtSample.xz * 33.33333)/vec2 (textureSize (specularSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (specularSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return retVal;
	}
	return texture (specularSampler, vec3 (inUV, 0.0));
}

vec3 sampleNormal (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (posAtSample.xz * 33.33333)/vec2 (textureSize (normalSampler, 0).xy);
		vec3 retVal = vec3 (0.0);
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 2.0)).rgb - vec3 (1.0)) * inpWeights.r;
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 1.0)).rgb - vec3 (1.0)) * inpWeights.g;
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 0.0)).rgb - vec3 (1.0)) * inpWeights.b;
		return normalize (retVal);
	}
	return 2.0 * texture (normalSampler, vec3 (inUV, 0.0)).rgb - vec3 (1.0);
}

vec4 sampleRoughness (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (posAtSample.xz * 33.33333)/vec2 (textureSize (roughnessSampler, 0).xy);
		vec4 retVal = vec4 (0.0);
		retVal += texture (roughnessSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (roughnessSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (roughnessSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return retVal;
	}
	return texture (roughnessSampler, vec3 (inUV, 0.0));
}

vec3 getFragmentNormal (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, mat3 tanSpace)
{
	if (getHasNrmMap() > 0.0)
		return normalize (tanSpace * sampleNormal (inpWeights, inUV, posAtSample, isTerrain));
	else
		return normalize (tanSpace[2]);
}

struct HitData
{
	vec3 pos;
	vec3 norm;
	vec3 faceNorm;
	vec3 tangent;
	vec3 bitan;
	
	vec3 albedoColor;
	vec2 anisoRoughness;
	float specAmount;
	vec3 specularColor;
	
	bool diElectric;
	float IoR;
	float emissivity;
};

HitData getHitData ()
{
	HitData retVal;

	mat4 currentTrans = curInstInfo.trans;
	mat4 currentTransIT = curInstInfo.transIT;
	RTXTriangle hitTri = curTriBuf.tri[gl_PrimitiveID];
	vec3 barycoords = vec3(1.0 - hitBaryCoord.x - hitBaryCoord.y, hitBaryCoord.x, hitBaryCoord.y);

	retVal.pos = (currentTrans * vec4 (hitTri.e1u1.xyz * barycoords.x + hitTri.e2v1.xyz * barycoords.y + hitTri.e3u2.xyz * barycoords.z, 1.0)).xyz;
	retVal.faceNorm = normalize (currentTransIT * vec4 (hitTri.normv2.xyz, 1.0)).xyz;
	retVal.tangent = normalize (currentTransIT * vec4 (hitTri.tanu3.xyz, 1.0)).xyz;
	retVal.bitan = normalize (currentTransIT * vec4 (hitTri.bitanv3.xyz, 1.0)).xyz;
	vec2 hitUV = vec2 (hitTri.e1u1.w, hitTri.e2v1.w) * barycoords.x + vec2 (hitTri.e3u2.w, hitTri.normv2.w) * barycoords.y + vec2 (hitTri.tanu3.w, hitTri.bitanv3.w) * barycoords.z;
	bool isTerrain = getIsTerrain ();
	
	vec3 terrainSampleWeights = vec3 (0.0);
	if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights (hitUV);
	
	mat3 tanSpace;
	tanSpace[2] = retVal.faceNorm;
	tanSpace[0] = retVal.tangent;
	tanSpace[1] = retVal.bitan;
	retVal.norm = getFragmentNormal (terrainSampleWeights, hitUV, retVal.pos, isTerrain, tanSpace);
	
	retVal.albedoColor = sampleDiffuse (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	vec3 roughnessAndSpecFetch = vec3 (1.0, 1.0, 0.0);
	if (curInstInfo.attribs.w > 0.0) roughnessAndSpecFetch = sampleRoughness (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	retVal.anisoRoughness = roughnessAndSpecFetch.xy;
	retVal.specAmount = roughnessAndSpecFetch.z;
	retVal.specularColor = sampleSpecular (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	
	retVal.diElectric = curInstInfo.attribs.z >= 1.0;
	retVal.IoR = fract (curInstInfo.attribs.z) * 10.0;
	retVal.emissivity = curInstInfo.attribs.y;
	
	return retVal;
}

void main()
{
	HitData curHitData = getHitData();
	
	vec3 emissivityAndRoughness = vec3 (curHitData.emissivity, curHitData.anisoRoughness);
	vec3 tracePos = curHitData.pos;
	bool isGlossBounce;
	vec3 curDiff = curHitData.albedoColor;
	bool diElectric = curHitData.diElectric;
	bool isGloss = curHitData.specAmount > 0.8;
	float IoR = curHitData.IoR;
	vec3 specColor = curHitData.specularColor;
	vec3 faceNorm = curHitData.faceNorm;
	
	if ( emissivityAndRoughness.x > 0.0 )
	{
		hitValues.radiance *= curDiff.rgb * emissivityAndRoughness.x;
		if ( curInstInfo.trans[0].w > 0.0 ) hitValues.radiance *= curInstInfo.trans[0].w; // Particle strength... only for particles
	}
	else if ( diElectric )
	{
		vec3 glassNorm = faceNorm;
		vec3 dirToTrace = gl_WorldRayDirectionNV;
		float hitAngle = dot (glassNorm, dirToTrace);
		if ( hitAngle < 0.0 )
		{
			if ( schlicks (1.0, IoR, glassNorm, dirToTrace) > 0.75 )
			{
				dirToTrace = reflect (dirToTrace, glassNorm);
			}
			else
			{
				dirToTrace = refract (dirToTrace, glassNorm, 1.0/IoR);
			}
		}
		else
		{
			float nRatio = 1.0/IoR;
			if ( (nRatio * nRatio) - 1.0 + (hitAngle * hitAngle) < 0.0 )
			{
				dirToTrace = reflect (dirToTrace, -glassNorm);
			}
			else
			{
				dirToTrace = refract (dirToTrace, -glassNorm, IoR);
			}
		}
		hitValues.radiance *= specColor;
		
		uint rayFlags = 0;
		uint cullMask = 0xff;
		float tmin = 0.1;
		float tmax = 1000000.0;
		
		traceNV(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , tracePos.xyz, tmin, dirToTrace, tmax, 0 );
	}
	else if ( isGloss )
	{
		vec3 glossNorm = faceNorm;
		vec3 dirToTrace = gl_WorldRayDirectionNV;
		dirToTrace = reflect (dirToTrace, glossNorm);
		hitValues.radiance *= specColor;
		
		uint rayFlags = 0;
		uint cullMask = 0xff;
		float tmin = 0.1;
		float tmax = 1000000.0;
		
		traceNV(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , tracePos.xyz, tmin, dirToTrace, tmax, 0 );
	}
	else {
		hitValues.radiance *= imageLoad (preIlluminateImage, getImageLoc(getUVW(tracePos))).rgb * curDiff.rgb;
	}
}