#version 460
#extension GL_NV_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require

#define M_PI 3.1415926535

#define diffuseSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5)]
#define normalSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 1)]
#define roughnessSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 2)]
#define specularSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 4)]
#define curInstInfo instanceInfo.props[nonuniformEXT(gl_InstanceCustomIndexNV)]
#define curTriBuf scene[nonuniformEXT(gl_InstanceCustomIndexNV)]

layout(location = 0) rayPayloadInNV struct {
	float emissivity;
	vec3 albedo;
	vec3 pos;
	mat3 tanSpace;
} hitValues;
hitAttributeNV vec3 hitBaryCoord;

struct InstanceProps
{
	mat4 trans;
	mat4 transDT;
	vec4 attribs;
	vec4 attribs2;
};

layout (binding = 18) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

struct TriangleFromVertBuf
{
	vec4 e1u1;
	vec4 norms1;
	vec4 tanspace1;
	vec4 v1col1;
	vec4 e2u2;
	vec4 norms2;
	vec4 tanspace2;
	vec4 v2col2;
	vec4 e3u3;
	vec4 norms3;
	vec4 tanspace3;
	vec4 v3col3;
};

layout(set = 1, binding = 0) buffer CompleteTriBuffer
{
	TriangleFromVertBuf tri[];
} scene[];

layout(set = 2, binding = 0) uniform sampler2DArray textures[];

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

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
}

bool getHasNrmMap (uint packedFlags)
{
	return (packedFlags & 0x00000001) != 0;
}

bool getHasRghMap (uint packedFlags)
{
	return (packedFlags & 0x00000002) != 0;
}

bool getDiElectric (uint packedFlags)
{
	return (packedFlags & 0x00000040) != 0;
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

vec3 getFragmentNormal (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, mat3 tanSpace, uint packedFlags)
{
	if (getHasNrmMap(packedFlags))
		return normalize (tanSpace * sampleNormal (inpWeights, inUV, posAtSample, isTerrain));
	else
		return normalize (tanSpace[2]);
}

HitData getHitDataSlim ()
{
	HitData retVal;

	mat4 currentTrans = curInstInfo.trans;
	mat4 currentTransDT = curInstInfo.transDT;
	uint packedFlags = floatBitsToUint (curInstInfo.attribs.x);
	TriangleFromVertBuf hitTri = curTriBuf.tri[gl_PrimitiveID];
	vec3 barycoords = vec3(1.0 - hitBaryCoord.x - hitBaryCoord.y, hitBaryCoord.x, hitBaryCoord.y);

	retVal.pos = (currentTrans * vec4 (hitTri.e1u1.xyz * barycoords.x + hitTri.e2u2.xyz * barycoords.y + hitTri.e3u3.xyz * barycoords.z, 1.0)).xyz;
	retVal.faceNorm = (currentTransDT * vec4 (toVec3(hitTri.norms1.xy), 1.0)).xyz;
	retVal.tangent = (currentTransDT * vec4 (toVec3(hitTri.tanspace1.xy), 1.0)).xyz;
	retVal.bitan = (currentTransDT * vec4 (toVec3(hitTri.tanspace1.zw), 1.0)).xyz;
	vec2 hitUV = vec2 (hitTri.e1u1.w, hitTri.v1col1.x) * barycoords.x + vec2 (hitTri.e2u2.w, hitTri.v2col2.x) * barycoords.y + vec2 (hitTri.e3u3.w, hitTri.v3col3.x) * barycoords.z;
	bool isTerrain = getIsTerrain ();
	
	vec3 terrainSampleWeights = vec3 (0.0);
	if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights (hitUV);
	
	mat3 tanSpace;
	tanSpace[2] = retVal.faceNorm;
	tanSpace[0] = retVal.tangent;
	tanSpace[1] = retVal.bitan;
	retVal.norm = getFragmentNormal (terrainSampleWeights, hitUV, retVal.pos, isTerrain, tanSpace, packedFlags);
	
	retVal.albedoColor = sampleDiffuse (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	vec3 roughnessAndSpecFetch = vec3 (1.0, 1.0, 0.0);
	//if (getHasRghMap(packedFlags)) roughnessAndSpecFetch = sampleRoughness (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	retVal.anisoRoughness = roughnessAndSpecFetch.xy;
	retVal.specAmount = roughnessAndSpecFetch.z;
	//retVal.specularColor = sampleSpecular (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	
	//retVal.diElectric = getDiElectric(packedFlags);
	retVal.emissivity = curInstInfo.attribs.z;
	retVal.IoR = curInstInfo.attribs.w;
	
	return retVal;
}

void main()
{
	HitData curHitData = getHitDataSlim();
	hitValues.emissivity = curHitData.emissivity;
	hitValues.albedo = curHitData.albedoColor;
	hitValues.pos = curHitData.pos;
	hitValues.tanSpace[0] = curHitData.tangent;
	hitValues.tanSpace[1] = curHitData.bitan;
	hitValues.tanSpace[2] = curHitData.norm;
}