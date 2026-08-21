#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

#define M_PI 3.1415926535
#define VOXEL_DIM_SIZE voxelizedInfo.mapMinVoxelSize.w

#define diffuseSampler sceneTextures[nonuniformEXT(chitInstanceId * 5)]
#define normalSampler sceneTextures[nonuniformEXT(chitInstanceId * 5 + 1)]
#define roughnessSampler sceneTextures[nonuniformEXT(chitInstanceId * 5 + 2)]
#define specularSampler sceneTextures[nonuniformEXT(chitInstanceId * 5 + 4)]

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 eye;
} frameMVP;

layout (binding = 1) uniform sampler2D materialAttach;
layout (binding = 2) uniform sampler2D worldPosAttach;
layout (binding = 3) uniform sampler2D normalAttach;
layout (binding = 4) uniform sampler2D tangentAttach;
layout (binding = 5) uniform samplerCube skyBox;

layout (binding = 6) uniform VoxelizedInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxWriteOffset;
	vec4 invMapDims;
	vec4 gridSize;
} voxelizedInfo;

layout (binding = 7, rgba8) uniform readonly image3D sceneVoxelized;

layout (binding = 8) uniform GridInfoUBO
{
	vec4 mapMinTrisPerCell;
	vec4 mapMaxVoxelSize;
	vec4 invMapDimsCellRadius;
	vec4 gridSize;
} gridInfo;

layout (binding = 9, r32ui) uniform readonly uimage3D sceneGrid;

layout (binding = 10) uniform SkyShadowMapMVPUBO
{
    mat4 projectionViewMatrix;
    vec4 eye;
} SkyShadowMapMVP;

layout (binding = 11) uniform RayleighMieUBO
{
	vec4 lightDir;
	vec4 sunDirAndExp;
	vec4 invW4InnerRad;

	float innerCloudRad;
	float outerRad;
	float scaleDepth;
	float scaleOverScaleDepth;
	
	float scatteringCoeff;
	float extinctionCoeff;
	float ambientCoeff;
	float stepSizeEvalLightDir;

	float stepSizeToSky;
	float numStepsToSky;
	float stepSizeToSun;
	float numStepsToSun;
	
	vec4 addSkyColorAndNightAmount;
	vec4 skyObjectLightAndAngle;
	vec4 horizonColor;
	vec4 approxGroundColor;
} rayleighMieInfo;

layout (binding = 12, r32ui) uniform readonly uimage2D shadowDistance;
layout (binding = 13, rgba8) uniform readonly image2D shadowColor;

layout (binding = 14) uniform sampler3D radiosityMapPX;
layout (binding = 15) uniform sampler3D radiosityMapNX;
layout (binding = 16) uniform sampler3D radiosityMapPY;
layout (binding = 17) uniform sampler3D radiosityMapNY;
layout (binding = 18) uniform sampler3D radiosityMapPZ;
layout (binding = 19) uniform sampler3D radiosityMapNZ;

layout (binding = 20, rgba16f) uniform writeonly image2D glossTraceOutput;

vec3 getUVW (vec3 worldPos)
{
	vec3 posUVW = (worldPos - voxelizedInfo.mapMinVoxelSize.xyz) * voxelizedInfo.invMapDims.xyz;
	if ( posUVW != clamp (posUVW, vec3 (0.0), vec3 (0.999999)) ) return vec3 (2.0);
	return posUVW;
}

struct GridTriangle
{
	vec4 e1u1;
	vec4 e2v1;
	vec4 e3u2;
	vec4 normv2;
	vec4 tanu3;
	vec4 bitanv3;
};

layout (set = 2, binding = 0) buffer GridTriangleBuffer
{
	GridTriangle tris[];
} sceneGeom[];

layout (set = 3, binding = 0) uniform sampler2DArray sceneTextures[];

struct InstanceProps
{
	mat4 trans;
	mat4 transDT;
	vec4 attribs;
	vec4 attribs2;
};

layout (set = 4, binding = 0) buffer StaticInstanceInfoSSBO
{
	InstanceProps props[];
} staticInstanceInfo;

layout (set = 4, binding = 1) buffer DynamicInstanceInfoSSBO
{
	InstanceProps props[];
} dynamicInstanceInfo;

layout (set = 4, binding = 2) uniform GridClearInfoUBO
{
	uvec4 offsetMaxTrisPerCellOffsetShiftedReserved;
} gridClearInfo;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
}

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyBox,dirToSample).xyz;
}

vec2 getScreenSampleCoord (vec3 ptWorld)
{
    vec4 ptInfo = SkyShadowMapMVP.projectionViewMatrix * vec4 (ptWorld,1.0);
    ptInfo.xy = ((ptInfo.xy/ptInfo.w)+vec2 (1.0))*0.5;
    return ptInfo.xy;
}

float luminance(vec3 rgb)
{
    return dot(rgb, vec3(0.2125, 0.7154, 0.0721));
}

vec3 getSkyDirectLight ()
{
	return rayleighMieInfo.skyObjectLightAndAngle.xyz * max (luminance (getSkyValue (rayleighMieInfo.lightDir.xyz) * 0.1) * sqrt (max (rayleighMieInfo.sunDirAndExp.y, 0.0)), 1.0);
}

vec3 getSunContrib (vec3 pos, vec3 norm)
{
	float sunDotProd = max (dot (norm, rayleighMieInfo.lightDir.xyz), 0.0);
	float shadowBias = (1.0 - sunDotProd) * 1.0 + 0.5;
	float distToSkyLum = length (SkyShadowMapMVP.eye.xyz - pos) - shadowBias;
	ivec2 texeLoc = ivec2 (clamp (getScreenSampleCoord (pos), vec2 (0.0), vec2 (0.999999)) * vec2 (imageSize(shadowDistance).xy));
	uint distanceFetch = imageLoad (shadowDistance, texeLoc).x;
	float shadowDist = sqrt (float (distanceFetch) * 0.1);
	float sunVis = max (sign (shadowDist - distToSkyLum), 0.0) * sunDotProd;
	if ( sunVis > 0.0 )
	{
		vec3 retVal = getSkyDirectLight () * sunVis;
		vec3 stainedColorFetch = imageLoad (shadowColor, texeLoc).rgb;
		if ( stainedColorFetch != vec3 (0.0) ) retVal *= stainedColorFetch;
		return retVal;
	}
	return vec3 (0.0);
}

// Moller-Trumbore
bool lineSegTri(vec3 orig, vec3 dir, vec3 p1, vec3 p2, vec3 p3, inout float curK)
{
	vec3 e1, e2;
	vec3 P, Q, T;
	float det, inv_det, u, v;
	float t;
	e1 = p2 - p1;
	e2 = p3 - p1;
	P = cross(dir, e2);
	det = dot(e1, P);
	if (det > -0.0001 && det < 0.0001) return false;
	inv_det = 1.0 / det;
	T = orig - p1;
	u = dot(T, P) * inv_det;
	if (u < 0.0 || u > 1.0) return false;
	Q = cross(T, e1);
	v = dot(dir, Q) * inv_det;
	if (v < 0.0 || (u + v) > 1.0) return false;
	t = dot(e2, Q) * inv_det;
	if (t > 0.0 && t < curK)
	{
		curK = t;
		return true;
	}
	return false;
}

// Cramer's rule, Christer Ericson 2005
vec3 barycentricCoords(vec3 p, vec3 a, vec3 b, vec3 c)
{
    vec3 v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float invDenom = 1.0 / (d00 * d11 - d01 * d01);
    float v = (d11 * d20 - d01 * d21) * invDenom;
    float w = (d00 * d21 - d01 * d20) * invDenom;
    float u = 1.0 - v - w;
	return vec3 (u,v,w);
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

bool getIsTerrain (uint chitInstanceId)
{
	return textureSize (diffuseSampler, 0).z > 1;
}

bool getIsAlphaMasked (uint packedFlags)
{
	return (packedFlags & 0x00000080) != 0;
}

vec3 sampleTerrainWeights (vec2 inUV, uint chitInstanceId)
{
	vec3 terrainMap = texture (diffuseSampler, vec3 (inUV, 3.0)).rgb;
	terrainMap /= (terrainMap.x + terrainMap.y + terrainMap.z);
	return terrainMap;
}

vec4 sampleDiffuse (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, uint chitInstanceId)
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

vec4 sampleSpecular (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, uint chitInstanceId)
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

vec3 sampleNormal (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, uint chitInstanceId)
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

vec4 sampleRoughness (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, uint chitInstanceId)
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

vec3 getFragmentNormal (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, mat3 tanSpace, uint packedFlags, uint chitInstanceId)
{
	if (getHasNrmMap(packedFlags))
		return normalize (tanSpace * sampleNormal (inpWeights, inUV, posAtSample, isTerrain, chitInstanceId));
	else
		return normalize (tanSpace[2]);
}

struct TriHitData
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

TriHitData getTriHitDataSlim (vec3 barycoords, uint packedFlags, vec3 hitPos, GridTriangle hitTri, uint chitInstanceId)
{
	TriHitData retVal;

	retVal.faceNorm = hitTri.normv2.xyz;
	retVal.tangent = hitTri.tanu3.xyz;
	retVal.bitan = hitTri.bitanv3.xyz;
	vec2 hitUV = vec2 (hitTri.e1u1.w, hitTri.e2v1.w) * barycoords.x + vec2 (hitTri.e3u2.w, hitTri.normv2.w) * barycoords.y + vec2 (hitTri.tanu3.w, hitTri.bitanv3.w) * barycoords.z;
	bool isTerrain = getIsTerrain (chitInstanceId);
	
	vec3 terrainSampleWeights = vec3 (0.0);
	if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights (hitUV, chitInstanceId);
	
	mat3 tanSpace;
	tanSpace[2] = retVal.faceNorm;
	tanSpace[0] = retVal.tangent;
	tanSpace[1] = retVal.bitan;
	retVal.norm = getFragmentNormal (terrainSampleWeights, hitUV, hitPos, isTerrain, tanSpace, packedFlags, chitInstanceId);
	
	retVal.albedoColor = sampleDiffuse (terrainSampleWeights, hitUV, hitPos, isTerrain, chitInstanceId).rgb;
	vec3 roughnessAndSpecFetch = vec3 (1.0, 1.0, 0.0);
	//if (getHasRghMap(packedFlags)) roughnessAndSpecFetch = sampleRoughness (terrainSampleWeights, hitUV, hitPos, isTerrain, chitInstanceId).rgb;
	retVal.anisoRoughness = roughnessAndSpecFetch.xy;
	retVal.specAmount = roughnessAndSpecFetch.z;
	//retVal.specularColor = sampleSpecular (terrainSampleWeights, hitUV, hitPos, isTerrain, chitInstanceId).rgb;
	
	//retVal.diElectric = getDiElectric(packedFlags);
	
	return retVal;
}

bool rayIntersect(vec3 rayStart, vec3 pixelDir, int minMovementToAvoidSelfIntersect, inout TriHitData triHitData)
{
	vec3 moveAmount = pixelDir * gridInfo.mapMaxVoxelSize.a; // Shorter lines are preferred
	rayStart += moveAmount * 0.1;
	vec3 curRayStart = rayStart;
	vec3 rayDir = pixelDir * 100000.0;
	ivec3 lastUVWLoc = ivec3 (-1);
	
	uint chitInstanceId = 0xFFFFFFFF;
	GridTriangle chitTri, curTri;
	uint chitPackedFlags;
	float chitEmissivity, chitIoR;
	float rayT = 1.0;
	bool intersected = false;
	InstanceProps curProps;
	vec3 curRayStartUVW;
	ivec3 curRayStartUVWLoc, instTriLoc;
	uint instanceTriId, instanceId, packedFlags;
	bool isAlphaMasked;
	
	for (int loopGuard = 0; loopGuard != 1000; loopGuard++)
	{
		curRayStartUVW = (curRayStart - gridInfo.mapMinTrisPerCell.xyz) * gridInfo.invMapDimsCellRadius.xyz;
	
		if ( curRayStartUVW != clamp (curRayStartUVW, vec3 (0.0), vec3 (1.0)) ) break;

		curRayStartUVWLoc = ivec3(curRayStartUVW * gridInfo.gridSize.xyz);
		if ( curRayStartUVWLoc == lastUVWLoc )
		{
			curRayStart += moveAmount;
			continue;
		}

		for (int ii = 0; ii != int(gridInfo.mapMinTrisPerCell.a); ii++)
		{
			instTriLoc = ivec3 (curRayStartUVWLoc.x*int(gridInfo.mapMinTrisPerCell.a) + ii, curRayStartUVWLoc.yz);
			instanceTriId = imageLoad (sceneGrid, instTriLoc).x;
			if ( instanceTriId == 0xFFFFFFFF ) break;
			
			instanceId = instanceTriId >> 16;

			float emissivity, IoR;
			if ( instanceId >= gridClearInfo.offsetMaxTrisPerCellOffsetShiftedReserved.z )
				curProps = dynamicInstanceInfo.props[instanceId - gridClearInfo.offsetMaxTrisPerCellOffsetShiftedReserved.z];
			else
				curProps = staticInstanceInfo.props[instanceId];
			packedFlags = floatBitsToUint (curProps.attribs.x);
			emissivity = curProps.attribs.z;
			IoR = curProps.attribs.w;
				
			isAlphaMasked = getIsAlphaMasked (packedFlags);

			curTri = sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (instanceTriId & 0x0000FFFF)];

			float saveRayT = rayT;
			if ( lineSegTri (rayStart, rayDir, curTri.e1u1.xyz, curTri.e2v1.xyz, curTri.e3u2.xyz, rayT) )
			{
				if (isAlphaMasked)
				{
					vec3 hitPt = rayDir * rayT + rayStart;
					vec3 baryCoords = barycentricCoords (hitPt, curTri.e1u1.xyz, curTri.e2v1.xyz, curTri.e3u2.xyz);
					vec2 hitUV = baryCoords.x * vec2 (curTri.e1u1.a, curTri.e2v1.a) + baryCoords.y * vec2 (curTri.e3u2.a, curTri.normv2.a) + baryCoords.z * vec2 (curTri.tanu3.a, curTri.bitanv3.a);
					if ( texture (sceneTextures[nonuniformEXT (instanceId * 5)], vec3 (hitUV, 0.0), 0.0).a == 1.0 )
					{
						chitInstanceId = instanceId;
						chitTri = curTri;
						chitPackedFlags = packedFlags;
						chitEmissivity = emissivity;
						chitIoR = IoR;
						intersected = true;
					}
					else
					{
						rayT = saveRayT;
					}
				}
				else
				{
					chitInstanceId = instanceId;
					chitTri = curTri;
					chitPackedFlags = packedFlags;
					chitEmissivity = emissivity;
					chitIoR = IoR;
					intersected = true;
				}
			}
			if ( ii == int(gridInfo.mapMinTrisPerCell.a) - 1 && !intersected && loopGuard > minMovementToAvoidSelfIntersect )
			{
				// exhaustion, there are more primitives that are probably missed so try and report an intersection
				vec3 potentialHitPt = curRayStart + moveAmount * 0.5;
				if (isAlphaMasked)
				{
					vec3 baryCoords = barycentricCoords (potentialHitPt, curTri.e1u1.xyz, curTri.e2v1.xyz, curTri.e3u2.xyz);
					vec2 hitUV = baryCoords.x * vec2 (curTri.e1u1.a, curTri.e2v1.a) + baryCoords.y * vec2 (curTri.e3u2.a, curTri.normv2.a) + baryCoords.z * vec2 (curTri.tanu3.a, curTri.bitanv3.a);
					if ( texture (sceneTextures[nonuniformEXT (instanceId * 5)], vec3 (hitUV, 0.0), 0.0).a == 1.0 )
					{
						chitInstanceId = instanceId;
						chitTri = curTri;
						chitPackedFlags = packedFlags;
						chitEmissivity = emissivity;
						chitIoR = IoR;
						rayT = length (potentialHitPt - rayStart) * 0.00001;
						intersected = true;
					}
				}
				else
				{
					chitInstanceId = instanceId;
					chitTri = curTri;
					chitPackedFlags = packedFlags;
					chitEmissivity = emissivity;
					chitIoR = IoR;
					rayT = length (potentialHitPt - rayStart) * 0.00001;
					intersected = true;
				}
			}
		}

		if ( intersected )
		{
			vec3 hitPt = rayDir * rayT + rayStart;
			vec3 baryCoords = barycentricCoords (hitPt, chitTri.e1u1.xyz, chitTri.e2v1.xyz, chitTri.e3u2.xyz);
			triHitData = getTriHitDataSlim (baryCoords, chitPackedFlags, hitPt, chitTri, chitInstanceId);
            triHitData.pos = hitPt;
            triHitData.emissivity = chitEmissivity;
            triHitData.IoR = chitIoR;
			return true;
		}
		curRayStart += moveAmount;
		lastUVWLoc = curRayStartUVWLoc;
	}
	return false;
}

vec2 unpackEmissivityRefractiveIndexDielectric (float inpEmissivityRefractiveIndexDielectricPacked, out bool isDielectric)
{
	float ourEmissivityRefractiveIndexDielectricPacked = inpEmissivityRefractiveIndexDielectricPacked;
	if ( ourEmissivityRefractiveIndexDielectricPacked < 0.0 )
	{
		ourEmissivityRefractiveIndexDielectricPacked = -ourEmissivityRefractiveIndexDielectricPacked;
		isDielectric = true;
	}
	else
	{
		isDielectric = false;
	}
	float emissivity = fract (ourEmissivityRefractiveIndexDielectricPacked)* 255.0;
	float refractiveIndex = floor (ourEmissivityRefractiveIndexDielectricPacked) / 1000.0;
	return vec2 (emissivity,refractiveIndex);
}

vec3 getDiffuseRadiance(vec3 pos, mat3 tangentSpace)
{
	vec4 fetch1, fetch2, fetch3, fetch4, fetch5, fetch6;
	fetch1 = fetch2 = fetch3 = fetch4 = fetch5 = fetch6 = vec4(0.0);
	for (int i = 0; i != 5; i++)
	{
		vec3 rayOriginUVW;
		switch (i)
		{
			case 0:
			 rayOriginUVW = getUVW (pos);
			 break;
			case 1:
			 rayOriginUVW = getUVW (pos - tangentSpace[0] * VOXEL_DIM_SIZE);
			 break;
			case 2:
			 rayOriginUVW = getUVW (pos + tangentSpace[0] * VOXEL_DIM_SIZE);
			 break;
			case 3:
			 rayOriginUVW = getUVW (pos - tangentSpace[1] * VOXEL_DIM_SIZE);
			 break;
			default:
			 rayOriginUVW = getUVW (pos + tangentSpace[1] * VOXEL_DIM_SIZE);
			 break;
		}
		vec4 neighborFetch1 = texture (radiosityMapPX, rayOriginUVW);
		if ( neighborFetch1.a > 0.0 ) fetch1 += vec4 (neighborFetch1.rgb / neighborFetch1.a, 1.0);
		vec4 neighborFetch2 = texture (radiosityMapNX, rayOriginUVW);
		if ( neighborFetch2.a > 0.0 ) fetch2 += vec4 (neighborFetch2.rgb / neighborFetch2.a, 1.0);
		vec4 neighborFetch3 = texture (radiosityMapPY, rayOriginUVW);
		if ( neighborFetch3.a > 0.0 ) fetch3 += vec4 (neighborFetch3.rgb / neighborFetch3.a, 1.0);
		vec4 neighborFetch4 = texture (radiosityMapNY, rayOriginUVW);
		if ( neighborFetch4.a > 0.0 ) fetch4 += vec4 (neighborFetch4.rgb / neighborFetch4.a, 1.0);
		vec4 neighborFetch5 = texture (radiosityMapPZ, rayOriginUVW);
		if ( neighborFetch5.a > 0.0 ) fetch5 += vec4 (neighborFetch5.rgb / neighborFetch5.a, 1.0);
		vec4 neighborFetch6 = texture (radiosityMapNZ, rayOriginUVW);
		if ( neighborFetch6.a > 0.0 ) fetch6 += vec4 (neighborFetch6.rgb / neighborFetch6.a, 1.0);
	}
	if ( fetch1.a > 0.0 ) fetch1.rgb /= fetch1.a;
	if ( fetch2.a > 0.0 ) fetch2.rgb /= fetch2.a;
	if ( fetch3.a > 0.0 ) fetch3.rgb /= fetch3.a;
	if ( fetch4.a > 0.0 ) fetch4.rgb /= fetch4.a;
	if ( fetch5.a > 0.0 ) fetch5.rgb /= fetch5.a;
	if ( fetch6.a > 0.0 ) fetch6.rgb /= fetch6.a;
	float factor1 = max (tangentSpace[2].x, 0.0);
	float factor2 = max (-tangentSpace[2].x, 0.0);
	float factor3 = max (tangentSpace[2].y, 0.0);
	float factor4 = max (-tangentSpace[2].y, 0.0);
	float factor5 = max (tangentSpace[2].z, 0.0);
	float factor6 = max (-tangentSpace[2].z, 0.0);
	vec3 contrib1 = factor1 * fetch1.rgb;
	vec3 contrib2 = factor2 * fetch2.rgb;
	vec3 contrib3 = factor3 * fetch3.rgb;
	vec3 contrib4 = factor4 * fetch4.rgb;
	vec3 contrib5 = factor5 * fetch5.rgb;
	vec3 contrib6 = factor6 * fetch6.rgb;
	vec3 totalContrib = (contrib1 + contrib2 + contrib3 + contrib4 + contrib5 + contrib6) / (factor1 + factor2 + factor3 + factor4 + factor5 + factor6);
	if ( luminance (totalContrib) < 0.05 ) totalContrib += vec3 (0.05 - luminance(totalContrib));
	return totalContrib;
}

void main()
{
	ivec2 texelLoc = ivec2 (inUV * vec2 (textureSize(materialAttach, 0)));
	ivec2 storeCoord = ivec2 (inUV * vec2(imageSize(glossTraceOutput)));

	vec4 fetchedWorldPosAndMaterialHint = texelFetch(worldPosAttach, texelLoc, 0);
	if (fetchedWorldPosAndMaterialHint == vec4 (0.0))
	{
		imageStore (glossTraceOutput, storeCoord, vec4 (0.0, 0.0, 0.0, -1.0));
		return ;
	}
	vec4 materialFetch = texelFetch(materialAttach, texelLoc, 0);
	bool isDielectric;
	vec2 emissivityRefractiveIndex = unpackEmissivityRefractiveIndexDielectric (materialFetch.w, isDielectric);

	if ( emissivityRefractiveIndex.x > 0.0 )
	{
		imageStore (glossTraceOutput, storeCoord, vec4 (0.0, 0.0, 0.0, -1.0));
		return ;
	}
	
	vec3 rayOrigin = fetchedWorldPosAndMaterialHint.xyz;
	vec3 rayDir = normalize (fetchedWorldPosAndMaterialHint.xyz - frameMVP.eye.xyz);
	
	vec4 fragmentRoughnessSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.z));
	
	vec4 normalFetch = texelFetch(normalAttach, texelLoc, 0);
	vec4 tangentSpaceFetch = texelFetch(tangentAttach, texelLoc, 0);

	mat3 tangentSpace;
	tangentSpace[2] = toVec3 (normalFetch.xy);
	vec3 fragmentGeomNorm = toVec3 (normalFetch.zw);
	tangentSpace[0] = toVec3 (tangentSpaceFetch.xy);
	tangentSpace[1] = toVec3 (tangentSpaceFetch.zw);

	tangentSpace[2] = -sign (dot (rayDir, tangentSpace[2])) * tangentSpace[2];
	fragmentGeomNorm = -sign (dot (rayDir, fragmentGeomNorm)) * fragmentGeomNorm;
	
	rayDir = reflect (rayDir, tangentSpace[2]);

	vec4 traceContrib = vec4 (0.0, 0.0, 0.0, -1.0);
	if ( fragmentRoughnessSpecularity.b > 0.0 )
	{
		float kernelSize = max(fragmentRoughnessSpecularity.r, fragmentRoughnessSpecularity.g);

		TriHitData triHitData;
		if ( rayIntersect (rayOrigin, rayDir, int(1.0 / abs (dot (rayDir, fragmentGeomNorm))), triHitData) )
		{
			vec3 toHit = triHitData.pos - rayOrigin;
			traceContrib.a = kernelSize * min (dot (toHit, toHit) * 0.0004, 1.0);
			if ( triHitData.emissivity > 0.0 )
			{
				traceContrib.rgb = triHitData.emissivity * triHitData.albedoColor;
			}
			else
			{
				mat3 hitTanSpace;
				hitTanSpace[0] = triHitData.tangent;
				hitTanSpace[1] = triHitData.bitan;
				hitTanSpace[2] = triHitData.norm;
				traceContrib.rgb = (getDiffuseRadiance (triHitData.pos, hitTanSpace) + getSunContrib (triHitData.pos, triHitData.norm)) * triHitData.albedoColor;
			}
		}
		else
		{
			traceContrib = vec4 (getSkyValue (rayDir), kernelSize);
		}
	}
	imageStore (glossTraceOutput, storeCoord, traceContrib);
}