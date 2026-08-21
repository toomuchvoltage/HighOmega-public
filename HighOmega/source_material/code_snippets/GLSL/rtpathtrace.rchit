#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require

#define HIGHOMEGA_MAX_NEE_HINTS 200
#define HIGHOMEGA_GENERATE_HINTS_PER_LUMINARE 20
#define VOXEL_DIM_SIZE voxelizedInfo.mapMinVoxelSize.w
#define M_PI 3.1415926535
#define M_INV_PI 0.31830988618

#define diffuseSampler textures[nonuniformEXT(gl_InstanceCustomIndexEXT * 5)]
#define normalSampler textures[nonuniformEXT(gl_InstanceCustomIndexEXT * 5 + 1)]
#define roughnessSampler textures[nonuniformEXT(gl_InstanceCustomIndexEXT * 5 + 2)]
#define specularSampler textures[nonuniformEXT(gl_InstanceCustomIndexEXT * 5 + 4)]
#define curInstInfo instanceInfo.props[nonuniformEXT(gl_InstanceCustomIndexEXT)]
#define curTriBuf scene[nonuniformEXT(gl_InstanceCustomIndexEXT)]

layout(location = 0) rayPayloadInEXT struct {
	vec3 contrib;
	vec3 explicitContrib;
	vec3 lastVertexNormal;
	vec3 lastVertexPos;
	bool lastVertexDiffuse;
	uint bounceCount;
	/*bool checkLuminare;
	bool hitHint;
	vec3 hintPos;
	vec3 hintNormal;
	vec3 luminareEnergy;*/

} hitValues;
hitAttributeEXT vec3 hitBaryCoord;

layout (binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

layout (binding = 4) uniform samplerCube skyBox;

layout (binding = 5) uniform SkyShadowMapMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} SkyShadowMapMVP;

layout (binding = 6) uniform RayleighMieUBO
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

layout (binding = 7, r32ui) uniform readonly uimage2D shadowDistance;
layout (binding = 8, rgba8) uniform readonly image2D shadowColor;

struct NEEHint
{
    vec4 posArea;
	vec4 norm;
};

layout (binding = 9) uniform NEEDataUBO
{
    NEEHint hint[HIGHOMEGA_MAX_NEE_HINTS];
    uvec4 hintCountReserved;
} NEEData;

struct InstanceProps
{
	vec4 attribs;
	vec4 attribs2;
};

layout (binding = 13) uniform PathTraceParamsUBO
{
	vec4 motionFactorDiffuseOnlyReserved;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;

layout (binding = 14) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

layout (binding = 15, rgba16f) uniform image3D radiosityMapPX;
layout (binding = 16, rgba16f) uniform image3D radiosityMapNX;
layout (binding = 17, rgba16f) uniform image3D radiosityMapPY;
layout (binding = 18, rgba16f) uniform image3D radiosityMapNY;
layout (binding = 19, rgba16f) uniform image3D radiosityMapPZ;
layout (binding = 20, rgba16f) uniform image3D radiosityMapNZ;

layout (binding = 21) uniform sampler3D radiosityMapPXSampler;
layout (binding = 22) uniform sampler3D radiosityMapNXSampler;
layout (binding = 23) uniform sampler3D radiosityMapPYSampler;
layout (binding = 24) uniform sampler3D radiosityMapNYSampler;
layout (binding = 25) uniform sampler3D radiosityMapPZSampler;
layout (binding = 26) uniform sampler3D radiosityMapNZSampler;

layout (binding = 27) uniform VoxelizedInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxDynamicFlag;
	vec4 invMapDims;
	vec4 gridSize;
} voxelizedInfo;

struct TriangleFromVertBuf
{
	vec4 e1Col1;
	vec4 uv1Norm1;
	vec4 e2Col2;
	vec4 uv2Norm2;
	vec4 e3Col3;
	vec4 uv3Norm3;
};

layout(set = 1, binding = 0) buffer CompleteTriBuffer
{
	TriangleFromVertBuf tri[];
} scene[];

layout(set = 2, binding = 0) uniform sampler2DArray textures[];

vec2 getScreenSampleCoord (vec3 ptWorld)
{
    vec4 ptInfo = SkyShadowMapMVP.projectionViewMatrix * vec4 (ptWorld,1.0);
    ptInfo.xy = ((ptInfo.xy/ptInfo.w)+vec2 (1.0))*0.5;
    return ptInfo.xy;
}

vec3 getRand (vec3 forPos)
{
    float rx = fract(sin(dot(forPos, vec3(-9062.9898,  4528.233,  1296.1938))) * 43758.5453);
    float ry = fract(cos(dot(forPos, vec3( 1137.1948,  7670.963, -4424.0687))) * 43758.5453);
    float rz = fract(sin(dot(forPos, vec3( 5649.4233, -3286.248,  4353.5856))) * 93758.5453);
    
    return (vec3 (rx,ry,rz) - vec3 (0.5))* 2.0;
}

float schlicks (float n1, float n2, vec3 norm, vec3 light)
{
	float R0 = (n1 - n2)/(n1 + n2);
	R0 *= R0;
	float _1_minus_dot = 1.0 - min(abs (dot (norm, light)), 1.0);
	_1_minus_dot = _1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot;
	return R0 + (1 - R0) * _1_minus_dot;
}

vec3 diffuseBounce (mat3 tangentSpace, vec3 randVal)
{
    return normalize (tangentSpace[2] + normalize (randVal)*0.999999);
}

#define SCALE_ROUGHNESS 0.1

vec3 reflectRay (vec3 rayToReflect, mat3 tangentSpace, vec2 withRoughness, vec3 randVal)
{
	return normalize (reflect (rayToReflect, tangentSpace[2]) + tangentSpace[0] * withRoughness.x * randVal.x * SCALE_ROUGHNESS + tangentSpace[1] * withRoughness.y * randVal.y * SCALE_ROUGHNESS);
}

vec3 refractRay (vec3 rayToRefract, mat3 tangentSpace, vec2 withRoughness, float n1, float n2, out bool TIR, vec3 randVal)
{
	vec3 snellsLaw = refract (rayToRefract, tangentSpace[2], n1/n2);
	if ( snellsLaw == vec3 (0.0) )
	{
		TIR = true;
		return reflectRay (rayToRefract, tangentSpace, withRoughness, randVal);
	}
	else
	{
		TIR = false;
		return normalize (snellsLaw + tangentSpace[0] * withRoughness.x * randVal.x * SCALE_ROUGHNESS + tangentSpace[1] * withRoughness.y * randVal.y * SCALE_ROUGHNESS);
	}
}

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyBox,dirToSample).xyz;
}

float luminance(vec3 rgb)
{
    return dot(rgb, vec3(0.2125, 0.7154, 0.0721));
}

vec3 getSkyDirectLight ()
{
	vec3 toSun = normalize (rayleighMieInfo.lightDir.xyz);
	vec3 toSunUp = normalize (cross (toSun, toSun + vec3 (0.1)));
	vec3 toSunSide = cross (toSun, toSunUp);
	vec3 retVal = vec3 (0.0);
	for (int i = -1; i != 2; i++)
		for (int j = -1; j != 2; j++)
			retVal += getSkyValue (rayleighMieInfo.lightDir.xyz + toSunUp * float(i) * 0.1 + toSunSide * float(j) * 0.1);
	return retVal * 0.111111111;
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

vec3 unpackColor(float inpPack)
{
	uint packInt = floatBitsToUint (inpPack);
	vec3 color;
	color.x = float((packInt & 0xFF000000) >> 24) / 255.0;
	color.y = float((packInt & 0x00FF0000) >> 16) / 255.0;
	color.z = float((packInt & 0x0000FF00) >> 8) / 255.0;
	return color;
}

HitData getHitData ()
{
	HitData retVal;

	uint packedFlags = floatBitsToUint (curInstInfo.attribs.x);
	TriangleFromVertBuf hitTri = curTriBuf.tri[gl_PrimitiveID];
	vec3 barycoords = vec3(1.0 - hitBaryCoord.x - hitBaryCoord.y, hitBaryCoord.x, hitBaryCoord.y);

	retVal.pos = hitTri.e1Col1.xyz * barycoords.x + hitTri.e2Col2.xyz * barycoords.y + hitTri.e3Col3.xyz * barycoords.z;
	vec3 fNorm = normalize (cross (hitTri.e1Col1.xyz - hitTri.e2Col2.xyz, hitTri.e3Col3.xyz - hitTri.e2Col2.xyz));
	vec3 vNorm = toVec3 (hitTri.uv1Norm1.zw);
	if ( dot (fNorm, vNorm) < 0.0 ) fNorm = -fNorm;
	retVal.faceNorm = fNorm;
	retVal.tangent = normalize (cross (fNorm, fNorm + vec3(0.1)));
	retVal.bitan = cross (fNorm, retVal.tangent);
	vec3 v1Col = unpackColor (hitTri.e1Col1.w);
	vec2 hitUV = hitTri.uv1Norm1.xy * barycoords.x + hitTri.uv2Norm2.xy * barycoords.y + hitTri.uv3Norm3.xy * barycoords.z;
	bool isTerrain = getIsTerrain ();
	
	vec3 terrainSampleWeights = vec3 (0.0);
	if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights (hitUV);
	
	mat3 tanSpace;
	tanSpace[2] = retVal.faceNorm;
	tanSpace[0] = retVal.tangent;
	tanSpace[1] = retVal.bitan;
	retVal.norm = getFragmentNormal (terrainSampleWeights, hitUV, retVal.pos, isTerrain, tanSpace, packedFlags);
	
	retVal.albedoColor = sampleDiffuse (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	retVal.albedoColor = pow(retVal.albedoColor, vec3(2.2));
	vec3 roughnessAndSpecFetch = vec3 (1.0, 1.0, 0.0);
	if (getHasRghMap(packedFlags)) roughnessAndSpecFetch = sampleRoughness (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	retVal.anisoRoughness = roughnessAndSpecFetch.xy;
	retVal.specAmount = roughnessAndSpecFetch.z;
	retVal.specularColor = sampleSpecular (terrainSampleWeights, hitUV, retVal.pos, isTerrain).rgb;
	retVal.specularColor = pow(retVal.specularColor, vec3(2.2));
	
	retVal.diElectric = getDiElectric(packedFlags);
	retVal.emissivity = curInstInfo.attribs.z;
	if ( v1Col.x > 0.0 ) retVal.emissivity *= v1Col.x; // particle strength...
	retVal.IoR = curInstInfo.attribs.w;
	
	return retVal;
}

vec3 getSunContrib (vec3 pos, vec3 norm)
{
	float sunDotProd = max (dot (norm, rayleighMieInfo.lightDir.xyz), 0.0);
	float shadowBias = (1.0 - sunDotProd) * 1.0 + 0.5;
	float distToSkyLum = length (vec3 (SkyShadowMapMVP.lookEyeX.a, SkyShadowMapMVP.upEyeY.a, SkyShadowMapMVP.sideEyeZ.a) - pos) - shadowBias;
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

vec3 getExplicitContrib(vec3 rayOrigin, vec3 originNormal, vec3 randVal)
{
	/*uint pickAHint = uint (float (NEEData.hintCountReserved.x) * ((randVal.z + 1.0) * 0.4999999));
	NEEHint pickedHint = NEEData.hint[pickAHint];
	vec3 traceLine = pickedHint.posArea.xyz - rayOrigin;
	float traceLineLength = max (length(traceLine), 0.0001);
    vec3 traceDir = normalize (traceLine);
	float cosOutgoing = max (dot (traceDir, originNormal), 0.0);
	if ( cosOutgoing == 0.0 ) return vec3 (0.0);
	float cosIncoming = max (dot (-traceDir, pickedHint.norm.xyz), 0.0);
	if ( cosIncoming == 0.0 ) return vec3 (0.0);

    uint rayFlags = 0;
    uint cullMask = 0xff;
    float tmin = 0.1;
    float tmax = (traceLineLength + 0.1)/traceLineLength;

	hitValues.checkLuminare = true;
	hitValues.hitHint = false;
	hitValues.hintPos = pickedHint.posArea.xyz;
	hitValues.hintNormal = pickedHint.norm.xyz;

	traceRayEXT(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , rayOrigin, tmin, traceLine, tmax, 0 );
	
	hitValues.checkLuminare = false;

	if ( hitValues.hitHint )
	{
		hitValues.hitHint = false;
		float lightPDF = dot (traceLine, traceLine) / (pickedHint.posArea.a * cosIncoming);
		float scatteringPDF = cosOutgoing * M_INV_PI;
		float MISWeight = lightPDF / ((lightPDF * lightPDF) + (scatteringPDF * scatteringPDF)); // Power heuristic, B=2.0
		// M_INV_PI is because of diffuse BRDF (albedo/PI) and the foreshortening factor needs to be here: things don't cancel out anymore as the PDF cancels with the MIS weight numerator
		return (float (NEEData.hintCountReserved.x) / float(HIGHOMEGA_GENERATE_HINTS_PER_LUMINARE)) * hitValues.luminareEnergy * MISWeight * cosOutgoing * M_INV_PI;
	}
	else*/
		return vec3 (0.0);
}

vec3 getUVW (vec3 worldPos)
{
	vec3 posUVW = (worldPos - voxelizedInfo.mapMinVoxelSize.xyz) * voxelizedInfo.invMapDims.xyz;
	if ( posUVW != clamp (posUVW, vec3 (0.0), vec3 (0.999999)) ) return vec3 (2.0);
	return posUVW;
}

ivec3 getUVWLoc (vec3 worldPos)
{
	vec3 posUVW = (worldPos - voxelizedInfo.mapMinVoxelSize.xyz) * voxelizedInfo.invMapDims.xyz;
	posUVW = clamp (posUVW, vec3 (0.0), vec3 (0.999999));
	return ivec3 (posUVW * voxelizedInfo.gridSize.xyz);
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
		vec4 neighborFetch1 = texture (radiosityMapPXSampler, rayOriginUVW);
		if ( neighborFetch1.a > 0.0 ) fetch1 += vec4 (neighborFetch1.rgb / neighborFetch1.a, 1.0);
		vec4 neighborFetch2 = texture (radiosityMapNXSampler, rayOriginUVW);
		if ( neighborFetch2.a > 0.0 ) fetch2 += vec4 (neighborFetch2.rgb / neighborFetch2.a, 1.0);
		vec4 neighborFetch3 = texture (radiosityMapPYSampler, rayOriginUVW);
		if ( neighborFetch3.a > 0.0 ) fetch3 += vec4 (neighborFetch3.rgb / neighborFetch3.a, 1.0);
		vec4 neighborFetch4 = texture (radiosityMapNYSampler, rayOriginUVW);
		if ( neighborFetch4.a > 0.0 ) fetch4 += vec4 (neighborFetch4.rgb / neighborFetch4.a, 1.0);
		vec4 neighborFetch5 = texture (radiosityMapPZSampler, rayOriginUVW);
		if ( neighborFetch5.a > 0.0 ) fetch5 += vec4 (neighborFetch5.rgb / neighborFetch5.a, 1.0);
		vec4 neighborFetch6 = texture (radiosityMapNZSampler, rayOriginUVW);
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
	return totalContrib;
}

void addDiffuseRadiance (vec3 irrad, vec3 pos, vec3 dir)
{
	ivec3 uvwLoc = getUVWLoc (pos);
	vec3 xIrrad = dir.x * irrad;
	vec3 yIrrad = dir.y * irrad;
	vec3 zIrrad = dir.z * irrad;
	vec4 lightPX = imageLoad (radiosityMapPX, uvwLoc);
	vec4 lightNX = imageLoad (radiosityMapNX, uvwLoc);
	vec4 lightPY = imageLoad (radiosityMapPY, uvwLoc);
	vec4 lightNY = imageLoad (radiosityMapNY, uvwLoc);
	vec4 lightPZ = imageLoad (radiosityMapPZ, uvwLoc);
	vec4 lightNZ = imageLoad (radiosityMapNZ, uvwLoc);
	if ( dir.x > 0.0 ) lightPX += vec4 (xIrrad, 0.5);
	if ( dir.x < 0.0 ) lightNX += vec4 (-xIrrad, 0.5);
	if ( dir.y > 0.0 ) lightPY += vec4 (yIrrad, 0.5);
	if ( dir.y < 0.0 ) lightNY += vec4 (-yIrrad, 0.5);
	if ( dir.z > 0.0 ) lightPZ += vec4 (zIrrad, 0.5);
	if ( dir.z < 0.0 ) lightNZ += vec4 (-zIrrad, 0.5);
	imageStore (radiosityMapPX, uvwLoc, lightPX);
	imageStore (radiosityMapNX, uvwLoc, lightNX);
	imageStore (radiosityMapPY, uvwLoc, lightPY);
	imageStore (radiosityMapNY, uvwLoc, lightNY);
	imageStore (radiosityMapPZ, uvwLoc, lightPZ);
	imageStore (radiosityMapNZ, uvwLoc, lightNZ);
}

void main()
{
	HitData curHitData = getHitData();
	
	/*if ( hitValues.checkLuminare )
	{
		if ( length (curHitData.pos - hitValues.hintPos) < 0.1 )
		{
			hitValues.hitHint = true;
			hitValues.luminareEnergy = curHitData.albedoColor * curHitData.emissivity;
		}
		else
			hitValues.hitHint = false;
		return ;
	}*/
	
	vec3 traceDir = gl_WorldRayDirectionEXT;

	if ( curHitData.emissivity > 0.0 )
	{
		hitValues.contrib *= curHitData.albedoColor * curHitData.emissivity;
		if ( hitValues.lastVertexDiffuse ) addDiffuseRadiance (curHitData.albedoColor * curHitData.emissivity, hitValues.lastVertexPos, traceDir);
		return ;
	}
	else
	{
		vec3 randVal = getRand(curHitData.pos + vec3 (pathTraceParams.timeTurnBlurDirectionRawLight.x));
		vec3 curModColor = curHitData.albedoColor;
		hitValues.lastVertexDiffuse = false;
		mat3 tangentSpace;
		tangentSpace[0] = curHitData.tangent;
		tangentSpace[1] = curHitData.bitan;
		tangentSpace[2] = curHitData.norm;
		if ( !curHitData.diElectric )
		{
			if ( curHitData.specAmount * schlicks (1.0, curHitData.IoR, curHitData.norm, traceDir) >= max ((randVal.z + 1.0) * 0.5, 0.000001) )
			{
				curModColor = curHitData.specularColor;
				traceDir = reflectRay (traceDir, tangentSpace, curHitData.anisoRoughness, randVal);
			}
			else
			{
				hitValues.lastVertexNormal = curHitData.norm;
				hitValues.lastVertexPos = curHitData.pos;
				hitValues.lastVertexDiffuse = true;
				traceDir = diffuseBounce (tangentSpace, randVal);
				hitValues.explicitContrib += curModColor * hitValues.contrib * getExplicitContrib (hitValues.lastVertexPos, hitValues.lastVertexNormal, randVal);
				hitValues.explicitContrib += curModColor * hitValues.contrib * getSunContrib (hitValues.lastVertexPos, hitValues.lastVertexNormal);
				// Use surface caching
				vec3 surfaceCacheFetch = getDiffuseRadiance (curHitData.pos, tangentSpace);
				if ( surfaceCacheFetch != vec3(0.0) )
				{
					hitValues.contrib *= curModColor * surfaceCacheFetch;
					return ;
				}
			}
		}
		else
		{
			bool TIR = false;
			float hitAngle = dot (traceDir, curHitData.norm);
			if ( hitAngle < 0.0 )
			{
				if ( schlicks (1.0, curHitData.IoR, curHitData.norm, traceDir) >= max ((randVal.z + 1.0) * 0.5, 0.000001) )
				{
					traceDir = reflectRay (traceDir, tangentSpace, curHitData.anisoRoughness, randVal);
				}
				else
				{
					traceDir = refractRay (traceDir, tangentSpace, curHitData.anisoRoughness, 1.0, curHitData.IoR, TIR, randVal);
				}
			}
			else
			{
				tangentSpace[2] = -tangentSpace[2];
				vec3 traceDirRefracted = refractRay (traceDir, tangentSpace, curHitData.anisoRoughness, curHitData.IoR, 1.0, TIR, randVal);
				if ( TIR )
				{
					traceDir = traceDirRefracted;
				}
				else
				{
					if ( schlicks (curHitData.IoR, 1.0, curHitData.norm, traceDir) >= max ((randVal.z + 1.0) * 0.5, 0.000001) )
					{
						traceDir = reflectRay (traceDir, tangentSpace, curHitData.anisoRoughness, randVal);
					}
					else
					{
						traceDir = traceDirRefracted;
					}
				}
			}
			hitValues.bounceCount--;
		}
		hitValues.contrib *= curModColor;
		hitValues.bounceCount++;
	}
	
	if ( hitValues.bounceCount == 3 )
	{
		hitValues.contrib = vec3 (0.0);
		if ( hitValues.lastVertexDiffuse ) addDiffuseRadiance (vec3 (0.0), hitValues.lastVertexPos, traceDir);
	}
	else
	{
		uint rayFlags = 0;
		uint cullMask = 0xff;
		float tmin = 0.1;
		float tmax = 1000000.0;

		traceRayEXT(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , curHitData.pos, tmin, traceDir, tmax, 0 );
	}
}