#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

#define HIGHOMEGA_MAX_NEE_HINTS 200
#define HIGHOMEGA_GENERATE_HINTS_PER_LUMINARE 20
#define VOXEL_DIM_SIZE voxelizedInfo.mapMinVoxelSize.w
#define M_PI 3.1415926535
#define M_INV_PI 0.31830988618

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} frameMVP;

layout (binding = 1) uniform sampler2D materialAttach;
layout (binding = 2) uniform sampler2D worldPosAttach;
layout (binding = 3) uniform sampler2D normalAttach;
layout (binding = 4) uniform samplerCube skyBox;

layout (binding = 5) uniform VoxelizedInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxDynamicFlag;
	vec4 invMapDims;
	vec4 gridSize;
} voxelizedInfo;

layout (binding = 6, rgba8) uniform readonly image3D sceneVoxelized;

layout (binding = 7) uniform SkyShadowMapMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} SkyShadowMapMVP;

layout (binding = 8) uniform RayleighMieUBO
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

layout (binding = 9, r32ui) uniform readonly uimage2D shadowDistance;
layout (binding = 10, rgba8) uniform readonly image2D shadowColor;

struct NEEHint
{
    vec4 posArea;
	vec4 norm;
};

layout (binding = 11) uniform NEEDataUBO
{
    NEEHint hint[HIGHOMEGA_MAX_NEE_HINTS];
    uvec4 hintCountReserved;
} NEEData;

layout (binding = 12) uniform sampler2D blueNoise;

layout (binding = 13) uniform PathTraceParamsUBO
{
	vec4 motionFactorDiffuseOnlyReserved;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;

layout (binding = 14, rgba16f) uniform image3D radiosityMapPX;
layout (binding = 15, rgba16f) uniform image3D radiosityMapNX;
layout (binding = 16, rgba16f) uniform image3D radiosityMapPY;
layout (binding = 17, rgba16f) uniform image3D radiosityMapNY;
layout (binding = 18, rgba16f) uniform image3D radiosityMapPZ;
layout (binding = 19, rgba16f) uniform image3D radiosityMapNZ;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

vec3 getRand (vec3 forPos)
{
    float rx = fract(sin(dot(forPos, vec3(-9062.9898,  4528.233,  1296.1938))) * 43758.5453);
    float ry = fract(cos(dot(forPos, vec3( 1137.1948,  7670.963, -4424.0687))) * 43758.5453);
    float rz = fract(sin(dot(forPos, vec3( 5649.4233, -3286.248,  4353.5856))) * 93758.5453);
    
    return (vec3 (rx,ry,rz) - vec3 (0.5))* 2.0;
}

vec2 unpackSpherical(float inpPack)
{
	uint packInt = floatBitsToUint(inpPack);
	vec2 retVal;
	retVal.x = float((packInt & 0xFFFF0000) >> 16) / 65535.0;
	retVal.y = float(packInt & 0x0000FFFF) / 65535.0;
	return retVal;
}

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
}

mat3 decodeTanBitan (float encodedTanBiTan, vec3 inpNorm)
{
	float encodedVal = round (encodedTanBiTan * 255.0);
	bool biTanDotBasisYNegative = false;
	if ( encodedVal >= 128.0 )
	{
		encodedVal -= 128.0;
		biTanDotBasisYNegative = true;
	}
	bool tanDotBasisYNegative = false;
	if ( encodedVal >= 64.0 )
	{
		encodedVal -= 64.0;
		tanDotBasisYNegative = true;
	}
	float basisXShadow = clamp (((encodedVal / 63.0) * 2.0) - 1.0, -1.0, 1.0);
	float basisYShadow = sqrt (1.0 - basisXShadow*basisXShadow);
	vec3 aBasisX = normalize (cross (inpNorm, inpNorm + vec3 (0.1)));
	vec3 aBasisY = cross (inpNorm,aBasisX);
	vec3 tanVector = basisXShadow * aBasisX;
	if (tanDotBasisYNegative)
		tanVector -= basisYShadow * aBasisY;
	else
		tanVector += basisYShadow * aBasisY;
	tanVector = normalize (tanVector);
	vec3 biTanVector = cross (inpNorm, tanVector);
	if ( dot (biTanVector, aBasisY) > 0.0 && biTanDotBasisYNegative ) biTanVector = -biTanVector;
	mat3 retVal;
	retVal[0] = tanVector;
	retVal[1] = biTanVector;
	retVal[2] = inpNorm;
	return retVal;
}

mat3 getOrthoBasisAndSpec (ivec3 texelLoc, out float specularityOut)
{
	vec4 fetchEncodedBasis = imageLoad(sceneVoxelized,texelLoc);
	specularityOut = fetchEncodedBasis.a;
	vec3 decodedNorm = normalize (toVec3 (fetchEncodedBasis.rg));
	mat3 decodedBasis = decodeTanBitan (fetchEncodedBasis.b, decodedNorm);
	return decodedBasis;
}

vec3 getEmissivityDielectricRefractiveIndexAndRoughness3DImage (ivec3 texelLoc, out bool diElectric, out float IoR)
{
	vec4 rawFetch = imageLoad(sceneVoxelized,texelLoc);
	float pureEmissivity = rawFetch.x * 25.5;
	vec2 fetchedRoughness = rawFetch.zw;
	
	float dielectricRefractiveIndex = rawFetch.y * 255.0;
	if ( dielectricRefractiveIndex >= 128.0 )
	{
		diElectric = true;
		dielectricRefractiveIndex -= 128.0;
	}
	else
		diElectric = false;
		
	IoR = dielectricRefractiveIndex * 0.03149606299212598425196850393701;

	return vec3 (pureEmissivity, fetchedRoughness);
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

struct VoxelHitData
{
	vec4 albedo;
	vec4 specular;
	mat3 tangentSpace;

	vec2 roughness;
	float specularity;

	bool diElectric;
	float IoR;
	float emissivity;
};

VoxelHitData getMaterial (vec3 uvwLoc)
{
	VoxelHitData voxHitData;
	voxHitData.albedo = vec4 (0.0);
	voxHitData.specular = vec4 (0.0);
	voxHitData.tangentSpace = mat3(1.0);
	voxHitData.roughness = vec2 (0.0);
	voxHitData.specularity = 0.0;
	voxHitData.diElectric = false;
	voxHitData.IoR = 0.0;
	voxHitData.emissivity = 0.0;
	
	ivec3 texelLoc = ivec3 (uvwLoc * voxelizedInfo.gridSize.xyz);
	texelLoc.x *= 4;

	vec4 diffFetch = imageLoad (sceneVoxelized, texelLoc);
	diffFetch.rgb = pow (diffFetch.rgb, vec3(2.2));
	if ( diffFetch.a == 0.0 ) return voxHitData;
	
	voxHitData.albedo = diffFetch;
	voxHitData.specular = imageLoad (sceneVoxelized, ivec3 (texelLoc.x + 3, texelLoc.yz));
	voxHitData.specular.rgb = pow (voxHitData.specular.rgb, vec3(2.2));

	bool diElectric = false;
	float IoR = 0.0;
	vec3 emissivityAndRoughness = getEmissivityDielectricRefractiveIndexAndRoughness3DImage (ivec3 (texelLoc.x + 2, texelLoc.yz), diElectric, IoR);
	voxHitData.emissivity = emissivityAndRoughness.x;
	voxHitData.roughness = emissivityAndRoughness.yz;
	voxHitData.diElectric = diElectric;
	voxHitData.IoR = IoR;
	
	float specularity = 0.0;
	mat3 tangentSpace = getOrthoBasisAndSpec (ivec3 (texelLoc.x + 1, texelLoc.yz), specularity);
	
	voxHitData.tangentSpace = tangentSpace;
	voxHitData.specularity = specularity;
	
	return voxHitData;
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
	vec3 toSun = normalize (rayleighMieInfo.lightDir.xyz);
	vec3 toSunUp = normalize (cross (toSun, toSun + vec3 (0.1)));
	vec3 toSunSide = cross (toSun, toSunUp);
	vec3 retVal = vec3 (0.0);
	for (int i = -1; i != 2; i++)
		for (int j = -1; j != 2; j++)
			retVal += getSkyValue (rayleighMieInfo.lightDir.xyz + toSunUp * float(i) * 0.1 + toSunSide * float(j) * 0.1);
	return retVal * 0.111111111;
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
	/*vec3 tracePos = rayOrigin + originNormal * VOXEL_DIM_SIZE * 1.732;
	uint pickAHint = uint (float (NEEData.hintCountReserved.x) * ((randVal.z + 1.0) * 0.4999999));
	NEEHint pickedHint = NEEData.hint[pickAHint];
    vec3 traceDir = normalize (pickedHint.posArea.xyz - rayOrigin);
	float cosOutgoing = max (dot (traceDir, originNormal), 0.0);
	if ( cosOutgoing == 0.0 ) return vec3 (0.0);
	float cosIncoming = max (dot (-traceDir, pickedHint.norm.xyz), 0.0);
	if ( cosIncoming == 0.0 ) return vec3 (0.0);

	for (int i = 0; i != 1000; i++)
	{
		vec3 curUVW = getUVW (tracePos);
		if ( curUVW == vec3 (2.0) )
		{
			return vec3 (0.0);
		}
		VoxelHitData hitMaterial = getMaterial (curUVW);
		if ( hitMaterial.albedo.a > 0.0 )
		{
			vec3 closenessToHintVec = pickedHint.posArea.xyz - tracePos;
			if ( hitMaterial.emissivity.x > 0.0 && dot (closenessToHintVec,closenessToHintVec) < VOXEL_DIM_SIZE*VOXEL_DIM_SIZE*3.0 ) // Make sure we actually hit the hint...
			{
				vec3 distToOrigin = pickedHint.posArea.xyz - rayOrigin;
				float GeomFactor = (cosIncoming * cosOutgoing) / dot (distToOrigin, distToOrigin);
				return (float (NEEData.hintCountReserved.x) / float(HIGHOMEGA_GENERATE_HINTS_PER_LUMINARE)) * hitMaterial.emissivity.x * hitMaterial.albedo.rgb * GeomFactor;
			}
			else
				return vec3 (0.0);
		}
		tracePos += traceDir * VOXEL_DIM_SIZE;
	}*/

	return vec3 (0.0);
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

vec3 getContribDiffuseOnly(vec3 rayOrigin, vec3 rayDir, out vec3 bounceDir, vec2 fragRoughness, float fragSpecularity, float refractiveIndex, bool isDielectric, mat3 tangentSpace, vec3 fragGeomNorm, inout vec3 hitPoint)
{
	vec3 contrib = vec3 (1.0);
	vec3 explicitContrib = vec3 (0.0);
	float bounceCount = 0.0;
	float maxBounce = 3.0;
	vec3 sunRadFetch = getSkyDirectLight ();

	vec3 tracePos = rayOrigin;
	vec3 traceDir = rayDir;
	vec3 moveDir = fragGeomNorm;
	vec3 lastVertexNormal = tangentSpace[2];
	vec3 lastVertexPos = tracePos;
	bool lastVertexDiffuse = false;
	vec3 randVal = vec3 (0.0);

	vec2 blueNoiseLoc = vec2 (int(gl_FragCoord.x) % textureSize (blueNoise, 0).x, int(gl_FragCoord.y) % textureSize (blueNoise, 0).y) / vec2 (textureSize (blueNoise, 0).xy);
	blueNoiseLoc += getRand (rayOrigin * pathTraceParams.timeTurnBlurDirectionRawLight.x).xy * 0.33333;
	ivec2 blueNoiseTexelLoc = ivec2 (clamp (fract(abs(blueNoiseLoc)), vec2 (0.0), vec2 (0.999999)) * vec2 (textureSize (blueNoise, 0).xy));
	vec4 blueNoiseFetch = texelFetch (blueNoise, blueNoiseTexelLoc, 0);
	randVal = blueNoiseFetch.xyz * 2.0 - vec3 (1.0);

	// Diffuse only
	bool glossOrTransmitBounce = false;
	lastVertexDiffuse = true;
	traceDir = diffuseBounce (tangentSpace, randVal);
	bounceDir = traceDir;
	explicitContrib += getExplicitContrib (lastVertexPos, lastVertexNormal, randVal);

	tracePos += moveDir * VOXEL_DIM_SIZE * 1.732;
	
	uint infLoopGuard = 0; // Radeon for some reason really needs this...

	while (bounceCount < maxBounce)
	{
		vec3 curUVW = getUVW (tracePos);
		if ( curUVW == vec3 (2.0) )
		{
			// Store to surface cache
			if ( lastVertexDiffuse ) addDiffuseRadiance (getSkyValue (traceDir), lastVertexPos, traceDir);
			return contrib * getSkyValue (traceDir) + explicitContrib;
		}
		VoxelHitData hitMaterial = getMaterial (curUVW);
		if ( hitMaterial.albedo.a != 0.0 )
		{
			if ( bounceCount == 0 ) hitPoint = tracePos;
			if ( hitMaterial.emissivity > 0.0 )
			{
				// Store to surface cache
				if ( lastVertexDiffuse ) addDiffuseRadiance (hitMaterial.albedo.rgb * hitMaterial.emissivity, lastVertexPos, traceDir);
				return contrib * hitMaterial.albedo.rgb * hitMaterial.emissivity + explicitContrib;
			}
			else
			{
				randVal = getRand(tracePos);
				moveDir = hitMaterial.tangentSpace[2];
				vec3 curModColor = hitMaterial.albedo.rgb;
				lastVertexDiffuse = false;
				if ( !hitMaterial.diElectric )
				{
					if ( hitMaterial.specularity * schlicks (1.0, hitMaterial.IoR, hitMaterial.tangentSpace[2], traceDir) >= max ((randVal.z + 1.0) * 0.5, 0.000001) )
					{
						curModColor = hitMaterial.specular.rgb;
						traceDir = reflectRay (traceDir, hitMaterial.tangentSpace, hitMaterial.roughness, randVal);
					}
					else
					{
						lastVertexNormal = moveDir;
						lastVertexPos = tracePos;
						lastVertexDiffuse = true;
						traceDir = diffuseBounce (hitMaterial.tangentSpace, randVal);
						explicitContrib += contrib * curModColor * getExplicitContrib (lastVertexPos, lastVertexNormal, randVal);
						explicitContrib += contrib * curModColor * getSunContrib (lastVertexPos + lastVertexNormal * VOXEL_DIM_SIZE * 1.732, lastVertexNormal);
					}
				}
				else
				{
					bool TIR = false;
					float hitAngle = dot (traceDir, hitMaterial.tangentSpace[2]);
					if ( hitAngle < 0.0 )
					{
						if ( schlicks (1.0, hitMaterial.IoR, hitMaterial.tangentSpace[2], traceDir) >= max ((randVal.z + 1.0) * 0.5, 0.000001) )
						{
							traceDir = reflectRay (traceDir, hitMaterial.tangentSpace, hitMaterial.roughness, randVal);
						}
						else
						{
							moveDir = -hitMaterial.tangentSpace[2];
							traceDir = refractRay (traceDir, hitMaterial.tangentSpace, hitMaterial.roughness, 1.0, hitMaterial.IoR, TIR, randVal);
						}
					}
					else
					{
						hitMaterial.tangentSpace[2] = -hitMaterial.tangentSpace[2];
						vec3 traceDirRefracted = refractRay (traceDir, hitMaterial.tangentSpace, hitMaterial.roughness, hitMaterial.IoR, 1.0, TIR, randVal);
						if ( TIR )
						{
							moveDir = -hitMaterial.tangentSpace[2];
							traceDir = traceDirRefracted;
						}
						else
						{
							if ( schlicks (hitMaterial.IoR, 1.0, hitMaterial.tangentSpace[2], traceDir) >= max ((randVal.z + 1.0) * 0.5, 0.000001) )
							{
								moveDir = -hitMaterial.tangentSpace[2];
								traceDir = reflectRay (traceDir, hitMaterial.tangentSpace, hitMaterial.roughness, randVal);
							}
							else
							{
								traceDir = traceDirRefracted;
							}
						}
					}
					bounceCount -= 1.0;
				}
				tracePos += moveDir * VOXEL_DIM_SIZE * 1.732;
				contrib *= curModColor;
				bounceCount += 1.0;
			}
		}
		tracePos += traceDir * VOXEL_DIM_SIZE;
		infLoopGuard++;
		if ( infLoopGuard > 1000 ) break;
	}

	if ( lastVertexDiffuse ) addDiffuseRadiance (vec3 (0.0), lastVertexPos, traceDir);
    return explicitContrib;
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

void main()
{
	ivec2 texelLoc = ivec2 (inUV * vec2 (textureSize(materialAttach, 0)));

	vec4 fetchedWorldPosAndMaterialHint = texelFetch(worldPosAttach, texelLoc, 0);
	if (fetchedWorldPosAndMaterialHint == vec4 (0.0))
	{
		return ;
	}
	vec4 materialFetch = texelFetch(materialAttach, texelLoc, 0);
	bool isDielectric;
	vec2 emissivityRefractiveIndex = unpackEmissivityRefractiveIndexDielectric (materialFetch.w, isDielectric);

	if ( emissivityRefractiveIndex.x > 0.0 )
	{
		return ;
	}
	
	vec3 rayOrigin = fetchedWorldPosAndMaterialHint.xyz;
	vec3 rayDir = normalize (fetchedWorldPosAndMaterialHint.xyz - vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a));
	
	vec4 fragmentRoughnessSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.z));
	
	vec4 normalFetch = texelFetch(normalAttach, texelLoc, 0);

	mat3 tangentSpace;
	tangentSpace[2] = toVec3 (unpackSpherical (normalFetch.x));
	vec3 fragmentGeomNorm = toVec3 (unpackSpherical (normalFetch.y));
	tangentSpace[0] = toVec3 (unpackSpherical (normalFetch.z));
	tangentSpace[1] = toVec3 (unpackSpherical (normalFetch.w));

	tangentSpace[2] = -sign (dot (rayDir, tangentSpace[2])) * tangentSpace[2];
	fragmentGeomNorm = -sign (dot (rayDir, fragmentGeomNorm)) * fragmentGeomNorm;

	vec3 hitPoint = vec3 (0.0);

	vec3 diffuseContrib = vec3 (0.0);
	vec3 glossContrib = vec3 (0.0);

	if ( fragmentRoughnessSpecularity.b < 1.0 )
	{
		vec3 outDir;
		diffuseContrib = getContribDiffuseOnly (rayOrigin, rayDir, outDir, fragmentRoughnessSpecularity.rg, fragmentRoughnessSpecularity.b, emissivityRefractiveIndex.y, isDielectric, tangentSpace, fragmentGeomNorm, hitPoint);
		// Store to surface cache
		addDiffuseRadiance (diffuseContrib, rayOrigin, outDir);
	}
}