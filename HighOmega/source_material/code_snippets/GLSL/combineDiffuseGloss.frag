#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

#define M_PI 3.1415926535
#define VOXEL_DIM_SIZE voxelizedInfo.mapMinVoxelSize.w

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 eye;
} frameMVP;

layout (binding = 1) uniform sampler2D materialAttach;
layout (binding = 2) uniform sampler2D worldPosAttach;
layout (binding = 3) uniform sampler2D normalAttach;
layout (binding = 4) uniform sampler2D tangentAttach;
layout (binding = 5) uniform sampler2D filteredGloss0;
layout (binding = 6) uniform sampler2D filteredGloss1;
layout (binding = 7) uniform sampler2D filteredGloss2;
layout (binding = 8) uniform sampler2D filteredGloss3;
layout (binding = 9) uniform sampler2D filteredGloss4;
layout (binding = 10) uniform sampler3D radiosityMapPX;
layout (binding = 11) uniform sampler3D radiosityMapNX;
layout (binding = 12) uniform sampler3D radiosityMapPY;
layout (binding = 13) uniform sampler3D radiosityMapNY;
layout (binding = 14) uniform sampler3D radiosityMapPZ;
layout (binding = 15) uniform sampler3D radiosityMapNZ;

layout (binding = 16) uniform VoxelizedInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxWriteOffset;
	vec4 invMapDims;
	vec4 gridSize;
} voxelizedInfo;

layout (binding = 17) uniform samplerCube skyBox;
layout (binding = 18) uniform sampler2D backDrop;
layout (binding = 19, rgba8) uniform readonly image2D shadowMapScreen;
layout (binding = 20) uniform RayleighMieUBO
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
layout (binding = 21) uniform sampler2D nearScattering;
layout (binding = 22, rgba16f) uniform writeonly image2D combinedOutput;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
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
	return rayleighMieInfo.skyObjectLightAndAngle.xyz * max (luminance (getSkyValue (rayleighMieInfo.lightDir.xyz) * 0.1) * sqrt (max (rayleighMieInfo.sunDirAndExp.y, 0.0)), 1.0);
}

float schlicks (float n1, float n2, vec3 norm, vec3 light)
{
	float R0 = (n1 - n2)/(n1 + n2);
	R0 *= R0;
	float _1_minus_dot = 1.0 - min(abs (dot (norm, light)), 1.0);
	_1_minus_dot = _1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot;
	return R0 + (1 - R0) * _1_minus_dot;
}

vec3 getUVW (vec3 worldPos)
{
	vec3 posUVW = (worldPos - voxelizedInfo.mapMinVoxelSize.xyz) * voxelizedInfo.invMapDims.xyz;
	if ( posUVW != clamp (posUVW, vec3 (0.0), vec3 (0.999999)) ) return vec3 (2.0);
	return posUVW;
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

vec3 glossValue(ivec2 texelLoc)
{
	vec4 baseGlossFetch = texelFetch (filteredGloss0, texelLoc, 0);
	if ( baseGlossFetch.a < 0.0 ) return baseGlossFetch.rgb;
	
	if ( baseGlossFetch.a < 0.25 )
		return mix (baseGlossFetch.rgb, texture (filteredGloss1, inUV).rgb, baseGlossFetch.a * 4.0);
	else if ( baseGlossFetch.a < 0.5 )
		return mix (texture (filteredGloss1, inUV).rgb, texture (filteredGloss2, inUV).rgb, (baseGlossFetch.a - 0.25) * 4.0);
	else if ( baseGlossFetch.a < 0.75 )
		return mix (texture (filteredGloss2, inUV).rgb, texture (filteredGloss3, inUV).rgb, (baseGlossFetch.a - 0.5) * 4.0);
	else
		return mix (texture (filteredGloss3, inUV).rgb, texture (filteredGloss4, inUV).rgb, (baseGlossFetch.a - 0.75) * 4.0);
}

void main()
{
	ivec2 texelLoc = ivec2 (inUV * vec2 (textureSize(worldPosAttach, 0)));
	vec3 filteredGloss = glossValue (texelLoc);

	vec4 fetchedWorldPosAndMaterialHint = texelFetch(worldPosAttach, texelLoc, 0);
	if ( fetchedWorldPosAndMaterialHint == vec4 (0.0) )
	{
		imageStore (combinedOutput, texelLoc, texture(backDrop, inUV) + texture (nearScattering, inUV));
		return ;
	}
	vec4 materialFetch = texelFetch(materialAttach, texelLoc, 0);
	vec4 albedoColor = unpackUnorm4x8(floatBitsToUint (materialFetch.x));

	bool isDielectric;
	vec3 emissivityRefractiveIndexDielectric;
	emissivityRefractiveIndexDielectric.xy = unpackEmissivityRefractiveIndexDielectric (materialFetch.a, isDielectric);
	if ( emissivityRefractiveIndexDielectric.x > 0.0 )
	{
		imageStore (combinedOutput, texelLoc, vec4 (emissivityRefractiveIndexDielectric.x * albedoColor.xyz, 1.0) + texture (nearScattering, inUV));
		return ;
	}
	
	vec4 tangentSpaceFetch = texelFetch(tangentAttach, texelLoc, 0);
	vec3 normFetch = toVec3(texelFetch (normalAttach, texelLoc, 0).xy);

	mat3 tangentSpace;
	tangentSpace[0] = toVec3 (tangentSpaceFetch.xy);
	tangentSpace[1] = toVec3 (tangentSpaceFetch.zw);
	tangentSpace[2] = normFetch;

	vec4 specularColor = unpackUnorm4x8(floatBitsToUint (materialFetch.y));
	vec4 roughnessSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.z));

	float schlickCompute = schlicks (1.0, emissivityRefractiveIndexDielectric.y, normFetch, normalize (frameMVP.eye.xyz - fetchedWorldPosAndMaterialHint.xyz));

	float sunLightDot = dot (normFetch, rayleighMieInfo.lightDir.xyz);
	if ( fetchedWorldPosAndMaterialHint.a < 0.0 ) sunLightDot = abs(sunLightDot);
	else sunLightDot = max (sunLightDot, 0.0);
	vec4 screenShadowMapFetch = imageLoad(shadowMapScreen, texelLoc);
	vec3 sunRadFetch = getSkyDirectLight () * screenShadowMapFetch.rgb;
	
	vec3 diffuseComponent = (sunRadFetch * sunLightDot + getDiffuseRadiance(fetchedWorldPosAndMaterialHint.xyz, tangentSpace)) * albedoColor.rgb;
	vec3 specularComponent = filteredGloss.rgb * specularColor.rgb;
	
	vec3 outputVal = mix (diffuseComponent, specularComponent, schlickCompute * roughnessSpecularity.z) + texture (nearScattering, inUV).rgb;
	
	imageStore (combinedOutput, texelLoc, vec4 (outputVal, 1.0));
}