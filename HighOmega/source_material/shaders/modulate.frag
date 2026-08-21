/*
	Copyright (c) 2026 TooMuchVoltage Software Inc.

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require
#extension GL_EXT_shader_16bit_storage : require
#extension GL_EXT_scalar_block_layout : require

#define M_PI 3.1415926535
#define HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES 6
#define HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE 32.0

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (binding = 1) uniform sampler2D glossResult;
layout (binding = 2, rgba32f) uniform readonly image2D materialAttach;
layout (binding = 3, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 4, rgba32f) uniform readonly image2D normAttach;

layout (binding = 5) uniform sampler3D radiosityMap[HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES * 6];
layout (binding = 6) uniform PathTraceParamsUBO
{
	vec4 radiosityMapCenterMotionFactor;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;
layout (binding = 7) uniform sampler2D backdrop;
layout (binding = 8) uniform samplerCube skyBox;
layout (binding = 9, rgba8) uniform readonly image2D shadowMapScreen;
layout (binding = 10) uniform RayleighMieUBO
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

struct InstanceProps
{
	vec3 geomMin;
	vec3 geomMax;
    vec3 attribs1;
    vec4 attribs2;
	uint16_t splatOffset;
	uint16_t diffOffset;
	uint16_t nrmOffset;
	uint16_t rghOffset;
	uint16_t hgtOffset;
	uint16_t spcOffset;
	uint idxVertOffset;
	uint transformOffset;
	uint prevTransformOffset;
};

layout (scalar, set = 2, binding = 0) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outFragColor;

float schlicks (float n1, float n2, vec3 norm, vec3 light)
{
	float R0 = (n1 - n2)/(n1 + n2);
	R0 *= R0;
	float _1_minus_dot = 1.0 - min(abs (dot (norm, light)), 1.0);
	_1_minus_dot = _1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot;
	return R0 + (1 - R0) * _1_minus_dot;
}

const float cascadeEdges[HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES] = {4.0, 8.0, 16.0, 32.0, 64.0, 128.0};

vec4 getUVW (vec3 pos)
{
	for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++) {
		vec3 diffVec = pos - (pathTraceParams.radiosityMapCenterMotionFactor.xyz - vec3 (HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE * 0.5 * cascadeEdges[i]));
		vec3 fracAmt = diffVec / (HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE * cascadeEdges[i]);
		vec3 clampFracAmt = clamp(fracAmt, vec3 (0.0), vec3 (1.0));
		if ( clampFracAmt != fracAmt && i < HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES - 1 ) continue;
		return vec4 (clampFracAmt, float (i));
	}
}

vec3 getDiffuseRadiance(vec3 pos, mat3 tangentSpace)
{
	vec4 fetch1, fetch2, fetch3, fetch4, fetch5, fetch6;
	fetch1 = fetch2 = fetch3 = fetch4 = fetch5 = fetch6 = vec4(0.0);
	uint cascadeLevel;
	for (int i = 0; i != 5; i++)
	{
		vec4 rayOriginUVW;
		switch (i)
		{
			case 0:
			 rayOriginUVW = getUVW (pos);
			 break;
			case 1:
			 rayOriginUVW = getUVW (pos - tangentSpace[0] * cascadeEdges[cascadeLevel]);
			 break;
			case 2:
			 rayOriginUVW = getUVW (pos + tangentSpace[0] * cascadeEdges[cascadeLevel]);
			 break;
			case 3:
			 rayOriginUVW = getUVW (pos - tangentSpace[1] * cascadeEdges[cascadeLevel]);
			 break;
			default:
			 rayOriginUVW = getUVW (pos + tangentSpace[1] * cascadeEdges[cascadeLevel]);
			 break;
		}
		cascadeLevel = int(rayOriginUVW.w);
		uint cacheSamplersBaseIndex = cascadeLevel * 6;
		vec4 neighborFetch1 = texture (radiosityMap[nonuniformEXT (cacheSamplersBaseIndex + 0)], rayOriginUVW.xyz);
		if ( neighborFetch1.a > 0.0 ) fetch1 += vec4 (neighborFetch1.rgb / neighborFetch1.a, 1.0);
		vec4 neighborFetch2 = texture (radiosityMap[nonuniformEXT (cacheSamplersBaseIndex + 1)], rayOriginUVW.xyz);
		if ( neighborFetch2.a > 0.0 ) fetch2 += vec4 (neighborFetch2.rgb / neighborFetch2.a, 1.0);
		vec4 neighborFetch3 = texture (radiosityMap[nonuniformEXT (cacheSamplersBaseIndex + 2)], rayOriginUVW.xyz);
		if ( neighborFetch3.a > 0.0 ) fetch3 += vec4 (neighborFetch3.rgb / neighborFetch3.a, 1.0);
		vec4 neighborFetch4 = texture (radiosityMap[nonuniformEXT (cacheSamplersBaseIndex + 3)], rayOriginUVW.xyz);
		if ( neighborFetch4.a > 0.0 ) fetch4 += vec4 (neighborFetch4.rgb / neighborFetch4.a, 1.0);
		vec4 neighborFetch5 = texture (radiosityMap[nonuniformEXT (cacheSamplersBaseIndex + 4)], rayOriginUVW.xyz);
		if ( neighborFetch5.a > 0.0 ) fetch5 += vec4 (neighborFetch5.rgb / neighborFetch5.a, 1.0);
		vec4 neighborFetch6 = texture (radiosityMap[nonuniformEXT (cacheSamplersBaseIndex + 5)], rayOriginUVW.xyz);
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
	vec3 totalContrib = (contrib1 + contrib2 + contrib3 + contrib4 + contrib5 + contrib6) / max(factor1 + factor2 + factor3 + factor4 + factor5 + factor6, 0.0001);
	return totalContrib;
}

void unpackEmissivityRefractiveIndexDielectric (float packedVal, out float emissivity, out float refractiveIndex, out bool diElectric)
{
	vec4 unpackedVal = unpackUnorm4x8 (floatBitsToUint (packedVal));
	emissivity = unpackedVal.x * 25.5;
	refractiveIndex = unpackedVal.y * 4.0;
	diElectric = unpackedVal.z == 1.0 ? true : false;
}

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyBox,dirToSample).xyz;
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
	return retVal * 0.111111111 * rayleighMieInfo.skyObjectLightAndAngle.xyz;
}

bool getDoubleSided (uint packedFlags)
{
	return ((packedFlags & 0x00000008) != 0);
}

vec3 fromZSignXY(uint inpPack)
{
	vec3 retVal;
	retVal.x = (float((inpPack & 0x7FFF0000u) >> 16) / 32766.0) * 2.0 - 1.0;
	retVal.y = (float(inpPack & 0x0000FFFFu) / 65534.0) * 2.0 - 1.0;
	vec2 xyVec = vec2(retVal.x, retVal.y);
	retVal.z = sqrt(clamp (1.0 - dot(xyVec, xyVec), 0.0, 1.0));
	if ((inpPack & 0x80000000u) != 0) retVal.z = -retVal.z;
	return retVal;
}

void main()
{
	vec2 gBufRes = vec2 (imageSize(worldPosAttach));
	vec2 traceRes = vec2 (textureSize(glossResult, 0));
	ivec2 texelLoc = ivec2 (inUV * gBufRes);
	ivec2 lowResTexelLoc = ivec2 (inUV * traceRes);
	ivec2 lowResTexelLocInHiRes = ivec2 (((vec2 (lowResTexelLoc) + vec2 (0.5)) / traceRes) * gBufRes);

	vec4 fetchedWorldPosAndInstID = imageLoad(worldPosAttach, texelLoc);
	if ( fetchedWorldPosAndInstID == vec4 (0.0) )
	{
		outFragColor = texture(backdrop, inUV);
		return ;
	}
	vec4 materialFetch = imageLoad(materialAttach, texelLoc);
	vec4 albedoColor = unpackUnorm4x8(floatBitsToUint (materialFetch.x));

	float emissivity, refractiveIndex;
	bool diElectric;
	unpackEmissivityRefractiveIndexDielectric (materialFetch.a, emissivity, refractiveIndex, diElectric);
	if ( emissivity > 0.0 )
	{
		outFragColor = vec4 (pow (albedoColor.xyz, vec3(1.0/2.2)), 1.0);
		return ;
	}

	vec4 curBasisFetch = imageLoad(normAttach, texelLoc);
	mat3 tanSpace;
	tanSpace[0] = fromZSignXY(floatBitsToUint (curBasisFetch.z));
	tanSpace[1] = fromZSignXY(floatBitsToUint (curBasisFetch.w));
	tanSpace[2] = fromZSignXY(floatBitsToUint (curBasisFetch.x));
	vec3 faceNorm = fromZSignXY(floatBitsToUint (curBasisFetch.y));
	if ( dot(faceNorm, tanSpace[2]) < 0.0 ) tanSpace[2] = -tanSpace[2];
	vec3 totalDiffuseContrib = getDiffuseRadiance (fetchedWorldPosAndInstID.xyz, tanSpace);

	vec4 specularColor = unpackUnorm4x8(floatBitsToUint (materialFetch.y));
	vec4 roughnessSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.z));

	float schlickCompute = schlicks (1.0, refractiveIndex, tanSpace[2], normalize (vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a) - fetchedWorldPosAndInstID.xyz));

	vec3 sunDirectContrib = vec3 (0.0);
	float sunLightDot = dot (tanSpace[2], rayleighMieInfo.lightDir.xyz);
	if ( getDoubleSided (floatBitsToUint (instanceInfo.props[floatBitsToUint(fetchedWorldPosAndInstID.a)].attribs1.x)) ) sunLightDot = abs (sunLightDot);
	else sunLightDot = max (sunLightDot, 0.0);
	vec4 screenShadowMapFetch = imageLoad(shadowMapScreen, texelLoc);
	vec3 sunRadFetch = getSkyDirectLight () * screenShadowMapFetch.rgb;
	sunDirectContrib = sunRadFetch * sunLightDot * (1.0 - schlickCompute * roughnessSpecularity.z);

	vec3 glossFetch = texture(glossResult, inUV).xyz;
	vec3 normFetch = tanSpace[2];
	vec4 lowResPosInstID = imageLoad(worldPosAttach, lowResTexelLocInHiRes);
	vec3 lowResNorm = fromZSignXY(floatBitsToUint (imageLoad(normAttach, lowResTexelLocInHiRes).x));
	vec3 lowResDiff = fetchedWorldPosAndInstID.xyz - lowResPosInstID.xyz;
	float lowResDist = dot (lowResDiff, lowResDiff);
	float normDotProd = abs (dot (normFetch, lowResNorm));
	bool goodOneFound = false;

	if ( floatBitsToUint(lowResPosInstID.a) != floatBitsToUint(fetchedWorldPosAndInstID.a) || lowResDist > 1.0 || normDotProd < 0.9 )
	{
		for (int n = 1; n != 4; n++)
			for (int i = -n; i != n+1; i++)
				for (int j = -n; j != n+1; j++)
				{
					if ( abs (i) <= n-1 && abs (j) <= n-1 ) continue;
					ivec2 curLowResTexelLoc = lowResTexelLoc + ivec2 (i, j);
					curLowResTexelLoc = clamp (curLowResTexelLoc, ivec2(0), ivec2(traceRes - vec2(1.0)));
					ivec2 curHiResTexelLoc = ivec2 (((vec2 (curLowResTexelLoc) + vec2 (0.5)) / traceRes) * gBufRes);
					vec4 sampledPosInstID = imageLoad(worldPosAttach, curHiResTexelLoc);
					if ( floatBitsToUint(sampledPosInstID.a) != floatBitsToUint(fetchedWorldPosAndInstID.a) ) continue;
					vec3 sampledNorm = fromZSignXY(floatBitsToUint (imageLoad(normAttach, curHiResTexelLoc).x));
					float curNormDotProd = dot (sampledNorm, normFetch);
					if ( curNormDotProd < 0.9 ) continue;
					if ( !goodOneFound )
					{
						goodOneFound = true;
						glossFetch = texelFetch(glossResult, curLowResTexelLoc, 0).xyz;
						break;
					}
					vec3 sampledDiff = fetchedWorldPosAndInstID.xyz - sampledPosInstID.xyz;
					float sampledDist = dot (sampledDiff, sampledDiff);
					if ( sampledDist < lowResDist )
					{
						lowResDist = sampledDist;
						glossFetch = texelFetch(glossResult, curLowResTexelLoc, 0).xyz;
					}
				}
	}

	totalDiffuseContrib = (totalDiffuseContrib + sunDirectContrib) * albedoColor.rgb;
	vec3 glossContrib = glossFetch * specularColor.rgb;
	vec3 combinedContrib = mix (totalDiffuseContrib, glossContrib, schlickCompute * roughnessSpecularity.z);
	
	outFragColor = vec4 (pow (combinedContrib, vec3(1.0/2.2)), 1.0);
}