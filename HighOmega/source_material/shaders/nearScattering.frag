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
#extension GL_EXT_control_flow_attributes : enable
#extension GL_EXT_scalar_block_layout : require

#define Inv4Pi 0.0795774715

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (binding = 1) uniform RayleighMieUBO
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

layout (binding = 2) uniform NearScatteringParamsUBO
{
	vec4 amountExtinction;
} nearScatteringParams;

layout (binding = 3) uniform sampler2D shadowDistanceNear;
layout (binding = 4) uniform sampler2D shadowColorNear;

layout (scalar, binding = 5) uniform SkyShadowMapMVPNearUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} SkyShadowMapMVPNear;

layout (binding = 6) uniform sampler2D shadowDistanceFar;
layout (binding = 7) uniform sampler2D shadowColorFar;

layout (scalar, binding = 8) uniform SkyShadowMapMVPFarUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} SkyShadowMapMVPFar;

layout (binding = 9, rgba32f) uniform readonly image2D worldPosAndEmissivityAttach; 
layout (binding = 10) uniform sampler2D screenSpaceFXGatherWorldPos;
layout (binding = 11) uniform samplerCube skyBox;
layout (binding = 12) uniform sampler2D blueNoise;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 diffOutput;

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

vec3 sampleShadow (vec3 samplePos)
{
	vec4 shadowUVFetch = SkyShadowMapMVPNear.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPNear.lookEyeX.a, SkyShadowMapMVPNear.upEyeY.a, SkyShadowMapMVPNear.sideEyeZ.a),1.0);
	shadowUVFetch.xyz /= shadowUVFetch.w;
	shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;
	vec3 shadowResult = vec3 (1.0);

	if ( shadowUVFetch.xy == clamp (shadowUVFetch.xy, vec2 (0.0), vec2 (1.0)) )
	{
		ivec2 readShadowLoc = ivec2 (shadowUVFetch.xy * vec2 (textureSize(shadowDistanceNear, 0).xy));
		float shadowDist = texelFetch (shadowDistanceNear, readShadowLoc, 0).x;
		shadowResult *= max (sign (shadowDist - shadowUVFetch.z), 0.0);
		vec4 colorFetch = texture (shadowColorNear, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}
	else
	{
		shadowUVFetch = SkyShadowMapMVPFar.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPFar.lookEyeX.a, SkyShadowMapMVPFar.upEyeY.a, SkyShadowMapMVPFar.sideEyeZ.a),1.0);
		shadowUVFetch.xyz /= shadowUVFetch.w;
		shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;

		ivec2 readShadowLoc = ivec2 (shadowUVFetch.xy * vec2 (textureSize(shadowDistanceFar, 0).xy));
		float shadowDist = texelFetch (shadowDistanceFar, readShadowLoc, 0).x;
		shadowResult *= max (sign (shadowDist - shadowUVFetch.z), 0.0);
		vec4 colorFetch = texture (shadowColorFar, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}

	return shadowResult;
}

float PHG (float g, float cosTheta)
{
	float gSq = g * g;
	float denomPreMul = 1 + gSq - (2.0 * g * cosTheta);
	return (1 - gSq) * Inv4Pi * inversesqrt(max (abs(denomPreMul * denomPreMul * denomPreMul), 0.00001));
}

float miePhase (float cosTheta)
{
	return mix (PHG (0.7, cosTheta), PHG (-0.2, cosTheta), 0.85);
}

void main()
{
	if ( nearScatteringParams.amountExtinction.x == 0.0f )
	{
		diffOutput = vec4 (0.0, 0.0, 0.0, 1.0);
		return;
	}
	
	vec2 screenSize = vec2 (imageSize (worldPosAndEmissivityAttach));
	vec3 basePos = imageLoad (worldPosAndEmissivityAttach, ivec2 (inUV * screenSize)).xyz;
	vec3 basePosSSFXGather = texelFetch (screenSpaceFXGatherWorldPos, ivec2 (inUV * screenSize), 0).xyz;
	vec3 eyeLoc = vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a);
	vec3 diff1 = basePos - eyeLoc;
	vec3 diff2 = basePosSSFXGather - eyeLoc;
	if ( basePos == vec3 (0.0) || (basePosSSFXGather != vec3 (0.0) && dot(diff2, diff2) < dot(diff1, diff1)) ) basePos = basePosSSFXGather;

	if ( basePos == vec3 (0.0) )
	{
		vec2 uvDeNorm = inUV * 2.0 - vec2 (1.0);
		basePos = eyeLoc + (frameMVP.lookEyeX.xyz - uvDeNorm.y * frameMVP.upEyeY.xyz * frameMVP.whrTanHalfFovY.y - uvDeNorm.x * frameMVP.sideEyeZ.xyz * frameMVP.whrTanHalfFovY.y * frameMVP.whrTanHalfFovY.x) * 1000.0;
	}
	
	vec3 retVal = vec3 (0.0);
	vec3 traceDir = basePos - eyeLoc;
	float maxDist = length (traceDir);
	vec3 traceDirStep = traceDir / max (maxDist, 0.00001);
	vec3 curLoc = eyeLoc;
	float scatter = 1.0;
	float randFetch = texelFetch (blueNoise, ivec2 (ivec2 (gl_FragCoord.xy) % textureSize (blueNoise, 0).xy), 0).r * 2.0 - 1.0;
	float traceLen = 1.0 + randFetch * 0.2;
	vec3 curStep = traceDirStep * traceLen;
	float curTravel = 0.0;
	vec3 skyVal = miePhase (dot (traceDirStep, -SkyShadowMapMVPNear.lookEyeX.xyz)) * getSkyDirectLight () * nearScatteringParams.amountExtinction.x;
	[[unroll]]
	for (int i = 0; i != 1000; i++)
	{
		curLoc += curStep;
		curTravel += traceLen;
		if ( curTravel > maxDist ) break;
		vec3 sunContrib = sampleShadow (curLoc);
		retVal += sunContrib * skyVal * scatter;
		if (sunContrib != vec3 (0.0)) scatter *= nearScatteringParams.amountExtinction.y;
		if ( scatter < 0.1 ) break;
	}
	
	diffOutput = vec4 (pow (retVal, vec3 (1.0/2.2)), scatter);
}