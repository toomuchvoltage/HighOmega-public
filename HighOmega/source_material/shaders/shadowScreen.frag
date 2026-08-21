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
#extension GL_EXT_scalar_block_layout : require

layout (binding = 1, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 2, rgba32f) uniform readonly image2D normAttach;
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

layout (binding = 9, rgba8) uniform writeonly image2D shadowMapScreen;

layout (constant_id = 0) const float nearMinBias = 0.00001;
layout (constant_id = 1) const float nearMaxBias = 0.00002;
layout (constant_id = 2) const float farMinBias = 0.001;
layout (constant_id = 3) const float farMaxBias = 0.002;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

vec3 sampleShadow (vec3 samplePos, vec3 sampleNorm)
{
	vec4 shadowUVFetch = SkyShadowMapMVPNear.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPNear.lookEyeX.a, SkyShadowMapMVPNear.upEyeY.a, SkyShadowMapMVPNear.sideEyeZ.a),1.0);
	shadowUVFetch.xyz /= shadowUVFetch.w;
	shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;
	vec3 shadowResult = vec3 (1.0);

	if ( shadowUVFetch.xy == clamp (shadowUVFetch.xy, vec2 (0.0), vec2 (1.0)) )
	{
		ivec2 readShadowLoc = ivec2 (shadowUVFetch.xy * vec2 (textureSize(shadowDistanceNear, 0).xy));
		float shadowDist = texelFetch (shadowDistanceNear, readShadowLoc, 0).x;
		shadowResult *= max (sign ((shadowDist + mix (nearMaxBias, nearMinBias, abs(dot (sampleNorm, SkyShadowMapMVPNear.lookEyeX.xyz)))) - shadowUVFetch.z), 0.0);
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
		shadowResult *= max (sign ((shadowDist + mix (farMaxBias, farMinBias, abs(dot (sampleNorm, SkyShadowMapMVPNear.lookEyeX.xyz)))) - shadowUVFetch.z), 0.0);
		vec4 colorFetch = texture (shadowColorFar, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}

	return shadowResult;
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
	ivec2 uvTexel = ivec2 (inUV * vec2 (imageSize(worldPosAttach)));
	vec4 fetchedWorldPos = imageLoad(worldPosAttach, uvTexel);

	if ( fetchedWorldPos == vec4 (0.0) )
	{
		imageStore (shadowMapScreen, ivec2 (gl_FragCoord.xy), vec4 (0.0));
		return ;
	}

	vec3 fetchedNormal = fromZSignXY (floatBitsToUint (imageLoad(normAttach, uvTexel).x));

	imageStore (shadowMapScreen, ivec2 (gl_FragCoord.xy), vec4 (sampleShadow (fetchedWorldPos.xyz, fetchedNormal), 1.0));
}