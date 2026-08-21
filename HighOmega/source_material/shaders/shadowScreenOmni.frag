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
#extension GL_EXT_scalar_block_layout : require

layout (binding = 1, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 2, rgba32f) uniform readonly image2D normAttach;
layout (binding = 3) uniform samplerCube shadowDistance;
layout (binding = 4) uniform samplerCube shadowColor;

layout (scalar, binding = 5) uniform SkyShadowMapMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} ShadowMapMVP[6];

layout (binding = 6, rgba8) uniform writeonly image2D shadowMapScreen;

layout (constant_id = 0) const float minBias = 0.00001;
layout (constant_id = 1) const float maxBias = 0.00002;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

vec3 sampleShadow (vec3 samplePos, vec3 sampleNorm)
{
	vec3 eyeLoc = vec3 (ShadowMapMVP[0].lookEyeX.a, ShadowMapMVP[0].upEyeY.a, ShadowMapMVP[0].sideEyeZ.a);
	vec3 sampleDir = normalize (samplePos - eyeLoc);
	sampleDir.z = -sampleDir.z;
	vec3 sampleDirAbs = abs(sampleDir);
	uint cubeFaceIdx = 0;
	if (sampleDirAbs.x >= sampleDirAbs.y && sampleDirAbs.x >= sampleDirAbs.z)
	{
		if (sampleDir.x >= 0.0)
			cubeFaceIdx = 0;
		else
			cubeFaceIdx = 1;
	}
	else if (sampleDirAbs.y >= sampleDirAbs.x && sampleDirAbs.y >= sampleDirAbs.z)
	{
		if (sampleDir.y >= 0.0)
			cubeFaceIdx = 2;
		else
			cubeFaceIdx = 3;
	}
	else
	{
		if (sampleDir.z >= 0.0)
			cubeFaceIdx = 4;
		else
			cubeFaceIdx = 5;
	}
	vec4 shadowProjResult = ShadowMapMVP[nonuniformEXT(cubeFaceIdx)].projectionViewMatrix * vec4 (samplePos - eyeLoc,1.0);
	shadowProjResult.z /= shadowProjResult.w;
	vec3 shadowResult = vec3 (1.0);

	float shadowDist = texture (shadowDistance, sampleDir).x;
	shadowResult *= max (sign ((shadowDist + mix (maxBias, minBias, abs(dot (sampleNorm, ShadowMapMVP[nonuniformEXT(cubeFaceIdx)].lookEyeX.xyz)))) - shadowProjResult.z), 0.0);
	vec4 colorFetch = texture (shadowColor, sampleDir);
	if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowProjResult.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;

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