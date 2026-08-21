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

#define M_PI 3.1415926535

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (binding = 1) uniform sampler2D lightAttach;
layout (binding = 2, rgba32f) uniform readonly image2D materialAttach;
layout (binding = 3, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 4, rgba32f) uniform readonly image2D normAttach;

layout (binding = 5) uniform PathTraceParamsUBO
{
	vec4 radiosityMapCenterMotionFactor;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outFragColorGloss;

layout (constant_id = 0) const float xDir = 1.0;
layout (constant_id = 1) const float yDir = 1.0;

void unpackEmissivityRefractiveIndexDielectric (float packedVal, out float emissivity, out float refractiveIndex, out bool diElectric)
{
	vec4 unpackedVal = unpackUnorm4x8 (floatBitsToUint (packedVal));
	emissivity = unpackedVal.x * 25.5;
	refractiveIndex = unpackedVal.y * 4.0;
	diElectric = unpackedVal.z == 1.0 ? true : false;
}

float gaussianWeight(int kernelWidth, int offset)
{
	float gaussTop = float(offset) / float(kernelWidth);
	return exp(-5.0 * gaussTop * gaussTop);
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
	vec2 outputSize = vec2 (textureSize(lightAttach, 0));
	vec2 gatherSize = vec2 (imageSize(materialAttach));
	ivec2 texelLoc = ivec2 (inUV * gatherSize);
	vec4 centerSampleGloss = texelFetch (lightAttach, ivec2 (gl_FragCoord.xy), 0);
	vec4 materialFetch = imageLoad (materialAttach, texelLoc);

	float emissivity, refractiveIndex;
	bool diElectric;
	unpackEmissivityRefractiveIndexDielectric (materialFetch.w, emissivity, refractiveIndex, diElectric);

	if ( emissivity > 0.0 )
	{
		outFragColorGloss = vec4 (0.0);
		return ;
	}

	vec3 roughnessAndSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.b)).xyz;
	float roughnessMeasure = sqrt(max (roughnessAndSpecularity.x, roughnessAndSpecularity.y));
	if ( pathTraceParams.timeTurnBlurDirectionRawLight.a == 1.0 || roughnessMeasure == 0.0 )
	{
		outFragColorGloss = centerSampleGloss;
		return ;
	}
	
	vec2 UVDir = vec2 (xDir, yDir);

	vec4 fetchedPosAndMaterialHint = imageLoad(worldPosAttach, texelLoc);
	vec3 fetchedNorm = fromZSignXY(floatBitsToUint (imageLoad(normAttach, texelLoc).x));

	vec4 glossAccum = centerSampleGloss;
	float glossWeightAccum = 1.0;
	vec3 toEye = vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a) - fetchedPosAndMaterialHint.xyz;
	int kernelWidth = int (mix (5.0, 15.0, roughnessMeasure));
	float radialCutoff = max (dot (toEye, toEye) * 0.001111, max (roughnessMeasure * 25.0, 1.0));
	for (int i = -kernelWidth; i != kernelWidth+1; i++)
	{
		vec2 curUV = inUV + float(i) * UVDir;
		if ( i == 0 || curUV != clamp (curUV,vec2(0.0),vec2(0.999999)) ) continue;
		ivec2 curTexelLoc = ivec2 (curUV * gatherSize);
		vec4 curWorldPosAndMaterialHint = imageLoad(worldPosAttach, curTexelLoc);
		vec3 curNormal = fromZSignXY (floatBitsToUint (imageLoad(normAttach, curTexelLoc).x));
		vec3 diffVec = fetchedPosAndMaterialHint.xyz - curWorldPosAndMaterialHint.xyz;
		if ( curWorldPosAndMaterialHint.a != fetchedPosAndMaterialHint.a || dot(diffVec, diffVec) > radialCutoff || dot (curNormal, fetchedNorm) < (0.9 - roughnessMeasure * 0.1) ) continue;
		ivec2 curTexelLocDownSample = ivec2 (curUV * outputSize);
		float curWeight = gaussianWeight (kernelWidth, abs(i));
		glossAccum.rgb += texelFetch(lightAttach, curTexelLocDownSample, 0).rgb * curWeight;
		glossWeightAccum += curWeight;
	}

	outFragColorGloss = vec4 (glossAccum.rgb / glossWeightAccum, centerSampleGloss.a);
}