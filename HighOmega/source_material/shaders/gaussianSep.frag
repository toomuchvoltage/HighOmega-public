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

layout (binding = 1, rgba32f) uniform readonly image2D worldPosAndEmissivityAttach;
layout (binding = 2, rgba8) uniform readonly image2D shadowMapScreen;
layout (binding = 3, rgba8) uniform writeonly image2D shadowMapScreenOut;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

const float weights[5] = {1.0, 0.92484881321, 0.73161562894, 0.49503589692, 0.28650479686}; // Gaussian: e ^ (-5.0 * ((abs(i) * 0.125)^2))

layout (constant_id = 0) const int xDir = 1;
layout (constant_id = 1) const int yDir = 0;

void main()
{
	vec4 fetchedWorldPosAndEmissivity = imageLoad(worldPosAndEmissivityAttach, ivec2 (gl_FragCoord.xy));
	if ( fetchedWorldPosAndEmissivity == vec4 (0.0) )
	{
		imageStore (shadowMapScreenOut, ivec2 (gl_FragCoord.xy), vec4 (0.0));
		return ;
	}
	vec4 centerSample = imageLoad(shadowMapScreen, ivec2 (gl_FragCoord.xy));
	vec4 accumSamples = centerSample;
	float sampleCount = 1.0;

	ivec2 UVDir = ivec2 (xDir, yDir);

	[[unroll]]
	for (int i = -4;i != 5;i++)
	{
		if ( i == 0 ) continue;
		ivec2 curTexel = ivec2 (gl_FragCoord.xy) + i * UVDir;
		vec4 curWorldPos = imageLoad(worldPosAndEmissivityAttach, curTexel);
		if ( curWorldPos == vec4 (0.0) ) continue;
		vec3 diffWithCenter = curWorldPos.xyz - fetchedWorldPosAndEmissivity.xyz;
		if ( dot (diffWithCenter, diffWithCenter) > 1.0 ) continue;
		sampleCount += weights[abs(i)];
		accumSamples += imageLoad(shadowMapScreen, curTexel) * weights[abs(i)];
	}

	imageStore (shadowMapScreenOut, ivec2 (gl_FragCoord.xy), accumSamples / sampleCount);
}