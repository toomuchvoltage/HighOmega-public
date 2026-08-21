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
layout (binding = 1) uniform sampler2D ssfxAttach;
layout (binding = 2, r32ui) uniform uimage2D velocityAttach;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outFragColor;

void main()
{
	uint velRead = imageLoad (velocityAttach, ivec2 (gl_FragCoord.xy)).x;
	if (velRead == 0xFFFFFFFFu)
	{
		outFragColor = texture (ssfxAttach, inUV);
		return ;
	}
	vec2 velocity;
	velocity.x = int((velRead & 0x0000FF00u) >> 8u) - 127;
	velocity.y = int(velRead &  0x000000FFu) - 127;

	float velocityLen = length(velocity);
	if ( velocityLen < 1.0 )
	{
		outFragColor = texture (ssfxAttach, inUV);
		return ;
	}
	vec2 velocityNorm = velocity / velocityLen;
	vec4 accumOut = vec4 (0.0);
	float accumSamples = 0.0;
	
	for (int i = -int(velocityLen * 0.5); i != int(velocityLen * 0.5) + 1; i++)
	{
		vec2 curSampleLoc = floor(gl_FragCoord.xy) + float (i) * velocityNorm;
		vec2 curSampleLocClamped = clamp (curSampleLoc, vec2 (0.0), vec2 (textureSize(ssfxAttach, 0).xy - ivec2(1)));
		if ( curSampleLoc != curSampleLocClamped ) continue;
		if ( imageLoad (velocityAttach, ivec2 (curSampleLoc)).x == 0xFFFFFFFFu ) continue;
		accumOut += texelFetch (ssfxAttach, ivec2 (curSampleLoc), 0);
		accumSamples += 1.0;
	}
	
	outFragColor = accumOut / accumSamples;
}