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

layout (binding = 1) uniform sampler2D inpImage;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 diffOutput;

layout (constant_id = 0) const float xDir = 1.0;
layout (constant_id = 1) const float yDir = 1.0;
layout (constant_id = 2) const int blurSize = 4;

void main()
{
	vec2 UVDir = vec2 (xDir, yDir);

	float invBlurTotalSize = 1.0 / (float(blurSize) * 2.0);

	vec4 accumSamples = vec4 (0.0);
	float accumWeights = 0.0;
	for (int i = -blurSize;i != blurSize+1;i++)
	{
		vec2 curUV = clamp (inUV + float(i) * UVDir, vec2 (0.0), vec2 (1.0));
		float xDiff = float(abs(i)) * invBlurTotalSize;
		float weight = exp (-5.0 * (xDiff * xDiff));
		accumSamples += weight * texture (inpImage, curUV);
		accumWeights += weight;
	}

	diffOutput = accumSamples / accumWeights;
}