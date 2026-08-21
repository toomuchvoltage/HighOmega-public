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

layout (scalar, binding = 1) buffer DofPropsUBO
{
	float invDimsX;
	float invDimsY;
	float blurDirection;
	float invCoCDist;
	vec4 screenMidPos;
	vec4 toScreenMidPosAlpha;
	float midScreenAlpha;
} dofProps;

layout (binding = 2, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 3) uniform sampler2D inpImage;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 diffOutput;

void applyFinalPost ()
{
	if (dofProps.blurDirection == 1.0)
	{
		diffOutput *= dofProps.toScreenMidPosAlpha.a;
	}
}

void main()
{
	vec3 screenMidPos = imageLoad (worldPosAttach, ivec2 (imageSize(worldPosAttach) >> 1)).rgb;
	if ( screenMidPos == vec3 (0.0) ) screenMidPos = vec3 (1000000.0);
	dofProps.screenMidPos = vec4 (screenMidPos, 1.0);

	if ( dofProps.toScreenMidPosAlpha.xyz == vec3 (0.0) ) dofProps.toScreenMidPosAlpha.xyz = dofProps.screenMidPos.xyz;

	vec3 midPos = imageLoad (worldPosAttach, ivec2 (inUV * imageSize(worldPosAttach))).rgb;
	if ( midPos == vec3 (0.0) ) midPos = vec3 (1000000.0);
	float blurSize = floor (min (length (dofProps.toScreenMidPosAlpha.xyz - midPos) * dofProps.invCoCDist, 20.0));
	if (blurSize == 0.0)
	{
		diffOutput = texelFetch (inpImage, ivec2 (inUV * textureSize(inpImage, 0)), 0);
		applyFinalPost ();
		return ;
	}

	vec2 UVDir = vec2 (0.0);
	if (dofProps.blurDirection == 0.0)
		UVDir = vec2 (dofProps.invDimsX, 0.0);
	else
		UVDir = vec2 (0.0, dofProps.invDimsY);

	float invBlurTotalSize = 1.0 / (blurSize * 2.0);

	vec4 accumSamples = vec4 (0.0);
	float accumWeights = 0.0;
	for (int i = -int(blurSize);i != int(blurSize)+1;i++)
	{
		vec2 curUV = clamp (inUV + float(i) * UVDir, vec2 (0.0), vec2 (1.0));
		float xDiff = ((float(i) + blurSize) * invBlurTotalSize) - 0.5;
		float weight = exp (-5.0 * (xDiff * xDiff));
		accumSamples += weight * texelFetch (inpImage, ivec2 (curUV * textureSize(inpImage, 0)), 0);
		accumWeights += weight;
	}

	if ( accumWeights == 0.0 )
		diffOutput = texelFetch (inpImage, ivec2 (inUV * textureSize(inpImage, 0)), 0);
	else
		diffOutput = accumSamples / accumWeights;

	applyFinalPost ();
}