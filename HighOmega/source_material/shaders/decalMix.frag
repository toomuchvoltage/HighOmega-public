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

layout (binding = 1) uniform sampler2D decalAlbedo;
layout (binding = 2) uniform sampler2D decalSpecular;
layout (binding = 3) uniform sampler2D decalRoughnessSpecularity;
layout (binding = 4) uniform sampler2D decalNormal;
layout (binding = 5) uniform sampler2D decalTangent;
layout (binding = 6) uniform sampler2D decalBiTangent;
layout (binding = 7, rgba32f) uniform image2D materialAttach;
layout (binding = 8, rgba32f) uniform image2D normAttach;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

uint toZSignXY(vec3 inpVec)
{
	uint retVal = ((uint((inpVec.x + 1.0) * 16383.0) << 16) | uint((inpVec.y + 1.0) * 32767.0));
	if (inpVec.z < 0.0) retVal |= 0x80000000u;
	return retVal;
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
	vec4 decalAlbedoFetch = texelFetch (decalAlbedo, ivec2(gl_FragCoord.xy), 0);
	if (decalAlbedoFetch.a > 0.0)
	{
		vec4 decalSpecularFetch = texelFetch (decalSpecular, ivec2(gl_FragCoord.xy), 0);
		vec4 decalRoughnessSpecularityFetch = texelFetch (decalRoughnessSpecularity, ivec2(gl_FragCoord.xy), 0);
		vec3 decalNormalFetch = texelFetch (decalNormal, ivec2(gl_FragCoord.xy), 0).rgb * 2.0 - vec3 (1.0);
		vec3 decalTanFetch = texelFetch (decalTangent, ivec2(gl_FragCoord.xy), 0).rgb * 2.0 - vec3 (1.0);
		vec3 decalBiTanFetch = texelFetch (decalBiTangent, ivec2(gl_FragCoord.xy), 0).rgb * 2.0 - vec3 (1.0);
		
		vec4 matFetch = imageLoad (materialAttach, ivec2(gl_FragCoord.xy));
		vec4 diffFetch = unpackUnorm4x8 (floatBitsToUint(matFetch.x));
		vec4 specFetch = unpackUnorm4x8 (floatBitsToUint(matFetch.y));
		vec4 roughFetch = unpackUnorm4x8 (floatBitsToUint(matFetch.z));

		vec4 basisFetch = imageLoad (normAttach, ivec2(gl_FragCoord.xy));
		vec3 finalNorm = fromZSignXY (floatBitsToUint (basisFetch.x));
		vec3 tanFetch = fromZSignXY (floatBitsToUint (basisFetch.z));
		vec3 biTanFetch = fromZSignXY (floatBitsToUint (basisFetch.w));

		diffFetch = mix (diffFetch, decalAlbedoFetch, decalAlbedoFetch.a);
		specFetch = mix (specFetch, decalSpecularFetch, decalAlbedoFetch.a);
		roughFetch = mix (roughFetch, decalRoughnessSpecularityFetch, decalAlbedoFetch.a);
		finalNorm = mix (finalNorm, decalNormalFetch, decalAlbedoFetch.a);
		tanFetch = mix (tanFetch, decalTanFetch, decalAlbedoFetch.a);
		biTanFetch = mix (biTanFetch, decalBiTanFetch, decalAlbedoFetch.a);

		imageStore (materialAttach, ivec2(gl_FragCoord.xy), vec4 (uintBitsToFloat (packUnorm4x8 (diffFetch)), uintBitsToFloat (packUnorm4x8 (specFetch)), uintBitsToFloat (packUnorm4x8 (roughFetch)), matFetch.w));
		imageStore (normAttach, ivec2(gl_FragCoord.xy), vec4 (uintBitsToFloat (toZSignXY(normalize (finalNorm))), basisFetch.y, uintBitsToFloat (toZSignXY(normalize (tanFetch))), uintBitsToFloat (toZSignXY(normalize (biTanFetch)))));
	}
}