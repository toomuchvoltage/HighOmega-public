/*
	Copyright (c) 2023 TooMuchVoltage Software Inc.

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

layout (location = 0) in vec4 inPosCol;
layout (location = 1) in vec2 inUV;
layout (location = 2) in uint inNorm;

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (location = 0) out vec2 outUV;
layout (location = 1) out vec3 outPos;
layout (location = 2) flat out uint outInstanceIndex;

out gl_PerVertex 
{
	vec4 gl_Position;
};

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

void unpackRasterVertex(out vec3 vPos, out vec3 vCol, out vec2 vUV, out vec3 vNorm, in vec4 rv1, in vec2 rv2, in uint rv3)
{
	vPos = rv1.xyz;
	vCol = unpackUnorm4x8(floatBitsToUint(rv1.w)).xyz;

	vUV = rv2;
	vNorm = fromZSignXY(rv3);
}

void main() 
{
	vec3 vColor, vNorm;
	unpackRasterVertex (outPos, vColor, outUV, vNorm, inPosCol, inUV, inNorm);
	gl_Position = frameMVP.projectionViewMatrix * vec4(outPos, 1.0);
	outInstanceIndex = gl_InstanceIndex;
}
