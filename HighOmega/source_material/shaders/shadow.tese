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
#extension GL_EXT_shader_16bit_storage : require
#extension GL_EXT_scalar_block_layout : require

#define TessVertexUVOffset instanceInfo.props[tessctrl_InstanceIndex[0]].attribs2.xy
#define TessVertexMaxHeight instanceInfo.props[tessctrl_InstanceIndex[0]].attribs2.z
#define InstanceFlags floatBitsToUint (instanceInfo.props[tessctrl_InstanceIndex[0]].attribs1.x)

#define curDiffuseSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[tessctrl_InstanceIndex[0]].diffOffset))]
#define curNormalSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[tessctrl_InstanceIndex[0]].nrmOffset))]
#define curRoughnessAndSpecSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[tessctrl_InstanceIndex[0]].rghOffset))]
#define curHeightmapSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[tessctrl_InstanceIndex[0]].hgtOffset))]
#define curSpecularSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[tessctrl_InstanceIndex[0]].spcOffset))]

layout(triangles, equal_spacing, cw) in;

layout (location = 0) in vec2 tessctrl_UV[];
layout (location = 1) in vec3 tessctrl_Pos[];
layout (location = 2) in vec3 tessctrl_Norm[];
layout (location = 3) flat in uint tessctrl_InstanceIndex[];

layout (set = 1, binding = 0) uniform sampler2DArray curSceneTextures[];

struct InstanceProps
{
	vec3 geomMin;
	vec3 geomMax;
    vec3 attribs1;
    vec4 attribs2;
	uint16_t splatOffset;
	uint16_t diffOffset;
	uint16_t nrmOffset;
	uint16_t rghOffset;
	uint16_t hgtOffset;
	uint16_t spcOffset;
	uint idxVertOffset;
	uint transformOffset;
	uint prevTransformOffset;
};

layout (scalar, set = 2, binding = 0) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

layout (scalar, binding = 0) uniform UBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (scalar, binding = 1) uniform RenderTimeUBO
{
	float cur;
	float prev;
} renderTime;

layout (location = 0) out vec2 outUV;
layout (location = 3) flat out uint outInstanceIndex;

bool getSmooth (uint packedFlags)
{
	return (packedFlags & 0x00000004) != 0;
}

float sampleHeight (vec2 UV)
{
	return texture (curHeightmapSampler, vec3 (UV, 0.0)).r;
}

void main() 
{
	outUV = gl_TessCoord.x*tessctrl_UV[0] + gl_TessCoord.y*tessctrl_UV[1] + gl_TessCoord.z*tessctrl_UV[2];
	vec3 outPos = gl_TessCoord.x*tessctrl_Pos[0] + gl_TessCoord.y*tessctrl_Pos[1] + gl_TessCoord.z*tessctrl_Pos[2];
	vec3 outNorm = gl_TessCoord.x*tessctrl_Norm[0] + gl_TessCoord.y*tessctrl_Norm[1] + gl_TessCoord.z*tessctrl_Norm[2];
	if ( getSmooth(InstanceFlags) ) outNorm = normalize (outNorm);
	
	vec3 moveOut = outNorm * TessVertexMaxHeight;
	vec2 moveUV = renderTime.cur * TessVertexUVOffset;

	outPos += sampleHeight (outUV + moveUV)*moveOut;

	gl_Position = frameMVP.projectionViewMatrix * vec4(outPos - vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a), 1.0);
}
