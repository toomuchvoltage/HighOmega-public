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

#define curDiffuseSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].diffOffset))]
#define curNormalSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].nrmOffset))]
#define curRoughnessAndSpecSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].rghOffset))]
#define curHeightmapSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].hgtOffset))]
#define curSpecularSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].spcOffset))]

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

layout (location = 0) in vec2 inUV;
layout (location = 3) flat in uint inInstanceIndex;

layout (location = 0) out vec4 diffOutput;

vec4 sampleDiffuse ()
{
	return texture (curDiffuseSampler, vec3 (inUV, 0.0));
}

void main()
{
	vec4 diffuseFetch = sampleDiffuse();
	if (diffuseFetch.a < 0.99) discard;

	diffOutput = vec4(0.0);
}