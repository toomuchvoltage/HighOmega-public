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

#define M_PI 3.1415926535

#define curDiffuseSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].diffOffset))]
#define curNormalSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].nrmOffset))]
#define curRoughnessAndSpecSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].rghOffset))]
#define curHeightmapSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].hgtOffset))]
#define curSpecularSampler curSceneTextures[nonuniformEXT(uint(instanceInfo.props[inInstanceIndex].spcOffset))]
#define instanceFlags floatBitsToUint (instanceInfo.props[inInstanceIndex].attribs1.x)

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

layout (binding = 1) uniform sampler2D visBufDepthStencil;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inVNorm;
layout (location = 3) in vec3 inTangent;
layout (location = 4) flat in uint inInstanceIndex;
layout (location = 5) in vec3 inVertexVelocity;

layout (location = 0) out vec4 worldPosAlbedoOutput;
layout (location = 1) out vec4 normInstIDVelocityRoughnessOutput;

bool getHasNrmMap (uint packedFlags)
{
	return (packedFlags & 0x00000001) != 0;
}

bool getHasRghMap (uint packedFlags)
{
	return (packedFlags & 0x00000002) != 0;
}

vec4 sampleRoughness ()
{
	return texture (curRoughnessAndSpecSampler, vec3 (inUV, 0.0));
}

vec3 sampleNormal ()
{
	return 2.0 * texture (curNormalSampler, vec3 (inUV, 0.0)).rgb - vec3 (1.0);
}

mat3 reOrthoNormalize (mat3 inpBasis)
{
	mat3 outMat;
	
	outMat[2] = normalize (inpBasis[2]);
	
	vec3 newBiTan = normalize (cross (outMat[2], inpBasis[0]));
	outMat[1] = newBiTan * sign(dot (inpBasis[1], newBiTan));

	vec3 newTan = cross (outMat[2], outMat[1]);
	outMat[0] = newTan * sign(dot (inpBasis[0], newTan));
	
	return outMat;
}

vec3 getFragmentNormal ()
{
	vec3 vertNorm = normalize (inVNorm);
	vec3 curTan = normalize (inTangent);
	vec3 curBiTan = cross (vertNorm, curTan);
	mat3 tanSpace;
	
	tanSpace[2] = vertNorm;
	tanSpace[0] = curTan;
	tanSpace[1] = curBiTan;
	tanSpace = reOrthoNormalize (tanSpace);

	if (getHasNrmMap (instanceFlags))
		return normalize (tanSpace * sampleNormal ());
	else
		return tanSpace[2];
}

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

uint packDeferredVelocity (vec3 velocity)
{
	float velLen = length(velocity);
	if (velLen > 0.0) velocity /= velLen;
	velocity = (velocity + vec3(1.0)) * 0.5;
	return packUnorm4x8 (vec4 (velocity, clamp (velLen / 25.5, 0.0, 1.0)));
}

void main()
{
	float centerDepth = texelFetch (visBufDepthStencil, ivec2(gl_FragCoord.xy), 0).x;
	if ( gl_FragCoord.z < centerDepth ) discard; // visBuf uses reverseZ
	
	vec2 roughnessFetch = vec2 (1.0);
	if (getHasRghMap (instanceFlags)) roughnessFetch = sampleRoughness ().xy;

	worldPosAlbedoOutput = vec4 (inPos, uintBitsToFloat (packUnorm4x8 (texture (curDiffuseSampler, vec3 (inUV, 0.0)))));
	normInstIDVelocityRoughnessOutput = vec4 (uintBitsToFloat (toZSignXY (getFragmentNormal ())), uintBitsToFloat(inInstanceIndex), uintBitsToFloat(packDeferredVelocity(inVertexVelocity)), max (roughnessFetch.x, roughnessFetch.y));
}