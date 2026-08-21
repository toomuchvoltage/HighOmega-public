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

layout (set = 2, binding = 2) buffer TransformsSSBO
{
	mat4 mats[];
} transforms;

layout (scalar, binding = 1) uniform PrevFrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} prevFrameMVP;

layout (binding = 2, r32ui) uniform uimage2D velocityAttach;
layout (binding = 3) uniform sampler2D visBufDepthStencil;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) flat in uint inInstanceIndex;
layout (location = 3) in float inStrength;
layout (location = 4) in vec3 inVertexVelocity;

layout (location = 0) out vec4 diffOutput;

vec4 sampleDiffuse ()
{
	return texture (curDiffuseSampler, vec3 (inUV, 0.0));
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

vec3 projectCoord (vec3 coord, mat4 MVP, vec3 eyeLoc)
{
	vec4 pCoord = MVP * vec4 (coord - eyeLoc, 1.0);
	pCoord.xyz = pCoord.xyz/pCoord.w;
	pCoord.xy = (pCoord.xy + vec2 (1.0)) * 0.5;
	return pCoord.xyz;
}

bool isViewerRelative (uint packedFlags)
{
	return (packedFlags & 0x00001000) != 0;
}

void main()
{
	float centerDepth = texelFetch (visBufDepthStencil, ivec2(gl_FragCoord.xy), 0).x;
	if ( gl_FragCoord.z < centerDepth ) discard; // visBuf uses reverseZ

	uint packedFlags = floatBitsToUint (instanceInfo.props[inInstanceIndex].attribs1.x);

	vec3 prevPos = inPos - inVertexVelocity;
	bool viewerRelative = isViewerRelative(packedFlags);

	vec4 diffuseFetch = sampleDiffuse();

	vec3 prevProjectedCoord = projectCoord (prevPos, prevFrameMVP.projectionViewMatrix, vec3 (prevFrameMVP.lookEyeX.a, prevFrameMVP.upEyeY.a, prevFrameMVP.sideEyeZ.a));
	vec2 velocity = prevProjectedCoord.xy * vec2 (imageSize (velocityAttach).xy) - floor(gl_FragCoord.xy);
	velocity *= diffuseFetch.a * (inStrength > 0.0 ? inStrength : 1.0) * instanceInfo.props[inInstanceIndex].attribs1.y * 0.01;
	float velLen = length(velocity); vec2 velNorm = velocity / max (velLen, 0.001);
	if (velLen > 127.0) { velocity = velNorm * 127.0; velLen = 127.0; }

	if (viewerRelative)
		imageAtomicExchange (velocityAttach, ivec2(gl_FragCoord.xy), 0xFFFFFFFFu);
	else
	{
		uint velWrite = (((int(velocity.x) + 127) << 8) | (int(velocity.y) + 127));
		uint prevVal = 0u, readVal;
		while ((readVal = imageAtomicCompSwap(velocityAttach, ivec2(gl_FragCoord.xy), prevVal, velWrite)) != prevVal)
		{
			if (readVal == 0xFFFFFFFFu) break;
			prevVal = readVal;
			vec2 readVelocity;
			readVelocity.x = int((prevVal & 0x0000FF00u) >> 8u) - 127;
			readVelocity.y = int(prevVal &  0x000000FFu) - 127;
			vec2 newWriteVel = readVelocity + velocity;
			float newWriteVelLen = length(newWriteVel);
			if (newWriteVelLen > 127.0) newWriteVel = (newWriteVel / newWriteVelLen) * 127.0;
			velWrite = (((int(newWriteVel.x) + 127) << 8) | (int(newWriteVel.y) + 127));
		}
	}

    diffOutput = diffuseFetch * instanceInfo.props[inInstanceIndex].attribs1.y;
	if (inStrength > 0.0) diffOutput *= inStrength;
}