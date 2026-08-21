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
#define instanceFlags floatBitsToUint (instanceInfo.props[inInstanceIndex].attribs1.x)

layout (set = 1, binding = 0) uniform sampler2DArray curSceneTextures[];

struct TriangleFromVertBufWide
{
	vec4 e1Col1;
	vec2 uv1;
	uint Norm1;
	vec4 e2Col2;
	vec2 uv2;
	uint Norm2;
	vec4 e3Col3;
	vec2 uv3;
	uint Norm3;
};

#define ReadTri(o,i,j) {uint triCount = giantVB.dat[i];\
						uint offsetToIndices = i + 2, offsetToVertices = 2 + (triCount * 3); offsetToVertices += i + ((24 - ((offsetToVertices * 4) % 24 /* sizeof(RasterVertex) */ )) / 4);\
						uint e1Offset = offsetToVertices + giantVB.dat[offsetToIndices + j * 3] * 6;\
						uint e2Offset = offsetToVertices + giantVB.dat[offsetToIndices + j * 3 + 1] * 6;\
						uint e3Offset = offsetToVertices + giantVB.dat[offsetToIndices + j * 3 + 2] * 6;\
						o.e1Col1 = vec4 (uintBitsToFloat(giantVB.dat[e1Offset]), uintBitsToFloat(giantVB.dat[e1Offset + 1]), uintBitsToFloat(giantVB.dat[e1Offset + 2]), uintBitsToFloat(giantVB.dat[e1Offset + 3]));\
						o.uv1 = unpackHalf2x16(giantVB.dat[e1Offset + 4]);\
						o.Norm1 = giantVB.dat[e1Offset + 5];\
						o.e2Col2 = vec4 (uintBitsToFloat(giantVB.dat[e2Offset]), uintBitsToFloat(giantVB.dat[e2Offset + 1]), uintBitsToFloat(giantVB.dat[e2Offset + 2]), uintBitsToFloat(giantVB.dat[e2Offset + 3]));\
						o.uv2 = unpackHalf2x16(giantVB.dat[e2Offset + 4]);\
						o.Norm2 = giantVB.dat[e2Offset + 5];\
						o.e3Col3 = vec4 (uintBitsToFloat(giantVB.dat[e3Offset]), uintBitsToFloat(giantVB.dat[e3Offset + 1]), uintBitsToFloat(giantVB.dat[e3Offset + 2]), uintBitsToFloat(giantVB.dat[e3Offset + 3]));\
						o.uv3 = unpackHalf2x16(giantVB.dat[e3Offset + 4]);\
						o.Norm3 = giantVB.dat[e3Offset + 5];}

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

layout(scalar, set = 2, binding = 1) buffer GiantVertBuffer
{
	uint dat[];
} giantVB;

layout (set = 2, binding = 2) buffer TransformsSSBO
{
	mat4 mats[];
} transforms;

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (binding = 1, rgba32f) uniform readonly image2D worldPosAttach;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inNorm;
layout (location = 3) flat in uint inInstanceIndex;

layout (location = 0) out vec4 albedoOutput;
layout (location = 1) out vec4 specularOutput;
layout (location = 2) out vec4 roughnessSpecularityOutput;
layout (location = 3) out vec4 normalOut;
layout (location = 4) out vec4 tangentOut;
layout (location = 5) out vec4 biTangentOut;

void GetTangentBiTangent(vec3 v_0, vec3 v_1, vec3 v_2, vec2 tex0, vec2 tex1, vec2 tex2, out vec3 x_1_out, out vec3 y_1_out)
{
	vec3 A = v_0 - v_1, B = v_2 - v_1;
	vec2 v1 = tex0 - tex1, v2 = tex2 - tex1, v3;
	vec3 origin, x_1, y_1;
	float a, b;

	float v1xv2y_v1yv2x = v1.x*v2.y - v1.y*v2.x;
	float inv_v1xv2y_v1yv2x;
	if (v1xv2y_v1yv2x != 0.0) inv_v1xv2y_v1yv2x = 1.0f / v1xv2y_v1yv2x;

	v3 = -tex1;
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;
	origin = v_1 + a * A + b * B;

	v3 = vec2(1.0, 0.0) - tex1;
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;

	x_1 = v_1 + a * A + b * B;

	v3 = vec2(0.0, 1.0) - tex1;
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;

	y_1 = v_1 + a * A + b * B;

	x_1_out = x_1 - origin;
	y_1_out = y_1 - origin;
}

bool getHasRghMap (uint packedFlags)
{
	return (packedFlags & 0x00000002) != 0;
}

bool getHasNrmMap (uint packedFlags)
{
	return (packedFlags & 0x00000001) != 0;
}

bool getSmooth (uint packedFlags)
{
	return (packedFlags & 0x00000004) != 0;
}

vec4 sampleDiffuse (vec2 curUV)
{
	return texture (curDiffuseSampler, vec3 (curUV, 0.0));
}

vec4 sampleSpecular (vec2 curUV)
{
	return texture (curSpecularSampler, vec3 (curUV, 0.0));
}

vec3 sampleNormal (vec2 curUV)
{
	return texture (curNormalSampler, vec3 (curUV, 0.0)).rgb * 2.0 - vec3(1.0);
}

vec4 sampleRoughness (vec2 curUV)
{
	return texture (curRoughnessAndSpecSampler, vec3 (curUV, 0.0));
}

void main()
{
	vec3 finalNorm = normalize (inNorm);
	vec3 toEye = vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a) - inPos;
	vec3 toBg = imageLoad (worldPosAttach, ivec2(gl_FragCoord.xy)).xyz - inPos;
	float distToDecal = dot (finalNorm, toBg);

	if ( abs(distToDecal) > 1.0 || dot(toEye, toBg) >= 0.0 ) discard;

	vec4 albedoFetch = sampleDiffuse(inUV);
	if (albedoFetch.a == 0.0) discard;

	TriangleFromVertBufWide curTri;
	ReadTri (curTri, instanceInfo.props[inInstanceIndex].idxVertOffset, gl_PrimitiveID);
	vec3 curTriE1 = (transforms.mats[instanceInfo.props[inInstanceIndex].transformOffset] * vec4 (curTri.e1Col1.xyz, 1.0)).xyz;
	vec3 curTriE2 = (transforms.mats[instanceInfo.props[inInstanceIndex].transformOffset] * vec4 (curTri.e2Col2.xyz, 1.0)).xyz;
	vec3 curTriE3 = (transforms.mats[instanceInfo.props[inInstanceIndex].transformOffset] * vec4 (curTri.e3Col3.xyz, 1.0)).xyz;

	mat3 tanSpace;
	GetTangentBiTangent (curTriE1, curTriE2, curTriE3, curTri.uv1, curTri.uv2, curTri.uv3, tanSpace[0], tanSpace[1]);
	vec3 curFNorm = normalize (cross (curTriE1 - curTriE2, curTriE3 - curTriE2));
	if ( dot (curFNorm,finalNorm) < 0.0 ) curFNorm = -curFNorm;
	tanSpace[2] = getSmooth(instanceFlags) ? finalNorm : curFNorm;
	tanSpace[0] = normalize (tanSpace[0]);
	tanSpace[1] = normalize (tanSpace[1]);
	
	if ( getHasNrmMap (instanceFlags) ) tanSpace[2] = normalize (tanSpace * sampleNormal (inUV));
	vec4 roughFetch = vec4 (1.0, 1.0, 0.0, 0.0);
	if ( getHasRghMap (instanceFlags) ) roughFetch = sampleRoughness(inUV);

	albedoOutput = albedoFetch;
	specularOutput = sampleSpecular(inUV);
	roughnessSpecularityOutput = roughFetch;
	normalOut = vec4 ((tanSpace[2] + vec3 (1.0)) * 0.5, albedoFetch.a);
	tangentOut = vec4 ((tanSpace[0] + vec3 (1.0)) * 0.5, albedoFetch.a);
	biTangentOut = vec4 ((tanSpace[1] + vec3 (1.0)) * 0.5, albedoFetch.a);
}