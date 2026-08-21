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
#extension GL_EXT_control_flow_attributes : enable
#extension GL_EXT_scalar_block_layout : require

#define M_PI 3.1415926535
#define HIGHOMEGA_ZONE_VOXELIZE_COARSENESS 1.0
#define HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES 6
#define HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE 32.0

layout (binding = 1, rgba32f) uniform readonly image2D materialAttach;
layout (binding = 2, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 3, rgba32f) uniform readonly image2D normAttach;

layout (scalar, binding = 4) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (binding = 5) uniform PathTraceParamsUBO
{
	vec4 radiosityMapCenterMotionFactor;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;

layout (binding = 6, rgba16f) uniform image3D radiosityMap[HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES * 6];
layout (binding = 7) uniform samplerCube skyBox;
layout (binding = 8) uniform sampler2D shadowDistanceNear;
layout (binding = 9) uniform sampler2D shadowColorNear;

layout (scalar, binding = 10) uniform SkyShadowMapMVPNearUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} SkyShadowMapMVPNear;

layout (binding = 11) uniform sampler2D shadowDistanceFar;
layout (binding = 12) uniform sampler2D shadowColorFar;

layout (scalar, binding = 13) uniform SkyShadowMapMVPFarUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} SkyShadowMapMVPFar;

layout (binding = 14) uniform RayleighMieUBO
{
	vec4 lightDir;
	vec4 sunDirAndExp;
	vec4 invW4InnerRad;

	float innerCloudRad;
	float outerRad;
	float scaleDepth;
	float scaleOverScaleDepth;
	
	float scatteringCoeff;
	float extinctionCoeff;
	float ambientCoeff;
	float stepSizeEvalLightDir;

	float stepSizeToSky;
	float numStepsToSky;
	float stepSizeToSun;
	float numStepsToSun;
	
	vec4 addSkyColorAndNightAmount;
	vec4 skyObjectLightAndAngle;
	vec4 horizonColor;
	vec4 approxGroundColor;
} rayleighMieInfo;

layout (scalar, binding = 15) uniform InfluenceUBO
{
	float factor;
} influence;

layout (constant_id = 0) const float nearMinBias = 0.00001;
layout (constant_id = 1) const float nearMaxBias = 0.00002;
layout (constant_id = 2) const float farMinBias = 0.001;
layout (constant_id = 3) const float farMaxBias = 0.002;

struct leafTransformInfo
{
	mat4 transMat;
	vec4 bMin;
	vec4 bMax;
};

layout (set = 3, binding = 0) buffer leafTransformInfoSSBO
{
	leafTransformInfo leaves[];
} transInfo;

struct GridLeafNode
{
	vec4 leafMinMorton;
	vec4 leafMaxLeafId;
};

layout (set = 3, binding = 1) buffer GridLeafNodeSSBO
{
	GridLeafNode nodes[];
} GridLeafNodeArr;

struct CWBVHInternalNode
{
	uvec4 n0;
	uvec4 n1;
	uvec4 n2;
	uvec4 n3;
	uvec4 n4;
};

layout (set = 3, binding = 2) buffer CWBVHInternalNodeSSBO
{
	CWBVHInternalNode nodes[];
} CWBVHNodeArr;

layout (set = 4, binding = 0, rgba8) uniform readonly image3D grids[];

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

bool rayBoxIntersectTime (vec3 l1,vec3 invm,vec3 bmin,vec3 bmax, out float tMin, out float tMax)
{
	vec3 bmin_l1 = (bmin - l1)*invm;
	vec3 bmax_l1 = (bmax - l1)*invm;
	vec3 minVec = min (bmin_l1, bmax_l1);
	vec3 maxVec = max (bmin_l1, bmax_l1);

	float tmin = max(max(minVec.x, minVec.y), minVec.z);
	float tmax = min(min(maxVec.x, maxVec.y), maxVec.z);

	bool retVal = ((tmax >= tmin) && (tmin < 1.0) && (tmax > 0.0));
	tMin = tmin;
	tMax = tmax;
	return retVal;
}

vec3 rayBoxIntersectTMin (vec3 l1,vec3 invm,vec3 bmin,vec3 bmax)
{
	vec3 bmin_l1 = (bmin - l1)*invm;
	vec3 bmax_l1 = (bmax - l1)*invm;
	return min (bmin_l1, bmax_l1);
}

vec3 getBoxNormal(vec3 rayDir, vec3 tMinComponent)
{
	return -sign(rayDir) * step(tMinComponent.yzx, tMinComponent.xyz) * step(tMinComponent.zxy, tMinComponent.xyz);
}

mat4 DirectionTransform(mat4 transform)
{
	mat4 retVal = mat4(1.0);
	retVal[0].xyz = cross(transform[1].xyz, transform[2].xyz);
	retVal[1].xyz = cross(transform[2].xyz, transform[0].xyz);
	retVal[2].xyz = cross(transform[0].xyz, transform[1].xyz);
	return retVal;
}

bool TraceGrid(vec3 p1, inout vec3 p2, inout vec3 voxCent, vec3 boxMin, vec3 boxMax, uint gridId, inout vec4 packedMat, inout vec3 outNorm)
{
	vec3 m = p2 - p1;
	vec3 p2_objspace;
	vec3 invm = vec3(1.0)/m;
	float tmin, tmax;
	bool orientedBox = false;
	if ( rayBoxIntersectTime (p1, invm, boxMin, boxMax, tmin, tmax) )
	{
		vec3 pmin = p1 + clamp (tmin, 0.0, 1.0) * m;
		vec3 pmax = p1 + clamp (tmax, 0.0, 1.0) * m;
		if (transInfo.leaves[gridId].transMat != mat4(1.0))
		{
			orientedBox = true;
			mat4 invTransMat = inverse(transInfo.leaves[gridId].transMat);
			p1 = (invTransMat * vec4 (p1, 1.0)).xyz;
			p2_objspace = (invTransMat * vec4 (p2, 1.0)).xyz;
			m = p2_objspace - p1;
			invm = vec3(1.0)/m;
			boxMin = transInfo.leaves[gridId].bMin.xyz;
			boxMax = transInfo.leaves[gridId].bMax.xyz;
			if ( rayBoxIntersectTime (p1, invm, boxMin, boxMax, tmin, tmax) )
			{
				pmin = p1 + clamp (tmin, 0.0, 1.0) * m;
				pmax = p1 + clamp (tmax, 0.0, 1.0) * m;
			}
			else
				return false;
		}

		vec3 pdir = pmax - pmin;
		float plen = length (pdir);
		if (plen == 0.0) return false;
		vec3 pdirNorm = pdir/plen;
		vec3 aabbLen = boxMax - boxMin;
		float plenSq = plen * plen;
		vec3 tminTemp, localInvM = vec3(1.0)/pdirNorm;
		
		vec3 sam = pmin;
		vec3 travelDiff = sam - pmin;
		while (dot(travelDiff, travelDiff) < plenSq)
		{
			ivec3 sampleLoc = ivec3 (clamp ((sam - boxMin) / aabbLen, vec3(0.0), vec3(0.999999)) * vec3(imageSize(grids[nonuniformEXT (gridId)]).xyz));
			vec4 imgFetch = imageLoad (grids[nonuniformEXT (gridId)], sampleLoc);
			if ( imgFetch.a == 0.0 )
			{
				p2 = sam;
				if ( orientedBox ) p2 = (transInfo.leaves[gridId].transMat * vec4 (p2, 1.0)).xyz;
				vec3 voxCentLocal = voxCent = boxMin + (vec3 (sampleLoc) + vec3 (0.5)) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
				if ( orientedBox ) voxCent = (transInfo.leaves[gridId].transMat * vec4 (voxCent, 1.0)).xyz;
				packedMat = imgFetch;
				tminTemp = rayBoxIntersectTMin (sam - pdirNorm, localInvM, voxCentLocal - vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS * 0.5), voxCentLocal + vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS * 0.5));
				outNorm = getBoxNormal (pdirNorm, tminTemp);
				if ( orientedBox ) outNorm = normalize ((DirectionTransform(transInfo.leaves[gridId].transMat) * vec4 (outNorm, 1.0)).xyz);
				return true;
			}
			else
			{
				sam += pdirNorm * floor(imgFetch.a * 255.0) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS * 0.2;
				travelDiff = sam - pmin;
			}
		}
	}
	return false;
}

struct VoxelHit
{
	vec3 color;
	bool specular;
	float emissivity;
	mat3 tanSpace;
	vec3 hitPos;
	vec3 hitPosNoOffset;
};

VoxelHit getHit (vec4 packedMat, vec3 hitNorm, vec3 traceDir, vec3 tracePos, vec3 voxCent)
{
	VoxelHit retVal;
	uint packedMatDeNorm = uint (packedMat.b * 255.0);
	retVal.color.r = float(uint(packedMat.g * 255.0) >> 4) / 15.0;
	retVal.color.g = packedMat.r;
	retVal.color.b = float(uint(packedMat.g * 255.0) & 0x0F) / 15.0;
	retVal.color = pow (retVal.color, vec3 (2.2)); // These are stored in gammaspace in the material channels.
	retVal.specular = ((packedMatDeNorm & 0x80) == 0x80) ? true : false;
	retVal.emissivity = (float(packedMatDeNorm & 0x7F) / 127.0) * 25.5;
	retVal.hitPos = retVal.hitPosNoOffset = tracePos;
	retVal.hitPos += hitNorm * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS * 1.73;
	retVal.tanSpace[2] = hitNorm;
	retVal.tanSpace[0] = normalize (cross (retVal.tanSpace[2], retVal.tanSpace[2] + vec3 (0.1)));
	retVal.tanSpace[1] = cross (retVal.tanSpace[2], retVal.tanSpace[0]);
	return retVal;
}

uint sign_extend_s8x4 (uint i)
{
	uint retVal = ((i & 0x80000000) != 0) ? 0xFF000000 : 0;
	retVal |=    (((i & 0x00800000) != 0) ? 0x00FF0000 : 0);
	retVal |=    (((i & 0x00008000) != 0) ? 0x0000FF00 : 0);
	retVal |=    (((i & 0x00000080) != 0) ? 0x000000FF : 0);
	return retVal;
}

bool LineGridCWBVH (vec3 p1, vec3 p2, out VoxelHit hitProps)
{
	uvec2 traversalStack[12];
	uint hitAddr = 0, stackPtr = 0;
	float tmin = 0.0, tmax = 1.0;
	vec3 rayD = p2 - p1;
	float originalRayLen = length(rayD);
	vec3 invRayD = 1.0 / rayD;
	uint octinv = (7 - ((rayD.x < 0.0 ? 4 : 0) | (rayD.y < 0.0 ? 2 : 0) | (rayD.z < 0.0 ? 1 : 0))) * 0x1010101;
	bool retVal = false;
	vec3 voxCent = vec3 (0.0), outNorm = vec3 (0.0);
	vec4 packedMat = vec4 (0.0);
	uvec2 ngroup = uvec2(0, 0x80000000), tgroup = uvec2(0);
	do
	{
		if (ngroup.y > 0x00FFFFFF)
		{
			uint hits = ngroup.y, imask = ngroup.y;
			uint child_bit_index = findMSB(hits);
			uint child_node_base_index = ngroup.x;
			ngroup.y &= (~(1 << child_bit_index));
			if (ngroup.y > 0x00FFFFFF) { traversalStack[stackPtr++] = ngroup; }
			{
				uint slot_index = (child_bit_index - 24) ^ (octinv & 255);
				uint relative_index = bitCount(imask & (~(0xFFFFFFFF << slot_index)));
				uint child_node_index = child_node_base_index + relative_index;
				uvec4 n0 = CWBVHNodeArr.nodes[child_node_index].n0, n1 = CWBVHNodeArr.nodes[child_node_index].n1;
				uvec4 n2 = CWBVHNodeArr.nodes[child_node_index].n2, n3 = CWBVHNodeArr.nodes[child_node_index].n3;
				uvec4 n4 = CWBVHNodeArr.nodes[child_node_index].n4;
				vec3 p = vec3(uintBitsToFloat(n0.x), uintBitsToFloat(n0.y), uintBitsToFloat(n0.z));
				uvec3 e = uvec3 (bitfieldExtract(n0.w, 0, 8), bitfieldExtract(n0.w, 8, 8), bitfieldExtract(n0.w, 16, 8));
				e = uvec3 ((e.x + 127) & 0xFF, (e.y + 127) & 0xFF, (e.z + 127) & 0xFF); // Kill the sign bit...
				ngroup.x = n1.x; tgroup.x = n1.y; tgroup.y = 0;
				uint hitmask = 0;
				float adjusted_idirx = uintBitsToFloat(e.x << 23u) * invRayD.x;
				float adjusted_idiry = uintBitsToFloat(e.y << 23u) * invRayD.y;
				float adjusted_idirz = uintBitsToFloat(e.z << 23u) * invRayD.z;
				float origx = -(p1.x - p.x) * invRayD.x;
				float origy = -(p1.y - p.y) * invRayD.y;
				float origz = -(p1.z - p.z) * invRayD.z;
				{	// First 4
					uint meta4 = n1.z, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					uint inner_mask4 = sign_extend_s8x4 (is_inner4 << 3);
					uint bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					uint child_bits4 = (meta4 >> 5) & 0x07070707;
					uint swizzledLox = (invRayD.x < 0.0) ? n3.z : n2.x, swizzledHix = (invRayD.x < 0.0) ? n2.x : n3.z;
					uint swizzledLoy = (invRayD.y < 0.0) ? n4.x : n2.z, swizzledHiy = (invRayD.y < 0.0) ? n2.z : n4.x;
					uint swizzledLoz = (invRayD.z < 0.0) ? n4.z : n3.x, swizzledHiz = (invRayD.z < 0.0) ? n3.x : n4.z;
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = bitfieldExtract(swizzledLox, 0 , 8) * adjusted_idirx + origx, tminx[1] = bitfieldExtract(swizzledLox, 8 , 8) * adjusted_idirx + origx, tminx[2] = bitfieldExtract(swizzledLox, 16, 8) * adjusted_idirx + origx;
					tminx[3] = bitfieldExtract(swizzledLox, 24, 8) * adjusted_idirx + origx, tminy[0] = bitfieldExtract(swizzledLoy, 0 , 8) * adjusted_idiry + origy, tminy[1] = bitfieldExtract(swizzledLoy, 8 , 8) * adjusted_idiry + origy;
					tminy[2] = bitfieldExtract(swizzledLoy, 16, 8) * adjusted_idiry + origy, tminy[3] = bitfieldExtract(swizzledLoy, 24, 8) * adjusted_idiry + origy, tminz[0] = bitfieldExtract(swizzledLoz, 0 , 8) * adjusted_idirz + origz;
					tminz[1] = bitfieldExtract(swizzledLoz, 8 , 8) * adjusted_idirz + origz, tminz[2] = bitfieldExtract(swizzledLoz, 16, 8) * adjusted_idirz + origz, tminz[3] = bitfieldExtract(swizzledLoz, 24, 8) * adjusted_idirz + origz;
					tmaxx[0] = bitfieldExtract(swizzledHix, 0 , 8) * adjusted_idirx + origx, tmaxx[1] = bitfieldExtract(swizzledHix, 8 , 8) * adjusted_idirx + origx, tmaxx[2] = bitfieldExtract(swizzledHix, 16, 8) * adjusted_idirx + origx;
					tmaxx[3] = bitfieldExtract(swizzledHix, 24, 8) * adjusted_idirx + origx, tmaxy[0] = bitfieldExtract(swizzledHiy, 0 , 8) * adjusted_idiry + origy, tmaxy[1] = bitfieldExtract(swizzledHiy, 8 , 8) * adjusted_idiry + origy;
					tmaxy[2] = bitfieldExtract(swizzledHiy, 16, 8) * adjusted_idiry + origy, tmaxy[3] = bitfieldExtract(swizzledHiy, 24, 8) * adjusted_idiry + origy, tmaxz[0] = bitfieldExtract(swizzledHiz, 0 , 8) * adjusted_idirz + origz;
					tmaxz[1] = bitfieldExtract(swizzledHiz, 8 , 8) * adjusted_idirz + origz, tmaxz[2] = bitfieldExtract(swizzledHiz, 16, 8) * adjusted_idirz + origz, tmaxz[3] = bitfieldExtract(swizzledHiz, 24, 8) * adjusted_idirz + origz;
					for (int i = 0; i < 4; i++)
					{
						// Use VMIN, VMAX to compute the slabs
						float cmin = max(max(max(tminx[i], tminy[i]), tminz[i]), tmin);
						float cmax = min(min(min(tmaxx[i], tmaxy[i]), tmaxz[i]), tmax);
						if (cmin <= cmax) hitmask |= bitfieldExtract(child_bits4, i * 8, 8) << bitfieldExtract(bit_index4, i * 8, 8);
					}
				}
				{	// Second 4
					uint meta4 = n1.w, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
					uint inner_mask4 = sign_extend_s8x4 (is_inner4 << 3);
					uint bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
					uint child_bits4 = (meta4 >> 5) & 0x07070707;
					uint swizzledLox = (invRayD.x < 0.0) ? n3.w : n2.y, swizzledHix = (invRayD.x < 0.0) ? n2.y : n3.w;
					uint swizzledLoy = (invRayD.y < 0.0) ? n4.y : n2.w, swizzledHiy = (invRayD.y < 0.0) ? n2.w : n4.y;
					uint swizzledLoz = (invRayD.z < 0.0) ? n4.w : n3.y, swizzledHiz = (invRayD.z < 0.0) ? n3.y : n4.w;
					float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
					tminx[0] = bitfieldExtract(swizzledLox, 0 , 8) * adjusted_idirx + origx, tminx[1] = bitfieldExtract(swizzledLox, 8 , 8) * adjusted_idirx + origx, tminx[2] = bitfieldExtract(swizzledLox, 16, 8) * adjusted_idirx + origx;
					tminx[3] = bitfieldExtract(swizzledLox, 24, 8) * adjusted_idirx + origx, tminy[0] = bitfieldExtract(swizzledLoy, 0 , 8) * adjusted_idiry + origy, tminy[1] = bitfieldExtract(swizzledLoy, 8 , 8) * adjusted_idiry + origy;
					tminy[2] = bitfieldExtract(swizzledLoy, 16, 8) * adjusted_idiry + origy, tminy[3] = bitfieldExtract(swizzledLoy, 24, 8) * adjusted_idiry + origy, tminz[0] = bitfieldExtract(swizzledLoz, 0 , 8) * adjusted_idirz + origz;
					tminz[1] = bitfieldExtract(swizzledLoz, 8 , 8) * adjusted_idirz + origz, tminz[2] = bitfieldExtract(swizzledLoz, 16, 8) * adjusted_idirz + origz, tminz[3] = bitfieldExtract(swizzledLoz, 24, 8) * adjusted_idirz + origz;
					tmaxx[0] = bitfieldExtract(swizzledHix, 0 , 8) * adjusted_idirx + origx, tmaxx[1] = bitfieldExtract(swizzledHix, 8 , 8) * adjusted_idirx + origx, tmaxx[2] = bitfieldExtract(swizzledHix, 16, 8) * adjusted_idirx + origx;
					tmaxx[3] = bitfieldExtract(swizzledHix, 24, 8) * adjusted_idirx + origx, tmaxy[0] = bitfieldExtract(swizzledHiy, 0 , 8) * adjusted_idiry + origy, tmaxy[1] = bitfieldExtract(swizzledHiy, 8 , 8) * adjusted_idiry + origy;
					tmaxy[2] = bitfieldExtract(swizzledHiy, 16, 8) * adjusted_idiry + origy, tmaxy[3] = bitfieldExtract(swizzledHiy, 24, 8) * adjusted_idiry + origy, tmaxz[0] = bitfieldExtract(swizzledHiz, 0 , 8) * adjusted_idirz + origz;
					tmaxz[1] = bitfieldExtract(swizzledHiz, 8 , 8) * adjusted_idirz + origz, tmaxz[2] = bitfieldExtract(swizzledHiz, 16, 8) * adjusted_idirz + origz, tmaxz[3] = bitfieldExtract(swizzledHiz, 24, 8) * adjusted_idirz + origz;
					for (int i = 0; i < 4; i++)
					{
						float cmin = max(max(max(tminx[i], tminy[i]), tminz[i]), tmin);
						float cmax = min(min(min(tmaxx[i], tmaxy[i]), tmaxz[i]), tmax);
						if (cmin <= cmax) hitmask |= bitfieldExtract(child_bits4, i * 8, 8) << bitfieldExtract(bit_index4, i * 8, 8);
					}
				}
				ngroup.y = (hitmask & 0xFF000000) | bitfieldExtract(n0.w, 24, 8);
				tgroup.y = hitmask & 0x00FFFFFF;
			}
		}
		else
		{
			tgroup = ngroup;
			ngroup = uvec2(0);
		}
		while (tgroup.y != 0)
		{
			uint primIndex = findMSB (tgroup.y);
			uint primAddr = tgroup.x + primIndex;
			if (TraceGrid(p1, p2, voxCent, GridLeafNodeArr.nodes[primAddr].leafMinMorton.xyz, GridLeafNodeArr.nodes[primAddr].leafMaxLeafId.xyz, floatBitsToUint (GridLeafNodeArr.nodes[primAddr].leafMaxLeafId.a), packedMat, outNorm))
			{
				tmax = length(p2 - p1)/originalRayLen;
				retVal = true;
			}
			tgroup.y -= (1 << primIndex);
		}
		if (ngroup.y <= 0x00FFFFFF)
		{
			if (stackPtr > 0)
			{
				ngroup = traversalStack[--stackPtr];
			}
			else
			{
				break;
			}
		}
	} while (true);
	
	hitProps = getHit (packedMat, outNorm, p2 - p1, p2, voxCent);
	return retVal;
}

vec3 getRand (vec3 forPos)
{
    float rx = fract(sin(dot(forPos, vec3(-1.9898,  -4.233, 5.1938))) * 40338.5453);
    float ry = fract(cos(dot(forPos, vec3( 1.1948,  7.963, -4.0687))) * 52718.5453);
    float rz = fract(sin(dot(forPos, vec3(-3.1948,  2.963,  5.0687))) * 63058.5453);
    
    return vec3 (rx, ry, rz);
}

void unpackEmissivityRefractiveIndexDielectric (float packedVal, out float emissivity, out float refractiveIndex, out bool diElectric)
{
	vec4 unpackedVal = unpackUnorm4x8 (floatBitsToUint (packedVal));
	emissivity = unpackedVal.x * 25.5;
	refractiveIndex = unpackedVal.y * 4.0;
	diElectric = unpackedVal.z == 1.0 ? true : false;
}

const float cascadeEdges[HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES] = {4.0, 8.0, 16.0, 32.0, 64.0, 128.0};

ivec4 getUVWLoc (vec3 pos)
{
	[[unroll]]
	for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++) {
		vec3 diffVec = pos - (pathTraceParams.radiosityMapCenterMotionFactor.xyz - vec3 (HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE * 0.5 * cascadeEdges[i]));
		vec3 fracAmt = diffVec / (HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE * cascadeEdges[i]);
		vec3 clampFracAmt = clamp(fracAmt, vec3 (0.0), vec3 (1.0));
		if ( clampFracAmt != fracAmt && i < HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES - 1 ) continue;
		return ivec4 (fracAmt * HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, i);
	}
}

void addDiffuseRadiance (vec3 irrad, vec3 pos, vec3 dir)
{
	if (any(isnan(irrad)) || any(isinf(irrad))) return ;
	ivec4 uvwLoc = getUVWLoc (pos);
	uint cascadeLevel = uvwLoc.w * 6;
	vec3 xIrrad = dir.x * irrad;
	vec3 yIrrad = dir.y * irrad;
	vec3 zIrrad = dir.z * irrad;
	vec4 lightPX = imageLoad (radiosityMap[nonuniformEXT (cascadeLevel + 0)], uvwLoc.xyz);
	vec4 lightNX = imageLoad (radiosityMap[nonuniformEXT (cascadeLevel + 1)], uvwLoc.xyz);
	vec4 lightPY = imageLoad (radiosityMap[nonuniformEXT (cascadeLevel + 2)], uvwLoc.xyz);
	vec4 lightNY = imageLoad (radiosityMap[nonuniformEXT (cascadeLevel + 3)], uvwLoc.xyz);
	vec4 lightPZ = imageLoad (radiosityMap[nonuniformEXT (cascadeLevel + 4)], uvwLoc.xyz);
	vec4 lightNZ = imageLoad (radiosityMap[nonuniformEXT (cascadeLevel + 5)], uvwLoc.xyz);
	if ( dir.x > 0.0 ) lightPX += vec4 (xIrrad, 0.5) * influence.factor;
	if ( dir.x < 0.0 ) lightNX += vec4 (-xIrrad, 0.5) * influence.factor;
	if ( dir.y > 0.0 ) lightPY += vec4 (yIrrad, 0.5) * influence.factor;
	if ( dir.y < 0.0 ) lightNY += vec4 (-yIrrad, 0.5) * influence.factor;
	if ( dir.z > 0.0 ) lightPZ += vec4 (zIrrad, 0.5) * influence.factor;
	if ( dir.z < 0.0 ) lightNZ += vec4 (-zIrrad, 0.5) * influence.factor;
	imageStore (radiosityMap[nonuniformEXT (cascadeLevel + 0)], uvwLoc.xyz, lightPX);
	imageStore (radiosityMap[nonuniformEXT (cascadeLevel + 1)], uvwLoc.xyz, lightNX);
	imageStore (radiosityMap[nonuniformEXT (cascadeLevel + 2)], uvwLoc.xyz, lightPY);
	imageStore (radiosityMap[nonuniformEXT (cascadeLevel + 3)], uvwLoc.xyz, lightNY);
	imageStore (radiosityMap[nonuniformEXT (cascadeLevel + 4)], uvwLoc.xyz, lightPZ);
	imageStore (radiosityMap[nonuniformEXT (cascadeLevel + 5)], uvwLoc.xyz, lightNZ);
}

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyBox,dirToSample).xyz;
}

vec3 getSkyDirectLight ()
{
	vec3 toSun = normalize (-SkyShadowMapMVPFar.lookEyeX.xyz);
	vec3 toSunUp = normalize (cross (toSun, toSun + vec3 (0.1)));
	vec3 toSunSide = cross (toSun, toSunUp);
	vec3 retVal = vec3 (0.0);
	for (int i = -1; i != 2; i++)
		for (int j = -1; j != 2; j++)
			retVal += getSkyValue (-SkyShadowMapMVPFar.lookEyeX.xyz + toSunUp * float(i) * 0.1 + toSunSide * float(j) * 0.1);
	return retVal * 0.111111111 * rayleighMieInfo.skyObjectLightAndAngle.xyz;
}

vec3 sampleShadow (vec3 samplePos, vec3 sampleNorm)
{
	vec4 shadowUVFetch = SkyShadowMapMVPNear.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPNear.lookEyeX.a, SkyShadowMapMVPNear.upEyeY.a, SkyShadowMapMVPNear.sideEyeZ.a),1.0);
	shadowUVFetch.xyz /= shadowUVFetch.w;
	shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;
	vec3 shadowResult = vec3 (1.0);

	if ( shadowUVFetch.xy == clamp (shadowUVFetch.xy, vec2 (0.0), vec2 (1.0)) )
	{
		ivec2 readShadowLoc = ivec2 (shadowUVFetch.xy * vec2 (textureSize(shadowDistanceNear, 0).xy));
		float shadowDist = texelFetch (shadowDistanceNear, readShadowLoc, 0).x;
		shadowResult *= max (sign ((shadowDist + mix (nearMaxBias, nearMinBias, abs(dot (sampleNorm, SkyShadowMapMVPNear.lookEyeX.xyz)))) - shadowUVFetch.z), 0.0);
		vec4 colorFetch = texture (shadowColorNear, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}
	else
	{
		shadowUVFetch = SkyShadowMapMVPFar.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPFar.lookEyeX.a, SkyShadowMapMVPFar.upEyeY.a, SkyShadowMapMVPFar.sideEyeZ.a),1.0);
		shadowUVFetch.xyz /= shadowUVFetch.w;
		shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;

		ivec2 readShadowLoc = ivec2 (shadowUVFetch.xy * vec2 (textureSize(shadowDistanceFar, 0).xy));
		float shadowDist = texelFetch (shadowDistanceFar, readShadowLoc, 0).x;
		shadowResult *= max (sign ((shadowDist + mix (farMaxBias, farMinBias, abs(dot (sampleNorm, SkyShadowMapMVPNear.lookEyeX.xyz)))) - shadowUVFetch.z), 0.0);
		vec4 colorFetch = texture (shadowColorFar, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}

	return shadowResult;
}

vec3 sampleHemiUniform (vec3 tanVec, vec3 biTan, vec3 norm, vec2 randVal)
{
	float r = sqrt(max(1.0 - randVal.x * randVal.x, 0.0));
	float phi = 2.0 * M_PI * randVal.y;
	return r * (cos(phi) * tanVec + sin(phi) * biTan) + randVal.x * norm;
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
	ivec2 fetchCoord = ivec2 (inUV * imageSize(materialAttach));
	vec4 materialFetch = imageLoad(materialAttach, fetchCoord);

	vec4 roughnessFetch = unpackUnorm4x8 (floatBitsToUint (materialFetch.z));

	if ( materialFetch == vec4(0.0) || roughnessFetch.z == 1.0 ) return ;

	float emissivity, refractiveIndex;
	bool diElectric;
	unpackEmissivityRefractiveIndexDielectric (materialFetch.w, emissivity, refractiveIndex, diElectric);

	if ( emissivity > 0.0 ) return ;

	vec4 worldPosFetch = imageLoad(worldPosAttach, fetchCoord);
	vec4 normalFetch = imageLoad(normAttach, fetchCoord);
	
	vec3 vertNorm = fromZSignXY (floatBitsToUint (normalFetch.y));
	vec3 tanVec = fromZSignXY (floatBitsToUint (normalFetch.z));
	vec3 biTanVec = fromZSignXY (floatBitsToUint (normalFetch.w));

	vec3 faceNorm = normalize (cross (tanVec, biTanVec));
	if ( dot (faceNorm, vertNorm) < 0.0 ) faceNorm = -faceNorm;

	vec3 eyeLoc = vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a);
	vec3 curPos = worldPosFetch.xyz;
	vec2 randVal = getRand (getRand(curPos + vec3 (mod(pathTraceParams.timeTurnBlurDirectionRawLight.x, 100.0)) * 10.0)).xy;
	vec3 rayDir = sampleHemiUniform (tanVec, biTanVec, faceNorm, randVal);
	vec3 ray2Dir;

	curPos += faceNorm * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS * 1.73;
	vec3 p1 = curPos + rayDir * 0.01, p2 = curPos + rayDir * 1000000.0;

	VoxelHit hit1Props, hit2Props;
	vec3 throughput = vec3 (1.0);
	vec3 directLight = vec3 (0.0);
	bool lastBounceDiffuse = false;
	if ( LineGridCWBVH (p1, p2, hit1Props) )
	{
		if ( hit1Props.emissivity > 0.0 )
		{
			throughput = hit1Props.color * hit1Props.emissivity;
			addDiffuseRadiance (throughput + directLight, worldPosFetch.xyz, rayDir);
		}
		else
		{
			if ( !hit1Props.specular )
			{
				randVal = getRand(getRand(hit1Props.hitPos + vec3 (mod(pathTraceParams.timeTurnBlurDirectionRawLight.x, 100.0)) * 10.0)).xy;
				ray2Dir = sampleHemiUniform (hit1Props.tanSpace[0], hit1Props.tanSpace[1], hit1Props.tanSpace[2], randVal);
				directLight += sampleShadow (hit1Props.hitPos, hit1Props.tanSpace[2]) * max (dot(hit1Props.tanSpace[2], -SkyShadowMapMVPFar.lookEyeX.xyz), 0.0) * getSkyDirectLight() * hit1Props.color * throughput;
				throughput = 2.0 * hit1Props.color * max(dot (ray2Dir, hit1Props.tanSpace[2]), 0.0);
				lastBounceDiffuse = true;
			}
			else
			{
				ray2Dir = reflect (rayDir, hit1Props.tanSpace[2]);
				throughput = hit1Props.color;
			}
			if ( LineGridCWBVH (hit1Props.hitPos, hit1Props.hitPos + ray2Dir * 1000000.0, hit2Props) )
			{
				if ( hit2Props.emissivity > 0.0 )
				{
					throughput *= hit2Props.color * hit2Props.emissivity;
					addDiffuseRadiance (throughput + directLight, worldPosFetch.xyz, rayDir);
					if ( lastBounceDiffuse ) addDiffuseRadiance (hit2Props.color * hit2Props.emissivity, hit1Props.hitPosNoOffset, ray2Dir);
				}
				else
				{
					if ( !hit2Props.specular )
						directLight += sampleShadow (hit2Props.hitPos, hit2Props.tanSpace[2]) * max (dot(hit2Props.tanSpace[2], -SkyShadowMapMVPFar.lookEyeX.xyz), 0.0) * getSkyDirectLight() * hit2Props.color * throughput;
					throughput = vec3 (0.0);
					addDiffuseRadiance (throughput + directLight, worldPosFetch.xyz, rayDir);
					if ( lastBounceDiffuse ) addDiffuseRadiance (vec3 (0.0), hit1Props.hitPosNoOffset, ray2Dir);
				}
			}
			else
			{
				vec3 skyValFetch = getSkyValue (ray2Dir);
				throughput *= skyValFetch;
				addDiffuseRadiance (throughput + directLight, worldPosFetch.xyz, rayDir);
				if ( lastBounceDiffuse ) addDiffuseRadiance (skyValFetch, hit1Props.hitPosNoOffset, ray2Dir);
			}
		}
	}
	else
	{
		throughput = getSkyValue (rayDir);
		addDiffuseRadiance (throughput + directLight, worldPosFetch.xyz, rayDir);
	}
}