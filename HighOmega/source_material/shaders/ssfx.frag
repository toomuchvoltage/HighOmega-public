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
#extension GL_EXT_control_flow_attributes : enable
#extension GL_EXT_shader_16bit_storage : require
#extension GL_EXT_scalar_block_layout : require

#define M_PI 3.1415926535

layout (binding = 1) uniform sampler2D modulateAttach;
layout (binding = 2, rgba32f) uniform readonly image2D worldPosAttach;
layout (binding = 3) uniform sampler2D ssWorldPosAlbedoAttach;
layout (binding = 4) uniform sampler2D ssNormInstIDVelocityRoughnessAttach;
layout (binding = 5) uniform samplerCube skyBox;
layout (binding = 6) uniform sampler2D backdrop;
layout (binding = 7) uniform sampler2D nearScattering;
layout (binding = 8) uniform sampler2D visBufDepthStencil;
layout (binding = 9, r32ui) uniform uimage2D velocityAttach;

layout (scalar, binding = 10) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (scalar, binding = 11) uniform PrevFrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} prevFrameMVP;

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

float schlicks (float n1, float n2, vec3 norm, vec3 light)
{
    float R0 = (n1 - n2)/(n1 + n2);
    R0 *= R0;
    float _1_minus_dot = 1.0 - min(abs (dot (norm, light)), 1.0);
    _1_minus_dot = _1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot*_1_minus_dot;
    return R0 + (1 - R0) * _1_minus_dot;
}

vec2 getScreenSampleCoord (vec3 ptWorld)
{
    vec4 ptInfo = frameMVP.projectionViewMatrix * vec4 (ptWorld - vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a),1.0);
    ptInfo.xy = ((ptInfo.xy/ptInfo.w)+vec2 (1.0))*0.5;
    return ptInfo.xy;
}

vec3 projectCoord (vec3 coord, mat4 MVP, vec3 eyeLoc)
{
	vec4 pCoord = MVP * vec4 (coord - eyeLoc, 1.0);
	pCoord.xyz = pCoord.xyz/pCoord.w;
	pCoord.xy = (pCoord.xy + vec2 (1.0)) * 0.5;
	return pCoord.xyz;
}

vec3 getSkyValue (vec3 dirToSample)
{
    dirToSample.z = -dirToSample.z;
    return texture(skyBox,dirToSample).xyz;
}

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outFragColor;

bool getIsBackdrop (uint packedFlags)
{
	return (packedFlags & 0x00000010) != 0;
}

bool isViewerRelative (uint packedFlags)
{
	return (packedFlags & 0x00001000) != 0;
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

vec3 unpackDeferredVelocity (uint velocityPacked)
{
	vec4 velocityChannels = unpackUnorm4x8(velocityPacked);
	vec3 velocityDir = normalize (velocityChannels.xyz * 2.0 - vec3(1.0));
	return velocityDir * velocityChannels.w * 25.5;
}

void main()
{
	ivec2 screenRes = textureSize (ssWorldPosAlbedoAttach, 0);
	vec4 posFetchAlbedo = texelFetch(ssWorldPosAlbedoAttach, ivec2 (gl_FragCoord.xy), 0);
	vec3 posFetch = posFetchAlbedo.xyz;

	if ( posFetch != vec3 (0.0) )
	{
		vec4 albedoFetch = unpackUnorm4x8(floatBitsToUint (posFetchAlbedo.a));
		vec3 toPosNorm = normalize (posFetch - vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a));
		vec4 normInstIDVelocityRoughnessFetch = texelFetch(ssNormInstIDVelocityRoughnessAttach, ivec2 (gl_FragCoord.xy), 0);
		uint InstID = floatBitsToUint(normInstIDVelocityRoughnessFetch.y);
		vec3 normFetch = fromZSignXY (floatBitsToUint(normInstIDVelocityRoughnessFetch.x));
		uint packedFlags = floatBitsToUint (instanceInfo.props[floatBitsToUint(normInstIDVelocityRoughnessFetch.y)].attribs1.x);
		vec3 worldSpaceVelocity = unpackDeferredVelocity(floatBitsToUint(normInstIDVelocityRoughnessFetch.z));
		vec3 reflectVec = reflect (toPosNorm, normFetch);
		vec3 refractVec = refract (toPosNorm, normFetch, 1.0 / 1.54);
		vec3 reflectColor = getSkyValue (reflectVec);
		vec3 refractColor;
		float refractScale = 1.0;
		vec2 refractUVClamped;
	
		float edgeCloseness = max (abs (inUV.x - 0.5), abs (inUV.y - 0.5));
		if ( edgeCloseness > 0.4 ) refractScale *= (1.0 - (edgeCloseness - 0.4) * 10.0); // Coords on the edge, tend to sample outside the screen... scale them back in...

		if ( getIsBackdrop(packedFlags) )
		{
			refractColor = texture (backdrop, clamp (getScreenSampleCoord (posFetch + refractVec * refractScale), vec2 (0.0), vec2 (0.999999))).rgb;
		}
		else
		{
			// Find a decent refract candidate... some fall back on the base map itself...
			for(int i = 0; i != 10; i++)
			{
				refractUVClamped = clamp (getScreenSampleCoord (posFetch + refractVec * refractScale), vec2 (0.0), vec2 (0.999999));
				ivec2 refractUVClampedTexel = ivec2(refractUVClamped * vec2 (screenRes));
				refractColor = texelFetch (modulateAttach, refractUVClampedTexel, 0).rgb;
				vec3 curRefractPos = imageLoad (worldPosAttach, refractUVClampedTexel).rgb;
				if ( curRefractPos == vec3 (0.0) || dot (curRefractPos - posFetch, normFetch) < 0.0 ) break;
				refractScale *= 0.9;
				if ( refractScale < 0.1 ) break;
			}

			// Trace for a decent reflection point candidate
			vec2 traceVec = normalize (getScreenSampleCoord (posFetch + reflectVec) - getScreenSampleCoord(posFetch)) / vec2 (screenRes);
			if ( traceVec != vec2 (0.0) )
			{
				vec2 curUV = inUV;
				for (int i = 0; i != 10; i++)
				{
					curUV += traceVec;
					if ( any (lessThan (curUV, vec2 (0.0))) || any (greaterThan (curUV, vec2 (0.999999))) ) break;
					ivec2 curTexel = ivec2(curUV * vec2 (screenRes));
					if ( texelFetch(ssWorldPosAlbedoAttach, curTexel, 0).xyz != vec3 (0.0) ) continue;
					if ( dot (normalize (imageLoad (worldPosAttach, curTexel).rgb - posFetch), reflectVec) > 0.99 )
					{
						reflectColor = texelFetch(modulateAttach, curTexel, 0).rgb;
						break ;
					}
				}
			}
		}
		bool viewerRelative = isViewerRelative(packedFlags);
		float fresnel = schlicks (1.0, 1.54, normFetch, toPosNorm);
		outFragColor = vec4 (mix (refractColor, reflectColor, fresnel) * albedoFetch.rgb, 1.0);

		vec3 prevPos = posFetch - worldSpaceVelocity;

		vec3 prevProjectedCoord = projectCoord (prevPos, prevFrameMVP.projectionViewMatrix, vec3 (prevFrameMVP.lookEyeX.a, prevFrameMVP.upEyeY.a, prevFrameMVP.sideEyeZ.a));
		vec2 velocity = prevProjectedCoord.xy * vec2 (imageSize (velocityAttach).xy) - floor(gl_FragCoord.xy);
		float velLen = length(velocity); vec2 velNorm = velocity / max (velLen, 0.001);
		if (velLen > 127.0) { velocity = velNorm * 127.0; velLen = 127.0; }

		uint velWrite = viewerRelative ? 0xFFFFFFFFu : (((int(velocity.x) + 127) << 8) | (int(velocity.y) + 127));
		imageStore (velocityAttach, ivec2 (gl_FragCoord.xy), uvec4 (velWrite, 0, 0, 0));
	}
	else
		outFragColor = texelFetch(modulateAttach, ivec2(inUV * vec2 (screenRes)), 0);

	vec4 nearScatteringFetch = texture(nearScattering, inUV);
	outFragColor.rgb = mix (nearScatteringFetch.rgb, outFragColor.rgb, nearScatteringFetch.a);
}