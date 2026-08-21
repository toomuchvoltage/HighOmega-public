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

layout (binding = 1) uniform sampler2D glossLightAttach;
layout (binding = 2, rgba32f) uniform readonly image2D materialAttach;
layout (binding = 3, rgba32f) uniform readonly image2D worldPosAttach;

layout (binding = 4) uniform PathTraceParamsUBO
{
	vec4 radiosityMapCenterMotionFactor;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;

#define TEMPORAL_TRAIL_AMOUNT 10

layout (binding = 5) uniform MVPsUBO
{
	mat4 matrices[TEMPORAL_TRAIL_AMOUNT];
} MVPs;

layout (binding = 6, rgba16f) uniform image3D glossLightTrail;
layout (binding = 7, rgba16f) uniform image3D worldPosCacheOutput;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outSpecularColor;

vec2 getScreenSampleCoord (mat4 inpMVP, vec3 inpPoint)
{
	vec4 pointTrans = inpMVP * vec4 (inpPoint,1.0);
	pointTrans.xy = ((pointTrans.xy/pointTrans.w)+vec2 (1.0))*0.5;
	return pointTrans.xy;
}

void temporalAccumulate(vec4 glossLightFetch, float roughnessScale, out vec4 glossRetVal, vec3 pos)
{
	ivec2 worldPosCacheSize = imageSize (worldPosCacheOutput).xy;
	vec4 glossOutputVal = vec4(0.0);
	float glossWeights = 0.0;

	for (int i = 0;i != TEMPORAL_TRAIL_AMOUNT;i++)
	{
		if ( i == int(pathTraceParams.timeTurnBlurDirectionRawLight.y) )
		{
			glossOutputVal += glossLightFetch;
			glossWeights += 1.0;
			continue;
		}

		vec2 mvpUV = getScreenSampleCoord (MVPs.matrices[i], pos.xyz);
		if ( mvpUV == clamp (mvpUV, vec2 (0.0), vec2 (0.999999)) )
		{
			ivec3 curLayerSampleCoord = ivec3 (ivec2 (mvpUV * vec2 (worldPosCacheSize)), i);
			vec3 curPos = imageLoad(worldPosCacheOutput, curLayerSampleCoord).xyz;
			vec3 diff = curPos.xyz - pos.xyz;

			if ( dot (diff,diff) < 1.0 )
			{
				float historicalImportance = 0.0;
				if ( i < int(pathTraceParams.timeTurnBlurDirectionRawLight.y) )
					historicalImportance = 1.0 - float(int(pathTraceParams.timeTurnBlurDirectionRawLight.y) - i)/float(TEMPORAL_TRAIL_AMOUNT);
				else
					historicalImportance = 1.0 - float(int(pathTraceParams.timeTurnBlurDirectionRawLight.y) + (TEMPORAL_TRAIL_AMOUNT - i))/float(TEMPORAL_TRAIL_AMOUNT);
				float curGlossWeight = smoothstep (0.0, 1.0, historicalImportance) * max (pathTraceParams.radiosityMapCenterMotionFactor.w, roughnessScale);

				glossOutputVal += imageLoad(glossLightTrail, curLayerSampleCoord) * curGlossWeight;
				glossWeights += curGlossWeight;
			}
		}
	}

	glossRetVal =  glossOutputVal / max (glossWeights, 0.001);
}

void unpackEmissivityRefractiveIndexDielectric (float packedVal, out float emissivity, out float refractiveIndex, out bool diElectric)
{
	vec4 unpackedVal = unpackUnorm4x8 (floatBitsToUint (packedVal));
	emissivity = unpackedVal.x * 25.5;
	refractiveIndex = unpackedVal.y * 4.0;
	diElectric = unpackedVal.z == 1.0 ? true : false;
}

void main()
{
	ivec2 texelLoc = ivec2 (inUV * vec2(imageSize (worldPosAttach)));
	vec4 fetchedWorldPosAndMaterialHint = imageLoad (worldPosAttach, texelLoc);
	if ( fetchedWorldPosAndMaterialHint == vec4 (0.0) )
	{
		outSpecularColor = vec4 (0.0);
		return ;
	}
	vec4 materialFetch = imageLoad (materialAttach, texelLoc);

	float emissivity, refractiveIndex;
	bool diElectric;
	unpackEmissivityRefractiveIndexDielectric (materialFetch.w, emissivity, refractiveIndex, diElectric);
	if ( emissivity > 0.0 )
	{
		outSpecularColor = vec4 (0.0);
		return ;
	}

	vec4 roughnessSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.z));
	vec4 glossLightFetch = texelFetch(glossLightAttach, ivec2 (gl_FragCoord.xy), 0);

	ivec3 storeCoord = ivec3 (ivec2 (gl_FragCoord.xy), int(pathTraceParams.timeTurnBlurDirectionRawLight.y));
	
	imageStore (glossLightTrail, storeCoord, glossLightFetch);
	imageStore (worldPosCacheOutput, storeCoord, vec4 (fetchedWorldPosAndMaterialHint.xyz, 1.0));

	vec4 glossLightAccum = vec4 (0.0);
	temporalAccumulate (glossLightFetch, max(roughnessSpecularity.x, roughnessSpecularity.y), glossLightAccum, fetchedWorldPosAndMaterialHint.xyz);
	
	outSpecularColor = glossLightAccum;
}