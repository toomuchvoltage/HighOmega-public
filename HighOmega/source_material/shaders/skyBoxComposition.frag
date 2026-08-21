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
#extension GL_EXT_scalar_block_layout : require

layout (scalar, binding = 1) uniform RasterMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} rasterMVP;

layout (binding = 2) uniform RayleighMieUBO
{
	vec4 lightDir;
	vec4 sunDirAndExp;
	vec4 invW4InnerRad;

	float innerCloudRad;
	float outerRad;
	float scaleDepth;
	float scaleOverScaleDepth;
	
	float scatteringCoeff;
	float transmissionCoeff;
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

layout (scalar, binding = 3) uniform SkyDomeParamsUBO
{
	vec2 invDims;
	float doAurora;
} skyDomeParams;

layout (scalar, binding = 4) uniform RenderTimeUBO
{
	float cur;
	float prev;
} renderTime;

layout (scalar, binding = 5) uniform skyBoxCompositionParamsUBO
{
	uint mode;
} skyBoxCompositionParams;

layout (binding = 6) uniform samplerCube skyDomeCube;
layout (binding = 7) uniform sampler2D skyDomeBackdrop;
layout (binding = 8, rgba8) uniform readonly image2D screenShadow;
layout (binding = 9, rgba32f) uniform readonly image2D distantPosAlbedoAttach;
layout (binding = 10, rgba32f) uniform readonly image2D distantNormAttach;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 skyBoxOutput;

float luminance(vec3 rgb)
{
    return dot(rgb, vec3(0.2125, 0.7154, 0.0721));
}

mat2 mm2(in float a)
{
	float c = cos(a), s = sin(a);return mat2(c,s,-s,c);
}

float tri(in float x)
{
	return clamp(abs(fract(x)-.5),0.01,0.49);
}

vec2 tri2(in vec2 p)
{
	return vec2(tri(p.x)+tri(p.y),tri(p.y+tri(p.x)));
}

// Thanks to Nimitz!
float triNoise2d(in vec2 p, float spd)
{
    float z = 1.8;
    float z2 = 2.5;
	float rz = 0.0;
    p *= mm2 (p.x * 0.06);
    vec2 bp = p;
	for (float i = 0.0; i< 5.0; i++)
	{
        vec2 dg = tri2 (bp * 1.85) * 0.75;
        dg *= mm2 (spd);
        p -= dg / z2;

        bp *= 1.3;
        z2 *= 0.45;
        z *= 0.42;
		p *= 1.21 + (rz - 1.0) * 0.02;
        
        rz += tri (p.x + tri(p.y))*z;
        p*= -mat2 (0.95534, 0.29552, -0.29552, 0.95534);
	}
    return clamp(1.0/pow(rz*29.0, 1.3), 0.0, 0.55);
}

vec4 distantGeomLighting (vec4 distantAlbedo, vec3 distantPos, vec3 distantNorm, vec3 sunColor)
{
    float distantShadow = imageLoad (screenShadow, ivec2 (gl_FragCoord.xy)).r;
    float diffuseDot = dot (distantNorm, rayleighMieInfo.lightDir.xyz);
    float diffuseRad = max (diffuseDot, 0.0);
    float specularityAmount = luminance (pow (distantAlbedo.rgb, vec3 (1.0/2.2)));
    specularityAmount *= specularityAmount;
    float specularityRad = specularityAmount * pow (max (dot (reflect (normalize (distantPos - vec3 (rasterMVP.lookEyeX.a, rasterMVP.upEyeY.a, rasterMVP.sideEyeZ.a)), distantNorm), rayleighMieInfo.lightDir.xyz), 0.0), 6.0);
    vec3 directLightAmount = (distantAlbedo.rgb * diffuseRad + vec3 (specularityRad)) * distantShadow * sunColor;
    vec3 backLightAmount = ((1.0 - diffuseDot) * 0.5) * rayleighMieInfo.addSkyColorAndNightAmount.xyz * distantAlbedo.rgb;
    float totalLightingMultiplier = 1.0;
    if (skyDomeParams.doAurora == 1.0 ) // Aurora caustics
    {
        for (int i = -1; i != 2; i++)
            for (int j = -1; j != 2; j++)
                totalLightingMultiplier += triNoise2d (distantPos.xz + vec2 (float(i), float(j)) * 0.1, renderTime.cur * 0.2);
		totalLightingMultiplier *= 3.0;
    }
    vec3 distantGeomColor = (directLightAmount + backLightAmount) * totalLightingMultiplier;
	vec4 retVal;
    if (skyDomeParams.doAurora == 1.0 )
    {
        retVal = vec4 (distantGeomColor, 1.0); // No distant haze during aurora
    }
    else
    {
		retVal = vec4 (mix (rayleighMieInfo.horizonColor.xyz * (1.0 + clamp ((11.0 - distantPos.y) * 0.5, 0.0, 1.0)), distantGeomColor, min (pow (10.0/length (distantPos), 5.0), 1.0)), 1.0);
    }
	return retVal;
}

vec3 cubemapFaceSampleDir ()
{
	vec2 deNormUV = (inUV - 0.5) * 2.0;
	return rasterMVP.lookEyeX.xyz - deNormUV.x * rasterMVP.sideEyeZ.xyz - deNormUV.y * rasterMVP.upEyeY.xyz;
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

vec4 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyDomeCube,dirToSample);
}

void main()
{
    ivec2 texelLoc = ivec2 (gl_FragCoord.xy);
    vec4 distantPosAlbedoFetch = imageLoad (distantPosAlbedoAttach, texelLoc);
    if (distantPosAlbedoFetch.rgb != vec3 (0.0))
    {
        skyBoxOutput = distantGeomLighting (unpackUnorm4x8(floatBitsToUint (distantPosAlbedoFetch.a)), distantPosAlbedoFetch.rgb, fromZSignXY (floatBitsToUint (imageLoad (distantNormAttach, texelLoc).x)), rayleighMieInfo.skyObjectLightAndAngle.xyz);
    }
	else
	{
		if ( skyBoxCompositionParams.mode == 6 )
			skyBoxOutput = texture (skyDomeBackdrop, inUV);
		else
			skyBoxOutput = getSkyValue (cubemapFaceSampleDir());
	}
	skyBoxOutput.rgb = pow(skyBoxOutput.rgb, vec3(1.0/2.2));
}