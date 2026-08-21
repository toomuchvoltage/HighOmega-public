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

#define M_PI 3.1415926535
#define Inv4Pi 0.0795774715

layout (location = 0) in vec3 eyeToVertPixel;
layout (location = 1) in vec3 rayColor;
layout (location = 2) in vec3 mieColor;

layout (location = 0) out vec4 skyOutput;

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

layout (binding = 1) uniform RayleighMieUBO
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

layout (scalar, binding = 2) uniform SkyDomeParamsUBO
{
	vec2 invDims;
	float doAurora;
} skyDomeParams;

layout (scalar, binding = 3) uniform RenderTimeUBO
{
	float cur;
	float prev;
} renderTime;

layout (binding = 4) uniform sampler3D noiseTex;
layout (binding = 5) uniform sampler3D noiseTex2;
layout (binding = 6) uniform sampler2D moonImg;
layout (binding = 7) uniform sampler2D nebulaImg;
layout (binding = 8, rgba32f) uniform readonly image2D distantPosAlbedoAttach;

float luminance(vec3 rgb)
{
    return dot(rgb, vec3(0.2125, 0.7154, 0.0721));
}

void findAtmosphereEntryExit (vec3 v1, vec3 m, float innerRad, float outerRad, out vec3 entryPoint, out vec3 exitPoint)
{
	float a = dot (m, m);
	float _4a = 4.0 * a;
	float b = dot (v1, m) * 2.0;
	float bSq = b * b;
	float c = dot (v1, v1);
	float c1 = c - (innerRad * innerRad);
	float c2 = c - (outerRad * outerRad);
	float q1 = sqrt (bSq - (_4a * c1));
	float q2 = sqrt (bSq - (_4a * c2));
	float invDenom = 1.0 / (2.0 * a);
	entryPoint = v1 + ((-b + q1) * invDenom) * m;
	exitPoint = v1 + ((-b + q2) * invDenom) * m;
}

void findAtmosphereExit (vec3 v1, vec3 m, float outerRad, out vec3 exitPoint)
{
	float a = dot (m, m);
	float _4a = 4.0 * a;
	float b = dot (v1, m) * 2.0;
	float bSq = b * b;
	float c = dot (v1, v1);
	float c2 = c - (outerRad * outerRad);
	float q2 = sqrt (bSq - (_4a * c2));
	float invDenom = 1.0 / (2.0 * a);
	exitPoint = v1 + ((-b + q2) * invDenom) * m;
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

float PHG (float g, float cosTheta)
{
	float gSq = g * g;
	float denomPreMul = 1 + gSq - (2.0 * g * cosTheta);
	return (1 - gSq) * Inv4Pi * inversesqrt(denomPreMul * denomPreMul * denomPreMul);
}

float miePhase (float cosTheta)
{
	return mix (PHG (0.8, cosTheta), PHG (-0.5, cosTheta), 0.5);
}

float isoPhase ()
{
	return Inv4Pi;
}

float evaluateDensity (vec3 evalPoint, float heightScale)
{
	vec3 cloudOffset = vec3 (renderTime.cur * 0.02 + 0.5, 0.5, 0.5);
	vec4 noiseFetch = texture (noiseTex, evalPoint * 0.5 + cloudOffset, 0);
	vec4 noiseFetch2 = texture (noiseTex2, evalPoint * 2.0 + cloudOffset, 0);
	float baseFBM = dot (noiseFetch, vec4 (0.5, 0.25, 0.125, 0.0625)) * heightScale;
	float detailFBM = dot (noiseFetch2.rgb, vec3 (0.5, 0.25, 0.125));
	float edgeDetector = 1.0 - baseFBM;
	edgeDetector *= edgeDetector * edgeDetector;
	return max (baseFBM - (1.0 - detailFBM) * edgeDetector, 0.0) * rayleighMieInfo.scatteringCoeff;
}

vec3 computeAmbientColor (float curDensity, float Hp, float Hb, vec3 skyColor, vec3 groundColor)
{
	vec3 IsotropicScatteringTop = exp(-1.8 * pow(Hp * curDensity, 0.7)) * skyColor;
	vec3 IsotropicScatteringBottom = exp(-1.8 * pow(Hb * curDensity, 0.7)) * groundColor;
    return IsotropicScatteringTop + IsotropicScatteringBottom;
}

vec4 computeClouds (vec3 sunColor)
{
	vec3 vertToEyeNorm = normalize (-eyeToVertPixel);
	
	float fadeCloudsOnHorizon = clamp (1.0 - vertToEyeNorm.y * 5.0, 0.0, 1.0);
	
	if ( fadeCloudsOnHorizon == 1.0 )
	{
		return vec4 (0.0, 0.0, 0.0, 1.0);
	}
	
	float invRadAtmosphereThickness = 1.0 / (rayleighMieInfo.outerRad - rayleighMieInfo.innerCloudRad);
	vec3 ambientColor = rayleighMieInfo.addSkyColorAndNightAmount.xyz * rayleighMieInfo.ambientCoeff;
	vec3 groundAmbientColor = rayleighMieInfo.approxGroundColor.xyz * rayleighMieInfo.ambientCoeff;
	
	vec3 entryPoint, exitPoint, exitToSun;
	findAtmosphereEntryExit (vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a), vertToEyeNorm, rayleighMieInfo.innerCloudRad, rayleighMieInfo.outerRad, entryPoint, exitPoint);
	vec3 dirVec = (exitPoint - entryPoint) * rayleighMieInfo.stepSizeToSky;
	float dirVecLen = length (dirVec);
	vec3 dirVecNorm = dirVec / dirVecLen;
	vec3 curPoint = entryPoint;
	float curTransmission = 1.0;
	vec3 totalColor = vec3 (0.0);
	float extinctionCoeff = rayleighMieInfo.transmissionCoeff + rayleighMieInfo.scatteringCoeff;
	for (int i = 0; i != int(rayleighMieInfo.numStepsToSky);i++)
	{
		float leftToSky = float (i) * rayleighMieInfo.stepSizeToSky;
		float curDensity = evaluateDensity (curPoint, leftToSky);

		if ( curDensity > 0.1 )
		{
			findAtmosphereExit (curPoint, rayleighMieInfo.lightDir.xyz, rayleighMieInfo.outerRad, exitToSun);

			vec3 evalLightDir = (exitToSun - curPoint) * rayleighMieInfo.stepSizeToSun;
			float evalLightDirLen = length (evalLightDir);
			vec3 curPointToSun = curPoint + evalLightDir * rayleighMieInfo.stepSizeEvalLightDir;
			float evalLightDensity = 1.0;
			for (int j = 0;j != int(rayleighMieInfo.numStepsToSun);j++)
			{
				if ( length (curPointToSun) > rayleighMieInfo.innerCloudRad ) // We have no clouds below the inner radius
				{
					float leftToSkyForSun = (length (curPointToSun) - rayleighMieInfo.innerCloudRad) * invRadAtmosphereThickness;
					float densityToSunEval = evaluateDensity (curPointToSun, leftToSkyForSun);
					if ( densityToSunEval > 0.1 ) evalLightDensity *= exp (-evalLightDirLen * densityToSunEval * extinctionCoeff);
				}
				curPointToSun += evalLightDir * rayleighMieInfo.stepSizeEvalLightDir;
			}

			vec3 curScattering = curDensity * rayleighMieInfo.scatteringCoeff * (computeAmbientColor (curDensity, 1.0 - leftToSky, leftToSky, ambientColor, groundAmbientColor) * isoPhase () + evalLightDensity * sunColor * miePhase (dot(dirVecNorm, rayleighMieInfo.lightDir.xyz)));
			float extinctionAmount = exp(-dirVecLen * curDensity * extinctionCoeff);
			vec3 curScatteringIntegral = (curScattering - curScattering * extinctionAmount) / max (curDensity * extinctionCoeff, 0.000001);
			totalColor += curTransmission * curScatteringIntegral;
			curTransmission *= extinctionAmount;
		}
		curPoint += dirVec;
		if ( curTransmission < 0.1 ) break;
	}
	curTransmission = mix (curTransmission, 1.0, fadeCloudsOnHorizon);
	totalColor = mix (totalColor, vec3 (0.0), fadeCloudsOnHorizon);
	
	return vec4 (totalColor, curTransmission);
}

vec4 computeAurora ()
{
	vec3 vertToEyeNorm = normalize (-eyeToVertPixel);

	vec3 entryPoint, exitPoint, exitToSun;
	findAtmosphereEntryExit (vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a), vertToEyeNorm, rayleighMieInfo.innerCloudRad, rayleighMieInfo.outerRad, entryPoint, exitPoint);
	vec3 dirVec = (exitPoint - entryPoint) * rayleighMieInfo.stepSizeToSky;
	float dirVecLen = length (dirVec);
	vec3 dirVecNorm = dirVec / dirVecLen;
	vec3 curPoint = entryPoint;
	float curTransmission = 1.0;
	vec3 totalColor = vec3 (0.0);
	for (int i = 0; i != int(rayleighMieInfo.numStepsToSky);i++)
	{
		float leftToSky = float (i) * rayleighMieInfo.stepSizeToSky;
		float curDensity = triNoise2d (curPoint.xz, renderTime.cur * 0.2) * (1.0 - leftToSky);

		totalColor += curTransmission * curDensity * mix (vec3 (0.0, 0.4, 0.0),vec3 (0.25, 0.0, 0.6), pow (leftToSky, 1.5)) * 0.4;
		curTransmission *= exp (-dirVecLen * curDensity);
		curPoint += dirVec;
	}
	
	return vec4 (totalColor, curTransmission);
}

vec3 Spin(vec3 axis, vec3 vec, float angle)
{
	vec3 axis_n = normalize(axis);
	float shadow = dot (axis_n, vec);
	vec3 bring_down = shadow*axis_n;
	vec3 plane_pt = vec - bring_down;

	vec3 X = normalize(plane_pt);
	vec3 Y = cross(axis_n, X);

	float pt_x = dot (plane_pt, X);
	float new_pt_x = cos(angle)*pt_x;
	float new_pt_y = sin(angle)*pt_x;

	return ((new_pt_x)*X) + ((new_pt_y)*Y) + bring_down;
}

vec3 plantImg (sampler2D inpImg, vec3 plantImgDir, vec3 spinAxis, float spinAmount, float textureAngleAmount, vec3 inpDir, bool Grayscale)
{
	vec3 sideX = normalize (cross (plantImgDir, plantImgDir + vec3 (0.0, 1.0, 1.0)));
	vec3 sideY = cross (plantImgDir, sideX);
	vec3 plantImgDirFinal = plantImgDir;
	
	if ( spinAmount != 0.0 )
	{
		sideX = Spin (spinAxis, sideX, spinAmount);
		sideY = Spin (spinAxis, sideY, spinAmount);
		plantImgDirFinal = Spin (spinAxis, plantImgDirFinal, spinAmount);
	}
	
	float shadowX = dot (inpDir, sideX);
	float shadowY = dot (inpDir, sideY);
	
	if ( dot (plantImgDirFinal, inpDir) < 0.0 || any (greaterThan (abs (vec2 (shadowX, shadowY)), vec2 (textureAngleAmount))) ) return vec3 (0.0);
	
	vec2 texUV = (vec2 (shadowX, shadowY) + vec2 (textureAngleAmount)) / (textureAngleAmount * 2.0);
	if ( texUV != clamp (texUV, vec2 (0.03), vec2 (0.97)) ) return vec3 (0.0);
	
	return Grayscale ? texture (inpImg, texUV).rrr : texture (inpImg, texUV).rgb;
}

vec3 plantHalo (vec3 plantHaloDir, float haloAngleAmount, vec3 inpDir)
{
	float nearness = dot (plantHaloDir, inpDir);
	
	vec3 retCol = vec3 (pow((max (nearness - (1.0 - haloAngleAmount), 0.0)) / haloAngleAmount, 5.0) * 0.75);
	retCol.rg *= 0.8;
	return retCol;
}

vec3 nightSky (vec3 inpDir)
{
	vec3 retCol = plantImg (moonImg, rayleighMieInfo.lightDir.xyz, vec3 (0.0), 0.0, 0.05, inpDir, true);
	retCol += plantImg (nebulaImg, normalize (vec3 (-1.0, 0.0, 0.0)), vec3 (0.0, 0.0, 1.0), rayleighMieInfo.skyObjectLightAndAngle.a, 0.8, inpDir, false) * 0.25;
	retCol += plantImg (nebulaImg, normalize (vec3 (-1.0, 0.0, 0.0)), vec3 (0.0, 0.0, 1.0), rayleighMieInfo.skyObjectLightAndAngle.a + M_PI * 0.5, 0.8, inpDir, false) * 0.25;
	retCol += plantHalo (rayleighMieInfo.lightDir.xyz, 0.025, inpDir);

	vec3 starVec = Spin (vec3 (0.0, 0.0, 1.0), inpDir, -rayleighMieInfo.skyObjectLightAndAngle.a);
	float starBrightness = max (luminance (texture (noiseTex2, starVec * 2.5, 0).rgb) - 0.75, 0.0) * 5.0;

	retCol += vec3 (starBrightness);
	return retCol;
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
	float cosine = dot(rayleighMieInfo.sunDirAndExp.xyz, eyeToVertPixel) / length(eyeToVertPixel);
	float mie_phase = 0.010016 * ((1.0 + cosine*cosine) / pow(1.9801 + 1.98*cosine, 1.5));
	float ray_phase = 0.75 * (1.0 + cosine*cosine);
	
	vec4 skyColor;

	skyColor.rgb = (ray_phase * rayColor) + (mie_phase * mieColor);

	// Our approximation to multiple scattering
	skyColor.rgb += rayleighMieInfo.addSkyColorAndNightAmount.xyz;

	skyColor.rgb = min(max(skyColor.rgb,0.0),10.0);

	skyColor.rgb = mix (skyColor.rgb, nightSky (normalize (-eyeToVertPixel)).rgb, rayleighMieInfo.addSkyColorAndNightAmount.a);

    ivec2 texelLoc1 = ivec2 (((gl_FragCoord.xy + vec2 (1.0, 0.0)) * skyDomeParams.invDims) * vec2 (imageSize(distantPosAlbedoAttach)));
    ivec2 texelLoc2 = ivec2 (((gl_FragCoord.xy - vec2 (1.0, 0.0)) * skyDomeParams.invDims) * vec2 (imageSize(distantPosAlbedoAttach)));
    ivec2 texelLoc3 = ivec2 (((gl_FragCoord.xy + vec2 (0.0, 1.0)) * skyDomeParams.invDims) * vec2 (imageSize(distantPosAlbedoAttach)));
    ivec2 texelLoc4 = ivec2 (((gl_FragCoord.xy - vec2 (0.0, 1.0)) * skyDomeParams.invDims) * vec2 (imageSize(distantPosAlbedoAttach)));
    vec4 distantPosAlbedoFetch1 = imageLoad (distantPosAlbedoAttach, texelLoc1);
    vec4 distantPosAlbedoFetch2 = imageLoad (distantPosAlbedoAttach, texelLoc2);
    vec4 distantPosAlbedoFetch3 = imageLoad (distantPosAlbedoAttach, texelLoc3);
    vec4 distantPosAlbedoFetch4 = imageLoad (distantPosAlbedoAttach, texelLoc4);
    if ( unpackUnorm4x8(floatBitsToUint (distantPosAlbedoFetch1.a)).a == 1.0 && 
		 unpackUnorm4x8(floatBitsToUint (distantPosAlbedoFetch2.a)).a == 1.0 && 
		 unpackUnorm4x8(floatBitsToUint (distantPosAlbedoFetch3.a)).a == 1.0 && 
		 unpackUnorm4x8(floatBitsToUint (distantPosAlbedoFetch4.a)).a == 1.0 )
    {
		 skyOutput = vec4 (skyColor.rgb, 1.0);
		 return ;
	}

	vec4 cloudColorAndTransmission;
	if ( skyDomeParams.doAurora == 1.0 )
		cloudColorAndTransmission = computeAurora();
	else
		cloudColorAndTransmission = computeClouds (rayleighMieInfo.skyObjectLightAndAngle.xyz);

	skyOutput = vec4 (skyColor.rgb * cloudColorAndTransmission.a + cloudColorAndTransmission.xyz, 1.0);
}