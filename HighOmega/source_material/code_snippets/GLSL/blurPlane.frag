#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define M_PI 3.1415926535

layout (binding = 1) uniform sampler2D lightAttach;
layout (binding = 2) uniform sampler2D materialAttach;
layout (binding = 3) uniform sampler2D worldPosAttach;
layout (binding = 4) uniform sampler2D normAttach;

layout (binding = 5) uniform PathTraceParamsUBO
{
	vec4 reservedMotionFactorDiffuseOrSpecularTurn;
	vec4 timeTurnBlurDirectionRawLight;
} pathTraceParams;

layout (binding = 6) uniform FrameMVPUBO
{
    mat4 projectionViewMatrix;
    vec4 eye;
} frameMVP;

vec2 getScreenSampleCoord (vec3 ptWorld)
{
	vec4 ptInfo = frameMVP.projectionViewMatrix * vec4 (ptWorld,1.0);
	ptInfo.xy = ((ptInfo.xy/ptInfo.w)+vec2 (1.0))*0.5;
	return ptInfo.xy;
}

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outFragColor;

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	float sinTheta = sin(theta);
	return vec3 (cos(phi) * sinTheta, cos(theta), sin(phi) * sinTheta);
}

float unpackEmissivity (float inpEmissivityRefractiveIndexDielectricPacked)
{
	return fract (abs (inpEmissivityRefractiveIndexDielectricPacked))* 255.0;
}

vec2 unpackSpherical(float inpPack)
{
	uint packInt = floatBitsToUint(inpPack);
	vec2 retVal;
	retVal.x = float((packInt & 0xFFFF0000) >> 16) / 65535.0;
	retVal.y = float(packInt & 0x0000FFFF) / 65535.0;
	return retVal;
}

void main()
{
	vec2 outputSize = vec2 (textureSize(lightAttach, 0));
	ivec2 texelLoc = ivec2 (inUV * outputSize);
	vec4 centerSample = texelFetch (lightAttach, texelLoc, 0);
	vec4 materialFetch = texelFetch (materialAttach, texelLoc, 0);
	if ( unpackEmissivity (materialFetch.a) > 0.0 )
	{
		outFragColor = vec4 (0.0);
		return ;
	}
	if ( pathTraceParams.timeTurnBlurDirectionRawLight.a == 1.0 )
	{
		outFragColor = centerSample;
		return ;
	}

	vec3 roughnessAndSpecularity = unpackUnorm4x8(floatBitsToUint (materialFetch.b)).xyz;

	vec4 fetchedPosAndMaterialHint = texelFetch(worldPosAttach, texelLoc, 0);
	vec4 normTexelFetch = texelFetch(normAttach, texelLoc, 0);
	vec3 fetchedNorm = toVec3(unpackSpherical (normTexelFetch.x));
	vec3 fetchedBlurDir;
	if (pathTraceParams.timeTurnBlurDirectionRawLight.z == 0.0)
		fetchedBlurDir = toVec3(unpackSpherical (normTexelFetch.z));
	else
		fetchedBlurDir = toVec3(unpackSpherical (normTexelFetch.w));

	vec3 startPos = fetchedPosAndMaterialHint.xyz - fetchedBlurDir * 5.0;
	vec3 endPos = fetchedPosAndMaterialHint.xyz + fetchedBlurDir * 5.0;
	vec3 blurStep = (endPos - startPos) * 0.025;
	vec3 curPos = startPos;

	vec4 lightAccum = centerSample;
	float weightAccum = 1.0;
	for (int i = 0;i != 40;i++)
	{
		vec2 curUV = getScreenSampleCoord (curPos);
		if ( curUV != clamp (curUV,vec2(0.0),vec2(0.999999)) )
		{
			curPos += blurStep;
			continue;
		}
		ivec2 curTexelLoc = ivec2 (curUV * outputSize);
		vec4 curWorldPosAndMaterialHint = texelFetch(worldPosAttach, curTexelLoc, 0);
		vec3 curNormal = toVec3 (unpackSpherical (texelFetch(normAttach, curTexelLoc, 0).x));
		vec3 diffVec = fetchedPosAndMaterialHint.xyz - curWorldPosAndMaterialHint.xyz;
		if ( curWorldPosAndMaterialHint.a != fetchedPosAndMaterialHint.a || dot(diffVec, diffVec) > 25.0 || dot (curNormal, fetchedNorm) < 0.9 )
		{
			curPos += blurStep;
			continue;
		}
		lightAccum.rgb += texelFetch(lightAttach, curTexelLoc, 0).rgb;
		weightAccum += 1.0;
		curPos += blurStep;
	}

	outFragColor = vec4 (lightAccum.rgb / weightAccum, centerSample.a);
}