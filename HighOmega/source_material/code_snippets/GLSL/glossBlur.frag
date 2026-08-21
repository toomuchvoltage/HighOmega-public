#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 eye;
} frameMVP;

layout (binding = 1, rgba16f) uniform readonly image2D glossTraceInput;
layout (binding = 2, rgba16f) uniform writeonly image2D glossTraceOutput;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

// exp ( -5.0 * xDiff * xDiff ) where xDiff is sqrt(i*i + j*j) / sqrt(3*3 + 3*3)
const float gaussWeights[49] = {0.00673795,0.0270218,0.0621765,0.082085,0.0621765,0.0270218,0.00673795,
								0.0270218,0.108368,0.249352,0.329193,0.249352,0.108368,0.0270218,
								0.0621765,0.249352,0.573753,0.757465,0.573753,0.249352,0.0621765,
								0.082085,0.329193,0.757465,1,0.757465,0.329193,0.082085,0.0621765,
								0.249352,0.573753,0.757465,0.573753,0.249352,0.0621765,0.0270218,
								0.108368,0.249352,0.329193,0.249352,0.108368,0.0270218,0.00673795,
								0.0270218,0.0621765,0.082085,0.0621765,0.0270218,0.00673795};
											 
void main()
{
	ivec2 texelLocHiRes = ivec2 (inUV * vec2 (imageSize(glossTraceInput)));
	ivec2 texelLocLowRes = ivec2 (inUV * vec2 (imageSize(glossTraceOutput)));
	vec4 centerFetch = imageLoad (glossTraceInput, texelLocHiRes);

	if ( centerFetch.a < 0.0 )
	{
		imageStore (glossTraceOutput, texelLocLowRes, centerFetch);
		return ;
	}

	vec4 accum = vec4 (centerFetch.rgb, 1.0);
	for (int i = -3; i != 4; i++)
		for (int j = -3; j != 4; j++)
		{
			if ( i == 0 && j == 0 ) continue;
			ivec2 curLoc = texelLocHiRes + ivec2 (i, j);
			if ( curLoc != clamp (curLoc, ivec2 (0), imageSize(glossTraceInput) - ivec2 (1)) ) continue;
			vec4 curFetch = imageLoad (glossTraceInput, curLoc);
			if ( curFetch.a < 0.0 ) continue;
			float curWeight = gaussWeights[(i+3)*7 + (j+3)];
			accum += vec4 (curFetch.rgb * curWeight, curWeight);
		}

	imageStore (glossTraceOutput, texelLocLowRes, vec4 (accum.rgb / accum.a, centerFetch.a));
}