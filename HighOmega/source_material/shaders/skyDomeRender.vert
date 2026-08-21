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

layout (location = 0) in vec4 inPosCol;

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

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

out gl_PerVertex 
{
	vec4 gl_Position;
};

layout (location = 0) out vec3 eyeToVertPixel;
layout (location = 1) out vec3 rayColor;
layout (location = 2) out vec3 mieColor;

float scale(float inp)
{
	float x = 1.0 - inp;
	return rayleighMieInfo.scaleDepth * exp(-0.00287 + x*(0.459 + x*(3.83 + x*(-6.80 + x*5.25))));
}

void main() 
{
	vec3 outPos = inPosCol.xyz;

	vec3 vert_to_eye = outPos - vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a);
	float vert_to_eye_len = length(vert_to_eye);
	vec3 vert_to_eye_norm = vert_to_eye / vert_to_eye_len;

	float start_depth = exp(rayleighMieInfo.scaleOverScaleDepth * (rayleighMieInfo.invW4InnerRad.a - frameMVP.upEyeY.a));
	float start_offset = start_depth*scale(vert_to_eye_norm.y);

	float sample_length = vert_to_eye_len * 0.2;
	float scaled_length = sample_length * 4.0;
	vec3 sample_ray = vert_to_eye_norm * sample_length;
	vec3 sample_point = vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a) + sample_ray * 0.5;

	vec3 color = vec3(0.0, 0.0, 0.0);
	for(int i=0; i<5; i++)
	{
		float height = length(sample_point);
		float depth = exp(rayleighMieInfo.scaleOverScaleDepth * (rayleighMieInfo.invW4InnerRad.a - height));
		float light_angle = abs(dot(rayleighMieInfo.sunDirAndExp.xyz, sample_point)) / height;
		float camera_angle = abs(dot(vert_to_eye_norm, sample_point)) / height;
		float scatter = start_offset + depth*(scale(light_angle) - scale(camera_angle));
		vec3 attenuate = exp(-scatter * (rayleighMieInfo.invW4InnerRad.xyz * 0.031415 + 0.012566));
		color += attenuate * (depth * scaled_length);
		sample_point += sample_ray;
	}

	eyeToVertPixel = -vert_to_eye;
	rayColor.rgb = color * (rayleighMieInfo.invW4InnerRad.xyz * 0.0025 * rayleighMieInfo.sunDirAndExp.a);
	mieColor.rgb = color * 0.001 * rayleighMieInfo.sunDirAndExp.a;

	gl_Position = frameMVP.projectionViewMatrix * vec4(outPos, 1.0);
}
