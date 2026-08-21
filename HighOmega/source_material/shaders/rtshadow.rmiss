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

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_scalar_block_layout : require

layout(location = 0) rayPayloadInEXT vec3 hitValue;

layout (binding = 3) uniform sampler2D shadowColorNear;

layout (scalar, binding = 4) uniform SkyShadowMapMVPNearUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} SkyShadowMapMVPNear;

layout (binding = 5) uniform sampler2D shadowColorFar;

layout (scalar, binding = 6) uniform SkyShadowMapMVPFarUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} SkyShadowMapMVPFar;

vec3 sampleShadow (vec3 samplePos)
{
	vec4 shadowUVFetch = SkyShadowMapMVPNear.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPNear.lookEyeX.a, SkyShadowMapMVPNear.upEyeY.a, SkyShadowMapMVPNear.sideEyeZ.a),1.0);
	shadowUVFetch.xyz /= shadowUVFetch.w;
	shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;
	vec3 shadowResult = vec3 (1.0);

	if ( shadowUVFetch.xy == clamp (shadowUVFetch.xy, vec2 (0.0), vec2 (1.0)) )
	{
		vec4 colorFetch = texture (shadowColorNear, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}
	else
	{
		shadowUVFetch = SkyShadowMapMVPFar.projectionViewMatrix * vec4 (samplePos - vec3 (SkyShadowMapMVPFar.lookEyeX.a, SkyShadowMapMVPFar.upEyeY.a, SkyShadowMapMVPFar.sideEyeZ.a),1.0);
		shadowUVFetch.xyz /= shadowUVFetch.w;
		shadowUVFetch.xy = (shadowUVFetch.xy+vec2 (1.0))*0.5;

		vec4 colorFetch = texture (shadowColorFar, shadowUVFetch.xy);
		if ( colorFetch.rgb != vec3 (0.0) && max (sign (colorFetch.a - shadowUVFetch.z), 0.0) == 0.0 ) shadowResult *= colorFetch.rgb;
	}

	return shadowResult;
}

void main()
{
	hitValue = sampleShadow (gl_WorldRayOriginEXT);
}