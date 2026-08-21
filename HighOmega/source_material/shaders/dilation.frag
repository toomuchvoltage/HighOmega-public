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
#extension GL_EXT_scalar_block_layout : require

#define M_PI 3.1415926535

layout (scalar, binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;
layout (binding = 1, r32ui) uniform uimage2D velocityAttach;
layout (binding = 2) uniform usampler2D visBufTriInfo;
layout (binding = 3) uniform sampler2D ssNormInstIDVelocityRoughnessAttach;
layout (binding = 4) uniform sampler2D visBufDepthStencil;
layout (binding = 5) uniform sampler2D screenDepthStencil;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

float getClosestDepth (ivec2 inpCoord)
{
	return max(texelFetch (visBufDepthStencil, inpCoord, 0).x, texelFetch (screenDepthStencil, inpCoord, 0).x);
}

uint getInstanceIDAndClosestDepth (ivec2 inpCoord, out float closestDepth)
{
	uint retVal;
	float ssDepth = texelFetch (screenDepthStencil, inpCoord, 0).x;
	float visBufDepth = texelFetch (visBufDepthStencil, inpCoord, 0).x;
	if ( ssDepth > visBufDepth )
		retVal = floatBitsToUint(texelFetch(ssNormInstIDVelocityRoughnessAttach, inpCoord, 0).y);
	else
		retVal = texelFetch (visBufTriInfo, inpCoord, 0).x;
	closestDepth = max (ssDepth, visBufDepth);
	return retVal;
}

void main()
{
	uint velRead = imageLoad (velocityAttach, ivec2(gl_FragCoord.xy)).x;
	if (velRead == 0xFFFFFFFFu) return ;
	float centerDepth;
	uint InstID = getInstanceIDAndClosestDepth (ivec2(gl_FragCoord.xy), centerDepth);

	vec2 velocity;
	velocity.x = int((velRead & 0x0000FF00u) >> 8u) - 127;
	velocity.y = int(velRead &  0x000000FFu) - 127;

	float velocityLen = length(velocity);
	if ( velocityLen < 1.0 ) return ;
	vec2 velocityNorm = (velocity / velocityLen);

	[[unroll]]
	for (int j = 0; j != 2; j++) // Fwd and backward smearing...
	{
		vec2 curVelocityNorm = (j == 0) ? velocityNorm : -velocityNorm;
		for (int i = 0; i != int(velocityLen) + 1; i++)
		{
			vec2 writeVelLocF = gl_FragCoord.xy + float (i) * curVelocityNorm;
			ivec2 writeVelLoc = ivec2 (clamp (writeVelLocF, vec2 (0.0), vec2 (imageSize(velocityAttach).xy - ivec2(1))));
			if ( floor(writeVelLocF) == floor(gl_FragCoord.xy) ) continue;
			float writeLocDepth;
			uint curInstID = getInstanceIDAndClosestDepth (writeVelLoc, writeLocDepth);
			if ( InstID == curInstID ) break ;
			if ( centerDepth < writeLocDepth ) continue; // visBuf uses reverseZ
			float fracTravel = float(i) / max (floor(velocityLen), 0.001);
			vec2 smearedVel = velocity * (1.0 - fracTravel);
			uint velWrite = (((int(smearedVel.x) + 127) << 8) | (int(smearedVel.y) + 127));
			uint prevVal = 0u, readVal;
			while ((readVal = imageAtomicCompSwap(velocityAttach, writeVelLoc, prevVal, velWrite)) != prevVal)
			{
				if (readVal == 0xFFFFFFFFu) break;
				prevVal = readVal;
				vec2 readVelocity;
				readVelocity.x = int((prevVal & 0x0000FF00u) >> 8u) - 127;
				readVelocity.y = int(prevVal &  0x000000FFu) - 127;
				vec2 newWriteVel = readVelocity + smearedVel;
				float newWriteVelLen = length(newWriteVel);
				if (newWriteVelLen > 127.0) newWriteVel = (newWriteVel / newWriteVelLen) * 127.0;
				velWrite = (((int(newWriteVel.x) + 127) << 8) | (int(newWriteVel.y) + 127));
			}
		}
	}
}