#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (binding = 1) uniform sampler2D materialAttach;
layout (binding = 2) uniform sampler2D worldPosAttach;
layout (binding = 3) uniform sampler2D normalAttach;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 outMaterial;
layout (location = 1) out vec4 outWorldPos;
layout (location = 2) out vec4 outNormal;

void main()
{
	ivec2 curLoc = ivec2 (inUV * vec2 (textureSize(materialAttach, 0)));
	outMaterial = texelFetch (materialAttach, curLoc, 0);
	outWorldPos = texelFetch (worldPosAttach, curLoc, 0);
	outNormal = texelFetch (normalAttach, curLoc, 0);
}