#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define ID gl_InvocationID
#define InterpSubdivAmount vert_PackedTessFlags[gl_InvocationID].w

layout(vertices = 3) out;

layout (location = 0) in vec2 vert_UV[];
layout (location = 1) in vec3 vert_PosW[];
layout (location = 2) in vec3 vert_VNorm[];
layout (location = 3) in vec3 vert_Tangent[];
layout (location = 4) flat in float vert_FlipFlag[];
layout (location = 5) flat in vec4 vert_InstanceFlags[];
layout (location = 6) flat in vec4 vert_InstanceFlags2[];
layout (location = 7) flat in vec4 vert_PackedTessFlags[];

layout (location = 0) out vec2 tessctrl_UV[];
layout (location = 1) out vec3 tessctrl_PosW[];
layout (location = 2) out vec3 tessctrl_VNorm[];
layout (location = 3) out vec3 tessctrl_Tangent[];
layout (location = 4) flat out float tessctrl_FlipFlag[];
layout (location = 5) flat out vec4 tessctrl_InstanceFlags[];
layout (location = 6) flat out vec4 tessctrl_InstanceFlags2[];
layout (location = 7) flat out vec4 tessctrl_PackedTessFlags[];

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} frameMVP;

void main() 
{
	tessctrl_UV[ID] = vert_UV[ID];
	tessctrl_PosW[ID] = vert_PosW[ID];
	tessctrl_VNorm[ID] = vert_VNorm[ID];
	tessctrl_Tangent[ID] = vert_Tangent[ID];
	tessctrl_FlipFlag[ID] = vert_FlipFlag[ID];
	tessctrl_InstanceFlags[ID] = vert_InstanceFlags[ID];
	tessctrl_InstanceFlags2[ID] = vert_InstanceFlags2[ID];
	tessctrl_PackedTessFlags[ID] = vert_PackedTessFlags[ID];

	if (ID == 0)
	{
		float divSize = min (20.0 / length (tessctrl_PosW[ID] - vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a)), 1.0);
		gl_TessLevelInner[0] = int (InterpSubdivAmount * divSize);
		gl_TessLevelOuter[0] = InterpSubdivAmount * divSize;
		gl_TessLevelOuter[1] = InterpSubdivAmount * divSize;
		gl_TessLevelOuter[2] = InterpSubdivAmount * divSize;
	}
}
