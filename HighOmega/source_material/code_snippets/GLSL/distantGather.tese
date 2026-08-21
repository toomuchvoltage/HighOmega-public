#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define InterpHeightMap outPackedTessFlags.z
#define suppliedVertexNormal (outInstanceFlags2.x == 1.0)

layout(triangles, equal_spacing, cw) in;

layout (location = 0) in vec2 tessctrl_UV[];
layout (location = 1) in vec3 tessctrl_Pos[];
layout (location = 2) in vec3 tessctrl_VNorm[];
layout (location = 3) in vec3 tessctrl_Tangent[];
layout (location = 4) flat in float tessctrl_FlipFlag[];
layout (location = 5) flat in vec4 tessctrl_InstanceFlags[];
layout (location = 6) flat in vec4 tessctrl_InstanceFlags2[];
layout (location = 7) flat in vec4 tessctrl_PackedTessFlags[];

layout (binding = 0) uniform UBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} frameMVP;
layout (set = 1, binding = 3) uniform sampler2DArray heightSampler;

layout (location = 0) out vec2 outUV;
layout (location = 1) out vec3 outPos;
layout (location = 2) out vec3 outVNorm;
layout (location = 3) out vec3 outTangent;
layout (location = 4) flat out float outFlipFlag;
layout (location = 5) flat out vec4 outInstanceFlags;
layout (location = 6) flat out vec4 outInstanceFlags2;

vec3 Barycentric(vec3 inpPoint)
{
	vec3 v0 = tessctrl_Pos[1] - tessctrl_Pos[0], v1 = tessctrl_Pos[2] - tessctrl_Pos[0], v2 = inpPoint - tessctrl_Pos[0];
	float d00 = dot(v0, v0);
	float d01 = dot(v0, v1);
	float d11 = dot(v1, v1);
	float d20 = dot(v2, v0);
	float d21 = dot(v2, v1);
	float invDenom = 1.0 / (d00 * d11 - d01 * d01);
	vec3 retVal;
	retVal.y = (d11 * d20 - d01 * d21) * invDenom;
	retVal.z = (d00 * d21 - d01 * d20) * invDenom;
	retVal.x = 1.0f - retVal.y - retVal.z;
	return retVal;
}

float sampleHeight (vec2 UV)
{
	return texture (heightSampler, vec3 (UV, 0.0)).r;
}

void main() 
{
	outUV = gl_TessCoord.x*tessctrl_UV[0] + gl_TessCoord.y*tessctrl_UV[1] + gl_TessCoord.z*tessctrl_UV[2];
	outPos = gl_TessCoord.x*tessctrl_Pos[0] + gl_TessCoord.y*tessctrl_Pos[1] + gl_TessCoord.z*tessctrl_Pos[2];
	outVNorm = gl_TessCoord.x*tessctrl_VNorm[0] + gl_TessCoord.y*tessctrl_VNorm[1] + gl_TessCoord.z*tessctrl_VNorm[2];
	outTangent = gl_TessCoord.x*tessctrl_Tangent[0] + gl_TessCoord.y*tessctrl_Tangent[1] + gl_TessCoord.z*tessctrl_Tangent[2];
	outFlipFlag = tessctrl_FlipFlag[0];
	outInstanceFlags = tessctrl_InstanceFlags[0];
	outInstanceFlags2 = tessctrl_InstanceFlags2[0];

	mat3 tanSpace;
	tanSpace[2] = normalize (cross (tessctrl_Pos[0] - tessctrl_Pos[1], tessctrl_Pos[2] - tessctrl_Pos[1]));
	if ( dot (tessctrl_VNorm[0], tanSpace[2]) < 0.0 ) tanSpace[2] = -tanSpace[2];
	tanSpace[0] = normalize (outTangent);
	tanSpace[1] = cross (tanSpace[2], tanSpace[0]);
	if ( outFlipFlag == 1.0 ) tanSpace[1] = -tanSpace[1];

	if ( suppliedVertexNormal ) outVNorm = normalize (outVNorm);
	vec4 outPackedTessFlags = tessctrl_PackedTessFlags[0];

	vec3 moveOut = outVNorm * InterpHeightMap;

	vec3 u1 = outPos - 0.05*tanSpace[0];
	vec3 u2 = outPos + 0.05*tanSpace[0];
	vec3 v1 = outPos - 0.05*tanSpace[1];
	vec3 v2 = outPos + 0.05*tanSpace[1];
	vec3 u1BC = Barycentric (u1);
	vec3 u2BC = Barycentric (u2);
	vec3 v1BC = Barycentric (v1);
	vec3 v2BC = Barycentric (v2);
	vec2 u1UV = u1BC.x*tessctrl_UV[0] + u1BC.y*tessctrl_UV[1] + u1BC.z*tessctrl_UV[2];
	vec2 u2UV = u2BC.x*tessctrl_UV[0] + u2BC.y*tessctrl_UV[1] + u2BC.z*tessctrl_UV[2];
	vec2 v1UV = v1BC.x*tessctrl_UV[0] + v1BC.y*tessctrl_UV[1] + v1BC.z*tessctrl_UV[2];
	vec2 v2UV = v2BC.x*tessctrl_UV[0] + v2BC.y*tessctrl_UV[1] + v2BC.z*tessctrl_UV[2];
	vec3 u1MoveOut = moveOut, u2MoveOut = moveOut, v1MoveOut = moveOut, v2MoveOut = moveOut;
	if ( suppliedVertexNormal )
	{
		u1MoveOut = normalize (u1BC.x*tessctrl_VNorm[0] + u1BC.y*tessctrl_VNorm[1] + u1BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
		u2MoveOut = normalize (u2BC.x*tessctrl_VNorm[0] + u2BC.y*tessctrl_VNorm[1] + u2BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
		v1MoveOut = normalize (v1BC.x*tessctrl_VNorm[0] + v1BC.y*tessctrl_VNorm[1] + v1BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
		v2MoveOut = normalize (v2BC.x*tessctrl_VNorm[0] + v2BC.y*tessctrl_VNorm[1] + v2BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
	}
	u1 += sampleHeight (u1UV)*u1MoveOut;
	u2 += sampleHeight (u2UV)*u2MoveOut;
	v1 += sampleHeight (v1UV)*v1MoveOut;
	v2 += sampleHeight (v2UV)*v2MoveOut;

	tanSpace[0] = normalize (u2 - u1);
	tanSpace[1] = normalize (v2 - v1);
	
	vec3 newNorm = normalize (cross (tanSpace[0],tanSpace[1]));
	tanSpace[2] = newNorm * sign(dot (newNorm, tanSpace[2]));
	
	vec3 redoneOutBiTan = cross (tanSpace[0], newNorm);
	tanSpace[1] = redoneOutBiTan * sign(dot (redoneOutBiTan, tanSpace[1]));

	outPos += sampleHeight (outUV)*moveOut;

	gl_Position = frameMVP.projectionViewMatrix * vec4(outPos, 1.0);
}
