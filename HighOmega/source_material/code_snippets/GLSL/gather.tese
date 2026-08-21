#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

#define InterpUVOffset outPackedTessFlags.xy
#define InterpHeightMap outPackedTessFlags.z
#define suppliedVertexNormal (outInstanceFlags2.x == 1.0)

layout(triangles, equal_spacing, cw) in;

layout (location = 0) in vec2 tessctrl_UV[];
layout (location = 1) in vec3 tessctrl_PosW[];
layout (location = 2) in vec3 tessctrl_VNorm[];
layout (location = 3) in vec3 tessctrl_Tangent[];
layout (location = 4) flat in float tessctrl_FlipFlag[];
layout (location = 5) flat in vec4 tessctrl_InstanceFlags[];
layout (location = 6) flat in vec4 tessctrl_InstanceFlags2[];
layout (location = 7) flat in vec4 tessctrl_PackedTessFlags[];

layout (binding = 0) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
} frameMVP;

layout (binding = 1) uniform RenderTimeUBO
{
	vec4 time;
} renderInfo;

layout (set = 1, binding = 3) uniform sampler2DArray heightSampler;

layout (location = 0) out vec2 outUV;
layout (location = 1) out vec3 outPosW;
layout (location = 2) out vec3 outVNorm;
layout (location = 3) out vec3 outTangent;
layout (location = 4) flat out float outFlipFlag;
layout (location = 5) flat out vec4 outInstanceFlags;
layout (location = 6) flat out vec4 outInstanceFlags2;
layout (location = 8) out vec3 outGeomNorm;

vec3 Barycentric(vec3 inpPoint)
{
	vec3 v0 = tessctrl_PosW[1] - tessctrl_PosW[0], v1 = tessctrl_PosW[2] - tessctrl_PosW[0], v2 = inpPoint - tessctrl_PosW[0];
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

float sampleHeight (vec2 UV, bool isTerrain, vec3 curPos)
{
	if ( isTerrain )
	{
		vec2 curTerrainUV = (curPos.xz * 33.33333)/vec2 (textureSize (heightSampler, 0).xy);
		vec3 terrainMap = texture (heightSampler, vec3 (UV, 3.0)).rgb;
		terrainMap /= (terrainMap.x + terrainMap.y + terrainMap.z);
		float retVal = 0.0;
		retVal += texture (heightSampler, vec3 (curTerrainUV, 2.0)).r * terrainMap.r;
		retVal += texture (heightSampler, vec3 (curTerrainUV, 1.0)).g * terrainMap.g;
		retVal += texture (heightSampler, vec3 (curTerrainUV, 0.0)).b * terrainMap.b;
		return retVal;
	}
	else
	{
		return texture (heightSampler, vec3 (UV, 0.0)).r;
	}
}

void main() 
{
	outUV = gl_TessCoord.x*tessctrl_UV[0] + gl_TessCoord.y*tessctrl_UV[1] + gl_TessCoord.z*tessctrl_UV[2];
	outPosW = gl_TessCoord.x*tessctrl_PosW[0] + gl_TessCoord.y*tessctrl_PosW[1] + gl_TessCoord.z*tessctrl_PosW[2];
	outVNorm = gl_TessCoord.x*tessctrl_VNorm[0] + gl_TessCoord.y*tessctrl_VNorm[1] + gl_TessCoord.z*tessctrl_VNorm[2];
	outTangent = gl_TessCoord.x*tessctrl_Tangent[0] + gl_TessCoord.y*tessctrl_Tangent[1] + gl_TessCoord.z*tessctrl_Tangent[2];
	outFlipFlag = tessctrl_FlipFlag[0];
	outInstanceFlags = tessctrl_InstanceFlags[0];
	outInstanceFlags2 = tessctrl_InstanceFlags2[0];

	mat3 tanSpace;
	tanSpace[2] = normalize (cross (tessctrl_PosW[0] - tessctrl_PosW[1], tessctrl_PosW[2] - tessctrl_PosW[1]));
	if ( dot (tessctrl_VNorm[0], tanSpace[2]) < 0.0 ) tanSpace[2] = -tanSpace[2];
	tanSpace[0] = normalize (outTangent);
	tanSpace[1] = cross (tanSpace[2], tanSpace[0]);
	if ( outFlipFlag == 1.0 ) tanSpace[1] = -tanSpace[1];

	if ( suppliedVertexNormal ) outVNorm = normalize (outVNorm);
	vec4 outPackedTessFlags = tessctrl_PackedTessFlags[0];
	
	vec2 moveUV = renderInfo.time.x * InterpUVOffset;
	vec3 moveOut = outVNorm * InterpHeightMap;
	outGeomNorm = outVNorm;

	vec3 u1 = outPosW - 0.05*tanSpace[0];
	vec3 u2 = outPosW + 0.05*tanSpace[0];
	vec3 v1 = outPosW - 0.05*tanSpace[1];
	vec3 v2 = outPosW + 0.05*tanSpace[1];
	vec3 u1BC = Barycentric (u1);
	vec3 u2BC = Barycentric (u2);
	vec3 v1BC = Barycentric (v1);
	vec3 v2BC = Barycentric (v2);
	vec2 u1UV = u1BC.x*tessctrl_UV[0] + u1BC.y*tessctrl_UV[1] + u1BC.z*tessctrl_UV[2];
	vec2 u2UV = u2BC.x*tessctrl_UV[0] + u2BC.y*tessctrl_UV[1] + u2BC.z*tessctrl_UV[2];
	vec2 v1UV = v1BC.x*tessctrl_UV[0] + v1BC.y*tessctrl_UV[1] + v1BC.z*tessctrl_UV[2];
	vec2 v2UV = v2BC.x*tessctrl_UV[0] + v2BC.y*tessctrl_UV[1] + v2BC.z*tessctrl_UV[2];
	bool isTerrain = textureSize (heightSampler, 0).z > 1;
	vec3 u1MoveOut = moveOut, u2MoveOut = moveOut, v1MoveOut = moveOut, v2MoveOut = moveOut;
	if ( suppliedVertexNormal )
	{
		u1MoveOut = normalize (u1BC.x*tessctrl_VNorm[0] + u1BC.y*tessctrl_VNorm[1] + u1BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
		u2MoveOut = normalize (u2BC.x*tessctrl_VNorm[0] + u2BC.y*tessctrl_VNorm[1] + u2BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
		v1MoveOut = normalize (v1BC.x*tessctrl_VNorm[0] + v1BC.y*tessctrl_VNorm[1] + v1BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
		v2MoveOut = normalize (v2BC.x*tessctrl_VNorm[0] + v2BC.y*tessctrl_VNorm[1] + v2BC.z*tessctrl_VNorm[2]) * InterpHeightMap;
	}
	u1 += sampleHeight (u1UV + moveUV, isTerrain, outPosW)*u1MoveOut;
	u2 += sampleHeight (u2UV + moveUV, isTerrain, outPosW)*u2MoveOut;
	v1 += sampleHeight (v1UV + moveUV, isTerrain, outPosW)*v1MoveOut;
	v2 += sampleHeight (v2UV + moveUV, isTerrain, outPosW)*v2MoveOut;

	tanSpace[0] = normalize (u2 - u1);
	tanSpace[1] = normalize (v2 - v1);
	
	vec3 newNorm = normalize (cross (tanSpace[0],tanSpace[1]));
	tanSpace[2] = newNorm * sign(dot (newNorm, tanSpace[2]));
	
	vec3 redoneOutBiTan = cross (tanSpace[0], newNorm);
	tanSpace[1] = redoneOutBiTan * sign(dot (redoneOutBiTan, tanSpace[1]));

	outPosW += sampleHeight (outUV.xy + moveUV, isTerrain, outPosW)*moveOut;

	gl_Position = frameMVP.projectionViewMatrix * vec4(outPosW, 1.0);
}
