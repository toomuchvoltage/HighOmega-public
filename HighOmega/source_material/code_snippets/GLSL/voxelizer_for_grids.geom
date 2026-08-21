#version 460
#extension GL_EXT_nonuniform_qualifier : require

layout(triangles, invocations = 1) in;
layout(triangle_strip, max_vertices = 3) out;

layout (location = 0) in vec2 vert_Pos_uv[];
layout (location = 1) in vec3 vert_Pos_W[];
layout (location = 2) in vec3 vert_Pos_VNorm[];
layout (location = 3) in vec3 vert_Pos_Tangent[];
layout (location = 4) flat in float vert_Pos_FlipFlag[];
layout (location = 5) flat in vec4 vert_InstanceFlags[];
layout (location = 6) flat in vec4 vert_InstanceFlags2[];
layout (location = 7) flat in ivec2 vert_InstanceTriangleId[];

layout (location = 0) out vec2 Pos_uv;
layout (location = 1) out vec3 Pos_W;
layout (location = 2) out vec3 Pos_VNorm;
layout (location = 3) out vec3 Pos_Tangent;
layout (location = 4) flat out float Pos_FlipFlag;
layout (location = 5) flat out vec4 outInstanceFlags;
layout (location = 6) flat out vec4 outInstanceFlags2;

layout (binding = 0, r32ui) uniform uimage3D sceneGrid;

layout (binding = 1) uniform TriGridInfoUBO
{
	vec4 mapMinTrisPerCell;
	vec4 mapMaxVoxelSize;
	vec4 invMapDimsCellRadius;
	vec4 gridSize;
} triGridInfo;

layout (binding = 2) uniform InstanceIdInfoUBO
{
	uvec4 offsetReserved;
} instanceIdInfo;

struct GridTriangle
{
	vec4 e1u1;
	vec4 e2v1;
	vec4 e3u2;
	vec4 normv2;
	vec4 tanu3;
	vec4 bitanv3;
};

layout (set = 2, binding = 0) buffer GridTriangleBuffer
{
	GridTriangle tris[];
} sceneGeom[];

layout (set = 3, binding = 0) uniform sampler2DArray textures[];

void main()
{
	uint instanceId = vert_InstanceTriangleId[0].x;
	uint triangleId = vert_InstanceTriangleId[0].y;

	mat3 tanSpace;
	tanSpace[2] = normalize (cross (vert_Pos_W[0] - vert_Pos_W[1], vert_Pos_W[2] - vert_Pos_W[1]));
	if ( dot (vert_Pos_VNorm[0], tanSpace[2]) < 0.0 ) tanSpace[2] = -tanSpace[2];
	tanSpace[0] = vert_Pos_Tangent[0];
	tanSpace[1] = cross (tanSpace[2], tanSpace[0]);
	if ( vert_Pos_FlipFlag[0] == 1.0 ) tanSpace[1] = -tanSpace[1];

	sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (triangleId)].e1u1 = vec4 (vert_Pos_W[0], vert_Pos_uv[0].x);
	sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (triangleId)].e2v1 = vec4 (vert_Pos_W[1], vert_Pos_uv[0].y);
	sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (triangleId)].e3u2 = vec4 (vert_Pos_W[2], vert_Pos_uv[1].x);
	sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (triangleId)].normv2  = vec4 (tanSpace[2], vert_Pos_uv[1].y);
	sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (triangleId)].tanu3   = vec4 (tanSpace[0], vert_Pos_uv[2].x);
	sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (triangleId)].bitanv3 = vec4 (tanSpace[1], vert_Pos_uv[2].y);

	vec3 minCorner = min (min(vert_Pos_W[0], vert_Pos_W[1]), vert_Pos_W[2]);
	vec3 maxCorner = max (max(vert_Pos_W[0], vert_Pos_W[1]), vert_Pos_W[2]);
	vec3 minUVW = clamp ((minCorner - triGridInfo.mapMinTrisPerCell.xyz) * triGridInfo.invMapDimsCellRadius.xyz, vec3 (0.0), vec3 (0.999999));
	vec3 maxUVW = clamp ((maxCorner - triGridInfo.mapMinTrisPerCell.xyz) * triGridInfo.invMapDimsCellRadius.xyz, vec3 (0.0), vec3 (0.999999));
	ivec3 minLoc = ivec3(minUVW * triGridInfo.gridSize.xyz);
	ivec3 maxLoc = ivec3(maxUVW * triGridInfo.gridSize.xyz);
	uint combinedInstTriId = (((instanceId + instanceIdInfo.offsetReserved.x) & 0x0000FFFF) << 16) | (triangleId & 0x0000FFFF);
	for (int i = minLoc.x; i != maxLoc.x + 1; i++)
		for (int j = minLoc.y; j != maxLoc.y + 1; j++)
			for (int k = minLoc.z; k != maxLoc.z + 1; k++)
			{
				vec3 cellCenter = (vec3 (i, j, k) + vec3 (0.5)) * triGridInfo.mapMaxVoxelSize.a + triGridInfo.mapMinTrisPerCell.xyz;
				if ( abs (dot (cellCenter - vert_Pos_W[0], tanSpace[2])) > triGridInfo.invMapDimsCellRadius.a ) continue;
				for (int ii = 0;ii != int(triGridInfo.mapMinTrisPerCell.a); ii++)
				{
					ivec3 instTriLoc = ivec3 (i*int(triGridInfo.mapMinTrisPerCell.a) + ii, j, k);

					uint instTriWrite = imageAtomicCompSwap (sceneGrid, instTriLoc, 0xFFFFFFFF, combinedInstTriId);

					if ( instTriWrite == 0xFFFFFFFF ) break;
				}
			}

	Pos_W = vert_Pos_W[0];
	Pos_uv = vert_Pos_uv[0];
	Pos_VNorm = vert_Pos_VNorm[0];
	Pos_Tangent = vert_Pos_Tangent[0];
	Pos_FlipFlag = vert_Pos_FlipFlag[0];
	outInstanceFlags = vert_InstanceFlags[0];
	outInstanceFlags2 = vert_InstanceFlags2[0];
	gl_Position = vec4 (-1.0, 1.0, 0.0, 1.0);
	EmitVertex();

	Pos_W = vert_Pos_W[1];
	Pos_uv = vert_Pos_uv[1];
	Pos_VNorm = vert_Pos_VNorm[1];
	Pos_Tangent = vert_Pos_Tangent[1];
	Pos_FlipFlag = vert_Pos_FlipFlag[1];
	outInstanceFlags = vert_InstanceFlags[1];
	outInstanceFlags2 = vert_InstanceFlags2[1];
	gl_Position = vec4 (1.0, 1.0, 0.0, 1.0);
	EmitVertex();

	Pos_W = vert_Pos_W[2];
	Pos_uv = vert_Pos_uv[2];
	Pos_VNorm = vert_Pos_VNorm[2];
	Pos_Tangent = vert_Pos_Tangent[2];
	Pos_FlipFlag = vert_Pos_FlipFlag[2];
	outInstanceFlags = vert_InstanceFlags[2];
	outInstanceFlags2 = vert_InstanceFlags2[2];
	gl_Position = vec4 (-1.0, -1.0, 0.0, 1.0);
	EmitVertex();

	EndPrimitive();
}