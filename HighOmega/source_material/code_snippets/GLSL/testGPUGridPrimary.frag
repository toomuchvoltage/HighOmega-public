#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

#define M_PI 3.1415926535

layout (binding = 1) uniform testPropsUBO
{
    float time;
    float res1;
    float res2;
    float res3;
} testProps;

layout (binding = 2, r32ui) uniform readonly uimage3D voxelizedScene;

layout (binding = 3) uniform GridInfoUBO
{
	vec4 mapMinTrisPerCell;
	vec4 mapMaxVoxelSize;
	vec4 invMapDimsCellRadius;
	vec4 gridSize;
} gridInfo;

layout (binding = 4) uniform sampler2D rrRes;

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

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inEye;

layout (location = 0) out vec4 diffOutput;

// Moller-Trumbore
bool lineSegTri(vec3 orig, vec3 dir, vec3 p1, vec3 p2, vec3 p3, inout float curK)
{
	vec3 e1, e2;
	vec3 P, Q, T;
	float det, inv_det, u, v;
	float t;
	e1 = p2 - p1;
	e2 = p3 - p1;
	P = cross(dir, e2);
	det = dot(e1, P);
	if (det > -0.0001 && det < 0.0001) return false;
	inv_det = 1.0 / det;
	T = orig - p1;
	u = dot(T, P) * inv_det;
	if (u < 0.0 || u > 1.0) return false;
	Q = cross(T, e1);
	v = dot(dir, Q) * inv_det;
	if (v < 0.0 || (u + v) > 1.0) return false;
	t = dot(e2, Q) * inv_det;
	if (t > 0.0 && t < curK)
	{
		curK = t;
		return true;
	}
	return false;
}

// Cramer's rule, Christer Ericson 2005
void barycentricCoords(vec3 p, vec3 a, vec3 b, vec3 c, out float u, out float v, out float w)
{
    vec3 v0 = b - a, v1 = c - a, v2 = p - a;
    float d00 = dot(v0, v0);
    float d01 = dot(v0, v1);
    float d11 = dot(v1, v1);
    float d20 = dot(v2, v0);
    float d21 = dot(v2, v1);
    float invDenom = 1.0 / (d00 * d11 - d01 * d01);
    v = (d11 * d20 - d01 * d21) * invDenom;
    w = (d00 * d21 - d01 * d20) * invDenom;
    u = 1.0 - v - w;
}

void main()
{
	vec2 dirXYComp = (inUV - vec2 (0.5)) * 2.0;

	/*vec3 ourEye = vec3 (cos (testProps.time *  0.1), 0.0, sin (testProps.time *  0.1)) * 10.0;
	vec3 ourLook = normalize (-ourEye);
	vec3 ourUp = vec3 (0.0, 1.0, 0.0);
	vec3 ourSide = cross (ourLook, ourUp);*/
	vec3 ourEye = vec3 (0.0);
	vec3 ourLook = vec3 (0.0, 0.0, 1.0);
	vec3 ourUp = vec3 (0.0, 1.0, 0.0);
	vec3 ourSide = cross (ourLook, ourUp);

	vec3 pixelDir = normalize (ourLook - ourUp * dirXYComp.y + ourSide * dirXYComp.x);
	vec3 moveAmount = pixelDir * gridInfo.mapMaxVoxelSize.a; // Shorter lines are preferred
	vec3 rayStart = ourEye + pixelDir;
	int minMovementToAvoidSelfIntersect = 0;
	vec3 curRayStart = rayStart;
	vec3 rayDir = pixelDir * 100000.0;
	ivec3 lastUVWLoc = ivec3 (-1);
	
	for (int loopGuard = 0; loopGuard != 1000; loopGuard++)
	{
		vec3 curRayStartUVW = (curRayStart - gridInfo.mapMinTrisPerCell.xyz) * gridInfo.invMapDimsCellRadius.xyz;
	
		if ( curRayStartUVW != clamp (curRayStartUVW, vec3 (0.0), vec3 (1.0)) ) break;

		ivec3 curRayStartUVWLoc = ivec3(curRayStartUVW * gridInfo.gridSize.xyz);
		if ( curRayStartUVWLoc == lastUVWLoc )
		{
			curRayStart += moveAmount;
			continue;
		}
		
		uint chitMaterialId = 0xFFFFFFFF;
		GridTriangle chitTri;

		float rayT = 1.0;
		bool intersected = false;
		
		for (int ii = 0; ii != int(gridInfo.mapMinTrisPerCell.a); ii++)
		{
			ivec3 instTriLoc = ivec3 (curRayStartUVWLoc.x*int(gridInfo.mapMinTrisPerCell.a) + ii, curRayStartUVWLoc.yz);
			uint instanceTriId = imageLoad (voxelizedScene, instTriLoc).x;
			if ( instanceTriId == 0xFFFFFFFF ) break;
			
			uint instanceId = instanceTriId >> 16;

			GridTriangle curTri = sceneGeom[nonuniformEXT (instanceId)].tris[nonuniformEXT (instanceTriId & 0x0000FFFF)];

			if ( lineSegTri (rayStart, rayDir, curTri.e1u1.xyz, curTri.e2v1.xyz, curTri.e3u2.xyz, rayT) )
			{
				chitMaterialId = instanceId;
				chitTri = curTri;
				intersected = true;
			}
			if ( ii == int(gridInfo.mapMinTrisPerCell.a) - 1 && !intersected && loopGuard > minMovementToAvoidSelfIntersect )
			{
				// exhaustion, there are more primitives that are probably missed so report an intersection
				chitMaterialId = instanceId;
				chitTri = curTri;
				rayT = length ((curRayStart + moveAmount * 0.5) - rayStart) / 100000.0;
				intersected = true;
			}
		}

		if ( intersected )
		{
			vec3 hitPt = rayDir * rayT + rayStart;
			diffOutput = vec4 (length (rayDir * rayT) * 0.005);
			return ;
		}
		curRayStart += moveAmount;
		lastUVWLoc = curRayStartUVWLoc;
	}
	diffOutput = vec4 (1.0, 0.0, 0.0, 1.0);
}