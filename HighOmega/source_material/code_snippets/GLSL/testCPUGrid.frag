#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

layout (binding = 1) uniform testPropsUBO
{
    float time;
    float res1;
    float res2;
    float res3;
} testProps;

layout (binding = 2) uniform GridInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxReserved;
	vec4 invMapDimReserved;
	vec4 gridSizeReserved;
} gridInfo;

struct GridTriangle
{
	vec4 e1u1;
	vec4 e2v1;
	vec4 e3u2;
	vec4 normv2;
	vec4 tanu3;
	vec4 bitanv3;
};

layout (set = 2, binding = 0) buffer SceneGridBuffer
{
	GridTriangle tris[];
} sceneGrid;

struct GridRef
{
	uvec4 offsetSizeReserved;
};

layout (set = 3, binding = 0) buffer GridRefBuffer
{
	GridRef info[];
} gridRef;

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

void main()
{
	vec2 dirXYComp = (inUV - vec2 (0.5)) * 4.0;

	vec3 ourEye = vec3 (cos (testProps.time *  0.1), 0.0, sin (testProps.time *  0.1)) * 10.0;
	vec3 ourLook = normalize (-ourEye);
	vec3 ourUp = vec3 (0.0, 1.0, 0.0);
	vec3 ourSide = cross (ourLook, ourUp);
	vec3 pixelDir = normalize (ourLook - ourUp * dirXYComp.y + ourSide * dirXYComp.x); // Keep it short in length!
	
	vec3 rayStart = ourEye;
	vec3 rayEnd = rayStart + pixelDir * 1000.0;
	vec3 rayDir = rayEnd - rayStart;
	vec3 curRayStart = rayStart + pixelDir;
	ivec3 lastUVWLoc = ivec3 (-1);
	
	for (int loopGuard = 0; loopGuard != 100; loopGuard++)
	{		
		vec3 curRayStartUVW = (curRayStart - gridInfo.mapMinVoxelSize.xyz) * gridInfo.invMapDimReserved.xyz;
	
		if ( curRayStartUVW != clamp (curRayStartUVW, vec3 (0.0), vec3 (1.0)) ) break;

		ivec3 curRayStartUVWLoc = ivec3(curRayStartUVW * gridInfo.gridSizeReserved.xyz);
		
		if ( lastUVWLoc == curRayStartUVWLoc )
		{
			curRayStart += pixelDir;
			continue;
		}

		float rayT = 1.0;
		int refLoc = (curRayStartUVWLoc.z * int(gridInfo.gridSizeReserved.x) * int(gridInfo.gridSizeReserved.y)) + (curRayStartUVWLoc.y * int(gridInfo.gridSizeReserved.x)) + curRayStartUVWLoc.x;
		for (int ii = 0; ii != gridRef.info[refLoc].offsetSizeReserved.y; ii++)
		{
			GridTriangle curTri = sceneGrid.tris[gridRef.info[refLoc].offsetSizeReserved.x + ii];
			
			lineSegTri (rayStart, rayDir, curTri.e1u1.xyz, curTri.e2v1.xyz, curTri.e3u2.xyz, rayT);
		}
		if ( rayT < 1.0 )
		{
			diffOutput = vec4 (length ((rayEnd - rayStart) * rayT) * 0.005);
			return ;
		}
		curRayStart += pixelDir;
		lastUVWLoc = curRayStartUVWLoc;
	}
	diffOutput = vec4 (1.0, 0.0, 0.0, 1.0);
}