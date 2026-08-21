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

struct BVHTriangle
{
	vec4 e1MinComp, e2MinComp, e3MinComp;
	vec4 nMaxComp, tMaxComp, bMaxComp;
	vec4 uv1uv2;
	vec4 uv3DielectricAndIoRThenEmissivity;
	uvec4 diffUVCoords;
	uvec4 normUVCoords;
	uvec4 roughUVCoords;
	uvec4 mortonCodeIntReserved;
};

struct BVHInternalNode
{
	vec4 aabbMin, aabbMax;
	uvec4 ChildAChildBLeafPtrReserved;
};

layout (binding = 2) buffer TriangleArrSSBO
{
	BVHTriangle tris[];
} triArr;

layout (binding = 3) buffer InternalNodeSSBO
{
	BVHInternalNode nodes[];
} nodeArr;

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;
layout (location = 2) in vec3 inEye;

layout (location = 0) out vec4 diffOutput;

bool rayBox (vec3 l1,vec3 invm,vec3 bmin,vec3 bmax)
{
	vec3 bmin_l1 = (bmin - l1)*invm;
	vec3 bmax_l1 = (bmax - l1)*invm;
	vec3 minVec = min (bmin_l1, bmax_l1);
	vec3 maxVec = max (bmin_l1, bmax_l1);

	float tmin = max(max(minVec.x, minVec.y), minVec.z);
	float tmax = min(min(maxVec.x, maxVec.y), maxVec.z);

	return tmax >= max(0.0, tmin);
}

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

bool LineWorld (vec3 p1, vec3 p2, out vec3 outPt, out vec3 outNorm, out vec3 outTan, out vec3 outBiTan, out vec2 outDiffAtlasUV, out vec2 outNrmAtlasUV, out vec2 outRghAtlasUV, out float DielectricAndIoR, out float emissivity)
{
	uint nodeStack[33];
	uint childProcessed[33];
	int stackPointer = 0;
	nodeStack[stackPointer] = 0;
	childProcessed[stackPointer] = 0;

	vec3 m = p2 - p1;
	vec3 invm = 1.0 / m;
	bool intersectionFound = false;

	while (true)
	{
		BVHInternalNode curNode = nodeArr.nodes[nodeStack[stackPointer]];
		vec3 nodeAABBMin = curNode.aabbMin.xyz;
		vec3 nodeAABBMax = curNode.aabbMax.xyz;
		
		if ( !(curNode.ChildAChildBLeafPtrReserved.x == 0 && curNode.ChildAChildBLeafPtrReserved.x == curNode.ChildAChildBLeafPtrReserved.y) )
		{
			if ( childProcessed[stackPointer] == 0 )
			{
				if ( rayBox (p1, invm, nodeAABBMin, nodeAABBMax) )
				{
					childProcessed[stackPointer] = 1;
					stackPointer++;
					nodeStack[stackPointer] = curNode.ChildAChildBLeafPtrReserved.x;
					childProcessed[stackPointer] = 0;
				}
				else
				{
					stackPointer--;
					if ( stackPointer == -1 ) break;
				}
			}
			else if ( childProcessed[stackPointer] == 1 )
			{
				childProcessed[stackPointer] = 2;
				stackPointer++;
				nodeStack[stackPointer] = curNode.ChildAChildBLeafPtrReserved.y;
				childProcessed[stackPointer] = 0;
			}
			else
			{
				stackPointer--;
				if ( stackPointer == -1 ) break;
			}
		}
		else
		{
			float curK = 1.0;
			BVHTriangle curTri = triArr.tris[curNode.ChildAChildBLeafPtrReserved.z];
			if ( lineSegTri (p1, m, 
							 curTri.e1MinComp.xyz, curTri.e2MinComp.xyz, curTri.e3MinComp.xyz, 
							 curK) )
			{
				vec3 foundP2 = p1 + m * curK;
				float bc1,bc2,bc3;
				barycentricCoords (foundP2, curTri.e1MinComp.xyz, curTri.e2MinComp.xyz, curTri.e3MinComp.xyz, bc1, bc2, bc3);
				m = foundP2 - p1;
				invm = 1.0 / m;
				outPt = foundP2;
				outNorm = curTri.nMaxComp.xyz;
				outTan = curTri.tMaxComp.xyz;
				outBiTan = curTri.bMaxComp.xyz;
				DielectricAndIoR = curTri.uv3DielectricAndIoRThenEmissivity.z;
				emissivity = curTri.uv3DielectricAndIoRThenEmissivity.w;
				vec2 outUV = fract (bc1 * curTri.uv1uv2.xy + bc2 * curTri.uv1uv2.zw + bc3 * curTri.uv3DielectricAndIoRThenEmissivity.xy);
				outDiffAtlasUV = vec2 (0.0);
				outNrmAtlasUV = vec2 (0.0);
				outRghAtlasUV = vec2 (0.0);
				intersectionFound = true;
			}
			stackPointer--;
			if ( stackPointer == -1 ) break;
		}
	}

	return intersectionFound;
}

void main()
{
	vec2 dirXYComp = (inUV - vec2 (0.5)) * 2.0;

	vec3 ourEye = vec3 (cos (testProps.time *  0.1), 0.0, sin (testProps.time *  0.1)) * 10.0;
	vec3 ourLook = normalize (-ourEye);
	vec3 ourUp = vec3 (0.0, 1.0, 0.0);
	vec3 ourSide = cross (ourLook, ourUp);
	vec3 pixelDir = normalize (ourLook - ourUp * dirXYComp.y + ourSide * dirXYComp.x);
	
	vec3 rayStart = ourEye;
	vec3 rayEnd = rayStart + pixelDir * 1000.0;
	
	vec3 outP;
	vec3 n, t, b;
	vec2 uv1, uv2, uv3;
	float f1, f2;
	
	if ( LineWorld (rayStart, rayEnd, outP, n, t, b, uv1, uv2, uv3, f1, f2) )
	{
		diffOutput = vec4 (length (outP - rayStart) * 0.005);
		return ;
	}
	diffOutput = vec4 (1.0, 0.0, 0.0, 1.0);
}