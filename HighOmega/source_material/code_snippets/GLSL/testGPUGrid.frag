#version 460

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require

#define M_PI 3.1415926535

layout (binding = 1, r32ui) uniform readonly uimage3D voxelizedScene;

layout (binding = 2) uniform GridInfoUBO
{
	vec4 mapMinTrisPerCell;
	vec4 mapMaxVoxelSize;
	vec4 invMapDimsCellRadius;
	vec4 gridSize;
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

layout (binding = 3) uniform sampler2D materialAttach;
layout (binding = 4) uniform sampler2D worldPosAttach;
layout (binding = 5) uniform sampler2D normalAttach;
layout (binding = 6) uniform sampler2D backDropAttach;
layout (binding = 7) uniform samplerCube skyBox;
layout (binding = 8) uniform sampler2D onScreenText;
layout (binding = 9) uniform sampler2D fontMap;

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

vec3 toVec3(vec2 inpVec)
{
	float phi = inpVec.x * (2.0 * M_PI);
	float theta = inpVec.y * M_PI;
	vec3 onPlane = vec3 (cos(phi), 0.0, 0.0) + vec3 (0.0, 0.0, sin(phi));
	return onPlane*sin(theta) + vec3(0.0, cos(theta), 0.0);
}

vec3 getSkyValue (vec3 dirToSample)
{
	//dirToSample.z = -dirToSample.z;
	//return texture(skyBox,dirToSample).xyz;
	return vec3 (0.478, 0.576, 0.694); 
}

vec4 getText()
{
	if ( gl_FragCoord.x > 720 || gl_FragCoord.y > 100 ) return vec4(0.0); // 60 * 12, 5 * 20

	ivec2 tCoord = ivec2 (gl_FragCoord.xy * vec2 (0.08333333333, 0.05)); // 1/12.0, 1/20.0
	uint charNum = uint (texelFetch (onScreenText, tCoord.xy, 0).x * 255.0);
	
	vec2 startUV = vec2 (charNum % 16, 15 - (charNum / 16)) * 0.0625; // 1.0/16.0
	
	vec4 textVal = texture (fontMap, startUV + vec2(int (gl_FragCoord.x) % 12, 19 - int(gl_FragCoord.y) % 20) * vec2 (0.00520833333, 0.003125)) * 1.2; // 1/(16.0 * 12.0), 1/(16.0 * 20.0)
	vec4 backDropVal = vec4 (0.0, 0.0, 0.0, 0.5);
	return mix (backDropVal, textVal, textVal.a);
}

void main()
{
	vec2 inUVSafe = min (inUV, vec2(0.999999));
	ivec2 texelLoc = ivec2 (inUVSafe * vec2 (textureSize(materialAttach, 0).xy));
	
	vec4 materialFetch = texelFetch(materialAttach, texelLoc, 0);
	if ( materialFetch == vec4 (0.0) )
	{
		diffOutput = vec4 (0.478, 0.576, 0.694, 1.0);
		vec4 textVal = getText();
		if ( textVal.a > 0.0 ) diffOutput = mix (diffOutput, textVal, textVal.a);
		return ;
	}
    vec4 fragmentAlbedo = unpackUnorm4x8(floatBitsToUint (materialFetch.x));
	vec3 fragPosition = texelFetch(worldPosAttach, texelLoc, 0).xyz;
	vec4 normalFetch = texelFetch(normalAttach, texelLoc, 0);
	vec3 fragNormal = toVec3 (normalFetch.xy);

	vec3 pixelDir = reflect (normalize (fragPosition - inEye), fragNormal);
	vec3 moveAmount = pixelDir * gridInfo.mapMaxVoxelSize.a; // Shorter lines are preferred
	vec3 rayStart = fragPosition + pixelDir * gridInfo.mapMaxVoxelSize.a * 0.1;
	int minMovementToAvoidSelfIntersect = int(ceil (abs(1.0 / dot (pixelDir, fragNormal))));
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
			float barryU = 0.0, barryV = 0.0, barryW = 1.0;
			barycentricCoords (hitPt, chitTri.e1u1.xyz, chitTri.e2v1.xyz, chitTri.e3u2.xyz, barryU, barryV, barryW);
			vec2 hitUV = barryU * vec2 (chitTri.e1u1.a, chitTri.e2v1.a) + barryV * vec2 (chitTri.e3u2.a, chitTri.normv2.a) + barryW * vec2 (chitTri.tanu3.a, chitTri.bitanv3.a);
			diffOutput = texture (textures[nonuniformEXT (chitMaterialId * 5)], vec3 (hitUV, 0.0)) * fragmentAlbedo;
			vec4 textVal = getText();
			if ( textVal.a > 0.0 ) diffOutput = mix (diffOutput, textVal, textVal.a);
			return ;
		}
		curRayStart += moveAmount;
		lastUVWLoc = curRayStartUVWLoc;
	}
	diffOutput = vec4 (getSkyValue(pixelDir), 1.0) * fragmentAlbedo;
	vec4 textVal = getText();
	if ( textVal.a > 0.0 ) diffOutput = mix (diffOutput, textVal, textVal.a);
}