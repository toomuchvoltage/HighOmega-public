#version 450

#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_nonuniform_qualifier : require
#extension GL_EXT_shader_16bit_storage : require
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_shader_explicit_arithmetic_types_float16 : require

#define diffuseSampler sceneTextures[nonuniformEXT(instanceInfo.props[chitInstanceId].diffOffset)]
#define normalSampler sceneTextures[nonuniformEXT(instanceInfo.props[chitInstanceId].nrmOffset)]
#define roughnessAndSpecSampler sceneTextures[nonuniformEXT(instanceInfo.props[chitInstanceId].rghOffset)]
#define specularSampler sceneTextures[nonuniformEXT(instanceInfo.props[chitInstanceId].spcOffset)]

layout (scalar, binding = 1) buffer DofPropsUBO
{
	float invDimsX;
	float invDimsY;
	float blurDirection;
	float invCoCDist;
	vec4 screenMidPos;
	vec4 toScreenMidPosAlpha;
	float midScreenAlpha;
} dofProps;

layout (binding = 2) uniform sampler2D worldPosAttach;
layout (binding = 3) uniform sampler2D normalAttach;
layout (binding = 4) uniform sampler2D inpImage;
layout (binding = 5) uniform sampler2D onScreenText;
layout (binding = 6) uniform sampler2D midScreenText;
layout (binding = 7) uniform sampler2D fontMap;
layout (scalar, binding = 8) uniform FrameMVPUBO
{
	mat4 projectionViewMatrix;
	vec4 lookEyeX;
	vec4 upEyeY;
	vec4 sideEyeZ;
	vec2 whrTanHalfFovY;
} frameMVP;

struct BVHTriangleCompressed
{
	uvec4 e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID;
};

struct BVHInternalNode
{
	vec4 aabbMinChildA;
	vec4 aabbMaxChildB;
};

layout (set = 4, binding = 0) buffer InternalNodeSSBO
{
	BVHInternalNode nodes[];
} nodeArr;

layout (set = 4, binding = 1) buffer TriangleArrSSBO
{
	BVHTriangleCompressed tris[];
} triArr;

struct InstanceProps
{
    vec3 attribs1;
    vec4 attribs2;
	uint16_t splatOffset;
	uint16_t diffOffset;
	uint16_t nrmOffset;
	uint16_t rghOffset;
	uint16_t hgtOffset;
	uint16_t spcOffset;
};

layout (scalar, set = 4, binding = 2) buffer InstanceInfoSSBO
{
    InstanceProps props[];
} instanceInfo;

layout (set = 5, binding = 0) uniform sampler2DArray sceneTextures[];

struct TriangleFromVertBuf
{
	vec4 e1Col1;
	f16vec2 uv1;
	uint Norm1;
	vec4 e2Col2;
	f16vec2 uv2;
	uint Norm2;
	vec4 e3Col3;
	f16vec2 uv3;
	uint Norm3;
};

layout(scalar, set = 6, binding = 0) buffer SourceTriBuffer
{
	TriangleFromVertBuf tri[];
} sourceGeom[];

layout (location = 0) in vec2 inUV;
layout (location = 1) in vec3 inPos;

layout (location = 0) out vec4 diffOutput;

/*vec4 getText()
{
	if ( gl_FragCoord.x > 720 || gl_FragCoord.y > 100 ) return vec4(0.0); // 60 * 12, 5 * 20

	ivec2 tCoord = ivec2 (gl_FragCoord.xy * vec2 (0.08333333333, 0.05)); // 1/12.0, 1/20.0
	uint charNum = uint (texelFetch (onScreenText, tCoord.xy, 0).x * 255.0);
	
	vec2 startUV = vec2 (charNum % 16, 15 - (charNum / 16)) * 0.0625; // 1.0/16.0
	
	vec4 textVal = texture (fontMap, startUV + vec2(int (gl_FragCoord.x) % 12, 19 - int(gl_FragCoord.y) % 20) * vec2 (0.00520833333, 0.003125)) * 1.2; // 1/(16.0 * 12.0), 1/(16.0 * 20.0)
	vec4 backDropVal = vec4 (0.0, 0.0, 0.0, 0.5);
	return mix (backDropVal, textVal, textVal.a);
}*/

vec4 getMidScreenText()
{
	vec2 startRect = textureSize(worldPosAttach, 0).xy * 0.5 - vec2 (180.0, 30.0); // (30 * 12, 3 * 20) * 0.5
	vec2 endRect = startRect + vec2(360.0, 60.0);
	if ( any(lessThan (gl_FragCoord.xy, startRect)) || any(greaterThan (gl_FragCoord.xy, endRect)) ) return vec4(0.0);

	vec2 relXYCoord = gl_FragCoord.xy - startRect;

	ivec2 tCoord = ivec2 (relXYCoord * vec2 (0.08333333333, 0.05)); // 1/12.0, 1/20.0
	uint charNum = uint (texelFetch (midScreenText, tCoord.xy, 0).x * 255.0);

	vec2 startUV = vec2 (charNum % 16, 15 - (charNum / 16)) * 0.0625; // 1.0/16.0

	vec4 textFetch = texture (fontMap, startUV + vec2(int (relXYCoord.x) % 12, 19 - int(relXYCoord.y) % 20) * vec2 (0.00520833333, 0.003125)) * 1.2; // 1/(16.0 * 12.0), 1/(16.0 * 20.0)
	vec4 backDropVal = vec4 (0.0, 0.0, 0.0, 0.5);
	textFetch = mix (backDropVal, textFetch, textFetch.a);
	
	float fadeAmount = smoothstep (0.0, 50.0, 410.0 * dofProps.midScreenAlpha - relXYCoord.x); // A smoothstep cover that moves with the mid screen alpha, also 410.0 = 360.0 + 50.0

	return vec4 (textFetch.rgb, textFetch.a * fadeAmount);
}

void applyFinalPost ()
{
	if (dofProps.blurDirection == 1.0)
	{
		//vec4 textVal = getText();
		//diffOutput = mix (diffOutput, textVal, textVal.a);
		vec4 textVal = getMidScreenText();
		diffOutput = mix (diffOutput, textVal, textVal.a);
		diffOutput *= dofProps.toScreenMidPosAlpha.a;
	}
}

struct TriHitData
{
	vec3 pos;
	vec3 norm;
	vec3 faceNorm;
	vec3 tangent;
	vec3 bitan;
	
	vec3 albedoColor;
	vec2 anisoRoughness;
	float specAmount;
	vec3 specularColor;
	
	bool diElectric;
	float IoR;
	float emissivity;
};

vec3 fromZSignXY(uint inpPack)
{
	vec3 retVal;
	retVal.x = (float((inpPack & 0x7FFF0000u) >> 16) / 32766.0) * 2.0 - 1.0;
	retVal.y = (float(inpPack & 0x0000FFFFu) / 65534.0) * 2.0 - 1.0;
	vec2 xyVec = vec2(retVal.x, retVal.y);
	retVal.z = sqrt(clamp (1.0 - dot(xyVec, xyVec), 0.0, 1.0));
	if ((inpPack & 0x80000000u) != 0) retVal.z = -retVal.z;
	return retVal;
}

vec3 sampleTerrainWeights (vec2 curUV, uint chitInstanceId)
{
	vec3 terrainMap = texture (diffuseSampler, vec3 (curUV, 3.0)).rgb;
	terrainMap /= (terrainMap.x + terrainMap.y + terrainMap.z);
	return terrainMap;
}

vec4 sampleDiffuse (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, uint chitInstanceId)
{
	if ( isTerrain )
	{
		vec2 planarUV;
		vec3 absNorm = abs(surfNorm);
		if ( absNorm.y > 0.7 )
			planarUV = PosW.xz;
		else if ( absNorm.x > 0.7 )
			planarUV = PosW.yz;
		else
			planarUV = PosW.xy;
		vec2 planarFactor = vec2 (33.33333) / vec2 (textureSize (diffuseSampler, 0).xy);
		vec2 curTerrainUV = planarUV * planarFactor;
		vec4 retVal = vec4 (0.0);
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (diffuseSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return vec4 (retVal.xyz, 1.0);
	}
	return texture (diffuseSampler, vec3 (curUV, 0.0));
}

vec4 sampleSpecular (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, uint chitInstanceId)
{
	if ( isTerrain )
	{
		vec2 planarUV;
		vec3 absNorm = abs(surfNorm);
		if ( absNorm.y > 0.7 )
			planarUV = PosW.xz;
		else if ( absNorm.x > 0.7 )
			planarUV = PosW.yz;
		else
			planarUV = PosW.xy;
		vec2 planarFactor = vec2 (33.33333) / vec2 (textureSize (specularSampler, 0).xy);
		vec2 curTerrainUV = planarUV * planarFactor;
		vec4 retVal = vec4 (0.0);
		retVal += texture (specularSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (specularSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return vec4 (retVal.xyz, 1.0);
	}
	return texture (specularSampler, vec3 (curUV, 0.0));
}

vec3 sampleNormal (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, uint chitInstanceId)
{
	if ( isTerrain )
	{
		vec2 planarUV;
		vec3 absNorm = abs(surfNorm);
		if ( absNorm.y > 0.7 )
			planarUV = PosW.xz;
		else if ( absNorm.x > 0.7 )
			planarUV = PosW.yz;
		else
			planarUV = PosW.xy;
		vec2 planarFactor = vec2 (33.33333) / vec2 (textureSize (normalSampler, 0).xy);
		vec2 curTerrainUV = planarUV * planarFactor;
		vec3 retVal = vec3 (0.0);
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 2.0)).rgb - vec3 (1.0)) * inpWeights.r;
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 1.0)).rgb - vec3 (1.0)) * inpWeights.g;
		retVal += (2.0 * texture (normalSampler, vec3 (curTerrainUV, 0.0)).rgb - vec3 (1.0)) * inpWeights.b;
		return normalize (retVal);
	}
	return 2.0 * texture (normalSampler, vec3 (curUV, 0.0)).rgb - vec3 (1.0);
}

vec4 sampleRoughness (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, uint chitInstanceId)
{
	if ( isTerrain )
	{
		vec2 planarUV;
		vec3 absNorm = abs(surfNorm);
		if ( absNorm.y > 0.7 )
			planarUV = PosW.xz;
		else if ( absNorm.x > 0.7 )
			planarUV = PosW.yz;
		else
			planarUV = PosW.xy;
		vec2 planarFactor = vec2 (33.33333) / vec2 (textureSize (roughnessAndSpecSampler, 0).xy);
		vec2 curTerrainUV = planarUV * planarFactor;
		vec4 retVal = vec4 (0.0);
		retVal += texture (roughnessAndSpecSampler, vec3 (curTerrainUV, 2.0)) * inpWeights.r;
		retVal += texture (roughnessAndSpecSampler, vec3 (curTerrainUV, 1.0)) * inpWeights.g;
		retVal += texture (roughnessAndSpecSampler, vec3 (curTerrainUV, 0.0)) * inpWeights.b;
		return vec4 (retVal.xyz, 1.0);
	}
	return texture (roughnessAndSpecSampler, vec3 (curUV, 0.0));
}

bool getHasNrmMap (uint packedFlags)
{
	return (packedFlags & 0x00000001) != 0;
}

vec3 getFragmentNormal (vec3 inpWeights, vec2 inUV, vec3 posAtSample, bool isTerrain, mat3 tanSpace, uint packedFlags, uint chitInstanceId)
{
	if (getHasNrmMap(packedFlags))
		return normalize (tanSpace * sampleNormal (inpWeights, isTerrain, tanSpace[2], posAtSample, inUV, chitInstanceId));
	else
		return normalize (tanSpace[2]);
}

bool getHasRghMap (uint packedFlags)
{
	return (packedFlags & 0x00000002) != 0;
}

bool getIsTerrain (uint chitInstanceId)
{
	return textureSize (diffuseSampler, 0).z > 1;
}

bool getDielectric (uint packedFlags)
{
	return (packedFlags & 0x00000040) != 0;
}

TriHitData getTriHitData (vec3 barycoords, uint packedFlags, float inEmissivity, float inIoR, TriangleFromVertBuf hitTri, uint chitInstanceId)
{
	TriHitData retVal;

	retVal.pos = hitTri.e1Col1.xyz * barycoords.x + hitTri.e2Col2.xyz * barycoords.y + hitTri.e3Col3.xyz * barycoords.z;
	retVal.faceNorm = normalize (cross (hitTri.e1Col1.xyz - hitTri.e2Col2.xyz, hitTri.e3Col3.xyz - hitTri.e2Col2.xyz));
	vec3 vNorm = fromZSignXY (hitTri.Norm1);
	if ( dot (retVal.faceNorm, vNorm) < 0.0 ) retVal.faceNorm = -retVal.faceNorm;
	retVal.tangent = normalize (cross (vNorm, vNorm + vec3 (0.1)));
	retVal.bitan = cross (retVal.faceNorm, retVal.tangent);
	vec4 v1Col = unpackUnorm4x8(floatBitsToUint(hitTri.e1Col1.w));
	vec2 hitUV = hitTri.uv1.xy * barycoords.x + hitTri.uv2.xy * barycoords.y + hitTri.uv3.xy * barycoords.z;
	bool isTerrain = getIsTerrain (chitInstanceId);
	
	vec3 terrainSampleWeights = vec3 (0.0);
	if ( isTerrain ) terrainSampleWeights = sampleTerrainWeights (hitUV, chitInstanceId);
	
	mat3 tanSpace;
	tanSpace[2] = retVal.faceNorm;
	tanSpace[0] = retVal.tangent;
	tanSpace[1] = retVal.bitan;
	retVal.norm = getFragmentNormal (terrainSampleWeights, hitUV, retVal.pos, isTerrain, tanSpace, packedFlags, chitInstanceId);
	
	retVal.albedoColor = sampleDiffuse (terrainSampleWeights, isTerrain, retVal.faceNorm, retVal.pos, hitUV, chitInstanceId).rgb;
	retVal.albedoColor.rgb = pow(retVal.albedoColor.rgb, vec3(2.2));
	vec3 roughnessAndSpecFetch = vec3 (1.0, 1.0, 0.0);
	if (getHasRghMap(packedFlags)) roughnessAndSpecFetch = sampleRoughness (terrainSampleWeights, isTerrain, retVal.faceNorm, retVal.pos, hitUV, chitInstanceId).rgb;
	retVal.anisoRoughness = roughnessAndSpecFetch.xy;
	retVal.specAmount = roughnessAndSpecFetch.z;
	retVal.specularColor = sampleSpecular (terrainSampleWeights, isTerrain, retVal.faceNorm, retVal.pos, hitUV, chitInstanceId).rgb;
	retVal.specularColor.rgb = pow(retVal.specularColor.rgb, vec3(2.2));
	retVal.emissivity = inEmissivity;
	if ( v1Col.x > 0.0 ) retVal.emissivity *= v1Col.x;
	retVal.IoR = inIoR;

	retVal.diElectric = getDielectric(packedFlags);
	
	return retVal;
}

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
void barycentricCoords(vec3 p, vec3 a, vec3 b, vec3 c, out vec3 uvw)
{
	vec3 v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = dot(v0, v0);
	float d01 = dot(v0, v1);
	float d11 = dot(v1, v1);
	float d20 = dot(v2, v0);
	float d21 = dot(v2, v1);
	float invDenom = 1.0 / (d00 * d11 - d01 * d01);
	uvw.y = (d11 * d20 - d01 * d21) * invDenom;
	uvw.z = (d00 * d21 - d01 * d20) * invDenom;
	uvw.x = 1.0 - uvw.y - uvw.z;
}

uint getChildProcessed (int stackPointer, uint childProcessed3, uint childProcessed2, uint childProcessed1)
{
	int bitStartIndex = int (stackPointer * 2);
	if ( bitStartIndex >= 64 )
	{
		bitStartIndex -= 64;
		return bitfieldExtract (childProcessed3, bitStartIndex, 2);
	}
	else if ( bitStartIndex >= 32 )
	{
		bitStartIndex -= 32;
		return bitfieldExtract (childProcessed2, bitStartIndex, 2);
	}
	else
	{
		return bitfieldExtract (childProcessed1, bitStartIndex, 2);
	}
}

void setChildProcessed (int stackPointer, inout uint childProcessed3, inout uint childProcessed2, inout uint childProcessed1, uint setVal)
{
	int bitStartIndex = int (stackPointer * 2);
	if ( bitStartIndex >= 64 )
	{
		bitStartIndex -= 64;
		childProcessed3 = bitfieldInsert (childProcessed3, setVal, bitStartIndex, 2);
	}
	else if ( bitStartIndex >= 32 )
	{
		bitStartIndex -= 32;
		childProcessed2 = bitfieldInsert (childProcessed2, setVal, bitStartIndex, 2);
	}
	else
	{
		childProcessed1 = bitfieldInsert (childProcessed1, setVal, bitStartIndex, 2);
	}
}

bool LineWorld (vec3 p1, vec3 p2, inout TriHitData triHitData)
{
	uint nodeStack[33] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
						  0, 0, 0};
	uint childProcessed3 = 0, childProcessed2 = 0, childProcessed1 = 0;
	int stackPointer = 0;
	nodeStack[stackPointer] = 0;

	vec3 m = p2 - p1;
	vec3 invm = 1.0 / m;
	bool intersectionFound = false;
	
	uint chitInstanceId, chitTriId;
	vec3 chitBarycoords;

	while (true)
	{
		BVHInternalNode curNode = nodeArr.nodes[nodeStack[stackPointer]];
		vec3 nodeAABBMin = curNode.aabbMinChildA.xyz;
		vec3 nodeAABBMax = curNode.aabbMaxChildB.xyz;
		
		if ( floatBitsToUint (curNode.aabbMinChildA.a) != floatBitsToUint (curNode.aabbMaxChildB.a) )
		{
			uint curChildProcessed = getChildProcessed (stackPointer, childProcessed3, childProcessed2, childProcessed1);
			if ( curChildProcessed < 2 )
			{
				if ( curChildProcessed == 1 || rayBox (p1, invm, nodeAABBMin, nodeAABBMax) )
				{
					setChildProcessed (stackPointer, childProcessed3, childProcessed2, childProcessed1, curChildProcessed + 1);
					stackPointer++;
					nodeStack[stackPointer] = (curChildProcessed == 0) ? floatBitsToUint (curNode.aabbMinChildA.a) : floatBitsToUint (curNode.aabbMaxChildB.a);
					setChildProcessed (stackPointer, childProcessed3, childProcessed2, childProcessed1, 0);
				}
				else
				{
					stackPointer--;
					if ( stackPointer == -1 ) break;
				}
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
			BVHTriangleCompressed curTri = triArr.tris[floatBitsToUint (curNode.aabbMinChildA.a)];
			vec3 e1, e2, e3;
			vec3 aabbLen = nodeAABBMax - nodeAABBMin;
			
			e1.x = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.x & 0xFF000000) >> 24) / 255.0);
			e1.y = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.x & 0x00FF0000) >> 16) / 255.0);
			e1.z = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.x & 0x0000FF00) >>  8) / 255.0);
			
			e2.x = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.y & 0xFF000000) >> 24) / 255.0);
			e2.y = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.y & 0x00FF0000) >> 16) / 255.0);
			e2.z = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.y & 0x0000FF00) >>  8) / 255.0);
			
			e3.x = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.z & 0xFF000000) >> 24) / 255.0);
			e3.y = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.z & 0x00FF0000) >> 16) / 255.0);
			e3.z = (float ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.z & 0x0000FF00) >>  8) / 255.0);
			
			e1 = e1 * aabbLen + nodeAABBMin;
			e2 = e2 * aabbLen + nodeAABBMin;
			e3 = e3 * aabbLen + nodeAABBMin;

			if ( lineSegTri (p1, m, e1, e2, e3, curK) )
			{
				vec3 foundP2 = p1 + m * curK;
				vec3 bcCoords;
				barycentricCoords (foundP2, e1, e2, e3, bcCoords);
				if ((curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.w & 0x80000000) != 0u)
				{
					uint opaqueCheckInstanceId, opaqueCheckTriId;
					opaqueCheckInstanceId  = (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.x & 0x000000FF) << 16;
					opaqueCheckInstanceId |= (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.y & 0x000000FF) << 8;
					opaqueCheckInstanceId |= (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.z & 0x000000FF);
					opaqueCheckTriId = curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.w & 0x7FFFFFFF;
					TriangleFromVertBuf opaqueCheckTri = sourceGeom[nonuniformEXT(opaqueCheckInstanceId)].tri[opaqueCheckTriId];
					vec2 opaqueCheckUV = opaqueCheckTri.uv1.xy * bcCoords.x + opaqueCheckTri.uv2.xy * bcCoords.y + opaqueCheckTri.uv3.xy * bcCoords.z;
					if ( sampleDiffuse (vec3 (0.0), false, vec3(0.0), foundP2, opaqueCheckUV, opaqueCheckInstanceId).a > 0.0 )
					{
						m = foundP2 - p1;
						invm = 1.0 / m;
						chitBarycoords = bcCoords;
						chitInstanceId = opaqueCheckInstanceId;
						chitTriId = opaqueCheckTriId;
						intersectionFound = true;
					}
				}
				else
				{
					m = foundP2 - p1;
					invm = 1.0 / m;
					chitBarycoords = bcCoords;
					chitInstanceId  = (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.x & 0x000000FF) << 16;
					chitInstanceId |= (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.y & 0x000000FF) << 8;
					chitInstanceId |= (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.z & 0x000000FF);
					chitTriId = (curTri.e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID.w & 0x7FFFFFFF);
					intersectionFound = true;
				}
			}
			stackPointer--;
			if ( stackPointer == -1 ) break;
		}
	}

	if (intersectionFound)
	{
		InstanceProps chitProps = instanceInfo.props[chitInstanceId];
		TriangleFromVertBuf chitTri = sourceGeom[nonuniformEXT(chitInstanceId)].tri[chitTriId];
		triHitData = getTriHitData (chitBarycoords, floatBitsToUint (chitProps.attribs1.x), chitProps.attribs1.y, chitProps.attribs1.z, chitTri, chitInstanceId);
	}
	return intersectionFound;
}

void main()
{
	vec3 screenMidPos = texelFetch (worldPosAttach, ivec2 (textureSize(worldPosAttach, 0) >> 1), 0).rgb;
	if ( screenMidPos == vec3 (0.0) ) screenMidPos = vec3 (1000000.0);
	dofProps.screenMidPos = vec4 (screenMidPos, 1.0);

	if ( dofProps.toScreenMidPosAlpha.xyz == vec3 (0.0) ) dofProps.toScreenMidPosAlpha.xyz = dofProps.screenMidPos.xyz;

	vec3 midPos = texelFetch (worldPosAttach, ivec2 (inUV * textureSize(worldPosAttach, 0)), 0).rgb;
	if ( midPos == vec3 (0.0) ) midPos = vec3 (1000000.0);
	float blurSize = floor (min (length (dofProps.toScreenMidPosAlpha.xyz - midPos) * dofProps.invCoCDist, 20.0));
	if (blurSize == 0.0)
	{
		diffOutput = texelFetch (inpImage, ivec2 (inUV * textureSize(inpImage, 0)), 0);
		applyFinalPost ();
	
		if (dofProps.blurDirection == 1.0)
		{
			vec3 startPos = texelFetch (worldPosAttach, ivec2 (gl_FragCoord.xy), 0).rgb;
			vec4 normalFetch = texelFetch(normalAttach, ivec2 (gl_FragCoord.xy), 0);
			vec3 eyeLoc = vec3 (frameMVP.lookEyeX.a, frameMVP.upEyeY.a, frameMVP.sideEyeZ.a);
			vec3 toPos = normalize (startPos - eyeLoc);
			vec3 vertNorm = fromZSignXY (floatBitsToUint (normalFetch.y));
			vec3 refVec = reflect (toPos, vertNorm);
			TriHitData triHitDat;
			if ( LineWorld (startPos + refVec * 0.033, startPos + refVec * 100000.0, triHitDat) )
				diffOutput.rgb *= triHitDat.albedoColor;
		}
		return ;
	}

	vec2 UVDir = vec2 (0.0);
	if (dofProps.blurDirection == 0.0)
		UVDir = vec2 (dofProps.invDimsX, 0.0);
	else
		UVDir = vec2 (0.0, dofProps.invDimsY);

	float invBlurTotalSize = 1.0 / (blurSize * 2.0);

	vec4 accumSamples = vec4 (0.0);
	float accumWeights = 0.0;
	for (int i = -int(blurSize);i != int(blurSize)+1;i++)
	{
		vec2 curUV = clamp (inUV + float(i) * UVDir, vec2 (0.0), vec2 (1.0));
		float xDiff = ((float(i) + blurSize) * invBlurTotalSize) - 0.5;
		float weight = exp (-5.0 * (xDiff * xDiff));
		accumSamples += weight * texelFetch (inpImage, ivec2 (curUV * textureSize(inpImage, 0)), 0);
		accumWeights += weight;
	}

	if ( accumWeights == 0.0 )
		diffOutput = texelFetch (inpImage, ivec2 (inUV * textureSize(inpImage, 0)), 0);
	else
		diffOutput = accumSamples / accumWeights;

	applyFinalPost ();
}