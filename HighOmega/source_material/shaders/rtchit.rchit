/*
	Copyright (c) 2026 TooMuchVoltage Software Inc.

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require
#extension GL_EXT_shader_16bit_storage : require
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_shader_explicit_arithmetic_types_float16 : require

layout(location = 0) rayPayloadInEXT struct TriangleHit {
	uint albedoEmissivity;
	vec3 particleEmissivity;
	vec4 particleBlending;
	uint specularSpecularity;
	mat3 tanSpace;
	vec3 hitPos;
	uint decalAlbedo;
	uint decalSpecularSpecularity;
	mat3 decalTanSpace;
	vec3 decalHitPos;
	bool didHit;
} hitValues;

hitAttributeEXT vec3 hitBaryCoord;

struct TriangleFromVertBufWide
{
	vec4 e1Col1;
	vec2 uv1;
	uint Norm1;
	vec4 e2Col2;
	vec2 uv2;
	uint Norm2;
	vec4 e3Col3;
	vec2 uv3;
	uint Norm3;
};

#define ReadTri(o,i,j) {uint triCount = giantVB.dat[i];\
						uint offsetToIndices = i + 2, offsetToVertices = 2 + (triCount * 3); offsetToVertices += i + ((24 - ((offsetToVertices * 4) % 24 /* sizeof(RasterVertex) */ )) / 4);\
						uint e1Offset = offsetToVertices + giantVB.dat[offsetToIndices + j * 3] * 6;\
						uint e2Offset = offsetToVertices + giantVB.dat[offsetToIndices + j * 3 + 1] * 6;\
						uint e3Offset = offsetToVertices + giantVB.dat[offsetToIndices + j * 3 + 2] * 6;\
						o.e1Col1 = vec4 (uintBitsToFloat(giantVB.dat[e1Offset]), uintBitsToFloat(giantVB.dat[e1Offset + 1]), uintBitsToFloat(giantVB.dat[e1Offset + 2]), uintBitsToFloat(giantVB.dat[e1Offset + 3]));\
						o.uv1 = unpackHalf2x16(giantVB.dat[e1Offset + 4]);\
						o.Norm1 = giantVB.dat[e1Offset + 5];\
						o.e2Col2 = vec4 (uintBitsToFloat(giantVB.dat[e2Offset]), uintBitsToFloat(giantVB.dat[e2Offset + 1]), uintBitsToFloat(giantVB.dat[e2Offset + 2]), uintBitsToFloat(giantVB.dat[e2Offset + 3]));\
						o.uv2 = unpackHalf2x16(giantVB.dat[e2Offset + 4]);\
						o.Norm2 = giantVB.dat[e2Offset + 5];\
						o.e3Col3 = vec4 (uintBitsToFloat(giantVB.dat[e3Offset]), uintBitsToFloat(giantVB.dat[e3Offset + 1]), uintBitsToFloat(giantVB.dat[e3Offset + 2]), uintBitsToFloat(giantVB.dat[e3Offset + 3]));\
						o.uv3 = unpackHalf2x16(giantVB.dat[e3Offset + 4]);\
						o.Norm3 = giantVB.dat[e3Offset + 5];}

layout(set = 1, binding = 0) uniform sampler2DArray sceneTextures[];

struct InstanceProps
{
	vec3 geomMin;
	vec3 geomMax;
    vec3 attribs1;
    vec4 attribs2;
	uint16_t splatOffset;
	uint16_t diffOffset;
	uint16_t nrmOffset;
	uint16_t rghOffset;
	uint16_t hgtOffset;
	uint16_t spcOffset;
	uint idxVertOffset;
	uint transformOffset;
	uint prevTransformOffset;
};

layout (scalar, set = 2, binding = 0) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

layout(scalar, set = 2, binding = 1) buffer GiantVertBuffer
{
	uint dat[];
} giantVB;

layout (set = 2, binding = 2) buffer TransformsSSBO
{
	mat4 mats[];
} transforms;

#define diffuseSampler sceneTextures[nonuniformEXT(instanceInfo.props[gl_InstanceCustomIndexEXT].diffOffset)]
#define normalSampler sceneTextures[nonuniformEXT(instanceInfo.props[gl_InstanceCustomIndexEXT].nrmOffset)]
#define roughnessAndSpecSampler sceneTextures[nonuniformEXT(instanceInfo.props[gl_InstanceCustomIndexEXT].rghOffset)]
#define specularSampler sceneTextures[nonuniformEXT(instanceInfo.props[gl_InstanceCustomIndexEXT].spcOffset)]
#define curInstInfo instanceInfo.props[nonuniformEXT(gl_InstanceCustomIndexEXT)]
#define curTransform transforms.mats[instanceInfo.props[nonuniformEXT(gl_InstanceCustomIndexEXT)].transformOffset]

void GetTangentBiTangent(vec3 v_0, vec3 v_1, vec3 v_2, vec2 tex0, vec2 tex1, vec2 tex2, out vec3 x_1_out, out vec3 y_1_out)
{
	vec3 A = v_0 - v_1, B = v_2 - v_1;
	vec2 v1 = tex0 - tex1, v2 = tex2 - tex1, v3;
	vec3 origin, x_1, y_1;
	float a, b;

	float v1xv2y_v1yv2x = v1.x*v2.y - v1.y*v2.x;
	float inv_v1xv2y_v1yv2x;
	if (v1xv2y_v1yv2x != 0.0) inv_v1xv2y_v1yv2x = 1.0f / v1xv2y_v1yv2x;

	v3 = -tex1;
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;
	origin = v_1 + a * A + b * B;

	v3 = vec2(1.0, 0.0) - tex1;
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;

	x_1 = v_1 + a * A + b * B;

	v3 = vec2(0.0, 1.0) - tex1;
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;

	y_1 = v_1 + a * A + b * B;

	x_1_out = x_1 - origin;
	y_1_out = y_1 - origin;
}

vec3 sampleTerrainWeights (vec2 curUV)
{
	vec3 terrainMap = texture (diffuseSampler, vec3 (curUV, 3.0)).rgb;
	terrainMap /= (terrainMap.x + terrainMap.y + terrainMap.z);
	return terrainMap;
}

// https://www.shadertoy.com/view/4djSRW by Dave Hoskins
float hash12(vec2 p)
{
	vec3 p3  = fract(vec3(p.xyx) * .1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

// iq technique + suslik
// https://iquilezles.org/articles/texturerepetition/
// https://www.shadertoy.com/view/Xtl3zf
void computeDeTileOffsets (vec2 inCoord, out vec4 coordOffsets, out float mixFactor)
{
	inCoord *= 10.0;
	float k00 = hash12(floor(inCoord));
	float k01 = hash12(floor(inCoord) + vec2 (0.0, 1.0));
	float k10 = hash12(floor(inCoord) + vec2 (1.0, 0.0));
	float k11 = hash12(floor(inCoord) + vec2 (1.0, 1.0));
	vec2 inUVFrac = fract(inCoord);
	float k = mix(mix(k00, k01, inUVFrac.y), mix(k10, k11, inUVFrac.y), inUVFrac.x);

	float l = k*8.0;
	mixFactor = fract(l);

	float ia = floor(l+0.5);
	float ib = floor(l);
	mixFactor = min(mixFactor, 1.0-mixFactor)*2.0;

	coordOffsets.xy = sin(vec2(3.0,7.0)*ia);
	coordOffsets.zw = sin(vec2(3.0,7.0)*ib);
}

vec4 sampleDiffuse (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, vec4 coordOffsets, float mixFactor)
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
		vec3 retVal = vec3 (0.0);

		vec3 colLayer2a = texture(diffuseSampler, vec3 (curTerrainUV + coordOffsets.xy, 2.0)).xyz;
		vec3 colLayer2b = texture(diffuseSampler, vec3 (curTerrainUV + coordOffsets.zw, 2.0)).xyz;
		vec3 colLayer2Diff = colLayer2a - colLayer2b;
		vec3 colLayer2 = mix(colLayer2a, colLayer2b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer2Diff.x + colLayer2Diff.y + colLayer2Diff.z)));

		vec3 colLayer1a = texture(diffuseSampler, vec3 (curTerrainUV + coordOffsets.xy, 1.0)).xyz;
		vec3 colLayer1b = texture(diffuseSampler, vec3 (curTerrainUV + coordOffsets.zw, 1.0)).xyz;
		vec3 colLayer1Diff = colLayer1a - colLayer1b;
		vec3 colLayer1 = mix(colLayer1a, colLayer1b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer1Diff.x + colLayer1Diff.y + colLayer1Diff.z)));

		vec3 colLayer0a = texture(diffuseSampler, vec3 (curTerrainUV + coordOffsets.xy, 0.0)).xyz;
		vec3 colLayer0b = texture(diffuseSampler, vec3 (curTerrainUV + coordOffsets.zw, 0.0)).xyz;
		vec3 colLayer0Diff = colLayer0a - colLayer0b;
		vec3 colLayer0 = mix(colLayer0a, colLayer0b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer0Diff.x + colLayer0Diff.y + colLayer0Diff.z)));

		retVal += colLayer2 * inpWeights.r;
		retVal += colLayer1 * inpWeights.g;
		retVal += colLayer0 * inpWeights.b;
		return vec4 (retVal, 1.0);
	}
	return texture (diffuseSampler, vec3 (curUV, 0.0));
}

vec4 sampleSpecular (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, vec4 coordOffsets, float mixFactor)
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
		vec3 retVal = vec3 (0.0);

		vec3 colLayer2a = texture(specularSampler, vec3 (curTerrainUV + coordOffsets.xy, 2.0)).xyz;
		vec3 colLayer2b = texture(specularSampler, vec3 (curTerrainUV + coordOffsets.zw, 2.0)).xyz;
		vec3 colLayer2Diff = colLayer2a - colLayer2b;
		vec3 colLayer2 = mix(colLayer2a, colLayer2b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer2Diff.x + colLayer2Diff.y + colLayer2Diff.z)));

		vec3 colLayer1a = texture(specularSampler, vec3 (curTerrainUV + coordOffsets.xy, 1.0)).xyz;
		vec3 colLayer1b = texture(specularSampler, vec3 (curTerrainUV + coordOffsets.zw, 1.0)).xyz;
		vec3 colLayer1Diff = colLayer1a - colLayer1b;
		vec3 colLayer1 = mix(colLayer1a, colLayer1b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer1Diff.x + colLayer1Diff.y + colLayer1Diff.z)));

		vec3 colLayer0a = texture(specularSampler, vec3 (curTerrainUV + coordOffsets.xy, 0.0)).xyz;
		vec3 colLayer0b = texture(specularSampler, vec3 (curTerrainUV + coordOffsets.zw, 0.0)).xyz;
		vec3 colLayer0Diff = colLayer0a - colLayer0b;
		vec3 colLayer0 = mix(colLayer0a, colLayer0b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer0Diff.x + colLayer0Diff.y + colLayer0Diff.z)));

		retVal += colLayer2 * inpWeights.r;
		retVal += colLayer1 * inpWeights.g;
		retVal += colLayer0 * inpWeights.b;
		return vec4 (retVal, 1.0);
	}
	return texture (specularSampler, vec3 (curUV, 0.0));
}

vec3 sampleNormal (vec3 inpWeights, bool isTerrain, vec3 surfNorm, vec3 PosW, vec2 curUV, inout mat3 tanSpace, vec4 coordOffsets, float mixFactor)
{
	if ( isTerrain )
	{
		vec2 planarUV;
		vec3 absNorm = abs(surfNorm);
		if ( absNorm.y > 0.7 )
		{
			tanSpace[0] = vec3 (1.0, 0.0, 0.0);
			tanSpace[1] = vec3 (0.0, 0.0, 1.0);
			planarUV = PosW.xz;
		}
		else if ( absNorm.x > 0.7 )
		{
			tanSpace[0] = vec3 (0.0, 1.0, 0.0);
			tanSpace[1] = vec3 (0.0, 0.0, 1.0);
			planarUV = PosW.yz;
		}
		else
		{
			tanSpace[0] = vec3 (1.0, 0.0, 0.0);
			tanSpace[1] = vec3 (0.0, 1.0, 0.0);
			planarUV = PosW.xy;
		}
		vec2 planarFactor = vec2 (33.33333) / vec2 (textureSize (normalSampler, 0).xy);
		vec2 curTerrainUV = planarUV * planarFactor;
		vec3 retVal = vec3 (0.0);

		vec3 colLayer2a = normalize (texture(normalSampler, vec3 (curTerrainUV + coordOffsets.xy, 2.0)).xyz * 2.0 - vec3(1.0));
		vec3 colLayer2b = normalize (texture(normalSampler, vec3 (curTerrainUV + coordOffsets.zw, 2.0)).xyz * 2.0 - vec3(1.0));
		vec3 colLayer2Diff = colLayer2a - colLayer2b;
		vec3 colLayer2 = mix(colLayer2a, colLayer2b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer2Diff.x + colLayer2Diff.y + colLayer2Diff.z)));

		vec3 colLayer1a = normalize (texture(normalSampler, vec3 (curTerrainUV + coordOffsets.xy, 1.0)).xyz * 2.0 - vec3(1.0));
		vec3 colLayer1b = normalize (texture(normalSampler, vec3 (curTerrainUV + coordOffsets.zw, 1.0)).xyz * 2.0 - vec3(1.0));
		vec3 colLayer1Diff = colLayer1a - colLayer1b;
		vec3 colLayer1 = mix(colLayer1a, colLayer1b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer1Diff.x + colLayer1Diff.y + colLayer1Diff.z)));

		vec3 colLayer0a = normalize (texture(normalSampler, vec3 (curTerrainUV + coordOffsets.xy, 0.0)).xyz * 2.0 - vec3(1.0));
		vec3 colLayer0b = normalize (texture(normalSampler, vec3 (curTerrainUV + coordOffsets.zw, 0.0)).xyz * 2.0 - vec3(1.0));
		vec3 colLayer0Diff = colLayer0a - colLayer0b;
		vec3 colLayer0 = mix(colLayer0a, colLayer0b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer0Diff.x + colLayer0Diff.y + colLayer0Diff.z)));

		retVal += normalize (colLayer2) * inpWeights.r;
		retVal += normalize (colLayer1) * inpWeights.g;
		retVal += normalize (colLayer0) * inpWeights.b;
		return normalize (retVal);
	}
	return 2.0 * texture (normalSampler, vec3 (curUV, 0.0)).rgb - vec3 (1.0);
}

vec4 sampleRoughness (vec3 inpWeights, bool isTerrain, vec3 PosW, vec3 surfNorm, vec2 curUV, vec4 coordOffsets, float mixFactor)
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
		vec3 retVal = vec3 (0.0);

		vec3 colLayer2a = texture(roughnessAndSpecSampler, vec3 (curTerrainUV + coordOffsets.xy, 2.0)).xyz;
		vec3 colLayer2b = texture(roughnessAndSpecSampler, vec3 (curTerrainUV + coordOffsets.zw, 2.0)).xyz;
		vec3 colLayer2Diff = colLayer2a - colLayer2b;
		vec3 colLayer2 = mix(colLayer2a, colLayer2b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer2Diff.x + colLayer2Diff.y + colLayer2Diff.z)));

		vec3 colLayer1a = texture(roughnessAndSpecSampler, vec3 (curTerrainUV + coordOffsets.xy, 1.0)).xyz;
		vec3 colLayer1b = texture(roughnessAndSpecSampler, vec3 (curTerrainUV + coordOffsets.zw, 1.0)).xyz;
		vec3 colLayer1Diff = colLayer1a - colLayer1b;
		vec3 colLayer1 = mix(colLayer1a, colLayer1b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer1Diff.x + colLayer1Diff.y + colLayer1Diff.z)));

		vec3 colLayer0a = texture(roughnessAndSpecSampler, vec3 (curTerrainUV + coordOffsets.xy, 0.0)).xyz;
		vec3 colLayer0b = texture(roughnessAndSpecSampler, vec3 (curTerrainUV + coordOffsets.zw, 0.0)).xyz;
		vec3 colLayer0Diff = colLayer0a - colLayer0b;
		vec3 colLayer0 = mix(colLayer0a, colLayer0b, smoothstep(0.2, 0.8, mixFactor - 0.1 * (colLayer0Diff.x + colLayer0Diff.y + colLayer0Diff.z)));

		retVal += colLayer2 * inpWeights.r;
		retVal += colLayer1 * inpWeights.g;
		retVal += colLayer0 * inpWeights.b;
		return vec4 (retVal, 1.0);
	}
	return texture (roughnessAndSpecSampler, vec3 (curUV, 0.0));
}

bool getHasNrmMap (uint packedFlags)
{
	return (packedFlags & 0x00000001) != 0;
}

bool getHasRghMap (uint packedFlags)
{
	return (packedFlags & 0x00000002) != 0;
}

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

mat4 DirectionTransform(mat4 transform)
{
	mat4 retVal = mat4(1.0);
	retVal[0].xyz = cross(transform[1].xyz, transform[2].xyz);
	retVal[1].xyz = cross(transform[2].xyz, transform[0].xyz);
	retVal[2].xyz = cross(transform[0].xyz, transform[1].xyz);
	return retVal;
}

void main()
{
	TriangleFromVertBufWide hitTri;
	ReadTri (hitTri, curInstInfo.idxVertOffset, gl_PrimitiveID)
	vec3 barycoords = vec3(1.0 - hitBaryCoord.x - hitBaryCoord.y, hitBaryCoord.x, hitBaryCoord.y);

	vec2 hitUV = hitTri.uv1 * barycoords.x + hitTri.uv2 * barycoords.y + hitTri.uv3 * barycoords.z;

	vec3 triE1 = (curTransform * vec4 (hitTri.e1Col1.xyz, 1.0)).xyz;
	vec3 triE2 = (curTransform * vec4 (hitTri.e2Col2.xyz, 1.0)).xyz;
	vec3 triE3 = (curTransform * vec4 (hitTri.e3Col3.xyz, 1.0)).xyz;
	mat4 curTransformDT = DirectionTransform(curTransform);
	vec3 triN1 = normalize((curTransformDT * vec4 (fromZSignXY (hitTri.Norm1), 1.0)).xyz);
	vec3 triN2 = normalize((curTransformDT * vec4 (fromZSignXY (hitTri.Norm2), 1.0)).xyz);
	vec3 triN3 = normalize((curTransformDT * vec4 (fromZSignXY (hitTri.Norm3), 1.0)).xyz);
	
	vec3 curNorm = triN1 * barycoords.x + triN2 * barycoords.y + triN3 * barycoords.z;
	vec3 curFNorm = normalize (cross (triE1 - triE2, triE3 - triE2));
	if ( dot (curFNorm, curNorm) < 0.0 ) curFNorm = -curFNorm;
	hitValues.didHit = true;
	hitValues.hitPos = triE1 * barycoords.x + triE2 * barycoords.y + triE3 * barycoords.z;
	uint packedFlags = floatBitsToUint (curInstInfo.attribs1.x);
	vec3 tanSpace0, tanSpace1;
	GetTangentBiTangent (triE1, triE2, triE3, hitTri.uv1, hitTri.uv2, hitTri.uv3, tanSpace0, tanSpace1);
	hitValues.tanSpace[0] = normalize (tanSpace0);
	hitValues.tanSpace[1] = normalize (tanSpace1);
	hitValues.tanSpace[2] = curFNorm;
	vec3 finalNorm;


	vec4 coordOffsets = vec4(0.0);
	float mixFactor = 0.0;
	vec3 terrainSampleWeights = vec3 (0.0);
	bool isTerrain = textureSize (diffuseSampler, 0).z > 1;
	if ( isTerrain )
	{
		terrainSampleWeights = sampleTerrainWeights (hitUV);
		computeDeTileOffsets (hitUV, coordOffsets, mixFactor);
	}

	if ( getHasNrmMap (packedFlags) )
	{
		vec3 normFetch = sampleNormal (terrainSampleWeights, isTerrain, curFNorm, hitValues.hitPos, hitUV, hitValues.tanSpace, coordOffsets, mixFactor);
		finalNorm = normalize (hitValues.tanSpace * normFetch);
	}
	else
		finalNorm = hitValues.tanSpace[2];
	hitValues.tanSpace[2] = finalNorm;
	float emissivity = clamp (curInstInfo.attribs1.y / 25.5, 0.0, 1.0);

	float specularity = 0.0;
	if ( getHasRghMap (packedFlags) ) specularity = sampleRoughness (terrainSampleWeights, isTerrain, curFNorm, hitValues.hitPos, hitUV, coordOffsets, mixFactor).b;
	vec3 specular = sampleSpecular (terrainSampleWeights, isTerrain, curFNorm, hitValues.hitPos, hitUV, coordOffsets, mixFactor).rgb;
	vec3 albedo = sampleDiffuse (terrainSampleWeights, isTerrain, curFNorm, hitValues.hitPos, hitUV, coordOffsets, mixFactor).rgb;

	vec4 decalAlbedo = unpackUnorm4x8 (hitValues.decalAlbedo);
	float decalDot = dot (hitValues.hitPos - hitValues.decalHitPos, hitValues.decalTanSpace[2]);
	if (decalAlbedo.a > 0.0 && decalDot < 0.0 && decalDot > -1.0 )
	{
		albedo = mix (albedo, decalAlbedo.rgb, decalAlbedo.a);
		vec4 decalSpecularSpecularity = unpackUnorm4x8 (hitValues.decalSpecularSpecularity);
		specular = mix (specular, decalSpecularSpecularity.rgb, decalAlbedo.a);
		specularity = mix (specularity, decalSpecularSpecularity.a, decalAlbedo.a);
		hitValues.tanSpace = hitValues.decalTanSpace;
		hitValues.hitPos = hitValues.decalHitPos;
	}

	hitValues.albedoEmissivity = packUnorm4x8 (vec4 (albedo, emissivity));
	hitValues.specularSpecularity = packUnorm4x8 (vec4 (specular, specularity));
}