#version 460
#extension GL_EXT_ray_tracing : require

layout (binding = 4) uniform samplerCube skyBox;

layout (binding = 15, rgba16f) uniform image3D radiosityMapPX;
layout (binding = 16, rgba16f) uniform image3D radiosityMapNX;
layout (binding = 17, rgba16f) uniform image3D radiosityMapPY;
layout (binding = 18, rgba16f) uniform image3D radiosityMapNY;
layout (binding = 19, rgba16f) uniform image3D radiosityMapPZ;
layout (binding = 20, rgba16f) uniform image3D radiosityMapNZ;

layout (binding = 27) uniform VoxelizedInfoUBO
{
	vec4 mapMinVoxelSize;
	vec4 mapMaxDynamicFlag;
	vec4 invMapDims;
	vec4 gridSize;
} voxelizedInfo;

layout(location = 0) rayPayloadInEXT struct {
	vec3 contrib;
	vec3 explicitContrib;
	vec3 lastVertexNormal;
	vec3 lastVertexPos;
	bool lastVertexDiffuse;
	uint bounceCount;
	/*bool checkLuminare;
	bool hitHint;
	vec3 hintPos;
	vec3 hintNormal;
	vec3 luminareEnergy;*/
} hitValues;

ivec3 getUVWLoc (vec3 worldPos)
{
	vec3 posUVW = (worldPos - voxelizedInfo.mapMinVoxelSize.xyz) * voxelizedInfo.invMapDims.xyz;
	posUVW = clamp (posUVW, vec3 (0.0), vec3 (0.999999));
	return ivec3 (posUVW * voxelizedInfo.gridSize.xyz);
}

void addDiffuseRadiance (vec3 irrad, vec3 pos, vec3 dir)
{
	ivec3 uvwLoc = getUVWLoc (pos);
	vec3 xIrrad = dir.x * irrad;
	vec3 yIrrad = dir.y * irrad;
	vec3 zIrrad = dir.z * irrad;
	vec4 lightPX = imageLoad (radiosityMapPX, uvwLoc);
	vec4 lightNX = imageLoad (radiosityMapNX, uvwLoc);
	vec4 lightPY = imageLoad (radiosityMapPY, uvwLoc);
	vec4 lightNY = imageLoad (radiosityMapNY, uvwLoc);
	vec4 lightPZ = imageLoad (radiosityMapPZ, uvwLoc);
	vec4 lightNZ = imageLoad (radiosityMapNZ, uvwLoc);
	if ( dir.x > 0.0 ) lightPX += vec4 (xIrrad, 0.5);
	if ( dir.x < 0.0 ) lightNX += vec4 (-xIrrad, 0.5);
	if ( dir.y > 0.0 ) lightPY += vec4 (yIrrad, 0.5);
	if ( dir.y < 0.0 ) lightNY += vec4 (-yIrrad, 0.5);
	if ( dir.z > 0.0 ) lightPZ += vec4 (zIrrad, 0.5);
	if ( dir.z < 0.0 ) lightNZ += vec4 (-zIrrad, 0.5);
	imageStore (radiosityMapPX, uvwLoc, lightPX);
	imageStore (radiosityMapNX, uvwLoc, lightNX);
	imageStore (radiosityMapPY, uvwLoc, lightPY);
	imageStore (radiosityMapNY, uvwLoc, lightNY);
	imageStore (radiosityMapPZ, uvwLoc, lightPZ);
	imageStore (radiosityMapNZ, uvwLoc, lightNZ);
}

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyBox,dirToSample).xyz;
}

void main()
{
	/*if ( hitValues.checkLuminare )
	{
		hitValues.hitHint = false;
		return ; // This normally shouldn't happen O.o
	}*/
    hitValues.contrib *= getSkyValue (gl_WorldRayDirectionEXT);
	if ( hitValues.lastVertexDiffuse ) addDiffuseRadiance (getSkyValue (gl_WorldRayDirectionEXT), hitValues.lastVertexPos, gl_WorldRayDirectionEXT);
}