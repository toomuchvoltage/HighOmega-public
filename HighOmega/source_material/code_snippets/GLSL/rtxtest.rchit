#version 460
#extension GL_NV_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require

#define diffuseSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5)]
#define normalSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 1)]
#define roughnessSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 2)]
#define specularSampler textures[nonuniformEXT(gl_InstanceCustomIndexNV * 5 + 4)]
#define curInstInfo instanceBuffers[nonuniformEXT(gl_InstanceCustomIndexNV)]
#define curTriBuf scene[nonuniformEXT(gl_InstanceCustomIndexNV)]

layout(location = 0) rayPayloadInNV struct {
	vec3 color;
} hitValues;
hitAttributeNV vec3 hitBaryCoord;

layout(set = 1, binding = 0) buffer InstanceInfo
{
	mat4 trans;
	mat4 transIT;
	vec4 attribs;
	vec4 attribs2;
} instanceBuffers[];

struct RTXTriangle
{
	vec4 e1u1;
	vec4 e2v1;
	vec4 e3u2;
	vec4 normv2;
	vec4 tanu3;
	vec4 bitanv3;
};

layout(set = 2, binding = 0) buffer CompleteTriBuffer
{
	RTXTriangle tri[];
} scene[];

layout(set = 3, binding = 0) uniform sampler2DArray textures[];

vec4 sampleDiffuse (vec2 inUV)
{
	return texture (diffuseSampler, vec3 (inUV, 0.0));
}

struct HitData
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

HitData getHitData ()
{
	HitData retVal;

	mat4 currentTrans = curInstInfo.trans;
	mat4 currentTransIT = curInstInfo.transIT;
	RTXTriangle hitTri = curTriBuf.tri[gl_PrimitiveID];
	vec3 barycoords = vec3(1.0 - hitBaryCoord.x - hitBaryCoord.y, hitBaryCoord.x, hitBaryCoord.y);

	vec2 hitUV = vec2 (hitTri.e1u1.w, hitTri.e2v1.w) * barycoords.x + vec2 (hitTri.e3u2.w, hitTri.normv2.w) * barycoords.y + vec2 (hitTri.tanu3.w, hitTri.bitanv3.w) * barycoords.z;

	retVal.albedoColor = sampleDiffuse (hitUV).rgb;

	return retVal;
}

void main()
{
	HitData curHitData = getHitData();

	hitValues.color = curHitData.albedoColor.rgb;
}