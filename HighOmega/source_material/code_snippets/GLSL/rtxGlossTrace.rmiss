#version 460
#extension GL_NV_ray_tracing : require

layout(location = 0) rayPayloadInNV struct {
	float emissivity;
	vec3 albedo;
	vec3 pos;
	mat3 tanSpace;
} hitValues;

void main()
{
	hitValues.pos = vec3 (0.0);
}