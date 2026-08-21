#version 460
#extension GL_NV_ray_tracing : require

layout(location = 0) rayPayloadInNV struct {
	float hitT;
} hitValues;
hitAttributeNV vec3 hitBaryCoord;

void main()
{
	hitValues.hitT = gl_HitTNV;
}