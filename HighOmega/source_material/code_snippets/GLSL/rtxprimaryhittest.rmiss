#version 460
#extension GL_NV_ray_tracing : require

layout(location = 0) rayPayloadInNV struct {
	float hitT;
} hitValues;

void main()
{
	hitValues.hitT = -1.0;
}