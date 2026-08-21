#version 460
#extension GL_NV_ray_tracing : require

layout (binding = 5) uniform samplerCube skyBox;

layout(location = 0) rayPayloadInNV struct {
	vec3 color;
} hitValues;

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	//return texture(skyBox,dirToSample).xyz;
	return vec3 (0.478, 0.576, 0.694);
}

void main()
{
    hitValues.color = getSkyValue (gl_WorldRayDirectionNV);
}