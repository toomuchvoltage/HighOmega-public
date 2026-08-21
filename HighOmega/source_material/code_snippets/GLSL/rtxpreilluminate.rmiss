#version 460
#extension GL_NV_ray_tracing : require

layout (binding = 7) uniform samplerCube skyBox;

layout(location = 0) rayPayloadInNV struct {
	vec3 radiance;
} hitValues;

vec3 getSkyValue (vec3 dirToSample)
{
	dirToSample.z = -dirToSample.z;
	return texture(skyBox,dirToSample).xyz;
}

void main()
{
    hitValues.radiance.xyz *= getSkyValue (gl_WorldRayDirectionNV);
}