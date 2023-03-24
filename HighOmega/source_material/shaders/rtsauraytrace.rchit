/*
	Copyright © 2023 TooMuchVoltage Software Inc. This notice shall always be coupled with any SauRay(TM) implementation and must be redistributed alongside it.

	This implementation of US20220219086A1 is provided royalty free for either of the following:

	* Games with gross revenues of under one(1) million dollars CAD.
	* Games with at least a publically distributed moddable server binary with which SauRay(TM) is successfully integrable.

	Public distribution requires either a public download link or a relatively simple registration and download process. If you are unsure of your registration process's straightforwardness, reach out directly.

	Free open-source games (i.e. Cube/Sauerbraten or Xonotic) automatically qualify since successful SauRay(TM) integration is ultimately feasible with sufficient effort.

	Open source games with non-Libre licenses (i.e. non-GPL, non-MIT) also qualify as long as the license is no further restrictive than that of Quake(idTech) II's. If unsure of whether your source code redistribution license is permissive enough, please reach out directly.

	For games where at least the distributed server component is either open-source or moddable (in a manner permissible by the IP owner) the game must be sufficiently thin-client so that a SauRay(TM) integration does not result in crashes or defects that largely break the game in most multiplayer game modes. If you are unsure of whether your distributed binaries qualify for this category, please get in touch directly.

	We can be reached at the email address: sauray@toomuchvoltage.com or using the contact information found on the website http://sauray.tech .

	If your game does not qualify under either of the above categories, contact us for a commercial license. The covered source files are protected by copyright and the aforementioned terms will apply beyond the life of US20220219086A1.

	All games using US20220219086A1 or this implementation of it must clearly declare that they're using it in a way noticeable and comprehensible by an average player of the game in the English language.

	Beyond what is stated in http://toomuchvoltage.com/pub/sauray_techbrief/sauray_techbrief.pdf this source code does not provide any warranties of merchantability or fitness for any particular purpose.
*/

#version 460
#extension GL_EXT_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require
#extension GL_EXT_shader_16bit_storage : require
#extension GL_EXT_scalar_block_layout : require
#extension GL_EXT_shader_explicit_arithmetic_types_float16 : require

#define curInstInfo instanceInfo.props[nonuniformEXT(gl_InstanceCustomIndexEXT)]
#define curTriBuf scene[nonuniformEXT(gl_InstanceCustomIndexEXT)]

layout(location = 0) rayPayloadInEXT struct {
	/*
		2 bits:  hitType
		1 bit:   secondary ray flag
		29 bits: originating player Id
	*/
	uint hitTypeSecFlagOrigPlayerId;
} rayLoad;
hitAttributeEXT vec3 hitBaryCoord;

layout (binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

struct InstanceProps
{
	vec4 attribs;
	vec4 attribs2;
};

layout (binding = 3) buffer InstanceInfoSSBO
{
	InstanceProps props[];
} instanceInfo;

struct PlayerFrustum
{
	vec4 eyeGeomRad;
	vec4 eye2Whr;
	vec4 lookUpLook2Up2;
	vec4 geomCentYScale;
	uint maskEnabledReserved;
};

layout (scalar, binding = 4) buffer playersInfoSSBO
{
	PlayerFrustum frusta[];
} playersInfo;

layout (scalar, binding = 8) uniform SunUBO
{
	vec3 dir;
} sun;

bool getScattering (uint packedFlags)
{
	return (packedFlags & 0x00000100) != 0;
}

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

layout(scalar, set = 1, binding = 0) buffer CompleteTriBuffer
{
	TriangleFromVertBuf tri[];
} scene[];

void main()
{
	if (gl_IncomingRayFlagsEXT == gl_RayFlagsCullNoOpaqueEXT && bitfieldExtract(rayLoad.hitTypeSecFlagOrigPlayerId, 29, 1) == 0)
	{
		uint curPlayerId = bitfieldExtract(rayLoad.hitTypeSecFlagOrigPlayerId, 0, 29);
		uint cullMask = bitfieldExtract(playersInfo.frusta[curPlayerId].maskEnabledReserved, 24, 8);
		uint rayFlags = gl_RayFlagsSkipClosestHitShaderEXT | gl_RayFlagsCullOpaqueEXT;
		float tmin = 0.1;
		float tmax = gl_HitTEXT;
		traceRayEXT(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , gl_WorldRayOriginEXT, tmin, gl_WorldRayDirectionEXT, tmax, 0 );

		if ( bitfieldExtract(rayLoad.hitTypeSecFlagOrigPlayerId, 30, 2) == 0 )
			rayLoad.hitTypeSecFlagOrigPlayerId = bitfieldInsert (rayLoad.hitTypeSecFlagOrigPlayerId, 1, 30, 2); // if hitType is 0, set to 1

		rayLoad.hitTypeSecFlagOrigPlayerId = bitfieldInsert (rayLoad.hitTypeSecFlagOrigPlayerId, 1, 29, 1); // Set to secondary

		if ( !getScattering (floatBitsToUint (curInstInfo.attribs.x)) )
		{
			vec3 hitPos = gl_WorldRayOriginEXT + gl_WorldRayDirectionEXT * (gl_HitTEXT - 0.15);

			rayFlags = gl_RayFlagsCullNoOpaqueEXT;
			tmin = 0.1;
			tmax = 1000000.0;
			traceRayEXT(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , hitPos, tmin, sun.dir.xyz, tmax, 0 );
		}
	}
}