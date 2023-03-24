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
#extension GL_EXT_scalar_block_layout : require

layout (binding = 0, set = 0) uniform accelerationStructureEXT topLevelAS;

layout(location = 0) rayPayloadInEXT struct {
	/*
		2 bits:  hitType
		1 bit:   secondary ray flag
		29 bits: originating player Id
	*/
	uint hitTypeSecFlagOrigPlayerId;
} rayLoad;

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

void main()
{
	if (gl_IncomingRayFlagsEXT == gl_RayFlagsCullNoOpaqueEXT)
	{
		uint curPlayerId = bitfieldExtract(rayLoad.hitTypeSecFlagOrigPlayerId, 0, 29);
		uint cullMask = bitfieldExtract(playersInfo.frusta[curPlayerId].maskEnabledReserved, 24, 8);
		uint rayFlags = gl_RayFlagsSkipClosestHitShaderEXT | gl_RayFlagsCullOpaqueEXT;
		float tmin = 0.1;
		float tmax = 1000000.0;
		traceRayEXT(topLevelAS, rayFlags, cullMask, 0 , 0 , 0 , gl_WorldRayOriginEXT, tmin, gl_WorldRayDirectionEXT, tmax, 0 );
	}
}