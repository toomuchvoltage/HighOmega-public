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

#include "sauray.h"

using namespace HIGHOMEGA;
using namespace HIGHOMEGA::WORLD;

extern HIGHOMEGA::INSTRUMENTATION::Instrument frameInstrument;

namespace HIGHOMEGA
{
	namespace SAURAY
	{
		class SaurayClientInterfaceClass
		{
		private:
			struct RandomizedSource
			{
				vec3 origin;
				vec3 listener;
				vec3 newDirection;
				float attenuationPower;
			};
			std::unordered_map <std::string, MeshMaterial> cachedMesheMaterials;
			std::unordered_map <unsigned int, RandomizedSource> randomizedAudioSources;
			struct GeomInstance
			{
				GraphicsModel *graphicsModel = nullptr;
				SubmittedRenderItem submitItem;
				bool modelSubmitted = false;
			};
			std::unordered_map <std::string, GeomInstance> allItems;
			unsigned int maxPlayers = 0;

		public:
			void Create(unsigned int inpMaxPlayers);
			void SetSampler(const std::string & samplerTag, unsigned char *inData, unsigned int inDataSize, ImageClass::PROVIDED_IMAGE_DATA_TYPE dataType, unsigned int w, unsigned int h, bool doMipMap, FORMAT inpFormat);
			void SetMaterial(const std::string & materialTag, const std::string & diffTag, const std::string & nrmTag, const std::string & rghTag, const std::string & spcTag,
				float emissivity, bool dielectric, float refractiveIndex, bool scattering, bool isAlphaKeyed, unsigned int playerId, unsigned char rayMask = 0xFFu);
			void SetGeom(const std::string & meshTag, const std::string & materialTag, std::vector <TriUV> & triList, bool immutable, GroupedRenderSubmission & renderSubmission);
			void RemoveGeom(std::string & meshTag, GroupedRenderSubmission & renderSubmission);
			void RandomizeAudioSource(unsigned int sourceId, vec3 actualListener, vec3 actualOrigin, vec3 & newOrigin, float randDistance, float updateDistanceThreshold);
			void Destroy(GroupedRenderSubmission & renderSubmission);
		};

		SaurayClientInterfaceClass saurayClientInterface;

		class SaurayPipelineSetupClass
		{
		private:
			SaurayDisplayTestClass SaurayDisplayTest;
			TriClass PostProcessTri;
			bool paramsWrong = false;
			bool PerfectSquare(unsigned int inpNum);

		public:
			GroupedTraceSubmission mainRTSubmission;
			std::thread saurayThread;
			bool saurayThreadStarted = false;
			SaurayTraceClass SaurayTrace;

			SaurayPipelineSetupClass();
			SaurayPipelineSetupClass(unsigned int inpMaxPlayers, unsigned int inpSideRes, unsigned int inpHistoryAmount);
			void Run();
		};

		SaurayPipelineSetupClass *DefaultPipelineSetup;
		bool saurayStarted = false;
	}
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::Create(unsigned int inpMaxPlayers)
{
	maxPlayers = inpMaxPlayers;
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::SetSampler(const std::string & samplerTag, unsigned char *inData, unsigned int inDataSize, ImageClass::PROVIDED_IMAGE_DATA_TYPE dataType, unsigned int w, unsigned int h, bool doMipMap, FORMAT inpFormat)
{
	if (TextureCache.find(samplerTag) != TextureCache.end())
	{
		TextureCache[samplerTag].elem.RemovePast();
	}
	try
	{
		TextureCache[samplerTag].elem.CreateTextureFromFileOrData(Instance, inData, inDataSize, dataType, w, h, 1u, false, true, false, doMipMap, inpFormat);
	}
	catch(...)
	{
		TextureCache.erase(samplerTag);
		LOG() << "Failed to create sampler with tag " << samplerTag;
		return;
	}
	TextureCache[samplerTag].elemCount = 1;
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::SetMaterial(const std::string & materialTag, const std::string & diffTag, const std::string & nrmTag, const std::string & rghTag, const std::string & spcTag, float emissivity, bool dielectric, float refractiveIndex, bool scattering, bool isAlphaKeyed, unsigned int playerId, unsigned char rayMask)
{
	if (TextureCache.find(diffTag) == TextureCache.end())
	{
		LOG() << "AddMaterial diffuse sampler with diffTag " << diffTag << " not found";
		return;
	}
	if (TextureCache.find(nrmTag) == TextureCache.end())
	{
		LOG() << "AddMaterial normal sampler with nrmTag " << nrmTag << " not found";
		return;
	}
	if (TextureCache.find(rghTag) == TextureCache.end())
	{
		LOG() << "AddMaterial roughness sampler with rghTag " << rghTag << " not found";
		return;
	}
	if (TextureCache.find(spcTag) == TextureCache.end())
	{
		LOG() << "AddMaterial specular sampler with spcTag " << spcTag << " not found";
		return;
	}
	if (playerId >= maxPlayers && playerId != 0xFFFFFFFFu)
	{
		LOG() << "AddMaterial playerId " << playerId << " higher than maxPlayers " << maxPlayers;
		return;
	}
	cachedMesheMaterials[materialTag].diffName = diffTag;
	cachedMesheMaterials[materialTag].diffRef = &TextureCache[diffTag];
	cachedMesheMaterials[materialTag].nrmName = nrmTag;
	cachedMesheMaterials[materialTag].nrmRef = &TextureCache[nrmTag];
	cachedMesheMaterials[materialTag].rghName = nrmTag;
	cachedMesheMaterials[materialTag].rghRef = &TextureCache[rghTag];
	cachedMesheMaterials[materialTag].hgtName = "";
	cachedMesheMaterials[materialTag].hgtRef = &TextureCache[diffTag];
	cachedMesheMaterials[materialTag].spcName = spcTag;
	cachedMesheMaterials[materialTag].spcRef = &TextureCache[spcTag];

	if (playerId != 0xFFFFFFFFu) isAlphaKeyed = true; // We use this to avoid self-intersection of frustum rays with the player geom that launched them...

	cachedMesheMaterials[materialTag].emissivity = emissivity;
	cachedMesheMaterials[materialTag].dielectric = dielectric;
	cachedMesheMaterials[materialTag].refractiveIndex = refractiveIndex;
	cachedMesheMaterials[materialTag].scattering = scattering;
	cachedMesheMaterials[materialTag].playerId = playerId;
	cachedMesheMaterials[materialTag].rayMask = rayMask;
	cachedMesheMaterials[materialTag].isAlphaKeyed = isAlphaKeyed;

	cachedMesheMaterials[materialTag].shaderName = std::string("default");
	cachedMesheMaterials[materialTag].smooth = false;
	cachedMesheMaterials[materialTag].mipmap = false;
	cachedMesheMaterials[materialTag].postProcess = false;
	cachedMesheMaterials[materialTag].backDropGlass = false;
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::SetGeom(const std::string & meshTag, const std::string & materialTag, std::vector<TriUV>& triList, bool immutable, GroupedRenderSubmission & renderSubmission)
{
	if (triList.size() == 0)
	{
		LOG() << "Cannot set geom with meshTag " << meshTag << " to have zero primitives";
		return;
	}
	if (cachedMesheMaterials.find(materialTag) == cachedMesheMaterials.end())
	{
		LOG() << "material with materialTag " << materialTag << " not found when making geom";
		return;
	}

	std::string meshTagCopy = meshTag;

	if (allItems[meshTag].graphicsModel == nullptr)
		allItems[meshTag].graphicsModel = new GraphicsModel(meshTagCopy, cachedMesheMaterials[materialTag], triList, false, immutable);
	else
		allItems[meshTag].graphicsModel->UpdateGeom(meshTagCopy, triList);

	if (!allItems[meshTag].modelSubmitted)
	{
		allItems[meshTag].submitItem = renderSubmission.Add(*allItems[meshTag].graphicsModel);
		allItems[meshTag].modelSubmitted = true;
	}
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::RemoveGeom(std::string & meshTag, GroupedRenderSubmission & renderSubmission)
{
	if (allItems.find(meshTag) != allItems.end())
	{
		if (allItems[meshTag].modelSubmitted)
		{
			renderSubmission.Remove(allItems[meshTag].submitItem);
			allItems[meshTag].modelSubmitted = false;
		}
	}
	else
		LOG() << "geom with meshTag " << meshTag << " was not found during removal attempt";
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::RandomizeAudioSource(unsigned int sourceId, vec3 actualListener, vec3 actualOrigin, vec3 & newOrigin, float randDistance, float updateDistanceThreshold)
{
	if (randomizedAudioSources.find(sourceId) == randomizedAudioSources.end() ||
		(randomizedAudioSources[sourceId].listener - actualListener).length() > updateDistanceThreshold ||
		(randomizedAudioSources[sourceId].origin - actualOrigin).length() > updateDistanceThreshold)
	{
		randomizedAudioSources[sourceId].listener = actualListener;
		randomizedAudioSources[sourceId].origin = actualOrigin;
		vec3 randVec = vec3((float)(rand() % 100 - 50), (float)(rand() % 100 - 50), (float)(rand() % 100 - 50)) * 0.02f * randDistance;
		randomizedAudioSources[sourceId].newDirection = (actualOrigin - actualListener) + randVec;
		randomizedAudioSources[sourceId].attenuationPower = 1.0f + (rand() % 100 - 50) * 0.004f;
	}

	float actualDist = (actualOrigin - actualListener).length();
	float cachedDist = (randomizedAudioSources[sourceId].origin - randomizedAudioSources[sourceId].listener).length();
	newOrigin = actualOrigin + powf(actualDist / cachedDist, randomizedAudioSources[sourceId].attenuationPower) * randomizedAudioSources[sourceId].newDirection;
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::Destroy(GroupedRenderSubmission & renderSubmission)
{
	for (std::pair<const std::string, GeomInstance> & curItem : allItems)
	{
		renderSubmission.Remove(curItem.second.submitItem);
		delete curItem.second.graphicsModel;
	}
	allItems.clear();
	cachedMesheMaterials.clear();
	randomizedAudioSources.clear();
}

bool HIGHOMEGA::SAURAY::SaurayPipelineSetupClass::PerfectSquare(unsigned int inpNum)
{
	double sqrtNum = ceil(sqrt((double)inpNum));
	return sqrtNum * sqrtNum == (double)inpNum;
}

HIGHOMEGA::SAURAY::SaurayPipelineSetupClass::SaurayPipelineSetupClass()
{
}

HIGHOMEGA::SAURAY::SaurayPipelineSetupClass::SaurayPipelineSetupClass(unsigned int inpMaxPlayers, unsigned int inpSideRes, unsigned int inpHistoryAmount)
{
	PostProcessTri.Create(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 1.77777778f, 65.0f);

	if (!(PerfectSquare(inpMaxPlayers) && inpMaxPlayers > 1))
	{
		LOG() << "inpMaxPlayers should be power of two and higher than one but is " << inpMaxPlayers;
		paramsWrong = true;
		return;
	}
	if (inpSideRes < 32)
	{
		LOG() << "inpSideRes should at least be 32 pixels wide";
		paramsWrong = true;
		return;
	}

	saurayClientInterface.Create(inpMaxPlayers);
	SaurayTrace.Create(mainRTSubmission, inpMaxPlayers, inpSideRes, inpHistoryAmount, sauray_debug_mode);
	if (sauray_debug_mode) SaurayDisplayTest.Create(PostProcessTri, SaurayTrace);
}

using namespace HIGHOMEGA::SAURAY;

float QuakeDegToRad(int deg)
{
	return deg * (HIGHOMEGA_PI / 180.0f);
}

vec3 QuakeAngleVectors(int qa_pitch, int qa_yaw, int qa_roll)
{
	float sp, sy, cp, cy;

	sy = sinf(QuakeDegToRad(qa_yaw));
	cy = cosf(QuakeDegToRad(qa_yaw));
	sp = sinf(QuakeDegToRad(qa_pitch));
	cp = cosf(QuakeDegToRad(qa_pitch));

	vec3 retVal;
	retVal.x = cp * cy;
	retVal.y = cp * sy;
	retVal.z = -sp;

	return retVal;
}

struct QueuedPlayer
{
	float absmin_x, absmin_y, absmin_z;
	float absmax_x, absmax_y, absmax_z;
	float e1x, e1y, e1z;
	float e2x, e2y, e2z;
	int qa1_pitch, qa1_yaw, qa1_roll;
	int qa2_pitch, qa2_yaw, qa2_roll;
	float yfov, whr;
};
std::unordered_map <unsigned int, QueuedPlayer> queuedPlayers;

void SaurayProcessPlayer(unsigned int player_id)
{
	float absmin_x, absmin_y, absmin_z,
		absmax_x, absmax_y, absmax_z,
		e1x, e1y, e1z,
		e2x, e2y, e2z;
	int qa1_pitch, qa1_yaw, qa1_roll,
		qa2_pitch, qa2_yaw, qa2_roll;
	float yfov, whr;

	absmin_x = queuedPlayers[player_id].absmin_x;
	absmin_y = queuedPlayers[player_id].absmin_y;
	absmin_z = queuedPlayers[player_id].absmin_z;
	absmax_x = queuedPlayers[player_id].absmax_x;
	absmax_y = queuedPlayers[player_id].absmax_y;
	absmax_z = queuedPlayers[player_id].absmax_z;
	e1x = queuedPlayers[player_id].e1x;
	e1y = queuedPlayers[player_id].e1y;
	e1z = queuedPlayers[player_id].e1z;
	e2x = queuedPlayers[player_id].e2x;
	e2y = queuedPlayers[player_id].e2y;
	e2z = queuedPlayers[player_id].e2z;
	qa1_pitch = queuedPlayers[player_id].qa1_pitch;
	qa1_yaw = queuedPlayers[player_id].qa1_yaw;
	qa1_roll = queuedPlayers[player_id].qa1_roll;
	qa2_pitch = queuedPlayers[player_id].qa2_pitch;
	qa2_yaw = queuedPlayers[player_id].qa2_yaw;
	qa2_roll = queuedPlayers[player_id].qa2_roll;
	yfov = queuedPlayers[player_id].yfov;
	whr = queuedPlayers[player_id].whr;

	bool enlargePlayers = false; // Determine if we have at least 1 lagging player...
	bool thisPlayerLagging = false; // Determine if THIS player is lagging
	for (int i = 0; i != DefaultPipelineSetup->SaurayTrace.playerFrusta.size(); i++)
		if (DefaultPipelineSetup->SaurayTrace.playerFrusta[i].eye2Whr[3] > 1.79f && (DefaultPipelineSetup->SaurayTrace.playerFrusta[i].maskEnabledReserved & 0x00FF0000))
		{
			enlargePlayers = true;
			break;
		}
	if (whr > 1.79f) thisPlayerLagging = true;
	unsigned char player_mask = 0xF0u, player_mask_enlarged = 0x0Fu, cast_mask;
	if (thisPlayerLagging) cast_mask = player_mask_enlarged;
	else                   cast_mask = player_mask;

	std::string curPlayerMaterial = std::string("player") + std::to_string(player_id) + std::string("Material");
	std::string curPlayerGeom = std::string("player") + std::to_string(player_id) + std::string("Geometry");
	std::string curPlayerMaterialEnlarged;
	std::string curPlayerGeomEnlarged;
	if (enlargePlayers)
	{
		curPlayerMaterialEnlarged = curPlayerMaterial + std::string("Enlarged");
		curPlayerGeomEnlarged = curPlayerGeom + std::string("Enlarged");
	}
	saurayClientInterface.SetMaterial(curPlayerMaterial,
		std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"),
		0.0f, false, 0.0f, false, false, player_id, player_mask);
	if (enlargePlayers)
	{
		saurayClientInterface.SetMaterial(curPlayerMaterialEnlarged,
			std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"),
			0.0f, false, 0.0f, false, false, player_id, player_mask_enlarged);
	}

	vec3 eye1 = vec3(e1x, e1y, e1z);
	vec3 eye2 = vec3(e2x, e2y, e2z);
	vec3 eyeDiff = (eye2 - eye1) * 0.5f;

	std::vector<TriUV> playerGeom, playerGeomEnlarged;
	if (enlargePlayers)
	{
		float *limits = DefaultPipelineSetup->SaurayTrace.playerLimits[player_id].aabbLim;

		MakeBox(vec3(absmin_x - limits[0], absmin_y - limits[2], absmin_z - limits[4]), vec3(absmax_x + limits[1], absmax_y + limits[3], absmax_z + limits[5]), playerGeomEnlarged);
		MakeBox(vec3(absmin_x - limits[6], absmin_y - limits[8], absmin_z - limits[10]) + eyeDiff, vec3(absmax_x + limits[7], absmax_y + limits[9], absmax_z + limits[11]) + eyeDiff, playerGeomEnlarged);
		MakeBox(vec3(absmin_x - limits[12], absmin_y - limits[14], absmin_z - limits[16]) + eyeDiff * 2.0f, vec3(absmax_x + limits[13], absmax_y + limits[15], absmax_z + limits[17]) + eyeDiff * 2.0f, playerGeomEnlarged);
	}
	MakeBox(vec3(absmin_x, absmin_y, absmin_z), vec3(absmax_x, absmax_y, absmax_z), playerGeom);
	MakeBox(vec3(absmin_x, absmin_y, absmin_z) + eyeDiff, vec3(absmax_x, absmax_y, absmax_z) + eyeDiff, playerGeom);
	MakeBox(vec3(absmin_x, absmin_y, absmin_z) + eyeDiff * 2.0f, vec3(absmax_x, absmax_y, absmax_z) + eyeDiff * 2.0f, playerGeom);

	saurayClientInterface.SetGeom(curPlayerGeom, curPlayerMaterial, playerGeom, false, DefaultPipelineSetup->mainRTSubmission);
	if (enlargePlayers) saurayClientInterface.SetGeom(curPlayerGeomEnlarged, curPlayerMaterialEnlarged, playerGeomEnlarged, false, DefaultPipelineSetup->mainRTSubmission);

	std::vector<TriUV> *envelopingGeom = &playerGeom;
	if (enlargePlayers) envelopingGeom = &playerGeomEnlarged;
	vec3 geomMin, geomMax;
	for (int i = 0; i != envelopingGeom->size(); i++)
	{
		if (i == 0)
		{
			geomMin.x = Min((*envelopingGeom)[i].eArr[0].x, (*envelopingGeom)[i].eArr[1].x, (*envelopingGeom)[i].eArr[2].x);
			geomMin.y = Min((*envelopingGeom)[i].eArr[0].y, (*envelopingGeom)[i].eArr[1].y, (*envelopingGeom)[i].eArr[2].y);
			geomMin.z = Min((*envelopingGeom)[i].eArr[0].z, (*envelopingGeom)[i].eArr[1].z, (*envelopingGeom)[i].eArr[2].z);

			geomMax.x = Max((*envelopingGeom)[i].eArr[0].x, (*envelopingGeom)[i].eArr[1].x, (*envelopingGeom)[i].eArr[2].x);
			geomMax.y = Max((*envelopingGeom)[i].eArr[0].y, (*envelopingGeom)[i].eArr[1].y, (*envelopingGeom)[i].eArr[2].y);
			geomMax.z = Max((*envelopingGeom)[i].eArr[0].z, (*envelopingGeom)[i].eArr[1].z, (*envelopingGeom)[i].eArr[2].z);
		}
		else
		{
			geomMin.x = min(geomMin.x, Min((*envelopingGeom)[i].eArr[0].x, (*envelopingGeom)[i].eArr[1].x, (*envelopingGeom)[i].eArr[2].x));
			geomMin.y = min(geomMin.y, Min((*envelopingGeom)[i].eArr[0].y, (*envelopingGeom)[i].eArr[1].y, (*envelopingGeom)[i].eArr[2].y));
			geomMin.z = min(geomMin.z, Min((*envelopingGeom)[i].eArr[0].z, (*envelopingGeom)[i].eArr[1].z, (*envelopingGeom)[i].eArr[2].z));

			geomMax.x = max(geomMax.x, Max((*envelopingGeom)[i].eArr[0].x, (*envelopingGeom)[i].eArr[1].x, (*envelopingGeom)[i].eArr[2].x));
			geomMax.y = max(geomMax.y, Max((*envelopingGeom)[i].eArr[0].y, (*envelopingGeom)[i].eArr[1].y, (*envelopingGeom)[i].eArr[2].y));
			geomMax.z = max(geomMax.z, Max((*envelopingGeom)[i].eArr[0].z, (*envelopingGeom)[i].eArr[1].z, (*envelopingGeom)[i].eArr[2].z));
		}
	}

	vec3 geomCent = (geomMin + geomMax) * 0.5f;
	float geomRad = (geomCent - geomMax).length();

	vec3 look1 = QuakeAngleVectors(qa1_pitch, qa1_yaw, qa1_roll).normalized();
	vec3 look2 = QuakeAngleVectors(qa2_pitch, qa2_yaw, qa2_roll).normalized();
	DefaultPipelineSetup->SaurayTrace.SetPlayer(player_id, cast_mask, eye1, look1, vec3(0.0f, 0.0f, 1.0f),
		eye2, look2, vec3(0.0f, 0.0f, 1.0f),
		yfov, whr, geomCent, geomRad);
}

std::vector <VkSemaphore> globalWaitSemaphores;

void HIGHOMEGA::SAURAY::SaurayPipelineSetupClass::Run()
{
	if (paramsWrong)
	{
		LOG() << "Not starting SauRay: supplied params were wrong.";
		return;
	}

	if (mainRTSubmission.rtScene.allTraceItems.size() == 0)
	{
		LOG() << "No geom to trace against...";
		return;
	}
	INSTRUMENTATION::FPSCounter::Start();
	waitSemaphores = globalWaitSemaphores;
	frameInstrument.Start();
	for (std::pair<const unsigned int, QueuedPlayer> & curPlayer : queuedPlayers) {
		SaurayProcessPlayer(curPlayer.first);
	}
	SaurayTrace.PrePass();
	for (std::pair<const unsigned int, QueuedPlayer> & curPlayer : queuedPlayers) {
		SaurayProcessPlayer(curPlayer.first);
	}
	SaurayTrace.Render();
	if (sauray_debug_mode) SaurayDisplayTest.Render();
	queuedPlayers.clear();
	frameInstrument.End();
	globalWaitSemaphores = waitSemaphores; // We (potentially) submit to the GPU from different threads, but the semaphore objects are the same
	INSTRUMENTATION::FPSCounter::End();
	INSTRUMENTATION::Instrument::EnableGlobally();

	INSTRUMENTATION::FPSCounter::Report([&](unsigned int inpFPS) {
		LOG() << "FPS: " << inpFPS << " Frame instrumentation: " << frameInstrument.ResultsMs();
	});
}

/************************************
  C Interface part of the source code
************************************/

int sauray_debug_mode = 0;
unsigned int sauray_debug_w = 1280;
unsigned int sauray_debug_h = 1280;
extern HIGHOMEGA::RENDER::ScreenSizeClass HIGHOMEGA::RENDER::ScreenSize;

void sauray_setdebug(unsigned int debug_w, unsigned int debug_h)
{
	sauray_debug_w = debug_w;
	sauray_debug_h = debug_h;
	sauray_debug_mode = 1;
}

int sauray_start(unsigned int max_players, unsigned int player_trace_res, unsigned int sauray_temporal_history_amount)
{
	try {
		if (saurayStarted)
		{
			saurayStarted = false;

			saurayClientInterface.Destroy(DefaultPipelineSetup->mainRTSubmission);

			delete DefaultPipelineSetup; // Simulate an engine re-boot
			globalWaitSemaphores.clear();
		}
		else
		{
			ScreenSize.Create(sauray_debug_w, sauray_debug_h);
			InitGraphicsSubSystem(true, true, sauray_debug_mode ? false : true);
		}

		DefaultPipelineSetup = new SaurayPipelineSetupClass(max_players, player_trace_res, sauray_temporal_history_amount);
		saurayStarted = true;

		return 0;
	}
	catch (std::runtime_error retErr) {
		LOG() << "Sauray start-up issue: " << retErr.what();
		return -1;
	}
	catch (const std::bad_alloc& e) {
		std::string outError = "Allocation failure: ";
		outError += e.what();
		LOG() << outError;
		return -1;
	}
}

int sauray_feedmap_quake2(char *mapName)
{
	unsigned int fileSize;
	unsigned char *fileContent;
	if (ResourceLoader::LoadFile(std::string("assets/maps/"+std::string(mapName)+".txt"), &fileContent, fileSize) != ResourceLoader::FILE_LOAD_SUCCESS)
	{
		LOG() << "Could not load map: assets/maps/" << mapName << ".txt";
		return -1;
	}
	LOG() << "Loaded assets/maps/" << mapName << ".txt successfully";
	float *mapTris = (float *)fileContent;
	unsigned int nTris = fileSize / (9 * sizeof(float));

	unsigned int RGBAdata = 0xFFFFFFFFu;
	saurayClientInterface.SetSampler(std::string("genericSampler"), (unsigned char *)&RGBAdata, sizeof(unsigned int), ImageClass::PROVIDED_IMAGE_DATA_TYPE::IMAGE_DATA_RGB, 1, 1, false, R8G8B8A8UN);
	saurayClientInterface.SetMaterial(std::string("mapMaterial"),
									  std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"),
									  0.0f, false, 0.0f, false, false, 0xFFFFFFFFu);

	std::vector<TriUV> mapGeom;
	TriUV curTri;
	mapGeom.resize(nTris);
	for (int i = 0; i != nTris; i++)
	{
		curTri.eArr[0].x = mapTris[i * 9];
		curTri.eArr[0].y = mapTris[i * 9 + 1];
		curTri.eArr[0].z = mapTris[i * 9 + 2];

		curTri.eArr[1].x = mapTris[i * 9 + 3];
		curTri.eArr[1].y = mapTris[i * 9 + 4];
		curTri.eArr[1].z = mapTris[i * 9 + 5];

		curTri.eArr[2].x = mapTris[i * 9 + 6];
		curTri.eArr[2].y = mapTris[i * 9 + 7];
		curTri.eArr[2].z = mapTris[i * 9 + 8];
		mapGeom[i] = curTri;
	}
	saurayClientInterface.SetGeom(std::string("mapGeometry"), std::string("mapMaterial"), mapGeom, true, DefaultPipelineSetup->mainRTSubmission);
	delete fileContent;

	return 0;
}

void sauray_player_quake2(unsigned int player_id,
	float absmin_x, float absmin_y, float absmin_z,
	float absmax_x, float absmax_y, float absmax_z,
	float e1x, float e1y, float e1z,
	float e2x, float e2y, float e2z,
	int qa1_pitch, int qa1_yaw, int qa1_roll,
	int qa2_pitch, int qa2_yaw, int qa2_roll,
	float yfov, float whr)
{
	queuedPlayers[player_id] = {
		absmin_x, absmin_y, absmin_z,
		absmax_x, absmax_y, absmax_z,
		e1x, e1y, e1z,
		e2x, e2y, e2z,
		qa1_pitch, qa1_yaw, qa1_roll,
		qa2_pitch, qa2_yaw, qa2_roll,
		yfov, whr
	};
}

int sauray_can_see_quake2(unsigned int viewer, unsigned int subject)
{
	return (int)DefaultPipelineSetup->SaurayTrace.CanSee(viewer, subject);
}

void sauray_randomize_audio_source(unsigned int listenerId, float listenerX, float listenerY, float listenerZ, float originX, float originY, float originZ, float* retOriginX, float* retOriginY, float* retOriginZ, float randDistance, float updateDistanceThreshold)
{
	vec3 outputOrigin;
	saurayClientInterface.RandomizeAudioSource(listenerId, vec3(listenerX, listenerY, listenerZ), vec3(originX, originY, originZ), outputOrigin, randDistance, updateDistanceThreshold);
	*retOriginX = outputOrigin.x;
	*retOriginY = outputOrigin.y;
	*retOriginZ = outputOrigin.z;
}

void sauray_remove_player(unsigned int player)
{
	std::string curPlayerGeom = std::string("player") + std::to_string(player) + std::string("Geometry");

	DefaultPipelineSetup->SaurayTrace.RemovePlayer(player);
	saurayClientInterface.RemoveGeom(curPlayerGeom, DefaultPipelineSetup->mainRTSubmission);
}

int sauray_loop()
{
	if (!saurayStarted)
	{
		LOG() << "Sauray runtime issue: Sauray not initialized";
		return -1;
	}

	try {
		DefaultPipelineSetup->Run();

		return 0;
	} catch (std::runtime_error retErr) {
		LOG() << "Sauray runtime issue: " << retErr.what();
		return -1;
	}
	catch (const std::bad_alloc& e) {
		std::string outError = "Allocation failure: ";
		outError += e.what();
		LOG() << outError;
		return -1;
	}
}

int sauray_thread_start()
{
	if (!saurayStarted)
	{
		LOG() << "Sauray runtime issue: Sauray not initialized";
		return -1;
	}
	if (DefaultPipelineSetup->saurayThreadStarted)
	{
		LOG() << "Sauray thread already started";
		return -1;
	}

	// Sauray thrown errors won't be caught... make sure you don't have errors to begin with...
	DefaultPipelineSetup->saurayThread = std::thread (&SaurayPipelineSetupClass::Run, DefaultPipelineSetup);
	DefaultPipelineSetup->saurayThreadStarted = true;
	return 0;
}

int sauray_thread_join()
{
	if (!DefaultPipelineSetup->saurayThreadStarted)
	{
		LOG() << "Sauray thread not started";
		return -1;
	}

	DefaultPipelineSetup->saurayThread.join();
	DefaultPipelineSetup->saurayThreadStarted = false;
	return 0;
}
