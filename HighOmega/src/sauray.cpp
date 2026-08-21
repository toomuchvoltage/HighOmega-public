/*
	Copyright © 2026 TooMuchVoltage Software Inc. This notice shall always be coupled with any SauRay(TM) implementation and must be redistributed alongside it.

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

extern "C" {
	__declspec(dllexport) DWORD NvOptimusEnablement = 0x00000001;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}

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
				GraphicsModelInstance * graphicsModelInst = nullptr;
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
			void Destroy();
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
			SaurayTraceClass SaurayTrace;

			SaurayPipelineSetupClass();
			SaurayPipelineSetupClass(unsigned int inpMaxPlayers, unsigned int inpSideRes, unsigned int inpHistoryAmount);
			void Run(bool threaded);
		};

		struct {
			volatile bool awaiting = false;
			volatile bool quit = false;
			std::mutex mutex;
			std::mutex quit_mutex;
			std::condition_variable condVar;
		} SignalData;
		std::thread *saurayThread = nullptr;
		bool saurayStarted = false;

		unsigned int cachedMaxPlayers, cachedPlayerTraceRes, cachedTemporalHistoryAmount;

		SaurayPipelineSetupClass *DefaultPipelineSetup = nullptr;
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
		allItems[meshTag].graphicsModel = new GraphicsModel(meshTagCopy, cachedMesheMaterials[materialTag], triList, immutable);
	else
		allItems[meshTag].graphicsModel->UpdateGeom(meshTagCopy, triList);

	if (!allItems[meshTag].modelSubmitted)
	{
		if (!allItems[meshTag].graphicsModelInst)
		{
			allItems[meshTag].graphicsModelInst = allItems[meshTag].graphicsModel->CreateInstance();
			renderSubmission.Add(*allItems[meshTag].graphicsModelInst);
		}
		allItems[meshTag].modelSubmitted = true;
	}
}

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::RemoveGeom(std::string & meshTag, GroupedRenderSubmission & renderSubmission)
{
	if (allItems.find(meshTag) != allItems.end())
	{
		if (allItems[meshTag].modelSubmitted)
		{
			allItems[meshTag].graphicsModelInst->modelRef->DestroyInstance(allItems[meshTag].graphicsModelInst);
			allItems[meshTag].graphicsModelInst = nullptr;
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

void HIGHOMEGA::SAURAY::SaurayClientInterfaceClass::Destroy()
{
	for (std::pair<const std::string, GeomInstance> & curItem : allItems)
		delete curItem.second.graphicsModel;
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
std::unordered_map <unsigned int, QueuedPlayer> queuedPlayersRenderThread;
std::vector<unsigned int> removePlayers;
std::vector<TriUV> mapGeom;
std::mutex envStateMutex;

void SaurayRemovePlayer(unsigned int player_id)
{
	std::string curPlayerGeom = std::string("player") + std::to_string(player_id) + std::string("Geometry");

	DefaultPipelineSetup->SaurayTrace.RemovePlayer(player_id);
	saurayClientInterface.RemoveGeom(curPlayerGeom, DefaultPipelineSetup->mainRTSubmission);
}

void SaurayProcessPlayer(unsigned int player_id)
{
	float absmin_x, absmin_y, absmin_z,
		absmax_x, absmax_y, absmax_z,
		e1x, e1y, e1z,
		e2x, e2y, e2z;
	int qa1_pitch, qa1_yaw, qa1_roll,
		qa2_pitch, qa2_yaw, qa2_roll;
	float yfov, whr;

	absmin_x = queuedPlayersRenderThread[player_id].absmin_x;
	absmin_y = queuedPlayersRenderThread[player_id].absmin_y;
	absmin_z = queuedPlayersRenderThread[player_id].absmin_z;
	absmax_x = queuedPlayersRenderThread[player_id].absmax_x;
	absmax_y = queuedPlayersRenderThread[player_id].absmax_y;
	absmax_z = queuedPlayersRenderThread[player_id].absmax_z;
	e1x = queuedPlayersRenderThread[player_id].e1x;
	e1y = queuedPlayersRenderThread[player_id].e1y;
	e1z = queuedPlayersRenderThread[player_id].e1z;
	e2x = queuedPlayersRenderThread[player_id].e2x;
	e2y = queuedPlayersRenderThread[player_id].e2y;
	e2z = queuedPlayersRenderThread[player_id].e2z;
	qa1_pitch = queuedPlayersRenderThread[player_id].qa1_pitch;
	qa1_yaw = queuedPlayersRenderThread[player_id].qa1_yaw;
	qa1_roll = queuedPlayersRenderThread[player_id].qa1_roll;
	qa2_pitch = queuedPlayersRenderThread[player_id].qa2_pitch;
	qa2_yaw = queuedPlayersRenderThread[player_id].qa2_yaw;
	qa2_roll = queuedPlayersRenderThread[player_id].qa2_roll;
	yfov = queuedPlayersRenderThread[player_id].yfov;
	whr = queuedPlayersRenderThread[player_id].whr;

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

void HIGHOMEGA::SAURAY::SaurayPipelineSetupClass::Run(bool threaded)
{
	if (threaded) DefaultPipelineSetup = new SaurayPipelineSetupClass(cachedMaxPlayers, cachedPlayerTraceRes, cachedTemporalHistoryAmount);

	for (;;)
	{
		if (DefaultPipelineSetup->paramsWrong)
		{
			LOG() << "Not starting SauRay: supplied params were wrong.";
			break;
		}

		if (DefaultPipelineSetup->mainRTSubmission.rtScene.allTraceItems.size() == 0 && mapGeom.size() > 0)
		{
			LOG() << "No geom to trace against...";
		}
		INSTRUMENTATION::FPSCounter::Start();
		frameInstrument.Start();
		{std::unique_lock <std::mutex> lk(envStateMutex);
		if (mapGeom.size() > 0)
		{
			unsigned int RGBAdata = 0xFFFFFFFFu;
			saurayClientInterface.SetSampler(std::string("genericSampler"), (unsigned char*)&RGBAdata, sizeof(unsigned int), ImageClass::PROVIDED_IMAGE_DATA_TYPE::IMAGE_DATA_RGB, 1, 1, false, R8G8B8A8UN);
			saurayClientInterface.SetMaterial(std::string("mapMaterial"),
				std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"), std::string("genericSampler"),
				0.0f, false, 0.0f, false, false, 0xFFFFFFFFu);
			saurayClientInterface.SetGeom(std::string("mapGeometry"), std::string("mapMaterial"), mapGeom, true, DefaultPipelineSetup->mainRTSubmission);
			mapGeom.clear();
		}
		for (unsigned int removePlayer : removePlayers)
			SaurayRemovePlayer(removePlayer);
		removePlayers.clear();
		queuedPlayersRenderThread = queuedPlayers;}
		for (std::pair<const unsigned int, QueuedPlayer>& curPlayer : queuedPlayersRenderThread) {
			SaurayProcessPlayer(curPlayer.first);
		}
		DefaultPipelineSetup->SaurayTrace.PrePass();
		for (std::pair<const unsigned int, QueuedPlayer>& curPlayer : queuedPlayersRenderThread) {
			SaurayProcessPlayer(curPlayer.first);
		}
		queuedPlayersRenderThread.clear();
		DefaultPipelineSetup->SaurayTrace.Render();
		if (sauray_debug_mode) DefaultPipelineSetup->SaurayDisplayTest.Render();
		frameInstrument.End();
		INSTRUMENTATION::FPSCounter::End();
		INSTRUMENTATION::Instrument::EnableGlobally();

		INSTRUMENTATION::FPSCounter::Report([&](unsigned int inpFPS) {
			LOG() << "FPS: " << inpFPS << " Frame instrumentation: " << frameInstrument.ResultsMs();
			});

		if (threaded)
		{
			{std::unique_lock <std::mutex> lk(SignalData.mutex);
			SignalData.awaiting = true;
			SignalData.condVar.notify_all();
			SignalData.condVar.wait(lk, [&awaiting = SignalData.awaiting] { return !awaiting; });}

			{std::unique_lock <std::mutex> lk(SignalData.quit_mutex);
			if (SignalData.quit) break; }
		}
		else
			break;
	}

	if (threaded)
	{
		saurayClientInterface.Destroy();
		delete DefaultPipelineSetup;
		DefaultPipelineSetup = nullptr;
	}
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

int sauray_start(unsigned int max_players, unsigned int player_trace_res, unsigned int sauray_temporal_history_amount, int sauray_threaded_mode)
{
	cachedMaxPlayers = max_players;
	cachedPlayerTraceRes = player_trace_res;
	cachedTemporalHistoryAmount = sauray_temporal_history_amount;

	try {
		if (saurayStarted)
		{
			if (saurayThread)
			{
				{std::unique_lock <std::mutex> lk(SignalData.mutex);
				std::unique_lock <std::mutex> lk2(SignalData.quit_mutex);
				SignalData.quit = true;
				SignalData.awaiting = false;
				SignalData.condVar.notify_all(); }
				saurayThread->join();
				delete saurayThread;
				saurayThread = nullptr;
			}
			else
			{
				if (DefaultPipelineSetup)
				{
					saurayClientInterface.Destroy();
					delete DefaultPipelineSetup;
					DefaultPipelineSetup = nullptr;
				}
			}
		}
		else
		{
			ScreenSize.Create(sauray_debug_w, sauray_debug_h);
			InitGraphicsSubSystem(true, WINDOW_MODE::WINDOWED, sauray_debug_mode ? false : true);
			saurayStarted = true;
		}

		SignalData.quit = false;
		SignalData.awaiting = false;
		queuedPlayers.clear();
		removePlayers.clear();
		mapGeom.clear();

		if (sauray_threaded_mode)
		{
			if (!saurayThread) saurayThread = new std::thread(&SaurayPipelineSetupClass::Run, DefaultPipelineSetup, true);
		}
		else
		{
			if (!DefaultPipelineSetup) DefaultPipelineSetup = new SaurayPipelineSetupClass(cachedMaxPlayers, cachedPlayerTraceRes, cachedTemporalHistoryAmount);
		}

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

	TriUV curTri;
	{std::unique_lock <std::mutex> lk(envStateMutex);
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
	}}
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
	if (!DefaultPipelineSetup)
	{
		LOG() << "Set player: Sauray not started properly";
		return ;
	}

	{std::unique_lock <std::mutex> lk(envStateMutex);
	queuedPlayers[player_id] = {
		absmin_x, absmin_y, absmin_z,
		absmax_x, absmax_y, absmax_z,
		e1x, e1y, e1z,
		e2x, e2y, e2z,
		qa1_pitch, qa1_yaw, qa1_roll,
		qa2_pitch, qa2_yaw, qa2_roll,
		yfov, whr
	};}
}

int sauray_can_see_quake2(unsigned int viewer, unsigned int subject)
{
	if (!DefaultPipelineSetup)
	{
		LOG() << "Can see: Sauray not started properly";
		return -1;
	}

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
	if (!DefaultPipelineSetup)
	{
		LOG() << "Remove player: Sauray not started properly";
		return ;
	}

	{std::unique_lock <std::mutex> lk(envStateMutex);
	queuedPlayers.erase(player);
	removePlayers.push_back(player); }
}

int sauray_loop()
{
	if (!saurayStarted)
	{
		LOG() << "Loop: Sauray not started";
		return -1;
	}
	if (saurayThread)
	{
		LOG() << "Loop: Sauray is already running in threaded mode. Cannot start synchronus mode.";
		return -1;
	}

	try {
		DefaultPipelineSetup->Run(false);

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

int sauray_thread_signal()
{
	if (!saurayStarted)
	{
		LOG() << "Signal: Sauray not started";
		return -1;
	}
	if (!saurayThread)
	{
		LOG() << "Signal: sauray thread not started";
		return -1;
	}

	{std::unique_lock <std::mutex> lk(SignalData.mutex);
	SignalData.awaiting = false;
	SignalData.condVar.notify_all(); }
	return 0;
}

int sauray_thread_wait()
{
	if (!saurayStarted)
	{
		LOG() << "Wait: Sauray not started";
		return -1;
	}
	if (!saurayThread)
	{
		LOG() << "Wait: Sauray thread not started";
		return -1;
	}

	{std::unique_lock <std::mutex> lk(SignalData.mutex);
	SignalData.condVar.wait(lk, [&awaiting = SignalData.awaiting] { return awaiting; });}
	return 0;
}
