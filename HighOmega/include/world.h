/*
	Copyright (c) 2023 TooMuchVoltage Software Inc.

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#ifndef HIGHOMEGA_WORLD_H
#define HIGHOMEGA_WORLD_H

#include <fiz-x.h>
#include <entities.h>
#include <geom.h>
#include "events.h"
#include "render.h"
#include "audio.h"
#include <thread>
#include <unordered_map>
#include <atomic>

using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::MESH;
using namespace HIGHOMEGA::FIZ_X;
using namespace HIGHOMEGA::EVENTS;
using namespace HIGHOMEGA::RENDER;
using namespace HIGHOMEGA::RENDER::PASSES;
using namespace HIGHOMEGA::AUDIO;
using namespace HIGHOMEGA::ENTITIES;

namespace HIGHOMEGA
{
	namespace WORLD
	{
		extern std::thread *PhysicsThread;
		extern std::thread *EntitiesThread;
		extern std::thread *ClothThread;
		extern std::thread *AudioThread;

		vec3 FindCenter(Mesh & inpMesh, std::function<bool(int, DataGroup &)> inpFilterFunction = [](int, DataGroup & inpGroup) -> bool {
			return true;
		});

		struct MeshModel
		{
			Mesh mesh;
			GraphicsModel *model;
		};
		class minMaxReductionClass
		{
		private:
			unsigned int WorkGroupMinMaxX();

		protected:
			struct minMaxParamsStruct
			{
				float minValue[4];
				float maxValueNumTris[4];
			};
			struct geomInfoStruct
			{
				GeometryClass *geom;
				mat4 orientCache;
				vec3 posCache;
			};
			bool modified = false;
			bool init = false;
			std::unordered_map <unsigned long long, geomInfoStruct> geomInfo;
			std::vector <unsigned long long> geomIds;
			std::vector <minMaxParamsStruct> geomParams;
			BufferClass *geomParamsBuf = nullptr;
			ShaderResourceSet *minMaxShader = nullptr;
			ComputeSubmission *minMaxSubmission = nullptr;

		public:
			unsigned long long RequestMinMax(GeometryClass *inGeom);
			void RemoveMinMaxRequest(unsigned long long requestId);
			void Process();
			void ClearContent();
		};
		extern minMaxReductionClass mainMinMaxReducer;

#define HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT 6
		class PlasteredItemsClass
		{
		private:
			std::vector <GroupedRenderSubmission *> submissionsForPlasteredItems;
			std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall;

			unsigned int WorkGroupTransformX();
			unsigned int WorkGroupTransformY();
			unsigned int WorkGroupIntegrateX();

		public:
			std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			float curTimeElapsed = 0.0f;
			struct transformSourceData
			{
				float mat[16];
			};
			struct integrateParamsStruct
			{
				float InstanceCountAmplitudePhaseCurTime[4];
			};
			struct transformParamsStruct
			{
				unsigned int triCountInstanceCount[2];
			} transformParams;
			ComputeSubmission *integrateSubmission = nullptr;
			ComputeSubmission *transformCollectionSubmission = nullptr;
			class plasterCollection
			{
			public:
				std::string meshModelKey;
				GraphicsModel *collectionModel;
				float wavePhase, waveAmplitude;
				unsigned int meshGroup, plasterGroup;

				BufferClass *transformInstancesSource = nullptr;
				BufferClass *transformInstances = nullptr;
				integrateParamsStruct integrateParams;
				BufferClass *integrateParamsBuf = nullptr;
				ShaderResourceSet *integrateShader = nullptr;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				std::vector <BufferClass *> transformParamsBuf;
				std::vector <ShaderResourceSet *> transformCollectionShader;

				std::vector <SubmittedRenderItem> allSubmissionInfos;
			};

			std::unordered_map <unsigned long long, std::vector<plasterCollection>> allCollections;
			
			void Combine(std::vector <PlasteredItemsClass> & plasteredClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall);
			unsigned long long Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*> &&submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducter);
			void Update(float elapseTime);
			void UpdateSDFs();
			void ForceRefreshSDFs(std::function<bool(MeshMaterial& curMat)> inpFilterFunction);
			void Remove(unsigned long long curId);
			void ClearContent();
		};

		extern PlasteredItemsClass plasteredItemsCollection;

		class ParticleSystemClass
		{
		private:
			ComputeSubmission *integrateSubmission = nullptr;
			ComputeSubmission *transformCollectionSubmission = nullptr;

		public:
			float curTimeElapsed = 0.0f;
			std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			struct ParticleItem
			{
				float pos[3];
				unsigned int planeNormal;
				unsigned int alphaHealthFadeRateShrinkRate;
				float scaleLineSpeed;
				unsigned int deathRateReserved;
			};
			struct EmitterParams
			{
				float posLinSpeedBase[4];
				float dirLinSpeedVar[4];
				float radAngSpeedBaseAngSpeedVarFadeRateBase[4];
				float fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[4];
				float deathRateVarInitialScaleInitialAlphaCurTime[4];
				unsigned int numParticles;
			};
			struct transformParamsStruct
			{
				unsigned int triCountInstanceCount[2];
			} transformParams;
			struct ParticleEmitter
			{
				MeshModel *meshModelRef = nullptr;
				GraphicsModel *collectionModel;
				unsigned int meshGroup, plasterGroup;

				std::vector<ParticleItem> particles;

				BufferClass *particlesBuf = nullptr;
				EmitterParams integrateParams;
				BufferClass *integrateParamsBuf = nullptr;
				ShaderResourceSet *integrateShader = nullptr;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				std::vector <BufferClass *> transformParamsBuf;
				std::vector <ShaderResourceSet *> transformCollectionShader;

				std::vector <SubmittedRenderItem> allSubmissionInfos;
			};
			std::unordered_map <unsigned long long, std::vector <ParticleEmitter>> allEmitters;

		private:
			bool integratedOnce = false;

			std::vector <GroupedRenderSubmission *> submissionsForParticle;
			std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall;
			void Add(ParticleEmitter & emitterRef);
			vec3 randNormVec();
			float randFract();
			unsigned int WorkGroupTransformX();
			unsigned int WorkGroupTransformY();
			unsigned int WorkGroupIntegrateX();

		public:
			unsigned long long Populate(Mesh & inpMesh, std::vector <GroupedRenderSubmission*> && subList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducter);
			void Remove(unsigned long long curId);
			void Combine(std::vector <ParticleSystemClass> & particleClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall);
			void Update(float elapseTime);
			void UpdateSDFs();
			void ForceRefreshSDFs(std::function<bool(MeshMaterial& curMat)> inpFilterFunction);
			void ClearContent();
		};

		extern ParticleSystemClass particleSystem;

		class GuidedModelSystemClass
		{
		private:
			struct rigidTransformParamsStruct
			{
				float matrices[32];
				unsigned int triCount;
			} rigidTransformParams;
			ComputeSubmission *transformRigidSubmission = nullptr;
			unsigned int WorkGroupTransformRigidX();
			std::vector<GroupedRenderSubmission*> submissionsForGuidedModels;
			std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall;

		public:
			std::unordered_map <std::string, Mesh> cachedMeshes;
			struct PathPoint
			{
				vec3 pos;
				float scale, duration;
				unsigned int order;
			};
			struct PathState
			{
				unsigned int curNum = 0;
				float curTimeElapsed = 0.0f;
				vec3 curPos, curDir;
				float curScale;
				Mesh *meshRef = nullptr;
				mat4 curTrans, curTransDT;
				mat4 prevTrans, prevTransDT;
				bool transOnce = false;
				unsigned int meshGroup = 0;

				GraphicsModel *transformedModel = nullptr;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				std::vector <BufferClass *> transformParamBufs;
				std::vector <ShaderResourceSet *> transformShader;

				std::vector <SubmittedRenderItem> allSubmissionInfos;
				std::vector <PathPoint> fullPath;
			};
			std::unordered_map <unsigned long long, std::vector <PathState>> allItems;

			unsigned long long Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*> &&subList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall);
			void Update(float elapseTime);
			void Combine(std::vector <GuidedModelSystemClass> & guidedModelsClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall);
			void Remove(unsigned long long curId);
			void UpdateSDFs();
			void ForceRefreshSDFs(std::function<bool(MeshMaterial& curMat)> inpFilterFunction);
			void ClearContent();
		};

		extern GuidedModelSystemClass guidedModelSystem;

		struct PipelineSetupReturn
		{
			bool changeMap = false;
			bool doCredits = false;
			std::string newMap;
			std::string newMapBelong;
		};

		enum CAMERA_ACTION
		{
			NONE = 0x00000000,
			FADE_IN = 0x00000001,
			FADE_OUT = 0x00000002,
			SHOW_AURORA = 0x00000004,
			SHOW_CLOUDS = 0x00000008,
			FORCE_SUN_ANGLE = 0x00000010,
			UNFORCE_SUN_ANGLE = 0x00000020,
			NEW_MAP = 0x00000040,
			CREDIT_ROLL = 0x00000080
		};

		class CameraSystemClass
		{
		private:
			unsigned int curNum = 0;
			float curTimeElapsed = 0.0f;

		public:
			struct Camera
			{
				vec3 pos, dir;
				float duration;
				unsigned int order;
				CAMERA_ACTION action;
				float actionParam;
				std::string newMap, newMapBelong;
			};
			float curAlpha = 1.0f;
			vec3 curPos, curDir;
			std::unordered_map <unsigned long long, std::vector <Camera>> allItems;

			unsigned long long Populate(Mesh & inpMesh);
			void Combine(std::vector <CameraSystemClass> & cameraRailClasses);
			void Remove(unsigned long long inpId);
			void Update(WorldParamsClass & WorldParams, PipelineSetupReturn & mapChangeDataStruct);
			void ClearContent();
		};

		extern CameraSystemClass cameraSystem;
		extern WorldParamsClass worldParams;

		class ZoneStreamingClass
		{
		private:
			struct zoneDesc
			{
				std::string name;
				bool operator==(const zoneDesc& other) const;
			};
			class ZoneDescHash
			{
			public:
				std::size_t operator()(const zoneDesc& k) const;
			};
			bool zoneStreamingActivated = false;
			std::unordered_map<std::string, std::vector<std::string>> zoneReferences;
			std::vector<zoneDesc> foundZones;
			std::vector<RigidBody *> rigidBodiesToAdd;
			void getTileNums(vec3 inpPos, int & tileX, int & tileY, int & tileZ);
			bool producingZones = false;
			std::vector<WorldParamsClass> worldParamsLoaders = std::vector<WorldParamsClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<GuidedModelSystemClass> guidedModelLoaders = std::vector<GuidedModelSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<PlasteredItemsClass> plasteredItemsLoaders = std::vector<PlasteredItemsClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<ParticleSystemClass> particleSystemLoaders = std::vector<ParticleSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<CameraSystemClass> cameraSystemLoaders = std::vector<CameraSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<minMaxReductionClass> perLoaderReducer = std::vector<minMaxReductionClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<LadderSystemClass> ladderSystemLoaders = std::vector<LadderSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::atomic<bool> producedZones[HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT] = { false, false, false, false, false, false };
			bool producedZonesOnce = false;
			std::vector <std::thread *> zoneProducerThread;
			std::mutex zone_producer_mutex;
			static void produceZones(ZoneStreamingClass *zoneStreamingPtr, unsigned int threadId);
			vec3 visibleMin, visibleMax;

		public:
			struct zone
			{
				GraphicsModel *graphicsModel = nullptr;
				RigidBody *rigidBody = nullptr;
				unsigned long long itemId = 0ull;
				unsigned long long plasteredId = 0ull;
				unsigned long long particlesId = 0ull;
				unsigned long long cameraRailId = 0ull;
				unsigned long long guidedModelsId = 0ull;
				unsigned long long worldParamsId = 0ull;
				unsigned long long laddersId = 0ull;
				std::vector <SubmittedRenderItem> allSubmissionInfos;
			};

			vec3 curPos;
			std::string zoneLocation = "";
			std::vector<GroupedRenderSubmission*> submissionList;
			std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall;
			std::unordered_map <zoneDesc, zone, ZoneDescHash> loadedZones;

			int tilesPerDrawRegionEdge();
			float unitsPerTileEdge();
			bool allZonesProduced();
			bool noZonesProduced();
			bool isZoneStreamingActivated();
			void setAllZonesNotProduced();
			void waitOnZoneProduction();
			void Create(std::string & inpZoneLocation, std::vector<GroupedRenderSubmission*> &&inpSubmissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall);
			vec3 & getVisbileMax();
			vec3 & getVisbileMin();
			void Update(vec3 & inpPos, bool forceUpdate = false);
			bool isInDrawRegion(vec3 & inEye, vec3 & objPos, float objRad);
			void ClearContent();
		};

		extern ZoneStreamingClass zoneStreaming;
		
		struct RagDollPiece
		{
			RigidBody *piece;
			mat4 refMatInv;
			std::string transformTarget;
		};
		class AnimatedMeshesClass
		{
		private:
			ComputeSubmission *animateSubmission = nullptr;

			unsigned int WorkGroupAnimateX();

		public:
			std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			struct animateParamsStruct
			{
				unsigned int triCount;
			} animateParams;
			class animatedModelItem
			{
			public:
				MeshModel *meshModelRef;
				float curAnimationPos, duration;
				GraphicsModel *animatedModel;
				bool ragDoll = false;
				std::vector<RagDollPiece> ragDollPieces;
				mat4 transMat;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				std::vector <BufferClass *> animateParamsBuf;
				std::vector <ShaderResourceSet *> animateShader;

				std::vector <SubmittedRenderItem> allSubmissionInfos;
				std::vector <GroupedRenderSubmission *> submissionsForAnimatedMeshes;
			};

			std::unordered_map <unsigned long long, animatedModelItem> allAnimatedModelItems;

			unsigned long long Add(std::string animatedModelFile, std::string animatedModelFileLoc, float duration, mat4 & inpTransMat, std::vector<RagDollPiece> *inpRagDollPieces, std::vector<GroupedRenderSubmission*> && subList);
			void Advance(unsigned long long animationId, float elapseTime, bool rollOver, mat4 & inpTransMat);
			void GetCurPosDuration(unsigned long long animationId, float & pos, float & duration);
			void Remove(unsigned long long animationId);
			void Process();
			void ClearContent();
		};

		extern AnimatedMeshesClass animatedMeshesCollection;

		class PhysicalItemClass
		{
		public:
			std::unordered_map <std::string, Mesh> cachedMeshes;
			struct rigidTransformParamsStruct
			{
				float matrices[32];
				unsigned int triCount;
			} rigidTransformParams;
			ComputeSubmission *transformRigidSubmission = nullptr;
			class RigidBodyItem
			{
			public:
				bool external = false;
				RigidBody *rigidBodyRef = nullptr;
				ConstraintCollection *constaintsRef = nullptr;
				GraphicsModel *modelRef = nullptr;
				mat4 prevTrans, prevTransDT;
				bool transOnce = false;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				std::vector <BufferClass *> transformParamBufs;
				std::vector <ShaderResourceSet *> transformShader;

				std::vector <SubmittedRenderItem> allSubmissionInfos;
				std::vector <SubmittedRenderItem> cachedSubmissions;
			};

		private:
			struct Destructible
			{
				DataBlock propBlock;
				std::vector<TriUV> sourceTriUVList;
				vec3 sourceMin, sourceMax;
				GraphicsModel *sourceModel;
				RigidBody *sourceBody;
			};
			std::unordered_map <std::string, Destructible> destructibles;
			std::mutex destructibleCacheMutex;
			struct BrokenPiece
			{
				std::vector <TriUV> geom;
				vec3 cent, aabbMin, aabbMax;
				float rad;
			};
			std::vector <GroupedRenderSubmission *> subListForDebris;
			std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCallForDebris;
			struct AddParams
			{
				std::string origPieceAction = std::string("");
				std::string newGroupId;
				std::string origGroupId;
				RigidBody *oobbPiece;
				MeshMaterial origMeshMaterial;
				std::vector <TriUV> triList;

				AddParams(std::string inpOrigPieceAction, std::string inpNewGroupId, RigidBody *inpOobbPiece, std::string inpOrigGroupId, std::vector <TriUV> inpTriList) :
					origPieceAction(inpOrigPieceAction), newGroupId(inpNewGroupId), oobbPiece(inpOobbPiece), origGroupId(inpOrigGroupId), triList(inpTriList) {}
			};
			std::vector<GraphicsModel *> recentlyAddedRigidModels;
			unsigned long long Add(std::string & newGroupId, RigidBody * origPiece, MeshMaterial & origMeshMaterial, const std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall, const std::vector<GroupedRenderSubmission *> submissionsToSubmitTo, std::vector <TriUV> & triList);
			unsigned int WorkGroupTransformRigidX();

			static void CutOutThread(PhysicalItemClass *physicalItemsCollection, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList);
			static void ShatterThread(PhysicalItemClass *physicalItemsCollection, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList);

		public:
			static std::thread *booleanOpThread;
			static std::atomic<bool> booleanOpThreadFinished;
			std::mutex destructionMutex;
			std::unordered_map <unsigned long long, RigidBodyItem> allItems;
			static std::vector<AddParams> deferredAdds;

			void ProcessDestructionCache(Mesh & inpMesh, GraphicsModel & inpModel, RigidBody & inpBody);
			unsigned long long Add(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos, bool isMap, std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, std::vector<GroupedRenderSubmission*> && subList);
			unsigned long long AddExternal(RigidBody * inpBody, GraphicsModel *inpModel, std::vector<SubmittedRenderItem> & inpSubs);
			void RemoveExternal(unsigned long long inpId);
			void AddRagDoll(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos, std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, std::vector<GroupedRenderSubmission*> && subList);
			void CutOut(RigidBody *inpRigidBody, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList);
			void Shatter(RigidBody *inpRigidBody, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList);
			void Update(std::function <bool(vec3 &, vec3 &, float)> isInDrawRegion);
			void ClearContent();
			void Remove(unsigned long long curId);
		};

		extern PhysicalItemClass physicalItemsCollection;

		inline CAMERA_ACTION operator|(CAMERA_ACTION a, CAMERA_ACTION b)
		{
			return static_cast<CAMERA_ACTION>(static_cast<unsigned int>(a) | static_cast<unsigned int>(b));
		}

		inline CAMERA_ACTION& operator|=(CAMERA_ACTION & a, CAMERA_ACTION b)
		{
			return a = static_cast<CAMERA_ACTION>(static_cast<unsigned int>(a) | static_cast<unsigned int>(b));
		}

		class DefaultPipelineSetupClass
		{
		private:
			//unsigned long long animId;
			//mat4 animMat;

			GroupedTraceSubmission mainRTSubmission;
			GroupedSDFBVHSubmission sdfBvhSubmission;
			MainMenuClass MainMenu;
			SkyDomeClass SkyDome;
			ClearSurfaceCacheClass ClearSurfaceCache;
			ShadowMapClass ShadowMapCascadeNear, ShadowMapCascadeFar;
			ShadowMapScreenClass ShadowMapScreen;
			VisibilityPassClass VisibilityPass;
			GatherResolveClass GatherResolve;
			PathTraceClass PathTrace;
			DoFClass DoF;
			TemporalAccumulateClass TemporalAccumulate;
			SpatialDenoiseClass SpatialDenoise;
			ModulateClass Modulate;
			NearScatteringClass NearScattering;
			ScreenSpaceGatherClass ScreenSpaceGather;
			ScreenSpaceFXClass ScreenSpaceFX;
			TriClass PostProcessTri;
			SplashDisplayClass SplashDisplay;
			bool ApplicationRebooting = false;

		public:
			DefaultPipelineSetupClass(std::string mapPath, std::string mapBelong, bool cmdOptHwRt, unsigned int cmdOptFullRes, bool cmdOptWindowed);
			PipelineSetupReturn Run();
			bool IsApplicationRebooting();
		};
		void CreateWorld(WorldParamsClass & WorldParams);
		void DestroyWorld(bool applicationExit);
	}
}

#endif