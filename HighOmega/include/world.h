/*
	Copyright (c) 2026 TooMuchVoltage Software Inc.

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

#pragma once

#include <fiz-x.h>
#include <entities.h>
#include <geom.h>
#include "events.h"
#include "render.h"
#include "audio.h"
#include "config.h"
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
using namespace HIGHOMEGA::CONFIG;

namespace HIGHOMEGA
{
	namespace WORLD
	{
		extern std::thread *PhysicsThread;
		extern std::thread *EntitiesThread;
		extern std::thread *AudioThread;

		vec3 FindCenter(Mesh & inpMesh, std::function<bool(RigidBody*, int, DataGroup &)> inpFilterFunction = [](RigidBody*, int, DataGroup & inpGroup) -> bool {
			return true;
		});

		struct MeshModel
		{
			Mesh mesh;
			GraphicsModel *model;
			bool preparingOnAnotherThread = false;
			unsigned int claims = 0u;
		};
		class minMaxReductionClass
		{
		private:
			unsigned int WorkGroupMinMaxX();

		protected:
			struct minMaxParamsStruct
			{
				float minValueIdxVertOffset[4];
				float maxValueNumVerts[4];
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
		// Thread-safe cache for heavy duty object constructions (meshes and associated models). Supports busy-waiting by other threads on common loads.
		class ThreadSafeMeshModelCache
		{
		protected:
			MeshModel* InsertEntry(std::function<void(Mesh& outMesh, GraphicsModel** outModel)> buildEntryFunction, std::string& insertionKey, std::string&& owningSystem, std::unordered_map<std::string, MeshModel>& cachedMeshesModels, std::mutex& cachedMeshesModelsMutex);
			void CleanupCache(std::unordered_map <std::string, MeshModel>& cachedMeshesModels, std::mutex& cachedMeshesModelsMutex);
		};
		class PlasteredItemsClass : public ThreadSafeMeshModelCache
		{
		private:
			std::vector <GroupedRenderSubmission *> submissionsForPlasteredItems;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;

			unsigned int WorkGroupTransformX();
			unsigned int WorkGroupTransformY();
			unsigned int WorkGroupIntegrateX();

			static std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			static std::mutex cachedMeshesModelsMutex;

		public:
			float curTimeElapsed = 0.0f;
			struct transformSourceData
			{
				float mat[16];
			};
			struct transformData
			{
				float mat[16];
				float prevMat[16];
			};
			struct integrateParamsStruct
			{
				float InstanceCountAmplitudePhaseCurTime[4];
			};
			struct transformParamsStruct
			{
				unsigned int vertCount;
				unsigned int instanceCount;
				unsigned int geomInstCount;
				unsigned int srcIdxVertOffset;
				unsigned int dstIdxVertOffset;
			};
			ComputeSubmission* integrateSubmission = nullptr;
			ShaderResourceSet* integrateShader = nullptr;
			ComputeSubmission* transformCollectionSubmission = nullptr;
			ShaderResourceSet* transformCollectionShader = nullptr;
			bool rerecordSubmission = false;

			class plasterCollection
			{
			public:
				std::string meshModelKey;
				GraphicsModel* collectionModel;
				GraphicsModelInstance* collectionModelInst = nullptr;
				float wavePhase, waveAmplitude;

				BufferClass *transformInstancesSource = nullptr;
				BufferClass *transformInstances = nullptr;
				integrateParamsStruct integrateParams;
				BufferClass *integrateParamsBuf = nullptr;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				BufferClass *transformParamsBuf = nullptr;
			};

			std::unordered_map <unsigned long long, std::vector<plasterCollection>> allCollections;
			
			void Combine(std::vector <PlasteredItemsClass> & plasteredClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall);
			unsigned long long Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*> &&submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducer);
			void Update(float elapseTime);
			void UpdateSDFs(bool forceRefresh = false);
			void Remove(unsigned long long curId);
			void ClearContent();
		};

		extern PlasteredItemsClass plasteredItemsCollection;

		class ParticleSystemClass : public ThreadSafeMeshModelCache
		{
		private:
			ComputeSubmission* integrateSubmission = nullptr;
			ShaderResourceSet* integrateShader = nullptr;
			ComputeSubmission* integrateAgainstSceneSubmission = nullptr;
			ShaderResourceSet* integrateAgainstSceneShader = nullptr;
			ComputeSubmission* transformCollectionSubmission = nullptr;
			ShaderResourceSet* transformCollectionShader = nullptr;

			bool rerecordSubmission = false;

			static std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			static std::mutex cachedMeshesModelsMutex;

		public:
			enum REUSE_TAG
			{
				REUSE_NONE,
				REUSE_WOOD,
				REUSE_WOOD_DUST,
				REUSE_GLASS,
				REUSE_METAL,
				REUSE_CONCRETE
			};
			struct ParticleItem
			{
				float pos[3];
				unsigned int velocityDir;
				unsigned int xAxis;
				unsigned int yAxis;
				unsigned char alpha;
				unsigned char health;
				unsigned char fadeRate;
				unsigned char deathRate;
				float scaleLinSpeed;
				unsigned short scaleRate;
				unsigned char flags;
				unsigned char prevDeltaT;
				float angSpeed;
			};
			struct EmitterParams
			{
				float posLinSpeedBase[4];
				float dirGravDirGravLenLinSpeedVar[4];
				float radAngSpeedBaseAngSpeedVarFadeRateBase[4];
				float fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[4];
				float deathRateVarInitialScaleInitialAlphaCurTime[4];
				unsigned int numParticles;
				unsigned int elasticitySpreadFlags;
			};
			struct transformParamsStruct
			{
				unsigned int vertCount;
				unsigned int instanceCount;
				unsigned int geomInstCount;
				unsigned int srcIdxVertOffset;
				unsigned int dstIdxVertOffset;
			};
			class ParticleEmitter
			{
			public:
				unsigned long long emittersId;
				std::string particleMesh;
				std::string particleMeshAssetLoc;

				GraphicsModel* collectionModel;
				GraphicsModelInstance* collectionModelInst = nullptr;
				std::string originString;

				std::vector<ParticleItem> particles;
				REUSE_TAG reUseTag = REUSE_NONE;
				bool requiresScene = false;
				float elasticity = 0.5f;
				float spread = 0.0f;
				bool scaleToLinVel = false;
				vec3 emitDir = vec3(0.0f, 1.0f, 0.0f);
				float expireAfterTime = -1.0f, hideAfterTime = -1.0f;
				TimerObject expireTimer, hideTimer;
				bool hidden = false;
				bool noSDFLeaf = false;
				bool randomLives = false;
				bool forceMinMax = false;
				vec3 forcedMin, forcedMax;
				vec3 hidePos = vec3(0.0f);

				BufferClass *particlesBuf = nullptr;
				EmitterParams integrateParams;
				BufferClass *integrateParamsBuf = nullptr;

				std::vector <GeometryClass *> sourceGeom, destGeom;
				BufferClass *transformParamsBuf;

				void ClearForReuse();
				void Populate();
				void AttemptHideEmitter();
			};
			std::unordered_map <unsigned long long, std::vector <ParticleEmitter>> allEmitters;

		private:
			unsigned int frameNumber = 0ul;
			unsigned long long sceneID = 0ull;

			std::vector <GroupedRenderSubmission *> submissionsForParticle;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;
			std::vector <ParticleEmitter> sceneInteractingEmitters;
			static vec3 randNormVec();
			static float randFract();
			unsigned int WorkGroupTransformX();
			unsigned int WorkGroupTransformY();
			unsigned int WorkGroupIntegrateX();

		public:
			unsigned long long Populate(Mesh & inpMesh, std::vector <GroupedRenderSubmission*> && subList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducer);
			static void EncodeElasticity(ParticleEmitter& inpEmitter);
			static void EncodeAffectedByGravity(ParticleEmitter& inpEmitter, bool affectedByGravity);
			static void EncodeSpread(ParticleEmitter& inpEmitter, float spreadAmount);
			static void EncodeOrientToLinVel(ParticleEmitter& inpEmitter, bool orientToLinVel);
			static void EncodeScaleToLinVel(ParticleEmitter& inpEmitter, bool scaleToLinVel);
			static void EncodeRepeat(ParticleEmitter& inpEmitter, bool repeat);
			void AddEmitter(ParticleEmitter& inpEmitter, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission);
			void RecycleEmitter(ParticleEmitter& inpEmitter, minMaxReductionClass& inpMinMaxReducer);
			unsigned long long AddBulletEmitter(const vec3& bulletPos, const vec3& bulletDir, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission, HIGHOMEGA::FIZ_X::MATERIAL hitMat, bool cacheForRecycling = false);
			void Remove(unsigned long long curId);
			void Combine(std::vector <ParticleSystemClass> & particleClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall);
			void Update(float elapseTime, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission);
			void UpdateSDFs(bool onlySceneInteracting = false, bool forceRefresh = false);
			void ClearContent();
		};

		extern ParticleSystemClass particleSystem;

		class GuidedModelSystemClass : public ThreadSafeMeshModelCache
		{
		private:
			std::vector<GroupedRenderSubmission*> submissionsForGuidedModels;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;

			static std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			static std::mutex cachedMeshesModelsMutex;

		public:

			struct PathPoint
			{
				vec3 pos;
				float scale, duration;
				unsigned int order;
			};
			struct PathState
			{
				std::string meshModelKey = "";
				MeshModel* meshModelRef;
				unsigned int curNum = 0;
				float curTimeElapsed = 0.0f;
				vec3 curPos, curDir;
				float curScale;
				mat4 curTrans;

				GraphicsModelInstance* modelInst = nullptr;

				std::vector <PathPoint> fullPath;
			};
			std::unordered_map <unsigned long long, std::vector <PathState>> allItems;

			unsigned long long Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*> &&subList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall);
			void Update(float elapseTime);
			void Combine(std::vector <GuidedModelSystemClass> & guidedModelsClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall);
			void Remove(unsigned long long curId);
			void UpdateSDFs(bool forceRefresh = false);
			void ClearContent();
		};

		extern GuidedModelSystemClass guidedModelSystem;

		struct PipelineSetupReturn
		{
			bool doCredits = false;
			std::string newMap;
			std::string newMapBelong;
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
				std::unordered_map<std::string, float> actionParams;
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

		struct RagDollPiece
		{
			RigidBody* piece;
			mat4 refMatInv;
			std::string transformTarget;
		};
		class AnimatedMeshesClass : public ThreadSafeMeshModelCache
		{
		private:
			bool rerecordSubmission = false;
			BufferClass* animateParamsBuf = nullptr;
			ShaderResourceSet* animateShader = nullptr;
			ComputeSubmission* animateSubmission = nullptr;

			unsigned int WorkGroupAnimateX();
			static std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			static std::mutex cachedMeshesModelsMutex;

		public:
			struct animateParamsStruct
			{
				unsigned int vertCount;
				unsigned int srcIdxVertOffset;
				unsigned int dstIdxVertOffset;
				unsigned int nBones;
			};
			class AnimatedModelItem
			{
			public:
				struct refAnim
				{
					std::string key = "";
					float duration = 1.0f;
					MeshModel* meshModelRef = nullptr;
				};
				std::vector<refAnim> refAnims;
				int useRefAnim;
				std::string meshModelKey;
				MeshModel* meshModelRef;
				float curAnimationPos = 0.0f, duration = 1.0f;
				GraphicsModel* animatedModel;
				GraphicsModelInstance* animatedModelInst = nullptr;
				bool ragDoll = false;
				bool rollOver = true;
				bool autoAdvance = true;
				std::vector<RagDollPiece> ragDollPieces;
				mat4 transMat;

				std::vector <GeometryClass*> sourceGeom, destGeom;
			};

			std::vector <GroupedRenderSubmission*> subList;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;
			std::unordered_map <unsigned long long, AnimatedModelItem> allAnimatedModelItems;

			unsigned long long Add(std::string animatedModelFile, std::vector<AnimatedModelItem::refAnim>&& inRefAnims, std::string animatedModelFileLoc, float duration, bool rollOver, mat4& inpTransMat, std::vector<RagDollPiece>* inpRagDollPieces, bool autoAdvance = true);
			void SetAnimationState(unsigned long long animationId, int useRefAnim, float animationPos, mat4& inpTransMat);
			void GetCurPosDuration(unsigned long long animationId, int useRefAnim, float& pos, float& duration);
			void Remove(unsigned long long animationId);
			void Process(float elapseTime);
			void Combine(AnimatedMeshesClass& animatedMeshes, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall);
			void ClearContent();
		};

		class ZoneStreamingClass;
		class PhysicalItemClass : public ThreadSafeMeshModelCache
		{
			friend class ZoneStreamingClass;
			static std::unordered_map <std::string, MeshModel> cachedMeshesModels;
			static std::mutex cachedMeshesModelsMutex;

		public:
			enum CACHED_BULLETHOLE_TYPE
			{
				BULLETHOLE_GLASS,
				BULLETHOLE_WOOD,
				BULLETHOLE_METAL,
				BULLETHOLE_CONCRETE
			};
			struct CachedBulletHoleList
			{
				MeshModel* meshModelRef = nullptr;
				struct CachedBulletHole
				{
					GraphicsModelInstance* modelInstRef = nullptr;
					vec3 localTan, localNorm, localPos;
					unsigned long long itemId = 0ul;
				};
				std::vector<CachedBulletHole> cachedBulletHoles;
				unsigned int currentHoleToPick = 0;
			};

			class RigidBodyItem
			{
			public:
				std::string zoneLocation = "", meshName = "";
				MeshModel* meshModelRef = nullptr;
				RigidBody* rigidBodyRef = nullptr;
				ConstraintCollection* constaintsRef = nullptr;
				GraphicsModel* modelRef = nullptr;
				GraphicsModelInstance* modelInstRef = nullptr;
				bool addedToWorld = false;
				bool renderOnly = false, neverHide = false;
				mat4 renderMat;
				float renderRad;
				std::vector<unsigned long long> clothsAndWindSources;
				std::vector<unsigned long long> buoyancyRanges;
				std::vector<unsigned long long> animatedMeshes;
				std::vector<unsigned long long> attachedBodies;
				std::vector<unsigned long long> ragdolls;
			};
			std::unordered_map<CACHED_BULLETHOLE_TYPE, CachedBulletHoleList> bulletHoles;
			void RemoveBulletHolesForItem(unsigned long long itemId);
			void UpdateBulletHolesForItem(unsigned long long itemId, mat4& inMat, mat4& inMatDT, bool updatePrevTransform = false);

		private:
			struct GeomToSerializeOrDeSerialize
			{
				std::string name;
				std::vector<TriUV> geomTriUV;
				mat4 trans;
			};
			std::vector<GeomToSerializeOrDeSerialize> allGeomToSerializeOrDeSerialize;
			struct DestructibleMapGeomToDeSerialize
			{
				std::string name;
				std::vector<TriUV> sourceTriUVList;
				vec3 sourceMin, sourceMax;
			};
			std::vector<DestructibleMapGeomToDeSerialize> allDestructibleMapGeomToDeSerialize;
			struct Destructible
			{
				DataBlock propBlock;
				std::vector<TriUV> sourceTriUVList;
				vec3 sourceMin, sourceMax;
				GraphicsModel* sourceModel = nullptr;
				RigidBody* sourceBody = nullptr;
			};
			struct DestructibleCut
			{
				vec3 hitPt;
				vec3 hitNorm;
				vec3 hitDir;
			};
			struct DestructibleHitCluster
			{
				vec3 center;
				unsigned int hitCount = 0u;
			};
			struct DestructibleHitInfo
			{
				std::vector<DestructibleHitCluster> hitClusters;
				std::vector<DestructibleCut> cuts;
				unsigned shatterHitCount = 0u;
			};
			static std::unordered_map <std::string, Destructible> destructibles;
			static std::unordered_map <std::string, DestructibleHitInfo> destructibleHitInfo;
			static std::mutex destructibleCacheMutex;
			ClothCollectionClass clothCollection;
			bool resetPhysicsThreadBouyancyRangeCollection = false;
			bool cachedBulletEmitters = false;
			BuoyancyRangeCollectionClass physicalItemBuoyancyRangeCollection;
			struct BrokenPiece
			{
				std::vector <TriUV> geom;
				vec3 cent, aabbMin, aabbMax;
				float rad;
			};
			vec3 cachedPos = vec3(0.0f);
			std::vector <GroupedRenderSubmission*> subList;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;
			unsigned long long Add(std::string& newGroupId, RigidBody* origPiece, MeshMaterial& origMeshMaterial, std::vector <TriUV>& triList);

			static void CutOutThread(PhysicalItemClass* physicalItemsCollection, unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float cutRadius, float cutDepth);
			static void ShatterThread(PhysicalItemClass* physicalItemsCollection, unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm);
			void ProcessDestructionCache(Mesh& inpMesh, GraphicsModel& inpModel, RigidBody& inpBody);
			static void DoStaticTessellation(PhysicalItemClass* physicalItemsCollection, vec3 curPos, float standingHeight);
			void FinishStaticTessellation();

		public:
			AnimatedMeshesClass animatedMeshCollection;

			std::thread* booleanOpThread = nullptr, *staticTessellationThread = nullptr;
			std::atomic<bool> booleanOpThreadFinished = false, staticTessellationFinished = false;
			std::mutex destructionMutex, staticTessellationMutex;
			std::unordered_map <unsigned long long, RigidBodyItem> allItems;
			struct AddParams
			{
				unsigned long long sourcePhysicalItemId;
				std::string origPieceAction = std::string("");
				std::string newGroupId;
				std::string origGroupId;
				RigidBody* oobbPiece;
				MeshMaterial origMeshMaterial;
				std::vector <TriUV> triList;
				AddParams(unsigned long long inPhysicalItemId, std::string inpOrigPieceAction, std::string inpNewGroupId, RigidBody* inpOobbPiece, std::string inpOrigGroupId, std::vector <TriUV> inpTriList) :
					sourcePhysicalItemId(inPhysicalItemId), origPieceAction(inpOrigPieceAction), newGroupId(inpNewGroupId), oobbPiece(inpOobbPiece), origGroupId(inpOrigGroupId), triList(inpTriList) {}
			};
			std::vector<AddParams> deferredAdds;

			unsigned long long AddZonePiece(std::string zoneLocation, std::string meshName, Mesh** loadedMesh, vec3 curPos, float standingHeight);
			unsigned long long Add(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos);
			unsigned long long AddRenderOnly(std::string meshLoc, std::string belong, const mat4& inpTrans, bool inNeverHide = false);
			unsigned long long AddRagDoll(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos);
			void CutOut(unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float cutRadius = 2.0f, float cutDepth = 4.0f);
			void Shatter(unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm);
			void Update(std::function <bool(vec3&, vec3&, float)> isInDrawRegion, vec3 curPos, float standingHeight, ParticleSystemClass& particleSystem, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission, WorldParamsClass& worldParams);
			void TransferCutOutBulletHoles(unsigned long long srcPhysicalItemId, unsigned long long dstPhysicalItemId, std::vector<TriUV>& dstTriList);
			void CacheBulletEmittersHoles(ParticleSystemClass& particleSystem, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission);
			void ProcessFireLines(ParticleSystemClass& particleSystem, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission);
			void Combine(std::vector <PhysicalItemClass>& cameraRailClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, bool forceAddToWorld = false);
			void UpdateSDFs(std::vector <PhysicalItemClass>& cameraRailClasses);
			void AddNewItemsToWorld(bool threaded = true);
			void getVisibleMinMax(vec3& outMin, vec3& outMax);
			void ClearContent();
			void Remove(unsigned long long curId);
		};

		extern PhysicalItemClass physicalItemsCollection;

		class ZoneStreamingClass
		{
		private:
			struct meshFromZoneDesc
			{
				std::string name;
				bool operator==(const meshFromZoneDesc& other) const;
			};
			class MeshFromZoneDescHash
			{
			public:
				std::size_t operator()(const meshFromZoneDesc& k) const;
			};
			bool zoneStreamingActivated = false;
			std::unordered_map<std::string, std::vector<std::string>> zoneReferences;
			std::vector<meshFromZoneDesc> foundMeshesFromZones;
			void getTileNums(vec3 inpPos, int & tileX, int & tileY, int & tileZ);
			bool producingZones = false;
			std::vector<WorldParamsClass> worldParamsLoaders = std::vector<WorldParamsClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<GuidedModelSystemClass> guidedModelLoaders = std::vector<GuidedModelSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<PlasteredItemsClass> plasteredItemsLoaders = std::vector<PlasteredItemsClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<ParticleSystemClass> particleSystemLoaders = std::vector<ParticleSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<CameraSystemClass> cameraSystemLoaders = std::vector<CameraSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<minMaxReductionClass> perLoaderReducer = std::vector<minMaxReductionClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<LadderSystemClass> ladderSystemLoaders = std::vector<LadderSystemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::vector<PhysicalItemClass> physicalItemLoaders = std::vector<PhysicalItemClass>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			std::atomic<bool> producedZones[HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT] = { false, false, false, false, false, false };
			std::vector<std::atomic<unsigned int>> threadProgressCounter = std::vector<std::atomic<unsigned int>>(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
			bool producedZonesOnce = false;
			std::vector <std::thread *> zoneProducerThread;
			std::mutex zone_producer_mutex;
			static void produceZones(ZoneStreamingClass *zoneStreamingPtr, unsigned int threadId);
			vec3 visibleMin, visibleMax, globalMin, globalMax;

		public:
			struct PlayerStatePostStream
			{
				vec3 pos, vel;
				vec2 lookAngles;
				float bodyStretch;
				std::string climbingLadderName;
				unsigned int ladderTransitionStateInt;
				float ladderTransitionFraction;
				vec3 ladderEye, ladderLook;
				float sunAngle;
			};
			struct addedItemsFromFoundZoneMesh
			{
				unsigned long long physicalItemId = 0ull;
				unsigned long long plasteredId = 0ull;
				unsigned long long particlesId = 0ull;
				unsigned long long cameraRailId = 0ull;
				unsigned long long guidedModelsId = 0ull;
				unsigned long long worldParamsId = 0ull;
				unsigned long long laddersId = 0ull;
			};

			bool setPlayerStatePostStream = false;
			PlayerStatePostStream playerStatePostStream;

			vec3 curPos;
			std::string zoneLocation = "";
			std::vector<GroupedRenderSubmission*> submissionList;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;
			std::unordered_map <meshFromZoneDesc, addedItemsFromFoundZoneMesh, MeshFromZoneDescHash> loadedItemsFromZoneMeshes;

			int tilesPerDrawRegionEdge();
			float unitsPerTileEdge();
			bool allZonesProduced();
			bool noZonesProduced();
			bool isZoneStreamingActivated();
			void setAllZonesNotProduced();
			void waitOnZoneProduction();
			void Create(std::string & inpZoneLocation, std::vector<GroupedRenderSubmission*> &&inpSubmissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, const std::function<void(float)>& progressUpdateCallback = [](float) {});
			vec3 & getVisibleMax();
			vec3 & getVisibleMin();
			vec3 & getGlobalMax();
			vec3 & getGlobalMin();
			void ForcePlayerFromPotentialGameLoad();
			void Update(vec3& inpPos, bool forceUpdate = false, const std::function<void(float)>& progressUpdateCallback = [](float) {});
			bool isInDrawRegion(vec3 & inEye, vec3 & objPos, float objRad);
			void ClearContent();
		};

		extern ZoneStreamingClass zoneStreaming;

		class DefaultPipelineSetupClass
		{
		private:
			//unsigned long long animId;
			//mat4 animMat;

			unsigned long long clairAudio = 0ul;
			GroupedTraceSubmission mainRTSubmission;
			GroupedSDFBVHSubmission sdfBvhSubmission;
			MainMenuClass MainMenu;
			SkyDomeClass SkyDome;
			ClearSurfaceCacheClass ClearSurfaceCache;
			ShadowMapClass ShadowMapCascadeNear, ShadowMapCascadeFar;
			ShadowMapScreenClass ShadowMapScreen;
			VisibilityPassClass VisibilityPass;
			GatherResolveClass GatherResolve;
			DecalPassClass DecalPass;
			PathTraceClass PathTrace;
			DoFClass DoF;
			TemporalAccumulateClass TemporalAccumulate;
			SpatialDenoiseClass SpatialDenoise;
			ModulateClass Modulate;
			NearScatteringClass NearScattering;
			ScreenSpaceGatherClass ScreenSpaceGather;
			ScreenSpaceFXClass ScreenSpaceFX;
			MoBlurClass MoBlur;
			TriClass PostProcessTri;
			SplashDisplayClass SplashDisplay;
			bool ApplicationQuitting = false;

		public:
			DefaultPipelineSetupClass(std::string mapPath, std::string mapBelong, bool cmdOptHwRt, unsigned int cmdOptFullRes, WINDOW_MODE cmdOptWindowed);
			PipelineSetupReturn Run();
			bool IsApplicationQuitting();
		};

		void CreateWorld(WorldParamsClass & WorldParams);
		void DestroyWorld();
	}
}