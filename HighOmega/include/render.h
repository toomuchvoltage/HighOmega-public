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

#include "gl.h"
#include "geom.h"
#include "vmath.h"
#include "mesh.h"
#include "events.h"
#include "accelstructs.h"
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <iterator>
#include <list>
#include <thread>
#include <mutex>
#include <noise.h>
#include <functional>
#include <algorithm>
#include "serialization.h"

using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;
using namespace HIGHOMEGA::GL;
using namespace HIGHOMEGA::GL::KHR_RT;
using namespace HIGHOMEGA::MESH;
using namespace HIGHOMEGA::MATH::NOISE;
using namespace HIGHOMEGA::GEOM;

namespace HIGHOMEGA
{
	namespace WORLD
	{
		class ParticleSystemClass;
	}
	namespace RENDER
	{
		namespace PASSES
		{
			class PathTraceClass;
			class TemporalAccumulateClass;
			void GetCubeFaceLookUp(unsigned int faceIdx, vec3& look, vec3& up);
		}
		namespace TEXT
		{
			vec3 MakeGeomFromText(const std::string& text, const vec2& dxdy, const vec3& offset, std::vector<unsigned char>& outIndexVertexData);
		}
		class ScreenSizeClass
		{
		public:
			unsigned int width;
			unsigned int height;

			ScreenSizeClass();
			void Create(unsigned int width, unsigned int height);
		};
		class FrustumClass
		{
			friend class PASSES::PathTraceClass;
			friend class PASSES::TemporalAccumulateClass;
		private:
			struct
			{
				float modelViewProj[16];
				float lookEyeX[4];
				float upEyeY[4];
				float sideEyeZ[4];
				float whrTanHalfFovY[2];
			} uboData;
			bool initBuffer = false;
			void SetView();
			void SetPerspective();
			void SetOrtho();

		public:
			BufferClass Buffer;

			float smoothingFactor = 0.0f;
			float headRotateAmount = 0.0f;
			vec3 entitiesLook, entitiesEye;
			vec3 eye, look, up;
			vec3 look_norm, side, actual_up;
			mat4 modelview_matrix;
			mat4 projection_matrix;
			mat4 modelviewprojection_matrix;
			float screen_fov, screen_whr, screen_near, screen_far;
			float ortho_left, ortho_right, ortho_bottom, ortho_top;
			bool isOrtho;
			bool reverseZ = false;
			bool cameraRelative = false;

			void CreatePerspective(const vec3& eye, const vec3& look, const vec3& up, float fov, float whr, float clipNear, float clipFar);
			void CreateOrtho(const vec3& eye, const vec3& look, const vec3& up, float left, float right, float bottom, float top, float clipNear, float clipFar);
			void Update(const vec3& eyeInBuffer, const vec3& lookInBuffer, const vec3& upInBuffer, float whrInBuffer, float fovYForBuffer);
			void Update();
			void CopyFromEntitiesThread();
			void CopyFromFrustum(const FrustumClass& Other);
			void LerpEyeLookUpWithFrustum(const FrustumClass& Other1, const FrustumClass& Other2, float alpha);
			void ForceEyeAndLook(const vec3& inpEye, const vec3& inpLook);
			vec3 GetUploadedUp();
			vec3 GetUploadedSide();
		};

		extern FrustumClass MainFrustum;
		extern ScreenSizeClass ScreenSize;

		extern std::unordered_map <std::string, HIGHOMEGA::CacheItem<ImageClass>> TextureCache;
		CacheItem<ImageClass> * AddOrFindCachedTexture(const std::string& belong, const std::string& texName, InstanceClass & ptrToInstance, bool isArray = false, int nLayers = 1, bool mipmap = true, bool useSRGB = false);

		class MeshMaterial
		{
		private:
			CacheItem<ImageClass>* TryDifferentLODs(const std::string& belong, const std::string& texName, InstanceClass& ptrToInstance, bool isArray = false, int nLayers = 1, bool mipmap = true, bool useSRGB = false, bool loadLowRes = false);

		public:
			float emissivity;
			float refractiveIndex;
			bool dielectric;
			bool parallaxOcclusionMapping;
			vec2 uvOffset;
			float heightMapDisplaceFactor;
			float subDivAmount;
			bool isTerrain;
			bool smooth;
			bool mipmap;
			bool postProcess;
			bool isAlphaKeyed;
			bool isDecal;
			bool backDropGlass;
			bool holographicInParticleScene;
			bool perVertexVelocity;
			bool isViewerRelative;
			bool isAlphaBlending;

			int renderOrder;

			PipelineFlags pipelineFlags;

			std::string diffName, nrmName, rghName, hgtName, spcName, shaderName;
			CacheItem<ImageClass> *diffRef, *nrmRef, *rghRef, *hgtRef, *spcRef;
			bool operator==(const MeshMaterial& other) const;
			void BumpClaims();
			void ReduceClaims(bool evictAtZero = true);
			MeshMaterial();
			MeshMaterial(HIGHOMEGA::MESH::DataBlock & propBlock, const std::string& belong, InstanceClass & ptrToInstance);
			MeshMaterial(const std::string& inDiffName, const std::string& belong, InstanceClass& ptrToInstance, bool noBumpClaims = false);
			void EnableAlphaBlend();
			void EnableAdditiveBlend();
		};
		class MeshMaterialHash
		{
		public:
			std::size_t operator()(const MeshMaterial& k) const;
		};
		struct Bone
		{
			std::string name;
			mat4 bone;
		};
		class Pose
		{
		public:
			unsigned int keyFrameTime;
			float keyFrameFract;
			std::vector <Bone> pose;

			void Inv();
			void Mul(Pose & rhs);
			void Transform(mat4 & transMat);
		};
		class Animation
		{
		public:
			unsigned int lastKeyFrameTime;
			std::vector <Pose> keyFrames;

			void GetPose(float fract, Pose & retPose);
			void GetPose(unsigned int providedKeyFrame, Pose & retPose);
		};
		class AnimationAndSnapshot
		{
		private:
			unsigned char *rawPoseData = nullptr;

		public:
			Animation anim;
			Pose referencePoseInv;
			Pose currentPose;
			BufferClass *currentAndPrevPoseBuf = nullptr;

			void CopyToPoseBuffer(Pose & inpPose);
			~AnimationAndSnapshot();
		};
		class GraphicsModel;
		class GraphicsModelInstance;
		class GroupedRenderSubmission;
		class GroupedRenderSubmission : public ChangeSignalClass
		{
			friend class GraphicsModel;
			friend class GraphicsModelInstance;
		protected:
			std::vector<GeometryClass*> allChangedGeom;
			std::unordered_map<unsigned long long, GraphicsModelInstance *> allSubmittedItems;
			static void CompileInstanceProperties(InstanceProperties & outProp, const MeshMaterial & inpMaterial, HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6], unsigned int transformOffset, unsigned int prevTransformOffset, GeometryClass& geom);
			void AddNotifySubmission(GraphicsModel& inpModel, std::function<bool(const MeshMaterial& curMat)> inpFilterFunction);
			void AddNotifySubmission(GeometryClass* geomPiece);
			void RemoveNotifySubmission(GraphicsModel& inpModel, std::function<bool(const MeshMaterial& curMat)> inpFilterFunction);
			void RemoveNotifySubmission(GeometryClass* geomPiece);
			virtual void Remove(GeometryClass* geomPiece) = 0;
			virtual void Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece) = 0;

		public:
			class SceneDataStruct
			{
			public:
				BufferClass* instancePropertiesBuffer = nullptr;
				unsigned int instancePropsCount = 0u;
				std::vector <unsigned int> availableInstanceProps;

				BufferClass* transformBuffer = nullptr;
				unsigned int transformCount = 0u;
				std::vector <unsigned int> availableTransformIds;

				struct samplerOffsetClaims
				{
					HIGHOMEGA_TEXTURE_OFFSET offset;
					unsigned int claims = 0u;
				};
				std::unordered_map <ImageClass*, samplerOffsetClaims> uniqueSamplers;
				std::vector <ShaderResource> uniqueSamplersArray;
				std::vector <HIGHOMEGA_TEXTURE_OFFSET> availableSamplers;

				ImageClass BlankTexture;

				unsigned long long descriptorId = 0u;
				unsigned long long sceneArrangementId = 0u;
				bool redoDescriptors = false;
				void UpdateDescriptors();
			};
			static thread_local SceneDataStruct *SceneData;
			static thread_local unsigned int SceneDataClaims;
			GroupedRenderSubmission();
			~GroupedRenderSubmission();
			virtual void Add(GraphicsModelInstance& inpModelInst, std::function<bool(const MeshMaterial & curMat)> inpFilterFunction = [](const MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			}) = 0;
			virtual void Remove(GraphicsModelInstance& inpModelInst) = 0;
		};
		class GraphicsModelInstance
		{
			friend class GraphicsModel;
		private:
			void Create(GraphicsModel& inModel, mat4* transform = nullptr, bool supportsPrevTransform = false);
			void Add(const MeshMaterial& inMaterial, GeometryClass& geom);
			void Remove(GeometryClass* geom);
			~GraphicsModelInstance();

		public:
			struct
			{
				std::vector<InstanceProperties> uploadInstProps;
				std::vector<unsigned int> uploadInstPropsIndex;
				std::vector <unsigned char> uploadTransforms;
				std::vector <unsigned int> uploadTransformIds;
			} uploadInfo;
			mat4 rootTransform;
			unsigned int rootTransformId;
			mat4 prevRootTransform;
			unsigned int prevRootTransformId;
			GroupedRenderSubmission::SceneDataStruct *sceneDataPtr = nullptr;
			struct MatGeomInstance
			{
				const MeshMaterial* mat;
				GeometryClass* geom;
				unsigned int geomPropsId;
			};
			std::vector<MatGeomInstance> matGeomInstances;
			GraphicsModel* modelRef;
			struct SubmissionAttributes
			{
				unsigned long long itemId;
				std::function<bool(const MeshMaterial& curMat)> filterFunction;
			};
			std::unordered_map<GroupedRenderSubmission*, SubmissionAttributes> submissionItems;

			GraphicsModelInstance();
			void SetMinMax(const vec3& min, const vec3& max);
			void Update(const mat4& transform);
		};
		void TransformMesh(Mesh& mesh, mat4& inpTransform);
		void GetMeshMinMax(Mesh& mesh, vec3& outMin, vec3& outMax);
#define HIGHOMEGA_ZONE_VOXELIZE_COARSENESS 1.0f
		class GraphicsModel
		{
			friend class GroupedRenderSubmission;
			friend class GroupedSDFBVHSubmission;
		public:
			static void UpdateSDFs(std::vector<GraphicsModel *>& updateItems, bool forceRefresh = false, std::vector<std::string> cacheNames = {});

		private:
			static unsigned int VoxelizeWorkGroupX();
			static unsigned int TrianglePerThreadCount();
			static unsigned int JFAWorkGroupX();
			static unsigned int JFAWorkGroupY();
			static unsigned int JFAWorkGroupZ();
			static unsigned int TessellateWorkGroup();

			bool isInit = false;
			bool cachedModelMinMax = false;
			vec3 cachedModelMin, cachedModelMax;
			std::vector<GraphicsModelInstance*> instances;
			std::unordered_map<GeometryClass *, std::vector<unsigned char>> idxVertCache;
			struct TessellateVerts
			{
				float tessellationDisplacement;
				float maxTessellationPower;
				float curTessellationPower;
				float targetTessellationPower;
				vec3 origMin, origMax, cent;
				float rad;
				std::vector<unsigned char> idxVertData;
				MeshMaterial mat;
				GeometryClass* curGeom, *addedGeom;
			};
			std::unordered_map<std::string, TessellateVerts> tessellateGeom;

			void RemovePast();
			void GenerateGeom(std::vector<TriUV> & triList, std::vector<unsigned char>& outIndexVertexData);
		public:
			static void ExtractGeom(std::vector<unsigned char>& indexVertexData, std::vector<TriUV>& outTriList);
			ImageClass *sdf = nullptr;
			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> MaterialGeomMap;
			std::unordered_map <std::string, AnimationAndSnapshot> armatures;

			GraphicsModel();
			template<typename... Args> GraphicsModel(Args&&... args)
			{
				Model(std::forward<Args>(args)...);
			}
			void Model(std::string& newGroupId, MeshMaterial& inMaterial, std::vector<TriUV>& triList, bool inpImmutable = true, bool breakable = false);
			void Model(std::string& newGroupId, MeshMaterial& inMaterial, std::vector <unsigned char>& idxVertData, bool inpImmutable = true, bool breakable = false);
			void Model(HIGHOMEGA::MESH::Mesh & inpMesh, std::string belong, InstanceClass &ptrToInstance, std::function<bool(int, DataGroup &)> inpFilterFunction = [](int, DataGroup & inpGroup) -> bool {
				float tmpFloat;
				return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
			}, bool inpImmutable = true, bool loadAnimationData = false, vec3* viewPos = nullptr, float* fullBodyHeight = nullptr, bool justKeyFrames = false);
			~GraphicsModel();
			GraphicsModelInstance* CreateInstance(mat4* transform = nullptr, bool supportsPrevTransform = false);
			void DestroyInstance(GraphicsModelInstance* inModelInst);
			GeometryClass *getGeometryById(const std::string & groupId);
			void getGeometryByIdPrefix(const std::string& prefix, std::vector<GeometryClass*>& allGeoms);
			MeshMaterial getMaterialById(const std::string & groupId);
			void doStaticTessellation(InstanceClass& ptrToInstance, bool inpImmutable = true, vec3 *viewPos = nullptr, float* fullBodyHeight = nullptr, bool firstRun = true);
			void removeOldTessellation();
			void removeGroupById(const std::string& groupId);
			void addGeomWithGroupId(const std::string& newGroupId, MeshMaterial& inMaterial, std::vector <unsigned char>& idxVertData, bool inpImmutable = true);
			void transformVertsSlow(const mat4& trans, const std::string& groupId = std::string(""), mat4 *localTrans = nullptr, 
									vec3 *iSectP1 = nullptr, vec3 *iSectP2 = nullptr, bool *lineHit = nullptr, vec3 *mulVCol = nullptr, bool nonPrefixMode = false);
			void BlankAndResize(unsigned int scaleFactor);
			void SetMinMax(const vec3& minVal, const vec3& maxVal);
			void SetDirty();
			void DownloadGeom(const std::string& groupId, std::vector <unsigned char>& outIdxVerts);
			void ChangeGeom(const std::string& groupId, std::vector <TriUV> & triList);
			void ChangeGeom(const std::string& groupId, std::vector <unsigned char> & inIdxVerts);
			void UpdateGeom(const std::string& groupId, std::vector <TriUV> & triList);
			void getModelMinMax(vec3& outMin, vec3& outMax);
		};

		enum WINDOW_MODE
		{
			WINDOWED = WindowClass::GL_WINDOWED_MODE::GL_WINDOWED,
			WINDOWED_FULLSCREEN = WindowClass::GL_WINDOWED_MODE::GL_WINDOWED_FULLSCREEN,
			FULLSCREEN = WindowClass::GL_WINDOWED_MODE::GL_FULLSCREEN
		};
		void InitGraphicsSubSystem(bool requestHWRT, WINDOW_MODE windowMode, bool headless = false);
		void WindowRecreate(unsigned int width, unsigned int height, WINDOW_MODE windowMode);
		class ComputeSubmission;
		class GroupedTraceSubmission : public GroupedRenderSubmission
		{
		protected:
			void Remove(GeometryClass* geomPiece);
			void Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece);

		public:
			RTScene rtScene;
			unsigned long long cachedArrangementId = 0u;
			unsigned int nInstancesCached = 0u;
			ComputeSubmission* rtCopyTransformSubmission = nullptr;
			ShaderResourceSet* rtCopyTransformResources = nullptr;
			std::function<void(unsigned int, BufferClass*, BufferClass*, bool)> rtCopyTransformCallback;
			~GroupedTraceSubmission();
			void ChangeSignal(GeometryClass *changedGeom);
			unsigned long long SceneID();

			void Add(GraphicsModelInstance & inpModelInst, std::function<bool(const MeshMaterial & curMat)> inpFilterFunction = [](const MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			void Remove(GraphicsModelInstance& inpModelInst);
			void DeleteRTResources();
		};
		class ComputeSubmission
		{
		private:
			InstanceClass *instanceRef = nullptr;

			ComputeletClass computelet;
			bool cpuSync = true;

			unsigned long long swapchainId = 0ull;
			struct SingleDispatch
			{
				DescriptorSets *DS = nullptr;
				std::vector<ShaderResource> cachedSwapchainDependentResources;
				unsigned int groupX, groupY, groupZ;
				std::unordered_set<SemaphoreClass*> waitOn;
				std::unordered_set<SemaphoreClass*> waitOnOld;
				std::unordered_set<SemaphoreClass*> signal;
			};
			std::unordered_map <std::string, SingleDispatch> dispatches;
			std::unordered_set<SemaphoreClass*> waitOn;
			std::unordered_set<SemaphoreClass*> waitOnOld;
			std::unordered_set<SemaphoreClass*> signal;

			unsigned int resourcesRequested;
			bool initComputelet = false;
			bool updateResources = false;
			bool useComputeQueue = false;

			Compute_PSO_DSL *computePsoDsl;

		public:

			ComputeSubmission();
			void MakeDispatch(InstanceClass & ptrToInstance, const std::string & inpName, ShaderResourceSet & inpShader, int inpGroupX, int inpGroupY, int inpGroupZ);
			void UpdateDispatchSize(InstanceClass & ptrToInstance, const std::string & inpName, int inpGroupX, int inpGroupY, int inpGroupZ);
			void RemoveDispatch(InstanceClass & ptrToInstance, const std::string & inpName);
			ComputeSubmission& MakeAsync();
			ComputeSubmission& UseComputeQueue();
			void Submit();
			~ComputeSubmission();
		};
		class GroupedRasterSubmission;
		class GroupedSDFBVHSubmission : public GroupedRenderSubmission
		{
			friend class GroupedRasterSubmission;
			friend class HIGHOMEGA::WORLD::ParticleSystemClass;
		protected:
			void Remove(GeometryClass* geomPiece);
			void Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece);

		private:
			struct singleLeafTransform
			{
				float transMat[16];
				float bMin[4];
				float bMax[4];
			};
			struct buildParamsStruct
			{
				float mapMinLeafCount[4];
				float mapMax[4];
			} buildParams;
			vec3 mapMin, mapMax;
			BufferClass buildParamsBuf;

			bool shadersCreated = false;
			ShaderResourceSet mortonShader;
			ComputeSubmission mortonSubmission;

			unsigned int leafCountAligned = 0u;
			std::vector <BoxLeaf> sdfLeaves, sdfLeavesTmp;
			std::vector<ImageClass *> allSDFBVHImages;
			std::vector<singleLeafTransform> leafTransformInfo;
			BufferClass leavesBuf;

			unsigned int nodeCountAligned = 0u;
			BufferClass cwNodesBuf;

			BufferClass invMatBuf;
			std::vector<ShaderResource> SDFs;

			SDFBVHClass SDFBVH;
			CWBVHClass SDFCWBVH;
			bool sceneChanged = false;
			unsigned long long sceneID = 0ull;

			unsigned int CompMortonWorkGroupSize();

		public:
			static std::function<bool(const MeshMaterial& curMat)> defaultFilterFunction;
			void ChangeSignal(GeometryClass *changedGeom);
			unsigned long long SceneID();

			void Add(GraphicsModelInstance& inpModel, std::function<bool(const MeshMaterial& curMat)> inpFilterFunction = [](const MeshMaterial& curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			void Remove(GraphicsModelInstance& inpModelInst);
			void DeleteSDFBVHResources();
			~GroupedSDFBVHSubmission();
		};
		namespace PASSES
		{
			class TriClass;
		}
		class GroupedRasterSubmission : public GroupedRenderSubmission
		{
			friend class MeshMaterial;
		protected:
			void Remove(GeometryClass* geomPiece);
			void Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece);

		public:
			enum CULL_MODE
			{
				TWOPASS_NO_FRUSTUM,
				TWOPASS
			};

		private:
			bool recordedCmdBuf;
			bool cullingResourcesChanged;
			bool redoSubmissionData;
			FramebufferClass* frameBuffer, *mainPassFrameBuffer = nullptr;
			std::unordered_map <std::string, ShaderResourceSet *> shaders;
			unsigned long long swapchainId = 0ull;
			vec3 clearColor;
			float clearColorW;
			float depthClear;
			unsigned int stencilClear;

			struct CullParamsStruct
			{
				float pl[16];
				unsigned int nInstances;
			} CullParams;
			BufferClass CullParamsBuf;
			struct InstanceCullStruct
			{
				unsigned int instId;
				unsigned int indirectBatchId;
				unsigned int indexCount;
				unsigned int firstIndex;
				unsigned int vertexOffset;
			};
			std::vector<InstanceCullStruct> InstanceCullData;
			BufferClass InstanceCullBuf, TrackedVisDataBuf;
			PipelineFlags defaultPipelineFlags;
			RasterletClass::DynamicPipelineFlags dynPipelineFlags;

			bool doesCulling;
			CULL_MODE cullMode;
			ComputeSubmission* cullingPrepassCompute = nullptr, *cullingMainpassCompute = nullptr;
			FrustumClass* cachedCullingFrustum;
			ShaderResourceSet* cullingPrepassResourceSet = nullptr, * cullingMainpassResourceSet = nullptr;
			bool Setup_HiZ = false;
			ComputeSubmission *MipChainPass1_HiZ = nullptr, *MipChainPass2_HiZ = nullptr;
			ShaderResourceSet *MipChainPass1ShaderResources_HiZ = nullptr, *MipChainPass2ShaderResources_HiZ = nullptr;
			struct mipParamsStruct
			{
				unsigned int blockSize[2];
			} mipParams_HiZ;
			BufferClass mipParamsBuffer_HiZ;

			unsigned long long sceneDataRecordID = 0ull;
			bool requestsBVH;
			bool requestsSDFBVH;
			GroupedSDFBVHSubmission *sourceSDFBVHSubmission;
			unsigned long long sourceSDFBVHId;

			void DestroySubmissionData();
			std::string GenerateRasterPSOKey(MeshMaterial & inpMat, PipelineFlags& inpPFlags, bool renderMode);
			unsigned long long GenerateRasterDSKey(std::vector<ShaderResource>& resources);

		public:
			RasterletClass Rasterlet;
			std::vector<ImageClass> MinZChain, MaxZChain;

			std::vector <MeshMaterial> transformedMaterials;
			std::unordered_map <MeshMaterial, std::vector<GraphicsModelInstance::MatGeomInstance *>, MeshMaterialHash> entireMatGeomMap;
			std::unordered_map <unsigned long long, DescriptorSets *> DSCache;
			std::vector <PSO_DSL_DS_GeomInstPairing> PSO_DSL_DS_GeomInstPairings;
			std::function<MeshMaterial(const MeshMaterial& inMat)> matTransform;

			GroupedRasterSubmission();
			~GroupedRasterSubmission();
			void Create(InstanceClass & ptrToInstance);
			void ChangeSignal(GeometryClass *changedGeom);
			void Add(GraphicsModelInstance& inpModelInst, std::function<bool(const MeshMaterial & curMat)> inpFilterFunction = [](const MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			static bool postProcessOnlyFilter(const MeshMaterial & curMat);
			static bool blendOnlyFilter(const MeshMaterial & curMat);
			static bool decalOnlyFilter(const MeshMaterial& curMat);
			static bool everythingButDecalsFilter(const MeshMaterial & curMat);
			static bool noBlendOrPostProcessOrDecalFilter(const MeshMaterial& curMat);
			void Remove(GraphicsModelInstance& inpModelInst);
			void SetFrameBuffer(FramebufferClass & inpFrameBuffer);
			void SetShader(std::string inpName, ShaderResourceSet & inpShader);
			void SetClearColor(vec3 inClearColor, float inClearColorW);
			void SetDepthClear(float inDepthClear);
			void SetStencilClear(unsigned int inStencilClear);
			void SetDefaultPipelineFlags(PipelineFlags & inDefaultPipelineFlags);
			void requestSDFBVH(GroupedSDFBVHSubmission & sdfBvhSubmission);
			void AllocateMipChainImages();
			void doCulling(FrustumClass &inpFrustum, CULL_MODE inCullMode, int useDepthLayer = -1);
			unsigned int WorkGroupTwoPassCullX();
			GroupedRasterSubmission& MakeAsync();
			void Render();
		};

		class WorldParamsClass;
		class WorldParamsClass
		{
		public:
			class Parameters
			{
				friend class WorldParamsClass;
			private:
				BufferClass *renderTimeBuffer = nullptr;

			public:
				float sunAngle = 0.0f;
				struct
				{
					float cur = 0.0f;
					float prev = 0.0f;
				} renderTime;
				float lastFrameTime = 0.0f;
				bool showAurora = false;
				float forceSunAngle = 0.0f;
				TimerObject frameTimer;
				vec3 startPlayerPos = vec3(0.0f, 20.0f, 0.0f);
				bool firstPersonControls = true;
				float lightShaftAmount = 0.0f;
				float lightShaftExtinction = 0.95f;
				float sunDirectLightStrength = 1.0f;
			};
			std::unordered_map <unsigned long long, Parameters> allItems;

			unsigned long long Populate(Mesh & inpMesh);
			void ClearContent();
			void Combine(std::vector<WorldParamsClass> & inpWorldParamSystems);
			void Remove(unsigned long long inpId);
			void StartFrameTimer();
			void EndFrameTimer();
			float GetFrameTime();
			void AddSunAngle(float sunAngle);
			void ForceSunAngle(float sunAngle);
			void UnforceSunAngle();
			void AddRenderTime(float addRenderTime);
			void ShowAurora();
			void ShowClouds();
			bool isShowingAurora();
			float GetSunAngle();
			vec3 SunDir();
			vec3 SunOrMoonDir();
			bool FirstPersonControls();
			vec3 StartPlayerPos();
			void SetLightShaftAmount(float lightShaftAmount);
			void SetLightShaftExtinction(float lightShaftExtinction);
			void SetSunDirectLightStrength(float sunDirectLightStrength);
			float GetLightShaftAmount();
			float GetLightShaftExtinction();
			float GetSunDirectLightStrength();
			BufferClass & GetRenderTimeBuffer();
		};

		namespace PASSES
		{
			class RTPass
			{
			protected:
				ShaderResourceSet rtShaderResourceSet;
				std::vector<ShaderResource> tracingResources;
				unsigned long long lastSceneId = 0ull;

			public:
				RTTracelet tracelet;
			};
			class DepthStencilHolder
			{
			public:
				ImageClass depthStencilAttach;
			};
			class BlurInputHolder
			{
			public:
				ImageClass blurInput;
			};
			class SinglePassClass : public DepthStencilHolder
			{
			protected:
				ShaderResourceSet shader;

			public:
				GroupedRasterSubmission submission;
			};
			class GatherPassClassCommon
			{
			public:
				ImageClass worldPosAttach, normalAttach;
			};
			#define HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT 10
			class SingleComputeClass
			{
			protected:
				ShaderResourceSet shader;

			public:
				ComputeSubmission submission;
			};
			class OffScreenPassClass : public SinglePassClass
			{
			protected:
				FrustumClass frustum;
				FramebufferClass frameBuffer;
			};
			class TriClass
			{
			private:
				struct
				{
					float dims[4];
				} dimsData;

			public:
				GraphicsModel triModel;
				GraphicsModelInstance *triModelInstance;
				FrustumClass triFrustum;

				void Create(const vec3& eyeInBuffer, const vec3& lookInBuffer, const vec3& upInBuffer, float whrInBuffer, float fovYForBuffer);
				void UpdateViewSpace(const vec3& eyeInBuffer, const vec3& lookInBuffer, const vec3& upInBuffer, float whrInBuffer, float fovYForBuffer);
			};
			class ShadowMapClass
			{
				friend class PathTraceClass;
				friend class PreIlluminateClass;
				friend class ShadowMapScreenClass;
				friend class TemporalAccumulateClass;
				friend class NearScatteringClass;
			public:
				enum SHADOWMAP_MODE
				{
					NONE,
					ORTHO,
					PERSPECTIVE,
					PERSPECTIVE_CUBIC
				};

			protected:
				struct PerPassData
				{
					ImageClass depthStencilAttach, shadowColorAttach;
					ShaderResourceSet shader, shaderAlpha, shaderStainedGlass, shaderStainedGlassTess;
					FrustumClass frustum;
					FramebufferClass frameBuffer;
					GroupedRasterSubmission submission;
				};
				std::vector<PerPassData> perPassData;
				SHADOWMAP_MODE mode = SHADOWMAP_MODE::NONE;
				vec2 samplingBias;
				WorldParamsClass * ptrWorldParams;
				float shadowMapSideBias;
				void ComputeOptimalOrthoFrustum(vec3 viewMin, vec3 viewMax, vec3 toSkyObject, FrustumClass & frustum);

			public:
				struct
				{
					vec3 viewMax, viewMin;
					unsigned int frameCount = 0u;
				} orthoData;

				ShadowMapClass& SetMode(SHADOWMAP_MODE inpMode);
				GroupedRasterSubmission& getSubmission(unsigned int passIdx = 0);
				ImageClass& getDSAttach(unsigned int passIdx = 0);
				ImageClass& getColorAttach(unsigned int passIdx = 0);
				FrustumClass& getFrustum(unsigned int passIdx = 0);
				void Create(WorldParamsClass & WorldParams, vec2 samplingBiasMinMax, unsigned int sideResolution = 512u);
				void RenderOrtho(const vec3& viewMin, const vec3& viewMax, float shadowMapSideBias = 0.0f, FrustumClass *nearCascadeFrustum = nullptr);
				void RenderPerspective(FrustumClass& origFrustum, const vec3& eyeOverride, const vec3& lookOverride, float fovOverride);
				void RenderCubicPerspective(const vec3& eye);
			};
			class ShadowMapScreenClass : public OffScreenPassClass, public RTPass
			{
			protected:
				bool builtHWRTSubmission = false, builtSoftRTSubmission = false;
				std::string ShadowTagCopy;
				TriClass* postProcessTriRef;
				GatherPassClassCommon *gatherPassCommonRef;
				ShadowMapClass *shadowMapNearRef, *shadowMapFarRef;
				bool useRTIfAvailableCached = false;
				bool singleCascade = false;

			public:
				ShaderResourceSet shaderH, shaderV;
				ImageClass shadowMapScreen;
				GroupedRasterSubmission submissionH, submissionV;
				FramebufferClass frameBufferH, frameBufferV;
				ImageClass blurHAttach;
				BufferClass blurPropsBuf;

				void CreateDirectional(TriClass & PostProcessTri, ShadowMapClass & ShadowMapCascadeNear, ShadowMapClass & ShadowMapCascadeFar, GatherPassClassCommon & GatherPass, const std::string& ShadowTag, bool useRTIfAvailable = false);
				void CreateSingleCascade(TriClass& PostProcessTri, ShadowMapClass& ShadowMap, GatherPassClassCommon& GatherPass, ImageClass& blurHTextureToReuse, const std::string& ShadowTag, bool omniDir = false);
				FrustumClass *getNearFrustum(unsigned int passIdx = 0u);
				void Render(GroupedTraceSubmission & rtSubmission);
			};

			class VisibilityPassClass : public OffScreenPassClass
			{
			private:
				ShaderResourceSet shaderAlphaKey;
				FrustumClass *frustumRef;

			public:
				FrustumClass visFrustum, prevVisFrustum;
				bool prevVisFrustumEverSet = false;
				ImageClass visibilityTriInfo;

				void Create(unsigned int width, unsigned int height, float alphaThreshold, FrustumClass & frustum, bool swapchainDependent = false);
				void Render();
			};

			class TabletScreenHolderClass
			{
			public:
				ImageClass tabletScreenImage, tabletScreenDS;
			};

			class MoBlurClass;
			class GatherResolveClass : public SingleComputeClass, public GatherPassClassCommon
			{
			private:
				MoBlurClass* moBlurRef = nullptr;
				bool madeDispatch = false;
				bool simple = false;
				unsigned long long sceneId = 0ull;
				unsigned int visBufResX = 0u, visBufResY = 0u;
				VisibilityPassClass* visPassRef = nullptr;

			public:
				ImageClass materialAttach;

				unsigned int GatherResolveWorkGroupX();
				unsigned int GatherResolveWorkGroupY();
				void Create(VisibilityPassClass& VisibilityPass, MoBlurClass* MoBlur = nullptr, bool simple = false);
				void Render();
			};

			class DecalPassClass : public OffScreenPassClass
			{
			private:
				ShaderResourceSet mixShader;
				FrustumClass mixFrustum;
				FramebufferClass mixFrameBuffer;
				GroupedRasterSubmission mixSubmission;

			public:
				ImageClass decalAlbedo, decalSpecular, decalRoughnessSpecularity, decalNormal, decalTangent, decalBiTangent;
				FrustumClass* frustumRef;
				FrustumClass decalFrustum;

				void Create(GatherResolveClass& GatherResolve, FrustumClass& frustum, TriClass& PostProcessTri);
				void Render();
			};

			class SkyDomeClass : public OffScreenPassClass
			{
			private:
				WorldParamsClass * ptrWorldParams;
				GraphicsModel skyDome;
				GraphicsModel mountains;
				GraphicsModelInstance *mountainsInst, *skyDomeInst;
				struct
				{
					float lightDir[4];
					float sunDirAndExp[4];
					float invW4InnerRad[4];

					float innerCloudRad;
					float outerRad;
					float scaleDepth;
					float scaleOverScaleDepth;

					float scatteringCoeff;
					float transmissionCoeff;
					float ambientCoeff;
					float stepSizeEvalLightDir;

					float stepSizeToSky;
					float numStepsToSky;
					float stepSizeToSun;
					float numStepsToSun;

					float addSkyColorAndNightAmount[4];
					float skyObjectLightAndAngle[4];
					float horizonColor[4];
					float approxGroundColor[4];

				} rayleighMieInfo;

				struct
				{
					float invDims[2];
					float doAurora;
				} SkyDomeParams, FullResSkyDomeParams;

				BufferClass skyDomeParamsBuf, fullResSkyDomeParamsBuf;
				ImageClass noiseImg, noiseImg2, moonImg, nebulaImg;

				struct
				{
					unsigned int mode; // 0 - 5 skybox, 6 backdrop
				} skyBoxCompositionParams;

				BufferClass skyBoxCompositionParamsBuf[7];

			public:
				BufferClass rayleighMieBuf;
				ShadowMapClass distantGeomShadowMapNear, distantGeomShadowMapFar;
				ShadowMapScreenClass distantGeomScreenShadow[7];
				ImageClass skyCubeMap, skyBackDrop, fullCubeMap, fullBackDrop;
				DepthStencilHolder backdropDS;
				VisibilityPassClass distantVis[7];
				GatherResolveClass distantGather[7];
				GroupedRasterSubmission skySubmissions[7];
				ShaderResourceSet skyShaders[7];
				FrustumClass skyFrustums[7];
				FramebufferClass skyFrameBuffers[7];
				GroupedRasterSubmission submissions[7];
				ShaderResourceSet shaders[7];
				FramebufferClass frameBuffers[7];

				unsigned int GetBackdropWidth();
				unsigned int GetBackdropHeight();
				unsigned int GetCubeResolution();
				float InnerRad();
				float InnerCloudRad();
				float OuterRad();
				float HeightOfAvgDensity();
				float SunExp();

				float ScatteringCoeff();
				float TransmissionCoeff();
				float AmbientCoeff();
				float NumStepsToSky();
				float NumStepsToSun();

				vec3 SkyBlue();
				vec3 GroundDirtColor();
				vec3 AuroraGreen();
				vec3 SunOrange();
				vec3 SunWhite();
				vec3 MoonLight();
				vec3 NebulaBlue();

				vec3 AddSkyColor();
				float NightAmount();
				vec3 SkyObjectLight();
				vec3 HorizonColor();
				vec3 ApproxGroundColor();

				void UpdateSkyInfo(bool uploadToo = false);
				void Create(TriClass & Tri, WorldParamsClass & WorldParams, MoBlurClass & MoBlur);
				void RenderDistantGeom();
				void Render(WorldParamsClass & WorldParams, GroupedTraceSubmission & rtSubmission);
			};
			class BlueNoiseHolderClass
			{
			protected:
				static ImageClass* blueNoise;
				static unsigned int blueNoiseClaims;

			public:
				BlueNoiseHolderClass();
				~BlueNoiseHolderClass();
			};
			class PathTraceClass : public OffScreenPassClass, public RTPass, public BlueNoiseHolderClass
			{
			private:
				bool builtHWRTSubmission = false, builtSoftRTSubmission = false;
				TriClass* postProcessTriRef;
				GroupedSDFBVHSubmission* sdfBVHSubmissionRef;
				GatherResolveClass *GatherPassRef;
				SkyDomeClass *SkyDomeRef;
				GroupedTraceSubmission *TraceRef;
				vec3 viewerCached;
				ShadowMapClass *shadowMapNearRef, *shadowMapFarRef;
				ShaderResourceSet rtShaderResourceSetGloss;
				std::vector<ShaderResource> tracingResourcesGloss;
				RTTracelet traceletGloss;
				BufferClass influenceBuf;

			public:
				struct InfluenceStruct
				{
					float factor;
				} influence;
				struct PathTraceParamsStruct
				{
					float radiosityMapCenterMotionFactor[4];
					float timeTurnBlurDirectionRawLight[4];
				} PathTraceParams;
				BufferClass PathTraceParamsBuf;
				ImageClass glossTraceOutput;
#define HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES 6
#define HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE 32
				ImageClass radiosityMaps[HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES][6];
				std::vector <ImageClass *> radiosityMapsVec;

				GroupedRasterSubmission glossSubmission;
				ShaderResourceSet glossTraceShader;
				FramebufferClass glossTraceFrameBuffer;

				void Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, SkyDomeClass & SkyDome, ShadowMapClass & ShadowMapNear, ShadowMapClass & ShadowMapFar, GroupedTraceSubmission & rtSubmission, GroupedSDFBVHSubmission *sdfBVHSubmission, const vec3 & initialViewer);
				void Render(const vec3& currentViewer);
			};

			class ClearSurfaceCacheClass : public SingleComputeClass
			{
			public:

				unsigned int WorkGroupSize();
				void Create(PathTraceClass & PathTrace);
				void Submit();
			};
			class TemporalAccumulateClass : public OffScreenPassClass
			{
			private:
				PathTraceClass *pathTraceRef;
				struct
				{
					float matrix[HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT][16];
				} MVPs;
				BufferClass MVPsBuf;

			public:
				ImageClass glossTemporalAccumulateResultAttach, glossLightTrailAttach, worldPosCacheAttach;

				void Create(TriClass &PostProcessTri, GatherResolveClass & GatherResolve, PathTraceClass & PathTrace);
				void Render();
			};
			class SpatialDenoiseClass
			{
			private:
				ShaderResourceSet shaderH,shaderV;
				PathTraceClass *pathTraceRef;

			public:
				GroupedRasterSubmission submissionH, submissionV;
				FrustumClass frustum;
				FramebufferClass frameBufferH, frameBufferV;
				ImageClass blurHAttach, blurVAttach;

				void Create(TriClass &PostProcessTri, PathTraceClass & PathTrace, TemporalAccumulateClass & TemporalAccumulate, GatherResolveClass &GatherResolve);
				void Render();
			};
			class SimpleGaussian
			{
			private:
				FramebufferClass frameBufferH, frameBufferV;

			public:
				ImageClass blurHAttach, blurVAttach, blurDSAttach;
				GroupedRasterSubmission blurHSubmission, blurVSubmission;
				ShaderResourceSet blurHShader, blurVShader;

				void Create(TriClass & PostProcessTri, BlurInputHolder & inputHolder, unsigned int outputWidth, unsigned int outputHeight, unsigned int blurSize, bool displayVOnScreen = false);
				void Render();
			};
			class ModulateClass : public OffScreenPassClass
			{
			public:
				ShaderResourceSet particleShader, particleShaderAlphaBlend;
				ImageClass modulatedOutput;

				void Create(TriClass & PostProcessTri, MoBlurClass& MoBlur, VisibilityPassClass & VisibilityPass, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, SpatialDenoiseClass & SpatialDenoise, SkyDomeClass & SkyDome, ShadowMapScreenClass & shadowMapScreen);
				void Render();
			};
			class ScreenSpaceGatherClass : public OffScreenPassClass
			{
			private:
				ShaderResourceSet shaderScreenSpace, shaderTessScreenSpace;

			public:
				ImageClass ssGatherPosAlbedo, ssNormInstIDVelocityRoughness;

				void Create(TriClass & PostProcessTri, MoBlurClass & MoBlur, VisibilityPassClass& VisibilityPass, GatherResolveClass &GatherPass, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams);
				void Render();
			};
			class NearScatteringClass : public OffScreenPassClass, public BlurInputHolder, public BlueNoiseHolderClass
			{
			private:
				WorldParamsClass* worldParamsRef = nullptr;
				struct
				{
					float amount;
					float extinction;
				} nearScatteringParams;
				BufferClass nearScatteringParamsBuf;

			public:
				SimpleGaussian blurPass;

				void Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, ShadowMapClass & ShadowMapNear, ShadowMapClass & ShadowMapFar, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams, ScreenSpaceGatherClass & ScreenSpaceGather, unsigned int outputSize);
				void Render();
			};
			class ScreenSpaceFXClass : public OffScreenPassClass
			{
			public:
				ImageClass ssfxOut;

				void Create(TriClass & PostProcessTri, SkyDomeClass & SkyDome, VisibilityPassClass& VisibilityPass, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, ModulateClass & Modulate, ScreenSpaceGatherClass & ScreenSpaceGather, MoBlurClass & MoBlur, NearScatteringClass & NearScattering);
				void Render();
			};
			class MoBlurClass : public OffScreenPassClass
			{
			private:
				ShaderResourceSet dilationShader;
				GroupedRasterSubmission dilationSubmission;
				FramebufferClass dilationFrameBuffer;

			public:
				ImageClass moBlurOut, velocityAttach;

				void CreateVelocityBuffer();
				void PrepareForFrame();
				void Create(TriClass& PostProcessTri, ScreenSpaceFXClass& ScreenSpaceFX, ScreenSpaceGatherClass& ScreenSpaceGather, VisibilityPassClass& VisibilityPass, GatherResolveClass& GatherPass);
				void Render();
			};
			class DoFClass : public SinglePassClass
			{
			private:
				GraphicsModel midScreenText, topScreenText;
				GraphicsModelInstance* midScreenTextInst = nullptr, *topScreenTextInst = nullptr;
				enum CUR_MID_SCREEN_TEXT_TYPE
				{
					NONE,
					CLIMB
				};
				CUR_MID_SCREEN_TEXT_TYPE curMidScreenTextType = NONE, targetMidScreenTextType = NONE;
				struct
				{
					float invDims[2];
					float blurDirection;
					float invCoCDist;
					float screenMidPos[4];
					float toScreenMidPosAlpha[4];
					float midScreenAlpha;
				} dofParams, dofParamsDownloaded;
				float midScreenAlphaTarget = 0.0f;
				BufferClass dofParamsBuf;
				FramebufferClass frameBufferH;
				bool onLadderMsg = false;

				void SetMidScreenMessage();

			public:
				ImageClass dofHAttach;
				GroupedRasterSubmission dofHSubmission, dofVSubmission;
				ShaderResourceSet dofHShader, dofVShader, textShader, midScrTextShader;

				void SetIsOnLadder(bool isOnLadder);
				void Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, MoBlurClass & MoBlur);
				void Render(float inpAlpha);
			};
			class SplashDisplayClass : public SinglePassClass
			{
			private:
				FrustumClass splashFrustum;
				struct
				{
					float alphaAmount;
				} splashParams;
				BufferClass splashParamsBuf;
				GraphicsModel credits;
				GraphicsModelInstance *creditsInst;

				TimerObject splashTimer;
				float timePassed = 0.0f;
				unsigned int splashPhase = 0;

			public:

				void Create(TriClass & PostProcessTri);
				void Render();
			};
			class MainMenuClass : public SinglePassClass
			{
			private:
				struct
				{
					float alphaAmount;
				} menuParams;
				BufferClass menuParamsBuf;
				FrustumClass menuFrustum;

				GraphicsModel cursor, mainmenu, hwrtbased, sdfbvhbased, screenRes[5], windowed, fullscreen, winfullscr, loading;
				std::array <vec2, 5> allowedResolutions = { vec2(640.0f, 480.0f), vec2(800.0f, 600.0f), vec2(1024.0f, 768.0f), vec2(1280.0f, 720.0f), vec2(1920.0f, 1080.0f) };
				GraphicsModelInstance *cursorInst, *mainmenuInst, *hwrtbasedInst, *sdfbvhbasedInst, *screenResInst[5], *windowedInst, *winfullscrInst, *fullscreenInst, *loadingInst;
				mat4 techniqueItemMat, screenResItemMat, windowedItemMat;
				vec2 cursorPos;

				unsigned int fullResSelection;
				WINDOW_MODE windowedSelection;
				bool hwrtSelection;

				bool done = false;
				bool showLoading = false;
				bool rebooting = false;
				bool prevLeftMouseDown = false;

				mat4 getCursorMat();
				mat4 getItemMat(float itemY);
				bool overButton(vec2 buttonPos, vec2 buttomDim);

			public:

				void Create(TriClass & PostProcessTri, bool cmdOptHwRt, unsigned int cmdOptFullRes, WINDOW_MODE cmdOptWindowed);
				void SetupScreen();
				void Render(bool postInit, const std::function<void(void)>& switchToSDFBVH, const std::function<void(void)>& switchToHWRT);
				bool IsDone();
				void SetNotDone();
				bool IsRebooting();
			};
		}
	}
}