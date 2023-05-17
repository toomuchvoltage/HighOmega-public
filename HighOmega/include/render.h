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

#ifndef HIGHOMEGA_RENDER_H
#define HIGHOMEGA_RENDER_H

#include "gl.h"
#include "geom.h"
#include "vmath.h"
#include "mesh.h"
#include "events.h"
#include "accelstructs.h"
#include <map>
#include <unordered_map>
#include <iterator>
#include <list>
#include <thread>
#include <mutex>
#include <noise.h>
#include <functional>
#include <algorithm>

using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;
using namespace HIGHOMEGA::GL;
using namespace HIGHOMEGA::GL::KHR_RT;
using namespace HIGHOMEGA::MESH;
using namespace HIGHOMEGA::MATH::NOISE;
using namespace HIGHOMEGA::GEOM;

namespace HIGHOMEGA
{
	namespace RENDER
	{
		namespace PASSES
		{
			class PathTraceClass;
			class SaurayTraceClass;
			class TemporalAccumulateClass;
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
			vec3 entitiesLook, entitiesEye;
			vec3 eye, look, up;
			mat4 modelview_matrix;
			mat4 projection_matrix;
			mat4 modelviewprojection_matrix;
			float screen_fov, screen_whr, screen_near, screen_far;
			float ortho_left, ortho_right, ortho_bottom, ortho_top;
			bool isOrtho;
			bool reverseZ = false;

			void CreatePerspective(vec3 eye, vec3 look, vec3 up, float fov, float whr, float clipNear, float clipFar);
			void CreateOrtho(vec3 eye, vec3 look, vec3 up, float left, float right, float bottom, float top, float clipNear, float clipFar);
			void Update(vec3 eyeInBuffer, vec3 lookInBuffer, vec3 upInBuffer, float whrInBuffer, float fovYForBuffer);
			void Update();
			void CopyFromEntitiesThread();
			void CopyFromFrustum(FrustumClass & Other);
			void ForceEyeAndLook(vec3 inpEye, vec3 inpLook);
		};

		extern FrustumClass MainFrustum;
		extern ScreenSizeClass ScreenSize;

		extern std::unordered_map <std::string, HIGHOMEGA::CacheItem<ImageClass>> TextureCache;
		CacheItem<ImageClass> * AddOrFindCachedTexture(std::string & belong, std::string & texName, InstanceClass & ptrToInstance, bool isArray = false, int nLayers = 1, bool mipmap = true, bool useSRGB = false);

		class MeshMaterial
		{
		public:
			float emissivity;
			float refractiveIndex;
			bool dielectric;
			vec2 uvOffset;
			float heightMapDisplaceFactor;
			float subDivAmount;
			bool isTerrain;
			bool smooth;
			bool mipmap;
			bool postProcess;
			bool isAlphaKeyed;
			bool backDropGlass;
			bool scattering;

			int renderOrder;
			unsigned int playerId = 0xFFFFFFFFu;
			unsigned char rayMask = 0xFFu;

			PipelineFlags pipelineFlags;

			std::string diffName, nrmName, rghName, hgtName, spcName, shaderName;
			CacheItem<ImageClass> *diffRef, *nrmRef, *rghRef, *hgtRef, *spcRef;
			bool operator==(const MeshMaterial& other) const;
			void BumpClaims();
			void ReduceClaims();
			MeshMaterial();
			MeshMaterial(HIGHOMEGA::MESH::DataBlock & propBlock, std::string belong, InstanceClass & ptrToInstance);
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
			BufferClass *currentPoseBuf = nullptr;

			void CopyToPoseBuffer(Pose & inpPose);
			~AnimationAndSnapshot();
		};
		class GraphicsModel;
		class GroupedRenderSubmission;
		class SubmittedRenderItem
		{
		public:
			GroupedRenderSubmission *producer = nullptr;
			GraphicsModel *item;
			unsigned long long itemId;
			std::function<bool(MeshMaterial & curMat)> filterFunction;
		};
		class GroupedRenderSubmission : public ChangeSignalClass
		{
			friend class GraphicsModel;
		protected:
			std::unordered_map<unsigned long long, SubmittedRenderItem> allSubmittedItems;
			static void CompileInstanceProperties(InstanceProperties & outProp, const MeshMaterial & inpMaterial);
		public:
			GroupedRenderSubmission();
			virtual SubmittedRenderItem Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction = [](MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			}) = 0;
			virtual void Remove(SubmittedRenderItem & inpSubmittedRenderItem) = 0;
		};
		void TransformMesh(Mesh& mesh, mat4& inpTransform);
#define HIGHOMEGA_ZONE_VOXELIZE_COARSENESS 4.0f
		class GraphicsModel
		{
		public:
			struct NEEHint
			{
				vec3 pos, norm;
			};
			struct NEEHintData
			{
				std::vector<NEEHint> hints;
				float area;
			};
			std::unordered_map <std::string, NEEHintData> allNEEHintData;
			mat4 modelTransMat, modelTransMatInv;
			static void UpdateSDFs(std::vector<SubmittedRenderItem>& updateItems, bool forceRefresh = false, std::vector<std::string> cacheNames = {});

		private:
			static unsigned int VoxelizeWorkGroupX();
			static unsigned int TrianglePerThreadCount();
			static unsigned int JFAWorkGroupX();
			static unsigned int JFAWorkGroupY();
			static unsigned int JFAWorkGroupZ();

			bool isInit = false;
			std::unordered_map<GeometryClass *, std::vector<RasterVertex>> vertexCache;

			void RemovePast();
			void GenerateGeom(std::vector<TriUV> & triList, std::vector <RasterVertex> & renderVertexVector);
		public:
			ImageClass *sdf = nullptr;
			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> MaterialGeomMap;
			std::unordered_map <std::string, AnimationAndSnapshot> armatures;

			GraphicsModel();
			template<typename... Args> GraphicsModel(Args&&... args)
			{
				Model(std::forward<Args>(args)...);
			}
			void Model(std::string & newGroupId, MeshMaterial & origMaterial, std::vector<TriUV>& triList, bool gpuResideOnly = true, bool inpImmutable = true);
			void Model(HIGHOMEGA::MESH::Mesh & inpMesh, std::string belong, InstanceClass &ptrToInstance, std::function<bool(int, DataGroup &)> inpFilterFunction = [](int, DataGroup & inpGroup) -> bool {
				float tmpFloat;
				return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
			}, mat4 *inpTransform = nullptr, bool gpuResideOnly = true, bool inpImmutable = true, bool loadAnimationData = false);
			~GraphicsModel();
			GeometryClass *getGeometryById(std::string & groupId);
			MeshMaterial getMaterialById(std::string & groupId);
			void removeGroupById(std::string & groupId);
			void transformVertsSlow(mat4 & trans, std::string groupId = std::string(""), mat4 *localTrans = nullptr,
				vec3 *iSectP1 = nullptr, vec3 *iSectP2 = nullptr, bool *lineHit = nullptr, vec3 *mulVCol = nullptr);
			std::vector <RasterVertex> * getVertsSlow(std::string groupId);
			void BlankAndResize(unsigned int scaleFactor);
			void SetMinMax(vec3 minVal, vec3 maxVal);
			void TransformCorners(mat4 inpMat);
			void SetDirty();
			void ChangeGeom(std::string & groupId, std::vector <TriUV> & triList);
			void UpdateGeom(std::string & groupId, std::vector <TriUV> & triList);
			void getModelMinMax(vec3 & outMin, vec3 & outMax);
			void getUntransformedMinMax(vec3 & outMin, vec3 & outMax);
		};

		void InitGraphicsSubSystem(bool requestHWRT, bool windowed, bool headless = false);
		struct MaterialGeomPairing
		{
			MeshMaterial mat;
			std::vector<GeometryClass *> *geom;
		};
		class GroupedTraceSubmission : public GroupedRenderSubmission
		{
		private:
			std::vector<unsigned long long> changedItems;
		public:
			RTScene rtScene;
			void ChangeSignal(unsigned long long changedItem);
			unsigned long long SceneID();
			SubmittedRenderItem Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction = [](MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			void Remove(SubmittedRenderItem & inpSubmittedRenderItem);
		};
		class ComputeSubmission
		{
		private:
			InstanceClass *instanceRef = nullptr;

			ComputeletClass computelet;

			struct SingleDispatch
			{
				DescriptorSets *DS = nullptr;
				unsigned int groupX, groupY, groupZ;
				ShaderResourceSet *curState;
			};
			std::unordered_map <std::string, SingleDispatch> dispatches;

			unsigned int resourcesRequested;
			bool computeSemInit = false;
			bool updateResources = false;

			Compute_PSO_DSL *computePsoDsl;

		public:

			ComputeSubmission();
			void MakeDispatch(InstanceClass & ptrToInstance, const std::string & inpName, ShaderResourceSet & inpShader, int inpGroupX, int inpGroupY, int inpGroupZ);
			void UpdateDispatchSize(InstanceClass & ptrToInstance, const std::string & inpName, int inpGroupX, int inpGroupY, int inpGroupZ);
			void RemoveDispatch(InstanceClass & ptrToInstance, const std::string & inpName);
			void makeAsync();
			void makeSerial();
			void Submit();
			~ComputeSubmission();
		};
		class GroupedRasterSubmission;
		class GroupedBVHSubmission : public GroupedRenderSubmission
		{
			friend class GroupedRasterSubmission;
		private:
			BufferClass nodesBuf, trisCompressedBuf;
			BufferClass instBuf;
			std::vector<InstanceProperties> instProps;
			std::vector<ShaderResource> sourceGeom, sourceMats;
			bool sceneChanged = false;
			unsigned long long sceneID = 0ull;

			struct buildParamsStruct
			{
				float mapMinTriCount[4];
				float mapMax[4];
			} buildParams;
			vec3 mapMin, mapMax;
			BufferClass buildParamsBuf;

			struct compMortonParamsStruct
			{
				unsigned int OffsetLen[2];
			};
			std::vector <compMortonParamsStruct> mortonParams;
			BufferClass compMortonParamsBuf;

			unsigned int totalInstanceCount = 0u;
			unsigned int totalSceneTriCount = 0u;

			unsigned int instanceCountAligned = 0u;
			unsigned int triCountAligned = 0u;
			unsigned int nodeCountAligned = 0u;

			bool shadersCreated = false;
			ShaderResourceSet mortonShader;
			ComputeSubmission mortonSubmission;
			ShaderResourceSet compressionShader;
			ComputeSubmission compressionSubmission;

			BufferClass triBuf;
			std::vector <BVHTriangle> tris, trisTmp;
			std::vector <BVHTriangleCompressed> trisComp;
			std::vector <BVHNode> nodes;

			BVHGenCPUClass bvhGenCPU;

			unsigned int CompMortonWorkGroupSize();
			unsigned int CompMortonTrisPerThreadSize();
			unsigned int CompressWorkGroupSize();

		public:
			void ChangeSignal(unsigned long long changedItem);
			unsigned long long SceneID();
			SubmittedRenderItem Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction = [](MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			void Remove(SubmittedRenderItem & inpSubmittedRenderItem);
		};
		class GroupedSDFBVHSubmission : public GroupedRenderSubmission
		{
			friend class GroupedRasterSubmission;
		private:
			struct singleLeafTransform
			{
				float transMat[16];
				float invTransMat[16];
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
			std::vector <SDFLeaf> sdfLeaves, sdfLeavesTmp;
			std::vector<SubmittedRenderItem> allSubmittedItemsVector;
			std::vector<ImageClass *> allSDFBVHImages;
			std::vector<singleLeafTransform> leafTransformInfo;
			BufferClass leavesBuf;

			unsigned int nodeCountAligned = 0u;
			std::vector <BVHNode> nodes;
			BufferClass nodesBuf;

			BufferClass invMatBuf;
			std::vector<ShaderResource> SDFs;
			std::vector<SubmittedRenderItem> changedItems;

			SDFBVHClass SDFBVH;
			bool sceneChanged = false;
			unsigned long long sceneID = 0ull;

			unsigned int CompMortonWorkGroupSize();

		public:
			void ChangeSignal(unsigned long long changedItem);
			unsigned long long SceneID();
			SubmittedRenderItem Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction = [](MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			void Remove(SubmittedRenderItem & inpSubmittedRenderItem);
		};
		namespace PASSES
		{
			class TriClass;
		}
		class GroupedRasterSubmission : public GroupedRenderSubmission
		{
			friend class MeshMaterial;
		public:
			enum CULL_MODE
			{
				HIZ_ONLY,
				FRUSTUM_HIZ,
				FRUSTUM_HIZ_REVERSE
			};

		private:
			bool recordedCmdBuf;
			bool resourcesChanged;
			bool cullingResourcesChanged;
			unsigned int resourcesRequested;
			bool redoSubmissionData;
			FramebufferClass *frameBuffer;
			std::unordered_map <std::string, ShaderResourceSet *> shaders;
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
			struct InstanceMinMaxStruct
			{
				float min[4];
				float max[4];
			};
			std::vector<InstanceMinMaxStruct> InstanceMinMaxData;
			BufferClass InstanceMinMaxBuf;
			PipelineFlags defaultPipelineFlags;
			RasterletClass::DynamicPipelineFlags dynPipelineFlags;
			BufferClass conditionalBuffer;

			bool doesCulling;
			CULL_MODE cullMode;
			ComputeSubmission* cullingCompute = nullptr;
			FrustumClass* cachedCullingFrustum;
			ShaderResourceSet* cullingResourceSet = nullptr;
			bool Setup_HiZ = false;
			ComputeSubmission *MipChainPass1_HiZ = nullptr, *MipChainPass2_HiZ = nullptr;
			ShaderResourceSet *MipChainPass1ShaderResources_HiZ = nullptr, *MipChainPass2ShaderResources_HiZ = nullptr;
			struct mipParamsStruct
			{
				unsigned int blockSize[2];
			} mipParams_HiZ;
			BufferClass mipParamsBuffer_HiZ;

			bool recordMatGeomBindings;
			GroupedRasterSubmission *takenMatGeomBindings = nullptr;
			unsigned long long matGeomSourceRecordID;
			unsigned long long cmdRecordId;
			bool requestsBVH;
			GroupedBVHSubmission *sourceBVH;
			unsigned long long sourceBVHId;
			bool requestsSDFBVH;
			GroupedSDFBVHSubmission *sourceSDFBVHSubmission;
			unsigned long long sourceSDFBVHId;

			void DestroySubmissionData();
			std::string GenerateRasterPSOKey(MeshMaterial & inpMat, PipelineFlags& inpPFlags, bool renderMode);

		public:
			RasterletClass Rasterlet;
			std::vector<ImageClass> Output_HiZ;
			BufferClass SSBO;
			std::vector <InstanceProperties> SSBOdata;

			std::unordered_map <MeshMaterial, std::vector<GeometryClass *>, MeshMaterialHash> entireMatGeomMap;
			static std::unordered_map <GroupedRasterSubmission *, std::unordered_map <std::string, std::unordered_map <std::string, DescriptorSets *>>> globalDSCache;
			static std::mutex globalDSCache_mutex;
			std::vector <MaterialGeomPairing> MaterialGeomPairings;
			std::vector <PSO_DSL_DS_GeomPairing> PSO_DSL_DS_GeomPairings;
			std::vector <ShaderResource> MaterialBindings;
			std::vector <ShaderResource> GeomBindings;
			std::function<MeshMaterial(const MeshMaterial& inMat)> matTransform;

			GroupedRasterSubmission();
			~GroupedRasterSubmission();
			void Create(InstanceClass & ptrToInstance);
			void ChangeSignal(unsigned long long changedItem);
			SubmittedRenderItem Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction = [](MeshMaterial & curMat) -> bool {
				if (curMat.postProcess) return false;
				return true;
			});
			static bool postProcessOnlyFilter(MeshMaterial & curMat);
			static bool everythingFilter(MeshMaterial & curMat);
			void Remove(SubmittedRenderItem & inpSubmittedRenderItem);
			void SetFrameBuffer(FramebufferClass & inpFrameBuffer);
			void SetShader(std::string inpName, ShaderResourceSet & inpShader);
			void SetClearColor(vec3 inClearColor, float inClearColorW);
			void SetDepthClear(float inDepthClear);
			void SetStencilClear(unsigned int inStencilClear);
			void SetDefaultPipelineFlags(PipelineFlags & inDefaultPipelineFlags);
			void makeAsync();
			void makeSerial();
			void requestBVH(GroupedBVHSubmission & bvhHolder);
			void requestSDFBVH(GroupedSDFBVHSubmission & sdfBvhSubmission);
			void doCulling(FrustumClass &inpFrustum, CULL_MODE inCullMode);
			void keepMatGeomBindings();
			void consumeMatGeomBindings(GroupedRasterSubmission *inpConsumeMatGeomBindingsFrom);
			unsigned int instanceCount();
			unsigned int WorkGroupFrustumCullX();
			unsigned int WorkGroupHiZCullX();
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
					float time[4];
				} renderInfo;
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
				FrustumClass triFrustum;

				void Create(vec3 eyeInBuffer, vec3 lookInBuffer, vec3 upInBuffer, float whrInBuffer, float fovYForBuffer);
				void UpdateViewSpace(vec3 eyeInBuffer, vec3 lookInBuffer, vec3 upInBuffer, float whrInBuffer, float fovYForBuffer);
			};
			class BoxClass
			{
			public:
				GraphicsModel boxModel;

				void Create();
			};
			class ShadowMapClass : public OffScreenPassClass
			{
				friend class PathTraceClass;
				friend class PreIlluminateClass;
				friend class ShadowMapScreenClass;
				friend class TemporalAccumulateClass;
				friend class NearScatteringClass;
			protected:
				vec2 samplingBias;
				ShaderResourceSet shaderAlpha, shaderStainedGlass, shaderStainedGlassTess;
				WorldParamsClass * ptrWorldParams;
				float shadowMapSideBias;
				void ComputeOptimalOrthoFrustum(vec3 viewMin, vec3 viewMax, vec3 toSkyObject, FrustumClass & frustum);

			public:
				ImageClass shadowColorAttach;
				vec3 viewMax, viewMin;

				void Create(WorldParamsClass & WorldParams);
				void Render(vec3 viewMin, vec3 viewMax, float shadowMapSideBias = 0.0f, vec2 samplingBiasMinMax = vec2 (0.1f, 0.25f), FrustumClass *nearCascadeFrustum = nullptr);
			};
			class ShadowMapScreenClass : public OffScreenPassClass, public RTPass
			{
			protected:
				struct
				{
					float blurDirectionNearMinBias;
					float nearMaxBias;
					float farMinBias;
					float farMaxBias;
				} shadowProps;
				BufferClass shadowPropsBuf;
				GatherPassClassCommon *gatherPassCommonRef;
				ShadowMapClass *shadowMapNearRef, *shadowMapFarRef;
				bool useRTIfAvailableCached = false;

			public:
				ShaderResourceSet shaderH, shaderV;
				ImageClass shadowMapScreen;
				GroupedRasterSubmission submissionH, submissionV;
				FramebufferClass frameBufferH, frameBufferV;
				ImageClass blurHAttach;
				BufferClass blurPropsBuf;

				void makeAsync();
				void makeSerial();
				void Create(TriClass & PostProcessTri, ShadowMapClass & ShadowMapCascadeNear, ShadowMapClass & ShadowMapCascadeFar, GatherPassClassCommon & GatherPass, bool useRTIfAvailable = false);
				void Render(GroupedTraceSubmission & rtSubmission);
			};

			class VisibilityPassClass : public OffScreenPassClass
			{
			private:
				ShaderResourceSet shaderAlphaKey;
				FrustumClass *frustumRef;
				FrustumClass visFrustum;

			public:
				ImageClass visibilityTriInfo;

				void Create(unsigned int width, unsigned int height, FrustumClass & frustum);
				void Render();
			};

			class GatherResolveClass : public OffScreenPassClass, public GatherPassClassCommon
			{
			public:
				ImageClass materialAttach;

				void Create(VisibilityPassClass & VisibilityPass, TriClass & PostProcessTri, FrustumClass & OriginalFrustum, bool simple = false);
				void Render();
			};

			class SkyDomeClass : public OffScreenPassClass
			{
			private:
				WorldParamsClass * ptrWorldParams;
				GraphicsModel skyDome;
				GraphicsModel mountains, mountainsTess;
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

				struct
				{
					unsigned int triCountTessFactorInputStrideOutputStride[4];
				} PrimitiveTessellateParams;
				BufferClass skyDomeParamsBuf, fullResSkyDomeParamsBuf, PrimitiveTessellateParamsBuf;
				ImageClass noiseImg, noiseImg2, moonImg, nebulaImg;

			public:
				BufferClass rayleighMieBuf;
				ShaderResourceSet tessellateMountainsShader;
				ComputeSubmission tessellateMountains;
				ShadowMapClass distantGeomShadowMapNear, distantGeomShadowMapFar;
				ShadowMapScreenClass distantGeomScreenShadow[7];
				ImageClass fullCubeMap, backDrop;
				DepthStencilHolder backdropDS;
				VisibilityPassClass distantVis[7];
				GatherResolveClass distantGather[7];
				GroupedRasterSubmission submissions[7];
				ShaderResourceSet shaders[7];
				FrustumClass frustums[7];
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
				unsigned int WorkGroupSize();
				unsigned int TessellatePower();
				unsigned int TessellateFactor();
				void Create(TriClass & Tri, WorldParamsClass & WorldParams);
				void RenderDistantGeom();
				void Render(WorldParamsClass & WorldParams, GroupedTraceSubmission & rtSubmission);
			};

			class PathTraceClass : public OffScreenPassClass, public RTPass
			{
			private:
				GatherResolveClass *GatherPassRef;
				SkyDomeClass *SkyDomeRef;
				GroupedTraceSubmission *TraceRef;
				vec3 viewerCached;
				ShadowMapClass *shadowMapNearRef, *shadowMapFarRef;
				ShaderResourceSet rtShaderResourceSetGloss;
				std::vector<ShaderResource> tracingResourcesGloss;
				RTTracelet traceletGloss;

			public:
				struct
				{
					float nearMinBias;
					float nearMaxBias;
					float farMinBias;
					float farMaxBias;
				} shadowBiases;
				BufferClass shadowBiasesBuf;
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
				void Render(vec3 & currentViewer);
			};

			class ClearSurfaceCacheClass : public SingleComputeClass
			{
			public:

				unsigned int WorkGroupSize();
				void Create(PathTraceClass & PathTrace);
				void Submit();
			};
			class BlueNoiseHolderClass
			{
			protected:
				static ImageClass *blueNoise;
				static unsigned int blueNoiseClaims;

			public:
				BlueNoiseHolderClass();
				~BlueNoiseHolderClass();
			};
			class SaurayTraceClass : public OffScreenPassClass, public RTPass, public BlueNoiseHolderClass
			{
			private:
				ShaderResourceSet rtShaderResourceSet2;
				std::vector<ShaderResource> tracingResources2;
				RTTracelet tracelet2;
				unsigned long long lastSceneId2 = 0ull;

				struct playerVisData
				{
					unsigned int visCell[4];
				};
				struct timeStruct
				{
					unsigned int frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[4];
				} timeInfo;
				BufferClass frustaBuf, limitsBuf, visibilityMatrixBuf, timeBuf;
				unsigned int playerResSide;
				unsigned int maxPlayers;
				unsigned int maxPlayerSqrt;
				unsigned int resSide;
				bool newPlayerInfo = false;
				unsigned int temporalAmount = 0;
				GroupedTraceSubmission *mainSubmissionRef = nullptr;

			public:
				struct playerFrustum
				{
					float eyeGeomRad[4];
					float eye2Whr[4];
					float lookUpLook2Up2[4];
					float geomCentYScale[4];
					unsigned int maskEnabledReserved;
				};
				std::vector<playerFrustum> playerFrusta;
				struct playerLimit
				{
					float aabbLim[20];
					float corners[24];
				};
				std::vector<playerLimit> playerLimits;

				std::vector<playerVisData> playerVisMatrix;
				ImageClass testOutput;

				void SetPlayer(unsigned int playerId, unsigned char otherTeamId, const vec3 & eye, const vec3 & look, const vec3 & up, const vec3 & eye2, const vec3 & look2, const vec3 & up2, float inYFov, float inWhr, vec3 & geomCent, float geomRad);
				void RemovePlayer(unsigned int playerId);
				void Create(GroupedTraceSubmission & mainSubmission, unsigned int inpMaxPlayers, unsigned int inpResSide, unsigned int inpHistoryAmount, bool debugMode);
				void PrePass();
				void Render();
				unsigned int CanSee(unsigned int viewer, unsigned int subject);
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
				ShaderResourceSet shaderH, shaderV;
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
				struct
				{
					float invDims[2];
					float blurDirection;
					float size;
				} blurParams;
				BufferClass blurParamsBuf;
				FramebufferClass frameBufferH, frameBufferV;

			public:
				ImageClass blurHAttach, blurVAttach, blurDSAttach;
				GroupedRasterSubmission blurHSubmission, blurVSubmission;
				ShaderResourceSet blurHShader, blurVShader;

				void Create(TriClass & PostProcessTri, BlurInputHolder & inputHolder, unsigned int outputWidth, unsigned int outputHeight, unsigned int blurSize, bool displayVOnScreen = false);
				void Render(unsigned int blurSize);
			};
			class ModulateClass : public OffScreenPassClass
			{
			public:
				ShaderResourceSet particleShader;
				ImageClass modulatedOutput;

				void Create(TriClass & PostProcessTri, VisibilityPassClass & VisibilityPass, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, SpatialDenoiseClass & SpatialDenoise, SkyDomeClass & SkyDome, ShadowMapScreenClass & shadowMapScreen);
				void Render();
			};
			class ScreenSpaceGatherClass : public OffScreenPassClass
			{
			private:
				ShaderResourceSet shaderScreenSpace, shaderTessScreenSpace;

			public:
				ImageClass ssGatherPosAlbedo, ssGatherNormRoughnessBackdrop;

				void Create(TriClass & PostProcessTri, GatherResolveClass &GatherPass, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams);
				void Render();
			};
			class NearScatteringClass : public OffScreenPassClass, public BlurInputHolder, public BlueNoiseHolderClass
			{
			private:
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

				void Create(TriClass & PostProcessTri, SkyDomeClass & SkyDome, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, ModulateClass & Modulate, ScreenSpaceGatherClass & ScreenSpaceGather, NearScatteringClass & NearScattering);
				void Render();
			};
			class DoFClass : public SinglePassClass
			{
			private:
				unsigned char onScreenText[300], midScreenText[90];
				ImageClass onScreenTextImage, midScreenTextImage, fontMap;
				enum CUR_MID_SCREEN_TEXT_TYPE
				{
					NONE,
					CLIMB
				};
				CUR_MID_SCREEN_TEXT_TYPE curMidScreenTextType = NONE;
				struct
				{
					float invDims[2];
					float blurDirection;
					float invCoCDist;
					float screenMidPos[4];
					float toScreenMidPosAlpha[4];
					float midScreenAlpha;
				} dofParams;
				float midScreenAlphaTarget = 0.0f;
				BufferClass dofParamsBuf;
				FramebufferClass frameBufferH;

			public:
				ImageClass dofHAttach;
				GroupedRasterSubmission dofHSubmission, dofVSubmission;
				ShaderResourceSet dofHShader, dofVShader;

				void Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, ScreenSpaceFXClass & ScreenSpaceFX);
				void Render(float inpAlpha);
				void SetMidScreenMessage(bool mainPlayerOnLadder);
			};
			class SaurayDisplayTestClass : public SinglePassClass
			{
			public:
				void Create(TriClass & PostProcessTri, SaurayTraceClass & SaurayTrace);
				void Render();
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

				TimerObject splashTimer;
				SubmittedRenderItem renderItem;
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

				GraphicsModel cursor, mainmenu, hwrtbased, sdfbvhbased, screenRes[5], windowed, fullscreen, loading;
				std::array <vec2, 5> allowedResolutions = { vec2(640.0f, 480.0f), vec2(800.0f, 600.0f), vec2(1024.0f, 768.0f), vec2(1280.0f, 720.0f), vec2(1920.0f, 1080.0f) };
				SubmittedRenderItem mainmenuItem, cursorItem, techniqueItem, screenResItem, windowedItem;
				mat4 techniqueItemMat, screenResItemMat, windowedItemMat;
				vec2 cursorPos;

				unsigned int fullResSelection, windowedSelection;
				bool hwrtSelection;

				bool done = false;
				bool rebooting = false;
				bool prevLeftMouseDown = false;

				mat4 getCursorMat();
				mat4 getItemMat(float itemY);
				bool overButton(vec2 buttonPos, vec2 buttomDim);

			public:

				void Create(TriClass & PostProcessTri, bool cmdOptHwRt, unsigned int cmdOptFullRes, bool cmdOptWindowed);
				void Render();
				bool IsDone();
				bool IsRebooting();
			};
		}
	}
}

#endif