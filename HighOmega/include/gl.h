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

#include "common.h"
#include <vulkan/vulkan.h>
#include <ktx.h>
#include <ktxvulkan.h>
#include <sdl/SDL.h>
#include <SDL/SDL_syswm.h> 
#undef main
#include <string>
#include <vector>
#include <list>
#include <array>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <vmath.h>
#include <iostream>
#include <iterator>
#include <util.h>
#include <algorithm>
#include <numeric>

namespace HIGHOMEGA
{
	namespace GL
	{
		class InstanceClass;
		class DescriptorSets;
		namespace MEMORY_MANAGER
		{
			namespace LIBKTX2
			{
				VkResult BindBufferMemoryCWrapper(VkBuffer buffer, uint64_t allocId);
				VkResult BindImageMemoryCWrapper(VkImage image, uint64_t allocId);
				void LockQueue(VkQueue queue);
				void UnlockQueue(VkQueue queue);
			};
			struct SubAlloc
			{
				unsigned long long offset = 0, len = 0;
				VkDeviceMemory mem;
				bool operator==(const SubAlloc& rhs) const
				{
					return (rhs.offset == offset && rhs.len == len);
				}
			};
			struct MemChunk
			{
				VkDeviceMemory mem;
				unsigned long long used = 0, maxSize = 0;
				std::vector<SubAlloc> allocs;
				int memoryTypeKey;
				unsigned long long id;
				bool operator==(const MemChunk& rhs) const
				{
					return rhs.mem == mem;
				}
			};
			enum MEMORY_MAP_TYPE
			{
				IMAGE = 0,
				BUFFER,
				DEV_ADDRESS
			};
			std::vector<std::tuple<int, unsigned long long, SubAlloc>> AllocMem(VkDevice & inpDev, VkMemoryAllocateInfo & allocInfo, VkMemoryRequirements & memReq, MEMORY_MAP_TYPE memoryMapType, bool useSparseResources, bool oneGiantVertBuffer = false);
			unsigned long long ReportMemoryHoles();
			void LogMemUsageStats();
			void FreeMem(std::vector<std::tuple<int, unsigned long long, SubAlloc>>& pages, VkDevice& inpDev);
		}
		namespace KHR_RT
		{
			class RTInstance
			{
			private:
				static bool rtEnabled;

			public:
				static PFN_vkCreateAccelerationStructureKHR fpCreateAccelerationStructureKHR;
				static PFN_vkDestroyAccelerationStructureKHR fpDestroyAccelerationStructureKHR;
				static PFN_vkCmdBuildAccelerationStructuresKHR fpCmdBuildAccelerationStructuresKHR;
				static PFN_vkCmdTraceRaysKHR fpCmdTraceRaysKHR;
				static PFN_vkGetBufferDeviceAddressKHR fpGetBufferDeviceAddressKHR;
				static PFN_vkCreateRayTracingPipelinesKHR fpCreateRayTracingPipelinesKHR;
				static PFN_vkGetAccelerationStructureBuildSizesKHR fpGetAccelerationStructureBuildSizesKHR;
				static PFN_vkGetAccelerationStructureDeviceAddressKHR fpGetAccelerationStructureDeviceAddressKHR;
				static PFN_vkGetRayTracingShaderGroupHandlesKHR fpGetRayTracingShaderGroupHandlesKHR;

				static VkPhysicalDeviceRayTracingPipelinePropertiesKHR raytracingPipelineProperties;

				static void Enable(InstanceClass & inpInstance);
				static void Disable();
				static bool Enabled();
			};
			class RTPipelineStateClass;
			class RTAccelStruct;
			class RTScene;
			class RTTracelet;
		}
		VkBool32 DEBUG_MESSAGE(VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objType, uint64_t srcObject, size_t location,
							   int32_t msgCode, const char* pLayerPrefix, const char* pMsg, void* pUserData);
		class InstanceClass;
		class RasterletClass;
		class WindowClass : public CStyleWrapper
		{
			friend class InstanceClass;
			friend class FramebufferClass;
		public:
			enum GL_WINDOWED_MODE
			{
				GL_WINDOWED = 0,
				GL_WINDOWED_FULLSCREEN,
				GL_FULLSCREEN
			};

		private:
			std::string appName;
			int startx, starty, w, h;
			GL_WINDOWED_MODE windowedMode;
			bool haveRenderer, haveWindow;
			SDL_Window *win;
			HWND window;
			HINSTANCE hInstance;

			WindowClass(const WindowClass &obj);
			void operator=(const WindowClass& b);

			void RemovePast();

		public:

			WindowClass();
			void Make(std::string appName, int startx, int starty, int w, int h, GL_WINDOWED_MODE windowedMode);
			void Recreate(int w, int h, GL_WINDOWED_MODE windowedMode);
			int GetWidth();
			int GetHeight();
			template<typename... Args> WindowClass(Args&&... args)
			{
				Make(std::forward<Args>(args)...);
			}
			~WindowClass();
		};
		enum TEXTURE_DIM
		{
			_1D = VK_IMAGE_VIEW_TYPE_1D,
			_2D = VK_IMAGE_VIEW_TYPE_2D,
			_3D = VK_IMAGE_VIEW_TYPE_3D,
			_CUBE = VK_IMAGE_VIEW_TYPE_CUBE,
			_1D_ARRAY = VK_IMAGE_VIEW_TYPE_1D_ARRAY,
			_2D_ARRAY = VK_IMAGE_VIEW_TYPE_2D_ARRAY,
			_CUBE_ARRAY = VK_IMAGE_VIEW_TYPE_CUBE_ARRAY
		};
		VkImageType GetVkImageTypeFromDim(TEXTURE_DIM inpDim);
		enum FORMAT
		{
			R8SRGB = VK_FORMAT_R8_SRGB,
			R8G8SRGB = VK_FORMAT_R8G8_SRGB,
			R8G8B8SRGB = VK_FORMAT_R8G8B8_SRGB,
			R8G8B8A8SRGB = VK_FORMAT_R8G8B8A8_SRGB,
			R8UN = VK_FORMAT_R8_UNORM,
			R8G8UN = VK_FORMAT_R8G8_UNORM,
			R8G8B8UN = VK_FORMAT_R8G8B8_UNORM,
			R8G8B8A8UN = VK_FORMAT_R8G8B8A8_UNORM,
			R8SN = VK_FORMAT_R8_SNORM,
			R8G8SN = VK_FORMAT_R8G8_SNORM,
			R8G8B8SN = VK_FORMAT_R8G8B8_SNORM,
			R8G8B8A8SN = VK_FORMAT_R8G8B8A8_SNORM,
			R8UI = VK_FORMAT_R8_UINT,
			R8G8UI = VK_FORMAT_R8G8_UINT,
			R8G8B8UI = VK_FORMAT_R8G8B8_UINT,
			R8G8B8A8UI = VK_FORMAT_R8G8B8A8_UINT,
			R32UI = VK_FORMAT_R32_UINT,
			R32G32UI = VK_FORMAT_R32G32_UINT,
			R32G32B32UI = VK_FORMAT_R32G32B32_UINT,
			R32G32B32A32UI = VK_FORMAT_R32G32B32A32_UINT,
			R32SI = VK_FORMAT_R32_SINT,
			R32F = VK_FORMAT_R32_SFLOAT,
			R32G32F = VK_FORMAT_R32G32_SFLOAT,
			R32G32B32F = VK_FORMAT_R32G32B32_SFLOAT,
			R32G32B32A32F = VK_FORMAT_R32G32B32A32_SFLOAT,
			R16F = VK_FORMAT_R16_SFLOAT,
			R16G16F = VK_FORMAT_R16G16_SFLOAT,
			R16G16B16F = VK_FORMAT_R16G16B16_SFLOAT,
			R16G16B16A16F = VK_FORMAT_R16G16B16A16_SFLOAT
		};
		int FormatSize(FORMAT inpFormat);
		class SemaphoreClass;
		class ComputeletClass;
		class RasterletClass;
		class BufferClass;
		class ImageClass;
		class FenceClass : public CStyleWrapper
		{
			friend VkResult MEMORY_MANAGER::LIBKTX2::BindBufferMemoryCWrapper(VkBuffer buffer, uint64_t allocId);
			friend VkResult MEMORY_MANAGER::LIBKTX2::BindImageMemoryCWrapper(VkImage image, uint64_t allocId);
			friend class SemaphoreClass;
			friend class RasterletClass;
			friend class BufferClass;
			friend class ImageClass;
		protected:
			VkFence fence;

		private:
			InstanceClass *ptrToInstance;
			bool haveFence;

		public:
			void RemovePast();
			FenceClass();
			void Fence(InstanceClass * inpPtrToInstance);
			template<typename... Args> FenceClass(Args&&... args)
			{
				Fence(std::forward<Args>(args)...);
			}
			void Wait();
			void Reset();
			~FenceClass();
		};
		class Timestamp
		{
		private:
			VkQueryPool timingPool = VK_NULL_HANDLE;
			InstanceClass* ptrToInstance = nullptr;
			unsigned long long execTime = 0u;

		public:
			~Timestamp();
			Timestamp();
			void Create(InstanceClass& instancePtr);
			void Start(VkCommandBuffer & cmdBuf);
			void End(VkCommandBuffer& cmdBuf);
			unsigned long long Report();
		};
		enum QUEUE_TYPE
		{
			GRAPHICS_QUEUE = 0,
			COMPUTE_QUEUE,
			TRANSFER_QUEUE,
			MAX_QUEUES
		};
		enum RESOURCE_SHARING_MODE
		{
			QUEUE_EXCLUSIVE = VK_SHARING_MODE_EXCLUSIVE,
			QUEUE_CONCURRENT = VK_SHARING_MODE_CONCURRENT
		};
		class CommandBuffer;
		class SemaphoreClass : public CStyleWrapper
		{
			friend class RasterletClass;
			friend class ComputeletClass;
			friend class CommandBuffer;
			friend class KHR_RT::RTScene;
		private:
			InstanceClass* ptrToInstance;
			VkSemaphore semaphore;
			bool haveSemaphore, binary, readyForWait;
			unsigned long long waitValue;
			unsigned long long signalValue;

		public:
			SemaphoreClass();
			void RemovePast();
			void Semaphore(const SemaphoreClass& other);
			void Semaphore(InstanceClass* inpPtrToInstance, bool inBinary = false);
			VkSemaphore& GetSemaphoreForPresent();
			template<typename... Args> SemaphoreClass(Args&&... args)
			{
				Semaphore(std::forward<Args>(args)...);
			}
			~SemaphoreClass();
		};
		class CommandBuffer
		{
			friend class ImageClass;
		protected:
			static ThreadLocalCache <VkCommandPool> cmdPools[QUEUE_TYPE::MAX_QUEUES];
			static InstanceClass *ptrToInstance;
			QUEUE_TYPE queueType = QUEUE_TYPE::GRAPHICS_QUEUE;
			ThreadLocalCache <VkCommandPool>::value *cmdPoolRef = nullptr;
			std::vector<VkCommandBuffer> cmdBuffers;
			std::vector<SemaphoreClass> ownSemaphores;
			bool haveSetupCmdBuffer = false;
			std::vector<std::unordered_set<SemaphoreClass*>> waitOn;
			std::vector<std::unordered_set<SemaphoreClass*>> waitOnOld;
			std::vector<std::unordered_set<SemaphoreClass*>> signal;
			std::vector<bool> cpuSync;

			void DestroyCommandBuffer();
			ThreadLocalCache <VkCommandPool>::value* CreateCommandPool(InstanceClass& providedInstance, QUEUE_TYPE inQueueType = QUEUE_TYPE::GRAPHICS_QUEUE);

		public:

			~CommandBuffer();
			void BeginCommandBuffer(InstanceClass & instanceRef, unsigned int inpNumCmdBufs = 1, unsigned int which = 0, QUEUE_TYPE inQueueType = QUEUE_TYPE::GRAPHICS_QUEUE);
			void EndCommandBuffer(unsigned int which = 0);
			void WaitOnSemaphores(const std::unordered_set<SemaphoreClass*>& inWaitOn, unsigned int which = 0);
			void WaitOnOldSemaphores(const std::unordered_set<SemaphoreClass*>& inWaitOnOld, unsigned int which = 0);
			void SignalSemaphores(const std::unordered_set<SemaphoreClass*>& inSignal, unsigned int which = 0);
			void NoCPUSync(unsigned int which = 0);
			void DoCPUSync(unsigned int which = 0);
			void SubmitCommandBuffer(unsigned int which = 0);
			void WaitForCompletion(unsigned int which = 0);
			unsigned int CommandBufferCount();
		};
		class KHR_RT::RTScene;
		enum MEMORY_OPTIONS
		{
			MEMORY_DEVICE_LOCAL = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
			MEMORY_HOST_VISIBLE = VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
			MEMORY_HOST_COHERENT = VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
			MEMORY_HOST_CACHED = VK_MEMORY_PROPERTY_HOST_CACHED_BIT,
			MEMORY_LAZILY_ALLOCATED = VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT,
			MEMORY_PROTECTED = VK_MEMORY_PROPERTY_PROTECTED_BIT
		};
		inline MEMORY_OPTIONS operator|(MEMORY_OPTIONS a, MEMORY_OPTIONS b)
		{
			return static_cast<MEMORY_OPTIONS>(static_cast<int>(a) | static_cast<int>(b));
		}
		inline MEMORY_OPTIONS operator&(MEMORY_OPTIONS a, MEMORY_OPTIONS b)
		{
			return static_cast<MEMORY_OPTIONS>(static_cast<int>(a) & static_cast<int>(b));
		}
		enum MEMORY_USAGE
		{
			USAGE_SRC = VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
			USAGE_DST = VK_BUFFER_USAGE_TRANSFER_DST_BIT,
			USAGE_VERT = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
			USAGE_INDEX = VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
			USAGE_SSBO = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			USAGE_UBO = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
			USAGE_ACCEL_STRUCT = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR,
			USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY = VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
			USAGE_SBT = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR,
			USAGE_DEVICE_ADDRESS = VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT,
			USAGE_CONDITIONAL_RENDERING = VK_BUFFER_USAGE_CONDITIONAL_RENDERING_BIT_EXT,
			USAGE_INDIRECT = VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT
		};
		inline MEMORY_USAGE operator|(MEMORY_USAGE a, MEMORY_USAGE b)
		{
			return static_cast<MEMORY_USAGE>(static_cast<int>(a) | static_cast<int>(b));
		}
		inline MEMORY_USAGE operator |= (MEMORY_USAGE &a, MEMORY_USAGE b)
		{
			int ai = static_cast<int>(a);
			int bi = static_cast<int>(b);
			ai |= bi;
			return a = static_cast<MEMORY_USAGE>(ai);
		}
		class InstanceClass;
		class VertBufferOffsetLen
		{
		public:
			unsigned long long offset = 0u;
			unsigned long long len = 0u;
			unsigned long long id = 0u;
			bool operator==(const VertBufferOffsetLen& rhs) const
			{
				return (rhs.offset == offset && rhs.len == len);
			}
		};
		class ShaderResource;
		class KHR_RT::RTAccelStruct;
		class BufferClass : public CommandBuffer
		{
			friend class RasterletClass;
			friend class GeometryClass;
			friend class ImageClass;
			friend class ShaderResource;
			friend class KHR_RT::RTAccelStruct;
		private:
			InstanceClass *instanceRef;
			SemaphoreClass semaphore;

			std::vector<std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>> subAllocs;

			bool haveBuffer;
			bool haveSubAlloc;
			bool usingGiantVertBuffer;
			VertBufferOffsetLen insertedEntry;

			void RemovePast();

			unsigned int totalDataSize;
			bool clearCmdBufConstructed, copyCmdBufferConstructed;
			unsigned int lastClearValue = 0u;
			unsigned int lastCopySrcOffset = 0u, lastCopyDstOffset = 0u, lastCopyAmount = 0u;

		public:
			// vertex buffer stuff
			VkPipelineVertexInputStateCreateInfo VERT_vi;
			std::vector<VkVertexInputBindingDescription> VERT_bindingDescriptions;
			std::vector<VkVertexInputAttributeDescription> VERT_attributeDescriptions;

			// index buffer stuff
			int INDEX_count;

			VkMemoryAllocateInfo allocInfo;
			VkBuffer buffer, cachedUploadBuffer, cachedDownloadBuffer, cachedCopyBuffer;
			VkDescriptorBufferInfo descriptor;

			MEMORY_USAGE usage;
			MEMORY_OPTIONS memOpts;
			QUEUE_TYPE queueType;
			RESOURCE_SHARING_MODE sharingMode;

			BufferClass();
			void Buffer(MEMORY_OPTIONS inpMemOpts, QUEUE_TYPE inQueueType, RESOURCE_SHARING_MODE inpSharingMode, MEMORY_USAGE inpUsage, InstanceClass & inpInstance, void *inpData, unsigned int inpDataSize);
			template<typename... Args> BufferClass(Args&&... args)
			{
				Buffer(std::forward<Args>(args)...);
			}
			VertBufferOffsetLen VertexBufferClassWithStaging(MEMORY_USAGE inpUsage, InstanceClass & inpInstance, void *inpData, unsigned int inpDataSize);
			void CopyToBuffer(InstanceClass& inpInstance, BufferClass& dstBuf, unsigned int inpDataSize, unsigned int srcOffset = 0u, unsigned int dstOffset = 0u);
			unsigned int getSize();
			void UploadSubData(unsigned int inpOffset, void *inpData, unsigned int inpDataSize);
			void DownloadSubData(unsigned int inpOffset, void *inpData, unsigned int inpDataSize);
			void Clear(unsigned int clearValue = 0u);
			~BufferClass();
		};
		extern thread_local BufferClass vertexStagingBuffer;
		extern thread_local BufferClass imageStagingBuffer;
		class GeometryClass;
		namespace KHR_RT
		{
			class RTAccelStruct : public CommandBuffer
			{
			protected:
				SemaphoreClass semaphore;
				struct blasBuildParams
				{
					std::vector<VkAccelerationStructureBuildGeometryInfoKHR> blasBuildInfos;
					std::vector<VkAccelerationStructureBuildRangeInfoKHR *> blasBuildRanges;
				};

				VkAccelerationStructureKHR accelStruct = VK_NULL_HANDLE;

				VkDeviceAddress blasAddress;
				BufferClass accelStructBuffer;
				BufferClass scratchBuffer;
				InstanceClass *ptrToInstance;

				bool hasAccelStruct = false, reuseUpdateCmdBuffer = false;
				bool immutable = true;

				void RemoveAccelStruct();
				void CreateAccelStruct(bool isBlas, VkAccelerationStructureGeometryKHR * inpGeom, VkAccelerationStructureBuildRangeInfoKHR * inpGeomOffset, BufferClass* rtInstanceBuffer, unsigned int instanceCount, bool inpImmutable, InstanceClass & inpInstance, blasBuildParams *inpParams = nullptr);
				void UpdateAccelStruct(bool isBlas, VkAccelerationStructureGeometryKHR * inpGeom, VkAccelerationStructureBuildRangeInfoKHR * inpGeomOffset, BufferClass* rtInstanceBuffer, unsigned int instanceCount, blasBuildParams *inpParams = nullptr);
				void FetchBlasAddress();

			public:
				RTAccelStruct();
				~RTAccelStruct();
			};
			class RTGeometry : public RTAccelStruct
			{
				friend class RTScene;
				friend class GeometryClass;
			private:
				VkAccelerationStructureGeometryKHR traceGeom;
				VkAccelerationStructureBuildRangeInfoKHR traceGeomOffset;
				bool created = false;
				bool dirty = false;

				void RemoveRTGeom();
			public:

				RTGeometry();
				~RTGeometry();
				void SetGeom(BufferClass & vertBuffer, VkDeviceAddress giantVertBufferAddress, unsigned int triCount, unsigned int vertCount, unsigned int vertDataOffset, unsigned int indexDataOffset, unsigned int vertStride, bool isAlphaKeyed, bool inpImmutable, InstanceClass & inpInstance);
				void CreateOrUpdate(blasBuildParams *inpParams = nullptr, unsigned long long* updateHash = nullptr);
				void SetDirty();
			};
		}
		class ImageClearColor
		{
		public:
			vec3 floatRgb;
			float floatAlpha;
			unsigned int uintR, uintG, uintB, uintA;
			bool useFloat;

			ImageClearColor(vec3 rgb, float alpha);
			ImageClearColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a);
		};
		class LibKTX2VDIWrapper
		{
		private:
			VkCommandPool cmdPool;
			InstanceClass* cachedInstance;
		public:
			ktxVulkanDeviceInfo ktxVDI;

			void Create(InstanceClass& ptrToInstance);
			void Destroy();
		};
		class ImageClass : public CStyleWrapper
		{
			friend class RasterletClass;
			friend class FramebufferClass;
			friend class DescriptorSets;
			friend class ShaderResource;
			friend class InstanceClass;
		public:
			VkImage image;
			VkImageView view;

			enum DEPTH_STENCIL_MODE
			{
				NONE = 0,
				SAMPLE_NONE,
				SAMPLE_DEPTH,
				SAMPLE_STENCIL
			};
		private:
			enum MIN_MAG_FILTER
			{
				MIN_MAG_NEAREST = VK_FILTER_NEAREST,
				MIN_MAG_LINEAR = VK_FILTER_LINEAR,
				MIN_MAG_CUBIC = VK_FILTER_CUBIC_IMG
			};
			enum MIPMAP_MODE
			{
				MIPMAP_NEAREST = VK_SAMPLER_MIPMAP_MODE_NEAREST,
				MIPMAP_LINEAR = VK_SAMPLER_MIPMAP_MODE_LINEAR
			};
			enum TEXTURE_ADDRESS_MODE
			{
				REPEAT = VK_SAMPLER_ADDRESS_MODE_REPEAT,
				MIRRORED_REPEAT = VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT,
				CLAMP_TO_EDGE = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE,
				CLAMP_TO_BORDER = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER,
				MIRROR_CLAMP_TO_EDGE = VK_SAMPLER_ADDRESS_MODE_MIRROR_CLAMP_TO_EDGE
			};
			enum PROVIDED_IMAGE_DATA_TYPE
			{
				IMAGE_DATA_RGB = 0,
				IMAGE_DATA_TGA,
				IMAGE_DATA_KTX
			};
			ThreadLocalCache <LibKTX2VDIWrapper>::value *ktx2VDIRef = nullptr;

			InstanceClass *cachedInstance;
			bool instanceEverSet;

			SemaphoreClass semaphore;
			ktxVulkanTexture ktxVulkanTexture;
			VkSampler sampler;
			std::vector <VkImageView> layerView;
			CommandBuffer clearCmdBuffer, setupStandAloneCmdBuffer, setupTextureTransferCmdBuffer, setupTextureGraphicsCmdBuffer, uploadCmdBuffer, downloadCmdBuffer, copyCmdBuffer;

			bool uploadedTexture;
			bool is3D, isArray;
			uint32_t width, height, depth, layers;
			uint32_t mipLevels;
			int bpp;
			unsigned int data_size;
			bool useFormat;
			FORMAT format;
			TEXTURE_DIM numDims;
			DEPTH_STENCIL_MODE depthStencilMode;
			bool createSampler, usedAsStorageTarget, linearFiltering, clampSamples, createCubeMap;
			unsigned char *downloadData = nullptr;
			unsigned int downloadBufferSize;
			VkBuffer cachedUploadBuffer, cachedDownloadBuffer;
			std::vector<std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>> subAllocs;
			std::function<unsigned int(unsigned int)> surfaceBasedWidthCallback, surfaceBasedHeightCallback;

			bool haveKTXVulkanTexture;
			bool haveSampler;
			bool haveImage;
			bool haveLayout;
			bool haveSubAlloc;
			bool haveImageView;
			bool recordedClearCmdBuffer;
			bool recordedUploadCmdBuffer;
			bool recordedCopyCmdBuffer;
			bool recordedDownloadCmdBuffer;

			enum QUEUE_TRANSFER
			{
				NO_TRANSFER,
				TRANSFER_TO_GRAPHICS_RELEASE,
				TRANSFER_TO_GRAPHICS_ACQUIRE,
			};

			void setImageLayout(VkCommandBuffer cmdbuffer, std::vector<VkImage *> & images, VkImageAspectFlags aspectMask, VkImageLayout oldImageLayout, VkImageLayout newImageLayout, int numLayers, int baseMipLevel, int mipLevelCount, QUEUE_TRANSFER queueTransfer = NO_TRANSFER);
			void CreateImageView(DEPTH_STENCIL_MODE depthStencilMode, bool useFormat, FORMAT inpFormat, TEXTURE_DIM numDims, int numLayers, int baseLayer, int mipLevelCount, VkImageView & viewPtr, bool & inpHaveImageView);
			void CreateImage(bool depthStencil, bool useFormat, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM numDims, int numLayers, int mipLevelCount, bool usedViaSampler, bool usedAsStorageTarget, QUEUE_TYPE queueType);
			void CreateMemoryForImage(QUEUE_TYPE queueType);
			void CreateSampler(MIN_MAG_FILTER minFilter, MIN_MAG_FILTER magFilter, MIPMAP_MODE mipMapMode, TEXTURE_ADDRESS_MODE uAddress, TEXTURE_ADDRESS_MODE vAddress, TEXTURE_ADDRESS_MODE wAddress, float mipLodBias, float minLod, float maxLod, bool enableAnisotropy, float maxAnisotropy);
			void CreateStandaloneImage(InstanceClass & ptrToInstance);
			void CreateTextureFromFileOrData(InstanceClass & ptrToInstance, unsigned char *data, unsigned int dataSize, PROVIDED_IMAGE_DATA_TYPE dataType, unsigned int inW, unsigned int inH, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat);
		public:
			static ThreadLocalCache <LibKTX2VDIWrapper> ktx2VDIPools;

			ImageClass();
			void RemovePast();
			static std::vector<ImageClass> FromSwapChain(InstanceClass & inpInstance);
			std::function<unsigned int(unsigned int)>& getSurfaceBasedWidthCallback();
			std::function<unsigned int(unsigned int)>& getSurfaceBasedHeightCallback();
			void CreateOnScreenDepthStencil(InstanceClass & ptrToInstance, int w, int h);
			void CreateOffScreenDepthStencil(InstanceClass & ptrToInstance, int w, int h, DEPTH_STENCIL_MODE inpDepthStencil = SAMPLE_NONE, 
				bool swapChainDependent = false, 
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; },
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			void CreateOffScreenColorAttachment(InstanceClass & ptrToInstance, FORMAT inFormat, int w, int h, bool inpLinearFiltering, bool inpClampSamples,
				bool swapChainDependent = false,
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; },
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			void CreateOffScreenColorArrayAttachment(InstanceClass & ptrToInstance, FORMAT inFormat, int w, int h, int numLayers, bool inpLinearFiltering, bool inpClampSamples,
				bool swapChainDependent = false,
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; },
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			void CreateImageStore(InstanceClass & ptrToInstance, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM inpNumDims, bool inpLinearFiltering,
				bool swapChainDependent = false,
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; },
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			void CreateCubeMap(InstanceClass & ptrToInstance, FORMAT inpFormat, int w, int h,
				bool swapChainDependent = false,
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; },
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			void CreateOffScreenDepthStencilCubeMap(InstanceClass& ptrToInstance, int w, int h, DEPTH_STENCIL_MODE inpDepthStencil = SAMPLE_NONE,
				bool swapChainDependent = false,
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; },
				const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			bool CreateTexture(InstanceClass & ptrToInstance, std::string belong, std::string fileName, unsigned int inD = 1, bool inIs3D = false, bool inIsArray = false, bool isCube = false, bool doMipMapping = true, FORMAT inpFormat = R8G8B8A8UN);
			bool CreateTexture(InstanceClass & ptrToInstance, unsigned int inW, unsigned int inH, unsigned char *inData, unsigned int inD = 1, bool inIs3D = false, bool inIsArray = false, bool isCube = false, bool doMipMapping = true, FORMAT inpFormat = R8G8B8A8UN);
			static void ClearColors(std::vector <ImageClass *> & images, ImageClearColor clearColor);
			static void CopyImages(std::vector <ImageClass *> & copySource, std::vector <ImageClass *> & copyTarget);
			void DownloadData(bool shaderReadAfterwards = false);
			void FreeLoadedData();
			unsigned char * DownloadedData();
			unsigned int DownloadedDataSize();
			void UploadData(unsigned char* inData, unsigned int inDataSize, bool shaderReadAfterwards = false);
			vec2 textureAtlasCoords;
			int getWidth();
			int getHeight();
			int getBPP();
			int getDepth();
			int getLayers();
			bool getIs3D();
			bool getIsArray();
			~ImageClass();
		};
		enum RENDER_MODE
		{
			ON_SCREEN = 0,
			OFF_SCREEN
		};
		class FramebufferClass : public CStyleWrapper
		{
			friend class RasterPipelineStateClass;
			friend class RasterletClass;
			friend class ShaderResource;
		private:
			VkRenderPass renderPass;
			std::vector <ImageClass *> colorAttachments;
			std::vector <unsigned int> colorAttachmentLayers;
			unsigned int depthAttachmentLayer;
			ImageClass *depthStencilAttachment;
			RENDER_MODE mode;
			std::vector <VkFramebuffer> swapChainBuffers;
			uint32_t currentSwapChainBuffer = 0;
			unsigned long long swapchainId = 0ul;
			VkFramebuffer attachmentsFrameBuffer;
			ImageClass attachmentsSampler;
			int width, height;
			std::function<unsigned int(unsigned int)> surfaceBasedWidthCallback, surfaceBasedHeightCallback;

			bool haveRenderPass;
			bool haveAttachmentsFrameBuffer;

			InstanceClass *cachedInstance;
			bool instanceEverSet;

			void RemovePast(int numFrameBuffersToDestroy);
		public:
			FramebufferClass();
			void AddColorAttachment(ImageClass & inAttach);
			void AddColorAttachmentWithLayer(ImageClass & inAttach, unsigned int layer);
			void SetDepthStencil(ImageClass& inAttach);
			void SetDepthStencilWithLayer(ImageClass& inAttach, unsigned int layer);
			ImageClass* GetDepthStencil();
			bool IsDepthStencilArrayed();
			ImageClass & GetSampler();
			void Create(RENDER_MODE inpMode, InstanceClass & inpInstance, WindowClass & windowRef, bool maintainAttachmentData = false);
			bool RecreateIfStale();
			void setWidth(int width);
			void setHeight(int height);
			void setSwapchainDependentWidthCallback(const std::function<unsigned int(unsigned int)>& surfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; });
			void setSwapchainDependentHeightCallback(const std::function<unsigned int(unsigned int)>& surfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			RENDER_MODE getRenderMode();
			int getWidth();
			int getHeight();
			void RemovePast();
			bool IsSwapchainDependent();
			~FramebufferClass();
		};
		class InstanceClass : public CStyleWrapper
		{
			friend bool getMemoryType(InstanceClass *ptrToInstance, uint32_t typeBits, VkFlags properties, uint32_t * typeIndex);
			friend VkResult MEMORY_MANAGER::LIBKTX2::BindBufferMemoryCWrapper(VkBuffer buffer, uint64_t allocId);
			friend VkResult MEMORY_MANAGER::LIBKTX2::BindImageMemoryCWrapper(VkImage image, uint64_t allocId);
			friend void MEMORY_MANAGER::LIBKTX2::LockQueue(VkQueue queue);
			friend void MEMORY_MANAGER::LIBKTX2::UnlockQueue(VkQueue queue);
			friend class DescriptorSetLayout;
			friend class DescriptorSets;
			friend class KHR_RT::RTInstance;
			friend class KHR_RT::RTAccelStruct;
			friend class KHR_RT::RTPipelineStateClass;
			friend class KHR_RT::RTScene;
			friend class KHR_RT::RTTracelet;
			friend class KHR_RT::RTGeometry;
			friend class ImageClass;
			friend class FramebufferClass;
			friend class RasterPipelineStateClass;
			friend class ComputePipelineStateClass;
			friend class SemaphoreClass;
			friend class FenceClass;
			friend class RasterletClass;
			friend class ComputeletClass;
			friend class BufferClass;
			friend class ShaderStage;
			friend class CommandBuffer;
			friend class Timestamp;
			friend class LibKTX2VDIWrapper;
		private:
			InstanceClass(const InstanceClass &obj);
			void operator=(const InstanceClass& b);

			bool validationLayer;
			bool supportsHWRT;
			static bool lowMemoryDevice;
			bool supportsSparseResources;
			unsigned long long vramAmount;

			struct memMapInfo
			{
				unsigned long long offset, len;
				void* dataPtr;
				unsigned int refCount = 0;
			};
			static std::unordered_map<VkDeviceMemory, memMapInfo> allMemMaps;

			struct queueObj
			{
				VkQueue queue;
				std::mutex mtx;
				std::string name;
				uint32_t nodeIndex = UINT32_MAX;
			};
			queueObj allQueues[QUEUE_TYPE::MAX_QUEUES];

			VkInstance instance;
			VkPhysicalDevice physicalDevice;
			VkDevice device;
			VkPhysicalDeviceProperties deviceProps;
			VkPhysicalDeviceMemoryProperties deviceMemoryProperties;
			FenceClass acquireImageFence;
			VkSubmitInfo submitInfo;
			VkPipelineStageFlags submitPipelineStages = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
			VkSurfaceKHR surface;
			VkFormat selectedDepthFormat;
			VkFormat colorFormat;
			VkColorSpaceKHR colorSpace;
			uint32_t presentQueueNodeIndex = UINT32_MAX;
			VkSwapchainKHR swapChain = VK_NULL_HANDLE;
			std::vector <ImageClass> swapChainImages;
			unsigned long long swapchainId = 0ull;
			std::unordered_set <ImageClass*> swapChainDependentImages;
			ImageClass depthStencilImage;
			FramebufferClass frameBuffer;

			PFN_vkGetPhysicalDeviceSurfaceSupportKHR fpGetPhysicalDeviceSurfaceSupportKHR;
			PFN_vkGetPhysicalDeviceSurfaceCapabilitiesKHR fpGetPhysicalDeviceSurfaceCapabilitiesKHR;
			PFN_vkGetPhysicalDeviceSurfaceFormatsKHR fpGetPhysicalDeviceSurfaceFormatsKHR;
			PFN_vkGetPhysicalDeviceSurfacePresentModesKHR fpGetPhysicalDeviceSurfacePresentModesKHR;
			PFN_vkCreateSwapchainKHR fpCreateSwapchainKHR;
			PFN_vkDestroySwapchainKHR fpDestroySwapchainKHR;
			PFN_vkGetSwapchainImagesKHR fpGetSwapchainImagesKHR;
			PFN_vkAcquireNextImageKHR fpAcquireNextImageKHR;
			PFN_vkQueuePresentKHR fpQueuePresentKHR;
			PFN_vkWaitSemaphoresKHR fpWaitSemaphoresKHR;

			PFN_vkCreateDebugReportCallbackEXT CreateDebugReportCallback;
			PFN_vkDestroyDebugReportCallbackEXT DestroyDebugReportCallback;
			PFN_vkDebugReportMessageEXT dbgBreakCallback;
			VkDebugReportCallbackEXT msgCallback;

			PFN_vkCreateDescriptorUpdateTemplateKHR fpCreateDescriptorUpdateTemplateKHR;
			PFN_vkUpdateDescriptorSetWithTemplateKHR fpUpdateDescriptorSetWithTemplateKHR;
			PFN_vkDestroyDescriptorUpdateTemplateKHR fpDestroyDescriptorUpdateTemplateKHR;

			PFN_vkCmdDrawIndexedIndirectCountKHR fpCmdDrawIndexedIndirectCountKHR;

			bool haveDevice;
			bool haveInstance;
			bool haveSurface;
			bool haveDebugCallback;

			bool haveSwapChain;

			void CreateSwapChainRemovePast();
		public:
			InstanceClass();
			void Make(bool validationLayer, WindowClass &inpWindow, bool requestHWRT, bool headless);
			template<typename... Args> InstanceClass(Args&&... args)
			{
				Make(std::forward<Args>(args)...);
			}
			void CreateSwapChain(WindowClass &windowRef);
			void AddSwapchainDependentImage(ImageClass& inpImg, 
											const std::function<unsigned int(unsigned int)>& surfaceBasedWidthCallback = [](unsigned int screenW) -> unsigned int { return screenW; }, 
											const std::function<unsigned int(unsigned int)>& surfaceBasedHeightCallback = [](unsigned int screenH) -> unsigned int { return screenH; });
			void RecreateSwapchainDependentImages(unsigned int width, unsigned int height);
			bool isIntel();
			FramebufferClass & swapChainFrameBuffer();
			bool SupportsHWRT();
			static bool LowMemoryDevice();
			bool SupportsSparseResources();
			void QueueFlush(QUEUE_TYPE queueToFlush);
			void GPUFlush();
			unsigned long long getSwapchainId();
			void RemovePast();
			VkResult ThreadSafeMapMemory(VkDeviceMemory inMem, VkDeviceSize offset, VkDeviceSize len, VkMemoryMapFlags flags, void** ppData);
			void ThreadSafeUnmapMemory(VkDeviceMemory inMem);
			~InstanceClass();
		};
		bool getMemoryType(InstanceClass *ptrToInstance, uint32_t typeBits, VkFlags properties, uint32_t * typeIndex);
		enum PIPELINE_STAGE
		{
			VERTEX = VK_SHADER_STAGE_VERTEX_BIT,
			TESS_CTRL = VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT,
			TESS_EVAL = VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT,
			GEOMETRY = VK_SHADER_STAGE_GEOMETRY_BIT,
			FRAGMENT = VK_SHADER_STAGE_FRAGMENT_BIT,
			GRAPHICS = VK_SHADER_STAGE_ALL_GRAPHICS,
			COMPUTE = VK_SHADER_STAGE_COMPUTE_BIT,
			RT_RAYGEN = VK_SHADER_STAGE_RAYGEN_BIT_KHR,
			RT_ANYHIT = VK_SHADER_STAGE_ANY_HIT_BIT_KHR,
			RT_RCHIT = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR,
			RT_MISS = VK_SHADER_STAGE_MISS_BIT_KHR,
			RT_INTERSECTION = VK_SHADER_STAGE_INTERSECTION_BIT_KHR,
			ALL = VK_SHADER_STAGE_ALL
		};
		inline PIPELINE_STAGE operator|(PIPELINE_STAGE a, PIPELINE_STAGE b)
		{
			return static_cast<PIPELINE_STAGE>(static_cast<int>(a) | static_cast<int>(b));
		}
		enum SHADER_RESOURCE_TYPE
		{
			RESOURCE_UBO = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
			RESOURCE_SSBO = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
			RESOURCE_SAMPLER = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
			RESOURCE_IMAGE_STORE = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE,
			RESOURCE_RT_ACCEL_STRUCT = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR
		};
		class ShaderResource
		{
			friend class DescriptorSetLayout;
			friend class DescriptorSets;
			friend class RasterPipelineStateClass;
			friend class ComputePipelineStateClass;
			friend class KHR_RT::RTPipelineStateClass;
			friend class RasterletClass;
		public:
			enum SHADER_RESOURCE_USAGE
			{
				USAGE_NOT_A_DEPENDENCY,
				USAGE_CONSUMER,
				USAGE_PRODUCER,
				USAGE_SIMULTANEOUS_PRODUCER
			};
		private:
			SHADER_RESOURCE_TYPE type;
			PIPELINE_STAGE visibility;
			unsigned int bindId;
			unsigned int setId;
			ImageClass *samplerRef = nullptr;
			ImageClass *imageViewRef = nullptr;
			unsigned int imageLayer = 0;
			BufferClass *uniformRef = nullptr;
			KHR_RT::RTScene *rtSceneRef = nullptr;
			std::vector <ShaderResource> arrayedResource;
			bool isVariableCount;
			SHADER_RESOURCE_USAGE usage;
		public:
			ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, ImageClass &inpImageRef, int layer = -1, SHADER_RESOURCE_USAGE inUsage = USAGE_CONSUMER);
			ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, BufferClass &inpUniformRef, SHADER_RESOURCE_USAGE inUsage = USAGE_CONSUMER);
			ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, FramebufferClass & inpImageRef, int attachmentNumber, SHADER_RESOURCE_USAGE inUsage = USAGE_CONSUMER);
			ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, KHR_RT::RTScene & inpSceneRef, SHADER_RESOURCE_USAGE inUsage = USAGE_CONSUMER);
			ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, std::vector <ShaderResource> & inpVariableCountResource, SHADER_RESOURCE_USAGE inUsage = USAGE_CONSUMER);
			ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, std::vector <ShaderResource> & inpVariableCountResource, SHADER_RESOURCE_USAGE inUsage = USAGE_CONSUMER);
			unsigned int ResourceCount();
			std::unordered_set<SemaphoreClass*> GetAsDependencies();
			bool IsProduced();
			bool IsSimultaneouslyProduced();
			bool IsSwapchainDependent();
			unsigned long long GetHash();
		};
		struct ShaderSpecilization
		{
			struct Constant
			{
				unsigned int bindPoint;
				unsigned int value;
			};
			std::string name;
			std::vector<Constant> inputs;
		};
		class ShaderStage
		{
		private:
			InstanceClass *ptrToInstance;
		public:
			VkPipelineShaderStageCreateInfo stage;

			struct SpecializationDataStruct
			{
				std::vector<unsigned int> *dataBuf = nullptr;
				std::vector<VkSpecializationMapEntry> *entries = nullptr;
				VkSpecializationInfo *info = nullptr;
			} SpecializationData;

			ShaderStage();
			void Shader(InstanceClass &ptrToInstance, std::string shaderFile, const char * entryName, VkShaderStageFlagBits stage_bit, ShaderSpecilization* shaderSpec = nullptr);
			template<typename... Args> ShaderStage(Args&&... args)
			{
				Shader(std::forward<Args>(args)...);
			}
			~ShaderStage();
		};
		CacheItem<ShaderStage> * AddOrFindCachedShaderStage(InstanceClass &ptrToInstance, std::string shaderFile, const char * entryName, VkShaderStageFlagBits stage_bit, ShaderSpecilization* shaderSpec, std::string& shaderStageKey);
		class ShaderResourceSet
		{
			friend class RasterPipelineStateClass;
			friend class ComputePipelineStateClass;
			friend class KHR_RT::RTPipelineStateClass;
		public:
			std::string vertex_shader;
			std::string vertex_entry;
			std::string tc_shader;
			std::string tc_entry;
			std::string te_shader;
			std::string te_entry;
			std::string geom_shader;
			std::string geom_entry;
			std::string fragment_shader;
			std::string fragment_entry;
			std::string compute_shader;
			std::string compute_entry;
			std::string rt_raygen_shader;
			std::string rt_raygen_entry;
			std::string rt_raychit_shader;
			std::string rt_raychit_entry;
			std::string rt_raymiss_shader;
			std::string rt_raymiss_entry;
			std::string rt_rayahit_shader;
			std::string rt_rayahit_entry;
			std::unordered_map<PIPELINE_STAGE, ShaderSpecilization> stageSpecializationData;

		private:
			char *vertex_entry_cstr;
			char *tc_entry_cstr;
			char *te_entry_cstr;
			char *geom_entry_cstr;
			char *fragment_entry_cstr;
			char *compute_entry_cstr;
			char *rt_raygen_entry_cstr;
			char *rt_raychit_entry_cstr;
			char *rt_raymiss_entry_cstr;
			char *rt_rayahit_entry_cstr;
			std::vector <ShaderResource> additionalResources;
			void StringToCString(char **destCString, const std::string& inpString);
		public:
			ShaderResourceSet();
			void Create(const std::string& inpComp, const std::string& inpCompEnt);
			void Create(const std::string& inpVert, const std::string& inpVertEnt, const std::string& inpFrag, const std::string& inpFragEnt);
			void Create(const std::string& inpVert, const std::string& inpVertEnt, const std::string& inpGeom, const std::string& inpGeomEnt, const std::string& inpFrag, const std::string& inpFragEnt);
			void Create(const std::string& inpVert, const std::string& inpVertEnt, const std::string& inpTessCtrl, const std::string& inpTessCtrlEnt, const std::string& inpTessEval, const std::string& inpTessEvalEnt, const std::string& inpFrag, const std::string& inpFragEnt);
			void CreateRT(const std::string& inpRaygen, const std::string& inpRaygenEnt, const std::string& inpRaychit, const std::string& inpRaychitEnt, const std::string& inpRaymiss, const std::string& inpRaymissEnt);
			void CreateRT(const std::string& inpRaygen, const std::string& inpRaygenEnt, const std::string& inpRaychit, const std::string& inpRaychitEnt, const std::string& inpRaymiss, const std::string& inpRaymissEnt, const std::string& inpRayahit, const std::string& inpRayahitEnt);
			void SetStageSpecializationData(PIPELINE_STAGE&& inStage, ShaderSpecilization inSpecializationData);
			char *getCompEntry();
			char *getVertEntry();
			char *getTCEntry();
			char *getTEEntry();
			char *getGeomEntry();
			char *getFragEntry();
			char *getRaygenEntry();
			char *getRaychitEntry();
			char *getRaymissEntry();
			char *getRayahitEntry();
			template<typename... A> void AddResource(A&&... a);
			void ClearResources();
			void RemovePast();
			std::vector <ShaderResource> & getAdditionalResources();
			bool areAdditionalResourcesSwapchainDependent();
			ShaderSpecilization* getStageSpecializationDataRef(PIPELINE_STAGE&& inStage);
			~ShaderResourceSet();
		};
		class DescriptorSetLayout : public CStyleWrapper
		{
			friend class InstanceClass;
			friend class DescriptorSets;
			friend class RasterPipelineStateClass;
			friend class ComputePipelineStateClass;
			friend class KHR_RT::RTPipelineStateClass;
		private:
			void RemovePast();

		protected:
			static InstanceClass * ptrToInstance;

			std::vector <VkDescriptorSetLayout> descriptorSetLayouts;
			std::vector<unsigned int> allSets;

		public:

			DescriptorSetLayout();
			void CreateDescriptorSetLayout(std::vector<ShaderResource>& allResources, InstanceClass * inpPtrToInstance);
			~DescriptorSetLayout();
		};
		class DescriptorSets : public CStyleWrapper
		{
			friend class InstanceClass;
			friend class RasterletClass;
			friend class ComputeletClass;
			friend class KHR_RT::RTTracelet;
		private:
			void RemovePast();

		protected:
			DescriptorSetLayout * ptrToDescSetLayout;
			static InstanceClass * ptrToInstance;

			std::vector <VkDescriptorSet> descriptorSets;
			std::unordered_map<unsigned int, std::vector<unsigned char>> descriptorUpdateTemplateData;
			std::unordered_map<unsigned int, std::vector<VkDescriptorUpdateTemplateEntry>> descriptorUpdateTemplateEntries;

			static ThreadLocalCache <std::vector<VkDescriptorPool>> descriptorPools;
			ThreadLocalCache <std::vector<VkDescriptorPool>>::value *descriptorPoolPtr = nullptr;
			VkDescriptorPool poolObject;

		public:

			DescriptorSets();
			void Make(DescriptorSetLayout * inpPtrToDescSetLayout);
			template<typename... Args> DescriptorSets(Args&&... args)
			{
				Make(std::forward<Args>(args)...);
			}
			void WriteDescriptorSets(std::vector<ShaderResource>& allResources);
			void RewriteDescriptorSets(std::vector<ShaderResource>& allResources);
			void UpdateDescriptorSets(std::vector<ShaderResource>& allResources);
			~DescriptorSets();
		};
		typedef short unsigned int HIGHOMEGA_TEXTURE_OFFSET;
#define HIGHOMEGA_INSTANCEPROPERTIES_MINMAX_OFFSET		   0 
#define HIGHOMEGA_INSTANCEPROPERTIES_TRIANGLEOFFSET_OFFSET (sizeof(float) * 13 + sizeof(HIGHOMEGA_TEXTURE_OFFSET) * 6) 
		struct InstanceProperties
		{
			float geomMin[3];
			float geomMax[3];
			float attribs1[3];
			float attribs2[4];
			HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6];
			unsigned int idxVertOffset;
			unsigned int transformOffset;
			unsigned int prevTransformOffset;
		};
		enum BlendFactors
		{
			FACTOR_ZERO = VK_BLEND_FACTOR_ZERO,
			FACTOR_ONE = VK_BLEND_FACTOR_ONE,
			FACTOR_SRC_COLOR = VK_BLEND_FACTOR_SRC_COLOR,
			FACTOR_ONE_MINUS_SRC_COLOR = VK_BLEND_FACTOR_ONE_MINUS_SRC_COLOR,
			FACTOR_DST_COLOR = VK_BLEND_FACTOR_DST_COLOR,
			FACTOR_ONE_MINUS_DST_COLOR = VK_BLEND_FACTOR_ONE_MINUS_DST_COLOR,
			FACTOR_SRC_ALPHA = VK_BLEND_FACTOR_SRC_ALPHA,
			FACTOR_ONE_MINUS_SRC_ALPHA = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA,
			FACTOR_DST_ALPHA = VK_BLEND_FACTOR_DST_ALPHA,
			FACTOR_ONE_MINUS_DST_ALPHA = VK_BLEND_FACTOR_ONE_MINUS_DST_ALPHA,
			FACTOR_CONSTANT_COLOR = VK_BLEND_FACTOR_CONSTANT_COLOR,
			FACTOR_ONE_MINUS_CONSTANT_COLOR = VK_BLEND_FACTOR_ONE_MINUS_CONSTANT_COLOR,
			FACTOR_CONSTANT_ALPHA = VK_BLEND_FACTOR_CONSTANT_ALPHA,
			FACTOR_ONE_MINUS_CONSTANT_ALPHA = VK_BLEND_FACTOR_ONE_MINUS_CONSTANT_ALPHA,
			FACTOR_SRC_ALPHA_SATURATE = VK_BLEND_FACTOR_SRC_ALPHA_SATURATE,
			FACTOR_SRC1_COLOR = VK_BLEND_FACTOR_SRC1_COLOR,
			FACTOR_ONE_MINUS_SRC1_COLOR = VK_BLEND_FACTOR_ONE_MINUS_SRC1_COLOR,
			FACTOR_SRC1_ALPHA = VK_BLEND_FACTOR_SRC1_ALPHA,
			FACTOR_ONE_MINUS_SRC1_ALPHA = VK_BLEND_FACTOR_ONE_MINUS_SRC1_ALPHA
		};
		enum BlendOps
		{
			BLEND_ADD = VK_BLEND_OP_ADD,
			BLEND_SUBTRACT = VK_BLEND_OP_SUBTRACT,
			BLEND_BLEND_REVERSE_SUBTRACT = VK_BLEND_OP_REVERSE_SUBTRACT,
			BLEND_BLEND_MIN = VK_BLEND_OP_MIN,
			BLEND_BLEND_MAX = VK_BLEND_OP_MAX
		};
		enum CompareTypes
		{
			COMPARE_NEVER = VK_COMPARE_OP_NEVER,
			COMPARE_LESS = VK_COMPARE_OP_LESS,
			COMPARE_EQUAL = VK_COMPARE_OP_EQUAL,
			COMPARE_LESS_OR_EQUAL = VK_COMPARE_OP_LESS_OR_EQUAL,
			COMPARE_GREATER = VK_COMPARE_OP_GREATER,
			COMPARE_NOT_EQUAL = VK_COMPARE_OP_NOT_EQUAL,
			COMPARE_GREATER_OR_EQUAL = VK_COMPARE_OP_GREATER_OR_EQUAL,
			COMPARE_ALWAYS = VK_COMPARE_OP_ALWAYS
		};
		enum StencilOperation
		{
			STENCIL_KEEP = VK_STENCIL_OP_KEEP,
			STENCIL_ZERO = VK_STENCIL_OP_ZERO,
			STENCIL_REPLACE = VK_STENCIL_OP_REPLACE,
			STENCIL_INCREMENT_AND_CLAMP = VK_STENCIL_OP_INCREMENT_AND_CLAMP,
			STENCIL_DECREMENT_AND_CLAMP = VK_STENCIL_OP_DECREMENT_AND_CLAMP,
			STENCIL_INVERT = VK_STENCIL_OP_INVERT,
			STENCIL_INCREMENT_AND_WRAP = VK_STENCIL_OP_INCREMENT_AND_WRAP,
			STENCIL_DECREMENT_AND_WRAP = VK_STENCIL_OP_DECREMENT_AND_WRAP
		};
		class PipelineFlags
		{
		public:
			bool depthWrite, depthTest;
			CompareTypes depthCompare;
			bool stencilWrite, stencilTest;
			CompareTypes stencilCompare;
			StencilOperation stencilOp;
			unsigned int stencilCompareValue;
			bool backFaceCulling, frontFaceCulling, frontFaceClockWise;
			bool blendEnable, alphaBlending, colorBlending;
			BlendFactors srcAlphaFactor, dstAlphaFactor, srcColorFactor, dstColorFactor;
			BlendOps alphaBlendOp, colorBlendOp;
			bool redMask, greenMask, blueMask, alphaMask;

			bool changedDepthWrite, changedDepthTest;
			bool changedBackFaceCulling, changedFrontFaceCulling, changedFrontFaceClockWise;
			bool changedBlendEnable;
			bool changedColorMask;

			PipelineFlags();
			bool operator==(const PipelineFlags &other) const;
		};
		class PipelineFlagsHash
		{
		public:
			std::size_t operator()(const PipelineFlags& k) const;
		};
		namespace KHR_RT
		{
			class RTPipelineStateClass
			{
				friend class RTTracelet;
			private:
				std::vector<VkPipelineShaderStageCreateInfo> shaderStages;
				std::vector<VkRayTracingShaderGroupCreateInfoKHR> shaderGroups;
				BufferClass shaderBindingTable;
				std::vector<std::string> usedShaderStages;

				VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
				VkPipeline pipeline = VK_NULL_HANDLE;
				VkPipelineCache pipelineCache = VK_NULL_HANDLE;

				bool havePipelineLayout;
				bool haveRTPipeline;
				bool havePipelineCache;

				InstanceClass *ptrToInstance;
				void ErasePipelineState();
			public:
				RTPipelineStateClass();
				void RTPipelineState(InstanceClass & renderInst, DescriptorSetLayout & DescSetLayout, ShaderResourceSet & inpShader);
				template<typename... Args> RTPipelineStateClass(Args&&... args)
				{
					RTPipelineState(std::forward<Args>(args)...);
				}
				bool IsInitialized();
				~RTPipelineStateClass();
			};
			class RTTracelet;
			struct TraceItem
			{
				GeometryClass *geomRef;
				mat4 theMat;
				VkAccelerationStructureInstanceKHR rtInstanceData;
			};
			class RTScene : public RTAccelStruct
			{
				friend class DescriptorSets;
				friend class RTPipelineStateClass;
				friend class RTTracelet;
				friend class HIGHOMEGA::GL::ShaderResource;
			private:
				unsigned long long sceneId = 0ull, previousUpdateHash = 0ull;
				InstanceClass *ptrToInstance;
				std::vector <VkAccelerationStructureInstanceKHR> instances;
				BufferClass rtInstanceBuffer;
				BufferClass rtCopyTransformParamsSSBO;
				std::unordered_map <unsigned long long, std::vector<TraceItem>> allTraceItems;
				bool needReCreation = false;
				bool needUpdate = false;

				void CreateInstanceData(std::vector <VkAccelerationStructureInstanceKHR> & traceItemsInstanceData);

			public:

				RTScene();
				~RTScene();
				void MarkSceneDirty();
				void MarkSceneNeedingUpdate();
				unsigned int GetTraceItemCount();
				void Add(unsigned long long inpId, unsigned int instanceId, GeometryClass & inpGeom, mat4 inpMat, InstanceClass & inpInstance);
				void Remove(GeometryClass& inpGeom);
				void RemoveAll(unsigned long long inpId);
				void DeleteRTResources();
				unsigned long long rtSceneID(std::function<void(unsigned int, BufferClass*, BufferClass*, bool)>& rtCopyTransforms);
			};
			class RTTracelet : public CommandBuffer
			{
			private:
				std::unordered_set<SemaphoreClass*> waitSems;
				std::unordered_set<SemaphoreClass*> waitOldSems;
				std::unordered_set<SemaphoreClass*> signalSems;
				unsigned long long swapchainId = 0ull;
				bool doCPUSync = true;
				DescriptorSetLayout DSL;
				RTPipelineStateClass PSO;
				DescriptorSets DS;
				InstanceClass *ptrToInstance = nullptr;
				bool recordedPSO = false;
				bool recordedCmdBuf = false;

			public:
				RTTracelet();
				void Make(InstanceClass & inpInstance);
				template<typename... Args> RTTracelet(Args&&... args)
				{
					Make(std::forward<Args>(args)...);
				}
				RTTracelet& MakeAsync();
				void Submit(unsigned int inpWidth, unsigned int inpHeight, unsigned int inpDepth, std::vector<ShaderResource> & inpTracingResources, bool updateResources, ShaderResourceSet & inpRTShaderResourceSet);
			};
		}
		class RasterPipelineStateClass
		{
			friend class RasterletClass;
		private:
			std::vector<std::string> usedShaderStages;
			VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
			VkPipeline pipeline = VK_NULL_HANDLE;
			VkPipelineCache pipelineCache = VK_NULL_HANDLE;

			bool havePipelineLayout;
			bool haveGraphicsPipeline;
			bool havePipelineCache;

			InstanceClass *ptrToInstance;
			void ErasePipelineState();
		public:
			RasterPipelineStateClass();
			RasterPipelineStateClass(InstanceClass & renderInst, DescriptorSetLayout & DescSetLayout, PipelineFlags &inpFlags, FramebufferClass & frameBuffer, GeometryClass &geomRef, ShaderResourceSet & inpShader);
			~RasterPipelineStateClass();
		};
		class Raster_PSO_DSL
		{
		public:
			RasterPipelineStateClass *PSO, *PSOMainPass = nullptr;
			DescriptorSetLayout *DSL;
		};
		extern std::unordered_map<std::string, Raster_PSO_DSL> globalRaster_PSO_DSL_Cache;
		extern std::mutex globalRaster_PSO_DSL_Cache_mutex;
		class ComputePipelineStateClass
		{
			friend class ComputeletClass;
		private:
			std::string usedShaderStage = std::string("");

			VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
			VkPipeline pipeline = VK_NULL_HANDLE;
			VkPipelineCache pipelineCache = VK_NULL_HANDLE;

			bool havePipelineLayout;
			bool haveComputePipeline;
			bool havePipelineCache;

			InstanceClass *ptrToInstance;
			void ErasePipelineState();
		public:
			ComputePipelineStateClass();
			ComputePipelineStateClass(InstanceClass & renderInst, DescriptorSetLayout & DescSetLayout, ShaderResourceSet & inpShader);
			~ComputePipelineStateClass();
		};
		class Compute_PSO_DSL
		{
		public:
			ComputePipelineStateClass *PSO;
			DescriptorSetLayout *DSL;
		};
		extern std::unordered_map<std::string, Compute_PSO_DSL> globalCompute_PSO_DSL_Cache;
		extern std::mutex globalCompute_PSO_DSL_Cache_mutex;
		extern BufferClass* giantVertBuffer;
		extern unsigned int giantVertBufferClaims;
		extern std::vector<VertBufferOffsetLen> giantVertBufferSubAllocs;
		extern std::shared_mutex giantVertBufferSharedMutex;
		extern std::mutex giantVertBufferDirectoryMutex;

		struct PSO_DSL_DS_GeomInstPairing
		{
			Raster_PSO_DSL *PSO_DSL;
			DescriptorSets *DS;
			int renderOrder = 0u;
			GeometryClass* geom;
			unsigned int inst;
		};
		class RasterletClass : public CommandBuffer
		{
		public:
			std::unordered_set<SemaphoreClass*> waitSems;
			std::unordered_set<SemaphoreClass*> waitOldSems;
			std::unordered_set<SemaphoreClass*> signalSems;
			bool doCPUSync = true;
			struct IndirectBoundaries
			{
				unsigned long long offset;
				unsigned int indDrawCount;
				RasterPipelineStateClass *PSOPtr, *PSOMainPassPtr;
				DescriptorSets* DSPtr;
			};
			std::vector<IndirectBoundaries> indirectBoundaries;
			struct DrawCommand // Exactly matches VkDrawIndexedIndirectCommand
			{
				unsigned int indexCount;
				unsigned int instanceCount;
				unsigned int firstIndex;
				unsigned int vertexOffset;
				unsigned int firstInstance;
			};

			struct DynamicPipelineFlags
			{
				unsigned int viewport_x, viewport_y;
				unsigned int viewport_width, viewport_height;
				float depth_clear;
				unsigned int stencil_clear;
				float clear_color[4];
			};
			bool doesCulling = false;
			BufferClass* indirectDrawBuffer = nullptr;

		private:

			InstanceClass *ptrToInstance = nullptr;
			FramebufferClass *ptrToFrameBuffer = nullptr;

			void RecordCommandBuffers(std::vector <PSO_DSL_DS_GeomInstPairing> & PSO_DSL_DS_GeomInstPairings, DynamicPipelineFlags &inpDynamicFlags, VkRenderPassBeginInfo & renderPassBeginInfo, unsigned int numCmdBuf, unsigned int whichCmdBuf, VkFramebuffer & inpFrameBuffer, VkImage *inpImg, RENDER_MODE inpMode, bool mainPass = false);

		public:
			RasterletClass();
			~RasterletClass();
			void Rasterlet(InstanceClass &renderInst);
			template<typename... Args> RasterletClass(Args&&... args)
			{
				Rasterlet(std::forward<Args>(args)...);
			}
			unsigned int IndirectBufferSize(unsigned int nInstances, unsigned int nBatches);
			unsigned int IndirectDrawCounterSize();
			void ResetRasterlet();
			void PrepareSubmission(std::vector <PSO_DSL_DS_GeomInstPairing> & PSO_DSL_DS_GeomInstPairings, DynamicPipelineFlags &inpDynamicFlags, FramebufferClass & frameBuffer, FramebufferClass* frameBufferMainPass);
			void Draw(bool prepass = false);
		};
		class ComputeletClass : public CommandBuffer
		{
		private:
			InstanceClass *instanceRef = nullptr;

		public:
			ComputeletClass();
			void Computelet(InstanceClass &renderInst);
			template<typename... Args> ComputeletClass(Args&&... args)
			{
				Computelet(std::forward<Args>(args)...);
			}
			void Start(ComputePipelineStateClass & PSO, QUEUE_TYPE inQueueType);
			void Dispatch(ComputePipelineStateClass & PSO, DescriptorSets & DS, int groupX, int groupY, int groupZ);
			void End();
			void Submit();
		};
		struct RasterVertex
		{
			float posCol[4];
			float uv;
			unsigned int Norm;
		};
		unsigned int getTriCount(std::vector<unsigned char>& idxVertData);
		unsigned int getVertexCount(std::vector<unsigned char>& idxVertData);
		void getIndicesVertices(std::vector<unsigned char>& idxVertData, RasterVertex** vertices, unsigned int& vertexCount, unsigned int** indices, unsigned int& triCount, unsigned int& vertexDataOffset);
		void createIndicesVertices(std::vector<unsigned int>& indices, std::vector<RasterVertex>& vertices, std::vector<unsigned char>& idxVertData);
		void createEmptyIndicesVertices(unsigned int nIndices, unsigned int nVertices, std::vector<unsigned char>& idxVertData);
		void multiplyGeometry(std::vector<unsigned char>& outIndexVertexData, unsigned int mulFactor);
		void collateGeometry(std::vector<unsigned char>& inOutIndexVertexData, std::vector<unsigned char>& inIndexVertexDataToAdd);
		bool isIndexVertexDataTooSmall(std::vector<unsigned char>& idxVertData);
		float packVertexVelocity(const vec3& inpVel, float inpPackedColor);
		void packRasterVertex(const vec3& pos, const vec3& col, const vec2& uv, const vec3& vNorm, RasterVertex & rv);
		void packRasterVertex(const vec3& pos, float packedColor, const vec2& uv, const vec3& vNorm, RasterVertex& rv);
		void unpackRasterVertex(vec3& pos, vec3& col, vec2& uv, vec3& vNorm, const RasterVertex& rv);
		struct VertexAnimationInfo
		{
			float bonesWeights[4];
		};
		class ChangeSignalClass;
		class GeometryClass
		{
			friend class RasterPipelineStateClass;
			friend class RasterletClass;
			friend class KHR_RT::RTScene;
		public:
			std::unordered_map <ChangeSignalClass *, unsigned int> notifySubmissions;
			class DataLayout
			{
			public:
				DataLayout();
				template<typename ... A>
				DataLayout(int inpBindPoint, FORMAT first, A&& ... a);
				std::vector<FORMAT> DataPoints;
				unsigned int BindPoint;
			};
		private:
			InstanceClass *instanceRef;

			BufferClass vertBuffer, vertAnimInfoBuf;
			KHR_RT::RTGeometry rtGeom;
			std::string groupId;
			std::string armatureId;

			vec3 geomCorners[8];
			vec3 geomMin, geomMax;
			bool downloadConstructed = false;
			bool uploadConstructed = false;
			VertBufferOffsetLen giantVertBufferOffsetLen;
			unsigned int totalDataSize;
			unsigned int triCount;
			unsigned int vertCount;
			unsigned int vertexDataOffsetFromDataStart;
			unsigned int vertAnimInfoBufSize;

			DataLayout layoutCache;
			bool isAlphaKeyedCache;
			bool breakableCache;
			bool immutable = true;
			
		public:

			~GeometryClass();
			void RemovePast();
			GeometryClass();
			void ChangeGeom(std::vector<unsigned char>& inpIndexVertexData);
			void Geometry(InstanceClass & inpInstance, std::vector<unsigned char>& inpIndexVertexData, const DataLayout &inpDataLayout, bool isAlphaKeyed, const std::string & inpGroupId, bool inpImmutable = true, std::vector <VertexAnimationInfo> *vertAnimInfo = nullptr, std::string *inpArmatureId = nullptr, bool breakable = false);
			template<typename... Args> GeometryClass(Args&&... args)
			{
				Geometry(std::forward<Args>(args)...);
			}
			void SetMinMax(vec3 minVal, vec3 maxVal);
			void Download(std::vector<unsigned char>& outIndexVertexData);
			void Update(std::vector<unsigned char>& inpIndexVertexData);
			KHR_RT::RTGeometry & getRTGeom();
			BufferClass & getAnimInfoBuf();
			void setRTBufferDirty();
			std::string getGroupId();
			std::string & getArmatureId();
			static void getMinMax(RasterVertex* verts, unsigned int vertCount, vec3& outMin, vec3& outMax);
			vec3 & getGeomMin();
			vec3 & getGeomMax();
			unsigned int getVertexOffsetInGiantVertexBuffer();
			unsigned int getIndexOffsetInGiantVertexBuffer();
			unsigned int getDataOffsetInGiantVertexBuffer();
			unsigned int getTriCount();
			unsigned int getVertCount();
			unsigned int getTotalDataSize();
			bool getBreakable();
		};
		class ChangeSignalClass
		{
		public:
			virtual void ChangeSignal(GeometryClass *) = 0;
		};

		void UnpackMat4(const mat4 & inpMat,void *loc);
		void PackMat4(void * loc, mat4 & inpMat);

		extern InstanceClass Instance;
		extern WindowClass Window;
		template<typename ...A>
		inline void ShaderResourceSet::AddResource(A && ...a)
		{
			additionalResources.emplace_back(std::forward<A>(a)...);
		}
		template<typename ...A>
		inline GeometryClass::DataLayout::DataLayout(int inpBindPoint, FORMAT first, A && ...a)
		{
			BindPoint = inpBindPoint;
			DataPoints.clear();
			DataPoints.reserve(sizeof...(a) + 1);
			DataPoints.push_back(first);
			(DataPoints.push_back(a),...);
		}
	}
}