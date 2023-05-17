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

#include "gl.h"

using namespace HIGHOMEGA;
using namespace HIGHOMEGA::GL;
using namespace HIGHOMEGA::GL::KHR_RT;
using namespace HIGHOMEGA::GL::MEMORY_MANAGER;

#define HIGHOMEGA_EXTRACT_SHADERS_FROM_BINARY 1

namespace HIGHOMEGA::GL
{
	bool InstanceClass::lowMemoryDevice = false;
	std::unordered_map <std::string, HIGHOMEGA::CacheItem<ShaderStage>> ShaderStageCache;
	std::unordered_map <int, std::list<MemChunk>> ImageMemoryMap;
	std::unordered_map <int, std::list<MemChunk>> BufferMemoryMap;
	std::unordered_map <int, std::list<MemChunk>> RTMemoryMap;
	std::mutex mem_manager_mutex;
}

std::unordered_map<std::string, Raster_PSO_DSL> HIGHOMEGA::GL::globalRaster_PSO_DSL_Cache;
std::mutex HIGHOMEGA::GL::globalRaster_PSO_DSL_Cache_mutex;
std::unordered_map<std::string, Compute_PSO_DSL> HIGHOMEGA::GL::globalCompute_PSO_DSL_Cache;
std::mutex HIGHOMEGA::GL::globalCompute_PSO_DSL_Cache_mutex;

ThreadLocalCache <BufferClass *> HIGHOMEGA::GL::BufferClass::stagingBuffers;
ThreadLocalCache <BufferClass *> HIGHOMEGA::GL::ImageClass::stagingBuffers;
ThreadLocalCache <VkCommandPool> HIGHOMEGA::GL::CommandBuffer::cmdPools;
thread_local std::vector <VkSemaphore> HIGHOMEGA::GL::waitSemaphores;
thread_local unsigned long long ThreadID = mersenneTwister64BitPRNG();

std::vector<std::pair<MemChunk *, SubAlloc>> HIGHOMEGA::GL::MEMORY_MANAGER::AllocMem(VkDevice & inpDev, VkMemoryAllocateInfo & allocInfo, VkMemoryRequirements & memReq, MEMORY_MAP_TYPE memoryMapType, bool useSparseResources)
{
	std::lock_guard <std::mutex> lk(mem_manager_mutex);
	unsigned long long storageSizeAligned = (unsigned long long)ceil((double)memReq.size / (double)memReq.alignment) * (unsigned long long)memReq.alignment;

	VkResult result;

	unsigned long long chunkMaxSize;
	std::list<MemChunk> *memChunkVecRef;
	switch(memoryMapType)
	{
	case IMAGE:
		memChunkVecRef = &ImageMemoryMap[allocInfo.memoryTypeIndex];
		chunkMaxSize = 1024 * 1024 * 1152;
		break;
	case BUFFER:
		memChunkVecRef = &BufferMemoryMap[allocInfo.memoryTypeIndex];
		chunkMaxSize = 1024 * 1024 * (useSparseResources ? 200 : 250);
		break;
	case DEV_ADDRESS:
	default:
		memChunkVecRef = &RTMemoryMap[allocInfo.memoryTypeIndex];
		chunkMaxSize = 1024 * 1024 * 250;
		break;
	}
	for (MemChunk & curChunk : *memChunkVecRef)
	{
		std::list<SubAlloc>::iterator it = curChunk.allocs.begin();
		if (curChunk.allocs.size() > 1)
		{
			while (true)
			{
				SubAlloc& curSubAlloc = *it;
				it++;
				SubAlloc& nextSubAlloc = *it;
				unsigned long long offsetAligned = (unsigned long long)ceil((double)(curSubAlloc.offset + curSubAlloc.len) / (double)memReq.alignment) * (unsigned long long)memReq.alignment;
				if (offsetAligned + storageSizeAligned <= nextSubAlloc.offset)
				{
					SubAlloc newSubAlloc;
					newSubAlloc.len = storageSizeAligned;
					newSubAlloc.offset = offsetAligned;
					curChunk.allocs.insert(it, newSubAlloc);

					return { std::make_pair(&curChunk, newSubAlloc) };
				}
				if (nextSubAlloc == curChunk.allocs.back()) break;
			}
		}

		unsigned long long usedAligned = (unsigned long long)ceil((double)(curChunk.used) / (double)memReq.alignment) * (unsigned long long)memReq.alignment;

		if (usedAligned + storageSizeAligned > chunkMaxSize) continue;

		SubAlloc newSubAlloc;
		newSubAlloc.len = storageSizeAligned;
		newSubAlloc.offset = usedAligned;
		curChunk.allocs.push_back(newSubAlloc);

		curChunk.used = usedAligned + storageSizeAligned;

		return { std::make_pair(&curChunk, newSubAlloc) };
	}
	unsigned long long storageSizeConsumed = 0u;
	std::vector<std::pair<MemChunk*, SubAlloc>> returnedSubAllocs;
	if (useSparseResources)
	{
		for (MemChunk& curChunk : *memChunkVecRef)
		{
			std::list<SubAlloc>::iterator it = curChunk.allocs.begin();
			if (curChunk.allocs.size() > 1)
			{
				while (true)
				{
					SubAlloc& curSubAlloc = *it;
					it++;
					SubAlloc& nextSubAlloc = *it;
					unsigned long long offsetAligned = (unsigned long long)ceil((double)(curSubAlloc.offset + curSubAlloc.len) / (double)memReq.alignment) * (unsigned long long)memReq.alignment;
					if (nextSubAlloc.offset > offsetAligned && storageSizeConsumed < storageSizeAligned)
					{
						SubAlloc newSubAlloc;
						newSubAlloc.len = min(nextSubAlloc.offset - offsetAligned, storageSizeAligned - storageSizeConsumed);
						newSubAlloc.offset = offsetAligned;
						storageSizeConsumed += newSubAlloc.len;
						curChunk.allocs.insert(it, newSubAlloc);

						returnedSubAllocs.push_back(std::make_pair(&curChunk, newSubAlloc));
					}
					if (nextSubAlloc == curChunk.allocs.back() || storageSizeConsumed == storageSizeAligned) break;
				}
			}

			unsigned long long usedAligned = (unsigned long long)ceil((double)(curChunk.used) / (double)memReq.alignment) * (unsigned long long)memReq.alignment;

			if (storageSizeConsumed == storageSizeAligned || chunkMaxSize == usedAligned) break;

			SubAlloc newSubAlloc;
			newSubAlloc.len = min(chunkMaxSize - usedAligned, storageSizeAligned - storageSizeConsumed);
			newSubAlloc.offset = usedAligned;
			storageSizeConsumed += newSubAlloc.len;
			curChunk.allocs.push_back(newSubAlloc);

			curChunk.used = chunkMaxSize;

			returnedSubAllocs.push_back (std::make_pair(&curChunk, newSubAlloc));
			if (storageSizeConsumed == storageSizeAligned) break;
		}
		if (storageSizeConsumed == storageSizeAligned)
			return returnedSubAllocs;
	}

	memChunkVecRef->push_back(MemChunk());
	MemChunk & addedChunkRef = memChunkVecRef->back();
	addedChunkRef.owner = memChunkVecRef;

	allocInfo.allocationSize = chunkMaxSize;
	VkMemoryAllocateFlagsInfo memFlagInfo;
	if (RTInstance::Enabled() && memoryMapType == DEV_ADDRESS)
	{
		memFlagInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO_KHR;
		memFlagInfo.pNext = VK_NULL_HANDLE;
		memFlagInfo.flags = VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT;
		memFlagInfo.deviceMask = 0u;
		allocInfo.pNext = &memFlagInfo;
	}
	result = vkAllocateMemory(inpDev, &allocInfo, nullptr, &addedChunkRef.mem);
	if (result != VK_SUCCESS) throw std::runtime_error("Could not allocate memory chunk");

	SubAlloc newSubAlloc;
	newSubAlloc.len = useSparseResources ? (storageSizeAligned - storageSizeConsumed) : storageSizeAligned;
	newSubAlloc.offset = addedChunkRef.used;
	addedChunkRef.allocs.push_back(newSubAlloc);

	addedChunkRef.used += newSubAlloc.len;

	if (useSparseResources)
	{
		returnedSubAllocs.push_back(std::make_pair(&addedChunkRef, newSubAlloc));
		return returnedSubAllocs;
	}
	else
		return { std::make_pair(&addedChunkRef, newSubAlloc) };
}

void HIGHOMEGA::GL::MEMORY_MANAGER::LogMemUsageStats()
{
	std::lock_guard <std::mutex> lk(mem_manager_mutex);
	LOG() << "Mem usage statistics: ";
	LOG() << "Images:";
	for (std::pair<const int, std::list<MemChunk>> curPair : ImageMemoryMap)
	{
		LOG() << "memoryTypeIndex: " << curPair.first;
		for (MemChunk & curChunk : curPair.second)
			LOG() << "Chunk usage: " << curChunk.used;
	}
	LOG() << "Buffers:";
	for (std::pair<const int, std::list<MemChunk>> curPair : BufferMemoryMap)
	{
		LOG() << "memoryTypeIndex: " << curPair.first;
		for (MemChunk & curChunk : curPair.second)
			LOG() << "Chunk usage: " << curChunk.used;
	}
	LOG() << "HW RT buffers:";
	for (std::pair<const int, std::list<MemChunk>> curPair : RTMemoryMap)
	{
		LOG() << "memoryTypeIndex: " << curPair.first;
		for (MemChunk & curChunk : curPair.second)
			LOG() << "Chunk usage: " << curChunk.used;
	}
}

void HIGHOMEGA::GL::MEMORY_MANAGER::FreeMem(std::vector<std::pair<MemChunk *, SubAlloc>>& pages, VkDevice & inpDev)
{
	std::lock_guard <std::mutex> lk(mem_manager_mutex);
	for (std::pair<MemChunk*, SubAlloc> & curPage : pages)
	{
		curPage.first->allocs.remove(curPage.second);
		if (curPage.first->allocs.size() == 0)
		{
			vkFreeMemory(inpDev, curPage.first->mem, nullptr);
			curPage.first->owner->remove(*curPage.first);
		}
	}
	pages.clear();
}

bool RTInstance::rtEnabled = false;
PFN_vkCreateAccelerationStructureKHR RTInstance::fpCreateAccelerationStructureKHR = VK_NULL_HANDLE;
PFN_vkDestroyAccelerationStructureKHR RTInstance::fpDestroyAccelerationStructureKHR = VK_NULL_HANDLE;
PFN_vkCmdBuildAccelerationStructuresKHR RTInstance::fpCmdBuildAccelerationStructuresKHR = VK_NULL_HANDLE;
PFN_vkCmdTraceRaysKHR RTInstance::fpCmdTraceRaysKHR = VK_NULL_HANDLE;
PFN_vkGetBufferDeviceAddressKHR RTInstance::fpGetBufferDeviceAddressKHR = VK_NULL_HANDLE;
PFN_vkCreateRayTracingPipelinesKHR RTInstance::fpCreateRayTracingPipelinesKHR = VK_NULL_HANDLE;
PFN_vkGetAccelerationStructureBuildSizesKHR RTInstance::fpGetAccelerationStructureBuildSizesKHR = VK_NULL_HANDLE;
PFN_vkGetAccelerationStructureDeviceAddressKHR RTInstance::fpGetAccelerationStructureDeviceAddressKHR = VK_NULL_HANDLE;
PFN_vkGetRayTracingShaderGroupHandlesKHR RTInstance::fpGetRayTracingShaderGroupHandlesKHR = VK_NULL_HANDLE;
VkPhysicalDeviceRayTracingPipelinePropertiesKHR RTInstance::raytracingPipelineProperties = { };

VkBool32 HIGHOMEGA::GL::DEBUG_MESSAGE(VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objType, uint64_t srcObject, size_t location,
	int32_t msgCode, const char* pLayerPrefix, const char* pMsg, void* pUserData)
{
	std::string outString = "";
	if (flags & VK_DEBUG_REPORT_ERROR_BIT_EXT) {
		outString += "ERROR: [";
		outString += pLayerPrefix;
		outString += "] Code " + std::to_string(msgCode) + " : " + pMsg;
		LOG() << outString;
	}
	else if (flags & VK_DEBUG_REPORT_WARNING_BIT_EXT) {
		outString += "WARNING: [";
		outString += pLayerPrefix;
		outString += "] Code " + std::to_string(msgCode) + " : " + pMsg;
		LOG() << outString;
	}
	else
		return false;

	fflush(stdout);
	return false;
}

VkImageType HIGHOMEGA::GL::GetVkImageTypeFromDim(TEXTURE_DIM inpDim)
{
	switch (inpDim)
	{
	case _1D:
		return VK_IMAGE_TYPE_1D;
	case _2D:
	case _2D_ARRAY:
		return VK_IMAGE_TYPE_2D;
	case _3D:
	default:
		return VK_IMAGE_TYPE_3D;
	}
}

bool HIGHOMEGA::GL::getMemoryType(InstanceClass * ptrToInstance, uint32_t typeBits, VkFlags properties, uint32_t * typeIndex)
{
	for (uint32_t i = 0; i < VK_MAX_MEMORY_TYPES; i++)
	{
		if ((typeBits & 1) == 1)
		{
			if ((ptrToInstance->deviceMemoryProperties.memoryTypes[i].propertyFlags & properties) == properties)
			{
				*typeIndex = i;
				return true;
			}
		}
		typeBits >>= 1;
	}
	return false;
}

void WindowClass::operator=(const WindowClass & b)
{
}

void WindowClass::RemovePast()
{
	if (haveRenderer) SDL_DestroyWindow(win);
	if (haveWindow) SDL_Quit();
	haveRenderer = false;
	haveWindow = false;
}

WindowClass::WindowClass()
{
	haveRenderer = false;
	haveWindow = false;
}

void WindowClass::Make(std::string appName, int startx, int starty, int w, int h, bool windowed)
{
	this->appName = appName;
	this->startx = startx;
	this->starty = starty;
	this->w = w;
	this->h = h;

	if (SDL_Init(SDL_INIT_VIDEO) != 0) { RemovePast(); throw std::runtime_error("Could not initialize SDL"); }
	haveWindow = true;

#ifdef WIN32
	SetProcessDpiAwareness(PROCESS_PER_MONITOR_DPI_AWARE);
#endif
	win = SDL_CreateWindow(appName.c_str(), startx, starty, w, h, SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_SHOWN | (windowed ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP));
	if (win == nullptr) { RemovePast();  throw std::runtime_error("Could not create window"); }
	haveRenderer = true;

	struct SDL_SysWMinfo wmInfo;
	SDL_VERSION(&wmInfo.version);

	if (SDL_GetWindowWMInfo(win, &wmInfo) == -1) { RemovePast(); throw std::runtime_error("Could not get window information"); }

	window = wmInfo.info.win.window;
	hInstance = GetModuleHandle(nullptr);
}

WindowClass::~WindowClass()
{
	RemovePast();
}

WindowClass HIGHOMEGA::GL::Window;

void InstanceClass::operator=(const InstanceClass & b)
{
}

void InstanceClass::RemovePast()
{
	acquireImageFence.RemovePast();
	renderComplete.RemovePast();

	if (haveKTXVDI) ktxVulkanDeviceInfo_Destruct(&ktxVDI);

	CreateSwapChainRemovePast();

	for (std::pair<const std::string, Raster_PSO_DSL> & curPSODSL : globalRaster_PSO_DSL_Cache)
	{
		delete curPSODSL.second.DSL;
		delete curPSODSL.second.PSO;
	}
	for (std::pair<const std::string, Compute_PSO_DSL> & curPSODSL : globalCompute_PSO_DSL_Cache)
	{
		delete curPSODSL.second.DSL;
		delete curPSODSL.second.PSO;
	}
	DestroyCommandBuffer();
	if (haveDebugCallback) DestroyDebugReportCallback(instance, msgCallback, nullptr);
	if (haveSurface) vkDestroySurfaceKHR(instance, surface, nullptr);
	if (haveDevice) vkDestroyDevice(device, nullptr);
	if (haveInstance) vkDestroyInstance(instance, nullptr);

	haveKTXVDI = false;
	haveSurface = false;
	haveDevice = false;
	haveInstance = false;
	haveDebugCallback = false;
}

void InstanceClass::CreateSwapChainRemovePast()
{
	DestroyCommandBuffer();
	frameBuffer.RemovePast();
	swapChainImages.clear();
	depthStencilImage.RemovePast();
	if (haveSwapChain) fpDestroySwapchainKHR(device, swapChain, nullptr);
	haveSwapChain = false;
}

InstanceClass::InstanceClass()
{
	haveKTXVDI = false;
	haveSurface = false;
	haveDevice = false;
	haveInstance = false;
	haveDebugCallback = false;
	haveSwapChain = false;
}

void InstanceClass::Make(bool validationLayer, WindowClass &inpWindow, bool requestHWRT, bool headless)
{
	int validationLayerCount = 1;
	const char *validationLayerNames[] =
	{
		"VK_LAYER_KHRONOS_validation"
	};

	this->validationLayer = validationLayer;

	VkApplicationInfo appInfo = {};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pApplicationName = inpWindow.appName.c_str();
	appInfo.pEngineName = inpWindow.appName.c_str();
	appInfo.apiVersion = VK_API_VERSION_1_1;

	std::vector<const char*> enabledExtensions = { VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_WIN32_SURFACE_EXTENSION_NAME, VK_KHR_GET_PHYSICAL_DEVICE_PROPERTIES_2_EXTENSION_NAME };

	VkInstanceCreateInfo instanceCreateInfo = {};
	instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	instanceCreateInfo.pNext = VK_NULL_HANDLE;
	instanceCreateInfo.pApplicationInfo = &appInfo;
	if (validationLayer)
	{
		enabledExtensions.push_back(VK_EXT_DEBUG_REPORT_EXTENSION_NAME);
	}
	instanceCreateInfo.enabledExtensionCount = (uint32_t)enabledExtensions.size();
	instanceCreateInfo.ppEnabledExtensionNames = enabledExtensions.data();
	if (validationLayer)
	{
		instanceCreateInfo.enabledLayerCount = validationLayerCount;
		instanceCreateInfo.ppEnabledLayerNames = validationLayerNames;
	}
	VkResult result = vkCreateInstance(&instanceCreateInfo, nullptr, &instance);
	if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Create instance failed"); }
	haveInstance = true;

	uint32_t gpuCount = 0;
	result = vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr);
	if (result != VK_SUCCESS || gpuCount == 0) { RemovePast(); throw std::runtime_error("Could not get physical devices"); }

	std::vector<VkPhysicalDevice> physicalDevices;
	physicalDevices = std::vector<VkPhysicalDevice>(gpuCount);
	result = vkEnumeratePhysicalDevices(instance, &gpuCount, physicalDevices.data());
	if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not enumerate physical devices"); }

	physicalDevice = physicalDevices[0];

	uint32_t submissionQueueIndex = 0;
	uint32_t queueCount;
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueCount, VK_NULL_HANDLE);
	if (queueCount == 0) { RemovePast(); throw std::runtime_error("No queues found at all"); }

	std::vector<VkQueueFamilyProperties> queueProps;
	queueProps.resize(queueCount);
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueCount, queueProps.data());

	for (submissionQueueIndex = 0; submissionQueueIndex < queueCount; submissionQueueIndex++)
		if (queueProps[submissionQueueIndex].queueFlags & VK_QUEUE_GRAPHICS_BIT && queueProps[submissionQueueIndex].queueFlags & VK_QUEUE_COMPUTE_BIT) break;

	if (submissionQueueIndex == queueCount) { RemovePast(); throw std::runtime_error("No graphics queue found"); }

	std::array<float, 1> queuePriorities = { 1.0f };
	VkDeviceQueueCreateInfo queueCreateInfo[1];
	queueCreateInfo[0] = {};
	queueCreateInfo[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queueCreateInfo[0].queueFamilyIndex = submissionQueueIndex;
	queueCreateInfo[0].queueCount = 1;
	queueCreateInfo[0].pQueuePriorities = queuePriorities.data();

	std::vector<const char*> enabledDeviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME,
														 VK_KHR_MAINTENANCE3_EXTENSION_NAME,
														 VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME,
														 VK_KHR_DESCRIPTOR_UPDATE_TEMPLATE_EXTENSION_NAME,
														 VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
														 VK_KHR_SPIRV_1_4_EXTENSION_NAME,
														 VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME,
														 VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
														 VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
														 VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME,
														 VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
														 VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
														 VK_EXT_CONDITIONAL_RENDERING_EXTENSION_NAME,
														 VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME,
														 VK_KHR_SHADER_FLOAT16_INT8_EXTENSION_NAME,
														 VK_KHR_16BIT_STORAGE_EXTENSION_NAME };

	VkPhysicalDeviceFeatures deviceFeatures = {};
	deviceFeatures.samplerAnisotropy = VK_TRUE;
	deviceFeatures.depthClamp = VK_TRUE;
	deviceFeatures.geometryShader = VK_TRUE;
	deviceFeatures.shaderStorageImageExtendedFormats = VK_TRUE;
	deviceFeatures.tessellationShader = VK_TRUE;
	deviceFeatures.vertexPipelineStoresAndAtomics = VK_TRUE;
	deviceFeatures.fragmentStoresAndAtomics = VK_TRUE;
	//deviceFeatures.sparseBinding = true;
	supportsSparseResources = false; // Sparse bindings did NOT solve anything...

	VkPhysicalDevice16BitStorageFeaturesKHR VkPhysicalDevice16BitStorageFeatures = {};
	VkPhysicalDevice16BitStorageFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES;
	VkPhysicalDevice16BitStorageFeatures.pNext = VK_NULL_HANDLE;
	VkPhysicalDevice16BitStorageFeatures.storageBuffer16BitAccess = VK_TRUE;
	VkPhysicalDevice16BitStorageFeatures.uniformAndStorageBuffer16BitAccess = VK_TRUE;

	VkPhysicalDeviceShaderFloat16Int8FeaturesKHR VkPhysicalDeviceShaderFloat16Int8Features = {};
	VkPhysicalDeviceShaderFloat16Int8Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_FLOAT16_INT8_FEATURES;
	VkPhysicalDeviceShaderFloat16Int8Features.pNext = &VkPhysicalDevice16BitStorageFeatures;
	VkPhysicalDeviceShaderFloat16Int8Features.shaderFloat16 = VK_TRUE;
	VkPhysicalDeviceShaderFloat16Int8Features.shaderInt8 = VK_FALSE;

	VkPhysicalDeviceScalarBlockLayoutFeatures physicalDeviceScalarBlockLayoutFeatures = {};
	physicalDeviceScalarBlockLayoutFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
	physicalDeviceScalarBlockLayoutFeatures.pNext = &VkPhysicalDeviceShaderFloat16Int8Features;
	physicalDeviceScalarBlockLayoutFeatures.scalarBlockLayout = VK_TRUE;

	VkPhysicalDeviceAccelerationStructureFeaturesKHR vkPhysicalDeviceAccelerationStructureFeatures;
	vkPhysicalDeviceAccelerationStructureFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ACCELERATION_STRUCTURE_FEATURES_KHR;
	vkPhysicalDeviceAccelerationStructureFeatures.pNext = &physicalDeviceScalarBlockLayoutFeatures;
	vkPhysicalDeviceAccelerationStructureFeatures.accelerationStructure = VK_TRUE;
	vkPhysicalDeviceAccelerationStructureFeatures.accelerationStructureCaptureReplay = VK_FALSE;
	vkPhysicalDeviceAccelerationStructureFeatures.accelerationStructureHostCommands = VK_FALSE;
	vkPhysicalDeviceAccelerationStructureFeatures.accelerationStructureIndirectBuild = VK_FALSE;
	vkPhysicalDeviceAccelerationStructureFeatures.descriptorBindingAccelerationStructureUpdateAfterBind = VK_TRUE;

	VkPhysicalDeviceRayTracingPipelineFeaturesKHR vkPhysicalDeviceRayTracingPipelineFeatures;
	vkPhysicalDeviceRayTracingPipelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_FEATURES_KHR;
	vkPhysicalDeviceRayTracingPipelineFeatures.pNext = &vkPhysicalDeviceAccelerationStructureFeatures;
	vkPhysicalDeviceRayTracingPipelineFeatures.rayTracingPipeline = VK_TRUE;
	vkPhysicalDeviceRayTracingPipelineFeatures.rayTracingPipelineShaderGroupHandleCaptureReplay = VK_FALSE;
	vkPhysicalDeviceRayTracingPipelineFeatures.rayTracingPipelineShaderGroupHandleCaptureReplayMixed = VK_FALSE;
	vkPhysicalDeviceRayTracingPipelineFeatures.rayTracingPipelineTraceRaysIndirect = VK_FALSE;
	vkPhysicalDeviceRayTracingPipelineFeatures.rayTraversalPrimitiveCulling = VK_FALSE;

	VkPhysicalDeviceBufferDeviceAddressFeatures vkPhysicalDeviceBufferAddressFeatures;
	vkPhysicalDeviceBufferAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
	vkPhysicalDeviceBufferAddressFeatures.pNext = &vkPhysicalDeviceRayTracingPipelineFeatures;
	vkPhysicalDeviceBufferAddressFeatures.bufferDeviceAddress = VK_TRUE;
	vkPhysicalDeviceBufferAddressFeatures.bufferDeviceAddressCaptureReplay = VK_FALSE;
	vkPhysicalDeviceBufferAddressFeatures.bufferDeviceAddressMultiDevice = VK_FALSE;

	VkPhysicalDeviceDescriptorIndexingFeaturesEXT deviceDescriptorIndexingFeature = {};
	deviceDescriptorIndexingFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES_EXT;
	deviceDescriptorIndexingFeature.pNext = &vkPhysicalDeviceBufferAddressFeatures;
	deviceDescriptorIndexingFeature.descriptorBindingVariableDescriptorCount = VK_TRUE;
	deviceDescriptorIndexingFeature.runtimeDescriptorArray = VK_TRUE;
	deviceDescriptorIndexingFeature.shaderSampledImageArrayNonUniformIndexing = VK_TRUE;
	deviceDescriptorIndexingFeature.shaderStorageBufferArrayNonUniformIndexing = VK_TRUE;
	deviceDescriptorIndexingFeature.shaderStorageImageArrayNonUniformIndexing = VK_TRUE;

	VkDeviceCreateInfo deviceCreateInfo = {};
	deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	deviceCreateInfo.pNext = &deviceDescriptorIndexingFeature;
	deviceCreateInfo.queueCreateInfoCount = 1;
	deviceCreateInfo.pQueueCreateInfos = queueCreateInfo;
	deviceCreateInfo.pEnabledFeatures = &deviceFeatures;

	deviceCreateInfo.enabledExtensionCount = (uint32_t)enabledDeviceExtensions.size();
	deviceCreateInfo.ppEnabledExtensionNames = enabledDeviceExtensions.data();
	if (validationLayer)
	{
		deviceCreateInfo.enabledLayerCount = validationLayerCount;
		deviceCreateInfo.ppEnabledLayerNames = validationLayerNames;
	}

	result = vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &device);
	if (result != VK_SUCCESS)
	{
		supportsHWRT = false;
		LOG() << "HW RT not supported. Forcing voxel-based technique.";
		enabledDeviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME,
									VK_KHR_MAINTENANCE3_EXTENSION_NAME,
									VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME,
									VK_KHR_DESCRIPTOR_UPDATE_TEMPLATE_EXTENSION_NAME,
									VK_EXT_CONDITIONAL_RENDERING_EXTENSION_NAME,
									VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME,
									VK_KHR_16BIT_STORAGE_EXTENSION_NAME };

		VkPhysicalDevice16BitStorageFeaturesKHR VkPhysicalDevice16BitStorageFeatures = {};
		VkPhysicalDevice16BitStorageFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES;
		VkPhysicalDevice16BitStorageFeatures.pNext = VK_NULL_HANDLE;
		VkPhysicalDevice16BitStorageFeatures.storageBuffer16BitAccess = VK_TRUE;
		VkPhysicalDevice16BitStorageFeatures.uniformAndStorageBuffer16BitAccess = VK_TRUE;

		VkPhysicalDeviceScalarBlockLayoutFeatures physicalDeviceScalarBlockLayoutFeatures;
		physicalDeviceScalarBlockLayoutFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
		physicalDeviceScalarBlockLayoutFeatures.pNext = &VkPhysicalDevice16BitStorageFeatures;
		physicalDeviceScalarBlockLayoutFeatures.scalarBlockLayout = VK_TRUE;

		VkPhysicalDeviceDescriptorIndexingFeaturesEXT deviceDescriptorIndexingFeature = {};
		deviceDescriptorIndexingFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES_EXT;
		deviceDescriptorIndexingFeature.pNext = &physicalDeviceScalarBlockLayoutFeatures;
		deviceDescriptorIndexingFeature.descriptorBindingVariableDescriptorCount = VK_TRUE;
		deviceDescriptorIndexingFeature.runtimeDescriptorArray = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderSampledImageArrayNonUniformIndexing = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderStorageBufferArrayNonUniformIndexing = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderStorageImageArrayNonUniformIndexing = VK_TRUE;

		deviceCreateInfo.enabledExtensionCount = (uint32_t)enabledDeviceExtensions.size();
		deviceCreateInfo.ppEnabledExtensionNames = enabledDeviceExtensions.data();
		deviceCreateInfo.pNext = &deviceDescriptorIndexingFeature;
		result = vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &device);
		if (result != VK_SUCCESS)
		{
			RemovePast();
			throw std::runtime_error("Could not create Vulkan device");
		}
	}
	else
	{
		supportsHWRT = true;
		if (requestHWRT)
			RTInstance::Enable(*this);
	}
	haveDevice = true;

	vkGetPhysicalDeviceProperties(physicalDevice, &deviceProps);

	vramAmount = 0;
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &deviceMemoryProperties);
	for (int i = 0; i != (int)deviceMemoryProperties.memoryHeapCount; i++)
	{
		if (!(deviceMemoryProperties.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT)) continue;
		vramAmount += (unsigned long long)deviceMemoryProperties.memoryHeaps[i].size;
	}
	double gbInDouble = (double)vramAmount / (1024.0 * 1024.0 * 1024.0);
	std::stringstream gbOut;
	gbOut << std::setprecision(2) << std::fixed << gbInDouble;
	LOG() << "Reported VRAM size: " << gbOut.str() << "GB";
	if (gbInDouble < 3.0)
		lowMemoryDevice = true;

	vkGetDeviceQueue(device, submissionQueueIndex, 0, &submissionQueue);

	bool foundFormat = false;

	std::vector<VkFormat> depthFormats = {
		VK_FORMAT_D32_SFLOAT_S8_UINT,
		VK_FORMAT_D32_SFLOAT,
		VK_FORMAT_D24_UNORM_S8_UINT,
		VK_FORMAT_D16_UNORM_S8_UINT,
		VK_FORMAT_D16_UNORM
	};

	for (VkFormat format : depthFormats)
	{
		VkFormatProperties formatProps;
		vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &formatProps);
		if (formatProps.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
		{
			selectedDepthFormat = format;
			foundFormat = true;
			break;
		}
	}

	if (foundFormat == false) { RemovePast(); throw std::runtime_error("No supported format found"); }

	fpGetPhysicalDeviceSurfaceSupportKHR = (PFN_vkGetPhysicalDeviceSurfaceSupportKHR)vkGetInstanceProcAddr(instance, "vkGetPhysicalDeviceSurfaceSupportKHR");
	fpGetPhysicalDeviceSurfaceCapabilitiesKHR = (PFN_vkGetPhysicalDeviceSurfaceCapabilitiesKHR)vkGetInstanceProcAddr(instance, "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");
	fpGetPhysicalDeviceSurfaceFormatsKHR = (PFN_vkGetPhysicalDeviceSurfaceFormatsKHR)vkGetInstanceProcAddr(instance, "vkGetPhysicalDeviceSurfaceFormatsKHR");
	fpGetPhysicalDeviceSurfacePresentModesKHR = (PFN_vkGetPhysicalDeviceSurfacePresentModesKHR)vkGetInstanceProcAddr(instance, "vkGetPhysicalDeviceSurfacePresentModesKHR");

	fpCreateSwapchainKHR = (PFN_vkCreateSwapchainKHR)vkGetDeviceProcAddr(device, "vkCreateSwapchainKHR");
	fpDestroySwapchainKHR = (PFN_vkDestroySwapchainKHR)vkGetDeviceProcAddr(device, "vkDestroySwapchainKHR");
	fpGetSwapchainImagesKHR = (PFN_vkGetSwapchainImagesKHR)vkGetDeviceProcAddr(device, "vkGetSwapchainImagesKHR");
	fpAcquireNextImageKHR = (PFN_vkAcquireNextImageKHR)vkGetDeviceProcAddr(device, "vkAcquireNextImageKHR");
	fpQueuePresentKHR = (PFN_vkQueuePresentKHR)vkGetDeviceProcAddr(device, "vkQueuePresentKHR");

	fpCreateDescriptorUpdateTemplateKHR = (PFN_vkCreateDescriptorUpdateTemplateKHR)vkGetDeviceProcAddr(device, "vkCreateDescriptorUpdateTemplateKHR");
	fpUpdateDescriptorSetWithTemplateKHR = (PFN_vkUpdateDescriptorSetWithTemplateKHR)vkGetDeviceProcAddr(device, "vkUpdateDescriptorSetWithTemplateKHR");
	fpDestroyDescriptorUpdateTemplateKHR = (PFN_vkDestroyDescriptorUpdateTemplateKHR)vkGetDeviceProcAddr(device, "vkDestroyDescriptorUpdateTemplateKHR");

	fpCmdBeginConditionalRenderingEXT = (PFN_vkCmdBeginConditionalRenderingEXT)vkGetDeviceProcAddr(device, "vkCmdBeginConditionalRenderingEXT");
	fpCmdEndConditionalRenderingEXT = (PFN_vkCmdEndConditionalRenderingEXT)vkGetDeviceProcAddr(device, "vkCmdEndConditionalRenderingEXT");

	if (fpGetPhysicalDeviceSurfaceSupportKHR == VK_NULL_HANDLE ||
		fpGetPhysicalDeviceSurfaceCapabilitiesKHR == VK_NULL_HANDLE ||
		fpGetPhysicalDeviceSurfaceFormatsKHR == VK_NULL_HANDLE ||
		fpGetPhysicalDeviceSurfacePresentModesKHR == VK_NULL_HANDLE ||
		fpCreateSwapchainKHR == VK_NULL_HANDLE ||
		fpDestroySwapchainKHR == VK_NULL_HANDLE ||
		fpGetSwapchainImagesKHR == VK_NULL_HANDLE ||
		fpAcquireNextImageKHR == VK_NULL_HANDLE ||
		fpQueuePresentKHR == VK_NULL_HANDLE ||
		fpCreateDescriptorUpdateTemplateKHR == VK_NULL_HANDLE ||
		fpUpdateDescriptorSetWithTemplateKHR == VK_NULL_HANDLE ||
		fpDestroyDescriptorUpdateTemplateKHR == VK_NULL_HANDLE ||
		fpCmdBeginConditionalRenderingEXT == VK_NULL_HANDLE ||
		fpCmdEndConditionalRenderingEXT == VK_NULL_HANDLE)
	{
		RemovePast();
		throw std::runtime_error("Could not get function pointers");
	}

	VkSemaphoreCreateInfo semaphoreCreateInfo = {};
	semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	semaphoreCreateInfo.pNext = VK_NULL_HANDLE;
	semaphoreCreateInfo.flags = 0;
	try { acquireImageFence.Fence(this); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create display image acquisition fence"); }
	try { renderComplete.Semaphore(this); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create semaphore renderComplete"); }

	if (!headless)
	{
		VkWin32SurfaceCreateInfoKHR surfaceCreateInfo = {};
		surfaceCreateInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
		surfaceCreateInfo.hinstance = inpWindow.hInstance;
		surfaceCreateInfo.hwnd = inpWindow.window;
		result = vkCreateWin32SurfaceKHR(instance, &surfaceCreateInfo, nullptr, &surface);
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create surface"); }
		haveSurface = true;
	}

	std::vector<VkBool32> supportsPresent;

	if (!headless)
	{
		supportsPresent = std::vector<VkBool32>(queueCount);
		for (uint32_t i = 0; i < queueCount; i++)
			fpGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, i, surface, &supportsPresent[i]);
	}

	for (uint32_t i = 0; i < queueCount; i++)
	{
		if ((queueProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0)
		{
			if (graphicsQueueNodeIndex == UINT32_MAX)
				graphicsQueueNodeIndex = i;

			if (!headless && supportsPresent[i] == VK_TRUE)
			{
				graphicsQueueNodeIndex = i;
				presentQueueNodeIndex = i;
				break;
			}
		}
	}
	for (uint32_t i = 0; i < queueCount; i++)
	{
		if ((queueProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0)
		{
			if (computeQueueNodeIndex == UINT32_MAX)
				computeQueueNodeIndex = i;
		}
	}
	for (uint32_t i = 0; i < queueCount; i++)
	{
		if ((queueProps[i].queueFlags & VK_QUEUE_TRANSFER_BIT) != 0)
		{
			if (transferQueueNodeIndex == UINT32_MAX)
				transferQueueNodeIndex = i;
		}
	}
	if (!headless && presentQueueNodeIndex == UINT32_MAX)
	{
		for (uint32_t i = 0; i < queueCount; ++i)
		{
			if (supportsPresent[i] == VK_TRUE)
			{
				presentQueueNodeIndex = i;
				break;
			}
		}
	}

	if (!headless)
	{
		if (graphicsQueueNodeIndex == UINT32_MAX || computeQueueNodeIndex == UINT32_MAX || presentQueueNodeIndex == UINT32_MAX || transferQueueNodeIndex == UINT32_MAX)
		{
			RemovePast();
			throw std::runtime_error("A graphics, present or compute queue not found.");
		}

		if (graphicsQueueNodeIndex != presentQueueNodeIndex)
		{
			RemovePast();
			throw std::runtime_error("Separate graphics and present queues not supported");
		}
	}
	else
	{
		if (graphicsQueueNodeIndex == UINT32_MAX || computeQueueNodeIndex == UINT32_MAX || transferQueueNodeIndex == UINT32_MAX)
		{
			RemovePast();
			throw std::runtime_error("A graphics or compute queue not found.");
		}
	}

	if (!headless)
	{
		uint32_t formatCount;
		result = fpGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, VK_NULL_HANDLE);
		if (result != VK_SUCCESS || formatCount == 0) { RemovePast(); throw std::runtime_error("Could not get any surface formats"); }

		std::vector<VkSurfaceFormatKHR> surfaceFormats;
		surfaceFormats = std::vector<VkSurfaceFormatKHR>(formatCount);
		result = fpGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, surfaceFormats.data());
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not retrieve details for surface formats"); }

		if (formatCount == 1 && surfaceFormats[0].format == VK_FORMAT_UNDEFINED)
			colorFormat = VK_FORMAT_B8G8R8A8_UNORM;
		else
			colorFormat = surfaceFormats[0].format;
		colorSpace = surfaceFormats[0].colorSpace;
	}

	if (validationLayer)
	{
		CreateDebugReportCallback = (PFN_vkCreateDebugReportCallbackEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugReportCallbackEXT");
		DestroyDebugReportCallback = (PFN_vkDestroyDebugReportCallbackEXT)vkGetInstanceProcAddr(instance, "vkDestroyDebugReportCallbackEXT");
		dbgBreakCallback = (PFN_vkDebugReportMessageEXT)vkGetInstanceProcAddr(instance, "vkDebugReportMessageEXT");

		VkDebugReportCallbackCreateInfoEXT dbgCreateInfo = {};
		dbgCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_REPORT_CREATE_INFO_EXT;
		dbgCreateInfo.pfnCallback = (PFN_vkDebugReportCallbackEXT)DEBUG_MESSAGE;
		dbgCreateInfo.flags = VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT;

		result = CreateDebugReportCallback(instance, &dbgCreateInfo, nullptr, &msgCallback);
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not register debug callback"); }
		haveDebugCallback = true;
	}

	try { CreateCommandPool(*this, true); }
	catch (...) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not create setup command buffer"); }

	VkCommandPool cmdPool;
	{std::unique_lock<std::mutex> lk(cmdPools.mtx); cmdPool = cmdPools.dir[ThreadID].elem; }

	KTX_error_code ktxResult = ktxVulkanDeviceInfo_Construct(&ktxVDI, physicalDevice, device, submissionQueue, cmdPool, nullptr);
	if (ktxResult != KTX_SUCCESS) { RemovePast(); throw std::runtime_error("Could not supply vulkan device info to libKTX"); }

	haveKTXVDI = true;
}

void InstanceClass::CreateSwapChain(WindowClass &windowRef)
{
	try { BeginCommandBuffer(*this); }
	catch (...) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not begin setup command buffer"); }

	VkResult result;
	VkSwapchainKHR oldSwapchain = swapChain;

	VkSurfaceCapabilitiesKHR surfCaps;
	result = fpGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &surfCaps);
	if (result != VK_SUCCESS) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not get surface capabilities"); }

	uint32_t presentModeCount;
	result = fpGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, VK_NULL_HANDLE);
	if (result != VK_SUCCESS || presentModeCount == 0) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not get present mode count"); }

	std::vector<VkPresentModeKHR> presentModes;
	presentModes = std::vector<VkPresentModeKHR>(presentModeCount);

	result = fpGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, presentModes.data());
	if (result != VK_SUCCESS) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not get present modes"); }

	VkExtent2D swapchainExtent = {};
	if (surfCaps.currentExtent.width == -1)
	{
		swapchainExtent.width = windowRef.w;
		swapchainExtent.height = windowRef.h;
	}
	else
	{
		swapchainExtent = surfCaps.currentExtent;
		windowRef.w = surfCaps.currentExtent.width;
		windowRef.h = surfCaps.currentExtent.height;
	}

	VkPresentModeKHR swapchainPresentMode = VK_PRESENT_MODE_FIFO_KHR;
	for (size_t i = 0; i < presentModeCount; i++)
	{
		if (presentModes[i] == VK_PRESENT_MODE_MAILBOX_KHR)
		{
			swapchainPresentMode = VK_PRESENT_MODE_MAILBOX_KHR;
			break;
		}
		if ((swapchainPresentMode != VK_PRESENT_MODE_MAILBOX_KHR) && (presentModes[i] == VK_PRESENT_MODE_IMMEDIATE_KHR))
			swapchainPresentMode = VK_PRESENT_MODE_IMMEDIATE_KHR;
	}

	uint32_t desiredNumberOfSwapchainImages = surfCaps.minImageCount + 1;
	if ((surfCaps.maxImageCount > 0) && (desiredNumberOfSwapchainImages > surfCaps.maxImageCount))
		desiredNumberOfSwapchainImages = surfCaps.maxImageCount;

	VkSurfaceTransformFlagsKHR preTransform;
	if (surfCaps.supportedTransforms & VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR)
		preTransform = VK_SURFACE_TRANSFORM_IDENTITY_BIT_KHR;
	else
		preTransform = surfCaps.currentTransform;

	VkSwapchainCreateInfoKHR swapchainCI = {};
	swapchainCI.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	swapchainCI.pNext = VK_NULL_HANDLE;
	swapchainCI.surface = surface;
	swapchainCI.minImageCount = desiredNumberOfSwapchainImages;
	swapchainCI.imageFormat = colorFormat;
	swapchainCI.imageColorSpace = colorSpace;
	swapchainCI.imageExtent = { swapchainExtent.width, swapchainExtent.height };
	swapchainCI.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	swapchainCI.preTransform = (VkSurfaceTransformFlagBitsKHR)preTransform;
	swapchainCI.imageArrayLayers = 1;
	swapchainCI.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
	swapchainCI.queueFamilyIndexCount = 0;
	swapchainCI.pQueueFamilyIndices = VK_NULL_HANDLE;
	swapchainCI.presentMode = swapchainPresentMode;
	swapchainCI.oldSwapchain = oldSwapchain;
	swapchainCI.clipped = true;
	swapchainCI.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;

	result = fpCreateSwapchainKHR(device, &swapchainCI, nullptr, &swapChain);
	if (result != VK_SUCCESS) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not create swapchain"); }
	haveSwapChain = true;

	if (oldSwapchain != VK_NULL_HANDLE)
	{
		frameBuffer.RemovePast();
		swapChainImages.clear();
		depthStencilImage.RemovePast();
		fpDestroySwapchainKHR(device, oldSwapchain, nullptr);
	}

	try { swapChainImages = ImageClass::FromSwapChain(*this); }
	catch (...) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not get swap chain images"); }

	try { depthStencilImage.CreateOnScreenDepthStencil(*this, windowRef.w, windowRef.h); }
	catch (...) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not create depth/stencil attachment"); }

	try {
		for (ImageClass & curImg : swapChainImages)
			frameBuffer.AddColorAttachment(curImg);
		frameBuffer.SetDepthStencil(depthStencilImage);
		frameBuffer.Create(ON_SCREEN, *this, windowRef);
	}
	catch (...) { CreateSwapChainRemovePast(); throw std::runtime_error("Could not create frame buffer"); }

	try { EndCommandBuffer(); }
	catch (...) { CreateSwapChainRemovePast();  throw std::runtime_error("Could not end setup command buffer"); }

	try { SubmitCommandBuffer(); }
	catch (...) { CreateSwapChainRemovePast();  throw std::runtime_error("Could not submit setup command buffer"); }
}

bool HIGHOMEGA::GL::InstanceClass::isNVidia()
{
	return deviceProps.vendorID == 0x10DE;
}

FramebufferClass & HIGHOMEGA::GL::InstanceClass::swapChainFrameBuffer()
{
	return frameBuffer;
}

bool HIGHOMEGA::GL::InstanceClass::SupportsHWRT()
{
	return supportsHWRT;
}

bool HIGHOMEGA::GL::InstanceClass::LowMemoryDevice()
{
	return lowMemoryDevice;
}

InstanceClass::~InstanceClass()
{
	RemovePast();
}

bool HIGHOMEGA::GL::InstanceClass::SupportsSparseResources()
{
	return supportsSparseResources;
}

InstanceClass HIGHOMEGA::GL::Instance;

void HIGHOMEGA::GL::KHR_RT::RTInstance::Enable(InstanceClass & inpInstance)
{
	fpCreateAccelerationStructureKHR = reinterpret_cast<PFN_vkCreateAccelerationStructureKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkCreateAccelerationStructureKHR"));
	fpDestroyAccelerationStructureKHR = reinterpret_cast<PFN_vkDestroyAccelerationStructureKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkDestroyAccelerationStructureKHR"));
	fpCmdBuildAccelerationStructuresKHR = reinterpret_cast<PFN_vkCmdBuildAccelerationStructuresKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkCmdBuildAccelerationStructuresKHR"));
	fpCmdTraceRaysKHR = reinterpret_cast<PFN_vkCmdTraceRaysKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkCmdTraceRaysKHR"));
	fpGetBufferDeviceAddressKHR = reinterpret_cast<PFN_vkGetBufferDeviceAddressKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkGetBufferDeviceAddressKHR"));
	fpCreateRayTracingPipelinesKHR = reinterpret_cast<PFN_vkCreateRayTracingPipelinesKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkCreateRayTracingPipelinesKHR"));
	fpGetAccelerationStructureBuildSizesKHR = reinterpret_cast<PFN_vkGetAccelerationStructureBuildSizesKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkGetAccelerationStructureBuildSizesKHR"));
	fpGetAccelerationStructureDeviceAddressKHR = reinterpret_cast<PFN_vkGetAccelerationStructureDeviceAddressKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkGetAccelerationStructureDeviceAddressKHR"));
	fpGetRayTracingShaderGroupHandlesKHR = reinterpret_cast<PFN_vkGetRayTracingShaderGroupHandlesKHR>(vkGetDeviceProcAddr(inpInstance.device, "vkGetRayTracingShaderGroupHandlesKHR"));

	raytracingPipelineProperties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_TRACING_PIPELINE_PROPERTIES_KHR;
	raytracingPipelineProperties.pNext = nullptr;
	raytracingPipelineProperties.maxRayRecursionDepth = 8;
	raytracingPipelineProperties.shaderGroupHandleSize = 0;
	VkPhysicalDeviceProperties2 props;
	props.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
	props.pNext = &raytracingPipelineProperties;
	props.properties = { };
	vkGetPhysicalDeviceProperties2(inpInstance.physicalDevice, &props);

	rtEnabled = true;
}

bool HIGHOMEGA::GL::KHR_RT::RTInstance::Enabled()
{
	return rtEnabled;
}

void HIGHOMEGA::GL::ShaderResourceSet::StringToCString(char ** destCString, std::string & inpString)
{
	*destCString = nullptr;
	*destCString = new char[strlen(inpString.c_str()) + 1];
	strcpy_s(*destCString, strlen(inpString.c_str()) + 1, inpString.c_str());
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, ImageClass & inpImageRef)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = &inpImageRef;
	imageViewRef = nullptr;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource.clear();
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, BufferClass & inpUniformRef)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	uniformRef = &inpUniformRef;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource.clear();
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, FramebufferClass & inpFrameBuf, int attachmentNumber)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = &inpFrameBuf.GetSampler();
	imageViewRef = inpFrameBuf.colorAttachments[attachmentNumber];
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource.clear();
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, KHR_RT::RTScene & inpSceneRef)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	uniformRef = nullptr;
	rtSceneRef = &inpSceneRef;
	isVariableCount = false;
	arrayedResource.clear();
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, std::vector<ShaderResource>& inpVariableCountResource)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = 0;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = true;
	arrayedResource = inpVariableCountResource;
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, std::vector<ShaderResource>& inpVariableCountResource)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource = inpVariableCountResource;
}

unsigned int HIGHOMEGA::GL::ShaderResource::ResourceCount()
{
	return (unsigned int)arrayedResource.size() > 0 ? (unsigned int)arrayedResource.size() : 1;
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(std::string inpComp, std::string inpCompEnt)
{
	compute_shader = inpComp;
	compute_entry = inpCompEnt;

	StringToCString(&compute_entry_cstr, inpCompEnt);
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(std::string inpVert, std::string inpVertEnt, std::string inpFrag, std::string inpFragEnt)
{
	vertex_shader = inpVert;
	vertex_entry = inpVertEnt;
	fragment_shader = inpFrag;
	fragment_entry = inpFragEnt;

	StringToCString(&vertex_entry_cstr, vertex_entry);
	StringToCString(&fragment_entry_cstr, fragment_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(std::string inpVert, std::string inpVertEnt, std::string inpGeom, std::string inpGeomEnt, std::string inpFrag, std::string inpFragEnt)
{
	Create(inpVert, inpVertEnt, inpFrag, inpFragEnt);

	geom_shader = inpGeom;
	geom_entry = inpGeomEnt;

	StringToCString(&geom_entry_cstr, geom_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(std::string inpVert, std::string inpVertEnt, std::string inpTessCtrl, std::string inpTessCtrlEnt, std::string inpTessEval, std::string inpTessEvalEnt, std::string inpFrag, std::string inpFragEnt)
{
	Create(inpVert, inpVertEnt, inpFrag, inpFragEnt);

	tc_shader = inpTessCtrl;
	tc_entry = inpTessCtrlEnt;
	te_shader = inpTessEval;
	te_entry = inpTessEvalEnt;

	StringToCString(&tc_entry_cstr, tc_entry);
	StringToCString(&te_entry_cstr, te_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::CreateRT(std::string inpRaygen, std::string inpRaygenEnt, std::string inpRaychit, std::string inpRaychitEnt, std::string inpRaymiss, std::string inpRaymissEnt)
{
	rt_raygen_shader = inpRaygen;
	rt_raygen_entry = inpRaygenEnt;
	rt_raychit_shader = inpRaychit;
	rt_raychit_entry = inpRaychitEnt;
	rt_raymiss_shader = inpRaymiss;
	rt_raymiss_entry = inpRaymissEnt;

	StringToCString(&rt_raygen_entry_cstr, rt_raygen_entry);
	StringToCString(&rt_raychit_entry_cstr, rt_raychit_entry);
	StringToCString(&rt_raymiss_entry_cstr, rt_raymiss_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::CreateRT(std::string inpRaygen, std::string inpRaygenEnt, std::string inpRaychit, std::string inpRaychitEnt, std::string inpRaymiss, std::string inpRaymissEnt, std::string inpRayahit, std::string inpRayahitEnt)
{
	rt_raygen_shader = inpRaygen;
	rt_raygen_entry = inpRaygenEnt;
	rt_raychit_shader = inpRaychit;
	rt_raychit_entry = inpRaychitEnt;
	rt_raymiss_shader = inpRaymiss;
	rt_raymiss_entry = inpRaymissEnt;
	rt_rayahit_shader = inpRayahit;
	rt_rayahit_entry = inpRayahitEnt;

	StringToCString(&rt_raygen_entry_cstr, rt_raygen_entry);
	StringToCString(&rt_raychit_entry_cstr, rt_raychit_entry);
	StringToCString(&rt_raymiss_entry_cstr, rt_raymiss_entry);
	StringToCString(&rt_rayahit_entry_cstr, rt_rayahit_entry);
}

HIGHOMEGA::GL::ShaderResourceSet::ShaderResourceSet()
{
	vertex_entry_cstr = nullptr;
	tc_entry_cstr = nullptr;
	te_entry_cstr = nullptr;
	geom_entry_cstr = nullptr;
	fragment_entry_cstr = nullptr;
	compute_entry_cstr = nullptr;
	rt_raygen_entry_cstr = nullptr;
	rt_raychit_entry_cstr = nullptr;
	rt_raymiss_entry_cstr = nullptr;
	rt_rayahit_entry_cstr = nullptr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getCompEntry()
{
	return compute_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getVertEntry()
{
	return vertex_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getTCEntry()
{
	return tc_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getTEEntry()
{
	return te_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getGeomEntry()
{
	return geom_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getFragEntry()
{
	return fragment_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getRaygenEntry()
{
	return rt_raygen_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getRaychitEntry()
{
	return rt_raychit_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getRaymissEntry()
{
	return rt_raymiss_entry_cstr;
}

char * HIGHOMEGA::GL::ShaderResourceSet::getRayahitEntry()
{
	return rt_rayahit_entry_cstr;
}

void HIGHOMEGA::GL::ShaderResourceSet::ClearResources()
{
	additionalResources.clear();
}

void HIGHOMEGA::GL::ShaderResourceSet::RemovePast()
{
	if (vertex_entry_cstr) delete vertex_entry_cstr;
	if (tc_entry_cstr) delete tc_entry_cstr;
	if (te_entry_cstr) delete te_entry_cstr;
	if (geom_entry_cstr) delete geom_entry_cstr;
	if (fragment_entry_cstr) delete fragment_entry_cstr;
	if (compute_entry_cstr) delete compute_entry_cstr;
	if (rt_raygen_entry_cstr) delete rt_raygen_entry_cstr;
	if (rt_raychit_entry_cstr) delete rt_raychit_entry_cstr;
	if (rt_raymiss_entry_cstr) delete rt_raymiss_entry_cstr;
	if (rt_rayahit_entry_cstr) delete rt_rayahit_entry_cstr;

	vertex_entry_cstr = nullptr;
	tc_entry_cstr = nullptr;
	te_entry_cstr = nullptr;
	geom_entry_cstr = nullptr;
	fragment_entry_cstr = nullptr;
	compute_entry_cstr = nullptr;
	rt_raygen_entry_cstr = nullptr;
	rt_raychit_entry_cstr = nullptr;
	rt_raymiss_entry_cstr = nullptr;
	rt_rayahit_entry_cstr = nullptr;

	ClearResources();
}

std::vector<ShaderResource> & HIGHOMEGA::GL::ShaderResourceSet::getAdditionalResources()
{
	return additionalResources;
}

HIGHOMEGA::GL::ShaderResourceSet::~ShaderResourceSet()
{
	RemovePast();
}

std::mutex shader_stage_mutex;
void HIGHOMEGA::GL::RasterPipelineStateClass::ErasePipelineState()
{
	if (!ptrToInstance) return;

	{std::lock_guard<std::mutex> lk(shader_stage_mutex);
	for (std::string & curStage : usedShaderStages)
	{
		ShaderStageCache[curStage].elemCount--;
		if (ShaderStageCache[curStage].elemCount == 0)
			ShaderStageCache.erase(curStage);
	}}

	if (haveGraphicsPipeline) vkDestroyPipeline(ptrToInstance->device, pipeline, nullptr);
	if (havePipelineLayout) vkDestroyPipelineLayout(ptrToInstance->device, pipelineLayout, nullptr);
	if (havePipelineCache) vkDestroyPipelineCache(ptrToInstance->device, pipelineCache, nullptr);

	havePipelineLayout = false;
	haveGraphicsPipeline = false;
	havePipelineCache = false;
}

HIGHOMEGA::GL::RasterPipelineStateClass::RasterPipelineStateClass()
{
	havePipelineLayout = false;
	haveGraphicsPipeline = false;
	havePipelineCache = false;
}

HIGHOMEGA::GL::RasterPipelineStateClass::RasterPipelineStateClass(InstanceClass & renderInst, DescriptorSetLayout & DescSetLayout, PipelineFlags & inpFlags, FramebufferClass & frameBuffer, GeometryClass & geomRef, ShaderResourceSet & inpShader)
{
	ptrToInstance = &renderInst;

	VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo = {};
	pPipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pPipelineLayoutCreateInfo.pNext = VK_NULL_HANDLE;
	pPipelineLayoutCreateInfo.setLayoutCount = (uint32_t)DescSetLayout.descriptorSetLayouts.size();
	pPipelineLayoutCreateInfo.pSetLayouts = DescSetLayout.descriptorSetLayouts.data();

	VkResult result = vkCreatePipelineLayout(ptrToInstance->device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayout);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create pipeline layout"); }
	havePipelineLayout = true;

	VkGraphicsPipelineCreateInfo pipelineCreateInfo = {};
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	pipelineCreateInfo.layout = pipelineLayout;

	VkPipelineRasterizationStateCreateInfo rasterizationState = {};
	rasterizationState.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterizationState.polygonMode = VK_POLYGON_MODE_FILL;
	rasterizationState.cullMode = inpFlags.backFaceCulling ? VK_CULL_MODE_BACK_BIT : (inpFlags.frontFaceCulling ? VK_CULL_MODE_FRONT_BIT : VK_CULL_MODE_NONE);
	rasterizationState.frontFace = inpFlags.frontFaceClockWise ? VK_FRONT_FACE_CLOCKWISE : VK_FRONT_FACE_COUNTER_CLOCKWISE;
	rasterizationState.depthClampEnable = VK_TRUE;
	rasterizationState.rasterizerDiscardEnable = VK_FALSE;
	rasterizationState.depthBiasEnable = VK_FALSE;
	rasterizationState.lineWidth = 1.0f;

	VkPipelineColorBlendStateCreateInfo colorBlendState = {};
	colorBlendState.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	std::vector<VkPipelineColorBlendAttachmentState> blendAttachmentState;

	for (int i = 0; i != ((frameBuffer.mode == ON_SCREEN) ? 1 : frameBuffer.colorAttachments.size()); i++)
	{
		VkPipelineColorBlendAttachmentState curAttachment = {};

		curAttachment.colorWriteMask = 0x0;
		if (inpFlags.redMask) curAttachment.colorWriteMask |= VK_COLOR_COMPONENT_R_BIT;
		if (inpFlags.greenMask) curAttachment.colorWriteMask |= VK_COLOR_COMPONENT_G_BIT;
		if (inpFlags.blueMask) curAttachment.colorWriteMask |= VK_COLOR_COMPONENT_B_BIT;
		if (inpFlags.alphaMask) curAttachment.colorWriteMask |= VK_COLOR_COMPONENT_A_BIT;

		curAttachment.blendEnable = inpFlags.blendEnable ? VK_TRUE : VK_FALSE;
		if (inpFlags.blendEnable && inpFlags.alphaBlending)
		{
			curAttachment.alphaBlendOp = (VkBlendOp)inpFlags.alphaBlendOp;
			curAttachment.srcAlphaBlendFactor = (VkBlendFactor)inpFlags.srcAlphaFactor;
			curAttachment.dstAlphaBlendFactor = (VkBlendFactor)inpFlags.dstAlphaFactor;
		}
		if (inpFlags.blendEnable && inpFlags.colorBlending)
		{
			curAttachment.colorBlendOp = (VkBlendOp)inpFlags.colorBlendOp;
			curAttachment.srcColorBlendFactor = (VkBlendFactor)inpFlags.srcColorFactor;
			curAttachment.dstColorBlendFactor = (VkBlendFactor)inpFlags.dstColorFactor;
		}
		blendAttachmentState.push_back(curAttachment);
	}

	colorBlendState.attachmentCount = (uint32_t)blendAttachmentState.size();
	colorBlendState.pAttachments = blendAttachmentState.data();

	VkPipelineViewportStateCreateInfo viewportState = {};
	viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportState.viewportCount = 1;
	viewportState.scissorCount = 1;

	VkPipelineDynamicStateCreateInfo dynamicState = {};
	std::vector<VkDynamicState> dynamicStateEnables = { VK_DYNAMIC_STATE_VIEWPORT , VK_DYNAMIC_STATE_SCISSOR };
	dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynamicState.pDynamicStates = dynamicStateEnables.data();
	dynamicState.dynamicStateCount = (uint32_t)dynamicStateEnables.size();

	VkPipelineDepthStencilStateCreateInfo depthStencilState = {};
	depthStencilState.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthStencilState.depthTestEnable = inpFlags.depthTest ? VK_TRUE : VK_FALSE;
	depthStencilState.depthWriteEnable = inpFlags.depthWrite ? VK_TRUE : VK_FALSE;
	depthStencilState.depthCompareOp = (VkCompareOp)inpFlags.depthCompare;
	depthStencilState.depthBoundsTestEnable = VK_FALSE;
	depthStencilState.back.failOp = (VkStencilOp)inpFlags.stencilOp;
	depthStencilState.back.passOp = (VkStencilOp)inpFlags.stencilOp;
	depthStencilState.back.compareOp = (VkCompareOp)inpFlags.stencilCompare;
	depthStencilState.back.reference = inpFlags.stencilCompareValue;
	depthStencilState.back.writeMask = inpFlags.stencilWrite ? 0xFF : 0x00;
	depthStencilState.stencilTestEnable = inpFlags.stencilTest ? VK_TRUE : VK_FALSE;
	depthStencilState.front = depthStencilState.back;

	VkPipelineMultisampleStateCreateInfo multisampleState = {};
	multisampleState.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	multisampleState.pSampleMask = VK_NULL_HANDLE;
	multisampleState.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

	std::vector<VkPipelineShaderStageCreateInfo> shaderStages;
	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.vertex_shader, inpShader.getVertEntry(), VK_SHADER_STAGE_VERTEX_BIT)->elem.stage); }
	catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create vertex shader"); }

	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.fragment_shader, inpShader.getFragEntry(), VK_SHADER_STAGE_FRAGMENT_BIT)->elem.stage); }
	catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create fragment shader"); }

	usedShaderStages.push_back(inpShader.vertex_shader + std::string(":") + inpShader.getVertEntry());
	usedShaderStages.push_back(inpShader.fragment_shader + std::string(":") + inpShader.getFragEntry());

	if (inpShader.getTCEntry())
	{
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.tc_shader, inpShader.getTCEntry(), VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT)->elem.stage); }
		catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create tessellation control shader"); }
		usedShaderStages.push_back(inpShader.tc_shader + std::string(":") + inpShader.getTCEntry());
	}

	if (inpShader.getTEEntry())
	{
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.te_shader, inpShader.getTEEntry(), VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT)->elem.stage); }
		catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create tessellation evaluation shader"); }
		usedShaderStages.push_back(inpShader.te_shader + std::string(":") + inpShader.getTEEntry());
	}

	if (inpShader.getGeomEntry())
	{
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.geom_shader, inpShader.getGeomEntry(), VK_SHADER_STAGE_GEOMETRY_BIT)->elem.stage); }
		catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create geometry shader"); }
		usedShaderStages.push_back(inpShader.geom_shader + std::string(":") + inpShader.getGeomEntry());
	}

	pipelineCreateInfo.stageCount = (uint32_t)shaderStages.size();
	VkPipelineTessellationStateCreateInfo tessStateCreateInfo = {};
	VkPipelineInputAssemblyStateCreateInfo inputAssemblyState = {};
	inputAssemblyState.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	if (inpShader.getTCEntry() && inpShader.getTEEntry())
	{
		tessStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_TESSELLATION_STATE_CREATE_INFO;
		tessStateCreateInfo.patchControlPoints = 3;
		tessStateCreateInfo.flags = 0;
		tessStateCreateInfo.pNext = VK_NULL_HANDLE;
		pipelineCreateInfo.pTessellationState = &tessStateCreateInfo;

		inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_PATCH_LIST;
	}
	else
	{
		pipelineCreateInfo.pTessellationState = VK_NULL_HANDLE;

		inputAssemblyState.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	}

	pipelineCreateInfo.pStages = shaderStages.data();
	pipelineCreateInfo.pVertexInputState = &geomRef.vertBuffer.VERT_vi;
	pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
	pipelineCreateInfo.pRasterizationState = &rasterizationState;
	pipelineCreateInfo.pColorBlendState = &colorBlendState;
	pipelineCreateInfo.pMultisampleState = &multisampleState;
	pipelineCreateInfo.pViewportState = &viewportState;
	pipelineCreateInfo.pDepthStencilState = &depthStencilState;
	pipelineCreateInfo.renderPass = frameBuffer.renderPass;
	pipelineCreateInfo.pDynamicState = &dynamicState;

	VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
	pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	result = vkCreatePipelineCache(ptrToInstance->device, &pipelineCacheCreateInfo, nullptr, &pipelineCache);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create a pipeline cache"); }
	havePipelineCache = true;

	result = vkCreateGraphicsPipelines(ptrToInstance->device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create pipeline"); }
	haveGraphicsPipeline = true;
}

HIGHOMEGA::GL::RasterPipelineStateClass::~RasterPipelineStateClass()
{
	ErasePipelineState();
}

void HIGHOMEGA::GL::ComputePipelineStateClass::ErasePipelineState()
{
	if (!ptrToInstance) return;

	if (usedShaderStage != std::string(""))
	{
		{std::lock_guard<std::mutex> lk(shader_stage_mutex);
		ShaderStageCache[usedShaderStage].elemCount--;
		if (ShaderStageCache[usedShaderStage].elemCount == 0)
			ShaderStageCache.erase(usedShaderStage);
		}
		usedShaderStage = std::string("");
	}
	if (haveComputePipeline) vkDestroyPipeline(ptrToInstance->device, pipeline, nullptr);
	if (havePipelineLayout) vkDestroyPipelineLayout(ptrToInstance->device, pipelineLayout, nullptr);
	if (havePipelineCache) vkDestroyPipelineCache(ptrToInstance->device, pipelineCache, nullptr);

	havePipelineLayout = false;
	haveComputePipeline = false;
	havePipelineCache = false;
}

HIGHOMEGA::GL::ComputePipelineStateClass::ComputePipelineStateClass()
{
	havePipelineLayout = false;
	haveComputePipeline = false;
	havePipelineCache = false;
}

HIGHOMEGA::GL::ComputePipelineStateClass::ComputePipelineStateClass(InstanceClass & renderInst, DescriptorSetLayout & DescSetLayout, ShaderResourceSet & inpShader)
{
	ptrToInstance = &renderInst;

	VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo = {};
	pPipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pPipelineLayoutCreateInfo.pNext = VK_NULL_HANDLE;
	pPipelineLayoutCreateInfo.setLayoutCount = (uint32_t)DescSetLayout.descriptorSetLayouts.size();
	pPipelineLayoutCreateInfo.pSetLayouts = DescSetLayout.descriptorSetLayouts.data();

	VkResult result = vkCreatePipelineLayout(ptrToInstance->device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayout);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create pipeline layout"); }
	havePipelineLayout = true;

	VkComputePipelineCreateInfo pipelineCreateInfo = {};
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineCreateInfo.layout = pipelineLayout;

	try { pipelineCreateInfo.stage = AddOrFindCachedShaderStage(*ptrToInstance, inpShader.compute_shader, inpShader.getCompEntry(), VK_SHADER_STAGE_COMPUTE_BIT)->elem.stage; }
	catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create compute shader"); }
	usedShaderStage = inpShader.compute_shader + ":" + inpShader.getCompEntry();

	VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
	pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	result = vkCreatePipelineCache(ptrToInstance->device, &pipelineCacheCreateInfo, nullptr, &pipelineCache);
	if (result != VK_SUCCESS) { throw std::runtime_error("Could not create a pipeline cache"); }
	havePipelineCache = true;

	result = vkCreateComputePipelines(ptrToInstance->device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create pipeline"); }
	haveComputePipeline = true;
}

HIGHOMEGA::GL::ComputePipelineStateClass::~ComputePipelineStateClass()
{
	ErasePipelineState();
}

void HIGHOMEGA::GL::SemaphoreClass::RemovePast()
{
	if (haveSemaphore)
	{
		std::vector<VkSemaphore>::iterator it = std::find(waitSemaphores.begin(), waitSemaphores.end(), semaphore);
		if (it != waitSemaphores.end())
		{
			FenceClass danglingSemFence;
			danglingSemFence.Fence(ptrToInstance);
			danglingSemFence.Reset();

			VkSubmitInfo submitInfo = {};
			submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
			submitInfo.commandBufferCount = 0;
			submitInfo.pWaitSemaphores = &semaphore;
			submitInfo.waitSemaphoreCount = 1;
			submitInfo.signalSemaphoreCount = 0;
			std::vector <VkPipelineStageFlags> waitMasks;
			for (int i = 0; i != submitInfo.waitSemaphoreCount; i++)
				waitMasks.push_back(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
			submitInfo.pWaitDstStageMask = waitMasks.data();

			VkResult result;
			{std::unique_lock<std::mutex> lk(ptrToInstance->queue_mutex);
			result = vkQueueSubmit(ptrToInstance->submissionQueue, 1, &submitInfo, danglingSemFence.fence); }
			if (result != VK_SUCCESS) { throw std::runtime_error("Could not wait on dangling semaphore"); }

			danglingSemFence.Wait();

			waitSemaphores.erase(it);
		}
		vkDestroySemaphore(ptrToInstance->device, semaphore, nullptr);
	}
	haveSemaphore = false;
}

HIGHOMEGA::GL::SemaphoreClass::SemaphoreClass()
{
	haveSemaphore = false;
}

void HIGHOMEGA::GL::SemaphoreClass::Semaphore(InstanceClass * inpPtrToInstance)
{
	ptrToInstance = inpPtrToInstance;

	VkSemaphoreCreateInfo semaphoreCreateInfo = {};
	semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	VkResult result = vkCreateSemaphore(ptrToInstance->device, &semaphoreCreateInfo, nullptr, &semaphore);
	if (result != VK_SUCCESS) throw std::runtime_error("Could not create semaphore");

	haveSemaphore = true;
}

HIGHOMEGA::GL::SemaphoreClass::~SemaphoreClass()
{
	RemovePast();
}

void HIGHOMEGA::GL::FenceClass::RemovePast()
{
	if (haveFence && ptrToInstance && ptrToInstance->haveDevice) vkDestroyFence(ptrToInstance->device, fence, nullptr);
	haveFence = false;
}

HIGHOMEGA::GL::FenceClass::FenceClass()
{
	haveFence = false;
}

void HIGHOMEGA::GL::FenceClass::Fence(InstanceClass * inpPtrToInstance)
{
	ptrToInstance = inpPtrToInstance;

	VkFenceCreateInfo fenceCreateInfo = {};
	fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceCreateInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;
	if (vkCreateFence(ptrToInstance->device, &fenceCreateInfo, nullptr, &fence) != VK_SUCCESS) throw std::runtime_error("Could not create fence");

	haveFence = true;
}

void HIGHOMEGA::GL::FenceClass::Wait()
{
	if (!ptrToInstance || !haveFence) throw std::runtime_error("fence not initialized for wait");

	VkResult result = vkWaitForFences(ptrToInstance->device, 1, &fence, VK_TRUE, UINT64_MAX);
	if (result != VK_SUCCESS)
	{
		if (result == VK_ERROR_OUT_OF_HOST_MEMORY) throw std::runtime_error("Could not wait on fence: out of host memory");
		else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) throw std::runtime_error("Could not wait on fence: out of device memory");
		else throw std::runtime_error("Could not wait on fence: device lost");
	}
}

void HIGHOMEGA::GL::FenceClass::Reset()
{
	if (!ptrToInstance || !haveFence) throw std::runtime_error("fence not initialized for reset");

	if (vkResetFences(ptrToInstance->device, 1, &fence) != VK_SUCCESS) throw std::runtime_error("Could not reset fence");
}

HIGHOMEGA::GL::FenceClass::~FenceClass()
{
	RemovePast();
}

HIGHOMEGA::GL::RasterletClass::RasterletClass()
{
	ptrToInstance = nullptr;
}

HIGHOMEGA::GL::RasterletClass::~RasterletClass()
{
	initSem = false;
}

void RasterletClass::Rasterlet(InstanceClass & renderInst)
{
	ptrToInstance = &renderInst;
}

void HIGHOMEGA::GL::RasterletClass::ResetRasterlet()
{
	if (ptrToFrameBuffer->mode == OFF_SCREEN && initSem)
	{
		renderSem.RemovePast();
		renderSem.Semaphore(ptrToInstance);
	}
}

void RasterletClass::PrepareSubmission(std::vector <PSO_DSL_DS_GeomPairing> & PSO_DSL_DS_GeomPairings, DynamicPipelineFlags &inpDynamicFlags, FramebufferClass & frameBuffer)
{
	ptrToFrameBuffer = &frameBuffer;

	unsigned int numCmdBufs = (ptrToFrameBuffer->mode == ON_SCREEN) ? ((int)frameBuffer.colorAttachments.size() + 1) : 1;

	if (ptrToFrameBuffer->mode == OFF_SCREEN && !initSem)
	{
		renderSem.Semaphore(ptrToInstance);
		initSem = true;
	}

	std::vector<VkClearValue> clearValues;
	try {
		if (ptrToFrameBuffer->mode == ON_SCREEN)
		{
			VkClearValue curVal;
			curVal.color = { inpDynamicFlags.clear_color[0], inpDynamicFlags.clear_color[1], inpDynamicFlags.clear_color[2], inpDynamicFlags.clear_color[3] };
			clearValues.push_back(curVal);
		}
		else
		{
			for (int i = 0; i != frameBuffer.colorAttachments.size(); i++)
			{
				VkClearValue curVal;
				curVal.color = { inpDynamicFlags.clear_color[0], inpDynamicFlags.clear_color[1], inpDynamicFlags.clear_color[2], inpDynamicFlags.clear_color[3] };
				clearValues.push_back(curVal);
			}
		}
		VkClearValue curVal;
		curVal.depthStencil = { inpDynamicFlags.depth_clear, inpDynamicFlags.stencil_clear };
		clearValues.push_back(curVal);
	}
	catch (...) { throw std::runtime_error("Could not create swap chain command buffers"); }

	VkRenderPassBeginInfo renderPassBeginInfo = {};
	renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassBeginInfo.pNext = VK_NULL_HANDLE;
	renderPassBeginInfo.renderPass = ptrToFrameBuffer->renderPass;
	renderPassBeginInfo.renderArea.offset.x = inpDynamicFlags.viewport_x;
	renderPassBeginInfo.renderArea.offset.y = inpDynamicFlags.viewport_y;
	renderPassBeginInfo.renderArea.extent.width = inpDynamicFlags.viewport_width;
	renderPassBeginInfo.renderArea.extent.height = inpDynamicFlags.viewport_height;
	renderPassBeginInfo.clearValueCount = (uint32_t)clearValues.size();
	renderPassBeginInfo.pClearValues = clearValues.data();

	if (frameBuffer.mode == ON_SCREEN)
	{
		for (unsigned int i = 0; i < numCmdBufs - 1; ++i)
			RecordCommandBuffers(PSO_DSL_DS_GeomPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, i, frameBuffer.swapChainBuffers[i], &frameBuffer.colorAttachments[i]->image, ON_SCREEN);
	}
	else
		RecordCommandBuffers(PSO_DSL_DS_GeomPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, 0, frameBuffer.attachmentsFrameBuffer, VK_NULL_HANDLE, OFF_SCREEN);
}

void HIGHOMEGA::GL::RasterletClass::RecordCommandBuffers(std::vector<PSO_DSL_DS_GeomPairing>& PSO_DSL_DS_GeomPairings, DynamicPipelineFlags & inpDynamicFlags, VkRenderPassBeginInfo & renderPassBeginInfo, unsigned int numCmdBuf, unsigned int whichCmdBuf, VkFramebuffer & inpFrameBuffer, VkImage *inpImg, RENDER_MODE inpMode)
{
	renderPassBeginInfo.framebuffer = inpFrameBuffer;

	BeginCommandBuffer(*ptrToInstance, numCmdBuf, whichCmdBuf);

	vkCmdBeginRenderPass(cmdBuffers[whichCmdBuf], &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

	VkViewport viewport = {};
	viewport.x = (float)inpDynamicFlags.viewport_x;
	viewport.y = (float)inpDynamicFlags.viewport_y;
	viewport.height = (float)inpDynamicFlags.viewport_height;
	viewport.width = (float)inpDynamicFlags.viewport_width;
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;
	vkCmdSetViewport(cmdBuffers[whichCmdBuf], 0, 1, &viewport);

	VkRect2D scissor = {};
	scissor.extent.width = inpDynamicFlags.viewport_width;
	scissor.extent.height = inpDynamicFlags.viewport_height;
	scissor.offset.x = inpDynamicFlags.viewport_x;
	scissor.offset.y = inpDynamicFlags.viewport_y;
	vkCmdSetScissor(cmdBuffers[whichCmdBuf], 0, 1, &scissor);

	int baseInstanceNumber = 0;

	RasterPipelineStateClass *PSOPtr = nullptr;
	VkConditionalRenderingBeginInfoEXT conditionalRenderingBeginInfo{};
	if (doesCulling)
	{
		conditionalRenderingBeginInfo.sType = VK_STRUCTURE_TYPE_CONDITIONAL_RENDERING_BEGIN_INFO_EXT;
		conditionalRenderingBeginInfo.buffer = conditionalBufferPtr->buffer;
	}

	for (PSO_DSL_DS_GeomPairing & curPairing : PSO_DSL_DS_GeomPairings)
	{
		if (PSOPtr != curPairing.PSO_DSL->PSO)
		{
			vkCmdBindPipeline(cmdBuffers[whichCmdBuf], VK_PIPELINE_BIND_POINT_GRAPHICS, curPairing.PSO_DSL->PSO->pipeline);
			PSOPtr = curPairing.PSO_DSL->PSO;
		}
		vkCmdBindDescriptorSets(cmdBuffers[whichCmdBuf], VK_PIPELINE_BIND_POINT_GRAPHICS, curPairing.PSO_DSL->PSO->pipelineLayout, 0, (uint32_t)curPairing.DS->descriptorSets.size(), curPairing.DS->descriptorSets.data(), 0, VK_NULL_HANDLE);
		VkDeviceSize offsets[1] = { 0 };
		for (GeometryClass *curGeom : *curPairing.geom)
		{
			vkCmdBindVertexBuffers(cmdBuffers[whichCmdBuf], curGeom->layoutCache.BindPoint, 1, &curGeom->vertBuffer.buffer, offsets);
			if (doesCulling)
			{
				conditionalRenderingBeginInfo.offset = baseInstanceNumber * sizeof(unsigned int);
				ptrToInstance->fpCmdBeginConditionalRenderingEXT(cmdBuffers[whichCmdBuf], &conditionalRenderingBeginInfo);
			}
			vkCmdDraw(cmdBuffers[whichCmdBuf], curGeom->vertBuffer.getSize() / sizeof(RasterVertex), (uint32_t)1, 0, baseInstanceNumber);
			if (doesCulling) ptrToInstance->fpCmdEndConditionalRenderingEXT(cmdBuffers[whichCmdBuf]);
			baseInstanceNumber++;
		}
	}

	vkCmdEndRenderPass(cmdBuffers[whichCmdBuf]);

	if (inpMode == ON_SCREEN)
	{
		VkImageMemoryBarrier prePresentBarrier = {};
		prePresentBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		prePresentBarrier.pNext = VK_NULL_HANDLE;
		prePresentBarrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		prePresentBarrier.dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		prePresentBarrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		prePresentBarrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
		prePresentBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		prePresentBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		prePresentBarrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		prePresentBarrier.image = *inpImg;

		VkImageMemoryBarrier *pMemoryBarrier = &prePresentBarrier;
		vkCmdPipelineBarrier(cmdBuffers[whichCmdBuf], VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, 0, 0, nullptr, 0, nullptr, 1, &prePresentBarrier);
	}

	EndCommandBuffer(whichCmdBuf);
}

void RasterletClass::Draw()
{
	if (!ptrToInstance) return;

	VkResult result;

	if (ptrToFrameBuffer->mode == ON_SCREEN)
	{
		ptrToInstance->acquireImageFence.Reset();

		result = ptrToInstance->fpAcquireNextImageKHR(ptrToInstance->device, ptrToInstance->swapChain, UINT64_MAX, VK_NULL_HANDLE, ptrToInstance->acquireImageFence.fence, &ptrToFrameBuffer->currentSwapChainBuffer);
		if (result != VK_SUCCESS) throw std::runtime_error("Problem acq. next image");

		ptrToInstance->acquireImageFence.Wait();

		VkImageMemoryBarrier postPresentBarrier = {};
		postPresentBarrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		postPresentBarrier.pNext = VK_NULL_HANDLE;
		postPresentBarrier.srcAccessMask = 0;
		postPresentBarrier.dstAccessMask = 0;
		postPresentBarrier.oldLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
		postPresentBarrier.newLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		postPresentBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		postPresentBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		postPresentBarrier.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
		postPresentBarrier.image = ptrToFrameBuffer->colorAttachments[ptrToFrameBuffer->currentSwapChainBuffer]->image;

		BeginCommandBuffer(*ptrToInstance, CommandBufferCount(), CommandBufferCount() - 1);

		vkCmdPipelineBarrier(
			cmdBuffers[CommandBufferCount() - 1],
			VK_PIPELINE_STAGE_ALL_COMMANDS_BIT,
			VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
			0,
			0, nullptr,
			0, nullptr,
			1, &postPresentBarrier);

		EndCommandBuffer(CommandBufferCount() - 1);

		SubmitCommandBuffer(CommandBufferCount() - 1);
	}

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

	if (submissionMode == SUBMIT_SERIAL)
	{
		submitInfo.waitSemaphoreCount = (uint32_t)waitSemaphores.size();
		submitInfo.pWaitSemaphores = waitSemaphores.data();
	}
	else if (submissionMode == SUBMIT_ASYNC)
	{
		submitInfo.waitSemaphoreCount = 0;
		submitInfo.pWaitSemaphores = nullptr;
	}
	std::vector <VkPipelineStageFlags> waitMasks;
	for (int i = 0; i != submitInfo.waitSemaphoreCount; i++)
		waitMasks.push_back(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
	submitInfo.pWaitDstStageMask = waitMasks.data();
	if (ptrToFrameBuffer->mode == OFF_SCREEN)
	{
		submitInfo.signalSemaphoreCount = 1;
		submitInfo.pSignalSemaphores = &renderSem.semaphore;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &cmdBuffers[0];
	}
	else
	{
		submitInfo.signalSemaphoreCount = 1;
		submitInfo.pSignalSemaphores = &ptrToInstance->renderComplete.semaphore;
		submitInfo.commandBufferCount = 1;
		submitInfo.pCommandBuffers = &cmdBuffers[ptrToFrameBuffer->currentSwapChainBuffer];
	}

	Reset();

	{std::unique_lock<std::mutex> lk(ptrToInstance->queue_mutex);
	result = vkQueueSubmit(ptrToInstance->submissionQueue, 1, &submitInfo, submissionMode == SUBMIT_SERIAL ? fence : VK_NULL_HANDLE); }
	if (result != VK_SUCCESS) throw std::runtime_error("Problem submitting draw calls for current frame");

	if (ptrToFrameBuffer->mode == ON_SCREEN)
	{
		VkPresentInfoKHR presentInfo = {};
		presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
		presentInfo.pNext = VK_NULL_HANDLE;
		presentInfo.swapchainCount = 1;
		presentInfo.pSwapchains = &ptrToInstance->swapChain;
		presentInfo.pImageIndices = &ptrToFrameBuffer->currentSwapChainBuffer;
		presentInfo.pWaitSemaphores = &ptrToInstance->renderComplete.semaphore;
		presentInfo.waitSemaphoreCount = 1;
		{std::unique_lock<std::mutex> lk(ptrToInstance->queue_mutex);
		result = ptrToInstance->fpQueuePresentKHR(ptrToInstance->submissionQueue, &presentInfo); }
		if (result != VK_SUCCESS) throw std::runtime_error("Problem presenting frame");
	}

	if (submissionMode == SUBMIT_SERIAL)
	{
		Wait();
		waitSemaphores.clear();
	}
	if (ptrToFrameBuffer->mode == OFF_SCREEN)
		waitSemaphores.push_back(renderSem.semaphore);
}

void HIGHOMEGA::GL::RasterletClass::makeAsync()
{
	submissionMode = SUBMIT_ASYNC;
}

void HIGHOMEGA::GL::RasterletClass::makeSerial()
{
	submissionMode = SUBMIT_SERIAL;
}

HIGHOMEGA::GL::ComputeletClass::ComputeletClass()
{
	instanceRef = nullptr;
}

void HIGHOMEGA::GL::ComputeletClass::Computelet(InstanceClass & renderInst)
{
	instanceRef = &renderInst;
	computeSem.Semaphore(instanceRef);
}

void HIGHOMEGA::GL::ComputeletClass::Start(ComputePipelineStateClass & PSO)
{
	if (!instanceRef) return;

	BeginCommandBuffer(*instanceRef);

	vkCmdBindPipeline(cmdBuffers[0], VK_PIPELINE_BIND_POINT_COMPUTE, PSO.pipeline);
}

void HIGHOMEGA::GL::ComputeletClass::Dispatch(ComputePipelineStateClass & PSO, DescriptorSets & DS, int groupX, int groupY, int groupZ)
{
	if (!instanceRef) return;

	vkCmdBindDescriptorSets(cmdBuffers[0], VK_PIPELINE_BIND_POINT_COMPUTE, PSO.pipelineLayout, 0, (uint32_t)DS.descriptorSets.size(), DS.descriptorSets.data(), 0, VK_NULL_HANDLE);
	vkCmdDispatch(cmdBuffers[0], groupX, groupY, groupZ);
}

void HIGHOMEGA::GL::ComputeletClass::End()
{
	if (!instanceRef) return;

	EndCommandBuffer();
}

void HIGHOMEGA::GL::ComputeletClass::Submit()
{
	if (!instanceRef) return;

	WaitSubmitSignalCommandBuffer(submissionMode, computeSem);
}

void HIGHOMEGA::GL::ComputeletClass::makeAsync()
{
	submissionMode = SUBMIT_ASYNC;
}

void HIGHOMEGA::GL::ComputeletClass::makeSerial()
{
	submissionMode = SUBMIT_SERIAL;
}

GeometryClass::DataLayout::DataLayout()
{
}

void HIGHOMEGA::GL::GeometryClass::getMinMax(std::vector<RasterVertex>& inpVertexData)
{
	for (int i = 0; i != inpVertexData.size(); i++)
	{
		vec3 edge, vnorm, vcol;
		vec2 uv;
		unpackRasterVertex(edge, vcol, uv, vnorm, inpVertexData[i]);
		if (i == 0)
		{
			geomMin = geomMax = edge;
		}
		else
		{
			geomMin.x = min(geomMin.x, edge.x);
			geomMin.y = min(geomMin.y, edge.y);
			geomMin.z = min(geomMin.z, edge.z);
			geomMax.x = max(geomMax.x, edge.x);
			geomMax.y = max(geomMax.y, edge.y);
			geomMax.z = max(geomMax.z, edge.z);
		}
	}
}

void HIGHOMEGA::GL::GeometryClass::SetMinMax(vec3 minVal, vec3 maxVal)
{
	geomMin = minVal;
	geomMax = maxVal;
}

HIGHOMEGA::GL::GeometryClass::~GeometryClass()
{
	RemovePast();
}

void HIGHOMEGA::GL::GeometryClass::RemovePast()
{
	for (std::pair<ChangeSignalClass * const, unsigned long long> & curSub : notifySubmissions)
		curSub.first->ChangeSignal(curSub.second);
	notifySubmissions.clear();

	vertBuffer.RemovePast();
}

GeometryClass::GeometryClass()
{
}

void HIGHOMEGA::GL::GeometryClass::ChangeGeom(std::vector<RasterVertex>& inpVertexData)
{
	std::unordered_map <ChangeSignalClass *, unsigned long long> curNotifySubList = notifySubmissions;

	InstanceClass *instanceRefCached = instanceRef;
	DataLayout layoutCacheCached = layoutCache;
	bool isAlphaKeyedCached = isAlphaKeyedCache;
	std::string groupIdCached = groupId;
	bool withStagingBuffersCached = withStagingBuffersCache;
	bool immutableCached = immutable;

	RemovePast();
	Geometry(*instanceRefCached, inpVertexData, layoutCacheCached, isAlphaKeyedCached, groupIdCached, withStagingBuffersCached, immutableCached, nullptr, nullptr);
	notifySubmissions = curNotifySubList;
}

void GeometryClass::Geometry(InstanceClass &inpInstance, std::vector<RasterVertex> &inpVertexData, const DataLayout &inpDataLayout, bool isAlphaKeyed, std::string & inpGroupId, bool withStagingBuffers, bool inpImmutable, std::vector <VertexAnimationInfo> *vertAnimInfo, std::string *inpArmatureId)
{
	layoutCache = inpDataLayout;

	instanceRef = &inpInstance;
	isAlphaKeyedCache = isAlphaKeyed;
	withStagingBuffersCache = withStagingBuffers;
	immutable = inpImmutable;
	groupId = inpGroupId;

	vertexBufferSize = (unsigned int)inpVertexData.size() * sizeof(RasterVertex);
	if (vertAnimInfo)
	{
		if (vertAnimInfo->size() != inpVertexData.size()) throw std::runtime_error("Mismatch between animation and vertex data size");
		vertAnimInfoBufSize = (unsigned int)vertAnimInfo->size() * sizeof(VertexAnimationInfo);
		vertAnimInfoBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, inpInstance, (void *)vertAnimInfo->data(), vertAnimInfoBufSize);
		if (inpArmatureId) armatureId = *inpArmatureId;
	}

	MEMORY_USAGE memUsage = USAGE_SSBO | USAGE_VERT;
	if (RTInstance::Enabled())
		memUsage |= (USAGE_ACCEL_STRUCT | USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY);
	if (withStagingBuffers)
	{
		vertBuffer.BufferClassWithStaging(SHARING_DEFAULT, MODE_CREATE, memUsage, inpInstance, (void *)inpVertexData.data(), vertexBufferSize);
	}
	else
	{
		vertBuffer.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, memUsage, inpInstance, (void *)inpVertexData.data(), vertexBufferSize);
	}

	getMinMax(inpVertexData);

	vertBuffer.VERT_bindingDescriptions.resize(1);
	vertBuffer.VERT_bindingDescriptions[0].binding = inpDataLayout.BindPoint;
	vertBuffer.VERT_bindingDescriptions[0].stride = sizeof(RasterVertex);
	vertBuffer.VERT_bindingDescriptions[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

	vertBuffer.VERT_attributeDescriptions.resize(inpDataLayout.DataPoints.size());

	unsigned int current_offset = 0;
	for (std::vector<DataLayout>::size_type i = 0; i != inpDataLayout.DataPoints.size(); i++)
	{
		vertBuffer.VERT_attributeDescriptions[i].binding = inpDataLayout.BindPoint;
		vertBuffer.VERT_attributeDescriptions[i].location = (uint32_t)i;
		vertBuffer.VERT_attributeDescriptions[i].format = (VkFormat)inpDataLayout.DataPoints[i];
		vertBuffer.VERT_attributeDescriptions[i].offset = current_offset;
		current_offset += FormatSize(inpDataLayout.DataPoints[i]);
	}

	vertBuffer.VERT_vi = {};
	vertBuffer.VERT_vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vertBuffer.VERT_vi.pNext = VK_NULL_HANDLE;
	vertBuffer.VERT_vi.flags = 0;
	vertBuffer.VERT_vi.vertexBindingDescriptionCount = (uint32_t)vertBuffer.VERT_bindingDescriptions.size();
	vertBuffer.VERT_vi.pVertexBindingDescriptions = vertBuffer.VERT_bindingDescriptions.data();
	vertBuffer.VERT_vi.vertexAttributeDescriptionCount = (uint32_t)vertBuffer.VERT_attributeDescriptions.size();
	vertBuffer.VERT_vi.pVertexAttributeDescriptions = vertBuffer.VERT_attributeDescriptions.data();

	if (RTInstance::Enabled())
	{
		rtGeom.SetGeom(vertBuffer, sizeof(RasterVertex), isAlphaKeyedCache, immutable, Instance);
	}
}

void HIGHOMEGA::GL::GeometryClass::Update(std::vector<RasterVertex>& inpVertexData)
{
	if ((unsigned int)inpVertexData.size() * sizeof(RasterVertex) != vertexBufferSize || immutable) return;

	vertBuffer.UploadSubData(0, inpVertexData.data(), (unsigned int)inpVertexData.size() * sizeof(RasterVertex));

	getMinMax(inpVertexData);

	if (RTInstance::Enabled())
	{
		rtGeom.SetGeom(vertBuffer, sizeof(RasterVertex), isAlphaKeyedCache, false, Instance);
	}
}

KHR_RT::RTGeometry & HIGHOMEGA::GL::GeometryClass::getRTGeom()
{
	return rtGeom;
}

BufferClass & HIGHOMEGA::GL::GeometryClass::getVertBuffer()
{
	return vertBuffer;
}

BufferClass & HIGHOMEGA::GL::GeometryClass::getAnimInfoBuf()
{
	return vertAnimInfoBuf;
}

void HIGHOMEGA::GL::GeometryClass::setRTBufferDirty()
{
	rtGeom.SetDirty();
}

std::string HIGHOMEGA::GL::GeometryClass::getGroupId()
{
	return groupId;
}

std::string & HIGHOMEGA::GL::GeometryClass::getArmatureId()
{
	return armatureId;
}

vec3 & HIGHOMEGA::GL::GeometryClass::getGeomMin()
{
	if (minMaxTransformed) return postTransformMin;
	return geomMin;
}

vec3 & HIGHOMEGA::GL::GeometryClass::getGeomMax()
{
	if (minMaxTransformed) return postTransformMax;
	return geomMax;
}

vec3 & HIGHOMEGA::GL::GeometryClass::getUntransformedGeomMin()
{
	return geomMin;
}

vec3 & HIGHOMEGA::GL::GeometryClass::getUntransformedGeomMax()
{
	return geomMax;
}

void HIGHOMEGA::GL::GeometryClass::TransformCorners(mat4 inpMat)
{
	vec3 cent = (geomMin + geomMax) * 0.5f; // Box2D trick from zeuxcg
	vec3 extent = (geomMax - geomMin) * 0.5f;

	mat4 inpMatDT = inpMat.DirectionTransform();
	for (int i = 0; i != 4; i++)
		for (int j = 0; j != 4; j++)
			inpMatDT.i[i][j] = fabs(inpMatDT.i[i][j]);
	cent = inpMat * cent;
	extent = inpMatDT * extent;

	postTransformMin = cent - extent;
	postTransformMax = cent + extent;
	minMaxTransformed = true;
}

void BufferClass::RemovePast()
{
	if (haveSubAlloc) FreeMem (subAllocs, instanceRef->device);
	if (haveBuffer)
	{
		vkDestroyBuffer(instanceRef->device, buffer, nullptr);
		if (stagingBufferPtr)
		{
			{std::unique_lock <std::mutex> lk(stagingBuffers.mtx);
			stagingBuffers.dir[stagingBufferPtr->keyRef].elemCount--;
			if (stagingBuffers.dir[stagingBufferPtr->keyRef].elemCount == 0)
			{
				delete stagingBuffers.dir[stagingBufferPtr->keyRef].elem;
				stagingBuffers.dir.erase(stagingBufferPtr->keyRef);
			}}
			stagingBufferPtr = nullptr;
		}
	}

	haveSubAlloc = false;
	haveBuffer = false;
	totalDataSize = 0;
}

BufferClass::BufferClass()
{
	haveSubAlloc = false;
	haveBuffer = false;
	totalDataSize = 0;
}

void HIGHOMEGA::GL::BufferClass::Buffer(MEMORY_OPTIONS inpMemOpts, BUFFER_SHARING inpSharing, BUFFER_MODE inpMode, MEMORY_USAGE inpUsage, InstanceClass & inpInstance, void * inpData, unsigned int inpDataSize)
{
	if (totalDataSize > 0) RemovePast();

	instanceRef = &inpInstance;
	VkMemoryRequirements memReqs;

	usage = inpUsage;
	memOpts = inpMemOpts;
	sharing = inpSharing;
	mode = inpMode;
	totalDataSize = inpDataSize;

	VkBufferCreateInfo bufferInfo = {};
	allocInfo = {};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.pNext = VK_NULL_HANDLE;
	allocInfo.allocationSize = 0;
	allocInfo.memoryTypeIndex = 0;

	if (mode == MODE_CREATE) bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = totalDataSize;
	bufferInfo.usage = inpUsage;
	if (sharing == SHARING_EXCLUSIVE) bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	if (inpInstance.SupportsSparseResources()) bufferInfo.flags |= VK_BUFFER_CREATE_SPARSE_BINDING_BIT;

	VkResult result = vkCreateBuffer(inpInstance.device, &bufferInfo, nullptr, &buffer);
	if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create uniform buffer"); }
	haveBuffer = true;

	vkGetBufferMemoryRequirements(inpInstance.device, buffer, &memReqs);
	getMemoryType(&inpInstance, memReqs.memoryTypeBits, memOpts, &allocInfo.memoryTypeIndex);

	try
	{
		subAllocs = AllocMem(inpInstance.device, allocInfo, memReqs, ((usage & USAGE_DEVICE_ADDRESS) != 0) ? DEV_ADDRESS : BUFFER, inpInstance.SupportsSparseResources());
		haveSubAlloc = true;
	}
	catch (...)
	{
		RemovePast();
		throw std::runtime_error("Could not allocate buffer memory");
	}

	if (inpInstance.SupportsSparseResources())
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		std::unique_lock <std::mutex> lk2(inpInstance.queue_mutex);
		FenceClass sparseFence;
		sparseFence.Fence(&inpInstance);
		sparseFence.Reset();
		VkSparseBufferMemoryBindInfo bufferMemoryBinds;
		bufferMemoryBinds.buffer = buffer;
		std::vector<VkSparseMemoryBind> memoryBinds;
		unsigned long long resourceOffset = 0ull;
		for (std::pair<MEMORY_MANAGER::MemChunk*, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
		{
			VkSparseMemoryBind curBufBind = {};
			curBufBind.memory = curSubAlloc.first->mem;
			curBufBind.memoryOffset = (VkDeviceSize)curSubAlloc.second.offset;
			curBufBind.size = (VkDeviceSize)curSubAlloc.second.len;
			curBufBind.resourceOffset = (VkDeviceSize)resourceOffset;
			resourceOffset += curSubAlloc.second.len;
			memoryBinds.push_back(curBufBind);
		}
		bufferMemoryBinds.bindCount = (uint32_t)memoryBinds.size();
		bufferMemoryBinds.pBinds = memoryBinds.data();
		VkBindSparseInfo vkBindSparseInfo = {};
		vkBindSparseInfo.sType = VK_STRUCTURE_TYPE_BIND_SPARSE_INFO;
		vkBindSparseInfo.bufferBindCount = 1;
		vkBindSparseInfo.pBufferBinds = &bufferMemoryBinds;
		result = vkQueueBindSparse(inpInstance.submissionQueue, 1, &vkBindSparseInfo, sparseFence.fence);
		sparseFence.Wait();
	}
	else
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		result = vkBindBufferMemory(inpInstance.device, buffer, subAllocs.begin()->first->mem, subAllocs.begin()->second.offset);
	}
	if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not bind buffer to memory"); }

	descriptor.buffer = buffer;
	descriptor.offset = 0;
	descriptor.range = totalDataSize;

	if (inpData != nullptr)
	{
		try
		{
			UploadSubData(0, inpData, totalDataSize);
		}
		catch (...) { RemovePast(); throw std::runtime_error("Could not upload uniform buffer data"); }
	}
}

void HIGHOMEGA::GL::BufferClass::BufferClassWithStaging(BUFFER_SHARING inpSharing, BUFFER_MODE inpMode, MEMORY_USAGE inpUsage, InstanceClass & inpInstance, void * inpData, unsigned int inpDataSize)
{
	totalDataSize = inpDataSize;

	{std::unique_lock <std::mutex> lk(stagingBuffers.mtx);
	if (stagingBuffers.dir.find(ThreadID) == stagingBuffers.dir.end() || stagingBuffers.dir[ThreadID].elem->getSize() < totalDataSize)
	{
		if (stagingBuffers.dir.find(ThreadID) != stagingBuffers.dir.end())
			delete stagingBuffers.dir[ThreadID].elem;
		else
		{
			stagingBuffers.dir[ThreadID].elemCount = 0;
			stagingBuffers.dir[ThreadID].keyRef = ThreadID;
		}
		stagingBuffers.dir[ThreadID].elem = new BufferClass(MEMORY_HOST_VISIBLE, inpSharing, inpMode, USAGE_SRC | inpUsage, inpInstance, nullptr, totalDataSize);
	}
	stagingBuffers.dir[ThreadID].elemCount++;
	stagingBufferPtr = &stagingBuffers.dir[ThreadID];}
	stagingBufferPtr->elem->UploadSubData(0, inpData, totalDataSize);
	Buffer(MEMORY_DEVICE_LOCAL, inpSharing, inpMode, USAGE_DST | inpUsage, inpInstance, nullptr, totalDataSize);

	VkBufferCopy copyRegion = {};

	try { BeginCommandBuffer(inpInstance); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not begin setup cmd buffer for buffer creation"); }

	copyRegion.size = totalDataSize;
	vkCmdCopyBuffer(cmdBuffers[0], stagingBufferPtr->elem->buffer, buffer, 1, &copyRegion);

	try { EndCommandBuffer(); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not end setup cmd buffer for buffer creation"); }

	try { SubmitCommandBuffer(); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not submit setup cmd buffer for buffer creation"); }
}

unsigned int HIGHOMEGA::GL::BufferClass::getSize()
{
	return totalDataSize;
}

void HIGHOMEGA::GL::BufferClass::UploadSubData(unsigned int inpOffset, void *inpData, unsigned int inpDataSize)
{
	std::lock_guard<std::mutex> lk(mem_manager_mutex);
	if (!haveBuffer || !haveSubAlloc) throw std::runtime_error("Buffer not initialized properly for upload");

	unsigned char *dataPtr;
	VkResult result;
	unsigned int copiedSoFar = 0u;
	for (std::pair<MEMORY_MANAGER::MemChunk*, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
	{
		if (inpOffset >= (unsigned int)curSubAlloc.second.len)
		{
			inpOffset -= (unsigned int)curSubAlloc.second.len;
			continue;
		}
		unsigned long long copyAmount = min(curSubAlloc.second.len - inpOffset, inpDataSize);
		result = vkMapMemory(instanceRef->device, curSubAlloc.first->mem, curSubAlloc.second.offset + inpOffset, (VkDeviceSize)copyAmount, 0, (void**)&dataPtr);
		if (result != VK_SUCCESS) throw std::runtime_error("Memory map failed for upload");
		memcpy(dataPtr, (void *)&(((unsigned char *)inpData)[copiedSoFar]), copyAmount);
		inpDataSize -= (unsigned int)copyAmount;
		copiedSoFar += (unsigned int)copyAmount;
		inpOffset = 0u;
		vkUnmapMemory(instanceRef->device, curSubAlloc.first->mem);
		if (inpDataSize == 0u) break;
	}
}

void HIGHOMEGA::GL::BufferClass::DownloadSubData(unsigned int inpOffset, void *inpData, unsigned int inpDataSize)
{
	std::lock_guard<std::mutex> lk(mem_manager_mutex);
	if (!haveBuffer || !haveSubAlloc) throw std::runtime_error("Buffer not initialized properly for download");

	unsigned char *dataPtr;
	VkResult result;
	unsigned int copiedSoFar = 0u;
	for (std::pair<MEMORY_MANAGER::MemChunk*, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
	{
		if (inpOffset >= (unsigned int)curSubAlloc.second.len)
		{
			inpOffset -= (unsigned int)curSubAlloc.second.len;
			continue;
		}
		unsigned long long copyAmount = min(curSubAlloc.second.len - inpOffset, inpDataSize);
		result = vkMapMemory(instanceRef->device, curSubAlloc.first->mem, curSubAlloc.second.offset + inpOffset, (VkDeviceSize)copyAmount, 0, (void**)&dataPtr);
		if (result != VK_SUCCESS) throw std::runtime_error("Memory map failed for download");
		memcpy((void*)&(((unsigned char*)inpData)[copiedSoFar]), dataPtr, copyAmount);
		inpDataSize -= (unsigned int)copyAmount;
		copiedSoFar += (unsigned int)copyAmount;
		inpOffset = 0u;
		vkUnmapMemory(instanceRef->device, curSubAlloc.first->mem);
		if (inpDataSize == 0u) break;
	}
}

BufferClass::~BufferClass()
{
	RemovePast();
}

HIGHOMEGA::GL::ShaderStage::ShaderStage()
{
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::ShaderStage::Shader(InstanceClass &inpPtrToInstance, std::string shaderFile, const char * entryName, VkShaderStageFlagBits stage_bit)
{
	VkShaderModule shaderModule;
	VkShaderModuleCreateInfo moduleCreateInfo;

	moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	moduleCreateInfo.pNext = VK_NULL_HANDLE;

	std::string shaderFileLowercase = shaderFile;
	std::transform(shaderFileLowercase.begin(), shaderFileLowercase.end(), shaderFileLowercase.begin(), ::tolower);

	unsigned int shaderFileSize = 0;
	char *shaderFileContent;

#ifdef HIGHOMEGA_EXTRACT_SHADERS_FROM_BINARY
	unsigned char *shaderBinary = HIGHOMEGA::ENCODED_SHADERS::shaderMap[shaderFileLowercase];
	for (int i = 0; i != HIGHOMEGA::ENCODED_SHADERS::shaderSizeMap[shaderFileLowercase]; i++)
		shaderBinary[i] = ~shaderBinary[i];
	shaderFileSize = HIGHOMEGA::ENCODED_SHADERS::shaderSizeMap[shaderFileLowercase];
	shaderFileContent = (char *)shaderBinary;

#else
	try
	{
		ResourceLoader::Load(shaderFile, &shaderFileContent, &shaderFileSize);
	}
	catch (...)
	{
		throw std::runtime_error("Could not load shader file");
	}
#endif

	moduleCreateInfo.pCode = (uint32_t*)shaderFileContent;
	moduleCreateInfo.codeSize = shaderFileSize;
	moduleCreateInfo.flags = 0;
	VkResult result = vkCreateShaderModule(inpPtrToInstance.device, &moduleCreateInfo, nullptr, &shaderModule);

#ifdef HIGHOMEGA_EXTRACT_SHADERS_FROM_BINARY
	shaderBinary = HIGHOMEGA::ENCODED_SHADERS::shaderMap[shaderFileLowercase];
	for (int i = 0; i != HIGHOMEGA::ENCODED_SHADERS::shaderSizeMap[shaderFileLowercase]; i++)
		shaderBinary[i] = ~shaderBinary[i];
#endif

	if (result != VK_SUCCESS || !shaderModule) throw std::runtime_error("Failed to create shader module");

	stage = {};
	stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stage.stage = stage_bit;
	stage.pName = entryName;
	stage.module = shaderModule;

	ptrToInstance = &inpPtrToInstance;
}

HIGHOMEGA::GL::ShaderStage::~ShaderStage()
{
	if (ptrToInstance != nullptr) vkDestroyShaderModule(ptrToInstance->device, stage.module, nullptr);
	ptrToInstance = nullptr;
}

CacheItem<ShaderStage>* HIGHOMEGA::GL::AddOrFindCachedShaderStage(InstanceClass & ptrToInstance, std::string shaderFile, const char * entryName, VkShaderStageFlagBits stage_bit)
{
	std::lock_guard<std::mutex> lk(shader_stage_mutex);
	std::string shaderStageKey = shaderFile + ":" + entryName;
	if (ShaderStageCache.find(shaderStageKey) == ShaderStageCache.end())
	{
		try
		{
			ShaderStageCache[shaderStageKey].elem.Shader(ptrToInstance, shaderFile, entryName, stage_bit);
			ShaderStageCache[shaderStageKey].elemCount = 1;
		}
		catch (...)
		{
			ShaderStageCache.erase(shaderStageKey);
			throw std::runtime_error("Could not create shader stage for cache");
		}
	}
	else
		ShaderStageCache[shaderStageKey].elemCount++;
	return &ShaderStageCache[shaderStageKey];
}

void HIGHOMEGA::GL::packRasterVertex(vec3 & pos, vec3 & col, vec2 & uv, vec3 & vNorm, RasterVertex & rv)
{
	rv.posCol[0] = pos.x;
	rv.posCol[1] = pos.y;
	rv.posCol[2] = pos.z;
	rv.posCol[3] = packColor(col);

	rv.uv = toFP16(uv);
	rv.Norm = toZSignXY(vNorm.normalized());
}

void HIGHOMEGA::GL::unpackRasterVertex(vec3 & pos, vec3 & col, vec2 & uv, vec3 & vNorm, RasterVertex & rv)
{
	pos.x = rv.posCol[0];
	pos.y = rv.posCol[1];
	pos.z = rv.posCol[2];
	unpackColor(rv.posCol[3], col);

	uv = toFP32 (rv.uv);
	vNorm = fromZSignXY(rv.Norm);
}

void HIGHOMEGA::GL::UnpackMat4(const mat4 & inpMat, void * loc)
{
	for (int i = 0; i != 16; i++)
		((float *)loc)[i] = inpMat.i[i % 4][i / 4];
}

void HIGHOMEGA::GL::PackMat4(void * loc, mat4 & inpMat)
{
	for (int i = 0; i != 16; i++)
		inpMat.i[i % 4][i / 4] = ((float *)loc)[i];
}

int HIGHOMEGA::GL::FormatSize(FORMAT inpFormat)
{
	switch (inpFormat)
	{
	case R8SRGB:
	case R8UN:
	case R8SN:
		return 1;
	case R16F:
	case R8G8SRGB:
	case R8G8UN:
	case R8G8SN:
		return 2;
	case R8G8B8SRGB:
	case R8G8B8UN:
	case R8G8B8SN:
		return 3;
	case R16G16F:
	case R8G8B8A8SRGB:
	case R8G8B8A8UN:
	case R8G8B8A8SN:
	case R32F:
	case R32UI:
	case R32SI:
		return 4;
	case R16G16B16F:
		return 6;
	case R16G16B16A16F:
	case R32G32UI:
	case R32G32F:
		return 8;
	case R32G32B32UI:
	case R32G32B32F:
		return 12;
	case R32G32B32A32UI:
	case R32G32B32A32F:
		return 16;
	default:
		return 0;
	}
}

HIGHOMEGA::GL::InstanceClass *HIGHOMEGA::GL::CommandBuffer::ptrToInstance = nullptr;

HIGHOMEGA::GL::Timestamp::~Timestamp()
{
	if (!ptrToInstance) return;
	vkDestroyQueryPool(ptrToInstance->device, timingPool, nullptr);
}

HIGHOMEGA::GL::Timestamp::Timestamp()
{
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::Timestamp::Create(InstanceClass& instancePtr)
{
	ptrToInstance = &instancePtr;
	VkQueryPoolCreateInfo queryPoolInfo = {};
	queryPoolInfo.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
	queryPoolInfo.queryType = VK_QUERY_TYPE_TIMESTAMP;
	queryPoolInfo.queryCount = 2;
	VkResult res = vkCreateQueryPool(ptrToInstance->device, &queryPoolInfo, nullptr, &timingPool);
	if (res != VK_SUCCESS) throw std::runtime_error("Timestamp: could not create query pool");
}

void HIGHOMEGA::GL::Timestamp::Start(VkCommandBuffer& cmdBuf)
{
	if (!ptrToInstance) throw std::runtime_error("Timestamp: starting without an instance");
	vkCmdWriteTimestamp(cmdBuf, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, timingPool, 0);
}

void HIGHOMEGA::GL::Timestamp::End(VkCommandBuffer& cmdBuf)
{
	if (!ptrToInstance) throw std::runtime_error("Timestamp: ending without an instance");
	vkCmdWriteTimestamp(cmdBuf, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timingPool, 1);
}

unsigned long long HIGHOMEGA::GL::Timestamp::Report()
{
	if (!ptrToInstance) throw std::runtime_error("Timestamp: reporting without an instance");
	unsigned long long queryResults[2];
	VkResult res = vkGetQueryPoolResults(ptrToInstance->device, timingPool, 0, 2, sizeof(queryResults), queryResults, sizeof(unsigned long long), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
	if (res != VK_SUCCESS) throw std::runtime_error("Timestamp: could not fetch timestamp results");
	return queryResults[1] - queryResults[0];
}

void HIGHOMEGA::GL::CommandBuffer::DestroyCommandBuffer()
{
	if (!ptrToInstance || !cmdPoolRef) return;

	if (haveSetupCmdBuffer)
	{
		vkFreeCommandBuffers(ptrToInstance->device, cmdPoolRef->elem, (uint32_t)cmdBuffers.size(), cmdBuffers.data());
		{std::lock_guard <std::mutex> lk(cmdPools.mtx);
		cmdPoolRef->elemCount--;
		if (cmdPoolRef->elemCount == 0)
		{
			vkDestroyCommandPool(ptrToInstance->device, cmdPoolRef->elem, nullptr);
			cmdPools.dir.erase(cmdPoolRef->keyRef);
		}}
		haveSetupCmdBuffer = false;
	}
	initFenceClass = false;
}

HIGHOMEGA::GL::CommandBuffer::~CommandBuffer()
{
	DestroyCommandBuffer();
}

ThreadLocalCache <VkCommandPool>::value *HIGHOMEGA::GL::CommandBuffer::CreateCommandPool(InstanceClass &providedInstance, bool doNotIncreaseRefCount)
{
	std::lock_guard <std::mutex> lk(cmdPools.mtx);
	if (!ptrToInstance) ptrToInstance = &providedInstance;

	if (cmdPools.dir.find(ThreadID) == cmdPools.dir.end())
	{
		VkCommandPoolCreateInfo cmdPoolInfo = {};
		cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
		cmdPoolInfo.queueFamilyIndex = ptrToInstance->graphicsQueueNodeIndex;
		cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
		VkResult result = vkCreateCommandPool(ptrToInstance->device, &cmdPoolInfo, nullptr, &cmdPools.dir[ThreadID].elem);
		if (result != VK_SUCCESS) { throw std::runtime_error("CommandBuffer: Could not create command pool"); }
		cmdPools.dir[ThreadID].elemCount = doNotIncreaseRefCount ? 0 : 1;
		cmdPools.dir[ThreadID].keyRef = ThreadID;
	}
	else
		if (!doNotIncreaseRefCount) cmdPools.dir[ThreadID].elemCount++;
	return &cmdPools.dir[ThreadID];
}

void HIGHOMEGA::GL::CommandBuffer::BeginCommandBuffer(InstanceClass & instanceRef, unsigned int inpNumCmdBufs, unsigned int which)
{
	if (inpNumCmdBufs == 0) throw std::runtime_error("CommandBuffer: number of command buffers cannot be zero");

	VkResult result;

	if (!initFenceClass)
	{
		Fence(&instanceRef);
		initFenceClass = true;
	}

	if (inpNumCmdBufs != cmdBuffers.size())
	{
		if (cmdBuffers.size() > 0) DestroyCommandBuffer();
		cmdPoolRef = CreateCommandPool(instanceRef);

		cmdBuffers.resize(inpNumCmdBufs);

		VkCommandBufferAllocateInfo cmdBufAllocateInfo = {};
		cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		cmdBufAllocateInfo.commandPool = cmdPoolRef->elem;
		cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		cmdBufAllocateInfo.commandBufferCount = (uint32_t)cmdBuffers.size();

		result = vkAllocateCommandBuffers(ptrToInstance->device, &cmdBufAllocateInfo, cmdBuffers.data());
		if (result != VK_SUCCESS) { throw std::runtime_error("CommandBuffer: Could not create command buffer"); }
		haveSetupCmdBuffer = true;
	}
	else
	{
		result = vkResetCommandBuffer(cmdBuffers[which], (VkCommandBufferResetFlags)0);
		if (result != VK_SUCCESS) { throw std::runtime_error("CommandBuffer: Could not reset command buffer"); }
	}

	VkCommandBufferBeginInfo cmdBufInfo = {};
	cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

	result = vkBeginCommandBuffer(cmdBuffers[which], &cmdBufInfo);
	if (result != VK_SUCCESS) throw std::runtime_error("CommandBuffer: Could not begin command buffer");
}

void HIGHOMEGA::GL::CommandBuffer::EndCommandBuffer(unsigned int which)
{
	if (!ptrToInstance || !cmdPoolRef) throw std::runtime_error("CommandBuffer: No instance or cmd pool supplied");

	VkResult result = vkEndCommandBuffer(cmdBuffers[which]);
	if (result != VK_SUCCESS) throw std::runtime_error("CommandBuffer: Could not end command buffer");
}

void HIGHOMEGA::GL::CommandBuffer::SubmitCommandBuffer(unsigned int which)
{
	if (!ptrToInstance || !cmdPoolRef) throw std::runtime_error("CommandBuffer: No instance or cmd pool supplied");

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmdBuffers[which];

	Reset();

	VkResult result;
	{std::unique_lock<std::mutex> lk(ptrToInstance->queue_mutex);
	result = vkQueueSubmit(ptrToInstance->submissionQueue, 1, &submitInfo, fence); }
	if (result != VK_SUCCESS)
	{
		if (result == VK_ERROR_OUT_OF_HOST_MEMORY) throw std::runtime_error("CommandBuffer could not submit queue: out of host memory");
		else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) throw std::runtime_error("CommandBuffer could not submit queue: out of device memory");
		else throw std::runtime_error("CommandBuffer could not submit queue: device lost");
	}

	Wait();
}

void HIGHOMEGA::GL::CommandBuffer::WaitSubmitSignalCommandBuffer(SUBMISSION_MODE inpSubMode, SemaphoreClass & inpSemaphore, unsigned int which)
{
	if (!ptrToInstance || !cmdPoolRef) throw std::runtime_error("CommandBuffer: No instance or cmd pool supplied");

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	if (inpSubMode == SUBMIT_SERIAL)
	{
		submitInfo.waitSemaphoreCount = (uint32_t)waitSemaphores.size();
		submitInfo.pWaitSemaphores = waitSemaphores.data();
	}
	else if (inpSubMode == SUBMIT_ASYNC)
	{
		submitInfo.waitSemaphoreCount = 0;
		submitInfo.pWaitSemaphores = nullptr;
	}
	std::vector <VkPipelineStageFlags> waitMasks;
	for (int i = 0; i != submitInfo.waitSemaphoreCount; i++)
		waitMasks.push_back(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
	submitInfo.pWaitDstStageMask = waitMasks.data();
	submitInfo.signalSemaphoreCount = 1;
	submitInfo.pSignalSemaphores = &inpSemaphore.semaphore;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmdBuffers[which];

	if (inpSubMode == SUBMIT_SERIAL) Reset();

	VkResult result;
	{std::unique_lock<std::mutex> lk(ptrToInstance->queue_mutex);
	result = vkQueueSubmit(ptrToInstance->submissionQueue, 1, &submitInfo, inpSubMode == SUBMIT_SERIAL ? fence : VK_NULL_HANDLE); }
	if (result != VK_SUCCESS)
	{
		if (result == VK_ERROR_OUT_OF_HOST_MEMORY) throw std::runtime_error("CommandBuffer could not submit and wait on queue: out of host memory");
		else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) throw std::runtime_error("CommandBuffer could not submit and wait on queue: out of device memory");
		else throw std::runtime_error("CommandBuffer could not submit and wait on queue: device lost");
	}

	if (inpSubMode == SUBMIT_SERIAL)
	{
		Wait();
		waitSemaphores.clear();
	}
	waitSemaphores.push_back(inpSemaphore.semaphore);
}

unsigned int HIGHOMEGA::GL::CommandBuffer::CommandBufferCount()
{
	return (unsigned int)cmdBuffers.size();
}

HIGHOMEGA::GL::ImageClearColor::ImageClearColor(vec3 rgb, float alpha)
{
	floatRgb = rgb;
	floatAlpha = alpha;
	useFloat = true;
}

HIGHOMEGA::GL::ImageClearColor::ImageClearColor(unsigned int r, unsigned int g, unsigned int b, unsigned int a)
{
	uintR = r;
	uintG = g;
	uintB = b;
	uintA = a;
	useFloat = false;
}

void HIGHOMEGA::GL::ImageClass::setImageLayout(VkCommandBuffer cmdbuffer, std::vector<VkImage *> & images, VkImageAspectFlags aspectMask, VkImageLayout oldImageLayout, VkImageLayout newImageLayout, int numLayers, int baseMipLevel, int mipLevelCount)
{
	std::vector <VkImageMemoryBarrier> barriers;
	barriers.resize(images.size());
	for (int i = 0; i != images.size(); i++)
	{
		barriers[i] = {};
		barriers[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barriers[i].pNext = VK_NULL_HANDLE;
		barriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barriers[i].oldLayout = oldImageLayout;
		barriers[i].newLayout = newImageLayout;
		barriers[i].image = *(images[i]);
		barriers[i].subresourceRange.aspectMask = aspectMask;
		barriers[i].subresourceRange.baseMipLevel = baseMipLevel;
		barriers[i].subresourceRange.levelCount = mipLevelCount;
		barriers[i].subresourceRange.layerCount = numLayers;

		switch (oldImageLayout)
		{
		case VK_IMAGE_LAYOUT_UNDEFINED:
			barriers[i].srcAccessMask = 0;
			break;
		case VK_IMAGE_LAYOUT_PREINITIALIZED:
			barriers[i].srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
			break;
		case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
			barriers[i].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
			break;
		case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
			barriers[i].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
			break;
		case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
			barriers[i].srcAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			break;
		case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
			barriers[i].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			break;
		case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
			barriers[i].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
			break;
		default:
			break;
		}

		switch (newImageLayout)
		{
		case VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL:
			barriers[i].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
			break;

		case VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL:
			barriers[i].dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
			break;

		case VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL:
			barriers[i].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
			break;

		case VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL:
			barriers[i].dstAccessMask = barriers[i].dstAccessMask | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
			break;

		case VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL:
			if (barriers[i].srcAccessMask == 0)
				barriers[i].srcAccessMask = VK_ACCESS_HOST_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
			barriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
			break;
		default:
			break;
		}
	}

	vkCmdPipelineBarrier(cmdbuffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, (uint32_t)barriers.size(), barriers.data());
}

void HIGHOMEGA::GL::ImageClass::CreateImageView(DEPTH_STENCIL_MODE depthStencilMode, bool useFormat, FORMAT inpFormat, TEXTURE_DIM numDims, int numLayers, int baseLayer, int mipLevelCount, VkImageView & viewPtr, bool & inpHaveImageView)
{
	if (cachedInstance == nullptr) throw std::runtime_error("We do not have a pointer to the Vulkan instance");

	VkImageViewCreateInfo attachmentView = {};
	attachmentView.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	attachmentView.pNext = VK_NULL_HANDLE;
	attachmentView.format = useFormat ? ((VkFormat)inpFormat) : (depthStencilMode != NONE ? cachedInstance->selectedDepthFormat : cachedInstance->colorFormat);
	if (depthStencilMode == NONE) attachmentView.components = { VK_COMPONENT_SWIZZLE_R,VK_COMPONENT_SWIZZLE_G,VK_COMPONENT_SWIZZLE_B,VK_COMPONENT_SWIZZLE_A };
	switch (depthStencilMode)
	{
	case NONE:
		attachmentView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		break;
	case SAMPLE_NONE:
		attachmentView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
		break;
	case SAMPLE_DEPTH:
		attachmentView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
		break;
	case SAMPLE_STENCIL:
		attachmentView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_STENCIL_BIT;
		break;
	}
	attachmentView.subresourceRange.baseMipLevel = 0;
	attachmentView.subresourceRange.levelCount = mipLevelCount;
	attachmentView.subresourceRange.baseArrayLayer = baseLayer;
	attachmentView.subresourceRange.layerCount = numLayers;
	attachmentView.viewType = (VkImageViewType)numDims;
	attachmentView.flags = 0;
	attachmentView.image = image;

	VkResult result = vkCreateImageView(cachedInstance->device, &attachmentView, nullptr, &viewPtr);
	if (result != VK_SUCCESS) throw std::runtime_error("Could not create image-view");

	inpHaveImageView = true;
}

void HIGHOMEGA::GL::ImageClass::CreateImage(bool depthStencil, bool useFormat, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM numDims, int numLayers, int mipLevelCount, bool exclusiveShareMode, bool usedViaSampler, bool usedAsStorageTarget)
{
	if (cachedInstance == nullptr) throw std::runtime_error("We do not have a pointer to the Vulkan instance");

	VkImageCreateInfo imageCreateStruct = {};
	imageCreateStruct.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageCreateStruct.pNext = VK_NULL_HANDLE;
	imageCreateStruct.imageType = GetVkImageTypeFromDim(numDims);
	imageCreateStruct.format = useFormat ? ((VkFormat)inpFormat) : (depthStencil ? cachedInstance->selectedDepthFormat : cachedInstance->colorFormat);
	imageCreateStruct.extent = { (uint32_t)w, (uint32_t)h, (uint32_t)d };
	if (exclusiveShareMode) imageCreateStruct.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageCreateStruct.mipLevels = mipLevelCount;
	imageCreateStruct.arrayLayers = numLayers;
	imageCreateStruct.samples = VK_SAMPLE_COUNT_1_BIT;
	imageCreateStruct.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageCreateStruct.usage = depthStencil ? VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT : VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	if (usedViaSampler) imageCreateStruct.usage |= VK_IMAGE_USAGE_SAMPLED_BIT;
	imageCreateStruct.usage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	if (usedAsStorageTarget) imageCreateStruct.usage |= VK_IMAGE_USAGE_STORAGE_BIT;
	imageCreateStruct.flags = (numDims == _2D && numLayers == 6) ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0;
	if (cachedInstance->SupportsSparseResources()) imageCreateStruct.flags |= VK_IMAGE_CREATE_SPARSE_BINDING_BIT;

	VkResult result = vkCreateImage(cachedInstance->device, &imageCreateStruct, nullptr, &image);
	if (result != VK_SUCCESS) throw std::runtime_error("Could not create image");

	haveImage = true;
}

void HIGHOMEGA::GL::ImageClass::CreateMemoryForImage()
{
	if (cachedInstance == nullptr) throw std::runtime_error("We do not have a pointer to the Vulkan instance");

	VkMemoryAllocateInfo mem_alloc = {};
	mem_alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	mem_alloc.pNext = VK_NULL_HANDLE;
	mem_alloc.allocationSize = 0;
	mem_alloc.memoryTypeIndex = 0;

	VkMemoryRequirements memReqs;

	vkGetImageMemoryRequirements(cachedInstance->device, image, &memReqs);
	getMemoryType(cachedInstance, memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &mem_alloc.memoryTypeIndex);
	
	try
	{
		subAllocs = AllocMem(cachedInstance->device, mem_alloc, memReqs, IMAGE, cachedInstance->SupportsSparseResources());
		haveSubAlloc = true;
	}
	catch (...)
	{
		RemovePast();
		throw std::runtime_error("Could not allocate image memory");
	}

	VkResult result;
	if (cachedInstance->SupportsSparseResources())
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		std::unique_lock <std::mutex> lk2(cachedInstance->queue_mutex);
		FenceClass sparseFence;
		sparseFence.Fence(cachedInstance);
		sparseFence.Reset();
		VkSparseImageOpaqueMemoryBindInfo imageMemoryBinds;
		imageMemoryBinds.image = image;
		std::vector<VkSparseMemoryBind> memoryBinds;
		unsigned long long resourceOffset = 0ull;
		for (std::pair<MEMORY_MANAGER::MemChunk*, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
		{
			VkSparseMemoryBind curImgBind = {};
			curImgBind.memory = curSubAlloc.first->mem;
			curImgBind.memoryOffset = (VkDeviceSize)curSubAlloc.second.offset;
			curImgBind.size = (VkDeviceSize)curSubAlloc.second.len;
			curImgBind.resourceOffset = (VkDeviceSize)resourceOffset;
			resourceOffset += curSubAlloc.second.len;
			memoryBinds.push_back(curImgBind);
		}
		imageMemoryBinds.bindCount = (uint32_t)memoryBinds.size();
		imageMemoryBinds.pBinds = memoryBinds.data();
		VkBindSparseInfo vkBindSparseInfo = {};
		vkBindSparseInfo.sType = VK_STRUCTURE_TYPE_BIND_SPARSE_INFO;
		vkBindSparseInfo.imageOpaqueBindCount = 1;
		vkBindSparseInfo.pImageOpaqueBinds = &imageMemoryBinds;
		result = vkQueueBindSparse(cachedInstance->submissionQueue, 1, &vkBindSparseInfo, sparseFence.fence);
		sparseFence.Wait();
	}
	else
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		result = vkBindImageMemory(cachedInstance->device, image, subAllocs.begin()->first->mem, subAllocs.begin()->second.offset);
	}
	if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not bind image to memory"); }
}

void HIGHOMEGA::GL::ImageClass::CreateSampler(MIN_MAG_FILTER minFilter, MIN_MAG_FILTER magFilter, MIPMAP_MODE mipMapMode, TEXTURE_ADDRESS_MODE uAddress, TEXTURE_ADDRESS_MODE vAddress, TEXTURE_ADDRESS_MODE wAddress, float mipLodBias, float minLod, float maxLod, bool enableAnisotropy, float maxAnisotropy)
{
	if (cachedInstance == nullptr) throw std::runtime_error("We do not have a pointer to the Vulkan instance");

	VkSamplerCreateInfo samplerCreateInfo = {};
	samplerCreateInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
	samplerCreateInfo.pNext = VK_NULL_HANDLE;
	samplerCreateInfo.magFilter = (VkFilter)magFilter;
	samplerCreateInfo.minFilter = (VkFilter)minFilter;
	samplerCreateInfo.mipmapMode = (VkSamplerMipmapMode)mipMapMode;
	samplerCreateInfo.addressModeU = (VkSamplerAddressMode)uAddress;
	samplerCreateInfo.addressModeV = (VkSamplerAddressMode)vAddress;
	samplerCreateInfo.addressModeW = (VkSamplerAddressMode)wAddress;
	samplerCreateInfo.mipLodBias = mipLodBias;
	samplerCreateInfo.compareOp = VK_COMPARE_OP_NEVER;
	samplerCreateInfo.compareEnable = VK_TRUE;
	samplerCreateInfo.minLod = minLod;
	samplerCreateInfo.maxLod = maxLod;
	samplerCreateInfo.maxAnisotropy = enableAnisotropy ? maxAnisotropy : 1.0f;
	samplerCreateInfo.anisotropyEnable = enableAnisotropy ? VK_TRUE : VK_FALSE;
	samplerCreateInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;

	VkResult result = vkCreateSampler(cachedInstance->device, &samplerCreateInfo, nullptr, &sampler);
	if (result != VK_SUCCESS) throw std::runtime_error("Could not create sampler");

	haveSampler = true;
}

void HIGHOMEGA::GL::ImageClass::RemovePast()
{
	if (cachedInstance == nullptr)
	{
		if (!instanceEverSet) return;
		throw std::runtime_error("We do not have a pointer to the Vulkan instance");
	}

	if (haveKTXVulkanTexture) ktxVulkanTexture_Destruct(&ktxVulkanTexture, cachedInstance->ktxVDI.device, nullptr);
	if (haveImageView) vkDestroyImageView(cachedInstance->device, view, nullptr);
	for (VkImageView & curView : layerView) {
		vkDestroyImageView(cachedInstance->device, curView, nullptr);
	}
	if (stagingBufferPtr)
	{
		{std::unique_lock <std::mutex> lk(stagingBuffers.mtx);
		stagingBuffers.dir[stagingBufferPtr->keyRef].elemCount--;
		if (stagingBuffers.dir[stagingBufferPtr->keyRef].elemCount == 0)
		{
			delete stagingBuffers.dir[stagingBufferPtr->keyRef].elem;
			stagingBuffers.dir.erase(stagingBufferPtr->keyRef);
		}}
		stagingBufferPtr = nullptr;
	}
	if (loadedDataBuffer) delete loadedDataBuffer;
	if (haveSubAlloc) FreeMem(subAllocs, cachedInstance->device);
	if (haveImage) vkDestroyImage(cachedInstance->device, image, nullptr);
	if (haveSampler) vkDestroySampler(cachedInstance->device, sampler, nullptr);
	if (downloadData) delete[] downloadData;

	haveKTXVulkanTexture = false;
	haveSampler = false;
	haveImage = false;
	haveLayout = false;
	haveSubAlloc = false;
	haveImageView = false;
	recordedClearCmdBuffer = false;
	recordedUploadCmdBuffer = false;
	recordedCopyCmdBuffer = false;
	recordedDownloadCmdBuffer = false;
	downloadData = nullptr;
	loadedDataBuffer = nullptr;
}

HIGHOMEGA::GL::ImageClass::ImageClass()
{
	uploadedTexture = false;
	usedAsStorageTarget = false;
	cachedInstance = nullptr;
	instanceEverSet = false;

	haveKTXVulkanTexture = false;
	haveSampler = false;
	haveImage = false;
	haveLayout = false;
	haveSubAlloc = false;
	haveImageView = false;
	recordedClearCmdBuffer = false;
	recordedUploadCmdBuffer = false;
	recordedCopyCmdBuffer = false;
	recordedDownloadCmdBuffer = false;
	downloadData = nullptr;
	loadedDataBuffer = nullptr;
}

std::vector<ImageClass> HIGHOMEGA::GL::ImageClass::FromSwapChain(InstanceClass & inpInstance)
{
	uint32_t swapChainImageCount = 0;

	VkResult result = inpInstance.fpGetSwapchainImagesKHR(inpInstance.device, inpInstance.swapChain, &swapChainImageCount, nullptr);
	if (result != VK_SUCCESS) { throw std::runtime_error("Could not get swapchain image count"); }

	std::vector <VkImage> tmpImages;
	tmpImages.resize(swapChainImageCount);

	std::vector <ImageClass> images;
	images.resize(swapChainImageCount);

	result = inpInstance.fpGetSwapchainImagesKHR(inpInstance.device, inpInstance.swapChain, &swapChainImageCount, tmpImages.data());
	if (result != VK_SUCCESS) { throw std::runtime_error("Could not get swapchain images"); }

	for (uint32_t i = 0; i < swapChainImageCount; i++)
	{
		images[i].haveImage = false; // Image provided by swap chain. No need to destroy as we didn't make it.
		images[i].cachedInstance = &inpInstance;
		images[i].instanceEverSet = true;
	}

	for (uint32_t i = 0; i < swapChainImageCount; i++)
	{
		images[i].image = tmpImages[i];

		std::vector <VkImage *> imagesInFrontOfBarrier;
		imagesInFrontOfBarrier.push_back(&images[i].image);

		images[i].setImageLayout(inpInstance.cmdBuffers[0], imagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, 1, 0, 1);

		images[i].CreateImageView(NONE, false, (FORMAT)0, _2D, 1, 0, 1, images[i].view, images[i].haveImageView);
		images[i].haveImageView = true;
	}

	return images;
}

void HIGHOMEGA::GL::ImageClass::CreateOnScreenDepthStencil(InstanceClass & ptrToInstance, int w, int h)
{
	CreateStandaloneImage(ptrToInstance, false, (FORMAT)0, w, h, 1, _2D, 1, false, SAMPLE_NONE, false, false, false, false, false, false);
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenDepthStencil(InstanceClass & ptrToInstance, int w, int h, DEPTH_STENCIL_MODE inpDepthStencil)
{
	if (inpDepthStencil == NONE) inpDepthStencil = SAMPLE_NONE;
	CreateStandaloneImage(ptrToInstance, false, (FORMAT)0, w, h, 1, _2D, 1, false, inpDepthStencil, inpDepthStencil != SAMPLE_NONE, false, true, false, false, false);
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenColorAttachment(InstanceClass & ptrToInstance, FORMAT inFormat, int w, int h, bool linearFiltering, bool clampSamples)
{
	CreateStandaloneImage(ptrToInstance, true, inFormat, w, h, 1, _2D, 1, false, NONE, true, false, true, linearFiltering, clampSamples, false);
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenColorArrayAttachment(InstanceClass & ptrToInstance, FORMAT inFormat, int w, int h, int numLayers, bool linearFiltering, bool clampSamples)
{
	CreateStandaloneImage(ptrToInstance, true, inFormat, w, h, 1, _2D_ARRAY, numLayers, false, NONE, true, false, true, linearFiltering, clampSamples, false);
}

void HIGHOMEGA::GL::ImageClass::CreateImageStore(InstanceClass & ptrToInstance, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM numDims, bool linearFiltering)
{
	CreateStandaloneImage(ptrToInstance, true, inpFormat, w, h, d, numDims, 1, true, NONE, true, true, true, linearFiltering, false, false);
}

void HIGHOMEGA::GL::ImageClass::CreateCubeMap(InstanceClass & ptrToInstance, FORMAT inpFormat, int w, int h)
{
	CreateStandaloneImage(ptrToInstance, true, inpFormat, w, h, 1, _CUBE, 6, false, NONE, true, false, true, true, true, true);
}

bool HIGHOMEGA::GL::ImageClass::CreateTexture(InstanceClass & ptrToInstance, std::string belong, std::string fileName, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat)
{
	unsigned char *content = nullptr;
	unsigned int contentSize;
	HIGHOMEGA::ResourceLoader::LOAD_LOCATION loadLocation;
	HIGHOMEGA::ResourceLoader::LOAD_CHOSEN_ASSET loadChosenAsset;
	if (HIGHOMEGA::ResourceLoader::Load(belong, fileName, &content, contentSize, loadLocation, loadChosenAsset) != HIGHOMEGA::ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS) return false;
	if (loadChosenAsset == HIGHOMEGA::ResourceLoader::LOAD_CHOSEN_ASSET::IMG_KTX)
		CreateTextureFromFileOrData(ptrToInstance, content, contentSize, IMAGE_DATA_KTX, 0, 0, inD, inIs3D, inIsArray, isCube, doMipMapping, inpFormat);
	else
		CreateTextureFromFileOrData(ptrToInstance, content, contentSize, IMAGE_DATA_TGA, 0, 0, inD, inIs3D, inIsArray, isCube, doMipMapping, inpFormat);
	delete[] content;
	return true;
}

bool HIGHOMEGA::GL::ImageClass::CreateTexture(InstanceClass & ptrToInstance, unsigned int inW, unsigned int inH, unsigned char *inData, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat)
{
	CreateTextureFromFileOrData(ptrToInstance, inData, 0, IMAGE_DATA_RGB, inW, inH, inD, inIs3D, inIsArray, isCube, doMipMapping, inpFormat);
	return true;
}

void HIGHOMEGA::GL::ImageClass::CreateStandaloneImage(InstanceClass & ptrToInstance, bool useFormat, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM numDims, int numLayers, bool exclusiveShareMode, DEPTH_STENCIL_MODE depthStencilMode, bool createSampler, bool usedAsStorageTarget, bool createSetupCmdBuffer, bool linearFiltering, bool clampSamples, bool createCubeMap)
{
	cachedInstance = &ptrToInstance;
	this->usedAsStorageTarget = usedAsStorageTarget;
	uploadedTexture = false;
	instanceEverSet = true;
	if (useFormat) format = inpFormat;

	if (createSetupCmdBuffer)
	{
		try { setupStandAloneCmdBuffer.BeginCommandBuffer(*cachedInstance); }
		catch (...) { RemovePast(); throw std::runtime_error("Could not begin setup cmd buffer for standalone image"); }
	}

	width = w;
	height = h;
	depth = d;
	layers = numLayers;
	isArray = is3D = false;
	if (layers > 1) isArray = true;
	else if (depth > 1) is3D = true;

	try { CreateImage(depthStencilMode != NONE, useFormat, inpFormat, w, h, d, createCubeMap ? _2D : numDims, numLayers, 1, exclusiveShareMode, createSampler, usedAsStorageTarget); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create image"); }

	try { CreateMemoryForImage(); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create and bind memory for image"); }

	std::vector <VkImage *> imageInFrontOfBarrier;
	imageInFrontOfBarrier.push_back(&image);

	setImageLayout(createSetupCmdBuffer ? setupStandAloneCmdBuffer.cmdBuffers[0] : cachedInstance->cmdBuffers[0], imageInFrontOfBarrier,
				   (depthStencilMode != NONE) ? (VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT) : VK_IMAGE_ASPECT_COLOR_BIT,
				   VK_IMAGE_LAYOUT_UNDEFINED,
				   (depthStencilMode == SAMPLE_NONE) ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_GENERAL,
				   numLayers, 0, 1);

	try
	{
		CreateImageView(depthStencilMode, useFormat, inpFormat, createCubeMap ? _CUBE : numDims, numLayers, 0, 1, view, haveImageView);
		if (numLayers > 1) {
			for (int i = 0; i != numLayers; i++)
			{
				bool tmpCreatedFlag = false;
				VkImageView curImageView;
				CreateImageView(NONE, useFormat, inpFormat, _2D, 1, i, 1, curImageView, tmpCreatedFlag);
				layerView.push_back(curImageView);
			}
		}
	}
	catch (...) { RemovePast(); throw std::runtime_error("Could not create image view"); }

	if (createSampler) {
		try {
			CreateSampler(linearFiltering ? ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR : ImageClass::MIN_MAG_FILTER::MIN_MAG_NEAREST,
				linearFiltering ? ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR : ImageClass::MIN_MAG_FILTER::MIN_MAG_NEAREST,
				ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
				clampSamples ? ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE : ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
				clampSamples ? ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE : ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
				clampSamples ? ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE : ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
				0.0f, 0.0f, 1.0f, false, 1);
		}
		catch (...) { RemovePast(); throw std::runtime_error("Could not create sampler"); }
	}

	if (createSetupCmdBuffer)
	{
		try { setupStandAloneCmdBuffer.EndCommandBuffer(); }
		catch (...) { RemovePast(); throw std::runtime_error("Could not end setup cmd buffer for standalone image"); }

		try { setupStandAloneCmdBuffer.SubmitCommandBuffer(); }
		catch (...) { RemovePast(); throw std::runtime_error("Could not submit setup cmd buffer for standalone image"); }
	}
}

void HIGHOMEGA::GL::ImageClass::CreateTextureFromFileOrData(InstanceClass & ptrToInstance, unsigned char *data, unsigned int dataSize, PROVIDED_IMAGE_DATA_TYPE dataType, unsigned int inW, unsigned int inH, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat)
{
	cachedInstance = &ptrToInstance;
	this->usedAsStorageTarget = false;
	uploadedTexture = true;
	instanceEverSet = true;

	is3D = inIs3D;
	isArray = inIsArray;

	if (is3D)
	{
		depth = inD;
		layers = 1;
	}
	else if (isArray)
	{
		depth = 1;
		layers = inD;
	}
	else if (isCube)
	{
		depth = 1;
		layers = 6;
	}
	else
	{
		depth = 1;
		layers = 1;
	}

	unsigned char *feedData;
	if (dataType == IMAGE_DATA_RGB)
	{
		format = inpFormat;
		bpp = FormatSize(format);
		width = inW;
		height = inH;
		data_size = width * height*depth*layers*bpp;
		feedData = data;
	}
	else if (dataType == IMAGE_DATA_KTX)
	{
		ktxTexture2* kTexture;
		KTX_error_code result = ktxTexture_CreateFromMemory(data, dataSize, KTX_TEXTURE_CREATE_NO_FLAGS, (ktxTexture**)&kTexture);
		if (result != KTX_SUCCESS) { RemovePast(); throw std::runtime_error("Error creating ktxTexture from ktx file"); }
		if (ktxTexture2_NeedsTranscoding(kTexture))
		{
			ktx_texture_transcode_fmt_e tf = KTX_TTF_BC3_RGBA;
			result = ktxTexture2_TranscodeBasis(kTexture, KTX_TTF_BC3_RGBA, 0);
			if (result != KTX_SUCCESS) { RemovePast(); throw std::runtime_error("Error transcoding ktx file to BC3"); }
		}
		{std::unique_lock<std::mutex> lk(cachedInstance->queue_mutex);
		result = ktxTexture_VkUploadEx((ktxTexture*)kTexture, &cachedInstance->ktxVDI, &ktxVulkanTexture, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_SAMPLED_BIT, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
		if (result != KTX_SUCCESS) { RemovePast(); throw std::runtime_error("Error creating ktxVulkanTexture from ktx file"); }}
		ktxTexture_Destroy((ktxTexture*)kTexture);

		haveKTXVulkanTexture = true;

		width = ktxVulkanTexture.width;
		height = ktxVulkanTexture.height;
		depth = ktxVulkanTexture.depth;
		layers = ktxVulkanTexture.layerCount;
		mipLevels = ktxVulkanTexture.levelCount;

		bpp = 4;

		format = (FORMAT)ktxVulkanTexture.imageFormat;
		image = ktxVulkanTexture.image;

		bool enableAnisotropicFiltering = doMipMapping;

		try {
			CreateSampler(ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR, ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR,
				ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
				ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
				0.0f, 0.0f, (float)mipLevels, enableAnisotropicFiltering, enableAnisotropicFiltering ? 8.0f : 1.0f);
		}
		catch (...) { RemovePast(); throw std::runtime_error("Could not create sampler for ktx"); }

		TEXTURE_DIM selectedDim = _2D;
		if (is3D) {
			selectedDim = _3D;
		}
		else if (isArray) {
			selectedDim = _2D_ARRAY;
		}

		try { CreateImageView(NONE, true, (FORMAT)format, isCube ? _CUBE : selectedDim, layers, 0, mipLevels, view, haveImageView); }
		catch (...) { RemovePast(); throw std::runtime_error("Could not create image view for ktx texture"); }

		return;
	}
	else
	{
		if (HIGHOMEGA::ResourceLoader::TGALoad(data, (int *)&width, (int *)&height, &feedData, &bpp) != HIGHOMEGA::ResourceLoader::TGA_PARSE_RESULT::TGA_SUCCESS)
			throw std::runtime_error("Bad TGA format");
		// Vulkan does not support RGB if the hardware does not.
		if (bpp == 3) {
			HIGHOMEGA::ResourceLoader::RGBtoRGBA(width, height, &feedData);
			bpp = 4;
		}
		if (bpp == 1)
			format = R8UN;
		else
			format = (inpFormat == R8G8B8A8UN) ? R8G8B8A8UN : R8G8B8A8SRGB;
		data_size = width * height*bpp;
		height /= (depth * layers);
	}

	VkFormatProperties formatProperties;

	if (doMipMapping)
		mipLevels = (unsigned int)floor(log2(max(width, height))) + 1;
	else
		mipLevels = 1;

	vkGetPhysicalDeviceFormatProperties(ptrToInstance.physicalDevice, (VkFormat)format, &formatProperties);
	if (!(formatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_SRC_BIT) || !(formatProperties.optimalTilingFeatures & VK_FORMAT_FEATURE_BLIT_DST_BIT))
	{
		RemovePast(); throw std::runtime_error("format cannot be blitted...");
	}

	TEXTURE_DIM selectedDim = _2D;
	if (is3D) {
		selectedDim = _3D;
	}
	else if (isArray) {
		selectedDim = _2D_ARRAY;
	}

	try { CreateImage(false, true, format, width, height, depth, selectedDim, layers, mipLevels, true, true, false); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create image"); }

	try { CreateMemoryForImage(); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create and bind memory for image"); }

	{std::unique_lock <std::mutex> lk(stagingBuffers.mtx);
	if (stagingBuffers.dir.find(ThreadID) == stagingBuffers.dir.end() || stagingBuffers.dir[ThreadID].elem->getSize() < data_size)
	{
		if (stagingBuffers.dir.find(ThreadID) != stagingBuffers.dir.end())
			delete stagingBuffers.dir[ThreadID].elem;
		else
		{
			stagingBuffers.dir[ThreadID].elemCount = 0;
			stagingBuffers.dir[ThreadID].keyRef = ThreadID;
		}
		stagingBuffers.dir[ThreadID].elem = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_SRC, ptrToInstance, nullptr, data_size);
	}
	stagingBuffers.dir[ThreadID].elemCount++;
	stagingBufferPtr = &stagingBuffers.dir[ThreadID]; }

	stagingBufferPtr->elem->UploadSubData(0, feedData, data_size);

	if (dataType == IMAGE_DATA_TGA)
	{
		delete[] feedData;
		feedData = nullptr;
	}

	std::vector<VkBufferImageCopy> bufferCopyRegions;

	VkBufferImageCopy bufferCopyRegion = {};
	bufferCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
	bufferCopyRegion.imageSubresource.mipLevel = 0;
	bufferCopyRegion.imageSubresource.baseArrayLayer = 0;
	bufferCopyRegion.imageSubresource.layerCount = layers;
	bufferCopyRegion.imageExtent.width = width;
	bufferCopyRegion.imageExtent.height = height;
	bufferCopyRegion.imageExtent.depth = depth;
	bufferCopyRegion.bufferOffset = 0;

	bufferCopyRegions.push_back(bufferCopyRegion);

	try { setupTextureCmdBuffer.BeginCommandBuffer(*cachedInstance); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not begin setup cmd buffer for create texture"); }

	std::vector <VkImage *> imageInFrontOfBarrier;
	imageInFrontOfBarrier.push_back(&image);

	setImageLayout(setupTextureCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layers, 0, mipLevels);

	vkCmdCopyBufferToImage(setupTextureCmdBuffer.cmdBuffers[0], stagingBufferPtr->elem->buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, (uint32_t)bufferCopyRegions.size(), bufferCopyRegions.data());

	setImageLayout(setupTextureCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, 0, 1);

	for (int i = 1; i < (int)mipLevels; i++)
	{
		VkImageBlit imageBlit{};

		imageBlit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageBlit.srcSubresource.layerCount = layers;
		imageBlit.srcSubresource.mipLevel = i - 1;
		imageBlit.srcOffsets[0].x = 0;
		imageBlit.srcOffsets[0].y = 0;
		imageBlit.srcOffsets[0].z = 0;
		imageBlit.srcOffsets[1].x = int32_t(max (width >> (i - 1), 1));
		imageBlit.srcOffsets[1].y = int32_t(max (height >> (i - 1), 1));
		imageBlit.srcOffsets[1].z = 1;
		imageBlit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageBlit.dstSubresource.layerCount = layers;
		imageBlit.dstSubresource.mipLevel = i;
		imageBlit.dstOffsets[0].x = 0;
		imageBlit.dstOffsets[0].y = 0;
		imageBlit.dstOffsets[0].z = 0;
		imageBlit.dstOffsets[1].x = int32_t(max (width >> i, 1));
		imageBlit.dstOffsets[1].y = int32_t(max (height >> i, 1));
		imageBlit.dstOffsets[1].z = 1;

		setImageLayout(setupTextureCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layers, i, 1);

		vkCmdBlitImage(setupTextureCmdBuffer.cmdBuffers[0], image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &imageBlit, VK_FILTER_LINEAR);

		setImageLayout(setupTextureCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, i, 1);
	}

	setImageLayout(setupTextureCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, layers, 0, mipLevels);

	try { setupTextureCmdBuffer.EndCommandBuffer(); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not end setup cmd buffer for create texture"); }

	try { setupTextureCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not submit setup cmd buffer for create texture"); }

	bool enableAnisotropicFiltering = doMipMapping;

	try {
		CreateSampler(ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR, ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR,
			ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
			ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
			0.0f, 0.0f, (float)mipLevels, enableAnisotropicFiltering, enableAnisotropicFiltering ? 8.0f : 1.0f);
	}
	catch (...) { RemovePast(); throw std::runtime_error("Could not create sampler"); }

	try { CreateImageView(NONE, true, format, isCube ? _CUBE : selectedDim, layers, 0, mipLevels, view, haveImageView); }
	catch (...) { RemovePast(); throw std::runtime_error("Could not create image view for texture"); }
}

void HIGHOMEGA::GL::ImageClass::ClearColors(std::vector <ImageClass *> & images, ImageClearColor clearColor)
{
	images[0]->mipLevels = 1;

	if (!images[0]->recordedClearCmdBuffer)
	{
		try { images[0]->clearCmdBuffer.BeginCommandBuffer(*(images[0]->cachedInstance)); }
		catch (...) { throw std::runtime_error("Could not begin setup cmd buffer for clear color"); }

		std::vector <VkImage *> imagesInFrontOfBarrier;
		for (ImageClass * curImg : images)
			imagesInFrontOfBarrier.push_back(&curImg->image);

		images[0]->setImageLayout(images[0]->clearCmdBuffer.cmdBuffers[0], imagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, 0, images[0]->mipLevels);

		VkImageSubresourceRange subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS };
		VkClearColorValue clearWith;
		if (clearColor.useFloat)
			clearWith = { clearColor.floatRgb.x, clearColor.floatRgb.y, clearColor.floatRgb.z, clearColor.floatAlpha };
		else
		{
			clearWith.uint32[0] = (uint32_t)clearColor.uintR;
			clearWith.uint32[1] = (uint32_t)clearColor.uintG;
			clearWith.uint32[2] = (uint32_t)clearColor.uintB;
			clearWith.uint32[3] = (uint32_t)clearColor.uintA;
		}
		for (ImageClass * curImg : images)
			vkCmdClearColorImage(images[0]->clearCmdBuffer.cmdBuffers[0], curImg->image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &clearWith, 1, &subresourceRange);

		images[0]->setImageLayout(images[0]->clearCmdBuffer.cmdBuffers[0], imagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, 1, 0, images[0]->mipLevels);

		try { images[0]->clearCmdBuffer.EndCommandBuffer(); }
		catch (...) { throw std::runtime_error("Could not end setup cmd buffer for clear color"); }

		images[0]->recordedClearCmdBuffer = true;
	}

	try { images[0]->clearCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { throw std::runtime_error("Could not submit setup cmd buffer for clear color"); }
}

void HIGHOMEGA::GL::ImageClass::CopyImages(std::vector <ImageClass *> & copySource, std::vector <ImageClass *> & copyTarget)
{
	copySource[0]->mipLevels = 1;

	if (!copySource[0]->recordedCopyCmdBuffer)
	{
		try { copySource[0]->copyCmdBuffer.BeginCommandBuffer(*(copySource[0]->cachedInstance)); }
		catch (...) { throw std::runtime_error("Could not begin setup cmd buffer for image to image copy"); }

		std::vector <VkImage *> sourceImagesInFrontOfBarrier;
		std::vector <VkImage *> targetImagesInFrontOfBarrier;
		for (int i = 0; i != copySource.size(); i++)
		{
			sourceImagesInFrontOfBarrier.push_back(&copySource[i]->image);
			targetImagesInFrontOfBarrier.push_back(&copyTarget[i]->image);
		}

		copySource[0]->setImageLayout(copySource[0]->copyCmdBuffer.cmdBuffers[0], sourceImagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL, copySource[0]->layers, 0, copySource[0]->mipLevels);
		copySource[0]->setImageLayout(copySource[0]->copyCmdBuffer.cmdBuffers[0], targetImagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, copySource[0]->layers, 0, copySource[0]->mipLevels);

		for (int i = 0; i != copySource.size(); i++)
		{
			VkImageCopy copyImageStruct;
			copyImageStruct.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			copyImageStruct.srcSubresource.baseArrayLayer = 0;
			copyImageStruct.srcSubresource.layerCount = copySource[i]->layers;
			copyImageStruct.srcSubresource.mipLevel = 0;
			copyImageStruct.srcOffset = { 0, 0, 0 };
			copyImageStruct.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
			copyImageStruct.dstSubresource.baseArrayLayer = 0;
			copyImageStruct.dstSubresource.layerCount = copySource[i]->layers;
			copyImageStruct.dstSubresource.mipLevel = 0;
			copyImageStruct.dstOffset = { 0, 0, 0 };
			copyImageStruct.extent.width = copySource[i]->width;
			copyImageStruct.extent.height = copySource[i]->height;
			copyImageStruct.extent.depth = copySource[i]->depth;
			vkCmdCopyImage(copySource[0]->copyCmdBuffer.cmdBuffers[0], copySource[i]->image, VK_IMAGE_LAYOUT_GENERAL, copyTarget[i]->image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyImageStruct);
		}

		copySource[0]->setImageLayout(copySource[0]->copyCmdBuffer.cmdBuffers[0], targetImagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, copySource[0]->layers, 0, copySource[0]->mipLevels);

		try { copySource[0]->copyCmdBuffer.EndCommandBuffer(); }
		catch (...) { throw std::runtime_error("Could not end setup cmd buffer for image to image copy"); }

		copySource[0]->recordedCopyCmdBuffer = true;
	}

	try { copySource[0]->copyCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { throw std::runtime_error("Could not submit setup cmd buffer for image to image copy"); }
}

void HIGHOMEGA::GL::ImageClass::DownloadData()
{
	if (cachedInstance == nullptr) throw std::runtime_error("We do not have a pointer to the Vulkan instance for image download");

	unsigned int downloadBufferSize = width * height * depth * layers * FormatSize(format);
	if (!loadedDataBuffer) loadedDataBuffer = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO | USAGE_DST, Instance, nullptr, downloadBufferSize);

	if (!recordedDownloadCmdBuffer)
	{
		std::vector<VkBufferImageCopy> imageCopyRegions;

		VkBufferImageCopy imageCopyRegion = {};
		imageCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageCopyRegion.imageSubresource.mipLevel = 0;
		imageCopyRegion.imageSubresource.baseArrayLayer = 0;
		imageCopyRegion.imageSubresource.layerCount = layers;
		imageCopyRegion.imageExtent.width = width;
		imageCopyRegion.imageExtent.height = height;
		imageCopyRegion.imageExtent.depth = depth;
		imageCopyRegion.bufferOffset = 0;

		imageCopyRegions.push_back(imageCopyRegion);

		try { downloadCmdBuffer.BeginCommandBuffer(*cachedInstance); }
		catch (...) { throw std::runtime_error("Could not begin setup cmd buffer for copy to buffer"); }

		std::vector <VkImage *> imageInFrontOfBarrier;
		imageInFrontOfBarrier.push_back(&image);

		setImageLayout(downloadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, 0, mipLevels);

		vkCmdCopyImageToBuffer(downloadCmdBuffer.cmdBuffers[0], image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, loadedDataBuffer->buffer, (uint32_t)imageCopyRegions.size(), imageCopyRegions.data());

		setImageLayout(downloadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, layers, 0, mipLevels);

		try { downloadCmdBuffer.EndCommandBuffer(); }
		catch (...) { throw std::runtime_error("Could not end setup cmd buffer for copy to buffer"); }

		recordedDownloadCmdBuffer = true;
	}

	try { downloadCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { throw std::runtime_error("Could not submit setup cmd buffer for copy to buffer"); }

	if (!downloadData) downloadData = new unsigned char[loadedDataBuffer->getSize()];
	loadedDataBuffer->DownloadSubData(0, downloadData, loadedDataBuffer->getSize());
}

void HIGHOMEGA::GL::ImageClass::FreeLoadedData()
{
	if (loadedDataBuffer) delete loadedDataBuffer;
	if (downloadData) delete[] downloadData;
	loadedDataBuffer = nullptr;
	downloadData = nullptr;
}

unsigned char * HIGHOMEGA::GL::ImageClass::DownloadedData()
{
	return downloadData;
}

unsigned int HIGHOMEGA::GL::ImageClass::DownloadedDataSize()
{
	if (!loadedDataBuffer) return 0;
	return loadedDataBuffer->getSize();
}

void HIGHOMEGA::GL::ImageClass::UploadData(unsigned char* inData, unsigned int inDataSize)
{
	if (cachedInstance == nullptr) throw std::runtime_error("We do not have a pointer to the Vulkan instance for image upload");

	unsigned int imageDataSize = width * height * depth * layers * FormatSize(format);
	if (inDataSize > imageDataSize) throw std::runtime_error("Image too small for uploaded data");

	if (!loadedDataBuffer) loadedDataBuffer = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SRC, *cachedInstance, nullptr, imageDataSize);
	loadedDataBuffer->UploadSubData(0, inData, inDataSize);

	if (!recordedUploadCmdBuffer)
	{
		std::vector<VkBufferImageCopy> imageCopyRegions;

		VkBufferImageCopy imageCopyRegion = {};
		imageCopyRegion.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageCopyRegion.imageSubresource.mipLevel = 0;
		imageCopyRegion.imageSubresource.baseArrayLayer = 0;
		imageCopyRegion.imageSubresource.layerCount = layers;
		imageCopyRegion.imageExtent.width = width;
		imageCopyRegion.imageExtent.height = height;
		imageCopyRegion.imageExtent.depth = depth;
		imageCopyRegion.bufferOffset = 0;

		imageCopyRegions.push_back(imageCopyRegion);

		try { uploadCmdBuffer.BeginCommandBuffer(*cachedInstance); }
		catch (...) { throw std::runtime_error("Could not begin setup cmd buffer for copy to image"); }

		std::vector <VkImage*> imageInFrontOfBarrier;
		imageInFrontOfBarrier.push_back(&image);

		setImageLayout(uploadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layers, 0, mipLevels);

		vkCmdCopyBufferToImage(uploadCmdBuffer.cmdBuffers[0], loadedDataBuffer->buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, (uint32_t)imageCopyRegions.size(), imageCopyRegions.data());

		setImageLayout(uploadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_GENERAL, layers, 0, mipLevels);

		try { uploadCmdBuffer.EndCommandBuffer(); }
		catch (...) { throw std::runtime_error("Could not end setup cmd buffer for copy to image"); }

		recordedUploadCmdBuffer = true;
	}

	try { uploadCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { throw std::runtime_error("Could not submit setup cmd buffer for copy to image"); }
}

int HIGHOMEGA::GL::ImageClass::getWidth()
{
	return width;
}

int HIGHOMEGA::GL::ImageClass::getHeight()
{
	return height;
}

int HIGHOMEGA::GL::ImageClass::getBPP()
{
	return bpp;
}

int HIGHOMEGA::GL::ImageClass::getDepth()
{
	return depth;
}

int HIGHOMEGA::GL::ImageClass::getLayers()
{
	return layers;
}

bool HIGHOMEGA::GL::ImageClass::getIs3D()
{
	return is3D;
}

bool HIGHOMEGA::GL::ImageClass::getIsArray()
{
	return isArray;
}

HIGHOMEGA::GL::ImageClass::~ImageClass()
{
	RemovePast();
}

void HIGHOMEGA::GL::FramebufferClass::RemovePast(int numFrameBuffersToDestroy)
{
	if (cachedInstance == nullptr)
	{
		if (!instanceEverSet) return;
		throw std::runtime_error("We do not have a pointer to the Vulkan instance");
	}

	if (haveRenderPass) vkDestroyRenderPass(cachedInstance->device, renderPass, nullptr);
	for (int i = 0; i != numFrameBuffersToDestroy; i++)
		vkDestroyFramebuffer(cachedInstance->device, swapChainBuffers[i], nullptr);
	swapChainBuffers.clear();
	attachmentsSampler.RemovePast();
	if (haveAttachmentsFrameBuffer) vkDestroyFramebuffer(cachedInstance->device, attachmentsFrameBuffer, nullptr);

	haveRenderPass = false;
	haveAttachmentsFrameBuffer = false;
}

void HIGHOMEGA::GL::FramebufferClass::RemovePast()
{
	RemovePast((int)swapChainBuffers.size());
}

HIGHOMEGA::GL::FramebufferClass::FramebufferClass()
{
	haveRenderPass = false;
	haveAttachmentsFrameBuffer = false;
	instanceEverSet = false;

	depthStencilAttachment = nullptr;
}

void HIGHOMEGA::GL::FramebufferClass::AddColorAttachment(ImageClass & inAttach)
{
	colorAttachments.push_back(&inAttach);
	colorAttachmentLayers.push_back(0);
}

void HIGHOMEGA::GL::FramebufferClass::AddColorAttachmentWithLayer(ImageClass & inAttach, unsigned int layer)
{
	colorAttachments.push_back(&inAttach);
	colorAttachmentLayers.push_back(layer + 1);
}

void HIGHOMEGA::GL::FramebufferClass::SetDepthStencil(ImageClass & inAttach)
{
	depthStencilAttachment = &inAttach;
}

ImageClass* HIGHOMEGA::GL::FramebufferClass::GetDepthStencil()
{
	return depthStencilAttachment;
}

ImageClass & HIGHOMEGA::GL::FramebufferClass::GetSampler()
{
	return attachmentsSampler;
}

void HIGHOMEGA::GL::FramebufferClass::Create(RENDER_MODE inpMode, InstanceClass & inpInstance, WindowClass & windowRef)
{
	cachedInstance = &inpInstance;
	instanceEverSet = true;
	mode = inpMode;

	std::vector<VkAttachmentReference> colorReference;
	std::vector<VkAttachmentDescription> attachmentsDesc;

	if (mode == ON_SCREEN)
	{
		VkAttachmentDescription curAttach;
		curAttach.format = inpInstance.colorFormat;
		curAttach.samples = VK_SAMPLE_COUNT_1_BIT;
		curAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		curAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		curAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		curAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		curAttach.initialLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		curAttach.finalLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		curAttach.flags = VK_ATTACHMENT_DESCRIPTION_MAY_ALIAS_BIT;
		attachmentsDesc.push_back(curAttach);

		VkAttachmentReference curRef = {};
		curRef.attachment = 0;
		curRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
		colorReference.push_back(curRef);
	}
	else
	{
		for (int i = 0; i != colorAttachments.size(); i++)
		{
			VkAttachmentDescription curAttach;
			curAttach.format = (VkFormat)colorAttachments[i]->format;
			curAttach.samples = VK_SAMPLE_COUNT_1_BIT;
			curAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
			curAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			curAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			curAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			curAttach.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
			curAttach.finalLayout = VK_IMAGE_LAYOUT_GENERAL;
			curAttach.flags = VK_ATTACHMENT_DESCRIPTION_MAY_ALIAS_BIT;
			attachmentsDesc.push_back(curAttach);

			VkAttachmentReference curRef = {};
			curRef.attachment = i;
			curRef.layout = VK_IMAGE_LAYOUT_GENERAL;
			colorReference.push_back(curRef);
		}
	}

	if (depthStencilAttachment)
	{
		VkAttachmentDescription depthStencilAttach;
		depthStencilAttach.format = inpInstance.selectedDepthFormat;
		depthStencilAttach.samples = VK_SAMPLE_COUNT_1_BIT;
		depthStencilAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
		depthStencilAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		depthStencilAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		depthStencilAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthStencilAttach.initialLayout = mode == ON_SCREEN ? (depthStencilAttachment->haveSampler ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) : VK_IMAGE_LAYOUT_UNDEFINED;
		depthStencilAttach.finalLayout = (depthStencilAttachment->haveSampler ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
		depthStencilAttach.flags = VK_ATTACHMENT_DESCRIPTION_MAY_ALIAS_BIT;
		attachmentsDesc.push_back(depthStencilAttach);
	}

	VkAttachmentReference depthReference = {};
	if (depthStencilAttachment)
	{
		depthReference.attachment = (uint32_t)(attachmentsDesc.size() - 1);
		depthReference.layout = (depthStencilAttachment->haveSampler ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
	}

	VkSubpassDescription subpass = {};
	subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpass.flags = 0;
	subpass.inputAttachmentCount = 0;
	subpass.pInputAttachments = VK_NULL_HANDLE;
	if (colorReference.size() == 0)
	{
		subpass.colorAttachmentCount = 0;
		subpass.pColorAttachments = VK_NULL_HANDLE;
	}
	else
	{
		subpass.colorAttachmentCount = (uint32_t)colorReference.size();
		subpass.pColorAttachments = colorReference.data();
	}
	subpass.pResolveAttachments = VK_NULL_HANDLE;
	if (depthStencilAttachment)
	{
		subpass.pDepthStencilAttachment = &depthReference;
	}
	else
	{
		subpass.pDepthStencilAttachment = VK_NULL_HANDLE;
	}
	subpass.preserveAttachmentCount = 0;
	subpass.pPreserveAttachments = VK_NULL_HANDLE;

	VkSubpassDependency dependencies[2];
	if (mode == OFF_SCREEN)
	{
		dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[0].dstSubpass = 0;
		dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

		dependencies[1].srcSubpass = 0;
		dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
		dependencies[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
		dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
		dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT | VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
		dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
		dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;
	}

	VkRenderPassCreateInfo renderPassInfo = {};
	renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassInfo.pNext = VK_NULL_HANDLE;
	renderPassInfo.attachmentCount = (uint32_t)attachmentsDesc.size();
	renderPassInfo.pAttachments = attachmentsDesc.data();
	renderPassInfo.subpassCount = 1;
	renderPassInfo.pSubpasses = &subpass;
	renderPassInfo.dependencyCount = (mode == OFF_SCREEN) ? 2 : 0;
	renderPassInfo.pDependencies = (mode == OFF_SCREEN) ? dependencies : VK_NULL_HANDLE;

	VkResult result = vkCreateRenderPass(cachedInstance->device, &renderPassInfo, nullptr, &renderPass);

	if (result != VK_SUCCESS) { throw std::runtime_error("Could not create a render pass"); }
	haveRenderPass = true;

	attachmentsSampler.cachedInstance = &inpInstance;

	if (mode == ON_SCREEN)
	{
		VkImageView attachmentsViews[2];

		attachmentsViews[1] = depthStencilAttachment->view;
		VkFramebufferCreateInfo frameBufferCreateInfo = {};
		frameBufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		frameBufferCreateInfo.pNext = VK_NULL_HANDLE;
		frameBufferCreateInfo.renderPass = renderPass;
		frameBufferCreateInfo.attachmentCount = 2;
		frameBufferCreateInfo.pAttachments = attachmentsViews;
		frameBufferCreateInfo.width = windowRef.w;
		frameBufferCreateInfo.height = windowRef.h;
		frameBufferCreateInfo.layers = 1;

		width = windowRef.w;
		height = windowRef.h;

		swapChainBuffers.resize(colorAttachments.size());
		for (uint32_t i = 0; i < swapChainBuffers.size(); i++)
		{
			attachmentsViews[0] = colorAttachments[i]->view;
			result = vkCreateFramebuffer(cachedInstance->device, &frameBufferCreateInfo, nullptr, &swapChainBuffers[i]);
			if (result != VK_SUCCESS) { RemovePast(i); throw std::runtime_error("Could not create a frame buffer"); }
		}
	}
	else
	{
		std::vector<VkImageView> attachmentsViews;
		for (int i = 0; i != colorAttachments.size(); i++)
		{
			if (colorAttachmentLayers[i] == 0)
				attachmentsViews.push_back(colorAttachments[i]->view);
			else
				attachmentsViews.push_back(colorAttachments[i]->layerView[colorAttachmentLayers[i] - 1]);
		}

		if (depthStencilAttachment) attachmentsViews.push_back(depthStencilAttachment->view);

		if (colorAttachments.size() > 0)
		{
			width = colorAttachments[0]->width;
			height = colorAttachments[0]->height;
		}
		else
		{
			if (depthStencilAttachment)
			{
				width = depthStencilAttachment->width;
				height = depthStencilAttachment->height;
			}
		}

		VkFramebufferCreateInfo fbufCreateInfo = {};
		fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		fbufCreateInfo.pNext = NULL;
		fbufCreateInfo.renderPass = renderPass;
		if (attachmentsViews.size() == 0)
		{
			fbufCreateInfo.pAttachments = VK_NULL_HANDLE;
			fbufCreateInfo.attachmentCount = 0;
		}
		else
		{
			fbufCreateInfo.pAttachments = attachmentsViews.data();
			fbufCreateInfo.attachmentCount = (uint32_t)attachmentsViews.size();
		}
		fbufCreateInfo.width = width;
		fbufCreateInfo.height = height;
		fbufCreateInfo.layers = 1;

		result = vkCreateFramebuffer(cachedInstance->device, &fbufCreateInfo, nullptr, &attachmentsFrameBuffer);
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create the framebuffer"); }
		haveAttachmentsFrameBuffer = true;

		try {
			attachmentsSampler.CreateSampler(ImageClass::MIN_MAG_FILTER::MIN_MAG_NEAREST, ImageClass::MIN_MAG_FILTER::MIN_MAG_NEAREST,
				ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
				ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE, ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE, ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE,
				0.0f, 0.0f, 1.0f, false, 1.0f);
		}
		catch (...) {
			RemovePast(); throw std::runtime_error("Could not create the attachment sampler");
		}
	}
}

void HIGHOMEGA::GL::FramebufferClass::setWidth(int width)
{
	this->width = width;
}

void HIGHOMEGA::GL::FramebufferClass::setHeight(int height)
{
	this->height = height;
}

RENDER_MODE HIGHOMEGA::GL::FramebufferClass::getRenderMode()
{
	return mode;
}

int HIGHOMEGA::GL::FramebufferClass::getWidth()
{
	return width;
}

int HIGHOMEGA::GL::FramebufferClass::getHeight()
{
	return height;
}

HIGHOMEGA::GL::FramebufferClass::~FramebufferClass()
{
	RemovePast();
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::RemovePast()
{
	if (!ptrToInstance) return;

	if (hasAccelStruct) RTInstance::fpDestroyAccelerationStructureKHR(ptrToInstance->device, accelStruct, nullptr);

	hasAccelStruct = false;
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::CreateAccelStruct(bool isBlas, VkAccelerationStructureGeometryKHR * inpGeom, VkAccelerationStructureBuildRangeInfoKHR * inpGeomOffset, std::vector <VkAccelerationStructureInstanceKHR> * instanceData, bool inpImmutable, InstanceClass & inpInstance, blasBuildParams *inpParams)
{
	ptrToInstance = &inpInstance;

	if (isBlas)
	{
		VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeomInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
		accelerationStructureBuildGeomInfo.flags = inpImmutable ? VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR : (VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR);
		accelerationStructureBuildGeomInfo.pNext = VK_NULL_HANDLE;
		accelerationStructureBuildGeomInfo.geometryCount = 1;
		accelerationStructureBuildGeomInfo.pGeometries = inpGeom;
		accelerationStructureBuildGeomInfo.ppGeometries = VK_NULL_HANDLE;
		accelerationStructureBuildGeomInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
		accelerationStructureBuildGeomInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		accelerationStructureBuildGeomInfo.srcAccelerationStructure = VK_NULL_HANDLE;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = VK_NULL_HANDLE;

		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
		sizeInfo.pNext = VK_NULL_HANDLE;
		RTInstance::fpGetAccelerationStructureBuildSizesKHR(Instance.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &accelerationStructureBuildGeomInfo, &inpGeomOffset->primitiveCount, &sizeInfo);

		VkAccelerationStructureCreateInfoKHR createInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
		createInfo.pNext = VK_NULL_HANDLE;
		createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		createInfo.size = sizeInfo.accelerationStructureSize;

		accelStructBuffer.Buffer(MEMORY_DEVICE_LOCAL, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY, *ptrToInstance, nullptr, (unsigned int)createInfo.size);
		createInfo.buffer = accelStructBuffer.buffer;

		VkResult result = RTInstance::fpCreateAccelerationStructureKHR(Instance.device, &createInfo, nullptr, &accelStruct);
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create accel struct"); }
		hasAccelStruct = true;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_SSBO | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.pNext = VK_NULL_HANDLE;
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		if (!inpParams)
		{
			BeginCommandBuffer(*ptrToInstance);

			RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[0], 1, &accelerationStructureBuildGeomInfo, &inpGeomOffset);

			EndCommandBuffer();
			SubmitCommandBuffer();
		}
		else
		{
			inpParams->blasBuildRanges.push_back(inpGeomOffset);
			inpParams->blasBuildInfos.push_back(accelerationStructureBuildGeomInfo);
		}
	}
	else
	{
		unsigned int curInstanceBufSize = (unsigned int)instanceData->size() * sizeof(VkAccelerationStructureInstanceKHR);

		if (instanceBuffer.getSize() < curInstanceBufSize)
		{
			instanceBuffer.Buffer(MEMORY_HOST_VISIBLE | MEMORY_HOST_COHERENT, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_SSBO | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY, *ptrToInstance, instanceData->data(), curInstanceBufSize);
		}

		VkBufferDeviceAddressInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		bufferInfo.buffer = instanceBuffer.buffer;
		VkDeviceAddress instanceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufferInfo);

		BeginCommandBuffer(*ptrToInstance);

		VkAccelerationStructureGeometryInstancesDataKHR instancesVk{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
		instancesVk.arrayOfPointers = VK_FALSE;
		instancesVk.data.deviceAddress = instanceAddress;

		VkAccelerationStructureGeometryKHR topASGeometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
		topASGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
		topASGeometry.geometry.instances = instancesVk;

		VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeomInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
		accelerationStructureBuildGeomInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
		accelerationStructureBuildGeomInfo.geometryCount = 1;
		accelerationStructureBuildGeomInfo.pGeometries = &topASGeometry;
		accelerationStructureBuildGeomInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
		accelerationStructureBuildGeomInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		accelerationStructureBuildGeomInfo.srcAccelerationStructure = VK_NULL_HANDLE;
		uint32_t instanceCount = (uint32_t)instanceData->size();
		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
		RTInstance::fpGetAccelerationStructureBuildSizesKHR(Instance.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &accelerationStructureBuildGeomInfo, &instanceCount, &sizeInfo);

		VkAccelerationStructureCreateInfoKHR createInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
		createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		createInfo.size = sizeInfo.accelerationStructureSize;

		accelStructBuffer.Buffer(MEMORY_DEVICE_LOCAL, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, nullptr, (unsigned int)createInfo.size);
		createInfo.buffer = accelStructBuffer.buffer;

		VkResult result = RTInstance::fpCreateAccelerationStructureKHR(Instance.device, &createInfo, nullptr, &accelStruct);
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create accel struct"); }
		hasAccelStruct = true;
		accelerationStructureBuildGeomInfo.srcAccelerationStructure = VK_NULL_HANDLE;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_SSBO, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		VkAccelerationStructureBuildRangeInfoKHR buildOffsetInfo{ static_cast<uint32_t>(instanceCount), 0, 0, 0 };
		const VkAccelerationStructureBuildRangeInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;
		RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[0], 1, &accelerationStructureBuildGeomInfo, &pBuildOffsetInfo);

		EndCommandBuffer();
		SubmitCommandBuffer();
	}
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::UpdateAccelStruct(bool isBlas, VkAccelerationStructureGeometryKHR * inpGeom, VkAccelerationStructureBuildRangeInfoKHR * inpGeomOffset, std::vector <VkAccelerationStructureInstanceKHR> * instanceData, blasBuildParams *inpParams)
{
	if (!ptrToInstance) throw std::runtime_error("Cannot submit create accel struct request since a ptr to instance was not found");

	if (isBlas)
	{
		VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeomInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
		accelerationStructureBuildGeomInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
		accelerationStructureBuildGeomInfo.geometryCount = 1;
		accelerationStructureBuildGeomInfo.pGeometries = inpGeom;
		accelerationStructureBuildGeomInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
		accelerationStructureBuildGeomInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		accelerationStructureBuildGeomInfo.srcAccelerationStructure = accelStruct;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
		RTInstance::fpGetAccelerationStructureBuildSizesKHR(ptrToInstance->device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &accelerationStructureBuildGeomInfo, &inpGeomOffset->primitiveCount, &sizeInfo);

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		if (!inpParams)
		{
			BeginCommandBuffer(*ptrToInstance);
			RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[0], 1, &accelerationStructureBuildGeomInfo, &inpGeomOffset);
			EndCommandBuffer();
			SubmitCommandBuffer();
		}
		else
		{
			inpParams->blasBuildRanges.push_back(inpGeomOffset);
			inpParams->blasBuildInfos.push_back(accelerationStructureBuildGeomInfo);
		}
	}
	else
	{
		unsigned int curInstanceBufSize = (unsigned int)instanceData->size() * sizeof(VkAccelerationStructureInstanceKHR);

		if (instanceBuffer.getSize() < curInstanceBufSize)
		{
			instanceBuffer.Buffer(MEMORY_HOST_VISIBLE | MEMORY_HOST_COHERENT, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, instanceData->data(), curInstanceBufSize);
		}

		VkBufferDeviceAddressInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		bufferInfo.buffer = instanceBuffer.buffer;
		VkDeviceAddress instanceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufferInfo);

		BeginCommandBuffer(*ptrToInstance);

		VkAccelerationStructureGeometryInstancesDataKHR instancesVk{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
		instancesVk.arrayOfPointers = VK_FALSE;
		instancesVk.data.deviceAddress = instanceAddress;

		VkAccelerationStructureGeometryKHR topASGeometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
		topASGeometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
		topASGeometry.geometry.instances = instancesVk;

		VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeomInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
		accelerationStructureBuildGeomInfo.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR;
		accelerationStructureBuildGeomInfo.geometryCount = 1;
		accelerationStructureBuildGeomInfo.pGeometries = &topASGeometry;
		accelerationStructureBuildGeomInfo.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_UPDATE_KHR;
		accelerationStructureBuildGeomInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		accelerationStructureBuildGeomInfo.srcAccelerationStructure = VK_NULL_HANDLE;
		uint32_t instanceCount = (uint32_t)instanceData->size();
		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
		RTInstance::fpGetAccelerationStructureBuildSizesKHR(Instance.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &accelerationStructureBuildGeomInfo, &instanceCount, &sizeInfo);

		accelerationStructureBuildGeomInfo.srcAccelerationStructure = accelStruct;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		VkAccelerationStructureBuildRangeInfoKHR buildOffsetInfo{ static_cast<uint32_t>(instanceCount), 0, 0, 0 };
		const VkAccelerationStructureBuildRangeInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;
		RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[0], 1, &accelerationStructureBuildGeomInfo, &pBuildOffsetInfo);

		EndCommandBuffer();
		SubmitCommandBuffer();
	}
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::FetchBlasAddress()
{
	if (!ptrToInstance) throw std::runtime_error("Cannot fetch blas address since a ptr to instance was not found");

	VkAccelerationStructureDeviceAddressInfoKHR addressInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
	addressInfo.accelerationStructure = accelStruct;
	blasAddress = RTInstance::fpGetAccelerationStructureDeviceAddressKHR(ptrToInstance->device, &addressInfo);
}

HIGHOMEGA::GL::KHR_RT::RTAccelStruct::RTAccelStruct()
{
	hasAccelStruct = false;
	ptrToInstance = nullptr;
}

HIGHOMEGA::GL::KHR_RT::RTAccelStruct::~RTAccelStruct()
{
	RemovePast();
}

HIGHOMEGA::GL::KHR_RT::RTGeometry::RTGeometry()
{
	ptrToInstance = nullptr;
	created = false;
	dirty = false;
}

HIGHOMEGA::GL::KHR_RT::RTGeometry::~RTGeometry()
{
	ptrToInstance = nullptr;
	created = false;
	dirty = false;
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::SetGeom(BufferClass & vertBuffer, unsigned int vertexSize, bool isAlphaKeyed, bool inpImmutable, InstanceClass & inpInstance)
{
	if (vertBuffer.getSize() == 0) return;

	immutable = inpImmutable;
	completeVertexBufferRef = &vertBuffer;

	ptrToInstance = &inpInstance;

	VkBufferDeviceAddressInfo bufDevAdInfo;
	bufDevAdInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	bufDevAdInfo.pNext = VK_NULL_HANDLE;
	bufDevAdInfo.buffer = vertBuffer.buffer;
	bufferDeviceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufDevAdInfo);

	traceGeom = {};
	traceGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	traceGeom.pNext = nullptr;
	traceGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
	traceGeom.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
	traceGeom.geometry.triangles.pNext = nullptr;
	traceGeom.geometry.triangles.vertexFormat = (VkFormat)R32G32B32F;
	traceGeom.geometry.triangles.vertexData.deviceAddress = bufferDeviceAddress;
	traceGeom.geometry.triangles.vertexStride = vertexSize;
	traceGeom.geometry.triangles.maxVertex = vertBuffer.getSize() / vertexSize;
	traceGeom.geometry.triangles.indexData.deviceAddress = (VkDeviceAddress)0;
	traceGeom.geometry.triangles.indexType = VK_INDEX_TYPE_NONE_KHR;
	traceGeom.geometry.triangles.transformData.deviceAddress = (VkDeviceAddress)0;
	if (!isAlphaKeyed) traceGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

	traceGeomOffset = {};
	traceGeomOffset.firstVertex = 0;
	traceGeomOffset.primitiveCount = vertBuffer.getSize() / (vertexSize * 3);
	traceGeomOffset.primitiveOffset = 0;
	traceGeomOffset.transformOffset = 0;

	dirty = true;
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::CreateOrUpdate(blasBuildParams *inpParams)
{
	if (!created)
	{
		CreateAccelStruct(true, &traceGeom, &traceGeomOffset, nullptr, immutable, *ptrToInstance, inpParams);
		created = true;
		dirty = false;
	}
	if (dirty && !immutable)
	{
		UpdateAccelStruct(true, &traceGeom, &traceGeomOffset, nullptr, inpParams);
		dirty = false;
	}
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::SetDirty()
{
	if (immutable) return;
	// This should be called when geometry changes inside a shader...
	dirty = true;
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::SetMask(unsigned char inpMask)
{
	rayMask = inpMask;
}

unsigned int HIGHOMEGA::GL::KHR_RT::RTGeometry::GetMask()
{
	return rayMask;
}

HIGHOMEGA::GL::InstanceClass *HIGHOMEGA::GL::DescriptorSetLayout::ptrToInstance = nullptr;

void HIGHOMEGA::GL::DescriptorSetLayout::RemovePast()
{
	if (!ptrToInstance) return;

	for (VkDescriptorSetLayout & curDescSetLayout : descriptorSetLayouts)
		vkDestroyDescriptorSetLayout(ptrToInstance->device, curDescSetLayout, nullptr);
}

HIGHOMEGA::GL::DescriptorSetLayout::DescriptorSetLayout()
{
}

void HIGHOMEGA::GL::DescriptorSetLayout::CreateDescriptorSetLayout(std::vector<ShaderResource>& allResources, InstanceClass * inpPtrToInstance)
{
	if (!ptrToInstance) ptrToInstance = inpPtrToInstance;

	for (ShaderResource & curRes : allResources)
		if (std::find(allSets.begin(), allSets.end(), curRes.setId) == allSets.end())
			allSets.push_back(curRes.setId);

	std::sort(allSets.begin(), allSets.end());

	VkResult result;

	descriptorSetLayouts.reserve(allSets.size());
	std::vector<VkDescriptorSetLayoutBinding> layoutBindings;
	std::vector<VkDescriptorBindingFlagsEXT> setBindingFlags;
	layoutBindings.reserve(allResources.size());
	setBindingFlags.reserve(allResources.size());
	for (unsigned int & i : allSets)
	{
		for (ShaderResource & curRes : allResources)
		{
			if (curRes.setId != i) continue;
			VkDescriptorSetLayoutBinding layoutBinding = {};
			layoutBinding.descriptorType = (VkDescriptorType)curRes.type;
			layoutBinding.descriptorCount = curRes.arrayedResource.size() > 0 ? (curRes.isVariableCount ? 100000 : (unsigned int)curRes.arrayedResource.size()) : 1;
			layoutBinding.stageFlags = (VkShaderStageFlags)curRes.visibility;
			layoutBinding.binding = curRes.bindId;
			layoutBindings.push_back(layoutBinding);
			setBindingFlags.push_back(curRes.isVariableCount ? VK_DESCRIPTOR_BINDING_VARIABLE_DESCRIPTOR_COUNT_BIT_EXT : 0);
		}

		VkDescriptorSetLayoutBindingFlagsCreateInfoEXT bindingFlags;
		bindingFlags.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO_EXT;
		bindingFlags.pNext = nullptr;
		bindingFlags.pBindingFlags = setBindingFlags.data();
		bindingFlags.bindingCount = (uint32_t)setBindingFlags.size();

		VkDescriptorSetLayoutCreateInfo descriptorLayout = {};
		descriptorLayout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
		descriptorLayout.pNext = &bindingFlags;
		descriptorLayout.bindingCount = (uint32_t)layoutBindings.size();
		descriptorLayout.pBindings = layoutBindings.data();

		VkDescriptorSetLayout curDescriptorSetLayout;

		result = vkCreateDescriptorSetLayout(ptrToInstance->device, &descriptorLayout, VK_NULL_HANDLE, &curDescriptorSetLayout);
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create descriptor set layout"); }

		descriptorSetLayouts.push_back(curDescriptorSetLayout);
		layoutBindings.clear();
		setBindingFlags.clear();
	}
}

HIGHOMEGA::GL::DescriptorSetLayout::~DescriptorSetLayout()
{
	RemovePast();
}

HIGHOMEGA::GL::InstanceClass *HIGHOMEGA::GL::DescriptorSets::ptrToInstance = nullptr;
ThreadLocalCache <std::vector<VkDescriptorPool>> HIGHOMEGA::GL::DescriptorSets::descriptorPools;

void HIGHOMEGA::GL::DescriptorSets::RemovePast()
{
	if (!ptrToDescSetLayout) return;

	for (VkDescriptorSet & curDescSet : descriptorSets)
		vkFreeDescriptorSets(ptrToDescSetLayout->ptrToInstance->device, poolObject, 1, &curDescSet);

	{std::lock_guard<std::mutex> lk(descriptorPools.mtx);
	descriptorPools.dir[descriptorPoolPtr->keyRef].elemCount--;
	if (descriptorPools.dir[descriptorPoolPtr->keyRef].elemCount == 0)
	{
		for (VkDescriptorPool & curPool : descriptorPools.dir[descriptorPoolPtr->keyRef].elem)
			vkDestroyDescriptorPool(ptrToInstance->device, curPool, nullptr);
		descriptorPools.dir[descriptorPoolPtr->keyRef].elem.clear();
		descriptorPools.dir.erase(descriptorPoolPtr->keyRef);
	}}
}

HIGHOMEGA::GL::DescriptorSets::DescriptorSets()
{
}

void HIGHOMEGA::GL::DescriptorSets::Make(DescriptorSetLayout * inpPtrToDescSetLayout)
{
	ptrToDescSetLayout = inpPtrToDescSetLayout;
	ptrToInstance = ptrToDescSetLayout->ptrToInstance;
}

void HIGHOMEGA::GL::DescriptorSets::WriteDescriptorSets(std::vector<ShaderResource>& allResources)
{
	if (!ptrToDescSetLayout) throw std::runtime_error("Attempting to write descriptor sets without a desc set layout");

	// Requested resource statistics
	unsigned int uboTypeCount = 0, ssboTypeCount = 0, samplerTypeCount = 0, imageStoreCount = 0, accelStructCount = 0;
	for (ShaderResource & curRes : allResources)
		switch (curRes.type)
		{
		case RESOURCE_UBO:
			uboTypeCount += max((unsigned int)curRes.arrayedResource.size(), 1);
			break;
		case RESOURCE_SSBO:
			ssboTypeCount += max((unsigned int)curRes.arrayedResource.size(), 1);
			break;
		case RESOURCE_SAMPLER:
			samplerTypeCount += max((unsigned int)curRes.arrayedResource.size(), 1);
			break;
		case RESOURCE_IMAGE_STORE:
			imageStoreCount += max((unsigned int)curRes.arrayedResource.size(), 1);
			break;
		case RESOURCE_RT_ACCEL_STRUCT:
			accelStructCount += max((unsigned int)curRes.arrayedResource.size(), 1);
			break;
		}
	std::vector<VkDescriptorPoolSize> typeCounts;
	if (uboTypeCount > 0)
	{
		typeCounts.emplace_back();
		typeCounts.back().type = (VkDescriptorType)RESOURCE_UBO;
		typeCounts.back().descriptorCount = uboTypeCount * 5;
	}
	if (ssboTypeCount > 0)
	{
		typeCounts.emplace_back();
		typeCounts.back().type = (VkDescriptorType)RESOURCE_SSBO;
		typeCounts.back().descriptorCount = ssboTypeCount * 5;
	}
	if (samplerTypeCount > 0)
	{
		typeCounts.emplace_back();
		typeCounts.back().type = (VkDescriptorType)RESOURCE_SAMPLER;
		typeCounts.back().descriptorCount = samplerTypeCount * 5;
	}
	if (imageStoreCount > 0)
	{
		typeCounts.emplace_back();
		typeCounts.back().type = (VkDescriptorType)RESOURCE_IMAGE_STORE;
		typeCounts.back().descriptorCount = imageStoreCount * 5;
	}
	if (accelStructCount > 0)
	{
		typeCounts.emplace_back();
		typeCounts.back().type = (VkDescriptorType)RESOURCE_RT_ACCEL_STRUCT;
		typeCounts.back().descriptorCount = accelStructCount * 5;
	}

	VkDescriptorPoolCreateInfo descriptorPoolInfo = {};
	descriptorPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	descriptorPoolInfo.pNext = VK_NULL_HANDLE;
	descriptorPoolInfo.poolSizeCount = (uint32_t)typeCounts.size();
	descriptorPoolInfo.pPoolSizes = typeCounts.data();
	descriptorPoolInfo.maxSets = (unsigned int)ptrToDescSetLayout->allSets.size() * 5;
	descriptorPoolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;

	{std::lock_guard<std::mutex> lk(descriptorPools.mtx);
	if (descriptorPools.dir[ThreadID].elem.size() == 0)
	{
		descriptorPools.dir[ThreadID].elem.emplace_back();

		VkResult result = vkCreateDescriptorPool(ptrToInstance->device, &descriptorPoolInfo, nullptr, &descriptorPools.dir[ThreadID].elem.back());
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create descriptor pool"); }

		descriptorPools.dir[ThreadID].elemCount = 1;
		descriptorPools.dir[ThreadID].keyRef = ThreadID;
		writtenOnce = true;
	}
	else
	{
		if (!writtenOnce) { descriptorPools.dir[ThreadID].elemCount++; writtenOnce = true; }
	}
	descriptorPoolPtr = &descriptorPools.dir[ThreadID];}

	poolObject = descriptorPoolPtr->elem.back();

	std::vector<unsigned int> descCounts;
	descCounts.resize(ptrToDescSetLayout->allSets.size());
	for (ShaderResource & curRes : allResources)
		descCounts[curRes.setId] = curRes.isVariableCount ? (unsigned int)curRes.arrayedResource.size() : 1;
	VkDescriptorSetVariableDescriptorCountAllocateInfoEXT variableDescCountAllocInfo;
	variableDescCountAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_VARIABLE_DESCRIPTOR_COUNT_ALLOCATE_INFO;
	variableDescCountAllocInfo.pNext = VK_NULL_HANDLE;
	variableDescCountAllocInfo.descriptorSetCount = (uint32_t)descCounts.size();
	variableDescCountAllocInfo.pDescriptorCounts = (uint32_t *)descCounts.data();

	VkDescriptorSetAllocateInfo allocInfo = {};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = descriptorPoolPtr->elem.back();
	allocInfo.pNext = &variableDescCountAllocInfo;
	allocInfo.descriptorSetCount = (uint32_t)ptrToDescSetLayout->allSets.size();
	allocInfo.pSetLayouts = ptrToDescSetLayout->descriptorSetLayouts.data();

	descriptorSets.resize(ptrToDescSetLayout->descriptorSetLayouts.size());

	VkResult result = vkAllocateDescriptorSets(ptrToInstance->device, &allocInfo, descriptorSets.data());
	if (result != VK_SUCCESS)
	{
		{std::lock_guard<std::mutex> lk(descriptorPools.mtx);

		descriptorPools.dir[ThreadID].elem.emplace_back();

		VkResult result = vkCreateDescriptorPool(ptrToInstance->device, &descriptorPoolInfo, nullptr, &descriptorPools.dir[ThreadID].elem.back());
		if (result != VK_SUCCESS) { RemovePast(); throw std::runtime_error("Could not create descriptor pool"); }

		if (!writtenOnce) { descriptorPools.dir[ThreadID].elemCount++; writtenOnce = true; }
		descriptorPoolPtr = &descriptorPools.dir[ThreadID];}

		allocInfo.descriptorPool = descriptorPoolPtr->elem.back();
		poolObject = descriptorPoolPtr->elem.back();

		result = vkAllocateDescriptorSets(ptrToInstance->device, &allocInfo, descriptorSets.data());

		if (result != VK_SUCCESS) { descriptorSets.clear(); RemovePast(); throw std::runtime_error("Could not allocate descriptor sets"); }
	}

	std::vector<VkWriteDescriptorSet> writeDescriptorSets;
	std::vector<VkDescriptorImageInfo> descriptorImageInfos;
	std::vector<VkWriteDescriptorSetAccelerationStructureKHR> accelStructInfos;
	std::vector<VkDescriptorBufferInfo> descriptorBufferInfos;
	unsigned int accelStructInfosCount = 0;
	unsigned int descriptorImageInfosCount = 0;
	unsigned int descriptorBufferInfosCount = 0;

	unsigned int totalResourcesRequested = 0;
	for (ShaderResource & curRes : allResources)
		totalResourcesRequested += max((uint32_t)curRes.arrayedResource.size(), 1);
	writeDescriptorSets.reserve(totalResourcesRequested);
	descriptorImageInfos.resize(totalResourcesRequested);
	accelStructInfos.resize(totalResourcesRequested);
	descriptorBufferInfos.resize(totalResourcesRequested);

	for (ShaderResource & curRes : allResources)
	{
		VkWriteDescriptorSet writeDescriptorSet = {};
		writeDescriptorSet.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		writeDescriptorSet.pNext = VK_NULL_HANDLE;
		writeDescriptorSet.dstSet = descriptorSets[curRes.setId];
		writeDescriptorSet.descriptorCount = 1;
		writeDescriptorSet.descriptorType = (VkDescriptorType)curRes.type;
		writeDescriptorSet.dstBinding = curRes.bindId;
		if (curRes.rtSceneRef)
		{
			if (RTInstance::Enabled())
			{
				VkWriteDescriptorSetAccelerationStructureKHR descriptorAccelerationStructureInfo;
				descriptorAccelerationStructureInfo.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
				descriptorAccelerationStructureInfo.pNext = nullptr;
				descriptorAccelerationStructureInfo.accelerationStructureCount = 1;
				descriptorAccelerationStructureInfo.pAccelerationStructures = &curRes.rtSceneRef->accelStruct;

				accelStructInfos[accelStructInfosCount] = descriptorAccelerationStructureInfo;
				writeDescriptorSet.pNext = &accelStructInfos[accelStructInfosCount];
				accelStructInfosCount++;
			}
		}
		else if (curRes.samplerRef)
		{
			VkDescriptorImageInfo descriptorImageInfo = {};
			descriptorImageInfo.sampler = curRes.samplerRef->sampler;
			descriptorImageInfo.imageView = curRes.imageViewRef ? curRes.imageViewRef->view : curRes.samplerRef->view;

			if (!curRes.samplerRef->uploadedTexture)
				descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
			else
				descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

			descriptorImageInfos[descriptorImageInfosCount] = descriptorImageInfo;
			writeDescriptorSet.pImageInfo = &descriptorImageInfos[descriptorImageInfosCount];
			descriptorImageInfosCount++;
		}
		else if (curRes.arrayedResource.size() > 0)
		{
			writeDescriptorSet.descriptorCount = (uint32_t)curRes.arrayedResource.size();
			bool setPointer = false;
			for (ShaderResource & curResInArray : curRes.arrayedResource)
			{
				if (curResInArray.samplerRef)
				{
					VkDescriptorImageInfo descriptorImageInfo = {};
					descriptorImageInfo.sampler = curResInArray.samplerRef->sampler;
					descriptorImageInfo.imageView = curResInArray.imageViewRef ? curResInArray.imageViewRef->view : curResInArray.samplerRef->view;

					if (!curResInArray.samplerRef->uploadedTexture)
						descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
					else
						descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

					descriptorImageInfos[descriptorImageInfosCount] = descriptorImageInfo;
					if (!setPointer)
					{
						writeDescriptorSet.pImageInfo = &descriptorImageInfos[descriptorImageInfosCount];
						setPointer = true;
					}
					descriptorImageInfosCount++;
				}
				else
				{
					descriptorBufferInfos[descriptorBufferInfosCount] = curResInArray.uniformRef->descriptor;
					if (!setPointer)
					{
						writeDescriptorSet.pBufferInfo = &descriptorBufferInfos[descriptorBufferInfosCount];
						setPointer = true;
					}
					descriptorBufferInfosCount++;
				}
			}
		}
		else
		{
			writeDescriptorSet.pBufferInfo = &curRes.uniformRef->descriptor;
		}
		writeDescriptorSets.push_back(writeDescriptorSet);
	}

	vkUpdateDescriptorSets(ptrToInstance->device, (uint32_t)writeDescriptorSets.size(), writeDescriptorSets.data(), 0, VK_NULL_HANDLE);
}

void HIGHOMEGA::GL::DescriptorSets::RewriteDescriptorSets(std::vector<ShaderResource>& allResources)
{
	if (!ptrToInstance || !descriptorPoolPtr) throw std::runtime_error("Attempting to write descriptor sets without a ref. to the Vulkan instance or having a descriptor pool");

	for (VkDescriptorSet & curDescSet : descriptorSets)
		vkFreeDescriptorSets(ptrToInstance->device, poolObject, 1, &curDescSet);
	descriptorSets.clear();

	WriteDescriptorSets(allResources);
}

void HIGHOMEGA::GL::DescriptorSets::UpdateDescriptorSets(std::vector<ShaderResource>& allResources)
{
	for (unsigned int curSet : ptrToDescSetLayout->allSets)
	{
		if (descriptorUpdateTemplateEntries.find(curSet) == descriptorUpdateTemplateEntries.end()) continue;
		descriptorUpdateTemplateEntries[curSet].clear();
		descriptorUpdateTemplateData[curSet].clear();
	}

	for (ShaderResource & curRes : allResources)
	{
		if (curRes.samplerRef)
		{
			VkDescriptorImageInfo descriptorImageInfo = {};
			descriptorImageInfo.sampler = curRes.samplerRef->sampler;
			descriptorImageInfo.imageView = curRes.imageViewRef ? curRes.imageViewRef->view : curRes.samplerRef->view;

			if (!curRes.samplerRef->uploadedTexture)
				descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
			else
				descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

			unsigned int writeOffset = (unsigned int)descriptorUpdateTemplateData[curRes.setId].size();
			descriptorUpdateTemplateData[curRes.setId].reserve((unsigned int)ceil((double)descriptorUpdateTemplateData[curRes.setId].size() / (double)409600) * (unsigned int)409600);
			descriptorUpdateTemplateData[curRes.setId].resize(writeOffset + sizeof(VkDescriptorImageInfo));
			memcpy(&descriptorUpdateTemplateData[curRes.setId][writeOffset], &descriptorImageInfo, sizeof(VkDescriptorImageInfo));

			VkDescriptorUpdateTemplateEntry newEntry;
			newEntry.dstBinding = curRes.bindId;
			newEntry.dstArrayElement = 0;
			newEntry.descriptorCount = 1;
			newEntry.descriptorType = (VkDescriptorType)curRes.type;
			newEntry.offset = writeOffset;
			newEntry.stride = 0;
			descriptorUpdateTemplateEntries[curRes.setId].push_back(newEntry);
		}
		else if (curRes.arrayedResource.size() > 0)
		{
			bool setOffset = false;
			unsigned int firstWriteOffset = 0, firstStride = 0;

			for (ShaderResource & curResInArray : curRes.arrayedResource)
			{
				if (curResInArray.samplerRef)
				{
					VkDescriptorImageInfo descriptorImageInfo = {};
					descriptorImageInfo.sampler = curResInArray.samplerRef->sampler;
					descriptorImageInfo.imageView = curResInArray.imageViewRef ? curResInArray.imageViewRef->view : curResInArray.samplerRef->view;

					if (!curResInArray.samplerRef->uploadedTexture)
						descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;
					else
						descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

					unsigned int writeOffset = (unsigned int)descriptorUpdateTemplateData[curRes.setId].size();

					descriptorUpdateTemplateData[curRes.setId].reserve((unsigned int)ceil((double)descriptorUpdateTemplateData[curRes.setId].size() / (double)409600) * (unsigned int)409600);
					descriptorUpdateTemplateData[curRes.setId].resize(writeOffset + sizeof(VkDescriptorImageInfo));
					memcpy(&descriptorUpdateTemplateData[curRes.setId][writeOffset], &descriptorImageInfo, sizeof(VkDescriptorImageInfo));

					if (!setOffset)
					{
						firstWriteOffset = writeOffset;
						firstStride = sizeof(VkDescriptorImageInfo);
						setOffset = true;
					}
				}
				else
				{
					unsigned int writeOffset = (unsigned int)descriptorUpdateTemplateData[curRes.setId].size();

					descriptorUpdateTemplateData[curRes.setId].reserve((unsigned int)ceil((double)descriptorUpdateTemplateData[curRes.setId].size() / (double)409600) * (unsigned int)409600);
					descriptorUpdateTemplateData[curRes.setId].resize(writeOffset + sizeof(VkDescriptorBufferInfo));
					memcpy(&descriptorUpdateTemplateData[curRes.setId][writeOffset], &curResInArray.uniformRef->descriptor, sizeof(VkDescriptorBufferInfo));

					if (!setOffset)
					{
						firstWriteOffset = writeOffset;
						firstStride = sizeof(VkDescriptorBufferInfo);
						setOffset = true;
					}
				}
			}

			VkDescriptorUpdateTemplateEntry newEntry;
			newEntry.dstBinding = curRes.bindId;
			newEntry.dstArrayElement = 0;
			newEntry.descriptorCount = (unsigned int)curRes.arrayedResource.size();
			newEntry.descriptorType = (VkDescriptorType)curRes.type;
			newEntry.offset = firstWriteOffset;
			newEntry.stride = firstStride;
			descriptorUpdateTemplateEntries[curRes.setId].push_back(newEntry);
		}
		else
		{
			unsigned int writeOffset = (unsigned int)descriptorUpdateTemplateData[curRes.setId].size();
			descriptorUpdateTemplateData[curRes.setId].reserve((unsigned int)ceil((double)descriptorUpdateTemplateData[curRes.setId].size() / (double)409600) * (unsigned int)409600);
			descriptorUpdateTemplateData[curRes.setId].resize(writeOffset + sizeof(VkDescriptorBufferInfo));
			memcpy(&descriptorUpdateTemplateData[curRes.setId][writeOffset], &curRes.uniformRef->descriptor, sizeof(VkDescriptorBufferInfo));

			VkDescriptorUpdateTemplateEntry newEntry;
			newEntry.dstBinding = curRes.bindId;
			newEntry.dstArrayElement = 0;
			newEntry.descriptorCount = 1;
			newEntry.descriptorType = (VkDescriptorType)curRes.type;
			newEntry.offset = writeOffset;
			newEntry.stride = 0;
			descriptorUpdateTemplateEntries[curRes.setId].push_back(newEntry);
		}
	}

	for (unsigned int curSet : ptrToDescSetLayout->allSets)
	{
		if (descriptorUpdateTemplateEntries.find(curSet) == descriptorUpdateTemplateEntries.end()) continue;
		VkDescriptorUpdateTemplateCreateInfo createInfo =
		{
			VK_STRUCTURE_TYPE_DESCRIPTOR_UPDATE_TEMPLATE_CREATE_INFO,
			nullptr,
			0,
			(unsigned int)descriptorUpdateTemplateEntries[curSet].size(),
			descriptorUpdateTemplateEntries[curSet].data(),
			VK_DESCRIPTOR_UPDATE_TEMPLATE_TYPE_DESCRIPTOR_SET,
			ptrToDescSetLayout->descriptorSetLayouts[curSet],
			(VkPipelineBindPoint)0,
			0,
			0
		};
		VkDescriptorUpdateTemplate myDescriptorUpdateTemplate;
		VkResult myResult = ptrToInstance->fpCreateDescriptorUpdateTemplateKHR(ptrToInstance->device, &createInfo, nullptr, &myDescriptorUpdateTemplate);
		if (myResult != VK_SUCCESS) throw std::runtime_error("Create descriptor update template failed.");
		ptrToInstance->fpUpdateDescriptorSetWithTemplateKHR(ptrToInstance->device, descriptorSets[curSet], myDescriptorUpdateTemplate, descriptorUpdateTemplateData[curSet].data());
		ptrToInstance->fpDestroyDescriptorUpdateTemplateKHR(ptrToInstance->device, myDescriptorUpdateTemplate, nullptr);
	}
}

HIGHOMEGA::GL::DescriptorSets::~DescriptorSets()
{
	RemovePast();
}

HIGHOMEGA::GL::PipelineFlags::PipelineFlags()
{
	depthWrite = depthTest = true;
	depthCompare = COMPARE_LESS_OR_EQUAL;
	stencilWrite = stencilTest = false;
	stencilCompare = COMPARE_ALWAYS;
	stencilOp = STENCIL_KEEP;
	stencilCompareValue = 0x00;
	backFaceCulling = frontFaceCulling = frontFaceClockWise = false;
	blendEnable = alphaBlending = colorBlending = false;
	srcAlphaFactor = dstAlphaFactor = srcColorFactor = dstColorFactor = FACTOR_ONE;
	alphaBlendOp = colorBlendOp = BLEND_ADD;
	redMask = greenMask = blueMask = alphaMask = true;

	changedDepthWrite = changedDepthTest = false;
	changedBackFaceCulling = changedFrontFaceCulling = changedFrontFaceClockWise = false;
	changedBlendEnable = false;
	changedColorMask = false;
}

bool HIGHOMEGA::GL::PipelineFlags::operator==(const PipelineFlags & other) const
{
	return (depthWrite == other.depthWrite) &&
		(depthTest == other.depthTest) &&
		(depthCompare == other.depthCompare) &&
		(stencilWrite == other.stencilWrite) &&
		(stencilTest == other.stencilTest) &&
		(stencilCompare == other.stencilCompare) &&
		(stencilOp == other.stencilOp) &&
		(stencilCompareValue == other.stencilCompareValue) &&
		(backFaceCulling == other.backFaceCulling) &&
		(frontFaceCulling == other.frontFaceCulling) &&
		(frontFaceClockWise == other.frontFaceClockWise) &&
		(blendEnable == other.blendEnable) &&
		(alphaBlending == other.alphaBlending) &&
		(colorBlending == other.colorBlending) &&
		(srcAlphaFactor == other.srcAlphaFactor) &&
		(dstAlphaFactor == other.dstAlphaFactor) &&
		(srcColorFactor == other.srcColorFactor) &&
		(dstColorFactor == other.dstColorFactor) &&
		(alphaBlendOp == other.alphaBlendOp) &&
		(colorBlendOp == other.colorBlendOp) &&
		(redMask == other.redMask) &&
		(greenMask == other.greenMask) &&
		(blueMask == other.blueMask) &&
		(alphaMask == other.alphaMask) &&
		(changedDepthWrite == other.changedDepthWrite) &&
		(changedDepthTest == other.changedDepthTest) &&
		(changedBackFaceCulling == other.changedBackFaceCulling) &&
		(changedFrontFaceCulling == other.changedFrontFaceCulling) &&
		(changedFrontFaceClockWise == other.changedFrontFaceClockWise) &&
		(changedBlendEnable == other.changedBlendEnable) &&
		(changedColorMask == other.changedColorMask);
}

std::size_t HIGHOMEGA::GL::PipelineFlagsHash::operator()(const PipelineFlags & k) const
{
	using std::size_t;
	using std::hash;
	using std::string;

	return (hash<bool>()(k.depthWrite))
		^ (hash<bool>()(k.depthTest) << 1)
		^ (hash<CompareTypes>()(k.depthCompare) << 2)
		^ (hash<bool>()(k.stencilWrite) << 3)
		^ (hash<bool>()(k.stencilTest) << 4)
		^ (hash<CompareTypes>()(k.stencilCompare) << 5)
		^ (hash<StencilOperation>()(k.stencilOp) << 6)
		^ (hash<unsigned int>()(k.stencilCompareValue) << 7)
		^ (hash<bool>()(k.backFaceCulling) << 8)
		^ (hash<bool>()(k.frontFaceCulling) << 9)
		^ (hash<bool>()(k.frontFaceClockWise) << 10)
		^ (hash<bool>()(k.blendEnable) << 11)
		^ (hash<bool>()(k.alphaBlending) << 12)
		^ (hash<bool>()(k.colorBlending) << 13)
		^ (hash<BlendFactors>()(k.srcAlphaFactor) << 14)
		^ (hash<BlendFactors>()(k.dstAlphaFactor) << 15)
		^ (hash<BlendFactors>()(k.srcColorFactor) << 16)
		^ (hash<BlendFactors>()(k.dstColorFactor) << 17)
		^ (hash<BlendOps>()(k.alphaBlendOp) << 18)
		^ (hash<BlendOps>()(k.colorBlendOp) << 19)
		^ (hash<bool>()(k.redMask) << 20)
		^ (hash<bool>()(k.greenMask) << 21)
		^ (hash<bool>()(k.blueMask) << 22)
		^ (hash<bool>()(k.alphaMask) << 23)
		^ (hash<bool>()(k.changedDepthTest) << 24)
		^ (hash<bool>()(k.changedBackFaceCulling) << 25)
		^ (hash<bool>()(k.changedFrontFaceCulling) << 26)
		^ (hash<bool>()(k.changedFrontFaceClockWise) << 27)
		^ (hash<bool>()(k.changedBlendEnable) << 28)
		^ (hash<bool>()(k.changedColorMask) << 29);
}

void HIGHOMEGA::GL::KHR_RT::RTPipelineStateClass::ErasePipelineState()
{
	if (!ptrToInstance) return;

	{std::lock_guard<std::mutex> lk(shader_stage_mutex);
	for (std::string & curStage : usedShaderStages)
	{
		ShaderStageCache[curStage].elemCount--;
		if (ShaderStageCache[curStage].elemCount == 0)
			ShaderStageCache.erase(curStage);
	}}

	if (haveRTPipeline) vkDestroyPipeline(ptrToInstance->device, pipeline, nullptr);
	if (havePipelineLayout) vkDestroyPipelineLayout(ptrToInstance->device, pipelineLayout, nullptr);
	if (havePipelineCache) vkDestroyPipelineCache(ptrToInstance->device, pipelineCache, nullptr);

	havePipelineLayout = false;
	haveRTPipeline = false;
	havePipelineCache = false;
	ptrToInstance = nullptr;
}

HIGHOMEGA::GL::KHR_RT::RTPipelineStateClass::RTPipelineStateClass()
{
	havePipelineLayout = false;
	haveRTPipeline = false;
	havePipelineCache = false;
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::KHR_RT::RTPipelineStateClass::RTPipelineState(InstanceClass & renderInst, DescriptorSetLayout & DescSetLayout, ShaderResourceSet & inpShader)
{
	ptrToInstance = &renderInst;

	VkPipelineLayoutCreateInfo pPipelineLayoutCreateInfo = {};
	pPipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	pPipelineLayoutCreateInfo.pNext = VK_NULL_HANDLE;
	pPipelineLayoutCreateInfo.setLayoutCount = (uint32_t)DescSetLayout.descriptorSetLayouts.size();
	pPipelineLayoutCreateInfo.pSetLayouts = DescSetLayout.descriptorSetLayouts.data();

	VkResult result = vkCreatePipelineLayout(ptrToInstance->device, &pPipelineLayoutCreateInfo, nullptr, &pipelineLayout);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create pipeline layout"); }
	havePipelineLayout = true;

	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_raygen_shader, inpShader.getRaygenEntry(), VK_SHADER_STAGE_RAYGEN_BIT_KHR)->elem.stage); }
	catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create raygen shader"); }
	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_raymiss_shader, inpShader.getRaymissEntry(), VK_SHADER_STAGE_MISS_BIT_KHR)->elem.stage); }
	catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create raymiss shader"); }
	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_raychit_shader, inpShader.getRaychitEntry(), VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)->elem.stage); }
	catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create raychit shader"); }
	usedShaderStages.push_back(inpShader.rt_raygen_shader + std::string(":") + inpShader.getRaygenEntry());
	usedShaderStages.push_back(inpShader.rt_raymiss_shader + std::string(":") + inpShader.getRaymissEntry());
	usedShaderStages.push_back(inpShader.rt_raychit_shader + std::string(":") + inpShader.getRaychitEntry());
	if (inpShader.getRayahitEntry())
	{
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_rayahit_shader, inpShader.getRayahitEntry(), VK_SHADER_STAGE_ANY_HIT_BIT_KHR)->elem.stage); }
		catch (...) { ErasePipelineState(); throw std::runtime_error("Could not create rayahit shader"); }
		usedShaderStages.push_back(inpShader.rt_rayahit_shader + std::string(":") + inpShader.getRayahitEntry());
	}

	shaderGroups.push_back({ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR, nullptr, VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR, 0, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR });
	shaderGroups.push_back({ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR, nullptr, VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR, 1, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR });
	shaderGroups.push_back({ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR, nullptr, VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR, VK_SHADER_UNUSED_KHR, 2, inpShader.getRayahitEntry() ? 3 : VK_SHADER_UNUSED_KHR, VK_SHADER_UNUSED_KHR });

	VkRayTracingPipelineCreateInfoKHR pipelineCreateInfo = {};
	pipelineCreateInfo.stageCount = (uint32_t)shaderStages.size();
	pipelineCreateInfo.pStages = shaderStages.data();
	pipelineCreateInfo.groupCount = (uint32_t)shaderGroups.size();
	pipelineCreateInfo.pGroups = shaderGroups.data();
	pipelineCreateInfo.maxPipelineRayRecursionDepth = 8;
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR;
	pipelineCreateInfo.layout = pipelineLayout;

	VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
	pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	result = vkCreatePipelineCache(ptrToInstance->device, &pipelineCacheCreateInfo, nullptr, &pipelineCache);
	if (result != VK_SUCCESS) { throw std::runtime_error("Could not create a pipeline cache"); }
	havePipelineCache = true;

	result = RTInstance::fpCreateRayTracingPipelinesKHR(ptrToInstance->device, VK_NULL_HANDLE, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not create rt pipeline"); }
	haveRTPipeline = true;

	unsigned int sbtChunkSize = (RTInstance::raytracingPipelineProperties.shaderGroupHandleSize + (RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment - 1)) & (~(RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment - 1));

	uint32_t shaderBindingTableSize = RTInstance::raytracingPipelineProperties.shaderGroupHandleSize * (uint32_t)shaderGroups.size();
	uint32_t shaderBindingTableSizeAligned = sbtChunkSize * (uint32_t)shaderGroups.size();

	shaderBindingTable.Buffer(MEMORY_HOST_VISIBLE | MEMORY_HOST_COHERENT, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_SRC | USAGE_SBT | USAGE_DEVICE_ADDRESS, Instance, nullptr, (unsigned int)shaderBindingTableSizeAligned);
	unsigned char *sbtData = new unsigned char[shaderBindingTableSize];
	unsigned char *sbtDataAligned = new unsigned char[shaderBindingTableSizeAligned];

	result = RTInstance::fpGetRayTracingShaderGroupHandlesKHR(ptrToInstance->device, pipeline, 0, (uint32_t)shaderGroups.size(), shaderBindingTableSize, sbtData);
	if (result != VK_SUCCESS) { ErasePipelineState(); throw std::runtime_error("Could not get shader group handle"); }
	for (int i = 0; i != (uint32_t)shaderGroups.size(); i++)
		memcpy(&sbtDataAligned[i * sbtChunkSize], &sbtData[i * RTInstance::raytracingPipelineProperties.shaderGroupHandleSize], RTInstance::raytracingPipelineProperties.shaderGroupHandleSize);

	shaderBindingTable.UploadSubData(0, sbtDataAligned, shaderBindingTableSizeAligned);

	delete[] sbtData;
	delete[] sbtDataAligned;
}

bool HIGHOMEGA::GL::KHR_RT::RTPipelineStateClass::IsInitialized()
{
	return haveRTPipeline;
}

HIGHOMEGA::GL::KHR_RT::RTPipelineStateClass::~RTPipelineStateClass()
{
	ErasePipelineState();
}

HIGHOMEGA::GL::KHR_RT::RTScene::RTScene()
{
	sceneId = 0ull;
}

HIGHOMEGA::GL::KHR_RT::RTScene::~RTScene()
{
	sceneId = 0ull;
	RemovePast();
}

void HIGHOMEGA::GL::KHR_RT::RTScene::Add(unsigned long long inpId, std::vector <ImageClass *> & inpMaterials, RTGeometry & inpGeom, InstanceProperties & inpInstanceData, InstanceClass & inpInstance)
{
	if (!ptrToInstance) ptrToInstance = &inpInstance;

	TraceItem trItem;
	trItem.geomRef = &inpGeom;
	trItem.material = inpMaterials;
	trItem.instanceData = inpInstanceData;

	VkAccelerationStructureInstanceKHR curInst;
	for (int i = 0; i != 3; i++)
		for (int j = 0; j != 4; j++)
			curInst.transform.matrix[i][j] = (i == j) ? 1.0f : 0.0f;

	curInst.instanceCustomIndex = 0; // This will be assigned later
	curInst.mask = inpGeom.GetMask();
	curInst.instanceShaderBindingTableRecordOffset = 0;
	curInst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;

	trItem.rtInstanceData = curInst;

	allTraceItems[inpId].push_back(trItem);
	needReCreation = true;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::RemoveAll(unsigned long long inpId)
{
	allTraceItems.erase(inpId);
	needReCreation = true;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::CreateInstanceData(std::vector <VkAccelerationStructureInstanceKHR> & instances)
{
	unsigned int instCount = 0u;
	for (std::pair<const unsigned long long, std::vector<TraceItem>> & traceItemKV : allTraceItems)
		instCount += (unsigned int)traceItemKV.second.size();

	instances.clear();
	allMat.clear();
	allGeom.clear();
	instances.reserve(instCount);
	allMat.reserve(instCount * 5);
	allGeom.reserve(instCount);

	unsigned int bufferInstanceCount = (((unsigned int)allTraceItems.size() / 1000) + 1) * 1000;
	unsigned int instanceBufferSize = bufferInstanceCount * sizeof(InstanceProperties);

	allInstanceData.clear();
	allInstanceData.reserve(bufferInstanceCount);
	if (instanceBufferSize > instancePropertiesBuffer.getSize())
	{
		instancePropertiesBuffer.Buffer(MEMORY_HOST_VISIBLE, SHARING_EXCLUSIVE, MODE_CREATE, USAGE_SSBO, *ptrToInstance, nullptr, instanceBufferSize);
	}

	unsigned int instanceIdCount = 0;
	for (std::pair<const unsigned long long, std::vector<TraceItem>> & traceItemKV : allTraceItems)
		for (TraceItem & curTraceItem : traceItemKV.second)
		{
			curTraceItem.rtInstanceData.instanceCustomIndex = instanceIdCount;
			curTraceItem.rtInstanceData.accelerationStructureReference = curTraceItem.geomRef->blasAddress;
			instances.push_back(curTraceItem.rtInstanceData);
			allInstanceData.push_back(curTraceItem.instanceData);
			allGeom.push_back(ShaderResource(RESOURCE_SSBO, RT_RAYGEN, 1, 0, *curTraceItem.geomRef->completeVertexBufferRef));
			for (ImageClass *curSam : curTraceItem.material)
				allMat.push_back(ShaderResource(RESOURCE_SAMPLER, RT_RAYGEN, 2, 0, *curSam));
			instanceIdCount++;
		}

	instancePropertiesBuffer.UploadSubData(0, allInstanceData.data(), (unsigned int)allInstanceData.size() * sizeof(InstanceProperties));
}

unsigned long long HIGHOMEGA::GL::KHR_RT::RTScene::rtSceneID()
{
	blasBuildParams blBuildParams;
	std::vector<RTGeometry *> rebuildGeoms;
	unsigned int rebuildInstCount = 0u;
	for (std::pair <const unsigned long long, std::vector<TraceItem>> & traceItemKV : allTraceItems)
		rebuildInstCount += (unsigned int)traceItemKV.second.size();
	rebuildGeoms.reserve(rebuildInstCount);

	for (std::pair <const unsigned long long, std::vector<TraceItem>> & traceItemKV : allTraceItems)
		for (TraceItem & curTraceItem : traceItemKV.second)
			if (curTraceItem.geomRef->dirty || !curTraceItem.geomRef->created)
			{
				if (!curTraceItem.geomRef->created)
				{
					needReCreation = true;
					rebuildGeoms.push_back(curTraceItem.geomRef);
				}
				needUpdate = true;
				curTraceItem.geomRef->CreateOrUpdate(&blBuildParams);
			}

	if (blBuildParams.blasBuildInfos.size() > 0)
	{
		BeginCommandBuffer(*ptrToInstance);

		RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[0], (unsigned int)blBuildParams.blasBuildInfos.size(),
			(const VkAccelerationStructureBuildGeometryInfoKHR *)blBuildParams.blasBuildInfos.data(),
			(const VkAccelerationStructureBuildRangeInfoKHR * const *)blBuildParams.blasBuildRanges.data());

		EndCommandBuffer();
		SubmitCommandBuffer();
	}

	for (RTGeometry * curGeom : rebuildGeoms)
		curGeom->FetchBlasAddress();

	if (needReCreation)
	{
		if (sceneId != 0ull) RTAccelStruct::~RTAccelStruct();
		sceneId = mersenneTwister64BitPRNG();

		CreateInstanceData(instances);

		try { CreateAccelStruct(false, nullptr, nullptr, &instances, false, *ptrToInstance, &blBuildParams); }
		catch (...) { RemovePast(); throw; }

		needReCreation = false;
		needUpdate = false;
	}

	if (needUpdate)
	{
		UpdateAccelStruct(false, nullptr, nullptr, &instances, &blBuildParams);
		needUpdate = false;
	}

	return sceneId;
}

std::vector<ShaderResource>& HIGHOMEGA::GL::KHR_RT::RTScene::getMaterialResources()
{
	return allMat;
}

std::vector<ShaderResource>& HIGHOMEGA::GL::KHR_RT::RTScene::getGeomResources()
{
	return allGeom;
}

BufferClass & HIGHOMEGA::GL::KHR_RT::RTScene::getInstancePropertiesBuffer()
{
	return instancePropertiesBuffer;
}

HIGHOMEGA::GL::KHR_RT::RTTracelet::RTTracelet()
{
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::KHR_RT::RTTracelet::Make(InstanceClass & inpInstance)
{
	ptrToInstance = &inpInstance;

	traceSem.Semaphore(ptrToInstance);
}

void HIGHOMEGA::GL::KHR_RT::RTTracelet::Submit(unsigned int inpWidth, unsigned int inpHeight, unsigned int inpDepth, std::vector<ShaderResource> & inpTracingResources, bool updateResources, ShaderResourceSet & inpRTShaderResourceSet)
{
	if (!ptrToInstance) return;

	if (updateResources) recordedCmdBuf = false;

	if (!recordedCmdBuf)
	{
		if (!recordedPSO)
		{
			DSL.CreateDescriptorSetLayout(inpTracingResources, ptrToInstance);
			PSO.RTPipelineState(*ptrToInstance, DSL, inpRTShaderResourceSet);
			DS.Make(&DSL);
			DS.WriteDescriptorSets(inpTracingResources);
			recordedPSO = true;
		}
		else if (updateResources)
		{
			DS.RewriteDescriptorSets(inpTracingResources);
		}

		BeginCommandBuffer(*ptrToInstance);

		vkCmdBindPipeline(cmdBuffers[0], VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, PSO.pipeline);
		vkCmdBindDescriptorSets(cmdBuffers[0], VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, PSO.pipelineLayout, 0, (uint32_t)DS.descriptorSets.size(), DS.descriptorSets.data(), 0, 0);

		unsigned int sbtChunkSize = (RTInstance::raytracingPipelineProperties.shaderGroupHandleSize + (RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment - 1)) & (~(RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment - 1));

		VkBufferDeviceAddressInfo bufDevAdInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		bufDevAdInfo.buffer = PSO.shaderBindingTable.buffer;
		VkDeviceAddress sbtDeviceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufDevAdInfo);

		std::array<VkStridedDeviceAddressRegionKHR, 4> strideAddresses{
			VkStridedDeviceAddressRegionKHR{sbtDeviceAddress + 0u * sbtChunkSize, sbtChunkSize, sbtChunkSize},
			VkStridedDeviceAddressRegionKHR{sbtDeviceAddress + 1u * sbtChunkSize, sbtChunkSize, sbtChunkSize},
			VkStridedDeviceAddressRegionKHR{sbtDeviceAddress + 2u * sbtChunkSize, sbtChunkSize, sbtChunkSize},
			VkStridedDeviceAddressRegionKHR{0u,0u,0u} };

		RTInstance::fpCmdTraceRaysKHR(cmdBuffers[0], &strideAddresses[0], &strideAddresses[1], &strideAddresses[2], &strideAddresses[3], inpWidth, inpHeight, inpDepth);

		EndCommandBuffer();

		recordedCmdBuf = true;
	}

	WaitSubmitSignalCommandBuffer(submissionMode, traceSem);
}

void HIGHOMEGA::GL::KHR_RT::RTTracelet::makeAsync()
{
	submissionMode = SUBMIT_ASYNC;
}

void HIGHOMEGA::GL::KHR_RT::RTTracelet::makeSerial()
{
	submissionMode = SUBMIT_SERIAL;
}