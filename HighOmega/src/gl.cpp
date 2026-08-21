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

#include "gl.h"
#include "encodedshaders.h"

using namespace HIGHOMEGA;
using namespace HIGHOMEGA::GL;
using namespace HIGHOMEGA::GL::KHR_RT;
using namespace HIGHOMEGA::GL::MEMORY_MANAGER;

#define HIGHOMEGA_EXTRACT_SHADERS_FROM_BINARY 1

namespace HIGHOMEGA::GL
{
	bool InstanceClass::lowMemoryDevice = false;
	std::unordered_map <std::string, HIGHOMEGA::CacheItem<ShaderStage>> ShaderStageCache;
	std::unordered_map <int, std::vector<MemChunk>> MemoryMap;
	std::unordered_map<VkDeviceMemory, InstanceClass::memMapInfo> InstanceClass::allMemMaps;
	std::mutex mem_manager_mutex;
}

std::unordered_map<std::string, Raster_PSO_DSL> HIGHOMEGA::GL::globalRaster_PSO_DSL_Cache;
std::mutex HIGHOMEGA::GL::globalRaster_PSO_DSL_Cache_mutex;
std::unordered_map<std::string, Compute_PSO_DSL> HIGHOMEGA::GL::globalCompute_PSO_DSL_Cache;
BufferClass* HIGHOMEGA::GL::giantVertBuffer = nullptr;
unsigned int HIGHOMEGA::GL::giantVertBufferClaims = 0u;
std::vector<HIGHOMEGA::GL::VertBufferOffsetLen> HIGHOMEGA::GL::giantVertBufferSubAllocs;
std::shared_mutex HIGHOMEGA::GL::giantVertBufferSharedMutex;
std::mutex HIGHOMEGA::GL::giantVertBufferDirectoryMutex;

thread_local BufferClass HIGHOMEGA::GL::vertexStagingBuffer;
thread_local BufferClass HIGHOMEGA::GL::imageStagingBuffer;
ThreadLocalCache <LibKTX2VDIWrapper> HIGHOMEGA::GL::ImageClass::ktx2VDIPools;
ThreadLocalCache <VkCommandPool> HIGHOMEGA::GL::CommandBuffer::cmdPools[QUEUE_TYPE::MAX_QUEUES];
thread_local unsigned long long ThreadID = threadSafeMersenneTwister64Bit();


std::mutex HIGHOMEGA::GL::globalCompute_PSO_DSL_Cache_mutex;

#define HIGHOMEGA_ONE_GIANT_VERTBUFFER_SIZE (1024 * 1024 * 300)

std::vector<std::tuple<int, unsigned long long, SubAlloc>> HIGHOMEGA::GL::MEMORY_MANAGER::AllocMem(VkDevice & inpDev, VkMemoryAllocateInfo & allocInfo, VkMemoryRequirements & memReq, MEMORY_MAP_TYPE memoryMapType, bool useSparseResources, bool oneGiantVertBuffer)
{
	std::lock_guard <std::mutex> lk(mem_manager_mutex);

	VkResult result;

	unsigned long long chunkMaxSize;
	int memTypeKey = (memoryMapType << (unsigned int)log2((double)VK_MAX_MEMORY_TYPES)) | allocInfo.memoryTypeIndex;
	switch(memoryMapType)
	{
	case IMAGE:
		chunkMaxSize = 1024 * 1024 * 512;
		break;
	case BUFFER:
		chunkMaxSize = oneGiantVertBuffer ? HIGHOMEGA_ONE_GIANT_VERTBUFFER_SIZE : 1024 * 1024 * (useSparseResources ? 200 : 256);
		break;
	case DEV_ADDRESS:
	default:
		chunkMaxSize = oneGiantVertBuffer ? HIGHOMEGA_ONE_GIANT_VERTBUFFER_SIZE : 1024 * 1024 * 256;
		break;
	}
	for (MemChunk & curChunk : MemoryMap[memTypeKey])
	{
		std::vector<SubAlloc>::iterator it = curChunk.allocs.begin();
		if (curChunk.allocs.size() > 0)
		{
			bool firstSearch = true;
			SubAlloc firstHole;
			firstHole.offset = 0u;
			firstHole.len = 0u;
			while (true)
			{
				SubAlloc& curSubAlloc = firstSearch ? firstHole : *it;
				if (!firstSearch) it++;
				SubAlloc& nextSubAlloc = firstSearch ? *curChunk.allocs.begin() : *it;
				firstSearch = false;
				unsigned long long offsetAligned = (curSubAlloc.offset + curSubAlloc.len + (memReq.alignment - 1)) & (~(memReq.alignment - 1));
				if (offsetAligned + memReq.size <= nextSubAlloc.offset)
				{
					SubAlloc newSubAlloc;
					newSubAlloc.len = memReq.size;
					newSubAlloc.offset = offsetAligned;
					newSubAlloc.mem = curChunk.mem;
					curChunk.allocs.insert(it, newSubAlloc);

					return { std::make_tuple(memTypeKey, curChunk.id, newSubAlloc) };
				}
				if (nextSubAlloc == curChunk.allocs.back()) break;
			}
		}

		unsigned long long usedAligned = (curChunk.used + (memReq.alignment - 1)) & (~(memReq.alignment - 1));

		if (usedAligned + memReq.size > curChunk.maxSize) continue;

		SubAlloc newSubAlloc;
		newSubAlloc.len = memReq.size;
		newSubAlloc.offset = usedAligned;
		newSubAlloc.mem = curChunk.mem;
		curChunk.allocs.push_back(newSubAlloc);

		curChunk.used = usedAligned + memReq.size;

		return { std::make_tuple(memTypeKey, curChunk.id, newSubAlloc) };
	}
	unsigned long long storageSizeConsumed = 0u;
	std::vector<std::tuple<int, unsigned long long, SubAlloc>> returnedSubAllocs;
	if (useSparseResources)
	{
		for (MemChunk& curChunk : MemoryMap[memTypeKey])
		{
			std::vector<SubAlloc>::iterator it = curChunk.allocs.begin();
			if (curChunk.allocs.size() > 0)
			{
				bool firstSearch = true;
				SubAlloc firstHole;
				firstHole.offset = 0u;
				firstHole.len = 0u;
				while (true)
				{
					SubAlloc& curSubAlloc = firstSearch ? firstHole : *it;
					if (!firstSearch) it++;
					SubAlloc& nextSubAlloc = firstSearch ? *curChunk.allocs.begin() : *it;
					firstSearch = false;
					unsigned long long offsetAligned = (curSubAlloc.offset + curSubAlloc.len + (memReq.alignment - 1)) & (~(memReq.alignment - 1));
					if (nextSubAlloc.offset > offsetAligned && storageSizeConsumed < memReq.size)
					{
						SubAlloc newSubAlloc;
						newSubAlloc.len = min(nextSubAlloc.offset - offsetAligned, memReq.size - storageSizeConsumed);
						newSubAlloc.offset = offsetAligned;
						newSubAlloc.mem = curChunk.mem;
						storageSizeConsumed += newSubAlloc.len;
						curChunk.allocs.insert(it, newSubAlloc);

						returnedSubAllocs.push_back(std::make_tuple(memTypeKey, curChunk.id, newSubAlloc));
					}
					if (nextSubAlloc == curChunk.allocs.back() || storageSizeConsumed == memReq.size) break;
				}
			}

			unsigned long long usedAligned = (curChunk.used + (memReq.alignment - 1)) & (~(memReq.alignment - 1));

			if (storageSizeConsumed == memReq.size || curChunk.maxSize == usedAligned) break;

			SubAlloc newSubAlloc;
			newSubAlloc.len = min(curChunk.maxSize - usedAligned, memReq.size - storageSizeConsumed);
			newSubAlloc.offset = usedAligned;
			newSubAlloc.mem = curChunk.mem;
			storageSizeConsumed += newSubAlloc.len;
			curChunk.allocs.push_back(newSubAlloc);

			curChunk.used = curChunk.maxSize;

			returnedSubAllocs.push_back (std::make_tuple(memTypeKey, curChunk.id, newSubAlloc));
			if (storageSizeConsumed == memReq.size) break;
		}
		if (storageSizeConsumed == memReq.size)
			return returnedSubAllocs;
	}

	MemoryMap[memTypeKey].push_back(MemChunk());
	MemChunk & addedChunkRef = MemoryMap[memTypeKey].back();
	addedChunkRef.memoryTypeKey = memTypeKey;
	addedChunkRef.id = threadSafeMersenneTwister64Bit();
	addedChunkRef.maxSize = chunkMaxSize;

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
	if (result != VK_SUCCESS) FATAL_ERROR("Could not allocate memory chunk");

	SubAlloc newSubAlloc;
	newSubAlloc.len = useSparseResources ? (memReq.size - storageSizeConsumed) : memReq.size;
	newSubAlloc.offset = addedChunkRef.used;
	newSubAlloc.mem = addedChunkRef.mem;
	addedChunkRef.allocs.push_back(newSubAlloc);

	addedChunkRef.used += newSubAlloc.len;

	if (useSparseResources)
	{
		returnedSubAllocs.push_back(std::make_tuple(memTypeKey, addedChunkRef.id, newSubAlloc));
		return returnedSubAllocs;
	}
	else
		return { std::make_tuple(memTypeKey, addedChunkRef.id, newSubAlloc) };
}

unsigned long long HIGHOMEGA::GL::MEMORY_MANAGER::ReportMemoryHoles()
{
	unsigned long long retVal = 0ull;
	for (std::pair<const int, std::vector<MemChunk>>& curList : MemoryMap)
		for (MemChunk& curChunk : curList.second)
		{
			std::vector<SubAlloc>::iterator it = curChunk.allocs.begin();
			if (curChunk.allocs.size() > 0)
			{
				bool firstSearch = true;
				SubAlloc firstHole;
				firstHole.offset = 0u;
				firstHole.len = 0u;
				while (true)
				{
					SubAlloc& curSubAlloc = firstSearch ? firstHole : *it;
					if (!firstSearch) it++;
					SubAlloc& nextSubAlloc = firstSearch ? *curChunk.allocs.begin() : *it;
					firstSearch = false;
					retVal += (nextSubAlloc.offset - (curSubAlloc.offset + curSubAlloc.len));
					if (nextSubAlloc == curChunk.allocs.back()) break;
				}
			}
		}
	giantVertBufferDirectoryMutex.lock();
	if (giantVertBufferSubAllocs.size() > 0)
	{
		std::vector<VertBufferOffsetLen>::iterator it = giantVertBufferSubAllocs.begin();
		unsigned int totalVertBufferHoles = 0u;
		bool firstSearch = true;
		VertBufferOffsetLen firstHole;
		firstHole.offset = 0u;
		firstHole.len = 0u;
		while (true)
		{
			VertBufferOffsetLen& curSubAlloc = firstSearch ? firstHole : *it;
			if (!firstSearch) it++;
			VertBufferOffsetLen& nextSubAlloc = firstSearch ? *giantVertBufferSubAllocs.begin() : *it;
			firstSearch = false;
			retVal += (nextSubAlloc.offset - (curSubAlloc.offset + curSubAlloc.len));
			if (nextSubAlloc == giantVertBufferSubAllocs.back()) break;
		}
	}
	giantVertBufferDirectoryMutex.unlock();
	return retVal;
}

void HIGHOMEGA::GL::MEMORY_MANAGER::LogMemUsageStats()
{
	std::lock_guard <std::mutex> lk(mem_manager_mutex);
	unsigned long long totalVRamUsed = 0ull;
	LOG() << "Mem usage statistics: ";
	for (std::pair<const int, std::vector<MemChunk>> curPair : MemoryMap)
	{
		LOG() << "memoryTypeIndex: " << curPair.first;
		for (MemChunk& curChunk : curPair.second)
		{
			LOG() << "Chunk usage: " << curChunk.used << " / " << curChunk.maxSize;
			totalVRamUsed += curChunk.maxSize;
		}
	}
	giantVertBufferDirectoryMutex.lock();
	unsigned int totalVertBufferUsage = 0u;
	for (VertBufferOffsetLen& curPair : giantVertBufferSubAllocs)
		totalVertBufferUsage += (unsigned int)curPair.len;
	LOG() << "Total usage of the giant vert buffer: " << totalVertBufferUsage;
	giantVertBufferDirectoryMutex.unlock();
	LOG() << "Total VRAM used: " << totalVRamUsed;

	LOG() << "Total memory holes from fragmentation: " << ReportMemoryHoles();
}

void HIGHOMEGA::GL::MEMORY_MANAGER::FreeMem(std::vector<std::tuple<int, unsigned long long, SubAlloc>>& pages, VkDevice & inpDev)
{
	std::lock_guard <std::mutex> lk(mem_manager_mutex);
	for (std::tuple<int, unsigned long long, SubAlloc> & curPage : pages)
	{
		if (MemoryMap.find(std::get<0>(curPage)) == MemoryMap.end()) FATAL_ERROR("Freeing memory failed, memory type key not found");
		std::vector<MemChunk>& chunksForType = MemoryMap[std::get<0>(curPage)];
		std::vector<MemChunk>::iterator memChunkRef = std::find_if(chunksForType.begin(), chunksForType.end(), [&](const MemChunk& memChunk) -> bool {
			return memChunk.id == std::get<1>(curPage);
		});
		if (memChunkRef == chunksForType.end()) FATAL_ERROR("Freeing memory failed, memory chunk not found");
		std::vector<SubAlloc>::iterator subAllocRef = std::find(memChunkRef->allocs.begin(), memChunkRef->allocs.end(), std::get<2>(curPage));
		if (subAllocRef == memChunkRef->allocs.end()) FATAL_ERROR("Freeing memory failed, sub allocation not found");
		memChunkRef->allocs.erase(subAllocRef);
		if (memChunkRef->allocs.size() == 0)
		{
			vkFreeMemory(inpDev, memChunkRef->mem, nullptr);
			chunksForType.erase(memChunkRef);
			if (!chunksForType.size()) MemoryMap.erase(std::get<0>(curPage));
		}
	}
	pages.clear();
}

namespace HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2
{
	std::unordered_map<uint64_t, std::vector<std::tuple<int, unsigned long long, SubAlloc>>> AllocMemCWrapperDirectory;
	std::unordered_map<uint64_t, int> AllocMemCWrapperDirectoryMemType;
	VkDevice* deviceCached = nullptr;
	uint64_t AllocMemCWrapper(VkMemoryAllocateInfo* allocInfo, VkMemoryRequirements* memReq, uint64_t* numPages)
	{
		uint64_t allocId = threadSafeMersenneTwister64Bit();
		try
		{
			std::vector<std::tuple<int, unsigned long long, SubAlloc>> returnedAlloc = AllocMem(*deviceCached, *allocInfo, *memReq, MEMORY_MAP_TYPE::IMAGE, false); // LibKTX2 *still* has no sparse binding support. Can someone other than me do this please?
			{ std::lock_guard <std::mutex> lk(mem_manager_mutex);
			AllocMemCWrapperDirectory[allocId] = returnedAlloc;
			AllocMemCWrapperDirectoryMemType[allocId] = allocInfo->memoryTypeIndex;
			*numPages = AllocMemCWrapperDirectory[allocId].size(); }
			return allocId;
		}
		catch (...)
		{
			return 0ull;
		}
	}

	VkResult BindBufferMemoryCWrapper(VkBuffer buffer, uint64_t allocId)
	{
		int memType;
		std::vector<std::tuple<int, unsigned long long, SubAlloc>> allocEntry;
		{ std::lock_guard <std::mutex> lk(mem_manager_mutex); 
		memType = AllocMemCWrapperDirectoryMemType[allocId];
		allocEntry = AllocMemCWrapperDirectory[allocId]; }
		if (Instance.SupportsSparseResources() && memType == 1 && false) // LibKTX2 *still* has no sparse binding support. Can someone other than me do this please?
		{
			FenceClass sparseFence;
			sparseFence.Fence(&Instance);
			sparseFence.Reset();
			VkSparseBufferMemoryBindInfo bufferMemoryBinds;
			bufferMemoryBinds.buffer = buffer;
			std::vector<VkSparseMemoryBind> memoryBinds;
			unsigned long long resourceOffset = 0ull;
			for (std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>& curSubAlloc : allocEntry)
			{
				VkSparseMemoryBind curBufBind = {};
				curBufBind.memory = std::get<2>(curSubAlloc).mem;
				curBufBind.memoryOffset = (VkDeviceSize)std::get<2>(curSubAlloc).offset;
				curBufBind.size = (VkDeviceSize)std::get<2>(curSubAlloc).len;
				curBufBind.resourceOffset = (VkDeviceSize)resourceOffset;
				curBufBind.flags = VK_SPARSE_MEMORY_BIND_METADATA_BIT;
				resourceOffset += std::get<2>(curSubAlloc).len;
				memoryBinds.push_back(curBufBind);
			}
			bufferMemoryBinds.bindCount = (uint32_t)memoryBinds.size();
			bufferMemoryBinds.pBinds = memoryBinds.data();
			VkBindSparseInfo vkBindSparseInfo = {};
			vkBindSparseInfo.sType = VK_STRUCTURE_TYPE_BIND_SPARSE_INFO;
			vkBindSparseInfo.bufferBindCount = 1;
			vkBindSparseInfo.pBufferBinds = &bufferMemoryBinds;
			VkResult result;
			{std::unique_lock <std::mutex> lk(mem_manager_mutex);
			Instance.allQueues[TRANSFER_QUEUE].mtx.lock();
			result = vkQueueBindSparse(Instance.allQueues[TRANSFER_QUEUE].queue, 1, &vkBindSparseInfo, sparseFence.fence);
			Instance.allQueues[TRANSFER_QUEUE].mtx.unlock();}
			sparseFence.Wait();
			return result;
		}
		else
		{
			VkResult result;
			{std::unique_lock <std::mutex> lk(mem_manager_mutex);
			result = vkBindBufferMemory(*deviceCached, buffer, std::get<2>(*allocEntry.begin()).mem, std::get<2>(*allocEntry.begin()).offset);}
			return result;
		}
	}

	VkResult BindImageMemoryCWrapper(VkImage image, uint64_t allocId)
	{
		int memType;
		std::vector<std::tuple<int, unsigned long long, SubAlloc>> allocEntry;
		{ std::lock_guard <std::mutex> lk(mem_manager_mutex); 
		memType = AllocMemCWrapperDirectoryMemType[allocId];
		allocEntry = AllocMemCWrapperDirectory[allocId]; }
		if (Instance.SupportsSparseResources() && memType == 1 && false) // LibKTX2 *still* has no sparse binding support. Can someone other than me do this please?
		{
			FenceClass sparseFence;
			sparseFence.Fence(&Instance);
			sparseFence.Reset();
			VkSparseImageOpaqueMemoryBindInfo imageMemoryBinds;
			imageMemoryBinds.image = image;
			std::vector<VkSparseMemoryBind> memoryBinds;
			unsigned long long resourceOffset = 0ull;
			for (std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>& curSubAlloc : allocEntry)
			{
				VkSparseMemoryBind curImgBind = {};
				curImgBind.memory = std::get<2>(curSubAlloc).mem;
				curImgBind.memoryOffset = (VkDeviceSize)std::get<2>(curSubAlloc).offset;
				curImgBind.size = (VkDeviceSize)std::get<2>(curSubAlloc).len;
				curImgBind.resourceOffset = (VkDeviceSize)resourceOffset;
				curImgBind.flags = VK_SPARSE_MEMORY_BIND_METADATA_BIT;
				resourceOffset += std::get<2>(curSubAlloc).len;
				memoryBinds.push_back(curImgBind);
			}
			imageMemoryBinds.bindCount = (uint32_t)memoryBinds.size();
			imageMemoryBinds.pBinds = memoryBinds.data();
			VkBindSparseInfo vkBindSparseInfo = {};
			vkBindSparseInfo.sType = VK_STRUCTURE_TYPE_BIND_SPARSE_INFO;
			vkBindSparseInfo.imageOpaqueBindCount = 1;
			vkBindSparseInfo.pImageOpaqueBinds = &imageMemoryBinds;
			VkResult result;
			{std::unique_lock <std::mutex> lk(mem_manager_mutex);
			Instance.allQueues[TRANSFER_QUEUE].mtx.lock();
			result = vkQueueBindSparse(Instance.allQueues[TRANSFER_QUEUE].queue, 1, &vkBindSparseInfo, sparseFence.fence);
			Instance.allQueues[TRANSFER_QUEUE].mtx.unlock();}
			sparseFence.Wait();
			return result;
		}
		else
		{
			VkResult result;
			{std::unique_lock <std::mutex> lk(mem_manager_mutex);
			result = vkBindImageMemory(*deviceCached, image, std::get<2>(*allocEntry.begin()).mem, std::get<2>(*allocEntry.begin()).offset);}
			return result;
		}
	}

	VkResult MapMemoryCWrapper(uint64_t allocId, uint64_t pageNumber, VkDeviceSize *mapLength, void** dataPtr)
	{
		mem_manager_mutex.lock();
		if (AllocMemCWrapperDirectory.find(allocId) == AllocMemCWrapperDirectory.end() || pageNumber >= AllocMemCWrapperDirectory[allocId].size()) { mem_manager_mutex.unlock(); return VK_ERROR_MEMORY_MAP_FAILED; }
		*mapLength = std::get<2>(AllocMemCWrapperDirectory[allocId][pageNumber]).len;
		VkDeviceMemory devMem = std::get<2>(AllocMemCWrapperDirectory[allocId][pageNumber]).mem;
		unsigned long long mapOffset = std::get<2>(AllocMemCWrapperDirectory[allocId][pageNumber]).offset;
		mem_manager_mutex.unlock();
		return Instance.ThreadSafeMapMemory(devMem, mapOffset, *mapLength, 0, dataPtr);
	}

	void UnmapMemoryCWrapper(uint64_t allocId, uint64_t pageNumber)
	{
		mem_manager_mutex.lock();
		if (AllocMemCWrapperDirectory.find(allocId) == AllocMemCWrapperDirectory.end() || pageNumber >= AllocMemCWrapperDirectory[allocId].size()) { mem_manager_mutex.unlock(); return; }
		VkDeviceMemory devMem = std::get<2>(AllocMemCWrapperDirectory[allocId][pageNumber]).mem;
		mem_manager_mutex.unlock();
		Instance.ThreadSafeUnmapMemory(devMem);
	}

	void FreeMemCWrapper(uint64_t allocId)
	{
		mem_manager_mutex.lock();
		if (AllocMemCWrapperDirectory.find(allocId) == AllocMemCWrapperDirectory.end()) { mem_manager_mutex.unlock(); return; }
		std::vector<std::tuple<int, unsigned long long, SubAlloc>> allocEntry = AllocMemCWrapperDirectory[allocId];
		mem_manager_mutex.unlock();
		FreeMem(allocEntry, *deviceCached);
		{ std::lock_guard <std::mutex> lk(mem_manager_mutex);
		AllocMemCWrapperDirectory.erase(allocId);
		AllocMemCWrapperDirectoryMemType.erase(allocId); }
	}

	void LockQueue(VkQueue)
	{
		Instance.allQueues[TRANSFER_QUEUE].mtx.lock();
	}

	void UnlockQueue(VkQueue)
	{
		Instance.allQueues[TRANSFER_QUEUE].mtx.unlock();
	}
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

VkBool32 HIGHOMEGA::GL::DEBUG_MESSAGE(VkDebugReportFlagsEXT flags,VkDebugReportObjectTypeEXT objType,uint64_t srcObject,size_t location,
											 int32_t msgCode,const char* pLayerPrefix,const char* pMsg,void* pUserData)
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

void WindowClass::Make(std::string appName, int startx, int starty, int w, int h, GL_WINDOWED_MODE windowedMode)
{
	this->appName = appName;
	this->startx = startx;
	this->starty = starty;
	this->w = w;
	this->h = h;
	this->windowedMode = windowedMode;

	if (SDL_Init(SDL_INIT_VIDEO) != 0) { RemovePast(); FATAL_ERROR("Could not initialize SDL"); }
	haveWindow = true;

	unsigned int windowedFlags = SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_SHOWN;
	switch (windowedMode)
	{
		case GL_WINDOWED_FULLSCREEN:
		{
			windowedFlags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
			break;
		}
		case GL_FULLSCREEN:
		{
			windowedFlags |= SDL_WINDOW_FULLSCREEN;
			break;
		}
	}
	win = SDL_CreateWindow(appName.c_str(), startx, starty, w, h, windowedFlags);
	if (win == nullptr) { RemovePast();  FATAL_ERROR("Could not create window"); }
	haveRenderer = true;

	SDL_SetWindowGrab(win, SDL_TRUE);

	struct SDL_SysWMinfo wmInfo;
	SDL_VERSION(&wmInfo.version);

	if (SDL_GetWindowWMInfo(win, &wmInfo) == -1) { RemovePast(); FATAL_ERROR("Could not get window information"); }

	window = wmInfo.info.win.window;
	hInstance = GetModuleHandle(nullptr);
}

void HIGHOMEGA::GL::WindowClass::Recreate(int w, int h, GL_WINDOWED_MODE windowedMode)
{
	this->w = w;
	this->h = h;
	SDL_SetWindowSize(win, w, h);
	this->windowedMode = windowedMode;
	unsigned int windowedFlags = 0u;
	switch (windowedMode)
	{
		case GL_WINDOWED_FULLSCREEN:
		{
			windowedFlags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
			break;
		}
		case GL_FULLSCREEN:
		{
			windowedFlags |= SDL_WINDOW_FULLSCREEN;
			break;
		}
	}
	SDL_SetWindowFullscreen(win, windowedFlags);
}

int HIGHOMEGA::GL::WindowClass::GetWidth()
{
	return w;
}

int HIGHOMEGA::GL::WindowClass::GetHeight()
{
	return h;
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

	CreateSwapChainRemovePast();

	for (std::pair<const std::string, Raster_PSO_DSL> & curPSODSL : globalRaster_PSO_DSL_Cache)
	{
		delete curPSODSL.second.DSL;
		delete curPSODSL.second.PSO;
		if (curPSODSL.second.PSOMainPass) delete curPSODSL.second.PSOMainPass;
	}
	for (std::pair<const std::string, Compute_PSO_DSL> & curPSODSL : globalCompute_PSO_DSL_Cache)
	{
		delete curPSODSL.second.DSL;
		delete curPSODSL.second.PSO;
	}
	if (haveDebugCallback) DestroyDebugReportCallback (instance, msgCallback, nullptr);
	if (haveSurface) vkDestroySurfaceKHR(instance, surface, nullptr);
	if (haveDevice) vkDestroyDevice(device, nullptr);
	if (haveInstance) vkDestroyInstance(instance, nullptr);

	haveSurface = false;
	haveDevice = false;
	haveInstance = false;
	haveDebugCallback = false;
}

VkResult HIGHOMEGA::GL::InstanceClass::ThreadSafeMapMemory(VkDeviceMemory inMem, VkDeviceSize offset, VkDeviceSize len, VkMemoryMapFlags flags, void** ppData)
{
	while (true)
	{
		mem_manager_mutex.lock();
		if (allMemMaps.find(inMem) == allMemMaps.end())
		{
			allMemMaps[inMem].offset = offset;
			allMemMaps[inMem].len = len;
			allMemMaps[inMem].refCount = 1;
			VkResult result = vkMapMemory(device, inMem, offset, len, 0, &allMemMaps[inMem].dataPtr);
			*ppData = allMemMaps[inMem].dataPtr;
			mem_manager_mutex.unlock();
			return result;
		}
		else if (allMemMaps[inMem].len != len || allMemMaps[inMem].offset != offset)
		{
			mem_manager_mutex.unlock();
		}
		else
		{
			allMemMaps[inMem].refCount++;
			*ppData = allMemMaps[inMem].dataPtr;
			mem_manager_mutex.unlock();
			return VK_SUCCESS;
		}
	}
}

void HIGHOMEGA::GL::InstanceClass::ThreadSafeUnmapMemory(VkDeviceMemory inMem)
{
	std::unique_lock <std::mutex> lk(mem_manager_mutex);
	if (allMemMaps.find(inMem) == allMemMaps.end()) return;
	if (allMemMaps[inMem].refCount == 0) FATAL_ERROR("Thread-safe memory unmap with a ref count of 0");
	allMemMaps[inMem].refCount--;
	if (allMemMaps[inMem].refCount == 0)
	{
		vkUnmapMemory(device, inMem);
		allMemMaps.erase(inMem);
	}
}

void InstanceClass::CreateSwapChainRemovePast()
{
	frameBuffer.RemovePast();
	swapChainImages.clear();
	depthStencilImage.RemovePast();
	if (haveSwapChain) fpDestroySwapchainKHR(device, swapChain, nullptr);
	haveSwapChain = false;
}

InstanceClass::InstanceClass()
{
	haveSurface = false;
	haveDevice = false;
	haveInstance = false;
	haveDebugCallback = false;
	haveSwapChain = false;
}

void InstanceClass::Make(bool validationLayer,WindowClass &inpWindow, bool requestHWRT, bool headless)
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
	if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Create instance failed"); }
	haveInstance = true;

	uint32_t gpuCount = 0;
	result = vkEnumeratePhysicalDevices(instance, &gpuCount, nullptr);
	if (result != VK_SUCCESS || gpuCount == 0) { RemovePast(); FATAL_ERROR("Could not get physical devices"); }

	std::vector<VkPhysicalDevice> physicalDevices;
	physicalDevices = std::vector<VkPhysicalDevice>(gpuCount);
	result = vkEnumeratePhysicalDevices(instance, &gpuCount, physicalDevices.data());
	if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not enumerate physical devices"); }

	physicalDevice = physicalDevices[0];

	uint32_t graphicsQueueIndex = 0, transferQueueIndex = 0, computeQueueIndex = 0;
	uint32_t queueCount;
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueCount, VK_NULL_HANDLE);
	if (queueCount == 0) { RemovePast(); FATAL_ERROR("No queues found at all"); }

	std::vector<VkQueueFamilyProperties> queueProps;
	queueProps.resize(queueCount);
	vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueCount, queueProps.data());

	for (graphicsQueueIndex = 0; graphicsQueueIndex < queueCount; graphicsQueueIndex++)
		if (queueProps[graphicsQueueIndex].queueFlags & VK_QUEUE_GRAPHICS_BIT && queueProps[graphicsQueueIndex].queueFlags & VK_QUEUE_COMPUTE_BIT) break;

	if (graphicsQueueIndex == queueCount) { RemovePast(); FATAL_ERROR("No graphics queue found"); }

	for (transferQueueIndex = 0; transferQueueIndex < queueCount; transferQueueIndex++)
		if (queueProps[transferQueueIndex].queueFlags & VK_QUEUE_TRANSFER_BIT && !(queueProps[transferQueueIndex].queueFlags & VK_QUEUE_GRAPHICS_BIT) && !(queueProps[transferQueueIndex].queueFlags & VK_QUEUE_COMPUTE_BIT)) break;

	if (transferQueueIndex == queueCount) { RemovePast(); FATAL_ERROR("No transfer queue found"); }

	for (computeQueueIndex = 0; computeQueueIndex < queueCount; computeQueueIndex++)
		if (queueProps[computeQueueIndex].queueFlags & VK_QUEUE_TRANSFER_BIT && !(queueProps[computeQueueIndex].queueFlags & VK_QUEUE_GRAPHICS_BIT) && queueProps[computeQueueIndex].queueFlags & VK_QUEUE_COMPUTE_BIT) break;

	if (computeQueueIndex == queueCount) { RemovePast(); FATAL_ERROR("No compute queue found. If you're on an Intel Arc card, please note that unfortunately this vendor is not currently supported."); }

	std::array<float, 1> queuePriorities = { 1.0f };
	VkDeviceQueueCreateInfo queueCreateInfo[3];
	queueCreateInfo[0] = {};
	queueCreateInfo[0].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queueCreateInfo[0].queueFamilyIndex = graphicsQueueIndex;
	queueCreateInfo[0].queueCount = 1;
	queueCreateInfo[0].pQueuePriorities = queuePriorities.data();
	queueCreateInfo[1] = {};
	queueCreateInfo[1].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queueCreateInfo[1].queueFamilyIndex = computeQueueIndex;
	queueCreateInfo[1].queueCount = 1;
	queueCreateInfo[1].pQueuePriorities = queuePriorities.data();
	queueCreateInfo[2] = {};
	queueCreateInfo[2].sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
	queueCreateInfo[2].queueFamilyIndex = transferQueueIndex;
	queueCreateInfo[2].queueCount = 1;
	queueCreateInfo[2].pQueuePriorities = queuePriorities.data();

	std::vector<const char*> enabledDeviceExtensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME,
														 VK_KHR_MAINTENANCE3_EXTENSION_NAME,
														 VK_EXT_DESCRIPTOR_INDEXING_EXTENSION_NAME,
														 VK_KHR_DESCRIPTOR_UPDATE_TEMPLATE_EXTENSION_NAME,
														 VK_KHR_GET_MEMORY_REQUIREMENTS_2_EXTENSION_NAME,
														 VK_KHR_SPIRV_1_4_EXTENSION_NAME,
														 VK_KHR_SHADER_FLOAT_CONTROLS_EXTENSION_NAME,
														 VK_KHR_RAY_QUERY_EXTENSION_NAME,
														 VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
														 VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
														 VK_KHR_PIPELINE_LIBRARY_EXTENSION_NAME,
														 VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
														 VK_KHR_BUFFER_DEVICE_ADDRESS_EXTENSION_NAME,
														 VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME,
														 VK_KHR_SHADER_FLOAT16_INT8_EXTENSION_NAME,
														 VK_KHR_16BIT_STORAGE_EXTENSION_NAME,
														 VK_KHR_8BIT_STORAGE_EXTENSION_NAME,
														 VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME,
														 VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME };

	VkPhysicalDeviceFeatures deviceFeatures = {};
	deviceFeatures.samplerAnisotropy = VK_TRUE;
	deviceFeatures.depthClamp = VK_TRUE;
	deviceFeatures.geometryShader = VK_TRUE;
	deviceFeatures.shaderStorageImageExtendedFormats = VK_TRUE;
	deviceFeatures.tessellationShader = VK_TRUE;
	deviceFeatures.vertexPipelineStoresAndAtomics = VK_TRUE;
	deviceFeatures.fragmentStoresAndAtomics = VK_TRUE;
	deviceFeatures.shaderInt16 = VK_TRUE;
	deviceFeatures.multiDrawIndirect = VK_TRUE;
	deviceFeatures.drawIndirectFirstInstance = VK_TRUE;
	//deviceFeatures.sparseBinding = VK_TRUE;
	supportsSparseResources = false; // Sparse bindings did NOT solve anything...

	VkPhysicalDevice8BitStorageFeaturesKHR VkPhysicalDevice8BitStorageFeatures = {};
	VkPhysicalDevice8BitStorageFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_8BIT_STORAGE_FEATURES;
	VkPhysicalDevice8BitStorageFeatures.pNext = VK_NULL_HANDLE;
	VkPhysicalDevice8BitStorageFeatures.storageBuffer8BitAccess = VK_TRUE;
	VkPhysicalDevice8BitStorageFeatures.uniformAndStorageBuffer8BitAccess = VK_TRUE;

	VkPhysicalDevice16BitStorageFeaturesKHR VkPhysicalDevice16BitStorageFeatures = {};
	VkPhysicalDevice16BitStorageFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES;
	VkPhysicalDevice16BitStorageFeatures.pNext = &VkPhysicalDevice8BitStorageFeatures;
	VkPhysicalDevice16BitStorageFeatures.storageBuffer16BitAccess = VK_TRUE;
	VkPhysicalDevice16BitStorageFeatures.uniformAndStorageBuffer16BitAccess = VK_TRUE;

	VkPhysicalDeviceShaderFloat16Int8FeaturesKHR VkPhysicalDeviceShaderFloat16Int8Features = {};
	VkPhysicalDeviceShaderFloat16Int8Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_FLOAT16_INT8_FEATURES;
	VkPhysicalDeviceShaderFloat16Int8Features.pNext = &VkPhysicalDevice16BitStorageFeatures;
	VkPhysicalDeviceShaderFloat16Int8Features.shaderFloat16 = VK_TRUE;
	VkPhysicalDeviceShaderFloat16Int8Features.shaderInt8 = VK_TRUE;

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

	VkPhysicalDeviceRayQueryFeaturesKHR VkPhysicalDeviceRayQueryFeatures;
	VkPhysicalDeviceRayQueryFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_RAY_QUERY_FEATURES_KHR;
	VkPhysicalDeviceRayQueryFeatures.pNext = &vkPhysicalDeviceRayTracingPipelineFeatures;
	VkPhysicalDeviceRayQueryFeatures.rayQuery = VK_TRUE;

	VkPhysicalDeviceBufferDeviceAddressFeatures vkPhysicalDeviceBufferAddressFeatures;
	vkPhysicalDeviceBufferAddressFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_BUFFER_DEVICE_ADDRESS_FEATURES;
	vkPhysicalDeviceBufferAddressFeatures.pNext = &VkPhysicalDeviceRayQueryFeatures;
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
	deviceDescriptorIndexingFeature.shaderUniformBufferArrayNonUniformIndexing = VK_TRUE;

	VkPhysicalDeviceTimelineSemaphoreFeatures timelineFeatures{};
	timelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
	timelineFeatures.pNext = &deviceDescriptorIndexingFeature;
	timelineFeatures.timelineSemaphore = VK_TRUE;

	VkDeviceCreateInfo deviceCreateInfo = {};
	deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	deviceCreateInfo.pNext = &timelineFeatures;
	deviceCreateInfo.queueCreateInfoCount = 3;
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
									VK_EXT_SCALAR_BLOCK_LAYOUT_EXTENSION_NAME,
									VK_KHR_16BIT_STORAGE_EXTENSION_NAME,
									VK_KHR_8BIT_STORAGE_EXTENSION_NAME,
									VK_KHR_DRAW_INDIRECT_COUNT_EXTENSION_NAME,
									VK_KHR_TIMELINE_SEMAPHORE_EXTENSION_NAME };

		VkPhysicalDevice8BitStorageFeaturesKHR VkPhysicalDevice8BitStorageFeatures = {};
		VkPhysicalDevice8BitStorageFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_8BIT_STORAGE_FEATURES;
		VkPhysicalDevice8BitStorageFeatures.pNext = VK_NULL_HANDLE;
		VkPhysicalDevice8BitStorageFeatures.storageBuffer8BitAccess = VK_TRUE;
		VkPhysicalDevice8BitStorageFeatures.uniformAndStorageBuffer8BitAccess = VK_TRUE;

		VkPhysicalDevice16BitStorageFeaturesKHR VkPhysicalDevice16BitStorageFeatures = {};
		VkPhysicalDevice16BitStorageFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_16BIT_STORAGE_FEATURES;
		VkPhysicalDevice16BitStorageFeatures.pNext = &VkPhysicalDevice8BitStorageFeatures;
		VkPhysicalDevice16BitStorageFeatures.storageBuffer16BitAccess = VK_TRUE;
		VkPhysicalDevice16BitStorageFeatures.uniformAndStorageBuffer16BitAccess = VK_TRUE;

		VkPhysicalDeviceShaderFloat16Int8FeaturesKHR VkPhysicalDeviceShaderFloat16Int8Features = {};
		VkPhysicalDeviceShaderFloat16Int8Features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SHADER_FLOAT16_INT8_FEATURES;
		VkPhysicalDeviceShaderFloat16Int8Features.pNext = &VkPhysicalDevice16BitStorageFeatures;
		VkPhysicalDeviceShaderFloat16Int8Features.shaderFloat16 = VK_FALSE;
		VkPhysicalDeviceShaderFloat16Int8Features.shaderInt8 = VK_TRUE;

		VkPhysicalDeviceScalarBlockLayoutFeatures physicalDeviceScalarBlockLayoutFeatures;
		physicalDeviceScalarBlockLayoutFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
		physicalDeviceScalarBlockLayoutFeatures.pNext = &VkPhysicalDeviceShaderFloat16Int8Features;
		physicalDeviceScalarBlockLayoutFeatures.scalarBlockLayout = VK_TRUE;

		VkPhysicalDeviceDescriptorIndexingFeaturesEXT deviceDescriptorIndexingFeature = {};
		deviceDescriptorIndexingFeature.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_FEATURES_EXT;
		deviceDescriptorIndexingFeature.pNext = &physicalDeviceScalarBlockLayoutFeatures;
		deviceDescriptorIndexingFeature.descriptorBindingVariableDescriptorCount = VK_TRUE;
		deviceDescriptorIndexingFeature.runtimeDescriptorArray = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderSampledImageArrayNonUniformIndexing = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderStorageBufferArrayNonUniformIndexing = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderStorageImageArrayNonUniformIndexing = VK_TRUE;
		deviceDescriptorIndexingFeature.shaderUniformBufferArrayNonUniformIndexing = VK_TRUE;

		VkPhysicalDeviceTimelineSemaphoreFeatures timelineFeatures{};
		timelineFeatures.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
		timelineFeatures.pNext = &deviceDescriptorIndexingFeature;
		timelineFeatures.timelineSemaphore = VK_TRUE;

		deviceCreateInfo.enabledExtensionCount = (uint32_t)enabledDeviceExtensions.size();
		deviceCreateInfo.ppEnabledExtensionNames = enabledDeviceExtensions.data();
		deviceCreateInfo.pNext = &timelineFeatures;
		result = vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &device);
		if (result != VK_SUCCESS)
		{
			RemovePast();
			FATAL_ERROR("Could not create Vulkan device");
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
	LOG() << "Reported VRAM size: " << toStringPrecision(gbInDouble, 2) << "GB";
	if (gbInDouble < 3.7)
		lowMemoryDevice = true;

	vkGetDeviceQueue(device, graphicsQueueIndex, 0, &allQueues[GRAPHICS_QUEUE].queue);
	vkGetDeviceQueue(device, transferQueueIndex, 0, &allQueues[TRANSFER_QUEUE].queue);
	vkGetDeviceQueue(device, computeQueueIndex, 0, &allQueues[COMPUTE_QUEUE].queue);
	allQueues[GRAPHICS_QUEUE].name = "graphics";
	allQueues[TRANSFER_QUEUE].name = "transfer";
	allQueues[COMPUTE_QUEUE].name = "compute";

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

	if (foundFormat == false) { RemovePast(); FATAL_ERROR("No supported format found"); }

	fpGetPhysicalDeviceSurfaceSupportKHR = (PFN_vkGetPhysicalDeviceSurfaceSupportKHR)vkGetInstanceProcAddr (instance,"vkGetPhysicalDeviceSurfaceSupportKHR");
	fpGetPhysicalDeviceSurfaceCapabilitiesKHR = (PFN_vkGetPhysicalDeviceSurfaceCapabilitiesKHR)vkGetInstanceProcAddr (instance, "vkGetPhysicalDeviceSurfaceCapabilitiesKHR");
	fpGetPhysicalDeviceSurfaceFormatsKHR = (PFN_vkGetPhysicalDeviceSurfaceFormatsKHR)vkGetInstanceProcAddr (instance, "vkGetPhysicalDeviceSurfaceFormatsKHR");
	fpGetPhysicalDeviceSurfacePresentModesKHR = (PFN_vkGetPhysicalDeviceSurfacePresentModesKHR)vkGetInstanceProcAddr (instance, "vkGetPhysicalDeviceSurfacePresentModesKHR");

	fpCreateSwapchainKHR = (PFN_vkCreateSwapchainKHR)vkGetDeviceProcAddr (device,"vkCreateSwapchainKHR");
	fpDestroySwapchainKHR = (PFN_vkDestroySwapchainKHR)vkGetDeviceProcAddr (device,"vkDestroySwapchainKHR");
	fpGetSwapchainImagesKHR = (PFN_vkGetSwapchainImagesKHR)vkGetDeviceProcAddr (device,"vkGetSwapchainImagesKHR");
	fpAcquireNextImageKHR = (PFN_vkAcquireNextImageKHR)vkGetDeviceProcAddr (device,"vkAcquireNextImageKHR");
	fpQueuePresentKHR = (PFN_vkQueuePresentKHR)vkGetDeviceProcAddr (device,"vkQueuePresentKHR");
	fpWaitSemaphoresKHR = (PFN_vkWaitSemaphoresKHR)vkGetDeviceProcAddr(device, "vkWaitSemaphoresKHR");

	fpCreateDescriptorUpdateTemplateKHR = (PFN_vkCreateDescriptorUpdateTemplateKHR)vkGetDeviceProcAddr (device, "vkCreateDescriptorUpdateTemplateKHR");
	fpUpdateDescriptorSetWithTemplateKHR = (PFN_vkUpdateDescriptorSetWithTemplateKHR)vkGetDeviceProcAddr(device, "vkUpdateDescriptorSetWithTemplateKHR");
	fpDestroyDescriptorUpdateTemplateKHR = (PFN_vkDestroyDescriptorUpdateTemplateKHR)vkGetDeviceProcAddr(device, "vkDestroyDescriptorUpdateTemplateKHR");

	fpCmdDrawIndexedIndirectCountKHR = (PFN_vkCmdDrawIndexedIndirectCountKHR)vkGetDeviceProcAddr(device, "vkCmdDrawIndexedIndirectCountKHR");

	if (fpGetPhysicalDeviceSurfaceSupportKHR == VK_NULL_HANDLE ||
		fpGetPhysicalDeviceSurfaceCapabilitiesKHR == VK_NULL_HANDLE ||
		fpGetPhysicalDeviceSurfaceFormatsKHR == VK_NULL_HANDLE ||
		fpGetPhysicalDeviceSurfacePresentModesKHR == VK_NULL_HANDLE ||
		fpCreateSwapchainKHR == VK_NULL_HANDLE ||
		fpDestroySwapchainKHR == VK_NULL_HANDLE ||
		fpGetSwapchainImagesKHR == VK_NULL_HANDLE ||
		fpAcquireNextImageKHR == VK_NULL_HANDLE ||
		fpQueuePresentKHR == VK_NULL_HANDLE ||
		fpWaitSemaphoresKHR == VK_NULL_HANDLE ||
		fpCreateDescriptorUpdateTemplateKHR == VK_NULL_HANDLE ||
		fpUpdateDescriptorSetWithTemplateKHR == VK_NULL_HANDLE ||
		fpDestroyDescriptorUpdateTemplateKHR == VK_NULL_HANDLE)
	{
		RemovePast();
		FATAL_ERROR("Could not get function pointers");
	}

	VkSemaphoreCreateInfo semaphoreCreateInfo = {};
	semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	semaphoreCreateInfo.pNext = VK_NULL_HANDLE;
	semaphoreCreateInfo.flags = 0;
	try { acquireImageFence.Fence(this); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not create display image acquisition fence"); }

	if (!headless)
	{
		VkWin32SurfaceCreateInfoKHR surfaceCreateInfo = {};
		surfaceCreateInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
		surfaceCreateInfo.hinstance = inpWindow.hInstance;
		surfaceCreateInfo.hwnd = inpWindow.window;
		result = vkCreateWin32SurfaceKHR(instance, &surfaceCreateInfo, nullptr, &surface);
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not create surface"); }
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
		if ((queueProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) != 0 && (queueProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0 && (queueProps[i].queueFlags & VK_QUEUE_TRANSFER_BIT) != 0)
		{
			if (allQueues[GRAPHICS_QUEUE].nodeIndex == UINT32_MAX)
				allQueues[GRAPHICS_QUEUE].nodeIndex = i;

			if (!headless && supportsPresent[i] == VK_TRUE)
			{
				allQueues[GRAPHICS_QUEUE].nodeIndex = i;
				presentQueueNodeIndex = i;
				break;
			}
		}
	}
	for (uint32_t i = 0; i < queueCount; i++)
	{
		if ((queueProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0 && (queueProps[i].queueFlags & VK_QUEUE_TRANSFER_BIT) != 0 && (queueProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0)
		{
			if (allQueues[COMPUTE_QUEUE].nodeIndex == UINT32_MAX)
				allQueues[COMPUTE_QUEUE].nodeIndex = i;
		}
	}
	for (uint32_t i = 0; i < queueCount; i++)
	{
		if ((queueProps[i].queueFlags & VK_QUEUE_TRANSFER_BIT) != 0 && (queueProps[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) == 0 && (queueProps[i].queueFlags & VK_QUEUE_COMPUTE_BIT) == 0)
		{
			if (allQueues[TRANSFER_QUEUE].nodeIndex == UINT32_MAX)
				allQueues[TRANSFER_QUEUE].nodeIndex = i;
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
		if (allQueues[GRAPHICS_QUEUE].nodeIndex == UINT32_MAX || allQueues[COMPUTE_QUEUE].nodeIndex == UINT32_MAX || presentQueueNodeIndex == UINT32_MAX || allQueues[TRANSFER_QUEUE].nodeIndex == UINT32_MAX)
		{
			RemovePast();
			FATAL_ERROR("A graphics, present, compute or transfer queue not found.");
		}

		if (allQueues[GRAPHICS_QUEUE].nodeIndex != presentQueueNodeIndex)
		{
			RemovePast();
			FATAL_ERROR("Separate graphics and present queues not supported");
		}
	}
	else
	{
		if (allQueues[GRAPHICS_QUEUE].nodeIndex == UINT32_MAX || allQueues[COMPUTE_QUEUE].nodeIndex == UINT32_MAX || allQueues[TRANSFER_QUEUE].nodeIndex == UINT32_MAX)
		{
			RemovePast();
			FATAL_ERROR("A graphics, compute or transfer queue not found.");
		}
	}

	if (!headless)
	{
		uint32_t formatCount;
		result = fpGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, VK_NULL_HANDLE);
		if (result != VK_SUCCESS || formatCount == 0) { RemovePast(); FATAL_ERROR("Could not get any surface formats"); }

		std::vector<VkSurfaceFormatKHR> surfaceFormats;
		surfaceFormats = std::vector<VkSurfaceFormatKHR>(formatCount);
		result = fpGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, surfaceFormats.data());
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not retrieve details for surface formats"); }

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
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not register debug callback"); }
		haveDebugCallback = true;
	}
}

void InstanceClass::CreateSwapChain(WindowClass &windowRef)
{
	VkResult result;
	VkSwapchainKHR oldSwapchain = swapChain;

	VkSurfaceCapabilitiesKHR surfCaps;
	result = fpGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &surfCaps);
	if (result != VK_SUCCESS) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not get surface capabilities"); }

	uint32_t presentModeCount;
	result = fpGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, VK_NULL_HANDLE);
	if (result != VK_SUCCESS || presentModeCount == 0) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not get present mode count"); }

	std::vector<VkPresentModeKHR> presentModes;
	presentModes = std::vector<VkPresentModeKHR>(presentModeCount);

	result = fpGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, surface, &presentModeCount, presentModes.data());
	if (result != VK_SUCCESS) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not get present modes"); }

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
	swapchainCI.queueFamilyIndexCount = 1;
	swapchainCI.pQueueFamilyIndices = &allQueues[GRAPHICS_QUEUE].nodeIndex;
	swapchainCI.presentMode = swapchainPresentMode;
	swapchainCI.oldSwapchain = oldSwapchain;
	swapchainCI.clipped = true;
	swapchainCI.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;

	result = fpCreateSwapchainKHR(device, &swapchainCI, nullptr, &swapChain);
	if (result != VK_SUCCESS) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not create swapchain"); }
	haveSwapChain = true;

	if (oldSwapchain != VK_NULL_HANDLE)
	{
		frameBuffer.RemovePast();
		for (ImageClass& curImg : swapChainImages)
			curImg.RemovePast();
		swapChainImages.clear();
		depthStencilImage.RemovePast();
		fpDestroySwapchainKHR(device, oldSwapchain, nullptr);
	}

	try { swapChainImages = ImageClass::FromSwapChain(*this); }
	catch (...) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not get swap chain images"); }
	swapchainId = threadSafeMersenneTwister64Bit();

	try { depthStencilImage.CreateOnScreenDepthStencil(*this, windowRef.w, windowRef.h); }
	catch (...) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not create depth/stencil attachment"); }

	try {
		for (ImageClass& curImg : swapChainImages)
			frameBuffer.AddColorAttachment(curImg);
		frameBuffer.SetDepthStencil(depthStencilImage);
		frameBuffer.Create(ON_SCREEN, *this, windowRef);
	}
	catch (...) { CreateSwapChainRemovePast(); FATAL_ERROR("Could not create frame buffer"); }
}

void HIGHOMEGA::GL::InstanceClass::AddSwapchainDependentImage(ImageClass& inpImg, const std::function<unsigned int(unsigned int)>& surfaceBasedWidthCallback, const std::function<unsigned int(unsigned int)>& surfaceBasedHeightCallback)
{
	inpImg.surfaceBasedWidthCallback = surfaceBasedWidthCallback;
	inpImg.surfaceBasedHeightCallback = surfaceBasedHeightCallback;
	swapChainDependentImages.insert(&inpImg);
}

void HIGHOMEGA::GL::InstanceClass::RecreateSwapchainDependentImages(unsigned int width, unsigned int height)
{
	for (ImageClass* recreateImage : Instance.swapChainDependentImages)
	{
		recreateImage->RemovePast();
		if (recreateImage->surfaceBasedWidthCallback) recreateImage->width = recreateImage->surfaceBasedWidthCallback(width);
		if (recreateImage->surfaceBasedHeightCallback) recreateImage->height = recreateImage->surfaceBasedHeightCallback(height);
		recreateImage->CreateStandaloneImage(*this);
	}
}

bool HIGHOMEGA::GL::InstanceClass::isIntel()
{
	return deviceProps.vendorID == 0x8086;
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

void HIGHOMEGA::GL::InstanceClass::QueueFlush(QUEUE_TYPE queueToFlush)
{
    VkResult result;
    {std::unique_lock<std::mutex> lk(allQueues[queueToFlush].mtx);
    result = vkQueueWaitIdle(allQueues[queueToFlush].queue);}
    if (result != VK_SUCCESS)
    {
        if (result == VK_ERROR_OUT_OF_HOST_MEMORY) FATAL_ERROR("could not flush " + allQueues[queueToFlush].name + " queue: out of host memory");
        else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) FATAL_ERROR("could not flush " + allQueues[queueToFlush].name + " queue: out of device memory");
        else FATAL_ERROR("could not flush " + allQueues[queueToFlush].name + " queue: device lost");
    }
}

void HIGHOMEGA::GL::InstanceClass::GPUFlush()
{
	VkResult result;
    {std::unique_lock<std::mutex> lk(allQueues[GRAPHICS_QUEUE].mtx);
	std::unique_lock<std::mutex> lk2(allQueues[COMPUTE_QUEUE].mtx);
	std::unique_lock<std::mutex> lk3(allQueues[TRANSFER_QUEUE].mtx);
    result = vkDeviceWaitIdle(device);}
    if (result != VK_SUCCESS)
    {
        if (result == VK_ERROR_OUT_OF_HOST_MEMORY) FATAL_ERROR("Could not flush device: out of host memory");
        else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) FATAL_ERROR("Could not flush device: out of device memory");
        else FATAL_ERROR("Could not flush device: device lost");
    }
}

unsigned long long HIGHOMEGA::GL::InstanceClass::getSwapchainId()
{
	return swapchainId;
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

void HIGHOMEGA::GL::KHR_RT::RTInstance::Disable()
{
	rtEnabled = false;
}

bool HIGHOMEGA::GL::KHR_RT::RTInstance::Enabled()
{
	return rtEnabled;
}

void HIGHOMEGA::GL::ShaderResourceSet::StringToCString(char ** destCString, const std::string& inpString)
{
	*destCString = nullptr;
	*destCString = new char[strlen(inpString.c_str()) + 1];
	strcpy_s(*destCString, strlen(inpString.c_str()) + 1, inpString.c_str());
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, ImageClass & inpImageRef, int layer, SHADER_RESOURCE_USAGE inUsage)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = &inpImageRef;
	imageLayer = (layer >= 0) ? (layer+1) : 0;
	imageViewRef = nullptr;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource.clear();
	usage = inUsage;
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, BufferClass & inpUniformRef, SHADER_RESOURCE_USAGE inUsage)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	imageLayer = 0;
	uniformRef = &inpUniformRef;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource.clear();
	usage = inUsage;
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, FramebufferClass & inpFrameBuf, int attachmentNumber, SHADER_RESOURCE_USAGE inUsage)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = &inpFrameBuf.GetSampler();
	imageViewRef = inpFrameBuf.colorAttachments[attachmentNumber];
	imageLayer = 0;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource.clear();
	usage = inUsage;
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, KHR_RT::RTScene & inpSceneRef, SHADER_RESOURCE_USAGE inUsage)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	imageLayer = 0;
	uniformRef = nullptr;
	rtSceneRef = &inpSceneRef;
	isVariableCount = false;
	arrayedResource.clear();
	usage = inUsage;
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, std::vector<ShaderResource>& inpVariableCountResource, SHADER_RESOURCE_USAGE inUsage)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = 0;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	imageLayer = 0;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = true;
	arrayedResource = inpVariableCountResource;
	usage = inUsage;
}

HIGHOMEGA::GL::ShaderResource::ShaderResource(SHADER_RESOURCE_TYPE inpType, PIPELINE_STAGE inpVisibility, unsigned int inpSetId, unsigned int inpBindId, std::vector<ShaderResource>& inpVariableCountResource, SHADER_RESOURCE_USAGE inUsage)
{
	type = inpType;
	visibility = inpVisibility;
	bindId = inpBindId;
	setId = inpSetId;
	samplerRef = nullptr;
	imageViewRef = nullptr;
	imageLayer = 0;
	uniformRef = nullptr;
	rtSceneRef = nullptr;
	isVariableCount = false;
	arrayedResource = inpVariableCountResource;
	usage = inUsage;
}

unsigned int HIGHOMEGA::GL::ShaderResource::ResourceCount()
{
	return (unsigned int)arrayedResource.size() > 0 ? (unsigned int)arrayedResource.size() : 1;
}

std::unordered_set<SemaphoreClass*> HIGHOMEGA::GL::ShaderResource::GetAsDependencies()
{
	std::unordered_set<SemaphoreClass*> retVal;
	if (usage == SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY) return retVal;
	if (imageViewRef && samplerRef) retVal.insert(&imageViewRef->semaphore);
	else if (!imageViewRef && samplerRef) retVal.insert(&samplerRef->semaphore);
	else if (uniformRef) retVal.insert(&uniformRef->semaphore);
	else if (rtSceneRef) retVal.insert(&rtSceneRef->semaphore);
	for (ShaderResource& curRes : arrayedResource)
	{
		if (curRes.imageViewRef && curRes.samplerRef) retVal.insert(&curRes.imageViewRef->semaphore);
		else if (!curRes.imageViewRef && curRes.samplerRef) retVal.insert(&curRes.samplerRef->semaphore);
		else if (curRes.uniformRef) retVal.insert(&curRes.uniformRef->semaphore);
		else if (curRes.rtSceneRef) retVal.insert(&curRes.rtSceneRef->semaphore);
	}
	return retVal;
}

bool HIGHOMEGA::GL::ShaderResource::IsProduced()
{
	return usage == USAGE_PRODUCER || usage == USAGE_SIMULTANEOUS_PRODUCER;
}

bool HIGHOMEGA::GL::ShaderResource::IsSimultaneouslyProduced()
{
	return usage == USAGE_SIMULTANEOUS_PRODUCER;
}

bool HIGHOMEGA::GL::ShaderResource::IsSwapchainDependent()
{
	if (samplerRef && (samplerRef->getSurfaceBasedWidthCallback() || samplerRef->getSurfaceBasedHeightCallback())) return true;
	if (imageViewRef && (imageViewRef->getSurfaceBasedWidthCallback() || imageViewRef->getSurfaceBasedHeightCallback())) return true;
	for (ShaderResource& curRes : arrayedResource)
		if (curRes.IsSwapchainDependent()) return true;
	return false;
}

unsigned long long HIGHOMEGA::GL::ShaderResource::GetHash()
{
	using std::hash;
	size_t retVal = 0ull;
	retVal ^= hash<unsigned int>()(type);
	retVal ^= (hash<unsigned int>()(visibility) << 1);
	retVal ^= (hash<unsigned int>()(bindId) << 2);
	retVal ^= (hash<unsigned int>()(setId) << 3);
	retVal ^= (hash<unsigned int>()(imageLayer) << 4);
	retVal ^= (hash<bool>()(usage) << 5);
	if (samplerRef) retVal ^= hash<void*>()(samplerRef);
	if (imageViewRef) retVal ^= hash<void*>()(imageViewRef);
	if (uniformRef) retVal ^= hash<void*>()(uniformRef);
	if (rtSceneRef) retVal ^= hash<void*>()(rtSceneRef);
	for (ShaderResource& curRes : arrayedResource)
		retVal ^= curRes.GetHash();
	return retVal;
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(const std::string& inpComp, const std::string& inpCompEnt)
{
	compute_shader = inpComp;
	compute_entry = inpCompEnt;

	StringToCString(&compute_entry_cstr, inpCompEnt);
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(const std::string& inpVert, const std::string& inpVertEnt, const std::string& inpFrag, const std::string& inpFragEnt)
{
	vertex_shader = inpVert;
	vertex_entry = inpVertEnt;
	fragment_shader = inpFrag;
	fragment_entry = inpFragEnt;

	StringToCString(&vertex_entry_cstr, vertex_entry);
	StringToCString(&fragment_entry_cstr, fragment_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(const std::string& inpVert, const std::string& inpVertEnt, const std::string& inpGeom, const std::string& inpGeomEnt, const std::string& inpFrag, const std::string& inpFragEnt)
{
	Create(inpVert, inpVertEnt, inpFrag, inpFragEnt);

	geom_shader = inpGeom;
	geom_entry = inpGeomEnt;

	StringToCString(&geom_entry_cstr, geom_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::Create(const std::string& inpVert, const std::string& inpVertEnt, const std::string& inpTessCtrl, const std::string& inpTessCtrlEnt, const std::string& inpTessEval, const std::string& inpTessEvalEnt, const std::string& inpFrag, const std::string& inpFragEnt)
{
	Create(inpVert, inpVertEnt, inpFrag, inpFragEnt);

	tc_shader = inpTessCtrl;
	tc_entry = inpTessCtrlEnt;
	te_shader = inpTessEval;
	te_entry = inpTessEvalEnt;

	StringToCString(&tc_entry_cstr, tc_entry);
	StringToCString(&te_entry_cstr, te_entry);
}

void HIGHOMEGA::GL::ShaderResourceSet::CreateRT(const std::string& inpRaygen, const std::string& inpRaygenEnt, const std::string& inpRaychit, const std::string& inpRaychitEnt, const std::string& inpRaymiss, const std::string& inpRaymissEnt)
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

void HIGHOMEGA::GL::ShaderResourceSet::CreateRT(const std::string& inpRaygen, const std::string& inpRaygenEnt, const std::string& inpRaychit, const std::string& inpRaychitEnt, const std::string& inpRaymiss, const std::string& inpRaymissEnt, const std::string& inpRayahit, const std::string& inpRayahitEnt)
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

void HIGHOMEGA::GL::ShaderResourceSet::SetStageSpecializationData(PIPELINE_STAGE&& inStage, ShaderSpecilization inSpecializationData)
{
	stageSpecializationData[inStage] = inSpecializationData;
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

bool HIGHOMEGA::GL::ShaderResourceSet::areAdditionalResourcesSwapchainDependent()
{
	for (ShaderResource& curRes : getAdditionalResources())
		if (curRes.IsSwapchainDependent()) return true;
	return false;
}

ShaderSpecilization* HIGHOMEGA::GL::ShaderResourceSet::getStageSpecializationDataRef(PIPELINE_STAGE&& inStage)
{
	if (stageSpecializationData.find(inStage) == stageSpecializationData.end())
		return nullptr;
	else
		return &stageSpecializationData[inStage];
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
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create pipeline layout"); }
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
	std::string vertexKey, fragmentKey;
	try { shaderStages.push_back (AddOrFindCachedShaderStage(*ptrToInstance, inpShader.vertex_shader, inpShader.getVertEntry(), VK_SHADER_STAGE_VERTEX_BIT, inpShader.getStageSpecializationDataRef(VERTEX), vertexKey)->elem.stage); }
	catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create vertex shader"); }

	try { shaderStages.push_back (AddOrFindCachedShaderStage(*ptrToInstance, inpShader.fragment_shader, inpShader.getFragEntry(), VK_SHADER_STAGE_FRAGMENT_BIT, inpShader.getStageSpecializationDataRef(FRAGMENT), fragmentKey)->elem.stage); }
	catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create fragment shader"); }

	usedShaderStages.push_back(vertexKey);
	usedShaderStages.push_back(fragmentKey);

	if (inpShader.getTCEntry())
	{
		std::string tcKey;
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.tc_shader, inpShader.getTCEntry(), VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT, inpShader.getStageSpecializationDataRef(TESS_CTRL), tcKey)->elem.stage); }
		catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create tessellation control shader"); }
		usedShaderStages.push_back(tcKey);
	}

	if (inpShader.getTEEntry())
	{
		std::string teKey;
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.te_shader, inpShader.getTEEntry(), VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT, inpShader.getStageSpecializationDataRef(TESS_EVAL), teKey)->elem.stage); }
		catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create tessellation evaluation shader"); }
		usedShaderStages.push_back(teKey);
	}

	if (inpShader.getGeomEntry())
	{
		std::string geomKey;
		try { shaderStages.push_back (AddOrFindCachedShaderStage(*ptrToInstance, inpShader.geom_shader, inpShader.getGeomEntry(), VK_SHADER_STAGE_GEOMETRY_BIT, inpShader.getStageSpecializationDataRef(GEOMETRY), geomKey)->elem.stage); }
		catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create geometry shader"); }
		usedShaderStages.push_back(geomKey);
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
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create a pipeline cache"); }
	havePipelineCache = true;

	result = vkCreateGraphicsPipelines(ptrToInstance->device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline);
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create pipeline"); }
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
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create pipeline layout"); }
	havePipelineLayout = true;

	VkComputePipelineCreateInfo pipelineCreateInfo = {};
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
	pipelineCreateInfo.layout = pipelineLayout;

	std::string computeKey;
	try { pipelineCreateInfo.stage = AddOrFindCachedShaderStage(*ptrToInstance, inpShader.compute_shader, inpShader.getCompEntry(), VK_SHADER_STAGE_COMPUTE_BIT, inpShader.getStageSpecializationDataRef(COMPUTE), computeKey)->elem.stage; }
	catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create compute shader"); }
	usedShaderStage = computeKey;

	VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
	pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
	result = vkCreatePipelineCache(ptrToInstance->device, &pipelineCacheCreateInfo, nullptr, &pipelineCache);
	if (result != VK_SUCCESS) { FATAL_ERROR("Could not create a pipeline cache"); }
	havePipelineCache = true;

	result = vkCreateComputePipelines(ptrToInstance->device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline);
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create pipeline"); }
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
		if (readyForWait)
		{
			if (binary)
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
				{std::unique_lock<std::mutex> lk(ptrToInstance->allQueues[GRAPHICS_QUEUE].mtx);
				result = vkQueueSubmit(ptrToInstance->allQueues[GRAPHICS_QUEUE].queue, 1, &submitInfo, danglingSemFence.fence);}
				if (result != VK_SUCCESS) { FATAL_ERROR("Could not wait on dangling semaphore"); }

				danglingSemFence.Wait();
			}
			else if (!binary && signalValue > waitValue)
			{
				VkSemaphoreWaitInfo waitInfo;
				waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
				waitInfo.pNext = NULL;
				waitInfo.flags = 0;
				waitInfo.semaphoreCount = 1;
				waitInfo.pSemaphores = &semaphore;
				waitInfo.pValues = &signalValue;

				ptrToInstance->fpWaitSemaphoresKHR(ptrToInstance->device, &waitInfo, UINT64_MAX);
			}
			readyForWait = false;
		}
		vkDestroySemaphore(ptrToInstance->device, semaphore, nullptr);
	}
	haveSemaphore = false;
	binary = false;
	waitValue = 0ull;
	signalValue = 0ull;
}

void HIGHOMEGA::GL::SemaphoreClass::Semaphore(const SemaphoreClass& other)
{
	*this = other;
}

HIGHOMEGA::GL::SemaphoreClass::SemaphoreClass()
{
	haveSemaphore = false;
	binary = false;
	readyForWait = false;
	waitValue = 0ull;
	signalValue = 0ull;
}

void HIGHOMEGA::GL::SemaphoreClass::Semaphore(InstanceClass * inpPtrToInstance, bool inBinary)
{
	ptrToInstance = inpPtrToInstance;
	binary = inBinary;

	VkSemaphoreTypeCreateInfo timelineCreateInfo;
	timelineCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
	timelineCreateInfo.pNext = NULL;
	timelineCreateInfo.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
	timelineCreateInfo.initialValue = 0;

	VkSemaphoreCreateInfo semaphoreCreateInfo = {};
	semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	if (!binary) semaphoreCreateInfo.pNext = &timelineCreateInfo;
	VkResult result = vkCreateSemaphore(ptrToInstance->device, &semaphoreCreateInfo, nullptr, &semaphore);
	if (result != VK_SUCCESS) FATAL_ERROR("Could not create semaphore");

	haveSemaphore = true;
}

VkSemaphore& HIGHOMEGA::GL::SemaphoreClass::GetSemaphoreForPresent()
{
	if (!haveSemaphore) FATAL_ERROR("Trying to fetch a non-initialized semaphore for presentation.");
	if (!binary) FATAL_ERROR("Trying to fetch a timeline semaphore for presentation. This is not doable in Vulkan yet.");
	readyForWait = false;
	return semaphore;
}

HIGHOMEGA::GL::SemaphoreClass::~SemaphoreClass()
{
	RemovePast();
}

void HIGHOMEGA::GL::FenceClass::RemovePast()
{
	if (haveFence && ptrToInstance && ptrToInstance->haveDevice) vkDestroyFence(ptrToInstance->device, fence, nullptr);
	haveFence = false;
	ptrToInstance = nullptr;
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
	if (vkCreateFence(ptrToInstance->device, &fenceCreateInfo, nullptr, &fence) != VK_SUCCESS) FATAL_ERROR("Could not create fence");

	haveFence = true;
}

void HIGHOMEGA::GL::FenceClass::Wait()
{
	if (!ptrToInstance || !haveFence) FATAL_ERROR("fence not initialized for wait");

	VkResult result = vkWaitForFences(ptrToInstance->device, 1, &fence, VK_TRUE, UINT64_MAX);
	if (result != VK_SUCCESS)
	{
		if (result == VK_ERROR_OUT_OF_HOST_MEMORY) FATAL_ERROR("Could not wait on fence: out of host memory");
		else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) FATAL_ERROR("Could not wait on fence: out of device memory");
		else FATAL_ERROR("Could not wait on fence: device lost");
	}
}

void HIGHOMEGA::GL::FenceClass::Reset()
{
	if (!ptrToInstance || !haveFence) FATAL_ERROR("fence not initialized for reset");

	if (vkResetFences(ptrToInstance->device, 1, &fence) != VK_SUCCESS) FATAL_ERROR("Could not reset fence");
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
	if (indirectDrawBuffer)
	{
		delete indirectDrawBuffer;
		indirectDrawBuffer = nullptr;
	}
}

void RasterletClass::Rasterlet(InstanceClass & renderInst)
{
	ptrToInstance = &renderInst;
}

unsigned int HIGHOMEGA::GL::RasterletClass::IndirectBufferSize(unsigned int nInstances, unsigned int nBatches)
{
	return (unsigned int)((sizeof(VkDrawIndexedIndirectCommand) * nInstances + sizeof(unsigned int)) * nBatches);
}

unsigned int HIGHOMEGA::GL::RasterletClass::IndirectDrawCounterSize()
{
	return sizeof(unsigned int);
}

void HIGHOMEGA::GL::RasterletClass::ResetRasterlet()
{
	DestroyCommandBuffer();
	waitSems.clear();
	waitOldSems.clear();
	signalSems.clear();
}

void RasterletClass::PrepareSubmission(std::vector <PSO_DSL_DS_GeomInstPairing> & PSO_DSL_DS_GeomInstPairings, DynamicPipelineFlags &inpDynamicFlags, FramebufferClass & frameBuffer, FramebufferClass* frameBufferMainPass)
{
	ptrToFrameBuffer = &frameBuffer;

	if (ptrToFrameBuffer->getRenderMode() == OFF_SCREEN)
	{
		for (ImageClass* curAttach : ptrToFrameBuffer->colorAttachments)
		{
			if (doesCulling && PSO_DSL_DS_GeomInstPairings.size()) waitSems.insert(&curAttach->semaphore); // Since we produce twice, we have to wait on the last production
			signalSems.insert(&curAttach->semaphore);
		}
		if (ptrToFrameBuffer->GetDepthStencil())
		{
			if (doesCulling && PSO_DSL_DS_GeomInstPairings.size()) waitSems.insert(&ptrToFrameBuffer->GetDepthStencil()->semaphore); // Since we produce twice, we have to wait on the last production
			signalSems.insert(&ptrToFrameBuffer->GetDepthStencil()->semaphore);
		}
	}
	if (doesCulling && PSO_DSL_DS_GeomInstPairings.size())
		waitSems.insert(&indirectDrawBuffer->semaphore);

	int cmdBufCountMultiplier = 1;
	if (doesCulling) cmdBufCountMultiplier = 2; // Due to two pass occlusion culling
	unsigned int numCmdBufs = (ptrToFrameBuffer->mode == ON_SCREEN) ? ((int)frameBuffer.colorAttachments.size() * cmdBufCountMultiplier + 1) : cmdBufCountMultiplier;

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
	} catch (...) { FATAL_ERROR("Could not create swap chain command buffers"); }

	VkRenderPassBeginInfo renderPassBeginInfo = {};
	renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassBeginInfo.pNext = VK_NULL_HANDLE;
	renderPassBeginInfo.renderArea.offset.x = inpDynamicFlags.viewport_x;
	renderPassBeginInfo.renderArea.offset.y = inpDynamicFlags.viewport_y;
	renderPassBeginInfo.renderArea.extent.width = inpDynamicFlags.viewport_width;
	renderPassBeginInfo.renderArea.extent.height = inpDynamicFlags.viewport_height;

	if (frameBuffer.mode == ON_SCREEN)
	{
		unsigned int swapChainImageCount = (unsigned int)frameBuffer.colorAttachments.size();

		if (doesCulling)
		{
			renderPassBeginInfo.renderPass = ptrToFrameBuffer->renderPass;
			renderPassBeginInfo.clearValueCount = (uint32_t)clearValues.size();
			renderPassBeginInfo.pClearValues = clearValues.data();

			for (unsigned int i = 0; i != swapChainImageCount; i++)
				RecordCommandBuffers(PSO_DSL_DS_GeomInstPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, i, frameBuffer.swapChainBuffers[i], &frameBuffer.colorAttachments[i]->image, ON_SCREEN);

			renderPassBeginInfo.renderPass = frameBufferMainPass->renderPass;
			renderPassBeginInfo.clearValueCount = 0;
			renderPassBeginInfo.pClearValues = nullptr;

			for (unsigned int i = swapChainImageCount; i != swapChainImageCount * cmdBufCountMultiplier; i++)
				RecordCommandBuffers(PSO_DSL_DS_GeomInstPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, i, frameBufferMainPass->swapChainBuffers[i - swapChainImageCount], &frameBuffer.colorAttachments[i - swapChainImageCount]->image, ON_SCREEN, true);
		}
		else
		{
			renderPassBeginInfo.renderPass = ptrToFrameBuffer->renderPass;
			renderPassBeginInfo.clearValueCount = (uint32_t)clearValues.size();
			renderPassBeginInfo.pClearValues = clearValues.data();

			for (unsigned int i = 0; i != swapChainImageCount; i++)
				RecordCommandBuffers(PSO_DSL_DS_GeomInstPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, i, frameBuffer.swapChainBuffers[i], &frameBuffer.colorAttachments[i]->image, ON_SCREEN);
		}
	}
	else
	{
		if (doesCulling)
		{
			renderPassBeginInfo.renderPass = ptrToFrameBuffer->renderPass;
			renderPassBeginInfo.clearValueCount = (uint32_t)clearValues.size();
			renderPassBeginInfo.pClearValues = clearValues.data();

			RecordCommandBuffers(PSO_DSL_DS_GeomInstPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, 0, frameBuffer.attachmentsFrameBuffer, VK_NULL_HANDLE, OFF_SCREEN);

			renderPassBeginInfo.renderPass = frameBufferMainPass->renderPass;
			renderPassBeginInfo.clearValueCount = 0;
			renderPassBeginInfo.pClearValues = nullptr;

			RecordCommandBuffers(PSO_DSL_DS_GeomInstPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, 1, frameBufferMainPass->attachmentsFrameBuffer, VK_NULL_HANDLE, OFF_SCREEN, true);
		}
		else
		{
			renderPassBeginInfo.renderPass = ptrToFrameBuffer->renderPass;
			renderPassBeginInfo.clearValueCount = (uint32_t)clearValues.size();
			renderPassBeginInfo.pClearValues = clearValues.data();

			RecordCommandBuffers(PSO_DSL_DS_GeomInstPairings, inpDynamicFlags, renderPassBeginInfo, numCmdBufs, 0, frameBuffer.attachmentsFrameBuffer, VK_NULL_HANDLE, OFF_SCREEN);
		}
	}
}

void HIGHOMEGA::GL::RasterletClass::RecordCommandBuffers(std::vector<PSO_DSL_DS_GeomInstPairing>& PSO_DSL_DS_GeomInstPairings, DynamicPipelineFlags & inpDynamicFlags, VkRenderPassBeginInfo & renderPassBeginInfo, unsigned int numCmdBuf, unsigned int whichCmdBuf, VkFramebuffer & inpFrameBuffer, VkImage *inpImg, RENDER_MODE inpMode, bool mainPass)
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

	RasterPipelineStateClass *PSOPtr = nullptr;

	VkDeviceSize offsets[1] = { 0 };
	if (PSO_DSL_DS_GeomInstPairings.size() > 0)
	{
		giantVertBufferSharedMutex.lock_shared();
		vkCmdBindVertexBuffers(cmdBuffers[whichCmdBuf], PSO_DSL_DS_GeomInstPairings[0].geom->layoutCache.BindPoint, 1, &giantVertBuffer->buffer, offsets);
		vkCmdBindIndexBuffer(cmdBuffers[whichCmdBuf], giantVertBuffer->buffer, (VkDeviceSize)0, VK_INDEX_TYPE_UINT32);
		giantVertBufferSharedMutex.unlock_shared();
		RasterPipelineStateClass* lastPSOPtr = nullptr;
		DescriptorSets* lastDSPtr = nullptr;
		for (IndirectBoundaries& curIndBounds : indirectBoundaries)
		{
			RasterPipelineStateClass* psoToConsider = (mainPass ? curIndBounds.PSOMainPassPtr : curIndBounds.PSOPtr);
			if (lastPSOPtr != psoToConsider)
			{
				vkCmdBindPipeline(cmdBuffers[whichCmdBuf], VK_PIPELINE_BIND_POINT_GRAPHICS, psoToConsider->pipeline);
				lastPSOPtr = psoToConsider;
			}
			if (lastDSPtr != curIndBounds.DSPtr)
			{
				vkCmdBindDescriptorSets(cmdBuffers[whichCmdBuf], VK_PIPELINE_BIND_POINT_GRAPHICS, psoToConsider->pipelineLayout, 0, (uint32_t)curIndBounds.DSPtr->descriptorSets.size(), curIndBounds.DSPtr->descriptorSets.data(), 0, VK_NULL_HANDLE);
				lastDSPtr = curIndBounds.DSPtr;
			}
			if (doesCulling)
				Instance.fpCmdDrawIndexedIndirectCountKHR(cmdBuffers[whichCmdBuf], indirectDrawBuffer->buffer, (VkDeviceSize)curIndBounds.offset,
					indirectDrawBuffer->buffer, (VkDeviceSize)(curIndBounds.offset - IndirectDrawCounterSize()),
					curIndBounds.indDrawCount, (unsigned int)sizeof(VkDrawIndexedIndirectCommand));
			else
				vkCmdDrawIndexedIndirect(cmdBuffers[whichCmdBuf], indirectDrawBuffer->buffer, (VkDeviceSize)curIndBounds.offset,
					curIndBounds.indDrawCount, (unsigned int)sizeof(VkDrawIndexedIndirectCommand));
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

void RasterletClass::Draw(bool prepass)
{
	if (!ptrToInstance) return;

	VkResult result;

	if (ptrToFrameBuffer->mode == ON_SCREEN)
	{
		ptrToInstance->acquireImageFence.Reset();

		result = ptrToInstance->fpAcquireNextImageKHR(ptrToInstance->device, ptrToInstance->swapChain, UINT64_MAX, VK_NULL_HANDLE, ptrToInstance->acquireImageFence.fence, &ptrToFrameBuffer->currentSwapChainBuffer);
		if (result != VK_SUCCESS) FATAL_ERROR("Problem acq. next image");

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

	unsigned int cmdBufferToSubmit;
	if (ptrToFrameBuffer->mode == OFF_SCREEN)
	{
		if (!doesCulling)
			cmdBufferToSubmit = 0;
		else
			cmdBufferToSubmit = prepass ? 0 : 1;
	}
	else
	{
		if (!doesCulling)
			cmdBufferToSubmit = ptrToFrameBuffer->currentSwapChainBuffer;
		else
			cmdBufferToSubmit = (prepass ? 0 : (CommandBufferCount() - 1)/2) + ptrToFrameBuffer->currentSwapChainBuffer;
	}

	WaitOnSemaphores(waitSems, cmdBufferToSubmit);
	WaitOnOldSemaphores(waitOldSems, cmdBufferToSubmit);

	std::unordered_set<SemaphoreClass*> signalSemsThisFrame = signalSems;
	if (ptrToFrameBuffer->mode == ON_SCREEN) signalSemsThisFrame.insert(&ptrToFrameBuffer->colorAttachments[cmdBufferToSubmit]->semaphore);
	SignalSemaphores(signalSemsThisFrame, cmdBufferToSubmit);

	if (!doCPUSync) NoCPUSync(cmdBufferToSubmit);

	SubmitCommandBuffer(cmdBufferToSubmit);

	if (ptrToFrameBuffer->mode == ON_SCREEN)
	{
		VkPresentInfoKHR presentInfo = {};
		presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
		presentInfo.pNext = VK_NULL_HANDLE;
		presentInfo.swapchainCount = 1;
		presentInfo.pSwapchains = &ptrToInstance->swapChain;
		presentInfo.pImageIndices = &ptrToFrameBuffer->currentSwapChainBuffer;
		presentInfo.pWaitSemaphores = &ptrToFrameBuffer->colorAttachments[cmdBufferToSubmit]->semaphore.GetSemaphoreForPresent();
		presentInfo.waitSemaphoreCount = 1;
		{std::unique_lock<std::mutex> lk(ptrToInstance->allQueues[GRAPHICS_QUEUE].mtx);
		result = ptrToInstance->fpQueuePresentKHR(ptrToInstance->allQueues[GRAPHICS_QUEUE].queue, &presentInfo); }
		if (result != VK_SUCCESS) FATAL_ERROR("Problem presenting frame");
	}
}

HIGHOMEGA::GL::ComputeletClass::ComputeletClass()
{
	instanceRef = nullptr;
}

void HIGHOMEGA::GL::ComputeletClass::Computelet(InstanceClass & renderInst)
{
	instanceRef = &renderInst;
}

void HIGHOMEGA::GL::ComputeletClass::Start(ComputePipelineStateClass & PSO, QUEUE_TYPE inQueueType)
{
	if (!instanceRef) return;

	BeginCommandBuffer(*instanceRef, 1u, 0u, inQueueType);

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

	SubmitCommandBuffer();
}

GeometryClass::DataLayout::DataLayout()
{
}

void HIGHOMEGA::GL::GeometryClass::getMinMax(RasterVertex *verts, unsigned int vertCount, vec3& outMin, vec3& outMax)
{
	for (int i = 0; i != vertCount; i++)
	{
		vec3 edge, vnorm, vcol;
		vec2 uv;
		unpackRasterVertex(edge, vcol, uv, vnorm, verts[i]);
		if (i == 0)
		{
			outMin = outMax = edge;
		}
		else
		{
			outMin.x = min(outMin.x, edge.x);
			outMin.y = min(outMin.y, edge.y);
			outMin.z = min(outMin.z, edge.z);
			outMax.x = max(outMax.x, edge.x);
			outMax.y = max(outMax.y, edge.y);
			outMax.z = max(outMax.z, edge.z);
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
	for (std::pair<ChangeSignalClass *, unsigned int> curSub : notifySubmissions)
		curSub.first->ChangeSignal(this);
	notifySubmissions.clear();

	vertBuffer.RemovePast();
	rtGeom.RemoveRTGeom();

	downloadConstructed = false;
	uploadConstructed = false;
}

GeometryClass::GeometryClass()
{
}

void HIGHOMEGA::GL::GeometryClass::ChangeGeom(std::vector<unsigned char>& inpIndexVertexData)
{
	std::unordered_map <ChangeSignalClass *, unsigned int> curNotifySubList = notifySubmissions;

	InstanceClass *instanceRefCached = instanceRef;
	DataLayout layoutCacheCached = layoutCache;
	bool isAlphaKeyedCached = isAlphaKeyedCache;
	std::string groupIdCached = groupId;
	bool immutableCached = immutable;
	bool breakableCached = breakableCache;

	RemovePast();
	Geometry(*instanceRefCached, inpIndexVertexData, layoutCacheCached, isAlphaKeyedCached, groupIdCached, immutableCached, nullptr, nullptr, breakableCached);
	notifySubmissions = curNotifySubList;
}

void GeometryClass::Geometry(InstanceClass &inpInstance, std::vector<unsigned char>& inpIndexVertexData, const DataLayout &inpDataLayout, bool isAlphaKeyed, const std::string & inpGroupId, bool inpImmutable, std::vector <VertexAnimationInfo> *vertAnimInfo, std::string *inpArmatureId, bool breakable)
{
	layoutCache = inpDataLayout;

	instanceRef = &inpInstance;
	isAlphaKeyedCache = isAlphaKeyed;
	immutable = inpImmutable;
	groupId = inpGroupId;
	breakableCache = breakable;

	totalDataSize = (unsigned int)inpIndexVertexData.size();
	if (vertAnimInfo)
	{
		if (vertAnimInfo->size() != getVertexCount(inpIndexVertexData)) FATAL_ERROR("Mismatch between animation and vertex data size");
		vertAnimInfoBufSize = (unsigned int)vertAnimInfo->size() * sizeof(VertexAnimationInfo);
		vertAnimInfoBuf.Buffer (MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, inpInstance, (void *)vertAnimInfo->data(), vertAnimInfoBufSize);
		if (inpArmatureId) armatureId = *inpArmatureId;
	}

	MEMORY_USAGE memUsage = USAGE_SSBO | USAGE_VERT | USAGE_INDEX;
	if (RTInstance::Enabled())
		memUsage |= (USAGE_ACCEL_STRUCT | USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY);
	giantVertBufferOffsetLen = vertBuffer.VertexBufferClassWithStaging(memUsage, inpInstance, (void *)inpIndexVertexData.data(), totalDataSize);

	RasterVertex* verts; unsigned int* indices;
	getIndicesVertices(inpIndexVertexData, &verts, vertCount, &indices, triCount, vertexDataOffsetFromDataStart);
	getMinMax(verts, vertCount, geomMin, geomMax);

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

	rtGeom.SetDirty(); // Set dirty just in case RT's enabled or will be
}

void HIGHOMEGA::GL::GeometryClass::Download(std::vector<unsigned char>& outIndexVertexData)
{
	MEMORY_USAGE memUsage = USAGE_SSBO | USAGE_VERT | USAGE_INDEX;
	if (RTInstance::Enabled())
		memUsage |= (USAGE_ACCEL_STRUCT | USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY);

	if (vertexStagingBuffer.getSize() < totalDataSize) vertexStagingBuffer.Buffer(MEMORY_HOST_VISIBLE, TRANSFER_QUEUE, QUEUE_CONCURRENT, USAGE_DST | USAGE_SRC | memUsage, Instance, nullptr, totalDataSize);

	if (vertBuffer.cachedDownloadBuffer != vertexStagingBuffer.buffer || !downloadConstructed)
	{
		try { vertBuffer.BeginCommandBuffer(Instance, 4, 1); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for buffer creation"); }

		giantVertBufferSharedMutex.lock_shared();
		VkBufferCopy copyRegion = {};
		copyRegion.srcOffset = getDataOffsetInGiantVertexBuffer() * sizeof(unsigned int);
		copyRegion.size = totalDataSize;
		vkCmdCopyBuffer(vertBuffer.cmdBuffers[1], giantVertBuffer->buffer, vertexStagingBuffer.buffer, 1, &copyRegion);
		giantVertBufferSharedMutex.unlock_shared();

		try { vertBuffer.EndCommandBuffer(1); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for buffer creation"); }

		downloadConstructed = true;
		vertBuffer.cachedDownloadBuffer = vertexStagingBuffer.buffer;
	}

	try { vertBuffer.SubmitCommandBuffer(1); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for buffer creation"); }

	outIndexVertexData.resize(totalDataSize);
	vertexStagingBuffer.DownloadSubData(0, outIndexVertexData.data(), totalDataSize);
}

void HIGHOMEGA::GL::GeometryClass::Update(std::vector<unsigned char>& inpIndexVertexData)
{
	if ((unsigned int)inpIndexVertexData.size() != totalDataSize || immutable) return;

	MEMORY_USAGE memUsage = USAGE_SSBO | USAGE_VERT | USAGE_INDEX;
	if (RTInstance::Enabled())
		memUsage |= (USAGE_ACCEL_STRUCT | USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY);

	if (vertexStagingBuffer.getSize() < totalDataSize) vertexStagingBuffer.Buffer(MEMORY_HOST_VISIBLE, TRANSFER_QUEUE, QUEUE_CONCURRENT, USAGE_DST | USAGE_SRC | memUsage, Instance, nullptr, totalDataSize);

	vertexStagingBuffer.UploadSubData(0, inpIndexVertexData.data(), totalDataSize);

	VkBufferCopy copyRegion = {};

	if (vertBuffer.cachedUploadBuffer != vertexStagingBuffer.buffer || !uploadConstructed)
	{
		try { vertBuffer.BeginCommandBuffer(Instance, 4, 0); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for buffer creation"); }

		giantVertBufferSharedMutex.lock_shared();
		copyRegion.dstOffset = getDataOffsetInGiantVertexBuffer() * sizeof(unsigned int);
		copyRegion.size = totalDataSize;
		vkCmdCopyBuffer(vertBuffer.cmdBuffers[0], vertexStagingBuffer.buffer, giantVertBuffer->buffer, 1, &copyRegion);
		giantVertBufferSharedMutex.unlock_shared();

		try { vertBuffer.EndCommandBuffer(0); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for buffer creation"); }

		uploadConstructed = true;
		vertBuffer.cachedUploadBuffer = vertexStagingBuffer.buffer;
	}

	try { vertBuffer.SubmitCommandBuffer(0); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for buffer creation"); }

	RasterVertex* verts; unsigned int* indices;
	getIndicesVertices(inpIndexVertexData, &verts, vertCount, &indices, triCount, vertexDataOffsetFromDataStart);
	getMinMax(verts, vertCount, geomMin, geomMax);

	rtGeom.SetDirty(); // Set dirty just in case RT's enabled or will be
}

KHR_RT::RTGeometry & HIGHOMEGA::GL::GeometryClass::getRTGeom()
{
	return rtGeom;
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
	return geomMin;
}

vec3 & HIGHOMEGA::GL::GeometryClass::getGeomMax()
{
	return geomMax;
}

unsigned int HIGHOMEGA::GL::GeometryClass::getVertexOffsetInGiantVertexBuffer()
{
	return ((unsigned int)giantVertBufferOffsetLen.offset + vertexDataOffsetFromDataStart) / (unsigned int)sizeof(RasterVertex);
}

unsigned int HIGHOMEGA::GL::GeometryClass::getIndexOffsetInGiantVertexBuffer()
{
	return ((unsigned int)giantVertBufferOffsetLen.offset + 2 * sizeof(unsigned int)) / (unsigned int)sizeof(unsigned int);
}

unsigned int HIGHOMEGA::GL::GeometryClass::getDataOffsetInGiantVertexBuffer()
{
	return ((unsigned int)giantVertBufferOffsetLen.offset) / (unsigned int)sizeof(unsigned int);
}

unsigned int HIGHOMEGA::GL::GeometryClass::getTriCount()
{
	return triCount;
}

unsigned int HIGHOMEGA::GL::GeometryClass::getVertCount()
{
	return vertCount;
}

unsigned int HIGHOMEGA::GL::GeometryClass::getTotalDataSize()
{
	return totalDataSize;
}

bool HIGHOMEGA::GL::GeometryClass::getBreakable()
{
	return breakableCache;
}

void BufferClass::RemovePast()
{
	if (haveBuffer) instanceRef->QueueFlush(queueType);
	semaphore.RemovePast();
	if (haveSubAlloc) FreeMem (subAllocs, instanceRef->device);
	if (haveBuffer) vkDestroyBuffer(instanceRef->device, buffer, nullptr);
	cachedUploadBuffer = cachedDownloadBuffer = VK_NULL_HANDLE;
	DestroyCommandBuffer();

	if (usingGiantVertBuffer)
	{
		giantVertBufferSharedMutex.lock();
		if (giantVertBuffer)
		{
			giantVertBufferClaims--;
			if (giantVertBufferClaims == 0u)
			{
				delete giantVertBuffer;
				giantVertBuffer = nullptr;
			}
			giantVertBufferSharedMutex.unlock();

			giantVertBufferDirectoryMutex.lock();
			giantVertBufferSubAllocs.erase(std::find_if(giantVertBufferSubAllocs.begin(), giantVertBufferSubAllocs.end(), [&](const HIGHOMEGA::GL::VertBufferOffsetLen& arg) -> bool {
				return arg.id == insertedEntry.id;
			}));
			insertedEntry = {};
			giantVertBufferDirectoryMutex.unlock();
		}
		else
			giantVertBufferSharedMutex.unlock();
		usingGiantVertBuffer = false;
	}

	haveSubAlloc = false;
	haveBuffer = false;
	clearCmdBufConstructed = false;
	copyCmdBufferConstructed = false;
	totalDataSize = 0;
}

BufferClass::BufferClass()
{
	insertedEntry = {};
	haveSubAlloc = false;
	haveBuffer = false;
	clearCmdBufConstructed = false;
	copyCmdBufferConstructed = false;
	usingGiantVertBuffer = false;
	totalDataSize = 0;
	cachedUploadBuffer = cachedDownloadBuffer = VK_NULL_HANDLE;
}

void HIGHOMEGA::GL::BufferClass::Buffer(MEMORY_OPTIONS inpMemOpts, QUEUE_TYPE inQueueType, RESOURCE_SHARING_MODE inpSharingMode, MEMORY_USAGE inpUsage, InstanceClass & inpInstance, void * inpData, unsigned int inpDataSize)
{
	if (totalDataSize > 0) RemovePast();

	instanceRef = &inpInstance;
	semaphore.Semaphore(instanceRef);
	VkMemoryRequirements memReqs;

	usage = inpUsage;
	memOpts = inpMemOpts;
	queueType = inQueueType;
	sharingMode = inpSharingMode;
	totalDataSize = inpDataSize;

	VkBufferCreateInfo bufferInfo = {};
	allocInfo = {};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.pNext = VK_NULL_HANDLE;
	allocInfo.allocationSize = 0;
	allocInfo.memoryTypeIndex = 0;

	bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	bufferInfo.size = totalDataSize;
	bufferInfo.usage = inpUsage;
	bufferInfo.sharingMode = (VkSharingMode)sharingMode;
	std::vector<uint32_t> allQueues;
	if (sharingMode == QUEUE_EXCLUSIVE)
	{
		bufferInfo.pQueueFamilyIndices = &instanceRef->allQueues[queueType].nodeIndex;
		bufferInfo.queueFamilyIndexCount = 1;
	}
	else
	{
		allQueues.push_back(instanceRef->allQueues[TRANSFER_QUEUE].nodeIndex);
		allQueues.push_back(instanceRef->allQueues[COMPUTE_QUEUE].nodeIndex);
		allQueues.push_back(instanceRef->allQueues[GRAPHICS_QUEUE].nodeIndex);
		bufferInfo.pQueueFamilyIndices = allQueues.data();
		bufferInfo.queueFamilyIndexCount = (uint32_t)allQueues.size();
	}
	if (inpInstance.SupportsSparseResources() && (memOpts & MEMORY_HOST_VISIBLE) == 0) bufferInfo.flags |= VK_BUFFER_CREATE_SPARSE_BINDING_BIT;

	VkResult result = vkCreateBuffer(inpInstance.device, &bufferInfo, nullptr, &buffer);
	if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not create uniform buffer"); }
	haveBuffer = true;

	vkGetBufferMemoryRequirements(inpInstance.device, buffer, &memReqs);
	if (inpUsage & USAGE_SBT) // I swear vkGetBufferMemoryRequirements() should've handled this
		memReqs.alignment = (memReqs.alignment * RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment) / std::gcd(memReqs.alignment, RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment);
	getMemoryType(&inpInstance, memReqs.memoryTypeBits, memOpts, &allocInfo.memoryTypeIndex);

	try
	{
		subAllocs = AllocMem(inpInstance.device, allocInfo, memReqs, ((usage & USAGE_DEVICE_ADDRESS) != 0) ? DEV_ADDRESS : BUFFER, inpInstance.SupportsSparseResources() && ((memOpts & MEMORY_HOST_VISIBLE) == 0), inpDataSize == HIGHOMEGA_ONE_GIANT_VERTBUFFER_SIZE);
		haveSubAlloc = true;
	}
	catch (...)
	{
		RemovePast();
		FATAL_ERROR("Could not allocate buffer memory");
	}

	if (inpInstance.SupportsSparseResources() && (memOpts & MEMORY_HOST_VISIBLE) == 0)
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		std::unique_lock <std::mutex> lk2(inpInstance.allQueues[inQueueType].mtx);
		FenceClass sparseFence;
		sparseFence.Fence(&inpInstance);
		sparseFence.Reset();
		VkSparseBufferMemoryBindInfo bufferMemoryBinds;
		bufferMemoryBinds.buffer = buffer;
		std::vector<VkSparseMemoryBind> memoryBinds;
		unsigned long long resourceOffset = 0ull;
		for (std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
		{
			VkSparseMemoryBind curBufBind = {};
			curBufBind.memory = std::get<2>(curSubAlloc).mem;
			curBufBind.memoryOffset = (VkDeviceSize)std::get<2>(curSubAlloc).offset;
			curBufBind.size = (VkDeviceSize)std::get<2>(curSubAlloc).len;
			curBufBind.resourceOffset = (VkDeviceSize)resourceOffset;
			curBufBind.flags = VK_SPARSE_MEMORY_BIND_METADATA_BIT;
			resourceOffset += std::get<2>(curSubAlloc).len;
			memoryBinds.push_back(curBufBind);
		}
		bufferMemoryBinds.bindCount = (uint32_t)memoryBinds.size();
		bufferMemoryBinds.pBinds = memoryBinds.data();
		VkBindSparseInfo vkBindSparseInfo = {};
		vkBindSparseInfo.sType = VK_STRUCTURE_TYPE_BIND_SPARSE_INFO;
		vkBindSparseInfo.bufferBindCount = 1;
		vkBindSparseInfo.pBufferBinds = &bufferMemoryBinds;
		result = vkQueueBindSparse(inpInstance.allQueues[inQueueType].queue, 1, &vkBindSparseInfo, sparseFence.fence);
		sparseFence.Wait();
	}
	else
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		result = vkBindBufferMemory(inpInstance.device, buffer, std::get<2>(*subAllocs.begin()).mem, std::get<2>(*subAllocs.begin()).offset);
	}
	if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not bind buffer to memory"); }

	descriptor.buffer = buffer;
	descriptor.offset = 0;
	descriptor.range = totalDataSize;

	if (inpData != nullptr)
	{
		try
		{
			UploadSubData(0, inpData, totalDataSize);
		}
		catch (...) { RemovePast(); FATAL_ERROR("Could not upload uniform buffer data"); }
	}
}

HIGHOMEGA::GL::VertBufferOffsetLen HIGHOMEGA::GL::BufferClass::VertexBufferClassWithStaging(MEMORY_USAGE inpUsage, InstanceClass & inpInstance, void * inpData, unsigned int inpDataSize)
{
	totalDataSize = inpDataSize;

	if (vertexStagingBuffer.getSize() < totalDataSize) vertexStagingBuffer.Buffer(MEMORY_HOST_VISIBLE, TRANSFER_QUEUE, QUEUE_CONCURRENT, USAGE_DST | USAGE_SRC | inpUsage, Instance, nullptr, totalDataSize);

	vertexStagingBuffer.UploadSubData(0, inpData, totalDataSize);

	giantVertBufferSharedMutex.lock();
	if (!giantVertBuffer)
	{
		giantVertBuffer = new BufferClass(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SRC | USAGE_DST | inpUsage, inpInstance, nullptr, HIGHOMEGA_ONE_GIANT_VERTBUFFER_SIZE);
		giantVertBufferClaims = 1;
	}
	else
		giantVertBufferClaims++;
	giantVertBufferSharedMutex.unlock();

	giantVertBufferDirectoryMutex.lock();
	std::vector<VertBufferOffsetLen>::iterator it = giantVertBufferSubAllocs.begin();
	bool foundAHole = false;
	insertedEntry.len = totalDataSize;
	insertedEntry.id = threadSafeMersenneTwister64Bit();
	if (giantVertBufferSubAllocs.size() > 0)
	{
		bool firstSearch = true;
		VertBufferOffsetLen firstHole;
		firstHole.offset = 0u;
		firstHole.len = 0u;
		while (true)
		{
			VertBufferOffsetLen& curSubAlloc = firstSearch ? firstHole : *it;
			if (!firstSearch) it++;
			VertBufferOffsetLen& nextSubAlloc = firstSearch ? *giantVertBufferSubAllocs.begin() : *it;
			firstSearch = false;
			unsigned long long offsetAligned = curSubAlloc.offset + curSubAlloc.len;
			if (offsetAligned + totalDataSize <= nextSubAlloc.offset)
			{
				insertedEntry.offset = offsetAligned;
				giantVertBufferSubAllocs.insert(it, insertedEntry);
				foundAHole = true;
				break;
			}
			if (nextSubAlloc == giantVertBufferSubAllocs.back()) break;
		}
	}
	if (!foundAHole)
	{
		if (giantVertBufferSubAllocs.size() == 0)
			insertedEntry.offset = 0u;
		else
			insertedEntry.offset = giantVertBufferSubAllocs.back().offset + giantVertBufferSubAllocs.back().len;
		if ((unsigned int)(insertedEntry.offset + insertedEntry.len) > HIGHOMEGA_ONE_GIANT_VERTBUFFER_SIZE) { RemovePast(); FATAL_ERROR("Main engine vertex buffer not large enough."); }
		giantVertBufferSubAllocs.push_back(insertedEntry);
	}
	giantVertBufferDirectoryMutex.unlock();

	usingGiantVertBuffer = true;

	VkBufferCopy copyRegion = {};

	try { BeginCommandBuffer(inpInstance, 4, 0); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for buffer creation"); }

	giantVertBufferSharedMutex.lock_shared();
	copyRegion.size = totalDataSize;
	copyRegion.dstOffset = (VkDeviceSize)insertedEntry.offset;
	vkCmdCopyBuffer(cmdBuffers[0], vertexStagingBuffer.buffer, giantVertBuffer->buffer, 1, &copyRegion);
	giantVertBufferSharedMutex.unlock_shared();

	try { EndCommandBuffer(0); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for buffer creation"); }

	try { SubmitCommandBuffer(0); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for buffer creation"); }

	return insertedEntry;
}

void HIGHOMEGA::GL::BufferClass::CopyToBuffer(InstanceClass& inpInstance, BufferClass& dstBuf, unsigned int inpDataSize, unsigned int srcOffset, unsigned int dstOffset)
{
	if (getSize() > dstBuf.getSize()) FATAL_ERROR("Cannot copy to smaller buffer");
	if (inpDataSize > getSize()) FATAL_ERROR("Copying more data than we have during a buffer copy");

	if (!copyCmdBufferConstructed || cachedCopyBuffer != dstBuf.buffer || lastCopySrcOffset != srcOffset || lastCopyDstOffset != dstOffset || lastCopyAmount != inpDataSize)
	{
		VkBufferCopy copyRegion = {};

		try { BeginCommandBuffer(inpInstance, 4, 2); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for buffer copy"); }

		copyRegion.size = inpDataSize;
		copyRegion.srcOffset = srcOffset;
		copyRegion.dstOffset = dstOffset;
		vkCmdCopyBuffer(cmdBuffers[2], buffer, dstBuf.buffer, 1, &copyRegion);

		try { EndCommandBuffer(2); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for buffer copy"); }

		cachedCopyBuffer = dstBuf.buffer;
		lastCopySrcOffset = srcOffset;
		lastCopyDstOffset = dstOffset;
		lastCopyAmount = inpDataSize;
		copyCmdBufferConstructed = true;
	}

	WaitOnSemaphores(std::unordered_set<SemaphoreClass*>{ &semaphore, &dstBuf.semaphore }, 2);
	SignalSemaphores(std::unordered_set<SemaphoreClass*>{ &dstBuf.semaphore }, 2);
	NoCPUSync(2);
	try { SubmitCommandBuffer(2); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for buffer copy"); }
}

unsigned int HIGHOMEGA::GL::BufferClass::getSize()
{
	return totalDataSize;
}

void HIGHOMEGA::GL::BufferClass::UploadSubData(unsigned int inpOffset, void *inpData, unsigned int inpDataSize)
{
	std::unique_lock<std::mutex> lk(mem_manager_mutex);
	if (!haveBuffer || !haveSubAlloc) FATAL_ERROR("Buffer not initialized properly for upload");

	unsigned char *dataPtr;
	VkResult result;
	unsigned int copiedSoFar = 0u;
	for (std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
	{
		if (inpOffset >= (unsigned int)std::get<2>(curSubAlloc).len)
		{
			inpOffset -= (unsigned int)std::get<2>(curSubAlloc).len;
			continue;
		}
		unsigned long long copyAmount = min(std::get<2>(curSubAlloc).len - inpOffset, inpDataSize);
		VkDeviceMemory devMem = std::get<2>(curSubAlloc).mem;
		unsigned long long mapOffset = std::get<2>(curSubAlloc).offset + inpOffset;
		mem_manager_mutex.unlock();
		result = instanceRef->ThreadSafeMapMemory(devMem, mapOffset, (VkDeviceSize)copyAmount, 0, (void**)&dataPtr);
		mem_manager_mutex.lock();
		if (result != VK_SUCCESS) FATAL_ERROR("Memory map failed for upload");
		memcpy(dataPtr, (void *)&(((unsigned char *)inpData)[copiedSoFar]), copyAmount);
		inpDataSize -= (unsigned int)copyAmount;
		copiedSoFar += (unsigned int)copyAmount;
		inpOffset = 0u;
		mem_manager_mutex.unlock();
		instanceRef->ThreadSafeUnmapMemory(devMem);
		mem_manager_mutex.lock();
		if (inpDataSize == 0u) break;
	}
}

void HIGHOMEGA::GL::BufferClass::DownloadSubData(unsigned int inpOffset, void *inpData, unsigned int inpDataSize)
{
	std::lock_guard<std::mutex> lk(mem_manager_mutex);
	if (!haveBuffer || !haveSubAlloc) FATAL_ERROR("Buffer not initialized properly for download");

	unsigned char *dataPtr;
	VkResult result;
	unsigned int copiedSoFar = 0u;
	for (std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
	{
		if (inpOffset >= (unsigned int)std::get<2>(curSubAlloc).len)
		{
			inpOffset -= (unsigned int)std::get<2>(curSubAlloc).len;
			continue;
		}
		unsigned long long copyAmount = min(std::get<2>(curSubAlloc).len - inpOffset, inpDataSize);
		VkDeviceMemory devMem = std::get<2>(curSubAlloc).mem;
		unsigned long long mapOffset = std::get<2>(curSubAlloc).offset + inpOffset;
		mem_manager_mutex.unlock();
		result = instanceRef->ThreadSafeMapMemory(devMem, mapOffset, (VkDeviceSize)copyAmount, 0, (void**)&dataPtr);
		mem_manager_mutex.lock();
		if (result != VK_SUCCESS) FATAL_ERROR("Memory map failed for download");
		memcpy((void*)&(((unsigned char*)inpData)[copiedSoFar]), dataPtr, copyAmount);
		inpDataSize -= (unsigned int)copyAmount;
		copiedSoFar += (unsigned int)copyAmount;
		inpOffset = 0u;
		mem_manager_mutex.unlock();
		instanceRef->ThreadSafeUnmapMemory(devMem);
		mem_manager_mutex.lock();
		if (inpDataSize == 0u) break;
	}
}

void HIGHOMEGA::GL::BufferClass::Clear(unsigned int clearValue)
{
	if (!clearCmdBufConstructed || lastClearValue != clearValue)
	{
		try { BeginCommandBuffer(Instance, 4, 3); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for buffer value set"); }

		vkCmdFillBuffer(cmdBuffers[3], buffer, (VkDeviceSize)0ull, VK_WHOLE_SIZE, clearValue);

		try { EndCommandBuffer(3); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for buffer value set"); }

		lastClearValue = clearValue;
		clearCmdBufConstructed = true;
	}

	try { SubmitCommandBuffer(3); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for buffer value set"); }
}

BufferClass::~BufferClass()
{
	RemovePast();
}

HIGHOMEGA::GL::ShaderStage::ShaderStage()
{
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::ShaderStage::Shader(InstanceClass &inpPtrToInstance, std::string shaderFile, const char * entryName, VkShaderStageFlagBits stage_bit, ShaderSpecilization* shaderSpec)
{
	VkShaderModule shaderModule;
	VkShaderModuleCreateInfo moduleCreateInfo;

	moduleCreateInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
	moduleCreateInfo.pNext = VK_NULL_HANDLE;

	std::string shaderFileLowercase = toLowerCase (shaderFile);

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
		FATAL_ERROR("Could not load shader file");
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

	if (result != VK_SUCCESS || !shaderModule ) FATAL_ERROR("Failed to create shader module");

	stage = {};
	stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
	stage.stage = stage_bit;
	char* entryNameCopy = new char[strlen(entryName) + 1];
	memcpy(entryNameCopy, entryName, strlen(entryName));
	entryNameCopy[strlen(entryName)] = '\0';
	stage.pName = entryNameCopy;
	stage.module = shaderModule;

	if (shaderSpec)
	{
		if (!SpecializationData.dataBuf)
		{
			SpecializationData.dataBuf = new std::vector<unsigned int>();
			SpecializationData.entries = new std::vector<VkSpecializationMapEntry>();
			SpecializationData.info = new VkSpecializationInfo;
		}
		if (SpecializationData.dataBuf->size() == 0) SpecializationData.dataBuf->resize(shaderSpec->inputs.size());
		else FATAL_ERROR("Specialization data buffer should be initialized only once");
		for (unsigned int i = 0; i != shaderSpec->inputs.size(); i++)
		{
			(*SpecializationData.dataBuf)[i] = shaderSpec->inputs[i].value;
			(*SpecializationData.entries).push_back({shaderSpec->inputs[i].bindPoint, (unsigned int)(i * sizeof(unsigned int)), sizeof(unsigned int)});
		}
		*(SpecializationData.info) = { (unsigned int)SpecializationData.entries->size(), SpecializationData.entries->data(), SpecializationData.dataBuf->size() * sizeof(unsigned int), SpecializationData.dataBuf->data() };
		stage.pSpecializationInfo = SpecializationData.info;
	}

	ptrToInstance = &inpPtrToInstance;
}

HIGHOMEGA::GL::ShaderStage::~ShaderStage()
{
	if (stage.pName) delete stage.pName;
	if (ptrToInstance != nullptr) vkDestroyShaderModule(ptrToInstance->device, stage.module, nullptr);
	if (SpecializationData.dataBuf) delete SpecializationData.dataBuf;
	if (SpecializationData.entries) delete SpecializationData.entries;
	if (SpecializationData.info) delete SpecializationData.info;
	SpecializationData.dataBuf = nullptr;
	SpecializationData.entries = nullptr;
	SpecializationData.info = nullptr;
	stage.pName = nullptr;
	ptrToInstance = nullptr;
}

CacheItem<ShaderStage>* HIGHOMEGA::GL::AddOrFindCachedShaderStage(InstanceClass & ptrToInstance, std::string shaderFile, const char * entryName, VkShaderStageFlagBits stage_bit, ShaderSpecilization* shaderSpec, std::string& shaderStageKey)
{
	CacheItem<ShaderStage>* retVal = nullptr;

	shaderStageKey = shaderFile + ":" + entryName;
	if (shaderSpec) shaderStageKey += ("[" + shaderSpec->name + "]");

	std::unique_lock<std::mutex> lk(shader_stage_mutex, std::defer_lock);
	lk.lock();

	if (ShaderStageCache.find(shaderStageKey) == ShaderStageCache.end())
	{
		ShaderStageCache[shaderStageKey].preparingOnAnotherThread = true;
		lk.unlock();

		ShaderStage* shaderStage = new ShaderStage;

		try
		{
			shaderStage->Shader(ptrToInstance, shaderFile, entryName, stage_bit, shaderSpec);
		}
		catch (...)
		{
			delete shaderStage;
			lk.lock();
			ShaderStageCache.erase(shaderStageKey);
			lk.unlock();
			FATAL_ERROR("Could not create shader stage for cache");
		}

		lk.lock();
		ShaderStageCache[shaderStageKey].elem = std::move(*shaderStage);
		operator delete(shaderStage);
		ShaderStageCache[shaderStageKey].elemCount = 1;
		ShaderStageCache[shaderStageKey].preparingOnAnotherThread = false;
	}
	else
	{
		if (ShaderStageCache[shaderStageKey].preparingOnAnotherThread)
		{
			lk.unlock();
			while (true)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				lk.lock();
				if (ShaderStageCache.find(shaderStageKey) == ShaderStageCache.end())
				{
					lk.unlock();
					FATAL_ERROR("Could not create shader stage for cache from another thread");
				}
				else
				{
					if (!ShaderStageCache[shaderStageKey].preparingOnAnotherThread)
					{
						retVal = &ShaderStageCache[shaderStageKey];
						ShaderStageCache[shaderStageKey].elemCount++;
						lk.unlock();
						return retVal;
					}
				}
				lk.unlock();
			}
		}
		ShaderStageCache[shaderStageKey].elemCount++;
	}
	retVal = &ShaderStageCache[shaderStageKey];
	lk.unlock();
	return retVal;
}

unsigned int HIGHOMEGA::GL::getTriCount(std::vector<unsigned char>& idxVertData)
{
	return *((unsigned int *)idxVertData.data());
}

unsigned int HIGHOMEGA::GL::getVertexCount(std::vector<unsigned char>& idxVertData)
{
	return *((unsigned int*)(idxVertData.data() + sizeof(unsigned int)));
}

void HIGHOMEGA::GL::getIndicesVertices(std::vector<unsigned char>& idxVertData, RasterVertex** vertices, unsigned int& vertexCount, unsigned int** indices, unsigned int& triCount, unsigned int& vertexDataOffset)
{
	triCount = getTriCount(idxVertData);
	vertexCount = getVertexCount(idxVertData);
	*indices = (unsigned int*)(idxVertData.data() + (sizeof(unsigned int) * 2));
	unsigned int preVertexDataSize = (unsigned int)(sizeof(unsigned int) * 2 + triCount * sizeof(unsigned int) * 3);
	unsigned int paddingAmount = (unsigned int)(sizeof(RasterVertex) - (preVertexDataSize % sizeof(RasterVertex)));
	vertexDataOffset = preVertexDataSize + paddingAmount;
	*vertices = (RasterVertex *)(idxVertData.data() + vertexDataOffset);
}

void HIGHOMEGA::GL::createIndicesVertices(std::vector<unsigned int>& indices, std::vector<RasterVertex>& vertices, std::vector<unsigned char>& idxVertData)
{
	unsigned int finalSize = sizeof(unsigned int) * 2;
	unsigned int indexDataSize = sizeof(unsigned int) * (unsigned int)indices.size();
	unsigned int vertexDataSize = sizeof(RasterVertex) * (unsigned int)vertices.size();
	finalSize += indexDataSize;
	unsigned int padFinalSize = 24 - (finalSize % 24); /* sizeof(RasterVertex) */
	finalSize += padFinalSize + vertexDataSize;
	idxVertData.resize(finalSize);
	unsigned int triCount = (unsigned int)(indices.size() / 3);
	unsigned int vertCount = (unsigned int)vertices.size();
	memcpy(idxVertData.data(), (void*)&triCount, sizeof(unsigned int));
	memcpy(idxVertData.data() + sizeof(unsigned int), (void*)&vertCount, sizeof(unsigned int));
	memcpy(idxVertData.data() + sizeof(unsigned int) * 2, indices.data(), indexDataSize);
	memcpy(idxVertData.data() + sizeof(unsigned int) * 2 + indexDataSize + padFinalSize, vertices.data(), vertexDataSize);
}

void HIGHOMEGA::GL::createEmptyIndicesVertices(unsigned int nIndices, unsigned int nVertices, std::vector<unsigned char>& idxVertData)
{
	unsigned int finalSize = sizeof(unsigned int) * 2;
	unsigned int indexDataSize = sizeof(unsigned int) * nIndices;
	unsigned int vertexDataSize = sizeof(RasterVertex) * nVertices;
	finalSize += indexDataSize;
	unsigned int padFinalSize = 24 - (finalSize % 24); /* sizeof(RasterVertex) */
	finalSize += padFinalSize + vertexDataSize;
	idxVertData.resize(finalSize);
	unsigned int triCount = (unsigned int)(nIndices / 3);
	unsigned int vertCount = (unsigned int)nVertices;
	memcpy(idxVertData.data(), (void*)&triCount, sizeof(unsigned int));
	memcpy(idxVertData.data() + sizeof(unsigned int), (void*)&vertCount, sizeof(unsigned int));
}

void HIGHOMEGA::GL::multiplyGeometry(std::vector<unsigned char>& outIndexVertexData, unsigned int mulFactor)
{
	RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
	getIndicesVertices(outIndexVertexData, &verts, vertCount, &indices, triCount, vertexDataOffset);
	std::vector<unsigned int> newIndices;
	std::vector<RasterVertex> newVertices;
	unsigned int indexCount = triCount * 3;
	newIndices.reserve(indexCount * mulFactor);
	newVertices.reserve(vertCount * mulFactor);
	for (unsigned int i = 0; i != mulFactor; i++)
	{
		for (unsigned int j = 0; j != indexCount; j++)
			newIndices.push_back(indices[j] + i * vertCount);
		for (unsigned int j = 0; j != vertCount; j++)
			newVertices.push_back(verts[j]);
	}
	createIndicesVertices(newIndices, newVertices, outIndexVertexData);
}

void HIGHOMEGA::GL::collateGeometry(std::vector<unsigned char>& inOutIndexVertexData, std::vector<unsigned char>& inIndexVertexDataToAdd)
{
	RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
	getIndicesVertices(inOutIndexVertexData, &verts, vertCount, &indices, triCount, vertexDataOffset);
	RasterVertex* verts2; unsigned int* indices2, triCount2, vertCount2, vertexDataOffset2;
	getIndicesVertices(inIndexVertexDataToAdd, &verts2, vertCount2, &indices2, triCount2, vertexDataOffset2);

	std::vector<unsigned int> newIndices;
	std::vector<RasterVertex> newVertices;
	newIndices.reserve((triCount + triCount2) * 3);
	newVertices.reserve(vertCount + vertCount2);

	for (unsigned int i = 0; i != triCount * 3; i++)
		newIndices.push_back(indices[i]);
	for (unsigned int i = 0; i != triCount2 * 3; i++)
		newIndices.push_back(indices2[i] + vertCount);

	for (unsigned int i = 0; i != vertCount; i++)
		newVertices.push_back(verts[i]);
	for (unsigned int i = 0; i != vertCount2; i++)
		newVertices.push_back(verts2[i]);

	createIndicesVertices(newIndices, newVertices, inOutIndexVertexData);
}

bool HIGHOMEGA::GL::isIndexVertexDataTooSmall(std::vector<unsigned char>& idxVertData)
{
	return idxVertData.size() <= (2 * sizeof(unsigned int));
}

float HIGHOMEGA::GL::packVertexVelocity(const vec3& inpVel, float inpPackedColor)
{
	float inpVelLen = inpVel.length();
	vec3 inpVelNorm = inpVel;
	if (inpVelLen != 0.0f) inpVelNorm /= inpVelLen;
	unsigned int zSignVelX = (unsigned int)(Clamp(inpVelNorm.x + 1.0f, 0.0f, 2.0f) * 63.5f);
	if (inpVelNorm.z < 0.0f) zSignVelX |= 0x80u;
	unsigned int velY = (unsigned int)(Clamp(inpVelNorm.y + 1.0f, 0.0f, 2.0f) * 127.5f);
	unsigned int velLen = Clamp((unsigned int)(inpVelLen * 10.0f), 0u, 255u);

	unsigned int inpPackedColorUint = *((unsigned int *)(&inpPackedColor));
	inpPackedColorUint &= 0x000000FFu;
	inpPackedColorUint |= (zSignVelX << 8u);
	inpPackedColorUint |= (velY << 16u);
	inpPackedColorUint |= (velLen << 24u);
	return *((float *)(&inpPackedColorUint));
}

void HIGHOMEGA::GL::packRasterVertex(const vec3& pos, const vec3& col, const vec2& uv, const vec3& vNorm, RasterVertex& rv)
{
	rv.posCol[0] = pos.x;
	rv.posCol[1] = pos.y;
	rv.posCol[2] = pos.z;
	rv.posCol[3] = packColor(col);

	rv.uv = toFP16(uv);
	rv.Norm = toZSignXY(vNorm.normalized());
}

void HIGHOMEGA::GL::packRasterVertex(const vec3& pos, float packedColor, const vec2& uv, const vec3& vNorm, RasterVertex& rv)
{
	rv.posCol[0] = pos.x;
	rv.posCol[1] = pos.y;
	rv.posCol[2] = pos.z;
	rv.posCol[3] = packedColor;

	rv.uv = toFP16(uv);
	rv.Norm = toZSignXY(vNorm.normalized());
}

void HIGHOMEGA::GL::unpackRasterVertex(vec3& pos, vec3& col, vec2& uv, vec3& vNorm, const RasterVertex& rv)
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
	case R8UI:
		return 1;
	case R16F:
	case R8G8SRGB:
	case R8G8UN:
	case R8G8SN:
	case R8G8UI:
		return 2;
	case R8G8B8SRGB:
	case R8G8B8UN:
	case R8G8B8SN:
	case R8G8B8UI:
		return 3;
	case R16G16F:
	case R8G8B8A8SRGB:
	case R8G8B8A8UN:
	case R8G8B8A8SN:
	case R8G8B8A8UI:
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
	if (res != VK_SUCCESS) FATAL_ERROR("Timestamp: could not create query pool");
}

void HIGHOMEGA::GL::Timestamp::Start(VkCommandBuffer& cmdBuf)
{
	if (!ptrToInstance) FATAL_ERROR("Timestamp: starting without an instance");
	vkCmdWriteTimestamp(cmdBuf, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, timingPool, 0);
}

void HIGHOMEGA::GL::Timestamp::End(VkCommandBuffer& cmdBuf)
{
	if (!ptrToInstance) FATAL_ERROR("Timestamp: ending without an instance");
	vkCmdWriteTimestamp(cmdBuf, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, timingPool, 1);
}

unsigned long long HIGHOMEGA::GL::Timestamp::Report()
{
	if (!ptrToInstance) FATAL_ERROR("Timestamp: reporting without an instance");
	unsigned long long queryResults[2];
	VkResult res = vkGetQueryPoolResults(ptrToInstance->device, timingPool, 0, 2, sizeof(queryResults), queryResults, sizeof(unsigned long long), VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT);
	if (res != VK_SUCCESS) FATAL_ERROR("Timestamp: could not fetch timestamp results");
	return queryResults[1] - queryResults[0];
}

void HIGHOMEGA::GL::CommandBuffer::DestroyCommandBuffer()
{
	if (!ptrToInstance || !cmdPoolRef) return;

	if (haveSetupCmdBuffer)
	{
		ownSemaphores.clear();
		vkFreeCommandBuffers(ptrToInstance->device, cmdPoolRef->elem, (uint32_t)cmdBuffers.size(), cmdBuffers.data());
		{std::lock_guard <std::mutex> lk(ptrToInstance->allQueues[queueType].mtx);
		cmdPoolRef->elemCount--;
		if (cmdPoolRef->elemCount == 0)
		{
			vkDestroyCommandPool(ptrToInstance->device, cmdPoolRef->elem, nullptr);
			cmdPools[queueType].dir.erase(cmdPoolRef->keyRef);
		}}
		cmdBuffers.clear();
		cmdPoolRef = nullptr;
		haveSetupCmdBuffer = false;
	}
	waitOn.clear();
	waitOnOld.clear();
	signal.clear();
	cpuSync.clear();
}

HIGHOMEGA::GL::CommandBuffer::~CommandBuffer()
{
	DestroyCommandBuffer();
}

ThreadLocalCache <VkCommandPool>::value *HIGHOMEGA::GL::CommandBuffer::CreateCommandPool(InstanceClass &providedInstance, QUEUE_TYPE inQueueType)
{
	queueType = inQueueType;

	std::lock_guard <std::mutex> lk(cmdPools[queueType].mtx);
	if (!ptrToInstance) ptrToInstance = &providedInstance;

	if (cmdPools[queueType].dir.find(ThreadID) == cmdPools[queueType].dir.end())
	{
		VkCommandPoolCreateInfo cmdPoolInfo = {};
		cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
		cmdPoolInfo.queueFamilyIndex = ptrToInstance->allQueues[queueType].nodeIndex;
		cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
		VkResult result = vkCreateCommandPool(ptrToInstance->device, &cmdPoolInfo, nullptr, &cmdPools[queueType].dir[ThreadID].elem);
		if (result != VK_SUCCESS) { FATAL_ERROR("CommandBuffer: Could not create command pool"); }
		cmdPools[queueType].dir[ThreadID].elemCount = 1;
		cmdPools[queueType].dir[ThreadID].keyRef = ThreadID;
	}
	else
		cmdPools[queueType].dir[ThreadID].elemCount++;
	return &cmdPools[queueType].dir[ThreadID];
}

void HIGHOMEGA::GL::CommandBuffer::BeginCommandBuffer(InstanceClass & instanceRef, unsigned int inpNumCmdBufs, unsigned int which, QUEUE_TYPE inQueueType)
{
	if (inpNumCmdBufs == 0) FATAL_ERROR("CommandBuffer: number of command buffers cannot be zero");

	queueType = inQueueType;

	VkResult result;

	if (inpNumCmdBufs != cmdBuffers.size())
	{
		if (cmdBuffers.size() > 0) DestroyCommandBuffer();
		cmdPoolRef = CreateCommandPool(instanceRef, queueType);

		cmdBuffers.resize(inpNumCmdBufs);
		ownSemaphores.resize(inpNumCmdBufs);
		for (unsigned int i = 0; i != inpNumCmdBufs; i++)
			ownSemaphores[i].Semaphore(&instanceRef);

		VkCommandBufferAllocateInfo cmdBufAllocateInfo = {};
		cmdBufAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
		cmdBufAllocateInfo.commandPool = cmdPoolRef->elem;
		cmdBufAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
		cmdBufAllocateInfo.commandBufferCount = (uint32_t)cmdBuffers.size();

		result = vkAllocateCommandBuffers(ptrToInstance->device, &cmdBufAllocateInfo, cmdBuffers.data());
		if (result != VK_SUCCESS) { FATAL_ERROR("CommandBuffer: Could not create command buffer"); }
		haveSetupCmdBuffer = true;
	}
	else
	{
		WaitForCompletion(which);
		result = vkResetCommandBuffer(cmdBuffers[which], (VkCommandBufferResetFlags)0);
		if (result != VK_SUCCESS) { FATAL_ERROR("CommandBuffer: Could not reset command buffer"); }
	}

	VkCommandBufferBeginInfo cmdBufInfo = {};
	cmdBufInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

	result = vkBeginCommandBuffer(cmdBuffers[which], &cmdBufInfo);
	if (result != VK_SUCCESS) FATAL_ERROR("CommandBuffer: Could not begin command buffer");
}

void HIGHOMEGA::GL::CommandBuffer::EndCommandBuffer(unsigned int which)
{
	if (!ptrToInstance || !cmdPoolRef) FATAL_ERROR("CommandBuffer: No instance or cmd pool supplied");

	VkResult result = vkEndCommandBuffer(cmdBuffers[which]);
	if (result != VK_SUCCESS) FATAL_ERROR("CommandBuffer: Could not end command buffer");
}

void HIGHOMEGA::GL::CommandBuffer::WaitOnSemaphores(const std::unordered_set<SemaphoreClass*>& inWaitOn, unsigned int which)
{
	if (which >= waitOn.size()) waitOn.resize(which + 1); // Still won't be waited on (uninitialized)
	waitOn[which] = inWaitOn;
}

void HIGHOMEGA::GL::CommandBuffer::WaitOnOldSemaphores(const std::unordered_set<SemaphoreClass*>& inWaitOnOld, unsigned int which)
{
	if (which >= waitOnOld.size()) waitOnOld.resize(which + 1); // Still won't be waited on (uninitialized)
	waitOnOld[which] = inWaitOnOld;
}

void HIGHOMEGA::GL::CommandBuffer::SignalSemaphores(const std::unordered_set<SemaphoreClass*>& inSignal, unsigned int which)
{
	if (which >= signal.size()) signal.resize(which + 1); // Still won't be signalled (uninitialized)
	signal[which] = inSignal;
}

void HIGHOMEGA::GL::CommandBuffer::NoCPUSync(unsigned int which)
{
	if (which >= cpuSync.size())
	{
		unsigned int prevSize = (unsigned int)cpuSync.size();
		cpuSync.resize(which + 1);
		unsigned int newSize = (unsigned int)cpuSync.size();
		for (unsigned int i = prevSize; i != newSize; i++)
			cpuSync[which] = true;
	}
	cpuSync[which] = false;
}

void HIGHOMEGA::GL::CommandBuffer::DoCPUSync(unsigned int which)
{
	if (which >= cpuSync.size())
	{
		unsigned int prevSize = (unsigned int)cpuSync.size();
		cpuSync.resize(which + 1);
		unsigned int newSize = (unsigned int)cpuSync.size();
		for (unsigned int i = prevSize; i != newSize; i++)
			cpuSync[which] = true;
	}
	cpuSync[which] = true;
}

void HIGHOMEGA::GL::CommandBuffer::SubmitCommandBuffer(unsigned int which)
{
	if (!ptrToInstance || !cmdPoolRef) FATAL_ERROR("CommandBuffer: No instance or cmd pool supplied");

	VkTimelineSemaphoreSubmitInfo timelineInfo;
	timelineInfo.sType = VK_STRUCTURE_TYPE_TIMELINE_SEMAPHORE_SUBMIT_INFO;
	timelineInfo.pNext = NULL;
	std::vector<unsigned long long> waitValues;
	std::vector<unsigned long long> signalValues;
	std::vector<VkSemaphore> waitSems;
	std::vector<VkSemaphore> signalSems;

	VkSubmitInfo submitInfo = {};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	if (which < waitOn.size())
		for (SemaphoreClass* curSem : waitOn[which])
			if (curSem->haveSemaphore && curSem->readyForWait)
			{
				if (curSem->binary)
				{
					waitValues.push_back(0u); // Nonsense
					waitSems.push_back(curSem->semaphore);
				}
				else if (curSem->signalValue > curSem->waitValue)
				{
					waitValues.push_back(curSem->waitValue);
					waitSems.push_back(curSem->semaphore);
					curSem->waitValue = curSem->signalValue;
				}
				curSem->readyForWait = false;
			}
	if (which < waitOnOld.size())
		for (SemaphoreClass* curSem : waitOnOld[which])
			if (curSem->haveSemaphore && curSem->readyForWait)
			{
				if (curSem->binary) FATAL_ERROR("Can't wait on old binary semaphores");
				else if (curSem->signalValue > curSem->waitValue)
				{
					waitValues.push_back(curSem->waitValue);
					waitSems.push_back(curSem->semaphore);
				}
			}
	if (ownSemaphores[which].haveSemaphore && ownSemaphores[which].readyForWait && ownSemaphores[which].signalValue > ownSemaphores[which].waitValue)
	{
		waitValues.push_back(ownSemaphores[which].waitValue);
		waitSems.push_back(ownSemaphores[which].semaphore);
		ownSemaphores[which].waitValue = ownSemaphores[which].signalValue;
		ownSemaphores[which].readyForWait = false;
	}

	if (which < signal.size())
		for (SemaphoreClass* curSem : signal[which])
			if (curSem->haveSemaphore)
				if (curSem->binary && !curSem->readyForWait)
				{
					signalValues.push_back(0u); // Nonsense
					curSem->readyForWait = true;
					signalSems.push_back(curSem->semaphore);
				}
				else if (!curSem->binary && !curSem->readyForWait && curSem->signalValue == curSem->waitValue)
				{
					signalValues.push_back(++curSem->signalValue);
					curSem->readyForWait = true;
					signalSems.push_back(curSem->semaphore);
				}
	if (ownSemaphores[which].haveSemaphore && !ownSemaphores[which].binary /* won't be */ && !ownSemaphores[which].readyForWait && ownSemaphores[which].signalValue == ownSemaphores[which].waitValue)
	{
		signalValues.push_back(++ownSemaphores[which].signalValue);
		ownSemaphores[which].readyForWait = true;
		signalSems.push_back(ownSemaphores[which].semaphore);
	}

	timelineInfo.waitSemaphoreValueCount = (uint32_t)waitValues.size();
	timelineInfo.pWaitSemaphoreValues = waitValues.data();
	timelineInfo.signalSemaphoreValueCount = (uint32_t)signalValues.size();
	timelineInfo.pSignalSemaphoreValues = signalValues.data();
	submitInfo.pNext = &timelineInfo;
	submitInfo.waitSemaphoreCount = (uint32_t)waitSems.size();
	submitInfo.pWaitSemaphores = waitSems.data();
	std::vector <VkPipelineStageFlags> waitMasks;
	for (int i = 0; i != submitInfo.waitSemaphoreCount; i++)
		waitMasks.push_back(VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT);
	submitInfo.pWaitDstStageMask = waitMasks.data();
	submitInfo.signalSemaphoreCount = (uint32_t)signalSems.size();
	submitInfo.pSignalSemaphores = signalSems.data();
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &cmdBuffers[which];

	VkResult result;
	{std::unique_lock<std::mutex> lk(ptrToInstance->allQueues[queueType].mtx);
	result = vkQueueSubmit(ptrToInstance->allQueues[queueType].queue, 1, &submitInfo, VK_NULL_HANDLE); }
	if (result != VK_SUCCESS)
	{
		if (result == VK_ERROR_OUT_OF_HOST_MEMORY) FATAL_ERROR("CommandBuffer could not submit and wait on queue: out of host memory");
		else if (result == VK_ERROR_OUT_OF_DEVICE_MEMORY) FATAL_ERROR("CommandBuffer could not submit and wait on queue: out of device memory");
		else FATAL_ERROR("CommandBuffer could not submit and wait on queue: device lost");
	}

	if ((which >= cpuSync.size()) || cpuSync[which]) WaitForCompletion(which);
}

void HIGHOMEGA::GL::CommandBuffer::WaitForCompletion(unsigned int which)
{
	if (!ptrToInstance || !cmdPoolRef || which >= ownSemaphores.size()) return;

	if (ownSemaphores[which].haveSemaphore && ownSemaphores[which].readyForWait && ownSemaphores[which].signalValue > ownSemaphores[which].waitValue)
	{
		VkSemaphoreWaitInfo waitInfo = {};
		waitInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
		waitInfo.semaphoreCount = 1;
		waitInfo.pSemaphores = &ownSemaphores[which].semaphore;
		waitInfo.pValues = &ownSemaphores[which].signalValue;
		waitInfo.flags = 0;
		ptrToInstance->fpWaitSemaphoresKHR(ptrToInstance->device, &waitInfo, 10000000000);
		ownSemaphores[which].waitValue = ownSemaphores[which].signalValue;
		ownSemaphores[which].readyForWait = false;
	}
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

void HIGHOMEGA::GL::ImageClass::setImageLayout(VkCommandBuffer cmdbuffer, std::vector<VkImage *> & images, VkImageAspectFlags aspectMask, VkImageLayout oldImageLayout, VkImageLayout newImageLayout, int numLayers, int baseMipLevel, int mipLevelCount, QUEUE_TRANSFER queueTransfer)
{
	if (!cachedInstance) FATAL_ERROR("Trying to do a layout transition for an uninitialized image");

	std::vector <VkImageMemoryBarrier> barriers;
	barriers.resize(images.size());
	for (int i = 0; i != images.size(); i++)
	{
		barriers[i] = {};
		barriers[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
		barriers[i].pNext = VK_NULL_HANDLE;
		switch (queueTransfer)
		{
			case TRANSFER_TO_GRAPHICS_RELEASE:
			case TRANSFER_TO_GRAPHICS_ACQUIRE:
			{
				barriers[i].srcQueueFamilyIndex = cachedInstance->allQueues[TRANSFER_QUEUE].nodeIndex;
				barriers[i].dstQueueFamilyIndex = cachedInstance->allQueues[GRAPHICS_QUEUE].nodeIndex;
				break;
			}
			case NO_TRANSFER:
			default:
			{
				barriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
				barriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
				break;
			}
		}
		barriers[i].oldLayout = oldImageLayout;
		barriers[i].newLayout = newImageLayout;
		barriers[i].image = *(images[i]);
		barriers[i].subresourceRange.aspectMask = aspectMask;
		barriers[i].subresourceRange.baseMipLevel = baseMipLevel;
		barriers[i].subresourceRange.levelCount = (mipLevelCount == 0) ? VK_REMAINING_MIP_LEVELS : mipLevelCount;
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

		if (queueTransfer == TRANSFER_TO_GRAPHICS_RELEASE)
			barriers[i].dstAccessMask = 0u;
		else if (queueTransfer == TRANSFER_TO_GRAPHICS_ACQUIRE)
			barriers[i].srcAccessMask = 0u;
	}

	vkCmdPipelineBarrier(cmdbuffer, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, VK_PIPELINE_STAGE_ALL_COMMANDS_BIT, 0, 0, nullptr, 0, nullptr, (uint32_t)barriers.size(), barriers.data());
}

void HIGHOMEGA::GL::ImageClass::CreateImageView(DEPTH_STENCIL_MODE depthStencilMode, bool useFormat, FORMAT inpFormat, TEXTURE_DIM numDims, int numLayers, int baseLayer, int mipLevelCount, VkImageView & viewPtr, bool & inpHaveImageView)
{
	if (cachedInstance == nullptr) FATAL_ERROR("We do not have a pointer to the Vulkan instance");

	VkImageViewCreateInfo attachmentView = {};
	attachmentView.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	attachmentView.pNext = VK_NULL_HANDLE;
	attachmentView.format = useFormat?((VkFormat)inpFormat):(depthStencilMode != NONE?cachedInstance->selectedDepthFormat:cachedInstance->colorFormat);
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
	if (result != VK_SUCCESS) FATAL_ERROR("Could not create image-view");

	inpHaveImageView = true;
}

void HIGHOMEGA::GL::ImageClass::CreateImage(bool depthStencil, bool useFormat, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM numDims, int numLayers, int mipLevelCount, bool usedViaSampler, bool usedAsStorageTarget, QUEUE_TYPE queueType)
{
	if (cachedInstance == nullptr) FATAL_ERROR("We do not have a pointer to the Vulkan instance");

	VkImageCreateInfo imageCreateStruct = {};
	imageCreateStruct.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageCreateStruct.pNext = VK_NULL_HANDLE;
	imageCreateStruct.imageType = GetVkImageTypeFromDim (numDims);
	imageCreateStruct.format = useFormat ? ((VkFormat)inpFormat) : (depthStencil ? cachedInstance->selectedDepthFormat : cachedInstance->colorFormat);
	imageCreateStruct.extent = { (uint32_t)w, (uint32_t)h, (uint32_t)d };
	imageCreateStruct.pQueueFamilyIndices = &cachedInstance->allQueues[queueType].nodeIndex;
	imageCreateStruct.queueFamilyIndexCount = 1;
	imageCreateStruct.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
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
	if (result != VK_SUCCESS) FATAL_ERROR("Could not create image");

	haveImage = true;
}

void HIGHOMEGA::GL::ImageClass::CreateMemoryForImage(QUEUE_TYPE queueType)
{
	if (cachedInstance == nullptr) FATAL_ERROR("We do not have a pointer to the Vulkan instance");

	VkMemoryAllocateInfo mem_alloc = {};
	mem_alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	mem_alloc.pNext = VK_NULL_HANDLE;
	mem_alloc.allocationSize = 0;
	mem_alloc.memoryTypeIndex = 0;

	VkMemoryRequirements memReqs;

	vkGetImageMemoryRequirements(cachedInstance->device, image, &memReqs);
	getMemoryType(cachedInstance, memReqs.memoryTypeBits, MEMORY_DEVICE_LOCAL, &mem_alloc.memoryTypeIndex);
	
	try
	{
		subAllocs = AllocMem(cachedInstance->device, mem_alloc, memReqs, IMAGE, cachedInstance->SupportsSparseResources());
		haveSubAlloc = true;
	}
	catch (...)
	{
		RemovePast();
		FATAL_ERROR("Could not allocate image memory");
	}

	VkResult result;
	if (cachedInstance->SupportsSparseResources())
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		std::unique_lock <std::mutex> lk2(cachedInstance->allQueues[queueType].mtx);
		FenceClass sparseFence;
		sparseFence.Fence(cachedInstance);
		sparseFence.Reset();
		VkSparseImageOpaqueMemoryBindInfo imageMemoryBinds;
		imageMemoryBinds.image = image;
		std::vector<VkSparseMemoryBind> memoryBinds;
		unsigned long long resourceOffset = 0ull;
		for (std::tuple<int, unsigned long long, MEMORY_MANAGER::SubAlloc>& curSubAlloc : subAllocs)
		{
			VkSparseMemoryBind curImgBind = {};
			curImgBind.memory = std::get<2>(curSubAlloc).mem;
			curImgBind.memoryOffset = (VkDeviceSize)std::get<2>(curSubAlloc).offset;
			curImgBind.size = (VkDeviceSize)std::get<2>(curSubAlloc).len;
			curImgBind.resourceOffset = (VkDeviceSize)resourceOffset;
			curImgBind.flags = VK_SPARSE_MEMORY_BIND_METADATA_BIT;
			resourceOffset += std::get<2>(curSubAlloc).len;
			memoryBinds.push_back(curImgBind);
		}
		imageMemoryBinds.bindCount = (uint32_t)memoryBinds.size();
		imageMemoryBinds.pBinds = memoryBinds.data();
		VkBindSparseInfo vkBindSparseInfo = {};
		vkBindSparseInfo.sType = VK_STRUCTURE_TYPE_BIND_SPARSE_INFO;
		vkBindSparseInfo.imageOpaqueBindCount = 1;
		vkBindSparseInfo.pImageOpaqueBinds = &imageMemoryBinds;
		result = vkQueueBindSparse(cachedInstance->allQueues[queueType].queue, 1, &vkBindSparseInfo, sparseFence.fence);
		sparseFence.Wait();
	}
	else
	{
		std::unique_lock <std::mutex> lk(mem_manager_mutex);
		result = vkBindImageMemory(cachedInstance->device, image, std::get<2>(*subAllocs.begin()).mem, std::get<2>(*subAllocs.begin()).offset);
	}
	if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not bind image to memory"); }
}

void HIGHOMEGA::GL::ImageClass::CreateSampler(MIN_MAG_FILTER minFilter, MIN_MAG_FILTER magFilter, MIPMAP_MODE mipMapMode, TEXTURE_ADDRESS_MODE uAddress, TEXTURE_ADDRESS_MODE vAddress, TEXTURE_ADDRESS_MODE wAddress, float mipLodBias, float minLod, float maxLod, bool enableAnisotropy, float maxAnisotropy)
{
	if (cachedInstance == nullptr) FATAL_ERROR("We do not have a pointer to the Vulkan instance");

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
	if (result != VK_SUCCESS) FATAL_ERROR("Could not create sampler");

	haveSampler = true;
}

void HIGHOMEGA::GL::ImageClass::RemovePast()
{
	if (cachedInstance == nullptr)
	{
		if (!instanceEverSet) return;
		FATAL_ERROR("We do not have a pointer to the Vulkan instance");
	}

	clearCmdBuffer.DestroyCommandBuffer();
	setupStandAloneCmdBuffer.DestroyCommandBuffer();
	setupTextureTransferCmdBuffer.DestroyCommandBuffer();
	setupTextureGraphicsCmdBuffer.DestroyCommandBuffer();
	uploadCmdBuffer.DestroyCommandBuffer();
	downloadCmdBuffer.DestroyCommandBuffer();
	copyCmdBuffer.DestroyCommandBuffer();

	semaphore.RemovePast();
	if ((haveKTXVulkanTexture && ktx2VDIRef) || haveImageView || layerView.size()) Instance.GPUFlush();
	if (haveKTXVulkanTexture && ktx2VDIRef)
	{
		HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::deviceCached = &ktx2VDIRef->elem.ktxVDI.device;
		ktxVulkanTexture_subAllocatorCallbacks subAllocCallbacks;
		subAllocCallbacks.allocMemFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::AllocMemCWrapper;
		subAllocCallbacks.bindBufferFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::BindBufferMemoryCWrapper;
		subAllocCallbacks.bindImageFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::BindImageMemoryCWrapper;
		subAllocCallbacks.memoryMapFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::MapMemoryCWrapper;
		subAllocCallbacks.memoryUnmapFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::UnmapMemoryCWrapper;
		subAllocCallbacks.freeMemFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::FreeMemCWrapper;
		ktxVulkanTexture_Destruct_WithSuballocator(&ktxVulkanTexture, ktx2VDIRef->elem.ktxVDI.device, nullptr, &subAllocCallbacks);
		{std::unique_lock <std::mutex> lk(ktx2VDIPools.mtx);
		ktx2VDIPools.dir[ktx2VDIRef->keyRef].elemCount--;
		if (ktx2VDIPools.dir[ktx2VDIRef->keyRef].elemCount == 0)
		{
			ktx2VDIPools.dir[ktx2VDIRef->keyRef].elem.Destroy();
			ktx2VDIPools.dir.erase(ktx2VDIRef->keyRef);
		}}
		ktx2VDIRef = nullptr;
	}
	if (haveImageView) vkDestroyImageView(cachedInstance->device, view, nullptr);
	for (VkImageView & curView : layerView) {
		vkDestroyImageView(cachedInstance->device, curView, nullptr);
	}
	layerView.clear();
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
	downloadBufferSize = 0u;
	cachedUploadBuffer = cachedDownloadBuffer = VK_NULL_HANDLE;
}

HIGHOMEGA::GL::ImageClass::ImageClass()
{
	uploadedTexture = false;
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
	downloadBufferSize = 0u;
	cachedUploadBuffer = cachedDownloadBuffer = VK_NULL_HANDLE;
}

std::vector<ImageClass> HIGHOMEGA::GL::ImageClass::FromSwapChain(InstanceClass & inpInstance)
{
	uint32_t swapChainImageCount = 0;

	VkResult result = inpInstance.fpGetSwapchainImagesKHR(inpInstance.device, inpInstance.swapChain, &swapChainImageCount, nullptr);
	if (result != VK_SUCCESS) { FATAL_ERROR("Could not get swapchain image count"); }

	std::vector <VkImage> tmpImages;
	tmpImages.resize(swapChainImageCount);

	std::vector <ImageClass> images;
	images.resize(swapChainImageCount);

	result = inpInstance.fpGetSwapchainImagesKHR(inpInstance.device, inpInstance.swapChain, &swapChainImageCount, tmpImages.data());
	if (result != VK_SUCCESS) { FATAL_ERROR("Could not get swapchain images"); }

	for (uint32_t i = 0; i < swapChainImageCount; i++)
	{
		images[i].haveImage = false; // Image provided by swap chain. No need to destroy as we didn't make it.
		images[i].cachedInstance = &inpInstance;
		images[i].instanceEverSet = true;
	}

	try { images[0].setupStandAloneCmdBuffer.BeginCommandBuffer(inpInstance); }
	catch (...) { FATAL_ERROR("Could not begin setup cmd buffer for swapchain image"); }

	for (uint32_t i = 0; i < swapChainImageCount; i++)
	{
		images[i].image = tmpImages[i];

		std::vector <VkImage *> imagesInFrontOfBarrier;
		imagesInFrontOfBarrier.push_back(&images[i].image);

		images[i].setImageLayout(images[0].setupStandAloneCmdBuffer.cmdBuffers[0], imagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_PRESENT_SRC_KHR, 1, 0, 1);

		images[i].CreateImageView(NONE, false, (FORMAT)0, _2D, 1, 0, 1, images[i].view, images[i].haveImageView);
		images[i].semaphore.Semaphore(&inpInstance, true);
		images[i].haveImageView = true;
	}

	try { images[0].setupStandAloneCmdBuffer.EndCommandBuffer(); }
	catch (...) { FATAL_ERROR("Could not end setup cmd buffer for swapchain image"); }

	try { images[0].setupStandAloneCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { FATAL_ERROR("Could not submit setup cmd buffer for swapchain image"); }

	return images;
}

std::function<unsigned int(unsigned int)>& HIGHOMEGA::GL::ImageClass::getSurfaceBasedWidthCallback()
{
	return surfaceBasedWidthCallback;
}

std::function<unsigned int(unsigned int)>& HIGHOMEGA::GL::ImageClass::getSurfaceBasedHeightCallback()
{
	return surfaceBasedHeightCallback;
}

void HIGHOMEGA::GL::ImageClass::CreateOnScreenDepthStencil(InstanceClass & ptrToInstance, int w, int h)
{
	useFormat = false; format = (FORMAT)0; width = w; height = h; depth = 1; numDims = _2D; layers = 1; depthStencilMode = SAMPLE_NONE;
	createSampler = false; usedAsStorageTarget = false; linearFiltering = false; clampSamples = false; createCubeMap = false;
	CreateStandaloneImage(ptrToInstance);
	// AddSwapchainDependentImage(...) unnecessary here. Another call to CreateSwapchain(...) will recreate this anyway.
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenDepthStencil(InstanceClass & ptrToInstance, int w, int h, DEPTH_STENCIL_MODE inpDepthStencil,
	bool swapChainDependent,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback)
{
	useFormat = false; format = (FORMAT)0; width = w; height = h; depth = 1; numDims = _2D; layers = 1; if (inpDepthStencil == NONE) inpDepthStencil = SAMPLE_NONE; depthStencilMode = inpDepthStencil; 
	createSampler = (inpDepthStencil != SAMPLE_NONE); usedAsStorageTarget = false; linearFiltering = false; clampSamples = false; createCubeMap = false;
	CreateStandaloneImage(ptrToInstance);
	if (swapChainDependent) ptrToInstance.AddSwapchainDependentImage(*this, inSurfaceBasedWidthCallback, inSurfaceBasedHeightCallback);
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenColorAttachment(InstanceClass & ptrToInstance, FORMAT inFormat, int w, int h, bool inpLinearFiltering, bool inpClampSamples,
	bool swapChainDependent,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback)
{
	useFormat = true; format = inFormat; width = w; height = h; depth = 1; numDims = _2D; layers = 1; depthStencilMode = NONE;
	createSampler = true; usedAsStorageTarget = false; linearFiltering = inpLinearFiltering; clampSamples = inpClampSamples; createCubeMap = false;
	CreateStandaloneImage(ptrToInstance);
	if (swapChainDependent) ptrToInstance.AddSwapchainDependentImage(*this, inSurfaceBasedWidthCallback, inSurfaceBasedHeightCallback);
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenColorArrayAttachment(InstanceClass & ptrToInstance, FORMAT inFormat, int w, int h, int numLayers, bool inpLinearFiltering, bool inpClampSamples,
	bool swapChainDependent,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback)
{
	useFormat = true; format = inFormat; width = w; height = h; depth = 1; numDims = _2D_ARRAY; layers = numLayers; depthStencilMode = NONE;
	createSampler = true; usedAsStorageTarget = false; linearFiltering = inpLinearFiltering; clampSamples = inpClampSamples; createCubeMap = false;
	CreateStandaloneImage(ptrToInstance);
	if (swapChainDependent) ptrToInstance.AddSwapchainDependentImage(*this, inSurfaceBasedWidthCallback, inSurfaceBasedHeightCallback);
}

void HIGHOMEGA::GL::ImageClass::CreateImageStore(InstanceClass & ptrToInstance, FORMAT inpFormat, int w, int h, int d, TEXTURE_DIM inpNumDims, bool inpLinearFiltering,
	bool swapChainDependent,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback)
{
	useFormat = true; format = inpFormat; width = w; height = h; depth = d; numDims = inpNumDims; layers = 1; depthStencilMode = NONE;
	createSampler = true; usedAsStorageTarget = true; linearFiltering = inpLinearFiltering; clampSamples = false; createCubeMap = false;
	CreateStandaloneImage(ptrToInstance);
	if (swapChainDependent) ptrToInstance.AddSwapchainDependentImage(*this, inSurfaceBasedWidthCallback, inSurfaceBasedHeightCallback);
}

void HIGHOMEGA::GL::ImageClass::CreateCubeMap(InstanceClass & ptrToInstance, FORMAT inpFormat, int w, int h,
	bool swapChainDependent,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback)
{
	useFormat = true; format = inpFormat; width = w; height = h; depth = 1; numDims = _CUBE; layers = 6; depthStencilMode = NONE;
	createSampler = true; usedAsStorageTarget = false; linearFiltering = true; clampSamples = true; createCubeMap = true;
	CreateStandaloneImage(ptrToInstance);
	if (swapChainDependent) ptrToInstance.AddSwapchainDependentImage(*this, inSurfaceBasedWidthCallback, inSurfaceBasedHeightCallback);
}

void HIGHOMEGA::GL::ImageClass::CreateOffScreenDepthStencilCubeMap(InstanceClass& ptrToInstance, int w, int h, DEPTH_STENCIL_MODE inpDepthStencil,
	bool swapChainDependent,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedWidthCallback,
	const std::function<unsigned int(unsigned int)>& inSurfaceBasedHeightCallback)
{
	useFormat = false; format = (FORMAT)0; width = w; height = h; depth = 1; numDims = _CUBE; layers = 6; if (inpDepthStencil == NONE) inpDepthStencil = SAMPLE_NONE; depthStencilMode = inpDepthStencil;
	createSampler = (inpDepthStencil != SAMPLE_NONE); usedAsStorageTarget = false; linearFiltering = false; clampSamples = false; createCubeMap = true;
	CreateStandaloneImage(ptrToInstance);
	if (swapChainDependent) ptrToInstance.AddSwapchainDependentImage(*this, inSurfaceBasedWidthCallback, inSurfaceBasedHeightCallback);
}

bool HIGHOMEGA::GL::ImageClass::CreateTexture(InstanceClass & ptrToInstance, std::string belong, std::string fileName, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat)
{
	unsigned char *content = nullptr;
	unsigned int contentSize;
	HIGHOMEGA::ResourceLoader::LOAD_LOCATION loadLocation;
	if (HIGHOMEGA::ResourceLoader::Load(belong, fileName, &content, contentSize, loadLocation) != HIGHOMEGA::ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS) return false;
	CreateTextureFromFileOrData(ptrToInstance, content, contentSize, fileName.find(".tga") != std::string::npos ? IMAGE_DATA_TGA : IMAGE_DATA_KTX, 0, 0, inD, inIs3D, inIsArray, isCube, doMipMapping, inpFormat);
	delete[] content;
	return true;
}

bool HIGHOMEGA::GL::ImageClass::CreateTexture(InstanceClass & ptrToInstance, unsigned int inW, unsigned int inH, unsigned char *inData, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat)
{
	CreateTextureFromFileOrData(ptrToInstance, inData, 0, IMAGE_DATA_RGB, inW, inH, inD, inIs3D, inIsArray, isCube, doMipMapping, inpFormat);
	return true;
}

void HIGHOMEGA::GL::ImageClass::CreateStandaloneImage(InstanceClass & ptrToInstance)
{
	cachedInstance = &ptrToInstance;
	uploadedTexture = false;
	instanceEverSet = true;

	isArray = is3D = false;
	if (layers > 1) isArray = true;
	else if (depth > 1) is3D = true;

	try { CreateImage(depthStencilMode != NONE, useFormat, format, width, height, depth, createCubeMap ? _2D : numDims, layers, 1, createSampler, usedAsStorageTarget, GRAPHICS_QUEUE); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not create image"); }

	try { CreateMemoryForImage(GRAPHICS_QUEUE); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not create and bind memory for image"); }

	std::vector <VkImage *> imageInFrontOfBarrier;
	imageInFrontOfBarrier.push_back(&image);

	try
	{
		CreateImageView(depthStencilMode, useFormat, format, createCubeMap ? _CUBE : numDims, layers, 0, 1, view, haveImageView);
		if (layers > 1) {
			for (int i = 0; i != layers; i++)
			{
				bool tmpCreatedFlag = false;
				VkImageView curImageView;
				CreateImageView(depthStencilMode, useFormat, format, _2D, 1, i, 1, curImageView, tmpCreatedFlag);
				layerView.push_back(curImageView);
			}
		}
	}
	catch (...) { RemovePast(); FATAL_ERROR("Could not create image view"); }

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
		catch (...) { RemovePast(); FATAL_ERROR("Could not create sampler"); }
	}


	try { setupStandAloneCmdBuffer.BeginCommandBuffer(*cachedInstance); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for standalone image"); }

	setImageLayout(setupStandAloneCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier,
		(depthStencilMode != NONE) ? (VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT) : VK_IMAGE_ASPECT_COLOR_BIT,
		VK_IMAGE_LAYOUT_UNDEFINED,
		(depthStencilMode == SAMPLE_NONE) ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL : VK_IMAGE_LAYOUT_GENERAL,
		layers, 0, 1);

	try { setupStandAloneCmdBuffer.EndCommandBuffer(); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for standalone image"); }

	try { setupStandAloneCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for standalone image"); }

	semaphore.Semaphore(cachedInstance);
}

void HIGHOMEGA::GL::LibKTX2VDIWrapper::Create(InstanceClass& ptrToInstance)
{
	cachedInstance = &ptrToInstance;
	VkCommandPoolCreateInfo cmdPoolInfo = {};
	cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	cmdPoolInfo.queueFamilyIndex = ptrToInstance.allQueues[TRANSFER_QUEUE].nodeIndex;
	cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	VkResult result = vkCreateCommandPool(cachedInstance->device, &cmdPoolInfo, nullptr, &cmdPool);
	if (result != VK_SUCCESS) { FATAL_ERROR("LibKTX2VDIWrapper: Could not create command pool"); }

	KTX_error_code ktxResult = ktxVulkanDeviceInfo_Construct(&ktxVDI, cachedInstance->physicalDevice, cachedInstance->device, cachedInstance->allQueues[TRANSFER_QUEUE].queue, cmdPool, nullptr);
	if (ktxResult != KTX_SUCCESS) { vkDestroyCommandPool(cachedInstance->device, cmdPool, nullptr); FATAL_ERROR("LibKTX2VDIWrapper: Could not supply vulkan device info to libKTX"); }
}

void HIGHOMEGA::GL::LibKTX2VDIWrapper::Destroy()
{
	ktxVulkanDeviceInfo_Destruct(&ktxVDI);
	vkDestroyCommandPool(cachedInstance->device, cmdPool, nullptr);
}

void HIGHOMEGA::GL::ImageClass::CreateTextureFromFileOrData(InstanceClass & ptrToInstance, unsigned char *data, unsigned int dataSize, PROVIDED_IMAGE_DATA_TYPE dataType, unsigned int inW, unsigned int inH, unsigned int inD, bool inIs3D, bool inIsArray, bool isCube, bool doMipMapping, FORMAT inpFormat)
{
	cachedInstance = &ptrToInstance;
	usedAsStorageTarget = false;
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
		ktx2VDIRef = nullptr;
		{std::lock_guard <std::mutex> lk(ktx2VDIPools.mtx);
		if (ktx2VDIPools.dir.find(ThreadID) == ktx2VDIPools.dir.end())
		{
			ktx2VDIPools.dir[ThreadID].elem.Create(*cachedInstance);
			ktx2VDIPools.dir[ThreadID].elemCount = 1;
			ktx2VDIPools.dir[ThreadID].keyRef = ThreadID;
		}
		else
			ktx2VDIPools.dir[ThreadID].elemCount++;
		ktx2VDIRef = &ktx2VDIPools.dir[ThreadID]; }

		ktxTexture2* kTexture;
		KTX_error_code result = ktxTexture_CreateFromMemory(data, dataSize, KTX_TEXTURE_CREATE_NO_FLAGS, (ktxTexture**)&kTexture);
		if (result != KTX_SUCCESS) { RemovePast(); FATAL_ERROR("Error creating ktxTexture from ktx file"); }
		if (ktxTexture2_NeedsTranscoding(kTexture))
		{
			ktx_transcode_fmt_e tf = KTX_TTF_BC3_RGBA;
			result = ktxTexture2_TranscodeBasis(kTexture, KTX_TTF_BC3_RGBA, 0);
			if (result != KTX_SUCCESS) { RemovePast(); FATAL_ERROR("Error transcoding ktx file to BC3"); }
		}
		HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::deviceCached = &ktx2VDIRef->elem.ktxVDI.device;
		ktxVulkanTexture_subAllocatorCallbacks subAllocCallbacks;
		subAllocCallbacks.allocMemFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::AllocMemCWrapper;
		subAllocCallbacks.bindBufferFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::BindBufferMemoryCWrapper;
		subAllocCallbacks.bindImageFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::BindImageMemoryCWrapper;
		subAllocCallbacks.memoryMapFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::MapMemoryCWrapper;
		subAllocCallbacks.memoryUnmapFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::UnmapMemoryCWrapper;
		subAllocCallbacks.freeMemFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::FreeMemCWrapper;
		ktxVulkanTexture_QueueGuardCallbacks queueGuardCallbacks;
		queueGuardCallbacks.queueLockFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::LockQueue;
		queueGuardCallbacks.queueUnlockFuncPtr = HIGHOMEGA::GL::MEMORY_MANAGER::LIBKTX2::UnlockQueue;
		result = ktxTexture_VkUploadEx_WithSuballocatorAndQueueGuard((ktxTexture*)kTexture, &ktx2VDIRef->elem.ktxVDI, &ktxVulkanTexture, VK_IMAGE_TILING_OPTIMAL, VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &subAllocCallbacks, &queueGuardCallbacks);
		if (result != KTX_SUCCESS) { RemovePast(); FATAL_ERROR("Error creating ktxVulkanTexture from ktx file"); }
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

		std::vector <VkImage*> imageInFrontOfBarrier;
		imageInFrontOfBarrier.push_back(&image);

		try { setupTextureTransferCmdBuffer.BeginCommandBuffer(*cachedInstance, 1u, 0u, TRANSFER_QUEUE); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for create compressed texture"); }

		setImageLayout(setupTextureTransferCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, layers, 0, 0, TRANSFER_TO_GRAPHICS_RELEASE);

		try { setupTextureTransferCmdBuffer.EndCommandBuffer(); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for create compressed texture"); }

		try { setupTextureTransferCmdBuffer.SubmitCommandBuffer(); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for create compressed texture"); }

		try { setupTextureGraphicsCmdBuffer.BeginCommandBuffer(*cachedInstance); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin second setup cmd buffer for create compressed texture"); }

		setImageLayout(setupTextureGraphicsCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, layers, 0, 0, TRANSFER_TO_GRAPHICS_ACQUIRE);

		try { setupTextureGraphicsCmdBuffer.EndCommandBuffer(); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin second setup cmd buffer for create compressed texture"); }

		try { setupTextureGraphicsCmdBuffer.SubmitCommandBuffer(); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not begin second setup cmd buffer for create compressed texture"); }

		bool enableAnisotropicFiltering = doMipMapping;

		try {
			CreateSampler(ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR, ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR,
				ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
				ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
				0.0f, 0.0f, (float)mipLevels, enableAnisotropicFiltering, enableAnisotropicFiltering ? 8.0f : 1.0f);
		}
		catch (...) { RemovePast(); FATAL_ERROR("Could not create sampler for ktx"); }

		TEXTURE_DIM selectedDim = _2D;
		if (is3D) {
			selectedDim = _3D;
		}
		else if (isArray) {
			selectedDim = _2D_ARRAY;
		}

		try { CreateImageView(NONE, true, (FORMAT)format, isCube ? _CUBE : selectedDim, layers, 0, mipLevels, view, haveImageView); }
		catch (...) { RemovePast(); FATAL_ERROR("Could not create image view for ktx texture"); }

		return;
	}
	else
	{
		if (HIGHOMEGA::ResourceLoader::TGALoad(data, (int *)&width, (int *)&height, &feedData, &bpp) != HIGHOMEGA::ResourceLoader::TGA_PARSE_RESULT::TGA_SUCCESS)
			FATAL_ERROR("Bad TGA format");
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
		RemovePast(); FATAL_ERROR("format cannot be blitted...");
	}

	TEXTURE_DIM selectedDim = _2D;
	if (is3D) {
		selectedDim = _3D;
	} else if (isArray) {
		selectedDim = _2D_ARRAY;
	}

	try { CreateImage(false, true, format, width, height, depth, selectedDim, layers, mipLevels, true, false, TRANSFER_QUEUE); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not create image"); }

	try { CreateMemoryForImage(TRANSFER_QUEUE); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not create and bind memory for image"); }

	if (imageStagingBuffer.getSize() < data_size) imageStagingBuffer.Buffer(MEMORY_HOST_VISIBLE, TRANSFER_QUEUE, QUEUE_CONCURRENT, USAGE_DST | USAGE_SRC, Instance, nullptr, data_size);

	imageStagingBuffer.UploadSubData(0, feedData, data_size);

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

	try { setupTextureTransferCmdBuffer.BeginCommandBuffer(*cachedInstance, 1u, 0u, TRANSFER_QUEUE); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not begin setup cmd buffer for create texture"); }

	std::vector <VkImage *> imageInFrontOfBarrier;
	imageInFrontOfBarrier.push_back(&image);

	setImageLayout(setupTextureTransferCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layers, 0, mipLevels);

	vkCmdCopyBufferToImage(setupTextureTransferCmdBuffer.cmdBuffers[0], imageStagingBuffer.buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, (uint32_t)bufferCopyRegions.size(), bufferCopyRegions.data());

	setImageLayout(setupTextureTransferCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, 0, 1, TRANSFER_TO_GRAPHICS_RELEASE);

	try { setupTextureTransferCmdBuffer.EndCommandBuffer(); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not end setup cmd buffer for create texture"); }

	try { setupTextureTransferCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit setup cmd buffer for create texture"); }

	try { setupTextureGraphicsCmdBuffer.BeginCommandBuffer(*cachedInstance); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not begin second setup cmd buffer for create texture"); }

	setImageLayout(setupTextureGraphicsCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, 0, 1, TRANSFER_TO_GRAPHICS_ACQUIRE);

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

		setImageLayout(setupTextureGraphicsCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layers, i, 1);

		vkCmdBlitImage(setupTextureGraphicsCmdBuffer.cmdBuffers[0], image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &imageBlit, VK_FILTER_LINEAR);

		setImageLayout(setupTextureGraphicsCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, i, 1);
	}

	setImageLayout(setupTextureGraphicsCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, layers, 0, mipLevels);

	try { setupTextureGraphicsCmdBuffer.EndCommandBuffer(); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not end second setup cmd buffer for create texture"); }

	try { setupTextureGraphicsCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not submit second setup cmd buffer for create texture"); }

	bool enableAnisotropicFiltering = doMipMapping;

	try {
		CreateSampler(ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR, ImageClass::MIN_MAG_FILTER::MIN_MAG_LINEAR,
			ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
			ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT, ImageClass::TEXTURE_ADDRESS_MODE::REPEAT,
			0.0f, 0.0f, (float)mipLevels, enableAnisotropicFiltering, enableAnisotropicFiltering ? 8.0f : 1.0f);
	}
	catch (...) { RemovePast(); FATAL_ERROR("Could not create sampler"); }

	try { CreateImageView(NONE, true, format, isCube ? _CUBE : selectedDim, layers, 0, mipLevels, view, haveImageView); }
	catch (...) { RemovePast(); FATAL_ERROR("Could not create image view for texture"); }
}

void HIGHOMEGA::GL::ImageClass::ClearColors(std::vector <ImageClass *> & images, ImageClearColor clearColor)
{
	images[0]->mipLevels = 1;

	if (!images[0]->recordedClearCmdBuffer)
	{
		try { images[0]->clearCmdBuffer.BeginCommandBuffer(*(images[0]->cachedInstance)); }
		catch (...) { FATAL_ERROR("Could not begin setup cmd buffer for clear color"); }

		std::vector <VkImage *> imagesInFrontOfBarrier;
		for (ImageClass * curImg : images)
			imagesInFrontOfBarrier.push_back(&curImg->image);

		images[0]->setImageLayout(images[0]->clearCmdBuffer.cmdBuffers[0], imagesInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, 0, images[0]->mipLevels);

		VkImageSubresourceRange subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS };
		VkClearColorValue clearWith;
		if ( clearColor.useFloat )
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
		catch (...) { FATAL_ERROR("Could not end setup cmd buffer for clear color"); }

		images[0]->recordedClearCmdBuffer = true;
	}

	images[0]->clearCmdBuffer.WaitOnSemaphores(std::unordered_set<SemaphoreClass*>{ &images[0]->semaphore });
	images[0]->clearCmdBuffer.SignalSemaphores(std::unordered_set<SemaphoreClass*>{ &images[0]->semaphore });
	images[0]->clearCmdBuffer.NoCPUSync();
	try { images[0]->clearCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { FATAL_ERROR("Could not submit setup cmd buffer for clear color"); }
}

void HIGHOMEGA::GL::ImageClass::CopyImages(std::vector <ImageClass *> & copySource, std::vector <ImageClass *> & copyTarget)
{
	copySource[0]->mipLevels = 1;

	if (!copySource[0]->recordedCopyCmdBuffer)
	{
		try { copySource[0]->copyCmdBuffer.BeginCommandBuffer(*(copySource[0]->cachedInstance)); }
		catch (...) { FATAL_ERROR("Could not begin setup cmd buffer for image to image copy"); }

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
		catch (...) { FATAL_ERROR("Could not end setup cmd buffer for image to image copy"); }

		copySource[0]->recordedCopyCmdBuffer = true;
	}

	try { copySource[0]->copyCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { FATAL_ERROR("Could not submit setup cmd buffer for image to image copy"); }
}

void HIGHOMEGA::GL::ImageClass::DownloadData(bool shaderReadAfterwards)
{
	if (cachedInstance == nullptr) FATAL_ERROR("We do not have a pointer to the Vulkan instance for image download");

	downloadBufferSize = width * height * depth * layers * FormatSize(format);
	if (imageStagingBuffer.getSize() < downloadBufferSize) imageStagingBuffer.Buffer (MEMORY_HOST_VISIBLE, TRANSFER_QUEUE, QUEUE_CONCURRENT, USAGE_DST | USAGE_SRC, Instance, nullptr, downloadBufferSize);

	if (cachedDownloadBuffer != imageStagingBuffer.buffer || !recordedDownloadCmdBuffer)
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
		catch (...) { FATAL_ERROR("Could not begin setup cmd buffer for copy to buffer"); }

		std::vector <VkImage *> imageInFrontOfBarrier;
		imageInFrontOfBarrier.push_back(&image);

		setImageLayout(downloadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, layers, 0, mipLevels);

		vkCmdCopyImageToBuffer(downloadCmdBuffer.cmdBuffers[0], image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, imageStagingBuffer.buffer, (uint32_t)imageCopyRegions.size(), imageCopyRegions.data());

		setImageLayout(downloadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, shaderReadAfterwards ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_GENERAL, layers, 0, mipLevels);

		try { downloadCmdBuffer.EndCommandBuffer(); }
		catch (...) { FATAL_ERROR("Could not end setup cmd buffer for copy to buffer"); }

		cachedDownloadBuffer = imageStagingBuffer.buffer;
		recordedDownloadCmdBuffer = true;
	}

	try { downloadCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { FATAL_ERROR("Could not submit setup cmd buffer for copy to buffer"); }

	if (!downloadData) downloadData = new unsigned char[downloadBufferSize];
	imageStagingBuffer.DownloadSubData(0, downloadData, downloadBufferSize);
}

void HIGHOMEGA::GL::ImageClass::FreeLoadedData()
{
	if (downloadData) delete[] downloadData;
	downloadData = nullptr;
	downloadBufferSize = 0u;
}

unsigned char * HIGHOMEGA::GL::ImageClass::DownloadedData()
{
	return downloadData;
}

unsigned int HIGHOMEGA::GL::ImageClass::DownloadedDataSize()
{
	return downloadBufferSize;
}

void HIGHOMEGA::GL::ImageClass::UploadData(unsigned char* inData, unsigned int inDataSize, bool shaderReadAfterwards)
{
	if (cachedInstance == nullptr) FATAL_ERROR("We do not have a pointer to the Vulkan instance for image upload");

	unsigned int imageDataSize = width * height * depth * layers * FormatSize(format);
	if (inDataSize > imageDataSize) FATAL_ERROR("Image too small for uploaded data");

	if (imageStagingBuffer.getSize() < imageDataSize) imageStagingBuffer.Buffer(MEMORY_HOST_VISIBLE, TRANSFER_QUEUE, QUEUE_CONCURRENT, USAGE_DST | USAGE_SRC, Instance, nullptr, imageDataSize);
	imageStagingBuffer.UploadSubData(0, inData, inDataSize);

	if (cachedUploadBuffer != imageStagingBuffer.buffer || !recordedUploadCmdBuffer)
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
		catch (...) { FATAL_ERROR("Could not begin setup cmd buffer for copy to image"); }

		std::vector <VkImage*> imageInFrontOfBarrier;
		imageInFrontOfBarrier.push_back(&image);

		setImageLayout(uploadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, layers, 0, mipLevels);

		vkCmdCopyBufferToImage(uploadCmdBuffer.cmdBuffers[0], imageStagingBuffer.buffer, image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, (uint32_t)imageCopyRegions.size(), imageCopyRegions.data());

		setImageLayout(uploadCmdBuffer.cmdBuffers[0], imageInFrontOfBarrier, VK_IMAGE_ASPECT_COLOR_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, shaderReadAfterwards ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_GENERAL, layers, 0, mipLevels);

		try { uploadCmdBuffer.EndCommandBuffer(); }
		catch (...) { FATAL_ERROR("Could not end setup cmd buffer for copy to image"); }

		cachedUploadBuffer = imageStagingBuffer.buffer;
		recordedUploadCmdBuffer = true;
	}

	try { uploadCmdBuffer.SubmitCommandBuffer(); }
	catch (...) { FATAL_ERROR("Could not submit setup cmd buffer for copy to image"); }
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
	if (!haveRenderPass) return;
	if (cachedInstance == nullptr)
	{
		if (!instanceEverSet) return;
		FATAL_ERROR("We do not have a pointer to the Vulkan instance");
	}

	Instance.GPUFlush();
	if (haveRenderPass) vkDestroyRenderPass(cachedInstance->device, renderPass, nullptr);
	if (mode == ON_SCREEN)
	{
		for (int i = 0; i != numFrameBuffersToDestroy; i++)
			vkDestroyFramebuffer(cachedInstance->device, swapChainBuffers[i], nullptr);
		swapChainBuffers.clear();
		colorAttachments.clear();
		colorAttachmentLayers.clear();
	}
	attachmentsSampler.RemovePast();
	if (haveAttachmentsFrameBuffer) vkDestroyFramebuffer(cachedInstance->device, attachmentsFrameBuffer, nullptr);

	haveRenderPass = false;
	haveAttachmentsFrameBuffer = false;
}

void HIGHOMEGA::GL::FramebufferClass::RemovePast()
{
	RemovePast((int)swapChainBuffers.size());
}

bool HIGHOMEGA::GL::FramebufferClass::IsSwapchainDependent()
{
	for (ImageClass* curImg : colorAttachments)
		if (curImg->getSurfaceBasedWidthCallback() || curImg->getSurfaceBasedHeightCallback()) return true;
	if (depthStencilAttachment && (depthStencilAttachment->getSurfaceBasedWidthCallback() || depthStencilAttachment->getSurfaceBasedHeightCallback())) return true;
	if (surfaceBasedWidthCallback || surfaceBasedHeightCallback) return true;
	return false;
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
	colorAttachmentLayers.push_back(layer+1);
}

void HIGHOMEGA::GL::FramebufferClass::SetDepthStencil(ImageClass & inAttach)
{
	depthStencilAttachment = &inAttach;
	depthAttachmentLayer = 0;
}

void HIGHOMEGA::GL::FramebufferClass::SetDepthStencilWithLayer(ImageClass& inAttach, unsigned int layer)
{
	depthStencilAttachment = &inAttach;
	depthAttachmentLayer = layer+1;
}

ImageClass* HIGHOMEGA::GL::FramebufferClass::GetDepthStencil()
{
	return depthStencilAttachment;
}

bool HIGHOMEGA::GL::FramebufferClass::IsDepthStencilArrayed()
{
	return (depthAttachmentLayer > 0);
}

ImageClass & HIGHOMEGA::GL::FramebufferClass::GetSampler()
{
	return attachmentsSampler;
}

void HIGHOMEGA::GL::FramebufferClass::Create(RENDER_MODE inpMode, InstanceClass & inpInstance, WindowClass & windowRef, bool maintainAttachmentData)
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
		curAttach.loadOp = maintainAttachmentData ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_CLEAR;
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
		for (int i = 0;i != colorAttachments.size();i++)
		{
			VkAttachmentDescription curAttach;
			curAttach.format = (VkFormat)colorAttachments[i]->format;
			curAttach.samples = VK_SAMPLE_COUNT_1_BIT;
			curAttach.loadOp = maintainAttachmentData ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_CLEAR;
			curAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
			curAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
			curAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
			curAttach.initialLayout = maintainAttachmentData ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_UNDEFINED;
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
		depthStencilAttach.loadOp = maintainAttachmentData ? VK_ATTACHMENT_LOAD_OP_LOAD : VK_ATTACHMENT_LOAD_OP_CLEAR;
		depthStencilAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
		depthStencilAttach.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
		depthStencilAttach.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
		depthStencilAttach.initialLayout = mode == ON_SCREEN ? (depthStencilAttachment->haveSampler ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) : (maintainAttachmentData ? (depthStencilAttachment->haveSampler ? VK_IMAGE_LAYOUT_GENERAL : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL) : VK_IMAGE_LAYOUT_UNDEFINED);
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

	if (result != VK_SUCCESS) { FATAL_ERROR("Could not create a render pass"); }
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
			if (result != VK_SUCCESS) { RemovePast(i); FATAL_ERROR("Could not create a frame buffer"); }
		}
	}
	else
	{
		std::vector<VkImageView> attachmentsViews;
		for (int i = 0;i != colorAttachments.size();i++)
		{
			if (colorAttachmentLayers[i] == 0 )
				attachmentsViews.push_back(colorAttachments[i]->view);
			else
				attachmentsViews.push_back(colorAttachments[i]->layerView[colorAttachmentLayers[i] - 1]);
		}

		if (depthStencilAttachment)
			if (depthAttachmentLayer == 0)
				attachmentsViews.push_back(depthStencilAttachment->view);
			else
				attachmentsViews.push_back(depthStencilAttachment->layerView[depthAttachmentLayer - 1]);

		if (colorAttachments.size() > 0)
		{
			width = colorAttachments[0]->width;
			height = colorAttachments[0]->height;
		}
		else if (depthStencilAttachment)
		{
			width = depthStencilAttachment->width;
			height = depthStencilAttachment->height;
		}
		else if (surfaceBasedWidthCallback && surfaceBasedHeightCallback)
		{
			width = surfaceBasedWidthCallback(windowRef.w);
			height = surfaceBasedHeightCallback(windowRef.h);
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
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not create the framebuffer"); }
		haveAttachmentsFrameBuffer = true;

		try {
			attachmentsSampler.CreateSampler(ImageClass::MIN_MAG_FILTER::MIN_MAG_NEAREST, ImageClass::MIN_MAG_FILTER::MIN_MAG_NEAREST,
											 ImageClass::MIPMAP_MODE::MIPMAP_LINEAR,
											 ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE, ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE, ImageClass::TEXTURE_ADDRESS_MODE::CLAMP_TO_EDGE,
											 0.0f, 0.0f, 1.0f, false, 1.0f);
		}
		catch (...) {
			RemovePast(); FATAL_ERROR("Could not create the attachment sampler");
		}

		if (IsSwapchainDependent()) swapchainId = cachedInstance->getSwapchainId();
		else swapchainId = 0ull;
	}
}

bool HIGHOMEGA::GL::FramebufferClass::RecreateIfStale()
{
	if (swapchainId && cachedInstance && swapchainId != cachedInstance->getSwapchainId())
	{
		swapchainId = cachedInstance->getSwapchainId();
		RemovePast();
		Create(OFF_SCREEN, *cachedInstance, Window);
		return true;
	}
	return false;
}

void HIGHOMEGA::GL::FramebufferClass::setWidth(int width)
{
	this->width = width;
}

void HIGHOMEGA::GL::FramebufferClass::setHeight(int height)
{
	this->height = height;
}

void HIGHOMEGA::GL::FramebufferClass::setSwapchainDependentWidthCallback(const std::function<unsigned int(unsigned int)>& surfaceBasedWidthCallback)
{
	this->surfaceBasedWidthCallback = surfaceBasedWidthCallback;
}

void HIGHOMEGA::GL::FramebufferClass::setSwapchainDependentHeightCallback(const std::function<unsigned int(unsigned int)>& surfaceBasedHeightCallback)
{
	this->surfaceBasedHeightCallback = surfaceBasedHeightCallback;
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

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::RemoveAccelStruct()
{
	if (hasAccelStruct && ptrToInstance && ptrToInstance->SupportsHWRT()) RTInstance::fpDestroyAccelerationStructureKHR(ptrToInstance->device, accelStruct, nullptr);

	hasAccelStruct = false;
	reuseUpdateCmdBuffer = false;
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::CreateAccelStruct(bool isBlas, VkAccelerationStructureGeometryKHR * inpGeom, VkAccelerationStructureBuildRangeInfoKHR * inpGeomOffset, BufferClass* rtInstanceBuffer, unsigned int instanceCount, bool inpImmutable, InstanceClass & inpInstance, blasBuildParams *inpParams)
{
	ptrToInstance = &inpInstance;

	if (isBlas)
	{
		VkAccelerationStructureBuildGeometryInfoKHR accelerationStructureBuildGeomInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
		accelerationStructureBuildGeomInfo.flags = inpImmutable ? VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR : (VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR  | VK_BUILD_ACCELERATION_STRUCTURE_ALLOW_UPDATE_BIT_KHR);
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

		accelStructBuffer.Buffer(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY, *ptrToInstance, nullptr, (unsigned int)createInfo.size);
		createInfo.buffer = accelStructBuffer.buffer;

		VkResult result = RTInstance::fpCreateAccelerationStructureKHR(Instance.device, &createInfo, nullptr, &accelStruct);
		if (result != VK_SUCCESS) { FATAL_ERROR("Could not create accel struct"); }
		hasAccelStruct = true;
		reuseUpdateCmdBuffer = false;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_SSBO | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.pNext = VK_NULL_HANDLE;
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		inpParams->blasBuildRanges.push_back(inpGeomOffset);
		inpParams->blasBuildInfos.push_back(accelerationStructureBuildGeomInfo);
	}
	else
	{
		VkBufferDeviceAddressInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		bufferInfo.buffer = rtInstanceBuffer->buffer;
		VkDeviceAddress instanceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufferInfo);

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
		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
		RTInstance::fpGetAccelerationStructureBuildSizesKHR(Instance.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &accelerationStructureBuildGeomInfo, &instanceCount, &sizeInfo);

		VkAccelerationStructureCreateInfoKHR createInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
		createInfo.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
		createInfo.size = sizeInfo.accelerationStructureSize;

		accelStructBuffer.Buffer(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, nullptr, (unsigned int)createInfo.size);
		createInfo.buffer = accelStructBuffer.buffer;

		VkResult result = RTInstance::fpCreateAccelerationStructureKHR(Instance.device, &createInfo, nullptr, &accelStruct);
		if (result != VK_SUCCESS) { FATAL_ERROR("Could not create accel struct"); }
		hasAccelStruct = true;
		reuseUpdateCmdBuffer = false;
		accelerationStructureBuildGeomInfo.srcAccelerationStructure = VK_NULL_HANDLE;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_SSBO, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		BeginCommandBuffer(*ptrToInstance, 3u, 1u);

		VkAccelerationStructureBuildRangeInfoKHR buildOffsetInfo{ static_cast<uint32_t>(instanceCount), 0, 0, 0 };
		const VkAccelerationStructureBuildRangeInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;
		RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[1], 1, &accelerationStructureBuildGeomInfo, &pBuildOffsetInfo);

		WaitOnSemaphores(std::unordered_set<SemaphoreClass*>{&semaphore, &rtInstanceBuffer->semaphore}, 1);
		SignalSemaphores(std::unordered_set<SemaphoreClass*>{&semaphore}, 1);
		NoCPUSync(1);
		EndCommandBuffer(1);
		SubmitCommandBuffer(1);
	}
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::UpdateAccelStruct(bool isBlas, VkAccelerationStructureGeometryKHR * inpGeom, VkAccelerationStructureBuildRangeInfoKHR * inpGeomOffset, BufferClass* rtInstanceBuffer, unsigned int instanceCount, blasBuildParams *inpParams)
{
	if (!ptrToInstance) FATAL_ERROR("Cannot submit create accel struct request since a ptr to instance was not found");

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
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		inpParams->blasBuildRanges.push_back(inpGeomOffset);
		inpParams->blasBuildInfos.push_back(accelerationStructureBuildGeomInfo);
	}
	else
	{
		VkBufferDeviceAddressInfo bufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		bufferInfo.buffer = rtInstanceBuffer->buffer;
		VkDeviceAddress instanceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufferInfo);

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
		VkAccelerationStructureBuildSizesInfoKHR sizeInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
		RTInstance::fpGetAccelerationStructureBuildSizesKHR(Instance.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &accelerationStructureBuildGeomInfo, &instanceCount, &sizeInfo);

		accelerationStructureBuildGeomInfo.srcAccelerationStructure = accelStruct;
		accelerationStructureBuildGeomInfo.dstAccelerationStructure = accelStruct;

		if ((unsigned int)sizeInfo.buildScratchSize > scratchBuffer.getSize())
		{
			scratchBuffer.Buffer(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT, *ptrToInstance, nullptr, (unsigned int)sizeInfo.buildScratchSize);
		}
		VkBufferDeviceAddressInfo scratchBufferInfo{ VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO };
		scratchBufferInfo.buffer = scratchBuffer.buffer;
		VkDeviceAddress scratchAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &scratchBufferInfo);

		accelerationStructureBuildGeomInfo.scratchData.deviceAddress = scratchAddress;

		VkAccelerationStructureBuildRangeInfoKHR buildOffsetInfo{ static_cast<uint32_t>(instanceCount), 0, 0, 0 };
		const VkAccelerationStructureBuildRangeInfoKHR* pBuildOffsetInfo = &buildOffsetInfo;

		if (!reuseUpdateCmdBuffer)
		{
			BeginCommandBuffer(*ptrToInstance, 3u, 2u);
			RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[2], 1, &accelerationStructureBuildGeomInfo, &pBuildOffsetInfo);
			EndCommandBuffer(2);
			reuseUpdateCmdBuffer = true;
		}

		WaitOnSemaphores(std::unordered_set<SemaphoreClass*>{&semaphore, &rtInstanceBuffer->semaphore}, 2);
		SignalSemaphores(std::unordered_set<SemaphoreClass*>{&semaphore}, 2);
		NoCPUSync(2);
		SubmitCommandBuffer(2);
	}
}

void HIGHOMEGA::GL::KHR_RT::RTAccelStruct::FetchBlasAddress()
{
	if (!ptrToInstance) FATAL_ERROR("Cannot fetch blas address since a ptr to instance was not found");

	VkAccelerationStructureDeviceAddressInfoKHR addressInfo{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
	addressInfo.accelerationStructure = accelStruct;
	blasAddress = RTInstance::fpGetAccelerationStructureDeviceAddressKHR(ptrToInstance->device, &addressInfo);
}

HIGHOMEGA::GL::KHR_RT::RTAccelStruct::RTAccelStruct()
{
	hasAccelStruct = false;
	reuseUpdateCmdBuffer = false;
	ptrToInstance = nullptr;
}

HIGHOMEGA::GL::KHR_RT::RTAccelStruct::~RTAccelStruct()
{
	RemoveAccelStruct();
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::RemoveRTGeom()
{
	RemoveAccelStruct();
	created = false;
	dirty = false;
}

HIGHOMEGA::GL::KHR_RT::RTGeometry::RTGeometry()
{
	created = false;
	dirty = false;
}

HIGHOMEGA::GL::KHR_RT::RTGeometry::~RTGeometry()
{
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::SetGeom(BufferClass & vertBuffer, VkDeviceAddress giantVertBufferAddress, unsigned int triCount, unsigned int vertCount, unsigned int vertDataOffset, unsigned int indexDataOffset, unsigned int vertexSize, bool isAlphaKeyed, bool inpImmutable, InstanceClass & inpInstance)
{
	if (vertBuffer.getSize() == 0) return;

	immutable = inpImmutable;

	ptrToInstance = &inpInstance;

	traceGeom = {};
	traceGeom.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	traceGeom.pNext = nullptr;
	traceGeom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
	traceGeom.geometry.triangles.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
	traceGeom.geometry.triangles.pNext = nullptr;
	traceGeom.geometry.triangles.vertexFormat = (VkFormat)R32G32B32F;
	traceGeom.geometry.triangles.vertexData.deviceAddress = (VkDeviceSize)(giantVertBufferAddress + vertDataOffset * vertexSize);
	traceGeom.geometry.triangles.vertexStride = vertexSize;
	traceGeom.geometry.triangles.maxVertex = vertCount - 1;
	traceGeom.geometry.triangles.indexData.deviceAddress = (VkDeviceSize)(giantVertBufferAddress + indexDataOffset * sizeof(unsigned int));
	traceGeom.geometry.triangles.indexType = VK_INDEX_TYPE_UINT32;
	traceGeom.geometry.triangles.transformData.deviceAddress = (VkDeviceAddress)0;
	if (!isAlphaKeyed) traceGeom.flags = VK_GEOMETRY_OPAQUE_BIT_KHR;

	traceGeomOffset = {};
	traceGeomOffset.firstVertex = 0;
	traceGeomOffset.primitiveCount = triCount;
	traceGeomOffset.primitiveOffset = 0;
	traceGeomOffset.transformOffset = 0;

	dirty = true;
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::CreateOrUpdate(blasBuildParams *inpParams, unsigned long long* updateHash)
{
	if (!created)
	{
		CreateAccelStruct(true, &traceGeom, &traceGeomOffset, nullptr, 0u, immutable, *ptrToInstance, inpParams);
		created = true;
		dirty = false;
	}
	if (dirty && !immutable)
	{
		UpdateAccelStruct(true, &traceGeom, &traceGeomOffset, nullptr, 0u, inpParams);
		if (updateHash) *updateHash ^= (unsigned long long)traceGeom.geometry.triangles.vertexData.deviceAddress;
		dirty = false;
	}
}

void HIGHOMEGA::GL::KHR_RT::RTGeometry::SetDirty()
{
	if (immutable) return;
	// This should be called when geometry changes inside a shader...
	dirty = true;
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
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not create descriptor set layout"); }

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
	descriptorSets.clear();

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
	if (!ptrToDescSetLayout) FATAL_ERROR("Attempting to write descriptor sets without a desc set layout");

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
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not create descriptor pool"); }

		descriptorPools.dir[ThreadID].elemCount = 0; // Increase will happen after success below!
		descriptorPools.dir[ThreadID].keyRef = ThreadID;
	}
	descriptorPoolPtr = &descriptorPools.dir[ThreadID];
	poolObject = descriptorPoolPtr->elem.back();}

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
	allocInfo.descriptorPool = poolObject;
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
		if (result != VK_SUCCESS) { RemovePast(); FATAL_ERROR("Could not create descriptor pool"); }

		descriptorPoolPtr = &descriptorPools.dir[ThreadID];
		poolObject = descriptorPoolPtr->elem.back();}

		allocInfo.descriptorPool = poolObject;

		result = vkAllocateDescriptorSets(ptrToInstance->device, &allocInfo, descriptorSets.data());

		if (result != VK_SUCCESS) { descriptorSets.clear(); RemovePast(); FATAL_ERROR("Could not allocate descriptor sets"); }
	}

	descriptorPoolPtr->elemCount++;

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
			ImageClass* imageToBind = curRes.imageViewRef ? curRes.imageViewRef : curRes.samplerRef;
			descriptorImageInfo.imageView = curRes.imageLayer > 0 ? imageToBind->layerView[curRes.imageLayer - 1] : imageToBind->view;

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
					ImageClass* imageToBind = curResInArray.imageViewRef ? curResInArray.imageViewRef : curResInArray.samplerRef;
					descriptorImageInfo.imageView = curResInArray.imageLayer > 0 ? imageToBind->layerView[curResInArray.imageLayer - 1] : imageToBind->view;

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
	if (!ptrToInstance || !descriptorPoolPtr) FATAL_ERROR("Attempting to write descriptor sets without a ref. to the Vulkan instance or having a descriptor pool");

	RemovePast();

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
		if (myResult != VK_SUCCESS) FATAL_ERROR("Create descriptor update template failed.");
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
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create pipeline layout"); }
	havePipelineLayout = true;

	std::string raygenKey, raymissKey, rchitKey;
	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_raygen_shader, inpShader.getRaygenEntry(), VK_SHADER_STAGE_RAYGEN_BIT_KHR, inpShader.getStageSpecializationDataRef(RT_RAYGEN), raygenKey)->elem.stage); }
	catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create raygen shader"); }
	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_raymiss_shader, inpShader.getRaymissEntry(), VK_SHADER_STAGE_MISS_BIT_KHR, inpShader.getStageSpecializationDataRef(RT_MISS), raymissKey)->elem.stage); }
	catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create raymiss shader"); }
	try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_raychit_shader, inpShader.getRaychitEntry(), VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR, inpShader.getStageSpecializationDataRef(RT_RCHIT), rchitKey)->elem.stage); }
	catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create raychit shader"); }
	usedShaderStages.push_back(raygenKey);
	usedShaderStages.push_back(raymissKey);
	usedShaderStages.push_back(rchitKey);
	if (inpShader.getRayahitEntry())
	{
		std::string rahitKey;
		try { shaderStages.push_back(AddOrFindCachedShaderStage(*ptrToInstance, inpShader.rt_rayahit_shader, inpShader.getRayahitEntry(), VK_SHADER_STAGE_ANY_HIT_BIT_KHR, inpShader.getStageSpecializationDataRef(RT_ANYHIT), rahitKey)->elem.stage); }
		catch (...) { ErasePipelineState(); FATAL_ERROR("Could not create rayahit shader"); }
		usedShaderStages.push_back(rahitKey);
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
	if (result != VK_SUCCESS) { FATAL_ERROR("Could not create a pipeline cache"); }
	havePipelineCache = true;

	result = RTInstance::fpCreateRayTracingPipelinesKHR(ptrToInstance->device, VK_NULL_HANDLE, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline);
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not create rt pipeline"); }
	haveRTPipeline = true;

	unsigned int sbtChunkSize = (RTInstance::raytracingPipelineProperties.shaderGroupHandleSize + (RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment - 1)) & (~(RTInstance::raytracingPipelineProperties.shaderGroupBaseAlignment - 1));

	uint32_t shaderBindingTableSize = RTInstance::raytracingPipelineProperties.shaderGroupHandleSize * (uint32_t)shaderGroups.size();
	uint32_t shaderBindingTableSizeAligned = sbtChunkSize * (uint32_t)shaderGroups.size();

	shaderBindingTable.Buffer(MEMORY_HOST_VISIBLE | MEMORY_HOST_COHERENT, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SRC | USAGE_SBT | USAGE_DEVICE_ADDRESS, Instance, nullptr, (unsigned int)shaderBindingTableSizeAligned);
	unsigned char *sbtData = new unsigned char[shaderBindingTableSize];
	unsigned char *sbtDataAligned = new unsigned char[shaderBindingTableSizeAligned];

	result = RTInstance::fpGetRayTracingShaderGroupHandlesKHR(ptrToInstance->device, pipeline, 0, (uint32_t)shaderGroups.size(), shaderBindingTableSize, sbtData);
	if (result != VK_SUCCESS) { ErasePipelineState(); FATAL_ERROR("Could not get shader group handle"); }
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
	sceneId = previousUpdateHash = 0ull;
}

HIGHOMEGA::GL::KHR_RT::RTScene::~RTScene()
{
	sceneId = previousUpdateHash = 0ull;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::MarkSceneDirty()
{
	needReCreation = true;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::MarkSceneNeedingUpdate()
{
	needUpdate = true;
}

unsigned int HIGHOMEGA::GL::KHR_RT::RTScene::GetTraceItemCount()
{
	unsigned int traceItemCount = 0;
	for (std::pair <const unsigned long long, std::vector<TraceItem>>& traceItemKV : allTraceItems)
		traceItemCount += (unsigned int)traceItemKV.second.size();

	return traceItemCount;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::Add(unsigned long long inpId, unsigned int instanceId, GeometryClass& inpGeom, mat4 theMat, InstanceClass & inpInstance)
{
	if (!ptrToInstance) ptrToInstance = &inpInstance;

	TraceItem trItem;
	trItem.geomRef = &inpGeom;

	VkAccelerationStructureInstanceKHR curInst;
	for (int i = 0; i != 3; i++)
		for (int j = 0; j != 4; j++)
			curInst.transform.matrix[i][j] = (i == j) ? 1.0f : 0.0f;

	curInst.instanceCustomIndex = instanceId;
	curInst.mask = 0xff;
	curInst.instanceShaderBindingTableRecordOffset = 0;
	curInst.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;

	trItem.theMat = theMat;
	trItem.rtInstanceData = curInst;

	allTraceItems[inpId].push_back(trItem);
	needReCreation = true;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::Remove(GeometryClass& inpGeom)
{
	for (std::pair<const unsigned long long, std::vector<TraceItem>>& curTraceItem : allTraceItems)
	{
		std::vector<TraceItem>::iterator it = std::find_if(allTraceItems[curTraceItem.first].begin(), allTraceItems[curTraceItem.first].end(), [&inpGeom](const TraceItem& trItem) -> bool {
			return (&inpGeom == trItem.geomRef);
			});
		if (it != allTraceItems[curTraceItem.first].end())
		{
			allTraceItems[curTraceItem.first].erase(it);
			needReCreation = true;
		}
	}
}

void HIGHOMEGA::GL::KHR_RT::RTScene::RemoveAll(unsigned long long inpId)
{
	allTraceItems.erase(inpId);
	needReCreation = true;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::DeleteRTResources()
{
	for (std::pair<const unsigned long long, std::vector<TraceItem>>& curTraceItems : allTraceItems)
		for (TraceItem& curTraceItem : curTraceItems.second)
			curTraceItem.geomRef->rtGeom.RemoveRTGeom();

	RemoveAccelStruct();
	sceneId = previousUpdateHash = 0ull;
	needReCreation = true;
}

void HIGHOMEGA::GL::KHR_RT::RTScene::CreateInstanceData(std::vector <VkAccelerationStructureInstanceKHR> & instances)
{
	instances.clear();
	instances.reserve(GetTraceItemCount());

	for (std::pair<const unsigned long long, std::vector<TraceItem>> & traceItemKV : allTraceItems)
		for (TraceItem & curTraceItem : traceItemKV.second)
		{
			curTraceItem.rtInstanceData.accelerationStructureReference = curTraceItem.geomRef->getRTGeom().blasAddress;
			instances.push_back(curTraceItem.rtInstanceData);
		}
}

unsigned long long HIGHOMEGA::GL::KHR_RT::RTScene::rtSceneID(std::function<void(unsigned int, BufferClass*, BufferClass*, bool)>& rtCopyTransforms)
{
	unsigned long long updateHash = 0ull;
	blasBuildParams blBuildParams;
	std::vector<RTGeometry *> rebuildGeoms;
	unsigned int rebuildInstCount = GetTraceItemCount();
	rebuildGeoms.reserve(rebuildInstCount);

	VkDeviceAddress bufferDeviceAddress = 0ull;

	for (std::pair <const unsigned long long, std::vector<TraceItem>> & traceItemKV : allTraceItems)
		for (TraceItem & curTraceItem : traceItemKV.second)
			if (curTraceItem.geomRef->getRTGeom().dirty || !curTraceItem.geomRef->getRTGeom().created)
			{
				if (!curTraceItem.geomRef->getRTGeom().created)
				{
					if (!bufferDeviceAddress)
					{
						VkBufferDeviceAddressInfo bufDevAdInfo;
						bufDevAdInfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
						bufDevAdInfo.pNext = VK_NULL_HANDLE;
						giantVertBufferSharedMutex.lock_shared();
						bufDevAdInfo.buffer = giantVertBuffer->buffer;
						giantVertBufferSharedMutex.unlock_shared();
						bufferDeviceAddress = RTInstance::fpGetBufferDeviceAddressKHR(Instance.device, &bufDevAdInfo);
					}

					curTraceItem.geomRef->rtGeom.SetGeom(curTraceItem.geomRef->vertBuffer, bufferDeviceAddress, curTraceItem.geomRef->triCount, curTraceItem.geomRef->vertCount,
														 curTraceItem.geomRef->getVertexOffsetInGiantVertexBuffer(), curTraceItem.geomRef->getIndexOffsetInGiantVertexBuffer(),
														 sizeof(RasterVertex), curTraceItem.geomRef->isAlphaKeyedCache, curTraceItem.geomRef->immutable, *ptrToInstance);
					needReCreation = true;
					rebuildGeoms.push_back(&curTraceItem.geomRef->getRTGeom());
				}
				needUpdate = true;
				curTraceItem.geomRef->getRTGeom().CreateOrUpdate(&blBuildParams, &updateHash);
			}

	if (blBuildParams.blasBuildInfos.size() > 0)
	{
		if (!semaphore.haveSemaphore)
			semaphore.Semaphore(ptrToInstance);

		if (previousUpdateHash != updateHash || rebuildGeoms.size())
		{
			BeginCommandBuffer(*ptrToInstance, 3u, 0u);

			RTInstance::fpCmdBuildAccelerationStructuresKHR(cmdBuffers[0], (unsigned int)blBuildParams.blasBuildInfos.size(),
															(const VkAccelerationStructureBuildGeometryInfoKHR *)blBuildParams.blasBuildInfos.data(),
															(const VkAccelerationStructureBuildRangeInfoKHR * const *)blBuildParams.blasBuildRanges.data());
			EndCommandBuffer(0);
			previousUpdateHash = updateHash;
		}

		WaitOnSemaphores(std::unordered_set<SemaphoreClass*>{&semaphore}, 0);
		SignalSemaphores(std::unordered_set<SemaphoreClass*>{&semaphore}, 0);
		if (rebuildGeoms.size()) DoCPUSync(0);
		else NoCPUSync(0);
		SubmitCommandBuffer(0);
	}

	for (RTGeometry * curGeom : rebuildGeoms)
		curGeom->FetchBlasAddress();

	if (needReCreation)
	{
		if (sceneId != 0ull) RTAccelStruct::RemoveAccelStruct();
		sceneId = threadSafeMersenneTwister64Bit();

		CreateInstanceData(instances);

		bool descUpdated = false;
		if (rtInstanceBuffer.getSize() < instances.size() * sizeof(VkAccelerationStructureInstanceKHR))
		{
			rtInstanceBuffer.Buffer(MEMORY_HOST_VISIBLE | MEMORY_HOST_COHERENT, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DEVICE_ADDRESS | USAGE_ACCEL_STRUCT | USAGE_SSBO | USAGE_ACCEL_STRUCT_BUILDER_READ_ONLY, Instance, instances.data(), (unsigned int)(instances.size() * sizeof(VkAccelerationStructureInstanceKHR)));
			descUpdated = true;
		}
		else
			if (instances.size() > 0) rtInstanceBuffer.UploadSubData(0, instances.data(), (unsigned int)(instances.size() * sizeof(VkAccelerationStructureInstanceKHR)));

		if (rtCopyTransformParamsSSBO.getSize() == 0)
		{
			rtCopyTransformParamsSSBO.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, &rebuildInstCount, (unsigned int)sizeof(unsigned int));
			descUpdated = true;
		}
		else
			rtCopyTransformParamsSSBO.UploadSubData(0, &rebuildInstCount, sizeof(unsigned int));

		rtCopyTransforms(rebuildInstCount, &rtInstanceBuffer, &rtCopyTransformParamsSSBO, descUpdated);

		try { CreateAccelStruct(false, nullptr, nullptr, &rtInstanceBuffer, rebuildInstCount, false, *ptrToInstance, &blBuildParams); }
		catch (...) { throw; }

		needReCreation = false;
		needUpdate = false;
	}

	if (needUpdate)
	{
		rtCopyTransforms(rebuildInstCount, &rtInstanceBuffer, &rtCopyTransformParamsSSBO, false);

		UpdateAccelStruct(false, nullptr, nullptr, &rtInstanceBuffer, rebuildInstCount, &blBuildParams);
		needUpdate = false;
	}

	return sceneId;
}

HIGHOMEGA::GL::KHR_RT::RTTracelet::RTTracelet()
{
	ptrToInstance = nullptr;
}

void HIGHOMEGA::GL::KHR_RT::RTTracelet::Make(InstanceClass & inpInstance)
{
	ptrToInstance = &inpInstance;
}

RTTracelet& HIGHOMEGA::GL::KHR_RT::RTTracelet::MakeAsync()
{
	doCPUSync = false;
	return *this;
}

void HIGHOMEGA::GL::KHR_RT::RTTracelet::Submit(unsigned int inpWidth, unsigned int inpHeight, unsigned int inpDepth, std::vector<ShaderResource> & inpTracingResources, bool updateResources, ShaderResourceSet & inpRTShaderResourceSet)
{
	if (!ptrToInstance) return;

	if (swapchainId && swapchainId != ptrToInstance->getSwapchainId()) updateResources = true;

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

		swapchainId = 0ull;
		for (ShaderResource& curRes : inpTracingResources)
			if (curRes.IsSwapchainDependent())
			{
				swapchainId = ptrToInstance->getSwapchainId();
				break;
			}

		waitSems.clear();
		waitOldSems.clear();
		signalSems.clear();
		for (ShaderResource& curRes : inpTracingResources)
		{
			if (curRes.IsSimultaneouslyProduced()) waitOldSems.merge(curRes.GetAsDependencies());
			else waitSems.merge(curRes.GetAsDependencies());
			if (curRes.IsProduced()) signalSems.merge(curRes.GetAsDependencies());
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

	WaitOnSemaphores(waitSems);
	WaitOnOldSemaphores(waitOldSems);
	SignalSemaphores(signalSems);
	if (!doCPUSync) NoCPUSync();
	SubmitCommandBuffer();
}