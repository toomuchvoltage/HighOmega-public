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

#include <world.h>

using namespace HIGHOMEGA::FIZ_X;
using namespace HIGHOMEGA::GEOM;
using namespace HIGHOMEGA::ENTITIES;
using namespace HIGHOMEGA::RENDER::PASSES;
using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;

std::thread *HIGHOMEGA::WORLD::PhysicsThread = nullptr;
std::thread *HIGHOMEGA::WORLD::EntitiesThread = nullptr;
std::thread *HIGHOMEGA::WORLD::AudioThread = nullptr;

extern HIGHOMEGA::INSTRUMENTATION::Instrument updateInstrument, traceInstrument, frameInstrument;
HIGHOMEGA::WORLD::minMaxReductionClass HIGHOMEGA::WORLD::mainMinMaxReducer;
std::unordered_map <std::string, HIGHOMEGA::WORLD::PhysicalItemClass::Destructible> HIGHOMEGA::WORLD::PhysicalItemClass::destructibles;
std::unordered_map <std::string, HIGHOMEGA::WORLD::PhysicalItemClass::DestructibleHitInfo> HIGHOMEGA::WORLD::PhysicalItemClass::destructibleHitInfo;
std::mutex HIGHOMEGA::WORLD::PhysicalItemClass::destructibleCacheMutex;

vec3 HIGHOMEGA::WORLD::FindCenter(Mesh & inpMesh, std::function<bool(RigidBody *, int, DataGroup &)> inpFilterFunction)
{
	vec3 newCenter(0.0f);
	int newCenterCount = 0;

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];
		if (!inpFilterFunction(nullptr, i, curPolyGroup)) continue;

		DataBlock *triBlock;
		if ( !Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock)) continue;

		RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
		getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

		vec3 edge, vnorm, vcol;
		vec2 uv;
		for (int j = 0; j != vertCount; j++)
		{
			unpackRasterVertex (edge, vcol, uv, vnorm, verts[j]);
			newCenter += edge;
			newCenterCount++;
		}
	}
	return newCenter / (float)newCenterCount;
}

HIGHOMEGA::WORLD::ZoneStreamingClass HIGHOMEGA::WORLD::zoneStreaming;

HIGHOMEGA::WORLD::PlasteredItemsClass HIGHOMEGA::WORLD::plasteredItemsCollection;
std::unordered_map <std::string, HIGHOMEGA::WORLD::MeshModel> HIGHOMEGA::WORLD::PlasteredItemsClass::cachedMeshesModels;
std::mutex HIGHOMEGA::WORLD::PlasteredItemsClass::cachedMeshesModelsMutex;

HIGHOMEGA::WORLD::PhysicalItemClass HIGHOMEGA::WORLD::physicalItemsCollection;
std::unordered_map <std::string, HIGHOMEGA::WORLD::MeshModel> HIGHOMEGA::WORLD::PhysicalItemClass::cachedMeshesModels;
std::mutex HIGHOMEGA::WORLD::PhysicalItemClass::cachedMeshesModelsMutex;

std::unordered_map <std::string, HIGHOMEGA::WORLD::MeshModel> HIGHOMEGA::WORLD::AnimatedMeshesClass::cachedMeshesModels;
std::mutex HIGHOMEGA::WORLD::AnimatedMeshesClass::cachedMeshesModelsMutex;

HIGHOMEGA::WORLD::ParticleSystemClass HIGHOMEGA::WORLD::particleSystem;
std::unordered_map <std::string, HIGHOMEGA::WORLD::MeshModel> HIGHOMEGA::WORLD::ParticleSystemClass::cachedMeshesModels;
std::mutex HIGHOMEGA::WORLD::ParticleSystemClass::cachedMeshesModelsMutex;

HIGHOMEGA::WORLD::GuidedModelSystemClass HIGHOMEGA::WORLD::guidedModelSystem;
std::unordered_map <std::string, HIGHOMEGA::WORLD::MeshModel> HIGHOMEGA::WORLD::GuidedModelSystemClass::cachedMeshesModels;
std::mutex HIGHOMEGA::WORLD::GuidedModelSystemClass::cachedMeshesModelsMutex;

HIGHOMEGA::WORLD::CameraSystemClass HIGHOMEGA::WORLD::cameraSystem;
HIGHOMEGA::RENDER::WorldParamsClass HIGHOMEGA::WORLD::worldParams;

unsigned int HIGHOMEGA::WORLD::minMaxReductionClass::WorkGroupMinMaxX()
{
	return 64;
}

unsigned long long HIGHOMEGA::WORLD::minMaxReductionClass::RequestMinMax(GeometryClass * inGeom)
{
	unsigned long long requestId = threadSafeMersenneTwister64Bit();

	geomInfo[requestId].geom = inGeom;
	geomInfo[requestId].orientCache = mat4();
	geomInfo[requestId].posCache = vec3(0.0f);

	modified = true;

	return requestId;
}

void HIGHOMEGA::WORLD::minMaxReductionClass::RemoveMinMaxRequest(unsigned long long requestId)
{
	geomInfo.erase(requestId);

	modified = true;
}

void HIGHOMEGA::WORLD::minMaxReductionClass::Process()
{
	if (geomInfo.size() == 0) return;
	if (!minMaxShader) minMaxShader = new ShaderResourceSet;
	if (!geomParamsBuf) geomParamsBuf = new BufferClass;
	if (!minMaxSubmission) minMaxSubmission = new ComputeSubmission;

	if (modified)
	{
		if (init)
		{
			delete minMaxShader;
			minMaxShader = new ShaderResourceSet;
			init = false;
		}

		if (geomInfo.size() > 0)
		{
			unsigned int itemCountAligned = (((unsigned int)geomInfo.size() / 1000) + 1) * 1000;
			unsigned int paramAlignmentSize = itemCountAligned * sizeof(minMaxParamsStruct);
			if (paramAlignmentSize > geomParamsBuf->getSize())
			{
				delete geomParamsBuf;
				geomParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, paramAlignmentSize);
				geomIds.reserve(itemCountAligned);
				geomParams.reserve(itemCountAligned);
			}
		}
		else
		{
			if (geomParamsBuf->getSize() == 0) geomParamsBuf->Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, sizeof(minMaxParamsStruct));
		}

		geomIds.clear();
		geomParams.clear();

		for (std::pair<const unsigned long long, geomInfoStruct> & curGeomInfo : geomInfo)
		{
			geomIds.push_back(curGeomInfo.first);
			geomParams.emplace_back();
			unsigned int idxVertOffset = curGeomInfo.second.geom->getDataOffsetInGiantVertexBuffer();
			unsigned int numVerts = curGeomInfo.second.geom->getVertCount();
			geomParams.back().minValueIdxVertOffset[3] = *((float*)&idxVertOffset);
			geomParams.back().maxValueNumVerts[3] = *((float *)&numVerts);
		}
		giantVertBufferSharedMutex.lock_shared();
		minMaxShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *giantVertBuffer);
		giantVertBufferSharedMutex.unlock_shared();
		geomParamsBuf->UploadSubData(0, geomParams.data(), (unsigned int)geomParams.size() * sizeof(minMaxParamsStruct));
		minMaxShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, 0, *geomParamsBuf);
		minMaxShader->Create("shaders/minMaxGeom.comp.spv", "main");
		if (init) minMaxSubmission->RemoveDispatch(Instance, std::string("default"));
		minMaxSubmission->MakeDispatch(Instance, std::string ("default"), *minMaxShader, 1, (unsigned int)geomParams.size(), 1);
		init = true;
		modified = false;
	}
	(*minMaxSubmission).Submit();

	geomParamsBuf->DownloadSubData(0, geomParams.data(), (unsigned int)geomParams.size() * sizeof(minMaxParamsStruct));
	for (unsigned int i = 0; i != geomParams.size(); i++)
	{
		vec3 minEval = vec3(geomParams[i].minValueIdxVertOffset[0], geomParams[i].minValueIdxVertOffset[1], geomParams[i].minValueIdxVertOffset[2]);
		vec3 maxEval = vec3(geomParams[i].maxValueNumVerts[0], geomParams[i].maxValueNumVerts[1], geomParams[i].maxValueNumVerts[2]);
		geomInfoStruct & curGeomInfo = geomInfo[geomIds[i]];
		curGeomInfo.geom->SetMinMax(minEval, maxEval);
	}

	geomInfo.clear(); // Should not be a running task... these are usually expensive
}

void HIGHOMEGA::WORLD::minMaxReductionClass::ClearContent()
{
	geomInfo.clear();
	if (geomParamsBuf) delete geomParamsBuf;
	if (minMaxShader) delete minMaxShader;
	if (minMaxSubmission) delete minMaxSubmission;
	geomParamsBuf = nullptr;
	minMaxShader = nullptr;
	minMaxSubmission = nullptr;

	init = false;
}

HIGHOMEGA::WORLD::MeshModel* HIGHOMEGA::WORLD::ThreadSafeMeshModelCache::InsertEntry(std::function<void(Mesh& outMesh, GraphicsModel** outModel)> buildEntryFunction, std::string& insertionKey, std::string&& owningSystem, std::unordered_map<std::string, MeshModel>& cachedMeshesModels, std::mutex& cachedMeshesModelsMutex)
{
	MeshModel* retMeshModel = nullptr;
	std::unique_lock<std::mutex> lk(cachedMeshesModelsMutex, std::defer_lock);
	lk.lock();
	if (cachedMeshesModels.find(insertionKey) == cachedMeshesModels.end())
	{
		cachedMeshesModels[insertionKey].preparingOnAnotherThread = true;
		lk.unlock();

		Mesh tmpMesh;
		GraphicsModel* tmpModel = nullptr;
		try
		{
			buildEntryFunction(tmpMesh, &tmpModel);
		}
		catch (...)
		{
			lk.lock();
			cachedMeshesModels.erase(insertionKey);
			lk.unlock();
			throw;
		}

		lk.lock();
		cachedMeshesModels[insertionKey].mesh = std::move(tmpMesh);
		cachedMeshesModels[insertionKey].model = tmpModel;
		cachedMeshesModels[insertionKey].preparingOnAnotherThread = false;
		cachedMeshesModels[insertionKey].claims = 1;
	}
	else
	{
		if (cachedMeshesModels[insertionKey].preparingOnAnotherThread)
		{
			lk.unlock();
			while (true)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				lk.lock();
				if (cachedMeshesModels.find(insertionKey) == cachedMeshesModels.end())
				{
					lk.unlock();
					FATAL_ERROR("Another thread failed to load cached mesh+model for " + owningSystem);
				}
				else
				{
					if (!cachedMeshesModels[insertionKey].preparingOnAnotherThread)
					{
						cachedMeshesModels[insertionKey].claims++;
						break;
					}
				}
				lk.unlock();
			}
		}
		else
			cachedMeshesModels[insertionKey].claims++;
	}
	retMeshModel = &cachedMeshesModels[insertionKey];
	lk.unlock();
	return retMeshModel;
}

void HIGHOMEGA::WORLD::ThreadSafeMeshModelCache::CleanupCache(std::unordered_map<std::string, MeshModel>& cachedMeshesModels, std::mutex& cachedMeshesModelsMutex)
{
	std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex);
	while (true)
	{
		bool deletedSomething = false;
		for (std::pair<const std::string, HIGHOMEGA::WORLD::MeshModel>& curMeshModelEntry : cachedMeshesModels)
		{
			if (curMeshModelEntry.second.claims == 0u)
			{
				delete curMeshModelEntry.second.model;
				cachedMeshesModels.erase(curMeshModelEntry.first);
				deletedSomething = true;
				break;
			}
		}
		if (!deletedSomething) break;
	}
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::Combine(std::vector <PlasteredItemsClass> & plasteredClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall)
{
	if (!integrateSubmission) integrateSubmission = new ComputeSubmission;
	if (!transformCollectionSubmission) transformCollectionSubmission = new ComputeSubmission;

	submissionsForPlasteredItems = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	for (PlasteredItemsClass & curPlasteredClass : plasteredClasses)
	{
		if (curPlasteredClass.integrateSubmission) delete curPlasteredClass.integrateSubmission;
		if (curPlasteredClass.integrateShader) delete curPlasteredClass.integrateShader;
		if (curPlasteredClass.transformCollectionSubmission) delete curPlasteredClass.transformCollectionSubmission;
		if (curPlasteredClass.transformCollectionShader) delete curPlasteredClass.transformCollectionShader;
		curPlasteredClass.integrateSubmission = nullptr;
		curPlasteredClass.integrateShader = nullptr;
		curPlasteredClass.transformCollectionSubmission = nullptr;
		curPlasteredClass.transformCollectionShader = nullptr;

		for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionsIdPair : curPlasteredClass.allCollections)
			allCollections[curCollectionsIdPair.first] = curCollectionsIdPair.second;
		curPlasteredClass.allCollections.clear();
		curPlasteredClass.curTimeElapsed = 0.0f;
	}

	rerecordSubmission = true;
}

unsigned long long HIGHOMEGA::WORLD::PlasteredItemsClass::Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*>&& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducer)
{
	submissionsForPlasteredItems = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	unsigned long long curPlasterId = threadSafeMersenneTwister64Bit();

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		DataBlock *triBlock;
		if ( !Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock)) continue;

		for (int j = 1;; j++)
		{
			std::string tmpStr;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "plasterMesh" + std::to_string(j), tmpStr)) break;

			plasterCollection curCollection;

			float plasterSeed;
			if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterSeed" + std::to_string(j), plasterSeed))
			{
				srand((unsigned int)plasterSeed);
			}
			float plasterSizeMin, plasterSizeMax, meshCountFloat;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterSizeMin" + std::to_string(j), plasterSizeMin)) plasterSizeMin = 2.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterSizeMax" + std::to_string(j), plasterSizeMax)) plasterSizeMax = 3.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterItemsWavePhase" + std::to_string(j), curCollection.wavePhase)) curCollection.wavePhase = 1.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterItemsWaveAmplitude" + std::to_string(j), curCollection.waveAmplitude)) curCollection.waveAmplitude = 1.0f;

			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterMeshCount" + std::to_string(j), meshCountFloat)) FATAL_ERROR("Could not get plaster mesh count");
			unsigned int meshCount = (unsigned int)meshCountFloat;

			std::string plasterMesh, plasterMeshAssetLoc;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "plasterMesh" + std::to_string(j), plasterMesh) ||
				 !Mesh::getDataRowString(curPolyGroup, "PROPS", "plasterMeshAssetLoc" + std::to_string(j), plasterMeshAssetLoc) ) FATAL_ERROR("Could not get plastered mesh");

			MeshModel* retMeshModel = InsertEntry([&plasterMesh, &plasterMeshAssetLoc](Mesh& outMesh, GraphicsModel** outModel) {
				outMesh = std::move(Mesh(plasterMesh));
				*outModel = new GraphicsModel(outMesh, plasterMeshAssetLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
					float tmpFloat;
					return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
				}, true);
			}, plasterMesh, std::string("plastering meshes"), cachedMeshesModels, cachedMeshesModelsMutex);

			curCollection.meshModelKey = plasterMesh;

			curCollection.collectionModel = new GraphicsModel(retMeshModel->mesh, plasterMeshAssetLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
				float tmpFloat;
				return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
			}, false);
			curCollection.collectionModel->BlankAndResize(meshCount);

			std::vector<transformSourceData> transformSourceMats;
			transformSourceMats.reserve(meshCount);
			std::vector<transformData> transformMats;
			transformMats.reserve(meshCount);

			RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
			getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

			for (int k = 0; k != meshCount; k++)
			{
				unsigned int pickPrim = (unsigned int)(((float)rand() / (float)RAND_MAX) * triCount);
				if (pickPrim == triCount) pickPrim--;

				vec3 v1, v2, v3;
				vec2 uv;
				vec3 vnorm, vcol;
				vec3 side1, side2, side3, faceNormal;

				unpackRasterVertex(v1, vcol, uv, vnorm, verts[indices[pickPrim * 3]]);
				unpackRasterVertex(v2, vcol, uv, vnorm, verts[indices[pickPrim * 3 + 1]]);
				unpackRasterVertex(v3, vcol, uv, vnorm, verts[indices[pickPrim * 3 + 2]]);

				vec3 v1_v2 = v1 - v2;
				vec3 v2_v3 = v2 - v3;
				vec3 v3_v1 = v3 - v1;

				faceNormal = cross(v1_v2, v2_v3).normalized();
				if (faceNormal * vnorm < 0.0f) faceNormal = -faceNormal;

				side1 = cross(v1_v2, faceNormal).normalized();
				if (side1 * v3_v1 > 0.0f) side1 = -side1;
				side2 = cross(v2_v3, faceNormal).normalized();
				if (side2 * v1_v2 > 0.0f) side2 = -side2;
				side3 = cross(v3_v1, faceNormal).normalized();
				if (side3 * v2_v3 > 0.0f) side3 = -side3;

				vec3 axis1 = cross(faceNormal, faceNormal + vec3(0.1f)).normalized();
				vec3 axis2 = cross(axis1, faceNormal);

				vec3 randPoint;
				while (true)
				{
					float uv1 = ((float)rand() / (float)RAND_MAX);
					float uv2 = ((float)rand() / (float)RAND_MAX);

					randPoint = uv1 * (v1_v2) + uv2 * (-v2_v3) + v2;

					if ((randPoint - v1) * side1 < 0.0f && (randPoint - v2) * side2 < 0.0f && (randPoint - v3) * side3 < 0.0f) break;
				}

				float scaleAmount = Lerp(plasterSizeMax, plasterSizeMin, 0.0f);

				mat4 curMat;
				curMat.Ident();

				curMat.i[0][3] = randPoint.x;
				curMat.i[1][3] = randPoint.y;
				curMat.i[2][3] = randPoint.z;

				curMat.i[0][0] = axis1.x * scaleAmount;
				curMat.i[1][0] = axis1.y * scaleAmount;
				curMat.i[2][0] = axis1.z * scaleAmount;

				curMat.i[0][1] = faceNormal.x * scaleAmount;
				curMat.i[1][1] = faceNormal.y * scaleAmount;
				curMat.i[2][1] = faceNormal.z * scaleAmount;

				curMat.i[0][2] = axis2.x * scaleAmount;
				curMat.i[1][2] = axis2.y * scaleAmount;
				curMat.i[2][2] = axis2.z * scaleAmount;

				transformSourceMats.emplace_back();
				UnpackMat4(curMat, &transformSourceMats.back());
				transformMats.emplace_back();
				UnpackMat4(curMat, &transformMats.back().mat);
				UnpackMat4(curMat, &transformMats.back().prevMat);
			}

			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[0] = *((float *)&meshCount);
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[1] = curCollection.waveAmplitude;
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[2] = curCollection.wavePhase;
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[3] = 0.0f;

			curCollection.transformInstancesSource = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, transformSourceMats.data(), meshCount * (unsigned int)sizeof(transformSourceData));
			curCollection.transformInstances = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, transformMats.data(), meshCount * (unsigned int)sizeof(transformData));
			curCollection.integrateParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, &curCollection.integrateParams, (unsigned int)sizeof(curCollection.integrateParams));

			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & collectionMaterialGeomMap = curCollection.collectionModel->MaterialGeomMap;
			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = retMeshModel->model->MaterialGeomMap;
			for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = collectionMaterialGeomMap.begin(); it != collectionMaterialGeomMap.end(); ++it)
			{
				for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
					curCollection.sourceGeom.push_back(&(*it2));
				for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
					curCollection.destGeom.push_back(&(*it2));
			}

			std::vector<transformParamsStruct> transformParams;
			for (unsigned int k = 0; k != curCollection.sourceGeom.size(); k++)
			{
				transformParamsStruct curParam;
				curParam.vertCount = curCollection.sourceGeom[k]->getVertCount();
				curParam.instanceCount = meshCount;
				curParam.geomInstCount = (unsigned int)curCollection.sourceGeom.size();
				curParam.srcIdxVertOffset = curCollection.sourceGeom[k]->getDataOffsetInGiantVertexBuffer();
				curParam.dstIdxVertOffset = curCollection.destGeom[k]->getDataOffsetInGiantVertexBuffer();
				transformParams.push_back(curParam);

				inpMinMaxReducer.RequestMinMax(curCollection.destGeom[k]);
			}

			curCollection.transformParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, transformParams.data(), (unsigned int)(transformParams.size() * sizeof(transformParamsStruct)));

			rerecordSubmission = true;

			allCollections[curPlasterId].push_back(curCollection);
		}
	}

	return curPlasterId;
}

unsigned int HIGHOMEGA::WORLD::PlasteredItemsClass::WorkGroupTransformX()
{
	return 8;
}

unsigned int HIGHOMEGA::WORLD::PlasteredItemsClass::WorkGroupTransformY()
{
	return 4;
}

unsigned int HIGHOMEGA::WORLD::PlasteredItemsClass::WorkGroupIntegrateX()
{
	return 32;
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::Update(float elapseTime)
{
	curTimeElapsed += elapseTime;

	for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionPair : allCollections)
		for (plasterCollection & curCollection : curCollectionPair.second)
		{
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[3] = curTimeElapsed;
			curCollection.integrateParamsBuf->UploadSubData(0, &curCollection.integrateParams, sizeof(curCollection.integrateParams));
			curCollection.collectionModel->SetDirty();
		}

	if (rerecordSubmission)
	{
		std::vector<ShaderResource> allTransformInstancesSource;
		std::vector<ShaderResource> allTransformInstances;
		std::vector<ShaderResource> allIntegrateParams;
		std::vector<ShaderResource> allTransformParams;
		unsigned int countCollections = 0u, maxMeshCount = 0u, maxVertCount = 0u;
		for (std::pair<const unsigned long long, std::vector<plasterCollection>>& curCollectionPair : allCollections)
			for (plasterCollection& curCollection : curCollectionPair.second)
			{
				allTransformInstancesSource.emplace_back(RESOURCE_SSBO, COMPUTE, 0, 0, *curCollection.transformInstancesSource, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				allTransformInstances.emplace_back(RESOURCE_SSBO, COMPUTE, 1, 0, *curCollection.transformInstances, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY); // Technically this is produced here, but we haven't run into issues not depending on it in transform
				allIntegrateParams.emplace_back(RESOURCE_SSBO, COMPUTE, 2, 0, *curCollection.integrateParamsBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				allTransformParams.emplace_back(RESOURCE_SSBO, COMPUTE, 2, 0, *curCollection.transformParamsBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				countCollections++;
				maxMeshCount = max (maxMeshCount, curCollection.transformInstances->getSize() / sizeof(transformData));
				for (unsigned int k = 0; k != curCollection.sourceGeom.size(); k++)
					maxVertCount = max (maxVertCount, curCollection.sourceGeom[k]->getVertCount());
			}

		if (integrateSubmission) delete integrateSubmission;
		if (integrateShader) delete integrateShader;
		if (transformCollectionSubmission) delete transformCollectionSubmission;
		if (transformCollectionShader) delete transformCollectionShader;
		integrateSubmission = nullptr;
		integrateShader = nullptr;
		transformCollectionSubmission = nullptr;
		transformCollectionShader = nullptr;

		if (countCollections > 0)
		{
			integrateShader = new ShaderResourceSet;
			integrateSubmission = new ComputeSubmission;
			integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, allTransformInstancesSource, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, allTransformInstances, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY); // Technically this is produced here, but we haven't run into issues not depending on it in transform
			integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, allIntegrateParams, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			integrateShader->Create("shaders/integratePlastered.comp.spv", "main");
			integrateSubmission->MakeDispatch(Instance, "integratePlasteredDispatch", *integrateShader, (unsigned int)ceil((double)maxMeshCount / (double)WorkGroupIntegrateX()), countCollections, 1);

			transformCollectionSubmission = new ComputeSubmission;
			transformCollectionShader = new ShaderResourceSet;
			giantVertBufferSharedMutex.lock_shared();
			transformCollectionShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			transformCollectionShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, allTransformInstances, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			transformCollectionShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, allTransformParams, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			transformCollectionShader->Create("shaders/transformPlastered.comp.spv", "main");
			transformCollectionSubmission->MakeDispatch(Instance, "transformPlasteredDispatch", *transformCollectionShader, (unsigned int)ceil((double)maxVertCount / (double)WorkGroupTransformX()), (unsigned int)ceil((double)maxMeshCount / (double)WorkGroupTransformY()), countCollections);

		}
		rerecordSubmission = false;
	}

	if (integrateSubmission)
	{
		(*integrateSubmission).MakeAsync().Submit();
		(*transformCollectionSubmission).MakeAsync().Submit();
	}

	for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionPair : allCollections)
		for (plasterCollection & curCollection : curCollectionPair.second)
			if (submissionsForPlasteredItems.size() > 0 && !curCollection.collectionModelInst)
			{
				curCollection.collectionModelInst = curCollection.collectionModel->CreateInstance();
				for (GroupedRenderSubmission* curSubmission : submissionsForPlasteredItems)
					perSubmissionCall(curSubmission, curCollection.collectionModelInst);
			}
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::UpdateSDFs(bool forceRefresh)
{
	std::vector<GraphicsModel*> modelsToUpdate;
	for (std::pair<const unsigned long long, std::vector<plasterCollection>>& curCollectionPair : allCollections)
		for (plasterCollection& curCollection : curCollectionPair.second)
			if (!curCollection.collectionModel->sdf) modelsToUpdate.push_back(curCollection.collectionModel);
	if (modelsToUpdate.size()) GraphicsModel::UpdateSDFs(modelsToUpdate, forceRefresh);
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::Remove(unsigned long long curId)
{
	if (allCollections.find(curId) == allCollections.end()) return;

	for (plasterCollection & curCollection : allCollections[curId])
	{
		delete curCollection.transformInstancesSource;
		delete curCollection.transformInstances;
		delete curCollection.integrateParamsBuf;
		delete curCollection.transformParamsBuf;

		delete curCollection.collectionModel;

		{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); cachedMeshesModels[curCollection.meshModelKey].claims--; }
	}

	rerecordSubmission = true;

	allCollections.erase(curId);
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::ClearContent()
{
	for (std::pair<const unsigned long long, std::vector<plasterCollection>>& curCollectionPair : allCollections)
		for (plasterCollection & curCollection : curCollectionPair.second)
		{
			delete curCollection.transformInstancesSource;
			delete curCollection.transformInstances;
			delete curCollection.integrateParamsBuf;
			delete curCollection.transformParamsBuf;

			delete curCollection.collectionModel;

			{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); cachedMeshesModels[curCollection.meshModelKey].claims--; }
		}

	rerecordSubmission = true;

	allCollections.clear();
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);

	if (integrateSubmission)
	{
		delete integrateSubmission;
		delete transformCollectionSubmission;
		integrateSubmission = nullptr;
		transformCollectionSubmission = nullptr;
	}

	submissionsForPlasteredItems.clear();
	curTimeElapsed = 0.0f;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::DoStaticTessellation(PhysicalItemClass* physicalItemsCollection, vec3 curPos, float standingHeight)
{
	std::unique_lock<std::mutex> lk(physicalItemsCollection->staticTessellationMutex);
	for (std::pair<const unsigned long long, RigidBodyItem>& curItem : physicalItemsCollection->allItems)
		if (curItem.second.modelRef) curItem.second.modelRef->doStaticTessellation(Instance, true, &curPos, &standingHeight, false);
	physicalItemsCollection->staticTessellationFinished = true;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::FinishStaticTessellation()
{
	for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
		if (curItem.second.modelRef) curItem.second.modelRef->removeOldTessellation();
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ProcessDestructionCache(Mesh& inpMesh, GraphicsModel& inpModel, RigidBody& inpBody)
{
	vec3 eArr[3];
	vec2 uvArr[3];
	vec3 vnorm, vcol, edge;
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		DataBlock* triBlock, * propBlock;
		HIGHOMEGA::MESH::DataGroup& curPolyGroup = inpMesh.DataGroups[i];

		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock) || !Mesh::getDataBlock(curPolyGroup, "PROPS", &propBlock)) continue;

		float outTmp;
		if (!Mesh::getDataRowFloat(*propBlock, "cuttable", outTmp) && !Mesh::getDataRowFloat(*propBlock, "shatterable", outTmp)) continue;

		bool fillingInData = false, fillingInPointers = false;
		{
			std::lock_guard<std::mutex> lk(destructibleCacheMutex);
			if (destructibles.find(curPolyGroup.name) == destructibles.end())
				fillingInData = true;
			else if (!destructibles[curPolyGroup.name].sourceBody)
				fillingInPointers = true; // Some data was filled in via loading a save...
		}

		if (fillingInData)
		{
			ObjectPiece& mapPiece = *inpBody.getGroupById(curPolyGroup.name);
			std::vector<TriUV> mapPieceGeom;

			RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
			getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

			vec3 pieceMin = vec3(FLT_MAX), pieceMax = vec3(-FLT_MAX);
			for (unsigned int i = 0; i != mapPiece.tris.size(); i++)
			{
				unpackRasterVertex(eArr[0], vcol, uvArr[0], vnorm, verts[indices[i * 3]]);
				unpackRasterVertex(eArr[1], vcol, uvArr[1], vnorm, verts[indices[i * 3 + 1]]);
				unpackRasterVertex(eArr[2], vcol, uvArr[2], vnorm, verts[indices[i * 3 + 2]]);
				vec3 normVec = cross (eArr[0] - eArr[1], eArr[2] - eArr[1]).normalized();
				if (vnorm * normVec < 0.0f) normVec = -normVec;
				AddTri(eArr, uvArr, normVec, mapPieceGeom);
				pieceMin.x = min(pieceMin.x, Min(eArr[0].x, eArr[1].x, eArr[2].x));
				pieceMin.y = min(pieceMin.y, Min(eArr[0].y, eArr[1].y, eArr[2].y));
				pieceMin.z = min(pieceMin.z, Min(eArr[0].z, eArr[1].z, eArr[2].z));
				pieceMax.x = max(pieceMax.x, Max(eArr[0].x, eArr[1].x, eArr[2].x));
				pieceMax.y = max(pieceMax.y, Max(eArr[0].y, eArr[1].y, eArr[2].y));
				pieceMax.z = max(pieceMax.z, Max(eArr[0].z, eArr[1].z, eArr[2].z));
			}

			{
				std::lock_guard<std::mutex> lk(destructibleCacheMutex);
				destructibles[curPolyGroup.name].propBlock = *propBlock;
				destructibles[curPolyGroup.name].sourceTriUVList = mapPieceGeom;
				destructibles[curPolyGroup.name].sourceMin = pieceMin;
				destructibles[curPolyGroup.name].sourceMax = pieceMax;
				destructibles[curPolyGroup.name].sourceModel = &inpModel;
				destructibles[curPolyGroup.name].sourceBody = &inpBody;
			}
		}
		else
		{
			std::lock_guard<std::mutex> lk(destructibleCacheMutex);
			if (fillingInPointers) destructibles[curPolyGroup.name].propBlock = *propBlock;
			if (destructibles[curPolyGroup.name].sourceTriUVList.size() == 0)
			{
				inpModel.removeGroupById(curPolyGroup.name);
				inpBody.removeGroupById(curPolyGroup.name);
			}
			else
			{
				inpModel.ChangeGeom(curPolyGroup.name, destructibles[curPolyGroup.name].sourceTriUVList);
				inpBody.ChangeGeom(curPolyGroup.name, destructibles[curPolyGroup.name].sourceTriUVList);
			}
			destructibles[curPolyGroup.name].sourceModel = &inpModel;
			destructibles[curPolyGroup.name].sourceBody = &inpBody;
		}
	}
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::AddZonePiece(std::string zoneLocation, std::string meshName, Mesh** loadedMesh, vec3 curPos, float standingHeight)
{
	mat4 tmpOrient;
	tmpOrient.Ident();
	vec3 tmpPos = vec3(0.0f);

	unsigned long long pickedId = threadSafeMersenneTwister64Bit();
	RigidBodyItem createdItem;

	Mesh* newMesh = new Mesh(zoneLocation + meshName + std::string(".3md"));
	GraphicsModel* graphicsModel;
	RigidBody* rigidBody;
	std::string placedMesh;
	if (newMesh->DataGroups.size() == 1 && Mesh::getDataRowString(newMesh->DataGroups[0], "PROPS", "placedRelativeMesh", placedMesh)) {
		std::string placedMeshAssetLoc;
		Mesh::getDataRowString(newMesh->DataGroups[0], "PROPS", "placedRelativeMeshAssetLoc", placedMeshAssetLoc);
		mat4 placedMeshTransform;
		Mesh::getDataRowMat4(newMesh->DataGroups[0], "PROPS", "placedRelativeMeshTransform", placedMeshTransform);
		float outTmp, animDuration;
		bool addingAnimatedModel = Mesh::getDataRowFloat(newMesh->DataGroups[0], "PROPS", "animatedModel", animDuration);
		bool addingRagdoll = Mesh::getDataRowFloat(newMesh->DataGroups[0], "PROPS", "ragdoll", outTmp);
		bool addingRigidBody = Mesh::getDataRowFloat(newMesh->DataGroups[0], "PROPS", "rigidBody", outTmp);
		if (addingAnimatedModel || addingRagdoll || addingRigidBody)
		{
			delete newMesh;
			std::string meshPrependString = meshName + ":";
			newMesh = new Mesh(zoneLocation + placedMesh, &meshPrependString);
			createdItem.zoneLocation = zoneLocation;
			createdItem.meshName = meshName;
			graphicsModel = new GraphicsModel();
			rigidBody = new RigidBody();
			if (addingRagdoll || addingRigidBody)
			{
				// Ragdolls and rigid bodies get no scaling...
				vec3 col1 = vec3(placedMeshTransform.i[0][0], placedMeshTransform.i[1][0], placedMeshTransform.i[2][0]).normalized();
				vec3 col2 = vec3(placedMeshTransform.i[0][1], placedMeshTransform.i[1][1], placedMeshTransform.i[2][1]).normalized();
				vec3 col3 = vec3(placedMeshTransform.i[0][2], placedMeshTransform.i[1][2], placedMeshTransform.i[2][2]).normalized();
				placedMeshTransform.i[0][0] = col1.x; placedMeshTransform.i[1][0] = col1.y; placedMeshTransform.i[2][0] = col1.z;
				placedMeshTransform.i[0][1] = col2.x; placedMeshTransform.i[1][1] = col2.y; placedMeshTransform.i[2][1] = col2.z;
				placedMeshTransform.i[0][2] = col3.x; placedMeshTransform.i[1][2] = col3.y; placedMeshTransform.i[2][2] = col3.z;
				// Separate orientation and translation
				vec3 placedMeshTranslationComponent = vec3 (placedMeshTransform.i[0][3], placedMeshTransform.i[1][3], placedMeshTransform.i[2][3]);
				placedMeshTransform.i[0][3] = 0.0f; placedMeshTransform.i[1][3] = 0.0f; placedMeshTransform.i[2][3] = 0.0f;
				if (addingRagdoll)
					createdItem.ragdolls.push_back(AddRagDoll(zoneLocation + placedMesh, zoneLocation + placedMeshAssetLoc, placedMeshTransform, placedMeshTranslationComponent));
				else
					createdItem.attachedBodies.push_back(Add(zoneLocation + placedMesh, zoneLocation + placedMeshAssetLoc, placedMeshTransform, placedMeshTranslationComponent));
			}
			else
			{
				unsigned long long animatedId = animatedMeshCollection.Add(zoneLocation + placedMesh, {}, zoneLocation + placedMeshAssetLoc, animDuration, true, placedMeshTransform, nullptr);
				createdItem.animatedMeshes.push_back(animatedId);
			}
		}
		else
		{
			delete newMesh;
			std::string meshPrependString = meshName + ":";
			newMesh = new Mesh(zoneLocation + placedMesh, &meshPrependString);
			TransformMesh(*newMesh, placedMeshTransform);
			createdItem.zoneLocation = zoneLocation;
			createdItem.meshName = meshName;
			graphicsModel = new GraphicsModel(*newMesh, zoneLocation + placedMeshAssetLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
				float tmpFloat;
				return (!Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) &&
						!Mesh::getDataRowFloat(inpGroup, "PROPS", "animatedModel", tmpFloat) &&
						!Mesh::getDataRowFloat(inpGroup, "PROPS", "ragdoll", tmpFloat) &&
						!Mesh::getDataRowFloat(inpGroup, "PROPS", "rigidBody", tmpFloat));
				}, true, false, &curPos, &standingHeight);
			rigidBody = new RigidBody(*newMesh, zoneLocation + placedMeshAssetLoc, tmpOrient, tmpPos, true,
				[&createdItem,
				&clothCollection = clothCollection,
				&physicalItemBouyancyRangeCollection = physicalItemBuoyancyRangeCollection,
				&inpMesh = *newMesh,
				belong = zoneLocation + placedMeshAssetLoc,
				&orient = tmpOrient,
				&animatedMeshes = animatedMeshCollection,
				thisPtr = this,
				&baseTransform = placedMeshTransform,
				&pos = tmpPos](RigidBody* bodyRef, int groupIndex, DataGroup& curPolyGroup) -> bool {
					float outTmp;
					if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "cloth", outTmp))
					{
						createdItem.clothsAndWindSources.push_back(clothCollection.Add(inpMesh, belong, bodyRef, orient, pos, groupIndex, curPolyGroup));
						return false;
					}
					if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "windSource", outTmp))
					{
						createdItem.clothsAndWindSources.push_back(clothCollection.AddWindSources(curPolyGroup));
						return false;
					}
					if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "buoyancyRange", outTmp))
					{
						createdItem.buoyancyRanges.push_back(physicalItemBouyancyRangeCollection.AddRange(curPolyGroup));
						return false;
					}
					float animDuration;
					bool addingAnimatedModel = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "animatedModel", animDuration);
					bool addingRagdoll = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "ragdoll", outTmp);
					bool addingRigidBody = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "rigidBody", outTmp);
					if (addingAnimatedModel || addingRagdoll || addingRigidBody)
					{
						std::string placedMesh;
						Mesh::getDataRowString(curPolyGroup, "PROPS", "placedRelativeMesh", placedMesh);
						std::string placedMeshAssetLoc;
						Mesh::getDataRowString(curPolyGroup, "PROPS", "placedRelativeMeshAssetLoc", placedMeshAssetLoc);
						mat4 placedTransform;
						Mesh::getDataRowMat4(curPolyGroup, "PROPS", "placedRelativeMeshTransform", placedTransform);
						mat4 combinedTransform = baseTransform * placedTransform;
						if (addingRagdoll || addingRigidBody)
						{
							// Ragdolls and rigid bodies get no scaling...
							vec3 col1 = vec3(combinedTransform.i[0][0], combinedTransform.i[1][0], combinedTransform.i[2][0]).normalized();
							vec3 col2 = vec3(combinedTransform.i[0][1], combinedTransform.i[1][1], combinedTransform.i[2][1]).normalized();
							vec3 col3 = vec3(combinedTransform.i[0][2], combinedTransform.i[1][2], combinedTransform.i[2][2]).normalized();
							combinedTransform.i[0][0] = col1.x; combinedTransform.i[1][0] = col1.y; combinedTransform.i[2][0] = col1.z;
							combinedTransform.i[0][1] = col2.x; combinedTransform.i[1][1] = col2.y; combinedTransform.i[2][1] = col2.z;
							combinedTransform.i[0][2] = col3.x; combinedTransform.i[1][2] = col3.y; combinedTransform.i[2][2] = col3.z;
							// Separate orientation and translation
							vec3 combinedTransformTranslationOnly = vec3(combinedTransform.i[0][3], combinedTransform.i[1][3], combinedTransform.i[2][3]);
							combinedTransform.i[0][3] = 0.0f; combinedTransform.i[1][3] = 0.0f; combinedTransform.i[2][3] = 0.0f;
							if (addingRagdoll)
								createdItem.ragdolls.push_back(thisPtr->AddRagDoll(belong + placedMesh, belong + placedMeshAssetLoc, combinedTransform, combinedTransformTranslationOnly));
							else
								createdItem.attachedBodies.push_back(thisPtr->Add(belong + placedMesh, belong + placedMeshAssetLoc, combinedTransform, combinedTransformTranslationOnly));
						}
						else
						{
							unsigned long long animatedId = animatedMeshes.Add(belong + placedMesh, {}, belong + placedMeshAssetLoc, animDuration, true, combinedTransform, nullptr);
							createdItem.animatedMeshes.push_back(animatedId);
						}
						return false;
					}
					return true;
				});
		}
	}
	else {
		createdItem.zoneLocation = zoneLocation;
		createdItem.meshName = meshName;
		graphicsModel = new GraphicsModel(*newMesh, zoneLocation, Instance, [](int, DataGroup& inpGroup) -> bool {
			float tmpFloat;
			return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
		}, true, false, &curPos, & standingHeight);
		rigidBody = new RigidBody(*newMesh, zoneLocation, tmpOrient, tmpPos, true,
			[&createdItem,
			&clothCollection = clothCollection,
			&physicalItemBouyancyRangeCollection = physicalItemBuoyancyRangeCollection,
			&inpMesh = *newMesh,
			&belong = createdItem.zoneLocation,
			&orient = tmpOrient,
			&animatedMeshes = animatedMeshCollection,
			&pos = tmpPos](RigidBody* bodyRef, int groupIndex, DataGroup& curPolyGroup) -> bool {
				float outTmp;
				if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "cloth", outTmp))
				{
					createdItem.clothsAndWindSources.push_back(clothCollection.Add(inpMesh, belong, bodyRef, orient, pos, groupIndex, curPolyGroup));
					return false;
				}
				if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "windSource", outTmp))
				{
					createdItem.clothsAndWindSources.push_back(clothCollection.AddWindSources(curPolyGroup));
					return false;
				}
				if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "buoyancyRange", outTmp))
				{
					createdItem.buoyancyRanges.push_back(physicalItemBouyancyRangeCollection.AddRange(curPolyGroup));
					return false;
				}
				return true;
			});
	}

	ProcessDestructionCache(*newMesh, *graphicsModel, *rigidBody);

	createdItem.modelRef = graphicsModel;
	createdItem.rigidBodyRef = rigidBody;
	allItems[pickedId] = createdItem;
	*loadedMesh = newMesh;
	return pickedId;
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::Add(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos)
{
	MeshModel* retMeshModel = InsertEntry([&meshLoc, &belong](Mesh& outMesh, GraphicsModel** outModel) {
		outMesh = std::move(Mesh(meshLoc));
		*outModel = new GraphicsModel(outMesh, belong, Instance, [](int, DataGroup& inpGroup) -> bool {
			float tmpFloat;
			return (!Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowFloat(inpGroup, "PROPS", "rigidBody", tmpFloat));
		}, false);
	}, meshLoc, std::string("physical items"), cachedMeshesModels, cachedMeshesModelsMutex);

	unsigned long long pickedId = threadSafeMersenneTwister64Bit();
	RigidBodyItem createdItem;
	createdItem.meshModelRef = retMeshModel;

	createdItem.rigidBodyRef = new RigidBody(retMeshModel->mesh, belong, inpOrient, inpPos, false,
		[&createdItem,
		&clothCollection = clothCollection,
		&inpMesh = retMeshModel->mesh,
		&belong = belong,
		&orient = inpOrient,
		thisPtr = this,
		&pos = inpPos](RigidBody* bodyRef, int groupIndex, DataGroup& curPolyGroup) -> bool {
			float outTmp;
			if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "cloth", outTmp))
			{
				createdItem.clothsAndWindSources.push_back(clothCollection.Add(inpMesh, belong, bodyRef, orient, pos, groupIndex, curPolyGroup));
				return false;
			}
			if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "rigidBody", outTmp))
			{
				// Reconstruct baseTransform
				mat4 baseTransform = orient;
				baseTransform.i[0][3] = pos.x;
				baseTransform.i[1][3] = pos.y;
				baseTransform.i[2][3] = pos.z;
				std::string placedMesh;
				Mesh::getDataRowString(curPolyGroup, "PROPS", "placedRelativeMesh", placedMesh);
				std::string placedMeshAssetLoc;
				Mesh::getDataRowString(curPolyGroup, "PROPS", "placedRelativeMeshAssetLoc", placedMeshAssetLoc);
				mat4 placedTransform;
				Mesh::getDataRowMat4(curPolyGroup, "PROPS", "placedRelativeMeshTransform", placedTransform);
				mat4 combinedTransform = baseTransform * placedTransform;
				// Rigid bodies get no scaling...
				vec3 col1 = vec3(combinedTransform.i[0][0], combinedTransform.i[1][0], combinedTransform.i[2][0]).normalized();
				vec3 col2 = vec3(combinedTransform.i[0][1], combinedTransform.i[1][1], combinedTransform.i[2][1]).normalized();
				vec3 col3 = vec3(combinedTransform.i[0][2], combinedTransform.i[1][2], combinedTransform.i[2][2]).normalized();
				combinedTransform.i[0][0] = col1.x; combinedTransform.i[1][0] = col1.y; combinedTransform.i[2][0] = col1.z;
				combinedTransform.i[0][1] = col2.x; combinedTransform.i[1][1] = col2.y; combinedTransform.i[2][1] = col2.z;
				combinedTransform.i[0][2] = col3.x; combinedTransform.i[1][2] = col3.y; combinedTransform.i[2][2] = col3.z;
				// Separate orientation and translation
				vec3 combinedTransformTranslationOnly = vec3(combinedTransform.i[0][3], combinedTransform.i[1][3], combinedTransform.i[2][3]);
				combinedTransform.i[0][3] = 0.0f; combinedTransform.i[1][3] = 0.0f; combinedTransform.i[2][3] = 0.0f;
				createdItem.attachedBodies.push_back(thisPtr->Add(belong + placedMesh, belong + placedMeshAssetLoc, combinedTransform, combinedTransformTranslationOnly));
				return false;
			}
			return true;
		});

	allItems[pickedId] = createdItem;
	return pickedId;
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::AddRenderOnly(std::string meshLoc, std::string belong, const mat4& inpTrans, bool inNeverHide)
{
	MeshModel* retMeshModel = InsertEntry([&meshLoc, &belong](Mesh& outMesh, GraphicsModel** outModel) {
		outMesh = std::move(Mesh(meshLoc));
		*outModel = new GraphicsModel(outMesh, belong, Instance, [](int, DataGroup& inpGroup) -> bool {
			float tmpFloat;
			return (!Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowFloat(inpGroup, "PROPS", "rigidBody", tmpFloat));
			}, false);
		}, meshLoc, std::string("physical items"), cachedMeshesModels, cachedMeshesModelsMutex);

	unsigned long long pickedId = threadSafeMersenneTwister64Bit();
	RigidBodyItem createdItem;
	createdItem.meshModelRef = retMeshModel;
	vec3 modMin, modMax, modCent;
	createdItem.meshModelRef->model->getModelMinMax(modMin, modMax);
	modCent = (modMin + modMax) * 0.5f;
	createdItem.renderRad = (modMax - modCent).length();
	createdItem.renderMat = inpTrans;
	createdItem.renderOnly = true;
	createdItem.neverHide = inNeverHide;

	allItems[pickedId] = createdItem;
	return pickedId;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::RemoveBulletHolesForItem(unsigned long long itemId)
{
	for (std::pair<const CACHED_BULLETHOLE_TYPE, CachedBulletHoleList>& curTypeBulletHoleListPair : bulletHoles)
		for (CachedBulletHoleList::CachedBulletHole& curBulletHole : curTypeBulletHoleListPair.second.cachedBulletHoles)
			if (curBulletHole.itemId == itemId)
			{
				curBulletHole.itemId = 0u;
				mat4 tmpTrans;tmpTrans.Ident();
				tmpTrans.i[0][3] = 0.0f;
				tmpTrans.i[1][3] = -400.0f;
				tmpTrans.i[2][3] = 0.0f;
				curBulletHole.modelInstRef->Update(tmpTrans);
			}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::UpdateBulletHolesForItem(unsigned long long itemId, mat4& inMat, mat4& inMatDT, bool updatePrevTransform)
{
	for (std::pair<const CACHED_BULLETHOLE_TYPE, CachedBulletHoleList>& curTypeBulletHoleListPair : bulletHoles)
		for (CachedBulletHoleList::CachedBulletHole& curBulletHole : curTypeBulletHoleListPair.second.cachedBulletHoles)
			if (curBulletHole.itemId == itemId)
			{
				vec3 curHoleTan = (inMatDT * curBulletHole.localTan).normalized();
				vec3 curHoleNorm = (inMatDT * curBulletHole.localNorm).normalized();
				vec3 curHoleBiTan = cross(curHoleTan, curHoleNorm);
				vec3 curHolePos = inMat * curBulletHole.localPos;
				mat4 finalHoleTrans;
				finalHoleTrans.Ident();
				finalHoleTrans.i[0][0] = curHoleTan.x;
				finalHoleTrans.i[1][0] = curHoleTan.y;
				finalHoleTrans.i[2][0] = curHoleTan.z;
				finalHoleTrans.i[0][1] = curHoleNorm.x;
				finalHoleTrans.i[1][1] = curHoleNorm.y;
				finalHoleTrans.i[2][1] = curHoleNorm.z;
				finalHoleTrans.i[0][2] = curHoleBiTan.x;
				finalHoleTrans.i[1][2] = curHoleBiTan.y;
				finalHoleTrans.i[2][2] = curHoleBiTan.z;
				finalHoleTrans.i[0][3] = curHolePos.x;
				finalHoleTrans.i[1][3] = curHolePos.y;
				finalHoleTrans.i[2][3] = curHolePos.z;
				curBulletHole.modelInstRef->Update(finalHoleTrans);
				if (updatePrevTransform) curBulletHole.modelInstRef->Update(finalHoleTrans);
			}
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::Add(std::string& newGroupId, RigidBody* oobbPiece, MeshMaterial& origMeshMaterial, std::vector <TriUV>& triList)
{
	unsigned long long pickedId = threadSafeMersenneTwister64Bit();
	RigidBodyItem createdItem;

	createdItem.modelRef = new GraphicsModel(newGroupId, origMeshMaterial, triList);
	createdItem.rigidBodyRef = oobbPiece;

	allItems[pickedId] = createdItem;
	return pickedId;
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::AddRagDoll(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos)
{
	std::unordered_map <std::string, RigidBody*> limbs;

	Mesh curMesh = Mesh(meshLoc);

	std::vector<RagDollPiece> ragDollPieces;

	std::vector<unsigned long long> addedIds;
	addedIds.reserve(curMesh.DataGroups.size());
	for (int i = 0; i != curMesh.DataGroups.size(); i++)
	{
		DataGroup& curPolyGroup = curMesh.DataGroups[i];

		DataBlock* tmpTriBlock;

		std::string transformTarget;
		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &tmpTriBlock) || !Mesh::getDataRowString(curPolyGroup, "PROPS", "ragDollPieceTransform", transformTarget)) continue;

		RigidBodyItem createdItem;

		std::function<bool(RigidBody*, int, DataGroup&)> currentIndexOnlyFilter = [&i = i](RigidBody*, int blockId, DataGroup& dataGroup) -> bool {
			if (blockId == i)
				return true;
			else
				return false;
			};
		vec3 tmpNewCenter = FindCenter(curMesh, currentIndexOnlyFilter);
		// We want to rotate the translated limb (it will ultimately rotate around its centroid). Therefore, apply translation first and then rotate.
		mat4 rotateAfterTrans;
		rotateAfterTrans.Ident();
		rotateAfterTrans.i[0][3] = tmpNewCenter.x;
		rotateAfterTrans.i[1][3] = tmpNewCenter.y;
		rotateAfterTrans.i[2][3] = tmpNewCenter.z;
		rotateAfterTrans = inpOrient * rotateAfterTrans;
		// Extract the final translation
		vec3 extractedTrans = vec3(rotateAfterTrans.i[0][3], rotateAfterTrans.i[1][3], rotateAfterTrans.i[2][3]);
		rotateAfterTrans.i[0][3] = 0.0f;
		rotateAfterTrans.i[1][3] = 0.0f;
		rotateAfterTrans.i[2][3] = 0.0f;
		// tmpNewCenter will be applied internally. Therefore, subtracting when providing the position to avoid double translation.
		createdItem.rigidBodyRef = new RigidBody(curMesh, belong, rotateAfterTrans, extractedTrans - tmpNewCenter + inpPos, false, currentIndexOnlyFilter, &tmpNewCenter);
		RagDollPiece ragDollPiece;
		ragDollPiece.refMatInv.Ident();
		ragDollPiece.refMatInv.i[0][3] = -tmpNewCenter.x;
		ragDollPiece.refMatInv.i[1][3] = -tmpNewCenter.y;
		ragDollPiece.refMatInv.i[2][3] = -tmpNewCenter.z;
		ragDollPiece.piece = createdItem.rigidBodyRef;
		ragDollPiece.transformTarget = transformTarget;
		ragDollPieces.push_back(ragDollPiece);

		unsigned long long curId = threadSafeMersenneTwister64Bit();
		allItems[curId] = createdItem;
		addedIds.push_back(curId);

		limbs[curPolyGroup.name] = createdItem.rigidBodyRef;
	}

	mat4 inpTrans;
	inpTrans = inpOrient;
	inpTrans.i[0][3] = inpPos.x;
	inpTrans.i[1][3] = inpPos.y;
	inpTrans.i[2][3] = inpPos.z;

	ConstraintCollection* addedConstraints = new ConstraintCollection(curMesh, limbs, inpTrans, Instance);

	mat4 tmpMat;
	tmpMat.Ident();
	RigidBodyItem mainItem;
	mainItem.constaintsRef = addedConstraints;
	mainItem.attachedBodies = addedIds;
	mainItem.animatedMeshes.push_back(animatedMeshCollection.Add(meshLoc, {}, belong, 0.0f, false, tmpMat, &ragDollPieces));
	unsigned long long curId = threadSafeMersenneTwister64Bit();
	allItems[curId] = mainItem;

	return curId;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::CutOut(unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float cutRadius, float cutDepth)
{
	// Destruction happens completely async...

	if (booleanOpThread) return;

	booleanOpThread = new std::thread(CutOutThread, this, physicalItemId, groupId, hitDir, hitPoint, hitNorm, cutRadius, cutDepth);
	booleanOpThreadFinished = false;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Shatter(unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm)
{
	// Destruction happens completely async...

	if (booleanOpThread) return;

	booleanOpThread = new std::thread(ShatterThread, this, physicalItemId, groupId, hitDir, hitPoint, hitNorm);
	booleanOpThreadFinished = false;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::CutOutThread(PhysicalItemClass* physicalItemsCollection, unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float cutRadius, float cutDepth)
{
	std::lock_guard<std::mutex> lkdestruct(physicalItemsCollection->destructionMutex);
	std::vector<TriUV> currentMapGeom;
	DataBlock* currentMapPropBlock;
	{
		std::lock_guard<std::mutex> lk(PhysicalItemClass::destructibleCacheMutex);
		currentMapGeom = PhysicalItemClass::destructibles[groupId].sourceTriUVList;
		currentMapPropBlock = &PhysicalItemClass::destructibles[groupId].propBlock;
	}

	std::vector<TriUV> cullGeom, ring, cap, slicedmap, tmp, ringFacingInside;
	MakeCylinder(hitPoint, hitNorm, 2.0f, 4.0f, cullGeom);
	Intersect(currentMapGeom, cullGeom, ring, tmp);
	ringFacingInside = ring;
	for (TriUV& curTri : ringFacingInside)
		curTri.normVec = -curTri.normVec;
	Intersect(cullGeom, currentMapGeom, cap, slicedmap);
	cap.insert(cap.end(), ring.begin(), ring.end());
	slicedmap.insert(slicedmap.end(), ringFacingInside.begin(), ringFacingInside.end());

	std::vector <TriUV> stuckToMapTris; // This will hold triangles that will end being stuck to the map

	// Process non-culled geometry: find islands that do not intersect the map, hole-fill and turn them into rigid bodies (their intersecting geom will be OOBBs)
	while (true)
	{
		std::vector<TriUV> groupedTris;
		FindConnectedNeighbors(slicedmap, groupedTris);
		if (!groupedTris.size()) break;
		std::vector <TriUV> groupedTrisCached = groupedTris;
		vec3 newCenter = Recenter(groupedTris);

		std::string newGroupId = groupId + "_[debrispiece]:" + std::to_string(threadSafeMersenneTwister64Bit());

		std::vector<TriUV> aabbTris;
		std::vector <intersection> tmpIntersectionList;
		GenerateAABB(groupedTris, aabbTris, ColTol);
		RigidBody* tmpRigidBodyAABB = new RigidBody(newGroupId, *currentMapPropBlock, aabbTris, newCenter);
		tmpRigidBodyAABB->ang_v = vec3((rand() % 100 - 50) * 0.02f) * 0.2f;
		CommonSharedMutex.lock_shared();
		for (std::pair<const unsigned long long, RigidBodyItem>& curItem : physicalItemsCollection->allItems)
		{
			if (!curItem.second.rigidBodyRef || !curItem.second.rigidBodyRef->isMap) continue;
			RigidBodyRigidBody(*tmpRigidBodyAABB, *curItem.second.rigidBodyRef, tmpIntersectionList, true);
			if (tmpIntersectionList.size() > 0) break;
		}
		CommonSharedMutex.unlock_shared();

		if (!tmpIntersectionList.size())
		{
			physicalItemsCollection->deferredAdds.emplace_back(physicalItemId, std::string(""), newGroupId, tmpRigidBodyAABB, groupId, groupedTris);
		}
		else
		{
			delete tmpRigidBodyAABB;
			stuckToMapTris.insert(stuckToMapTris.end(), groupedTrisCached.begin(), groupedTrisCached.end());
		}
	}

	// This is the same process as above except every island will turn into a rigid body (they're being culled off the map.)
	while (true)
	{
		std::vector<TriUV> groupedTris;
		FindConnectedNeighbors(cap, groupedTris);
		if (!groupedTris.size()) break;
		vec3 newCenter = Recenter(groupedTris);

		std::string newGroupId = groupId + "_[debrispiece]:" + std::to_string(threadSafeMersenneTwister64Bit());

		std::vector<TriUV> aabbTris;
		std::vector <intersection> tmpIntersectionList;
		GenerateAABB(groupedTris, aabbTris, ColTol);
		RigidBody* tmpRigidBody = new RigidBody(newGroupId, *currentMapPropBlock, aabbTris, newCenter);
		tmpRigidBody->lin_v = hitDir;
		tmpRigidBody->ang_v = vec3((rand() % 100 - 50) * 0.02f) * 0.2f;

		physicalItemsCollection->deferredAdds.emplace_back(physicalItemId, std::string(""), newGroupId, tmpRigidBody, groupId, groupedTris);
	}

	// If there's nothing left for the map destroy the rendered geom and rigid body piece
	if (!stuckToMapTris.size())
	{
		physicalItemsCollection->deferredAdds.emplace_back(physicalItemId, std::string("remove"), groupId, nullptr, groupId, std::vector<TriUV>());
	}
	else // Otherwise update the rendered geom and rigid body piece with the left-over triangles
	{
		physicalItemsCollection->deferredAdds.emplace_back(physicalItemId, std::string("change"), groupId, nullptr, groupId, stuckToMapTris);
	}

	physicalItemsCollection->booleanOpThreadFinished = true;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ShatterThread(PhysicalItemClass* physicalItemsCollection, unsigned long long physicalItemId, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm)
{
	std::lock_guard<std::mutex> lkdestruct(physicalItemsCollection->destructionMutex);
	// Rigid bodies generated by debris will not introduce anything needing an exclusive lock (as of now...)
	std::list <BrokenPiece> piecesMap;
	BrokenPiece& addedPiece = piecesMap.emplace_back();
	std::vector<TriUV>& mapPieceGeom = addedPiece.geom;

	DataBlock* currentMapPropBlock;
	{
		std::lock_guard<std::mutex> lk(PhysicalItemClass::destructibleCacheMutex);
		mapPieceGeom = PhysicalItemClass::destructibles[groupId].sourceTriUVList;
		addedPiece.aabbMin = PhysicalItemClass::destructibles[groupId].sourceMin;
		addedPiece.aabbMax = PhysicalItemClass::destructibles[groupId].sourceMax;
		currentMapPropBlock = &PhysicalItemClass::destructibles[groupId].propBlock;
	}

	addedPiece.cent = (addedPiece.aabbMin + addedPiece.aabbMax) * 0.5f;
	float slicerRad = addedPiece.rad = (addedPiece.aabbMax - addedPiece.cent).length();

	for (unsigned int i = 0; i != 10; i++)
	{
		TriUV slashTri;
		MakeRandomEncompassingTri(addedPiece.aabbMin, addedPiece.aabbMax, slashTri);
		std::vector<TriUV> slashTriVector;
		slashTriVector.push_back(slashTri);

		unsigned int numBrokenPieces = (unsigned int)piecesMap.size();
		unsigned int countPieceProcess = 0;
		for (std::list<BrokenPiece>::iterator it = piecesMap.begin(); it != piecesMap.end();)
		{
			bool objectSlashed = false;
			std::vector<TriUV> above, below;
			float aboveRad, belowRad;
			vec3 aboveCent, belowCent, aboveMin, aboveMax, belowMin, belowMax;
			if (fabs((it->cent - slashTri.eArr[0]) * slashTri.normVec) < it->rad)
			{
				std::vector<TriUV> cap, flippedCap, tmp;
				if (Intersect(it->geom, slashTriVector, cap, tmp))
				{
					flippedCap = cap;
					for (TriUV& curFlippedCapTri : flippedCap)
						curFlippedCapTri.normVec = -curFlippedCapTri.normVec;
					Slice(slashTri, it->geom, above, below);
					bool aboveChecked = false, belowChecked = false;
					for (TriUV& curAboveTri : above)
					{
						if (!aboveChecked)
						{
							aboveMin.x = Min(curAboveTri.eArr[0].x, curAboveTri.eArr[1].x, curAboveTri.eArr[2].x);
							aboveMin.y = Min(curAboveTri.eArr[0].y, curAboveTri.eArr[1].y, curAboveTri.eArr[2].y);
							aboveMin.z = Min(curAboveTri.eArr[0].z, curAboveTri.eArr[1].z, curAboveTri.eArr[2].z);
							aboveMax.x = Max(curAboveTri.eArr[0].x, curAboveTri.eArr[1].x, curAboveTri.eArr[2].x);
							aboveMax.y = Max(curAboveTri.eArr[0].y, curAboveTri.eArr[1].y, curAboveTri.eArr[2].y);
							aboveMax.z = Max(curAboveTri.eArr[0].z, curAboveTri.eArr[1].z, curAboveTri.eArr[2].z);
							aboveChecked = true;
						}
						else
						{
							aboveMin.x = min(aboveMin.x, Min(curAboveTri.eArr[0].x, curAboveTri.eArr[1].x, curAboveTri.eArr[2].x));
							aboveMin.y = min(aboveMin.y, Min(curAboveTri.eArr[0].y, curAboveTri.eArr[1].y, curAboveTri.eArr[2].y));
							aboveMin.z = min(aboveMin.z, Min(curAboveTri.eArr[0].z, curAboveTri.eArr[1].z, curAboveTri.eArr[2].z));
							aboveMax.x = max(aboveMax.x, Max(curAboveTri.eArr[0].x, curAboveTri.eArr[1].x, curAboveTri.eArr[2].x));
							aboveMax.y = max(aboveMax.y, Max(curAboveTri.eArr[0].y, curAboveTri.eArr[1].y, curAboveTri.eArr[2].y));
							aboveMax.z = max(aboveMax.z, Max(curAboveTri.eArr[0].z, curAboveTri.eArr[1].z, curAboveTri.eArr[2].z));
						}
					}
					aboveCent = (aboveMin + aboveMax) * 0.5f;
					aboveRad = (aboveMax - aboveCent).length();
					for (TriUV& curBelowTri : below)
					{
						if (!belowChecked)
						{
							belowMin.x = Min(curBelowTri.eArr[0].x, curBelowTri.eArr[1].x, curBelowTri.eArr[2].x);
							belowMin.y = Min(curBelowTri.eArr[0].y, curBelowTri.eArr[1].y, curBelowTri.eArr[2].y);
							belowMin.z = Min(curBelowTri.eArr[0].z, curBelowTri.eArr[1].z, curBelowTri.eArr[2].z);
							belowMax.x = Max(curBelowTri.eArr[0].x, curBelowTri.eArr[1].x, curBelowTri.eArr[2].x);
							belowMax.y = Max(curBelowTri.eArr[0].y, curBelowTri.eArr[1].y, curBelowTri.eArr[2].y);
							belowMax.z = Max(curBelowTri.eArr[0].z, curBelowTri.eArr[1].z, curBelowTri.eArr[2].z);
							belowChecked = true;
						}
						else
						{
							belowMin.x = min(belowMin.x, Min(curBelowTri.eArr[0].x, curBelowTri.eArr[1].x, curBelowTri.eArr[2].x));
							belowMin.y = min(belowMin.y, Min(curBelowTri.eArr[0].y, curBelowTri.eArr[1].y, curBelowTri.eArr[2].y));
							belowMin.z = min(belowMin.z, Min(curBelowTri.eArr[0].z, curBelowTri.eArr[1].z, curBelowTri.eArr[2].z));
							belowMax.x = max(belowMax.x, Max(curBelowTri.eArr[0].x, curBelowTri.eArr[1].x, curBelowTri.eArr[2].x));
							belowMax.y = max(belowMax.y, Max(curBelowTri.eArr[0].y, curBelowTri.eArr[1].y, curBelowTri.eArr[2].y));
							belowMax.z = max(belowMax.z, Max(curBelowTri.eArr[0].z, curBelowTri.eArr[1].z, curBelowTri.eArr[2].z));
						}
					}
					belowCent = (belowMin + belowMax) * 0.5f;
					belowRad = (belowMax - belowCent).length();
					below.insert(below.end(), cap.begin(), cap.end());
					above.insert(above.end(), flippedCap.begin(), flippedCap.end());
					if (above.size() == 0 || below.size() == 0) // Sometimes due to thresholding one of them might end up empty...
						objectSlashed = false;
					else
						objectSlashed = true;
				}
			}

			if (objectSlashed)
			{
				piecesMap.erase(it++);
				BrokenPiece& newAbovePiece = piecesMap.emplace_back();
				newAbovePiece.aabbMin = aboveMin;
				newAbovePiece.aabbMax = aboveMax;
				newAbovePiece.geom = above;
				newAbovePiece.cent = aboveCent;
				newAbovePiece.rad = aboveRad;
				BrokenPiece& newBelowPiece = piecesMap.emplace_back();
				newBelowPiece.aabbMin = belowMin;
				newBelowPiece.aabbMax = belowMax;
				newBelowPiece.geom = below;
				newBelowPiece.cent = belowCent;
				newBelowPiece.rad = belowRad;
			}
			else
				it++;
			countPieceProcess++;
			if (countPieceProcess == numBrokenPieces) break;
		}
	}

	unsigned int maxPieces = (unsigned int)piecesMap.size();
	unsigned int countPieces = 0;
	for (std::list<BrokenPiece>::iterator it = piecesMap.begin(); it != piecesMap.end();)
	{
		bool splitSomething = true;
		while (true)
		{
			std::vector<TriUV> groupedTris;
			FindConnectedNeighbors(it->geom, groupedTris);
			if (!groupedTris.size()) break;

			if (groupedTris.size() == it->geom.size())
			{
				splitSomething = false;
				break;
			}
			else
			{
				vec3 newFragMin, newFragMax, newFragmentCent;
				float newFragmentRad;
				BrokenPiece& newFragmentPiece = piecesMap.emplace_back();
				bool newFragChecked = false;
				for (TriUV& curTri : groupedTris)
				{
					if (!newFragChecked)
					{
						newFragMin.x = Min(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x);
						newFragMin.y = Min(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y);
						newFragMin.z = Min(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z);
						newFragMax.x = Max(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x);
						newFragMax.y = Max(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y);
						newFragMax.z = Max(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z);
						newFragChecked = true;
					}
					else
					{
						newFragMin.x = min(newFragMin.x, Min(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x));
						newFragMin.y = min(newFragMin.y, Min(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y));
						newFragMin.z = min(newFragMin.z, Min(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z));
						newFragMax.x = max(newFragMax.x, Max(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x));
						newFragMax.y = max(newFragMax.y, Max(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y));
						newFragMax.z = max(newFragMax.z, Max(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z));
					}
				}
				newFragmentCent = (newFragMax + newFragMin) * 0.5f;
				newFragmentRad = (newFragMax - newFragmentCent).length();
				newFragmentPiece.aabbMin = newFragMin;
				newFragmentPiece.aabbMax = newFragMax;
				newFragmentPiece.geom = groupedTris;
				newFragmentPiece.cent = newFragmentCent;
				newFragmentPiece.rad = newFragmentRad;

				splitSomething = true;
			}
		}

		if (splitSomething)
			piecesMap.erase(it++);
		else
			it++;

		countPieces++;
		if (countPieces == maxPieces) break;
	}

	for (BrokenPiece& curPiece : piecesMap)
	{
		vec3 newCenter = Recenter(curPiece.geom);

		std::string newGroupId = groupId + "_[debrispiece]:" + std::to_string(threadSafeMersenneTwister64Bit());

		std::vector<TriUV> aabbTris;
		std::vector <intersection> tmpIntersectionList;
		GenerateAABB(curPiece.geom, aabbTris, ColTol);
		RigidBody* tmpRigidBodyAABB = new RigidBody(newGroupId, *currentMapPropBlock, aabbTris, newCenter);
		tmpRigidBodyAABB->ang_v = vec3((rand() % 100 - 50) * 0.02f) * 0.4f;
		tmpRigidBodyAABB->lin_v = -hitNorm * 3.0f + hitDir * 0.25f + vec3((rand() % 100 - 50) * 0.02f) * 0.2f;

		physicalItemsCollection->deferredAdds.emplace_back(physicalItemId, std::string(""), newGroupId, tmpRigidBodyAABB, groupId, curPiece.geom);
	}
	physicalItemsCollection->deferredAdds.emplace_back(physicalItemId, std::string("remove"), groupId, nullptr, groupId, std::vector<TriUV>());

	physicalItemsCollection->booleanOpThreadFinished = true;
}

namespace HIGHOMEGA::FIZ_X
{
	struct DestructibleEffectInfo
	{
		std::string audioLoc;
		HIGHOMEGA::FIZ_X::MATERIAL particleType;
	};
	std::unordered_map <HIGHOMEGA::FIZ_X::MATERIAL, DestructibleEffectInfo> matToShatterFX = {
		{CERAMIC, {"assets/audio/sfx/shatter/glass.wav", GLASS}} ,
		{GLASS, {"assets/audio/sfx/shatter/glass.wav", GLASS}} ,
		{WOOD, {"assets/audio/sfx/shatter/wood.wav", CONCRETE}}
	};
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Update(std::function <bool(vec3&, vec3&, float)> isInDrawRegion, vec3 curPos, float standingHeight, ParticleSystemClass& particleSystem, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission, WorldParamsClass& worldParams)
{
	// Allow more geometry destruction to happen...
	if (booleanOpThread && booleanOpThreadFinished)
	{
		CommonSharedMutex.unlock_shared();
		booleanOpThread->join();
		delete booleanOpThread;
		booleanOpThread = nullptr;
		booleanOpThreadFinished = false;

		// Add all objects for which addition was deferred
		for (AddParams& curAdd : deferredAdds)
		{
			if (curAdd.origPieceAction == std::string(""))
			{
				MeshMaterial destructMaterial;
				{
					std::lock_guard<std::mutex> lk(destructibleCacheMutex);
					destructMaterial = destructibles[curAdd.origGroupId].sourceModel->getMaterialById(curAdd.origGroupId);
				}
				unsigned long long addedPieceId = Add(curAdd.newGroupId, curAdd.oobbPiece, destructMaterial, curAdd.triList);
				HIGHOMEGA::FIZ_X::MATERIAL& objMat = allItems[addedPieceId].rigidBodyRef->pieces[0].mat;
				vec3 objPos = allItems[addedPieceId].rigidBodyRef->pos;
				vec3 objLinV = allItems[addedPieceId].rigidBodyRef->lin_v;
				if (matToShatterFX.find(objMat) != matToShatterFX.end())
				{
					AudioSystem.Insert(matToShatterFX[objMat].audioLoc, AudioSystemClass::SOUND_TYPE::SFX, objPos, objLinV, 3.0f);
					particleSystem.AddBulletEmitter(objPos, -objLinV.normalized(), inpMinMaxReducer, sdfBvhSubmission, rtSubmission, matToShatterFX[objMat].particleType);
				}
				TransferCutOutBulletHoles(curAdd.sourcePhysicalItemId, addedPieceId, curAdd.triList);
			}
			else if (curAdd.origPieceAction == std::string("remove"))
			{
				CommonSharedMutex.lock();
				{
					std::lock_guard<std::mutex> lk(destructibleCacheMutex);
					destructibles[curAdd.origGroupId].sourceBody->removeGroupById(curAdd.newGroupId);
					destructibles[curAdd.origGroupId].sourceModel->removeGroupById(curAdd.newGroupId);
					destructibles[curAdd.origGroupId].sourceTriUVList = {};
				}
				CommonSharedMutex.unlock();
			}
			else if (curAdd.origPieceAction == std::string("change"))
			{
				CommonSharedMutex.lock();
				{
					std::lock_guard<std::mutex> lk(destructibleCacheMutex);
					destructibles[curAdd.origGroupId].sourceBody->ChangeGeom(curAdd.newGroupId, curAdd.triList);
					destructibles[curAdd.origGroupId].sourceModel->ChangeGeom(curAdd.newGroupId, curAdd.triList);
					destructibles[curAdd.origGroupId].sourceTriUVList = curAdd.triList;
				}
				CommonSharedMutex.unlock();
			}
		}
		deferredAdds.clear();
		CommonSharedMutex.lock_shared();
	}

	if (allGeomToSerializeOrDeSerialize.size())
	{
		while (true)
		{
			bool addedALoadedDebrisPiece = false;
			for (std::vector<GeomToSerializeOrDeSerialize>::iterator it = allGeomToSerializeOrDeSerialize.begin(); it != allGeomToSerializeOrDeSerialize.end(); ++it)
			{
				GeomToSerializeOrDeSerialize& curGeomToLoad = *it;
				std::vector<std::string> geomNameSplit;

				std::string matchedName = "";
				DataBlock propBlockFound;
				MeshMaterial destructMaterialFound;
				bool destructModelInMemory = false;

				split(geomNameSplit, curGeomToLoad.name, "_[debrispiece]:");
				{
					std::lock_guard<std::mutex> lk(destructibleCacheMutex);
					if (destructibles.find(geomNameSplit[0]) != destructibles.end())
					{
						matchedName = curGeomToLoad.name;
						propBlockFound = destructibles[geomNameSplit[0]].propBlock;
						if (destructibles[geomNameSplit[0]].sourceModel)
						{
							destructMaterialFound = destructibles[geomNameSplit[0]].sourceModel->getMaterialById(geomNameSplit[0]);
							destructModelInMemory = true;
						}
					}
				}
				if (matchedName.length())
				{
					if (destructModelInMemory)
					{
						CommonSharedMutex.unlock_shared();
						CommonSharedMutex.lock();
						vec3 newCenter = Recenter(curGeomToLoad.geomTriUV);
						std::vector<TriUV> aabbTris;
						GenerateAABB(curGeomToLoad.geomTriUV, aabbTris, ColTol);
						std::string newDebrisName = geomNameSplit[0] + "_[debrispiece]:" + std::to_string(threadSafeMersenneTwister64Bit());
						RigidBody* oobbRigidBody = new RigidBody(newDebrisName, propBlockFound, aabbTris, newCenter);
						mat4 rigidBodyOrient = curGeomToLoad.trans;
						vec3 rigidBodyPos = vec3(rigidBodyOrient.i[0][3], rigidBodyOrient.i[1][3], rigidBodyOrient.i[2][3]);
						rigidBodyOrient.i[0][3] = 0.0f;
						rigidBodyOrient.i[1][3] = 0.0f;
						rigidBodyOrient.i[2][3] = 0.0f;
						Add(newDebrisName, oobbRigidBody, destructMaterialFound, curGeomToLoad.geomTriUV);
						oobbRigidBody->orient = rigidBodyOrient;
						oobbRigidBody->pos = rigidBodyPos;
						CommonSharedMutex.unlock();
						CommonSharedMutex.lock_shared();
					}
					allGeomToSerializeOrDeSerialize.erase(it);
					addedALoadedDebrisPiece = true;
					break;
				}
			}
			if (!addedALoadedDebrisPiece) break;
		}
	}
	if (allDestructibleMapGeomToDeSerialize.size())
	{
		while (true)
		{
			bool addedAGeomSource = false;
			for (std::vector<DestructibleMapGeomToDeSerialize>::iterator it = allDestructibleMapGeomToDeSerialize.begin(); it != allDestructibleMapGeomToDeSerialize.end(); ++it)
			{
				DestructibleMapGeomToDeSerialize& curSourceGeom = *it;

				std::string matchedName = "";
				DataBlock propBlockFound;
				MeshMaterial destructMaterialFound;

				{
					std::lock_guard<std::mutex> lk(destructibleCacheMutex);
					if (destructibles.find(curSourceGeom.name) != destructibles.end() && destructibles[curSourceGeom.name].sourceBody)
					{
						matchedName = curSourceGeom.name;
						propBlockFound = destructibles[curSourceGeom.name].propBlock;
					}
				}
				if (matchedName.length())
				{
					CommonSharedMutex.unlock_shared();
					CommonSharedMutex.lock();
					{
						std::lock_guard<std::mutex> lk(destructibleCacheMutex);
						if (destructibles[matchedName].sourceBody) // Could be loading out of zone...
							if (destructibles[matchedName].sourceTriUVList.size() == 0)
							{
								destructibles[matchedName].sourceModel->removeGroupById(matchedName);
								destructibles[matchedName].sourceBody->removeGroupById(matchedName);
							}
							else
							{
								destructibles[matchedName].sourceModel->ChangeGeom(matchedName, destructibles[matchedName].sourceTriUVList);
								destructibles[matchedName].sourceBody->ChangeGeom(matchedName, destructibles[matchedName].sourceTriUVList);
							}
					}
					CommonSharedMutex.unlock();
					CommonSharedMutex.lock_shared();
					allDestructibleMapGeomToDeSerialize.erase(it);
					addedAGeomSource = true;
					break;
				}
			}
			if (!addedAGeomSource) break;
		}
	}

	// Do static tessellation...
	if ((curPos - cachedPos).length() > 100.0f && !staticTessellationThread)
	{
		cachedPos = curPos;
		staticTessellationFinished = false;
		staticTessellationThread = new std::thread(DoStaticTessellation, this, curPos, standingHeight);
	}
	if (staticTessellationThread && staticTessellationFinished)
	{
		staticTessellationThread->join();
		delete staticTessellationThread;
		staticTessellationThread = nullptr;
		staticTessellationFinished = false;
		FinishStaticTessellation();
	}

	std::vector<GraphicsModel *> buildingModels, rigidModels;
	std::vector<std::string> buildingCacheNames;
	buildingCacheNames.reserve(allItems.size());
	buildingModels.reserve(allItems.size());
	rigidModels.reserve(allItems.size());

	if (!RTInstance::Enabled())
	{
		for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
		{
			if (curItem.second.modelRef && !curItem.second.modelRef->sdf)
			{
				if (curItem.second.meshName != "")
				{
					buildingModels.push_back(curItem.second.modelRef);
					buildingCacheNames.push_back(curItem.second.zoneLocation + curItem.second.meshName + ".matdf");
				}
				else
					rigidModels.push_back(curItem.second.modelRef);
			}
			else if (curItem.second.meshModelRef && !curItem.second.meshModelRef->model->sdf && std::find(rigidModels.begin(), rigidModels.end(), curItem.second.meshModelRef->model) == rigidModels.end())
			{
				rigidModels.push_back(curItem.second.meshModelRef->model);
			}
		}
		if (rigidModels.size() > 0) GraphicsModel::UpdateSDFs(rigidModels); // SDF leaves for rigid bodies...
		if (buildingModels.size() > 0) GraphicsModel::UpdateSDFs(buildingModels, false, buildingCacheNames); // SDF leaves for buildings...
	}

	AddNewItemsToWorld(true);

	animatedMeshCollection.Process(worldParams.GetFrameTime());

	clothCollection.Update();

	ProcessFireLines(particleSystem, inpMinMaxReducer, sdfBvhSubmission, rtSubmission);

	for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
	{
		RigidBodyItem& curRigidItem = curItem.second;
		if (curRigidItem.rigidBodyRef && curRigidItem.rigidBodyRef->isMap) continue;

		vec3 bodyPos;
		float bodyRad;
		mat4 trans;
		if (!curRigidItem.renderOnly && curRigidItem.rigidBodyRef)
		{
			RigidBody* rigidBodyRef = curRigidItem.rigidBodyRef;
			bodyPos = curRigidItem.rigidBodyRef->pos;
			bodyRad = curRigidItem.rigidBodyRef->radius;
			trans = curRigidItem.rigidBodyRef->orient;
			trans.i[0][3] = curRigidItem.rigidBodyRef->pos.x;
			trans.i[1][3] = curRigidItem.rigidBodyRef->pos.y;
			trans.i[2][3] = curRigidItem.rigidBodyRef->pos.z;
		}
		else
		{
			bodyPos.x = curRigidItem.renderMat.i[0][3];
			bodyPos.y = curRigidItem.renderMat.i[1][3];
			bodyPos.z = curRigidItem.renderMat.i[2][3];
			bodyRad = curRigidItem.renderRad;
			trans = curRigidItem.renderMat;
		}
		bool justCreated = false;

		if ((curRigidItem.modelRef || curRigidItem.meshModelRef) && !curRigidItem.neverHide)
		{
			if (subList.size() > 0 && !isInDrawRegion(MainFrustum.eye, bodyPos, bodyRad) && curItem.second.modelInstRef)
			{
				curItem.second.modelInstRef->modelRef->DestroyInstance(curItem.second.modelInstRef);
				curItem.second.modelInstRef = nullptr;
				RemoveBulletHolesForItem(curItem.first);
			}
			else if (subList.size() > 0 && isInDrawRegion(MainFrustum.eye, bodyPos, bodyRad) && !curItem.second.modelInstRef)
			{
				curItem.second.modelInstRef = (curRigidItem.modelRef ? *curRigidItem.modelRef : *curRigidItem.meshModelRef->model).CreateInstance(&trans, curRigidItem.rigidBodyRef ? (!curRigidItem.rigidBodyRef->isMap) : true);
				for (GroupedRenderSubmission* curSub : subList)
					perSubmissionCall(curSub, curItem.second.modelInstRef);
				justCreated = true;
			}
		}

		if (curItem.second.modelInstRef && !justCreated) curItem.second.modelInstRef->Update(trans);
		if (curRigidItem.rigidBodyRef)
		{
			mat4 transDT = trans.DirectionTransform();
			UpdateBulletHolesForItem(curItem.first, trans, transDT);
		}
	}

	if (GetStateOfAction(CMD_QUICKSAVE))
	{
		allGeomToSerializeOrDeSerialize.clear();
		for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
		{
			RigidBodyItem& curRigidItem = curItem.second;
			if (!(curRigidItem.rigidBodyRef && !curRigidItem.rigidBodyRef->isMap && curRigidItem.modelRef && curRigidItem.modelRef->MaterialGeomMap.size() == 1 &&
				curRigidItem.modelRef->MaterialGeomMap.begin()->second.begin()->getGroupId().contains("_[debrispiece]:"))) continue;
			std::string geomName = curRigidItem.modelRef->MaterialGeomMap.begin()->second.begin()->getGroupId();
			std::vector<unsigned char> geom;
			std::vector<TriUV> geomTriUV;
			curRigidItem.modelRef->MaterialGeomMap.begin()->second.begin()->Download(geom);
			mat4 tmpTrans = curRigidItem.rigidBodyRef->orient;
			tmpTrans.i[0][3] = curRigidItem.rigidBodyRef->pos.x;
			tmpTrans.i[1][3] = curRigidItem.rigidBodyRef->pos.y;
			tmpTrans.i[2][3] = curRigidItem.rigidBodyRef->pos.z;
			GraphicsModel::ExtractGeom(geom, geomTriUV);
			allGeomToSerializeOrDeSerialize.emplace_back(geomName, geomTriUV, tmpTrans);
		}
		std::vector<unsigned char> outStream;
		HIGHOMEGA::SERIALIZATION::serialize<unsigned int>(HIGHOMEGA_SAVE_VERSION, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<std::string>(zoneStreaming.zoneLocation, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<vec3>(player.physics.bodyPos, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<float>(player.physics.lookVectorXZAngle, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<float>(player.physics.lookVectorYAngle, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<vec3>(player.physics.playerVel, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<float>(player.physics.bodyStretch, outStream);

		CommonSharedMutex.unlock_shared();
		CommonSharedMutex.lock();
		HIGHOMEGA::SERIALIZATION::serialize<std::string>(player.physics.ladderInfo.climbingLadder ? player.physics.ladderInfo.climbingLadder->name : "", outStream);
		HIGHOMEGA::SERIALIZATION::serialize<vec3>(player.physics.ladderInfo.curEye, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<vec3>(player.physics.ladderInfo.curLook, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<unsigned int>((unsigned int)player.physics.ladderInfo.transitionState, outStream);
		HIGHOMEGA::SERIALIZATION::serialize<float>(player.physics.ladderInfo.transitionFraction, outStream);
		CommonSharedMutex.unlock();
		CommonSharedMutex.lock_shared();

		HIGHOMEGA::SERIALIZATION::serialize<float>(worldParams.GetSunAngle(), outStream);

		HIGHOMEGA::SERIALIZATION::serialize<unsigned int>((unsigned int)allGeomToSerializeOrDeSerialize.size(), outStream);
		for (GeomToSerializeOrDeSerialize& curGeomToSave : allGeomToSerializeOrDeSerialize)
		{
			HIGHOMEGA::SERIALIZATION::serialize<std::string>(curGeomToSave.name, outStream);
			HIGHOMEGA::SERIALIZATION::serializeVector<TriUV>(curGeomToSave.geomTriUV, outStream);
			HIGHOMEGA::SERIALIZATION::serialize<mat4>(curGeomToSave.trans, outStream);
		}
		std::unordered_map <std::string, HIGHOMEGA::WORLD::PhysicalItemClass::Destructible> destructiblesCopy;
		{
			std::lock_guard<std::mutex> lk(destructibleCacheMutex);
			destructiblesCopy = destructibles;
		}
		HIGHOMEGA::SERIALIZATION::serialize<unsigned int>((unsigned int)destructiblesCopy.size(), outStream);
		for (std::pair<const std::string, Destructible>& curDestructible : destructiblesCopy)
		{
			HIGHOMEGA::SERIALIZATION::serialize<std::string>(curDestructible.first, outStream);
			HIGHOMEGA::SERIALIZATION::serialize<vec3>(curDestructible.second.sourceMin, outStream);
			HIGHOMEGA::SERIALIZATION::serialize<vec3>(curDestructible.second.sourceMax, outStream);
			HIGHOMEGA::SERIALIZATION::serializeVector<TriUV>(curDestructible.second.sourceTriUVList, outStream);
		}
		if (!ResourceLoader::DirectoryExists("saves")) ResourceLoader::MakeDirectory("saves");
		ResourceSaver::SaveBlob("saves/quicksave.sav", outStream.data(), (unsigned int)outStream.size());
		allGeomToSerializeOrDeSerialize.clear();
	}
	else if (GetStateOfAction(CMD_QUICKLOAD))
	{
		unsigned char* content;
		unsigned int contentSize;
		if (HIGHOMEGA::ResourceLoader::LoadFile("saves/quicksave.sav", &content, contentSize) == HIGHOMEGA::ResourceLoader::FILE_LOAD_RESULT::FILE_LOAD_SUCCESS)
		{
			CommonSharedMutex.unlock_shared();
			CommonSharedMutex.lock();
			bool removedAPiece;
			do
			{
				removedAPiece = false;
				for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
				{
					RigidBodyItem& curRigidItem = curItem.second;
					if (!(curRigidItem.rigidBodyRef && !curRigidItem.rigidBodyRef->isMap && curRigidItem.modelRef && curRigidItem.modelRef->MaterialGeomMap.size() == 1 &&
						curRigidItem.modelRef->MaterialGeomMap.begin()->second.begin()->getGroupId().contains("_[debrispiece]:"))) continue;
					Remove(curItem.first);
					removedAPiece = true;
					break;
				}
			} while (removedAPiece);
			CommonSharedMutex.unlock();
			CommonSharedMutex.lock_shared();

			std::vector<unsigned char> loadedStream;
			loadedStream.resize(contentSize);
			memcpy(loadedStream.data(), content, contentSize);
			delete content;

			unsigned int streamIdx = 0u;
			unsigned int saveVersion;
			allGeomToSerializeOrDeSerialize.clear();
			HIGHOMEGA::SERIALIZATION::deserialize<unsigned int>(loadedStream, saveVersion, streamIdx);
			if (saveVersion == HIGHOMEGA_SAVE_VERSION)
			{
				std::string zoneLocation; // We don't care about this... for _now_
				HIGHOMEGA::SERIALIZATION::deserialize<std::string>(loadedStream, zoneLocation, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<vec3>(loadedStream, zoneStreaming.playerStatePostStream.pos, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<float>(loadedStream, zoneStreaming.playerStatePostStream.lookAngles.x, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<float>(loadedStream, zoneStreaming.playerStatePostStream.lookAngles.y, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<vec3>(loadedStream, zoneStreaming.playerStatePostStream.vel, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<float>(loadedStream, zoneStreaming.playerStatePostStream.bodyStretch, streamIdx);

				HIGHOMEGA::SERIALIZATION::deserialize<std::string>(loadedStream, zoneStreaming.playerStatePostStream.climbingLadderName, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<vec3>(loadedStream, zoneStreaming.playerStatePostStream.ladderEye, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<vec3>(loadedStream, zoneStreaming.playerStatePostStream.ladderLook, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<unsigned int>(loadedStream, zoneStreaming.playerStatePostStream.ladderTransitionStateInt, streamIdx);
				HIGHOMEGA::SERIALIZATION::deserialize<float>(loadedStream, zoneStreaming.playerStatePostStream.ladderTransitionFraction, streamIdx);

				HIGHOMEGA::SERIALIZATION::deserialize<float>(loadedStream, zoneStreaming.playerStatePostStream.sunAngle, streamIdx);

				unsigned int numGeom = 0u;
				HIGHOMEGA::SERIALIZATION::deserialize<unsigned int>(loadedStream, numGeom, streamIdx);
				allGeomToSerializeOrDeSerialize.resize(numGeom);
				for (GeomToSerializeOrDeSerialize& curGeomToSave : allGeomToSerializeOrDeSerialize)
				{
					HIGHOMEGA::SERIALIZATION::deserialize<std::string>(loadedStream, curGeomToSave.name, streamIdx);
					HIGHOMEGA::SERIALIZATION::deserializeVector<TriUV>(loadedStream, curGeomToSave.geomTriUV, streamIdx);
					HIGHOMEGA::SERIALIZATION::deserialize<mat4>(loadedStream, curGeomToSave.trans, streamIdx);
				}
				unsigned int allDestructibleMapGeomToDeSerializeLen;
				HIGHOMEGA::SERIALIZATION::deserialize<unsigned int>(loadedStream, allDestructibleMapGeomToDeSerializeLen, streamIdx);
				allDestructibleMapGeomToDeSerialize.resize(allDestructibleMapGeomToDeSerializeLen);
				for (unsigned int i = 0; i != allDestructibleMapGeomToDeSerializeLen; i++)
				{
					HIGHOMEGA::SERIALIZATION::deserialize<std::string>(loadedStream, allDestructibleMapGeomToDeSerialize[i].name, streamIdx);
					HIGHOMEGA::SERIALIZATION::deserialize<vec3>(loadedStream, allDestructibleMapGeomToDeSerialize[i].sourceMin, streamIdx);
					HIGHOMEGA::SERIALIZATION::deserialize<vec3>(loadedStream, allDestructibleMapGeomToDeSerialize[i].sourceMax, streamIdx);
					HIGHOMEGA::SERIALIZATION::deserializeVector<TriUV>(loadedStream, allDestructibleMapGeomToDeSerialize[i].sourceTriUVList, streamIdx);
				}
				{
					std::lock_guard<std::mutex> lk(destructibleCacheMutex);
					for (DestructibleMapGeomToDeSerialize& curSourceGeom : allDestructibleMapGeomToDeSerialize)
					{
						destructibles[curSourceGeom.name].sourceMin = curSourceGeom.sourceMin;
						destructibles[curSourceGeom.name].sourceMax = curSourceGeom.sourceMax;
						destructibles[curSourceGeom.name].sourceTriUVList = curSourceGeom.sourceTriUVList;
					}
				}
				zoneStreaming.setPlayerStatePostStream = true;
			}
		}
	}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::TransferCutOutBulletHoles(unsigned long long srcPhysicalItemId, unsigned long long dstPhysicalItemId, std::vector<TriUV>& dstTriList)
{
	vec3 pieceMin(FLT_MAX), pieceMax(-FLT_MAX);
	for (TriUV& curTri : dstTriList)
	{
		pieceMin.x = min(Min(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x), pieceMin.x);
		pieceMin.y = min(Min(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y), pieceMin.y);
		pieceMin.z = min(Min(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z), pieceMin.z);
		pieceMax.x = max(Max(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x), pieceMax.x);
		pieceMax.y = max(Max(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y), pieceMax.y);
		pieceMax.z = max(Max(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z), pieceMax.z);
	}
	RigidBody* sourceBodyRef = allItems[srcPhysicalItemId].rigidBodyRef;
	RigidBody* destBodyRef = allItems[dstPhysicalItemId].rigidBodyRef;
	mat4 srcTrans = sourceBodyRef->orient;
	srcTrans.i[0][3] = sourceBodyRef->pos.x;
	srcTrans.i[1][3] = sourceBodyRef->pos.y;
	srcTrans.i[2][3] = sourceBodyRef->pos.z;
	mat4 dstTrans = destBodyRef->orient;
	dstTrans.i[0][3] = destBodyRef->pos.x;
	dstTrans.i[1][3] = destBodyRef->pos.y;
	dstTrans.i[2][3] = destBodyRef->pos.z;
	mat4 dstTransInv = dstTrans.Inv();
	mat4 srcTransDT = srcTrans.DirectionTransform();
	mat4 dstTransInvDT = dstTransInv.DirectionTransform();

	pieceMin = dstTrans * (pieceMin - vec3(0.15f));
	pieceMax = dstTrans * (pieceMax + vec3(0.15f));
	for (std::pair<const CACHED_BULLETHOLE_TYPE, CachedBulletHoleList>& curTypeBulletHoleListPair : bulletHoles)
		for (CachedBulletHoleList::CachedBulletHole& curHole : curTypeBulletHoleListPair.second.cachedBulletHoles)
		{
			if (curHole.itemId != srcPhysicalItemId) continue;
			vec3 srcHoleWorldPos = srcTrans * curHole.localPos;

			if (srcHoleWorldPos.x < pieceMin.x || srcHoleWorldPos.y < pieceMin.y || srcHoleWorldPos.z < pieceMin.z ||
				srcHoleWorldPos.x > pieceMax.x || srcHoleWorldPos.y > pieceMax.y || srcHoleWorldPos.z > pieceMax.z) continue;

			vec3 srcHoleWorldTan = (srcTransDT * curHole.localTan).normalized();
			vec3 srcHoleWorldNorm = (srcTransDT * curHole.localNorm).normalized();

			vec3 dstHoleLocalPos = dstTransInv * srcHoleWorldPos;
			vec3 dstHoleLocalTan = (dstTransInvDT * srcHoleWorldTan).normalized();
			vec3 dstHoleLocalNorm = (dstTransInvDT * srcHoleWorldNorm).normalized();

			curHole.localNorm = dstHoleLocalNorm;
			curHole.localTan = dstHoleLocalTan;
			curHole.localPos = dstHoleLocalPos;
			curHole.itemId = dstPhysicalItemId;
		}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::CacheBulletEmittersHoles(ParticleSystemClass& particleSystem, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission)
{
	if (cachedBulletEmitters) return;

	const int cacheEmitterBulletHoleMax = 10;
	for (int i = 0; i != cacheEmitterBulletHoleMax; i++)
	{
		particleSystem.AddBulletEmitter(vec3(0.0f, -400.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), inpMinMaxReducer, sdfBvhSubmission, rtSubmission, CONCRETE, true);
		particleSystem.AddBulletEmitter(vec3(0.0f, -400.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), inpMinMaxReducer, sdfBvhSubmission, rtSubmission, GLASS, true);
		particleSystem.AddBulletEmitter(vec3(0.0f, -400.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), inpMinMaxReducer, sdfBvhSubmission, rtSubmission, WOOD, true);
		particleSystem.AddBulletEmitter(vec3(0.0f, -400.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), inpMinMaxReducer, sdfBvhSubmission, rtSubmission, METALPLATFORM, true);
	}
	for (int i = 0; i != 4; i++)
	{
		CACHED_BULLETHOLE_TYPE bulletHoleType;
		std::string bulletholeMeshPath;
		switch (i)
		{
			case 0:
			{
				bulletHoleType = BULLETHOLE_GLASS;
				bulletholeMeshPath = "assets/models/decals/bullethole_glass.3md";
				break;
			}
			case 1:
			{
				bulletHoleType = BULLETHOLE_WOOD;
				bulletholeMeshPath = "assets/models/decals/bullethole_wood.3md";
				break;
			}
			case 2:
			{
				bulletHoleType = BULLETHOLE_METAL;
				bulletholeMeshPath = "assets/models/decals/bullethole_metal.3md";
				break;
			}
			default:
			{
				bulletHoleType = BULLETHOLE_CONCRETE;
				bulletholeMeshPath = "assets/models/decals/bullethole_concrete.3md";
				break;
			}
		}
		std::string belong = "assets/models/decals/";
		bulletHoles[bulletHoleType].meshModelRef = InsertEntry([&bulletholeMeshPath, &belong](Mesh& outMesh, GraphicsModel** outModel) {
			outMesh = std::move(Mesh(bulletholeMeshPath));
			*outModel = new GraphicsModel(outMesh, belong, Instance, [](int, DataGroup& inpGroup) -> bool {
				float tmpFloat;
				return (!Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowFloat(inpGroup, "PROPS", "rigidBody", tmpFloat));
				}, true);
			}, bulletholeMeshPath, std::string("physical items (loading bullet holes)"), cachedMeshesModels, cachedMeshesModelsMutex);
		mat4 tmpMat;
		tmpMat.Ident();
		tmpMat.i[0][3] = 0.0f;
		tmpMat.i[1][3] = -400.0f;
		tmpMat.i[2][3] = 0.0f;
		for (int i = 0; i != cacheEmitterBulletHoleMax; i++)
		{
			bulletHoles[bulletHoleType].cachedBulletHoles.push_back({
				bulletHoles[bulletHoleType].meshModelRef->model->CreateInstance(&tmpMat, true)
				});
			for (GroupedRenderSubmission* curSub : subList)
				perSubmissionCall(curSub, bulletHoles[bulletHoleType].cachedBulletHoles.back().modelInstRef);
		}
	}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ProcessFireLines(ParticleSystemClass& particleSystem, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission)
{
	using namespace HIGHOMEGA::ENTITIES;
	std::vector<FireLine> localFireLines;
	{
		std::lock_guard<std::mutex> lk(fireLineMutex);
		localFireLines = fireLines;
		fireLines.clear();
	}

	for (const FireLine& curFireLine : localFireLines)
	{
		vec3 linA = curFireLine.orig;
		vec3 linB = curFireLine.orig + curFireLine.dir * 1000.0f;
		bool hitSomething = false;
		vec3 hitNorm;
		RigidBody* hitBody = nullptr;
		ObjectPiece* hitPiece = nullptr;
		unsigned long long hitId;
		for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
		{
			if (!curItem.second.rigidBodyRef) continue;
			std::string groupId;
			if (LineMeshClosest(*curItem.second.rigidBodyRef, linA, linB, hitNorm, &hitPiece))
			{
				hitBody = curItem.second.rigidBodyRef;
				hitId = curItem.first;
				hitSomething = true;
			}
		}
		if (hitSomething)
		{
			CACHED_BULLETHOLE_TYPE bulletHoleType;
			std::string audioLoc = "assets/audio/sfx/impact/";
			if (hitPiece->mat == GLASS || hitPiece->mat == CERAMIC)
			{
				bulletHoleType = BULLETHOLE_GLASS;
				audioLoc += "glass.wav";
			}
			else if (hitPiece->mat == WOOD)
			{
				bulletHoleType = BULLETHOLE_WOOD;
				audioLoc += "wood.wav";
			}
			else if (hitPiece->mat == METALPLATFORM || hitPiece->mat == METALRODS)
			{
				bulletHoleType = BULLETHOLE_METAL;
				audioLoc += "metal.wav";
			}
			else
			{
				bulletHoleType = BULLETHOLE_CONCRETE;
				audioLoc += "concrete.wav";
			}

			AudioSystem.Insert(audioLoc, AudioSystemClass::SOUND_TYPE::SFX, linB, curFireLine.dir, 3.0f, false);

			if (hitBody->isMap && (hitPiece->shatterable || hitPiece->cuttable))
			{
				if (destructibleHitInfo.find(hitPiece->groupId) == destructibleHitInfo.end())
				{
					if (hitPiece->cuttable)
						destructibleHitInfo[hitPiece->groupId].hitClusters.emplace_back(linB, 1u);
					else
						destructibleHitInfo[hitPiece->groupId].shatterHitCount = 1u;
				}
				else
				{
					if (hitPiece->cuttable)
					{
						bool makingACut = false;
						for (std::vector<DestructibleHitCluster>::iterator it = destructibleHitInfo[hitPiece->groupId].hitClusters.begin(); it != destructibleHitInfo[hitPiece->groupId].hitClusters.end(); ++it)
						{
							if (((*it).center - linB).length() < hitPiece->cutRadius)
							{
								(*it).hitCount++;
								if ((*it).hitCount >= hitPiece->strength)
								{
									destructibleHitInfo[hitPiece->groupId].hitClusters.erase(it);
									makingACut = true;
									break;
								}
							}
						}
						if (makingACut) CutOut(hitId, hitPiece->groupId, MainFrustum.look * 20.0f, linB, hitNorm, hitPiece->cutRadius, hitPiece->cutDepth);
						else destructibleHitInfo[hitPiece->groupId].hitClusters.emplace_back(linB, 1u);
					}
					else
					{
						destructibleHitInfo[hitPiece->groupId].shatterHitCount++;
						if (destructibleHitInfo[hitPiece->groupId].shatterHitCount >= hitPiece->strength)
							Shatter(hitId, hitPiece->groupId, MainFrustum.look * 20.0f, linB, hitNorm);
					}
				}
			}

			if (hitNorm * curFireLine.dir > 0.0f) hitNorm = -hitNorm;
			hitNorm = hitNorm.normalized();
			vec3 tan = cross(hitNorm, hitNorm + vec3(0.1f)).normalized();
			tan = Spin(hitNorm, tan, (rand() % 1000) * 0.001f * HIGHOMEGA_PI * 2.0f);
			vec3 biTan = cross(tan, hitNorm);
			linB += hitNorm * 0.1f;
			mat4 hitBodyTrans = hitBody->orient;
			hitBodyTrans.i[0][3] = hitBody->pos.x;
			hitBodyTrans.i[1][3] = hitBody->pos.y;
			hitBodyTrans.i[2][3] = hitBody->pos.z;
			hitBody->EnqeueImpulse(curFireLine.dir * 10.0f, linB - hitNorm * 0.1f);
			mat4 hitBodyTransDT = hitBodyTrans.DirectionTransform();
			mat4 hitBodyInvTrans = hitBodyTrans.Inv();
			mat4 hitBodyTransDTInv = hitBodyTransDT.Inv();
			unsigned int pickedBulletHoleIdx = bulletHoles[bulletHoleType].currentHoleToPick;
			bulletHoles[bulletHoleType].cachedBulletHoles[pickedBulletHoleIdx].localPos = hitBodyInvTrans * linB;
			bulletHoles[bulletHoleType].cachedBulletHoles[pickedBulletHoleIdx].localTan = (hitBodyTransDTInv * tan).normalized();
			bulletHoles[bulletHoleType].cachedBulletHoles[pickedBulletHoleIdx].localNorm = (hitBodyTransDTInv * hitNorm).normalized();
			bulletHoles[bulletHoleType].cachedBulletHoles[pickedBulletHoleIdx].itemId = hitId;
			bulletHoles[bulletHoleType].currentHoleToPick = (bulletHoles[bulletHoleType].currentHoleToPick + 1) % bulletHoles[bulletHoleType].cachedBulletHoles.size();
			UpdateBulletHolesForItem(hitId, hitBodyTrans, hitBodyTransDT, true);
			particleSystem.AddBulletEmitter(linB, hitNorm, inpMinMaxReducer, sdfBvhSubmission, rtSubmission, hitPiece->mat);
		}
		clothCollection.IntersectRay(linA, linB);
	}
	for (ClothCollectionClass::TriHit& curTriHit : clothCollection.allTriHits)
	{
		particleSystem.AddBulletEmitter(curTriHit.pt, curTriHit.n, inpMinMaxReducer, sdfBvhSubmission, rtSubmission, CONCRETE);
		particleSystem.AddBulletEmitter(curTriHit.pt, -curTriHit.n, inpMinMaxReducer, sdfBvhSubmission, rtSubmission, CONCRETE);
	}
	clothCollection.allTriHits.clear();
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Combine(std::vector<PhysicalItemClass>& physicalItemClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, bool forceAddToWorld)
{
	physicalItemsCollection.subList = submissionList;
	physicalItemsCollection.perSubmissionCall = inpPerSubmissionCall;

	for (PhysicalItemClass& curPhysicalItemClass : physicalItemClasses)
	{
		allItems.insert(curPhysicalItemClass.allItems.begin(), curPhysicalItemClass.allItems.end());
		clothCollection.Combine(curPhysicalItemClass.clothCollection, submissionList, inpPerSubmissionCall);
		animatedMeshCollection.Combine(curPhysicalItemClass.animatedMeshCollection, submissionList, inpPerSubmissionCall);
		if (curPhysicalItemClass.physicalItemBuoyancyRangeCollection.ranges.size() > 0)
		{
			physicalItemBuoyancyRangeCollection.Combine(curPhysicalItemClass.physicalItemBuoyancyRangeCollection);
			resetPhysicsThreadBouyancyRangeCollection = true;
		}
		curPhysicalItemClass.allItems.clear();
	}

	if (forceAddToWorld) physicalItemsCollection.AddNewItemsToWorld(false);
}

void HIGHOMEGA::WORLD::PhysicalItemClass::UpdateSDFs(std::vector<PhysicalItemClass>& physicalItemClasses)
{
	std::vector<GraphicsModel *> buildingModels, rigidModels;
	std::vector<std::string> buildingCacheNames;
	buildingCacheNames.reserve(allItems.size());
	buildingModels.reserve(allItems.size());
	rigidModels.reserve(allItems.size());
	if (!RTInstance::Enabled())
	{
		for (PhysicalItemClass& curPhysicalItemClass : physicalItemClasses)
			for (std::pair<const unsigned long long, RigidBodyItem>& curItem : curPhysicalItemClass.allItems)
			{
				if (curItem.second.modelRef && !curItem.second.modelRef->sdf)
				{
					if (curItem.second.meshName != "")
					{
						buildingModels.push_back(curItem.second.modelRef);
						buildingCacheNames.push_back(curItem.second.zoneLocation + curItem.second.meshName + ".matdf");
					}
					else
						rigidModels.push_back(curItem.second.modelRef);
				}
				else if (curItem.second.meshModelRef && !curItem.second.meshModelRef->model->sdf && std::find(rigidModels.begin(), rigidModels.end(), curItem.second.meshModelRef->model) == rigidModels.end())
				{
					rigidModels.push_back(curItem.second.meshModelRef->model);
				}
			}
		if (rigidModels.size() > 0) GraphicsModel::UpdateSDFs(rigidModels); // SDF leaves for rigid bodies...
		if (buildingModels.size() > 0) GraphicsModel::UpdateSDFs(buildingModels, false, buildingCacheNames); // SDF leaves for buildings...
	}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::AddNewItemsToWorld(bool threaded)
{
	bool needToAddToWorld = false;
	for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
		if (!curItem.second.addedToWorld)
		{
			needToAddToWorld = true;
			break;
		}

	if (needToAddToWorld || resetPhysicsThreadBouyancyRangeCollection)
	{
		if (threaded)
		{
			CommonSharedMutex.unlock_shared();
			CommonSharedMutex.lock();
		}
		for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
		{
			if (curItem.second.addedToWorld) continue;
			if (curItem.second.rigidBodyRef) allBodies.push_back(curItem.second.rigidBodyRef);
			if (curItem.second.constaintsRef) constraintCollection.push_back(curItem.second.constaintsRef);
			if ((curItem.second.modelRef || curItem.second.meshModelRef) && !curItem.second.modelInstRef && subList.size() > 0)
			{
				curItem.second.modelInstRef = (curItem.second.modelRef ? *curItem.second.modelRef : *curItem.second.meshModelRef->model).CreateInstance(nullptr, curItem.second.rigidBodyRef ? (!curItem.second.rigidBodyRef->isMap) : true);
				for (GroupedRenderSubmission* curSub : subList)
					perSubmissionCall(curSub, curItem.second.modelInstRef);
			}
			curItem.second.addedToWorld = true;
		}
		if (resetPhysicsThreadBouyancyRangeCollection)
		{
			buoyancyRangeCollection = physicalItemBuoyancyRangeCollection;
			resetPhysicsThreadBouyancyRangeCollection = false;
		}
		if (threaded)
		{
			CommonSharedMutex.unlock();
			CommonSharedMutex.lock_shared();
		}
	}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::getVisibleMinMax(vec3& outMin, vec3& outMax)
{
	outMin = vec3(FLT_MAX), outMax = vec3(-FLT_MAX);
	vec3 curMin, curMax;
	for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
	{
		if ((!curItem.second.modelRef || curItem.second.modelRef->MaterialGeomMap.size() == 0) && (!curItem.second.meshModelRef || curItem.second.meshModelRef->model->MaterialGeomMap.size() == 0)) continue;
		(curItem.second.modelRef ? curItem.second.modelRef : curItem.second.meshModelRef->model)->getModelMinMax(curMin, curMax);
		if (curItem.second.modelInstRef) TransformCorners(curMin, curMax, curItem.second.modelInstRef->rootTransform);
		outMin.x = min(curMin.x, outMin.x);
		outMin.y = min(curMin.y, outMin.y);
		outMin.z = min(curMin.z, outMin.z);
		outMax.x = max(curMax.x, outMax.x);
		outMax.y = max(curMax.y, outMax.y);
		outMax.z = max(curMax.z, outMax.z);
	}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ClearContent()
{
	// Before we clear out the class, ensure there's no debris in flight for the system to process...
	if (booleanOpThread && !booleanOpThreadFinished)
	{
		booleanOpThread->join();
		delete booleanOpThread;
		booleanOpThread = nullptr;
		booleanOpThreadFinished = false;
		for (PhysicalItemClass::AddParams& curParams : deferredAdds)
			delete curParams.oobbPiece;
		deferredAdds.clear();
	}
	// ... also ensure we're not tessellating...
	if (staticTessellationThread && !staticTessellationFinished)
	{
		staticTessellationThread->join();
		delete staticTessellationThread;
		staticTessellationThread = nullptr;
		staticTessellationFinished = false;
	}

	for (std::pair<const unsigned long long, RigidBodyItem>& curItem : allItems)
	{
		RigidBodyItem& curRigidItem = curItem.second;
		if (curRigidItem.rigidBodyRef) delete curRigidItem.rigidBodyRef;

		if (curRigidItem.modelRef) delete curRigidItem.modelRef;
		if (curRigidItem.constaintsRef)
		{
			std::vector<ConstraintCollection*>::iterator it = std::find(constraintCollection.begin(), constraintCollection.end(), curRigidItem.constaintsRef);
			if (it != constraintCollection.end())
				constraintCollection.erase(it);
			delete curRigidItem.constaintsRef;
		}

		if (curRigidItem.meshModelRef) { std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); curRigidItem.meshModelRef->claims--; }
	}
	allItems.clear();
	for (std::pair<const CACHED_BULLETHOLE_TYPE, CachedBulletHoleList>& curTypeBulletHoleListPair : bulletHoles)
	{
		{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); curTypeBulletHoleListPair.second.meshModelRef->claims = 0u; }
		for (CachedBulletHoleList::CachedBulletHole& curBulletHole : curTypeBulletHoleListPair.second.cachedBulletHoles)
			curBulletHole.modelInstRef->modelRef->DestroyInstance(curBulletHole.modelInstRef);
	}
	bulletHoles.clear();
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);

	clothCollection.ClearContent();
	physicalItemBuoyancyRangeCollection.ClearContent();
	animatedMeshCollection.ClearContent();
	buoyancyRangeCollection.ClearContent(); // By this point the physics thread should be dead...
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Remove(unsigned long long curId)
{
	std::vector<unsigned long long> cachedAttachedBodies;
	std::vector<unsigned long long> cachedRagDolls;

	if (allItems.find(curId) != allItems.end())
	{
		RigidBodyItem& curRigidItem = allItems[curId];
		cachedAttachedBodies = curRigidItem.attachedBodies;
		cachedRagDolls = curRigidItem.ragdolls;
		if (curRigidItem.rigidBodyRef)
		{
			std::vector<RigidBody*>::iterator it = std::find(allBodies.begin(), allBodies.end(), curRigidItem.rigidBodyRef);
			if (it != allBodies.end())
				allBodies.erase(it);
		}
		if (curRigidItem.constaintsRef)
		{
			std::vector<ConstraintCollection*>::iterator it = std::find(constraintCollection.begin(), constraintCollection.end(), curRigidItem.constaintsRef);
			if (it != constraintCollection.end())
				constraintCollection.erase(it);
		}
		if (curRigidItem.rigidBodyRef)
		{
			{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
			for (ObjectPiece& curPiece : curRigidItem.rigidBodyRef->pieces)
			{
				if (destructibles.find(curPiece.groupId) != destructibles.end())
				{
					destructibles[curPiece.groupId].sourceModel = nullptr;
					destructibles[curPiece.groupId].sourceBody = nullptr;
				}
			}}
			delete curRigidItem.rigidBodyRef;
		}
		if (curRigidItem.constaintsRef) delete curRigidItem.constaintsRef;
		for (unsigned long long clothId : curRigidItem.clothsAndWindSources)
			clothCollection.Remove(clothId);
		for (unsigned long long buoyancyRange : curRigidItem.buoyancyRanges)
		{
			physicalItemBuoyancyRangeCollection.Remove(buoyancyRange);
			resetPhysicsThreadBouyancyRangeCollection = true;
		}

		if (curRigidItem.meshModelRef) { std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); curRigidItem.meshModelRef->claims--; }
		if (curRigidItem.modelInstRef) curRigidItem.modelInstRef->modelRef->DestroyInstance(curRigidItem.modelInstRef);
		RemoveBulletHolesForItem(curId);
		if (curRigidItem.modelRef) delete curRigidItem.modelRef;
		for (unsigned long long curId : curRigidItem.animatedMeshes)
			animatedMeshCollection.Remove(curId);
		allItems.erase(curId);
	}
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);

	for (unsigned long long curAttachedBody : cachedAttachedBodies)
		Remove(curAttachedBody);

	for (unsigned long long curRagDoll : cachedRagDolls)
		Remove(curRagDoll);
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::getTileNums(vec3 inpPos, int & tileX, int & tileY, int & tileZ)
{
	inpPos /= unitsPerTileEdge();
	tileX = (int)floor (inpPos.x);
	tileY = (int)floor (inpPos.y);
	tileZ = (int)floor (inpPos.z);
}

bool HIGHOMEGA::WORLD::ZoneStreamingClass::meshFromZoneDesc::operator==(const meshFromZoneDesc& other) const
{
	return this->name == other.name;
}

std::size_t HIGHOMEGA::WORLD::ZoneStreamingClass::MeshFromZoneDescHash::operator()(const meshFromZoneDesc& k) const
{
	return std::hash<std::string>{}(k.name);
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::produceZones(ZoneStreamingClass * zoneStreamingPtr, unsigned int threadId)
{
	try {
		mat4 tmpOrient;
		tmpOrient.Ident();
		vec3 tmpPos = vec3(0.0f);
		zoneStreamingPtr->threadProgressCounter[threadId] = 0u;
		for (unsigned int i = 0; i != zoneStreamingPtr->foundMeshesFromZones.size(); i++)
		{
			if (i % HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT == threadId) zoneStreamingPtr->threadProgressCounter[threadId] = i;
			if (i % HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT != threadId) continue;
			meshFromZoneDesc& foundMeshFromZone = zoneStreamingPtr->foundMeshesFromZones[i];
			bool foundZoneLoaded;
			{std::unique_lock<std::mutex> lk(zoneStreamingPtr->zone_producer_mutex);
			foundZoneLoaded = (zoneStreamingPtr->loadedItemsFromZoneMeshes.find(foundMeshFromZone) != zoneStreamingPtr->loadedItemsFromZoneMeshes.end()); }
			if (foundZoneLoaded) continue;

			float playerStandingHeight = player.physics.GetStandingHeight();
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> passThroughSubmit = [](GroupedRenderSubmission*, GraphicsModelInstance*) -> void { return; };
			Mesh* newMesh;
			unsigned long long physicaItemId = zoneStreamingPtr->physicalItemLoaders[threadId].AddZonePiece(zoneStreamingPtr->zoneLocation, foundMeshFromZone.name, &newMesh, zoneStreamingPtr->curPos, playerStandingHeight);
			unsigned long long plasteredId = zoneStreamingPtr->plasteredItemsLoaders[threadId].Populate(*newMesh, {}, passThroughSubmit, zoneStreamingPtr->perLoaderReducer[threadId]);
			unsigned long long particlesId = zoneStreamingPtr->particleSystemLoaders[threadId].Populate(*newMesh, {}, passThroughSubmit, zoneStreamingPtr->perLoaderReducer[threadId]);
			unsigned long long cameraRailId = zoneStreamingPtr->cameraSystemLoaders[threadId].Populate(*newMesh);
			unsigned long long guidedModelsId = zoneStreamingPtr->guidedModelLoaders[threadId].Populate(*newMesh, {}, passThroughSubmit);
			unsigned long long worldParamsId = zoneStreamingPtr->worldParamsLoaders[threadId].Populate(*newMesh);
			unsigned long long laddersId = zoneStreamingPtr->ladderSystemLoaders[threadId].Populate(*newMesh);
			{std::unique_lock<std::mutex> lk(zoneStreamingPtr->zone_producer_mutex);
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].physicalItemId = physicaItemId;
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].worldParamsId = worldParamsId;
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].plasteredId = plasteredId;
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].particlesId = particlesId;
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].guidedModelsId = guidedModelsId;
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].cameraRailId = cameraRailId;
			zoneStreamingPtr->loadedItemsFromZoneMeshes[foundMeshFromZone].laddersId = laddersId; }
			delete newMesh;
		}
		zoneStreamingPtr->plasteredItemsLoaders[threadId].Update(0.0f);
		zoneStreamingPtr->particleSystemLoaders[threadId].Update(0.0f, zoneStreamingPtr->perLoaderReducer[threadId], nullptr, nullptr);
		zoneStreamingPtr->perLoaderReducer[threadId].Process();
		if (RTInstance::Enabled()) zoneStreamingPtr->perLoaderReducer[threadId].ClearContent();
		// Comment out and use force refresh in main pipeline loop if you'd like frame-by-frame updates
		zoneStreamingPtr->plasteredItemsLoaders[threadId].UpdateSDFs();
		zoneStreamingPtr->particleSystemLoaders[threadId].UpdateSDFs();
		zoneStreamingPtr->guidedModelLoaders[threadId].UpdateSDFs();

		if (threadId == 0 && !RTInstance::Enabled())
		{
			while (true)
			{
				bool othersDone = true;
				for (int i = 1; i != HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT; i++)
					if (!zoneStreamingPtr->producedZones[i])
					{
						othersDone = false;
						break;
					}
				if (othersDone) break;
			}
			zoneStreamingPtr->physicalItemLoaders[threadId].UpdateSDFs(zoneStreamingPtr->physicalItemLoaders);
		}

		zoneStreamingPtr->producedZones[threadId] = true;
	}
	catch (std::runtime_error retErr) {
		LOG() << "Zone streaming thread: " << threadId << ", " << retErr.what();
		return ;
	}
	catch (const std::bad_alloc& e) {
		std::string outError = "Allocation failure: ";
		outError += e.what();
		LOG() << "Zone streaming thread: " << threadId << ", " << outError;
		return ;
	}
}

int HIGHOMEGA::WORLD::ZoneStreamingClass::tilesPerDrawRegionEdge()
{
	return 10;
}

float HIGHOMEGA::WORLD::ZoneStreamingClass::unitsPerTileEdge()
{
	return 100.0f;
}

bool HIGHOMEGA::WORLD::ZoneStreamingClass::allZonesProduced()
{
	for (int i = 0; i != HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT; i++)
		if (!producedZones[i]) return false;
	return true;
}

bool HIGHOMEGA::WORLD::ZoneStreamingClass::noZonesProduced()
{
	for (int i = 0; i != HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT; i++)
		if (producedZones[i]) return false;
	return true;
}

bool HIGHOMEGA::WORLD::ZoneStreamingClass::isZoneStreamingActivated()
{
	return zoneStreamingActivated;
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::setAllZonesNotProduced()
{
	for (int i = 0; i != HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT; i++)
		producedZones[i] = false;
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::waitOnZoneProduction()
{
	for (unsigned int i = 0; i != (unsigned int)zoneProducerThread.size(); i++)
	{
		zoneProducerThread[i]->join();
		delete zoneProducerThread[i];
	}
	zoneProducerThread.clear();
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::Create(std::string & inpZoneLocation, std::vector<GroupedRenderSubmission*> &&inpSubmissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, const std::function<void(float)>& progressUpdateCallback)
{
	zoneLocation = inpZoneLocation;
	submissionList = inpSubmissionList;
	perSubmissionCall = inpPerSubmissionCall;
	zoneStreamingActivated = true;

	Mesh zoneReferencesMesh = Mesh(inpZoneLocation + "zones.3md");
	DataBlock *zoneRefBlock, *worldBoundsBlock;

	if (!Mesh::getDataBlock(zoneReferencesMesh.DataGroups[0], "WORLDBOUNDS", &worldBoundsBlock))
		FATAL_ERROR("Could not get world bounds from zone ref 3md");
	if (worldBoundsBlock->rows.size() != 1 || worldBoundsBlock->rows[0].size() != 6)
		FATAL_ERROR("Malformed zone ref 3md (world bounds)");
	if (!worldBoundsBlock->rows[0][0].fvalue(globalMin.x) ||
		!worldBoundsBlock->rows[0][1].fvalue(globalMin.y) ||
		!worldBoundsBlock->rows[0][2].fvalue(globalMin.z) ||
		!worldBoundsBlock->rows[0][3].fvalue(globalMax.x) ||
		!worldBoundsBlock->rows[0][4].fvalue(globalMax.y) ||
		!worldBoundsBlock->rows[0][5].fvalue(globalMax.z))
		FATAL_ERROR("Malformed zone ref 3md (world bounds value fetch)");

	if (!Mesh::getDataBlock(zoneReferencesMesh.DataGroups[0], "DESCRIPTION", &zoneRefBlock))
		FATAL_ERROR("Could not get zone references from zone ref 3md");

	for (unsigned int i = 0; i != zoneRefBlock->rows.size(); i++)
	{
		std::string & zoneName = zoneRefBlock->rows[i][0].svalRef();
		int numRefs;
		if (!zoneRefBlock->rows[i][1].ivalue(numRefs))
			FATAL_ERROR("Malformed zone ref 3md");
		for (unsigned int j = 0; j != numRefs; j++)
			zoneReferences[zoneName].push_back(zoneRefBlock->rows[i][2 + j].svalRef());
	}

	curPos = vec3(0.0f);
	Update(curPos, true, progressUpdateCallback);
}

vec3 & HIGHOMEGA::WORLD::ZoneStreamingClass::getVisibleMax()
{
	return visibleMax;
}

vec3 & HIGHOMEGA::WORLD::ZoneStreamingClass::getVisibleMin()
{
	return visibleMin;
}

vec3& HIGHOMEGA::WORLD::ZoneStreamingClass::getGlobalMax()
{
	return globalMin;
}

vec3& HIGHOMEGA::WORLD::ZoneStreamingClass::getGlobalMin()
{
	return globalMax;
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::ForcePlayerFromPotentialGameLoad()
{
	if (zoneStreaming.setPlayerStatePostStream)
	{
		if (zoneStreaming.playerStatePostStream.climbingLadderName != "")
		{
			bool breakOut = false;
			for (std::pair <const unsigned long long, std::vector<HIGHOMEGA::ENTITIES::LadderSystemClass::Ladder>>& curLadderList : mainLadderSystem.ladders)
			{
				for (HIGHOMEGA::ENTITIES::LadderSystemClass::Ladder& curLadder : curLadderList.second)
				{
					if (zoneStreaming.playerStatePostStream.climbingLadderName == curLadder.name)
					{
						player.physics.ladderInfo.climbingLadder = &curLadder;
						player.physics.ladderInfo.transitionState = (CharacterPhysics::LADDER_TRANSITION_STATE)zoneStreaming.playerStatePostStream.ladderTransitionStateInt;
						player.physics.ladderInfo.transitionFraction = zoneStreaming.playerStatePostStream.ladderTransitionFraction;
						player.physics.ladderInfo.curEye = zoneStreaming.playerStatePostStream.ladderEye;
						player.physics.ladderInfo.curLook = zoneStreaming.playerStatePostStream.ladderLook;
						breakOut = true;
						break;
					}
				}
				if (breakOut) break;
			}
			if (!breakOut) FATAL_ERROR("Could not locate the ladder you're on from the save game");
		}
		else
		{
			player.physics.bodyPos = zoneStreaming.playerStatePostStream.pos;
			player.physics.playerVel = zoneStreaming.playerStatePostStream.vel;
			player.physics.lookVectorXZAngle = zoneStreaming.playerStatePostStream.lookAngles.x;
			player.physics.lookVectorYAngle = zoneStreaming.playerStatePostStream.lookAngles.y;
			player.physics.bodyStretch = zoneStreaming.playerStatePostStream.bodyStretch;
		}
		player.physics.quickWalk = false;

		worldParams.ForceSunAngle(zoneStreaming.playerStatePostStream.sunAngle);
		worldParams.UnforceSunAngle();

		zoneStreaming.setPlayerStatePostStream = false;
	}
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::Update(vec3 & inpPos, bool forceUpdate, const std::function<void(float)>& progressUpdateCallback)
{
	if (!zoneStreamingActivated) return;

	bool refreshedZones = false;
	if (!producingZones && noZonesProduced() && ((curPos - inpPos).length() > 100.0f || forceUpdate || zoneStreaming.setPlayerStatePostStream))
	{
		int curTileX, curTileY, curTileZ;
		curPos = zoneStreaming.setPlayerStatePostStream ? zoneStreaming.playerStatePostStream.pos : inpPos;
		if (zoneStreaming.setPlayerStatePostStream) producedZonesOnce = false;
		getTileNums(curPos, curTileX, curTileY, curTileZ);

		foundMeshesFromZones.clear();

		for (int i = -tilesPerDrawRegionEdge(); i != tilesPerDrawRegionEdge() + 1; i++)
			for (int j = -tilesPerDrawRegionEdge(); j != tilesPerDrawRegionEdge() + 1; j++)
				for (int k = -tilesPerDrawRegionEdge(); k != tilesPerDrawRegionEdge() + 1; k++)
				{
					std::string checkZoneName = std::to_string(curTileX + i);
					checkZoneName += std::string(".");
					checkZoneName += std::to_string(curTileY + j);
					checkZoneName += std::string(".");
					checkZoneName += std::to_string(curTileZ + k);
					if (zoneReferences.find(checkZoneName) != zoneReferences.end())
						for (unsigned int l = 0; l != zoneReferences[checkZoneName].size(); l++)
						{
							std::vector<meshFromZoneDesc>::iterator it = std::find_if(foundMeshesFromZones.begin(), foundMeshesFromZones.end(),
								[&zoneMeshNameRef = zoneReferences[checkZoneName][l]](const meshFromZoneDesc& x) {
									return x.name == zoneMeshNameRef;
								});
							if (it == foundMeshesFromZones.end())
							{
								foundMeshesFromZones.emplace_back();
								foundMeshesFromZones.back().name = zoneReferences[checkZoneName][l];
							}
						}
				}
		refreshedZones = true;
	}

	if (!producingZones && refreshedZones)
	{
		producingZones = true;
		setAllZonesNotProduced();
		zoneProducerThread.resize(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
		for (unsigned int i = 0; i != (unsigned int)zoneProducerThread.size(); i++)
			zoneProducerThread[i] = new std::thread(produceZones, this, i);
		if (!producedZonesOnce)
		{
			producedZonesOnce = true;
			while (!allZonesProduced())
			{
				unsigned int totalProgress = 0u;
				for (std::atomic<unsigned int>& curThreadProgress : threadProgressCounter)
					totalProgress += curThreadProgress;
				progressUpdateCallback(((float)totalProgress / (float)(foundMeshesFromZones.size() * HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT)) * 100.0f);
			}
			waitOnZoneProduction();
		}
		else return;
	}
	bool destructionLockAcquired = physicalItemsCollection.destructionMutex.try_lock();
	bool tessellationLockAcquired = physicalItemsCollection.staticTessellationMutex.try_lock();
	if (allZonesProduced() && destructionLockAcquired && tessellationLockAcquired) // Do not evict, mid destruction or mid tessellation...
	{
		waitOnZoneProduction();
		std::vector<meshFromZoneDesc> zoneMeshesToRemove;
		for (std::pair <const meshFromZoneDesc, addedItemsFromFoundZoneMesh> & curAddedItemsForZoneMesh : loadedItemsFromZoneMeshes)
		{
			bool curMeshFromZoneApproved = false;
			for (meshFromZoneDesc & foundMeshFromZone : foundMeshesFromZones)
				if (curAddedItemsForZoneMesh.first == foundMeshFromZone)
				{
					curMeshFromZoneApproved = true;
					break;
				}
			if (!curMeshFromZoneApproved)
				zoneMeshesToRemove.push_back(curAddedItemsForZoneMesh.first);
		}

		CommonSharedMutex.lock();
		for (meshFromZoneDesc& curZoneMeshToRemove : zoneMeshesToRemove)
		{
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].physicalItemId != 0ull) physicalItemsCollection.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].physicalItemId);
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].plasteredId != 0ull) plasteredItemsCollection.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].plasteredId);
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].particlesId != 0ull) particleSystem.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].particlesId);
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].guidedModelsId != 0ull) guidedModelSystem.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].guidedModelsId);
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].cameraRailId != 0ull) cameraSystem.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].cameraRailId);
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].worldParamsId != 0ull) worldParams.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].worldParamsId);
		}

		physicalItemsCollection.Combine(physicalItemLoaders, submissionList, perSubmissionCall, forceUpdate);
		cameraSystem.Combine(cameraSystemLoaders);
		plasteredItemsCollection.Combine(plasteredItemsLoaders, submissionList, perSubmissionCall);
		particleSystem.Combine(particleSystemLoaders, submissionList, perSubmissionCall);
		guidedModelSystem.Combine(guidedModelLoaders, submissionList, perSubmissionCall);
		worldParams.Combine(worldParamsLoaders);

		for (meshFromZoneDesc& curZoneMeshToRemove : zoneMeshesToRemove)
		{
			if (loadedItemsFromZoneMeshes[curZoneMeshToRemove].laddersId != 0ull) mainLadderSystem.Remove(loadedItemsFromZoneMeshes[curZoneMeshToRemove].laddersId);
		}
		mainLadderSystem.Combine(ladderSystemLoaders);
		zoneStreaming.ForcePlayerFromPotentialGameLoad();
		CommonSharedMutex.unlock();

		for (meshFromZoneDesc& curZoneMeshToRemove : zoneMeshesToRemove)
			loadedItemsFromZoneMeshes.erase(curZoneMeshToRemove);

		// Get visible min/max from all items on scene
		physicalItemsCollection.getVisibleMinMax(visibleMin, visibleMax);

		producingZones = false;
		setAllZonesNotProduced();

		physicalItemsCollection.destructionMutex.unlock();
		physicalItemsCollection.staticTessellationMutex.unlock();
	}
	else
	{
		if (destructionLockAcquired) physicalItemsCollection.destructionMutex.unlock();
		if (tessellationLockAcquired) physicalItemsCollection.staticTessellationMutex.unlock();
	}
}

bool HIGHOMEGA::WORLD::ZoneStreamingClass::isInDrawRegion(vec3 & inEye, vec3 & objPos, float objRad)
{
	int tileX, tileY, tileZ;
	getTileNums(inEye, tileX, tileY, tileZ);
	vec3 sideEdge = vec3 ((float)tilesPerDrawRegionEdge() * (float)unitsPerTileEdge());
	vec3 eyeMax = vec3((float)tileX, (float)tileY, (float)tileZ) * (float)unitsPerTileEdge() + sideEdge;
	vec3 eyeMin = vec3((float)tileX, (float)tileY, (float)tileZ) * (float)unitsPerTileEdge() - sideEdge;
	vec3 objMax = objPos + vec3(objRad);
	vec3 objMin = objPos - vec3(objRad);

	if (eyeMin.x > objMax.x) return false;
	if (eyeMax.x < objMin.x) return false;
	if (eyeMin.y > objMax.y) return false;
	if (eyeMax.y < objMin.y) return false;
	if (eyeMin.z > objMax.z) return false;
	if (eyeMax.z < objMin.z) return false;

	return true;
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::ClearContent()
{
	zoneStreamingActivated = false;
	zoneReferences.clear();
	foundMeshesFromZones.clear();
	producedZonesOnce = false;

	waitOnZoneProduction();

	producingZones = false;
	setAllZonesNotProduced();

	curPos = vec3(0.0f);
	zoneLocation = "";
	submissionList.clear();
	loadedItemsFromZoneMeshes.clear();
}

unsigned int HIGHOMEGA::WORLD::AnimatedMeshesClass::WorkGroupAnimateX()
{
	return 32;
}

unsigned long long HIGHOMEGA::WORLD::AnimatedMeshesClass::Add(std::string animatedModelFile, std::vector<AnimatedModelItem::refAnim>&& inRefAnims, std::string animatedModelFileLoc, float duration, bool rollOver, mat4 & inpTransMat, std::vector<RagDollPiece>* inpRagDollPieces, bool autoAdvance)
{
	MeshModel* retMeshModel = InsertEntry([&animatedModelFile, &animatedModelFileLoc](Mesh& outMesh, GraphicsModel** outModel) {
		outMesh = std::move(Mesh(animatedModelFile));
		*outModel = new GraphicsModel(outMesh, animatedModelFileLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
			float tmpFloat;
			std::string tmpString;
			return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowString(inpGroup, "PROPS", "ragDollPieceTransform", tmpString);
		}, true, true);
	}, animatedModelFile, std::string("animated models"), cachedMeshesModels, cachedMeshesModelsMutex);

	unsigned long long animationId = threadSafeMersenneTwister64Bit();
	AnimatedModelItem & newModel = allAnimatedModelItems[animationId];
	newModel.refAnims = std::move(inRefAnims);
	for (AnimatedModelItem::refAnim& curAnim : newModel.refAnims)
	{
		curAnim.meshModelRef = InsertEntry([refAnimKey = curAnim.key, &animatedModelFileLoc](Mesh& outMesh, GraphicsModel** outModel) {
			outMesh = std::move(Mesh(refAnimKey));
			*outModel = new GraphicsModel(outMesh, animatedModelFileLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
				float tmpFloat;
				std::string tmpString;
				return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowString(inpGroup, "PROPS", "ragDollPieceTransform", tmpString);
				}, true, true, nullptr, nullptr, true);
			}, curAnim.key, std::string("animated models"), cachedMeshesModels, cachedMeshesModelsMutex);
	}
	newModel.useRefAnim = -1;
	if (inpRagDollPieces)
	{
		newModel.ragDollPieces = *inpRagDollPieces;
		newModel.ragDoll = true;
	}
	else
		newModel.ragDoll = false;
	newModel.autoAdvance = autoAdvance;
	newModel.rollOver = rollOver;
	newModel.meshModelKey = animatedModelFile;
	newModel.meshModelRef = retMeshModel;
	newModel.duration = duration;
	newModel.curAnimationPos = 0.0f;
	newModel.animatedModel = new GraphicsModel(newModel.meshModelRef->mesh, animatedModelFileLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		std::string tmpString;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowString(inpGroup, "PROPS", "ragDollPieceTransform", tmpString);
	}, false, true);
	newModel.transMat = inpTransMat;

	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = newModel.meshModelRef->model->MaterialGeomMap;
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & destMaterialGeomMap = newModel.animatedModel->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = destMaterialGeomMap.begin(); it != destMaterialGeomMap.end(); ++it)
	{
		for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
			newModel.sourceGeom.push_back(&(*it2));
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			newModel.destGeom.push_back(&(*it2));
	}

	rerecordSubmission = true;

	return animationId;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::GetCurPosDuration(unsigned long long animationId, int useRefAnim, float & pos, float & duration)
{
	if (allAnimatedModelItems.find(animationId) == allAnimatedModelItems.end()) return;
	if (useRefAnim != -1 && allAnimatedModelItems[animationId].refAnims.size() <= useRefAnim)  FATAL_ERROR("reference animation not found when trying to set animation state");

	pos = allAnimatedModelItems[animationId].curAnimationPos;
	if (useRefAnim == -1)
		duration = allAnimatedModelItems[animationId].duration;
	else
		duration = allAnimatedModelItems[animationId].refAnims[useRefAnim].duration;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::Remove(unsigned long long animationId)
{
	if (allAnimatedModelItems.find(animationId) == allAnimatedModelItems.end()) return;

	AnimatedModelItem & curAnimatedModel = allAnimatedModelItems[animationId];
	delete curAnimatedModel.animatedModel;

	{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); cachedMeshesModels[curAnimatedModel.meshModelKey].claims--;
	for (AnimatedModelItem::refAnim& curRefAnim: curAnimatedModel.refAnims) cachedMeshesModels[curRefAnim.key].claims--; }

	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);

	allAnimatedModelItems.erase(animationId);

	rerecordSubmission = true;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::Process(float elapseTime)
{
	for (std::pair <const unsigned long long, AnimatedModelItem>& curAnimatedModelKV : allAnimatedModelItems)
	{
		AnimatedModelItem & curAnimatedModel = curAnimatedModelKV.second;
		if (subList.size() > 0 && !curAnimatedModel.animatedModelInst)
		{
			curAnimatedModel.animatedModelInst = curAnimatedModel.animatedModel->CreateInstance();
			for (GroupedRenderSubmission* curSubmission : subList)
				perSubmissionCall(curSubmission, curAnimatedModel.animatedModelInst);
		}
		curAnimatedModel.animatedModelInst->Update(curAnimatedModel.transMat);

		if (curAnimatedModel.autoAdvance) curAnimatedModel.curAnimationPos += elapseTime;

		bool hasRefAnim = curAnimatedModel.useRefAnim != -1;
		float animationDuration = hasRefAnim ? curAnimatedModel.refAnims[curAnimatedModel.useRefAnim].duration : curAnimatedModel.duration;

		if (!curAnimatedModel.rollOver)
			curAnimatedModel.curAnimationPos = Clamp (curAnimatedModel.curAnimationPos, 0.0f, animationDuration);
		else
			if (curAnimatedModel.curAnimationPos > 0.0f)
				curAnimatedModel.curAnimationPos = std::fmodf(curAnimatedModel.curAnimationPos, animationDuration);
			else
				curAnimatedModel.curAnimationPos = animationDuration + std::fmodf(curAnimatedModel.curAnimationPos, animationDuration);

		float maxBoneScale = 0.0f;
		vec3 animatedInstMin = vec3(FLT_MAX), animatedInstMax = vec3(-FLT_MAX);
		for (std::pair<const std::string, AnimationAndSnapshot> & armNameToCurAnimSnapshot : curAnimatedModel.animatedModel->armatures)
		{
			AnimationAndSnapshot & animAndSnapshotRef = hasRefAnim ? curAnimatedModel.refAnims[curAnimatedModel.useRefAnim].meshModelRef->model->armatures[armNameToCurAnimSnapshot.first] : armNameToCurAnimSnapshot.second;
			if (curAnimatedModel.ragDoll)
			{
				for (RagDollPiece & ragDollPiece : curAnimatedModel.ragDollPieces)
				{
					for (int i = 0; i != animAndSnapshotRef.currentPose.pose.size(); i++)
					{
						if (ragDollPiece.transformTarget != animAndSnapshotRef.currentPose.pose[i].name) continue;
						mat4 curTrans = ragDollPiece.piece->orient;
						curTrans.i[0][3] = ragDollPiece.piece->pos.x;
						curTrans.i[1][3] = ragDollPiece.piece->pos.y;
						curTrans.i[2][3] = ragDollPiece.piece->pos.z;
						animAndSnapshotRef.currentPose.pose[i].bone = curTrans * ragDollPiece.refMatInv;
						// Very precise approach to compute ragdoll AABB
						animatedInstMin = vec3(min(animatedInstMin.x, ragDollPiece.piece->body_min.x), min(animatedInstMin.y, ragDollPiece.piece->body_min.y), min(animatedInstMin.z, ragDollPiece.piece->body_min.z));
						animatedInstMax = vec3(max(animatedInstMax.x, ragDollPiece.piece->body_max.x), max(animatedInstMax.y, ragDollPiece.piece->body_max.y), max(animatedInstMax.z, ragDollPiece.piece->body_max.z));
						break;
					}
				}
			}
			else
			{
				animAndSnapshotRef.anim.GetPose(curAnimatedModel.curAnimationPos / animationDuration, animAndSnapshotRef.currentPose);
				animAndSnapshotRef.currentPose.Mul(hasRefAnim ? armNameToCurAnimSnapshot.second.referencePoseInv : animAndSnapshotRef.referencePoseInv);
				for (int i = 0; i != animAndSnapshotRef.currentPose.pose.size(); i++)
				{
					mat4 curBoneNoTran = animAndSnapshotRef.currentPose.pose[i].bone;
					maxBoneScale = max(maxBoneScale, vec3(curBoneNoTran.i[0][0], curBoneNoTran.i[1][0], curBoneNoTran.i[2][0]).length());
					maxBoneScale = max(maxBoneScale, vec3(curBoneNoTran.i[0][1], curBoneNoTran.i[1][1], curBoneNoTran.i[2][1]).length());
					maxBoneScale = max(maxBoneScale, vec3(curBoneNoTran.i[0][2], curBoneNoTran.i[1][2], curBoneNoTran.i[2][2]).length());
				}
			}
			(hasRefAnim ? armNameToCurAnimSnapshot.second : animAndSnapshotRef).CopyToPoseBuffer(animAndSnapshotRef.currentPose);
		}
		if (!curAnimatedModel.ragDoll)
		{
			// Not the best approach to computing skinned mesh AABB, but does the job for now
			curAnimatedModel.meshModelRef->model->getModelMinMax(animatedInstMin, animatedInstMax);
			animatedInstMax = vec3(max(fabs(animatedInstMax.x), fabs(animatedInstMin.x)), max(fabs(animatedInstMax.y), fabs(animatedInstMin.y)), max(fabs(animatedInstMax.z), fabs(animatedInstMin.z)));
			animatedInstMax *= maxBoneScale;
			animatedInstMin = -animatedInstMax; // This allows for full off-center swings of the object to be accounted for
		}
		if (curAnimatedModel.animatedModelInst) curAnimatedModel.animatedModelInst->SetMinMax(animatedInstMin, animatedInstMax);
		curAnimatedModel.animatedModel->SetDirty();
	}

	if (rerecordSubmission)
	{
		unsigned int maxVerts = 0u;
		std::vector<animateParamsStruct> animateParams;
		std::vector<ShaderResource> allAnimInfoBufs;
		std::vector<ShaderResource> allPoseBufs;
		for (std::pair <const unsigned long long, AnimatedModelItem>& curAnimatedModelKV : allAnimatedModelItems)
		{
			AnimatedModelItem& curModel = curAnimatedModelKV.second;
			for (unsigned int i = 0; i != curModel.sourceGeom.size(); i++)
			{
				animateParamsStruct curParam;
				curParam.vertCount = curModel.sourceGeom[i]->getVertCount();
				maxVerts = max(maxVerts, curParam.vertCount);
				curParam.srcIdxVertOffset = curModel.sourceGeom[i]->getDataOffsetInGiantVertexBuffer();
				curParam.dstIdxVertOffset = curModel.destGeom[i]->getDataOffsetInGiantVertexBuffer();
				BufferClass* currentAndPrevPoseBuf = curModel.animatedModel->armatures[curModel.sourceGeom[i]->getArmatureId()].currentAndPrevPoseBuf;
				curParam.nBones = currentAndPrevPoseBuf->getSize() / (unsigned int)(sizeof(float) * 16 * 2);
				animateParams.push_back(curParam);
				allAnimInfoBufs.push_back(ShaderResource(RESOURCE_SSBO, COMPUTE, 1, 0, curModel.sourceGeom[i]->getAnimInfoBuf()));
				allPoseBufs.push_back(ShaderResource(RESOURCE_SSBO, COMPUTE, 2, 0, *currentAndPrevPoseBuf));
			}
		}

		if (animateSubmission) delete animateSubmission;
		if (animateParamsBuf) delete animateParamsBuf;
		if (animateShader) delete animateShader;
		animateSubmission = nullptr;
		animateShader = nullptr;
		animateParamsBuf = nullptr;

		if (maxVerts > 0)
		{
			animateParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, animateParams.data(), (unsigned int)(animateParams.size() * sizeof(animateParamsStruct)));
			animateShader = new ShaderResourceSet;
			animateSubmission = new ComputeSubmission;
			giantVertBufferSharedMutex.lock_shared();
			animateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			animateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, *(animateParamsBuf), ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			animateShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, allAnimInfoBufs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			animateShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, allPoseBufs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			animateShader->Create("shaders/transformAnimated.comp.spv", "main");
			animateSubmission->MakeDispatch(Instance, "animatedTransformDispatch", *animateShader, (unsigned int)ceil((double)maxVerts / (double)WorkGroupAnimateX()), (unsigned int)animateParams.size(), 1);
		}
		rerecordSubmission = false;
	}

	if (animateSubmission)
		(*animateSubmission).MakeAsync().Submit();
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::SetAnimationState(unsigned long long animationId, int useRefAnim, float animationPos, mat4& inpTransMat)
{
	if (allAnimatedModelItems.find(animationId) == allAnimatedModelItems.end()) return;
	if (useRefAnim != -1 && allAnimatedModelItems[animationId].refAnims.size() <= useRefAnim)  FATAL_ERROR("reference animation not found when trying to set animation state");

	allAnimatedModelItems[animationId].curAnimationPos = animationPos;
	allAnimatedModelItems[animationId].transMat = inpTransMat;
	allAnimatedModelItems[animationId].useRefAnim = useRefAnim;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::Combine(AnimatedMeshesClass& animatedMeshes, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall)
{
	if (!animateSubmission) animateSubmission = new ComputeSubmission;

	subList = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	if (animatedMeshes.animateSubmission) delete animatedMeshes.animateSubmission;
	if (animatedMeshes.animateShader) delete animatedMeshes.animateShader;
	if (animatedMeshes.animateParamsBuf) delete animatedMeshes.animateParamsBuf;
	animatedMeshes.animateSubmission = nullptr;
	animatedMeshes.animateShader = nullptr;
	animatedMeshes.animateParamsBuf = nullptr;

	for (std::pair<const unsigned long long, AnimatedModelItem>& curAnimatedModel : animatedMeshes.allAnimatedModelItems)
		allAnimatedModelItems[curAnimatedModel.first] = curAnimatedModel.second;
	animatedMeshes.allAnimatedModelItems.clear();

	rerecordSubmission = true;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::ClearContent()
{
	for (std::pair <const unsigned long long, AnimatedModelItem>& curAnimatedModelKV : allAnimatedModelItems)
	{
		AnimatedModelItem & curAnimatedModel = curAnimatedModelKV.second;
		delete curAnimatedModel.animatedModel;

		{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); cachedMeshesModels[curAnimatedModel.meshModelKey].claims--;
		for (AnimatedModelItem::refAnim& curRefAnim : curAnimatedModel.refAnims) cachedMeshesModels[curRefAnim.key].claims--; }
	}
	allAnimatedModelItems.clear();
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);

	if (animateSubmission) delete animateSubmission;
	if (animateShader) delete animateShader;
	if (animateParamsBuf) delete animateParamsBuf;
	animateSubmission = nullptr;
	animateShader = nullptr;
	animateParamsBuf = nullptr;

	rerecordSubmission = true;
}

// Should *only* be used after an AddEmitter(...) call. Otherwise will leak.
void HIGHOMEGA::WORLD::ParticleSystemClass::ParticleEmitter::ClearForReuse()
{
	sourceGeom.clear();
	destGeom.clear();
	particlesBuf = nullptr;
	integrateParamsBuf = nullptr;
	transformParamsBuf = nullptr;
	particles.clear();
}

void HIGHOMEGA::WORLD::ParticleSystemClass::ParticleEmitter::Populate()
{
	particles.clear();
	for (unsigned int i = 0; i != integrateParams.numParticles; i++)
	{
		ParticleItem curItem;

		vec3 emitterPos = vec3(integrateParams.posLinSpeedBase[0], integrateParams.posLinSpeedBase[1], integrateParams.posLinSpeedBase[2]);
		vec3 initParticlePos = emitterPos + cross(emitDir, randNormVec()).normalized() * integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[0];

		vec3 planeNormal = randNormVec();
		curItem.pos[0] = initParticlePos.x;
		curItem.pos[1] = initParticlePos.y;
		curItem.pos[2] = initParticlePos.z;
		curItem.velocityDir = toZSignXY((emitDir + spread * randNormVec()).normalized());
		curItem.xAxis = toZSignXY(planeNormal);
		curItem.yAxis = toZSignXY(cross(planeNormal, randNormVec()).normalized());
		curItem.alpha = (unsigned char)(randomLives ? 1.0f : (Clamp(integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2], 0.0f, 1.0f) * 255.0f));
		curItem.health = (unsigned char)(randomLives ? (float)(rand() % 255) : 255.0f);
		curItem.fadeRate = (unsigned char)(Clamp(integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] + integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[0] * randFract(), 0.0f, 1.0f) * 255.0f);
		curItem.deathRate = (unsigned char)(Clamp(integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[3] + integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0] * randFract(), 0.0f, 1.0f) * 255.0f);
		curItem.scaleLinSpeed = toFP16(vec2(integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1], integrateParams.posLinSpeedBase[3] + integrateParams.dirGravDirGravLenLinSpeedVar[3] * randFract()));
		curItem.scaleRate = toFP16(integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[1] + integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[2] * randFract());
		curItem.flags = (unsigned char)(scaleToLinVel ? 1 : 0);
		curItem.prevDeltaT = (unsigned char)(1);
		curItem.angSpeed = integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] + integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[2] * randFract();

		particles.push_back(curItem);
	}
}

void HIGHOMEGA::WORLD::ParticleSystemClass::ParticleEmitter::AttemptHideEmitter()
{
	if (reUseTag != REUSE_NONE && !hidden && hideTimer.Diff() > hideAfterTime)
	{
		integrateParams.posLinSpeedBase[0] = hidePos.x;
		integrateParams.posLinSpeedBase[1] = hidePos.y;
		integrateParams.posLinSpeedBase[2] = hidePos.z;

		Populate();
		particlesBuf->UploadSubData(0, particles.data(), (unsigned int)(particles.size() * sizeof(ParticleItem)));

		hidden = true;
	}
}

vec3 HIGHOMEGA::WORLD::ParticleSystemClass::randNormVec()
{
	return vec3((rand() % 100 - 50) * 0.02f, (rand() % 100 - 50) * 0.02f, (rand() % 100 - 50) * 0.02f).normalized();
}

float HIGHOMEGA::WORLD::ParticleSystemClass::randFract()
{
	return (rand() % 1000 - 500) * 0.002f;
}

unsigned int HIGHOMEGA::WORLD::ParticleSystemClass::WorkGroupTransformX()
{
	return 8;
}

unsigned int HIGHOMEGA::WORLD::ParticleSystemClass::WorkGroupTransformY()
{
	return 4;
}

unsigned int HIGHOMEGA::WORLD::ParticleSystemClass::WorkGroupIntegrateX()
{
	return 32;
}

unsigned long long HIGHOMEGA::WORLD::ParticleSystemClass::Populate(Mesh & inpMesh, std::vector <GroupedRenderSubmission*> && subList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducer)
{
	submissionsForParticle = subList;
	perSubmissionCall = inpPerSubmissionCall;
	float tmpFloat;

	unsigned long long emittersId = threadSafeMersenneTwister64Bit();

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		for (int j = 1;; j++)
		{
			ParticleEmitter curEmitter;
			std::string jAsString = std::to_string(j);
			float particleCountFloat;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleCount" + jAsString, particleCountFloat)) break;
			unsigned int particleCount = (unsigned int)particleCountFloat;

			vec3 emitPos, emitDir;
			float emitRad;

			if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", emitPos) ||
				 !Mesh::getDataRowFloat(curPolyGroup, "DESCRIPTION", "dist", emitRad) ||
				 !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", emitDir) ) FATAL_ERROR("Could not get emitter pos/dir/dist");

			curEmitter.integrateParams.posLinSpeedBase[0] = emitPos.x;
			curEmitter.integrateParams.posLinSpeedBase[1] = emitPos.y;
			curEmitter.integrateParams.posLinSpeedBase[2] = emitPos.z;
			curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[0] = emitRad;
			unsigned int emitDirEncoded = toZSignXY(emitDir);
			unsigned int gravDir = toZSignXY(GravDir.normalized());
			curEmitter.emitDir = emitDir;
			curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[0] = *((float*)&emitDirEncoded);
			curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[1] = *((float*)&gravDir);
			curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[2] = Grav;

			std::string particleMesh, particleMeshAssetLoc;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "particleMesh" + jAsString, particleMesh) ||
				 !Mesh::getDataRowString(curPolyGroup, "PROPS", "particleMeshAssetLoc" + jAsString, particleMeshAssetLoc) ) FATAL_ERROR("Could not get particle mesh and mesh loc");

			// Property fetching and defaults 
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleLinVelBase" + jAsString, curEmitter.integrateParams.posLinSpeedBase[3])) curEmitter.integrateParams.posLinSpeedBase[3] = 10.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleLinVelVar" + jAsString, curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[3])) curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[3] = 1.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleAngVelBase" + jAsString, curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1])) curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] = 0.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleAngVelVar" + jAsString, curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[2])) curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[2] = 0.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleFadeRateBase" + jAsString, curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3])) curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] = 0.95f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleFadeRateVar" + jAsString, curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[0])) curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[0] = 0.01f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleScaleRateBase" + jAsString, curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[1])) curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[1] = 0.99f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleScaleRateVar" + jAsString, curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[2])) curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[2] = 0.01f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleDeathRateBase" + jAsString, curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[3])) curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[3] = 0.95f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleDeathRateVar" + jAsString, curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0])) curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0] = 0.01f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleInitScale" + jAsString, curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1])) curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 3.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleInitAlpha" + jAsString, curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2])) curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2] = 1.0f;
			curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[3] = 0.01f;

			curEmitter.integrateParams.numParticles = particleCount;
			curEmitter.originString = std::string("_") + std::to_string(i) + std::string("_") + std::to_string(j);
			curEmitter.particleMesh = particleMesh;
			curEmitter.particleMeshAssetLoc = particleMeshAssetLoc;
			curEmitter.emittersId = emittersId;
			curEmitter.requiresScene = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleRequiresScene" + jAsString, tmpFloat);
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleElasticity" + jAsString, curEmitter.elasticity) ) curEmitter.elasticity = 0.5f;

			curEmitter.integrateParams.elasticitySpreadFlags = 0u;
			EncodeElasticity(curEmitter);
			EncodeAffectedByGravity(curEmitter, Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleAffectedByGravity" + jAsString, tmpFloat));
			EncodeOrientToLinVel(curEmitter, Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleOrientToLinVel" + jAsString, tmpFloat));
			curEmitter.scaleToLinVel = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleScaleToLinVel" + jAsString, tmpFloat);
			EncodeScaleToLinVel(curEmitter, curEmitter.scaleToLinVel);
			if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleSpread" + jAsString, curEmitter.spread)) curEmitter.spread = 0.0f;
			EncodeSpread(curEmitter, curEmitter.spread);
			EncodeRepeat(curEmitter, true);
			curEmitter.randomLives = true;

			// These cannot be initialized off the main rendering thread... the right scene isn't present here
			if (curEmitter.requiresScene)
			{
				sceneInteractingEmitters.push_back(curEmitter);
				continue;
			}

			AddEmitter(curEmitter, inpMinMaxReducer, nullptr, nullptr);
		}
	}
	return emittersId;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::EncodeElasticity(ParticleEmitter& inpEmitter)
{
	inpEmitter.integrateParams.elasticitySpreadFlags |= ((unsigned int)(Clamp(inpEmitter.elasticity, 0.0f, 1.0f) * 255.0f) << 24u);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::EncodeAffectedByGravity(ParticleEmitter& inpEmitter, bool affectedByGravity)
{
	if (affectedByGravity) inpEmitter.integrateParams.elasticitySpreadFlags |= 1u;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::EncodeSpread(ParticleEmitter& inpEmitter, float spreadAmount)
{
	inpEmitter.integrateParams.elasticitySpreadFlags |= ((unsigned char)(Clamp (spreadAmount, 0.0f, 1.0f) * 255.0f) << 16u);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::EncodeOrientToLinVel(ParticleEmitter& inpEmitter, bool orientToLinVel)
{
	if (orientToLinVel) inpEmitter.integrateParams.elasticitySpreadFlags |= 2u;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::EncodeScaleToLinVel(ParticleEmitter& inpEmitter, bool scaleToLinVel)
{
	if (scaleToLinVel) inpEmitter.integrateParams.elasticitySpreadFlags |= 4u;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::EncodeRepeat(ParticleEmitter& inpEmitter, bool repeat)
{
	if (repeat) inpEmitter.integrateParams.elasticitySpreadFlags |= 8u;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::AddEmitter(ParticleEmitter& inpEmitter, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission)
{
	MeshModel* retMeshModel = InsertEntry([&inpEmitter](Mesh& outMesh, GraphicsModel** outModel) {
		outMesh = std::move(Mesh(inpEmitter.particleMesh));
		*outModel = new GraphicsModel(outMesh, inpEmitter.particleMeshAssetLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
			float tmpFloat;
			return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
		}, true);
	}, inpEmitter.particleMesh, std::string("particle meshes"), cachedMeshesModels, cachedMeshesModelsMutex);

	inpEmitter.collectionModel = new GraphicsModel(retMeshModel->mesh, inpEmitter.particleMeshAssetLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
		float tmpFloat;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
		}, false);
	inpEmitter.collectionModel->BlankAndResize(inpEmitter.integrateParams.numParticles);
	if (inpEmitter.forceMinMax) inpEmitter.collectionModel->SetMinMax(inpEmitter.forcedMin, inpEmitter.forcedMax);

	inpEmitter.Populate();

	ComputeSubmission *integrateRef;
	if (inpEmitter.requiresScene) // This better be happening on the main rendering thread...
	{
		if (!integrateAgainstSceneSubmission) integrateAgainstSceneSubmission = new ComputeSubmission;
		integrateRef = integrateAgainstSceneSubmission;
	}
	else
	{
		if (!integrateSubmission) integrateSubmission = new ComputeSubmission;
		integrateRef = integrateSubmission;
	}
	if (!transformCollectionSubmission) transformCollectionSubmission = new ComputeSubmission;

	inpEmitter.particlesBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, inpEmitter.particles.data(), (unsigned int)(inpEmitter.particles.size() * sizeof(ParticleItem)));
	inpEmitter.integrateParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, &inpEmitter.integrateParams, (unsigned int)sizeof(inpEmitter.integrateParams));

	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash>& collectionMaterialGeomMap = inpEmitter.collectionModel->MaterialGeomMap;
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash>& sourceMaterialGeomMap = retMeshModel->model->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = collectionMaterialGeomMap.begin(); it != collectionMaterialGeomMap.end(); ++it)
	{
		for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
			inpEmitter.sourceGeom.push_back(&(*it2));
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			inpEmitter.destGeom.push_back(&(*it2));
	}

	std::vector<transformParamsStruct> transformParams;
	for (unsigned int i = 0; i != inpEmitter.sourceGeom.size(); i++)
	{
		transformParamsStruct curParam;
		curParam.vertCount = inpEmitter.sourceGeom[i]->getVertCount();
		curParam.instanceCount = inpEmitter.integrateParams.numParticles;
		curParam.geomInstCount = (unsigned int)inpEmitter.sourceGeom.size();
		curParam.srcIdxVertOffset = inpEmitter.sourceGeom[i]->getDataOffsetInGiantVertexBuffer();
		curParam.dstIdxVertOffset = inpEmitter.destGeom[i]->getDataOffsetInGiantVertexBuffer();
		transformParams.push_back(curParam);

		if (!inpEmitter.forceMinMax) inpMinMaxReducer.RequestMinMax(inpEmitter.destGeom[i]);
	}

	inpEmitter.transformParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, transformParams.data(), (unsigned int)(transformParams.size() * sizeof(transformParamsStruct)));

	rerecordSubmission = true;

	allEmitters[inpEmitter.emittersId].push_back(inpEmitter);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::RecycleEmitter(ParticleEmitter& inpEmitter, minMaxReductionClass& inpMinMaxReducer)
{
	if (inpEmitter.reUseTag == REUSE_NONE) return;
	float maxTimeAlive = 0.0f;
	ParticleEmitter* chosenEmitter = nullptr;
	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
		for (ParticleEmitter& curEmitter : curEmittersIdPair.second)
			if (curEmitter.reUseTag == inpEmitter.reUseTag && (curEmitter.hidden || curEmitter.hideTimer.Diff() > maxTimeAlive))
			{
				maxTimeAlive = (float)curEmitter.hideTimer.Diff();
				chosenEmitter = &curEmitter;
			}
	if (chosenEmitter)
	{
		chosenEmitter->integrateParams.posLinSpeedBase[0] = inpEmitter.integrateParams.posLinSpeedBase[0];
		chosenEmitter->integrateParams.posLinSpeedBase[1] = inpEmitter.integrateParams.posLinSpeedBase[1];
		chosenEmitter->integrateParams.posLinSpeedBase[2] = inpEmitter.integrateParams.posLinSpeedBase[2];

		unsigned int emitDirEncoded = toZSignXY(inpEmitter.emitDir);
		unsigned int gravDir = toZSignXY(GravDir.normalized());
		chosenEmitter->emitDir = inpEmitter.emitDir;
		chosenEmitter->integrateParams.dirGravDirGravLenLinSpeedVar[0] = *((float*)&emitDirEncoded);
		chosenEmitter->integrateParams.dirGravDirGravLenLinSpeedVar[1] = *((float*)&gravDir);
		chosenEmitter->integrateParams.dirGravDirGravLenLinSpeedVar[2] = Grav;

		chosenEmitter->forceMinMax = inpEmitter.forceMinMax;
		if (chosenEmitter->forceMinMax)
		{
			chosenEmitter->forcedMin = inpEmitter.forcedMin;
			chosenEmitter->forcedMax = inpEmitter.forcedMax;
			chosenEmitter->collectionModelInst->SetMinMax(chosenEmitter->forcedMin, chosenEmitter->forcedMax);
		}
		else
			for (unsigned int i = 0; i != chosenEmitter->sourceGeom.size(); i++)
				inpMinMaxReducer.RequestMinMax(chosenEmitter->destGeom[i]);

		chosenEmitter->Populate();
		chosenEmitter->particlesBuf->UploadSubData(0, chosenEmitter->particles.data(), (unsigned int)(chosenEmitter->particles.size() * sizeof(ParticleItem)));

		chosenEmitter->hidden = false;
		chosenEmitter->hideTimer.Start();
	}
}

unsigned long long HIGHOMEGA::WORLD::ParticleSystemClass::AddBulletEmitter(const vec3& bulletPos, const vec3& bulletDir, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission, HIGHOMEGA::FIZ_X::MATERIAL hitMat, bool cacheForRecycling)
{
	ParticleEmitter curEmitter;

	curEmitter.integrateParams.posLinSpeedBase[0] = bulletPos.x;
	curEmitter.integrateParams.posLinSpeedBase[1] = bulletPos.y;
	curEmitter.integrateParams.posLinSpeedBase[2] = bulletPos.z;
	curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[0] = 0.0f;
	unsigned int emitDirEncoded = toZSignXY(bulletDir);
	unsigned int gravDir = toZSignXY(GravDir.normalized());
	curEmitter.emitDir = bulletDir;
	curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[0] = *((float*)&emitDirEncoded);
	curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[1] = *((float*)&gravDir);
	curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[2] = Grav;

	curEmitter.integrateParams.posLinSpeedBase[3] = 20.0f; // speed
	curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[3] = 5.0f;
	curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] = 0.0f; // rot
	curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[2] = 0.0f;
	curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] = 0.95f; // fade rate
	curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[0] = 0.01f;
	curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[1] = 0.99f; // scale rate
	curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[2] = 0.01f;
	curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[3] = 0.95f; // death
	curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0] = 0.01f;
	curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2] = 1.0f; // Alpha
	curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[3] = 0.01f;

	unsigned long long emittersId = threadSafeMersenneTwister64Bit();

	bool emitBulletWoodDust = false;

	curEmitter.originString = std::string("_bulletdebris_");
	curEmitter.emittersId = emittersId;
	if (hitMat == DIRT || hitMat == RUBBLE || hitMat == CONCRETE)
	{
		curEmitter.integrateParams.numParticles = 10;
		curEmitter.particleMesh = std::string("assets/models/particles/bulletplumes.3md");
		curEmitter.integrateParams.posLinSpeedBase[3] = 10.0f; // speed
		curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[3] = 2.5f;
		curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 0.5f; // Scale
		curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] = 0.0f; // rot
		curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[1] = 1.1f; // scale rate
		curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[2] = 0.0f;
		curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] = 0.8f; // fade rate
		curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[0] = 0.1f;
		curEmitter.noSDFLeaf = true;
		curEmitter.forceMinMax = true;
		curEmitter.forcedMin = bulletPos - vec3(50.0f);
		curEmitter.forcedMax = bulletPos + vec3(50.0f);
		curEmitter.integrateParams.elasticitySpreadFlags = 0u;
		EncodeOrientToLinVel(curEmitter, false);

		curEmitter.particleMeshAssetLoc = std::string("assets/models/particles/");
		curEmitter.requiresScene = true;
		curEmitter.elasticity = 0.0f;

		EncodeElasticity(curEmitter);
		EncodeAffectedByGravity(curEmitter, false);
		curEmitter.scaleToLinVel = false;
		EncodeScaleToLinVel(curEmitter, curEmitter.scaleToLinVel);
		curEmitter.spread = 0.0f;
		EncodeSpread(curEmitter, curEmitter.spread);
		EncodeRepeat(curEmitter, false);
		curEmitter.hideAfterTime = 2.0f;
		curEmitter.hidden = true;
		curEmitter.hidePos = bulletPos;
		curEmitter.reUseTag = REUSE_CONCRETE;
		curEmitter.randomLives = false;
	}
	else if (hitMat == GLASS || hitMat == CERAMIC || hitMat == WOOD || hitMat == METALPLATFORM || hitMat == METALRODS)
	{
		curEmitter.integrateParams.numParticles = 20;
		curEmitter.integrateParams.elasticitySpreadFlags = 0u;
		if (hitMat == GLASS || hitMat == CERAMIC || hitMat == WOOD)
		{
			if (hitMat == GLASS || hitMat == CERAMIC)
			{
				curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 0.2f; // Scale
				curEmitter.particleMesh = std::string("assets/models/particles/bulletshard.3md");
				curEmitter.reUseTag = REUSE_GLASS;
			}
			else
			{
				curEmitter.integrateParams.numParticles = 100;
				curEmitter.particleMesh = std::string("assets/models/particles/bulletsplinter.3md");
				curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 0.1f; // Scale
				curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] = HIGHOMEGA_PI * 2.0f; // rot
				curEmitter.noSDFLeaf = true;
				curEmitter.forceMinMax = true;
				curEmitter.forcedMin = bulletPos - vec3(50.0f);
				curEmitter.forcedMax = bulletPos + vec3(50.0f);
				curEmitter.reUseTag = REUSE_WOOD;

				emitBulletWoodDust = true;
			}
			EncodeOrientToLinVel(curEmitter, false);
			curEmitter.scaleToLinVel = false;
		}
		else
		{
			curEmitter.particleMesh = std::string("assets/models/particles/bulletspark.3md");
			curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 0.1f; // Scale
			EncodeOrientToLinVel(curEmitter, true);
			curEmitter.scaleToLinVel = true;
			curEmitter.reUseTag = REUSE_METAL;
		}
		curEmitter.particleMeshAssetLoc = std::string("assets/models/particles/");
		curEmitter.requiresScene = true;
		curEmitter.elasticity = 0.5f;
		if (!RTInstance::Enabled())
		{
			curEmitter.integrateParams.posLinSpeedBase[0] += bulletDir.x * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
			curEmitter.integrateParams.posLinSpeedBase[1] += bulletDir.y * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
			curEmitter.integrateParams.posLinSpeedBase[2] += bulletDir.z * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
		}

		EncodeElasticity(curEmitter);
		EncodeAffectedByGravity(curEmitter, true);
		EncodeScaleToLinVel(curEmitter, curEmitter.scaleToLinVel);
		curEmitter.spread = 0.25f;
		EncodeSpread(curEmitter, curEmitter.spread);
		EncodeRepeat(curEmitter, false);
		curEmitter.hideAfterTime = 2.0f;
		curEmitter.hidden = true;
		curEmitter.hidePos = bulletPos;
		curEmitter.randomLives = false;
	}

	if (cacheForRecycling)
		AddEmitter(curEmitter, inpMinMaxReducer, sdfBvhSubmission, rtSubmission);
	else
		RecycleEmitter(curEmitter, inpMinMaxReducer);

	if (emitBulletWoodDust)
	{
		curEmitter.ClearForReuse();

		curEmitter.integrateParams.numParticles = 10;
		curEmitter.particleMesh = std::string("assets/models/particles/bulletwooddust.3md");
		curEmitter.integrateParams.posLinSpeedBase[3] = 5.0f; // speed
		curEmitter.integrateParams.dirGravDirGravLenLinSpeedVar[3] = 2.5f;
		curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 0.1f; // Scale
		curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] = 0.0f; // rot
		curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[1] = 1.15f; // scale rate
		curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[2] = 0.0f;
		curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] = 0.8f; // fade rate
		curEmitter.integrateParams.fadeRateVarScaleRateBaseScaleRateVarDeathRateBase[0] = 0.1f;
		curEmitter.noSDFLeaf = true;
		curEmitter.forceMinMax = true;
		curEmitter.forcedMin = bulletPos - vec3(50.0f);
		curEmitter.forcedMax = bulletPos + vec3(50.0f);
		curEmitter.integrateParams.elasticitySpreadFlags = 0u;
		EncodeOrientToLinVel(curEmitter, false);
		
		curEmitter.originString = std::string("_bulletwooddust_"); // A new origin string to tack on to the pre-existing emittersId

		curEmitter.requiresScene = true;
		curEmitter.elasticity = 0.0f;

		EncodeElasticity(curEmitter);
		EncodeAffectedByGravity(curEmitter, false);
		curEmitter.scaleToLinVel = false;
		EncodeScaleToLinVel(curEmitter, curEmitter.scaleToLinVel);
		curEmitter.spread = 0.0f;
		EncodeSpread(curEmitter, curEmitter.spread);
		EncodeRepeat(curEmitter, false);
		curEmitter.hideAfterTime = 2.0f;
		curEmitter.hidden = true;
		curEmitter.hidePos = bulletPos;
		curEmitter.randomLives = false;
		curEmitter.reUseTag = REUSE_WOOD_DUST;

		if (cacheForRecycling)
			AddEmitter(curEmitter, inpMinMaxReducer, sdfBvhSubmission, rtSubmission);
		else
			RecycleEmitter(curEmitter, inpMinMaxReducer);
	}

	return emittersId;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Remove(unsigned long long curId)
{
	if (allEmitters.find(curId) == allEmitters.end()) return;

	for (ParticleEmitter & curEmitter : allEmitters[curId])
	{
		delete curEmitter.transformParamsBuf;
		delete curEmitter.particlesBuf;
		delete curEmitter.integrateParamsBuf;

		delete curEmitter.collectionModel;

		{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); cachedMeshesModels[curEmitter.particleMesh].claims--; }
	}

	rerecordSubmission = true;

	allEmitters.erase(curId);
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Combine(std::vector<ParticleSystemClass>& particleClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall)
{
	submissionsForParticle = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	for (ParticleSystemClass & curParticleClass : particleClasses)
	{
		if (curParticleClass.integrateSubmission) delete curParticleClass.integrateSubmission;
		if (curParticleClass.integrateShader) delete curParticleClass.integrateShader;
		if (curParticleClass.integrateAgainstSceneSubmission) delete curParticleClass.integrateAgainstSceneSubmission; // This really shouldn't be happening here... unless you have an off-main-rendering-thread scene that is useful...
		if (curParticleClass.integrateAgainstSceneShader) delete curParticleClass.integrateAgainstSceneShader; // This really shouldn't be happening here... unless you have an off-main-rendering-thread scene that is useful...
		if (curParticleClass.transformCollectionSubmission) delete curParticleClass.transformCollectionSubmission;
		if (curParticleClass.transformCollectionShader) delete curParticleClass.transformCollectionShader;
		curParticleClass.integrateSubmission = nullptr;
		curParticleClass.integrateShader = nullptr;
		curParticleClass.integrateAgainstSceneSubmission = nullptr;
		curParticleClass.integrateAgainstSceneShader = nullptr;
		curParticleClass.transformCollectionSubmission = nullptr;
		curParticleClass.transformCollectionShader = nullptr;

		for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : curParticleClass.allEmitters)
			allEmitters[curEmittersIdPair.first] = curEmittersIdPair.second;
		sceneInteractingEmitters.insert(sceneInteractingEmitters.end(), curParticleClass.sceneInteractingEmitters.begin(), curParticleClass.sceneInteractingEmitters.end());
		curParticleClass.sceneInteractingEmitters.clear();
		curParticleClass.allEmitters.clear();
	}

	rerecordSubmission = true;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Update(float elapseTime, minMaxReductionClass& inpMinMaxReducer, GroupedSDFBVHSubmission* sdfBvhSubmission, GroupedTraceSubmission* rtSubmission)
{
	if (sdfBvhSubmission || rtSubmission)
	{
		unsigned long long curSceneID = RTInstance::Enabled() ? rtSubmission->SceneID() : sdfBvhSubmission->SceneID();
		if (sceneID != curSceneID)
		{
			sceneID = curSceneID;
			rerecordSubmission = true;
		}

		if (sceneInteractingEmitters.size() > 0)
		{
			for (ParticleEmitter& curEmitter : sceneInteractingEmitters)
				AddEmitter(curEmitter, inpMinMaxReducer, sdfBvhSubmission, rtSubmission);
			sceneInteractingEmitters.clear();
		}
	}

	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
		for (ParticleEmitter& curEmitter : curEmittersIdPair.second)
			curEmitter.AttemptHideEmitter();

	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : allEmitters)
		for (ParticleEmitter & curEmitter : curEmittersIdPair.second)
		{
			curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[3] = curEmitter.hidden ? 0.0f : elapseTime;
			curEmitter.integrateParamsBuf->UploadSubData(0, &curEmitter.integrateParams, sizeof(curEmitter.integrateParams));
		}

	if (rerecordSubmission)
	{
		std::vector<ShaderResource> allParticleBufs;
		std::vector<ShaderResource> allNonSceneInteractingParticleBufs;
		std::vector<ShaderResource> allSceneInteractingParticleBufs;
		std::vector<ShaderResource> allNonSceneInteractingIntegrateParams;
		std::vector<ShaderResource> allSceneInteractingIntegrateParams;
		std::vector<ShaderResource> allTransformParams;
		unsigned int maxParticleCount = 0u , maxSceneInteractingParticles = 0u, maxNonSceneInteractingParticles = 0u, maxVertCount = 0u;
		for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
			for (ParticleEmitter& curEmitter : curEmittersIdPair.second)
			{
				allParticleBufs.emplace_back(RESOURCE_SSBO, COMPUTE, 0, 0, *curEmitter.particlesBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				(curEmitter.requiresScene ? allSceneInteractingParticleBufs : allNonSceneInteractingParticleBufs).emplace_back(RESOURCE_SSBO, COMPUTE, 0, 0, *curEmitter.particlesBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				(curEmitter.requiresScene ? allSceneInteractingIntegrateParams : allNonSceneInteractingIntegrateParams).emplace_back(RESOURCE_SSBO, COMPUTE, 0, 0, *curEmitter.integrateParamsBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY); // Technically this is produced here, but we haven't run into issues not depending on it in transform
				allTransformParams.emplace_back(RESOURCE_SSBO, COMPUTE, 2, 0, *curEmitter.transformParamsBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				maxParticleCount = max(maxParticleCount, curEmitter.integrateParams.numParticles);
				(curEmitter.requiresScene ? maxSceneInteractingParticles : maxNonSceneInteractingParticles) = max((curEmitter.requiresScene ? maxSceneInteractingParticles : maxNonSceneInteractingParticles), curEmitter.integrateParams.numParticles);
				for (unsigned int i = 0; i != curEmitter.sourceGeom.size(); i++)
					maxVertCount = max(maxVertCount, curEmitter.sourceGeom[i]->getVertCount());
			}

		if (integrateSubmission) delete integrateSubmission;
		if (integrateShader) delete integrateShader;
		if (integrateAgainstSceneSubmission) delete integrateAgainstSceneSubmission;
		if (integrateAgainstSceneShader) delete integrateAgainstSceneShader;
		if (transformCollectionSubmission) delete transformCollectionSubmission;
		if (transformCollectionShader) delete transformCollectionShader;
		integrateSubmission = nullptr;
		integrateShader = nullptr;
		integrateAgainstSceneSubmission = nullptr;
		integrateAgainstSceneShader = nullptr;
		transformCollectionSubmission = nullptr;
		transformCollectionShader = nullptr;

		if (allParticleBufs.size() > 0)
		{
			if (allNonSceneInteractingParticleBufs.size() > 0)
			{
				integrateSubmission = new ComputeSubmission;
				integrateShader = new ShaderResourceSet;
				integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, allNonSceneInteractingParticleBufs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY); // Technically this is produced here, but we haven't run into issues not depending on it in transform
				integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, allNonSceneInteractingIntegrateParams, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				integrateShader->Create("shaders/integrateParticles.comp.spv", "main");
				integrateSubmission->MakeDispatch(Instance, "nonSceneInteractingParticleIntegrateDispatch", *integrateShader, (unsigned int)ceil((double)maxNonSceneInteractingParticles / (double)WorkGroupIntegrateX()), (unsigned int)allNonSceneInteractingIntegrateParams.size(), 1);
			}

			if (allSceneInteractingParticleBufs.size() > 0 && (sdfBvhSubmission || rtSubmission))
			{
				integrateAgainstSceneSubmission = new ComputeSubmission;
				integrateAgainstSceneShader = new ShaderResourceSet;
				integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, allSceneInteractingParticleBufs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY); // Technically this is produced here, but we haven't run into issues not depending on it in transform
				integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, allSceneInteractingIntegrateParams, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				if (!RTInstance::Enabled())
				{
					sdfBvhSubmission->SceneID();
					integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, 0, sdfBvhSubmission->invMatBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
					integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, 1, sdfBvhSubmission->leavesBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
					integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, 2, sdfBvhSubmission->cwNodesBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
					integrateAgainstSceneShader->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 3, sdfBvhSubmission->SDFs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				}
				else
				{
					rtSubmission->SceneID();
					integrateAgainstSceneShader->AddResource(RESOURCE_RT_ACCEL_STRUCT, COMPUTE, 2, 0, rtSubmission->rtScene);
					integrateAgainstSceneShader->AddResource(RESOURCE_SAMPLER, COMPUTE, 3, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
					integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 4, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
					giantVertBufferSharedMutex.lock_shared();
					integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 4, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
					giantVertBufferSharedMutex.unlock_shared();
					integrateAgainstSceneShader->AddResource(RESOURCE_SSBO, COMPUTE, 4, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				}
				integrateAgainstSceneShader->Create(RTInstance::Enabled() ? "shaders/integrateParticlesAgainstSceneRT.comp.spv" : "shaders/integrateParticlesAgainstScene.comp.spv", "main");
				integrateAgainstSceneSubmission->MakeDispatch(Instance, "sceneInteractingParticleIntegrateDispatch", *integrateAgainstSceneShader, (unsigned int)ceil((double)maxSceneInteractingParticles / (double)WorkGroupIntegrateX()), (unsigned int)allSceneInteractingIntegrateParams.size(), 1);
			}

			transformCollectionSubmission = new ComputeSubmission;
			transformCollectionShader = new ShaderResourceSet;
			giantVertBufferSharedMutex.lock_shared();
			transformCollectionShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			transformCollectionShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, allParticleBufs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			transformCollectionShader->AddResource(RESOURCE_SSBO, COMPUTE, 2, allTransformParams, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			transformCollectionShader->Create("shaders/transformParticles.comp.spv", "main");
			transformCollectionSubmission->MakeDispatch(Instance, "particleTransformDispatch", *transformCollectionShader, (unsigned int)ceil((double)maxVertCount / (double)WorkGroupTransformX()), (unsigned int)ceil((double)maxParticleCount / (double)WorkGroupTransformY()), (unsigned int)allParticleBufs.size());
		}

		rerecordSubmission = false;
	}

	if (integrateSubmission) (*integrateSubmission).MakeAsync().Submit();
	if (integrateAgainstSceneSubmission) (*integrateAgainstSceneSubmission).MakeAsync().Submit();
	if (transformCollectionSubmission) (*transformCollectionSubmission).MakeAsync().Submit();

	if (sdfBvhSubmission && !RTInstance::Enabled())
	{
		frameNumber++;
		if (frameNumber % 60 == 0)
		{
			for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
				for (ParticleEmitter& curEmitter : curEmittersIdPair.second)
				{
					if (!curEmitter.requiresScene) continue;
					for (unsigned int i = 0; i != curEmitter.sourceGeom.size(); i++)
						inpMinMaxReducer.RequestMinMax(curEmitter.destGeom[i]);
				}

			inpMinMaxReducer.Process();

			UpdateSDFs(false, true);

			sdfBvhSubmission->sceneChanged = true;
			sdfBvhSubmission->SceneID(); // Update the SDF BVH since leaf bounds for particles changed... the next run of this->Update() will once again re-create scene interacting emitter submissions
		}
	}

	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
		for (ParticleEmitter& curEmitter : curEmittersIdPair.second)
		{
			if (submissionsForParticle.size() > 0 && !curEmitter.collectionModelInst)
			{
				curEmitter.collectionModelInst = curEmitter.collectionModel->CreateInstance();
				for (GroupedRenderSubmission* curSubmission : submissionsForParticle)
					perSubmissionCall(curSubmission, curEmitter.collectionModelInst);
			}
			curEmitter.collectionModel->SetDirty();
		}

	while (true)
	{
		bool removedSomething = false;
		for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
		{
			for (ParticleEmitter& curEmitter : curEmittersIdPair.second)
				if (curEmitter.expireAfterTime >= 0.0f && curEmitter.expireTimer.Diff() >= curEmitter.expireAfterTime)
				{
					Remove(curEmitter.emittersId);
					removedSomething = true;
					break;
				}
			if (removedSomething) break;
		}
		if (!removedSomething) break;
	}
}

void HIGHOMEGA::WORLD::ParticleSystemClass::UpdateSDFs(bool onlySceneInteracting, bool forceRefresh)
{
	std::vector<GraphicsModel*> modelsToUpdate;
	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersPair : allEmitters)
		for (ParticleEmitter& curEmitter : curEmittersPair.second)
		{
			if ((onlySceneInteracting && !curEmitter.requiresScene) || curEmitter.noSDFLeaf) continue;
			modelsToUpdate.push_back(curEmitter.collectionModel);
		}

	if (modelsToUpdate.size() > 0) GraphicsModel::UpdateSDFs(modelsToUpdate, forceRefresh);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::ClearContent()
{
	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>>& curEmittersIdPair : allEmitters)
		for (ParticleEmitter & curEmitter : curEmittersIdPair.second)
		{
			delete curEmitter.transformParamsBuf;
			delete curEmitter.particlesBuf;
			delete curEmitter.integrateParamsBuf;

			delete curEmitter.collectionModel;

			{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); cachedMeshesModels[curEmitter.particleMesh].claims--; }
		}
	allEmitters.clear();
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);

	if (integrateSubmission) delete integrateSubmission;
	if (integrateShader) delete integrateShader;
	if (integrateAgainstSceneSubmission) delete integrateAgainstSceneSubmission;
	if (integrateAgainstSceneShader) delete integrateAgainstSceneShader;
	if (transformCollectionSubmission) delete transformCollectionSubmission;
	if (transformCollectionShader) delete transformCollectionShader;
	integrateSubmission = nullptr;
	integrateShader = nullptr;
	integrateAgainstSceneSubmission = nullptr;
	integrateAgainstSceneShader = nullptr;
	transformCollectionSubmission = nullptr;
	transformCollectionShader = nullptr;

	rerecordSubmission = true;
}

unsigned long long HIGHOMEGA::WORLD::CameraSystemClass::Populate(Mesh & inpMesh)
{
	unsigned long long curCamRailId = threadSafeMersenneTwister64Bit();
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];
		Camera curCamera;

		if (curPolyGroup.type != "CAMERA") continue;

		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "duration", curCamera.duration)) curCamera.duration = 1.0f;
		float orderFloat;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "order", orderFloat)) FATAL_ERROR("Could not get camera order");
		curCamera.order = (unsigned int)orderFloat;

		if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", curCamera.pos) ||
			 !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", curCamera.dir) ) FATAL_ERROR("Could not get camera pos/dir");

		float tmpFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "fadein", tmpFloat)) curCamera.actionParams["fadein"] = 0.0f;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "fadeout", tmpFloat)) curCamera.actionParams["fadeout"] = 0.0f;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "showAurora", tmpFloat)) curCamera.actionParams["showAurora"] = 0.0f;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "showClouds", tmpFloat)) curCamera.actionParams["showClouds"] = 0.0f;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "forceSunAngle", tmpFloat)) curCamera.actionParams["forceSunAngle"] = tmpFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "unforceSunAngle", tmpFloat)) curCamera.actionParams["unforceSunAngle"] = 0.0f;
		if (Mesh::getDataRowString(curPolyGroup, "PROPS", "newMap", curCamera.newMap))
		{
			curCamera.actionParams["newMap"] = 0.0f;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "newMapBelong", curCamera.newMapBelong)) FATAL_ERROR("Could not get new map directory for camera newMap action");
		}
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "creditRoll", tmpFloat)) curCamera.actionParams["creditRoll"] = 0.0f;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "lightShaftAmount", tmpFloat)) curCamera.actionParams["lightShaftAmount"] = tmpFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "lightShaftExtinction", tmpFloat)) curCamera.actionParams["lightShaftExtinction"] = tmpFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "sunDirectLightStrength", tmpFloat)) curCamera.actionParams["sunDirectLightStrength"] = tmpFloat;

		allItems[curCamRailId].push_back(curCamera);
	}

	if (allItems.find(curCamRailId) == allItems.end()) return curCamRailId; // No rail was added...

	std::sort(allItems[curCamRailId].begin(), allItems[curCamRailId].end(), [](const Camera& lhs, const Camera& rhs)
	{
		return lhs.order < rhs.order;
	});

	// Quick safety check...
	for (int i = 0; i != allItems[curCamRailId].size(); i++)
		if (allItems[curCamRailId][i].order != i + 1) FATAL_ERROR("There's an issue with numbering in the camera system");

	return curCamRailId;
}

void HIGHOMEGA::WORLD::CameraSystemClass::Combine(std::vector<CameraSystemClass>& cameraRailClasses)
{
	for (CameraSystemClass & curCamSystem : cameraRailClasses)
		if (curCamSystem.allItems.size() > 0 && curCamSystem.allItems.begin()->second.size() > 0)
		{
			allItems = curCamSystem.allItems;
			curNum = 0;
			curTimeElapsed = 0.0f;
			curAlpha = 1.0f;
			break;
		}

	for (CameraSystemClass & curCamSystem : cameraRailClasses)
		curCamSystem.ClearContent();
}

void HIGHOMEGA::WORLD::CameraSystemClass::Remove(unsigned long long inpId)
{
	allItems.erase(inpId);
}

void HIGHOMEGA::WORLD::CameraSystemClass::Update(WorldParamsClass & WorldParams, PipelineSetupReturn & mapChangeDataStruct)
{
	if (allItems.size() == 0 || allItems.begin()->second.size() == 0) return;
	std::vector<Camera> & cameraRail = allItems.begin()->second;
	if (curNum == cameraRail.size() - 1)
	{
		if (cameraRail[curNum].actionParams.find("newMap") != cameraRail[curNum].actionParams.end())
		{
			// unhandled for now since the mechanism changed with zone streaming... should effectively re-use streaming mechanisms
			return;
		}
		if (cameraRail[curNum].actionParams.find("creditRoll") != cameraRail[curNum].actionParams.end())
		{
			mapChangeDataStruct.doCredits = true;
			return;
		}
		return;
	}

	Camera & curCamera = cameraRail[curNum];
	Camera & nextCamera = cameraRail[curNum + 1];
	float lerpFraction = curTimeElapsed / cameraRail[curNum].duration;

	if (lerpFraction < 0.5f && curNum > 0)
	{
		Camera & prevCamera = cameraRail[curNum - 1];
		Camera & nextCamera = cameraRail[curNum + 1];

		vec3 toCurrent = curCamera.pos - prevCamera.pos;
		vec3 toNext = nextCamera.pos - curCamera.pos;

		float bezierFraction = (curTimeElapsed + 0.5f * prevCamera.duration) / (0.5f * prevCamera.duration + 0.5f * curCamera.duration);
		curPos = QuadraticBezier(prevCamera.pos + toCurrent * 0.5f, curCamera.pos, curCamera.pos + toNext * 0.5f, bezierFraction);
		curDir = QuadraticBezier(Lerp (prevCamera.dir, curCamera.dir, 0.5f), curCamera.dir, Lerp(curCamera.dir, nextCamera.dir, 0.5f), bezierFraction).normalized();
	}
	else if (lerpFraction > 0.5f && curNum < cameraRail.size() - 2)
	{
		Camera & nextCamera = cameraRail[curNum + 1];
		Camera & nextNextCamera = cameraRail[curNum + 2];

		vec3 toNext = nextCamera.pos - curCamera.pos;
		vec3 toNextNext = nextNextCamera.pos - nextCamera.pos;

		float bezierFraction = (curTimeElapsed - 0.5f * curCamera.duration) / (0.5f * curCamera.duration + 0.5f * nextCamera.duration);
		curPos = QuadraticBezier(curCamera.pos + toNext * 0.5f, nextCamera.pos, nextCamera.pos + toNextNext * 0.5f, bezierFraction);
		curDir = QuadraticBezier(Lerp(curCamera.dir, nextCamera.dir, 0.5f), nextCamera.dir, Lerp(nextCamera.dir, nextNextCamera.dir, 0.5f), bezierFraction).normalized();
	}
	else
	{
		Camera & nextCamera = cameraRail[curNum + 1];

		curPos = Lerp(curCamera.pos, nextCamera.pos, lerpFraction);
		curDir = Lerp(curCamera.dir, nextCamera.dir, lerpFraction).normalized();
	}

	if (curCamera.actionParams.find("fadein") != curCamera.actionParams.end()) curAlpha = lerpFraction;
	if (curCamera.actionParams.find("fadeout") != curCamera.actionParams.end()) curAlpha = 1.0f - lerpFraction;
	if (curCamera.actionParams.find("showAurora") != curCamera.actionParams.end()) WorldParams.ShowAurora();
	if (curCamera.actionParams.find("showClouds") != curCamera.actionParams.end()) WorldParams.ShowClouds();
	if (curCamera.actionParams.find("forceSunAngle") != curCamera.actionParams.end()) WorldParams.ForceSunAngle(curCamera.actionParams["forceSunAngle"]);
	if (curCamera.actionParams.find("unforceSunAngle") != curCamera.actionParams.end()) WorldParams.UnforceSunAngle();
	if (curCamera.actionParams.find("lightShaftAmount") != curCamera.actionParams.end()) WorldParams.SetLightShaftAmount(curCamera.actionParams["lightShaftAmount"]);
	if (curCamera.actionParams.find("lightShaftExtinction") != curCamera.actionParams.end()) WorldParams.SetLightShaftExtinction(curCamera.actionParams["lightShaftExtinction"]);
	if (curCamera.actionParams.find("sunDirectLightStrength") != curCamera.actionParams.end()) WorldParams.SetSunDirectLightStrength(curCamera.actionParams["sunDirectLightStrength"]);

	curTimeElapsed += WorldParams.GetFrameTime();
	if (curTimeElapsed > curCamera.duration)
	{
		if (curCamera.actionParams.find("fadein") != curCamera.actionParams.end()) curAlpha = 1.0f;
		if (curCamera.actionParams.find("fadeout") != curCamera.actionParams.end()) curAlpha = 0.0f;

		curTimeElapsed = 0.0f;
		curNum++;
	}
}

void HIGHOMEGA::WORLD::CameraSystemClass::ClearContent()
{
	curNum = 0;
	curTimeElapsed = 0.0f;
	curAlpha = 1.0f;
	allItems.clear();
}

unsigned long long HIGHOMEGA::WORLD::GuidedModelSystemClass::Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*> &&subList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall)
{
	unsigned long long guidedModelsId = threadSafeMersenneTwister64Bit();
	submissionsForGuidedModels = subList;
	perSubmissionCall = inpPerSubmissionCall;

	std::vector <unsigned int> allPathNumbers;
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		float guidedModelPathNumberFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "guidedModelPathNumber", guidedModelPathNumberFloat))
		{
			unsigned int curPathNum = (unsigned int)guidedModelPathNumberFloat;
			if (std::find(allPathNumbers.begin(), allPathNumbers.end(), curPathNum) == allPathNumbers.end())
				allPathNumbers.push_back(curPathNum);
		}
	}

	std::sort(allPathNumbers.begin(), allPathNumbers.end());

	for (unsigned int pathNum : allPathNumbers)
	{
		PathState curPath;
		curPath.curTrans.Ident();
		for (int i = 0; i != inpMesh.DataGroups.size(); i++)
		{
			HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

			float guidedModelPathNumberFloat;
			if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "guidedModelPathNumber", guidedModelPathNumberFloat))
			{
				unsigned int curPathNum = (unsigned int)guidedModelPathNumberFloat;
				if (curPathNum != pathNum) continue;
			}
			else
				continue;

			PathPoint curPoint;

			if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", curPoint.pos)) FATAL_ERROR("Guided model pos not provided");
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "scale", curPoint.scale)) curPoint.scale = 1.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "duration", curPoint.duration)) curPoint.duration = 1.0f;

			float orderFloat;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "order", orderFloat)) FATAL_ERROR("Guided model order not provided");
			curPoint.order = (unsigned int)orderFloat;

			if (curPath.meshModelKey == "")
			{
				std::string guidedModelMesh, guidedModelMeshAssetLoc;
				if (!Mesh::getDataRowString(curPolyGroup, "PROPS", "guidedModelMesh", guidedModelMesh) ||
					!Mesh::getDataRowString(curPolyGroup, "PROPS", "guidedModelMeshAssetLoc", guidedModelMeshAssetLoc)) FATAL_ERROR("Guided model or asset loc not found");

				curPath.meshModelRef = InsertEntry([&guidedModelMesh, &guidedModelMeshAssetLoc](Mesh& outMesh, GraphicsModel** outModel) {
					outMesh = std::move(Mesh(guidedModelMesh));
					*outModel = new GraphicsModel(outMesh, guidedModelMeshAssetLoc, Instance, [](int, DataGroup& inpGroup) -> bool {
						float tmpFloat;
						return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
					});
				}, guidedModelMesh, std::string("guided meshes"), cachedMeshesModels, cachedMeshesModelsMutex);

				curPath.meshModelKey = guidedModelMesh;
			}

			curPath.fullPath.push_back(curPoint);
		}

		std::sort(curPath.fullPath.begin(), curPath.fullPath.end(), [](const PathPoint& lhs, const PathPoint& rhs)
		{
			return lhs.order < rhs.order;
		});

		allItems[guidedModelsId].push_back(curPath);
	}

	return guidedModelsId;
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::Update(float elapseTime)
{
	for (std::pair<const unsigned long long, std::vector<PathState>>& curItem : allItems)
		for (PathState& curPS : curItem.second)
		{
			unsigned int prevNum, curNum, nextNum, nextNextNum;

			curNum = curPS.curNum;

			if (curNum == 0) prevNum = (unsigned int)curPS.fullPath.size() - 1;
			else prevNum = curNum - 1;

			if (curNum == (unsigned int)curPS.fullPath.size() - 1) nextNum = 0;
			else nextNum = curNum + 1;

			if (nextNum == (unsigned int)curPS.fullPath.size() - 1) nextNextNum = 0;
			else nextNextNum = nextNum + 1;

			vec3 toCurrent = curPS.fullPath[curNum].pos - curPS.fullPath[prevNum].pos;
			vec3 toNext = curPS.fullPath[nextNum].pos - curPS.fullPath[curNum].pos;
			vec3 toNextNext = curPS.fullPath[nextNextNum].pos - curPS.fullPath[nextNum].pos;

			float lerpFraction = curPS.curTimeElapsed / curPS.fullPath[curNum].duration;
			if (lerpFraction < 0.25f)
			{
				float bezierFraction = (lerpFraction + 0.25f) / 0.5f;
				curPS.curDir = Lerp(toCurrent.normalized(), toNext.normalized(), bezierFraction).normalized();
				curPS.curScale = Lerp(curPS.fullPath[prevNum].scale, curPS.fullPath[curNum].scale, bezierFraction);
				curPS.curPos = QuadraticBezier(curPS.fullPath[prevNum].pos + toCurrent * 0.75f, curPS.fullPath[curNum].pos, curPS.fullPath[curNum].pos + toNext * 0.25f, bezierFraction);
			}
			else if (lerpFraction > 0.75f)
			{
				float bezierFraction = (lerpFraction - 0.75f) / 0.5f;
				curPS.curDir = Lerp(toNext.normalized(), toNextNext.normalized(), bezierFraction).normalized();
				curPS.curScale = Lerp(curPS.fullPath[curNum].scale, curPS.fullPath[nextNum].scale, bezierFraction);
				curPS.curPos = QuadraticBezier(curPS.fullPath[curNum].pos + toNext * 0.75f, curPS.fullPath[nextNum].pos, curPS.fullPath[nextNum].pos + toNextNext * 0.25f, bezierFraction);
			}
			else
			{
				curPS.curDir = toNext.normalized();
				curPS.curScale = curPS.fullPath[curNum].scale;
				curPS.curPos = Lerp(curPS.fullPath[curNum].pos, curPS.fullPath[nextNum].pos, lerpFraction);
			}

			vec3 curSide = cross(vec3(0.0f, 1.0f, 0.0f), curPS.curDir).normalized();
			vec3 curUp = cross(curSide, curPS.curDir);
			if (curUp.y < 0.0f) curUp = -curUp;

			curPS.curTrans.Ident();
			curPS.curTrans.i[0][0] = curPS.curDir.x * curPS.curScale;
			curPS.curTrans.i[1][0] = curPS.curDir.y * curPS.curScale;
			curPS.curTrans.i[2][0] = curPS.curDir.z * curPS.curScale;

			curPS.curTrans.i[0][1] = curUp.x * curPS.curScale;
			curPS.curTrans.i[1][1] = curUp.y * curPS.curScale;
			curPS.curTrans.i[2][1] = curUp.z * curPS.curScale;

			curPS.curTrans.i[0][2] = curSide.x * curPS.curScale;
			curPS.curTrans.i[1][2] = curSide.y * curPS.curScale;
			curPS.curTrans.i[2][2] = curSide.z * curPS.curScale;

			curPS.curTrans.i[0][3] = curPS.curPos.x;
			curPS.curTrans.i[1][3] = curPS.curPos.y;
			curPS.curTrans.i[2][3] = curPS.curPos.z;

			curPS.curTimeElapsed += elapseTime;
			if (curPS.curTimeElapsed > curPS.fullPath[curNum].duration)
			{
				curPS.curTimeElapsed = 0.0f;
				curPS.curNum++;
				if (curPS.curNum == (unsigned int)curPS.fullPath.size()) curPS.curNum = 0;
			}
		}

	for (std::pair<const unsigned long long, std::vector<PathState>>& curGuidedModels : allItems)
		for (PathState& curPathState : curGuidedModels.second)
		{
			if (submissionsForGuidedModels.size() > 0 && !curPathState.modelInst)
			{
				curPathState.modelInst = curPathState.meshModelRef->model->CreateInstance(&curPathState.curTrans, true);
				for (GroupedRenderSubmission* curSubmission : submissionsForGuidedModels)
					perSubmissionCall(curSubmission, curPathState.modelInst);
			}
			if (curPathState.modelInst) curPathState.modelInst->Update(curPathState.curTrans);
		}
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::Combine(std::vector <GuidedModelSystemClass> & guidedModelsClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall)
{
	submissionsForGuidedModels = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	for (GuidedModelSystemClass & curGuidedModelsClass : guidedModelsClasses)
	{
		for (std::pair<const unsigned long long, std::vector<PathState>> & curPathStatesIdPair : curGuidedModelsClass.allItems)
			allItems[curPathStatesIdPair.first] = curPathStatesIdPair.second;
		curGuidedModelsClass.allItems.clear();
	}
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::Remove(unsigned long long curId)
{
	if (allItems.find(curId) == allItems.end()) return;

	for (PathState & curPathState : allItems[curId])
	{
		if (curPathState.modelInst) curPathState.modelInst->modelRef->DestroyInstance(curPathState.modelInst);
		{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); if (cachedMeshesModels.find(curPathState.meshModelKey) != cachedMeshesModels.end()) cachedMeshesModels[curPathState.meshModelKey].claims--; }
	}
	allItems.erase(curId);
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::UpdateSDFs(bool forceRefresh)
{
	std::unordered_set<GraphicsModel*> modelsToUpdateSet;
	for (std::pair<const unsigned long long, std::vector<PathState>>& curPathStatesIdPair : allItems)
		for (PathState& curPathState : curPathStatesIdPair.second)
			if (!curPathState.meshModelRef->model->sdf) modelsToUpdateSet.insert(curPathState.meshModelRef->model);
	if (modelsToUpdateSet.size())
	{
		std::vector<GraphicsModel*> modelsToUpdate(modelsToUpdateSet.begin(), modelsToUpdateSet.end());
		GraphicsModel::UpdateSDFs(modelsToUpdate, forceRefresh);
	}
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::ClearContent()
{
	for (std::pair<const unsigned long long, std::vector<PathState>> & curItem : allItems)
		for (PathState & curItem : curItem.second)
		{
			if (curItem.modelInst) curItem.modelInst->modelRef->DestroyInstance(curItem.modelInst);
			{ std::lock_guard<std::mutex> lk(cachedMeshesModelsMutex); if (cachedMeshesModels.find(curItem.meshModelKey) != cachedMeshesModels.end()) cachedMeshesModels[curItem.meshModelKey].claims--; }
		}
	allItems.clear();
	CleanupCache(cachedMeshesModels, cachedMeshesModelsMutex);
}

void HIGHOMEGA::WORLD::CreateWorld(WorldParamsClass & WorldParams)
{
	PhysicsThread = new std::thread(PhysicsLoop);
	EntitiesThread = new std::thread(EntitiesLoop, &WorldParams);
	AudioThread = new std::thread(AudioSystemClass::AudioLoop, &AudioSystem);
}

void HIGHOMEGA::WORLD::DestroyWorld()
{
	{std::unique_lock <std::mutex> lk(fizXSignal.quit_mutex);
	std::unique_lock <std::mutex> lk2(entitiesSignal.quit_mutex);
	fizXSignal.quit = true;
	entitiesSignal.quit = true;}

	PhysicsThread->join();
	EntitiesThread->join();
	delete PhysicsThread;
	delete EntitiesThread;
	PhysicsThread = nullptr;
	EntitiesThread = nullptr;

	{std::unique_lock <std::mutex> lk4(AudioSystem.audioSignal.quit_mutex); AudioSystem.audioSignal.quit = true;}
	AudioThread->join();
	delete AudioThread;
	AudioThread = nullptr;

	plasteredItemsCollection.ClearContent();
	physicalItemsCollection.ClearContent();
	particleSystem.ClearContent();
	guidedModelSystem.ClearContent();
	zoneStreaming.ClearContent();
	cameraSystem.ClearContent();
	mainMinMaxReducer.ClearContent();

	allBodies.clear();

	// We need these threads again...
	fizXSignal.started = false;
	fizXSignal.quit = false;

	entitiesSignal.quit = false;
}

HIGHOMEGA::WORLD::DefaultPipelineSetupClass::DefaultPipelineSetupClass(std::string mapPath, std::string mapBelong, bool cmdOptHwRt, unsigned int cmdOptFullRes, WINDOW_MODE cmdOptWindowed)
{
	PostProcessTri.Create(MainFrustum.eye, MainFrustum.look, MainFrustum.up, MainFrustum.screen_whr, MainFrustum.screen_fov);

	static bool MainMenuPassed = false;
	if (!MainMenuPassed)
	{
		MainMenu.Create(PostProcessTri, cmdOptHwRt, cmdOptFullRes, cmdOptWindowed);
		while (!MainMenu.IsDone())
		{
			Handler(); // We have no choice but to have this here because of SDL
			MainMenu.Render(false,
				[&]() {
					RTInstance::Disable();
				},
				[]() {
					RTInstance::Enable(Instance);
				});
		}
		if (MainMenu.IsRebooting())
		{
			ApplicationQuitting = true;
			return;
		}

		MainMenuPassed = true;
	}

	sdfBvhSubmission.defaultFilterFunction = [](const MeshMaterial& curMat) -> bool {
		if (curMat.postProcess) return false;
		return true;
	};
	zoneStreaming.Create(mapBelong, {&VisibilityPass.submission, &DecalPass.submission, &mainRTSubmission, &sdfBvhSubmission, 
									 &ShadowMapCascadeNear.SetMode(ShadowMapClass::SHADOWMAP_MODE::ORTHO).getSubmission(),
									 &ShadowMapCascadeFar.SetMode(ShadowMapClass::SHADOWMAP_MODE::ORTHO).getSubmission(),
									 &Modulate.submission, &ScreenSpaceGather.submission},
	[&](GroupedRenderSubmission *inpSub, GraphicsModelInstance *inpGraphicsModelInst) -> void
	{
		if (inpSub == &ShadowMapCascadeNear.getSubmission() || inpSub == &ShadowMapCascadeFar.getSubmission())
			inpSub->Add(*inpGraphicsModelInst, GroupedRasterSubmission::everythingButDecalsFilter);
		else if (inpSub == &ScreenSpaceGather.submission)
			inpSub->Add(*inpGraphicsModelInst, GroupedRasterSubmission::postProcessOnlyFilter);
		else if (inpSub == &Modulate.submission)
			inpSub->Add(*inpGraphicsModelInst, GroupedRasterSubmission::blendOnlyFilter);
		else if (inpSub == &DecalPass.submission)
			inpSub->Add(*inpGraphicsModelInst, GroupedRasterSubmission::decalOnlyFilter);
		else if (inpSub == &VisibilityPass.submission)
			inpSub->Add(*inpGraphicsModelInst, GroupedRasterSubmission::noBlendOrPostProcessOrDecalFilter);
		else
			inpSub->Add(*inpGraphicsModelInst);
	});
	CreateWorld(worldParams);

	MoBlur.CreateVelocityBuffer();
	SkyDome.Create(PostProcessTri, worldParams, MoBlur);
	ShadowMapCascadeNear.Create(worldParams, vec2(0.0001f, 0.0002f));
	ShadowMapCascadeFar.Create(worldParams, vec2(0.001f, 0.002f));
	VisibilityPass.Create(ScreenSize.width, ScreenSize.height, 0.1f, MainFrustum, true);
	GatherResolve.Create(VisibilityPass, &MoBlur);
	DecalPass.Create(GatherResolve, MainFrustum, PostProcessTri);
	PathTrace.Create(PostProcessTri, GatherResolve, SkyDome, ShadowMapCascadeNear, ShadowMapCascadeFar, mainRTSubmission, &sdfBvhSubmission, worldParams.StartPlayerPos());
	ClearSurfaceCache.Create(PathTrace);
	TemporalAccumulate.Create(PostProcessTri, GatherResolve, PathTrace);
	SpatialDenoise.Create(PostProcessTri, PathTrace, TemporalAccumulate, GatherResolve);
	ShadowMapScreen.CreateDirectional(PostProcessTri, ShadowMapCascadeNear, ShadowMapCascadeFar, GatherResolve, "Primary", true);
	Modulate.Create(PostProcessTri, MoBlur, VisibilityPass, GatherResolve, PathTrace, SpatialDenoise, SkyDome, ShadowMapScreen);
	ScreenSpaceGather.Create(PostProcessTri, MoBlur, VisibilityPass, GatherResolve, SkyDome, worldParams);
	NearScattering.Create(PostProcessTri, GatherResolve, ShadowMapCascadeNear, ShadowMapCascadeFar, SkyDome, worldParams, ScreenSpaceGather, 512);
	ScreenSpaceFX.Create(PostProcessTri, SkyDome, VisibilityPass, GatherResolve, PathTrace, Modulate, ScreenSpaceGather, MoBlur, NearScattering);
	MoBlur.Create(PostProcessTri, ScreenSpaceFX, ScreenSpaceGather, VisibilityPass, GatherResolve);
	DoF.Create(PostProcessTri, GatherResolve, MoBlur);

	SplashDisplay.Create(PostProcessTri);
	physicalItemsCollection.CacheBulletEmittersHoles(particleSystem, mainMinMaxReducer, &sdfBvhSubmission, &mainRTSubmission);

	if (HIGHOMEGA::creativeLicense) clairAudio = AudioSystem.Insert("assets/audio/clair.wav", AudioSystemClass::SOUND_TYPE::MUSIC, vec3(0.0f), vec3(0.0f), 0.5f);
}

HIGHOMEGA::WORLD::PipelineSetupReturn HIGHOMEGA::WORLD::DefaultPipelineSetupClass::Run()
{
	PipelineSetupReturn potentialMapChangeInfo;
	for (;;)
	{
		worldParams.StartFrameTimer();
		frameInstrument.Start();
		INSTRUMENTATION::FPSCounter::Start();

		CommonSharedMutex.lock_shared();

		if (worldParams.FirstPersonControls())
		{
			MainFrustum.CopyFromEntitiesThread();
		}
		else
		{
			cameraSystem.Update(worldParams, potentialMapChangeInfo);
			if (potentialMapChangeInfo.doCredits)
			{
				worldParams.EndFrameTimer();
				CommonSharedMutex.unlock_shared();
				while (true)
				{
					static bool loggedMemStats = false;
					if (!loggedMemStats)
					{
						GL::MEMORY_MANAGER::LogMemUsageStats();
						loggedMemStats = true;
					}
					Handler(); // We have no choice but to have this here because of SDL
					SplashDisplay.Render();
				}
			}
			vec3 prevEye = MainFrustum.eye;
			MainFrustum.ForceEyeAndLook(cameraSystem.curPos, cameraSystem.curDir);
			vec3 curSide = cross(cameraSystem.curDir, vec3(0.0f, 1.0f, 0.0f)).normalized();
			vec3 realUp = cross(curSide, cameraSystem.curDir);
			if (realUp.y < 0.0f) realUp = -realUp;
			AudioSystem.SetListener(cameraSystem.curPos, cameraSystem.curPos - prevEye, cameraSystem.curDir, realUp);
			if (clairAudio) AudioSystem.SetPosition(clairAudio, cameraSystem.curPos);
		}
		MainFrustum.Update();
		PostProcessTri.UpdateViewSpace(MainFrustum.eye, MainFrustum.look, MainFrustum.up, MainFrustum.screen_whr, MainFrustum.screen_fov);

		plasteredItemsCollection.Update(worldParams.GetFrameTime());
		physicalItemsCollection.Update([&](vec3 & inEye, vec3 & bodyPos, float bodyRad) -> bool { return zoneStreaming.isInDrawRegion(inEye, bodyPos, bodyRad); }, player.physics.bodyPos, player.physics.GetStandingHeight(), particleSystem, mainMinMaxReducer, &sdfBvhSubmission, &mainRTSubmission, worldParams);
		particleSystem.Update(worldParams.GetFrameTime(), mainMinMaxReducer, &sdfBvhSubmission, &mainRTSubmission);
		guidedModelSystem.Update(worldParams.GetFrameTime());
		mainMinMaxReducer.Process();

		MoBlur.PrepareForFrame();
		Handler(); // We have no choice but to have this here because of SDL
		worldParams.AddSunAngle(worldParams.GetFrameTime() * (!HIGHOMEGA::creativeLicense ? 0.0075f : 0.075f));
		SkyDome.Render(worldParams, mainRTSubmission);
		Handler(); // We have no choice but to have this here because of SDL
		ShadowMapCascadeNear.RenderOrtho(zoneStreaming.getVisibleMin(), zoneStreaming.getVisibleMax(), 0.0f, &MainFrustum);
		Handler(); // We have no choice but to have this here because of SDL
		ShadowMapCascadeFar.RenderOrtho(zoneStreaming.getVisibleMin(), zoneStreaming.getVisibleMax(), 0.0f);
		Handler(); // We have no choice but to have this here because of SDL
		VisibilityPass.Render();
		Handler(); // We have no choice but to have this here because of SDL
		GatherResolve.Render();
		Handler(); // We have no choice but to have this here because of SDL
		DecalPass.Render();
		Handler(); // We have no choice but to have this here because of SDL
		ClearSurfaceCache.Submit();
		Handler(); // We have no choice but to have this here because of SDL
		ShadowMapScreen.Render(mainRTSubmission);
		Handler(); // We have no choice but to have this here because of SDL
		PathTrace.Render(MainFrustum.eye);
		Handler(); // We have no choice but to have this here because of SDL
		TemporalAccumulate.Render();
		Handler(); // We have no choice but to have this here because of SDL
		SpatialDenoise.Render();
		Handler(); // We have no choice but to have this here because of SDL
		Modulate.Render();
		Handler(); // We have no choice but to have this here because of SDL
		ScreenSpaceGather.Render();
		Handler(); // We have no choice but to have this here because of SDL
		NearScattering.Render();
		Handler(); // We have no choice but to have this here because of SDL
		ScreenSpaceFX.Render();
		Handler(); // We have no choice but to have this here because of SDL
		MoBlur.Render();
		Handler(); // We have no choice but to have this here because of SDL
		DoF.SetIsOnLadder(player.physics.ladderInfo.inEntryZone);
		DoF.Render(worldParams.FirstPersonControls() ? 1.0f : cameraSystem.curAlpha);
		frameInstrument.End();
		INSTRUMENTATION::FPSCounter::End();

		worldParams.AddRenderTime(worldParams.GetFrameTime());

		INSTRUMENTATION::Instrument::EnableGlobally();

		bool boxActionSignal = false;
		INSTRUMENTATION::FPSCounter::Report([&](unsigned int inpFPS) {
			boxActionSignal = true;
			LOG() << "FPS: " << inpFPS;
		});

		CommonSharedMutex.unlock_shared();
		/*bool curFire = GetStateOfKeybAction(CMD_FIRE);
		static bool prevFire = false;
		if (!curFire && prevFire)
		{
			mat4 mapOrient;
			mapOrient.Ident();
			unsigned long long droppedBoxId = physicalItemsCollection.Add("source_material/dev_test_models/boxes/box_cloth_test.3md", "source_material/dev_test_models/boxes/", mapOrient, MainFrustum.eye + MainFrustum.look * 8.0f);
			CommonSharedMutex.lock();
			physicalItemsCollection.allItems[droppedBoxId].rigidBodyRef->lin_v = MainFrustum.look * 8.0f;
			CommonSharedMutex.unlock();
		}
		prevFire = curFire;*/

		bool curMemreportKey = GetStateOfAction(CMD_MEMREPORT);
		static bool prevMemreportKey = false;
		if (!curMemreportKey && prevMemreportKey)
		{
			MEMORY_MANAGER::LogMemUsageStats();
		}
		prevMemreportKey = curMemreportKey;

		zoneStreaming.Update(MainFrustum.eye);
		static bool prevHitEsc = false;
		bool hitEsc = GetStateOfAction(CMD_MAIN_MENU);
		if (!hitEsc && prevHitEsc)
		{
			prevHitEsc = hitEsc;
			if (clairAudio) AudioSystem.Pause(clairAudio);
			CommonSharedMutex.lock();
			MainMenu.SetNotDone();
			while (!MainMenu.IsDone())
			{
				Handler(); // We have no choice but to have this here because of SDL
				MainMenu.Render(true,
					[&]() {
						zoneStreaming.waitOnZoneProduction();
						mainRTSubmission.DeleteRTResources();
						RTInstance::Disable();
						guidedModelSystem.UpdateSDFs();
						plasteredItemsCollection.UpdateSDFs();
					},
					[&]() {
						zoneStreaming.waitOnZoneProduction();
						sdfBvhSubmission.DeleteSDFBVHResources();
						RTInstance::Enable(Instance);
					});
			}
			CommonSharedMutex.unlock();
			if (clairAudio) AudioSystem.Resume(clairAudio);
			worldParams.StartFrameTimer();
		}
		else
			prevHitEsc = hitEsc;

		worldParams.EndFrameTimer();
	}
	DestroyWorld();

	return potentialMapChangeInfo;
}

bool HIGHOMEGA::WORLD::DefaultPipelineSetupClass::IsApplicationQuitting()
{
	return ApplicationQuitting;
}