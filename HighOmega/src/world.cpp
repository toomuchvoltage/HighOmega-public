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

#include <world.h>

using namespace HIGHOMEGA::FIZ_X;
using namespace HIGHOMEGA::GEOM;
using namespace HIGHOMEGA::ENTITIES;
using namespace HIGHOMEGA::RENDER::PASSES;
using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;

std::thread *HIGHOMEGA::WORLD::PhysicsThread = nullptr;
std::thread *HIGHOMEGA::WORLD::EntitiesThread = nullptr;
std::thread *HIGHOMEGA::WORLD::ClothThread = nullptr;
std::thread *HIGHOMEGA::WORLD::AudioThread = nullptr;

extern HIGHOMEGA::INSTRUMENTATION::Instrument updateInstrument, traceInstrument, frameInstrument;
HIGHOMEGA::WORLD::minMaxReductionClass HIGHOMEGA::WORLD::mainMinMaxReducer;

vec3 HIGHOMEGA::WORLD::FindCenter(Mesh & inpMesh, std::function<bool(int, DataGroup &)> inpFilterFunction)
{
	vec3 newCenter(0.0f);
	int newCenterCount = 0;

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];
		if (!inpFilterFunction(i, curPolyGroup)) continue;

		DataBlock *triBlock;
		if ( !Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock)) continue;

		vec3 edge, vnorm, vcol;
		vec2 uv;
		for (int j = 0; j != triBlock->blob.size() / sizeof(RasterVertex); j++)
		{
			unpackRasterVertex (edge, vcol, uv, vnorm, ((RasterVertex *)triBlock->blob.data())[j]);
			newCenter += edge;
			newCenterCount++;
		}
	}
	return newCenter / (float)newCenterCount;
}

HIGHOMEGA::WORLD::ZoneStreamingClass HIGHOMEGA::WORLD::zoneStreaming;
HIGHOMEGA::WORLD::PlasteredItemsClass HIGHOMEGA::WORLD::plasteredItemsCollection;
HIGHOMEGA::WORLD::AnimatedMeshesClass HIGHOMEGA::WORLD::animatedMeshesCollection;
HIGHOMEGA::WORLD::PhysicalItemClass HIGHOMEGA::WORLD::physicalItemsCollection;
HIGHOMEGA::WORLD::ParticleSystemClass HIGHOMEGA::WORLD::particleSystem;
HIGHOMEGA::WORLD::CameraSystemClass HIGHOMEGA::WORLD::cameraSystem;
HIGHOMEGA::WORLD::GuidedModelSystemClass HIGHOMEGA::WORLD::guidedModelSystem;
HIGHOMEGA::RENDER::WorldParamsClass HIGHOMEGA::WORLD::worldParams;

unsigned int HIGHOMEGA::WORLD::minMaxReductionClass::WorkGroupMinMaxX()
{
	return 64;
}

unsigned long long HIGHOMEGA::WORLD::minMaxReductionClass::RequestMinMax(GeometryClass * inGeom)
{
	unsigned long long requestId = mersenneTwister64BitPRNG();

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
				geomParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, paramAlignmentSize);
				geomIds.reserve(itemCountAligned);
				geomParams.reserve(itemCountAligned);
			}
		}
		else
		{
			if (geomParamsBuf->getSize() == 0) geomParamsBuf->Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, sizeof(minMaxParamsStruct));
		}

		geomIds.clear();
		geomParams.clear();

		std::vector <ShaderResource> allGeomResourceSet;
		for (std::pair<const unsigned long long, geomInfoStruct> & curGeomInfo : geomInfo)
		{
			geomIds.push_back(curGeomInfo.first);
			geomParams.emplace_back();
			unsigned int numTris = curGeomInfo.second.geom->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
			geomParams.back().maxValueNumTris[3] = *((float *)&numTris);
			allGeomResourceSet.emplace_back(RESOURCE_SSBO, COMPUTE, 0, 0, curGeomInfo.second.geom->getVertBuffer());
		}
		minMaxShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, allGeomResourceSet);
		geomParamsBuf->UploadSubData(0, geomParams.data(), (unsigned int)geomParams.size() * sizeof(minMaxParamsStruct));
		minMaxShader->AddResource(RESOURCE_SSBO, COMPUTE, 1, 0, *geomParamsBuf);
		minMaxShader->Create("shaders/minMaxGeom.comp.spv", "main");
		if (init) minMaxSubmission->RemoveDispatch(Instance, std::string("default"));
		minMaxSubmission->MakeDispatch(Instance, std::string ("default"), *minMaxShader, 1, (unsigned int)allGeomResourceSet.size(), 1);
		init = true;
		modified = false;
	}
	minMaxSubmission->makeSerial();
	minMaxSubmission->Submit();

	geomParamsBuf->DownloadSubData(0, geomParams.data(), (unsigned int)geomParams.size() * sizeof(minMaxParamsStruct));
	for (unsigned int i = 0; i != geomParams.size(); i++)
	{
		vec3 minEval = vec3(geomParams[i].minValue[0], geomParams[i].minValue[1], geomParams[i].minValue[2]);
		vec3 maxEval = vec3(geomParams[i].maxValueNumTris[0], geomParams[i].maxValueNumTris[1], geomParams[i].maxValueNumTris[2]);
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
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::Combine(std::vector <PlasteredItemsClass> & plasteredClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall)
{
	if (!integrateSubmission) integrateSubmission = new ComputeSubmission;
	if (!transformCollectionSubmission) transformCollectionSubmission = new ComputeSubmission;

	submissionsForPlasteredItems = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	for (PlasteredItemsClass & curPlasteredClass : plasteredClasses)
	{
		for (std::pair<const std::string, MeshModel> & curMeshModel : curPlasteredClass.cachedMeshesModels)
			if (cachedMeshesModels.find(curMeshModel.first) == cachedMeshesModels.end())
				cachedMeshesModels[curMeshModel.first] = curMeshModel.second;
		curPlasteredClass.cachedMeshesModels.clear();

		if (curPlasteredClass.integrateSubmission) delete curPlasteredClass.integrateSubmission;
		if (curPlasteredClass.transformCollectionSubmission) delete curPlasteredClass.transformCollectionSubmission;
		curPlasteredClass.integrateSubmission = nullptr;
		curPlasteredClass.transformCollectionSubmission = nullptr;

		for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionsIdPair : curPlasteredClass.allCollections)
		{
			allCollections[curCollectionsIdPair.first] = curCollectionsIdPair.second;

			for (plasterCollection & curPlasterCollection : curCollectionsIdPair.second)
			{
				std::string integrateDispatchKey = std::string("plastered_") + std::to_string(curCollectionsIdPair.first) + std::string("_") + std::to_string(curPlasterCollection.meshGroup) + std::string("_") + std::to_string(curPlasterCollection.plasterGroup);
				unsigned int meshCount = curPlasterCollection.transformInstancesSource->getSize() / sizeof(transformSourceData);
				integrateSubmission->MakeDispatch(Instance, integrateDispatchKey, *curPlasterCollection.integrateShader, (unsigned int)ceil((double)meshCount / (double)WorkGroupIntegrateX()), 1, 1);
				for (int i = 0; i != curPlasterCollection.sourceGeom.size(); i++)
				{
					unsigned int numTris = curPlasterCollection.sourceGeom[i]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					std::string transformDispatchKey = std::string("plasteredIntegrate_") + std::to_string(curCollectionsIdPair.first) + std::string("_") + std::to_string(curPlasterCollection.meshGroup) + std::string("_") + std::to_string(curPlasterCollection.plasterGroup) + std::string("_") + std::to_string(i);
					transformCollectionSubmission->MakeDispatch(Instance, transformDispatchKey, *curPlasterCollection.transformCollectionShader[i], (unsigned int)ceil((double)numTris / (double)WorkGroupTransformX()), (unsigned int)ceil((double)meshCount / (double)WorkGroupTransformY()), 1);
				}
			}
		}
		curPlasteredClass.allCollections.clear();
		curPlasteredClass.curTimeElapsed = 0.0f;
	}
}

unsigned long long HIGHOMEGA::WORLD::PlasteredItemsClass::Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*>&& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducer)
{
	submissionsForPlasteredItems = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	unsigned long long curPlasterId = mersenneTwister64BitPRNG();

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
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterSizeMin" + std::to_string(j), plasterSizeMin)) plasterSizeMin = 0.001f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterSizeMax" + std::to_string(j), plasterSizeMax)) plasterSizeMax = 1.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterItemsWavePhase" + std::to_string(j), curCollection.wavePhase)) curCollection.wavePhase = 0.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterItemsWaveAmplitude" + std::to_string(j), curCollection.waveAmplitude)) curCollection.waveAmplitude = 0.0f;

			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "plasterMeshCount" + std::to_string(j), meshCountFloat)) throw std::runtime_error("Could not get plaster mesh count");
			unsigned int meshCount = (unsigned int)meshCountFloat;

			std::string plasterMesh, plasterMeshAssetLoc;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "plasterMesh" + std::to_string(j), plasterMesh) ||
				 !Mesh::getDataRowString(curPolyGroup, "PROPS", "plasterMeshAssetLoc" + std::to_string(j), plasterMeshAssetLoc) ) throw std::runtime_error("Could not get plastered mesh");

			if (cachedMeshesModels.find(plasterMesh) == cachedMeshesModels.end())
			{
				cachedMeshesModels[plasterMesh].mesh = Mesh(plasterMesh);
				cachedMeshesModels[plasterMesh].model = new GraphicsModel(cachedMeshesModels[plasterMesh].mesh, plasterMeshAssetLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
					float tmpFloat;
					return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
				}, nullptr, true, true);
			}
			curCollection.meshModelKey = plasterMesh;

			curCollection.collectionModel = new GraphicsModel(cachedMeshesModels[plasterMesh].mesh, plasterMeshAssetLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
				float tmpFloat;
				return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
			}, nullptr, true, false);
			curCollection.collectionModel->BlankAndResize(meshCount);

			std::vector<transformSourceData> transformSourceMats;
			transformSourceMats.reserve(meshCount);

			for (int k = 0; k != meshCount; k++)
			{
				unsigned int numPrims = (unsigned int)triBlock->blob.size() / (sizeof(RasterVertex) * 3);
				unsigned int pickPrim = (unsigned int)(((float)rand() / (float)RAND_MAX) * numPrims);
				if (pickPrim == numPrims) pickPrim--;

				vec3 v1, v2, v3;
				vec2 uv;
				vec3 vnorm, vcol;
				vec3 side1, side2, side3, faceNormal;

				unpackRasterVertex(v1, vcol, uv, vnorm, ((RasterVertex *)triBlock->blob.data())[pickPrim * 3]);
				unpackRasterVertex(v2, vcol, uv, vnorm, ((RasterVertex *)triBlock->blob.data())[pickPrim * 3 + 1]);
				unpackRasterVertex(v3, vcol, uv, vnorm, ((RasterVertex *)triBlock->blob.data())[pickPrim * 3 + 2]);

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
				UnpackMat4 (curMat, &transformSourceMats.back());
			}

			if (!integrateSubmission) integrateSubmission = new ComputeSubmission;
			if (!transformCollectionSubmission) transformCollectionSubmission = new ComputeSubmission;

			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[0] = *((float *)&meshCount);
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[1] = curCollection.waveAmplitude;
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[2] = curCollection.wavePhase;
			curCollection.integrateParams.InstanceCountAmplitudePhaseCurTime[3] = 0.0f;

			curCollection.transformInstancesSource = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, transformSourceMats.data(), meshCount * (unsigned int)sizeof(transformSourceData));
			curCollection.transformInstances = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, meshCount * (unsigned int)sizeof(transformSourceData) * 2); // mat and matDT
			curCollection.integrateParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &curCollection.integrateParams, (unsigned int)sizeof(curCollection.integrateParams));
			curCollection.integrateShader = new ShaderResourceSet;
			curCollection.integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *curCollection.transformInstancesSource);
			curCollection.integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, *curCollection.transformInstances);
			curCollection.integrateShader->AddResource(RESOURCE_UBO, COMPUTE, 0, 2, *curCollection.integrateParamsBuf);
			curCollection.integrateShader->Create("shaders/integratePlastered.comp.spv", "main");
			std::string integrateDispatchKey = std::string("plastered_") + std::to_string(curPlasterId) + std::string("_") + std::to_string(i) + std::string("_") + std::to_string(j);
			curCollection.meshGroup = i;
			curCollection.plasterGroup = j;
			integrateSubmission->MakeDispatch(Instance, integrateDispatchKey, *curCollection.integrateShader, (unsigned int)ceil((double)meshCount / (double)WorkGroupIntegrateX()), 1, 1);

			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & collectionMaterialGeomMap = curCollection.collectionModel->MaterialGeomMap;
			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = cachedMeshesModels[curCollection.meshModelKey].model->MaterialGeomMap;
			for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = collectionMaterialGeomMap.begin(); it != collectionMaterialGeomMap.end(); ++it)
			{
				for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
					curCollection.sourceGeom.push_back(&(*it2));
				for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
					curCollection.destGeom.push_back(&(*it2));
			}
			for (unsigned int k = 0; k != curCollection.sourceGeom.size(); k++)
			{
				std::string transformDispatchKey = std::string("plasteredIntegrate_") + std::to_string(curPlasterId) + std::string("_") + std::to_string(i) + std::string("_") + std::to_string(j) + std::string("_") + std::to_string(k);
				unsigned int numTris = curCollection.sourceGeom[k]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
				transformParams.triCountInstanceCount[0] = numTris;
				transformParams.triCountInstanceCount[1] = meshCount;
				curCollection.transformParamsBuf.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &transformParams, (unsigned int)sizeof(transformParamsStruct)));
				curCollection.transformCollectionShader.push_back(new ShaderResourceSet);
				ShaderResourceSet *curShaderResourceSet = curCollection.transformCollectionShader.back();
				curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, curCollection.sourceGeom[k]->getVertBuffer());
				curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, curCollection.destGeom[k]->getVertBuffer());
				curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, *(curCollection.transformInstances));
				curShaderResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 3, *(curCollection.transformParamsBuf.back()));
				curShaderResourceSet->Create("shaders/transformPlastered.comp.spv", "main");
				transformCollectionSubmission->MakeDispatch(Instance, transformDispatchKey, *curShaderResourceSet, (unsigned int)ceil((double)numTris / (double)WorkGroupTransformX()), (unsigned int)ceil((double)meshCount / (double)WorkGroupTransformY()), 1);

				inpMinMaxReducer.RequestMinMax(curCollection.destGeom[k]);
			}

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

	if (integrateSubmission)
	{
		integrateSubmission->makeSerial();
		integrateSubmission->Submit();
		transformCollectionSubmission->makeSerial();
		transformCollectionSubmission->Submit();
	}

	for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionPair : allCollections)
		for (plasterCollection & curCollection : curCollectionPair.second)
			if (curCollection.allSubmissionInfos.size() == 0)
				for (GroupedRenderSubmission *curSubmission : submissionsForPlasteredItems)
					curCollection.allSubmissionInfos.emplace_back(perSubmissionCall (curSubmission, curCollection.collectionModel));
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::UpdateSDFs()
{
	std::vector<SubmittedRenderItem> sris;
	for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionPair : allCollections)
		for (plasterCollection & curCollection : curCollectionPair.second)
		{
			SubmittedRenderItem sri;
			sri.item = curCollection.collectionModel;
			sris.push_back(sri);
		}
	GraphicsModel::UpdateSDFs(sris);
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::Remove(unsigned long long curId)
{
	if (allCollections.find(curId) == allCollections.end()) return;

	for (plasterCollection & curCollection : allCollections[curId])
	{
		for (SubmittedRenderItem & curSubmission : curCollection.allSubmissionInfos)
			curSubmission.producer->Remove(curSubmission);
		for (unsigned int i = 0; i != curCollection.sourceGeom.size(); i++)
		{
			delete curCollection.transformParamsBuf[i];
			delete curCollection.transformCollectionShader[i];
		}

		delete curCollection.transformInstancesSource;
		delete curCollection.transformInstances;
		delete curCollection.integrateParamsBuf;
		delete curCollection.integrateShader;

		delete curCollection.collectionModel;

		if (integrateSubmission)
		{
			std::string integrateKey = std::string("plastered_") + std::to_string(curId) + std::string("_") + std::to_string(curCollection.meshGroup) + std::string("_") + std::to_string(curCollection.plasterGroup);
			integrateSubmission->RemoveDispatch(Instance, integrateKey);
		}

		if (transformCollectionSubmission)
			for (int i = 0; i != curCollection.sourceGeom.size(); i++)
			{
				std::string transformKey = std::string("plasteredIntegrate_") + std::to_string(curId) + std::string("_") + std::to_string(curCollection.meshGroup) + std::string("_") + std::to_string(curCollection.plasterGroup) + std::string("_") + std::to_string(i);
				transformCollectionSubmission->RemoveDispatch(Instance, transformKey);
			}
	}

	allCollections.erase(curId);
}

void HIGHOMEGA::WORLD::PlasteredItemsClass::ClearContent()
{
	for (std::pair<const unsigned long long, std::vector<plasterCollection>> & curCollectionPair : allCollections)
		for (plasterCollection & curCollection : curCollectionPair.second)
		{
			for (SubmittedRenderItem & curSubmission : curCollection.allSubmissionInfos)
				curSubmission.producer->Remove(curSubmission);
			for (unsigned int i = 0; i != curCollection.sourceGeom.size(); i++)
			{
				delete curCollection.transformParamsBuf[i];
				delete curCollection.transformCollectionShader[i];
			}
		
			delete curCollection.transformInstancesSource;
			delete curCollection.transformInstances;
			delete curCollection.integrateParamsBuf;
			delete curCollection.integrateShader;

			delete curCollection.collectionModel;
		}
	allCollections.clear();

	for (std::pair<const std::string, MeshModel> & curCachedMeshModel : cachedMeshesModels)
		delete curCachedMeshModel.second.model;
	cachedMeshesModels.clear();

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

void HIGHOMEGA::WORLD::ZoneStreamingClass::getTileNums(vec3 inpPos, int & tileX, int & tileY, int & tileZ)
{
	inpPos /= unitsPerTileEdge();
	tileX = (int)floor (inpPos.x);
	tileY = (int)floor (inpPos.y);
	tileZ = (int)floor (inpPos.z);
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::produceZones(ZoneStreamingClass * zoneStreamingPtr, unsigned int threadId)
{
	try {
		mat4 tmpOrient;
		tmpOrient.Ident();
		vec3 tmpPos = vec3(0.0f);
		for (unsigned int i = 0; i != zoneStreamingPtr->foundZonesCached.size(); i++)
		{
			if (i % HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT != threadId) continue;
			std::string& foundZone = zoneStreamingPtr->foundZonesCached[i];
			bool foundZoneLoaded;
			{std::unique_lock<std::mutex> lk(zoneStreamingPtr->zone_producer_mutex);
			foundZoneLoaded = (zoneStreamingPtr->loadedZones.find(foundZone) != zoneStreamingPtr->loadedZones.end()); }
			if (foundZoneLoaded) continue;
			Mesh* newMesh = new Mesh(zoneStreamingPtr->zoneLocation + foundZone + std::string(".3md"));
			GraphicsModel* graphicsModel;
			RigidBody* rigidBody;
			std::string placedMesh;
			if (newMesh->DataGroups.size() == 1 && Mesh::getDataRowString(newMesh->DataGroups[0], "PROPS", "placedMesh", placedMesh)) {
				std::string placedMeshAssetLoc;
				Mesh::getDataRowString(newMesh->DataGroups[0], "PROPS", "placedMeshAssetLoc", placedMeshAssetLoc);
				mat4 placedMeshTransform;
				Mesh::getDataRowMat4(newMesh->DataGroups[0], "PROPS", "placedMeshTransform", placedMeshTransform);
				delete newMesh;
				vec3 axisX = vec3(placedMeshTransform.i[0][0], placedMeshTransform.i[1][0], placedMeshTransform.i[2][0]).normalized();
				vec3 axisY = vec3(placedMeshTransform.i[0][1], placedMeshTransform.i[1][1], placedMeshTransform.i[2][1]).normalized();
				vec3 axisZ = vec3(placedMeshTransform.i[0][2], placedMeshTransform.i[1][2], placedMeshTransform.i[2][2]).normalized();
				placedMeshTransform.i[0][0] = axisX.x; placedMeshTransform.i[1][0] = axisX.y; placedMeshTransform.i[2][0] = axisX.z; placedMeshTransform.i[3][0] = 0.0f;
				placedMeshTransform.i[0][1] = axisY.x; placedMeshTransform.i[1][1] = axisY.y; placedMeshTransform.i[2][1] = axisY.z; placedMeshTransform.i[3][1] = 0.0f;
				placedMeshTransform.i[0][2] = axisZ.x; placedMeshTransform.i[1][2] = axisZ.y; placedMeshTransform.i[2][2] = axisZ.z; placedMeshTransform.i[3][2] = 0.0f;
				placedMeshTransform.i[3][3] = 0.0f;
				std::string meshPrependString = foundZone + ":";
				newMesh = new Mesh(placedMesh, &meshPrependString);
				TransformMesh(*newMesh, placedMeshTransform);
				graphicsModel = new GraphicsModel(*newMesh, placedMeshAssetLoc, Instance);
				rigidBody = new RigidBody(*newMesh, placedMeshAssetLoc, tmpOrient, tmpPos, true);
			}
			else {
				graphicsModel = new GraphicsModel(*newMesh, zoneStreamingPtr->zoneLocation, Instance);
				rigidBody = new RigidBody(*newMesh, zoneStreamingPtr->zoneLocation, tmpOrient, tmpPos, true);
			}
			std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> passThroughSubmit =
				[](GroupedRenderSubmission*, GraphicsModel*) -> SubmittedRenderItem
			{
				return SubmittedRenderItem();
			};
			unsigned long long plasteredId = zoneStreamingPtr->plasteredItemsLoaders[threadId].Populate(*newMesh, {}, passThroughSubmit, zoneStreamingPtr->perLoaderReducer[threadId]);
			unsigned long long particlesId = zoneStreamingPtr->particleSystemLoaders[threadId].Populate(*newMesh, {}, passThroughSubmit, zoneStreamingPtr->perLoaderReducer[threadId]);
			unsigned long long cameraRailId = zoneStreamingPtr->cameraSystemLoaders[threadId].Populate(*newMesh);
			unsigned long long guidedModelsId = zoneStreamingPtr->guidedModelLoaders[threadId].Populate(*newMesh, {}, passThroughSubmit);
			unsigned long long worldParamsId = zoneStreamingPtr->worldParamsLoaders[threadId].Populate(*newMesh);
			unsigned long long laddersId = zoneStreamingPtr->ladderSystemLoaders[threadId].Populate(*newMesh);
			{std::unique_lock<std::mutex> lk(zoneStreamingPtr->zone_producer_mutex);
			zoneStreamingPtr->loadedZones[foundZone].worldParamsId = worldParamsId;
			zoneStreamingPtr->loadedZones[foundZone].graphicsModel = graphicsModel;
			zoneStreamingPtr->loadedZones[foundZone].rigidBody = rigidBody;
			zoneStreamingPtr->loadedZones[foundZone].plasteredId = plasteredId;
			zoneStreamingPtr->loadedZones[foundZone].particlesId = particlesId;
			zoneStreamingPtr->loadedZones[foundZone].guidedModelsId = guidedModelsId;
			zoneStreamingPtr->loadedZones[foundZone].cameraRailId = cameraRailId;
			zoneStreamingPtr->loadedZones[foundZone].laddersId = laddersId;
			zoneStreamingPtr->rigidBodiesToAdd.push_back(zoneStreamingPtr->loadedZones[foundZone].rigidBody); }
			physicalItemsCollection.ProcessDestructionCache(*newMesh, *graphicsModel, *rigidBody);
			delete newMesh;
		}
		zoneStreamingPtr->plasteredItemsLoaders[threadId].Update(0.0f);
		zoneStreamingPtr->particleSystemLoaders[threadId].Update(0.0f);
		zoneStreamingPtr->perLoaderReducer[threadId].Process();
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

			std::vector<SubmittedRenderItem> sri;
			for (unsigned int i = 0; i != zoneStreamingPtr->foundZonesCached.size(); i++)
			{
				std::string& foundZone = zoneStreamingPtr->foundZonesCached[i];
				SubmittedRenderItem curSri;
				curSri.item = zoneStreamingPtr->loadedZones[foundZone].graphicsModel;
				sri.push_back(curSri);
			}

			GraphicsModel::UpdateSDFs(sri);
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

void HIGHOMEGA::WORLD::ZoneStreamingClass::Create(std::string & inpZoneLocation, std::vector<GroupedRenderSubmission*> &&inpSubmissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall)
{
	zoneLocation = inpZoneLocation;
	submissionList = inpSubmissionList;
	perSubmissionCall = inpPerSubmissionCall;
	zoneStreamingActivated = true;

	Mesh zoneReferencesMesh = Mesh(inpZoneLocation + "zones.3md");
	DataBlock *zoneRefBlock;
	if (!Mesh::getDataBlock(zoneReferencesMesh.DataGroups[0], "DESCRIPTION", &zoneRefBlock))
		throw std::runtime_error("Could not get zone references from zone ref 3md");

	for (unsigned int i = 0; i != zoneRefBlock->rows.size(); i++)
	{
		std::string & zoneName = zoneRefBlock->rows[i][0].svalRef();
		int numRefs;
		if (!zoneRefBlock->rows[i][1].ivalue(numRefs))
			throw std::runtime_error("Malformed zone ref 3md");
		for (unsigned int j = 0; j != numRefs; j++)
			zoneReferences[zoneName].push_back(zoneRefBlock->rows[i][2 + j].svalRef());
	}

	Update(vec3(0.0f));
}

vec3 & HIGHOMEGA::WORLD::ZoneStreamingClass::getVisbileMax()
{
	return visibleMax;
}

vec3 & HIGHOMEGA::WORLD::ZoneStreamingClass::getVisbileMin()
{
	return visibleMin;
}

void HIGHOMEGA::WORLD::ZoneStreamingClass::Update(const vec3 & inpPos)
{
	if (!zoneStreamingActivated) return;

	if (!producingZones && !allZonesProduced())
	{
		int curTileX, curTileY, curTileZ;
		curPos = inpPos;
		curPos = inpPos;
		getTileNums(curPos, curTileX, curTileY, curTileZ);

		std::vector<std::string> foundZones;

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
							if ( std::find (foundZones.begin(), foundZones.end(), zoneReferences[checkZoneName][l]) == foundZones.end() )
								foundZones.push_back(zoneReferences[checkZoneName][l]);
				}

		if (lastZonesSaved && foundZones == foundZonesCached) return;
		foundZonesCached = foundZones;
		lastZonesSaved = true;
	}

	if (!producingZones)
	{
		producingZones = true;
		rigidBodiesToAdd.clear();
		setAllZonesNotProduced();
		zoneProducerThread.resize(HIGHOMEGA_ZONE_STREAMING_THREAD_COUNT);
		for (unsigned int i = 0; i != (unsigned int)zoneProducerThread.size(); i++)
			zoneProducerThread[i] = new std::thread(produceZones, this, i);
		if (!producedZonesOnce)
		{
			producedZonesOnce = true;
			waitOnZoneProduction();
		}
		else return;
	}
	if (allZonesProduced() && !PhysicalItemClass::booleanOpThread) // Do not evict, mid destruction
	{
		waitOnZoneProduction();
		std::vector<std::string> zonesToRemove;
		for (std::pair <const std::string, zone> & curZone : loadedZones)
		{
			bool curZoneApproved = false;
			for (std::string & foundZone : foundZonesCached)
				if (curZone.first == foundZone)
				{
					curZoneApproved = true;
					break;
				}
			if (!curZoneApproved)
				zonesToRemove.push_back(curZone.first);
		}

		for (std::string & curZoneToRemove : zonesToRemove)
		{
			for (int i = 0; i != loadedZones[curZoneToRemove].allSubmissionInfos.size(); i++)
				loadedZones[curZoneToRemove].allSubmissionInfos[i].producer->Remove(loadedZones[curZoneToRemove].allSubmissionInfos[i]);
			delete loadedZones[curZoneToRemove].graphicsModel;
			if (loadedZones[curZoneToRemove].itemId != 0ull) physicalItemsCollection.RemoveExternal(loadedZones[curZoneToRemove].itemId);
			if (loadedZones[curZoneToRemove].plasteredId != 0ull) plasteredItemsCollection.Remove(loadedZones[curZoneToRemove].plasteredId);
			if (loadedZones[curZoneToRemove].particlesId != 0ull) particleSystem.Remove(loadedZones[curZoneToRemove].particlesId);
			if (loadedZones[curZoneToRemove].guidedModelsId != 0ull) guidedModelSystem.Remove(loadedZones[curZoneToRemove].guidedModelsId);
			if (loadedZones[curZoneToRemove].cameraRailId != 0ull) cameraSystem.Remove(loadedZones[curZoneToRemove].cameraRailId);
			if (loadedZones[curZoneToRemove].worldParamsId != 0ull) worldParams.Remove(loadedZones[curZoneToRemove].worldParamsId);
		}

		for (std::pair<const std::string, zone> & curLoadedZone : loadedZones)
		{
			if (curLoadedZone.second.allSubmissionInfos.size() == 0)
			{
				for (int i = 0; i != submissionList.size(); i++)
					curLoadedZone.second.allSubmissionInfos.push_back(perSubmissionCall (submissionList[i], curLoadedZone.second.graphicsModel));
				curLoadedZone.second.itemId = physicalItemsCollection.AddExternal(curLoadedZone.second.rigidBody, curLoadedZone.second.graphicsModel, curLoadedZone.second.allSubmissionInfos);
			}
		}
		cameraSystem.Combine(cameraSystemLoaders);
		plasteredItemsCollection.Combine(plasteredItemsLoaders, submissionList, perSubmissionCall);
		particleSystem.Combine(particleSystemLoaders, submissionList, perSubmissionCall);
		guidedModelSystem.Combine(guidedModelLoaders, submissionList, perSubmissionCall);
		worldParams.Combine(worldParamsLoaders);

		CommonSharedMutex.lock();
		for (std::string & curZoneToRemove : zonesToRemove)
		{
			std::vector<RigidBody *>::iterator curBody = std::find(allBodies.begin(), allBodies.end(), loadedZones[curZoneToRemove].rigidBody);
			if (curBody != allBodies.end())
				allBodies.erase(curBody);
			if (loadedZones[curZoneToRemove].laddersId != 0ull) mainLadderSystem.Remove(loadedZones[curZoneToRemove].laddersId);
		}
		mainLadderSystem.Combine(ladderSystemLoaders);
		CommonSharedMutex.unlock();

		for (std::string & curZoneToRemove : zonesToRemove)
		{
			delete loadedZones[curZoneToRemove].rigidBody;
			loadedZones.erase(curZoneToRemove);
		}

		CommonSharedMutex.lock();
		for (RigidBody *bodyToAdd : rigidBodiesToAdd)
			allBodies.push_back(bodyToAdd);
		CommonSharedMutex.unlock();

		bool firstMinMax = true;
		for (std::pair<const std::string, zone> & curZone : loadedZones)
		{
			if (curZone.second.graphicsModel->MaterialGeomMap.size() == 0) continue;
			vec3 curMin, curMax;
			curZone.second.graphicsModel->getModelMinMax(curMin, curMax);
			if (firstMinMax)
			{
				visibleMin = curMin;
				visibleMax = curMax;
				firstMinMax = false;
			}
			else
			{
				visibleMin.x = min(curMin.x, visibleMin.x);
				visibleMin.y = min(curMin.y, visibleMin.y);
				visibleMin.z = min(curMin.z, visibleMin.z);
				visibleMax.x = max(curMax.x, visibleMax.x);
				visibleMax.y = max(curMax.y, visibleMax.y);
				visibleMax.z = max(curMax.z, visibleMax.z);
			}
		}

		producingZones = false;
		setAllZonesNotProduced();
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
	foundZonesCached.clear();
	rigidBodiesToAdd.clear();
	lastZonesSaved = false;
	producedZonesOnce = false;

	waitOnZoneProduction();

	producingZones = false;
	setAllZonesNotProduced();

	for (std::pair<const std::string, zone> & curLoadedZone : loadedZones)
	{
		zone & curZone = curLoadedZone.second;
		for (int i = 0; i != curZone.allSubmissionInfos.size(); i++)
			curZone.allSubmissionInfos[i].producer->Remove(curZone.allSubmissionInfos[i]);
		delete curZone.graphicsModel;
		std::vector<RigidBody *>::iterator curBody = std::find(allBodies.begin(), allBodies.end(), curZone.rigidBody);
		if (curBody != allBodies.end())
			allBodies.erase(curBody);
		delete curZone.rigidBody;
	}
	loadedZones.clear();
	curPos = vec3(0.0f);
	zoneLocation = "";
	submissionList.clear();
	loadedZones.clear();
}

unsigned int HIGHOMEGA::WORLD::AnimatedMeshesClass::WorkGroupAnimateX()
{
	return 32;
}

unsigned long long HIGHOMEGA::WORLD::AnimatedMeshesClass::Add(std::string animatedModelFile, std::string animatedModelFileLoc, float duration, mat4 & inpTransMat, std::vector<RagDollPiece>* inpRagDollPieces, std::vector<GroupedRenderSubmission*>&& subList)
{
	if (cachedMeshesModels.find(animatedModelFile) == cachedMeshesModels.end())
	{
		cachedMeshesModels[animatedModelFile].mesh = Mesh(animatedModelFile);
		cachedMeshesModels[animatedModelFile].model = new GraphicsModel(cachedMeshesModels[animatedModelFile].mesh, animatedModelFileLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
			float tmpFloat;
			std::string tmpString;
			return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowString(inpGroup, "PROPS", "ragDollPieceTransform", tmpString);
		}, nullptr, true, true, true);
	}

	unsigned long long animationId = mersenneTwister64BitPRNG();
	animatedModelItem & newModel = allAnimatedModelItems[animationId];
	if (inpRagDollPieces)
	{
		newModel.ragDollPieces = *inpRagDollPieces;
		newModel.ragDoll = true;
	}
	else
		newModel.ragDoll = false;
	newModel.meshModelRef = &cachedMeshesModels[animatedModelFile];
	newModel.duration = duration;
	newModel.curAnimationPos = 0.0f;
	newModel.animatedModel = new GraphicsModel(cachedMeshesModels[animatedModelFile].mesh, animatedModelFileLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		std::string tmpString;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat) && !Mesh::getDataRowString(inpGroup, "PROPS", "ragDollPieceTransform", tmpString);
	}, nullptr, true, false, true);
	newModel.submissionsForAnimatedMeshes = subList;
	newModel.transMat = inpTransMat;

	if (!animateSubmission) animateSubmission = new ComputeSubmission;
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = newModel.meshModelRef->model->MaterialGeomMap;
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & destMaterialGeomMap = newModel.animatedModel->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = destMaterialGeomMap.begin(); it != destMaterialGeomMap.end(); ++it)
	{
		for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
			newModel.sourceGeom.push_back(&(*it2));
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			newModel.destGeom.push_back(&(*it2));
	}
	for (unsigned int i = 0; i != newModel.sourceGeom.size(); i++)
	{
		unsigned int numTris = newModel.sourceGeom[i]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
		animateParams.triCount = numTris;
		newModel.animateParamsBuf.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &animateParams, (unsigned int)sizeof(animateParamsStruct)));
		newModel.animateShader.push_back(new ShaderResourceSet);
		ShaderResourceSet *curShaderResourceSet = newModel.animateShader.back();
		curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, newModel.sourceGeom[i]->getVertBuffer());
		curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, newModel.destGeom[i]->getVertBuffer());
		curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, newModel.sourceGeom[i]->getAnimInfoBuf());
		curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, *(newModel.animatedModel->armatures[newModel.sourceGeom[i]->getArmatureId()].currentPoseBuf));
		curShaderResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 4, *(newModel.animateParamsBuf.back()));
		curShaderResourceSet->Create("shaders/transformAnimated.comp.spv", "main");
		std::string animateDispatchName = "animated_" + std::to_string(animationId) + std::string("_") + std::to_string(i);
		animateSubmission->MakeDispatch(Instance, animateDispatchName, *curShaderResourceSet, (unsigned int)ceil((double)numTris / (double)WorkGroupAnimateX()), 1, 1);
	}

	return animationId;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::Advance(unsigned long long animationId, float elapseTime, bool rollOver, mat4 & inpTransMat)
{
	if (allAnimatedModelItems.find(animationId) == allAnimatedModelItems.end() || elapseTime == 0.0f) return;

	animatedModelItem & curAnimatedModel = allAnimatedModelItems[animationId];

	curAnimatedModel.curAnimationPos += elapseTime;
	if (!rollOver)
		curAnimatedModel.curAnimationPos = min(max(0.0f, curAnimatedModel.curAnimationPos), curAnimatedModel.duration);
	else
		if (curAnimatedModel.curAnimationPos > 0.0f)
			curAnimatedModel.curAnimationPos = std::fmodf(curAnimatedModel.curAnimationPos, curAnimatedModel.duration);
		else
			curAnimatedModel.curAnimationPos = curAnimatedModel.duration + std::fmodf(curAnimatedModel.curAnimationPos, curAnimatedModel.duration);
	curAnimatedModel.transMat = inpTransMat;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::GetCurPosDuration(unsigned long long animationId, float & pos, float & duration)
{
	if (allAnimatedModelItems.find(animationId) == allAnimatedModelItems.end()) return;

	pos = allAnimatedModelItems[animationId].curAnimationPos;
	duration = allAnimatedModelItems[animationId].duration;
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::Remove(unsigned long long animationId)
{
	if (allAnimatedModelItems.find(animationId) == allAnimatedModelItems.end()) return;

	animatedModelItem & curAnimatedModel = allAnimatedModelItems[animationId];
	for (SubmittedRenderItem & curSubmissionInfo : curAnimatedModel.allSubmissionInfos)
		curSubmissionInfo.producer->Remove(curSubmissionInfo);

	for (unsigned int i = 0; i != curAnimatedModel.sourceGeom.size(); i++)
	{
		std::string animateDispatchName = "animated_" + std::to_string(animationId) + std::string("_") + std::to_string(i);
		animateSubmission->RemoveDispatch(Instance, animateDispatchName);
		delete curAnimatedModel.animateParamsBuf[i];
		delete curAnimatedModel.animateShader[i];
	}

	delete curAnimatedModel.animatedModel;
	allAnimatedModelItems.erase(animationId);
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::Process()
{
	for (std::pair <const unsigned long long, animatedModelItem> & curAnimatedModelKV : allAnimatedModelItems)
	{
		animatedModelItem & curAnimatedModel = curAnimatedModelKV.second;
		for (std::pair<const std::string, AnimationAndSnapshot> & armNameToCurAnimSnapshot : curAnimatedModel.animatedModel->armatures)
		{
			AnimationAndSnapshot & animAndSnapshotRef = armNameToCurAnimSnapshot.second;
			if (curAnimatedModel.ragDoll)
			{
				vec3 ragDollCentMin, ragDollCentMax;
				bool setOnce = false;
				for (RagDollPiece & ragDollPiece : curAnimatedModel.ragDollPieces)
				{
					if (!setOnce)
					{
						ragDollCentMin = ragDollCentMax = ragDollPiece.piece->pos;
						setOnce = true;
					}
					else
					{
						ragDollCentMin = vec3(min(ragDollCentMin.x, ragDollPiece.piece->pos.x), min(ragDollCentMin.y, ragDollPiece.piece->pos.y), min(ragDollCentMin.z, ragDollPiece.piece->pos.z));
						ragDollCentMax = vec3(max(ragDollCentMax.x, ragDollPiece.piece->pos.x), max(ragDollCentMax.y, ragDollPiece.piece->pos.y), max(ragDollCentMax.z, ragDollPiece.piece->pos.z));
					}
					for (int i = 0; i != animAndSnapshotRef.currentPose.pose.size(); i++)
					{
						if (ragDollPiece.transformTarget != animAndSnapshotRef.currentPose.pose[i].name) continue;
						mat4 curTrans = ragDollPiece.piece->orient;
						curTrans.i[0][3] = ragDollPiece.piece->pos.x;
						curTrans.i[1][3] = ragDollPiece.piece->pos.y;
						curTrans.i[2][3] = ragDollPiece.piece->pos.z;
						animAndSnapshotRef.currentPose.pose[i].bone = curTrans * ragDollPiece.refMatInv;
						break;
					}
				}
				ragDollCentMin -= vec3(5.0f);
				ragDollCentMax += vec3(5.0f);
				curAnimatedModel.animatedModel->SetMinMax(ragDollCentMin, ragDollCentMax);
			}
			else
			{
				animAndSnapshotRef.anim.GetPose(curAnimatedModel.curAnimationPos / curAnimatedModel.duration, animAndSnapshotRef.currentPose);
				animAndSnapshotRef.currentPose.Mul(animAndSnapshotRef.referencePoseInv);
				animAndSnapshotRef.currentPose.Transform(curAnimatedModel.transMat);
				vec3 refMin, refMax, refCent;
				curAnimatedModel.meshModelRef->model->getModelMinMax(refMin, refMax);
				refCent = (refMin + refMax) * 0.5f;
				float refRad = (refMin - refMax).length(); // Use the diameter because the object may be off center
				refCent = curAnimatedModel.transMat * refCent; // Move the center to the translated position
				curAnimatedModel.animatedModel->SetMinMax (refCent - vec3(refRad), refCent + vec3(refRad));
			}
			curAnimatedModel.animatedModel->SetDirty();
			animAndSnapshotRef.CopyToPoseBuffer(animAndSnapshotRef.currentPose);
		}
	}

	if (animateSubmission)
	{
		animateSubmission->makeSerial();
		animateSubmission->Submit();
	}

	for (std::pair <const unsigned long long, animatedModelItem> & curAnimatedModelKV : allAnimatedModelItems)
	{
		animatedModelItem & curAnimatedModel = curAnimatedModelKV.second;
		if (curAnimatedModel.allSubmissionInfos.size() == 0)
			for (GroupedRenderSubmission *curSubmission : curAnimatedModel.submissionsForAnimatedMeshes)
				curAnimatedModel.allSubmissionInfos.emplace_back(curSubmission->Add(*curAnimatedModel.animatedModel));
	}
}

void HIGHOMEGA::WORLD::AnimatedMeshesClass::ClearContent()
{
	for (std::pair <const unsigned long long, animatedModelItem> & curAnimatedModelKV : allAnimatedModelItems)
	{
		animatedModelItem & curAnimatedModel = curAnimatedModelKV.second;
		for (SubmittedRenderItem & curSubmission : curAnimatedModel.allSubmissionInfos)
			curSubmission.producer->Remove(curSubmission);
		for (unsigned int i = 0; i != curAnimatedModel.sourceGeom.size(); i++)
		{
			delete curAnimatedModel.animateParamsBuf[i];
			delete curAnimatedModel.animateShader[i];
		}

		delete curAnimatedModel.animatedModel;
	}
	allAnimatedModelItems.clear();

	if (animateSubmission)
	{
		delete animateSubmission;
		animateSubmission = nullptr;
	}

	for (std::pair<const std::string, MeshModel> & curCachedMeshModel : cachedMeshesModels)
		delete curCachedMeshModel.second.model;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ProcessDestructionCache(Mesh & inpMesh, GraphicsModel & inpModel, RigidBody & inpBody)
{
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		DataBlock *triBlock, *propBlock;
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock) || !Mesh::getDataBlock(curPolyGroup, "PROPS", &propBlock)) continue;

		float outTmp;
		if (!Mesh::getDataRowFloat(*propBlock, "breakable", outTmp)) continue;

		std::string curGroupId = curPolyGroup.name + std::string("_") + std::to_string(i);

		bool fillingInData = false;
		{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
		if (destructibles.find(curGroupId) == destructibles.end())
			fillingInData = true; }

		if (fillingInData)
		{
			ObjectPiece & mapPiece = *inpBody.getGroupById(curGroupId);
			std::vector<TriUV> mapPieceGeom;

			vec3 eArr[3];
			vec2 uvArr[3];
			vec3 pieceMin, pieceMax;
			for (unsigned int i = 0; i != mapPiece.tris.size(); i++)
			{
				vec3 e1v = mapPiece.tris[i].e1;
				vec3 e2v = mapPiece.tris[i].e2;
				vec3 e3v = mapPiece.tris[i].e3;
				vec3 normVec = mapPiece.tris[i].n;
				vec2 e1uv = mapPiece.tris[i].uv1;
				vec2 e2uv = mapPiece.tris[i].uv2;
				vec2 e3uv = mapPiece.tris[i].uv3;
				eArr[0] = e1v;
				eArr[1] = e2v;
				eArr[2] = e3v;
				uvArr[0] = e1uv;
				uvArr[1] = e2uv;
				uvArr[2] = e3uv;
				AddTri(eArr, uvArr, normVec, mapPieceGeom);
				if (i == 0)
				{
					pieceMin.x = Min(e1v.x, e2v.x, e3v.x);
					pieceMin.y = Min(e1v.y, e2v.y, e3v.y);
					pieceMin.z = Min(e1v.z, e2v.z, e3v.z);
					pieceMax.x = Max(e1v.x, e2v.x, e3v.x);
					pieceMax.y = Max(e1v.y, e2v.y, e3v.y);
					pieceMax.z = Max(e1v.z, e2v.z, e3v.z);
				}
				else
				{
					pieceMin.x = min(pieceMin.x, Min(e1v.x, e2v.x, e3v.x));
					pieceMin.y = min(pieceMin.y, Min(e1v.y, e2v.y, e3v.y));
					pieceMin.z = min(pieceMin.z, Min(e1v.z, e2v.z, e3v.z));
					pieceMax.x = max(pieceMax.x, Max(e1v.x, e2v.x, e3v.x));
					pieceMax.y = max(pieceMax.y, Max(e1v.y, e2v.y, e3v.y));
					pieceMax.z = max(pieceMax.z, Max(e1v.z, e2v.z, e3v.z));
				}
			}

			{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
			destructibles[curGroupId].propBlock = *propBlock;
			destructibles[curGroupId].sourceTriUVList = mapPieceGeom;
			destructibles[curGroupId].sourceMin = pieceMin;
			destructibles[curGroupId].sourceMax = pieceMax;
			destructibles[curGroupId].sourceModel = &inpModel;
			destructibles[curGroupId].sourceBody = &inpBody; }
		}
		else
		{
			{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
			if (destructibles[curGroupId].sourceTriUVList.size() == 0)
			{
				inpModel.removeGroupById(curGroupId);
				inpBody.removeGroupById(curGroupId);
			}
			else
			{
				inpModel.ChangeGeom(curGroupId, destructibles[curGroupId].sourceTriUVList);
				inpBody.ChangeGeom(curGroupId, destructibles[curGroupId].sourceTriUVList);
			}
			destructibles[curGroupId].sourceModel = &inpModel;
			destructibles[curGroupId].sourceBody = &inpBody; }
		}
	}
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::Add(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos, bool isMap, std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, std::vector<GroupedRenderSubmission*> && subList)
{
	if (cachedMeshes.find(meshLoc) == cachedMeshes.end())
		cachedMeshes[meshLoc] = Mesh(meshLoc);

	RigidBodyItem createdItem;
	createdItem.external = false;

	if (isMap)
		createdItem.modelRef = new GraphicsModel(cachedMeshes[meshLoc], belong, Instance, [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
	}, nullptr, true, true);
	else
		createdItem.modelRef = new GraphicsModel(cachedMeshes[meshLoc], belong, Instance, [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
	}, nullptr, true, false);
	createdItem.rigidBodyRef = new RigidBody(cachedMeshes[meshLoc], belong, inpOrient, inpPos, isMap);
	recentlyAddedRigidModels.push_back(createdItem.modelRef);

	unsigned long long pickedId = mersenneTwister64BitPRNG();

	if (!isMap)
	{
		if (!transformRigidSubmission) transformRigidSubmission = new ComputeSubmission;
		std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = createdItem.modelRef->MaterialGeomMap;
		std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & destMaterialGeomMap = createdItem.modelRef->MaterialGeomMap;
		for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = destMaterialGeomMap.begin(); it != destMaterialGeomMap.end(); ++it)
		{
			for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
				createdItem.sourceGeom.push_back(&(*it2));
			for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
				createdItem.destGeom.push_back(&(*it2));
		}
		for (unsigned int i = 0; i != createdItem.sourceGeom.size(); i++)
		{
			unsigned int numTris = createdItem.sourceGeom[i]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
			rigidTransformParams.triCount = numTris;
			createdItem.transformParamBufs.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &rigidTransformParams, (unsigned int)sizeof(rigidTransformParamsStruct)));
			createdItem.transformShader.push_back(new ShaderResourceSet);
			ShaderResourceSet *curShaderResourceSet = createdItem.transformShader.back();
			curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, createdItem.sourceGeom[i]->getVertBuffer());
			curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, createdItem.destGeom[i]->getVertBuffer());
			curShaderResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 2, *(createdItem.transformParamBufs.back()));
			curShaderResourceSet->Create("shaders/transformRigid.comp.spv", "main");
			std::string transformDispatchName = "rigidtransform_" + std::to_string(pickedId) + std::string("_") + std::to_string(i);
			transformRigidSubmission->MakeDispatch(Instance, transformDispatchName, *curShaderResourceSet, (unsigned int)ceil((double)numTris / (double)WorkGroupTransformRigidX()), 1, 1);
		}
	}

	CommonSharedMutex.lock();
	allBodies.push_back(createdItem.rigidBodyRef);
	for (GroupedRenderSubmission *curSubmission : subList)
		createdItem.allSubmissionInfos.emplace_back(perSubmissionCall(curSubmission, createdItem.modelRef));
	allItems[pickedId] = createdItem;
	CommonSharedMutex.unlock();

	return pickedId;
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::AddExternal(RigidBody * inpBody, GraphicsModel * inpModel, std::vector<SubmittedRenderItem> & inpSubs)
{
	unsigned long long pickedId = mersenneTwister64BitPRNG();
	CommonSharedMutex.lock();
	allItems[pickedId].rigidBodyRef = inpBody;
	allItems[pickedId].modelRef = inpModel;
	allItems[pickedId].external = true;
	allItems[pickedId].allSubmissionInfos = inpSubs;
	CommonSharedMutex.unlock();

	return pickedId;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::RemoveExternal(unsigned long long inpId)
{
	CommonSharedMutex.lock();
	allItems.erase(inpId);
	CommonSharedMutex.unlock();
}

unsigned long long HIGHOMEGA::WORLD::PhysicalItemClass::Add(std::string & newGroupId, RigidBody *oobbPiece, MeshMaterial & origMeshMaterial, const std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall, const std::vector<GroupedRenderSubmission *> submissionsToSubmitTo, std::vector <TriUV> & triList)
{
	RigidBodyItem createdItem;
	createdItem.external = false;

	createdItem.modelRef = new GraphicsModel(newGroupId, origMeshMaterial, triList, true, false);
	recentlyAddedRigidModels.push_back(createdItem.modelRef);
	createdItem.rigidBodyRef = oobbPiece;

	if (!transformRigidSubmission) transformRigidSubmission = new ComputeSubmission;
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = createdItem.modelRef->MaterialGeomMap;
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & destMaterialGeomMap = createdItem.modelRef->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = destMaterialGeomMap.begin(); it != destMaterialGeomMap.end(); ++it)
	{
		for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
			createdItem.sourceGeom.push_back(&(*it2));
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			createdItem.destGeom.push_back(&(*it2));
	}

	unsigned long long pickedId = mersenneTwister64BitPRNG();

	for (unsigned int i = 0; i != createdItem.sourceGeom.size(); i++)
	{
		unsigned int numTris = createdItem.sourceGeom[i]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
		rigidTransformParams.triCount = numTris;
		createdItem.transformParamBufs.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &rigidTransformParams, (unsigned int)sizeof(rigidTransformParamsStruct)));
		createdItem.transformShader.push_back(new ShaderResourceSet);
		ShaderResourceSet *curShaderResourceSet = createdItem.transformShader.back();
		curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, createdItem.sourceGeom[i]->getVertBuffer());
		curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, createdItem.destGeom[i]->getVertBuffer());
		curShaderResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 2, *(createdItem.transformParamBufs.back()));
		curShaderResourceSet->Create("shaders/transformRigid.comp.spv", "main");
		std::string transformDispatchName = "rigidtransform_" + std::to_string (pickedId) + "_" + std::to_string(i);
		transformRigidSubmission->MakeDispatch(Instance, transformDispatchName, *curShaderResourceSet, (unsigned int)ceil((double)numTris / (double)WorkGroupTransformRigidX()), 1, 1);
	}

	CommonSharedMutex.lock();
	allBodies.push_back(createdItem.rigidBodyRef);
	for (GroupedRenderSubmission *curSubmission : submissionsToSubmitTo)
		createdItem.allSubmissionInfos.emplace_back(perSubmissionCall(curSubmission, createdItem.modelRef));
	allItems[pickedId] = createdItem;
	CommonSharedMutex.unlock();

	return pickedId;
}

unsigned int HIGHOMEGA::WORLD::PhysicalItemClass::WorkGroupTransformRigidX()
{
	return 32;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::AddRagDoll(std::string meshLoc, std::string belong, mat4 inpOrient, vec3 inpPos, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall, std::vector<GroupedRenderSubmission*> && subList)
{
	std::unordered_map <std::string, RigidBody *> limbs;

	Mesh curMesh = Mesh(meshLoc);

	std::vector<RagDollPiece> ragDollPieces;

	std::vector<unsigned long long> addedIds;
	addedIds.reserve(curMesh.DataGroups.size());
	for (int i = 0; i != curMesh.DataGroups.size(); i++)
	{
		DataGroup &curPolyGroup = curMesh.DataGroups[i];

		DataBlock *tmpTriBlock;

		std::string transformTarget;
		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &tmpTriBlock) || !Mesh::getDataRowString(curPolyGroup, "PROPS", "ragDollPieceTransform", transformTarget)) continue;

		RigidBodyItem createdItem;
		createdItem.external = false;

		std::function<bool(int, DataGroup &)> currentIndexOnlyFilter = [&i = i](int blockId, DataGroup & dataGroup) -> bool {
			if (blockId == i)
				return true;
			else
				return false;
		};
		vec3 tmpNewCenter = FindCenter(curMesh, currentIndexOnlyFilter);
		createdItem.rigidBodyRef = new RigidBody(curMesh, belong, inpOrient, inpPos, false, currentIndexOnlyFilter, &tmpNewCenter);
		RagDollPiece ragDollPiece;
		ragDollPiece.refMatInv.Ident();
		ragDollPiece.refMatInv.i[0][3] = -tmpNewCenter.x;
		ragDollPiece.refMatInv.i[1][3] = -tmpNewCenter.y;
		ragDollPiece.refMatInv.i[2][3] = -tmpNewCenter.z;
		ragDollPiece.piece = createdItem.rigidBodyRef;
		ragDollPiece.transformTarget = transformTarget;
		ragDollPieces.push_back(ragDollPiece);

		unsigned long long curId = mersenneTwister64BitPRNG();
		allItems[curId] = createdItem;
		addedIds.push_back(curId);

		limbs[curPolyGroup.name] = createdItem.rigidBodyRef;
	}

	ConstraintCollection *addedConstraints = new ConstraintCollection(curMesh, limbs, inpPos, Instance);

	CommonSharedMutex.lock();
	for (unsigned long long curId : addedIds)
	{
		allItems[curId].constaintsRef = addedConstraints;
		allBodies.push_back(allItems[curId].rigidBodyRef);
	}
	CommonSharedMutex.unlock();

	mat4 tmpMat;
	tmpMat.Ident();
	animatedMeshesCollection.Add(meshLoc, belong, 0.0f, tmpMat, &ragDollPieces, std::vector<GroupedRenderSubmission*>(subList));
}

void HIGHOMEGA::WORLD::PhysicalItemClass::CutOut(RigidBody * inpRigidBody, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList)
{
	// Destruction happens completely async...

	if (booleanOpThread) return;

	subListForDebris = subList;
	perSubmissionCallForDebris = perSubmissionCall;
	booleanOpThread = new std::thread(CutOutThread, this, groupId, hitDir, hitPoint, hitNorm, hitRadius, perSubmissionCall, subList);
	booleanOpThreadFinished = false;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Shatter(RigidBody * inpRigidBody, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList)
{
	// Destruction happens completely async...

	if (booleanOpThread) return;

	subListForDebris = subList;
	perSubmissionCallForDebris = perSubmissionCall;
	booleanOpThread = new std::thread(ShatterThread, this, groupId, hitDir, hitPoint, hitNorm, hitRadius, perSubmissionCall, subList);
	booleanOpThreadFinished = false;
}

std::thread *HIGHOMEGA::WORLD::PhysicalItemClass::booleanOpThread = nullptr;
std::atomic<bool> HIGHOMEGA::WORLD::PhysicalItemClass::booleanOpThreadFinished = false;
std::vector<HIGHOMEGA::WORLD::PhysicalItemClass::AddParams> HIGHOMEGA::WORLD::PhysicalItemClass::deferredAdds;
void HIGHOMEGA::WORLD::PhysicalItemClass::CutOutThread(PhysicalItemClass *physicalItemsCollection, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission *, GraphicsModel *)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList)
{
	std::vector<TriUV> currentMapGeom;
	DataBlock * currentMapPropBlock;
	{std::lock_guard<std::mutex> lk(physicalItemsCollection->destructibleCacheMutex);
	currentMapGeom = physicalItemsCollection->destructibles[groupId].sourceTriUVList; 
	currentMapPropBlock = &physicalItemsCollection->destructibles[groupId].propBlock; }

	std::vector<TriUV> cullGeom, ring, cap, slicedmap, tmp, ringFacingInside;
	MakeCylinder(hitPoint, hitNorm, 2.0f, 4.0f, cullGeom);
	Intersect(currentMapGeom, cullGeom, ring, tmp);
	ringFacingInside = ring;
	for (TriUV & curTri : ringFacingInside)
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
		
		std::string newGroupId = groupId;newGroupId += "_piece";newGroupId += std::to_string(mersenneTwister64BitPRNG());

		std::vector<TriUV> aabbTris;
		std::vector <intersection> tmpIntersectionList;
		GenerateAABB(groupedTris, aabbTris, ColTol);
		RigidBody *tmpRigidBodyAABB = new RigidBody(newGroupId, *currentMapPropBlock, aabbTris, newCenter);
		tmpRigidBodyAABB->ang_v = vec3((rand() % 100 - 50) * 0.02f) * 0.2f;
		CommonSharedMutex.lock_shared();
		for (std::pair<const unsigned long long, RigidBodyItem> & curItem : physicalItemsCollection->allItems)
		{
			if (!curItem.second.rigidBodyRef->isMap) continue;
			RigidBodyRigidBody(*tmpRigidBodyAABB, *curItem.second.rigidBodyRef, tmpIntersectionList, true);
			if (tmpIntersectionList.size() > 0) break;
		}
		CommonSharedMutex.unlock_shared();

		if (!tmpIntersectionList.size())
		{
			deferredAdds.emplace_back(std::string(""), newGroupId, tmpRigidBodyAABB, groupId, groupedTris);
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

		std::string newGroupId = groupId; newGroupId += "_piece"; newGroupId += std::to_string(mersenneTwister64BitPRNG());

		std::vector<TriUV> aabbTris;
		std::vector <intersection> tmpIntersectionList;
		GenerateAABB(groupedTris, aabbTris, ColTol);
		RigidBody *tmpRigidBody = new RigidBody(newGroupId, *currentMapPropBlock, aabbTris, newCenter);
		tmpRigidBody->lin_v = hitDir;
		tmpRigidBody->ang_v = vec3((rand() % 100 - 50) * 0.02f) * 0.2f;

		deferredAdds.emplace_back(std::string(""), newGroupId, tmpRigidBody, groupId, groupedTris);
	}

	// If there's nothing left for the map destroy the rendered geom and rigid body piece
	if (!stuckToMapTris.size())
	{
		deferredAdds.emplace_back(std::string("remove"), groupId, nullptr, groupId, std::vector<TriUV>());
	}
	else // Otherwise update the rendered geom and rigid body piece with the left-over triangles
	{
		deferredAdds.emplace_back(std::string("change"), groupId, nullptr, groupId, stuckToMapTris);
	}

	booleanOpThreadFinished = true;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ShatterThread(PhysicalItemClass *physicalItemsCollection, std::string groupId, vec3 hitDir, vec3 hitPoint, vec3 hitNorm, float hitRadius, const std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> perSubmissionCall, const std::vector<GroupedRenderSubmission*> subList)
{
	// Rigid bodies generated by debris will not introduce anything needing an exclusive lock (as of now...)
	std::list <BrokenPiece> piecesMap;
	piecesMap.emplace_back();
	BrokenPiece & addedPiece = piecesMap.back();
	std::vector<TriUV> & mapPieceGeom = addedPiece.geom;

	DataBlock * currentMapPropBlock;
	{std::lock_guard<std::mutex> lk(physicalItemsCollection->destructibleCacheMutex);
	mapPieceGeom = physicalItemsCollection->destructibles[groupId].sourceTriUVList;
	addedPiece.aabbMin = physicalItemsCollection->destructibles[groupId].sourceMin;
	addedPiece.aabbMax = physicalItemsCollection->destructibles[groupId].sourceMax;
	currentMapPropBlock = &physicalItemsCollection->destructibles[groupId].propBlock; }

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
					for (TriUV & curFlippedCapTri : flippedCap)
						curFlippedCapTri.normVec = -curFlippedCapTri.normVec;
					Slice(slashTri, it->geom, above, below);
					bool aboveChecked = false, belowChecked = false;
					for (TriUV & curAboveTri : above)
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
					for (TriUV & curBelowTri : below)
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
				piecesMap.emplace_back();
				BrokenPiece & newAbovePiece = piecesMap.back();
				newAbovePiece.aabbMin = aboveMin;
				newAbovePiece.aabbMax = aboveMax;
				newAbovePiece.geom = above;
				newAbovePiece.cent = aboveCent;
				newAbovePiece.rad = aboveRad;
				piecesMap.emplace_back();
				BrokenPiece & newBelowPiece = piecesMap.back();
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
				piecesMap.emplace_back();
				BrokenPiece & newFragmentPiece = piecesMap.back();
				bool newFragChecked = false;
				for (TriUV & curTri : groupedTris)
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

	for (BrokenPiece & curPiece : piecesMap)
	{
		vec3 newCenter = Recenter(curPiece.geom);

		std::string newGroupId = groupId; newGroupId += "_piece"; newGroupId += std::to_string(mersenneTwister64BitPRNG());

		std::vector<TriUV> aabbTris;
		std::vector <intersection> tmpIntersectionList;
		GenerateAABB(curPiece.geom, aabbTris, ColTol);
		RigidBody *tmpRigidBodyAABB = new RigidBody(newGroupId, *currentMapPropBlock, aabbTris, newCenter);
		tmpRigidBodyAABB->ang_v = vec3((rand() % 100 - 50) * 0.02f) * 0.4f;
		tmpRigidBodyAABB->lin_v = -hitNorm * 3.0f + hitDir * 0.25f + vec3((rand() % 100 - 50) * 0.02f) * 0.2f;

		deferredAdds.emplace_back(std::string(""), newGroupId, tmpRigidBodyAABB, groupId, curPiece.geom);
	}
	deferredAdds.emplace_back(std::string("remove"), groupId, nullptr, groupId, std::vector<TriUV>());

	booleanOpThreadFinished = true;
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Update(std::function <bool(vec3 &, vec3 &, float)> isInDrawRegion)
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
		for (AddParams & curAdd : deferredAdds)
		{
			if (curAdd.origPieceAction == std::string(""))
			{
				MeshMaterial destructMaterial;
				{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
				destructMaterial = destructibles[curAdd.origGroupId].sourceModel->getMaterialById(curAdd.origGroupId); }
				Add(curAdd.newGroupId, curAdd.oobbPiece, destructMaterial, perSubmissionCallForDebris, subListForDebris, curAdd.triList);
			}
			else if (curAdd.origPieceAction == std::string("remove"))
			{
				CommonSharedMutex.lock();
				{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
				destructibles[curAdd.origGroupId].sourceBody->removeGroupById(curAdd.newGroupId);
				destructibles[curAdd.origGroupId].sourceModel->removeGroupById(curAdd.newGroupId);
				destructibles[curAdd.origGroupId].sourceTriUVList = {}; }
				CommonSharedMutex.unlock();
			}
			else if (curAdd.origPieceAction == std::string("change"))
			{
				CommonSharedMutex.lock();
				{std::lock_guard<std::mutex> lk(destructibleCacheMutex);
				destructibles[curAdd.origGroupId].sourceBody->ChangeGeom(curAdd.newGroupId, curAdd.triList);
				destructibles[curAdd.origGroupId].sourceModel->ChangeGeom(curAdd.newGroupId, curAdd.triList);
				destructibles[curAdd.origGroupId].sourceTriUVList = curAdd.triList; }
				CommonSharedMutex.unlock();
			}
		}
		deferredAdds.clear();
		CommonSharedMutex.lock_shared();
	}
	// Build SDF leaves for recently added debris/rigid bodies
	if (recentlyAddedRigidModels.size() > 0)
	{
		std::vector<SubmittedRenderItem> sri;
		sri.reserve(recentlyAddedRigidModels.size());
		for (GraphicsModel *curModel : recentlyAddedRigidModels)
		{
			sri.emplace_back();
			sri.back().item = curModel;
		}
		GraphicsModel::UpdateSDFs(sri);
		recentlyAddedRigidModels.clear();
	}

	for (std::pair<const unsigned long long, RigidBodyItem> & curItem : allItems)
	{
		RigidBodyItem & curRigidItem = curItem.second;
		if (!curRigidItem.rigidBodyRef || curRigidItem.rigidBodyRef->isMap || curRigidItem.external) continue;
		RigidBody *rigidBodyRef = curRigidItem.rigidBodyRef;
		mat4 trans = rigidBodyRef->orient;
		trans.i[0][3] = rigidBodyRef->pos.x;
		trans.i[1][3] = rigidBodyRef->pos.y;
		trans.i[2][3] = rigidBodyRef->pos.z;
		mat4 transDT = trans.DirectionTransform();

		if (!curRigidItem.transOnce)
		{
			curRigidItem.transOnce = true;
			curRigidItem.prevTrans.Ident();
			curRigidItem.prevTransDT.Ident();
		}

		mat4 finalTrans = trans * curRigidItem.prevTrans.Inv();
		mat4 finalTransDT = transDT * curRigidItem.prevTransDT.Inv();

		UnpackMat4(finalTrans, &rigidTransformParams.matrices[0]);
		UnpackMat4(finalTransDT, &rigidTransformParams.matrices[16]);
		for (int i = 0; i != curRigidItem.transformParamBufs.size(); i++)
			curRigidItem.transformParamBufs[i]->UploadSubData(0, &rigidTransformParams.matrices[0], 32 * sizeof(float));

		curRigidItem.modelRef->TransformCorners(trans);
		if (trans != curRigidItem.prevTrans)
			curRigidItem.modelRef->SetDirty();

		curRigidItem.prevTrans = trans;
		curRigidItem.prevTransDT = transDT;

		if (!isInDrawRegion(MainFrustum.eye, rigidBodyRef->pos, rigidBodyRef->radius) && curItem.second.allSubmissionInfos.size() > 0)
		{
			curItem.second.cachedSubmissions = curItem.second.allSubmissionInfos;
			for (SubmittedRenderItem & curSri : curItem.second.allSubmissionInfos)
				curSri.producer->Remove(curSri);
			curItem.second.allSubmissionInfos.clear();
		}
		else if (isInDrawRegion(MainFrustum.eye, rigidBodyRef->pos, rigidBodyRef->radius) && curItem.second.allSubmissionInfos.size() == 0)
		{
			for (SubmittedRenderItem & curSri : curItem.second.cachedSubmissions)
				curItem.second.allSubmissionInfos.push_back (curSri.producer->Add(*curSri.item, curSri.filterFunction));
			curItem.second.cachedSubmissions.clear();
		}
	}

	if (transformRigidSubmission)
	{
		transformRigidSubmission->makeSerial();
		transformRigidSubmission->Submit();
	}
}

void HIGHOMEGA::WORLD::PhysicalItemClass::ClearContent()
{
	for (std::pair<const unsigned long long, RigidBodyItem> & curItem : allItems)
	{
		RigidBodyItem & curRigidItem = curItem.second;
		if (curRigidItem.external) continue;
		for (SubmittedRenderItem & curSubmissionInfo : curRigidItem.allSubmissionInfos)
			curSubmissionInfo.producer->Remove(curSubmissionInfo);
		if (curRigidItem.rigidBodyRef) delete curRigidItem.rigidBodyRef;

		for (unsigned int i = 0; i != curRigidItem.sourceGeom.size(); i++)
		{
			delete curRigidItem.transformParamBufs[i];
			delete curRigidItem.transformShader[i];
		}
		if (curRigidItem.modelRef) delete curRigidItem.modelRef;
		if (curRigidItem.constaintsRef)
		{
			std::vector<ConstraintCollection *>::iterator it = std::find(constraintCollection.begin(), constraintCollection.end(), curRigidItem.constaintsRef);
			if (it != constraintCollection.end())
				constraintCollection.erase(it);
			delete curRigidItem.constaintsRef;
		}
	}
	if (transformRigidSubmission)
	{
		delete transformRigidSubmission;
		transformRigidSubmission = nullptr;
	}

	allItems.clear();
}

void HIGHOMEGA::WORLD::PhysicalItemClass::Remove(unsigned long long curId)
{
	CommonSharedMutex.lock();
	if (allItems.find(curId) != allItems.end())
	{
		RigidBodyItem & curRigidItem = allItems[curId];
		if (curRigidItem.external)
		{
			CommonSharedMutex.unlock();
			return;
		}
		for (SubmittedRenderItem & curSubmissionInfo : curRigidItem.allSubmissionInfos)
			curSubmissionInfo.producer->Remove(curSubmissionInfo);
		if (curRigidItem.rigidBodyRef)
		{
			std::vector<RigidBody *>::iterator it = std::find(allBodies.begin(), allBodies.end(), curRigidItem.rigidBodyRef);
			if (it != allBodies.end())
				allBodies.erase(it);
		}
		if (curRigidItem.constaintsRef)
		{
			std::vector<ConstraintCollection *>::iterator it = std::find(constraintCollection.begin(), constraintCollection.end(), curRigidItem.constaintsRef);
			if (it != constraintCollection.end())
				constraintCollection.erase(it);
		}
		CommonSharedMutex.unlock();
		if (curRigidItem.rigidBodyRef) delete curRigidItem.rigidBodyRef;
		if (curRigidItem.constaintsRef) delete curRigidItem.constaintsRef;
		CommonSharedMutex.lock();

		for (unsigned int i = 0; i != curRigidItem.sourceGeom.size(); i++)
		{
			if (transformRigidSubmission) transformRigidSubmission->RemoveDispatch(Instance, "rigidtransform_" + std::to_string(curId) + "_" + std::to_string(i));
			delete curRigidItem.transformParamBufs[i];
			delete curRigidItem.transformShader[i];
		}
		if (curRigidItem.modelRef) delete curRigidItem.modelRef;

		allItems.erase(curId);
	}
	CommonSharedMutex.unlock();
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Add(ParticleEmitter & emitterRef)
{
	ParticleItem curItem;

	vec3 emitterPos = vec3(emitterRef.integrateParams.posLinSpeedBase[0], emitterRef.integrateParams.posLinSpeedBase[1], emitterRef.integrateParams.posLinSpeedBase[2]);
	vec3 emitterDir = vec3(emitterRef.integrateParams.dirLinSpeedVar[0], emitterRef.integrateParams.dirLinSpeedVar[1], emitterRef.integrateParams.dirLinSpeedVar[2]);
	vec3 initParticlePos = emitterPos + cross(emitterDir, randNormVec()).normalized() * emitterRef.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[0];

	vec3 rotAxis = randNormVec();
	curItem.rotAxisScale[0] = rotAxis.x;
	curItem.rotAxisScale[1] = rotAxis.y;
	curItem.rotAxisScale[2] = rotAxis.z;
	curItem.rotAxisScale[3] = emitterRef.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1];
	vec3 xAxis = cross(rotAxis + vec3(0.1f), rotAxis).normalized();
	vec3 yAxis = cross(rotAxis, xAxis);
	curItem.xAxisAlpha[0] = xAxis.x;
	curItem.xAxisAlpha[1] = xAxis.y;
	curItem.xAxisAlpha[2] = xAxis.z;
	curItem.xAxisAlpha[3] = emitterRef.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2];
	curItem.yAxisHealth[0] = yAxis.x;
	curItem.yAxisHealth[1] = yAxis.y;
	curItem.yAxisHealth[2] = yAxis.z;
	curItem.yAxisHealth[3] = 0.0001f;
	curItem.posFadeRate[0] = initParticlePos.x;
	curItem.posFadeRate[1] = initParticlePos.y;
	curItem.posFadeRate[2] = initParticlePos.z;
	curItem.posFadeRate[3] = emitterRef.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] + emitterRef.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[0] * randFract();
	curItem.shrinkRateDeathRateLinSpeed[0] = emitterRef.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[1] + emitterRef.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[2] * randFract();
	curItem.shrinkRateDeathRateLinSpeed[1] = emitterRef.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[3] + emitterRef.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0] * randFract();
	curItem.shrinkRateDeathRateLinSpeed[2] = emitterRef.integrateParams.posLinSpeedBase[3] + emitterRef.integrateParams.dirLinSpeedVar[3] * randFract();
	mat4 curTransMat = prepareTransMat(curItem);
	UnpackMat4(curTransMat, curItem.trans);

	emitterRef.particles.push_back(curItem);
}

mat4 HIGHOMEGA::WORLD::ParticleSystemClass::prepareTransMat(ParticleItem & inpItem)
{
	vec3 xAxis = vec3(inpItem.xAxisAlpha[0], inpItem.xAxisAlpha[1], inpItem.xAxisAlpha[2]);
	vec3 yAxis = vec3(inpItem.yAxisHealth[0], inpItem.yAxisHealth[1], inpItem.yAxisHealth[2]);
	vec3 rotAxis = vec3(inpItem.rotAxisScale[0], inpItem.rotAxisScale[1], inpItem.rotAxisScale[2]);
	float scale = inpItem.rotAxisScale[3];
	vec3 pos = vec3(inpItem.posFadeRate[0], inpItem.posFadeRate[1], inpItem.posFadeRate[2]);

	mat4 transMat;
	transMat.Ident();
	transMat.i[0][0] = xAxis.x * scale; transMat.i[0][1] = yAxis.x * scale;	transMat.i[0][2] = rotAxis.x * scale; transMat.i[0][3] = pos.x;
	transMat.i[1][0] = xAxis.y * scale; transMat.i[1][1] = yAxis.y * scale; transMat.i[1][2] = rotAxis.y * scale; transMat.i[1][3] = pos.y;
	transMat.i[2][0] = xAxis.z * scale; transMat.i[2][1] = yAxis.z * scale;	transMat.i[2][2] = rotAxis.z * scale; transMat.i[2][3] = pos.z;
	return transMat;
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

unsigned long long HIGHOMEGA::WORLD::ParticleSystemClass::Populate(Mesh & inpMesh, std::vector <GroupedRenderSubmission*> && subList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall, minMaxReductionClass & inpMinMaxReducter)
{
	submissionsForParticle = subList;
	perSubmissionCall = inpPerSubmissionCall;

	unsigned long long emittersId = mersenneTwister64BitPRNG();

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
				 !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", emitDir) ) throw std::runtime_error("Could not get emitter pos/dir/dist");

			curEmitter.integrateParams.posLinSpeedBase[0] = emitPos.x;
			curEmitter.integrateParams.posLinSpeedBase[1] = emitPos.y;
			curEmitter.integrateParams.posLinSpeedBase[2] = emitPos.z;
			curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[0] = emitRad;
			curEmitter.integrateParams.dirLinSpeedVar[0] = emitDir.x;
			curEmitter.integrateParams.dirLinSpeedVar[1] = emitDir.y;
			curEmitter.integrateParams.dirLinSpeedVar[2] = emitDir.z;

			std::string particleMesh, particleMeshAssetLoc;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "particleMesh" + jAsString, particleMesh) ||
				 !Mesh::getDataRowString(curPolyGroup, "PROPS", "particleMeshAssetLoc" + jAsString, particleMeshAssetLoc) ) throw std::runtime_error("Could not get particle mesh and mesh loc");

			if (cachedMeshesModels.find(particleMesh) == cachedMeshesModels.end())
			{
				cachedMeshesModels[particleMesh].mesh = Mesh(particleMesh);
				cachedMeshesModels[particleMesh].model = new GraphicsModel(cachedMeshesModels[particleMesh].mesh, particleMeshAssetLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
					float tmpFloat;
					return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
				}, nullptr, true, true);
			}

			curEmitter.collectionModel = new GraphicsModel(cachedMeshesModels[particleMesh].mesh, particleMeshAssetLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
				float tmpFloat;
				return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
			}, nullptr, true, false);
			curEmitter.collectionModel->BlankAndResize(particleCount);

			curEmitter.meshModelRef = &cachedMeshesModels[particleMesh];

			// Property fetching and defaults 
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleLinVelBase" + jAsString, curEmitter.integrateParams.posLinSpeedBase[3])) curEmitter.integrateParams.posLinSpeedBase[3] = 0.2f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleLinVelVar" + jAsString, curEmitter.integrateParams.dirLinSpeedVar[3])) curEmitter.integrateParams.dirLinSpeedVar[3] = 0.02f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleAngVelBase" + jAsString, curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1])) curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[1] = 0.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleAngVelVar" + jAsString, curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[2])) curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[2] = 0.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleFadeRateBase" + jAsString, curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3])) curEmitter.integrateParams.radAngSpeedBaseAngSpeedVarFadeRateBase[3] = 0.95f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleFadeRateVar" + jAsString, curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[0])) curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[0] = 0.01f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleShrinkRateBase" + jAsString, curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[1])) curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[1] = 0.99f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleShrinkRateVar" + jAsString, curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[2])) curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[2] = 0.001f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleDeathRateBase" + jAsString, curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[3])) curEmitter.integrateParams.fadeRateVarShrinkRateBaseShrinkRateVarDeathRateBase[3] = 0.95f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleDeathRateVar" + jAsString, curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0])) curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[0] = 0.01f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleInitScale" + jAsString, curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1])) curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[1] = 3.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "particleInitAlpha" + jAsString, curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2])) curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[2] = 1.0f;
			curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[3] = 0.0f;

			for (int k = 0; k != particleCount; k++)
				Add(curEmitter);

			if (!integrateSubmission) integrateSubmission = new ComputeSubmission;
			if (!transformCollectionSubmission) transformCollectionSubmission = new ComputeSubmission;

			curEmitter.integrateParams.numParticles = particleCount;

			curEmitter.particlesBuf = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, curEmitter.particles.data(), (unsigned int)(curEmitter.particles.size() * sizeof(ParticleItem)));
			curEmitter.integrateParamsBuf = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &curEmitter.integrateParams, (unsigned int)sizeof(curEmitter.integrateParams));
			curEmitter.integrateShader = new ShaderResourceSet;
			curEmitter.integrateShader->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *curEmitter.particlesBuf);
			curEmitter.integrateShader->AddResource(RESOURCE_UBO, COMPUTE, 0, 1, *curEmitter.integrateParamsBuf);
			curEmitter.integrateShader->Create("shaders/integrateParticles.comp.spv", "main");
			std::string curIntegrationDispatchName = std::string("particle_") + std::to_string(emittersId) + std::string("_") + std::to_string(i) + std::string("_") + std::to_string(j);
			integrateSubmission->MakeDispatch(Instance, curIntegrationDispatchName, *curEmitter.integrateShader, (unsigned int)ceil((double)particleCount / (double)WorkGroupIntegrateX()), 1, 1);
			curEmitter.meshGroup = i;
			curEmitter.plasterGroup = j;

			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & collectionMaterialGeomMap = curEmitter.collectionModel->MaterialGeomMap;
			std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = curEmitter.meshModelRef->model->MaterialGeomMap;
			for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = collectionMaterialGeomMap.begin(); it != collectionMaterialGeomMap.end(); ++it)
			{
				for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
					curEmitter.sourceGeom.push_back(&(*it2));
				for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
					curEmitter.destGeom.push_back(&(*it2));
			}
			for (unsigned int k = 0; k != curEmitter.sourceGeom.size(); k++)
			{
				unsigned int numTris = curEmitter.sourceGeom[k]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
				transformParams.triCountInstanceCount[0] = numTris;
				transformParams.triCountInstanceCount[1] = particleCount;
				curEmitter.transformParamsBuf.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &transformParams, (unsigned int)sizeof(transformParamsStruct)));
				curEmitter.transformCollectionShader.push_back(new ShaderResourceSet);
				ShaderResourceSet *curShaderResourceSet = curEmitter.transformCollectionShader.back();
				curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, curEmitter.sourceGeom[k]->getVertBuffer());
				curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, curEmitter.destGeom[k]->getVertBuffer());
				curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, *(curEmitter.particlesBuf));
				curShaderResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 3, *(curEmitter.transformParamsBuf.back()));
				curShaderResourceSet->Create("shaders/transformParticles.comp.spv", "main");
				std::string curTransformDispatchName = std::string("particleIntegrate_") + std::to_string(emittersId) + std::string("_") + std::to_string(i) + std::string("_") + std::to_string(j) + std::string("_") + std::to_string(k);
				transformCollectionSubmission->MakeDispatch(Instance, curTransformDispatchName, *curShaderResourceSet, (unsigned int)ceil((double)numTris / (double)WorkGroupTransformX()), (unsigned int)ceil((double)particleCount / (double)WorkGroupTransformY()), 1);

				inpMinMaxReducter.RequestMinMax(curEmitter.destGeom[k]);
			}

			allEmitters[emittersId].push_back(curEmitter);
		}
	}
	return emittersId;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Remove(unsigned long long curId)
{
	if (allEmitters.find(curId) == allEmitters.end()) return;

	for (ParticleEmitter & curEmitter : allEmitters[curId])
	{
		for (SubmittedRenderItem & curSubmission : curEmitter.allSubmissionInfos)
			curSubmission.producer->Remove(curSubmission);
		for (unsigned int i = 0; i != curEmitter.sourceGeom.size(); i++)
		{
			delete curEmitter.transformParamsBuf[i];
			delete curEmitter.transformCollectionShader[i];
		}

		delete curEmitter.particlesBuf;
		delete curEmitter.integrateParamsBuf;
		delete curEmitter.integrateShader;

		delete curEmitter.collectionModel;

		if (integrateSubmission)
		{
			std::string integrateKey = std::string("particle_") + std::to_string(curId) + std::string("_") + std::to_string(curEmitter.meshGroup) + std::string("_") + std::to_string(curEmitter.plasterGroup);
			integrateSubmission->RemoveDispatch(Instance, integrateKey);
		}

		if (transformCollectionSubmission)
			for (int i = 0; i != curEmitter.sourceGeom.size(); i++)
			{
				std::string transformKey = std::string("particleIntegrate_") + std::to_string(curId) + std::string("_") + std::to_string(curEmitter.meshGroup) + std::string("_") + std::to_string(curEmitter.plasterGroup) + std::string("_") + std::to_string(i);
				transformCollectionSubmission->RemoveDispatch(Instance, transformKey);
			}
	}

	allEmitters.erase(curId);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Combine(std::vector<ParticleSystemClass>& particleClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall)
{
	if (!integrateSubmission) integrateSubmission = new ComputeSubmission;
	if (!transformCollectionSubmission) transformCollectionSubmission = new ComputeSubmission;

	submissionsForParticle = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	for (ParticleSystemClass & curParticleClass : particleClasses)
	{
		for (std::pair<const std::string, MeshModel> & curMeshModel : curParticleClass.cachedMeshesModels)
			if (cachedMeshesModels.find(curMeshModel.first) == cachedMeshesModels.end())
				cachedMeshesModels[curMeshModel.first] = curMeshModel.second;
		curParticleClass.cachedMeshesModels.clear();

		if (curParticleClass.integrateSubmission) delete curParticleClass.integrateSubmission;
		if (curParticleClass.transformCollectionSubmission) delete curParticleClass.transformCollectionSubmission;
		curParticleClass.integrateSubmission = nullptr;
		curParticleClass.transformCollectionSubmission = nullptr;

		for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : curParticleClass.allEmitters)
		{
			allEmitters[curEmittersIdPair.first] = curEmittersIdPair.second;

			for (ParticleEmitter & curParticleEmitter : curEmittersIdPair.second)
			{
				std::string integrateDispatchKey = std::string("particle_") + std::to_string(curEmittersIdPair.first) + std::string("_") + std::to_string(curParticleEmitter.meshGroup) + std::string("_") + std::to_string(curParticleEmitter.plasterGroup);
				unsigned int particleCount = (unsigned int)curParticleEmitter.particles.size();
				integrateSubmission->MakeDispatch(Instance, integrateDispatchKey, *curParticleEmitter.integrateShader, (unsigned int)ceil((double)particleCount / (double)WorkGroupIntegrateX()), 1, 1);
				for (int i = 0; i != curParticleEmitter.sourceGeom.size(); i++)
				{
					unsigned int numTris = curParticleEmitter.sourceGeom[i]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					std::string transformDispatchKey = std::string("particleIntegrate_") + std::to_string(curEmittersIdPair.first) + std::string("_") + std::to_string(curParticleEmitter.meshGroup) + std::string("_") + std::to_string(curParticleEmitter.plasterGroup) + std::string("_") + std::to_string(i);
					transformCollectionSubmission->MakeDispatch(Instance, transformDispatchKey, *curParticleEmitter.transformCollectionShader[i], (unsigned int)ceil((double)numTris / (double)WorkGroupTransformX()), (unsigned int)ceil((double)particleCount / (double)WorkGroupTransformY()), 1);
				}
			}
		}
		curParticleClass.allEmitters.clear();
		curParticleClass.curTimeElapsed = 0.0f;
		curParticleClass.integratedOnce = false;
	}
	integratedOnce = true;
}

void HIGHOMEGA::WORLD::ParticleSystemClass::Update(float elapseTime)
{
	curTimeElapsed += elapseTime;

	if (!integratedOnce)
	{
		integratedOnce = true;
		for (int ii = 0; ii != 1000; ii++)
		{
			for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : allEmitters)
				for (ParticleEmitter & curEmitter : curEmittersIdPair.second)
				{
					curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[3] = curTimeElapsed;
					curEmitter.integrateParamsBuf->UploadSubData(0, &curEmitter.integrateParams, sizeof(curEmitter.integrateParams));
				}
			if (integrateSubmission)
			{
				integrateSubmission->makeSerial();
				integrateSubmission->Submit();
			}
		}
	}

	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : allEmitters)
		for (ParticleEmitter & curEmitter : curEmittersIdPair.second)
		{
			curEmitter.integrateParams.deathRateVarInitialScaleInitialAlphaCurTime[3] = curTimeElapsed;
			curEmitter.integrateParamsBuf->UploadSubData(0, &curEmitter.integrateParams, sizeof(curEmitter.integrateParams));
		}

	if (integrateSubmission)
	{
		integrateSubmission->makeSerial();
		integrateSubmission->Submit();
		transformCollectionSubmission->makeSerial();
		transformCollectionSubmission->Submit();
	}

	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : allEmitters)
		for (ParticleEmitter & curEmitter : curEmittersIdPair.second)
			if (curEmitter.allSubmissionInfos.size() == 0)
				for (GroupedRenderSubmission *curSubmission : submissionsForParticle)
					curEmitter.allSubmissionInfos.emplace_back(perSubmissionCall(curSubmission, curEmitter.collectionModel));
}

void HIGHOMEGA::WORLD::ParticleSystemClass::UpdateSDFs()
{
	std::vector<SubmittedRenderItem> sris;
	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersPair : allEmitters)
		for (ParticleEmitter & curEmitter : curEmittersPair.second)
		{
			SubmittedRenderItem sri;
			sri.item = curEmitter.collectionModel;
			sris.push_back(sri);
		}
	GraphicsModel::UpdateSDFs(sris);
}

void HIGHOMEGA::WORLD::ParticleSystemClass::ClearContent()
{
	for (std::pair<const unsigned long long, std::vector<ParticleEmitter>> & curEmittersIdPair : allEmitters)
		for (ParticleEmitter & curEmitter : curEmittersIdPair.second)
		{
			for (SubmittedRenderItem & curSubmission : curEmitter.allSubmissionInfos)
				curSubmission.producer->Remove(curSubmission);
			for (unsigned int i = 0; i != curEmitter.sourceGeom.size(); i++)
			{
				delete curEmitter.transformParamsBuf[i];
				delete curEmitter.transformCollectionShader[i];
			}

			delete curEmitter.particlesBuf;
			delete curEmitter.integrateParamsBuf;
			delete curEmitter.integrateShader;

			delete curEmitter.collectionModel;
		}
	allEmitters.clear();

	for (std::pair<const std::string, MeshModel> & curCachedMeshModel : cachedMeshesModels)
		delete curCachedMeshModel.second.model;
	cachedMeshesModels.clear();

	if (integrateSubmission)
	{
		delete integrateSubmission;
		delete transformCollectionSubmission;
		integrateSubmission = nullptr;
		transformCollectionSubmission = nullptr;
	}

	integratedOnce = false;
	curTimeElapsed = 0.0f;
}

unsigned long long HIGHOMEGA::WORLD::CameraSystemClass::Populate(Mesh & inpMesh)
{
	unsigned long long curCamRailId = mersenneTwister64BitPRNG();
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];
		Camera curCamera;

		if (curPolyGroup.type != "CAMERA") continue;

		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "duration", curCamera.duration)) curCamera.duration = 1.0f;
		float orderFloat;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "order", orderFloat)) throw std::runtime_error("Could not get camera order");
		curCamera.order = (unsigned int)orderFloat;

		if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", curCamera.pos) ||
			 !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", curCamera.dir) ) throw std::runtime_error("Could not get camera pos/dir");

		float tmpFloat;
		curCamera.action = NONE;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "fadein", tmpFloat)) curCamera.action |= FADE_IN;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "fadeout", tmpFloat)) curCamera.action |= FADE_OUT;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "showAurora", tmpFloat)) curCamera.action |= SHOW_AURORA;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "showClouds", tmpFloat)) curCamera.action |= SHOW_CLOUDS;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "forceSunAngle", curCamera.actionParam)) curCamera.action |= FORCE_SUN_ANGLE;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "unforceSunAngle", tmpFloat)) curCamera.action |= UNFORCE_SUN_ANGLE;
		if (Mesh::getDataRowString(curPolyGroup, "PROPS", "newMap", curCamera.newMap))
		{
			curCamera.action |= NEW_MAP;
			if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "newMapBelong", curCamera.newMapBelong)) throw std::runtime_error("Could not get new map directory for camera newMap action");
		}
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "creditRoll", tmpFloat)) curCamera.action |= CREDIT_ROLL;

		allItems[curCamRailId].push_back(curCamera);
	}

	if (allItems.find(curCamRailId) == allItems.end()) return curCamRailId; // No rail was added...

	std::sort(allItems[curCamRailId].begin(), allItems[curCamRailId].end(), [](const Camera& lhs, const Camera& rhs)
	{
		return lhs.order < rhs.order;
	});

	// Quick safety check...
	for (int i = 0; i != allItems[curCamRailId].size(); i++)
		if (allItems[curCamRailId][i].order != i + 1) throw std::runtime_error("There's an issue with numbering in the camera system");

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
		if (cameraRail[curNum].action & NEW_MAP)
		{
			mapChangeDataStruct.changeMap = true;
			mapChangeDataStruct.newMap = cameraRail[curNum].newMap;
			mapChangeDataStruct.newMapBelong = cameraRail[curNum].newMapBelong;
			return;
		}
		if (cameraRail[curNum].action & CREDIT_ROLL)
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

	if (curCamera.action & FADE_IN) curAlpha = lerpFraction;
	if (curCamera.action & FADE_OUT) curAlpha = 1.0f - lerpFraction;
	if (curCamera.action & SHOW_AURORA) WorldParams.ShowAurora();
	if (curCamera.action & SHOW_CLOUDS) WorldParams.ShowClouds();
	if (curCamera.action & FORCE_SUN_ANGLE) WorldParams.ForceSunAngle(curCamera.actionParam);
	if (curCamera.action & UNFORCE_SUN_ANGLE) WorldParams.UnforceSunAngle();

	curTimeElapsed += WorldParams.GetFrameTime();
	if (curTimeElapsed > curCamera.duration)
	{
		if (curCamera.action & FADE_IN) curAlpha = 1.0f;
		if (curCamera.action & FADE_OUT) curAlpha = 0.0f;

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

unsigned long long HIGHOMEGA::WORLD::GuidedModelSystemClass::Populate(Mesh & inpMesh, std::vector<GroupedRenderSubmission*> &&subList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall)
{
	unsigned long long guidedModelsId = mersenneTwister64BitPRNG();
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

			if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", curPoint.pos)) throw std::runtime_error("Guided model pos not provided");
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "scale", curPoint.scale)) curPoint.scale = 1.0f;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "duration", curPoint.duration)) curPoint.duration = 1.0f;

			float orderFloat;
			if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "order", orderFloat)) throw std::runtime_error("Guided model order not provided");
			curPoint.order = (unsigned int)orderFloat;

			if (!curPath.transformedModel)
			{
				std::string guidedModelMesh, guidedModelMeshAssetLoc;
				if (!Mesh::getDataRowString(curPolyGroup, "PROPS", "guidedModelMesh", guidedModelMesh) ||
					!Mesh::getDataRowString(curPolyGroup, "PROPS", "guidedModelMeshAssetLoc", guidedModelMeshAssetLoc)) throw std::runtime_error("Guided model or asset loc not found");

				if (cachedMeshes.find(guidedModelMesh) == cachedMeshes.end())
					cachedMeshes[guidedModelMesh] = Mesh(guidedModelMesh);
				curPath.meshRef = &cachedMeshes[guidedModelMesh];
				curPath.transformedModel = new GraphicsModel(cachedMeshes[guidedModelMesh], guidedModelMeshAssetLoc, Instance, [](int, DataGroup & inpGroup) -> bool {
					float tmpFloat;
					return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
				}, nullptr, true, false);
				if (!transformRigidSubmission) transformRigidSubmission = new ComputeSubmission;
				std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & sourceMaterialGeomMap = curPath.transformedModel->MaterialGeomMap;
				std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & destMaterialGeomMap = curPath.transformedModel->MaterialGeomMap;
				for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = destMaterialGeomMap.begin(); it != destMaterialGeomMap.end(); ++it)
				{
					for (std::list<GeometryClass>::iterator it2 = sourceMaterialGeomMap[it->first].begin(); it2 != sourceMaterialGeomMap[it->first].end(); it2++)
						curPath.sourceGeom.push_back(&(*it2));
					for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
						curPath.destGeom.push_back(&(*it2));
				}
				curPath.meshGroup = i;
				UnpackMat4(curPath.curTrans, &rigidTransformParams.matrices[0]);
				UnpackMat4(curPath.curTrans.DirectionTransform(), &rigidTransformParams.matrices[16]);
				for (unsigned int j = 0; j != curPath.sourceGeom.size(); j++)
				{
					unsigned int numTris = curPath.sourceGeom[j]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					rigidTransformParams.triCount = numTris;
					curPath.transformParamBufs.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &rigidTransformParams, (unsigned int)sizeof(rigidTransformParamsStruct)));
					curPath.transformShader.push_back(new ShaderResourceSet);
					ShaderResourceSet *curShaderResourceSet = curPath.transformShader.back();
					curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, curPath.sourceGeom[j]->getVertBuffer());
					curShaderResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, curPath.destGeom[j]->getVertBuffer());
					curShaderResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 2, *(curPath.transformParamBufs.back()));
					curShaderResourceSet->Create("shaders/transformRigid.comp.spv", "main");
					std::string transformDispatchName = "guidedmodels_" + std::to_string(guidedModelsId) + std::string("_") + std::to_string(i) + std::string("_") + std::to_string(j);
					transformRigidSubmission->MakeDispatch(Instance, transformDispatchName, *curShaderResourceSet, (unsigned int)ceil((double)numTris / (double)WorkGroupTransformRigidX()), 1, 1);
				}
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

unsigned int HIGHOMEGA::WORLD::GuidedModelSystemClass::WorkGroupTransformRigidX()
{
	return 32;
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::Update(float elapseTime)
{
	for (std::pair<const unsigned long long, std::vector<PathState>> & curItem : allItems)
		for (PathState & curPS : curItem.second)
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
				curPS.curDir = Lerp (toCurrent.normalized(), toNext.normalized(), bezierFraction).normalized();
				curPS.curScale = Lerp(curPS.fullPath[prevNum].scale, curPS.fullPath[curNum].scale, bezierFraction);
				curPS.curPos = QuadraticBezier (curPS.fullPath[prevNum].pos + toCurrent * 0.75f, curPS.fullPath[curNum].pos, curPS.fullPath[curNum].pos + toNext * 0.25f, bezierFraction);
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
				curPS.curPos = Lerp (curPS.fullPath[curNum].pos, curPS.fullPath[nextNum].pos, lerpFraction);
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

			curPS.curTransDT = curPS.curTrans.DirectionTransform();

			if (!curPS.transOnce)
			{
				curPS.transOnce = true;
				curPS.prevTrans.Ident();
				curPS.prevTransDT.Ident();
			}

			mat4 finalTrans = curPS.curTrans * curPS.prevTrans.Inv();
			mat4 finalTransDT = curPS.curTransDT * curPS.prevTransDT.Inv();

			UnpackMat4(finalTrans, &rigidTransformParams.matrices[0]);
			UnpackMat4(finalTransDT, &rigidTransformParams.matrices[16]);
			for (int i = 0; i != curPS.transformParamBufs.size(); i++)
				curPS.transformParamBufs[i]->UploadSubData(0, &rigidTransformParams.matrices[0], 32 * sizeof(float));

			curPS.transformedModel->TransformCorners(curPS.curTrans);
			curPS.transformedModel->SetDirty();

			curPS.prevTrans = curPS.curTrans;
			curPS.prevTransDT = curPS.curTransDT;

			curPS.curTimeElapsed += elapseTime;
			if (curPS.curTimeElapsed > curPS.fullPath[curNum].duration)
			{
				curPS.curTimeElapsed = 0.0f;
				curPS.curNum++;
				if (curPS.curNum == (unsigned int)curPS.fullPath.size()) curPS.curNum = 0;
			}
		}

	if (transformRigidSubmission)
	{
		transformRigidSubmission->makeSerial();
		transformRigidSubmission->Submit();
	}

	for (std::pair<const unsigned long long, std::vector<PathState>> & curGuidedModels : allItems)
		for (PathState & curPathState : curGuidedModels.second)
			if (curPathState.allSubmissionInfos.size() == 0)
				for (GroupedRenderSubmission *curSubmission : submissionsForGuidedModels)
					curPathState.allSubmissionInfos.emplace_back(perSubmissionCall (curSubmission, curPathState.transformedModel));
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::Combine(std::vector <GuidedModelSystemClass> & guidedModelsClasses, std::vector<GroupedRenderSubmission*>& submissionList, std::function<SubmittedRenderItem(GroupedRenderSubmission*, GraphicsModel*)> inpPerSubmissionCall)
{
	if (!transformRigidSubmission) transformRigidSubmission = new ComputeSubmission;

	submissionsForGuidedModels = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	for (GuidedModelSystemClass & curGuidedModelsClass : guidedModelsClasses)
	{
		for (std::pair<const std::string, Mesh> & curMesh : curGuidedModelsClass.cachedMeshes)
			if (cachedMeshes.find(curMesh.first) == cachedMeshes.end())
				cachedMeshes[curMesh.first] = curMesh.second;
		curGuidedModelsClass.cachedMeshes.clear();

		if (curGuidedModelsClass.transformRigidSubmission) delete curGuidedModelsClass.transformRigidSubmission;
		curGuidedModelsClass.transformRigidSubmission = nullptr;

		for (std::pair<const unsigned long long, std::vector<PathState>> & curPathStatesIdPair : curGuidedModelsClass.allItems)
		{
			allItems[curPathStatesIdPair.first] = curPathStatesIdPair.second;

			for (PathState & curPathState : curPathStatesIdPair.second)
				for (int i = 0; i != curPathState.sourceGeom.size(); i++)
				{
					unsigned int numTris = curPathState.sourceGeom[i]->getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					std::string transformDispatchKey = std::string("guidedmodels_") + std::to_string(curPathStatesIdPair.first) + std::string("_") + std::to_string(curPathState.meshGroup) + std::string("_") + std::to_string(i);
					transformRigidSubmission->MakeDispatch(Instance, transformDispatchKey, *curPathState.transformShader[i], (unsigned int)ceil((double)numTris / (double)WorkGroupTransformRigidX()), 1, 1);
				}
		}
		curGuidedModelsClass.allItems.clear();
	}
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::Remove(unsigned long long curId)
{
	if (allItems.find(curId) == allItems.end()) return;

	for (std::pair<const unsigned long long, std::vector<PathState>> & curPathStatesIdPair : allItems)
		for (PathState & curPathState : curPathStatesIdPair.second)
		{
			for (SubmittedRenderItem & curSubmission : curPathState.allSubmissionInfos)
				curSubmission.producer->Remove(curSubmission);
			for (unsigned int i = 0; i != curPathState.sourceGeom.size(); i++)
			{
				delete curPathState.transformParamBufs[i];
				delete curPathState.transformShader[i];
			}

			delete curPathState.transformedModel;

			if (transformRigidSubmission)
				for (int i = 0; i != curPathState.sourceGeom.size(); i++)
				{
					std::string transformKey = std::string("guidedmodels_") + std::to_string(curId) + std::string("_") + std::to_string(curPathState.meshGroup) + std::string("_") + std::to_string(i);
					transformRigidSubmission->RemoveDispatch(Instance, transformKey);
				}
		}

	allItems.erase(curId);
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::UpdateSDFs()
{
	std::vector<SubmittedRenderItem> sris;
	for (std::pair<const unsigned long long, std::vector<PathState>> & curPathStatesIdPair : allItems)
		for (PathState & curPathState : curPathStatesIdPair.second)
		{
			SubmittedRenderItem sri;
			sri.item = curPathState.transformedModel;
			sris.push_back(sri);
		}
	GraphicsModel::UpdateSDFs(sris);
}

void HIGHOMEGA::WORLD::GuidedModelSystemClass::ClearContent()
{
	for (std::pair<const unsigned long long, std::vector<PathState>> & curItem : allItems)
		for (PathState & curItem : curItem.second)
		{
			for (SubmittedRenderItem curSubmissionInfo : curItem.allSubmissionInfos)
				curSubmissionInfo.producer->Remove(curSubmissionInfo);
			for (unsigned int i = 0; i != curItem.sourceGeom.size(); i++)
			{
				delete curItem.transformParamBufs[i];
				delete curItem.transformShader[i];
			}
			delete curItem.transformedModel;
		}
	allItems.clear();
	cachedMeshes.clear();
	if (transformRigidSubmission)
	{
		delete transformRigidSubmission;
		transformRigidSubmission = nullptr;
	}
}

void HIGHOMEGA::WORLD::CreateWorld(WorldParamsClass & WorldParams)
{
	PhysicsThread = new std::thread(PhysicsLoop);
	EntitiesThread = new std::thread(EntitiesLoop, &WorldParams);
	ClothThread = new std::thread(ClothLoop);
	AudioThread = new std::thread(AudioLoop);
}

void HIGHOMEGA::WORLD::DestroyWorld(bool mapChange)
{
	{std::unique_lock <std::mutex> lk(fizXSignal.quit_mutex);
	std::unique_lock <std::mutex> lk2(entitiesSignal.quit_mutex);
	std::unique_lock <std::mutex> lk3(clothSignal.quit_mutex);
	fizXSignal.quit = true;
	entitiesSignal.quit = true;
	clothSignal.quit = true;}

	PhysicsThread->join();
	EntitiesThread->join();
	ClothThread->join();
	delete PhysicsThread;
	delete EntitiesThread;
	delete ClothThread;
	PhysicsThread = nullptr;
	EntitiesThread = nullptr;
	ClothThread = nullptr;

	if (!mapChange)
	{
		{std::unique_lock <std::mutex> lk4(audioSignal.quit_mutex); audioSignal.quit = true;}
		AudioThread->join();
		delete AudioThread;
		AudioThread = nullptr;
	}

	clothCollection.ClearContent();
	plasteredItemsCollection.ClearContent();
	physicalItemsCollection.ClearContent();
	particleSystem.ClearContent();
	guidedModelSystem.ClearContent();
	zoneStreaming.ClearContent();
	cameraSystem.ClearContent();
	mainMinMaxReducer.ClearContent();
	if (PhysicalItemClass::booleanOpThread && !PhysicalItemClass::booleanOpThreadFinished)
	{
		PhysicalItemClass::booleanOpThread->join();
		delete PhysicalItemClass::booleanOpThread;
		PhysicalItemClass::booleanOpThread = nullptr;
		PhysicalItemClass::booleanOpThreadFinished = false;
		PhysicalItemClass::deferredAdds.clear();
	}

	allBodies.clear();

	// We need these threads again...
	fizXSignal.started = false;
	fizXSignal.quit = false;

	entitiesSignal.quit = false;

	clothSignal.quit = false;
}

HIGHOMEGA::WORLD::DefaultPipelineSetupClass::DefaultPipelineSetupClass(std::string mapPath, std::string mapBelong, bool cmdOptHwRt, unsigned int cmdOptFullRes, bool cmdOptWindowed)
{
	PostProcessTri.Create(MainFrustum.eye, MainFrustum.look, MainFrustum.up, MainFrustum.screen_whr, MainFrustum.screen_fov);

	static bool MainMenuPassed = false;
	if (!MainMenuPassed)
	{
		MainMenu.Create(PostProcessTri, cmdOptHwRt, cmdOptFullRes, cmdOptWindowed);
		while (!MainMenu.IsDone())
		{
			Handler(); // We have no choice but to have this here because of SDL
			MainMenu.Render();
		}
		if (MainMenu.IsRebooting())
		{
			ApplicationRebooting = true;
			return;
		}

		MainMenuPassed = true;
	}

	zoneStreaming.Create(mapBelong, { &VisibilityPass.submission, &mainRTSubmission, &sdfBvhSubmission, &ShadowMapCascadeNear.submission, &ShadowMapCascadeFar.submission, &ScreenSpaceGather.submission },
	[&](GroupedRenderSubmission *inpSub, GraphicsModel *inpGraphicsModel) -> SubmittedRenderItem
	{
		if (inpSub == &ShadowMapCascadeNear.submission || inpSub == &ShadowMapCascadeFar.submission)
			return inpSub->Add(*inpGraphicsModel, GroupedRasterSubmission::everythingFilter);
		else if (inpSub == &ScreenSpaceGather.submission)
			return inpSub->Add(*inpGraphicsModel, GroupedRasterSubmission::postProcessOnlyFilter);
		else
			return inpSub->Add(*inpGraphicsModel);
	});
	CreateWorld(worldParams);

	SkyDome.Create(PostProcessTri, worldParams);
	ShadowMapCascadeNear.Create(worldParams);
	ShadowMapCascadeFar.Create(worldParams);
	VisibilityPass.Create(ScreenSize.width, ScreenSize.height, MainFrustum);
	GatherResolve.Create(VisibilityPass, PostProcessTri, MainFrustum);
	PathTrace.Create(PostProcessTri, GatherResolve, SkyDome, ShadowMapCascadeNear, ShadowMapCascadeFar, mainRTSubmission, &sdfBvhSubmission, worldParams.StartPlayerPos());
	ClearSurfaceCache.Create(PathTrace);
	TemporalAccumulate.Create(PostProcessTri, GatherResolve, PathTrace);
	SpatialDenoise.Create(PostProcessTri, PathTrace, TemporalAccumulate, GatherResolve);
	ShadowMapScreen.Create(PostProcessTri, ShadowMapCascadeNear, ShadowMapCascadeFar, GatherResolve, true);
	Modulate.Create(PostProcessTri, VisibilityPass, GatherResolve, PathTrace, SpatialDenoise, SkyDome, ShadowMapScreen);
	ScreenSpaceGather.Create(PostProcessTri, GatherResolve, SkyDome, worldParams);
	NearScattering.Create(PostProcessTri, GatherResolve, ShadowMapCascadeNear, ShadowMapCascadeFar, SkyDome, worldParams, ScreenSpaceGather, 512);
	ScreenSpaceFX.Create(PostProcessTri, SkyDome, GatherResolve, PathTrace, Modulate, ScreenSpaceGather, NearScattering);
	DoF.Create(PostProcessTri, GatherResolve, ScreenSpaceFX);

	SplashDisplay.Create(PostProcessTri);

	static bool startedMusic = true;
	if (!startedMusic)
	{
		AudioSharedMutex.lock();
		allSounds.push_back(std::unique_ptr <SoundBite> (new SoundBite("assets/audio/clair.wav", 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f)));
		AudioSharedMutex.unlock();
		startedMusic = true;
	}

	//animMat.Ident();
	//animId = animatedMeshesCollection.Add("source_material/dev_test_models/animated_test/animated_test.3md", "source_material/dev_test_models/animated_test/", 1.0f, animMat, nullptr, { &VisibilityPass.submission, &ShadowMap.submission, &mainRTSubmission });
	clothCollection.SetRenderSubmissions({ &VisibilityPass.submission, &ShadowMapCascadeNear.submission, &ShadowMapCascadeFar.submission, &mainRTSubmission });
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
			if (potentialMapChangeInfo.changeMap)
			{
				worldParams.EndFrameTimer();
				CommonSharedMutex.unlock_shared();
				break;
			}
			MainFrustum.ForceEyeAndLook(cameraSystem.curPos, cameraSystem.curDir);
		}
		MainFrustum.Update();
		PostProcessTri.UpdateViewSpace(MainFrustum.eye, MainFrustum.look, MainFrustum.up, MainFrustum.screen_whr, MainFrustum.screen_fov);

		//animatedMeshesCollection.Advance(animId, WorldParams.GetFrameTime(), true, animMat);
		plasteredItemsCollection.Update(worldParams.GetFrameTime());
		//animatedMeshesCollection.Process();
		physicalItemsCollection.Update([&](vec3 & inEye, vec3 & bodyPos, float bodyRad) -> bool { return zoneStreaming.isInDrawRegion(inEye, bodyPos, bodyRad); });
		clothCollection.UploadData();
		particleSystem.Update(worldParams.GetFrameTime());
		guidedModelSystem.Update(worldParams.GetFrameTime());
		mainMinMaxReducer.Process();

		Handler(); // We have no choice but to have this here because of SDL
		worldParams.AddSunAngle(worldParams.GetFrameTime() * 0.0075f);
		SkyDome.Render(worldParams, mainRTSubmission);
		Handler(); // We have no choice but to have this here because of SDL
		ShadowMapCascadeNear.Render(zoneStreaming.getVisbileMin(), zoneStreaming.getVisbileMax(), 0.0f, vec2 (0.00001f, 0.00002f), &MainFrustum);
		Handler(); // We have no choice but to have this here because of SDL
		ShadowMapCascadeFar.Render(zoneStreaming.getVisbileMin(), zoneStreaming.getVisbileMax(), 0.0f, vec2(0.001f, 0.002f));
		Handler(); // We have no choice but to have this here because of SDL
		VisibilityPass.Render();
		Handler(); // We have no choice but to have this here because of SDL
		GatherResolve.Render();
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
		DoF.SetMidScreenMessage(player.ladderInfo.inEntryZone);
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

		static unsigned long long droppedBoxId = 0u;

		CommonSharedMutex.unlock_shared();
		/*if (boxActionSignal)
		{
			boxActionSignal = false;

			if (droppedBoxId == 0u)
			{
				mat4 mapOrient;
				mapOrient.Ident();
				droppedBoxId = physicalItemsCollection.Add("source_material/dev_test_models/boxes/box_cloth_test.3md", "source_material/dev_test_models/boxes/", mapOrient, vec3(0.0f, 20.0f, 40.0f), false,
					[](GroupedRenderSubmission *curSubmission, GraphicsModel *inpModel) -> SubmittedRenderItem
				{
					return curSubmission->Add(*inpModel);
				}, { &VisibilityPass.submission, &mainRTSubmission, &sdfBvhSubmission, &ShadowMapCascadeNear.submission, &ShadowMapCascadeFar.submission });
			}
			else
			{
				physicalItemsCollection.Remove(droppedBoxId);
				droppedBoxId = 0u;
			}
		}*/

		static bool prevUse = false;
		if (!GetStateOfKeybAction(CMD_USE) && prevUse && !player.ladderInfo.climbingLadder)
		{
			RigidBody *hitObj;
			std::string hitGroupId;
			vec3 hitNorm;
			vec3 lineStart = MainFrustum.eye;
			vec3 lineEnd = MainFrustum.eye + MainFrustum.look * 50.0f;
			if (LineWorldClosest(lineStart, lineEnd, &hitObj, hitGroupId, hitNorm))
			{
				if (hitObj->isMap && hitObj->getGroupById(hitGroupId) && hitObj->getGroupById(hitGroupId)->breakable)
				{
					physicalItemsCollection.CutOut(hitObj, hitGroupId, MainFrustum.look * 20.0f, lineEnd, hitNorm, 3.0f, [](GroupedRenderSubmission *curSubmission, GraphicsModel *inpModel)->SubmittedRenderItem
					{
						return curSubmission->Add(*inpModel);
					}, { &VisibilityPass.submission, &mainRTSubmission, &sdfBvhSubmission, &ShadowMapCascadeNear.submission, &ShadowMapCascadeFar.submission });
				}
			}
		}

		zoneStreaming.Update(MainFrustum.eye);

		prevUse = GetStateOfKeybAction(CMD_USE);

		worldParams.EndFrameTimer();
	}
	DestroyWorld(potentialMapChangeInfo.changeMap);

	return potentialMapChangeInfo;
}

bool HIGHOMEGA::WORLD::DefaultPipelineSetupClass::IsApplicationRebooting()
{
	return ApplicationRebooting;
}