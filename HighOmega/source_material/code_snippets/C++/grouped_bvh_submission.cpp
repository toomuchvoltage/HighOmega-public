unsigned int HIGHOMEGA::RENDER::GroupedBVHSubmission::CompMortonWorkGroupSize()
{
	return 32;
}

unsigned int HIGHOMEGA::RENDER::GroupedBVHSubmission::CompMortonTrisPerThreadSize()
{
	return 1000;
}

unsigned int HIGHOMEGA::RENDER::GroupedBVHSubmission::CompressWorkGroupSize()
{
	return 32;
}

void HIGHOMEGA::RENDER::GroupedBVHSubmission::ChangeSignal(unsigned long long changedItem)
{
	sceneChanged = true;
}

unsigned long long HIGHOMEGA::RENDER::GroupedBVHSubmission::SceneID()
{
	if (RTInstance::Enabled()) return sceneID;

	if (sceneChanged)
	{
		totalInstanceCount = 0u;
		totalSceneTriCount = 0u;
		for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
			for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
			{
				MeshMaterial itFirst = curMatGeomPairing.first;
				if (!curSubmittedItem.second.filterFunction(itFirst)) continue;
				for (GeometryClass & curGeom : curMatGeomPairing.second)
				{
					totalSceneTriCount += curGeom.getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					totalInstanceCount++;
				}
			}

		instanceCountAligned = (unsigned int)(ceil((double)totalInstanceCount / 1000.0) * 1000.0);
		triCountAligned = (unsigned int)(ceil((double)totalSceneTriCount / 1000.0) * 1000.0);
		nodeCountAligned = (unsigned int)(ceil((double)(2 * totalSceneTriCount - 1) / 2000.0) * 2000.0);

		mortonParams.resize(instanceCountAligned);
		sourceGeom.reserve(instanceCountAligned);
		sourceGeom.clear();
		sourceMats.clear();
		trisComp.resize(triCountAligned);
		nodes.resize(nodeCountAligned);

		unsigned int triOffset = 0, curInstanceIndex = 0;
		unsigned int maxTriCount = 0;

		std::unordered_map <ImageClass*, HIGHOMEGA_TEXTURE_OFFSET> uniqueSamplers;
		for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
			for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
			{
				MeshMaterial curMat = curMatGeomPairing.first;
				if (!curSubmittedItem.second.filterFunction(curMat)) continue;
				for (GeometryClass & curGeom : curMatGeomPairing.second)
				{
					unsigned int curTriCount = curGeom.getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					sourceGeom.emplace_back(RESOURCE_SSBO, COMPUTE, 1, 0, curGeom.getVertBuffer());

					if (uniqueSamplers.find(&curMat.diffRef->elem) == uniqueSamplers.end())
						uniqueSamplers[&curMat.diffRef->elem] = (HIGHOMEGA_TEXTURE_OFFSET)uniqueSamplers.size();
					if (curMat.nrmRef && uniqueSamplers.find(&curMat.nrmRef->elem) == uniqueSamplers.end())
						uniqueSamplers[&curMat.nrmRef->elem] = (HIGHOMEGA_TEXTURE_OFFSET)uniqueSamplers.size();
					if (curMat.rghRef && uniqueSamplers.find(&curMat.rghRef->elem) == uniqueSamplers.end())
						uniqueSamplers[&curMat.rghRef->elem] = (HIGHOMEGA_TEXTURE_OFFSET)uniqueSamplers.size();
					if (curMat.hgtRef && uniqueSamplers.find(&curMat.hgtRef->elem) == uniqueSamplers.end())
						uniqueSamplers[&curMat.hgtRef->elem] = (HIGHOMEGA_TEXTURE_OFFSET)uniqueSamplers.size();
					if (curMat.spcRef && uniqueSamplers.find(&curMat.spcRef->elem) == uniqueSamplers.end())
						uniqueSamplers[&curMat.spcRef->elem] = (HIGHOMEGA_TEXTURE_OFFSET)uniqueSamplers.size();

					mortonParams[curInstanceIndex].OffsetLen[0] = triOffset;
					mortonParams[curInstanceIndex].OffsetLen[1] = curTriCount;
					triOffset += curTriCount;
					curInstanceIndex++;
					vec3 & rtInstMin = curGeom.getGeomMin();
					vec3 & rtInstMax = curGeom.getGeomMax();
					if (maxTriCount == 0)
					{
						buildParams.mapMinTriCount[0] = rtInstMin.x;
						buildParams.mapMinTriCount[1] = rtInstMin.y;
						buildParams.mapMinTriCount[2] = rtInstMin.z;
						buildParams.mapMax[0] = rtInstMax.x;
						buildParams.mapMax[1] = rtInstMax.y;
						buildParams.mapMax[2] = rtInstMax.z;
						maxTriCount = curTriCount;
					}
					else
					{
						buildParams.mapMinTriCount[0] = min(buildParams.mapMinTriCount[0], rtInstMin.x);
						buildParams.mapMinTriCount[1] = min(buildParams.mapMinTriCount[1], rtInstMin.y);
						buildParams.mapMinTriCount[2] = min(buildParams.mapMinTriCount[2], rtInstMin.z);
						buildParams.mapMax[0] = max(buildParams.mapMax[0], rtInstMax.x);
						buildParams.mapMax[1] = max(buildParams.mapMax[1], rtInstMax.y);
						buildParams.mapMax[2] = max(buildParams.mapMax[2], rtInstMax.z);
						maxTriCount = max(maxTriCount, curTriCount);
					}
				}
			}

		struct ImgRefOrder
		{
			ImageClass* img;
			unsigned int order;
		};
		std::vector<ImgRefOrder> ImgRefOrderVector;
		for (const std::pair<ImageClass*, HIGHOMEGA_TEXTURE_OFFSET>& curSampler : uniqueSamplers)
			ImgRefOrderVector.push_back(ImgRefOrder{ curSampler.first, curSampler.second });
		std::sort(ImgRefOrderVector.begin(), ImgRefOrderVector.end(), [](const ImgRefOrder& lhs, const ImgRefOrder& rhs) {
			return lhs.order <= rhs.order;
		});
		for (ImgRefOrder& curImgRefOrder : ImgRefOrderVector)
			sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 5, 0, *curImgRefOrder.img);

		buildParams.mapMinTriCount[3] = *((float *)(&totalSceneTriCount));

		if (buildParamsBuf.getSize() == 0)
			buildParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &buildParams, (unsigned int)sizeof(buildParamsStruct));
		else
			buildParamsBuf.UploadSubData(0, &buildParams, sizeof(buildParamsStruct));

		if (instanceCountAligned > 0)
		{
			unsigned int compMortonParamsBufSize = instanceCountAligned * sizeof(compMortonParamsStruct);
			if (compMortonParamsBufSize > compMortonParamsBuf.getSize())
			{
				compMortonParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, compMortonParamsBufSize);
			}
			compMortonParamsBuf.UploadSubData(0, mortonParams.data(), (unsigned int)totalInstanceCount * sizeof(compMortonParamsStruct));
		}
		else
		{
			if (compMortonParamsBuf.getSize() == 0) compMortonParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(compMortonParamsStruct));
		}

		if (triCountAligned > 0)
		{
			unsigned int triBufSize = triCountAligned * sizeof(BVHTriangle);
			if (triBufSize > triBuf.getSize())
				triBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, triBufSize);
		}
		else
		{
			if (triBuf.getSize() == 0) triBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHTriangle));
		}

		if (instanceCountAligned > 0)
		{
			unsigned int instBufSize = instanceCountAligned * sizeof(InstanceProperties);
			if (instBufSize > instBuf.getSize())
			{
				instBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, instBufSize);
			}
			instProps.resize(totalInstanceCount);
			unsigned int instancePropCounter = 0u;
			HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6];
			for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
				for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
				{
					MeshMaterial curMat = curMatGeomPairing.first;
					if (!curSubmittedItem.second.filterFunction(curMat)) continue;

					textureOffsets[0] = 0xFF;
					textureOffsets[1] = textureOffsets[2] = textureOffsets[3] = textureOffsets[4] = textureOffsets[5] = uniqueSamplers[&curMat.diffRef->elem];
					if (curMat.nrmRef) textureOffsets[2] = uniqueSamplers[&curMat.nrmRef->elem];
					if (curMat.rghRef) textureOffsets[3] = uniqueSamplers[&curMat.rghRef->elem];
					if (curMat.hgtRef) textureOffsets[4] = uniqueSamplers[&curMat.hgtRef->elem];
					if (curMat.spcRef) textureOffsets[5] = uniqueSamplers[&curMat.spcRef->elem];

					CompileInstanceProperties(instProps[instancePropCounter++], curMat, textureOffsets);
				}
			instBuf.UploadSubData(0, instProps.data(), totalInstanceCount * sizeof(InstanceProperties));
		}
		else
		{
			if (instBuf.getSize() == 0) instBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(InstanceProperties));
		}

		if (nodeCountAligned > 0)
		{
			unsigned int nodesBufSize = nodeCountAligned * sizeof(BVHNode);
			if (nodesBufSize > nodesBuf.getSize())
				nodesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, nodesBufSize);
		}
		else
		{
			if (nodesBuf.getSize() == 0) nodesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHNode));
		}

		if (triCountAligned > 0)
		{
			unsigned int compressedTrisBufSize = triCountAligned * sizeof(BVHTriangleCompressed);
			if (compressedTrisBufSize > trisCompressedBuf.getSize())
				trisCompressedBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, compressedTrisBufSize);
		}
		else
		{
			if (trisCompressedBuf.getSize() == 0) trisCompressedBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHTriangleCompressed));
		}

		if (shadersCreated)
		{
			mortonShader.RemovePast();
			compressionShader.RemovePast();
			shadersCreated = false;
		}
		mortonShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 0, buildParamsBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, compMortonParamsBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, triBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 1, sourceGeom);
		mortonShader.Create("shaders/computeMortonCodes.comp.spv", "main");
		mortonSubmission.MakeDispatch(Instance, std::string("default"), mortonShader, (unsigned int)ceil((double)maxTriCount / (double)(CompMortonWorkGroupSize() * CompMortonTrisPerThreadSize())), curInstanceIndex, 1);

		compressionShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 0, buildParamsBuf);
		compressionShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, triBuf);
		compressionShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, instBuf);
		compressionShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, trisCompressedBuf);
		compressionShader.Create("shaders/compressTris.comp.spv", "main");
		compressionSubmission.MakeDispatch(Instance, std::string("default"), compressionShader, (unsigned int)ceil((double)totalSceneTriCount / CompressWorkGroupSize()), 1, 1);

		shadersCreated = true;
		sceneChanged = false;

		sceneID = mersenneTwister64BitPRNG();
	}
	else
	{
		bool firstComparison = true;
		for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
		{
			for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
			{
				MeshMaterial curMat = curMatGeomPairing.first;
				if (!curSubmittedItem.second.filterFunction(curMat)) continue;
				for (GeometryClass & curGeom : curMatGeomPairing.second)
				{
					vec3 & rtInstMin = curGeom.getGeomMin();
					vec3 & rtInstMax = curGeom.getGeomMax();
					if (firstComparison)
					{
						firstComparison = false;
						buildParams.mapMinTriCount[0] = rtInstMin.x;
						buildParams.mapMinTriCount[1] = rtInstMin.y;
						buildParams.mapMinTriCount[2] = rtInstMin.z;
						buildParams.mapMax[0] = rtInstMax.x;
						buildParams.mapMax[1] = rtInstMax.y;
						buildParams.mapMax[2] = rtInstMax.z;
					}
					else
					{
						buildParams.mapMinTriCount[0] = min(buildParams.mapMinTriCount[0], rtInstMin.x);
						buildParams.mapMinTriCount[1] = min(buildParams.mapMinTriCount[1], rtInstMin.y);
						buildParams.mapMinTriCount[2] = min(buildParams.mapMinTriCount[2], rtInstMin.z);
						buildParams.mapMax[0] = max(buildParams.mapMax[0], rtInstMax.x);
						buildParams.mapMax[1] = max(buildParams.mapMax[1], rtInstMax.y);
						buildParams.mapMax[2] = max(buildParams.mapMax[2], rtInstMax.z);
					}
				}
			}
		}
		buildParamsBuf.UploadSubData(0, &buildParams, sizeof(buildParamsStruct));
	}
	mortonSubmission.Submit();

	mapMin = vec3(buildParams.mapMinTriCount[0], buildParams.mapMinTriCount[1], buildParams.mapMinTriCount[2]);
	mapMax = vec3(buildParams.mapMax[0], buildParams.mapMax[1], buildParams.mapMax[2]);

	tris.resize(triCountAligned);
	trisTmp.resize(triCountAligned);
	triBuf.DownloadSubData(0, tris.data(), totalSceneTriCount * sizeof(BVHTriangle));

	const unsigned int numPredicates = 256;
	for (unsigned int bitShift = 0; bitShift != 32; bitShift += 8)
	{
		unsigned int bitMask = (0x000000FFu) << bitShift;
		BVHTriangle *srcBuf, *dstBuf;
		if ((bitShift / 8) % 2 == 0)
		{
			srcBuf = tris.data();
			dstBuf = trisTmp.data();
		}
		else
		{
			srcBuf = trisTmp.data();
			dstBuf = tris.data();
		}
		unsigned int predicateSums[numPredicates];
		unsigned int predicateOffets[numPredicates];
		for (unsigned int i = 0; i != numPredicates; i++)
			predicateSums[i] = 0;
		for (unsigned int i = 0; i != totalSceneTriCount; i++)
			predicateSums[((*(unsigned int *)(&srcBuf[i].e3Morton[3])) & bitMask) >> bitShift]++;
		for (unsigned int i = 0; i != numPredicates; i++)
		{
			predicateOffets[i] = 0;
			for (unsigned int j = 0; j != i; j++)
				predicateOffets[i] += predicateSums[j];
		}
		for (unsigned int i = 0; i != totalSceneTriCount; i++)
		{
			unsigned int curPred = ((*(unsigned int *)(&srcBuf[i].e3Morton[3]) & bitMask)) >> bitShift;
			dstBuf[predicateOffets[curPred]] = srcBuf[i];
			predicateOffets[curPred]++;
		}
	}

	triBuf.UploadSubData(0, tris.data(), totalSceneTriCount * sizeof(BVHTriangle));

	compressionSubmission.Submit();

	bvhGenCPU.ProduceNodesOnly(tris.data(), nodes.data(), totalSceneTriCount);

	nodesBuf.UploadSubData(0, nodes.data(), (2 * totalSceneTriCount - 1) * sizeof(BVHNode));

	return sceneID;
}

SubmittedRenderItem HIGHOMEGA::RENDER::GroupedBVHSubmission::Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial&curMat)> inpFilterFunction)
{
	SubmittedRenderItem retVal;
	retVal.producer = this;
	retVal.item = &inpModel;
	retVal.itemId = mersenneTwister64BitPRNG();
	retVal.filterFunction = inpFilterFunction;

	allSubmittedItems[retVal.itemId] = retVal;

	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = inpModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions[this] = retVal.itemId;
	}

	sceneChanged = true;

	return retVal;
}

void HIGHOMEGA::RENDER::GroupedBVHSubmission::Remove(SubmittedRenderItem & inpSubmittedRenderItem)
{
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = allSubmittedItems[inpSubmittedRenderItem.itemId].item->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions.erase(this);

	allSubmittedItems.erase(inpSubmittedRenderItem.itemId);

	sceneChanged = true;
}

...

void HIGHOMEGA::RENDER::GroupedRasterSubmission::requestBVH(GroupedBVHSubmission & bvhHolder)
{
	sourceBVH = &bvhHolder;
	sourceBVHId = 0u;
	requestsBVH = true;
}

...

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Render()
{
	if (frameBuffer == nullptr || shaders.size() == 0) return;

	unsigned long long curBVHId;
	if (requestsBVH && sourceBVHId != (curBVHId = sourceBVH->SceneID()))
	{
		sourceBVHId = curBVHId;
		if (recordedCmdBuf)
		{
			allResourcesChanged = true;
			redoSubmissionData = true;
		}
	}
	
	...
	
			if (requestsBVH)
			{
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, 0, sourceBVH->nodesBuf);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, 1, sourceBVH->trisCompressedBuf);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, 2, sourceBVH->instBuf);
				shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 5, sourceBVH->sourceMats);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 6, sourceBVH->sourceGeom);
			}
	...
}