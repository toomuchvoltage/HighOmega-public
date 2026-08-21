void HIGHOMEGA::FIZ_X::AddRange(std::vector<range>& inpRangeVector, vec3 rmin, vec3 rmax)
{
	for (range & curRange : inpRangeVector)
	{
		if (rmax.x < curRange.rmin.x ||
			rmax.y < curRange.rmin.y ||
			rmax.z < curRange.rmin.z ||
			rmin.y > curRange.rmax.y ||
			rmin.z > curRange.rmax.z ||
			rmin.y > curRange.rmax.y) continue;
		curRange.rmin.x = min(rmin.x, curRange.rmin.x);
		curRange.rmin.y = min(rmin.y, curRange.rmin.y);
		curRange.rmin.z = min(rmin.z, curRange.rmin.z);
		curRange.rmax.x = max(rmax.x, curRange.rmax.x);
		curRange.rmax.y = max(rmax.y, curRange.rmax.y);
		curRange.rmax.z = max(rmax.z, curRange.rmax.z);
		return;
	}
	range newRange;
	newRange.rmin = rmin;
	newRange.rmax = rmax;
	inpRangeVector.push_back(newRange);
}

std::vector<range> HIGHOMEGA::FIZ_X::FindDynamicRanges(vec3 tMin, vec3 tMax)
{
	// This is not an exhaustive search... 
	// ... and was mainly used to just update dynamic parts of distance fields in compute
	std::vector <range> allRanges;

	vec3 curMin, curMax;
	for (ConstraintCollection * curConstraints : constraintCollection)
	{
		curConstraints->GetEntireBodyMinMax(curMin, curMax);
		AddRange(allRanges, curMin, curMax);
	}
	for (ClothPiece it : clothCollection.clothItems)
		AddRange(allRanges, it.clothMin, it.clothMax);

	AddRange(allRanges, tMin, tMax);

	return allRanges;
}

void HIGHOMEGA::RENDER::PASSES::DFClass::Create(GPUGridBuilderClass & SceneVoxelizer, GPUGridBuilderClass & Voxelizer)
{
	submission[0].Create(Instance, Voxelizer.getGridX(), Voxelizer.getGridY(), Voxelizer.getGridZ());
	submission[1].Create(Instance, Voxelizer.getGridX(), Voxelizer.getGridY(), Voxelizer.getGridZ());
	computeParams.gridDims[0] = (float)Voxelizer.getGridX() - 1;
	computeParams.gridDims[1] = (float)Voxelizer.getGridY() - 1;
	computeParams.gridDims[2] = (float)Voxelizer.getGridZ() - 1;
	computeParams.startGridCoord[0] = 0;
	computeParams.startGridCoord[1] = 0;
	computeParams.startGridCoord[2] = 0;
	computeParams.endGridCoord[0] = Voxelizer.getGridX() - 1;
	computeParams.endGridCoord[1] = Voxelizer.getGridY() - 1;
	computeParams.endGridCoord[2] = Voxelizer.getGridZ() - 1;
	new (&computeParamsBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &computeParams, sizeof(computeParams));
	dfImage.CreateImageStore(Instance, R8UN, Voxelizer.getGridX(), Voxelizer.getGridY(), Voxelizer.getGridZ(), _3D, false);
	dfImageVector.push_back(&dfImage);
	dfImage.ClearColors(dfImageVector, ImageClearColor (vec3(5.0f / 255.0f), 5.0f / 255.0f));
	dynamicDFImage.CreateImageStore(Instance, R8UN, Voxelizer.getGridX(), Voxelizer.getGridY(), Voxelizer.getGridZ(), _3D, false);
	dynamicDFImageVector.push_back(&dynamicDFImage);

	shader[0].Create("shaders/df.comp.spv", "main");
	shader[0].AddResource(ShaderResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 0, SceneVoxelizer.gridScene));
	shader[0].AddResource(ShaderResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, dfImage));
	shader[0].AddResource(ShaderResource(RESOURCE_UBO, COMPUTE, 0, 2, computeParamsBuf));

	submission[0].SetShader("default", shader[0]);

	shader[1].Create("shaders/df.comp.spv", "main");
	shader[1].AddResource(ShaderResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 0, Voxelizer.gridScene));
	shader[1].AddResource(ShaderResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, dynamicDFImage));
	shader[1].AddResource(ShaderResource(RESOURCE_UBO, COMPUTE, 0, 2, computeParamsBuf));

	submission[1].SetShader("default", shader[1]);
}

void HIGHOMEGA::RENDER::PASSES::DFClass::UpdateDynamic(std::vector <range> & allDynamicRanges, GPUGridBuilderClass & SceneVoxelizer)
{
	//if (RTXInstance::Enabled()) return; // last pipeline stuck to voxel cone traced radiosity maps which needed distance fields

	dfImage.CopyImages(dfImageVector, dynamicDFImageVector);
	for (range & curRange : allDynamicRanges)
	{
		vec3 & updateMin = curRange.rmin;
		vec3 & updateMax = curRange.rmax;
		computeParams.startGridCoord[0] = (unsigned int)max(floor((updateMin.x - SceneVoxelizer.gridInfo.mapMinTrisPerCell[0]) / SceneVoxelizer.VoxelDim()) - 5.0f, 0.0f);
		computeParams.startGridCoord[1] = (unsigned int)max(floor((updateMin.y - SceneVoxelizer.gridInfo.mapMinTrisPerCell[1]) / SceneVoxelizer.VoxelDim()) - 5.0f, 0.0f);
		computeParams.startGridCoord[2] = (unsigned int)max(floor((updateMin.z - SceneVoxelizer.gridInfo.mapMinTrisPerCell[2]) / SceneVoxelizer.VoxelDim()) - 5.0f, 0.0f);
		computeParams.endGridCoord[0] = (unsigned int)ceil((updateMax.x - SceneVoxelizer.gridInfo.mapMaxVoxelSize[0]) / SceneVoxelizer.VoxelDim()) + 5;
		computeParams.endGridCoord[1] = (unsigned int)ceil((updateMax.y - SceneVoxelizer.gridInfo.mapMaxVoxelSize[1]) / SceneVoxelizer.VoxelDim()) + 5;
		computeParams.endGridCoord[2] = (unsigned int)ceil((updateMax.z - SceneVoxelizer.gridInfo.mapMaxVoxelSize[2]) / SceneVoxelizer.VoxelDim()) + 5;

		computeParamsBuf.Map(0, sizeof(computeParams));
		computeParamsBuf.UploadSubData(0, &computeParams, sizeof(computeParams));

		submission[1].Submit();
	}
}

void HIGHOMEGA::RENDER::PASSES::DFClass::UpdateBase(GPUGridBuilderClass & SceneVoxelizer)
{
	//if (RTXInstance::Enabled()) return; // last pipeline stuck to voxel cone traced radiosity maps which needed distance fields

	computeParams.startGridCoord[0] = 0;
	computeParams.startGridCoord[1] = 0;
	computeParams.startGridCoord[2] = 0;
	computeParams.endGridCoord[0] = SceneVoxelizer.getGridX() - 1;
	computeParams.endGridCoord[1] = SceneVoxelizer.getGridY() - 1;
	computeParams.endGridCoord[2] = SceneVoxelizer.getGridZ() - 1;

	computeParamsBuf.Map(0, sizeof(computeParams));
	computeParamsBuf.UploadSubData(0, &computeParams, sizeof(computeParams));

	for (int i = 0; i != 3; i++)
		submission[0].Submit();
}


void HIGHOMEGA::RENDER::PASSES::PreIlluminateClass::Create(GPUGridBuilderClass & Voxelizer, DFClass & DF, SkyDomeClass & SkyDome, ShadowMapClass & ShadowMap)
{
	voxelizerRef = &Voxelizer;
	skyDomeRef = &SkyDome;
	shadowMapRef = &ShadowMap;

	submission.Create(Instance, Voxelizer.getGridX(), Voxelizer.getGridY(), Voxelizer.getGridZ());
	preIlluminateImage.CreateImageStore(Instance, R16G16B16A16F, Voxelizer.getGridX(), Voxelizer.getGridY(), Voxelizer.getGridZ(), _3D, false);
	preIllumVector.push_back(&preIlluminateImage);
	preIlluminateImage.ClearColors(preIllumVector, ImageClearColor (vec3(0.0f), 0.0f));
	
	/*if (RTXInstance::Enabled()) // last pipeline stuck to voxel cone traced radiosity maps which needed distance fields
	{
		rtxShaderState.CreateRTX("shaders/rtxpreilluminate.rgen.spv", "main", "shaders/rtxpreilluminate.rchit.spv", "main", "shaders/rtxpreilluminate.rmiss.spv", "main");
		tracelet.Create(Instance);
	}*/

	shader.Create("shaders/preIlluminate.comp.spv", "main");
	shader.AddResource(ShaderResource(RESOURCE_SAMPLER, COMPUTE, 0, 0, Voxelizer.gridScene));
	shader.AddResource(ShaderResource(RESOURCE_SAMPLER, COMPUTE, 0, 4, DF.dynamicDFImage));
	shader.AddResource(ShaderResource(RESOURCE_SAMPLER, COMPUTE, 0, 5, SkyDome.fullCubeMap));
	shader.AddResource(ShaderResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 6, preIlluminateImage));
	shader.AddResource(ShaderResource(RESOURCE_UBO, COMPUTE, 0, 7, Voxelizer.gridInfoBuf));
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::PreIlluminateClass::ClearColors()
{
	preIlluminateImage.ClearColors(preIllumVector, ImageClearColor (vec3 (0.0f), 0.0f));
}

void HIGHOMEGA::RENDER::PASSES::PreIlluminateClass::Submit()
{
	ClearColors();

	/*if (RTXInstance::Enabled()) // last pipeline stuck to voxel cone traced radiosity maps which needed distance fields
	{
		RTXScene & rtxSceneRef = mainRTXSubmission->rtxScene;
		if (rtxSceneRef.IsSceneNew())
		{
			tracelet.Destroy();
			tracingResources.clear();
			tracingResources.push_back(ShaderResource(RESOURCE_RTX_ACCEL_STRUCT, RTX_RAYGEN | RTX_RCHIT, 0, 0, rtxSceneRef));
			tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 1, voxelizerRef->diffuse3DImage));
			tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 2, voxelizerRef->normal3DImage));
			tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 3, voxelizerRef->emissivityRefractiveIndexAndRoughness3DImage));
			tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 4, voxelizerRef->specular3DImage));
			tracingResources.push_back(ShaderResource(RESOURCE_IMAGE_STORE, RTX_RAYGEN | RTX_RCHIT, 0, 5, preIlluminateImage));
			tracingResources.push_back(ShaderResource(RESOURCE_UBO, RTX_RAYGEN | RTX_RCHIT, 0, 6, voxelizerRef->gridInfoBuf));
			tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_MISS, 0, 7, skyDomeRef->fullCubeMap));
			tracingResources.push_back(ShaderResource(RESOURCE_UBO, RTX_RAYGEN, 0, 8, shadowMapRef->sunDirBuf));
			std::vector <ShaderResource> variableCountInstanceBuffer, variableCountCompleteVertexBuffer, variableCountSamplers;
			for (int i = 0; i != rtxSceneRef.allTraceItems.size(); i++)
				variableCountInstanceBuffer.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT, 1, 0, rtxSceneRef.instancePropertiesBuffers[i]));
			for (std::pair <const unsigned long long, TraceItem> & curTraceItemKV : rtxSceneRef.allTraceItems)
				variableCountCompleteVertexBuffer.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT, 2, 0, curTraceItemKV.second.geomRef->completeVertexBuffer));
			for (std::pair <const unsigned long long, TraceItem> & curTraceItemKV : rtxSceneRef.allTraceItems)
				for (ImageClass * curImage : curTraceItemKV.second.material)
					variableCountSamplers.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RCHIT, 3, 0, *curImage));
			tracingResources.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT, 1, 0, variableCountInstanceBuffer));
			tracingResources.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT, 2, 0, variableCountCompleteVertexBuffer));
			tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RCHIT, 3, 0, variableCountSamplers));
		}

		tracelet.Submit(voxelizerRef->getGridX(), voxelizerRef->getGridY(), voxelizerRef->getGridZ(), tracingResources, rtxShaderState);
	}
	else*/
	{
		submission.Submit();
	}
}

