void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Create(TriClass& PostProcessTri, DownSampleClass& DownSample, VoxelizerClass& SceneVoxelizer, VoxelizerClass& Voxelizer, SkyDomeClass& SkyDome, ShadowMapClass& ShadowMap)
{
	if (blueNoise->getWidth() == 0) blueNoise->CreateTexture(Instance, "assets/common/", "bluenoise.tga", 1, false, false, false, false);

	triRef = &PostProcessTri;
	downSampleRef = &DownSample;
	skyDomeRef = &SkyDome;
	shadowMapRef = &ShadowMap;
	staticVoxelizerRef = &SceneVoxelizer;
	dynamicVoxelizerRef = &Voxelizer;

	PathTraceParams.motionFactorDiffuseOnlyReserved[0] = 0.0f;
	PathTraceParams.motionFactorDiffuseOnlyReserved[1] = RTInstance::Enabled() ? 1.0f : 0.0f;
	PathTraceParams.motionFactorDiffuseOnlyReserved[2] = 0.0f;
	PathTraceParams.motionFactorDiffuseOnlyReserved[3] = 0.0f;
	PathTraceParams.timeTurnDirectionRawLight[0] = 0.0f;
	PathTraceParams.timeTurnDirectionRawLight[1] = 0.0f;
	PathTraceParams.timeTurnDirectionRawLight[2] = 0.0f;
	PathTraceParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &PathTraceParams, (unsigned int)sizeof(PathTraceParams));

	glossTraceOutput.CreateImageStore(Instance, R16G16B16A16F, DownSample.worldPosAttach.getWidth(), DownSample.worldPosAttach.getHeight(), 1, _2D, false);

	if (RTInstance::Enabled())
	{
		rtShaderResourceSet.CreateRT("shaders/rtpathtrace.rgen.spv", "main", "shaders/rtpathtrace.rchit.spv", "main", "shaders/rtpathtrace.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
		tracelet.Make(Instance);

		shader.Create("shaders/postprocess.vert.spv", "main", "shaders/pathtraceDiffuseOnly.frag.spv", "main");

		shader.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, downSampleRef->materialAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, downSampleRef->worldPosAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, downSampleRef->normalAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, SkyDome.fullCubeMap);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, SceneVoxelizer.voxelizedInfoBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, SceneVoxelizer.voxelizedScene);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 7, ShadowMap.frustum.Buffer);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, SkyDome.rayleighMieBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, ShadowMap.shadowDistanceAttach);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 10, ShadowMap.shadowColorAttach);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 11, AllNEEDataBuf);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 12, *blueNoise);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 13, PathTraceParamsBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 14, SceneVoxelizer.radiosityMaps[0]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 15, SceneVoxelizer.radiosityMaps[1]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 16, SceneVoxelizer.radiosityMaps[2]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 17, SceneVoxelizer.radiosityMaps[3]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 18, SceneVoxelizer.radiosityMaps[4]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 19, SceneVoxelizer.radiosityMaps[5]);
	}
	else
	{
		shader.Create("shaders/postprocess.vert.spv", "main", "shaders/pathtrace.frag.spv", "main");

		shader.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, downSampleRef->materialAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, downSampleRef->worldPosAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, downSampleRef->normalAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, SkyDome.fullCubeMap);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, SceneVoxelizer.voxelizedInfoBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, SceneVoxelizer.voxelizedScene);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 7, ShadowMap.frustum.Buffer);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, SkyDome.rayleighMieBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, ShadowMap.shadowDistanceAttach);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 10, ShadowMap.shadowColorAttach);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 11, glossTraceOutput);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 12, AllNEEDataBuf);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 13, *blueNoise);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 14, PathTraceParamsBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 15, SceneVoxelizer.radiosityMaps[0]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 16, SceneVoxelizer.radiosityMaps[1]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 17, SceneVoxelizer.radiosityMaps[2]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 18, SceneVoxelizer.radiosityMaps[3]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 19, SceneVoxelizer.radiosityMaps[4]);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 20, SceneVoxelizer.radiosityMaps[5]);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 21, SceneVoxelizer.radiosityMaps[0]);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 22, SceneVoxelizer.radiosityMaps[1]);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 23, SceneVoxelizer.radiosityMaps[2]);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 24, SceneVoxelizer.radiosityMaps[3]);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 25, SceneVoxelizer.radiosityMaps[4]);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 26, SceneVoxelizer.radiosityMaps[5]);
	}
	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	frameBuffer.setWidth(ScreenSize.width);
	frameBuffer.setHeight(ScreenSize.height);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);
	submission.SetFrameBuffer(frameBuffer);

	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Render(GroupedTraceSubmission& rtSubmission)
{
	if (RTInstance::Enabled())
	{
		RTScene& rtSceneRef = rtSubmission.rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = rtSubmission.SceneID();
		if (curSceneId != lastSceneId)
		{
			lastSceneId = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources.clear();
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN | RT_RCHIT, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 1, downSampleRef->materialAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 2, downSampleRef->worldPosAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 3, downSampleRef->normalAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN | RT_RCHIT | RT_MISS, 0, 4, skyDomeRef->fullCubeMap);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RCHIT, 0, 5, shadowMapRef->frustum.Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN | RT_RCHIT, 0, 6, skyDomeRef->rayleighMieBuf);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN | RT_RCHIT, 0, 7, shadowMapRef->shadowDistanceAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN | RT_RCHIT, 0, 8, shadowMapRef->shadowColorAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN | RT_RCHIT, 0, 9, AllNEEDataBuf);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 10, triRef->triFrustum.Buffer);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 11, glossTraceOutput);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 12, *blueNoise);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN | RT_RCHIT, 0, 13, PathTraceParamsBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT, 0, 14, rtSceneRef.instancePropertiesBuffer);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RCHIT | RT_MISS, 0, 15, staticVoxelizerRef->radiosityMaps[0]);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RCHIT | RT_MISS, 0, 16, staticVoxelizerRef->radiosityMaps[1]);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RCHIT | RT_MISS, 0, 17, staticVoxelizerRef->radiosityMaps[2]);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RCHIT | RT_MISS, 0, 18, staticVoxelizerRef->radiosityMaps[3]);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RCHIT | RT_MISS, 0, 19, staticVoxelizerRef->radiosityMaps[4]);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RCHIT | RT_MISS, 0, 20, staticVoxelizerRef->radiosityMaps[5]);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT, 0, 21, staticVoxelizerRef->radiosityMaps[0]);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT, 0, 22, staticVoxelizerRef->radiosityMaps[1]);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT, 0, 23, staticVoxelizerRef->radiosityMaps[2]);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT, 0, 24, staticVoxelizerRef->radiosityMaps[3]);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT, 0, 25, staticVoxelizerRef->radiosityMaps[4]);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT, 0, 26, staticVoxelizerRef->radiosityMaps[5]);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RCHIT | RT_MISS, 0, 27, staticVoxelizerRef->voxelizedInfoBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 1, 0, rtSceneRef.getGeomResources());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 2, 0, rtSceneRef.getMaterialResources());
		}
		tracelet.Submit(downSampleRef->materialAttach.getWidth(), downSampleRef->materialAttach.getHeight(), 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
	}

	if (!bvhBuilderRun && !RTInstance::Enabled())
	{
		// We should supply the BVH submission here...
		bvhBuilderRun = true;
	}

	submission.Render();
}