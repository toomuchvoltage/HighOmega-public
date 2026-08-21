void HIGHOMEGA::RENDER::PASSES::TestBVHTraceClass::Create(TriClass & PostProcessTri, void * tris, void * nodes, unsigned int triSize, unsigned int nodeSize)
{
	mat4 triMat;
	triMat.Ident();

	testParams.time = 0.0f;
	new (&testParamsBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &testParams, sizeof(testParams));
	new (&triBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, tris, triSize);
	new (&nodeBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nodes, nodeSize);

	testSubmission.Add(PostProcessTri.triModel, triMat);
	testSubmission.Create(Instance);

	testSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	testShader.Create("shaders/postprocess.vert.spv", "main", "shaders/testBVH.frag.spv", "main");
	testShader.AddResource(ShaderResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 1, testParamsBuf));
	testShader.AddResource(ShaderResource(RESOURCE_SSBO, FRAGMENT, 0, 2, triBuf));
	testShader.AddResource(ShaderResource(RESOURCE_SSBO, FRAGMENT, 0, 3, nodeBuf));
	testSubmission.SetShader("default", testShader);
}

void HIGHOMEGA::RENDER::PASSES::TestBVHTraceClass::Render()
{
	testParams.time += 0.1f;
	testParamsBuf.Map(0, sizeof(testParams));
	testParamsBuf.UploadSubData(0, &testParams, sizeof(testParams));
	if (!HIGHOMEGA::EVENTS::windowMinimized) testSubmission.Render();
}

void HIGHOMEGA::RENDER::PASSES::TestGPUGridTraceClass::Create(TriClass & PostProcessTri, GPUGridBuilderClass & GPUGridBuilder, GPUGridBuilderClass & dynamicGPUGridBuilder, GatherPassClass & GatherPass, SkyDomeClass & SkyDome)
{
	refGridBuilder = &GPUGridBuilder;

	mat4 triMat;
	triMat.Ident();

	memset(onScreenText, (int)(' '), 60 * 5);
	onScreenTextImage.CreateTexture(Instance, 60, 5, onScreenText, 1u, false, false, false, false, 1);
	fontMap.CreateTexture(Instance, "assets/common/fontmap.tga");

	testSubmission.Add(PostProcessTri.triModel, triMat);
	testSubmission.Create(Instance);

	testSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	testShader.Create("shaders/postprocess.vert.spv", "main", "shaders/testGPUGrid.frag.spv", "main");
	testShader.AddResource(ShaderResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer));
	testShader.AddResource(ShaderResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GPUGridBuilder.gridScene));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 2, GPUGridBuilder.gridInfoBuf));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherPass.materialAttach));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, GatherPass.worldPosAttach));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, GatherPass.normalAttach));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, SkyDome.backDrop));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, SkyDome.fullCubeMap));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, onScreenTextImage));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, fontMap));
	testSubmission.SetShader("default", testShader);
	testSubmission.requestGPUGridGeom(GPUGridBuilder.submission);
	testSubmission.requestGPUGridGeom(dynamicGPUGridBuilder.submission);
}

void HIGHOMEGA::RENDER::PASSES::TestGPUGridTraceClass::Render()
{
	sprintf_s((char *)onScreenText, 300, "Rendered using our technique on RTX 2080Ti");
	sprintf_s((char *)(&onScreenText[60]), 300, "Resolution: %dx%d", ScreenSize.width, ScreenSize.height);
	sprintf_s((char *)(&onScreenText[60 * 2]), 300, "voxelDim: %.2f^3 cubic inches  maxTrisPerCell: %d", refGridBuilder->VoxelDim(), refGridBuilder->MaxTrisPerCell());
	unsigned long long memTotal = refGridBuilder->getGridX() * refGridBuilder->getGridY() * refGridBuilder->getGridZ() * refGridBuilder->MaxTrisPerCell() * sizeof(unsigned int) / (1024 * 1024);
	sprintf_s((char *)(&onScreenText[60 * 3]), 300, "Mem usage: %uMBs", (unsigned int)memTotal);
	if (INSTRUMENTATION::totalFrameTimeCount > 0) sprintf_s((char *)(&onScreenText[60 * 4]), 300, "Frame cost  min: %.2fms  max: %.2fms  avg: %.2fms", INSTRUMENTATION::minFrameCost * 1000.0f, INSTRUMENTATION::maxFrameCost * 1000.0f, (INSTRUMENTATION::totalFrameCost / (float)INSTRUMENTATION::totalFrameTimeCount) * 1000.0f);

	onScreenTextImage.ReuploadData(60, 5, onScreenText);
	if (!HIGHOMEGA::EVENTS::windowMinimized) testSubmission.Render();
}


void HIGHOMEGA::RENDER::PASSES::TestGPUGridPrimaryTraceClass::Create(TriClass & PostProcessTri, GPUGridBuilderClass & GPUGridBuilder, GPUGridBuilderClass & dynamicGPUGridBuilder, GatherPassClass & GatherPass, SkyDomeClass & SkyDome)
{
	mat4 triMat;
	triMat.Ident();

	testParams.time = 0.0f;
	new (&testParamsBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &testParams, sizeof(testParams));

	testSubmission.Add(PostProcessTri.triModel, triMat);
	testSubmission.Create(Instance);

	testSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	testShader.Create("shaders/postprocess.vert.spv", "main", "shaders/testGPUGridPrimary.frag.spv", "main");
	testShader.AddResource(ShaderResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 1, testParamsBuf));
	testShader.AddResource(ShaderResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GPUGridBuilder.gridScene));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 3, GPUGridBuilder.gridInfoBuf));
	testSubmission.SetShader("default", testShader);
	testSubmission.requestGPUGridGeom(GPUGridBuilder.submission);
	testSubmission.requestGPUGridGeom(dynamicGPUGridBuilder.submission);
}

void HIGHOMEGA::RENDER::PASSES::TestGPUGridPrimaryTraceClass::Render()
{
	testParams.time += 0.01f;
	testParamsBuf.Map(0, sizeof(testParams));
	testParamsBuf.UploadSubData(0, &testParams, sizeof(testParams));

	if (!HIGHOMEGA::EVENTS::windowMinimized) testSubmission.Render();
}

void HIGHOMEGA::RENDER::PASSES::TestRadeonRaysTraceClass::Create(TriClass & PostProcessTri, unsigned int rrWidth, unsigned int rrHeight, unsigned char * rrOutput)
{
	mat4 triMat;
	triMat.Ident();

	rrOutputImage.CreateTexture(Instance, rrWidth, rrHeight, rrOutput);

	testSubmission.Add(PostProcessTri.triModel, triMat);
	testSubmission.Create(Instance);

	testSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	testShader.Create("shaders/postprocess.vert.spv", "main", "shaders/testRadeonRays.frag.spv", "main");
	testShader.AddResource(ShaderResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, rrOutputImage));
	testSubmission.SetShader("default", testShader);

}

void HIGHOMEGA::RENDER::PASSES::TestRadeonRaysTraceClass::Render()
{
	if (!HIGHOMEGA::EVENTS::windowMinimized) testSubmission.Render();
}

void HIGHOMEGA::RENDER::PASSES::TestCPUGridTraceClass::Create(TriClass & PostProcessTri)
{
	mat4 triMat;
	triMat.Ident();

	testParams.time = 0.0f;
	new (&testParamsBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &testParams, sizeof(testParams));
	new (&gridInfoBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &gridInfo, sizeof(gridInfo));

	testSubmission.Add(PostProcessTri.triModel, triMat);
	testSubmission.Create(Instance);

	testSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	testShader.Create("shaders/postprocess.vert.spv", "main", "shaders/testCPUGrid.frag.spv", "main");
	testShader.AddResource(ShaderResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 1, testParamsBuf));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 2, gridInfoBuf));
	testSubmission.SetShader("default", testShader);
	testSubmission.requestCPUGridGeom(*mainGridSubmission);
}

void HIGHOMEGA::RENDER::PASSES::TestCPUGridTraceClass::Render()
{
	testParams.time += 0.1f;
	testParamsBuf.Map(0, sizeof(testParams));
	testParamsBuf.UploadSubData(0, &testParams, sizeof(testParams));

	gridInfo.mapMinVoxelSize[0] = mainGridSubmission->getGridMin().x;
	gridInfo.mapMinVoxelSize[1] = mainGridSubmission->getGridMin().y;
	gridInfo.mapMinVoxelSize[2] = mainGridSubmission->getGridMin().z;
	gridInfo.mapMinVoxelSize[3] = mainGridSubmission->getVoxelDim();
	gridInfo.mapMaxReserved[0] = mainGridSubmission->getGridMax().x;
	gridInfo.mapMaxReserved[1] = mainGridSubmission->getGridMax().y;
	gridInfo.mapMaxReserved[2] = mainGridSubmission->getGridMax().z;
	gridInfo.invMapDimReserved[0] = 1.0f / (mainGridSubmission->getGridMax().x - mainGridSubmission->getGridMin().x);
	gridInfo.invMapDimReserved[1] = 1.0f / (mainGridSubmission->getGridMax().y - mainGridSubmission->getGridMin().y);
	gridInfo.invMapDimReserved[2] = 1.0f / (mainGridSubmission->getGridMax().z - mainGridSubmission->getGridMin().z);
	gridInfo.gridSizeReserved[0] = ceil((mainGridSubmission->getGridMax().x - mainGridSubmission->getGridMin().x) / mainGridSubmission->getVoxelDim());
	gridInfo.gridSizeReserved[1] = ceil((mainGridSubmission->getGridMax().y - mainGridSubmission->getGridMin().y) / mainGridSubmission->getVoxelDim());
	gridInfo.gridSizeReserved[2] = ceil((mainGridSubmission->getGridMax().z - mainGridSubmission->getGridMin().z) / mainGridSubmission->getVoxelDim());
	gridInfoBuf.Map(0, sizeof(gridInfo));
	gridInfoBuf.UploadSubData(0, &gridInfo, sizeof(gridInfo));

	if (!HIGHOMEGA::EVENTS::windowMinimized) testSubmission.Render();
}

void HIGHOMEGA::RENDER::PASSES::TestRTXPrimaryClass::Create()
{
	pathTraceOutput.CreateImageStore(Instance, R8G8B8A8UN, (unsigned int)ScreenSize.width, (unsigned int)ScreenSize.height, 1, _2D, false);

	rtxShaderState.CreateRTX("shaders/rtxprimaryhittest.rgen.spv", "main", "shaders/rtxprimaryhittest.rchit.spv", "main", "shaders/rtxprimaryhittest.rmiss.spv", "main", "shaders/rtxalphakey.rahit.spv", "main");
	tracelet.Create(Instance);
}

void HIGHOMEGA::RENDER::PASSES::TestRTXPrimaryClass::Render()
{
	RTXScene & rtxSceneRef = mainRTXSubmission->rtxScene;
	if (rtxSceneRef.IsSceneNew())
	{
		tracelet.Destroy();
		tracingResources.clear();
		tracingResources.push_back(ShaderResource(RESOURCE_RTX_ACCEL_STRUCT, RTX_RAYGEN, 0, 0, rtxSceneRef));
		tracingResources.push_back(ShaderResource(RESOURCE_IMAGE_STORE, RTX_RAYGEN, 0, 1, pathTraceOutput));
	}

	tracelet.Submit(pathTraceOutput.getWidth(), pathTraceOutput.getHeight(), 1, tracingResources, rtxShaderState);
}

void HIGHOMEGA::RENDER::PASSES::TestRTXClass::Create(TriClass &PostProcessTri, GatherPassClass & GatherPass, SkyDomeClass &SkyDome)
{
	triRef = &PostProcessTri;
	gatherPassRef = &GatherPass;
	skyDomeRef = &SkyDome;

	pathTraceOutput.CreateImageStore(Instance, R8G8B8A8UN, (unsigned int)ScreenSize.width, (unsigned int)ScreenSize.height, 1, _2D, false);

	rtxShaderState.CreateRTX("shaders/rtxtest.rgen.spv", "main", "shaders/rtxtest.rchit.spv", "main", "shaders/rtxtest.rmiss.spv", "main", "shaders/rtxalphakey.rahit.spv", "main");
	tracelet.Create(Instance);
}

void HIGHOMEGA::RENDER::PASSES::TestRTXClass::Render()
{
	RTXScene & rtxSceneRef = mainRTXSubmission->rtxScene;
	if (rtxSceneRef.IsSceneNew())
	{
		tracelet.Destroy();
		tracingResources.clear();
		tracingResources.push_back(ShaderResource(RESOURCE_RTX_ACCEL_STRUCT, RTX_RAYGEN, 0, 0, rtxSceneRef));
		tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 1, gatherPassRef->materialAttach));
		tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 2, gatherPassRef->worldPosAttach));
		tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 3, gatherPassRef->normalAttach));
		tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RAYGEN, 0, 4, skyDomeRef->backDrop));
		tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_MISS, 0, 5, skyDomeRef->fullCubeMap));
		tracingResources.push_back(ShaderResource(RESOURCE_UBO, RTX_RAYGEN, 0, 6, skyDomeRef->rayleighMieBuf));
		tracingResources.push_back(ShaderResource(RESOURCE_UBO, RTX_RAYGEN, 0, 7, triRef->triFrustum.Buffer));
		tracingResources.push_back(ShaderResource(RESOURCE_IMAGE_STORE, RTX_RAYGEN, 0, 8, pathTraceOutput));
		std::vector <ShaderResource> variableCountInstanceBuffer, variableCountCompleteVertexBuffer, variableCountSamplers;
		for (int i = 0; i != rtxSceneRef.allTraceItems.size(); i++)
			variableCountInstanceBuffer.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT | RTX_ANYHIT, 1, 0, rtxSceneRef.instancePropertiesBuffers[i]));
		for (std::pair <const unsigned long long, TraceItem> & curTraceItemKV : rtxSceneRef.allTraceItems)
			variableCountCompleteVertexBuffer.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT | RTX_ANYHIT, 2, 0, curTraceItemKV.second.geomRef->completeVertexBuffer));
		for (std::pair <const unsigned long long, TraceItem> & curTraceItemKV : rtxSceneRef.allTraceItems)
			for (ImageClass * curImage : curTraceItemKV.second.material)
				variableCountSamplers.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RCHIT | RTX_ANYHIT, 3, 0, *curImage));
		tracingResources.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT | RTX_ANYHIT, 1, 0, variableCountInstanceBuffer));
		tracingResources.push_back(ShaderResource(RESOURCE_SSBO, RTX_RCHIT | RTX_ANYHIT, 2, 0, variableCountCompleteVertexBuffer));
		tracingResources.push_back(ShaderResource(RESOURCE_SAMPLER, RTX_RCHIT | RTX_ANYHIT, 3, 0, variableCountSamplers));
	}

	tracelet.Submit(pathTraceOutput.getWidth(), pathTraceOutput.getHeight(), 1, tracingResources, rtxShaderState);
}

void HIGHOMEGA::RENDER::PASSES::TestScreenClass::Create(TriClass & PostProcessTri, VoxelizerClass & Voxelizer, TestRTXClass & TestRTXPrimary)
{
	mat4 triMat;
	triMat.Ident();

	memset(onScreenText, (int)(' '), 60 * 5);
	onScreenTextImage.CreateTexture(Instance, 60, 5, onScreenText, 1u, false, false, false, false, 1);
	fontMap.CreateTexture(Instance, "assets/common/fontmap.tga");

	testParams.time = 0.0f;
	new (&testParamsBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &testParams, sizeof(testParams));

	testSubmission.Add(PostProcessTri.triModel, triMat);
	testSubmission.Create(Instance);

	testSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	testShader.Create("shaders/postprocess.vert.spv", "main", "shaders/testScreen.frag.spv", "main");
	testShader.AddResource(ShaderResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer));
	testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 1, testParamsBuf));
	/*testShader.AddResource(ShaderResource(RESOURCE_UBO, FRAGMENT, 0, 2, Voxelizer.gridInfoBuf));
	testShader.AddResource(ShaderResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, Voxelizer.voxelizedScene));*/
	testShader.AddResource(ShaderResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, TestRTXPrimary.pathTraceOutput));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, onScreenTextImage));
	testShader.AddResource(ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, fontMap));

	testSubmission.SetShader("default", testShader);
}

void HIGHOMEGA::RENDER::PASSES::TestScreenClass::Render()
{
	sprintf_s((char *)onScreenText, 300, "Rendered using RTX on RTX 2080Ti");
	sprintf_s((char *)(&onScreenText[60]), 300, "Resolution: %dx%d", ScreenSize.width, ScreenSize.height);
	sprintf_s((char *)(&onScreenText[60 * 2]), 300, "Mem usage: %uMBs (scratch + supplied geom)", INSTRUMENTATION::rtxBuffTotal / (1024 * 1024));
	if (INSTRUMENTATION::totalFrameTimeCount > 0) sprintf_s((char *)(&onScreenText[60 * 3]), 300, "Frame cost  min: %.2fms  max: %.2fms  avg: %.2fms", INSTRUMENTATION::minFrameCost * 1000.0f, INSTRUMENTATION::maxFrameCost * 1000.0f, (INSTRUMENTATION::totalFrameCost / (float)INSTRUMENTATION::totalFrameTimeCount) * 1000.0f);

	onScreenTextImage.ReuploadData(60, 5, onScreenText);
	testParams.time += 0.1f;
	testParamsBuf.Map(0, sizeof(testParams));
	testParamsBuf.UploadSubData(0, &testParams, sizeof(testParams));

	if (!HIGHOMEGA::EVENTS::windowMinimized) testSubmission.Render();
}