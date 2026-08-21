class PathTraceClass : public OffScreenPassClass, public RTPass, public BlueNoiseHolderClass
{
private:
	bool builtHWRTSubmission = false, builtSoftRTSubmission = false;
	bool prevFrustumEverSet = false;
	TriClass* postProcessTriRef;
	GroupedSDFBVHSubmission* sdfBVHSubmissionRef;
	GatherResolveClass* GatherPassRef;
	SkyDomeClass* SkyDomeRef;
	GroupedTraceSubmission* TraceRef;
	vec3 viewerCached;
	ShadowMapClass* shadowMapNearRef, * shadowMapFarRef;
	ShaderResourceSet rtShaderResourceSetGloss;
	std::vector<ShaderResource> tracingResourcesGloss;
	RTTracelet traceletGloss;

public:
	struct PathTraceParamsStruct
	{
		float radiosityMapCenterMotionFactor[4];
		float time;
		unsigned int diffuseMVPTurnGlossTrailTurn;
		float rawLight;
	} PathTraceParams;
	BufferClass PathTraceParamsBuf;
	ImageClass diffuseTraceOutput, glossTraceOutput;
	FrustumClass prevFrustum;

	GroupedRasterSubmission glossSubmission;
	ShaderResourceSet glossTraceShader;
	FramebufferClass glossTraceFrameBuffer;

	void Create(TriClass& PostProcessTri, GatherResolveClass& GatherPass, SkyDomeClass& SkyDome, ShadowMapClass& ShadowMapNear, ShadowMapClass& ShadowMapFar, GroupedTraceSubmission& rtSubmission, GroupedSDFBVHSubmission* sdfBVHSubmission, const vec3& initialViewer);
	void Render(const vec3& currentViewer);
};

...

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Create(TriClass& PostProcessTri, GatherResolveClass& GatherPass, SkyDomeClass& SkyDome, ShadowMapClass& ShadowMapNear, ShadowMapClass& ShadowMapFar, GroupedTraceSubmission& rtSubmission, GroupedSDFBVHSubmission* sdfBVHSubmission, const vec3& initialViewer)
{
	postProcessTriRef = &PostProcessTri;
	sdfBVHSubmissionRef = sdfBVHSubmission;
	GatherPassRef = &GatherPass;
	TraceRef = &rtSubmission;
	shadowMapNearRef = &ShadowMapNear;
	shadowMapFarRef = &ShadowMapFar;
	SkyDomeRef = &SkyDome;

	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);
	glossSubmission.Add(*PostProcessTri.triModelInstance);
	glossSubmission.Create(Instance);

	unsigned int traceWidth = 320;
	unsigned int traceHeight = 240;
	// (3 floats for pos, 1 for normal, 4*5 fp16s for hemicubic irradiance) * 2 for current and past frames (pingponged)
	diffuseTraceOutput.CreateImageStore(Instance, R32UI, traceWidth * 28, traceHeight, 1, _2D, false);
	glossTraceOutput.CreateImageStore(Instance, R16G16B16A16F, traceWidth, traceHeight, 1, _2D, false);
	PathTraceParams.radiosityMapCenterMotionFactor[0] = initialViewer.x;
	PathTraceParams.radiosityMapCenterMotionFactor[1] = initialViewer.y;
	PathTraceParams.radiosityMapCenterMotionFactor[2] = initialViewer.z;
	PathTraceParams.radiosityMapCenterMotionFactor[3] = 0.0f;
	PathTraceParams.time = 0.0f;
	PathTraceParams.diffuseMVPTurnGlossTrailTurn = 0;
	PathTraceParams.rawLight = 0.0f;
	PathTraceParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &PathTraceParams, (unsigned int)sizeof(PathTraceParams));
	std::vector<ImageClass*> clearDiffTrace = { &diffuseTraceOutput };
	diffuseTraceOutput.ClearColors(clearDiffTrace, ImageClearColor(0u, 0u, 0u, 0u));
}

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Render(const vec3& currentViewer)
{
	if (!prevFrustumEverSet)
	{
		prevFrustum.CopyFromFrustum(MainFrustum);
		prevFrustum.Update();
		prevFrustumEverSet = true;
	}
	if (RTInstance::Enabled())
	{
		if (!builtHWRTSubmission)
		{
			rtShaderResourceSet.CreateRT("shaders/rtdiffusetrace.rgen.spv", "main", "shaders/rtchit.rchit.spv", "main", "shaders/rtmiss.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
			rtShaderResourceSet.SetStageSpecializationData(RT_RAYGEN, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			tracelet.Make(Instance);
			rtShaderResourceSetGloss.CreateRT("shaders/rtglosstrace.rgen.spv", "main", "shaders/rtchit.rchit.spv", "main", "shaders/rtmiss.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
			rtShaderResourceSetGloss.SetStageSpecializationData(RT_RAYGEN, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			traceletGloss.Make(Instance);
			builtHWRTSubmission = true;
		}
		RTScene& rtSceneRef = TraceRef->rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = TraceRef->SceneID();
		if (curSceneId != lastSceneId)
		{
			lastSceneId = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources.clear();
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, GatherPassRef->materialAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 2, GatherPassRef->worldPosAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 3, GatherPassRef->normalAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 4, MainFrustum.Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 5, prevFrustum.Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 6, PathTraceParamsBuf);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 7, diffuseTraceOutput);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 8, SkyDomeRef->fullCubeMap);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 9, shadowMapNearRef->getDSAttach());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 10, shadowMapNearRef->getColorAttach());
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 11, shadowMapNearRef->getFrustum().Buffer);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 12, shadowMapFarRef->getDSAttach());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 13, shadowMapFarRef->getColorAttach());
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 14, shadowMapFarRef->getFrustum().Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 15, SkyDomeRef->rayleighMieBuf);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.lock_shared();
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);

			tracingResourcesGloss.clear();
			tracingResourcesGloss.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, GatherPassRef->materialAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 2, GatherPassRef->worldPosAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 3, GatherPassRef->normalAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 4, glossTraceOutput);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 5, MainFrustum.Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 6, PathTraceParamsBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 7, SkyDomeRef->fullCubeMap);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 8, shadowMapNearRef->getDSAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 9, shadowMapNearRef->getColorAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 10, shadowMapNearRef->getFrustum().Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 11, shadowMapFarRef->getDSAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 12, shadowMapFarRef->getColorAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 13, shadowMapFarRef->getFrustum().Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 14, SkyDomeRef->rayleighMieBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 15, *blueNoise);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.lock_shared();
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
		}
		tracelet.MakeAsync().Submit(glossTraceOutput.getWidth(), glossTraceOutput.getHeight(), 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
		traceletGloss.MakeAsync().Submit(glossTraceOutput.getWidth(), glossTraceOutput.getHeight(), 1, tracingResourcesGloss, rewriteDescriptoSets, rtShaderResourceSetGloss);
	}
	else
	{
		if (!builtSoftRTSubmission)
		{
			frameBuffer.setWidth(glossTraceOutput.getWidth());
			frameBuffer.setHeight(glossTraceOutput.getHeight());
			frameBuffer.Create(OFF_SCREEN, Instance, Window);
			submission.SetFrameBuffer(frameBuffer);

			glossTraceFrameBuffer.setWidth(glossTraceOutput.getWidth());
			glossTraceFrameBuffer.setHeight(glossTraceOutput.getHeight());
			glossTraceFrameBuffer.Create(OFF_SCREEN, Instance, Window);
			glossSubmission.SetFrameBuffer(glossTraceFrameBuffer);

			shader.Create("shaders/postprocess.vert.spv", "main", "shaders/SDFBVHdiffusetrace.frag.spv", "main");
			shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, postProcessTriRef->triFrustum.Buffer);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPassRef->materialAttach);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPassRef->worldPosAttach);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherPassRef->normalAttach);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, MainFrustum.Buffer);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, PathTraceParamsBuf);
			//shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, radiosityMapResources);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, SkyDomeRef->fullCubeMap);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, shadowMapNearRef->getDSAttach());
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, shadowMapNearRef->getColorAttach());
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 10, shadowMapNearRef->getFrustum().Buffer);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, shadowMapFarRef->getDSAttach());
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 12, shadowMapFarRef->getColorAttach());
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 13, shadowMapFarRef->getFrustum().Buffer);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 14, SkyDomeRef->rayleighMieBuf);
			//shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 15, influenceBuf);
			shader.SetStageSpecializationData(FRAGMENT, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			submission.SetShader("default", shader);
			submission.requestSDFBVH(*sdfBVHSubmissionRef);

			glossTraceShader.Create("shaders/postprocess.vert.spv", "main", "shaders/SDFBVHglosstrace.frag.spv", "main");
			glossTraceShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, postProcessTriRef->triFrustum.Buffer);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPassRef->materialAttach);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPassRef->worldPosAttach);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherPassRef->normalAttach);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 4, glossTraceOutput);
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, MainFrustum.Buffer);
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 6, PathTraceParamsBuf);
			//glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, radiosityMapResources);
			//glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 8, radiosityMapResources);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, SkyDomeRef->fullCubeMap);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 10, shadowMapNearRef->getDSAttach());
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, shadowMapNearRef->getColorAttach());
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 12, shadowMapNearRef->getFrustum().Buffer);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 13, shadowMapFarRef->getDSAttach());
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 14, shadowMapFarRef->getColorAttach());
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 15, shadowMapFarRef->getFrustum().Buffer);
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 16, SkyDomeRef->rayleighMieBuf);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 17, *blueNoise);
			glossTraceShader.SetStageSpecializationData(FRAGMENT, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			glossSubmission.SetShader("default", glossTraceShader);
			glossSubmission.requestSDFBVH(*sdfBVHSubmissionRef);

			builtSoftRTSubmission = true;
		}
		submission.MakeAsync().Render();
		glossSubmission.MakeAsync().Render();
	}
}

...

void HIGHOMEGA::RENDER::PASSES::TemporalAccumulateClass::Render()
{
	unsigned int curGlossTrailTurn = (pathTraceRef->PathTraceParams.diffuseMVPTurnGlossTrailTurn & 0x0000000F);
	unsigned int curDiffuseMVPTurn = ((pathTraceRef->PathTraceParams.diffuseMVPTurnGlossTrailTurn & 0x00000010) >> 4);
	static vec3 lastEye = vec3(0.0f), lastLook = vec3(0.0f);

	memcpy(MVPs.matrix[curGlossTrailTurn], MainFrustum.uboData.modelViewProj, sizeof(MainFrustum.uboData.modelViewProj));
	MVPsBuf.UploadSubData(curGlossTrailTurn * sizeof(MainFrustum.uboData.modelViewProj), &MVPs.matrix[curGlossTrailTurn], sizeof(MainFrustum.uboData.modelViewProj));

	float eyeDiffFactor = min(0.005f / max((MainFrustum.eye - lastEye).length(), 0.00001f), 1.0f);
	float lookDiffFactor = min(0.1f / max((MainFrustum.look - lastLook).length(), 0.00001f), 1.0f);
	pathTraceRef->PathTraceParams.radiosityMapCenterMotionFactor[3] = eyeDiffFactor * lookDiffFactor;
	pathTraceRef->PathTraceParamsBuf.UploadSubData(0, &pathTraceRef->PathTraceParams, sizeof(pathTraceRef->PathTraceParams));

	submission.MakeAsync().Render();

	curGlossTrailTurn = (curGlossTrailTurn + 1) % HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT;
	curDiffuseMVPTurn = curDiffuseMVPTurn ? 0 : 1;
	pathTraceRef->PathTraceParams.diffuseMVPTurnGlossTrailTurn = ((curDiffuseMVPTurn << 4) | curGlossTrailTurn);

	lastEye = MainFrustum.eye;
	lastLook = MainFrustum.look;
}

...

void HIGHOMEGA::RENDER::PASSES::SpatialDenoiseClass::Render()
{
	pathTraceRef->PathTraceParams.rawLight = GetStateOfAction(CMD_SWITCH_TO_PT_MODE) ? 1.0f : 0.0f;

	pathTraceRef->PathTraceParams.time += 0.01f;

	pathTraceRef->PathTraceParamsBuf.UploadSubData(0, &pathTraceRef->PathTraceParams, sizeof(pathTraceRef->PathTraceParams));

	pathTraceRef->prevFrustum.CopyFromFrustum(MainFrustum);
	pathTraceRef->prevFrustum.Update();

	submissionH.MakeAsync().Render();
	submissionV.MakeAsync().Render();
}