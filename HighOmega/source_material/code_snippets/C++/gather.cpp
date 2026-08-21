
void HIGHOMEGA::RENDER::PASSES::GatherPassClass::Create(SkyDomeClass & SkyDome, WorldParamsClass & WorldParams)
{
	submission.SetClearColor(vec3(0.0f, 0.0f, 0.0f), 0.0f);
	PipelineFlags gatherPS;
	gatherPS.backFaceCulling = true;
	submission.SetDefaultPipelineFlags(gatherPS);
	submission.Create(Instance);

	materialAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false);
	worldPosAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false);
	normalAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false);
	frameBuffer.AddColorAttachment(materialAttach);
	frameBuffer.AddColorAttachment(worldPosAttach);
	frameBuffer.AddColorAttachment(normalAttach);
	frameBuffer.SetDepthStencil(SkyDome.backdropDS.depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.SetFrameBuffer(frameBuffer);
	submission.makeAsync();

	shader.Create("shaders/gather.vert.spv", "main", "shaders/gather.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, MainFrustum.Buffer);
	submission.SetShader("default", shader);

	shaderTess.Create("shaders/gather.vert.spv", "main", "shaders/gather.tesc.spv", "main", "shaders/gather.tese.spv", "main", "shaders/gather.frag.spv", "main");
	shaderTess.AddResource(RESOURCE_UBO, VERTEX | TESS_CTRL | TESS_EVAL, 0, 0, MainFrustum.Buffer);
	shaderTess.AddResource(RESOURCE_UBO, TESS_EVAL, 0, 1, WorldParams.renderTimeBuffer);
	submission.SetShader("shaderTess", shaderTess);
}

void HIGHOMEGA::RENDER::PASSES::GatherPassClass::Render(OcclusionGeom & occlusionGeom)
{
	submission.doCulling(MainFrustum, occlusionGeom);
	submission.Render();
}
