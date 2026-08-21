
void HIGHOMEGA::RENDER::PASSES::DownSampleClass::Create(TriClass& PostProcessTri, SkyDomeClass& SkyDome, GatherResolveClass& GatherPass)
{
	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	materialAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, 640, 480, false, false);
	worldPosAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, 640, 480, false, false);
	normalAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, 640, 480, false, false);

	frameBuffer.AddColorAttachment(materialAttach);
	frameBuffer.AddColorAttachment(worldPosAttach);
	frameBuffer.AddColorAttachment(normalAttach);
	frameBuffer.SetDepthStencil(SkyDome.backdropDS.depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.SetFrameBuffer(frameBuffer);

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/downsample.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.materialAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherPass.normalAttach);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::DownSampleClass::Render()
{
	submission.Render();
}