
unsigned int HIGHOMEGA::RENDER::PASSES::VoxelizerClass::getVoxelizedX()
{
	return voxelizedX;
}

unsigned int HIGHOMEGA::RENDER::PASSES::VoxelizerClass::getVoxelizedY()
{
	return voxelizedY;
}

unsigned int HIGHOMEGA::RENDER::PASSES::VoxelizerClass::getVoxelizedZ()
{
	return voxelizedZ;
}

unsigned int HIGHOMEGA::RENDER::PASSES::VoxelizerClass::getVoxelGridMemUsage()
{
	return voxelGridMemUsage;
}

unsigned int HIGHOMEGA::RENDER::PASSES::VoxelizerClass::getIrradianceCacheMemUsage()
{
	return irradianceCacheMemUsage;
}

float HIGHOMEGA::RENDER::PASSES::VoxelizerClass::VoxelDim()
{
	return voxelDim;
}

void HIGHOMEGA::RENDER::PASSES::VoxelizerClass::Create(vec3 viewMin, vec3 viewMax, unsigned int voxelCoarseness, VoxelizerClass* UseVoxelizer)
{
	buildOnRef = UseVoxelizer;

	voxelDim = (float)voxelCoarseness;

	submission.Create(Instance);

	frameBuffer.setWidth(64);
	frameBuffer.setHeight(64);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	vec3 mapDims;

	voxelizedInfo.mapMinVoxelSize[0] = viewMin.x - voxelDim; // Make it slightly conservative...
	voxelizedInfo.mapMinVoxelSize[1] = viewMin.y - voxelDim;
	voxelizedInfo.mapMinVoxelSize[2] = viewMin.z - voxelDim;
	voxelizedInfo.mapMinVoxelSize[3] = voxelDim;
	voxelizedInfo.mapMaxDynamicFlag[0] = viewMax.x + voxelDim;
	voxelizedInfo.mapMaxDynamicFlag[1] = viewMax.y + voxelDim;
	voxelizedInfo.mapMaxDynamicFlag[2] = viewMax.z + voxelDim;
	voxelizedInfo.mapMaxDynamicFlag[3] = buildOnRef ? 1.0f : 0.0f;
	mapDims = vec3(voxelizedInfo.mapMaxDynamicFlag[0] - voxelizedInfo.mapMinVoxelSize[0], voxelizedInfo.mapMaxDynamicFlag[1] - voxelizedInfo.mapMinVoxelSize[1], voxelizedInfo.mapMaxDynamicFlag[2] - voxelizedInfo.mapMinVoxelSize[2]);
	voxelizedInfo.invMapDims[0] = 1.0f / mapDims.x;
	voxelizedInfo.invMapDims[1] = 1.0f / mapDims.y;
	voxelizedInfo.invMapDims[2] = 1.0f / mapDims.z;
	voxelizedX = (unsigned int)ceil(mapDims.x / voxelDim);
	voxelizedY = (unsigned int)ceil(mapDims.y / voxelDim);
	voxelizedZ = (unsigned int)ceil(mapDims.z / voxelDim);
	voxelizedInfo.gridSize[0] = (float)voxelizedX;
	voxelizedInfo.gridSize[1] = (float)voxelizedY;
	voxelizedInfo.gridSize[2] = (float)voxelizedZ;

	voxelizedInfoBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &voxelizedInfo, (unsigned int)sizeof(voxelizedInfo));

	if (!buildOnRef)
	{
		std::string gridInfoLog;

		voxelizedScene.CreateImageStore(Instance, R8G8B8A8UN, voxelizedX * 4, voxelizedY, voxelizedZ, _3D, false);
		voxelizedSceneImages.push_back(&voxelizedScene);

		voxelGridMemUsage = (voxelizedX * voxelizedY * voxelizedZ * 4 * sizeof(unsigned int)) / (1024 * 1024);

		gridInfoLog = "New voxel grid. Cell size: ";
		gridInfoLog += std::to_string(voxelDim);
		gridInfoLog += "^3.0 grid dim: ";
		gridInfoLog += std::to_string(voxelizedX);
		gridInfoLog += ",";
		gridInfoLog += std::to_string(voxelizedY);
		gridInfoLog += ",";
		gridInfoLog += std::to_string(voxelizedZ);
		gridInfoLog += ": ";
		gridInfoLog += std::to_string(voxelGridMemUsage);
		gridInfoLog += "MBs";
		LOG() << gridInfoLog;

		for (int i = 0; i != 6; i++)
		{
			radiosityMaps[i].CreateImageStore(Instance, R16G16B16A16F, voxelizedX, voxelizedY, voxelizedZ, _3D, true);
			voxelizedSceneImages.push_back(&radiosityMaps[i]);
		}

		irradianceCacheMemUsage = (voxelizedX * voxelizedY * voxelizedZ * 6 * sizeof(unsigned int) * 2) / (1024 * 1024);

		gridInfoLog = "Irradiance cache (6 RGBA16Fs): ";
		gridInfoLog += std::to_string(irradianceCacheMemUsage);
		gridInfoLog += "MBs";
		LOG() << gridInfoLog;
	}

	submission.SetFrameBuffer(frameBuffer);
	shader.Create("shaders/raster_voxelizer.vert.spv", "main", "shaders/raster_voxelizer.frag.spv", "main");
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 0, buildOnRef ? buildOnRef->voxelizedScene : voxelizedScene);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, voxelizedInfoBuf);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::VoxelizerClass::Render()
{
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::VoxelizerClass::ClearScene()
{
	voxelizedScene.ClearColors(voxelizedSceneImages, ImageClearColor(vec3(0.0f), 0.0f));
}