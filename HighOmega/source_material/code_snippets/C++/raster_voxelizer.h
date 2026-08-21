class VoxelizerClass : public OffScreenPassClass
{
private:
	VoxelizerClass* buildOnRef = nullptr;
	std::vector <ImageClass*> voxelizedSceneImages;

	unsigned int voxelizedX, voxelizedY, voxelizedZ;
	float voxelDim;
	unsigned int voxelGridMemUsage;
	unsigned int irradianceCacheMemUsage;

public:
	ImageClass voxelizedScene;
	struct
	{
		float mapMinVoxelSize[4];
		float mapMaxDynamicFlag[4];
		float invMapDims[4];
		float gridSize[4];
	} voxelizedInfo;
	BufferClass voxelizedInfoBuf;
	ImageClass radiosityMaps[6];

	unsigned int getVoxelizedX();
	unsigned int getVoxelizedY();
	unsigned int getVoxelizedZ();
	unsigned int getVoxelGridMemUsage();
	unsigned int getIrradianceCacheMemUsage();
	float VoxelDim();
	void Create(vec3 viewMin, vec3 viewMax, unsigned int voxelCoarseness, VoxelizerClass* UseVoxelizer = nullptr);
	void Render();
	void ClearScene();
};