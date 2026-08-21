class DFClass : public DoubleComputeClass
{
private:
	std::vector <ImageClass *> dfImageVector;
	std::vector <ImageClass *> dynamicDFImageVector;

public:
	struct
	{
		float gridDims[4];
		unsigned int startGridCoord[4];
		unsigned int endGridCoord[4];
	} computeParams;
	BufferClass computeParamsBuf;
	ImageClass dfImage, dynamicDFImage;

	void Create(GPUGridBuilderClass & SceneVoxelizer, GPUGridBuilderClass & Voxelizer);
	void UpdateDynamic(std::vector <range> & allDynamicRanges, GPUGridBuilderClass & SceneVoxelizer);
	void UpdateBase(GPUGridBuilderClass & SceneVoxelizer);
};

class PreIlluminateClass : public SingleComputeClass, public RTXPass
{
private:
	std::vector <ImageClass *> preIllumVector;
	GPUGridBuilderClass *voxelizerRef;
	SkyDomeClass *skyDomeRef;
	ShadowMapClass *shadowMapRef;

public:

	ImageClass preIlluminateImage;

	void Create(GPUGridBuilderClass & Voxelizer, DFClass & DF, SkyDomeClass & SkyDome, ShadowMapClass & ShadowMap);
	void ClearColors();
	void Submit();
};