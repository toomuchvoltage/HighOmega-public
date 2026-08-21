class PathTraceClass : public OffScreenPassClass, public RTPass, public NEEDataHolderClass, public BlueNoiseHolderClass
{
private:
	TriClass* triRef;
	DownSampleClass* downSampleRef;
	SkyDomeClass* skyDomeRef;
	ShadowMapClass* shadowMapRef;
	VoxelizerClass* staticVoxelizerRef, * dynamicVoxelizerRef;

	bool bvhBuilderRun = false;

public:
	struct PathTraceParamsStruct
	{
		float motionFactorDiffuseOnlyReserved[4];
		float timeTurnDirectionRawLight[4];
	} PathTraceParams;
	BufferClass PathTraceParamsBuf;
	ImageClass glossTraceOutput;

	void Create(TriClass& PostProcessTri, DownSampleClass& DownSample, VoxelizerClass& SceneVoxelizer, VoxelizerClass& Voxelizer, SkyDomeClass& SkyDome, ShadowMapClass& ShadowMap);
	void Render(GroupedTraceSubmission& rtSubmission);
};