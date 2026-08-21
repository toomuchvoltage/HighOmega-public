class TestBVHTraceClass
{
private:
	struct
	{
		float time;
		float reserved[3];
	} testParams;
	BufferClass testParamsBuf;
	BufferClass triBuf, nodeBuf;

public:
	GroupedRasterSubmission testSubmission;
	ShaderState testShader;

	void Create(TriClass & PostProcessTri, void *tris, void *nodes, unsigned int triSize, unsigned int nodeSize);
	void Render();
};
class TestGPUGridTraceClass
{
public:
	unsigned char onScreenText[300];
	ImageClass onScreenTextImage, fontMap;

	GPUGridBuilderClass *refGridBuilder;
	GroupedRasterSubmission testSubmission;
	ShaderState testShader;

	void Create(TriClass & PostProcessTri, GPUGridBuilderClass &GPUGridBuilder, GPUGridBuilderClass &dynamicGPUGridBuilder, GatherPassClass & GatherPass, SkyDomeClass & SkyDome);
	void Render();
};
class TestGPUGridPrimaryTraceClass
{
private:
	struct
	{
		float time;
		float reserved[3];
	} testParams;
	BufferClass testParamsBuf;

public:
	GroupedRasterSubmission testSubmission;
	ShaderState testShader;

	void Create(TriClass & PostProcessTri, GPUGridBuilderClass &GPUGridBuilder, GPUGridBuilderClass &dynamicGPUGridBuilder, GatherPassClass & GatherPass, SkyDomeClass & SkyDome);
	void Render();
};
class TestRadeonRaysTraceClass
{
private:
	ImageClass rrOutputImage;

public:
	GroupedRasterSubmission testSubmission;
	ShaderState testShader;

	void Create(TriClass & PostProcessTri, unsigned int rrWidth, unsigned int rrHeight, unsigned char * rrOutput);
	void Render();
};
class TestCPUGridTraceClass : public GridPass
{
private:
	struct
	{
		float time;
		float reserved[3];
	} testParams;
	BufferClass testParamsBuf;

	struct
	{
		float mapMinVoxelSize[4];
		float mapMaxReserved[4];
		float invMapDimReserved[4];
		float gridSizeReserved[4];
	} gridInfo;
	BufferClass gridInfoBuf;

public:
	GroupedRasterSubmission testSubmission;
	ShaderState testShader;

	void Create(TriClass & PostProcessTri);
	void Render();
};
class TestRTXPrimaryClass : public OffScreenPassClass, public RTXPass
{
public:
	ImageClass pathTraceOutput;

	void Create();
	void Render();
};
class TestRTXClass : public OffScreenPassClass, public RTXPass
{
private:
	TriClass * triRef;
	GatherPassClass *gatherPassRef;
	SkyDomeClass * skyDomeRef;

public:
	ImageClass pathTraceOutput;

	void Create(TriClass &PostProcessTri, GatherPassClass & GatherPass, SkyDomeClass &SkyDome);
	void Render();
};
class TestScreenClass
{
private:
	struct
	{
		float time;
		float reserved[3];
	} testParams;
	BufferClass testParamsBuf;

public:
	unsigned char onScreenText[300];
	ImageClass onScreenTextImage, fontMap;

	GroupedRasterSubmission testSubmission;
	ShaderState testShader;

	void Create(TriClass & PostProcessTri, VoxelizerClass & Voxelizer, TestRTXClass & TestRTXPrimary);
	void Render();
};