
class DownSampleClass : public OffScreenPassClass, public GatherPassClassCommon
{
public:
	ImageClass materialAttach;

	void Create(TriClass& PostProcessTri, SkyDomeClass& SkyDome, GatherResolveClass& GatherPass);
	void Render();
};