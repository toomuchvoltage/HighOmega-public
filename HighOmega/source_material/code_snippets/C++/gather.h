class GatherPassClass : public OffScreenPassClass, public GatherPassClassCommon
{
private:
	ShaderResourceSet shaderTess;

public:
	ImageClass materialAttach;

	void Create(SkyDomeClass & SkyDome, WorldParamsClass & WorldParams);
	void Render(OcclusionGeom & occlusionGeom);
};