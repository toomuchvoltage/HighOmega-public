struct GridRef
{
	unsigned int offsetSizeReserved[4];
};
class GroupedCPUGridSubmission : public GroupedRenderSubmission
{
private:
	struct GridPiece
	{
		std::vector<GridTriangle> tris;
	};
	struct GridObject
	{
		std::unordered_map<MeshMaterial, std::list<GridPiece>, MeshMaterialHash> transformedObject;
		vec3 objMin, objMax;
		bool justAdded, dirty, remove;
	};
	std::unordered_map <unsigned long long, GridObject> gridObjects;
	struct GridCell
	{
		std::vector<GridObject *> overlappingCellObjects;
		std::vector<GridTriangle> tris;
		bool dirty;
	};

	bool redoBindings;
	BufferClass gridTriBuf;
	BufferClass gridRefBuf;

	vec3 sceneViewMax, sceneViewMin;
	unsigned int gridX, gridY, gridZ;

	std::vector <GridCell> grid;

public:
	GroupedCPUGridSubmission();
	void PotentiallyRedosubmission();
	SubmittedRenderItem Add(GraphicsModel & inpModel, mat4 &inpMat4, std::function<bool(MeshMaterial & curMat)> inpFilterFunction = [](MeshMaterial & curMat) -> bool {
		if (curMat.postProcess) return false;
	});
	void Update(SubmittedRenderItem & providedRenderItem, mat4 & inpMat);
	void UpdateBulk(SubmittedRenderItem & providedRenderItem, float *inpMat, float *inpMatIT);
	void UpdateBulkPrepare();
	void UpdateBulkClose();
	void Remove(SubmittedRenderItem inpSubmittedRenderItem);
	void Refresh();
	bool SceneHasChanged();
	void MarkSceneOld();
	vec3 getGridMin();
	vec3 getGridMax();
	float getVoxelDim();
	BufferClass & getGridTriBuf();
	BufferClass & getGridRefBuf();
};

class GroupedRasterSubmission : public GroupedRenderSubmission
{
private:
	....
	GroupedCPUGridSubmission *cachedCPUGridSource;
	....
	
	void requestCPUGridGeom(GroupedCPUGridSubmission & gridSource);
}