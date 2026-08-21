
#define HIGHOMEGA_GENERATE_HINTS_PER_LUMINARE 20

struct Bone
{

...

#define HIGHOMEGA_MAX_NEE_HINTS 200

class NEEDataHolderClass
{
public:
	struct NEEHintStruct
	{
		float posArea[4];
		float norm[4];
	};
	struct AllNEEDataStruct
	{
		NEEHintStruct hint[HIGHOMEGA_MAX_NEE_HINTS];
		unsigned int hintCountReserved[4];
	} AllNEEData;
	BufferClass AllNEEDataBuf;
};

class BlueNoiseHolderClass
{

....

class PhysicalItemClass
{
....
public:
....
	void GenerateAllNEEHints(NEEDataHolderClass & NEEDataHolder);
};

....

void HIGHOMEGA::WORLD::PhysicalItemClass::GenerateAllNEEHints(NEEDataHolderClass & NEEDataHolder)
{
	unsigned int hintCounter = 0;
	for (std::pair<const unsigned long long, RigidBodyItem> & curItem : allItems)
	{
		RigidBodyItem & curRigidItem = curItem.second;
		if (!curRigidItem.modelRef) continue;
		for (std::pair<const std::string, GraphicsModel::NEEHintData> curHintDataPair : curRigidItem.modelRef->allNEEHintData) {
			std::vector<GraphicsModel::NEEHint> & curHintVector = curHintDataPair.second.hints;
			for (GraphicsModel::NEEHint & curHint : curHintVector) {
				NEEDataHolder.AllNEEData.hint[hintCounter].posArea[0] = curHint.pos.x;
				NEEDataHolder.AllNEEData.hint[hintCounter].posArea[1] = curHint.pos.y;
				NEEDataHolder.AllNEEData.hint[hintCounter].posArea[2] = curHint.pos.z;
				NEEDataHolder.AllNEEData.hint[hintCounter].posArea[3] = curHintDataPair.second.area;
				NEEDataHolder.AllNEEData.hint[hintCounter].norm[0] = curHint.norm.x;
				NEEDataHolder.AllNEEData.hint[hintCounter].norm[1] = curHint.norm.y;
				NEEDataHolder.AllNEEData.hint[hintCounter].norm[2] = curHint.norm.z;
				hintCounter++;
				if (hintCounter == HIGHOMEGA_MAX_NEE_HINTS) break;
			}
			if (hintCounter == HIGHOMEGA_MAX_NEE_HINTS) break;
		}
		if (hintCounter == HIGHOMEGA_MAX_NEE_HINTS) break;
	}
	NEEDataHolder.AllNEEData.hintCountReserved[0] = hintCounter;

	if (NEEDataHolder.AllNEEDataBuf.getSize() == 0)
		NEEDataHolder.AllNEEDataBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &NEEDataHolder.AllNEEData, (unsigned int)sizeof(NEEDataHolder.AllNEEData));
	else
		NEEDataHolder.AllNEEDataBuf.UploadSubData(0, &NEEDataHolder.AllNEEData, sizeof(NEEDataHolder.AllNEEData));
}

....

class GraphicsModel
{
public:
	struct NEEHint
	{
		vec3 pos, norm;
	};
	struct NEEHintData
	{
		std::vector<NEEHint> hints;
		float area;
	};
	std::unordered_map <std::string, NEEHintData> allNEEHintData;
	
	...

private:
	void GenerateNEEHints(MeshMaterial & meshMat, std::string groupId, std::vector <RasterVertex> & renderVertexVector);
}

...

void HIGHOMEGA::RENDER::GraphicsModel::GenerateNEEHints(MeshMaterial & meshMat, std::string groupId, std::vector<RasterVertex>& renderVertexVector)
{
	if (meshMat.emissivity == 0.0f) return;

	allNEEHintData[groupId].hints.clear();
	for (int i = 0; i != HIGHOMEGA_GENERATE_HINTS_PER_LUMINARE; i++)
	{
		unsigned int primPick = (unsigned int)(((float)rand() / (float)RAND_MAX) * (renderVertexVector.size() / 3));

		unsigned int e1Index = primPick * 3;
		unsigned int e2Index = e1Index + 1;
		unsigned int e3Index = e1Index + 2;

		vec3 e1, e2, e3, vnorm1, vnorm2, vnorm3, col1, col2, col3;
		vec2 uv1, uv2, uv3;
		unpackRasterVertex(e1, col1, uv1, vnorm1, renderVertexVector[e1Index]);
		unpackRasterVertex(e2, col2, uv2, vnorm2, renderVertexVector[e2Index]);
		unpackRasterVertex(e3, col3, uv3, vnorm3, renderVertexVector[e3Index]);
		vec3 norm = cross(e3 - e2, e1 - e2).normalized();
		if (vnorm1 * norm < 0.0f) norm = -norm;

		vec3 e2_e1 = e2 - e1;
		vec3 e3_e2 = e3 - e2;
		vec3 e1_e3 = e1 - e3;
		vec3 side1 = cross(e2_e1, norm);
		if (side1 * e3_e2 > 0.0f) side1 = -side1;
		vec3 side2 = cross(e3_e2, norm);
		if (side2 * e1_e3 > 0.0f) side2 = -side2;
		vec3 side3 = cross(e1_e3, norm);
		if (side3 * e2_e1 > 0.0f) side3 = -side3;

		vec3 randPoint = e2;

		if ( cross (e2_e1, e3_e2).length() * 0.5f > 0.1f )
			while (true)
			{
				float u = ((float)rand()) / ((float)RAND_MAX);
				float v = ((float)rand()) / ((float)RAND_MAX);

				randPoint = u * (-e2_e1) + v * (e3_e2)+e2;

				if ((randPoint - e1) * side1 < 0.0f && (randPoint - e2) * side2 < 0.0f && (randPoint - e3) * side3 < 0.0f) break;
			}

		NEEHint curHint;
		curHint.pos = randPoint;
		curHint.norm = norm;
		allNEEHintData[groupId].hints.push_back(curHint);
	}

	allNEEHintData[groupId].area = 0.0f;
	for (int i = 0; i != renderVertexVector.size() / 3; i++) {
		unsigned int e1Index = i * 3;
		unsigned int e2Index = e1Index + 1;
		unsigned int e3Index = e1Index + 2;

		vec3 e1, e2, e3, vnorm1, vnorm2, vnorm3, col1, col2, col3;
		vec2 uv1, uv2, uv3;
		unpackRasterVertex(e1, col1, uv1, vnorm1, renderVertexVector[e1Index]);
		unpackRasterVertex(e2, col2, uv2, vnorm2, renderVertexVector[e2Index]);
		unpackRasterVertex(e3, col3, uv3, vnorm3, renderVertexVector[e3Index]);

		allNEEHintData[groupId].area += cross(e3 - e2, e1 - e2).length();
	}
	allNEEHintData[groupId].area *= 0.5f;
}

...

void HIGHOMEGA::RENDER::GraphicsModel::Model(std::string & newGroupId, MeshMaterial & origMaterial, std::vector<TriUV>& triList, bool gpuResideOnly, bool inpImmutable)
{

...

	GenerateNEEHints(origMaterial, newGroupId, renderVertexVector);

	isInit = true;
}

...

void HIGHOMEGA::RENDER::GraphicsModel::Model(HIGHOMEGA::MESH::Mesh & inpMesh, std::string belong, InstanceClass &ptrToInstance, std::function<bool(int, DataGroup &)> inpFilterFunction, vec3 *inpNewCenter, bool gpuResideOnly, bool inpImmutable, bool loadAnimationData)
{
	if (isInit) RemovePast();

	...

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
	
		....

		GenerateNEEHints(mat, newGroupId, verts);

		verts.clear();
		vertAnimData.clear();
	}

	isInit = true;
}

...

void HIGHOMEGA::RENDER::GraphicsModel::ChangeGeom(std::string & groupId, std::vector<TriUV> & triList)
{
...

	GenerateNEEHints(materialById, groupId, renderVertexVector);
}

...

void HIGHOMEGA::RENDER::GraphicsModel::UpdateGeom(std::string & groupId, std::vector<TriUV>& triList)
{
...

	GenerateNEEHints(materialById, groupId, renderVertexVector);
}