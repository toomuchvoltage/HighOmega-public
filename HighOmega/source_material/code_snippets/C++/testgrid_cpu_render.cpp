GroupedCPUGridSubmission *GridPass::mainGridSubmission = nullptr;

....


HIGHOMEGA::RENDER::GroupedCPUGridSubmission::GroupedCPUGridSubmission()
{
	gridX = gridY = gridZ = 0;
	redoBindings = false;
}

float HIGHOMEGA::RENDER::GroupedCPUGridSubmission::getVoxelDim()
{
	return 3.0f;
}

BufferClass & HIGHOMEGA::RENDER::GroupedCPUGridSubmission::getGridTriBuf()
{
	return gridTriBuf;
}

BufferClass & HIGHOMEGA::RENDER::GroupedCPUGridSubmission::getGridRefBuf()
{
	return gridRefBuf;
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::PotentiallyRedosubmission()
{
	for (std::unordered_map<unsigned long long, GridObject>::iterator it = gridObjects.begin(); it != gridObjects.end(); ++it)
		if (!it->second.justAdded && !it->second.remove) it->second.dirty = true;
}

SubmittedRenderItem HIGHOMEGA::RENDER::GroupedCPUGridSubmission::Add(GraphicsModel & inpModel, mat4 & inpMat4, std::function<bool(MeshMaterial&curMat)> inpFilterFunction)
{
	SubmittedRenderItem retVal;
	retVal.item = &inpModel;

	retVal.itemId = marseneTwister64BitPRNG();
	retVal.filterFunction = inpFilterFunction;

	UnpackMat4(inpMat4, retVal.trans);
	UnpackMat4(inpMat4.Inv().Transpose(), retVal.transIT);

	allSubmittedItems[retVal.itemId] = retVal;

	gridObjects[retVal.itemId] = GridObject();
	gridObjects[retVal.itemId].justAdded = true;
	gridObjects[retVal.itemId].dirty = false;
	gridObjects[retVal.itemId].remove = false;

	return retVal;
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::Update(SubmittedRenderItem & providedRenderItem, mat4 & inpMat)
{
	SubmittedRenderItem & curItem = allSubmittedItems[providedRenderItem.itemId];
	float saveCurTrans[16];
	memcpy(saveCurTrans, curItem.trans, sizeof(saveCurTrans));
	UnpackMat4(inpMat, curItem.trans);
	if (!memcmp(saveCurTrans, curItem.trans, sizeof(saveCurTrans))) return;
	UnpackMat4(inpMat.Inv().Transpose(), curItem.transIT);

	if (!gridObjects[curItem.itemId].justAdded && !gridObjects[curItem.itemId].remove) gridObjects[curItem.itemId].dirty = true;
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::UpdateBulk(SubmittedRenderItem & providedRenderItem, float * inpMat, float * inpMatIT)
{
	SubmittedRenderItem & curItem = allSubmittedItems[providedRenderItem.itemId];

	if (!memcmp(curItem.trans, inpMat, sizeof(curItem.trans))) return;

	memcpy(curItem.trans, inpMat, sizeof(curItem.trans));
	memcpy(curItem.transIT, inpMatIT, sizeof(curItem.transIT));

	if (!gridObjects[curItem.itemId].justAdded && !gridObjects[curItem.itemId].remove) gridObjects[curItem.itemId].dirty = true;
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::UpdateBulkPrepare()
{
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::UpdateBulkClose()
{
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::Remove(SubmittedRenderItem inpSubmittedRenderItem)
{
	allSubmittedItems.erase(inpSubmittedRenderItem.itemId);
	if (!gridObjects[inpSubmittedRenderItem.itemId].justAdded) gridObjects[inpSubmittedRenderItem.itemId].remove = true;
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::Refresh()
{
	if (gridX == 0 && gridY == 0 && gridZ == 0)
	{
		bool firstMinMax = false;
		for (std::unordered_map<unsigned long long, SubmittedRenderItem>::iterator it = allSubmittedItems.begin(); it != allSubmittedItems.end(); ++it)
		{
			vec3 curViewMin = it->second.item->getViewMin();
			vec3 curViewMax = it->second.item->getViewMax();
			if (!firstMinMax)
			{
				sceneViewMin = curViewMin;
				sceneViewMax = curViewMax;
				firstMinMax = true;
			}
			else
			{
				sceneViewMin = vec3(min(curViewMin.x, sceneViewMin.x), min(curViewMin.x, sceneViewMin.y), min(curViewMin.z, sceneViewMin.z));
				sceneViewMax = vec3(max(curViewMax.x, sceneViewMax.x), max(curViewMax.x, sceneViewMax.y), max(curViewMax.z, sceneViewMax.z));
			}
		}

		gridX = (unsigned int)ceil((sceneViewMax.x - sceneViewMin.x) / getVoxelDim());
		gridY = (unsigned int)ceil((sceneViewMax.y - sceneViewMin.y) / getVoxelDim());
		gridZ = (unsigned int)ceil((sceneViewMax.z - sceneViewMin.z) / getVoxelDim());

		grid.resize(gridX * gridY * gridZ);

		for (unsigned int i = 0;i != gridX;i++)
			for (unsigned int j = 0; j != gridY; j++)
				for (unsigned int k = 0; k != gridZ; k++)
					grid[k * gridX * gridY + j * gridX + i].dirty = true;
	}

	mat4 identMat;
	identMat.Ident();
	for (std::unordered_map<unsigned long long, GridObject>::iterator it = gridObjects.begin(); it != gridObjects.end(); ++it)
	{
		if (it->second.justAdded || it->second.dirty)
		{
			if (it->second.dirty)
			{
				vec3 curObjectMin = it->second.objMin;
				vec3 curObjectMax = it->second.objMax;

				unsigned int curGridMinLocX = (unsigned int)floor((curObjectMin.x - sceneViewMin.x) / getVoxelDim());
				unsigned int curGridMinLocY = (unsigned int)floor((curObjectMin.y - sceneViewMin.y) / getVoxelDim());
				unsigned int curGridMinLocZ = (unsigned int)floor((curObjectMin.z - sceneViewMin.z) / getVoxelDim());

				unsigned int curGridMaxLocX = (unsigned int)floor((curObjectMax.x - sceneViewMin.x) / getVoxelDim());
				unsigned int curGridMaxLocY = (unsigned int)floor((curObjectMax.y - sceneViewMin.y) / getVoxelDim());
				unsigned int curGridMaxLocZ = (unsigned int)floor((curObjectMax.z - sceneViewMin.z) / getVoxelDim());

				for (unsigned int i = curGridMinLocX; i != curGridMaxLocX + 1; i++)
					for (unsigned int j = curGridMinLocY; j != curGridMaxLocY + 1; j++)
						for (unsigned int k = curGridMinLocZ; k != curGridMaxLocZ + 1; k++)
						{
							GridCell & curCell = grid[k * gridX * gridY + j * gridX + i];
							std::vector<GridObject *>::iterator it2 = std::find(curCell.overlappingCellObjects.begin(), curCell.overlappingCellObjects.end(), &it->second);
							if (it2 != curCell.overlappingCellObjects.end())
								curCell.overlappingCellObjects.erase(it2);
							curCell.dirty = true;
						}
			}

			it->second.justAdded = it->second.dirty = false;
			GraphicsModel & refModel = *allSubmittedItems[it->first].item;
			mat4 transMat, transITMat;
			PackMat4(allSubmittedItems[it->first].trans, transMat);
			PackMat4(allSubmittedItems[it->first].transIT, transITMat);
			it->second.transformedObject.clear();
			bool isTransIdent = (transMat == identMat);
			bool minMaxSet = false;
			vec3 objectMin;
			vec3 objectMax;

			for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it2 = refModel.MaterialGeomMap.begin(); it2 != refModel.MaterialGeomMap.end(); ++it2)
				for (std::list<GeometryClass>::iterator it3 = it2->second.begin(); it3 != it2->second.end(); it3++)
				{
					std::vector<RasterVertex> & cachedGeom = it3->getCachedGeom();
					it->second.transformedObject[it2->first].push_back(GridPiece());
					GridPiece & curPiece = it->second.transformedObject[it2->first].back();
					curPiece.tris.reserve(cachedGeom.size() / 3);
					vec3 e1, e2, e3, norm;
					for (unsigned int i = 0; i != cachedGeom.size() / 3; i++)
					{
						unsigned int i_3 = i * 3;
						RasterVertex rv1 = cachedGeom[i_3];
						RasterVertex rv2 = cachedGeom[i_3 + 1];
						RasterVertex rv3 = cachedGeom[i_3 + 2];
						e1 = vec3(rv1.pos[0], rv1.pos[1], rv1.pos[2]);
						e2 = vec3(rv2.pos[0], rv2.pos[1], rv2.pos[2]);
						e3 = vec3(rv3.pos[0], rv3.pos[1], rv3.pos[2]);
						norm = vec3(rv1.fnorm[0], rv1.fnorm[1], rv1.fnorm[2]);

						if (!isTransIdent)
						{
							e1 = transMat * e1;
							e2 = transMat * e2;
							e3 = transMat * e3;
							norm = (transITMat * norm).normalized();
						}

						GridTriangle curTri;
						curTri.normv2[0] = norm.x;
						curTri.normv2[1] = norm.y;
						curTri.normv2[2] = norm.z;
						curTri.e1u1[0] = e1.x;
						curTri.e1u1[1] = e1.y;
						curTri.e1u1[2] = e1.z;
						curTri.e2v1[0] = e2.x;
						curTri.e2v1[1] = e2.y;
						curTri.e2v1[2] = e2.z;
						curTri.e3u2[0] = e3.x;
						curTri.e3u2[1] = e3.y;
						curTri.e3u2[2] = e3.z;
						vec3 triMin = vec3(Min(e1.x, e2.x, e3.x), Min(e1.y, e2.y, e3.y), Min(e1.z, e2.z, e3.z));
						vec3 triMax = vec3(Max(e1.x, e2.x, e3.x), Max(e1.y, e2.y, e3.y), Max(e1.z, e2.z, e3.z));
						if (!minMaxSet)
						{
							objectMin = triMin;
							objectMax = triMax;
							minMaxSet = true;
						}
						else
						{
							objectMin = vec3(min(triMin.x, objectMin.x), min(triMin.y, objectMin.y), min(triMin.z, objectMin.z));
							objectMax = vec3(max(triMax.x, objectMax.x), max(triMax.y, objectMax.y), max(triMax.z, objectMax.z));
						}
						curPiece.tris.push_back(curTri);
					}
				}

			it->second.objMin = objectMin;
			it->second.objMax = objectMax;

			unsigned int gridMinLocX = (unsigned int)floor((objectMin.x - sceneViewMin.x) / getVoxelDim());
			unsigned int gridMinLocY = (unsigned int)floor((objectMin.y - sceneViewMin.y) / getVoxelDim());
			unsigned int gridMinLocZ = (unsigned int)floor((objectMin.z - sceneViewMin.z) / getVoxelDim());

			unsigned int gridMaxLocX = (unsigned int)floor((objectMax.x - sceneViewMin.x) / getVoxelDim());
			unsigned int gridMaxLocY = (unsigned int)floor((objectMax.y - sceneViewMin.y) / getVoxelDim());
			unsigned int gridMaxLocZ = (unsigned int)floor((objectMax.z - sceneViewMin.z) / getVoxelDim());

			for (unsigned int i = gridMinLocX; i != gridMaxLocX + 1; i++)
				for (unsigned int j = gridMinLocY; j != gridMaxLocY + 1; j++)
					for (unsigned int k = gridMinLocZ; k != gridMaxLocZ + 1; k++)
					{
						GridCell & curCell = grid[k * gridX * gridY + j * gridX + i];
						if (std::find(curCell.overlappingCellObjects.begin(), curCell.overlappingCellObjects.end(), &it->second) == curCell.overlappingCellObjects.end())
							curCell.overlappingCellObjects.push_back(&it->second);
						curCell.dirty = true;
					}
		}
	}

	redoBindings = false;

	float gridCellRadius = sqrt(0.75f) * getVoxelDim();

	for (unsigned int i = 0; i != gridX; i++)
	{
		for (unsigned int j = 0; j != gridY; j++)
		{
			for (unsigned int k = 0; k != gridZ; k++)
			{
				GridCell & curCell = grid[k * gridX * gridY + j * gridX + i];
				if (curCell.dirty)
				{
					vec3 cellCenter = vec3((float)i + 0.5f, (float)j + 0.5f, (float)k + 0.5f) * getVoxelDim() + sceneViewMin;
					curCell.tris.clear();
					for (GridObject * curObj : curCell.overlappingCellObjects)
					{
						for (std::unordered_map<MeshMaterial, std::list<GridPiece>>::iterator it = curObj->transformedObject.begin(); it != curObj->transformedObject.end(); ++it)
						{
							for (std::list<GridPiece>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
							{
								for (GridTriangle & curTri : it2->tris)
								{
									if (abs(vec3(curTri.normv2[0], curTri.normv2[1], curTri.normv2[2]) * (cellCenter - vec3(curTri.e1u1[0], curTri.e1u1[1], curTri.e1u1[2]))) < gridCellRadius)
									{
										curCell.tris.push_back(curTri);
										if (curCell.tris.size() > 30) break;
									}
									if (curCell.tris.size() > 30) break;
								}
								if (curCell.tris.size() > 30) break;
							}
							if (curCell.tris.size() > 30) break;
						}
						if (curCell.tris.size() > 30) break;
					}
					curCell.dirty = false;
					redoBindings = true;
				}
			}
		}
	}

	if (redoBindings)
	{
		unsigned int gridOffset = 0;

		unsigned int totalTriCount = 0;

		std::vector <GridRef> gridReference;
		gridReference.resize(gridX * gridY * gridZ);

		std::vector <GridTriangle> allSceneTris;
		for (unsigned int i = 0; i != gridX; i++)
			for (unsigned int j = 0; j != gridY; j++)
				for (unsigned int k = 0; k != gridZ; k++)
					totalTriCount += (unsigned int)grid[k * gridX * gridY + j * gridX + i].tris.size();

		allSceneTris.reserve(totalTriCount);

		for (unsigned int i = 0; i != gridX; i++)
			for (unsigned int j = 0; j != gridY; j++)
				for (unsigned int k = 0; k != gridZ; k++)
				{
					unsigned int cellIndex = k * gridX * gridY + j * gridX + i;
					for (GridTriangle & curTri : grid[cellIndex].tris)
						allSceneTris.push_back(curTri);
					gridReference[cellIndex].offsetSizeReserved[0] = gridOffset;
					gridReference[cellIndex].offsetSizeReserved[1] = (unsigned int)grid[cellIndex].tris.size();
					gridOffset += (unsigned int)grid[cellIndex].tris.size();
				}

		if (gridTriBuf.getSize() > 0) gridTriBuf.~BufferClass();
		if (gridRefBuf.getSize() > 0) gridRefBuf.~BufferClass();

		new (&gridTriBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, allSceneTris.data(), (unsigned int)allSceneTris.size() * sizeof(GridTriangle));
		new (&gridRefBuf) BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, gridReference.data(), (unsigned int)gridReference.size() * sizeof(GridRef));
	}
}

bool HIGHOMEGA::RENDER::GroupedCPUGridSubmission::SceneHasChanged()
{
	return redoBindings;
}

void HIGHOMEGA::RENDER::GroupedCPUGridSubmission::MarkSceneOld()
{
	// We don't really need to...
}

vec3 HIGHOMEGA::RENDER::GroupedCPUGridSubmission::getGridMin()
{
	return sceneViewMin;
}

vec3 HIGHOMEGA::RENDER::GroupedCPUGridSubmission::getGridMax()
{
	return sceneViewMax;
}

...

class GridPass
{
public:
	struct
	{
		float mapMinVoxelSize[4];
		float mapMaxReserved[4];
	} gridInfo;
	BufferClass gridInfoBuf;

	static GroupedCPUGridSubmission *mainGridSubmission;

	static void FrameStart();
	static void FrameEnd();

	GridPass();
	static void DestroyGrid();
};

...


void HIGHOMEGA::RENDER::GroupedRasterSubmission::DestroySubmissionData()
{
	....

	if (cachedCPUGridSource && sourceGridBindingsChanged)
	{
		sourceGridBindingsChanged = false;
		clearRasterPipelineCache();
	}
}

HIGHOMEGA::RENDER::GroupedRasterSubmission::GroupedRasterSubmission()
{
	....
	cachedCPUGridSource = nullptr;
}

...

void HIGHOMEGA::RENDER::GroupedRasterSubmission::requestCPUGridGeom(GroupedCPUGridSubmission & gridSource)
{
	cachedCPUGridSource = &gridSource;
}

...

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Render()
{
	...

	if (cachedCPUGridSource && cachedCPUGridSource->SceneHasChanged())
	{
		sourceGridBindingsChanged = true;
		redoSubmissionData = true;
	}

	...
	
				--> if (cachedCPUGridSource)
				--> {
				--> 	shaderResources.push_back(ShaderResource(RESOURCE_SSBO, FRAGMENT, 2, 0, cachedCPUGridSource->getGridTriBuf()));
				--> 	shaderResources.push_back(ShaderResource(RESOURCE_SSBO, FRAGMENT, 3, 0, cachedCPUGridSource->getGridRefBuf()));
				-->	}
				std::string selShaderName = curMat.shaderName;
				if (shaders.find(curMat.shaderName) == shaders.end()) {
					selShaderName = "default";
				}
				shaderResources.insert(shaderResources.end(), shaders[selShaderName]->getAdditionalResources().begin(), shaders[selShaderName]->getAdditionalResources().end());
				rasterPipelineCache[curMatGeomPairing.mat] = new RasterPipelineStateClass(Instance, pFlags, *frameBuffer, *(curMatGeomPairing.Geom.begin()->first), shaderResources, shaders[selShaderName]);
				
	....
}


void HIGHOMEGA::RENDER::PASSES::GridPass::FrameStart()
{
	mainGridSubmission->Refresh();
}

void HIGHOMEGA::RENDER::PASSES::GridPass::FrameEnd()
{
	mainGridSubmission->MarkSceneOld();
}

HIGHOMEGA::RENDER::PASSES::GridPass::GridPass()
{
	if (!mainGridSubmission) mainGridSubmission = new GroupedCPUGridSubmission();
}

void HIGHOMEGA::RENDER::PASSES::GridPass::DestroyGrid()
{
	if (mainGridSubmission) delete mainGridSubmission;

	mainGridSubmission = nullptr;
}
