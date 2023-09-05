/*
	Copyright (c) 2023 TooMuchVoltage Software Inc.

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

#include "render.h"

using namespace HIGHOMEGA;
using namespace HIGHOMEGA::RENDER;
using namespace HIGHOMEGA::RENDER::PASSES;
using namespace HIGHOMEGA::GL;
using namespace HIGHOMEGA::MESH;
using namespace HIGHOMEGA::EVENTS;
using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;

FrustumClass HIGHOMEGA::RENDER::MainFrustum;
ScreenSizeClass HIGHOMEGA::RENDER::ScreenSize;
std::unordered_map <std::string, HIGHOMEGA::CacheItem<ImageClass>> HIGHOMEGA::RENDER::TextureCache;
ImageClass *BlueNoiseHolderClass::blueNoise = nullptr;
unsigned int BlueNoiseHolderClass::blueNoiseClaims = 0;

extern HIGHOMEGA::INSTRUMENTATION::Instrument updateInstrument, traceInstrument, frameInstrument;
namespace HIGHOMEGA::GL
{
	std::mutex texture_mutex;
}

CacheItem<ImageClass> * HIGHOMEGA::RENDER::AddOrFindCachedTexture(std::string & belong, std::string & texName, InstanceClass & ptrToInstance, bool isArray, int nLayers, bool mipmap, bool useSRGB)
{
	std::lock_guard<std::mutex> lk(texture_mutex);
	if (TextureCache.find(texName) == TextureCache.end())
	{
		bool loadResult;

		if (isArray)
			loadResult = TextureCache[texName].elem.CreateTexture(ptrToInstance, belong, texName, nLayers, false, true, false, mipmap, useSRGB ? R8G8B8A8SRGB : R8G8B8A8UN);
		else
			loadResult = TextureCache[texName].elem.CreateTexture(ptrToInstance, belong, texName, 1U, false, false, false, mipmap, useSRGB ? R8G8B8A8SRGB : R8G8B8A8UN);
		if (!loadResult)
		{
			TextureCache.erase(texName);
			return nullptr;
		}
		else
			TextureCache[texName].elemCount = 1;
	}
	else
		TextureCache[texName].elemCount++;
	return &TextureCache[texName];
}

void HIGHOMEGA::RENDER::TransformMesh(Mesh & mesh, mat4 & inpTransform)
{
	vec3 pos, col, dir, vNorm;
	vec2 uv;
	mat4 inpTransformDT = inpTransform.DirectionTransform();
	for (int i = 0; i != mesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = mesh.DataGroups[i];

		HIGHOMEGA::MESH::DataBlock *triBlock = nullptr;
		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock))
		{
			HIGHOMEGA::MESH::DataBlock *descBlock = nullptr;
			if (Mesh::getDataBlock(curPolyGroup, "DESCRIPTION", &descBlock))
			{
				if (Mesh::getDataRowVec3(*descBlock, "pos", pos))
				{
					pos = inpTransform * pos;
					if (!Mesh::setDataRowVec3(*descBlock, "pos", pos)) throw std::runtime_error("Failed to transform desc. pos during mesh transform");
				}
				if (Mesh::getDataRowVec3(*descBlock, "dir", dir))
				{
					dir = inpTransformDT * dir;
					if (!Mesh::setDataRowVec3(*descBlock, "dir", dir)) throw std::runtime_error("Failed to transform desc. dir during mesh transform");
				}
			}
			continue;
		}

		if (triBlock->mode != "blob") throw std::runtime_error("Mesh's tri block is not a blob");

		RasterVertex *rasterVerts = (RasterVertex *)triBlock->blob.data();
		unsigned int numVerts = (unsigned int)triBlock->blob.size() / sizeof(RasterVertex);

		for (int j = 0; j != numVerts; j++)
		{
			unpackRasterVertex(pos, col, uv, vNorm, rasterVerts[j]);
			pos = inpTransform * pos;
			vNorm = inpTransformDT * vNorm;
			packRasterVertex(pos, col, uv, vNorm, rasterVerts[j]);
		}
	}
}

void HIGHOMEGA::RENDER::InitGraphicsSubSystem(bool requestHWRT, bool windowed, bool headless)
{
	std::string window_title;
	window_title = std::string("HighOmega " + std::to_string(HIGHOMEGA_VERSION));
	if (!headless) Window.Make(window_title.c_str(), 100, 100, ScreenSize.width, ScreenSize.height, windowed);
	Instance.Make(false, Window, requestHWRT, headless);
	if (!headless) Instance.CreateSwapChain(Window);
}

bool HIGHOMEGA::RENDER::MeshMaterial::operator==(const MeshMaterial & other) const
{
	return (diffName == other.diffName) &&
		(nrmName == other.nrmName) &&
		(rghName == other.rghName) &&
		(hgtName == other.hgtName) &&
		(spcName == other.spcName) &&
		(shaderName == other.shaderName) &&
		(pipelineFlags == other.pipelineFlags) &&
		(smooth == other.smooth) &&
		(mipmap == other.mipmap) &&
		(isAlphaKeyed == other.isAlphaKeyed) &&
		(postProcess == other.postProcess) &&
		(backDropGlass == other.backDropGlass) &&
		(scattering == other.scattering) &&
		(playerId == other.playerId) &&
		(rayMask == other.rayMask);
}

void HIGHOMEGA::RENDER::MeshMaterial::BumpClaims()
{
	std::lock_guard<std::mutex> lk(texture_mutex);
	if (diffRef) diffRef->elemCount++;
	if (nrmRef) nrmRef->elemCount++;
	if (rghRef) rghRef->elemCount++;
	if (hgtRef) hgtRef->elemCount++;
	if (spcRef) spcRef->elemCount++;
}

void HIGHOMEGA::RENDER::MeshMaterial::ReduceClaims()
{
	std::string matStringKey = diffName + nrmName + spcName + rghName + hgtName;
	std::lock_guard<std::mutex> lk(texture_mutex);
	bool removedASampler = false;
	if (diffRef)
	{
		diffRef->elemCount--;
		if (!diffRef->elemCount)
		{
			TextureCache.erase(diffName);
			diffRef = nullptr;
			diffName = "";
			removedASampler = true;
		}
	}
	if (nrmRef)
	{
		nrmRef->elemCount--;
		if (!nrmRef->elemCount)
		{
			TextureCache.erase(nrmName);
			nrmRef = nullptr;
			nrmName = "";
			removedASampler = true;
		}
	}
	if (rghRef)
	{
		rghRef->elemCount--;
		if (!rghRef->elemCount)
		{
			TextureCache.erase(rghName);
			rghRef = nullptr;
			rghName = "";
			removedASampler = true;
		}
	}
	if (hgtRef)
	{
		hgtRef->elemCount--;
		if (!hgtRef->elemCount)
		{
			TextureCache.erase(hgtName);
			hgtRef = nullptr;
			hgtName = "";
			removedASampler = true;
		}
	}
	if (spcRef)
	{
		spcRef->elemCount--;
		if (!spcRef->elemCount)
		{
			TextureCache.erase(spcName);
			spcRef = nullptr;
			spcName = "";
			removedASampler = true;
		}
	}
	if (removedASampler)
	{
		std::lock_guard<std::mutex> lk2(GroupedRasterSubmission::globalDSCache_mutex);
		for (std::pair <GroupedRasterSubmission* const, std::unordered_map <std::string, std::unordered_map <std::string, DescriptorSets *>>> & curSubToAllPSODS : GroupedRasterSubmission::globalDSCache)
		{
			if (curSubToAllPSODS.second.find(matStringKey) == curSubToAllPSODS.second.end()) continue;
			for (std::pair <std::string const, DescriptorSets*>& curAllPSODS : curSubToAllPSODS.second[matStringKey])
				curAllPSODS.second->SetDirty(true);
			curSubToAllPSODS.first->redoSubmissionData = true;
		}
	}
}

HIGHOMEGA::RENDER::MeshMaterial::MeshMaterial()
{
}

CacheItem<ImageClass>* HIGHOMEGA::RENDER::MeshMaterial::TryDifferentLODs(std::string& belong, std::string& texName, InstanceClass& ptrToInstance, bool isArray, int nLayers, bool mipmap, bool useSRGB, bool loadLowRes)
{
	std::string textureName = texName;

	textureName.replace(textureName.rfind(".tga"), std::string(".tga").length(), ".ktx");
	std::string preExtension = ".ktx";
	if (loadLowRes)
	{
		textureName.replace(textureName.rfind(".ktx"), std::string(".ktx").length(), ".lowq.ktx");
		preExtension = ".lowq.ktx";
	}

	CacheItem<ImageClass>* texRef = AddOrFindCachedTexture(belong, textureName, ptrToInstance, isArray, nLayers, mipmap, useSRGB);
	if (!texRef)
	{
		if (loadLowRes)
		{
			textureName = texName;
			textureName.replace(textureName.rfind(".tga"), std::string(".tga").length(), ".ktx");
			preExtension = ".ktx";
		}
		else
		{
			textureName = texName;
			preExtension = ".tga";
		}
		texRef = AddOrFindCachedTexture(belong, textureName, ptrToInstance, isArray, nLayers, mipmap, useSRGB);
		if (!texRef)
		{
			if (!loadLowRes) return nullptr;
			texRef = AddOrFindCachedTexture(belong, texName, ptrToInstance, isArray, nLayers, mipmap, useSRGB);
			if (!texRef) return nullptr;
		}
	}
	
	return texRef;
}

HIGHOMEGA::RENDER::MeshMaterial::MeshMaterial(HIGHOMEGA::MESH::DataBlock & propBlock, std::string belong, InstanceClass & ptrToInstance)
{
	if (!Mesh::getDataRowString(propBlock, "texname", diffName)) throw std::runtime_error("Bad diffName fetch for graphics model load");

	isTerrain = false;
	if (diffName.find("{terrain}") != std::string::npos) isTerrain = true;

	mipmap = true;
	if (diffName.find("{nomip}") != std::string::npos) mipmap = false;

	diffRef = TryDifferentLODs(belong, diffName, ptrToInstance, true, isTerrain ? 4 : 1, mipmap, true);
	if (!diffRef) throw std::runtime_error("Could not find albedo map");

	nrmName = diffName;
	nrmName.replace(nrmName.rfind(".tga"), std::string(".tga").length(), ".nrm.tga");
	nrmRef = TryDifferentLODs(belong, nrmName, ptrToInstance, true, isTerrain ? 3 : 1, mipmap, false);
	if (!nrmRef) nrmName = "";

	rghName = diffName;
	rghName.replace(rghName.rfind(".tga"), std::string(".tga").length(), ".rgh.tga");
	rghRef = TryDifferentLODs(belong, rghName, ptrToInstance, true, isTerrain ? 3 : 1, mipmap, false);
	if (!rghRef) rghName = "";

	hgtName = diffName;
	hgtName.replace(hgtName.rfind(".tga"), std::string(".tga").length(), ".hgt.tga");
	hgtRef = TryDifferentLODs(belong, hgtName, ptrToInstance, true, isTerrain ? 4 : 1, mipmap, false); // Height map needs this again because the albedo map stencils are not accessible
	if (!hgtRef) hgtName = "";

	spcName = diffName;
	spcName.replace(spcName.rfind(".tga"), std::string(".tga").length(), ".spc.tga");
	spcRef = TryDifferentLODs(belong, spcName, ptrToInstance, true, isTerrain ? 3 : 1, mipmap, true);
	if (!spcRef) spcName = "";

	if (!Mesh::getDataRowFloat(propBlock, "emissivity", emissivity)) emissivity = 0.0f;

	if (!Mesh::getDataRowFloat(propBlock, "refractiveIndex", refractiveIndex)) refractiveIndex = 0.0f;

	float tmpFloat;
	dielectric = Mesh::getDataRowFloat(propBlock, "dielectric", tmpFloat);

	if (!Mesh::getDataRowString(propBlock, "shaderName", shaderName)) shaderName = "default";

	if (!Mesh::getDataRowFloat(propBlock, "uOffsetRate", uvOffset.x)) uvOffset.x = 0.0f;
	if (!Mesh::getDataRowFloat(propBlock, "vOffsetRate", uvOffset.y)) uvOffset.y = 0.0f;

	if (!Mesh::getDataRowFloat(propBlock, "heightMapDisplaceFactor", heightMapDisplaceFactor)) heightMapDisplaceFactor = 0.0f;

	if (!Mesh::getDataRowFloat(propBlock, "subDivAmount", subDivAmount)) subDivAmount = 0.0f;

	if (Mesh::getDataRowFloat(propBlock, "renderOrder", tmpFloat))
		renderOrder = (int)tmpFloat;
	else
		renderOrder = 0;

	float smoothValue = 0.0f;
	bool smoothSet;
	smoothSet = Mesh::getDataRowFloat(propBlock, "smooth", smoothValue);
	if (smoothSet)
		smooth = smoothValue;

	if (Mesh::getDataRowFloat(propBlock, "depthTest", tmpFloat))
	{
		pipelineFlags.depthTest = (tmpFloat == 1.0f);
		pipelineFlags.changedDepthTest = true;
	}

	if (Mesh::getDataRowFloat(propBlock, "depthWrite", tmpFloat))
	{
		pipelineFlags.depthWrite = (tmpFloat == 1.0f);
		pipelineFlags.changedDepthWrite = true;
	}

	if (Mesh::getDataRowFloat(propBlock, "backFaceCulling", tmpFloat))
	{
		pipelineFlags.backFaceCulling = (tmpFloat == 1.0f);
		pipelineFlags.changedBackFaceCulling = true;
	}

	if (Mesh::getDataRowFloat(propBlock, "frontFaceCulling", tmpFloat))
	{
		pipelineFlags.frontFaceCulling = (tmpFloat == 1.0f);
		pipelineFlags.changedFrontFaceCulling = true;
	}

	if (Mesh::getDataRowFloat(propBlock, "frontFaceClockwise", tmpFloat))
	{
		pipelineFlags.frontFaceClockWise = (tmpFloat == 1.0f);
		pipelineFlags.changedFrontFaceClockWise = true;
	}

	if (Mesh::getDataRowFloat(propBlock, "additiveBlend", tmpFloat))
	{
		pipelineFlags.blendEnable = true;
		pipelineFlags.alphaBlending = true;
		pipelineFlags.colorBlending = true;
		pipelineFlags.srcAlphaFactor = FACTOR_ONE;
		pipelineFlags.dstAlphaFactor = FACTOR_ZERO;
		pipelineFlags.srcColorFactor = FACTOR_SRC_ALPHA;
		pipelineFlags.dstColorFactor = FACTOR_ONE;
		pipelineFlags.alphaBlendOp = BLEND_ADD;
		pipelineFlags.colorBlendOp = BLEND_ADD;

		pipelineFlags.changedBlendEnable = true;
	}

	if (Mesh::getDataRowFloat(propBlock, "alphaBlend", tmpFloat))
	{
		pipelineFlags.blendEnable = true;
		pipelineFlags.alphaBlending = true;
		pipelineFlags.colorBlending = true;
		pipelineFlags.srcAlphaFactor = FACTOR_ONE;
		pipelineFlags.dstAlphaFactor = FACTOR_ZERO;
		pipelineFlags.srcColorFactor = FACTOR_SRC_ALPHA;
		pipelineFlags.dstColorFactor = FACTOR_ONE_MINUS_SRC_ALPHA;
		pipelineFlags.alphaBlendOp = BLEND_ADD;
		pipelineFlags.colorBlendOp = BLEND_ADD;

		pipelineFlags.changedBlendEnable = true;
	}

	postProcess = Mesh::getDataRowFloat(propBlock, "isPostProcess", tmpFloat);

	isAlphaKeyed = Mesh::getDataRowFloat(propBlock, "isAlphaKeyed", tmpFloat);

	backDropGlass = Mesh::getDataRowFloat(propBlock, "isBackDropGlass", tmpFloat);

}

std::size_t HIGHOMEGA::RENDER::MeshMaterialHash::operator()(const MeshMaterial & k) const
{
	using std::size_t;
	using std::hash;
	using std::string;

	return hash<string>()(k.diffName)
		^ hash<string>()(k.nrmName)
		^ hash<string>()(k.rghName)
		^ hash<string>()(k.hgtName)
		^ hash<string>()(k.spcName)
		^ hash<string>()(k.shaderName)
		^ HIGHOMEGA::GL::PipelineFlagsHash()(k.pipelineFlags)
		^ hash<bool>()(k.smooth)
		^ hash<bool>()(k.mipmap)
		^ hash<bool>()(k.isAlphaKeyed)
		^ hash<bool>()(k.postProcess)
		^ hash<bool>()(k.backDropGlass)
		^ hash<bool>()(k.scattering)
		^ hash<unsigned int>()(k.playerId)
		^ hash<unsigned char>()(k.rayMask);
}

void HIGHOMEGA::RENDER::Pose::Inv()
{
	for (Bone & curBone : pose)
		curBone.bone = curBone.bone.Inv();
}

void HIGHOMEGA::RENDER::Pose::Mul(Pose & rhs)
{
	if (rhs.pose.size() != pose.size()) throw std::runtime_error("rhs pose should be the same size as what it is mul'd by");

	for (unsigned int i = 0; i != pose.size(); i++)
		pose[i].bone *= rhs.pose[i].bone;
}

void HIGHOMEGA::RENDER::Pose::Transform(mat4 & transMat)
{
	for (Bone & curBone : pose)
		curBone.bone = transMat * curBone.bone;
}

void HIGHOMEGA::RENDER::Animation::GetPose(float fract, Pose & retPose)
{
	fract = min (max(0.0f, fract), 1.0f);

	for (unsigned int i = 0; i != keyFrames.size() - 1; i++)
	{
		if (fract >= keyFrames[i].keyFrameFract && fract <= keyFrames[i + 1].keyFrameFract)
		{
			retPose.pose.resize(keyFrames[i].pose.size());
			retPose.keyFrameFract = fract;
			retPose.keyFrameTime = (unsigned int)(fract * (float)lastKeyFrameTime);

			float curFract = (float)(retPose.keyFrameTime - keyFrames[i].keyFrameTime) / (float)(keyFrames[i + 1].keyFrameTime - keyFrames[i].keyFrameTime);
			curFract = SmootherstepFast(curFract);

			for (unsigned int j = 0; j != keyFrames[i].pose.size(); j++)
			{
				retPose.pose[j].name = keyFrames[i].pose[j].name;
				retPose.pose[j].bone = keyFrames[i].pose[j].bone * (1.0f - curFract) + curFract * keyFrames[i + 1].pose[j].bone;
			}
			return ;
		}
	}
}

void HIGHOMEGA::RENDER::Animation::GetPose(unsigned int providedKeyFrame, Pose & retPose)
{
	if (providedKeyFrame > keyFrames[keyFrames.size() - 1].keyFrameTime) providedKeyFrame = keyFrames[keyFrames.size() - 1].keyFrameTime;

	for (unsigned int i = 0; i != keyFrames.size() - 1; i++)
	{
		if (providedKeyFrame >= keyFrames[i].keyFrameTime && providedKeyFrame <= keyFrames[i + 1].keyFrameTime)
		{
			float curFract = (float)(providedKeyFrame - keyFrames[i].keyFrameTime) / (float)(keyFrames[i + 1].keyFrameTime - keyFrames[i].keyFrameTime);
			curFract = SmootherstepFast(curFract);

			retPose.pose.resize(keyFrames[i].pose.size());
			retPose.keyFrameFract = (float)providedKeyFrame / (float)lastKeyFrameTime;
			retPose.keyFrameTime = providedKeyFrame;

			for (unsigned int j = 0; j != keyFrames[i].pose.size(); j++)
			{
				retPose.pose[j].name = keyFrames[i].pose[j].name;
				retPose.pose[j].bone = keyFrames[i].pose[j].bone * (1.0f - curFract) + curFract * keyFrames[i + 1].pose[j].bone;
			}
			return ;
		}
	}
}

void HIGHOMEGA::RENDER::AnimationAndSnapshot::CopyToPoseBuffer(Pose & inpPose)
{
	unsigned int rawPoseDataSize = (unsigned int)inpPose.pose.size() * 16 * sizeof(float);

	if (!currentPoseBuf || currentPoseBuf->getSize() != rawPoseDataSize) return;

	if (!rawPoseData) rawPoseData = new unsigned char[rawPoseDataSize];

	unsigned int dataOffset = 0;
	for (Bone & curBone : inpPose.pose)
	{
		UnpackMat4(curBone.bone, (void *)&rawPoseData[dataOffset]);
		dataOffset += sizeof(float) * 16;
	}

	currentPoseBuf->UploadSubData(0, rawPoseData, rawPoseDataSize);
}

HIGHOMEGA::RENDER::AnimationAndSnapshot::~AnimationAndSnapshot()
{
	if (currentPoseBuf) delete currentPoseBuf;
	if (rawPoseData) delete rawPoseData;

	currentPoseBuf = nullptr;
	rawPoseData = nullptr;
}

unsigned int HIGHOMEGA::RENDER::GraphicsModel::VoxelizeWorkGroupX()
{
	return 32;
}

unsigned int HIGHOMEGA::RENDER::GraphicsModel::TrianglePerThreadCount()
{
	return 1000;
}

unsigned int HIGHOMEGA::RENDER::GraphicsModel::JFAWorkGroupX()
{
	return 4;
}

unsigned int HIGHOMEGA::RENDER::GraphicsModel::JFAWorkGroupY()
{
	return 4;
}

unsigned int HIGHOMEGA::RENDER::GraphicsModel::JFAWorkGroupZ()
{
	return 4;
}

unsigned int HIGHOMEGA::RENDER::GraphicsModel::TessellateWorkGroup()
{
	return 32;
}

void HIGHOMEGA::RENDER::GraphicsModel::UpdateSDFs(std::vector<SubmittedRenderItem>& updateItems, bool forceRefresh, std::vector<std::string> cacheNames)
{
	if (RTInstance::Enabled()) return;

	std::vector<ComputeSubmission *> csSet;
	std::vector<ShaderResourceSet *> srsSet;
	std::vector<BufferClass *> allVoxelizeParamBufs;
	std::vector<ImageClass *> distFieldGenList;
	std::vector<std::string> distFieldCacheNames;

	csSet.push_back(new ComputeSubmission);

	struct
	{
		float imgMinTriCount[4];
		float imgMaxVoxelSize[4];
		InstanceProperties instProps;
	} voxelizeParams;

	struct
	{
		unsigned int offsetStage[2];
	} JFAParams;

	for (unsigned int i = 0; i != updateItems.size(); i++)
	{
		SubmittedRenderItem& curPair = updateItems[i];
		if (curPair.item->MaterialGeomMap.size() == 0)
		{
			if (curPair.item->sdf) delete curPair.item->sdf;
			curPair.item->sdf = nullptr;
			continue;
		}
		vec3 modMin, modMax;
		curPair.item->getModelMinMax(modMin, modMax);
		modMin -= vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		modMax += vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		voxelizeParams.imgMinTriCount[0] = modMin.x;
		voxelizeParams.imgMinTriCount[1] = modMin.y;
		voxelizeParams.imgMinTriCount[2] = modMin.z;
		voxelizeParams.imgMaxVoxelSize[0] = modMax.x;
		voxelizeParams.imgMaxVoxelSize[1] = modMax.y;
		voxelizeParams.imgMaxVoxelSize[2] = modMax.z;
		voxelizeParams.imgMaxVoxelSize[3] = HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;

		if (!curPair.item->sdf)
		{
			curPair.item->sdf = new ImageClass();
			curPair.item->sdf->CreateImageStore(Instance, R8G8B8A8UN, (int)ceilf((modMax.x - modMin.x) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS), (int)ceilf((modMax.y - modMin.y) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS), (int)ceilf((modMax.z - modMin.z) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS), _3D, false);
			unsigned char* cacheContent = nullptr;
			unsigned int cacheSize = 0;
			ResourceLoader::LOAD_LOCATION loadLocation;
			if (cacheNames.size() > 0 && ResourceLoader::Load("", cacheNames[i], &cacheContent, cacheSize, loadLocation) == ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS)
			{
				curPair.item->sdf->UploadData(cacheContent, cacheSize);
				curPair.item->sdf->FreeLoadedData();
				continue;
			}
		}
		else
			if (!forceRefresh) continue;
		std::vector<ImageClass *> clearList;
		clearList.push_back(curPair.item->sdf);
		distFieldGenList.push_back(curPair.item->sdf);
		if (cacheNames.size() > 0) distFieldCacheNames.push_back(cacheNames[i]);
		curPair.item->sdf->ClearColors(clearList, ImageClearColor(vec3(0.0f), 0.0f));

		for (std::pair <const MeshMaterial, std::list<GeometryClass>> & matGeomPair : curPair.item->MaterialGeomMap)
		{
			MeshMaterial curMat = matGeomPair.first;
			if (curPair.filterFunction && !curPair.filterFunction(curMat)) continue;
			for (GeometryClass & curGeom : matGeomPair.second)
			{
				unsigned int triCount = curGeom.getVertBuffer().getSize() / sizeof(RasterVertex);
				voxelizeParams.imgMinTriCount[3] = *((float*)&triCount);
				GroupedRenderSubmission::CompileInstanceProperties(voxelizeParams.instProps, curMat);
				allVoxelizeParamBufs.push_back(new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &voxelizeParams, (unsigned int)sizeof(voxelizeParams)));
				srsSet.push_back(new ShaderResourceSet);
				srsSet.back()->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, *allVoxelizeParamBufs.back());
				srsSet.back()->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, *curPair.item->sdf);
				srsSet.back()->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 2, matGeomPair.first.diffRef->elem);
				srsSet.back()->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 3, matGeomPair.first.spcRef ? matGeomPair.first.spcRef->elem : matGeomPair.first.diffRef->elem);
				srsSet.back()->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 4, matGeomPair.first.rghRef ? matGeomPair.first.rghRef->elem : matGeomPair.first.diffRef->elem);
				srsSet.back()->AddResource(RESOURCE_SSBO, COMPUTE, 0, 5, curGeom.getVertBuffer());
				srsSet.back()->Create("shaders/voxelize.comp.spv", "main");
				std::stringstream ptrStream;
				ptrStream << &curGeom;
				csSet[0]->MakeDispatch(Instance, std::string("voxelize_") + ptrStream.str(), *srsSet.back(), (unsigned int)ceil((double)triCount / (double)(VoxelizeWorkGroupX() * TrianglePerThreadCount())), 1, 1);
			}
		}
	}
	csSet[0]->Submit();

	BufferClass JFAParamsBuf;
	JFAParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(JFAParams.offsetStage));
	for (unsigned int i = 0; i != distFieldGenList.size(); i++)
	{
		ImageClass* curImg = distFieldGenList[i];
		csSet.push_back(new ComputeSubmission);
		srsSet.push_back(new ShaderResourceSet);
		ImageClass JFAImg;
		JFAImg.CreateImageStore(Instance, R32G32B32A32UI, curImg->getWidth(), curImg->getHeight(), curImg->getDepth(), _3D, false);

		srsSet.back()->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, JFAParamsBuf);
		srsSet.back()->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, JFAImg);
		srsSet.back()->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 2, *curImg);
		srsSet.back()->Create("shaders/jfa.comp.spv", "main");
		csSet.back()->MakeDispatch(Instance, std::string("JFA"), *srsSet.back(), (unsigned int)ceil((double)JFAImg.getWidth() / (double)JFAWorkGroupX()),
			(unsigned int)ceil((double)JFAImg.getHeight() / (double)JFAWorkGroupY()),
			(unsigned int)ceil((double)JFAImg.getDepth() / (double)JFAWorkGroupZ()));

		// Place seeds
		JFAParams.offsetStage[1] = 0;
		JFAParamsBuf.UploadSubData(0, JFAParams.offsetStage, sizeof(JFAParams));
		csSet.back()->Submit();

		// Do JFA iterations
		JFAParams.offsetStage[1] = 1;
		unsigned int jfaIter = 0;
		unsigned int log2Res = (unsigned int)(ceil(log2(Max((double)JFAImg.getWidth(), (double)JFAImg.getHeight(), (double)JFAImg.getDepth()))));

		while (true)
		{
			int offsetPower = log2Res - jfaIter - 1;
			if (offsetPower < 0) break;
			unsigned int jfaOffset = (unsigned int)(pow(2.0, (double)offsetPower));

			JFAParams.offsetStage[0] = jfaOffset;
			JFAParamsBuf.UploadSubData(0, JFAParams.offsetStage, sizeof(JFAParams));
			csSet.back()->Submit();
			jfaIter++;
		}

		// Compute distance field
		JFAParams.offsetStage[1] = 2;
		JFAParamsBuf.UploadSubData(0, JFAParams.offsetStage, sizeof(JFAParams));
		csSet.back()->Submit();

		if (distFieldCacheNames.size() > 0)
		{
			curImg->DownloadData();
			ResourceSaver::SaveBlob (distFieldCacheNames[i], curImg->DownloadedData(), curImg->DownloadedDataSize());
			curImg->FreeLoadedData();
		}
	}

	for (BufferClass * curBuf : allVoxelizeParamBufs)
		delete curBuf;
	for (ShaderResourceSet * curSrs : srsSet)
		delete curSrs;
	for (ComputeSubmission * curCs : csSet)
		delete curCs;
	allVoxelizeParamBufs.clear();
	srsSet.clear();
	csSet.clear();
}

void HIGHOMEGA::RENDER::GraphicsModel::RemovePast()
{
	allNEEHintData.clear();
	vertexCache.clear();
	armatures.clear();

	modelTransMat.Ident();
	modelTransMatInv.Ident();
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
	{
		MeshMaterial curMat = it->first;
		curMat.ReduceClaims();
	}
	MaterialGeomMap.clear();
	if (sdf) delete sdf;
	sdf = nullptr;

	isInit = false;
}

void HIGHOMEGA::RENDER::GraphicsModel::GenerateGeom(std::vector<TriUV>& triList, std::vector<RasterVertex>& renderVertexVector)
{
	unsigned int iOffset = 0;
	renderVertexVector.reserve(triList.size() * 3);

	for (TriUV & curTri : triList)
	{
		RasterVertex rv;
		packRasterVertex(curTri.eArr[0], curTri.colArr[0], curTri.uvArr[0], curTri.normVec, rv);
		renderVertexVector.push_back(rv);
		packRasterVertex(curTri.eArr[1], curTri.colArr[1], curTri.uvArr[1], curTri.normVec, rv);
		renderVertexVector.push_back(rv);
		packRasterVertex(curTri.eArr[2], curTri.colArr[2], curTri.uvArr[2], curTri.normVec, rv);
		renderVertexVector.push_back(rv);
	}
}

HIGHOMEGA::RENDER::GraphicsModel::GraphicsModel()
{
	modelTransMat.Ident();
	modelTransMatInv.Ident();
}

void HIGHOMEGA::RENDER::GraphicsModel::Model(std::string & newGroupId, MeshMaterial & origMaterial, std::vector<TriUV>& triList, bool gpuResideOnly, bool inpImmutable)
{
	if (isInit) RemovePast();

	modelTransMat.Ident();
	modelTransMatInv.Ident();
	std::vector <RasterVertex> renderVertexVector;
	GenerateGeom(triList, renderVertexVector);

	MaterialGeomMap[origMaterial].emplace_back();
	GeometryClass *curGeom = &MaterialGeomMap[origMaterial].back();
	origMaterial.BumpClaims();

	curGeom->Geometry(Instance, renderVertexVector, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), origMaterial.isAlphaKeyed, newGroupId, gpuResideOnly, inpImmutable);
	curGeom->getRTGeom().SetMask(origMaterial.rayMask);

	isInit = true;
}

void HIGHOMEGA::RENDER::GraphicsModel::Model(HIGHOMEGA::MESH::Mesh & inpMesh, std::string belong, InstanceClass &ptrToInstance, std::function<bool(int, DataGroup &)> inpFilterFunction, mat4 *inpTransform, bool gpuResideOnly, bool inpImmutable, bool loadAnimationData, vec3* viewPos)
{
	if (isInit) RemovePast();

	modelTransMat.Ident();
	modelTransMatInv.Ident();

	std::vector <RasterVertex> verts;
	std::vector <VertexAnimationInfo> vertAnimData;
	std::string parentArmature;

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		if (!inpFilterFunction(i, inpMesh.DataGroups[i])) continue;

		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		HIGHOMEGA::MESH::DataBlock *keyFrameBlock = nullptr;
		Mesh::getDataBlock(curPolyGroup, "KEYFRAME_DATA", &keyFrameBlock);
		if (loadAnimationData && keyFrameBlock) // Animation data...
		{
			Animation animationData;
			int nBones, nKeyFrames;
			if ( !keyFrameBlock->rows[0][1].ivalue(nBones) ||
				 !keyFrameBlock->rows[0][3].ivalue(nKeyFrames) ) throw std::runtime_error("Malformed animation data, fetching num of bones and key frames");
			for (int j = 0; j != nKeyFrames; j++)
			{
				unsigned int poseDataOffset = 1 + (j * (nBones + 1));
				Pose newPose;
				int keyFrameFetch;
				if ( !keyFrameBlock->rows[poseDataOffset][1].ivalue(keyFrameFetch) ) throw std::runtime_error("Malformed animation data, fetching key frame");
				newPose.keyFrameTime = (unsigned int)keyFrameFetch;
				for (int k = 0; k != nBones; k++)
				{
					Bone newBone;
					if ( !keyFrameBlock->rows[poseDataOffset + (k + 1)][0].svalue(newBone.name) ) throw std::runtime_error("Malformed animation data, fetching bone name");
					for (int l = 0; l != 16; l++)
						if ( !keyFrameBlock->rows[poseDataOffset + (k + 1)][l + 1].fvalue(newBone.bone.i[l / 4][l % 4]) ) throw std::runtime_error("Malformed animation data, fetching bone pose");
					newPose.pose.push_back(newBone);
				}
				animationData.keyFrames.push_back(newPose);
			}
			animationData.lastKeyFrameTime = 0;
			for (int j = 0; j != animationData.keyFrames.size(); j++)
				animationData.lastKeyFrameTime = max(animationData.lastKeyFrameTime, animationData.keyFrames[j].keyFrameTime);
			for (Pose & curPose : animationData.keyFrames)
				curPose.keyFrameFract = (float)curPose.keyFrameTime / (float)animationData.lastKeyFrameTime;
			armatures[curPolyGroup.name].anim = animationData;
			armatures[curPolyGroup.name].anim.GetPose(0u, armatures[curPolyGroup.name].referencePoseInv);
			armatures[curPolyGroup.name].referencePoseInv.Inv();
			armatures[curPolyGroup.name].anim.GetPose(0u, armatures[curPolyGroup.name].currentPose);
			armatures[curPolyGroup.name].currentPoseBuf = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(float) * 16 * (unsigned int)armatures[curPolyGroup.name].currentPose.pose.size());
			continue;
		}

		HIGHOMEGA::MESH::DataBlock *triBlock = nullptr;
		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock)) continue;

		float tmpFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "invisible", tmpFloat)) continue;

		HIGHOMEGA::MESH::DataBlock *colsBlock = nullptr;
		HIGHOMEGA::MESH::DataBlock *vertexWeightsBlock = nullptr, *vertexWeightRefsBlock = nullptr, *parentArmatureRef = nullptr;
		bool hasCols;
		bool hasAnim;

		if ( Mesh::getDataBlock(curPolyGroup, "COLS", &colsBlock) )
			hasCols = true;
		else
			hasCols = false;

		if (loadAnimationData &&
			Mesh::getDataBlock(curPolyGroup, "WEIGHTS", &vertexWeightsBlock))
			hasAnim = true;
		else
			hasAnim = false;

		verts.resize(triBlock->blob.size() / sizeof(RasterVertex));
		if (verts.size() == 0) continue;
		memcpy(verts.data(), triBlock->blob.data(), triBlock->blob.size());
		if (hasAnim)
		{
			vertAnimData.resize(vertexWeightsBlock->blob.size() / sizeof(VertexAnimationInfo));
			memcpy(vertAnimData.data(), vertexWeightsBlock->blob.data(), vertexWeightsBlock->blob.size());
			Mesh::getDataBlock(curPolyGroup, "PARENT_ARMATURE", &parentArmatureRef);
		}

		vec3 edge, vnorm, col;
		vec2 uv;
		if (inpTransform)
			for (unsigned int i = 0; i != verts.size(); i++)
			{
				unpackRasterVertex(edge, col, uv, vnorm, verts[i]);
				edge = *inpTransform * edge;
				packRasterVertex(edge, col, uv, vnorm, verts[i]);
			}

		if (hasAnim)
		{
			 if ( !parentArmatureRef->rows[0][0].svalue(parentArmature) ) throw std::runtime_error("Could not get parent armature for graphics model load");
		}

		HIGHOMEGA::MESH::DataBlock *propsBlock = nullptr;
		if (!Mesh::getDataBlock(curPolyGroup, "PROPS", &propsBlock)) continue;

		MeshMaterial mat(*propsBlock, belong, ptrToInstance);

		std::string newGroupId = curPolyGroup.name;
		newGroupId += "_";
		newGroupId += std::to_string(i);

		float staticTessPower;
		if (!hasAnim && Mesh::getDataRowFloat(*propsBlock, "staticTessellationPower", staticTessPower))
		{
			tessellateGeom[newGroupId].maxTessellationPower = staticTessPower;
			tessellateGeom[newGroupId].curTessellationPower = 0.0f;
			tessellateGeom[newGroupId].targetTessellationPower = staticTessPower;
			Mesh::getDataRowFloat(*propsBlock, "staticTessellationDisplacement", tessellateGeom[newGroupId].tessellationDisplacement);
			tessellateGeom[newGroupId].verts = verts;
			tessellateGeom[newGroupId].curGeom = nullptr;
			tessellateGeom[newGroupId].addedGeom = nullptr;
			tessellateGeom[newGroupId].mat = mat;

			verts.clear();
			continue;
		}

		MaterialGeomMap[mat].emplace_back ();
		GeometryClass *curGeom = &MaterialGeomMap[mat].back();

		if (hasAnim)
		{
			curGeom->Geometry(ptrToInstance, verts, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), mat.isAlphaKeyed, newGroupId, gpuResideOnly, inpImmutable, &vertAnimData, &parentArmature);
		}
		else
		{
			curGeom->Geometry(ptrToInstance, verts, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), mat.isAlphaKeyed, newGroupId, gpuResideOnly, inpImmutable);
		}

		verts.clear();
		vertAnimData.clear();
	}

	doStaticTessellation(ptrToInstance, gpuResideOnly, inpImmutable, viewPos);

	isInit = true;
}

HIGHOMEGA::RENDER::GraphicsModel::~GraphicsModel()
{
	RemovePast();
}

GeometryClass * HIGHOMEGA::RENDER::GraphicsModel::getGeometryById(std::string & groupId)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if (it2->getGroupId() == groupId)
				return &(*it2);
	return nullptr;
}

MeshMaterial HIGHOMEGA::RENDER::GraphicsModel::getMaterialById(std::string & groupId)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if (it2->getGroupId() == groupId) {
				return it->first;
			}
	return MeshMaterial();
}

void HIGHOMEGA::RENDER::GraphicsModel::doStaticTessellation(InstanceClass& ptrToInstance, bool gpuResideOnly, bool inpImmutable, vec3* viewPos)
{
	for (std::pair<const std::string, TessellateVerts>& curPair : tessellateGeom)
	{
		std::string curGroupId = curPair.first;
		TessellateVerts& curTessVerts = curPair.second;
		MeshMaterial& curMat = curTessVerts.mat;

		if (!curTessVerts.curGeom) GeometryClass::getMinMax(curTessVerts.verts, curTessVerts.origMin, curTessVerts.origMax);
		if (viewPos)
		{
			curTessVerts.cent = (curTessVerts.origMin + curTessVerts.origMax) * 0.5f;
			curTessVerts.rad = (curTessVerts.origMax - curTessVerts.origMin).length() * 0.5f;
			curTessVerts.targetTessellationPower = min (curTessVerts.maxTessellationPower, floor(curTessVerts.maxTessellationPower * (curTessVerts.rad / (*viewPos - curTessVerts.cent).length())));
		}
		else
			curTessVerts.targetTessellationPower = curTessVerts.maxTessellationPower;

		if (curTessVerts.curTessellationPower == curTessVerts.targetTessellationPower && curTessVerts.curGeom) continue;
		curTessVerts.curTessellationPower = curTessVerts.targetTessellationPower;

		if (curTessVerts.curTessellationPower == 0.0f)
		{
			MaterialGeomMap[curMat].emplace_back();
			GeometryClass* addedGeom = &MaterialGeomMap[curMat].back();
			addedGeom->Geometry(ptrToInstance, curTessVerts.verts, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, gpuResideOnly, inpImmutable);
			addedGeom->setRTBufferDirty();
			if (curTessVerts.curGeom)
			{
				curTessVerts.addedGeom = addedGeom;
				curTessVerts.addedGeom->notifySubmissions = curTessVerts.curGeom->notifySubmissions;
			}
			else
				curTessVerts.curGeom = addedGeom;
			continue;
		}

		GeometryClass reconstructedBase(ptrToInstance, curTessVerts.verts, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, gpuResideOnly, inpImmutable);
		unsigned int baseVertexCount = reconstructedBase.getVertBuffer().getSize() / sizeof(RasterVertex);

		struct
		{
			unsigned int triCountTessFactorInputStrideOutputStride[4];
			float displacementAmount;
		} PrimitiveTessellateParams;
		PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[1] = (unsigned int)powf(4.0f, curTessVerts.curTessellationPower);
		PrimitiveTessellateParams.displacementAmount = curTessVerts.tessellationDisplacement;

		BufferClass PrimitiveTessellateParamsBuf;
		ShaderResourceSet tessellateShader;
		ComputeSubmission tessellateCompute;

		std::vector<RasterVertex> tmpVerts;
		tmpVerts.resize(baseVertexCount * PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[1]);
		MaterialGeomMap[curMat].emplace_back();
		MaterialGeomMap[curMat].back().Geometry(ptrToInstance, tmpVerts, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, gpuResideOnly, inpImmutable);

		PrimitiveTessellateParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(PrimitiveTessellateParams));

		tessellateShader.Create("shaders/primitiveTessellate.comp.spv", "main");
		tessellateShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, reconstructedBase.getVertBuffer());
		tessellateShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, MaterialGeomMap[curMat].back().getVertBuffer());
		tessellateShader.AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 2, curMat.hgtRef->elem);
		tessellateShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 3, PrimitiveTessellateParamsBuf);
		tessellateCompute.MakeDispatch(Instance, std::string("default"), tessellateShader, 1, 1, 1);

		for (unsigned int i = 0; i != (unsigned int)curTessVerts.curTessellationPower; i++)
		{
			switch (i)
			{
			case 0:
				PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[0] = baseVertexCount / 3;
				PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[2] = PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[1];
				PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[3] = PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[1] / 4;
				break;
			default:
				PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[0] *= 4;
				PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[2] /= 4;
				PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[3] /= 4;
				break;
			}
			PrimitiveTessellateParamsBuf.UploadSubData(0, &PrimitiveTessellateParams, sizeof(PrimitiveTessellateParams));
			tessellateCompute.UpdateDispatchSize(Instance, std::string("default"), (unsigned int)ceil((double)PrimitiveTessellateParams.triCountTessFactorInputStrideOutputStride[0] / (double)TessellateWorkGroup()), 1, 1);
			tessellateCompute.Submit();
		}

		std::list<GeometryClass>::iterator addedGeomIt = MaterialGeomMap[curMat].end();
		--addedGeomIt;
		addedGeomIt->SetMinMax(curTessVerts.origMin - vec3(PrimitiveTessellateParams.displacementAmount), curTessVerts.origMax + vec3(PrimitiveTessellateParams.displacementAmount));
		addedGeomIt->setRTBufferDirty();
		if (curTessVerts.curGeom)
		{
			curTessVerts.addedGeom = &(*addedGeomIt);
			curTessVerts.addedGeom->notifySubmissions = curTessVerts.curGeom->notifySubmissions;
		}
		else
			curTessVerts.curGeom = &(*addedGeomIt);
	}
}

void HIGHOMEGA::RENDER::GraphicsModel::removeOldTessellation()
{
	for (std::pair<const std::string, TessellateVerts>& curPair : tessellateGeom)
	{
		TessellateVerts& curTessVerts = curPair.second;
		if (curTessVerts.addedGeom)
		{
			MeshMaterial& curMat = curTessVerts.mat;
			for (std::list<GeometryClass>::iterator it = MaterialGeomMap[curMat].begin(); it != MaterialGeomMap[curMat].end(); it++)
				if (curTessVerts.curGeom == &(*it))
				{
					MaterialGeomMap[curMat].erase(it);
					curTessVerts.curGeom = curTessVerts.addedGeom;
					curTessVerts.addedGeom = nullptr;
					break;
				}
		}
	}
}

void HIGHOMEGA::RENDER::GraphicsModel::removeGroupById(std::string & groupId)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
	{
		std::list<GeometryClass>::iterator findRes = std::find_if(it->second.begin(), it->second.end(), [&groupId](GeometryClass & curGeom) { return curGeom.getGroupId() == groupId; });
		if (findRes != it->second.end())
		{
			it->second.erase(findRes);
			if (it->second.size() == 0)
				MaterialGeomMap.erase(it->first);
			return;
		}
	}
}

void HIGHOMEGA::RENDER::GraphicsModel::transformVertsSlow(mat4 & trans, std::string groupId, mat4 *localTrans, vec3 *iSectP1, vec3 *iSectP2, bool *lineHit, vec3 *mulVCol)
{
	mat4 tmpIdent;
	tmpIdent.Ident();

	bool singleGroup = false;
	if (groupId != std::string("")) singleGroup = true;

	vec3 vCol;

	bool doOptionalHitDetection = false;
	if (iSectP1 && iSectP2 && lineHit)
	{
		doOptionalHitDetection = true;
		*lineHit = false;
	}

	mat4 finalLocalTrans = trans;

	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			GeometryClass *curKey = &(*it2);
			if (singleGroup && curKey->getGroupId().rfind (groupId, 0) != 0) continue;
			if (vertexCache.find(curKey) == vertexCache.end())
			{
				vertexCache[curKey].resize(it2->getVertBuffer().getSize() / sizeof(RasterVertex));
				it2->getVertBuffer().DownloadSubData(0, vertexCache[curKey].data(), (unsigned int)vertexCache[curKey].size() * sizeof(RasterVertex));
			}
			else if (vertexCache[curKey].size() != it2->getVertBuffer().getSize() / sizeof(RasterVertex))
			{
				vertexCache[curKey].clear();
				vertexCache[curKey].resize(it2->getVertBuffer().getSize() / sizeof(RasterVertex));
				it2->getVertBuffer().DownloadSubData(0, vertexCache[curKey].data(), (unsigned int)vertexCache[curKey].size() * sizeof(RasterVertex));
			}

			if (localTrans)
			{
				vec3 instCent = vec3(0.0f);
				vec3 edge, vnorm, vcol;
				vec2 uv;
				for (int i = 0; i != vertexCache[curKey].size(); i++)
				{
					unpackRasterVertex(edge, vcol, uv, vnorm, vertexCache[curKey][i]);
					instCent += edge;
				}
				instCent /= (float)vertexCache[curKey].size();

				mat4 deTranslate, translate;
				deTranslate.Ident();
				translate.Ident();
				deTranslate.i[0][3] = -instCent.x;
				deTranslate.i[1][3] = -instCent.y;
				deTranslate.i[2][3] = -instCent.z;
				translate.i[0][3] = instCent.x;
				translate.i[1][3] = instCent.y;
				translate.i[2][3] = instCent.z;
				finalLocalTrans = trans * translate * (*localTrans) * deTranslate;
			}

			std::vector<RasterVertex> transformedGeom = vertexCache[curKey];
			for (int i = 0; i != transformedGeom.size(); i++)
			{
				vec3 edge, vnorm, vcol;
				vec2 uv;
				unpackRasterVertex(edge, vcol, uv, vnorm, transformedGeom[i]);
				edge = finalLocalTrans * edge;

				if (mulVCol)
				{
					vcol.x *= mulVCol->x;
					vcol.y *= mulVCol->y;
					vcol.z *= mulVCol->z;
				}

				packRasterVertex(edge, vcol, uv, vnorm, transformedGeom[i]);

				if (doOptionalHitDetection && i % 3 == 2 && *lineHit == false)
				{
					vec3 e1, e2, e3;
					unpackRasterVertex(e1, vcol, uv, vnorm, transformedGeom[i]);
					unpackRasterVertex(e2, vcol, uv, vnorm, transformedGeom[i - 1]);
					unpackRasterVertex(e3, vcol, uv, vnorm, transformedGeom[i - 2]);
					float tmpK = 1.0f;
					if (LineSegTri(*iSectP1, *iSectP2, e1, e2, e3, tmpK))
						*lineHit = true;
				}
			}
			it2->getVertBuffer().UploadSubData(0, transformedGeom.data(), (unsigned int)transformedGeom.size() * sizeof(RasterVertex));
		}
}

std::vector<RasterVertex>* HIGHOMEGA::RENDER::GraphicsModel::getVertsSlow(std::string groupId)
{
	GeometryClass *curKey = getGeometryById(groupId);
	if (curKey == nullptr) return nullptr;
	if (vertexCache.find(curKey) == vertexCache.end())
	{
		vertexCache[curKey].resize(curKey->getVertBuffer().getSize() / sizeof(RasterVertex));
		curKey->getVertBuffer().DownloadSubData(0, vertexCache[curKey].data(), (unsigned int)vertexCache[curKey].size() * sizeof(RasterVertex));
	}
	return &vertexCache[curKey];
}

void HIGHOMEGA::RENDER::GraphicsModel::BlankAndResize(unsigned int scaleFactor)
{
	std::vector<RasterVertex> tmpVerts;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			tmpVerts.resize((it2->getVertBuffer().getSize() / sizeof(RasterVertex)) * scaleFactor);
			it2->ChangeGeom(tmpVerts);
		}
}

void HIGHOMEGA::RENDER::GraphicsModel::SetMinMax(vec3 minVal, vec3 maxVal)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			it2->SetMinMax(minVal, maxVal);
}

void HIGHOMEGA::RENDER::GraphicsModel::TransformCorners(mat4 inpMat)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			it2->TransformCorners(inpMat);
	modelTransMat = inpMat;
	modelTransMatInv = inpMat.Inv();
}

void HIGHOMEGA::RENDER::GraphicsModel::SetDirty()
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			it2->setRTBufferDirty();
}

void HIGHOMEGA::RENDER::GraphicsModel::ChangeGeom(std::string & groupId, std::vector<TriUV> & triList)
{
	GeometryClass *geomPtr = getGeometryById (groupId);
	if (!geomPtr) return;

	std::vector <RasterVertex> renderVertexVector;
	GenerateGeom(triList, renderVertexVector);

	MeshMaterial materialById = getMaterialById(groupId);

	geomPtr->ChangeGeom(renderVertexVector);
}

void HIGHOMEGA::RENDER::GraphicsModel::UpdateGeom(std::string & groupId, std::vector<TriUV>& triList)
{
	GeometryClass *geomPtr = getGeometryById(groupId);
	if (!geomPtr) return;

	std::vector <RasterVertex> renderVertexVector;
	GenerateGeom(triList, renderVertexVector);

	MeshMaterial materialById = getMaterialById(groupId);

	geomPtr->Update(renderVertexVector);
}

void HIGHOMEGA::RENDER::GraphicsModel::getModelMinMax(vec3 & outMin, vec3 & outMax)
{
	bool firstGeom = true;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			if (firstGeom)
			{ 
				outMin = it2->getGeomMin();
				outMax = it2->getGeomMax();
				firstGeom = false;
			}
			else
			{
				outMin = vec3(min(outMin.x, it2->getGeomMin().x), min(outMin.y, it2->getGeomMin().y), min(outMin.z, it2->getGeomMin().z));
				outMax = vec3(max(outMax.x, it2->getGeomMax().x), max(outMax.y, it2->getGeomMax().y), max(outMax.z, it2->getGeomMax().z));
			}
		}
}

void HIGHOMEGA::RENDER::GraphicsModel::getUntransformedMinMax(vec3 & outMin, vec3 & outMax)
{
	bool firstGeom = true;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			if (firstGeom)
			{
				outMin = it2->getUntransformedGeomMin();
				outMax = it2->getUntransformedGeomMax();
				firstGeom = false;
			}
			else
			{
				outMin = vec3(min(outMin.x, it2->getUntransformedGeomMin().x), min(outMin.y, it2->getUntransformedGeomMin().y), min(outMin.z, it2->getUntransformedGeomMin().z));
				outMax = vec3(max(outMax.x, it2->getUntransformedGeomMax().x), max(outMax.y, it2->getUntransformedGeomMax().y), max(outMax.z, it2->getUntransformedGeomMax().z));
			}
		}
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::ChangeSignal(unsigned long long changedItem)
{
	if (!RTInstance::Enabled()) return;

	changedItems.push_back(changedItem);
}

unsigned long long HIGHOMEGA::RENDER::GroupedTraceSubmission::SceneID()
{
	for (unsigned long long curChangedItem : changedItems)
	{
		rtScene.RemoveAll(curChangedItem);

		InstanceProperties curProps;
		std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = allSubmittedItems[curChangedItem].item->MaterialGeomMap;
		std::vector <ImageClass *> materialCollection;
		materialCollection.reserve(5);
		for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
		{
			MeshMaterial itFirst = it->first;
			if (!allSubmittedItems[curChangedItem].filterFunction(itFirst)) continue;

			materialCollection.clear();
			materialCollection.push_back(&itFirst.diffRef->elem);

			if (itFirst.nrmRef) materialCollection.push_back(&itFirst.nrmRef->elem);
			else materialCollection.push_back(&itFirst.diffRef->elem);

			if (itFirst.rghRef) materialCollection.push_back(&itFirst.rghRef->elem);
			else materialCollection.push_back(&itFirst.diffRef->elem);

			if (itFirst.hgtRef) materialCollection.push_back(&itFirst.hgtRef->elem);
			else materialCollection.push_back(&itFirst.diffRef->elem);

			if (itFirst.spcRef) materialCollection.push_back(&itFirst.spcRef->elem);
			else materialCollection.push_back(&itFirst.diffRef->elem);

			CompileInstanceProperties(curProps, itFirst);

			for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
				rtScene.Add(curChangedItem, materialCollection, (*it2).getRTGeom(), curProps, Instance);
		}
	}
	changedItems.clear();

	return rtScene.rtSceneID();
}

SubmittedRenderItem HIGHOMEGA::RENDER::GroupedTraceSubmission::Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction)
{
	if (!RTInstance::Enabled())
	{
		SubmittedRenderItem retVal;
		retVal.producer = this;
		return retVal;
	}

	InstanceProperties curProps;
	SubmittedRenderItem retVal;
	retVal.producer = this;
	retVal.item = &inpModel;
	retVal.itemId = mersenneTwister64BitPRNG();
	retVal.filterFunction = inpFilterFunction;

	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = inpModel.MaterialGeomMap;
	std::vector <ImageClass *> materialCollection;
	materialCollection.reserve(5);
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;

		materialCollection.clear();
		materialCollection.push_back(&itFirst.diffRef->elem);

		if (itFirst.nrmRef) materialCollection.push_back(&itFirst.nrmRef->elem);
		else materialCollection.push_back(&itFirst.diffRef->elem);

		if (itFirst.rghRef) materialCollection.push_back(&itFirst.rghRef->elem);
		else materialCollection.push_back(&itFirst.diffRef->elem);

		if (itFirst.hgtRef) materialCollection.push_back(&itFirst.hgtRef->elem);
		else materialCollection.push_back(&itFirst.diffRef->elem);

		if (itFirst.spcRef) materialCollection.push_back(&itFirst.spcRef->elem);
		else materialCollection.push_back(&itFirst.diffRef->elem);

		CompileInstanceProperties(curProps, itFirst);

		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			rtScene.Add(retVal.itemId, materialCollection, (*it2).getRTGeom(), curProps, Instance);
			(*it2).notifySubmissions[this] = retVal.itemId;
		}
	}

	allSubmittedItems[retVal.itemId] = retVal;

	return retVal;
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::Remove(SubmittedRenderItem & inpSubmittedRenderItem)
{
	if (!RTInstance::Enabled()) return;

	rtScene.RemoveAll(inpSubmittedRenderItem.itemId);

	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = allSubmittedItems[inpSubmittedRenderItem.itemId].item->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions.erase(this);

	allSubmittedItems.erase(inpSubmittedRenderItem.itemId);
}

unsigned int HIGHOMEGA::RENDER::GroupedBVHSubmission::CompMortonWorkGroupSize()
{
	return 32;
}

unsigned int HIGHOMEGA::RENDER::GroupedBVHSubmission::CompMortonTrisPerThreadSize()
{
	return 1000;
}

unsigned int HIGHOMEGA::RENDER::GroupedBVHSubmission::CompressWorkGroupSize()
{
	return 32;
}

void HIGHOMEGA::RENDER::GroupedBVHSubmission::ChangeSignal(unsigned long long changedItem)
{
	sceneChanged = true;
}

unsigned long long HIGHOMEGA::RENDER::GroupedBVHSubmission::SceneID()
{
	if (RTInstance::Enabled()) return sceneID;

	if (sceneChanged)
	{
		totalInstanceCount = 0u;
		totalSceneTriCount = 0u;
		for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
			for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
			{
				MeshMaterial itFirst = curMatGeomPairing.first;
				if (!curSubmittedItem.second.filterFunction(itFirst)) continue;
				for (GeometryClass & curGeom : curMatGeomPairing.second)
				{
					totalSceneTriCount += curGeom.getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					totalInstanceCount++;
				}
			}

		instanceCountAligned = (unsigned int)(ceil((double)totalInstanceCount / 1000.0) * 1000.0);
		triCountAligned = (unsigned int)(ceil((double)totalSceneTriCount / 1000.0) * 1000.0);
		nodeCountAligned = (unsigned int)(ceil((double)(2 * totalSceneTriCount - 1) / 2000.0) * 2000.0);

		mortonParams.resize(instanceCountAligned);
		sourceGeom.reserve(instanceCountAligned);
		sourceMats.reserve(instanceCountAligned * 5);
		sourceGeom.clear();
		sourceMats.clear();
		trisComp.resize(triCountAligned);
		nodes.resize(nodeCountAligned);

		unsigned int triOffset = 0, curInstanceIndex = 0;
		unsigned int maxTriCount = 0;
		for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
			for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
			{
				MeshMaterial curMat = curMatGeomPairing.first;
				if (!curSubmittedItem.second.filterFunction(curMat)) continue;
				for (GeometryClass & curGeom : curMatGeomPairing.second)
				{
					unsigned int curTriCount = curGeom.getVertBuffer().getSize() / (sizeof(RasterVertex) * 3);
					sourceGeom.emplace_back(RESOURCE_SSBO, COMPUTE, 1, 0, curGeom.getVertBuffer());
					sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.diffRef->elem);
					if (curMat.nrmRef) sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.nrmRef->elem);
					else sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.diffRef->elem);
					if (curMat.rghRef) sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.rghRef->elem);
					else sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.diffRef->elem);
					if (curMat.hgtRef) sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.hgtRef->elem);
					else sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.diffRef->elem);
					if (curMat.spcRef) sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.spcRef->elem);
					else sourceMats.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, 0, curMat.diffRef->elem);

					mortonParams[curInstanceIndex].OffsetLen[0] = triOffset;
					mortonParams[curInstanceIndex].OffsetLen[1] = curTriCount;
					triOffset += curTriCount;
					curInstanceIndex++;
					vec3 & rtInstMin = curGeom.getGeomMin();
					vec3 & rtInstMax = curGeom.getGeomMax();
					if (maxTriCount == 0)
					{
						buildParams.mapMinTriCount[0] = rtInstMin.x;
						buildParams.mapMinTriCount[1] = rtInstMin.y;
						buildParams.mapMinTriCount[2] = rtInstMin.z;
						buildParams.mapMax[0] = rtInstMax.x;
						buildParams.mapMax[1] = rtInstMax.y;
						buildParams.mapMax[2] = rtInstMax.z;
						maxTriCount = curTriCount;
					}
					else
					{
						buildParams.mapMinTriCount[0] = min(buildParams.mapMinTriCount[0], rtInstMin.x);
						buildParams.mapMinTriCount[1] = min(buildParams.mapMinTriCount[1], rtInstMin.y);
						buildParams.mapMinTriCount[2] = min(buildParams.mapMinTriCount[2], rtInstMin.z);
						buildParams.mapMax[0] = max(buildParams.mapMax[0], rtInstMax.x);
						buildParams.mapMax[1] = max(buildParams.mapMax[1], rtInstMax.y);
						buildParams.mapMax[2] = max(buildParams.mapMax[2], rtInstMax.z);
						maxTriCount = max(maxTriCount, curTriCount);
					}
				}
			}
		buildParams.mapMinTriCount[3] = *((float *)(&totalSceneTriCount));

		if (buildParamsBuf.getSize() == 0)
			buildParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &buildParams, (unsigned int)sizeof(buildParamsStruct));
		else
			buildParamsBuf.UploadSubData(0, &buildParams, sizeof(buildParamsStruct));

		if (instanceCountAligned > 0)
		{
			unsigned int compMortonParamsBufSize = instanceCountAligned * sizeof(compMortonParamsStruct);
			if (compMortonParamsBufSize > compMortonParamsBuf.getSize())
			{
				compMortonParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, compMortonParamsBufSize);
			}
			compMortonParamsBuf.UploadSubData(0, mortonParams.data(), (unsigned int)totalInstanceCount * sizeof(compMortonParamsStruct));
		}
		else
		{
			if (compMortonParamsBuf.getSize() == 0) compMortonParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(compMortonParamsStruct));
		}

		if (triCountAligned > 0)
		{
			unsigned int triBufSize = triCountAligned * sizeof(BVHTriangle);
			if (triBufSize > triBuf.getSize())
				triBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, triBufSize);
		}
		else
		{
			if (triBuf.getSize() == 0) triBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHTriangle));
		}

		if (instanceCountAligned > 0)
		{
			unsigned int instBufSize = instanceCountAligned * sizeof(InstanceProperties);
			if (instBufSize > instBuf.getSize())
			{
				instBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, instBufSize);
			}
			instProps.resize(totalInstanceCount);
			unsigned int instancePropCounter = 0u;
			for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
				for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
				{
					MeshMaterial curMat = curMatGeomPairing.first;
					if (!curSubmittedItem.second.filterFunction(curMat)) continue;
					CompileInstanceProperties(instProps[instancePropCounter++], curMat);
				}
			instBuf.UploadSubData(0, instProps.data(), totalInstanceCount * sizeof(InstanceProperties));
		}
		else
		{
			if (instBuf.getSize() == 0) instBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(InstanceProperties));
		}

		if (nodeCountAligned > 0)
		{
			unsigned int nodesBufSize = nodeCountAligned * sizeof(BVHNode);
			if (nodesBufSize > nodesBuf.getSize())
				nodesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, nodesBufSize);
		}
		else
		{
			if (nodesBuf.getSize() == 0) nodesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHNode));
		}

		if (triCountAligned > 0)
		{
			unsigned int compressedTrisBufSize = triCountAligned * sizeof(BVHTriangleCompressed);
			if (compressedTrisBufSize > trisCompressedBuf.getSize())
				trisCompressedBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, compressedTrisBufSize);
		}
		else
		{
			if (trisCompressedBuf.getSize() == 0) trisCompressedBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHTriangleCompressed));
		}

		if (shadersCreated)
		{
			mortonShader.RemovePast();
			compressionShader.RemovePast();
			shadersCreated = false;
		}
		mortonShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 0, buildParamsBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, compMortonParamsBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, triBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 1, sourceGeom);
		mortonShader.Create("shaders/computeMortonCodes.comp.spv", "main");
		mortonSubmission.MakeDispatch(Instance, std::string("default"), mortonShader, (unsigned int)ceil((double)maxTriCount / (double)(CompMortonWorkGroupSize() * CompMortonTrisPerThreadSize())), curInstanceIndex, 1);

		compressionShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 0, buildParamsBuf);
		compressionShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, triBuf);
		compressionShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, instBuf);
		compressionShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, trisCompressedBuf);
		compressionShader.Create("shaders/compressTris.comp.spv", "main");
		compressionSubmission.MakeDispatch(Instance, std::string("default"), compressionShader, (unsigned int)ceil((double)totalSceneTriCount / CompressWorkGroupSize()), 1, 1);

		shadersCreated = true;
		sceneChanged = false;

		sceneID = mersenneTwister64BitPRNG();
	}
	else
	{
		bool firstComparison = true;
		for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
		{
			for (std::pair<const MeshMaterial, std::list<GeometryClass>> & curMatGeomPairing : curSubmittedItem.second.item->MaterialGeomMap)
			{
				MeshMaterial curMat = curMatGeomPairing.first;
				if (!curSubmittedItem.second.filterFunction(curMat)) continue;
				for (GeometryClass & curGeom : curMatGeomPairing.second)
				{
					vec3 & rtInstMin = curGeom.getGeomMin();
					vec3 & rtInstMax = curGeom.getGeomMax();
					if (firstComparison)
					{
						firstComparison = false;
						buildParams.mapMinTriCount[0] = rtInstMin.x;
						buildParams.mapMinTriCount[1] = rtInstMin.y;
						buildParams.mapMinTriCount[2] = rtInstMin.z;
						buildParams.mapMax[0] = rtInstMax.x;
						buildParams.mapMax[1] = rtInstMax.y;
						buildParams.mapMax[2] = rtInstMax.z;
					}
					else
					{
						buildParams.mapMinTriCount[0] = min(buildParams.mapMinTriCount[0], rtInstMin.x);
						buildParams.mapMinTriCount[1] = min(buildParams.mapMinTriCount[1], rtInstMin.y);
						buildParams.mapMinTriCount[2] = min(buildParams.mapMinTriCount[2], rtInstMin.z);
						buildParams.mapMax[0] = max(buildParams.mapMax[0], rtInstMax.x);
						buildParams.mapMax[1] = max(buildParams.mapMax[1], rtInstMax.y);
						buildParams.mapMax[2] = max(buildParams.mapMax[2], rtInstMax.z);
					}
				}
			}
		}
		buildParamsBuf.UploadSubData(0, &buildParams, sizeof(buildParamsStruct));
	}
	mortonSubmission.Submit();

	mapMin = vec3(buildParams.mapMinTriCount[0], buildParams.mapMinTriCount[1], buildParams.mapMinTriCount[2]);
	mapMax = vec3(buildParams.mapMax[0], buildParams.mapMax[1], buildParams.mapMax[2]);

	tris.resize(triCountAligned);
	trisTmp.resize(triCountAligned);
	triBuf.DownloadSubData(0, tris.data(), totalSceneTriCount * sizeof(BVHTriangle));

	const unsigned int numPredicates = 256;
	for (unsigned int bitShift = 0; bitShift != 32; bitShift += 8)
	{
		unsigned int bitMask = (0x000000FFu) << bitShift;
		BVHTriangle *srcBuf, *dstBuf;
		if ((bitShift / 8) % 2 == 0)
		{
			srcBuf = tris.data();
			dstBuf = trisTmp.data();
		}
		else
		{
			srcBuf = trisTmp.data();
			dstBuf = tris.data();
		}
		unsigned int predicateSums[numPredicates];
		unsigned int predicateOffets[numPredicates];
		for (unsigned int i = 0; i != numPredicates; i++)
			predicateSums[i] = 0;
		for (unsigned int i = 0; i != totalSceneTriCount; i++)
			predicateSums[((*(unsigned int *)(&srcBuf[i].e3Morton[3])) & bitMask) >> bitShift]++;
		for (unsigned int i = 0; i != numPredicates; i++)
		{
			predicateOffets[i] = 0;
			for (unsigned int j = 0; j != i; j++)
				predicateOffets[i] += predicateSums[j];
		}
		for (unsigned int i = 0; i != totalSceneTriCount; i++)
		{
			unsigned int curPred = ((*(unsigned int *)(&srcBuf[i].e3Morton[3]) & bitMask)) >> bitShift;
			dstBuf[predicateOffets[curPred]] = srcBuf[i];
			predicateOffets[curPred]++;
		}
	}

	triBuf.UploadSubData(0, tris.data(), totalSceneTriCount * sizeof(BVHTriangle));

	compressionSubmission.Submit();

	bvhGenCPU.ProduceNodesOnly(tris.data(), nodes.data(), totalSceneTriCount);

	nodesBuf.UploadSubData(0, nodes.data(), (2 * totalSceneTriCount - 1) * sizeof(BVHNode));

	return sceneID;
}

SubmittedRenderItem HIGHOMEGA::RENDER::GroupedBVHSubmission::Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial&curMat)> inpFilterFunction)
{
	SubmittedRenderItem retVal;
	retVal.producer = this;
	retVal.item = &inpModel;
	retVal.itemId = mersenneTwister64BitPRNG();
	retVal.filterFunction = inpFilterFunction;

	allSubmittedItems[retVal.itemId] = retVal;

	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = inpModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions[this] = retVal.itemId;
	}

	sceneChanged = true;

	return retVal;
}

void HIGHOMEGA::RENDER::GroupedBVHSubmission::Remove(SubmittedRenderItem & inpSubmittedRenderItem)
{
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = allSubmittedItems[inpSubmittedRenderItem.itemId].item->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions.erase(this);

	allSubmittedItems.erase(inpSubmittedRenderItem.itemId);

	sceneChanged = true;
}

unsigned int HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::CompMortonWorkGroupSize()
{
	return 32;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::ChangeSignal(unsigned long long changedItem)
{
	changedItems.push_back(allSubmittedItems[changedItem]);
}

unsigned long long HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::SceneID()
{
	if (RTInstance::Enabled()) return sceneID;

	if (changedItems.size() > 0)
	{
		GraphicsModel::UpdateSDFs(changedItems, true);
		changedItems.clear();

		sceneChanged = true;
	}

	sdfLeaves.clear();
	allSDFBVHImages.clear();
	allSubmittedItemsVector.clear();
	leafTransformInfo.clear();

	sdfLeaves.reserve(allSubmittedItems.size());
	allSDFBVHImages.reserve(allSubmittedItems.size());
	allSubmittedItemsVector.reserve(allSubmittedItems.size());
	leafTransformInfo.reserve(allSubmittedItems.size());

	for (std::pair<const unsigned long long, SubmittedRenderItem> & curSubmittedItem : allSubmittedItems)
		allSubmittedItemsVector.push_back(curSubmittedItem.second);

	unsigned int gridCounter = 0u;
	vec3 modMin, modMax;
	vec3 untransformedModMin, untransformedModMax;
	vec3 globalMin, globalMax;
	for (SubmittedRenderItem & curSubmittedItem : allSubmittedItemsVector)
	{
		if (!curSubmittedItem.item->sdf) continue;
		curSubmittedItem.item->getModelMinMax(modMin, modMax);
		curSubmittedItem.item->getUntransformedMinMax(untransformedModMin, untransformedModMax);

		modMin -= vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		modMax += vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		untransformedModMin -= vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		untransformedModMax += vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);

		modMax.x = modMin.x + ceilf((modMax.x - modMin.x) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
		modMax.y = modMin.y + ceilf((modMax.y - modMin.y) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
		modMax.z = modMin.z + ceilf((modMax.z - modMin.z) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
		untransformedModMax.x = untransformedModMin.x + ceilf((untransformedModMax.x - untransformedModMin.x) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
		untransformedModMax.y = untransformedModMin.y + ceilf((untransformedModMax.y - untransformedModMin.y) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;
		untransformedModMax.z = untransformedModMin.z + ceilf((untransformedModMax.z - untransformedModMin.z) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS) * HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;

		sdfLeaves.emplace_back();
		sdfLeaves.back().leafMaxLeafId[0] = modMax.x;
		sdfLeaves.back().leafMaxLeafId[1] = modMax.y;
		sdfLeaves.back().leafMaxLeafId[2] = modMax.z;
		sdfLeaves.back().leafMaxLeafId[3] = *((float *)&gridCounter);
		sdfLeaves.back().leafMinMorton[0] = modMin.x;
		sdfLeaves.back().leafMinMorton[1] = modMin.y;
		sdfLeaves.back().leafMinMorton[2] = modMin.z;
		allSDFBVHImages.push_back(curSubmittedItem.item->sdf);
		leafTransformInfo.emplace_back();
		UnpackMat4(curSubmittedItem.item->modelTransMat, leafTransformInfo.back().transMat);
		UnpackMat4(curSubmittedItem.item->modelTransMatInv, leafTransformInfo.back().invTransMat);
		leafTransformInfo.back().bMin[0] = untransformedModMin.x;
		leafTransformInfo.back().bMin[1] = untransformedModMin.y;
		leafTransformInfo.back().bMin[2] = untransformedModMin.z;
		leafTransformInfo.back().bMax[0] = untransformedModMax.x;
		leafTransformInfo.back().bMax[1] = untransformedModMax.y;
		leafTransformInfo.back().bMax[2] = untransformedModMax.z;
		if (gridCounter == 0u)
		{
			globalMin = modMin;
			globalMax = modMax;
		}
		else
		{
			globalMin.x = min(globalMin.x, modMin.x);
			globalMin.y = min(globalMin.y, modMin.y);
			globalMin.z = min(globalMin.z, modMin.z);
			globalMax.x = max(globalMax.x, modMax.x);
			globalMax.y = max(globalMax.y, modMax.y);
			globalMax.z = max(globalMax.z, modMax.z);
		}
		gridCounter++;
	}
	unsigned int leafCount = (unsigned int)sdfLeaves.size();
	unsigned int nodeCount = 2 * leafCount - 1;
	leafCountAligned = (unsigned int)(ceil((double)leafCount / 1000.0) * 1000.0);
	nodeCountAligned = (unsigned int)(ceil((double)nodeCount / 2000.0) * 2000.0);

	if (sceneChanged)
	{
		nodes.resize(nodeCount);

		if (buildParamsBuf.getSize() == 0) buildParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(buildParamsStruct));

		if (leafCountAligned > 0)
		{
			unsigned int leavesBufSize = leafCountAligned * sizeof(SDFLeaf);
			if (leavesBufSize > leavesBuf.getSize())
				leavesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, leavesBufSize);
		}
		else
		{
			if (leavesBuf.getSize() == 0) leavesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(SDFLeaf));
		}

		if (nodeCountAligned > 0)
		{
			unsigned int nodesBufSize = nodeCountAligned * sizeof(BVHNode);
			if (nodesBufSize > nodesBuf.getSize())
				nodesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, nodesBufSize);
		}
		else
		{
			if (nodesBuf.getSize() == 0) nodesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BVHNode));
		}

		if (shadersCreated)
		{
			mortonShader.RemovePast();
			shadersCreated = false;
		}
		mortonShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 0, buildParamsBuf);
		mortonShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, leavesBuf);
		mortonShader.Create("shaders/computeMortonCodesSDFLeaves.comp.spv", "main");
		mortonSubmission.MakeDispatch(Instance, std::string("default"), mortonShader, (unsigned int)ceil((double)leafCount / (double)CompMortonWorkGroupSize()), 1, 1);

		shadersCreated = true;

		if (leafCountAligned > 0)
		{
			unsigned int invMatBufSize = leafCountAligned * sizeof(singleLeafTransform);
			if (invMatBufSize > invMatBuf.getSize())
			{
				invMatBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, invMatBufSize);
			}
		}
		else
		{
			if (invMatBuf.getSize() == 0) invMatBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(singleLeafTransform));
		}

		SDFs.clear();
		SDFs.reserve(allSDFBVHImages.size());
		for (unsigned int i = 0; i != allSDFBVHImages.size(); i++)
			SDFs.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 3, 0, *(allSDFBVHImages[i]));

		sceneID = mersenneTwister64BitPRNG();

		sceneChanged = false;
	}

	leavesBuf.UploadSubData(0, sdfLeaves.data(), leafCount * sizeof(SDFLeaf));

	buildParams.mapMinLeafCount[0] = globalMin.x;
	buildParams.mapMinLeafCount[1] = globalMin.y;
	buildParams.mapMinLeafCount[2] = globalMin.z;
	buildParams.mapMinLeafCount[3] = *((float *)(&leafCount));
	buildParams.mapMax[0] = globalMax.x;
	buildParams.mapMax[1] = globalMax.y;
	buildParams.mapMax[2] = globalMax.z;
	buildParamsBuf.UploadSubData(0, &buildParams, sizeof(buildParamsStruct));
	mortonSubmission.Submit();

	leavesBuf.DownloadSubData(0, sdfLeaves.data(), leafCount * sizeof(SDFLeaf));

	sdfLeavesTmp.resize(leafCount);

	const unsigned int numPredicates = 256;
	for (unsigned int bitShift = 0; bitShift != 32; bitShift += 8)
	{
		unsigned int bitMask = (0x000000FFu) << bitShift;
		SDFLeaf *srcBuf, *dstBuf;
		if ((bitShift / 8) % 2 == 0)
		{
			srcBuf = sdfLeaves.data();
			dstBuf = sdfLeavesTmp.data();
		}
		else
		{
			srcBuf = sdfLeavesTmp.data();
			dstBuf = sdfLeaves.data();
		}
		unsigned int predicateSums[numPredicates];
		unsigned int predicateOffets[numPredicates];
		for (unsigned int i = 0; i != numPredicates; i++)
			predicateSums[i] = 0;
		for (unsigned int i = 0; i != leafCount; i++)
			predicateSums[((*(unsigned int *)(&srcBuf[i].leafMinMorton[3])) & bitMask) >> bitShift]++;
		for (unsigned int i = 0; i != numPredicates; i++)
		{
			predicateOffets[i] = 0;
			for (unsigned int j = 0; j != i; j++)
				predicateOffets[i] += predicateSums[j];
		}
		for (unsigned int i = 0; i != leafCount; i++)
		{
			unsigned int curPred = ((*(unsigned int *)(&srcBuf[i].leafMinMorton[3]) & bitMask)) >> bitShift;
			dstBuf[predicateOffets[curPred]] = srcBuf[i];
			predicateOffets[curPred]++;
		}
	}

	SDFBVH.ProduceNodesOnly(sdfLeaves.data(), nodes.data(), leafCount);
	nodesBuf.UploadSubData(0, nodes.data(), nodeCount * sizeof(BVHNode));
	invMatBuf.UploadSubData(0, leafTransformInfo.data(), (unsigned int)leafTransformInfo.size() * sizeof(singleLeafTransform));

	return sceneID;
}

SubmittedRenderItem HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial&curMat)> inpFilterFunction)
{
	SubmittedRenderItem retVal;
	retVal.producer = this;
	retVal.item = &inpModel;
	retVal.itemId = mersenneTwister64BitPRNG();
	retVal.filterFunction = inpFilterFunction;

	allSubmittedItems[retVal.itemId] = retVal;

	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = inpModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions[this] = retVal.itemId;
	}

	sceneChanged = true;

	return retVal;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::Remove(SubmittedRenderItem & inpSubmittedRenderItem)
{
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = allSubmittedItems[inpSubmittedRenderItem.itemId].item->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions.erase(this);

	sceneChanged = true;

	allSubmittedItems.erase(inpSubmittedRenderItem.itemId);
}

void HIGHOMEGA::RENDER::GroupedRenderSubmission::CompileInstanceProperties(InstanceProperties & outProp, const MeshMaterial & inpMaterial)
{
	// Build the per-instance data we're sending to the shader!

	unsigned int attribs0 = 0u;

	// Do we have a normal map?
	if (inpMaterial.nrmName != "") 
		attribs0 |= 0x00000001;

	// Do we have a roughness map?
	if (inpMaterial.rghName != "") 
		attribs0 |= 0x00000002;

	// Is this a material smooth? (using vertex normals?)
	if (inpMaterial.smooth)
		attribs0 |= 0x00000004;

	// Is this a double-sided?
	if (!inpMaterial.pipelineFlags.backFaceCulling && inpMaterial.pipelineFlags.changedBackFaceCulling)
		attribs0 |= 0x00000008;

	// Is this a backdrop glass?
	if (inpMaterial.backDropGlass)
		attribs0 |= 0x00000010;

	// Is this a post process material?
	if (inpMaterial.postProcess)
		attribs0 |= 0x00000020;

	// Is it di-electric?
	if (inpMaterial.dielectric)
		attribs0 |= 0x00000040;

	// Is it alpha keyed?
	if (inpMaterial.isAlphaKeyed)
		attribs0 |= 0x00000080;

	// Scattering (i.e. smoke)
	if (inpMaterial.scattering)
		attribs0 |= 0x00000100;

	outProp.attribs[0] = *((float *)&attribs0);

	// Player ID
	outProp.attribs[1] = *((float *)&inpMaterial.playerId);

	// emissivity
	outProp.attribs[2] = inpMaterial.emissivity;

	// refractiveIndex
	outProp.attribs[3] = inpMaterial.refractiveIndex;

	// Total u offset
	outProp.attribs[4] = inpMaterial.uvOffset.x;

	// Total v offset
	outProp.attribs[5] = inpMaterial.uvOffset.y;

	// Vertex displacement height factor
	outProp.attribs[6] = inpMaterial.heightMapDisplaceFactor;

	// Subdivision amount for tessellation
	outProp.attribs[7] = inpMaterial.subDivAmount;
}

HIGHOMEGA::RENDER::GroupedRenderSubmission::GroupedRenderSubmission()
{
}

std::unordered_map <GroupedRasterSubmission*, std::unordered_map <std::string, std::unordered_map <std::string, DescriptorSets*>>> GroupedRasterSubmission::globalDSCache;
std::mutex GroupedRasterSubmission::globalDSCache_mutex;

void HIGHOMEGA::RENDER::GroupedRasterSubmission::DestroySubmissionData()
{
	entireMatGeomMap.clear();
	MaterialGeomPairings.clear();
	MaterialBindings.clear();
	GeomBindings.clear();
	SSBOdata.clear();
	// SSBO used to be destroyed here... but now is just naturally destroyed when the submission is destroyed
	PSO_DSL_DS_GeomPairings.clear();
	Rasterlet.ResetRasterlet();
}

std::string HIGHOMEGA::RENDER::GroupedRasterSubmission::GenerateRasterPSOKey(MeshMaterial& inpMat, PipelineFlags& inpPFlags, bool renderMode)
{
	std::string selShaderName = inpMat.shaderName;
	if (shaders.find(inpMat.shaderName) == shaders.end()) {
		selShaderName = "default";
	}

	std::string rasterPSOKey = "";
	rasterPSOKey += shaders[selShaderName]->vertex_shader;
	rasterPSOKey += shaders[selShaderName]->tc_shader;
	rasterPSOKey += shaders[selShaderName]->te_shader;
	rasterPSOKey += shaders[selShaderName]->geom_shader,
	rasterPSOKey += shaders[selShaderName]->fragment_shader;
	rasterPSOKey += std::to_string(inpPFlags.backFaceCulling);
	rasterPSOKey += std::to_string(inpPFlags.frontFaceCulling);
	rasterPSOKey += std::to_string(inpPFlags.frontFaceClockWise);
	rasterPSOKey += std::to_string(inpPFlags.blendEnable);
	rasterPSOKey += (std::to_string(inpPFlags.alphaBlendOp) + std::to_string(inpPFlags.alphaBlending));
	rasterPSOKey += (std::to_string(inpPFlags.colorBlendOp) + std::to_string(inpPFlags.colorBlending));
	rasterPSOKey += (std::to_string(inpPFlags.srcColorFactor) + std::to_string(inpPFlags.srcAlphaFactor));
	rasterPSOKey += (std::to_string(inpPFlags.srcAlphaFactor) + std::to_string(inpPFlags.dstAlphaFactor));
	rasterPSOKey += (std::to_string(inpPFlags.redMask) + std::to_string(inpPFlags.greenMask) + std::to_string(inpPFlags.blueMask) + std::to_string(inpPFlags.alphaMask));
	rasterPSOKey += std::to_string(inpPFlags.depthTest);
	rasterPSOKey += std::to_string(inpPFlags.depthWrite);
	rasterPSOKey += std::to_string(renderMode);
	return rasterPSOKey;
}

HIGHOMEGA::RENDER::GroupedRasterSubmission::GroupedRasterSubmission()
{
	frameBuffer = nullptr;
	shaders.clear();
	recordedCmdBuf = false;
	allResourcesChanged = false;
	cullingResourcesChanged = false;
	cullingCompute = nullptr;
	cullingResourceSet = nullptr;
	resourcesRequested = 0;
	redoSubmissionData = false;
	clearColor = vec3(0.0f);
	clearColorW = 1.0f;
	depthClear = 1.0f;
	stencilClear = 0;
	requestsBVH = false;
	doesCulling = false;
	Setup_HiZ = false;
	recordMatGeomBindings = false;
	takenMatGeomBindings = nullptr;
	cmdRecordId = 0lu;
	MipChainPass1_HiZ = MipChainPass2_HiZ = nullptr;
	MipChainPass1ShaderResources_HiZ = MipChainPass2ShaderResources_HiZ = nullptr;
	matTransform = 0;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Create(InstanceClass & ptrToInstance)
{
	Rasterlet.Rasterlet(Instance);
}

HIGHOMEGA::RENDER::GroupedRasterSubmission::~GroupedRasterSubmission()
{
	{std::lock_guard<std::mutex>lk(globalDSCache_mutex);
	for (std::pair <const std::string, std::unordered_map <std::string, DescriptorSets*>>& curAllPSODS : globalDSCache[this])
		for (std::pair <const std::string, DescriptorSets*>& curDS : curAllPSODS.second)
			delete curDS.second;
	globalDSCache.erase(this);}
	if (cullingCompute) delete cullingCompute;
	if (cullingResourceSet) delete cullingResourceSet;
	if (MipChainPass1_HiZ) delete MipChainPass1_HiZ;
	if (MipChainPass2_HiZ) delete MipChainPass2_HiZ;
	if (MipChainPass1ShaderResources_HiZ) delete MipChainPass1ShaderResources_HiZ;
	if (MipChainPass2ShaderResources_HiZ) delete MipChainPass2ShaderResources_HiZ;
	cullingCompute = nullptr;
	cullingResourceSet = nullptr;
	MipChainPass1_HiZ = MipChainPass2_HiZ = nullptr;
	MipChainPass1ShaderResources_HiZ = MipChainPass2ShaderResources_HiZ = nullptr;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::ChangeSignal(unsigned long long changedItem)
{
	if (recordedCmdBuf) redoSubmissionData = true;
}

SubmittedRenderItem HIGHOMEGA::RENDER::GroupedRasterSubmission::Add(GraphicsModel & inpModel, std::function<bool(MeshMaterial & curMat)> inpFilterFunction)
{
	SubmittedRenderItem retVal;
	retVal.producer = this;
	retVal.item = &inpModel;
	retVal.itemId = mersenneTwister64BitPRNG();
	retVal.filterFunction = inpFilterFunction;

	allSubmittedItems[retVal.itemId] = retVal;

	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = inpModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions[this] = retVal.itemId;
	}

	if (recordedCmdBuf) redoSubmissionData = true;

	return retVal;
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::postProcessOnlyFilter(MeshMaterial & curMat)
{
	if (!curMat.postProcess) return false;
	return true;
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::everythingFilter(MeshMaterial & curMat)
{
	return true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Remove(SubmittedRenderItem & inpSubmittedRenderItem)
{
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = allSubmittedItems[inpSubmittedRenderItem.itemId].item->MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			(*it2).notifySubmissions.erase(this);

	allSubmittedItems.erase(inpSubmittedRenderItem.itemId);

	if (recordedCmdBuf) redoSubmissionData = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::SetFrameBuffer(FramebufferClass & inpFrameBuffer)
{
	frameBuffer = &inpFrameBuffer;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::SetShader(std::string inpName, ShaderResourceSet & inpShader)
{
	shaders[inpName] = &inpShader;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::SetClearColor(vec3 inClearColor, float inClearColorW)
{
	clearColor = inClearColor;
	clearColorW = inClearColorW;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::SetDepthClear(float inDepthClear)
{
	depthClear = inDepthClear;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::SetStencilClear(unsigned int inStencilClear)
{
	stencilClear = inStencilClear;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::SetDefaultPipelineFlags(PipelineFlags & inDefaultPipelineFlags)
{
	defaultPipelineFlags = inDefaultPipelineFlags;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::makeAsync()
{
	Rasterlet.makeAsync();
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::makeSerial()
{
	Rasterlet.makeSerial();
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::requestBVH(GroupedBVHSubmission & bvhHolder)
{
	sourceBVH = &bvhHolder;
	sourceBVHId = 0u;
	requestsBVH = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::requestSDFBVH(GroupedSDFBVHSubmission & sdfBvhSubmission)
{
	sourceSDFBVHSubmission = &sdfBvhSubmission;
	sourceSDFBVHId = 0u;
	requestsSDFBVH = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::doCulling(FrustumClass &inpFrustum, CULL_MODE inCullMode)
{
	vec3 frustumCullEye = inpFrustum.eye;
	vec3 lookNorm = inpFrustum.look.normalized();
	vec3 side = cross(lookNorm, inpFrustum.up).normalized();
	vec3 realUp = cross(side, lookNorm);
	if (realUp * inpFrustum.up < 0.0f) realUp = -realUp;
	vec3 screenCenter = inpFrustum.eye + lookNorm;
	float fovYScale = tanf((inpFrustum.screen_fov / 180.0f) * HIGHOMEGA_PI * 0.5f);
	vec3 realUpScaled = realUp * fovYScale;
	vec3 screenTopCenter = screenCenter + realUpScaled;
	vec3 screenBottomCenter = screenCenter - realUpScaled;
	vec3 sideScaled = side * fovYScale * inpFrustum.screen_whr;

	if (inpFrustum.isOrtho)
	{
		vec3 left = side;
		vec3 right = -side;
		vec3 top = realUp;
		vec3 bottom = -realUp;

		float absTop = fabs(inpFrustum.ortho_top);
		float absBottom = fabs(inpFrustum.ortho_bottom);
		float absLeft = fabs(inpFrustum.ortho_left);
		float absRight = fabs(inpFrustum.ortho_right);

		CullParams.pl[0] = left.x;
		CullParams.pl[1] = left.y;
		CullParams.pl[2] = left.z;
		CullParams.pl[3] = -(left * (frustumCullEye + left * absLeft));
		CullParams.pl[4] = right.x;
		CullParams.pl[5] = right.y;
		CullParams.pl[6] = right.z;
		CullParams.pl[7] = -(right * (frustumCullEye + right * absRight));
		CullParams.pl[8] = top.x;
		CullParams.pl[9] = top.y;
		CullParams.pl[10] = top.z;
		CullParams.pl[11] = -(top * (frustumCullEye + top * absTop));
		CullParams.pl[12] = bottom.x;
		CullParams.pl[13] = bottom.y;
		CullParams.pl[14] = bottom.z;
		CullParams.pl[15] = -(bottom * (frustumCullEye + bottom * absBottom));
	}
	else
	{
		vec3 frustumCullP1 = screenTopCenter + sideScaled;
		vec3 frustumCullP2 = screenTopCenter - sideScaled;
		vec3 frustumCullP3 = screenBottomCenter - sideScaled;
		vec3 frustumCullP4 = screenBottomCenter + sideScaled;
		vec3 frustumCullCent = (frustumCullP1 + frustumCullP2 + frustumCullP3 + frustumCullP4) * 0.25f;
		vec3 p2_Eye = frustumCullP2 - frustumCullEye;
		vec3 p4_Eye = frustumCullP4 - frustumCullEye;
		vec3 cullCent_p2 = frustumCullCent - frustumCullP2;
		vec3 cullCent_p4 = frustumCullCent - frustumCullP4;
		vec3 frustumCullN1 = cross(frustumCullP1 - frustumCullP2, p2_Eye).normalized();
		vec3 frustumCullN2 = cross(frustumCullP2 - frustumCullP3, p2_Eye).normalized();
		vec3 frustumCullN3 = cross(frustumCullP3 - frustumCullP4, p4_Eye).normalized();
		vec3 frustumCullN4 = cross(frustumCullP4 - frustumCullP1, p4_Eye).normalized();
		if (frustumCullN1 * cullCent_p2 > 0.0f) frustumCullN1 = -frustumCullN1;
		if (frustumCullN2 * cullCent_p2 > 0.0f) frustumCullN2 = -frustumCullN2;
		if (frustumCullN3 * cullCent_p4 > 0.0f) frustumCullN3 = -frustumCullN3;
		if (frustumCullN4 * cullCent_p4 > 0.0f) frustumCullN4 = -frustumCullN4;
		CullParams.pl[0] = frustumCullN1.x;
		CullParams.pl[1] = frustumCullN1.y;
		CullParams.pl[2] = frustumCullN1.z;
		CullParams.pl[3] = -(frustumCullN1 * frustumCullEye);
		CullParams.pl[4] = frustumCullN2.x;
		CullParams.pl[5] = frustumCullN2.y;
		CullParams.pl[6] = frustumCullN2.z;
		CullParams.pl[7] = -(frustumCullN2 * frustumCullEye);
		CullParams.pl[8] = frustumCullN3.x;
		CullParams.pl[9] = frustumCullN3.y;
		CullParams.pl[10] = frustumCullN3.z;
		CullParams.pl[11] = -(frustumCullN3 * frustumCullEye);
		CullParams.pl[12] = frustumCullN4.x;
		CullParams.pl[13] = frustumCullN4.y;
		CullParams.pl[14] = frustumCullN4.z;
		CullParams.pl[15] = -(frustumCullN4 * frustumCullEye);
	}

	if (!Setup_HiZ)
	{
		Output_HiZ.resize(7);
		for (int i = 0; i != 7; i++)
			Output_HiZ[i].CreateImageStore(Instance, R32F, 64 >> i, 64 >> i, 1, _2D, true);

		mipParams_HiZ.blockSize[0] = (unsigned int)ceil((double)frameBuffer->getWidth() / 64.0);
		mipParams_HiZ.blockSize[1] = (unsigned int)ceil((double)frameBuffer->getHeight() / 64.0);
		mipParamsBuffer_HiZ.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &mipParams_HiZ, (unsigned int)sizeof(mipParamsStruct));

		MipChainPass1_HiZ = new ComputeSubmission;
		MipChainPass2_HiZ = new ComputeSubmission;
		MipChainPass1ShaderResources_HiZ = new ShaderResourceSet;
		MipChainPass2ShaderResources_HiZ = new ShaderResourceSet;

		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 0, *(frameBuffer->GetDepthStencil()));
		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, Output_HiZ[0]);
		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_UBO, COMPUTE, 0, 2, mipParamsBuffer_HiZ);
		MipChainPass1ShaderResources_HiZ->Create(cullMode == FRUSTUM_HIZ_REVERSE ? "shaders/depthmip1ReverseZ.comp.spv" : "shaders/depthmip1.comp.spv", "main");
		MipChainPass1_HiZ->MakeDispatch(Instance, std::string("DepthMip1"), *MipChainPass1ShaderResources_HiZ, 8, 8, 1);

		for (int i = 0; i != 7; i++)
			MipChainPass2ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, i, Output_HiZ[i]);
		MipChainPass2ShaderResources_HiZ->Create(cullMode == FRUSTUM_HIZ_REVERSE ? "shaders/depthmip2ReverseZ.comp.spv" : "shaders/depthmip2.comp.spv", "main");
		MipChainPass2_HiZ->MakeDispatch(Instance, std::string("DepthMip2"), *MipChainPass2ShaderResources_HiZ, 1, 1, 1);

		Setup_HiZ = true;
	}

	cullMode = inCullMode;
	cachedCullingFrustum = &inpFrustum;

	Rasterlet.doesCulling = doesCulling = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::keepMatGeomBindings()
{
	recordMatGeomBindings = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::consumeMatGeomBindings(GroupedRasterSubmission *inpConsumeMatGeomBindingsFrom)
{
	takenMatGeomBindings = inpConsumeMatGeomBindingsFrom;
	matGeomSourceRecordID = 0ul;
}

unsigned int HIGHOMEGA::RENDER::GroupedRasterSubmission::instanceCount()
{
	return (unsigned int)SSBOdata.size();
}

unsigned int HIGHOMEGA::RENDER::GroupedRasterSubmission::WorkGroupFrustumCullX()
{
	return 32;
}

unsigned int HIGHOMEGA::RENDER::GroupedRasterSubmission::WorkGroupHiZCullX()
{
	return 32;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Render()
{
	if (frameBuffer == nullptr || shaders.size() == 0) return;

	unsigned long long curBVHId;
	if (requestsBVH && sourceBVHId != (curBVHId = sourceBVH->SceneID()))
	{
		sourceBVHId = curBVHId;
		if (recordedCmdBuf)
		{
			allResourcesChanged = true;
			redoSubmissionData = true;
		}
	}

	unsigned long long curSDFBVHId;
	if (requestsSDFBVH && sourceSDFBVHId != (curSDFBVHId = sourceSDFBVHSubmission->SceneID()))
	{
		sourceSDFBVHId = curSDFBVHId;
		if (recordedCmdBuf)
		{
			allResourcesChanged = true;
			redoSubmissionData = true;
		}
	}

	if (takenMatGeomBindings && matGeomSourceRecordID != takenMatGeomBindings->cmdRecordId)
	{
		matGeomSourceRecordID = takenMatGeomBindings->cmdRecordId;
		if (recordedCmdBuf)
		{
			allResourcesChanged = true;
			redoSubmissionData = true;
		}
	}

	if (redoSubmissionData)
	{
		DestroySubmissionData();
		recordedCmdBuf = false;
		redoSubmissionData = false;
	}

	if (!recordedCmdBuf)
	{
		for (std::pair <const unsigned long long, SubmittedRenderItem> & curRenderItemKV : allSubmittedItems)
		{
			std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash> & curItemMatGeomMap = curRenderItemKV.second.item->MaterialGeomMap;
			for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
			{
				MeshMaterial itFirst = it->first;
				if (!curRenderItemKV.second.filterFunction(itFirst)) continue;

				if (matTransform) itFirst = matTransform(itFirst);

				for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
					entireMatGeomMap[itFirst].push_back(&(*it2));
			}
		}

		unsigned int instanceCount = 0;
		MaterialGeomPairings.reserve(entireMatGeomMap.size());
		for (std::unordered_map<MeshMaterial, std::vector<GeometryClass *>>::iterator it = entireMatGeomMap.begin(); it != entireMatGeomMap.end(); ++it)
		{
			MaterialGeomPairing curPairing;
			curPairing.mat = it->first;
			curPairing.geom = &it->second;
			MaterialGeomPairings.push_back(curPairing);
			instanceCount += (unsigned int)it->second.size();
		}

		std::sort(MaterialGeomPairings.begin(), MaterialGeomPairings.end(), [&shadersRef = shaders](const MaterialGeomPairing& lhs, const MaterialGeomPairing& rhs)
		{
			std::string lhsSelShaderName = lhs.mat.shaderName;
			std::string rhsSelShaderName = rhs.mat.shaderName;
			if (shadersRef.find(lhs.mat.shaderName) == shadersRef.end()) {
				lhsSelShaderName = "default";
			}
			if (shadersRef.find(rhs.mat.shaderName) == shadersRef.end()) {
				rhsSelShaderName = "default";
			}
			ShaderResourceSet *lhsShaderRef = shadersRef[lhsSelShaderName];
			ShaderResourceSet *rhsShaderRef = shadersRef[rhsSelShaderName];
			std::string lhsPipelineStagesString = lhsShaderRef->vertex_shader + lhsShaderRef->tc_shader + lhsShaderRef->te_shader + lhsShaderRef->geom_shader + lhsShaderRef->fragment_shader;
			std::string rhsPipelineStagesString = rhsShaderRef->vertex_shader + rhsShaderRef->tc_shader + rhsShaderRef->te_shader + rhsShaderRef->geom_shader + rhsShaderRef->fragment_shader;

			return (lhs.mat.renderOrder < rhs.mat.renderOrder) || (lhs.mat.renderOrder == rhs.mat.renderOrder && lhsPipelineStagesString < rhsPipelineStagesString);
		});
		SSBOdata.resize(instanceCount);
		unsigned int instCounter = 0;
		for (MaterialGeomPairing & curMatGeomPairing : MaterialGeomPairings)
			for (GeometryClass * curGeom : *curMatGeomPairing.geom)
			{
				if (recordMatGeomBindings)
				{
					MeshMaterial & curMat = curMatGeomPairing.mat;
					MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.diffRef->elem);
					if (curMat.nrmRef) MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.nrmRef->elem);
					else MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.diffRef->elem);
					if (curMat.rghRef) MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.rghRef->elem);
					else MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.diffRef->elem);
					if (curMat.hgtRef) MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.hgtRef->elem);
					else MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.diffRef->elem);
					if (curMat.spcRef) MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.spcRef->elem);
					else MaterialBindings.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, 0, curMat.diffRef->elem);

					GeomBindings.emplace_back(RESOURCE_SSBO, FRAGMENT, 3, 0, curGeom->getVertBuffer());
				}
				CompileInstanceProperties(SSBOdata[instCounter++], curMatGeomPairing.mat);
			}

		cmdRecordId = mersenneTwister64BitPRNG();

		if (SSBOdata.size() > 0)
		{
			unsigned int SSBOLargeAlignmentSize = ((((unsigned int)SSBOdata.size() / 1000) + 1) * 1000) * sizeof(InstanceProperties);
			if (SSBOLargeAlignmentSize > SSBO.getSize())
			{
				SSBO.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, SSBOLargeAlignmentSize);
				allResourcesChanged = true;
			}
			SSBO.UploadSubData(0, SSBOdata.data(), (unsigned int)SSBOdata.size() * sizeof(InstanceProperties));
		}
		else
		{
			if (SSBO.getSize() == 0)
			{
				SSBO.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(InstanceProperties));
				allResourcesChanged = true;
			}
		}

		if (allResourcesChanged) {
			std::lock_guard<std::mutex>lk(globalDSCache_mutex);
			for (std::pair <std::string const, std::unordered_map <std::string, DescriptorSets*>>& curSubToAllPSODS : GroupedRasterSubmission::globalDSCache[this])
				for (std::pair <std::string const, DescriptorSets*>& curAllPSODS : curSubToAllPSODS.second)
					curAllPSODS.second->SetDirty(true);
			allResourcesChanged = false;
		}

		if (doesCulling)
		{
			unsigned int conditionalInstanceCount = (((instanceCount / 1000) + 1) * 1000);
			if (instanceCount > 0)
			{
				unsigned int conditionalBufferSize = conditionalInstanceCount * sizeof(unsigned int);
				if (conditionalBufferSize > conditionalBuffer.getSize())
				{
					conditionalBuffer.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO | USAGE_CONDITIONAL_RENDERING, Instance, nullptr, conditionalBufferSize);
					InstanceMinMaxBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)(conditionalInstanceCount * sizeof(InstanceMinMaxStruct)));
					cullingResourcesChanged = true;
				}
			}
			else
			{
				if (conditionalBuffer.getSize() == 0) conditionalBuffer.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO | USAGE_CONDITIONAL_RENDERING, Instance, nullptr, (unsigned int)sizeof(unsigned int) * 4);
				if (InstanceMinMaxBuf.getSize() == 0) InstanceMinMaxBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(InstanceMinMaxStruct));
				cullingResourcesChanged = true;
			}
			Rasterlet.conditionalBufferPtr = &conditionalBuffer;
		}

		PipelineFlags pFlags;
		std::vector <ShaderResource> shaderResources;
		shaderResources.reserve(50);
		for (MaterialGeomPairing & curMatGeomPairing : MaterialGeomPairings)
		{
			pFlags = defaultPipelineFlags;
			MeshMaterial curMat = curMatGeomPairing.mat;

			if (curMat.pipelineFlags.changedDepthTest) pFlags.depthTest = curMat.pipelineFlags.depthTest;
			if (curMat.pipelineFlags.changedDepthWrite) pFlags.depthWrite = curMat.pipelineFlags.depthWrite;
			if (curMat.pipelineFlags.changedBackFaceCulling) pFlags.backFaceCulling = curMat.pipelineFlags.backFaceCulling;
			if (curMat.pipelineFlags.changedFrontFaceCulling) pFlags.frontFaceCulling = curMat.pipelineFlags.frontFaceCulling;
			if (curMat.pipelineFlags.changedFrontFaceClockWise) pFlags.frontFaceClockWise = curMat.pipelineFlags.frontFaceClockWise;

			if (curMat.pipelineFlags.changedBlendEnable)
			{
				pFlags.blendEnable = curMat.pipelineFlags.blendEnable;
				pFlags.alphaBlending = curMat.pipelineFlags.alphaBlending;
				pFlags.colorBlending = curMat.pipelineFlags.colorBlending;
				pFlags.srcColorFactor = curMat.pipelineFlags.srcColorFactor;
				pFlags.dstColorFactor = curMat.pipelineFlags.dstColorFactor;
				pFlags.srcAlphaFactor = curMat.pipelineFlags.srcAlphaFactor;
				pFlags.dstAlphaFactor = curMat.pipelineFlags.dstAlphaFactor;
				pFlags.alphaBlendOp = curMat.pipelineFlags.alphaBlendOp;
				pFlags.colorBlendOp = curMat.pipelineFlags.colorBlendOp;
			}

			if (curMat.pipelineFlags.changedColorMask)
			{
				pFlags.redMask = curMat.pipelineFlags.redMask;
				pFlags.greenMask = curMat.pipelineFlags.greenMask;
				pFlags.blueMask = curMat.pipelineFlags.blueMask;
				pFlags.alphaMask = curMat.pipelineFlags.alphaMask;
			}

			shaderResources.clear();
			shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 1, 0, curMat.diffRef->elem);
			if (curMat.nrmRef) shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 1, 1, curMat.nrmRef->elem);
			else shaderResources.emplace_back(RESOURCE_SAMPLER, ALL, 1, 1, curMat.diffRef->elem);
			if (curMat.rghRef) shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 1, 2, curMat.rghRef->elem);
			else shaderResources.emplace_back(RESOURCE_SAMPLER, ALL, 1, 2, curMat.diffRef->elem);
			if (curMat.hgtRef) shaderResources.emplace_back(RESOURCE_SAMPLER, TESS_EVAL, 1, 3, curMat.hgtRef->elem);
			else shaderResources.emplace_back(RESOURCE_SAMPLER, ALL, 1, 3, curMat.diffRef->elem);
			if (curMat.spcRef) shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 1, 4, curMat.spcRef->elem);
			else shaderResources.emplace_back(RESOURCE_SAMPLER, ALL, 1, 4, curMat.diffRef->elem);
			shaderResources.emplace_back(RESOURCE_SSBO, VERTEX, 1, 5, SSBO);
			if (requestsBVH)
			{
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 0, sourceBVH->nodesBuf);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 1, sourceBVH->trisCompressedBuf);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 2, sourceBVH->instBuf);
				shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, sourceBVH->sourceMats);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, sourceBVH->sourceGeom);
			}
			if (requestsSDFBVH)
			{
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 0, sourceSDFBVHSubmission->nodesBuf);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 1, sourceSDFBVHSubmission->invMatBuf);
				shaderResources.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 3, sourceSDFBVHSubmission->SDFs);
			}
			if (takenMatGeomBindings)
			{
				shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, takenMatGeomBindings->MaterialBindings);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 3, takenMatGeomBindings->GeomBindings);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, 0, takenMatGeomBindings->SSBO);
			}
			std::string selShaderName = curMat.shaderName;
			if (shaders.find(curMat.shaderName) == shaders.end()) {
				selShaderName = "default";
			}
			shaderResources.insert(shaderResources.end(), shaders[selShaderName]->getAdditionalResources().begin(), shaders[selShaderName]->getAdditionalResources().end());
			unsigned int curResourcesRequested = 0;
			for (ShaderResource & curRes : shaderResources)
				curResourcesRequested += curRes.ResourceCount();

			std::string rasterPSOKey = GenerateRasterPSOKey(curMat, pFlags, frameBuffer->getRenderMode());

			Raster_PSO_DSL *RasterPSODSL = nullptr;
			{
				std::lock_guard<std::mutex> lk(globalRaster_PSO_DSL_Cache_mutex);
				if (globalRaster_PSO_DSL_Cache.find(rasterPSOKey) == globalRaster_PSO_DSL_Cache.end())
				{
					globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL = new DescriptorSetLayout();
					globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL->CreateDescriptorSetLayout(shaderResources, &Instance);
					globalRaster_PSO_DSL_Cache[rasterPSOKey].PSO = new RasterPipelineStateClass(Instance, *globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL, pFlags, *frameBuffer, *((*curMatGeomPairing.geom)[0]), *shaders[selShaderName]);
				}
				RasterPSODSL = &globalRaster_PSO_DSL_Cache[rasterPSOKey];
			}
			
			DescriptorSets *DSPtr = nullptr;
			{
				std::lock_guard<std::mutex>lk(globalDSCache_mutex);
				std::string matStringKey = curMatGeomPairing.mat.diffName + curMatGeomPairing.mat.nrmName + curMatGeomPairing.mat.spcName + curMatGeomPairing.mat.rghName + curMatGeomPairing.mat.hgtName;
				if (globalDSCache[this].find(matStringKey) == globalDSCache[this].end() || globalDSCache[this][matStringKey].find(rasterPSOKey) == globalDSCache[this][matStringKey].end())
				{
					globalDSCache[this][matStringKey][rasterPSOKey] = new DescriptorSets(RasterPSODSL->DSL);
					globalDSCache[this][matStringKey][rasterPSOKey]->WriteDescriptorSets(shaderResources);
				}
				else if (globalDSCache[this][matStringKey][rasterPSOKey]->GetDirty())
				{
					if (curResourcesRequested == resourcesRequested)
					{
						shaderResources.clear();
						shaderResources.emplace_back(RESOURCE_SSBO, VERTEX, 1, 5, SSBO);
						if (requestsBVH)
						{
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 0, sourceBVH->nodesBuf);
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 1, sourceBVH->trisCompressedBuf);
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 2, sourceBVH->instBuf);
							shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 3, sourceBVH->sourceMats);
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, sourceBVH->sourceGeom);
						}
						if (requestsSDFBVH)
						{
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 0, sourceSDFBVHSubmission->nodesBuf);
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 2, 1, sourceSDFBVHSubmission->invMatBuf);
							shaderResources.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 3, sourceSDFBVHSubmission->SDFs);
						}
						if (takenMatGeomBindings)
						{
							shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 2, takenMatGeomBindings->MaterialBindings);
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 3, takenMatGeomBindings->GeomBindings);
							shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 4, 0, takenMatGeomBindings->SSBO);
						}

						globalDSCache[this][matStringKey][rasterPSOKey]->UpdateDescriptorSets(shaderResources);
					}
					else
					{
						globalDSCache[this][matStringKey][rasterPSOKey]->RewriteDescriptorSets(shaderResources);
					}
					globalDSCache[this][matStringKey][rasterPSOKey]->SetDirty(false);
				}
				DSPtr = globalDSCache[this][matStringKey][rasterPSOKey];
			}
			resourcesRequested = curResourcesRequested;
			PSO_DSL_DS_GeomPairing curPairing;
			curPairing.PSO_DSL = RasterPSODSL;
			curPairing.DS = DSPtr;
			curPairing.geom = curMatGeomPairing.geom;
			PSO_DSL_DS_GeomPairings.push_back(curPairing);
		}
		dynPipelineFlags.clear_color[0] = clearColor.x;
		dynPipelineFlags.clear_color[1] = clearColor.y;
		dynPipelineFlags.clear_color[2] = clearColor.z;
		dynPipelineFlags.clear_color[3] = clearColorW;
		dynPipelineFlags.depth_clear = depthClear;
		dynPipelineFlags.stencil_clear = stencilClear;
		dynPipelineFlags.viewport_width = frameBuffer->getWidth();
		dynPipelineFlags.viewport_height = frameBuffer->getHeight();
		dynPipelineFlags.viewport_x = 0;
		dynPipelineFlags.viewport_y = 0;
		Rasterlet.PrepareSubmission(PSO_DSL_DS_GeomPairings, dynPipelineFlags, *frameBuffer);
		recordedCmdBuf = true;
	}

	if (doesCulling)
	{
		if (!cullingCompute)
		{
			cullingCompute = new ComputeSubmission;
			cullingResourcesChanged = true;
		}
		if (CullParamsBuf.getSize() == 0)
		{
			CullParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(CullParamsStruct));
			cullingResourcesChanged = true;
		}
		unsigned int instanceCount = (unsigned int)SSBOdata.size();
		CullParams.nInstances = instanceCount;
		CullParamsBuf.UploadSubData(0, &CullParams, sizeof(CullParamsStruct));

		InstanceMinMaxData.resize(instanceCount);
		unsigned int instCounter2 = 0;
		for (MaterialGeomPairing & curMatGeomPairing : MaterialGeomPairings)
			for (GeometryClass * curGeom : *curMatGeomPairing.geom)
			{
				InstanceMinMaxData[instCounter2].min[0] = curGeom->getGeomMin().x;
				InstanceMinMaxData[instCounter2].min[1] = curGeom->getGeomMin().y;
				InstanceMinMaxData[instCounter2].min[2] = curGeom->getGeomMin().z;
				InstanceMinMaxData[instCounter2].max[0] = curGeom->getGeomMax().x;
				InstanceMinMaxData[instCounter2].max[1] = curGeom->getGeomMax().y;
				InstanceMinMaxData[instCounter2].max[2] = curGeom->getGeomMax().z;
				instCounter2++;
			}
		if (instanceCount > 0) InstanceMinMaxBuf.UploadSubData(0, InstanceMinMaxData.data(), (unsigned int)InstanceMinMaxData.size() * sizeof(InstanceMinMaxStruct));

		if (cullingResourcesChanged)
		{
			if (cullingResourceSet) delete cullingResourceSet;
			cullingResourceSet = new ShaderResourceSet;

			unsigned int conditionalInstanceCount = (((instanceCount / 1000) + 1) * 1000);

			cullingResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, CullParamsBuf);
			cullingResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 1, cachedCullingFrustum->Buffer);
			cullingResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, InstanceMinMaxBuf);
			cullingResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, conditionalBuffer);
			std::vector<ShaderResource> mipChain;
			for (int i = 0; i != Output_HiZ.size(); i++)
				mipChain.emplace_back(RESOURCE_SAMPLER, COMPUTE, 0, 4, Output_HiZ[i]);
			cullingResourceSet->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 4, mipChain);
			cullingResourceSet->Create(cullMode == HIZ_ONLY ? "shaders/HiZOnlyCull.comp.spv" : (cullMode == FRUSTUM_HIZ_REVERSE ? "shaders/frustumHiZCullReverseZ.comp.spv" : "shaders/frustumHiZCull.comp.spv"), "main");
			cullingCompute->MakeDispatch(Instance, std::string("cullPass"), *cullingResourceSet, (unsigned int)ceil((double)conditionalInstanceCount / (double)WorkGroupHiZCullX()), 1, 1);

			cullingResourcesChanged = false;
		}
		MipChainPass1_HiZ->Submit();
		MipChainPass2_HiZ->Submit();
		cullingCompute->Submit();
	}

	Rasterlet.Draw();
}

HIGHOMEGA::RENDER::ComputeSubmission::ComputeSubmission()
{
	resourcesRequested = 0;
}

void HIGHOMEGA::RENDER::ComputeSubmission::MakeDispatch(InstanceClass & ptrToInstance, const std::string & inpName, ShaderResourceSet & inpShader, int inpGroupX, int inpGroupY, int inpGroupZ)
{
	if (!computeSemInit)
	{
		computelet.Computelet(ptrToInstance);
		computeSemInit = true;
	}
	instanceRef = &ptrToInstance;

	{std::lock_guard<std::mutex> lk(globalCompute_PSO_DSL_Cache_mutex);
	if (globalCompute_PSO_DSL_Cache.find(inpShader.compute_shader) == globalCompute_PSO_DSL_Cache.end())
	{
		globalCompute_PSO_DSL_Cache[inpShader.compute_shader].DSL = new DescriptorSetLayout();
		globalCompute_PSO_DSL_Cache[inpShader.compute_shader].DSL->CreateDescriptorSetLayout(inpShader.getAdditionalResources(), &ptrToInstance);
		globalCompute_PSO_DSL_Cache[inpShader.compute_shader].PSO = new ComputePipelineStateClass(ptrToInstance, *globalCompute_PSO_DSL_Cache[inpShader.compute_shader].DSL, inpShader);
	}
	computePsoDsl = &globalCompute_PSO_DSL_Cache[inpShader.compute_shader]; }

	unsigned int curResourcesRequested = 0;
	for (ShaderResource & curRes : inpShader.getAdditionalResources())
		curResourcesRequested += curRes.ResourceCount();

	if (dispatches.find(inpName) == dispatches.end())
	{
		dispatches[inpName].DS = new DescriptorSets(computePsoDsl->DSL);
		dispatches[inpName].DS->WriteDescriptorSets(inpShader.getAdditionalResources());
	}
	else
	{
		if (curResourcesRequested != resourcesRequested)
			dispatches[inpName].DS->RewriteDescriptorSets(inpShader.getAdditionalResources());
		else
			dispatches[inpName].DS->UpdateDescriptorSets(inpShader.getAdditionalResources());
	}
	resourcesRequested = curResourcesRequested;
	dispatches[inpName].curState = &inpShader;
	dispatches[inpName].groupX = inpGroupX;
	dispatches[inpName].groupY = inpGroupY;
	dispatches[inpName].groupZ = inpGroupZ;

	updateResources = true;
}

void HIGHOMEGA::RENDER::ComputeSubmission::UpdateDispatchSize(InstanceClass & ptrToInstance, const std::string & inpName, int inpGroupX, int inpGroupY, int inpGroupZ)
{
	if (!computeSemInit)
	{
		computelet.Computelet(ptrToInstance);
		computeSemInit = true;
	}
	instanceRef = &ptrToInstance;

	dispatches[inpName].groupX = inpGroupX;
	dispatches[inpName].groupY = inpGroupY;
	dispatches[inpName].groupZ = inpGroupZ;

	updateResources = true;
}

void HIGHOMEGA::RENDER::ComputeSubmission::RemoveDispatch(InstanceClass & ptrToInstance, const std::string & inpName)
{
	if (dispatches.find(inpName) != dispatches.end())
	{
		delete dispatches[inpName].DS;
		dispatches.erase(inpName);
		updateResources = true;
	}
}

void HIGHOMEGA::RENDER::ComputeSubmission::makeAsync()
{
	computelet.makeAsync();
}

void HIGHOMEGA::RENDER::ComputeSubmission::makeSerial()
{
	computelet.makeSerial();
}

void HIGHOMEGA::RENDER::ComputeSubmission::Submit()
{
	if (!instanceRef) return;

	if (updateResources)
	{
		computelet.Start(*computePsoDsl->PSO);
		for (std::pair<const std::string, SingleDispatch> & curDispatch : dispatches)
			computelet.Dispatch(*computePsoDsl->PSO, *curDispatch.second.DS, curDispatch.second.groupX, curDispatch.second.groupY, curDispatch.second.groupZ);
		computelet.End();

		updateResources = false;
	}

	computelet.Submit();
}

HIGHOMEGA::RENDER::ComputeSubmission::~ComputeSubmission()
{
	for (std::pair<const std::string, SingleDispatch> curDispatch : dispatches)
		delete curDispatch.second.DS;
	dispatches.clear();

	updateResources = false; // Just in case
	computeSemInit = false; // The semaphore will self-destruct
}

void HIGHOMEGA::RENDER::FrustumClass::CreatePerspective(vec3 eye, vec3 look, vec3 up, float fov, float whr, float clipNear, float clipFar)
{
	this->eye = eye;
	this->up = up;
	this->look = look;

	screen_fov = fov;
	screen_whr = whr;
	screen_near = clipNear;
	screen_far = clipFar;

	isOrtho = false;
}

void HIGHOMEGA::RENDER::FrustumClass::CreateOrtho(vec3 eye, vec3 look, vec3 up, float left, float right, float bottom, float top, float clipNear, float clipFar)
{
	this->eye = eye;
	this->up = up;
	this->look = look;

	ortho_left = left;
	ortho_right = right;
	ortho_bottom = bottom;
	ortho_top = top;
	screen_near = clipNear;
	screen_far = clipFar;

	isOrtho = true;
}

void HIGHOMEGA::RENDER::FrustumClass::SetView()
{
	vec3 look_norm = look.normalized();
	vec3 side = cross(look_norm, up).normalized();
	vec3 new_up = cross(side, look_norm);

	mat4 orient_matrix;
	orient_matrix.Ident();
	orient_matrix.i[0][0] = side.x;
	orient_matrix.i[1][0] = side.y;
	orient_matrix.i[2][0] = side.z;

	orient_matrix.i[0][1] = new_up.x;
	orient_matrix.i[1][1] = new_up.y;
	orient_matrix.i[2][1] = new_up.z;

	orient_matrix.i[0][2] = -look_norm.x;
	orient_matrix.i[1][2] = -look_norm.y;
	orient_matrix.i[2][2] = -look_norm.z;

	mat4 translate_matrix;
	translate_matrix.Ident();
	translate_matrix.i[3][0] = -eye.x;
	translate_matrix.i[3][1] = -eye.y;
	translate_matrix.i[3][2] = -eye.z;

	modelview_matrix = (translate_matrix * orient_matrix).Transpose();
	modelviewprojection_matrix = projection_matrix * modelview_matrix;
}

void HIGHOMEGA::RENDER::FrustumClass::SetPerspective()
{
	float fovy_2_rad = screen_fov * 0.00872664625f;
	float _f_ = cos(fovy_2_rad) / sin(fovy_2_rad);
	float near_sub_far_inv = 1.0f / (screen_near - screen_far);

	projection_matrix.i[0][0] = _f_ / screen_whr;
	projection_matrix.i[1][0] = 0.0f;
	projection_matrix.i[2][0] = 0.0f;
	projection_matrix.i[3][0] = 0.0f;

	projection_matrix.i[0][1] = 0.0f;
	projection_matrix.i[1][1] = -_f_;
	projection_matrix.i[2][1] = 0.0f;
	projection_matrix.i[3][1] = 0.0f;

	projection_matrix.i[0][2] = 0.0f;
	projection_matrix.i[1][2] = 0.0f;
	projection_matrix.i[2][2] = reverseZ ? -screen_near*near_sub_far_inv : screen_far*near_sub_far_inv;
	projection_matrix.i[3][2] = -1.0f;

	projection_matrix.i[0][3] = 0.0f;
	projection_matrix.i[1][3] = 0.0f;
	projection_matrix.i[2][3] = reverseZ ? (-screen_far*screen_near)*near_sub_far_inv : (screen_far*screen_near)*near_sub_far_inv;
	projection_matrix.i[3][3] = 0.0f;

	modelviewprojection_matrix = projection_matrix * modelview_matrix;
}

void HIGHOMEGA::RENDER::FrustumClass::SetOrtho()
{
	float right_left_inv = 1.0f / (ortho_right - ortho_left);
	float bottom_top_inv = 1.0f / (ortho_bottom - ortho_top);
	float near_far_inv = 1.0f / (screen_near - screen_far);

	projection_matrix.i[0][0] = 2.0f*right_left_inv;
	projection_matrix.i[1][0] = 0.0f;
	projection_matrix.i[2][0] = 0.0f;
	projection_matrix.i[3][0] = 0.0f;

	projection_matrix.i[0][1] = 0.0f;
	projection_matrix.i[1][1] = 2.0f*bottom_top_inv;
	projection_matrix.i[2][1] = 0.0f;
	projection_matrix.i[3][1] = 0.0f;

	projection_matrix.i[0][2] = 0.0f;
	projection_matrix.i[1][2] = 0.0f;
	projection_matrix.i[2][2] = near_far_inv;
	projection_matrix.i[3][2] = 0.0f;

	projection_matrix.i[0][3] = -(ortho_right + ortho_left)*right_left_inv;
	projection_matrix.i[1][3] = -(ortho_top + ortho_bottom)*bottom_top_inv;
	projection_matrix.i[2][3] = screen_near*near_far_inv;
	projection_matrix.i[3][3] = 1.0f;

	modelviewprojection_matrix = projection_matrix * modelview_matrix;
}

void HIGHOMEGA::RENDER::FrustumClass::Update(vec3 eyeInBuffer, vec3 lookInBuffer, vec3 upInBuffer, float whrInBuffer, float fovYForBuffer)
{
	if (!initBuffer)
	{
		Buffer.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &uboData, (unsigned int)sizeof(uboData));
		initBuffer = true;
	}

	SetView();
	if (isOrtho)
	{
		SetOrtho();
	}
	else
	{
		SetPerspective();
	}
	UnpackMat4(modelviewprojection_matrix, &uboData);
	vec3 sideVector = cross(upInBuffer, lookInBuffer).normalized();
	vec3 upVector = cross(lookInBuffer, sideVector);
	if (upVector * upInBuffer < 0.0f) upVector = -upVector;
	uboData.lookEyeX[0] = lookInBuffer.x;
	uboData.lookEyeX[1] = lookInBuffer.y;
	uboData.lookEyeX[2] = lookInBuffer.z;
	uboData.lookEyeX[3] = eyeInBuffer.x;
	uboData.upEyeY[0] = upVector.x;
	uboData.upEyeY[1] = upVector.y;
	uboData.upEyeY[2] = upVector.z;
	uboData.upEyeY[3] = eyeInBuffer.y;
	uboData.sideEyeZ[0] = sideVector.x;
	uboData.sideEyeZ[1] = sideVector.y;
	uboData.sideEyeZ[2] = sideVector.z;
	uboData.sideEyeZ[3] = eyeInBuffer.z;
	uboData.whrTanHalfFovY[0] = whrInBuffer;
	uboData.whrTanHalfFovY[1] = tanf (((fovYForBuffer / 180.0f) * HIGHOMEGA_PI) * 0.5f);

	if (initBuffer)
	{
		Buffer.UploadSubData(0, (void *)&uboData, sizeof(uboData));
	}
}

void HIGHOMEGA::RENDER::FrustumClass::Update()
{
	Update(eye, look, up, screen_whr, screen_fov);
}

void HIGHOMEGA::RENDER::FrustumClass::CopyFromEntitiesThread()
{
	if (smoothingFactor == 0.0f)
		smoothingFactor = 1.0f;
	else
		smoothingFactor = 0.3f;

	look = Slerp(look, MainFrustum.entitiesLook, smoothingFactor);
	eye += (MainFrustum.entitiesEye - eye)*smoothingFactor;
}

void HIGHOMEGA::RENDER::FrustumClass::CopyFromFrustum(FrustumClass & Other)
{
	eye = Other.eye;
	look = Other.look;
	up = Other.up;

	screen_fov = Other.screen_fov;
	screen_whr = Other.screen_whr;
	screen_near = Other.screen_near;
	screen_far = Other.screen_far;

	ortho_left = Other.ortho_left;
	ortho_right = Other.ortho_right;
	ortho_bottom = Other.ortho_bottom;
	ortho_top = Other.ortho_top;
	isOrtho = Other.isOrtho;
	reverseZ = Other.reverseZ;
}

void HIGHOMEGA::RENDER::FrustumClass::ForceEyeAndLook(vec3 inpEye, vec3 inpLook)
{
	look = inpLook;
	eye = inpEye;
}

HIGHOMEGA::RENDER::ScreenSizeClass::ScreenSizeClass()
{
}

void HIGHOMEGA::RENDER::ScreenSizeClass::Create(unsigned int width, unsigned int height)
{
	this->width = width;
	this->height = height;
}

void HIGHOMEGA::RENDER::PASSES::TriClass::Create(vec3 eyeInBuffer, vec3 lookInBuffer, vec3 upInBuffer, float whrInBuffer, float fovYForBuffer)
{
	Mesh triMesh = Mesh("assets/models/tri/tri.3md");
	triModel.Model(triMesh, "assets/models/tri/", Instance);

	triFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, 1.0f, 1.0f, 10.0f);

	UpdateViewSpace(eyeInBuffer, lookInBuffer, upInBuffer, whrInBuffer, fovYForBuffer);
}

void HIGHOMEGA::RENDER::PASSES::TriClass::UpdateViewSpace(vec3 eyeInBuffer, vec3 lookInBuffer, vec3 upInBuffer, float whrInBuffer, float fovYForBuffer)
{
	triFrustum.Update(eyeInBuffer, lookInBuffer, upInBuffer, whrInBuffer, fovYForBuffer);
}

void HIGHOMEGA::RENDER::PASSES::BoxClass::Create()
{
	Mesh boxMesh = Mesh("assets/models/tri/tri.3md");
	boxModel.Model(boxMesh, "source_material/dev_test_models/boxes/", Instance);
}

unsigned long long HIGHOMEGA::RENDER::WorldParamsClass::Populate(Mesh & inpMesh)
{
	unsigned long long curId = mersenneTwister64BitPRNG();

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		float tmpFloat;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "sceneProps", tmpFloat)) continue;

		if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", allItems[curId].startPlayerPos)) throw std::runtime_error("Start player pos not found");
		allItems[curId].firstPersonControls = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "firstPersonControls", tmpFloat);
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "lightShaftAmount", allItems[curId].lightShaftAmount)) allItems[curId].lightShaftAmount = 0.0f;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "lightShaftExtinction", allItems[curId].lightShaftExtinction)) allItems[curId].lightShaftExtinction = 0.98f;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "sunDirectLightStrength", allItems[curId].sunDirectLightStrength)) allItems[curId].sunDirectLightStrength = 1.0f;

		allItems[curId].sunAngle = 0.0f;
		allItems[curId].lastFrameTime = 0.0f;
		allItems[curId].showAurora = false;
		allItems[curId].forceSunAngle = false;
		allItems[curId].renderInfo.time[0] = 0.0f;
		allItems[curId].renderTimeBuffer = nullptr;
		break;
	}

	return curId;
}

void HIGHOMEGA::RENDER::WorldParamsClass::ClearContent()
{
	for (std::pair<const unsigned long long, Parameters> & curItem : allItems)
		if (curItem.second.renderTimeBuffer) delete curItem.second.renderTimeBuffer;
	allItems.clear();
}

void HIGHOMEGA::RENDER::WorldParamsClass::Combine(std::vector<WorldParamsClass>& inpWorldParamSystems)
{
	if (allItems.size() == 0)
		for (WorldParamsClass & inpParams : inpWorldParamSystems)
			if (inpParams.allItems.size() > 0)
			{
				allItems = inpParams.allItems;
				break;
			}

	for (WorldParamsClass & inpParams : inpWorldParamSystems)
		inpParams.ClearContent();
}

void HIGHOMEGA::RENDER::WorldParamsClass::Remove(unsigned long long inpId)
{
	if (allItems.find(inpId) == allItems.end()) return;
	if (allItems.size() == 1) return; // Don't remove the last one...

	if (allItems[inpId].renderTimeBuffer) delete allItems[inpId].renderTimeBuffer;

	allItems.erase(inpId);
}

void HIGHOMEGA::RENDER::WorldParamsClass::StartFrameTimer()
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.frameTimer.Start();
}

void HIGHOMEGA::RENDER::WorldParamsClass::EndFrameTimer()
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.lastFrameTime = (float)allItems.begin()->second.frameTimer.Diff();
}

float HIGHOMEGA::RENDER::WorldParamsClass::GetFrameTime()
{
	if (allItems.size() == 0) return 0.0f;
	return allItems.begin()->second.lastFrameTime;
}

void HIGHOMEGA::RENDER::WorldParamsClass::AddSunAngle(float sunAngle)
{
	if (allItems.size() == 0) return ;
	if (allItems.begin()->second.forceSunAngle) return;
	allItems.begin()->second.sunAngle += sunAngle;
}

void HIGHOMEGA::RENDER::WorldParamsClass::ForceSunAngle(float sunAngle)
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.forceSunAngle = true;
	allItems.begin()->second.sunAngle = sunAngle;
}

void HIGHOMEGA::RENDER::WorldParamsClass::UnforceSunAngle()
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.forceSunAngle = false;
}

void HIGHOMEGA::RENDER::WorldParamsClass::AddRenderTime(float addRenderTime)
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.renderInfo.time[0] += addRenderTime;
	Parameters & ourParams = allItems.begin()->second;
	if (!ourParams.renderTimeBuffer)
		ourParams.renderTimeBuffer = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &ourParams.renderInfo, (unsigned int)sizeof(ourParams.renderInfo));
	else
		ourParams.renderTimeBuffer->UploadSubData(0, &ourParams.renderInfo, (unsigned int)sizeof(ourParams.renderInfo));
}

void HIGHOMEGA::RENDER::WorldParamsClass::ShowAurora()
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.showAurora = true;
}

void HIGHOMEGA::RENDER::WorldParamsClass::ShowClouds()
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.showAurora = false;
}

bool HIGHOMEGA::RENDER::WorldParamsClass::isShowingAurora()
{
	if (allItems.size() == 0) return false;
	return allItems.begin()->second.showAurora;
}

float HIGHOMEGA::RENDER::WorldParamsClass::GetSunAngle()
{
	if (allItems.size() == 0) return 0.5f * HIGHOMEGA_PI;
	//return 0.25f * HIGHOMEGA_PI;
	return allItems.begin()->second.sunAngle;
}

vec3 HIGHOMEGA::RENDER::WorldParamsClass::SunDir()
{
	if (allItems.size() == 0) return vec3 (0.0f, 1.0f, 0.0f);
	return vec3(cosf(GetSunAngle()), sinf(GetSunAngle()), 0.0f).normalized();
}

vec3 HIGHOMEGA::RENDER::WorldParamsClass::SunOrMoonDir()
{
	if (allItems.size() == 0) return vec3(0.0f, 1.0f, 0.0f);
	vec3 retVal = SunDir();
	if (retVal.y < 0.0f) retVal = -retVal;
	return retVal;
}

bool HIGHOMEGA::RENDER::WorldParamsClass::FirstPersonControls()
{
	if (allItems.size() == 0) return true;
	return allItems.begin()->second.firstPersonControls;
}

vec3 HIGHOMEGA::RENDER::WorldParamsClass::StartPlayerPos()
{
	if (allItems.size() == 0) return vec3(0.0f);
	return allItems.begin()->second.startPlayerPos;
}

float HIGHOMEGA::RENDER::WorldParamsClass::GetLightShaftAmount()
{
	if (allItems.size() == 0) return 0.0f;
	return allItems.begin()->second.lightShaftAmount;
}

float HIGHOMEGA::RENDER::WorldParamsClass::GetLightShaftExtinction()
{
	if (allItems.size() == 0) return 0.0f;
	return allItems.begin()->second.lightShaftExtinction;
}

float HIGHOMEGA::RENDER::WorldParamsClass::GetSunDirectLightStrength()
{
	if (allItems.size() == 0) return 1.0f;
	return allItems.begin()->second.sunDirectLightStrength;
}

BufferClass & HIGHOMEGA::RENDER::WorldParamsClass::GetRenderTimeBuffer()
{
	if (allItems.size() == 0) throw std::runtime_error("No sceneProps to provide a renderTimeBuffer for");
	Parameters & ourParams = allItems.begin()->second;
	if (!ourParams.renderTimeBuffer)
		ourParams.renderTimeBuffer = new BufferClass(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &ourParams.renderInfo, (unsigned int)sizeof(ourParams.renderInfo));
	else
		ourParams.renderTimeBuffer->UploadSubData(0, &ourParams.renderInfo, (unsigned int)sizeof(ourParams.renderInfo));
	return *(allItems.begin()->second.renderTimeBuffer);
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::ComputeOptimalOrthoFrustum(vec3 viewMin, vec3 viewMax, vec3 toSkyObject, FrustumClass & frustum)
{
	vec3 mapCenter = (viewMin + viewMax) * 0.5f;
	vec3 sunEye = mapCenter;
	vec3 sunLook = -toSkyObject;
	vec3 sunUp = cross(cross(sunLook, vec3(0.0f, 1.0f, 0.0f)), sunLook).normalized();
	if (sunUp * vec3(0.0f, 1.0f, 0.0f) < 0.0f) sunUp = -sunUp;
	vec3 sunSide = cross(sunLook, sunUp);

	std::array<vec3, 8> eightCorners = {
		vec3(viewMax.x, viewMax.y, viewMax.z),
		vec3(viewMin.x, viewMax.y, viewMax.z),
		vec3(viewMax.x, viewMin.y, viewMax.z),
		vec3(viewMin.x, viewMin.y, viewMax.z),
		vec3(viewMax.x, viewMax.y, viewMin.z),
		vec3(viewMin.x, viewMax.y, viewMin.z),
		vec3(viewMax.x, viewMin.y, viewMin.z),
		vec3(viewMin.x, viewMin.y, viewMin.z)
	};

	float maxTopDist = -10000000000000.0f;
	float maxSideDist = -10000000000000.0f;
	float maxFrontDist = -10000000000000.0f;

	for (int i = 0; i != 8; i++)
	{
		float curDist = fabs((eightCorners[i] - mapCenter) * sunUp);
		if (curDist > maxTopDist)
			maxTopDist = curDist;
	}
	for (int i = 0; i != 8; i++)
	{
		float curDist = fabs((eightCorners[i] - mapCenter) * sunSide);
		if (curDist > maxSideDist)
			maxSideDist = curDist;
	}
	for (int i = 0; i != 8; i++)
	{
		float curDist = fabs((eightCorners[i] - mapCenter) * sunLook);
		if (curDist > maxFrontDist)
			maxFrontDist = curDist;
	}

	float left = -maxSideDist - shadowMapSideBias * 0.5f;
	float right = maxSideDist + shadowMapSideBias * 0.5f;
	float bottom = -maxTopDist - shadowMapSideBias * 0.5f;
	float top = maxTopDist + shadowMapSideBias * 0.5f;
	float back = 0.0f;
	float front = 20000.0f;

	sunEye += toSkyObject * 10000.0f;

	frustum.CreateOrtho(sunEye,
		sunLook,
		sunUp,
		left,
		right,
		bottom,
		top,
		back,
		front);
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::Create(WorldParamsClass & WorldParams)
{
	ptrWorldParams = &WorldParams;

	submission.SetClearColor(vec3(0.0f, 0.0f, 0.0f), 0.0f);
	submission.Create(Instance);

	shadowColorAttach.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, 512, 512, true, true);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, 512, 512, ImageClass::SAMPLE_DEPTH);

	frameBuffer.AddColorAttachment(shadowColorAttach);
	frameBuffer.SetDepthStencil(depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	PipelineFlags defaultPipelineFlags;
	defaultPipelineFlags.backFaceCulling = true;
	defaultPipelineFlags.redMask = false;
	defaultPipelineFlags.greenMask = false;
	defaultPipelineFlags.blueMask = false;
	defaultPipelineFlags.alphaMask = false;
	submission.SetDefaultPipelineFlags(defaultPipelineFlags);
	submission.matTransform = [](const MeshMaterial& inMat) ->MeshMaterial {
		MeshMaterial retMat = inMat;

		if (!inMat.postProcess) {
			if (inMat.isAlphaKeyed) {
				retMat.shaderName = "shadowAlpha";
			}
		}
		else {
			retMat.pipelineFlags.depthWrite = false;

			retMat.pipelineFlags.redMask = true;
			retMat.pipelineFlags.greenMask = true;
			retMat.pipelineFlags.blueMask = true;
			retMat.pipelineFlags.alphaMask = true;

			retMat.pipelineFlags.blendEnable = true;
			retMat.pipelineFlags.alphaBlending = true;
			retMat.pipelineFlags.colorBlending = true;
			retMat.pipelineFlags.srcAlphaFactor = FACTOR_ONE;
			retMat.pipelineFlags.dstAlphaFactor = FACTOR_ZERO;
			retMat.pipelineFlags.srcColorFactor = FACTOR_SRC_ALPHA;
			retMat.pipelineFlags.dstColorFactor = FACTOR_ONE;
			retMat.pipelineFlags.alphaBlendOp = BLEND_ADD;
			retMat.pipelineFlags.colorBlendOp = BLEND_ADD;

			retMat.pipelineFlags.changedDepthWrite = true;
			retMat.pipelineFlags.changedColorMask = true;
			retMat.pipelineFlags.changedBlendEnable = true;
			if (inMat.shaderName == "shaderTessScreenSpace") {
				retMat.shaderName = "shadowStainedGlassTess";
			}
			else {
				retMat.shaderName = "shadowStainedGlass";
			}
		}

		return retMat;
	};
	submission.SetFrameBuffer(frameBuffer);

	shader.Create("shaders/shadow.vert.spv", "main", "shaders/shadow.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, frustum.Buffer);
	shaderAlpha.Create("shaders/shadow.vert.spv", "main", "shaders/shadowAlpha.frag.spv", "main");
	shaderAlpha.AddResource(RESOURCE_UBO, VERTEX, 0, 0, frustum.Buffer);
	shaderStainedGlass.Create("shaders/shadow.vert.spv", "main", "shaders/shadowStainedGlass.frag.spv", "main");
	shaderStainedGlass.AddResource(RESOURCE_UBO, VERTEX, 0, 0, frustum.Buffer);
	shaderStainedGlassTess.Create("shaders/shadow.vert.spv", "main", "shaders/shadow.tesc.spv", "main", "shaders/shadow.tese.spv", "main", "shaders/shadowStainedGlass.frag.spv", "main");
	shaderStainedGlassTess.AddResource(RESOURCE_UBO, VERTEX | TESS_EVAL, 0, 0, frustum.Buffer);
	shaderStainedGlassTess.AddResource(RESOURCE_UBO, TESS_EVAL, 0, 1, WorldParams.GetRenderTimeBuffer());
	submission.SetShader("default", shader);
	submission.SetShader("shadowAlpha", shaderAlpha);
	submission.SetShader("shadowStainedGlass", shaderStainedGlass);
	submission.SetShader("shadowStainedGlassTess", shaderStainedGlassTess);
	submission.makeAsync();
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::Render(vec3 viewMin, vec3 viewMax, float shadowMapSideBias, vec2 samplingBiasMinMax, FrustumClass *nearCascadeFrustum)
{
	if (nearCascadeFrustum)
	{
		vec3 viewSideLenScaled = (viewMax - viewMin) * 0.05f;
		viewSideLenScaled.x = min(viewSideLenScaled.x, 100.0f);
		viewSideLenScaled.y = min(viewSideLenScaled.y, 100.0f);
		viewSideLenScaled.z = min(viewSideLenScaled.z, 100.0f);
		vec3 projectedLook = vec3(nearCascadeFrustum->look.x, 0.0f, nearCascadeFrustum->look.z).normalized() * vec3 (viewSideLenScaled.x, 0.0f, viewSideLenScaled.y).length();
		this->viewMax = nearCascadeFrustum->eye - vec3(0.0f, 18.0f, 0.0f) + viewSideLenScaled + projectedLook;
		this->viewMin = nearCascadeFrustum->eye - vec3(0.0f, 18.0f, 0.0f) - viewSideLenScaled + projectedLook;
	}
	else
	{
		this->viewMax = viewMax;
		this->viewMin = viewMin;
	}
	samplingBias = samplingBiasMinMax;
	this->shadowMapSideBias = shadowMapSideBias;
	ComputeOptimalOrthoFrustum(this->viewMin, this->viewMax, ptrWorldParams->SunOrMoonDir(), frustum);
	frustum.Update();

	submission.doCulling(frustum, nearCascadeFrustum ? GroupedRasterSubmission::CULL_MODE::FRUSTUM_HIZ : GroupedRasterSubmission::CULL_MODE::HIZ_ONLY);
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::makeAsync()
{
	submission.makeAsync();
	if (RTInstance::Enabled() && useRTIfAvailableCached) tracelet.makeAsync();
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::makeSerial()
{
	submission.makeSerial();
	if (RTInstance::Enabled() && useRTIfAvailableCached) tracelet.makeSerial();
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::Create(TriClass & PostProcessTri, ShadowMapClass & ShadowMapCascadeNear, ShadowMapClass & ShadowMapCascadeFar, GatherPassClassCommon & GatherPass, bool useHWRTIfAvailable)
{
	gatherPassCommonRef = &GatherPass;
	shadowMapNearRef = &ShadowMapCascadeNear;
	shadowMapFarRef = &ShadowMapCascadeFar;
	useRTIfAvailableCached = useHWRTIfAvailable;
	shadowPropsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &shadowProps, (unsigned int)sizeof(shadowProps));

	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	shadowMapScreen.CreateImageStore(Instance, R8G8B8A8UN, GatherPass.worldPosAttach.getWidth(), GatherPass.worldPosAttach.getHeight(), 1, _2D, false);

	if (RTInstance::Enabled() && useHWRTIfAvailable)
	{
		rtShaderResourceSet.CreateRT("shaders/rtshadow.rgen.spv", "main", "shaders/rtshadow.rchit.spv", "main", "shaders/rtshadow.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
		tracelet.Make(Instance);
	}
	else
	{
		frameBuffer.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBuffer.setHeight(GatherPass.worldPosAttach.getHeight());
		PipelineFlags shadowScreenPF;
		shadowScreenPF.depthTest = shadowScreenPF.depthWrite = false;
		submission.SetDefaultPipelineFlags(shadowScreenPF);
		frameBuffer.Create(OFF_SCREEN, Instance, Window);
		submission.SetFrameBuffer(frameBuffer);

		shader.Create("shaders/postprocess.vert.spv", "main", "shaders/shadowScreen.frag.spv", "main");

		shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.normalAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ShadowMapCascadeNear.depthStencilAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, ShadowMapCascadeNear.shadowColorAttach);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, ShadowMapCascadeNear.frustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, ShadowMapCascadeFar.depthStencilAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, ShadowMapCascadeFar.shadowColorAttach);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, ShadowMapCascadeFar.frustum.Buffer);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 9, shadowPropsBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 10, shadowMapScreen);

		submission.SetShader("default", shader);
	}

	submissionH.Add(PostProcessTri.triModel);
	submissionH.Create(Instance);
	submissionV.Add(PostProcessTri.triModel);
	submissionV.Create(Instance);

	blurHAttach.CreateImageStore(Instance, R8G8B8A8UN, GatherPass.worldPosAttach.getWidth(), GatherPass.worldPosAttach.getHeight(), 1, _2D, false);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;
	frameBufferH.setWidth(GatherPass.worldPosAttach.getWidth());
	frameBufferH.setHeight(GatherPass.worldPosAttach.getHeight());
	frameBufferH.Create(OFF_SCREEN, Instance, Window);
	frameBufferV.setWidth(GatherPass.worldPosAttach.getWidth());
	frameBufferV.setHeight(GatherPass.worldPosAttach.getHeight());
	frameBufferV.Create(OFF_SCREEN, Instance, Window);
	submissionH.SetFrameBuffer(frameBufferH);
	submissionV.SetFrameBuffer(frameBufferV);
	submissionH.SetDefaultPipelineFlags(defPipelineFlags);
	submissionV.SetDefaultPipelineFlags(defPipelineFlags);

	shaderH.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSep.frag.spv", "main");
	shaderV.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSep.frag.spv", "main");

	shaderH.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderH.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, shadowMapScreen);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, blurHAttach);
	shaderH.AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, shadowPropsBuf);

	shaderV.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderV.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, blurHAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, shadowMapScreen);
	shaderV.AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, shadowPropsBuf);

	submissionH.SetShader("default", shaderH);

	submissionV.SetShader("default", shaderV);
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::Render(GroupedTraceSubmission & rtSubmission)
{
	if (RTInstance::Enabled() && useRTIfAvailableCached)
	{
		RTScene & rtSceneRef = rtSubmission.rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = rtSubmission.SceneID();
		if (curSceneId != lastSceneId)
		{
			lastSceneId = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources.clear();
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 1, gatherPassCommonRef->worldPosAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 2, shadowMapScreen);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_MISS, 0, 3, shadowMapNearRef->shadowColorAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_MISS | RT_RAYGEN, 0, 4, shadowMapNearRef->frustum.Buffer);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_MISS, 0, 5, shadowMapFarRef->shadowColorAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_MISS, 0, 6, shadowMapFarRef->frustum.Buffer);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_ANYHIT, 1, rtSceneRef.getGeomResources());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_ANYHIT, 2, rtSceneRef.getMaterialResources());
		}

		tracelet.Submit(gatherPassCommonRef->worldPosAttach.getWidth(), gatherPassCommonRef->worldPosAttach.getHeight(), 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
	}
	else
	{
		shadowProps.blurDirectionNearMinBias = shadowMapNearRef->samplingBias.x;
		shadowProps.nearMaxBias = shadowMapNearRef->samplingBias.y;
		shadowProps.farMinBias = shadowMapFarRef->samplingBias.x;
		shadowProps.farMaxBias = shadowMapFarRef->samplingBias.y;
		shadowPropsBuf.UploadSubData(0, &shadowProps, sizeof(shadowProps));

		submission.Render();
	}

	shadowProps.blurDirectionNearMinBias = 0.0f;
	shadowPropsBuf.UploadSubData(0, &shadowProps, sizeof(shadowProps));

	submissionH.Render();

	shadowProps.blurDirectionNearMinBias = 1.0f;
	shadowPropsBuf.UploadSubData(0, &shadowProps, sizeof(shadowProps));

	submissionV.Render();
}

unsigned int HIGHOMEGA::RENDER::PASSES::SkyDomeClass::GetBackdropWidth()
{
	return 640;
}

unsigned int HIGHOMEGA::RENDER::PASSES::SkyDomeClass::GetBackdropHeight()
{
	return 480;
}

unsigned int HIGHOMEGA::RENDER::PASSES::SkyDomeClass::GetCubeResolution()
{
	return 128;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::InnerRad()
{
	return 9.75f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::InnerCloudRad()
{
	return 9.85f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::OuterRad()
{
	return 10.0f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::HeightOfAvgDensity()
{
	return 0.125f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::SunExp()
{
	return 10.0f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::ScatteringCoeff()
{
	return 10.0f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::TransmissionCoeff()
{
	return 0.01f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::AmbientCoeff()
{
	return 20.0f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::NumStepsToSky()
{
	return 64.0f;
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::NumStepsToSun()
{
	return 6.0f;
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::SkyBlue()
{
	return vec3(0.466666f, 0.5819604f, 0.6917645f);
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::GroundDirtColor()
{
	return vec3(0.54f, 0.27f, 0.07f);
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::AuroraGreen()
{
	return vec3(0.1f, 0.2f, 0.1f);
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::SunOrange()
{
	return vec3(1.0f, 0.247f, 0.0f) * 5.0f;
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::SunWhite()
{
	return vec3(1.0f) * ptrWorldParams->GetSunDirectLightStrength();
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::MoonLight()
{
	if (ptrWorldParams->isShowingAurora())
		return AuroraGreen () * 0.75f;
	else
		return NebulaBlue ();
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::NebulaBlue()
{
	return vec3(0.12f, 0.12f, 0.2f);
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::AddSkyColor()
{
	if (ptrWorldParams->isShowingAurora())
	{
		return AuroraGreen () * 0.5f;
	}
	else
	{
		if (ptrWorldParams->SunDir().y > 0.0f)
		{
			return sqrt(max(ptrWorldParams->SunDir().y, 0.0f)) * SkyBlue();
		}
		else
		{
			return sqrt(max(-ptrWorldParams->SunDir().y, 0.0f)) * NebulaBlue();
		}
	}
}

float HIGHOMEGA::RENDER::PASSES::SkyDomeClass::NightAmount()
{
	if (ptrWorldParams->SunDir().y >= 0.0f)
	{
		return 0.0f;
	}
	else if ( ptrWorldParams->SunDir().y > -0.2f && ptrWorldParams->SunDir().y <= 0.0f )
	{
		return sqrt(-ptrWorldParams->SunDir().y * 5.0f);
	}
	else
	{
		return 1.0f;
	}
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::SkyObjectLight()
{
	if (ptrWorldParams->SunDir().y > 0.2f)
	{
		return SunWhite();
	}
	else if (ptrWorldParams->SunDir().y > 0.0f && ptrWorldParams->SunDir().y < 0.2f)
	{
		float lightIntensity = sqrt (ptrWorldParams->SunDir().y * 5.0f);
		return Lerp (SunOrange() * lightIntensity, SunWhite(), lightIntensity);
	}
	else if (ptrWorldParams->SunDir().y > -0.2f && ptrWorldParams->SunDir().y < 0.0f)
	{
		float lightIntensity = sqrt(-ptrWorldParams->SunDir().y * 5.0f);
		return MoonLight() * lightIntensity;
	}
	else
	{
		return MoonLight();
	}
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::HorizonColor()
{
	return max(ptrWorldParams->SunDir().y, 0.0f) * SkyBlue();
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::ApproxGroundColor()
{
	return max(ptrWorldParams->SunDir().y, 0.0f) * GroundDirtColor();
}

void HIGHOMEGA::RENDER::PASSES::SkyDomeClass::UpdateSkyInfo(bool uploadToo)
{
	rayleighMieInfo.lightDir[0] = ptrWorldParams->SunOrMoonDir().x;
	rayleighMieInfo.lightDir[1] = ptrWorldParams->SunOrMoonDir().y;
	rayleighMieInfo.lightDir[2] = ptrWorldParams->SunOrMoonDir().z;
	rayleighMieInfo.sunDirAndExp[0] = ptrWorldParams->SunDir().x;
	rayleighMieInfo.sunDirAndExp[1] = ptrWorldParams->SunDir().y;
	rayleighMieInfo.sunDirAndExp[2] = ptrWorldParams->SunDir().z;
	rayleighMieInfo.sunDirAndExp[3] = SunExp();
	rayleighMieInfo.invW4InnerRad[0] = 5.602044f;
	rayleighMieInfo.invW4InnerRad[1] = 9.473284f;
	rayleighMieInfo.invW4InnerRad[2] = 19.643802f;
	rayleighMieInfo.invW4InnerRad[3] = InnerRad();

	rayleighMieInfo.innerCloudRad = InnerCloudRad();
	rayleighMieInfo.outerRad = OuterRad();
	rayleighMieInfo.scaleDepth = HeightOfAvgDensity();
	rayleighMieInfo.scaleOverScaleDepth = (1.0f / max(OuterRad() - InnerRad(), 0.01f)) / rayleighMieInfo.scaleDepth;

	rayleighMieInfo.scatteringCoeff = ScatteringCoeff();
	rayleighMieInfo.transmissionCoeff = TransmissionCoeff();
	rayleighMieInfo.ambientCoeff = AmbientCoeff();
	rayleighMieInfo.stepSizeEvalLightDir = (1.0f / NumStepsToSun()) * 0.4f;

	rayleighMieInfo.stepSizeToSky = 1.0f / NumStepsToSky();
	rayleighMieInfo.numStepsToSky = NumStepsToSky();
	rayleighMieInfo.stepSizeToSun = 1.0f / NumStepsToSun();
	rayleighMieInfo.numStepsToSun = NumStepsToSun();

	rayleighMieInfo.addSkyColorAndNightAmount[0] = AddSkyColor().x;
	rayleighMieInfo.addSkyColorAndNightAmount[1] = AddSkyColor().y;
	rayleighMieInfo.addSkyColorAndNightAmount[2] = AddSkyColor().z;
	rayleighMieInfo.addSkyColorAndNightAmount[3] = NightAmount();
	rayleighMieInfo.skyObjectLightAndAngle[0] = SkyObjectLight().x;
	rayleighMieInfo.skyObjectLightAndAngle[1] = SkyObjectLight().y;
	rayleighMieInfo.skyObjectLightAndAngle[2] = SkyObjectLight().z;
	rayleighMieInfo.skyObjectLightAndAngle[3] = ptrWorldParams->GetSunAngle();
	rayleighMieInfo.horizonColor[0] = HorizonColor().x;
	rayleighMieInfo.horizonColor[1] = HorizonColor().y;
	rayleighMieInfo.horizonColor[2] = HorizonColor().z;
	rayleighMieInfo.approxGroundColor[0] = ApproxGroundColor().x;
	rayleighMieInfo.approxGroundColor[1] = ApproxGroundColor().y;
	rayleighMieInfo.approxGroundColor[2] = ApproxGroundColor().z;

	if (uploadToo) {
		rayleighMieBuf.UploadSubData(0, &rayleighMieInfo, sizeof(rayleighMieInfo));
	}
}

void HIGHOMEGA::RENDER::PASSES::SkyDomeClass::Create(TriClass & Tri, WorldParamsClass & WorldParams)
{
	ptrWorldParams = &WorldParams;

	Mesh domeMesh = Mesh("assets/models/skydome/skydome.3md");
	Mesh mountainsMesh = Mesh("assets/models/mountains/mountains.3md");
	skyDome.Model(domeMesh, "assets/models/skydome/", Instance);
	mountains.Model(mountainsMesh, "assets/models/mountains/", Instance);

	PerlinNoise pn(128, 128, 128, vec3(0.1f), 6);
	WorleyNoise wn(128, 128, 128, 10, true, 6);
	WorleyNoise wn1(128, 128, 128, 8, true);
	WorleyNoise wn2(128, 128, 128, 6, true);
	WorleyNoise wn3(128, 128, 128, 4, true);
	WorleyNoise wns1(128, 128, 128, 6, true);
	WorleyNoise wns2(128, 128, 128, 4, true);
	WorleyNoise wns3(128, 128, 128, 2, true);
	PerlinWorley pw(&pn, &wn);
	NoiseGenerator noiseGen(pw, wn1, wn2, wn3, "cache/cloudpw.noise");
	NoiseGenerator noiseGen2(wns1, wns2, wns3, wns3, "cache/cloudwhf.noise");

	moonImg.CreateTexture(Instance, "assets/textures/", "moon.tga");
	nebulaImg.CreateTexture(Instance, "assets/textures/", "nebula.tga");
	noiseImg.CreateTexture(Instance, noiseGen.width, noiseGen.height, noiseGen.vals, noiseGen.depth, true, false, false, false);
	noiseImg2.CreateTexture(Instance, noiseGen2.width, noiseGen2.height, noiseGen2.vals, noiseGen2.depth, true, false, false, false);

	UpdateSkyInfo();

	distantGeomShadowMapNear.submission.Add(mountains);
	distantGeomShadowMapFar.submission.Add(mountains);
	distantGeomShadowMapNear.Create(WorldParams);
	distantGeomShadowMapFar.Create(WorldParams);

	SkyDomeParams.invDims[0] = SkyDomeParams.invDims[1] = 1.0f / GetCubeResolution();
	FullResSkyDomeParams.invDims[0] = 1.0f / GetBackdropWidth();
	FullResSkyDomeParams.invDims[1] = 1.0f / GetBackdropHeight();
	SkyDomeParams.doAurora = FullResSkyDomeParams.doAurora = 0.0f;

	rayleighMieBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &rayleighMieInfo, (unsigned int)sizeof(rayleighMieInfo));
	skyDomeParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &SkyDomeParams, (unsigned int)sizeof(SkyDomeParams));
	fullResSkyDomeParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &FullResSkyDomeParams, (unsigned int)sizeof(FullResSkyDomeParams));

	skyCubeMap.CreateCubeMap(Instance, R16G16B16A16F, GetCubeResolution(), GetCubeResolution());
	fullCubeMap.CreateCubeMap(Instance, R16G16B16A16F, GetCubeResolution(), GetCubeResolution());
	skyBackDrop.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, GetBackdropWidth(), GetBackdropHeight(), true, false);
	fullBackDrop.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, ScreenSize.width, ScreenSize.height, false, false);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, skyCubeMap.getWidth(), skyCubeMap.getHeight());
	backdropDS.depthStencilAttach.CreateOffScreenDepthStencil(Instance, skyBackDrop.getWidth(), skyBackDrop.getWidth());

	for (int i = 0; i != 7; i++)
	{
		vec3 skyDomeLook, skyDomeUp;
		if (i < 6) {
			if (i == 0) {
				skyDomeLook = vec3(1.0f, 0.0f, 0.0f);
				skyDomeUp = vec3(0.0f, 1.0f, 0.0f);
			}
			else if (i == 1) {
				skyDomeLook = vec3(-1.0f, 0.0f, 0.0f);
				skyDomeUp = vec3(0.0f, 1.0f, 0.0f);
			}
			else if (i == 2) {
				skyDomeLook = vec3(0.0f, 1.0f, 0.0f);
				skyDomeUp = vec3(0.0f, 0.0f, 1.0f);
			}
			else if (i == 3) {
				skyDomeLook = vec3(0.0f, -1.0f, 0.0f);
				skyDomeUp = vec3(0.0f, 0.0f, -1.0f);
			}
			else if (i == 4) {
				skyDomeLook = vec3(0.0f, 0.0f, -1.0f);
				skyDomeUp = vec3(0.0f, 1.0f, 0.0f);
			}
			else if (i == 5) {
				skyDomeLook = vec3(0.0f, 0.0f, 1.0f);
				skyDomeUp = vec3(0.0f, 1.0f, 0.0f);
			}
			skyFrustums[i].CreatePerspective(vec3(0.0f, InnerRad(), 0.0f), skyDomeLook, skyDomeUp, 90.0f, 1.0f, 0.1f, 10.0f);
			skyFrustums[i].Update();
		}
		else {
			skyFrustums[i].CopyFromFrustum(MainFrustum);
			skyFrustums[i].eye = vec3(0.0f, InnerRad(), 0.0f);
			skyFrustums[i].Update();
		}

		distantVis[i].Create(i < 6 ? skyCubeMap.getWidth() : ScreenSize.width, i < 6 ? skyCubeMap.getHeight() : ScreenSize.height, skyFrustums[i]);
		distantVis[i].submission.Add(mountains);
		distantGather[i].Create(distantVis[i], Tri, skyFrustums[i], true);

		distantGeomScreenShadow[i].Create(Tri, distantGeomShadowMapNear, distantGeomShadowMapFar, distantGather[i]);

		skyShaders[i].Create("shaders/skyDomeRender.vert.spv", "main", "shaders/skyDomeRender.frag.spv", "main");
		skyShaders[i].AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, skyFrustums[i].Buffer);
		skyShaders[i].AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 1, rayleighMieBuf);
		skyShaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 2, i < 6 ? skyDomeParamsBuf : fullResSkyDomeParamsBuf);
		skyShaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 3, WorldParams.GetRenderTimeBuffer());
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, noiseImg);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, noiseImg2);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, moonImg);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, nebulaImg);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, distantGather[i].worldPosAttach);

		skySubmissions[i].Add(skyDome);
		skySubmissions[i].Create(Instance);

		if (i < 6) {
			skyFrameBuffers[i].AddColorAttachmentWithLayer(skyCubeMap, i);
			skyFrameBuffers[i].SetDepthStencil(depthStencilAttach);
		}
		else {
			skyFrameBuffers[i].AddColorAttachment(skyBackDrop);
			skyFrameBuffers[i].SetDepthStencil(backdropDS.depthStencilAttach);
		}
		skyFrameBuffers[i].Create(OFF_SCREEN, Instance, Window);
		skySubmissions[i].SetFrameBuffer(skyFrameBuffers[i]);
		skySubmissions[i].makeAsync();
		skySubmissions[i].SetShader("default", skyShaders[i]);

		skyBoxCompositionParams.mode = i;
		skyBoxCompositionParamsBuf[i].Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &skyBoxCompositionParams, (unsigned int)sizeof(skyBoxCompositionParams));

		shaders[i].Create("shaders/postprocess.vert.spv", "main", "shaders/skyBoxComposition.frag.spv", "main");
		shaders[i].AddResource(RESOURCE_UBO, VERTEX, 0, 0, Tri.triFrustum.Buffer);
		shaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, skyFrustums[i].Buffer);
		shaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 2, rayleighMieBuf);
		shaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 3, i < 6 ? skyDomeParamsBuf : fullResSkyDomeParamsBuf);
		shaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, WorldParams.GetRenderTimeBuffer());
		shaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, skyBoxCompositionParamsBuf[i]);
		shaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, skyCubeMap);
		shaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, skyBackDrop);
		shaders[i].AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 8, distantGeomScreenShadow[i].shadowMapScreen);
		shaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, distantGather[i].worldPosAttach);
		shaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 10, distantGather[i].normalAttach);

		submissions[i].Add(Tri.triModel);
		submissions[i].Create(Instance);
		if (i < 6) {
			frameBuffers[i].AddColorAttachmentWithLayer(fullCubeMap, i);
		}
		else {
			frameBuffers[i].AddColorAttachment(fullBackDrop);
		}
		frameBuffers[i].Create(OFF_SCREEN, Instance, Window);
		PipelineFlags defaultPF;
		defaultPF.depthTest = defaultPF.depthWrite = false;
		submissions[i].SetDefaultPipelineFlags(defaultPF);
		submissions[i].SetFrameBuffer(frameBuffers[i]);
		submissions[i].makeAsync();
		submissions[i].SetShader("default", shaders[i]);
	}

	RenderDistantGeom();
}

void HIGHOMEGA::RENDER::PASSES::SkyDomeClass::RenderDistantGeom()
{
	// Only render the cube map faces
	for (int i = 0; i != 6; i++)
	{
		distantVis[i].Render();
		distantGather[i].Render();
	}
}

void HIGHOMEGA::RENDER::PASSES::SkyDomeClass::Render(WorldParamsClass & WorldParams, GroupedTraceSubmission & rtSubmission)
{
	UpdateSkyInfo(true);
	SkyDomeParams.doAurora = FullResSkyDomeParams.doAurora = (WorldParams.isShowingAurora() ? 1.0f : 0.0f);
	skyDomeParamsBuf.UploadSubData(0, &SkyDomeParams, sizeof(SkyDomeParams));
	fullResSkyDomeParamsBuf.UploadSubData(0, &FullResSkyDomeParams, sizeof(FullResSkyDomeParams));

	// Re-render the mountains for the backdrop... 
	skyFrustums[6].CopyFromFrustum(MainFrustum);
	skyFrustums[6].eye = vec3(0.0f, InnerRad(), 0.0f);
	skyFrustums[6].Update();

	distantVis[6].Render();
	distantGather[6].Render();

	// Update shadow map
	vec3 mountainMin, mountainMax;
	mountains.getModelMinMax(mountainMin, mountainMax);
	distantGeomShadowMapNear.Render(mountainMin, mountainMax, 1.0f, vec2 (0.0000025f, 0.000005f), &skyFrustums[6]);
	distantGeomShadowMapFar.Render(mountainMin, mountainMax, 1.0f, vec2(0.0000025f, 0.000005f));

	static bool firstRun = true;
	static unsigned int frameCounter = 0;
	if (firstRun)
	{
		// Filter all shadow maps
		for (int i = 0; i != 7; i++)
			distantGeomScreenShadow[i].Render(rtSubmission);

		// Re-render all faces and backdrop
		for (int i = 0; i != 7; i++)
		{
			if (i != 6)
				skySubmissions[i].makeAsync();
			else
				skySubmissions[i].makeSerial();
			skySubmissions[i].Render();
		}
		for (int i = 0; i != 7; i++)
		{
			if (i != 6)
				submissions[i].makeAsync();
			else
				submissions[i].makeSerial();
			submissions[i].Render();
		}
		firstRun = false;
	}
	else
	{
		if (frameCounter % 18 < 6)
		{
			distantGeomScreenShadow[frameCounter % 18].makeAsync();
			distantGeomScreenShadow[frameCounter % 18].Render(rtSubmission);
		}
		else if (frameCounter % 18 < 12)
		{
			skySubmissions[frameCounter % 18 - 6].Render();
		}
		else
		{
			submissions[frameCounter % 18 - 12].Render();
		}
		distantGeomScreenShadow[6].Render(rtSubmission);
		skySubmissions[6].Render();
		submissions[6].Render();
		frameCounter++;
	}
}

unsigned int HIGHOMEGA::RENDER::PASSES::ClearSurfaceCacheClass::WorkGroupSize()
{
	return 4;
}

void HIGHOMEGA::RENDER::PASSES::ClearSurfaceCacheClass::Create(PathTraceClass & PathTrace)
{
	unsigned int voxelizedXWorkGroups = (unsigned int)ceil((double)HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE / (double)WorkGroupSize());
	unsigned int voxelizedYWorkGroups = (unsigned int)ceil((double)HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE / (double)WorkGroupSize());
	unsigned int voxelizedZWorkGroups = (unsigned int)ceil((double)HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE / (double)WorkGroupSize());

	std::vector<ShaderResource> radiosityMaps;
	for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++)
		for (int j = 0; j != 6; j++)
			radiosityMaps.emplace_back(RESOURCE_IMAGE_STORE, COMPUTE, 0, 0, PathTrace.radiosityMaps[i][j]);

	shader.Create("shaders/clearSurfaceCache.comp.spv", "main");
	shader.AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 0, radiosityMaps);
	submission.MakeDispatch(Instance, std::string ("default"), shader, voxelizedXWorkGroups, voxelizedYWorkGroups, voxelizedZWorkGroups);
	submission.makeAsync();
}

void HIGHOMEGA::RENDER::PASSES::ClearSurfaceCacheClass::Submit()
{
	submission.Submit();
}

void HIGHOMEGA::RENDER::PASSES::VisibilityPassClass::Create(unsigned int width, unsigned int height, FrustumClass & frustum)
{
	unsigned int val = 0xFFFFFFFF;
	float valF = *((float *)(&val));
	submission.SetClearColor(vec3(valF), valF);
	submission.Create(Instance);
	frustumRef = &frustum;

	visibilityTriInfo.CreateOffScreenColorAttachment(Instance, R32G32UI, width, height, false, false);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, width, height, ImageClass::SAMPLE_DEPTH);
	frameBuffer.AddColorAttachment(visibilityTriInfo);
	frameBuffer.SetDepthStencil(depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	PipelineFlags defaultPipelineFlags;
	defaultPipelineFlags.backFaceCulling = true;
	defaultPipelineFlags.depthCompare = COMPARE_GREATER_OR_EQUAL;
	submission.SetDepthClear(0.0f);
	submission.SetDefaultPipelineFlags(defaultPipelineFlags);
	submission.SetFrameBuffer(frameBuffer);

	shader.Create("shaders/visibility.vert.spv", "main", "shaders/visibility.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, visFrustum.Buffer);
	shaderAlphaKey.Create("shaders/visibility.vert.spv", "main", "shaders/visibilityAlphaKey.frag.spv", "main");
	shaderAlphaKey.AddResource(RESOURCE_UBO, VERTEX, 0, 0, visFrustum.Buffer);
	submission.SetShader("default", shader);
	submission.SetShader("alphaKey", shaderAlphaKey);
}

void HIGHOMEGA::RENDER::PASSES::VisibilityPassClass::Render()
{
	visFrustum.CopyFromFrustum(*frustumRef);
	visFrustum.reverseZ = true;
	visFrustum.Update();
	submission.doCulling(visFrustum, GroupedRasterSubmission::CULL_MODE::FRUSTUM_HIZ);
	submission.keepMatGeomBindings();
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::GatherResolveClass::Create(VisibilityPassClass & VisibilityPass, TriClass & PostProcessTri, FrustumClass & OriginalFrustum, bool simple)
{
	submission.SetClearColor(vec3(0.0f), 0.0f);
	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	if (!simple) materialAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, VisibilityPass.visibilityTriInfo.getWidth(), VisibilityPass.visibilityTriInfo.getHeight(), false, false);
	worldPosAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, VisibilityPass.visibilityTriInfo.getWidth(), VisibilityPass.visibilityTriInfo.getHeight(), false, false);
	normalAttach.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, VisibilityPass.visibilityTriInfo.getWidth(), VisibilityPass.visibilityTriInfo.getHeight(), false, false);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	if (!simple) frameBuffer.AddColorAttachment(materialAttach);
	frameBuffer.AddColorAttachment(worldPosAttach);
	frameBuffer.AddColorAttachment(normalAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);
	submission.SetFrameBuffer(frameBuffer);
	submission.SetDefaultPipelineFlags(defPipelineFlags);

	shader.Create("shaders/postprocess.vert.spv", "main", simple ? "shaders/gatherresolvesimple.frag.spv" : "shaders/gatherresolve.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, OriginalFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, VisibilityPass.visibilityTriInfo);
	submission.consumeMatGeomBindings(&VisibilityPass.submission);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::GatherResolveClass::Render()
{
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, SkyDomeClass & SkyDome, ShadowMapClass & ShadowMapNear, ShadowMapClass & ShadowMapFar, GroupedTraceSubmission & rtSubmission, GroupedSDFBVHSubmission *sdfBVHSubmission, const vec3 & initialViewer)
{
	GatherPassRef = &GatherPass;
	TraceRef = &rtSubmission;
	shadowMapNearRef = &ShadowMapNear;
	shadowMapFarRef = &ShadowMapFar;
	SkyDomeRef = &SkyDome;

	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);
	glossSubmission.Add(PostProcessTri.triModel);
	glossSubmission.Create(Instance);
	shadowBiasesBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &shadowBiases, (unsigned int)sizeof(shadowBiases));

	unsigned int traceWidth = 320;
	unsigned int traceHeight = 240;
	glossTraceOutput.CreateImageStore(Instance, R16G16B16A16F, traceWidth, traceHeight, 1, _2D, false);
	PathTraceParams.radiosityMapCenterMotionFactor[0] = initialViewer.x;
	PathTraceParams.radiosityMapCenterMotionFactor[1] = initialViewer.y;
	PathTraceParams.radiosityMapCenterMotionFactor[2] = initialViewer.z;
	PathTraceParams.radiosityMapCenterMotionFactor[3] = 0.0f;
	PathTraceParams.timeTurnBlurDirectionRawLight[0] = 0.0f;
	PathTraceParams.timeTurnBlurDirectionRawLight[1] = 0.0f;
	PathTraceParams.timeTurnBlurDirectionRawLight[2] = 0.0f;
	PathTraceParams.timeTurnBlurDirectionRawLight[3] = 0.0f;
	PathTraceParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &PathTraceParams, (unsigned int)sizeof(PathTraceParams));
	influence.factor = 1.0f;
	influenceBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &influence, (unsigned int)sizeof(influence));

	for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++)
		for (int j = 0; j != 6; j++)
		{
			radiosityMaps[i][j].CreateImageStore(Instance, R16G16B16A16F, HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, _3D, true);
			radiosityMapsVec.push_back(&radiosityMaps[i][j]);
		}
	radiosityMapsVec[0]->ClearColors(radiosityMapsVec, ImageClearColor(vec3(0.0f), 0.0f));

	if (RTInstance::Enabled())
	{
		rtShaderResourceSet.CreateRT("shaders/rtdiffusetrace.rgen.spv", "main", "shaders/rtdiffusetrace.rchit.spv", "main", "shaders/rtdiffusetrace.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
		tracelet.Make(Instance);
		rtShaderResourceSetGloss.CreateRT("shaders/rtglosstrace.rgen.spv", "main", "shaders/rtglosstrace.rchit.spv", "main", "shaders/rtglosstrace.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
		traceletGloss.Make(Instance);
	}
	else
	{
		frameBuffer.setWidth(glossTraceOutput.getWidth());
		frameBuffer.setHeight(glossTraceOutput.getHeight());
		frameBuffer.Create(OFF_SCREEN, Instance, Window);
		submission.SetFrameBuffer(frameBuffer);

		glossTraceFrameBuffer.setWidth(glossTraceOutput.getWidth());
		glossTraceFrameBuffer.setHeight(glossTraceOutput.getHeight());
		glossTraceFrameBuffer.Create(OFF_SCREEN, Instance, Window);
		glossSubmission.SetFrameBuffer(glossTraceFrameBuffer);

		std::vector <ShaderResource> radiosityMapResources;
		for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++)
			for (int j = 0; j != 6; j++)
				radiosityMapResources.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 0, radiosityMaps[i][j]);

		shader.Create("shaders/postprocess.vert.spv", "main", "shaders/SDFBVHdiffusetrace.frag.spv", "main");
		shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.materialAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherPass.normalAttach);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, MainFrustum.Buffer);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, PathTraceParamsBuf);
		shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, radiosityMapResources);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, SkyDome.fullCubeMap);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, ShadowMapNear.depthStencilAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, ShadowMapNear.shadowColorAttach);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 10, ShadowMapNear.frustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, ShadowMapFar.depthStencilAttach);
		shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 12, ShadowMapFar.shadowColorAttach);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 13, ShadowMapFar.frustum.Buffer);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 14, shadowBiasesBuf);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 15, SkyDome.rayleighMieBuf);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 16, influenceBuf);
		submission.SetShader("default", shader);
		submission.requestSDFBVH(*sdfBVHSubmission);

		glossTraceShader.Create("shaders/postprocess.vert.spv", "main", "shaders/SDFBVHglosstrace.frag.spv", "main");
		glossTraceShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.materialAttach);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherPass.normalAttach);
		glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 4, glossTraceOutput);
		glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, MainFrustum.Buffer);
		glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 6, PathTraceParamsBuf);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, radiosityMapResources);
		glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 8, radiosityMapResources);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, SkyDome.fullCubeMap);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 10, ShadowMapNear.depthStencilAttach);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, ShadowMapNear.shadowColorAttach);
		glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 12, ShadowMapNear.frustum.Buffer);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 13, ShadowMapFar.depthStencilAttach);
		glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 14, ShadowMapFar.shadowColorAttach);
		glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 15, ShadowMapFar.frustum.Buffer);
		glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 16, shadowBiasesBuf);
		glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 17, SkyDome.rayleighMieBuf);
		glossSubmission.SetShader("default", glossTraceShader);
		glossSubmission.requestSDFBVH(*sdfBVHSubmission);
	}
}

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Render(vec3 & currentViewer)
{
	if (influence.factor != 1.0f)
	{
		influence.factor += (1.0f - influence.factor) * 0.9f;
		if (fabs(influence.factor - 1.0f) < 0.1f)
			influence.factor = 1.0f;
		influenceBuf.UploadSubData(0, &influence, sizeof(influence));
	}
	if ((currentViewer - viewerCached).length() > 100.0f)
	{
		radiosityMapsVec[0]->ClearColors(radiosityMapsVec, ImageClearColor(vec3(0.0f), 0.0f));
		viewerCached = currentViewer;
		PathTraceParams.radiosityMapCenterMotionFactor[0] = viewerCached.x;
		PathTraceParams.radiosityMapCenterMotionFactor[1] = viewerCached.y;
		PathTraceParams.radiosityMapCenterMotionFactor[2] = viewerCached.z;
		PathTraceParamsBuf.UploadSubData(0, &PathTraceParams, sizeof(PathTraceParams));
		influence.factor = 7.5f;
		influenceBuf.UploadSubData(0, &influence, sizeof(influence));
	}

	shadowBiases.nearMinBias = shadowMapNearRef->samplingBias.x;
	shadowBiases.nearMaxBias = shadowMapNearRef->samplingBias.y;
	shadowBiases.farMinBias = shadowMapFarRef->samplingBias.x;
	shadowBiases.farMaxBias = shadowMapFarRef->samplingBias.y;
	shadowBiasesBuf.UploadSubData(0, &shadowBiases, sizeof(shadowBiases));

	if (RTInstance::Enabled())
	{
		RTScene & rtSceneRef = TraceRef->rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = TraceRef->SceneID();
		if (curSceneId != lastSceneId)
		{
			std::vector <ShaderResource> radiosityMapResources;
			for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++)
				for (int j = 0; j != 6; j++)
					radiosityMapResources.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 0, radiosityMaps[i][j]);

			lastSceneId = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources.clear();
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 1, GatherPassRef->materialAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 2, GatherPassRef->worldPosAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 3, GatherPassRef->normalAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 4, MainFrustum.Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 5, PathTraceParamsBuf);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 6, radiosityMapResources);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 7, SkyDomeRef->fullCubeMap);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 8, shadowMapNearRef->depthStencilAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 9, shadowMapNearRef->shadowColorAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 10, shadowMapNearRef->frustum.Buffer);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 11, shadowMapFarRef->depthStencilAttach);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 12, shadowMapFarRef->shadowColorAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 13, shadowMapFarRef->frustum.Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 14, shadowBiasesBuf);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 15, SkyDomeRef->rayleighMieBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 1, rtSceneRef.getGeomResources());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 2, rtSceneRef.getMaterialResources());
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT, 3, 0, rtSceneRef.getInstancePropertiesBuffer());

			tracingResourcesGloss.clear();
			tracingResourcesGloss.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 1, GatherPassRef->materialAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 2, GatherPassRef->worldPosAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 3, GatherPassRef->normalAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 4, glossTraceOutput);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 5, MainFrustum.Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 6, PathTraceParamsBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 7, radiosityMapResources);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 8, radiosityMapResources);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 9, SkyDomeRef->fullCubeMap);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 10, shadowMapNearRef->depthStencilAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 11, shadowMapNearRef->shadowColorAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 12, shadowMapNearRef->frustum.Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 13, shadowMapFarRef->depthStencilAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 14, shadowMapFarRef->shadowColorAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 15, shadowMapFarRef->frustum.Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 16, shadowBiasesBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 17, SkyDomeRef->rayleighMieBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 1, rtSceneRef.getGeomResources());
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 2, rtSceneRef.getMaterialResources());
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT, 3, 0, rtSceneRef.getInstancePropertiesBuffer());
		}
		tracelet.Submit(glossTraceOutput.getWidth(), glossTraceOutput.getHeight(), 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
		traceletGloss.Submit(glossTraceOutput.getWidth(), glossTraceOutput.getHeight(), 1, tracingResourcesGloss, rewriteDescriptoSets, rtShaderResourceSetGloss);
	}
	else
	{
		submission.Render();
		if (influence.factor == 7.5f)
		{
			PathTraceParams.timeTurnBlurDirectionRawLight[0] += 0.1f;
			PathTraceParamsBuf.UploadSubData(0, &PathTraceParams, sizeof(PathTraceParams));
			submission.Render();
			PathTraceParams.timeTurnBlurDirectionRawLight[0] += 0.1f;
			PathTraceParamsBuf.UploadSubData(0, &PathTraceParams, sizeof(PathTraceParams));
			submission.Render();
			PathTraceParams.timeTurnBlurDirectionRawLight[0] += 0.1f;
			PathTraceParamsBuf.UploadSubData(0, &PathTraceParams, sizeof(PathTraceParams));
			submission.Render();
			PathTraceParams.timeTurnBlurDirectionRawLight[0] += 0.1f;
			PathTraceParamsBuf.UploadSubData(0, &PathTraceParams, sizeof(PathTraceParams));
			submission.Render();
		}
		glossSubmission.Render();
	}
}

HIGHOMEGA::RENDER::PASSES::BlueNoiseHolderClass::BlueNoiseHolderClass()
{
	if (!blueNoise) blueNoise = new ImageClass;
	blueNoiseClaims++;
}

HIGHOMEGA::RENDER::PASSES::BlueNoiseHolderClass::~BlueNoiseHolderClass()
{
	blueNoiseClaims--;
	if (blueNoiseClaims == 0)
	{
		delete blueNoise;
		blueNoise = nullptr;
	}
}

/*
	******************************************************
	*************Beginning of SauRay(TM) code*************
	******************************************************

	Copyright © 2023 TooMuchVoltage Software Inc. This notice shall always be coupled with any SauRay(TM) implementation and must be redistributed alongside it.

	This implementation of US20220219086A1 is provided royalty free for either of the following:

	* Games with gross revenues of under one(1) million dollars CAD.
	* Games with at least a publically distributed moddable server binary with which SauRay(TM) is successfully integrable.

	Public distribution requires either a public download link or a relatively simple registration and download process. If you are unsure of your registration process's straightforwardness, reach out directly.

	Free open-source games (i.e. Cube/Sauerbraten or Xonotic) automatically qualify since successful SauRay(TM) integration is ultimately feasible with sufficient effort.

	Open source games with non-Libre licenses (i.e. non-GPL, non-MIT) also qualify as long as the license is no further restrictive than that of Quake(idTech) II's. If unsure of whether your source code redistribution license is permissive enough, please reach out directly.

	For games where at least the distributed server component is either open-source or moddable (in a manner permissible by the IP owner) the game must be sufficiently thin-client so that a SauRay(TM) integration does not result in crashes or defects that largely break the game in most multiplayer game modes. If you are unsure of whether your distributed binaries qualify for this category, please get in touch directly.

	We can be reached at the email address: sauray@toomuchvoltage.com or using the contact information found on the website http://sauray.tech .

	If your game does not qualify under either of the above categories, contact us for a commercial license. The covered source files are protected by copyright and the aforementioned terms will apply beyond the life of US20220219086A1.

	All games using US20220219086A1 or this implementation of it must clearly declare that they're using it in a way noticeable and comprehensible by an average player of the game in the English language.

	Beyond what is stated in http://toomuchvoltage.com/pub/sauray_techbrief/sauray_techbrief.pdf this source code does not provide any warranties of merchantability or fitness for any particular purpose.
*/

void HIGHOMEGA::RENDER::PASSES::SaurayTraceClass::SetPlayer(unsigned int playerId, unsigned char otherTeamId, const vec3 & eye, const vec3 & look, const vec3 & up, const vec3 & eye2, const vec3 & look2, const vec3 & up2, float inYFov, float inWhr, vec3 & geomCent, float geomRad)
{
	if (playerId >= maxPlayers)
	{
		LOG() << "SetPlayer ignored. PlayerID (" << playerId << ") is higher than maxPlayers (" << maxPlayers << ")";
		return;
	}
	playerFrusta[playerId].eyeGeomRad[0] = eye.x;
	playerFrusta[playerId].eyeGeomRad[1] = eye.y;
	playerFrusta[playerId].eyeGeomRad[2] = eye.z;
	playerFrusta[playerId].eyeGeomRad[3] = geomRad;
	playerFrusta[playerId].eye2Whr[0] = eye2.x;
	playerFrusta[playerId].eye2Whr[1] = eye2.y;
	playerFrusta[playerId].eye2Whr[2] = eye2.z;
	playerFrusta[playerId].eye2Whr[3] = inWhr;
	vec2 lookSph = toSpherical(look);
	vec2 upSph = toSpherical(up);
	vec2 look2Sph = toSpherical(look2);
	vec2 up2Sph = toSpherical(up2);
	playerFrusta[playerId].lookUpLook2Up2[0] = packSpherical(lookSph);
	playerFrusta[playerId].lookUpLook2Up2[1] = packSpherical(upSph);
	playerFrusta[playerId].lookUpLook2Up2[2] = packSpherical(look2Sph);
	playerFrusta[playerId].lookUpLook2Up2[3] = packSpherical(up2Sph);
	playerFrusta[playerId].geomCentYScale[0] = geomCent.x;
	playerFrusta[playerId].geomCentYScale[1] = geomCent.y;
	playerFrusta[playerId].geomCentYScale[2] = geomCent.z;
	playerFrusta[playerId].geomCentYScale[3] = tanf(((inYFov * HIGHOMEGA_PI) / 180.0f) * 0.5f);
	playerFrusta[playerId].maskEnabledReserved = (otherTeamId << 24);
	playerFrusta[playerId].maskEnabledReserved |= (1 << 16);
	newPlayerInfo = true;
}

void HIGHOMEGA::RENDER::PASSES::SaurayTraceClass::RemovePlayer(unsigned int playerId)
{
	if (playerId >= maxPlayers)
	{
		LOG() << "RemovePlayer ignored. PlayerID (" << playerId << ") is higher than maxPlayers (" << maxPlayers << ")";
		return;
	}
	playerFrusta[playerId].maskEnabledReserved &= 0xFF00FFFF;
	newPlayerInfo = true;
}

void HIGHOMEGA::RENDER::PASSES::SaurayTraceClass::Create(GroupedTraceSubmission & mainSubmission, unsigned int inpMaxPlayers, unsigned int inpResSide, unsigned int inpHistoryAmount, bool debugMode)
{
	if (blueNoise->getWidth() == 0) blueNoise->CreateTexture(Instance, "assets/common/", "bluenoise.tga", 1, false, false, false, false);

	mainSubmissionRef = &mainSubmission;
	playerResSide = inpResSide;
	maxPlayers = inpMaxPlayers;
	maxPlayerSqrt = (unsigned int)sqrt(maxPlayers);
	resSide = playerResSide * maxPlayerSqrt;
	temporalAmount = inpHistoryAmount;

	playerFrusta.resize(maxPlayers);
	playerLimits.resize(maxPlayers);
	playerVisMatrix.resize(maxPlayers * maxPlayers);
	for (int i = 0; i != playerLimits.size(); i++)
	{
		for (int j = 0; j != 20; j++)
			playerLimits[i].aabbLim[j] = 0.0f;
		for (int j = 0; j != 24; j++)
			playerLimits[i].corners[j] = 0.0f;
	}

	for (int i = 0; i != maxPlayers; i++)
		playerFrusta[i].maskEnabledReserved &= 0xFF00FFFF;
	memset((void *)playerVisMatrix.data(), 0, (unsigned int)playerVisMatrix.size() * sizeof(playerVisData));
	timeInfo.frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[0] = 0;
	timeInfo.frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[1] = maxPlayerSqrt;
	timeInfo.frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[2] = playerResSide;
	timeInfo.frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[3] = temporalAmount;

	frustaBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, (void *)playerFrusta.data(), (unsigned int)(playerFrusta.size() * sizeof(playerFrustum)));
	limitsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, (void *)playerLimits.data(), (unsigned int)(playerLimits.size() * sizeof(playerLimit)));
	visibilityMatrixBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, (void *)playerVisMatrix.data(), (unsigned int)(playerVisMatrix.size() * sizeof(playerVisData)));
	timeBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &timeInfo, (unsigned int)sizeof(timeInfo));

	testOutput.CreateImageStore(Instance, R8G8B8A8UN, resSide, resSide, 1, _2D, false);

	if (RTInstance::Enabled())
	{
		if (debugMode)
			rtShaderResourceSet.CreateRT("shaders/rtsauraytrace.rgen.spv", "main", "shaders/rtsauraytrace.rchit.spv", "main", "shaders/rtsauraytrace.rmiss.spv", "main", "shaders/rtsauraytrace.rahit.spv", "main");
		else
			rtShaderResourceSet.CreateRT("shaders/rtsauraytrace_release.rgen.spv", "main", "shaders/rtsauraytrace_release.rchit.spv", "main", "shaders/rtsauraytrace_release.rmiss.spv", "main", "shaders/rtsauraytrace_release.rahit.spv", "main");
		rtShaderResourceSet2.CreateRT("shaders/rtsauraylimits.rgen.spv", "main", "shaders/rtsauraylimits.rchit.spv", "main", "shaders/rtsauraylimits.rmiss.spv", "main", "shaders/rtsauraylimits.rahit.spv", "main");
		tracelet.Make(Instance);
		tracelet2.Make(Instance);
	}
}

void HIGHOMEGA::RENDER::PASSES::SaurayTraceClass::PrePass()
{
	if (RTInstance::Enabled())
	{
		if (newPlayerInfo)
		{
			frustaBuf.UploadSubData(0, (void *)playerFrusta.data(), (unsigned int)playerFrusta.size() * sizeof(playerFrustum));
			newPlayerInfo = false;
		}
		RTScene & rtSceneRef = mainSubmissionRef->rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = mainSubmissionRef->SceneID();
		if (curSceneId != lastSceneId2)
		{
			lastSceneId2 = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources2.clear();
			tracingResources2.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResources2.emplace_back(RESOURCE_SSBO, RT_RAYGEN, 0, 1, frustaBuf);
			tracingResources2.emplace_back(RESOURCE_SSBO, RT_RAYGEN, 0, 2, limitsBuf);
		}
		tracelet2.Submit((unsigned int)playerLimits.size() * 26, 1, 1, tracingResources2, rewriteDescriptoSets, rtShaderResourceSet2);
		limitsBuf.DownloadSubData(0, playerLimits.data(), (unsigned int)playerLimits.size() * sizeof(playerLimit));
	}
}

void HIGHOMEGA::RENDER::PASSES::SaurayTraceClass::Render()
{
	if (RTInstance::Enabled())
	{
		if (newPlayerInfo)
		{
			frustaBuf.UploadSubData(0, (void *)playerFrusta.data(), (unsigned int)playerFrusta.size() * sizeof(playerFrustum));
			newPlayerInfo = false;
		}
		unsigned int curTemporalBit = timeInfo.frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[0] % temporalAmount;
		unsigned int curVisCellChannel = (curTemporalBit / 32) % 4;
		unsigned int curMask = (~(0x00000001 << (curTemporalBit % 32)));
		for (unsigned int i = 0; i != maxPlayers; i++)
			for (unsigned int j = 0; j != maxPlayers; j++)
			{
				unsigned int curCellID = i * maxPlayers + j;
				playerVisMatrix[curCellID].visCell[curVisCellChannel] &= curMask;

				if (temporalAmount < 32) playerVisMatrix[curCellID].visCell[0] &= (0xFFFFFFFF >> (32 - temporalAmount));
				else if (temporalAmount < 64) playerVisMatrix[curCellID].visCell[1] &= (0xFFFFFFFF >> (64 - temporalAmount));
				else if (temporalAmount < 96) playerVisMatrix[curCellID].visCell[2] &= (0xFFFFFFFF >> (96 - temporalAmount));
				else if (temporalAmount < 128) playerVisMatrix[curCellID].visCell[3] &= (0xFFFFFFFF >> (128 - temporalAmount));
			}
		visibilityMatrixBuf.UploadSubData(0, playerVisMatrix.data(), (unsigned int)playerVisMatrix.size() * sizeof(playerVisData));
		RTScene & rtSceneRef = mainSubmissionRef->rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = mainSubmissionRef->SceneID();
		if (curSceneId != lastSceneId)
		{
			lastSceneId = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources.clear();
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, testOutput);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 2, *blueNoise);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_ANYHIT, 0, 3, rtSceneRef.getInstancePropertiesBuffer());
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RAYGEN, 0, 4, frustaBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_ANYHIT, 0, 5, visibilityMatrixBuf);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN | RT_ANYHIT, 0, 6, timeBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RAYGEN, 0, 7, limitsBuf);
		}
		tracelet.Submit(resSide, resSide, 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
		visibilityMatrixBuf.DownloadSubData(0, playerVisMatrix.data(), (unsigned int)playerVisMatrix.size() * sizeof(playerVisData));
		for (unsigned int i = 0; i != maxPlayers; i++)
			for (unsigned int j = 0; j != maxPlayers; j++)
			{
				unsigned int curCellID = i * maxPlayers + j;
				if (temporalAmount < 32) playerVisMatrix[curCellID].visCell[0] &= (0xFFFFFFFF >> (32 - temporalAmount));
				else if (temporalAmount < 64) playerVisMatrix[curCellID].visCell[1] &= (0xFFFFFFFF >> (64 - temporalAmount));
				else if (temporalAmount < 96) playerVisMatrix[curCellID].visCell[2] &= (0xFFFFFFFF >> (96 - temporalAmount));
				else if (temporalAmount < 128) playerVisMatrix[curCellID].visCell[3] &= (0xFFFFFFFF >> (128 - temporalAmount));
			}
		timeInfo.frameCountMaxPlayersSqrtSideResTemporalHistoryAmount[0]++;
		timeBuf.UploadSubData(0, &timeInfo, sizeof(timeInfo));
	}
}

unsigned int HIGHOMEGA::RENDER::PASSES::SaurayTraceClass::CanSee(unsigned int viewer, unsigned int subject)
{
	unsigned int res = playerVisMatrix[subject * maxPlayers + viewer].visCell[0];
	res |= playerVisMatrix[subject * maxPlayers + viewer].visCell[1];
	res |= playerVisMatrix[subject * maxPlayers + viewer].visCell[2];
	res |= playerVisMatrix[subject * maxPlayers + viewer].visCell[3];
	return res;
}

/*
	******************************************************
	****************End of SauRay(TM) code****************
	******************************************************
*/

void HIGHOMEGA::RENDER::PASSES::TemporalAccumulateClass::Create(TriClass &PostProcessTri, GatherResolveClass & GatherResolve, PathTraceClass & PathTrace)
{
	pathTraceRef = &PathTrace;

	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	MVPsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &MVPs, (unsigned int)sizeof(MVPs));

	glossTemporalAccumulateResultAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), false, false);
	glossLightTrailAttach.CreateImageStore(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT, _3D, false);
	worldPosCacheAttach.CreateImageStore(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT, _3D, false);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthWrite = false;
	defPipelineFlags.depthTest = false;
	frameBuffer.AddColorAttachment(glossTemporalAccumulateResultAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.SetFrameBuffer(frameBuffer);
	submission.SetDefaultPipelineFlags(defPipelineFlags);

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/temporalAccumulate.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, PathTrace.glossTraceOutput);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherResolve.materialAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherResolve.worldPosAttach);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, PathTrace.PathTraceParamsBuf);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, MVPsBuf);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, glossLightTrailAttach);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 7, worldPosCacheAttach);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::TemporalAccumulateClass::Render()
{
	unsigned int curTurn = (int)(pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[1]);
	static vec3 lastEye = vec3(0.0f), lastLook = vec3(0.0f);

	memcpy(MVPs.matrix[curTurn], MainFrustum.uboData.modelViewProj, sizeof(MainFrustum.uboData.modelViewProj));
	MVPsBuf.UploadSubData(curTurn * sizeof(MainFrustum.uboData.modelViewProj), &MVPs.matrix[curTurn], sizeof(MainFrustum.uboData.modelViewProj));

	float eyeDiffFactor = min(0.005f / max((MainFrustum.eye - lastEye).length(), 0.00001f), 1.0f);
	float lookDiffFactor = min(0.1f / max((MainFrustum.look - lastLook).length(), 0.00001f), 1.0f);
	pathTraceRef->PathTraceParams.radiosityMapCenterMotionFactor[3] = eyeDiffFactor * lookDiffFactor;
	pathTraceRef->PathTraceParamsBuf.UploadSubData(0, &pathTraceRef->PathTraceParams, sizeof(pathTraceRef->PathTraceParams));

	submission.Render();

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[1] = (float)((curTurn + 1) % HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT);

	lastEye = MainFrustum.eye;
	lastLook = MainFrustum.look;
}

void HIGHOMEGA::RENDER::PASSES::SpatialDenoiseClass::Create(TriClass & PostProcessTri, PathTraceClass & PathTrace, TemporalAccumulateClass & TemporalAccumulate, GatherResolveClass &GatherResolve)
{
	pathTraceRef = &PathTrace;

	blurHAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), false, false);
	blurVAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), true, false);

	submissionH.Add(PostProcessTri.triModel);
	submissionH.Create(Instance);
	submissionV.Add(PostProcessTri.triModel);
	submissionV.Create(Instance);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	frameBufferH.AddColorAttachment(blurHAttach);
	frameBufferV.AddColorAttachment(blurVAttach);
	frameBufferH.Create(OFF_SCREEN, Instance, Window);
	frameBufferV.Create(OFF_SCREEN, Instance, Window);
	submissionH.SetFrameBuffer(frameBufferH);
	submissionH.SetDefaultPipelineFlags(defPipelineFlags);
	submissionV.SetFrameBuffer(frameBufferV);
	submissionV.SetDefaultPipelineFlags(defPipelineFlags);

	shaderH.Create("shaders/postprocess.vert.spv", "main", "shaders/glossBilateral.frag.spv", "main");
	shaderV.Create("shaders/postprocess.vert.spv", "main", "shaders/glossBilateral.frag.spv", "main");

	shaderH.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderH.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, TemporalAccumulate.glossTemporalAccumulateResultAttach);
	shaderH.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherResolve.materialAttach);
	shaderH.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherResolve.worldPosAttach);
	shaderH.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, GatherResolve.normalAttach);
	shaderH.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, pathTraceRef->PathTraceParamsBuf);

	shaderV.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderV.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, blurHAttach);
	shaderV.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherResolve.materialAttach);
	shaderV.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherResolve.worldPosAttach);
	shaderV.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, GatherResolve.normalAttach);
	shaderV.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, pathTraceRef->PathTraceParamsBuf);

	submissionH.SetShader("default", shaderH);
	submissionV.SetShader("default", shaderV);
}
 
void HIGHOMEGA::RENDER::PASSES::SpatialDenoiseClass::Render()
{
	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[3] = GetStateOfKeybAction(CMD_SWITCH_TO_PT_MODE) ? 1.0f : 0.0f;

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[0] += 0.01f;

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[2] = 0.0f;
	pathTraceRef->PathTraceParamsBuf.UploadSubData(0, &pathTraceRef->PathTraceParams, sizeof(pathTraceRef->PathTraceParams));

	submissionH.Render();

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[2] = 1.0f;
	pathTraceRef->PathTraceParamsBuf.UploadSubData(0, &pathTraceRef->PathTraceParams, sizeof(pathTraceRef->PathTraceParams));

	submissionV.Render();
}

void HIGHOMEGA::RENDER::PASSES::SimpleGaussian::Create(TriClass & PostProcessTri, BlurInputHolder & inputHolder, unsigned int outputWidth, unsigned int outputHeight, unsigned int blurSize, bool displayVOnScreen)
{
	blurParams.invDims[0] = 1.0f / (float)(outputWidth);
	blurParams.invDims[1] = 1.0f / (float)(outputHeight);
	blurParams.blurDirection = 0.0f;
	blurParams.size = (float)blurSize;
	blurParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &blurParams, (unsigned int)sizeof(blurParams));

	blurHAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, outputWidth, outputHeight, true, true);
	blurVAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, outputWidth, outputHeight, true, true);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	frameBufferH.AddColorAttachment(blurHAttach);
	frameBufferH.Create(OFF_SCREEN, Instance, Window);

	if (displayVOnScreen)
	{
		blurVSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());
	}
	else
	{
		frameBufferV.AddColorAttachment(blurVAttach);
		frameBufferV.Create(OFF_SCREEN, Instance, Window);
		blurVSubmission.SetFrameBuffer(frameBufferV);
	}

	blurHSubmission.Add(PostProcessTri.triModel);
	blurHSubmission.Create(Instance);
	blurVSubmission.Add(PostProcessTri.triModel);
	blurVSubmission.Create(Instance);

	blurHSubmission.SetFrameBuffer(frameBufferH);

	blurHSubmission.SetDefaultPipelineFlags(defPipelineFlags);
	blurVSubmission.SetDefaultPipelineFlags(defPipelineFlags);

	blurHShader.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSepSimple.frag.spv", "main");
	blurHShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	blurHShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, blurParamsBuf);
	blurHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, inputHolder.blurInput);
	blurHSubmission.SetShader("default", blurHShader);

	blurVShader.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSepSimple.frag.spv", "main");
	blurVShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	blurVShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, blurParamsBuf);
	blurVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, blurHAttach);
	blurVSubmission.SetShader("default", blurVShader);
}

void HIGHOMEGA::RENDER::PASSES::SimpleGaussian::Render(unsigned int blurSize)
{
	blurParams.blurDirection = 0.0f;
	blurParams.size = (float)blurSize;
	blurParamsBuf.UploadSubData(0, &blurParams, sizeof(blurParams));

	blurHSubmission.Render();

	blurParams.blurDirection = 1.0f;
	blurParams.size = (float)blurSize;
	blurParamsBuf.UploadSubData(0, &blurParams, sizeof(blurParams));

	blurVSubmission.Render();
}

void HIGHOMEGA::RENDER::PASSES::ModulateClass::Create(TriClass & PostProcessTri, VisibilityPassClass & VisibilityPass, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, SpatialDenoiseClass & SpatialDenoise, SkyDomeClass & SkyDome, ShadowMapScreenClass & shadowMapScreen)
{
	modulatedOutput.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, ScreenSize.width, ScreenSize.height, false, true);
	frameBuffer.AddColorAttachment(modulatedOutput);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	submission.SetFrameBuffer(frameBuffer);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	submission.SetDefaultPipelineFlags(defPipelineFlags);

	std::vector<ShaderResource> radiosityMaps;
	for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++)
		for (int j = 0; j != 6; j++)
			radiosityMaps.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 0, PathTrace.radiosityMaps[i][j]);

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/modulate.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, SpatialDenoise.blurVAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.materialAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, GatherPass.normalAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, radiosityMaps);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 6, PathTrace.PathTraceParamsBuf);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, SkyDome.fullBackDrop);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, SkyDome.fullCubeMap);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, shadowMapScreen.shadowMapScreen);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 10, SkyDome.rayleighMieBuf);
	submission.consumeMatGeomBindings(&VisibilityPass.submission);
	submission.SetShader("default", shader);

	particleShader.Create("shaders/particle.vert.spv", "main", "shaders/particle.frag.spv", "main");
	particleShader.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, MainFrustum.Buffer);
	particleShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	submission.SetShader("shaderParticle", particleShader);
}

void HIGHOMEGA::RENDER::PASSES::ModulateClass::Render()
{
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceGatherClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams)
{
	ssGatherPosAlbedo.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false);
	ssGatherNormRoughnessBackdrop.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, ScreenSize.width, ScreenSize.height);

	frameBuffer.AddColorAttachment(ssGatherPosAlbedo);
	frameBuffer.AddColorAttachment(ssGatherNormRoughnessBackdrop);
	frameBuffer.SetDepthStencil(depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.Create(Instance);

	submission.SetFrameBuffer(frameBuffer);

	shaderScreenSpace.Create("shaders/ssgather.vert.spv", "main", "shaders/ssgather.frag.spv", "main");
	shaderScreenSpace.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, MainFrustum.Buffer);
	shaderScreenSpace.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderTessScreenSpace.Create("shaders/ssgather.vert.spv", "main", "shaders/ssgather.tesc.spv", "main", "shaders/ssgather.tese.spv", "main", "shaders/ssgather.frag.spv", "main");
	shaderTessScreenSpace.AddResource(RESOURCE_UBO, VERTEX | TESS_EVAL | FRAGMENT, 0, 0, MainFrustum.Buffer);
	shaderTessScreenSpace.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderTessScreenSpace.AddResource(RESOURCE_UBO, TESS_EVAL, 0, 2, WorldParams.GetRenderTimeBuffer());
	submission.SetShader("shaderScreenSpace", shaderScreenSpace);
	submission.SetShader("shaderTessScreenSpace", shaderTessScreenSpace);
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceGatherClass::Render()
{
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::NearScatteringClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, ShadowMapClass & ShadowMapNear, ShadowMapClass & ShadowMapFar, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams, ScreenSpaceGatherClass & ScreenSpaceGather, unsigned int outputSize)
{
	if (blueNoise->getWidth() == 0) blueNoise->CreateTexture(Instance, "assets/common/", "bluenoise.tga", 1, false, false, false, false);

	blurInput.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, outputSize, outputSize, true, true);

	nearScatteringParams.amount = WorldParams.GetLightShaftAmount();
	nearScatteringParams.extinction = WorldParams.GetLightShaftExtinction();
	nearScatteringParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &nearScatteringParams, (unsigned int)sizeof(nearScatteringParams));

	frameBuffer.AddColorAttachment(blurInput);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	submission.SetFrameBuffer(frameBuffer);
	submission.SetDefaultPipelineFlags(defPipelineFlags);

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/nearScattering.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, SkyDome.rayleighMieBuf);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 2, nearScatteringParamsBuf);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ShadowMapNear.depthStencilAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, ShadowMapNear.shadowColorAttach);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, ShadowMapNear.frustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, ShadowMapFar.depthStencilAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, ShadowMapFar.shadowColorAttach);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, ShadowMapFar.frustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 10, ScreenSpaceGather.ssGatherPosAlbedo);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, SkyDome.fullCubeMap);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 12, *blueNoise);
	submission.SetShader("default", shader);
	submission.makeAsync();

	blurPass.Create(PostProcessTri, *this, outputSize, outputSize, 4);

	std::vector<ImageClass *> blurVImageVector;
	blurVImageVector.push_back(&blurPass.blurVAttach);
	blurPass.blurVAttach.ClearColors(blurVImageVector, ImageClearColor(vec3(0.0f), 1.0f));
}

void HIGHOMEGA::RENDER::PASSES::NearScatteringClass::Render()
{
	if (nearScatteringParams.amount > 0.0f)
	{
		submission.Render();
		blurPass.Render(4);
	}
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceFXClass::Create(TriClass & PostProcessTri, SkyDomeClass & SkyDome, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, ModulateClass & Modulate, ScreenSpaceGatherClass & ScreenSpaceGather, NearScatteringClass & NearScattering)
{
	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	ssfxOut.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, ScreenSize.width, ScreenSize.height, false, true);

	frameBuffer.AddColorAttachment(ssfxOut);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.SetFrameBuffer(frameBuffer);
	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;
	submission.SetDefaultPipelineFlags(defPipelineFlags);

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/ssfx.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, Modulate.modulatedOutput);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ScreenSpaceGather.ssGatherPosAlbedo);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, ScreenSpaceGather.ssGatherNormRoughnessBackdrop);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, SkyDome.fullCubeMap);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, SkyDome.fullBackDrop);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, NearScattering.blurPass.blurVAttach);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, MainFrustum.Buffer);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceFXClass::Render()
{
	submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, ScreenSpaceFXClass & ScreenSpaceFX)
{
	memset(onScreenText, (int)(' '), 60 * 5);
	onScreenTextImage.CreateTexture(Instance, 60, 5, onScreenText, 1u, false, false, false, false, R8UN);

	curMidScreenTextType = NONE;
	memset(midScreenText, (int)(' '), 30 * 3);
	midScreenTextImage.CreateTexture(Instance, 30, 3, midScreenText, 1u, false, false, false, false, R8UN);

	fontMap.CreateTexture(Instance, "assets/common/", "fontmap.tga");

	dofParams.invDims[0] = 1.0f / (float)(ScreenSize.width);
	dofParams.invDims[1] = 1.0f / (float)(ScreenSize.height);
	for (int i = 0; i != 4; i++) {
		dofParams.screenMidPos[i] = 0.0f;
		dofParams.toScreenMidPosAlpha[i] = 0.0f;
	}
	dofParams.midScreenAlpha = 0.0f;
	midScreenAlphaTarget = 0.0f;
	dofParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_SSBO, Instance, &dofParams, (unsigned int)sizeof(dofParams));

	dofHAttach.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, ScreenSize.width, ScreenSize.height, false, true);

	frameBufferH.AddColorAttachment(dofHAttach);
	frameBufferH.Create(OFF_SCREEN, Instance, Window);

	dofHSubmission.Add(PostProcessTri.triModel);
	dofHSubmission.Create(Instance);
	dofVSubmission.Add(PostProcessTri.triModel);
	dofVSubmission.Create(Instance);

	dofHSubmission.SetFrameBuffer(frameBufferH);
	dofVSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());
	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	dofHShader.Create("shaders/postprocess.vert.spv", "main", "shaders/dof.frag.spv", "main");
	dofHShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	dofHShader.AddResource(RESOURCE_SSBO, FRAGMENT, 0, 1, dofParamsBuf);
	dofHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	dofHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ScreenSpaceFX.ssfxOut);
	dofHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, onScreenTextImage);
	dofHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, midScreenTextImage);
	dofHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, fontMap);
	dofHSubmission.SetDefaultPipelineFlags(defPipelineFlags);
	dofHSubmission.SetShader("default", dofHShader);

	dofVShader.Create("shaders/postprocess.vert.spv", "main", "shaders/dof.frag.spv", "main");
	dofVShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	dofVShader.AddResource(RESOURCE_SSBO, FRAGMENT, 0, 1, dofParamsBuf);
	dofVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	dofVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, dofHAttach);
	dofVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, onScreenTextImage);
	dofVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, midScreenTextImage);
	dofVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, fontMap);
	dofVSubmission.SetDefaultPipelineFlags(defPipelineFlags);
	dofVSubmission.SetShader("default", dofVShader);
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::Render(float inpAlpha)
{
	memset(onScreenText, (int)(' '), 60 * 5);
	sprintf_s((char *)(&onScreenText[60 * 0]), 59, "Total: %s", frameInstrument.ResultsMs(52).c_str());
	sprintf_s((char *)(&onScreenText[60 * 1]), 59, "FPS: %d", HIGHOMEGA::INSTRUMENTATION::FPSCounter::Report());
	sprintf_s((char *)(&onScreenText[60 * 2]), 59, "Update: %s", updateInstrument.ResultsMs(51).c_str());

	onScreenTextImage.UploadData(onScreenText, 60 * 5);

	dofParams.blurDirection = 0.0f;
	dofParams.invCoCDist = 1.0f / 10000000.0f;
	dofParamsBuf.UploadSubData(0, &dofParams, sizeof(dofParams));

	dofHSubmission.Render();

	dofParams.blurDirection = 1.0f;
	dofParams.toScreenMidPosAlpha[3] = inpAlpha;
	dofParamsBuf.UploadSubData(0, &dofParams, sizeof(dofParams));

	if (!HIGHOMEGA::EVENTS::windowMinimized) dofVSubmission.Render();

	dofParamsBuf.DownloadSubData(0, &dofParams, sizeof(dofParams));
	vec3 screenMidPos = vec3(dofParams.screenMidPos[0], dofParams.screenMidPos[1], dofParams.screenMidPos[2]);
	vec3 toScreenMidPos = vec3(dofParams.toScreenMidPosAlpha[0], dofParams.toScreenMidPosAlpha[1], dofParams.toScreenMidPosAlpha[2]);

	if ((toScreenMidPos - screenMidPos).length() > 1000.0f)
		toScreenMidPos += (screenMidPos - toScreenMidPos) * 0.9f;
	else
		toScreenMidPos += (screenMidPos - toScreenMidPos) * 0.1f;

	dofParams.toScreenMidPosAlpha[0] = toScreenMidPos.x;
	dofParams.toScreenMidPosAlpha[1] = toScreenMidPos.y;
	dofParams.toScreenMidPosAlpha[2] = toScreenMidPos.z;
	dofParams.toScreenMidPosAlpha[3] = inpAlpha;
	dofParamsBuf.UploadSubData(0, &dofParams, sizeof(dofParams));
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::SetMidScreenMessage(bool mainPlayerOnLadder)
{
	if (mainPlayerOnLadder)
	{
		if (curMidScreenTextType != CLIMB)
		{
			SDL_Scancode useScanCode;
			bool foundScanCode;
			foundScanCode = GetScanCodeForAction(CMD_USE, useScanCode);
			std::string midScrText;
			if (!foundScanCode)
				midScrText = "Use key is unbound. Bind and press it to climb...";
			else
			{
				midScrText = "                                 ";
				midScrText += "Press '";
				midScrText += SDL_GetScancodeName(useScanCode);
				midScrText += "' to climb...";
			}

			curMidScreenTextType = CLIMB;
			sprintf_s((char *)midScreenText, 90, midScrText.c_str());
			midScreenTextImage.UploadData(midScreenText, 30 * 3, true);
		}
		midScreenAlphaTarget = 1.0f;
	}
	else
		midScreenAlphaTarget = 0.0f;

	dofParams.midScreenAlpha += (midScreenAlphaTarget - dofParams.midScreenAlpha) * 0.1f;
	if (fabs(dofParams.midScreenAlpha - midScreenAlphaTarget) < 0.1f)
	{
		dofParams.midScreenAlpha = midScreenAlphaTarget;
		if (midScreenAlphaTarget == 0.0f && curMidScreenTextType != NONE)
		{
			curMidScreenTextType = NONE;
			memset(midScreenText, (int)(' '), 30 * 3);
			midScreenTextImage.UploadData(midScreenText, 30 * 3, true);
		}
	}
}

void HIGHOMEGA::RENDER::PASSES::SaurayDisplayTestClass::Create(TriClass & PostProcessTri, SaurayTraceClass & SaurayTrace)
{
	submission.Add(PostProcessTri.triModel);
	submission.Create(Instance);

	submission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/saurayTestOutput.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, SaurayTrace.testOutput);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::SaurayDisplayTestClass::Render()
{
	if (!HIGHOMEGA::EVENTS::windowMinimized) submission.Render();
}

void HIGHOMEGA::RENDER::PASSES::SplashDisplayClass::Create(TriClass & PostProcessTri)
{
	splashFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, (float)ScreenSize.width / (float)ScreenSize.height, 0.1f, 100.0f);
	splashFrustum.Update();

	std::function<bool(int, DataGroup &)> inpFilterFunction = [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
	};

	Mesh creditsMesh = Mesh("assets/credits/credits.3md");
	credits.Model(creditsMesh, "assets/credits/", Instance, inpFilterFunction, nullptr, false);

	mat4 creditsMat;
	creditsMat.Ident();
	creditsMat.i[0][0] = ((float)ScreenSize.width / (float)ScreenSize.height) / 1.777777777f; // credits is kinda 1080p-ish in terms of whr
	splashParams.alphaAmount = 1.0f;
	splashParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &splashParams, (unsigned int)sizeof(splashParams));

	renderItem = submission.Add(credits);
	renderItem.item->transformVertsSlow(creditsMat);
	submission.Create(Instance);

	submission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/splashShader.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, splashFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, splashParamsBuf);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::SplashDisplayClass::Render()
{
	splashParams.alphaAmount = min(timePassed / 4.0f, 1.0f);
	splashParamsBuf.UploadSubData(0, &splashParams, sizeof(splashParams));

	splashTimer.Start();

	if (!HIGHOMEGA::EVENTS::windowMinimized) submission.Render();

	timePassed += (float)splashTimer.Diff();
}

mat4 HIGHOMEGA::RENDER::PASSES::MainMenuClass::getCursorMat()
{
	mat4 cursorMat;
	cursorMat.Ident();
	cursorMat.i[0][0] = cursorMat.i[1][1] = cursorMat.i[2][2] = 0.05f;
	cursorMat.i[1][1] = -cursorMat.i[1][1];
	cursorMat.i[0][3] = cursorPos.x;
	cursorMat.i[1][3] = cursorPos.y;
	return cursorMat;
}

mat4 HIGHOMEGA::RENDER::PASSES::MainMenuClass::getItemMat(float itemY)
{
	mat4 itemMat;
	itemMat.Ident();
	itemMat.i[0][0] = itemMat.i[1][1] = itemMat.i[2][2] = 0.8f;
	itemMat.i[0][0] *= 0.7f;
	itemMat.i[0][3] = 0.4f;
	itemMat.i[1][3] = itemY;
	return itemMat;
}

bool HIGHOMEGA::RENDER::PASSES::MainMenuClass::overButton(vec2 buttonPos, vec2 buttomDim)
{
	if (fabs(cursorPos.x - buttonPos.x) < buttomDim.x &&  fabs(cursorPos.y - buttonPos.y) < buttomDim.y)
		return true;
	else
		return false;
}

void HIGHOMEGA::RENDER::PASSES::MainMenuClass::Create(TriClass & PostProcessTri, bool cmdOptHwRt, unsigned int cmdOptFullRes, bool cmdOptWindowed)
{
	this->hwrtSelection = cmdOptHwRt;
	this->fullResSelection = cmdOptFullRes;
	this->windowedSelection = cmdOptWindowed;

	mouseMovementConsumers["MainMenuCursor"] = vec2(0.0f);

	menuFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, (float)ScreenSize.width / (float)ScreenSize.height, 0.1f, 100.0f);
	menuFrustum.Update();

	menuParams.alphaAmount = 1.0f;
	menuParamsBuf.Buffer(MEMORY_HOST_VISIBLE, SHARING_DEFAULT, MODE_CREATE, USAGE_UBO, Instance, &menuParams, (unsigned int)sizeof(menuParams));

	std::function<bool(int, DataGroup &)> inpFilterFunction = [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
	};

	Mesh cursorMesh = Mesh("assets/mainmenu/cursor.3md");
	Mesh mainMenuMesh = Mesh("assets/mainmenu/mainmenu.3md");
	Mesh hwrtbasedMesh = Mesh("assets/mainmenu/hwrtbased.3md");
	Mesh sdfbvhbasedMesh = Mesh("assets/mainmenu/sdfbvhbased.3md");
	Mesh windowedMesh = Mesh("assets/mainmenu/windowed.3md");
	Mesh fullscreenMesh = Mesh("assets/mainmenu/fullscreen.3md");
	Mesh loadingMesh = Mesh("assets/mainmenu/loading.3md");
	cursor.Model(cursorMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	mainmenu.Model(mainMenuMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	hwrtbased.Model(hwrtbasedMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	sdfbvhbased.Model(sdfbvhbasedMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	windowed.Model(windowedMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	fullscreen.Model(fullscreenMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	loading.Model(loadingMesh, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	for (int i = 0; i != allowedResolutions.size(); i++)
	{
		std::string resModelName = "assets/mainmenu/";
		resModelName += std::to_string((int)allowedResolutions[i].x);
		resModelName += "x";
		resModelName += std::to_string((int)allowedResolutions[i].y);
		resModelName += ".3md";
		Mesh resMeshName = Mesh(resModelName);
		screenRes[i].Model(resMeshName, "assets/mainmenu/", Instance, inpFilterFunction, nullptr, false);
	}

	cursorPos = vec2(0.0f);

	cursorItem = submission.Add(cursor);
	mat4 cursorItemMat = getCursorMat();
	cursorItem.item->transformVertsSlow(cursorItemMat);

	techniqueItemMat = getItemMat(0.1825f - 0.1125f * 0.0f);
	screenResItemMat = getItemMat(0.1825f - 0.1125f * 1.0f);
	windowedItemMat = getItemMat(0.1825f - 0.1125f * 2.0f);

	if (Instance.SupportsHWRT() && this->hwrtSelection)
	{
		techniqueItem = submission.Add(hwrtbased);
	}
	else
	{
		this->hwrtSelection = false;
		techniqueItem = submission.Add(sdfbvhbased);
	}
	screenResItem = submission.Add(screenRes[this->fullResSelection]);
	techniqueItem.item->transformVertsSlow(techniqueItemMat);
	screenResItem.item->transformVertsSlow(screenResItemMat);
	if (this->windowedSelection)
		windowedItem = submission.Add(windowed);
	else
		windowedItem = submission.Add(fullscreen);
	windowedItem.item->transformVertsSlow(windowedItemMat);
	mainmenuItem = submission.Add(mainmenu);
	submission.Create(Instance);

	submission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/splashShader.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, menuFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, menuParamsBuf);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::MainMenuClass::Render()
{
	vec2 cursorDelta = mouseMovementConsumers["MainMenuCursor"];
	mouseMovementConsumers["MainMenuCursor"] = vec2(0.0f);
	cursorPos += cursorDelta * 0.002f;
	cursorPos.x = min(max(cursorPos.x, -menuFrustum.screen_whr), menuFrustum.screen_whr);
	cursorPos.y = min(max(cursorPos.y, -1.0f), 1.0f);

	mat4 cursorMat = getCursorMat();
	cursorItem.item->transformVertsSlow(cursorMat);

	bool curLeftMouseDown = HIGHOMEGA::EVENTS::leftMouseDown;
	bool clickDone = (!curLeftMouseDown && prevLeftMouseDown);

	float techniqueItemButtonY = 0.15f - 0.11f * 0.0f;
	float screenResItemButtonY = 0.15f - 0.11f * 1.0f;
	float windowedItemButtonY = 0.15f - 0.11f * 2.0f;

	if ( (overButton(vec2(0.172f, techniqueItemButtonY), vec2(0.025f)) || overButton(vec2(0.658f, techniqueItemButtonY), vec2(0.025f))) && clickDone && Instance.SupportsHWRT())
	{
		hwrtSelection = !hwrtSelection;
		submission.Remove(techniqueItem);
		if (hwrtSelection)
			techniqueItem = submission.Add(hwrtbased);
		else
			techniqueItem = submission.Add(sdfbvhbased);
		techniqueItem.item->transformVertsSlow(techniqueItemMat);
	}

	if ((overButton(vec2(0.172f, screenResItemButtonY), vec2(0.025f)) || overButton(vec2(0.658f, screenResItemButtonY), vec2(0.025f))) && clickDone )
	{
		if (overButton(vec2(0.172f, screenResItemButtonY), vec2(0.025f))) fullResSelection--;
		else if (overButton(vec2(0.658f, screenResItemButtonY), vec2(0.025f))) fullResSelection++;
		if (fullResSelection == -1) fullResSelection = 4;
		else if (fullResSelection == 5) fullResSelection = 0;
		submission.Remove(screenResItem);
		screenResItem = submission.Add(screenRes[fullResSelection]);
		screenResItem.item->transformVertsSlow(screenResItemMat);
	}

	if ((overButton(vec2(0.172f, windowedItemButtonY), vec2(0.025f)) || overButton(vec2(0.658f, windowedItemButtonY), vec2(0.025f))) && clickDone)
	{
		windowedSelection = !windowedSelection;
		submission.Remove(windowedItem);
		if (windowedSelection)
			windowedItem = submission.Add(windowed);
		else
			windowedItem = submission.Add(fullscreen);
		windowedItem.item->transformVertsSlow(windowedItemMat);
	}

	if (overButton(vec2(-0.316f, -0.624f), vec2(0.147777f, 0.038f)) && clickDone)
	{
		std::string reLaunchCmdLine = "highomega";
		reLaunchCmdLine += " ";
		reLaunchCmdLine += hwrtSelection ? "hwrt" : "sdfbvh";
		reLaunchCmdLine += " ";
		reLaunchCmdLine += std::to_string((int)allowedResolutions[fullResSelection].x);
		reLaunchCmdLine += "x";
		reLaunchCmdLine += std::to_string((int)allowedResolutions[fullResSelection].y);
		reLaunchCmdLine += " ";
		reLaunchCmdLine += windowedSelection ? "windowed" : "fullscreen";
#ifdef WIN32
		STARTUPINFO info = { sizeof(info) };
		PROCESS_INFORMATION processInfo;
		CreateProcess(NULL, const_cast<char *>(reLaunchCmdLine.c_str()), NULL, NULL, TRUE, 0, NULL, NULL, &info, &processInfo);
#endif

		rebooting = true;
		done = true;
		return;
	}

	if (overButton(vec2(0.322f, -0.624f), vec2(0.147777f, 0.038f)) && clickDone)
	{
		submission.Remove(cursorItem);
		submission.Remove(techniqueItem);
		submission.Remove(screenResItem);
		submission.Remove(windowedItem);
		submission.Remove(mainmenuItem);
		mat4 midMat;
		midMat.Ident();
		midMat.i[0][0] = 0.56f;
		submission.Add(loading);
		loading.transformVertsSlow(midMat);
		done = true;
	}

	if (!HIGHOMEGA::EVENTS::windowMinimized) submission.Render();

	prevLeftMouseDown = curLeftMouseDown;
}

bool HIGHOMEGA::RENDER::PASSES::MainMenuClass::IsDone()
{
	return done;
}

bool HIGHOMEGA::RENDER::PASSES::MainMenuClass::IsRebooting()
{
	return rebooting;
}