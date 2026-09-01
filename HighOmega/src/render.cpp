/*
	Copyright (c) 2026 TooMuchVoltage Software Inc.

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
std::function<bool(const MeshMaterial& curMat)> HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::defaultFilterFunction;

void HIGHOMEGA::RENDER::PASSES::GetCubeFaceLookUp(unsigned int faceIdx, vec3& look, vec3& up)
{
	switch (faceIdx)
	{
		case 0:
		{
			look = vec3(1.0f, 0.0f, 0.0f);
			up = vec3(0.0f, 1.0f, 0.0f);
			break;
		}
		case 1:
		{
			look = vec3(-1.0f, 0.0f, 0.0f);
			up = vec3(0.0f, 1.0f, 0.0f);
			break;
		}
		case 2:
		{
			look = vec3(0.0f, 1.0f, 0.0f);
			up = vec3(0.0f, 0.0f, 1.0f);
			break;
		}
		case 3:
		{
			look = vec3(0.0f, -1.0f, 0.0f);
			up = vec3(0.0f, 0.0f, -1.0f);
			break;
		}
		case 4:
		{
			look = vec3(0.0f, 0.0f, -1.0f);
			up = vec3(0.0f, 1.0f, 0.0f);
			break;
		}
		case 5:
		default:
		{
			look = vec3(0.0f, 0.0f, 1.0f);
			up = vec3(0.0f, 1.0f, 0.0f);
			break;
		}
	}
}

vec3 HIGHOMEGA::RENDER::TEXT::MakeGeomFromText(const std::string& text, const vec2& glyphSize, const vec3& offset, std::vector<unsigned char>& outIndexVertexData)
{
	unsigned int iOffset = 0u;

	std::vector<RasterVertex> vertData;
	std::vector<unsigned int> indexData;
	indexData.reserve(text.size() * 6);
	vertData.reserve(text.size() * 4);

	vec3 curPos = offset - vec3(0.0f, glyphSize.y, 0.0f), curBottomCorner, maxPos = vec3(0.0f);
	vec3 initPos = curPos;
	RasterVertex rv;
	vec3 textNorm = vec3(0.0f, 0.0f, 1.0f);
	vec3 textVertCol = vec3(0.0f);
	for (unsigned int i = 0; i != text.size(); i++)
	{
		unsigned char curChar = text.c_str()[i];
		if (curChar == '\n')
		{
			curPos.x = offset.x;
			curPos.y -= glyphSize.y;
			continue;
		}
		vec2 curCharUVStart = vec2((curChar % 16) * 0.0625f, (15 - (curChar / 16)) * 0.0625f);

		packRasterVertex(curPos, textVertCol, curCharUVStart, textNorm, rv);
		vertData.push_back(rv);
		packRasterVertex(curPos + vec3(glyphSize.x, 0.0, 0.0f), textVertCol, curCharUVStart + vec2(0.0625f, 0.0f), textNorm, rv);
		vertData.push_back(rv);
		packRasterVertex((curBottomCorner = curPos + vec3(glyphSize.x, glyphSize.y, 0.0f)), textVertCol, curCharUVStart + vec2(0.0625f), textNorm, rv);
		vertData.push_back(rv);
		packRasterVertex(curPos + vec3(0.0f, glyphSize.y, 0.0f), textVertCol, curCharUVStart + vec2(0.0f, 0.0625f), textNorm, rv);
		vertData.push_back(rv);

		indexData.push_back(iOffset);
		indexData.push_back(iOffset + 1);
		indexData.push_back(iOffset + 3);
		indexData.push_back(iOffset + 1);
		indexData.push_back(iOffset + 2);
		indexData.push_back(iOffset + 3);

		maxPos.x = max(maxPos.x, fabs(curBottomCorner.x - initPos.x));
		maxPos.y = max(maxPos.y, fabs(curBottomCorner.y - initPos.y));

		iOffset += 4;

		curPos.x += glyphSize.x;
	}

	createIndicesVertices(indexData, vertData, outIndexVertexData);

	return maxPos;
}

extern HIGHOMEGA::INSTRUMENTATION::Instrument updateInstrument, traceInstrument, frameInstrument;
namespace HIGHOMEGA::GL
{
	std::mutex texture_mutex;
}

CacheItem<ImageClass> * HIGHOMEGA::RENDER::AddOrFindCachedTexture(const std::string& belong, const std::string& texName, InstanceClass & ptrToInstance, bool isArray, int nLayers, bool mipmap, bool useSRGB)
{
	CacheItem<ImageClass>* retVal = nullptr;

	std::unique_lock<std::mutex> lk(texture_mutex, std::defer_lock);
	lk.lock();

	if (TextureCache.find(texName) == TextureCache.end())
	{
		bool loadResult;
		TextureCache[texName].preparingOnAnotherThread = true;
		lk.unlock();

		ImageClass* tmpTexture = new ImageClass;

		if (isArray)
			loadResult = tmpTexture->CreateTexture(ptrToInstance, belong, texName, nLayers, false, true, false, mipmap, useSRGB ? R8G8B8A8SRGB : R8G8B8A8UN);
		else
			loadResult = tmpTexture->CreateTexture(ptrToInstance, belong, texName, 1U, false, false, false, mipmap, useSRGB ? R8G8B8A8SRGB : R8G8B8A8UN);
		if (!loadResult)
		{
			delete tmpTexture;
			lk.lock();
			TextureCache.erase(texName);
			lk.unlock();
			return nullptr;
		}
		else
		{
			lk.lock();
			TextureCache[texName].elem = std::move(*tmpTexture);
			operator delete(tmpTexture);
			TextureCache[texName].elemCount = 1;
			TextureCache[texName].preparingOnAnotherThread = false;
		}
	}
	else
	{
		if (TextureCache[texName].preparingOnAnotherThread)
		{
			lk.unlock();
			while (true)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				lk.lock();
				if (TextureCache.find(texName) == TextureCache.end())
				{
					lk.unlock();
					return nullptr;
				}
				else
				{
					if (!TextureCache[texName].preparingOnAnotherThread)
					{
						retVal = &TextureCache[texName];
						TextureCache[texName].elemCount++;
						lk.unlock();
						return retVal;
					}
				}
				lk.unlock();
			}
		}
		TextureCache[texName].elemCount++;
	}
	retVal = &TextureCache[texName];
	lk.unlock();
	return retVal;
}

void HIGHOMEGA::RENDER::TransformMesh(Mesh & mesh, mat4 & inpTransform)
{
	mat4 inpTransformDT = inpTransform.DirectionTransform();
	std::vector<std::thread *> transformThreads;
	for (unsigned int i = 0; i != mesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup& curPolyGroup = mesh.DataGroups[i];
		transformThreads.push_back(new std::thread([&]() {
			mat4 instTrans, newTrans;
			vec3 pos, col, dir, vNorm;
			vec2 uv;
			HIGHOMEGA::MESH::DataBlock* triBlock = nullptr;
			if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock))
			{
				HIGHOMEGA::MESH::DataBlock* descBlock = nullptr;
				if (Mesh::getDataBlock(curPolyGroup, "DESCRIPTION", &descBlock))
				{
					if (Mesh::getDataRowVec3(*descBlock, "pos", pos))
					{
						pos = inpTransform * pos;
						if (!Mesh::setDataRowVec3(*descBlock, "pos", pos)) FATAL_ERROR("Failed to transform desc. pos during mesh transform");
					}
					if (Mesh::getDataRowVec3(*descBlock, "dir", dir))
					{
						dir = inpTransformDT * dir;
						if (!Mesh::setDataRowVec3(*descBlock, "dir", dir)) FATAL_ERROR("Failed to transform desc. dir during mesh transform");
					}
				}
				return;
			}

			if (triBlock->mode != "blob") FATAL_ERROR("Mesh's tri block is not a blob");

			HIGHOMEGA::MESH::DataBlock* instBlock = nullptr;
			if (Mesh::getDataBlock(curPolyGroup, "INSTANCES", &instBlock))
			{
				unsigned int instCount = (unsigned int)instBlock->blob.size() / (sizeof(float) * 16);
				for (unsigned int j = 0; j != instCount; j++)
				{
					PackMat4(&instBlock->blob.data()[sizeof(float) * 16 * j], instTrans);
					newTrans = inpTransform * instTrans;
					UnpackMat4(newTrans, &instBlock->blob.data()[sizeof(float) * 16 * j]);
				}
			}
			else
			{
				RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
				getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);
				for (unsigned int j = 0; j != vertCount; j++)
				{
					unpackRasterVertex(pos, col, uv, vNorm, verts[j]);
					pos = inpTransform * pos;
					vNorm = inpTransformDT * vNorm;
					packRasterVertex(pos, col, uv, vNorm, verts[j]);
				}
			}
		}));
	}
	for (std::thread* curTransThread : transformThreads)
		curTransThread->join();
	for (std::thread* curTransThread : transformThreads)
		delete curTransThread;
	transformThreads.clear();
}

void HIGHOMEGA::RENDER::GetMeshMinMax(Mesh& mesh, vec3& outMin, vec3& outMax)
{
	outMin = vec3(FLT_MAX);
	outMax = vec3(-FLT_MAX);
	for (unsigned int i = 0; i != mesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup& curPolyGroup = mesh.DataGroups[i];
		mat4 instTrans;
		vec3 pos, col, dir, vNorm;
		vec2 uv;
		HIGHOMEGA::MESH::DataBlock* triBlock = nullptr;
		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock))
		{
			HIGHOMEGA::MESH::DataBlock* descBlock = nullptr;
			if (Mesh::getDataBlock(curPolyGroup, "DESCRIPTION", &descBlock))
			{
				if (Mesh::getDataRowVec3(*descBlock, "pos", pos))
				{
					outMin.x = min(outMin.x, pos.x); outMin.y = min(outMin.y, pos.y); outMin.z = min(outMin.z, pos.z);
					outMax.x = max(outMax.x, pos.x); outMax.y = max(outMax.y, pos.y); outMax.z = max(outMax.z, pos.z);
				}
			}
			continue;
		}

		if (triBlock->mode != "blob") FATAL_ERROR("Mesh's tri block is not a blob");

		HIGHOMEGA::MESH::DataBlock* instBlock = nullptr;
		if (Mesh::getDataBlock(curPolyGroup, "INSTANCES", &instBlock))
		{
			unsigned int instCount = (unsigned int)instBlock->blob.size() / (sizeof(float) * 16);
			for (unsigned int j = 0; j != instCount; j++)
			{
				PackMat4(&instBlock->blob.data()[sizeof(float) * 16 * j], instTrans);
				RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
				getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);
				for (unsigned int k = 0; k != vertCount; k++)
				{
					unpackRasterVertex(pos, col, uv, vNorm, verts[k]);
					pos = instTrans * pos;
					outMin.x = min(outMin.x, pos.x); outMin.y = min(outMin.y, pos.y); outMin.z = min(outMin.z, pos.z);
					outMax.x = max(outMax.x, pos.x); outMax.y = max(outMax.y, pos.y); outMax.z = max(outMax.z, pos.z);
				}
			}
		}
		else
		{
			RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
			getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);
			for (unsigned int j = 0; j != vertCount; j++)
			{
				unpackRasterVertex(pos, col, uv, vNorm, verts[j]);
				outMin.x = min(outMin.x, pos.x); outMin.y = min(outMin.y, pos.y); outMin.z = min(outMin.z, pos.z);
				outMax.x = max(outMax.x, pos.x); outMax.y = max(outMax.y, pos.y); outMax.z = max(outMax.z, pos.z);
			}
		}
	}
}

void HIGHOMEGA::RENDER::InitGraphicsSubSystem(bool requestHWRT, WINDOW_MODE windowMode, bool headless)
{
	std::string window_title;
	window_title = std::string("HighOmega " + std::to_string(HIGHOMEGA_VERSION));
	if (!headless) Window.Make(window_title.c_str(), 100, 100, ScreenSize.width, ScreenSize.height, (WindowClass::GL_WINDOWED_MODE)windowMode);
	Instance.Make(false, Window, requestHWRT, headless);
	if (!headless) Instance.CreateSwapChain(Window);
}

void HIGHOMEGA::RENDER::WindowRecreate(unsigned int width, unsigned int height, WINDOW_MODE windowMode)
{
	ScreenSize.Create(width, height);
	MainFrustum.screen_whr = (float)ScreenSize.width / (float)ScreenSize.height;
	MainFrustum.Update();
	Window.Recreate(width, height, (WindowClass::GL_WINDOWED_MODE)windowMode);
	Instance.CreateSwapChain(Window);
	Instance.RecreateSwapchainDependentImages(width, height);
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
		(isDecal == other.isDecal) &&
		(postProcess == other.postProcess) &&
		(backDropGlass == other.backDropGlass) &&
		(holographicInParticleScene == other.holographicInParticleScene) &&
		(perVertexVelocity == other.perVertexVelocity) &&
		(parallaxOcclusionMapping == other.parallaxOcclusionMapping) &&
		(scattering == other.scattering) &&
		(playerId == other.playerId) &&
		(rayMask == other.rayMask) &&
		(renderOrder == other.renderOrder);
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

void HIGHOMEGA::RENDER::MeshMaterial::ReduceClaims(bool evictAtZero)
{
	std::lock_guard<std::mutex> lk(texture_mutex);
	if (diffRef)
	{
		diffRef->elemCount--;
		if (!diffRef->elemCount && evictAtZero)
		{
			TextureCache.erase(diffName);
			diffRef = nullptr;
			diffName = "";
		}
	}
	if (nrmRef)
	{
		nrmRef->elemCount--;
		if (!nrmRef->elemCount && evictAtZero)
		{
			TextureCache.erase(nrmName);
			nrmRef = nullptr;
			nrmName = "";
		}
	}
	if (rghRef)
	{
		rghRef->elemCount--;
		if (!rghRef->elemCount && evictAtZero)
		{
			TextureCache.erase(rghName);
			rghRef = nullptr;
			rghName = "";
		}
	}
	if (hgtRef)
	{
		hgtRef->elemCount--;
		if (!hgtRef->elemCount && evictAtZero)
		{
			TextureCache.erase(hgtName);
			hgtRef = nullptr;
			hgtName = "";
		}
	}
	if (spcRef)
	{
		spcRef->elemCount--;
		if (!spcRef->elemCount && evictAtZero)
		{
			TextureCache.erase(spcName);
			spcRef = nullptr;
			spcName = "";
		}
	}
}

HIGHOMEGA::RENDER::MeshMaterial::MeshMaterial()
{
}

CacheItem<ImageClass>* HIGHOMEGA::RENDER::MeshMaterial::TryDifferentLODs(const std::string& belong, const std::string& texName, InstanceClass& ptrToInstance, bool isArray, int nLayers, bool mipmap, bool useSRGB, bool loadLowRes)
{
	std::string textureName = texName;

	size_t extIndex = textureName.rfind(".tga");
	if (extIndex == std::string::npos) FATAL_ERROR("Extension not supported: " + textureName);

	textureName.replace(extIndex, std::string(".tga").length(), ".ktx");
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

HIGHOMEGA::RENDER::MeshMaterial::MeshMaterial(HIGHOMEGA::MESH::DataBlock & propBlock, const std::string& belong, InstanceClass & ptrToInstance)
{
	if (!Mesh::getDataRowString(propBlock, "texname", diffName)) FATAL_ERROR("Bad diffName fetch for graphics model load");

	isTerrain = false;
	if (diffName.find("{terrain}") != std::string::npos) isTerrain = true;

	mipmap = true;
	if (diffName.find("{nomip}") != std::string::npos) mipmap = false;

	diffRef = TryDifferentLODs(belong, diffName, ptrToInstance, true, isTerrain ? 4 : 1, mipmap, true);
	if (!diffRef) FATAL_ERROR("Could not find albedo map: " + diffName);

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

	smooth = 1.0f;
	float flatValue;
	if (Mesh::getDataRowFloat(propBlock, "flat", flatValue))
		smooth = (flatValue == 1.0f) ? 0.0f : 1.0f;

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
		EnableAdditiveBlend();
	}

	isAlphaBlending = false;
	if (Mesh::getDataRowFloat(propBlock, "alphaBlend", tmpFloat))
	{
		EnableAlphaBlend();
	}

	postProcess = Mesh::getDataRowFloat(propBlock, "isPostProcess", tmpFloat);

	isAlphaKeyed = Mesh::getDataRowFloat(propBlock, "isAlphaKeyed", tmpFloat);

	isDecal = Mesh::getDataRowFloat(propBlock, "isDecal", tmpFloat);

	backDropGlass = Mesh::getDataRowFloat(propBlock, "isBackDropGlass", tmpFloat);

	holographicInParticleScene = Mesh::getDataRowFloat(propBlock, "isHolographicInParticleScene", tmpFloat);

	perVertexVelocity = Mesh::getDataRowFloat(propBlock, "perVertexVelocity", tmpFloat);

	isViewerRelative = Mesh::getDataRowFloat(propBlock, "viewerRelative", tmpFloat);

	parallaxOcclusionMapping = false;
	if (Mesh::getDataRowFloat(propBlock, "parallaxOcclusionMapping", tmpFloat))
	parallaxOcclusionMapping = (tmpFloat == 1.0f);

	holographicInParticleScene = Mesh::getDataRowFloat(propBlock, "isHolographicInParticleScene", tmpFloat);
}

HIGHOMEGA::RENDER::MeshMaterial::MeshMaterial(const std::string& inDiffName, const std::string& belong, InstanceClass& ptrToInstance, bool noBumpClaims)
{
	diffName = inDiffName;

	isTerrain = false;
	if (diffName.find("{terrain}") != std::string::npos) isTerrain = true;

	mipmap = true;
	if (diffName.find("{nomip}") != std::string::npos) mipmap = false;

	diffRef = TryDifferentLODs(belong, diffName, ptrToInstance, true, isTerrain ? 4 : 1, mipmap, true);
	if (!diffRef) FATAL_ERROR("Could not find albedo map: " + diffName);

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

	if (noBumpClaims) ReduceClaims(false);
}

void HIGHOMEGA::RENDER::MeshMaterial::EnableAlphaBlend()
{
	pipelineFlags.blendEnable = true;
	pipelineFlags.alphaBlending = true;
	pipelineFlags.colorBlending = true;
	pipelineFlags.srcAlphaFactor = FACTOR_ONE;
	pipelineFlags.dstAlphaFactor = FACTOR_ONE;
	pipelineFlags.srcColorFactor = FACTOR_SRC_ALPHA;
	pipelineFlags.dstColorFactor = FACTOR_ONE_MINUS_SRC_ALPHA;
	pipelineFlags.alphaBlendOp = BLEND_ADD;
	pipelineFlags.colorBlendOp = BLEND_ADD;

	pipelineFlags.changedBlendEnable = true;
	isAlphaBlending = true;
}

void HIGHOMEGA::RENDER::MeshMaterial::EnableAdditiveBlend()
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
		^ hash<bool>()(k.isDecal)
		^ hash<bool>()(k.postProcess)
		^ hash<bool>()(k.backDropGlass)
		^ hash<bool>()(k.parallaxOcclusionMapping)
		^ hash<bool>()(k.holographicInParticleScene)
		^ hash<bool>()(k.scattering)
		^ hash<unsigned int>()(k.playerId)
		^ hash<unsigned char>()(k.rayMask)
		^ hash<bool>()(k.perVertexVelocity)
		^ hash<int>()(k.renderOrder);
}

void HIGHOMEGA::RENDER::Pose::Inv()
{
	for (Bone & curBone : pose)
		curBone.bone = curBone.bone.Inv();
}

void HIGHOMEGA::RENDER::Pose::Mul(Pose & rhs)
{
	if (rhs.pose.size() != pose.size()) FATAL_ERROR("rhs pose should be the same size as what it is mul'd by");

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
	fract = Clamp (fract, 0.0f, 1.0f);

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
			return;
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
			return;
		}
	}
}

void HIGHOMEGA::RENDER::AnimationAndSnapshot::CopyToPoseBuffer(Pose & inpPose)
{
	unsigned int rawPoseDataSize = (unsigned int)inpPose.pose.size() * 16 * sizeof(float);

	if (!currentAndPrevPoseBuf || currentAndPrevPoseBuf->getSize() / 2 != rawPoseDataSize) return;

	if (!rawPoseData) rawPoseData = new unsigned char[rawPoseDataSize];

	unsigned int dataOffset = 0;
	for (Bone & curBone : inpPose.pose)
	{
		UnpackMat4(curBone.bone, (void *)&rawPoseData[dataOffset]);
		dataOffset += sizeof(float) * 16;
	}

	currentAndPrevPoseBuf->CopyToBuffer(Instance, *currentAndPrevPoseBuf, rawPoseDataSize, 0u, rawPoseDataSize);
	currentAndPrevPoseBuf->UploadSubData(0, rawPoseData, rawPoseDataSize);
}

HIGHOMEGA::RENDER::AnimationAndSnapshot::~AnimationAndSnapshot()
{
	if (currentAndPrevPoseBuf) delete currentAndPrevPoseBuf;
	if (rawPoseData) delete rawPoseData;

	currentAndPrevPoseBuf = nullptr;
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

void HIGHOMEGA::RENDER::GraphicsModel::UpdateSDFs(std::vector<GraphicsModel *>& updateItems, bool forceRefresh, std::vector<std::string> cacheNames)
{
	if (RTInstance::Enabled()) return;
	if (!GroupedSDFBVHSubmission::defaultFilterFunction) return; // Too early. Before even main game.

	std::vector<ComputeSubmission*> csSet;
	std::vector<ShaderResourceSet *> srsSet;
	std::vector<BufferClass *> allVoxelizeParamBufs;
	std::vector<std::pair<ImageClass *, bool>> distFieldGenList;
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
		GraphicsModel* curPair = updateItems[i];
		if (curPair->MaterialGeomMap.size() == 0)
		{
			if (curPair->sdf) delete curPair->sdf;
			curPair->sdf = nullptr;
			continue;
		}
		vec3 modMin, modMax;
		curPair->getModelMinMax(modMin, modMax);
		modMin -= vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		modMax += vec3(HIGHOMEGA_ZONE_VOXELIZE_COARSENESS);
		voxelizeParams.imgMinTriCount[0] = modMin.x;
		voxelizeParams.imgMinTriCount[1] = modMin.y;
		voxelizeParams.imgMinTriCount[2] = modMin.z;
		voxelizeParams.imgMaxVoxelSize[0] = modMax.x;
		voxelizeParams.imgMaxVoxelSize[1] = modMax.y;
		voxelizeParams.imgMaxVoxelSize[2] = modMax.z;
		voxelizeParams.imgMaxVoxelSize[3] = HIGHOMEGA_ZONE_VOXELIZE_COARSENESS;

		if (!curPair->sdf)
		{
			curPair->sdf = new ImageClass();
			curPair->sdf->CreateImageStore(Instance, R8G8B8A8UN, (int)ceilf((modMax.x - modMin.x) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS), (int)ceilf((modMax.y - modMin.y) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS), (int)ceilf((modMax.z - modMin.z) / HIGHOMEGA_ZONE_VOXELIZE_COARSENESS), _3D, false);
			unsigned char* cacheContent = nullptr;
			unsigned int cacheSize = 0;
			ResourceLoader::LOAD_LOCATION loadLocation;
			if (cacheNames.size() > 0 && ResourceLoader::Load("", cacheNames[i], &cacheContent, cacheSize, loadLocation) == ResourceLoader::RESOURCE_LOAD_RESULT::RESOURCE_LOAD_SUCCESS)
			{
				curPair->sdf->UploadData(cacheContent, cacheSize);
				curPair->sdf->FreeLoadedData();
				continue;
			}
		}
		else
			if (!forceRefresh) continue;
		std::vector<ImageClass *> clearList;
		clearList.push_back(curPair->sdf);
		bool singleBreakablePiece = (curPair->MaterialGeomMap.size() == 1 && curPair->MaterialGeomMap.begin()->second.size() == 1 && curPair->MaterialGeomMap.begin()->second.begin()->getBreakable());
		distFieldGenList.emplace_back(curPair->sdf, singleBreakablePiece);
		if (cacheNames.size() > 0) distFieldCacheNames.push_back(cacheNames[i]);
		curPair->sdf->ClearColors(clearList, ImageClearColor(vec3(0.0f), 0.0f));

		HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6];
		for (std::pair <const MeshMaterial, std::list<GeometryClass>> & matGeomPair : curPair->MaterialGeomMap)
		{
			MeshMaterial curMat = matGeomPair.first;
			if (!GroupedSDFBVHSubmission::defaultFilterFunction(curMat)) continue;
			for (GeometryClass & curGeom : matGeomPair.second)
			{
				unsigned int triCount = curGeom.getTriCount();
				voxelizeParams.imgMinTriCount[3] = *((float*)&triCount);
				GroupedRenderSubmission::CompileInstanceProperties(voxelizeParams.instProps, curMat, textureOffsets, 0u, 0xFFFFFFFFu, curGeom);
				allVoxelizeParamBufs.push_back(new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &voxelizeParams, (unsigned int)sizeof(voxelizeParams)));
				srsSet.push_back(new ShaderResourceSet);
				srsSet.back()->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, *allVoxelizeParamBufs.back());
				srsSet.back()->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, *curPair->sdf, -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
				giantVertBufferSharedMutex.lock_shared();
				srsSet.back()->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, *giantVertBuffer);
				giantVertBufferSharedMutex.unlock_shared();
				srsSet.back()->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 3, matGeomPair.first.diffRef->elem);
				srsSet.back()->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 4, matGeomPair.first.rghRef ? matGeomPair.first.rghRef->elem : matGeomPair.first.diffRef->elem);
				srsSet.back()->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 5, matGeomPair.first.spcRef ? matGeomPair.first.spcRef->elem : matGeomPair.first.diffRef->elem);
				srsSet.back()->Create("shaders/voxelize.comp.spv", "main");
				std::stringstream ptrStream;
				ptrStream << &curGeom;
				csSet[0]->MakeDispatch(Instance, std::string("voxelize_") + ptrStream.str(), *srsSet.back(), (unsigned int)ceil((double)triCount / (double)(VoxelizeWorkGroupX() * TrianglePerThreadCount())), 1, 1);
			}
		}
	}
	(*csSet[0]).Submit();

	BufferClass JFAParamsBuf;
	JFAParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(JFAParams.offsetStage));
	for (unsigned int i = 0; i != distFieldGenList.size(); i++)
	{
		ImageClass* curImg = distFieldGenList[i].first;
		csSet.push_back(new ComputeSubmission);
		srsSet.push_back(new ShaderResourceSet);
		ImageClass JFAImg;
		JFAImg.CreateImageStore(Instance, R32G32B32A32UI, curImg->getWidth(), curImg->getHeight(), curImg->getDepth(), _3D, false);

		srsSet.back()->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, JFAParamsBuf);
		srsSet.back()->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, JFAImg, -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		srsSet.back()->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 2, *curImg);
		srsSet.back()->Create("shaders/jfa.comp.spv", "main");
		csSet.back()->MakeDispatch(Instance, std::string("JFA"), *srsSet.back(), (unsigned int)ceil((double)JFAImg.getWidth() / (double)JFAWorkGroupX()),
			(unsigned int)ceil((double)JFAImg.getHeight() / (double)JFAWorkGroupY()),
			(unsigned int)ceil((double)JFAImg.getDepth() / (double)JFAWorkGroupZ()));

		// Place seeds
		JFAParams.offsetStage[1] = 0;
		JFAParamsBuf.UploadSubData(0, JFAParams.offsetStage, sizeof(JFAParams));
		(*csSet.back()).Submit();

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
			(*csSet.back()).Submit();
			jfaIter++;
		}

		// Compute distance field
		JFAParams.offsetStage[1] = 2;
		JFAParamsBuf.UploadSubData(0, JFAParams.offsetStage, sizeof(JFAParams));
		(*csSet.back()).Submit();

		if (distFieldCacheNames.size() > 0 && !distFieldGenList[i].second) // Don't cache breakables
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
	for (GraphicsModelInstance* curInst : instances)
		delete curInst;
	instances.clear();

	idxVertCache.clear();
	armatures.clear();

	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
	{
		MeshMaterial curMat = it->first;
		curMat.ReduceClaims();
	}
	MaterialGeomMap.clear();
	if (sdf) delete sdf;
	sdf = nullptr;

	isInit = false;
	cachedModelMinMax = false;
}

void HIGHOMEGA::RENDER::GraphicsModel::GenerateGeom(std::vector<TriUV>& triList, std::vector<unsigned char>& outIndexVertexData)
{
	std::vector<RasterVertex> vertData;
	std::vector<unsigned int> indexData;
	indexData.reserve(triList.size() * 3);
	vertData.reserve(triList.size() * 3);

	for (unsigned int i = 0; i != triList.size() * 3; i++)
		indexData.push_back(i);

	for (TriUV & curTri : triList)
	{
		RasterVertex rv;
		packRasterVertex(curTri.eArr[0], curTri.colArr[0], curTri.uvArr[0], curTri.normVec, rv);
		vertData.push_back(rv);
		packRasterVertex(curTri.eArr[1], curTri.colArr[1], curTri.uvArr[1], curTri.normVec, rv);
		vertData.push_back(rv);
		packRasterVertex(curTri.eArr[2], curTri.colArr[2], curTri.uvArr[2], curTri.normVec, rv);
		vertData.push_back(rv);
	}

	createIndicesVertices(indexData, vertData, outIndexVertexData);
}

void HIGHOMEGA::RENDER::GraphicsModel::ExtractGeom(std::vector<unsigned char>& indexVertexData, std::vector<TriUV>& outTriList)
{
	RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
	getIndicesVertices(indexVertexData, &verts, vertCount, &indices, triCount, vertexDataOffset);

	for (unsigned int i = 0; i != triCount; i++)
	{
		vec3 edge[3], col[3], vnorm;
		vec2 uv[3];
		TriUV curTri;
		for (unsigned int j = 0; j != 3; j++)
		{
			unpackRasterVertex(edge[j], col[j], uv[j], vnorm, verts[indices[i * 3 + j]]);
			curTri.eArr[j] = edge[j];
			curTri.colArr[j] = col[j];
			curTri.uvArr[j] = uv[j];
		}
		curTri.normVec = cross(edge[0] - edge[1], edge[2] - edge[1]).normalized();
		if (curTri.normVec * vnorm < 0.0f) curTri.normVec = -curTri.normVec;

		outTriList.push_back(curTri);
	}
}

HIGHOMEGA::RENDER::GraphicsModel::GraphicsModel()
{
}

void HIGHOMEGA::RENDER::GraphicsModel::Model(std::string& newGroupId, MeshMaterial& inMaterial, std::vector<TriUV>& triList, bool inpImmutable, bool breakable)
{
	if (isInit) RemovePast();

	std::vector <unsigned char> idxVertData;
	GenerateGeom(triList, idxVertData);

	GeometryClass* curGeom = &MaterialGeomMap[inMaterial].emplace_back();
	inMaterial.BumpClaims();

	curGeom->Geometry(Instance, idxVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), inMaterial.isAlphaKeyed, newGroupId, inpImmutable, nullptr, nullptr, breakable);
	curGeom->getRTGeom().SetMask(inMaterial.rayMask);

	isInit = true;
}

void HIGHOMEGA::RENDER::GraphicsModel::Model(std::string& newGroupId, MeshMaterial& inMaterial, std::vector <unsigned char>& idxVertData, bool inpImmutable, bool breakable)
{
	if (isInit) RemovePast();

	GeometryClass* curGeom = &MaterialGeomMap[inMaterial].emplace_back();
	inMaterial.BumpClaims();

	curGeom->Geometry(Instance, idxVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), inMaterial.isAlphaKeyed, newGroupId, inpImmutable, nullptr, nullptr, breakable);
	curGeom->getRTGeom().SetMask(inMaterial.rayMask);

	isInit = true;
}

void HIGHOMEGA::RENDER::GraphicsModel::Model(HIGHOMEGA::MESH::Mesh & inpMesh, std::string belong, InstanceClass &ptrToInstance, std::function<bool(int, DataGroup &)> inpFilterFunction, bool inpImmutable, bool loadAnimationData, vec3* viewPos, float* fullBodyHeight, bool justKeyFrames)
{
	if (isInit) RemovePast();

	std::vector <unsigned char> indexVertData;
	std::vector <VertexAnimationInfo> vertAnimData;
	std::string parentArmature;

	vec3 edge, vnorm, col;
	vec2 uv;
	mat4 instTransform, instTransformDT;
	for (unsigned int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		if (!inpFilterFunction(i, inpMesh.DataGroups[i])) continue;

		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		HIGHOMEGA::MESH::DataBlock *keyFrameBlock = nullptr;
		Mesh::getDataBlock(curPolyGroup, "KEYFRAME_DATA", &keyFrameBlock);
		if (loadAnimationData && keyFrameBlock) // Animation data...
		{
			Animation animationData;
			unsigned int nBones, nKeyFrames;

			unsigned int dataOffset = 0u;

			nBones = *((unsigned int *)(&keyFrameBlock->blob.data()[dataOffset]));
			if (nBones > 1024) FATAL_ERROR("More than 1024 bones for skinned mesh? Likely data corruption.");
			dataOffset += sizeof(unsigned int);

			unsigned char boneName[1024];
			std::vector<std::string> boneNames;
			for (unsigned int i = 0; i != nBones; i++)
			{
				unsigned int boneNameLen = *((unsigned int*)(&keyFrameBlock->blob.data()[dataOffset]));
				if (boneNameLen >= 1024) FATAL_ERROR("Bone name length more than 1023? Likely data corruption.");
				dataOffset += sizeof(unsigned int);

				memcpy(boneName, &keyFrameBlock->blob.data()[dataOffset], boneNameLen);
				boneName[boneNameLen] = NULL;
				dataOffset += boneNameLen;

				boneNames.emplace_back((char *)boneName);
			}

			nKeyFrames = *((unsigned int*)(&keyFrameBlock->blob.data()[dataOffset]));
			if (nKeyFrames > 10000) FATAL_ERROR("More than 10000 keyframes for skinned mesh animation? Likely data corruption.");
			dataOffset += sizeof(unsigned int);

			float matrixFetch[16];
			for (unsigned int i = 0; i != nKeyFrames; i++)
			{
				unsigned int curKeyFrame = *((unsigned int*)(&keyFrameBlock->blob.data()[dataOffset]));
				dataOffset += sizeof(unsigned int);

				Pose newPose;
				newPose.keyFrameTime = curKeyFrame;
				for (unsigned int j = 0; j != nBones; j++)
				{
					Bone newBone;
					newBone.name = boneNames[j];
					memcpy(matrixFetch, &keyFrameBlock->blob.data()[dataOffset], sizeof(float) * 16);
					PackMat4(matrixFetch, newBone.bone);
					dataOffset += sizeof(float) * 16;
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
			if (!justKeyFrames) armatures[curPolyGroup.name].currentAndPrevPoseBuf = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO | USAGE_SRC | USAGE_DST, Instance, nullptr, (unsigned int)sizeof(float) * 16 * (unsigned int)armatures[curPolyGroup.name].currentPose.pose.size() * 2); // * 2, cause it previous frame's pose will follow right after
			continue;
		}
		if (justKeyFrames) continue;

		HIGHOMEGA::MESH::DataBlock *triBlock = nullptr;
		if (!Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock)) continue;

		float tmpFloat;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "invisible", tmpFloat)) continue;

		HIGHOMEGA::MESH::DataBlock *colsBlock = nullptr;
		HIGHOMEGA::MESH::DataBlock *vertexWeightsBlock = nullptr, *vertexWeightRefsBlock = nullptr, *parentArmatureRef = nullptr;
		bool hasCols;
		bool hasAnim;

		if (Mesh::getDataBlock(curPolyGroup, "COLS", &colsBlock))
			hasCols = true;
		else
			hasCols = false;

		if (loadAnimationData &&
			Mesh::getDataBlock(curPolyGroup, "WEIGHTS", &vertexWeightsBlock))
			hasAnim = true;
		else
			hasAnim = false;

		indexVertData.resize(triBlock->blob.size());

		if (isIndexVertexDataTooSmall(indexVertData)) continue;
		memcpy(indexVertData.data(), triBlock->blob.data(), triBlock->blob.size());
		if (hasAnim)
		{
			vertAnimData.resize(vertexWeightsBlock->blob.size() / sizeof(VertexAnimationInfo));
			memcpy(vertAnimData.data(), vertexWeightsBlock->blob.data(), vertexWeightsBlock->blob.size());
			Mesh::getDataBlock(curPolyGroup, "PARENT_ARMATURE", &parentArmatureRef);
		}

		HIGHOMEGA::MESH::DataBlock* instBlock = nullptr;
		bool hasInstances = Mesh::getDataBlock(curPolyGroup, "INSTANCES", &instBlock);
		if (hasInstances)
		{
			unsigned int numInstances = (unsigned int)(instBlock->blob.size() / (sizeof(float) * 16));

			RasterVertex* oldVerts; unsigned int* oldIndices, oldTriCount, oldVertCount, oldVertexDataOffset;
			getIndicesVertices(indexVertData, &oldVerts, oldVertCount, &oldIndices, oldTriCount, oldVertexDataOffset);

			std::vector<unsigned char> newIndexVertData = indexVertData;
			multiplyGeometry(newIndexVertData, numInstances);
			RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
			getIndicesVertices(newIndexVertData, &verts, vertCount, &indices, triCount, vertexDataOffset);

			for (unsigned int j = 0u; j != numInstances; j++) {
				PackMat4(&instBlock->blob.data()[j * sizeof(float) * 16], instTransform);
				instTransformDT = instTransform.DirectionTransform();
				for (unsigned int k = 0; k != oldVertCount; k++) {
					unpackRasterVertex(edge, col, uv, vnorm, oldVerts[k]);
					edge = instTransform * edge;
					vnorm = (instTransformDT * vnorm).normalized();
					packRasterVertex(edge, col, uv, vnorm, verts[j * oldVertCount + k]);
				}
			}
			indexVertData = newIndexVertData;
		}

		if (hasAnim)
		{
			if (!parentArmatureRef->rows[0][0].svalue(parentArmature)) FATAL_ERROR("Could not get parent armature for graphics model load");
		}

		HIGHOMEGA::MESH::DataBlock *propsBlock = nullptr;
		if (!Mesh::getDataBlock(curPolyGroup, "PROPS", &propsBlock)) continue;

		MeshMaterial mat(*propsBlock, belong, ptrToInstance);

		float staticTessPower;
		if (!hasAnim && Mesh::getDataRowFloat(*propsBlock, "staticTessellationPower", staticTessPower))
		{
			tessellateGeom[curPolyGroup.name].maxTessellationPower = staticTessPower;
			tessellateGeom[curPolyGroup.name].curTessellationPower = 0.0f;
			tessellateGeom[curPolyGroup.name].targetTessellationPower = staticTessPower;
			Mesh::getDataRowFloat(*propsBlock, "staticTessellationDisplacement", tessellateGeom[curPolyGroup.name].tessellationDisplacement);
			tessellateGeom[curPolyGroup.name].idxVertData = indexVertData;
			tessellateGeom[curPolyGroup.name].curGeom = nullptr;
			tessellateGeom[curPolyGroup.name].addedGeom = nullptr;
			tessellateGeom[curPolyGroup.name].mat = mat;

			indexVertData.clear();
			continue;
		}

		float outTmp;
		bool destructible = Mesh::getDataRowFloat(*propsBlock, "cuttable", outTmp) || Mesh::getDataRowFloat(*propsBlock, "shatterable", outTmp);

		GeometryClass* curGeom = &MaterialGeomMap[mat].emplace_back ();

		if (hasAnim)
		{
			curGeom->Geometry(ptrToInstance, indexVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), mat.isAlphaKeyed, curPolyGroup.name, inpImmutable, &vertAnimData, &parentArmature, destructible);
		}
		else
		{
			curGeom->Geometry(ptrToInstance, indexVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), mat.isAlphaKeyed, curPolyGroup.name, inpImmutable, nullptr, nullptr, destructible);
		}
		curGeom->getRTGeom().SetMask(mat.rayMask);

		indexVertData.clear();
		vertAnimData.clear();
	}

	doStaticTessellation(ptrToInstance, inpImmutable, viewPos, fullBodyHeight);

	isInit = true;
}

HIGHOMEGA::RENDER::GraphicsModel::~GraphicsModel()
{
	RemovePast();
}

GraphicsModelInstance* HIGHOMEGA::RENDER::GraphicsModel::CreateInstance(mat4* transform, bool supportsPrevTransform)
{
	GraphicsModelInstance *newInst = instances.emplace_back(new GraphicsModelInstance);
	newInst->Create(*this, transform, supportsPrevTransform);
	return newInst;
}

void HIGHOMEGA::RENDER::GraphicsModel::DestroyInstance(GraphicsModelInstance* inModelInst)
{
	std::vector<GraphicsModelInstance*>::iterator ptrToRemove = std::find(instances.begin(), instances.end(), inModelInst);
	if (ptrToRemove == instances.end()) FATAL_ERROR("Trying to delete a graphics model instance that doesn't exist.");
	delete (*ptrToRemove);
	instances.erase(ptrToRemove);
}

GeometryClass * HIGHOMEGA::RENDER::GraphicsModel::getGeometryById(const std::string & groupId)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if (it2->getGroupId() == groupId)
				return &(*it2);
	return nullptr;
}

void HIGHOMEGA::RENDER::GraphicsModel::getGeometryByIdPrefix(const std::string& prefix, std::vector<GeometryClass*>& allGeoms)
{
	allGeoms.clear();
	bool addEverything = (prefix == "");
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if (addEverything || (!addEverything && it2->getGroupId().rfind(prefix, 0) == 0))
				allGeoms.push_back(&(*it2));
}

MeshMaterial HIGHOMEGA::RENDER::GraphicsModel::getMaterialById(const std::string & groupId)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if (it2->getGroupId() == groupId) {
				return it->first;
			}
	return MeshMaterial();
}

void HIGHOMEGA::RENDER::GraphicsModel::doStaticTessellation(InstanceClass& ptrToInstance, bool inpImmutable, vec3* viewPos, float* fullBodyHeight, bool firstRun)
{
	for (std::pair<const std::string, TessellateVerts>& curPair : tessellateGeom)
	{
		std::string curGroupId = curPair.first;
		TessellateVerts& curTessVerts = curPair.second;
		MeshMaterial& curMat = curTessVerts.mat;

		RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
		getIndicesVertices(curTessVerts.idxVertData, &verts, vertCount, &indices, triCount, vertexDataOffset);

		if (!curTessVerts.curGeom) GeometryClass::getMinMax(verts, vertCount, curTessVerts.origMin, curTessVerts.origMax);
		if (viewPos)
		{
			curTessVerts.cent = (curTessVerts.origMin + curTessVerts.origMax) * 0.5f;
			curTessVerts.rad = (curTessVerts.origMax - curTessVerts.origMin).length() * 0.5f;
			curTessVerts.targetTessellationPower = min (curTessVerts.maxTessellationPower, floor(curTessVerts.maxTessellationPower * (max (fullBodyHeight ? *fullBodyHeight * 2.0f : 0.0f, curTessVerts.rad) / (*viewPos - curTessVerts.cent).length())));
		}
		else
			curTessVerts.targetTessellationPower = curTessVerts.maxTessellationPower;

		if (curTessVerts.curTessellationPower == curTessVerts.targetTessellationPower && curTessVerts.curGeom) continue;
		curTessVerts.curTessellationPower = curTessVerts.targetTessellationPower;

		if (curTessVerts.curTessellationPower == 0.0f)
		{
			if (curTessVerts.curGeom)
			{
				curTessVerts.addedGeom = new GeometryClass(ptrToInstance, curTessVerts.idxVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, inpImmutable);
				curTessVerts.addedGeom->SetMinMax(curTessVerts.origMin - vec3(curTessVerts.tessellationDisplacement), curTessVerts.origMax + vec3(curTessVerts.tessellationDisplacement));
				curTessVerts.addedGeom->setRTBufferDirty();
				curTessVerts.addedGeom->notifySubmissions = curTessVerts.curGeom->notifySubmissions;
				if (!firstRun) break; // Past the first run, reduce pressure on the GPU. Do one instance at a time.
			}
			else
			{
				GeometryClass* addedGeom = &MaterialGeomMap[curMat].emplace_back();
				addedGeom->Geometry(ptrToInstance, curTessVerts.idxVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, inpImmutable);
				addedGeom->SetMinMax(curTessVerts.origMin - vec3(curTessVerts.tessellationDisplacement), curTessVerts.origMax + vec3(curTessVerts.tessellationDisplacement));
				addedGeom->setRTBufferDirty();
				curTessVerts.curGeom = addedGeom;
			}

			continue;
		}

		GeometryClass reconstructedBase(ptrToInstance, curTessVerts.idxVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, inpImmutable);

		struct
		{
			unsigned int sourceTriCount;
			unsigned int trisPerPatch;
			unsigned int sideVertexCount;
			unsigned int vertsPerPatch;
			unsigned int srcIdxVertOffset;
			unsigned int dstIdxVertOffset;
			float displacementAmount;
		} PrimitiveTessellateParams;
		PrimitiveTessellateParams.sourceTriCount = triCount;
		PrimitiveTessellateParams.trisPerPatch = (unsigned int)powf(4.0f, curTessVerts.curTessellationPower);
		PrimitiveTessellateParams.sideVertexCount = (unsigned int)powf(2.0f, curTessVerts.curTessellationPower) + 1;
		// The following is dervied by simplifying: (2^n + 1 - 0) + (2^n + 1 - 1) + ... + (2^n + 1 - 2^n)
		// Using triangular numbers we get: (2^n + 1)*(2^n + 1) - (2^n * (2^n + 1)) / 2
		// Ultimately can be simplified to: https://math.stackexchange.com/questions/2529679/count-of-vertices-of-a-subdivided-triangle
		PrimitiveTessellateParams.vertsPerPatch = (unsigned int)((powf(2.0f, curTessVerts.curTessellationPower) + 1.0f) * (powf(2.0f, curTessVerts.curTessellationPower - 1.0f) + 1.0f));
		PrimitiveTessellateParams.srcIdxVertOffset = reconstructedBase.getDataOffsetInGiantVertexBuffer();
		PrimitiveTessellateParams.displacementAmount = curTessVerts.tessellationDisplacement;

		BufferClass PrimitiveTessellateParamsBuf;
		ShaderResourceSet tessellateShader;
		ComputeSubmission tessellateCompute;

		unsigned int tessIndexCount = triCount * PrimitiveTessellateParams.trisPerPatch * 3;
		unsigned int tessVertCount = triCount * PrimitiveTessellateParams.vertsPerPatch;
		std::vector<unsigned char> sizedIdxVerts;
		createEmptyIndicesVertices(tessIndexCount, tessVertCount, sizedIdxVerts);
		GeometryClass* newGeom = new GeometryClass(ptrToInstance, sizedIdxVerts, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), curMat.isAlphaKeyed, curGroupId, inpImmutable);
		PrimitiveTessellateParams.dstIdxVertOffset = newGeom->getDataOffsetInGiantVertexBuffer();

		PrimitiveTessellateParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &PrimitiveTessellateParams, (unsigned int)sizeof(PrimitiveTessellateParams));

		tessellateShader.Create("shaders/primitiveTessellate.comp.spv", "main");
		giantVertBufferSharedMutex.lock_shared();
		tessellateShader.AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *giantVertBuffer);
		giantVertBufferSharedMutex.unlock_shared();
		tessellateShader.AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 1, curMat.hgtRef->elem);
		tessellateShader.AddResource(RESOURCE_UBO, COMPUTE, 0, 2, PrimitiveTessellateParamsBuf);
		tessellateCompute.MakeDispatch(Instance, std::string("default"), tessellateShader, (unsigned int)ceil((double)triCount / (double)TessellateWorkGroup()), 1, 1);
		tessellateCompute.Submit();

		newGeom->SetMinMax(curTessVerts.origMin - vec3(PrimitiveTessellateParams.displacementAmount), curTessVerts.origMax + vec3(PrimitiveTessellateParams.displacementAmount));
		newGeom->setRTBufferDirty();
		if (curTessVerts.curGeom)
		{
			curTessVerts.addedGeom = newGeom;
			curTessVerts.addedGeom->notifySubmissions = curTessVerts.curGeom->notifySubmissions;
			if (!firstRun) break; // Past the first run, reduce pressure on the GPU. Do one instance at a time.
		}
		else
		{
			MaterialGeomMap[curMat].emplace_back();
			curTessVerts.curGeom = &(MaterialGeomMap[curMat].back());
			MaterialGeomMap[curMat].back() = std::move(*newGeom);
			operator delete(newGeom);
		}
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
					curTessVerts.curGeom->RemovePast();
					idxVertCache.erase(curTessVerts.curGeom);
					*curTessVerts.curGeom = std::move(*curTessVerts.addedGeom);
					operator delete(curTessVerts.addedGeom);
					curTessVerts.addedGeom = nullptr;
					break;
				}
		}
	}
}

void HIGHOMEGA::RENDER::GraphicsModel::removeGroupById(const std::string & groupId)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
	{
		std::list<GeometryClass>::iterator findRes = std::find_if(it->second.begin(), it->second.end(), [&groupId](GeometryClass & curGeom) { return curGeom.getGroupId() == groupId; });
		if (findRes != it->second.end())
		{
			idxVertCache.erase(&(*findRes));
			for (GraphicsModelInstance* curInst : instances)
				curInst->Remove(&(*findRes)); // Remove the piece from every submission, model instance and ultimately the model itself
			it->second.erase(findRes);
			if (it->second.size() == 0)
			{
				MeshMaterial firstCopy = it->first;
				firstCopy.ReduceClaims();
				MaterialGeomMap.erase(it->first);
			}
			std::vector<GraphicsModel*>updateThis;
			updateThis.push_back(this);
			UpdateSDFs(updateThis, true);
			break;
		}
	}
}

void HIGHOMEGA::RENDER::GraphicsModel::addGeomWithGroupId(const std::string& newGroupId, MeshMaterial& inMaterial, std::vector<unsigned char>& idxVertData, bool inpImmutable)
{
	if (!isInit) FATAL_ERROR("Trying to add mat/geom to an uninitialized graphics model");

	GeometryClass* curGeom = &MaterialGeomMap[inMaterial].emplace_back();
	inMaterial.BumpClaims();

	curGeom->Geometry(Instance, idxVertData, GeometryClass::DataLayout(0, FORMAT::R32G32B32A32F, FORMAT::R16G16F, FORMAT::R32UI), inMaterial.isAlphaKeyed, newGroupId, inpImmutable);
	curGeom->getRTGeom().SetMask(inMaterial.rayMask);
	std::unordered_map <MeshMaterial, std::list<GeometryClass>, MeshMaterialHash>::iterator it = MaterialGeomMap.find(inMaterial);

	for (GraphicsModelInstance* curInst : instances)
		curInst->Add(it->first, *curGeom); // Add it to every model instance and its submissions

	std::vector<GraphicsModel*>updateThis;
	updateThis.push_back(this);
	UpdateSDFs(updateThis, true);
}

void HIGHOMEGA::RENDER::GraphicsModel::transformVertsSlow(const mat4& trans, const std::string& groupIdPrefixOrId, mat4 *localTrans, vec3 *iSectP1, vec3 *iSectP2, bool *lineHit, vec3 *mulVCol, bool nonPrefixMode)
{
	mat4 tmpIdent;
	tmpIdent.Ident();

	bool doOptionalHitDetection = false;
	if (iSectP1 && iSectP2 && lineHit)
	{
		doOptionalHitDetection = true;
		*lineHit = false;
	}

	mat4 finalLocalTrans = trans;

	vec3 iSectDir, instCent;
	vec3 e1, e2, e3;
	vec3 edge, vnorm, vcol;
	vec2 uv;

	std::vector<GeometryClass*> fetchedGeom;
	if (nonPrefixMode)
		fetchedGeom = { getGeometryById(groupIdPrefixOrId) };
	else
		getGeometryByIdPrefix(groupIdPrefixOrId, fetchedGeom);

	for (GeometryClass* curGeom : fetchedGeom)
	{
		if (idxVertCache.find(curGeom) == idxVertCache.end() || idxVertCache[curGeom].size() != curGeom->getTotalDataSize())
			curGeom->Download(idxVertCache[curGeom]);

		std::vector<unsigned char> originalVerts = idxVertCache[curGeom];
		RasterVertex* verts; unsigned int *indices, triCount, vertCount, vertexDataOffset;
		getIndicesVertices(originalVerts, &verts, vertCount, &indices, triCount, vertexDataOffset);

		if (localTrans)
		{
			instCent = vec3(0.0f);
			for (int i = 0; i != vertCount; i++)
			{
				unpackRasterVertex(edge, vcol, uv, vnorm, verts[i]);
				instCent += edge;
			}
			instCent /= (float)vertCount;

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

		for (int i = 0; i != vertCount; i++)
		{
			unpackRasterVertex(edge, vcol, uv, vnorm, verts[i]);
			edge = finalLocalTrans * edge;

			if (mulVCol)
			{
				vcol.x *= mulVCol->x;
				vcol.y *= mulVCol->y;
				vcol.z *= mulVCol->z;
			}

			packRasterVertex(edge, vcol, uv, vnorm, verts[i]);
		}

		if (doOptionalHitDetection)
			for (int i = 0; i != triCount; i++)
			{
				unpackRasterVertex(e1, vcol, uv, vnorm, verts[indices[i * 3]]);
				unpackRasterVertex(e2, vcol, uv, vnorm, verts[indices[i * 3 + 1]]);
				unpackRasterVertex(e3, vcol, uv, vnorm, verts[indices[i * 3 + 2]]);
				float tmpK = 1.0f;
				iSectDir = (*iSectP2 - *iSectP1);
				if (LineSegTri(*iSectP1, iSectDir, e1, e2, e3, tmpK))
				{
					*lineHit = true;
					break;
				}
			}
		curGeom->Update(originalVerts);
	}
}

void HIGHOMEGA::RENDER::GraphicsModel::BlankAndResize(unsigned int scaleFactor)
{
	std::vector<unsigned char> curIndexVertBuf;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			it2->Download(curIndexVertBuf);
			multiplyGeometry(curIndexVertBuf, scaleFactor);
			it2->ChangeGeom(curIndexVertBuf);
		}
}

void HIGHOMEGA::RENDER::GraphicsModel::SetMinMax(const vec3& minVal, const vec3& maxVal)
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			it2->SetMinMax(minVal, maxVal);
}

void HIGHOMEGA::RENDER::GraphicsModel::SetDirty()
{
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			it2->setRTBufferDirty();
}

void HIGHOMEGA::RENDER::GraphicsModel::DownloadGeom(const std::string& groupId, std::vector<unsigned char>& outIdxVerts)
{
	GeometryClass* geomPtr = getGeometryById(groupId);
	if (!geomPtr) return;

	geomPtr->Download(outIdxVerts);
}

void HIGHOMEGA::RENDER::GraphicsModel::ChangeGeom(const std::string& groupId, std::vector<TriUV> & triList)
{
	GeometryClass *geomPtr = getGeometryById(groupId);
	if (!geomPtr) return;

	std::vector <unsigned char> idxVertData;
	GenerateGeom(triList, idxVertData);

	if (idxVertCache.find(geomPtr) != idxVertCache.end()) idxVertCache.erase(geomPtr);
	geomPtr->ChangeGeom(idxVertData);
}

void HIGHOMEGA::RENDER::GraphicsModel::ChangeGeom(const std::string& groupId, std::vector<unsigned char> & inIdxVerts)
{
	GeometryClass* geomPtr = getGeometryById(groupId);
	if (!geomPtr) return;

	if (idxVertCache.find(geomPtr) != idxVertCache.end()) idxVertCache.erase(geomPtr);
	geomPtr->ChangeGeom(inIdxVerts);
}

void HIGHOMEGA::RENDER::GraphicsModel::UpdateGeom(const std::string& groupId, std::vector<TriUV>& triList)
{
	GeometryClass *geomPtr = getGeometryById(groupId);
	if (!geomPtr) return;

	std::vector <unsigned char> idxVertData;
	GenerateGeom(triList, idxVertData);

	if (idxVertCache.find(geomPtr) != idxVertCache.end()) idxVertCache.erase(geomPtr);
	geomPtr->Update(idxVertData);
}

void HIGHOMEGA::RENDER::GraphicsModel::getModelMinMax(vec3& outMin, vec3& outMax)
{
	if (cachedModelMinMax)
	{
		outMin = cachedModelMin;
		outMax = cachedModelMax;
		return;
	}
	outMin = vec3(FLT_MAX);
	outMax = vec3(-FLT_MAX);
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = MaterialGeomMap.begin(); it != MaterialGeomMap.end(); ++it)
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			vec3 curMin, curMax;
			curMin = it2->getGeomMin();
			curMax = it2->getGeomMax();
			outMin = vec3(min(outMin.x, curMin.x), min(outMin.y, curMin.y), min(outMin.z, curMin.z));
			outMax = vec3(max(outMax.x, curMax.x), max(outMax.y, curMax.y), max(outMax.z, curMax.z));
		}

	cachedModelMin = outMin;
	cachedModelMax = outMax;
	cachedModelMinMax = true;
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::ChangeSignal(GeometryClass* changedGeom)
{
	allChangedGeom.push_back(changedGeom);

	rtScene.MarkSceneDirty();
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::Remove(GeometryClass* geomPiece)
{
	RemoveNotifySubmission(geomPiece);

	rtScene.Remove(*geomPiece);
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece)
{
	if (!inModelInst.submissionItems[this].filterFunction(inMaterial)) return;

	AddNotifySubmission(geomPiece);

	mat4 tmpMat;
	tmpMat.Ident();

	rtScene.Add(inModelInst.submissionItems[this].itemId, geomInstId, *geomPiece, tmpMat, Instance);
}

HIGHOMEGA::RENDER::GroupedTraceSubmission::~GroupedTraceSubmission()
{
	while (allSubmittedItems.size() > 0)
		Remove(*(allSubmittedItems.begin()->second));
	if (rtCopyTransformSubmission) delete rtCopyTransformSubmission;
	if (rtCopyTransformResources) delete rtCopyTransformResources;
	rtCopyTransformSubmission = nullptr;
	rtCopyTransformResources = nullptr;
}

unsigned long long HIGHOMEGA::RENDER::GroupedTraceSubmission::SceneID()
{
	for (GeometryClass* curChangedGeom: allChangedGeom)
		for (std::pair<const unsigned long long, GraphicsModelInstance*>& curSubmittedItem : allSubmittedItems)
			for (GraphicsModelInstance::MatGeomInstance& curMatGeomInst : curSubmittedItem.second->matGeomInstances)
				if (curMatGeomInst.geom == curChangedGeom)
				{
					unsigned int newIdxVertOffset = curChangedGeom->getDataOffsetInGiantVertexBuffer();
					SceneData->instancePropertiesBuffer->UploadSubData(curMatGeomInst.geomPropsId * sizeof(InstanceProperties) + HIGHOMEGA_INSTANCEPROPERTIES_TRIANGLEOFFSET_OFFSET, &newIdxVertOffset, sizeof(unsigned int));
				}
	allChangedGeom.clear();

	if (cachedArrangementId != SceneData->sceneArrangementId)
	{
		cachedArrangementId = SceneData->sceneArrangementId;
		rtScene.MarkSceneNeedingUpdate();
	}

	rtCopyTransformCallback = [&InstanceRef = Instance,
							   &transformBufferRef = GroupedRenderSubmission::SceneData->transformBuffer,
							   &instancePropsBufferRef = GroupedRenderSubmission::SceneData->instancePropertiesBuffer,
							   &nInstancesCachedRef = nInstancesCached,
							   &rtCopyTransformSubmissionRef = rtCopyTransformSubmission, 
							   &rtCopyTransformResourcesRef = rtCopyTransformResources] (unsigned int nInstances, BufferClass *rtInstanceBuffer, BufferClass* rtCopyTransformParamsBuffer, bool descNeedUpdating) -> void {

		bool redoSubmission = false;
		if (!rtCopyTransformSubmissionRef || !rtCopyTransformResourcesRef || descNeedUpdating || nInstancesCachedRef != nInstances)
		{
			nInstancesCachedRef = nInstances;
			redoSubmission = true;
		}

		if (redoSubmission)
		{
			unsigned int CompRTCopyTransformsWorkGroupSize = 32u;
			if (!rtCopyTransformSubmissionRef) rtCopyTransformSubmissionRef = new ComputeSubmission;
			if (rtCopyTransformResourcesRef) delete rtCopyTransformResourcesRef;
			rtCopyTransformResourcesRef = new ShaderResourceSet;
			rtCopyTransformResourcesRef->AddResource(RESOURCE_SSBO, COMPUTE, 0, 0, *rtInstanceBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
			rtCopyTransformResourcesRef->AddResource(RESOURCE_SSBO, COMPUTE, 0, 1, *transformBufferRef);
			rtCopyTransformResourcesRef->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, *instancePropsBufferRef);
			rtCopyTransformResourcesRef->AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, *rtCopyTransformParamsBuffer);
			rtCopyTransformResourcesRef->Create("shaders/rtCopyTransforms.comp.spv", "main");
			rtCopyTransformSubmissionRef->MakeDispatch(Instance, std::string("rtCopyTransforms"), *rtCopyTransformResourcesRef, (unsigned int)ceil((double)nInstances / (double)CompRTCopyTransformsWorkGroupSize), 1, 1);
		}
		(*rtCopyTransformSubmissionRef).MakeAsync().Submit();
	};

	return rtScene.rtSceneID(rtCopyTransformCallback) ^ GroupedRenderSubmission::SceneData->descriptorId;
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::Add(GraphicsModelInstance & inpModelInst, std::function<bool(const MeshMaterial & curMat)> inpFilterFunction)
{
	unsigned long long itemId = threadSafeMersenneTwister64Bit();
	inpModelInst.submissionItems[this] = { itemId, inpFilterFunction };
	mat4 tmpMat;
	tmpMat.Ident();

	for (GraphicsModelInstance::MatGeomInstance& curMatGeomInst : inpModelInst.matGeomInstances)
	{
		MeshMaterial itFirst = *curMatGeomInst.mat;
		if (!inpFilterFunction(itFirst)) continue;

		rtScene.Add(itemId, curMatGeomInst.geomPropsId, *curMatGeomInst.geom, tmpMat, Instance);
	}

	AddNotifySubmission(*inpModelInst.modelRef, inpFilterFunction);

	allSubmittedItems[itemId] = &inpModelInst;
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::Remove(GraphicsModelInstance& inpModelInst)
{
	if (inpModelInst.submissionItems.find(this) == inpModelInst.submissionItems.end()) return;

	unsigned long long itemId = inpModelInst.submissionItems[this].itemId;

	rtScene.RemoveAll(itemId);

	RemoveNotifySubmission(*inpModelInst.modelRef, inpModelInst.submissionItems[this].filterFunction);

	allSubmittedItems.erase(itemId);

	inpModelInst.submissionItems.erase(this);
}

void HIGHOMEGA::RENDER::GroupedTraceSubmission::DeleteRTResources()
{
	rtScene.DeleteRTResources();

	cachedArrangementId = 0u;
	nInstancesCached = 0u;
	if (rtCopyTransformSubmission) delete rtCopyTransformSubmission;
	if (rtCopyTransformResources) delete rtCopyTransformResources;
	rtCopyTransformSubmission = nullptr;
	rtCopyTransformResources = nullptr;
}

unsigned int HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::CompMortonWorkGroupSize()
{
	return 32;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::ChangeSignal(GeometryClass* changedGeom)
{
	allChangedGeom.push_back(changedGeom);
}

unsigned long long HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::SceneID()
{
	if (RTInstance::Enabled()) return sceneID;

	if (allChangedGeom.size() > 0)
	{
		std::unordered_set<GraphicsModel*> changedModelSet;
		for (GeometryClass *curGeom : allChangedGeom)
			for (std::pair<unsigned long long, GraphicsModelInstance*> curItem : allSubmittedItems)
			{
				if (std::find(changedModelSet.begin(), changedModelSet.end(), curItem.second->modelRef) != changedModelSet.end()) continue;
				for (std::pair <const MeshMaterial, std::list<GeometryClass>>& curMatGeomList : curItem.second->modelRef->MaterialGeomMap)
				{
					bool modelAdded = false;
					for (GeometryClass& curGeomFromModel : curMatGeomList.second)
						if (&curGeomFromModel == curGeom)
						{
							changedModelSet.insert(curItem.second->modelRef);
							modelAdded = true;
							break;
						}
					if (modelAdded) break;
				}
			}

		std::vector<GraphicsModel*> changedModels;
		changedModels.insert(changedModels.end(), changedModelSet.begin(), changedModelSet.end());
		GraphicsModel::UpdateSDFs(changedModels, true);

		allChangedGeom.clear();
	}

	sdfLeaves.clear();
	allSDFBVHImages.clear();
	leafTransformInfo.clear();

	sdfLeaves.reserve(allSubmittedItems.size());
	allSDFBVHImages.reserve(allSubmittedItems.size());
	leafTransformInfo.reserve(allSubmittedItems.size());

	unsigned int gridCounter = 0u;
	vec3 modMin, modMax;
	vec3 untransformedModMin, untransformedModMax;
	vec3 globalMin = vec3(FLT_MAX), globalMax = vec3(-FLT_MAX);
	for (std::pair<unsigned long long, GraphicsModelInstance*> curSubmittedItem : allSubmittedItems)
	{
		if (!curSubmittedItem.second->modelRef->sdf) continue;
		curSubmittedItem.second->modelRef->getModelMinMax(untransformedModMin, untransformedModMax);
		modMin = untransformedModMin;
		modMax = untransformedModMax;
		TransformCorners(modMin, modMax, curSubmittedItem.second->rootTransform);

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
		allSDFBVHImages.push_back(curSubmittedItem.second->modelRef->sdf);
		leafTransformInfo.emplace_back();
		UnpackMat4(curSubmittedItem.second->rootTransform, leafTransformInfo.back().transMat);
		leafTransformInfo.back().bMin[0] = untransformedModMin.x;
		leafTransformInfo.back().bMin[1] = untransformedModMin.y;
		leafTransformInfo.back().bMin[2] = untransformedModMin.z;
		leafTransformInfo.back().bMax[0] = untransformedModMax.x;
		leafTransformInfo.back().bMax[1] = untransformedModMax.y;
		leafTransformInfo.back().bMax[2] = untransformedModMax.z;
		globalMin.x = min(globalMin.x, modMin.x);
		globalMin.y = min(globalMin.y, modMin.y);
		globalMin.z = min(globalMin.z, modMin.z);
		globalMax.x = max(globalMax.x, modMax.x);
		globalMax.y = max(globalMax.y, modMax.y);
		globalMax.z = max(globalMax.z, modMax.z);
		gridCounter++;
	}
	unsigned int leafCount = (unsigned int)sdfLeaves.size();
	unsigned int nodeCount = 2 * leafCount - 1;
	leafCountAligned = (unsigned int)(ceil((double)leafCount / 1000.0) * 1000.0);
	nodeCountAligned = (unsigned int)(ceil((double)nodeCount / 2000.0) * 2000.0);

	if (sceneChanged)
	{
		if (buildParamsBuf.getSize() == 0) buildParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(buildParamsStruct));

		if (leafCountAligned > 0)
		{
			unsigned int leavesBufSize = leafCountAligned * sizeof(BoxLeaf);
			if (leavesBufSize > leavesBuf.getSize())
				leavesBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, leavesBufSize);
		}
		else
		{
			if (leavesBuf.getSize() == 0) leavesBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(BoxLeaf));
		}

		if (nodeCountAligned > 0)
		{
			unsigned int nodesBufSize = nodeCountAligned * sizeof(BVHNode);
			unsigned int cwNodesBufSize = nodeCountAligned * sizeof(CWBVHClass::CWBVHNode);
			if (cwNodesBufSize > cwNodesBuf.getSize())
				cwNodesBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, cwNodesBufSize);
		}
		else
		{
			if (cwNodesBuf.getSize() == 0)
				cwNodesBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(CWBVHClass::CWBVHNode));
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
				invMatBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, invMatBufSize);
			}
		}
		else
		{
			if (invMatBuf.getSize() == 0) invMatBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, nullptr, (unsigned int)sizeof(singleLeafTransform));
		}

		SDFs.clear();
		SDFs.reserve(allSDFBVHImages.size());
		for (unsigned int i = 0; i != allSDFBVHImages.size(); i++)
			SDFs.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 3, 0, *(allSDFBVHImages[i]));

		sceneID = threadSafeMersenneTwister64Bit();

		sceneChanged = false;
	}

	leavesBuf.UploadSubData(0, sdfLeaves.data(), leafCount * sizeof(BoxLeaf));

	buildParams.mapMinLeafCount[0] = globalMin.x;
	buildParams.mapMinLeafCount[1] = globalMin.y;
	buildParams.mapMinLeafCount[2] = globalMin.z;
	buildParams.mapMinLeafCount[3] = *((float *)(&leafCount));
	buildParams.mapMax[0] = globalMax.x;
	buildParams.mapMax[1] = globalMax.y;
	buildParams.mapMax[2] = globalMax.z;
	buildParamsBuf.UploadSubData(0, &buildParams, sizeof(buildParamsStruct));
	mortonSubmission.Submit();

	leavesBuf.DownloadSubData(0, sdfLeaves.data(), leafCount * sizeof(BoxLeaf));

	RadixSort(sdfLeaves, sdfLeavesTmp);

	leavesBuf.UploadSubData(0, sdfLeaves.data(), leafCount * sizeof(BoxLeaf));

	SDFCWBVH.Build(sdfLeaves);
	cwNodesBuf.UploadSubData(0, SDFCWBVH.compressedNodes.data(), (unsigned int)SDFCWBVH.compressedNodes.size() * sizeof(CWBVHClass::CWBVHNode));
	invMatBuf.UploadSubData(0, leafTransformInfo.data(), (unsigned int)leafTransformInfo.size() * sizeof(singleLeafTransform));

	return sceneID;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::Remove(GeometryClass* geomPiece)
{
	RemoveNotifySubmission(geomPiece);

	sceneChanged = true;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece)
{
	if (!inModelInst.submissionItems[this].filterFunction(inMaterial)) return;

	AddNotifySubmission(geomPiece);

	sceneChanged = true;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::Add(GraphicsModelInstance & inpModelInst, std::function<bool(const MeshMaterial&curMat)> inpFilterFunction)
{
	unsigned long long itemId = threadSafeMersenneTwister64Bit();

	inpModelInst.submissionItems[this] = { itemId, inpFilterFunction };

	allSubmittedItems[itemId] = &inpModelInst;

	AddNotifySubmission(*inpModelInst.modelRef, inpFilterFunction);

	sceneChanged = true;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::Remove(GraphicsModelInstance& inpModelInst)
{
	if (inpModelInst.submissionItems.find(this) == inpModelInst.submissionItems.end()) return;

	RemoveNotifySubmission(*inpModelInst.modelRef, inpModelInst.submissionItems[this].filterFunction);
	unsigned long long itemId = inpModelInst.submissionItems[this].itemId;

	allSubmittedItems.erase(itemId);

	inpModelInst.submissionItems.erase(this);

	sceneChanged = true;
}

void HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::DeleteSDFBVHResources()
{
	for (std::pair<unsigned long long, GraphicsModelInstance*> curSubmittedItem : allSubmittedItems)
	{
		if (curSubmittedItem.second->modelRef->sdf) delete curSubmittedItem.second->modelRef->sdf;
		curSubmittedItem.second->modelRef->sdf = nullptr;
	}
	sceneChanged = true;
}

HIGHOMEGA::RENDER::GroupedSDFBVHSubmission::~GroupedSDFBVHSubmission()
{
	while (allSubmittedItems.size() > 0)
		Remove(*(allSubmittedItems.begin()->second));
}

thread_local GroupedRenderSubmission::SceneDataStruct *GroupedRenderSubmission::SceneData = nullptr;
thread_local unsigned int GroupedRenderSubmission::SceneDataClaims = 0u;

void HIGHOMEGA::RENDER::GroupedRenderSubmission::CompileInstanceProperties(InstanceProperties & outProp, const MeshMaterial & inpMaterial, HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6], unsigned int transformOffset, unsigned int prevTransformOffset, GeometryClass& geom)
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

	// Is decal?
	if (inpMaterial.isDecal)
		attribs0 |= 0x00000200;

	// Is pass-through in particle scene?
	if (inpMaterial.holographicInParticleScene)
		attribs0 |= 0x00000400;

	// Has per-vertex velocity?
	if (inpMaterial.perVertexVelocity)
		attribs0 |= 0x00000800;

	// Is relative to the viewer (disables viewer induced moblur)?
	if (inpMaterial.isViewerRelative)
		attribs0 |= 0x00001000;

	// Is it modulating Albedo?
	if (inpMaterial.isAlphaBlending)
		attribs0 |= 0x00002000;

	vec3 geomMin = geom.getGeomMin();
	vec3 geomMax = geom.getGeomMax();
	outProp.geomMin[0] = geomMin.x;
	outProp.geomMin[1] = geomMin.y;
	outProp.geomMin[2] = geomMin.z;
	outProp.geomMax[0] = geomMax.x;
	outProp.geomMax[1] = geomMax.y;
	outProp.geomMax[2] = geomMax.z;

	outProp.attribs1[0] = *((float *)&attribs0);

	// emissivity
	outProp.attribs1[1] = inpMaterial.emissivity;

	// refractiveIndex
	outProp.attribs1[2] = inpMaterial.refractiveIndex;

	// Total u offset
	outProp.attribs2[0] = inpMaterial.uvOffset.x;

	// Total v offset
	outProp.attribs2[1] = inpMaterial.uvOffset.y;

	// Vertex displacement height factor
	outProp.attribs2[2] = inpMaterial.heightMapDisplaceFactor;

	// Player ID (we're not going to factor in most tessellation work into vis determination)
	outProp.attribs2[3] = *((float*)&inpMaterial.playerId);

	memcpy(outProp.textureOffsets, textureOffsets, sizeof(outProp.textureOffsets));

	outProp.idxVertOffset = geom.getDataOffsetInGiantVertexBuffer();
	outProp.transformOffset = transformOffset;
	outProp.prevTransformOffset = prevTransformOffset;
}

HIGHOMEGA::RENDER::GraphicsModelInstance::GraphicsModelInstance()
{
}

void HIGHOMEGA::RENDER::GraphicsModelInstance::Remove(GeometryClass* geom)
{
	for (std::pair<GroupedRenderSubmission*, SubmissionAttributes> curSub : submissionItems)
		curSub.first->Remove(geom);

	std::vector<MatGeomInstance>::iterator it = std::find_if(matGeomInstances.begin(), matGeomInstances.end(), [geom](const MatGeomInstance& curMatGeom) -> bool {
		return (curMatGeom.geom == geom);
	});

	GroupedRenderSubmission::SceneDataStruct& sceneData = *GroupedRenderSubmission::SceneData;
	std::array<ImageClass*, 5> removeSamplers;
	if (it != matGeomInstances.end())
	{
		MatGeomInstance &matGeomInst = *it;
		removeSamplers[0] = &matGeomInst.mat->diffRef->elem;
		removeSamplers[1] = matGeomInst.mat->nrmRef ? &matGeomInst.mat->nrmRef->elem : &matGeomInst.mat->diffRef->elem;
		removeSamplers[2] = matGeomInst.mat->rghRef ? &matGeomInst.mat->rghRef->elem : &matGeomInst.mat->diffRef->elem;
		removeSamplers[3] = matGeomInst.mat->hgtRef ? &matGeomInst.mat->hgtRef->elem : &matGeomInst.mat->diffRef->elem;
		removeSamplers[4] = matGeomInst.mat->spcRef ? &matGeomInst.mat->spcRef->elem : &matGeomInst.mat->diffRef->elem;
		for (int i = 0; i != 5; i++)
		{
			if (sceneData.uniqueSamplers.find(removeSamplers[i]) == sceneData.uniqueSamplers.end()) LOG() << "ERROR: Trying to remove a sampler that does not exist";
			GroupedRenderSubmission::SceneDataStruct::samplerOffsetClaims& curOffsetClaims = sceneData.uniqueSamplers[removeSamplers[i]];
			if (curOffsetClaims.claims > 0u)
			{
				curOffsetClaims.claims--;
				if (curOffsetClaims.claims == 0u)
				{
					sceneData.uniqueSamplersArray[curOffsetClaims.offset] = ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 0, sceneData.BlankTexture);
					sceneData.availableSamplers.push_back(curOffsetClaims.offset);
					sceneData.uniqueSamplers.erase(removeSamplers[i]);
				}
			}
			else
				LOG() << "ERROR: Trying to reduce sampler claims to negative.";
		}
		sceneData.availableInstanceProps.push_back(matGeomInst.geomPropsId);

		matGeomInstances.erase(it);
	}

	sceneData.redoDescriptors = true;
	sceneData.UpdateDescriptors();
}

HIGHOMEGA::RENDER::GraphicsModelInstance::~GraphicsModelInstance()
{
	if (sceneDataPtr != GroupedRenderSubmission::SceneData)
	{
		LOG() << "ERROR: Attempting to destroy graphics model instance on the wrong thread";
		return;
	}

	while (submissionItems.size() > 0)
		for (std::pair<GroupedRenderSubmission*, SubmissionAttributes> curSub : submissionItems)
		{
			curSub.first->Remove(*this); // This will remove the current pair from submissionItems automatically. Therefore the iterator will be invalid and we have to start over.
			break;
		}

	GroupedRenderSubmission::SceneDataStruct& sceneData = *GroupedRenderSubmission::SceneData;
	std::array<ImageClass*, 5> removeSamplers;
	for (MatGeomInstance& matGeomInst : matGeomInstances)
	{
		removeSamplers[0] = &matGeomInst.mat->diffRef->elem;
		removeSamplers[1] = matGeomInst.mat->nrmRef ? &matGeomInst.mat->nrmRef->elem : &matGeomInst.mat->diffRef->elem;
		removeSamplers[2] = matGeomInst.mat->rghRef ? &matGeomInst.mat->rghRef->elem : &matGeomInst.mat->diffRef->elem;
		removeSamplers[3] = matGeomInst.mat->hgtRef ? &matGeomInst.mat->hgtRef->elem : &matGeomInst.mat->diffRef->elem;
		removeSamplers[4] = matGeomInst.mat->spcRef ? &matGeomInst.mat->spcRef->elem : &matGeomInst.mat->diffRef->elem;
		for (int i = 0; i != 5; i++)
		{
			if (sceneData.uniqueSamplers.find(removeSamplers[i]) == sceneData.uniqueSamplers.end()) LOG() << "ERROR: Trying to remove a sampler that does not exist";
			GroupedRenderSubmission::SceneDataStruct::samplerOffsetClaims& curOffsetClaims = sceneData.uniqueSamplers[removeSamplers[i]];
			if (curOffsetClaims.claims > 0u)
			{
				curOffsetClaims.claims--;
				if (curOffsetClaims.claims == 0u)
				{
					sceneData.uniqueSamplersArray[curOffsetClaims.offset] = ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 0, sceneData.BlankTexture);
					sceneData.availableSamplers.push_back(curOffsetClaims.offset);
					sceneData.uniqueSamplers.erase(removeSamplers[i]);
				}
			}
			else
				LOG() << "ERROR: Trying to reduce sampler claims to negative.";
		}
		sceneData.availableInstanceProps.push_back(matGeomInst.geomPropsId);
	}
	if (prevRootTransformId != 0xFFFFFFFFu) sceneData.availableTransformIds.push_back(prevRootTransformId);
	sceneData.availableTransformIds.push_back(rootTransformId);
	matGeomInstances.clear();

	sceneData.redoDescriptors = true;
	sceneData.UpdateDescriptors();
}

void HIGHOMEGA::RENDER::GraphicsModelInstance::Add(const MeshMaterial& inMaterial, GeometryClass& geom)
{
	GroupedRenderSubmission::SceneDataStruct& sceneData = *GroupedRenderSubmission::SceneData;

	uploadInfo.uploadInstPropsIndex.reserve(uploadInfo.uploadInstPropsIndex.size() + 1);

	HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6];
	textureOffsets[0] = 0xFF;
	HIGHOMEGA_TEXTURE_OFFSET* assignSlotTo;
	for (int i = 0; i != 5; i++)
	{
		ImageClass* imageRef;
		switch (i)
		{
		case 0:
			imageRef = &(inMaterial.diffRef->elem);
			assignSlotTo = &textureOffsets[1];
			break;
		case 1:
			imageRef = inMaterial.nrmRef ? &(inMaterial.nrmRef->elem) : &(inMaterial.diffRef->elem);
			assignSlotTo = &textureOffsets[2];
			break;
		case 2:
			imageRef = inMaterial.rghRef ? &(inMaterial.rghRef->elem) : &(inMaterial.diffRef->elem);
			assignSlotTo = &textureOffsets[3];
			break;
		case 3:
			imageRef = inMaterial.hgtRef ? &(inMaterial.hgtRef->elem) : &(inMaterial.diffRef->elem);
			assignSlotTo = &textureOffsets[4];
			break;
		default:
			imageRef = inMaterial.spcRef ? &(inMaterial.spcRef->elem) : &(inMaterial.diffRef->elem);
			assignSlotTo = &textureOffsets[5];
			break;
		}
		if (sceneData.uniqueSamplers.find(imageRef) == sceneData.uniqueSamplers.end())
		{
			if (sceneData.availableSamplers.size() > 0)
			{
				HIGHOMEGA_TEXTURE_OFFSET emptySlot = sceneData.availableSamplers[sceneData.availableSamplers.size() - 1];
				sceneData.availableSamplers.pop_back();
				sceneData.uniqueSamplers[imageRef] = { emptySlot, 1 };
				sceneData.uniqueSamplersArray[emptySlot] = ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 0, *imageRef);
			}
			else
			{
				sceneData.uniqueSamplers[imageRef] = { (HIGHOMEGA_TEXTURE_OFFSET)sceneData.uniqueSamplersArray.size(), 1 };
				sceneData.uniqueSamplersArray.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 0, 0, *imageRef);
			}
			sceneData.redoDescriptors = true;
		}
		else
		{
			sceneData.uniqueSamplers[imageRef].claims++;
		}
		*assignSlotTo = sceneData.uniqueSamplers[imageRef].offset;
	}

	if (sceneData.availableInstanceProps.size() == 0)
	{
		unsigned int instancePropsPrevSize = sceneData.instancePropsCount;
		sceneData.instancePropsCount += 1000;
		for (unsigned int i = sceneData.instancePropsCount - 1; i != instancePropsPrevSize - 1; i--)
			sceneData.availableInstanceProps.push_back(i);

		sceneData.redoDescriptors = true;
	}
	unsigned int pickedInstancePropIndex = sceneData.availableInstanceProps[sceneData.availableInstanceProps.size() - 1];
	sceneData.availableInstanceProps.pop_back();

	uploadInfo.uploadInstPropsIndex.push_back(pickedInstancePropIndex);
	uploadInfo.uploadInstProps.emplace_back();

	GroupedRenderSubmission::CompileInstanceProperties(uploadInfo.uploadInstProps.back(), inMaterial, textureOffsets, rootTransformId, prevRootTransformId, geom);

	matGeomInstances.emplace_back(&inMaterial, &geom, pickedInstancePropIndex);

	sceneData.UpdateDescriptors();

	for (unsigned int i = 0; i != uploadInfo.uploadInstPropsIndex.size(); i++)
		sceneData.instancePropertiesBuffer->UploadSubData((unsigned int)(uploadInfo.uploadInstPropsIndex[i] * sizeof(InstanceProperties)), &uploadInfo.uploadInstProps[i], (unsigned int)sizeof(InstanceProperties));
	uploadInfo.uploadInstPropsIndex.clear();
	uploadInfo.uploadInstProps.clear();

	for (std::pair<GroupedRenderSubmission*, SubmissionAttributes> curSub : submissionItems)
		curSub.first->Add(*this, inMaterial, pickedInstancePropIndex, &geom);
}

void HIGHOMEGA::RENDER::GraphicsModelInstance::Create(GraphicsModel& inModel, mat4* transform, bool supportsPrevTransform)
{
	GroupedRenderSubmission::SceneDataStruct& sceneData = *GroupedRenderSubmission::SceneData;
	sceneDataPtr = GroupedRenderSubmission::SceneData;
	rootTransform.Ident();
	if (transform) rootTransform = *transform;
	prevRootTransform = rootTransform;

	if (sceneData.availableTransformIds.size() < (supportsPrevTransform ? 2 : 1))
	{
		unsigned int transformsPrevSize = sceneData.transformCount;
		sceneData.transformCount += 1000;
		for (unsigned int i = sceneData.transformCount - 1; i != transformsPrevSize - 1; i--)
			sceneData.availableTransformIds.push_back(i);

		sceneData.redoDescriptors = true;
	}

	unsigned int pickedTransformIndex = sceneData.availableTransformIds[sceneData.availableTransformIds.size() - 1];
	sceneData.availableTransformIds.pop_back();

	uploadInfo.uploadTransforms.resize(uploadInfo.uploadTransforms.size() + (sizeof(float) * 16));
	UnpackMat4(rootTransform, &uploadInfo.uploadTransforms[uploadInfo.uploadTransforms.size() - (sizeof(float) * 16)]);
	uploadInfo.uploadTransformIds.push_back(pickedTransformIndex);
	rootTransformId = pickedTransformIndex;

	unsigned int pickedPrevTransformIndex = prevRootTransformId = 0xFFFFFFFFu;
	if (supportsPrevTransform)
	{
		pickedPrevTransformIndex = sceneData.availableTransformIds[sceneData.availableTransformIds.size() - 1];
		sceneData.availableTransformIds.pop_back();

		uploadInfo.uploadTransforms.resize(uploadInfo.uploadTransforms.size() + (sizeof(float) * 16));
		UnpackMat4(prevRootTransform, &uploadInfo.uploadTransforms[uploadInfo.uploadTransforms.size() - (sizeof(float) * 16)]);
		uploadInfo.uploadTransformIds.push_back(pickedPrevTransformIndex);
		prevRootTransformId = pickedPrevTransformIndex;
	}

	HIGHOMEGA_TEXTURE_OFFSET textureOffsets[6];
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash>& curItemMatGeomMap = inModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		unsigned int numClaims = (unsigned int)it->second.size();

		uploadInfo.uploadInstPropsIndex.reserve(uploadInfo.uploadInstPropsIndex.size() + numClaims);

		textureOffsets[0] = 0xFF;
		HIGHOMEGA_TEXTURE_OFFSET* assignSlotTo;
		for (int i = 0; i != 5; i++)
		{
			ImageClass* imageRef;
			switch (i)
			{
			case 0:
				imageRef = &((*it).first.diffRef->elem);
				assignSlotTo = &textureOffsets[1];
				break;
			case 1:
				imageRef = (*it).first.nrmRef ? &((*it).first.nrmRef->elem) : &((*it).first.diffRef->elem);
				assignSlotTo = &textureOffsets[2];
				break;
			case 2:
				imageRef = (*it).first.rghRef ? &((*it).first.rghRef->elem) : &((*it).first.diffRef->elem);
				assignSlotTo = &textureOffsets[3];
				break;
			case 3:
				imageRef = (*it).first.hgtRef ? &((*it).first.hgtRef->elem) : &((*it).first.diffRef->elem);
				assignSlotTo = &textureOffsets[4];
				break;
			default:
				imageRef = (*it).first.spcRef ? &((*it).first.spcRef->elem) : &((*it).first.diffRef->elem);
				assignSlotTo = &textureOffsets[5];
				break;
			}
			if (sceneData.uniqueSamplers.find(imageRef) == sceneData.uniqueSamplers.end())
			{
				if (sceneData.availableSamplers.size() > 0)
				{
					HIGHOMEGA_TEXTURE_OFFSET emptySlot = sceneData.availableSamplers[sceneData.availableSamplers.size() - 1];
					sceneData.availableSamplers.pop_back();
					sceneData.uniqueSamplers[imageRef] = { emptySlot, numClaims };
					sceneData.uniqueSamplersArray[emptySlot] = ShaderResource(RESOURCE_SAMPLER, FRAGMENT, 0, 0, *imageRef);
				}
				else
				{
					sceneData.uniqueSamplers[imageRef] = { (HIGHOMEGA_TEXTURE_OFFSET)sceneData.uniqueSamplersArray.size(), numClaims };
					sceneData.uniqueSamplersArray.emplace_back(RESOURCE_SAMPLER, FRAGMENT, 0, 0, *imageRef);
				}
				sceneData.redoDescriptors = true;
			}
			else
			{
				sceneData.uniqueSamplers[imageRef].claims += numClaims;
			}
			*assignSlotTo = sceneData.uniqueSamplers[imageRef].offset;
		}
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
		{
			if (sceneData.availableInstanceProps.size() == 0)
			{
				unsigned int instancePropsPrevSize = sceneData.instancePropsCount;
				sceneData.instancePropsCount += 1000;
				for (unsigned int i = sceneData.instancePropsCount - 1; i != instancePropsPrevSize - 1; i--)
					sceneData.availableInstanceProps.push_back(i);

				sceneData.redoDescriptors = true;
			}
			unsigned int pickedInstancePropIndex = sceneData.availableInstanceProps[sceneData.availableInstanceProps.size() - 1];
			sceneData.availableInstanceProps.pop_back();

			uploadInfo.uploadInstPropsIndex.push_back(pickedInstancePropIndex);
			uploadInfo.uploadInstProps.emplace_back();

			GroupedRenderSubmission::CompileInstanceProperties(uploadInfo.uploadInstProps.back(), (*it).first, textureOffsets, pickedTransformIndex, pickedPrevTransformIndex, (*it2));

			matGeomInstances.emplace_back(&it->first, &(*it2), pickedInstancePropIndex);
		}
	}

	modelRef = &inModel;

	sceneData.UpdateDescriptors();

	for (unsigned int i = 0; i != uploadInfo.uploadInstPropsIndex.size(); i++)
		sceneData.instancePropertiesBuffer->UploadSubData((unsigned int)(uploadInfo.uploadInstPropsIndex[i] * sizeof(InstanceProperties)), &uploadInfo.uploadInstProps[i], (unsigned int)sizeof(InstanceProperties));
	for (unsigned int i = 0; i != uploadInfo.uploadTransformIds.size(); i++)
		sceneData.transformBuffer->UploadSubData((unsigned int)(uploadInfo.uploadTransformIds[i] * sizeof(float) * 16), &uploadInfo.uploadTransforms[i * sizeof(float) * 16], (unsigned int)(sizeof(float) * 16));
	uploadInfo.uploadInstPropsIndex.clear();
	uploadInfo.uploadInstProps.clear();
	uploadInfo.uploadTransformIds.clear();
	uploadInfo.uploadTransforms.clear();
}

void HIGHOMEGA::RENDER::GraphicsModelInstance::SetMinMax(const vec3& min, const vec3& max)
{
	float geomMinMax[6] = { min.x, min.y, min.z, max.x, max.y, max.z };
	for (unsigned int i = 0; i != matGeomInstances.size(); i++)
		GroupedRenderSubmission::SceneData->instancePropertiesBuffer->UploadSubData((unsigned int)(matGeomInstances[i].geomPropsId * sizeof(InstanceProperties) + HIGHOMEGA_INSTANCEPROPERTIES_MINMAX_OFFSET), geomMinMax, (unsigned int)(sizeof(float) * 6));
}

void HIGHOMEGA::RENDER::GraphicsModelInstance::Update(const mat4& transform)
{
	GroupedRenderSubmission::SceneDataStruct& sceneData = *GroupedRenderSubmission::SceneData;

	float tmpMatFloat[16 * 2];
	mat4 tmpMat;
	prevRootTransform = rootTransform;
	rootTransform = transform;
	if (prevRootTransformId != 0xFFFFFFFFu)
	{
		UnpackMat4(prevRootTransform, &tmpMatFloat[16]);
		sceneData.transformBuffer->UploadSubData((unsigned int)(prevRootTransformId * sizeof(float) * 16), &tmpMatFloat[16], (unsigned int)(sizeof(float) * 16));
	}
	UnpackMat4(rootTransform, tmpMatFloat);
	sceneData.transformBuffer->UploadSubData((unsigned int)(rootTransformId * sizeof(float) * 16), tmpMatFloat, (unsigned int)(sizeof(float) * 16));
	sceneData.sceneArrangementId = threadSafeMersenneTwister64Bit();
}

void HIGHOMEGA::RENDER::GroupedRenderSubmission::SceneDataStruct::UpdateDescriptors()
{
	if (!redoDescriptors) return;

	if (!instancePropertiesBuffer || (instancePropertiesBuffer && instancePropertiesBuffer->getSize() / sizeof(InstanceProperties) < instancePropsCount))
	{
		BufferClass* newInstancePropsBuffer = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO | USAGE_SRC | USAGE_DST, Instance, nullptr, (unsigned int)(instancePropsCount * sizeof(InstanceProperties)));
		if (instancePropertiesBuffer)
		{
			instancePropertiesBuffer->CopyToBuffer(Instance, *newInstancePropsBuffer, instancePropertiesBuffer->getSize());
			delete instancePropertiesBuffer;
		}
		instancePropertiesBuffer = newInstancePropsBuffer;
	}
	if (!transformBuffer || (transformBuffer && transformBuffer->getSize() / (sizeof(float) * 16) < transformCount))
	{
		BufferClass* newTransformBuffer = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO | USAGE_SRC | USAGE_DST, Instance, nullptr, (unsigned int)(transformCount * sizeof(float) * 16));
		if (transformBuffer)
		{
			transformBuffer->CopyToBuffer(Instance, *newTransformBuffer, transformBuffer->getSize());
			delete transformBuffer;
		}
		transformBuffer = newTransformBuffer;
	}
	descriptorId = threadSafeMersenneTwister64Bit();
	redoDescriptors = false;
}

HIGHOMEGA::RENDER::GroupedRenderSubmission::GroupedRenderSubmission()
{
	if (!SceneData)
	{
		SceneData = new SceneDataStruct;
		unsigned int whiteColor = 0xFFFFFFFFu;
		SceneData->BlankTexture.CreateTexture(Instance, 1u, 1u, (unsigned char*)&whiteColor, 1u, false, true);
	}
	SceneDataClaims++;
}

HIGHOMEGA::RENDER::GroupedRenderSubmission::~GroupedRenderSubmission()
{
	SceneDataClaims--;
	if (SceneDataClaims == 0u)
		if (SceneData)
		{
			delete SceneData->instancePropertiesBuffer;
			delete SceneData->transformBuffer;
			delete SceneData;
			SceneData = nullptr;
		}
}

void HIGHOMEGA::RENDER::GroupedRenderSubmission::AddNotifySubmission(GraphicsModel& inpModel, std::function<bool(const MeshMaterial& curMat)> inpFilterFunction)
{
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash>& curItemMatGeomMap = inpModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if ((*it2).notifySubmissions.find(this) == (*it2).notifySubmissions.end())
				(*it2).notifySubmissions[this] = 1;
			else
				(*it2).notifySubmissions[this]++;
	}
}

void HIGHOMEGA::RENDER::GroupedRenderSubmission::AddNotifySubmission(GeometryClass* geomPiece)
{
	// Filtering should have happened at a higher level...
	if (geomPiece->notifySubmissions.find(this) == geomPiece->notifySubmissions.end())
		geomPiece->notifySubmissions[this] = 1;
	else
		geomPiece->notifySubmissions[this]++;
}

void HIGHOMEGA::RENDER::GroupedRenderSubmission::RemoveNotifySubmission(GraphicsModel& inpModel, std::function<bool(const MeshMaterial& curMat)> inpFilterFunction)
{
	std::unordered_map<MeshMaterial, std::list<GeometryClass>, MeshMaterialHash>& curItemMatGeomMap = inpModel.MaterialGeomMap;
	for (std::unordered_map<MeshMaterial, std::list<GeometryClass>>::iterator it = curItemMatGeomMap.begin(); it != curItemMatGeomMap.end(); ++it)
	{
		MeshMaterial itFirst = it->first;
		if (!inpFilterFunction(itFirst)) continue;
		for (std::list<GeometryClass>::iterator it2 = it->second.begin(); it2 != it->second.end(); it2++)
			if ((*it2).notifySubmissions.find(this) != (*it2).notifySubmissions.end() && (*it2).notifySubmissions[this] > 0u)
			{
				(*it2).notifySubmissions[this]--;
				if ((*it2).notifySubmissions[this] == 0u) (*it2).notifySubmissions.erase(this);
			}
			else
				FATAL_ERROR("Notify submission was not found or did not have claims on GeometryClass during removal.");
	}
}

void HIGHOMEGA::RENDER::GroupedRenderSubmission::RemoveNotifySubmission(GeometryClass* geomPiece)
{
	if (geomPiece->notifySubmissions.find(this) != geomPiece->notifySubmissions.end() && geomPiece->notifySubmissions[this] > 0u)
	{
		geomPiece->notifySubmissions[this]--;
		if (geomPiece->notifySubmissions[this] == 0u) geomPiece->notifySubmissions.erase(this);
	}
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Remove(GeometryClass* geomPiece)
{
	RemoveNotifySubmission(geomPiece);

	if (recordedCmdBuf) redoSubmissionData = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Add(GraphicsModelInstance& inModelInst, const MeshMaterial& inMaterial, unsigned int geomInstId, GeometryClass* geomPiece)
{
	if (!inModelInst.submissionItems[this].filterFunction(inMaterial)) return;

	AddNotifySubmission(geomPiece);

	if (recordedCmdBuf) redoSubmissionData = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::DestroySubmissionData()
{
	entireMatGeomMap.clear();
	PSO_DSL_DS_GeomInstPairings.clear();
	Rasterlet.ResetRasterlet();
}

std::string HIGHOMEGA::RENDER::GroupedRasterSubmission::GenerateRasterPSOKey(MeshMaterial& inpMat, PipelineFlags& inpPFlags, bool renderMode)
{
	std::string selShaderName = inpMat.shaderName;
	if (shaders.find(inpMat.shaderName) == shaders.end()) {
		selShaderName = "default";
	}

	ShaderSpecilization* spec;

	std::string rasterPSOKey = "";
	rasterPSOKey += shaders[selShaderName]->vertex_shader;
	spec = shaders[selShaderName]->getStageSpecializationDataRef(VERTEX);
	if (spec) rasterPSOKey += "[" + spec->name + "]";
	rasterPSOKey += shaders[selShaderName]->tc_shader;
	spec = shaders[selShaderName]->getStageSpecializationDataRef(TESS_CTRL);
	if (spec) rasterPSOKey += "[" + spec->name + "]";
	rasterPSOKey += shaders[selShaderName]->te_shader;
	spec = shaders[selShaderName]->getStageSpecializationDataRef(TESS_EVAL);
	if (spec) rasterPSOKey += "[" + spec->name + "]";
	rasterPSOKey += shaders[selShaderName]->geom_shader;
	spec = shaders[selShaderName]->getStageSpecializationDataRef(GEOMETRY);
	if (spec) rasterPSOKey += "[" + spec->name + "]";
	rasterPSOKey += shaders[selShaderName]->fragment_shader;
	spec = shaders[selShaderName]->getStageSpecializationDataRef(FRAGMENT);
	if (spec) rasterPSOKey += "[" + spec->name + "]";
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

unsigned long long HIGHOMEGA::RENDER::GroupedRasterSubmission::GenerateRasterDSKey(std::vector<ShaderResource>& resources)
{
	size_t retVal = 0ull;
	for (ShaderResource& curRes : resources)
		retVal ^= curRes.GetHash();
	return retVal;
}

HIGHOMEGA::RENDER::GroupedRasterSubmission::GroupedRasterSubmission()
{
	frameBuffer = nullptr;
	shaders.clear();
	recordedCmdBuf = false;
	cullingResourcesChanged = false;
	cullingPrepassCompute = nullptr;
	cullingMainpassCompute = nullptr;
	cullingPrepassResourceSet = nullptr;
	cullingMainpassResourceSet = nullptr;
	redoSubmissionData = false;
	clearColor = vec3(0.0f);
	clearColorW = 1.0f;
	depthClear = 1.0f;
	stencilClear = 0;
	requestsBVH = false;
	doesCulling = false;
	Setup_HiZ = false;
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
	while (allSubmittedItems.size() > 0)
		Remove(*(allSubmittedItems.begin()->second));
	for (std::pair<const unsigned long long, DescriptorSets*>& curDS : DSCache)
		delete curDS.second;
	DSCache.clear();
	if (cullingPrepassCompute) delete cullingPrepassCompute;
	if (cullingMainpassCompute) delete cullingMainpassCompute;
	if (cullingPrepassResourceSet) delete cullingPrepassResourceSet;
	if (cullingMainpassResourceSet) delete cullingMainpassResourceSet;
	if (MipChainPass1_HiZ) delete MipChainPass1_HiZ;
	if (MipChainPass2_HiZ) delete MipChainPass2_HiZ;
	if (MipChainPass1ShaderResources_HiZ) delete MipChainPass1ShaderResources_HiZ;
	if (MipChainPass2ShaderResources_HiZ) delete MipChainPass2ShaderResources_HiZ;
	if (mainPassFrameBuffer) delete mainPassFrameBuffer;
	cullingPrepassCompute = nullptr;
	cullingMainpassCompute = nullptr;
	cullingPrepassResourceSet = nullptr;
	cullingMainpassResourceSet = nullptr;
	MipChainPass1_HiZ = MipChainPass2_HiZ = nullptr;
	MipChainPass1ShaderResources_HiZ = MipChainPass2ShaderResources_HiZ = nullptr;
	mainPassFrameBuffer = nullptr;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::ChangeSignal(GeometryClass* changedGeom)
{
	allChangedGeom.push_back(changedGeom);
	if (recordedCmdBuf) redoSubmissionData = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Add(GraphicsModelInstance& inpModelInst, std::function<bool(const MeshMaterial & curMat)> inpFilterFunction)
{
	unsigned long long itemId = threadSafeMersenneTwister64Bit();

	inpModelInst.submissionItems[this] = { itemId, inpFilterFunction };

	allSubmittedItems[itemId] = &inpModelInst;

	AddNotifySubmission(*inpModelInst.modelRef, inpFilterFunction);

	if (recordedCmdBuf) redoSubmissionData = true;
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::postProcessOnlyFilter(const MeshMaterial & curMat)
{
	return curMat.postProcess;
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::blendOnlyFilter(const MeshMaterial& curMat)
{
	return curMat.pipelineFlags.alphaBlending && curMat.pipelineFlags.changedBlendEnable && (!curMat.isDecal);
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::everythingButDecalsFilter(const MeshMaterial & curMat)
{
	return !curMat.isDecal;
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::decalOnlyFilter(const MeshMaterial& curMat)
{
	return curMat.isDecal;
}

bool HIGHOMEGA::RENDER::GroupedRasterSubmission::noBlendOrPostProcessOrDecalFilter(const MeshMaterial& curMat)
{
	if (curMat.postProcess || (curMat.pipelineFlags.alphaBlending && curMat.pipelineFlags.changedBlendEnable) || curMat.isDecal) return false;
	return true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Remove(GraphicsModelInstance& inpModelInst)
{
	if (inpModelInst.submissionItems.find(this) == inpModelInst.submissionItems.end()) return;

	RemoveNotifySubmission(*inpModelInst.modelRef, inpModelInst.submissionItems[this].filterFunction);
	unsigned long long itemId = inpModelInst.submissionItems[this].itemId;

	allSubmittedItems.erase(itemId);

	inpModelInst.submissionItems.erase(this);

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

void HIGHOMEGA::RENDER::GroupedRasterSubmission::requestSDFBVH(GroupedSDFBVHSubmission & sdfBvhSubmission)
{
	sourceSDFBVHSubmission = &sdfBvhSubmission;
	sourceSDFBVHId = 0u;
	requestsSDFBVH = true;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::AllocateMipChainImages()
{
	if (MinZChain.size() > 0) return; // Already done
	MinZChain.resize(7);
	MaxZChain.resize(7);
	for (int i = 0; i != 7; i++)
	{
		MinZChain[i].CreateImageStore(Instance, R32F, 64 >> i, 64 >> i, 1, _2D, true);
		MaxZChain[i].CreateImageStore(Instance, R32F, 64 >> i, 64 >> i, 1, _2D, true);
	}

}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::doCulling(FrustumClass &inpFrustum, CULL_MODE inCullMode, int useDepthLayer)
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
		AllocateMipChainImages();

		mipParams_HiZ.blockSize[0] = (unsigned int)ceil((double)frameBuffer->getWidth() / 64.0);
		mipParams_HiZ.blockSize[1] = (unsigned int)ceil((double)frameBuffer->getHeight() / 64.0);
		mipParamsBuffer_HiZ.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &mipParams_HiZ, (unsigned int)sizeof(mipParamsStruct));

		MipChainPass1_HiZ = new ComputeSubmission;
		MipChainPass2_HiZ = new ComputeSubmission;
		MipChainPass1ShaderResources_HiZ = new ShaderResourceSet;
		MipChainPass2ShaderResources_HiZ = new ShaderResourceSet;

		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 0, *(frameBuffer->GetDepthStencil()), useDepthLayer);
		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, MinZChain[0], -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 2, MaxZChain[0], -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		MipChainPass1ShaderResources_HiZ->AddResource(RESOURCE_UBO, COMPUTE, 0, 3, mipParamsBuffer_HiZ);
		MipChainPass1ShaderResources_HiZ->Create("shaders/depthmip1.comp.spv", "main");
		MipChainPass1_HiZ->MakeDispatch(Instance, std::string("DepthMip1"), *MipChainPass1ShaderResources_HiZ, 8, 8, 1);

		std::vector<ShaderResource> RestOfMinZChain, RestOfMaxZChain;
		MipChainPass2ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 0, MinZChain[0], -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_CONSUMER);
		for (int i = 1; i != 7; i++)
			RestOfMinZChain.emplace_back(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, MinZChain[i], -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		MipChainPass2ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 1, RestOfMinZChain, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		MipChainPass2ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 2, MaxZChain[0], -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_CONSUMER);
		for (int i = 1; i != 7; i++)
			RestOfMaxZChain.emplace_back(RESOURCE_IMAGE_STORE, COMPUTE, 0, 3, MaxZChain[i], -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		MipChainPass2ShaderResources_HiZ->AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 3, RestOfMaxZChain, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		MipChainPass2ShaderResources_HiZ->Create("shaders/depthmip2.comp.spv", "main");
		MipChainPass2_HiZ->MakeDispatch(Instance, std::string("DepthMip2"), *MipChainPass2ShaderResources_HiZ, 1, 1, 1);

		Setup_HiZ = true;
	}

	cullMode = inCullMode;
	cachedCullingFrustum = &inpFrustum;

	Rasterlet.doesCulling = doesCulling = true;
}

unsigned int HIGHOMEGA::RENDER::GroupedRasterSubmission::WorkGroupTwoPassCullX()
{
	return 32;
}

GroupedRasterSubmission& HIGHOMEGA::RENDER::GroupedRasterSubmission::MakeAsync()
{
	Rasterlet.doCPUSync = false;
	return *this;
}

void HIGHOMEGA::RENDER::GroupedRasterSubmission::Render()
{
	if (frameBuffer == nullptr || shaders.size() == 0) return;

	unsigned long long curSDFBVHId;
	if (requestsSDFBVH && sourceSDFBVHId != (curSDFBVHId = sourceSDFBVHSubmission->SceneID()))
	{
		sourceSDFBVHId = curSDFBVHId;
		if (recordedCmdBuf) redoSubmissionData = true;
	}
	if (frameBuffer && frameBuffer->getRenderMode() == OFF_SCREEN && frameBuffer->RecreateIfStale() && doesCulling && mainPassFrameBuffer)
	{
		delete mainPassFrameBuffer;
		mainPassFrameBuffer = nullptr;
		if (recordedCmdBuf) redoSubmissionData = true;
	}
	if (swapchainId && swapchainId != Instance.getSwapchainId())
	{
		swapchainId = Instance.getSwapchainId();
		if (Setup_HiZ)
		{
			mipParams_HiZ.blockSize[0] = (unsigned int)ceil((double)frameBuffer->getWidth() / 64.0);
			mipParams_HiZ.blockSize[1] = (unsigned int)ceil((double)frameBuffer->getHeight() / 64.0);
			mipParamsBuffer_HiZ.UploadSubData(0, &mipParams_HiZ, sizeof(mipParamsStruct));
		}
		if (recordedCmdBuf) redoSubmissionData = true;
	}

	if (sceneDataRecordID != GroupedRenderSubmission::SceneData->descriptorId)
	{
		sceneDataRecordID = GroupedRenderSubmission::SceneData->descriptorId;
		if (recordedCmdBuf) redoSubmissionData = true;
		if (doesCulling) cullingResourcesChanged = true;
	}
	if (!sceneDataRecordID) return;

	if (redoSubmissionData)
	{
		for (GeometryClass* curChangedGeom : allChangedGeom)
			for (std::pair<const unsigned long long, GraphicsModelInstance*>& curSubmittedItem : allSubmittedItems)
				for (GraphicsModelInstance::MatGeomInstance& curMatGeomInst : curSubmittedItem.second->matGeomInstances)
					if (curMatGeomInst.geom == curChangedGeom)
					{
						unsigned int newIdxVertOffset = curChangedGeom->getDataOffsetInGiantVertexBuffer();
						SceneData->instancePropertiesBuffer->UploadSubData(curMatGeomInst.geomPropsId * sizeof(InstanceProperties) + HIGHOMEGA_INSTANCEPROPERTIES_TRIANGLEOFFSET_OFFSET, &newIdxVertOffset, sizeof(unsigned int));
					}
		allChangedGeom.clear();

		DestroySubmissionData();
		recordedCmdBuf = false;
		redoSubmissionData = false;
	}

	if (!recordedCmdBuf)
	{
		swapchainId = 0ull;
		if (!swapchainId && (frameBuffer->getRenderMode() == ON_SCREEN || frameBuffer->IsSwapchainDependent())) swapchainId = Instance.getSwapchainId();
		if (doesCulling && !mainPassFrameBuffer)
		{
			mainPassFrameBuffer = new FramebufferClass;
			*mainPassFrameBuffer = *frameBuffer;
			mainPassFrameBuffer->Create(frameBuffer->getRenderMode(), Instance, Window, true);
		}

		transformedMaterials.clear();
		for (std::pair <const unsigned long long, GraphicsModelInstance *> & curRenderItemKV : allSubmittedItems)
			for (GraphicsModelInstance::MatGeomInstance& curMatGeomInst : curRenderItemKV.second->matGeomInstances)
			{
				MeshMaterial itFirst = *curMatGeomInst.mat;
				if (!curRenderItemKV.second->submissionItems[this].filterFunction(itFirst)) continue;

				if (matTransform)
				{
					transformedMaterials.push_back(matTransform(itFirst));
					entireMatGeomMap[transformedMaterials.back()].push_back(&curMatGeomInst);
				}
				else
					entireMatGeomMap[itFirst].push_back(&curMatGeomInst);
			}

		PipelineFlags pFlags;
		std::vector <ShaderResource> shaderResources;
		shaderResources.reserve(50);

		Rasterlet.waitSems.clear();
		Rasterlet.waitOldSems.clear();
		Rasterlet.signalSems.clear();
		for (std::pair <const MeshMaterial, std::vector<GraphicsModelInstance::MatGeomInstance *>> & curMatGeomPairing : entireMatGeomMap)
		{
			pFlags = defaultPipelineFlags;
			MeshMaterial curMat = curMatGeomPairing.first;

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
			shaderResources.emplace_back(RESOURCE_SAMPLER, FRAGMENT | TESS_EVAL, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			shaderResources.emplace_back(RESOURCE_SSBO, VERTEX | FRAGMENT | TESS_CTRL | TESS_EVAL, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.lock_shared();
			shaderResources.emplace_back(RESOURCE_SSBO, VERTEX | FRAGMENT | TESS_CTRL | TESS_EVAL, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			shaderResources.emplace_back(RESOURCE_SSBO, VERTEX | FRAGMENT | TESS_CTRL | TESS_EVAL, 2, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			if (requestsSDFBVH)
			{
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 3, 0, sourceSDFBVHSubmission->invMatBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 3, 1, sourceSDFBVHSubmission->leavesBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				shaderResources.emplace_back(RESOURCE_SSBO, FRAGMENT, 3, 2, sourceSDFBVHSubmission->cwNodesBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
				shaderResources.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 4, sourceSDFBVHSubmission->SDFs, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			}
			std::string selShaderName = curMat.shaderName;
			if (shaders.find(curMat.shaderName) == shaders.end()) {
				selShaderName = "default";
			}
			shaderResources.insert(shaderResources.end(), shaders[selShaderName]->getAdditionalResources().begin(), shaders[selShaderName]->getAdditionalResources().end());
			if (!swapchainId && shaders[selShaderName]->areAdditionalResourcesSwapchainDependent()) swapchainId = Instance.getSwapchainId();
			for (ShaderResource& curRes : shaderResources)
			{
				if (curRes.IsSimultaneouslyProduced()) Rasterlet.waitOldSems.merge(curRes.GetAsDependencies());
				else Rasterlet.waitSems.merge(curRes.GetAsDependencies());
				if (curRes.IsProduced()) Rasterlet.signalSems.merge(curRes.GetAsDependencies());
			}

			std::string rasterPSOKey = GenerateRasterPSOKey(curMat, pFlags, frameBuffer->getRenderMode());

			Raster_PSO_DSL *RasterPSODSL = nullptr;
			{
				std::lock_guard<std::mutex> lk(globalRaster_PSO_DSL_Cache_mutex);
				if (globalRaster_PSO_DSL_Cache.find(rasterPSOKey) == globalRaster_PSO_DSL_Cache.end())
				{
					globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL = new DescriptorSetLayout();
					globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL->CreateDescriptorSetLayout(shaderResources, &Instance);
					globalRaster_PSO_DSL_Cache[rasterPSOKey].PSO = new RasterPipelineStateClass(Instance, *globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL, pFlags, *frameBuffer, *curMatGeomPairing.second[0]->geom, *shaders[selShaderName]);
				}
				if (doesCulling && !globalRaster_PSO_DSL_Cache[rasterPSOKey].PSOMainPass)
					globalRaster_PSO_DSL_Cache[rasterPSOKey].PSOMainPass = new RasterPipelineStateClass(Instance, *globalRaster_PSO_DSL_Cache[rasterPSOKey].DSL, pFlags, *mainPassFrameBuffer, *curMatGeomPairing.second[0]->geom, *shaders[selShaderName]);
				RasterPSODSL = &globalRaster_PSO_DSL_Cache[rasterPSOKey];
			}
			
			unsigned long long rasterDSKey = GenerateRasterDSKey(shaderResources);
			if (DSCache.find(rasterDSKey) == DSCache.end())
			{
				DSCache[rasterDSKey] = new DescriptorSets(RasterPSODSL->DSL);
				DSCache[rasterDSKey]->WriteDescriptorSets(shaderResources);
			}
			else
			{
				DSCache[rasterDSKey]->RewriteDescriptorSets(shaderResources);
			}
			DescriptorSets* DSPtr = DSCache[rasterDSKey];
			for (GraphicsModelInstance::MatGeomInstance* curGeom : curMatGeomPairing.second)
			{
				PSO_DSL_DS_GeomInstPairing curPairing;
				curPairing.PSO_DSL = RasterPSODSL;
				curPairing.renderOrder = curMat.renderOrder;
				curPairing.DS = DSPtr;
				curPairing.geom = curGeom->geom;
				curPairing.inst = curGeom->geomPropsId;
				curPairing.renderOrder = curMat.renderOrder;
				PSO_DSL_DS_GeomInstPairings.push_back(curPairing);
			}
		}

		std::sort(PSO_DSL_DS_GeomInstPairings.begin(), PSO_DSL_DS_GeomInstPairings.end(), [&shadersRef = shaders](const PSO_DSL_DS_GeomInstPairing& lhs, const PSO_DSL_DS_GeomInstPairing& rhs)
			{
				return (lhs.renderOrder < rhs.renderOrder) ||
					   (lhs.renderOrder == rhs.renderOrder && lhs.PSO_DSL->PSO < rhs.PSO_DSL->PSO) ||
					   (lhs.renderOrder == rhs.renderOrder && lhs.PSO_DSL->PSO == rhs.PSO_DSL->PSO && lhs.DS < rhs.DS);
			});

		if (doesCulling && PSO_DSL_DS_GeomInstPairings.size() > 0)
		{

			if (!cullingPrepassCompute)
			{
				cullingPrepassCompute = new ComputeSubmission;
				cullingResourcesChanged = true;
			}
			if (!cullingMainpassCompute)
			{
				cullingMainpassCompute = new ComputeSubmission;
				cullingResourcesChanged = true;
			}
			if (CullParamsBuf.getSize() == 0)
			{
				CullParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, nullptr, (unsigned int)sizeof(CullParamsStruct));
				cullingResourcesChanged = true;
			}
			CullParams.nInstances = (unsigned int)PSO_DSL_DS_GeomInstPairings.size();

			std::vector<unsigned int> trackedVisData;
			InstanceCullData.resize(PSO_DSL_DS_GeomInstPairings.size());
			trackedVisData.resize((unsigned int)ceil((double)PSO_DSL_DS_GeomInstPairings.size() / 32.0));
			memset(trackedVisData.data(), 0, trackedVisData.size() * sizeof(unsigned int));

			RasterPipelineStateClass* lastPSOPtr = PSO_DSL_DS_GeomInstPairings[0].PSO_DSL->PSO;
			RasterPipelineStateClass* lastPSOMainPassPtr = PSO_DSL_DS_GeomInstPairings[0].PSO_DSL->PSOMainPass;
			DescriptorSets* lastDSPtr = PSO_DSL_DS_GeomInstPairings[0].DS;
			unsigned int curIndirectBatchId = 0u;
			Rasterlet.indirectBoundaries.clear();
			Rasterlet.indirectBoundaries.emplace_back(Rasterlet.IndirectDrawCounterSize(), (unsigned int)PSO_DSL_DS_GeomInstPairings.size(), lastPSOPtr, lastPSOMainPassPtr, lastDSPtr);
			for (unsigned int i = 0; i != PSO_DSL_DS_GeomInstPairings.size(); i++)
			{
				if (lastPSOPtr != PSO_DSL_DS_GeomInstPairings[i].PSO_DSL->PSO || lastDSPtr != PSO_DSL_DS_GeomInstPairings[i].DS)
				{
					lastPSOPtr = PSO_DSL_DS_GeomInstPairings[i].PSO_DSL->PSO;
					lastPSOMainPassPtr = PSO_DSL_DS_GeomInstPairings[i].PSO_DSL->PSOMainPass;
					lastDSPtr = PSO_DSL_DS_GeomInstPairings[i].DS;
					curIndirectBatchId++;
					unsigned long long batchDrawArgumentsOffset = (unsigned long long)(curIndirectBatchId * Rasterlet.IndirectBufferSize((unsigned int)PSO_DSL_DS_GeomInstPairings.size(), 1) + Rasterlet.IndirectDrawCounterSize());
					Rasterlet.indirectBoundaries.emplace_back(batchDrawArgumentsOffset, (unsigned int)PSO_DSL_DS_GeomInstPairings.size(), lastPSOPtr, lastPSOMainPassPtr, lastDSPtr);
				}
				PSO_DSL_DS_GeomInstPairing& curPairing = PSO_DSL_DS_GeomInstPairings[i];
				InstanceCullData[i].instId = curPairing.inst;
				InstanceCullData[i].indirectBatchId = curIndirectBatchId;
				InstanceCullData[i].indexCount = curPairing.geom->getTriCount() * 3;
				InstanceCullData[i].firstIndex = curPairing.geom->getIndexOffsetInGiantVertexBuffer();
				InstanceCullData[i].vertexOffset = curPairing.geom->getVertexOffsetInGiantVertexBuffer();
			}
			if (InstanceCullBuf.getSize() < (unsigned int)InstanceCullData.size() * sizeof(InstanceCullStruct))
			{
				InstanceCullBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, InstanceCullData.data(), (unsigned int)InstanceCullData.size() * sizeof(InstanceCullStruct));
				cullingResourcesChanged = true;
			}
			else
				InstanceCullBuf.UploadSubData(0, InstanceCullData.data(), (unsigned int)InstanceCullData.size() * sizeof(InstanceCullStruct));
			if (TrackedVisDataBuf.getSize() < (unsigned int)trackedVisData.size() * sizeof(unsigned int))
			{
				TrackedVisDataBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, trackedVisData.data(), (unsigned int)trackedVisData.size() * sizeof(unsigned int));
				cullingResourcesChanged = true;
			}
			else
				TrackedVisDataBuf.UploadSubData(0, trackedVisData.data(), (unsigned int)trackedVisData.size() * sizeof(unsigned int));

			unsigned int indBufSize = Rasterlet.IndirectBufferSize((unsigned int)PSO_DSL_DS_GeomInstPairings.size(), curIndirectBatchId + 1);

			if (!Rasterlet.indirectDrawBuffer || Rasterlet.indirectDrawBuffer->getSize() < indBufSize)
			{
				if (Rasterlet.indirectDrawBuffer) delete Rasterlet.indirectDrawBuffer;
				Rasterlet.indirectDrawBuffer = new BufferClass(MEMORY_DEVICE_LOCAL, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_DST | USAGE_SSBO | USAGE_INDIRECT, Instance, nullptr, indBufSize);
				cullingResourcesChanged = true;
			}
			if (cullingResourcesChanged)
			{
				if (cullingPrepassResourceSet) delete cullingPrepassResourceSet;
				if (cullingMainpassResourceSet) delete cullingMainpassResourceSet;
				cullingPrepassResourceSet = new ShaderResourceSet;
				cullingMainpassResourceSet = new ShaderResourceSet;

				cullingPrepassResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, CullParamsBuf);
				cullingPrepassResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 1, cachedCullingFrustum->Buffer);
				cullingPrepassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer);
				cullingPrepassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, InstanceCullBuf);
				cullingPrepassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 4, *Rasterlet.indirectDrawBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
				cullingPrepassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 5, TrackedVisDataBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
				cullingPrepassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 6, *GroupedRenderSubmission::SceneData->transformBuffer);
				cullingPrepassResourceSet->Create(cullMode == TWOPASS_NO_FRUSTUM ? "shaders/twoPassCullPrepassNoFrustum.comp.spv" : "shaders/twoPassCullPrepass.comp.spv", "main");
				cullingPrepassCompute->MakeDispatch(Instance, std::string("cullPrepass"), *cullingPrepassResourceSet, (unsigned int)ceil((double)GroupedRenderSubmission::SceneData->instancePropsCount / (double)WorkGroupTwoPassCullX()), 1, 1);

				cullingMainpassResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 0, CullParamsBuf);
				cullingMainpassResourceSet->AddResource(RESOURCE_UBO, COMPUTE, 0, 1, cachedCullingFrustum->Buffer);
				cullingMainpassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 2, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer);
				cullingMainpassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 3, InstanceCullBuf);
				cullingMainpassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 4, *Rasterlet.indirectDrawBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
				cullingMainpassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 5, TrackedVisDataBuf, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
				cullingMainpassResourceSet->AddResource(RESOURCE_SSBO, COMPUTE, 0, 6, *GroupedRenderSubmission::SceneData->transformBuffer);
				std::vector<ShaderResource> mipChain;
				for (int i = 0; i != (cachedCullingFrustum->reverseZ ? MinZChain.size() : MaxZChain.size()); i++)
					mipChain.emplace_back(RESOURCE_SAMPLER, COMPUTE, 0, 7, (cachedCullingFrustum->reverseZ ? MinZChain[i] : MaxZChain[i]));
				cullingMainpassResourceSet->AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 7, mipChain);
				cullingMainpassResourceSet->Create(cullMode == TWOPASS_NO_FRUSTUM ? "shaders/twoPassCullMainpassNoFrustum.comp.spv" : (cachedCullingFrustum->reverseZ ? "shaders/twoPassCullMainpassReverseZ.comp.spv" : "shaders/twoPassCullMainpass.comp.spv"), "main");
				cullingMainpassCompute->MakeDispatch(Instance, std::string("cullMainpass"), *cullingMainpassResourceSet, (unsigned int)ceil((double)GroupedRenderSubmission::SceneData->instancePropsCount / (double)WorkGroupTwoPassCullX()), 1, 1);

				cullingResourcesChanged = false;
			}
		}
		else if(!doesCulling && PSO_DSL_DS_GeomInstPairings.size() > 0)
		{
			std::vector<RasterletClass::DrawCommand> indirectDraws;
			unsigned int curDrawOffset = 0u;
			unsigned int curDrawCount = 0u;
			indirectDraws.reserve(PSO_DSL_DS_GeomInstPairings.size());
			RasterPipelineStateClass* lastPSOPtr = PSO_DSL_DS_GeomInstPairings[0].PSO_DSL->PSO;
			RasterPipelineStateClass* lastPSOMainPassPtr = PSO_DSL_DS_GeomInstPairings[0].PSO_DSL->PSOMainPass;
			DescriptorSets* lastDSPtr = PSO_DSL_DS_GeomInstPairings[0].DS;

			Rasterlet.indirectBoundaries.clear();
			Rasterlet.indirectBoundaries.reserve(10);
			for (unsigned int i = 0; i != PSO_DSL_DS_GeomInstPairings.size(); i++)
			{
				if (lastPSOPtr != PSO_DSL_DS_GeomInstPairings[i].PSO_DSL->PSO || lastDSPtr != PSO_DSL_DS_GeomInstPairings[i].DS)
				{
					Rasterlet.indirectBoundaries.emplace_back((unsigned long long)(curDrawOffset * sizeof(RasterletClass::DrawCommand)), curDrawCount, lastPSOPtr, lastPSOMainPassPtr, lastDSPtr);
					lastPSOPtr = PSO_DSL_DS_GeomInstPairings[i].PSO_DSL->PSO;
					lastPSOMainPassPtr = PSO_DSL_DS_GeomInstPairings[i].PSO_DSL->PSOMainPass;
					lastDSPtr = PSO_DSL_DS_GeomInstPairings[i].DS;
					curDrawOffset += curDrawCount;
					curDrawCount = 0u;
				}
				curDrawCount++;
				indirectDraws.emplace_back(
					PSO_DSL_DS_GeomInstPairings[i].geom->getTriCount() * 3,
					1,
					PSO_DSL_DS_GeomInstPairings[i].geom->getIndexOffsetInGiantVertexBuffer(),
					PSO_DSL_DS_GeomInstPairings[i].geom->getVertexOffsetInGiantVertexBuffer(),
					PSO_DSL_DS_GeomInstPairings[i].inst);
			}
			if (curDrawCount > 0)
			{
				Rasterlet.indirectBoundaries.emplace_back((unsigned long long)(curDrawOffset * sizeof(RasterletClass::DrawCommand)), curDrawCount, lastPSOPtr, lastPSOMainPassPtr, lastDSPtr);
				curDrawOffset += curDrawCount;
				curDrawCount = 0u;
			}

			if (!Rasterlet.indirectDrawBuffer || Rasterlet.indirectDrawBuffer->getSize() < indirectDraws.size() * sizeof(RasterletClass::DrawCommand))
			{
				if (Rasterlet.indirectDrawBuffer) delete Rasterlet.indirectDrawBuffer;
				Rasterlet.indirectDrawBuffer = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO | USAGE_INDIRECT, Instance, indirectDraws.data(), (unsigned int)(indirectDraws.size() * sizeof(RasterletClass::DrawCommand)));
			}
			else
				Rasterlet.indirectDrawBuffer->UploadSubData(0, indirectDraws.data(), (unsigned int)(indirectDraws.size() * sizeof(RasterletClass::DrawCommand)));
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
		Rasterlet.PrepareSubmission(PSO_DSL_DS_GeomInstPairings, dynPipelineFlags, *frameBuffer, doesCulling ? mainPassFrameBuffer : nullptr);
		recordedCmdBuf = true;
	}

	if (doesCulling && PSO_DSL_DS_GeomInstPairings.size() > 0)
	{
		CullParamsBuf.UploadSubData(0, &CullParams, sizeof(CullParamsStruct));
		Rasterlet.indirectDrawBuffer->Clear();
		(*cullingPrepassCompute).MakeAsync().Submit();
		Rasterlet.Draw(true);
		(*MipChainPass1_HiZ).MakeAsync().Submit();
		(*MipChainPass2_HiZ).MakeAsync().Submit();
		Rasterlet.indirectDrawBuffer->Clear();
		(*cullingMainpassCompute).MakeAsync().Submit();
	}
	Rasterlet.Draw();
}

HIGHOMEGA::RENDER::ComputeSubmission::ComputeSubmission()
{
	resourcesRequested = 0;
}

void HIGHOMEGA::RENDER::ComputeSubmission::MakeDispatch(InstanceClass & ptrToInstance, const std::string & inpName, ShaderResourceSet & inpShader, int inpGroupX, int inpGroupY, int inpGroupZ)
{
	if (!initComputelet)
	{
		computelet.Computelet(ptrToInstance);
		initComputelet = true;
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

	if (dispatches.find(inpName) == dispatches.end())
	{
		dispatches[inpName].DS = new DescriptorSets(computePsoDsl->DSL);
		dispatches[inpName].DS->WriteDescriptorSets(inpShader.getAdditionalResources());
	}
	else
	{
		dispatches[inpName].DS->RewriteDescriptorSets(inpShader.getAdditionalResources());
	}

	if (inpShader.areAdditionalResourcesSwapchainDependent())
	{
		dispatches[inpName].cachedSwapchainDependentResources = inpShader.getAdditionalResources();
		swapchainId = Instance.getSwapchainId();
	}
	else
	{
		dispatches[inpName].cachedSwapchainDependentResources.clear();
		swapchainId = 0ull;
	}

	unsigned int curResourcesRequested = 0;
	for (ShaderResource& curRes : inpShader.getAdditionalResources())
	{
		curResourcesRequested += curRes.ResourceCount();
		if (curRes.IsSimultaneouslyProduced()) dispatches[inpName].waitOnOld.merge(curRes.GetAsDependencies());
		else dispatches[inpName].waitOn.merge(curRes.GetAsDependencies());
		if (curRes.IsProduced()) dispatches[inpName].signal.merge(curRes.GetAsDependencies());
	}

	resourcesRequested = curResourcesRequested;
	dispatches[inpName].groupX = inpGroupX;
	dispatches[inpName].groupY = inpGroupY;
	dispatches[inpName].groupZ = inpGroupZ;

	updateResources = true;
}

void HIGHOMEGA::RENDER::ComputeSubmission::UpdateDispatchSize(InstanceClass & ptrToInstance, const std::string & inpName, int inpGroupX, int inpGroupY, int inpGroupZ)
{
	if (!initComputelet)
	{
		computelet.Computelet(ptrToInstance);
		initComputelet = true;
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

ComputeSubmission& HIGHOMEGA::RENDER::ComputeSubmission::MakeAsync()
{
	cpuSync = false;
	return *this;
}

ComputeSubmission& HIGHOMEGA::RENDER::ComputeSubmission::UseComputeQueue()
{
	useComputeQueue = true;
	return *this;
}

void HIGHOMEGA::RENDER::ComputeSubmission::Submit()
{
	if (!instanceRef) return;

	if (swapchainId && swapchainId != Instance.getSwapchainId())
	{
		for (std::pair <const std::string, SingleDispatch>& curDispatch : dispatches)
			if (curDispatch.second.cachedSwapchainDependentResources.size())
				curDispatch.second.DS->RewriteDescriptorSets(curDispatch.second.cachedSwapchainDependentResources);
		swapchainId = Instance.getSwapchainId();
		updateResources = true;
	}

	if (updateResources)
	{
		waitOn.clear();
		waitOnOld.clear();
		signal.clear();
		computelet.Start(*computePsoDsl->PSO, useComputeQueue ? COMPUTE_QUEUE : GRAPHICS_QUEUE);
		for (std::pair<const std::string, SingleDispatch>& curDispatch : dispatches)
		{
			computelet.Dispatch(*computePsoDsl->PSO, *curDispatch.second.DS, curDispatch.second.groupX, curDispatch.second.groupY, curDispatch.second.groupZ);
			waitOn.merge(curDispatch.second.waitOn);
			waitOnOld.merge(curDispatch.second.waitOnOld);
			signal.merge(curDispatch.second.signal);
		}
		computelet.End();

		updateResources = false;
	}

	computelet.WaitOnSemaphores(waitOn);
	computelet.WaitOnOldSemaphores(waitOnOld);
	computelet.SignalSemaphores(signal);
	if (!cpuSync) computelet.NoCPUSync();
	computelet.Submit();
}

HIGHOMEGA::RENDER::ComputeSubmission::~ComputeSubmission()
{
	for (std::pair<const std::string, SingleDispatch> curDispatch : dispatches)
		delete curDispatch.second.DS;
	dispatches.clear();
	waitOn.clear();
	waitOnOld.clear();
	signal.clear();

	updateResources = false;
	initComputelet = false;
	useComputeQueue = false;
	cpuSync = true;
}

void HIGHOMEGA::RENDER::FrustumClass::CreatePerspective(const vec3& eye, const vec3& look, const vec3& up, float fov, float whr, float clipNear, float clipFar)
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

void HIGHOMEGA::RENDER::FrustumClass::CreateOrtho(const vec3& eye, const vec3& look, const vec3& up, float left, float right, float bottom, float top, float clipNear, float clipFar)
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
	look_norm = look.normalized();
	side = cross(look_norm, up).normalized();
	actual_up = cross(side, look_norm);

	if (headRotateAmount > 0.0f)
	{
		actual_up = Spin(look_norm, actual_up, headRotateAmount);
		side = Spin(look_norm, side, headRotateAmount);
	}

	mat4 orient_matrix;
	orient_matrix.Ident();
	orient_matrix.i[0][0] = side.x;
	orient_matrix.i[1][0] = side.y;
	orient_matrix.i[2][0] = side.z;

	orient_matrix.i[0][1] = actual_up.x;
	orient_matrix.i[1][1] = actual_up.y;
	orient_matrix.i[2][1] = actual_up.z;

	orient_matrix.i[0][2] = -look_norm.x;
	orient_matrix.i[1][2] = -look_norm.y;
	orient_matrix.i[2][2] = -look_norm.z;

	mat4 translate_matrix;
	translate_matrix.Ident();
	if (!cameraRelative)
	{
		translate_matrix.i[3][0] = -eye.x;
		translate_matrix.i[3][1] = -eye.y;
		translate_matrix.i[3][2] = -eye.z;
	}

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
	projection_matrix.i[2][3] = screen_near * near_far_inv;
	projection_matrix.i[3][3] = 1.0f;

	modelviewprojection_matrix = projection_matrix * modelview_matrix;
}

void HIGHOMEGA::RENDER::FrustumClass::Update(const vec3& eyeInBuffer, const vec3& lookInBuffer, const vec3& upInBuffer, float whrInBuffer, float fovYForBuffer)
{
	if (!initBuffer)
	{
		Buffer.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &uboData, (unsigned int)sizeof(uboData));
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
	uboData.whrTanHalfFovY[1] = tanf(((fovYForBuffer / 180.0f) * HIGHOMEGA_PI) * 0.5f);

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

void HIGHOMEGA::RENDER::FrustumClass::CopyFromFrustum(const FrustumClass & Other)
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
	cameraRelative = Other.cameraRelative;
	headRotateAmount = Other.headRotateAmount;
}

void HIGHOMEGA::RENDER::FrustumClass::LerpEyeLookUpWithFrustum(const FrustumClass& Other1, const FrustumClass& Other2, float alpha)
{
	eye = Lerp(Other1.eye, Other2.eye, alpha);
	look = Lerp(Other1.look, Other2.look, alpha);
	up = Lerp(Other1.up, Other2.up, alpha);
}

void HIGHOMEGA::RENDER::FrustumClass::ForceEyeAndLook(const vec3& inpEye, const vec3& inpLook)
{
	look = inpLook;
	eye = inpEye;
}

vec3 HIGHOMEGA::RENDER::FrustumClass::GetUploadedUp()
{
	return vec3(uboData.upEyeY[0], uboData.upEyeY[1], uboData.upEyeY[2]);
}

vec3 HIGHOMEGA::RENDER::FrustumClass::GetUploadedSide()
{
	return vec3(uboData.sideEyeZ[0], uboData.sideEyeZ[1], uboData.sideEyeZ[2]);
}

HIGHOMEGA::RENDER::ScreenSizeClass::ScreenSizeClass()
{
}

void HIGHOMEGA::RENDER::ScreenSizeClass::Create(unsigned int width, unsigned int height)
{
	this->width = width;
	this->height = height;
}

void HIGHOMEGA::RENDER::PASSES::TriClass::Create(const vec3& eyeInBuffer, const vec3& lookInBuffer, const vec3& upInBuffer, float whrInBuffer, float fovYForBuffer)
{
	Mesh triMesh = Mesh("assets/models/tri/tri.3md");
	triModel.Model(triMesh, "assets/models/tri/", Instance);
	triModelInstance = triModel.CreateInstance();

	triFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, 1.0f, 1.0f, 10.0f);

	UpdateViewSpace(eyeInBuffer, lookInBuffer, upInBuffer, whrInBuffer, fovYForBuffer);
}

void HIGHOMEGA::RENDER::PASSES::TriClass::UpdateViewSpace(const vec3& eyeInBuffer, const vec3& lookInBuffer, const vec3& upInBuffer, float whrInBuffer, float fovYForBuffer)
{
	triFrustum.Update(eyeInBuffer, lookInBuffer, upInBuffer, whrInBuffer, fovYForBuffer);
}

unsigned long long HIGHOMEGA::RENDER::WorldParamsClass::Populate(Mesh & inpMesh)
{
	unsigned long long curId = threadSafeMersenneTwister64Bit();

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		float tmpFloat;
		if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "sceneProps", tmpFloat)) continue;

		if (!Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", allItems[curId].startPlayerPos)) FATAL_ERROR("Start player pos not found");
		allItems[curId].firstPersonControls = Mesh::getDataRowFloat(curPolyGroup, "PROPS", "firstPersonControls", tmpFloat);
		if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "lightShaftAmount", allItems[curId].lightShaftAmount)) allItems[curId].lightShaftAmount = 0.0f;
		if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "lightShaftExtinction", allItems[curId].lightShaftExtinction)) allItems[curId].lightShaftExtinction = 0.98f;
		if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "sunDirectLightStrength", allItems[curId].sunDirectLightStrength)) allItems[curId].sunDirectLightStrength = 1.0f;

		allItems[curId].sunAngle = 0.0f;
		allItems[curId].lastFrameTime = 0.0f;
		allItems[curId].showAurora = false;
		allItems[curId].forceSunAngle = false;
		allItems[curId].renderTime.cur = 0.0f;
		allItems[curId].renderTime.prev = 0.0f;
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
	if (allItems.size() == 0) return;
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
	Parameters& ourParams = allItems.begin()->second;
	ourParams.renderTime.prev = ourParams.renderTime.cur;
	ourParams.renderTime.cur += addRenderTime;
	if (!ourParams.renderTimeBuffer)
		ourParams.renderTimeBuffer = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &ourParams.renderTime, (unsigned int)sizeof(ourParams.renderTime));
	else
		ourParams.renderTimeBuffer->UploadSubData(0, &ourParams.renderTime, (unsigned int)sizeof(ourParams.renderTime));
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
	if (allItems.size() == 0) return vec3(0.0f, 1.0f, 0.0f);
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

void HIGHOMEGA::RENDER::WorldParamsClass::SetLightShaftAmount(float lightShaftAmount)
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.lightShaftAmount = lightShaftAmount;
}

void HIGHOMEGA::RENDER::WorldParamsClass::SetLightShaftExtinction(float lightShaftExtinction)
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.lightShaftExtinction = lightShaftExtinction;
}

void HIGHOMEGA::RENDER::WorldParamsClass::SetSunDirectLightStrength(float sunDirectLightStrength)
{
	if (allItems.size() == 0) return;
	allItems.begin()->second.sunDirectLightStrength = sunDirectLightStrength;
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
	if (allItems.size() == 0) FATAL_ERROR("No sceneProps to provide a renderTimeBuffer for");
	Parameters & ourParams = allItems.begin()->second;
	if (!ourParams.renderTimeBuffer)
		ourParams.renderTimeBuffer = new BufferClass(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &ourParams.renderTime, (unsigned int)sizeof(ourParams.renderTime));
	else
		ourParams.renderTimeBuffer->UploadSubData(0, &ourParams.renderTime, (unsigned int)sizeof(ourParams.renderTime));
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
	float front = ((viewMax - viewMin).length() + shadowMapSideBias * 2.0f) * 2.0f;
	float back = -front;

	sunEye += toSkyObject * front * 0.5f;

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

ShadowMapClass& HIGHOMEGA::RENDER::PASSES::ShadowMapClass::SetMode(SHADOWMAP_MODE inpMode)
{
	if (inpMode == NONE) FATAL_ERROR("You shouldn't go back to mode none for shadow maps");
	if (mode != NONE) return *this;

	mode = inpMode;
	switch (mode)
	{
	case ORTHO:
	case PERSPECTIVE:
		perPassData.resize(1);
		break;
	case PERSPECTIVE_CUBIC:
		perPassData.resize(6);
		break;
	default:
		FATAL_ERROR("Unknown shadow map mode");
		break;
	}
	
	return *this;
}

GroupedRasterSubmission& HIGHOMEGA::RENDER::PASSES::ShadowMapClass::getSubmission(unsigned int passIdx)
{
	if (passIdx >= perPassData.size()) FATAL_ERROR("Requested shadow map submission doesn't exist");
	return perPassData[passIdx].submission;
}

ImageClass& HIGHOMEGA::RENDER::PASSES::ShadowMapClass::getDSAttach(unsigned int passIdx)
{
	if (passIdx >= perPassData.size()) FATAL_ERROR("Requested shadow depth stencil attachment doesn't exist");
	return perPassData[passIdx].depthStencilAttach;
}

ImageClass& HIGHOMEGA::RENDER::PASSES::ShadowMapClass::getColorAttach(unsigned int passIdx)
{
	if (passIdx >= perPassData.size()) FATAL_ERROR("Requested shadow color attachment doesn't exist");
	return perPassData[passIdx].shadowColorAttach;
}

FrustumClass& HIGHOMEGA::RENDER::PASSES::ShadowMapClass::getFrustum(unsigned int passIdx)
{
	if (passIdx >= perPassData.size()) FATAL_ERROR("Requested shadow frustum doesn't exist");
	return perPassData[passIdx].frustum;
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::Create(WorldParamsClass & WorldParams, vec2 samplingBiasMinMax, unsigned int sideResolution)
{
	if (HIGHOMEGA::creativeLicense) sideResolution = 2048u;

	ptrWorldParams = &WorldParams;
	samplingBias = samplingBiasMinMax;

	if (perPassData.size() == 0) FATAL_ERROR("Shadow map mode is unset during create");
	if (perPassData.size() == 6)
	{
		perPassData[0].depthStencilAttach.CreateOffScreenDepthStencilCubeMap(Instance, sideResolution, sideResolution, ImageClass::SAMPLE_DEPTH);
		perPassData[0].shadowColorAttach.CreateCubeMap(Instance, R8G8B8A8UN, sideResolution , sideResolution);
	}

	for (int i = 0; i != perPassData.size(); i++)
	{
		PerPassData& passData = perPassData[i];

		passData.frustum.CreatePerspective(vec3(0.0f), vec3(1.0f, 0.0f, 0.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, 1.0f, 1.0f, 10000.0f);
		passData.frustum.Update();

		passData.submission.SetClearColor(vec3(0.0f, 0.0f, 0.0f), 0.0f);
		passData.submission.Create(Instance);

		if (perPassData.size() != 6)
		{
			passData.shadowColorAttach.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, sideResolution, sideResolution, true, true);
			passData.depthStencilAttach.CreateOffScreenDepthStencil(Instance, sideResolution, sideResolution, ImageClass::SAMPLE_DEPTH);
			passData.frameBuffer.AddColorAttachment(passData.shadowColorAttach);
			passData.frameBuffer.SetDepthStencil(passData.depthStencilAttach);
		}
		else
		{
			passData.frameBuffer.AddColorAttachmentWithLayer(perPassData[0].shadowColorAttach, i);
			passData.frameBuffer.SetDepthStencilWithLayer(perPassData[0].depthStencilAttach, i);
		}
		passData.frameBuffer.Create(OFF_SCREEN, Instance, Window);

		PipelineFlags defaultPipelineFlags;
		defaultPipelineFlags.backFaceCulling = true;
		defaultPipelineFlags.redMask = false;
		defaultPipelineFlags.greenMask = false;
		defaultPipelineFlags.blueMask = false;
		defaultPipelineFlags.alphaMask = false;
		passData.submission.SetDefaultPipelineFlags(defaultPipelineFlags);
		passData.submission.matTransform = [](const MeshMaterial& inMat) ->MeshMaterial {
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
				retMat.pipelineFlags.changedDepthWrite = true;
				retMat.pipelineFlags.changedColorMask = true;
				retMat.renderOrder = INT_MAX;
				if (inMat.shaderName == "shaderTessScreenSpace") {
					retMat.shaderName = "shadowStainedGlassTess";
				}
				else {
					retMat.shaderName = "shadowStainedGlass";
				}
			}

			return retMat;
			};
		passData.submission.SetFrameBuffer(passData.frameBuffer);

		passData.shader.Create("shaders/shadow.vert.spv", "main", "shaders/shadow.frag.spv", "main");
		passData.shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, passData.frustum.Buffer);
		passData.shaderAlpha.Create("shaders/shadow.vert.spv", "main", "shaders/shadowAlpha.frag.spv", "main");
		passData.shaderAlpha.AddResource(RESOURCE_UBO, VERTEX, 0, 0, passData.frustum.Buffer);
		passData.shaderStainedGlass.Create("shaders/shadow.vert.spv", "main", "shaders/shadowStainedGlass.frag.spv", "main");
		passData.shaderStainedGlass.AddResource(RESOURCE_UBO, VERTEX, 0, 0, passData.frustum.Buffer);
		passData.shaderStainedGlassTess.Create("shaders/shadow.vert.spv", "main", "shaders/shadow.tesc.spv", "main", "shaders/shadow.tese.spv", "main", "shaders/shadowStainedGlass.frag.spv", "main");
		passData.shaderStainedGlassTess.AddResource(RESOURCE_UBO, VERTEX | TESS_EVAL, 0, 0, passData.frustum.Buffer);
		passData.shaderStainedGlassTess.AddResource(RESOURCE_UBO, TESS_EVAL, 0, 1, WorldParams.GetRenderTimeBuffer());
		passData.submission.SetShader("default", passData.shader);
		passData.submission.SetShader("shadowAlpha", passData.shaderAlpha);
		passData.submission.SetShader("shadowStainedGlass", passData.shaderStainedGlass);
		passData.submission.SetShader("shadowStainedGlassTess", passData.shaderStainedGlassTess);
	}
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::RenderOrtho(const vec3& viewMin, const vec3& viewMax, float shadowMapSideBias, FrustumClass *nearCascadeFrustum)
{
	if (mode != ORTHO) FATAL_ERROR("Wrong mode trying to render ortho shadow");
	PerPassData& passData = perPassData[0];

	if (nearCascadeFrustum)
	{
		vec3 viewSideLenScaled = (viewMax - viewMin) * 0.05f;
		viewSideLenScaled.x = min(viewSideLenScaled.x, 100.0f);
		viewSideLenScaled.y = min(viewSideLenScaled.y, 100.0f);
		viewSideLenScaled.z = min(viewSideLenScaled.z, 100.0f);
		vec3 projectedLook = vec3(nearCascadeFrustum->look.x, 0.0f, nearCascadeFrustum->look.z).normalized() * vec3 (viewSideLenScaled.x, 0.0f, viewSideLenScaled.y).length();
		orthoData.viewMax = nearCascadeFrustum->eye - vec3(0.0f, 18.0f, 0.0f) + viewSideLenScaled + projectedLook;
		orthoData.viewMin = nearCascadeFrustum->eye - vec3(0.0f, 18.0f, 0.0f) - viewSideLenScaled + projectedLook;
	}
	else
	{
		orthoData.viewMax = viewMax;
		orthoData.viewMin = viewMin;
		if (orthoData.frameCount % 60 != 0)
		{
			orthoData.frameCount = (orthoData.frameCount + 1) % 60;
			if (!HIGHOMEGA::creativeLicense) return; // For the main demo, update even the far cascade every frame.
		}
		orthoData.frameCount = (orthoData.frameCount + 1) % 60;
	}
	this->shadowMapSideBias = shadowMapSideBias;
	ComputeOptimalOrthoFrustum(orthoData.viewMin, orthoData.viewMax, ptrWorldParams->SunOrMoonDir(), passData.frustum);
	passData.frustum.cameraRelative = true;
	passData.frustum.Update();

	passData.submission.doCulling(passData.frustum, nearCascadeFrustum ? GroupedRasterSubmission::CULL_MODE::TWOPASS : GroupedRasterSubmission::CULL_MODE::TWOPASS_NO_FRUSTUM);
	passData.submission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::RenderPerspective(FrustumClass& origFrustum, const vec3& eyeOverride, const vec3& lookOverride, float fovOverride)
{
	if (mode != PERSPECTIVE) FATAL_ERROR("Wrong mode trying to render perspective shadow");
	PerPassData& passData = perPassData[0];

	passData.frustum.CopyFromFrustum(origFrustum);
	passData.frustum.screen_fov = fovOverride;
	passData.frustum.eye = eyeOverride;
	passData.frustum.look = lookOverride;
	passData.frustum.cameraRelative = true;
	passData.frustum.Update();

	passData.submission.doCulling(passData.frustum, GroupedRasterSubmission::CULL_MODE::TWOPASS);
	passData.submission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapClass::RenderCubicPerspective(const vec3& eye)
{
	for (int i = 0; i != 6; i++)
	{
		PerPassData& passData = perPassData[i];

		vec3 lookVec, upVec;
		GetCubeFaceLookUp(i, lookVec, upVec);
		passData.frustum.CreatePerspective(eye, lookVec, upVec, 90.0f, 1.0f, 1.0f, 10000.0f);
		passData.frustum.cameraRelative = true;
		passData.frustum.Update();
		passData.submission.doCulling(passData.frustum, GroupedRasterSubmission::CULL_MODE::TWOPASS, i);
		passData.submission.MakeAsync().Render();
	}
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::CreateDirectional(TriClass & PostProcessTri, ShadowMapClass & ShadowMapCascadeNear, ShadowMapClass & ShadowMapCascadeFar, GatherPassClassCommon & GatherPass, const std::string& ShadowTag, bool useHWRTIfAvailable)
{
	postProcessTriRef = &PostProcessTri;
	gatherPassCommonRef = &GatherPass;
	shadowMapNearRef = &ShadowMapCascadeNear;
	shadowMapFarRef = &ShadowMapCascadeFar;
	useRTIfAvailableCached = useHWRTIfAvailable;
	ShadowTagCopy = ShadowTag;

	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);

	if (GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback())
	{
		frameBuffer.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
		frameBuffer.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	}
	else
	{
		frameBuffer.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBuffer.setHeight(GatherPass.worldPosAttach.getHeight());
	}
	PipelineFlags shadowScreenPF;
	shadowScreenPF.depthTest = shadowScreenPF.depthWrite = false;
	submission.SetDefaultPipelineFlags(shadowScreenPF);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);
	submission.SetFrameBuffer(frameBuffer);

	shadowMapScreen.CreateImageStore(Instance, R8G8B8A8UN, GatherPass.worldPosAttach.getWidth(), GatherPass.worldPosAttach.getHeight(), 1, _2D, false,
		(GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback()));

	submissionH.Add(*PostProcessTri.triModelInstance);
	submissionH.Create(Instance);
	submissionV.Add(*PostProcessTri.triModelInstance);
	submissionV.Create(Instance);

	blurHAttach.CreateImageStore(Instance, R8G8B8A8UN, GatherPass.worldPosAttach.getWidth(), GatherPass.worldPosAttach.getHeight(), 1, _2D, false, 
		(GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback()));

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;
	if (GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback())
	{
		frameBufferH.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
		frameBufferH.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	}
	else
	{
		frameBufferH.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBufferH.setHeight(GatherPass.worldPosAttach.getHeight());
	}
	frameBufferH.Create(OFF_SCREEN, Instance, Window);
	if (GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback())
	{
		frameBufferV.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
		frameBufferV.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	}
	else
	{
		frameBufferV.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBufferV.setHeight(GatherPass.worldPosAttach.getHeight());
	}
	frameBufferV.Create(OFF_SCREEN, Instance, Window);
	submissionH.SetFrameBuffer(frameBufferH);
	submissionV.SetFrameBuffer(frameBufferV);
	submissionH.SetDefaultPipelineFlags(defPipelineFlags);
	submissionV.SetDefaultPipelineFlags(defPipelineFlags);

	shaderH.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSep.frag.spv", "main");
	shaderV.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSep.frag.spv", "main");

	shaderH.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, shadowMapScreen);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, blurHAttach);

	shaderV.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, blurHAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, shadowMapScreen);
	shaderV.SetStageSpecializationData(FRAGMENT, { "VertPass", {{0u, 0u}, {1u, 1u}} });

	submissionH.SetShader("default", shaderH);

	submissionV.SetShader("default", shaderV);
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::CreateSingleCascade(TriClass& PostProcessTri, ShadowMapClass& ShadowMap, GatherPassClassCommon& GatherPass, ImageClass& blurHTextureToReuse, const std::string& ShadowTag, bool omniDir)
{
	singleCascade = true;

	gatherPassCommonRef = &GatherPass;
	shadowMapNearRef = &ShadowMap;
	ShadowTagCopy = ShadowTag;

	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);

	shadowMapScreen.CreateImageStore(Instance, R8G8B8A8UN, GatherPass.worldPosAttach.getWidth(), GatherPass.worldPosAttach.getHeight(), 1, _2D, false,
		(GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback()));

	if (GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback())
	{
		frameBuffer.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
		frameBuffer.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	}
	else
	{
		frameBuffer.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBuffer.setHeight(GatherPass.worldPosAttach.getHeight());
	}
	PipelineFlags shadowScreenPF;
	shadowScreenPF.depthTest = shadowScreenPF.depthWrite = false;
	submission.SetDefaultPipelineFlags(shadowScreenPF);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);
	submission.SetFrameBuffer(frameBuffer);

	shader.Create("shaders/postprocess.vert.spv", "main", omniDir ? "shaders/shadowScreenOmni.frag.spv" : "shaders/shadowScreenSingleCascade.frag.spv", "main");

	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPass.normalAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ShadowMap.getDSAttach());
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, ShadowMap.getColorAttach());
	if (omniDir)
	{
		std::vector<ShaderResource> frustaBuffers;
		for (int i = 0; i != 6; i++)
			frustaBuffers.emplace_back(RESOURCE_UBO, FRAGMENT, 0, 5, ShadowMap.getFrustum(i).Buffer);
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, frustaBuffers);
	}
	else
		shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, ShadowMap.getFrustum().Buffer);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, shadowMapScreen);
	shader.SetStageSpecializationData(FRAGMENT, { "ShadowBiases_" + ShadowTag, {
		{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
		{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)}
	} });

	submission.SetShader("default", shader);

	submissionH.Add(*PostProcessTri.triModelInstance);
	submissionH.Create(Instance);
	submissionV.Add(*PostProcessTri.triModelInstance);
	submissionV.Create(Instance);

	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;
	if (GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback())
	{
		frameBufferH.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
		frameBufferH.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	}
	else
	{
		frameBufferH.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBufferH.setHeight(GatherPass.worldPosAttach.getHeight());
	}
	frameBufferH.Create(OFF_SCREEN, Instance, Window);
	if (GatherPass.worldPosAttach.getSurfaceBasedWidthCallback() || GatherPass.worldPosAttach.getSurfaceBasedHeightCallback())
	{
		frameBufferV.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
		frameBufferV.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	}
	else
	{
		frameBufferV.setWidth(GatherPass.worldPosAttach.getWidth());
		frameBufferV.setHeight(GatherPass.worldPosAttach.getHeight());
	}
	frameBufferV.Create(OFF_SCREEN, Instance, Window);
	submissionH.SetFrameBuffer(frameBufferH);
	submissionV.SetFrameBuffer(frameBufferV);
	submissionH.SetDefaultPipelineFlags(defPipelineFlags);
	submissionV.SetDefaultPipelineFlags(defPipelineFlags);

	shaderH.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSep.frag.spv", "main");
	shaderV.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSep.frag.spv", "main");

	shaderH.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, shadowMapScreen);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, blurHTextureToReuse);

	shaderV.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPass.worldPosAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, blurHTextureToReuse);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, shadowMapScreen);
	shaderV.SetStageSpecializationData(FRAGMENT, { "VertPass", {{0u, 0u}, {1u, 1u}} });

	submissionH.SetShader("default", shaderH);

	submissionV.SetShader("default", shaderV);
}

FrustumClass* HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::getNearFrustum(unsigned int passIdx)
{
	if (shadowMapNearRef) return &shadowMapNearRef->getFrustum(passIdx);
	return nullptr;
}

void HIGHOMEGA::RENDER::PASSES::ShadowMapScreenClass::Render(GroupedTraceSubmission & rtSubmission)
{
	if (RTInstance::Enabled() && useRTIfAvailableCached && !singleCascade)
	{
		if (!builtHWRTSubmission)
		{
			rtShaderResourceSet.CreateRT("shaders/rtshadow.rgen.spv", "main", "shaders/rtshadow.rchit.spv", "main", "shaders/rtshadow.rmiss.spv", "main", "shaders/rtshadowalphakey.rahit.spv", "main");
			tracelet.Make(Instance);
			builtHWRTSubmission = true;
		}
		RTScene & rtSceneRef = rtSubmission.rtScene;
		bool rewriteDescriptoSets = false;
		unsigned long long curSceneId = rtSubmission.SceneID();
		if (curSceneId != lastSceneId)
		{
			lastSceneId = curSceneId;
			rewriteDescriptoSets = true;
			tracingResources.clear();
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, gatherPassCommonRef->worldPosAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 2, shadowMapScreen);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_MISS, 0, 3, shadowMapNearRef->getColorAttach());
			tracingResources.emplace_back(RESOURCE_UBO, RT_MISS | RT_RAYGEN, 0, 4, shadowMapNearRef->getFrustum().Buffer);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_MISS, 0, 5, shadowMapFarRef->getColorAttach());
			tracingResources.emplace_back(RESOURCE_UBO, RT_MISS, 0, 6, shadowMapFarRef->getFrustum().Buffer);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_ANYHIT, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_ANYHIT, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.lock_shared();
			tracingResources.emplace_back(RESOURCE_SSBO, RT_ANYHIT, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
		}

		tracelet.MakeAsync().Submit(gatherPassCommonRef->worldPosAttach.getWidth(), gatherPassCommonRef->worldPosAttach.getHeight(), 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
	}
	else
	{
		if (!builtSoftRTSubmission && !singleCascade)
		{
			shader.Create("shaders/postprocess.vert.spv", "main", "shaders/shadowScreen.frag.spv", "main");

			shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, postProcessTriRef->triFrustum.Buffer);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, gatherPassCommonRef->worldPosAttach);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, gatherPassCommonRef->normalAttach);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, shadowMapNearRef->getDSAttach());
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, shadowMapNearRef->getColorAttach());
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, shadowMapNearRef->getFrustum().Buffer);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, shadowMapFarRef->getDSAttach());
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, shadowMapFarRef->getColorAttach());
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, shadowMapFarRef->getFrustum().Buffer);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, shadowMapScreen);
			shader.SetStageSpecializationData(FRAGMENT, { "ShadowBiases_" + ShadowTagCopy, {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });

			submission.SetShader("default", shader);

			builtSoftRTSubmission = true;
		}
		submission.MakeAsync().Render();
	}
	submissionH.MakeAsync().Render();
	submissionV.MakeAsync().Render();
}

unsigned int HIGHOMEGA::RENDER::PASSES::SkyDomeClass::GetBackdropWidth()
{
	if (HIGHOMEGA::creativeLicense) return ScreenSize.width;
	return 640;
}

unsigned int HIGHOMEGA::RENDER::PASSES::SkyDomeClass::GetBackdropHeight()
{
	if (HIGHOMEGA::creativeLicense) return ScreenSize.height;
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
		return AuroraGreen() * 0.75f;
	else
		return NebulaBlue();
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::NebulaBlue()
{
	return vec3(0.12f, 0.12f, 0.2f);
}

vec3 HIGHOMEGA::RENDER::PASSES::SkyDomeClass::AddSkyColor()
{
	if (ptrWorldParams->isShowingAurora())
	{
		return AuroraGreen() * 0.5f;
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
	else if (ptrWorldParams->SunDir().y > -0.2f && ptrWorldParams->SunDir().y <= 0.0f)
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
		float lightIntensity = sqrt(ptrWorldParams->SunDir().y * 5.0f);
		return Lerp(SunOrange() * lightIntensity, SunWhite(), lightIntensity);
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

void HIGHOMEGA::RENDER::PASSES::SkyDomeClass::Create(TriClass & Tri, WorldParamsClass & WorldParams, MoBlurClass& MoBlur)
{
	ptrWorldParams = &WorldParams;

	Mesh domeMesh = Mesh("assets/models/skydome/skydome.3md");
	Mesh mountainsMesh = Mesh("assets/models/mountains/mountains.3md");
	skyDome.Model(domeMesh, "assets/models/skydome/", Instance);
	mountains.Model(mountainsMesh, "assets/models/mountains/", Instance);
	mountainsInst = mountains.CreateInstance();
	skyDomeInst = skyDome.CreateInstance();

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

	distantGeomShadowMapNear.SetMode(ShadowMapClass::SHADOWMAP_MODE::ORTHO).getSubmission().Add(*mountainsInst);
	distantGeomShadowMapFar.SetMode(ShadowMapClass::SHADOWMAP_MODE::ORTHO).getSubmission().Add(*mountainsInst);
	distantGeomShadowMapNear.Create(WorldParams, vec2(0.00025f, 0.0005f));
	distantGeomShadowMapFar.Create(WorldParams, vec2(0.00025f, 0.0005f));

	SkyDomeParams.invDims[0] = SkyDomeParams.invDims[1] = 1.0f / GetCubeResolution();
	FullResSkyDomeParams.invDims[0] = 1.0f / GetBackdropWidth();
	FullResSkyDomeParams.invDims[1] = 1.0f / GetBackdropHeight();
	SkyDomeParams.doAurora = FullResSkyDomeParams.doAurora = 0.0f;

	rayleighMieBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &rayleighMieInfo, (unsigned int)sizeof(rayleighMieInfo));
	skyDomeParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &SkyDomeParams, (unsigned int)sizeof(SkyDomeParams));
	fullResSkyDomeParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &FullResSkyDomeParams, (unsigned int)sizeof(FullResSkyDomeParams));

	skyCubeMap.CreateCubeMap(Instance, R16G16B16A16F, GetCubeResolution(), GetCubeResolution());
	fullCubeMap.CreateCubeMap(Instance, R16G16B16A16F, GetCubeResolution(), GetCubeResolution());
	skyBackDrop.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, GetBackdropWidth(), GetBackdropHeight(), true, false, HIGHOMEGA::creativeLicense);
	fullBackDrop.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, ScreenSize.width, ScreenSize.height, false, false, true);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, skyCubeMap.getWidth(), skyCubeMap.getHeight());
	backdropDS.depthStencilAttach.CreateOffScreenDepthStencil(Instance, GetBackdropWidth(), GetBackdropHeight(), ImageClass::SAMPLE_NONE, HIGHOMEGA::creativeLicense);

	for (int i = 0; i != 7; i++)
	{
		vec3 skyDomeLook, skyDomeUp;
		if (i < 6) {
			GetCubeFaceLookUp(i, skyDomeLook, skyDomeUp);
			skyFrustums[i].CreatePerspective(vec3(0.0f, InnerRad(), 0.0f), skyDomeLook, skyDomeUp, 90.0f, 1.0f, 0.1f, 10.0f);
			skyFrustums[i].Update();
		}
		else {
			skyFrustums[i].CopyFromFrustum(MainFrustum);
			skyFrustums[i].eye = vec3(0.0f, InnerRad(), 0.0f);
			skyFrustums[i].Update();
		}

		distantVis[i].Create(i < 6 ? skyCubeMap.getWidth() : ScreenSize.width, i < 6 ? skyCubeMap.getHeight() : ScreenSize.height, 0.1f, skyFrustums[i], i == 6);
		distantVis[i].submission.Add(*mountainsInst);
		distantGather[i].Create(distantVis[i], &MoBlur, true);

		distantGeomScreenShadow[i].CreateDirectional(Tri, distantGeomShadowMapNear, distantGeomShadowMapFar, distantGather[i], "Distant");

		skyShaders[i].Create("shaders/skyDomeRender.vert.spv", "main", "shaders/skyDomeRender.frag.spv", "main");
		skyShaders[i].AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, skyFrustums[i].Buffer);
		skyShaders[i].AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 1, rayleighMieBuf);
		skyShaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 2, i < 6 ? skyDomeParamsBuf : fullResSkyDomeParamsBuf);
		skyShaders[i].AddResource(RESOURCE_UBO, FRAGMENT, 0, 3, WorldParams.GetRenderTimeBuffer());
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, noiseImg);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, noiseImg2);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, moonImg);
		skyShaders[i].AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, nebulaImg);
		skyShaders[i].AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 8, distantGather[i].worldPosAttach);

		skySubmissions[i].Add(*skyDomeInst);
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
		skySubmissions[i].SetShader("default", skyShaders[i]);

		skyBoxCompositionParams.mode = i;
		skyBoxCompositionParamsBuf[i].Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &skyBoxCompositionParams, (unsigned int)sizeof(skyBoxCompositionParams));

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
		shaders[i].AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, distantGather[i].worldPosAttach);
		shaders[i].AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 10, distantGather[i].normalAttach);

		submissions[i].Add(*Tri.triModelInstance);
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
	distantGeomShadowMapNear.RenderOrtho(mountainMin, mountainMax, 1.0f, &skyFrustums[6]);
	distantGeomShadowMapFar.RenderOrtho(mountainMin, mountainMax, 1.0f);

	static bool firstRun = true;
	static unsigned int frameCounter = 0;
	if (firstRun)
	{
		// Filter all shadow maps
		for (int i = 0; i != 7; i++)
			distantGeomScreenShadow[i].Render(rtSubmission);

		// Re-render all faces and backdrop
		for (int i = 0; i != 7; i++)
			skySubmissions[i].MakeAsync().Render();
		for (int i = 0; i != 7; i++)
			submissions[i].MakeAsync().Render();
		firstRun = false;
	}
	else
	{
		if (frameCounter % 18 < 6)
			distantGeomScreenShadow[frameCounter % 18].Render(rtSubmission);
		else if (frameCounter % 18 < 12)
			skySubmissions[frameCounter % 18 - 6].MakeAsync().Render();
		else
			submissions[frameCounter % 18 - 12].MakeAsync().Render();
		distantGeomScreenShadow[6].Render(rtSubmission);
		skySubmissions[6].MakeAsync().Render();
		submissions[6].MakeAsync().Render();
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
	submission.MakeDispatch(Instance, std::string("default"), shader, voxelizedXWorkGroups, voxelizedYWorkGroups, voxelizedZWorkGroups);
}

void HIGHOMEGA::RENDER::PASSES::ClearSurfaceCacheClass::Submit()
{
	submission.MakeAsync().Submit();
}

void HIGHOMEGA::RENDER::PASSES::VisibilityPassClass::Create(unsigned int width, unsigned int height, float alphaThreshold, FrustumClass & frustum, bool swapchainDependent)
{
	unsigned int val = 0xFFFFFFFF;
	float valF = *((float *)(&val));
	submission.SetClearColor(vec3(valF), valF);
	submission.Create(Instance);
	frustumRef = &frustum;

	visibilityTriInfo.CreateOffScreenColorAttachment(Instance, R32G32UI, width, height, false, false, swapchainDependent);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, width, height, ImageClass::SAMPLE_DEPTH, swapchainDependent);
	frameBuffer.AddColorAttachment(visibilityTriInfo);
	frameBuffer.SetDepthStencil(depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	PipelineFlags defaultPipelineFlags;
	defaultPipelineFlags.backFaceCulling = true;
	defaultPipelineFlags.depthCompare = COMPARE_GREATER_OR_EQUAL;
	submission.SetDepthClear(0.0f);
	submission.SetDefaultPipelineFlags(defaultPipelineFlags);
	submission.SetFrameBuffer(frameBuffer);
	submission.matTransform = [](const MeshMaterial& inMat) ->MeshMaterial {
		if (!inMat.postProcess && inMat.isAlphaKeyed) {
			MeshMaterial retMat = inMat;
			retMat.shaderName = "alphaKey";
			return retMat;
		}
		return inMat;
	};

	shader.Create("shaders/visibility.vert.spv", "main", "shaders/visibility.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, visFrustum.Buffer);
	shaderAlphaKey.Create("shaders/visibility.vert.spv", "main", "shaders/visibilityAlphaKey.frag.spv", "main");
	shaderAlphaKey.AddResource(RESOURCE_UBO, VERTEX, 0, 0, visFrustum.Buffer);
	shaderAlphaKey.SetStageSpecializationData(FRAGMENT, { "alphaThreshold"+std::to_string(alphaThreshold), {{0u, *(unsigned int*)(&alphaThreshold)}}});
	submission.SetShader("default", shader);
	submission.SetShader("alphaKey", shaderAlphaKey);
}

void HIGHOMEGA::RENDER::PASSES::VisibilityPassClass::Render()
{
	if (prevVisFrustumEverSet)
	{
		prevVisFrustum.CopyFromFrustum(visFrustum);
		prevVisFrustum.Update();
	}
	visFrustum.CopyFromFrustum(*frustumRef);
	visFrustum.reverseZ = true;
	visFrustum.cameraRelative = true;
	visFrustum.Update();
	submission.doCulling(visFrustum, GroupedRasterSubmission::CULL_MODE::TWOPASS);
	submission.MakeAsync().Render();
	if (!prevVisFrustumEverSet)
	{
		prevVisFrustum.CopyFromFrustum(visFrustum);
		prevVisFrustum.Update();
		prevVisFrustumEverSet = true;
	}
}

void HIGHOMEGA::RENDER::PASSES::DecalPassClass::Create(GatherResolveClass& GatherResolve, FrustumClass& frustum, TriClass& PostProcessTri)
{
	mixSubmission.Add(*PostProcessTri.triModelInstance);
	mixSubmission.SetClearColor(vec3(0.0f), 0.0f);
	mixSubmission.Create(Instance);

	submission.Create(Instance);
	submission.SetClearColor(vec3(0.0f), 0.0f);
	frustumRef = &frustum;

	decalAlbedo.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, GatherResolve.worldPosAttach.getWidth(), GatherResolve.worldPosAttach.getHeight(), false, false, true);
	decalSpecular.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, GatherResolve.worldPosAttach.getWidth(), GatherResolve.worldPosAttach.getHeight(), false, false, true);
	decalRoughnessSpecularity.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, GatherResolve.worldPosAttach.getWidth(), GatherResolve.worldPosAttach.getHeight(), false, false, true);
	decalNormal.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, GatherResolve.worldPosAttach.getWidth(), GatherResolve.worldPosAttach.getHeight(), false, false, true);
	decalTangent.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, GatherResolve.worldPosAttach.getWidth(), GatherResolve.worldPosAttach.getHeight(), false, false, true);
	decalBiTangent.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, GatherResolve.worldPosAttach.getWidth(), GatherResolve.worldPosAttach.getHeight(), false, false, true);

	PipelineFlags defaultPipelineFlags;
	defaultPipelineFlags.backFaceCulling = true;
	defaultPipelineFlags.depthTest = false;
	defaultPipelineFlags.depthWrite = false;
	frameBuffer.AddColorAttachment(decalAlbedo);
	frameBuffer.AddColorAttachment(decalSpecular);
	frameBuffer.AddColorAttachment(decalRoughnessSpecularity);
	frameBuffer.AddColorAttachment(decalNormal);
	frameBuffer.AddColorAttachment(decalTangent);
	frameBuffer.AddColorAttachment(decalBiTangent);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);
	submission.SetDepthClear(0.0f);
	submission.SetDefaultPipelineFlags(defaultPipelineFlags);
	submission.SetFrameBuffer(frameBuffer);
	shader.Create("shaders/decal.vert.spv", "main", "shaders/decal.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, decalFrustum.Buffer);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherResolve.worldPosAttach);
	submission.SetShader("default", shader);


	mixFrameBuffer.setSwapchainDependentWidthCallback([&GatherResolve](unsigned int) -> unsigned int { return GatherResolve.worldPosAttach.getWidth(); });
	mixFrameBuffer.setSwapchainDependentHeightCallback([&GatherResolve](unsigned int) -> unsigned int { return GatherResolve.worldPosAttach.getHeight(); });
	mixFrameBuffer.Create(OFF_SCREEN, Instance, Window);
	mixSubmission.SetFrameBuffer(mixFrameBuffer);
	mixShader.Create("shaders/postprocess.vert.spv", "main", "shaders/decalMix.frag.spv", "main");
	mixShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	mixShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, decalAlbedo);
	mixShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, decalSpecular);
	mixShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, decalRoughnessSpecularity);
	mixShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, decalNormal);
	mixShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, decalTangent);
	mixShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, decalBiTangent);
	mixShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 7, GatherResolve.materialAttach);
	mixShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 8, GatherResolve.normalAttach);
	mixSubmission.SetShader("default", mixShader);
}

void HIGHOMEGA::RENDER::PASSES::DecalPassClass::Render()
{
	decalFrustum.CopyFromFrustum(*frustumRef);
	decalFrustum.reverseZ = true;
	decalFrustum.Update();
	submission.MakeAsync().Render();
	mixSubmission.MakeAsync().Render();
}

unsigned int HIGHOMEGA::RENDER::PASSES::GatherResolveClass::GatherResolveWorkGroupX()
{
	return 8u;
}

unsigned int HIGHOMEGA::RENDER::PASSES::GatherResolveClass::GatherResolveWorkGroupY()
{
	return 8u;
}

void HIGHOMEGA::RENDER::PASSES::GatherResolveClass::Create(VisibilityPassClass& VisibilityPass, MoBlurClass *MoBlur, bool simple)
{
	moBlurRef = MoBlur;
	if (!simple) materialAttach.CreateImageStore(Instance, R32G32B32A32F, VisibilityPass.visibilityTriInfo.getWidth(), VisibilityPass.visibilityTriInfo.getHeight(), 1, _2D, false,
		(VisibilityPass.visibilityTriInfo.getSurfaceBasedWidthCallback() || VisibilityPass.visibilityTriInfo.getSurfaceBasedHeightCallback()));
	worldPosAttach.CreateImageStore(Instance, R32G32B32A32F, VisibilityPass.visibilityTriInfo.getWidth(), VisibilityPass.visibilityTriInfo.getHeight(), 1, _2D, false,
		(VisibilityPass.visibilityTriInfo.getSurfaceBasedWidthCallback() || VisibilityPass.visibilityTriInfo.getSurfaceBasedHeightCallback()));
	normalAttach.CreateImageStore(Instance, R32G32B32A32F, VisibilityPass.visibilityTriInfo.getWidth(), VisibilityPass.visibilityTriInfo.getHeight(), 1, _2D, false,
		(VisibilityPass.visibilityTriInfo.getSurfaceBasedWidthCallback() || VisibilityPass.visibilityTriInfo.getSurfaceBasedHeightCallback()));
	visPassRef = &VisibilityPass;

	this->simple = simple;
}

void HIGHOMEGA::RENDER::PASSES::GatherResolveClass::Render()
{
	if (sceneId != GroupedRenderSubmission::SceneData->descriptorId)
	{
		sceneId = GroupedRenderSubmission::SceneData->descriptorId;
		madeDispatch = false;
	}
	if (!madeDispatch)
	{
		shader.ClearResources();
		shader.Create(simple ? "shaders/gatherresolvesimple.comp.spv" : "shaders/gatherresolve.comp.spv", "main");
		shader.AddResource(RESOURCE_UBO, COMPUTE, 0, 0, visPassRef->visFrustum.Buffer);
		shader.AddResource(RESOURCE_UBO, COMPUTE, 0, 1, visPassRef->prevVisFrustum.Buffer);
		shader.AddResource(RESOURCE_SAMPLER, COMPUTE, 0, 2, visPassRef->visibilityTriInfo);
		if (!simple) shader.AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 3, materialAttach);
		shader.AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 4, worldPosAttach, -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		shader.AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 5, normalAttach, -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		shader.AddResource(RESOURCE_IMAGE_STORE, COMPUTE, 0, 6, moBlurRef->velocityAttach, -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
		shader.AddResource(RESOURCE_SAMPLER, COMPUTE, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
		shader.AddResource(RESOURCE_SSBO, COMPUTE, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
		giantVertBufferSharedMutex.lock_shared();
		shader.AddResource(RESOURCE_SSBO, COMPUTE, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
		giantVertBufferSharedMutex.unlock_shared();
		shader.AddResource(RESOURCE_SSBO, COMPUTE, 2, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
		submission.MakeDispatch(Instance, simple ? std::string("gatherresolve_simple") : std::string("gatherresolve"), shader, (unsigned int)ceil((double)visPassRef->visibilityTriInfo.getWidth() / (double)GatherResolveWorkGroupX()), (unsigned int)ceil((double)visPassRef->visibilityTriInfo.getHeight() / (double)GatherResolveWorkGroupY()), 1);
		visBufResX = visPassRef->visibilityTriInfo.getWidth();
		visBufResY = visPassRef->visibilityTriInfo.getHeight();
		madeDispatch = true;
	}
	if (madeDispatch && (visPassRef->visibilityTriInfo.getWidth() != visBufResX || visPassRef->visibilityTriInfo.getHeight() != visBufResY))
	{
		submission.UpdateDispatchSize(Instance, simple ? std::string("gatherresolve_simple") : std::string("gatherresolve"), (unsigned int)ceil((double)visPassRef->visibilityTriInfo.getWidth() / (double)GatherResolveWorkGroupX()), (unsigned int)ceil((double)visPassRef->visibilityTriInfo.getHeight() / (double)GatherResolveWorkGroupY()), 1);
		visBufResX = visPassRef->visibilityTriInfo.getWidth();
		visBufResY = visPassRef->visibilityTriInfo.getHeight();
	}
	submission.MakeAsync().Submit();
}

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, SkyDomeClass & SkyDome, ShadowMapClass & ShadowMapNear, ShadowMapClass & ShadowMapFar, GroupedTraceSubmission & rtSubmission, GroupedSDFBVHSubmission *sdfBVHSubmission, const vec3 & initialViewer)
{
	postProcessTriRef = &PostProcessTri;
	sdfBVHSubmissionRef = sdfBVHSubmission;
	GatherPassRef = &GatherPass;
	TraceRef = &rtSubmission;
	shadowMapNearRef = &ShadowMapNear;
	shadowMapFarRef = &ShadowMapFar;
	SkyDomeRef = &SkyDome;

	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);
	glossSubmission.Add(*PostProcessTri.triModelInstance);
	glossSubmission.Create(Instance);

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
	PathTraceParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &PathTraceParams, (unsigned int)sizeof(PathTraceParams));
	influence.factor = 1.0f;
	influenceBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &influence, (unsigned int)sizeof(influence));

	for (int i = 0; i != HIGHOMEGA_MAXIMUM_IRRADIANCE_CACHE_CASCADES; i++)
		for (int j = 0; j != 6; j++)
		{
			radiosityMaps[i][j].CreateImageStore(Instance, R16G16B16A16F, HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, HIGHOMEGA_IRRADIANCE_CACHE_SIDE_SIZE, _3D, true);
			radiosityMapsVec.push_back(&radiosityMaps[i][j]);
		}
	radiosityMapsVec[0]->ClearColors(radiosityMapsVec, ImageClearColor(vec3(0.0f), 0.0f));
}

void HIGHOMEGA::RENDER::PASSES::PathTraceClass::Render(const vec3& currentViewer)
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

	if (RTInstance::Enabled())
	{
		if (!builtHWRTSubmission)
		{
			rtShaderResourceSet.CreateRT("shaders/rtdiffusetrace.rgen.spv", "main", "shaders/rtchit.rchit.spv", "main", "shaders/rtmiss.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
			rtShaderResourceSet.SetStageSpecializationData(RT_RAYGEN, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			tracelet.Make(Instance);
			rtShaderResourceSetGloss.CreateRT("shaders/rtglosstrace.rgen.spv", "main", "shaders/rtchit.rchit.spv", "main", "shaders/rtmiss.rmiss.spv", "main", "shaders/rtalphakey.rahit.spv", "main");
			rtShaderResourceSetGloss.SetStageSpecializationData(RT_RAYGEN, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			traceletGloss.Make(Instance);
			builtHWRTSubmission = true;
		}
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
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, GatherPassRef->materialAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 2, GatherPassRef->worldPosAttach);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 3, GatherPassRef->normalAttach);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 4, MainFrustum.Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 5, PathTraceParamsBuf);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 6, radiosityMapResources);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 7, SkyDomeRef->fullCubeMap);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 8, shadowMapNearRef->getDSAttach());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 9, shadowMapNearRef->getColorAttach());
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 10, shadowMapNearRef->getFrustum().Buffer);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 11, shadowMapFarRef->getDSAttach());
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 12, shadowMapFarRef->getColorAttach());
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 13, shadowMapFarRef->getFrustum().Buffer);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 14, SkyDomeRef->rayleighMieBuf);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.lock_shared();
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);

			tracingResourcesGloss.clear();
			tracingResourcesGloss.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN, 0, 0, rtSceneRef);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, GatherPassRef->materialAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 2, GatherPassRef->worldPosAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 3, GatherPassRef->normalAttach);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 4, glossTraceOutput);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 5, MainFrustum.Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 6, PathTraceParamsBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 7, radiosityMapResources);
			tracingResourcesGloss.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 8, radiosityMapResources);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 9, SkyDomeRef->fullCubeMap);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 10, shadowMapNearRef->getDSAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 11, shadowMapNearRef->getColorAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 12, shadowMapNearRef->getFrustum().Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 13, shadowMapFarRef->getDSAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 14, shadowMapFarRef->getColorAttach());
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 15, shadowMapFarRef->getFrustum().Buffer);
			tracingResourcesGloss.emplace_back(RESOURCE_UBO, RT_RAYGEN, 0, 16, SkyDomeRef->rayleighMieBuf);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 17, *blueNoise);
			tracingResourcesGloss.emplace_back(RESOURCE_SAMPLER, RT_RCHIT | RT_ANYHIT, 1, GroupedRenderSubmission::SceneData->uniqueSamplersArray, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 0, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.lock_shared();
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 1, *giantVertBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
			giantVertBufferSharedMutex.unlock_shared();
			tracingResourcesGloss.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 2, 2, *GroupedRenderSubmission::SceneData->transformBuffer, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_NOT_A_DEPENDENCY);
		}
		tracelet.MakeAsync().Submit(glossTraceOutput.getWidth(), glossTraceOutput.getHeight(), 1, tracingResources, rewriteDescriptoSets, rtShaderResourceSet);
		traceletGloss.MakeAsync().Submit(glossTraceOutput.getWidth(), glossTraceOutput.getHeight(), 1, tracingResourcesGloss, rewriteDescriptoSets, rtShaderResourceSetGloss);
	}
	else
	{
		if (!builtSoftRTSubmission)
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
			shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, postProcessTriRef->triFrustum.Buffer);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPassRef->materialAttach);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPassRef->worldPosAttach);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherPassRef->normalAttach);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 4, MainFrustum.Buffer);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, PathTraceParamsBuf);
			shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 6, radiosityMapResources);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, SkyDomeRef->fullCubeMap);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, shadowMapNearRef->getDSAttach());
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, shadowMapNearRef->getColorAttach());
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 10, shadowMapNearRef->getFrustum().Buffer);
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, shadowMapFarRef->getDSAttach());
			shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 12, shadowMapFarRef->getColorAttach());
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 13, shadowMapFarRef->getFrustum().Buffer);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 14, SkyDomeRef->rayleighMieBuf);
			shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 15, influenceBuf);
			shader.SetStageSpecializationData(FRAGMENT, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			submission.SetShader("default", shader);
			submission.requestSDFBVH(*sdfBVHSubmissionRef);

			glossTraceShader.Create("shaders/postprocess.vert.spv", "main", "shaders/SDFBVHglosstrace.frag.spv", "main");
			glossTraceShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, postProcessTriRef->triFrustum.Buffer);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, GatherPassRef->materialAttach);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPassRef->worldPosAttach);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherPassRef->normalAttach);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 4, glossTraceOutput);
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, MainFrustum.Buffer);
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 6, PathTraceParamsBuf);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, radiosityMapResources);
			glossTraceShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 8, radiosityMapResources);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 9, SkyDomeRef->fullCubeMap);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 10, shadowMapNearRef->getDSAttach());
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, shadowMapNearRef->getColorAttach());
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 12, shadowMapNearRef->getFrustum().Buffer);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 13, shadowMapFarRef->getDSAttach());
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 14, shadowMapFarRef->getColorAttach());
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 15, shadowMapFarRef->getFrustum().Buffer);
			glossTraceShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 16, SkyDomeRef->rayleighMieBuf);
			glossTraceShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 17, *blueNoise);
			glossTraceShader.SetStageSpecializationData(FRAGMENT, { "ShadowBiases", {
				{0u, *(unsigned int*)(&shadowMapNearRef->samplingBias.x)},
				{1u, *(unsigned int*)(&shadowMapNearRef->samplingBias.y)},
				{2u, *(unsigned int*)(&shadowMapFarRef->samplingBias.x)},
				{3u, *(unsigned int*)(&shadowMapFarRef->samplingBias.y)}
			} });
			glossSubmission.SetShader("default", glossTraceShader);
			glossSubmission.requestSDFBVH(*sdfBVHSubmissionRef);

			builtSoftRTSubmission = true;
		}
		// Between this, glossSubmission and ClearSurfaceCache we're conducting concurrent non-atomic accesses to radiosityMapResources.
		// This ends up creating a scenario where a full CPU sync works better than barriers and GPU syncs combined and creates quality parity
		// between radiosityMapResources in HWRT and SDF-BVH. The exact mechanism why is unknown. The small hit to GPU occupancy is worth it.
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
		glossSubmission.MakeAsync().Render();
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

	Copyright © 2026 TooMuchVoltage Software Inc. This notice shall always be coupled with any SauRay(TM) implementation and must be redistributed alongside it.

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
	playerFrusta[playerId].lookUpLook2Up2[0] = toZSignXY(look);
	playerFrusta[playerId].lookUpLook2Up2[1] = toZSignXY(up);
	playerFrusta[playerId].lookUpLook2Up2[2] = toZSignXY(look2);
	playerFrusta[playerId].lookUpLook2Up2[3] = toZSignXY(up2);
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

	frustaBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, (void *)playerFrusta.data(), (unsigned int)(playerFrusta.size() * sizeof(playerFrustum)));
	limitsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, (void *)playerLimits.data(), (unsigned int)(playerLimits.size() * sizeof(playerLimit)));
	visibilityMatrixBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, (void *)playerVisMatrix.data(), (unsigned int)(playerVisMatrix.size() * sizeof(playerVisData)));
	timeBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, (void *)&timeInfo, (unsigned int)sizeof(timeInfo));
	sunDirBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, (void *)&sun, (unsigned int)sizeof(sun));

	testOutput.CreateImageStore(Instance, R8G8B8A8UN, resSide, resSide, 1, _2D, false);

	if (RTInstance::Enabled())
	{
		if (debugMode)
			rtShaderResourceSet.CreateRT("shaders/rtsauraytrace.rgen.spv", "main", "shaders/rtsauraytrace.rchit.spv", "main", "shaders/rtsauraytrace.rmiss.spv", "main", "shaders/rtsauraytrace.rahit.spv", "main");
		else
			rtShaderResourceSet.CreateRT("shaders/rtsauraytrace_release.rgen.spv", "main", "shaders/rtsauraytrace.rchit.spv", "main", "shaders/rtsauraytrace.rmiss.spv", "main", "shaders/rtsauraytrace.rahit.spv", "main");
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
		if (sunDirChanged)
		{
			sunDirBuf.UploadSubData(0, (void *)&sun, (unsigned int)sizeof(sun));
			sunDirChanged = false;
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
			tracingResources.emplace_back(RESOURCE_RT_ACCEL_STRUCT, RT_RAYGEN | RT_RCHIT | RT_MISS, 0, 0, rtSceneRef);
			tracingResources.emplace_back(RESOURCE_IMAGE_STORE, RT_RAYGEN, 0, 1, testOutput, -1, ShaderResource::SHADER_RESOURCE_USAGE::USAGE_PRODUCER);
			tracingResources.emplace_back(RESOURCE_SAMPLER, RT_RAYGEN, 0, 2, *blueNoise);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RCHIT | RT_ANYHIT, 0, 3, *GroupedRenderSubmission::SceneData->instancePropertiesBuffer);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RAYGEN | RT_RCHIT | RT_MISS, 0, 4, frustaBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RAYGEN | RT_ANYHIT, 0, 5, visibilityMatrixBuf);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RAYGEN | RT_ANYHIT, 0, 6, timeBuf);
			tracingResources.emplace_back(RESOURCE_SSBO, RT_RAYGEN, 0, 7, limitsBuf);
			tracingResources.emplace_back(RESOURCE_UBO, RT_RCHIT, 0, 8, sunDirBuf);
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

	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);

	MVPsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &MVPs, (unsigned int)sizeof(MVPs));

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
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherResolve.materialAttach);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherResolve.worldPosAttach);
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

	submission.MakeAsync().Render();

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[1] = (float)((curTurn + 1) % HIGHOMEGA_TEMPORAL_TRAIL_AMOUNT);

	lastEye = MainFrustum.eye;
	lastLook = MainFrustum.look;
}

void HIGHOMEGA::RENDER::PASSES::SpatialDenoiseClass::Create(TriClass & PostProcessTri, PathTraceClass & PathTrace, TemporalAccumulateClass & TemporalAccumulate, GatherResolveClass &GatherResolve)
{
	pathTraceRef = &PathTrace;

	blurHAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), false, false);
	blurVAttach.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, PathTrace.glossTraceOutput.getWidth(), PathTrace.glossTraceOutput.getHeight(), true, false);

	submissionH.Add(*PostProcessTri.triModelInstance);
	submissionH.Create(Instance);
	submissionV.Add(*PostProcessTri.triModelInstance);
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

	float invSizeX = 1.0f / ((float)TemporalAccumulate.glossTemporalAccumulateResultAttach.getWidth());
	float invSizeY = 1.0f / ((float)TemporalAccumulate.glossTemporalAccumulateResultAttach.getHeight());
	unsigned int invSizeXUint = *((unsigned int*)(&invSizeX));
	unsigned int invSizeYUint = *((unsigned int*)(&invSizeY));

	shaderH.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderH.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, TemporalAccumulate.glossTemporalAccumulateResultAttach);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherResolve.materialAttach);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherResolve.worldPosAttach);
	shaderH.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 4, GatherResolve.normalAttach);
	shaderH.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, pathTraceRef->PathTraceParamsBuf);
	shaderH.SetStageSpecializationData(FRAGMENT, { "HorizPass", {{0u, invSizeXUint}, {1u, 0u}} });

	shaderV.AddResource(RESOURCE_UBO, VERTEX | FRAGMENT, 0, 0, PostProcessTri.triFrustum.Buffer);
	shaderV.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, blurHAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherResolve.materialAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherResolve.worldPosAttach);
	shaderV.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 4, GatherResolve.normalAttach);
	shaderV.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, pathTraceRef->PathTraceParamsBuf);
	shaderV.SetStageSpecializationData(FRAGMENT, { "VertiPass", {{0u, 0u}, {1u, invSizeYUint}} });

	submissionH.SetShader("default", shaderH);
	submissionV.SetShader("default", shaderV);
}

void HIGHOMEGA::RENDER::PASSES::SpatialDenoiseClass::Render()
{
	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[3] = GetStateOfAction(CMD_SWITCH_TO_PT_MODE) ? 1.0f : 0.0f;

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[0] += 0.01f;

	pathTraceRef->PathTraceParams.timeTurnBlurDirectionRawLight[2] = 0.0f;
	pathTraceRef->PathTraceParamsBuf.UploadSubData(0, &pathTraceRef->PathTraceParams, sizeof(pathTraceRef->PathTraceParams));

	submissionH.MakeAsync().Render();
	submissionV.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::SimpleGaussian::Create(TriClass & PostProcessTri, BlurInputHolder & inputHolder, unsigned int outputWidth, unsigned int outputHeight, unsigned int blurSize, bool displayVOnScreen)
{
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

	blurHSubmission.Add(*PostProcessTri.triModelInstance);
	blurHSubmission.Create(Instance);
	blurVSubmission.Add(*PostProcessTri.triModelInstance);
	blurVSubmission.Create(Instance);

	blurHSubmission.SetFrameBuffer(frameBufferH);

	blurHSubmission.SetDefaultPipelineFlags(defPipelineFlags);
	blurVSubmission.SetDefaultPipelineFlags(defPipelineFlags);

	float invSizeX = 1.0f / (float)(outputWidth);
	float invSizeY = 1.0f / (float)(outputHeight);
	unsigned int invSizeXUint = *((unsigned int*)(&invSizeX));
	unsigned int invSizeYUint = *((unsigned int*)(&invSizeY));

	blurHShader.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSepSimple.frag.spv", "main");
	blurHShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	blurHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, inputHolder.blurInput);
	blurHShader.SetStageSpecializationData(FRAGMENT, { "HorizFogPass", {{0u, invSizeXUint}, {1u, 0u}, {2u, 4u}} });
	blurHSubmission.SetShader("default", blurHShader);

	blurVShader.Create("shaders/postprocess.vert.spv", "main", "shaders/gaussianSepSimple.frag.spv", "main");
	blurVShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	blurVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, blurHAttach);
	blurVShader.SetStageSpecializationData(FRAGMENT, { "VertiFogPass", {{0u, 0u}, {1u, invSizeYUint}, {2u, 4u}} });
	blurVSubmission.SetShader("default", blurVShader);
}

void HIGHOMEGA::RENDER::PASSES::SimpleGaussian::Render()
{
	blurHSubmission.MakeAsync().Render();
	blurVSubmission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::ModulateClass::Create(TriClass & PostProcessTri, MoBlurClass & MoBlur, VisibilityPassClass & VisibilityPass, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, SpatialDenoiseClass & SpatialDenoise, SkyDomeClass & SkyDome, ShadowMapScreenClass & shadowMapScreen)
{
	modulatedOutput.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, ScreenSize.width, ScreenSize.height, false, true, true);
	frameBuffer.AddColorAttachment(modulatedOutput);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.Add(*PostProcessTri.triModelInstance);
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
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPass.materialAttach);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 3, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 4, GatherPass.normalAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, radiosityMaps);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 6, PathTrace.PathTraceParamsBuf);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, SkyDome.fullBackDrop);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, SkyDome.fullCubeMap);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, shadowMapScreen.shadowMapScreen);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 10, SkyDome.rayleighMieBuf);
	submission.SetShader("default", shader);

	particleShader.Create("shaders/particle.vert.spv", "main", "shaders/particleAdditiveBlend.frag.spv", "main");
	particleShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, VisibilityPass.visFrustum.Buffer);
	particleShader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, VisibilityPass.prevVisFrustum.Buffer);
	particleShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, MoBlur.velocityAttach);
	particleShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, VisibilityPass.depthStencilAttach);
	particleShaderAlphaBlend.Create("shaders/particle.vert.spv", "main", "shaders/particleAlphaBlend.frag.spv", "main");
	particleShaderAlphaBlend.AddResource(RESOURCE_UBO, VERTEX, 0, 0, VisibilityPass.visFrustum.Buffer);
	particleShaderAlphaBlend.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, VisibilityPass.prevVisFrustum.Buffer);
	particleShaderAlphaBlend.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, MoBlur.velocityAttach);
	particleShaderAlphaBlend.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, VisibilityPass.depthStencilAttach);
	submission.SetShader("shaderParticleAdditiveBlend", particleShader);
	submission.SetShader("shaderParticleAlphaBlend", particleShaderAlphaBlend);
}

void HIGHOMEGA::RENDER::PASSES::ModulateClass::Render()
{
	submission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceGatherClass::Create(TriClass & PostProcessTri, MoBlurClass& MoBlur, VisibilityPassClass & VisibilityPass, GatherResolveClass & GatherPass, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams)
{
	ssGatherPosAlbedo.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false, true);
	ssNormInstIDVelocityRoughness.CreateOffScreenColorAttachment(Instance, R32G32B32A32F, ScreenSize.width, ScreenSize.height, false, false, true);
	depthStencilAttach.CreateOffScreenDepthStencil(Instance, ScreenSize.width, ScreenSize.height, ImageClass::SAMPLE_DEPTH, true);

	frameBuffer.AddColorAttachment(ssGatherPosAlbedo);
	frameBuffer.AddColorAttachment(ssNormInstIDVelocityRoughness);
	frameBuffer.SetDepthStencil(depthStencilAttach);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.Create(Instance);

	PipelineFlags defaultPipelineFlags;
	defaultPipelineFlags.depthCompare = COMPARE_GREATER_OR_EQUAL;
	submission.SetDepthClear(0.0f);
	submission.SetDefaultPipelineFlags(defaultPipelineFlags);
	submission.SetFrameBuffer(frameBuffer);

	shaderScreenSpace.Create("shaders/ssgather.vert.spv", "main", "shaders/ssgather.frag.spv", "main");
	shaderScreenSpace.AddResource(RESOURCE_UBO, VERTEX, 0, 0, VisibilityPass.visFrustum.Buffer);
	shaderScreenSpace.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, VisibilityPass.depthStencilAttach);
	shaderTessScreenSpace.Create("shaders/ssgather.vert.spv", "main", "shaders/ssgather.tesc.spv", "main", "shaders/ssgather.tese.spv", "main", "shaders/ssgather.frag.spv", "main");
	shaderTessScreenSpace.AddResource(RESOURCE_UBO, VERTEX | TESS_EVAL, 0, 0, VisibilityPass.visFrustum.Buffer);
	shaderTessScreenSpace.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, VisibilityPass.depthStencilAttach);
	shaderTessScreenSpace.AddResource(RESOURCE_UBO, TESS_EVAL, 0, 2, WorldParams.GetRenderTimeBuffer());
	submission.SetShader("shaderScreenSpace", shaderScreenSpace);
	submission.SetShader("shaderTessScreenSpace", shaderTessScreenSpace);
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceGatherClass::Render()
{
	submission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::NearScatteringClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, ShadowMapClass & ShadowMapNear, ShadowMapClass & ShadowMapFar, SkyDomeClass & SkyDome, WorldParamsClass & WorldParams, ScreenSpaceGatherClass & ScreenSpaceGather, unsigned int outputSize)
{
	worldParamsRef = &WorldParams;
	if (blueNoise->getWidth() == 0) blueNoise->CreateTexture(Instance, "assets/common/", "bluenoise.tga", 1, false, false, false, false);

	blurInput.CreateOffScreenColorAttachment(Instance, R16G16B16A16F, outputSize, outputSize, true, true);

	nearScatteringParams.amount = worldParamsRef->GetLightShaftAmount();
	nearScatteringParams.extinction = worldParamsRef->GetLightShaftExtinction();
	nearScatteringParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &nearScatteringParams, (unsigned int)sizeof(nearScatteringParams));

	frameBuffer.AddColorAttachment(blurInput);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);

	submission.Add(*PostProcessTri.triModelInstance);
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
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ShadowMapNear.getDSAttach());
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, ShadowMapNear.getColorAttach());
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 5, ShadowMapNear.getFrustum().Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, ShadowMapFar.getDSAttach());
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, ShadowMapFar.getColorAttach());
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 8, ShadowMapFar.getFrustum().Buffer);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 10, ScreenSpaceGather.ssGatherPosAlbedo);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, SkyDome.fullCubeMap);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 12, *blueNoise);
	submission.SetShader("default", shader);

	blurPass.Create(PostProcessTri, *this, outputSize, outputSize, 4);

	std::vector<ImageClass *> blurVImageVector;
	blurVImageVector.push_back(&blurPass.blurVAttach);
	blurPass.blurVAttach.ClearColors(blurVImageVector, ImageClearColor(vec3(0.0f), 1.0f));
}

void HIGHOMEGA::RENDER::PASSES::NearScatteringClass::Render()
{
	if (worldParamsRef->GetLightShaftAmount() > 0.0f)
	{
		nearScatteringParams.amount = worldParamsRef->GetLightShaftAmount();
		nearScatteringParams.extinction = worldParamsRef->GetLightShaftExtinction();
		nearScatteringParamsBuf.UploadSubData(0, &nearScatteringParams, sizeof(nearScatteringParams));
		submission.MakeAsync().Render();
		blurPass.Render();
	}
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceFXClass::Create(TriClass & PostProcessTri, SkyDomeClass & SkyDome, VisibilityPassClass & VisibilityPass, GatherResolveClass & GatherPass, PathTraceClass & PathTrace, ModulateClass & Modulate, ScreenSpaceGatherClass & ScreenSpaceGather, MoBlurClass & MoBlur, NearScatteringClass & NearScattering)
{
	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);

	ssfxOut.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, ScreenSize.width, ScreenSize.height, false, true, true);

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
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ScreenSpaceGather.ssGatherPosAlbedo);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, ScreenSpaceGather.ssNormInstIDVelocityRoughness);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, SkyDome.fullCubeMap);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 6, SkyDome.fullBackDrop);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 7, NearScattering.blurPass.blurVAttach);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 8, VisibilityPass.depthStencilAttach);
	VisibilityPass.submission.AllocateMipChainImages(); // Pre-populate these objects cause we need the references right now...
	std::vector<ShaderResource> MinZChainRes, MaxZChainRes;
	for (int i = 0; i != 7; i++)
		MinZChainRes.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, VisibilityPass.submission.MinZChain[i]);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 9, MinZChainRes);
	for (int i = 0; i != 7; i++)
		MaxZChainRes.emplace_back(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 10, VisibilityPass.submission.MaxZChain[i]);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 10, MaxZChainRes);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 11, ScreenSpaceGather.depthStencilAttach);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 12, MoBlur.velocityAttach);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 13, VisibilityPass.visFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 14, VisibilityPass.prevVisFrustum.Buffer);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::ScreenSpaceFXClass::Render()
{
	submission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::MoBlurClass::CreateVelocityBuffer()
{
	velocityAttach.CreateImageStore(Instance, R32UI, ScreenSize.width, ScreenSize.height, 1, _2D, false, true);
}

void HIGHOMEGA::RENDER::PASSES::MoBlurClass::PrepareForFrame()
{
	std::vector<ImageClass*> clearVel = { &velocityAttach };
	velocityAttach.ClearColors(clearVel, ImageClearColor(0x7F7Fu, 0u, 0u, 0u));
}

void HIGHOMEGA::RENDER::PASSES::MoBlurClass::Create(TriClass& PostProcessTri, ScreenSpaceFXClass& ScreenSpaceFX, ScreenSpaceGatherClass& ScreenSpaceGather, VisibilityPassClass& VisibilityPass, GatherResolveClass& GatherPass)
{
	dilationSubmission.Add(*PostProcessTri.triModelInstance);
	dilationSubmission.Create(Instance);
	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);

	dilationFrameBuffer.setSwapchainDependentWidthCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getWidth(); });
	dilationFrameBuffer.setSwapchainDependentHeightCallback([&GatherPass](unsigned int) -> unsigned int { return GatherPass.worldPosAttach.getHeight(); });
	dilationFrameBuffer.Create(OFF_SCREEN, Instance, Window);
	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;
	dilationSubmission.SetFrameBuffer(dilationFrameBuffer);
	dilationSubmission.SetDefaultPipelineFlags(defPipelineFlags);

	moBlurOut.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, ScreenSize.width, ScreenSize.height, false, true, true);

	frameBuffer.AddColorAttachment(moBlurOut);
	frameBuffer.Create(OFF_SCREEN, Instance, Window);
	submission.SetFrameBuffer(frameBuffer);
	submission.SetDefaultPipelineFlags(defPipelineFlags);

	dilationShader.Create("shaders/postprocess.vert.spv", "main", "shaders/dilation.frag.spv", "main");
	dilationShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	dilationShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 1, velocityAttach);
	dilationShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 2, VisibilityPass.visibilityTriInfo);
	dilationShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, ScreenSpaceGather.ssNormInstIDVelocityRoughness);
	dilationShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 4, VisibilityPass.depthStencilAttach);
	dilationShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 5, ScreenSpaceGather.depthStencilAttach);
	dilationSubmission.SetShader("default", dilationShader);

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/moblur.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, ScreenSpaceFX.ssfxOut);
	shader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, velocityAttach);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::MoBlurClass::Render()
{
	dilationSubmission.MakeAsync().Render();
	submission.MakeAsync().Render();
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::Create(TriClass & PostProcessTri, GatherResolveClass & GatherPass, MoBlurClass& MoBlur)
{
	MeshMaterial textMat("fontmap.tga", "assets/common/", Instance, true);
	textMat.shaderName = "midScrText";
	textMat.pipelineFlags.blendEnable = true;
	textMat.pipelineFlags.alphaBlending = true;
	textMat.pipelineFlags.colorBlending = true;
	textMat.pipelineFlags.srcAlphaFactor = FACTOR_ONE;
	textMat.pipelineFlags.dstAlphaFactor = FACTOR_ONE;
	textMat.pipelineFlags.srcColorFactor = FACTOR_SRC_ALPHA;
	textMat.pipelineFlags.dstColorFactor = FACTOR_ONE_MINUS_SRC_ALPHA;
	textMat.pipelineFlags.alphaBlendOp = BLEND_ADD;
	textMat.pipelineFlags.colorBlendOp = BLEND_ADD;
	textMat.pipelineFlags.changedBlendEnable = true;
	textMat.isAlphaBlending = true;
	textMat.renderOrder = 1;

	std::vector<unsigned char> textIdxVerts;
	vec3 extents = HIGHOMEGA::RENDER::TEXT::MakeGeomFromText(" ", vec2(0.02f, 0.06f), vec3(0.0f), textIdxVerts);
	std::string midScreenGroupId = "midScreenTextGroup";
	midScreenText.Model(midScreenGroupId, textMat, textIdxVerts);
	mat4 transMat;
	transMat.Ident();
	transMat.i[0][3] = extents.x * -0.5f;
	transMat.i[1][3] = extents.y * 0.5f;
	midScreenTextInst = midScreenText.CreateInstance(&transMat);

	/*textMat.shaderName = "text";
	textMat.renderOrder = 2;

	std::string topScreenGroupId = "topScreenTextGroup";
	textIdxVerts.clear();
	extents = HIGHOMEGA::RENDER::TEXT::MakeGeomFromText(" ", vec2(0.02f, 0.06f), vec3(0.0f), textIdxVerts);
	topScreenText.Model(topScreenGroupId, textMat, textIdxVerts);
	transMat.Ident();
	transMat.i[0][3] = -1.0f;
	transMat.i[1][3] = 1.0f;
	topScreenTextInst = topScreenText.CreateInstance(&transMat);*/

	dofParams.invDims[0] = 1.0f / (float)(ScreenSize.width);
	dofParams.invDims[1] = 1.0f / (float)(ScreenSize.height);
	for (int i = 0; i != 4; i++) {
		dofParams.screenMidPos[i] = 0.0f;
		dofParams.toScreenMidPosAlpha[i] = 0.0f;
	}
	dofParams.midScreenAlpha = 0.5f;
	midScreenAlphaTarget = 0.5f;
	dofParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_SSBO, Instance, &dofParams, (unsigned int)sizeof(dofParams));

	dofHAttach.CreateOffScreenColorAttachment(Instance, R8G8B8A8UN, ScreenSize.width, ScreenSize.height, false, true, true);

	frameBufferH.AddColorAttachment(dofHAttach);
	frameBufferH.Create(OFF_SCREEN, Instance, Window);

	dofHSubmission.Add(*PostProcessTri.triModelInstance);
	dofHSubmission.Create(Instance);
	dofVSubmission.Add(*PostProcessTri.triModelInstance);
	dofVSubmission.Add(*midScreenTextInst);
	//dofVSubmission.Add(*topScreenTextInst);
	dofVSubmission.Create(Instance);

	dofHSubmission.SetFrameBuffer(frameBufferH);
	dofVSubmission.SetFrameBuffer(Instance.swapChainFrameBuffer());
	PipelineFlags defPipelineFlags;
	defPipelineFlags.depthTest = false;
	defPipelineFlags.depthWrite = false;

	dofHShader.Create("shaders/postprocess.vert.spv", "main", "shaders/dof.frag.spv", "main");
	dofHShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	dofHShader.AddResource(RESOURCE_SSBO, FRAGMENT, 0, 1, dofParamsBuf);
	dofHShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	dofHShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, MoBlur.moBlurOut);
	dofHSubmission.SetDefaultPipelineFlags(defPipelineFlags);
	dofHSubmission.SetShader("default", dofHShader);

	dofVShader.Create("shaders/postprocess.vert.spv", "main", "shaders/dof.frag.spv", "main");
	dofVShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	dofVShader.AddResource(RESOURCE_SSBO, FRAGMENT, 0, 1, dofParamsBuf);
	dofVShader.AddResource(RESOURCE_IMAGE_STORE, FRAGMENT, 0, 2, GatherPass.worldPosAttach);
	dofVShader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 3, dofHAttach);
	textShader.Create("shaders/postprocessTrans.vert.spv", "main", "shaders/text.frag.spv", "main");
	textShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	textShader.AddResource(RESOURCE_SSBO, FRAGMENT, 0, 1, dofParamsBuf);
	midScrTextShader.Create("shaders/postprocessTrans.vert.spv", "main", "shaders/text.frag.spv", "main");
	midScrTextShader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	midScrTextShader.AddResource(RESOURCE_SSBO, FRAGMENT, 0, 1, dofParamsBuf);
	midScrTextShader.SetStageSpecializationData(FRAGMENT, { "TextMode_MidScreen", {
		{0u, 1},
	} });
	dofVSubmission.SetDefaultPipelineFlags(defPipelineFlags);
	dofVSubmission.SetShader("default", dofVShader);
	dofVSubmission.SetShader("text", textShader);
	dofVSubmission.SetShader("midScrText", midScrTextShader);
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::Render(float inpAlpha)
{
	SetMidScreenMessage();

	/*if (topScreenTextInst)
	{
		std::vector<unsigned char> textIdxVerts;
		std::string allTopText = "Total: " + frameInstrument.ResultsMs(52) + std::string("\n");
		allTopText += "FPS: " + std::to_string(HIGHOMEGA::INSTRUMENTATION::FPSCounter::Report()) + std::string("\n");
		allTopText += "Update: " + updateInstrument.ResultsMs(51);
		HIGHOMEGA::RENDER::TEXT::MakeGeomFromText(allTopText, vec2(0.02f, 0.06f), vec3(0.0f), textIdxVerts);
		std::string topScreenGroupId = "topScreenTextGroup";
		topScreenTextInst->modelRef->ChangeGeom(topScreenGroupId, textIdxVerts);
	}*/

	dofParams.blurDirection = 0.0f;
	dofParams.invCoCDist = 1.0f / 10000000.0f;
	dofParams.invDims[0] = 1.0f / (float)(ScreenSize.width);
	dofParams.invDims[1] = 1.0f / (float)(ScreenSize.height);
	dofParamsBuf.UploadSubData(0, &dofParams, sizeof(dofParams));

	dofHSubmission.MakeAsync().Render();

	dofParams.blurDirection = 1.0f;
	dofParams.toScreenMidPosAlpha[3] = inpAlpha;
	dofParamsBuf.UploadSubData(0, &dofParams, sizeof(dofParams));

	if (!HIGHOMEGA::EVENTS::windowMinimized) dofVSubmission.MakeAsync().Render();

	dofParamsBuf.DownloadSubData(0, &dofParamsDownloaded, sizeof(dofParamsDownloaded));
	vec3 screenMidPos = vec3(dofParamsDownloaded.screenMidPos[0], dofParamsDownloaded.screenMidPos[1], dofParamsDownloaded.screenMidPos[2]);
	vec3 toScreenMidPos = vec3(dofParamsDownloaded.toScreenMidPosAlpha[0], dofParamsDownloaded.toScreenMidPosAlpha[1], dofParamsDownloaded.toScreenMidPosAlpha[2]);

	if ((toScreenMidPos - screenMidPos).length() > 1000.0f)
		toScreenMidPos += (screenMidPos - toScreenMidPos) * 0.9f;
	else
		toScreenMidPos += (screenMidPos - toScreenMidPos) * 0.1f;

	dofParams.toScreenMidPosAlpha[0] = toScreenMidPos.x;
	dofParams.toScreenMidPosAlpha[1] = toScreenMidPos.y;
	dofParams.toScreenMidPosAlpha[2] = toScreenMidPos.z;
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::SetMidScreenMessage()
{
	std::string midScrText;
	if (onLadderMsg)
	{
		if (curMidScreenTextType != CLIMB) midScrText = !ActionHasKey(CMD_USE) ? "Use key is unbound. Bind and press it to climb..." : "Press '" + GetKeynameForAction(CMD_USE) + "' to climb...";
		targetMidScreenTextType = CLIMB;
		midScreenAlphaTarget = 1.5f;
	}
	else
	{
		targetMidScreenTextType = NONE;
		midScreenAlphaTarget = 0.5f;
	}

	if (curMidScreenTextType != targetMidScreenTextType && (targetMidScreenTextType != NONE || fabs(dofParams.midScreenAlpha - midScreenAlphaTarget) < 0.1f))
	{
		if (targetMidScreenTextType == NONE) midScrText = "";
		std::vector<unsigned char> textIdxVerts;
		vec3 extents = HIGHOMEGA::RENDER::TEXT::MakeGeomFromText(midScrText, vec2(0.02f, 0.06f), vec3(0.0f), textIdxVerts);
		mat4 transMat;
		transMat.Ident();
		transMat.i[0][3] = extents.x * -0.5f;
		transMat.i[1][3] = extents.y * 0.5f;
		if (midScreenTextInst)
		{
			midScreenTextInst->Update(transMat);
			std::string midScreenGroupId = "midScreenTextGroup";
			midScreenTextInst->modelRef->ChangeGeom(midScreenGroupId, textIdxVerts);
		}
		curMidScreenTextType = targetMidScreenTextType;
	}

	dofParams.midScreenAlpha += (midScreenAlphaTarget - dofParams.midScreenAlpha) * 0.1f;
	if (fabs(dofParams.midScreenAlpha - midScreenAlphaTarget) < 0.1f) dofParams.midScreenAlpha = midScreenAlphaTarget;
}

void HIGHOMEGA::RENDER::PASSES::DoFClass::SetIsOnLadder(bool isOnLadder)
{
	onLadderMsg = isOnLadder;
}

void HIGHOMEGA::RENDER::PASSES::SaurayDisplayTestClass::Create(TriClass & PostProcessTri, SaurayTraceClass & SaurayTrace)
{
	submission.Add(*PostProcessTri.triModelInstance);
	submission.Create(Instance);

	submission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/saurayTestOutput.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, PostProcessTri.triFrustum.Buffer);
	shader.AddResource(RESOURCE_SAMPLER, FRAGMENT, 0, 1, SaurayTrace.testOutput);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::SaurayDisplayTestClass::Render()
{
	if (!HIGHOMEGA::EVENTS::windowMinimized) submission.MakeAsync().Render();
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
	credits.Model(creditsMesh, "assets/credits/", Instance, inpFilterFunction, false);
	creditsInst = credits.CreateInstance();

	mat4 creditsMat;
	creditsMat.Ident();
	creditsMat.i[0][0] = ((float)ScreenSize.width / (float)ScreenSize.height) / 1.777777777f; // credits is kinda 1080p-ish in terms of whr
	splashParams.alphaAmount = 1.0f;
	splashParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &splashParams, (unsigned int)sizeof(splashParams));

	submission.Add(*creditsInst);
	credits.transformVertsSlow(creditsMat);
	submission.Create(Instance);

	submission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/splashShader.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, splashFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, splashParamsBuf);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::SplashDisplayClass::Render()
{
	splashFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, (float)ScreenSize.width / (float)ScreenSize.height, 0.1f, 100.0f);
	splashFrustum.Update();

	mat4 creditsMat;
	creditsMat.Ident();
	creditsMat.i[0][0] = ((float)ScreenSize.width / (float)ScreenSize.height) / 1.777777777f; // credits is kinda 1080p-ish in terms of whr
	credits.transformVertsSlow(creditsMat);

	splashParams.alphaAmount = min(timePassed / 4.0f, 1.0f);
	splashParamsBuf.UploadSubData(0, &splashParams, sizeof(splashParams));

	splashTimer.Start();

	if (!HIGHOMEGA::EVENTS::windowMinimized) submission.MakeAsync().Render();

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

void HIGHOMEGA::RENDER::PASSES::MainMenuClass::Create(TriClass & PostProcessTri, bool cmdOptHwRt, unsigned int cmdOptFullRes, WINDOW_MODE cmdOptWindowed)
{
	this->hwrtSelection = cmdOptHwRt;
	this->fullResSelection = cmdOptFullRes;
	this->windowedSelection = cmdOptWindowed;

	mouseMovementConsumers["MainMenuCursor"] = vec2(0.0f);

	menuFrustum.CreatePerspective(vec3(0.0f, 0.0f, 1.0f), vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 1.0f, 0.0f), 90.0f, (float)ScreenSize.width / (float)ScreenSize.height, 0.1f, 100.0f);
	menuFrustum.Update();

	menuParams.alphaAmount = 1.0f;
	menuParamsBuf.Buffer(MEMORY_HOST_VISIBLE, GRAPHICS_QUEUE, QUEUE_EXCLUSIVE, USAGE_UBO, Instance, &menuParams, (unsigned int)sizeof(menuParams));

	std::function<bool(int, DataGroup &)> inpFilterFunction = [](int, DataGroup & inpGroup) -> bool {
		float tmpFloat;
		return !Mesh::getDataRowFloat(inpGroup, "PROPS", "cloth", tmpFloat);
	};

	Mesh cursorMesh = Mesh("assets/mainmenu/cursor.3md");
	Mesh mainMenuMesh = Mesh("assets/mainmenu/mainmenu.3md");
	Mesh hwrtbasedMesh = Mesh("assets/mainmenu/hwrtbased.3md");
	Mesh sdfbvhbasedMesh = Mesh("assets/mainmenu/sdfbvhbased.3md");
	Mesh windowedMesh = Mesh("assets/mainmenu/windowed.3md");
	Mesh winfullscrMesh = Mesh("assets/mainmenu/winfullscr.3md");
	Mesh fullscreenMesh = Mesh("assets/mainmenu/fullscreen.3md");
	Mesh loadingMesh = Mesh("assets/mainmenu/loading.3md");
	cursor.Model(cursorMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	mainmenu.Model(mainMenuMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	hwrtbased.Model(hwrtbasedMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	sdfbvhbased.Model(sdfbvhbasedMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	windowed.Model(windowedMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	winfullscr.Model(winfullscrMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	fullscreen.Model(fullscreenMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);
	loading.Model(loadingMesh, "assets/mainmenu/", Instance, inpFilterFunction, false);

	loadingInst = loading.CreateInstance();
	for (int i = 0; i != allowedResolutions.size(); i++)
	{
		std::string resModelName = "assets/mainmenu/";
		resModelName += std::to_string((int)allowedResolutions[i].x);
		resModelName += "x";
		resModelName += std::to_string((int)allowedResolutions[i].y);
		resModelName += ".3md";
		Mesh resMeshName = Mesh(resModelName);
		screenRes[i].Model(resMeshName, "assets/mainmenu/", Instance, inpFilterFunction, false);
		screenResInst[i] = screenRes[i].CreateInstance();
	}
	cursorInst = cursor.CreateInstance();
	hwrtbasedInst = hwrtbased.CreateInstance();
	sdfbvhbasedInst = sdfbvhbased.CreateInstance();
	windowedInst = windowed.CreateInstance();
	winfullscrInst = winfullscr.CreateInstance();
	fullscreenInst = fullscreen.CreateInstance();
	mainmenuInst = mainmenu.CreateInstance();
	cursorPos = vec2(0.0f);

	mat4 cursorItemMat = getCursorMat();
	cursor.transformVertsSlow(cursorItemMat);

	submission.Create(Instance);
	SetupScreen();

	submission.SetFrameBuffer(Instance.swapChainFrameBuffer());

	shader.Create("shaders/postprocess.vert.spv", "main", "shaders/splashShader.frag.spv", "main");
	shader.AddResource(RESOURCE_UBO, VERTEX, 0, 0, menuFrustum.Buffer);
	shader.AddResource(RESOURCE_UBO, FRAGMENT, 0, 1, menuParamsBuf);
	submission.SetShader("default", shader);
}

void HIGHOMEGA::RENDER::PASSES::MainMenuClass::SetupScreen()
{
	techniqueItemMat = getItemMat(0.1825f - 0.1125f * 0.0f);
	screenResItemMat = getItemMat(0.1825f - 0.1125f * 1.0f);
	windowedItemMat = getItemMat(0.1825f - 0.1125f * 2.0f);

	if (Instance.SupportsHWRT() && this->hwrtSelection)
	{
		submission.Add(*hwrtbasedInst);
		hwrtbased.transformVertsSlow(techniqueItemMat);
	}
	else
	{
		this->hwrtSelection = false;
		submission.Add(*sdfbvhbasedInst);
		sdfbvhbased.transformVertsSlow(techniqueItemMat);
	}
	submission.Add(*(screenResInst[this->fullResSelection]));
	for (int i = 0; i != allowedResolutions.size(); i++)
		screenRes[i].transformVertsSlow(screenResItemMat);
	switch (this->windowedSelection)
	{
	case WINDOWED:
	{
		submission.Add(*windowedInst);
		windowed.transformVertsSlow(windowedItemMat);
		break;
	}
	case WINDOWED_FULLSCREEN:
	{
		submission.Add(*winfullscrInst);
		winfullscr.transformVertsSlow(windowedItemMat);
		break;
	}
	case FULLSCREEN:
	{
		submission.Add(*fullscreenInst);
		fullscreen.transformVertsSlow(windowedItemMat);
		break;
	}
	}
	submission.Add(*mainmenuInst);
	submission.Add(*cursorInst);
}

void HIGHOMEGA::RENDER::PASSES::MainMenuClass::Render(bool postInit, const std::function<void(void)>& switchToSDFBVH, const std::function<void(void)>& switchToHWRT)
{
	menuFrustum.screen_whr = (float)ScreenSize.width / (float)ScreenSize.height;
	menuFrustum.Update();
	if (showLoading)
	{
		submission.Remove(*loadingInst);
		SetupScreen();
		showLoading = false;
	}
	vec2 cursorDelta = mouseMovementConsumers["MainMenuCursor"];
	mouseMovementConsumers["MainMenuCursor"] = vec2(0.0f);
	cursorPos += cursorDelta * 0.002f;
	cursorPos.x = Clamp (cursorPos.x, -menuFrustum.screen_whr, menuFrustum.screen_whr);
	cursorPos.y = Clamp (cursorPos.y, -1.0f, 1.0f);

	mat4 cursorMat = getCursorMat();
	cursor.transformVertsSlow(cursorMat);

	bool curLeftMouseDown = HIGHOMEGA::EVENTS::leftMouseDown;
	bool clickDone = (!curLeftMouseDown && prevLeftMouseDown);

	float techniqueItemButtonY = 0.15f - 0.11f * 0.0f;
	float screenResItemButtonY = 0.15f - 0.11f * 1.0f;
	float windowedItemButtonY = 0.15f - 0.11f * 2.0f;

	if ((overButton(vec2(0.172f, techniqueItemButtonY), vec2(0.025f)) || overButton(vec2(0.658f, techniqueItemButtonY), vec2(0.025f))) && clickDone && Instance.SupportsHWRT())
	{
		hwrtSelection = !hwrtSelection;
		submission.Remove(*hwrtbasedInst);
		submission.Remove(*sdfbvhbasedInst);
		if (hwrtSelection)
		{
			submission.Add(*hwrtbasedInst);
			hwrtbased.transformVertsSlow(techniqueItemMat);
		}
		else
		{
			submission.Add(*sdfbvhbasedInst);
			sdfbvhbased.transformVertsSlow(techniqueItemMat);
		}
	}

	if ((overButton(vec2(0.172f, screenResItemButtonY), vec2(0.025f)) || overButton(vec2(0.658f, screenResItemButtonY), vec2(0.025f))) && clickDone)
	{
		if (overButton(vec2(0.172f, screenResItemButtonY), vec2(0.025f))) fullResSelection--;
		else if (overButton(vec2(0.658f, screenResItemButtonY), vec2(0.025f))) fullResSelection++;
		if (fullResSelection == -1) fullResSelection = 4;
		else if (fullResSelection == 5) fullResSelection = 0;
		for (int i = 0; i != 5; i++)
			submission.Remove(*(screenResInst[i]));
		submission.Add(*(screenResInst[fullResSelection]));
		screenRes[fullResSelection].transformVertsSlow(screenResItemMat);
	}

	bool overBackButton;
	if ((overBackButton = overButton(vec2(0.172f, windowedItemButtonY), vec2(0.025f)) || overButton(vec2(0.658f, windowedItemButtonY), vec2(0.025f))) && clickDone)
	{
		int windowedSelectionInt = (int)windowedSelection + (overBackButton ? -1 : 1);
		if (windowedSelectionInt == -1) windowedSelectionInt = 2;
		else if (windowedSelectionInt == 3) windowedSelectionInt = 0;
		windowedSelection = (WINDOW_MODE)windowedSelectionInt;
		submission.Remove(*windowedInst);
		submission.Remove(*winfullscrInst);
		submission.Remove(*fullscreenInst);
		switch (windowedSelection)
		{
			case WINDOWED:
			{
				submission.Add(*windowedInst);
				windowed.transformVertsSlow(windowedItemMat);
				break;
			}
			case WINDOWED_FULLSCREEN:
			{
				submission.Add(*winfullscrInst);
				winfullscr.transformVertsSlow(windowedItemMat);
				break;
			}
			case FULLSCREEN:
			{
				submission.Add(*fullscreenInst);
				fullscreen.transformVertsSlow(windowedItemMat);
				break;
			}
		}
	}

	if (overButton(vec2(-0.316f, -0.624f), vec2(0.147777f, 0.038f)) && clickDone)
	{
		WindowRecreate((int)allowedResolutions[fullResSelection].x, (int)allowedResolutions[fullResSelection].y, windowedSelection);
		if (Instance.SupportsHWRT() && hwrtSelection)
		{
			switchToHWRT();
		}
		else if (!hwrtSelection)
		{
			switchToSDFBVH();
		}
	}

	if (overButton(vec2(0.322f, -0.624f), vec2(0.147777f, 0.038f)) && clickDone)
	{
		submission.Remove(*cursorInst);
		submission.Remove(*sdfbvhbasedInst);
		submission.Remove(*hwrtbasedInst);
		for (int i = 0; i != 5; i++)
			submission.Remove(*(screenResInst[i]));
		submission.Remove(*windowedInst);
		submission.Remove(*winfullscrInst);
		submission.Remove(*fullscreenInst);
		submission.Remove(*mainmenuInst);
		mat4 midMat;
		midMat.Ident();
		midMat.i[0][0] = 0.56f;
		submission.Add(*loadingInst);
		loading.transformVertsSlow(midMat);
		showLoading = true;
		done = true;
	}
	static bool prevHitEsc = false;
	bool hitEsc = GetStateOfAction(CMD_MAIN_MENU);
	if (postInit && !hitEsc && prevHitEsc)
	{
		done = true;
	}
	prevHitEsc = hitEsc;

	if (!HIGHOMEGA::EVENTS::windowMinimized) submission.MakeAsync().Render();

	prevLeftMouseDown = curLeftMouseDown;
}

bool HIGHOMEGA::RENDER::PASSES::MainMenuClass::IsDone()
{
	return done;
}

void HIGHOMEGA::RENDER::PASSES::MainMenuClass::SetNotDone()
{
	done = false;
}

bool HIGHOMEGA::RENDER::PASSES::MainMenuClass::IsRebooting()
{
	return rebooting;
}