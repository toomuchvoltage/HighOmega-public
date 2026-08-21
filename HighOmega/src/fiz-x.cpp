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

#include <fiz-x.h>

using namespace HIGHOMEGA::FIZ_X;
using namespace HIGHOMEGA::RENDER;

vec3 HIGHOMEGA::FIZ_X::GravDir = vec3(0.0f, -1.0f, 0.0f);
float HIGHOMEGA::FIZ_X::Grav = 9.81f;
float HIGHOMEGA::FIZ_X::DeltaT = 0.05f; // The amount of time that passes between frames
float HIGHOMEGA::FIZ_X::ColTol = 1.0f; // Collision Tolerance value

std::vector<RigidBody *> HIGHOMEGA::FIZ_X::allBodies;
std::vector<std::vector<BROADSWEEP_RESULT>> HIGHOMEGA::FIZ_X::collisionMap;
std::vector <intersection> HIGHOMEGA::FIZ_X::collisionNodes;
std::vector <ConstraintCollection *> HIGHOMEGA::FIZ_X::constraintCollection;
std::vector<std::thread *> HIGHOMEGA::FIZ_X::collisionThreads;
fizXThreadStruct HIGHOMEGA::FIZ_X::collisionThreadParams[HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT];
fizXSignalStruct HIGHOMEGA::FIZ_X::fizXSignal;
BuoyancyRangeCollectionClass HIGHOMEGA::FIZ_X::buoyancyRangeCollection;
std::unordered_map<unsigned long long, PlayerClothObstacles> HIGHOMEGA::FIZ_X::allObstacles;
std::shared_mutex HIGHOMEGA::FIZ_X::allObstaclesMutex;

MATERIAL HIGHOMEGA::FIZ_X::ObjectPiece::GetMaterial(std::string & mat)
{
	if (mat == "concrete") return CONCRETE;
	else if (mat == "dirt") return DIRT;
	else if (mat == "rubble") return RUBBLE;
	else if (mat == "ceramic") return CERAMIC;
	else if (mat == "glass") return GLASS;
	else if (mat == "metal") return METALPLATFORM;
	else if (mat == "metalrods") return METALRODS;
	else if (mat == "plastic") return PLASTIC;
	else if (mat == "wood") return WOOD;
	else if (mat == "grass") return GRASS;
	else if (mat == "water") return WATER;
	return DIRT;
}

float HIGHOMEGA::FIZ_X::ObjectPiece::GetDensity(std::string & inTex)
{
	if (inTex == "wood.tga") return 0.03f;
	return 0.01f;
}

float HIGHOMEGA::FIZ_X::ObjectPiece::GetElasticity(std::string & inTex)
{
	return 0.1f;
}

float HIGHOMEGA::FIZ_X::ObjectPiece::GetFriction(std::string & inTex)
{
	return 0.1f;
}

HIGHOMEGA::FIZ_X::ObjectPiece::ObjectPiece()
{
}

HIGHOMEGA::FIZ_X::ObjectPiece::ObjectPiece(std::string pieceId, DataBlock & inpBlock)
{
	if (!Mesh::getDataRowString(inpBlock, "texname", diffName)) FATAL_ERROR("Error fetching diffuse tex. name for RigidBody");

	elasticity = GetElasticity(diffName);
	friction = GetFriction(diffName);
	groupId = pieceId;

	float strengthFloat;
	if (cuttable = Mesh::getDataRowFloat(inpBlock, "cuttable", strengthFloat))
	{
		strength = (int)strengthFloat;
		if (!Mesh::getDataRowFloat(inpBlock, "cutRadius", cutRadius)) cutRadius = 2.0f;
		if (!Mesh::getDataRowFloat(inpBlock, "cutDepth", cutDepth)) cutDepth = 4.0f;
	}
	if (shatterable = Mesh::getDataRowFloat(inpBlock, "shatterable", strengthFloat))
		strength = (int)strengthFloat;

	std::string matType;
	if (!Mesh::getDataRowString(inpBlock, "material", matType))
		matType = std::string("concrete");
	mat = GetMaterial(matType);
}

void HIGHOMEGA::FIZ_X::RigidBody::IT_Tri(vec3 & v1, vec3 & v2, vec3 & v3, vec3 & v1_v2, vec3 & v3_v2, float step, float * Ixx, float * Iyy, float * Izz, float * Ixy, float * Iyz, float * Izx)
{
	vec3 v3_v2_norm = v3_v2.normalized();
	vec3 altitudeBase = v2 + v1_v2 * v3_v2_norm * v3_v2_norm;
	float altitudeLen = (v1 - altitudeBase).length();
	vec3 v3_v1 = v3 - v1;

	vec3 startL, endL, lDir, lDirNorm, center, centSq;
	float lDirLen;
	for (float i = 0.0f; i < altitudeLen; i += step)
	{
		float frac = (i + step * 0.5f) / altitudeLen;
		startL = v1 - v1_v2 * frac;
		endL = v1 + v3_v1 * frac;
		lDir = endL - startL;
		lDirLen = lDir.length();
		if (lDirLen == 0.0f) continue;
		lDirNorm = lDir / lDirLen;
		for (float j = 0.0f; j < lDirLen; j += step)
		{
			center = startL + lDirNorm * (j + step * 0.5f);
			centSq = vec3(center.x * center.x, center.y * center.y, center.z * center.z);
			*Ixx += centSq.y + centSq.z;
			*Iyy += centSq.x + centSq.z;
			*Izz += centSq.x + centSq.y;
			*Ixy -= center.x * center.y;
			*Iyz -= center.y * center.z;
			*Izx -= center.z * center.x;
		}
	}
}

void HIGHOMEGA::FIZ_X::RigidBody::GetCurInertiaTensorInv()
{
	cur_inertia_tensor_inv = orient*inertia_tensor_inv*orient.Transpose();
}

void HIGHOMEGA::FIZ_X::RigidBody::GenerateGeom(std::vector<TriUV>& triList, ObjectPiece & objPiece)
{
	objPiece.tris.clear();
	transOnce = false;
	objPiece.verts.clear();
	objPiece.tris.reserve(triList.size() * 3);
	objPiece.verts.reserve(triList.size() * 3);

	for (TriUV& curTri : triList)
	{
		objPiece.verts.emplace_back(curTri.eArr[0].x, curTri.eArr[0].y, curTri.eArr[0].z);
		objPiece.verts.emplace_back(curTri.eArr[1].x, curTri.eArr[1].y, curTri.eArr[1].z);
		objPiece.verts.emplace_back(curTri.eArr[2].x, curTri.eArr[2].y, curTri.eArr[2].z);
	}

	unsigned int triCounter = 0u;
	for (TriUV & curTri : triList)
	{
		vec3 faceNorm = cross(curTri.eArr[0] - curTri.eArr[1], curTri.eArr[2] - curTri.eArr[1]).normalized();
		objPiece.tris.emplace_back(triCounter++, triCounter++, triCounter++, faceNorm * curTri.normVec < 0.0f);
	}
}

HIGHOMEGA::FIZ_X::RigidBody::~RigidBody()
{
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody()
{
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody(std::string & newGroupId, DataBlock &propBlock, std::vector<TriUV>& triList, vec3 inpNewCenter)
{
	pieces.emplace_back(ObjectPiece(newGroupId, propBlock));
	ObjectPiece & newPiece = pieces[0];

	GenerateGeom(triList, newPiece);

	// Initializing all that needs to be initialized

	isMap = false;

	pos = inpNewCenter;
	orient.Ident();

	InitMassInertiaTensorAndBoundingSpheres();
	Update();
	BuildBVH();

	lin_v = vec3(0);
	ang_v = vec3(0);

	Frozen = false;
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody(Mesh & inpMesh, std::string belong , mat4 inpOrient, vec3 inpPos, bool inIsMap, std::function<bool(RigidBody *, int, DataGroup &)> inpFilterFunction, vec3 *inpNewCenter)
{
	isMap = inIsMap;

	vec3 newCenter(0.0f);
	if (inpNewCenter) newCenter = *inpNewCenter;

	pos = inpPos + newCenter;
	orient = inpOrient;

	vec3 e1, e2, e3;
	vec3 fNorm;
	for (unsigned int i = 0u; i != inpMesh.DataGroups.size(); i++)
	{
		if (!inpFilterFunction(this, i, inpMesh.DataGroups[i])) continue;

		DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		DataBlock *triBlock;
		if ( !Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock) ) continue;

		float outTmp;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "holographic", outTmp)) continue;

		DataBlock *propsBlock;
		if (!Mesh::getDataBlock(curPolyGroup, "PROPS", &propsBlock)) continue;

		std::vector<mat4> matrices;
		HIGHOMEGA::MESH::DataBlock* instBlock = nullptr;
		if (Mesh::getDataBlock(curPolyGroup, "INSTANCES", &instBlock))
		{
			unsigned int numInstances = (unsigned int)(instBlock->blob.size() / (sizeof(float) * 16));
			matrices.resize(numInstances);
			for (unsigned int j = 0u; j != numInstances; j++)
				PackMat4(&instBlock->blob.data()[j * sizeof(float) * 16], matrices[j]);
		}
		else
		{
			mat4 tmpMat;
			tmpMat.Ident();
			matrices.push_back(tmpMat);
		}

		RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
		getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

		for (unsigned int j = 0u; j != matrices.size(); j++)
		{
			ObjectPiece& curPiece = pieces.emplace_back(ObjectPiece(curPolyGroup.name, *propsBlock));
			curPiece.verts.reserve(vertCount);
			curPiece.tris.reserve(triCount);
			for (unsigned int k = 0u; k != vertCount; k++)
			{
				e1 = (matrices[j] * vec3 (verts[k].posCol[0], verts[k].posCol[1], verts[k].posCol[2])) - newCenter;
				curPiece.verts.emplace_back(e1.x, e1.y, e1.z);
			}
			for (unsigned int k = 0u; k != triCount; k++)
			{
				e1 = vec3(curPiece.verts[indices[k * 3]].x, curPiece.verts[indices[k * 3]].y, curPiece.verts[indices[k * 3]].z);
				e2 = vec3(curPiece.verts[indices[k * 3 + 1]].x, curPiece.verts[indices[k * 3 + 1]].y, curPiece.verts[indices[k * 3 + 1]].z);
				e3 = vec3(curPiece.verts[indices[k * 3 + 2]].x, curPiece.verts[indices[k * 3 + 2]].y, curPiece.verts[indices[k * 3 + 2]].z);
				fNorm = cross(e1 - e2, e3 - e2);
				curPiece.tris.emplace_back(indices[k * 3], indices[k * 3 + 1], indices[k * 3 + 2], (fNorm * fromZSignXY(verts[indices[k * 3]].Norm)) < 0.0f);
			}
		}
	}

	transOnce = false;
	Frozen = false;

	InitMassInertiaTensorAndBoundingSpheres();
	Update();
	if (isMap) mass = HIGHOMEGA_INFINITE_MASS;
	BuildBVH();

	lin_v = vec3(0);
	ang_v = vec3(0);


	bobbies_dia = 0.0f;
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		if (inpMesh.DataGroups[i].type != "BOBBIES") continue;

		DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		DataBlock *buoyancyBobbies = nullptr;
		if ( !Mesh::getDataBlock(curPolyGroup, "DESCRIPTION", &buoyancyBobbies)) FATAL_ERROR("Error fetching desc. for RigidBody bobbies");

		if ( !buoyancyBobbies->rows[0][1].fvalue(bobbies_dia)) FATAL_ERROR("Error fetching diameter for RigidBody bobbies");

		float xFetch, yFetch, zFetch;
		for (int j = 1; j != buoyancyBobbies->rows.size(); j++)
		{
			if ( !buoyancyBobbies->rows[j][0].fvalue(xFetch) ||
					!buoyancyBobbies->rows[j][1].fvalue(yFetch) ||
					!buoyancyBobbies->rows[j][2].fvalue(zFetch) ) FATAL_ERROR("Error fetching bobby for RigidBody");
			bobbies.emplace_back(xFetch, yFetch, zFetch);
		}

		break;
	}
}

void HIGHOMEGA::FIZ_X::RigidBody::ApplyForce(vec3 force, vec3 point, float delta_t)
{
	if (mass == HIGHOMEGA_INFINITE_MASS) return;
	lin_v += (force / mass)*delta_t;
	ang_v += (cur_inertia_tensor_inv*cross(point - pos, force))*delta_t;
	Frozen = false;
}

void HIGHOMEGA::FIZ_X::RigidBody::ApplyImpulse(vec3 impulse, vec3 point)
{
	if (mass == HIGHOMEGA_INFINITE_MASS) return;
	lin_v += impulse / mass;
	ang_v += cur_inertia_tensor_inv*cross(point - pos, impulse);
	Frozen = false;
}

void HIGHOMEGA::FIZ_X::RigidBody::EnqeueImpulse(vec3 impulse, vec3 point)
{
	if (mass == HIGHOMEGA_INFINITE_MASS) return;
	{ std::lock_guard<std::mutex> lk(enqueuedImpulseMutex);
	enqueuedImpulses.emplace_back(impulse, point); }
}

void HIGHOMEGA::FIZ_X::RigidBody::ProcessEnqueuedImpulses()
{
	{ std::lock_guard<std::mutex> lk(enqueuedImpulseMutex);
	for (EnqueuedImpulse& curImpulse : enqueuedImpulses)
		ApplyImpulse(curImpulse.impulse, curImpulse.pos);
	enqueuedImpulses.clear(); }
}

void HIGHOMEGA::FIZ_X::RigidBody::Move(float delta_t)
{
	if (Frozen || mass == HIGHOMEGA_INFINITE_MASS) return;
	pos += lin_v*delta_t;
	if (ang_v.length() > 0.0001f)
	{
		vec3 c1(orient.i[0][0], orient.i[1][0], orient.i[2][0]);
		vec3 c2(orient.i[0][1], orient.i[1][1], orient.i[2][1]);
		vec3 c3(orient.i[0][2], orient.i[1][2], orient.i[2][2]);
		vec3 r_dot = ang_v*delta_t;
		float r_dot_l = r_dot.length();
		c1 = Spin(r_dot, c1, r_dot_l).normalized();
		c2 = Spin(r_dot, c2, r_dot_l).normalized();
		c3 = Spin(r_dot, c3, r_dot_l).normalized();
		orient.i[0][0] = c1.x; orient.i[1][0] = c1.y; orient.i[2][0] = c1.z;
		orient.i[0][1] = c2.x; orient.i[1][1] = c2.y; orient.i[2][1] = c2.z;
		orient.i[0][2] = c3.x; orient.i[1][2] = c3.y; orient.i[2][2] = c3.z;
	}
}

void HIGHOMEGA::FIZ_X::RigidBody::InitMassInertiaTensorAndBoundingSpheres()
{
	radius = 0.0;
	mass = 0.0f;

	vec3 objMin = vec3(FLT_MAX), objMax = vec3(-FLT_MAX);
	for (int i = 0; i != pieces.size(); i++)
	{
		ObjectPiece & curPiece = pieces[i];
		curPiece.pos = vec3(0.0f);
		curPiece.radius = 0.0f;
		vec3 pieceMin = vec3(FLT_MAX), pieceMax = vec3(-FLT_MAX);
		for (int j = 0; j != curPiece.tris.size(); j++)
		{
			pieceMin.x = min(pieceMin.x, Min(curPiece.verts[curPiece.tris[j].e1].x, curPiece.verts[curPiece.tris[j].e2].x, curPiece.verts[curPiece.tris[j].e3].x));
			pieceMin.y = min(pieceMin.y, Min(curPiece.verts[curPiece.tris[j].e1].y, curPiece.verts[curPiece.tris[j].e2].y, curPiece.verts[curPiece.tris[j].e3].y));
			pieceMin.z = min(pieceMin.z, Min(curPiece.verts[curPiece.tris[j].e1].z, curPiece.verts[curPiece.tris[j].e2].z, curPiece.verts[curPiece.tris[j].e3].z));
			pieceMax.x = max(pieceMax.x, Max(curPiece.verts[curPiece.tris[j].e1].x, curPiece.verts[curPiece.tris[j].e2].x, curPiece.verts[curPiece.tris[j].e3].x));
			pieceMax.y = max(pieceMax.y, Max(curPiece.verts[curPiece.tris[j].e1].y, curPiece.verts[curPiece.tris[j].e2].y, curPiece.verts[curPiece.tris[j].e3].y));
			pieceMax.z = max(pieceMax.z, Max(curPiece.verts[curPiece.tris[j].e1].z, curPiece.verts[curPiece.tris[j].e2].z, curPiece.verts[curPiece.tris[j].e3].z));
		}
		objMin.x = min(objMin.x, pieceMin.x);
		objMin.y = min(objMin.y, pieceMin.y);
		objMin.z = min(objMin.z, pieceMin.z);
		objMax.x = max(objMax.x, pieceMax.x);
		objMax.y = max(objMax.y, pieceMax.y);
		objMax.z = max(objMax.z, pieceMax.z);

		curPiece.pos = (pieceMin + pieceMax) * 0.5f;
		curPiece.radius = (pieceMax - curPiece.pos).length();
	}
	vec3 objCent = (objMin + objMax) * 0.5f;
	radius = (objMax - objCent).length();

	inertia_tensor.Ident();
	if (!isMap)
	{
		float Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Iyz = 0, Izx = 0;
		vec3 v1, v2, v3, v1_v2, v3_v2, triAreaCross;
		for (ObjectPiece & curPiece : pieces)
		{
			float IxxAccum = 0, IyyAccum = 0, IzzAccum = 0, IxyAccum = 0, IyzAccum = 0, IzxAccum = 0;
			float curDensity = curPiece.GetDensity(curPiece.diffName);
			for (int j = 0; j != curPiece.tris.size(); j++)
			{
				v1 = vec3(curPiece.verts[curPiece.tris[j].e1].x, curPiece.verts[curPiece.tris[j].e1].y, curPiece.verts[curPiece.tris[j].e1].z);
				v2 = vec3(curPiece.verts[curPiece.tris[j].e2].x, curPiece.verts[curPiece.tris[j].e2].y, curPiece.verts[curPiece.tris[j].e2].z);
				v3 = vec3(curPiece.verts[curPiece.tris[j].e3].x, curPiece.verts[curPiece.tris[j].e3].y, curPiece.verts[curPiece.tris[j].e3].z);
				v1_v2 = v1 - v2;
				v3_v2 = v3 - v2;
				triAreaCross = cross(v1_v2, v3_v2);
				float triAreaApprox = triAreaCross * triAreaCross;
				if (triAreaApprox < 0.000004f) continue;
				float triArea = sqrtf(triAreaApprox) * 0.5f;
				float triMass = triArea * curDensity;
				mass += triMass;
				IT_Tri(v1, v2, v3, v1_v2, v3_v2, 0.1f, &IxxAccum, &IyyAccum, &IzzAccum, &IxyAccum, &IyzAccum, &IzxAccum);
			}
			float triFragMass = curDensity * 0.01f;
			Ixx += IxxAccum * triFragMass;
			Iyy += IyyAccum * triFragMass;
			Izz += IzzAccum * triFragMass;
			Ixy += IxyAccum * triFragMass;
			Izx += IzxAccum * triFragMass;
			Iyz += IyzAccum * triFragMass;
		}

		inertia_tensor.i[0][0] = Ixx;
		inertia_tensor.i[1][1] = Iyy;
		inertia_tensor.i[2][2] = Izz;
		inertia_tensor.i[0][1] = inertia_tensor.i[1][0] = Ixy;
		inertia_tensor.i[0][2] = inertia_tensor.i[2][0] = Izx;
		inertia_tensor.i[1][2] = inertia_tensor.i[2][1] = Iyz;
	}
	inertia_tensor_inv = inertia_tensor.Inv();
}

void HIGHOMEGA::FIZ_X::RigidBody::BuildBVH()
{
	triRefs.clear();
	boxLeaves.clear();
	cwbvh.nodes.clear();
	cwbvh.compressedNodes.clear();

	mat4 toObjSpace;
	if (!isMap) toObjSpace = prevTrans.Inv();

	vec3 e1, e2, e3;
	for (unsigned int i = 0; i != pieces.size(); i++)
	{
		ObjectPiece& curPiece = pieces[i];
		for (unsigned int j = 0; j != curPiece.tris.size(); j++)
		{
			ObjectTri& curTri = curPiece.tris[j];
			if (!isMap)
			{
				e1 = toObjSpace * vec3(curPiece.verts[curTri.e1].x, curPiece.verts[curTri.e1].y, curPiece.verts[curTri.e1].z);
				e2 = toObjSpace * vec3(curPiece.verts[curTri.e2].x, curPiece.verts[curTri.e2].y, curPiece.verts[curTri.e2].z);
				e3 = toObjSpace * vec3(curPiece.verts[curTri.e3].x, curPiece.verts[curTri.e3].y, curPiece.verts[curTri.e3].z);
			}
			else
			{
				e1 = vec3(curPiece.verts[curTri.e1].x, curPiece.verts[curTri.e1].y, curPiece.verts[curTri.e1].z);
				e2 = vec3(curPiece.verts[curTri.e2].x, curPiece.verts[curTri.e2].y, curPiece.verts[curTri.e2].z);
				e3 = vec3(curPiece.verts[curTri.e3].x, curPiece.verts[curTri.e3].y, curPiece.verts[curTri.e3].z);
			}
			triRefs.emplace_back(i, j);
			unsigned int LeafId = (unsigned int)(triRefs.size() - 1);
			BoxLeaf boxLeaf;
			boxLeaf.leafMaxLeafId[0] = Max(e1.x, e2.x, e3.x);
			boxLeaf.leafMaxLeafId[1] = Max(e1.y, e2.y, e3.y);
			boxLeaf.leafMaxLeafId[2] = Max(e1.z, e2.z, e3.z);
			boxLeaf.leafMaxLeafId[3] = *((float*)&LeafId);
			boxLeaf.leafMinMorton[0] = Min(e1.x, e2.x, e3.x);
			boxLeaf.leafMinMorton[1] = Min(e1.y, e2.y, e3.y);
			boxLeaf.leafMinMorton[2] = Min(e1.z, e2.z, e3.z);
			boxLeaves.emplace_back(boxLeaf);
		}
	}

	if (!boxLeaves.size()) return;

	SDFBVHClass::CalcMortons(boxLeaves.data(), 0, (unsigned int)(boxLeaves.size() - 1), body_min, body_max);
	std::vector<BoxLeaf> tmpBoxLeaves;
	RadixSort(boxLeaves, tmpBoxLeaves);
	cwbvh.Build(boxLeaves);
}

void HIGHOMEGA::FIZ_X::RigidBody::Update()
{
	if (Frozen || mass == HIGHOMEGA_INFINITE_MASS) return;

	if (!transOnce)
	{
		transOnce = true;
		prevTrans.Ident();
	}

	mat4 totalTrans = orient;
	totalTrans.i[0][3] = pos.x;
	totalTrans.i[1][3] = pos.y;
	totalTrans.i[2][3] = pos.z;
	mat4 totalTransFromRef = totalTrans * prevTrans.Inv();

	vec3 transEdge;
	for (ObjectPiece& curPiece : pieces)
		for (unsigned int i = 0; i != curPiece.verts.size(); i++)
		{
			transEdge = totalTransFromRef * vec3(curPiece.verts[i].x, curPiece.verts[i].y, curPiece.verts[i].z);
			curPiece.verts[i] = { transEdge.x, transEdge.y, transEdge.z };
		}

	prevTrans = totalTrans;

	GetCurInertiaTensorInv();
	UpdateAABB();
}

void HIGHOMEGA::FIZ_X::RigidBody::UpdateAABB()
{
	body_min = vec3(FLT_MAX);
	body_max = vec3(-FLT_MAX);
	for (int i = 0; i != pieces.size(); i++)
	{
		ObjectPiece & curPiece = pieces[i];
		curPiece.piece_min = vec3(FLT_MAX);
		curPiece.piece_max = vec3(-FLT_MAX);
		for (int j = 0; j != curPiece.tris.size(); j++)
		{
			curPiece.piece_min.x = min(curPiece.piece_min.x, Min(curPiece.verts[curPiece.tris[j].e1].x, curPiece.verts[curPiece.tris[j].e2].x, curPiece.verts[curPiece.tris[j].e3].x));
			curPiece.piece_min.y = min(curPiece.piece_min.y, Min(curPiece.verts[curPiece.tris[j].e1].y, curPiece.verts[curPiece.tris[j].e2].y, curPiece.verts[curPiece.tris[j].e3].y));
			curPiece.piece_min.z = min(curPiece.piece_min.z, Min(curPiece.verts[curPiece.tris[j].e1].z, curPiece.verts[curPiece.tris[j].e2].z, curPiece.verts[curPiece.tris[j].e3].z));
			curPiece.piece_max.x = max(curPiece.piece_max.x, Max(curPiece.verts[curPiece.tris[j].e1].x, curPiece.verts[curPiece.tris[j].e2].x, curPiece.verts[curPiece.tris[j].e3].x));
			curPiece.piece_max.y = max(curPiece.piece_max.y, Max(curPiece.verts[curPiece.tris[j].e1].y, curPiece.verts[curPiece.tris[j].e2].y, curPiece.verts[curPiece.tris[j].e3].y));
			curPiece.piece_max.z = max(curPiece.piece_max.z, Max(curPiece.verts[curPiece.tris[j].e1].z, curPiece.verts[curPiece.tris[j].e2].z, curPiece.verts[curPiece.tris[j].e3].z));
		}
		body_min.x = min(body_min.x, curPiece.piece_min.x);
		body_min.y = min(body_min.y, curPiece.piece_min.y);
		body_min.z = min(body_min.z, curPiece.piece_min.z);
		body_max.x = max(body_max.x, curPiece.piece_max.x);
		body_max.y = max(body_max.y, curPiece.piece_max.y);
		body_max.z = max(body_max.z, curPiece.piece_max.z);
	}
}

void HIGHOMEGA::FIZ_X::RigidBody::ApplyBuoyancy(vec3 min, vec3 max, float rho, float b, float A, float CD, float dt)
{
	if ( bobbies.size() == 0 ) return;
	vec3 force(0.0f), center(0.0f);

	if (body_max.x < min.x || body_min.x > max.x ||
		body_max.y < min.y || body_min.y > max.y ||
		body_max.z < min.z || body_min.z > max.z) return;

	float bobby_rad = bobbies_dia*0.5f;
	float bobby_volume = (1.33333f)*HIGHOMEGA_PI*(bobby_rad*bobby_rad*bobby_rad);
	float total_volume = 0;
	unsigned int bobbies_submerged = 0;
	for (vec3 & curBobby : bobbies)
	{
		vec3 p = pos + (orient*curBobby);
		if (p.x >= min.x && p.x <= max.x &&
			p.z >= min.z && p.z <= max.z &&
			p.y >= min.y && p.y <= max.y + bobby_rad)
		{
			if (p.y <= max.y - bobby_rad)
			{
				center += p;
				total_volume += bobby_volume;
			}
			else
			{
				float D = max.y - (p.y - bobby_rad);
				float D2 = D*D;
				total_volume += HIGHOMEGA_PI *((bobby_rad*D2) - (0.33333f*D2*D));
				center += vec3(p.x, p.y + (-bobby_rad + (D*0.5f)), p.z);
			}
			bobbies_submerged++;
		}
	}
	if (bobbies_submerged > 0)
	{
		center /= (float)bobbies_submerged;
		force = rho*total_volume*Grav*(-GravDir);
		ApplyForce(force, center, dt);
		ApplyForce(-0.5f*rho*A*CD*(lin_v*lin_v)*lin_v.normalized(), pos, dt); // Non-linear drag
		ApplyForce(-b*lin_v, pos, dt); // Linear Drag
									   // Using spring balancing
		vec3 top_glob = pos + vec3(0.0f, radius, 0.0f);
		vec3 top_loc = pos + orient*vec3(0.0f, radius, 0.0f);
		vec3 bot_loc = pos + orient*vec3(0.0f, -radius, 0.0f);
		vec3 spring_force = (top_glob - top_loc) - 0.5*(lin_v + cross(ang_v, top_loc - pos));
		ApplyForce(spring_force, top_loc, dt);
		ApplyForce(-spring_force, bot_loc, dt);
		// If object is spinning around itself, freeze it
		if (cross(ang_v, vec3(0, radius, 0)).length() < 0.01f)
			ang_v *= 0.5;
		if (ang_v.length() + lin_v.length() < 0.1f)
		{
			ang_v = lin_v = vec3(0.0f);
			Frozen = true;
		}
	}
}

void HIGHOMEGA::FIZ_X::RigidBody::ChangeGeom(std::string & groupId, std::vector <TriUV> & triList)
{
	ObjectPiece *oldPiece = getGroupById(groupId);
	if (!oldPiece) return;

	ObjectPiece newPiece;
	GenerateGeom(triList, newPiece);

	newPiece.elasticity = oldPiece->elasticity;
	newPiece.friction = oldPiece->friction;
	newPiece.cuttable = oldPiece->cuttable;
	newPiece.shatterable = oldPiece->shatterable;
	newPiece.strength = oldPiece->strength;
	newPiece.cutRadius = oldPiece->cutRadius;
	newPiece.cutDepth = oldPiece->cutDepth;
	newPiece.radius = oldPiece->radius;
	newPiece.pos = oldPiece->pos;
	newPiece.piece_min = oldPiece->piece_min;
	newPiece.piece_max = oldPiece->piece_max;
	newPiece.mat = oldPiece->mat;
	newPiece.diffName = oldPiece->diffName;
	newPiece.groupId = oldPiece->groupId;

	*oldPiece = newPiece;

	Update();
	BuildBVH();
}

ObjectPiece * HIGHOMEGA::FIZ_X::RigidBody::getGroupById(std::string & groupId)
{
	for (unsigned int i = 0; i != pieces.size(); i++)
		if (pieces[i].groupId == groupId)
			return &pieces[i];

	return nullptr;
}

void HIGHOMEGA::FIZ_X::RigidBody::removeGroupById(std::string & groupId)
{
	for (unsigned int i = 0; i != pieces.size(); i++)
		if (pieces[i].groupId == groupId) {
			pieces.erase(pieces.begin() + i);
			BuildBVH();
			return;
		}
}

bool HIGHOMEGA::FIZ_X::RigidBody::IntersectLeaf(vec3& linA, vec3& linB, void* rigidBody, unsigned int primId)
{
	RigidBody& bodyRef = *((RigidBody*)rigidBody);
	unsigned int triRef = *((unsigned int*)&bodyRef.boxLeaves[primId].leafMaxLeafId[3]);
	ObjectPiece& piece = bodyRef.pieces[bodyRef.triRefs[triRef].pieceId];
	ObjectTri& tri = piece.tris[bodyRef.triRefs[triRef].triId];

	vec3 e1, e2, e3;
	mat4 toObjSpace;
	if (!bodyRef.isMap)
	{
		toObjSpace = bodyRef.prevTrans.Inv();
		e1 = toObjSpace * vec3(piece.verts[tri.e1].x, piece.verts[tri.e1].y, piece.verts[tri.e1].z);
		e2 = toObjSpace * vec3(piece.verts[tri.e2].x, piece.verts[tri.e2].y, piece.verts[tri.e2].z);
		e3 = toObjSpace * vec3(piece.verts[tri.e3].x, piece.verts[tri.e3].y, piece.verts[tri.e3].z);
	}
	else
	{
		e1 = vec3(piece.verts[tri.e1].x, piece.verts[tri.e1].y, piece.verts[tri.e1].z);
		e2 = vec3(piece.verts[tri.e2].x, piece.verts[tri.e2].y, piece.verts[tri.e2].z);
		e3 = vec3(piece.verts[tri.e3].x, piece.verts[tri.e3].y, piece.verts[tri.e3].z);
	}
	float k = 1.0f;

	vec3 linDir = linB - linA;
	if (LineSegTri(linA, linDir, e1, e2, e3, k))
	{
		linB = linA + linDir * k;
		return true;
	}
	return false;
}

HIGHOMEGA::FIZ_X::intersection::intersection(vec3 & in_pt, vec3 & in_n, RigidBody * in_a, RigidBody * in_b, float in_elas, float in_fric, MATERIAL in_mat)
{
	pt = in_pt;
	n = in_n;
	a = in_a;
	b = in_b;
	elas = in_elas;
	fric = in_fric;
	m1 = in_mat;
}

HIGHOMEGA::FIZ_X::ClothPiece::ClothPiece()
{
}

HIGHOMEGA::FIZ_X::ClothPiece::ClothPiece(GraphicsModel *inpClothModel, RigidBody *inpRigidBodyRef, DataGroup *inpDataGroupRef)
{
	clothModel = inpClothModel;
	rigidBodyRef = inpRigidBodyRef;
	DataBlock *triBlockTmp;
	if ( !Mesh::getDataBlock(*inpDataGroupRef, "TRIS", &triBlockTmp) ) FATAL_ERROR("Error fetching blocks for cloth");
	triBlock = *triBlockTmp;
}

// https://iquilezles.org/articles/distfunctions/
float HIGHOMEGA::FIZ_X::ClothCollectionClass::sdCappedCylinder(const vec3& p, const vec3& a, const vec3& b, float r)
{
	vec3 ba = b - a;
	vec3 pa = p - a;
	float baba = ba * ba;
	float paba = pa * ba;
	float x = (pa * baba - ba * paba).length() - r * baba;
	float y = fabs(paba - baba * 0.5f) - baba * 0.5f;
	float x2 = x * x;
	float y2 = y * y * baba;
	float d = (max(x, y) < 0.0f) ? -min(x2, y2) : (((x > 0.0f) ? x2 : 0.0f) + ((y > 0.0f) ? y2 : 0.0f));
	return sign(d) * sqrtf(fabs(d)) / baba;
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::ClothLoop(ClothCollectionClass* clothCollection)
{
	vec3 curCol, toHit;
	vec2 curUV;
	vec3 curPos, p1, p2, p3;
	vec3 curNorm;

	vec3 diffVec;
	float diffVecLen;
	vec3 diffVecNorm;
	vec3 forceCompute;

	vec3 normTotal;
	float normCount;
	vec3 rvNorm;
	vec3 clothMinTmp, clothMaxTmp;
	std::vector<vec3> windForces;
	vec3 centMass(0.0f);

	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		CommonSharedMutex.lock_shared();
		clothCollection->clothAccessMutex.lock_shared();
		try
		{
			for (std::pair<const unsigned long long, ClothPiece>& curCloth : clothCollection->clothItems)
			{
				std::vector<ClothPiece::RayHit> rayHitsLocalCopy;
				{ std::lock_guard<std::mutex> lk(clothCollection->rayHitMutex); 
				rayHitsLocalCopy = curCloth.second.rayHits; curCloth.second.rayHits.clear(); }
				ClothPiece& curClothPiece = curCloth.second;
				vec3 curClothCent = (curCloth.second.clothMin + curCloth.second.clothMax) * 0.5f;
				float clothRad = (curClothCent - curCloth.second.clothMin).length();
				if (!curClothPiece.unaffectedByWind)
				{
					clothCollection->windSourceMutex.lock_shared();
					for (std::pair<const unsigned long long, WindSource>& curWindSource : clothCollection->windSources)
					{
						if (sdCappedCylinder(curClothCent, curWindSource.second.pos, curWindSource.second.pos + curWindSource.second.dist * curWindSource.second.dir, curWindSource.second.rad + clothRad) >= 0.0f) continue;
						windForces.emplace_back(curWindSource.second.dir * curWindSource.second.strength * ((rand() % 1000 - 900) > 0 ? 1.0f : 0.0f) * (float)(rand() % 1000) * 0.001f);
					}
					clothCollection->windSourceMutex.unlock_shared();
				}
				if (!curClothPiece.rigidBodyRef->isMap)
				{
					bool resetPos = false;
					mat4 trans = curClothPiece.rigidBodyRef->orient;
					trans.i[0][3] = curClothPiece.rigidBodyRef->pos.x;
					trans.i[1][3] = curClothPiece.rigidBodyRef->pos.y;
					trans.i[2][3] = curClothPiece.rigidBodyRef->pos.z;
					for (ClothPiece::PointMass& curPtMass : curClothPiece.pts)
					{
						if (!curPtMass.fixed) continue;
						if ((trans * curPtMass.origPos - curPtMass.pos).length() > curClothPiece.snapClothBackToBodyThreshold)
							resetPos = true;
						else
							resetPos = false;
						break;
					}
					for (ClothPiece::PointMass& curPtMass : curClothPiece.pts)
						if (resetPos || (!resetPos && curPtMass.fixed))
							curPtMass.pos = trans * curPtMass.origPos;
				}
				for (ClothPiece::PointMass& curPt : curClothPiece.pts)
				{
					if (curPt.fixed) continue;
					for (ClothPiece::NeighborInfo& curNeigh : curPt.neighbors)
					{
						ClothPiece::PointMass& neighPt = curClothPiece.pts[curNeigh.index];
						diffVec = neighPt.pos - curPt.pos;
						diffVecLen = diffVec.length();
						diffVecNorm = diffVec / diffVecLen;
						forceCompute = (diffVecLen - curNeigh.restLen) * diffVecNorm * curClothPiece.k - curPt.vel * curClothPiece.b;
						forceCompute += GravDir * Grav;
						for (vec3& curForce : windForces)
							forceCompute += curForce;

						curPt.prevAccel = curPt.accel;
						curPt.accel = forceCompute / curClothPiece.m;

						curPt.prevVel = curPt.vel;
						curPt.vel += (curPt.accel + curPt.prevAccel) * (DeltaT * 0.5f);
					}
				}
				windForces.clear();
				for (ClothPiece::PointMass& curPt : curClothPiece.pts)
				{
					if (curPt.fixed) continue;
					curPt.prevPos = curPt.pos;
					for (ClothPiece::RayHit& rayHit : rayHitsLocalCopy)
					{
						toHit = rayHit.pt - curPt.pos;
						vec3 ptDisplacement = Clamp((1.0f - ((toHit * toHit) * rayHit.invInflRadSq)), 0.0f, 1.0f) * rayHit.dir;
						curPt.pos += ptDisplacement;
						curPt.vel += ptDisplacement * 0.5f;
					}
					allObstaclesMutex.lock_shared();
					for (std::pair<const unsigned long long, PlayerClothObstacles>& curObst : allObstacles)
						for (unsigned int i = 0; i != 3; i++)
						{
							diffVec = curPt.pos - curObst.second.pos[i];
							float diffVecLen = diffVec.length();
							float sphereRad = curObst.second.rad;
							if (diffVecLen < sphereRad)
								curPt.pos += (sphereRad - diffVecLen) * diffVec.normalized();
						}
					allObstaclesMutex.unlock_shared();
				}
				if (curClothPiece.pressureSoftBody)
				{
					centMass = vec3(0.0f);
					for (ClothPiece::PointMass& curPt : curClothPiece.pts)
						centMass += curPt.pos;
					centMass /= (float)curClothPiece.pts.size();
					for (ClothPiece::PointMass& curPt : curClothPiece.pts)
					{
						if (curPt.fixed) continue;
						vec3 linB = curPt.pos;
						bool somethingHit = false;
						ObjectPiece* curPiece = nullptr;
						for (RigidBody* curBody : allBodies)
							somethingHit = somethingHit || LineMeshClosest(*curBody, centMass, linB, curNorm, &curPiece);
						if (somethingHit)
						{
							curPt.pos -= ((curPt.pos - linB) * curNorm) * curNorm;
							float projVel = curPt.vel * curNorm;
							if (projVel < 0.0f) curPt.vel -= projVel * (1.0f + curPiece->elasticity) * curNorm;
						}
					}
				}
				clothMinTmp = vec3(FLT_MAX);
				clothMaxTmp = vec3(-FLT_MAX);
				if (curClothPiece.suddenDisplacement != vec3(0.0))
				{
					for (ClothPiece::PointMass& curPt : curClothPiece.pts)
						if (!curPt.fixed) curPt.pos += curClothPiece.suddenDisplacement;
					curClothPiece.suddenDisplacement = vec3(0.0f);
				}
				for (ClothPiece::PointMass& curPt : curClothPiece.pts)
				{
					if (!curPt.fixed) curPt.pos += curPt.vel * DeltaT + curPt.accel * (DeltaT * DeltaT * 0.5f);
					clothMinTmp = vec3(min(clothMinTmp.x, curPt.pos.x), min(clothMinTmp.y, curPt.pos.y), min(clothMinTmp.z, curPt.pos.z));
					clothMaxTmp = vec3(max(clothMaxTmp.x, curPt.pos.x), max(clothMaxTmp.y, curPt.pos.y), max(clothMaxTmp.z, curPt.pos.z));
				}
				curClothPiece.clothMin = clothMinTmp;
				curClothPiece.clothMax = clothMaxTmp;

				RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
				getIndicesVertices(curClothPiece.triBlock.blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

				for (ClothPiece::PointMass& curPt : curClothPiece.pts)
				{
					normTotal = vec3(0.0f);
					normCount = 0.0f;
					for (unsigned int i = 0; i != curPt.neighbors.size(); i++)
					{
						if (!curPt.neighbors[i].sharetri) continue; // Diagonal springs won't make a renderable triangle... so they shouldn't contribute normal info

						ClothPiece::PointMass& neigh1Pt = curClothPiece.pts[curPt.neighbors[i].index];
						ClothPiece::PointMass& neigh2Pt = curClothPiece.pts[curPt.neighbors[i + 1].index];

						normTotal += cross(neigh1Pt.pos - curPt.pos, neigh2Pt.pos - curPt.pos).normalized();
						normCount += 1.0f;

						i++; // Skip over the following neighbor
					}
					curPt.norm = (normTotal / normCount).normalized();
					if (!curPt.flipNorm)
					{
						RasterVertex& rv = verts[*curPt.vertIndex.begin()];
						rvNorm = fromZSignXY(rv.Norm);
						if (rvNorm * curPt.norm < 0.0f)
						{
							curPt.norm = -curPt.norm;
							curPt.flipNorm = -1;
						}
						else
							curPt.flipNorm = 1;
					}
					else
						curPt.norm = sign((float)curPt.flipNorm) * curPt.norm;
					for (unsigned int i : curPt.vertIndex)
					{
						RasterVertex& readWriteRV = verts[i];
						unpackRasterVertex(curPos, curCol, curUV, curNorm, readWriteRV);
						packRasterVertex(curPt.pos, packVertexVelocity(curPt.fixed ? vec3 (0.0f) : (curPt.pos - curPt.prevPos), readWriteRV.posCol[3]), curUV, curPt.norm, readWriteRV);
					}
				}

				if (curClothPiece.pressureSoftBody)
				{
					float volumeEstimate = 0.0f;
					for (unsigned int i = 0; i != triCount; i++)
					{
						unpackRasterVertex(curPos, curCol, curUV, rvNorm, verts[indices[i * 3]]);
						ClothPiece::PointMass& ptM1 = curClothPiece.pts[curClothPiece.vertIndexToPtMass[indices[i * 3]]];
						ClothPiece::PointMass& ptM2 = curClothPiece.pts[curClothPiece.vertIndexToPtMass[indices[i * 3 + 1]]];
						ClothPiece::PointMass& ptM3 = curClothPiece.pts[curClothPiece.vertIndexToPtMass[indices[i * 3 + 2]]];
						p1 = ptM1.pos;
						p2 = ptM2.pos;
						p3 = ptM3.pos;
						vec3 triCross = cross(p1 - p2, p3 - p2);
						float triCrossLen = triCross.length();
						if (triCrossLen == 0.0f) continue;
						curNorm = triCross / triCrossLen;
						if (curNorm * rvNorm < 0.0f) curNorm = -curNorm;
						bool subtractVol = false;
						if ((centMass - p1) * curNorm > 0.0f) subtractVol = true;
						float pyramidArea = 0.3333333f * 0.5f * triCrossLen * fabs((centMass - p1) * curNorm);
						volumeEstimate += subtractVol ? (-pyramidArea) : pyramidArea;
					}
					volumeEstimate = fabs(volumeEstimate); // Just making sure
					float pressure = curClothPiece.nRT / volumeEstimate;
					for (unsigned int i = 0; i != triCount; i++)
					{
						ClothPiece::PointMass& ptM1 = curClothPiece.pts[curClothPiece.vertIndexToPtMass[indices[i * 3]]];
						ClothPiece::PointMass& ptM2 = curClothPiece.pts[curClothPiece.vertIndexToPtMass[indices[i * 3 + 1]]];
						ClothPiece::PointMass& ptM3 = curClothPiece.pts[curClothPiece.vertIndexToPtMass[indices[i * 3 + 2]]];
						p1 = ptM1.pos;
						p2 = ptM2.pos;
						p3 = ptM3.pos;
						float triArea = cross(p1 - p2, p3 - p2).length() * 0.5f;
						float pressureForce = (pressure / triArea) * 0.3333333f;
						ptM1.vel += ptM1.norm * pressureForce;
						ptM2.vel += ptM2.norm * pressureForce;
						ptM3.vel += ptM3.norm * pressureForce;
					}
				}
			}
			clothCollection->allMinMaxesComputed = true;
		}
		catch (std::exception e)
		{
			clothCollection->clothAccessMutex.unlock_shared();
			CommonSharedMutex.unlock_shared();
			return;
		}
		clothCollection->clothAccessMutex.unlock_shared();
		CommonSharedMutex.unlock_shared();

		double timerDiff = 10.0 - timerObj.Diff() * 1000.0;
		if (timerDiff > 0.0f) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{ std::unique_lock<std::mutex> lk(clothCollection->clothSignal.quit_mutex); readQuit = clothCollection->clothSignal.quit; }
		if (readQuit) return;
	}
}

unsigned long long HIGHOMEGA::FIZ_X::ClothCollectionClass::Add(Mesh & inpMesh, std::string belong, RigidBody *inpRigidBody, mat4 inpOrient, vec3 inpPos, int inpGroupId, DataGroup & inpPolyGroup)
{
	ClothPiece newCloth(new GraphicsModel(inpMesh, belong, Instance, [&inpGroupId = inpGroupId](int groupId, DataGroup& inpPolyGroup) -> bool {
		if (groupId == inpGroupId)
			return true;
		else
			return false;
	}, false), inpRigidBody, &inpPolyGroup);

	float outTmp;
	newCloth.pressureSoftBody = Mesh::getDataRowFloat(inpPolyGroup, "PROPS", "pressureSoftBody", outTmp);
	if (!Mesh::getDataRowFloat(inpPolyGroup, "PROPS", "pressureSoftBodyMolesRTemperature", newCloth.nRT)) newCloth.nRT = 1200000.0f;
	if (!Mesh::getDataRowFloat(inpPolyGroup, "PROPS", "springPointMass", newCloth.m)) newCloth.m = 70.0f;
	if (!Mesh::getDataRowFloat(inpPolyGroup, "PROPS", "springConstant", newCloth.k)) newCloth.k = 100.0f;
	if (!Mesh::getDataRowFloat(inpPolyGroup, "PROPS", "springDrag", newCloth.b)) newCloth.b = 1.0f;
	newCloth.unaffectedByWind = Mesh::getDataRowFloat(inpPolyGroup, "PROPS", "unaffectedByWind", outTmp);

	DataBlock *triBlock = &newCloth.triBlock;

	mat4 trans = inpOrient;
	trans.i[0][3] = inpPos.x;
	trans.i[1][3] = inpPos.y;
	trans.i[2][3] = inpPos.z;

	vec3 pos1, pos2, pos3, col1, col2, col3, vNorm1, vNorm2, vNorm3;
	vec2 uv1, uv2, uv3;
	unsigned int i1, i2, i3;

	RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
	getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

	if (newCloth.pressureSoftBody) newCloth.vertIndexToPtMass.resize(vertCount);

	for (unsigned int i = 0; i != triCount; i++)
	{
		ClothPiece::PointMass pMass;
		pMass.accel = pMass.vel = vec3(0.0f);

		unsigned int vertIndex1 = i * 3;
		unsigned int vertIndex2 = vertIndex1 + 1;
		unsigned int vertIndex3 = vertIndex1 + 2;

		unpackRasterVertex(pos1, col1, uv1, vNorm1, verts[indices[vertIndex1]]);
		unpackRasterVertex(pos2, col2, uv2, vNorm2, verts[indices[vertIndex2]]);
		unpackRasterVertex(pos3, col3, uv3, vNorm3, verts[indices[vertIndex3]]);

		bool foundI1 = false, foundI2 = false, foundI3 = false;
		for (unsigned int j = 0; j != newCloth.pts.size(); j++)
		{
			if (WithinAABB(newCloth.pts[j].origPos, pos1, 0.01f))
			{
				i1 = j;
				foundI1 = true;
				break;
			}
		}
		if (!foundI1)
		{
			pMass.origPos = pos1;
			pMass.pos = trans * pMass.origPos;
			pMass.fixed = (col1 == vec3(0.0f, 1.0f, 0.0f));
			newCloth.pts.push_back(pMass);
			i1 = (unsigned int)newCloth.pts.size() - 1;
		}
		for (unsigned int j = 0; j != newCloth.pts.size(); j++)
		{
			if (WithinAABB(newCloth.pts[j].origPos, pos2, 0.01f))
			{
				i2 = j;
				foundI2 = true;
				break;
			}
		}
		if (!foundI2)
		{
			pMass.origPos = pos2;
			pMass.pos = trans * pMass.origPos;
			pMass.fixed = (col2 == vec3(0.0f, 1.0f, 0.0f));
			newCloth.pts.push_back(pMass);
			i2 = (unsigned int)newCloth.pts.size() - 1;
		}
		for (unsigned int j = 0; j != newCloth.pts.size(); j++)
		{
			if (WithinAABB(newCloth.pts[j].origPos, pos3, 0.01f))
			{
				i3 = j;
				foundI3 = true;
				break;
			}
		}
		if (!foundI3)
		{
			pMass.origPos = pos3;
			pMass.pos = trans * pMass.origPos;
			pMass.fixed = (col3 == vec3(0.0f, 1.0f, 0.0f));
			newCloth.pts.push_back(pMass);
			i3 = (unsigned int)newCloth.pts.size() - 1;
		}
		newCloth.pts[i1].vertIndex.insert(indices[vertIndex1]);
		newCloth.pts[i2].vertIndex.insert(indices[vertIndex2]);
		newCloth.pts[i3].vertIndex.insert(indices[vertIndex3]);
		if (newCloth.pressureSoftBody)
		{
			newCloth.vertIndexToPtMass[indices[vertIndex1]] = i1;
			newCloth.vertIndexToPtMass[indices[vertIndex2]] = i2;
			newCloth.vertIndexToPtMass[indices[vertIndex3]] = i3;
		}
		ClothPiece::NeighborInfo neighborInf;
		neighborInf.restLen = 0.0f;

		neighborInf.index = i2;
		newCloth.pts[i1].neighbors.push_back(neighborInf);
		neighborInf.index = i3;
		newCloth.pts[i1].neighbors.push_back(neighborInf);

		neighborInf.index = i3;
		newCloth.pts[i2].neighbors.push_back(neighborInf);
		neighborInf.index = i1;
		newCloth.pts[i2].neighbors.push_back(neighborInf);

		neighborInf.index = i1;
		newCloth.pts[i3].neighbors.push_back(neighborInf);
		neighborInf.index = i2;
		newCloth.pts[i3].neighbors.push_back(neighborInf);

		unsigned commonIndex1, commonIndex2, uncommonIndex;
		bool addDiagonalRefs = false;
		if (foundI1 && foundI2 && !foundI3)
		{
			commonIndex1 = i1;
			commonIndex2 = i2;
			uncommonIndex = i3;
			addDiagonalRefs = true;
		}
		if (!foundI1 && foundI2 && foundI3)
		{
			commonIndex1 = i2;
			commonIndex2 = i3;
			uncommonIndex = i1;
			addDiagonalRefs = true;
		}
		if (foundI1 && !foundI2 && foundI3)
		{
			commonIndex1 = i1;
			commonIndex2 = i3;
			uncommonIndex = i2;
			addDiagonalRefs = true;
		}

		if (addDiagonalRefs)
		{
			float commonSideLen = (newCloth.pts[commonIndex1].pos - newCloth.pts[commonIndex2].pos).length();
			bool addedDiagonal = false;
			for (unsigned int j = 0; j != newCloth.pts[commonIndex1].neighbors.size(); j++)
			{
				if (addedDiagonal) break;
				for (unsigned int k = 0; k != newCloth.pts[commonIndex2].neighbors.size(); k++)
					if (newCloth.pts[commonIndex1].neighbors[j].index == newCloth.pts[commonIndex2].neighbors[k].index)
					{
						unsigned int diagonalIndex = newCloth.pts[commonIndex1].neighbors[j].index;
						if (fabs((newCloth.pts[diagonalIndex].pos - newCloth.pts[uncommonIndex].pos).length() - commonSideLen) < 0.01f)
						{
							newCloth.pts[uncommonIndex].neighbors.emplace_back(diagonalIndex, 0.0f, false);

							newCloth.pts[diagonalIndex].neighbors.emplace_back(uncommonIndex, 0.0f, false);

							addedDiagonal = true;
							break;
						}
					}
			}
		}
	}

	for (ClothPiece::PointMass& curPtMass : newCloth.pts)
	{
		if (curPtMass.neighbors.size() == 0) FATAL_ERROR("A point mass in added cloth piece did not have neighbors");
		for (int j = 0; j != curPtMass.neighbors.size(); j++)
			curPtMass.neighbors[j].restLen = (newCloth.pts[curPtMass.neighbors[j].index].pos - curPtMass.pos).length();
	}
	unsigned long long addedClothId = threadSafeMersenneTwister64Bit();

	clothAccessMutex.lock();
	clothItems[addedClothId] = newCloth;
	clothAccessMutex.unlock();

	return addedClothId;
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::GetMinMax(unsigned long long clothId, vec3& outMin, vec3& outMax)
{
	clothAccessMutex.lock_shared();
	if (clothItems.find(clothId) != clothItems.end())
	{
		outMin = clothItems[clothId].clothMin;
		outMax = clothItems[clothId].clothMax;
	}
	clothAccessMutex.unlock_shared();
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::AddSuddenDisplacement(unsigned long long clothId, const vec3& inSuddenDisp)
{
	clothAccessMutex.lock_shared();
	if (clothItems.find(clothId) != clothItems.end())
		clothItems[clothId].suddenDisplacement += inSuddenDisp;
	clothAccessMutex.unlock_shared();
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::AddImpulse(unsigned long long clothId, const vec3& inImpulse)
{
	clothAccessMutex.lock_shared();
	if (clothItems.find(clothId) != clothItems.end())
		for (ClothPiece::PointMass& curPt : clothItems[clothId].pts)
			if (!curPt.fixed) curPt.vel += (inImpulse / clothItems[clothId].m);
	clothAccessMutex.unlock_shared();
}

unsigned long long HIGHOMEGA::FIZ_X::ClothCollectionClass::AddWindSources(HIGHOMEGA::MESH::DataGroup& curPolyGroup)
{
	WindSource curSource;
	float tmpFloat;

	if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "windSource", tmpFloat)) FATAL_ERROR("Something went horribly wrong. Adding a windSource with no strength.");

	if (!Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", curSource.pos)) FATAL_ERROR("A windSource is erroneous, couldn't find pos.");
	if (!Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", curSource.dir)) FATAL_ERROR("A windSource is erroneous, couldn't find dir.");
	if (!Mesh::getDataRowFloat(curPolyGroup, "DESCRIPTION", "rad", curSource.rad)) FATAL_ERROR("A windSource is erroneous, couldn't find rad.");
	if (!Mesh::getDataRowFloat(curPolyGroup, "DESCRIPTION", "dist", curSource.dist)) FATAL_ERROR("A windSource is erroneous, couldn't find dist.");
	if (!Mesh::getDataRowFloat(curPolyGroup, "DESCRIPTION", "enrg", curSource.strength)) FATAL_ERROR("A windSource is erroneous, couldn't find energy.");

	unsigned long long addedId = threadSafeMersenneTwister64Bit();

	windSourceMutex.lock();
	windSources[addedId] = curSource;
	windSourceMutex.unlock();

	return addedId;
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::Remove(unsigned long long clothOrSourceId)
{
	windSourceMutex.lock_shared();
	bool isWindSource = !(windSources.find(clothOrSourceId) == windSources.end());
	windSourceMutex.unlock_shared();
	clothAccessMutex.lock_shared();
	bool isCloth = !(clothItems.find(clothOrSourceId) == clothItems.end());
	clothAccessMutex.unlock_shared();

	if (isWindSource)
	{
		windSourceMutex.lock();
		windSources.erase(clothOrSourceId);
		windSourceMutex.unlock();
	}
	else if (isCloth)
	{
		delete clothItems[clothOrSourceId].clothModel;

		clothAccessMutex.lock();
		clothItems.erase(clothOrSourceId);
		clothAccessMutex.unlock();
	}
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::Combine(ClothCollectionClass& clothCollection, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall)
{
	subList = submissionList;
	perSubmissionCall = inpPerSubmissionCall;

	clothAccessMutex.lock();
	clothItems.insert(clothCollection.clothItems.begin(), clothCollection.clothItems.end());
	clothAccessMutex.unlock();

	windSourceMutex.lock();
	windSources.insert(clothCollection.windSources.begin(), clothCollection.windSources.end());
	windSourceMutex.unlock();

	clothCollection.clothItems.clear();
	clothCollection.windSources.clear();
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::Update()
{
	if (!simulationThread)
		simulationThread = new std::thread(ClothLoop, this);

	clothAccessMutex.lock_shared();
	for (std::pair<const unsigned long long, ClothPiece> & curCloth : clothItems)
	{
		if (!curCloth.second.clothModelInst && subList.size() > 0)
		{
			curCloth.second.clothModelInst = curCloth.second.clothModel->CreateInstance();
			for (GroupedRenderSubmission* curSubmission : subList)
				perSubmissionCall(curSubmission, curCloth.second.clothModelInst);
		}
		curCloth.second.clothModel->MaterialGeomMap.begin()->second.begin()->Update(curCloth.second.triBlock.blob);
		if (curCloth.second.clothModelInst) curCloth.second.clothModelInst->SetMinMax(curCloth.second.clothMin, curCloth.second.clothMax);
		curCloth.second.clothModel->SetDirty();
	}
	clothAccessMutex.unlock_shared();
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::IntersectRay(vec3& linA, vec3& linB)
{
	if (allMinMaxesComputed)
	{
		vec3 linDir = linB - linA;
		vec3 invM = linDir.inv();
		vec3 displaceVec = linDir.normalized() * displaceStrength;
		float invInflRadSq = 1.0f / (displaceStrength * displaceStrength * 4.0f);
		clothAccessMutex.lock_shared();
		for (std::pair<const unsigned long long, ClothPiece>& curClothPair : clothItems)
		{
			if (!HIGHOMEGA::GEOM::SlabTest(linA, invM, curClothPair.second.clothMin, curClothPair.second.clothMax)) continue;
			RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
			getIndicesVertices(curClothPair.second.triBlock.blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

			unsigned int i1, i2, i3;
			vec3 p1, p2, p3, tmpCol, tmpNorm;
			vec2 tmpUV;
			vec3 triMin, triMax;

			float k;
			for (unsigned int i = 0; i != triCount; i++)
			{
				i1 = indices[i * 3];
				i2 = indices[i * 3 + 1];
				i3 = indices[i * 3 + 2];

				unpackRasterVertex(p1, tmpCol, tmpUV, tmpNorm, verts[i1]);
				unpackRasterVertex(p2, tmpCol, tmpUV, tmpNorm, verts[i2]);
				unpackRasterVertex(p3, tmpCol, tmpUV, tmpNorm, verts[i3]);

				triMin.x = Min(p1.x, p2.x, p3.x);
				triMin.y = Min(p1.y, p2.y, p3.y);
				triMin.z = Min(p1.z, p2.z, p3.z);
				triMax.x = Max(p1.x, p2.x, p3.x);
				triMax.y = Max(p1.y, p2.y, p3.y);
				triMax.z = Max(p1.z, p2.z, p3.z);

				if (!SlabTest(linA, invM, triMin, triMax)) continue;

				k = 1.0f;
				if (LineSegTri(linA, linDir, p1, p2, p3, k))
				{
					vec3 hitPos = linA + linDir * k;
					{
						std::lock_guard<std::mutex> lk(rayHitMutex);
						curClothPair.second.rayHits.emplace_back(hitPos, displaceVec, invInflRadSq);
					}
					allTriHits.emplace_back(hitPos, cross(p1 - p2, p3 - p2).normalized());
				}
			}
		}
		clothAccessMutex.unlock_shared();
	}
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::ClearContent()
{
	if (simulationThread)
	{
		{ std::unique_lock <std::mutex> lk(clothSignal.quit_mutex); clothSignal.quit = true; }
		simulationThread->join();
		delete simulationThread;
		simulationThread = nullptr;
	}
	for (std::pair<const unsigned long long, ClothPiece> & curCloth : clothItems) {
		delete curCloth.second.clothModel;
	}
	clothItems.clear();
	windSources.clear();
	subList.clear();
}

unsigned long long HIGHOMEGA::FIZ_X::BuoyancyRangeCollectionClass::AddRange(DataGroup & inpPolyGroup)
{
	vec3 minRange, maxRange;

	DataBlock *triBlock;
	if ( !Mesh::getDataBlock(inpPolyGroup, "TRIS", &triBlock) ) FATAL_ERROR("Error fetching buoyancy range vert block");

	RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
	getIndicesVertices(triBlock->blob, &verts, vertCount, &indices, triCount, vertexDataOffset);

	vec3 curPos;
	vec3 curCol;
	vec2 curUV;
	vec3 curNorm;
	for (int i = 0; i != vertCount; i++)
	{
		unpackRasterVertex(curPos, curCol, curUV, curNorm, verts[i]);
		if (i == 0)
		{
			maxRange = minRange = curPos;
		}
		else
		{
			minRange = vec3(min(minRange.x, curPos.x), min(minRange.y, curPos.y), min(minRange.z, curPos.z));
			maxRange = vec3(max(maxRange.x, curPos.x), max(maxRange.y, curPos.y), max(maxRange.z, curPos.z));
		}
	}

	BuoyancyRange curRange;
	curRange.minRange = minRange;
	curRange.maxRange = maxRange;

	unsigned long long itemId = threadSafeMersenneTwister64Bit();
	ranges[itemId] = curRange;

	return itemId;
}

void HIGHOMEGA::FIZ_X::BuoyancyRangeCollectionClass::Combine(BuoyancyRangeCollectionClass& buoyancyRangeCollection)
{
	ranges.insert(buoyancyRangeCollection.ranges.begin(), buoyancyRangeCollection.ranges.end());
	buoyancyRangeCollection.ranges.clear();
}

void HIGHOMEGA::FIZ_X::BuoyancyRangeCollectionClass::Remove(unsigned long long rangeId)
{
	ranges.erase(rangeId);
}

void HIGHOMEGA::FIZ_X::BuoyancyRangeCollectionClass::ClearContent()
{
	ranges.clear();
}

void HIGHOMEGA::FIZ_X::PiecePiece(ObjectPiece & a, ObjectPiece & b, RigidBody * aBody, RigidBody * bBody, std::vector<intersection>& intersect_list, bool quitOnIntersection)
{
	bool a_breakable = a.shatterable || a.cuttable;
	bool b_breakable = b.shatterable || b.cuttable;
	if (a_breakable == a_breakable && b_breakable) return; // This is usually a lot of these pieces getting stuck into eachother... not worth it

	vec3 t1[3], t2[3], n1_v, n2_v;
	vec3 pt1, pt2;
	TRITRI_INTERSECT_MODE mode;

	for (ObjectTri& aTri : a.tris)
	{
		float aTriMinX = Min(a.verts[aTri.e1].x, a.verts[aTri.e2].x, a.verts[aTri.e3].x);
		float aTriMinY = Min(a.verts[aTri.e1].y, a.verts[aTri.e2].y, a.verts[aTri.e3].y);
		float aTriMinZ = Min(a.verts[aTri.e1].z, a.verts[aTri.e2].z, a.verts[aTri.e3].z);
		float aTriMaxX = Max(a.verts[aTri.e1].x, a.verts[aTri.e2].x, a.verts[aTri.e3].x);
		float aTriMaxY = Max(a.verts[aTri.e1].y, a.verts[aTri.e2].y, a.verts[aTri.e3].y);
		float aTriMaxZ = Max(a.verts[aTri.e1].z, a.verts[aTri.e2].z, a.verts[aTri.e3].z);
		for (ObjectTri& bTri : b.tris)
		{
			float bTriMinX = Min(b.verts[bTri.e1].x, b.verts[bTri.e2].x, b.verts[bTri.e3].x);
			float bTriMinY = Min(b.verts[bTri.e1].y, b.verts[bTri.e2].y, b.verts[bTri.e3].y);
			float bTriMinZ = Min(b.verts[bTri.e1].z, b.verts[bTri.e2].z, b.verts[bTri.e3].z);
			float bTriMaxX = Max(b.verts[bTri.e1].x, b.verts[bTri.e2].x, b.verts[bTri.e3].x);
			float bTriMaxY = Max(b.verts[bTri.e1].y, b.verts[bTri.e2].y, b.verts[bTri.e3].y);
			float bTriMaxZ = Max(b.verts[bTri.e1].z, b.verts[bTri.e2].z, b.verts[bTri.e3].z);
			if (aTriMinX > bTriMaxX) continue;
			if (aTriMinY > bTriMaxY) continue;
			if (aTriMinZ > bTriMaxZ) continue;
			if (aTriMaxX < bTriMinX) continue;
			if (aTriMaxY < bTriMinY) continue;
			if (aTriMaxZ < bTriMinZ) continue;

			t1[0] = vec3(a.verts[aTri.e1].x, a.verts[aTri.e1].y, a.verts[aTri.e1].z);
			t1[1] = vec3(a.verts[aTri.e2].x, a.verts[aTri.e2].y, a.verts[aTri.e2].z);
			t1[2] = vec3(a.verts[aTri.e3].x, a.verts[aTri.e3].y, a.verts[aTri.e3].z);
			n1_v = cross(t1[0] - t1[1], t1[2] - t1[1]).normalized();
			if (aTri.nFlip) n1_v = -n1_v;

			t2[0] = vec3(b.verts[bTri.e1].x, b.verts[bTri.e1].y, b.verts[bTri.e1].z);
			t2[1] = vec3(b.verts[bTri.e2].x, b.verts[bTri.e2].y, b.verts[bTri.e2].z);
			t2[2] = vec3(b.verts[bTri.e3].x, b.verts[bTri.e3].y, b.verts[bTri.e3].z);
			n2_v = cross(t2[0] - t2[1], t2[2] - t2[1]).normalized();
			if (bTri.nFlip) n2_v = -n2_v;

			if (TriTri(t1, n1_v, t2, n2_v, &pt1, &pt2, &mode))
			{
				float a_coef_elas = a.elasticity;
				float b_coef_elas = b.elasticity;
				float a_coef_fric = a.friction;
				float b_coef_fric = b.friction;
				intersect_list.reserve(((intersect_list.size() / 100) + 1) * 100);
				switch (mode)
				{
				case OVERLAP:
				{
					float avg_elas = (a_coef_elas + b_coef_elas)*0.5f;
					float avg_fric = (a_coef_fric + b_coef_fric)*0.5f;
					vec3 midPt = (pt1 + pt2) * 0.5f;
					vec3 midPtNorm = (pt2 - pt1).normalized();
					intersect_list.emplace_back(midPt, midPtNorm, aBody, bBody, avg_elas, avg_fric, a.mat);
					intersect_list.emplace_back(pt1, n2_v, aBody, bBody, b_coef_elas, b_coef_fric, a.mat);
					intersect_list.emplace_back(pt2, n1_v, bBody, aBody, a_coef_elas, a_coef_fric, b.mat);
					break;
				}
				case T1_INSIDE_T2:
				{
					intersect_list.emplace_back(pt1, n2_v, aBody, bBody, b_coef_elas, b_coef_fric, a.mat);
					intersect_list.emplace_back(pt2, n2_v, aBody, bBody, b_coef_elas, b_coef_fric, b.mat);
					break;
				}
				case T2_INSIDE_T1:
				{
					intersect_list.emplace_back(pt1, n1_v, bBody, aBody, a_coef_elas, a_coef_fric, a.mat);
					intersect_list.emplace_back(pt2, n1_v, bBody, aBody, a_coef_elas, a_coef_fric, b.mat);
					break;
				}
				}
				if (quitOnIntersection) return;
			}
		}
	}
}

void HIGHOMEGA::FIZ_X::CandidatesRigidBody(std::vector<unsigned int>& candidateList, RigidBody* aBody, RigidBody* bBody, std::vector<intersection>& intersect_list, bool quitOnIntersection)
{
	vec3 t1[3], t2[3], n1_v, n2_v;
	vec3 pt1, pt2;
	TRITRI_INTERSECT_MODE mode;

	for (unsigned int curCandidate : candidateList)
	{
		ObjectPiece& a = aBody->pieces[aBody->triRefs[curCandidate].pieceId];
		ObjectTri& aTri = a.tris[aBody->triRefs[curCandidate].triId];
		for (ObjectPiece& b : bBody->pieces)
		{
			bool a_breakable = a.shatterable || a.cuttable;
			bool b_breakable = b.shatterable || b.cuttable;
			if (a_breakable == b_breakable && b_breakable) continue; // This is usually a lot of these pieces getting stuck into eachother... not worth it
			for (ObjectTri& bTri : b.tris)
			{
				t1[0] = vec3(a.verts[aTri.e1].x, a.verts[aTri.e1].y, a.verts[aTri.e1].z);
				t1[1] = vec3(a.verts[aTri.e2].x, a.verts[aTri.e2].y, a.verts[aTri.e2].z);
				t1[2] = vec3(a.verts[aTri.e3].x, a.verts[aTri.e3].y, a.verts[aTri.e3].z);
				n1_v = cross(t1[0] - t1[1], t1[2] - t1[1]).normalized();
				if (aTri.nFlip) n1_v = -n1_v;

				t2[0] = vec3(b.verts[bTri.e1].x, b.verts[bTri.e1].y, b.verts[bTri.e1].z);
				t2[1] = vec3(b.verts[bTri.e2].x, b.verts[bTri.e2].y, b.verts[bTri.e2].z);
				t2[2] = vec3(b.verts[bTri.e3].x, b.verts[bTri.e3].y, b.verts[bTri.e3].z);
				n2_v = cross(t2[0] - t2[1], t2[2] - t2[1]).normalized();
				if (bTri.nFlip) n2_v = -n2_v;

				if (TriTri(t1, n1_v, t2, n2_v, &pt1, &pt2, &mode))
				{
					float a_coef_elas = a.elasticity;
					float b_coef_elas = b.elasticity;
					float a_coef_fric = a.friction;
					float b_coef_fric = b.friction;
					intersect_list.reserve(((intersect_list.size() / 100) + 1) * 100);
					switch (mode)
					{
					case OVERLAP:
					{
						float avg_elas = (a_coef_elas + b_coef_elas) * 0.5f;
						float avg_fric = (a_coef_fric + b_coef_fric) * 0.5f;
						vec3 midPt = (pt1 + pt2) * 0.5f;
						vec3 midPtNorm = (pt2 - pt1).normalized();
						intersect_list.emplace_back(midPt, midPtNorm, aBody, bBody, avg_elas, avg_fric, a.mat);
						intersect_list.emplace_back(pt1, n2_v, aBody, bBody, b_coef_elas, b_coef_fric, a.mat);
						intersect_list.emplace_back(pt2, n1_v, bBody, aBody, a_coef_elas, a_coef_fric, b.mat);
						break;
					}
					case T1_INSIDE_T2:
					{
						intersect_list.emplace_back(pt1, n2_v, aBody, bBody, b_coef_elas, b_coef_fric, a.mat);
						intersect_list.emplace_back(pt2, n2_v, aBody, bBody, b_coef_elas, b_coef_fric, b.mat);
						break;
					}
					case T2_INSIDE_T1:
					{
						intersect_list.emplace_back(pt1, n1_v, bBody, aBody, a_coef_elas, a_coef_fric, a.mat);
						intersect_list.emplace_back(pt2, n1_v, bBody, aBody, a_coef_elas, a_coef_fric, b.mat);
						break;
					}
					}
					if (quitOnIntersection) return;
				}
			}
		}
	}
}

void HIGHOMEGA::FIZ_X::RigidBodyRigidBody(RigidBody & a, RigidBody & b, std::vector <intersection> & intersect_list, bool quitOnIntersection)
{
	if (a.mass == HIGHOMEGA_INFINITE_MASS && b.mass == HIGHOMEGA_INFINITE_MASS) return;
	if (a.body_max.x < b.body_min.x ||
		a.body_min.x > b.body_max.x ||
		a.body_max.y < b.body_min.y ||
		a.body_min.y > b.body_max.y ||
		a.body_max.z < b.body_min.z ||
		a.body_min.z > b.body_max.z)
		return;
	if (a.mass != HIGHOMEGA_INFINITE_MASS && b.mass != HIGHOMEGA_INFINITE_MASS)
	{
		for (ObjectPiece & aPiece : a.pieces)
			for (ObjectPiece & bPiece : b.pieces)
			{
				PiecePiece(aPiece, bPiece, &a, &b, intersect_list, quitOnIntersection);
				if (quitOnIntersection && intersect_list.size() > 0) return;
			}
	}
	else
	{
		RigidBody& nonMovingBody = (a.mass == HIGHOMEGA_INFINITE_MASS) ? a : b;
		RigidBody& movingBody = (a.mass == HIGHOMEGA_INFINITE_MASS) ? b : a;
		thread_local std::vector<unsigned int> bvh8IsectCandidateTris;
		bvh8IsectCandidateTris.clear();
		nonMovingBody.cwbvh.FindBVH8Overlaps(0, nonMovingBody.boxLeaves, movingBody.body_min, movingBody.body_max, bvh8IsectCandidateTris);
		CandidatesRigidBody(bvh8IsectCandidateTris, &nonMovingBody, &movingBody, intersect_list, quitOnIntersection);
	}
}

void HIGHOMEGA::FIZ_X::BoxBodyOverlap(vec3& boxMin, vec3& boxMax, RigidBody& body, std::vector<OverlappingTri>& overlapList)
{
	if (body.body_max.x < boxMin.x ||
		body.body_min.x > boxMax.x ||
		body.body_max.y < boxMin.y ||
		body.body_min.y > boxMax.y ||
		body.body_max.z < boxMin.z ||
		body.body_min.z > boxMax.z)
		return;
	if (body.mass != HIGHOMEGA_INFINITE_MASS)
	{
		for (ObjectPiece& curPiece : body.pieces)
		{
			if (curPiece.piece_max.x < boxMin.x ||
				curPiece.piece_min.x > boxMax.x ||
				curPiece.piece_max.y < boxMin.y ||
				curPiece.piece_min.y > boxMax.y ||
				curPiece.piece_max.z < boxMin.z ||
				curPiece.piece_min.z > boxMax.z)
				continue;
			for (ObjectTri& curTri : curPiece.tris)
			{
				overlapList.emplace_back(
					vec3(curPiece.verts[curTri.e1].x, curPiece.verts[curTri.e1].y, curPiece.verts[curTri.e1].z),
					vec3(curPiece.verts[curTri.e2].x, curPiece.verts[curTri.e2].y, curPiece.verts[curTri.e2].z),
					vec3(curPiece.verts[curTri.e3].x, curPiece.verts[curTri.e3].y, curPiece.verts[curTri.e3].z),
					curTri.nFlip, curPiece.mat);
			}
		}
	}
	else
	{
		thread_local std::vector<unsigned int> bvh8IsectCandidateTris;
		bvh8IsectCandidateTris.clear();
		body.cwbvh.FindBVH8Overlaps(0, body.boxLeaves, boxMin, boxMax, bvh8IsectCandidateTris);
		for (unsigned int curCandidate : bvh8IsectCandidateTris)
		{
			ObjectPiece& candidatePiece = body.pieces[body.triRefs[curCandidate].pieceId];
			ObjectTri& candidateTri = candidatePiece.tris[body.triRefs[curCandidate].triId];
			overlapList.emplace_back(
				vec3(candidatePiece.verts[candidateTri.e1].x, candidatePiece.verts[candidateTri.e1].y, candidatePiece.verts[candidateTri.e1].z),
				vec3(candidatePiece.verts[candidateTri.e2].x, candidatePiece.verts[candidateTri.e2].y, candidatePiece.verts[candidateTri.e2].z),
				vec3(candidatePiece.verts[candidateTri.e3].x, candidatePiece.verts[candidateTri.e3].y, candidatePiece.verts[candidateTri.e3].z),
				candidateTri.nFlip, candidatePiece.mat);
		}
	}
}

bool HIGHOMEGA::FIZ_X::LineMeshClosest(RigidBody& body, vec3& linA, vec3& linB, vec3& closestNorm, ObjectPiece** hitPiece)
{
	bool retVal = false;
	vec3 linDir = linB - linA;
	vec3 invM = linDir.inv();
	if (!body.pieces.size() || !HIGHOMEGA::GEOM::SlabTest(linA, invM, body.body_min, body.body_max)) return retVal;
	unsigned int closestPrimId;
	bool invertedLine = false;
	mat4 totalBodyTrans, totalBodyTransInv;
	if (!body.isMap)
	{
		totalBodyTrans = body.orient;
		totalBodyTrans.i[0][3] = body.pos.x;
		totalBodyTrans.i[1][3] = body.pos.y;
		totalBodyTrans.i[2][3] = body.pos.z;
		totalBodyTransInv = totalBodyTrans.Inv();
		linA = totalBodyTransInv * linA;
		linB = totalBodyTransInv * linB;
		invertedLine = true;
	}
	if (body.cwbvh.LineCWBVH<RigidBody::IntersectLeaf>(linA, linB, (void *)&body, closestPrimId))
	{
		unsigned int closestPrim = *((unsigned int*)&body.boxLeaves[closestPrimId].leafMaxLeafId[3]);
		*hitPiece = &body.pieces[body.triRefs[closestPrim].pieceId];
		ObjectTri& hitTri = (*hitPiece)->tris[body.triRefs[closestPrim].triId];
		vec3 e1, e2, e3;
		mat4 toObjSpace;
		if (invertedLine)
		{
			toObjSpace = body.prevTrans.Inv();
			e1 = toObjSpace * vec3((*hitPiece)->verts[hitTri.e1].x, (*hitPiece)->verts[hitTri.e1].y, (*hitPiece)->verts[hitTri.e1].z);
			e2 = toObjSpace * vec3((*hitPiece)->verts[hitTri.e2].x, (*hitPiece)->verts[hitTri.e2].y, (*hitPiece)->verts[hitTri.e2].z);
			e3 = toObjSpace * vec3((*hitPiece)->verts[hitTri.e3].x, (*hitPiece)->verts[hitTri.e3].y, (*hitPiece)->verts[hitTri.e3].z);
		}
		else
		{
			e1 = vec3((*hitPiece)->verts[hitTri.e1].x, (*hitPiece)->verts[hitTri.e1].y, (*hitPiece)->verts[hitTri.e1].z);
			e2 = vec3((*hitPiece)->verts[hitTri.e2].x, (*hitPiece)->verts[hitTri.e2].y, (*hitPiece)->verts[hitTri.e2].z);
			e3 = vec3((*hitPiece)->verts[hitTri.e3].x, (*hitPiece)->verts[hitTri.e3].y, (*hitPiece)->verts[hitTri.e3].z);
		}
		closestNorm = cross(e1 - e2, e3 - e2).normalized();
		if (hitTri.nFlip) closestNorm = -closestNorm;
		if (invertedLine) closestNorm = (totalBodyTrans.DirectionTransform() * closestNorm).normalized();
		retVal = true;
	}
	if (invertedLine)
	{
		linA = totalBodyTrans * linA;
		linB = totalBodyTrans * linB;
	}
	return retVal;
}

void HIGHOMEGA::FIZ_X::BroadSweep()
{
	collisionMap.resize(allBodies.size());
	for (int i = 0; i != collisionMap.size(); i++)
		collisionMap[i].resize(allBodies.size());

	for (int i = 0; i != allBodies.size(); i++)
		for (int j = 0; j != allBodies.size(); j++)
			if (i == j)
				collisionMap[i][j] = WILL_NOT_COLLIDE;
			else
				collisionMap[i][j] = DO_NOT_KNOW_YET;

	for (int i = 0; i != allBodies.size(); i++)
		for (int j = 0; j != allBodies.size(); j++)
		{
			if (collisionMap[i][j] != DO_NOT_KNOW_YET) continue;
			if (((allBodies[i]->Frozen || allBodies[i]->mass == HIGHOMEGA_INFINITE_MASS) && (allBodies[j]->Frozen && allBodies[j]->mass != HIGHOMEGA_INFINITE_MASS)) ||
			    ((allBodies[j]->Frozen || allBodies[j]->mass == HIGHOMEGA_INFINITE_MASS) && (allBodies[i]->Frozen && allBodies[i]->mass != HIGHOMEGA_INFINITE_MASS)))
				collisionMap[i][j] = collisionMap[j][i] = WILL_NOT_COLLIDE;
		}

	for (int i = 0; i != allBodies.size(); i++)
		for (int j = 0; j != allBodies.size(); j++)
		{
			if (collisionMap[i][j] != DO_NOT_KNOW_YET) continue;

			if (allBodies[i]->body_min.x > allBodies[j]->body_max.x ||
				allBodies[i]->body_min.y > allBodies[j]->body_max.y ||
				allBodies[i]->body_min.z > allBodies[j]->body_max.z ||
				allBodies[i]->body_max.x < allBodies[j]->body_min.x ||
				allBodies[i]->body_max.y < allBodies[j]->body_min.y ||
				allBodies[i]->body_max.z < allBodies[j]->body_min.z)
			{
				collisionMap[i][j] = collisionMap[j][i] = WILL_NOT_COLLIDE;
			}
			else
			{
				collisionMap[i][j] = WILL_COLLIDE;
				collisionMap[j][i] = KNOW_IT_WILL_COLLIDE;
			}
		}
}

void HIGHOMEGA::FIZ_X::ResolveCollisions()
{
	for (int count = 0; count != 50; count++) // Trying to get a plausible behavior by going through all collisions multiple times
	{
		for (unsigned int i = 0; i != collisionNodes.size(); i++)
		{
			intersection & it = collisionNodes[i];
			vec3 relative_v_at_p = (it.a->lin_v + cross(it.a->ang_v, it.pt - it.a->pos)) - (it.b->lin_v + cross(it.b->ang_v, it.pt - it.b->pos));
			float top_side = -(1.0f + it.elas)*it.n*relative_v_at_p;
			float ma_div, mb_div;
			vec3 a_vec_contrib, b_vec_contrib;
			if (it.a->mass == HIGHOMEGA_INFINITE_MASS)
			{
				ma_div = 0.0f;
				a_vec_contrib = vec3(0);
			}
			else
			{
				ma_div = 1.0f / it.a->mass;
				a_vec_contrib = cross(it.a->cur_inertia_tensor_inv*cross(it.pt - it.a->pos, it.n), it.pt - it.a->pos);
			}
			if (it.b->mass == HIGHOMEGA_INFINITE_MASS)
			{
				mb_div = 0.0f;
				b_vec_contrib = vec3(0);
			}
			else
			{
				mb_div = 1.0f / it.b->mass;
				b_vec_contrib = cross(it.b->cur_inertia_tensor_inv*cross(it.pt - it.b->pos, it.n), it.pt - it.b->pos);
			}
			float impulse_mag = top_side / (ma_div + mb_div + (it.n*(a_vec_contrib + b_vec_contrib)));
			if (impulse_mag > 0.0f)
			{
				it.a->ApplyImpulse(impulse_mag*it.n, it.pt);
				it.b->ApplyImpulse(-impulse_mag*it.n, it.pt);
			}
		}
	}
}

void HIGHOMEGA::FIZ_X::PropagateShock()
{
	bool doneFreezing = false;
	while (!doneFreezing)
	{
		doneFreezing = true;
		for (unsigned int i = 0; i != collisionNodes.size(); i++)
		{
			intersection & it = collisionNodes[i];
			if ((it.a->mass == HIGHOMEGA_INFINITE_MASS || it.a->Frozen) && (it.b->mass == HIGHOMEGA_INFINITE_MASS || it.b->Frozen)) continue;
			RigidBody *nonMovingObject, *movingObject;
			if (it.a->mass == HIGHOMEGA_INFINITE_MASS || it.a->Frozen) {
				nonMovingObject = it.a;
				movingObject = it.b;
			}
			else {
				nonMovingObject = it.b;
				movingObject = it.a;
			}
			float kineticEnergy = 0.5f*(movingObject->mass*(movingObject->lin_v*movingObject->lin_v) +
										movingObject->ang_v.x*movingObject->cur_inertia_tensor_inv.i[0][0]*movingObject->cur_inertia_tensor_inv.i[0][0] +
										movingObject->ang_v.y*movingObject->cur_inertia_tensor_inv.i[1][1]*movingObject->cur_inertia_tensor_inv.i[1][1] +
										movingObject->ang_v.z*movingObject->cur_inertia_tensor_inv.i[2][2]*movingObject->cur_inertia_tensor_inv.i[2][2]);
			if (kineticEnergy < 0.001f) {
				movingObject->Frozen = true;
				doneFreezing = false;
				break;
			}
		}
	}
}

void HIGHOMEGA::FIZ_X::ApplyFriction(float integrateAmount)
{
	for (unsigned int i = 0; i != collisionNodes.size(); i++) // Friction
	{
		intersection & it = collisionNodes[i];
		vec3 relative_v_at_p = (it.a->lin_v + cross(it.a->ang_v, it.pt - it.a->pos)) - (it.b->lin_v + cross(it.b->ang_v, it.pt - it.b->pos));
		vec3 fric_dir = cross(it.n, cross(it.n, relative_v_at_p)).normalized();
		if (fric_dir*relative_v_at_p > 0.0) fric_dir = -fric_dir;
		float SampleMass = it.a->mass;
		if (SampleMass == HIGHOMEGA_INFINITE_MASS) SampleMass = it.b->mass;
		fric_dir *= SampleMass;
		if (fric_dir.length() > 0.0001f)
		{
			it.a->ApplyForce(fric_dir*it.fric, it.pt, integrateAmount);
			it.b->ApplyForce(-fric_dir*it.fric, it.pt, integrateAmount);
		}
	}
}

void HIGHOMEGA::FIZ_X::CollideMultiThread (void *threadId)
{
	fizXThreadStruct *inpParam = (fizXThreadStruct *)threadId;

	for (;;)
	{
		inpParam->nodes.clear();

		for (int i = 0; i != allBodies.size();i++)
			for (int j = 0; j != allBodies.size(); j++)
			{
				if (collisionMap[i][j] != WILL_COLLIDE || (i*allBodies.size() + j) % HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT != inpParam->threadIndex) continue;
				RigidBodyRigidBody(*(allBodies[i]), *(allBodies[j]), inpParam->nodes);
			}
		
		{std::unique_lock <std::mutex> lk(inpParam->mutex);
		inpParam->stopped = true;
		inpParam->condVar.wait(lk, [inpParam] { return !inpParam->stopped; });}

		{std::unique_lock <std::mutex> lk(inpParam->quit_mutex);
		if (inpParam->quit) return;}
	}
}

unsigned int HIGHOMEGA::FIZ_X::FindTimeStep()
{
	vec3 top_total_vel = vec3 (0.0f);
	float top_total_vel_len = 0.0f;
	RigidBody *body_with_top_total_vel = nullptr;
	vec3 top_total_vel_point = vec3(0.0f);

	float top_total_rel_vel_len = 0.0f;

	for (RigidBody * curBody : allBodies)
	{
		if (curBody->Frozen || curBody->mass == HIGHOMEGA_INFINITE_MASS) continue;
		vec3 side = cross(curBody->ang_v, curBody->lin_v);
		if (side == vec3(0.0))
		{
			vec3 jitter = vec3((rand() % 100)*0.01f, (rand() % 100)*0.01f, (rand() % 100)*0.01f);
			side = cross(curBody->ang_v, curBody->lin_v + jitter);
		}
		side = side.normalized()*curBody->radius;
		vec3 tan_vel = cross (curBody->ang_v,side);
		if (tan_vel*curBody->lin_v < 0.0f)
		{
			tan_vel = -tan_vel;
			side = -side;
		}
		vec3 max_obj_tot_vel = curBody->lin_v + tan_vel;
		float max_obj_tot_vel_len = max_obj_tot_vel.length();
		if (max_obj_tot_vel_len > top_total_vel_len)
		{
			top_total_vel = max_obj_tot_vel;
			top_total_vel_len = max_obj_tot_vel_len;
			body_with_top_total_vel = curBody;
			top_total_vel_point = curBody->pos + side;
		}
	}

	if (!body_with_top_total_vel) return 1;

	for (RigidBody * curBody : allBodies)
	{
		if (curBody->Frozen || curBody->mass == HIGHOMEGA_INFINITE_MASS || curBody == body_with_top_total_vel) continue;
		vec3 side = cross(curBody->ang_v, curBody->lin_v);
		if (side == vec3(0.0))
		{
			vec3 jitter = vec3((rand() % 100)*0.01f, (rand() % 100)*0.01f, (rand() % 100)*0.01f);
			side = cross(curBody->ang_v, curBody->lin_v + jitter);
		}
		side = side.normalized()*curBody->radius;
		vec3 tan_vel = cross(curBody->ang_v, side);
		if (tan_vel*curBody->lin_v < 0.0f)
		{
			tan_vel = -tan_vel;
			side = -side;
		}
		vec3 _2nd_max_obj_tot_vel = curBody->lin_v + tan_vel;
		vec3 cur_furthest_point = curBody->pos + side;
		float rel_speed_against_top_vel = max(-(_2nd_max_obj_tot_vel - top_total_vel)*(cur_furthest_point - top_total_vel_point).normalized(), 0.0f);
		if (rel_speed_against_top_vel > top_total_rel_vel_len)
			top_total_rel_vel_len = rel_speed_against_top_vel;
	}

	return (unsigned int)max (ceil((top_total_rel_vel_len * DeltaT) / ColTol),1.0f);
}

void HIGHOMEGA::FIZ_X::MoveAll(float delta_t)
{
	for (RigidBody * curBody : allBodies)
	{
		curBody->Move(delta_t);
		curBody->Update();
	}
}

void HIGHOMEGA::FIZ_X::ApplyGravityBuoyancyEnqueuedImpulses(float delta_t)
{
	for (RigidBody * curBody : allBodies)
	{
		curBody->ProcessEnqueuedImpulses();
		if (curBody->Frozen) continue;
		curBody->ApplyForce(curBody->mass*GravDir*Grav, curBody->pos, delta_t);
		for (std::pair<const unsigned long long, BuoyancyRange> & curRange : buoyancyRangeCollection.ranges)
			curBody->ApplyBuoyancy(curRange.second.minRange, curRange.second.maxRange, 0.08f, 0.1f, 25.0f, 0.095f, delta_t);
	}
}

void HIGHOMEGA::FIZ_X::PhysicsLoop()
{
	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		CommonSharedMutex.lock_shared();
		try
		{

			ApplyGravityBuoyancyEnqueuedImpulses(DeltaT);

			float timePassed = 0.0f;
			while (timePassed < DeltaT)
			{
				BroadSweep();
				HIGHOMEGA::FIZ_X::collisionNodes.clear();

				for (int i = 0; i != HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT; i++)
				{
					{std::unique_lock <std::mutex> lk(collisionThreadParams[i].mutex);
					std::unique_lock <std::mutex> lk2(collisionThreadParams[i].quit_mutex);
					collisionThreadParams[i].stopped = collisionThreadParams[i].quit = false;
					collisionThreadParams[i].condVar.notify_all();}
				}

				if (!fizXSignal.started)
				{
					for (int i = 0; i != HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT; i++) {
						collisionThreadParams[i].threadIndex = i;
						collisionThreads.push_back (new std::thread (CollideMultiThread, (void *)&collisionThreadParams[i]));
					}
					fizXSignal.started = true;
				}

				bool everything_done;
				for(;;)
				{
					everything_done = true;
					for (int i = 0; i != HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT; i++) {
						{std::unique_lock <std::mutex> lk(collisionThreadParams[i].mutex);
						if (!collisionThreadParams[i].stopped) {
							everything_done = false;
							break;
						}}
					}
					if (everything_done) break;
				}

				for (int i = 0;i != HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT;i++)
					collisionNodes.insert(collisionNodes.end(), collisionThreadParams[i].nodes.begin(), collisionThreadParams[i].nodes.end());
				ResolveCollisions();
				float integrateAmount = DeltaT / ((float)FindTimeStep());
				MoveAll(integrateAmount);
				ApplyFriction(integrateAmount);
				for (ConstraintCollection * curConstraints : constraintCollection)
					curConstraints->ApplyConstraintForces(integrateAmount);
				for (ConstraintCollection * curConstraints : constraintCollection)
					curConstraints->RandomShit();
				PropagateShock();
				timePassed += integrateAmount;
			}
		}
		catch (...)
		{
			CommonSharedMutex.unlock_shared();
			return;
		}
		CommonSharedMutex.unlock_shared();

		double timerDiff = 10.0 - timerObj.Diff()*1000.0;
		if (timerDiff > 0.0f) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{std::unique_lock <std::mutex> lk(fizXSignal.quit_mutex); readQuit = fizXSignal.quit;}
		if (readQuit) break;
	}

	for (int i = 0; i != HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT; i++) {
		{std::unique_lock <std::mutex> lk(collisionThreadParams[i].mutex);
		std::unique_lock <std::mutex> lk2(collisionThreadParams[i].quit_mutex);
		collisionThreadParams[i].quit = true;
		collisionThreadParams[i].stopped = false;
		collisionThreadParams[i].condVar.notify_all();}
		collisionThreads[i]->join();
		delete collisionThreads[i];
	}
	collisionThreads.clear();
}

HIGHOMEGA::FIZ_X::ConstraintCollection::ConstraintCollection(Mesh & inpMesh, std::unordered_map <std::string, RigidBody *> & inpLimbs, mat4 inpTrans, InstanceClass &ptrToInstance)
{
	allLimbs = inpLimbs;

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		DataGroup &curPolyGroup = inpMesh.DataGroups[i];
		std::string constraintType;

		if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "constraint", constraintType)) continue;

		Constraint curConst;
		if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "parent", curConst.parent) ||
				!Mesh::getDataRowString(curPolyGroup, "PROPS", "child", curConst.child) ) FATAL_ERROR("Error fetching child or parent for constraints");
		vec3 constPos;
		if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", constPos) ) FATAL_ERROR("Error fetching constraint position");
		constPos = inpTrans * constPos;
		float constOrderFloat;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "order", constOrderFloat) ) FATAL_ERROR("Error fetching constraint order");
		curConst.order = (unsigned int)constOrderFloat;
		maxOrder = max(maxOrder, curConst.order);

		mat4 parentTrans, parentTransInv;
		if (curConst.parent == "[WORLD]")
		{
			parentTrans.Ident();
			parentTransInv.Ident();
		}
		else
		{
			parentTrans = allLimbs[curConst.parent]->orient;
			parentTrans.i[0][3] = allLimbs[curConst.parent]->pos.x;
			parentTrans.i[1][3] = allLimbs[curConst.parent]->pos.y;
			parentTrans.i[2][3] = allLimbs[curConst.parent]->pos.z;
			parentTransInv = parentTrans.Inv();
		}
		curConst.posWrtParent = parentTransInv * constPos;

		mat4 childTrans, childTransInv;
		if (curConst.child == "[WORLD]")
		{
			childTrans.Ident();
			childTransInv.Ident();
		}
		else
		{
			childTrans = allLimbs[curConst.child]->orient;
			childTrans.i[0][3] = allLimbs[curConst.child]->pos.x;
			childTrans.i[1][3] = allLimbs[curConst.child]->pos.y;
			childTrans.i[2][3] = allLimbs[curConst.child]->pos.z;
			childTransInv = childTrans.Inv();
		}
		curConst.posWrtChild = childTransInv * constPos;

		if (constraintType == "hinge")
		{
			curConst.type = HINGE;

			vec3 hingeTan;
			if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", hingeTan) ) FATAL_ERROR("Error fetching hinge tangent");
			hingeTan = (inpTrans.DirectionTransform() * hingeTan).normalized();
			curConst.tanPtWrtParent = parentTransInv * (constPos + hingeTan);
			curConst.tanPtWrtChild = childTransInv * (constPos + hingeTan);
		}
		else
		{
			curConst.type = POSITIONAL;
		}

		allConstraints.push_back(curConst);
	}
}

void HIGHOMEGA::FIZ_X::ConstraintCollection::ApplyConstraintForces(float inpDeltaT)
{
	mat4 parentTrans, childTrans;
	vec3 parentTotalVel, childTotalVel;
	bool parentIsWorld, childIsWorld;
	for (Constraint & curConst : allConstraints)
	{
		parentIsWorld = false;
		childIsWorld = false;
		if (curConst.child == "[WORLD]")
		{
			childTrans.Ident();
			childIsWorld = true;
		}
		else
		{
			childTrans = allLimbs[curConst.child]->orient;
			childTrans.i[0][3] = allLimbs[curConst.child]->pos.x;
			childTrans.i[1][3] = allLimbs[curConst.child]->pos.y;
			childTrans.i[2][3] = allLimbs[curConst.child]->pos.z;
		}

		if (curConst.parent == "[WORLD]")
		{
			parentTrans.Ident();
			parentIsWorld = true;
		}
		else
		{
			parentTrans = allLimbs[curConst.parent]->orient;
			parentTrans.i[0][3] = allLimbs[curConst.parent]->pos.x;
			parentTrans.i[1][3] = allLimbs[curConst.parent]->pos.y;
			parentTrans.i[2][3] = allLimbs[curConst.parent]->pos.z;
		}

		vec3 curChildConstPos = childTrans * curConst.posWrtChild;
		vec3 curParentConstPos = parentTrans * curConst.posWrtParent;

		parentTotalVel = vec3(0.0f);
		childTotalVel = vec3(0.0f);
		if (!parentIsWorld) parentTotalVel = (allLimbs[curConst.parent]->lin_v + cross(allLimbs[curConst.parent]->ang_v, curParentConstPos - allLimbs[curConst.parent]->pos));
		if (!childIsWorld) childTotalVel = (allLimbs[curConst.child]->lin_v + cross(allLimbs[curConst.child]->ang_v, curChildConstPos - allLimbs[curConst.child]->pos));
		vec3 totRelVel = parentTotalVel - childTotalVel;

		float springK = 25.5f;
		float springB = 1.0f;

		vec3 springForce = (curParentConstPos - curChildConstPos)*springK + totRelVel * springB;

		if (!childIsWorld) allLimbs[curConst.child]->ApplyForce(springForce, curChildConstPos, inpDeltaT);
		if (!parentIsWorld) allLimbs[curConst.parent]->ApplyForce(-springForce, curParentConstPos, inpDeltaT);

		if (curConst.type == HINGE)
		{
			curChildConstPos = childTrans * curConst.tanPtWrtChild;
			curParentConstPos = parentTrans * curConst.tanPtWrtParent;

			parentTotalVel = vec3(0.0f);
			childTotalVel = vec3(0.0f);
			if (!parentIsWorld) parentTotalVel = (allLimbs[curConst.parent]->lin_v + cross(allLimbs[curConst.parent]->ang_v, curParentConstPos - allLimbs[curConst.parent]->pos));
			if (!childIsWorld) childTotalVel = (allLimbs[curConst.child]->lin_v + cross(allLimbs[curConst.child]->ang_v, curChildConstPos - allLimbs[curConst.child]->pos));
			totRelVel = parentTotalVel - childTotalVel;

			vec3 springForce = (curParentConstPos - curChildConstPos)*springK + totRelVel * springB;

			if (!childIsWorld) allLimbs[curConst.child]->ApplyForce(springForce, curChildConstPos, inpDeltaT);
			if (!parentIsWorld) allLimbs[curConst.parent]->ApplyForce(-springForce, curParentConstPos, inpDeltaT);
		}
	}
}

void HIGHOMEGA::FIZ_X::ConstraintCollection::RandomShit()
{
	if (rand() % 100 > 96) {
		/*allLimbs["Cube"]->ang_v += vec3((rand() % 100 - 50)*0.02f, (rand() % 100 - 50)*0.02f, (rand() % 100 - 50)*0.02f) * 4.0f;
		allLimbs["Cube"]->lin_v += vec3((rand() % 100 - 50)*0.02f, (rand() % 100 - 50)*0.02f, (rand() % 100 - 50)*0.02f) * 4.0f;*/
	}
}