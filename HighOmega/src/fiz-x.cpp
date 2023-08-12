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
ClothCollectionClass HIGHOMEGA::FIZ_X::clothCollection;
BuoyancyRangeCollectionClass HIGHOMEGA::FIZ_X::buoyancyRangeCollection;
struct HIGHOMEGA::FIZ_X::clothSignalStruct HIGHOMEGA::FIZ_X::clothSignal;

MATERIAL HIGHOMEGA::FIZ_X::ObjectPiece::GetMaterial(std::string & mat)
{
	if (mat == "dirt") return DIRT;
	else if (mat == "rubble") return RUBBLE;
	else if (mat == "concrete") return CONCRETE;
	else if (mat == "metal") return METAL;
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
	if (!Mesh::getDataRowString(inpBlock, "texname", diffName)) throw std::runtime_error("Error fetching diffuse tex. name for RigidBody");

	elasticity = GetElasticity(diffName);
	friction = GetFriction(diffName);
	groupId = pieceId;

	float outTmp;
	if (Mesh::getDataRowFloat(inpBlock, "breakable", outTmp))
	{
		breakable = true;
	}
	else
	{
		breakable = false;
	}

	std::string matType;
	if (!Mesh::getDataRowString(inpBlock, "material", matType))
		matType = std::string("dirt");
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
	objPiece.tris.reserve(triList.size());
	transOnce = false;

	for (TriUV & curTri : triList)
	{
		ObjectTri tri;
		tri.e1 = curTri.eArr[0];
		tri.e2 = curTri.eArr[1];
		tri.e3 = curTri.eArr[2];
		tri.uv1 = curTri.uvArr[0];
		tri.uv2 = curTri.uvArr[1];
		tri.uv3 = curTri.uvArr[2];
		tri.min = vec3(Min(tri.e1.x, tri.e2.x, tri.e3.x), Min(tri.e1.y, tri.e2.y, tri.e3.y), Min(tri.e1.z, tri.e2.z, tri.e3.z));
		tri.max = vec3(Max(tri.e1.x, tri.e2.x, tri.e3.x), Max(tri.e1.y, tri.e2.y, tri.e3.y), Max(tri.e1.z, tri.e2.z, tri.e3.z));
		tri.n = curTri.normVec;
		objPiece.tris.push_back(tri);
	}
}

HIGHOMEGA::FIZ_X::RigidBody::~RigidBody()
{
	for (unsigned long long & curId : clothIds)
		clothCollection.Remove(curId);
	for (unsigned long long & curId : buoyancyRangeIds)
		buoyancyRangeCollection.Remove(curId);
	clothIds.clear();
	buoyancyRangeIds.clear();
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody(std::string & newGroupId, DataBlock &propBlock, std::vector<TriUV>& triList, vec3 inpNewCenter) : meshRef (nullptr)
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

	lin_v = vec3(0);
	ang_v = vec3(0);

	Frozen = false;
}

HIGHOMEGA::FIZ_X::RigidBody::RigidBody(Mesh & inpMesh, std::string belong , mat4 inpOrient, vec3 inpPos, bool inIsMap, std::function<bool(int, DataGroup &)> inpFilterFunction, vec3 *inpNewCenter, bool loadLowRes) : meshRef(&inpMesh)
{
	isMap = inIsMap;

	vec3 newCenter(0.0f);
	if (inpNewCenter) newCenter = *inpNewCenter;

	pos = inpPos + newCenter;
	orient = inpOrient;

	for (int i = 0; i != meshRef->DataGroups.size(); i++)
	{
		if (!inpFilterFunction(i, meshRef->DataGroups[i])) continue;

		DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		DataBlock *triBlock;
		if ( !Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock) ) continue;

		float outTmp;
		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "holographic", outTmp)) continue;

		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "cloth", outTmp))
		{
			clothIds.push_back(clothCollection.AddCloth(inpMesh, belong, this, orient, pos, i, curPolyGroup));
			continue;
		}

		if (Mesh::getDataRowFloat(curPolyGroup, "PROPS", "buoyancyRange", outTmp))
		{
			buoyancyRangeIds.push_back (buoyancyRangeCollection.AddRange(curPolyGroup));
			continue;
		}

		DataBlock *propsBlock;
		if (!Mesh::getDataBlock(curPolyGroup, "PROPS", &propsBlock)) continue;

		pieces.emplace_back(ObjectPiece(curPolyGroup.name + std::string("_") + std::to_string(i), *propsBlock));
		ObjectPiece &curPiece = pieces.back();

		vec3 e1, e2, e3;
		vec2 uv1, uv2, uv3;
		vec3 vNorm1, vNorm2, vNorm3;
		vec3 col1, col2, col3;
		ObjectTri tri;

		for (int j = 0; j != triBlock->blob.size() / (sizeof(RasterVertex) * 3); j++)
		{
			ObjectTri tri;

			unpackRasterVertex(e1, col1, uv1, vNorm1, ((RasterVertex *)triBlock->blob.data())[j * 3]);
			unpackRasterVertex(e2, col2, uv2, vNorm2, ((RasterVertex *)triBlock->blob.data())[j * 3 + 1]);
			unpackRasterVertex(e3, col3, uv3, vNorm3, ((RasterVertex *)triBlock->blob.data())[j * 3 + 2]);

			tri.e1 = e1 - newCenter;
			tri.e2 = e2 - newCenter;
			tri.e3 = e3 - newCenter;
			tri.uv1 = uv1;
			tri.uv2 = uv2;
			tri.uv3 = uv3;
			tri.min = vec3(Min(tri.e1.x, tri.e2.x, tri.e3.x), Min(tri.e1.y, tri.e2.y, tri.e3.y), Min(tri.e1.z, tri.e2.z, tri.e3.z));
			tri.max = vec3(Max(tri.e1.x, tri.e2.x, tri.e3.x), Max(tri.e1.y, tri.e2.y, tri.e3.y), Max(tri.e1.z, tri.e2.z, tri.e3.z));
			tri.n = cross(tri.e1 - tri.e2, tri.e3 - tri.e2).normalized();
			if (vNorm1 * tri.n < 0.0f) tri.n = -tri.n;

			curPiece.tris.push_back(tri);
		}
	}

	transOnce = false;
	Frozen = false;

	InitMassInertiaTensorAndBoundingSpheres();
	Update();
	if (isMap) mass = HIGHOMEGA_INFINITE_MASS;

	lin_v = vec3(0);
	ang_v = vec3(0);


	bobbies_dia = 0.0f;
	for (int i = 0; i != meshRef->DataGroups.size(); i++)
	{
		if (meshRef->DataGroups[i].type != "BOBBIES") continue;

		DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		DataBlock *buoyancyBobbies = nullptr;
		if ( !Mesh::getDataBlock(curPolyGroup, "DESCRIPTION", &buoyancyBobbies)) throw std::runtime_error("Error fetching desc. for RigidBody bobbies");

		if ( !buoyancyBobbies->rows[0][1].fvalue(bobbies_dia)) throw std::runtime_error("Error fetching diameter for RigidBody bobbies");

		float xFetch, yFetch, zFetch;
		for (int j = 1; j != buoyancyBobbies->rows.size(); j++)
		{
			if ( !buoyancyBobbies->rows[j][0].fvalue(xFetch) ||
					!buoyancyBobbies->rows[j][1].fvalue(yFetch) ||
					!buoyancyBobbies->rows[j][2].fvalue(zFetch) ) throw std::runtime_error("Error fetching bobby for RigidBody");
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

	vec3 objMin, objMax;
	for (int i = 0; i != pieces.size(); i++)
	{
		ObjectPiece & curPiece = pieces[i];
		curPiece.pos = vec3(0.0f);
		curPiece.radius = 0.0f;
		vec3 pieceMin, pieceMax;
		for (int j = 0; j != curPiece.tris.size(); j++)
		{
			if (j == 0)
			{
				pieceMin = curPiece.tris[j].min;
				pieceMax = curPiece.tris[j].max;
				if (i == 0)
				{
					objMin = pieceMin;
					objMax = pieceMax;
				}
			}
			else
			{
				pieceMin.x = min(pieceMin.x, curPiece.tris[j].min.x);
				pieceMin.y = min(pieceMin.y, curPiece.tris[j].min.y);
				pieceMin.z = min(pieceMin.z, curPiece.tris[j].min.z);
				pieceMax.x = max(pieceMax.x, curPiece.tris[j].max.x);
				pieceMax.y = max(pieceMax.y, curPiece.tris[j].max.y);
				pieceMax.z = max(pieceMax.z, curPiece.tris[j].max.z);
			}
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
		for (ObjectPiece & curPiece : pieces)
		{
			float IxxAccum = 0, IyyAccum = 0, IzzAccum = 0, IxyAccum = 0, IyzAccum = 0, IzxAccum = 0;
			float curDensity = curPiece.GetDensity(curPiece.diffName);
			for (int j = 0; j != curPiece.tris.size(); j++)
			{
				vec3 v1 = curPiece.tris[j].e1;
				vec3 v2 = curPiece.tris[j].e2;
				vec3 v3 = curPiece.tris[j].e3;
				vec3 v1_v2 = v1 - v2;
				vec3 v3_v2 = v3 - v2;
				vec3 triAreaCross = cross(v1_v2, v3_v2);
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

void HIGHOMEGA::FIZ_X::RigidBody::Update()
{
	if (Frozen || mass == HIGHOMEGA_INFINITE_MASS) return;

	mat4 totalTrans = orient, totalTransDT;
	totalTrans.i[0][3] = pos.x;
	totalTrans.i[1][3] = pos.y;
	totalTrans.i[2][3] = pos.z;
	totalTransDT = totalTrans.DirectionTransform();

	if (!transOnce)
	{
		transOnce = true;
		prevTrans.Ident();
		prevTransDT.Ident();
	}

	mat4 totalTransFromRef = totalTrans * prevTrans.Inv();
	mat4 totalTransDTFromRef = totalTransDT * prevTransDT.Inv();

	for (ObjectPiece & curPiece : pieces)
		for (unsigned int i = 0; i != curPiece.tris.size(); i++)
		{
			curPiece.tris[i].e1 = totalTransFromRef * curPiece.tris[i].e1;
			curPiece.tris[i].e2 = totalTransFromRef * curPiece.tris[i].e2;
			curPiece.tris[i].e3 = totalTransFromRef * curPiece.tris[i].e3;
			curPiece.tris[i].n = totalTransDTFromRef * curPiece.tris[i].n;
			curPiece.tris[i].min = vec3(Min(curPiece.tris[i].e1.x, curPiece.tris[i].e2.x, curPiece.tris[i].e3.x), Min(curPiece.tris[i].e1.y, curPiece.tris[i].e2.y, curPiece.tris[i].e3.y), Min(curPiece.tris[i].e1.z, curPiece.tris[i].e2.z, curPiece.tris[i].e3.z));
			curPiece.tris[i].max = vec3(Max(curPiece.tris[i].e1.x, curPiece.tris[i].e2.x, curPiece.tris[i].e3.x), Max(curPiece.tris[i].e1.y, curPiece.tris[i].e2.y, curPiece.tris[i].e3.y), Max(curPiece.tris[i].e1.z, curPiece.tris[i].e2.z, curPiece.tris[i].e3.z));
		}

	prevTrans = totalTrans;
	prevTransDT = totalTransDT;

	GetCurInertiaTensorInv();
	UpdateAABB();
}

void HIGHOMEGA::FIZ_X::RigidBody::UpdateAABB()
{
	for (int i = 0; i != pieces.size(); i++)
	{
		ObjectPiece & curPiece = pieces[i];
		for (int j = 0; j != curPiece.tris.size(); j++)
		{
			if (j == 0)
			{
				curPiece.piece_min = curPiece.tris[j].min;
				curPiece.piece_max = curPiece.tris[j].max;
				if (i == 0)
				{
					body_min = curPiece.piece_min;
					body_max = curPiece.piece_max;
				}
			}
			else
			{
				curPiece.piece_min.x = min(curPiece.piece_min.x, curPiece.tris[j].min.x);
				curPiece.piece_min.y = min(curPiece.piece_min.y, curPiece.tris[j].min.y);
				curPiece.piece_min.z = min(curPiece.piece_min.z, curPiece.tris[j].min.z);
				curPiece.piece_max.x = max(curPiece.piece_max.x, curPiece.tris[j].max.x);
				curPiece.piece_max.y = max(curPiece.piece_max.y, curPiece.tris[j].max.y);
				curPiece.piece_max.z = max(curPiece.piece_max.z, curPiece.tris[j].max.z);
			}
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
	newPiece.breakable = oldPiece->breakable;
	newPiece.radius = oldPiece->radius;
	newPiece.pos = oldPiece->pos;
	newPiece.piece_min = oldPiece->piece_min;
	newPiece.piece_max = oldPiece->piece_max;
	newPiece.mat = oldPiece->mat;
	newPiece.diffName = oldPiece->diffName;
	newPiece.groupId = oldPiece->groupId;

	*oldPiece = newPiece;
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
			return;
		}
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
	if ( !Mesh::getDataBlock(*inpDataGroupRef, "TRIS", &triBlockTmp) ) throw std::runtime_error("Error fetching blocks for cloth");
	triBlock = *triBlockTmp;
}

bool HIGHOMEGA::FIZ_X::operator==(const ClothPiece::NeighborInfo & lhs, const ClothPiece::NeighborInfo & rhs)
{
	return (lhs.index == rhs.index);
}

unsigned long long HIGHOMEGA::FIZ_X::ClothCollectionClass::AddCloth(Mesh & inpMesh, std::string belong, RigidBody *inpRigidBody, mat4 inpOrient, vec3 inpPos, int inpGroupId, DataGroup & inpPolyGroup)
{
	ClothPiece newCloth (new GraphicsModel(inpMesh, belong, Instance, [&inpGroupId = inpGroupId](int groupId, DataGroup & inpPolyGroup) -> bool {
		if (groupId == inpGroupId)
			return true;
		else
			return false;
	}, nullptr, false, false), inpRigidBody, &inpPolyGroup);

	DataBlock *triBlock = &newCloth.triBlock;

	mat4 trans = inpOrient;
	trans.i[0][3] = inpPos.x;
	trans.i[1][3] = inpPos.y;
	trans.i[2][3] = inpPos.z;

	vec3 pos1, pos2, pos3, col1, col2, col3, vNorm1, vNorm2, vNorm3;
	vec2 uv1, uv2, uv3;
	unsigned int i1, i2, i3;
	for (unsigned int i = 0; i != triBlock->blob.size() / (sizeof(RasterVertex) * 3); i++)
	{
		ClothPiece::PointMass pMass;
		pMass.accel = pMass.vel = vec3(0.0f);

		unsigned int vertIndex1 = i * 3;
		unsigned int vertIndex2 = vertIndex1 + 1;
		unsigned int vertIndex3 = vertIndex1 + 2;

		unpackRasterVertex(pos1, col1, uv1, vNorm1, ((RasterVertex *)triBlock->blob.data())[vertIndex1]);
		unpackRasterVertex(pos2, col2, uv2, vNorm2, ((RasterVertex *)triBlock->blob.data())[vertIndex2]);
		unpackRasterVertex(pos3, col3, uv3, vNorm3, ((RasterVertex *)triBlock->blob.data())[vertIndex3]);

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
		newCloth.pts[i1].vertIndex.push_back(vertIndex1);
		newCloth.pts[i2].vertIndex.push_back(vertIndex2);
		newCloth.pts[i3].vertIndex.push_back(vertIndex3);
		ClothPiece::NeighborInfo neighborInf;
		neighborInf.restLen = 0.0f;

		neighborInf.index = i2;
		newCloth.pts[i1].neighbors.push_back(neighborInf);
		neighborInf.index = i3;
		newCloth.pts[i1].neighbors.push_back(neighborInf);

		neighborInf.index = i1;
		newCloth.pts[i2].neighbors.push_back(neighborInf);
		neighborInf.index = i3;
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
							newCloth.pts[uncommonIndex].neighbors.emplace_back();
							newCloth.pts[uncommonIndex].neighbors.back().index = diagonalIndex;

							newCloth.pts[diagonalIndex].neighbors.emplace_back();
							newCloth.pts[diagonalIndex].neighbors.back().index = uncommonIndex;

							addedDiagonal = true;
							break;
						}
					}
			}
		}
	}

	for (ClothPiece::PointMass & curPtMass : newCloth.pts)
		for (int j = 0; j != curPtMass.neighbors.size(); j++)
			curPtMass.neighbors[j].restLen = (newCloth.pts[curPtMass.neighbors[j].index].pos - curPtMass.pos).length();

	unsigned long long addedClothId = mersenneTwister64BitPRNG();

	CommonSharedMutex.lock();
	clothItems[addedClothId] = newCloth;
	CommonSharedMutex.unlock();

	return addedClothId;
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::SetRenderSubmissions(std::vector<HIGHOMEGA::RENDER::GroupedRenderSubmission*>&& inpSubList)
{
	subList = inpSubList;
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::Remove(unsigned long long clothId)
{
	CommonSharedMutex.lock();
	for (SubmittedRenderItem curClothSubInfo : clothItems[clothId].allSubmissions)
		curClothSubInfo.producer->Remove(curClothSubInfo);
	delete clothItems[clothId].clothModel;
	clothItems.erase(clothId);
	CommonSharedMutex.unlock();
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::UploadData()
{
	for (std::pair<const unsigned long long, ClothPiece> & curCloth : clothItems)
	{
		if (curCloth.second.allSubmissions.size() == 0)
			for (GroupedRenderSubmission *curSubmission : subList)
				curCloth.second.allSubmissions.emplace_back(curSubmission->Add(*(curCloth.second.clothModel)));
		curCloth.second.clothModel->MaterialGeomMap.begin()->second.begin()->Update(curCloth.second.renderVerts);
		curCloth.second.clothModel->SetDirty();
	}
}

void HIGHOMEGA::FIZ_X::ClothCollectionClass::ClearContent()
{
	for (std::pair<const unsigned long long, ClothPiece> & curCloth : clothItems) {
		for (SubmittedRenderItem curClothSubInfo : curCloth.second.allSubmissions)
			curClothSubInfo.producer->Remove(curClothSubInfo);
		delete curCloth.second.clothModel;
	}
	clothItems.clear();
	subList.clear();
}

unsigned long long HIGHOMEGA::FIZ_X::BuoyancyRangeCollectionClass::AddRange(DataGroup & inpPolyGroup)
{
	vec3 minRange, maxRange;

	DataBlock *triBlock;
	if ( !Mesh::getDataBlock(inpPolyGroup, "TRIS", &triBlock) ) throw std::runtime_error("Error fetching buoyancy range vert block");

	vec3 curPos;
	vec3 curCol;
	vec2 curUV;
	vec3 curNorm;
	for (int i = 0; i != triBlock->blob.size() / sizeof(RasterVertex); i++)
	{
		unpackRasterVertex(curPos, curCol, curUV, curNorm, ((RasterVertex *)triBlock->blob.data())[i]);
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

	unsigned long long itemId = mersenneTwister64BitPRNG();
	CommonSharedMutex.lock();
	ranges[itemId] = curRange;
	CommonSharedMutex.unlock();

	return itemId;
}

void HIGHOMEGA::FIZ_X::BuoyancyRangeCollectionClass::Remove(unsigned long long rangeId)
{
	CommonSharedMutex.lock();
	ranges.erase(rangeId);
	CommonSharedMutex.unlock();
}

void HIGHOMEGA::FIZ_X::ClothLoop()
{
	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		CommonSharedMutex.lock_shared();
		try
		{
			for (std::pair<const unsigned long long, ClothPiece> & curCloth : clothCollection.clothItems)
			{
				ClothPiece & curClothPiece = curCloth.second;
				vec3 windForce = vec3(-60.0f, 0.0f, 0.0f) * max((rand() % 1000 - 900) / 100.0f, 0.0f);
				if (!curClothPiece.rigidBodyRef->isMap)
				{
					bool resetPos = false;
					mat4 trans = curClothPiece.rigidBodyRef->orient;
					trans.i[0][3] = curClothPiece.rigidBodyRef->pos.x;
					trans.i[1][3] = curClothPiece.rigidBodyRef->pos.y;
					trans.i[2][3] = curClothPiece.rigidBodyRef->pos.z;
					for (ClothPiece::PointMass & curPtMass : curClothPiece.pts)
					{
						if (!curPtMass.fixed) continue;
						if ((trans * curPtMass.origPos - curPtMass.pos).length() > curClothPiece.snapClothBackToBodyThreshold)
							resetPos = true;
						else
							resetPos = false;
						break;
					}
					for (ClothPiece::PointMass & curPtMass : curClothPiece.pts)
						if (resetPos || (!resetPos && curPtMass.fixed))
							curPtMass.pos = trans * curPtMass.origPos;
				}
				float ToverM = DeltaT / curClothPiece.m;
				for (ClothPiece::PointMass & curPt : curClothPiece.pts)
				{
					if (curPt.fixed) continue;
					for (int j = 0; j != curPt.neighbors.size(); j++)
					{
						ClothPiece::PointMass & neighPt = curClothPiece.pts[curPt.neighbors[j].index];
						vec3 diffVec = neighPt.pos - curPt.pos;
						float diffVecLen = diffVec.length();
						vec3 diffVecNorm = diffVec / diffVecLen;
						vec3 forceCompute = (diffVecLen - curPt.neighbors[j].restLen) * diffVecNorm * curClothPiece.k - curPt.vel * curClothPiece.b;
						forceCompute += GravDir * Grav;
						forceCompute += windForce;

						curPt.prevAccel = curPt.accel;
						curPt.accel = forceCompute / curClothPiece.m;

						curPt.prevVel = curPt.vel;
						curPt.vel += (curPt.accel + curPt.prevAccel) * (DeltaT * 0.5f);
					}
				}
				for (int i = 0; i != curClothPiece.pts.size(); i++)
				{
					ClothPiece::PointMass & curPt = curClothPiece.pts[i];
					curPt.updatedTangentSpace = false;
					if (!curPt.fixed)
					{
						curPt.prevPos = curPt.pos;
						curPt.pos += curPt.vel * DeltaT + curPt.accel * (DeltaT * DeltaT * 0.5f);
					}
					if (i == 0)
					{
						curClothPiece.clothMin = curClothPiece.clothMax = curPt.pos;
					}
					else
					{
						curClothPiece.clothMin = vec3(min(curClothPiece.clothMin.x, curPt.pos.x), min(curClothPiece.clothMin.y, curPt.pos.y), min(curClothPiece.clothMin.z, curPt.pos.z));
						curClothPiece.clothMax = vec3(max(curClothPiece.clothMax.x, curPt.pos.x), max(curClothPiece.clothMax.y, curPt.pos.y), max(curClothPiece.clothMax.z, curPt.pos.z));
					}
				}

				curClothPiece.renderVerts.resize((unsigned int)curClothPiece.triBlock.blob.size() / sizeof(RasterVertex));

				for (int i = 0; i != curClothPiece.pts.size(); i++)
				{
					int ptIndex = 0;
					ClothPiece::PointMass & curPt = curClothPiece.pts[i];
					if (!curPt.updatedTangentSpace)
					{
						vec3 normTotal(0.0f);
						float normCount = 0.0f;
						for (int j = 0; j != curPt.neighbors.size(); j++)
						{
							ClothPiece::PointMass & neigh1 = curClothPiece.pts[curPt.neighbors[j].index];
							int j_1 = j + 1;
							if (j_1 == curPt.neighbors.size()) j_1 = 0;
							ClothPiece::PointMass & neigh2 = curClothPiece.pts[curPt.neighbors[j_1].index];
							normTotal += cross(neigh1.pos - curPt.pos, neigh2.pos - curPt.pos).normalized();
							normCount += 1.0f;
						}
						curPt.norm = (normTotal / normCount).normalized();
						RasterVertex & rv = ((RasterVertex *)curClothPiece.triBlock.blob.data())[curPt.vertIndex[0]];
						vec3 rvNorm = fromZSignXY(rv.Norm);
						if (rvNorm * curPt.norm < 0.0f) curPt.norm = -curPt.norm;
						curPt.updatedTangentSpace = true;
					}
					for (unsigned int j = 0; j != curPt.vertIndex.size(); j++)
					{
						vec3 curCol;
						vec2 curUV;
						vec3 curPos;
						vec3 curNorm;
						RasterVertex & readRV = ((RasterVertex *)curClothPiece.triBlock.blob.data())[curPt.vertIndex[j]];
						RasterVertex & writeRV = curClothPiece.renderVerts[curPt.vertIndex[j]];
						unpackRasterVertex(curPos, curCol, curUV, curNorm, readRV);
						packRasterVertex(curPt.pos, curCol, curUV, curPt.norm, writeRV);
					}
				}
			}
		}
		catch (std::exception e)
		{
			CommonSharedMutex.unlock_shared();
			return;
		}
		CommonSharedMutex.unlock_shared();

		double timerDiff = 10.0 - timerObj.Diff()*1000.0;
		if (timerDiff > 0.0f) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{std::unique_lock<std::mutex> lk(clothSignal.quit_mutex); readQuit = clothSignal.quit; }
		if (readQuit) return;
	}
}

void HIGHOMEGA::FIZ_X::PiecePiece(ObjectPiece & a, ObjectPiece & b, RigidBody * aBody, RigidBody * bBody, std::vector<intersection>& intersect_list, bool quitOnIntersection)
{
	for (ObjectTri & aTri : a.tris)
		for (ObjectTri & bTri : b.tris)
		{
			if (aTri.min.x > bTri.max.x) continue;
			if (aTri.max.x < bTri.min.x) continue;
			if (aTri.min.y > bTri.max.y) continue;
			if (aTri.max.y < bTri.min.y) continue;
			if (aTri.min.z > bTri.max.z) continue;
			if (aTri.max.z < bTri.min.z) continue;

			vec3 t1[3], t2[3], n1_v, n2_v;
			t1[0] = aTri.e1;
			t1[1] = aTri.e2;
			t1[2] = aTri.e3;
			n1_v = aTri.n;

			t2[0] = bTri.e1;
			t2[1] = bTri.e2;
			t2[2] = bTri.e3;
			n2_v = bTri.n;

			vec3 pt1, pt2;
			TRITRI_INTERSECT_MODE mode;

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

void HIGHOMEGA::FIZ_X::RigidBodyRigidBody(RigidBody & a, RigidBody & b, std::vector <intersection> & intersect_list, bool quitOnIntersection)
{
	if (a.mass == HIGHOMEGA_INFINITE_MASS && b.mass == HIGHOMEGA_INFINITE_MASS) return;
	vec3 a_max = a.body_max;
	vec3 b_max = b.body_max;
	vec3 a_min = a.body_min;
	vec3 b_min = b.body_min;
	for (ObjectPiece & aPiece : a.pieces)
	{
		if (a.mass == HIGHOMEGA_INFINITE_MASS)
		{
			vec3 group_min = aPiece.piece_min;
			vec3 group_max = aPiece.piece_max;
			if (group_max.x < b_min.x ||
				group_min.x > b_max.x ||
				group_max.y < b_min.y ||
				group_min.y > b_max.y ||
				group_max.z < b_min.z ||
				group_min.z > b_max.z)
				continue;
		}
		for (ObjectPiece & bPiece : b.pieces)
		{
			if (b.mass == HIGHOMEGA_INFINITE_MASS)
			{
				vec3 group_min = bPiece.piece_min;
				vec3 group_max = bPiece.piece_max;
				if (group_max.x < a_min.x ||
					group_min.x > a_max.x ||
					group_max.y < a_min.y ||
					group_min.y > a_max.y ||
					group_max.z < a_min.z ||
					group_min.z > a_max.z)
					continue;
			}
			if (aPiece.breakable == bPiece.breakable && bPiece.breakable) continue; // This is usually a lot of these pieces getting stuck into eachother... not worth it
			PiecePiece(aPiece, bPiece, &a, &b, intersect_list, quitOnIntersection);
			if (quitOnIntersection && intersect_list.size() > 0) return;
		}
	}
}

bool HIGHOMEGA::FIZ_X::LinePieceClosest(ObjectPiece & piece, vec3 & linA, vec3 & linB, vec3 & linMin, vec3 & linMax, vec3 & closestNorm)
{
	float k = 1.0f;
	bool retVal = false;
	vec3 group_min = piece.piece_min;
	vec3 group_max = piece.piece_max;
	if (group_max.x < linMin.x ||
		group_min.x > linMax.x ||
		group_max.y < linMin.y ||
		group_min.y > linMax.y ||
		group_max.z < linMin.z ||
		group_min.z > linMax.z)
		return retVal;
	vec3 linOrig = linA;
	vec3 linDir = linB - linA;
	for (ObjectTri & curTri : piece.tris)
	{
		if (LineSegTri(linOrig, linDir, curTri.e1, curTri.e2, curTri.e3, k))
		{
			linB = linOrig + linDir * k;
			linMin = vec3(min(linA.x, linB.x), min(linA.y, linB.y), min(linA.z, linB.z));
			linMax = vec3(max(linA.x, linB.x), max(linA.y, linB.y), max(linA.z, linB.z));
			closestNorm = curTri.n;
			retVal = true;
		}
	}
	return retVal;
}

bool HIGHOMEGA::FIZ_X::LineMeshClosest(RigidBody & body, vec3 & linA, vec3 & linB, vec3 & linMin, vec3 & linMax, vec3 & closestNorm, std::string & hitGroupId)
{
	bool retVal = false;
	for (ObjectPiece & curPiece : body.pieces)
	{
		if (LinePieceClosest(curPiece, linA, linB, linMin, linMax, closestNorm))
		{
			hitGroupId = curPiece.groupId;
			retVal = true;
		}
	}
	return retVal;
}

bool HIGHOMEGA::FIZ_X::LineWorldClosest(vec3 & linA, vec3 & linB, RigidBody **hitBody, std::string & hitGroupId, vec3 & hitNorm)
{
	bool retVal = false;
	vec3 linMin = vec3(min(linA.x, linB.x), min(linA.y, linB.y), min(linA.z, linB.z));
	vec3 linMax = vec3(max(linA.x, linB.x), max(linA.y, linB.y), max(linA.z, linB.z));
	std::string groupId;
	for (RigidBody * curBody : allBodies)
	{
		if (LineMeshClosest(*curBody, linA, linB, linMin, linMax, hitNorm, groupId))
		{
			hitGroupId = groupId;
			*hitBody = curBody;
			retVal = true;
		}
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

void HIGHOMEGA::FIZ_X::ApplyGravityBuoyancy(float delta_t)
{
	for (RigidBody * curBody : allBodies)
	{
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

			ApplyGravityBuoyancy(DeltaT);

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

HIGHOMEGA::FIZ_X::ConstraintCollection::ConstraintCollection(Mesh & inpMesh, std::unordered_map <std::string, RigidBody *> & inpLimbs, vec3 inpPos, InstanceClass &ptrToInstance)
{
	allLimbs = inpLimbs;

	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		DataGroup &curPolyGroup = inpMesh.DataGroups[i];
		std::string constraintType;

		if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "constraint", constraintType)) continue;

		Constraint curConst;
		if ( !Mesh::getDataRowString(curPolyGroup, "PROPS", "parent", curConst.parent) ||
				!Mesh::getDataRowString(curPolyGroup, "PROPS", "child", curConst.child) ) throw std::runtime_error("Error fetching child or parent for constraints");
		vec3 constPos;
		if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", constPos) ) throw std::runtime_error("Error fetching constraint position");
		constPos += inpPos;
		float constOrderFloat;
		if ( !Mesh::getDataRowFloat(curPolyGroup, "PROPS", "order", constOrderFloat) ) throw std::runtime_error("Error fetching constraint order");
		curConst.order = (unsigned int)constOrderFloat;
		maxOrder = max(maxOrder, curConst.order);

		mat4 parentTrans = allLimbs[curConst.parent]->orient;
		parentTrans.i[0][3] = allLimbs[curConst.parent]->pos.x;
		parentTrans.i[1][3] = allLimbs[curConst.parent]->pos.y;
		parentTrans.i[2][3] = allLimbs[curConst.parent]->pos.z;
		mat4 parentTransInv = parentTrans.Inv();

		curConst.posWrtParent = parentTransInv * constPos;

		mat4 childTrans = allLimbs[curConst.child]->orient;
		childTrans.i[0][3] = allLimbs[curConst.child]->pos.x;
		childTrans.i[1][3] = allLimbs[curConst.child]->pos.y;
		childTrans.i[2][3] = allLimbs[curConst.child]->pos.z;
		mat4 childTransInv = childTrans.Inv();

		curConst.posWrtChild = childTransInv * constPos;

		if (constraintType == "hinge")
		{
			curConst.type = HINGE;

			vec3 hingeTan;
			if ( !Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", hingeTan) ) throw std::runtime_error("Error fetching hinge tangent");
			hingeTan = hingeTan.normalized();
			curConst.tanPtWrtParent = parentTransInv * (constPos + hingeTan);
			curConst.tanPtWrtChild = childTransInv * (constPos + hingeTan);
		}
		else
		{
			curConst.type = POSITIONAL;
		}

		allConstraints.push_back(curConst);
	}

	CommonSharedMutex.lock();
	constraintCollection.push_back(this);
	CommonSharedMutex.unlock();
}

void HIGHOMEGA::FIZ_X::ConstraintCollection::ApplyConstraintForces(float inpDeltaT)
{
	for (Constraint & curConst : allConstraints)
	{
		mat4 childTrans = allLimbs[curConst.child]->orient;
		childTrans.i[0][3] = allLimbs[curConst.child]->pos.x;
		childTrans.i[1][3] = allLimbs[curConst.child]->pos.y;
		childTrans.i[2][3] = allLimbs[curConst.child]->pos.z;

		mat4 parentTrans = allLimbs[curConst.parent]->orient;
		parentTrans.i[0][3] = allLimbs[curConst.parent]->pos.x;
		parentTrans.i[1][3] = allLimbs[curConst.parent]->pos.y;
		parentTrans.i[2][3] = allLimbs[curConst.parent]->pos.z;

		vec3 curChildConstPos = childTrans * curConst.posWrtChild;
		vec3 curParentConstPos = parentTrans * curConst.posWrtParent;

		vec3 totRelVel = (allLimbs[curConst.parent]->lin_v + cross(allLimbs[curConst.parent]->ang_v, curParentConstPos - allLimbs[curConst.parent]->pos)) -
						 (allLimbs[curConst.child]->lin_v + cross(allLimbs[curConst.child]->ang_v, curChildConstPos - allLimbs[curConst.child]->pos));

		float springK = 25.5f;
		float springB = 1.0f;

		vec3 springForce = (curParentConstPos - curChildConstPos)*springK + totRelVel * springB;

		allLimbs[curConst.child]->ApplyForce(springForce, curChildConstPos, inpDeltaT);
		allLimbs[curConst.parent]->ApplyForce(-springForce, curParentConstPos, inpDeltaT);

		if (curConst.type == HINGE)
		{
			curChildConstPos = childTrans * curConst.tanPtWrtChild;
			curParentConstPos = parentTrans * curConst.tanPtWrtParent;

			totRelVel = (allLimbs[curConst.parent]->lin_v + cross(allLimbs[curConst.parent]->ang_v, curParentConstPos - allLimbs[curConst.parent]->pos)) -
						(allLimbs[curConst.child]->lin_v + cross(allLimbs[curConst.child]->ang_v, curChildConstPos - allLimbs[curConst.child]->pos));

			vec3 springForce = (curParentConstPos - curChildConstPos)*springK + totRelVel * springB;

			allLimbs[curConst.child]->ApplyForce(springForce, curChildConstPos, inpDeltaT);
			allLimbs[curConst.parent]->ApplyForce(-springForce, curParentConstPos, inpDeltaT);
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