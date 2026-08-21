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

#pragma once

#include <mesh.h>
#include "render.h"
#include "util.h"
#include <geom.h>
#include <list>
#include <map>
#include <algorithm>
#include <chrono>
#include <thread>
#include <condition_variable>
#include "accelstructs.h"

using namespace HIGHOMEGA::MESH;
using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::MATH::ACCEL_STRUCT;
using namespace HIGHOMEGA::GEOM;
using namespace HIGHOMEGA::RENDER;
using namespace HIGHOMEGA::GL;

namespace HIGHOMEGA
{
	namespace FIZ_X
	{
		enum BROADSWEEP_RESULT
		{
			DO_NOT_KNOW_YET,
			WILL_NOT_COLLIDE,
			WILL_COLLIDE,
			KNOW_IT_WILL_COLLIDE
		};
		extern vec3 GravDir;
		extern float Grav;
		extern float DeltaT;
		extern float ColTol;
		extern std::vector<std::vector<BROADSWEEP_RESULT>> collisionMap;
		struct ObjectTri
		{
			unsigned int e1, e2, e3;
			bool nFlip;
		};
		struct ObjectVertex
		{
			float x, y, z;
		};
		enum MATERIAL
		{
			DIRT,
			RUBBLE,
			CERAMIC,
			GLASS,
			CONCRETE,
			METALPLATFORM,
			METALRODS,
			PLASTIC,
			WOOD,
			GRASS,
			WATER
		};
		class ObjectPiece
		{
		public:
			std::string groupId;
			float elasticity;
			float friction;
			bool cuttable;
			bool shatterable;
			unsigned int strength;
			float cutRadius, cutDepth;
			float radius;
			vec3 pos;
			vec3 piece_min, piece_max;
			MATERIAL mat;
			std::string diffName;
			std::vector <ObjectTri> tris;
			std::vector <ObjectVertex> verts;

			MATERIAL GetMaterial(std::string &mat);
			float GetDensity(std::string & inTex);
			float GetElasticity(std::string & inTex);
			float GetFriction(std::string & inTex);
			ObjectPiece();
			ObjectPiece(std::string groupName, DataBlock & inpBlock);
		};
		class RigidBody;
		class ClothPiece
		{
		public:
			class NeighborInfo
			{
			public:
				int index;
				float restLen = 0.0f;
				bool sharetri = true;
			};
			struct PointMass
			{
				vec3 pos;
				vec3 prevPos;
				vec3 origPos;
				vec3 norm;
				char flipNorm = 0;
				vec3 vel;
				vec3 prevVel;
				vec3 accel;
				vec3 prevAccel;
				std::vector <NeighborInfo> neighbors;
				std::unordered_set <unsigned int> vertIndex;
				bool fixed;
			};
			float m, k, b, nRT, snapClothBackToBodyThreshold = 10.0f;
			vec3 suddenDisplacement = vec3(0.0f);
			std::vector <PointMass> pts;
			GraphicsModel *clothModel = nullptr;
			GraphicsModelInstance* clothModelInst = nullptr;
			RigidBody *rigidBodyRef = nullptr;
			DataBlock triBlock = DataBlock(std::string(""));
			vec3 clothMin, clothMax;
			bool pressureSoftBody, unaffectedByWind;
			std::vector<unsigned int> vertIndexToPtMass;
			struct RayHit
			{
				vec3 pt, dir;
				float invInflRadSq;
				RayHit(vec3 inPt, vec3 inDir, float inInvInfRadSq) : pt(inPt), dir(inDir), invInflRadSq(inInvInfRadSq) {}
			};
			std::vector<RayHit> rayHits;

			ClothPiece();
			ClothPiece(GraphicsModel *inpClothModel, RigidBody *inpRigidBodyRef, DataGroup *inpDataGroupRef);
		};
		struct BuoyancyRange
		{
			vec3 minRange, maxRange;
		};
		struct PlayerClothObstacles
		{
			vec3 pos[3];
			float rad;
		};
		extern std::unordered_map<unsigned long long, PlayerClothObstacles> allObstacles;
		extern std::shared_mutex allObstaclesMutex;
		class ClothCollectionClass
		{
		private:
			struct clothSignalStruct {
				bool quit;
				std::mutex quit_mutex;
			};
			clothSignalStruct clothSignal;
			std::thread* simulationThread = nullptr;
			std::shared_mutex clothAccessMutex;
			std::shared_mutex windSourceMutex;
			std::vector<GroupedRenderSubmission*> subList;
			std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> perSubmissionCall;
			static float sdCappedCylinder(const vec3& p, const vec3& a, const vec3& b, float r);
			static void ClothLoop(ClothCollectionClass* clothCollection);
			std::mutex rayHitMutex;
			bool allMinMaxesComputed = false;
			const float displaceStrength = 10.0f;

		public:
			struct TriHit
			{
				vec3 pt, n;

				TriHit(vec3 inPt, vec3 inNorm) : pt(inPt), n(inNorm) {}
			};
			struct WindSource
			{
				vec3 pos, dir;
				float rad, dist, strength;
			};
			std::vector<TriHit> allTriHits;
			std::unordered_map<unsigned long long, ClothPiece> clothItems;
			std::unordered_map<unsigned long long, WindSource> windSources;

			unsigned long long Add(Mesh & inpMesh, std::string belong, RigidBody *inpRigidBody, mat4 inpOrient, vec3 inpPos, int inpGroupId, DataGroup & inpPolyGroup);
			void GetMinMax(unsigned long long clothId, vec3& outMin, vec3& outMax);
			void AddSuddenDisplacement(unsigned long long clothId, const vec3& inSuddenDisp);
			void AddImpulse(unsigned long long clothId, const vec3& inImpulse);
			unsigned long long AddWindSources(HIGHOMEGA::MESH::DataGroup& curPolyGroup);
			void Remove(unsigned long long clothOrSourceId);
			void Combine(ClothCollectionClass& clothCollection, std::vector<GroupedRenderSubmission*>& submissionList, std::function<void(GroupedRenderSubmission*, GraphicsModelInstance*)> inpPerSubmissionCall);
			void Update();
			void IntersectRay(vec3& linA, vec3& linB);
			void ClearContent();
		};
		class BuoyancyRangeCollectionClass
		{
		public:
			std::unordered_map<unsigned long long, BuoyancyRange> ranges;

			unsigned long long AddRange(DataGroup & inpPolyGroup);
			void Combine(BuoyancyRangeCollectionClass& buoyancyRangeCollection);
			void Remove(unsigned long long rangeId);
			void ClearContent();
		};
		extern BuoyancyRangeCollectionClass buoyancyRangeCollection;
		#define HIGHOMEGA_INFINITE_MASS -1
		class RigidBody
		{
		private:
			std::vector <vec3> bobbies; // Buoyancy bobbies
			float bobbies_dia; // Bobby diameter

			void IT_Tri(vec3 & v1, vec3 & v2, vec3 & v3, vec3 & v1_v2, vec3 & v3_v2, float step, float *Ixx, float *Iyy, float *Izz, float *Ixy, float *Iyz, float *Izx);
			void GetCurInertiaTensorInv();
			void GenerateGeom(std::vector <TriUV> & triList, ObjectPiece & objPiece);

		public:
			struct triRefInBVH
			{
				unsigned int pieceId, triId;
			};
			std::vector<triRefInBVH> triRefs;
			std::vector<BoxLeaf> boxLeaves;
			CWBVHClass cwbvh;
			std::vector <ObjectPiece> pieces;
			vec3 body_min, body_max;
			float radius;
			vec3 pos;
			mat4 orient;
			mat4 prevTrans;
			bool transOnce = false;

			bool isMap; // Is this object the map?
			bool Frozen; // Object Frozen because of inactivity

			vec3 lin_v;
			vec3 ang_v;
			float mass;
			mat4 inertia_tensor;
			mat4 inertia_tensor_inv;
			mat4 cur_inertia_tensor_inv;
			std::mutex enqueuedImpulseMutex;
			struct EnqueuedImpulse
			{
				vec3 impulse, pos;
			};
			std::vector<EnqueuedImpulse> enqueuedImpulses;

			~RigidBody();
			RigidBody();
			RigidBody(std::string & newGroupId, DataBlock &propBlock, std::vector <TriUV> & triList, vec3 inpNewCenter);
			RigidBody(Mesh &inpMesh, std::string belong, mat4 inpOrient, vec3 inpPos, bool inIsMap = false, std::function<bool(RigidBody*, int, DataGroup & )> inpFilterFunction = [](RigidBody*, int, DataGroup & inpGroup) -> bool {
				return true;
			}, vec3 *inpNewCenter = nullptr);
			void ApplyForce(vec3 force, vec3 point, float delta_t);
			void ApplyImpulse(vec3 impulse, vec3 point);
			void EnqeueImpulse(vec3 impulse, vec3 point);
			void ProcessEnqueuedImpulses();
			void Move(float delta_t);
			void InitMassInertiaTensorAndBoundingSpheres();
			void BuildBVH();

			void Update();
			void UpdateAABB();
			void ApplyBuoyancy(vec3 min, vec3 max, float rho, float b, float A, float CD, float dt);
			void ChangeGeom(std::string & groupId, std::vector <TriUV> & triList);
			ObjectPiece *getGroupById(std::string & groupId);
			void removeGroupById(std::string & groupId);
			static bool IntersectLeaf(vec3& linA, vec3& linB, void *rigidBody, unsigned int primId);
		};
		extern std::vector<RigidBody *> allBodies;
		enum CONSTRAINT_TYPE
		{
			POSITIONAL = 0,
			HINGE
		};
		class Constraint
		{
			friend class ConstraintCollection;
		private:
			CONSTRAINT_TYPE type;

			vec3 tanPtWrtParent;
			vec3 tanPtWrtChild;
			vec3 posWrtParent;
			vec3 posWrtChild;
			std::string parent;
			std::string child;
			unsigned int order;
		};
		class intersection
		{
		public:
			vec3 pt, n;
			RigidBody *a, *b;
			float elas, fric;
			MATERIAL m1;

			intersection(vec3 & in_pt, vec3 & in_n, RigidBody * in_a, RigidBody * in_b, float in_elas, float in_fric, MATERIAL in_mat);
		};
		class ConstraintCollection
		{
		private:
			unsigned int maxOrder = 0;
			std::unordered_map <std::string, RigidBody *> allLimbs;
			std::vector <Constraint> allConstraints;
		public:
			ConstraintCollection(Mesh & inpMesh, std::unordered_map <std::string, RigidBody *> & inpLimbs, mat4 inpTrans, InstanceClass &ptrToInstance);
			void ApplyConstraintForces(float inpDeltaT);
			void RandomShit();
		};
		extern std::vector <intersection> collisionNodes;
		extern std::vector <ConstraintCollection *> constraintCollection;
		#define HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT 4
		struct fizXThreadStruct {
			int threadIndex;
			bool stopped;
			bool quit;
			std::vector <intersection> nodes;
			std::mutex mutex;
			std::mutex quit_mutex;
			std::condition_variable condVar;
		};
		extern fizXThreadStruct collisionThreadParams[HIGHOMEGA_MAX_FIZ_X_THREAD_COUNT];
		extern std::vector<std::thread *> collisionThreads;
		struct fizXSignalStruct {
			bool started;
			bool quit;
			std::mutex quit_mutex;
		};
		extern fizXSignalStruct fizXSignal;
		struct lineHit
		{
			float k;
			vec3 n;
			ObjectPiece *piece;
			RigidBody *body;
		};
		struct OverlappingTri
		{
			vec3 e[3];
			bool n;
			MATERIAL mat;
			OverlappingTri(const vec3& e0, const vec3& e1, const vec3& e2, const bool& nFlip, const MATERIAL& inMat) : e{ e0, e1, e2 }, n(nFlip), mat(inMat) {}
		};
		void PiecePiece(ObjectPiece & a, ObjectPiece & b, RigidBody * aBody, RigidBody * bBody, std::vector <intersection> & intersect_list, bool quitOnIntersection = false);
		void CandidatesRigidBody(std::vector<unsigned int>& candidateList, RigidBody* aBody, RigidBody* bBody, std::vector <intersection>& intersect_list, bool quitOnIntersection = false);
		void RigidBodyRigidBody(RigidBody & a, RigidBody & b, std::vector <intersection> & intersect_list, bool quitOnIntersection = false);
		void BoxBodyOverlap(vec3& boxMin, vec3& boxMax, RigidBody& body, std::vector <OverlappingTri>& overlapList);
		bool LineMeshClosest(RigidBody& body, vec3& linA, vec3& linB, vec3& closestNorm, ObjectPiece** hitPiece);
		void BroadSweep();
		void ResolveCollisions();
		void PropagateShock();
		void ApplyFriction(float integrateAmount);
		void CollideMultiThread(void *threadId);
		unsigned int FindTimeStep();
		void MoveAll(float delta_t);
		void ApplyGravityBuoyancyEnqueuedImpulses(float delta_t);
		void PhysicsLoop();
	}
}