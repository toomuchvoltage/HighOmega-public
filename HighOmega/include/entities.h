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

#ifndef HIGHOMEGA_ENTITIES_H
#define HIGHOMEGA_ENTITIES_H

#include "util.h"
#include "events.h"
#include "render.h"
#include "fiz-x.h"
#include "geom.h"
#include <vector>

using namespace HIGHOMEGA::GEOM;
using namespace HIGHOMEGA::FIZ_X;

namespace HIGHOMEGA
{
	namespace ENTITIES
	{
		struct entitiesSignalStruct {
			bool quit;
			std::mutex quit_mutex;
		};
		extern entitiesSignalStruct entitiesSignal;
		void EntitiesLoop(WorldParamsClass *WorldParams);
		extern float viewerMouseSpeed;
		extern float viewerKeybSpeed;
		class LadderSystemClass
		{
		public:
			struct Ladder
			{
				vec3 entryMin, entryMax;
				vec3 start, stareDir, end, dropOff, letOff;
			};
			std::unordered_map <unsigned long long, std::unordered_map<std::string, Ladder>> ladders;

			unsigned long long Populate(Mesh & inpMesh);
			void Combine(std::vector<LadderSystemClass> & inLadderSystems);
			void Remove(unsigned long long inpGeomId);
			void ClearContent();
		};
		extern LadderSystemClass mainLadderSystem;
#define HIGHOMEGA_PLAYER_PHYSICS_THREADS 4
		class CharacterPhysics
		{
		private:
			float lookVectorXZAngle = 0.0f;
			float lookVectorYAngle = 0.0f;
			vec3 ProcessLookVector(vec2 mouseDxDy);
			struct SpherePushOutTri
			{
				vec3 e[3], s[3], n;
				SpherePushOutTri(vec3 & e0, vec3 & e1, vec3 & e2, vec3 & s0, vec3 & s1, vec3 & s2, vec3 & inN) {
					e[0] = e0;
					e[1] = e1;
					e[2] = e2;
					s[0] = s0;
					s[1] = s1;
					s[2] = s2;
					n = inN;
				}
			};
			std::vector<SpherePushOutTri> PushOutTris[HIGHOMEGA_PLAYER_PHYSICS_THREADS];
			std::thread *PushOutThreads[HIGHOMEGA_PLAYER_PHYSICS_THREADS];

		public:
			enum LADDER_TRANSITION_STATE
			{
				GETTING_ON = 0,
				CLIMBING,
				GETTING_OFF,
				CLIMBING_OFF_BOTTOM
			};
			struct
			{
				bool inEntryZone = false;
				LadderSystemClass::Ladder *climbingLadder = nullptr;
				LADDER_TRANSITION_STATE transitionState = GETTING_ON;
				vec3 curEye = vec3(0.0f);
				vec3 curLook = vec3(0.0f);
				float transitionFraction = 0.0f;
			} ladderInfo;
			void ForceLookVector(vec3 inLookVector);
			vec3 bodyPos, bodyDir, lookDir, playerVel;
			float bodyRad, bodyStretch, bodyStretchSittingFraction, bodyStretchStandingFraction;
			float slopeForWalking, walkSpeed, crouchSlowDown;
			float gravStrength, jumpStrength;
			bool feetOnGround;
			MATERIAL groundMat;
			CharacterPhysics();
			static void BodySphereTriOverlap(std::vector<SpherePushOutTri> & isecTris, vec3 & sphereMin, vec3 & sphereMax, unsigned int threadId);
			static void FootTriOverlap(vec3 * footTri1, vec3 & footTri1Norm, vec3 * footTri2, vec3 & footTri2Norm, vec3 & footTri1Min, vec3 & footTri1Max, vec3 & footTri2Min, vec3 & footTri2Max, vec3 & footTrisMin, vec3 & footTrisMax, bool & footOnGround, MATERIAL & inpMat, unsigned int threadId);
			void Move(bool fwd,bool back,bool left,bool right,bool jump,bool crouch);
			vec3 GetEye();
			void ForcePosFromEye(vec3 inEye);
		};
		extern CharacterPhysics player;
	}
}

#endif