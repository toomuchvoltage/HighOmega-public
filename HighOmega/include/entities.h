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

#include "util.h"
#include "events.h"
#include "render.h"
#include "fiz-x.h"
#include "geom.h"
#include "audio.h"
#include <vector>

using namespace HIGHOMEGA::GEOM;
using namespace HIGHOMEGA::FIZ_X;
using namespace HIGHOMEGA::AUDIO;

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
		struct FireLine
		{
			vec3 orig;
			vec3 dir;
		};
		extern std::vector<FireLine> fireLines;
		extern std::mutex fireLineMutex;
		class CharacterPhysics;
		class CharacterAudio
		{
		public:
			struct walkingSound
			{
				bool curPlaying = false;
				bool prevPlaying = false;
				unsigned long long walkSound = 0ul;
				float initialVolume = 1.0f;
				std::string path = "";
			};
			std::unordered_map<HIGHOMEGA::FIZ_X::MATERIAL, walkingSound> walkingSounds;
			bool playingLadderSound = false;
			unsigned long long climbSoundLadder = 0ul;

			void Process(CharacterPhysics& playerPhysicsRef);
		};
		class CharacterProps
		{
		public:
			void Process(bool isOnLadder, CharacterPhysics& playerPhysicsRef, CharacterAudio& playerAudioRef);
		};
		class LadderSystemClass
		{
		public:
			struct Ladder
			{
				std::string name;
				vec3 entryMin, entryMax;
				vec3 start, stareDir, end, dropOff, letOff;
			};
			std::unordered_map <unsigned long long, std::vector<Ladder>> ladders;

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
			float lateralSwingAngle = 0.0f;
			vec3 ProcessLookVector(vec2 mouseDxDy);
			std::vector<OverlappingTri> CloseToBodyTris;
			std::vector<vec3> footNormals;

		public:
			float lookVectorXZAngle = 0.0f;
			float lookVectorYAngle = 0.0f;
			PlayerClothObstacles* playerClothObstacle = nullptr;
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
			vec3 bodyPos, prevBodyPos, bodyDir, lookDir, playerVel;
			unsigned int countStuckFrames;
			float bodyRad, bodyStretch, bodyStretchSittingFraction, bodyStretchStandingFraction;
			float slopeForWalking, walkSpeed, crouchSlowDown;
			float gravStrength, jumpStrength;
			bool feetOnGround;
			bool feetStuck;
			bool quickWalk;
			MATERIAL groundMat;
			CharacterPhysics();
			void GenerateFeet(vec3& inBodyPos, vec3 feetTri1[3], vec3 feetTri2[3], vec3& feetTri1Min, vec3& feetTri1Max, vec3& feetTri2Min, vec3& feetTri2Max, vec3& feetFront, vec3& feetSide, vec3& feetTriMin, vec3& feetTriMax);
			static void MovingBodyTriOverlap(std::vector<OverlappingTri> & overlappingTris, vec3 & boxMin, vec3 & boxMax);
			static void FootTriOverlap(std::vector<OverlappingTri>& isecTris, vec3 * footTri1, vec3 & footTri1Norm, vec3 * footTri2, vec3 & footTri2Norm, vec3 & footTri1Min, vec3 & footTri1Max, vec3 & footTri2Min, vec3 & footTri2Max, vec3 & footTrisMin, vec3 & footTrisMax, std::vector<vec3>& footNormals, MATERIAL & inpMat);
			void Move(bool fwd,bool back,bool left,bool right,bool jump,bool crouch);
			void UpdateClothObstacles(unsigned long long playerId);
			vec3 GetEye();
			float GetStandingHeight();
			float GetLateralSwingAngle();
			void ForcePosFromEye(vec3 inEye);
		};
		class Character
		{
		public:
			unsigned long long id = 0ul;
			CharacterPhysics physics;
			CharacterAudio audio;
			CharacterProps props;

			void Process(WorldParamsClass& WorldParams);
			Character();
		};
		extern Character player;
	}
}