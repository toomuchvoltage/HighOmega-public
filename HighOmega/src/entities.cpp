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

#include "entities.h"

using namespace HIGHOMEGA::EVENTS;
using namespace HIGHOMEGA::RENDER;
using namespace HIGHOMEGA::FIZ_X;

float HIGHOMEGA::ENTITIES::viewerMouseSpeed = 0.1f;
HIGHOMEGA::ENTITIES::CharacterPhysics HIGHOMEGA::ENTITIES::player;

struct HIGHOMEGA::ENTITIES::entitiesSignalStruct HIGHOMEGA::ENTITIES::entitiesSignal;
HIGHOMEGA::ENTITIES::LadderSystemClass HIGHOMEGA::ENTITIES::mainLadderSystem;
void HIGHOMEGA::ENTITIES::EntitiesLoop (WorldParamsClass *WorldParams)
{
	mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
	player.bodyPos = WorldParams->StartPlayerPos();

	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		CommonSharedMutex.lock_shared();
		try
		{
			if (!player.ladderInfo.climbingLadder)
			{
				if (WorldParams->FirstPersonControls())
				{
					player.Move(GetStateOfKeybAction(CMD_MOVE_FORWARD),
						GetStateOfKeybAction(CMD_MOVE_BACKWARD),
						GetStateOfKeybAction(CMD_MOVE_LEFT),
						GetStateOfKeybAction(CMD_MOVE_RIGHT),
						GetStateOfKeybAction(CMD_JUMP),
						GetStateOfKeybAction(CMD_CROUCH));
				}

				mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

				bool insideLadderEntry = false;
				if (player.bodyStretch == 1.0f)
					for (std::pair <const unsigned long long, std::unordered_map<std::string, HIGHOMEGA::ENTITIES::LadderSystemClass::Ladder>> & curLadderList : mainLadderSystem.ladders)
						for (std::pair<const std::string, HIGHOMEGA::ENTITIES::LadderSystemClass::Ladder> & curLadder : curLadderList.second)
							if (player.GetEye().x > curLadder.second.entryMin.x &&
								player.GetEye().y > curLadder.second.entryMin.y &&
								player.GetEye().z > curLadder.second.entryMin.z &&
								player.GetEye().x < curLadder.second.entryMax.x &&
								player.GetEye().y < curLadder.second.entryMax.y &&
								player.GetEye().z < curLadder.second.entryMax.z)
							{
								if (GetStateOfKeybAction(CMD_USE))
								{
									player.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_ON;
									player.ladderInfo.climbingLadder = &curLadder.second;
									player.ladderInfo.transitionFraction = 0.0f;
									player.ladderInfo.curEye = player.GetEye();
									player.ladderInfo.curLook = player.lookDir;
								}
								else
									insideLadderEntry = true;
								break;
							}
				player.ladderInfo.inEntryZone = insideLadderEntry;

				MainFrustum.entitiesLook = player.lookDir;
				MainFrustum.entitiesEye = player.GetEye();
			}
			else
			{
				switch (player.ladderInfo.transitionState)
				{
					case CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_ON:
						player.ladderInfo.transitionFraction += (1.0f - player.ladderInfo.transitionFraction) * 0.1f;
						player.ladderInfo.curEye += (player.ladderInfo.climbingLadder->start - player.ladderInfo.curEye) * 0.1f;
						player.ladderInfo.curLook += (player.ladderInfo.climbingLadder->stareDir - player.ladderInfo.curLook) * 0.1f;
						player.ladderInfo.curLook = player.ladderInfo.curLook.normalized();

						if (player.ladderInfo.transitionFraction > 0.9f)
						{
							player.ladderInfo.curEye = player.ladderInfo.climbingLadder->start;
							player.ladderInfo.curLook = player.ladderInfo.climbingLadder->stareDir;
							player.ladderInfo.transitionFraction = 0.0f;
							player.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING;
						}
						break;
					case CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING:
						{
							vec3 ladderDiff = player.ladderInfo.climbingLadder->end - player.ladderInfo.climbingLadder->start;
							float ladderLen = ladderDiff.length();
							vec3 ladderMoveDir = ladderDiff / ladderLen;
							if (GetStateOfKeybAction(CMD_MOVE_FORWARD))
							{
								player.ladderInfo.curEye += ladderMoveDir * 0.1f;
							}
							else if (GetStateOfKeybAction(CMD_MOVE_BACKWARD))
							{
								player.ladderInfo.curEye -= ladderMoveDir * 0.1f;
							}
							vec3 eyeToLadderStartDiff = player.ladderInfo.curEye - player.ladderInfo.climbingLadder->start;
							float ladderProgress = eyeToLadderStartDiff.length() / ladderLen;
							if (eyeToLadderStartDiff * ladderDiff < 0.0f)
							{
								player.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING_OFF_BOTTOM;
								player.ladderInfo.curEye = player.ladderInfo.climbingLadder->start;
							}
							else if (ladderProgress > 1.0f)
							{
								player.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_OFF;
								player.ladderInfo.curEye = player.ladderInfo.climbingLadder->end;
							}
							else if (GetStateOfKeybAction(CMD_USE))
							{
								player.lookDir = player.ladderInfo.curLook;
								player.ForceLookVector(player.ladderInfo.curLook);
								player.ForcePosFromEye(player.ladderInfo.curEye);
								player.playerVel = vec3(0.0f);

								player.ladderInfo.climbingLadder = nullptr;
								mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
							}
						}
						break;
					case CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING_OFF_BOTTOM:
						{
							player.ladderInfo.transitionFraction += (1.0f - player.ladderInfo.transitionFraction) * 0.1f;
							player.ladderInfo.curEye += (player.ladderInfo.climbingLadder->letOff - player.ladderInfo.curEye) * 0.1f;
							if (player.ladderInfo.transitionFraction > 0.9f)
							{
								player.ladderInfo.curEye = player.ladderInfo.climbingLadder->letOff;
								player.ladderInfo.transitionFraction = 0.0f;

								player.lookDir = player.ladderInfo.climbingLadder->stareDir;
								player.ForceLookVector(player.lookDir);
								player.ForcePosFromEye(player.ladderInfo.curEye);
								player.playerVel = vec3(0.0f);

								player.ladderInfo.climbingLadder = nullptr;
								mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
							}
						}
						break;
					case CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_OFF:
					{
						player.ladderInfo.transitionFraction += (1.0f - player.ladderInfo.transitionFraction) * 0.1f;
						player.ladderInfo.curEye += (player.ladderInfo.climbingLadder->dropOff - player.ladderInfo.curEye) * 0.1f;
						if (player.ladderInfo.transitionFraction > 0.9f)
						{
							player.ladderInfo.curEye = player.ladderInfo.climbingLadder->dropOff;
							player.ladderInfo.transitionFraction = 0.0f;

							player.lookDir = player.ladderInfo.climbingLadder->stareDir;
							player.ForceLookVector(player.lookDir);
							player.ForcePosFromEye(player.ladderInfo.curEye);
							player.playerVel = vec3(0.0f);

							player.ladderInfo.climbingLadder = nullptr;
							mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
						}
					}
					break;
				}

				MainFrustum.entitiesLook = player.ladderInfo.curLook;
				MainFrustum.entitiesEye = player.ladderInfo.curEye;
			}
		}
		catch (...)
		{
			CommonSharedMutex.unlock_shared();
			return;
		}
		CommonSharedMutex.unlock_shared();

		double timerDiff = 10.0 - timerObj.Diff()*1000.0;
		if ( timerDiff > 0.0 ) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{std::unique_lock<std::mutex> lk(entitiesSignal.quit_mutex); readQuit = entitiesSignal.quit; }
		if (readQuit) return;
	}
}

unsigned long long HIGHOMEGA::ENTITIES::LadderSystemClass::Populate(Mesh & inpMesh)
{
	std::vector <RasterVertex> verts;
	vec3 entryMin, entryMax, curPos, tmpVec3;
	vec2 tmpVec2;
	unsigned long long curLaddersId = mersenneTwister64BitPRNG();
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		std::string ladderName;
		if (!Mesh::getDataRowString(curPolyGroup, "PROPS", "ladder", ladderName)) continue;

		HIGHOMEGA::MESH::DataBlock *triBlock = nullptr;
		Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock);
		std::string ladderComponent;
		Mesh::getDataRowString(curPolyGroup, "PROPS", "component", ladderComponent);

		if (ladderComponent == "entry")
		{
			verts.clear();
			verts.resize(triBlock->blob.size() / sizeof(RasterVertex));
			memcpy(verts.data(), triBlock->blob.data(), triBlock->blob.size());
			for (unsigned int i = 0; i != verts.size(); i++)
			{
				unpackRasterVertex(curPos, tmpVec3, tmpVec2, tmpVec3, verts[i]);
				if (i == 0)
				{
					entryMax = entryMin = curPos;
				}
				else
				{
					entryMin.x = min(entryMin.x, curPos.x);
					entryMin.y = min(entryMin.y, curPos.y);
					entryMin.z = min(entryMin.z, curPos.z);
					entryMax.x = max(entryMax.x, curPos.x);
					entryMax.y = max(entryMax.y, curPos.y);
					entryMax.z = max(entryMax.z, curPos.z);
				}
			}
			ladders[curLaddersId][ladderName].entryMin = entryMin;
			ladders[curLaddersId][ladderName].entryMax = entryMax;
		}
		else if (ladderComponent == "start")
		{
			Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", ladders[curLaddersId][ladderName].start);
			Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "dir", ladders[curLaddersId][ladderName].stareDir);
		}
		else if (ladderComponent == "end")
		{
			Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", ladders[curLaddersId][ladderName].end);
		}
		else if (ladderComponent == "dropoff")
		{
			Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", ladders[curLaddersId][ladderName].dropOff);
		}
		else if (ladderComponent == "letoff")
		{
			Mesh::getDataRowVec3(curPolyGroup, "DESCRIPTION", "pos", ladders[curLaddersId][ladderName].letOff);
		}
		else
			continue;
	}

	return curLaddersId;
}

void HIGHOMEGA::ENTITIES::LadderSystemClass::Combine(std::vector<LadderSystemClass> & inLadderSystems)
{
	for (LadderSystemClass & curLadder : inLadderSystems)
		for (std::pair<const unsigned long long, std::unordered_map <std::string, Ladder>> & curDesc : curLadder.ladders)
			ladders[curDesc.first] = curDesc.second;

	for (LadderSystemClass & curLadder : inLadderSystems)
		curLadder.ClearContent();
}

void HIGHOMEGA::ENTITIES::LadderSystemClass::Remove(unsigned long long inpGeomId)
{
	ladders.erase(inpGeomId);
}

void HIGHOMEGA::ENTITIES::LadderSystemClass::ClearContent()
{
	ladders.clear();
}

HIGHOMEGA::ENTITIES::CharacterPhysics::CharacterPhysics()
{
	bodyPos = vec3(0.0f, 20.0f, 0.0f);
	bodyDir = vec3 (0.0f,1.0f,0.0f);
	lookDir = vec3(0.0f, 0.0f, -1.0f);
	playerVel = vec3(0.0f);
	bodyRad = 5.0f;
	bodyStretchStandingFraction = bodyStretch = 1.0f;
	bodyStretchSittingFraction = 0.5f;
	slopeForWalking = -0.7f;
	walkSpeed = 0.5f;
	crouchSlowDown = 0.5f;
	gravStrength = 0.05f;
	jumpStrength = 1.0f;
	feetOnGround = false;
}

vec3 HIGHOMEGA::ENTITIES::CharacterPhysics::ProcessLookVector(vec2 mouseDxDy)
{
	float ang_v = MainFrustum.screen_fov;
	float ang_h = MainFrustum.screen_fov;

	vec3 up_dir = MainFrustum.up;

	lookVectorXZAngle += ((float)(mouseDxDy.x) / ScreenSize.width) * ang_h * viewerMouseSpeed;
	lookVectorYAngle += ((float)(mouseDxDy.y) / ScreenSize.height) * ang_v * viewerMouseSpeed;

	if (lookVectorXZAngle > 2.0f*HIGHOMEGA_PI) lookVectorXZAngle -= 2.0f*HIGHOMEGA_PI;
	if (lookVectorXZAngle < -2.0f*HIGHOMEGA_PI) lookVectorXZAngle += 2.0f*HIGHOMEGA_PI;
	if (lookVectorYAngle < -HIGHOMEGA_PI / 2.0f + 0.001f) lookVectorYAngle = -HIGHOMEGA_PI / 2.0f + 0.001f;
	if (lookVectorYAngle > HIGHOMEGA_PI / 2.0f - 0.001f) lookVectorYAngle = HIGHOMEGA_PI / 2.0f - 0.001f;

	vec3 side = cross(up_dir, up_dir + vec3(0.01f)).normalized(); // Trying to form two basis vectors for eye vector
	vec3 forth = cross(side, up_dir).normalized();

	return cos(lookVectorYAngle)*cos(lookVectorXZAngle)*side + sin(lookVectorYAngle)*up_dir + cos(lookVectorYAngle)*sin(lookVectorXZAngle)*forth;
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::ForceLookVector(vec3 inLookVector)
{
	vec3 up_dir = MainFrustum.up;

	vec3 side = cross(up_dir, up_dir + vec3(0.01f)).normalized(); // Trying to form two basis vectors for eye vector
	vec3 forth = cross(side, up_dir).normalized();

	lookVectorYAngle = (HIGHOMEGA_PI / 2.0f) - acosf(inLookVector * up_dir);

	lookVectorXZAngle = acosf(inLookVector * side);
	if (inLookVector * forth < 0.0f) lookVectorXZAngle = 2.0f*HIGHOMEGA_PI - lookVectorXZAngle;
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::BodySphereTriOverlap(std::vector<SpherePushOutTri>& isecTris, vec3 & sphereMin, vec3 & sphereMax, unsigned int threadId)
{
	vec3 e[3], s[3], e_min, e_max, n, e1_e0, e2_e1, e0_e2;

	for (int i = 0; i != allBodies.size(); i++)
	{
		RigidBody *curBody = allBodies[i];
		if (!curBody->isMap)
		{
			if (sphereMin.x > allBodies[i]->body_max.x ||
				sphereMin.y > allBodies[i]->body_max.y ||
				sphereMin.z > allBodies[i]->body_max.z ||
				sphereMax.x < allBodies[i]->body_min.x ||
				sphereMax.y < allBodies[i]->body_min.y ||
				sphereMax.z < allBodies[i]->body_min.z) continue;
		}
		for (int j = 0; j != curBody->pieces.size(); j++)
		{
			ObjectPiece & curPiece = curBody->pieces[j];
			if (curBody->isMap)
			{
				if (sphereMin.x > curPiece.piece_max.x ||
					sphereMin.y > curPiece.piece_max.y ||
					sphereMin.z > curPiece.piece_max.z ||
					sphereMax.x < curPiece.piece_min.x ||
					sphereMax.y < curPiece.piece_min.y ||
					sphereMax.z < curPiece.piece_min.z) continue;
			}
			for (int k = 0; k != curPiece.tris.size(); k++)
			{
				if (k % HIGHOMEGA_PLAYER_PHYSICS_THREADS != threadId) continue;
				ObjectTri & curTri = curPiece.tris[k];
				e[0] = curTri.e1;
				e[1] = curTri.e2;
				e[2] = curTri.e3;
				n = curTri.n;
				e1_e0 = e[1] - e[0];
				e2_e1 = e[2] - e[1];
				e0_e2 = e[0] - e[2];
				s[0] = cross(n, e1_e0);
				s[1] = cross(n, e2_e1);
				s[2] = cross(n, e0_e2);
				if (e2_e1 * s[0] > 0.0f) s[0] = -s[0];
				if (e0_e2 * s[1] > 0.0f) s[1] = -s[1];
				if (e1_e0 * s[2] > 0.0f) s[2] = -s[2];
				e_min.x = Min(e[0].x, e[1].x, e[2].x);
				e_min.y = Min(e[0].y, e[1].y, e[2].y);
				e_min.z = Min(e[0].z, e[1].z, e[2].z);
				e_max.x = Max(e[0].x, e[1].x, e[2].x);
				e_max.y = Max(e[0].y, e[1].y, e[2].y);
				e_max.z = Max(e[0].z, e[1].z, e[2].z);
				if (sphereMin.x > e_max.x || sphereMin.y > e_max.y || sphereMin.z > e_max.z ||
					sphereMax.x < e_min.x || sphereMax.y < e_min.y || sphereMax.z < e_min.z)
				{
				}
				else
				{
					isecTris.emplace_back(e[0], e[1], e[2], s[0], s[1], s[2], n);
				}
			}
		}
	}
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::FootTriOverlap(vec3 * footTri1, vec3 & footTri1Norm, vec3 * footTri2, vec3 & footTri2Norm, vec3 & footTri1Min, vec3 & footTri1Max, vec3 & footTri2Min, vec3 & footTri2Max, vec3 & footTrisMin, vec3 & footTrisMax, bool & footOnGround, MATERIAL & inpMat, unsigned int threadId)
{
	vec3 e[3], s[3], e_min, e_max, n, e1_e0, e2_e1, e0_e2;
	vec3 tmp1, tmp2;

	for (RigidBody *curBody : allBodies)
	{
		if (!curBody->isMap)
		{
			if (footTrisMin.x > curBody->body_max.x ||
				footTrisMin.y > curBody->body_max.y ||
				footTrisMin.z > curBody->body_max.z ||
				footTrisMax.x < curBody->body_min.x ||
				footTrisMax.y < curBody->body_min.y ||
				footTrisMax.z < curBody->body_min.z) continue;
		}
		for (ObjectPiece & curPiece : curBody->pieces)
		{
			if (curBody->isMap)
			{
				if (footTrisMin.x > curPiece.piece_max.x ||
					footTrisMin.y > curPiece.piece_max.y ||
					footTrisMin.z > curPiece.piece_max.z ||
					footTrisMax.x < curPiece.piece_min.x ||
					footTrisMax.y < curPiece.piece_min.y ||
					footTrisMax.z < curPiece.piece_min.z) continue;
			}
			for (int k = 0; k != curPiece.tris.size(); k++)
			{
				if (k % HIGHOMEGA_PLAYER_PHYSICS_THREADS != threadId) continue;
				ObjectTri & curTri = curPiece.tris[k];
				e[0] = curTri.e1;
				e[1] = curTri.e2;
				e[2] = curTri.e3;
				n = curTri.n;
				TRITRI_INTERSECT_MODE intMode;

				e_min.x = Min(e[0].x, e[1].x, e[2].x);
				e_min.y = Min(e[0].y, e[1].y, e[2].y);
				e_min.z = Min(e[0].z, e[1].z, e[2].z);
				e_max.x = Max(e[0].x, e[1].x, e[2].x);
				e_max.y = Max(e[0].y, e[1].y, e[2].y);
				e_max.z = Max(e[0].z, e[1].z, e[2].z);

				if (footTri1Min.x > e_max.x || footTri1Min.y > e_max.y || footTri1Min.z > e_max.z ||
					footTri1Max.x < e_min.x || footTri1Max.y < e_min.y || footTri1Max.z < e_min.z)
				{
				}
				else
					if (TriTri(footTri1, footTri1Norm, e, n, &tmp1, &tmp2, &intMode))
					{
						inpMat = curPiece.mat;
						footOnGround = true;
						return;
					}

				if (footTri2Min.x > e_max.x || footTri2Min.y > e_max.y || footTri2Min.z > e_max.z ||
					footTri2Max.x < e_min.x || footTri2Max.y < e_min.y || footTri2Max.z < e_min.z)
				{
				}
				else
					if (TriTri(footTri2, footTri2Norm, e, n, &tmp1, &tmp2, &intMode))
					{
						inpMat = curPiece.mat;
						footOnGround = true;
						return;
					}
			}
		}
	}
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::Move(bool fwd, bool back, bool left, bool right, bool jump, bool crouch)
{
	lookDir = ProcessLookVector(mouseMovementConsumers["PlayerHead"]).normalized();
	mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

	vec3 side = cross(lookDir, -GravDir).normalized();
	vec3 fwdDir = cross(side, GravDir).normalized();
	if (fwdDir*lookDir < 0.0f) fwdDir = -fwdDir;

	vec3 moveVel = vec3(0.0f);

	if (fwd) {
		moveVel += fwdDir;
	}
	if (back) {
		moveVel -= fwdDir;
	}
	if (right) {
		moveVel += side;
	}
	if (left) {
		moveVel -= side;
	}
	if (moveVel.length() > 0.0f)
	{
		moveVel = moveVel.normalized()*walkSpeed;
		if (crouch) moveVel *= crouchSlowDown;
	}

	if (!feetOnGround)
		playerVel += GravDir*gravStrength;
	else
	{
		playerVel = moveVel;
		if (!crouch && jump) playerVel += -GravDir*jumpStrength;
	}

	int maxCi = (int)(playerVel.length() + 1.0f);

	for (int ci = 0; ci != maxCi; ci++)
	{
		bodyPos += playerVel / (float)maxCi;
		for (int i = 2; i != -1; i--)
		{
			if (crouch)
			{
				bodyStretch += (bodyStretchSittingFraction - bodyStretch)*0.1f;
				if (fabs(bodyStretchSittingFraction - bodyStretch) < 0.1f) bodyStretch = bodyStretchSittingFraction;
			}
			else
			{
				bodyStretch += (bodyStretchStandingFraction - bodyStretch)*0.1f;
				if (fabs(bodyStretchStandingFraction - bodyStretch) < 0.1f) bodyStretch = bodyStretchStandingFraction;
			}

			lookDir = ProcessLookVector(mouseMovementConsumers["PlayerHead"]).normalized();
			mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

			vec3 currentSphere = bodyPos + bodyDir*((float)(i + 1))*bodyRad*bodyStretch;
			vec3 currentSphereMoved = currentSphere;
			vec3 sphereMax = currentSphere + vec3(bodyRad);
			vec3 sphereMin = currentSphere - vec3(bodyRad);

			for (int j = 0; j != HIGHOMEGA_PLAYER_PHYSICS_THREADS; j++)
				PushOutThreads[j] = new std::thread(BodySphereTriOverlap, std::ref(PushOutTris[j]), std::ref(sphereMin), std::ref(sphereMax), j);
			for (int j = 0; j != HIGHOMEGA_PLAYER_PHYSICS_THREADS; j++)
				PushOutThreads[j]->join();

			for (int j = 0; j != HIGHOMEGA_PLAYER_PHYSICS_THREADS; j++)
				for (SpherePushOutTri & curPushOutTri : PushOutTris[j])
					PushOutSphere(curPushOutTri.e, curPushOutTri.s, curPushOutTri.n, &currentSphereMoved, bodyRad);

			for (int j = 0; j != HIGHOMEGA_PLAYER_PHYSICS_THREADS; j++)
				delete PushOutThreads[j];

			for (int j = 0; j != HIGHOMEGA_PLAYER_PHYSICS_THREADS; j++)
				PushOutTris[j].clear();

			sphereMax = currentSphereMoved + vec3(bodyRad);
			sphereMin = currentSphereMoved - vec3(bodyRad);
			bodyPos += (currentSphereMoved - currentSphere);
		}
	}

	lookDir = ProcessLookVector(mouseMovementConsumers["PlayerHead"]).normalized();
	mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

	feetOnGround = false;

	vec3 foot_side = cross (bodyDir, bodyDir + vec3(0.1f)).normalized();
	vec3 foot_front = cross(foot_side, bodyDir).normalized();

	vec3 foot_tri1[3] = { bodyPos,bodyPos - bodyDir*3.0f + foot_side*bodyRad,bodyPos - bodyDir*3.0f - foot_side*bodyRad };
	vec3 foot_tri2[3] = { bodyPos,bodyPos - bodyDir*3.0f + foot_front*bodyRad,bodyPos - bodyDir*3.0f - foot_front*bodyRad };

	vec3 foot_tri1_min = vec3(Min(foot_tri1[0].x, foot_tri1[1].x, foot_tri1[2].x), Min(foot_tri1[0].y, foot_tri1[1].y, foot_tri1[2].y), Min(foot_tri1[0].z, foot_tri1[1].z, foot_tri1[2].z));
	vec3 foot_tri1_max = vec3(Max(foot_tri1[0].x, foot_tri1[1].x, foot_tri1[2].x), Max(foot_tri1[0].y, foot_tri1[1].y, foot_tri1[2].y), Max(foot_tri1[0].z, foot_tri1[1].z, foot_tri1[2].z));

	vec3 foot_tri2_min = vec3(Min(foot_tri2[0].x, foot_tri2[1].x, foot_tri2[2].x), Min(foot_tri2[0].y, foot_tri2[1].y, foot_tri2[2].y), Min(foot_tri2[0].z, foot_tri2[1].z, foot_tri2[2].z));
	vec3 foot_tri2_max = vec3(Max(foot_tri2[0].x, foot_tri2[1].x, foot_tri2[2].x), Max(foot_tri2[0].y, foot_tri2[1].y, foot_tri2[2].y), Max(foot_tri2[0].z, foot_tri2[1].z, foot_tri2[2].z));

	vec3 foot_tris_min, foot_tris_max;
	foot_tris_min = vec3(min(foot_tri1_min.x, foot_tri2_min.x), min(foot_tri1_min.y, foot_tri2_min.y), min(foot_tri1_min.z, foot_tri2_min.z));
	foot_tris_max = vec3(max(foot_tri1_max.x, foot_tri2_max.x), max(foot_tri1_max.y, foot_tri2_max.y), max(foot_tri1_max.z, foot_tri2_max.z));

	for (int i = 0; i != HIGHOMEGA_PLAYER_PHYSICS_THREADS; i++)
		PushOutThreads[i] = new std::thread(FootTriOverlap, foot_tri1, std::ref(foot_front), foot_tri2, std::ref(foot_side), std::ref(foot_tri1_min), std::ref(foot_tri1_max), std::ref(foot_tri2_min), std::ref(foot_tri2_max), std::ref(foot_tris_min), std::ref(foot_tris_max), std::ref(feetOnGround), std::ref(groundMat), i);
	for (int i = 0; i != HIGHOMEGA_PLAYER_PHYSICS_THREADS; i++)
		PushOutThreads[i]->join();

	for (int i = 0; i != HIGHOMEGA_PLAYER_PHYSICS_THREADS; i++)
		delete PushOutThreads[i];
}

vec3 HIGHOMEGA::ENTITIES::CharacterPhysics::GetEye()
{
	return bodyPos + bodyDir*3.0f*bodyRad*bodyStretch;
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::ForcePosFromEye(vec3 inEye)
{
	bodyPos = inEye - bodyDir * 3.0f*bodyRad*bodyStretch;
}
