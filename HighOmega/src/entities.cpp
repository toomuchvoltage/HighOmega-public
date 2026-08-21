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

#include "entities.h"

using namespace HIGHOMEGA::EVENTS;
using namespace HIGHOMEGA::RENDER;
using namespace HIGHOMEGA::FIZ_X;

float HIGHOMEGA::ENTITIES::viewerMouseSpeed = 0.1f;
HIGHOMEGA::ENTITIES::Character HIGHOMEGA::ENTITIES::player;

std::vector<HIGHOMEGA::ENTITIES::FireLine> HIGHOMEGA::ENTITIES::fireLines;
std::mutex HIGHOMEGA::ENTITIES::fireLineMutex;

struct HIGHOMEGA::ENTITIES::entitiesSignalStruct HIGHOMEGA::ENTITIES::entitiesSignal;
HIGHOMEGA::ENTITIES::LadderSystemClass HIGHOMEGA::ENTITIES::mainLadderSystem;
void HIGHOMEGA::ENTITIES::EntitiesLoop (WorldParamsClass *WorldParams)
{
	mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
	player.physics.bodyPos = WorldParams->StartPlayerPos();

	for (;;)
	{
		TimerObject timerObj;
		timerObj.Start();

		CommonSharedMutex.lock_shared();
		try
		{
			player.Process(*WorldParams);
		}
		catch (...)
		{
			CommonSharedMutex.unlock_shared();
			break;
		}
		CommonSharedMutex.unlock_shared();

		double timerDiff = 10.0 - timerObj.Diff()*1000.0;
		if ( timerDiff > 0.0 ) std::this_thread::sleep_for(std::chrono::milliseconds((int)timerDiff));
		else std::this_thread::sleep_for(std::chrono::milliseconds(1));

		bool readQuit;
		{std::unique_lock<std::mutex> lk(entitiesSignal.quit_mutex); readQuit = entitiesSignal.quit; }
		if (readQuit) break;
	}
	allObstaclesMutex.lock();
	allObstacles.erase(player.id);
	allObstaclesMutex.unlock();
}

unsigned long long HIGHOMEGA::ENTITIES::LadderSystemClass::Populate(Mesh & inpMesh)
{
	vec3 curPos, tmpVec3, tmpNorm;
	vec2 tmpVec2;
	unsigned long long curLaddersId = threadSafeMersenneTwister64Bit();
	for (int i = 0; i != inpMesh.DataGroups.size(); i++)
	{
		HIGHOMEGA::MESH::DataGroup &curPolyGroup = inpMesh.DataGroups[i];

		float tmpFloat;
		if (!Mesh::getDataRowFloat(curPolyGroup, "PROPS", "ladder", tmpFloat)) continue;

		HIGHOMEGA::MESH::DataBlock *triBlock = nullptr;
		Mesh::getDataBlock(curPolyGroup, "TRIS", &triBlock);

		RasterVertex* verts; unsigned int* indices, triCount, vertCount, vertexDataOffset;
		std::vector<unsigned char>& blobData = triBlock->blob;
		getIndicesVertices(blobData, &verts, vertCount, &indices, triCount, vertexDataOffset);

		if (vertCount != 4) FATAL_ERROR("Exactly four vertices are expected of ladder geom");

		std::vector<vec3> fourCorners;
		for (unsigned int i = 0; i != vertCount; i++)
		{
			unpackRasterVertex(curPos, tmpVec3, tmpVec2, tmpNorm, verts[i]);
			fourCorners.push_back(curPos);
		}

		std::sort(fourCorners.begin(), fourCorners.end(), [](const vec3& a, const vec3& b) -> bool {
			return (b.y < a.y);
		});

		vec3 ladderNorm = cross(fourCorners[0] - fourCorners[1], fourCorners[2] - fourCorners[1]).normalized();
		if (ladderNorm * tmpNorm < 0.0f) ladderNorm = -ladderNorm;
		vec3 bottomMid = (fourCorners[3] + fourCorners[2]) * 0.5f;
		vec3 topMid = (fourCorners[0] + fourCorners[1]) * 0.5f;
		vec3 toClimber = ladderNorm;
		toClimber.y = 0.0f;
		if (toClimber.length() > 0.0f)
			toClimber = toClimber.normalized();
		else
		{
			toClimber = bottomMid - topMid;
			toClimber.y = 0.0f;
			toClimber = toClimber.normalized();
		}
		vec3 entryPoint = bottomMid + toClimber * 5.0f;

		if (fabs(fourCorners[3].y - fourCorners[0].y) < 15.0f)
			entryPoint.y += 10.0f;
		else
			entryPoint.y += 15.0f;

		Ladder& curLadder = ladders[curLaddersId].emplace_back();
		curLadder.name = curPolyGroup.name + "_fwd";
		curLadder.entryMin = entryPoint - vec3(5.0f);
		curLadder.entryMax = entryPoint + vec3(5.0f);
		curLadder.stareDir = -ladderNorm;
		curLadder.start = bottomMid + ladderNorm * 5.0f + (topMid - bottomMid).normalized() * 18.0f;
		curLadder.letOff = entryPoint + toClimber;
		curLadder.end = topMid + ladderNorm * 5.0f;
		curLadder.dropOff = topMid - toClimber + vec3(0.0f, 18.0f, 0.0);

		vec3 topDropOff = curLadder.dropOff;
		vec3 bottomStart = curLadder.start;
		vec3 bottomEnd = curLadder.end;
		vec3 bottomLetOff = curLadder.letOff;
		entryPoint = curLadder.dropOff - vec3(0.0f, 8.0f, 0.0f);

		Ladder& curLadder2 = ladders[curLaddersId].emplace_back();
		curLadder2.name = curPolyGroup.name + "_bwd";
		curLadder2.entryMin = entryPoint - vec3(5.0f);
		curLadder2.entryMax = entryPoint + vec3(5.0f);
		curLadder2.stareDir = -ladderNorm;
		curLadder2.start = bottomEnd;
		curLadder2.letOff = topDropOff;
		curLadder2.end = bottomStart;
		curLadder2.dropOff = bottomLetOff;
	}

	return curLaddersId;
}

void HIGHOMEGA::ENTITIES::LadderSystemClass::Combine(std::vector<LadderSystemClass> & inLadderSystems)
{
	for (LadderSystemClass & curLadder : inLadderSystems)
		for (std::pair<const unsigned long long, std::vector <Ladder>> & curDesc : curLadder.ladders)
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
	prevBodyPos = bodyPos = vec3(0.0f, 20.0f, 0.0f);
	countStuckFrames = 0u;
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
	feetStuck = false;
	quickWalk = false;
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::GenerateFeet(vec3& inBodyPos, vec3 feetTri1[3], vec3 feetTri2[3], vec3& feetTri1Min, vec3& feetTri1Max, vec3& feetTri2Min, vec3& feetTri2Max, vec3& feetFront, vec3& feetSide, vec3& feetTriMin, vec3& feetTriMax)
{
	feetSide = cross(bodyDir, bodyDir + vec3(0.1f)).normalized();
	feetFront = cross(feetSide, bodyDir).normalized();

	feetTri1[0] = inBodyPos;
	feetTri1[1] = inBodyPos - bodyDir * 3.0f + feetSide * bodyRad;
	feetTri1[2] = inBodyPos - bodyDir * 3.0f - feetSide * bodyRad;

	feetTri2[0] = inBodyPos;
	feetTri2[1] = inBodyPos - bodyDir * 3.0f + feetFront * bodyRad;
	feetTri2[2] = inBodyPos - bodyDir * 3.0f - feetFront * bodyRad;

	feetTri1Min = vec3(Min(feetTri1[0].x, feetTri1[1].x, feetTri1[2].x), Min(feetTri1[0].y, feetTri1[1].y, feetTri1[2].y), Min(feetTri1[0].z, feetTri1[1].z, feetTri1[2].z));
	feetTri1Max = vec3(Max(feetTri1[0].x, feetTri1[1].x, feetTri1[2].x), Max(feetTri1[0].y, feetTri1[1].y, feetTri1[2].y), Max(feetTri1[0].z, feetTri1[1].z, feetTri1[2].z));

	feetTri2Min = vec3(Min(feetTri2[0].x, feetTri2[1].x, feetTri2[2].x), Min(feetTri2[0].y, feetTri2[1].y, feetTri2[2].y), Min(feetTri2[0].z, feetTri2[1].z, feetTri2[2].z));
	feetTri2Max = vec3(Max(feetTri2[0].x, feetTri2[1].x, feetTri2[2].x), Max(feetTri2[0].y, feetTri2[1].y, feetTri2[2].y), Max(feetTri2[0].z, feetTri2[1].z, feetTri2[2].z));

	feetTriMin = vec3(min(feetTri1Min.x, feetTri2Min.x), min(feetTri1Min.y, feetTri2Min.y), min(feetTri1Min.z, feetTri2Min.z));
	feetTriMax = vec3(max(feetTri1Max.x, feetTri2Max.x), max(feetTri1Max.y, feetTri2Max.y), max(feetTri1Max.z, feetTri2Max.z));
}

vec3 HIGHOMEGA::ENTITIES::CharacterPhysics::ProcessLookVector(vec2 mouseDxDy)
{
	float ang_v = MainFrustum.screen_fov;
	float ang_h = MainFrustum.screen_fov;

	vec3 up_dir = MainFrustum.up;

	lateralSwingAngle *= 0.99f;
	float lateralDelta = ((float)(mouseDxDy.x) / ScreenSize.width) * ang_h * viewerMouseSpeed;
	lateralSwingAngle += lateralDelta * 0.1f;
	lookVectorXZAngle += lateralDelta;
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

void HIGHOMEGA::ENTITIES::CharacterPhysics::MovingBodyTriOverlap(std::vector<OverlappingTri>& overlappingTris, vec3 & boxMin, vec3 & boxMax)
{
	for (RigidBody* curBody : allBodies)
		BoxBodyOverlap(boxMin, boxMax, *curBody, overlappingTris);
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::FootTriOverlap(std::vector<OverlappingTri>& isecTris, vec3 * footTri1, vec3 & footTri1Norm, vec3 * footTri2, vec3 & footTri2Norm, vec3 & footTri1Min, vec3 & footTri1Max, vec3 & footTri2Min, vec3 & footTri2Max, vec3 & footTrisMin, vec3 & footTrisMax, std::vector<vec3>& footNormals, MATERIAL & inpMat)
{
	vec3 e_min, e_max, n;
	vec3 tmp1, tmp2;

	for (unsigned int i = 0; i != isecTris.size(); i++)
	{
		OverlappingTri& curTri = isecTris[i];
		n = cross(curTri.e[0] - curTri.e[1], curTri.e[2] - curTri.e[1]).normalized();
		if (curTri.n) n = -n;
		TRITRI_INTERSECT_MODE intMode;

		if (n * GravDir >= 0.0f) continue;

		e_min.x = Min(curTri.e[0].x, curTri.e[1].x, curTri.e[2].x);
		e_min.y = Min(curTri.e[0].y, curTri.e[1].y, curTri.e[2].y);
		e_min.z = Min(curTri.e[0].z, curTri.e[1].z, curTri.e[2].z);
		e_max.x = Max(curTri.e[0].x, curTri.e[1].x, curTri.e[2].x);
		e_max.y = Max(curTri.e[0].y, curTri.e[1].y, curTri.e[2].y);
		e_max.z = Max(curTri.e[0].z, curTri.e[1].z, curTri.e[2].z);

		if (footTri1Min.x > e_max.x || footTri1Min.y > e_max.y || footTri1Min.z > e_max.z ||
			footTri1Max.x < e_min.x || footTri1Max.y < e_min.y || footTri1Max.z < e_min.z)
		{
		}
		else
			if (TriTri(footTri1, footTri1Norm, curTri.e, n, &tmp1, &tmp2, &intMode))
			{
				inpMat = curTri.mat;
				footNormals.emplace_back(n);
			}

		if (footTri2Min.x > e_max.x || footTri2Min.y > e_max.y || footTri2Min.z > e_max.z ||
			footTri2Max.x < e_min.x || footTri2Max.y < e_min.y || footTri2Max.z < e_min.z)
		{
		}
		else
			if (TriTri(footTri2, footTri2Norm, curTri.e, n, &tmp1, &tmp2, &intMode))
			{
				inpMat = curTri.mat;
				footNormals.emplace_back(n);
			}
	}
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::Move(bool fwd, bool back, bool left, bool right, bool jump, bool crouch)
{
	vec2 mouseDxDy = mouseMovementConsumers["PlayerHead"];
	lookDir = ProcessLookVector(mouseDxDy).normalized();
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
	static bool prevTab = false;
	bool curTab = GetStateOfAction(CMD_QUICKWALK);
	if (!curTab && prevTab) quickWalk = !quickWalk;
	prevTab = curTab;

	if (moveVel.length() > 0.0f)
	{
		moveVel = moveVel.normalized()*(quickWalk ? walkSpeed * 5.0f : walkSpeed);
		if (crouch) moveVel *= crouchSlowDown;
	}

	if (!feetOnGround)
		playerVel += GravDir*gravStrength;
	else
	{
		playerVel = moveVel;
		if (!crouch && jump) playerVel += -GravDir*jumpStrength;
	}

	vec3 travellingBodyMin (FLT_MAX), travellingBodyMax (-FLT_MAX), curBodyPos = bodyPos;
	float curBodyStretch = bodyStretch;

	vec3 feetTri1[3], feetTri2[3], feetTri1Min, feetTri1Max, feetTri2Min, feetTri2Max, feetFront, feetSide, feetTriMin, feetTriMax;

	int maxCi = (int)(playerVel.length() + 1.0f);

	for (int ci = 0; ci != maxCi; ci++)
	{
		curBodyPos += playerVel / (float)maxCi;
		for (int i = 2; i != -1; i--)
		{
			if (crouch)
			{
				curBodyStretch += (bodyStretchSittingFraction - curBodyStretch) * 0.1f;
				if (fabs(bodyStretchSittingFraction - curBodyStretch) < 0.1f) curBodyStretch = bodyStretchSittingFraction;
			}
			else
			{
				curBodyStretch += (bodyStretchStandingFraction - curBodyStretch) * 0.1f;
				if (fabs(bodyStretchStandingFraction - curBodyStretch) < 0.1f) curBodyStretch = bodyStretchStandingFraction;
			}

			vec3 currentSphere = curBodyPos + bodyDir * ((float)(i + 1)) * bodyRad * curBodyStretch;
			vec3 sphereMax = currentSphere + vec3(bodyRad);
			vec3 sphereMin = currentSphere - vec3(bodyRad);
			travellingBodyMin.x = min(travellingBodyMin.x, sphereMin.x);
			travellingBodyMin.y = min(travellingBodyMin.y, sphereMin.y);
			travellingBodyMin.z = min(travellingBodyMin.z, sphereMin.z);
			travellingBodyMax.x = max(travellingBodyMax.x, sphereMax.x);
			travellingBodyMax.y = max(travellingBodyMax.y, sphereMax.y);
			travellingBodyMax.z = max(travellingBodyMax.z, sphereMax.z);
		}
		GenerateFeet(curBodyPos, feetTri1, feetTri2, feetTri1Min, feetTri1Max, feetTri2Min, feetTri2Max, feetFront, feetSide, feetTriMin, feetTriMax);
		travellingBodyMin.x = min(travellingBodyMin.x, feetTriMin.x);
		travellingBodyMin.y = min(travellingBodyMin.y, feetTriMin.y);
		travellingBodyMin.z = min(travellingBodyMin.z, feetTriMin.z);
		travellingBodyMax.x = max(travellingBodyMax.x, feetTriMax.x);
		travellingBodyMax.y = max(travellingBodyMax.y, feetTriMax.y);
		travellingBodyMax.z = max(travellingBodyMax.z, feetTriMax.z);
	}

	MovingBodyTriOverlap(CloseToBodyTris, travellingBodyMin, travellingBodyMax);

	vec3 tmpSide[3], tmpNorm;
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

			mouseDxDy = mouseMovementConsumers["PlayerHead"];
			lookDir = ProcessLookVector(mouseDxDy).normalized();
			mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

			vec3 currentSphere = bodyPos + bodyDir*((float)(i + 1))*bodyRad*bodyStretch;
			vec3 currentSphereMoved = currentSphere;
			vec3 sphereMax = currentSphere + vec3(bodyRad);
			vec3 sphereMin = currentSphere - vec3(bodyRad);

			for (OverlappingTri& curPushOutTri : CloseToBodyTris)
			{
				tmpNorm = cross(curPushOutTri.e[0] - curPushOutTri.e[1], curPushOutTri.e[2] - curPushOutTri.e[1]).normalized();
				if (curPushOutTri.n) tmpNorm = -tmpNorm;
				vec3 e1_e0 = curPushOutTri.e[1] - curPushOutTri.e[0];
				vec3 e2_e1 = curPushOutTri.e[2] - curPushOutTri.e[1];
				vec3 e0_e2 = curPushOutTri.e[0] - curPushOutTri.e[2];
				tmpSide[0] = cross(tmpNorm, e1_e0);
				tmpSide[1] = cross(tmpNorm, e2_e1);
				tmpSide[2] = cross(tmpNorm, e0_e2);
				if (e2_e1 * tmpSide[0] > 0.0f) tmpSide[0] = -tmpSide[0];
				if (e0_e2 * tmpSide[1] > 0.0f) tmpSide[1] = -tmpSide[1];
				if (e1_e0 * tmpSide[2] > 0.0f) tmpSide[2] = -tmpSide[2];
				PushOutSphere(curPushOutTri.e, tmpSide, tmpNorm, &currentSphereMoved, bodyRad);
			}

			sphereMax = currentSphereMoved + vec3(bodyRad);
			sphereMin = currentSphereMoved - vec3(bodyRad);
			bodyPos += (currentSphereMoved - currentSphere);
		}
	}

	mouseDxDy = mouseMovementConsumers["PlayerHead"];
	lookDir = ProcessLookVector(mouseDxDy).normalized();
	mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

	bool findOutFeetOnGround = false;

	GenerateFeet(bodyPos, feetTri1, feetTri2, feetTri1Min, feetTri1Max, feetTri2Min, feetTri2Max, feetFront, feetSide, feetTriMin, feetTriMax);

	vec3 avgFootNormal = vec3(0.0f);
	int countFootNormals = 0;
	footNormals.clear();

	FootTriOverlap (CloseToBodyTris, feetTri1, feetFront, feetTri2, feetSide, feetTri1Min, feetTri1Max, feetTri2Min, feetTri2Max, feetTriMin, feetTriMax, footNormals, groundMat);
	for (int i = 0; i != footNormals.size(); i++)
		avgFootNormal += footNormals[i];
	countFootNormals += (unsigned int)footNormals.size();
	if (countFootNormals > 0)
		avgFootNormal = (avgFootNormal / (float)countFootNormals).normalized();
	if (avgFootNormal != vec3(0.0f) && avgFootNormal * GravDir < -0.7f)
		findOutFeetOnGround = true;

	bool prevFeetOnGround = feetOnGround;
	feetOnGround = findOutFeetOnGround;
	if (!feetOnGround)
	{
		if ((prevBodyPos - bodyPos).length() < 3.0f) countStuckFrames++;
		else countStuckFrames = 0u;
		if (countStuckFrames >= 10 && fabs(playerVel * GravDir) > 10.0f) // Feet stuck
		{
			feetOnGround = true;
			feetStuck = true;
		}
		else
			feetStuck = false;
	}
	else
		countStuckFrames = 0u;
	prevBodyPos = bodyPos;

	CloseToBodyTris.clear();
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::UpdateClothObstacles(unsigned long long playerId)
{
	vec3 sph1 = bodyPos + bodyDir * 1.0f * bodyRad * bodyStretch;
	vec3 sph2 = bodyPos + bodyDir * 2.0f * bodyRad * bodyStretch;
	vec3 sph3 = bodyPos + bodyDir * 3.0f * bodyRad * bodyStretch;

	if (!playerClothObstacle)
	{
		allObstaclesMutex.lock();
		allObstacles[playerId] = { {bodyPos + bodyDir * 1.0f * bodyRad * bodyStretch,
									bodyPos + bodyDir * 2.0f * bodyRad * bodyStretch,
									bodyPos + bodyDir * 3.0f * bodyRad * bodyStretch}, bodyRad };
		playerClothObstacle = &allObstacles[playerId];
		allObstaclesMutex.unlock();
	}
	else
	{
		allObstaclesMutex.lock_shared();
		playerClothObstacle->pos[0] = sph1;
		playerClothObstacle->pos[1] = sph2;
		playerClothObstacle->pos[2] = sph3;
		allObstaclesMutex.unlock_shared();
	}
}

vec3 HIGHOMEGA::ENTITIES::CharacterPhysics::GetEye()
{
	return bodyPos + bodyDir*3.0f*bodyRad*bodyStretch;
}

float HIGHOMEGA::ENTITIES::CharacterPhysics::GetStandingHeight()
{
	return 3.0f * bodyRad + 3.0f;
}

float HIGHOMEGA::ENTITIES::CharacterPhysics::GetLateralSwingAngle()
{
	return lateralSwingAngle;
}

void HIGHOMEGA::ENTITIES::CharacterAudio::Process(CharacterPhysics& playerPhysicsRef)
{
	for (std::pair<const HIGHOMEGA::FIZ_X::MATERIAL, walkingSound>& curMatWalkSound : walkingSounds)
		curMatWalkSound.second.curPlaying = false;
	bool curPlayLadder = false;
	if (playerPhysicsRef.feetOnGround)
	{
		if (playerPhysicsRef.playerVel != vec3(0.0))
		{
			if (walkingSounds.size() == 0)
			{
				walkingSounds[DIRT] = { false, false, 0ul, 1.0f, "assets/audio/sfx/walk/sand.wav" };
				walkingSounds[RUBBLE] = { false, false, 0ul, 1.0f, "assets/audio/sfx/walk/rubble.wav" };
				walkingSounds[METALPLATFORM] = { false, false, 0ul, 0.3333f, "assets/audio/sfx/walk/metalplatform.wav" };
				walkingSounds[METALRODS] = { false, false, 0ul, 0.3333f, "assets/audio/sfx/walk/metalrods.wav" };
				walkingSounds[CONCRETE] = { false, false, 0ul, 0.3333f, "assets/audio/sfx/walk/concrete.wav" };
				walkingSounds[GLASS] = walkingSounds[CERAMIC] = { false, false, 0ul, 0.3f, "assets/audio/sfx/walk/ceramic.wav" };
				walkingSounds[WOOD] = { false, false, 0ul, 1.0f, "assets/audio/sfx/walk/wood.wav" };
				for (std::pair<const HIGHOMEGA::FIZ_X::MATERIAL, walkingSound> & curMatWalkSound : walkingSounds)
				{
					curMatWalkSound.second.walkSound = AudioSystem.Insert(curMatWalkSound.second.path, AudioSystemClass::SOUND_TYPE::SFX, vec3(0.0f), vec3(0.0f), curMatWalkSound.second.initialVolume, true);
					AudioSystem.Pause(curMatWalkSound.second.walkSound);
				}
			}
			walkingSounds[playerPhysicsRef.groundMat].curPlaying = true;
		}
	}
	else
	{
		if (playerPhysicsRef.ladderInfo.climbingLadder &&
			playerPhysicsRef.ladderInfo.transitionState == HIGHOMEGA::ENTITIES::CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING &&
			(GetStateOfAction(CMD_MOVE_BACKWARD) || GetStateOfAction(CMD_MOVE_FORWARD)))
		{
			if (!climbSoundLadder)
				climbSoundLadder = AudioSystem.Insert("assets/audio/sfx/climb/ladder.wav", AudioSystemClass::SOUND_TYPE::SFX, vec3(0.0f), vec3(0.0f), 0.3333f, true);
			curPlayLadder = true;
		}
		else
			curPlayLadder = false;
	}
	for (std::pair<const HIGHOMEGA::FIZ_X::MATERIAL, walkingSound>& curMatWalkSound : walkingSounds)
		if (curMatWalkSound.second.curPlaying != curMatWalkSound.second.prevPlaying)
		{
			if (curMatWalkSound.second.curPlaying)
			{
				AudioSystem.SetPosition(curMatWalkSound.second.walkSound, playerPhysicsRef.GetEye());
				AudioSystem.Resume(curMatWalkSound.second.walkSound);
			}
			else
				AudioSystem.Pause(curMatWalkSound.second.walkSound);
			curMatWalkSound.second.prevPlaying = curMatWalkSound.second.curPlaying;
		}
	if (playingLadderSound != curPlayLadder)
	{
		if (curPlayLadder)
		{
			AudioSystem.SetPosition(climbSoundLadder, playerPhysicsRef.GetEye());
			AudioSystem.Resume(climbSoundLadder);
		}
		else
			AudioSystem.Pause(climbSoundLadder);
		playingLadderSound = curPlayLadder;
	}
}

void HIGHOMEGA::ENTITIES::CharacterProps::Process(bool isOnLadder, CharacterPhysics& playerPhysicsRef, CharacterAudio& playerAudioRef)
{
	AudioSystem.SetListener(playerPhysicsRef.GetEye(), playerPhysicsRef.playerVel, playerPhysicsRef.lookDir, playerPhysicsRef.bodyDir);
}

void HIGHOMEGA::ENTITIES::CharacterPhysics::ForcePosFromEye(vec3 inEye)
{
	bodyPos = inEye - bodyDir * 3.0f*bodyRad*bodyStretch;
}

void HIGHOMEGA::ENTITIES::Character::Process(WorldParamsClass& WorldParams)
{
	if (!physics.ladderInfo.climbingLadder)
	{
		if (WorldParams.FirstPersonControls())
		{
			physics.UpdateClothObstacles(id);
			physics.Move(GetStateOfAction(CMD_MOVE_FORWARD),
				GetStateOfAction(CMD_MOVE_BACKWARD),
				GetStateOfAction(CMD_MOVE_LEFT),
				GetStateOfAction(CMD_MOVE_RIGHT),
				GetStateOfAction(CMD_JUMP),
				GetStateOfAction(CMD_CROUCH));

			audio.Process(physics);
			props.Process(physics.ladderInfo.climbingLadder, physics, audio);

			static bool prevFire = false;
			bool curFire = GetStateOfAction(CMD_FIRE);
			if (!curFire && prevFire)
			{
				std::lock_guard<std::mutex> lk(fireLineMutex);
				fireLines.push_back({ physics.GetEye(), physics.lookDir });
			}
			prevFire = curFire;
		}

		mouseMovementConsumers["PlayerHead"] = vec2(0.0f);

		bool insideLadderEntry = false;
		if (physics.feetOnGround)
		{
			bool breakOut = false;
			for (std::pair <const unsigned long long, std::vector<HIGHOMEGA::ENTITIES::LadderSystemClass::Ladder>>& curLadderList : mainLadderSystem.ladders)
			{
				for (HIGHOMEGA::ENTITIES::LadderSystemClass::Ladder& curLadder : curLadderList.second)
					if (physics.GetEye().x > curLadder.entryMin.x &&
						physics.GetEye().y > curLadder.entryMin.y &&
						physics.GetEye().z > curLadder.entryMin.z &&
						physics.GetEye().x < curLadder.entryMax.x &&
						physics.GetEye().y < curLadder.entryMax.y &&
						physics.GetEye().z < curLadder.entryMax.z)
					{
						if (GetStateOfAction(CMD_USE))
						{
							physics.bodyStretch = 1.0f;
							physics.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_ON;
							physics.ladderInfo.climbingLadder = &curLadder;
							physics.ladderInfo.transitionFraction = 0.0f;
							physics.ladderInfo.curEye = physics.GetEye();
							physics.ladderInfo.curLook = physics.lookDir;
						}
						else
							insideLadderEntry = true;
						breakOut = true;
						break;
					}
				if (breakOut) break;
			}
		}
		physics.ladderInfo.inEntryZone = insideLadderEntry;

		MainFrustum.entitiesLook = physics.lookDir;
		MainFrustum.entitiesEye = physics.GetEye();
	}
	else
	{
		physics.feetOnGround = false;
		physics.playerVel = vec3(0.0f);
		audio.Process(physics);
		props.Process(physics.ladderInfo.climbingLadder, physics, audio);

		switch (physics.ladderInfo.transitionState)
		{
		case CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_ON:
		{
			physics.ladderInfo.transitionFraction += (1.0f - physics.ladderInfo.transitionFraction) * 0.1f;
			physics.ladderInfo.curEye += (physics.ladderInfo.climbingLadder->start - physics.ladderInfo.curEye) * 0.1f;
			physics.ladderInfo.curLook += (physics.ladderInfo.climbingLadder->stareDir - physics.ladderInfo.curLook) * 0.1f;
			physics.ladderInfo.curLook = physics.ladderInfo.curLook.normalized();

			if (physics.ladderInfo.transitionFraction > 0.9f)
			{
				physics.ladderInfo.curEye = physics.ladderInfo.climbingLadder->start;
				physics.ladderInfo.curLook = physics.ladderInfo.climbingLadder->stareDir;
				physics.ladderInfo.transitionFraction = 0.0f;
				physics.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING;
			}
		}
		break;
		case CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING:
		{
			vec3 ladderDiff = physics.ladderInfo.climbingLadder->end - physics.ladderInfo.climbingLadder->start;
			float ladderLen = ladderDiff.length();
			vec3 ladderMoveDir = ladderDiff / ladderLen;
			if (GetStateOfAction(physics.bodyDir * ladderDiff > 0.0f ? CMD_MOVE_FORWARD : CMD_MOVE_BACKWARD))
			{
				physics.ladderInfo.curEye += ladderMoveDir * 0.1f;
			}
			else if (GetStateOfAction(physics.bodyDir * ladderDiff > 0.0f ? CMD_MOVE_BACKWARD : CMD_MOVE_FORWARD))
			{
				physics.ladderInfo.curEye -= ladderMoveDir * 0.1f;
			}
			vec3 eyeToLadderStartDiff = physics.ladderInfo.curEye - physics.ladderInfo.climbingLadder->start;
			float ladderProgress = eyeToLadderStartDiff.length() / ladderLen;
			if (eyeToLadderStartDiff * ladderDiff < 0.0f)
			{
				physics.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING_OFF_BOTTOM;
				physics.ladderInfo.curEye = physics.ladderInfo.climbingLadder->start;
			}
			else if (ladderProgress > 1.0f)
			{
				physics.ladderInfo.transitionState = CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_OFF;
				physics.ladderInfo.curEye = physics.ladderInfo.climbingLadder->end;
			}
			else if (GetStateOfAction(CMD_USE))
			{
				physics.lookDir = physics.ladderInfo.curLook;
				physics.ForceLookVector(physics.ladderInfo.curLook);
				physics.ForcePosFromEye(physics.ladderInfo.curEye);
				physics.playerVel = vec3(0.0f);

				physics.ladderInfo.climbingLadder = nullptr;
				mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
			}
		}
		break;
		case CharacterPhysics::LADDER_TRANSITION_STATE::CLIMBING_OFF_BOTTOM:
		{
			physics.ladderInfo.transitionFraction += (1.0f - physics.ladderInfo.transitionFraction) * 0.1f;
			physics.ladderInfo.curEye += (physics.ladderInfo.climbingLadder->letOff - physics.ladderInfo.curEye) * 0.1f;
			if (physics.ladderInfo.transitionFraction > 0.9f)
			{
				physics.ladderInfo.curEye = physics.ladderInfo.climbingLadder->letOff;
				physics.ladderInfo.transitionFraction = 0.0f;

				physics.lookDir = physics.ladderInfo.climbingLadder->stareDir;
				physics.ForceLookVector(physics.lookDir);
				physics.ForcePosFromEye(physics.ladderInfo.curEye);
				physics.playerVel = vec3(0.0f);

				physics.ladderInfo.climbingLadder = nullptr;
				mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
			}
		}
		break;
		case CharacterPhysics::LADDER_TRANSITION_STATE::GETTING_OFF:
		{
			physics.ladderInfo.transitionFraction += (1.0f - physics.ladderInfo.transitionFraction) * 0.1f;
			physics.ladderInfo.curEye += (physics.ladderInfo.climbingLadder->dropOff - physics.ladderInfo.curEye) * 0.1f;
			if (physics.ladderInfo.transitionFraction > 0.9f)
			{
				physics.ladderInfo.curEye = physics.ladderInfo.climbingLadder->dropOff;
				physics.ladderInfo.transitionFraction = 0.0f;

				physics.lookDir = physics.ladderInfo.climbingLadder->stareDir;
				physics.ForceLookVector(physics.lookDir);
				physics.ForcePosFromEye(physics.ladderInfo.curEye);
				physics.playerVel = vec3(0.0f);

				physics.ladderInfo.climbingLadder = nullptr;
				mouseMovementConsumers["PlayerHead"] = vec2(0.0f);
			}
		}
		break;
		}

		MainFrustum.entitiesLook = physics.ladderInfo.curLook;
		MainFrustum.entitiesEye = physics.ladderInfo.curEye;
	}
}

HIGHOMEGA::ENTITIES::Character::Character()
{
	id = threadSafeMersenneTwister64Bit();
}
