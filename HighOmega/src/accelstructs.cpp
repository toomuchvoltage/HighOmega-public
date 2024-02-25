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

#include "accelstructs.h"

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::morton3D(float x, float y, float z)
{
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = expandBits((unsigned int)x);
	unsigned int yy = expandBits((unsigned int)y);
	unsigned int zz = expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::countLeadingZeroes(unsigned int inpCode)
{
	int n = 32;
	unsigned int y;
	unsigned int x = inpCode;

	y = x >> 16; if (y != 0) { n = n - 16; x = y; }
	y = x >> 8; if (y != 0) { n = n - 8; x = y; }
	y = x >> 4; if (y != 0) { n = n - 4; x = y; }
	y = x >> 2; if (y != 0) { n = n - 2; x = y; }
	y = x >> 1; if (y != 0) return n - 2;
	return n - int(x);
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::CalcMortons(BVHTriangle * triangleArr, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap)
{
	vec3 invMapSize = vec3(1.0f / (maxMap.x - minMap.x), 1.0f / (maxMap.y - minMap.y), 1.0f / (maxMap.z - minMap.z));
	for (int i = start; i != end; i++) {
		BVHTriangle & curTri = triangleArr[i];
		vec3 triMin, triMax;
		triMin.x = min(min(curTri.e1Instance[0], curTri.e2Primitive[0]), curTri.e3Morton[0]);
		triMin.y = min(min(curTri.e1Instance[1], curTri.e2Primitive[1]), curTri.e3Morton[1]);
		triMin.z = min(min(curTri.e1Instance[2], curTri.e2Primitive[2]), curTri.e3Morton[2]);
		triMax.x = max(max(curTri.e1Instance[0], curTri.e2Primitive[0]), curTri.e3Morton[0]);
		triMax.y = max(max(curTri.e1Instance[1], curTri.e2Primitive[1]), curTri.e3Morton[1]);
		triMax.z = max(max(curTri.e1Instance[2], curTri.e2Primitive[2]), curTri.e3Morton[2]);
		vec3 cent = (triMin + triMax) * 0.5f;
		cent.x = (cent.x - minMap.x) * invMapSize.x;
		cent.y = (cent.y - minMap.y) * invMapSize.y;
		cent.z = (cent.z - minMap.z) * invMapSize.z;

		unsigned int newMorton = morton3D(cent.x, cent.y, cent.z);
		memcpy(&triangleArr[i].e3Morton[3], &newMorton, sizeof(unsigned int));
	}
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::QuickSortPartition(BVHTriangle * A, unsigned int lo, unsigned int hi)
{
	unsigned int pivotCode;
	memcpy(&pivotCode, &A[(lo + hi) / 2].e3Morton[3], sizeof(unsigned int));
	unsigned int i = lo - 1;
	unsigned int j = hi + 1;
	for (;;)
	{
		do
		{
			i++;
			unsigned int getMorton;
			memcpy(&getMorton, &A[i].e3Morton[3], sizeof(unsigned int));
			if (getMorton >= pivotCode) break;
		} while (true);

		do
		{
			j--;
			unsigned int getMorton;
			memcpy(&getMorton, &A[j].e3Morton[3], sizeof(unsigned int));
			if (getMorton <= pivotCode) break;
		} while (true);

		if (i >= j) return j;

		BVHTriangle Ai;
		memcpy(&Ai, &A[i], sizeof(BVHTriangle));
		memcpy(&A[i], &A[j], sizeof(BVHTriangle));
		memcpy(&A[j], &Ai, sizeof(BVHTriangle));
	}
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::QuickSort(BVHTriangle * A, unsigned int lo, unsigned int hi)
{
	if (lo < hi) {
		unsigned int p = QuickSortPartition(A, lo, hi);
		QuickSort(A, lo, p);
		QuickSort(A, p + 1, hi);
	}
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::findSplit(int first, int last)
{
	unsigned int firstCode, lastCode;
	memcpy(&firstCode, &triangleArr[first].e3Morton[3], sizeof(unsigned int));
	memcpy(&lastCode, &triangleArr[last].e3Morton[3], sizeof(unsigned int));

	if (firstCode == lastCode) return (first + last) >> 1;

	int commonPrefix = countLeadingZeroes(firstCode ^ lastCode);

	int split = first;
	int step = last - first;

	do
	{
		step = (step + 1) >> 1;
		int newSplit = split + step;

		if (newSplit < last)
		{
			unsigned int splitCode;
			memcpy(&splitCode, &triangleArr[newSplit].e3Morton[3], sizeof(unsigned int));
			int splitPrefix = countLeadingZeroes(firstCode ^ splitCode);
			if (splitPrefix > commonPrefix)
				split = newSplit;
		}
	} while (step > 1);

	return split;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::genHierarchyCompressTris(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax)
{
	if (lo == hi)
	{
		BVHNode *retVal = &nodes[nodeCounter];
		nodeCounter++;
		memcpy(&retVal->aabbMinChildA[3], &lo, sizeof(int));
		memcpy(&retVal->aabbMaxChildB[3], &lo, sizeof(int));

		BVHTriangle & leafTri = triangleArr[lo];

		vec3 e1 = vec3(leafTri.e1Instance[0], leafTri.e1Instance[1], leafTri.e1Instance[2]);
		vec3 e2 = vec3(leafTri.e2Primitive[0], leafTri.e2Primitive[1], leafTri.e2Primitive[2]);
		vec3 e3 = vec3(leafTri.e3Morton[0], leafTri.e3Morton[1], leafTri.e3Morton[2]);

		retVal->aabbMinChildA[0] = min(min(e1.x, e2.x), e3.x);
		retVal->aabbMinChildA[1] = min(min(e1.y, e2.y), e3.y);
		retVal->aabbMinChildA[2] = min(min(e1.z, e2.z), e3.z);

		retVal->aabbMaxChildB[0] = max(max(e1.x, e2.x), e3.x);
		retVal->aabbMaxChildB[1] = max(max(e1.y, e2.y), e3.y);
		retVal->aabbMaxChildB[2] = max(max(e1.z, e2.z), e3.z);

		retMin = vec3(retVal->aabbMinChildA[0], retVal->aabbMinChildA[1], retVal->aabbMinChildA[2]);
		retMax = vec3(retVal->aabbMaxChildB[0], retVal->aabbMaxChildB[1], retVal->aabbMaxChildB[2]);

		vec3 aabbLenInv8bit = retMax - retMin;
		if (aabbLenInv8bit.x != 0.0) aabbLenInv8bit.x = 255.0f / aabbLenInv8bit.x;
		if (aabbLenInv8bit.y != 0.0) aabbLenInv8bit.y = 255.0f / aabbLenInv8bit.y;
		if (aabbLenInv8bit.z != 0.0) aabbLenInv8bit.z = 255.0f / aabbLenInv8bit.z;

		vec3 frac1 = e1 - retMin;
		vec3 frac2 = e2 - retMin;
		vec3 frac3 = e3 - retMin;

		frac1.pointwiseMul(aabbLenInv8bit);
		frac2.pointwiseMul(aabbLenInv8bit);
		frac3.pointwiseMul(aabbLenInv8bit);

		unsigned int tmp;
		unsigned int f1x = (unsigned char)frac1.x;
		unsigned int f2x = (unsigned char)frac2.x;
		unsigned int f3x = (unsigned char)frac3.x;
		unsigned int f1y = (unsigned char)frac1.y;
		unsigned int f2y = (unsigned char)frac2.y;
		unsigned int f3y = (unsigned char)frac3.y;
		unsigned int f1z = (unsigned char)frac1.z;
		unsigned int f2z = (unsigned char)frac2.z;
		unsigned int f3z = (unsigned char)frac3.z;
		ShiftLeft(f1x, f2x, f3x, tmp, 24);
		ShiftLeft(f1y, f2y, f3y, tmp, 16);
		ShiftLeft(f1z, f2z, f3z, tmp, 8);

		unsigned int instanceId = *((unsigned int *)(&leafTri.e1Instance[3]));

		unsigned int e1Comp = f1x | f1y | f1z | ((instanceId & 0x00FF0000u) >> 16);
		unsigned int e2Comp = f2x | f2y | f2z | ((instanceId & 0x0000FF00u) >> 8);
		unsigned int e3Comp = f3x | f3y | f3z |  (instanceId & 0x000000FFu);

		triangleArrComp[lo].e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID[0] = e1Comp;
		triangleArrComp[lo].e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID[1] = e2Comp;
		triangleArrComp[lo].e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID[2] = e3Comp;
		triangleArrComp[lo].e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID[3] = (*((unsigned int *)(&leafTri.e2Primitive[3])) & 0x7FFFFFFFu);
		unsigned int attribs0 = *((unsigned int *)(&((*instPropsRef)[instanceId].attribs1[0])));
		if ((attribs0 & 0x00000080u) != 0u)
			triangleArrComp[lo].e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID[3] |= 0x80000000;

		return nodeCounter - 1;
	}

	BVHNode *retVal = &nodes[nodeCounter];
	nodeCounter++;
	unsigned int retNodePosition = nodeCounter - 1;
	int split = findSplit(lo, hi);

	vec3 leftMin, leftMax, rightMin, rightMax;
	unsigned int childA = genHierarchyCompressTris(nodeCounter, lo, split, leftMin, leftMax);
	unsigned int childB = genHierarchyCompressTris(nodeCounter, split + 1, hi, rightMin, rightMax);
	memcpy(&retVal->aabbMinChildA[3], &childA, sizeof(unsigned int));
	memcpy(&retVal->aabbMaxChildB[3], &childB, sizeof(unsigned int));
	retMin = vec3(min(leftMin.x, rightMin.x), min(leftMin.y, rightMin.y), min(leftMin.z, rightMin.z));
	retMax = vec3(max(leftMax.x, rightMax.x), max(leftMax.y, rightMax.y), max(leftMax.z, rightMax.z));
	retVal->aabbMinChildA[0] = retMin.x;
	retVal->aabbMinChildA[1] = retMin.y;
	retVal->aabbMinChildA[2] = retMin.z;
	retVal->aabbMaxChildB[0] = retMax.x;
	retVal->aabbMaxChildB[1] = retMax.y;
	retVal->aabbMaxChildB[2] = retMax.z;
	return retNodePosition;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::genHierarchy(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax)
{
	if (lo == hi)
	{
		BVHNode *retVal = &nodesOutsidePtr[nodeCounter];
		nodeCounter++;
		retVal->aabbMinChildA[3] = *((float *)&lo);
		retVal->aabbMaxChildB[3] = *((float *)&lo);

		BVHTriangle & leafTri = triangleArr[lo];

		vec3 e1 = vec3(leafTri.e1Instance[0], leafTri.e1Instance[1], leafTri.e1Instance[2]);
		vec3 e2 = vec3(leafTri.e2Primitive[0], leafTri.e2Primitive[1], leafTri.e2Primitive[2]);
		vec3 e3 = vec3(leafTri.e3Morton[0], leafTri.e3Morton[1], leafTri.e3Morton[2]);

		retVal->aabbMinChildA[0] = min(min(e1.x, e2.x), e3.x);
		retVal->aabbMinChildA[1] = min(min(e1.y, e2.y), e3.y);
		retVal->aabbMinChildA[2] = min(min(e1.z, e2.z), e3.z);

		retVal->aabbMaxChildB[0] = max(max(e1.x, e2.x), e3.x);
		retVal->aabbMaxChildB[1] = max(max(e1.y, e2.y), e3.y);
		retVal->aabbMaxChildB[2] = max(max(e1.z, e2.z), e3.z);

		retMin = vec3(retVal->aabbMinChildA[0], retVal->aabbMinChildA[1], retVal->aabbMinChildA[2]);
		retMax = vec3(retVal->aabbMaxChildB[0], retVal->aabbMaxChildB[1], retVal->aabbMaxChildB[2]);

		return nodeCounter - 1;
	}

	BVHNode *retVal = &nodesOutsidePtr[nodeCounter];
	nodeCounter++;
	unsigned int retNodePosition = nodeCounter - 1;
	int split = findSplit(lo, hi);

	vec3 leftMin, leftMax, rightMin, rightMax;
	unsigned int childA = genHierarchy(nodeCounter, lo, split, leftMin, leftMax);
	unsigned int childB = genHierarchy(nodeCounter, split + 1, hi, rightMin, rightMax);
	memcpy(&retVal->aabbMinChildA[3], &childA, sizeof(unsigned int));
	memcpy(&retVal->aabbMaxChildB[3], &childB, sizeof(unsigned int));
	retMin = vec3(min(leftMin.x, rightMin.x), min(leftMin.y, rightMin.y), min(leftMin.z, rightMin.z));
	retMax = vec3(max(leftMax.x, rightMax.x), max(leftMax.y, rightMax.y), max(leftMax.z, rightMax.z));
	retVal->aabbMinChildA[0] = retMin.x;
	retVal->aabbMinChildA[1] = retMin.y;
	retVal->aabbMinChildA[2] = retMin.z;
	retVal->aabbMaxChildB[0] = retMax.x;
	retVal->aabbMaxChildB[1] = retMax.y;
	retVal->aabbMaxChildB[2] = retMax.z;
	return retNodePosition;
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::ProduceBVH(BVHTriangle * inpTris, int inpNTris, vec3 mapMin, vec3 mapMax, std::vector<InstanceProperties> & inpInstPropsRef)
{
	nTris = inpNTris;
	triangleArr = inpTris;
	unsigned int nodeSizeAligned = nTris * 2 - 1;
	unsigned int triangleSizeAligned = nTris;
	nodeSizeAligned = (unsigned int)(ceil((double)nodeSizeAligned / 2000.0) * 2000);
	triangleSizeAligned = (unsigned int)(ceil((double)triangleSizeAligned / 1000.0) * 1000);
	nodes.resize(nodeSizeAligned);
	triangleArrComp.resize(triangleSizeAligned);
	instPropsRef = &inpInstPropsRef;
	this->mapMin = mapMin;
	this->mapMax = mapMax;

	std::thread mortonThread1(CalcMortons, triangleArr, 0, nTris / 2, mapMin, mapMax);
	std::thread mortonThread2(CalcMortons, triangleArr, nTris / 2, nTris, mapMin, mapMax);
	mortonThread1.join();
	mortonThread2.join();
	
	QuickSort(triangleArr, 0, nTris - 1);

	unsigned int nodeCounter = 0;

	vec3 retMapMin, retMapMax;
	genHierarchyCompressTris(nodeCounter, 0, nTris - 1, retMapMin, retMapMax);
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::ProduceNodesOnly(BVHTriangle * inpTris, BVHNode *inpNodesOutsidePtr, int inpNTris)
{
	triangleArr = inpTris;
	nodesOutsidePtr = inpNodesOutsidePtr;

	unsigned int nodeCounter = 0;

	vec3 retMapMin, retMapMax;
	genHierarchy(nodeCounter, 0, inpNTris - 1, retMapMin, retMapMax);
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::getNumTris()
{
	return nTris;
}

void * HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::getNodes()
{
	return (void *)nodes.data();
}

void * HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::getTrianglesCompressed()
{
	return (void *)triangleArrComp.data();
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::getNodesSizeAligned()
{
	return (unsigned int)nodes.size();
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenCPUClass::getTrianglesCompressedSizeAligned()
{
	return (unsigned int)triangleArrComp.size();
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::CalcMortons(SDFLeaf * sdfLeaves, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap)
{
	vec3 invMapSize = vec3(1.0f / (maxMap.x - minMap.x), 1.0f / (maxMap.y - minMap.y), 1.0f / (maxMap.z - minMap.z));
	for (int i = start; i != end; i++) {
		SDFLeaf & curGrid = sdfLeaves[i];
		vec3 triMin, triMax;
		triMin.x = curGrid.leafMinMorton[0];
		triMin.y = curGrid.leafMinMorton[1];
		triMin.z = curGrid.leafMinMorton[2];
		triMax.x = curGrid.leafMaxLeafId[0];
		triMax.y = curGrid.leafMaxLeafId[1];
		triMax.z = curGrid.leafMaxLeafId[2];
		vec3 cent = (triMin + triMax) * 0.5f;
		cent.x = (cent.x - minMap.x) * invMapSize.x;
		cent.y = (cent.y - minMap.y) * invMapSize.y;
		cent.z = (cent.z - minMap.z) * invMapSize.z;

		unsigned int newMorton = morton3D(cent.x, cent.y, cent.z);
		memcpy(&curGrid.leafMinMorton[3], &newMorton, sizeof(unsigned int));
	}
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::QuickSortPartition(SDFLeaf * A, unsigned int lo, unsigned int hi)
{
	unsigned int pivotCode;
	memcpy(&pivotCode, &A[(lo + hi) / 2].leafMinMorton[3], sizeof(unsigned int));
	unsigned int i = lo - 1;
	unsigned int j = hi + 1;
	for (;;)
	{
		do
		{
			i++;
			unsigned int getMorton;
			memcpy(&getMorton, &A[i].leafMinMorton[3], sizeof(unsigned int));
			if (getMorton >= pivotCode) break;
		} while (true);

		do
		{
			j--;
			unsigned int getMorton;
			memcpy(&getMorton, &A[j].leafMinMorton[3], sizeof(unsigned int));
			if (getMorton <= pivotCode) break;
		} while (true);

		if (i >= j) return j;

		SDFLeaf Ai;
		memcpy(&Ai, &A[i], sizeof(SDFLeaf));
		memcpy(&A[i], &A[j], sizeof(SDFLeaf));
		memcpy(&A[j], &Ai, sizeof(SDFLeaf));
	}
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::QuickSort(SDFLeaf * A, unsigned int lo, unsigned int hi)
{
	if (lo < hi) {
		unsigned int p = QuickSortPartition(A, lo, hi);
		QuickSort(A, lo, p);
		QuickSort(A, p + 1, hi);
	}
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::findSplit(int first, int last)
{
	unsigned int firstCode, lastCode;
	memcpy(&firstCode, &gridArr[first].leafMinMorton[3], sizeof(unsigned int));
	memcpy(&lastCode, &gridArr[last].leafMinMorton[3], sizeof(unsigned int));

	if (firstCode == lastCode) return (first + last) >> 1;

	int commonPrefix = countLeadingZeroes(firstCode ^ lastCode);

	int split = first;
	int step = last - first;

	do
	{
		step = (step + 1) >> 1;
		int newSplit = split + step;

		if (newSplit < last)
		{
			unsigned int splitCode;
			memcpy(&splitCode, &gridArr[newSplit].leafMinMorton[3], sizeof(unsigned int));
			int splitPrefix = countLeadingZeroes(firstCode ^ splitCode);
			if (splitPrefix > commonPrefix)
				split = newSplit;
		}
	} while (step > 1);

	return split;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::genHierarchy(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax)
{
	if (lo == hi)
	{
		BVHNode *retVal = &nodes[nodeCounter];
		nodeCounter++;
		memcpy(&retVal->aabbMinChildA[3], &gridArr[lo].leafMaxLeafId[3], sizeof(int));
		memcpy(&retVal->aabbMaxChildB[3], &gridArr[lo].leafMaxLeafId[3], sizeof(int));

		retVal->aabbMinChildA[0] = gridArr[lo].leafMinMorton[0];
		retVal->aabbMinChildA[1] = gridArr[lo].leafMinMorton[1];
		retVal->aabbMinChildA[2] = gridArr[lo].leafMinMorton[2];
		retVal->aabbMaxChildB[0] = gridArr[lo].leafMaxLeafId[0];
		retVal->aabbMaxChildB[1] = gridArr[lo].leafMaxLeafId[1];
		retVal->aabbMaxChildB[2] = gridArr[lo].leafMaxLeafId[2];

		retMin = vec3(retVal->aabbMinChildA[0], retVal->aabbMinChildA[1], retVal->aabbMinChildA[2]);
		retMax = vec3(retVal->aabbMaxChildB[0], retVal->aabbMaxChildB[1], retVal->aabbMaxChildB[2]);

		return nodeCounter - 1;
	}

	BVHNode *retVal = &nodes[nodeCounter];
	nodeCounter++;
	unsigned int retNodePosition = nodeCounter - 1;
	int split = findSplit(lo, hi);

	vec3 leftMin, leftMax, rightMin, rightMax;
	unsigned int childA = genHierarchy(nodeCounter, lo, split, leftMin, leftMax);
	unsigned int childB = genHierarchy(nodeCounter, split + 1, hi, rightMin, rightMax);
	memcpy(&retVal->aabbMinChildA[3], &childA, sizeof(unsigned int));
	memcpy(&retVal->aabbMaxChildB[3], &childB, sizeof(unsigned int));
	retMin = vec3(min(leftMin.x, rightMin.x), min(leftMin.y, rightMin.y), min(leftMin.z, rightMin.z));
	retMax = vec3(max(leftMax.x, rightMax.x), max(leftMax.y, rightMax.y), max(leftMax.z, rightMax.z));
	retVal->aabbMinChildA[0] = retMin.x;
	retVal->aabbMinChildA[1] = retMin.y;
	retVal->aabbMinChildA[2] = retMin.z;
	retVal->aabbMaxChildB[0] = retMax.x;
	retVal->aabbMaxChildB[1] = retMax.y;
	retVal->aabbMaxChildB[2] = retMax.z;
	return retNodePosition;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::genHierarchyExternal(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax)
{
	if (lo == hi)
	{
		BVHNode *retVal = &nodesOutsidePtr[nodeCounter];
		nodeCounter++;
		memcpy(&retVal->aabbMinChildA[3], &gridArr[lo].leafMaxLeafId[3], sizeof(int));
		memcpy(&retVal->aabbMaxChildB[3], &gridArr[lo].leafMaxLeafId[3], sizeof(int));

		retVal->aabbMinChildA[0] = gridArr[lo].leafMinMorton[0];
		retVal->aabbMinChildA[1] = gridArr[lo].leafMinMorton[1];
		retVal->aabbMinChildA[2] = gridArr[lo].leafMinMorton[2];
		retVal->aabbMaxChildB[0] = gridArr[lo].leafMaxLeafId[0];
		retVal->aabbMaxChildB[1] = gridArr[lo].leafMaxLeafId[1];
		retVal->aabbMaxChildB[2] = gridArr[lo].leafMaxLeafId[2];

		retMin = vec3(retVal->aabbMinChildA[0], retVal->aabbMinChildA[1], retVal->aabbMinChildA[2]);
		retMax = vec3(retVal->aabbMaxChildB[0], retVal->aabbMaxChildB[1], retVal->aabbMaxChildB[2]);

		return nodeCounter - 1;
	}

	BVHNode *retVal = &nodesOutsidePtr[nodeCounter];
	nodeCounter++;
	unsigned int retNodePosition = nodeCounter - 1;
	int split = findSplit(lo, hi);

	vec3 leftMin, leftMax, rightMin, rightMax;
	unsigned int childA = genHierarchyExternal(nodeCounter, lo, split, leftMin, leftMax);
	unsigned int childB = genHierarchyExternal(nodeCounter, split + 1, hi, rightMin, rightMax);
	memcpy(&retVal->aabbMinChildA[3], &childA, sizeof(unsigned int));
	memcpy(&retVal->aabbMaxChildB[3], &childB, sizeof(unsigned int));
	retMin = vec3(min(leftMin.x, rightMin.x), min(leftMin.y, rightMin.y), min(leftMin.z, rightMin.z));
	retMax = vec3(max(leftMax.x, rightMax.x), max(leftMax.y, rightMax.y), max(leftMax.z, rightMax.z));
	retVal->aabbMinChildA[0] = retMin.x;
	retVal->aabbMinChildA[1] = retMin.y;
	retVal->aabbMinChildA[2] = retMin.z;
	retVal->aabbMaxChildB[0] = retMax.x;
	retVal->aabbMaxChildB[1] = retMax.y;
	retVal->aabbMaxChildB[2] = retMax.z;
	return retNodePosition;
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::ProduceBVH(SDFLeaf * inpGrids, int inpNGrids, vec3 mapMin, vec3 mapMax)
{
	nGrids = inpNGrids;
	gridArr = inpGrids;
	unsigned int nodeSizeAligned = nGrids * 2 - 1;
	nodeSizeAligned = (unsigned int)(ceil((double)nodeSizeAligned / 2000.0) * 2000);
	nodes.resize(nodeSizeAligned);
	this->mapMin = mapMin;
	this->mapMax = mapMax;

	std::thread mortonThread1(CalcMortons, gridArr, 0, nGrids / 2, mapMin, mapMax);
	std::thread mortonThread2(CalcMortons, gridArr, nGrids / 2, nGrids, mapMin, mapMax);
	mortonThread1.join();
	mortonThread2.join();

	QuickSort(gridArr, 0, nGrids - 1);

	unsigned int nodeCounter = 0;

	vec3 retMapMin, retMapMax;
	genHierarchy(nodeCounter, 0, nGrids - 1, retMapMin, retMapMax);
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::ProduceNodesOnly(SDFLeaf * inpGrids, BVHNode * inpNodesOutsidePtr, int inpNGrids)
{
	gridArr = inpGrids;
	nodesOutsidePtr = inpNodesOutsidePtr;

	unsigned int nodeCounter = 0;

	vec3 retMapMin, retMapMax;
	genHierarchyExternal(nodeCounter, 0, inpNGrids - 1, retMapMin, retMapMax);
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::getNumGrids()
{
	return nGrids;
}

void * HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::getNodes()
{
	return (void *)nodes.data();
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::getNodesSizeAligned()
{
	return (unsigned int)nodes.size();
}