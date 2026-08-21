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
	x = Clamp(x * 1024.0f, 0.0f, 1023.0f);
	y = Clamp(y * 1024.0f, 0.0f, 1023.0f);
	z = Clamp(z * 1024.0f, 0.0f, 1023.0f);
	unsigned int xx = expandBits((unsigned int)x);
	unsigned int yy = expandBits((unsigned int)y);
	unsigned int zz = expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::RadixSort(std::vector<BoxLeaf>& inLeaves, std::vector<BoxLeaf>& inLeavesTmp)
{
	inLeavesTmp.resize(inLeaves.size());

	const unsigned int numPredicates = 256;
	for (unsigned int bitShift = 0; bitShift != 32; bitShift += 8)
	{
		unsigned int bitMask = (0x000000FFu) << bitShift;
		BoxLeaf* srcBuf, * dstBuf;
		if ((bitShift / 8) % 2 == 0)
		{
			srcBuf = inLeaves.data();
			dstBuf = inLeavesTmp.data();
		}
		else
		{
			srcBuf = inLeavesTmp.data();
			dstBuf = inLeaves.data();
		}
		unsigned int predicateSums[numPredicates];
		unsigned int predicateOffets[numPredicates];
		for (unsigned int i = 0; i != numPredicates; i++)
			predicateSums[i] = 0;
		for (unsigned int i = 0; i != inLeaves.size(); i++)
			predicateSums[((*(unsigned int*)(&srcBuf[i].leafMinMorton[3])) & bitMask) >> bitShift]++;
		for (unsigned int i = 0; i != numPredicates; i++)
		{
			predicateOffets[i] = 0;
			for (unsigned int j = 0; j != i; j++)
				predicateOffets[i] += predicateSums[j];
		}
		for (unsigned int i = 0; i != inLeaves.size(); i++)
		{
			unsigned int curPred = ((*(unsigned int*)(&srcBuf[i].leafMinMorton[3]) & bitMask)) >> bitShift;
			dstBuf[predicateOffets[curPred]] = srcBuf[i];
			predicateOffets[curPred]++;
		}
	}
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

	int commonPrefix = std::countl_zero(firstCode ^ lastCode);

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
			int splitPrefix = std::countl_zero(firstCode ^ splitCode);
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

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::CalcMortons(BoxLeaf * sdfLeaves, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap)
{
	vec3 invMapSize = vec3(1.0f / (maxMap.x - minMap.x), 1.0f / (maxMap.y - minMap.y), 1.0f / (maxMap.z - minMap.z));
	for (int i = start; i != end; i++) {
		BoxLeaf & curGrid = sdfLeaves[i];
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

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::QuickSortPartition(BoxLeaf * A, unsigned int lo, unsigned int hi)
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

		BoxLeaf Ai;
		memcpy(&Ai, &A[i], sizeof(BoxLeaf));
		memcpy(&A[i], &A[j], sizeof(BoxLeaf));
		memcpy(&A[j], &Ai, sizeof(BoxLeaf));
	}
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::QuickSort(BoxLeaf * A, unsigned int lo, unsigned int hi)
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

	int commonPrefix = std::countl_zero(firstCode ^ lastCode);

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
			int splitPrefix = std::countl_zero(firstCode ^ splitCode);
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

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::ProduceBVH(BoxLeaf * inpGrids, int inpNGrids, vec3 mapMin, vec3 mapMax)
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

void HIGHOMEGA::MATH::ACCEL_STRUCT::SDFBVHClass::ProduceNodesOnly(BoxLeaf * inpGrids, BVHNode * inpNodesOutsidePtr, int inpNGrids)
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

void HIGHOMEGA::MATH::ACCEL_STRUCT::CWBVHClass::Build(std::vector<BoxLeaf>& sdfLeaves)
{
	// Quick n' dirty BVH8 off of Morton sorted volumes
	unsigned int increaseByNodeSize = (unsigned int)ceil((double)sdfLeaves.size() / 8.0);
	nodes.resize(increaseByNodeSize);

	unsigned int childCounter = 0u;
	for (unsigned int i = 0; i != nodes.size(); i++)
	{
		nodes[i].AABBMin[0] = nodes[i].AABBMin[1] = nodes[i].AABBMin[2] = FLT_MAX;
		nodes[i].AABBMax[0] = nodes[i].AABBMax[1] = nodes[i].AABBMax[2] = -FLT_MAX;
		for (unsigned int j = 0; j != 8; j++)
		{
			nodes[i].childId[j] = childCounter;
			nodes[i].childType[j] = CHILDTYPE::LEAF;
			nodes[i].AABBMin[0] = min(nodes[i].AABBMin[0], sdfLeaves[childCounter].leafMinMorton[0]);
			nodes[i].AABBMin[1] = min(nodes[i].AABBMin[1], sdfLeaves[childCounter].leafMinMorton[1]);
			nodes[i].AABBMin[2] = min(nodes[i].AABBMin[2], sdfLeaves[childCounter].leafMinMorton[2]);
			nodes[i].AABBMax[0] = max(nodes[i].AABBMax[0], sdfLeaves[childCounter].leafMaxLeafId[0]);
			nodes[i].AABBMax[1] = max(nodes[i].AABBMax[1], sdfLeaves[childCounter].leafMaxLeafId[1]);
			nodes[i].AABBMax[2] = max(nodes[i].AABBMax[2], sdfLeaves[childCounter].leafMaxLeafId[2]);
			childCounter++;
			if (childCounter >= sdfLeaves.size()) break;
		}
		if (childCounter >= sdfLeaves.size()) break;
	}

	while (increaseByNodeSize > 1)
	{
		unsigned int newIncreaseByNodeSize = (unsigned int)ceil((double)increaseByNodeSize / 8.0);

		std::vector<BVHNode> furtherInternalNodes;
		furtherInternalNodes.resize(newIncreaseByNodeSize);

		childCounter = 0u;
		for (unsigned int i = 0; i != newIncreaseByNodeSize; i++)
		{
			furtherInternalNodes[i].AABBMin[0] = furtherInternalNodes[i].AABBMin[1] = furtherInternalNodes[i].AABBMin[2] = FLT_MAX;
			furtherInternalNodes[i].AABBMax[0] = furtherInternalNodes[i].AABBMax[1] = furtherInternalNodes[i].AABBMax[2] = -FLT_MAX;
			for (unsigned int j = 0; j != 8; j++)
			{
				furtherInternalNodes[i].childId[j] = childCounter;
				furtherInternalNodes[i].childType[j] = CHILDTYPE::SUBTREE;
				furtherInternalNodes[i].AABBMin[0] = min(furtherInternalNodes[i].AABBMin[0], nodes[childCounter].AABBMin[0]);
				furtherInternalNodes[i].AABBMin[1] = min(furtherInternalNodes[i].AABBMin[1], nodes[childCounter].AABBMin[1]);
				furtherInternalNodes[i].AABBMin[2] = min(furtherInternalNodes[i].AABBMin[2], nodes[childCounter].AABBMin[2]);
				furtherInternalNodes[i].AABBMax[0] = max(furtherInternalNodes[i].AABBMax[0], nodes[childCounter].AABBMax[0]);
				furtherInternalNodes[i].AABBMax[1] = max(furtherInternalNodes[i].AABBMax[1], nodes[childCounter].AABBMax[1]);
				furtherInternalNodes[i].AABBMax[2] = max(furtherInternalNodes[i].AABBMax[2], nodes[childCounter].AABBMax[2]);
				childCounter++;
				if (childCounter >= increaseByNodeSize) break;
			}
			if (childCounter >= increaseByNodeSize) break;
		}

		unsigned int furtherInternalNodesSize = (unsigned int)furtherInternalNodes.size();
		for (int i = 0; i != nodes.size(); i++)
			for (int j = 0; j != 8; j++)
				if (nodes[i].childType[j] == CHILDTYPE::SUBTREE)
					nodes[i].childId[j] += furtherInternalNodesSize;
		for (int i = 0; i != furtherInternalNodes.size(); i++)
			for (int j = 0; j != 8; j++)
				if (furtherInternalNodes[i].childType[j] == CHILDTYPE::SUBTREE)
					furtherInternalNodes[i].childId[j] += furtherInternalNodesSize;

		furtherInternalNodes.insert(furtherInternalNodes.end(), nodes.begin(), nodes.end());

		nodes = furtherInternalNodes;

		increaseByNodeSize = newIncreaseByNodeSize;
	}

	// Child node ordering
	for (unsigned int i = 0; i != nodes.size(); i++)
	{
		vec3 parentMin = vec3(nodes[i].AABBMin[0], nodes[i].AABBMin[1], nodes[i].AABBMin[2]);
		vec3 parentMax = vec3(nodes[i].AABBMax[0], nodes[i].AABBMax[1], nodes[i].AABBMax[2]);
		vec3 parentCentroid = (parentMin + parentMax) * 0.5f;
		float cost[8][8];
		for (unsigned int j = 0; j != 8; j++)
		{
			if (nodes[i].childType[j] == EMPTY)
			{
				for (unsigned int k = 0; k != 8; k++)
					cost[k][j] = FLT_MAX;
				continue;
			}
			unsigned int curChildId = nodes[i].childId[j];
			vec3 childMin, childMax;
			if (nodes[i].childType[j] == LEAF)
			{
				childMin = vec3(sdfLeaves[curChildId].leafMinMorton[0], sdfLeaves[curChildId].leafMinMorton[1], sdfLeaves[curChildId].leafMinMorton[2]);
				childMax = vec3(sdfLeaves[curChildId].leafMaxLeafId[0], sdfLeaves[curChildId].leafMaxLeafId[1], sdfLeaves[curChildId].leafMaxLeafId[2]);
			}
			else
			{
				childMin = vec3(nodes[curChildId].AABBMin[0], nodes[curChildId].AABBMin[1], nodes[curChildId].AABBMin[2]);
				childMax = vec3(nodes[curChildId].AABBMax[0], nodes[curChildId].AABBMax[1], nodes[curChildId].AABBMax[2]);
			}
			vec3 childCentroid = (childMin + childMax) * 0.5f;

			for (unsigned int k = 0; k != 8; k++)
			{
				vec3 sDir = vec3 ((((k >> 2) & 1) == 1) ? -1.0f : 1.0f, (((k >> 1) & 1) == 1) ? -1.0f : 1.0f, (((k >> 0) & 1) == 1) ? -1.0f : 1.0f);
				cost[k][j] = (childCentroid - parentCentroid) * sDir;
			}
		}

		bool slotUsed[8] = { false, false, false, false, false, false, false, false };
		bool childAssigned[8] = { false, false, false, false, false, false, false, false };
		unsigned int childAssignment[8];

		while (true)
		{
			bool minChildFound = false;
			float minCost = FLT_MAX;
			unsigned int minChild;
			unsigned int minS;

			for (unsigned int j = 0; j != 8; j++)
				for (unsigned int k = 0; k != 8; k++)
					if (!slotUsed[k] && !childAssigned[j] && cost[k][j] < minCost)
					{
						minCost = cost[k][j];
						minS = k;
						minChild = j;
						minChildFound = true;
					}

			if (minChildFound)
			{
				slotUsed[minS] = true;
				childAssigned[minChild] = true;
				childAssignment[minChild] = minS;
			}
			else
				break;

			for (unsigned int j = 0; j != 8; j++)
			{
				if (childAssigned[j]) continue;
				for (unsigned int k = 0; k != 8; k++)
				{
					if (slotUsed[k]) continue;
					slotUsed[k] = true;
					childAssigned[j] = true;
					childAssignment[j] = k;
					break;
				}
			}
		}

		BVHNode curNode = nodes[i];
		for (unsigned int j = 0; j != 8; j++)
		{
			nodes[i].childId[childAssignment[j]] = curNode.childId[j];
			nodes[i].childType[childAssignment[j]] = curNode.childType[j];
		}
	}

	// Node compression
	compressedNodes.resize(nodes.size());

	const float Nq = 8.0f;
	const float inv2Nq_1 = 1.0f / (powf(2.0f, Nq) - 1.0f);

	for (unsigned int i = 0; i != nodes.size(); i++)
	{
		compressedNodes[i].Px = nodes[i].AABBMin[0];
		compressedNodes[i].Py = nodes[i].AABBMin[1];
		compressedNodes[i].Pz = nodes[i].AABBMin[2];

		vec3 nodeAABBLen = vec3(nodes[i].AABBMax[0], nodes[i].AABBMax[1], nodes[i].AABBMax[2]) - vec3(nodes[i].AABBMin[0], nodes[i].AABBMin[1], nodes[i].AABBMin[2]);

		compressedNodes[i].ex = (char)ceilf(log2f(nodeAABBLen.x * inv2Nq_1));
		compressedNodes[i].ey = (char)ceilf(log2f(nodeAABBLen.y * inv2Nq_1));
		compressedNodes[i].ez = (char)ceilf(log2f(nodeAABBLen.z * inv2Nq_1));
		compressedNodes[i].imask = 0;

		float inv2ex = 1.0f / powf(2.0f, (float)compressedNodes[i].ex);
		float inv2ey = 1.0f / powf(2.0f, (float)compressedNodes[i].ey);
		float inv2ez = 1.0f / powf(2.0f, (float)compressedNodes[i].ez);

		vec3 nodeMin = vec3(nodes[i].AABBMin[0], nodes[i].AABBMin[1], nodes[i].AABBMin[2]);
		vec3 nodeMax = vec3(nodes[i].AABBMax[0], nodes[i].AABBMax[1], nodes[i].AABBMax[2]);

		unsigned int childNodeBaseIndex = 0xFFFFFFFFu;
		unsigned int primitiveBaseIndex = 0xFFFFFFFFu;
		for (unsigned int j = 0; j != 8; j++)
		{
			if (nodes[i].childType[j] == EMPTY) continue;
			if (nodes[i].childType[j] == LEAF)
				primitiveBaseIndex = min(nodes[i].childId[j], primitiveBaseIndex);
			else
				childNodeBaseIndex = min(nodes[i].childId[j], childNodeBaseIndex);
		}
		compressedNodes[i].childNodeBaseIndex = childNodeBaseIndex;
		compressedNodes[i].primitiveBaseIndex = primitiveBaseIndex;

		for (unsigned int j = 0; j != 8; j++)
		{
			vec3 childMin, childMax;
			unsigned int curChildId = nodes[i].childId[j];
			if (nodes[i].childType[j] == EMPTY)
			{
				compressedNodes[i].meta[j] = 0;
				continue;
			}
			compressedNodes[i].meta[j] = (unsigned char)0x20; // Even leaf nodes have 1 'primitive' in our setup...
			if (nodes[i].childType[j] == LEAF)
			{
				childMin = vec3(sdfLeaves[curChildId].leafMinMorton[0], sdfLeaves[curChildId].leafMinMorton[1], sdfLeaves[curChildId].leafMinMorton[2]);
				childMax = vec3(sdfLeaves[curChildId].leafMaxLeafId[0], sdfLeaves[curChildId].leafMaxLeafId[1], sdfLeaves[curChildId].leafMaxLeafId[2]);
				compressedNodes[i].meta[j] |= (unsigned char)min (curChildId - primitiveBaseIndex, 23); // We ultimately wants this child Id to index into the final array...
			}
			else
			{
				childMin = vec3(nodes[curChildId].AABBMin[0], nodes[curChildId].AABBMin[1], nodes[curChildId].AABBMin[2]);
				childMax = vec3(nodes[curChildId].AABBMax[0], nodes[curChildId].AABBMax[1], nodes[curChildId].AABBMax[2]);
				compressedNodes[i].imask |= (1 << j);
				compressedNodes[i].meta[j] |= (unsigned char)(24 + min(nodes[i].childId[j] - childNodeBaseIndex, 7));
			}
			vec3 loDiff = childMin - nodeMin;
			vec3 hiDiff = childMax - nodeMin;
			compressedNodes[i].qlox[j] = (unsigned char)floorf(loDiff.x * inv2ex);
			compressedNodes[i].qloy[j] = (unsigned char)floorf(loDiff.y * inv2ey);
			compressedNodes[i].qloz[j] = (unsigned char)floorf(loDiff.z * inv2ez);
			compressedNodes[i].qhix[j] = (unsigned char)ceilf(hiDiff.x * inv2ex);
			compressedNodes[i].qhiy[j] = (unsigned char)ceilf(hiDiff.y * inv2ey);
			compressedNodes[i].qhiz[j] = (unsigned char)ceilf(hiDiff.z * inv2ez);
		}
	}
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::CWBVHClass::FindBVH8Overlaps(unsigned int nodeId, std::vector<BoxLeaf>& boxLeaves, vec3& boxMin, vec3& boxMax, std::vector<unsigned int>& bvh8overlaps)
{
	if (nodes.size() == 0) return;

	if (boxMin.x > nodes[nodeId].AABBMax[0] ||
		boxMax.x < nodes[nodeId].AABBMin[0] ||
		boxMin.y > nodes[nodeId].AABBMax[1] ||
		boxMax.y < nodes[nodeId].AABBMin[1] ||
		boxMin.z > nodes[nodeId].AABBMax[2] ||
		boxMax.z < nodes[nodeId].AABBMin[2])
		return;

	for (unsigned int i = 0; i != 8; i++)
		switch (nodes[nodeId].childType[i])
		{
			case CWBVHClass::CHILDTYPE::EMPTY:
			{
				continue;
			}
			case CWBVHClass::CHILDTYPE::LEAF:
			{
				if (!(boxMin.x > boxLeaves[nodes[nodeId].childId[i]].leafMaxLeafId[0] ||
					boxMax.x < boxLeaves[nodes[nodeId].childId[i]].leafMinMorton[0] ||
					boxMin.y > boxLeaves[nodes[nodeId].childId[i]].leafMaxLeafId[1] ||
					boxMax.y < boxLeaves[nodes[nodeId].childId[i]].leafMinMorton[1] ||
					boxMin.z > boxLeaves[nodes[nodeId].childId[i]].leafMaxLeafId[2] ||
					boxMax.z < boxLeaves[nodes[nodeId].childId[i]].leafMinMorton[2]))
				{
					float leafId = boxLeaves[nodes[nodeId].childId[i]].leafMaxLeafId[3];
					bvh8overlaps.push_back(*((unsigned int*)&leafId));
				}
				break;
			}
			case CWBVHClass::CHILDTYPE::SUBTREE:
			{
				FindBVH8Overlaps(nodes[nodeId].childId[i], boxLeaves, boxMin, boxMax, bvh8overlaps);
				break;
			}
		}
}
