#include "accelstructs.h"

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::expandBits(unsigned int v)
{
	v = (v * 0x00010001u) & 0xFF0000FFu;
	v = (v * 0x00000101u) & 0x0F00F00Fu;
	v = (v * 0x00000011u) & 0xC30C30C3u;
	v = (v * 0x00000005u) & 0x49249249u;
	return v;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::morton3D(float x, float y, float z)
{
	x = min(max(x * 1024.0f, 0.0f), 1023.0f);
	y = min(max(y * 1024.0f, 0.0f), 1023.0f);
	z = min(max(z * 1024.0f, 0.0f), 1023.0f);
	unsigned int xx = expandBits((unsigned int)x);
	unsigned int yy = expandBits((unsigned int)y);
	unsigned int zz = expandBits((unsigned int)z);
	return xx * 4 + yy * 2 + zz;
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::CalcMortons(BVHTriangle * triangleArr, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap)
{
	vec3 invMapSize = vec3(1.0f / (maxMap.x - minMap.x), 1.0f / (maxMap.y - minMap.y), 1.0f / (maxMap.z - minMap.z));
	for (int i = start; i != end; i++) {
		BVHTriangle & curTri = triangleArr[i];
		vec3 cent = (vec3(curTri.e1MinComp[3], curTri.e2MinComp[3], curTri.e3MinComp[3]) + vec3(curTri.nMaxComp[3], curTri.tMaxComp[3], curTri.bMaxComp[3])) * 0.5f;
		cent.x = (cent.x - minMap.x) * invMapSize.x;
		cent.y = (cent.y - minMap.y) * invMapSize.y;
		cent.z = (cent.z - minMap.z) * invMapSize.z;

		triangleArr[i].mortonCode = morton3D(cent.x, cent.y, cent.z);
	}
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::QuickSortPartition(BVHTriangle * A, unsigned int lo, unsigned int hi)
{
	unsigned int pivotCode = A[(lo + hi) / 2].mortonCode;
	unsigned int i = lo - 1;
	unsigned int j = hi + 1;
	for (;;)
	{
		do
		{
			i++;
		} while (A[i].mortonCode < pivotCode);

		do
		{
			j--;
		} while (A[j].mortonCode > pivotCode);

		if (i >= j) return j;

		BVHTriangle Ai;
		memcpy(&Ai, &A[i], sizeof(BVHTriangle));
		memcpy(&A[i], &A[j], sizeof(BVHTriangle));
		memcpy(&A[j], &Ai, sizeof(BVHTriangle));
	}
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::QuickSort(BVHTriangle * A, unsigned int lo, unsigned int hi)
{
	if (lo < hi) {
		unsigned int p = QuickSortPartition(A, lo, hi);
		QuickSort(A, lo, p);
		QuickSort(A, p + 1, hi);
	}
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::countLeadingZeroes(unsigned int inpCode)
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

int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::findSplit(int first, int last)
{
	unsigned int firstCode = triangleArr[first].mortonCode;
	unsigned int lastCode = triangleArr[last].mortonCode;

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
			unsigned int splitCode = triangleArr[newSplit].mortonCode;
			int splitPrefix = countLeadingZeroes(firstCode ^ splitCode);
			if (splitPrefix > commonPrefix)
				split = newSplit;
		}
	} while (step > 1);

	return split;
}

unsigned int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::genHierarchy(BVHInternalNode * allNodes, unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax)
{
	if (lo == hi)
	{
		BVHInternalNode *retVal = &allNodes[nodeCounter];
		nodeCounter++;
		retVal->ChildA = retVal->ChildB = 0;

		BVHTriangle & leafTri = triangleArr[lo];

		retVal->leafPtr = lo;

		retVal->aabbMin[0] = leafTri.e1MinComp[3];
		retVal->aabbMin[1] = leafTri.e2MinComp[3];
		retVal->aabbMin[2] = leafTri.e3MinComp[3];

		retVal->aabbMax[0] = leafTri.nMaxComp[3];
		retVal->aabbMax[1] = leafTri.tMaxComp[3];
		retVal->aabbMax[2] = leafTri.bMaxComp[3];

		retMin = vec3(retVal->aabbMin[0], retVal->aabbMin[1], retVal->aabbMin[2]);
		retMax = vec3(retVal->aabbMax[0], retVal->aabbMax[1], retVal->aabbMax[2]);

		return nodeCounter - 1;
	}

	BVHInternalNode *retVal = &allNodes[nodeCounter];
	nodeCounter++;
	unsigned int retNodePosition = nodeCounter - 1;
	int split = findSplit(lo, hi);

	vec3 leftMin, leftMax, rightMin, rightMax;
	retVal->ChildA = genHierarchy(allNodes, nodeCounter, lo, split, leftMin, leftMax);
	retVal->ChildB = genHierarchy(allNodes, nodeCounter, split + 1, hi, rightMin, rightMax);
	retMin = vec3(min(leftMin.x, rightMin.x), min(leftMin.y, rightMin.y), min(leftMin.z, rightMin.z));
	retMax = vec3(max(leftMax.x, rightMax.x), max(leftMax.y, rightMax.y), max(leftMax.z, rightMax.z));
	retVal->aabbMin[0] = retMin.x;
	retVal->aabbMin[1] = retMin.y;
	retVal->aabbMin[2] = retMin.z;
	retVal->aabbMax[0] = retMax.x;
	retVal->aabbMax[1] = retMax.y;
	retVal->aabbMax[2] = retMax.z;
	return retNodePosition;
}

void HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::ProduceInternalNodes(BVHTriangle * inpTris, int inpNTris)
{
	nTris = inpNTris;
	triangleArr = inpTris;

	internalNodes = new BVHInternalNode[nTris * 2 - 1];

	std::thread mortonThread1(CalcMortons, triangleArr, 0, nTris / 2, mapMin, mapMax);
	std::thread mortonThread2(CalcMortons, triangleArr, nTris / 2, nTris, mapMin, mapMax);
	mortonThread1.join();
	mortonThread2.join();

	QuickSort(triangleArr, 0, nTris - 1);

	unsigned int nodeCounter = 0;

	vec3 retMapMin, retMapMax;
	genHierarchy(internalNodes, nodeCounter, 0, nTris - 1, retMapMin, retMapMax);
}

int HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::getNumTris()
{
	return nTris;
}

void * HIGHOMEGA::MATH::ACCEL_STRUCT::BVHGenClass::getInternalNodes()
{
	return (void *)internalNodes;
}
