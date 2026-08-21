#ifndef HIGHOMEGA_ACCELSTRUCTS_H
#define HIGHOMEGA_ACCELSTRUCTS_H

#include "vmath.h"
#include <thread>

using namespace HIGHOMEGA::MATH;

namespace HIGHOMEGA
{
	namespace MATH
	{
		namespace ACCEL_STRUCT
		{
			struct BVHTriangle
			{
				float e1MinComp[4], e2MinComp[4], e3MinComp[4];
				float nMaxComp[4], tMaxComp[4], bMaxComp[4];
				float uv1uv2[4];
				float uv3[2];
				float dielectricAndIoR;
				float emissivity;
				unsigned int diffUVCoords[4];
				unsigned int normUVCoords[4];
				unsigned int roughUVCoords[4];
				unsigned int mortonCode;
				unsigned int reserved[3];
			};

			struct BVHInternalNode
			{
				float aabbMin[4], aabbMax[4];
				unsigned int ChildA, ChildB, leafPtr, reserved;
			};

			class BVHGenClass
			{
			private:
				int nTris;
				BVHTriangle * triangleArr;
				BVHInternalNode * internalNodes;
				vec3 mapMin, mapMax;

				static unsigned int expandBits(unsigned int v);
				static unsigned int morton3D(float x, float y, float z);
				static void CalcMortons(BVHTriangle *triangleArr, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap);
				unsigned int QuickSortPartition(BVHTriangle * A, unsigned int lo, unsigned int hi);
				void QuickSort(BVHTriangle * A, unsigned int lo, unsigned int hi);
				int countLeadingZeroes(unsigned int  inpCode);
				int findSplit(int first, int last);
				unsigned int genHierarchy(BVHInternalNode * allNodes, unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
			public:
				void ProduceInternalNodes(BVHTriangle * inpTris, int inpNTris);
				int getNumTris();
				void *getInternalNodes();
			};
		}
	}
}

#endif