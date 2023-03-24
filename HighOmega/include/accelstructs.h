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

#ifndef HIGHOMEGA_ACCELSTRUCTS_H
#define HIGHOMEGA_ACCELSTRUCTS_H

#include "vmath.h"
#include "util.h"
#include <thread>
#include "gl.h"

using namespace HIGHOMEGA::MATH;
using namespace HIGHOMEGA::GL;

namespace HIGHOMEGA
{
	namespace MATH
	{
		namespace ACCEL_STRUCT
		{
			struct SDFLeaf
			{
				float leafMinMorton[4];
				float leafMaxLeafId[4];
			};

			struct BVHTriangle
			{
				float e1Instance[4], e2Primitive[4], e3Morton[4];
			};

			struct BVHTriangleCompressed
			{
				unsigned int e1InstComp1e2InstComp2e3InstComp3OpaqueFlagPrimID[4];
			};

			struct BVHNode
			{
				float aabbMinChildA[4], aabbMaxChildB[4];
			};

			unsigned int expandBits(unsigned int v);
			unsigned int morton3D(float x, float y, float z);
			int countLeadingZeroes(unsigned int  inpCode);

			class BVHGenCPUClass
			{
			private:
				int nTris;
				BVHTriangle * triangleArr;
				std::vector<BVHTriangleCompressed> triangleArrComp;
				std::vector<BVHNode> nodes;
				BVHNode *nodesOutsidePtr = nullptr;
				std::vector<InstanceProperties> *instPropsRef;
				vec3 mapMin, mapMax;

				static void CalcMortons(BVHTriangle *triangleArr, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap);
				unsigned int QuickSortPartition(BVHTriangle * A, unsigned int lo, unsigned int hi);
				void QuickSort(BVHTriangle * A, unsigned int lo, unsigned int hi);
				int findSplit(int first, int last);
				unsigned int genHierarchyCompressTris(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
				unsigned int genHierarchy(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
			public:
				void ProduceBVH(BVHTriangle * inpTris, int inpNTris, vec3 mapMin, vec3 mapMax, std::vector<InstanceProperties> & inpInstPropsRef);
				void ProduceNodesOnly(BVHTriangle * inpTris, BVHNode *inpNodesOutsidePtr, int inpNTris);
				int getNumTris();
				void *getNodes();
				void *getTrianglesCompressed();
				unsigned int getNodesSizeAligned();
				unsigned int getTrianglesCompressedSizeAligned();
			};

			class SDFBVHClass
			{
			private:
				int nGrids;
				SDFLeaf * gridArr;
				std::vector<BVHNode> nodes;
				BVHNode *nodesOutsidePtr = nullptr;
				vec3 mapMin, mapMax;

				static void CalcMortons(SDFLeaf *sdfLeaves, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap);
				unsigned int QuickSortPartition(SDFLeaf * A, unsigned int lo, unsigned int hi);
				void QuickSort(SDFLeaf * A, unsigned int lo, unsigned int hi);
				int findSplit(int first, int last);
				unsigned int genHierarchy(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
				unsigned int genHierarchyExternal(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
			public:
				void ProduceBVH(SDFLeaf * inpGrids, int inpNGrids, vec3 mapMin, vec3 mapMax);
				void ProduceNodesOnly(SDFLeaf * inpGrids, BVHNode *inpNodesOutsidePtr, int inpNGrids);
				int getNumGrids();
				void *getNodes();
				unsigned int getNodesSizeAligned();
			};
		}
	}
}

#endif