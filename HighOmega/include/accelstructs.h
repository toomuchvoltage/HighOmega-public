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
			struct BoxLeaf
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
			void RadixSort(std::vector <BoxLeaf>& inLeaves, std::vector <BoxLeaf>& inLeavesTmp);

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

			/* GLSL component of SDF-CompactLBVH
			
			struct GridBVHInternalNode
			{
				vec4 aabbMinChildA;
				vec4 aabbMaxChildB;
			};

			layout (set = 3, binding = 0) buffer GridBVHInternalNodeSSBO
			{
				GridBVHInternalNode nodes[];
			} gridBVHNodeArr;

			uint getChildProcessed (int stackPointer, uint childProcessed3, uint childProcessed2, uint childProcessed1)
			{
				int bitStartIndex = int (stackPointer * 2);
				if ( bitStartIndex >= 64 )
				{
					bitStartIndex -= 64;
					return bitfieldExtract (childProcessed3, bitStartIndex, 2);
				}
				else if ( bitStartIndex >= 32 )
				{
					bitStartIndex -= 32;
					return bitfieldExtract (childProcessed2, bitStartIndex, 2);
				}
				else
				{
					return bitfieldExtract (childProcessed1, bitStartIndex, 2);
				}
			}

			void setChildProcessed (int stackPointer, inout uint childProcessed3, inout uint childProcessed2, inout uint childProcessed1, uint setVal)
			{
				int bitStartIndex = int (stackPointer * 2);
				if ( bitStartIndex >= 64 )
				{
					bitStartIndex -= 64;
					childProcessed3 = bitfieldInsert (childProcessed3, setVal, bitStartIndex, 2);
				}
				else if ( bitStartIndex >= 32 )
				{
					bitStartIndex -= 32;
					childProcessed2 = bitfieldInsert (childProcessed2, setVal, bitStartIndex, 2);
				}
				else
				{
					childProcessed1 = bitfieldInsert (childProcessed1, setVal, bitStartIndex, 2);
				}
			}

			bool LineGridBVH (vec3 p1, vec3 p2, out VoxelHit hitProps)
			{
				uint nodeStack[33] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
									  0, 0, 0};
				uint childProcessed3 = 0, childProcessed2 = 0, childProcessed1 = 0;
				int stackPointer = 0;
				nodeStack[stackPointer] = 0;

				vec3 foundP2 = p2;
				vec3 voxCent = p2;
				vec3 m = p2 - p1;
				vec3 invm = 1.0 / m;
				bool intersectionFound = false;
				vec4 packedMat;
				vec3 hitNorm = vec3 (1.0, 0.0, 0.0);

				uint chitInstanceId, chitTriId;
				vec3 chitBarycoords;

				while (true)
				{
					GridBVHInternalNode curNode = gridBVHNodeArr.nodes[nodeStack[stackPointer]];
					vec3 nodeAABBMin = curNode.aabbMinChildA.xyz;
					vec3 nodeAABBMax = curNode.aabbMaxChildB.xyz;

					if ( floatBitsToUint (curNode.aabbMinChildA.a) != floatBitsToUint (curNode.aabbMaxChildB.a) )
					{
						uint curChildProcessed = getChildProcessed (stackPointer, childProcessed3, childProcessed2, childProcessed1);
						if ( curChildProcessed < 2 )
						{
							float tMin, tMax;
							if ( curChildProcessed == 1 || rayBoxIntersectTime (p1, invm, nodeAABBMin, nodeAABBMax, tMin, tMax) )
							{
								setChildProcessed (stackPointer, childProcessed3, childProcessed2, childProcessed1, curChildProcessed + 1);
								stackPointer++;
								nodeStack[stackPointer] = (curChildProcessed == 0) ? floatBitsToUint (curNode.aabbMinChildA.a) : floatBitsToUint (curNode.aabbMaxChildB.a);
								setChildProcessed (stackPointer, childProcessed3, childProcessed2, childProcessed1, 0);
							}
							else
							{
								stackPointer--;
								if ( stackPointer == -1 ) break;
							}
						}
						else
						{
							stackPointer--;
							if ( stackPointer == -1 ) break;
						}
					}
					else
					{
						if ( TraceGrid (p1, foundP2, voxCent, nodeAABBMin, nodeAABBMax, floatBitsToUint (curNode.aabbMinChildA.a), packedMat, hitNorm) )
						{
							m = foundP2 - p1;
							invm = 1.0 / m;
							intersectionFound = true;
						}
						stackPointer--;
						if ( stackPointer == -1 ) break;
					}
				}

				hitProps = getHit (packedMat, hitNorm, m, foundP2, voxCent);
				return intersectionFound;
			}
			*/

			class SDFBVHClass
			{
			private:
				int nGrids;
				BoxLeaf * gridArr;
				std::vector<BVHNode> nodes;
				BVHNode *nodesOutsidePtr = nullptr;
				vec3 mapMin, mapMax;

				unsigned int QuickSortPartition(BoxLeaf * A, unsigned int lo, unsigned int hi);
				void QuickSort(BoxLeaf * A, unsigned int lo, unsigned int hi);
				int findSplit(int first, int last);
				unsigned int genHierarchy(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
				unsigned int genHierarchyExternal(unsigned int & nodeCounter, int lo, int hi, vec3 & retMin, vec3 & retMax);
			public:
				static void CalcMortons(BoxLeaf* sdfLeaves, unsigned int start, unsigned int end, vec3 minMap, vec3 maxMap);
				void ProduceBVH(BoxLeaf * inpGrids, int inpNGrids, vec3 mapMin, vec3 mapMax);
				void ProduceNodesOnly(BoxLeaf * inpGrids, BVHNode *inpNodesOutsidePtr, int inpNGrids);
				int getNumGrids();
				void *getNodes();
				unsigned int getNodesSizeAligned();
			};

			class CWBVHClass
			{
			public:
				enum CHILDTYPE
				{
					EMPTY = 0,
					SUBTREE,
					LEAF
				};
				struct BVHNode
				{
					CHILDTYPE childType[8] = { EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY, EMPTY };
					unsigned int childId[8];
					float AABBMin[3];
					float AABBMax[3];
				};
				typedef unsigned int uint;
				struct uvec2
				{
					struct
					{
						uint x, y;
					};
					uvec2() = default;
					uvec2(uint inXY)
					{
						x = y = inXY;
					}
					uvec2(uint inX, uint inY)
					{
						x = inX;
						y = inY;
					}
				};
				struct uvec3
				{
					struct
					{
						uint x, y, z;
					};
					uvec3() = default;
					uvec3(uint inX, uint inY, uint inZ)
					{
						x = inX;
						y = inY;
						z = inZ;
					}
				};
				struct uvec4
				{
					struct
					{
						uint x, y, z, w;
					};
					uvec4() = default;
					uvec4(uint inX, uint inY, uint inZ, uint inW)
					{
						x = inX;
						y = inY;
						z = inZ;
						w = inW;
					}
				};
				struct CWBVHNode
				{
					float Px, Py, Pz;
					char ex, ey, ez, imask;
					unsigned int childNodeBaseIndex;
					unsigned int primitiveBaseIndex;
					unsigned char meta[8];
					unsigned char qlox[8];
					unsigned char qloy[8];
					unsigned char qloz[8];
					unsigned char qhix[8];
					unsigned char qhiy[8];
					unsigned char qhiz[8];
				};

				std::vector<BVHNode> nodes;
				std::vector<CWBVHNode> compressedNodes;

				inline unsigned int sign_extend_s8x4(unsigned int i)
				{
					unsigned int retVal =  ((i & 0x80000000) != 0) ? 0xFF000000 : 0;
					retVal |=			  (((i & 0x00800000) != 0) ? 0x00FF0000 : 0);
					retVal |=			  (((i & 0x00008000) != 0) ? 0x0000FF00 : 0);
					retVal |=			  (((i & 0x00000080) != 0) ? 0x000000FF : 0);
					return retVal;
				}
				inline unsigned int bitfieldExtract(unsigned int num, int offset, int bits) {
					return ((num >> offset) & ((1u << bits) - 1));
				}
				inline float uintBitsToFloat(unsigned int num) {
					return *((float*)&num);
				}
				inline unsigned int findMSB(unsigned int num) {
					return 31 - std::countl_zero(num);
				}
				template <bool IntersectLeaf(vec3&, vec3&, void *, unsigned int)>
				bool LineCWBVH(vec3& p1, vec3& p2, void *body, unsigned int& closestHitPrim)
				{
					uvec2 traversalStack[12];
					uint hitAddr = 0, stackPtr = 0;
					float tmin = 0.0f, tmax = 1.0f;
					vec3 rayD = p2 - p1;
					float originalRayLen = rayD.length();
					vec3 invRayD = rayD.inv();
					uint octinv = (7 - ((rayD.x < 0.0 ? 4 : 0) | (rayD.y < 0.0 ? 2 : 0) | (rayD.z < 0.0 ? 1 : 0))) * 0x1010101;
					bool retVal = false;
					uvec2 ngroup = uvec2(0, 0x80000000), tgroup = uvec2(0);
					do
					{
						if (ngroup.y > 0x00FFFFFF)
						{
							uint hits = ngroup.y, imask = ngroup.y;
							uint child_bit_index = findMSB(hits);
							uint child_node_base_index = ngroup.x;
							ngroup.y &= (~(1 << child_bit_index));
							if (ngroup.y > 0x00FFFFFF) { traversalStack[stackPtr++] = ngroup; }
							{
								uint slot_index = (child_bit_index - 24) ^ (octinv & 255);
								uint relative_index = std::popcount(imask & (~(0xFFFFFFFF << slot_index)));
								uint child_node_index = child_node_base_index + relative_index;
								uvec4 n0, n1, n2, n3, n4;
								memcpy(&n0.x, ((unsigned char*)&compressedNodes[child_node_index]), 16);
								memcpy(&n1.x, ((unsigned char*)&compressedNodes[child_node_index]) + 16, 16);
								memcpy(&n2.x, ((unsigned char*)&compressedNodes[child_node_index]) + 16 * 2, 16);
								memcpy(&n3.x, ((unsigned char*)&compressedNodes[child_node_index]) + 16 * 3, 16);
								memcpy(&n4.x, ((unsigned char*)&compressedNodes[child_node_index]) + 16 * 4, 16);
								vec3 p = vec3(uintBitsToFloat(n0.x), uintBitsToFloat(n0.y), uintBitsToFloat(n0.z));
								uvec3 e = uvec3(bitfieldExtract(n0.w, 0, 8), bitfieldExtract(n0.w, 8, 8), bitfieldExtract(n0.w, 16, 8));
								e = uvec3((e.x + 127) & 0xFF, (e.y + 127) & 0xFF, (e.z + 127) & 0xFF); // Kill the sign bit...
								ngroup.x = n1.x; tgroup.x = n1.y; tgroup.y = 0;
								uint hitmask = 0;
								float adjusted_idirx = uintBitsToFloat(e.x << 23u) * invRayD.x;
								float adjusted_idiry = uintBitsToFloat(e.y << 23u) * invRayD.y;
								float adjusted_idirz = uintBitsToFloat(e.z << 23u) * invRayD.z;
								float origx = -(p1.x - p.x) * invRayD.x;
								float origy = -(p1.y - p.y) * invRayD.y;
								float origz = -(p1.z - p.z) * invRayD.z;
								{    // First 4
									uint meta4 = n1.z, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
									uint inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
									uint bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
									uint child_bits4 = (meta4 >> 5) & 0x07070707;
									uint swizzledLox = (invRayD.x < 0.0) ? n3.z : n2.x, swizzledHix = (invRayD.x < 0.0) ? n2.x : n3.z;
									uint swizzledLoy = (invRayD.y < 0.0) ? n4.x : n2.z, swizzledHiy = (invRayD.y < 0.0) ? n2.z : n4.x;
									uint swizzledLoz = (invRayD.z < 0.0) ? n4.z : n3.x, swizzledHiz = (invRayD.z < 0.0) ? n3.x : n4.z;
									float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
									tminx[0] = bitfieldExtract(swizzledLox, 0, 8) * adjusted_idirx + origx, tminx[1] = bitfieldExtract(swizzledLox, 8, 8) * adjusted_idirx + origx, tminx[2] = bitfieldExtract(swizzledLox, 16, 8) * adjusted_idirx + origx;
									tminx[3] = bitfieldExtract(swizzledLox, 24, 8) * adjusted_idirx + origx, tminy[0] = bitfieldExtract(swizzledLoy, 0, 8) * adjusted_idiry + origy, tminy[1] = bitfieldExtract(swizzledLoy, 8, 8) * adjusted_idiry + origy;
									tminy[2] = bitfieldExtract(swizzledLoy, 16, 8) * adjusted_idiry + origy, tminy[3] = bitfieldExtract(swizzledLoy, 24, 8) * adjusted_idiry + origy, tminz[0] = bitfieldExtract(swizzledLoz, 0, 8) * adjusted_idirz + origz;
									tminz[1] = bitfieldExtract(swizzledLoz, 8, 8) * adjusted_idirz + origz, tminz[2] = bitfieldExtract(swizzledLoz, 16, 8) * adjusted_idirz + origz, tminz[3] = bitfieldExtract(swizzledLoz, 24, 8) * adjusted_idirz + origz;
									tmaxx[0] = bitfieldExtract(swizzledHix, 0, 8) * adjusted_idirx + origx, tmaxx[1] = bitfieldExtract(swizzledHix, 8, 8) * adjusted_idirx + origx, tmaxx[2] = bitfieldExtract(swizzledHix, 16, 8) * adjusted_idirx + origx;
									tmaxx[3] = bitfieldExtract(swizzledHix, 24, 8) * adjusted_idirx + origx, tmaxy[0] = bitfieldExtract(swizzledHiy, 0, 8) * adjusted_idiry + origy, tmaxy[1] = bitfieldExtract(swizzledHiy, 8, 8) * adjusted_idiry + origy;
									tmaxy[2] = bitfieldExtract(swizzledHiy, 16, 8) * adjusted_idiry + origy, tmaxy[3] = bitfieldExtract(swizzledHiy, 24, 8) * adjusted_idiry + origy, tmaxz[0] = bitfieldExtract(swizzledHiz, 0, 8) * adjusted_idirz + origz;
									tmaxz[1] = bitfieldExtract(swizzledHiz, 8, 8) * adjusted_idirz + origz, tmaxz[2] = bitfieldExtract(swizzledHiz, 16, 8) * adjusted_idirz + origz, tmaxz[3] = bitfieldExtract(swizzledHiz, 24, 8) * adjusted_idirz + origz;
									for (int i = 0; i < 4; i++)
									{
										// Use VMIN, VMAX to compute the slabs
										float cmin = max(max(max(tminx[i], tminy[i]), tminz[i]), tmin);
										float cmax = min(min(min(tmaxx[i], tmaxy[i]), tmaxz[i]), tmax);
										if (cmin <= cmax) hitmask |= bitfieldExtract(child_bits4, i * 8, 8) << bitfieldExtract(bit_index4, i * 8, 8);
									}
								}
								{    // Second 4
									uint meta4 = n1.w, is_inner4 = (meta4 & (meta4 << 1)) & 0x10101010;
									uint inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
									uint bit_index4 = (meta4 ^ (octinv & inner_mask4)) & 0x1F1F1F1F;
									uint child_bits4 = (meta4 >> 5) & 0x07070707;
									uint swizzledLox = (invRayD.x < 0.0) ? n3.w : n2.y, swizzledHix = (invRayD.x < 0.0) ? n2.y : n3.w;
									uint swizzledLoy = (invRayD.y < 0.0) ? n4.y : n2.w, swizzledHiy = (invRayD.y < 0.0) ? n2.w : n4.y;
									uint swizzledLoz = (invRayD.z < 0.0) ? n4.w : n3.y, swizzledHiz = (invRayD.z < 0.0) ? n3.y : n4.w;
									float tminx[4], tminy[4], tminz[4], tmaxx[4], tmaxy[4], tmaxz[4];
									tminx[0] = bitfieldExtract(swizzledLox, 0, 8) * adjusted_idirx + origx, tminx[1] = bitfieldExtract(swizzledLox, 8, 8) * adjusted_idirx + origx, tminx[2] = bitfieldExtract(swizzledLox, 16, 8) * adjusted_idirx + origx;
									tminx[3] = bitfieldExtract(swizzledLox, 24, 8) * adjusted_idirx + origx, tminy[0] = bitfieldExtract(swizzledLoy, 0, 8) * adjusted_idiry + origy, tminy[1] = bitfieldExtract(swizzledLoy, 8, 8) * adjusted_idiry + origy;
									tminy[2] = bitfieldExtract(swizzledLoy, 16, 8) * adjusted_idiry + origy, tminy[3] = bitfieldExtract(swizzledLoy, 24, 8) * adjusted_idiry + origy, tminz[0] = bitfieldExtract(swizzledLoz, 0, 8) * adjusted_idirz + origz;
									tminz[1] = bitfieldExtract(swizzledLoz, 8, 8) * adjusted_idirz + origz, tminz[2] = bitfieldExtract(swizzledLoz, 16, 8) * adjusted_idirz + origz, tminz[3] = bitfieldExtract(swizzledLoz, 24, 8) * adjusted_idirz + origz;
									tmaxx[0] = bitfieldExtract(swizzledHix, 0, 8) * adjusted_idirx + origx, tmaxx[1] = bitfieldExtract(swizzledHix, 8, 8) * adjusted_idirx + origx, tmaxx[2] = bitfieldExtract(swizzledHix, 16, 8) * adjusted_idirx + origx;
									tmaxx[3] = bitfieldExtract(swizzledHix, 24, 8) * adjusted_idirx + origx, tmaxy[0] = bitfieldExtract(swizzledHiy, 0, 8) * adjusted_idiry + origy, tmaxy[1] = bitfieldExtract(swizzledHiy, 8, 8) * adjusted_idiry + origy;
									tmaxy[2] = bitfieldExtract(swizzledHiy, 16, 8) * adjusted_idiry + origy, tmaxy[3] = bitfieldExtract(swizzledHiy, 24, 8) * adjusted_idiry + origy, tmaxz[0] = bitfieldExtract(swizzledHiz, 0, 8) * adjusted_idirz + origz;
									tmaxz[1] = bitfieldExtract(swizzledHiz, 8, 8) * adjusted_idirz + origz, tmaxz[2] = bitfieldExtract(swizzledHiz, 16, 8) * adjusted_idirz + origz, tmaxz[3] = bitfieldExtract(swizzledHiz, 24, 8) * adjusted_idirz + origz;
									for (int i = 0; i < 4; i++)
									{
										float cmin = max(max(max(tminx[i], tminy[i]), tminz[i]), tmin);
										float cmax = min(min(min(tmaxx[i], tmaxy[i]), tmaxz[i]), tmax);
										if (cmin <= cmax) hitmask |= bitfieldExtract(child_bits4, i * 8, 8) << bitfieldExtract(bit_index4, i * 8, 8);
									}
								}
								ngroup.y = (hitmask & 0xFF000000) | bitfieldExtract(n0.w, 24, 8);
								tgroup.y = hitmask & 0x00FFFFFF;
							}
						}
						else
						{
							tgroup = ngroup;
							ngroup = uvec2(0);
						}
						while (tgroup.y != 0)
						{
							uint primIndex = findMSB(tgroup.y);
							uint primAddr = tgroup.x + primIndex;
							if (IntersectLeaf(p1, p2, body, primAddr))
							{
								closestHitPrim = primAddr;
								tmax = (p2 - p1).length() / originalRayLen;
								retVal = true;
							}
							tgroup.y -= (1 << primIndex);
						}
						if (ngroup.y <= 0x00FFFFFF)
						{
							if (stackPtr > 0)
							{
								ngroup = traversalStack[--stackPtr];
							}
							else
							{
								break;
							}
						}
					} while (true);

					return retVal;
				}
				void Build(std::vector<BoxLeaf>& sdfLeaves);
				void FindBVH8Overlaps(unsigned int nodeId, std::vector<BoxLeaf>& boxLeaves, vec3& boxMin, vec3& boxMax, std::vector<unsigned int>& bvh8overlaps);
			};
		}
	}
}