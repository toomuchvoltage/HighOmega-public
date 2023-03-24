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

#ifndef HIGHOMEGA_GEOM_H
#define HIGHOMEGA_GEOM_H

#include "vmath.h"
#include <vector>

using namespace HIGHOMEGA::MATH;

namespace HIGHOMEGA
{
	namespace GEOM
	{
		enum TRITRI_INTERSECT_MODE
		{
			OVERLAP,
			T1_INSIDE_T2,
			T2_INSIDE_T1
		};
		bool TriTri(vec3 t1[3], vec3 & n1, vec3 t2[3], vec3 & n2, vec3 *out1, vec3 *out2, TRITRI_INTERSECT_MODE *mode);
		struct PointDirIntersection
		{
		public:
			vec3 pt;
			vec3 dir;
			PointDirIntersection(vec3 pt,vec3 dir);
		};
		bool TriSphere(vec3 t[3], vec3 n, bool doubleSided, vec3 c, float r,std::vector <PointDirIntersection> & intersectList);
		bool LineInsideOrHittingSphere(vec3 e1, vec3 e2, vec3 cent, float rad, vec3 *out1, vec3 *out2);
		bool LineHittingSphereOnce(vec3 e1, vec3 e2, vec3 cent, float rad, vec3 *out);
		bool PushOutSphere(vec3 t[3], vec3 s[3], vec3 n, vec3 *c, float r);
		bool PointInTri(vec3 & p, vec3 & p1, vec3 & p2, vec3 & s1, vec3 & s2, vec3 & s3);
		bool LineSegTri(vec3 & l1, vec3 & l2, vec3 & p1, vec3 & p2, vec3 & p3, vec3 & n, vec3 & s1, vec3 & s2, vec3 & s3, float & k);
		bool LineSegTri(vec3 & l1, vec3 & l2, vec3 & p1, vec3 & p2, vec3 & p3, float & k);
		void BarycentricCoords(vec3 p, vec3 a, vec3 b, vec3 c, float *u, float *v, float *w);
		struct TessTriangle
		{
			vec3 e1, e2, e3;
			vec2 uv1, uv2, uv3;
		};
		std::vector <TessTriangle> TessellateTriangle(TessTriangle & inpTri, float maxAllowedSideSizeSq);
		struct TriUV
		{
			vec3 eArr[3];
			vec3 colArr[3];
			vec2 uvArr[3];
			vec3 normVec;
		};
		void MakeBox(vec3 minPt, vec3 maxPt, std::vector<TriUV> & cullGeom);
		void MakeCylinder(vec3 cent, vec3 norm, float rad, float height, std::vector<TriUV> & cullGeom);
		bool LineLine(vec3 p1, vec3 m1, vec3 p2, vec3 m2, vec3 & outp);
		void MakeRandomEncompassingTri(vec3 aabbMin, vec3 aabbMax, TriUV & cullGeom);
		bool SplitTri(TriUV & t1, TriUV & t2, std::vector<TriUV> & splitTris, bool splitT1);
		void ComputeSideNorms(std::vector<TriUV> & allTris, std::vector<vec3> & sideNorms);
		bool InsideTest(vec3 pt, std::vector<TriUV> & allTris, std::vector<vec3> & sideNorms);
		bool Intersect(std::vector<TriUV> & slicer, std::vector<TriUV> & slicee, std::vector<TriUV> & inside, std::vector<TriUV> & outside);
		void Slice(TriUV & slicer, std::vector<TriUV> & slicee, std::vector<TriUV> & above, std::vector<TriUV> & below);
		void AddTri(vec3 e[3], vec2 uv[3], vec3 & normVec, std::vector <TriUV> & triList);
		vec3 Recenter(std::vector <TriUV> & triList);
		void GenerateAABB(std::vector <TriUV> & triList, std::vector <TriUV> & aabbTris, float epsilon = 0.0f);
		inline bool ManhattanDistClose(TriUV & t1, TriUV & t2);
		void FindConnectedNeighbors(std::vector<TriUV>& triListSrc, std::vector <TriUV> & triListDst);
		struct Line
		{
		public:
			vec3 p1, p2;
			Line(vec3 p1, vec3 p2);
		};
		void FindEdges(std::vector<TriUV>& triList, std::vector<Line>& edges);
	}
}

#endif