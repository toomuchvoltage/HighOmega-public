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

#include <geom.h>

bool HIGHOMEGA::GEOM::TriTri(vec3 t1[3], vec3 & n1, vec3 t2[3], vec3 & n2, vec3 *out1, vec3 *out2, TRITRI_INTERSECT_MODE *mode)
{
	float D1 = -(n1*t1[0]);
	float D2 = -(n2*t2[0]);
	float d_v1[3];
	d_v1[0] = (n2*t1[0]) + D2;
	d_v1[1] = (n2*t1[1]) + D2;
	d_v1[2] = (n2*t1[2]) + D2;
	float sgn_d_v10 = sign(d_v1[0]);
	float sgn_d_v11 = sign(d_v1[1]);
	float sgn_d_v12 = sign(d_v1[2]);
	if (sgn_d_v10 == sgn_d_v11 && sgn_d_v11 == sgn_d_v12)
		return false;
	float d_v2[3];
	d_v2[0] = (n1*t2[0]) + D1;
	d_v2[1] = (n1*t2[1]) + D1;
	d_v2[2] = (n1*t2[2]) + D1;
	float sgn_d_v20 = sign(d_v2[0]);
	float sgn_d_v21 = sign(d_v2[1]);
	float sgn_d_v22 = sign(d_v2[2]);
	if (sgn_d_v20 == sgn_d_v21 && sgn_d_v21 == sgn_d_v22)
		return false;
	int order1_0, order1_1, order1_2;
	int order2_0, order2_1, order2_2;
	if (sgn_d_v10 == sgn_d_v11)
	{
		order1_0 = 2;
		order1_1 = 1;
		order1_2 = 0;
	}
	else
		if (sgn_d_v11 == sgn_d_v12)
		{
			order1_0 = 0;
			order1_1 = 1;
			order1_2 = 2;
		}
		else
		{
			order1_0 = 1;
			order1_1 = 2;
			order1_2 = 0;
		}
	if (sgn_d_v20 == sgn_d_v21)
	{
		order2_0 = 2;
		order2_1 = 1;
		order2_2 = 0;
	}
	else
		if (sgn_d_v21 == sgn_d_v22)
		{
			order2_0 = 0;
			order2_1 = 1;
			order2_2 = 2;
		}
		else
		{
			order2_0 = 1;
			order2_1 = 2;
			order2_2 = 0;
		}
	vec3 D = cross(n1, n2).normalized();
	float Dx = abs(D.x);
	float Dy = abs(D.y);
	float Dz = abs(D.z);
	float max_Dxyz = Max(Dx, Dy, Dz);
	float base;
	vec3 O;
	if (max_Dxyz == Dx)
	{
		base = n1.y*n2.z - n2.y*n1.z;
		O.x = 0;
		O.y = (D2*n1.z - D1*n2.z) / base;
		O.z = (D1*n2.y - D2*n1.y) / base;
	}
	else
		if (max_Dxyz == Dy)
		{
			base = n1.x*n2.z - n2.x*n1.z;
			O.x = (D2*n1.z - D1*n2.z) / base;
			O.y = 0;
			O.z = (D1*n2.x - D2*n1.x) / base;
		}
		else
		{
			base = n1.x*n2.y - n2.x*n1.y;
			O.x = (D2*n1.y - D1*n2.y) / base;
			O.y = (D1*n2.x - D2*n1.x) / base;
			O.z = 0;
		}
	float p_v10 = D*(t1[order1_0] - O);
	float p_v11 = D*(t1[order1_1] - O);
	float p_v12 = D*(t1[order1_2] - O);
	float p_v20 = D*(t2[order2_0] - O);
	float p_v21 = D*(t2[order2_1] - O);
	float p_v22 = D*(t2[order2_2] - O);
	float t_1[2], t_2[2];
	t_1[0] = p_v11 + (p_v10 - p_v11)*(d_v1[order1_1] / (d_v1[order1_1] - d_v1[order1_0]));
	t_1[1] = p_v12 + (p_v10 - p_v12)*(d_v1[order1_2] / (d_v1[order1_2] - d_v1[order1_0]));
	t_2[0] = p_v21 + (p_v20 - p_v21)*(d_v2[order2_1] / (d_v2[order2_1] - d_v2[order2_0]));
	t_2[1] = p_v22 + (p_v20 - p_v22)*(d_v2[order2_2] / (d_v2[order2_2] - d_v2[order2_0]));
	if (t_1[0] > t_1[1])
	{
		float temp = t_1[0];
		t_1[0] = t_1[1];
		t_1[1] = temp;
	}
	if (t_2[0] > t_2[1])
	{
		float temp = t_2[0];
		t_2[0] = t_2[1];
		t_2[1] = temp;
	}
	if (t_1[1] < t_2[0] || t_1[0] > t_2[1])
		return false;
	if (t_1[1] < t_2[1] && t_1[0] < t_2[0])
	{
		*out1 = O + t_1[1] * D;
		*out2 = O + t_2[0] * D;
		*mode = OVERLAP;
	}
	else
		if (t_1[0] > t_2[0] && t_1[1] > t_2[1])
		{
			*out1 = O + t_1[0] * D;
			*out2 = O + t_2[1] * D;
			*mode = OVERLAP;
		}
		else
			if (t_1[0] > t_2[0] && t_1[1] < t_2[1])
			{
				*out1 = O + t_1[0] * D;
				*out2 = O + t_1[1] * D;
				*mode = T1_INSIDE_T2;
			}
			else
			{
				*out1 = O + t_2[0] * D;
				*out2 = O + t_2[1] * D;
				*mode = T2_INSIDE_T1;
			}
	return true;
}

HIGHOMEGA::GEOM::PointDirIntersection::PointDirIntersection(vec3 pt, vec3 dir)
{
	this->pt = pt;
	this->dir = dir;
}

bool HIGHOMEGA::GEOM::TriSphere(vec3 tri[3], vec3 norm, bool doubleSided, vec3 cent, float rad, std::vector<PointDirIntersection>& intersectList)
{
	vec3 nn = norm;

	float distToPlane = nn*(cent - tri[0]);

	if (!doubleSided && distToPlane < 0.0f) return false;
	if (doubleSided && distToPlane < 0.0f)
	{
		nn = -nn;
		distToPlane = -distToPlane;
	}

	if (distToPlane > rad) return false;

	vec3 m[3];
	m[0] = tri[1] - tri[0];
	m[1] = tri[2] - tri[1];
	m[2] = tri[0] - tri[2];
	vec3 t_c[3];
	t_c[0] = tri[0] - cent;
	t_c[1] = tri[1] - cent;
	t_c[2] = tri[2] - cent;

	float radSq = rad*rad;

	bool intersectionHappened = false;

	for (int i = 0; i != 3; i++)
	{
		float a = m[i] * m[i];
		float b = 2.0f * t_c[i] * m[i];
		float c = (t_c[i] * t_c[i]) - radSq;

		if (c < 0.0f)
		{
			intersectList.emplace_back(tri[i],nn);
			intersectionHappened = true;
		}

		float qSq = (b*b) - (4.0f*a*c);
		if (qSq < 0.0f) continue;

		float q = sqrtf(qSq);
		float denom = 1.0f / (2.0f*a);

		float k1 = (-b + q) * denom;
		float k2 = (-b - q) * denom;

		if (k1 >= 0.0f && k1 <= 1.0f)
		{
			intersectList.emplace_back(tri[i] + m[i] * k1,nn);
			intersectionHappened = true;
		}
		if (k2 >= 0.0f && k2 <= 1.0f)
		{
			intersectList.emplace_back(tri[i] + m[i] * k2,nn);
			intersectionHappened = true;
		}
	}

	return intersectionHappened;
}

bool HIGHOMEGA::GEOM::LineInsideOrHittingSphere(vec3 e1, vec3 e2, vec3 cent, float rad, vec3 * out1, vec3 * out2)
{
	vec3 m = e2 - e1;
	vec3 e1_c = e1 - cent;
	vec3 e2_c = e2 - cent;
	float e1_c_len = e1_c.length();
	float e2_c_len = e2_c.length();

	std::vector <vec3> intList;

	if (e1_c_len < rad) intList.push_back(e1);
	if (e2_c_len < rad) intList.push_back(e2);

	if (intList.size() == 2)
	{
		*out1 = intList[0];
		*out2 = intList[1];
		return true;
	}

	float radSq = rad*rad;

	float a = m * m;
	float b = 2.0f * e1_c * m;
	float c = (e1_c * e1_c) - radSq;

	float qSq = (b*b) - (4.0f*a*c);
	if (qSq < 0.0f) return false;

	float q = sqrtf(qSq);
	float denom = 1.0f / (2.0f*a);

	float k1 = (-b + q) * denom;
	float k2 = (-b - q) * denom;

	if (k1 >= 0.0f && k1 <= 1.0f) intList.push_back (e1 + m * k1);
	if (k2 >= 0.0f && k2 <= 1.0f) intList.push_back (e1 + m * k2);

	if (intList.size() == 2)
	{
		*out1 = intList[0];
		*out2 = intList[1];
		return true;
	}
	else
		return false;
}

bool HIGHOMEGA::GEOM::LineHittingSphereOnce(vec3 e1, vec3 e2, vec3 cent, float rad, vec3 * out)
{
	vec3 m = e2 - e1;
	vec3 e1_c = e1 - cent;

	std::vector <vec3> intList;

	float radSq = rad * rad;

	float a = m * m;
	float b = 2.0f * e1_c * m;
	float c = (e1_c * e1_c) - radSq;

	float qSq = (b*b) - (4.0f*a*c);
	if (qSq < 0.0f) return false;

	float q = sqrtf(qSq);
	float denom = 1.0f / (2.0f*a);

	float k1 = (-b + q) * denom;
	float k2 = (-b - q) * denom;

	if (k1 >= 0.0f && k1 <= 1.0f)
	{
		*out = e1 + m * k1;
		return true;
	}
	else if (k2 >= 0.0f && k2 <= 1.0f)
	{
		*out = e1 + m * k2;
		return true;
	}
	else
		return false;
}

bool HIGHOMEGA::GEOM::PushOutSphere(vec3 t[3], vec3 s[3], vec3 n, vec3 *c, float r)
{
	float distToPlane = (*c - t[0])*n;
	if (distToPlane < 0.0f || distToPlane > r) return false;

	vec3 t_c[3];
	t_c[0] = t[0] - *c;
	t_c[1] = t[1] - *c;
	t_c[2] = t[2] - *c;

	float distToSide1 = -t_c[0] * s[0];
	float distToSide2 = -t_c[1] * s[1];
	float distToSide3 = -t_c[2] * s[2];

	float t0DistToC = t_c[0].length();
	float t1DistToC = t_c[1].length();
	float t2DistToC = t_c[2].length();

	int behindSidePlaneCount = 0;
	if (distToSide1 < 0.0f) behindSidePlaneCount++;
	if (distToSide2 < 0.0f) behindSidePlaneCount++;
	if (distToSide3 < 0.0f) behindSidePlaneCount++;

	if (behindSidePlaneCount == 3 || (t0DistToC < r && t1DistToC < r && t2DistToC < r)) {
		*c += (r - distToPlane)*n;
		return true;
	}

	if (behindSidePlaneCount == 1) {
		if (t0DistToC < r) {
			*c += (-t_c[0]).normalized()*(r - t0DistToC);
			return true;
		}
		if (t1DistToC < r) {
			*c += (-t_c[1]).normalized()*(r - t1DistToC);
			return true;
		}
		if (t2DistToC < r) {
			*c += (-t_c[2]).normalized()*(r - t2DistToC);
			return true;
		}
	}

	if (behindSidePlaneCount == 2) {
		vec3 tmp1, tmp2;
		if (distToSide1 > 0.0) {
			if (LineInsideOrHittingSphere(t[0], t[1], *c, r, &tmp1, &tmp2))
			{
				vec3 m = (t[1] - t[0]).normalized();
				vec3 ptOnLine = t[0] + (-t_c[0])*m*m;
				vec3 backToC = (ptOnLine - *c);
				float lenToC = r - backToC.length();
				if (lenToC > 0.0f)
				{
					*c += (-backToC).normalized()*lenToC;
					return true;
				}
			}
		}
		if (distToSide2 > 0.0) {
			if (LineInsideOrHittingSphere(t[1], t[2], *c, r, &tmp1, &tmp2))
			{
				vec3 m = (t[2] - t[1]).normalized();
				vec3 ptOnLine = t[1] + (-t_c[1])*m*m;
				vec3 backToC = (ptOnLine - *c);
				float lenToC = r - backToC.length();
				if (lenToC > 0.0f)
				{
					*c += (-backToC).normalized()*lenToC;
					return true;
				}
			}
		}
		if (distToSide3 > 0.0) {
			if (LineInsideOrHittingSphere(t[2], t[0], *c, r, &tmp1, &tmp2))
			{
				vec3 m = (t[0] - t[2]).normalized();
				vec3 ptOnLine = t[2] + (-t_c[2])*m*m;
				vec3 backToC = (ptOnLine - *c);
				float lenToC = r - backToC.length();
				if (lenToC > 0.0f)
				{
					*c += (-backToC).normalized()*lenToC;
					return true;
				}
			}
		}
	}

	return false;
}

bool HIGHOMEGA::GEOM::PointInTri(vec3 & p, vec3 & p2, vec3 & p3, vec3 & s1, vec3 & s2, vec3 & s3)
{
	vec3 p_p2 = p - p2;
	vec3 p_p3 = p - p3;

	return (s1*p_p2 < 0.0f && s2*p_p2 < 0.0f && s3*p_p3 < 0.0f);
}

bool HIGHOMEGA::GEOM::LineSegTri(vec3 & l1, vec3 & l2, vec3 & p1, vec3 & p2, vec3 & p3, vec3 & n, vec3 & s1, vec3 & s2, vec3 & s3, float & k)
{
	vec3 m = l2 - l1;
	float mdotn = m * n;
	if (abs(mdotn) == 0.0f) return false;
	float D = -(p1*n);
	k = -(D + (n*l1)) / (mdotn);
	if (k < 0.0f || k > 1.0f) return false;
	vec3 p = l1 + k*m;

	return PointInTri(p, p2, p3, s1, s2, s3);
}

// Moller-Trumbore
bool HIGHOMEGA::GEOM::LineSegTri(vec3 & orig, vec3 & dir, vec3 & p1, vec3 & p2, vec3 & p3, float & k)
{
	vec3 e1, e2;
	vec3 P, Q, T;
	float det, inv_det, u, v;
	float t;
	e1 = p2 - p1;
	e2 = p3 - p1;
	P = cross(dir, e2);
	det = e1 * P;
	if (det == 0.0f) return false;
	inv_det = 1.0f / det;
	T = orig - p1;
	u = (T * P) * inv_det;
	if (u < 0.0f || u > 1.0f) return false;
	Q = cross(T, e1);
	v = (dir * Q) * inv_det;
	if (v < 0.0f || (u + v) > 1.0f) return false;
	t = (e2 * Q) * inv_det;
	if (t > 0.0f && t < k)
	{
		k = t;
		return true;
	}
	return false;
}

void HIGHOMEGA::GEOM::BarycentricCoords(vec3 p, vec3 a, vec3 b, vec3 c, float *u, float *v, float *w)
{
	vec3 v0 = b - a, v1 = c - a, v2 = p - a;
	float d00 = v0 * v0;
	float d01 = v0 * v1;
	float d11 = v1 * v1;
	float d20 = v2 * v0;
	float d21 = v2 * v1;
	float invDenom = 1.0f / (d00 * d11 - d01 * d01);
	*v = (d11 * d20 - d01 * d21) * invDenom;
	*w = (d00 * d21 - d01 * d20) * invDenom;
	*u = 1.0f - *v - *w;
}

std::vector<HIGHOMEGA::GEOM::TessTriangle> HIGHOMEGA::GEOM::TessellateTriangle(TessTriangle & inpTri, float maxAllowedSideSizeSq)
{
	std::vector<HIGHOMEGA::GEOM::TessTriangle> retVal;
	vec3 diff1 = inpTri.e2 - inpTri.e1;
	vec3 diff2 = inpTri.e3 - inpTri.e2;
	vec3 diff3 = inpTri.e1 - inpTri.e3;
	float len1 = diff1 * diff1;
	float len2 = diff2 * diff2;
	float len3 = diff3 * diff3;
	float curMaxSide = Max(len1, len2, len3);
	if (curMaxSide < maxAllowedSideSizeSq)
	{
		retVal.push_back(inpTri);
		return retVal;
	}
	float breakPoint1 = (rand() % 1000 - 500) * 0.0001f + 0.5f;
	float breakPoint2 = (rand() % 1000 - 500) * 0.0001f + 0.5f;
	float breakPoint3 = (rand() % 1000 - 500) * 0.0001f + 0.5f;
	vec3 mid1 = inpTri.e1 + diff1 * breakPoint1;
	vec3 mid2 = inpTri.e2 + diff2 * breakPoint2;
	vec3 mid3 = inpTri.e3 + diff3 * breakPoint3;
	vec2 mid1uv = Lerp(inpTri.uv1, inpTri.uv2, breakPoint1);
	vec2 mid2uv = Lerp(inpTri.uv2, inpTri.uv3, breakPoint2);
	vec2 mid3uv = Lerp(inpTri.uv3, inpTri.uv1, breakPoint3);

	TessTriangle curSubTri;
	for (int i = 0; i != 4; i++)
	{
		switch (i)
		{
		case 0:
			curSubTri.e1 = mid1;
			curSubTri.e2 = mid2;
			curSubTri.e3 = mid3;
			curSubTri.uv1 = mid1uv;
			curSubTri.uv2 = mid2uv;
			curSubTri.uv3 = mid3uv;
			break;
		case 1:
			curSubTri.e1 = inpTri.e1;
			curSubTri.e2 = mid1;
			curSubTri.e3 = mid3;
			curSubTri.uv1 = inpTri.uv1;
			curSubTri.uv2 = mid1uv;
			curSubTri.uv3 = mid3uv;
			break;
		case 2:
			curSubTri.e1 = inpTri.e3;
			curSubTri.e2 = mid2;
			curSubTri.e3 = mid3;
			curSubTri.uv1 = inpTri.uv3;
			curSubTri.uv2 = mid2uv;
			curSubTri.uv3 = mid3uv;
			break;
		case 3:
			curSubTri.e1 = inpTri.e2;
			curSubTri.e2 = mid1;
			curSubTri.e3 = mid2;
			curSubTri.uv1 = inpTri.uv2;
			curSubTri.uv2 = mid1uv;
			curSubTri.uv3 = mid2uv;
			break;
		}
		std::vector<HIGHOMEGA::GEOM::TessTriangle> retTris = TessellateTriangle(curSubTri, maxAllowedSideSizeSq);
		retVal.insert(retVal.end(), retTris.begin(), retTris.end());
	}
	return retVal;
}

void HIGHOMEGA::GEOM::MakeBox(vec3 minPt, vec3 maxPt, std::vector<TriUV>& cullGeom)
{
	TriUV newT;
	newT.normVec = vec3(0.0f, 1.0f, 0.0f);

	newT.eArr[0] = maxPt;
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, maxPt.y, minPt.z);
	newT.uvArr[1] = vec2(1.0f, 1.0f);
	newT.eArr[2] = vec3(minPt.x, maxPt.y, minPt.z);
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.eArr[0] = vec3(minPt.x, maxPt.y, maxPt.z);
	newT.uvArr[0] = vec2(0.0f, 0.0f);
	newT.eArr[1] = maxPt;
	newT.uvArr[1] = vec2(1.0f, 0.0f);
	newT.eArr[2] = vec3(minPt.x, maxPt.y, minPt.z);
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.normVec = vec3(0.0f, -1.0f, 0.0f);

	newT.eArr[0] = vec3(maxPt.x, minPt.y, maxPt.z);
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, minPt.y, minPt.z);
	newT.uvArr[1] = vec2(1.0f, 1.0f);
	newT.eArr[2] = minPt;
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.eArr[0] = vec3(minPt.x, minPt.y, maxPt.z);
	newT.uvArr[0] = vec2(0.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, minPt.y, maxPt.z);
	newT.uvArr[1] = vec2(1.0f, 0.0f);
	newT.eArr[2] = minPt;
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

//----------------------------------------

	newT.normVec = vec3(-1.0f, 0.0f, 0.0f);

	newT.eArr[0] = vec3(minPt.x, maxPt.y, maxPt.z);
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(minPt.x, maxPt.y, minPt.z);
	newT.uvArr[1] = vec2(0.0f, 0.0f);
	newT.eArr[2] = minPt;
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.eArr[0] = vec3(minPt.x, maxPt.y, maxPt.z);
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(minPt.x, minPt.y, maxPt.z);
	newT.uvArr[1] = vec2(1.0f, 1.0f);
	newT.eArr[2] = minPt;
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.normVec = vec3(1.0f, 0.0f, 0.0f);

	newT.eArr[0] = maxPt;
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, maxPt.y, minPt.z);
	newT.uvArr[1] = vec2(0.0f, 0.0f);
	newT.eArr[2] = vec3(maxPt.x, minPt.y, minPt.z);
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.eArr[0] = maxPt;
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, minPt.y, maxPt.z);
	newT.uvArr[1] = vec2(1.0f, 1.0f);
	newT.eArr[2] = vec3(maxPt.x, minPt.y, minPt.z);
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	//----------------------------------------

	newT.normVec = vec3(0.0f, 0.0f, 1.0f);

	newT.eArr[0] = vec3(minPt.x, maxPt.y, maxPt.z);
	newT.uvArr[0] = vec2(0.0f, 0.0f);
	newT.eArr[1] = maxPt;
	newT.uvArr[1] = vec2(1.0f, 0.0f);
	newT.eArr[2] = vec3(minPt.x, minPt.y, maxPt.z);
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.eArr[0] = maxPt;
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, minPt.y, maxPt.z);
	newT.uvArr[1] = vec2(1.0f, 1.0f);
	newT.eArr[2] = vec3(minPt.x, minPt.y, maxPt.z);
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.normVec = vec3(0.0f, 0.0f, -1.0f);

	newT.eArr[0] = vec3(minPt.x, maxPt.y, minPt.z);
	newT.uvArr[0] = vec2(0.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, maxPt.y, minPt.z);
	newT.uvArr[1] = vec2(1.0f, 0.0f);
	newT.eArr[2] = minPt;
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);

	newT.eArr[0] = vec3(maxPt.x, maxPt.y, minPt.z);
	newT.uvArr[0] = vec2(1.0f, 0.0f);
	newT.eArr[1] = vec3(maxPt.x, minPt.y, minPt.z);
	newT.uvArr[1] = vec2(1.0f, 1.0f);
	newT.eArr[2] = minPt;
	newT.uvArr[2] = vec2(0.0f, 1.0f);
	cullGeom.push_back(newT);
}

void HIGHOMEGA::GEOM::MakeCylinder(vec3 cent, vec3 norm, float rad, float height, std::vector<TriUV>& cullGeom)
{
	std::vector<vec3> allOnPlaneNorms;
	vec3 X = cross(norm, norm + vec3 (0.1f)).normalized();
	float angleChunk = HIGHOMEGA_PI * 0.4f;
	float angleVar = HIGHOMEGA_PI * 0.16666667f;

	for (int i = 0; i != 5; i++)
	{
		float spinAmt = (rand() % 1000 - 500) * 0.002f * angleVar;
		X = Spin(norm, X, spinAmt);
		allOnPlaneNorms.push_back(X);

		X = Spin(norm, X, angleChunk);
	}

	vec3 Y = cross(norm, X);

	for (int i = 0; i != allOnPlaneNorms.size(); i++)
	{
		int nextIndex = i + 1;
		if (nextIndex == allOnPlaneNorms.size()) nextIndex = 0;
		vec3 curTan = (allOnPlaneNorms[nextIndex] - allOnPlaneNorms[i]).normalized();

		TriUV curFace;
		curFace.normVec = cross (curTan, norm);
		if (allOnPlaneNorms[i] * curFace.normVec < 0.0f) curFace.normVec = -curFace.normVec;

		curFace.eArr[0] = cent + allOnPlaneNorms[i] * rad - norm * height;
		curFace.eArr[1] = cent + allOnPlaneNorms[nextIndex] * rad - norm * height;
		curFace.eArr[2] = cent + allOnPlaneNorms[i] * rad + norm * height;
		curFace.uvArr[0] = vec2(0.0f);
		curFace.uvArr[1] = vec2(1.0f, 0.0f);
		curFace.uvArr[2] = vec2(0.0f, 1.0f);
		cullGeom.push_back(curFace);

		curFace.eArr[0] = cent + allOnPlaneNorms[nextIndex] * rad - norm * height;
		curFace.eArr[1] = cent + allOnPlaneNorms[nextIndex] * rad + norm * height;
		curFace.eArr[2] = cent + allOnPlaneNorms[i] * rad + norm * height;
		curFace.uvArr[0] = vec2(1.0f, 0.0f);
		curFace.uvArr[1] = vec2(1.0f, 1.0f);
		curFace.uvArr[2] = vec2(0.0f, 1.0f);
		cullGeom.push_back(curFace);

		curFace.normVec = norm;

		curFace.eArr[0] = cent + norm * height;
		curFace.eArr[1] = cent + allOnPlaneNorms[i] * rad + norm * height;
		curFace.eArr[2] = cent + allOnPlaneNorms[nextIndex] * rad + norm * height;
		vec3 diff1 = curFace.eArr[1] - curFace.eArr[0];
		vec3 diff2 = curFace.eArr[2] - curFace.eArr[0];
		curFace.uvArr[0] = vec2(0.0f, 0.0f);
		curFace.uvArr[1] = vec2(diff1 * X, diff1 * Y);
		curFace.uvArr[2] = vec2(diff2 * X, diff2 * Y);
		cullGeom.push_back(curFace);

		curFace.normVec = -norm;

		curFace.eArr[0] = cent - norm * height;
		curFace.eArr[1] = cent + allOnPlaneNorms[i] * rad - norm * height;
		curFace.eArr[2] = cent + allOnPlaneNorms[nextIndex] * rad - norm * height;
		cullGeom.push_back(curFace);
	}
}

bool HIGHOMEGA::GEOM::LineLine(vec3 p1, vec3 m1, vec3 p2, vec3 m2, vec3 & outp)
{
	float denom;
	denom = (m1.x * m2.y) - (m2.x * m1.y);
	if (denom != 0.0f)
	{
		float t2 = ((m1.y * p2.x) - (m1.y * p1.x) - (m1.x * p2.y) + (m1.x * p1.y)) / denom;
		outp = p2 + m2 * t2;
		return true;
	}
	denom = (m1.x * m2.z) - (m2.x * m1.z);
	if (denom != 0.0f)
	{
		float t2 = ((m1.z * p2.x) - (m1.z * p1.x) - (m1.x * p2.z) + (m1.x * p1.z)) / denom;
		outp = p2 + m2 * t2;
		return true;
	}
	denom = (m1.y * m2.z) - (m2.y * m1.z);
	if (denom != 0.0f)
	{
		float t2 = ((m1.z * p2.y) - (m1.z * p1.y) - (m1.y * p2.z) + (m1.y * p1.z)) / denom;
		outp = p2 + m2 * t2;
		return true;
	}
	return false;
}

#define SIDE_SCALER 10.0f

void HIGHOMEGA::GEOM::MakeRandomEncompassingTri(vec3 aabbMin, vec3 aabbMax, TriUV & cullGeom)
{
	vec3 fractInBox, ptInside, cutPlaneNorm;
	vec3 aabbCent = (aabbMin + aabbMax) * 0.5f;
	float aabbRad = (aabbMax - aabbCent).length();

	fractInBox = vec3((rand() % 100) * 0.01f, (rand() % 100) * 0.01f, (rand() % 100) * 0.01f) * 0.5f + vec3 (0.25f);
	ptInside = aabbMin + fractInBox * (aabbMax - aabbMin);
	cutPlaneNorm = vec3 ((rand() % 100 - 50) * 0.02f, (rand() % 100 - 50) * 0.02f, (rand() % 100 - 50) * 0.02f).normalized();
	vec3 toCent = ptInside - aabbCent;
	vec3 toCentNorm = toCent.normalized();

	vec3 vecOnPlane = cross (cutPlaneNorm, vec3((rand() % 100 - 50) * 0.02f, (rand() % 100 - 50) * 0.02f, (rand() % 100 - 50) * 0.02f)).normalized();
	vec3 p1 = ptInside + vecOnPlane * aabbRad * 2 * SIDE_SCALER;
	vec3 m1 = cross(p1 - ptInside, cutPlaneNorm);
	vecOnPlane = Spin(cutPlaneNorm, vecOnPlane, 0.66666666667f * HIGHOMEGA_PI);
	vec3 p2 = ptInside + vecOnPlane * aabbRad * 2 * SIDE_SCALER;
	vec3 m2 = cross(p2 - ptInside, cutPlaneNorm);
	vecOnPlane = Spin(cutPlaneNorm, vecOnPlane, 0.66666666667f * HIGHOMEGA_PI);
	vec3 p3 = ptInside + vecOnPlane * aabbRad * 2 * SIDE_SCALER;
	vec3 m3 = cross(p3 - ptInside, cutPlaneNorm);

	vec3 c1, c2, c3;
	LineLine(p1, m1, p2, m2, c1);
	LineLine(p2, m2, p3, m3, c2);
	LineLine(p1, m1, p3, m3, c3);

	cullGeom.normVec = cross(c3 - c2, c1 - c2).normalized();
	cullGeom.eArr[0] = c1;
	cullGeom.uvArr[0] = vec2(0.0f);
	cullGeom.eArr[1] = c2;
	cullGeom.uvArr[1] = vec2(50.0f, 100.0f);
	cullGeom.eArr[2] = c3;
	cullGeom.uvArr[2] = vec2(100.0f, 0.0f);
}

bool HIGHOMEGA::GEOM::SplitTri(TriUV & t1, TriUV & t2, std::vector<TriUV>& splitTris, bool splitT1)
{
	float D1 = -(t1.eArr[0] * t1.normVec);

	float t20Dist = t2.eArr[0] * t1.normVec + D1;
	float t21Dist = t2.eArr[1] * t1.normVec + D1;
	float t22Dist = t2.eArr[2] * t1.normVec + D1;
	float t20Sign = signEpsilon(t20Dist);
	float t21Sign = signEpsilon(t21Dist);
	float t22Sign = signEpsilon(t22Dist);
	if (!(t20Sign == -1.0f && t21Sign == 1.0f && t22Sign == 1.0f) &&
		!(t20Sign == 1.0f && t21Sign == -1.0f && t22Sign == 1.0f) &&
		!(t20Sign == 1.0f && t21Sign == 1.0f && t22Sign == -1.0f) &&
		!(t20Sign == 1.0f && t21Sign == -1.0f && t22Sign == -1.0f) &&
		!(t20Sign == -1.0f && t21Sign == 1.0f && t22Sign == -1.0f) &&
		!(t20Sign == -1.0f && t21Sign == -1.0f && t22Sign == 1.0f)) return false;

	float D2 = -(t2.eArr[0] * t2.normVec);

	float t10Dist = t1.eArr[0] * t2.normVec + D2;
	float t11Dist = t1.eArr[1] * t2.normVec + D2;
	float t12Dist = t1.eArr[2] * t2.normVec + D2;
	float t10Sign = signEpsilon(t10Dist);
	float t11Sign = signEpsilon(t11Dist);
	float t12Sign = signEpsilon(t12Dist);
	if (!(t10Sign == -1.0f && t11Sign == 1.0f && t12Sign == 1.0f) &&
		!(t10Sign == 1.0f && t11Sign == -1.0f && t12Sign == 1.0f) &&
		!(t10Sign == 1.0f && t11Sign == 1.0f && t12Sign == -1.0f) &&
		!(t10Sign == 1.0f && t11Sign == -1.0f && t12Sign == -1.0f) &&
		!(t10Sign == -1.0f && t11Sign == 1.0f && t12Sign == -1.0f) &&
		!(t10Sign == -1.0f && t11Sign == -1.0f && t12Sign == 1.0f)) return false;

	vec3 t1hit1, t1hit2;
	float t1hit1k, t1hit2k;
	int t1Hitp0, t1Hitp1, t1Hitp2;
	vec3 t2hit1, t2hit2;
	float t2hit1k, t2hit2k;
	int t2Hitp0, t2Hitp1, t2Hitp2;

	if (t10Sign != t11Sign && t11Sign == t12Sign)
	{
		vec3 m1 = t1.eArr[1] - t1.eArr[0];
		vec3 m2 = t1.eArr[2] - t1.eArr[0];
		t1hit1k = (-t10Dist / (m1 * t2.normVec));
		t1hit2k = (-t10Dist / (m2 * t2.normVec));
		t1hit1 = t1.eArr[0] + t1hit1k * m1;
		t1hit2 = t1.eArr[0] + t1hit2k * m2;
		t1Hitp0 = 0;
		t1Hitp1 = 1;
		t1Hitp2 = 2;
	}
	else if (t11Sign != t10Sign && t10Sign == t12Sign)
	{
		vec3 m1 = t1.eArr[0] - t1.eArr[1];
		vec3 m2 = t1.eArr[2] - t1.eArr[1];
		t1hit1k = (-t11Dist / (m1 * t2.normVec));
		t1hit2k = (-t11Dist / (m2 * t2.normVec));
		t1hit1 = t1.eArr[1] + t1hit1k * m1;
		t1hit2 = t1.eArr[1] + t1hit2k * m2;
		t1Hitp0 = 1;
		t1Hitp1 = 0;
		t1Hitp2 = 2;
	}
	else
	{
		vec3 m1 = t1.eArr[0] - t1.eArr[2];
		vec3 m2 = t1.eArr[1] - t1.eArr[2];
		t1hit1k = (-t12Dist / (m1 * t2.normVec));
		t1hit2k = (-t12Dist / (m2 * t2.normVec));
		t1hit1 = t1.eArr[2] + t1hit1k * m1;
		t1hit2 = t1.eArr[2] + t1hit2k * m2;
		t1Hitp0 = 2;
		t1Hitp1 = 0;
		t1Hitp2 = 1;
	}

	if (t20Sign != t21Sign && t21Sign == t22Sign)
	{
		vec3 m1 = t2.eArr[1] - t2.eArr[0];
		vec3 m2 = t2.eArr[2] - t2.eArr[0];
		t2hit1k = (-t20Dist / (m1 * t1.normVec));
		t2hit2k = (-t20Dist / (m2 * t1.normVec));
		t2hit1 = t2.eArr[0] + t2hit1k * m1;
		t2hit2 = t2.eArr[0] + t2hit2k * m2;
		t2Hitp0 = 0;
		t2Hitp1 = 1;
		t2Hitp2 = 2;
	}
	else if (t21Sign != t20Sign && t20Sign == t22Sign)
	{
		vec3 m1 = t2.eArr[0] - t2.eArr[1];
		vec3 m2 = t2.eArr[2] - t2.eArr[1];
		t2hit1k = (-t21Dist / (m1 * t1.normVec));
		t2hit2k = (-t21Dist / (m2 * t1.normVec));
		t2hit1 = t2.eArr[1] + t2hit1k * m1;
		t2hit2 = t2.eArr[1] + t2hit2k * m2;
		t2Hitp0 = 1;
		t2Hitp1 = 0;
		t2Hitp2 = 2;
	}
	else
	{
		vec3 m1 = t2.eArr[0] - t2.eArr[2];
		vec3 m2 = t2.eArr[1] - t2.eArr[2];
		t2hit1k = (-t22Dist / (m1 * t1.normVec));
		t2hit2k = (-t22Dist / (m2 * t1.normVec));
		t2hit1 = t2.eArr[2] + t2hit1k * m1;
		t2hit2 = t2.eArr[2] + t2hit2k * m2;
		t2Hitp0 = 2;
		t2Hitp1 = 0;
		t2Hitp2 = 1;
	}

	vec3 overlapP0 = t1hit1;
	vec3 overlapM = t1hit2 - t1hit1;
	vec3 overlapMAbs = vec3(fabsf(overlapM.x), fabsf(overlapM.y), fabsf(overlapM.z));
	float maxComp = Max(overlapMAbs.x, overlapMAbs.y, overlapMAbs.z);

	float k1, k2;

	if (maxComp == overlapMAbs.x)
	{
		k1 = (t2hit1.x - overlapP0.x) / overlapM.x;
		k2 = (t2hit2.x - overlapP0.x) / overlapM.x;
	}
	else if (maxComp == overlapMAbs.y)
	{
		k1 = (t2hit1.y - overlapP0.y) / overlapM.y;
		k2 = (t2hit2.y - overlapP0.y) / overlapM.y;
	}
	else
	{
		k1 = (t2hit1.z - overlapP0.z) / overlapM.z;
		k2 = (t2hit2.z - overlapP0.z) / overlapM.z;
	}

	if ((k1 < 0.0f && k2 < 0.0f) || (k1 > 1.0f && k2 > 1.0f)) return false;

	unsigned int prevSize = (int)splitTris.size();

	TriUV newT;
	if (splitT1)
	{
		newT = t1;

		vec2 t1hit1uv = t1.uvArr[t1Hitp0] + (t1.uvArr[t1Hitp1] - t1.uvArr[t1Hitp0]) * t1hit1k;
		vec2 t1hit2uv = t1.uvArr[t1Hitp0] + (t1.uvArr[t1Hitp2] - t1.uvArr[t1Hitp0]) * t1hit2k;

		newT.eArr[0] = t1.eArr[t1Hitp1];
		newT.eArr[1] = t1hit1;
		newT.eArr[2] = t1hit2;
		newT.uvArr[0] = t1.uvArr[t1Hitp1];
		newT.uvArr[1] = t1hit1uv;
		newT.uvArr[2] = t1hit2uv;
		AddTri(newT.eArr, newT.uvArr, newT.normVec, splitTris);

		newT.eArr[0] = t1.eArr[t1Hitp1];
		newT.eArr[1] = t1.eArr[t1Hitp2];
		newT.eArr[2] = t1hit2;
		newT.uvArr[0] = t1.uvArr[t1Hitp1];
		newT.uvArr[1] = t1.uvArr[t1Hitp2];
		newT.uvArr[2] = t1hit2uv;
		AddTri(newT.eArr, newT.uvArr, newT.normVec, splitTris);

		newT.eArr[0] = t1hit1;
		newT.eArr[1] = t1.eArr[t1Hitp0];
		newT.eArr[2] = t1hit2;
		newT.uvArr[0] = t1hit1uv;
		newT.uvArr[1] = t1.uvArr[t1Hitp0];
		newT.uvArr[2] = t1hit2uv;
		AddTri(newT.eArr, newT.uvArr, newT.normVec, splitTris);
	}
	else
	{
		newT = t2;

		vec2 t2hit1uv = t2.uvArr[t2Hitp0] + (t2.uvArr[t2Hitp1] - t2.uvArr[t2Hitp0]) * t2hit1k;
		vec2 t2hit2uv = t2.uvArr[t2Hitp0] + (t2.uvArr[t2Hitp2] - t2.uvArr[t2Hitp0]) * t2hit2k;

		newT.eArr[0] = t2.eArr[t2Hitp1];
		newT.eArr[1] = t2hit1;
		newT.eArr[2] = t2hit2;
		newT.uvArr[0] = t2.uvArr[t2Hitp1];
		newT.uvArr[1] = t2hit1uv;
		newT.uvArr[2] = t2hit2uv;
		AddTri(newT.eArr, newT.uvArr, newT.normVec, splitTris);

		newT.eArr[0] = t2.eArr[t2Hitp1];
		newT.eArr[1] = t2.eArr[t2Hitp2];
		newT.eArr[2] = t2hit2;
		newT.uvArr[0] = t2.uvArr[t2Hitp1];
		newT.uvArr[1] = t2.uvArr[t2Hitp2];
		newT.uvArr[2] = t2hit2uv;
		AddTri(newT.eArr, newT.uvArr, newT.normVec, splitTris);

		newT.eArr[0] = t2hit1;
		newT.eArr[1] = t2.eArr[t2Hitp0];
		newT.eArr[2] = t2hit2;
		newT.uvArr[0] = t2hit1uv;
		newT.uvArr[1] = t2.uvArr[t2Hitp0];
		newT.uvArr[2] = t2hit2uv;
		AddTri(newT.eArr, newT.uvArr, newT.normVec, splitTris);
	}

	if (splitTris.size() == prevSize) return false;

	return true;
}

void HIGHOMEGA::GEOM::ComputeSideNorms(std::vector<TriUV>& allTris, std::vector<vec3>& sideNorms)
{
	for (TriUV & curTri : allTris)
	{
		vec3 p1_p0 = curTri.eArr[1] - curTri.eArr[0];
		vec3 p2_p1 = curTri.eArr[2] - curTri.eArr[1];
		vec3 p0_p2 = curTri.eArr[0] - curTri.eArr[2];

		vec3 side1 = cross(p1_p0, curTri.normVec);
		vec3 side2 = cross(p2_p1, curTri.normVec);
		vec3 side3 = cross(p0_p2, curTri.normVec);

		if (side1 * p2_p1 > 0.0f) side1 = -side1;
		if (side2 * p0_p2 > 0.0f) side2 = -side2;
		if (side3 * p1_p0 > 0.0f) side3 = -side3;

		sideNorms.push_back(side1);
		sideNorms.push_back(side2);
		sideNorms.push_back(side3);
	}
}

bool HIGHOMEGA::GEOM::InsideTest(vec3 pt, std::vector<TriUV>& allTris, std::vector<vec3>& sideNorms)
{
	vec3 pt1 = pt;
	vec3 pt2 = pt - vec3 (0.0f, 0.0f, 1000.0f);
	vec3 pt3 = pt - vec3(0.0f, 1000.0f, 0.0f);
	vec3 pt4 = pt - vec3(1000.0f, 0.0f, 0.0f);

	float tmpK;
	unsigned int countHits1 = 0, countHits2 = 0, countHits3 = 0;
	unsigned insideEval = 0;
	for (unsigned int i = 0; i != allTris.size(); i++)
	{
		if (LineSegTri(pt1, pt2, allTris[i].eArr[0], allTris[i].eArr[1], allTris[i].eArr[2], allTris[i].normVec, sideNorms[i * 3], sideNorms[i * 3 + 1], sideNorms[i * 3 + 2], tmpK)) countHits1++;
		if (LineSegTri(pt1, pt3, allTris[i].eArr[0], allTris[i].eArr[1], allTris[i].eArr[2], allTris[i].normVec, sideNorms[i * 3], sideNorms[i * 3 + 1], sideNorms[i * 3 + 2], tmpK)) countHits2++;
	}
	if (!(countHits1 % 2 == 0)) insideEval++;
	if (!(countHits2 % 2 == 0)) insideEval++;

	if (insideEval == 0) return false;
	if (insideEval == 2) return true;

	for (unsigned int i = 0; i != allTris.size(); i++)
		if (LineSegTri(pt1, pt4, allTris[i].eArr[0], allTris[i].eArr[1], allTris[i].eArr[2], allTris[i].normVec, sideNorms[i * 3], sideNorms[i * 3 + 1], sideNorms[i * 3 + 2], tmpK)) countHits3++;

	if (insideEval == 1) return false;
	return true;
}

bool HIGHOMEGA::GEOM::Intersect(std::vector<TriUV>& slicer, std::vector<TriUV>& slicee, std::vector<TriUV> & inside, std::vector<TriUV> & outside)
{
	std::vector<TriUV> sliceeCopy = slicee, sliceeCopy2, isectTmp;
	sliceeCopy2.reserve(sliceeCopy.size());

	bool intersected = false;
	for (TriUV & inputTri : slicer)
	{
		for (int i = 0; i != sliceeCopy.size(); i++)
		{
			if (SplitTri(sliceeCopy[i], inputTri, isectTmp, true))
			{
				sliceeCopy2.insert(sliceeCopy2.end(), isectTmp.begin(), isectTmp.end());
				isectTmp.clear();
				intersected = true;
			}
			else
			{
				sliceeCopy2.push_back(sliceeCopy[i]);
			}
		}
		sliceeCopy = sliceeCopy2;
		sliceeCopy2.clear();
	}

	if (!intersected)
	{
		outside = slicee;
		return false;
	}

	std::vector<vec3> slicerSideNorms;
	inside.reserve(sliceeCopy.size());
	outside.reserve(sliceeCopy.size());
	slicerSideNorms.reserve(slicer.size() * 3);
	ComputeSideNorms(slicer, slicerSideNorms);

	for (TriUV & aFragment : sliceeCopy)
	{
		vec3 fragCent = (aFragment.eArr[0] + aFragment.eArr[1] + aFragment.eArr[2]) * 0.3333333f;
		if (InsideTest(fragCent, slicer, slicerSideNorms))
			inside.push_back(aFragment);
		else
			outside.push_back(aFragment);
	}

	return true;
}

void HIGHOMEGA::GEOM::Slice(TriUV & slicer, std::vector<TriUV>& slicee, std::vector<TriUV>& above, std::vector<TriUV>& below)
{
	std::vector<TriUV> sliceeCopy, isectTmp;
	sliceeCopy.reserve(slicee.size());

	for (int i = 0; i != slicee.size(); i++)
	{
		if (SplitTri(slicee[i], slicer, isectTmp, true))
		{
			sliceeCopy.insert(sliceeCopy.end(), isectTmp.begin(), isectTmp.end());
			isectTmp.clear();
		}
		else
		{
			sliceeCopy.push_back(slicee[i]);
		}
	}

	above.reserve(sliceeCopy.size());
	below.reserve(sliceeCopy.size());

	for (TriUV & aFragment : sliceeCopy)
	{
		vec3 fragCent = (aFragment.eArr[0] + aFragment.eArr[1] + aFragment.eArr[2]) * 0.3333333f;
		if ((fragCent - slicer.eArr[0]) * slicer.normVec > 0.0f)
			above.push_back(aFragment);
		else
			below.push_back(aFragment);
	}
}

void HIGHOMEGA::GEOM::AddTri(vec3 e[3], vec2 uv[3], vec3 & normVec, std::vector<TriUV>& triList)
{
	vec3 triCross = cross(e[0] - e[1], e[2] - e[1]);
	if (triCross * triCross < 0.0000004f) return;
	TriUV tmpTri;
	tmpTri.eArr[0] = e[0];
	tmpTri.eArr[1] = e[1];
	tmpTri.eArr[2] = e[2];
	tmpTri.uvArr[0] = uv[0];
	tmpTri.uvArr[1] = uv[1];
	tmpTri.uvArr[2] = uv[2];
	tmpTri.normVec = normVec;
	triList.push_back(tmpTri);
}

vec3 HIGHOMEGA::GEOM::Recenter(std::vector<TriUV>& triList)
{
	vec3 geomCenter(0.0f);
	for (TriUV & curTri : triList)
	{
		geomCenter += curTri.eArr[0];
		geomCenter += curTri.eArr[1];
		geomCenter += curTri.eArr[2];
	}
	geomCenter = geomCenter / (triList.size() * 3.0f);

	for (TriUV & curTri : triList)
	{
		curTri.eArr[0] -= geomCenter;
		curTri.eArr[1] -= geomCenter;
		curTri.eArr[2] -= geomCenter;
	}

	return geomCenter;
}

void HIGHOMEGA::GEOM::GenerateAABB(std::vector<TriUV>& triList, std::vector <TriUV> & aabbTris, float epsilon)
{
	vec3 minVec = triList[0].eArr[0], maxVec = triList[0].eArr[0];
	for (TriUV & curTri : triList)
	{
		minVec.x = min(Min(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x), minVec.x);
		minVec.y = min(Min(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y), minVec.y);
		minVec.z = min(Min(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z), minVec.z);

		maxVec.x = max(Max(curTri.eArr[0].x, curTri.eArr[1].x, curTri.eArr[2].x), maxVec.x);
		maxVec.y = max(Max(curTri.eArr[0].y, curTri.eArr[1].y, curTri.eArr[2].y), maxVec.y);
		maxVec.z = max(Max(curTri.eArr[0].z, curTri.eArr[1].z, curTri.eArr[2].z), maxVec.z);
	}

	if (epsilon != 0.0f)
	{
		vec3 aabbLen = maxVec - minVec;
		if (aabbLen.x < epsilon)
		{
			maxVec.x += epsilon * 0.5f;
			minVec.x -= epsilon * 0.5f;
		}
		if (aabbLen.y < epsilon)
		{
			maxVec.y += epsilon * 0.5f;
			minVec.y -= epsilon * 0.5f;
		}
		if (aabbLen.z < epsilon)
		{
			maxVec.z += epsilon * 0.5f;
			minVec.z -= epsilon * 0.5f;
		}
	}

	MakeBox(minVec, maxVec, aabbTris);
}

bool HIGHOMEGA::GEOM::ManhattanDistClose(TriUV & t1, TriUV & t2)
{
	vec3 diff1;
	float manDist;

	diff1 = t1.eArr[0] - t2.eArr[0];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[0] - t2.eArr[1];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[0] - t2.eArr[2];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[1] - t2.eArr[0];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[1] - t2.eArr[1];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[1] - t2.eArr[2];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[2] - t2.eArr[0];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[2] - t2.eArr[1];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	diff1 = t1.eArr[2] - t2.eArr[2];
	manDist = fabsf(diff1.x) + fabsf(diff1.y) + fabsf(diff1.z);
	if (manDist < 0.0001f) return true;

	return false;
}

void HIGHOMEGA::GEOM::FindConnectedNeighbors(std::vector<TriUV>& triListSrc, std::vector<TriUV>& triListDst)
{
	if (triListSrc.size() == 0)
	{
		triListDst.clear();
		return;
	}

	std::vector <bool> visitedTris;
	visitedTris.resize(triListSrc.size());
	for (int i = 0; i != visitedTris.size(); i++)
		visitedTris[i] = false;
	visitedTris[0] = true;

	bool foundOne = true;
	while (foundOne)
	{
		foundOne = false;
		for (int i = 0; i != triListSrc.size(); i++)
		{
			for (int j = 0; j != triListSrc.size(); j++)
			{
				if (i == j || visitedTris[i] == visitedTris[j]) continue;
				vec3 *eArr1 = triListSrc[i].eArr;
				vec3 *eArr2 = triListSrc[j].eArr;
				if (ManhattanDistClose(triListSrc[i], triListSrc[j]))
				{
					visitedTris[i] = visitedTris[j] = true;
					foundOne = true;
				}
			}
		}
	}

	std::vector <TriUV> newSrc;
	newSrc.reserve(triListSrc.size());
	triListDst.clear();
	triListDst.reserve(triListSrc.size());
	for (int i = 0; i != visitedTris.size(); i++)
		if (visitedTris[i]) triListDst.push_back(triListSrc[i]);
		else newSrc.push_back(triListSrc[i]);

	triListSrc = newSrc;
}

HIGHOMEGA::GEOM::Line::Line(vec3 p1, vec3 p2)
{
	this->p1 = p1;
	this->p2 = p2;
}

void HIGHOMEGA::GEOM::FindEdges(std::vector<TriUV>& triList, std::vector<Line>& edges)
{
	vec3 diff1, diff2;
	for (TriUV & curTri : triList)
	{
		for (int j = 0; j != 3; j++)
		{
			vec3 l1 = curTri.eArr[j];
			vec3 l2 = curTri.eArr[(j + 1) % 3];
			bool lineExists = false;
			for (int k = 0; k != edges.size(); k++)
			{
				diff1 = l1 - edges[k].p1;
				diff2 = l2 - edges[k].p2;
				if (diff1*diff1 < 0.000625f && diff2*diff2 < 0.000625f)
				{
					edges.erase(edges.begin() + k);
					lineExists = true;
					break;
				}
				diff1 = l1 - edges[k].p2;
				diff2 = l2 - edges[k].p1;
				if (diff1*diff1 < 0.000625f && diff2*diff2 < 0.000625f)
				{
					edges.erase(edges.begin() + k);
					lineExists = true;
					break;
				}
			}
			if (!lineExists)
				edges.emplace_back(l1, l2);
		}
	}
}