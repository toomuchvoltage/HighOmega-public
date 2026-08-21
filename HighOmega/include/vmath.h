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

#include <xmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>
#include <math.h>

// NOTE: This file contains SSE4/SIMD optimized code, read at your own risk

#ifndef min
#define min(a,b) (((a)<(b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) (((a)>(b)) ? (a) : (b))
#endif

#define Min(x,y,z) min (min(x,y),z)
#define Max(x,y,z) max (max(x,y),z)

namespace HIGHOMEGA
{
	namespace MATH
	{
		// #define ZERO 0.0001 [==> only needed by the LCP solver <==]
#define HIGHOMEGA_PI 3.14159265358979323846264338327950288f
#define HIGHOMEGA_INV_PI 0.31830988618379067153776752674503f
#define HIGHOMEGA_INV_2PI 0.15915494309189533576888376337251f

		class vec2
		{
		public:
			float x, y;
			vec2();
			vec2(float i);
			vec2(float ix, float iy);
			vec2& operator=(const vec2& rhs);
			vec2 operator+(const vec2& rhs) const;
			vec2 operator-(const vec2& rhs) const;
			float operator*(const vec2& in) const;
			vec2 operator*(const float n) const;
			vec2 operator/(const float n) const;
			vec2& operator+=(const vec2& rhs);
			vec2& operator-=(const vec2& rhs);
			vec2& operator*=(const float rhs);
			vec2& operator/=(const float rhs);
			vec2 operator- () const;
			bool operator== (const vec2& rhs) const;
			bool operator!= (const vec2& rhs) const;
		};

		class vec3
		{
		public:
			float x, y, z;
			vec3();
			vec3(float i);
			vec3(float ix, float iy, float iz);
			vec3& operator=(const vec3& rhs);
			vec3 operator+(const vec3& rhs) const;
			vec3 operator-(const vec3& rhs) const;
			float operator*(const vec3& in) const;
			vec3 operator*(const float n) const;
			vec3 operator/(const float n) const;
			vec3& operator+=(const vec3& rhs);
			vec3& operator-=(const vec3& rhs);
			vec3& operator*=(const float rhs);
			vec3& operator/=(const float rhs);
			vec3 operator-() const;
			float length() const;
			vec3 normalized() const;
			vec3& pointwiseMul(vec3 & rhs);
			vec3 inv() const;
			vec3& round();
			bool operator== (const vec3& rhs) const;
			bool operator!= (const vec3& rhs) const;
		};

		class mat4
		{
		public:
			float i[4][4];
			mat4();
			mat4(float _11, float _12, float _13, float _14,
				float _21, float _22, float _23, float _24,
				float _31, float _32, float _33, float _34,
				float _41, float _42, float _43, float _44);
			mat4& operator=(const mat4& in);
			mat4 operator+(const mat4& in) const;
			mat4 operator-(const mat4& in) const;
			mat4 operator* (const float n) const;
			mat4 operator* (const mat4& in) const;
			vec3 operator* (const vec3& in) const;
			mat4 operator/(const float n) const;
			mat4& operator+=(const mat4& in);
			mat4& operator-=(const mat4& in);
			mat4& operator*=(const float n);
			mat4& operator*=(const mat4& in);
			mat4& operator/=(const float n);
			bool operator==(const mat4& in) const;
			bool operator!=(const mat4& in) const;
			mat4 Inv() const;
			mat4 Transpose() const;
			mat4 DirectionTransform() const;
			void Ident();
		};

		struct range
		{
			vec3 rmin, rmax;
		};

		struct irange
		{
			unsigned int rmin[4];
			unsigned int rmax[4];
		};

		struct edge
		{
			vec3 e1, e2;
		};

		bool WithinAABB(const vec3 & p1, const  vec3 & p2, float eps);

		vec2 operator*(const float n, const vec2& in);

		vec3 operator*(const float n, const vec3& in);

		mat4 operator*(const float n, const mat4& in);

		vec3 cross(const vec3& a, const vec3& b);

		vec2 toSpherical(const vec3& inpVec);

		vec3 toCartesian(const vec2& inpVec);

		unsigned int toZSignXY(const vec3& inpVec);

		vec3 fromZSignXY(unsigned int inpPack);

		float packSpherical(vec2 & inpSph);

		vec2 unpackSpherical(float inpPack);

		float packColor(const vec3 & color);

		float packColor(const vec3 & color, float alpha);

		void unpackColor(float inpPack, vec3 & color);

		unsigned short toFP16(float f);

		float toFP16(const vec2& f);

		float toFP32(unsigned short h);

		vec2 toFP32(float hv2);

		vec3 Slerp(const vec3& currentVec, const vec3& targetVec, float smoothingFactor);

		float Lerp(float a, float b, float lerpFactor);

		inline float Clamp(float x, float a, float b)
		{
			return min(max(x, a), b);
		}

		inline int Clamp(int x, int a, int b)
		{
			return min(max(x, a), b);
		}

		inline unsigned int Clamp(unsigned int x, unsigned int a, unsigned int b)
		{
			return min(max(x, a), b);
		}

		vec3 QuadraticBezier(const vec3& p0, const vec3& p1, const vec3& p2, float t);

		void GetTangentBiTangent(vec3 v[3], vec2 tex[3], vec3 & x_1_out, vec3 & y_1_out);

		void GetSideNormals(const vec3 & e1, const vec3 & e2, const vec3 & e3, const vec3 & n, vec3 & s1, vec3 & s2, vec3 & s3);

		vec2 Lerp(const vec2& a, const vec2& b, float lerpFactor);

		vec3 Lerp(const vec3& a, const vec3& b, float lerpFactor);

		float SmootherstepFast(float inp);

		vec3 Spin(const vec3& axis, const vec3& vec, float angle);

		float sign(float in);

		float signEpsilon(float in);

		void ShiftLeft(unsigned int & num1, unsigned int & num2, unsigned int & num3, unsigned int & num4, int amount);

		void ShiftLeft(unsigned int & num1, unsigned int & num2, unsigned int & num3, unsigned int & num4, int amount1, int amount2, int amount3, int amount4);

		void And(unsigned int & num1, unsigned int & num2, unsigned int & num3, unsigned int & num4, int amount);

		void TransformCorners(vec3& cornersMin, vec3& cornersMax, const mat4& inpMat);
		/*

		[==>Since the LCP based approach is not being used the Gauss-Jordan solver,  <==]
		[==>the Dantzig-based LCP solver and the PGS LCP solver are of no use.       <==]
		[==>either way they're kept here.                                            <==]

		float LCP_M[500][500];
		float LCP_z[500];
		float LCP_Q[500];
		float LCP_hi[500];

		void Gauss_Seidel_LCP (int n);

		static float A11[1000][1000];
		static float v1[1000];
		bool GJSolve (int n);

		// Dantzig-based solver (by Baraff)
		float LCP_M[1000][1000],LCP_Q[1000];
		float LCP_f[1000];
		static float a[1000];
		static float da[1000];
		static float df[1000];
		static int type[1000];
		int SolveLCP (int n);

		[==>end of commented out unused solver stuff<==]

		*/

		vec3 ReflectPos(const vec3& i, const vec3& n, const vec3& p);

		vec3 ReflectDir(const vec3& i, const vec3& n);

		mat4 RotationX(float deg);
		mat4 RotationY(float deg);
		mat4 RotationZ(float deg);
	}
}