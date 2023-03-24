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

#include <vmath.h>

using namespace HIGHOMEGA::MATH;

vec2::vec2()
{
	x = 0; y = 0;
}
vec2::vec2(float i)
{
	x = i; y = i;
}
vec2::vec2(float ix, float iy)
{
	x = ix; y = iy;
}
vec2& vec2::operator=(const vec2& rhs)
{
	x = rhs.x; y = rhs.y;
	return *this;
}
vec2 vec2::operator+(const vec2& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, _2_a[4] = { rhs.x,rhs.y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_add_ps(*_1_p, *_2_p);
	return vec2(res_a[0], res_a[1]);
}
vec2 vec2::operator-(const vec2& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, _2_a[4] = { rhs.x,rhs.y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_sub_ps(*_1_p, *_2_p);
	return vec2(res_a[0], res_a[1]);
}
float vec2::operator*(const vec2& in)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, _2_a[4] = { in.x,in.y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_dp_ps(*_1_p, *_2_p, 0b00110001);
	return res_a[0];
}
vec2 vec2::operator*(const float n)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(n), *res_p = (__m128 *)res_a;
	*res_p = _mm_mul_ps(*_1_p, _2_p);
	return vec2(res_a[0], res_a[1]);
}
vec2 vec2::operator/(const float n)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(n), *res_p = (__m128 *)res_a;
	*res_p = _mm_div_ps(*_1_p, _2_p);
	return vec2(res_a[0], res_a[1]);
}
vec2& vec2::operator+=(const vec2& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, _2_a[4] = { rhs.x,rhs.y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_add_ps(*_1_p, *_2_p);
	x = res_a[0];
	y = res_a[1];
	return *this;
}
vec2& vec2::operator-=(const vec2& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, _2_a[4] = { rhs.x,rhs.y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_sub_ps(*_1_p, *_2_p);
	x = res_a[0];
	y = res_a[1];
	return *this;
}
vec2& vec2::operator*=(const float rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(rhs), *res_p = (__m128 *)res_a;
	*res_p = _mm_mul_ps(*_1_p, _2_p);
	x = res_a[0];
	y = res_a[1];
	return *this;
}
vec2& vec2::operator/=(const float rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,0.0f,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(rhs), *res_p = (__m128 *)res_a;
	*res_p = _mm_div_ps(*_1_p, _2_p);
	x = res_a[0];
	y = res_a[1];
	return *this;
}
vec2 vec2::operator- ()
{
	return vec2(-x, -y);
}
bool vec2::operator== (const vec2& rhs)
{
	if (rhs.x == x && rhs.y == y)
		return true;
	else
		return false;
}
bool vec2::operator!= (const vec2& rhs)
{
	if (rhs.x != x || rhs.y != y)
		return true;
	else
		return false;
}

vec3::vec3()
{
	x = 0; y = 0; z = 0;
}
vec3::vec3(float i)
{
	x = i; y = i; z = i;
}
vec3::vec3(float ix, float iy, float iz)
{
	x = ix; y = iy; z = iz;
}
vec3& vec3::operator=(const vec3& rhs)
{
	x = rhs.x; y = rhs.y; z = rhs.z;
	return *this;
}
vec3 vec3::operator+(const vec3& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, _2_a[4] = { rhs.x,rhs.y,rhs.z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_add_ps(*_1_p, *_2_p);
	return vec3(res_a[0], res_a[1], res_a[2]);
}
vec3 vec3::operator-(const vec3& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, _2_a[4] = { rhs.x,rhs.y,rhs.z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_sub_ps(*_1_p, *_2_p);
	return vec3(res_a[0], res_a[1], res_a[2]);
}
float vec3::operator*(const vec3& in)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, _2_a[4] = { in.x,in.y,in.z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_dp_ps(*_1_p, *_2_p, 0b01110001);
	return res_a[0];
}
vec3 vec3::operator*(const float n)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(n), *res_p = (__m128 *)res_a;
	*res_p = _mm_mul_ps(*_1_p, _2_p);
	return vec3(res_a[0], res_a[1], res_a[2]);
}
vec3 vec3::operator/(const float n)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(n), *res_p = (__m128 *)res_a;
	*res_p = _mm_div_ps(*_1_p, _2_p);
	return vec3(res_a[0], res_a[1], res_a[2]);
}
vec3& vec3::operator+=(const vec3& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, _2_a[4] = { rhs.x,rhs.y,rhs.z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_add_ps(*_1_p, *_2_p);
	x = res_a[0];
	y = res_a[1];
	z = res_a[2];
	return *this;
}
vec3& vec3::operator-=(const vec3& rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, _2_a[4] = { rhs.x,rhs.y,rhs.z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_sub_ps(*_1_p, *_2_p);
	x = res_a[0];
	y = res_a[1];
	z = res_a[2];
	return *this;
}
vec3& vec3::operator*=(const float rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(rhs), *res_p = (__m128 *)res_a;
	*res_p = _mm_mul_ps(*_1_p, _2_p);
	x = res_a[0];
	y = res_a[1];
	z = res_a[2];
	return *this;
}
vec3& vec3::operator/=(const float rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps1(rhs), *res_p = (__m128 *)res_a;
	*res_p = _mm_div_ps(*_1_p, _2_p);
	x = res_a[0];
	y = res_a[1];
	z = res_a[2];
	return *this;
}
vec3 vec3::operator-()
{
	return vec3(-x, -y, -z);
}
float vec3::length()
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, _2_a[4] = { x,y,z,0.0f }, res_a[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	__m128 *_1_p = (__m128 *)_1_a, *_2_p = (__m128 *)_2_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_dp_ps(*_1_p, *_2_p, 0b01110001);
	*res_p = _mm_sqrt_ss(*res_p);
	return res_a[0];
}
vec3 vec3::normalized()
{
	float len = length();
	if (len == 0.0)
		return vec3(0);
	else
	{
		return (*this) / len;
	}
}
vec3 & HIGHOMEGA::MATH::vec3::pointwiseMul(vec3 & rhs)
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, _2_p = _mm_set_ps(0.0f, rhs.z, rhs.y, rhs.x), *res_p = (__m128 *)res_a;
	*res_p = _mm_mul_ps(*_1_p, _2_p);
	x = res_a[0];
	y = res_a[1];
	z = res_a[2];
	return *this;
}
vec3 & HIGHOMEGA::MATH::vec3::round()
{
	__declspec(align(16)) float _1_a[4] = { x,y,z,0.0f }, res_a[4];
	__m128 *_1_p = (__m128 *)_1_a, *res_p = (__m128 *)res_a;
	*res_p = _mm_round_ps(*_1_p, _MM_FROUND_TO_NEAREST_INT);
	x = res_a[0];
	y = res_a[1];
	z = res_a[2];
	return *this;
}
bool vec3::operator== (const vec3& rhs)
{
	if (rhs.x == x && rhs.y == y && rhs.z == z)
		return true;
	else
		return false;
}
bool vec3::operator!= (const vec3& rhs)
{
	if (rhs.x != x || rhs.y != y || rhs.z != z)
		return true;
	else
		return false;
}

mat4::mat4()
{
	this->i[0][0] = 0;
	this->i[0][1] = 0;
	this->i[0][2] = 0;
	this->i[0][3] = 0;

	this->i[1][0] = 0;
	this->i[1][1] = 0;
	this->i[1][2] = 0;
	this->i[1][3] = 0;

	this->i[2][0] = 0;
	this->i[2][1] = 0;
	this->i[2][2] = 0;
	this->i[2][3] = 0;

	this->i[3][0] = 0;
	this->i[3][1] = 0;
	this->i[3][2] = 0;
	this->i[3][3] = 0;
}
mat4::mat4(float _11, float _12, float _13, float _14,
	float _21, float _22, float _23, float _24,
	float _31, float _32, float _33, float _34,
	float _41, float _42, float _43, float _44)
{
	i[0][0] = _11; i[0][1] = _12; i[0][2] = _13; i[0][3] = _14;
	i[1][0] = _21; i[1][1] = _22; i[1][2] = _23; i[1][3] = _24;
	i[2][0] = _31; i[2][1] = _32; i[2][2] = _33; i[2][3] = _34;
	i[3][0] = _41; i[3][1] = _42; i[3][2] = _43; i[3][3] = _44;
}
mat4& mat4::operator=(const mat4& in)
{
	this->i[0][0] = in.i[0][0];
	this->i[0][1] = in.i[0][1];
	this->i[0][2] = in.i[0][2];
	this->i[0][3] = in.i[0][3];

	this->i[1][0] = in.i[1][0];
	this->i[1][1] = in.i[1][1];
	this->i[1][2] = in.i[1][2];
	this->i[1][3] = in.i[1][3];

	this->i[2][0] = in.i[2][0];
	this->i[2][1] = in.i[2][1];
	this->i[2][2] = in.i[2][2];
	this->i[2][3] = in.i[2][3];

	this->i[3][0] = in.i[3][0];
	this->i[3][1] = in.i[3][1];
	this->i[3][2] = in.i[3][2];
	this->i[3][3] = in.i[3][3];
	return *this;
}
mat4 mat4::operator+(const mat4& in)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i2[16] = { in.i[0][0],in.i[0][1],in.i[0][2],in.i[0][3],
		in.i[1][0],in.i[1][1],in.i[1][2],in.i[1][3],
		in.i[2][0],in.i[2][1],in.i[2][2],in.i[2][3],
		in.i[3][0],in.i[3][1],in.i[3][2],in.i[3][3] };
	__declspec(align(64)) float i3[16];
	mat4 out;
	__m256 *_1_p1 = (__m256 *)&i1[0], *_2_p1 = (__m256 *)&i2[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_1_p2 = (__m256 *)&i1[8], *_2_p2 = (__m256 *)&i2[8], *res_p2 = (__m256 *)&i3[8];
	*res_p1 = _mm256_add_ps(*_1_p1, *_2_p1);
	*res_p2 = _mm256_add_ps(*_1_p2, *_2_p2);
	out.i[0][0] = i3[0]; out.i[0][1] = i3[1]; out.i[0][2] = i3[2]; out.i[0][3] = i3[3];
	out.i[1][0] = i3[4]; out.i[1][1] = i3[5]; out.i[1][2] = i3[6]; out.i[1][3] = i3[7];
	out.i[2][0] = i3[8]; out.i[2][1] = i3[9]; out.i[2][2] = i3[10]; out.i[2][3] = i3[11];
	out.i[3][0] = i3[12]; out.i[3][1] = i3[13]; out.i[3][2] = i3[14]; out.i[3][3] = i3[15];
	return out;
}
mat4 mat4::operator-(const mat4& in)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i2[16] = { in.i[0][0],in.i[0][1],in.i[0][2],in.i[0][3],
		in.i[1][0],in.i[1][1],in.i[1][2],in.i[1][3],
		in.i[2][0],in.i[2][1],in.i[2][2],in.i[2][3],
		in.i[3][0],in.i[3][1],in.i[3][2],in.i[3][3] };
	__declspec(align(64)) float i3[16];
	mat4 out;
	__m256 *_1_p1 = (__m256 *)&i1[0], *_2_p1 = (__m256 *)&i2[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_1_p2 = (__m256 *)&i1[8], *_2_p2 = (__m256 *)&i2[8], *res_p2 = (__m256 *)&i3[8];
	*res_p1 = _mm256_sub_ps(*_1_p1, *_2_p1);
	*res_p2 = _mm256_sub_ps(*_1_p2, *_2_p2);
	out.i[0][0] = i3[0]; out.i[0][1] = i3[1]; out.i[0][2] = i3[2]; out.i[0][3] = i3[3];
	out.i[1][0] = i3[4]; out.i[1][1] = i3[5]; out.i[1][2] = i3[6]; out.i[1][3] = i3[7];
	out.i[2][0] = i3[8]; out.i[2][1] = i3[9]; out.i[2][2] = i3[10]; out.i[2][3] = i3[11];
	out.i[3][0] = i3[12]; out.i[3][1] = i3[13]; out.i[3][2] = i3[14]; out.i[3][3] = i3[15];
	return out;
}
mat4 mat4::operator* (const float n)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i2[16];
	mat4 out;
	__m256 *_i1p1 = (__m256 *)&i1[0], *res_p1 = (__m256 *)&i2[0];
	__m256 *_i1p2 = (__m256 *)&i1[8], *res_p2 = (__m256 *)&i2[8];
	__m256 fact = _mm256_set_ps(n, n, n, n, n, n, n, n);
	*res_p1 = _mm256_mul_ps(*_i1p1, fact);
	*res_p2 = _mm256_mul_ps(*_i1p2, fact);
	out.i[0][0] = i2[0]; out.i[0][1] = i2[1]; out.i[0][2] = i2[2]; out.i[0][3] = i2[3];
	out.i[1][0] = i2[4]; out.i[1][1] = i2[5]; out.i[1][2] = i2[6]; out.i[1][3] = i2[7];
	out.i[2][0] = i2[8]; out.i[2][1] = i2[9]; out.i[2][2] = i2[10]; out.i[2][3] = i2[11];
	out.i[3][0] = i2[12]; out.i[3][1] = i2[13]; out.i[3][2] = i2[14]; out.i[3][3] = i2[15];
	return out;
}
mat4 mat4::operator* (const mat4& in)
{
	mat4 out;
	__declspec(align(128)) float i_[32] = { i[0][0],i[0][1],i[0][2],i[0][3],i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3],i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(32)) float rhs1[8] = { in.i[0][0],in.i[1][0],in.i[2][0],in.i[3][0],in.i[0][1],in.i[1][1],in.i[2][1],in.i[3][1] };
	__declspec(align(32)) float rhs2[8] = { in.i[0][2],in.i[1][2],in.i[2][2],in.i[3][2],in.i[0][3],in.i[1][3],in.i[2][3],in.i[3][3] };
	__declspec(align(32)) float res[8];
	__m256 *rhsp1 = (__m256 *)rhs1, *rhsp2 = (__m256 *)rhs2, *resp = (__m256 *)res;
	*resp = _mm256_dp_ps(*((__m256 *)i_), *rhsp1, 0b11110001); out.i[0][0] = res[0]; out.i[0][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)i_), *rhsp2, 0b11110001); out.i[0][2] = res[0]; out.i[0][3] = res[4];

	*resp = _mm256_dp_ps(*((__m256 *)&i_[8]), *rhsp1, 0b11110001); out.i[1][0] = res[0]; out.i[1][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)&i_[8]), *rhsp2, 0b11110001); out.i[1][2] = res[0]; out.i[1][3] = res[4];

	*resp = _mm256_dp_ps(*((__m256 *)&i_[16]), *rhsp1, 0b11110001); out.i[2][0] = res[0]; out.i[2][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)&i_[16]), *rhsp2, 0b11110001); out.i[2][2] = res[0]; out.i[2][3] = res[4];

	*resp = _mm256_dp_ps(*((__m256 *)&i_[24]), *rhsp1, 0b11110001); out.i[3][0] = res[0]; out.i[3][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)&i_[24]), *rhsp2, 0b11110001); out.i[3][2] = res[0]; out.i[3][3] = res[4];

	return out;
}
vec3 mat4::operator* (const vec3& in)
{
	__declspec(align(32)) float lhs[8] = { in.x,in.y,in.z,0.0f,in.x,in.y,in.z,0.0f };
	__declspec(align(32)) float rhs1[8] = { i[0][0],i[0][1],i[0][2],0.0,i[1][0],i[1][1],i[1][2],0.0 };
	__declspec(align(32)) float res1[8];
	__declspec(align(16)) float rhs2[4] = { i[2][0],i[2][1],i[2][2],0.0 };
	__declspec(align(16)) float res2[4];
	__m256 *lhsp = (__m256 *)lhs;
	__m256 *rhsp1 = (__m256 *)rhs1, *resp1 = (__m256 *)res1;
	__m128 *rhsp2 = (__m128 *)&rhs2, *resp2 = (__m128 *)&res2;
	*resp1 = _mm256_dp_ps(*lhsp, *rhsp1, 0b01110001);
	*resp2 = _mm_dp_ps(*((__m128 *)lhsp), *rhsp2, 0b01110001);

	return vec3(res1[0] + i[0][3], res1[4] + i[1][3], res2[0] + i[2][3]);
}
mat4 mat4::operator/(const float n)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i3[16];
	mat4 out;
	__m256 *_i1p1 = (__m256 *)&i1[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_i1p2 = (__m256 *)&i1[8], *res_p2 = (__m256 *)&i3[8];
	__m256 fact = _mm256_set_ps(n, n, n, n, n, n, n, n);
	*res_p1 = _mm256_div_ps(*_i1p1, fact);
	*res_p2 = _mm256_div_ps(*_i1p2, fact);
	out.i[0][0] = i3[0]; out.i[0][1] = i3[1]; out.i[0][2] = i3[2]; out.i[0][3] = i3[3];
	out.i[1][0] = i3[4]; out.i[1][1] = i3[5]; out.i[1][2] = i3[6]; out.i[1][3] = i3[7];
	out.i[2][0] = i3[8]; out.i[2][1] = i3[9]; out.i[2][2] = i3[10]; out.i[2][3] = i3[11];
	out.i[3][0] = i3[12]; out.i[3][1] = i3[13]; out.i[3][2] = i3[14]; out.i[3][3] = i3[15];
	return out;
}
mat4& mat4::operator+=(const mat4& in)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i2[16] = { in.i[0][0],in.i[0][1],in.i[0][2],in.i[0][3],
		in.i[1][0],in.i[1][1],in.i[1][2],in.i[1][3],
		in.i[2][0],in.i[2][1],in.i[2][2],in.i[2][3],
		in.i[3][0],in.i[3][1],in.i[3][2],in.i[3][3] };
	__declspec(align(64)) float i3[16];
	__m256 *_i1p1 = (__m256 *)&i1[0], *_i2p1 = (__m256 *)&i2[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_i1p2 = (__m256 *)&i1[8], *_i2p2 = (__m256 *)&i2[8], *res_p2 = (__m256 *)&i3[8];
	*res_p1 = _mm256_add_ps(*_i1p1, *_i2p1);
	*res_p2 = _mm256_add_ps(*_i1p2, *_i2p2);
	i[0][0] = i3[0]; i[0][1] = i3[1]; i[0][2] = i3[2]; i[0][3] = i3[3];
	i[1][0] = i3[4]; i[1][1] = i3[5]; i[1][2] = i3[6]; i[1][3] = i3[7];
	i[2][0] = i3[8]; i[2][1] = i3[9]; i[2][2] = i3[10]; i[2][3] = i3[11];
	i[3][0] = i3[12]; i[3][1] = i3[13]; i[3][2] = i3[14]; i[3][3] = i3[15];
	return *this;
}
mat4& mat4::operator-=(const mat4& in)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i2[16] = { in.i[0][0],in.i[0][1],in.i[0][2],in.i[0][3],
		in.i[1][0],in.i[1][1],in.i[1][2],in.i[1][3],
		in.i[2][0],in.i[2][1],in.i[2][2],in.i[2][3],
		in.i[3][0],in.i[3][1],in.i[3][2],in.i[3][3] };
	__declspec(align(64)) float i3[16];
	__m256 *_i1p1 = (__m256 *)&i1[0], *_i2p1 = (__m256 *)&i2[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_i1p2 = (__m256 *)&i1[8], *_i2p2 = (__m256 *)&i2[8], *res_p2 = (__m256 *)&i3[8];
	*res_p1 = _mm256_sub_ps(*_i1p1, *_i2p1);
	*res_p2 = _mm256_sub_ps(*_i1p2, *_i2p2);
	i[0][0] = i3[0]; i[0][1] = i3[1]; i[0][2] = i3[2]; i[0][3] = i3[3];
	i[1][0] = i3[4]; i[1][1] = i3[5]; i[1][2] = i3[6]; i[1][3] = i3[7];
	i[2][0] = i3[8]; i[2][1] = i3[9]; i[2][2] = i3[10]; i[2][3] = i3[11];
	i[3][0] = i3[12]; i[3][1] = i3[13]; i[3][2] = i3[14]; i[3][3] = i3[15];
	return *this;
}
mat4& mat4::operator*=(const float n)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i3[16];
	__m256 *_i1p1 = (__m256 *)&i1[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_i1p2 = (__m256 *)&i1[8], *res_p2 = (__m256 *)&i3[8];
	__m256 fact = _mm256_set_ps(n, n, n, n, n, n, n, n);
	*res_p1 = _mm256_mul_ps(*_i1p1, fact);
	*res_p2 = _mm256_mul_ps(*_i1p2, fact);
	i[0][0] = i3[0]; i[0][1] = i3[1]; i[0][2] = i3[2]; i[0][3] = i3[3];
	i[1][0] = i3[4]; i[1][1] = i3[5]; i[1][2] = i3[6]; i[1][3] = i3[7];
	i[2][0] = i3[8]; i[2][1] = i3[9]; i[2][2] = i3[10]; i[2][3] = i3[11];
	i[3][0] = i3[12]; i[3][1] = i3[13]; i[3][2] = i3[14]; i[3][3] = i3[15];
	return *this;
}
mat4 & HIGHOMEGA::MATH::mat4::operator*=(const mat4 & in)
{
	__declspec(align(128)) float i_[32] = { i[0][0],i[0][1],i[0][2],i[0][3],i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3],i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(32)) float rhs1[8] = { in.i[0][0],in.i[1][0],in.i[2][0],in.i[3][0],in.i[0][1],in.i[1][1],in.i[2][1],in.i[3][1] };
	__declspec(align(32)) float rhs2[8] = { in.i[0][2],in.i[1][2],in.i[2][2],in.i[3][2],in.i[0][3],in.i[1][3],in.i[2][3],in.i[3][3] };
	__declspec(align(32)) float res[8];
	__m256 *rhsp1 = (__m256 *)rhs1, *rhsp2 = (__m256 *)rhs2, *resp = (__m256 *)res;
	*resp = _mm256_dp_ps(*((__m256 *)i_), *rhsp1, 0b11110001); i[0][0] = res[0]; i[0][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)i_), *rhsp2, 0b11110001); i[0][2] = res[0]; i[0][3] = res[4];

	*resp = _mm256_dp_ps(*((__m256 *)&i_[8]), *rhsp1, 0b11110001); i[1][0] = res[0]; i[1][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)&i_[8]), *rhsp2, 0b11110001); i[1][2] = res[0]; i[1][3] = res[4];

	*resp = _mm256_dp_ps(*((__m256 *)&i_[16]), *rhsp1, 0b11110001); i[2][0] = res[0]; i[2][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)&i_[16]), *rhsp2, 0b11110001); i[2][2] = res[0]; i[2][3] = res[4];

	*resp = _mm256_dp_ps(*((__m256 *)&i_[24]), *rhsp1, 0b11110001); i[3][0] = res[0]; i[3][1] = res[4];
	*resp = _mm256_dp_ps(*((__m256 *)&i_[24]), *rhsp2, 0b11110001); i[3][2] = res[0]; i[3][3] = res[4];

	return *this;
}
mat4& mat4::operator/=(const float n)
{
	__declspec(align(64)) float i1[16] = { i[0][0],i[0][1],i[0][2],i[0][3],
		i[1][0],i[1][1],i[1][2],i[1][3],
		i[2][0],i[2][1],i[2][2],i[2][3],
		i[3][0],i[3][1],i[3][2],i[3][3] };
	__declspec(align(64)) float i3[16];
	__m256 *_i1p1 = (__m256 *)&i1[0], *res_p1 = (__m256 *)&i3[0];
	__m256 *_i1p2 = (__m256 *)&i1[8], *res_p2 = (__m256 *)&i3[8];
	__m256 fact = _mm256_set_ps(n, n, n, n, n, n, n, n);
	*res_p1 = _mm256_div_ps(*_i1p1, fact);
	*res_p2 = _mm256_div_ps(*_i1p2, fact);
	i[0][0] = i3[0]; i[0][1] = i3[1]; i[0][2] = i3[2]; i[0][3] = i3[3];
	i[1][0] = i3[4]; i[1][1] = i3[5]; i[1][2] = i3[6]; i[1][3] = i3[7];
	i[2][0] = i3[8]; i[2][1] = i3[9]; i[2][2] = i3[10]; i[2][3] = i3[11];
	i[3][0] = i3[12]; i[3][1] = i3[13]; i[3][2] = i3[14]; i[3][3] = i3[15];
	return *this;
}
bool mat4::operator==(const mat4& in)
{
	if (this->i[0][0] != in.i[0][0]) return false;
	if (this->i[0][1] != in.i[0][1]) return false;
	if (this->i[0][2] != in.i[0][2]) return false;
	if (this->i[0][3] != in.i[0][3]) return false;

	if (this->i[1][0] != in.i[1][0]) return false;
	if (this->i[1][1] != in.i[1][1]) return false;
	if (this->i[1][2] != in.i[1][2]) return false;
	if (this->i[1][3] != in.i[1][3]) return false;

	if (this->i[2][0] != in.i[2][0]) return false;
	if (this->i[2][1] != in.i[2][1]) return false;
	if (this->i[2][2] != in.i[2][2]) return false;
	if (this->i[2][3] != in.i[2][3]) return false;

	if (this->i[3][0] != in.i[3][0]) return false;
	if (this->i[3][1] != in.i[3][1]) return false;
	if (this->i[3][2] != in.i[3][2]) return false;
	if (this->i[3][3] != in.i[3][3]) return false;
	return true;
}
bool mat4::operator!=(const mat4& in)
{
	if (this->i[0][0] != in.i[0][0]) return true;
	if (this->i[0][1] != in.i[0][1]) return true;
	if (this->i[0][2] != in.i[0][2]) return true;
	if (this->i[0][3] != in.i[0][3]) return true;

	if (this->i[1][0] != in.i[1][0]) return true;
	if (this->i[1][1] != in.i[1][1]) return true;
	if (this->i[1][2] != in.i[1][2]) return true;
	if (this->i[1][3] != in.i[1][3]) return true;

	if (this->i[2][0] != in.i[2][0]) return true;
	if (this->i[2][1] != in.i[2][1]) return true;
	if (this->i[2][2] != in.i[2][2]) return true;
	if (this->i[2][3] != in.i[2][3]) return true;

	if (this->i[3][0] != in.i[3][0]) return true;
	if (this->i[3][1] != in.i[3][1]) return true;
	if (this->i[3][2] != in.i[3][2]) return true;
	if (this->i[3][3] != in.i[3][3]) return true;
	return false;
}
mat4 HIGHOMEGA::MATH::mat4::Inv()
{
	float C00, C01, C02, C03;
	float C10, C11, C12, C13;
	float C20, C21, C22, C23;
	float C30, C31, C32, C33;

	__declspec(align(32)) float lhs_wide[8] = { i[2][2],i[2][0],i[2][1],i[2][0],i[2][3],i[2][3],i[2][2],i[2][2] };
	__declspec(align(32)) float rhs_wide[8] = { i[3][3],i[3][3],i[3][2],i[3][2],i[3][2],i[3][0],i[3][1],i[3][0] };
	__m256 *lhs_widep = (__m256 *)lhs_wide, *rhs_widep = (__m256 *)rhs_wide;
	__m256 res_wide;
	__declspec(align(16)) float res[4];
	__m128 *resp = (__m128 *)res;
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp = _mm_sub_ps(*((__m128 *)res_wide.m256_f32), *((__m128 *)&res_wide.m256_f32[4]));

	float _2233_2332 = res[0];
	float _2033_2330 = res[1];
	float _2132_2231 = res[2];
	float _2032_2230 = res[3];
	
	__declspec(align(16)) float lhs2[4] = { i[2][0],i[2][1],i[2][1],i[2][3] };
	__declspec(align(16)) float rhs2[4] = { i[3][1],i[3][0],i[3][3],i[3][1] };
	__declspec(align(16)) float res2[4];
	__m128 *_lhs2p = (__m128 *)lhs2, *_rhs2p = (__m128 *)rhs2, *resp2p = (__m128 *)res2;
	*resp = _mm_mul_ps(*_lhs2p, *_rhs2p);
	float _2031_2130 = res[0] - res[1];
	float _2133_2331 = res[2] - res[3];

	lhs_wide[0] = i[1][1]; lhs_wide[1] = i[1][2]; lhs_wide[2] = i[1][3]; lhs_wide[3] = i[1][0]; lhs_wide[4] = i[1][2]; lhs_wide[5] = i[1][3]; lhs_wide[6] = i[1][0]; lhs_wide[7] = i[1][1];
	rhs_wide[0] = _2233_2332; rhs_wide[1] = _2133_2331; rhs_wide[2] = _2132_2231; rhs_wide[3] = _2233_2332; rhs_wide[4] = _2033_2330; rhs_wide[5] = _2032_2230; rhs_wide[6] = _2133_2331; rhs_wide[7] = _2033_2330;
	lhs2[0] = i[1][3]; lhs2[1] = i[1][0]; lhs2[2] = i[1][1]; lhs2[3] = i[1][2];
	rhs2[0] = _2031_2130; rhs2[1] = _2132_2231; rhs2[2] = _2032_2230; rhs2[3] = _2031_2130;
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp2p = _mm_mul_ps(*_lhs2p, *_rhs2p);
	C00 = (res_wide.m256_f32[0]
		- res_wide.m256_f32[1]
		+ res_wide.m256_f32[2]);
	C01 = (res_wide.m256_f32[3]
		- res_wide.m256_f32[4]
		+ res_wide.m256_f32[5]);
	C02 = (res_wide.m256_f32[6]
		- res_wide.m256_f32[7]
		+ res2[0]);
	C03 = (res2[1]
		- res2[2]
		+ res2[3]);

	lhs_wide[0] = i[0][0]; lhs_wide[1] = i[0][1]; lhs_wide[2] = i[0][2]; lhs_wide[3] = i[0][3];
	rhs_wide[0] = C00; rhs_wide[1] = C01; rhs_wide[2] = C02; rhs_wide[3] = C03;
	*resp = _mm_mul_ps(*((__m128 *)lhs_widep->m256_f32), *((__m128 *)rhs_widep->m256_f32));
	float Det = res[0] - res[1] + res[2] - res[3];
	if (Det == 0.0f) Det = 0.00001f; // No way we're gonna ditch this

	lhs_wide[0] = i[1][2]; lhs_wide[1] = i[1][0]; lhs_wide[2] = i[1][1]; lhs_wide[3] = i[1][0]; lhs_wide[4] = i[1][3]; lhs_wide[5] = i[1][3]; lhs_wide[6] = i[1][2]; lhs_wide[7] = i[1][2];
	rhs_wide[0] = i[3][3]; rhs_wide[1] = i[3][3]; rhs_wide[2] = i[3][2]; rhs_wide[3] = i[3][2]; rhs_wide[4] = i[3][2]; rhs_wide[5] = i[3][0]; rhs_wide[6] = i[3][1]; rhs_wide[7] = i[3][0];
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp = _mm_sub_ps(*((__m128 *)res_wide.m256_f32), *((__m128 *)&res_wide.m256_f32[4]));
	float _1233_1332 = res[0];
	float _1033_1330 = res[1];
	float _1132_1231 = res[2];
	float _1032_1230 = res[3];

	lhs_wide[0] = i[1][0]; lhs_wide[1] = i[1][1]; lhs_wide[2] = i[1][2]; lhs_wide[3] = i[1][0]; lhs_wide[4] = i[1][1]; lhs_wide[5] = i[1][3]; lhs_wide[6] = i[1][3]; lhs_wide[7] = i[1][3];
	rhs_wide[0] = i[3][1]; rhs_wide[1] = i[3][3]; rhs_wide[2] = i[2][3]; rhs_wide[3] = i[2][3]; rhs_wide[4] = i[3][0]; rhs_wide[5] = i[3][1]; rhs_wide[6] = i[2][2]; rhs_wide[7] = i[2][0];
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp = _mm_sub_ps(*((__m128 *)res_wide.m256_f32), *((__m128 *)&res_wide.m256_f32[4]));
	float _1031_1130 = res[0];
	float _1133_1331 = res[1];
	float _1223_1322 = res[2];
	float _1023_1320 = res[3];

	lhs_wide[0] = i[1][1]; lhs_wide[1] = i[1][0]; lhs_wide[2] = i[1][0]; lhs_wide[3] = i[1][1];	lhs_wide[4] = i[1][2]; lhs_wide[5] = i[1][2]; lhs_wide[6] = i[1][1]; lhs_wide[7] = i[1][3];
	rhs_wide[0] = i[2][2]; rhs_wide[1] = i[2][2]; rhs_wide[2] = i[2][1]; rhs_wide[3] = i[2][3]; rhs_wide[4] = i[2][1]; rhs_wide[5] = i[2][0]; rhs_wide[6] = i[2][0]; rhs_wide[7] = i[2][1];
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp = _mm_sub_ps(*((__m128 *)res_wide.m256_f32), *((__m128 *)&res_wide.m256_f32[4]));
	float _1122_1221 = res[0];
	float _1022_1220 = res[1];
	float _1021_1120 = res[2];
	float _1123_1321 = res[3];

	lhs_wide[0] = i[0][1]; lhs_wide[1] = i[0][2]; lhs_wide[2] = i[0][3]; lhs_wide[3] = i[0][0]; lhs_wide[4] = i[0][2]; lhs_wide[5] = i[0][3]; lhs_wide[6] = i[0][0]; lhs_wide[7] = i[0][1];
	rhs_wide[0] = _2233_2332; rhs_wide[1] = _2133_2331; rhs_wide[2] = _2132_2231; rhs_wide[3] = _2233_2332; rhs_wide[4] = _2033_2330; rhs_wide[5] = _2032_2230; rhs_wide[6] = _2133_2331; rhs_wide[7] = _2033_2330;
	lhs2[0] = i[0][3]; lhs2[1] = i[0][0]; lhs2[2] = i[0][1]; lhs2[3] = i[0][2];
	rhs2[0] = _2031_2130; rhs2[1] = _2132_2231; rhs2[2] = _2032_2230; rhs2[3] = _2031_2130;
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp2p = _mm_mul_ps(*_lhs2p, *_rhs2p);
	C10 = (res_wide.m256_f32[0]
		- res_wide.m256_f32[1]
		+ res_wide.m256_f32[2]);
	C11 = (res_wide.m256_f32[3]
		- res_wide.m256_f32[4]
		+ res_wide.m256_f32[5]);
	C12 = (res_wide.m256_f32[6]
		- res_wide.m256_f32[7]
		+ res2[0]);
	C13 = (res2[1]
		- res2[2]
		+ res2[3]);

	rhs_wide[0] = _1233_1332; rhs_wide[1] = _1133_1331; rhs_wide[2] = _1132_1231; rhs_wide[3] = _1233_1332; rhs_wide[4] = _1033_1330; rhs_wide[5] = _1032_1230; rhs_wide[6] = _1133_1331; rhs_wide[7] = _1033_1330;
	rhs2[0] = _1031_1130; rhs2[1] = _1132_1231; rhs2[2] = _1032_1230; rhs2[3] = _1031_1130;
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp2p = _mm_mul_ps(*_lhs2p, *_rhs2p);
	C20 = (res_wide.m256_f32[0]
		- res_wide.m256_f32[1]
		+ res_wide.m256_f32[2]);
	C21 = (res_wide.m256_f32[3]
		- res_wide.m256_f32[4]
		+ res_wide.m256_f32[5]);
	C22 = (res_wide.m256_f32[6]
		- res_wide.m256_f32[7]
		+ res2[0]);
	C23 = (res2[1]
		- res2[2]
		+ res2[3]);

	rhs_wide[0] = _1223_1322; rhs_wide[1] = _1123_1321; rhs_wide[2] = _1122_1221; rhs_wide[3] = _1223_1322; rhs_wide[4] = _1023_1320; rhs_wide[5] = _1022_1220; rhs_wide[6] = _1123_1321; rhs_wide[7] = _1023_1320;
	rhs2[0] = _1021_1120; rhs2[1] = _1122_1221; rhs2[2] = _1022_1220; rhs2[3] = _1021_1120;
	res_wide = _mm256_mul_ps(*lhs_widep, *rhs_widep);
	*resp2p = _mm_mul_ps(*_lhs2p, *_rhs2p);
	C30 = (res_wide.m256_f32[0]
		- res_wide.m256_f32[1]
		+ res_wide.m256_f32[2]);
	C31 = (res_wide.m256_f32[3]
		- res_wide.m256_f32[4]
		+ res_wide.m256_f32[5]);
	C32 = (res_wide.m256_f32[6]
		- res_wide.m256_f32[7]
		+ res2[0]);
	C33 = (res2[1]
		- res2[2]
		+ res2[3]);

	mat4 adj(C00, -C10, C20, -C30,
		-C01, C11, -C21, C31,
		C02, -C12, C22, -C32,
		-C03, C13, -C23, C33);

	return adj / Det;
}
mat4 mat4::Transpose()
{
	mat4 o;
	o.i[0][0] = i[0][0]; o.i[0][1] = i[1][0]; o.i[0][2] = i[2][0]; o.i[0][3] = i[3][0];
	o.i[1][0] = i[0][1]; o.i[1][1] = i[1][1]; o.i[1][2] = i[2][1]; o.i[1][3] = i[3][1];
	o.i[2][0] = i[0][2]; o.i[2][1] = i[1][2]; o.i[2][2] = i[2][2]; o.i[2][3] = i[3][2];
	o.i[3][0] = i[0][3]; o.i[3][1] = i[1][3]; o.i[3][2] = i[2][3]; o.i[3][3] = i[3][3];
	return o;
}
mat4 HIGHOMEGA::MATH::mat4::DirectionTransform()
{
	mat4 retVal;
	retVal.Ident();
	vec3 c1 = cross(vec3(i[0][1], i[1][1], i[2][1]), vec3(i[0][2], i[1][2], i[2][2])).normalized();
	vec3 c2 = cross(vec3(i[0][2], i[1][2], i[2][2]), vec3(i[0][0], i[1][0], i[2][0])).normalized();
	vec3 c3 = cross(vec3(i[0][0], i[1][0], i[2][0]), vec3(i[0][1], i[1][1], i[2][1])).normalized();

	retVal.i[0][0] = c1.x;
	retVal.i[1][0] = c1.y;
	retVal.i[2][0] = c1.z;

	retVal.i[0][1] = c2.x;
	retVal.i[1][1] = c2.y;
	retVal.i[2][1] = c2.z;

	retVal.i[0][2] = c3.x;
	retVal.i[1][2] = c3.y;
	retVal.i[2][2] = c3.z;

	return retVal;
}
void mat4::Ident()
{
	i[0][0] = 1; i[0][1] = 0; i[0][2] = 0; i[0][3] = 0;
	i[1][0] = 0; i[1][1] = 1; i[1][2] = 0; i[1][3] = 0;
	i[2][0] = 0; i[2][1] = 0; i[2][2] = 1; i[2][3] = 0;
	i[3][0] = 0; i[3][1] = 0; i[3][2] = 0; i[3][3] = 1;
}

bool HIGHOMEGA::MATH::WithinAABB(vec3 & p1, vec3 & p2, float eps)
{
	vec3 pdiff = p2 - p1;
	return (fabs(pdiff.x) < eps && fabs(pdiff.y) < eps && fabs(pdiff.z) < eps);
}

vec2 HIGHOMEGA::MATH::operator*(const float n, const vec2& in)
{
	vec2 out = in;
	return out*n;
}

vec3 HIGHOMEGA::MATH::operator*(const float n, const vec3& in)
{
	vec3 out = in;
	return out*n;
}

mat4 HIGHOMEGA::MATH::operator*(const float n, const mat4& in)
{
	mat4 out = in;
	return out*n;
}

vec3 HIGHOMEGA::MATH::cross(vec3 a, vec3 b)
{
	return vec3(a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x);
}

vec2 HIGHOMEGA::MATH::toSpherical(vec3 inpVec)
{
	vec3 inpVecOnXZ = vec3(inpVec.x, 0.0f, inpVec.z).normalized();
	float phi = acosf(inpVecOnXZ.x);
	if (inpVecOnXZ.z < 0.0f) phi = 2.0f * HIGHOMEGA_PI - phi;
	float theta = acosf(inpVec.y);
	return vec2(phi * HIGHOMEGA_INV_2PI, theta * HIGHOMEGA_INV_PI);
}

vec3 HIGHOMEGA::MATH::toCartesian(vec2 inpVec)
{
	float phi = inpVec.x * (2.0f * HIGHOMEGA_PI);
	float theta = inpVec.y * HIGHOMEGA_PI;
	float sinTheta = sinf(theta);
	return vec3(cosf(phi) * sinTheta, cosf(theta), sinf(phi) * sinTheta);
}

unsigned int HIGHOMEGA::MATH::toZSignXY(vec3 inpVec)
{
	unsigned int retVal = (((unsigned int)((inpVec.x + 1.0f) * 16383.0f) << 16) | (unsigned int)((inpVec.y + 1.0f) * 32767.0f));
	if (inpVec.z < 0.0f) retVal |= 0x80000000u;
	return retVal;
}

vec3 HIGHOMEGA::MATH::fromZSignXY(unsigned int inpPack)
{
	vec3 retVal;
	retVal.x = ((float)((inpPack & 0x7FFF0000u) >> 16) / 32766.0f) * 2.0f - 1.0f;
	retVal.y = ((float)(inpPack & 0x0000FFFFu) / 65534.0f) * 2.0f - 1.0f;
	vec2 xyVec(retVal.x, retVal.y);
	retVal.z = sqrtf(min (max (1.0f - (xyVec * xyVec), 0.0f), 1.0f));
	if (inpPack & 0x80000000u) retVal.z = -retVal.z;
	return retVal;
}

float HIGHOMEGA::MATH::packSpherical(vec2 & inpSph)
{
	unsigned int retVal = (((unsigned int)(inpSph.x * 65535.0f) & 0x0000FFFF) << 16) | ((unsigned int)(inpSph.y * 65535.0f) & 0x0000FFFF);
	return *((float *)&retVal);
}

vec2 HIGHOMEGA::MATH::unpackSpherical(float inpPack)
{
	unsigned int packInt = *((unsigned int *)&inpPack);
	vec2 retVal;
	retVal.x = (float)((packInt & 0xFFFF0000) >> 16) / 65535.0f;
	retVal.y = (float)(packInt & 0x0000FFFF) / 65535.0f;
	return retVal;
}

float HIGHOMEGA::MATH::packColor(vec3 & color)
{
	vec3 colorNorm = color * 255.0f;
	unsigned int retVal = (((unsigned char)colorNorm.x) << 24) | (((unsigned char)colorNorm.y) << 16) | (((unsigned char)colorNorm.z) << 8);
	return *((float *)&retVal);
}

void HIGHOMEGA::MATH::unpackColor(float inpPack, vec3 & color)
{
	unsigned int packInt = *((unsigned int *)&inpPack);
	color.x = (float)((packInt & 0xFF000000) >> 24) / 255.0f;
	color.y = (float)((packInt & 0x00FF0000) >> 16) / 255.0f;
	color.z = (float)((packInt & 0x0000FF00) >> 8) / 255.0f;
}

unsigned short HIGHOMEGA::MATH::toFP16(float f)
{
	__declspec(align(32)) float src[8] = {f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
	__declspec(align(16)) unsigned short dst[8];
	_mm_storeu_si128((__m128i*)dst, _mm256_cvtps_ph(_mm256_loadu_ps(src), 0));
	return dst[0];
}

float HIGHOMEGA::MATH::toFP16(vec2 v)
{
	__declspec(align(32)) float src[8] = { v.x, v.y, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	__declspec(align(16)) unsigned short dst[8];
	_mm_storeu_si128((__m128i*)dst, _mm256_cvtps_ph(_mm256_loadu_ps(src), 0));
	return *((float *)dst);
}

float HIGHOMEGA::MATH::toFP32(unsigned short h)
{
	__declspec(align(16)) unsigned short src[8] = { h, 0u, 0u, 0u, 0u, 0u, 0u, 0u };
	__declspec(align(32)) float dst[8];
	_mm256_storeu_ps(dst, _mm256_cvtph_ps(_mm_loadu_si128((__m128i*)src)));
	return dst[0];
}

vec2 HIGHOMEGA::MATH::toFP32(float hv2)
{
	__declspec(align(16)) unsigned short src[8];
	__declspec(align(32)) float dst[8];
	*((float *)src) = hv2;
	_mm256_storeu_ps(dst, _mm256_cvtph_ps(_mm_loadu_si128((__m128i*)src)));
	return vec2(dst[0], dst[1]);
}

vec3 HIGHOMEGA::MATH::Slerp(vec3 currentVec, vec3 targetVec, float smoothingFactor)
{
	float dotProd = targetVec * currentVec;
	float Omega = acosf(dotProd);
	float sinOmega = sinf(Omega);
	if (dotProd > 0.999f)
		return (1.0f - smoothingFactor) * currentVec + smoothingFactor * targetVec;
	else
		return (sinf((1.0f - smoothingFactor) * Omega) / sinOmega) * currentVec + (sinf(smoothingFactor * Omega) / sinOmega) * targetVec;
}

float HIGHOMEGA::MATH::Lerp(float a, float b, float lerpFactor)
{
	return a + lerpFactor * (b - a);
}

vec3 HIGHOMEGA::MATH::QuadraticBezier(vec3 p0, vec3 p1, vec3 p2, float t)
{
	float _1_t_sq = 1.0f - t;
	_1_t_sq *= _1_t_sq;

	float t_sq = t * t;

	return p1 + _1_t_sq * (p0 - p1) + t_sq * (p2 - p1);
}

void HIGHOMEGA::MATH::GetTangentBiTangent(vec3 v[3], vec2 tex[3], vec3 & x_1_out, vec3 & y_1_out)
{
	vec3 A = v[0] - v[1], B = v[2] - v[1];
	vec2 v1 = tex[0] - tex[1], v2 = tex[2] - tex[1], v3;
	vec3 origin, x_1, y_1;
	float a, b;

	float v1xv2y_v1yv2x = v1.x*v2.y - v1.y*v2.x;
	float inv_v1xv2y_v1yv2x;
	if (v1xv2y_v1yv2x != 0.0) inv_v1xv2y_v1yv2x = 1.0f / v1xv2y_v1yv2x;

	v3 = -tex[1];
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;
	origin = v[1] + a * A + b * B;

	v3 = vec2(1, 0) - tex[1];
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;

	x_1 = v[1] + a * A + b * B;

	v3 = vec2(0, 1) - tex[1];
	if (v1xv2y_v1yv2x == 0.0)
		a = 1.0;
	else
		a = (v3.x*v2.y - v3.y*v2.x) * inv_v1xv2y_v1yv2x;
	if (v2.x != 0.0)
		b = (v3.x - a * v1.x) / v2.x;
	else
		b = (v3.y - a * v1.y) / v2.y;

	y_1 = v[1] + a * A + b * B;

	x_1_out = x_1 - origin;
	y_1_out = y_1 - origin;
}

void HIGHOMEGA::MATH::GetSideNormals(vec3 & e1, vec3 & e2, vec3 & e3, vec3 & n, vec3 & s1, vec3 & s2, vec3 & s3)
{
	vec3 e2_e1 = e2 - e1;
	vec3 e3_e2 = e3 - e2;
	vec3 e1_e3 = e1 - e3;

	s1 = cross(n, e2_e1).normalized();
	if (s1 * e1_e3 < 0.0f) s1 = -s1;

	s2 = cross(n, e3_e2).normalized();
	if (s2 * e2_e1 < 0.0f) s2 = -s2;

	s3 = cross(n, e1_e3).normalized();
	if (s3 * e3_e2 < 0.0f) s3 = -s3;
}

vec2 HIGHOMEGA::MATH::Lerp(vec2 a, vec2 b, float lerpFactor)
{
	return a + lerpFactor * (b - a);
}

vec3 HIGHOMEGA::MATH::Lerp(vec3 a, vec3 b, float lerpFactor)
{
	return a + lerpFactor * (b - a);
}

float HIGHOMEGA::MATH::SmootherstepFast(float inp)
{
	return inp * inp * inp * (inp * (inp * 6.0f - 15.0f) + 10.0f);
}

vec3 HIGHOMEGA::MATH::Spin(vec3 axis, vec3 vec, float angle)
{
	vec3 axis_n = axis.normalized();
	float shadow = axis_n*vec;
	vec3 bring_down = shadow*axis_n;
	vec3 plane_pt = vec - bring_down;

	vec3 X = plane_pt.normalized();
	vec3 Y = cross(axis_n, X);

	float pt_x = plane_pt*X;
	float new_pt_x = cosf(angle)*pt_x;
	float new_pt_y = sinf(angle)*pt_x;

	return ((new_pt_x)*X) + ((new_pt_y)*Y) + bring_down;
}

float HIGHOMEGA::MATH::sign(float in)
{
	if (in > 0.0) return 1.0;
	if (in == 0.0) return 0.0;
	return -1.0;
}

float HIGHOMEGA::MATH::signEpsilon(float in)
{
	if (in > 0.00001f) return 1.0f;
	else if (in < -0.00001f) return -1.0f;
	else return 0.0f;
}

void HIGHOMEGA::MATH::ShiftLeft(unsigned int & num1, unsigned int & num2, unsigned int & num3, unsigned int & num4, int amount)
{
	__m128i num4num3num2num1 = _mm_set_epi32(num4, num3, num2, num1), res;
	res = _mm_slli_epi32(num4num3num2num1, amount);
	num1 = res.m128i_u32[0];
	num2 = res.m128i_u32[1];
	num3 = res.m128i_u32[2];
	num4 = res.m128i_u32[3];
}

void HIGHOMEGA::MATH::ShiftLeft(unsigned int & num1, unsigned int & num2, unsigned int & num3, unsigned int & num4, int amount1, int amount2, int amount3, int amount4)
{
	__m128i num4num3num2num1 = _mm_set_epi32(num4, num3, num2, num1), count4count3count2count1 = _mm_set_epi32(amount4, amount3, amount2, amount1), res;
	res = _mm_sllv_epi32(num4num3num2num1, count4count3count2count1);
	num1 = res.m128i_u32[0];
	num2 = res.m128i_u32[1];
	num3 = res.m128i_u32[2];
	num4 = res.m128i_u32[3];
}

void HIGHOMEGA::MATH::And(unsigned int & num1, unsigned int & num2, unsigned int & num3, unsigned int & num4, int amount)
{
	__m128 num4num3num2num1 = _mm_set_ps(*((float *)&num4), *((float *)&num3), *((float *)&num2), *((float *)&num1));
	__m128 mask4mask3mask2mask1 = _mm_set_ps(*((float *)&amount), *((float *)&amount), *((float *)&amount), *((float *)&amount)), res;
	res = _mm_and_ps(num4num3num2num1, mask4mask3mask2mask1);
	num1 = res.m128_u32[0];
	num2 = res.m128_u32[1];
	num3 = res.m128_u32[2];
	num4 = res.m128_u32[3];
}

/*

[==>Since the LCP based approach is not being used the Gauss-Jordan solver,  <==]
[==>the Dantzig-based LCP solver and the PGS LCP solver are of no use.       <==]
[==>either way they're kept here.                                            <==]

float LCP_M[500][500];
float LCP_z[500];
float LCP_Q[500];
float LCP_hi[500];

void Gauss_Seidel_LCP (int n)
{
for (int i = 0;i != n;i++) // We're gonna solve Mz=-Q for Z with warm starting
LCP_Q[i] = -LCP_Q[i];
for (int count = 0;count != 10;count++) // Instead of using stopping criteria, we'll just use 10 iterations
{
for (int i = 0;i != n;i++)
{
float delta = 0;
for (int j = 0;j != i;j++)
delta += LCP_M[i][j]*LCP_z[j];
for (int j = i+1;j != n;j++)
delta += LCP_M[i][j]*LCP_z[j];
LCP_z[i] = (LCP_Q[i] - delta)/LCP_M[i][i];
if ( LCP_z[i] > 1000.0 ) LCP_z[i] = 1000.0;
if ( LCP_z[i] < 0.0 ) LCP_z[i] = 0.0;
}
}
}

static float A11[1000][1000];
static float v1[1000];
bool GJSolve (int n)
{
for (int i = 0;i != n;i++) // Gauss
{
float div = A11[i][i];
if ( div == 0.0 )
{
if ( i < n-1 )
{
int j;
for (j = i+1;A11[j][i] == 0.0 && j != n-1;j++) {}
if ( j == n-1 && A11[j][i] == 0.0 ) return false;
else
{
for (int k = 0;k != n;k++)
A11[i][k] += A11[j][k];
div = A11[i][i];
}
}
else
return false;
}
for (int j = i;j != n;j++)
A11[i][j] /= div;
v1[i] /= div;
for (int j = i+1;j != n;j++)
{
float f = A11[j][i];
for (int k = i;k != n;k++)
A11[j][k] -= f*A11[i][k];
v1[j] -= f*v1[i];
}
}
for (int i = n-1;i != -1;i--) // Jordan
{
for (int j = i-1;j != -1;j--)
{
float f = A11[j][i];
for (int k = i;k != n;k++)
A11[j][k] -= f*A11[i][k];
v1[j] -= f*v1[i];
}
}
return true;
}

// Dantzig-based solver (by Baraff)
float LCP_M[1000][1000],LCP_Q[1000];
float LCP_f[1000];
static float a[1000];
static float da[1000];
static float df[1000];
static int type[1000];
int SolveLCP (int n)
{
int d;
int nc,c,id;

bool allpos = false;

for (int i = 0;i != n;i++)
{
LCP_f[i] = 0;
a[i] = LCP_Q[i];
type[i] = 0;
}
nc = c = 0;
id = n;

int cycles = 0;
bool bad = false;
while (!allpos) {

bool done = false;

d = -1;
for (int i = 0;i != n;i++)
if ( a[i] < -ZERO ) {d = i;break;}
if ( d == -1 )
{allpos = true;done = true;}

while (!done)
{
cycles++;
if ( cycles > n*50 ) {done = true;allpos = true;bad = true;}

for (int i = 0;i != n;i++)
df[i] = 0;
df[d] = 1;

int x = 0,y = 0;
for (int i = 0;i != n;i++)
{
if ( type [i] == 1 )
{
for (int j = 0;j != n;j++)
if ( type [j] == 1 ) {A11[x][y] = LCP_M[i][j];y++;}
y = 0;
x++;
}
}
x = 0;
for (int i = 0;i != n;i++)
if ( type [i] == 1 ) {v1[x] = -LCP_M[i][d];x++;}

if ( GJSolve (c) == false ) {done = true;allpos = true;bad = true;}

x = 0;
for (int i = 0;i != n;i++)
if ( type[i] == 1 ) {df[i] = v1[x];x++;}

for (int i = 0;i != n;i++)
{
da[i] = 0;
for (int j = 0;j != n;j++)
da[i] += LCP_M[i][j]*df[j];
}

float s = 1000000000,s_;
int j = -1;
if ( da[d] > ZERO )
{
j = d;
s = -a[d]/da[d];
}
for (int i = 0;i != n;i++)
if ( type[i] == 1 && df[i] < -ZERO )
{
s_ = -LCP_f[i]/df[i];
if ( s_ < s )
{
s = s_;
j = i;
}
}
for (int i = 0;i != n;i++)
if ( type[i] == -1 && da[i] < -ZERO )
{
s_ = -a[i]/da[i];
if (s_ < s)
{
s = s_;
j = i;
}
}

for (int i = 0;i != n;i++)
{
LCP_f[i] += s*df[i];
a[i] += s*da[i];
}

if ( j != -1 )
{
if ( type[j] == 1 )
{
type[j] = -1;
nc++;
c--;
}
else
if ( type[j] == -1 )
{
type[j] = 1;
nc--;
c++;
}
else
{
type[j] = 1;
id--;
c++;
done = true;
}
}
else
{done = true;allpos = true;bad = true;}

}
}

if ( bad == true ) return -1;

return 0;
}

[==>end of commented out unused solver stuff<==]

*/

#ifndef min
#define min(a,b) (((a)<(b)) ? (a) : (b))
#endif

#ifndef max
#define max(a,b) (((a)>(b)) ? (a) : (b))
#endif

#define Min(x,y,z) min (min(x,y),z)
#define Max(x,y,z) max (max(x,y),z)

vec3 HIGHOMEGA::MATH::ReflectPos(vec3 i, vec3 n, vec3 p)
{
	vec3 tmp = i - p;
	tmp = tmp - (2.0f*(tmp*n))*n;
	return tmp + p;
}

vec3 HIGHOMEGA::MATH::ReflectDir(vec3 i, vec3 n)
{
	return i - (2.0f*(i*n))*n;
}

mat4 HIGHOMEGA::MATH::RotationX(float deg)
{
	mat4 tmp;
	tmp.Ident();
	float sin_deg = sinf(deg);
	float cos_deg = cosf(deg);

	tmp.i[1][1] = cos_deg;
	tmp.i[1][2] = -sin_deg;

	tmp.i[2][1] = sin_deg;
	tmp.i[2][2] = cos_deg;

	return tmp;
}

mat4 HIGHOMEGA::MATH::RotationY(float deg)
{
	mat4 tmp;
	tmp.Ident();
	float sin_deg = sinf(deg);
	float cos_deg = cosf(deg);

	tmp.i[0][0] = cos_deg;
	tmp.i[0][2] = -sin_deg;

	tmp.i[2][0] = sin_deg;
	tmp.i[2][2] = cos_deg;

	return tmp;
}

mat4 HIGHOMEGA::MATH::RotationZ(float deg)
{
	mat4 tmp;
	tmp.Ident();
	float sin_deg = sinf(deg);
	float cos_deg = cosf(deg);

	tmp.i[0][0] = cos_deg;
	tmp.i[0][1] = -sin_deg;

	tmp.i[1][0] = sin_deg;
	tmp.i[1][1] = cos_deg;

	return tmp;
}
