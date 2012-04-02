#ifndef _VECTOR_MATH_H
#define _VECTOR_MATH_H

#include <Core.h>
#include <MatrixMath.h>

struct vec3
{
	static vec3 Transform(const vec3& lhs, const Matrix<3,3>& rhs);

	vec3();
	vec3(const vec3& rhs);
	vec3(f32 x, f32 y, f32 z);
	vec3(const Matrix<3,1>& m);

	f32 LengthSq() const;
	f32 Length() const;
	void Normalize();

	Matrix<3,1> ToMatrix() const;

	vec3& operator += (const vec3& rhs);
	vec3& operator -= (const vec3& rhs);
	vec3& operator *= (const f32 rhs);
	vec3& operator /= (const f32 rhs);

	f32 x, y, z;
};

vec3 operator + (const vec3& lhs, const vec3& rhs);
vec3 operator - (const vec3& lhs, const vec3& rhs);
vec3 operator * (const vec3& lhs, const f32 rhs);
vec3 operator * (const f32 lhs, const vec3& rhs);
vec3 operator / (const vec3& lhs, const f32 rhs);

#endif
