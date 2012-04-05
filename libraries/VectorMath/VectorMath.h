#ifndef _VECTOR_MATH_H
#define _VECTOR_MATH_H

#include <Core.h>
#include <MatrixMath.h>

struct vec3
{
	static vec3 Transform(const vec3& lhs, const Matrix<3,3>& rhs);

	vec3();
	vec3(const vec3& rhs);
	vec3(float x, float y, float z);
	vec3(const Matrix<3,1>& m);

	float LengthSq() const;
	float Length() const;
	void Normalize();

	Matrix<3,1> ToMatrix() const;

	vec3& operator += (const vec3& rhs);
	vec3& operator -= (const vec3& rhs);
	vec3& operator *= (const float rhs);
	vec3& operator /= (const float rhs);

	float x, y, z;
};

vec3 operator + (const vec3& lhs, const vec3& rhs);
vec3 operator - (const vec3& lhs, const vec3& rhs);
vec3 operator * (const vec3& lhs, const float rhs);
vec3 operator * (const float lhs, const vec3& rhs);
vec3 operator / (const vec3& lhs, const float rhs);

#endif
