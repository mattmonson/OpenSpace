#ifndef _QUATERNION_H
#define _QUATERNION_H

#include <Core.h>
#include <VectorMath.h>
#include <MatrixMath.h>

struct Quaternion
{
	Quaternion();
	Quaternion(const Quaternion& rhs);
	Quaternion(float x, float y, float z, float w);
	Quaternion(const Matrix<4,1>& m);
	Quaternion(const vec3& axis, float angle);
	Quaternion(float roll, float pitch, float yaw);

	float LengthSq() const;
	float Length() const;
	void Normalize();

	vec3 GetRollPitchYaw() const;
	float GetRoll() const;
	float GetPitch() const;
	float GetYaw() const;

	Matrix<3,3> GetMatrix() const;
	Matrix<3,3> GetDQDX() const;
	Matrix<3,3> GetDQDY() const;
	Matrix<3,3> GetDQDZ() const;
	Matrix<3,3> GetDQDW() const;

	vec3 Transform(const vec3& v) const;

	Quaternion& operator += (const Quaternion& rhs);
	Quaternion& operator -= (const Quaternion& rhs);
	Quaternion& operator *= (const float rhs);
	Quaternion& operator /= (const float rhs);

	float x, y, z, w;
};

Quaternion operator + (const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator - (const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator * (const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator * (const Quaternion& lhs, const float rhs);
Quaternion operator * (const float lhs, const Quaternion& rhs);
Quaternion operator / (const Quaternion& lhs, const float rhs);

#endif
