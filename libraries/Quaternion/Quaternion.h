#ifndef _QUATERNION_H
#define _QUATERNION_H

#include <Core.h>
#include <VectorMath.h>
#include <MatrixMath.h>

struct Quaternion
{
	Quaternion();
	Quaternion(const Quaternion& rhs);
	Quaternion(f32 x, f32 y, f32 z, f32 w);
	Quaternion(const Matrix<4,1>& m);
	Quaternion(const vec3& axis, f32 angle);
	Quaternion(f32 roll, f32 pitch, f32 yaw);

	f32 LengthSq() const;
	f32 Length() const;
	void Normalize();

	vec3 GetRollPitchYaw() const;
	f32 GetRoll() const;
	f32 GetPitch() const;
	f32 GetYaw() const;

	Matrix<3,3> GetMatrix() const;
	Matrix<3,3> GetDQDX() const;
	Matrix<3,3> GetDQDY() const;
	Matrix<3,3> GetDQDZ() const;
	Matrix<3,3> GetDQDW() const;

	vec3 Transform(const vec3& v) const;

	Quaternion& operator += (const Quaternion& rhs);
	Quaternion& operator -= (const Quaternion& rhs);
	Quaternion& operator *= (const f32 rhs);
	Quaternion& operator /= (const f32 rhs);

	f32 x, y, z, w;
};

Quaternion operator + (const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator - (const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator * (const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator * (const Quaternion& lhs, const f32 rhs);
Quaternion operator * (const f32 lhs, const Quaternion& rhs);
Quaternion operator / (const Quaternion& lhs, const f32 rhs);

#endif
