#include "Quaternion.h"

Quaternion::Quaternion() :
	x(0),
	y(0),
	z(0),
	w(1)
{
}

Quaternion::Quaternion(const Quaternion& rhs) :
	x(rhs.x),
	y(rhs.y),
	z(rhs.z),
	w(rhs.w)
{
}

Quaternion::Quaternion(float x, float y, float z, float w) :
	x(x),
	y(y),
	z(z),
	w(w)
{
}

Quaternion::Quaternion(const Matrix<4,1>& m) :
	x(m.m[0][0]),
	y(m.m[1][0]),
	z(m.m[2][0]),
	w(m.m[3][0])
{
}

Quaternion::Quaternion(const vec3& axis, float angle) :
	x(sin(angle/2) * axis.x),
	y(sin(angle/2) * axis.y),
	z(sin(angle/2) * axis.z),
	w(cos(angle/2))
{
}

Quaternion::Quaternion(float roll, float pitch, float yaw) :
	x(cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)),
	y(sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)),
	z(cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)),
	w(cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2))
{
}

float Quaternion::LengthSq() const
{
	return x*x + y*y + z*z + w*w;
}

float Quaternion::Length() const
{
	return sqrt(x*x + y*y + z*z + w*w);
}

void Quaternion::Normalize()
{
	float lengthSq = LengthSq();
	if (lengthSq > 0.0f)
		*this /= sqrt(lengthSq);
}

vec3 Quaternion::GetRollPitchYaw() const
{
	return vec3(GetRoll(), GetPitch(), GetYaw());
}

float Quaternion::GetRoll() const
{
	return atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
}

float Quaternion::GetPitch() const
{
	return asin(2 * (w * y - z * x));
}

float Quaternion::GetYaw() const
{
	return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

Matrix<3,3> Quaternion::GetMatrix() const
{
	Matrix<3,3> m;
	m.m[0][0] = w * w + x * x - y * y - z * z;
	m.m[0][1] = 2 * (x * y - z * w);
	m.m[0][2] = 2 * (x * z + y * w);

	m.m[1][0] = 2 * (x * y + z * w);
	m.m[1][1] = w * w + y * y - z * z - x * x;
	m.m[1][2] = 2 * (y * z - x * w);

	m.m[2][0] = 2 * (x * z - y * w);
	m.m[2][1] = 2 * (y * z + x * w);
	m.m[2][2] = w * w + z * z - x * x - y * y;
	return m;
}

Matrix<3,3> Quaternion::GetDQDX() const
{
	Matrix<3,3> m;
	m.m[0][0] = 2 * x;
	m.m[0][1] = 2 * y;
	m.m[0][2] = 2 * z;

	m.m[1][0] = 2 * y;
	m.m[1][1] = -2 * x;
	m.m[1][2] = -2 * w;

	m.m[2][0] = 2 * z;
	m.m[2][1] = 2 * w;
	m.m[2][2] = -2 * x;
	return m;
}

Matrix<3,3> Quaternion::GetDQDY() const
{
	Matrix<3,3> m;
	m.m[0][0] = -2 * y;
	m.m[0][1] = 2 * x;
	m.m[0][2] = 2 * w;

	m.m[1][0] = 2 * x;
	m.m[1][1] = 2 * y;
	m.m[1][2] = 2 * z;

	m.m[2][0] = -2 * w;
	m.m[2][1] = 2 * z;
	m.m[2][2] = -2 * y;
	return m;
}

Matrix<3,3> Quaternion::GetDQDZ() const
{
	Matrix<3,3> m;
	m.m[0][0] = -2 * z;
	m.m[0][1] = -2 * w;
	m.m[0][2] = 2 * x;

	m.m[1][0] = 2 * w;
	m.m[1][1] = -2 * z;
	m.m[1][2] = 2 * y;

	m.m[2][0] = 2 * x;
	m.m[2][1] = 2 * y;
	m.m[2][2] = 2 * z;
	return m;
}

Matrix<3,3> Quaternion::GetDQDW() const
{
	Matrix<3,3> m;
	m.m[0][0] = 2 * w;
	m.m[0][1] = -2 * z;
	m.m[0][2] = 2 * y;

	m.m[1][0] = 2 * z;
	m.m[1][1] = 2 * w;
	m.m[1][2] = -2 * x;

	m.m[2][0] = -2 * y;
	m.m[2][1] = 2 * x;
	m.m[2][2] = 2 * w;
	return m;
}

vec3 Quaternion::Transform(const vec3& v) const
{
	return vec3::Transform(v, GetMatrix());
}

Quaternion& Quaternion::operator += (const Quaternion& rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	w += rhs.w;
	return *this;
}

Quaternion& Quaternion::operator -= (const Quaternion& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	w -= rhs.w;
	return *this;
}

Quaternion& Quaternion::operator *= (const float rhs)
{
	x *= rhs;
	y *= rhs;
	z *= rhs;
	w *= rhs;
	return *this;
}

Quaternion& Quaternion::operator /= (const float rhs)
{
	x /= rhs;
	y /= rhs;
	z /= rhs;
	w /= rhs;
	return *this;
}
  
Quaternion operator + (const Quaternion& lhs, const Quaternion& rhs)
{
	return Quaternion(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z, lhs.w + rhs.w);
}

Quaternion operator - (const Quaternion& lhs, const Quaternion& rhs)
{
	return Quaternion(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z, lhs.w - rhs.w);
}

Quaternion operator * (const Quaternion& lhs, const Quaternion& rhs)
{
	return Quaternion
	(
		lhs.x * rhs.w + lhs.w * rhs.x + lhs.y * rhs.z - lhs.z * rhs.y,
		lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x,
		lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w,
		lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z
	);
}

Quaternion operator * (const Quaternion& lhs, const float rhs)
{
	return Quaternion(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs, lhs.w * rhs);
}

Quaternion operator * (const float lhs, const Quaternion& rhs)
{
	return Quaternion(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z, lhs * rhs.w);
}

Quaternion operator / (const Quaternion& lhs, const float rhs)
{
	return Quaternion(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs, lhs.w / rhs);
}
