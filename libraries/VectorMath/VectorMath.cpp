#include "VectorMath.h"

vec3 vec3::Transform(const vec3& lhs, const Matrix<3,3>& rhs)
{
	return vec3
	(
		lhs.x * rhs.m[0][0] + lhs.y * rhs.m[0][1] + lhs.z * rhs.m[0][2],
		lhs.x * rhs.m[1][0] + lhs.y * rhs.m[1][1] + lhs.z * rhs.m[1][2],
		lhs.x * rhs.m[2][0] + lhs.y * rhs.m[2][1] + lhs.z * rhs.m[2][2]
	);
}

vec3::vec3() :
	x(0),
	y(0),
	z(0)
{
}

vec3::vec3(const vec3& rhs) :
	x(rhs.x),
	y(rhs.y),
	z(rhs.z)
{
}

vec3::vec3(float x, float y, float z) :
	x(x),
	y(y),
	z(z)
{
}

vec3::vec3(const Matrix<3,1>& m) :
	x(m.m[0][0]),
	y(m.m[1][0]),
	z(m.m[2][0])
{
}

float vec3::LengthSq() const
{
	return x*x + y*y + z*z;
}

float vec3::Length() const
{
	return sqrt(x*x + y*y + z*z);
}

void vec3::Normalize()
{
	float lengthSq = LengthSq();
	if (lengthSq > 0.0f)
		*this /= sqrt(lengthSq);
}

Matrix<3,1> vec3::ToMatrix() const
{
	Matrix<3,1> ret;
	ret.m[0][0] = x;
	ret.m[1][0] = y;
	ret.m[2][0] = z;
	return ret;
}

vec3& vec3::operator += (const vec3& rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

vec3& vec3::operator -= (const vec3& rhs)
{
	x -= rhs.x;
	y -= rhs.y;
	z -= rhs.z;
	return *this;
}

vec3& vec3::operator *= (const float rhs)
{
	x *= rhs;
	y *= rhs;
	z *= rhs;
	return *this;
}

vec3& vec3::operator /= (const float rhs)
{
	x /= rhs;
	y /= rhs;
	z /= rhs;
	return *this;
}
  
vec3 operator + (const vec3& lhs, const vec3& rhs)
{
	return vec3(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

vec3 operator - (const vec3& lhs, const vec3& rhs)
{
	return vec3(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

vec3 operator * (const vec3& lhs, const float rhs)
{
	return vec3(lhs.x * rhs, lhs.y * rhs, lhs.z * rhs);
}

vec3 operator * (const float lhs, const vec3& rhs)
{
	return vec3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z);
}

vec3 operator / (const vec3& lhs, const float rhs)
{
	return vec3(lhs.x / rhs, lhs.y / rhs, lhs.z / rhs);
}
