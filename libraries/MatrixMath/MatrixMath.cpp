#include "MatrixMath.h"

template<>
f32 Matrix<2,2>::Determinant() const
{
	return m[0][0] * m[1][1] - m[0][1] * m[1][0];
}

template<>
Matrix<2,2> Matrix<2,2>::Inverse() const
{
	Matrix<2,2> ret;
	ret.m[0][0] = m[0][0];
	ret.m[0][1] = -m[1][0];
	ret.m[1][0] = -m[0][1];
	ret.m[1][1] = m[1][1];
	return ret * (1 / Determinant());
}

template<>
f32 Matrix<4,4>::Determinant() const
{
	return
		m[0][3] * m[1][2] * m[2][1] * m[3][0]-m[0][2] * m[1][3] * m[2][1] * m[3][0]-m[0][3] * m[1][1] * m[2][2] * m[3][0]+m[0][1] * m[1][3] * m[2][2] * m[3][0]+
		m[0][2] * m[1][1] * m[2][3] * m[3][0]-m[0][1] * m[1][2] * m[2][3] * m[3][0]-m[0][3] * m[1][2] * m[2][0] * m[3][1]+m[0][2] * m[1][3] * m[2][0] * m[3][1]+
		m[0][3] * m[1][0] * m[2][2] * m[3][1]-m[0][0] * m[1][3] * m[2][2] * m[3][1]-m[0][2] * m[1][0] * m[2][3] * m[3][1]+m[0][0] * m[1][2] * m[2][3] * m[3][1]+
		m[0][3] * m[1][1] * m[2][0] * m[3][2]-m[0][1] * m[1][3] * m[2][0] * m[3][2]-m[0][3] * m[1][0] * m[2][1] * m[3][2]+m[0][0] * m[1][3] * m[2][1] * m[3][2]+
		m[0][1] * m[1][0] * m[2][3] * m[3][2]-m[0][0] * m[1][1] * m[2][3] * m[3][2]-m[0][2] * m[1][1] * m[2][0] * m[3][3]+m[0][1] * m[1][2] * m[2][0] * m[3][3]+
		m[0][2] * m[1][0] * m[2][1] * m[3][3]-m[0][0] * m[1][2] * m[2][1] * m[3][3]-m[0][1] * m[1][0] * m[2][2] * m[3][3]+m[0][0] * m[1][1] * m[2][2] * m[3][3];
}

template<>
Matrix<4,4> Matrix<4,4>::Inverse() const
{
	Matrix<4,4> ret;
	ret.m[0][0] = m[1][2]*m[2][3]*m[3][1] - m[1][3]*m[2][2]*m[3][1] + m[1][3]*m[2][1]*m[3][2] - m[1][1]*m[2][3]*m[3][2] - m[1][2]*m[2][1]*m[3][3] + m[1][1]*m[2][2]*m[3][3];
	ret.m[0][1] = m[0][3]*m[2][2]*m[3][1] - m[0][2]*m[2][3]*m[3][1] - m[0][3]*m[2][1]*m[3][2] + m[0][1]*m[2][3]*m[3][2] + m[0][2]*m[2][1]*m[3][3] - m[0][1]*m[2][2]*m[3][3];
	ret.m[0][2] = m[0][2]*m[1][3]*m[3][1] - m[0][3]*m[1][2]*m[3][1] + m[0][3]*m[1][1]*m[3][2] - m[0][1]*m[1][3]*m[3][2] - m[0][2]*m[1][1]*m[3][3] + m[0][1]*m[1][2]*m[3][3];
	ret.m[0][3] = m[0][3]*m[1][2]*m[2][1] - m[0][2]*m[1][3]*m[2][1] - m[0][3]*m[1][1]*m[2][2] + m[0][1]*m[1][3]*m[2][2] + m[0][2]*m[1][1]*m[2][3] - m[0][1]*m[1][2]*m[2][3];
	
	ret.m[1][0] = m[1][3]*m[2][2]*m[3][0] - m[1][2]*m[2][3]*m[3][0] - m[1][3]*m[2][0]*m[3][2] + m[1][0]*m[2][3]*m[3][2] + m[1][2]*m[2][0]*m[3][3] - m[1][0]*m[2][2]*m[3][3];
	ret.m[1][1] = m[0][2]*m[2][3]*m[3][0] - m[0][3]*m[2][2]*m[3][0] + m[0][3]*m[2][0]*m[3][2] - m[0][0]*m[2][3]*m[3][2] - m[0][2]*m[2][0]*m[3][3] + m[0][0]*m[2][2]*m[3][3];
	ret.m[1][2] = m[0][3]*m[1][2]*m[3][0] - m[0][2]*m[1][3]*m[3][0] - m[0][3]*m[1][0]*m[3][2] + m[0][0]*m[1][3]*m[3][2] + m[0][2]*m[1][0]*m[3][3] - m[0][0]*m[1][2]*m[3][3];
	ret.m[1][3] = m[0][2]*m[1][3]*m[2][0] - m[0][3]*m[1][2]*m[2][0] + m[0][3]*m[1][0]*m[2][2] - m[0][0]*m[1][3]*m[2][2] - m[0][2]*m[1][0]*m[2][3] + m[0][0]*m[1][2]*m[2][3];
	
	ret.m[2][0] = m[1][1]*m[2][3]*m[3][0] - m[1][3]*m[2][1]*m[3][0] + m[1][3]*m[2][0]*m[3][1] - m[1][0]*m[2][3]*m[3][1] - m[1][1]*m[2][0]*m[3][3] + m[1][0]*m[2][1]*m[3][3];
	ret.m[2][1] = m[0][3]*m[2][1]*m[3][0] - m[0][1]*m[2][3]*m[3][0] - m[0][3]*m[2][0]*m[3][1] + m[0][0]*m[2][3]*m[3][1] + m[0][1]*m[2][0]*m[3][3] - m[0][0]*m[2][1]*m[3][3];
	ret.m[2][2] = m[0][1]*m[1][3]*m[3][0] - m[0][3]*m[1][1]*m[3][0] + m[0][3]*m[1][0]*m[3][1] - m[0][0]*m[1][3]*m[3][1] - m[0][1]*m[1][0]*m[3][3] + m[0][0]*m[1][1]*m[3][3];
	ret.m[2][3] = m[0][3]*m[1][1]*m[2][0] - m[0][1]*m[1][3]*m[2][0] - m[0][3]*m[1][0]*m[2][1] + m[0][0]*m[1][3]*m[2][1] + m[0][1]*m[1][0]*m[2][3] - m[0][0]*m[1][1]*m[2][3];

	ret.m[3][0] = m[1][2]*m[2][1]*m[3][0] - m[1][1]*m[2][2]*m[3][0] - m[1][2]*m[2][0]*m[3][1] + m[1][0]*m[2][2]*m[3][1] + m[1][1]*m[2][0]*m[3][2] - m[1][0]*m[2][1]*m[3][2];
	ret.m[3][1] = m[0][1]*m[2][2]*m[3][0] - m[0][2]*m[2][1]*m[3][0] + m[0][2]*m[2][0]*m[3][1] - m[0][0]*m[2][2]*m[3][1] - m[0][1]*m[2][0]*m[3][2] + m[0][0]*m[2][1]*m[3][2];
	ret.m[3][2] = m[0][2]*m[1][1]*m[3][0] - m[0][1]*m[1][2]*m[3][0] - m[0][2]*m[1][0]*m[3][1] + m[0][0]*m[1][2]*m[3][1] + m[0][1]*m[1][0]*m[3][2] - m[0][0]*m[1][1]*m[3][2];
	ret.m[3][3] = m[0][1]*m[1][2]*m[2][0] - m[0][2]*m[1][1]*m[2][0] + m[0][2]*m[1][0]*m[2][1] - m[0][0]*m[1][2]*m[2][1] - m[0][1]*m[1][0]*m[2][2] + m[0][0]*m[1][1]*m[2][2];
	return ret * (1 / Determinant());
}