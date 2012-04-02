#ifndef _MATRIX_MATH_H
#define _MATRIX_MATH_H

#include <Core.h>

template <u32 rows, u32 cols>
struct Matrix
{
public:
	static Matrix<rows, cols> Zero();
#if rows == cols
	static Matrix<rows, cols> Identity();
#endif

	f32 SumSq() const;
	f32 Sum() const;

	Matrix<cols, rows> Transpose() const;
	f32 Determinant() const;
	Matrix<rows, cols> Inverse() const;

	Matrix<rows, cols*2> DoubleRight() const { return CatRight(*this, *this); }
	Matrix<rows*2, cols> DoubleDown() const { return CatDown(*this, *this); }
	Matrix<rows*2, cols*2> DoubleDiagonal() const { return CatDiagonal(*this, *this); }

	f32 m[rows][cols];
};

template <u32 rows, u32 cols>
Matrix<rows, cols> Matrix<rows, cols>::Zero()
{
	Matrix<rows, cols> ret;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] = 0.0f;
	return ret;
}

template <u32 rows, u32 cols>
Matrix<rows, cols> Matrix<rows, cols>::Identity()
{
	Matrix<rows, cols> ret;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] = (r == c ? 1.0f : 0.0f);
	return ret;
}

template <u32 rows, u32 cols>
f32 Matrix<rows, cols>::SumSq() const
{
	f32 ret = 0;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret += m[r][c] * m[r][c];
	return ret;
}

template <u32 rows, u32 cols>
f32 Matrix<rows, cols>::Sum() const
{
	f32 ret = 0;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret += m[r][c];
	return ret;
}

template <u32 rows, u32 cols>
Matrix<cols, rows> Matrix<rows, cols>::Transpose() const
{
	Matrix<cols, rows> ret;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[c][r] = m[r][c];
	return ret;
}

template <u32 rows, u32 cols>
Matrix<rows, cols> operator + (const Matrix<rows, cols>& lhs, const Matrix<rows, cols>& rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] += rhs.m[r][c];
	return ret;
}

template <u32 rows, u32 cols>
Matrix<rows, cols> operator - (const Matrix<rows, cols>& lhs, const Matrix<rows, cols>& rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] -= rhs.m[r][c];
	return ret;
}

template <u32 rows, u32 shared, u32 cols>
Matrix<rows, cols> operator * (const Matrix<rows, shared>& lhs, const Matrix<shared, cols>& rhs)
{
	Matrix<rows, cols> ret = Matrix<rows, cols>::Zero();
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			for (u32 i=0; i<shared; ++i)
				ret.m[r][c] += lhs.m[r][i] * rhs.m[i][c];
	return ret;
}

template <u32 rows, u32 cols>
Matrix<rows, cols> operator * (const Matrix<rows, cols>& lhs, f32 rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] *= rhs;
	return ret;
}

template <u32 rows, u32 cols>
Matrix<rows, cols> operator * (f32 lhs, const Matrix<rows, cols>& rhs)
{
	Matrix<rows, cols> ret = rhs;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] *= lhs;
	return ret;
}

template <u32 rows, u32 cols>
Matrix<rows, cols> operator / (const Matrix<rows, cols>& lhs, f32 rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (u32 r=0; r<rows; ++r)
		for (u32 c=0; c<cols; ++c)
			ret.m[r][c] /= rhs;
	return ret;
}

template <u32 rows, u32 cols, u32 cols2>
Matrix<rows, cols+cols2> CatRight(const Matrix<rows, cols>& lhs, const Matrix<rows, cols2>& rhs)
{
	Matrix<rows, cols+cols2> ret;
	for (int r=0; r<rows; ++r)
		for (int c=0; c<cols; ++c)
			ret.m[r][c] = lhs.m[r][c];
	for (int r=0; r<rows; ++r)
		for (int c=0; c<cols2; ++c)
			ret.m[r][c + cols] = rhs.m[r][c];
	return ret;
}

template <u32 rows, u32 rows2, u32 cols>
Matrix<rows+rows2, cols> CatDown(const Matrix<rows, cols>& lhs, const Matrix<rows2, cols>& rhs)
{
	Matrix<rows+rows2, cols> ret;
	for (int r=0; r<rows; ++r)
		for (int c=0; c<cols; ++c)
			ret.m[r][c] = lhs.m[r][c];
	for (int r=0; r<rows2; ++r)
		for (int c=0; c<cols; ++c)
			ret.m[r + rows][c] = rhs.m[r][c];
	return ret;
}

template <u32 rows, u32 cols, u32 rows2, u32 cols2>
Matrix<rows+rows2, cols+cols2> CatDiagonal(const Matrix<rows, cols>& lhs, const Matrix<rows2, cols2>& rhs)
{
	Matrix<rows+rows2, cols+cols2> ret = Matrix<rows+rows2, cols+cols2>::Zero();
	for (int r=0; r<rows; ++r)
		for (int c=0; c<cols; ++c)
			ret.m[r][c] = lhs.m[r][c];
	for (int r=0; r<rows2; ++r)
		for (int c=0; c<cols2; ++c)
			ret.m[r + rows][c + cols] = rhs.m[r][c];
	return ret;
}

#endif
