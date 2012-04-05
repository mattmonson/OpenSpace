#ifndef _MATRIX_MATH_H
#define _MATRIX_MATH_H

#include <Core.h>

template <uint32_t rows, uint32_t cols>
struct Matrix
{
public:
	static Matrix<rows, cols> Zero();
#if rows == cols
	static Matrix<rows, cols> Identity();
#endif

	float SumSq() const;
	float Sum() const;

	Matrix<cols, rows> Transpose() const;
	float Determinant() const;
	Matrix<rows, cols> Inverse() const;

	Matrix<rows, cols*2> DoubleRight() const { return CatRight(*this, *this); }
	Matrix<rows*2, cols> DoubleDown() const { return CatDown(*this, *this); }
	Matrix<rows*2, cols*2> DoubleDiagonal() const { return CatDiagonal(*this, *this); }

	float m[rows][cols];
};

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> Matrix<rows, cols>::Zero()
{
	Matrix<rows, cols> ret;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] = 0.0f;
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> Matrix<rows, cols>::Identity()
{
	Matrix<rows, cols> ret;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] = (r == c ? 1.0f : 0.0f);
	return ret;
}

template <uint32_t rows, uint32_t cols>
float Matrix<rows, cols>::SumSq() const
{
	float ret = 0;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret += m[r][c] * m[r][c];
	return ret;
}

template <uint32_t rows, uint32_t cols>
float Matrix<rows, cols>::Sum() const
{
	float ret = 0;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret += m[r][c];
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<cols, rows> Matrix<rows, cols>::Transpose() const
{
	Matrix<cols, rows> ret;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[c][r] = m[r][c];
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> operator + (const Matrix<rows, cols>& lhs, const Matrix<rows, cols>& rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] += rhs.m[r][c];
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> operator - (const Matrix<rows, cols>& lhs, const Matrix<rows, cols>& rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] -= rhs.m[r][c];
	return ret;
}

template <uint32_t rows, uint32_t shared, uint32_t cols>
Matrix<rows, cols> operator * (const Matrix<rows, shared>& lhs, const Matrix<shared, cols>& rhs)
{
	Matrix<rows, cols> ret = Matrix<rows, cols>::Zero();
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			for (uint32_t i=0; i<shared; ++i)
				ret.m[r][c] += lhs.m[r][i] * rhs.m[i][c];
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> operator * (const Matrix<rows, cols>& lhs, float rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] *= rhs;
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> operator * (float lhs, const Matrix<rows, cols>& rhs)
{
	Matrix<rows, cols> ret = rhs;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] *= lhs;
	return ret;
}

template <uint32_t rows, uint32_t cols>
Matrix<rows, cols> operator / (const Matrix<rows, cols>& lhs, float rhs)
{
	Matrix<rows, cols> ret = lhs;
	for (uint32_t r=0; r<rows; ++r)
		for (uint32_t c=0; c<cols; ++c)
			ret.m[r][c] /= rhs;
	return ret;
}

template <uint32_t rows, uint32_t cols, uint32_t cols2>
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

template <uint32_t rows, uint32_t rows2, uint32_t cols>
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

template <uint32_t rows, uint32_t cols, uint32_t rows2, uint32_t cols2>
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
