/*
 * This file is part of EIBA.
 *
 * Copyright (C) 2017 Zhejiang University
 * For more information see <https://github.com/ZJUCVG/EIBA>
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef _MATRIX_NxN_H_
#define _MATRIX_NxN_H_

#include "LinearSystem.h"
#include "Matrix4x4.h"
#include "Matrix6x6.h"
#include "Matrix8x8.h"
#include "VectorN.h"

namespace LA
{
template <int N> class MatrixNxNf
{
  public:
    inline const float *operator[](const int i) const { return m_data[i]; }
    inline float *operator[](const int i) { return m_data[i]; }
    inline MatrixNxNf<N> operator-(const MatrixNxNf<N> &B) const
    {
        MatrixNxNf<N> AmB;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) AmB[i][j] = m_data[i][j] - B[i][j];
        return AmB;
    }

    template <class MATRIX>
    inline void SetBlockDiagonalUpper(const int i, const MATRIX &M);
    template <>
    inline void
    SetBlockDiagonalUpper<SymmetricMatrix3x3f>(const int i,
                                               const SymmetricMatrix3x3f &M)
    {
        int k = i;
        memcpy(m_data[k] + k, &M.m00(), 12);
        ++k;
        memcpy(m_data[k] + k, &M.m11(), 8);
        ++k;
        m_data[k][k] = M.m22();
    }
    template <>
    inline void
    SetBlockDiagonalUpper<SymmetricMatrix8x8f>(const int i,
                                               const SymmetricMatrix8x8f &M)
    {
        int k = i;
        memcpy(m_data[k] + k, &M.m00(), 32);
        ++k;
        memcpy(m_data[k] + k, &M.m11(), 28);
        ++k;
        memcpy(m_data[k] + k, &M.m22(), 24);
        ++k;
        memcpy(m_data[k] + k, &M.m33(), 20);
        ++k;
        memcpy(m_data[k] + k, &M.m44(), 16);
        ++k;
        memcpy(m_data[k] + k, &M.m55(), 12);
        ++k;
        memcpy(m_data[k] + k, &M.m66(), 8);
        ++k;
        m_data[k][k] = M.m77();
    }

    template <class MATRIX>
    inline void SetBlock(const int i, const int j, const MATRIX &M);
    template <>
    inline void SetBlock<Matrix3x3f>(const int i, const int j,
                                     const Matrix3x3f &M)
    {
        int k = i;
        memcpy(m_data[k++] + j, &M.m00(), 12);
        memcpy(m_data[k++] + j, &M.m10(), 12);
        memcpy(m_data[k] + j, &M.m20(), 12);
    }
    template <>
    inline void SetBlock<Vector3f>(const int i, const int j, const Vector3f &v)
    {
        int k = i;
        m_data[k++][j] = v.v0();
        m_data[k++][j] = v.v1();
        m_data[k][j] = v.v2();
    }
    template <>
    inline void SetBlock<AlignedVector8f>(const int i, const int j,
                                          const AlignedVector8f &v)
    {
        int k = i;
        m_data[k++][j] = v.v0();
        m_data[k++][j] = v.v1();
        m_data[k++][j] = v.v2();
        m_data[k++][j] = v.v3();
        m_data[k++][j] = v.v4();
        m_data[k++][j] = v.v5();
        m_data[k++][j] = v.v6();
        m_data[k][j] = v.v7();
    }

    template <class MATRIX>
    inline void GetBlock(const int i, const int j, MATRIX &M) const;
    template <>
    inline void GetBlock<AlignedMatrix3x3f>(const int i, const int j,
                                            AlignedMatrix3x3f &M) const
    {
        memcpy(&M.m00(), m_data[i] + j, 12);
        memcpy(&M.m10(), m_data[i + 1] + j, 12);
        memcpy(&M.m20(), m_data[i + 2] + j, 12);
    }
    template <>
    inline void GetBlock<AlignedMatrix4x4f>(const int i, const int j,
                                            AlignedMatrix4x4f &M) const
    {
        memcpy(&M.m00(), m_data[i] + j, 16);
        memcpy(&M.m10(), m_data[i + 1] + j, 16);
        memcpy(&M.m20(), m_data[i + 2] + j, 16);
        memcpy(&M.m30(), m_data[i + 3] + j, 16);
    }
        template<> inline void GetBlock(AlignedMatrix6x6f>(const int i, const int j, AlignedMatrix6x6f &M) const
	{
        memcpy(&M.m00(), m_data[i] + j, 24);
        memcpy(&M.m10(), m_data[i + 1] + j, 24);
        memcpy(&M.m20(), m_data[i + 2] + j, 24);
        memcpy(&M.m30(), m_data[i + 3] + j, 24);
        memcpy(&M.m40(), m_data[i + 4] + j, 24);
        memcpy(&M.m50(), m_data[i + 5] + j, 24);
	}

	inline void MakeZero() {
        memset(this, 0, sizeof(MatrixNxNf<N>)); }

	inline void SetLowerFromUpper()
	{
        for (int i = 0; i < N; ++i)
            for (int j = i; j < N; ++j) m_data[j][i] = m_data[i][j];
	}

	inline bool InverseLDL() {
        return InverseLDL(*this); }
	static inline bool InverseLDL(MatrixNxNf<N> &A)
	{
        float *_A[N];
        for (int i = 0; i < N; ++i) _A[i] = A[i];
        return LS::InverseLDL<float, N>(_A);
	}

	static inline bool SolveLDL(MatrixNxNf<N> &A, VectorNf<N> &b)
	{
        float *_A[N];
        for (int i = 0; i < N; ++i) _A[i] = A[i];
        return LS::SolveLDL<float, N>(_A, b);
	}
	template<int _N> static inline bool SolveLDL(MatrixNxNf<N> &A, VectorNf<N> &b)
	{
#ifdef CFG_DEBUG
        UT_ASSERT(_N < N);
#endif
        float *_A[_N];
        for (int i = 0; i < _N; ++i) _A[i] = A[i];
        if (!LS::SolveLDL<float, _N>(_A, b)) return false;
        for (int i = _N; i < N; ++i) b[i] = 0.0f;
        return true;
	}
	
	inline void Print(const bool e = false) const
	{
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (e)
                    UT::Print("%e ", m_data[i][j]);
                else
                    UT::Print("%.4f ", m_data[i][j]);
            }
            UT::Print("\n");
        }
	}

	inline bool AssertEqual(const MatrixNxNf<N> &M, const int verbose = 1, const float eps = 0.0f) const
	{
        if (UT::VectorAssertEqual(&m_data[0][0], &M.m_data[0][0], N * N, 0,
                                  eps))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
            const MatrixNxNf<N> E = *this - M;
            UT::PrintSeparator();
            E.Print(verbose > 1);
        }
        return false;
	}

protected:

	float m_data[N][N];
};
}

#ifdef CFG_DEBUG_EIGEN
template <int N> class EigenMatrixNxNf : public Eigen::Matrix<float, N, N>
{
  public:
    inline EigenMatrixNxNf() = default;
    inline EigenMatrixNxNf(const Eigen::Matrix<float, N, N> &e_M)
        : Eigen::Matrix<float, N, N>(e_M)
    {
    }
    inline EigenMatrixNxNf(const LA::MatrixNxNf<N> &M)
        : Eigen::Matrix<float, N, N>()
    {
        Eigen::Matrix<float, N, N> &e_M = *this;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) e_M(i, j) = M[i][j];
    }
    inline void operator=(const Eigen::Matrix<float, N, N> &e_M)
    {
        *((Eigen::Matrix<float, N, N> *)this) = e_M;
    }
    inline LA::MatrixNxNf<N> GetMatrixNxNf() const
    {
        LA::MatrixNxNf<N> M;
        const Eigen::Matrix<float, N, N> &e_M = *this;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) M[i][j] = e_M(i, j);
        return M;
    }
    template <int _N>
    static inline EigenVectorNf<N> Solve(const EigenMatrixNxNf<N> &A,
                                         const EigenVectorNf<N> &b)
    {
        EigenVectorNf<N> x;
        x.block<_N, 1>(0, 0) =
            A.block<_N, _N>(0, 0).inverse() * b.block<_N, 1>(0, 0);
        for (int i = _N; i < N; ++i) x(i, 0) = 0.0f;
        return x;
    }
    inline void Print(const bool e = false) const { GetMatrixNxNf().Print(e); }
    inline bool AssertEqual(const LA::MatrixNxNf<N> &M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetMatrixNxNf().AssertEqual(M, verbose, eps);
    }
    inline bool AssertEqual(const EigenMatrixNxNf &e_M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_M.GetMatrixNxNf(), verbose, eps);
    }
};
#endif
#endif
