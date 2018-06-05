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

#include "PlatformIndependence/sse.h"

#ifndef _MATRIX_NxN_H_
#define _MATRIX_NxN_H_

#include "Matrix2x6.h"
#include "Matrix3x6.h"
#include "Matrix4x4.h"
#include "Matrix6x6.h"
#include "Matrix8x8.h"
#include "VectorN.h"

namespace LA
{
template <int M, int N> class MatrixMxNf
{
  public:
    inline const float *operator[](const int i) const { return m_data[i]; }
    inline float *operator[](const int i) { return m_data[i]; }
    inline MatrixMxNf<M, N> operator-(const MatrixMxNf<M, N> &B) const
    {
        MatrixMxNf<M, N> AmB;
        const MatrixMxNf<M, N> &A = *this;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) AmB[i][j] = A[i][j] - B[i][j];
        return AmB;
    }

#define M conflicts_with_template_arg_M
    // template<class MATRIX> inline void SetBlockDiagonalUpper(const int i,
    // const MATRIX &);
    inline void SetBlockDiagonalUpper(const int i, const SymmetricMatrix3x3f &M)
    {
        int k = i;
        memcpy(m_data[k] + k, &M.m00(), 12);
        ++k;
        memcpy(m_data[k] + k, &M.m11(), 8);
        ++k;
        m_data[k][k] = M.m22();
    }
    inline void SetBlockDiagonalUpper(const int i, const SymmetricMatrix8x8f &M)
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
#undef M

#define M conflicts_with_template_arg_M
    // template<class MATRIX> inline void SetBlock(const int i, const int j,
    // const MATRIX &M);
    inline void SetBlock(const int i, const int j, const Matrix3x3f &M)
    {
        memcpy(m_data[i] + j, &M.m00(), 12);
        memcpy(m_data[i + 1] + j, &M.m10(), 12);
        memcpy(m_data[i + 2] + j, &M.m20(), 12);
    }
#undef M
    inline void SetBlock(const int i, const int j, const Vector3f &v)
    {
        m_data[i][j] = v.v0();
        m_data[i + 1][j] = v.v1();
        m_data[i + 2][j] = v.v2();
    }
    inline void SetBlock(const int i, const int j, const AlignedVector8f &v)
    {
        m_data[i][j] = v.v0();
        m_data[i + 1][j] = v.v1();
        m_data[i + 2][j] = v.v2();
        m_data[i + 3][j] = v.v3();
        m_data[i + 4][j] = v.v4();
        m_data[i + 5][j] = v.v5();
        m_data[i + 6][j] = v.v6();
        m_data[i + 7][j] = v.v7();
    }

#define M conflicts_with_template_arg_M
    // template<class MATRIX> inline void GetBlock(const int i, const int j,
    // MATRIX &M) const;
    inline void GetBlock(const int i, const int j, AlignedMatrix3x3f &M) const
    {
        memcpy(&M.m00(), m_data[i] + j, 12);
        memcpy(&M.m10(), m_data[i + 1] + j, 12);
        memcpy(&M.m20(), m_data[i + 2] + j, 12);
    }
    inline void GetBlock(const int i, const int j, AlignedMatrix4x4f &M) const
    {
        memcpy(&M.m00(), m_data[i] + j, 16);
        memcpy(&M.m10(), m_data[i + 1] + j, 16);
        memcpy(&M.m20(), m_data[i + 2] + j, 16);
        memcpy(&M.m30(), m_data[i + 3] + j, 16);
    }
    inline void GetBlock(const int i, const int j, AlignedMatrix6x6f &M) const
    {
        memcpy(M[0], m_data[i] + j, 24);
        memcpy(M[1], m_data[i + 1] + j, 24);
        memcpy(M[2], m_data[i + 2] + j, 24);
        memcpy(M[3], m_data[i + 3] + j, 24);
        memcpy(M[4], m_data[i + 4] + j, 24);
        memcpy(M[5], m_data[i + 5] + j, 24);
    }
#undef M

    inline void MakeZero() { memset(this, 0, sizeof(MatrixMxNf<M, N>)); }
    inline void Print(const bool e = false) const
    {
        for (int i = 0; i < M; ++i) {
            for (int j = 0; j < N; ++j) {
                if (e)
                    UT::Print("%e ", m_data[i][j]);
                else
                    UT::Print("%.4f ", m_data[i][j]);
            }
            UT::Print("\n");
        }
    }

    inline bool AssertEqual(const MatrixMxNf<M, N> &_M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&m_data[0][0], &_M.m_data[0][0], M * N, 0,
                                  eps))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            _M.Print(verbose > 1);
            const MatrixMxNf<M, N> E = *this - _M;
            UT::PrintSeparator();
            E.Print(verbose > 1);
        }
        return false;
    }

  protected:
    float m_data[M][N];
};

template <int N> class MatrixNxNf : public MatrixMxNf<N, N>
{
  public:
    inline void SetLowerFromUpper()
    {
        for (int i = 0; i < N; ++i)
            for (int j = i; j < N; ++j) this->m_data[j][i] = this->m_data[i][j];
    }

    inline bool InverseLDL() { return InverseLDL(*this); }
    static inline bool InverseLDL(MatrixNxNf<N> &A)
    {
        float *_A[N];
        for (int i = 0; i < N; ++i) _A[i] = A[i];
        return LS::InverseLDL<float, N>(_A);
    }

    static inline bool SolveLDL(MatrixNxNf<N> &A, AlignedVectorNf<N> &b)
    {
        float *_A[N];
        for (int i = 0; i < N; ++i) _A[i] = A[i];
        return LS::SolveLDL<float, N>(_A, b);
    }
    template <int _N>
    static inline bool SolveLDL(MatrixNxNf<N> &A, AlignedVectorNf<N> &b)
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
};

template <int M, int N>
class
#ifdef WIN32
    _CRT_ALIGN(16)
#endif
        AlignedMatrixMxNf
{
  public:
    class Row
    {
      public:
        inline operator const float *() const { return m_data; }
        inline operator float *() { return m_data; }
      public:
        union {
            struct {
                float m_data[N];
            };
            struct {
                _pi__m128 m_data4[(N + 3) >> 2];
            };
        };
    };

  public:
    inline const Row &operator()(const int i) const { return m_rows[i]; }
    inline Row &operator()(const int i) { return m_rows[i]; }
    inline const float *operator[](const int i) const
    {
        return m_rows[i].m_data;
    }
    inline float *operator[](const int i) { return m_rows[i].m_data; }
    inline AlignedMatrixMxNf<M, N>
    operator-(const AlignedMatrixMxNf<M, N> &B) const
    {
        AlignedMatrixMxNf<M, N> AmB;
        const AlignedMatrixMxNf<M, N> &A = *this;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) AmB[i][j] = A[i][j] - B[i][j];
        return AmB;
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrixMxNf<M, N>)); }
#define M conflicts_with_template_arg_M
    // template<class MATRIX> inline void GetBlock(const int i, const int j,
    // MATRIX &M) const;
    inline void GetBlock(const int i, const int j, SymmetricMatrix6x6f &M) const
    {
        int k = i;
        memcpy(&M.m00(), m_rows[k] + k, 24);
        ++k;
        memcpy(&M.m11(), m_rows[k] + k, 20);
        ++k;
        memcpy(&M.m22(), m_rows[k] + k, 16);
        ++k;
        memcpy(&M.m33(), m_rows[k] + k, 12);
        ++k;
        memcpy(&M.m44(), m_rows[k] + k, 8);
        ++k;
        M.m55() = m_rows[k][k];
    }
    inline void GetBlock(const int i, const int j, AlignedMatrix6x6f &M) const
    {
        memcpy(M[0], m_rows[i] + j, 24);
        memcpy(M[1], m_rows[i + 1] + j, 24);
        memcpy(M[2], m_rows[i + 2] + j, 24);
        memcpy(M[3], m_rows[i + 3] + j, 24);
        memcpy(M[4], m_rows[i + 4] + j, 24);
        memcpy(M[5], m_rows[i + 5] + j, 24);
    }
    inline void GetBlock(const int i, const int j, Vector6f &v) const
    {
        v.v0() = m_rows[i][j];
        v.v1() = m_rows[i + 1][j];
        v.v2() = m_rows[i + 2][j];
        v.v3() = m_rows[i + 3][j];
        v.v4() = m_rows[i + 4][j];
        v.v5() = m_rows[i + 5][j];
    }
#undef M

    inline void SetLowerFromUpper()
    {
        const int _N = std::min(M, N);
        for (int i = 0; i < _N; ++i)
            for (int j = i; j < _N; ++j) m_rows[j][i] = m_rows[i][j];
    }

    inline void Print(const bool e = false) const
    {
        for (int i = 0; i < M; ++i) {
            const float *row = m_rows[i];
            for (int j = 0; j < N; ++j) {
                if (e)
                    UT::Print("%e ", row[j]);
                else
                    UT::Print("%.4f ", row[j]);
            }
            UT::Print("\n");
        }
    }

    inline bool AssertEqual(const AlignedMatrixMxNf<M, N> &_M,
                            const int verbose = 1, const float eps = 0.0f) const
    {
        bool equal = true;
        for (int i = 0; i < M && equal; ++i)
            equal = UT::VectorAssertEqual(m_rows[i].m_data, _M.m_rows[i].m_data,
                                          N, 0, eps);
        if (equal) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            _M.Print(verbose > 1);
            const AlignedMatrixMxNf<M, N> E = *this - _M;
            UT::PrintSeparator();
            E.Print(verbose > 1);
        }
        return false;
    }

  protected:
    Row m_rows[M];
}
#ifndef WIN32
__attribute__((aligned(16)))
#endif
;

class AlignedMatrix2x13f : public AlignedMatrixMxNf<2, 13>
{
  public:
    inline void Set(const Vector2f &M0, const AlignedMatrix2x6f &M1,
                    const AlignedMatrix2x6f &M2)
    {
        float *r;
        r = m_rows[0];
        r[0] = M0.v0();
        memcpy(r + 1, &M1.m00(), 24);
        memcpy(r + 7, &M2.m00(), 24);
        r = m_rows[1];
        r[0] = M0.v1();
        memcpy(r + 1, &M1.m10(), 24);
        memcpy(r + 7, &M2.m10(), 24);
    }

    static inline void AB(const SymmetricMatrix2x2f &A,
                          const AlignedMatrix2x13f &B, AlignedMatrix2x13f &AB)
    {
        const _pi__m128 a00 = _pi_mm_set1_ps(A.m00()),
                        a01 = _pi_mm_set1_ps(A.m01()),
                        a11 = _pi_mm_set1_ps(A.m11());
        const AlignedMatrix2x13f::Row &B0 = B(0), &B1 = B(1);
        AB(0)
            .m_data4[0] = _pi_mm_add_ps(_pi_mm_mul_ps(a00, B0.m_data4[0]),
                                        _pi_mm_mul_ps(a01, B1.m_data4[0]));
        AB(0)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(a00, B0.m_data4[1]),
                                        _pi_mm_mul_ps(a01, B1.m_data4[1]));
        AB(0)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(a00, B0.m_data4[2]),
                                        _pi_mm_mul_ps(a01, B1.m_data4[2]));
        AB(0).m_data[12] = A.m00() * B0.m_data[12] + A.m01() * B1.m_data[12];
        AB(1)
            .m_data4[0] = _pi_mm_add_ps(_pi_mm_mul_ps(a01, B0.m_data4[0]),
                                        _pi_mm_mul_ps(a11, B1.m_data4[0]));
        AB(1)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(a01, B0.m_data4[1]),
                                        _pi_mm_mul_ps(a11, B1.m_data4[1]));
        AB(1)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(a01, B0.m_data4[2]),
                                        _pi_mm_mul_ps(a11, B1.m_data4[2]));
        AB(1).m_data[12] = A.m01() * B0.m_data[12] + A.m11() * B1.m_data[12];
    }
};

class AlignedMatrix2x14f : public AlignedMatrixMxNf<2, 14>
{
  public:
    inline void Set(const AlignedMatrix2x13f &M0, const Vector2f &M1)
    {
        float *r;
        r = m_rows[0];
        memcpy(r, M0[0], 52);
        r[13] = M1.v0();
        r = m_rows[1];
        memcpy(r, M0[1], 52);
        r[13] = M1.v1();
    }
};

class AlignedMatrix3x13f : public AlignedMatrixMxNf<3, 13>
{
  public:
    inline void Set(const AlignedVector3f &M0, const AlignedMatrix3x6f &M1,
                    const AlignedMatrix3x6f &M2)
    {
        float *r;
        r = m_rows[0];
        r[0] = M0.v0();
        memcpy(r + 1, &M1.m00(), 24);
        memcpy(r + 7, &M2.m00(), 24);
        r = m_rows[1];
        r[0] = M0.v1();
        memcpy(r + 1, &M1.m10(), 24);
        memcpy(r + 7, &M2.m10(), 24);
        r = m_rows[2];
        r[0] = M0.v2();
        memcpy(r + 1, &M1.m20(), 24);
        memcpy(r + 7, &M2.m20(), 24);
    }

    static inline void AB(const SymmetricMatrix2x2f &A0, const float A1,
                          const AlignedMatrix3x13f &B, AlignedMatrix3x13f &AB)
    {
        const _pi__m128 a00 = _pi_mm_set1_ps(A0.m00()),
                        a01 = _pi_mm_set1_ps(A0.m01()),
                        a11 = _pi_mm_set1_ps(A0.m11()),
                        a22 = _pi_mm_set1_ps(A1);
        const AlignedMatrix3x13f::Row &B0 = B(0), &B1 = B(1), &B2 = B(2);
        AB(0)
            .m_data4[0] = _pi_mm_add_ps(_pi_mm_mul_ps(a00, B0.m_data4[0]),
                                        _pi_mm_mul_ps(a01, B1.m_data4[0]));
        AB(0)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(a00, B0.m_data4[1]),
                                        _pi_mm_mul_ps(a01, B1.m_data4[1]));
        AB(0)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(a00, B0.m_data4[2]),
                                        _pi_mm_mul_ps(a01, B1.m_data4[2]));
        AB(0).m_data[12] = A0.m00() * B0.m_data[12] + A0.m01() * B1.m_data[12];
        AB(1)
            .m_data4[0] = _pi_mm_add_ps(_pi_mm_mul_ps(a01, B0.m_data4[0]),
                                        _pi_mm_mul_ps(a11, B1.m_data4[0]));
        AB(1)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(a01, B0.m_data4[1]),
                                        _pi_mm_mul_ps(a11, B1.m_data4[1]));
        AB(1)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(a01, B0.m_data4[2]),
                                        _pi_mm_mul_ps(a11, B1.m_data4[2]));
        AB(1).m_data[12] = A0.m01() * B0.m_data[12] + A0.m11() * B1.m_data[12];
        AB(2).m_data4[0] = _pi_mm_mul_ps(a22, B2.m_data4[0]);
        AB(2).m_data4[1] = _pi_mm_mul_ps(a22, B2.m_data4[1]);
        AB(2).m_data4[2] = _pi_mm_mul_ps(a22, B2.m_data4[2]);
        AB(2).m_data[12] = A1 * B2.m_data[12];
    }
};

class AlignedMatrix3x14f : public AlignedMatrixMxNf<3, 14>
{
  public:
    inline void Set(const AlignedMatrix3x13f &M0, const Vector2f &M10,
                    const float M11)
    {
        float *r;
        r = m_rows[0];
        memcpy(r, M0[0], 52);
        r[13] = M10.v0();
        r = m_rows[1];
        memcpy(r, M0[1], 52);
        r[13] = M10.v1();
        r = m_rows[2];
        memcpy(r, M0[2], 52);
        r[13] = M11;
    }
};

class AlignedMatrix13x14f : public AlignedMatrixMxNf<13, 14>
{
  public:
    inline void Get(float &M00, Vector6f &M01, Vector6f &M02, float &M03,
                    SymmetricMatrix6x6f &M11, AlignedMatrix6x6f &M12,
                    Vector6f &M13, SymmetricMatrix6x6f &M22,
                    Vector6f &M23) const
    {
        M00 = m_rows[0][0];
        memcpy(M01, m_rows[0] + 1, 24);
        memcpy(M02, m_rows[0] + 7, 24);
        M03 = m_rows[0][13];
        GetBlock(1, 1, M11);
        GetBlock(1, 7, M12);
        GetBlock(1, 13, M13);
        GetBlock(7, 7, M22);
        GetBlock(7, 13, M23);
    }

    static inline void ATBToUpper(const AlignedMatrix2x13f &A,
                                  const AlignedMatrix2x14f &B,
                                  AlignedMatrix13x14f &ATB)
    {
        float a0, a1;
        _pi__m128 _a0, _a1;
        const float *A0 = A[0], *A1 = A[1];
        const AlignedMatrix2x14f::Row &B0 = B(0), &B1 = B(1);
        _a0 = _pi_mm_set1_ps(A0[0]);
        _a1 = _pi_mm_set1_ps(A1[0]);
        ATB(0)
            .m_data4[0] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[0]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[0]));
        ATB(0)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[1]));
        ATB(0)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[2]));
        ATB(0)
            .m_data[12] =
            _a0.m128_f32[0] * B0.m_data[12] + _a1.m128_f32[0] * B1.m_data[12];
        ATB(0)
            .m_data[13] =
            _a0.m128_f32[0] * B0.m_data[13] + _a1.m128_f32[0] * B1.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[1]);
        _a1 = _pi_mm_set1_ps(A1[1]);
        ATB(1)
            .m_data4[0] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[0]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[0]));
        ATB(1)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[1]));
        ATB(1)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[2]));
        ATB(1)
            .m_data[12] =
            _a0.m128_f32[0] * B0.m_data[12] + _a1.m128_f32[0] * B1.m_data[12];
        ATB(1)
            .m_data[13] =
            _a0.m128_f32[0] * B0.m_data[13] + _a1.m128_f32[0] * B1.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[2]);
        _a1 = _pi_mm_set1_ps(A1[2]);
        ATB(2)
            .m_data[2] =
            _a0.m128_f32[0] * B0.m_data[2] + _a1.m128_f32[0] * B1.m_data[2];
        ATB(2)
            .m_data[3] =
            _a0.m128_f32[0] * B0.m_data[3] + _a1.m128_f32[0] * B1.m_data[3];
        ATB(2)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[1]));
        ATB(2)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[2]));
        ATB(2)
            .m_data[12] =
            _a0.m128_f32[0] * B0.m_data[12] + _a1.m128_f32[0] * B1.m_data[12];
        ATB(2)
            .m_data[13] =
            _a0.m128_f32[0] * B0.m_data[13] + _a1.m128_f32[0] * B1.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[3]);
        _a1 = _pi_mm_set1_ps(A1[3]);
        ATB(3)
            .m_data[3] =
            _a0.m128_f32[0] * B0.m_data[3] + _a1.m128_f32[0] * B1.m_data[3];
        ATB(3)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[1]));
        ATB(3)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[2]));
        ATB(3)
            .m_data[12] =
            _a0.m128_f32[0] * B0.m_data[12] + _a1.m128_f32[0] * B1.m_data[12];
        ATB(3)
            .m_data[13] =
            _a0.m128_f32[0] * B0.m_data[13] + _a1.m128_f32[0] * B1.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[4]);
        _a1 = _pi_mm_set1_ps(A1[4]);
        ATB(4)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[1]));
        ATB(4)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[2]));
        ATB(4)
            .m_data[12] =
            _a0.m128_f32[0] * B0.m_data[12] + _a1.m128_f32[0] * B1.m_data[12];
        ATB(4)
            .m_data[13] =
            _a0.m128_f32[0] * B0.m_data[13] + _a1.m128_f32[0] * B1.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[5]);
        _a1 = _pi_mm_set1_ps(A1[5]);
        ATB(5)
            .m_data4[1] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[1]));
        ATB(5)
            .m_data4[2] = _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                                        _pi_mm_mul_ps(_a1, B1.m_data4[2]));
        ATB(5)
            .m_data[12] =
            _a0.m128_f32[0] * B0.m_data[12] + _a1.m128_f32[0] * B1.m_data[12];
        ATB(5)
            .m_data[13] =
            _a0.m128_f32[0] * B0.m_data[13] + _a1.m128_f32[0] * B1.m_data[13];
        a0 = A0[6];
        a1 = A1[6];
        ATB(6).m_data[6] = a0 * B0.m_data[6] + a1 * B1.m_data[6];
        ATB(6).m_data[7] = a0 * B0.m_data[7] + a1 * B1.m_data[7];
        ATB(6)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]));
        ATB(6).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(6).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[7];
        a1 = A1[7];
        ATB(7).m_data[7] = a0 * B0.m_data[7] + a1 * B1.m_data[7];
        ATB(7)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]));
        ATB(7).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(7).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[8];
        a1 = A1[8];
        ATB(8)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]));
        ATB(8).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(8).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[9];
        a1 = A1[9];
        ATB(9)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]));
        ATB(9).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(9).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[10];
        a1 = A1[10];
        ATB(10).m_data[10] = a0 * B0.m_data[10] + a1 * B1.m_data[10];
        ATB(10).m_data[11] = a0 * B0.m_data[11] + a1 * B1.m_data[11];
        ATB(10).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(10).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[11];
        a1 = A1[11];
        ATB(11).m_data[11] = a0 * B0.m_data[11] + a1 * B1.m_data[11];
        ATB(11).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(11).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[12];
        a1 = A1[12];
        ATB(12).m_data[12] = a0 * B0.m_data[12] + a1 * B1.m_data[12];
        ATB(12).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
        a0 = A0[13];
        a1 = A1[13];
        ATB(13).m_data[13] = a0 * B0.m_data[13] + a1 * B1.m_data[13];
    }

    static inline void ATBToUpper(const AlignedMatrix3x13f &A,
                                  const AlignedMatrix3x14f &B,
                                  AlignedMatrix13x14f &ATB)
    {
        float a0, a1, a2;
        _pi__m128 _a0, _a1, _a2;
        const float *A0 = A[0], *A1 = A[1], *A2 = A[2];
        const AlignedMatrix3x14f::Row &B0 = B(0), &B1 = B(1), &B2 = B(2);
        _a0 = _pi_mm_set1_ps(A0[0]);
        _a1 = _pi_mm_set1_ps(A1[0]);
        _a2 = _pi_mm_set1_ps(A2[0]);
        ATB(0)
            .m_data4[0] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[0]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[0]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[0])));
        ATB(0)
            .m_data4[1] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[1]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[1])));
        ATB(0)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[2]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[2])));
        ATB(0)
            .m_data[12] = _a0.m128_f32[0] * B0.m_data[12] +
                          _a1.m128_f32[0] * B1.m_data[12] +
                          _a2.m128_f32[0] * B2.m_data[12];
        ATB(0)
            .m_data[13] = _a0.m128_f32[0] * B0.m_data[13] +
                          _a1.m128_f32[0] * B1.m_data[13] +
                          _a2.m128_f32[0] * B2.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[1]);
        _a1 = _pi_mm_set1_ps(A1[1]);
        _a2 = _pi_mm_set1_ps(A2[1]);
        ATB(1)
            .m_data4[0] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[0]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[0]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[0])));
        ATB(1)
            .m_data4[1] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[1]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[1])));
        ATB(1)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[2]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[2])));
        ATB(1)
            .m_data[12] = _a0.m128_f32[0] * B0.m_data[12] +
                          _a1.m128_f32[0] * B1.m_data[12] +
                          _a2.m128_f32[0] * B2.m_data[12];
        ATB(1)
            .m_data[13] = _a0.m128_f32[0] * B0.m_data[13] +
                          _a1.m128_f32[0] * B1.m_data[13] +
                          _a2.m128_f32[0] * B2.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[2]);
        _a1 = _pi_mm_set1_ps(A1[2]);
        _a2 = _pi_mm_set1_ps(A2[2]);
        ATB(2)
            .m_data[2] = _a0.m128_f32[0] * B0.m_data[2] +
                         _a1.m128_f32[0] * B1.m_data[2] +
                         _a2.m128_f32[0] * B2.m_data[2];
        ATB(2)
            .m_data[3] = _a0.m128_f32[0] * B0.m_data[3] +
                         _a1.m128_f32[0] * B1.m_data[3] +
                         _a2.m128_f32[0] * B2.m_data[3];
        ATB(2)
            .m_data4[1] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[1]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[1])));
        ATB(2)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[2]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[2])));
        ATB(2)
            .m_data[12] = _a0.m128_f32[0] * B0.m_data[12] +
                          _a1.m128_f32[0] * B1.m_data[12] +
                          _a2.m128_f32[0] * B2.m_data[12];
        ATB(2)
            .m_data[13] = _a0.m128_f32[0] * B0.m_data[13] +
                          _a1.m128_f32[0] * B1.m_data[13] +
                          _a2.m128_f32[0] * B2.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[3]);
        _a1 = _pi_mm_set1_ps(A1[3]);
        _a2 = _pi_mm_set1_ps(A2[3]);
        ATB(3)
            .m_data[3] = _a0.m128_f32[0] * B0.m_data[3] +
                         _a1.m128_f32[0] * B1.m_data[3] +
                         _a2.m128_f32[0] * B2.m_data[3];
        ATB(3)
            .m_data4[1] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[1]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[1])));
        ATB(3)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[2]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[2])));
        ATB(3)
            .m_data[12] = _a0.m128_f32[0] * B0.m_data[12] +
                          _a1.m128_f32[0] * B1.m_data[12] +
                          _a2.m128_f32[0] * B2.m_data[12];
        ATB(3)
            .m_data[13] = _a0.m128_f32[0] * B0.m_data[13] +
                          _a1.m128_f32[0] * B1.m_data[13] +
                          _a2.m128_f32[0] * B2.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[4]);
        _a1 = _pi_mm_set1_ps(A1[4]);
        _a2 = _pi_mm_set1_ps(A2[4]);
        ATB(4)
            .m_data4[1] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[1]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[1])));
        ATB(4)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[2]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[2])));
        ATB(4)
            .m_data[12] = _a0.m128_f32[0] * B0.m_data[12] +
                          _a1.m128_f32[0] * B1.m_data[12] +
                          _a2.m128_f32[0] * B2.m_data[12];
        ATB(4)
            .m_data[13] = _a0.m128_f32[0] * B0.m_data[13] +
                          _a1.m128_f32[0] * B1.m_data[13] +
                          _a2.m128_f32[0] * B2.m_data[13];
        _a0 = _pi_mm_set1_ps(A0[5]);
        _a1 = _pi_mm_set1_ps(A1[5]);
        _a2 = _pi_mm_set1_ps(A2[5]);
        ATB(5)
            .m_data4[1] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[1]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[1]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[1])));
        ATB(5)
            .m_data4[2] =
            _pi_mm_add_ps(_pi_mm_mul_ps(_a0, B0.m_data4[2]),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_a1, B1.m_data4[2]),
                                        _pi_mm_mul_ps(_a2, B2.m_data4[2])));
        ATB(5)
            .m_data[12] = _a0.m128_f32[0] * B0.m_data[12] +
                          _a1.m128_f32[0] * B1.m_data[12] +
                          _a2.m128_f32[0] * B2.m_data[12];
        ATB(5)
            .m_data[13] = _a0.m128_f32[0] * B0.m_data[13] +
                          _a1.m128_f32[0] * B1.m_data[13] +
                          _a2.m128_f32[0] * B2.m_data[13];
        a0 = A0[6];
        a1 = A1[6];
        a2 = A2[6];
        ATB(6)
            .m_data[6] =
            a0 * B0.m_data[6] + a1 * B1.m_data[6] + a2 * B2.m_data[6];
        ATB(6)
            .m_data[7] =
            a0 * B0.m_data[7] + a1 * B1.m_data[7] + a2 * B2.m_data[7];
        ATB(6)
            .m_data4[2] = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a2), B2.m_data4[2])));
        ATB(6)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(6)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[7];
        a1 = A1[7];
        a2 = A2[7];
        ATB(7)
            .m_data[7] =
            a0 * B0.m_data[7] + a1 * B1.m_data[7] + a2 * B2.m_data[7];
        ATB(7)
            .m_data4[2] = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a2), B2.m_data4[2])));
        ATB(7)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(7)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[8];
        a1 = A1[8];
        a2 = A2[8];
        ATB(8)
            .m_data4[2] = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a2), B2.m_data4[2])));
        ATB(8)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(8)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[9];
        a1 = A1[9];
        a2 = A2[9];
        ATB(9)
            .m_data4[2] = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(a0), B0.m_data4[2]),
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(a1), B1.m_data4[2]),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(a2), B2.m_data4[2])));
        ATB(9)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(9)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[10];
        a1 = A1[10];
        a2 = A2[10];
        ATB(10)
            .m_data[10] =
            a0 * B0.m_data[10] + a1 * B1.m_data[10] + a2 * B2.m_data[10];
        ATB(10)
            .m_data[11] =
            a0 * B0.m_data[11] + a1 * B1.m_data[11] + a2 * B2.m_data[11];
        ATB(10)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(10)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[11];
        a1 = A1[11];
        a2 = A2[11];
        ATB(11)
            .m_data[11] =
            a0 * B0.m_data[11] + a1 * B1.m_data[11] + a2 * B2.m_data[11];
        ATB(11)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(11)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[12];
        a1 = A1[12];
        a2 = A2[12];
        ATB(12)
            .m_data[12] =
            a0 * B0.m_data[12] + a1 * B1.m_data[12] + a2 * B2.m_data[12];
        ATB(12)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
        a0 = A0[13];
        a1 = A1[13];
        a2 = A2[13];
        ATB(13)
            .m_data[13] =
            a0 * B0.m_data[13] + a1 * B1.m_data[13] + a2 * B2.m_data[13];
    }
};
}

#ifdef CFG_DEBUG_EIGEN
template <int M, int N>
class EigenMatrixMxNf : public Eigen::Matrix<float, M, N>
{
  public:
    inline EigenMatrixMxNf() = default;
    inline EigenMatrixMxNf(const Eigen::Matrix<float, M, N> &e_M)
        : Eigen::Matrix<float, M, N>(e_M)
    {
    }
    inline EigenMatrixMxNf(const LA::MatrixMxNf<M, N> &_M)
        : Eigen::Matrix<float, M, N>()
    {
        Eigen::Matrix<float, M, N> &e_M = *this;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) e_M(i, j) = _M[i][j];
    }
    inline EigenMatrixMxNf(const LA::AlignedMatrixMxNf<M, N> &_M)
        : Eigen::Matrix<float, M, N>()
    {
        Eigen::Matrix<float, M, N> &e_M = *this;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) e_M(i, j) = _M[i][j];
    }
    inline void operator=(const Eigen::Matrix<float, M, N> &e_M)
    {
        *((Eigen::Matrix<float, M, N> *)this) = e_M;
    }
    inline LA::MatrixMxNf<M, N> GetMatrixMxNf() const
    {
        LA::MatrixMxNf<M, N> _M;
        const Eigen::Matrix<float, M, N> &e_M = *this;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) _M[i][j] = e_M(i, j);
        return _M;
    }
    inline LA::AlignedMatrixMxNf<M, N> GetAlignedMatrixMxNf() const
    {
        LA::AlignedMatrixMxNf<M, N> _M;
        const Eigen::Matrix<float, M, N> &e_M = *this;
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < N; ++j) _M[i][j] = e_M(i, j);
        return _M;
    }
    inline void Print(const bool e = false) const { GetMatrixMxNf().Print(e); }
    inline bool AssertEqual(const LA::MatrixMxNf<M, N> &_M,
                            const int verbose = 1, const float eps = 0.0f) const
    {
        return GetMatrixMxNf().AssertEqual(_M, verbose, eps);
    }
    inline bool AssertEqual(const LA::AlignedMatrixMxNf<M, N> &_M,
                            const int verbose = 1, const float eps = 0.0f) const
    {
        return GetAlignedMatrixMxNf().AssertEqual(_M, verbose, eps);
    }
    inline bool AssertEqual(const EigenMatrixMxNf &e_M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_M.GetMatrixMxNf(), verbose, eps);
    }
};
template <int N> class EigenMatrixNxNf : public EigenMatrixMxNf<N, N>
{
  public:
    inline EigenMatrixNxNf() = default;
    inline EigenMatrixNxNf(const Eigen::Matrix<float, N, N> &e_M)
        : EigenMatrixMxNf<N, N>(e_M)
    {
    }
    inline EigenMatrixNxNf(const LA::MatrixNxNf<N> &M)
        : EigenMatrixMxNf<N, N>(M)
    {
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
};

class EigenMatrix2x13f : public EigenMatrixMxNf<2, 13>
{
  public:
    inline EigenMatrix2x13f() : EigenMatrixMxNf<2, 13>() {}
    inline EigenMatrix2x13f(const Eigen::Matrix<float, 2, 13> &e_M)
        : EigenMatrixMxNf<2, 13>(e_M)
    {
    }
    inline EigenMatrix2x13f(const LA::MatrixMxNf<2, 13> &_M)
        : EigenMatrixMxNf<2, 13>(_M)
    {
    }
    inline EigenMatrix2x13f(const LA::AlignedMatrixMxNf<2, 13> &_M)
        : EigenMatrixMxNf<2, 13>(_M)
    {
    }
    inline EigenMatrix2x13f(const EigenVector2f &e_M0,
                            const EigenMatrix2x6f &e_M1,
                            const EigenMatrix2x6f &e_M2)
    {
        block<2, 1>(0, 0) = e_M0;
        block<2, 6>(0, 1) = e_M1;
        block<2, 6>(0, 7) = e_M2;
    }
};
class EigenMatrix2x14f : public EigenMatrixMxNf<2, 14>
{
  public:
    inline EigenMatrix2x14f() : EigenMatrixMxNf<2, 14>() {}
    inline EigenMatrix2x14f(const Eigen::Matrix<float, 2, 14> &e_M)
        : EigenMatrixMxNf<2, 14>(e_M)
    {
    }
    inline EigenMatrix2x14f(const LA::MatrixMxNf<2, 14> &_M)
        : EigenMatrixMxNf<2, 14>(_M)
    {
    }
    inline EigenMatrix2x14f(const LA::AlignedMatrixMxNf<2, 14> &_M)
        : EigenMatrixMxNf<2, 14>(_M)
    {
    }
    inline EigenMatrix2x14f(const EigenMatrix2x13f &e_M0,
                            const EigenVector2f &e_M1)
    {
        block<2, 13>(0, 0) = e_M0;
        block<2, 1>(0, 13) = e_M1;
    }
};
class EigenMatrix3x13f : public EigenMatrixMxNf<3, 13>
{
  public:
    inline EigenMatrix3x13f() : EigenMatrixMxNf<3, 13>() {}
    inline EigenMatrix3x13f(const Eigen::Matrix<float, 3, 13> &e_M)
        : EigenMatrixMxNf<3, 13>(e_M)
    {
    }
    inline EigenMatrix3x13f(const LA::MatrixMxNf<3, 13> &_M)
        : EigenMatrixMxNf<3, 13>(_M)
    {
    }
    inline EigenMatrix3x13f(const LA::AlignedMatrixMxNf<3, 13> &_M)
        : EigenMatrixMxNf<3, 13>(_M)
    {
    }
    inline EigenMatrix3x13f(const EigenVector3f &e_M0,
                            const EigenMatrix3x6f &e_M1,
                            const EigenMatrix3x6f &e_M2)
    {
        block<3, 1>(0, 0) = e_M0;
        block<3, 6>(0, 1) = e_M1;
        block<3, 6>(0, 7) = e_M2;
    }
};
class EigenMatrix3x14f : public EigenMatrixMxNf<3, 14>
{
  public:
    inline EigenMatrix3x14f() : EigenMatrixMxNf<3, 14>() {}
    inline EigenMatrix3x14f(const Eigen::Matrix<float, 3, 14> &e_M)
        : EigenMatrixMxNf<3, 14>(e_M)
    {
    }
    inline EigenMatrix3x14f(const LA::MatrixMxNf<3, 14> &_M)
        : EigenMatrixMxNf<3, 14>(_M)
    {
    }
    inline EigenMatrix3x14f(const LA::AlignedMatrixMxNf<3, 14> &_M)
        : EigenMatrixMxNf<3, 14>(_M)
    {
    }
    inline EigenMatrix3x14f(const EigenMatrix3x13f &e_M0,
                            const EigenVector3f &e_M1)
    {
        block<3, 13>(0, 0) = e_M0;
        block<3, 1>(0, 13) = e_M1;
    }
};
class EigenMatrix13x14f : public EigenMatrixMxNf<13, 14>
{
  public:
    inline EigenMatrix13x14f() : EigenMatrixMxNf<13, 14>() {}
    inline EigenMatrix13x14f(const Eigen::Matrix<float, 13, 14> &e_M)
        : EigenMatrixMxNf<13, 14>(e_M)
    {
    }
    inline EigenMatrix13x14f(const LA::MatrixMxNf<13, 14> &_M)
        : EigenMatrixMxNf<13, 14>(_M)
    {
    }
    inline EigenMatrix13x14f(const LA::AlignedMatrixMxNf<13, 14> &_M)
        : EigenMatrixMxNf<13, 14>(_M)
    {
    }
};
class EigenMatrix6x12f : public EigenMatrixMxNf<6, 12>
{
  public:
    inline EigenMatrix6x12f() : EigenMatrixMxNf<6, 12>() {}
    inline EigenMatrix6x12f(const Eigen::Matrix<float, 6, 12> &e_M)
        : EigenMatrixMxNf<6, 12>(e_M)
    {
    }
    inline EigenMatrix6x12f(const EigenMatrix6x6f &e_M0,
                            const EigenMatrix6x6f &e_M1)
    {
        block<6, 6>(0, 0) = e_M0;
        block<6, 6>(0, 6) = e_M1;
    }
};
class EigenMatrix6x13f : public EigenMatrixMxNf<6, 13>
{
  public:
    inline EigenMatrix6x13f() : EigenMatrixMxNf<6, 13>() {}
    inline EigenMatrix6x13f(const Eigen::Matrix<float, 6, 13> &e_M)
        : EigenMatrixMxNf<6, 13>(e_M)
    {
    }
    inline EigenMatrix6x13f(const EigenMatrix6x12f &e_M0,
                            const EigenVector6f &e_M1)
    {
        block<6, 12>(0, 0) = e_M0;
        block<6, 1>(0, 12) = e_M1;
    }
};
#endif
#endif
