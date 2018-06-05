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

#ifndef _MATRIX_4x4_H_
#define _MATRIX_4x4_H_

#include "LinearSystem.h"
#include "Matrix3x3.h"
#include "Vector4.h"

namespace LA
{
class AlignedMatrix4x4f
{
  public:
    inline const _pi__m128 &m_00_01_02_03() const { return m_data[0]; }
    inline _pi__m128 &m_00_01_02_03() { return m_data[0]; }
    inline const _pi__m128 &m_10_11_12_13() const { return m_data[1]; }
    inline _pi__m128 &m_10_11_12_13() { return m_data[1]; }
    inline const _pi__m128 &m_20_21_22_23() const { return m_data[2]; }
    inline _pi__m128 &m_20_21_22_23() { return m_data[2]; }
    inline const _pi__m128 &m_30_31_32_33() const { return m_data[3]; }
    inline _pi__m128 &m_30_31_32_33() { return m_data[3]; }
    inline const float &m00() const { return m_data[0].m128_f32[0]; }
    inline float &m00() { return m_data[0].m128_f32[0]; }
    inline const float &m01() const { return m_data[0].m128_f32[1]; }
    inline float &m01() { return m_data[0].m128_f32[1]; }
    inline const float &m02() const { return m_data[0].m128_f32[2]; }
    inline float &m02() { return m_data[0].m128_f32[2]; }
    inline const float &m03() const { return m_data[0].m128_f32[3]; }
    inline float &m03() { return m_data[0].m128_f32[3]; }
    inline const float &m10() const { return m_data[1].m128_f32[0]; }
    inline float &m10() { return m_data[1].m128_f32[0]; }
    inline const float &m11() const { return m_data[1].m128_f32[1]; }
    inline float &m11() { return m_data[1].m128_f32[1]; }
    inline const float &m12() const { return m_data[1].m128_f32[2]; }
    inline float &m12() { return m_data[1].m128_f32[2]; }
    inline const float &m13() const { return m_data[1].m128_f32[3]; }
    inline float &m13() { return m_data[1].m128_f32[3]; }
    inline const float &m20() const { return m_data[2].m128_f32[0]; }
    inline float &m20() { return m_data[2].m128_f32[0]; }
    inline const float &m21() const { return m_data[2].m128_f32[1]; }
    inline float &m21() { return m_data[2].m128_f32[1]; }
    inline const float &m22() const { return m_data[2].m128_f32[2]; }
    inline float &m22() { return m_data[2].m128_f32[2]; }
    inline const float &m23() const { return m_data[2].m128_f32[3]; }
    inline float &m23() { return m_data[2].m128_f32[3]; }
    inline const float &m30() const { return m_data[3].m128_f32[0]; }
    inline float &m30() { return m_data[3].m128_f32[0]; }
    inline const float &m31() const { return m_data[3].m128_f32[1]; }
    inline float &m31() { return m_data[3].m128_f32[1]; }
    inline const float &m32() const { return m_data[3].m128_f32[2]; }
    inline float &m32() { return m_data[3].m128_f32[2]; }
    inline const float &m33() const { return m_data[3].m128_f32[3]; }
    inline float &m33() { return m_data[3].m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator=(const AlignedMatrix3x3f &M)
    {
        memcpy(&m00(), &M.m00(), 12);
        m03() = 0.0f;
        memcpy(&m10(), &M.m10(), 12);
        m13() = 0.0f;
        memcpy(&m20(), &M.m20(), 12);
        m23() = 0.0f;
        m_30_31_32_33() = _pi_mm_setr_ps(0.0f, 0.0f, 0.0f, 1.0f);
    }
    inline AlignedMatrix4x4f operator-(const AlignedMatrix4x4f &B) const
    {
        AlignedMatrix4x4f AmB;
        AmB.m_00_01_02_03() = _pi_mm_sub_ps(m_00_01_02_03(), B.m_00_01_02_03());
        AmB.m_10_11_12_13() = _pi_mm_sub_ps(m_10_11_12_13(), B.m_10_11_12_13());
        AmB.m_20_21_22_23() = _pi_mm_sub_ps(m_20_21_22_23(), B.m_20_21_22_23());
        AmB.m_30_31_32_33() = _pi_mm_sub_ps(m_30_31_32_33(), B.m_30_31_32_33());
        return AmB;
    }

    inline void Get(AlignedMatrix3x3f &M) const
    {
        memcpy(M, this, sizeof(AlignedMatrix3x3f));
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix4x4f)); }
    inline void MakeIdentity()
    {
        MakeZero();
        m00() = m11() = m22() = m33() = 1.0f;
    }

    inline void SetLowerFromUpper()
    {
        m10() = m01();
        m20() = m02();
        m30() = m03();
        m21() = m12();
        m31() = m13();
        m32() = m23();
    }

    inline bool InverseLDL(const bool decomposed = false)
    {
        return InverseLDL(*this, decomposed);
    }
    static inline bool InverseLDL(AlignedMatrix4x4f &A,
                                  const bool decomposed = false)
    {
        float *_A[4] = {&A.m00(), &A.m10(), &A.m20(), &A.m30()};
        return LS::InverseLDL<float, 4>(_A, decomposed);
    }
    static inline bool SolveLDL(AlignedMatrix4x4f &A, AlignedVector4f &b)
    {
        float *_A[4] = {&A.m00(), &A.m10(), &A.m20(), &A.m30()};
        return LS::SolveLDL<float, 4>(_A, b);
    }
    template <int N>
    static inline bool SolveLDL(AlignedMatrix4x4f &A, AlignedVector4f &b)
    {
        float *_A[4] = {&A.m00(), &A.m10(), &A.m20(), &A.m30()};
        return LS::SolveLDL<float, N>(_A, b);
    }

    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e\n", m00(), m01(), m02(), m03());
            UT::Print("%e %e %e %e\n", m10(), m11(), m12(), m13());
            UT::Print("%e %e %e %e\n", m20(), m21(), m22(), m23());
            UT::Print("%e %e %e %e\n", m30(), m31(), m32(), m33());
        } else {
            UT::Print("%f %f %f %f\n", m00(), m01(), m02(), m03());
            UT::Print("%f %f %f %f\n", m10(), m11(), m12(), m13());
            UT::Print("%f %f %f %f\n", m20(), m21(), m22(), m23());
            UT::Print("%f %f %f %f\n", m30(), m31(), m32(), m33());
        }
    }
    inline bool AssertEqual(const AlignedMatrix4x4f &M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 16, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

  protected:
    _pi__m128 m_data[4];
};

template <typename TYPE> class SymmetricMatrix4x4
{
  public:
    inline const TYPE &m00() const { return m_data[0]; }
    inline TYPE &m00() { return m_data[0]; }
    inline const TYPE &m01() const { return m_data[1]; }
    inline TYPE &m01() { return m_data[1]; }
    inline const TYPE &m02() const { return m_data[2]; }
    inline TYPE &m02() { return m_data[2]; }
    inline const TYPE &m03() const { return m_data[3]; }
    inline TYPE &m03() { return m_data[3]; }
    inline const TYPE &m11() const { return m_data[4]; }
    inline TYPE &m11() { return m_data[4]; }
    inline const TYPE &m12() const { return m_data[5]; }
    inline TYPE &m12() { return m_data[5]; }
    inline const TYPE &m13() const { return m_data[6]; }
    inline TYPE &m13() { return m_data[6]; }
    inline const TYPE &m22() const { return m_data[7]; }
    inline TYPE &m22() { return m_data[7]; }
    inline const TYPE &m23() const { return m_data[8]; }
    inline TYPE &m23() { return m_data[8]; }
    inline const TYPE &m33() const { return m_data[9]; }
    inline TYPE &m33() { return m_data[9]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void MakeZero()
    {
        memset(this, 0, sizeof(SymmetricMatrix4x4<TYPE>));
    }

  protected:
    TYPE m_data[10];
};

typedef SymmetricMatrix4x4<float> SymmetricMatrix4x4f;
typedef SymmetricMatrix4x4<double> SymmetricMatrix4x4d;
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix4x4f : public Eigen::Matrix4f
{
  public:
    inline EigenMatrix4x4f() = default;
    inline EigenMatrix4x4f(const Eigen::Matrix4f &e_M) : Eigen::Matrix4f(e_M) {}
    inline EigenMatrix4x4f(const LA::AlignedMatrix4x4f &M) : Eigen::Matrix4f()
    {
        Eigen::Matrix4f &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(0, 2) = M.m02();
        e_M(0, 3) = M.m03();
        e_M(1, 0) = M.m10();
        e_M(1, 1) = M.m11();
        e_M(1, 2) = M.m12();
        e_M(1, 3) = M.m13();
        e_M(2, 0) = M.m20();
        e_M(2, 1) = M.m21();
        e_M(2, 2) = M.m22();
        e_M(2, 3) = M.m23();
        e_M(3, 0) = M.m30();
        e_M(3, 1) = M.m31();
        e_M(3, 2) = M.m32();
        e_M(3, 3) = M.m33();
    }
    inline void operator=(const Eigen::Matrix4f &e_M)
    {
        *((Eigen::Matrix4f *)this) = e_M;
    }
    inline LA::AlignedMatrix4x4f GetAlignedMatrix4x4f() const
    {
        LA::AlignedMatrix4x4f M;
        const Eigen::Matrix4f &e_M = *this;
        M.m00() = e_M(0, 0);
        M.m01() = e_M(0, 1);
        M.m02() = e_M(0, 2);
        M.m03() = e_M(0, 3);
        M.m10() = e_M(1, 0);
        M.m11() = e_M(1, 1);
        M.m12() = e_M(1, 2);
        M.m13() = e_M(1, 3);
        M.m20() = e_M(2, 0);
        M.m21() = e_M(2, 1);
        M.m22() = e_M(2, 2);
        M.m23() = e_M(2, 3);
        M.m30() = e_M(3, 0);
        M.m31() = e_M(3, 1);
        M.m32() = e_M(3, 2);
        M.m33() = e_M(3, 3);
        return M;
    }
    template <int N>
    static inline EigenVector4f Solve(const EigenMatrix4x4f &A,
                                      const EigenVector4f &b)
    {
        EigenVector4f x;
        x.block<N, 1>(0, 0) =
            A.block<N, N>(0, 0).inverse() * b.block<N, 1>(0, 0);
        for (int i = N; i < 4; ++i) x(i, 0) = 0.0f;
        return x;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix4x4f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix4x4f &M,
                            const int verbose = 1, const float eps = 0.0f) const
    {
        return GetAlignedMatrix4x4f().AssertEqual(M, verbose, eps);
    }
    inline bool AssertEqual(const EigenMatrix4x4f &e_M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_M.GetAlignedMatrix4x4f(), verbose, eps);
    }
};
#endif
#endif
