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

#ifndef _MATRIX_2x3_H_
#define _MATRIX_2x3_H_

#include "Vector3.h"

namespace LA
{
class AlignedMatrix2x3f
{
  public:
    inline const _pi__m128 &m_00_01_02_r0() const { return m_data[0]; }
    inline _pi__m128 &m_00_01_02_r0() { return m_data[0]; }
    inline const _pi__m128 &m_10_11_12_r1() const { return m_data[1]; }
    inline _pi__m128 &m_10_11_12_r1() { return m_data[1]; }
    inline const float &m00() const { return m_data[0].m128_f32[0]; }
    inline float &m00() { return m_data[0].m128_f32[0]; }
    inline const float &m01() const { return m_data[0].m128_f32[1]; }
    inline float &m01() { return m_data[0].m128_f32[1]; }
    inline const float &m02() const { return m_data[0].m128_f32[2]; }
    inline float &m02() { return m_data[0].m128_f32[2]; }
    inline const float &r0() const { return m_data[0].m128_f32[3]; }
    inline float &r0() { return m_data[0].m128_f32[3]; }
    inline const float &m10() const { return m_data[1].m128_f32[0]; }
    inline float &m10() { return m_data[1].m128_f32[0]; }
    inline const float &m11() const { return m_data[1].m128_f32[1]; }
    inline float &m11() { return m_data[1].m128_f32[1]; }
    inline const float &m12() const { return m_data[1].m128_f32[2]; }
    inline float &m12() { return m_data[1].m128_f32[2]; }
    inline const float &r1() const { return m_data[1].m128_f32[3]; }
    inline float &r1() { return m_data[1].m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator+=(const AlignedMatrix2x3f &M)
    {
        m_00_01_02_r0() = _pi_mm_add_ps(M.m_00_01_02_r0(), m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_add_ps(M.m_10_11_12_r1(), m_10_11_12_r1());
    }
    inline void operator-=(const AlignedMatrix2x3f &M)
    {
        m_00_01_02_r0() = _pi_mm_sub_ps(m_00_01_02_r0(), M.m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_sub_ps(m_10_11_12_r1(), M.m_10_11_12_r1());
    }
    inline void operator*=(const float s)
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        Scale(_s);
    }
    inline void operator*=(const _pi__m128 &s) { Scale(s); }
    inline void Set(const float m00, const float m01, const float m02,
                    const float m10, const float m11, const float m12)
    {
        m_00_01_02_r0() = _pi_mm_setr_ps(m00, m01, m02, 0);
        m_10_11_12_r1() = _pi_mm_setr_ps(m10, m11, m12, 0);
    }
    inline void Set(const float *M)
    {
        memcpy(&m00(), M, 12);
        memcpy(&m10(), M + 3, 12);
    }
    inline void Get(float *M) const
    {
        memcpy(M, &m00(), 12);
        memcpy(M + 3, &m10(), 12);
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x3f)); }
    inline void MakeZero2x2()
    {
        m00() = 0.0f;
        m01() = 0.0f;
        m10() = 0.0f;
        m11() = 0.0f;
    }

    inline void Scale(const _pi__m128 &s)
    {
        m_00_01_02_r0() = _pi_mm_mul_ps(s, m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_mul_ps(s, m_10_11_12_r1());
    }

    inline void AddATBTo(const Vector2f &B, AlignedVector3f &ATB) const
    {
        ATB.v012r() = _pi_mm_add_ps(
            _pi_mm_add_ps(
                _pi_mm_mul_ps(m_00_01_02_r0(), _pi_mm_set1_ps(B.v0())),
                _pi_mm_mul_ps(m_10_11_12_r1(), _pi_mm_set1_ps(B.v1()))),
            ATB.v012r());
    }

    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e\n", m00(), m01(), m02());
            UT::Print("%e %e %e\n", m10(), m11(), m12());
        } else {
            UT::Print("%f %f %f\n", m00(), m01(), m02());
            UT::Print("%f %f %f\n", m10(), m11(), m12());
        }
    }
    inline void AssertZero() const
    {
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_00_01_02_r0(), m_00_01_02_r0())), 0.0f);
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_10_11_12_r1(), m_10_11_12_r1())), 0.0f);
    }
    inline bool AssertEqual(const AlignedMatrix2x3f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 3, 0) &&
            UT::VectorAssertEqual(&m10(), &M.m10(), 3, 0))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

    inline void SetInfinite()
    {
        m_00_01_02_r0() = m_10_11_12_r1() = _pi_mm_set1_ps(FLT_MAX);
    }
    inline void AssertInfinite() const
    {
        UT_ASSERT(m00() == FLT_MAX);
        UT_ASSERT(m01() == FLT_MAX);
        UT_ASSERT(m02() == FLT_MAX);
        UT_ASSERT(r0() == FLT_MAX);
        UT_ASSERT(m10() == FLT_MAX);
        UT_ASSERT(m11() == FLT_MAX);
        UT_ASSERT(m12() == FLT_MAX);
        UT_ASSERT(r1() == FLT_MAX);
    }
    inline void AssertFinite() const
    {
        UT_ASSERT(m00() != FLT_MAX);
        UT_ASSERT(m01() != FLT_MAX);
        UT_ASSERT(m02() != FLT_MAX);
        UT_ASSERT(r0() != FLT_MAX);
        UT_ASSERT(m10() != FLT_MAX);
        UT_ASSERT(m11() != FLT_MAX);
        UT_ASSERT(m12() != FLT_MAX);
        UT_ASSERT(r1() != FLT_MAX);
    }

    static inline void AB(const SymmetricMatrix2x2f &A,
                          const AlignedMatrix2x3f &B1,
                          const AlignedMatrix2x3f &B2, AlignedMatrix2x3f &AB1,
                          AlignedMatrix2x3f &AB2)
    {
        const _pi__m128 a00 = _pi_mm_set1_ps(A.m00()),
                        a01 = _pi_mm_set1_ps(A.m01()),
                        a11 = _pi_mm_set1_ps(A.m11());
        AB1.m_00_01_02_r0() =
            _pi_mm_add_ps(_pi_mm_mul_ps(a00, B1.m_00_01_02_r0()),
                          _pi_mm_mul_ps(a01, B1.m_10_11_12_r1()));
        AB1.m_10_11_12_r1() =
            _pi_mm_add_ps(_pi_mm_mul_ps(a01, B1.m_00_01_02_r0()),
                          _pi_mm_mul_ps(a11, B1.m_10_11_12_r1()));
        AB2.m_00_01_02_r0() =
            _pi_mm_add_ps(_pi_mm_mul_ps(a00, B2.m_00_01_02_r0()),
                          _pi_mm_mul_ps(a01, B2.m_10_11_12_r1()));
        AB2.m_10_11_12_r1() =
            _pi_mm_add_ps(_pi_mm_mul_ps(a01, B2.m_00_01_02_r0()),
                          _pi_mm_mul_ps(a11, B2.m_10_11_12_r1()));
    }

  protected:
    _pi__m128 m_data[2];
};
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x3f : public Eigen::Matrix<float, 2, 3>
{
  public:
    inline EigenMatrix2x3f() = default;
    inline EigenMatrix2x3f(const Eigen::Matrix<float, 2, 3> &e_M)
        : Eigen::Matrix<float, 2, 3>(e_M)
    {
    }
    inline EigenMatrix2x3f(const LA::AlignedMatrix2x3f &M)
        : Eigen::Matrix<float, 2, 3>()
    {
        Eigen::Matrix<float, 2, 3> &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(0, 2) = M.m02();
        e_M(1, 0) = M.m10();
        e_M(1, 1) = M.m11();
        e_M(1, 2) = M.m12();
    }
    inline void operator=(const Eigen::Matrix<float, 2, 3> &e_M)
    {
        *((Eigen::Matrix<float, 2, 3> *)this) = e_M;
    }
    inline LA::AlignedMatrix2x3f GetAlignedMatrix2x3f() const
    {
        LA::AlignedMatrix2x3f M;
        const Eigen::Matrix<float, 2, 3> &e_M = *this;
        M.m00() = e_M(0, 0);
        M.m01() = e_M(0, 1);
        M.m02() = e_M(0, 2);
        M.m10() = e_M(1, 0);
        M.m11() = e_M(1, 1);
        M.m12() = e_M(1, 2);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix2x3f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix2x3f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix2x3f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const EigenMatrix2x3f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix2x3f(), verbose);
    }
};
#endif
#endif
