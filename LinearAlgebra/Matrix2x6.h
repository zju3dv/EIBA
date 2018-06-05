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

#ifndef _MATRIX_2x6_H_
#define _MATRIX_2x6_H_

#include "Matrix2x3.h"
#include "Vector6.h"

namespace LA
{
class AlignedMatrix2x6f
{
  public:
    inline const _pi__m128 &m_00_01_02_03() const { return m_data[0]; }
    inline _pi__m128 &m_00_01_02_03() { return m_data[0]; }
    inline const _pi__m128 &m_04_05_x_x() const { return m_data[1]; }
    inline _pi__m128 &m_04_05_x_x() { return m_data[1]; }
    inline const _pi__m128 &m_10_11_12_13() const { return m_data[2]; }
    inline _pi__m128 &m_10_11_12_13() { return m_data[2]; }
    inline const _pi__m128 &m_14_15_x_x() const { return m_data[3]; }
    inline _pi__m128 &m_14_15_x_x() { return m_data[3]; }
    inline const float &m00() const { return m_data[0].m128_f32[0]; }
    inline float &m00() { return m_data[0].m128_f32[0]; }
    inline const float &m01() const { return m_data[0].m128_f32[1]; }
    inline float &m01() { return m_data[0].m128_f32[1]; }
    inline const float &m02() const { return m_data[0].m128_f32[2]; }
    inline float &m02() { return m_data[0].m128_f32[2]; }
    inline const float &m03() const { return m_data[0].m128_f32[3]; }
    inline float &m03() { return m_data[0].m128_f32[3]; }
    inline const float &m04() const { return m_data[1].m128_f32[0]; }
    inline float &m04() { return m_data[1].m128_f32[0]; }
    inline const float &m05() const { return m_data[1].m128_f32[1]; }
    inline float &m05() { return m_data[1].m128_f32[1]; }
    inline const float &m10() const { return m_data[2].m128_f32[0]; }
    inline float &m10() { return m_data[2].m128_f32[0]; }
    inline const float &m11() const { return m_data[2].m128_f32[1]; }
    inline float &m11() { return m_data[2].m128_f32[1]; }
    inline const float &m12() const { return m_data[2].m128_f32[2]; }
    inline float &m12() { return m_data[2].m128_f32[2]; }
    inline const float &m13() const { return m_data[2].m128_f32[3]; }
    inline float &m13() { return m_data[2].m128_f32[3]; }
    inline const float &m14() const { return m_data[3].m128_f32[0]; }
    inline float &m14() { return m_data[3].m128_f32[0]; }
    inline const float &m15() const { return m_data[3].m128_f32[1]; }
    inline float &m15() { return m_data[3].m128_f32[1]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void Set(const AlignedMatrix2x3f &M1, AlignedMatrix2x3f &M2)
    {
        memcpy(&m00(), &M1.m00(), 12);
        memcpy(&m03(), &M2.m00(), 12);
        memcpy(&m10(), &M1.m10(), 12);
        memcpy(&m13(), &M2.m10(), 12);
    }
    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x6f)); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(),
                      m05());
            UT::Print("%e %e %e %e %e %e\n", m10(), m11(), m12(), m13(), m14(),
                      m15());
        } else {
            UT::Print("%f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(),
                      m05());
            UT::Print("%f %f %f %f %f %f\n", m10(), m11(), m12(), m13(), m14(),
                      m15());
        }
    }
    inline bool AssertEqual(const AlignedMatrix2x6f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 6, 0) &&
            UT::VectorAssertEqual(&m10(), &M.m10(), 6, 0))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

    static inline void AB(const SymmetricMatrix2x2f &A,
                          const AlignedMatrix2x6f &B, AlignedMatrix2x6f &AB)
    {
        const _pi__m128 a01 = _pi_mm_set1_ps(A.m01());
        AB.m_00_01_02_03() = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(A.m00()), B.m_00_01_02_03()),
            _pi_mm_mul_ps(a01, B.m_10_11_12_13()));
        AB.m04() = A.m00() * B.m04() + A.m01() * B.m14();
        AB.m05() = A.m00() * B.m05() + A.m01() * B.m15();
        AB.m_10_11_12_13() = _pi_mm_add_ps(
            _pi_mm_mul_ps(a01, B.m_00_01_02_03()),
            _pi_mm_mul_ps(_pi_mm_set1_ps(A.m11()), B.m_10_11_12_13()));
        AB.m14() = A.m01() * B.m04() + A.m11() * B.m14();
        AB.m15() = A.m01() * B.m05() + A.m11() * B.m15();
    }

    static inline void AddATBTo(const AlignedMatrix2x6f &A, Vector2f &B,
                                AlignedVector6f &ATB)
    {
        ATB.v0123() = _pi_mm_add_ps(
            _pi_mm_add_ps(
                _pi_mm_mul_ps(A.m_00_01_02_03(), _pi_mm_set1_ps(B.v0())),
                _pi_mm_mul_ps(A.m_10_11_12_13(), _pi_mm_set1_ps(B.v1()))),
            ATB.v0123());
        ATB.v4() = A.m04() * B.v0() + A.m14() + B.v1() + ATB.v4();
        ATB.v5() = A.m05() * B.v0() + A.m15() * B.v1() + ATB.v5();
    }

  protected:
    _pi__m128 m_data[4];
};
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x6f : public Eigen::Matrix<float, 2, 6>
{
  public:
    inline EigenMatrix2x6f() = default;
    inline EigenMatrix2x6f(const Eigen::Matrix<float, 2, 6> &e_M)
        : Eigen::Matrix<float, 2, 6>(e_M)
    {
    }
    inline EigenMatrix2x6f(const LA::AlignedMatrix2x6f &M)
        : Eigen::Matrix<float, 2, 6>()
    {
        const float *_M[2] = {&M.m00(), &M.m10()};
        Eigen::Matrix<float, 2, 6> &e_M = *this;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 6; ++j) e_M(i, j) = _M[i][j];
    }
    inline EigenMatrix2x6f(const EigenMatrix2x3f &M0, const EigenMatrix2x3f &M1)
    {
        block<2, 3>(0, 0) = M0;
        block<2, 3>(0, 3) = M1;
    }
    inline void operator=(const Eigen::Matrix<float, 2, 6> &e_M)
    {
        *((Eigen::Matrix<float, 2, 6> *)this) = e_M;
    }
    inline LA::AlignedMatrix2x6f GetAlignedMatrix2x6f() const
    {
        LA::AlignedMatrix2x6f M;
        float *_M[2] = {&M.m00(), &M.m10()};
        const Eigen::Matrix<float, 2, 6> &e_M = *this;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 6; ++j) _M[i][j] = e_M(i, j);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix2x6f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix2x6f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix2x6f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const EigenMatrix2x6f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix2x6f(), verbose);
    }
};
#endif
#endif
