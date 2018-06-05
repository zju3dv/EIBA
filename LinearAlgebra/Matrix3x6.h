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

#ifndef _MATRIX_3x6_H_
#define _MATRIX_3x6_H_

#include "Matrix2x3.h"
#include "Matrix3x3.h"

namespace LA
{
class AlignedMatrix3x6f
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
    inline const _pi__m128 &m_20_21_22_23() const { return m_data[4]; }
    inline _pi__m128 &m_20_21_22_23() { return m_data[4]; }
    inline const _pi__m128 &m_24_25_x_x() const { return m_data[5]; }
    inline _pi__m128 &m_24_25_x_x() { return m_data[5]; }
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
    inline const float &m20() const { return m_data[4].m128_f32[0]; }
    inline float &m20() { return m_data[4].m128_f32[0]; }
    inline const float &m21() const { return m_data[4].m128_f32[1]; }
    inline float &m21() { return m_data[4].m128_f32[1]; }
    inline const float &m22() const { return m_data[4].m128_f32[2]; }
    inline float &m22() { return m_data[4].m128_f32[2]; }
    inline const float &m23() const { return m_data[4].m128_f32[3]; }
    inline float &m23() { return m_data[4].m128_f32[3]; }
    inline const float &m24() const { return m_data[5].m128_f32[0]; }
    inline float &m24() { return m_data[5].m128_f32[0]; }
    inline const float &m25() const { return m_data[5].m128_f32[1]; }
    inline float &m25() { return m_data[5].m128_f32[1]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix3x6f)); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(),
                      m05());
            UT::Print("%e %e %e %e %e %e\n", m10(), m11(), m12(), m13(), m14(),
                      m15());
            UT::Print("%e %e %e %e %e %e\n", m20(), m21(), m22(), m23(), m24(),
                      m25());
        } else {
            UT::Print("%f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(),
                      m05());
            UT::Print("%f %f %f %f %f %f\n", m10(), m11(), m12(), m13(), m14(),
                      m15());
            UT::Print("%f %f %f %f %f %f\n", m20(), m21(), m22(), m23(), m24(),
                      m25());
        }
    }
    inline bool AssertEqual(const AlignedMatrix3x6f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 6, 0) &&
            UT::VectorAssertEqual(&m10(), &M.m10(), 6, 0) &&
            UT::VectorAssertEqual(&m20(), &M.m20(), 6, 0))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

    static inline void AB(const SymmetricMatrix3x3f &A,
                          const AlignedMatrix3x6f &B, AlignedMatrix3x6f &AB)
    {
        const _pi__m128 a01 = _pi_mm_set1_ps(A.m01()),
                        a02 = _pi_mm_set1_ps(A.m02()),
                        a12 = _pi_mm_set1_ps(A.m12());
        AB.m_00_01_02_03() = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(A.m00()), B.m_00_01_02_03()),
            _pi_mm_add_ps(_pi_mm_mul_ps(a01, B.m_10_11_12_13()),
                          _pi_mm_mul_ps(a02, B.m_20_21_22_23())));
        AB.m04() = A.m00() * B.m04() + A.m01() * B.m14() + A.m02() * B.m24();
        AB.m05() = A.m00() * B.m05() + A.m01() * B.m15() + A.m02() * B.m25();
        AB.m_10_11_12_13() =
            _pi_mm_add_ps(_pi_mm_mul_ps(a01, B.m_00_01_02_03()),
                          _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(A.m11()),
                                                      B.m_10_11_12_13()),
                                        _pi_mm_mul_ps(a12, B.m_20_21_22_23())));
        AB.m14() = A.m01() * B.m04() + A.m11() * B.m14() + A.m12() * B.m24();
        AB.m15() = A.m01() * B.m05() + A.m11() * B.m15() + A.m12() * B.m25();
        AB.m_20_21_22_23() =
            _pi_mm_add_ps(_pi_mm_mul_ps(a02, B.m_00_01_02_03()),
                          _pi_mm_add_ps(_pi_mm_mul_ps(a12, B.m_10_11_12_13()),
                                        _pi_mm_mul_ps(_pi_mm_set1_ps(A.m22()),
                                                      B.m_20_21_22_23())));
        AB.m24() = A.m02() * B.m04() + A.m12() * B.m14() + A.m22() * B.m24();
        AB.m25() = A.m02() * B.m05() + A.m12() * B.m15() + A.m22() * B.m25();
    }
    static inline void AB(const SymmetricMatrix2x2f &A0, const float A1,
                          const AlignedMatrix3x6f &B, AlignedMatrix3x6f &AB)
    {
        const _pi__m128 a01 = _pi_mm_set1_ps(A0.m01());
        AB.m_00_01_02_03() = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(A0.m00()), B.m_00_01_02_03()),
            _pi_mm_mul_ps(a01, B.m_10_11_12_13()));
        AB.m04() = A0.m00() * B.m04() + A0.m01() * B.m14();
        AB.m05() = A0.m00() * B.m05() + A0.m01() * B.m15();
        AB.m_10_11_12_13() = _pi_mm_add_ps(
            _pi_mm_mul_ps(a01, B.m_00_01_02_03()),
            _pi_mm_mul_ps(_pi_mm_set1_ps(A0.m11()), B.m_10_11_12_13()));
        AB.m14() = A0.m01() * B.m04() + A0.m11() * B.m14();
        AB.m15() = A0.m01() * B.m05() + A0.m11() * B.m15();
        AB.m_20_21_22_23() =
            _pi_mm_mul_ps(_pi_mm_set1_ps(A1), B.m_20_21_22_23());
        AB.m24() = A1 * B.m24();
        AB.m25() = A1 * B.m25();
    }

  protected:
    _pi__m128 m_data[6];
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenMatrix3x6f : public Eigen::Matrix<float, 3, 6>
{
  public:
    inline EigenMatrix3x6f() = default;
    inline EigenMatrix3x6f(const Eigen::Matrix<float, 3, 6> &e_M)
        : Eigen::Matrix<float, 3, 6>(e_M)
    {
    }
    inline EigenMatrix3x6f(const LA::AlignedMatrix3x6f &M)
        : Eigen::Matrix<float, 3, 6>()
    {
        const float *_M[3] = {&M.m00(), &M.m10(), &M.m20()};
        Eigen::Matrix<float, 3, 6> &e_M = *this;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 6; ++j) e_M(i, j) = _M[i][j];
    }
    inline EigenMatrix3x6f(const EigenMatrix3x3f &e_M0,
                           const EigenMatrix3x3f &e_M1)
    {
        block<3, 3>(0, 0) = e_M0;
        block<3, 3>(0, 3) = e_M1;
    }
    inline EigenMatrix3x6f(const EigenMatrix2x3f &e_M00,
                           const EigenMatrix2x3f &e_M01,
                           const Eigen::Matrix<float, 1, 3> &e_M10,
                           const Eigen::Matrix<float, 1, 3> &e_M11)
    {
        block<2, 3>(0, 0) = e_M00;
        block<2, 3>(0, 3) = e_M01;
        block<1, 3>(2, 0) = e_M10;
        block<1, 3>(2, 3) = e_M11;
    }
    inline void operator=(const Eigen::Matrix<float, 3, 6> &e_M)
    {
        *((Eigen::Matrix<float, 3, 6> *)this) = e_M;
    }
    inline LA::AlignedMatrix3x6f GetAlignedMatrix3x6f() const
    {
        LA::AlignedMatrix3x6f M;
        float *_M[3] = {&M.m00(), &M.m10(), &M.m20()};
        const Eigen::Matrix<float, 3, 6> &e_M = *this;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 6; ++j) _M[i][j] = e_M(i, j);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix3x6f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix3x6f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix3x6f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const EigenMatrix3x6f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix3x6f(), verbose);
    }
};
#endif
#endif
