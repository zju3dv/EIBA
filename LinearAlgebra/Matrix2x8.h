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

#ifndef _MATRIX_2x8_H_
#define _MATRIX_2x8_H_

#include "Vector8.h"

namespace LA
{
class AlignedMatrix2x8f
{
  public:
    inline const _pi__m128 &m_00_01_02_03() const { return m_data[0]; }
    inline _pi__m128 &m_00_01_02_03() { return m_data[0]; }
    inline const _pi__m128 &m_04_05_06_07() const { return m_data[1]; }
    inline _pi__m128 &m_04_05_06_07() { return m_data[1]; }
    inline const _pi__m128 &m_10_11_12_13() const { return m_data[2]; }
    inline _pi__m128 &m_10_11_12_13() { return m_data[2]; }
    inline const _pi__m128 &m_14_15_16_17() const { return m_data[3]; }
    inline _pi__m128 &m_14_15_16_17() { return m_data[3]; }
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
    inline const float &m06() const { return m_data[1].m128_f32[2]; }
    inline float &m06() { return m_data[1].m128_f32[2]; }
    inline const float &m07() const { return m_data[1].m128_f32[3]; }
    inline float &m07() { return m_data[1].m128_f32[3]; }
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
    inline const float &m16() const { return m_data[3].m128_f32[2]; }
    inline float &m16() { return m_data[3].m128_f32[2]; }
    inline const float &m17() const { return m_data[3].m128_f32[3]; }
    inline float &m17() { return m_data[3].m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator+=(const AlignedMatrix2x8f &M)
    {
        m_00_01_02_03() = _pi_mm_add_ps(M.m_00_01_02_03(), m_00_01_02_03());
        m_04_05_06_07() = _pi_mm_add_ps(M.m_04_05_06_07(), m_04_05_06_07());
        m_10_11_12_13() = _pi_mm_add_ps(M.m_10_11_12_13(), m_10_11_12_13());
        m_14_15_16_17() = _pi_mm_add_ps(M.m_14_15_16_17(), m_14_15_16_17());
    }
    inline void operator-=(const AlignedMatrix2x8f &M)
    {
        m_00_01_02_03() = _pi_mm_sub_ps(m_00_01_02_03(), M.m_00_01_02_03());
        m_04_05_06_07() = _pi_mm_sub_ps(m_04_05_06_07(), M.m_04_05_06_07());
        m_10_11_12_13() = _pi_mm_sub_ps(m_10_11_12_13(), M.m_10_11_12_13());
        m_14_15_16_17() = _pi_mm_sub_ps(m_14_15_16_17(), M.m_14_15_16_17());
    }
    inline void operator*=(const _pi__m128 &s)
    {
        m_00_01_02_03() = _pi_mm_mul_ps(s, m_00_01_02_03());
        m_04_05_06_07() = _pi_mm_mul_ps(s, m_04_05_06_07());
        m_10_11_12_13() = _pi_mm_mul_ps(s, m_10_11_12_13());
        m_14_15_16_17() = _pi_mm_mul_ps(s, m_14_15_16_17());
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x8f)); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e %e %e %e %e\n", m00(), m01(), m02(), m03(),
                      m04(), m05(), m06(), m07());
            UT::Print("%e %e %e %e %e %e %e %e\n", m10(), m11(), m12(), m13(),
                      m14(), m15(), m16(), m17());
        } else {
            UT::Print("%f %f %f %f %f %f %f %f\n", m00(), m01(), m02(), m03(),
                      m04(), m05(), m06(), m07());
            UT::Print("%f %f %f %f %f %f %f %f\n", m10(), m11(), m12(), m13(),
                      m14(), m15(), m16(), m17());
        }
    }
    inline void AssertZero() const
    {
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_00_01_02_03(), m_00_01_02_03())), 0.0f);
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_04_05_06_07(), m_04_05_06_07())), 0.0f);
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_10_11_12_13(), m_10_11_12_13())), 0.0f);
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_14_15_16_17(), m_14_15_16_17())), 0.0f);
    }
    inline bool AssertEqual(const AlignedMatrix2x8f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 16, 0)) return true;
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
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x8f : public Eigen::Matrix<float, 2, 8>
{
  public:
    inline EigenMatrix2x8f() = default;
    inline EigenMatrix2x8f(const Eigen::Matrix<float, 2, 8> &e_M)
        : Eigen::Matrix<float, 2, 8>(e_M)
    {
    }
    inline EigenMatrix2x8f(const LA::AlignedMatrix2x8f &M)
        : Eigen::Matrix<float, 2, 8>()
    {
        const float *_M[2] = {&M.m00(), &M.m10()};
        Eigen::Matrix<float, 2, 8> &e_M = *this;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 8; ++j) e_M(i, j) = _M[i][j];
    }
    inline void operator=(const Eigen::Matrix<float, 2, 8> &e_M)
    {
        *((Eigen::Matrix<float, 2, 8> *)this) = e_M;
    }
    inline LA::AlignedMatrix2x8f GetAlignedMatrix2x8f() const
    {
        LA::AlignedMatrix2x8f M;
        float *_M[2] = {&M.m00(), &M.m10()};
        const Eigen::Matrix<float, 2, 8> &e_M = *this;
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 8; ++j) _M[i][j] = e_M(i, j);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix2x8f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix2x8f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix2x8f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const EigenMatrix2x8f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix2x8f(), verbose);
    }
};
#endif
#endif
