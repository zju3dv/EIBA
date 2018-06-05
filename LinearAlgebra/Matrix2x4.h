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

#ifndef _MATRIX_2x4_H_
#define _MATRIX_2x4_H_

#include "Utility.h"

namespace LA
{
class AlignedMatrix2x4f
{
  public:
    inline const _pi__m128 &m_00_01_02_03() const { return m_data[0]; }
    inline _pi__m128 &m_00_01_02_03() { return m_data[0]; }
    inline const _pi__m128 &m_10_11_12_13() const { return m_data[1]; }
    inline _pi__m128 &m_10_11_12_13() { return m_data[1]; }
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
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator+=(const AlignedMatrix2x4f &M)
    {
        m_data[0] = _pi_mm_add_ps(M.m_data[0], m_data[0]);
        m_data[1] = _pi_mm_add_ps(M.m_data[1], m_data[1]);
    }
    inline void operator-=(const AlignedMatrix2x4f &M)
    {
        m_data[0] = _pi_mm_sub_ps(m_data[0], M.m_data[0]);
        m_data[1] = _pi_mm_sub_ps(m_data[1], M.m_data[1]);
    }
    inline void operator*=(const _pi__m128 &s)
    {
        m_data[0] = _pi_mm_mul_ps(s, m_data[0]);
        m_data[1] = _pi_mm_mul_ps(s, m_data[1]);
    }

    inline void Set(const float m00, const float m01, const float m02,
                    const float m03, const float m10, const float m11,
                    const float m12, const float m13)
    {
        m_data[0] = _pi_mm_setr_ps(m00, m01, m02, m03);
        m_data[1] = _pi_mm_setr_ps(m10, m11, m12, m13);
    }
    inline void Set(const float *M)
    {
        memcpy(&m00(), M, sizeof(AlignedMatrix2x4f));
    }
    inline void Get(float *M) const
    {
        memcpy(M, &m00(), sizeof(AlignedMatrix2x4f));
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x4f)); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e\n", m00(), m01(), m02(), m03());
            UT::Print("%e %e %e %e\n", m10(), m11(), m12(), m13());
        } else {
            UT::Print("%f %f %f %f\n", m00(), m01(), m02(), m03());
            UT::Print("%f %f %f %f\n", m10(), m11(), m12(), m13());
        }
    }
    inline void AssertZero() const
    {
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_00_01_02_03(), m_00_01_02_03())), 0.0f);
        UT::AssertEqual(
            SSE::Sum012(_pi_mm_mul_ps(m_10_11_12_13(), m_10_11_12_13())), 0.0f);
    }
    inline bool AssertEqual(const AlignedMatrix2x4f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 8, 0)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

  protected:
    _pi__m128 m_data[2];
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenMatrix2x4f : public Eigen::Matrix<float, 2, 4>
{
  public:
    inline EigenMatrix2x4f() = default;
    inline EigenMatrix2x4f(const Eigen::Matrix<float, 2, 4> &e_M)
        : Eigen::Matrix<float, 2, 4>(e_M)
    {
    }
    inline EigenMatrix2x4f(const LA::AlignedMatrix2x4f &M)
        : Eigen::Matrix<float, 2, 4>()
    {
        Eigen::Matrix<float, 2, 4> &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(0, 2) = M.m02();
        e_M(0, 3) = M.m03();
        e_M(1, 0) = M.m10();
        e_M(1, 1) = M.m11();
        e_M(0, 2) = M.m02();
        e_M(0, 3) = M.m03();
    }
    inline void operator=(const Eigen::Matrix<float, 2, 4> &e_M)
    {
        *((Eigen::Matrix<float, 2, 4> *)this) = e_M;
    }
    inline LA::AlignedMatrix2x4f GetAlignedMatrix2x4f() const
    {
        LA::AlignedMatrix2x4f M;
        const Eigen::Matrix<float, 2, 4> &e_M = *this;
        M.m00() = e_M(0, 0);
        M.m01() = e_M(0, 1);
        M.m02() = e_M(0, 2);
        M.m03() = e_M(0, 3);
        M.m10() = e_M(1, 0);
        M.m11() = e_M(1, 1);
        M.m12() = e_M(1, 2);
        M.m13() = e_M(1, 3);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix2x4f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix2x4f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix2x4f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const EigenMatrix2x4f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix2x4f(), verbose);
    }
};
#endif
#endif
