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

#ifndef _VECTOR_8_H_
#define _VECTOR_8_H_

#include "Utility.h"

namespace LA
{
class AlignedVector8f
{
  public:
    inline AlignedVector8f() = default;
    inline AlignedVector8f(const float *v) { Set(v); }
    inline const _pi__m128 &v0123() const { return m_data[0]; }
    inline _pi__m128 &v0123() { return m_data[0]; }
    inline const _pi__m128 &v4567() const { return m_data[1]; }
    inline _pi__m128 &v4567() { return m_data[1]; }
    inline const float &v0() const { return m_data[0].m128_f32[0]; }
    inline float &v0() { return m_data[0].m128_f32[0]; }
    inline const float &v1() const { return m_data[0].m128_f32[1]; }
    inline float &v1() { return m_data[0].m128_f32[1]; }
    inline const float &v2() const { return m_data[0].m128_f32[2]; }
    inline float &v2() { return m_data[0].m128_f32[2]; }
    inline const float &v3() const { return m_data[0].m128_f32[3]; }
    inline float &v3() { return m_data[0].m128_f32[3]; }
    inline const float &v4() const { return m_data[1].m128_f32[0]; }
    inline float &v4() { return m_data[1].m128_f32[0]; }
    inline const float &v5() const { return m_data[1].m128_f32[1]; }
    inline float &v5() { return m_data[1].m128_f32[1]; }
    inline const float &v6() const { return m_data[1].m128_f32[2]; }
    inline float &v6() { return m_data[1].m128_f32[2]; }
    inline const float &v7() const { return m_data[1].m128_f32[3]; }
    inline float &v7() { return m_data[1].m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator+=(const AlignedVector8f &v)
    {
        v0123() = _pi_mm_add_ps(v.v0123(), v0123());
        v4567() = _pi_mm_add_ps(v.v4567(), v4567());
    }
    inline void operator-=(const AlignedVector8f &v)
    {
        v0123() = _pi_mm_sub_ps(v0123(), v.v0123());
        v4567() = _pi_mm_sub_ps(v4567(), v.v4567());
    }
    inline void operator*=(const float s) { Scale(s); }
    inline void operator*=(const _pi__m128 &s) { Scale(s); }
    inline AlignedVector8f operator*(const _pi__m128 &s) const
    {
        AlignedVector8f v;
        GetScaled(s, v);
        return v;
    }

    inline void Set(const float *v)
    {
        memcpy(this, v, sizeof(AlignedVector8f));
    }
    inline void Get(float *v) const
    {
        memcpy(v, this, sizeof(AlignedVector8f));
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedVector8f)); }
    inline void MakeMinus()
    {
        v0123() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v0123());
        v4567() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v4567());
    }

    inline void Scale(const float s)
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        Scale(_s);
    }
    inline void Scale(const _pi__m128 &s)
    {
        v0123() = _pi_mm_mul_ps(s, v0123());
        v4567() = _pi_mm_mul_ps(s, v4567());
    }
    inline void GetScaled(const _pi__m128 &s, AlignedVector8f &v) const
    {
        v.v0123() = _pi_mm_mul_ps(s, v0123());
        v.v4567() = _pi_mm_mul_ps(s, v4567());
    }
    inline float SquaredLength() const
    {
        return SSE::Sum(_pi_mm_add_ps(_pi_mm_mul_ps(v0123(), v0123()),
                                      _pi_mm_mul_ps(v4567(), v4567())));
    }
    inline float Dot(const AlignedVector8f &v) const
    {
        return SSE::Sum(_pi_mm_add_ps(_pi_mm_mul_ps(v0123(), v.v0123()),
                                      _pi_mm_mul_ps(v4567(), v.v4567())));
    }

    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("%e %e %e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(),
                      v5(), v6(), v7());
        else
            UT::Print("%f %f %f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(),
                      v5(), v6(), v7());
    }

    static inline void apb(const AlignedVector8f &a, const AlignedVector8f &b,
                           AlignedVector8f &apb)
    {
        apb.v0123() = _pi_mm_add_ps(a.v0123(), b.v0123());
        apb.v4567() = _pi_mm_add_ps(a.v4567(), b.v4567());
    }

    inline void Random(const float mMax) { UT::Random(&v0(), 8, -mMax, mMax); }
    static inline AlignedVector8f GetRandom(const float mMax)
    {
        AlignedVector8f v;
        v.Random(mMax);
        return v;
    }

    inline void AssertZero() const { UT::AssertEqual(SquaredLength(), 0.0f); }
    inline bool AssertEqual(const LA::AlignedVector8f &v,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&v0(), &v.v0(), 8, 0)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
        }
        return false;
    }

  protected:
    _pi__m128 m_data[2];
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector8f : public Eigen::Matrix<float, 8, 1>
{
  public:
    inline EigenVector8f() = default;
    inline EigenVector8f(const Eigen::Matrix<float, 8, 1> &e_v)
        : Eigen::Matrix<float, 8, 1>(e_v)
    {
    }
    inline EigenVector8f(const LA::AlignedVector8f &v)
        : Eigen::Matrix<float, 8, 1>()
    {
        const float *_v = v;
        Eigen::Matrix<float, 8, 1> &e_v = *this;
        for (int i = 0; i < 8; ++i) e_v(i, 0) = _v[i];
    }
    inline void operator=(const Eigen::Matrix<float, 8, 1> &e_v)
    {
        *((Eigen::Matrix<float, 8, 1> *)this) = e_v;
    }
    inline LA::AlignedVector8f GetAlignedVector8f() const
    {
        LA::AlignedVector8f v;
        float *_v = v;
        const Eigen::Matrix<float, 8, 1> &e_v = *this;
        for (int i = 0; i < 8; ++i) _v[i] = e_v(i, 0);
        return v;
    }
    inline float SquaredLength() const
    {
        return GetAlignedVector8f().SquaredLength();
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedVector8f().Print(e);
    }
    static inline EigenVector8f GetRandom(const float mMax)
    {
        return EigenVector8f(LA::AlignedVector8f::GetRandom(mMax));
    }
    inline bool AssertEqual(const LA::AlignedVector8f &v,
                            const int verbose = 1) const
    {
        return GetAlignedVector8f().AssertEqual(v, verbose);
    }
    inline bool AssertEqual(const EigenVector8f &e_v,
                            const int verbose = 1) const
    {
        return AssertEqual(e_v.GetAlignedVector8f(), verbose);
    }
};
#endif
#endif
