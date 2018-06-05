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

#ifndef _VECTOR_4_H_
#define _VECTOR_4_H_

#include "Utility.h"

namespace LA
{
class AlignedVector4f
{
  public:
    inline const _pi__m128 &v0123() const { return m_data; }
    inline _pi__m128 &v0123() { return m_data; }
    inline const float &v0() const { return m_data.m128_f32[0]; }
    inline float &v0() { return m_data.m128_f32[0]; }
    inline const float &v1() const { return m_data.m128_f32[1]; }
    inline float &v1() { return m_data.m128_f32[1]; }
    inline const float &v2() const { return m_data.m128_f32[2]; }
    inline float &v2() { return m_data.m128_f32[2]; }
    inline const float &v3() const { return m_data.m128_f32[3]; }
    inline float &v3() { return m_data.m128_f32[3]; }
    inline const _pi__m128 &xyzw() const { return v0123(); }
    inline _pi__m128 &xyzw() { return v0123(); }
    inline const float &x() const { return v0(); }
    inline float &x() { return v0(); }
    inline const float &y() const { return v1(); }
    inline float &y() { return v1(); }
    inline const float &z() const { return v2(); }
    inline float &z() { return v2(); }
    inline const float &w() const { return v3(); }
    inline float &w() { return v3(); }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline bool operator==(const AlignedVector4f &v) const
    {
        return v0() == v.v0() && v1() == v.v1() && v2() == v.v2() &&
               v3() == v.v3();
    }
    inline void operator*=(const float s) { Scale(s); }
    inline AlignedVector4f operator-(const AlignedVector4f &b) const
    {
        AlignedVector4f amb;
        amb.v0123() = _pi_mm_sub_ps(v0123(), b.v0123());
        return amb;
    }

    inline void Set(const float *v) { memcpy(this, v, 16); }
    inline void Set(const double *v)
    {
        v0123() =
            _pi_mm_setr_ps(float(v[0]), float(v[1]), float(v[2]), float(v[3]));
    }
    inline void Get(float *v) const { memcpy(v, this, 16); }
    inline void MakeZero() { memset(this, 0, sizeof(AlignedVector4f)); }
    inline void MakeMinus()
    {
        v0123() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v0123());
    }
    inline AlignedVector4f GetMinus() const
    {
        AlignedVector4f v;
        v.v0123() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v0123());
        return v;
    }

    inline bool Valid() const { return v0() != FLT_MAX; }
    inline bool Invalid() const { return v0() == FLT_MAX; }
    inline void Invalidate() { v0() = FLT_MAX; }
    inline void Scale(const float s)
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        Scale(_s);
    }
    inline void Scale(const _pi__m128 &s)
    {
        v0123() = _pi_mm_mul_ps(s, v0123());
    }
    inline float SquaredLength() const
    {
        return SSE::Sum(_pi_mm_mul_ps(v0123(), v0123()));
    }
    inline float Dot(const AlignedVector4f &v) const
    {
        return SSE::Sum(_pi_mm_mul_ps(v0123(), v.v0123()));
    }

    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("%e %e %e %e\n", v0(), v1(), v2(), v3());
        else
            UT::Print("%f %f %f %f\n", v0(), v1(), v2(), v3());
    }
    inline void Save(FILE *fp, const bool e = false) const
    {
        if (e)
            fprintf(fp, "%e %e %e %e\n", v0(), v1(), v2(), v3());
        else
            fprintf(fp, "%f %f %f %f\n", v0(), v1(), v2(), v3());
    }
    inline void Load(FILE *fp)
    {
#ifdef CFG_DEBUG
        const int N = fscanf(fp, "%f %f %f %f", &v0(), &v1(), &v2(), &v3());
        UT_ASSERT(N == 4);
#else
        fscanf(fp, "%f %f %f %f", &v0(), &v1(), &v2(), &v3());
#endif
    }

    inline void Random(const float mMax) { UT::Random(&v0(), 4, -mMax, mMax); }
    static inline AlignedVector4f GetRandom(const float mMax)
    {
        AlignedVector4f v;
        v.Random(mMax);
        return v;
    }

    inline void AssertZero() const { UT::AssertEqual(SquaredLength(), 0.0f); }
    inline bool AssertEqual(const AlignedVector4f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&v0(), &v.v0(), 4, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
        }
        return false;
    }

    inline void SetInfinite() { v0123() = _pi_mm_set1_ps(FLT_MAX); }
    inline void AssertInfinite() const
    {
        UT_ASSERT(v0() == FLT_MAX);
        UT_ASSERT(v1() == FLT_MAX);
        UT_ASSERT(v2() == FLT_MAX);
        UT_ASSERT(v3() == FLT_MAX);
    }
    inline void AssertFinite() const
    {
        UT_ASSERT(v0() != FLT_MAX);
        UT_ASSERT(v1() != FLT_MAX);
        UT_ASSERT(v2() != FLT_MAX);
        UT_ASSERT(v3() != FLT_MAX);
    }

  protected:
    _pi__m128 m_data;
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector4f : public Eigen::Vector4f
{
  public:
    inline EigenVector4f() = default;
    inline EigenVector4f(const Eigen::Vector4f &e_v) : Eigen::Vector4f(e_v) {}
    inline EigenVector4f(const LA::AlignedVector4f &v)
        : Eigen::Vector4f(v.v0(), v.v1(), v.v2(), v.v3())
    {
    }
    inline EigenVector4f(const float v0, const float v1, const float v2,
                         const float v3)
        : Eigen::Vector4f(v0, v1, v2, v3)
    {
    }
    inline void operator=(const Eigen::Vector4f &e_v)
    {
        *((Eigen::Vector4f *)this) = e_v;
    }
    inline LA::AlignedVector4f GetAlignedVector4f() const
    {
        LA::AlignedVector4f v;
        const Eigen::Vector4f &e_v = *this;
        v.v0() = e_v(0);
        v.v1() = e_v(1);
        v.v2() = e_v(2);
        v.v3() = e_v(3);
        return v;
    }
    inline float SquaredLength() const
    {
        return GetAlignedVector4f().SquaredLength();
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedVector4f().Print(e);
    }
    static inline EigenVector4f GetRandom(const float mMax)
    {
        return EigenVector4f(LA::AlignedVector4f::GetRandom(mMax));
    }
    inline bool AssertEqual(const LA::AlignedVector4f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetAlignedVector4f().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const EigenVector4f &e_v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_v.GetAlignedVector4f(), verbose, eps);
    }
};
#endif
#endif
