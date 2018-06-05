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

#ifndef _VECTOR_6_H_
#define _VECTOR_6_H_

#include "Vector3.h"

namespace LA
{
class AlignedVector6f
{
  public:
    inline AlignedVector6f() = default;
    inline AlignedVector6f(const float *v) { Set(v); }
    inline const _pi__m128 &v0123() const { return m_data[0]; }
    inline _pi__m128 &v0123() { return m_data[0]; }
    inline const _pi__m128 &v45xx() const { return m_data[1]; }
    inline _pi__m128 &v45xx() { return m_data[1]; }
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
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator*=(const float s) { Scale(s); }
    inline void operator*=(const _pi__m128 &s) { Scale(s); }
    inline AlignedVector6f operator-(const AlignedVector6f &b) const
    {
        AlignedVector6f amb;
        amb.v0123() = _pi_mm_sub_ps(v0123(), b.v0123());
        amb.v4() = v4() - b.v4();
        amb.v5() = v5() - b.v5();
        return amb;
    }
    inline AlignedVector6f operator*(const _pi__m128 &s) const
    {
        AlignedVector6f v;
        GetScaled(s, v);
        return v;
    }

    inline void Set(const float *v)
    {
        memcpy(this, v, sizeof(AlignedVector6f));
    }
    inline void Set(const AlignedVector3f &v0, const AlignedVector3f &v1)
    {
        memcpy(&this->v0(), v0, 12);
        memcpy(&this->v3(), v1, 12);
    }
    inline void Get(float *v) const
    {
        memcpy(v, this, sizeof(AlignedVector6f));
    }

    template <class VECTOR> inline void GetBlock(const int i, VECTOR &v) const;

    inline void MakeZero() { memset(this, 0, sizeof(AlignedVector6f)); }
    inline void MakeMinus()
    {
        v0123() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v0123());
        v4() = -v4();
        v5() = -v5();
    }

    inline void Scale(const float s)
    {
        v0123() = _pi_mm_mul_ps(_pi_mm_set1_ps(s), v0123());
        v4() *= s;
        v5() *= s;
    }
    inline void Scale(const _pi__m128 &s)
    {
        v0123() = _pi_mm_mul_ps(s, v0123());
        v4() *= s.m128_f32[0];
        v5() *= s.m128_f32[0];
    }
    inline void GetScaled(const _pi__m128 &s, AlignedVector6f &v) const
    {
        v.v0123() = _pi_mm_mul_ps(s, v0123());
        v.v4() = s.m128_f32[0] * v4();
        v.v5() = s.m128_f32[0] * v5();
    }
    inline float SquaredLength() const
    {
        return SSE::Sum(_pi_mm_mul_ps(v0123(), v0123())) + v4() * v4() +
               v5() * v5();
    }
    inline float Dot(const AlignedVector6f &v) const
    {
        return SSE::Sum(_pi_mm_mul_ps(v0123(), v.v0123())) + v4() * v.v4() +
               v5() * v.v5();
    }
    inline float Dot(const float *v) const
    {
        return SSE::Sum(_pi_mm_mul_ps(v0123(), *((_pi__m128 *)v))) +
               v4() * v[4] + v5() * v[5];
    }

    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("%e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(),
                      v5());
        else
            UT::Print("%f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(),
                      v5());
    }

    inline void Random(const float mMax) { UT::Random(&v0(), 6, -mMax, mMax); }
    static inline AlignedVector6f GetRandom(const float mMax)
    {
        AlignedVector6f v;
        v.Random(mMax);
        return v;
    }

    inline void AssertZero() const { UT::AssertEqual(SquaredLength(), 0.0f); }
    inline bool AssertEqual(const LA::AlignedVector6f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&v0(), &v.v0(), 6, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
            const AlignedVector6f e = *this - v;
            e.Print(verbose > 1);
        }
        return false;
    }

  protected:
    _pi__m128 m_data[2];
};
template <>
inline void AlignedVector6f::GetBlock<AlignedVector3f>(const int i,
                                                       AlignedVector3f &v) const
{
    const float *_v = *this;
    memcpy(v, &_v[i], 12);
}

template <typename TYPE> class Vector6
{
  public:
    inline Vector6() = default;
    inline Vector6(const TYPE *v) { Set(v); }
    inline const TYPE &v0() const { return m_data[0]; }
    inline TYPE &v0() { return m_data[0]; }
    inline const TYPE &v1() const { return m_data[1]; }
    inline TYPE &v1() { return m_data[1]; }
    inline const TYPE &v2() const { return m_data[2]; }
    inline TYPE &v2() { return m_data[2]; }
    inline const TYPE &v3() const { return m_data[3]; }
    inline TYPE &v3() { return m_data[3]; }
    inline const TYPE &v4() const { return m_data[4]; }
    inline TYPE &v4() { return m_data[4]; }
    inline const TYPE &v5() const { return m_data[5]; }
    inline TYPE &v5() { return m_data[5]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void operator=(const TYPE *v) { Set(v); }
    inline Vector6<TYPE> operator*(const TYPE s) const
    {
        Vector6<TYPE> v;
        GetScaled(s, v);
        return v;
    }
    inline void operator+=(const Vector6<TYPE> &v)
    {
        v0() = v.v0() + v0();
        v1() = v.v1() + v1();
        v2() = v.v2() + v2();
        v3() = v.v3() + v3();
        v4() = v.v4() + v4();
        v5() = v.v5() + v5();
    }
    inline Vector6<TYPE> operator-(const Vector6<TYPE> &b) const
    {
        Vector6<TYPE> amb;
        amb.v0() = v0() - b.v0();
        amb.v1() = v1() - b.v1();
        amb.v2() = v2() - b.v2();
        amb.v3() = v3() - b.v3();
        amb.v4() = v4() - b.v4();
        amb.v5() = v5() - b.v5();
        return amb;
    }

    inline void Set(const TYPE *v) { memcpy(this, v, sizeof(Vector6<TYPE>)); }
    inline void Get(TYPE *v) const { memcpy(v, this, sizeof(Vector6<TYPE>)); }
    template <class VECTOR> inline void GetBlock(const int i, VECTOR &v) const;
    inline void GetBlock(const int i, Vector3<TYPE> &v) const
    {
        memcpy(v, &m_data[i], sizeof(Vector3<TYPE>()));
    }

    inline void MakeZero() { memset(this, 0, sizeof(Vector6<TYPE>)); }
    inline void Invalidate() { v0() = UT::Invalid<TYPE>(); }
    inline bool Valid() const { return v0() != UT::Invalid<TYPE>(); }
    inline bool Invalid() const { return v0() == UT::Invalid<TYPE>(); }
    inline void GetScaled(const TYPE s, Vector6<TYPE> &v) const
    {
        v.v0() = s * v0();
        v.v1() = s * v1();
        v.v2() = s * v2();
        v.v3() = s * v3();
        v.v4() = s * v4();
        v.v5() = s * v5();
    }

    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("%e %e %e %e %e %e\n", v0(), v1(), v2(), v3(), v4(),
                      v5());
        else
            UT::Print("%f %f %f %f %f %f\n", v0(), v1(), v2(), v3(), v4(),
                      v5());
    }

    inline bool AssertEqual(const Vector6<TYPE> &v, const int verbose = 1,
                            const TYPE eps = 0) const
    {
        if (UT::VectorAssertEqual<TYPE>(&v0(), &v.v0(), 6, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
            const Vector6<TYPE> e = *this - v;
            e.Print(verbose > 1);
        }
        return false;
    }

    inline void Random(const TYPE vMin, const TYPE vMax)
    {
        UT::Random(&v0(), 6, vMin, vMax);
    }
    static inline Vector6<TYPE> GetRandom(const TYPE vMin, const TYPE vMax)
    {
        Vector6<TYPE> v;
        v.Random(vMin, vMax);
        return v;
    }

  protected:
    TYPE m_data[6];
};

typedef Vector6<float> Vector6f;

class ProductVector6f
{
  public:
    inline const _pi__m128 &v0123() const { return m_data[0]; }
    inline _pi__m128 &v0123() { return m_data[0]; }
    inline const _pi__m128 &v4501() const { return m_data[1]; }
    inline _pi__m128 &v4501() { return m_data[1]; }
    inline const _pi__m128 &v2345() const { return m_data[2]; }
    inline _pi__m128 &v2345() { return m_data[2]; }
    inline const float &v0() const { return m_data[0].m128_f32[0]; }
    inline const float &v1() const { return m_data[0].m128_f32[1]; }
    inline const float &v2() const { return m_data[0].m128_f32[2]; }
    inline const float &v3() const { return m_data[0].m128_f32[3]; }
    inline const float &v4() const { return m_data[1].m128_f32[0]; }
    inline const float &v5() const { return m_data[1].m128_f32[1]; }
    inline void Set(const float *v)
    {
        v0123() = _pi_mm_loadu_ps(v);
        v2345() = _pi_mm_loadu_ps(v + 2);
        // v4501() = _pi_mm_shuffle_ps(v2345(), v0123(), _MM_SHUFFLE(1, 0, 3,
        // 2));
        v4501() = _pi_mm_shuffle_ps(v2345(), v0123(), 78);
    }
    inline void Get(float *v) const
    {
        //_pi_mm_storeu_ps(v, _pi_mm_add_ps(v0123(), _pi_mm_shuffle_ps(v4501(),
        // v2345(), _MM_SHUFFLE(1, 0, 3, 2))));
        _pi_mm_storeu_ps(
            v, _pi_mm_add_ps(v0123(), _pi_mm_shuffle_ps(v4501(), v2345(), 78)));
        v[4] = m_data[1].m128_f32[0] + m_data[2].m128_f32[2];
        v[5] = m_data[1].m128_f32[2] + m_data[2].m128_f32[3];
    }

    inline void MakeZero() { memset(this, 0, sizeof(ProductVector6f)); }
  protected:
    _pi__m128 m_data[3];
};
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector6f : public Eigen::Matrix<float, 6, 1>
{
  public:
    inline EigenVector6f() = default;
    inline EigenVector6f(const Eigen::Matrix<float, 6, 1> &e_v)
        : Eigen::Matrix<float, 6, 1>(e_v)
    {
    }
    inline EigenVector6f(const LA::Vector6f &v) : Eigen::Matrix<float, 6, 1>()
    {
        const float *_v = v;
        Eigen::Matrix<float, 6, 1> &e_v = *this;
        for (int i = 0; i < 6; ++i) e_v(i, 0) = _v[i];
    }
    inline EigenVector6f(const LA::AlignedVector6f &v)
        : Eigen::Matrix<float, 6, 1>()
    {
        const float *_v = v;
        Eigen::Matrix<float, 6, 1> &e_v = *this;
        for (int i = 0; i < 6; ++i) e_v(i, 0) = _v[i];
    }
    inline EigenVector6f(const EigenVector3f &e_v0, const EigenVector3f &e_v1)
    {
        block<3, 1>(0, 0) = e_v0;
        block<3, 1>(3, 0) = e_v1;
    }
    inline void operator=(const Eigen::Matrix<float, 6, 1> &e_v)
    {
        *((Eigen::Matrix<float, 6, 1> *)this) = e_v;
    }
    inline LA::AlignedVector6f GetAlignedVector6f() const
    {
        LA::AlignedVector6f v;
        float *_v = v;
        const Eigen::Matrix<float, 6, 1> &e_v = *this;
        for (int i = 0; i < 6; ++i) _v[i] = e_v(i, 0);
        return v;
    }
    inline LA::Vector6f GetVector6f() const
    {
        LA::Vector6f v;
        float *_v = v;
        const Eigen::Matrix<float, 6, 1> &e_v = *this;
        for (int i = 0; i < 6; ++i) _v[i] = e_v(i, 0);
        return v;
    }
    inline float SquaredLength() const
    {
        return GetAlignedVector6f().SquaredLength();
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedVector6f().Print(e);
    }
    static inline EigenVector6f GetRandom(const float mMax)
    {
        return EigenVector6f(LA::AlignedVector6f::GetRandom(mMax));
    }
    inline bool AssertEqual(const LA::AlignedVector6f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetAlignedVector6f().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const LA::Vector6f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetVector6f().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const EigenVector6f &e_v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_v.GetAlignedVector6f(), verbose, eps);
    }
};
#endif
#endif
