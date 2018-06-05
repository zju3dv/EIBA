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

#ifndef _VECTOR_3_H_
#define _VECTOR_3_H_

#include "Utility.h"
#include "Vector2.h"

namespace LA
{
class AlignedVector3f
{
  public:
    inline AlignedVector3f() = default;
    inline AlignedVector3f(const float *v) { Set(v); }
    inline AlignedVector3f(const float v0, const float v1, const float v2)
    {
        v012r() = _pi_mm_setr_ps(v0, v1, v2, 0.0f);
    }
    inline AlignedVector3f(const Vector2f &x, const float z)
    {
        xyzr() = _pi_mm_setr_ps(x.x() * z, x.y() * z, z, 0.0f);
    }
    inline AlignedVector3f(const _pi__m128 &v) : m_data(v) {}
    inline const _pi__m128 &v012r() const { return m_data; }
    inline _pi__m128 &v012r() { return m_data; }
    inline const _pi__m128 &xyzr() const { return m_data; }
    inline _pi__m128 &xyzr() { return m_data; }
    inline const float &v0() const { return m_data.m128_f32[0]; }
    inline float &v0() { return m_data.m128_f32[0]; }
    inline const float &v1() const { return m_data.m128_f32[1]; }
    inline float &v1() { return m_data.m128_f32[1]; }
    inline const float &v2() const { return m_data.m128_f32[2]; }
    inline float &v2() { return m_data.m128_f32[2]; }
    inline const float &r() const { return m_data.m128_f32[3]; }
    inline float &r() { return m_data.m128_f32[3]; }
    inline const float &x() const { return m_data.m128_f32[0]; }
    inline float &x() { return m_data.m128_f32[0]; }
    inline const float &y() const { return m_data.m128_f32[1]; }
    inline float &y() { return m_data.m128_f32[1]; }
    inline const float &z() const { return m_data.m128_f32[2]; }
    inline float &z() { return m_data.m128_f32[2]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline bool operator==(const AlignedVector3f &v) const
    {
        return v0() == v.v0() && v1() == v.v1() && v2() == v.v2();
    }
    inline void operator+=(const AlignedVector3f &v)
    {
        v012r() = _pi_mm_add_ps(v.v012r(), v012r());
    }
    inline void operator-=(const AlignedVector3f &v)
    {
        v012r() = _pi_mm_sub_ps(v012r(), v.v012r());
    }
    inline void operator*=(const float s) { Scale(s); }
    inline void operator/=(const float d) { Scale(1.0f / d); }
    inline AlignedVector3f operator+(const AlignedVector3f &v) const
    {
        return AlignedVector3f(_pi_mm_add_ps(v012r(), v.v012r()));
    }
    inline AlignedVector3f operator+(const float v) const
    {
        return AlignedVector3f(_pi_mm_add_ps(v012r(), _pi_mm_set1_ps(v)));
    }
    inline AlignedVector3f operator-(const AlignedVector3f &v) const
    {
        return AlignedVector3f(_pi_mm_sub_ps(v012r(), v.v012r()));
    }
    inline AlignedVector3f operator*(const float s) const
    {
        return AlignedVector3f(_pi_mm_mul_ps(v012r(), _pi_mm_set1_ps(s)));
    }
    inline AlignedVector3f operator*(const _pi__m128 &s) const
    {
        return AlignedVector3f(_pi_mm_mul_ps(v012r(), s));
    }
    inline AlignedVector3f operator/(const float d) const
    {
        return AlignedVector3f(
            _pi_mm_mul_ps(v012r(), _pi_mm_set1_ps(1.0f / d)));
    }

    inline void Set(const float *v) { memcpy(this, v, 12); }
    inline void Set(const double *v)
    {
        v012r() = _pi_mm_setr_ps(float(v[0]), float(v[1]), float(v[2]), 0.0f);
    }
    inline void Set(const float v0, const float v1, const float v2)
    {
        v012r() = _pi_mm_setr_ps(v0, v1, v2, 0.0f);
    }
    inline void Set(const Vector2f &x, const float z)
    {
        xyzr() = _pi_mm_setr_ps(x.x() * z, x.y() * z, z, 0.0f);
    }
    inline void Get(float *v) const { memcpy(v, this, 12); }
    inline void Get(double *v) const
    {
        v[0] = double(v0());
        v[1] = double(v1());
        v[2] = double(v2());
    }
    inline void GetMinus(AlignedVector3f &v) const
    {
        v.v012r() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v012r());
    }
    inline AlignedVector3f GetMinus() const
    {
        AlignedVector3f v;
        GetMinus(v);
        return v;
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedVector3f)); }
    inline void MakeMinus()
    {
        v012r() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), v012r());
    }

    inline bool Valid() const { return v0() != FLT_MAX; }
    inline bool Invalid() const { return v0() == FLT_MAX; }
    inline void Invalidate() { v0() = FLT_MAX; }
    inline void Project(Vector2f &x) const
    {
        x.y() = 1 / z();
        x.x() = this->x() * x.y();
        x.y() = this->y() * x.y();
    }
    inline const Vector2f GetProjected() const
    {
        Vector2f x;
        Project(x);
        return x;
    }

    inline void Normalize()
    {
        v012r() = _pi_mm_mul_ps(_pi_mm_set1_ps(1.0f / sqrt(SquaredLength())),
                                v012r());
    }
    inline AlignedVector3f GetNormalized() const
    {
        AlignedVector3f v;
        v.v012r() = _pi_mm_mul_ps(_pi_mm_set1_ps(1.0f / sqrt(SquaredLength())),
                                  v012r());
        return v;
    }

    inline void Scale(const float s)
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        Scale(_s);
    }
    inline void Scale(const _pi__m128 &s)
    {
        v012r() = _pi_mm_mul_ps(s, v012r());
    }
    inline void GetScaled(const float s, AlignedVector3f &v) const
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        GetScaled(_s, v);
    }
    inline void GetScaled(const _pi__m128 &s, AlignedVector3f &v) const
    {
        v.v012r() = _pi_mm_mul_ps(s, v012r());
    }
    inline void GetScaled(const LA::AlignedVector3f &s,
                          AlignedVector3f &v) const
    {
        GetScaled(s.v012r(), v);
    }
    inline AlignedVector3f GetScaled(const float s) const
    {
        AlignedVector3f v;
        GetScaled(s, v);
        return v;
    }
    inline void GetSquareRoot(AlignedVector3f &v) const
    {
        v.v012r() = _pi_mm_sqrt_ps(v012r());
    }
    inline AlignedVector3f GetSquareRoot() const
    {
        AlignedVector3f v;
        GetSquareRoot(v);
        return v;
    }
    inline void MakeSquareRoot() { v012r() = _pi_mm_sqrt_ps(v012r()); }
    inline float Sum() const { return SSE::Sum012(v012r()); }
    inline float SquaredLength() const
    {
        return SSE::Sum012(_pi_mm_mul_ps(v012r(), v012r()));
    }
    inline float Dot(const AlignedVector3f &v) const
    {
        return SSE::Sum012(_pi_mm_mul_ps(v012r(), v.v012r()));
    }
    inline float Dot(const float *v) const
    {
        return SSE::Sum012(_pi_mm_mul_ps(v012r(), *((_pi__m128 *)v)));
    }
    inline AlignedVector3f Cross(const AlignedVector3f &v) const
    {
        AlignedVector3f c;
        c.v0() = v1() * v.v2() - v2() * v.v1();
        c.v1() = v2() * v.v0() - v0() * v.v2();
        c.v2() = v0() * v.v1() - v1() * v.v0();
        return c;
    }

    inline void Interpolate(const float w1, const AlignedVector3f &v1,
                            const AlignedVector3f &v2)
    {
        v012r() =
            _pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(w1), v1.v012r()),
                          _pi_mm_mul_ps(_pi_mm_set1_ps(1.0f - w1), v2.v012r()));
    }

    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("%e %e %e\n", v0(), v1(), v2());
        else
            UT::Print("%f %f %f\n", v0(), v1(), v2());
    }
    inline void Print(const std::string str, const bool e) const
    {
        UT::Print("%s", str.c_str());
        if (e)
            UT::Print("%e %e %e\n", v0(), v1(), v2());
        else
            UT::Print("%f %f %f\n", v0(), v1(), v2());
    }
    inline void Save(FILE *fp, const bool e = false) const
    {
        if (e)
            fprintf(fp, "%e %e %e\n", v0(), v1(), v2());
        else
            fprintf(fp, "%f %f %f\n", v0(), v1(), v2());
    }
    // inline void Load(FILE *fp, const bool e = false)
    //{
    //	if(e)
    //		fscanf(fp, "%e %e %e", &v0(), &v1(), &v2());
    //	else
    //		fscanf(fp, "%f %f %f", &v0(), &v1(), &v2());
    //}
    inline void Load(FILE *fp)
    {
#ifdef CFG_DEBUG
        const int N = fscanf(fp, "%f %f %f", &v0(), &v1(), &v2());
        UT_ASSERT(N == 3);
#else
        fscanf(fp, "%f %f %f", &v0(), &v1(), &v2());
#endif
    }

    inline bool AssertZero(const float eps = 0.0f) const
    {
        return UT::AssertEqual(SquaredLength(), 0.0f, 0, eps);
    }
    inline bool AssertEqual(const AlignedVector3f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&v0(), &v.v0(), 3, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
        }
        return false;
    }

    inline void SetInfinite() { v012r() = _pi_mm_set1_ps(FLT_MAX); }
    inline void AssertInfinite() const
    {
        UT_ASSERT(v0() == FLT_MAX);
        UT_ASSERT(v1() == FLT_MAX);
        UT_ASSERT(v2() == FLT_MAX);
        UT_ASSERT(r() == FLT_MAX);
    }
    inline void AssertFinite() const
    {
        UT_ASSERT(v0() != FLT_MAX);
        UT_ASSERT(v1() != FLT_MAX);
        UT_ASSERT(v2() != FLT_MAX);
        UT_ASSERT(r() != FLT_MAX);
    }

    inline void Random(const float vMax) { UT::Random(&v0(), 3, -vMax, vMax); }
    inline void Random(const float vMin, const float vMax)
    {
        UT::Random(&v0(), 3, vMin, vMax);
    }
    static inline AlignedVector3f GetRandom(const float vMax)
    {
        AlignedVector3f v;
        v.Random(vMax);
        return v;
    }
    static inline AlignedVector3f GetRandom(const float vMin, const float vMax)
    {
        AlignedVector3f v;
        v.Random(vMin, vMax);
        return v;
    }

  protected:
    _pi__m128 m_data;
};

template <typename TYPE> class Vector3
{
  public:
    inline Vector3<TYPE>() = default;
    inline Vector3<TYPE>(const TYPE *v) { Set(v); }
    inline Vector3<TYPE>(const TYPE v0, const TYPE v1, const TYPE v2)
    {
        this->v0() = v0;
        this->v1() = v1;
        this->v2() = v2;
    }
    inline Vector3<TYPE>(const Vector2<TYPE> &v0, const TYPE v1)
    {
        this->v0() = v0.v0();
        this->v1() = v0.v1();
        this->v2() = v1;
    }

    inline const TYPE &v0() const { return m_data[0]; }
    inline TYPE &v0() { return m_data[0]; }
    inline const TYPE &v1() const { return m_data[1]; }
    inline TYPE &v1() { return m_data[1]; }
    inline const TYPE &v2() const { return m_data[2]; }
    inline TYPE &v2() { return m_data[2]; }
    inline const TYPE &x() const { return m_data[0]; }
    inline TYPE &x() { return m_data[0]; }
    inline const TYPE &y() const { return m_data[1]; }
    inline TYPE &y() { return m_data[1]; }
    inline const TYPE &z() const { return m_data[2]; }
    inline TYPE &z() { return m_data[2]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void operator=(const TYPE *v) { Set(v); }
    inline void operator+=(const Vector3<TYPE> &v)
    {
        v0() = v.v0() + v0();
        v1() = v.v1() + v1();
        v2() = v.v2() + v2();
    }
    inline void operator+=(const AlignedVector3f &v)
    {
        v0() = v.v0() + v0();
        v1() = v.v1() + v1();
        v2() = v.v2() + v2();
    }
    inline void operator*=(const TYPE s) { Scale(s); }
    inline Vector3<TYPE> operator+(const Vector3<TYPE> &v) const
    {
        return Vector3<TYPE>(v0() + v.v0(), v1() + v.v1(), v2() + v.v2());
    }
    inline Vector3<TYPE> operator*(const TYPE s) const
    {
        return Vector3<TYPE>(v0() * s, v1() * s, v2() * s);
    }

    inline void Set(const TYPE v0, const TYPE v1, const TYPE v2)
    {
        this->v0() = v0;
        this->v1() = v1;
        this->v2() = v2;
    }
    inline void Set(const TYPE *v) { memcpy(this, v, sizeof(Vector3<TYPE>)); }
    inline void Get(TYPE *v) const { memcpy(v, this, sizeof(Vector3<TYPE>)); }
    inline void MakeZero() { memset(this, 0, sizeof(Vector3<TYPE>)); }
    inline void Scale(const TYPE s)
    {
        v0() *= s;
        v1() *= s;
        v2() *= s;
    }
    inline void Normalize() { Scale(1 / sqrt(SquaredLength())); }
    inline TYPE SquaredLength() const
    {
        return v0() * v0() + v1() * v1() + v2() * v2();
    }

    inline void Invalidate() { v0() = UT::Invalid<TYPE>(); }
    inline bool Valid() const { return v0() != UT::Invalid<TYPE>(); }
    inline bool Invalid() const { return v0() == UT::Invalid<TYPE>(); }
    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("%e %e %e\n", v0(), v1(), v2());
        else
            UT::Print("%f %f %f\n", v0(), v1(), v2());
    }

    inline bool AssertEqual(const Vector3<TYPE> &v, const int verbose = 1,
                            const TYPE eps = 0) const
    {
        if (UT::VectorAssertEqual<TYPE>(&v0(), &v.v0(), 3, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
        }
        return false;
    }

    inline void Random(const TYPE vMin, const TYPE vMax)
    {
        UT::Random(&v0(), 3, vMin, vMax);
    }
    static inline Vector3<TYPE> GetRandom(const TYPE vMin, const TYPE vMax)
    {
        Vector3<TYPE> v;
        v.Random(vMin, vMax);
        return v;
    }

  protected:
    TYPE m_data[3];
};

typedef Vector3<int> Vector3i;
typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;
typedef Vector3<ubyte> Vector3ub;
}

#ifdef CFG_DEBUG_EIGEN
class EigenVector3f : public Eigen::Vector3f
{
  public:
    inline EigenVector3f() = default;
    inline EigenVector3f(const Eigen::Vector3f &e_v) : Eigen::Vector3f(e_v) {}
    inline EigenVector3f(const LA::AlignedVector3f &v)
        : Eigen::Vector3f(v.v0(), v.v1(), v.v2())
    {
    }
    inline EigenVector3f(const LA::Vector3f &v)
        : Eigen::Vector3f(v.v0(), v.v1(), v.v2())
    {
    }
    inline EigenVector3f(const float v0, const float v1, const float v2)
        : Eigen::Vector3f(v0, v1, v2)
    {
    }
    inline EigenVector3f(const EigenVector2f &e_v0, const float v1)
        : Eigen::Vector3f(e_v0(0), e_v0(1), v1)
    {
    }
    inline void operator=(const Eigen::Vector3f &e_v)
    {
        *((Eigen::Vector3f *)this) = e_v;
    }
    inline LA::AlignedVector3f GetAlignedVector3f() const
    {
        LA::AlignedVector3f v;
        const Eigen::Vector3f &e_v = *this;
        v.v0() = e_v(0);
        v.v1() = e_v(1);
        v.v2() = e_v(2);
        return v;
    }
    inline LA::Vector3f GetVector3f() const
    {
        LA::Vector3f v;
        const Eigen::Vector3f &e_v = *this;
        v.v0() = e_v(0);
        v.v1() = e_v(1);
        v.v2() = e_v(2);
        return v;
    }
    inline EigenVector2f Project() const
    {
        EigenVector2f e_x;
        e_x.y() = 1.0f / z();
        e_x.x() = x() * e_x.y();
        e_x.y() = y() * e_x.y();
        return e_x;
    }
    inline float SquaredLength() const
    {
        return GetAlignedVector3f().SquaredLength();
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedVector3f().Print(e);
    }
    static inline EigenVector3f GetRandom(const float vMax)
    {
        return EigenVector3f(LA::AlignedVector3f::GetRandom(vMax));
    }
    inline bool AssertEqual(const LA::AlignedVector3f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetAlignedVector3f().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const LA::Vector3f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetVector3f().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const EigenVector3f &e_v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_v.GetAlignedVector3f(), verbose, eps);
    }
};
#endif
#endif
