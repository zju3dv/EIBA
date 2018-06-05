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

#ifndef _VECTOR_2_H_
#define _VECTOR_2_H_

#include "Utility.h"

namespace LA
{
template <typename TYPE> class Vector2
{
  public:
    inline Vector2() = default;
    inline Vector2(const TYPE v0, const TYPE v1) { Set(v0, v1); }
    inline const TYPE &v0() const { return m_data[0]; }
    inline TYPE &v0() { return m_data[0]; }
    inline const TYPE &v1() const { return m_data[1]; }
    inline TYPE &v1() { return m_data[1]; }
    inline const TYPE &x() const { return m_data[0]; }
    inline TYPE &x() { return m_data[0]; }
    inline const TYPE &y() const { return m_data[1]; }
    inline TYPE &y() { return m_data[1]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline bool operator==(const Vector2<TYPE> &v) const
    {
        return v0() == v.v0() && v1() == v.v1();
    }
    inline void operator+=(const Vector2<TYPE> &v)
    {
        v0() = v.v0() + v0();
        v1() = v.v1() + v1();
    }
    inline void operator-=(const Vector2<TYPE> &v)
    {
        v0() = -v.v0() + v0();
        v1() = -v.v1() + v1();
    }
    inline void operator*=(const TYPE s)
    {
        v0() *= s;
        v1() *= s;
    }
    inline void operator*=(const Vector2<TYPE> &s)
    {
        v0() *= s.v0();
        v1() *= s.v1();
    }
    inline Vector2<TYPE> operator+(const Vector2<TYPE> &v) const
    {
        return Vector2<TYPE>(v0() + v.v0(), v1() + v.v1());
    }
    inline Vector2<TYPE> operator-(const Vector2<TYPE> &v) const
    {
        return Vector2<TYPE>(v0() - v.v0(), v1() - v.v1());
    }
    inline Vector2<TYPE> operator*(const TYPE s) const
    {
        return Vector2<TYPE>(v0() * s, v1() * s);
    }

    inline void Set(const TYPE v0, const TYPE v1)
    {
        this->v0() = v0;
        this->v1() = v1;
    }

    inline void MakeZero() { memset(this, 0, sizeof(Vector2<TYPE>)); }
    inline bool Valid() const { return v0() != UT::Invalid<TYPE>(); }
    inline bool Invalid() const { return v0() == UT::Invalid<TYPE>(); }
    inline void Invalidate() { v0() = UT::Invalid<TYPE>(); }
    inline TYPE SquaredLength() const { return v0() * v0() + v1() * v1(); }
    inline TYPE Dot(const Vector2<TYPE> &v) const
    {
        return v0() * v.v0() + v1() * v.v1();
    }
    inline void Print(const bool e = false) const
    {
        UT::PrintValue<TYPE>(v0(), e);
        UT::Print(" ");
        UT::PrintValue<TYPE>(v1(), e);
        UT::Print("\n");
    }
    inline void Print(const std::string str, const bool e) const
    {
        UT::Print("%s", str.c_str());
        Print(e);
    }
    inline bool AssertEqual(const Vector2<TYPE> &v, const int verbose = 1,
                            const TYPE eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&v0(), &v.v0(), 2, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
        }
        return false;
    }

  protected:
    TYPE m_data[2];
};

typedef Vector2<short> Vector2s;
typedef Vector2<ushort> Vector2us;
typedef Vector2<int> Vector2i;
typedef Vector2<float> Vector2f;
typedef Vector2<double> Vector2d;
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
class EigenVector2f : public Eigen::Vector2f
{
  public:
    inline EigenVector2f() = default;
    inline EigenVector2f(const Eigen::Vector2f &e_v) : Eigen::Vector2f(e_v) {}
    inline EigenVector2f(const LA::Vector2f &v)
        : Eigen::Vector2f(v.v0(), v.v1())
    {
    }
    inline EigenVector2f(const float v0, const float v1)
        : Eigen::Vector2f(v0, v1)
    {
    }
    inline EigenVector2f(const float *v) : Eigen::Vector2f(v[0], v[1]) {}
    inline void operator=(const Eigen::Vector2f &e_v)
    {
        *((Eigen::Vector2f *)this) = e_v;
    }
    inline LA::Vector2f GetVector2f() const
    {
        LA::Vector2f v;
        const Eigen::Vector2f &e_v = *this;
        v.v0() = e_v(0);
        v.v1() = e_v(1);
        return v;
    }
    inline float SquaredLength() const { return GetVector2f().SquaredLength(); }
    inline void Print(const bool e = false) const { GetVector2f().Print(e); }
    inline bool AssertEqual(const LA::Vector2f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return GetVector2f().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const EigenVector2f &e_v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_v.GetVector2f(), verbose, eps);
    }
};
#endif
#endif
