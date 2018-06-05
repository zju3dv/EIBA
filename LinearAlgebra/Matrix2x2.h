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

#ifndef _MATRIX_2x2_H_
#define _MATRIX_2x2_H_

#include "Vector2.h"

namespace LA
{
class AlignedMatrix2x2f
{
  public:
    inline const _pi__m128 &m_00_01_10_11() const { return m_data; }
    inline _pi__m128 &m_00_01_10_11() { return m_data; }
    inline const float &m00() const { return m_data.m128_f32[0]; }
    inline float &m00() { return m_data.m128_f32[0]; }
    inline const float &m01() const { return m_data.m128_f32[1]; }
    inline float &m01() { return m_data.m128_f32[1]; }
    inline const float &m10() const { return m_data.m128_f32[2]; }
    inline float &m10() { return m_data.m128_f32[2]; }
    inline const float &m11() const { return m_data.m128_f32[3]; }
    inline float &m11() { return m_data.m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void operator*=(const AlignedMatrix2x2f &M)
    {
        m_00_01_10_11() = _pi_mm_mul_ps(M.m_00_01_10_11(), m_00_01_10_11());
    }
    inline Vector2f operator*(const Vector2f &b) const
    {
        Vector2f Ab;
        const _pi__m128 t = _pi_mm_mul_ps(
            m_00_01_10_11(), _pi_mm_setr_ps(b.v0(), b.v1(), b.v0(), b.v1()));
        Ab.v0() = t.m128_f32[0] + t.m128_f32[1];
        Ab.v1() = t.m128_f32[2] + t.m128_f32[3];
        return Ab;
    }

    inline void SetDiagonal(const Vector2f &d)
    {
        m00() = d.v0();
        m11() = d.v1();
    }
    inline void GetDiagonal(Vector2f &d) const
    {
        d.v0() = m00();
        d.v1() = m11();
    }
    inline void ScaleDiagonal(const float s)
    {
        m00() *= s;
        m11() *= s;
    }
    inline void IncreaseDiagonal(const float d)
    {
        m00() = d + m00();
        m11() = d + m11();
    }
    inline void IncreaseDiagonal(const float d0, const float d1)
    {
        m00() = d0 + m00();
        m11() = d1 + m11();
    }
    inline void SetLowerFromUpper() { m10() = m01(); }
    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix2x2f)); }
    inline void MakeIdentity()
    {
        m_00_01_10_11() = _pi_mm_setr_ps(1.0f, 0.0f, 0.0f, 1.0f);
    }

    inline float Determinant() const { return m00() * m11() - m01() * m10(); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e\n", m00(), m01());
            UT::Print("%e %e\n", m10(), m11());
        } else {
            UT::Print("%f %f\n", m00(), m01());
            UT::Print("%f %f\n", m10(), m11());
        }
    }
    inline bool AssertEqual(const AlignedMatrix2x2f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 4, 0)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

  protected:
    _pi__m128 m_data;
};

template <typename TYPE> class SymmetricMatrix2x2
{
  public:
    inline SymmetricMatrix2x2() = default;
    inline SymmetricMatrix2x2(const TYPE m00, const TYPE m01, const TYPE m11)
    {
        Set(m00, m01, m11);
    }

    inline const TYPE &m00() const { return m_data[0]; }
    inline TYPE &m00() { return m_data[0]; }
    inline const TYPE &m01() const { return m_data[1]; }
    inline TYPE &m01() { return m_data[1]; }
    inline const TYPE &m11() const { return m_data[2]; }
    inline TYPE &m11() { return m_data[2]; }
    inline bool operator==(const SymmetricMatrix2x2<TYPE> &M) const
    {
        return m00() == M.m00() && m01() == M.m01() && m11() == M.m11();
    }
    inline void operator+=(const SymmetricMatrix2x2<TYPE> &M)
    {
        m00() = M.m00() + m00();
        m01() = M.m01() + m01();
        m11() = M.m11() + m11();
    }
    inline void operator*=(const TYPE s) { Scale(s); }
    inline void operator*=(const SymmetricMatrix2x2<TYPE> &s)
    {
        m00() *= s.m00();
        m01() *= s.m01();
        m11() *= s.m11();
    }
    inline void operator/=(const TYPE d) { Scale(1 / d); }
    inline SymmetricMatrix2x2<TYPE> operator*(const TYPE s) const
    {
        return SymmetricMatrix2x2<TYPE>(m00() * s, m01() * s, m11() * s);
    }
    inline Vector2<TYPE> operator*(const Vector2<TYPE> &b) const
    {
        Vector2<TYPE> _Ab;
        Ab(*this, b, _Ab);
        return _Ab;
    }

    inline void Set(const TYPE m00, const TYPE m01, const TYPE m11)
    {
        this->m00() = m00;
        this->m01() = m01;
        this->m11() = m11;
    }
    inline void MakeZero()
    {
        memset(this, 0, sizeof(SymmetricMatrix2x2<TYPE>));
    }
    inline void MakeDiagonal(const TYPE d)
    {
        m00() = m11() = d;
        m01() = 0.0f;
    }
    inline void IncreaseDiagonal(const TYPE d)
    {
        m00() = d + m00();
        m11() = d + m11();
    }
    inline TYPE Determinant() const { return m00() * m11() - m01() * m01(); }
    inline void Scale(const TYPE s)
    {
        m00() *= s;
        m01() *= s;
        m11() *= s;
    }
    inline void GetScaled(const TYPE s, SymmetricMatrix2x2<TYPE> &M) const
    {
        M.m00() = m00() * s;
        M.m01() = m01() * s;
        M.m11() = m11() * s;
    }

    inline bool GetInverse(SymmetricMatrix2x2<TYPE> &MI, const TYPE eps) const
    {
        const TYPE d = Determinant();
        if (UT::NotANumber<TYPE>(d) || fabs(d) <= eps) {
            MI.Invalidate();
            return false;
        }
        MI.m11() = 1 / d;
        MI.m00() = m11() * MI.m11();
        MI.m01() = -m01() * MI.m11();
        MI.m11() = m00() * MI.m11();
        return true;
    }
    inline SymmetricMatrix2x2<TYPE> GetInverse(const TYPE eps) const
    {
        SymmetricMatrix2x2<TYPE> MI;
        GetInverse(MI, eps);
        return MI;
    }

    inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }
    inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
    inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e\n", m00(), m01());
            UT::Print("%e %e\n", m01(), m11());
        } else {
            UT::Print("%f %f\n", m00(), m01());
            UT::Print("%f %f\n", m01(), m11());
        }
    }

    inline bool AssertEqual(const SymmetricMatrix2x2<TYPE> &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 3, 0)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

    static inline TYPE MahalanobisDistance(const SymmetricMatrix2x2<TYPE> &W,
                                           const Vector2<TYPE> &d)
    {
        return (W.m00() * d.x() + W.m01() * d.y()) * d.x() +
               (W.m01() * d.x() + W.m11() * d.y()) * d.y();
    }

    static inline void aaT(const Vector2<TYPE> &a,
                           SymmetricMatrix2x2<TYPE> &aaT)
    {
        aaT.m00() = a.v0() * a.v0();
        aaT.m01() = a.v0() * a.v1();
        aaT.m11() = a.v1() * a.v1();
    }
    static inline void Ab(const SymmetricMatrix2x2<TYPE> &A,
                          const Vector2<TYPE> &b, Vector2<TYPE> &Ab)
    {
        Ab.v0() = A.m00() * b.v0() + A.m01() * b.v1();
        Ab.v1() = A.m01() * b.v0() + A.m11() * b.v1();
    }

  protected:
    TYPE m_data[3];
};

typedef SymmetricMatrix2x2<float> SymmetricMatrix2x2f;
typedef SymmetricMatrix2x2<double> SymmetricMatrix2x2d;
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix2x2f : public Eigen::Matrix2f
{
  public:
    inline EigenMatrix2x2f() = default;
    inline EigenMatrix2x2f(const Eigen::Matrix2f &e_M) : Eigen::Matrix2f(e_M) {}
    inline EigenMatrix2x2f(const LA::AlignedMatrix2x2f &M) : Eigen::Matrix2f()
    {
        Eigen::Matrix2f &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(1, 0) = M.m10();
        e_M(1, 1) = M.m11();
    }
    inline EigenMatrix2x2f(const LA::SymmetricMatrix2x2f &M) : Eigen::Matrix2f()
    {
        Eigen::Matrix2f &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(1, 0) = M.m01();
        e_M(1, 1) = M.m11();
    }
    inline void operator=(const Eigen::Matrix2f &e_M)
    {
        *((Eigen::Matrix2f *)this) = e_M;
    }
    inline LA::AlignedMatrix2x2f GetAlignedMatrix2x2f() const
    {
        LA::AlignedMatrix2x2f M;
        const Eigen::Matrix2f &e_M = *this;
        M.m00() = e_M(0, 0);
        M.m01() = e_M(0, 1);
        M.m10() = e_M(1, 0);
        M.m11() = e_M(1, 1);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix2x2f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix2x2f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix2x2f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const EigenMatrix2x2f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix2x2f(), verbose);
    }
};
#endif

#endif
