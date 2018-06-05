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

#ifndef _MATRIX_3x3_H_
#define _MATRIX_3x3_H_

#include "Matrix2x2.h"
#include "Vector3.h"

namespace LA
{
class AlignedMatrix3x3f
{
  public:
    inline AlignedMatrix3x3f() = default;
    inline AlignedMatrix3x3f(const float d) { MakeDiagonal(d); }
    inline const _pi__m128 &m_00_01_02_r0() const { return m_data[0]; }
    inline _pi__m128 &m_00_01_02_r0() { return m_data[0]; }
    inline const _pi__m128 &m_10_11_12_r1() const { return m_data[1]; }
    inline _pi__m128 &m_10_11_12_r1() { return m_data[1]; }
    inline const _pi__m128 &m_20_21_22_r2() const { return m_data[2]; }
    inline _pi__m128 &m_20_21_22_r2() { return m_data[2]; }
    inline const float &m00() const { return m_data[0].m128_f32[0]; }
    inline float &m00() { return m_data[0].m128_f32[0]; }
    inline const float &m01() const { return m_data[0].m128_f32[1]; }
    inline float &m01() { return m_data[0].m128_f32[1]; }
    inline const float &m02() const { return m_data[0].m128_f32[2]; }
    inline float &m02() { return m_data[0].m128_f32[2]; }
    inline const float &r0() const { return m_data[0].m128_f32[3]; }
    inline float &r0() { return m_data[0].m128_f32[3]; }
    inline const float &m10() const { return m_data[1].m128_f32[0]; }
    inline float &m10() { return m_data[1].m128_f32[0]; }
    inline const float &m11() const { return m_data[1].m128_f32[1]; }
    inline float &m11() { return m_data[1].m128_f32[1]; }
    inline const float &m12() const { return m_data[1].m128_f32[2]; }
    inline float &m12() { return m_data[1].m128_f32[2]; }
    inline const float &r1() const { return m_data[1].m128_f32[3]; }
    inline float &r1() { return m_data[1].m128_f32[3]; }
    inline const float &m20() const { return m_data[2].m128_f32[0]; }
    inline float &m20() { return m_data[2].m128_f32[0]; }
    inline const float &m21() const { return m_data[2].m128_f32[1]; }
    inline float &m21() { return m_data[2].m128_f32[1]; }
    inline const float &m22() const { return m_data[2].m128_f32[2]; }
    inline float &m22() { return m_data[2].m128_f32[2]; }
    inline const float &r2() const { return m_data[2].m128_f32[3]; }
    inline float &r2() { return m_data[2].m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline bool operator==(const AlignedMatrix3x3f &M) const
    {
        return m00() == M.m00() && m01() == M.m01() && m02() == M.m02() &&
               m10() == M.m10() && m11() == M.m11() && m12() == M.m12() &&
               m20() == M.m20() && m21() == M.m21() && m22() == M.m22();
    }
    inline void operator+=(const AlignedMatrix3x3f &M)
    {
        m_00_01_02_r0() = _pi_mm_add_ps(M.m_00_01_02_r0(), m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_add_ps(M.m_10_11_12_r1(), m_10_11_12_r1());
        m_20_21_22_r2() = _pi_mm_add_ps(M.m_20_21_22_r2(), m_20_21_22_r2());
    }
    inline void operator+=(const float *M)
    {
        AlignedMatrix3x3f _M;
        _M.Set(M);
        *this += _M;
    }
    inline void operator-=(const AlignedMatrix3x3f &M)
    {
        m_00_01_02_r0() = _pi_mm_sub_ps(m_00_01_02_r0(), M.m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_sub_ps(m_10_11_12_r1(), M.m_10_11_12_r1());
        m_20_21_22_r2() = _pi_mm_sub_ps(m_20_21_22_r2(), M.m_20_21_22_r2());
    }
    inline void operator*=(const float s) { Scale(s); }
    inline void operator*=(const _pi__m128 &s) { Scale(s); }
    inline void operator/=(const float d) { Scale(1.0f / d); }
    inline AlignedMatrix3x3f operator+(const AlignedMatrix3x3f &B) const
    {
        AlignedMatrix3x3f ApB;
        ApB.m_00_01_02_r0() = _pi_mm_add_ps(m_00_01_02_r0(), B.m_00_01_02_r0());
        ApB.m_10_11_12_r1() = _pi_mm_add_ps(m_10_11_12_r1(), B.m_10_11_12_r1());
        ApB.m_20_21_22_r2() = _pi_mm_add_ps(m_20_21_22_r2(), B.m_20_21_22_r2());
        return ApB;
    }
    inline AlignedMatrix3x3f operator-(const AlignedMatrix3x3f &B) const
    {
        AlignedMatrix3x3f AmB;
        AmB.m_00_01_02_r0() = _pi_mm_sub_ps(m_00_01_02_r0(), B.m_00_01_02_r0());
        AmB.m_10_11_12_r1() = _pi_mm_sub_ps(m_10_11_12_r1(), B.m_10_11_12_r1());
        AmB.m_20_21_22_r2() = _pi_mm_sub_ps(m_20_21_22_r2(), B.m_20_21_22_r2());
        return AmB;
    }
    inline AlignedMatrix3x3f operator*(const float s) const
    {
        AlignedMatrix3x3f sA;
        GetScaled(s, sA);
        return sA;
    }
    inline AlignedMatrix3x3f operator*(const _pi__m128 &s) const
    {
        AlignedMatrix3x3f sA;
        GetScaled(s, sA);
        return sA;
    }
    inline AlignedMatrix3x3f operator*(const AlignedMatrix3x3f &B) const
    {
        const AlignedMatrix3x3f BT = B.GetTranspose();
        AlignedMatrix3x3f AB;
        ABT(*this, BT, AB);
        return AB;
    }
    inline AlignedVector3f operator*(const AlignedVector3f &b) const
    {
        AlignedVector3f Ab;
        AlignedMatrix3x3f::Ab(*this, b, Ab);
        return Ab;
    }
    inline AlignedMatrix3x3f operator/(const float s) const
    {
        AlignedMatrix3x3f sIA;
        GetScaled(1.0f / s, sIA);
        return sIA;
    }

    inline void Set(const float *M)
    {
        memcpy(&m00(), M, 12);
        memcpy(&m10(), M + 3, 12);
        memcpy(&m20(), M + 6, 12);
    }
    inline void Set(const float *M0, const float *M1, const float *M2)
    {
        memcpy(&m00(), M0, 12);
        memcpy(&m10(), M1, 12);
        memcpy(&m20(), M2, 12);
    }
    inline void Set(const double *M)
    {
        m00() = float(M[0]);
        m01() = float(M[1]);
        m02() = float(M[2]);
        m10() = float(M[3]);
        m11() = float(M[4]);
        m12() = float(M[5]);
        m20() = float(M[6]);
        m21() = float(M[7]);
        m22() = float(M[8]);
    }
    inline void Get(float *M) const
    {
        memcpy(M, &m00(), 12);
        memcpy(M + 3, &m10(), 12);
        memcpy(M + 6, &m20(), 12);
    }
    inline void Get_chk(float *M0, float *M1, float *M2) const
    {
        memcpy(M0, &m00(), 12);
        memcpy(M1, &m10(), 12);
        memcpy(M2, &m20(), 12);
    }
    inline void Get(float *m0, float *m1, float *m2) const
    {
        memcpy(m0, &m00(), 12);
        memcpy(m1 + 3, &m10(), 12);
        memcpy(m2 + 6, &m20(), 12);
    }
    inline void GetMinus(AlignedMatrix3x3f &M) const
    {
        M.m_00_01_02_r0() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), m_00_01_02_r0());
        M.m_10_11_12_r1() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), m_10_11_12_r1());
        M.m_20_21_22_r2() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), m_20_21_22_r2());
    }
    inline AlignedMatrix3x3f GetMinus() const
    {
        AlignedMatrix3x3f M;
        GetMinus(M);
        return M;
    }
    inline void SetColumn0(const AlignedVector3f &v)
    {
        m00() = v.v0();
        m10() = v.v1();
        m20() = v.v2();
    }
    inline void SetColumn1(const AlignedVector3f &v)
    {
        m01() = v.v0();
        m11() = v.v1();
        m21() = v.v2();
    }
    inline void SetColumn2(const AlignedVector3f &v)
    {
        m02() = v.v0();
        m12() = v.v1();
        m22() = v.v2();
    }
    inline void IncreaseColumn2(const AlignedVector3f &v)
    {
        m02() = v.v0() + m02();
        m12() = v.v1() + m12();
        m22() = v.v2() + m22();
    }
    inline void GetColumn0(AlignedVector3f &v) const
    {
        v.v012r() = _pi_mm_setr_ps(m00(), m10(), m20(), 0.0f);
    }
    inline void GetColumn1(AlignedVector3f &v) const
    {
        v.v012r() = _pi_mm_setr_ps(m01(), m11(), m21(), 0.0f);
    }
    inline void GetColumn2(AlignedVector3f &v) const
    {
        v.v012r() = _pi_mm_setr_ps(m02(), m12(), m22(), 0.0f);
    }
    inline AlignedVector3f GetColumn0() const
    {
        return AlignedVector3f(m00(), m10(), m20());
    }
    inline AlignedVector3f GetColumn1() const
    {
        return AlignedVector3f(m01(), m11(), m21());
    }
    inline AlignedVector3f GetColumn2() const
    {
        return AlignedVector3f(m02(), m12(), m22());
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix3x3f)); }
    inline void MakeZero2x2()
    {
        m00() = 0.0f;
        m01() = 0.0f;
        m10() = 0.0f;
        m11() = 0.0f;
    }
    inline void MakeZero2x3()
    {
        m_00_01_02_r0() = m_10_11_12_r1() = _pi_mm_setzero_ps();
    }
    inline void MakeZero3x2()
    {
        MakeZero2x2();
        m20() = 0.0f;
        m21() = 0.0f;
    }
    inline void MakeMinus()
    {
        m_00_01_02_r0() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), m_10_11_12_r1());
        m_20_21_22_r2() = _pi_mm_sub_ps(_pi_mm_setzero_ps(), m_20_21_22_r2());
    }
    inline void MakeIdentity()
    {
        MakeZero();
        m00() = m11() = m22() = 1.0f;
    }

    inline bool Valid() const { return m00() != FLT_MAX; }
    inline bool Invalid() const { return m00() == FLT_MAX; }
    inline void Invalidate() { m00() = FLT_MAX; }
    inline void MakeDiagonal(const float d)
    {
        MakeZero();
        SetDiagonal(d);
    }
    inline void SetDiagonal(const float d)
    {
        m00() = d;
        m11() = d;
        m22() = d;
    }
    inline void SetDiagonal(const AlignedVector3f &d)
    {
        m00() = d.v0();
        m11() = d.v1();
        m22() = d.v2();
    }
    inline void GetDiagonal(AlignedVector3f &d) const
    {
        d.v0() = m00();
        d.v1() = m11();
        d.v2() = m22();
    }
    inline AlignedVector3f GetDiagonal() const
    {
        AlignedVector3f d;
        GetDiagonal(d);
        return d;
    }
    inline void ScaleDiagonal(const float s)
    {
        m00() *= s;
        m11() *= s;
        m22() *= s;
    }
    inline void IncreaseDiagonal(const float d)
    {
        m00() = d + m00();
        m11() = d + m11();
        m22() = d + m22();
    }
    inline void IncreaseDiagonal(const float d0, const float d1, const float d2)
    {
        m00() = d0 + m00();
        m11() = d1 + m11();
        m22() = d2 + m22();
    }
    inline void SetLowerFromUpper()
    {
        m10() = m01();
        m20() = m02();
        m21() = m12();
    }
    inline void GetTranspose(AlignedMatrix3x3f &MT) const
    {
        MT.m00() = m00();
        MT.m01() = m10();
        MT.m02() = m20();
        MT.m10() = m01();
        MT.m11() = m11();
        MT.m12() = m21();
        MT.m20() = m02();
        MT.m21() = m12();
        MT.m22() = m22();
    }
    inline AlignedMatrix3x3f GetTranspose() const
    {
        AlignedMatrix3x3f MT;
        GetTranspose(MT);
        return MT;
    }
    inline void Transpose()
    {
        UT_SWAP(m01(), m10());
        UT_SWAP(m02(), m20());
        UT_SWAP(m12(), m21());
    }
    inline bool GetInverse(AlignedMatrix3x3f &MI,
                           const float eps = FLT_EPSILON) const
    {
        MI.m00() = m11() * m22() - m12() * m21();
        MI.m01() = m02() * m21() - m01() * m22();
        MI.m02() = m01() * m12() - m02() * m11();
        MI.m10() = m12() * m20() - m10() * m22();
        MI.m11() = m00() * m22() - m02() * m20();
        MI.m12() = m02() * m10() - m00() * m12();
        MI.m20() = m10() * m21() - m11() * m20();
        MI.m21() = m01() * m20() - m00() * m21();
        MI.m22() = m00() * m11() - m01() * m10();
        const float d = m00() * MI.m00() + m01() * MI.m10() + m02() * MI.m20();
        if (UT::NotANumber(d) || fabs(d) <= eps) {
            MI.Invalidate();
            return false;
        } else {
            MI /= d;
            return true;
        }
    }
    inline AlignedMatrix3x3f GetInverse(const float eps = FLT_EPSILON) const
    {
        AlignedMatrix3x3f MI;
        GetInverse(MI, eps);
        return MI;
    }

    inline void Scale(const float s)
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        Scale(_s);
    }
    inline void Scale(const _pi__m128 &s)
    {
        m_00_01_02_r0() = _pi_mm_mul_ps(s, m_00_01_02_r0());
        m_10_11_12_r1() = _pi_mm_mul_ps(s, m_10_11_12_r1());
        m_20_21_22_r2() = _pi_mm_mul_ps(s, m_20_21_22_r2());
    }
    inline void GetScaled(const float s, AlignedMatrix3x3f &M) const
    {
        const _pi__m128 _s = _pi_mm_set1_ps(s);
        GetScaled(_s, M);
    }
    inline void GetScaled(const _pi__m128 &s, AlignedMatrix3x3f &M) const
    {
        M.m_00_01_02_r0() = _pi_mm_mul_ps(s, m_00_01_02_r0());
        M.m_10_11_12_r1() = _pi_mm_mul_ps(s, m_10_11_12_r1());
        M.m_20_21_22_r2() = _pi_mm_mul_ps(s, m_20_21_22_r2());
    }
    inline float Determinant() const
    {
        return m00() * (m11() * m22() - m12() * m21()) +
               m01() * (m12() * m20() - m10() * m22()) +
               m02() * (m10() * m21() - m11() * m20());
    }
    inline float Trace() const { return m00() + m11() + m22(); }
    inline float SquaredFrobeniusNorm() const
    {
        return SSE::Sum012(_pi_mm_add_ps(
            _pi_mm_mul_ps(m_00_01_02_r0(), m_00_01_02_r0()),
            _pi_mm_add_ps(_pi_mm_mul_ps(m_10_11_12_r1(), m_10_11_12_r1()),
                          _pi_mm_mul_ps(m_20_21_22_r2(), m_20_21_22_r2()))));
    }
    inline void EnforceUnitFrobeniusNorm()
    {
        Scale(sqrt(1.0f / SquaredFrobeniusNorm()));
    }
    inline void EnforceUnitLastEntry() { Scale(1.0f / m22()); }
    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e\n", m00(), m01(), m02());
            UT::Print("%e %e %e\n", m10(), m11(), m12());
            UT::Print("%e %e %e\n", m20(), m21(), m22());
        } else {
            UT::Print("%f %f %f\n", m00(), m01(), m02());
            UT::Print("%f %f %f\n", m10(), m11(), m12());
            UT::Print("%f %f %f\n", m20(), m21(), m22());
        }
    }
    inline void Print(const std::string str, const bool e) const
    {
        const std::string _str(str.size(), ' ');
        if (e) {
            UT::Print("%s%e %e %e\n", str.c_str(), m00(), m01(), m02());
            UT::Print("%s%e %e %e\n", _str.c_str(), m10(), m11(), m12());
            UT::Print("%s%e %e %e\n", _str.c_str(), m20(), m21(), m22());
        } else {
            UT::Print("%s%f %f %f\n", str.c_str(), m00(), m01(), m02());
            UT::Print("%s%f %f %f\n", _str.c_str(), m10(), m11(), m12());
            UT::Print("%s%f %f %f\n", _str.c_str(), m20(), m21(), m22());
        }
    }
    inline void Save(FILE *fp) const
    {
        fprintf(fp, "%f %f %f\n", m00(), m01(), m02());
        fprintf(fp, "%f %f %f\n", m10(), m11(), m12());
        fprintf(fp, "%f %f %f\n", m20(), m21(), m22());
    }
    inline void Load(FILE *fp)
    {
        fscanf(fp, "%f %f %f", &m00(), &m01(), &m02());
        fscanf(fp, "%f %f %f", &m10(), &m11(), &m12());
        fscanf(fp, "%f %f %f", &m20(), &m21(), &m22());
    }

    inline void AssertZero() const
    {
        UT::AssertEqual(SquaredFrobeniusNorm(), 0.0f);
    }
    inline bool AssertEqual(const AlignedMatrix3x3f &M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 3, 0, eps) &&
            UT::VectorAssertEqual(&m10(), &M.m10(), 3, 0, eps) &&
            UT::VectorAssertEqual(&m20(), &M.m20(), 3, 0, eps))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }
    inline void AssertIdentity() const
    {
        UT_ASSERT(m00() == 1.0f && m01() == 0.0f && m02() == 0.0f);
        UT_ASSERT(m10() == 0.0f && m11() == 1.0f && m12() == 0.0f);
        UT_ASSERT(m20() == 0.0f && m21() == 0.0f && m22() == 1.0f);
    }
    inline void AssertSymmetric() const
    {
        UT_ASSERT(m01() == m10() && m02() == m20() && m12() == m21());
    }
    inline void AssertSymmetric(const LA::AlignedMatrix3x3f &M) const
    {
        UT_ASSERT(m00() == M.m00() && m01() == M.m10() && m02() == M.m20());
        UT_ASSERT(m10() == M.m01() && m11() == M.m11() && m12() == M.m21());
        UT_ASSERT(m20() == M.m02() && m21() == M.m12() && m22() == M.m22());
    }
    inline void SetInfinite()
    {
        m_00_01_02_r0() = m_10_11_12_r1() = m_20_21_22_r2() =
            _pi_mm_set1_ps(FLT_MAX);
    }
    inline void AssertInfinite() const
    {
        UT_ASSERT(m00() == FLT_MAX);
        UT_ASSERT(m01() == FLT_MAX);
        UT_ASSERT(m02() == FLT_MAX);
        UT_ASSERT(r0() == FLT_MAX);
        UT_ASSERT(m10() == FLT_MAX);
        UT_ASSERT(m11() == FLT_MAX);
        UT_ASSERT(m12() == FLT_MAX);
        UT_ASSERT(r1() == FLT_MAX);
        UT_ASSERT(m20() == FLT_MAX);
        UT_ASSERT(m21() == FLT_MAX);
        UT_ASSERT(m22() == FLT_MAX);
        UT_ASSERT(r2() == FLT_MAX);
    }
    inline void AssertFinite() const
    {
        UT_ASSERT(m00() != FLT_MAX);
        UT_ASSERT(m01() != FLT_MAX);
        UT_ASSERT(m02() != FLT_MAX);
        UT_ASSERT(r0() != FLT_MAX);
        UT_ASSERT(m10() != FLT_MAX);
        UT_ASSERT(m11() != FLT_MAX);
        UT_ASSERT(m12() != FLT_MAX);
        UT_ASSERT(r1() != FLT_MAX);
        UT_ASSERT(m20() != FLT_MAX);
        UT_ASSERT(m21() != FLT_MAX);
        UT_ASSERT(m22() != FLT_MAX);
        UT_ASSERT(r2() != FLT_MAX);
    }

    inline void Random(const float mMin, const float mMax)
    {
        UT::Random(&m00(), 3, mMin, mMax);
        UT::Random(&m10(), 3, mMin, mMax);
        UT::Random(&m20(), 3, mMin, mMax);
    }
    static inline AlignedMatrix3x3f GetRandom(const float mMin,
                                              const float mMax)
    {
        AlignedMatrix3x3f M;
        M.Random(mMin, mMax);
        return M;
    }

    static inline void aaT(const AlignedVector3f &a, AlignedMatrix3x3f &aaT)
    {
        aaT.m_00_01_02_r0() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v0()), a.v012r());
        aaT.m11() = a.v1() * a.v1();
        aaT.m12() = a.v1() * a.v2();
        aaT.m22() = a.v2() * a.v2();
        aaT.SetLowerFromUpper();
    }
    static inline void abT(const AlignedVector3f &a, const AlignedVector3f &b,
                           AlignedMatrix3x3f &abT)
    {
        abT.m_00_01_02_r0() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v0()), b.v012r());
        abT.m_10_11_12_r1() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v1()), b.v012r());
        abT.m_20_21_22_r2() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v2()), b.v012r());
    }
    static inline void Ab(const AlignedMatrix3x3f &A, const AlignedVector3f &b,
                          AlignedVector3f &Ab)
    {
        Ab.v0() = SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), b.v012r()));
        Ab.v1() = SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), b.v012r()));
        Ab.v2() = SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), b.v012r()));
    }
    static inline void AddAbTo(const AlignedMatrix3x3f &A,
                               const AlignedVector3f &b, float *Ab)
    {
        Ab[0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), b.v012r())) + Ab[0];
        Ab[1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), b.v012r())) + Ab[1];
        Ab[2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), b.v012r())) + Ab[2];
    }
    static inline void AB(const AlignedMatrix3x3f &A,
                          const AlignedMatrix3x3f &B, AlignedMatrix3x3f &AB)
    {
        const AlignedMatrix3x3f BT = B.GetTranspose();
        ABT(A, BT, AB);
    }
    static inline void ABT(const AlignedMatrix3x3f &A,
                           const AlignedMatrix3x3f &B, AlignedMatrix3x3f &ABT)
    {
        ABT.m00() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT.m01() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT.m02() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT.m10() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_00_01_02_r0()));
        ABT.m11() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT.m12() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT.m20() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_00_01_02_r0()));
        ABT.m21() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_10_11_12_r1()));
        ABT.m22() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }
    static inline void ABT(const AlignedVector3f &A, const AlignedMatrix3x3f &B,
                           AlignedVector3f &ABT)
    {
        ABT.v0() = SSE::Sum012(_pi_mm_mul_ps(A.v012r(), B.m_00_01_02_r0()));
        ABT.v1() = SSE::Sum012(_pi_mm_mul_ps(A.v012r(), B.m_10_11_12_r1()));
        ABT.v2() = SSE::Sum012(_pi_mm_mul_ps(A.v012r(), B.m_20_21_22_r2()));
    }
    static inline AlignedMatrix3x3f GetABT(const AlignedMatrix3x3f &A,
                                           const AlignedMatrix3x3f &B)
    {
        AlignedMatrix3x3f _ABT;
        ABT(A, B, _ABT);
        return _ABT;
    }
    static inline void ATB(const AlignedMatrix3x3f &A,
                           const AlignedMatrix3x3f &B, AlignedMatrix3x3f &ATB)
    {
        ATB.m_00_01_02_r0() = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(A.m00()), B.m_00_01_02_r0()),
            _pi_mm_add_ps(
                _pi_mm_mul_ps(_pi_mm_set1_ps(A.m10()), B.m_10_11_12_r1()),
                _pi_mm_mul_ps(_pi_mm_set1_ps(A.m20()), B.m_20_21_22_r2())));
        ATB.m_10_11_12_r1() = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(A.m01()), B.m_00_01_02_r0()),
            _pi_mm_add_ps(
                _pi_mm_mul_ps(_pi_mm_set1_ps(A.m11()), B.m_10_11_12_r1()),
                _pi_mm_mul_ps(_pi_mm_set1_ps(A.m21()), B.m_20_21_22_r2())));
        ATB.m_20_21_22_r2() = _pi_mm_add_ps(
            _pi_mm_mul_ps(_pi_mm_set1_ps(A.m02()), B.m_00_01_02_r0()),
            _pi_mm_add_ps(
                _pi_mm_mul_ps(_pi_mm_set1_ps(A.m12()), B.m_10_11_12_r1()),
                _pi_mm_mul_ps(_pi_mm_set1_ps(A.m22()), B.m_20_21_22_r2())));
    }
    template <typename TYPE>
    static inline void ATb(const AlignedMatrix3x3f &A, const Vector2<TYPE> &b,
                           AlignedVector3f &ATb)
    {
        ATb.v012r() = _pi_mm_add_ps(
            _pi_mm_add_ps(
                _pi_mm_mul_ps(A.m_00_01_02_r0(), _pi_mm_set1_ps(b.v0())),
                _pi_mm_mul_ps(A.m_10_11_12_r1(), _pi_mm_set1_ps(b.v1()))),
            A.m_20_21_22_r2());
    }
    template <typename TYPE>
    static inline LA::AlignedVector3f GetATb(const AlignedMatrix3x3f &A,
                                             const Vector2<TYPE> &b)
    {
        AlignedVector3f _ATb;
        ATb(A, b, _ATb);
        return _ATb;
    }

  protected:
    _pi__m128 m_data[3];
};

template <typename TYPE> class Matrix3x3
{
  public:
    inline Matrix3x3<TYPE>() = default;
    inline Matrix3x3<TYPE>(const TYPE *M) { Set(M); }
    inline const TYPE &m00() const { return m_data[0]; }
    inline TYPE &m00() { return m_data[0]; }
    inline const TYPE &m01() const { return m_data[1]; }
    inline TYPE &m01() { return m_data[1]; }
    inline const TYPE &m02() const { return m_data[2]; }
    inline TYPE &m02() { return m_data[2]; }
    inline const TYPE &m10() const { return m_data[3]; }
    inline TYPE &m10() { return m_data[3]; }
    inline const TYPE &m11() const { return m_data[4]; }
    inline TYPE &m11() { return m_data[4]; }
    inline const TYPE &m12() const { return m_data[5]; }
    inline TYPE &m12() { return m_data[5]; }
    inline const TYPE &m20() const { return m_data[6]; }
    inline TYPE &m20() { return m_data[6]; }
    inline const TYPE &m21() const { return m_data[7]; }
    inline TYPE &m21() { return m_data[7]; }
    inline const TYPE &m22() const { return m_data[8]; }
    inline TYPE &m22() { return m_data[8]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void operator=(const AlignedMatrix3x3f &M);

    inline void Set(const TYPE *M) { memcpy(this, M, sizeof(Matrix3x3<TYPE>)); }
  public:
    TYPE m_data[9];
};

typedef Matrix3x3<float> Matrix3x3f;
typedef Matrix3x3<double> Matrix3x3d;

template <> inline void Matrix3x3f::operator=(const AlignedMatrix3x3f &M)
{
    memcpy(&m00(), &M.m00(), 12);
    memcpy(&m10(), &M.m10(), 12);
    memcpy(&m20(), &M.m20(), 12);
}

template <typename TYPE> class SymmetricMatrix3x3
{
  public:
    inline SymmetricMatrix3x3<TYPE>() = default;
    inline SymmetricMatrix3x3<TYPE>(const TYPE *M) { Set(M); }
    inline const TYPE &m00() const { return m_data[0]; }
    inline TYPE &m00() { return m_data[0]; }
    inline const TYPE &m01() const { return m_data[1]; }
    inline TYPE &m01() { return m_data[1]; }
    inline const TYPE &m02() const { return m_data[2]; }
    inline TYPE &m02() { return m_data[2]; }
    inline const TYPE &m11() const { return m_data[3]; }
    inline TYPE &m11() { return m_data[3]; }
    inline const TYPE &m12() const { return m_data[4]; }
    inline TYPE &m12() { return m_data[4]; }
    inline const TYPE &m22() const { return m_data[5]; }
    inline TYPE &m22() { return m_data[5]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void operator*=(const TYPE s) { Scale(s); }
    inline void operator/=(const TYPE d) { Scale(1 / d); }
    inline void operator+=(const SymmetricMatrix3x3<TYPE> &M)
    {
        m00() = M.m00() + m00();
        m01() = M.m01() + m01();
        m02() = M.m02() + m02();
        m11() = M.m11() + m11();
        m12() = M.m12() + m12();
        m22() = M.m22() + m22();
    }
    inline SymmetricMatrix3x3<TYPE> operator*(const TYPE s) const
    {
        SymmetricMatrix3x3<TYPE> M;
        GetScaled(s, M);
        return M;
    }
    inline SymmetricMatrix3x3<TYPE> operator/(const TYPE s) const
    {
        SymmetricMatrix3x3<TYPE> M;
        GetScaled(1 / s, M);
        return M;
    }

    inline void Set(const TYPE *M)
    {
        memcpy(this, M, sizeof(SymmetricMatrix3x3<TYPE>));
    }
    inline void SetRow0(const _pi__m128 &v);
    inline void Get(AlignedMatrix3x3f &M) const;
    inline void Get(SymmetricMatrix3x3<TYPE> &M) const { M = *this; }
    inline void Get(SymmetricMatrix2x2<TYPE> &M) const
    {
        M.m00() = m00();
        M.m01() = m01();
        M.m11() = m11();
    }

    inline AlignedMatrix3x3f GetAlignedMatrix3x3f() const
    {
        AlignedMatrix3x3f M;
        Get(M);
        return M;
    }

    inline void MakeZero()
    {
        memset(this, 0, sizeof(SymmetricMatrix3x3<TYPE>));
    }
    inline void MakeMinus()
    {
        m00() = -m00();
        m01() = -m01();
        m02() = -m02();
        m11() = -m11();
        m12() = -m12();
        m22() = -m22();
    }

    inline TYPE Determinant() const
    {
        return m00() * (m11() * m22() - m12() * m12()) +
               m01() * (m12() * m02() - m01() * m22()) +
               m02() * (m01() * m12() - m11() * m02());
    }

    inline void Scale(const TYPE s)
    {
        m00() *= s;
        m01() *= s;
        m02() *= s;
        m11() *= s;
        m12() *= s;
        m22() *= s;
    }
    inline void GetScaled(const TYPE s, SymmetricMatrix3x3<TYPE> &M) const
    {
        M.m00() = m00() * s;
        M.m01() = m01() * s;
        M.m02() = m02() * s;
        M.m11() = m11() * s;
        M.m12() = m12() * s;
        M.m22() = m22() * s;
    }
    inline void GetScaled(const TYPE s, SymmetricMatrix2x2<TYPE> &M) const
    {
        M.m00() = m00() * s;
        M.m01() = m01() * s;
        M.m11() = m11() * s;
    }

    inline void MakeDiagonal(const TYPE d)
    {
        MakeZero();
        SetDiagonal(d);
    }
    inline void MakeDiagonal(const SymmetricMatrix2x2<TYPE> &D0, const TYPE d1)
    {
        MakeZero();
        SetDiagonal(D0, d1);
    }
    inline void SetDiagonal(const TYPE d)
    {
        m00() = d;
        m11() = d;
        m22() = d;
    }
    inline void SetDiagonal(const SymmetricMatrix2x2<TYPE> &D0, const TYPE d1)
    {
        m00() = D0.m00();
        m01() = D0.m01();
        m11() = D0.m11();
        m22() = d1;
    }
    inline void ScaleDiagonal(const TYPE s)
    {
        m00() *= s;
        m11() *= s;
        m22() *= s;
    }
    inline void IncreaseDiagonal(const TYPE d)
    {
        m00() = d + m00();
        m11() = d + m11();
        m22() = d + m22();
    }
    inline void IncreaseDiagonal(const TYPE d0, const TYPE d1, const TYPE d2)
    {
        m00() = d0 + m00();
        m11() = d1 + m11();
        m22() = d2 + m22();
    }
    inline void IncreaseDiagonal(const SymmetricMatrix2x2<TYPE> &D0,
                                 const TYPE d1)
    {
        m00() = D0.m00() + m00();
        m01() = D0.m01() + m01();
        m11() = D0.m11() + m11();
        m22() = d1 + m22();
    }

    inline bool GetInverse(SymmetricMatrix3x3<TYPE> &MI, const TYPE eps) const
    {
        MI.m00() = m11() * m22() - m12() * m12();
        MI.m01() = m02() * m12() - m01() * m22();
        MI.m02() = m01() * m12() - m02() * m11();
        MI.m11() = m00() * m22() - m02() * m02();
        MI.m12() = m02() * m01() - m00() * m12();
        MI.m22() = m00() * m11() - m01() * m01();
        const TYPE d = m00() * MI.m00() + m01() * MI.m01() + m02() * MI.m02();
        if (UT::NotANumber<TYPE>(d) || fabs(d) <= eps) {
            MI.Invalidate();
            return false;
        } else {
            MI /= d;
            return true;
        }
    }
    inline SymmetricMatrix3x3<TYPE> GetInverse(const TYPE eps) const
    {
        SymmetricMatrix3x3<TYPE> MI;
        GetInverse(MI, eps);
        return MI;
    }

    inline bool GetInverse(AlignedMatrix3x3f &MI, const TYPE eps) const
    {
        MI.m00() = float(m11() * m22() - m12() * m12());
        MI.m01() = float(m02() * m12() - m01() * m22());
        MI.m02() = float(m01() * m12() - m02() * m11());
        MI.m11() = float(m00() * m22() - m02() * m02());
        MI.m12() = float(m02() * m01() - m00() * m12());
        MI.m22() = float(m00() * m11() - m01() * m01());
        const TYPE d = m00() * MI.m00() + m01() * MI.m01() + m02() * MI.m02();
        if (UT::NotANumber<TYPE>(d) || fabs(d) <= eps) {
            MI.Invalidate();
            return false;
        } else {
            const TYPE s = 1 / d;
            MI.m_00_01_02_r0() =
                _pi_mm_mul_ps(MI.m_00_01_02_r0(), _pi_mm_set1_ps(float(s)));
            MI.m10() = MI.m01();
            MI.m20() = MI.m02();
            MI.m11() = float(MI.m11() * s);
            MI.m12() = MI.m21() = float(MI.m12() * s);
            MI.m22() = float(MI.m22() * s);
            return true;
        }
    }

    inline void Invalidate() { m00() = UT::Invalid<TYPE>(); }
    inline bool Valid() const { return m00() != UT::Invalid<TYPE>(); }
    inline bool Invalid() const { return m00() == UT::Invalid<TYPE>(); }
    static inline float MahalanobisDistance(const AlignedMatrix3x3f &W,
                                            const AlignedVector3f &d)
    {
#ifdef CFG_DEBUG
        W.AssertSymmetric();
#endif
        // return (W.m00() * d.v0() + W.m01() * d.v1() + W.m02() * d.v2()) *
        // d.v0()
        //	 + (W.m01() * d.v0() + W.m11() * d.v1() + W.m12() * d.v2()) *
        // d.v1()
        //	 + (W.m02() * d.v0() + W.m12() * d.v1() + W.m22() * d.v2()) *
        // d.v2();
        const LA::AlignedVector3f Wd = W * d;
        return d.Dot(Wd);
    }
    static inline TYPE MahalanobisDistance(const SymmetricMatrix3x3<TYPE> &W,
                                           const Vector2<TYPE> &d0,
                                           const TYPE d1)
    {
        return (W.m00() * d0.v0() + W.m01() * d0.v1() + W.m02() * d1) *
                   d0.v0() +
               (W.m01() * d0.v0() + W.m11() * d0.v1() + W.m12() * d1) *
                   d0.v1() +
               (W.m02() * d0.v0() + W.m12() * d0.v1() + W.m22() * d1) * d1;
    }
    static inline TYPE MahalanobisDistance(const SymmetricMatrix3x3<TYPE> &W,
                                           const Vector2<TYPE> &d)
    {
        return (W.m00() * d.v0() + W.m01() * d.v1()) * d.v0() +
               (W.m01() * d.v0() + W.m11() * d.v1()) * d.v1();
    }

    static inline void aaT(const AlignedVector3f &a, TYPE *aaT);
    static inline void aaT(const Vector2<TYPE> &a0, const TYPE a1,
                           SymmetricMatrix3x3<TYPE> &aaT)
    {
        aaT.m00() = a0.v0() * a0.v0();
        aaT.m01() = a0.v0() * a0.v1();
        aaT.m02() = a0.v0() * a1;
        aaT.m11() = a0.v1() * a0.v1();
        aaT.m12() = a0.v1() * a1;
        aaT.m22() = a1 * a1;
    }
    static inline void Ab(const SymmetricMatrix2x2<TYPE> &A0, const TYPE A1,
                          const AlignedVector3f &b, AlignedVector3f &Ab)
    {
        Ab.v0() = A0.m00() * b.v0() + A0.m01() * b.v1();
        Ab.v1() = A0.m01() * b.v0() + A0.m11() * b.v1();
        Ab.v2() = A1 * b.v2();
    }
    static inline void AAT(const AlignedMatrix3x3f &A,
                           SymmetricMatrix3x3<TYPE> &ABT);
    static inline void ABT(const AlignedMatrix3x3f &A,
                           const AlignedMatrix3x3f &B,
                           SymmetricMatrix3x3<TYPE> &ABT);
    static inline void AddABTTo(const AlignedMatrix3x3f &A,
                                const AlignedMatrix3x3f &B,
                                SymmetricMatrix3x3<TYPE> &ABT);
    static inline void AddAsATTo(const AlignedMatrix3x3f &A, const _pi__m128 &s,
                                 AlignedMatrix3x3f &As,
                                 SymmetricMatrix3x3<TYPE> &AsAT)
    {
        A.GetScaled(s, As);
        AddABTTo(As, A, AsAT);
    }
    static inline void ASAT(const AlignedMatrix3x3f &A,
                            const AlignedMatrix3x3f &S, AlignedMatrix3x3f &AS,
                            AlignedMatrix3x3f &ASAT)
    {
        AlignedMatrix3x3f::ABT(A, S, AS);
        LA::AlignedMatrix3x3f::ABT(AS, A, ASAT);
    }
    static inline void ASAT(const AlignedMatrix3x3f &A,
                            const AlignedMatrix3x3f &S, AlignedMatrix3x3f &AS,
                            SymmetricMatrix3x3<TYPE> &ASAT)
    {
        AlignedMatrix3x3f::ABT(A, S, AS);
        ABT(AS, A, ASAT);
    }
    static inline void AddASATTo(const AlignedMatrix3x3f &A,
                                 const AlignedMatrix3x3f &S,
                                 AlignedMatrix3x3f &AS,
                                 SymmetricMatrix3x3<TYPE> &ASAT)
    {
        AlignedMatrix3x3f::ABT(A, S, AS);
        AddABTTo(AS, A, ASAT);
    }
    static inline void ApB(const SymmetricMatrix3x3<TYPE> &A,
                           const SymmetricMatrix3x3<TYPE> &B,
                           SymmetricMatrix3x3<TYPE> &ApB)
    {
        ApB.m00() = A.m00() + B.m00();
        ApB.m01() = A.m01() + B.m01();
        ApB.m02() = A.m02() + B.m02();
        ApB.m11() = A.m11() + B.m11();
        ApB.m12() = A.m12() + B.m12();
        ApB.m22() = A.m22() + B.m22();
    }

  protected:
    TYPE m_data[6];
};

typedef SymmetricMatrix3x3<float> SymmetricMatrix3x3f;
typedef SymmetricMatrix3x3<double> SymmetricMatrix3x3d;

template <> inline void SymmetricMatrix3x3f::SetRow0(const _pi__m128 &v)
{
    memcpy(&m00(), &v, 12);
}

template <> inline void SymmetricMatrix3x3f::Get(AlignedMatrix3x3f &M) const
{
    M.m_00_01_02_r0() = _pi_mm_setr_ps(m00(), m01(), m02(), 0.0f);
    M.m_10_11_12_r1() = _pi_mm_setr_ps(m01(), m11(), m12(), 0.0f);
    M.m_20_21_22_r2() = _pi_mm_setr_ps(m02(), m12(), m22(), 0.0f);
}

template <>
inline void SymmetricMatrix3x3f::aaT(const LA::AlignedVector3f &a, float *aaT)
{
    const _pi__m128 t = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v0()), a.v012r());
    memcpy(aaT, &t, 12);
    aaT[3] = a.v1() * a.v1();
    aaT[4] = a.v1() * a.v2();
    aaT[5] = a.v2() * a.v2();
}

template <>
inline void SymmetricMatrix3x3f::AAT(const AlignedMatrix3x3f &A,
                                     SymmetricMatrix3x3f &AAT)
{
    AAT.m00() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), A.m_00_01_02_r0()));
    AAT.m01() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), A.m_10_11_12_r1()));
    AAT.m02() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), A.m_20_21_22_r2()));
    AAT.m11() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), A.m_10_11_12_r1()));
    AAT.m12() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), A.m_20_21_22_r2()));
    AAT.m22() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), A.m_20_21_22_r2()));
}

template <>
inline void SymmetricMatrix3x3f::ABT(const AlignedMatrix3x3f &A,
                                     const AlignedMatrix3x3f &B,
                                     SymmetricMatrix3x3f &ABT)
{
    ABT.m00() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
    ABT.m01() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
    ABT.m02() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
    ABT.m11() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
    ABT.m12() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
    ABT.m22() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
}

template <>
inline void SymmetricMatrix3x3f::AddABTTo(const AlignedMatrix3x3f &A,
                                          const AlignedMatrix3x3f &B,
                                          SymmetricMatrix3x3f &ABT)
{
    ABT.m00() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0())) +
        ABT.m00();
    ABT.m01() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1())) +
        ABT.m01();
    ABT.m02() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2())) +
        ABT.m02();
    ABT.m11() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1())) +
        ABT.m11();
    ABT.m12() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2())) +
        ABT.m12();
    ABT.m22() =
        SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2())) +
        ABT.m22();
}
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix3x3f : public Eigen::Matrix3f
{
  public:
    inline EigenMatrix3x3f() = default;
    inline EigenMatrix3x3f(const Eigen::Matrix3f &e_M) : Eigen::Matrix3f(e_M) {}
    inline EigenMatrix3x3f(const LA::AlignedMatrix3x3f &M) : Eigen::Matrix3f()
    {
        Eigen::Matrix3f &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(0, 2) = M.m02();
        e_M(1, 0) = M.m10();
        e_M(1, 1) = M.m11();
        e_M(1, 2) = M.m12();
        e_M(2, 0) = M.m20();
        e_M(2, 1) = M.m21();
        e_M(2, 2) = M.m22();
    }
    inline EigenMatrix3x3f(const LA::SymmetricMatrix3x3f &M) : Eigen::Matrix3f()
    {
        Eigen::Matrix3f &e_M = *this;
        e_M(0, 0) = M.m00();
        e_M(0, 1) = M.m01();
        e_M(0, 2) = M.m02();
        e_M(1, 0) = M.m01();
        e_M(1, 1) = M.m11();
        e_M(1, 2) = M.m12();
        e_M(2, 0) = M.m02();
        e_M(2, 1) = M.m12();
        e_M(2, 2) = M.m22();
    }
    inline EigenMatrix3x3f(const LA::SymmetricMatrix2x2f &M0, const float M1)
        : Eigen::Matrix3f()
    {
        Eigen::Matrix3f &e_M = *this;
        e_M(0, 0) = M0.m00();
        e_M(0, 1) = M0.m01();
        e_M(0, 2) = 0.0f;
        e_M(1, 0) = M0.m01();
        e_M(1, 1) = M0.m11();
        e_M(1, 2) = 0.0f;
        e_M(2, 0) = 0.0f;
        e_M(2, 1) = 0.0f;
        e_M(2, 2) = M1;
    }
    inline void operator=(const Eigen::Matrix3f &e_M)
    {
        *((Eigen::Matrix3f *)this) = e_M;
    }
    inline LA::AlignedMatrix3x3f GetAlignedMatrix3x3f() const
    {
        LA::AlignedMatrix3x3f M;
        const Eigen::Matrix3f &e_M = *this;
        M.m00() = e_M(0, 0);
        M.m01() = e_M(0, 1);
        M.m02() = e_M(0, 2);
        M.m10() = e_M(1, 0);
        M.m11() = e_M(1, 1);
        M.m12() = e_M(1, 2);
        M.m20() = e_M(2, 0);
        M.m21() = e_M(2, 1);
        M.m22() = e_M(2, 2);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix3x3f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix3x3f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix3x3f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const LA::SymmetricMatrix3x3f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix3x3f().AssertEqual(M.GetAlignedMatrix3x3f(),
                                                  verbose);
    }
    inline bool AssertEqual(const EigenMatrix3x3f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix3x3f(), verbose);
    }
    static inline EigenMatrix3x3f Zero()
    {
        return EigenMatrix3x3f(Eigen::Matrix3f::Zero());
    }
};
#endif
#endif
