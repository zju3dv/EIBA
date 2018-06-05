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

#ifndef _MATRIX_8x8_H_
#define _MATRIX_8x8_H_

#include "LinearSystem.h"
#include "Vector8.h"

namespace LA
{
class AlignedMatrix8x8f
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
    inline const _pi__m128 &m_20_21_22_23() const { return m_data[4]; }
    inline _pi__m128 &m_20_21_22_23() { return m_data[4]; }
    inline const _pi__m128 &m_24_25_26_27() const { return m_data[5]; }
    inline _pi__m128 &m_24_25_26_27() { return m_data[5]; }
    inline const _pi__m128 &m_30_31_32_33() const { return m_data[6]; }
    inline _pi__m128 &m_30_31_32_33() { return m_data[6]; }
    inline const _pi__m128 &m_34_35_36_37() const { return m_data[7]; }
    inline _pi__m128 &m_34_35_36_37() { return m_data[7]; }
    inline const _pi__m128 &m_40_41_42_43() const { return m_data[8]; }
    inline _pi__m128 &m_40_41_42_43() { return m_data[8]; }
    inline const _pi__m128 &m_44_45_46_47() const { return m_data[9]; }
    inline _pi__m128 &m_44_45_46_47() { return m_data[9]; }
    inline const _pi__m128 &m_50_51_52_53() const { return m_data[10]; }
    inline _pi__m128 &m_50_51_52_53() { return m_data[10]; }
    inline const _pi__m128 &m_54_55_56_57() const { return m_data[11]; }
    inline _pi__m128 &m_54_55_56_57() { return m_data[11]; }
    inline const _pi__m128 &m_60_61_62_63() const { return m_data[12]; }
    inline _pi__m128 &m_60_61_62_63() { return m_data[12]; }
    inline const _pi__m128 &m_64_65_66_67() const { return m_data[13]; }
    inline _pi__m128 &m_64_65_66_67() { return m_data[13]; }
    inline const _pi__m128 &m_70_71_72_73() const { return m_data[14]; }
    inline _pi__m128 &m_70_71_72_73() { return m_data[14]; }
    inline const _pi__m128 &m_74_75_76_77() const { return m_data[15]; }
    inline _pi__m128 &m_74_75_76_77() { return m_data[15]; }
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
    inline const float &m26() const { return m_data[5].m128_f32[2]; }
    inline float &m26() { return m_data[5].m128_f32[2]; }
    inline const float &m27() const { return m_data[5].m128_f32[3]; }
    inline float &m27() { return m_data[5].m128_f32[3]; }
    inline const float &m30() const { return m_data[6].m128_f32[0]; }
    inline float &m30() { return m_data[6].m128_f32[0]; }
    inline const float &m31() const { return m_data[6].m128_f32[1]; }
    inline float &m31() { return m_data[6].m128_f32[1]; }
    inline const float &m32() const { return m_data[6].m128_f32[2]; }
    inline float &m32() { return m_data[6].m128_f32[2]; }
    inline const float &m33() const { return m_data[6].m128_f32[3]; }
    inline float &m33() { return m_data[6].m128_f32[3]; }
    inline const float &m34() const { return m_data[7].m128_f32[0]; }
    inline float &m34() { return m_data[7].m128_f32[0]; }
    inline const float &m35() const { return m_data[7].m128_f32[1]; }
    inline float &m35() { return m_data[7].m128_f32[1]; }
    inline const float &m36() const { return m_data[7].m128_f32[2]; }
    inline float &m36() { return m_data[7].m128_f32[2]; }
    inline const float &m37() const { return m_data[7].m128_f32[3]; }
    inline float &m37() { return m_data[7].m128_f32[3]; }
    inline const float &m40() const { return m_data[8].m128_f32[0]; }
    inline float &m40() { return m_data[8].m128_f32[0]; }
    inline const float &m41() const { return m_data[8].m128_f32[1]; }
    inline float &m41() { return m_data[8].m128_f32[1]; }
    inline const float &m42() const { return m_data[8].m128_f32[2]; }
    inline float &m42() { return m_data[8].m128_f32[2]; }
    inline const float &m43() const { return m_data[8].m128_f32[3]; }
    inline float &m43() { return m_data[8].m128_f32[3]; }
    inline const float &m44() const { return m_data[9].m128_f32[0]; }
    inline float &m44() { return m_data[9].m128_f32[0]; }
    inline const float &m45() const { return m_data[9].m128_f32[1]; }
    inline float &m45() { return m_data[9].m128_f32[1]; }
    inline const float &m46() const { return m_data[9].m128_f32[2]; }
    inline float &m46() { return m_data[9].m128_f32[2]; }
    inline const float &m47() const { return m_data[9].m128_f32[3]; }
    inline float &m47() { return m_data[9].m128_f32[3]; }
    inline const float &m50() const { return m_data[10].m128_f32[0]; }
    inline float &m50() { return m_data[10].m128_f32[0]; }
    inline const float &m51() const { return m_data[10].m128_f32[1]; }
    inline float &m51() { return m_data[10].m128_f32[1]; }
    inline const float &m52() const { return m_data[10].m128_f32[2]; }
    inline float &m52() { return m_data[10].m128_f32[2]; }
    inline const float &m53() const { return m_data[10].m128_f32[3]; }
    inline float &m53() { return m_data[10].m128_f32[3]; }
    inline const float &m54() const { return m_data[11].m128_f32[0]; }
    inline float &m54() { return m_data[11].m128_f32[0]; }
    inline const float &m55() const { return m_data[11].m128_f32[1]; }
    inline float &m55() { return m_data[11].m128_f32[1]; }
    inline const float &m56() const { return m_data[11].m128_f32[2]; }
    inline float &m56() { return m_data[11].m128_f32[2]; }
    inline const float &m57() const { return m_data[11].m128_f32[3]; }
    inline float &m57() { return m_data[11].m128_f32[3]; }
    inline const float &m60() const { return m_data[12].m128_f32[0]; }
    inline float &m60() { return m_data[12].m128_f32[0]; }
    inline const float &m61() const { return m_data[12].m128_f32[1]; }
    inline float &m61() { return m_data[12].m128_f32[1]; }
    inline const float &m62() const { return m_data[12].m128_f32[2]; }
    inline float &m62() { return m_data[12].m128_f32[2]; }
    inline const float &m63() const { return m_data[12].m128_f32[3]; }
    inline float &m63() { return m_data[12].m128_f32[3]; }
    inline const float &m64() const { return m_data[13].m128_f32[0]; }
    inline float &m64() { return m_data[13].m128_f32[0]; }
    inline const float &m65() const { return m_data[13].m128_f32[1]; }
    inline float &m65() { return m_data[13].m128_f32[1]; }
    inline const float &m66() const { return m_data[13].m128_f32[2]; }
    inline float &m66() { return m_data[13].m128_f32[2]; }
    inline const float &m67() const { return m_data[13].m128_f32[3]; }
    inline float &m67() { return m_data[13].m128_f32[3]; }
    inline const float &m70() const { return m_data[14].m128_f32[0]; }
    inline float &m70() { return m_data[14].m128_f32[0]; }
    inline const float &m71() const { return m_data[14].m128_f32[1]; }
    inline float &m71() { return m_data[14].m128_f32[1]; }
    inline const float &m72() const { return m_data[14].m128_f32[2]; }
    inline float &m72() { return m_data[14].m128_f32[2]; }
    inline const float &m73() const { return m_data[14].m128_f32[3]; }
    inline float &m73() { return m_data[14].m128_f32[3]; }
    inline const float &m74() const { return m_data[15].m128_f32[0]; }
    inline float &m74() { return m_data[15].m128_f32[0]; }
    inline const float &m75() const { return m_data[15].m128_f32[1]; }
    inline float &m75() { return m_data[15].m128_f32[1]; }
    inline const float &m76() const { return m_data[15].m128_f32[2]; }
    inline float &m76() { return m_data[15].m128_f32[2]; }
    inline const float &m77() const { return m_data[15].m128_f32[3]; }
    inline float &m77() { return m_data[15].m128_f32[3]; }
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix8x8f)); }
    inline void MakeIdentity()
    {
        MakeZero();
        m00() = m11() = m22() = m33() = m44() = m55() = m66() = m77() = 1.0f;
    }

    inline void SetLowerFromUpper()
    {
        m10() = m01();
        m20() = m02();
        m21() = m12();
        m30() = m03();
        m31() = m13();
        m32() = m23();
        m40() = m04();
        m41() = m14();
        m42() = m24();
        m43() = m34();
        m50() = m05();
        m51() = m15();
        m52() = m25();
        m53() = m35();
        m54() = m45();
        m60() = m06();
        m61() = m16();
        m62() = m26();
        m63() = m36();
        m64() = m46();
        m65() = m56();
        m70() = m07();
        m71() = m17();
        m72() = m27();
        m73() = m37();
        m74() = m47();
        m75() = m57();
        m76() = m67();
    }

    static inline bool SolveLDL(AlignedMatrix8x8f &A, AlignedVector8f &b)
    {
        float *_A[8] = {&A.m00(), &A.m10(), &A.m20(), &A.m30(),
                        &A.m40(), &A.m50(), &A.m60(), &A.m70()};
        return LS::SolveLDL<float, 8>(_A, b);
    }

    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e %e %e %e %e\n", m00(), m01(), m02(), m03(),
                      m04(), m05(), m06(), m07());
            UT::Print("%e %e %e %e %e %e %e %e\n", m10(), m11(), m12(), m13(),
                      m14(), m15(), m16(), m17());
            UT::Print("%e %e %e %e %e %e %e %e\n", m20(), m21(), m22(), m23(),
                      m24(), m25(), m26(), m27());
            UT::Print("%e %e %e %e %e %e %e %e\n", m30(), m31(), m32(), m33(),
                      m34(), m35(), m36(), m37());
            UT::Print("%e %e %e %e %e %e %e %e\n", m40(), m41(), m42(), m43(),
                      m44(), m45(), m46(), m47());
            UT::Print("%e %e %e %e %e %e %e %e\n", m50(), m51(), m52(), m53(),
                      m54(), m55(), m56(), m57());
            UT::Print("%e %e %e %e %e %e %e %e\n", m60(), m61(), m62(), m63(),
                      m64(), m65(), m66(), m67());
            UT::Print("%e %e %e %e %e %e %e %e\n", m70(), m71(), m72(), m73(),
                      m74(), m75(), m76(), m77());
        } else {
            UT::Print("%f %f %f %f %f %f %f %f\n", m00(), m01(), m02(), m03(),
                      m04(), m05(), m06(), m07());
            UT::Print("%f %f %f %f %f %f %f %f\n", m10(), m11(), m12(), m13(),
                      m14(), m15(), m16(), m17());
            UT::Print("%f %f %f %f %f %f %f %f\n", m20(), m21(), m22(), m23(),
                      m24(), m25(), m26(), m27());
            UT::Print("%f %f %f %f %f %f %f %f\n", m30(), m31(), m32(), m33(),
                      m34(), m35(), m36(), m37());
            UT::Print("%f %f %f %f %f %f %f %f\n", m40(), m41(), m42(), m43(),
                      m44(), m45(), m46(), m47());
            UT::Print("%f %f %f %f %f %f %f %f\n", m50(), m51(), m52(), m53(),
                      m54(), m55(), m56(), m57());
            UT::Print("%f %f %f %f %f %f %f %f\n", m60(), m61(), m62(), m63(),
                      m64(), m65(), m66(), m67());
            UT::Print("%f %f %f %f %f %f %f %f\n", m70(), m71(), m72(), m73(),
                      m74(), m75(), m76(), m77());
        }
    }
    inline bool AssertEqual(const AlignedMatrix8x8f &M,
                            const int verbose = 1) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 64, 0)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

  protected:
    _pi__m128 m_data[16];
};

template <typename TYPE> class SymmetricMatrix8x8
{
  public:
    inline SymmetricMatrix8x8<TYPE>() = default;
    inline SymmetricMatrix8x8<TYPE>(const TYPE *M) { Set(M); }
    inline const TYPE &m00() const { return m_data[0]; }
    inline TYPE &m00() { return m_data[0]; }
    inline const TYPE &m01() const { return m_data[1]; }
    inline TYPE &m01() { return m_data[1]; }
    inline const TYPE &m02() const { return m_data[2]; }
    inline TYPE &m02() { return m_data[2]; }
    inline const TYPE &m03() const { return m_data[3]; }
    inline TYPE &m03() { return m_data[3]; }
    inline const TYPE &m04() const { return m_data[4]; }
    inline TYPE &m04() { return m_data[4]; }
    inline const TYPE &m05() const { return m_data[5]; }
    inline TYPE &m05() { return m_data[5]; }
    inline const TYPE &m06() const { return m_data[6]; }
    inline TYPE &m06() { return m_data[6]; }
    inline const TYPE &m07() const { return m_data[7]; }
    inline TYPE &m07() { return m_data[7]; }
    inline const TYPE &m11() const { return m_data[8]; }
    inline TYPE &m11() { return m_data[8]; }
    inline const TYPE &m12() const { return m_data[9]; }
    inline TYPE &m12() { return m_data[9]; }
    inline const TYPE &m13() const { return m_data[10]; }
    inline TYPE &m13() { return m_data[10]; }
    inline const TYPE &m14() const { return m_data[11]; }
    inline TYPE &m14() { return m_data[11]; }
    inline const TYPE &m15() const { return m_data[12]; }
    inline TYPE &m15() { return m_data[12]; }
    inline const TYPE &m16() const { return m_data[13]; }
    inline TYPE &m16() { return m_data[13]; }
    inline const TYPE &m17() const { return m_data[14]; }
    inline TYPE &m17() { return m_data[14]; }
    inline const TYPE &m22() const { return m_data[15]; }
    inline TYPE &m22() { return m_data[15]; }
    inline const TYPE &m23() const { return m_data[16]; }
    inline TYPE &m23() { return m_data[16]; }
    inline const TYPE &m24() const { return m_data[17]; }
    inline TYPE &m24() { return m_data[17]; }
    inline const TYPE &m25() const { return m_data[18]; }
    inline TYPE &m25() { return m_data[18]; }
    inline const TYPE &m26() const { return m_data[19]; }
    inline TYPE &m26() { return m_data[19]; }
    inline const TYPE &m27() const { return m_data[20]; }
    inline TYPE &m27() { return m_data[20]; }
    inline const TYPE &m33() const { return m_data[21]; }
    inline TYPE &m33() { return m_data[21]; }
    inline const TYPE &m34() const { return m_data[22]; }
    inline TYPE &m34() { return m_data[22]; }
    inline const TYPE &m35() const { return m_data[23]; }
    inline TYPE &m35() { return m_data[23]; }
    inline const TYPE &m36() const { return m_data[24]; }
    inline TYPE &m36() { return m_data[24]; }
    inline const TYPE &m37() const { return m_data[25]; }
    inline TYPE &m37() { return m_data[25]; }
    inline const TYPE &m44() const { return m_data[26]; }
    inline TYPE &m44() { return m_data[26]; }
    inline const TYPE &m45() const { return m_data[27]; }
    inline TYPE &m45() { return m_data[27]; }
    inline const TYPE &m46() const { return m_data[28]; }
    inline TYPE &m46() { return m_data[28]; }
    inline const TYPE &m47() const { return m_data[29]; }
    inline TYPE &m47() { return m_data[29]; }
    inline const TYPE &m55() const { return m_data[30]; }
    inline TYPE &m55() { return m_data[30]; }
    inline const TYPE &m56() const { return m_data[31]; }
    inline TYPE &m56() { return m_data[31]; }
    inline const TYPE &m57() const { return m_data[32]; }
    inline TYPE &m57() { return m_data[32]; }
    inline const TYPE &m66() const { return m_data[33]; }
    inline TYPE &m66() { return m_data[33]; }
    inline const TYPE &m67() const { return m_data[34]; }
    inline TYPE &m67() { return m_data[34]; }
    inline const TYPE &m77() const { return m_data[35]; }
    inline TYPE &m77() { return m_data[35]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void operator+=(const SymmetricMatrix8x8<TYPE> &M);
    inline void operator*=(const _pi__m128 &s);

    inline void Set(const TYPE *M)
    {
        memcpy(this, M, sizeof(SymmetricMatrix8x8<TYPE>));
    }

    inline AlignedMatrix8x8f GetAlignedMatrix8x8f() const;

    inline void MakeZero()
    {
        memset(this, 0, sizeof(SymmetricMatrix8x8<TYPE>));
    }

    static inline void aaT(const AlignedVector8f &v, TYPE *aaT);
    static inline void ApB(const SymmetricMatrix8x8<TYPE> &A,
                           const SymmetricMatrix8x8<TYPE> &B,
                           SymmetricMatrix8x8<TYPE> &ApB);

  protected:
    TYPE m_data[36];
};

typedef SymmetricMatrix8x8<float> SymmetricMatrix8x8f;
typedef SymmetricMatrix8x8<double> SymmetricMatrix8x8d;

template <>
inline void SymmetricMatrix8x8f::operator+=(const SymmetricMatrix8x8f &M)
{
    _pi__m128 *m1 = (_pi__m128 *)this;
    const _pi__m128 *m2 = (_pi__m128 *)M.m_data;
    m1[0] = _pi_mm_add_ps(m2[0], m1[0]);
    m1[1] = _pi_mm_add_ps(m2[1], m1[1]);
    m1[2] = _pi_mm_add_ps(m2[2], m1[2]);
    m1[3] = _pi_mm_add_ps(m2[3], m1[3]);
    m1[4] = _pi_mm_add_ps(m2[4], m1[4]);
    m1[5] = _pi_mm_add_ps(m2[5], m1[5]);
    m1[6] = _pi_mm_add_ps(m2[6], m1[6]);
    m1[7] = _pi_mm_add_ps(m2[7], m1[7]);
    m1[8] = _pi_mm_add_ps(m2[8], m1[8]);
}

template <> inline void SymmetricMatrix8x8f::operator*=(const _pi__m128 &s)
{
    _pi__m128 *m = (_pi__m128 *)m_data;
    m[0] = _pi_mm_mul_ps(s, m[0]);
    m[1] = _pi_mm_mul_ps(s, m[1]);
    m[2] = _pi_mm_mul_ps(s, m[2]);
    m[3] = _pi_mm_mul_ps(s, m[3]);
    m[4] = _pi_mm_mul_ps(s, m[4]);
    m[5] = _pi_mm_mul_ps(s, m[5]);
    m[6] = _pi_mm_mul_ps(s, m[6]);
    m[7] = _pi_mm_mul_ps(s, m[7]);
    m[8] = _pi_mm_mul_ps(s, m[8]);
}

template <>
inline AlignedMatrix8x8f SymmetricMatrix8x8f::GetAlignedMatrix8x8f() const
{
    AlignedMatrix8x8f M;
    M.m00() = m00();
    M.m01() = M.m10() = m01();
    M.m02() = M.m20() = m02();
    M.m03() = M.m30() = m03();
    M.m04() = M.m40() = m04();
    M.m05() = M.m50() = m05();
    M.m06() = M.m60() = m06();
    M.m07() = M.m70() = m07();
    M.m11() = m11();
    M.m12() = M.m21() = m12();
    M.m13() = M.m31() = m13();
    M.m14() = M.m41() = m14();
    M.m15() = M.m51() = m15();
    M.m16() = M.m61() = m16();
    M.m17() = M.m71() = m17();
    M.m22() = m22();
    M.m23() = M.m32() = m23();
    M.m24() = M.m42() = m24();
    M.m25() = M.m52() = m25();
    M.m26() = M.m62() = m26();
    M.m27() = M.m72() = m27();
    M.m33() = m33();
    M.m34() = M.m43() = m34();
    M.m35() = M.m53() = m35();
    M.m36() = M.m63() = m36();
    M.m37() = M.m73() = m37();
    M.m44() = m44();
    M.m45() = M.m54() = m45();
    M.m46() = M.m64() = m46();
    M.m47() = M.m74() = m47();
    M.m55() = m55();
    M.m56() = M.m65() = m56();
    M.m57() = M.m75() = m57();
    M.m66() = m66();
    M.m67() = M.m76() = m67();
    M.m77() = m77();
    return M;
}

template <>
inline void SymmetricMatrix8x8f::aaT(const AlignedVector8f &a, float *aaT)
{
    AlignedVector8f t;
    t.v4567() = _pi_mm_set1_ps(a.v0());
    t.v0123() = _pi_mm_mul_ps(t.v4567(), a.v0123());
    t.v4567() = _pi_mm_mul_ps(t.v4567(), a.v4567());
    memcpy(aaT, t, sizeof(t));
    t.v4567() = _pi_mm_set1_ps(a.v1());
    t.v0123() = _pi_mm_mul_ps(t.v4567(), a.v0123());
    t.v4567() = _pi_mm_mul_ps(t.v4567(), a.v4567());
    memcpy(aaT + 8, &t.v1(), 28);
    aaT[15] = a.v2() * a.v2();
    aaT[16] = a.v2() * a.v3();
    t.v4567() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v2()), a.v4567());
    memcpy(aaT + 17, &t.v4(), 16);
    aaT[21] = a.v3() * a.v3();
    t.v4567() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v3()), a.v4567());
    memcpy(aaT + 22, &t.v4(), 16);
    t.v4567() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v4()), a.v4567());
    memcpy(aaT + 26, &t.v4(), 16);
    t.v4567() = _pi_mm_mul_ps(_pi_mm_set1_ps(a.v5()), a.v4567());
    memcpy(aaT + 30, &t.v5(), 12);
    aaT[33] = a.v6() * a.v6();
    aaT[34] = a.v6() * a.v7();
    aaT[35] = a.v7() * a.v7();
}

template <>
inline void SymmetricMatrix8x8f::ApB(const SymmetricMatrix8x8f &A,
                                     const SymmetricMatrix8x8f &B,
                                     SymmetricMatrix8x8f &ApB)
{
    const _pi__m128 *a = (_pi__m128 *)A.m_data, *b = (_pi__m128 *)B.m_data;
    _pi__m128 *apb = (_pi__m128 *)ApB.m_data;
    apb[0] = _pi_mm_add_ps(a[0], b[0]);
    apb[1] = _pi_mm_add_ps(a[1], b[1]);
    apb[2] = _pi_mm_add_ps(a[2], b[2]);
    apb[3] = _pi_mm_add_ps(a[3], b[3]);
    apb[4] = _pi_mm_add_ps(a[4], b[4]);
    apb[5] = _pi_mm_add_ps(a[5], b[5]);
    apb[6] = _pi_mm_add_ps(a[6], b[6]);
    apb[7] = _pi_mm_add_ps(a[7], b[7]);
    apb[8] = _pi_mm_add_ps(a[8], b[8]);
}
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix8x8f : public Eigen::Matrix<float, 8, 8>
{
  public:
    inline EigenMatrix8x8f() = default;
    inline EigenMatrix8x8f(const Eigen::Matrix<float, 8, 8> &e_M)
        : Eigen::Matrix<float, 8, 8>(e_M)
    {
    }
    inline EigenMatrix8x8f(const LA::AlignedMatrix8x8f &M)
        : Eigen::Matrix<float, 8, 8>()
    {
        const float *_M[8] = {&M.m00(), &M.m10(), &M.m20(), &M.m30(),
                              &M.m40(), &M.m50(), &M.m60(), &M.m70()};
        Eigen::Matrix<float, 8, 8> &e_M = *this;
        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j) e_M(i, j) = _M[i][j];
    }
    inline EigenMatrix8x8f(const LA::SymmetricMatrix8x8f &M)
        : Eigen::Matrix<float, 8, 8>()
    {
        Eigen::Matrix<float, 8, 8> &e_M = *this;
        const float *_M = M;
        for (int i = 0, k = 0; i < 8; ++i)
            for (int j = i; j < 8; ++j, ++k) e_M(i, j) = e_M(j, i) = _M[k];
    }
    inline void operator=(const Eigen::Matrix<float, 8, 8> &e_M)
    {
        *((Eigen::Matrix<float, 8, 8> *)this) = e_M;
    }
    inline LA::AlignedMatrix8x8f GetAlignedMatrix8x8f() const
    {
        LA::AlignedMatrix8x8f M;
        float *_M[8] = {&M.m00(), &M.m10(), &M.m20(), &M.m30(),
                        &M.m40(), &M.m50(), &M.m60(), &M.m70()};
        const Eigen::Matrix<float, 8, 8> &e_M = *this;
        for (int i = 0; i < 8; ++i)
            for (int j = 0; j < 8; ++j) _M[i][j] = e_M(i, j);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix8x8f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix8x8f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix8x8f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const LA::SymmetricMatrix8x8f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix8x8f().AssertEqual(M.GetAlignedMatrix8x8f(),
                                                  verbose);
    }
    inline bool AssertEqual(const EigenMatrix8x8f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix8x8f(), verbose);
    }
};
#endif
#endif
