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

#ifndef _MATRIX_6x6_H_
#define _MATRIX_6x6_H_

#include "LinearSystem.h"
#include "Vector6.h"

namespace LA
{
#if 0
class AlignedMatrix6x6f
{

public:

	inline const _pi__m128& m_00_01_02_03() const { return m_data[0]; }	inline _pi__m128& m_00_01_02_03() { return m_data[0]; }
	inline const _pi__m128& m_04_05_x_x  () const { return m_data[1]; }	inline _pi__m128& m_04_05_x_x  () { return m_data[1]; }
	inline const _pi__m128& m_10_11_12_13() const { return m_data[2]; }	inline _pi__m128& m_10_11_12_13() { return m_data[2]; }
	inline const _pi__m128& m_14_15_x_x  () const { return m_data[3]; }	inline _pi__m128& m_14_15_x_x  () { return m_data[3]; }
	inline const _pi__m128& m_20_21_22_23() const { return m_data[4]; }	inline _pi__m128& m_20_21_22_23() { return m_data[4]; }
	inline const _pi__m128& m_24_25_x_x  () const { return m_data[5]; }	inline _pi__m128& m_24_25_x_x  () { return m_data[5]; }
	inline const _pi__m128& m_30_31_32_33() const { return m_data[6]; }	inline _pi__m128& m_30_31_32_33() { return m_data[6]; }
	inline const _pi__m128& m_34_35_x_x  () const { return m_data[7]; }	inline _pi__m128& m_34_35_x_x  () { return m_data[7]; }
	inline const _pi__m128& m_40_41_42_43() const { return m_data[8]; }	inline _pi__m128& m_40_41_42_43() { return m_data[8]; }
	inline const _pi__m128& m_44_45_x_x  () const { return m_data[9]; }	inline _pi__m128& m_44_45_x_x  () { return m_data[9]; }
	inline const _pi__m128& m_50_51_52_53() const { return m_data[10]; }	inline _pi__m128& m_50_51_52_53() { return m_data[10]; }
	inline const _pi__m128& m_54_55_x_x  () const { return m_data[11]; }	inline _pi__m128& m_54_55_x_x  () { return m_data[11]; }
	inline const float& m00() const { return m_data[0].m128_f32[0]; }	inline float& m00() { return m_data[0].m128_f32[0]; }
	inline const float& m01() const { return m_data[0].m128_f32[1]; }	inline float& m01() { return m_data[0].m128_f32[1]; }
	inline const float& m02() const { return m_data[0].m128_f32[2]; }	inline float& m02() { return m_data[0].m128_f32[2]; }
	inline const float& m03() const { return m_data[0].m128_f32[3]; }	inline float& m03() { return m_data[0].m128_f32[3]; }
	inline const float& m04() const { return m_data[1].m128_f32[0]; }	inline float& m04() { return m_data[1].m128_f32[0]; }
	inline const float& m05() const { return m_data[1].m128_f32[1]; }	inline float& m05() { return m_data[1].m128_f32[1]; }
	inline const float& m10() const { return m_data[2].m128_f32[0]; }	inline float& m10() { return m_data[2].m128_f32[0]; }
	inline const float& m11() const { return m_data[2].m128_f32[1]; }	inline float& m11() { return m_data[2].m128_f32[1]; }
	inline const float& m12() const { return m_data[2].m128_f32[2]; }	inline float& m12() { return m_data[2].m128_f32[2]; }
	inline const float& m13() const { return m_data[2].m128_f32[3]; }	inline float& m13() { return m_data[2].m128_f32[3]; }
	inline const float& m14() const { return m_data[3].m128_f32[0]; }	inline float& m14() { return m_data[3].m128_f32[0]; }
	inline const float& m15() const { return m_data[3].m128_f32[1]; }	inline float& m15() { return m_data[3].m128_f32[1]; }
	inline const float& m20() const { return m_data[4].m128_f32[0]; }	inline float& m20() { return m_data[4].m128_f32[0]; }
	inline const float& m21() const { return m_data[4].m128_f32[1]; }	inline float& m21() { return m_data[4].m128_f32[1]; }
	inline const float& m22() const { return m_data[4].m128_f32[2]; }	inline float& m22() { return m_data[4].m128_f32[2]; }
	inline const float& m23() const { return m_data[4].m128_f32[3]; }	inline float& m23() { return m_data[4].m128_f32[3]; }
	inline const float& m24() const { return m_data[5].m128_f32[0]; }	inline float& m24() { return m_data[5].m128_f32[0]; }
	inline const float& m25() const { return m_data[5].m128_f32[1]; }	inline float& m25() { return m_data[5].m128_f32[1]; }
	inline const float& m30() const { return m_data[6].m128_f32[0]; }	inline float& m30() { return m_data[6].m128_f32[0]; }
	inline const float& m31() const { return m_data[6].m128_f32[1]; }	inline float& m31() { return m_data[6].m128_f32[1]; }
	inline const float& m32() const { return m_data[6].m128_f32[2]; }	inline float& m32() { return m_data[6].m128_f32[2]; }
	inline const float& m33() const { return m_data[6].m128_f32[3]; }	inline float& m33() { return m_data[6].m128_f32[3]; }
	inline const float& m34() const { return m_data[7].m128_f32[0]; }	inline float& m34() { return m_data[7].m128_f32[0]; }
	inline const float& m35() const { return m_data[7].m128_f32[1]; }	inline float& m35() { return m_data[7].m128_f32[1]; }
	inline const float& m40() const { return m_data[8].m128_f32[0]; }	inline float& m40() { return m_data[8].m128_f32[0]; }
	inline const float& m41() const { return m_data[8].m128_f32[1]; }	inline float& m41() { return m_data[8].m128_f32[1]; }
	inline const float& m42() const { return m_data[8].m128_f32[2]; }	inline float& m42() { return m_data[8].m128_f32[2]; }
	inline const float& m43() const { return m_data[8].m128_f32[3]; }	inline float& m43() { return m_data[8].m128_f32[3]; }
	inline const float& m44() const { return m_data[9].m128_f32[0]; }	inline float& m44() { return m_data[9].m128_f32[0]; }
	inline const float& m45() const { return m_data[9].m128_f32[1]; }	inline float& m45() { return m_data[9].m128_f32[1]; }
	inline const float& m50() const { return m_data[10].m128_f32[0]; }	inline float& m50() { return m_data[10].m128_f32[0]; }
	inline const float& m51() const { return m_data[10].m128_f32[1]; }	inline float& m51() { return m_data[10].m128_f32[1]; }
	inline const float& m52() const { return m_data[10].m128_f32[2]; }	inline float& m52() { return m_data[10].m128_f32[2]; }
	inline const float& m53() const { return m_data[10].m128_f32[3]; }	inline float& m53() { return m_data[10].m128_f32[3]; }
	inline const float& m54() const { return m_data[11].m128_f32[0]; }	inline float& m54() { return m_data[11].m128_f32[0]; }
	inline const float& m55() const { return m_data[11].m128_f32[1]; }	inline float& m55() { return m_data[11].m128_f32[1]; }

	inline operator const float* () const { return (const float *) this; }
	inline operator		  float* ()		  { return (	  float *) this; }

	inline void operator += (const AlignedMatrix6x6f &M)
	{
		m_00_01_02_03() = _pi_mm_add_ps(M.m_00_01_02_03(), m_00_01_02_03());	m04() = M.m04() + m04();	m05() = M.m05() + m05();
		m_10_11_12_13() = _pi_mm_add_ps(M.m_10_11_12_13(), m_10_11_12_13());	m14() = M.m14() + m14();	m15() = M.m15() + m15();
		m_20_21_22_23() = _pi_mm_add_ps(M.m_20_21_22_23(), m_20_21_22_23());	m24() = M.m24() + m24();	m25() = M.m25() + m25();
		m_30_31_32_33() = _pi_mm_add_ps(M.m_30_31_32_33(), m_30_31_32_33());	m34() = M.m34() + m34();	m35() = M.m35() + m35();
		m_40_41_42_43() = _pi_mm_add_ps(M.m_40_41_42_43(), m_40_41_42_43());	m44() = M.m44() + m44();	m45() = M.m45() + m45();
		m_50_51_52_53() = _pi_mm_add_ps(M.m_50_51_52_53(), m_50_51_52_53());	m54() = M.m54() + m54();	m55() = M.m55() + m55();
	}
	inline void operator -= (const AlignedMatrix6x6f &M)
	{
		m_00_01_02_03() = _pi_mm_sub_ps(m_00_01_02_03(), M.m_00_01_02_03());	m04() = m04() - M.m04();	m05() = m05() - M.m05();
		m_10_11_12_13() = _pi_mm_sub_ps(m_10_11_12_13(), M.m_10_11_12_13());	m14() = m14() - M.m14();	m15() = m15() - M.m15();
		m_20_21_22_23() = _pi_mm_sub_ps(m_20_21_22_23(), M.m_20_21_22_23());	m24() = m24() - M.m24();	m25() = m25() - M.m25();
		m_30_31_32_33() = _pi_mm_sub_ps(m_30_31_32_33(), M.m_30_31_32_33());	m34() = m34() - M.m34();	m35() = m35() - M.m35();
		m_40_41_42_43() = _pi_mm_sub_ps(m_40_41_42_43(), M.m_40_41_42_43());	m44() = m44() - M.m44();	m45() = m45() - M.m45();
		m_50_51_52_53() = _pi_mm_sub_ps(m_50_51_52_53(), M.m_50_51_52_53());	m54() = m54() - M.m54();	m55() = m55() - M.m55();
	}

	inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix6x6f)); }
	inline void MakeIdentity() { MakeZero(); m00() = m11() = m22() = m33() = m44() = m55() = 1.0f; }

	inline void SetLowerFromUpper()
	{
		m10() = m01();
		m20() = m02();	m21() = m12();
		m30() = m03();	m31() = m13();	m32() = m23();
		m40() = m04();	m41() = m14();	m42() = m24();	m43() = m34();
		m50() = m05();	m51() = m15();	m52() = m25();	m53() = m35();	m54() = m45();
	}

	static inline void AddATBToSymmetric(const AlignedMatrix2x6f &A, const AlignedMatrix2x6f &B, AlignedMatrix6x6f &ATB, const bool last)
	{
		ATB.m_00_01_02_03() = _pi_mm_add_ps(_pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(A.m00()), B.m_00_01_02_03()), 
													_pi_mm_mul_ps(_pi_mm_set1_ps(A.m10()), B.m_10_11_12_13())), ATB.m_00_01_02_03());
		ATB.m_10_11_12_13() = _pi_mm_add_ps(_pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(A.m01()), B.m_00_01_02_03()), 
													_pi_mm_mul_ps(_pi_mm_set1_ps(A.m11()), B.m_10_11_12_13())), ATB.m_10_11_12_13());
		ATB.m22() = A.m02() * B.m02() + A.m12() * B.m12() + ATB.m22();
		ATB.m23() = A.m02() * B.m03() + A.m12() * B.m13() + ATB.m23();
		ATB.m33() = A.m03() * B.m03() + A.m13() * B.m13() + ATB.m33();
		ATB.m_40_41_42_43() = _pi_mm_add_ps(_pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(B.m04()), A.m_00_01_02_03()), 
													_pi_mm_mul_ps(_pi_mm_set1_ps(B.m14()), A.m_10_11_12_13())), ATB.m_40_41_42_43());
		ATB.m_50_51_52_53() = _pi_mm_add_ps(_pi_mm_add_ps(_pi_mm_mul_ps(_pi_mm_set1_ps(B.m05()), A.m_00_01_02_03()), 
													_pi_mm_mul_ps(_pi_mm_set1_ps(B.m15()), A.m_10_11_12_13())), ATB.m_50_51_52_53());
		ATB.m44() = A.m04() * B.m04() + A.m14() * B.m14() + ATB.m44();
		ATB.m45() = A.m04() * B.m05() + A.m14() * B.m15() + ATB.m45();
		ATB.m55() = A.m05() * B.m05() + A.m15() * B.m15() + ATB.m55();
		if(last)
		{
																					ATB.m04() = ATB.m40();	ATB.m05() = ATB.m50();
																					ATB.m14() = ATB.m41();	ATB.m15() = ATB.m51();
			ATB.m20() = ATB.m02();	ATB.m21() = ATB.m12();							ATB.m24() = ATB.m42();	ATB.m25() = ATB.m52();
			ATB.m30() = ATB.m03();	ATB.m31() = ATB.m13();	ATB.m32() = ATB.m23();	ATB.m34() = ATB.m43();	ATB.m35() = ATB.m53();
		}
	}

	static inline bool SolveLDL(AlignedMatrix6x6f &A, AlignedVector6f &b)
	{
		float* _A[6] = {&A.m00(), &A.m10(), &A.m20(), &A.m30(), &A.m40(), &A.m50()};
		return LS::SolveLDL<float, 6>(_A, b);
	}

	inline void Print(const bool e = false) const
	{
		if(e)
		{
			UT::Print("%e %e %e %e %e %e", m00(), m01(), m02(), m03(), m04(), m05());
			UT::Print("%e %e %e %e %e %e", m10(), m11(), m12(), m13(), m14(), m15());
			UT::Print("%e %e %e %e %e %e", m20(), m21(), m22(), m23(), m24(), m25());
			UT::Print("%e %e %e %e %e %e", m30(), m31(), m32(), m33(), m34(), m35());
			UT::Print("%e %e %e %e %e %e", m40(), m41(), m42(), m43(), m44(), m45());
			UT::Print("%e %e %e %e %e %e", m50(), m51(), m52(), m53(), m54(), m55());
		}
		else
		{
			UT::Print("%f %f %f %f %f %f", m00(), m01(), m02(), m03(), m04(), m05());
			UT::Print("%f %f %f %f %f %f", m10(), m11(), m12(), m13(), m14(), m15());
			UT::Print("%f %f %f %f %f %f", m20(), m21(), m22(), m23(), m24(), m25());
			UT::Print("%f %f %f %f %f %f", m30(), m31(), m32(), m33(), m34(), m35());
			UT::Print("%f %f %f %f %f %f", m40(), m41(), m42(), m43(), m44(), m45());
			UT::Print("%f %f %f %f %f %f", m50(), m51(), m52(), m53(), m54(), m55());
		}
	}
	inline bool AssertEqual(const AlignedMatrix6x6f &M, const int verbose = 1) const
	{
		if(UT::VectorAssertEqual(&m00(), &M.m00(), 6, 0) && UT::VectorAssertEqual(&m10(), &M.m10(), 6, 0)
		&& UT::VectorAssertEqual(&m20(), &M.m20(), 6, 0) && UT::VectorAssertEqual(&m30(), &M.m30(), 6, 0)
		&& UT::VectorAssertEqual(&m40(), &M.m40(), 6, 0) && UT::VectorAssertEqual(&m50(), &M.m50(), 6, 0))
			return true;
		if(verbose)
		{
			UT::PrintSeparator();
			Print(verbose > 1);
			UT::PrintSeparator();
			M.Print(verbose > 1);
		}
		return false;
	}

protected:

	_pi__m128 m_data[12];
};
#endif

class AlignedMatrix6x6f
{
  public:
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline const float *operator[](const int i) const { return m_data[i]; }
    inline float *operator[](const int i) { return m_data[i]; }
    inline void operator+=(const AlignedMatrix6x6f &A)
    {
        m_data4[0] = _pi_mm_add_ps(A.m_data4[0], m_data4[0]);
        m_data4[1] = _pi_mm_add_ps(A.m_data4[1], m_data4[1]);
        m_data4[2] = _pi_mm_add_ps(A.m_data4[2], m_data4[2]);
        m_data4[3] = _pi_mm_add_ps(A.m_data4[3], m_data4[3]);
        m_data4[4] = _pi_mm_add_ps(A.m_data4[4], m_data4[4]);
        m_data4[5] = _pi_mm_add_ps(A.m_data4[5], m_data4[5]);
        m_data4[6] = _pi_mm_add_ps(A.m_data4[6], m_data4[6]);
        m_data4[7] = _pi_mm_add_ps(A.m_data4[7], m_data4[7]);
        m_data4[8] = _pi_mm_add_ps(A.m_data4[8], m_data4[8]);
    }
    inline void operator-=(const AlignedMatrix6x6f &A)
    {
        m_data4[0] = _pi_mm_sub_ps(m_data4[0], A.m_data4[0]);
        m_data4[1] = _pi_mm_sub_ps(m_data4[1], A.m_data4[1]);
        m_data4[2] = _pi_mm_sub_ps(m_data4[2], A.m_data4[2]);
        m_data4[3] = _pi_mm_sub_ps(m_data4[3], A.m_data4[3]);
        m_data4[4] = _pi_mm_sub_ps(m_data4[4], A.m_data4[4]);
        m_data4[5] = _pi_mm_sub_ps(m_data4[5], A.m_data4[5]);
        m_data4[6] = _pi_mm_sub_ps(m_data4[6], A.m_data4[6]);
        m_data4[7] = _pi_mm_sub_ps(m_data4[7], A.m_data4[7]);
        m_data4[8] = _pi_mm_sub_ps(m_data4[8], A.m_data4[8]);
    }
    inline AlignedMatrix6x6f operator-(const AlignedMatrix6x6f &A) const
    {
        AlignedMatrix6x6f AmB;
        AmB.m_data4[0] = _pi_mm_sub_ps(m_data4[0], A.m_data4[0]);
        AmB.m_data4[1] = _pi_mm_sub_ps(m_data4[1], A.m_data4[1]);
        AmB.m_data4[2] = _pi_mm_sub_ps(m_data4[2], A.m_data4[2]);
        AmB.m_data4[3] = _pi_mm_sub_ps(m_data4[3], A.m_data4[3]);
        AmB.m_data4[4] = _pi_mm_sub_ps(m_data4[4], A.m_data4[4]);
        AmB.m_data4[5] = _pi_mm_sub_ps(m_data4[5], A.m_data4[5]);
        AmB.m_data4[6] = _pi_mm_sub_ps(m_data4[6], A.m_data4[6]);
        AmB.m_data4[7] = _pi_mm_sub_ps(m_data4[7], A.m_data4[7]);
        AmB.m_data4[8] = _pi_mm_sub_ps(m_data4[8], A.m_data4[8]);
        return AmB;
    }
    inline void operator*=(const _pi__m128 &s) { Scale(s); }
    inline AlignedMatrix6x6f operator*(const _pi__m128 &s) const
    {
        AlignedMatrix6x6f A;
        GetScaled(s, A);
        return A;
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedMatrix6x6f)); }
    inline void MakeIdentity()
    {
        MakeZero();
        m_data[0][0] = m_data[1][1] = m_data[2][2] = m_data[3][3] =
            m_data[4][4] = m_data[5][5] = 1.0f;
    }

    inline void IncreaseDiagonal(const float d012, const float d345)
    {
        m_data[0][0] = d012 + m_data[0][0];
        m_data[1][1] = d012 + m_data[1][1];
        m_data[2][2] = d012 + m_data[2][2];
        m_data[3][3] = d345 + m_data[3][3];
        m_data[4][4] = d345 + m_data[4][4];
        m_data[5][5] = d345 + m_data[5][5];
    }

    inline void SetLowerFromUpper()
    {
        m_data[1][0] = m_data[0][1];
        m_data[2][0] = m_data[0][2];
        m_data[2][1] = m_data[1][2];
        m_data[3][0] = m_data[0][3];
        m_data[3][1] = m_data[1][3];
        m_data[3][2] = m_data[2][3];
        m_data[4][0] = m_data[0][4];
        m_data[4][1] = m_data[1][4];
        m_data[4][2] = m_data[2][4];
        m_data[4][3] = m_data[3][4];
        m_data[5][0] = m_data[0][5];
        m_data[5][1] = m_data[1][5];
        m_data[5][2] = m_data[2][5];
        m_data[5][3] = m_data[3][5];
        m_data[5][4] = m_data[4][5];
    }
    inline void GetTranspose(AlignedMatrix6x6f &M) const
    {
        M.m_data[0][0] = m_data[0][0];
        M.m_data[0][1] = m_data[1][0];
        M.m_data[0][2] = m_data[2][0];
        M.m_data[0][3] = m_data[3][0];
        M.m_data[0][4] = m_data[4][0];
        M.m_data[0][5] = m_data[5][0];
        M.m_data[1][0] = m_data[0][1];
        M.m_data[1][1] = m_data[1][1];
        M.m_data[1][2] = m_data[2][1];
        M.m_data[1][3] = m_data[3][1];
        M.m_data[1][4] = m_data[4][1];
        M.m_data[1][5] = m_data[5][1];
        M.m_data[2][0] = m_data[0][2];
        M.m_data[2][1] = m_data[1][2];
        M.m_data[2][2] = m_data[2][2];
        M.m_data[2][3] = m_data[3][2];
        M.m_data[2][4] = m_data[4][2];
        M.m_data[2][5] = m_data[5][2];
        M.m_data[3][0] = m_data[0][3];
        M.m_data[3][1] = m_data[1][3];
        M.m_data[3][2] = m_data[2][3];
        M.m_data[3][3] = m_data[3][3];
        M.m_data[3][4] = m_data[4][3];
        M.m_data[3][5] = m_data[5][3];
        M.m_data[4][0] = m_data[0][4];
        M.m_data[4][1] = m_data[1][4];
        M.m_data[4][2] = m_data[2][4];
        M.m_data[4][3] = m_data[3][4];
        M.m_data[4][4] = m_data[4][4];
        M.m_data[4][5] = m_data[5][4];
        M.m_data[5][0] = m_data[0][5];
        M.m_data[5][1] = m_data[1][5];
        M.m_data[5][2] = m_data[2][5];
        M.m_data[5][3] = m_data[3][5];
        M.m_data[5][4] = m_data[4][5];
        M.m_data[5][5] = m_data[5][5];
    }

    template <class MATRIX>
    inline void GetBlock(const int i, const int j, MATRIX &M) const;

    inline void Scale(const _pi__m128 &s)
    {
        m_data4[0] = _pi_mm_mul_ps(s, m_data4[0]);
        m_data4[1] = _pi_mm_mul_ps(s, m_data4[1]);
        m_data4[2] = _pi_mm_mul_ps(s, m_data4[2]);
        m_data4[3] = _pi_mm_mul_ps(s, m_data4[3]);
        m_data4[4] = _pi_mm_mul_ps(s, m_data4[4]);
        m_data4[5] = _pi_mm_mul_ps(s, m_data4[5]);
        m_data4[6] = _pi_mm_mul_ps(s, m_data4[6]);
        m_data4[7] = _pi_mm_mul_ps(s, m_data4[7]);
        m_data4[8] = _pi_mm_mul_ps(s, m_data4[8]);
    }
    inline void GetScaled(const _pi__m128 &s, AlignedMatrix6x6f &A) const
    {
        A.m_data4[0] = _pi_mm_mul_ps(s, m_data4[0]);
        A.m_data4[1] = _pi_mm_mul_ps(s, m_data4[1]);
        A.m_data4[2] = _pi_mm_mul_ps(s, m_data4[2]);
        A.m_data4[3] = _pi_mm_mul_ps(s, m_data4[3]);
        A.m_data4[4] = _pi_mm_mul_ps(s, m_data4[4]);
        A.m_data4[5] = _pi_mm_mul_ps(s, m_data4[5]);
        A.m_data4[6] = _pi_mm_mul_ps(s, m_data4[6]);
        A.m_data4[7] = _pi_mm_mul_ps(s, m_data4[7]);
        A.m_data4[8] = _pi_mm_mul_ps(s, m_data4[8]);
    }

    inline bool InverseLDL() { return InverseLDL(*this); }
    static inline bool InverseLDL(AlignedMatrix6x6f &A)
    {
        float *_A[6] = {A[0], A[1], A[2], A[3], A[4], A[5]};
        return LS::InverseLDL<float, 6>(_A);
    }
    inline bool GetInverseLDL(AlignedMatrix6x6f &A) const
    {
        A = *this;
        return A.InverseLDL();
    }
    static inline bool SolveLDL(AlignedMatrix6x6f &A, AlignedVector6f &b)
    {
        float *_A[6] = {A[0], A[1], A[2], A[3], A[4], A[5]};
        return LS::SolveLDL<float, 6>(_A, b);
    }

    inline void Print(const bool e = false) const
    {
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                if (e)
                    UT::Print("%e ", m_data[i][j]);
                else
                    UT::Print("%f ", m_data[i][j]);
            }
            UT::Print("\n");
        }
    }
    inline bool AssertEqual(const AlignedMatrix6x6f &M, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(&m_data[0][0], &M[0][0], 36, 0, eps))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
            const AlignedMatrix6x6f E = *this - M;
            UT::PrintSeparator();
            E.Print(verbose > 1);
        }
        return false;
    }

    static inline void abT(const float *a, const ProductVector6f &b,
                           AlignedMatrix6x6f &abT)
    {
        _pi__m128 a0, a1;
        a0 = _pi_mm_set1_ps(a[0]);
        a1 = _pi_mm_set1_ps(a[1]);
        abT.m_data4[0] = _pi_mm_mul_ps(a0, b.v0123());
        abT.m_data4[1] = _pi_mm_mul_ps(_pi_mm_movelh_ps(a0, a1), b.v4501());
        abT.m_data4[2] = _pi_mm_mul_ps(a1, b.v2345());
        a0 = _pi_mm_set1_ps(a[2]);
        a1 = _pi_mm_set1_ps(a[3]);
        abT.m_data4[3] = _pi_mm_mul_ps(a0, b.v0123());
        abT.m_data4[4] = _pi_mm_mul_ps(_pi_mm_movelh_ps(a0, a1), b.v4501());
        abT.m_data4[5] = _pi_mm_mul_ps(a1, b.v2345());
        a0 = _pi_mm_set1_ps(a[4]);
        a1 = _pi_mm_set1_ps(a[5]);
        abT.m_data4[6] = _pi_mm_mul_ps(a0, b.v0123());
        abT.m_data4[7] = _pi_mm_mul_ps(_pi_mm_movelh_ps(a0, a1), b.v4501());
        abT.m_data4[8] = _pi_mm_mul_ps(a1, b.v2345());
    }

    static inline void Ab(const AlignedMatrix6x6f &A, const ProductVector6f &b,
                          Vector6f &Ab)
    {
        _pi__m128 t;
        t = _pi_mm_mul_ps(A.m_data4[1], b.v4501());
        Ab.v0() = t.m128_f32[0] + t.m128_f32[1] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[0], b.v0123()));
        Ab.v1() = t.m128_f32[2] + t.m128_f32[3] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[2], b.v2345()));
        t = _pi_mm_mul_ps(A.m_data4[4], b.v4501());
        Ab.v2() = t.m128_f32[0] + t.m128_f32[1] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[3], b.v0123()));
        Ab.v3() = t.m128_f32[2] + t.m128_f32[3] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[5], b.v2345()));
        t = _pi_mm_mul_ps(A.m_data4[7], b.v4501());
        Ab.v4() = t.m128_f32[0] + t.m128_f32[1] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[6], b.v0123()));
        Ab.v5() = t.m128_f32[2] + t.m128_f32[3] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[8], b.v2345()));
    }
    static inline void AddAbTo(const AlignedMatrix6x6f &A,
                               const ProductVector6f &b, Vector6f &Ab)
    {
        _pi__m128 t;
        t = _pi_mm_mul_ps(A.m_data4[1], b.v4501());
        Ab.v0() = t.m128_f32[0] + t.m128_f32[1] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[0], b.v0123())) + Ab.v0();
        Ab.v1() = t.m128_f32[2] + t.m128_f32[3] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[2], b.v2345())) + Ab.v1();
        t = _pi_mm_mul_ps(A.m_data4[4], b.v4501());
        Ab.v2() = t.m128_f32[0] + t.m128_f32[1] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[3], b.v0123())) + Ab.v2();
        Ab.v3() = t.m128_f32[2] + t.m128_f32[3] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[5], b.v2345())) + Ab.v3();
        t = _pi_mm_mul_ps(A.m_data4[7], b.v4501());
        Ab.v4() = t.m128_f32[0] + t.m128_f32[1] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[6], b.v0123())) + Ab.v4();
        Ab.v5() = t.m128_f32[2] + t.m128_f32[3] +
                  SSE::Sum(_pi_mm_mul_ps(A.m_data4[8], b.v2345())) + Ab.v5();
    }

    static inline void ABTTo00(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               AlignedMatrix6x6f &ABT)
    {
        ABT[0][0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT[0][1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT[0][2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT[1][0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_00_01_02_r0()));
        ABT[1][1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT[1][2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT[2][0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_00_01_02_r0()));
        ABT[2][1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_10_11_12_r1()));
        ABT[2][2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }
    static inline void ABTTo03(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               AlignedMatrix6x6f &ABT)
    {
        ABT[0][3] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT[0][4] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT[0][5] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT[1][3] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_00_01_02_r0()));
        ABT[1][4] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT[1][5] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT[2][3] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_00_01_02_r0()));
        ABT[2][4] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_10_11_12_r1()));
        ABT[2][5] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }
    static inline void ABTTo30(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               AlignedMatrix6x6f &ABT)
    {
        ABT[3][0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT[3][1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT[3][2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT[4][0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_00_01_02_r0()));
        ABT[4][1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT[4][2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT[5][0] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_00_01_02_r0()));
        ABT[5][1] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_10_11_12_r1()));
        ABT[5][2] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }
    static inline void ABTTo33(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               AlignedMatrix6x6f &ABT)
    {
        ABT[3][3] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT[3][4] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT[3][5] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT[4][3] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_00_01_02_r0()));
        ABT[4][4] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT[4][5] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT[5][3] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_00_01_02_r0()));
        ABT[5][4] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_10_11_12_r1()));
        ABT[5][5] =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }

  public:
    union {
        struct {
            float m_data[6][6];
        };
        struct {
            _pi__m128 m_data4[9];
        };
    };
};
template <>
inline void
AlignedMatrix6x6f::GetBlock<AlignedMatrix3x3f>(const int i, const int j,
                                               AlignedMatrix3x3f &M) const
{
    memcpy(&M.m00(), m_data[i] + j, 12);
    memcpy(&M.m10(), m_data[i + 1] + j, 12);
    memcpy(&M.m20(), m_data[i + 2] + j, 12);
}

template <typename TYPE> class SymmetricMatrix6x6
{
  public:
    inline SymmetricMatrix6x6<TYPE>() = default;
    inline SymmetricMatrix6x6<TYPE>(const TYPE *M) { Set(M); }
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
    inline const TYPE &m11() const { return m_data[6]; }
    inline TYPE &m11() { return m_data[6]; }
    inline const TYPE &m12() const { return m_data[7]; }
    inline TYPE &m12() { return m_data[7]; }
    inline const TYPE &m13() const { return m_data[8]; }
    inline TYPE &m13() { return m_data[8]; }
    inline const TYPE &m14() const { return m_data[9]; }
    inline TYPE &m14() { return m_data[9]; }
    inline const TYPE &m15() const { return m_data[10]; }
    inline TYPE &m15() { return m_data[10]; }
    inline const TYPE &m22() const { return m_data[11]; }
    inline TYPE &m22() { return m_data[11]; }
    inline const TYPE &m23() const { return m_data[12]; }
    inline TYPE &m23() { return m_data[12]; }
    inline const TYPE &m24() const { return m_data[13]; }
    inline TYPE &m24() { return m_data[13]; }
    inline const TYPE &m25() const { return m_data[14]; }
    inline TYPE &m25() { return m_data[14]; }
    inline const TYPE &m33() const { return m_data[15]; }
    inline TYPE &m33() { return m_data[15]; }
    inline const TYPE &m34() const { return m_data[16]; }
    inline TYPE &m34() { return m_data[16]; }
    inline const TYPE &m35() const { return m_data[17]; }
    inline TYPE &m35() { return m_data[17]; }
    inline const TYPE &m44() const { return m_data[18]; }
    inline TYPE &m44() { return m_data[18]; }
    inline const TYPE &m45() const { return m_data[19]; }
    inline TYPE &m45() { return m_data[19]; }
    inline const TYPE &m55() const { return m_data[20]; }
    inline TYPE &m55() { return m_data[20]; }
    inline operator const TYPE *() const { return (const TYPE *)this; }
    inline operator TYPE *() { return (TYPE *)this; }
    inline void operator*=(const _pi__m128 &s);

    inline void Set(const TYPE *M)
    {
        memcpy(this, M, sizeof(SymmetricMatrix6x6<TYPE>));
    }

    inline AlignedMatrix6x6f GetAlignedMatrix6x6f() const
    {
        AlignedMatrix6x6f M;
        GetAlignedMatrix6x6f(M);
        return M;
    }
    inline void GetAlignedMatrix6x6f(AlignedMatrix6x6f &M) const;

    inline void MakeZero()
    {
        memset(this, 0, sizeof(SymmetricMatrix6x6<TYPE>));
    }

    inline void Print(const bool e = false) const
    {
        if (e) {
            UT::Print("%e %e %e %e %e %e\n", m00(), m01(), m02(), m03(), m04(),
                      m05());
            UT::Print("%e %e %e %e %e %e\n", m01(), m11(), m12(), m13(), m14(),
                      m15());
            UT::Print("%e %e %e %e %e %e\n", m02(), m12(), m22(), m23(), m24(),
                      m25());
            UT::Print("%e %e %e %e %e %e\n", m03(), m13(), m23(), m33(), m34(),
                      m35());
            UT::Print("%e %e %e %e %e %e\n", m04(), m14(), m24(), m34(), m44(),
                      m45());
            UT::Print("%e %e %e %e %e %e\n", m05(), m15(), m25(), m35(), m45(),
                      m55());
        } else {
            UT::Print("%f %f %f %f %f %f\n", m00(), m01(), m02(), m03(), m04(),
                      m05());
            UT::Print("%f %f %f %f %f %f\n", m01(), m11(), m12(), m13(), m14(),
                      m15());
            UT::Print("%f %f %f %f %f %f\n", m02(), m12(), m22(), m23(), m24(),
                      m25());
            UT::Print("%f %f %f %f %f %f\n", m03(), m13(), m23(), m33(), m34(),
                      m35());
            UT::Print("%f %f %f %f %f %f\n", m04(), m14(), m24(), m34(), m44(),
                      m45());
            UT::Print("%f %f %f %f %f %f\n", m05(), m15(), m25(), m35(), m45(),
                      m55());
        }
    }

    inline bool AssertEqual(const SymmetricMatrix6x6<TYPE> &M,
                            const int verbose = 1, const TYPE eps = 0) const
    {
        if (UT::VectorAssertEqual(&m00(), &M.m00(), 21, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            M.Print(verbose > 1);
        }
        return false;
    }

    static inline void ABTTo00(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               SymmetricMatrix6x6<TYPE> &ABT)
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

    static inline void ABTTo03(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               SymmetricMatrix6x6<TYPE> &ABT)
    {
        ABT.m03() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT.m04() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT.m05() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT.m13() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_00_01_02_r0()));
        ABT.m14() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT.m15() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT.m23() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_00_01_02_r0()));
        ABT.m24() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_10_11_12_r1()));
        ABT.m25() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }
    static inline void ABTTo33(const AlignedMatrix3x3f &A,
                               const AlignedMatrix3x3f &B,
                               SymmetricMatrix6x6<TYPE> &ABT)
    {
        ABT.m33() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_00_01_02_r0()));
        ABT.m34() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_10_11_12_r1()));
        ABT.m35() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), B.m_20_21_22_r2()));
        ABT.m44() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_10_11_12_r1()));
        ABT.m45() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), B.m_20_21_22_r2()));
        ABT.m55() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), B.m_20_21_22_r2()));
    }

    static inline void AbTo0(const AlignedMatrix3x3f &A,
                             const AlignedVector3f &b, Vector6<TYPE> &Ab)
    {
        Ab.v0() = SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), b.v012r()));
        Ab.v1() = SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), b.v012r()));
        Ab.v2() = SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), b.v012r()));
    }
    static inline void AddAbTo0(const AlignedMatrix3x3f &A,
                                const AlignedVector3f &b, Vector6<TYPE> &Ab)
    {
        Ab.v0() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), b.v012r())) + Ab.v0();
        Ab.v1() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), b.v012r())) + Ab.v1();
        Ab.v2() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), b.v012r())) + Ab.v2();
    }
    static inline void AbTo3(const AlignedMatrix3x3f &A,
                             const AlignedVector3f &b, Vector6<TYPE> &Ab)
    {
        Ab.v3() = SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), b.v012r()));
        Ab.v4() = SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), b.v012r()));
        Ab.v5() = SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), b.v012r()));
    }
    static inline void AddAbTo3(const AlignedMatrix3x3f &A,
                                const AlignedVector3f &b, Vector6<TYPE> &Ab)
    {
        Ab.v3() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_00_01_02_r0(), b.v012r())) + Ab.v3();
        Ab.v4() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_10_11_12_r1(), b.v012r())) + Ab.v4();
        Ab.v5() =
            SSE::Sum012(_pi_mm_mul_ps(A.m_20_21_22_r2(), b.v012r())) + Ab.v5();
    }

  protected:
    TYPE m_data[21];
};

typedef SymmetricMatrix6x6<float> SymmetricMatrix6x6f;
typedef SymmetricMatrix6x6<double> SymmetricMatrix6x6d;

template <> inline void SymmetricMatrix6x6f::operator*=(const _pi__m128 &s)
{
    _pi__m128 *M = (_pi__m128 *)m_data;
    for (int i = 0; i < 5; ++i, ++M) *M = _pi_mm_mul_ps(*M, s);
    m_data[20] *= s.m128_f32[0];
}

template <>
inline void
SymmetricMatrix6x6f::GetAlignedMatrix6x6f(AlignedMatrix6x6f &M) const
{
    memcpy(M[0], &m00(), 24);
    memcpy(M[1] + 1, &m11(), 20);
    memcpy(M[2] + 2, &m22(), 16);
    memcpy(M[3] + 3, &m33(), 12);
    memcpy(M[4] + 4, &m44(), 8);
    M[5][5] = m55();
    M.SetLowerFromUpper();
}
}

#ifdef CFG_DEBUG_EIGEN
class EigenMatrix6x6f : public Eigen::Matrix<float, 6, 6>
{
  public:
    inline EigenMatrix6x6f() = default;
    inline EigenMatrix6x6f(const Eigen::Matrix<float, 6, 6> &e_M)
        : Eigen::Matrix<float, 6, 6>(e_M)
    {
    }
    inline EigenMatrix6x6f(const LA::AlignedMatrix6x6f &M)
        : Eigen::Matrix<float, 6, 6>()
    {
        Eigen::Matrix<float, 6, 6> &e_M = *this;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) e_M(i, j) = M[i][j];
    }
    inline EigenMatrix6x6f(const LA::SymmetricMatrix6x6f &M)
        : Eigen::Matrix<float, 6, 6>()
    {
        Eigen::Matrix<float, 6, 6> &e_M = *this;
        const float *_M = M;
        for (int i = 0, k = 0; i < 6; ++i)
            for (int j = i; j < 6; ++j, ++k) e_M(i, j) = e_M(j, i) = _M[k];
    }
    inline EigenMatrix6x6f(const EigenMatrix3x3f &e_M00,
                           const EigenMatrix3x3f &e_M01,
                           const EigenMatrix3x3f &e_M10,
                           const EigenMatrix3x3f &e_M11)
    {
        block<3, 3>(0, 0) = e_M00;
        block<3, 3>(0, 3) = e_M01;
        block<3, 3>(3, 0) = e_M10;
        block<3, 3>(3, 3) = e_M11;
    }
    inline EigenMatrix6x6f(const LA::AlignedMatrix3x3f &M00,
                           const LA::AlignedMatrix3x3f &M01,
                           const LA::AlignedMatrix3x3f &M10,
                           const LA::AlignedMatrix3x3f &M11)
    {
        block<3, 3>(0, 0) = EigenMatrix3x3f(M00);
        block<3, 3>(0, 3) = EigenMatrix3x3f(M01);
        block<3, 3>(3, 0) = EigenMatrix3x3f(M10);
        block<3, 3>(3, 3) = EigenMatrix3x3f(M11);
    }
    inline void operator=(const Eigen::Matrix<float, 6, 6> &e_M)
    {
        *((Eigen::Matrix<float, 6, 6> *)this) = e_M;
    }
    inline LA::SymmetricMatrix6x6f GetSymmetricMatrix6x6f() const
    {
        LA::SymmetricMatrix6x6f M;
        float *_M = &M.m00();
        const Eigen::Matrix<float, 6, 6> &e_M = *this;
        for (int i = 0; i < 6; ++i)
            for (int j = i; j < 6; ++j) *_M++ = e_M(i, j);
        return M;
    }
    inline LA::AlignedMatrix6x6f GetAlignedMatrix6x6f() const
    {
        LA::AlignedMatrix6x6f M;
        const Eigen::Matrix<float, 6, 6> &e_M = *this;
        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j) M[i][j] = e_M(i, j);
        return M;
    }
    inline void Print(const bool e = false) const
    {
        GetAlignedMatrix6x6f().Print(e);
    }
    inline bool AssertEqual(const LA::AlignedMatrix6x6f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix6x6f().AssertEqual(M, verbose);
    }
    inline bool AssertEqual(const LA::SymmetricMatrix6x6f &M,
                            const int verbose = 1) const
    {
        return GetAlignedMatrix6x6f().AssertEqual(M.GetAlignedMatrix6x6f(),
                                                  verbose);
    }
    inline bool AssertEqual(const EigenMatrix6x6f &e_M,
                            const int verbose = 1) const
    {
        return AssertEqual(e_M.GetAlignedMatrix6x6f(), verbose);
    }
};
#endif
#endif
