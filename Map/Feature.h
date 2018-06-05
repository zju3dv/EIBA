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

#ifndef _FEATURE_H_
#define _FEATURE_H_

#include "stdafx.h"

#include "Camera.h"
#include "Depth.h"
#include "Matrix2x6.h"
#include "Matrix3x6.h"

#ifdef CFG_TUNE_PARAMETERS
extern float FTR_PATCH_MIN_WARP_MOVEMENT;
extern float FTR_PATCH_MAX_INTENSITY_ERROR_ZMSSD;
extern float FTR_PATCH_MAX_INTENSITY_ERROR_ZMSSD_RATIO;
extern float FTR_PATCH_MAX_INTENSITY_ERROR_KLT;
extern float FTR_PATCH_MAX_INTENSITY_OFFSET;
extern float FTR_PATCH_VARIANCE_INTENSITY;
extern float FTR_PATCH_EPSILON_LOCATION;
extern float FTR_PATCH_KLT_MIN_SCORE;
#include "Configurator.h"
extern void LOAD_PARAMETERS_FEATURE(const Configurator &cfgor);
#else
#define FTR_PATCH_MIN_WARP_MOVEMENT 0.5f
#define FTR_PATCH_MAX_INTENSITY_ERROR_ZMSSD 20.0f
#define FTR_PATCH_MAX_INTENSITY_ERROR_ZMSSD_RATIO 0.49f
#define FTR_PATCH_MAX_INTENSITY_ERROR_KLT 10.0f
//#define FTR_PATCH_MAX_INTENSITY_ERROR_KLT			20.0f
//#define FTR_PATCH_MAX_INTENSITY_OFFSET			10.0f
#define FTR_PATCH_MAX_INTENSITY_OFFSET 50.0f
//#define FTR_PATCH_VARIANCE_INTENSITY				9.0f	// 3^2
#define FTR_PATCH_VARIANCE_INTENSITY 100.0f // 10^2
//#define FTR_PATCH_EPSILON_LOCATION				0.01f
#define FTR_PATCH_EPSILON_LOCATION 1.0f
//#define FTR_PATCH_KLT_MIN_SCORE					10.0f
#define FTR_PATCH_KLT_MIN_SCORE 50.0f
#endif

#define FTR_PATCH_SIZE 8
#define FTR_PATCH_SIZE_HALF 4
#define FTR_PATCH_SIZE_PLUS_TWO 10
#define FTR_PATCH_SIZE_PLUS_TWO_HALF 5
#define FTR_PATCH_PIXELS 64

#define FTR_PATCH_FLAG_DEFAULT 0
#define FTR_PATCH_FLAG_ZMSSD 1
#define FTR_PATCH_FLAG_KLT_EXTRACTION 2
#define FTR_PATCH_FLAG_KLT_TRACKING 4
#define FTR_PATCH_FLAG_UNSTABLE 8
#define FTR_PATCH_FLAG_INVALID 16

namespace FTR
{

// source feature (2D points with inv depth)
class Source
{
  public:
    inline bool operator==(const Source &x) const
    {
        return m_x == x.m_x
#ifdef CFG_DEPTH_MAP
               && m_d == x.m_d
#endif
            ;
    }

  public:
    Point2D m_x;
#ifdef CFG_DEPTH_MAP
    float m_d;
#endif
};

class Measurement
{
  public:
    class Match
    {
      public:
        inline Match() {}
        inline Match(const int iz1, const int iz2) : m_iz1(iz1), m_iz2(iz2) {}
        inline bool operator<(const Match &izm) const
        {
            return m_iz1 < izm.m_iz1;
        }
        inline bool operator==(const Match &izm) const
        {
            return m_iz1 == izm.m_iz1 && m_iz2 == izm.m_iz2;
        }

      public:
        int m_iz1, m_iz2;
    };
    class ESError
    {
      public:
        inline ESError() {}
        inline ESError(const Intrinsic &K, const LA::Vector2f &ex
#ifdef CFG_DEPTH_MAP
                       ,
                       const float ed = FLT_MAX
#endif
                       )
            : m_ex(K.fx() * ex.x(), K.fy() * ex.y())
#ifdef CFG_DEPTH_MAP
              ,
              m_ed(ed)
#endif
        {
        }
        inline void Initialize()
        {
            m_ex.MakeZero();
#ifdef CFG_DEPTH_MAP
            m_ed = FLT_MAX;
#endif
        }
        inline float SquaredLength() const { return m_ex.SquaredLength(); }
        inline void Print(const bool l = true) const
        {
            const float ex = sqrt(m_ex.SquaredLength());
            if (l)
                UT::Print("%f", ex);
            else
                UT::Print("%.2f", ex);
#ifdef CFG_DEPTH_MAP
            if (m_ed != FLT_MAX) {
                if (l)
                    UT::Print(" %f", m_ed);
                else
                    UT::Print(" %.2f", m_ed);
            }
#endif
        }

      public:
        LA::Vector2f m_ex;
#ifdef CFG_DEPTH_MAP
        float m_ed;
#endif
    };
    class ESIndex
    {
      public:
        inline ESIndex() : m_ixFrm(-1), m_ix(-1), m_izFrm(-1), m_iz(-1) {}
        inline ESIndex(const int ixFrm, const int ix, const int izFrm = -1,
                       const int iz = -1)
            : m_ixFrm(ixFrm), m_ix(ix), m_izFrm(izFrm), m_iz(iz)
        {
        }
        inline operator int() const { return m_iz; }
        inline void Print() const
        {
            if (m_ixFrm == -1 || m_ix == -1) return;
            UT::Print(" [%d] %d", m_ixFrm, m_ix);
            if (m_izFrm != -1) UT::Print(" [%d]", m_izFrm);
            if (m_iz != -1) UT::Print(" %d", m_iz);
        }

      public:
        int m_ixFrm, m_ix, m_izFrm, m_iz;
    };
    // error statistics
    class ES : public UT::ES<ESError, ESIndex>
    {
      public:
        inline void Initialize()
        {
            UT::ES<ESError, ESIndex>::Initialize();
            m_SNr = 0;
            m_eMax.Initialize();
        }
        inline void Accumulate(const Intrinsic &K, const LA::Vector2f &ex,
                               const float we2, const ESIndex iz,
                               const bool r = true)
        {
            UT::ES<ESError, ESIndex>::Accumulate(ESError(K, ex), we2, iz);
            if (r) ++m_SNr;
        }
#ifdef CFG_DEPTH_MAP
        inline void Accumulate(const Intrinsic &K, const LA::Vector2f &ex,
                               const float ed, const float we2,
                               const ESIndex idx, const bool r = true)
        {
            // disable accumulate when ex = 0
            if (ex.x() == 0.0f && ex.y() == 0.0f)
            {
                return;
            }
            UT::ES<ESError, ESIndex>::Accumulate(ESError(K, ex, ed), we2, idx);
            if (r) ++m_SNr;
        }
#endif
        inline void Print(const std::string str = "", const bool r = true) const
        {
            // if(!Valid())
            //	return;
            UT::ES<ESError, ESIndex>::Print(str + "ex = ", false, false);
            if (r)
                UT::Print(" (%d / %d = %d%%)", m_SNr, m_SN,
                          m_SN == 0 ? 0
                                    : int(m_SNr * 100 / float(m_SN) + 0.5f));
            UT::Print("\n");
        }

      public:
        int m_SNr;
    };

  public:
    inline Measurement() {}
    inline bool operator==(const Source &x) const
    {
        return m_z == x.m_x
#ifdef CFG_DEPTH_MAP
               && m_d == x.m_d
#endif
            ;
    }
    inline bool operator==(const Measurement &z) const
    {
        return m_ix == z.m_ix && m_z == z.m_z && m_W == z.m_W
#ifdef CFG_DEPTH_MAP
               && m_d == z.m_d
#endif
            ;
    }
    inline bool operator<(const int ix) const { return m_ix < ix; }
    inline bool operator<(const Measurement &z) const { return m_ix < z.m_ix; }
    inline bool Valid() const { return m_ix >= 0; }
    inline bool Invalid() const { return m_ix == -1; }
    inline void Invalidate() { m_ix = -1; }
    inline void GetError(const Rigid3D &T12, const Point2D &x1,
                         const DepthInverseGaussian &d1,
                         LA::Vector2f &ex2) const
    {
        d1.Project(T12, x1, ex2);
        ex2 -= m_z;
    }
    inline void GetErrorJacobian(const Rigid3D &T12, const Point2D &x1,
                                 const DepthInverseGaussian &d1,
                                 LA::Vector2f &ex2, LA::Vector2f &Jx2d1) const
    {
        d1.Project(T12, x1, ex2, Jx2d1);
        ex2 -= m_z;
    }
    inline void GetErrorJacobian(const Rigid3D &T12, const Point2D &x1,
                                 const DepthInverseGaussian &d1,
                                 const Rigid3D &T2, LA::Vector2f &ex2,
                                 LA::AlignedMatrix2x6f &Jx2pr2) const
    {
        float d2;
        d1.Project(T12, x1, ex2, d2);
        const _pi__m128 _d2 = _pi_mm_set1_ps(d2);
        Jx2pr2.m_00_01_02_03() = _pi_mm_mul_ps(
            _d2, _pi_mm_sub_ps(
                     _pi_mm_mul_ps(_pi_mm_set1_ps(ex2.x()), T2.r_20_21_22_x()),
                     T2.r_00_01_02_x()));
        Jx2pr2.m_10_11_12_13() = _pi_mm_mul_ps(
            _d2, _pi_mm_sub_ps(
                     _pi_mm_mul_ps(_pi_mm_set1_ps(ex2.y()), T2.r_20_21_22_x()),
                     T2.r_10_11_12_x()));
        Jx2pr2.m03() = ex2.x() * ex2.y();
        Jx2pr2.m04() = -(ex2.x() * ex2.x() + 1.0f);
        Jx2pr2.m05() = ex2.y();
        Jx2pr2.m13() = ex2.y() * ex2.y() + 1.0f;
        Jx2pr2.m14() = -Jx2pr2.m03();
        Jx2pr2.m15() = -ex2.x();
        ex2 -= m_z;
    }
    inline void GetErrorJacobian(const Rigid3D &T12, const Point2D &x1,
                                 const DepthInverseGaussian &d1,
                                 const Rigid3D &T2, LA::Vector2f &ex2,
                                 LA::Vector2f &Jx2d1,
                                 LA::AlignedMatrix2x6f &Jx2pr1,
                                 LA::AlignedMatrix2x6f &Jx2pr2) const
    {
        float d12, d2;
        d1.Project(T12, x1, ex2, d12, d2, Jx2d1);
        const _pi__m128 _d12 = _pi_mm_set1_ps(d12), _d2 = _pi_mm_set1_ps(d2),
                        _x2 = _pi_mm_set1_ps(ex2.x()),
                        _y2 = _pi_mm_set1_ps(ex2.y());
        Jx2pr1.m_00_01_02_03() = _pi_mm_mul_ps(
            _d2, _pi_mm_sub_ps(T2.r_00_01_02_x(),
                               _pi_mm_mul_ps(_x2, T2.r_20_21_22_x())));
        Jx2pr1.m_10_11_12_13() = _pi_mm_mul_ps(
            _d2, _pi_mm_sub_ps(T2.r_10_11_12_x(),
                               _pi_mm_mul_ps(_y2, T2.r_20_21_22_x())));
        Jx2pr2.m_00_01_02_03() = _pi_mm_mul_ps(
            _d12, _pi_mm_sub_ps(_pi_mm_mul_ps(_x2, T12.r_20_21_22_x()),
                                T12.r_00_01_02_x()));
        Jx2pr2.m_10_11_12_13() = _pi_mm_mul_ps(
            _d12, _pi_mm_sub_ps(_pi_mm_mul_ps(_y2, T12.r_20_21_22_x()),
                                T12.r_10_11_12_x()));
        Jx2pr1.m03() = Jx2pr2.m01() - Jx2pr2.m02() * x1.y();
        Jx2pr1.m04() = Jx2pr2.m02() * x1.x() - Jx2pr2.m00();
        Jx2pr1.m05() = Jx2pr2.m00() * x1.y() - Jx2pr2.m01() * x1.x();
        Jx2pr1.m13() = Jx2pr2.m11() - Jx2pr2.m12() * x1.y();
        Jx2pr1.m14() = Jx2pr2.m12() * x1.x() - Jx2pr2.m10();
        Jx2pr1.m15() = Jx2pr2.m10() * x1.y() - Jx2pr2.m11() * x1.x();
        Jx2pr2.m_00_01_02_03() =
            _pi_mm_sub_ps(_pi_mm_setzero_ps(), Jx2pr1.m_00_01_02_03());
        Jx2pr2.m_10_11_12_13() =
            _pi_mm_sub_ps(_pi_mm_setzero_ps(), Jx2pr1.m_10_11_12_13());
        Jx2pr2.m03() = ex2.x() * ex2.y();
        Jx2pr2.m04() = -(ex2.x() * ex2.x() + 1.0f);
        Jx2pr2.m05() = ex2.y();
        Jx2pr2.m13() = ex2.y() * ex2.y() + 1.0f;
        Jx2pr2.m14() = -Jx2pr2.m03();
        Jx2pr2.m15() = -ex2.x();
        ex2 -= m_z;
    }
#ifdef CFG_DEPTH_MAP
    inline void GetError(const Rigid3D &T12, const Point2D &x1,
                         const DepthInverseGaussian &d1, LA::Vector2f &ex2,
                         float &ed2) const
    {
        d1.Project(T12, x1, ex2, ed2);
        ex2 -= m_z;
        ed2 -= m_d;
    }
    inline void GetErrorJacobian(const Rigid3D &T12, const Point2D &x1,
                                 const DepthInverseGaussian &d1,
                                 LA::Vector2f &ex2, float &ed2,
                                 LA::AlignedVector3f &Jxd2d1) const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(m_d != 0.0f);
#endif
        d1.Project(T12, x1, ex2, ed2, Jxd2d1);
        ex2 -= m_z;
        ed2 -= m_d;
    }
    inline void GetErrorJacobian(const Rigid3D &T12, const Point2D &x1,
                                 const DepthInverseGaussian &d1,
                                 const Rigid3D &T2, LA::Vector2f &ex2,
                                 float &ed2,
                                 LA::AlignedMatrix3x6f &Jxd2pr2) const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(m_d != 0.0f);
#endif
        d1.Project(T12, x1, ex2, ed2);
        const _pi__m128 d2 = _pi_mm_set1_ps(ed2);
        Jxd2pr2.m_00_01_02_03() = _pi_mm_mul_ps(
            d2, _pi_mm_sub_ps(
                    _pi_mm_mul_ps(_pi_mm_set1_ps(ex2.x()), T2.r_20_21_22_x()),
                    T2.r_00_01_02_x()));
        Jxd2pr2.m_10_11_12_13() = _pi_mm_mul_ps(
            d2, _pi_mm_sub_ps(
                    _pi_mm_mul_ps(_pi_mm_set1_ps(ex2.y()), T2.r_20_21_22_x()),
                    T2.r_10_11_12_x()));
        Jxd2pr2.m_20_21_22_23() =
            _pi_mm_mul_ps(_pi_mm_set1_ps(ed2 * ed2), T2.r_20_21_22_x());
        Jxd2pr2.m03() = ex2.x() * ex2.y();
        Jxd2pr2.m04() = -(ex2.x() * ex2.x() + 1.0f);
        Jxd2pr2.m05() = ex2.y();
        Jxd2pr2.m13() = ex2.y() * ex2.y() + 1.0f;
        Jxd2pr2.m14() = -Jxd2pr2.m03();
        Jxd2pr2.m15() = -ex2.x();
        Jxd2pr2.m23() = ex2.y() * ed2;
        Jxd2pr2.m24() = -ex2.x() * ed2;
        Jxd2pr2.m25() = 0.0f;
        ex2 -= m_z;
        ed2 -= m_d;
    }
    inline void GetErrorJacobian(const Rigid3D &T12, const Point2D &x1,
                                 const DepthInverseGaussian &d1,
                                 const Rigid3D &T2, LA::Vector2f &ex2,
                                 float &ed2, LA::AlignedVector3f &Jxd2d1,
                                 LA::AlignedMatrix3x6f &Jxd2pr1,
                                 LA::AlignedMatrix3x6f &Jxd2pr2) const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(m_d != 0.0f);
#endif
        float d12;
        d1.Project(T12, x1, ex2, d12, ed2, Jxd2d1);
        const _pi__m128 _d12 = _pi_mm_set1_ps(d12), _d2 = _pi_mm_set1_ps(ed2),
                        _x2 = _pi_mm_set1_ps(ex2.x()),
                        _y2 = _pi_mm_set1_ps(ex2.y());
        Jxd2pr1.m_00_01_02_03() = _pi_mm_mul_ps(
            _d2, _pi_mm_sub_ps(T2.r_00_01_02_x(),
                               _pi_mm_mul_ps(_x2, T2.r_20_21_22_x())));
        Jxd2pr1.m_10_11_12_13() = _pi_mm_mul_ps(
            _d2, _pi_mm_sub_ps(T2.r_10_11_12_x(),
                               _pi_mm_mul_ps(_y2, T2.r_20_21_22_x())));
        Jxd2pr1.m_20_21_22_23() =
            _pi_mm_mul_ps(_pi_mm_set1_ps(-ed2 * ed2), T2.r_20_21_22_x());
        Jxd2pr2.m_00_01_02_03() = _pi_mm_mul_ps(
            _d12, _pi_mm_sub_ps(_pi_mm_mul_ps(_x2, T12.r_20_21_22_x()),
                                T12.r_00_01_02_x()));
        Jxd2pr2.m_10_11_12_13() = _pi_mm_mul_ps(
            _d12, _pi_mm_sub_ps(_pi_mm_mul_ps(_y2, T12.r_20_21_22_x()),
                                T12.r_10_11_12_x()));
        Jxd2pr2.m_20_21_22_23() =
            _pi_mm_mul_ps(_pi_mm_set1_ps(d12 * ed2), T12.r_20_21_22_x());
        Jxd2pr1.m03() = Jxd2pr2.m01() - Jxd2pr2.m02() * x1.y();
        Jxd2pr1.m04() = Jxd2pr2.m02() * x1.x() - Jxd2pr2.m00();
        Jxd2pr1.m05() = Jxd2pr2.m00() * x1.y() - Jxd2pr2.m01() * x1.x();
        Jxd2pr1.m13() = Jxd2pr2.m11() - Jxd2pr2.m12() * x1.y();
        Jxd2pr1.m14() = Jxd2pr2.m12() * x1.x() - Jxd2pr2.m10();
        Jxd2pr1.m15() = Jxd2pr2.m10() * x1.y() - Jxd2pr2.m11() * x1.x();
        Jxd2pr1.m23() = Jxd2pr2.m21() - Jxd2pr2.m22() * x1.y();
        Jxd2pr1.m24() = Jxd2pr2.m22() * x1.x() - Jxd2pr2.m20();
        Jxd2pr1.m25() = Jxd2pr2.m20() * x1.y() - Jxd2pr2.m21() * x1.x();
        Jxd2pr2.m_00_01_02_03() =
            _pi_mm_sub_ps(_pi_mm_setzero_ps(), Jxd2pr1.m_00_01_02_03());
        Jxd2pr2.m_10_11_12_13() =
            _pi_mm_sub_ps(_pi_mm_setzero_ps(), Jxd2pr1.m_10_11_12_13());
        Jxd2pr2.m_20_21_22_23() =
            _pi_mm_sub_ps(_pi_mm_setzero_ps(), Jxd2pr1.m_20_21_22_23());
        Jxd2pr2.m03() = ex2.x() * ex2.y();
        Jxd2pr2.m04() = -(ex2.x() * ex2.x() + 1.0f);
        Jxd2pr2.m05() = ex2.y();
        Jxd2pr2.m13() = ex2.y() * ex2.y() + 1.0f;
        Jxd2pr2.m14() = -Jxd2pr2.m03();
        Jxd2pr2.m15() = -ex2.x();
        Jxd2pr2.m23() = ex2.y() * ed2;
        Jxd2pr2.m24() = -ex2.x() * ed2;
        Jxd2pr2.m25() = 0.0f;
        ex2 -= m_z;
        ed2 -= m_d;
    }
#endif
  public:
    // index of corresponding source index (local)
    int m_ix;
    // 2D measurement
    Point2D m_z;
    // ??? covariance ???
    LA::SymmetricMatrix2x2f m_W;
#ifdef CFG_DEPTH_MAP
    // inv depth measurement
    float m_d;
#endif
};

class Factor
{
  public:
    class Source
    {
      public:
        Source() {}
        class Depth
        {
          public:
            float m_a, m_b;
        };

      public:
        inline void operator+=(const Source &A)
        {
            m_data[0] = _pi_mm_add_ps(A.m_data[0], m_data[0]);
            m_data[1] = _pi_mm_add_ps(A.m_data[1], m_data[1]);
        }
        inline void operator-=(const Source &A)
        {
            m_data[0] = _pi_mm_sub_ps(m_data[0], A.m_data[0]);
            m_data[1] = _pi_mm_sub_ps(m_data[1], A.m_data[1]);
        }
        inline void operator+=(const Depth &A)
        {
            fe.m_ad = A.m_a + fe.m_ad;
            fe.m_b = A.m_b + fe.m_b;
        }
        inline void operator-=(const Depth &A)
        {
            fe.m_ad = -A.m_a + fe.m_ad;
            fe.m_b = -A.m_b + fe.m_b;
        }
        inline Source operator*(const _pi__m128 &s) const
        {
            Source A;
            GetScaled(s, A);
            return A;
        }
        inline void MakeZero() { memset(this, 0, sizeof(Source)); }
        inline void GetScaled(const _pi__m128 &s, Source &A) const
        {
            A.m_data[0] = _pi_mm_mul_ps(s, m_data[0]);
            A.m_data[1] = _pi_mm_mul_ps(s, m_data[1]);
        }
        inline float BackSubstitute(const LA::ProductVector6f &xc) const
        {
            return fe.m_b + SSE::Sum(_pi_mm_mul_ps(m_data[0], xc.v0123())) +
                   fe.m_ac.v4() * xc.v4() + fe.m_ac.v5() * xc.v5();
        }
        inline void Print(const bool e = false) const
        {
            if (e)
                UT::Print("%e %e\n", fe.m_ad, fe.m_b);
            else
                UT::Print("%f %f\n", fe.m_ad, fe.m_b);
            fe.m_ac.Print(e);
        }
        inline bool AssertEqual(const Source &A, const int verbose = 1,
                                const float eps = 0.0f) const
        {
            if (UT::VectorAssertEqual<float>(fe.m_ac, A.fe.m_ac, 8, 0, eps))
                return true;
            if (verbose) {
                UT::PrintSeparator();
                Print(verbose > 1);
                A.Print(verbose > 1);
            }
            return false;
        }
        static inline void Marginalize(const Source &Mdc, const Source &Adc,
                                       Camera::Pose::Unitary &Mcc)
        {
#define m_ac fe.m_ac
#define m_b fe.m_b
#define m_A fe.m_A
            _pi__m128 t1, t2;
            t1 = _pi_mm_set1_ps(Mdc.m_ac.v0());
            Mcc.m_data[0] = _pi_mm_mul_ps(t1, Adc.m_data[0]);
            t2 = _pi_mm_mul_ps(t1, Adc.m_data[1]);
            memcpy(&Mcc.m_A.m04(), &t2.m128_f32[0], 8);
            Mcc.m_b.v0() = t2.m128_f32[2];
            t1 = _pi_mm_set1_ps(Mdc.m_ac.v1());
            t2 = _pi_mm_mul_ps(t1, Adc.m_data[0]);
            memcpy(&Mcc.m_A.m11(), &t2.m128_f32[1], 12);
            t2 = _pi_mm_mul_ps(t1, Adc.m_data[1]);
            memcpy(&Mcc.m_A.m14(), &t2.m128_f32[0], 8);
            Mcc.m_b.v1() = t2.m128_f32[2];
            Mcc.m_A.m22() = Mdc.m_ac.v2() * Adc.m_ac.v2();
            Mcc.m_A.m23() = Mdc.m_ac.v2() * Adc.m_ac.v3();
            t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(Mdc.m_ac.v2()), Adc.m_data[1]);
            memcpy(&Mcc.m_A.m24(), &t2.m128_f32[0], 8);
            Mcc.m_b.v2() = t2.m128_f32[2];
            Mcc.m_A.m33() = Mdc.m_ac.v3() * Adc.m_ac.v3();
            t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(Mdc.m_ac.v3()), Adc.m_data[1]);
            memcpy(&Mcc.m_A.m34(), &t2.m128_f32[0], 8);
            Mcc.m_b.v3() = t2.m128_f32[2];
            t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(Mdc.m_ac.v4()), Adc.m_data[1]);
            memcpy(&Mcc.m_A.m44(), &t2.m128_f32[0], 8);
            Mcc.m_b.v4() = t2.m128_f32[2];
            Mcc.m_A.m55() = Mdc.m_ac.v5() * Adc.m_ac.v5();
            Mcc.m_b.v5() = Mdc.m_ac.v5() * Adc.m_b;
#undef m_ac
#undef m_b
#undef m_A
        }

      public:
        struct CanNotMakeAnonymous {
            LA::Vector6f m_ac;
            float m_b;
            union {
                struct {
                    float m_ad;
                };
                struct {
                    float m_md;
                };
            };
        };
        union {
            CanNotMakeAnonymous fe;
            _pi__m128 m_data[2];
        };
    };
    class Measurement : public LA::Vector6f
    {
      public:
        inline Measurement() {}
        inline Measurement(const LA::Vector6f &A) { memcpy(this, A, 24); }
        static inline void Marginalize(const LA::AlignedVector6f &Mdc,
                                       const LA::AlignedVector6f &Adc,
                                       const float bd,
                                       Camera::Pose::Unitary &Mcc)
        {
#define m_A fe.m_A
#define m_b fe.m_b
            _pi__m128 t1, t2;
            const _pi__m128 t3 = _pi_mm_setr_ps(Adc.v4(), Adc.v5(), bd, 0.0f);
            t1 = _pi_mm_set1_ps(Mdc.v0());
            Mcc.m_data[0] = _pi_mm_mul_ps(t1, Adc.v0123());
            t2 = _pi_mm_mul_ps(t1, t3);
            memcpy(&Mcc.m_A.m04(), &t2.m128_f32[0], 8);
            Mcc.m_b.v0() = t2.m128_f32[2];
            t1 = _pi_mm_set1_ps(Mdc.v1());
            t2 = _pi_mm_mul_ps(t1, Adc.v0123());
            memcpy(&Mcc.m_A.m11(), &t2.m128_f32[1], 12);
            t2 = _pi_mm_mul_ps(t1, t3);
            memcpy(&Mcc.m_A.m14(), &t2.m128_f32[0], 8);
            Mcc.m_b.v1() = t2.m128_f32[2];
            Mcc.m_A.m22() = Mdc.v2() * Adc.v2();
            Mcc.m_A.m23() = Mdc.v2() * Adc.v3();
            t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(Mdc.v2()), t3);
            memcpy(&Mcc.m_A.m24(), &t2.m128_f32[0], 8);
            Mcc.m_b.v2() = t2.m128_f32[2];
            Mcc.m_A.m33() = Mdc.v3() * Adc.v3();
            t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(Mdc.v3()), t3);
            memcpy(&Mcc.m_A.m34(), &t2.m128_f32[0], 8);
            Mcc.m_b.v3() = t2.m128_f32[2];
            t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(Mdc.v4()), t3);
            memcpy(&Mcc.m_A.m44(), &t2.m128_f32[0], 8);
            Mcc.m_b.v4() = t2.m128_f32[2];
            Mcc.m_A.m55() = Mdc.v5() * Adc.v5();
            Mcc.m_b.v5() = Mdc.v5() * bd;
#undef m_A
#undef m_b
        }
        inline float BackSubstitute(const LA::ProductVector6f &xc) const
        {
            return SSE::Sum(_pi_mm_mul_ps(_pi_mm_loadu_ps(&v0()), xc.v0123())) +
                   v4() * xc.v4() + v5() * xc.v5();
        }
        static inline void BackSubstitute(const LA::ProductVector6f &Mdc,
                                          const LA::ProductVector6f &xc,
                                          float *xd)
        {
            const _pi__m128 t = _pi_mm_mul_ps(Mdc.v4501(), xc.v4501());
            xd[0] = SSE::Sum(_pi_mm_mul_ps(Mdc.v0123(), xc.v0123())) +
                    t.m128_f32[0] + t.m128_f32[1];
            xd[1] = SSE::Sum(_pi_mm_mul_ps(Mdc.v2345(), xc.v2345())) +
                    t.m128_f32[2] + t.m128_f32[3];
        }
        static inline void Marginalize(const Source &Adc1,
                                       const LA::ProductVector6f &Mdc2,
                                       Camera::Pose::Binary &Mcc)
        {
            LA::AlignedMatrix6x6f::abT(Adc1.fe.m_ac, Mdc2, Mcc);
        }
        static inline void Marginalize(const Measurement &Adc1,
                                       const LA::ProductVector6f &Mdc2,
                                       Camera::Pose::Binary &Mcc)
        {
            LA::AlignedMatrix6x6f::abT(Adc1, Mdc2, Mcc);
        }
    };
};

}

#endif
