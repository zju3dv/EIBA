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

#ifndef _DEPTH_H_
#define _DEPTH_H_

#include "stdafx.h"

#include "AlignedVector.h"
#include "Intrinsic.h"
#include "Rigid.h"

#ifdef CFG_TUNE_PARAMETERS
extern float DEPTH_MIN;
extern float DEPTH_MAX;
extern float DEPTH_RANGE;
extern float DEPTH_INITIAL_MEAN;
extern float DEPTH_INITIAL_VARIANCE;
extern float DEPTH_EPSILON;
extern float DEPTH_WALK_VARIANCE;
extern float DEPTH_RELIABLE_VARIANCE;
extern float DEPTH_CONVERGE_VARIANCE;
extern float DEPTH_MIN_INLIER_RATIO;
#ifdef CFG_DEPTH_MAP
extern float DEPTH_MAP_FACTOR;
extern float DEPTH_MAP_VARIANCE;
extern int DEPTH_MAP_SMOOTHNESS_PATCH_SIZE;
extern int DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF;
extern int DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS;
extern int DEPTH_MAP_SMOOTHNESS_PATCH_CENTER;
extern ushort DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION;
#endif
#include "Configurator.h"
extern void LOAD_PARAMETERS_DEPTH(const Configurator &cfgor);
#else
#define DEPTH_MIN 0.02f // 1/50
#define DEPTH_MAX 2.0f  // 1/0.5
//#define DEPTH_MAX								10.0f
////
// 1/0.1
#define DEPTH_RANGE 9.98f // DEPTH_MAX - DEPTH_MIN
//#define DEPTH_INITIAL_MEAN					0.2f	// 1/5
#define DEPTH_INITIAL_MEAN 0.5f     // 1/2
#define DEPTH_INITIAL_VARIANCE 1.0f // 1^2
#define DEPTH_EPSILON 0.001f
#define DEPTH_WALK_VARIANCE 0.01f // 0.1^2
#define DEPTH_RELIABLE_VARIANCE 1.0e-2f // 0.1^2
//#define DEPTH_RELIABLE_VARIANCE				1.0f	// 1.0^2
#define DEPTH_CONVERGE_VARIANCE 1.0e-6f // 0.001^2
//#define DEPTH_MIN_INLIER_RATIO				0.05f
#define DEPTH_MIN_INLIER_RATIO 0.4f
#ifdef CFG_DEPTH_MAP
#define DEPTH_MAP_FACTOR 5000.0f // d = 5000/z
//#define DEPTH_MAP_VARIANCE 0.01f // 0.1^2
extern float DEPTH_MAP_VARIANCE;
//#define DEPTH_MAP_SMOOTHNESS_PATCH_SIZE		2
#define DEPTH_MAP_SMOOTHNESS_PATCH_SIZE 4
#define DEPTH_MAP_SMOOTHNESS_PATCH_SIZE_HALF 1
#define DEPTH_MAP_SMOOTHNESS_PATCH_PIXELS 16
#define DEPTH_MAP_SMOOTHNESS_PATCH_CENTER 5
#define DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION 250 // 0.05 * DEPTH_MAP_FACTOR
//#define DEPTH_MAP_SMOOTHNESS_MAX_DEVIATION	2500	// 0.5 *
// DEPTH_MAP_FACTOR
#endif
#endif
#ifdef CFG_DEPTH_MAP
extern float DEPTH_MAP_FACTOR_INPUT;
#endif

class DepthInverseGaussian
{
  public:
    inline DepthInverseGaussian() {}
    inline DepthInverseGaussian(const float u, const float s2)
        : m_u(u), m_s2(s2)
    {
    }

    inline const float &u() const { return m_u; }
    inline float &u() { return m_u; }
    inline const float &s2() const { return m_s2; }
    inline float &s2() { return m_s2; }
    inline bool operator==(const DepthInverseGaussian &d) const
    {
        return u() == d.u() && s2() == d.s2();
    }

    // inline void Initialize() { u() = DEPTH_INITIAL_MEAN; s2() =
    // DEPTH_INITIAL_VARIANCE; }
    inline void Initialize(const float u)
    {
        this->u() = u;
        const float DEPTH_INITIAL_VARIANCE = 1.0f; // 1^2
        s2() = DEPTH_INITIAL_VARIANCE;
    }
    inline void Initialize(const float u, const float s2)
    {
        this->u() = u;
        this->s2() = s2;
    }
    inline void Propagate(const float dt)
    {
        s2() = DEPTH_WALK_VARIANCE * dt * dt + s2();
    }
    inline void Update(const DepthInverseGaussian &d)
    {
        const float Ss2 = s2() + d.s2();
        if (Ss2 < FLT_EPSILON) return;
        const float Ss2I = 1.0f / Ss2;
        u() = (s2() * d.u() + d.s2() * u()) * Ss2I;
        s2() = s2() * d.s2() * Ss2I;
    }
    inline bool Valid() const { return u() > 0.0f; }
    inline bool Invalid() const { return u() == 0.0f; }
    inline void Invalidate() { u() = 0.0f; }
    inline bool Reliable() const { return s2() <= DEPTH_RELIABLE_VARIANCE; }
    inline bool Converge() const { return s2() <= DEPTH_CONVERGE_VARIANCE; }
    inline void Project(const Rigid3D &T12, const Point2D &x1,
                        LA::Vector2f &x2) const
    {
        Project(T12, x1, u(), x2);
    }
    static inline void Project(const Rigid3D &T12, const Point2D &x1,
                               const float d1, LA::Vector2f &x2)
    {
        const _pi__m128 t = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, d1);
        const float d12 =
            1.0f / SSE::Sum(_pi_mm_mul_ps(T12.r20_r21_r22_tz(), t));
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t)) * d12;
    }
    inline void Project(const Rigid3D &T12, const Point2D &x1, LA::Vector2f &x2,
                        LA::Vector2f &Jx2) const
    {
        const _pi__m128 t = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u());
        const float d12 =
            1.0f / SSE::Sum(_pi_mm_mul_ps(T12.r20_r21_r22_tz(), t));
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t)) * d12;
        Jx2.x() = (T12.tx() - x2.x() * T12.tz()) * d12;
        Jx2.y() = (T12.ty() - x2.y() * T12.tz()) * d12;
    }
    inline void Project(const Rigid3D &T12, const Point2D &x1, LA::Vector2f &x2,
                        float &d2) const
    {
        const _pi__m128 t = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u());
        const float d12 =
            1.0f / SSE::Sum(_pi_mm_mul_ps(T12.r20_r21_r22_tz(), t));
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t)) * d12;
        d2 = d12 * u();
    }
    inline void Project(const Rigid3D &T12, const Point2D &x1, LA::Vector2f &x2,
                        float &d2, LA::Vector2f &Jx2) const
    {
        const _pi__m128 t = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u());
        const float d12 =
            1.0f / SSE::Sum(_pi_mm_mul_ps(T12.r20_r21_r22_tz(), t));
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t)) * d12;
        d2 = d12 * u();
        Jx2.x() = (T12.tx() - x2.x() * T12.tz()) * d12;
        Jx2.y() = (T12.ty() - x2.y() * T12.tz()) * d12;
    }
    inline void Project(const Rigid3D &T12, const Point2D &x1, LA::Vector2f &x2,
                        float &d12, float &d2, LA::Vector2f &Jx2) const
    {
        const _pi__m128 t = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u());
        d12 = 1.0f / SSE::Sum(_pi_mm_mul_ps(T12.r20_r21_r22_tz(), t));
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t)) * d12;
        d2 = d12 * u();
        Jx2.x() = (T12.tx() - x2.x() * T12.tz()) * d12;
        Jx2.y() = (T12.ty() - x2.y() * T12.tz()) * d12;
    }
#ifdef CFG_DEPTH_MAP
    inline void Project(const Rigid3D &T12, const Point2D &x1, LA::Vector2f &x2,
                        float &d2, LA::AlignedVector3f &Jxd2) const
    {
        const _pi__m128 t1 = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u());
        const _pi__m128 t2 = _pi_mm_mul_ps(T12.r20_r21_r22_tz(), t1);
        const float t3 = SSE::Sum012(t2);
        const float d12 = 1.0f / (t2.m128_f32[3] + t3);
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t1)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t1)) * d12;
        d2 = d12 * u();
        Jxd2.x() = (T12.tx() - x2.x() * T12.tz()) * d12;
        Jxd2.y() = (T12.ty() - x2.y() * T12.tz()) * d12;
        Jxd2.z() = d12 * d12 * t3;
    }
    inline void Project(const Rigid3D &T12, const Point2D &x1, LA::Vector2f &x2,
                        float &d12, float &d2, LA::AlignedVector3f &Jxd2) const
    {
        const _pi__m128 t1 = _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u());
        const _pi__m128 t2 = _pi_mm_mul_ps(T12.r20_r21_r22_tz(), t1);
        const float t3 = SSE::Sum012(t2);
        d12 = 1.0f / (t2.m128_f32[3] + t3);
        x2.x() = SSE::Sum(_pi_mm_mul_ps(T12.r00_r01_r02_tx(), t1)) * d12;
        x2.y() = SSE::Sum(_pi_mm_mul_ps(T12.r10_r11_r12_ty(), t1)) * d12;
        d2 = d12 * u();
        Jxd2.x() = (T12.tx() - x2.x() * T12.tz()) * d12;
        Jxd2.y() = (T12.ty() - x2.y() * T12.tz()) * d12;
        Jxd2.z() = d12 * d12 * t3;
    }
#endif

    inline Point2D GetProjected(const Rigid3D &T12, const Point2D &x1) const
    {
        Point2D x2;
        Project(T12, x1, x2);
        return x2;
    }
    inline float GetProjectedZ(const Rigid3D &T12, const Point2D &x1) const
    {
        return SSE::Sum(
                   _pi_mm_mul_ps(T12.r20_r21_r22_tz(),
                                 _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u()))) /
               u();
    }
    inline bool ProjectD(const Rigid3D::Row &T12z, const Point2D &x1,
                         DepthInverseGaussian &d2) const
    {
        const _pi__m128 t1 = _pi_mm_mul_ps(
            T12z.r0_r1_r2_t(), _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u()));
        const float t2 = SSE::Sum012(t1), t3 = 1.0f / (t2 + t1.m128_f32[3]);
        d2.u() = u() * t3;
        if (d2.u() < FLT_EPSILON) return false;
        const float j = t2 * t3 * t3;
        d2.s2() = j * j * s2();
        return true;
    }
    inline float GetProjectedD(const Rigid3D::Row &T12z,
                               const Point2D &x1) const
    {
        return u() / SSE::Sum(_pi_mm_mul_ps(
                         T12z.r0_r1_r2_t(),
                         _pi_mm_setr_ps(x1.x(), x1.y(), 1.0f, u())));
    }
    inline void Print(const bool e = false, const bool n = true) const
    {
        if (Invalid()) return;
        if (e)
            UT::Print("%e +- %e", u(), sqrt(s2()));
        else
            UT::Print("%f +- %f", u(), sqrt(s2()));
        if (n) UT::Print("\n");
    }
    inline void Print(const std::string str, const bool e, const bool n) const
    {
        if (Invalid()) return;
        UT::Print("%s", str.c_str());
        Print(e, n);
    }

  protected:
    float m_u, m_s2;
};

class DepthInverseGaussianBeta : public DepthInverseGaussian
{
  public:
    inline const float &a() const { return m_a; }
    inline float &a() { return m_a; }
    inline const float &b() const { return m_b; }
    inline float &b() { return m_b; }
    // inline void Initialize() { DepthInverseGaussian::Initialize(); a() = b()
    // = 10.0f; }
    inline void Initialize(const float u)
    {
        DepthInverseGaussian::Initialize(u);
        a() = b() = 10.0f;
    }
    inline void Initialize(const DepthInverseGaussian &d)
    {
        u() = d.u();
        s2() = d.s2();
        a() = b() = 10.0f;
    }
    inline void Initialize(const float u, const float s2)
    {
        DepthInverseGaussian::Initialize(u, s2);
        a() = b() = 10.0f;
    }
    inline bool Update(const DepthInverseGaussian &d)
    {
        const float Ss2 = s2() + d.s2();
        if (Ss2 < FLT_EPSILON) return true;
        const float SabI = 1.0f / (a() + b()), Ss2I = 1.0f / Ss2,
                    dz = d.u() - u(), dz2 = dz * dz;
        const float C1 =
            a() * SabI * exp(-dz * dz * 0.5f * Ss2I) * sqrt(UT_1_2PI * Ss2I);
        const float C2 = b() * SabI / DEPTH_RANGE;
        const float SCI = 1.0f / (C1 + C2);
        const float Cn1 = C1 * SCI, Cn2 = C2 * SCI;
        const float _s2 = 1.0f / (1.0f / s2() + 1.0f / d.s2()),
                    m = _s2 * (u() / s2() + d.u() / d.s2());
        const float _u = Cn1 * m + Cn2 * u();
        s2() = Cn1 * (_s2 + m * m) + Cn2 * (s2() + u() * u()) - _u * _u;
        u() = _u;

        const float Sab1I = 1.0f / (a() + b() + 1.0f),
                    Sab2I = 1.0f / (a() + b() + 2.0f);
        const float G1 = Cn1 * (a() + 1.0f) * Sab1I, G2 = Cn2 * a() * Sab1I;
        const float R1 = G1 + G2, R1I = 1.0f / R1;
        const float R2 = (G1 * (a() + 2.0f) + G2 * (a() + 1.0f)) * Sab2I;
        a() = (R2 - R1) / (R1 - R2 * R1I);
        b() = a() * (R1I - 1.0f);

        return u() >= DEPTH_MIN && u() <= DEPTH_MAX &&
               a() >= (a() + b()) * DEPTH_MIN_INLIER_RATIO;
    }
    inline void Print(const std::string str = "", const bool e = false,
                      const bool n = true) const
    {
        DepthInverseGaussian::Print(str, e, false);
        UT::Print("  pi = %.2f%% (%.2f %.2f)", a() * 100 / (a() + b()), a(),
                  b());
        if (n) UT::Print("\n");
    }

  protected:
    float m_a, m_b;
};

#endif
