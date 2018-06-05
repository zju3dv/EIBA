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

#ifndef _INTRINSIC_H_
#define _INTRINSIC_H_

#include "stdafx.h"

#include "AlignedVector.h"
#include "Matrix3x3.h"
#include "Point.h"
#include "Vector4.h"

class Intrinsic
{
  public:
    class Parameter
    {
      public:
        inline void DownSample()
        {
            m_fx *= 0.5f;
            m_fy *= 0.5f;
            m_cx *= 0.5f;
            m_cy *= 0.5f;
        }
        inline void GetDownSampled(Parameter &k) const
        {
            k.m_fx = m_fx * 0.5f;
            k.m_fy = m_fy * 0.5f;
            k.m_cx = m_cx * 0.5f;
            k.m_cy = m_cy * 0.5f;
            memcpy(k.m_ds, m_ds, sizeof(k.m_ds));
        }

      public:
        float m_fx, m_fy, m_cx, m_cy, m_ds[5];
    };

  public:
    inline const int &w() const { return m_w; }
    inline const int &h() const { return m_h; }
    inline const Parameter &k() const { return m_k; }
    inline const bool NeedRectification() const { return m_needRect == 1; }
    inline const bool FishEye() const { return m_fishEye == 1; }
    inline const float &fx() const { return m_fx.m128_f32[0]; }
    inline const float &fy() const { return m_fy.m128_f32[0]; }
    inline const float &fxI() const { return m_fxI.m128_f32[0]; }
    inline const float &fyI() const { return m_fyI.m128_f32[0]; }
    inline const float &cx() const { return m_cx.m128_f32[0]; }
    inline const float &cy() const { return m_cy.m128_f32[0]; }
    inline const float &fxIcx() const { return m_fxIcx.m128_f32[0]; }
    inline const float &fyIcy() const { return m_fyIcy.m128_f32[0]; }
    inline const float &fxx() const { return m_fxx; }
    inline const float &fxxI() const { return m_fxxI; }
    inline const float &fxy() const { return m_fxy; }
    inline const float &fxyI() const { return m_fxyI; }
    inline const float &fyy() const { return m_fyy; }
    inline const float &fyyI() const { return m_fyyI; }
    inline const _pi__m128 &Fx() const { return m_fx; }
    inline const _pi__m128 &Fy() const { return m_fy; }
    inline const _pi__m128 &FxI() const { return m_fxI; }
    inline const _pi__m128 &FyI() const { return m_fyI; }
    inline const _pi__m128 &Cx() const { return m_cx; }
    inline const _pi__m128 &Cy() const { return m_cy; }
    inline const _pi__m128 &FxIcx() const { return m_fxIcx; }
    inline const _pi__m128 &FyIcy() const { return m_fyIcy; }
    inline LA::AlignedMatrix3x3f operator*(const LA::AlignedMatrix3x3f &M) const
    {
        LA::AlignedMatrix3x3f KM;
        KM.m_00_01_02_r0() =
            _pi_mm_add_ps(_pi_mm_mul_ps(Fx(), M.m_00_01_02_r0()),
                          _pi_mm_mul_ps(Cx(), M.m_20_21_22_r2()));
        KM.m_10_11_12_r1() =
            _pi_mm_add_ps(_pi_mm_mul_ps(Fy(), M.m_10_11_12_r1()),
                          _pi_mm_mul_ps(Cy(), M.m_20_21_22_r2()));
        KM.m_20_21_22_r2() = M.m_20_21_22_r2();
        return KM;
    }
    inline LA::AlignedVector3f operator*(const LA::AlignedVector3f &v) const
    {
        LA::AlignedVector3f Kv;
        Kv.x() = fx() * v.x() + cx() * v.z();
        Kv.y() = fy() * v.y() + cy() * v.z();
        Kv.z() = v.z();
        return Kv;
    }

    inline void Set(const int w, const int h, const float fx,
                    const float fy = 0.0f, const float cx = 0.0f,
                    const float cy = 0.0f, const float *ds = NULL,
                    const bool fishEye = false, const float fxr = 0.0f,
                    const float fyr = 0.0f)
    {
        m_w = w;
        m_h = h;
        m_k.m_fx = fx;
        m_k.m_fy = fy == 0.0f ? fx : fy;
        const float _cx = (w - 1) * 0.5f, _cy = (h - 1) * 0.5f;
        m_k.m_cx = cx == 0.0f ? _cx : cx;
        m_k.m_cy = cy == 0.0f ? _cy : cy;
        if (ds || fxr != 0.0f || fyr != 0.0f) {
            m_needRect = 1;
            m_fishEye = fishEye ? 1 : 0;
            if (ds)
                memcpy(m_k.m_ds, ds, sizeof(m_k.m_ds));
            else
                memset(m_k.m_ds, 0, sizeof(m_k.m_ds));
            const float _fxr = fxr == 0.0f ? GetRectifiedFocal() : fxr;
            const float _fyr = fyr == 0.0f ? _fxr : fyr;
            Set(_fxr, _fyr, _cx, _cy);
        } else {
            Set(m_k.m_fx, m_k.m_fy, m_k.m_cx, m_k.m_cy);
            m_needRect = 0;
            m_fishEye = 0;
        }
    }
    inline void DownSample()
    {
        m_w /= 2;
        m_h /= 2;
        m_k.DownSample();
        Set(fx() * 0.5f, fy() * 0.5f, cx() * 0.5f, cy() * 0.5f);
    }
    inline void GetDownSampled(Intrinsic &K) const
    {
        K.m_w = m_w / 2;
        K.m_w = SSE_BYTE_FLOOR(K.m_w);
        K.m_h = m_h / 2;
        K.m_needRect = m_needRect;
        m_k.GetDownSampled(K.m_k);
        K.Set(fx() * 0.5f, fy() * 0.5f, cx() * 0.5f, cy() * 0.5f);
    }
    inline double GetFovX() const { return FocalToFov(m_w, fxI()); }
    inline double GetFovY() const { return FocalToFov(m_h, fyI()); }
    static inline double FocalToFov(const int r, const float fI)
    {
        return atan(r * 0.5 * fI) * 2 * UT_FACTOR_RAD_TO_DEG;
    }
    static inline float FovToFocal(const int r, const double fov)
    {
        return float(r / (2 * tan(fov * 0.5 * UT_FACTOR_DEG_TO_RAD)));
    }

    inline void Get(LA::AlignedMatrix3x3f &K) const
    {
        K.m00() = fx();
        K.m01() = 0.0f;
        K.m02() = cx();
        K.m10() = 0.0f;
        K.m11() = fy();
        K.m12() = cy();
        K.m20() = 0.0f;
        K.m21() = 0.0f;
        K.m22() = 1.0f;
    }
    inline void GetInverse(LA::AlignedMatrix3x3f &KI) const
    {
        KI.m00() = fxI();
        KI.m01() = 0.0f;
        KI.m02() = -cx() * fxI();
        KI.m10() = 0.0f;
        KI.m11() = fyI();
        KI.m12() = -cy() * fyI();
        KI.m20() = 0.0f;
        KI.m21() = 0.0f;
        KI.m22() = 1.0f;
    }
    inline void GetTranspose(LA::AlignedMatrix3x3f &KT) const
    {
        KT.m00() = fx();
        KT.m01() = 0.0f;
        KT.m02() = 0.0f;
        KT.m10() = 0.0f;
        KT.m11() = fy();
        KT.m12() = 0.0f;
        KT.m20() = cx();
        KT.m21() = cy();
        KT.m22() = 1.0f;
    }
    inline void GetTransposeInverse(LA::AlignedMatrix3x3f &KTI) const
    {
        KTI.m00() = fxI();
        KTI.m01() = 0.0f;
        KTI.m02() = 0.0f;
        KTI.m10() = 0.0f;
        KTI.m11() = fyI();
        KTI.m12() = 0.0f;
        KTI.m20() = -cx() * fxI();
        KTI.m21() = -cy() * fyI();
        KTI.m22() = 1.0f;
    }
    inline void GetInverseTranspose(LA::AlignedMatrix3x3f &KIT) const
    {
        GetTransposeInverse(KIT);
    }
    inline void GetInverse(Intrinsic &KI) const
    {
        KI.Set(fxI(), fyI(), -cx() * fxI(), -cy() * fyI());
    }

    inline float GetRectifiedFocal() const
    {
        return (m_k.m_fx + m_k.m_fy) * 0.5f;
    }

    inline void NormalizedToImage(Point2D &x) const
    {
        x.x() = fx() * x.x() + cx();
        x.y() = fy() * x.y() + cy();
    }
    inline void NormalizedToImage(const Point2D &xn, Point2D &x) const
    {
        x.x() = fx() * xn.x() + cx();
        x.y() = fy() * xn.y() + cy();
    }
    inline Point2D GetNormalizedToImage(const Point2D &xn) const
    {
        Point2D x;
        NormalizedToImage(xn, x);
        return x;
    }
    inline void NormalizedToImage2(const _pi__m128 &xn, _pi__m128 &x) const
    {
        x = _pi_mm_add_ps(_pi_mm_mul_ps(m_f, xn), m_c);
    }
    inline void NormalizedToImage2(_pi__m128 &x) const
    {
        x = _pi_mm_add_ps(_pi_mm_mul_ps(m_f, x), m_c);
    }
    inline void NormalizedToImageN(AlignedVector<Point2D> &xs) const
    {
        const int N = int(xs.Size()), NF = N - (N & 1);
        _pi__m128 *x = (_pi__m128 *)xs.Data();
        for (int i = 0; i < NF; i += 2, ++x) NormalizedToImage2(*x);
        if (NF != N) NormalizedToImage(xs[NF]);
    }
    inline void NormalizedToImageN(const AlignedVector<Point2D> &xns,
                                   AlignedVector<Point2D> &xs) const
    {
        const int N = int(xs.Size()), NF = N - (N & 1);
        xs.Resize(N);
        const _pi__m128 *xn = (const _pi__m128 *)xns.Data();
        _pi__m128 *x = (_pi__m128 *)xs.Data();
        for (int i = 0; i < NF; i += 2, ++x, ++x) NormalizedToImage2(*xn, *x);
        if (NF != N) NormalizedToImage(xns[NF], xs[NF]);
    }
    inline void NormalizedToImageN(const Point2D *xns, const int N,
                                   Point2D *xs) const
    {
        const int NF = N - (N & 1);
        const _pi__m128 *xn = (const _pi__m128 *)xns;
        _pi__m128 *x = (_pi__m128 *)xs;
        for (int i = 0; i < NF; i += 2, ++xn, ++x) NormalizedToImage2(*xn, *x);
        if (NF != N) NormalizedToImage(xns[NF], xs[NF]);
    }
    inline void NormalizedToImageN(Point2D *xs, const int N) const
    {
        const int NF = N - (N & 1);
        _pi__m128 *x = (_pi__m128 *)xs;
        for (int i = 0; i < NF; i += 2, ++x) NormalizedToImage2(*x);
        if (NF != N) NormalizedToImage(xs[NF]);
    }
    // inline void NormalizedToImage(Line2D &l) const
    //{
    //	l.c() -= l.a() * cxfxI() + l.b() * cyfyI();
    //	l.a() *= fxI();
    //	l.b() *= fyI();
    //	l.Normalize();
    //}
    // inline void NormalizedToImage(const Line2D &ln, Line2D &Kl) const
    //{
    //	Kl.c() = ln.c() - (ln.a() * cxfxI() + ln.b() * cyfyI());
    //	Kl.a() = ln.a() * fxI();
    //	Kl.b() = ln.b() * fyI();
    //	Kl.Normalize();
    //}
    inline void NormalizedToImage(LA::SymmetricMatrix2x2f &S) const
    {
        S.m00() *= fxx();
        S.m01() *= fxy();
        S.m11() *= fyy();
    }
    inline void NormalizedToImage(const LA::SymmetricMatrix2x2f &Sn,
                                  LA::SymmetricMatrix2x2f &S) const
    {
        S.m00() = Sn.m00() * fxx();
        S.m01() = Sn.m01() * fxy();
        S.m11() = Sn.m11() * fyy();
    }
    inline LA::SymmetricMatrix2x2f
    GetNormalizedToImage(const LA::SymmetricMatrix2x2f &Sn) const
    {
        Point2DCovariance S;
        NormalizedToImage(Sn, S);
        return S;
    }
    inline void NormalizedToImage(const LA::AlignedMatrix3x3f &Hn,
                                  LA::AlignedMatrix3x3f &H) const
    {
        NormalizedToImage(*this, *this, Hn, H);
    }
    inline LA::AlignedMatrix3x3f
    GetNormalizedToImage(const LA::AlignedMatrix3x3f &Hn) const
    {
        LA::AlignedMatrix3x3f H;
        NormalizedToImage(Hn, H);
        return H;
    }
    // H = K2 * Hn * K1^{-1}
    static inline void NormalizedToImage(const Intrinsic &K1,
                                         const Intrinsic &K2,
                                         const LA::AlignedMatrix3x3f &Hn,
                                         LA::AlignedMatrix3x3f &H)
    {
        LA::AlignedMatrix3x3f M;
        M.m_00_01_02_r0() =
            _pi_mm_add_ps(_pi_mm_mul_ps(K2.Fx(), Hn.m_00_01_02_r0()),
                          _pi_mm_mul_ps(K2.Cx(), Hn.m_20_21_22_r2()));
        M.m_10_11_12_r1() =
            _pi_mm_add_ps(_pi_mm_mul_ps(K2.Fy(), Hn.m_10_11_12_r1()),
                          _pi_mm_mul_ps(K2.Cy(), Hn.m_20_21_22_r2()));
        M.m_20_21_22_r2() = Hn.m_20_21_22_r2();
        M.Transpose();
        H.m_00_01_02_r0() = _pi_mm_mul_ps(M.m_00_01_02_r0(), K1.FxI());
        H.m_10_11_12_r1() = _pi_mm_mul_ps(M.m_10_11_12_r1(), K1.FyI());
        H.m_20_21_22_r2() = _pi_mm_sub_ps(
            M.m_20_21_22_r2(),
            _pi_mm_add_ps(_pi_mm_mul_ps(M.m_00_01_02_r0(), K1.FxIcx()),
                          _pi_mm_mul_ps(M.m_10_11_12_r1(), K1.FyIcy())));
        H.Transpose();
    }

    inline void NormalizedToImageDistorted(const Point2D &xn, Point2D &xd) const
    {
        const float x = xn.x(), x2 = x * x, y = xn.y(), y2 = y * y,
                    r2 = x2 + y2;
        if (FishEye()) {
            const float r = sqrt(r2), t1 = atan(r), t2 = t1 * t1, t4 = t2 * t2,
                        t6 = t2 * t4, t8 = t4 * t4;
            const float s = t1 * (m_k.m_ds[3] * t8 + m_k.m_ds[2] * t6 +
                                  m_k.m_ds[1] * t4 + m_k.m_ds[0] * t2 + 1.0f) /
                            r;
            xd.x() = s * x;
            xd.y() = s * y;
        } else {
            const float d2y = m_k.m_ds[2] * y * 2.0f,
                        d3y = m_k.m_ds[3] * y * 2.0f;
            const float r4 = r2 * r2, r6 = r2 * r4;
            const float dr =
                (m_k.m_ds[4] * r6 + m_k.m_ds[1] * r4 + m_k.m_ds[0] * r2 + 1.0f);
            const float dx = d2y * x + m_k.m_ds[3] * (r2 + x2 + x2);
            const float dy = m_k.m_ds[2] * (r2 + y2 + y2) + d3y * x;
            xd.x() = dx + dr * x;
            xd.y() = dy + dr * y;
        }
        xd.x() = m_k.m_fx * xd.x() + m_k.m_cx;
        xd.y() = m_k.m_fy * xd.y() + m_k.m_cy;
    }

    inline void ImageToNormalized(Point2D &x) const
    {
        x.x() = (x.x() - cx()) * fxI();
        x.y() = (x.y() - cy()) * fyI();
    }
    inline void ImageToNormalized(const Point2D &x, Point2D &xn) const
    {
        xn.x() = (x.x() - cx()) * fxI();
        xn.y() = (x.y() - cy()) * fyI();
    }
    inline Point2D GetImageToNormalized(const Point2D &x) const
    {
        Point2D xn;
        ImageToNormalized(x, xn);
        return xn;
    }
    template <typename TYPE>
    inline void ImageToNormalized(const TYPE xx, const TYPE xy,
                                  Point2D &xn) const
    {
        xn.x() = (xx - cx()) * fxI();
        xn.y() = (xy - cy()) * fyI();
    }
    template <typename TYPE>
    inline Point2D GetImageToNormalized(const TYPE xx, const TYPE xy) const
    {
        Point2D xn;
        ImageToNormalized<TYPE>(xx, xy, xn);
        return xn;
    }
    template <typename TYPE>
    inline void ImageToNormalized(const TYPE xx, const TYPE xy, float &xn,
                                  float &yn) const
    {
        xn = (xx - cx()) * fxI();
        yn = (xy - cy()) * fyI();
    }
    inline void ImageToNormalized2(_pi__m128 &x) const
    {
        x = _pi_mm_mul_ps(_pi_mm_sub_ps(x, m_c), m_fI);
    }
    inline void ImageToNormalized2(const _pi__m128 &x, _pi__m128 &xn) const
    {
        xn = _pi_mm_mul_ps(_pi_mm_sub_ps(x, m_c), m_fI);
    }
    inline void ImageToNormalizedN(AlignedVector<Point2D> &xs) const
    {
        const int N = int(xs.Size()), NF = N - (N & 1);
        _pi__m128 *x = (_pi__m128 *)xs.Data();
        for (int i = 0; i < NF; i += 2, ++x) ImageToNormalized2(*x);
        if (NF != N) ImageToNormalized(xs[NF]);
    }
    inline void ImageToNormalizedN(const AlignedVector<Point2D> &xs,
                                   AlignedVector<Point2D> &xns) const
    {
        const int N = int(xs.Size()), NF = N - (N & 1);
        xns.Resize(N);
        const _pi__m128 *x = (const _pi__m128 *)xs.Data();
        _pi__m128 *xn = (_pi__m128 *)xns.Data();
        for (int i = 0; i < NF; i += 2, ++x, ++xn) ImageToNormalized2(*x, *xn);
        if (NF != N) ImageToNormalized(xs[NF], xns[NF]);
    }
    inline void ImageToNormalizedN(const Point2D *xs, const int N,
                                   Point2D *xns) const
    {
        const int NF = N - (N & 1);
        const _pi__m128 *x = (const _pi__m128 *)xs;
        _pi__m128 *xn = (_pi__m128 *)xns;
        for (int i = 0; i < NF; i += 2, ++x, ++xn) ImageToNormalized2(*x, *xn);
        if (NF != N) ImageToNormalized(xs[NF], xns[NF]);
    }
    inline void ImageToNormalizedN(Point2D *xs, const int N) const
    {
        const int NF = N - (N & 1);
        _pi__m128 *x = (_pi__m128 *)xs;
        for (int i = 0; i < NF; i += 2, ++x) ImageToNormalized2(*x);
        if (NF != N) ImageToNormalized(xs[NF]);
    }
    // inline void ImageToNormalized(Line2D &l) const
    //{
    //	l.c() += cx() * l.a() + cy() * l.b();
    //	l.a() *= fx();
    //	l.b() *= fy();
    //	l.Normalize();
    //}
    inline void ImageToNormalized(const Point2DCovariance &S,
                                  Point2DCovariance &Sn) const
    {
        Sn.sxx() = S.sxx() * fxxI();
        Sn.sxy() = S.sxy() * fxyI();
        Sn.syy() = S.syy() * fyyI();
    }
    inline Point2DCovariance
    GetImageToNormalized(const Point2DCovariance &S) const
    {
        Point2DCovariance Sn;
        ImageToNormalized(S, Sn);
        return Sn;
    }

    inline void ImageOriginCornerToCenter(Point2D &x) const
    {
        x.x() = x.x() - cx();
        x.y() = x.y() - cy();
    }
    inline void ImageOriginCornerToCenter2(_pi__m128 &x) const
    {
        x = _pi_mm_sub_ps(x, m_c);
    }
    inline void ImageOriginCornerToCenterN(AlignedVector<Point2D> &xs) const
    {
        const int N = int(xs.Size()), NF = N - (N & 1);
        _pi__m128 *x = (_pi__m128 *)xs.Data();
        for (int i = 0; i < NF; i += 2, ++x) ImageOriginCornerToCenter2(*x);
        if (NF != N) ImageOriginCornerToCenter(xs[NF]);
    }

    inline void Apply(LA::AlignedVector3f &v) const
    {
        v.x() = fx() * v.x() + cx() * v.z();
        v.y() = fy() * v.y() + cy() * v.z();
    }

    inline void Load(FILE *fp)
    {
        float fx, fy, cx, cy;
        fscanf(fp, "%f %f %f %f", &fx, &fy, &cx, &cy);
        Set(fx, fy, cx, cy);
    }
    inline bool Load(const char *fileName)
    {
        FILE *fp = fopen(fileName, "r");
        if (!fp) return false;
        Load(fp);
        fclose(fp);
        UT::PrintLoaded(fileName);
        return true;
    }
    inline void Save(FILE *fp) const
    {
        fprintf(fp, "%f %f %f %f\n", fx(), fy(), cx(), cy());
    }
    inline bool Save(const char *fileName) const
    {
        FILE *fp = fopen(fileName, "w");
        if (!fp) return false;
        Save(fp);
        fclose(fp);
        UT::PrintSaved(fileName);
        return true;
    }
    inline void SaveActb(FILE *fp) const
    {
        const double k[] = {double(fx()), double(fy()), double(cx()),
                            double(cy()), 0.0,          1.0};
        fwrite(k, sizeof(k), 1, fp);
    }
    inline void LoadActb(FILE *fp)
    {
        double k[6];
        fread(k, sizeof(k), 1, fp);
        Set(float(k[0]), float(k[1]), float(k[2]), float(k[3]));
    }

    inline void Print() const
    {
        UT::Print("%f %f %f %f\n", fx(), fy(), cx(), cy());
    }

  protected:
    inline void Set(const float fx, const float fy, const float cx,
                    const float cy)
    {
        const float fxI = 1.0f / fx, fyI = 1.0f / fy;
        m_f = _pi_mm_setr_ps(fx, fy, fx, fy);
        m_fI = _pi_mm_setr_ps(fxI, fyI, fxI, fyI);
        m_c = _pi_mm_setr_ps(cx, cy, cx, cy);
        m_fx = _pi_mm_set1_ps(fx);
        m_fy = _pi_mm_set1_ps(fy);
        m_cx = _pi_mm_set1_ps(cx);
        m_cy = _pi_mm_set1_ps(cy);
        m_fxI = _pi_mm_set1_ps(fxI);
        m_fyI = _pi_mm_set1_ps(fyI);
        m_fxIcx = _pi_mm_set1_ps(fxI * cx);
        m_fyIcy = _pi_mm_set1_ps(fyI * cy);
        m_fxx = fx * fx;
        m_fxxI = 1.0f / m_fxx;
        m_fxy = fx * fy;
        m_fxyI = 1.0f / m_fxy;
        m_fyy = fy * fy;
        m_fyyI = 1.0f / m_fyy;
    }

  protected:
    Parameter m_k;
    int m_w, m_h;
    short m_needRect, m_fishEye;
    _pi__m128 m_f, m_fI, m_c;
    _pi__m128 m_fx, m_fy, m_cx, m_cy;
    _pi__m128 m_fxI, m_fyI, m_fxIcx, m_fyIcy;
    float m_fxx, m_fxxI, m_fxy, m_fxyI, m_fyy, m_fyyI;
};

#ifdef CFG_DEBUG_EIGEN
class EigenIntrinsic : public Eigen::Matrix3f
{
  public:
    inline EigenIntrinsic() : Eigen::Matrix3f() {}
    inline EigenIntrinsic(const Eigen::Matrix3f &K) : Eigen::Matrix3f(K) {}
    inline EigenIntrinsic(const Intrinsic &K) : Eigen::Matrix3f()
    {
        Eigen::Matrix3f &e_K = *this;
        e_K(0, 0) = K.fx();
        e_K(0, 1) = 0.0f;
        e_K(0, 2) = K.cx();
        e_K(1, 0) = 0.0f;
        e_K(1, 1) = K.fy();
        e_K(1, 2) = K.cy();
        e_K(2, 0) = 0.0f;
        e_K(2, 1) = 0.0f;
        e_K(2, 2) = 1.0f;
    }
    inline void operator=(const Eigen::Matrix3f &e_K)
    {
        *((Eigen::Matrix3f *)this) = e_K;
    }
    inline EigenPoint2D GetNormaliedToImage(const EigenPoint2D &x) const
    {
        const Eigen::Matrix3f &e_K = *this;
        const float e_fx = e_K(0, 0), e_fy = e_K(1, 1), e_cx = e_K(0, 2),
                    e_cy = e_K(1, 2);
        return EigenPoint2D(e_fx * x.x() + e_cx, e_fy * x.y() + e_cy);
    }
    template <typename TYPE>
    inline EigenPoint2D GetImageToNormalized(const TYPE x, const TYPE y) const
    {
        const Eigen::Matrix3f &e_K = *this;
        const float e_fx = e_K(0, 0), e_fy = e_K(1, 1), e_cx = e_K(0, 2),
                    e_cy = e_K(1, 2);
        return EigenPoint2D((x - e_cx) / e_fx, (y - e_cy) / e_fy);
    }
    inline EigenMatrix2x3f GetNormaliedToImage(const EigenMatrix2x3f &J) const
    {
        const Eigen::Matrix3f &e_K = *this;
        const float e_fx = e_K(0, 0), e_fy = e_K(1, 1);
        EigenMatrix2x3f e_KJ;
        e_KJ.block<1, 3>(0, 0) = e_fx * J.block<1, 3>(0, 0);
        e_KJ.block<1, 3>(1, 0) = e_fy * J.block<1, 3>(1, 0);
        return e_KJ;
    }
};
#endif

#endif
