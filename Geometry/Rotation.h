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

#ifndef _ROTATION_H_
#define _ROTATION_H_

#include "stdafx.h"

#include "Matrix3x3.h"
#include "Matrix6x6.h"
#include "Point.h"
#include "Vector4.h"

class AxisAngle : public LA::AlignedVector4f
{
  public:
    inline AxisAngle() {}
    inline AxisAngle(const LA::AlignedVector3f &k, const float th)
    {
        xyzw() = k.xyzr();
        w() = th;
    }

    inline void MakeIdentity()
    {
        xyzw() = _pi_mm_setr_ps(1.0f, 0.0f, 0.0f, 0.0f);
    }

    inline void FromVectors(const LA::AlignedVector3f &v1,
                            const LA::AlignedVector3f &v2)
    {
        SSE::Cross012(v1.xyzr(), v2.xyzr(), xyzw());
        w() = acosf(v1.Dot(v2));
        Normalize();
    }

    inline void SetRodrigues(const LA::AlignedVector3f &w)
    {
        const float th = sqrt(w.SquaredLength());
        if (th < FLT_EPSILON)
            MakeIdentity();
        else {
            this->xyzw() = _pi_mm_mul_ps(w.xyzr(), _pi_mm_set1_ps(1.0f / th));
            this->w() = th;
        }
    }
    inline void GetRodrigues(LA::AlignedVector3f &w) const
    {
        const float th = this->w() < UT_PI ? this->w() : this->w() - UT_2PI;
        w.xyzr() = _pi_mm_mul_ps(this->xyzw(), _pi_mm_set1_ps(th));
    }

    inline void Normalize()
    {
        const float th = w();
        xyzw() = _pi_mm_mul_ps(
            _pi_mm_set1_ps(1.0f /
                           sqrt(SSE::Sum012(_pi_mm_mul_ps(xyzw(), xyzw())))),
            xyzw());
        w() = th;
    }

    inline void Print(const bool e = false) const
    {
        if (e)
            UT::Print("(%e %e %e) %e\n", x(), y(), z(),
                      w() * UT_FACTOR_RAD_TO_DEG);
        else
            UT::Print("(%f %f %f) %f\n", x(), y(), z(),
                      w() * UT_FACTOR_RAD_TO_DEG);
    }

    inline void Random(const float thMax)
    {
        LA::AlignedVector4f::Random(1.0f);
        w() *= thMax;
        Normalize();
    }
    static inline AxisAngle GetRandom(const float thMax)
    {
        AxisAngle kth;
        kth.Random(thMax);
        return kth;
    }

    inline bool AssertEqual(const AxisAngle &kth, const int verbose = 1) const
    {
        if (fabs(w()) < FLT_EPSILON && fabs(kth.w()) < FLT_EPSILON) return true;
        AxisAngle kth1 = *this, kth2 = kth;
        if (kth1.w() > UT_PI) kth1.w() = kth1.w() - UT_2PI;
        if (kth2.w() > UT_PI) kth2.w() = kth2.w() - UT_2PI;
        if (kth1.w() > 0.0f && kth2.w() < 0.0f ||
            kth1.w() < 0.0f && kth2.w() > 0.0f)
            kth2.MakeMinus();
        // return kth1.LA::AlignedVector4f::AssertEqual(kth2, verbose);
        // if(UT::AssertEqual(SSE::Dot012(kth1.xyzw(), kth2.xyzw()), 1.0f) &&
        // UT::AssertEqual(kth1.w(), kth2.w()))
        const float d = SSE::Dot012(kth1.xyzw(), kth2.xyzw());
        // if(UT::AssertEqual(acosf(UT_CLAMP(d, -1.0f, 1.0f)), 0.0f) &&
        // UT::AssertEqual(kth1.w(), kth2.w()))
        const float eps = 0.1f;
        // const float eps = 0.01f;
        if (UT::AssertEqual(acosf(UT_CLAMP(d, -1.0f, 1.0f)), 0.0f, 0,
                            eps * UT_FACTOR_DEG_TO_RAD) &&
            UT::AssertEqual(kth1.w(), kth2.w(), 0))
            return true;
        if (verbose) {
            UT::PrintSeparator();
            kth1.Print(verbose > 1);
            kth2.Print(verbose > 1);
        }
        return false;
    }
};

class Quaternion : public LA::AlignedVector4f
{
  public:
    inline Quaternion() {}
    inline Quaternion(const LA::AlignedVector3f &w) { SetRodrigues(w); }
    inline Quaternion(const LA::AlignedVector4f &q) { xyzw() = q.xyzw(); }
    inline void Get(float *v) const { memcpy(v, this, 16); }
    inline Quaternion operator*(const Quaternion &q) const
    {
        Quaternion _q;
        AB(*this, q, _q);
        return _q;
    }
    inline Quaternion operator/(const Quaternion &q) const
    {
        Quaternion _q;
        ABI(*this, q, _q);
        return _q;
    }

    inline void SetRodrigues(const LA::AlignedVector3f &w)
    {
        const float th2 = w.SquaredLength(), th = sqrt(th2);
        if (th < FLT_EPSILON) {
            const float s = 1.0f / sqrt(th2 + 4.0f);
            this->xyzw() = _pi_mm_mul_ps(w.xyzr(), _pi_mm_set1_ps(s));
            this->w() = s + s;
        } else {
            const float thh = th * 0.5f;
            this->xyzw() =
                _pi_mm_mul_ps(w.xyzr(), _pi_mm_set1_ps(sin(thh) / th));
            this->w() = cos(thh);
        }
    }
    inline void GetRodrigues(LA::AlignedVector3f &w) const
    {
        const float thh = acosf(this->w()), sthh = sin(thh);
        if (sthh < FLT_EPSILON)
            w.MakeZero();
        else
            w.xyzr() = _pi_mm_mul_ps(xyzw(), _pi_mm_set1_ps(thh * 2.0f / sthh));
    }
    inline void SetAxisAngle(const AxisAngle &kth)
    {
        const float thh = kth.w() * 0.5f;
        xyzw() = _pi_mm_mul_ps(kth.xyzw(), _pi_mm_set1_ps(sin(thh)));
        w() = cos(thh);
    }
    inline void GetAxisAngle(AxisAngle &kth) const
    {
        kth.xyzw() = xyzw();
        kth.w() = acosf(w()) * 2.0f;
        kth.Normalize();
    }

    inline void MakeIdentity()
    {
        xyzw() = _pi_mm_setr_ps(0.0f, 0.0f, 0.0f, 1.0f);
    }
    inline void MakeZero() { MakeIdentity(); }
    inline void Normalize()
    {
        const float s = 1.0f / sqrt(SquaredLength());
        Scale(w() > 0.0f ? s : -s);
    }
    inline void Inverse() { w() = -w(); }
    inline Quaternion GetInverse() const
    {
        Quaternion q = *this;
        q.Inverse();
        return q;
    }

    inline void Slerp(const float w1, const Quaternion &q1,
                      const Quaternion &q2)
    {
        x() = q1.Dot(q2);
        if (x() > 0.0f) {
            if (x() > 1.0f) x() = 0.0f;
            // else if(x() < -1.0f)
            //	x() = UT_PI;
            else
                x() = acosf(x());
            if (fabs(x()) < FLT_EPSILON) {
                *this = q1;
                return;
            }
            y() = 1 / sin(x());
            const _pi__m128 s1 = _pi_mm_set1_ps(sin(w1 * x()) * y());
            const _pi__m128 s2 = _pi_mm_set1_ps(sin((1 - w1) * x()) * y());
            xyzw() = _pi_mm_add_ps(_pi_mm_mul_ps(s1, q1.xyzw()),
                                   _pi_mm_mul_ps(s2, q2.xyzw()));
            Normalize();
        } else {
            x() = -x();
            if (x() > 1.0f) x() = 0.0f;
            // else if(x() < -1.0f)
            //	x() = UT_PI;
            else
                x() = acosf(x());
            if (fabs(x()) < FLT_EPSILON) {
                *this = q1;
                return;
            }
            y() = 1 / sin(x());
            const _pi__m128 s1 = _pi_mm_set1_ps(sin(w1 * x()) * y());
            const _pi__m128 s2 = _pi_mm_set1_ps(sin((1 - w1) * x()) * y());
            xyzw() = _pi_mm_sub_ps(_pi_mm_mul_ps(s1, q1.xyzw()),
                                   _pi_mm_mul_ps(s2, q2.xyzw()));
        }
    }
    static inline float GetAngle(const Quaternion &q1, const Quaternion &q2)
    {
        const float d = q1.Dot(q2);
        return acosf(UT_CLAMP(d, -1.0f, 1.0f)) * 2.0f;
    }

    inline void Random(const float thMax)
    {
        AxisAngle kth;
        kth.Random(thMax);
        SetAxisAngle(kth);
    }
    static inline Quaternion GetRandom(const float thMax)
    {
        Quaternion q;
        q.Random(thMax);
        return q;
    }

    inline bool AssertEqual(const Quaternion &q, const int verbose = 1,
                            const float eps = 0.001745329252f) const
    {
        Quaternion q1 = *this, q2 = q;
        if (q1.w() > 0.0f && q2.w() < 0.0f || q1.w() < 0.0f && q2.w() > 0.0f)
            q2.MakeMinus();
        // return q1.LA::AlignedVector4f::AssertEqual(q2, verbose, eps);
        // const float eps = 0.1f * UT_FACTOR_DEG_TO_RAD;
        if (UT::AssertEqual(GetAngle(q1, q2), 0.0f, verbose, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            q.Print(verbose > 1);
        }
        return false;
    }

    static inline void AB(const Quaternion &A, const Quaternion &B,
                          Quaternion &AB)
    {
        _pi__m128 t;
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.x()));
        AB.x() = t.m128_f32[3];
        AB.y() = -t.m128_f32[2];
        AB.z() = t.m128_f32[1];
        AB.w() = -t.m128_f32[0];
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.y()));
        AB.x() = t.m128_f32[2] + AB.x();
        AB.y() = t.m128_f32[3] + AB.y();
        AB.z() = -t.m128_f32[0] + AB.z();
        AB.w() = -t.m128_f32[1] + AB.w();
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.z()));
        AB.x() = -t.m128_f32[1] + AB.x();
        AB.y() = t.m128_f32[0] + AB.y();
        AB.z() = t.m128_f32[3] + AB.z();
        AB.w() = -t.m128_f32[2] + AB.w();
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.w()));
        AB.x() = t.m128_f32[0] + AB.x();
        AB.y() = t.m128_f32[1] + AB.y();
        AB.z() = t.m128_f32[2] + AB.z();
        AB.w() = t.m128_f32[3] + AB.w();
        AB.Normalize();
    }

    static inline void ABI(const Quaternion &A, const Quaternion &B,
                           Quaternion &AB)
    {
        _pi__m128 t;
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.x()));
        AB.x() = t.m128_f32[3];
        AB.y() = -t.m128_f32[2];
        AB.z() = t.m128_f32[1];
        AB.w() = -t.m128_f32[0];
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.y()));
        AB.x() = t.m128_f32[2] + AB.x();
        AB.y() = t.m128_f32[3] + AB.y();
        AB.z() = -t.m128_f32[0] + AB.z();
        AB.w() = -t.m128_f32[1] + AB.w();
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.z()));
        AB.x() = -t.m128_f32[1] + AB.x();
        AB.y() = t.m128_f32[0] + AB.y();
        AB.z() = t.m128_f32[3] + AB.z();
        AB.w() = -t.m128_f32[2] + AB.w();
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(-B.w()));
        AB.x() = t.m128_f32[0] + AB.x();
        AB.y() = t.m128_f32[1] + AB.y();
        AB.z() = t.m128_f32[2] + AB.z();
        AB.w() = t.m128_f32[3] + AB.w();
        AB.Normalize();
    }

    static inline void AIB(const Quaternion &A, const Quaternion &B,
                           Quaternion &AIB)
    {
        _pi__m128 t;
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.x()));
        AIB.x() = -t.m128_f32[3];
        AIB.y() = -t.m128_f32[2];
        AIB.z() = t.m128_f32[1];
        AIB.w() = -t.m128_f32[0];
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.y()));
        AIB.x() = t.m128_f32[2] + AIB.x();
        AIB.y() = -t.m128_f32[3] + AIB.y();
        AIB.z() = -t.m128_f32[0] + AIB.z();
        AIB.w() = -t.m128_f32[1] + AIB.w();
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.z()));
        AIB.x() = -t.m128_f32[1] + AIB.x();
        AIB.y() = t.m128_f32[0] + AIB.y();
        AIB.z() = -t.m128_f32[3] + AIB.z();
        AIB.w() = -t.m128_f32[2] + AIB.w();
        t = _pi_mm_mul_ps(A.xyzw(), _pi_mm_set1_ps(B.w()));
        AIB.x() = t.m128_f32[0] + AIB.x();
        AIB.y() = t.m128_f32[1] + AIB.y();
        AIB.z() = t.m128_f32[2] + AIB.z();
        AIB.w() = -t.m128_f32[3] + AIB.w();
        AIB.Normalize();
    }
};

class SkewSymmetricMatrix : public LA::AlignedVector3f
{
  public:
    inline SkewSymmetricMatrix() {}
    inline SkewSymmetricMatrix(const LA::AlignedVector3f &w)
        : LA::AlignedVector3f(w)
    {
    }

    inline LA::AlignedMatrix3x3f operator*(const LA::AlignedMatrix3x3f &B) const
    {
        LA::AlignedMatrix3x3f _AB;
        AB(*this, B, _AB);
        return _AB;
    }
    inline LA::AlignedVector3f operator*(const LA::AlignedVector3f &B)
    {
        LA::AlignedVector3f AB;
        SkewSymmetricMatrix::AB(*this, B, AB);
        return AB;
    }

    inline float m00() const { return 0.0f; }
    inline float m01() const { return -v2(); }
    inline float m02() const { return v1(); }
    inline float m10() const { return v2(); }
    inline float m11() const { return 0.0f; }
    inline float m12() const { return -v0(); }
    inline float m20() const { return -v1(); }
    inline float m21() const { return v0(); }
    inline float m22() const { return 0.0f; }
    inline LA::AlignedMatrix3x3f GetAlignedMatrix3x3f() const
    {
        LA::AlignedMatrix3x3f M;
        M.m00() = m00();
        M.m01() = m01();
        M.m02() = m02();
        M.m10() = m10();
        M.m11() = m11();
        M.m12() = m12();
        M.m20() = m20();
        M.m21() = m21();
        M.m22() = m22();
        return M;
    }
    inline void GetSquared(LA::SymmetricMatrix3x3f &M) const
    {
        M.SetRow0(_pi_mm_mul_ps(_pi_mm_set1_ps(v0()), v012r()));
        M.m11() = v1() * v1();
        M.m12() = v1() * v2();
        M.m22() = v2() * v2();
        const float l2 = M.m00() + M.m11() + M.m22();
        M.m00() -= l2;
        M.m11() -= l2;
        M.m22() -= l2;
    }

    inline void Transpose() { MakeMinus(); }
    static inline void AB(const SkewSymmetricMatrix &A,
                          const LA::AlignedVector3f &B, LA::AlignedVector3f &AB)
    {
        AB.x() = A.y() * B.z() - A.z() * B.y();
        AB.y() = A.z() * B.x() - A.x() * B.z();
        AB.z() = A.x() * B.y() - A.y() * B.x();
    }
    static inline void AB(const SkewSymmetricMatrix &A,
                          const LA::AlignedMatrix3x3f &B,
                          LA::AlignedMatrix3x3f &AB)
    {
        const _pi__m128 a0 = _pi_mm_set1_ps(A.v0()),
                        a1 = _pi_mm_set1_ps(A.v1()),
                        a2 = _pi_mm_set1_ps(A.v2());
        AB.m_00_01_02_r0() =
            _pi_mm_sub_ps(_pi_mm_mul_ps(a1, B.m_20_21_22_r2()),
                          _pi_mm_mul_ps(a2, B.m_10_11_12_r1()));
        AB.m_10_11_12_r1() =
            _pi_mm_sub_ps(_pi_mm_mul_ps(a2, B.m_00_01_02_r0()),
                          _pi_mm_mul_ps(a0, B.m_20_21_22_r2()));
        AB.m_20_21_22_r2() =
            _pi_mm_sub_ps(_pi_mm_mul_ps(a0, B.m_10_11_12_r1()),
                          _pi_mm_mul_ps(a1, B.m_00_01_02_r0()));
    }
    static inline void AddABTo(const SkewSymmetricMatrix &A,
                               const LA::AlignedMatrix3x3f &B,
                               LA::AlignedMatrix3x3f &AB)
    {
        const _pi__m128 a0 = _pi_mm_set1_ps(A.v0()),
                        a1 = _pi_mm_set1_ps(A.v1()),
                        a2 = _pi_mm_set1_ps(A.v2());
        AB.m_00_01_02_r0() =
            _pi_mm_add_ps(_pi_mm_sub_ps(_pi_mm_mul_ps(a1, B.m_20_21_22_r2()),
                                        _pi_mm_mul_ps(a2, B.m_10_11_12_r1())),
                          AB.m_00_01_02_r0());
        AB.m_10_11_12_r1() =
            _pi_mm_add_ps(_pi_mm_sub_ps(_pi_mm_mul_ps(a2, B.m_00_01_02_r0()),
                                        _pi_mm_mul_ps(a0, B.m_20_21_22_r2())),
                          AB.m_10_11_12_r1());
        AB.m_20_21_22_r2() =
            _pi_mm_add_ps(_pi_mm_sub_ps(_pi_mm_mul_ps(a0, B.m_10_11_12_r1()),
                                        _pi_mm_mul_ps(a1, B.m_00_01_02_r0())),
                          AB.m_20_21_22_r2());
    }
    static inline void AB(const LA::AlignedMatrix3x3f &A,
                          const SkewSymmetricMatrix &B,
                          LA::AlignedMatrix3x3f &AB)
    {
        AB.m00() = A.m01() * B.v2() - A.m02() * B.v1();
        AB.m01() = A.m02() * B.v0() - A.m00() * B.v2();
        AB.m02() = A.m00() * B.v1() - A.m01() * B.v0();
        AB.m10() = A.m11() * B.v2() - A.m12() * B.v1();
        AB.m11() = A.m12() * B.v0() - A.m10() * B.v2();
        AB.m12() = A.m10() * B.v1() - A.m11() * B.v0();
        AB.m20() = A.m21() * B.v2() - A.m22() * B.v1();
        AB.m21() = A.m22() * B.v0() - A.m20() * B.v2();
        AB.m22() = A.m20() * B.v1() - A.m21() * B.v0();
    }
    static inline void AddABTo(const LA::AlignedMatrix3x3f &A,
                               const SkewSymmetricMatrix &B,
                               LA::AlignedMatrix3x3f &AB)
    {
        AB.m00() = A.m01() * B.v2() - A.m02() * B.v1() + AB.m00();
        AB.m01() = A.m02() * B.v0() - A.m00() * B.v2() + AB.m01();
        AB.m02() = A.m00() * B.v1() - A.m01() * B.v0() + AB.m02();
        AB.m10() = A.m11() * B.v2() - A.m12() * B.v1() + AB.m10();
        AB.m11() = A.m12() * B.v0() - A.m10() * B.v2() + AB.m11();
        AB.m12() = A.m10() * B.v1() - A.m11() * B.v0() + AB.m12();
        AB.m20() = A.m21() * B.v2() - A.m22() * B.v1() + AB.m20();
        AB.m21() = A.m22() * B.v0() - A.m20() * B.v2() + AB.m21();
        AB.m22() = A.m20() * B.v1() - A.m21() * B.v0() + AB.m22();
    }
    static inline void ABT(const LA::AlignedMatrix3x3f &A,
                           const SkewSymmetricMatrix &B,
                           LA::AlignedMatrix3x3f &AB)
    {
        AB.m00() = A.m02() * B.v1() - A.m01() * B.v2();
        AB.m01() = A.m00() * B.v2() - A.m02() * B.v0();
        AB.m02() = A.m01() * B.v0() - A.m00() * B.v1();
        AB.m10() = A.m12() * B.v1() - A.m11() * B.v2();
        AB.m11() = A.m10() * B.v2() - A.m12() * B.v0();
        AB.m12() = A.m11() * B.v0() - A.m10() * B.v1();
        AB.m20() = A.m22() * B.v1() - A.m21() * B.v2();
        AB.m21() = A.m20() * B.v2() - A.m22() * B.v0();
        AB.m22() = A.m21() * B.v0() - A.m20() * B.v1();
    }
    static inline void AddABTTo03(const LA::AlignedMatrix3x3f &A,
                                  const SkewSymmetricMatrix &B,
                                  LA::SymmetricMatrix6x6f &AB)
    {
        AB.m03() = A.m02() * B.v1() - A.m01() * B.v2() + AB.m03();
        AB.m04() = A.m00() * B.v2() - A.m02() * B.v0() + AB.m04();
        AB.m05() = A.m01() * B.v0() - A.m00() * B.v1() + AB.m05();
        AB.m13() = A.m12() * B.v1() - A.m11() * B.v2() + AB.m13();
        AB.m14() = A.m10() * B.v2() - A.m12() * B.v0() + AB.m14();
        AB.m15() = A.m11() * B.v0() - A.m10() * B.v1() + AB.m15();
        AB.m23() = A.m22() * B.v1() - A.m21() * B.v2() + AB.m23();
        AB.m24() = A.m20() * B.v2() - A.m22() * B.v0() + AB.m24();
        AB.m25() = A.m21() * B.v0() - A.m20() * B.v1() + AB.m25();
    }
    static inline void AddABTTo33(const LA::AlignedMatrix3x3f &A,
                                  const SkewSymmetricMatrix &B,
                                  LA::SymmetricMatrix6x6f &AB)
    {
        AB.m33() = A.m02() * B.v1() - A.m01() * B.v2() + AB.m33();
        AB.m34() = A.m00() * B.v2() - A.m02() * B.v0() + AB.m34();
        AB.m35() = A.m01() * B.v0() - A.m00() * B.v1() + AB.m35();
        AB.m44() = A.m10() * B.v2() - A.m12() * B.v0() + AB.m44();
        AB.m45() = A.m11() * B.v0() - A.m10() * B.v1() + AB.m45();
        AB.m55() = A.m21() * B.v0() - A.m20() * B.v1() + AB.m55();
    }
    static inline void AddATBTo(const SkewSymmetricMatrix &A,
                                const LA::AlignedVector3f &B,
                                LA::AlignedVector3f &ATB)
    {
        ATB.v0() = A.v2() * B.v1() - A.v1() * B.v2() + ATB.v0();
        ATB.v1() = A.v0() * B.v2() - A.v2() * B.v0() + ATB.v1();
        ATB.v2() = A.v1() * B.v0() - A.v0() * B.v1() + ATB.v2();
    }
};

class Rotation3D : public LA::AlignedMatrix3x3f
{
  public:
    inline Rotation3D() {}
    inline Rotation3D(const LA::AlignedVector3f &w) { SetRodrigues(w); }
    inline Rotation3D(const Quaternion &q) { SetQuaternion(q); }
    inline Rotation3D(const LA::AlignedMatrix3x3f &R)
    {
        memcpy(this, R, sizeof(Rotation3D));
    }

    inline const _pi__m128 &r_00_01_02_x() const { return m_00_01_02_r0(); }
    inline _pi__m128 &r_00_01_02_x() { return m_00_01_02_r0(); }
    inline const _pi__m128 &r_10_11_12_x() const { return m_10_11_12_r1(); }
    inline _pi__m128 &r_10_11_12_x() { return m_10_11_12_r1(); }
    inline const _pi__m128 &r_20_21_22_x() const { return m_20_21_22_r2(); }
    inline _pi__m128 &r_20_21_22_x() { return m_20_21_22_r2(); }
    inline const float &r00() const { return m00(); }
    inline float &r00() { return m00(); }
    inline const float &r01() const { return m01(); }
    inline float &r01() { return m01(); }
    inline const float &r02() const { return m02(); }
    inline float &r02() { return m02(); }
    inline const float &r10() const { return m10(); }
    inline float &r10() { return m10(); }
    inline const float &r11() const { return m11(); }
    inline float &r11() { return m11(); }
    inline const float &r12() const { return m12(); }
    inline float &r12() { return m12(); }
    inline const float &r20() const { return m20(); }
    inline float &r20() { return m20(); }
    inline const float &r21() const { return m21(); }
    inline float &r21() { return m21(); }
    inline const float &r22() const { return m22(); }
    inline float &r22() { return m22(); }
    inline Rotation3D operator/(const Rotation3D &Rb) const
    {
        Rotation3D RaRbT;
        LA::AlignedMatrix3x3f::ABT(*this, Rb, RaRbT);
        return RaRbT;
    }

    inline void MakeOrthogonal()
    {
        Quaternion q;
        GetQuaternion(q);
        SetQuaternion(q);
    }

    inline void SetRodrigues(const LA::AlignedVector3f &w)
    {
        const LA::AlignedVector3f w2 = _pi_mm_mul_ps(w.xyzr(), w.xyzr());
        const float th2 = w2.Sum(), th = sqrt(th2);
        if (th2 < FLT_EPSILON) {
            Quaternion q;
            const float s = 1.0f / sqrt(th2 + 4.0f);
            q.xyzw() = _pi_mm_mul_ps(w.xyzr(), _pi_mm_set1_ps(s));
            q.w() = s + s;
            SetQuaternion(q);
            return;
        }
        const float t1 = sin(th) / th, t2 = (1.0f - cos(th)) / th2,
                    t3 = 1.0f - t2 * th2;
        const LA::AlignedVector3f t1w(
            _pi_mm_mul_ps(_pi_mm_set1_ps(t1), w.xyzr()));
        const LA::AlignedVector3f t2w2(
            _pi_mm_mul_ps(_pi_mm_set1_ps(t2), w2.xyzr()));
        const float t2wx = t2 * w.x(), t2wxy = t2wx * w.y(),
                    t2wxz = t2wx * w.z(), t2wyz = t2 * w.y() * w.z();
        r00() = t3 + t2w2.x();
        r01() = t2wxy + t1w.z();
        r02() = t2wxz - t1w.y();
        r10() = t2wxy - t1w.z();
        r11() = t3 + t2w2.y();
        r12() = t2wyz + t1w.x();
        r20() = t2wxz + t1w.y();
        r21() = t2wyz - t1w.x();
        r22() = t3 + t2w2.z();
    }
    inline void GetRodrigues(LA::AlignedVector3f &w) const
    {
        // AxisAngle kth;
        // GetAxisAngle(kth);
        // kth.GetRodrigues(w);
        const float tr = Trace(), cth = (tr - 1.0f) * 0.5f,
                    th = acosf(UT_CLAMP(cth, -1.0f, 1.0f)), sth = sin(th);
        const float t = fabs(sth) < FLT_EPSILON ? 0.5f : th / (sth * 2.0f);
        w.x() = r12() - r21();
        w.y() = r20() - r02();
        w.z() = r01() - r10();
        w *= t;
    }
    inline LA::AlignedVector3f GetRodrigues() const
    {
        LA::AlignedVector3f w;
        GetRodrigues(w);
        return w;
    }
    static inline void GetRodriguesJacobian(const LA::AlignedVector3f &w,
                                            LA::AlignedMatrix3x3f &Jr)
    {
        const LA::AlignedVector3f w2(_pi_mm_mul_ps(w.xyzr(), w.xyzr()));
        const float th2 = w2.Sum(), th = sqrt(th2);
        if (th2 < FLT_EPSILON) {
            Jr.MakeIdentity();
            return;
        }
        const float th2I = 1.0f / th2, t1 = (1.0f - cos(th)) * th2I,
                    t2 = (1.0f - sin(th) / th) * th2I, t3 = 1.0f - t2 * th2;
        const LA::AlignedVector3f t1w(
            _pi_mm_mul_ps(_pi_mm_set1_ps(t1), w.xyzr()));
        const LA::AlignedVector3f t2w2(
            _pi_mm_mul_ps(_pi_mm_set1_ps(t2), w2.xyzr()));
        const float t2wx = t2 * w.x(), t2wxy = t2wx * w.y(),
                    t2wxz = t2wx * w.z(), t2wyz = t2 * w.y() * w.z();
        Jr.m00() = t3 + t2w2.x();
        Jr.m01() = t2wxy + t1w.z();
        Jr.m02() = t2wxz - t1w.y();
        Jr.m10() = t2wxy - t1w.z();
        Jr.m11() = t3 + t2w2.y();
        Jr.m12() = t2wyz + t1w.x();
        Jr.m20() = t2wxz + t1w.y();
        Jr.m21() = t2wyz - t1w.x();
        Jr.m22() = t3 + t2w2.z();
    }
    static inline void GetRodriguesJacobianInverse(const LA::AlignedVector3f &w,
                                                   LA::AlignedMatrix3x3f &JrI)
    {
        const LA::AlignedVector3f w2(_pi_mm_mul_ps(w.xyzr(), w.xyzr()));
        const float th2 = w2.Sum(), th = sqrt(th2);
        if (th2 < FLT_EPSILON) {
            JrI.MakeIdentity();
            return;
        }
        const float th2I = 1.0f / th2, t1 = -0.5f,
                    t2 = (th2I + (1.0f + cos(th)) / (2.0f * th * sin(th))),
                    t3 = 1.0f - t2 * th2;
        const LA::AlignedVector3f t1w(
            _pi_mm_mul_ps(_pi_mm_set1_ps(t1), w.xyzr()));
        const LA::AlignedVector3f t2w2(
            _pi_mm_mul_ps(_pi_mm_set1_ps(t2), w2.xyzr()));
        const float t2wx = t2 * w.x(), t2wxy = t2wx * w.y(),
                    t2wxz = t2wx * w.z(), t2wyz = t2 * w.y() * w.z();
        JrI.m00() = t3 + t2w2.x();
        JrI.m01() = t2wxy + t1w.z();
        JrI.m02() = t2wxz - t1w.y();
        JrI.m10() = t2wxy - t1w.z();
        JrI.m11() = t3 + t2w2.y();
        JrI.m12() = t2wyz + t1w.x();
        JrI.m20() = t2wxz + t1w.y();
        JrI.m21() = t2wyz - t1w.x();
        JrI.m22() = t3 + t2w2.z();
    }
    static inline LA::AlignedMatrix3x3f
    GetRodriguesJacobianInverse(const LA::AlignedVector3f &w)
    {
        LA::AlignedMatrix3x3f JrI;
        GetRodriguesJacobianInverse(w, JrI);
        return JrI;
    }

    inline void SetAxisAngle(const AxisAngle &kth)
    {
        const float cth = cos(kth.w()), sth2 = 1.0f - cth * cth,
                    sth1 = sqrt(sth2), sth = kth.w() >= 0.0f ? sth1 : -sth1;
        const LA::AlignedVector3f tk =
            _pi_mm_mul_ps(_pi_mm_set1_ps(1 - cth), kth.xyzw());
        const LA::AlignedVector3f tk2 = _pi_mm_mul_ps(tk.xyzr(), kth.xyzw());
        const float tkxy = tk.x() * kth.y(), tkxz = tk.x() * kth.z(),
                    tkyz = tk.y() * kth.z();
        const LA::AlignedVector3f sthk =
            _pi_mm_mul_ps(_pi_mm_set1_ps(sth), kth.xyzw());
        r00() = cth + tk2.x();
        r01() = tkxy + sthk.z();
        r02() = tkxz - sthk.y();
        r10() = tkxy - sthk.z();
        r11() = cth + tk2.y();
        r12() = tkyz + sthk.x();
        r20() = tkxz + sthk.y();
        r21() = tkyz - sthk.x();
        r22() = cth + tk2.z();
    }
    inline void GetAxisAngle(AxisAngle &kth) const
    {
        const float tr = Trace(),
                    cth = (tr - 1.0f) * 0.5f /*, th = acosf(cth)*/;
        // const bool large = th >= UT_PI, small = th <= 0.0f;
        // const bool large = cth <= -1.0f, small = cth >= 1.0f;
        // const float _cth = cos(th);
        // const bool large = _cth <= -1.0f, small = _cth >= 1.0f;
        const bool large =
            cth<FLT_EPSILON - 1.0f, small = FLT_EPSILON + cth> 1.0f;
        if (!large && !small) {
            kth.x() = r12() - r21();
            kth.y() = r20() - r02();
            kth.z() = r01() - r10();
            kth.w() = acosf(cth);
            kth.Normalize();
        } else if (large) {
            if (r00() >= r11() && r00() >= r22()) {
                kth.x() = -sqrt(r00() - r11() - r22() + 1.0f) * 0.5f;
                kth.w() = 0.5f / kth.x();
                kth.y() = r01() * kth.w();
                kth.z() = r02() * kth.w();
            } else if (r11() >= r00() && r11() >= r22()) {
                kth.y() = -sqrt(r11() - r00() - r22() + 1.0f) * 0.5f;
                kth.w() = 0.5f / kth.y();
                kth.x() = r01() * kth.w();
                kth.z() = r12() * kth.w();
            } else if (r22() >= r00() && r22() >= r11()) {
                kth.z() = -sqrt(r22() - r00() - r11() + 1.0f) * 0.5f;
                kth.w() = 0.5f / kth.z();
                kth.x() = r02() * kth.w();
                kth.y() = r12() * kth.w();
            }
            kth.w() = UT_PI;
        } else if (small)
            kth.MakeIdentity();
    }
    inline AxisAngle GetAxisAngle() const
    {
        AxisAngle kth;
        GetAxisAngle(kth);
        return kth;
    }
    static inline float GetAngle(const Rotation3D &R1, const Rotation3D &R2)
    {
        const float tr = SSE::Sum012(_pi_mm_add_ps(
            _pi_mm_mul_ps(R1.r_00_01_02_x(), R2.r_00_01_02_x()),
            _pi_mm_add_ps(
                _pi_mm_mul_ps(R1.r_10_11_12_x(), R2.r_10_11_12_x()),
                _pi_mm_mul_ps(R1.r_20_21_22_x(), R2.r_20_21_22_x()))));
        const float d = (tr - 1.0f) * 0.5f;
        return UT_DOT_TO_ANGLE(d);
    }

    inline void SetQuaternion(const Quaternion &q)
    {
        const _pi__m128 t1 = _pi_mm_mul_ps(_pi_mm_set1_ps(q.x()), q.xyzw());
        const _pi__m128 t2 = _pi_mm_mul_ps(_pi_mm_set1_ps(q.y()), q.xyzw());
        const float qzz = q.z() * q.z(), qzw = q.z() * q.w();
        r00() = t2.m128_f32[1] + qzz;
        r01() = t1.m128_f32[1] + qzw;
        r02() = t1.m128_f32[2] - t2.m128_f32[3];
        r10() = t1.m128_f32[1] - qzw;
        r11() = t1.m128_f32[0] + qzz;
        r12() = t2.m128_f32[2] + t1.m128_f32[3];
        r20() = t1.m128_f32[2] + t2.m128_f32[3];
        r21() = t2.m128_f32[2] - t1.m128_f32[3];
        r22() = t1.m128_f32[0] + t2.m128_f32[1];
        r_00_01_02_x() = _pi_mm_add_ps(r_00_01_02_x(), r_00_01_02_x());
        r00() = -r00() + 1.0f;
        r_10_11_12_x() = _pi_mm_add_ps(r_10_11_12_x(), r_10_11_12_x());
        r11() = -r11() + 1.0f;
        r_20_21_22_x() = _pi_mm_add_ps(r_20_21_22_x(), r_20_21_22_x());
        r22() = -r22() + 1.0f;
    }
    inline void GetQuaternion(Quaternion &q) const
    {
        q.w() = r00() + r11() + r22();
        // if(q.w() > r00() && q.w() > r11() && q.w() > r22())
        if (q.w() > 0.0f) {
            q.w() = sqrt(q.w() + 1) * 0.5f;
            q.z() = 0.25f / q.w();
            q.x() = (r12() - r21()) * q.z();
            q.y() = (r20() - r02()) * q.z();
            q.z() = (r01() - r10()) * q.z();
        } else if (r00() > r11() && r00() > r22()) {
            q.x() = sqrt(r00() + r00() - q.w() + 1) * 0.5f;
            q.w() = 0.25f / q.x();
            q.y() = (r01() + r10()) * q.w();
            q.z() = (r02() + r20()) * q.w();
            q.w() = (r12() - r21()) * q.w();
        } else if (r11() > r22()) {
            q.y() = sqrt(r11() + r11() - q.w() + 1) * 0.5f;
            q.w() = 0.25f / q.y();
            q.x() = (r01() + r10()) * q.w();
            q.z() = (r12() + r21()) * q.w();
            q.w() = (r20() - r02()) * q.w();
        } else {
            q.z() = sqrt(r22() + r22() - q.w() + 1) * 0.5f;
            q.w() = 0.25f / q.z();
            q.x() = (r02() + r20()) * q.w();
            q.y() = (r12() + r21()) * q.w();
            q.w() = (r01() - r10()) * q.w();
        }
        q.Normalize();
    }
    inline Quaternion GetQuaternion() const
    {
        Quaternion q;
        GetQuaternion(q);
        return q;
    }

    inline void SetEulerAngleX(const float th)
    {
        // Rx = [1, 0, 0; 0, cx, -sx; 0, sx, cx]
        // R = Rx^T = [1, 0, 0; 0, cx, sx; 0, -sx, cx]
        MakeIdentity();
        r11() = r22() = cos(th);
        r12() = sin(th);
        r21() = -r12();
    }
    inline void SetEulerAngleY(const float th)
    {
        // Ry = [cy, 0, sy; 0, 1, 0; -sy, 0, cy]
        // R = Ry^T = [cy, 0, -sy; 0, 1, 0; sy, 0, cy]
        MakeIdentity();
        r00() = r22() = cos(th);
        r20() = sin(th);
        r02() = -r20();
    }
    inline void SetEulerAngleZ(const float th)
    {
        // Rz = [cz, -sz, 0; sz, cz, 0; 0, 0, 1]
        // R = Rz^T = [cz, sz, 0; -sz, cz, 0; 0, 0, 1]
        MakeIdentity();
        r00() = r11() = cos(th);
        r01() = sin(th);
        r10() = -r01();
    }
    inline void SetEulerAnglesZXY(const float yaw, const float pitch,
                                  const float roll)
    {
        // R(c->w) = Rz(yaw) * Rx(pitch) * Ry(roll)
        // R = R(c->w)^T
        const float cx = cos(pitch), sx = sin(pitch), cy = cos(roll),
                    sy = sin(roll), cz = cos(yaw), sz = sin(yaw);
        const float cycz = cy * cz, sxsy = sx * sy, cysz = cy * sz;
        // r00() = cycz - sxsy * sz;
        // r10() = -cx * sz;
        // r20() = sy * cz + sx * cysz;
        // r01() = cysz + sxsy * cz;
        // r11() = cx * cz;
        // r21() = sy * sz - sx * cycz;
        // r02() = -cx * sy;
        // r12() = sx;
        // r22() = cx * cy;
        r00() = cycz - sxsy * sz;
        r01() = cysz + sxsy * cz;
        r02() = -cx * sy;
        r10() = -cx * sz;
        r11() = cx * cz;
        r12() = sx;
        r20() = sy * cz + sx * cysz;
        r21() = sy * sz - sx * cycz;
        r22() = cx * cy;
    }
    inline void SetEulerAnglesZYX(const float yaw, const float pitch,
                                  const float roll)
    {
        // R(c->w) = Rz(yaw) * Ry(pitch) * Rx(roll)
        // R = R(c->w)^T
        const float cx = cos(roll), sx = sin(roll), cy = cos(pitch),
                    sy = sin(pitch), cz = cos(yaw), sz = sin(yaw);
        const float sxsy = sx * sy, cxsz = cx * sz, cxcz = cx * cz;
        // r00() = cy * cz;
        // r10() = sxsy * cz - cxsz;
        // r20() = sx * sz + cxcz * sy;
        // r01() = cy * sz;
        // r11() = cxcz + sxsy * sz;
        // r21() = cxsz * sy - sx * cz;
        // r02() = -sy;
        // r12() = sx * cy;
        // r22() = cx * cy;
        r00() = cy * cz;
        r01() = cy * sz;
        r02() = -sy;
        r10() = sxsy * cz - cxsz;
        r11() = cxcz + sxsy * sz;
        r12() = sx * cy;
        r20() = sx * sz + cxcz * sy;
        r21() = cxsz * sy - sx * cz;
        r22() = cx * cy;
    }

    inline void Apply(const LA::AlignedVector3f &X,
                      LA::AlignedVector3f &RX) const
    {
        Apply(X.xyzr(), RX.xyzr());
    }
    inline void Apply(const _pi__m128 &X, _pi__m128 &RX) const
    {
        RX.m128_f32[0] = SSE::Sum012(_pi_mm_mul_ps(r_00_01_02_x(), X));
        RX.m128_f32[1] = SSE::Sum012(_pi_mm_mul_ps(r_10_11_12_x(), X));
        RX.m128_f32[2] = SSE::Sum012(_pi_mm_mul_ps(r_20_21_22_x(), X));
    }
    inline LA::AlignedVector3f GetApplied(const LA::AlignedVector3f &X) const
    {
        LA::AlignedVector3f RX;
        Apply(X, RX);
        return RX;
    }
    inline void ApplyInversely(const LA::AlignedVector3f &X,
                               LA::AlignedVector3f &RTX) const
    {
        RTX.xyzr() = _pi_mm_add_ps(
            _pi_mm_mul_ps(r_00_01_02_x(), _pi_mm_set1_ps(X.x())),
            _pi_mm_add_ps(
                _pi_mm_mul_ps(r_10_11_12_x(), _pi_mm_set1_ps(X.y())),
                _pi_mm_mul_ps(r_20_21_22_x(), _pi_mm_set1_ps(X.z()))));
    }
    inline LA::AlignedVector3f
    GetAppliedInversely(const LA::AlignedVector3f &X) const
    {
        LA::AlignedVector3f RTX;
        ApplyInversely(X, RTX);
        return RTX;
    }
    inline void ApplyInversely(const LA::AlignedVector3f &X, Point3D &RTX) const
    {
        RTX.xyzw() = _pi_mm_add_ps(
            _pi_mm_mul_ps(r_00_01_02_x(), _pi_mm_set1_ps(X.x())),
            _pi_mm_add_ps(
                _pi_mm_mul_ps(r_10_11_12_x(), _pi_mm_set1_ps(X.y())),
                _pi_mm_mul_ps(r_20_21_22_x(), _pi_mm_set1_ps(X.z()))));
        RTX.w() = 1.0f;
    }

    inline float DotOrientation(const Rotation3D &R) const
    {
        const float d = SSE::Dot012(r_20_21_22_x(), R.r_20_21_22_x());
        return UT_CLAMP(d, -1.0f, 1.0f);
    }

    inline void Random(const float thMax)
    {
        AxisAngle kth;
        kth.Random(thMax);
        SetAxisAngle(kth);
    }
    static inline Rotation3D GetRandom(const float thMax)
    {
        Rotation3D R;
        R.Random(thMax);
        return R;
    }

    inline bool AssertEqual(const Rotation3D &R, const int verbose = 1,
                            const float eps = 0.001745329252f) const
    {
        // if(UT::AssertEqual(&r00(), 12, 0, eps))
        //	return true;
        const float dr = Rotation3D::GetAngle(*this, R);
        // const float eps = 0.1f * UT_FACTOR_DEG_TO_RAD;
        if (UT::AssertEqual(dr, 0.0f, 1, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            UT::PrintSeparator();
            R.Print(verbose > 1);
        }
        return false;
    }

    inline bool AssertOrthogonal() const
    {
        if (!UT::AssertEqual(Determinant(), 1.0f)) return false;
        if (!AssertOrthogonalRows()) return false;
        const Rotation3D RT = GetTranspose();
        if (!RT.AssertOrthogonalRows()) return false;
        return true;
    }
    inline bool AssertOrthogonalRows(const float eps = 0.001745329252f) const
    {
        // const float eps = 0.1f * UT_FACTOR_DEG_TO_RAD;
        const float d00 = SSE::Dot012(r_00_01_02_x(), r_00_01_02_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(d00, -1.0f, 1.0f)), 0.0f, 1, eps))
            return false;
        const float d01 = SSE::Dot012(r_00_01_02_x(), r_10_11_12_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(d01, -1.0f, 1.0f)) - UT_PI_2, 0.0f,
                             1, eps))
            return false;
        const float d02 = SSE::Dot012(r_00_01_02_x(), r_20_21_22_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(d02, -1.0f, 1.0f)) - UT_PI_2, 0.0f,
                             1, eps))
            return false;
        const float d11 = SSE::Dot012(r_10_11_12_x(), r_10_11_12_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(d11, -1.0f, 1.0f)), 0.0f, 1, eps))
            return false;
        const float d12 = SSE::Dot012(r_10_11_12_x(), r_20_21_22_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(d12, -1.0f, 1.0f)) - UT_PI_2, 0.0f,
                             1, eps))
            return false;
        const float d22 = SSE::Dot012(r_20_21_22_x(), r_20_21_22_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(d22, -1.0f, 1.0f)), 0.0f, 1, eps))
            return false;

        _pi__m128 c;
        SSE::Cross012(r_10_11_12_x(), r_20_21_22_x(), c);
        const float dc0 = SSE::Dot012(c, r_00_01_02_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(dc0, -1.0f, 1.0f)), 0.0f, 1, eps))
            return false;
        SSE::Cross012(r_20_21_22_x(), r_00_01_02_x(), c);
        const float dc1 = SSE::Dot012(c, r_10_11_12_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(dc1, -1.0f, 1.0f)), 0.0f, 1, eps))
            return false;
        SSE::Cross012(r_00_01_02_x(), r_10_11_12_x(), c);
        const float dc2 = SSE::Dot012(c, r_20_21_22_x());
        if (!UT::AssertEqual(acosf(UT_CLAMP(dc2, -1.0f, 1.0f)), 0.0f, 1, eps))
            return false;
        return true;
    }
};

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Geometry>
class EigenAxisAngle : public Eigen::AngleAxisf
{
  public:
    inline EigenAxisAngle() : Eigen::AngleAxisf() {}
    inline EigenAxisAngle(const Eigen::AngleAxisf &e_th)
        : Eigen::AngleAxisf(e_th)
    {
    }
    inline EigenAxisAngle(const AxisAngle &kth) : Eigen::AngleAxisf()
    {
        Set(EigenVector3f(kth.x(), kth.y(), kth.z()), kth.w());
    }
    inline void operator=(const Eigen::AngleAxisf &e_kth)
    {
        *((Eigen::AngleAxisf *)this) = e_kth;
    }
    inline AxisAngle GetAxisAngle() const
    {
        return AxisAngle(GetAxis().GetAlignedVector3f().GetNormalized(),
                         GetAngle());
    }
    inline void Set(const EigenVector3f &e_k, const float th)
    {
        axis() = e_k;
        angle() = th;
#ifdef CFG_DEBUG
        UT::AssertEqual(e_k.SquaredLength(), 1.0f);
#endif
    }
    inline EigenVector3f GetAxis() const { return EigenVector3f(axis()); }
    inline float GetAngle() const { return angle(); }
    inline void MakeIdentity()
    {
        axis() = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        angle() = 0.0f;
    }
    inline void SetRodrigues(const EigenVector3f &e_w)
    {
        const float th = sqrt(e_w.SquaredLength());
        if (th < FLT_EPSILON)
            MakeIdentity();
        else {
            axis() = e_w / th;
            angle() = th;
        }
#ifdef CFG_DEBUG
        UT::AssertEqual(axis().squaredNorm(), 1.0f);
#endif
    }
    inline void GetRodrigues(EigenVector3f &e_w) const
    {
        const float e_th = angle() < UT_PI ? angle() : angle() - UT_2PI;
        e_w = axis() * e_th;
    }
    inline EigenVector3f GetRodrigues() const
    {
        EigenVector3f e_w;
        GetRodrigues(e_w);
        return e_w;
    }
    inline void Print(const bool e = false) const { GetAxisAngle().Print(e); }
    static inline EigenAxisAngle GetRandom(const float thMax)
    {
        return EigenAxisAngle(AxisAngle::GetRandom(thMax));
    }
    inline bool AssertEqual(const EigenAxisAngle &e_kth,
                            const int verbose = 1) const
    {
        return GetAxisAngle().AssertEqual(e_kth.GetAxisAngle(), verbose);
    }
};
class EigenQuaternion : public EigenVector4f
{
  public:
    inline EigenQuaternion() : EigenVector4f() {}
    inline EigenQuaternion(const EigenVector4f &e_q) : EigenVector4f(e_q) {}
    inline EigenQuaternion(const Quaternion &q)
        : EigenVector4f(q.x(), q.y(), q.z(), q.w())
    {
    }
    inline Quaternion GetQuaternion() const { return GetAlignedVector4f(); }
    inline Eigen::Quaternionf GetEigenQuaternion() const
    {
        Eigen::Quaternionf e_q;
        e_q.x() = x();
        e_q.y() = y();
        e_q.z() = z();
        e_q.w() = w();
        return e_q;
    }
    inline void MakeIdentity()
    {
        x() = 0.0f;
        y() = 0.0f;
        z() = 0.0f;
        w() = 1.0f;
    }
    inline void Normalize()
    {
        const float s = 1.0f / ::sqrt(SquaredLength());
        (*this) *= w() > 0.0f ? s : -s;
    }
    inline void SetRodrigues(const EigenVector3f &e_w)
    {
        const float th = ::sqrt(e_w.SquaredLength());
        if (th < FLT_EPSILON) {
            block<3, 1>(0, 0) = e_w * 0.5f;
            w() = 1.0f;
            Normalize();
        } else {
            EigenAxisAngle e_kth;
            e_kth.SetRodrigues(e_w);
            SetAxisAngle(e_kth);
        }
    }
    inline void GetRodrigues(EigenVector3f &e_w) const
    {
        EigenAxisAngle e_kth;
        GetAxisAngle(e_kth);
        e_kth.GetRodrigues(e_w);
    }
    inline void SetAxisAngle(const EigenAxisAngle &e_kth)
    {
        const EigenVector3f e_k = e_kth.GetAxis();
#ifdef CFG_DEBUG
        UT::AssertEqual(e_k.SquaredLength(), 1.0f);
#endif
        const float thh = e_kth.GetAngle() * 0.5f, sthh = ::sin(thh),
                    cthh = ::cos(thh);
        x() = e_k.x() * sthh;
        y() = e_k.y() * sthh;
        z() = e_k.z() * sthh;
        w() = cthh;
    }
    inline void GetAxisAngle(EigenAxisAngle &e_kth) const
    {
        const EigenVector3f e_k(x(), y(), z());
        e_kth.Set(EigenVector3f(e_k / ::sqrt(e_k.squaredNorm())),
                  ::acosf(w()) * 2.0f);
    }
    inline bool AssertEqual(const Quaternion &q, const int verbose = 1) const
    {
        return GetQuaternion().AssertEqual(q, verbose);
    }
    inline bool AssertEqual(const EigenQuaternion &e_q,
                            const int verbose = 1) const
    {
        return GetQuaternion().AssertEqual(e_q.GetQuaternion(), verbose);
    }
};
class EigenRotation3D : public EigenMatrix3x3f
{
  public:
    inline EigenRotation3D() : EigenMatrix3x3f() {}
    inline EigenRotation3D(const Eigen::Matrix3f &e_R) : EigenMatrix3x3f(e_R) {}
    inline EigenRotation3D(const EigenMatrix3x3f &e_R) : EigenMatrix3x3f(e_R) {}
    inline EigenRotation3D(const EigenVector3f &e_w) { SetRodrigues(e_w); }
    inline EigenRotation3D(const EigenAxisAngle &e_kth) { SetAxisAngle(e_kth); }
    inline EigenRotation3D(const EigenQuaternion &e_q) { SetQuaternion(e_q); }
    inline EigenRotation3D(const Rotation3D &R) : EigenMatrix3x3f(R) {}
    inline void SetRodrigues(const EigenVector3f &e_w)
    {
#if 0
		EigenAxisAngle e_kth;
		e_kth.SetRodrigues(e_w);
		SetAxisAngle(e_kth);
#else
        Rotation3D R;
        R.SetRodrigues(e_w.GetAlignedVector3f());
        *this = R;
#endif
    }
    inline void GetRodrigues(EigenVector3f &e_w) const
    {
#if 0
		EigenAxisAngle e_kth;
		GetAxisAngle(e_kth);
		e_kth.GetRodrigues(e_w);
#else
        const Rotation3D R = GetAlignedMatrix3x3f();
        e_w = R.GetRodrigues();
#endif
    }
    inline EigenVector3f GetRodrigues() const
    {
        EigenVector3f e_w;
        GetRodrigues(e_w);
        return e_w;
    }
    inline void operator=(const Eigen::Matrix3f &e_R)
    {
        *((EigenMatrix3x3f *)this) = e_R;
    }
    inline void SetAxisAngle(const EigenAxisAngle &e_kth)
    {
        *this = e_kth.toRotationMatrix().transpose();
    }
    inline void GetAxisAngle(EigenAxisAngle &e_kth) const
    {
        e_kth.fromRotationMatrix(transpose());
    }
    inline void SetQuaternion(const EigenQuaternion &e_q)
    {
        *this = e_q.GetEigenQuaternion().toRotationMatrix().transpose();
    }
    inline void GetQuaternion(EigenQuaternion &e_q) const
    {
        EigenAxisAngle e_kth;
        e_kth.fromRotationMatrix(transpose());
        e_q.SetAxisAngle(e_kth);
    }
};
class EigenSkewSymmetricMatrix : public EigenMatrix3x3f
{
  public:
    inline EigenSkewSymmetricMatrix() : EigenMatrix3x3f() {}
    inline EigenSkewSymmetricMatrix(const Eigen::Vector3f &e_w)
        : EigenMatrix3x3f()
    {
        Set(EigenVector3f(e_w).GetAlignedVector3f());
    }
    inline EigenSkewSymmetricMatrix(const SkewSymmetricMatrix &M)
        : EigenMatrix3x3f()
    {
        Set(M);
    }
    inline void Set(const LA::AlignedVector3f &w)
    {
        Eigen::Matrix3f &e_S = *this;
        e_S(0, 0) = 0.0f;
        e_S(0, 1) = -w.z();
        e_S(0, 2) = w.y();
        e_S(1, 0) = w.z();
        e_S(1, 1) = 0.0f;
        e_S(1, 2) = -w.x();
        e_S(2, 0) = -w.y();
        e_S(2, 1) = w.x();
        e_S(2, 2) = 0.0f;
    }
};
#endif
#endif
