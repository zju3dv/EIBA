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

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "stdafx.h"

#include "Rigid.h"

#ifdef CFG_TUNE_PARAMETERS
extern float CAMERA_EPSILON_BIAS_GYROSCOPE;
#include "Configurator.h"
extern void LOAD_PARAMETERS_CAMERA(const Configurator &cfgor);
#else
#define CAMERA_EPSILON_BIAS_GYROSCOPE 3.046174198662e-4f // (1.0*pi/180)^2
#endif

#define m_A fe.m_A
#define m_b fe.m_b

class Camera
{
  public:
    class Pose
    {
      public:
        class Unitary
        {
          public:
            inline Unitary() {}
            inline Unitary(const LA::SymmetricMatrix6x6f &A,
                           const LA::Vector6f &b)
                : fe(A, b)
            {
            }
            inline void operator+=(const Unitary &A)
            {
                m_data[0] = _pi_mm_add_ps(A.m_data[0], m_data[0]);
                m_data[1] = _pi_mm_add_ps(A.m_data[1], m_data[1]);
                m_data[2] = _pi_mm_add_ps(A.m_data[2], m_data[2]);
                m_data[3] = _pi_mm_add_ps(A.m_data[3], m_data[3]);
                m_data[4] = _pi_mm_add_ps(A.m_data[4], m_data[4]);
                m_data[5] = _pi_mm_add_ps(A.m_data[5], m_data[5]);
                m_data[6] = _pi_mm_add_ps(A.m_data[6], m_data[6]);
            }
            inline void operator-=(const Unitary &A)
            {
                m_data[0] = _pi_mm_sub_ps(m_data[0], A.m_data[0]);
                m_data[1] = _pi_mm_sub_ps(m_data[1], A.m_data[1]);
                m_data[2] = _pi_mm_sub_ps(m_data[2], A.m_data[2]);
                m_data[3] = _pi_mm_sub_ps(m_data[3], A.m_data[3]);
                m_data[4] = _pi_mm_sub_ps(m_data[4], A.m_data[4]);
                m_data[5] = _pi_mm_sub_ps(m_data[5], A.m_data[5]);
                m_data[6] = _pi_mm_sub_ps(m_data[6], A.m_data[6]);
            }
            inline Unitary operator-(const Unitary &B) const
            {
                Unitary AmB;
                AmB.m_data[0] = _pi_mm_sub_ps(m_data[0], B.m_data[0]);
                AmB.m_data[1] = _pi_mm_sub_ps(m_data[1], B.m_data[1]);
                AmB.m_data[2] = _pi_mm_sub_ps(m_data[2], B.m_data[2]);
                AmB.m_data[3] = _pi_mm_sub_ps(m_data[3], B.m_data[3]);
                AmB.m_data[4] = _pi_mm_sub_ps(m_data[4], B.m_data[4]);
                AmB.m_data[5] = _pi_mm_sub_ps(m_data[5], B.m_data[5]);
                AmB.m_data[6] = _pi_mm_sub_ps(m_data[6], B.m_data[6]);
                return AmB;
            }
            inline Unitary operator*(const _pi__m128 &s) const
            {
                Unitary A;
                GetScaled(s, A);
                return A;
            }
            inline void MakeZero() { memset(this, 0, sizeof(Unitary)); }
            inline void GetScaled(const _pi__m128 &s, Unitary &A) const
            {
                A.m_data[0] = _pi_mm_mul_ps(s, m_data[0]);
                A.m_data[1] = _pi_mm_mul_ps(s, m_data[1]);
                A.m_data[2] = _pi_mm_mul_ps(s, m_data[2]);
                A.m_data[3] = _pi_mm_mul_ps(s, m_data[3]);
                A.m_data[4] = _pi_mm_mul_ps(s, m_data[4]);
                A.m_data[5] = _pi_mm_mul_ps(s, m_data[5]);
                A.m_data[6] = _pi_mm_mul_ps(s, m_data[6]);
            }
            inline void Print(const bool e = false) const
            {
                m_A.Print(e);
                m_b.Print(e);
            }
            inline bool AssertEqual(const Unitary &A, const int verbose = 1,
                                    const float epsAbs = 0.0f,
                                    const float epsRel = 0.0f) const
            {
                // if(UT::VectorAssertEqual((float *) m_data, (float *)
                // A.m_data, 27, 0, eps))
                //	return true;

                LA::AlignedMatrix3x3f App1, Apr1, Arr1, App2, Apr2, Arr2;
                const LA::AlignedMatrix6x6f A1 = m_A.GetAlignedMatrix6x6f(),
                                            A2 = A.m_A.GetAlignedMatrix6x6f();
                A1.GetBlock(0, 0, App1);
                A1.GetBlock(0, 3, Apr1);
                A1.GetBlock(3, 3, Arr1);
                A2.GetBlock(0, 0, App2);
                A2.GetBlock(0, 3, Apr2);
                A2.GetBlock(3, 3, Arr2);
                const float ap = std::sqrt(std::min(
                    App1.SquaredFrobeniusNorm(), App2.SquaredFrobeniusNorm()));
                const float ar = std::sqrt(std::min(
                    Arr1.SquaredFrobeniusNorm(), Arr2.SquaredFrobeniusNorm()));

                LA::AlignedVector3f bp1, br1, bp2, br2;
                const LA::AlignedVector6f b1 = (const float *)(m_b),
                                          b2 = (const float *)(A.m_b);
                b1.GetBlock(0, bp1);
                b1.GetBlock(3, br1);
                b2.GetBlock(0, bp2);
                b2.GetBlock(3, br2);
                const float bp = std::sqrt(
                    std::min(bp1.SquaredLength(), bp2.SquaredLength()));
                const float br = std::sqrt(
                    std::min(br1.SquaredLength(), br2.SquaredLength()));

                if (App1.AssertEqual(App2, 0, std::max(epsAbs, ap * epsRel)) &&
                    Apr1.AssertEqual(
                        Apr2, 0,
                        std::max(epsAbs, std::sqrt(ap * ar) * epsRel)) &&
                    Arr1.AssertEqual(Arr2, 0, std::max(epsAbs, ar * epsRel)) &&
                    bp1.AssertEqual(bp2, 0, std::max(epsAbs, bp * epsRel)) &&
                    br1.AssertEqual(br2, 0, std::max(epsAbs, br * epsRel)))
                    return true;

                if (verbose) {
                    UT::PrintSeparator();
                    Print(verbose > 1);
                    UT::PrintSeparator();
                    A.Print(verbose > 1);
                    const Unitary E = *this - A;
                    UT::PrintSeparator();
                    E.Print(verbose > 1);
                }
                return false;
            }

          public:
#undef m_A
#undef m_b
            struct CanNotMakeAnonymous {
                CanNotMakeAnonymous() = default;
                CanNotMakeAnonymous(const LA::SymmetricMatrix6x6f &A,
                                    const LA::Vector6f b)
                    : m_A(A), m_b(b)
                {
                }
                LA::SymmetricMatrix6x6f m_A;
                LA::Vector6f m_b;
            };
#define m_A fe.m_A
#define m_b fe.m_b
            union {
                CanNotMakeAnonymous fe;
                _pi__m128 m_data[7];
            };
        };
        class Binary : public LA::AlignedMatrix6x6f
        {
          public:
            inline Binary() {}
            inline Binary(const LA::AlignedMatrix6x6f &A)
                : LA::AlignedMatrix6x6f(A)
            {
            }
            inline bool AssertEqual(const Binary &A, const int verbose = 1,
                                    const float epsAbs = 0.0f,
                                    const float epsRel = 0.0f) const
            {
                LA::AlignedMatrix3x3f App1, Apr1, Arp1, Arr1, App2, Apr2, Arp2,
                    Arr2;
                const LA::AlignedMatrix6x6f &A1 = *this, &A2 = A;
                A1.GetBlock(0, 0, App1);
                A1.GetBlock(0, 3, Apr1);
                A1.GetBlock(3, 0, Arp1);
                A1.GetBlock(3, 3, Arr1);
                A2.GetBlock(0, 0, App2);
                A2.GetBlock(0, 3, Apr2);
                A2.GetBlock(3, 0, Arp2);
                A2.GetBlock(3, 3, Arr2);
                const float ap = std::sqrt(std::min(
                    App1.SquaredFrobeniusNorm(), App2.SquaredFrobeniusNorm()));
                const float ar = std::sqrt(std::min(
                    Arr1.SquaredFrobeniusNorm(), Arr2.SquaredFrobeniusNorm()));
                if (App1.AssertEqual(App2, 0, std::max(epsAbs, ap * epsRel)) &&
                    Apr1.AssertEqual(
                        Apr2, 0,
                        std::max(epsAbs, std::sqrt(ap * ar) * epsRel)) &&
                    Arp1.AssertEqual(
                        Arp2, 0,
                        std::max(epsAbs, std::sqrt(ap * ar) * epsRel)) &&
                    Arr1.AssertEqual(Arr2, 0, std::max(epsAbs, ar * epsRel)))
                    return true;
                if (verbose) {
                    UT::PrintSeparator();
                    Print(verbose > 1);
                    UT::PrintSeparator();
                    A.Print(verbose > 1);
                }
                return false;
            }
        };

        class Prior
        {
          public:
            class Rotation
            {
              public:
                inline Rotation() {}
                inline void MakeZero() { memset(this, 0, sizeof(Rotation)); }
                inline void MakeIdentity() { m_R.MakeIdentity(); }
                inline bool Valid() const { return m_R.Valid(); }
                inline bool Invalid() const { return m_R.Invalid(); }
                inline void Invalidate() { m_R.Invalidate(); }
              public:
                Rotation3D m_R;
                LA::AlignedMatrix3x3f m_Wrr;
            };
            class Rigid : public Rotation
            {
              public:
                class Error
                {
                  public:
                    inline Error() {}
                    inline Error(const LA::AlignedVector3f &er,
                                 const LA::AlignedVector3f &ep)
                        : m_er(er), m_ep(ep)
                    {
                    }

                  public:
                    LA::AlignedVector3f m_er, m_ep;
                };
                class Jacobian2
                {
                  public:
                    LA::AlignedMatrix3x3f m_JrrT, m_JppT;
                };
                class Jacobian1 : public Jacobian2
                {
                  public:
                    SkewSymmetricMatrix m_JprT;
                };
                class WeightedJacobian
                {
                  public:
                    static inline void ATB(const WeightedJacobian &WJ,
                                           const Jacobian2 &J, const Error &e,
                                           Unitary &A)
                    {
                        LA::SymmetricMatrix6x6f::ABTTo00(WJ.m_WJppT, J.m_JppT,
                                                         A.m_A);
                        LA::SymmetricMatrix6x6f::ABTTo03(WJ.m_WJrpT, J.m_JrrT,
                                                         A.m_A);
                        LA::SymmetricMatrix6x6f::AbTo0(WJ.m_WJrpT, e.m_er,
                                                       A.m_b);
                        LA::SymmetricMatrix6x6f::AddAbTo0(WJ.m_WJppT, e.m_ep,
                                                          A.m_b);
                        LA::SymmetricMatrix6x6f::ABTTo33(WJ.m_WJrrT, J.m_JrrT,
                                                         A.m_A);
                        LA::SymmetricMatrix6x6f::AbTo3(WJ.m_WJrrT, e.m_er,
                                                       A.m_b);
                        LA::SymmetricMatrix6x6f::AddAbTo3(WJ.m_WJprT, e.m_ep,
                                                          A.m_b);
                    }
                    static inline void ATB(const WeightedJacobian &WJ1,
                                           const WeightedJacobian &WJ2,
                                           const Jacobian1 &J1,
                                           const Jacobian2 &J2, const Error &e,
                                           Unitary &A11, Binary &A12,
                                           Unitary &A22)
                    {
                        ATB(WJ1, J1, e, A11);
                        ATB(WJ2, J2, e, A22);
                        SkewSymmetricMatrix::AddABTTo03(WJ1.m_WJppT, J1.m_JprT,
                                                        A11.m_A);
                        SkewSymmetricMatrix::AddABTTo33(WJ1.m_WJprT, J1.m_JprT,
                                                        A11.m_A);
                        LA::AlignedMatrix6x6f::ABTTo00(WJ1.m_WJppT, J2.m_JppT,
                                                       A12);
                        LA::AlignedMatrix6x6f::ABTTo03(WJ1.m_WJrpT, J2.m_JrrT,
                                                       A12);
                        LA::AlignedMatrix6x6f::ABTTo30(WJ1.m_WJprT, J2.m_JppT,
                                                       A12);
                        LA::AlignedMatrix6x6f::ABTTo33(WJ1.m_WJrrT, J2.m_JrrT,
                                                       A12);
                    }

                  public:
                    LA::AlignedMatrix3x3f m_WJrpT, m_WJrrT, m_WJppT, m_WJprT;
                };
                class ESError : public Error
                {
                  public:
                    inline ESError() {}
                    inline ESError(const Error &e)
                    {
                        e.m_er.GetScaled(UT_FACTOR_RAD_TO_DEG, m_er);
                        m_ep = e.m_ep;
                    }
                    inline float SquaredLength() const { return -1.0f; }
                    inline void Print(const bool l = true) const
                    {
                        const float er = std::sqrt(m_er.SquaredLength()),
                                    ep = std::sqrt(m_ep.SquaredLength());
                        if (l)
                            UT::Print("%f + %f", er, ep);
                        else
                            UT::Print("%.3f + %.3f", er, ep);
                    }
                };
                class ESIndex
                {
                  public:
                    inline ESIndex() : m_iFrm1(-1), m_iFrm2(-1) {}
                    inline ESIndex(const int iFrm1, const int iFrm2 = -1)
                        : m_iFrm1(iFrm1), m_iFrm2(iFrm2)
                    {
                    }
                    inline void Print() const
                    {
                        if (m_iFrm1 != -1) UT::Print(" [%d]", m_iFrm1);
                        if (m_iFrm2 != -1) UT::Print(" [%d]", m_iFrm2);
                    }

                  public:
                    int m_iFrm1, m_iFrm2;
                };
                class ES : public UT::ES<ESError, ESIndex>
                {
                  public:
                    inline void Initialize()
                    {
                        UT::ES<ESError, ESIndex>::Initialize();
                        m_eMax.m_er.MakeZero();
                        m_eMax.m_ep.MakeZero();
                    }
                    inline void Accumulate(const Error &e, const float we2,
                                           const ESIndex &idx)
                    {
                        UT::ES<ESError, ESIndex>::Accumulate(ESError(e), we2,
                                                             idx);
                    }
                    inline void Print(const std::string str = "",
                                      const bool r = true) const
                    {
                        // if(Valid())
                        UT::ES<ESError, ESIndex>::Print(str + "ep = ", false);
                    }
                };

              public:
                inline Rigid() {}
                inline Rigid(const Rigid &T) { *this = T; }
                inline void operator=(const Rigid &T)
                {
                    memcpy(this, &T, sizeof(Rigid));
                }
                inline void MakeZero() { memset(this, 0, sizeof(Rigid)); }
                inline void GetWeighted(const float w, Rigid &T) const
                {
                    const _pi__m128 _w = _pi_mm_set1_ps(w);
                    GetWeighted(_w, T);
                }
                inline void GetWeighted(const _pi__m128 &w, Rigid &T) const
                {
                    T.m_R = m_R;
                    T.m_p = m_p;
                    m_Wrr.GetScaled(w, T.m_Wrr);
                    m_Wrp.GetScaled(w, T.m_Wrp);
                    T.m_Wrp.GetTranspose(T.m_Wpr);
                    m_Wpp.GetScaled(w, T.m_Wpp);
                }
                static inline Rotation3D GetRotationState(const Rotation3D &R1,
                                                          const Rotation3D &R2)
                {
                    return R2 / R1;
                }
                static inline Rotation3D GetRotationState(const Camera &C1,
                                                          const Camera &C2)
                {
                    return Rotation3D(C2.m_T) / C1.m_T;
                }
                inline const Rotation3D &GetRotationMeasurement() const
                {
                    return m_R;
                }
                inline LA::AlignedVector3f
                GetRotationError(const Rotation3D &R1,
                                 const Rotation3D &R2) const
                {
                    const Rotation3D eR =
                        GetRotationState(R1, R2) / GetRotationMeasurement();
                    return eR.GetRodrigues();
                }
                inline LA::AlignedVector3f
                GetRotationError(const Camera &C1, const Camera &C2) const
                {
                    return GetRotationError(C1.m_T, C2.m_T);
                }
                inline void GetRotationErrorJacobian(
                    const Rotation3D &R1, const Rotation3D &R2,
                    LA::AlignedVector3f &e, LA::AlignedMatrix3x3f &Jr2) const
                {
                    e = GetRotationError(R1, R2);
                    Rotation3D::GetRodriguesJacobianInverse(e, Jr2);
                }
                inline void
                GetRotationErrorJacobian(const Camera &C1, const Camera &C2,
                                         LA::AlignedVector3f &e,
                                         LA::AlignedMatrix3x3f &Jr2) const
                {
                    GetRotationErrorJacobian(C1.m_T, C2.m_T, e, Jr2);
                }
                inline void GetRotationErrorJacobian(
                    const Rotation3D &R1, const Rotation3D &R2,
                    LA::AlignedVector3f &e, LA::AlignedMatrix3x3f &Jr1,
                    LA::AlignedMatrix3x3f &Jr2) const
                {
                    const Rotation3D dR = GetRotationState(R1, R2),
                                     eR = dR / m_R;
                    eR.GetRodrigues(e);
                    Rotation3D::GetRodriguesJacobianInverse(e, Jr2);
                    Jr1 = Jr2 * dR;
                    Jr1.MakeMinus();
                }
                inline void
                GetRotationErrorJacobian(const Camera &C1, const Camera &C2,
                                         LA::AlignedVector3f &e,
                                         LA::AlignedMatrix3x3f &Jr1,
                                         LA::AlignedMatrix3x3f &Jr2) const
                {
                    GetRotationErrorJacobian(C1.m_T, C2.m_T, e, Jr1, Jr2);
                }

                static inline LA::AlignedVector3f
                GetPositionState(const Rigid3D &T1, const Point3D &p2)
                {
                    return T1.GetAppliedRotation(p2) + T1.GetTranslation();
                }
                static inline LA::AlignedVector3f
                GetPositionState(const Camera &C1, const Camera &C2)
                {
                    return C1.m_T.GetAppliedRotation(
                        LA::AlignedVector3f(C2.m_p) - C1.m_p);
                }
                inline const LA::AlignedVector3f &GetPositionMeasurement() const
                {
                    return m_p;
                }
                inline LA::AlignedVector3f
                GetPositionMeasurement(const LA::AlignedVector3f &xdp) const
                {
                    const float z2 = GetPositionMeasurementScale(xdp);
                    return GetPositionMeasurement(z2);
                }
                inline LA::AlignedVector3f
                GetPositionMeasurement(const float z2) const
                {
                    if (z2 == 1.0f)
                        return m_p;
                    else
                        return m_p * z2;
                }
                inline float GetPositionMeasurementScale(
                    const LA::AlignedVector3f &xdp) const
                {
                    const float a = m_p.SquaredLength();
                    if (a < FLT_EPSILON)
                        return 1.0f;
                    else
                        return m_p.Dot(xdp) / a;
                }
                inline LA::AlignedVector3f
                GetPositionError(const Rigid3D &T1, const Point3D &p2) const
                {
                    return GetPositionState(T1, p2) - GetPositionMeasurement();
                }
                inline LA::AlignedVector3f
                GetPositionError(const Camera &C1, const Camera &C2) const
                {
                    return GetPositionState(C1, C2) - GetPositionMeasurement();
                }
                inline void
                GetPositionErrorJacobian(const Rigid3D &T1, const Point3D &p2,
                                         LA::AlignedVector3f &e,
                                         LA::AlignedMatrix3x3f &Jp2) const
                {
                    e = GetPositionError(T1, p2);
                    Jp2 = T1;
                }
                inline void
                GetPositionErrorJacobian(const Camera &C1, const Camera &C2,
                                         LA::AlignedVector3f &e,
                                         LA::AlignedMatrix3x3f &Jp2) const
                {
                    e = GetPositionError(C1, C2);
                    Jp2 = C1.m_T;
                }
                inline void
                GetPositionErrorJacobian(const Rigid3D &T1, const Point3D &p2,
                                         LA::AlignedVector3f &e,
                                         SkewSymmetricMatrix &Jr1,
                                         LA::AlignedMatrix3x3f &Jp1,
                                         LA::AlignedMatrix3x3f &Jp2) const
                {
                    Jr1 = GetPositionState(T1, p2);
                    e = Jr1 - GetPositionMeasurement();
                    Jp2 = T1;
                    Jp2.GetMinus(Jp1);
                }
                inline Error GetError(const Rigid3D &T1, const Rotation3D &R2,
                                      const Point3D &p2) const
                {
                    Error e;
                    e.m_er = GetRotationError(T1, R2);
                    e.m_ep = GetPositionError(T1, p2);
                    return e;
                }
                inline void GetErrorJacobian(const Rigid3D &T1,
                                             const Rotation3D &R2,
                                             const Point3D &p2, Error &e,
                                             Jacobian1 &J1, Jacobian2 &J2) const
                {
                    GetRotationErrorJacobian(T1, R2, e.m_er, J1.m_JrrT,
                                             J2.m_JrrT);
                    J1.m_JrrT.Transpose();
                    J2.m_JrrT.Transpose();
                    GetPositionErrorJacobian(T1, p2, e.m_ep, J1.m_JprT,
                                             J1.m_JppT, J2.m_JppT);
                    J1.m_JprT.Transpose();
                    J1.m_JppT.Transpose();
                    J2.m_JppT.Transpose();
                }
                inline void GetWeightedJacobian(const Jacobian2 &J,
                                                WeightedJacobian &WJ) const
                {
                    LA::AlignedMatrix3x3f::ABT(J.m_JppT, m_Wrp, WJ.m_WJrpT);
                    LA::AlignedMatrix3x3f::ABT(J.m_JrrT, m_Wrr, WJ.m_WJrrT);
                    LA::AlignedMatrix3x3f::ABT(J.m_JppT, m_Wpp, WJ.m_WJppT);
                    LA::AlignedMatrix3x3f::ABT(J.m_JrrT, m_Wpr, WJ.m_WJprT);
                }
                inline void GetWeightedJacobian(const Jacobian1 &J1,
                                                const Jacobian2 &J2,
                                                WeightedJacobian &WJ1,
                                                WeightedJacobian &WJ2) const
                {
                    GetWeightedJacobian(J1, WJ1);
                    GetWeightedJacobian(J2, WJ2);
                    SkewSymmetricMatrix::AddABTo(J1.m_JprT, m_Wpr, WJ1.m_WJrrT);
                    SkewSymmetricMatrix::AddABTo(J1.m_JprT, m_Wpp, WJ1.m_WJprT);
                }

                inline void Print(const bool e = false,
                                  const bool w = true) const
                {
                    UT::PrintSeparator();
                    m_R.Print("dR  = ", e);
                    m_p.Print("dp  = ", e);
                    if (w) {
                        m_Wrr.Print("Wrr = ", e);
                        m_Wrp.Print("Wrp = ", e);
                        m_Wpp.Print("Wpp = ", e);
                    }
                }
                inline void AssertConsistency() const
                {
                    m_Wrr.AssertSymmetric();
                    m_Wrp.AssertSymmetric(m_Wpr);
                    m_Wpp.AssertSymmetric();
                }

              public:
                LA::AlignedVector3f m_p;
                LA::AlignedMatrix3x3f m_Wrp, m_Wpr, m_Wpp;
            };
        };
    };

  public:
    inline Camera() {}
    inline Camera(const Camera &C) { *this = C; }
    inline void operator=(const Camera &C) { memcpy(this, &C, sizeof(Camera)); }
    inline bool operator==(const Camera &C) const
    {
        return m_T == C.m_T && m_p == C.m_p && m_v == C.m_v && m_ba == C.m_ba &&
               m_bw == C.m_bw;
    }

    inline void MakeIdentity()
    {
        m_T.MakeIdentity();
        m_p.MakeZero();
    }

    inline void Get(float *q, float *p, float *v) const
    {
        m_T.GetQuaternion().Get(q);
        m_p.Get(p);
        m_v.Get(v);
    }

    inline bool Valid() const { return m_T.Valid(); }
    inline bool Invalid() const { return m_T.Invalid(); }
    inline void Invalidate() { m_T.Invalidate(); }
    inline void Print(const bool e = false) const
    {
        UT::PrintSeparator();
        m_T.Print(" T = ", e);
        m_p.Print(" p = ", e);
        m_v.Print(" v = ", e);
        m_ba.Print("ba = ", e);
        m_bw.Print("bw = ", e);
    }

    inline void AssertConsistency() const
    {
        m_T.AssertOrthogonal();
        UT_ASSERT(m_p.Valid());
        m_p.AssertEqual(m_T.GetPosition());
    }

    inline bool AssertEqual(const Camera &C, const int verbose = 1,
                            const float rEps = 0.001745329252f,
                            const float pEps = 0.0f,
                            const float vEps = 0.0f) const
    {
        if (m_T.AssertEqual(C.m_T, 0, rEps, pEps) &&
            m_p.AssertEqual(C.m_p, 0, pEps) &&
            m_v.AssertEqual(C.m_v, 0, vEps) && m_ba.AssertEqual(C.m_ba, 0) &&
            m_bw.AssertEqual(C.m_bw, 0))
            return true;
        if (verbose) {
            Print(verbose > 1);
            C.Print(verbose > 1);
        }
        return false;
    }

  public:
    Rigid3D m_T;
    Point3D m_p;
    LA::AlignedVector3f m_v, m_ba, m_bw;
};
#undef m_A
#undef m_b

#endif
