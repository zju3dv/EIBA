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

#include "PlatformIndependence/def_missing.h"
#include "PlatformIndependence/sse.h"

#include "GlobalBundleAdjustor.h"
#include "Matrix3x8.h"
#include "MatrixMxN.h"
#include "Depth.h"

#define DEPTH_MAP_VARIANCE 0.01f // 0.1^2
#define DEPTH_INITIAL_VARIANCE 1.0f // 1^2
#define DEPTH_MAX 2.0f  // 1/0.5
#define DEPTH_MIN 0.02f // 1/50


//#define CFG_DEBUG
#ifdef CFG_DEBUG
#define GBA_DEBUG
#ifdef GBA_DEBUG
#ifdef CFG_DEBUG_EIGEN
//#define GBA_DEBUG_EIGEN_JACOBIAN_FEATURE
#define GBA_DEBUG_EIGEN_JACOBIAN_PRIOR
#endif
#define GBA_DEBUG_ERROR_FRAME 187
#define GBA_DEBUG_ERROR_FEATURE 29
//#define GBA_DEBUG_UPDATE_FRAME	0
//#define GBA_DEBUG_UPDATE_FEATURE	208
#endif
#endif

#define GBA_M_ESTIMATOR_FUNCTION ME::FUNCTION_HUBER

//#define GBA_VARIANCE_FIX_ROTATION 3.046174198662e-6f // (0.1*pi/180)^2
//#define GBA_VARIANCE_FIX_POSITION 0.01f              // 0.1^2

#define GBA_PCG_MIN_ITERATIONS 10
#define GBA_PCG_MAX_ITERATIONS 100
#define GBA_PCG_MIN_DOT_RATIO 1.0e-4f
//#define GBA_PCG_MIN_DOT_RATIO		1.0e-3f
//#define GBA_PCG_MAX_DOT_RATIO		10.0f
#define GBA_PCG_MAX_DOT_RATIO 1000.0f

void GlobalBundleAdjustor::UpdateFactors()
{
    float r2;
    LA::Vector2f ex, Jxdx;
    LA::SymmetricMatrix2x2f Wx;
    LA::AlignedMatrix2x6f Jxcx, Jxcz;
    LA::AlignedMatrix2x13f Jx, WJx;
    LA::AlignedMatrix2x14f Jex;
    LA::AlignedMatrix13x14f A;
#ifdef CFG_DEPTH_MAP
    float ed;
    LA::AlignedVector3f Jxddx;
    LA::AlignedMatrix3x6f Jxdcx, Jxdcz;
    LA::AlignedMatrix3x13f Jxd, WJxd;
    LA::AlignedMatrix3x14f Jexd;
#endif
#ifdef CFG_VERBOSE
    int SNz1 = 0, SNz2 = 0;
#endif
    const float wd = 1.0f / DEPTH_MAP_VARIANCE,
                r2Max = ME::Variance<GBA_M_ESTIMATOR_FUNCTION>();
    const int nKFs = int(m_KFs.size());
#ifdef CFG_DEPTH_MAP
    const int iKFLast = nKFs - 1;
    for (int iKF = 0; iKF < iKFLast; ++iKF) {
        KeyFrame &KF = m_KFs[iKF];
        const DepthInverseGaussian *ds = m_dsLP.data() + KF.m_id;
        const ubyte *uds = m_uds.data() + KF.m_id;
        const int Nx = int(KF.m_xs.size());
        for (int ix = 0, jx = 0; ix < Nx; ++ix) {
            const float d = KF.m_xs[ix].m_d;
            if (d == 0.0f)
                continue;
            else if (uds[ix]) {
                FTR::Factor::Source &SAdcx = KF.m_SAdcxs[ix];
                FTR::Factor::Source::Depth &Ad = KF.m_Ads[jx];
#ifdef GBA_DEBUG
#if defined GBA_DEBUG_UPDATE_FRAME && defined GBA_DEBUG_UPDATE_FEATURE
                const bool debug = KF.m_T.m_iFrm == GBA_DEBUG_UPDATE_FRAME &&
                                   ix == GBA_DEBUG_UPDATE_FEATURE;
#else
                const bool debug = false;
#endif
                if (debug)
                    UT::Print("%f - %f = %f\n", SAdcx.m_ad, Ad.m_a,
                              SAdcx.m_ad - Ad.m_a);
#endif
                SAdcx -= Ad;
                ed = ds[ix].u() - d;
                r2 = wd * ed * ed;
                const float w = ME::Weight<GBA_M_ESTIMATOR_FUNCTION>(r2) * wd *
                                GBA_WEIGHT_FEATURE;
                Ad.m_a = w;
                Ad.m_b = w * ed;
#ifdef GBA_DEBUG
                if (debug)
                    UT::Print("%f + %f = %f\n", SAdcx.m_ad, Ad.m_a,
                              SAdcx.m_ad + Ad.m_a);
#endif
                SAdcx += Ad;
            }
            ++jx;
        }
    }
#endif
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        KeyFrame &KF = m_KFs[iKF];
        const ubyte uc = m_ucs[iKF];
        const Rigid3D &C = m_CsLP[iKF];
        Camera::Pose::Binary *SAcxzs = m_SAcbs.Data() + KF.m_iK;
        Camera::Pose::Unitary &SAczz = m_SAcus[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            const ubyte ur = uc | m_ucs[Z.m_iKF];
            const Rigid3D Tr = C / m_CsLP[Z.m_iKF];
            /*const */ KeyFrame &_KF = m_KFs[Z.m_iKF];
            const ubyte *_uds = m_uds.data() + _KF.m_id;
            const DepthInverseGaussian *_ds = m_dsLP.data() + _KF.m_id;
            // FTR::Factor::Source *_SAdcxs = m_SAdcxs.Data() + _KF.m_id;
            Camera::Pose::Unitary &SAcxx = m_SAcus[Z.m_iKF];
            Camera::Pose::Binary &SAcxz = SAcxzs[KF.m_iZ2k[iZ]];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
#if 0
//#if 1
				if(UT::Debugging() && iKF == 11 && iz == 35)
					UT::Check("GBA::UpdateFactors\n");
#endif
                const FTR::Measurement &z = KF.m_zs[iz];
                if ((KF.m_uzs[iz] = ur | _uds[z.m_ix]) == 0) continue;
#ifdef CFG_VERBOSE
                if (m_verbose >= 2) ++SNz1;
#endif
                // FTR::Factor::Source &SAdcx = _SAdcxs[z.m_ix];
                FTR::Factor::Source &SAdcx = _KF.m_SAdcxs[z.m_ix];
                FTR::Factor::Source &Adcx = KF.m_Adcxs[iz];
                FTR::Factor::Measurement &Adcz = KF.m_Adczs[iz];
                Camera::Pose::Unitary &Acxx = KF.m_Acxxs[iz];
                Camera::Pose::Binary &Acxz = KF.m_Acxzs[iz];
                Camera::Pose::Unitary &Aczz = KF.m_Aczzs[iz];
#ifdef GBA_DEBUG
#if defined GBA_DEBUG_UPDATE_FRAME && defined GBA_DEBUG_UPDATE_FEATURE
                const bool debug = _KF.m_T.m_iFrm == GBA_DEBUG_UPDATE_FRAME &&
                                   z.m_ix == GBA_DEBUG_UPDATE_FEATURE;
#else
                const bool debug = false;
#endif
                if (debug)
                    UT::Print("[%d] %d: %f - %f = %f\n", KF.m_T.m_iFrm, iz,
                              SAdcx.m_ad, Adcx.m_ad, SAdcx.m_ad - Adcx.m_ad);
#endif
                SAdcx -= Adcx;
                SAcxx -= Acxx;
                SAcxz -= Acxz;
                SAczz -= Aczz;
                const bool vd = z.m_d != 0.0f;
#ifdef CFG_DEPTH_MAP
                if (vd) {
                    z.GetErrorJacobian(Tr, _KF.m_xs[z.m_ix].m_x, _ds[z.m_ix], C,
                                       ex, ed, Jxddx, Jxdcx, Jxdcz);
                    Jxd.Set(Jxddx, Jxdcx, Jxdcz);
                    Jexd.Set(Jxd, ex, ed);
                    r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W,
                                                                      ex) +
                         wd * ed * ed;
                } else
#endif
                {
                    z.GetErrorJacobian(Tr, _KF.m_xs[z.m_ix].m_x, _ds[z.m_ix], C,
                                       ex, Jxdx, Jxcx, Jxcz);
                    Jx.Set(Jxdx, Jxcx, Jxcz);
                    Jex.Set(Jx, ex);
                    r2 =
                        LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ex);
                }
                const float w = ME::Weight<GBA_M_ESTIMATOR_FUNCTION>(r2) *
                                GBA_WEIGHT_FEATURE;
                z.m_W.GetScaled(w, Wx);
#ifdef CFG_DEPTH_MAP
                if (vd) {
                    const float _wd = w * wd;
                    LA::AlignedMatrix3x13f::AB(Wx, _wd, Jxd, WJxd);
                    LA::AlignedMatrix13x14f::ATBToUpper(WJxd, Jexd, A);
                } else
#endif
                {
                    LA::AlignedMatrix2x13f::AB(Wx, Jx, WJx);
                    LA::AlignedMatrix13x14f::ATBToUpper(WJx, Jex, A);
                }
#define Adcx Adcx.fe
#define Aczz Aczz.fe
#define Acxx Acxx.fe
                A.Get(Adcx.m_ad, Adcx.m_ac, Adcz, Adcx.m_b, Acxx.m_A, Acxz,
                      Acxx.m_b, Aczz.m_A, Aczz.m_b);
#undef Adcx
#undef Aczz
#undef Acxx
#ifdef CFG_DEBUG
                if (debug)
                    UT::Print("[%d] %d: %f + %f = %f\n", KF.m_T.m_iFrm, iz,
                              SAdcx.m_ad, Adcx.m_ad, SAdcx.m_ad + Adcx.m_ad);
#endif
                SAdcx += Adcx;
                SAcxx += Acxx;
                SAcxz += Acxz;
                SAczz += Aczz;
#ifdef GBA_DEBUG_EIGEN_JACOBIAN_FEATURE
                const EigenErrorJacobian::Feature e_Je =
                    DebugEigenErrorJacobianFeature(Z.m_iKF, iKF, iz);
#ifdef CFG_DEPTH_MAP
                if (vd) {
                    const EigenMatrix3x13f e_Jxd = EigenMatrix3x13f(
                        e_Je.m_Jxdd, e_Je.m_Jxdcx, e_Je.m_Jxdcz);
                    const EigenMatrix3x14f e_Jexd =
                        EigenMatrix3x14f(e_Jxd, e_Je.m_exd);
                    e_Je.m_Jxdd.AssertEqual(Jxddx);
                    e_Je.m_Jxdcx.AssertEqual(Jxdcx);
                    e_Je.m_Jxdcz.AssertEqual(Jxdcz);
                    e_Je.m_exd.AssertEqual(LA::Vector3f(ex, ed));
                } else
#endif
                {
                    const EigenMatrix2x13f e_Jx =
                        EigenMatrix2x13f(e_Je.m_Jxd, e_Je.m_Jxcx, e_Je.m_Jxcz);
                    const EigenMatrix2x14f e_Jex =
                        EigenMatrix2x14f(e_Jx, e_Je.m_ex);
                    e_Je.m_Jxd.AssertEqual(Jxdx);
                    e_Je.m_Jxcx.AssertEqual(Jxcx);
                    e_Je.m_Jxcz.AssertEqual(Jxcz);
                    e_Jx.AssertEqual(Jx);
                    e_Jex.AssertEqual(Jex);
                }
#endif
            }
        }
#ifdef CFG_VERBOSE
        if (m_verbose >= 2) SNz2 += int(KF.m_zs.size());
#endif
    }

    Camera::Pose::Prior::Rigid Zp;
    Camera::Pose::Prior::Rigid::Error ep;
    Camera::Pose::Prior::Rigid::Jacobian1 Jp1;
    Camera::Pose::Prior::Rigid::Jacobian2 Jp2;
    Camera::Pose::Prior::Rigid::WeightedJacobian WJp1, WJp2;
    const _pi__m128 wp = _pi_mm_set1_ps(GBA_WEIGHT_PRIOR);
#ifdef CFG_VERBOSE
    int SNp1 = 0, SNp2 = 0;
#endif
    for (int iKF2 = 0; iKF2 < nKFs; ++iKF2) {
        KeyFrame &KF2 = m_KFs[iKF2];
        const int Np = int(KF2.m_iKFsPrior.size());
        if (Np == 0) continue;
        Camera::Pose::Binary *SAp12s = m_SAcbs.Data() + KF2.m_iK;
        Camera::Pose::Unitary &SAp22 = m_SAcus[iKF2];
        const Rigid3D &C2 = m_CsLP[iKF2];
        const Point3D p2 = C2.GetPosition();
        for (int ip = 0; ip < Np; ++ip) {
            const int iKF1 = KF2.m_iKFsPrior[ip];
            Camera::Pose::Unitary &Ap11 = KF2.m_Ap11s[ip];
            if (m_ucs[iKF1] == 0 && m_ucs[iKF2] == 0) continue;
#ifdef CFG_VERBOSE
            if (m_verbose >= 2) ++SNp1;
#endif
            Camera::Pose::Unitary &SAp11 = m_SAcus[iKF1];
            Camera::Pose::Binary &SAp12 = SAp12s[KF2.m_iZp2k[ip]],
                                 &Ap12 = KF2.m_Ap12s[ip];
            Camera::Pose::Unitary &Ap22 = KF2.m_Ap22s[ip];
            SAp11 -= Ap11;
            SAp12 -= Ap12;
            SAp22 -= Ap22;
            KF2.m_Zps[ip].GetWeighted(wp, Zp);
            Zp.GetErrorJacobian(m_CsLP[iKF1], C2, p2, ep, Jp1, Jp2);
#ifdef GBA_DEBUG_EIGEN_JACOBIAN_PRIOR
            const float eps = 0.01f;
            const EigenErrorJacobian::Prior e_Je =
                DebugEigenErrorJacobianPrior(iKF1, iKF2, ip);
            EigenMatrix3x3f(e_Je.m_Jrpc1.block<3, 3>(0, 3).transpose())
                .AssertEqual(Jp1.m_JrrT);
            EigenMatrix3x3f(e_Je.m_Jrpc2.block<3, 3>(0, 3).transpose())
                .AssertEqual(Jp2.m_JrrT);
            EigenMatrix3x3f(e_Je.m_Jrpc1.block<3, 3>(3, 0).transpose())
                .AssertEqual(Jp1.m_JppT);
            EigenMatrix3x3f(e_Je.m_Jrpc2.block<3, 3>(3, 0).transpose())
                .AssertEqual(Jp2.m_JppT);
            EigenMatrix3x3f(e_Je.m_Jrpc1.block<3, 3>(3, 3).transpose())
                .AssertEqual(Jp1.m_JprT.GetAlignedMatrix3x3f());
            EigenVector3f(e_Je.m_erp.block<3, 1>(0, 0))
                .AssertEqual(ep.m_er, 1, eps * UT_FACTOR_DEG_TO_RAD);
            EigenVector3f(e_Je.m_erp.block<3, 1>(3, 0)).AssertEqual(ep.m_ep);
#endif
            Zp.GetWeightedJacobian(Jp1, Jp2, WJp1, WJp2);
            Camera::Pose::Prior::Rigid::WeightedJacobian::ATB(
                WJp1, WJp2, Jp1, Jp2, ep, Ap11, Ap12, Ap22);
            SAp11 += Ap11;
            SAp12 += Ap12;
            SAp22 += Ap22;
        }
#ifdef CFG_VERBOSE
        if (m_verbose >= 2) SNp2 += Np;
#endif
    }

    if (m_iKFFix != -1) {
        Camera::Pose::Unitary &Af = m_SAcus[nKFs];
#define Af Af.fe
        Af.m_A.MakeZero();
        const float wfr = GBA_WEIGHT_FIX / GBA_VARIANCE_FIX_ROTATION,
                    wfp = GBA_WEIGHT_FIX / GBA_VARIANCE_FIX_POSITION;
        const LA::AlignedVector3f efp = m_CsLP[m_iKFFix].GetPosition() -
                                        m_CsLP[nKFs].GetPosition(),
                                  bfp = efp * wfp;
        Af.m_A.m00() = Af.m_A.m11() = Af.m_A.m22() = wfp;
        Af.m_b.v0() = bfp.v0();
        Af.m_b.v1() = bfp.v1();
        Af.m_b.v2() = bfp.v2();
        const LA::AlignedVector3f efr =
            (Rotation3D(m_CsLP[m_iKFFix]) / m_CsLP[nKFs]).GetRodrigues();
        const LA::AlignedMatrix3x3f
            JfrT = Rotation3D::GetRodriguesJacobianInverse(efr).GetTranspose(),
            wJfrT = JfrT * wfr;
        LA::SymmetricMatrix6x6f::ABTTo33(wJfrT, JfrT, Af.m_A);
        LA::SymmetricMatrix6x6f::AbTo3(wJfrT, efr, Af.m_b);
#undef Af
        Af += m_SAcus[m_iKFFix];
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 2) {
        UT::PrintSeparator();
        UT::Print("[%d] Updated Measurement Factor = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNz1, SNz2,
                  SNz2 == 0 ? 0.0f : float(SNz1 * 100) / SNz2);
        UT::Print("[%d] Updated Prior Factor = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNp1, SNp2,
                  SNp2 == 0 ? 0.0f : float(SNp1 * 100) / SNp2);
    }
#endif

#if 0
//#if 1
	if(nKFs > 0 && m_KFs[nKFs - 1].m_T.m_iFrm == 175)
		m_SAcus[6].Print();
#endif
}

void GlobalBundleAdjustor::UpdateSchurComplement()
{
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            ubyte *uds = m_uds.data() + m_KFs[Z.m_iKF].m_id;
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz)
                uds[KF.m_zs[iz].m_ix] |= KF.m_uzs[iz];
        }
    }

    const int Nd = int(m_ds.size()), NdC = SSE_FLOAT_CEIL(Nd);
    m_work.Resize(NdC + Nd * sizeof(_pi__m128) / sizeof(float));
    AlignedVector<float> mds(m_work.Data(), Nd, false);
    AlignedVector<_pi__m128> _pi_mds((_pi__m128 *)(mds.Data() + NdC), Nd,
                                     false);

    mds.Resize(0);
    m_idxsTmp1.assign(Nd, -1);
    // for(int id = 0, iud = 0; id < Nd; ++id)
    //{
    //	if(m_uds[id] == 0/* || m_SAdcxs[id].m_ad < FLT_EPSILON*/)
    //		continue;
    //	mds.Push(m_SAdcxs[id].m_ad);
    //	m_idxsTmp1[id] = iud++;
    //}
    const int iKFLast = nKFs - 1;
    for (int iKF = 0, iud = 0; iKF < iKFLast; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const ubyte *uds = m_uds.data() + KF.m_id;
        int *ix2ud = m_idxsTmp1.data() + KF.m_id;
        const int Nx = KF.m_SAdcxs.Size();
        for (int ix = 0; ix < Nx; ++ix) {
            if (uds[ix] == 0 /* || KF.m_SAdcxs[ix].m_ad < FLT_EPSILON*/)
                continue;
#if defined GBA_DEBUG_FRAME && defined GBA_DEBUG_FEATURE
            if (KF.m_T.m_iFrm == GBA_DEBUG_FRAME && ix == GBA_DEBUG_FEATURE)
                UT::Print("[%d] %d SAdd = %e\n", KF.m_T.m_iFrm, ix,
                          KF.m_SAdcxs[ix].m_ad);
#endif
            mds.Push(KF.m_SAdcxs[ix].fe.m_ad);
            ix2ud[ix] = iud++;
        }
    }
    const int Nud = mds.Size();
    SSE::Inverse(Nud, mds.Data(), 1.0f, true);
    _pi_mds.Resize(Nud);
    for (int iud = 0; iud < Nud; ++iud) _pi_mds[iud] = _pi_mm_set1_ps(mds[iud]);
    for (int iKF = 0; iKF < iKFLast; ++iKF) {
        KeyFrame &KF = m_KFs[iKF];
        Camera::Pose::Unitary &SAcxx = m_SAcus[iKF];
        // const FTR::Factor::Source *SAdcxs = m_SAdcxs.Data() + KF.m_id;
        // FTR::Factor::Source *SMdcxs = m_SMdcxs.Data() + KF.m_id;
        // Camera::Pose::Unitary *SMcxxs = m_SMcxxs.Data() + KF.m_id;
        const int *ix2ud = m_idxsTmp1.data() + KF.m_id;
        const int Nx = int(KF.m_xs.size());
        for (int ix = 0; ix < Nx; ++ix) {
            const int iud = ix2ud[ix];
            if (iud == -1) continue;
            // const FTR::Factor::Source &SAdcx = SAdcxs[ix];
            // FTR::Factor::Source &SMdcx = SMdcxs[ix];
            // Camera::Pose::Unitary &SMcxx = SMcxxs[ix];
            const FTR::Factor::Source &SAdcx = KF.m_SAdcxs[ix];
            FTR::Factor::Source &SMdcx = KF.m_SMdcxs[ix];
            Camera::Pose::Unitary &SMcxx = KF.m_SMcxxs[ix];
            SAdcx.GetScaled(_pi_mds[iud], SMdcx);
            SMdcx.fe.m_md = mds[iud];
            SAcxx += SMcxx;
            FTR::Factor::Source::Marginalize(SMdcx, SAdcx, SMcxx);
            SAcxx -= SMcxx;
#if defined GBA_DEBUG_FRAME && defined GBA_DEBUG_FEATURE
            if (KF.m_T.m_iFrm == GBA_DEBUG_FRAME && ix == GBA_DEBUG_FEATURE)
                UT::Print("[%d] %d SMdd = %e\n", KF.m_T.m_iFrm, ix, SMdcx.m_md);
#endif
#if 0
//#if 1
			if(UT::Debugging() && iKF == 10)
			{
				if(ix == 0)
					UT::PrintSeparator();
				UT::Print("%d ", ix);
				SAcxx.m_b.Print();
			}
#endif
        }
    }
#if 0
//#if 1
	if(UT::Debugging())
		return;
#endif
#ifdef CFG_VERBOSE
    if (m_verbose >= 2)
        UT::Print("[%d] Updated Source = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, Nud, Nd,
                  Nd == 0 ? 0.0f : float(Nud * 100) / Nd);
    int SNz1 = 0, SNuz1 = 0;
    int SNz2 = 0, SNuz2 = 0;
#endif
    LA::AlignedVector6f Adcz;
    LA::AlignedVector6f Mdcz1;
    LA::ProductVector6f Mdcz2;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        KeyFrame &KF = m_KFs[iKF];
        Camera::Pose::Binary *SAcbs = m_SAcbs.Data() + KF.m_iK;
        Camera::Pose::Unitary &SAczz = m_SAcus[iKF];
        m_MdczsTmp.Resize(0);
        m_idxsTmp2.assign(KF.m_zs.size(), -1);
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0, iuz = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            const KeyFrame &_KF = m_KFs[Z.m_iKF];
            // const FTR::Factor::Source *_SAdcxs = m_SAdcxs.Data() + _KF.m_id;
            const int *_ix2ud = m_idxsTmp1.data() + _KF.m_id;
            Camera::Pose::Binary &SAcxz = SAcbs[KF.m_iZ2k[iZ]];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
                const int ix = KF.m_zs[iz].m_ix, iud = _ix2ud[ix];
                if (iud == -1) continue;
                Camera::Pose::Binary &Mcxz = KF.m_Mcxzs[iz];
                Camera::Pose::Unitary &Mczz = KF.m_Mczzs[iz];
                SAcxz += Mcxz;
                SAczz += Mczz;
                // const FTR::Factor::Source &SAdcx = _SAdcxs[ix];
                const FTR::Factor::Source &SAdcx = _KF.m_SAdcxs[ix];
                Adcz.Set(KF.m_Adczs[iz]);
                Adcz.GetScaled(_pi_mds[iud], Mdcz1);
                Mdcz2.Set(Mdcz1);
                KF.m_Mdczs[iz].Set(Mdcz1);
                FTR::Factor::Measurement::Marginalize(SAdcx, Mdcz2, Mcxz);
                FTR::Factor::Measurement::Marginalize(Mdcz1, Adcz, SAdcx.fe.m_b,
                                                      Mczz);
                SAcxz -= Mcxz;
                SAczz -= Mczz;
                m_MdczsTmp.Push(Mdcz2);
                m_idxsTmp2[iz] = iuz++;
#ifdef CFG_VERBOSE
                if (m_verbose >= 2) ++SNuz1;
#endif
            }
        }
#ifdef CFG_VERBOSE
        if (m_verbose >= 2) SNz1 += int(KF.m_zs.size());
#endif
        const int Nk = int(KF.m_iKFsMatch.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const KeyFrame &_KF = m_KFs[KF.m_iKFsMatch[ik]];
            Camera::Pose::Binary &SAczm = SAcbs[ik];
            const int i1 = KF.m_Zm.m_ik2zm[ik], i2 = KF.m_Zm.m_ik2zm[ik + 1];
            for (int i = i1; i < i2; ++i) {
                const FTR::Measurement::Match &izm = KF.m_Zm.m_izms[i];
                const int iuz = m_idxsTmp2[izm.m_iz2];
                if (iuz == -1) continue;
                Camera::Pose::Binary &Mczm = KF.m_Zm.m_Mczms[i];
                SAczm += Mczm;
                FTR::Factor::Measurement::Marginalize(_KF.m_Adczs[izm.m_iz1],
                                                      m_MdczsTmp[iuz], Mczm);
                SAczm -= Mczm;
#ifdef CFG_VERBOSE
                if (m_verbose >= 2) ++SNuz2;
#endif
            }
        }
#ifdef CFG_VERBOSE
        if (m_verbose >= 2) SNz2 += int(KF.m_Zm.m_izms.size());
#endif
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 2) {
        UT::Print("[%d] Updated Measurement = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNuz1, SNz1,
                  SNz1 == 0 ? 0.0f : float(SNuz1 * 100) / SNz1);
        UT::Print("[%d] Updated Measurement Match = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNuz2, SNz2,
                  SNz2 == 0 ? 0.0f : float(SNuz2 * 100) / SNz2);
    }
#endif
}

bool GlobalBundleAdjustor::UpdateCameraStates()
{
    const int nKFs = int(m_KFs.size()), NK = m_SAcbs.Size(),
              Nd = int(m_ds.size()), NdC = SSE_FLOAT_CEIL(Nd);
    const int Nc = nKFs * sizeof(LA::Vector6f) / sizeof(float),
              NcC = SSE_FLOAT_CEIL(Nc);
    const int N1 = nKFs * 2 * sizeof(LA::AlignedMatrix6x6f) / sizeof(float) +
                   NK * sizeof(LA::AlignedMatrix6x6f) / sizeof(float) +
                   NcC * 5 + nKFs * sizeof(LA::ProductVector6f) / sizeof(float);
    const int N2 = NdC * 3;
    m_work.Resize(NcC + std::max(N1, N2));
    m_xcs.Set<false>(m_work.Data(), Nc);
    m_Ms.Set<false>((LA::AlignedMatrix6x6f *)(m_xcs.Data() + NcC), nKFs);
    m_Aus.Set<false>(m_Ms.End(), nKFs);
    m_Abs.Set<false>(m_SAcbs.Data(), NK);
    m_AbTs.Set<false>(m_Aus.End(), NK);
    m_bs.Set<false>((float *)m_AbTs.End(), Nc);
    m_rs.Set<false>(m_bs.Data() + NcC, Nc);
    m_ps.Set<false>(m_rs.Data() + NcC, Nc);
    m_zs.Set<false>(m_ps.Data() + NcC, Nc);
    m_ts.Set<false>(m_zs.Data() + NcC, Nc);
    m_xps.Set<false>((LA::ProductVector6f *)(m_ts.Data() + NcC), nKFs);

    float *b = m_bs.Data();
    for (int iKF = 0; iKF < nKFs; ++iKF, b += 6) {
        LA::AlignedMatrix6x6f &A = m_Aus[iKF];
        if (iKF == m_iKFFix) {
            m_SAcus[nKFs].fe.m_A.GetAlignedMatrix6x6f(A);
            m_SAcus[nKFs].fe.m_b.Get(b);
        } else {
            m_SAcus[iKF].fe.m_A.GetAlignedMatrix6x6f(A);
            m_SAcus[iKF].fe.m_b.Get(b);
        }
        if (!A.GetInverseLDL(m_Ms[iKF])) m_Ms[iKF].MakeZero();
    }
    for (int ik = 0; ik < NK; ++ik) m_Abs[ik].GetTranspose(m_AbTs[ik]);

    if (!SolveSchurComplement()) return false;

    LA::AlignedVector3f dp, dr, p;
    Rotation3D dR;
    m_xcs.MakeMinus();
    m_xc2s.Set<false>(m_xcs.Data() + NcC, Nc);
    m_xcs.GetSquared(m_xc2s);
    const LA::Vector6f *xcs = (LA::Vector6f *)m_xcs.Data(),
                       *xc2s = (LA::Vector6f *)m_xc2s.Data();
#ifdef CFG_VERBOSE
    int SNc = 0;
#endif
    float dp2Max = 0.0f, dr2Max = 0.0f;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        Rigid3D &CLP = m_CsLP[iKF], &C = m_Cs[iKF];
        const LA::Vector6f &x = xcs[iKF], &x2 = xc2s[iKF];
        dp.Set(&x.v0());
        dr.Set(&x.v3());
        dR.SetRodrigues(dr);
        C = dR * CLP;
        C.MakeOrthogonal();
        CLP.GetPosition(p);
        p += dp;
        C.SetPosition(p);
        const float dp2 = x2.v0() + x2.v1() + x2.v2(),
                    dr2 = x2.v3() + x2.v4() + x2.v5();
        if (dp2 < GBA_UPDATE_POSITION && dr2 < GBA_UPDATE_ROTATION)
            m_ucs[iKF] = 0;
        else {
            m_ucs[iKF] = m_Ucs[iKF] = 1;
            CLP = C;
#ifdef CFG_VERBOSE
            if (m_verbose >= 2) ++SNc;
#endif
        }
        dp2Max = std::max(dp2, dp2Max);
        dr2Max = std::max(dr2, dr2Max);
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 2)
        UT::Print("[%d] Updated Camera = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNc, nKFs, float(SNc * 100) / nKFs);
#endif
    return dr2Max > GBA_CONVERGE_ROTATION || dp2Max > GBA_CONVERGE_POSITION;
}

bool GlobalBundleAdjustor::UpdateDepthStates()
{
    const int Nd = int(m_ds.size()), NdC = SSE_FLOAT_CEIL(Nd);
    m_uds.assign(Nd, 0);
    const int nKFs = int(m_KFs.size()), iKFLast = nKFs - 1;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        if (!m_ucs[iKF]) continue;
        const KeyFrame &KF = m_KFs[iKF];
        if (iKF < iKFLast) {
            ubyte *uds = m_uds.data() + KF.m_id;
            const int Nx = int(KF.m_xs.size());
            for (int ix = 0; ix < Nx; ++ix) uds[ix] = 1;
        }
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            ubyte *uds = m_uds.data() + m_KFs[Z.m_iKF].m_id;
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) uds[KF.m_zs[iz].m_ix] = 1;
        }
    }
    int iud = 0;
    m_idxsTmp1.assign(Nd, -1);
    for (int id = 0; id < Nd; ++id) {
        if (m_uds[id]) m_idxsTmp1[id] = iud++;
    }
    const int Nc = nKFs * sizeof(LA::Vector6f) / sizeof(float),
              NcC = SSE_FLOAT_CEIL(Nc);
    const int Nud = iud, NudC = SSE_FLOAT_CEIL(Nud);
    m_xds.Set<false>(m_xcs.Data() + NcC, Nud);
    m_xd2s.Set<false>(m_xds.Data() + NudC, Nud);
    m_xds2s.Set<false>(m_xd2s.Data() + NudC, Nud);

    LA::ProductVector6f xc;
    const LA::Vector6f *xcs = (LA::Vector6f *)m_xcs.Data();
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        xc.Set(xcs[iKF]);
        const KeyFrame &KF = m_KFs[iKF];
        if (iKF < iKFLast) {
            const int *ix2ud = m_idxsTmp1.data() + KF.m_id;
            // const FTR::Factor::Source *SMdcxs = m_SMdcxs.Data() + KF.m_id;
            const int Nx = int(KF.m_xs.size());
            for (int ix = 0; ix < Nx; ++ix) {
                const int iud = ix2ud[ix];
                if (iud == -1) continue;
                // const FTR::Factor::Source &SMdcx = SMdcxs[ix];
                const FTR::Factor::Source &SMdcx = KF.m_SMdcxs[ix];
                m_xds[iud] = SMdcx.BackSubstitute(xc);
                m_xds2s[iud] = SMdcx.fe.m_md;
            }
        }
        // const int Nz = int(KF.m_zs.size()), NzF = Nz - (Nz & 1);
        // m_xdsTmp.resize(Nz);
        // const LA::ProductVector6f *Mdcz = (LA::ProductVector6f *)
        // KF.m_Mdczs.Data();
        // float *xd = m_xdsTmp.data();
        // for(int iz = 0; iz < NzF; iz += 2, ++Mdcz, xd += 2)
        //	FTR::Factor::Measurement::BackSubstitute(*Mdcz, xc, xd);
        // if(NzF != Nz)
        //	m_xdsTmp[NzF] = KF.m_Mdczs[NzF].BackSubstitute(xc);
        // const int NZ = int(KF.m_Zs.size());
        // for(int iZ = 0; iZ < NZ; ++iZ)
        //{
        //	const FRM::Measurement &Z = KF.m_Zs[iZ];
        //	const KeyFrame &_KF = m_KFs[Z.m_iKF];
        //	float *_xds = m_xds.Data() + _KF.m_id;
        //	const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
        //	for(int iz = iz1; iz < iz2; ++iz)
        //		_xds[KF.m_zs[iz].m_ix] += m_xdsTmp[iz];
        //}
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            const KeyFrame &_KF = m_KFs[Z.m_iKF];
            const int *ix2ud = m_idxsTmp1.data() + _KF.m_id;
            // float *_xds = m_xds.Data() + _KF.m_id;
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
                const int iud = ix2ud[KF.m_zs[iz].m_ix];
                if (iud != -1) m_xds[iud] += KF.m_Mdczs[iz].BackSubstitute(xc);
            }
        }
    }
    m_xds.MakeMinus();
#ifdef CFG_VERBOSE
    int SNd = 0;
#endif
    m_xds.GetSquared(m_xd2s);
    m_xds2s *= GBA_WEIGHT_FEATURE;
    for (int id = 0; id < Nd; ++id) {
        const int iud = m_idxsTmp1[id];
        if (iud == -1) continue;
        DepthInverseGaussian &d = m_ds[id], &dLP = m_dsLP[id];
        d.u() = m_xds[iud] + dLP.u();
        d.s2() = m_xds2s[iud];
        if (d.u() < DEPTH_MIN || d.u() > DEPTH_MAX || d.s2() == 0.0f ||
            d.s2() > DEPTH_INITIAL_VARIANCE) {
            d = dLP;
            m_uds[id] = 0;
        } else if (m_xd2s[iud] < GBA_UPDATE_DEPTH)
            m_uds[id] = 0;
        else {
            m_uds[id] = m_Uds[id] = 1;
            dLP = d;
#ifdef CFG_VERBOSE
            if (m_verbose >= 2) ++SNd;
#endif
        }
    }
#ifdef CFG_VERBOSE
    if (m_verbose >= 2)
        UT::Print("[%d] Updated Depth  = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNd, Nd,
                  Nd == 0 ? 0.0f : float(SNd * 100) / Nd);
#endif
    const float dd2Max = m_xd2s.Maximal();
    return dd2Max > GBA_CONVERGE_DEPTH;
}

bool GlobalBundleAdjustor::SolveSchurComplement()
{
    float rTzPre, rTzCur, rTzAbs, alpha, beta;
    m_rs.Swap(m_bs);
    ApplyM(m_rs, m_ps);
    rTzCur = m_rs.Dot(m_ps);
    rTzAbs = fabs(rTzCur);
    if (rTzAbs < FLT_EPSILON) {
        m_xcs.MakeZero();
        return true;
    }
    ApplyA(m_ps, m_ts);
    alpha = rTzCur / m_ps.Dot(m_ts);
    // printf("%f, %f\n", rTzCur, alpha);
    const float rTzMin = rTzAbs * GBA_PCG_MIN_DOT_RATIO,
                rTzMax = rTzAbs * GBA_PCG_MAX_DOT_RATIO;
    if (!_finite(alpha)) return rTzAbs < rTzMin;
    m_xcs.Set(m_ps);
    m_xcs *= alpha;
    m_ts *= alpha;
    m_rs -= m_ts;
//#if _DEBUG
#if 0
	ApplyA(A, dx, m_t);
	m_t -= b;
	IO::PrintSeparator();
	printf("%e: %e\n", rTzCur, m_t.SquaredLength());
#endif
    for (int iIter = 0; iIter < GBA_PCG_MAX_ITERATIONS; ++iIter) {
        ApplyM(m_rs, m_zs);
        rTzPre = rTzCur;
        rTzCur = m_rs.Dot(m_zs);
        rTzAbs = fabs(rTzCur);
        if (rTzAbs <= rTzMin && iIter >= GBA_PCG_MIN_ITERATIONS ||
            rTzAbs < FLT_EPSILON)
            return true;
        else if (rTzAbs > rTzMax)
            return false;
        beta = rTzCur / rTzPre;
        m_ps *= beta;
        m_ps += m_zs;
        ApplyA(m_ps, m_ts);
        alpha = rTzCur / m_ps.Dot(m_ts);
        if (!_finite(alpha)) return false;
        m_ts *= alpha;
        m_rs -= m_ts;
        m_ts = m_ps;
        m_ts *= alpha;
        m_xcs += m_ts;
//#if _DEBUG
#if 0
		ApplyA(A, dx, m_t);
		m_t -= b;
		printf("%e: %e\n", rTzCur, m_t.SquaredLength());
#endif
    }
    return true;
}

void GlobalBundleAdjustor::ApplyM(const LA::AlignedVectorXf &xs,
                                  LA::AlignedVectorXf &Mxs)
{
    LA::ProductVector6f x;
    const LA::Vector6f *_xs = (LA::Vector6f *)xs.Data();
    LA::Vector6f *_Mxs = (LA::Vector6f *)Mxs.Data();
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        x.Set(_xs[iKF]);
        LA::AlignedMatrix6x6f::Ab(m_Ms[iKF], x, _Mxs[iKF]);
    }
}

void GlobalBundleAdjustor::ApplyA(const LA::AlignedVectorXf &xs,
                                  LA::AlignedVectorXf &Axs)
{
    const LA::Vector6f *_xs = (LA::Vector6f *)xs.Data();
    LA::Vector6f *_Axs = (LA::Vector6f *)Axs.Data();
    const int nKFs = int(m_KFs.size());
    m_xps.Resize(nKFs);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        m_xps[iKF].Set(_xs[iKF]);
        LA::AlignedMatrix6x6f::Ab(m_Aus[iKF], m_xps[iKF], _Axs[iKF]);
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const LA::AlignedMatrix6x6f *Abs = m_Abs.Data() + KF.m_iK,
                                    *AbTs = m_AbTs.Data() + KF.m_iK;
        const LA::ProductVector6f &xp = m_xps[iKF];
        LA::Vector6f &Ax = _Axs[iKF];
        const int Nk = int(KF.m_ik2KF.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const int _iKF = KF.m_ik2KF[ik];
            LA::AlignedMatrix6x6f::AddAbTo(Abs[ik], xp, _Axs[_iKF]);
            LA::AlignedMatrix6x6f::AddAbTo(AbTs[ik], m_xps[_iKF], Ax);
        }
    }
}

FTR::Measurement::ES GlobalBundleAdjustor::ComputeErrorStatisticFeature(
    const std::vector<Rigid3D> &Cs,
    const std::vector<DepthInverseGaussian> &ds)
{
    FTR::Measurement::ES ES;
    ES.Initialize();

    float r2;
    LA::Vector2f ex;
#ifdef CFG_DEPTH_MAP
    float ed;
#endif
    const float wd = 1.0f / DEPTH_MAP_VARIANCE,
                r2Max = ME::Variance<GBA_M_ESTIMATOR_FUNCTION>();
    const int nKFs = int(m_KFs.size());
#ifdef CFG_DEPTH_MAP
    ex.MakeZero();
    const int iKFLast = nKFs - 1;
    for (int iKF = 0; iKF < iKFLast; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const DepthInverseGaussian *_ds = ds.data() + KF.m_id;
        const int Nx = int(KF.m_xs.size());
        for (int ix = 0; ix < Nx; ++ix) {
            const float d = KF.m_xs[ix].m_d;
            if (d == 0.0f) continue;
            ed = _ds[ix].u() - d;
#ifdef GBA_DEBUG
#if defined GBA_DEBUG_ERROR_FRAME && defined GBA_DEBUG_ERROR_FEATURE
            const bool debug = KF.m_T.m_iFrm == GBA_DEBUG_ERROR_FRAME &&
                               ix == GBA_DEBUG_ERROR_FEATURE;
#else
            const bool debug = false;
#endif
            if (debug) UT::Print("%f\n", ed);
#endif
            r2 = wd * ed * ed;
            const float w = ME::Weight<GBA_M_ESTIMATOR_FUNCTION>(r2);
            ES.Accumulate(m_K, ex, ed, r2 * w * GBA_WEIGHT_FEATURE,
                          FTR::Measurement::ESIndex(KF.m_T.m_iFrm, ix),
                          r2 < r2Max);
        }
    }
#endif
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const Rigid3D &C = Cs[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            const Rigid3D Tr = C / Cs[Z.m_iKF];
            const KeyFrame &_KF = m_KFs[Z.m_iKF];
            const DepthInverseGaussian *_ds = ds.data() + _KF.m_id;
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
                const FTR::Measurement &z = KF.m_zs[iz];
#ifdef CFG_DEPTH_MAP
                const bool vd = z.m_d != 0.0f;
                if (vd) {
                    z.GetError(Tr, _KF.m_xs[z.m_ix].m_x, _ds[z.m_ix], ex, ed);
                    r2 = LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W,
                                                                      ex) +
                         wd * ed * ed;
                } else
#endif
                {
                    z.GetError(Tr, _KF.m_xs[z.m_ix].m_x, _ds[z.m_ix], ex);
                    r2 =
                        LA::SymmetricMatrix2x2f::MahalanobisDistance(z.m_W, ex);
                }
#ifdef GBA_DEBUG
#if defined GBA_DEBUG_ERROR_FRAME && defined GBA_DEBUG_ERROR_FEATURE
                const bool debug = _KF.m_T.m_iFrm == GBA_DEBUG_ERROR_FRAME &&
                                   z.m_ix == GBA_DEBUG_ERROR_FEATURE;
#else
                const bool debug = false;
#endif
                if (debug) {
                    // UT::DebugStart();
                    UT::PrintSeparator();
                    m_Cs[Z.m_iKF].Print("Cx = ", false);
                    _KF.m_xs[z.m_ix].m_x.Print("x  = ", false);
                    _ds[z.m_ix].Print("dx = ", false, true);
                    C.Print("Cz = ", false);
                    z.m_z.Print("z  = ", false);
                    UT::Print("ex = %f %f --> %f %f\n", ex.x(), ex.y(),
                              ex.x() * m_K.fx(), ex.y() * m_K.fy());
                    // UT::DebugStop();
                }
#endif
                const float we2 = r2 *
                                  ME::Weight<GBA_M_ESTIMATOR_FUNCTION>(r2) *
                                  GBA_WEIGHT_FEATURE;
                const FTR::Measurement::ESIndex idx(
                    _KF.m_T.m_iFrm, z.m_ix /*, KF.m_T.m_iFrm, iz*/);
                const bool r = r2 < r2Max;
#ifdef CFG_DEPTH_MAP
                if (vd)
                    ES.Accumulate(m_K, ex, ed, we2, idx, r);
                else
#endif
                    ES.Accumulate(m_K, ex, we2, idx, r);
            }
        }
    }
    return ES;
}

Camera::Pose::Prior::Rigid::ES GlobalBundleAdjustor::ComputeErrorStatisticPrior(
    const AlignedVector<Rigid3D> &Cs)
{
    Camera::Pose::Prior::Rigid::ES ES;
    ES.Initialize();

    const int nKFs = int(m_KFs.size());
    for (int iKF2 = 0; iKF2 < nKFs; ++iKF2) {
        KeyFrame &KF2 = m_KFs[iKF2];
        const int Np = int(KF2.m_iKFsPrior.size());
        if (Np == 0) continue;
        const Rigid3D &C2 = Cs[iKF2];
        const Point3D p2 = C2.GetPosition();
        for (int ip = 0; ip < Np; ++ip) {
            const int iKF1 = KF2.m_iKFsPrior[ip];
            const Camera::Pose::Prior::Rigid &Zp = KF2.m_Zps[ip];
            const Camera::Pose::Prior::Rigid::Error e =
                Zp.GetError(Cs[iKF1], C2, p2);
#if 0
//#if 1
			if(m_KFs[iKF1].m_T.m_iFrm == 0 && KF2.m_T.m_iFrm == 84)
			{
				UT::PrintSeparator();
				Cs[iKF1].Print();
				UT::PrintSeparator();
				C2.Print();
				p2.Print();
				exit(0);
			}
#endif
            const LA::AlignedVector3f Wer =
                Zp.m_Wrr * e.m_er + Zp.m_Wrp * e.m_ep;
            const LA::AlignedVector3f Wep =
                Zp.m_Wpr * e.m_er + Zp.m_Wpp * e.m_ep;
            const float we2 =
                (e.m_er.Dot(Wer) + e.m_ep.Dot(Wep)) * GBA_WEIGHT_PRIOR;
            ES.Accumulate(e, we2, Camera::Pose::Prior::Rigid::ESIndex(
                                      m_KFs[iKF1].m_T.m_iFrm, KF2.m_T.m_iFrm));
        }
    }
    return ES;
}
