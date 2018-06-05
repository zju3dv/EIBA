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

#include "GlobalBundleAdjustor.h"
#include "MatrixMxN.h"

#ifdef CFG_DEBUG
//#define GBA_DEBUG_EIGEN_GRADIENT_FEATURE
//#define GBA_DEBUG_EIGEN_GRADIENT_PRIOR
void GlobalBundleAdjustor::DebugGenerateTracks()
{
    const int nKFs = int(m_KFs.size());
    e_Xs.resize(nKFs);
    e_M.resize(nKFs);
    e_I.resize(nKFs);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const int Nx = int(KF.m_xs.size());
        std::vector<Track> &Xs = e_Xs[iKF];
        Xs.resize(Nx);
        for (int ix = 0; ix < Nx; ++ix) Xs[ix].Initialize();
        e_M[iKF].assign(nKFs, 0);
        e_I[iKF].assign(nKFs, -1);
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            std::vector<Track> &Xs = e_Xs[Z.m_iKF];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz)
                Xs[KF.m_zs[iz].m_ix].m_zs.push_back(
                    Track::Measurement(iKF, iz));
        }
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        e_M[iKF][iKF] = 1;
        const std::vector<Track> &Xs = e_Xs[iKF];
        const int Nx = int(Xs.size());
        for (int ix = 0; ix < Nx; ++ix) {
            const Track &X = Xs[ix];
            const int Nz = int(X.m_zs.size());
            for (int iz = 0; iz < Nz; ++iz) e_M[iKF][X.m_zs[iz].m_iKF] = 1;
            for (int iz1 = 0; iz1 < Nz; ++iz1) {
                const int iKF1 = X.m_zs[iz1].m_iKF;
                for (int iz2 = iz1 + 1; iz2 < Nz; ++iz2) {
                    const int iKF2 = X.m_zs[iz2].m_iKF;
                    e_M[iKF1][iKF2] = 1;
                }
            }
        }
        const KeyFrame &KF = m_KFs[iKF];
        const int Np = int(KF.m_iKFsPrior.size());
        for (int ip = 0; ip < Np; ++ip) e_M[KF.m_iKFsPrior[ip]][iKF] = 1;
    }
    for (int iKF1 = 0, i = 0; iKF1 < nKFs; ++iKF1)
        for (int iKF2 = iKF1; iKF2 < nKFs; ++iKF2) {
            if (e_M[iKF1][iKF2]) e_I[iKF1][iKF2] = i++;
        }
}

void GlobalBundleAdjustor::DebugUpdateFactors()
{
    float r2;
    EigenMatrix2x2f Wx;
#ifdef CFG_DEPTH_MAP
    EigenMatrix3x3f Wxd;
#endif
    EigenFactor::Feature Ax;
    EigenFactor::Prior Ap;
    const int nKFs = int(m_KFs.size()), iKFLast = nKFs - 1,
              Ncc = e_I[iKFLast][iKFLast] + 1;
    e_SAccs.resize(Ncc);
    for (int icc = 0; icc < Ncc; ++icc) e_SAccs[icc].setZero();
    e_Sbcs.resize(nKFs);
    for (int iKF = 0; iKF < nKFs; ++iKF) e_Sbcs[iKF].setZero();
    const float wd = 1.0f / DEPTH_MAP_VARIANCE;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        // const FTR::Factor::Source *SAdcxs = m_SAdcxs.Data() + KF.m_id;
        const DepthInverseGaussian *ds = m_dsLP.data() + KF.m_id;
        std::vector<Track> &Xs = e_Xs[iKF];
        const int Nx = int(Xs.size());
        for (int ix = 0; ix < Nx; ++ix) {
            Track &X = Xs[ix];
            X.m_SAdcx.MakeZero();
            const int Nz = int(X.m_zs.size());
            X.m_Adczs.resize(Nz);
            for (int iz = 0; iz < Nz; ++iz) {
                const Track::Measurement &z = X.m_zs[iz];
                const EigenErrorJacobian::Feature Je =
                    DebugEigenErrorJacobianFeature(iKF, z.m_iKF, z.m_iz);
                const FTR::Measurement &_z = m_KFs[z.m_iKF].m_zs[z.m_iz];
#ifdef CFG_DEPTH_MAP
                const bool vd = _z.m_d != 0.0f;
                if (vd) {
                    Wxd = EigenMatrix3x3f(_z.m_W, wd);
                    r2 = Je.m_exd.dot(Wxd * Je.m_exd);
                } else
#endif
                {
                    Wx = EigenMatrix2x2f(_z.m_W);
                    r2 = Je.m_ex.dot(Wx * Je.m_ex);
                }
                const float w =
                    ME::Weight<ME::FUNCTION_HUBER>(r2) * GBA_WEIGHT_FEATURE;
#ifdef CFG_DEPTH_MAP
                if (vd) {
                    Wxd *= w;
                    const EigenMatrix3x13f Jxd =
                        EigenMatrix3x13f(Je.m_Jxdd, Je.m_Jxdcx, Je.m_Jxdcz);
                    const EigenMatrix3x14f Jexd =
                        EigenMatrix3x14f(Jxd, Je.m_exd);
                    const EigenMatrix3x13f WJxd = Wxd * Jxd;
                    Ax = WJxd.transpose() * Jexd;
                } else
#endif
                {
                    Wx *= w;
                    const EigenMatrix2x13f Jx =
                        EigenMatrix2x13f(Je.m_Jxd, Je.m_Jxcx, Je.m_Jxcz);
                    const EigenMatrix2x14f Jex = EigenMatrix2x14f(Jx, Je.m_ex);
                    const EigenMatrix2x13f WJx = Wx * Jx;
                    Ax = WJx.transpose() * Jex;
                }
                Ax.AssertEqual(m_KFs[z.m_iKF], z.m_iz);
                Ax.Set(m_KFs[z.m_iKF], z.m_iz);

                X.m_SAdcx += Ax.m_Adcx;
                X.m_Adczs[iz] = Ax.m_Adcz;
                const int icxx = e_I[iKF][iKF], icxz = e_I[iKF][z.m_iKF],
                          iczz = e_I[z.m_iKF][z.m_iKF];
                e_SAccs[icxx] += Ax.m_Acxx;
                e_SAccs[icxz] += Ax.m_Acxz;
                e_SAccs[iczz] += Ax.m_Aczz;
                e_Sbcs[iKF] += Ax.m_bcx;
                e_Sbcs[z.m_iKF] += Ax.m_bcz;
            }
            if (iKF == iKFLast) continue;
#ifdef CFG_DEPTH_MAP
            const float d = KF.m_xs[ix].m_d;
            if (d != 0.0f) {
                const float ed = ds[ix].u() - d;
                r2 = wd * ed * ed;
                const float w = ME::Weight<ME::FUNCTION_HUBER>(r2) * wd *
                                GBA_WEIGHT_FEATURE;
                X.m_SAdcx.m_add += w;
                X.m_SAdcx.m_bd += w * ed;
            }
#endif
            // X.m_SAdcx.AssertEqual(SAdcxs[ix]);
            // X.m_SAdcx.Set(SAdcxs[ix]);
            X.m_SAdcx.AssertEqual(KF.m_SAdcxs[ix]);
            X.m_SAdcx.Set(KF.m_SAdcxs[ix]);
        }
    }
#if 0
//#if 1
	if(UT::Debugging())
	{
		const EigenMatrix6x6f &A = e_SAccs[e_I[10][10]];
		UT::PrintSeparator('*');
		A.Print();
		UT::PrintSeparator('*');
	}
#endif
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const int Np = int(KF.m_iKFsPrior.size());
        for (int ip = 0; ip < Np; ++ip) {
            const Camera::Pose::Prior::Rigid &Zp = KF.m_Zps[ip];
            const EigenMatrix6x6f Wp =
                EigenMatrix6x6f(Zp.m_Wrr, Zp.m_Wrp, Zp.m_Wpr, Zp.m_Wpp) *
                GBA_WEIGHT_PRIOR;
            const int _iKF = KF.m_iKFsPrior[ip];
            const EigenErrorJacobian::Prior Je =
                DebugEigenErrorJacobianPrior(_iKF, iKF, ip);
            const EigenMatrix6x12f Jp =
                EigenMatrix6x12f(Je.m_Jrpc1, Je.m_Jrpc2);
            const EigenMatrix6x13f Jep = EigenMatrix6x13f(Jp, Je.m_erp);
            const EigenMatrix6x12f WJp = Wp * Jp;
            Ap = WJp.transpose() * Jep;
            Ap.AssertEqual(KF, ip);
            Ap.Set(KF, ip);

            const int ic11 = e_I[_iKF][_iKF], ic12 = e_I[_iKF][iKF],
                      ic22 = e_I[iKF][iKF];
            e_SAccs[ic11] += Ap.m_Ap11;
            e_SAccs[ic12] += Ap.m_Ap12;
            e_SAccs[ic22] += Ap.m_Ap22;
            e_Sbcs[_iKF] += Ap.m_bp1;
            e_Sbcs[iKF] += Ap.m_bp2;
        }
    }
#if 0
//#if 1
	if(UT::Debugging())
	{
		const float epsAbs = 1.0e-3f, epsRel = 1.0e-3f;
		for(int iKF = 0; iKF < nKFs; ++iKF)
		{
			const int icu = e_I[iKF][iKF];
			const Camera::Pose::Unitary SAcu(e_SAccs[icu].GetSymmetricMatrix6x6f(), e_Sbcs[iKF].GetVector6f());
			SAcu.AssertEqual(m_SAcus[iKF], 1, epsAbs, epsRel);
			e_SAccs[icu] = m_SAcus[iKF].m_A;
			e_Sbcs[iKF] = m_SAcus[iKF].m_b;

			const KeyFrame &KF = m_KFs[iKF];
			const Camera::Pose::Binary *SAcbs = m_SAcbs.Data() + KF.m_iK;
			const int Nk = int(KF.m_ik2KF.size());
			for(int ik = 0; ik < Nk; ++ik)
			{
				const int _iKF = KF.m_ik2KF[ik], icb = e_I[_iKF][iKF];
				const Camera::Pose::Binary SAcb(e_SAccs[icb].GetAlignedMatrix6x6f());
				SAcb.AssertEqual(SAcbs[ik], 1, epsAbs, epsRel);
				e_SAccs[icb] = SAcbs[ik];
			}
		}
	}
#endif
}

void GlobalBundleAdjustor::DebugUpdateSchurComplement()
{
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const std::vector<Track> &Xs = e_Xs[iKF];
        const int Nx = int(Xs.size());
        for (int ix = 0; ix < Nx; ++ix) {
            const Track &X = Xs[ix];
            // if(X.m_SAdcx.m_add < FLT_EPSILON)
            //	continue;
            const float mdd =
                X.m_SAdcx.m_add == 0.0f ? 0.0f : 1.0f / X.m_SAdcx.m_add;
            const EigenVector6f mdcxT = X.m_SAdcx.m_adcx.transpose() * mdd;
            const int icxx = e_I[iKF][iKF];
            e_SAccs[icxx] -= mdcxT * X.m_SAdcx.m_adcx;
            e_Sbcs[iKF] -= mdcxT * X.m_SAdcx.m_bd;
#if 0
//#if 1
			if(UT::Debugging())
			{
				if(iKF == 10)
				{
					if(ix == 0)
						UT::PrintSeparator();
					UT::Print("%d ", ix);
					e_Sbcs[iKF].Print();
				}
				continue;
			}
#endif
            const int Nz = int(X.m_zs.size());
            for (int iz = 0; iz < Nz; ++iz) {
                const int icxz = e_I[iKF][X.m_zs[iz].m_iKF];
                e_SAccs[icxz] -= mdcxT * X.m_Adczs[iz];
            }
            for (int iz1 = 0; iz1 < Nz; ++iz1) {
                const int iKF1 = X.m_zs[iz1].m_iKF;
                const EigenVector6f mdczT = X.m_Adczs[iz1].transpose() * mdd;
                for (int iz2 = iz1; iz2 < Nz; ++iz2) {
                    const int iKF2 = X.m_zs[iz2].m_iKF, iczz = e_I[iKF1][iKF2];
                    e_SAccs[iczz] -= mdczT * X.m_Adczs[iz2];
                }
                e_Sbcs[iKF1] -= mdczT * X.m_SAdcx.m_bd;
            }
        }
    }
    const float epsAbs = 1.0e-3f, epsRel = 1.0e-3f;
    e_M.resize(nKFs);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const int icu = e_I[iKF][iKF];
        const Camera::Pose::Unitary SAcu(e_SAccs[icu].GetSymmetricMatrix6x6f(),
                                         e_Sbcs[iKF].GetVector6f());
        SAcu.AssertEqual(m_SAcus[iKF], 1, epsAbs, epsRel);
        e_M[iKF].assign(nKFs, 0);
        e_M[iKF][iKF] = 1;

        const KeyFrame &KF = m_KFs[iKF];
        const Camera::Pose::Binary *SAcbs = m_SAcbs.Data() + KF.m_iK;
        const int Nk = int(KF.m_ik2KF.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const int _iKF = KF.m_ik2KF[ik], icb = e_I[_iKF][iKF];
            const Camera::Pose::Binary SAcb(
                e_SAccs[icb].GetAlignedMatrix6x6f());
            SAcb.AssertEqual(SAcbs[ik], 1, epsAbs, epsRel);
            e_M[_iKF][iKF] = 1;
        }
    }
    for (int iKF1 = 0; iKF1 < nKFs; ++iKF1)
        for (int iKF2 = iKF1; iKF2 < nKFs; ++iKF2) {
            const ubyte m = e_M[iKF1][iKF2];
            const int i = e_I[iKF1][iKF2];
            UT_ASSERT(m == 0 && i == -1 || m != 0 && i != -1);
        }
}

void GlobalBundleAdjustor::DebugUpdateStates()
{
    const int nKFs = int(m_KFs.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const int icu = e_I[iKF][iKF];
        if (iKF == m_iKFFix) {
            e_SAccs[icu] = m_SAcus[nKFs].m_A;
            e_Sbcs[iKF] = m_SAcus[nKFs].m_b;
        } else {
            e_SAccs[icu] = m_SAcus[iKF].m_A;
            e_Sbcs[iKF] = m_SAcus[iKF].m_b;
        }
        const KeyFrame &KF = m_KFs[iKF];
        const Camera::Pose::Binary *SAcbs = m_SAcbs.Data() + KF.m_iK;
        const int Nk = int(KF.m_ik2KF.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const int _iKF = KF.m_ik2KF[ik], icb = e_I[_iKF][iKF];
            e_SAccs[icb] = SAcbs[ik];
        }
    }

    e_xcs.resize(nKFs);
    const LA::Vector6f *xcs = (LA::Vector6f *)m_xcs.Data();
    for (int iKF = 0; iKF < nKFs; ++iKF) e_xcs[iKF] = EigenVector6f(xcs[iKF]);
#if 0
	const float eps1 = 0.01f, eps2 = 0.1f;
	for(int iKF1 = 0; iKF1 < nKFs; ++iKF1)
	{
		EigenVector6f Sb;
		Sb.setZero();
		for(int iKF0 = 0; iKF0 < iKF1; ++iKF0)
		{
			const int icc = e_I[iKF0][iKF1];
			if(icc != -1)
				Sb += e_SAccs[icc].transpose() * e_xcs[iKF0];
		}
		for(int iKF2 = iKF1; iKF2 < nKFs; ++iKF2)
		{
			const int icc = e_I[iKF1][iKF2];
			if(icc != -1)
				Sb += e_SAccs[icc] * e_xcs[iKF2];
		}
		Sb = -Sb;
		Sb.AssertEqual(e_Sbcs[iKF1], 1, std::max(eps1, Sb.norm() * eps2));
	}
#endif
    const int iKFLast = nKFs - 1;
    for (int iKF = 0; iKF < iKFLast; ++iKF) {
        const float *xds = m_xds.Data() + m_KFs[iKF].m_id;
        const EigenVector6f &xc = e_xcs[iKF];
        const std::vector<Track> &Xs = e_Xs[iKF];
        const int Nx = int(Xs.size());
        for (int ix = 0; ix < Nx; ++ix) {
            const Track &X = Xs[ix];
            float Sb = X.m_SAdcx.m_add * xds[ix] + X.m_SAdcx.m_adcx.dot(xc);
            const int Nz = int(X.m_zs.size());
            for (int iz = 0; iz < Nz; ++iz)
                Sb += X.m_Adczs[iz].dot(e_xcs[X.m_zs[iz].m_iKF]);
            Sb = -Sb;
            UT::AssertEqual(Sb, X.m_SAdcx.m_bd, 1);
        }
    }
}

GlobalBundleAdjustor::EigenErrorJacobian::Feature
GlobalBundleAdjustor::DebugEigenErrorJacobianFeature(const int iKF1,
                                                     const int iKF2,
                                                     const int iz)
{
    const KeyFrame &KF1 = m_KFs[iKF1], &KF2 = m_KFs[iKF2];
    const Rigid3D &C1 = m_CsLP[iKF1], &C2 = m_CsLP[iKF2];
    const FTR::Measurement &z = KF2.m_zs[iz];
    const Rigid3D Tr = C2 / C1;
#ifdef CFG_DEPTH_MAP
    const bool vd = z.m_d != 0.0f;
#endif;

    const EigenRotation3D e_R1 = EigenRotation3D(C1),
                          e_R2 = EigenRotation3D(C2);
    const EigenPoint3D e_p1 = EigenVector3f(C1.GetPosition()),
                       e_p2 = EigenVector3f(C2.GetPosition());
    const EigenRotation3D e_R12 = EigenRotation3D(Tr);
    const EigenVector3f e_t12 = EigenVector3f(Tr.GetTranslation());

    const EigenPoint3D e_x1 = EigenPoint2D(KF1.m_xs[z.m_ix].m_x);
    const float e_d1 = m_dsLP[KF1.m_id + z.m_ix].u(), e_z1 = 1.0f / e_d1;
    const EigenPoint3D e_X1 = EigenVector3f(e_x1 * e_z1);
    const EigenPoint3D e_X2 = EigenVector3f(e_R12 * e_X1 + e_t12);
    const EigenPoint2D e_x2 = e_X2.Project();
    const float e_d2 = 1.0f / e_X2.z();
    const EigenVector2f e_ex = e_x2 - EigenVector2f(z.m_z);
    const float e_ed =
#ifdef CFG_DEPTH_MAP
        vd ? e_d2 - z.m_d :
#endif
           0.0f;
    const EigenVector3f e_JXd = -e_R12 * e_x1 * e_z1 * e_z1;
    const EigenMatrix3x3f e_JXr1 = -e_R12 * EigenSkewSymmetricMatrix(e_X1);
    const EigenMatrix3x3f e_JXp1 = e_R2;
    const EigenMatrix3x3f e_JXr2 = EigenSkewSymmetricMatrix(e_X2);
    const EigenMatrix3x3f e_JXp2 = -e_R2;
    const EigenMatrix2x3f e_JxX = e_X2.GetJacobianProjection();
    const EigenVector2f e_Jxd = e_JxX * e_JXd;
    const EigenMatrix2x3f e_Jxr1 = e_JxX * e_JXr1;
    const EigenMatrix2x3f e_Jxp1 = e_JxX * e_JXp1;
    const EigenMatrix2x3f e_Jxr2 = e_JxX * e_JXr2;
    const EigenMatrix2x3f e_Jxp2 = e_JxX * e_JXp2;
    const float e_jdz = -e_d2 * e_d2;
    Eigen::Matrix<float, 1, 3> e_jzX;
    e_jzX(0, 0) = 0.0f;
    e_jzX(0, 1) = 0.0f;
    e_jzX(0, 2) = 1.0f;
    const Eigen::Matrix<float, 1, 3> e_jdX = e_jdz * e_jzX;
    const float e_jdd = e_jdX.dot(e_JXd);
    const Eigen::Matrix<float, 1, 3> e_jdr1 = e_jdX * e_JXr1;
    const Eigen::Matrix<float, 1, 3> e_jdp1 = e_jdX * e_JXp1;
    const Eigen::Matrix<float, 1, 3> e_jdr2 = e_jdX * e_JXr2;
    const Eigen::Matrix<float, 1, 3> e_jdp2 = e_jdX * e_JXp2;
#ifdef GBA_DEBUG_EIGEN_GRADIENT_FEATURE
    // const float e_ddMax = 0.0f;
    const float e_ddMax = 0.01f;
    // const float e_dr1Max = 0.0f;
    const float e_dr1Max = 1.0f;
    // const float e_dp1Max = 0.0f;
    const float e_dp1Max = 0.1f;
    // const float e_dr2Max = 0.0f;
    const float e_dr2Max = 1.0f;
    // const float e_dp2Max = 0.0f;
    const float e_dp2Max = 0.1f;
    const float e_dd = UT::Random<float>(-e_ddMax, e_ddMax);
    const EigenVector3f e_dr1 =
        EigenAxisAngle::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD)
            .GetRodrigues();
    const EigenVector3f e_dp1 = EigenVector3f::GetRandom(e_dp1Max);
    const EigenVector3f e_dr2 =
        EigenAxisAngle::GetRandom(e_dr2Max * UT_FACTOR_DEG_TO_RAD)
            .GetRodrigues();
    const EigenVector3f e_dp2 = EigenVector3f::GetRandom(e_dp2Max);
    const float e_d1GT = e_d1 + e_dd, e_z1GT = 1.0f / e_d1GT;
    const EigenRotation3D e_R1GT =
        EigenMatrix3x3f(EigenRotation3D(e_dr1) * e_R1);
    const EigenPoint3D e_p1GT = EigenVector3f(e_p1 + e_dp1);
    const EigenRotation3D e_R2GT =
        EigenMatrix3x3f(EigenRotation3D(e_dr2) * e_R2);
    const EigenPoint3D e_p2GT = EigenVector3f(e_p2 + e_dp2);
    const EigenPoint3D e_X1GT = EigenVector3f(e_x1 * e_z1GT);
    const EigenPoint3D e_X2GT =
        EigenVector3f(e_R2GT * (e_R1GT.transpose() * e_X1GT + e_p1GT - e_p2GT));
    const EigenPoint2D e_x2GT = e_X2GT.Project();
    const float e_d2GT = 1.0f / e_X2GT.z();
    const EigenVector2f e_ex1 = e_x2 - e_x2GT,
                        e_ex2 = e_ex1 + e_Jxd * e_dd + e_Jxr1 * e_dr1 +
                                e_Jxp1 * e_dp1 + e_Jxr2 * e_dr2 +
                                e_Jxp2 * e_dp2;
    UT::CheckReduction(e_ex1, e_ex2);
    const float e_ed1 = e_d2 - e_d2GT, e_ed2 = e_ed1 + e_jdd * e_dd +
                                               e_jdr1 * e_dr1 + e_jdp1 * e_dp1 +
                                               e_jdr2 * e_dr2 + e_jdp2 * e_dp2;
    UT::CheckReduction(e_ed1, e_ed2);
#endif

    EigenErrorJacobian::Feature e_Je;
#ifdef CFG_DEPTH_MAP
    if (vd) {
        e_Je.m_Jxdd = EigenVector3f(e_Jxd, e_jdd);
        e_Je.m_Jxdcx = EigenMatrix3x6f(e_Jxp1, e_Jxr1, e_jdp1, e_jdr1);
        e_Je.m_Jxdcz = EigenMatrix3x6f(e_Jxp2, e_Jxr2, e_jdp2, e_jdr2);
        e_Je.m_exd = EigenVector3f(e_ex, e_ed);
    } else
#endif
    {
        e_Je.m_Jxd = e_Jxd;
        e_Je.m_Jxcx = EigenMatrix2x6f(e_Jxp1, e_Jxr1);
        e_Je.m_Jxcz = EigenMatrix2x6f(e_Jxp2, e_Jxr2);
        e_Je.m_ex = e_ex;
    }
    return e_Je;
}

GlobalBundleAdjustor::EigenErrorJacobian::Prior
GlobalBundleAdjustor::DebugEigenErrorJacobianPrior(const int iKF1,
                                                   const int iKF2, const int ip)
{
    const Camera::Pose::Prior::Rigid &Zp = m_KFs[iKF2].m_Zps[ip];
    const EigenRotation3D e_Rp = EigenMatrix3x3f(Zp.m_R),
                          e_RpT = e_Rp.transpose();
    const EigenPoint3D e_pp = EigenPoint3D(Zp.m_p);
    const Rigid3D &C1 = m_CsLP[iKF1], &C2 = m_CsLP[iKF2];
    const EigenRotation3D e_R1 = EigenRotation3D(C1),
                          e_R2 = EigenRotation3D(C2), e_R1T = e_R1.transpose();
    const EigenPoint3D e_p1 = EigenVector3f(C1.GetPosition()),
                       e_p2 = EigenVector3f(C2.GetPosition());
    const EigenRotation3D e_R12 = e_R2 * e_R1T;
    const EigenRotation3D e_eR = e_R12 * e_RpT;
    const EigenVector3f e_er = e_eR.GetRodrigues();
    const EigenMatrix3x3f e_JrI = EigenMatrix3x3f(
        Rotation3D::GetRodriguesJacobianInverse(e_er.GetAlignedVector3f()));
    const EigenMatrix3x3f e_Jrr1 = -e_JrI * e_R12;
    const EigenMatrix3x3f e_Jrr2 = e_JrI;
    const EigenVector3f e_p12 = e_R1 * (e_p2 - e_p1);
    const EigenVector3f e_ep = e_p12 - e_pp;
    const EigenMatrix3x3f e_Jpr1 = EigenSkewSymmetricMatrix(e_p12);
    const EigenMatrix3x3f e_Jpp1 = -e_R1;
    const EigenMatrix3x3f e_Jpp2 = e_R1;
#ifdef GBA_DEBUG_EIGEN_GRADIENT_PRIOR
    // const float e_dr1Max = 0.0f;
    const float e_dr1Max = 1.0f;
    // const float e_dr2Max = 0.0f;
    const float e_dr2Max = 1.0f;
    // const float e_dp1Max = 0.0f;
    const float e_dp1Max = 0.1f;
    // const float e_dp2Max = 0.0f;
    const float e_dp2Max = 0.1f;
    // const EigenVector3f e_dr1 = EigenAxisAngle::GetRandom(e_dr1Max *
    // UT_FACTOR_DEG_TO_RAD).GetRodrigues();
    // const EigenVector3f e_dr2 = EigenAxisAngle::GetRandom(e_dr2Max *
    // UT_FACTOR_DEG_TO_RAD).GetRodrigues();
    // const EigenRotation3D e_R1GT = EigenMatrix3x3f(EigenRotation3D(e_dr1) *
    // e_R1);
    // const EigenRotation3D e_R2GT = EigenMatrix3x3f(EigenRotation3D(e_dr2) *
    // e_R2);
    EigenVector3f e_dr1, e_dr2;
    EigenRotation3D e_R1GT, e_R2GT;
    if (e_dr1Max == 0.0f && e_dr2Max == 0.0f) {
        e_dr1.setZero();
        e_R1GT = e_R1;
        e_dr2.setZero();
        e_R2GT = e_R2;
    } else if (e_dr1Max == 0.0f) {
        e_dr1.setZero();
        e_R1GT = e_R1;
        e_R2GT = e_Rp * e_R1GT;
        e_dr2 = EigenRotation3D(e_R2GT * e_R2.transpose()).GetRodrigues();
    } else if (e_dr2Max == 0.0f) {
        e_dr2.setZero();
        e_R2GT = e_R2;
        e_R1GT = e_RpT * e_R2GT;
        e_dr1 = EigenRotation3D(e_R1GT * e_R1T).GetRodrigues();
    } else {
        e_dr1 = EigenAxisAngle::GetRandom(e_dr1Max * UT_FACTOR_DEG_TO_RAD)
                    .GetRodrigues();
        e_R1GT = EigenMatrix3x3f(EigenRotation3D(e_dr1) * e_R1);
        e_R2GT = e_Rp * e_R1GT;
        e_dr2 = EigenRotation3D(e_R2GT * e_R2.transpose()).GetRodrigues();
    }
    const EigenVector3f e_dp1 = EigenVector3f::GetRandom(e_dp1Max);
    const EigenVector3f e_dp2 = EigenVector3f::GetRandom(e_dp2Max);
    const EigenPoint3D e_p1GT = EigenVector3f(e_p1 + e_dp1);
    const EigenPoint3D e_p2GT = EigenVector3f(e_p2 + e_dp2);
    const EigenRotation3D e_R12GT = e_R2GT * e_R1GT.transpose();
    const EigenVector3f e_p12GT = e_R1GT * (e_p2GT - e_p1GT);
    const EigenRotation3D e_eRGT = e_R12 * e_R12GT.transpose();
    const EigenVector3f e_er1 = e_eRGT.GetRodrigues(),
                        e_er2 = e_er1 + e_Jrr1 * e_dr1 + e_Jrr2 * e_dr2;
    const EigenVector3f e_ep1 = e_p12 - e_p12GT,
                        e_ep2 = e_ep1 + e_Jpr1 * e_dr1 + e_Jpp1 * e_dp1 +
                                e_Jpp2 * e_dp2;
    UT::CheckReduction(e_er1, e_er2);
    UT::CheckReduction(e_ep1, e_ep2);
#endif
    EigenErrorJacobian::Prior e_Je;
    e_Je.m_Jrpc1 =
        EigenMatrix6x6f(EigenMatrix3x3f::Zero(), e_Jrr1, e_Jpp1, e_Jpr1);
    e_Je.m_Jrpc2 = EigenMatrix6x6f(EigenMatrix3x3f::Zero(), e_Jrr2, e_Jpp2,
                                   EigenMatrix3x3f::Zero());
    e_Je.m_erp = EigenVector6f(e_er, e_ep);
    return e_Je;
}
#endif
