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

#ifdef CFG_DEBUG
#ifdef CFG_DEBUG_EIGEN
#define GBA_DEBUG_EIGEN
#endif
#if defined CFG_DEBUG && defined CFG_VIEW
//#define GBA_DEBUG_VIEW	0
//#define GBA_DEBUG_VIEW	1
#define GBA_DEBUG_VIEW 2
#ifdef GBA_DEBUG_VIEW
#define GBA_DEBUG_VIEW_FRAME 117
#endif
#endif
#ifdef CFG_DEBUG
//#define GBA_DEBUG_ITERATION	3
#endif
#endif

#ifdef CFG_TUNE_PARAMETERS
float GBA_WEIGHT_FEATURE = 1.0e-5f;
float GBA_WEIGHT_PRIOR = 1.0e-3f;
float GBA_WEIGHT_FIX = 1.0e2f;
float GBA_UPDATE_ROTATION = 3.046174198662e-6f;
float GBA_UPDATE_POSITION = 1.0e-4f;
float GBA_UPDATE_DEPTH = 1.0e-4f;
float GBA_CONVERGE_ROTATION = 7.615435234857e-5f;
float GBA_CONVERGE_POSITION = 0.0025f;
float GBA_CONVERGE_DEPTH = 0.0025f;
float GBA_VARIANCE_FIX_ROTATION = 3.046174198662e-6f; // (0.1*pi/180)^2
float GBA_VARIANCE_FIX_POSITION =  0.01f; // 0.1^2
float DEPTH_MAP_VARIANCE = 0.01f; // 0.1^2

extern void LOAD_PARAMETERS_GLOBAL_BUNDLE_ADJUSTMENT(const Configurator &cfgor)
{
    GBA_WEIGHT_FEATURE = cfgor.GetArgument(
        "param_global_bundle_adjustment_weight_feature", GBA_WEIGHT_FEATURE);
    GBA_WEIGHT_PRIOR = cfgor.GetArgument(
        "param_global_bundle_adjustment_weight_prior", GBA_WEIGHT_PRIOR);
    GBA_WEIGHT_FIX = cfgor.GetArgument(
        "param_global_bundle_adjustment_weight_fix", GBA_WEIGHT_FIX);
    GBA_UPDATE_ROTATION =
        cfgor.GetArgument("param_global_bundle_adjustment_update_rotation",
                          sqrt(GBA_UPDATE_ROTATION) * UT_FACTOR_RAD_TO_DEG);
    GBA_UPDATE_ROTATION = GBA_UPDATE_ROTATION * GBA_UPDATE_ROTATION *
                          UT_FACTOR_DEG_TO_RAD * UT_FACTOR_DEG_TO_RAD;
    GBA_UPDATE_POSITION =
        cfgor.GetArgument("param_global_bundle_adjustment_update_position",
                          sqrt(GBA_UPDATE_POSITION));
    GBA_UPDATE_POSITION = GBA_UPDATE_POSITION * GBA_UPDATE_POSITION;
    GBA_UPDATE_DEPTH = cfgor.GetArgument(
        "param_global_bundle_adjustment_update_depth", sqrt(GBA_UPDATE_DEPTH));
    GBA_UPDATE_DEPTH = GBA_UPDATE_DEPTH * GBA_UPDATE_DEPTH;

    GBA_CONVERGE_ROTATION =
        cfgor.GetArgument("param_global_bundle_adjustment_converge_rotation",
                          sqrt(GBA_CONVERGE_ROTATION) * UT_FACTOR_RAD_TO_DEG);
    GBA_CONVERGE_ROTATION = GBA_CONVERGE_ROTATION * GBA_CONVERGE_ROTATION *
                            UT_FACTOR_DEG_TO_RAD * UT_FACTOR_DEG_TO_RAD;
    GBA_CONVERGE_POSITION =
        cfgor.GetArgument("param_global_bundle_adjustment_converge_position",
                          sqrt(GBA_CONVERGE_POSITION));
    GBA_CONVERGE_POSITION = GBA_CONVERGE_POSITION * GBA_CONVERGE_POSITION;
    GBA_CONVERGE_DEPTH =
        cfgor.GetArgument("param_global_bundle_adjustment_converge_depth",
                          sqrt(GBA_CONVERGE_DEPTH));
    GBA_CONVERGE_DEPTH = GBA_CONVERGE_DEPTH * GBA_CONVERGE_DEPTH;
}
#endif

#define GBA_MAX_ITERATIONS 10

int g_verbose = 0;

void GlobalBundleAdjustor::Initialize(GlobalMap *GM,
//                                      CallBackUpdateKeyFrame updateKFCB,
                                      const bool serial, const int verbose)
{
//    MT::Thread::Initialize(serial, "GlobalBundleAdjustor");
    m_GM = GM;
//    m_updateKFCB = updateKFCB;
    m_verbose = verbose;
    g_verbose = m_verbose;
    m_K = GM->m_K;
}

void GlobalBundleAdjustor::Reset()
{
//    MT::Thread::Reset();
    m_timingFileName = "";
    m_fp = NULL;
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_IKFs1.resize(0);
    m_IKFs2.resize(0);
    m_IZps1.resize(0);
    m_IZps2.resize(0);
    MT_WRITE_LOCK_END(m_MT);
    m_iKFFix = -1;
    m_Ucs.resize(0);
    m_Uds.resize(0);
    m_KFs.resize(0);
    m_ucs.resize(0);
    m_uds.resize(0);
    m_Cs.Resize(0);
    m_CsLP.Resize(0);
#ifdef CFG_GROUND_TRUTH
    m_CsGTKF.Resize(0);
#endif
    m_ds.resize(0);
    m_dsLP.resize(0);
    // m_SAdcxs.Resize(0);
    // m_SMdcxs.Resize(0);
    // m_SMcxxs.Resize(0);
    m_SAcus.Resize(0);
    m_SAcbs.Resize(0);
}

void GlobalBundleAdjustor::Run(std::vector<EIBA::KeyFrame> *optKFs)
{
    SynchronizeData();
#ifdef GBA_DEBUG_EIGEN
    DebugGenerateTracks();
#endif

    // reprojection error output
    std::vector<Rigid3D> Cs_tmp;
    float reproject_error_before = 0;
    if (m_verbose) {
        for (int i = 0; i < m_Cs.Size(); ++i) {
            Cs_tmp.push_back(m_Cs[i]);
        }
        const FTR::Measurement::ES ESx1 = ComputeErrorStatisticFeature(Cs_tmp, m_ds);
        reproject_error_before = ESx1.Average();
//        printf("%f", ESx1.Average());
    }


#ifdef CFG_VERBOSE
    const std::string str = "     ";
    if (m_verbose > 1) {
        UT::PrintSeparator('*');
        UT::Print("[%d] Global Bundle Adjustment...\n",
                  m_KFs.back().m_T.m_iFrm);
    }
#endif
#if 0
//#if 1
	const int nKFs = int(m_KFs.size());
	//if(nKFs >= 5)
	{
		UT::Check("Camera Noise\n");
		const float erMax = 10.0f;
		const float epMax = 1.0f;
		const float evMax = 1.0f;
		const float edMax = 0.1f;
		for(int iKF = 0; iKF < nKFs; ++iKF)
		{
			Rigid3D &C = m_Cs[iKF];
			Point3D p = C.GetPosition();
			C = Rotation3D::GetRandom(erMax * UT_FACTOR_DEG_TO_RAD) * C;
			p += LA::AlignedVector3f::GetRandom(epMax);
			C.SetPosition(p);
			m_CsLP[iKF] = C;
			m_ucs[iKF] = 1;
		}
		const int Nd = int(m_ds.size());
		for(int id = 0; id < Nd; ++id)
		{
			m_ds[id].u() += UT::Random<float>(edMax);
			m_dsLP[id] = m_ds[id];
			m_uds[id] = 1;
		}
	}
#endif
#ifdef GBA_DEBUG_VIEW
#ifdef GBA_DEBUG_VIEW_FRAME
    const bool debug =
        !m_KFs.empty() && m_KFs.back().m_T.m_iFrm == GBA_DEBUG_VIEW_FRAME;
#else
    const bool debug = false;
#endif
    const int iFrmActiveBkp = m_IT->m_iFrmActive;
    const int keyDrawCamTypeKFBkp = m_IT->m_keyDrawCamTypeKF;
    const int keyDrawDepTypeBkp = m_IT->m_keyDrawDepType;
    if (debug) {
        UT::DebugStart();
        m_IT->SetPause(true);
        m_IT->ActivateFrame(int(m_KFs.size()) - 1);
        m_IT->m_keyDrawCamTypeKF = ImageTracker::DRAW_CAM_KF_GBA;
        m_IT->m_keyDrawDepType = ImageTracker::DRAW_DEP_GBA;
    }
#endif
    for (int i = 0; i < TM_TYPES; ++i) m_timers[i].Reset();
    m_timers[TM_TOTAL].Start();
    for (int iIter = 0; iIter < GBA_MAX_ITERATIONS; ++iIter) {
#ifdef CFG_VERBOSE
        if (m_verbose > 1) {
            std::vector<Rigid3D> Cs_tmp;
            Cs_tmp.resize(0);
            for (int i = 0; i < m_Cs.Size(); ++i) {
                Cs_tmp.push_back(m_Cs[i]);
            }
            const FTR::Measurement::ES ESx =
                ComputeErrorStatisticFeature(Cs_tmp, m_ds);
            const Camera::Pose::Prior::Rigid::ES ESp =
                ComputeErrorStatisticPrior(m_Cs);
            // ESx.Print(iIter == 0 ? str0 : str);
            if (m_verbose >= 2)
                UT::PrintSeparator('*');
            else if (iIter == 0)
                UT::PrintSeparator();
            if (ESx.Valid()) {
                UT::Print("*%02d: ", iIter);
                ESx.Print();
            }
            if (ESp.Valid()) {
                ESp.Print(str);
                UT::Print("%se  = %.2e\n", str.c_str(),
                          ESx.m_Swe2 + ESp.m_Swe2);
            }
        }
#endif
#if defined GBA_DEBUG_VIEW && GBA_DEBUG_VIEW >= 2
        if (debug) m_IT->RunView();
#endif
#ifdef GBA_DEBUG_ITERATION
        const bool debug = iIter == GBA_DEBUG_ITERATION;
        if (debug) {
            UT::DebugStart();
#if 0
			const int nKFs = int(m_KFs.size());
			for(int iKF = 0; iKF < nKFs; ++iKF)
			{
				KeyFrame &KF = m_KFs[iKF];
				KF.m_Adcxs.MakeZero();
				KF.m_Adczs.MakeZero();
				KF.m_Acxxs.MakeZero();
				KF.m_Acxzs.MakeZero();
				KF.m_Aczzs.MakeZero();
				KF.m_Mdczs.MakeZero();
				KF.m_Mcxzs.MakeZero();
				KF.m_Mczzs.MakeZero();
				KF.m_Ap11s.MakeZero();
				KF.m_Ap12s.MakeZero();
				KF.m_Ap22s.MakeZero();
				KF.m_Zm.m_Mczms.MakeZero();
			}
			m_SAdcxs.MakeZero();
			m_SMdcxs.MakeZero();
			m_SMcxxs.MakeZero();
			m_SAcus.MakeZero();
			m_SAcbs.MakeZero();
#endif
        }
#endif
        m_timers[TM_FACTOR].Start();
        UpdateFactors();
        m_timers[TM_FACTOR].Stop();
#ifdef GBA_DEBUG_EIGEN
        DebugUpdateFactors();
#endif
        m_timers[TM_SCHUR_COMPLEMENT].Start();
        UpdateSchurComplement();
        m_timers[TM_SCHUR_COMPLEMENT].Stop();
#ifdef GBA_DEBUG_EIGEN
        DebugUpdateSchurComplement();
#endif
#if 0
//#ifdef GBA_DEBUG_ITERATION
		if(debug)
			AssertConsistency();
#endif
        m_timers[TM_CAMERA_STATE].Start();
        const bool uc = UpdateCameraStates();
        m_timers[TM_CAMERA_STATE].Stop();
        m_timers[TM_DEPTH_STATE].Start();
        const bool ud = UpdateDepthStates();
        m_timers[TM_DEPTH_STATE].Stop();
#ifdef GBA_DEBUG_EIGEN
        DebugUpdateStates();
#endif
#ifdef GBA_DEBUG_ITERATION
        if (debug) UT::DebugStop();
#endif
        if (!uc && !ud || !BufferDataEmpty()) break;
    }
    m_timers[TM_TOTAL].Stop();
    for (int i = 0; i < TM_TYPES; ++i) m_timers[i].Finish();
    if (m_timingFileName != "") {
        if (!m_fp) m_fp = fopen(m_timingFileName.c_str(), "w");
        for (int i = 0; i < TM_TYPES; ++i)
            fprintf(m_fp, "%f ", m_timers[i].GetAverageSeconds() * 1000);
        fprintf(m_fp, "\n");
    }
#if defined GBA_DEBUG_VIEW && GBA_DEBUG_VIEW >= 1
    if (debug) {
        m_IT->RunView();
        m_IT->ActivateFrame(iFrmActiveBkp);
        m_IT->m_keyDrawCamTypeKF = keyDrawCamTypeKFBkp;
        m_IT->m_keyDrawDepType = keyDrawDepTypeBkp;
        UT::DebugStop();
    }
#endif

    // reprojection error output
    if (m_verbose) {
        Cs_tmp.resize(0);
        for (int i = 0; i < m_Cs.Size(); ++i) {
            Cs_tmp.push_back(m_Cs[i]);
        }
        const FTR::Measurement::ES ESx2 =
                ComputeErrorStatisticFeature(Cs_tmp, m_ds);
        UT::Print("[%d] Reprojection Error: %f--> %f\n",
                  m_KFs.back().m_T.m_iFrm,
                  reproject_error_before, ESx2.Average());
    }

    UpdateData(optKFs);
}

void GlobalBundleAdjustor::Stop()
{
    if (m_fp) {
        fclose(m_fp);
        UT::PrintSaved(m_timingFileName);
    }
#if 0
//#if 1
	const int nKFs = int(m_KFs.size());
	for(int iKF = 0; iKF < nKFs; ++iKF)
	{
		const int _iKF = iKF - 1;
		const KeyFrame &KF = m_KFs[iKF];
		const int iZ = KF.SearchFrameMeasurement(_iKF), Nm1 = iZ == -1 ? 0 : KF.m_Zs[iZ].m_izIP[FRM_IP_LEVELS] - KF.m_Zs[iZ].m_izIP[0];
		const int ik = KF.SearchKeyFrameMatch(_iKF), Nm2 = ik == -1 ? 0 : KF.m_Zm.m_ik2zm[ik + 1] - KF.m_Zm.m_ik2zm[ik];
		UT::Print("%d: %d\n", iKF, Nm1 + Nm2);
	}
#endif
//    MT::Thread::Stop();
}

void GlobalBundleAdjustor::PushKeyFrame(
    const GlobalMap::KeyFrame &KF, const Rigid3D &C,
    const std::vector<DepthInverseGaussian> &dxs,
    const std::vector<DepthInverseGaussian> &dzs, const int iKFFix)
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_IKFs1.push_back(InputKeyFrame(KF, C, dxs, dzs, iKFFix));
    MT_WRITE_LOCK_END(m_MT);
}

void GlobalBundleAdjustor::PushKeyFramePriors(
    const std::vector<GlobalMap::KeyFramePrior> &Zps)
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_IZps1.insert(m_IZps1.end(), Zps.begin(), Zps.end());
    MT_WRITE_LOCK_END(m_MT);
}

void GlobalBundleAdjustor::SynchronizeData()
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_IKFs1.swap(m_IKFs2);
    m_IZps1.swap(m_IZps2);
    MT_WRITE_LOCK_END(m_MT);

    const int nKFs1 = int(m_KFs.size()), nKFs2 = nKFs1 + int(m_IKFs2.size());
    m_KFs.resize(nKFs2);
    m_ucs.resize(nKFs2, 1);
    m_Ucs.resize(nKFs2, 1);
    m_Cs.Resize(nKFs2, true);
    m_CsLP.Resize(nKFs2 + 1, true);
#ifdef CFG_GROUND_TRUTH
    if (!m_CsGT.Empty()) m_CsGTKF.Resize(nKFs2, true);
#endif
    // const int Nd1 = int(m_ds.size());
    const int NK1 = m_SAcbs.Size();
    int SNk = NK1;
    std::list<InputKeyFrame>::const_iterator IKF = m_IKFs2.begin();
    for (int iKF = nKFs1; iKF < nKFs2; ++iKF, ++IKF) {
        KeyFrame &KF = m_KFs[iKF];
        KF.Initialize(*IKF, SNk);
        PushFeatureMeasurementMatchesFirst(KF, m_idxsTmp1, m_idxsTmp2);
        const int Nk = int(KF.m_iKFsMatch.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const int _iKF = KF.m_iKFsMatch[ik];
            PushFeatureMeasurementMatchesNext(m_KFs[_iKF],m_idxsTmp1,
                                              m_idxsTmp2, KF.m_Zm);
        }
        SNk += int(KF.m_ik2KF.size());
#ifdef CFG_DEBUG
        UT_ASSERT(KF.m_iKFsPrior.empty());
#endif

        m_Cs[iKF] = IKF->m_C;
        m_CsLP[iKF] = IKF->m_C;
#ifdef CFG_GROUND_TRUTH
        if (!m_CsGT.Empty()) m_CsGTKF[iKF] = m_CsGT[KF.m_T.m_iFrm].m_T;
#endif
        m_ds.insert(m_ds.end(), IKF->m_dxs.begin(), IKF->m_dxs.end());
        m_dsLP.insert(m_dsLP.end(), IKF->m_dxs.begin(), IKF->m_dxs.end());
        m_uds.resize(m_ds.size(), 1);
        m_Uds.resize(m_ds.size(), 1);
#ifdef CFG_DEBUG
        UT_ASSERT(IKF->m_id == int(m_ds.size()));
        UT_ASSERT(IKF->m_dzs.size() == IKF->m_zs.size());
#endif

        const int NZ = int(IKF->m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = IKF->m_Zs[iZ];
            const KeyFrame &_KF = m_KFs[Z.m_iKF];
            DepthInverseGaussian *ds = m_ds.data() + _KF.m_id,
                                 *dsLP = m_dsLP.data() + _KF.m_id;
            ubyte *uds = m_uds.data() + _KF.m_id;
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
                const int ix = IKF->m_zs[iz].m_ix;
                ds[ix] = dsLP[ix] = IKF->m_dzs[iz];
                uds[ix] = 1;
            }
        }

        // m_iKFFix = IKF->m_iKFFix;
        // m_CsLP[nKFs2] = IKF->m_C;
    }
    // const int Nd2 = int(m_ds.size());
    // m_SAdcxs.Resize(Nd2, true);			m_SAdcxs.MakeZero(Nd1,
    // Nd2);
    // m_SMdcxs.Resize(Nd2, true);			m_SMdcxs.MakeZero(Nd1,
    // Nd2);
    // m_SMcxxs.Resize(Nd2, true);			m_SMcxxs.MakeZero(Nd1,
    // Nd2);
    const int iKFLast = nKFs2 - 1;
    for (int iKF = nKFs1 == 0 ? 0 : nKFs1 - 1; iKF < iKFLast; ++iKF) {
        KeyFrame &KF = m_KFs[iKF];
        const int Nx = int(KF.m_xs.size());
        KF.m_SAdcxs.Resize(Nx);
        KF.m_SAdcxs.MakeZero();
        KF.m_SMdcxs.Resize(Nx);
        KF.m_SMdcxs.MakeZero();
        KF.m_SMcxxs.Resize(Nx);
        KF.m_SMcxxs.MakeZero();
    }
    const int NK2 = SNk;
    // m_SAcus.Resize(nKFs2, true)	;	m_SAcus.MakeZero(nKFs1, nKFs2);
    m_SAcus.Resize(nKFs2 + 1, true);
    m_SAcus.MakeZero(nKFs1, nKFs2);
    m_SAcbs.Resize(NK2, true);
    m_SAcbs.MakeZero(NK1, NK2);
    m_IKFs2.resize(0);
#if 0
//#if 1
	if(m_KFs.back().m_T.m_iFrm == 57)
		UT::Check();
#endif
    bool loop = false;
    for (std::list<GlobalMap::KeyFramePrior>::const_iterator IZp =
             m_IZps2.begin();
         IZp != m_IZps2.end(); ++IZp) {
        KeyFrame &KF = m_KFs[IZp->m_iKF2];
        const int Nk = int(KF.m_ik2KF.size());
        const int ip = KF.PushKeyFramePrior(IZp->m_iKF1, IZp->m_Zp, m_ZpsTmp,
                                            m_AcusTmp, m_AcbsTmp);
        if (ip == -1) continue;
        m_ucs[IZp->m_iKF1] = m_ucs[IZp->m_iKF2] = 1;
        if (int(KF.m_ik2KF.size()) == Nk) continue;
        loop = true;
        const int iK = KF.m_iK + KF.m_iZp2k[ip];
        m_SAcbs.Insert(iK, 1, m_AcbsTmp);
        m_SAcbs[iK].MakeZero();
        for (int iKF = IZp->m_iKF2 + 1; iKF < nKFs2; ++iKF) ++m_KFs[iKF].m_iK;
    }
    m_IZps2.resize(0);
#if 1
#if 0
//#if 1
	if(loop)
		m_iKFFix = nKFs2 - 1;
	else
#endif
		m_iKFFix = 0;
	m_CsLP[nKFs2] = m_CsLP[m_iKFFix];
#endif
#if 0
    if (loop) {
        m_verbose = 1;
        UT::PrintSeparator('!');
        UT::PrintSeparator('!');
        UT::PrintSeparator('!');
        UT::PrintSeparator('!');
        UT::PrintSeparator('!');
    } else
        m_verbose = g_verbose;
#endif
#if 0
//#if 1
	AssertConsistency();
#endif
}

void GlobalBundleAdjustor::UpdateData(std::vector<EIBA::KeyFrame> *KFs)
{
    const int nKFs = int(m_KFs.size()), Nd = int(m_ds.size());
    m_GM->GBA_Update(m_Cs, m_ds);
#if 0
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        if (m_Ucs[iKF]) (*m_updateKFCB)(m_KFs[iKF].m_T.m_iFrm, m_Cs[iKF]);
    }
#endif

    if (KFs != nullptr) {
        KFs->resize(nKFs);
        for (size_t iKF = 0; iKF < nKFs; ++iKF) {
            auto &KF_to_set = (*KFs)[iKF];
            // set keyframe rotation and position
            Rotation3D(m_Cs[iKF]).Get(KF_to_set.m_camera.m_rotation[0]);

            float tmp_pos[3];
            m_Cs[iKF].GetPosition().Get((float *) &tmp_pos);
            KF_to_set.m_camera.m_position.m_x = tmp_pos[0];
            KF_to_set.m_camera.m_position.m_y = tmp_pos[1];
            KF_to_set.m_camera.m_position.m_z = tmp_pos[2];

            KF_to_set.m_camera_updated = m_Ucs[iKF] != 0;

            // set keyframe source inverse depth
            // only provide last frame Keyframes
            if (iKF == nKFs - 1)
                continue;
            const auto *_ds = m_ds.data() + m_KFs[iKF].m_id;
            const auto *_depth_marks = m_Uds.data() + m_KFs[iKF].m_id;
            const auto Nx = m_KFs[iKF].m_xs.size();
            KF_to_set.m_inv_depths.resize(Nx);
            KF_to_set.m_depths_updated.resize(Nx);
            for (size_t ix = 0; ix < Nx; ++ix) {
                KF_to_set.m_inv_depths[ix] = _ds[ix].u();
                KF_to_set.m_depths_updated[ix] = _depth_marks[ix] != 0;
            }
        }
    }

#ifdef CFG_VERBOSE
    if (m_verbose > 1) {
        int SNc = 0, SNd = 0;
        for (int iKF = 0; iKF < nKFs; ++iKF) {
            if (m_Ucs[iKF]) ++SNc;
        }
        for (int id = 0; id < Nd; ++id) {
            if (m_Uds[id]) ++SNd;
        }
        UT::PrintSeparator();
        UT::Print("[%d] Updated Camera = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNc, nKFs, float(SNc * 100) / nKFs);
        UT::Print("[%d] Updated Depth  = %d / %d = %.2f%%\n",
                  m_KFs.back().m_T.m_iFrm, SNd, Nd,
                  Nd == 0 ? 0.0f : float(SNd * 100) / Nd);
    }
#endif
    UT::VectorMakeZero(m_Ucs);
    UT::VectorMakeZero(m_Uds);
}

bool GlobalBundleAdjustor::BufferDataEmpty()
{
    MT_READ_LOCK_BEGIN(m_MT);
    return m_IKFs1.empty();
    MT_READ_LOCK_END(m_MT);
}

void GlobalBundleAdjustor::PushFeatureMeasurementMatchesFirst(
    const FRM::Frame &F, std::vector<int> &iKF2X, std::vector<int> &iX2z)
{
    int SNx = 0;
    // NZ : number of frames which has matched features
    const int NZ = int(F.m_Zs.size());
    // iKF2X : matched keyframe to source feature index
    iKF2X.assign(m_KFs.size(), -1);
    for (int iZ = 0; iZ < NZ; ++iZ) {
        const int iKF = F.m_Zs[iZ].m_iKF;
        iKF2X[iKF] = SNx;
        SNx += int(m_KFs[iKF].m_xs.size());
    }
    // (nearly global) source feature index to measurement index
    iX2z.assign(SNx, -1);
    for (int iZ = 0; iZ < NZ; ++iZ) {
        const FRM::Measurement &Z = F.m_Zs[iZ];
        int *ix2z = iX2z.data() + iKF2X[Z.m_iKF];
        const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
        for (int iz = iz1; iz < iz2; ++iz) ix2z[F.m_zs[iz].m_ix] = iz;
    }
}

void GlobalBundleAdjustor::PushFeatureMeasurementMatchesNext(
    const FRM::Frame &F1, const std::vector<int> &iKF2X,
    const std::vector<int> &iX2z2, KeyFrame::MeasurementMatch &Zm)
{
    m_izmsTmp.resize(0);
    const int NZ1 = int(F1.m_Zs.size());
    for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
        const FRM::Measurement &Z1 = F1.m_Zs[iZ1];
        const int iX = iKF2X[Z1.m_iKF];
        if (iX == -1) continue;
        const int *ix2z2 = iX2z2.data() + iX;
        const int iz11 = Z1.m_izIP[0], iz12 = Z1.m_izIP[FRM_IP_LEVELS];
        for (int iz1 = iz11; iz1 < iz12; ++iz1) {
            const int iz2 = ix2z2[F1.m_zs[iz1].m_ix];
            if (iz2 != -1)
                m_izmsTmp.push_back(FTR::Measurement::Match(iz1, iz2));
        }
    }
    Zm.PushFeatureMeasurementMatches(m_izmsTmp);
}
