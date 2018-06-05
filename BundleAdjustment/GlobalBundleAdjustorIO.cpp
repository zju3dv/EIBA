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

#ifdef ENABLE_YAML_IO
#include <yaml-cpp/yaml.h>
#include <iomanip>
#include <iostream>

namespace {
    int s_cnt = 0;
}
#endif


// #ifdef CFG_VIEW
void GlobalBundleAdjustor::SaveB(FILE *fp)
{
    MT_READ_LOCK_BEGIN(m_MT);
    FRM::ListSaveB(m_IKFs1, fp);
    FRM::ListSaveB(m_IKFs2, fp);
    UT::ListSaveB(m_IZps1, fp);
    UT::ListSaveB(m_IZps2, fp);
    MT_READ_LOCK_END(m_MT);
#if 0
    UT::SaveB(m_iKFFix, fp);
    FRM::VectorSaveB(m_KFs, fp);
    UT::VectorSaveB(m_ucs, fp);
    UT::VectorSaveB(m_Ucs, fp);
    UT::VectorSaveB(m_uds, fp);
    UT::VectorSaveB(m_Uds, fp);
    m_Cs.SaveB(fp);
    m_CsLP.SaveB(fp);
#ifdef CFG_GROUND_TRUTH
    m_CsGTKF.SaveB(fp);
#endif
    UT::VectorSaveB(m_ds, fp);
    UT::VectorSaveB(m_dsLP, fp);
    // m_SAdcxs.SaveB(fp);
    // m_SMdcxs.SaveB(fp);
    // m_SMcxxs.SaveB(fp);
    m_SAcus.SaveB(fp);
    m_SAcbs.SaveB(fp);
#endif
}

void GlobalBundleAdjustor::LoadB(FILE *fp)
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    FRM::ListLoadB(m_IKFs1, fp);
    UT::ListLoadB(m_IZps1, fp);
#ifdef ENABLE_YAML_IO
    // save to yaml after load from file
    std::stringstream ss;
    ss << "data/simple/" << std::setfill('0') << std::setw(5) << s_cnt << ".yaml";
        SaveToYAML(ss.str());
    s_cnt++;
    std::cout << "here" << std::endl;
#endif
    MT_WRITE_LOCK_END(m_MT);
#if 0
    UT::LoadB(m_iKFFix, fp);
    FRM::VectorLoadB(m_KFs, fp);
    UT::VectorLoadB(m_ucs, fp);
    UT::VectorLoadB(m_Ucs, fp);
    UT::VectorLoadB(m_uds, fp);
    UT::VectorLoadB(m_Uds, fp);
    m_Cs.LoadB(fp);
    m_CsLP.LoadB(fp);
#ifdef CFG_GROUND_TRUTH
    m_CsGTKF.LoadB(fp);
#endif
    UT::VectorLoadB(m_ds, fp);
    UT::VectorLoadB(m_dsLP, fp);
    // m_SAdcxs.LoadB(fp);
    // m_SMdcxs.LoadB(fp);
    // m_SMcxxs.LoadB(fp);
    m_SAcus.LoadB(fp);
    m_SAcbs.LoadB(fp);
#endif
}
// #endif

void GlobalBundleAdjustor::ResetBAParam(const EIBA::BAParam &param)
{
    GBA_WEIGHT_FIX = param.weight_fix;
    GBA_WEIGHT_FEATURE = param.weight_feature;
    GBA_WEIGHT_PRIOR = param.weight_prior;
    GBA_VARIANCE_FIX_ROTATION = param.variance_fix_rotation;
    GBA_VARIANCE_FIX_POSITION = param.variance_fix_position;
    DEPTH_MAP_VARIANCE = param.variance_depth_map;
    GBA_UPDATE_ROTATION = param.update_rotation;
    GBA_UPDATE_POSITION = param.update_position;
    GBA_UPDATE_DEPTH = param.update_depth;
    g_verbose = param.verbose;
    m_verbose = param.verbose;
}

void GlobalBundleAdjustor::SaveToYAML(std::string filename)
{
#ifdef ENABLE_YAML_IO
    YAML::Node root_node;

    auto &InputKF = *m_IKFs1.begin();

    auto features_node = root_node["features"];

    auto initial_guess = features_node["initial_guess"];

    auto camera_pose = initial_guess["current_camera_pose_guess"];

    // store m_C
    float Rot[3][3], Pos[3];
    Rotation3D(InputKF.m_C).Get(&Rot[0][0]);
    InputKF.m_C.GetPosition().Get((float *) &Pos);
    for (size_t i = 0; i < 3; ++i) {
        // force to store as sequence for every row
        camera_pose[i] = YAML::Load("[]");
        for (size_t j = 0; j < 4; ++j) {
            if (j == 3)
                camera_pose[i].push_back(Pos[i]);
            else
                camera_pose[i].push_back(Rot[i][j]);
        }
    }

    // store m_dxs
    auto last_inv_depth_guess = initial_guess["last_inv_depth_guess"];
    for (size_t i = 0; i < InputKF.m_dxs.size(); ++i) {
        auto &dx = InputKF.m_dxs[i];
        last_inv_depth_guess[i] = YAML::Load("[]");
        last_inv_depth_guess[i].push_back(dx.u());
    }

    // point2d + inverse depth
    auto source_features = features_node["source_features"];
    for (size_t i = 0; i < InputKF.m_xs.size(); ++i) {
        auto &x = InputKF.m_xs[i];
        source_features[i] = YAML::Load("[]");
        source_features[i].push_back(x.m_x.x());
        source_features[i].push_back(x.m_x.y());
        source_features[i].push_back(x.m_d);
    }

    // store Frame in GlobalMap::KeyFrame
    // m_Zs
    int iz_cnt = 0;
    auto measured_features = features_node["measured_features"];
    for (auto &Z : InputKF.m_Zs) {
        auto iKF = Z.m_iKF;
        auto iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[4];
        for (int iz = iz1; iz < iz2; ++iz) {
            auto node = measured_features[iz_cnt++];
            auto &z = InputKF.m_zs[iz];
            node["kf_idx"] = iKF;
            node["ftr_idx"] = z.m_ix;
            node["pt"] = YAML::Load("[]");
            node["pt"].push_back(z.m_z.x());
            node["pt"].push_back(z.m_z.y());
            node["pt"].push_back(z.m_d);
            node["cov2"][0] = YAML::Load("[]");
            node["cov2"][0].push_back(z.m_W.m00());
            node["cov2"][0].push_back(z.m_W.m01());
            node["cov2"][1] = YAML::Load("[]");
            node["cov2"][1].push_back(z.m_W.m01());
            node["cov2"][1].push_back(z.m_W.m11());

        }
    }


    // store frame constraints
    auto constraint_node = root_node["frame_constraints"];
    int iPrior = 0;
    for (auto &prior : m_IZps1) {
        float Rot[3][3], Pos[3];
        Rotation3D(InputKF.m_C).Get(&Rot[0][0]);
        InputKF.m_C.GetPosition().Get((float *) &Pos);
        float W[6][6];
        prior.m_Zp.m_R.Get(&Rot[0][0]);
        prior.m_Zp.m_p.Get(Pos);
        prior.m_Zp.m_Wrr.Get_chk(&W[0][0], &W[1][0], &W[2][0]);
        prior.m_Zp.m_Wrp.Get_chk(&W[0][3], &W[1][3], &W[2][3]);
        prior.m_Zp.m_Wpr.Get_chk(&W[3][0], &W[4][0], &W[5][0]);
        prior.m_Zp.m_Wpp.Get_chk(&W[3][3], &W[4][3], &W[5][3]);

        auto node = constraint_node[iPrior];
        node["keyframe_index_1"] = prior.m_iKF1;
        node["keyframe_index_2"] = prior.m_iKF2;

        auto camera_node = node["relative_pose_Rp"];
        for (size_t i = 0; i < 3; ++i) {
            // force to store as sequence for every row
            camera_node[i] = YAML::Load("[]");
            for (size_t j = 0; j < 4; ++j) {
                if (j == 3)
                    camera_node[i].push_back(Pos[i]);
                else
                    camera_node[i].push_back(Rot[i][j]);
            }
        }

        auto cov_node = node["cov6"];
        for (size_t i = 0; i < 6; ++i) {
            cov_node[i] = YAML::Load("[]");
            for (size_t j = 0; j < 6; ++j) {
                cov_node[i].push_back(W[i][j]);
            }
        }
        iPrior++;
    }

    std::ofstream fout(filename);
    fout << root_node;
#endif

}

void GlobalBundleAdjustor::AssertConsistency()
{
    const int nKFs = int(m_KFs.size()), Nd = int(m_ds.size());
    UT_ASSERT(int(m_ucs.size()) == nKFs && int(m_Ucs.size()) == nKFs &&
              int(m_uds.size()) == Nd && int(m_Uds.size()) == Nd);
    UT_ASSERT(m_Cs.Size() == nKFs && m_CsLP.Size() == nKFs + 1 &&
              int(m_dsLP.size()) == Nd);
    UT_ASSERT(nKFs == 0 || m_KFs.back().m_id == Nd);
    for (int iKF = 0; iKF < nKFs; ++iKF) m_Cs[iKF].AssertOrthogonal();

    int SNk = 0, SNd = 0;
    const int iKFLast = nKFs - 1;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        KF.AssertConsistency(iKF == iKFLast);
        UT_ASSERT(KF.m_iK == SNk);
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const int _iKF = KF.m_Zs[iZ].m_iKF;
            UT_ASSERT(_iKF < iKF);
        }
        const int Nk = int(KF.m_iKFsMatch.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const int _iKF = KF.m_iKFsMatch[ik];
            UT_ASSERT(_iKF < iKF);
            KF.m_Zm.AssertConsistency(ik, m_KFs[_iKF], KF, m_izmsTmp);
        }
        SNk += int(KF.m_ik2KF.size());
        SNd += KF.m_SAdcxs.Size();
    }
    const int NK = SNk;
    if (nKFs > 0) UT_ASSERT(m_KFs.back().m_id == Nd);
    UT_ASSERT(SNd == Nd);

    m_work.Resize(Nd * sizeof(FTR::Factor::Source) / sizeof(float));
    AlignedVector<FTR::Factor::Source> SAdcxs(
        (FTR::Factor::Source *)m_work.Data(), Nd, false);
    SAdcxs.MakeZero();
#ifdef CFG_DEPTH_MAP
    for (int iKF = 0; iKF < iKFLast; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        FTR::Factor::Source *_SAdcxs = SAdcxs.Data() + KF.m_id;
        const int Nx = int(KF.m_xs.size());
        for (int ix = 0, jx = 0; ix < Nx; ++ix) {
            if (KF.m_xs[ix].m_d != 0.0f) _SAdcxs[ix] += KF.m_Ads[jx++];
        }
    }
#endif
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            FTR::Factor::Source *_SAdcxs = SAdcxs.Data() + m_KFs[Z.m_iKF].m_id;
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz)
                _SAdcxs[KF.m_zs[iz].m_ix] += KF.m_Adcxs[iz];
        }
    }
    // UT_ASSERT(m_SAdcxs.Size() == Nd && m_SMdcxs.Size() == Nd &&
    // m_SMcxxs.Size() == Nd);
    // for(int id = 0; id < Nd; ++id)
    //	SAdcxs[id].AssertEqual(m_SAdcxs[id]);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const FTR::Factor::Source *_SAdcxs = SAdcxs.Data() + KF.m_id;
        const int Nx = KF.m_SAdcxs.Size();
        for (int ix = 0; ix < Nx; ++ix)
            KF.m_SAdcxs[ix].AssertEqual(_SAdcxs[ix]);
    }

    m_work.Resize(nKFs * sizeof(Camera::Pose::Unitary) / sizeof(float));
    AlignedVector<Camera::Pose::Unitary> SAcus(
        (Camera::Pose::Unitary *)m_work.Data(), nKFs, false);
    SAcus.MakeZero();
    const float epsAbs = 1.0e-3f, epsRel = 1.0e-3f;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        Camera::Pose::Unitary &SAczz = SAcus[iKF];
        const KeyFrame &KF = m_KFs[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            Camera::Pose::Unitary &SAcxx = SAcus[Z.m_iKF];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
#if 0
//#if 1
				if(UT::Debugging() && (iKF == 10 || Z.m_iKF == 10))
					UT::Print("%d %d %f %f", iKF, iz, SAcxx.m_A.m00(), SAczz.m_A.m00());
#endif
                SAcxx += KF.m_Acxxs[iz];
                SAczz += KF.m_Aczzs[iz];
#if 0
//#if 1
				if(UT::Debugging() && (iKF == 10 || Z.m_iKF == 10))
					UT::Print(" --> %f %f\n", SAcxx.m_A.m00(), SAczz.m_A.m00());
#endif
            }
        }
    }
#if 0
//#if 1
	if(UT::Debugging())
	{
		UT::PrintSeparator('*');
		SAcus[10].Print();
		UT::PrintSeparator('*');
	}
#endif
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        Camera::Pose::Unitary &SA2 = SAcus[iKF];
        const KeyFrame &KF = m_KFs[iKF];
        const int Np = int(KF.m_iKFsPrior.size());
        for (int ip = 0; ip < Np; ++ip) {
            SAcus[KF.m_iKFsPrior[ip]] += KF.m_Ap11s[ip];
            SA2 += KF.m_Ap22s[ip];
        }
    }
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        Camera::Pose::Unitary &SAczz = SAcus[iKF];
        const KeyFrame &KF = m_KFs[iKF];
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) SAczz -= KF.m_Mczzs[iz];
        }
        // if(iKF < iKFLast)
        //{
        //	Camera::Pose::Unitary &SAcxx = SAcus[iKF];
        //	const Camera::Pose::Unitary *SMcxxs = m_SMcxxs.Data() + KF.m_id;
        //	const int Nx = int(KF.m_xs.size());
        //	for(int ix = 0; ix < Nx; ++ix)
        //		SAcxx -= SMcxxs[ix];
        //}
        Camera::Pose::Unitary &SAcxx = SAcus[iKF];
        const int Nx = KF.m_SMcxxs.Size();
        for (int ix = 0; ix < Nx; ++ix) SAcxx -= KF.m_SMcxxs[ix];
    }
    UT_ASSERT(m_SAcus.Size() == nKFs + 1);
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        SAcus[iKF].AssertEqual(m_SAcus[iKF], 1, epsAbs, epsRel);
#if 0
//#ifdef CFG_DEBUG_EIGEN
		if(e_I.empty())
			continue;
		const int icu = e_I[iKF][iKF];
		const Camera::Pose::Unitary SAcu(e_SAccs[icu].GetSymmetricMatrix6x6f(), e_Sbcs[iKF].GetVector6f());
		SAcu.AssertEqual(SAcus[iKF], 1, epsAbs, epsRel);
#endif
    }

    UT_ASSERT(m_SAcbs.Size() == NK);
    m_work.Resize(NK * sizeof(Camera::Pose::Binary) / sizeof(float));
    AlignedVector<Camera::Pose::Binary> SAcbs(
        (Camera::Pose::Binary *)m_work.Data(), NK, false);
    SAcbs.MakeZero();
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        Camera::Pose::Binary *_SAcbs = SAcbs.Data() + KF.m_iK;
        const int NZ = int(KF.m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const FRM::Measurement &Z = KF.m_Zs[iZ];
            Camera::Pose::Binary &SAcxz = _SAcbs[KF.m_iZ2k[iZ]];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iz = iz1; iz < iz2; ++iz) {
                SAcxz += KF.m_Acxzs[iz];
                SAcxz -= KF.m_Mcxzs[iz];
            }
        }
        const int Nk = int(KF.m_iKFsMatch.size());
        for (int ik = 0; ik < Nk; ++ik) {
            Camera::Pose::Binary &SAczm = _SAcbs[ik];
            const int i1 = KF.m_Zm.m_ik2zm[ik], i2 = KF.m_Zm.m_ik2zm[ik + 1];
            for (int i = i1; i < i2; ++i) SAczm -= KF.m_Zm.m_Mczms[i];
        }
        const int Np = int(KF.m_iKFsPrior.size());
        for (int ip = 0; ip < Np; ++ip)
            _SAcbs[KF.m_iZp2k[ip]] += KF.m_Ap12s[ip];
    }
    for (int iK = 0; iK < NK; ++iK)
        SAcbs[iK].AssertEqual(m_SAcbs[iK], 1, epsAbs, epsRel);
#ifdef CFG_DEBUG_EIGEN
    if (e_I.empty()) return;
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        const KeyFrame &KF = m_KFs[iKF];
        const Camera::Pose::Binary *_SAcbs = SAcbs.Data() + KF.m_iK;
        const int Nk = int(KF.m_ik2KF.size());
        for (int ik = 0; ik < Nk; ++ik) {
            const int _iKF = KF.m_ik2KF[ik], icb = e_I[_iKF][iKF];
            const Camera::Pose::Binary SAcb(
                e_SAccs[icb].GetAlignedMatrix6x6f());
            SAcb.AssertEqual(_SAcbs[ik], 1, epsAbs, epsRel);
        }
    }
#endif
}
