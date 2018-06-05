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

#include "BAInterface.h"
#include "GlobalBundleAdjustor.h"
#include <memory>
#include <iomanip>
#include <algorithm>
#include <iostream>
#include <set>

#ifdef ENABLE_YAML_IO
#include <yaml-cpp/yaml.h>
#endif

using namespace EIBA;
using namespace std;

namespace EIBA {
    std::vector<std::set<int, std::less<int>>> g_KFsMatch;
    GlobalBundleAdjustor *g_gba = nullptr;
    GlobalMap *g_global_map = nullptr;
    int g_curr_KF_cnt = 0;
    int g_src_feature_cnt = 0;
    int g_mea_feature_cnt = 0;
}


BAInterface::BAInterface()
{
    if (g_gba == nullptr) {
        g_gba = new GlobalBundleAdjustor();
    }
    if (g_global_map == nullptr) {
        g_global_map = new GlobalMap();
    }
    g_gba->Initialize(g_global_map);
    g_gba->ResetBAParam(BAParam());
}

BAInterface::~BAInterface()
{
    if (g_gba != nullptr) {
        delete g_gba;
    }
    if (g_global_map != nullptr) {
        delete g_global_map;
    }
}

void BAInterface::SetParam(const BAParam &param)
{
    Intrinsic K;
    K.Set(param.width, param.height,
        param.fx_fy_cx_cy[0], param.fx_fy_cx_cy[1],
        param.fx_fy_cx_cy[2], param.fx_fy_cx_cy[3]);
    g_gba->m_GM->m_K = K;
    g_gba->m_K = K;
    g_gba->ResetBAParam(param);
}

void BAInterface::PushKeyframeFeatures(const Camera &initial_camera,
                                       const std::vector<float> &initial_depths,
                                       const std::vector<Feature> &source_features,
                                       const std::vector<Measurement> &measured_features)
{
    GlobalMap::KeyFrame KF;
    std::vector<DepthInverseGaussian> dxs;
    std::vector<DepthInverseGaussian> dzs;

    // prepare camera pose initial guess
    Rigid3D camera;
    float Pos[3] = {initial_camera.m_position.m_x, initial_camera.m_position.m_y, initial_camera.m_position.m_z};
    Rotation3D rot;
    Point3D pos;
    pos.Set(Pos);
    rot.Set(initial_camera.m_rotation[0]);
    camera.Set(rot.GetQuaternion(), pos);

    // prepare last frame depth initial guess
    for (auto &d : initial_depths) {
        DepthInverseGaussian d_tmp;
        d_tmp.Initialize(d);
        dxs.emplace_back(d_tmp);
    }

    // prepare source features
    KF.m_id = g_src_feature_cnt;
    for (auto &x : source_features) {
        FTR::Source x_tmp;
        x_tmp.m_x.Set(x.m_pt.m_x, x.m_pt.m_y);
        x_tmp.m_d = x.m_inv_depth;
        KF.m_xs.emplace_back(x_tmp);
        g_src_feature_cnt++;
    }

    // prepare measurements m_Zs, m_zs
    // handle measurement order
    auto compare = [](const Measurement &m1, const Measurement &m2) {
        if (m1.m_matched_kf_idx == m2.m_matched_kf_idx) {
            return m1.m_matched_ftr_idx < m2.m_matched_ftr_idx;
        } else {
            return m1.m_matched_kf_idx < m2.m_matched_kf_idx;
        }
    };
    auto zs_sorted = measured_features;
    std::sort(zs_sorted.begin(), zs_sorted.end(), compare);

    int idx_match_frm = 0;
    if (!zs_sorted.empty()) {
        KF.m_Zs.resize(1);
        KF.m_Zs[0].m_iKF = zs_sorted[0].m_matched_kf_idx;
        KF.m_Zs[0].m_izIP[0] = 0;
    }

    int idx_measure_feature = 0;
    for (auto &z_src : zs_sorted) {

        // new frame measurement
        if (KF.m_Zs[idx_match_frm].m_iKF != z_src.m_matched_kf_idx) {
            auto &izIP = KF.m_Zs[idx_match_frm].m_izIP;
            izIP[1] = izIP[2] = izIP[3] = izIP[4] = idx_measure_feature;
            idx_match_frm++;
            KF.m_Zs.resize(idx_match_frm + 1);
            KF.m_Zs[idx_match_frm].m_izIP[0] = idx_measure_feature;
            KF.m_Zs[idx_match_frm].m_iKF = z_src.m_matched_kf_idx;
        }

        FTR::Measurement measurement;
        measurement.m_d = z_src.m_inv_depth;
        measurement.m_z.Set(z_src.m_pt.m_x, z_src.m_pt.m_y);
        measurement.m_ix = z_src.m_matched_ftr_idx;
        measurement.m_W.Set(z_src.m_cov2[0][0], z_src.m_cov2[0][1], z_src.m_cov2[1][1]);
        KF.m_zs.emplace_back(measurement);
        idx_measure_feature++;
        g_mea_feature_cnt++;
    }
    // process last measurement
    if (!zs_sorted.empty()) {
        auto &izIP = KF.m_Zs[idx_match_frm].m_izIP;
        izIP[1] = izIP[2] = izIP[3] = izIP[4] = idx_measure_feature;
    }


    // get m_dzs
    AlignedVector<Rigid3D> Cs_tmp;
    std::vector<DepthInverseGaussian> ds_tmp;
    g_gba->m_GM->IT_Synchronize(Cs_tmp, ds_tmp);
    dzs.resize(KF.m_zs.size());
    const int NZ = int(KF.m_Zs.size());
    for (int iZ = 0; iZ < NZ; ++iZ) {
        const FRM::Measurement &Z = KF.m_Zs[iZ];
        const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
        const auto *_ds = ds_tmp.data() + g_gba->m_KFs[Z.m_iKF].m_id;
        for (int iz = iz1; iz < iz2; ++iz) {
            // if dzs correspond to last frame,
            // which means it's dxs has only been provided in current frame,
            // then get dzs from dxs_history
            if (Z.m_iKF >= g_curr_KF_cnt - 1) {
                dzs[iz] = dxs[KF.m_zs[iz].m_ix];
            } else {
                dzs[iz] = _ds[KF.m_zs[iz].m_ix];
            }
        }
    }


    // get m_iKFsMatch
    g_KFsMatch.resize(g_curr_KF_cnt + 2);

    for (auto &Z : KF.m_Zs) {
        auto &src_matches = g_KFsMatch[Z.m_iKF];
        src_matches.insert(g_curr_KF_cnt);
    }

    std::set<int, std::less<int>> currKFMatch;
    for (auto &Z: KF.m_Zs) {
        currKFMatch.insert(Z.m_iKF);
        for (auto &match : g_KFsMatch[Z.m_iKF]) {
            currKFMatch.insert(match);
        }
    }
    currKFMatch.erase(g_curr_KF_cnt);

    KF.m_iKFsMatch.resize(0);
    for (auto &idx : currKFMatch) {
        KF.m_iKFsMatch.emplace_back(idx);
    }

    KF.m_T.m_iFrm = g_curr_KF_cnt;
    g_curr_KF_cnt++;

    g_gba->PushKeyFrame(KF, camera, dxs, dzs);
}

void BAInterface::PushBetweenFrameConstraint(int kf_idx1,
                                             int kf_idx2,
                                             const Camera &C12,
                                             const float cov6[6][6])
{
    GlobalMap::KeyFramePrior prior;
    float Pos[3] = {C12.m_position.m_x, C12.m_position.m_y, C12.m_position.m_z};
    prior.m_Zp.m_R.Set(C12.m_rotation[0]);
    prior.m_Zp.m_p.Set(Pos);
    auto SetMatrix3x3= [this](const float W[6][6], LA::AlignedMatrix3x3f &dst, int start_x, int start_y) {
        float W_3x3[3][3];
        for(int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                W_3x3[i][j] = W[i + start_x][j + start_y];
            }
        }
        dst.Set(W_3x3[0]);
    };

    SetMatrix3x3(cov6, prior.m_Zp.m_Wrr, 0, 0);
    SetMatrix3x3(cov6, prior.m_Zp.m_Wrp, 0, 3);
    SetMatrix3x3(cov6, prior.m_Zp.m_Wpr, 3, 0);
    SetMatrix3x3(cov6, prior.m_Zp.m_Wpp, 3, 3);
    prior.m_iKF1 = kf_idx1;
    prior.m_iKF2 = kf_idx2;
    std::vector<GlobalMap::KeyFramePrior> priors;
    priors.emplace_back(prior);
    g_gba->PushKeyFramePriors(priors);
}

void BAInterface::Optimize(std::vector<KeyFrame> *optKFs)
{
    g_gba->Run(optKFs);
}

#ifdef ENABLE_YAML_IO
void BAInterface::PushFrameInfoFromYAML(std::string filename)
{
    YAML::Node root_node = YAML::LoadFile(filename);
    auto features_node = root_node["features"];
    auto initial_guess = features_node["initial_guess"];
    auto current_cam_pose = initial_guess["current_camera_pose_guess"];

    Camera initial_camera;
    float Pos[3];
    for (size_t i = 0; i < 3; ++i) {
        // force to store as sequence for every row
        for (size_t j = 0; j < 4; ++j) {
            if (j == 3) {
                Pos[i] = current_cam_pose[i][j].as<float>();
            } else {
                initial_camera.m_rotation[i][j] = current_cam_pose[i][j].as<float>();
            }
        }
    }
    initial_camera.m_position.m_x = Pos[0];
    initial_camera.m_position.m_y = Pos[1];
    initial_camera.m_position.m_z = Pos[2];

    auto last_inv_depth_guess = initial_guess["last_inv_depth_guess"];
    std::vector<float> inv_depths;
    for (auto node : last_inv_depth_guess) {
        inv_depths.emplace_back(node[0].as<float>());
    }

    auto source_features = features_node["source_features"];
    std::vector<Feature> src_features;
    for (auto node : source_features) {
        Feature ftr;
        ftr.m_pt.m_x = node[0].as<float>();
        ftr.m_pt.m_y = node[1].as<float>();
        ftr.m_inv_depth = node[2].as<float>();
        src_features.emplace_back(ftr);
    }

    auto measured_features = features_node["measured_features"];
    std::vector<Measurement> measures;
    for (auto node : measured_features) {
        Measurement mea;
        mea.m_matched_kf_idx = node["kf_idx"].as<int>();
        mea.m_matched_ftr_idx = node["ftr_idx"].as<int>();
        mea.m_pt.m_x = node["pt"][0].as<float>();
        mea.m_pt.m_y = node["pt"][1].as<float>();
        mea.m_inv_depth = node["pt"][2].as<float>();
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                mea.m_cov2[i][j] = node["cov2"][i][j].as<float>();
            }
        }
        measures.emplace_back(mea);
    }

    PushKeyframeFeatures(initial_camera, inv_depths, src_features, measures);

    // store frame constraints
    auto constraint_node = root_node["frame_constraints"];
    for (auto node : constraint_node) {
        float Pos[3];
        Camera C12;
        auto &Rot = C12.m_rotation;
        auto relative_pose_Rp = node["relative_pose_Rp"];
        for (size_t i = 0; i < 3; ++i) {
            // force to store as sequence for every row
            for (size_t j = 0; j < 4; ++j) {
                if (j == 3) {
                    Pos[i] = relative_pose_Rp[i][j].as<float>();
                } else {
                    Rot[i][j] = relative_pose_Rp[i][j].as<float>();
                }
            }
        }

        C12.m_position.m_x = Pos[0];
        C12.m_position.m_y = Pos[1];
        C12.m_position.m_z = Pos[2];

        float cov6[6][6];
        auto cov_node = node["cov6"];
        for (size_t i = 0; i < 6; ++i) {
            for (size_t j = 0; j < 6; ++j) {
                cov6[i][j] = cov_node[i][j].as<float>();
            }
        }

        PushBetweenFrameConstraint(
                node["keyframe_index_1"].as<int>(),
                node["keyframe_index_2"].as<int>(),
                C12, cov6);
    }
}
#endif


