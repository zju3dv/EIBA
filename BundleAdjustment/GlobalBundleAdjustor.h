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
#include "BAInterface.h"

#ifndef _GLOBAL_BUNDLE_ADJUSTOR_H_
#define _GLOBAL_BUNDLE_ADJUSTOR_H_

#include "stdafx.h"

#include "GlobalMap.h"
#include "Timer.h"

extern int g_verbose;

#ifdef CFG_TUNE_PARAMETERS
extern float GBA_WEIGHT_FEATURE;
extern float GBA_WEIGHT_PRIOR;
extern float GBA_WEIGHT_FIX;
extern float GBA_UPDATE_ROTATION;
extern float GBA_UPDATE_POSITION;
extern float GBA_UPDATE_DEPTH;
extern float GBA_CONVERGE_ROTATION;
extern float GBA_CONVERGE_POSITION;
extern float GBA_CONVERGE_DEPTH;
extern float GBA_VARIANCE_FIX_ROTATION;
extern float GBA_VARIANCE_FIX_POSITION;

extern void LOAD_PARAMETERS_GLOBAL_BUNDLE_ADJUSTMENT(const Configurator &cfgor);
#else
#define GBA_WEIGHT_FEATURE 1.0e-5f
#define GBA_WEIGHT_PRIOR 1.0e-5f
#define GBA_WEIGHT_FIX 1.0e2f
#define GBA_UPDATE_ROTATION 3.046174198662e-6f   // (0.1*pi/180)^2
#define GBA_UPDATE_POSITION 1.0e-4f              // 0.01^2
#define GBA_UPDATE_DEPTH 1.0e-4f                 // 0.01^2
#define GBA_CONVERGE_ROTATION 7.615435234857e-5f // (0.5*pi/180)^2
#define GBA_CONVERGE_POSITION 0.0025f            // 0.05^2
#define GBA_CONVERGE_DEPTH 0.0025f               // 0.05^2
#endif


class GlobalBundleAdjustor{
  public:
    void ResetBAParam(const EIBA::BAParam &param);
    // save BA frame info to YAML
    void SaveToYAML(std::string filename);
// #ifdef CFG_VIEW
  public:
    virtual void SaveB(FILE *fp);
    virtual void LoadB(FILE *fp);
// #endif

    typedef void (*CallBackUpdateKeyFrame)(const int iFrm, const Rigid3D &C);

#ifdef CFG_GROUND_TRUTH
    inline void SetGroundTruth(const AlignedVector<Camera> &CsGT)
    {
        m_CsGT.Set(CsGT);
    }
    AlignedVector<Camera> m_CsGT;
#endif

    inline void SetTimingFile(const std::string fileName)
    {
        m_timingFileName = fileName;
    }

    virtual void Initialize(GlobalMap *GM,
//                            CallBackUpdateKeyFrame updateKFCB,
                            const bool serial = false, const int verbose = 0);
    virtual void Reset();
    virtual void Run(std::vector<EIBA::KeyFrame> *optKFs = nullptr);
    virtual void Stop();
    virtual void PushKeyFrame(const GlobalMap::KeyFrame &KF, const Rigid3D &C,
                              const std::vector<DepthInverseGaussian> &dxs,
                              const std::vector<DepthInverseGaussian> &dzs,
                              const int iKFFix = 0);
    virtual void
    PushKeyFramePriors(const std::vector<GlobalMap::KeyFramePrior> &Zps);

  public:
    class InputKeyFrame : public GlobalMap::KeyFrame
    {
      public:
        inline InputKeyFrame() : KeyFrame() {}
        inline InputKeyFrame(const GlobalMap::KeyFrame &KF, const Rigid3D &C,
                             const std::vector<DepthInverseGaussian> &dxs,
                             const std::vector<DepthInverseGaussian> &dzs,
                             const int iKFFix = 0)
            : GlobalMap::KeyFrame(KF)
        {
            m_C = C;
            m_dxs = dxs;
            m_dzs = dzs;
            m_iKFFix = iKFFix;
        }
        inline void SaveB(FILE *fp) const
        {
            GlobalMap::KeyFrame::SaveB(fp);
            UT::SaveB(m_C, fp);
            UT::VectorSaveB(m_dxs, fp);
            UT::VectorSaveB(m_dzs, fp);
            UT::SaveB(m_iKFFix, fp);
        }
        inline void LoadB(FILE *fp)
        {
            GlobalMap::KeyFrame::LoadB(fp);
            UT::LoadB(m_C, fp);
            UT::VectorLoadB(m_dxs, fp);
            UT::VectorLoadB(m_dzs, fp);
            UT::LoadB(m_iKFFix, fp);
            printf("m_dxs(inv_d+cov_d) %d \n", m_dxs.size());
            printf("m_dzs(inv_d+cov_d) %d \n", m_dzs.size());
            m_iKFFix = 0;
        }

      public:
        // camera pose (rotation, position)
        Rigid3D m_C;
        // m_dxs : (last keyframe's) source depth (to optimized)
        // m_dzs : measured depth
        std::vector<DepthInverseGaussian> m_dxs, m_dzs;
        // which frame to fix and not optimize, default is 0 (first frame)
        int m_iKFFix;
    };

    class KeyFrame : public GlobalMap::KeyFrame
    {
      public:
        class MeasurementMatch
        {
          public:
            inline void operator=(const MeasurementMatch &Zm)
            {
                m_ik2zm = Zm.m_ik2zm;
                m_izms = Zm.m_izms;
                m_Mczms.Set(Zm.m_Mczms);
            }
            inline void Initialize()
            {
                m_ik2zm.assign(1, 0);
                m_izms.resize(0);
                m_Mczms.Resize(0);
            }
            inline void PushFeatureMeasurementMatches(
                const std::vector<FTR::Measurement::Match> &izms)
            {
                const int Nzm1 = m_ik2zm.back(), Nzm = int(izms.size()),
                          Nzm2 = Nzm1 + Nzm;
                m_ik2zm.push_back(Nzm2);
                m_izms.insert(m_izms.end(), izms.begin(), izms.end());
                m_Mczms.Resize(Nzm2, true);
                m_Mczms.MakeZero(Nzm1, Nzm2);
            }
            inline void SaveB(FILE *fp) const
            {
                UT::VectorSaveB(m_ik2zm, fp);
                UT::VectorSaveB(m_izms, fp);
                m_Mczms.SaveB(fp);
            }
            inline void LoadB(FILE *fp)
            {
                UT::VectorLoadB(m_ik2zm, fp);
                UT::VectorLoadB(m_izms, fp);
                m_Mczms.LoadB(fp);
            }
            inline void AssertConsistency(const int Nk) const
            {
                const int Nzm = int(m_izms.size());
                UT_ASSERT(int(m_ik2zm.size()) == Nk + 1);
                UT_ASSERT(m_ik2zm[0] == 0 && m_ik2zm[Nk] == Nzm);
                UT_ASSERT(m_Mczms.Size() == Nzm);
                for (int ik = 0; ik < Nk; ++ik) {
                    const int i1 = m_ik2zm[ik], i2 = m_ik2zm[ik + 1];
                    for (int i = i1 + 1; i < i2; ++i)
                        UT_ASSERT(m_izms[i - 1] < m_izms[i]);
                }
            }
            inline void AssertConsistency(
                const int ik, const FRM::Frame &F1, const FRM::Frame &F2,
                std::vector<FTR::Measurement::Match> &izmsTmp) const
            {
                F1.SearchFeatureMeasurementMatches(F2, izmsTmp);
                const int i1 = m_ik2zm[ik], i2 = m_ik2zm[ik + 1], Nzm = i2 - i1;
                UT_ASSERT(Nzm == int(izmsTmp.size()));
                const FTR::Measurement::Match *izms = m_izms.data() + i1;
                for (int i = 0; i < Nzm; ++i) {
                    const FTR::Measurement::Match &izm = izms[i];
                    UT_ASSERT(izm == izmsTmp[i]);
                    UT_ASSERT(F1.m_zs[izm.m_iz1].m_ix ==
                              F2.m_zs[izm.m_iz2].m_ix);
#if 0
					if(i > 0)
						UT_ASSERT(F1.m_zs[izm.m_iz1].m_ix > F1.m_zs[izms[i - 1].m_iz1].m_ix);
#endif
                }
            }

          public:
            // map from keyframe to measurement match?
            std::vector<int> m_ik2zm;
            // measurement match
            std::vector<FTR::Measurement::Match> m_izms;
            // ???
            AlignedVector<Camera::Pose::Binary> m_Mczms;
        };

      public:
        inline KeyFrame() : GlobalMap::KeyFrame() {}
        inline KeyFrame(const KeyFrame &KF) { *this = KF; }
        inline void operator=(const KeyFrame &KF)
        {
            *((GlobalMap::KeyFrame *)this) = KF;
            m_iK = KF.m_iK;
            m_iZ2k = KF.m_iZ2k;
            m_iZp2k = KF.m_iZp2k;
            m_ik2KF = KF.m_ik2KF;
#ifdef CFG_DEPTH_MAP
            m_Ads.Set(KF.m_Ads);
#endif
            m_uzs = KF.m_uzs;
            m_Adcxs.Set(KF.m_Adcxs);
            m_Adczs.Set(KF.m_Adczs);
            m_Acxxs.Set(KF.m_Acxxs);
            m_Acxzs.Set(KF.m_Acxzs);
            m_Aczzs.Set(KF.m_Aczzs);
            m_Mdczs.Set(KF.m_Mdczs);
            m_Mcxzs.Set(KF.m_Mcxzs);
            m_Mczzs.Set(KF.m_Mczzs);
            m_Zm = KF.m_Zm;
            m_SAdcxs.Set(KF.m_SAdcxs);
            m_SMdcxs.Set(KF.m_SMdcxs);
            m_SMcxxs.Set(KF.m_SMcxxs);
            m_Ap11s.Set(KF.m_Ap11s);
            m_Ap12s.Set(KF.m_Ap12s);
            m_Ap22s.Set(KF.m_Ap22s);
        }
        inline void Initialize(const GlobalMap::KeyFrame &KF, const int iK)
        {
            *((GlobalMap::KeyFrame *)this) = KF;
            PyramidToFirstLevel();

            m_iK = iK;
            std::vector<int>::iterator ik = m_iKFsMatch.begin();
            const int NZ = int(m_Zs.size());
            m_iZ2k.resize(NZ);
            for (int iZ = 0; iZ < NZ; ++iZ) {
                ik = std::lower_bound(ik, m_iKFsMatch.end(), m_Zs[iZ].m_iKF);
                m_iZ2k[iZ] = int(ik - m_iKFsMatch.begin());
                UT_ASSERT(ik != m_iKFsMatch.end() && *ik == m_Zs[iZ].m_iKF);
            }

            m_ik2KF = m_iKFsMatch;
            ik = m_iKFsMatch.begin();
            const int Np = int(m_iKFsPrior.size());
            m_iZp2k.resize(Np);
            for (int ip = 0; ip < Np; ++ip) {
                const int iKF = m_iKFsPrior[ip];
                ik = std::lower_bound(ik, m_iKFsMatch.end(), iKF);
                if (ik == m_iKFsMatch.end() || *ik != iKF) {
                    m_iZp2k[ip] = int(m_ik2KF.size());
                    m_ik2KF.push_back(iKF);
                } else
                    m_iZp2k[ip] = int(ik - m_iKFsMatch.begin());
            }

#ifdef CFG_DEPTH_MAP
            int ix, jx;
            const int Nx = int(m_xs.size());
            for (ix = jx = 0; ix < Nx; ++ix) {
                if (m_xs[ix].m_d != 0.0f) ++jx;
            }
            const int Nxd = jx;
            m_Ads.Resize(jx);
            m_Ads.MakeZero();
#endif

            const int Nz = int(KF.m_zs.size());
            m_uzs.assign(Nz, 0);
            m_Adcxs.Resize(Nz);
            m_Adcxs.MakeZero();
            m_Adczs.Resize(Nz);
            m_Adczs.MakeZero();
            m_Acxxs.Resize(Nz);
            m_Acxxs.MakeZero();
            m_Acxzs.Resize(Nz);
            m_Acxzs.MakeZero();
            m_Aczzs.Resize(Nz);
            m_Aczzs.MakeZero();
            m_Mdczs.Resize(Nz);
            m_Mdczs.MakeZero();
            m_Mcxzs.Resize(Nz);
            m_Mcxzs.MakeZero();
            m_Mczzs.Resize(Nz);
            m_Mczzs.MakeZero();
            m_Zm.Initialize();
            m_SAdcxs.Resize(0);
            m_SMdcxs.Resize(0);
            m_SMcxxs.Resize(0);
            // const int Nx = int(KF.m_xs.size());
            // m_SAdcxs.Resize(Nx);	m_SAdcxs.MakeZero();
            // m_SMdcxs.Resize(Nx);	m_SMdcxs.MakeZero();
            // m_SMcxxs.Resize(Nx);	m_SMdcxs.MakeZero();
            m_Ap11s.Resize(Np);
            m_Ap11s.MakeZero();
            m_Ap12s.Resize(Np);
            m_Ap12s.MakeZero();
            m_Ap22s.Resize(Np);
            m_Ap22s.MakeZero();
        }
        inline int
        PushKeyFramePrior(const int iKF, const Camera::Pose::Prior::Rigid &Zp,
                          AlignedVector<Camera::Pose::Prior::Rigid> &ZpsTmp,
                          AlignedVector<Camera::Pose::Unitary> &AusTmp,
                          AlignedVector<Camera::Pose::Binary> &AbsTmp)
        {
            const std::vector<int>::const_iterator ip =
                std::lower_bound(m_iKFsPrior.begin(), m_iKFsPrior.end(), iKF);
            if (ip != m_iKFsPrior.end() && *ip == iKF) return -1;
            int _ip;
            const int ik = SearchKeyFrameMatch(iKF),
                      _ik = ik == -1 ? int(m_ik2KF.size()) : ik;
            if (ip == m_iKFsPrior.end()) {
                _ip = int(m_iKFsPrior.size());
                m_iKFsPrior.push_back(iKF);
                m_Zps.Push(Zp);
                m_iZp2k.push_back(_ik);
                const int Np = _ip + 1;
                m_Ap11s.Resize(Np, true);
                m_Ap12s.Resize(Np, true);
                m_Ap22s.Resize(Np, true);
            } else {
                _ip = int(ip - m_iKFsPrior.begin());
                m_iKFsPrior.insert(ip, iKF);
                m_Zps.Insert(_ip, Zp, ZpsTmp);
                m_iZp2k.insert(m_iZp2k.begin() + _ip, _ik);
                m_Ap11s.Insert(_ip, 1, AusTmp);
                m_Ap12s.Insert(_ip, 1, AbsTmp);
                m_Ap22s.Insert(_ip, 1, AusTmp);
            }
            m_Ap11s[_ip].MakeZero();
            m_Ap12s[_ip].MakeZero();
            m_Ap22s[_ip].MakeZero();
            if (ik == -1) m_ik2KF.push_back(iKF);
            return _ip;
        }
        inline void SaveB(FILE *fp) const
        {
            GlobalMap::KeyFrame::SaveB(fp);
            UT::SaveB(m_iK, fp);
            UT::VectorSaveB(m_iZ2k, fp);
            UT::VectorSaveB(m_iZp2k, fp);
            UT::VectorSaveB(m_ik2KF, fp);
#ifdef CFG_DEPTH_MAP
            m_Ads.SaveB(fp);
#endif
            UT::VectorSaveB(m_uzs, fp);
            m_Adcxs.SaveB(fp);
            m_Adczs.SaveB(fp);
            m_Acxxs.SaveB(fp);
            m_Acxzs.SaveB(fp);
            m_Aczzs.SaveB(fp);
            m_Mdczs.SaveB(fp);
            m_Mcxzs.SaveB(fp);
            m_Mczzs.SaveB(fp);
            m_Zm.SaveB(fp);
            m_SAdcxs.SaveB(fp);
            m_SMdcxs.SaveB(fp);
            m_SMcxxs.SaveB(fp);
            m_Ap11s.SaveB(fp);
            m_Ap12s.SaveB(fp);
            m_Ap22s.SaveB(fp);
        }
        inline void LoadB(FILE *fp)
        {
            GlobalMap::KeyFrame::LoadB(fp);
            UT::LoadB(m_iK, fp);
            UT::VectorLoadB(m_iZ2k, fp);
            UT::VectorLoadB(m_iZp2k, fp);
            UT::VectorLoadB(m_ik2KF, fp);
#ifdef CFG_DEPTH_MAP
            m_Ads.LoadB(fp);
#endif
            UT::VectorLoadB(m_uzs, fp);
            m_Adcxs.LoadB(fp);
            m_Adczs.LoadB(fp);
            m_Acxxs.LoadB(fp);
            m_Acxzs.LoadB(fp);
            m_Aczzs.LoadB(fp);
            m_Mdczs.LoadB(fp);
            m_Mcxzs.LoadB(fp);
            m_Mczzs.LoadB(fp);
            m_Zm.LoadB(fp);
            m_SAdcxs.LoadB(fp);
            m_SMdcxs.LoadB(fp);
            m_SMcxxs.LoadB(fp);
            m_Ap11s.LoadB(fp);
            m_Ap12s.LoadB(fp);
            m_Ap22s.LoadB(fp);
        }
        inline void AssertConsistency(const bool lastKF) const
        {
            GlobalMap::KeyFrame::AssertConsistency();
            const int NZ = int(m_Zs.size());
            UT_ASSERT(int(m_iZ2k.size()) == NZ);
            for (int iZ = 0; iZ < NZ; ++iZ)
                UT_ASSERT(m_Zs[iZ].m_iKF == m_iKFsMatch[m_iZ2k[iZ]]);
            const int Nk = int(m_iKFsMatch.size()),
                      Np = int(m_iKFsPrior.size());
            UT_ASSERT(int(m_iZp2k.size()) == Np);
            for (int ik = 0; ik < Nk; ++ik)
                UT_ASSERT(m_iKFsMatch[ik] == m_ik2KF[ik]);
            int SNk = Nk;
            for (int ip = 0; ip < Np; ++ip) {
                const int iKF = m_iKFsPrior[ip], ik = m_iZp2k[ip];
                UT_ASSERT(m_ik2KF[ik] == iKF);
                if (ik < Nk)
                    UT_ASSERT(m_iKFsMatch[ik] == iKF);
                else
                    ++SNk;
            }
            UT_ASSERT(int(m_ik2KF.size()) == SNk);
            const int Nz = int(m_zs.size()), Nx = int(m_xs.size());
            UT_ASSERT(m_Adcxs.Size() == Nz && m_Adczs.Size() == Nz);
            UT_ASSERT(m_Acxxs.Size() == Nz && m_Acxzs.Size() == Nz &&
                      m_Aczzs.Size() == Nz);
            UT_ASSERT(m_Mdczs.Size() == Nz && m_Mcxzs.Size() == Nz &&
                      m_Mczzs.Size() == Nz);
            m_Zm.AssertConsistency(Nk);
            if (lastKF)
                UT_ASSERT(m_SAdcxs.Empty() && m_SMdcxs.Empty() &&
                          m_SMcxxs.Empty());
            else
                UT_ASSERT(m_SAdcxs.Size() == Nx && m_SMdcxs.Size() == Nx &&
                          m_SMcxxs.Size() == Nx);
            UT_ASSERT(m_Ap11s.Size() == Np && m_Ap12s.Size() == Np &&
                      m_Ap22s.Size() == Np);
        }

      public:
        int m_iK;
        std::vector<int> m_iZ2k, m_iZp2k, m_ik2KF;
#ifdef CFG_DEPTH_MAP
        AlignedVector<FTR::Factor::Source::Depth> m_Ads;
#endif
        // SA : scaled A
        // x features, z measurements
        std::vector<ubyte> m_uzs;
        AlignedVector<FTR::Factor::Source> m_Adcxs;
        AlignedVector<FTR::Factor::Measurement> m_Adczs;
        AlignedVector<Camera::Pose::Unitary> m_Acxxs;
        AlignedVector<Camera::Pose::Binary> m_Acxzs;
        AlignedVector<Camera::Pose::Unitary> m_Aczzs;
        AlignedVector<FTR::Factor::Measurement> m_Mdczs;
        AlignedVector<Camera::Pose::Binary> m_Mcxzs;
        AlignedVector<Camera::Pose::Unitary> m_Mczzs;
        MeasurementMatch m_Zm;
        AlignedVector<FTR::Factor::Source> m_SAdcxs;
        AlignedVector<FTR::Factor::Source> m_SMdcxs;
        AlignedVector<Camera::Pose::Unitary> m_SMcxxs;
        AlignedVector<Camera::Pose::Unitary> m_Ap11s;
        AlignedVector<Camera::Pose::Binary> m_Ap12s;
        AlignedVector<Camera::Pose::Unitary> m_Ap22s;
    };



  protected:
    virtual void SynchronizeData();
    virtual void UpdateData(std::vector<EIBA::KeyFrame> *KFs);
    virtual bool BufferDataEmpty();

    virtual void PushFeatureMeasurementMatchesFirst(const FRM::Frame &F,
                                                    std::vector<int> &iKF2X,
                                                    std::vector<int> &iX2z);
    virtual void PushFeatureMeasurementMatchesNext(
        const FRM::Frame &F1,
        const std::vector<int> &iKF2X, const std::vector<int> &iX2z2,
        KeyFrame::MeasurementMatch &Zm);

    virtual void UpdateFactors();
    virtual void UpdateSchurComplement();
    virtual bool UpdateCameraStates();
    virtual bool UpdateDepthStates();
    virtual bool SolveSchurComplement();
    virtual void ApplyM(const LA::AlignedVectorXf &xs,
                        LA::AlignedVectorXf &Mxs);
    virtual void ApplyA(const LA::AlignedVectorXf &xs,
                        LA::AlignedVectorXf &Axs);
    virtual FTR::Measurement::ES
    ComputeErrorStatisticFeature(const std::vector<Rigid3D> &Cs,
                                 const std::vector<DepthInverseGaussian> &ds);
    virtual Camera::Pose::Prior::Rigid::ES
    ComputeErrorStatisticPrior(const AlignedVector<Rigid3D> &Cs);


    virtual void AssertConsistency();
  public:

    GlobalMap *m_GM;
//    CallBackUpdateKeyFrame m_updateKFCB;
    int m_verbose;
    Intrinsic m_K;

    std::string m_timingFileName;
    FILE *m_fp;

    enum TimerType {
        TM_TOTAL,
        TM_FACTOR,
        TM_SCHUR_COMPLEMENT,
        TM_CAMERA_STATE,
        TM_DEPTH_STATE,
        TM_TYPES
    };
    Timer m_timers[TM_TYPES];

    std::list<InputKeyFrame> m_IKFs1, m_IKFs2;
    std::list<GlobalMap::KeyFramePrior> m_IZps1, m_IZps2;
//    boost::shared_mutex m_MT;
    std::shared_timed_mutex m_MT;

    int m_iKFFix;
    std::vector<KeyFrame> m_KFs;
    std::vector<ubyte> m_ucs, m_uds;
    std::vector<ubyte> m_Ucs, m_Uds;
    // m_CsLP : cameras of last process?
    AlignedVector<Rigid3D> m_Cs, m_CsLP;
#ifdef CFG_GROUND_TRUTH
    AlignedVector<Rigid3D> m_CsGTKF;
#endif
    std::vector<DepthInverseGaussian> m_ds, m_dsLP;

    // AlignedVector<FTR::Factor::Source> m_SAdcxs;
    // AlignedVector<FTR::Factor::Source> m_SMdcxs;
    // AlignedVector<Camera::Pose::Unitary> m_SMcxxs;

    //us unitary, bs binary
    AlignedVector<Camera::Pose::Unitary> m_SAcus;
    AlignedVector<Camera::Pose::Binary> m_SAcbs;

    LA::AlignedVectorXf m_xcs, m_xc2s;
    LA::AlignedVectorXf m_xds, m_xd2s, m_xds2s;
    AlignedVector<LA::AlignedMatrix6x6f> m_Ms, m_Aus, m_Abs, m_AbTs;
    LA::AlignedVectorXf m_bs, m_rs, m_ps, m_zs, m_ts;
    AlignedVector<LA::ProductVector6f> m_xps;

    std::vector<int> m_idxsTmp1, m_idxsTmp2;
    std::vector<FTR::Measurement::Match> m_izmsTmp;
    AlignedVector<Camera::Pose::Prior::Rigid> m_ZpsTmp;
    AlignedVector<Camera::Pose::Unitary> m_AcusTmp;
    AlignedVector<Camera::Pose::Binary> m_AcbsTmp;
    AlignedVector<LA::ProductVector6f> m_MdczsTmp;
    std::vector<float> m_xdsTmp;
    AlignedVector<float> m_work;

#ifdef CFG_DEBUG_EIGEN
  protected:
    class EigenErrorJacobian
    {
      public:
        class Feature
        {
          public:
            union {
                struct {
                    EigenVector3f m_Jxdd;
                    EigenMatrix3x6f m_Jxdcx, m_Jxdcz;
                    EigenVector3f m_exd;
                };
                struct {
                    EigenVector2f m_Jxd;
                    EigenMatrix2x6f m_Jxcx, m_Jxcz;
                    EigenVector2f m_ex;
                };
            };
        };
        class Prior
        {
          public:
            EigenVector6f m_erp;
            EigenMatrix6x6f m_Jrpc1, m_Jrpc2;
        };
    };
    class EigenFactor
    {
      public:
        class Feature
        {
          public:
            class DepthCamera
            {
              public:
                class Source
                {
                  public:
                    inline void operator=(const Eigen::Matrix<float, 1, 14> &A)
                    {
                        m_add = A(0, 0);
                        m_adcx = A.block<1, 6>(0, 1);
                        m_bd = A(0, 13);
                    }
                    inline void operator+=(const Source &A)
                    {
                        m_add += A.m_add;
                        m_adcx += A.m_adcx;
                        m_bd += A.m_bd;
                    }
                    inline void Set(const FTR::Factor::Source &Adcx)
                    {
                        m_add = Adcx.m_ad;
                        m_adcx = EigenVector6f(Adcx.m_ac).transpose();
                        m_bd = Adcx.m_b;
                    }
                    inline void MakeZero()
                    {
                        m_add = 0.0f;
                        m_adcx.setZero();
                        m_bd = 0.0f;
                    }
                    inline void
                    AssertEqual(const FTR::Factor::Source &Adcx) const
                    {
                        const float eps = 1.0e-3f;
                        UT::AssertEqual(m_add, Adcx.m_ad);
                        EigenVector6f(m_adcx.transpose())
                            .AssertEqual(Adcx.m_ac, 1, eps);
                        UT::AssertEqual(m_bd, Adcx.m_b);
                    }

                  public:
                    float m_add;
                    Eigen::Matrix<float, 1, 6> m_adcx;
                    float m_bd;
                };
                typedef Eigen::Matrix<float, 1, 6> Measurement;
            };
            inline void operator=(const Eigen::Matrix<float, 13, 14> &A)
            {
                m_Adcx = A.block<1, 14>(0, 0);
                m_Adcz = A.block<1, 6>(0, 7);
                m_Acxx = A.block<6, 6>(1, 1);
                m_Acxz = A.block<6, 6>(1, 7);
                m_bcx = A.block<6, 1>(1, 13);
                m_Aczz = A.block<6, 6>(7, 7);
                m_bcz = A.block<6, 1>(7, 13);
            }
            inline void Set(const FTR::Factor::Source &Adcx,
                            const FTR::Factor::Measurement &Adcz,
                            const Camera::Pose::Unitary &Acxx,
                            const Camera::Pose::Binary &Acxz,
                            const Camera::Pose::Unitary &Aczz)
            {
                m_Adcx.Set(Adcx);
                m_Adcz = EigenVector6f(Adcz).transpose();
                m_Acxx = EigenMatrix6x6f(Acxx.m_A);
                m_Acxz = EigenMatrix6x6f(Acxz);
                m_bcx = EigenVector6f(Acxx.m_b);
                m_Aczz = EigenMatrix6x6f(Aczz.m_A);
                m_bcz = EigenVector6f(Aczz.m_b);
            }
            inline void Set(const KeyFrame &KF, const int iz)
            {
                Set(KF.m_Adcxs[iz], KF.m_Adczs[iz], KF.m_Acxxs[iz],
                    KF.m_Acxzs[iz], KF.m_Aczzs[iz]);
            }
            inline void AssertEqual(const FTR::Factor::Source &Adcx,
                                    const FTR::Factor::Measurement &Adcz,
                                    const Camera::Pose::Unitary &Acxx,
                                    const Camera::Pose::Binary &Acxz,
                                    const Camera::Pose::Unitary &Aczz) const
            {
                const float eps = 1.0e-3f;
                m_Adcx.AssertEqual(Adcx);
                EigenVector6f(m_Adcz.transpose()).AssertEqual(Adcz, 1, eps);
                m_Acxx.AssertEqual(Acxx.m_A);
                m_Acxz.AssertEqual(Acxz);
                m_Aczz.AssertEqual(Aczz.m_A);
                m_bcx.AssertEqual(Acxx.m_b, 1, eps);
                m_bcz.AssertEqual(Aczz.m_b, 1, eps);
            }
            inline void AssertEqual(const KeyFrame &KF, const int iz) const
            {
                AssertEqual(KF.m_Adcxs[iz], KF.m_Adczs[iz], KF.m_Acxxs[iz],
                            KF.m_Acxzs[iz], KF.m_Aczzs[iz]);
            }

          public:
            DepthCamera::Source m_Adcx;
            DepthCamera::Measurement m_Adcz;
            EigenMatrix6x6f m_Acxx, m_Acxz;
            EigenVector6f m_bcx;
            EigenMatrix6x6f m_Aczz;
            EigenVector6f m_bcz;
        };
        class Prior
        {
          public:
            inline void operator=(const Eigen::Matrix<float, 12, 13> &A)
            {
                m_Ap11 = A.block<6, 6>(0, 0);
                m_Ap12 = A.block<6, 6>(0, 6);
                m_Ap22 = A.block<6, 6>(6, 6);
                m_bp1 = A.block<6, 1>(0, 12);
                m_bp2 = A.block<6, 1>(6, 12);
            }
            inline void Set(const Camera::Pose::Unitary &Ap11,
                            const Camera::Pose::Binary &Ap12,
                            const Camera::Pose::Unitary &Ap22)
            {
                m_Ap11 = Ap11.m_A;
                m_Ap12 = Ap12;
                m_Ap22 = Ap22.m_A;
                m_bp1 = Ap11.m_b;
                m_bp2 = Ap22.m_b;
            }
            inline void Set(const KeyFrame &KF, const int ip)
            {
                Set(KF.m_Ap11s[ip], KF.m_Ap12s[ip], KF.m_Ap22s[ip]);
            }
            inline void AssertEqual(const Camera::Pose::Unitary &Ap11,
                                    const Camera::Pose::Binary &Ap12,
                                    const Camera::Pose::Unitary &Ap22) const
            {
                m_Ap11.AssertEqual(Ap11.m_A);
                m_Ap12.AssertEqual(Ap12);
                m_Ap22.AssertEqual(Ap22.m_A);
                const float eps = 1.0e-3f;
                m_bp1.AssertEqual(Ap11.m_b, 1, eps);
                m_bp2.AssertEqual(Ap22.m_b, 1, eps);
            }
            inline void AssertEqual(const KeyFrame &KF, const int ip) const
            {
                AssertEqual(KF.m_Ap11s[ip], KF.m_Ap12s[ip], KF.m_Ap22s[ip]);
            }

          public:
            EigenMatrix6x6f m_Ap11, m_Ap12, m_Ap22;
            EigenVector6f m_bp1, m_bp2;
        };
    };
    class Track
    {
      public:
        class Measurement
        {
          public:
            inline Measurement() {}
            inline Measurement(const int iKF, const int iz)
                : m_iKF(iKF), m_iz(iz)
            {
            }
            inline bool operator<(const Measurement &z) const
            {
                return m_iKF < z.m_iKF;
            }

          public:
            int m_iKF, m_iz;
        };

      public:
        inline void Initialize()
        {
            m_zs.resize(0);
            m_Adczs.resize(0);
        }

      public:
        std::vector<Measurement> m_zs;
        EigenFactor::Feature::DepthCamera::Source m_SAdcx;
        std::vector<EigenFactor::Feature::DepthCamera::Measurement> m_Adczs;
    };

  protected:
    virtual void DebugGenerateTracks();
    virtual void DebugUpdateFactors();
    virtual void DebugUpdateSchurComplement();
    virtual void DebugUpdateStates();
    virtual EigenErrorJacobian::Feature
    DebugEigenErrorJacobianFeature(const int iKF1, const int iKF2,
                                   const int iz);
    virtual EigenErrorJacobian::Prior
    DebugEigenErrorJacobianPrior(const int iKF1, const int iKF2, const int ip);

  protected:
    std::vector<std::vector<Track>> e_Xs;
    std::vector<std::vector<ubyte>> e_M;
    std::vector<std::vector<int>> e_I;
    std::vector<EigenMatrix6x6f> e_SAccs;
    std::vector<EigenVector6f> e_Sbcs, e_xcs;
#endif
};

#endif
