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

#ifndef _GLOBAL_MAP_H_
#define _GLOBAL_MAP_H_

#include "stdafx.h"

#include "Depth.h"
#include "Frame.h"
#include "MultiThread.h"
#include "Rigid.h"

class GlobalMap
{
  public:
    class KeyFrame : public FRM::Frame
    {
      public:
        inline KeyFrame() : FRM::Frame() {}
        inline KeyFrame(const KeyFrame &KF) { *this = KF; }
        inline void operator=(const KeyFrame &KF)
        {
            *((FRM::Frame *)this) = KF;
            m_id = KF.m_id;
            memcpy(m_ixIP, KF.m_ixIP, sizeof(m_ixIP));
            m_xs = KF.m_xs;
            m_iKFsPrior = KF.m_iKFsPrior;
            m_Zps.Set(KF.m_Zps);
        }
        inline bool operator==(const KeyFrame &KF) const
        {
            return *((FRM::Frame *)this) == KF && m_id == KF.m_id &&
                   UT::VectorEqual(m_ixIP, KF.m_ixIP, FRM_IP_LEVELS_PLUS_ONE) &&
                   UT::VectorEqual(m_xs, KF.m_xs) &&
                   UT::VectorEqual(m_iKFsPrior, KF.m_iKFsPrior);
        }
        inline bool operator==(const int iFrm) const
        {
            return m_T.m_iFrm == iFrm;
        }
        inline bool operator<(const int iFrm) const
        {
            return m_T.m_iFrm < iFrm;
        }
        inline void Initialize(const FRM::Frame &F, const int id)
        {
            FRM::Frame::Initialize(F);
            m_id = id;
            memset(m_ixIP, 0, sizeof(m_ixIP));
            m_xs.resize(0);
            m_iKFsPrior.resize(0);
            m_Zps.Resize(0);
        }
#if 0
		inline int SearchFeatureLevel(const int ix) const
		{
#ifdef CFG_DEBUG
			UT_ASSERT(ix < int(m_xs.size()));
#endif
			for(int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl)
			{
				if(m_ixIP[iLvl + 1] > ix)
					return iLvl;
			}
			return -1;
		}
#endif
        inline int SearchKeyFrameMatch(const int iKF) const
        {
            const std::vector<int>::const_iterator ik =
                std::lower_bound(m_iKFsMatch.begin(), m_iKFsMatch.end(), iKF);
            return ik == m_iKFsMatch.end() || *ik != iKF
                       ? -1
                       : int(ik - m_iKFsMatch.begin());
        }
        inline int SearchKeyFramePrior(const int iKF) const
        {
            const std::vector<int>::const_iterator ip =
                std::lower_bound(m_iKFsPrior.begin(), m_iKFsPrior.end(), iKF);
            return ip == m_iKFsPrior.end() || *ip != iKF
                       ? -1
                       : int(ip - m_iKFsPrior.begin());
        }
        inline bool
        PushKeyFramePrior(const int iKF, const Camera::Pose::Prior::Rigid &Zp,
                          AlignedVector<Camera::Pose::Prior::Rigid> &ZpsTmp)
        {
            const std::vector<int>::const_iterator ip =
                std::lower_bound(m_iKFsPrior.begin(), m_iKFsPrior.end(), iKF);
            if (ip != m_iKFsPrior.end() && *ip == iKF) return false;
            if (ip == m_iKFsPrior.end()) {
                m_iKFsPrior.push_back(iKF);
                m_Zps.Push(Zp);
            } else {
                m_Zps.Insert(int(ip - m_iKFsPrior.begin()), Zp, ZpsTmp);
                m_iKFsPrior.insert(ip, iKF);
            }
            return true;
        }
        inline void SaveB(FILE *fp) const
        {
            FRM::Frame::SaveB(fp);
            UT::SaveB(m_id, fp);
            UT::SaveB(m_ixIP, FRM_IP_LEVELS_PLUS_ONE, fp);
            UT::VectorSaveB(m_xs, fp);
            UT::VectorSaveB(m_iKFsPrior, fp);
            m_Zps.SaveB(fp);
        }
        inline void LoadB(FILE *fp)
        {
            FRM::Frame::LoadB(fp);
            UT::LoadB(m_id, fp);
            UT::LoadB(m_ixIP, FRM_IP_LEVELS_PLUS_ONE, fp);
            UT::VectorLoadB(m_xs, fp);
            printf("source m_xs(2D+inv_d) %d \n", m_xs.size());
            UT::VectorLoadB(m_iKFsPrior, fp);
            m_Zps.LoadB(fp);
        }
        inline void AssertConsistency() const
        {
            Frame::AssertConsistency();
            UT_ASSERT(m_ixIP[0] == 0);
            for (int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl)
                UT_ASSERT(m_ixIP[iLvl] <= m_ixIP[iLvl + 1]);
            UT_ASSERT(m_ixIP[FRM_IP_LEVELS] == int(m_xs.size()));
            const int Np = int(m_iKFsPrior.size());
            UT_ASSERT(m_Zps.Size() == Np);
            for (int ip = 0; ip < Np; ++ip) {
                if (ip > 0) UT_ASSERT(m_iKFsPrior[ip - 1] < m_iKFsPrior[ip]);
                m_Zps[ip].AssertConsistency();
            }
        }

      public:
        // m_id : start index (global) of current keyframe's features
        // m_ixIP : start index (local) of current keyframe's features in each level, start from 0
        int m_id, m_ixIP[FRM_IP_LEVELS_PLUS_ONE];
        // source features
        std::vector<FTR::Source> m_xs;
        // it seems that this is always empty
        std::vector<int> m_iKFsPrior;
        // it seems that this is always empty
        AlignedVector<Camera::Pose::Prior::Rigid> m_Zps;
    };

    class KeyFramePrior
    {
      public:
        inline KeyFramePrior() {}
        inline KeyFramePrior(const int iKF1, const int iKF2,
                             const Camera::Pose::Prior::Rigid &Zp)
            : m_iKF1(iKF1), m_iKF2(iKF2), m_Zp(Zp)
        {
        }

      public:
        int m_iKF1, m_iKF2;
        Camera::Pose::Prior::Rigid m_Zp;
    };

  public:
    void IT_Reset(const int nKFsMax);
    void IT_Push(const FRM::Tag &T, const Rigid3D &C,
                 const std::vector<DepthInverseGaussianBeta> &ds);
    bool IT_Synchronize(AlignedVector<Rigid3D> &Cs,
                        std::vector<DepthInverseGaussian> &ds);
    bool LD_Synchronize(AlignedVector<Rigid3D> &Cs,
                        std::vector<DepthInverseGaussian> &ds);
    bool LBA_Synchronize(AlignedVector<Rigid3D> &Cs);
    bool OST_Synchronize(std::vector<FRM::Tag> &Ts, AlignedVector<Rigid3D> &Cs);
    void GBA_Update(const AlignedVector<Rigid3D> &Cs,
                    const std::vector<DepthInverseGaussian> &ds);

    void SaveB(FILE *fp);
    void LoadB(FILE *fp);
    void AssertConsistency();

  public:

    Intrinsic m_K;
    Point3D m_pIMU;

  protected:
    std::vector<FRM::Tag> m_Ts;
    AlignedVector<Rigid3D> m_Cs;
    std::vector<DepthInverseGaussian> m_ds;
    bool m_sIT, m_sLD, m_sLBA, m_sOST;
//    boost::shared_mutex m_MT;
    std::shared_timed_mutex m_MT;
};

#endif
