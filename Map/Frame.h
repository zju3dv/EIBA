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

#ifndef _FRAME_H_

#include "stdafx.h"

#include "Camera.h"
#include "Feature.h"
#include "M-Estimator.h"
#include "Matrix2x3.h"
#include "Matrix2x4.h"
#include "Matrix8x8.h"

#define FRM_IP_LEVELS 4
#define FRM_IP_LEVELS_MINUS_ONE 3
#define FRM_IP_LEVELS_PLUS_ONE 5
//#define FRM_IT_LEVEL			4

#define FRM_IT_CHECK_INVALID_DEFAULT 0
#define FRM_IT_CHKCK_INVALID_FIRST 1
#define FRM_IT_CHKCK_INVALID_ALL 2

//#define FRM_ALN_DECOUPLE_RIGID

#define FRM_ALN_FLAG_DEFAULT 0
#define FRM_ALN_FLAG_ROTATION 1
#define FRM_ALN_FLAG_POSITION 2
#define FRM_ALN_FLAG_DEPTH 4
#define FRM_ALN_FLAG_INITIALIZED 8

#ifdef CFG_TUNE_PARAMETERS
extern int FRM_IT_LEVEL;
extern float FRM_IT_BLUR_SIGMA;
extern float FRM_ALN_WEIGHT_INTENSITY;
extern float FRM_ALN_WEIGHT_PRIOR_ROTATION;
extern int FRM_ALN_MAX_ITERATIONS;
extern float FRM_ALN_VARIANCE_INTENSITY;
extern float FRM_ALN_VARIANCE_MATCH;
extern float FRM_ALN_VARIANCE_PRIOR_INTENSITY_OFFSET;
extern float FRM_ALN_VARIANCE_PRIOR_DEPTH;
#ifdef CFG_DEPTH_MAP
extern float FRM_ALN_VARIANCE_PRIOR_DEPTH_MAP;
#endif
extern float FRM_ALN_VARIANCE_PRIOR_ROTATION;
extern float FRM_ALN_VARIANCE_PRIOR_POSITION_X;
extern float FRM_ALN_VARIANCE_PRIOR_POSITION_Y;
extern float FRM_ALN_VARIANCE_PRIOR_POSITION_Z;
extern float FRM_ALN_MAX_ERROR_INTENSITY;
extern float FRM_ALN_MAX_ERROR_MATCH;
extern float FRM_ALN_MIN_OVERLAP_RATIO;
extern float FRM_ALN_MIN_OVERLAP_RATIO_ROBUST;
extern float FRM_ALN_CONVERGE_MOVEMENT;
extern float FRM_ALN_CONVERGE_DEPTH;
extern float FRM_ALN_EPSILON_ROTATION;
extern float FRM_ALN_EPSILON_POSITION;
#include "Configurator.h"
extern void LOAD_PARAMETERS_FRAME_ALIGNMENT(const Configurator &cfgor);
#else
#define FRM_IT_LEVEL 4
#define FRM_IT_BLUR_SIGMA 1.5f
#define FRM_ALN_WEIGHT_INTENSITY 1.0e-3f
//#define FRM_ALN_WEIGHT_INTENSITY				1.0f
#define FRM_ALN_WEIGHT_PRIOR_ROTATION 1.0e-3f
//#define FRM_ALN_WEIGHT_PRIOR_ROTATION			1.0f
#define FRM_ALN_MAX_ITERATIONS 20
#define FRM_ALN_VARIANCE_INTENSITY 100.0f // 10^2
#define FRM_ALN_VARIANCE_MATCH 1.0f       // 1^2
#define FRM_ALN_VARIANCE_PRIOR_INTENSITY_OFFSET 9.0f // 3^2
//#define FRM_ALN_VARIANCE_PRIOR_DEPTH			1.0f
//// 1^2
#define FRM_ALN_VARIANCE_PRIOR_DEPTH 100.0f // 10^2
#ifdef CFG_DEPTH_MAP
//#define FRM_ALN_VARIANCE_PRIOR_DEPTH_MAP		0.01f
//// 0.1^2
#define FRM_ALN_VARIANCE_PRIOR_DEPTH_MAP 1.0f // 1^2
#endif
#define FRM_ALN_VARIANCE_PRIOR_ROTATION 3.046174198662e-4f // (1.0*pi/180)^2
#define FRM_ALN_VARIANCE_PRIOR_POSITION_X 9.0f             // 3^2+
#define FRM_ALN_VARIANCE_PRIOR_POSITION_Y 9.0f
#define FRM_ALN_VARIANCE_PRIOR_POSITION_Z 9.0f
#define FRM_ALN_MAX_ERROR_INTENSITY 20.0f
#define FRM_ALN_MAX_ERROR_MATCH 1.0f
#define FRM_ALN_MIN_OVERLAP_RATIO 0.3f
#define FRM_ALN_MIN_OVERLAP_RATIO_ROBUST 0.3f
#define FRM_ALN_CONVERGE_MOVEMENT 0.01f
//#define FRM_ALN_CONVERGE_MOVEMENT				0.1f
#define FRM_ALN_CONVERGE_DEPTH 0.01f
//#define FRM_ALN_CONVERGE_DEPTH				0.1f
#define FRM_ALN_EPSILON_ROTATION 1.745329252e-4f // 0.01*pi/180
#define FRM_ALN_EPSILON_POSITION 0.001f
#endif

namespace FRM
{
template <class TYPE>
inline void VectorSaveB(const std::vector<TYPE> &V, FILE *fp)
{
    const int N = int(V.size());
    UT::SaveB(N, fp);
    for (int i = 0; i < N; ++i) V[i].SaveB(fp);
}
template <class TYPE> inline void VectorLoadB(std::vector<TYPE> &V, FILE *fp)
{
    const int N = UT::LoadB<int>(fp);
    V.resize(N);
    for (int i = 0; i < N; ++i) V[i].LoadB(fp);
}

template <class TYPE> inline void ListSaveB(const std::list<TYPE> &L, FILE *fp)
{
    const int N = int(L.size());
    UT::SaveB(N, fp);
    for (typename std::list<TYPE>::const_iterator it = L.begin(); it != L.end();
         ++it)
        it->SaveB(fp);
}
template <class TYPE> inline void ListLoadB(std::list<TYPE> &L, FILE *fp)
{
    const int N = UT::LoadB<int>(fp);
    L.resize(N);
    for (typename std::list<TYPE>::iterator it = L.begin(); it != L.end(); ++it)
        it->LoadB(fp);
}

// store frame tag
class Tag
{
  public:
    inline bool operator==(const Tag &T) const
    {
        return m_iFrm == T.m_iFrm && m_t == T.m_t;
    }
    inline bool operator!=(const Tag &T) const
    {
        return m_iFrm != T.m_iFrm || m_t != T.m_t;
    }
    inline bool operator<(const Tag &T) const
    {
        return m_iFrm < T.m_iFrm && m_t < T.m_t;
    }
    inline bool operator>(const Tag &T) const
    {
        return m_iFrm > T.m_iFrm && m_t > T.m_t;
    }
    inline void SaveB(FILE *fp) const
    {
        UT::SaveB(m_iFrm, fp);
        UT::SaveB(m_t, fp);
#ifdef CFG_VIEW
        UT::StringSaveB(m_fileName, fp);
#ifdef CFG_DEPTH_MAP
        UT::StringSaveB(m_fileNameDep, fp);
#endif
#endif
    }
    inline void LoadB(FILE *fp)
    {
        UT::LoadB(m_iFrm, fp);
        UT::LoadB(m_t, fp);
        UT::StringLoadB(m_fileName, fp);
        UT::StringLoadB(m_fileNameDep, fp);
    }

  public:
    int m_iFrm;
    float m_t;
    std::string m_fileName;
    std::string m_fileNameDep;
};

class Measurement
{
  public:
    inline Measurement() {}
    inline Measurement(const int iKF, const int izIP[FRM_IP_LEVELS_PLUS_ONE])
    {
        m_iKF = iKF;
        memcpy(m_izIP, izIP, sizeof(m_izIP));
    }
    inline bool operator==(const Measurement &Z) const
    {
        return m_iKF == Z.m_iKF &&
               UT::VectorEqual(m_izIP, Z.m_izIP, FRM_IP_LEVELS_PLUS_ONE);
    }
    inline bool operator<(const int iKF) const { return m_iKF < iKF; }
    inline int SearchFeatureMeasurementLevel(const int iz) const
    {
        if (iz < m_izIP[0] || iz >= m_izIP[FRM_IP_LEVELS]) return -1;
        for (int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl) {
            if (iz < m_izIP[iLvl + 1]) return iLvl;
        }
        return -1;
    }
    inline void AssertConsistency() const
    {
        UT_ASSERT(m_iKF >= 0);
        for (int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl)
            UT_ASSERT(m_izIP[iLvl] <= m_izIP[iLvl + 1]);
    }

  public:
    // m_iKF : keyframe index
    // m_izIP : measurement start index for every image level
    int m_iKF, m_izIP[FRM_IP_LEVELS_PLUS_ONE];
};

class Frame
{
  public:
    inline Frame() {}
    inline Frame(const Frame &F) { *this = F; }
    inline void operator=(const Frame &F)
    {
        m_T = F.m_T;
        m_Zs = F.m_Zs;
        m_zs = F.m_zs;
        m_iKFsMatch = F.m_iKFsMatch;
    }
    inline bool operator==(const Frame &F) const
    {
        return m_T == F.m_T && UT::VectorEqual(m_Zs, F.m_Zs) &&
               UT::VectorEqual(m_zs, F.m_zs) &&
               UT::VectorEqual(m_iKFsMatch, F.m_iKFsMatch);
    }
    inline void Initialize(const Tag &T)
    {
        m_T = T;
        ClearMeasurements();
    }
    inline void Initialize(const Frame &F)
    {
        m_T = F.m_T;
        m_Zs = F.m_Zs;
        m_zs = F.m_zs;
        m_iKFsMatch = F.m_iKFsMatch;
    }
    inline void ClearMeasurements()
    {
        m_Zs.resize(0);
        m_zs.resize(0);
        m_iKFsMatch.resize(0);
    }
    inline int SearchFrameMeasurement(const int iKF) const
    {
        const std::vector<Measurement>::const_iterator iZ =
            std::lower_bound(m_Zs.begin(), m_Zs.end(), iKF);
        if (iZ == m_Zs.end() || iZ->m_iKF != iKF)
            return -1;
        else
            return int(iZ - m_Zs.begin());
    }
    inline int SearchFeatureMeasurement(const int iKF, const int ix) const
    {
        const int iZ = SearchFrameMeasurement(iKF);
        if (iZ == -1) return -1;
        const Measurement &Z = m_Zs[iZ];
        for (int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl) {
            const std::vector<FTR::Measurement>::const_iterator
                iz1 = m_zs.begin() + Z.m_izIP[iLvl],
                iz2 = m_zs.begin() + Z.m_izIP[iLvl + 1];
            const std::vector<FTR::Measurement>::const_iterator iz =
                std::lower_bound(iz1, iz2, ix);
            if (iz != iz2 && iz->m_ix == ix) return int(iz - m_zs.begin());
        }
        return -1;
    }
#if 0
	inline bool SearchFeatureMeasurement(const int iKF, const int ix, int &iz, int &iLvl) const
	{
		iz = iLvl = -1;
		const int iZ = SearchFrameMeasurement(iKF);
		if(iZ == -1)
			return false;
		const Measurement &Z = m_Zs[iZ];
		for(int _iLvl = 0; _iLvl < FRM_IP_LEVELS; ++_iLvl)
		{
			const std::vector<FTR::Measurement>::const_iterator iz1 = m_zs.begin() + Z.m_izIP[_iLvl], iz2 = m_zs.begin() + Z.m_izIP[_iLvl + 1];
			const std::vector<FTR::Measurement>::const_iterator _iz = std::lower_bound(iz1, iz2, ix);
			if(_iz != iz2 && _iz->m_ix == ix)
			{
				iz = int(_iz - m_zs.begin());
				iLvl = _iLvl;
				return true;
			}
		}
		return false;
	}
#endif
    inline int SearchFeatureMeasurementLevel(const int iz) const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(iz < int(m_zs.size()));
#endif
        const int NZ = int(m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const Measurement &Z = m_Zs[iZ];
            if (Z.m_izIP[FRM_IP_LEVELS] <= iz) continue;
            for (int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl) {
                if (Z.m_izIP[iLvl + 1] > iz) return iLvl;
            }
        }
        return -1;
    }
    inline void SearchFeatureMeasurementMatches(
        const Frame &F, std::vector<FTR::Measurement::Match> &izms) const
    {
        izms.resize(0);
        const int NZ1 = int(m_Zs.size());
        for (int iZ1 = 0; iZ1 < NZ1; ++iZ1) {
            const Measurement &Z1 = m_Zs[iZ1];
            const int iZ2 = F.SearchFrameMeasurement(Z1.m_iKF);
            if (iZ2 == -1) continue;
            const Measurement &Z2 = F.m_Zs[iZ2];
            const std::vector<FTR::Measurement>::const_iterator
                iz21 = F.m_zs.begin() + Z2.m_izIP[0],
                iz22 = F.m_zs.begin() + Z2.m_izIP[FRM_IP_LEVELS];
            const int iz11 = Z1.m_izIP[0], iz12 = Z1.m_izIP[FRM_IP_LEVELS];
            for (int iz1 = iz11; iz1 < iz12; ++iz1) {
                const int ix = m_zs[iz1].m_ix;
                const std::vector<FTR::Measurement>::const_iterator iz2 =
                    std::lower_bound(iz21, iz22, ix);
                if (iz2 != iz22 && iz2->m_ix == ix)
                    izms.push_back(FTR::Measurement::Match(
                        iz1, int(iz2 - F.m_zs.begin())));
            }
        }
    }
    inline int CountFeatureMeasurements(const int iLvl) const
    {
        int SN = 0;
        const int NZ = int(m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const Measurement &Z = m_Zs[iZ];
            SN += Z.m_izIP[iLvl + 1] - Z.m_izIP[iLvl];
        }
        return SN;
    }
    inline void PyramidToFirstLevel()
    {
        const int NZ = int(m_Zs.size());
        for (int iZ = 0; iZ < NZ; ++iZ) {
            Measurement &Z = m_Zs[iZ];
            const int iz1 = Z.m_izIP[0], iz2 = Z.m_izIP[FRM_IP_LEVELS];
            for (int iLvl = 1; iLvl < FRM_IP_LEVELS; ++iLvl)
                Z.m_izIP[iLvl] = iz2;
            std::sort(m_zs.begin() + iz1, m_zs.begin() + iz2);
        }
    }
    inline void SaveB(FILE *fp) const
    {
        m_T.SaveB(fp);
        UT::VectorSaveB(m_Zs, fp);
        UT::VectorSaveB(m_zs, fp);
        UT::VectorSaveB(m_iKFsMatch, fp);
    }
    inline void LoadB(FILE *fp)
    {
        m_T.LoadB(fp);
        UT::VectorLoadB(m_Zs, fp);
        UT::VectorLoadB(m_zs, fp);
        UT::VectorLoadB(m_iKFsMatch, fp);
    }
    inline void AssertConsistency() const
    {
        const int NZ = int(m_Zs.size());
        if (NZ == 0) return;
        for (int iZ = 0; iZ < NZ; ++iZ) m_Zs[iZ].AssertConsistency();
        UT_ASSERT(m_Zs.front().m_izIP[0] == 0);
        for (int iZ = 1; iZ < NZ; ++iZ) {
            const Measurement &Z1 = m_Zs[iZ - 1], &Z2 = m_Zs[iZ];
            UT_ASSERT(Z1.m_iKF < Z2.m_iKF);
            UT_ASSERT(Z1.m_izIP[FRM_IP_LEVELS] == Z2.m_izIP[0]);
        }
        UT_ASSERT(m_Zs.back().m_izIP[FRM_IP_LEVELS] == int(m_zs.size()));
        const int nKFsMatch = int(m_iKFsMatch.size());
        for (int i = 1; i < nKFsMatch; ++i)
            UT_ASSERT(m_iKFsMatch[i - 1] < m_iKFsMatch[i]);
        std::vector<int>::const_iterator ik = m_iKFsMatch.begin();
        for (int iZ = 0; iZ < NZ; ++iZ) {
            const Measurement &Z = m_Zs[iZ];
            for (int iLvl = 0; iLvl < FRM_IP_LEVELS; ++iLvl) {
                const int iz1 = Z.m_izIP[iLvl], iz2 = Z.m_izIP[iLvl + 1];
                for (int iz = iz1 + 1; iz < iz2; ++iz)
                    UT_ASSERT(m_zs[iz - 1].m_ix < m_zs[iz].m_ix);
            }
            ik = std::lower_bound(ik, m_iKFsMatch.end(), Z.m_iKF);
            UT_ASSERT(ik != m_iKFsMatch.end() && *ik == Z.m_iKF);
        }
    }

  public:
    // frame tag info
    Tag m_T;
    // measurements start index (store each matched frames by pyramid level)
    std::vector<Measurement> m_Zs;
    // feature measurements
    std::vector<FTR::Measurement> m_zs;
    // matched keyframe's index
    std::vector<int> m_iKFsMatch;
};

namespace ALN
{
class Jacobian
{
  public:
    inline bool Empty() const { return m_jxrs.Empty(); }
    inline void Resize(const int N)
    {
        m_jxrs.Resize(N);
        m_jxhs.Resize(N);
    }

  public:
    AlignedVector<LA::AlignedMatrix2x3f> m_jxrs;
    AlignedVector<LA::AlignedMatrix2x4f> m_jxhs;
};

class Factor
{
  public:
    class Data3
    {
        struct CanNotMakeAnonymous {
            LA::AlignedVector3f m_j;
            LA::SymmetricMatrix3x3f m_a;
        };

      public:
        CanNotMakeAnonymous fe;
        _pi__m128 m_data[3];
    };
    class Data6
    {
        struct CanNotMakeAnonymous {
            LA::AlignedVector6f m_j;
            LA::SymmetricMatrix3x3f m_arr, m_app;
            LA::Matrix3x3f m_arp;
        };

      public:
        CanNotMakeAnonymous fe;
        _pi__m128 m_data[8];
    };
    class Data8
    {
      public:
        LA::AlignedVector8f m_j;
        LA::SymmetricMatrix8x8f m_a;
    };

  public:
    inline void Resize(const int N)
    {
        m_ars.Resize(N);
        m_ahs.Resize(N);
#ifdef FRM_ALN_DECOUPLE_RIGID
        m_aps.Resize(N);
#else
        m_arps.Resize(N);
#endif
    }

  public:
    AlignedVector<Data3> m_ars;
    AlignedVector<Data8> m_ahs;
#ifdef FRM_ALN_DECOUPLE_RIGID
    AlignedVector<Data3> m_aps;
#else
    AlignedVector<Data6> m_arps;
#endif
};

class MatchList
{
  public:
    inline MatchList(AlignedVector<Point2D> &x1s, AlignedVector<Point2D> &x2s)
        : m_x1s(x1s), m_x2s(x2s)
    {
    }
    inline int Size() const { return m_x1s.Size(); }
  public:
    AlignedVector<Point2D> m_x1s, m_x2s;
};
}
}

#endif
