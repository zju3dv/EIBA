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

#include "GlobalMap.h"

void GlobalMap::IT_Reset(const int nKFsMax)
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_Cs.Reserve(nKFsMax);
    m_Cs.Resize(0);
    m_ds.resize(0);
    m_sIT = m_sLD = m_sLBA = m_sOST = true;
    MT_WRITE_LOCK_END(m_MT);
}

void GlobalMap::IT_Push(const FRM::Tag &T, const Rigid3D &C,
                        const std::vector<DepthInverseGaussianBeta> &ds)
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_Ts.push_back(T);
    m_Cs.Push(C);
    m_ds.insert(m_ds.end(), ds.begin(), ds.end());
    /*m_sLD = m_sLBA = */ m_sOST = false;
    MT_WRITE_LOCK_END(m_MT);
}

bool GlobalMap::IT_Synchronize(AlignedVector<Rigid3D> &Cs,
                               std::vector<DepthInverseGaussian> &ds)
{
    MT_READ_LOCK_BEGIN(m_MT);
    if (m_sIT) return true;
#ifdef CFG_DEBUG
    UT_ASSERT(m_Cs.Size() == Cs.Size() && m_ds.size() == ds.size());
#endif
    Cs.Set(m_Cs);
    ds = m_ds;
    MT_READ_LOCK_END(m_MT);
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_sIT = true;
    MT_WRITE_LOCK_END(m_MT);
    return false;
}

bool GlobalMap::LD_Synchronize(AlignedVector<Rigid3D> &Cs,
                               std::vector<DepthInverseGaussian> &ds)
{
    MT_READ_LOCK_BEGIN(m_MT);
    if (m_sLD) return true;
#ifdef CFG_DEBUG
    UT_ASSERT(m_Cs.Size() == Cs.Size() && m_ds.size() == ds.size());
#endif
    Cs.Set(m_Cs);
    ds = m_ds;
    MT_READ_LOCK_END(m_MT);
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_sLD = true;
    MT_WRITE_LOCK_END(m_MT);
    return false;
}

bool GlobalMap::LBA_Synchronize(AlignedVector<Rigid3D> &Cs)
{
    MT_READ_LOCK_BEGIN(m_MT);
    if (m_sLBA) return true;
#ifdef CFG_DEBUG
    UT_ASSERT(m_Cs.Size() == Cs.Size());
#endif
    Cs.Set(m_Cs);
    MT_READ_LOCK_END(m_MT);
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_sLBA = true;
    MT_WRITE_LOCK_END(m_MT);
    return false;
}

bool GlobalMap::OST_Synchronize(std::vector<FRM::Tag> &Ts,
                                AlignedVector<Rigid3D> &Cs)
{
    MT_READ_LOCK_BEGIN(m_MT);
    if (m_sOST) return true;
    Ts = m_Ts;
    Cs.Set(m_Cs);
    MT_READ_LOCK_END(m_MT);
    MT_WRITE_LOCK_BEGIN(m_MT);
    m_sOST = true;
    MT_WRITE_LOCK_END(m_MT);
    return false;
}

void GlobalMap::GBA_Update(const AlignedVector<Rigid3D> &Cs,
                           const std::vector<DepthInverseGaussian> &ds)
{
//    MT_WRITE_LOCK_BEGIN(m_MT);
// m_Cs.Set(Cs);
// m_ds = ds;
#ifdef CFG_DEBUG
    UT_ASSERT(m_Cs.Size() >= Cs.Size() && m_ds.size() >= ds.size());
#endif
    m_Cs.Resize(Cs.Size());
    m_ds.resize(ds.size());
    memcpy(m_Cs.Data(), Cs.Data(), sizeof(Rigid3D) * Cs.Size());
    memcpy(m_ds.data(), ds.data(), sizeof(DepthInverseGaussian) * ds.size());
    m_sIT = m_sLD = m_sLBA = m_sOST = false;
//    MT_WRITE_LOCK_END(m_MT);
}

void GlobalMap::SaveB(FILE *fp)
{
    MT_READ_LOCK_BEGIN(m_MT);
    UT::SaveB(m_K, fp);
    UT::SaveB(m_pIMU, fp);
    FRM::VectorSaveB(m_Ts, fp);
    m_Cs.SaveB(fp);
    UT::VectorSaveB(m_ds, fp);
    UT::SaveB(m_sIT, fp);
    UT::SaveB(m_sLD, fp);
    UT::SaveB(m_sLBA, fp);
    UT::SaveB(m_sOST, fp);
    MT_READ_LOCK_END(m_MT);
}

void GlobalMap::LoadB(FILE *fp)
{
    MT_WRITE_LOCK_BEGIN(m_MT);
    UT::LoadB(m_K, fp);
    UT::LoadB(m_pIMU, fp);
    FRM::VectorLoadB(m_Ts, fp);
    m_Cs.LoadB(fp);
    UT::VectorLoadB(m_ds, fp);
    UT::LoadB(m_sIT, fp);
    UT::LoadB(m_sLD, fp);
    UT::LoadB(m_sLBA, fp);
    UT::LoadB(m_sOST, fp);
    MT_WRITE_LOCK_END(m_MT);
}

void GlobalMap::AssertConsistency()
{
    MT_READ_LOCK_BEGIN(m_MT);
    const int nKFs = int(m_Ts.size());
    for (int iKF = 0; iKF < nKFs; ++iKF) {
        if (iKF > 0) UT_ASSERT(m_Ts[iKF - 1] < m_Ts[iKF]);
        m_Cs[iKF].AssertOrthogonal();
    }
    MT_READ_LOCK_END(m_MT);
}
