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

#ifndef _ALIGNED_VECTOR_H_
#define _ALIGNED_VECTOR_H_

#include "Utility.h"

template <class TYPE, const int GROWTH = 0, const int GROWTH_MAX = 1024>
class AlignedVector
{
  public:
    inline AlignedVector()
    {
        m_own = true;
        m_data = NULL;
        m_size = m_capacity = 0;
    }
    inline AlignedVector(const int N)
    {
        m_own = true;
        m_data = SSE::Malloc<TYPE>(N);
        m_size = m_capacity = N;
    }
    // inline AlignedVector(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
    //{
    //	m_own = true;
    //	m_data = NULL;
    //	m_size = m_capacity = 0;
    //	Set(V);
    //}
    inline AlignedVector(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
    {
        m_own = true;
        m_data = NULL;
        m_size = m_capacity = 0;
        Set<false>((TYPE *)V.Data(), V.Size());
    }
    inline AlignedVector(TYPE *V, const int N, const bool own = true)
    {
        m_own = true;
        m_data = NULL;
        m_size = m_capacity = 0;
        if (own)
            Set<true>(V, N);
        else
            Set<false>(V, N);
    }
    inline ~AlignedVector()
    {
        if (m_data && m_own) SSE::Free<TYPE>(m_data);
    }
    // inline void operator = (const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
    // { Set(V); }
    inline void operator=(AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
    {
        Set<false>(V.Data(), V.Size());
    }
    inline void Resize(const int N, const bool retain = false)
    {
        if (N <= m_capacity)
            m_size = N;
        else {
            // if(m_data && m_own)
            //	SSE::Free<TYPE>(m_data);
            // else
            //	m_own = true;
            // m_data = SSE::Malloc<TYPE>(N);

            TYPE *dataBkp = m_data;
            m_data = SSE::Malloc<TYPE>(N);
            if (dataBkp) {
                if (retain) memcpy(m_data, dataBkp, sizeof(TYPE) * m_size);
                if (m_own) SSE::Free<TYPE>(dataBkp);
            }
            m_own = true;

            m_size = m_capacity = N;
        }
    }
    inline void Reserve(const int N)
    {
        Clear();
        m_data = SSE::Malloc<TYPE>(N);
        m_capacity = N;
    }
    inline void Clear()
    {
        if (m_data && m_own) SSE::Free<TYPE>(m_data);
        m_own = true;
        m_data = NULL;
        m_size = m_capacity = 0;
    }
    inline bool Empty() const { return m_size == 0; }
    inline void Push(const TYPE &v)
    {
        if (m_size == m_capacity) {
            const int growth =
                GROWTH == 0 ? std::max(std::min(m_capacity, GROWTH_MAX), 1)
                            : GROWTH;
            m_capacity += growth;
            TYPE *dataBkp = m_data;
            m_data = SSE::Malloc<TYPE>(m_capacity);
            if (dataBkp) {
                memcpy(m_data, dataBkp, sizeof(TYPE) * m_size);
                if (m_own) SSE::Free<TYPE>(dataBkp);
            }
            m_data[m_size++] = v;
            m_own = true;
        } else
            m_data[m_size++] = v;
    }
    inline void Push(const TYPE *V, const int N)
    {
        if (m_size + N > m_capacity) {
            const int growth = N + GROWTH - (GROWTH % N);
            m_capacity += growth;
            TYPE *dataBkp = m_data;
            m_data = SSE::Malloc<TYPE>(m_capacity);
            if (dataBkp) {
                memcpy(m_data, dataBkp, sizeof(TYPE) * m_size);
                if (m_own) SSE::Free<TYPE>(dataBkp);
            }
            m_own = true;
        }
        memcpy(m_data + m_size, V, sizeof(TYPE) * N);
        m_size += N;
    }
    inline void Insert(const int i, const int N,
                       AlignedVector<TYPE, GROWTH, GROWTH_MAX> &VTmp)
    {
        VTmp.Set<true>(Data() + i, Size() - i);
        Resize(i + N, true);
        Push(VTmp.Data(), VTmp.Size());
    }
    inline void Insert(const int i, const TYPE &v,
                       AlignedVector<TYPE, GROWTH, GROWTH_MAX> &VTmp)
    {
        VTmp.Set<true>(Data() + i, Size() - i);
        Resize(i + 1, true);
        m_data[i] = v;
        Push(VTmp.Data(), VTmp.Size());
    }
    inline void Erase(const int i)
    {
        for (int j = i; j < m_size; ++j) m_data[j] = m_data[j + 1];
        Resize(m_size - 1);
    }
    inline void MakeZero() { memset(Data(), 0, sizeof(TYPE) * Size()); }
    inline void MakeZero(const int i1, const int i2)
    {
        memset(Data() + i1, 0, sizeof(TYPE) * (i2 - i1));
    }
    template <bool OWN> inline void Set(TYPE *V, const int N)
    {
        OWN ? Set_True(V, N) : Set_False(V, N);
    }
    inline void Set_True(TYPE *V, const int N)
    {
        Resize(N);
        memcpy(Data(), V, sizeof(TYPE) * N);
    }
    inline void Set_False(TYPE *V, const int N)
    {
        if (m_data && m_own) SSE::Free<TYPE>(m_data);
        m_data = V;
        m_size = m_capacity = N;
        m_own = false;
    }
    inline void Set(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
    {
        const int N = V.Size();
        Resize(N);
        memcpy(m_data, V.Data(), sizeof(TYPE) * N);
    }
    inline void Set(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V,
                    const std::vector<int> &iVs)
    {
        const int N = int(iVs.size());
        Resize(N);
        for (int i = 0; i < N; ++i) m_data[i] = V[iVs[i]];
    }
    inline void Get(TYPE *V) const { memcpy(V, m_data, sizeof(TYPE) * m_size); }
    inline void Swap(AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V)
    {
        UT_SWAP(m_own, V.m_own);
        UT_SWAP(m_data, V.m_data);
        UT_SWAP(m_size, V.m_size);
        UT_SWAP(m_capacity, V.m_capacity);
    }
    inline void Concatenate(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V1,
                            const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V2)
    {
        Resize(V1.Size() + V2.Size());
        memcpy(m_data, V1.Data(), sizeof(TYPE) * V1.Size());
        memcpy(m_data + V1.Size(), V2.Data(), sizeof(TYPE) * V2.Size());
    }

    inline bool
    operator==(const AlignedVector<TYPE, GROWTH, GROWTH_MAX> &V) const
    {
        return Size() == V.Size() && UT::VectorEqual(Data(), V.Data(), Size());
    }
    inline const TYPE &operator[](const int i) const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(i >= 0 && i < m_size);
#endif
        return m_data[i];
    }
    inline TYPE &operator[](const int i)
    {
#ifdef CFG_DEBUG
        UT_ASSERT(i >= 0 && i < m_size);
#endif
        return m_data[i];
    }
    inline const TYPE *Data() const { return m_data; }
    inline TYPE *Data() { return m_data; }
    inline const TYPE &Back() const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(m_size != 0);
#endif
        return m_data[m_size - 1];
    }
    inline TYPE &Back()
    {
#ifdef CFG_DEBUG
        UT_ASSERT(m_size != 0);
#endif
        return m_data[m_size - 1];
    }

    inline const TYPE *End() const { return m_data + m_size; }
    inline TYPE *End() { return m_data + m_size; }
    inline int Size() const { return m_size; }
    inline void SaveB(FILE *fp) const
    {
        UT::SaveB<int>(m_size, fp);
        UT::SaveB<TYPE>(m_data, m_size, fp);
    }
    inline void LoadB(FILE *fp)
    {
        const int N = UT::LoadB<int>(fp);
        Resize(N);
        UT::LoadB<TYPE>(m_data, N, fp);
    }
    inline bool SaveB(const std::string fileName) const
    {
        FILE *fp = fopen(fileName.c_str(), "wb");
        if (!fp) return false;
        SaveB(fp);
        fclose(fp);
        UT::PrintSaved(fileName);
        return true;
    }
    inline bool LoadB(const std::string fileName)
    {
        FILE *fp = fopen(fileName.c_str(), "rb");
        if (!fp) return false;
        LoadB(fp);
        fclose(fp);
        UT::PrintLoaded(fileName);
        return true;
    }

    inline float MemoryMB() const
    {
        return m_own ? 0 : UT::MemoryMB<TYPE>(m_capacity);
    }

  protected:
    bool m_own;
    TYPE *m_data;
    int m_size, m_capacity;
};

#endif
