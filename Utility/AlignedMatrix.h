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

#ifndef _ALIGNED_MATRIX_H_
#define _ALIGNED_MATRIX_H_

#include "SSE.h"

template <class TYPE, int N> class AlignedMatrixN
{
  public:
    inline AlignedMatrixN()
    {
        m_own = true;
        m_data[0] = NULL;
    }
    inline ~AlignedMatrixN()
    {
        if (m_data[0] && m_own) SSE::Free<TYPE>(m_data[0]);
    }

    inline void Create()
    {
        if (m_data[0] && m_own) return;
        m_own = true;
        m_data[0] = SSE::Malloc<TYPE>(N);
        for (int r = 1; r < N; ++r) m_data[r] = m_data[r - 1] + N;
    }

    inline void Destroy()
    {
        if (m_data[0] && m_own) SSE::Free<TYPE>(m_data[0]);
        m_own = true;
        // m_data[0] = NULL;
        memset(m_data, 0, sizeof(m_data));
    }

    template <bool OWN> inline void Set(TYPE *V) { OWN ? Set_T(V) : Set_F(V); }
    inline void Set_T(TYPE *data)
    {
        Create();
        memcpy(m_data[0], data, sizeof(TYPE) * N * N);
    }
    inline void Set_F(TYPE *data)
    {
        m_own = false;
        m_data[0] = data;
        for (int r = 1; r < N; ++r) m_data[r] = m_data[r - 1] + N;
    }

    inline const TYPE *operator[](const int y) const { return m_data[y]; }
    inline TYPE *operator[](const int y) { return m_data[y]; }
  protected:
    bool m_own;
    TYPE *m_data[N];
};

template <class TYPE> class AlignedMatrixX
{
  public:
    inline AlignedMatrixX(TYPE *data, const int Nr, const int Nc,
                          const bool own = true)
    {
        m_own = own;
        m_Nr = Nr;
        m_Nc = Nc;
        m_data.resize(m_Nr);
        if (m_own) {
            const int N = m_Nr * m_Nc;
            m_data[0] = SSE::Malloc<TYPE>(N);
            memcpy(m_data[0], data, sizeof(TYPE) * N);
        } else
            m_data[0] = data;
        for (int r = 1; r < m_Nr; ++r) m_data[r] = m_data[r - 1] + m_Nc;
    }

    inline ~AlignedMatrixX()
    {
        if (!m_data.empty() && m_own) SSE::Free<TYPE>(m_data[0]);
    }

    inline const TYPE *operator[](const int y) const { return m_data[y]; }
    inline TYPE *operator[](const int y) { return m_data[y]; }
  protected:
    bool m_own;
    int m_Nr, m_Nc;
    std::vector<TYPE *> m_data;
};

#endif
