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

#ifndef _CANDIDATE_H_
#define _CANDIDATE_H_

#include "stdafx.h"

#include <vector>

template <typename TYPE> class Candidate
{
  public:
    inline Candidate() {}
    inline Candidate(const int idx, const TYPE score) { Set(idx, score); }
    inline void Set(const int idx, const TYPE score)
    {
        m_idx = idx;
        m_score = score;
    }
    inline void Get(int &idx, TYPE &score) const
    {
        idx = m_idx;
        score = m_score;
    }
    inline bool operator<(const Candidate &c) const
    {
        return m_score < c.m_score;
    }
    inline bool operator>(const Candidate &c) const
    {
        return m_score > c.m_score;
    }
    static inline void MakeZero(const int N,
                                std::vector<Candidate<TYPE>> &candidates)
    {
        candidates.resize(N);
        for (int i = 0; i < N; ++i) candidates[i].Set(i, 0);
    }
    static inline void RemoveZero(std::vector<Candidate<TYPE>> &candidates)
    {
        int i, j;
        const int N = int(candidates.size());
        for (i = j = 0; i < N; ++i) {
            if (candidates[i].m_score != 0) candidates[j++] = candidates[i];
        }
        candidates.resize(j);
    }
    static inline void SortAscending(std::vector<Candidate<TYPE>> &candidates)
    {
        std::sort(candidates.begin(), candidates.end());
    }
    static inline void SortDescending(std::vector<Candidate<TYPE>> &candidates)
    {
        std::sort(candidates.begin(), candidates.end(),
                  std::greater<Candidate<TYPE>>());
    }
    static inline void
    MarkCandidates(const std::vector<Candidate<TYPE>> &candidates,
                   std::vector<ubyte> &marks, const int N, const ubyte mark)
    {
#ifdef CFG_DEBUG
        UT_ASSERT(N < int(candidates.size()));
#endif
        for (int i = 0; i < N; ++i) marks[candidates[i].m_idx] |= mark;
    }
    // static inline void MarkCandidates(const std::vector<Candidate<TYPE> >
    // &candidates, std::vector<ubyte> &marks, const int N1, const ubyte mark1,
    // const ubyte mark2)
    //{
    //	const int N = int(candidates.size()), _N1 = std::min(N1, N);
    //	MarkCandidates(candidates, marks, _N1, mark1);
    //	for(int i = _N1; i < N; ++i)
    //		marks[candidates[i].m_idx] = mark2;
    //}
  public:
    int m_idx;
    TYPE m_score;
};

#endif
