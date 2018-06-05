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

#ifndef _LINEAR_SYSTEM_H_
#define _LINEAR_SYSTEM_H_

#include <string.h>

#include "Utility.h"

namespace LA
{
namespace LS
{
template <typename TYPE, int N> inline bool DecomposeLDL(TYPE **A)
{
    TYPE *t = A[N - 1];
    const TYPE eps = UT::Epsilon<TYPE>();
    for (int k = 0; k < N; ++k) {
        TYPE *ak = A[k];
        const TYPE dk = ak[k];
        if (UT::NotANumber(dk) || fabs(dk) <= eps) return false;
        memcpy(t, ak + k + 1, sizeof(TYPE) * (N - k - 1));
        const TYPE dkI = 1 / dk;
        ak[k] = dkI;
        for (int j = k + 1; j < N; ++j) ak[j] *= dkI;
        for (int i = k + 1; i < N; ++i) {
            TYPE *ai = A[i];
            const TYPE aki = t[i - k - 1];
            for (int j = i; j < N; ++j) ai[j] -= aki * ak[j];
        }
    }
    return true;
}

template <typename TYPE, int N> inline bool SolveLDL(TYPE **A, TYPE *b)
{
    if (!DecomposeLDL<TYPE, N>(A)) return false;
    for (int i = 0; i < N; ++i) {
        const TYPE *ai = A[i];
        const TYPE bi = b[i];
        for (int j = i + 1; j < N; ++j) b[j] -= ai[j] * bi;
    }
    for (int i = N - 1; i >= 0; --i) {
        const TYPE *ai = A[i];
        TYPE bi = ai[i] * b[i];
        for (int j = i + 1; j < N; ++j) bi -= ai[j] * b[j];
        b[i] = bi;
    }
    return true;
}

template <typename TYPE, int N>
inline bool SolveLDL(TYPE **A, const bool decomposed = false)
{
    if (!decomposed && !DecomposeLDL<TYPE, N>(A)) return false;
    for (int i = 0; i < N; ++i) {
        const TYPE *ai = A[i];
        const TYPE bi = ai[N];
        for (int j = i + 1; j < N; ++j) A[j][N] -= ai[j] * bi;
    }
    for (int i = N - 1; i >= 0; --i) {
        const TYPE *ai = A[i];
        TYPE bi = ai[i] * ai[N];
        for (int j = i + 1; j < N; ++j) bi -= ai[j] * A[j][N];
        A[i][N] = bi;
    }
    return true;
}

template <typename TYPE, int N>
inline bool InverseLDL(TYPE **A, const bool decomposed = false)
{
    if (!decomposed && !DecomposeLDL<TYPE, N>(A)) return false;
    for (int k = 0; k < N; ++k) {
        const TYPE *ak = A[k];
        for (int j = k + 1; j < N; ++j) A[j][k] = -ak[j];
        for (int i = k + 1; i < N; ++i) {
            const TYPE *ai = A[i];
            const TYPE bi = ai[k];
            for (int j = i + 1; j < N; ++j) A[j][k] -= ai[j] * bi;
        }
        for (int i = N - 1; i > k; --i) {
            const TYPE *ai = A[i];
            TYPE bi = ai[i] * ai[k];
            for (int j = i + 1; j < N; ++j) bi -= ai[j] * A[j][k];
            A[i][k] = bi;
        }
        TYPE bk = ak[k];
        for (int j = k + 1; j < N; ++j) bk -= ak[j] * A[j][k];
        A[k][k] = bk;
    }
    for (int i = 0; i < N; ++i)
        for (int j = i + 1; j < N; ++j) A[i][j] = A[j][i];
    return true;
}
}
}

#ifdef CFG_DEBUG_EIGEN
#include <Eigen/Eigen>
template <int N, int Nm>
inline void EigenMarginalize(Eigen::Matrix<float, N, N> &e_A,
                             Eigen::Matrix<float, N, 1> &e_b)
{
    const Eigen::Matrix<float, N - Nm, Nm> e_M =
        e_A.block<N - Nm, Nm>(Nm, 0) * e_A.block<Nm, Nm>(0, 0).inverse();
    e_A.block<N - Nm, N>(Nm, 0) -= e_M * e_A.block<Nm, N>(0, 0);
    e_b.block<N - Nm, 1>(Nm, 0) -= e_M * e_b.block<Nm, 1>(0, 0);
}
#endif

#endif
