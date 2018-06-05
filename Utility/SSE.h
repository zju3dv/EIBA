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

#include "PlatformIndependence/def_missing.h"
#include "PlatformIndependence/sse.h"

#ifndef _SSE_H_
#define _SSE_H_

#include "stdafx.h"

#include <algorithm>

#include <emmintrin.h>
#include <limits.h>
#include <string.h> // memcpy
#include <xmmintrin.h>

// BYTE-ALIGNMENT for data allocation (16 required for SSE, 32 required for AVX)
// PREVIOUS version uses only SSE. The new version will include AVX.
// SO the alignment is increased from 16 to 32
#define SSE_ALIGNMENT 32
#define SSE_FLOAT_FLOOR(N) ((N) - ((N)&3))
#define SSE_FLOAT_CEIL(N) (((N) + 3) & (~3))
#define SSE_SHORT_FLOOR(N) ((N) - ((N)&7))
#define SSE_SHORT_CEIL(N) (((N) + 7) & (~7))
#define SSE_BYTE_FLOOR(N) ((N) - ((N)&15))
#define SSE_BYTE_CEIL(N) (((N) + 15) & (~15))

namespace SSE
{
template <class TYPE> inline TYPE *Malloc(const int N)
{
    return (TYPE *)_aligned_malloc(N * sizeof(TYPE), SSE_ALIGNMENT);
}
template <class TYPE> inline void Free(TYPE *p) { _aligned_free(p); }
inline float Sum(const _pi__m128 &v)
{
    return v.m128_f32[0] + v.m128_f32[1] + v.m128_f32[2] + v.m128_f32[3];
}
inline float Sum012(const _pi__m128 &v)
{
    return v.m128_f32[0] + v.m128_f32[1] + v.m128_f32[2];
}
inline int SumU16(const _pi__m128i &v)
{
    return int(v.m128i_u16[0]) + int(v.m128i_u16[1]) + int(v.m128i_u16[2]) +
           int(v.m128i_u16[3]) + int(v.m128i_u16[4]) + int(v.m128i_u16[5]) +
           int(v.m128i_u16[6]) + int(v.m128i_u16[7]);
}
inline short SumI16(const _pi__m128i &v)
{
    return int(v.m128i_i16[0]) + int(v.m128i_i16[1]) + int(v.m128i_i16[2]) +
           int(v.m128i_i16[3]) + int(v.m128i_i16[4]) + int(v.m128i_i16[5]) +
           int(v.m128i_i16[6]) + int(v.m128i_i16[7]);
}
inline int SumU32(const _pi__m128i &v)
{
    return v.m128i_u32[0] + v.m128i_u32[1] + v.m128i_u32[2] + v.m128i_u32[3];
}
inline int SumI32(const _pi__m128i &v)
{
    return v.m128i_i32[0] + v.m128i_i32[1] + v.m128i_i32[2] + v.m128i_i32[3];
}

inline float Dot012(const _pi__m128 &v1, const _pi__m128 &v2)
{
    return Sum012(_pi_mm_mul_ps(v1, v2));
}
inline void Normalize012(_pi__m128 &v)
{
    v = _pi_mm_mul_ps(
        _pi_mm_set1_ps(1.0f / sqrt(SSE::Sum012(_pi_mm_mul_ps(v, v)))), v);
}
inline void Cross012(const _pi__m128 &v1, const _pi__m128 &v2, _pi__m128 &v1xv2)
{
    v1xv2.m128_f32[0] =
        v1.m128_f32[1] * v2.m128_f32[2] - v1.m128_f32[2] * v2.m128_f32[1];
    v1xv2.m128_f32[1] =
        v1.m128_f32[2] * v2.m128_f32[0] - v1.m128_f32[0] * v2.m128_f32[2];
    v1xv2.m128_f32[2] =
        v1.m128_f32[0] * v2.m128_f32[1] - v1.m128_f32[1] * v2.m128_f32[0];
}

inline float Maximal(const _pi__m128 &v)
{
    return std::max(std::max(v.m128_f32[0], v.m128_f32[1]),
                    std::max(v.m128_f32[2], v.m128_f32[3]));
}
inline ushort MaximalU16(const _pi__m128i &v)
{
    return std::max(std::max(std::max(v.m128i_u16[0], v.m128i_u16[1]),
                             std::max(v.m128i_u16[2], v.m128i_u16[3])),
                    std::max(std::max(v.m128i_u16[4], v.m128i_u16[5]),
                             std::max(v.m128i_u16[6], v.m128i_u16[7])));
}
inline float Minimal(const _pi__m128 &v)
{
    return std::min(std::min(v.m128_f32[0], v.m128_f32[1]),
                    std::min(v.m128_f32[2], v.m128_f32[3]));
}
inline ushort MinimalU16(const _pi__m128i &v)
{
    return std::min(std::min(std::min(v.m128i_u16[0], v.m128i_u16[1]),
                             std::min(v.m128i_u16[2], v.m128i_u16[3])),
                    std::min(std::min(v.m128i_u16[4], v.m128i_u16[5]),
                             std::min(v.m128i_u16[6], v.m128i_u16[7])));
}

inline ushort OrU16(const _pi__m128i &v)
{
    return v.m128i_u16[0] | v.m128i_i16[1] | v.m128i_i16[2] | v.m128i_i16[3] |
           v.m128i_i16[4] | v.m128i_i16[5] | v.m128i_i16[6] | v.m128i_i16[7];
}
inline int OrI32(const _pi__m128 &v)
{
    return v.m128_i32[0] | v.m128_i32[1] | v.m128_i32[2] | v.m128_i32[3];
}

template <typename TYPE> inline void Set(const int N, TYPE *Vs, const TYPE v);
template <> inline void Set<float>(const int N, float *V, const float v)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    const _pi__m128 v1 = _pi_mm_set1_ps(v);
    _pi__m128 *v2 = (_pi__m128 *)V;
    for (int i = 0; i < NF; i += 4, ++v2) *v2 = v1;
    for (int i = NF; i < N; ++i) V[i] = v;
}

template <typename TYPE_SRC, typename TYPE_DST>
inline void Convert(const int N, const TYPE_SRC *Vs, TYPE_DST *Vd);
template <>
inline void Convert<ubyte, float>(const int N, const ubyte *Vs, float *Vd)
{
    _pi__m128i t;
    const _pi__m128i *vs = (_pi__m128i *)Vs;
    _pi__m128 *vd = (_pi__m128 *)Vd;
    const _pi__m128i zero = _pi_mm_setzero_si128();
    const int NF = SSE_BYTE_FLOOR(N);
    for (int i = 0; i < NF; i += 16, ++vs, vd += 4) {
        t = _pi_mm_unpacklo_epi8(*vs, zero);
        vd[0] = _pi_mm_cvtepi32_ps(_pi_mm_unpacklo_epi16(t, zero));
        vd[1] = _pi_mm_cvtepi32_ps(_pi_mm_unpackhi_epi16(t, zero));
        t = _pi_mm_unpackhi_epi8(*vs, zero);
        vd[2] = _pi_mm_cvtepi32_ps(_pi_mm_unpacklo_epi16(t, zero));
        vd[3] = _pi_mm_cvtepi32_ps(_pi_mm_unpackhi_epi16(t, zero));
    }
    for (int i = NF; i < N; ++i) Vd[i] = float(Vs[i]);
}
template <>
inline void Convert<short, float>(const int N, const short *Vs, float *Vd)
{
    const _pi__m128i *vs = (_pi__m128i *)Vs;
    _pi__m128 *vd = (_pi__m128 *)Vd;
    const _pi__m128i zero = _pi_mm_setzero_si128();
    const int NF = SSE_SHORT_FLOOR(N);
    for (int i = 0; i < NF; i += 8, ++vs, vd += 2) {
        vd[0] = _pi_mm_cvtepi32_ps(
            _pi_mm_srai_epi32(_pi_mm_unpacklo_epi16(zero, *vs), 16));
        vd[1] = _pi_mm_cvtepi32_ps(
            _pi_mm_srai_epi32(_pi_mm_unpackhi_epi16(zero, *vs), 16));
    }
    for (int i = NF; i < N; ++i) Vd[i] = float(Vs[i]);
}
template <>
inline void Convert<ushort, float>(const int N, const ushort *Vs, float *Vd)
{
    const _pi__m128i *vs = (_pi__m128i *)Vs;
    _pi__m128 *vd = (_pi__m128 *)Vd;
    const _pi__m128i zero = _pi_mm_setzero_si128();
    const int NF = SSE_SHORT_FLOOR(N);
    for (int i = 0; i < NF; i += 8, ++vs, vd += 2) {
        vd[0] = _pi_mm_cvtepi32_ps(_pi_mm_unpacklo_epi16(*vs, zero));
        vd[1] = _pi_mm_cvtepi32_ps(_pi_mm_unpackhi_epi16(*vs, zero));
    }
    for (int i = NF; i < N; ++i) Vd[i] = float(Vs[i]);
}
template <>
inline void Convert<int, float>(const int N, const int *Vs, float *Vd)
{
    const _pi__m128i *vs = (_pi__m128i *)Vs;
    _pi__m128 *vd = (_pi__m128 *)Vd;
    const int NF = SSE_FLOAT_FLOOR(N);
    for (int i = 0; i < NF; i += 4, ++vs, ++vd) *vd = _pi_mm_cvtepi32_ps(*vs);
    for (int i = NF; i < N; ++i) Vd[i] = float(Vs[i]);
}
template <>
inline void Convert<float, float>(const int N, const float *Vs, float *Vd)
{
    memcpy(Vd, Vs, sizeof(float) * N);
}
template <>
inline void Convert<float, ushort>(const int N, const float *Vs, ushort *Vd)
{
    const _pi__m128 *vs = (_pi__m128 *)Vs;
    _pi__m128i *vd = (_pi__m128i *)Vd;
    const int NF = SSE_SHORT_FLOOR(N);
    for (int i = 0; i < NF; i += 8, vs += 2, ++vd)
        *vd = _pi_mm_packs_epi32(_pi_mm_cvtps_epi32(vs[0]),
                                 _pi_mm_cvtps_epi32(vs[1]));
    for (int i = NF; i < N; ++i) Vd[i] = short(Vs[i]);
}

template <typename TYPE> inline void Add(const int N, TYPE *V, const TYPE a);
template <> inline void Add<short>(const int N, short *V, const short a)
{
    const int NF = SSE_SHORT_FLOOR(N);
    const _pi__m128i _a = _pi_mm_set1_epi16(a);
    _pi__m128i *v = (_pi__m128i *)V;
    for (int i = 0; i < NF; i += 8, ++v) *v = _pi_mm_add_epi16(_a, *v);
    for (int i = NF; i < N; ++i) V[i] += a;
}
template <> inline void Add<float>(const int N, float *V, const float a)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    const _pi__m128 _a = _pi_mm_set1_ps(a);
    _pi__m128 *v = (_pi__m128 *)V;
    for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_add_ps(_a, *v);
    for (int i = NF; i < N; ++i) V[i] = a + V[i];
}

template <typename TYPE>
inline void Subtract(const int N, TYPE *V, const TYPE s);
template <> inline void Subtract<float>(const int N, float *V, const float s)
{
    Add(N, V, -s);
}

template <typename TYPE_DATA, typename TYPE_SCALAR>
inline void Scale(const int N, TYPE_DATA *V, const TYPE_SCALAR s);
template <>
inline void Scale<ushort, ushort>(const int N, ushort *V, const ushort s)
{
    if (s == 1) return;
    const _pi__m128i _s = _pi_mm_set1_epi16(s);
    _pi__m128i *v = (_pi__m128i *)V;
    const int NF = SSE_SHORT_FLOOR(N);
    for (int i = 0; i < NF; i += 8, ++v) *v = _pi_mm_mullo_epi16(*v, _s);
    for (int i = NF; i < N; ++i) V[i] *= s;
}
template <>
inline void Scale<float, float>(const int N, float *V, const float s)
{
    if (s == 1.0f) return;
    const _pi__m128 _s = _pi_mm_set1_ps(s);
    _pi__m128 *v = (_pi__m128 *)V;
    const int NF = SSE_FLOAT_FLOOR(N);
    for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_mul_ps(*v, _s);
    for (int i = NF; i < N; ++i) V[i] *= s;
}
template <>
inline void Scale<float, _pi__m128>(const int N, float *V, const _pi__m128 s)
{
    if (s.m128_f32[0] == 1.0f) return;
    _pi__m128 *v = (_pi__m128 *)V;
    const int NF = SSE_FLOAT_FLOOR(N);
    for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_mul_ps(*v, s);
    for (int i = NF; i < N; ++i) V[i] *= s.m128_f32[0];
}
template <typename TYPE_SRC, typename TYPE_DST>
inline void Scale(const int N, const TYPE_SRC *Vs, TYPE_DST *Vd,
                  const TYPE_DST s)
{
    Convert<TYPE_SRC, TYPE_DST>(N, Vs, Vd);
    Scale<TYPE_DST>(N, Vd, s);
}

inline void Minus(const int N, float *V)
{
    const _pi__m128 zero = _pi_mm_setzero_ps();
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 *v = (_pi__m128 *)V;
    for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_sub_ps(zero, *v);
    for (int i = NF; i < N; ++i) V[i] = -V[i];
}
inline void Inverse(const int N, float *V, const float s = 1.0f,
                    const bool chkZero = false)
{
    const _pi__m128 _s = _pi_mm_set1_ps(s);
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 *v = (_pi__m128 *)V;
    if (chkZero) {
        const _pi__m128 zero = _pi_mm_setzero_ps();
        for (int i = 0; i < NF; i += 4, ++v)
            *v = _pi_mm_and_ps(_pi_mm_div_ps(_s, *v),
                               _pi_mm_cmpneq_ps(*v, zero));
        for (int i = NF; i < N; ++i) V[i] = V[i] == 0.0f ? 0.0f : s / V[i];
    } else {
        for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_div_ps(_s, *v);
        for (int i = NF; i < N; ++i) V[i] = s / V[i];
    }
}

template <typename TYPE> inline float Sum(const int N, const TYPE *V);
template <> inline float Sum<ubyte>(const int N, const ubyte *V)
{
    _pi__m128i t;
    const _pi__m128i zero = _pi_mm_setzero_si128();
    const int NF = SSE_BYTE_FLOOR(N);
    _pi__m128i S = zero;
    const _pi__m128i *v = (_pi__m128i *)V;
    for (int i = 0; i < NF; i += 16, ++v) {
        t = _pi_mm_add_epi16(_pi_mm_unpacklo_epi8(*v, zero),
                             _pi_mm_unpackhi_epi8(*v, zero));
        S = _pi_mm_add_epi32(_pi_mm_add_epi32(_pi_mm_unpacklo_epi16(t, zero),
                                              _pi_mm_unpackhi_epi16(t, zero)),
                             S);
    }
    for (int i = NF; i < N; ++i) S.m128i_u32[0] += V[i];
    return float(SumU32(S));
}
template <> inline float Sum<float>(const int N, const float *V)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 S = _pi_mm_setzero_ps();
    const _pi__m128 *v = (_pi__m128 *)V;
    for (int i = 0; i < NF; i += 4, ++v) S = _pi_mm_add_ps(*v, S);
    for (int i = NF; i < N; ++i) S.m128_f32[0] = V[i] + S.m128_f32[0];
    return Sum(S);
}
template <typename TYPE>
inline TYPE Sum(const int i1, const int i2, const TYPE *V);
template <> inline float Sum<float>(const int i1, const int i2, const float *V)
{
    if (i2 - i1 < 4) {
        float s = 0.0f;
        for (int i = i1; i < i2; ++i) s = V[i] + s;
        return s;
    }
    const int i1c = SSE_FLOAT_CEIL(i1);
    float S = Sum(i2 - i1c, V + i1c);
    for (int i = i1; i < i1c; ++i) S = V[i] + S;
    return S;
}
template <typename TYPE> inline float Average(const int N, const TYPE *V)
{
    return Sum(N, V) / N;
}

template <typename TYPE> inline TYPE Maximal(const int N, const TYPE *V);
template <> inline ushort Maximal<ushort>(const int N, const ushort *V)
{
    const int NF = SSE_SHORT_FLOOR(N);
    _pi__m128i m = _pi_mm_set1_epi16(0);
    const _pi__m128i *v = (_pi__m128i *)V;
    for (int i = 0; i < NF; i += 8, ++v) m = _pi_mm_max_epu16(*v, m);
    for (int i = NF; i < N; ++i)
        m.m128i_u16[0] = std::max(V[i], m.m128i_u16[0]);
    return MaximalU16(m);
}
template <> inline float Maximal<float>(const int N, const float *V)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 m = _pi_mm_set1_ps(-FLT_MAX);
    const _pi__m128 *v = (_pi__m128 *)V;
    for (int i = 0; i < NF; i += 4, ++v) m = _pi_mm_max_ps(*v, m);
    for (int i = NF; i < N; ++i) m.m128_f32[0] = std::max(V[i], m.m128_f32[0]);
    return Maximal(m);
}

template <typename TYPE> inline TYPE Minimal(const int N, const TYPE *V);
template <> inline ushort Minimal<ushort>(const int N, const ushort *V)
{
    const int NF = SSE_SHORT_FLOOR(N);
    _pi__m128i m = _pi_mm_set1_epi16(USHRT_MAX);
    const _pi__m128i *v = (_pi__m128i *)V;
    for (int i = 0; i < NF; i += 8, ++v) m = _pi_mm_min_epu16(*v, m);
    for (int i = NF; i < N; ++i)
        m.m128i_u16[0] = std::min(V[i], m.m128i_u16[0]);
    return MinimalU16(m);
}
template <> inline float Minimal<float>(const int N, const float *V)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 m = _pi_mm_set1_ps(FLT_MAX);
    const _pi__m128 *v = (_pi__m128 *)V;
    for (int i = 0; i < NF; i += 4, ++v) m = _pi_mm_min_ps(*v, m);
    for (int i = NF; i < N; ++i) m.m128_f32[0] = std::min(V[i], m.m128_f32[0]);
    return Minimal(m);
}

template <typename TYPE>
inline bool ExistEqual(const int N, const TYPE *V, const TYPE v);
template <>
inline bool ExistEqual<ushort>(const int N, const ushort *V, const ushort v)
{
    const int NF = SSE_SHORT_FLOOR(N);
    _pi__m128i m = _pi_mm_setzero_si128();
    const _pi__m128i *v1 = (_pi__m128i *)V;
    const _pi__m128i v2 = _pi_mm_set1_epi16(v);
    for (int i = 0; i < NF; i += 8, ++v1)
        m = _pi_mm_or_si128(_pi_mm_cmpeq_epi16(*v1, v2), m);
    for (int i = NF; i < N; ++i) {
        if (V[i] == v) m.m128i_u16[0] = USHRT_MAX;
    }
    return OrU16(m) != 0;
}

template <typename TYPE>
inline bool ExistGreater(const int N, const TYPE *V, const TYPE v);
template <>
inline bool ExistGreater<ushort>(const int N, const ushort *V, const ushort v)
{
    const int NF = SSE_SHORT_FLOOR(N);
    _pi__m128i m = _pi_mm_setzero_si128();
    const _pi__m128i *v1 = (_pi__m128i *)V;
    const _pi__m128i v2 = _pi_mm_set1_epi16(v);
    for (int i = 0; i < NF; i += 8, ++v1)
        m = _pi_mm_or_si128(_pi_mm_cmpgt_epi16(*v1, v2), m);
    for (int i = NF; i < N; ++i) {
        if (V[i] > v) m.m128i_u16[0] = USHRT_MAX;
    }
    return OrU16(m) != 0;
}
template <>
inline bool ExistGreater<float>(const int N, const float *V, const float v)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 m = _pi_mm_setzero_ps();
    const _pi__m128 *v1 = (_pi__m128 *)V;
    const _pi__m128 v2 = _pi_mm_set1_ps(v);
    for (int i = 0; i < NF; i += 4, ++v1)
        m = _pi_mm_or_ps(_pi_mm_cmpgt_ps(*v1, v2), m);
    for (int i = NF; i < N; ++i) {
        if (V[i] > v) m.m128_i32[0] = 0xffffffff;
    }
    return OrI32(m) != 0;
}

template <typename TYPE>
inline bool ExistLess(const int N, const TYPE *V, const TYPE v);
template <>
inline bool ExistLess<ushort>(const int N, const ushort *V, const ushort v)
{
    const int NF = SSE_SHORT_FLOOR(N);
    _pi__m128i m = _pi_mm_setzero_si128();
    const _pi__m128i *v1 = (_pi__m128i *)V;
    const _pi__m128i v2 = _pi_mm_set1_epi16(v);
    for (int i = 0; i < NF; i += 8, ++v1)
        m = _pi_mm_or_si128(_pi_mm_cmplt_epi16(*v1, v2), m);
    for (int i = NF; i < N; ++i) {
        if (V[i] < v) m.m128i_u16[0] = USHRT_MAX;
    }
    return OrU16(m) != 0;
}
template <>
inline bool ExistLess<float>(const int N, const float *V, const float v)
{
    const int NF = SSE_FLOAT_FLOOR(N);
    _pi__m128 m = _pi_mm_setzero_ps();
    const _pi__m128 *v1 = (_pi__m128 *)V;
    const _pi__m128 v2 = _pi_mm_set1_ps(v);
    for (int i = 0; i < NF; i += 4, ++v1)
        m = _pi_mm_or_ps(_pi_mm_cmplt_ps(*v1, v2), m);
    for (int i = NF; i < N; ++i) {
        if (V[i] < v) m.m128_i32[0] = 0xffffffff;
    }
    return OrI32(m) != 0;
}
}

#endif
