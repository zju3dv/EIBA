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

#ifndef _M_ESTIMATOR_H_
#define _M_ESTIMATOR_H_

#include "stdafx.h"

#include "Utility.h"
#include "VectorN.h"

#define VARIANCE_HUBER 1.809025f    // 1.345^2
#define VARIANCE_TUKEY 21.95016201f // 4.6851^2
#define VARIANCE_TUKEY_INVERSE 0.0455577503f
#define VARIANCE_STUDENT_T 5.68774801f // 2.3849^2
#define VARIANCE_STUDENT_T_INVERSE 0.1758165091f

namespace ME
{
enum Function {
    FUNCTION_NONE,
    FUNCTION_HUBER,
    FUNCTION_TUKEY,
    FUNCTION_STUDENT_T_5,
    FUNCTION_CAUCHY
};

template <int FUNCTION> inline float Weight(const float r2);
template <int FUNCTION>
inline void Weight(const AlignedVector<float> &r2s, AlignedVector<float> &ws);
template <int FUNCTION> inline float Cost(const float r2);
template <int FUNCTION> inline float Cost(const AlignedVector<float> &r2s);
template <int FUNCTION> inline float Variance() { return FLT_MAX; }
// inline float Weight(const int func, const float r2)
//{
//	switch(func)
//	{
//	case FUNCTION_NONE:			return
// Weight<FUNCTION_NONE>(r2);
//	case FUNCTION_HUBER:		return Weight<FUNCTION_HUBER>(r2);
//	case FUNCTION_TUKEY:		return Weight<FUNCTION_TUKEY>(r2);
//	case FUNCTION_STUDENT_T_5:	return Weight<FUNCTION_STUDENT_T_5>(r2);
//	case FUNCTION_CAUCHY:		return Weight<FUNCTION_CAUCHY>(r2);
//	}
//	return 0.0f;
//}
// inline void Weight(const int func, const AlignedVector<float> &r2s,
// AlignedVector<float> &ws)
//{
//	switch(func)
//	{
//	case FUNCTION_NONE:			Weight<FUNCTION_NONE>(r2s, ws);
// break;
//	case FUNCTION_HUBER:		Weight<FUNCTION_HUBER>(r2s, ws);
// break;
//	case FUNCTION_TUKEY:		Weight<FUNCTION_TUKEY>(r2s, ws);
// break;
//	case FUNCTION_STUDENT_T_5:	Weight<FUNCTION_STUDENT_T_5>(r2s, ws);
// break;
//	case FUNCTION_CAUCHY:		Weight<FUNCTION_CAUCHY>(r2s, ws);
// break;
//	}
//}
// inline float Cost(const int func, const AlignedVector<float> &r2s)
//{
//	switch(func)
//	{
//	case FUNCTION_NONE:			return Cost<FUNCTION_NONE>(r2s);
//	case FUNCTION_HUBER:		return Cost<FUNCTION_HUBER>(r2s);
//	case FUNCTION_TUKEY:		return Cost<FUNCTION_TUKEY>(r2s);
//	case FUNCTION_STUDENT_T_5:	return Cost<FUNCTION_STUDENT_T_5>(r2s);
//	case FUNCTION_CAUCHY:		return Cost<FUNCTION_CAUCHY>(r2s);
//	}
//	return 0.0f;
//}
// inline float Cost(const int func, const float r2)
//{
//	switch(func)
//	{
//	case FUNCTION_NONE:			return Cost<FUNCTION_NONE>(r2);
//	case FUNCTION_HUBER:		return Cost<FUNCTION_HUBER>(r2);
//	case FUNCTION_TUKEY:		return Cost<FUNCTION_TUKEY>(r2);
//	case FUNCTION_STUDENT_T_5:	return Cost<FUNCTION_STUDENT_T_5>(r2);
//	case FUNCTION_CAUCHY:		return Cost<FUNCTION_CAUCHY>(r2);
//	}
//	return 0.0f;
//}

template <int FUNCTION> inline int Count(const AlignedVector<float> &r2s)
{
    int SN = 0;
    const float s2 = Variance<FUNCTION>();
    const int N = r2s.Size();
    for (int i = 0; i < N; ++i) {
        if (r2s[i] < s2) ++SN;
    }
    return SN;
}

template <int DATA_DIM, int PARAM_DOF>
inline float Variance(AlignedVector<float> &r2s)
{
    const int ith = r2s.Size() >> 1;
    std::nth_element(r2s.Data(), r2s.Data() + ith, r2s.Data() + r2s.Size());
    const float t =
        1.4826f * (1.0f + 5.0f / (r2s.Size() * DATA_DIM - PARAM_DOF));
    return t * t * r2s[ith];
}

template <> inline float Weight<FUNCTION_NONE>(const float r2) { return 1.0f; }
template <>
inline void Weight<FUNCTION_NONE>(const AlignedVector<float> &r2s,
                                  AlignedVector<float> &ws)
{
    const int N = r2s.Size();
    ws.Resize(N);
    for (int i = 0; i < N; ++i) ws[i] = 1.0f;
}
template <> inline float Cost<FUNCTION_NONE>(const float r2) { return r2; }
template <> inline float Cost<FUNCTION_NONE>(const AlignedVector<float> &r2s)
{
    LA::AlignedVectorXf _r2s((float *)r2s.Data(), r2s.Size(), false);
    return _r2s.Sum();
}

template <> inline float Weight<FUNCTION_HUBER>(const float r2)
{
    if (r2 > VARIANCE_HUBER)
        return sqrt(VARIANCE_HUBER / r2);
    else
        return 1.0f;
}
template <>
inline void Weight<FUNCTION_HUBER>(const AlignedVector<float> &r2s,
                                   AlignedVector<float> &ws)
{
    const int N = r2s.Size();
    ws.Resize(N);
    for (int i = 0; i < N; ++i) ws[i] = Weight<FUNCTION_HUBER>(r2s[i]);
}
template <> inline float Cost<FUNCTION_HUBER>(const float r2)
{
    // if(r2 > VARIANCE_HUBER)
    //	return sqrt(VARIANCE_HUBER * r2) - VARIANCE_HUBER * 0.5f;
    // else
    //	return r2 * 0.5f;
    if (r2 > VARIANCE_HUBER)
        return sqrt(VARIANCE_HUBER * r2) * 2.0f - VARIANCE_HUBER;
    else
        return r2;
}
template <> inline float Cost<FUNCTION_HUBER>(const AlignedVector<float> &r2s)
{
    float C = 0.0f;
    const int N = r2s.Size();
    for (int i = 0; i < N; ++i) C = Cost<FUNCTION_HUBER>(r2s[i]) + C;
    return C;
}
template <> inline float Variance<FUNCTION_HUBER>() { return VARIANCE_HUBER; }
template <> inline float Weight<FUNCTION_TUKEY>(const float r2)
{
    if (r2 > VARIANCE_TUKEY) return 0.0f;
    const float t = -(r2 * VARIANCE_TUKEY_INVERSE) + 1.0f;
    return t * t;
}
template <>
inline void Weight<FUNCTION_TUKEY>(const AlignedVector<float> &r2s,
                                   AlignedVector<float> &ws)
{
    const int N = r2s.Size(), NF = SSE_FLOAT_FLOOR(N);
    ws.Resize(N);
    const _pi__m128 one = _pi_mm_set1_ps(1.0f),
                    s2 = _pi_mm_set1_ps(VARIANCE_TUKEY),
                    ns2I = _pi_mm_set1_ps(-VARIANCE_TUKEY_INVERSE);
    const _pi__m128 *r2 = (_pi__m128 *)r2s.Data();
    _pi__m128 *w = (_pi__m128 *)ws.Data();
    for (int i = 0; i < NF; i += 4, ++r2, ++w) {
        const _pi__m128 t = _pi_mm_add_ps(_pi_mm_mul_ps(*r2, ns2I), one);
        *w = _pi_mm_mul_ps(t, t);
        *w = _pi_mm_and_ps(*w, _pi_mm_cmpge_ps(s2, *r2));
    }
    for (int i = NF; i < N; ++i) ws[i] = Weight<FUNCTION_TUKEY>(r2s[i]);
#if 0
//#ifdef CFG_DEBUG
	for(int i = 0; i < N; ++i)
		UT::AssertEqual(ws[i], Weight<FUNCTION_TUKEY>(r2s[i]));
#endif
}
template <> inline float Cost<FUNCTION_TUKEY>(const float r2)
{
    // if(r2 > VARIANCE_TUKEY)
    //	return VARIANCE_TUKEY / 6;
    // const float t = 1 - r2 / VARIANCE_TUKEY;
    // return (1.0f - t * t * t) * VARIANCE_TUKEY / 6;
    if (r2 > VARIANCE_TUKEY) return 1.0f;
    const float t = -r2 * VARIANCE_TUKEY_INVERSE + 1.0f;
    return -t * t * t + 1.0f;
}
template <> inline float Cost<FUNCTION_TUKEY>(const AlignedVector<float> &r2s)
{
    _pi__m128 c = _pi_mm_setzero_ps();
    const _pi__m128 one = _pi_mm_set1_ps(1.0f),
                    s2 = _pi_mm_set1_ps(VARIANCE_TUKEY),
                    ns2I = _pi_mm_set1_ps(-VARIANCE_TUKEY_INVERSE);
    const _pi__m128 *r2 = (_pi__m128 *)r2s.Data();
    const int N = r2s.Size(), NF = SSE_FLOAT_FLOOR(N);
    for (int i = 0; i < NF; i += 4, ++r2) {
        _pi__m128 t = _pi_mm_add_ps(_pi_mm_mul_ps(*r2, ns2I), one);
        t = _pi_mm_mul_ps(_pi_mm_mul_ps(t, t), t);
        t = _pi_mm_and_ps(t, _pi_mm_cmpge_ps(s2, *r2));
        c = _pi_mm_add_ps(t, c);
    }
    float C = NF - SSE::Sum(c);
    for (int i = NF; i < N; ++i) C = Cost<FUNCTION_TUKEY>(r2s[i]) + C;
#if 0
//#ifdef CFG_DEBUG
	float _C = 0.0f;
	for(int i = 0; i < N; ++i)
		_C = Cost<FUNCTION_TUKEY>(r2s[i]) + _C;
	UT::AssertEqual(C, _C);
#endif
    return C;
}
template <> inline float Variance<FUNCTION_TUKEY>() { return VARIANCE_TUKEY; }
template <int DOF> inline float WeightStudentT(const float r2)
{
    return DOF / (r2 * VARIANCE_STUDENT_T_INVERSE + DOF);
}
template <int DOF>
inline void WeightsStudentT(const AlignedVector<float> &r2s,
                            AlignedVector<float> &ws)
{
    const int N = r2s.Size(), NF = SSE_FLOAT_FLOOR(N);
    ws.Resize(N);
    const _pi__m128 d = _pi_mm_set1_ps(float(DOF)),
                    s2I = _pi_mm_set1_ps(VARIANCE_STUDENT_T_INVERSE);
    const _pi__m128 *r2 = (_pi__m128 *)r2s.Data();
    _pi__m128 *w = (_pi__m128 *)ws.Data();
    for (int i = 0; i < NF; i += 4, ++r2, ++w)
        *w = _pi_mm_div_ps(d, _pi_mm_add_ps(_pi_mm_mul_ps(*r2, s2I), d));
    for (int i = NF; i < N; ++i) ws[i] = WeightStudentT<DOF>(r2s[i]);
#if 0
//#ifdef CFG_DEBUG
	for(int i = 0; i < N; ++i)
		UT::AssertEqual(ws[i], WeightStudentT<DOF>(r2s[i]));
#endif
}
template <int DOF> inline float CostStudentT(const float r2)
{
    return logf(r2 / (VARIANCE_STUDENT_T * DOF) + 1.0f);
}
template <int DOF> inline float CostStudentT(const AlignedVector<float> &r2s)
{
    float C = 0.0f;
    const _pi__m128 one = _pi_mm_set1_ps(1.0f),
                    s2dI = _pi_mm_set1_ps(1.0f / (VARIANCE_STUDENT_T * DOF));
    const _pi__m128 *r2 = (_pi__m128 *)r2s.Data();
    const int N = r2s.Size(), NF = SSE_FLOAT_FLOOR(N);
    for (int i = 0; i < NF; i += 4, ++r2) {
        const _pi__m128 t = _pi_mm_add_ps(_pi_mm_mul_ps(*r2, s2dI), one);
        C = logf(t.m128_f32[0]) + logf(t.m128_f32[1]) + logf(t.m128_f32[2]) +
            logf(t.m128_f32[3]) + C;
    }
    for (int i = NF; i < N; ++i) C = logf(r2s[i] * s2dI.m128_f32[0] + 1.0f) + C;
#if 0
//#ifdef CFG_DEBUG
	float _C = 0.0f;
	for(int i = 0; i < N; ++i)
		_C = CostStudentT<DOF>(r2s[i]) + _C;
	UT::AssertEqual(C, _C);
#endif
    return C;
}
template <> inline float Weight<FUNCTION_STUDENT_T_5>(const float r2)
{
    return WeightStudentT<5>(r2);
}
template <> inline float Weight<FUNCTION_CAUCHY>(const float r2)
{
    return WeightStudentT<1>(r2);
}
template <>
inline void Weight<FUNCTION_STUDENT_T_5>(const AlignedVector<float> &r2s,
                                         AlignedVector<float> &ws)
{
    WeightsStudentT<5>(r2s, ws);
}
template <>
inline void Weight<FUNCTION_CAUCHY>(const AlignedVector<float> &r2s,
                                    AlignedVector<float> &ws)
{
    WeightsStudentT<1>(r2s, ws);
}
template <> inline float Cost<FUNCTION_STUDENT_T_5>(const float r2)
{
    return CostStudentT<5>(r2);
}
template <> inline float Cost<FUNCTION_CAUCHY>(const float r2)
{
    return CostStudentT<1>(r2);
}
template <>
inline float Cost<FUNCTION_STUDENT_T_5>(const AlignedVector<float> &r2s)
{
    return CostStudentT<5>(r2s);
}
template <> inline float Cost<FUNCTION_CAUCHY>(const AlignedVector<float> &r2s)
{
    return CostStudentT<1>(r2s);
}
}

#endif
