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

#ifndef _VECTOR_N_H_
#define _VECTOR_N_H_

#include "AlignedVector.h"
#include "Vector4.h"
#include "Vector6.h"
#include "Vector8.h"

namespace LA
{
template <int N>
class
#ifdef WIN32
    _CRT_ALIGN(16)
#endif
        AlignedVectorNf
{
  public:
    inline operator const float *() const { return (const float *)this; }
    inline operator float *() { return (float *)this; }
    inline const float &operator[](const int i) const { return m_data[i]; }
    inline float &operator[](const int i) { return m_data[i]; }
    // inline AlignedVectorNf<N> operator - (const AlignedVectorNf &b) const
    //{
    //	AlignedVectorNf<N> amb;
    //	for(int i = 0; i < N; ++i)
    //		amb[i] = m_data[i] - b[i];
    //	return amb;
    //}

    template <class VECTOR> inline void GetBlock(const int i, VECTOR &v) const;
    inline void GetBlock(const int i, AlignedVector4f &v) const
    {
        memcpy(v, &m_data[i], 16);
    }

    template <class VECTOR> inline void SetBlock(const int i, const VECTOR &v);
    inline void SetBlock(const int i, const Vector3f &v)
    {
        memcpy(&m_data[i], v, 12);
    }
    inline void SetBlock(const int i, const AlignedVector4f &v)
    {
        memcpy(&m_data[i], v, 16);
    }
    inline void SetBlock(const int i, const Vector6f &v)
    {
        memcpy(&m_data[i], v, 24);
    }
    inline void SetBlock(const int i, const AlignedVector8f &v)
    {
        memcpy(&m_data[i], v, 32);
    }

    inline void MakeZero() { memset(this, 0, sizeof(AlignedVectorNf<N>)); }
    inline void MakeMinus()
    {
        const _pi__m128 zero = _pi_mm_setzero_ps();
        const int NF = SSE_FLOAT_FLOOR(N);
        _pi__m128 *v = (_pi__m128 *)m_data;
        for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_sub_ps(zero, *v);
        for (int i = NF; i < N; ++i) m_data[i] = -m_data[i];
    }

    inline void Print(const bool e = false) const
    {
        for (int i = 0; i < N; ++i) {
            if (e)
                UT::Print("%e ", m_data[i]);
            else
                UT::Print("%f ", m_data[i]);
        }
        UT::Print("\n");
    }

    inline bool AssertEqual(const AlignedVectorNf<N> &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        if (UT::VectorAssertEqual(m_data, v.m_data, N, 0, eps)) return true;
        if (verbose) {
            UT::PrintSeparator();
            Print(verbose > 1);
            v.Print(verbose > 1);
        }
        return false;
    }

  protected:
    float m_data[N];

}
#ifndef WIN32
__attribute__((aligned(16)))
#endif
;

class AlignedVectorXf : public AlignedVector<float>
{
  public:
    inline AlignedVectorXf() = default;
    inline AlignedVectorXf(float *V, const int N, const bool own = true)
        : AlignedVector<float>(V, N, own)
    {
    }

    inline void operator=(const AlignedVectorXf &V)
    {
        AlignedVector<float>::Set(V);
    }

    inline void operator+=(const float a) { SSE::Add(Size(), Data(), a); }
    inline void operator+=(const AlignedVectorXf &V)
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
#ifdef CFG_DEBUG
        UT_ASSERT(V.Size() == N);
#endif
        _pi__m128 *v1 = (_pi__m128 *)Data();
        const _pi__m128 *v2 = (_pi__m128 *)V.Data();
        for (int i = 0; i < NF; i += 4, ++v1, ++v2)
            *v1 = _pi_mm_add_ps(*v2, *v1);
        for (int i = NF; i < N; ++i) m_data[i] = V[i] + m_data[i];
    }
    inline void operator-=(const float s) { SSE::Subtract(Size(), Data(), s); }
    inline void operator-=(const float *V)
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
        _pi__m128 *v1 = (_pi__m128 *)Data();
        const _pi__m128 *v2 = (_pi__m128 *)V;
        for (int i = 0; i < NF; i += 4, ++v1, ++v2)
            *v1 = _pi_mm_sub_ps(*v1, *v2);
        for (int i = NF; i < N; ++i) m_data[i] -= V[i];
    }
    inline void operator-=(const AlignedVectorXf &V)
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
#ifdef CFG_DEBUG
        UT_ASSERT(V.Size() == N);
#endif
        _pi__m128 *v1 = (_pi__m128 *)Data();
        const _pi__m128 *v2 = (_pi__m128 *)V.Data();
        for (int i = 0; i < NF; i += 4, ++v1, ++v2)
            *v1 = _pi_mm_sub_ps(*v1, *v2);
        for (int i = NF; i < N; ++i) m_data[i] -= V[i];
    }
    inline void operator*=(const float s) { SSE::Scale(Size(), Data(), s); }
    inline void operator*=(const _pi__m128 &s)
    {
        SSE::Scale(Size(), Data(), s);
    }
    inline void operator*=(const AlignedVectorXf &S)
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
#ifdef CFG_DEBUG
        UT_ASSERT(S.Size() == N);
#endif
        const _pi__m128 *s = (_pi__m128 *)S.Data();
        _pi__m128 *v = (_pi__m128 *)Data();
        for (int i = 0; i < NF; i += 4, ++s, ++v) *v = _pi_mm_mul_ps(*v, *s);
        for (int i = NF; i < N; ++i) m_data[i] *= S[i];
    }
    inline void operator/=(const float d)
    {
        SSE::Scale(Size(), Data(), 1.0f / d);
    }

    inline void Set(const float v) { SSE::Set(Size(), Data(), v); }
    template <bool OWN> inline void Set(float *V, const int N)
    {
        AlignedVector<float>::Set<OWN>(V, N);
    }
    inline void Set(const AlignedVectorXf &V) { AlignedVector<float>::Set(V); }
    template <typename TYPE> inline void Set(const AlignedVector<TYPE> &V)
    {
        Set<TYPE>(V.Data(), V.Size());
    }
    template <typename TYPE> inline void Set(const TYPE *V, const int N)
    {
        Resize(N);
        SSE::Convert<TYPE, float>(N, V, m_data);
    }

    static inline float SquaredLength(const float *V, const int N)
    {
        _pi__m128 s = _pi_mm_setzero_ps();
        const _pi__m128 *v = (_pi__m128 *)V;
        const int NF = SSE_FLOAT_FLOOR(N);
        for (int i = 0; i < NF; i += 4, ++v)
            s = _pi_mm_add_ps(_pi_mm_mul_ps(*v, *v), s);
        for (int i = NF; i < N; ++i)
            s.m128_f32[0] = V[i] * V[i] + s.m128_f32[0];
        return SSE::Sum(s);
    }
    inline float SquaredLength() const { return SquaredLength(Data(), Size()); }
    inline void MakeSquared()
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
        _pi__m128 *v = (_pi__m128 *)Data();
        for (int i = 0; i < NF; i += 4, ++v) *v = _pi_mm_mul_ps(*v, *v);
        for (int i = NF; i < N; ++i) m_data[i] = m_data[i] * m_data[i];
    }
    inline void GetSquared(AlignedVector<float> &V2) const
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
        V2.Resize(N);
        const _pi__m128 *v = (_pi__m128 *)Data();
        _pi__m128 *v2 = (_pi__m128 *)V2.Data();
        for (int i = 0; i < NF; i += 4, ++v, ++v2) *v2 = _pi_mm_mul_ps(*v, *v);
        for (int i = NF; i < N; ++i) V2[i] = m_data[i] * m_data[i];
    }

    inline void MakeMinus() { SSE::Minus(Size(), Data()); }
    inline void MakeInverse(const float s = 1.0f, const bool chkZero = false)
    {
        SSE::Inverse(Size(), Data(), s, chkZero);
    }

    inline float Sum() const { return Sum(Size()); }
    inline float Sum(const int N) const { return SSE::Sum(N, Data()); }
    inline float Sum(const int i1, const int i2) const
    {
        return SSE::Sum(i1, i2, Data());
    }
    inline float Average() const { return SSE::Average(Size(), Data()); }
    inline float Maximal() const { return SSE::Maximal(Size(), Data()); }
    inline float Minimal() const { return SSE::Maximal(Size(), Data()); }
    inline float Dot(const AlignedVectorXf &V) const
    {
        return Dot(V, V.Size());
    }
    inline float Dot(const AlignedVectorXf &V, const int N) const
    {
#ifdef CFG_DEBUG
        UT_ASSERT(N <= Size() && N <= V.Size());
#endif
        const int NF = SSE_FLOAT_FLOOR(N);
        _pi__m128 d = _pi_mm_setzero_ps();
        const _pi__m128 *v1 = (_pi__m128 *)Data(), *v2 = (_pi__m128 *)V.Data();
        for (int i = 0; i < NF; i += 4, ++v1, ++v2)
            d = _pi_mm_add_ps(_pi_mm_mul_ps(*v1, *v2), d);
        for (int i = NF; i < N; ++i)
            d.m128_f32[0] = m_data[i] * V.m_data[i] + d.m128_f32[0];
        return SSE::Sum(d);
    }

    inline void GetScaled(const _pi__m128 &s, AlignedVectorXf &V) const
    {
        const int N = Size(), NF = SSE_FLOAT_FLOOR(N);
        V.Resize(N);
        const _pi__m128 *v1 = (_pi__m128 *)Data();
        _pi__m128 *v2 = (_pi__m128 *)V.Data();
        for (int i = 0; i < NF; i += 4, ++v1, ++v2) *v2 = _pi_mm_mul_ps(*v1, s);
        for (int i = NF; i < N; ++i) V[i] = m_data[i] * s.m128_f32[0];
    }

    static inline void AmB(const float *A, const float *B, float *AmB,
                           const int N)
    {
        const int NF = SSE_FLOAT_FLOOR(N);
        const _pi__m128 *a = (_pi__m128 *)A, *b = (_pi__m128 *)B;
        _pi__m128 *amb = (_pi__m128 *)AmB;
        for (int i = 0; i < NF; i += 4, ++a, ++b, ++amb)
            *amb = _pi_mm_sub_ps(*a, *b);
        for (int i = NF; i < N; ++i) AmB[i] = A[i] - B[i];
    }
    static inline void AmB(const float *A, const float B, float *AmB,
                           const int N)
    {
        const int NF = SSE_FLOAT_FLOOR(N);
        const _pi__m128 *a = (_pi__m128 *)A;
        const _pi__m128 b = _pi_mm_set1_ps(B);
        _pi__m128 *amb = (_pi__m128 *)AmB;
        for (int i = 0; i < NF; i += 4, ++a, ++amb) *amb = _pi_mm_sub_ps(*a, b);
        for (int i = NF; i < N; ++i) AmB[i] = A[i] - B;
    }
    static inline void AmB(const AlignedVectorXf &A, const AlignedVectorXf &B,
                           AlignedVectorXf &AmB)
    {
        const int N = A.Size();
#ifdef CFG_DEBUG
        UT_ASSERT(B.Size() == N);
#endif
        AmB.Resize(N);
        AlignedVectorXf::AmB(A.Data(), B.Data(), AmB.Data(), N);
    }
    static inline void AmB(const AlignedVectorXf &A, const float B,
                           AlignedVectorXf &AmB)
    {
        const int N = A.Size();
        AmB.Resize(N);
        AlignedVectorXf::AmB(A.Data(), B, AmB.Data(), N);
    }
    static inline void AB(const AlignedVectorXf &A, const AlignedVectorXf &B,
                          AlignedVectorXf &AB)
    {
        const int N = A.Size(), NF = SSE_FLOAT_FLOOR(N);
#ifdef CFG_DEBUG
        UT_ASSERT(B.Size() == N);
#endif
        AB.Resize(N);
        const _pi__m128 *a = (_pi__m128 *)A.Data(), *b = (_pi__m128 *)B.Data();
        _pi__m128 *ab = (_pi__m128 *)AB.Data();
        for (int i = 0; i < NF; i += 4, ++a, ++b, ++ab)
            *ab = _pi_mm_mul_ps(*a, *b);
        for (int i = NF; i < N; ++i) AB[i] = A[i] * B[i];
    }
    static inline void AddABTo(const AlignedVectorXf &A,
                               const AlignedVectorXf &B, AlignedVectorXf &AB)
    {
        const int N = A.Size(), NF = SSE_FLOAT_FLOOR(N);
#ifdef CFG_DEBUG
        UT_ASSERT(B.Size() == N && AB.Size() == N);
#endif
        const _pi__m128 *a = (_pi__m128 *)A.Data(), *b = (_pi__m128 *)B.Data();
        _pi__m128 *ab = (_pi__m128 *)AB.Data();
        for (int i = 0; i < NF; i += 4, ++a, ++b, ++ab)
            *ab = _pi_mm_add_ps(_pi_mm_mul_ps(*a, *b), *ab);
        for (int i = NF; i < N; ++i) AB[i] = A[i] * B[i] + AB[i];
    }
    static inline float SquaredDistance(const float *A, const float *B,
                                        const int N)
    {
        const int NF = SSE_FLOAT_FLOOR(N);
        const _pi__m128 *a = (_pi__m128 *)A, *b = (_pi__m128 *)B;
        _pi__m128 d, s = _pi_mm_setzero_ps();
        for (int i = 0; i < NF; i += 4, ++a, ++b) {
            d = _pi_mm_sub_ps(*a, *b);
            s = _pi_mm_add_ps(_pi_mm_mul_ps(d, d), s);
        }
        for (int i = NF; i < N; ++i) {
            d.m128_f32[0] = A[i] - B[i];
            s.m128_f32[0] = d.m128_f32[0] * d.m128_f32[0] + s.m128_f32[0];
        }
        return SSE::Sum(s);
    }
};
}

#ifdef CFG_DEBUG_EIGEN
template <int N> class EigenVectorNf : public Eigen::Matrix<float, N, 1>
{
  public:
    inline EigenVectorNf() = default;
    inline EigenVectorNf(const Eigen::Matrix<float, N, 1> &e_v)
        : Eigen::Matrix<float, N, 1>(e_v)
    {
    }
    inline EigenVectorNf(const LA::AlignedVectorNf<N> &v)
        : Eigen::Matrix<float, N, 1>()
    {
        Eigen::Matrix<float, N, 1> &e_v = *this;
        for (int i = 0; i < N; ++i) e_v(i, 0) = v[i];
    }
    inline void operator=(const Eigen::Matrix<float, N, 1> &e_v)
    {
        *((Eigen::Matrix<float, N, 1> *)this) = e_v;
    }
    inline LA::AlignedVectorNf<N> GetVectorNf() const
    {
        LA::AlignedVectorNf<N> v;
        const Eigen::Matrix<float, N, 1> &e_v = *this;
        for (int i = 0; i < N; ++i) v[i] = e_v(i, 0);
        return v;
    }
    inline void Print(const bool e = false) const { GetVectorNf().Print(e); }
    inline bool AssertEqual(const LA::AlignedVectorNf<N> &v,
                            const int verbose = 1, const float eps = 0.0f) const
    {
        return GetVectorNf().AssertEqual(v, verbose, eps);
    }
    inline bool AssertEqual(const EigenVectorNf &e_v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return AssertEqual(e_v.GetVectorNf(), verbose, eps);
    }
    inline bool AssertEqual(const LA::Vector6f &v, const int verbose = 1,
                            const float eps = 0.0f) const
    {
        return LA::Vector6f(GetVectorNf()).AssertEqual(v, verbose, eps);
    }
};
#endif
#endif
