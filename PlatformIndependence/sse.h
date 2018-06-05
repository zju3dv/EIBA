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

#ifndef PLATFORM_INDEPENDENCE_SSE_H_
#define PLATFORM_INDEPENDENCE_SSE_H_
// clang-format off
// Wanna preserve the order
#include <mmintrin.h>  // MMX
#include <xmmintrin.h> //SSE
#include <emmintrin.h> //SSE2

// FIXME: following works, but not know why
#ifndef WIN32
#include <pmmintrin.h> //SSE3
#include <tmmintrin.h> //SSSE3
#include <smmintrin.h> //SSE4.1
#include <nmmintrin.h> //SSE4.2
#include <ammintrin.h> //SSE4A
#endif

// clang-format on

#ifdef WIN32
#define _pi__m64 __m64
#define _pi__m128 __m128
#define _pi__m128i __m128i
#define _pi__m128d __m128d
#else
// namespace pi
// {
// union __m64 {
//     uint64 m64_u64;
//     float m64_f32[2];
//     int8 m64_i8[8];
//     int16 m64_i16[4];
//     int32 m64_i32[2];
//     int64 m64_i64;
//     uint8 m64_u8[8];
//     uint16 m64_u16[4];
//     uint32 m64_u32[2];
//     ::__m64 native_obj;
// } __attribute__((aligned(8), packed));
// }
// /* It seems make pe::__m64 implicitly convert to ::__m64 a bad idea, reasons:
//  * (1) the compiler do the implicit conversion at most once, e.g following
//  not
//  *     work:
//  *     class Foo
//  *     {
//  *       public:
//  *         Foo(::__m64);
//  *     };
//  *     void g(Foo) {}
//  *     pe::__m64 x;
//  *     g(x); // error, need 2 implicit conversion to work
//  * (2) it may fails on compound types, e.g:
//  *     void f(::__m64* );
//  *     pe::__m64 x;
//  *     f(&x);
//  *
//  * What' more, should make __m64 takes trial constructor(i.e do not provide
//  * default one), otherwise it can not used in anonymous unions, e.g:
//  *     struct Foo{
//  *     union{
//  *        int foo;
//  *        __m64 bar;
//  *     }; // gcc complains if __m64 has constructor
//  * As a consequence, we can not supply any constructors at all.
//  *
//  * What we need to do is define pe::__m64 to be able to mimic ::_m64
//  * completely, i.e define all necessary intrinsic functions on pe::__m64
//  *
//  * The same reason holds for other types.
//  */

/* Problems with above approach :
 * (1) conflict with global definition, we can solve it by using #define, e.g
 *     void _pi_mm_blah(...) { _mm_blah(...) }
 *     #define _mm_blah _pi_mm_blah
 * (2) some intrinsic functions are implemented as macro, use clang to parse is
 * not easy
 */

/*
 * It seems using macro gives an easy and portable solution!
 */
namespace
{
static_assert(sizeof(char) == 1, "sizeof(char) != 1");
static_assert(sizeof(short) == 2, "sizeof(short) != 2");
static_assert(sizeof(int) == 4, "sizeof(int) != 4");
static_assert(sizeof(long) == 8, "sizeof(long) != 8");

static_assert(sizeof(float) == 4, "sizeof(float) != 4");
static_assert(sizeof(double) == 8, "sizeof(double) != 8");

typedef signed char int8;
typedef unsigned char uint8;
typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;
typedef long int64;
typedef unsigned long uint64;
}

union _pi__m64 {
    uint64 m64_u64;
    float m64_f32[2];
    int8 m64_i8[8];
    int16 m64_i16[4];
    int32 m64_i32[2];
    int64 m64_i64;
    uint8 m64_u8[8];
    uint16 m64_u16[4];
    uint32 m64_u32[2];

    __m64 native_obj;
} __attribute__((aligned(8), packed));

union _pi__m128 {
    float m128_f32[4];
    uint64 m128_u64[2];
    int8 m128_i8[16];
    int16 m128_i16[8];
    int32 m128_i32[4];
    int64 m128_i64[2];
    uint8 m128_u8[16];
    uint16 m128_u16[8];
    uint32 m128_u32[4];

    __m128 native_obj;
} __attribute__((aligned(16), packed));

union _pi__m128i {
    int8 m128i_i8[16];
    int16 m128i_i16[8];
    int32 m128i_i32[4];
    int64 m128i_i64[2];
    uint8 m128i_u8[16];
    uint16 m128i_u16[8];
    uint32 m128i_u32[4];
    uint64 m128i_u64[2];

    __m128i native_obj;
} __attribute__((aligned(16), packed));

union _pi__m128d {
    double m128d_f64[2];
    __m128d native_obj;
} __attribute__((aligned(16), packed));

inline _pi__m64 from_native_obj(const __m64 &x)
{
    _pi__m64 y;
    y.native_obj = x;
    return y;
}

inline _pi__m128 from_native_obj(const __m128 &x)
{
    _pi__m128 y;
    y.native_obj = x;
    return y;
}

inline _pi__m128i from_native_obj(const __m128i &x)
{
    _pi__m128i y;
    y.native_obj = x;
    return y;
}

inline _pi__m128d from_native_obj(const __m128d &x)
{
    _pi__m128d y;
    y.native_obj = x;
    return y;
}
#endif // WIN32
#include "sse_impl.h"
#endif
