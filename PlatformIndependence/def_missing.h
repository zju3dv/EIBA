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

#ifndef PLATFORM_INDEPENDENCE_DEF_MISSING_H_
#define PLATFORM_INDEPENDENCE_DEF_MISSING_H_

#ifndef WIN32
#include <cmath>
#include <limits.h> // UCHAR_MAX...
#ifndef __APPLE__
    #include <malloc.h>
#endif
#include <math.h>
#include <unistd.h>

#ifndef _access
#define _access access
#endif

#ifndef _aligned_malloc
#   ifdef __APPLE__
#       define _aligned_malloc(size, align) malloc(size)
#   else
#       define _aligned_malloc(size, align) memalign(align, size)
#   endif
#endif

#ifndef _aligned_free
#define _aligned_free free
#endif

#ifndef _finite
#define _finite std::isfinite
#endif

#ifndef _isnan
// #define _isnan isnan // not works
inline int _isnan(double x) { return std::isnan(x); }
#endif

#ifndef _isnanf
#define _isnanf std::isnan
#endif

#ifndef Sleep
#define Sleep sleep
#endif

// #ifndef FLT_EPSILON
// #define FLT_EPSILON 1e-6f
// #endif

// #ifndef DBL_EPSILON
// #define DBL_EPSILON 1e-6
// #endif

// // copy from stdafx.h

// typedef unsigned char ubyte;
// typedef unsigned short ushort;
// typedef unsigned int uint;
// typedef unsigned long long ullong;

// #ifdef max
// #undef max
// #endif
// #ifdef min
// #undef min
// #endif

// #ifndef MAX_LINE_LENGTH
// #define MAX_LINE_LENGTH 512
// #endif

// #ifndef PI
// #define PI 3.141592654f
// #endif

// #ifndef PIx2
// #define PIx2 6.283185308f
// #endif

// #ifndef FACTOR_RAD_TO_DEG
// #define FACTOR_RAD_TO_DEG 57.295779505601046646705075978956f
// #endif

// #ifndef FACTOR_DEG_TO_RAD
// #define FACTOR_DEG_TO_RAD 0.01745329252222222222222222222222f
// #endif

// #ifndef SWAP
// #define SWAP(a, b, t)                                                          \
//     (t) = (a);                                                                 \
//     (a) = (b);                                                                 \
//     (b) = (t)
// #endif
#endif // #ifndef WIN32

#endif
