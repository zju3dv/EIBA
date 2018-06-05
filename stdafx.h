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

// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#ifdef WIN32
#include "targetver.h"
#include <tchar.h>
#endif

#include <stdio.h>

#define CFG_SERIAL
#define CFG_VERBOSE

#ifdef WIN32
//#define CFG_VIEW
//#define CFG_TEST
#define CFG_DEPTH_MAP
#define CFG_GROUND_TRUTH
#define CFG_TUNE_PARAMETERS
//#define CFG_DEBUG
#ifdef CFG_DEBUG
#define CFG_DEBUG_EIGEN
#endif
#endif

#define CFG_VIEW
#define CFG_DEPTH_MAP


// TODO: reference additional headers your program requires here
#include <algorithm>
#include <assert.h>
#include <fstream>
#include <list>
#include <math.h>
#include <queue>
#include <stdlib.h>
#include <string>
#include <vector>
#ifdef WIN32
#include <io.h>
#include <windows.h>
#endif
#include <float.h>

typedef unsigned char ubyte;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long long ullong;

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
#ifdef small
#undef small
#endif
#ifdef _C2
#undef _C2
#endif
#ifdef _T
#undef _T
#endif
#ifdef ERROR
#undef ERROR
#endif
