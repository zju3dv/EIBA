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

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include "stdafx.h"

#include <limits.h> // UCHAR_MAX ...
#include <stdarg.h> // va_list ...

#include "SSE.h"

//#define UT_STRING_MAX_LENGTH	128
#define UT_STRING_MAX_LENGTH 512
#define UT_FACTOR_RAD_TO_DEG 57.295779505601046646705075978956f
#define UT_FACTOR_DEG_TO_RAD 0.01745329252222222222222222222222f

#define UT_E 2.71828182845904523536f         // e
#define UT_LOG2E 1.44269504088896340736f     // log2(e)
#define UT_LOG10E 0.434294481903251827651f   // log10(e)
#define UT_LN2 0.693147180559945309417f      // ln(2)
#define UT_LN10 2.30258509299404568402f      // ln(10)
#define UT_PI 3.14159265358979323846f        // pi
#define UT_2PI 6.28318530717958647692f       // pi*2
#define UT_PI_2 1.57079632679489661923f      // pi/2
#define UT_PI_4 0.785398163397448309616f     // pi/4
#define UT_1_PI 0.318309886183790671538f     // 1/pi
#define UT_1_2PI 0.159154943091895335769f    // 1/(pi*2)
#define UT_2_PI 0.636619772367581343076f     // 2/pi
#define UT_2_SQRTPI 1.1283791670955125739f   // 2/sqrt(pi)
#define UT_SQRT2 1.4142135623730950488f      // sqrt(2)
#define UT_1_SQRT2 0.7071067811865475244f    // 1/sqrt(2)
#define UT_1_SQRT2PI 0.39894228040143267794f // 1/sqrt(2pi)

#define UT_GET_STRING(format, str)                                             \
    {                                                                          \
        if (format) {                                                          \
            va_list args;                                                      \
            va_start(args, format);                                            \
            vsprintf(str, format, args);                                       \
            va_end(args);                                                      \
        } else {                                                               \
            str[0] = 0;                                                        \
        }                                                                      \
    }

#define UT_SWAP(a, b)                                                          \
    {                                                                          \
        const auto t = (a);                                                    \
        (a) = (b);                                                             \
        (b) = (t);                                                             \
    }

#define UT_CLAMP(v, vMin, vMax) std::min(std::max((v), (vMin)), (vMax))

#define UT_DOT_TO_ANGLE(d) acos(UT_CLAMP((d), -1.0f, 1.0f))

//#if _DEBUG
//#define UT_ASSERT(expression) assert(expression)
//#else
#define UT_ASSERT(expression) UT::Assert(expression, #expression)
//#endif

#define UT_FLT_EPSILON_MIN 1.0e-05f
#define UT_FLT_EPSILON_MAX 1.0e-03f
#define UT_FLT_EPSILON_RATIO_MIN 1.0e-03f
#define UT_FLT_EPSILON_RATIO_MAX 0.01f

#define UT_DBL_EPSILON_MIN DBL_EPSILON
#define UT_DBL_EPSILON_MAX 1.0e-06
#define UT_DBL_EPSILON_RATIO_MIN 1.0e-05
#define UT_DBL_EPSILON_RATIO_MAX 0.001

namespace UT
{
void Print(const char *format, ...);
void PrintSeparator(const char c = '-');
void SaveSeparator(FILE *fp, const char c = '-');
inline void PrintLoaded(const char *fileName)
{
    Print("Loaded \'%s\'\n", fileName);
}
inline void PrintLoaded(const std::string fileName)
{
    PrintLoaded(fileName.c_str());
}
inline void PrintSaved(const char *fileName)
{
    Print("Saved \'%s\'\n", fileName);
}
inline void PrintSaved(const std::string fileName)
{
    PrintSaved(fileName.c_str());
}
template <typename TYPE>
inline void PrintValue(const TYPE v, const bool e = false)
{
    Print("%d", v);
}
template <> inline void PrintValue<float>(const float v, const bool e)
{
    if (e)
        Print("%e", v);
    else
        Print("%f", v);
}
template <> inline void PrintValue<double>(const double v, const bool e)
{
    if (e)
        Print("%e", v);
    else
        Print("%f", v);
}
template <typename TYPE>
inline void PrintError(const TYPE v1, const TYPE v2, const bool e = false)
{
    Print("|");
    PrintValue<TYPE>(v1, e);
    Print(" - ");
    PrintValue<TYPE>(v2, e);
    Print("| = ");
    PrintValue<TYPE>(v1 >= v2 ? v1 - v2 : v2 - v1, e);
    Print("\n");
}

void Assert(const bool expression, const char *format, ...);
void Error(const char *format = NULL, ...);
void Check(const char *format = NULL, ...);

void DebugStart();
bool Debugging();
void DebugStop();

template <typename TYPE> inline TYPE Random();
template <> inline int Random<int>() { return rand() % 32767; } // [0, 32767]
template <> inline bool Random<bool>() { return (Random<int>() & 1) == 0; }
template <> inline ubyte Random<ubyte>() { return Random<int>() & 255; }
template <> inline ushort Random<ushort>() { return ushort(Random<int>()); }
template <> inline short Random<short>()
{
    return short(Random<int>() - 16384);
}
template <> inline float Random<float>() { return Random<int>() / 32767.0f; }
template <typename TYPE> inline TYPE Random(const TYPE vMax);
template <> inline ubyte Random<ubyte>(const ubyte vMax)
{
    return rand() % vMax;
}
template <> inline int Random<int>(const int vMax)
{
    return Random<int>() % vMax;
}
template <> inline float Random<float>(const float vMax)
{
    return Random<float>() * vMax;
}

template <typename TYPE> inline TYPE Random(const TYPE vMin, const TYPE vMax)
{
    return TYPE(Random<float>() * (vMax - vMin) + vMin);
}

template <typename TYPE> inline void Random(TYPE *v, const int N)
{
    for (int i = 0; i < N; ++i) v[i] = Random<TYPE>();
}
template <typename TYPE>
inline void Random(TYPE *v, const int N, const TYPE vMax)
{
    for (int i = 0; i < N; ++i) v[i] = Random<TYPE>(vMax);
}
template <typename TYPE>
inline void Random(TYPE *v, const int N, const TYPE vMin, const TYPE vMax)
{
    for (int i = 0; i < N; ++i) v[i] = Random<TYPE>(vMin, vMax);
}

template <typename TYPE> inline TYPE Epsilon();
template <> inline float Epsilon<float>() { return FLT_EPSILON; }
template <> inline double Epsilon<double>() { return DBL_EPSILON; }
template <typename TYPE> inline TYPE Invalid();
template <> inline ubyte Invalid<ubyte>() { return UCHAR_MAX; }
template <> inline short Invalid<short>() { return SHRT_MAX; }
template <> inline ushort Invalid<ushort>() { return USHRT_MAX; }
template <> inline int Invalid<int>() { return INT_MAX; }
template <> inline float Invalid<float>() { return FLT_MAX; }
template <> inline double Invalid<double>() { return DBL_MAX; }
template <typename TYPE> inline bool NotANumber(const TYPE v);
template <> inline bool NotANumber<float>(const float v)
{
    return _isnanf(v) != 0;
}
template <> inline bool NotANumber<double>(const double v)
{
    return _isnan(v) != 0;
}

template <class TYPE> inline void SaveB(const TYPE &v, FILE *fp)
{
    fwrite(&v, sizeof(TYPE), 1, fp);
}
template <class TYPE> inline void SaveB(const TYPE *v, const int N, FILE *fp)
{
    if (N > 0) fwrite(v, sizeof(TYPE), N, fp);
}
template <class TYPE> inline void LoadB(TYPE &v, FILE *fp)
{
    fread(&v, sizeof(TYPE), 1, fp);
}
template <class TYPE> inline void LoadB(TYPE *v, const int N, FILE *fp)
{
    if (N > 0) fread(v, sizeof(TYPE), N, fp);
}
template <typename TYPE> inline TYPE LoadB(FILE *fp)
{
    TYPE v;
    LoadB<TYPE>(v, fp);
    return v;
}
template <typename TYPE> inline bool Load(TYPE &v, FILE *fp);
template <> inline bool Load<float>(float &v, FILE *fp)
{
    return fscanf(fp, "%f", &v) == 1;
}
template <> inline bool Load<double>(double &v, FILE *fp)
{
    return fscanf(fp, "%lf", &v) == 1;
}
template <typename TYPE> inline TYPE Load(FILE *fp)
{
    TYPE v;
    if (Load<TYPE>(v, fp))
        return v;
    else
        return Invalid<TYPE>();
}
template <class TYPE> inline float MemoryMB(const int N = 1)
{
    return sizeof(TYPE) * N / 1024.0f;
}

bool FileExists(const std::string fileName);
bool FileCopy(const std::string fileNameSrc, const std::string fileNameDst);
bool FileDelete(const std::string fileName, const bool check = true,
                const bool verbose = true);
std::vector<std::string> FilesSearch(const std::string fileNameFirst,
                                     const int iStart = 0, const int iStep = 1,
                                     const int iEnd = INT_MAX,
                                     const bool verbose = true);
void FilesStartSaving(const std::string fileNameFirst, const bool check = true,
                      const bool verbose = true);

std::string FileNameExtractDirectory(const std::string fileName);
std::string FileNameRemoveDirectory(const std::string fileName);
std::string FileNameExtractExtension(const std::string fileName);
std::string FileNameRemoveExtension(const std::string fileName);
std::string FileNameRemoveDirectoryExtension(const std::string fileName);
std::string FileNameRemovePrefix(const std::string fileName,
                                 const std::string prefix);
std::string FileNameAppendSuffix(const std::string fileName,
                                 const std::string suffix);
std::string FileNameAppendSuffix(const std::string fileName, const int suffix);
std::string FileNameReplaceDirectory(const std::string fileName,
                                     const std::string dirSrc,
                                     const std::string dirDst);
template <typename TYPE>
inline TYPE FileNameExtractSuffix(const std::string fileName);
template <> inline int FileNameExtractSuffix<int>(const std::string fileName)
{
    int i2 = int(fileName.length());
    while (--i2 >= 0 && !isdigit(fileName[i2]))
        ;
    int i1 = ++i2;
    while (--i1 >= 0 && isdigit(fileName[i1]))
        ;
    if (++i1 == i2)
        return -1;
    else
        return atoi(fileName.substr(i1, i2 - i1).c_str());
}
template <>
inline double FileNameExtractSuffix<double>(const std::string fileName)
{
    const std::string _fileName =
        FileNameRemoveDirectoryExtension(fileName).c_str();
    const int N = int(_fileName.length());
    for (int i = 0; i < N; ++i) {
        if (isdigit(_fileName[i]))
            return atof(_fileName.substr(i, N - i).c_str());
    }
    return DBL_MAX;
}
std::string FileNameIncreaseSuffix(const std::string fileName, const int incr);

std::string String(const char *format, ...);
std::string StringInput();
std::string StringReplace(const std::string str, const std::string strSrc,
                          const std::string strDst);
void StringSaveB(const std::string &str, FILE *fp);
void StringLoadB(std::string &str, FILE *fp);
void StringsSaveB(const std::vector<std::string> &strs, FILE *fp);
void StringsLoadB(std::vector<std::string> &strs, FILE *fp);
inline float StringMemoryMB(const std::string &str)
{
    return MemoryMB<char>(int(str.size()));
}

std::vector<std::string> Strings(const std::string str0);
std::vector<std::string> Strings(const std::string str0,
                                 const std::string str1);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2,
                                 const std::string str3);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4);
std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4,
                                 const std::string str5);
std::vector<std::string> Strings(const std::string *strs, const int N);
template <class TYPE_SRC, class TYPE_DST>
inline std::vector<const TYPE_DST *> Pointers(const std::vector<TYPE_SRC> &V)
{
    std::vector<const TYPE_DST *> Vptr;
    const int N = int(V.size());
    Vptr.resize(N);
    for (int i = 0; i < N; ++i) Vptr[i] = &V[i];
    return Vptr;
}
template <class TYPE_SRC_1, class TYPE_SRC_2, class TYPE_DST>
inline std::vector<const TYPE_DST *> Pointers(const std::vector<TYPE_SRC_1> &V1,
                                              const std::vector<TYPE_SRC_2> &V2)
{
    std::vector<const TYPE_DST *> Vptr;
    const int N1 = int(V1.size());
    for (int i = 0; i < N1; ++i) Vptr.push_back(&V1[i]);
    const int N2 = int(V2.size());
    for (int i = 0; i < N2; ++i) Vptr.push_back(&V2[i]);
    return Vptr;
}
void SaveValues(const char *fileName, const std::vector<float> &vals);
void SaveHistogram(const char *fileName, const std::vector<float> &vals,
                   const int nBins);

template <class TYPE> inline void VectorMakeZero(std::vector<TYPE> &V)
{
    memset(V.data(), 0, sizeof(TYPE) * V.size());
}
template <class TYPE>
inline void VectorMakeZero(std::vector<TYPE> &V, const int i1, const int i2)
{
    memset(V.data() + i1, 0, sizeof(TYPE) * (i2 - i1));
}
template <class TYPE_1, class TYPE_2>
inline bool VectorEqual(const TYPE_1 *v1, const TYPE_2 *v2, const int N)
{
    bool equal = true;
    for (int i = 0; i < N && equal; ++i) equal = v1[i] == v2[i];
    return equal;
}
template <class TYPE_1, class TYPE_2>
inline bool VectorEqual(const std::vector<TYPE_1> &V1,
                        const std::vector<TYPE_2> &V2)
{
    return V1.size() == V2.size() &&
           VectorEqual<TYPE_1, TYPE_2>(V1.data(), V2.data(), int(V1.size()));
}

template <typename TYPE>
inline bool AssertEqual(const TYPE v1, const TYPE v2, const int verbose = 1,
                        const TYPE eps = 0);
template <typename TYPE>
inline bool VectorAssertEqual(const TYPE *v1, const TYPE *v2, const int N,
                              const int verbose = 1, const TYPE eps = 0)
{
    bool equal = true;
    for (int i = 0; i < N && equal; ++i)
        equal = AssertEqual<TYPE>(v1[i], v2[i], 1, eps);
    if (!equal && verbose > 0) {
        const bool e = verbose > 1;
        PrintSeparator();
        for (int i = 0; i < N; ++i) PrintError<TYPE>(v1[i], v2[i], e);
    }
    return equal;
}
template <class TYPE_1, class TYPE_2>
inline void VectorAssertEqual(const std::vector<TYPE_1> &V1,
                              const std::vector<TYPE_2> &V2)
{
    UT_ASSERT(VectorEqual(V1, V2));
}
template <class TYPE>
inline std::vector<TYPE> VectorRepeat(const std::vector<TYPE> &V, const int N)
{
    if (N <= 0)
        return std::vector<TYPE>();
    else if (N == 1)
        return V;
    std::vector<TYPE> _V;
    for (int i = 0; i < N; ++i) _V.insert(_V.end(), V.begin(), V.end());
    return _V;
}
template <class TYPE>
inline void VectorSaveB(const std::vector<TYPE> &V, FILE *fp)
{
    const int N = int(V.size());
    SaveB<int>(N, fp);
    SaveB<TYPE>(V.data(), N, fp);
}
template <class TYPE>
inline bool VectorSaveB(const std::vector<TYPE> &V, const char *fileName)
{
    FILE *fp = fopen(fileName, "wb");
    if (!fp) return false;
    VectorSaveB<TYPE>(V, fp);
    fclose(fp);
    PrintSaved(fileName);
    return true;
}
template <class TYPE> inline int VectorLoadB(std::vector<TYPE> &V, FILE *fp)
{
    const int N = LoadB<int>(fp);
//    Print("Vector Load length %d\n", N);
    V.resize(N);
    LoadB<TYPE>(V.data(), N, fp);
    return N;
}
template <class TYPE>
inline bool VectorLoadB(std::vector<TYPE> &V, const char *fileName)
{
    FILE *fp = fopen(fileName, "rb");
    if (!fp) return false;
    VectorLoadB<TYPE>(V, fp);
    fclose(fp);
    PrintLoaded(fileName);
    return true;
}
template <class TYPE>
inline void VectorsSaveB(const std::vector<std::vector<TYPE>> &Vs, FILE *fp)
{
    const int N = int(Vs.size());
    SaveB<int>(N, fp);
    for (int i = 0; i < N; ++i) VectorSaveB(Vs[i], fp);
}
template <class TYPE>
inline void VectorsLoadB(std::vector<std::vector<TYPE>> &Vs, FILE *fp)
{
    const int N = LoadB<int>(fp);
    Vs.resize(N);
    for (int i = 0; i < N; ++i) VectorLoadB(Vs[i], fp);
}
template <class TYPE> inline float VectorMemoryMB(const std::vector<TYPE> &V)
{
    return MemoryMB<TYPE>(int(V.capacity()));
}
template <class TYPE>
inline float VectorsMemoryMB(const std::vector<std::vector<TYPE>> &Vs)
{
    float sum = 0.0f;
    const int N = int(Vs.size());
    for (int i = 0; i < N; ++i) sum = VectorMemoryMB(Vs[i]) + sum;
    return sum;
}

template <class TYPE> inline void ListSaveB(const std::list<TYPE> &L, FILE *fp)
{
    const int N = int(L.size());
    SaveB<int>(N, fp);
    for (typename std::list<TYPE>::const_iterator it = L.begin(); it != L.end();
         ++it)
        SaveB<TYPE>(*it, fp);
}
template <class TYPE> inline void ListLoadB(std::list<TYPE> &L, FILE *fp)
{
    const int N = LoadB<int>(fp);
    L.resize(N);
    for (typename std::list<TYPE>::iterator it = L.begin(); it != L.end(); ++it)
        LoadB<TYPE>(*it, fp);
}

enum CornerType {
    CORNER_LEFT_TOP,
    CORNER_LEFT_DOWN,
    CORNER_RIGHT_DOWN,
    CORNER_RIGHT_TOP,
    CORNER_TYPES
};



template <typename TYPE>
inline bool Equal(const TYPE v1, const TYPE v2, const TYPE eps = 0)
{
    return eps == 0 && v1 == v2 ||
           eps > 0 && (v1 > v2 && v1 - v2 <= eps || v1 < v2 && v2 - v1 <= eps);
}
template <>
inline bool Equal<float>(const float v1, const float v2, const float eps)
{
    const float e = fabs(v1 - v2), va1 = fabs(v1), va2 = fabs(v2),
                vMin = std::min(va1, va2), vMax = std::max(va1, va2);
    const float vd = vMin >= UT_FLT_EPSILON_MIN ? vMin : vMax, er = e / vd;
    return e < eps || e <= UT_FLT_EPSILON_MIN ||
           er <= UT_FLT_EPSILON_RATIO_MIN ||
           e <= UT_FLT_EPSILON_MAX && er <= UT_FLT_EPSILON_RATIO_MAX;
}
template <>
inline bool Equal<double>(const double v1, const double v2, const double eps)
{
    const double e = fabs(v1 - v2), va1 = fabs(v1), va2 = fabs(v2),
                 vMin = std::min(va1, va2), vMax = std::max(va1, va2);
    const double vd = vMin >= UT_DBL_EPSILON_MIN ? vMin : vMax, er = e / vd;
    return e < eps || e <= UT_DBL_EPSILON_MIN ||
           er <= UT_DBL_EPSILON_RATIO_MIN ||
           e <= UT_DBL_EPSILON_MAX && er <= UT_DBL_EPSILON_RATIO_MAX;
}
template <typename TYPE>
inline bool AssertEqual(const TYPE v1, const TYPE v2, const int verbose,
                        const TYPE eps)
{
    if (Equal<TYPE>(v1, v2, eps)) return true;
    if (verbose > 0) {
        PrintSeparator();
        PrintError<TYPE>(v1, v2, verbose > 1);
    }
    return Equal(v1, v2, eps);
}

template <class TYPE>
inline bool
CheckReduction(const TYPE &e1, const TYPE &e2, const int verbose = 1,
               const float eMin = FLT_EPSILON, const float eMax = FLT_MAX)
{
    const float e1_2 = e1.SquaredLength();
    if (e1_2 < eMin * eMin) return true;
    if (e1_2 > eMax * eMax) return false;
    const float e2_2 = e2.SquaredLength();
    if (e1_2 >= e2_2) return true;
    if (verbose > 0) {
        PrintSeparator();
        if (verbose > 1) {
            Print("    %e: ", e1_2);
            e1.Print(true);
            Print("--> %e: ", e2_2);
            e2.Print(true);
        } else {
            Print("    %f: ", e1_2);
            e1.Print(false);
            Print("--> %f: ", e2_2);
            e2.Print(false);
        }
    }
    return false;
}
template <>
inline bool CheckReduction<float>(const float &e1, const float &e2,
                                  const int verbose, const float eMin,
                                  const float eMax)
{
    const float e1_a = fabs(e1);
    if (e1_a < eMin) return true;
    if (e1_a > eMax) return false;
    const float e2_a = fabs(e2);
    if (e1_a >= e2_a) return true;
    if (verbose > 0) {
        PrintSeparator();
        if (verbose > 1)
            Print("    %e --> %e\n", e1, e2);
        else
            Print("    %f --> %f\n", e1, e2);
    }
    return false;
}

template <typename TYPE> inline TYPE Input();
template <> inline std::string Input<std::string>() { return StringInput(); }
template <> inline int Input<int>()
{
    const std::string input = Input<std::string>();
    int _input;
    sscanf(input.c_str(), "%d", &_input);
    return _input;
}
template <> inline char Input<char>()
{
    const std::string input = Input<std::string>();
    char _input;
    sscanf(input.c_str(), "%c", &_input);
    return _input;
}
template <typename TYPE> inline TYPE Input(const char *str)
{
    Print("%s << ", str);
    return Input<TYPE>();
}

template <class ERROR> inline float ESSquaredLength(const ERROR &e)
{
    return e.SquaredLength();
}
template <> inline float ESSquaredLength<float>(const float &e)
{
    return e * e;
}
template <class ERROR>
inline void ESErrorPrint(const ERROR &e, const bool l = true)
{
    e.Print(l);
}
template <> inline void ESErrorPrint<float>(const float &e, const bool l)
{
    Print("%f", e);
}
template <class INDEX> inline void ESIndexPrint(const INDEX &idx)
{
    idx.Print();
}
template <> inline void ESIndexPrint<int>(const int &idx)
{
    if (idx != -1) Print(" [%d]", idx);
}
template <class ERROR, class INDEX> class ES
{
  public:
    inline void Initialize()
    {
        m_Se2 = m_Swe2 = 0.0f;
        m_SN = 0;
        m_e2Max = m_we2Max = -1.0f;
    }
    inline void Accumulate(const ERROR &e, const float we2 = -1.0f,
                           const INDEX idx = -1)
    {
        const float e2 = ESSquaredLength<ERROR>(e);
        if (e2 == -1.0f)
            m_Se2 = -1.0f;
        else
            m_Se2 = e2 + m_Se2;
        if (we2 == -1.0f)
            m_Swe2 = -1.0f;
        else
            m_Swe2 = we2 + m_Swe2;
        ++m_SN;
        if (we2 == -1.0f && e2 > m_e2Max || we2 != -1.0f && we2 > m_we2Max) {
            m_e2Max = e2;
            m_we2Max = we2;
            m_eMax = e;
            m_idxMax = idx;
        }
    }
    inline bool Valid() const { return m_SN > 0; }
    inline float Average() const
    {
        return m_SN == 0 ? 0.0f : sqrt(m_Se2 / m_SN);
    }
    inline void Print(const std::string str = "", const bool l = true,
                      const bool n = true) const
    {
        // if(!Valid())
        //	return;
        UT::Print("%s", str.c_str());
        if (m_Swe2 != -1.0f) {
            if (l)
                UT::Print("%e", m_Swe2);
            else
                UT::Print("%.2e", m_Swe2);
        }
        if (m_Se2 != -1.0f) {
            if (m_Swe2 != -1.0f) UT::Print(" <-- ");
            const float eAvg = Average();
            if (l)
                UT::Print("%f", eAvg);
            else
                UT::Print("%.2f", eAvg);
        }
        UT::Print(" <= ");
        ESErrorPrint(m_eMax, l);
        if (m_we2Max != -1.0f) {
            UT::Print(" --> ");
            if (l)
                UT::Print("%e", m_we2Max);
            else
                UT::Print("%.2e", m_we2Max);
        }
        ESIndexPrint<INDEX>(m_idxMax);
        if (n) UT::Print("\n");
    }

  public:
    float m_Se2, m_Swe2;
    int m_SN;
    float m_e2Max, m_we2Max;
    ERROR m_eMax;
    INDEX m_idxMax;
};
}

#endif
