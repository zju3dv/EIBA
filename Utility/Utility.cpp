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

#include "MultiThread.h"
#include "Utility.h"
#include <iostream>

//#define UT_PRINT_CHECK

namespace UT
{
//static boost::mutex g_mutex;
static std::mutex g_mutex;
static char g_str[UT_STRING_MAX_LENGTH];
static bool g_debug = false;

void Print(const char *format, ...)
{
    MT_SCOPE_LOCK_BEGIN(g_mutex);
    UT_GET_STRING(format, g_str);
    printf("%s", g_str);
    MT_SCOPE_LOCK_END(g_mutex);
}

void PrintSeparator(const char c)
{
    Print("\r");
    for (int i = 0; i < 79; ++i) Print("%c", c);
    Print("\n");
}

void SaveSeparator(FILE *fp, const char c)
{
    fprintf(fp, "\r");
    for (int i = 0; i < 79; ++i) fprintf(fp, "%c", c);
    fprintf(fp, "\n");
}

void Assert(const bool expression, const char *format, ...)
{
    // assert(expression);
    if (expression) return;
    char str[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, str);
    Print("%s", str);
    exit(0);
}

void Error(const char *format, ...)
{
    PrintSeparator('!');
    char str[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, str);
    Print("ERROR: %s", str);
    exit(0);
}

void Check(const char *format, ...)
{
#ifdef UT_PRINT_CHECK
    PrintSeparator('!');
    char str[UT_STRING_MAX_LENGTH];
    UT_GET_STRING(format, str);
    Print("CHECK: %s", str);
#endif
}

void DebugStart()
{
    PrintSeparator('!');
    Print("Debug...\n");
    MT_SCOPE_LOCK_BEGIN(g_mutex);
    UT_ASSERT(!g_debug);
    g_debug = true;
    MT_SCOPE_LOCK_END(g_mutex);
}

bool Debugging()
{
    MT_SCOPE_LOCK_BEGIN(g_mutex);
    return g_debug;
    MT_SCOPE_LOCK_END(g_mutex);
}

void DebugStop()
{
    MT_SCOPE_LOCK_BEGIN(g_mutex);
    UT_ASSERT(g_debug);
    g_debug = false;
    MT_SCOPE_LOCK_END(g_mutex);
}

bool FileExists(const std::string fileName)
{
#ifdef WIN32
    return _access(fileName.c_str(), 0) == 0;
#else
    return false;
#endif
}

bool FileCopy(const std::string fileNameSrc, const std::string fileNameDst)
{
    if (FileExists(fileNameDst)) return false;
    char command[UT_STRING_MAX_LENGTH];
    sprintf(command, "copy %s %s",
            StringReplace(fileNameSrc, "/", "\\").c_str(),
            StringReplace(fileNameDst, "/", "\\").c_str());
    Print("%s\n", command);
#ifdef WIN32
    system(command);
#endif
    return true;
}

bool FileDelete(const std::string fileName, const bool check,
                const bool verbose)
{
#ifdef CFG_DEBUG
    UT_ASSERT(FileExists(fileName));
#endif
    char command[UT_STRING_MAX_LENGTH];
    sprintf(command, "del %s", fileName.c_str());
    if (check) {
        char input[UT_STRING_MAX_LENGTH];
        Print("Delete \'%s\'? (Y/N) ", fileName.c_str());
        scanf("%s", input);
        if (strlen(input) != 1 || input[0] != 'Y' && input[0] != 'y')
            return false;
    }
#ifdef WIN32
    if (verbose) Print("Deleted \'%s\'\n", fileName.c_str());
    system(command);
#endif
    return true;
}

std::vector<std::string> FilesSearch(const std::string fileNameFirst,
                                     const int iStart, const int iStep,
                                     const int iEnd, const bool verbose)
{
    std::vector<std::string> fileNames;
#ifdef WIN32
    if (FileNameRemoveDirectoryExtension(fileNameFirst) == "*") {
        const std::string dir = FileNameExtractDirectory(fileNameFirst);
        _finddata_t fd;
        intptr_t fh = _findfirst(fileNameFirst.c_str(), &fd);
        if (fh == -1) return fileNames;
        do {
            const std::string fileName = dir + fd.name;
            fileNames.push_back(fileName);
            if (verbose) Print("\rFound \'%s\'", fileName.c_str());
        } while (_findnext(fh, &fd) == 0);

        int i, j;
        const int N = int(fileNames.size());
        for (i = iStart, j = 0; i < N && i <= iEnd; i += iStep)
            fileNames[j++] = fileNames[i];
        fileNames.resize(j);
    } else {
        int i = iStart;
        while (i <= iEnd) {
            const std::string fileName =
                FileNameIncreaseSuffix(fileNameFirst, i);
            if (!FileExists(fileName)) break;
            fileNames.push_back(fileName);
            if (verbose) Print("\rFound \'%s\'", fileName.c_str());
            if (iStep == 0) break;
            i += iStep;
        }
    }
    if (verbose) {
        if (fileNames.empty()) Print("Not found \'%s\'", fileNameFirst.c_str());
        Print("\n");
    }
#else
    fprintf(stderr, "Not implemeted, file: %s, line %d\n", __FILE__, __LINE__);
#endif
    return fileNames;
}

void FilesStartSaving(const std::string fileNameFirst, const bool check,
                      const bool verbose)
{
#ifdef WIN32
    const std::string dir = FileNameExtractDirectory(fileNameFirst);
    if (FileExists(dir)) {
        const std::vector<std::string> fileNames =
            FilesSearch(fileNameFirst, 0, 1, INT_MAX, verbose);
        const int N = int(fileNames.size());
        for (int i = 0; i < N; ++i) {
            if (!FileDelete(fileNames[i], check && i == 0, verbose)) break;
        }
    } else
        CreateDirectory(dir.c_str(), 0);
#endif
}

std::string FileNameExtractDirectory(const std::string fileName)
{
    const std::string::size_type i1 = fileName.rfind('/'),
                                 i2 = fileName.rfind('\\');
    if (i1 == std::string::npos && i2 == std::string::npos)
        return std::string();
    else if (i1 != std::string::npos && i2 == std::string::npos)
        return fileName.substr(0, i1 + 1);
    else if (i1 == std::string::npos && i2 != std::string::npos)
        return fileName.substr(0, i2 + 1);
    else if (i1 > i2)
        return fileName.substr(0, i1 + 1);
    else
        return fileName.substr(0, i2 + 1);
}

std::string FileNameRemoveDirectory(const std::string fileName)
{
    const std::string::size_type i1 = fileName.rfind('/'),
                                 i2 = fileName.rfind('\\');
    if (i1 == std::string::npos && i2 == std::string::npos)
        return fileName;
    else if (i1 != std::string::npos && i2 == std::string::npos)
        return fileName.substr(i1 + 1, fileName.size());
    else if (i1 == std::string::npos && i2 != std::string::npos)
        return fileName.substr(i2 + 1, fileName.size());
    else if (i1 > i2)
        return fileName.substr(i1 + 1, fileName.size());
    else
        return fileName.substr(i2 + 1, fileName.size());
}

std::string FileNameExtractExtension(const std::string fileName)
{
    const std::string::size_type i = fileName.rfind('.');
    if (i == std::string::npos)
        return std::string();
    else
        return fileName.substr(i + 1, fileName.size());
}

std::string FileNameRemoveExtension(const std::string fileName)
{
    const std::string::size_type i = fileName.rfind('.');
    if (i == std::string::npos)
        return fileName;
    else
        return fileName.substr(0, i);
}

std::string FileNameRemoveDirectoryExtension(const std::string fileName)
{
    return FileNameRemoveDirectory(FileNameRemoveExtension(fileName));
}

std::string FileNameRemovePrefix(const std::string fileName,
                                 const std::string prefix)
{
    if (fileName == "" || fileName.find(prefix) != 0)
        return fileName;
    else
        return fileName.substr(prefix.length(), fileName.length());
}

std::string FileNameAppendSuffix(const std::string fileName,
                                 const std::string suffix)
{
    return FileNameRemoveExtension(fileName) + suffix + "." +
           FileNameExtractExtension(fileName);
}

std::string FileNameAppendSuffix(const std::string fileName, const int suffix)
{
    char buf[UT_STRING_MAX_LENGTH];
    sprintf(buf, "%d", suffix);
    return FileNameAppendSuffix(fileName, buf);
}

std::string FileNameReplaceDirectory(const std::string fileName,
                                     const std::string dirSrc,
                                     const std::string dirDst)
{
    if (fileName == "" || dirSrc != "" && fileName.find(dirSrc) != 0)
        return fileName;
    else if (fileName[0] == '.')
        return dirDst + fileName;
    else
        return dirDst + fileName.substr(dirSrc.length(), fileName.length());
}

std::string FileNameIncreaseSuffix(const std::string fileName, const int incr)
{
    const int len = int(fileName.length());
    int i2 = len;
    while (--i2 >= 0 && !isdigit(fileName[i2]))
        ;
    int i1 = ++i2;
    while (--i1 >= 0 && isdigit(fileName[i1]))
        ;
    const int number =
        ++i1 == i2 ? incr : atoi(fileName.substr(i1, i2 - i1).c_str()) + incr;
    const int width1 = i2 - i1, width2 = int(log10f(float(number)));
    const int width = width1 > width2 ? width1 : width2;

    char buf[10];
    switch (width) {
        case 2: sprintf(buf, "%.2d", number); break;
        case 3: sprintf(buf, "%.3d", number); break;
        case 4: sprintf(buf, "%.4d", number); break;
        case 5: sprintf(buf, "%.5d", number); break;
        case 6: sprintf(buf, "%.6d", number); break;
        case 7: sprintf(buf, "%.7d", number); break;
        case 8: sprintf(buf, "%.8d", number); break;
        case 9: sprintf(buf, "%.9d", number); break;
        case 10: sprintf(buf, "%.10d", number); break;
        default: sprintf(buf, "%d", number); break;
    }
    return fileName.substr(0, i1) + buf + fileName.substr(i2, len - i2);
}

std::vector<std::string> Strings(const std::string str0)
{
    std::vector<std::string> strs(1);
    strs[0] = str0;
    return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1)
{
    std::vector<std::string> strs(2);
    strs[0] = str0;
    strs[1] = str1;
    return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2)
{
    std::vector<std::string> strs(3);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3)
{
    std::vector<std::string> strs(4);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    strs[3] = str3;
    return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4)
{
    std::vector<std::string> strs(5);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    strs[3] = str3;
    strs[4] = str4;
    return strs;
}

std::vector<std::string> Strings(const std::string str0, const std::string str1,
                                 const std::string str2, const std::string str3,
                                 const std::string str4, const std::string str5)
{
    std::vector<std::string> strs(6);
    strs[0] = str0;
    strs[1] = str1;
    strs[2] = str2;
    strs[3] = str3;
    strs[4] = str4;
    strs[5] = str5;
    return strs;
}

std::vector<std::string> Strings(const std::string *strs, const int N)
{
    std::vector<std::string> _strs(N);
    for (int i = 0; i < N; ++i) _strs[i] = strs[i];
    return _strs;
}

void StringSaveB(const std::string &str, FILE *fp)
{
    char buf[UT_STRING_MAX_LENGTH];
    sprintf(buf, "%s\n", str.c_str());
    fwrite(buf, 1, strlen(buf), fp);
}

void StringLoadB(std::string &str, FILE *fp)
{
    char buf[UT_STRING_MAX_LENGTH];
    fgets(buf, UT_STRING_MAX_LENGTH, fp);
    const int len = int(strlen(buf));
    if (buf[len - 1] == 10) buf[len - 1] = 0;
    str = buf;
}

std::string String(const char *format, ...)
{
    MT_SCOPE_LOCK_BEGIN(g_mutex);
    UT_GET_STRING(format, g_str);
    return g_str;
    MT_SCOPE_LOCK_END(g_mutex);
}

std::string StringInput()
{
    std::string input;
    MT_SCOPE_LOCK_BEGIN(g_mutex);
//    gets(g_str);
//    input = g_str;
    std::cin >> g_str;
    input = g_str;
    MT_SCOPE_LOCK_END(g_mutex);
    return input;
}

std::string StringReplace(const std::string str, const std::string strSrc,
                          const std::string strDst)
{
    std::string::size_type pos;
    std::string res = str;
    while (1) {
        if ((pos = res.find(strSrc)) == std::string::npos) break;
        res.replace(pos, strSrc.length(), strDst);
    }
    return res;
}

void StringsSaveB(const std::vector<std::string> &strs, FILE *fp)
{
    const int size = int(strs.size());
    SaveB<int>(size, fp);
    for (int i = 0; i < size; ++i) StringSaveB(strs[i], fp);
}

void StringsLoadB(std::vector<std::string> &strs, FILE *fp)
{
    const int size = LoadB<int>(fp);
    strs.resize(size);
    for (int i = 0; i < size; ++i) StringLoadB(strs[i], fp);
}

void SaveValues(const char *fileName, const std::vector<float> &vals)
{
    FILE *fp = fopen(fileName, "w");
    const int N = int(vals.size());
    for (int i = 0; i < N; ++i) fprintf(fp, "%f\n", vals[i]);
    fclose(fp);
}

void SaveHistogram(const char *fileName, const std::vector<float> &vals,
                   const int nBins)
{
    int i;
    float val, valMin = FLT_MAX, valMax = -FLT_MAX;
    const int N = int(vals.size());
    for (i = 0; i < N; ++i) {
        val = vals[i];
        if (val < valMin) valMin = val;
        if (val > valMax) valMax = val;
    }
    const float binWidth = (valMax - valMin) / (nBins - 1);

    int iBin;
    std::vector<int> hist(N, 0);
    for (i = 0; i < N; ++i) {
        iBin = int((vals[i] - valMin) / binWidth);
        ++hist[iBin];
    }

    FILE *fp = fopen(fileName, "w");
    for (iBin = 0, val = valMin; iBin < nBins; ++iBin, val += binWidth)
        fprintf(fp, "%f %d\n", val, hist[iBin]);
    fclose(fp);
}
}
