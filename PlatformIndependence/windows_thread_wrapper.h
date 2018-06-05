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

#ifndef _WINDOWS_THREAD_WRAPPER_H_
#define _WINDOWS_THREAD_WRAPPER_H_

#ifndef WIN32
#include <pthread.h>

#define WINAPI

typedef void *DWORD;
typedef unsigned long ULONG;
typedef DWORD *LPDWORD;
typedef void *LPVOID;
typedef pthread_t HANDLE;
typedef void *LPSECURITY_ATTRIBUTES;
typedef DWORD(WINAPI *PTHREAD_START_ROUTINE)(LPVOID lpThreadParameter);
typedef PTHREAD_START_ROUTINE LPTHREAD_START_ROUTINE;
typedef bool BOOL;
#define TRUE 1
#define FALSE 0

#define INFINITE 0xFFFFFFFF

inline HANDLE CreateThread(LPSECURITY_ATTRIBUTES lpThreadAttributes,
                           size_t dwStackSize,
                           LPTHREAD_START_ROUTINE lpStartAddress,
                           LPVOID lpParameter, ULONG dwCreationFlags,
                           LPDWORD lpThreadId)
{
    pthread_t pid;
    pthread_create(&pid, (pthread_attr_t *)lpThreadAttributes,
                   (void *(*)(void *))lpStartAddress, lpParameter);
    return pid;
}

inline BOOL CloseHandle(HANDLE hObject)
{
    return (pthread_detach(hObject) == 0);
}
#endif // WIN32
#endif
