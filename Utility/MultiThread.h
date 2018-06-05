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
#include "PlatformIndependence/windows_thread_wrapper.h"

#ifndef _MULTI_THREAD_H_
#define _MULTI_THREAD_H_

#include "stdafx.h"

//#include <boost/thread/condition.hpp>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/shared_mutex.hpp>
#include <condition_variable>
#include <shared_mutex>

#include "Utility.h"

//#define MT_SCOPE_LOCK_BEGIN(m)                                                 \
//    {                                                                          \
//        boost::mutex::scoped_lock sl(m);
//#define MT_SCOPE_LOCK_END(m) }
//#define MT_READ_LOCK_BEGIN(m)                                                  \
//    {                                                                          \
//        boost::shared_lock<boost::shared_mutex> rl(m);
//#define MT_READ_LOCK_END(m) }
//#define MT_WRITE_LOCK_BEGIN(m)                                                 \
//    {                                                                          \
//        boost::upgrade_lock<boost::shared_mutex> wl1(m);                       \
//        boost::upgrade_to_unique_lock<boost::shared_mutex> wl2(wl1);
//#define MT_WRITE_LOCK_END(m) }

#define MT_SCOPE_LOCK_BEGIN(m) { std::lock_guard<std::mutex> lock{m}
#define MT_SCOPE_LOCK_END(m) }
#define MT_READ_LOCK_BEGIN(m) { std::shared_lock<std::shared_timed_mutex> rl(m);
#define MT_READ_LOCK_END(m) }
#define MT_WRITE_LOCK_BEGIN(m) { std::unique_lock<std::shared_timed_mutex> wl(m);
#define MT_WRITE_LOCK_END(m) }

namespace MT
{
class Thread
{
  public:
    virtual void Initialize(const bool serial = false,
                            const std::string name = "")
    {
        m_serial = serial;
        m_name = name;
    }

    virtual void Reset()
    {
        MT_WRITE_LOCK_BEGIN(m_runMT);
        m_run = 0;
        MT_WRITE_LOCK_END(m_runMT);
    }

    virtual void Start()
    {
        Reset();
#ifdef CFG_SERIAL
        if (!m_serial)
#endif
            CloseHandle(CreateThread(NULL, 0, Run, this, 0, NULL));
    }

    virtual void Stop()
    {
#ifdef CFG_SERIAL
        if (m_serial) return;
#endif
        // UT::Print("[%s] Stop1\n", m_name.c_str());
        MT_WRITE_LOCK_BEGIN(m_runMT);
        if (m_run == -2) return;
        m_run = -1;
        m_runCDT.notify_all();
        MT_WRITE_LOCK_END(m_runMT);

        MT_READ_LOCK_BEGIN(m_runMT);
        while (m_run != -2) m_runCDT.wait(rl);
        MT_READ_LOCK_END(m_runMT);
        // UT::Print("[%s] Stop2\n", m_name.c_str());
    }

    virtual bool WakeUp()
    {
#ifdef CFG_SERIAL
        if (m_serial) {
            Run();
            return true;
        }
#endif
        MT_WRITE_LOCK_BEGIN(m_runMT);
        if (m_run != 0) return false;
        m_run = 1;
        m_runCDT.notify_one();
        MT_WRITE_LOCK_END(m_runMT);
        return true;
    }

    virtual void Run() = 0;
    static inline DWORD WINAPI Run(LPVOID pParam)
    {
        Thread *pThread = (Thread *)pParam;
        // UT::Print("[%s] Run1\n", pThread->m_name.c_str());
        while (1) {
            MT_READ_LOCK_BEGIN(pThread->m_runMT);
            while (pThread->m_run == 0) pThread->m_runCDT.wait(rl);
            if (pThread->m_run == -1) break;
            MT_READ_LOCK_END(pThread->m_runMT);

            pThread->Run();

            MT_WRITE_LOCK_BEGIN(pThread->m_runMT);
            pThread->m_run = 0;
#ifdef CFG_VIEW
            pThread->m_runCDT.notify_one();
#endif
            MT_WRITE_LOCK_END(pThread->m_runMT);
        }
        // UT::Print("[%s] Run2\n", pThread->m_name.c_str());
        MT_WRITE_LOCK_BEGIN(pThread->m_runMT);
        pThread->m_run = -2;
        pThread->m_runCDT.notify_all();
        MT_WRITE_LOCK_END(pThread->m_runMT);
        return 0;
    }

    virtual bool Running()
    {
        MT_READ_LOCK_BEGIN(m_runMT);
        return m_run == 1;
        MT_READ_LOCK_END(m_runMT);
    }

#ifdef CFG_VIEW
    virtual void Synchronize()
    {
        MT_READ_LOCK_BEGIN(m_runMT);
        while (m_run == 1) m_runCDT.wait(rl);
        MT_READ_LOCK_END(m_runMT);
    }
#endif

  protected:
    int m_run;
//    boost::shared_mutex m_runMT;
      std::shared_timed_mutex m_runMT;
//    boost::condition m_runCDT;
      std::condition_variable_any m_runCDT;

    bool m_serial;
    std::string m_name;
};
}

#endif
