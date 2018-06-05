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

#ifndef _TIMER_H_
#define _TIMER_H_

#ifndef WIN32
#include <sys/time.h>
#endif

#include "Utility.h"

class Timer
{
  public:
    inline void Reset(const int N = 1)
    {
        m_first = true;
        m_idx = 0;
        m_start = m_avg = 0.0;
        m_times.assign(N, 0.0);
    }
    inline void Start() { m_start = GetTime(); }
    inline void Stop()
    {
        m_times[m_idx] = GetTime() - m_start + m_times[m_idx];
    }
    inline void Finish()
    {
        ++m_idx;
        m_avg = 0.0;
        if (m_first) {
            for (int i = 0; i < m_idx; ++i) m_avg = m_times[i] + m_avg;
            m_avg /= m_idx;
        } else {
            const int N = int(m_times.size());
            for (int i = 0; i < N; ++i) m_avg = m_times[i] + m_avg;
            m_avg /= N;
        }
        if (m_idx == m_times.size()) {
            m_first = false;
            m_idx = 0;
        }
        m_times[m_idx] = 0.0;
    }
    inline double GetAverageSeconds() const { return m_avg; }
    inline int GetAverageMilliseconds() const
    {
        return int(m_avg * 1000.0 + 0.5);
    }
    inline double GetFPS() const
    {
        return m_avg == 0.0 ? 0.0 : 1 / GetAverageSeconds();
    }

    inline void SaveB(FILE *fp) const
    {
        UT::SaveB(m_first, fp);
        UT::SaveB(m_idx, fp);
        UT::SaveB(m_start, fp);
        UT::SaveB(m_avg, fp);
        UT::VectorSaveB(m_times, fp);
    }
    inline void LoadB(FILE *fp)
    {
        UT::LoadB(m_first, fp);
        UT::LoadB(m_idx, fp);
        UT::LoadB(m_start, fp);
        UT::LoadB(m_avg, fp);
        UT::VectorLoadB(m_times, fp);
    }

    inline static double GetTime()
    {
#ifndef WIN32
        struct timeval time;
        gettimeofday(&time, NULL);
        return time.tv_sec + (time.tv_usec / 1000000.0);

#else
        SYSTEMTIME systime;
        GetSystemTime(&systime);
        return 0.001 * double(systime.wMilliseconds) +
               1.0 * double(systime.wSecond) + 60.0 * double(systime.wMinute) +
               3600.0 * double(systime.wHour);
#endif
    }

  private:
    bool m_first;
    int m_idx;
    double m_start, m_avg;
    std::vector<double> m_times;
};

#endif
