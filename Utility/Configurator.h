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

#ifndef _CONFIGURATOR_H_
#define _CONFIGURATOR_H_

#include <map>

#include "Utility.h"

class Configurator
{
  public:
    inline Configurator() {}
    inline Configurator(const char *fileName) { Load(fileName); }
    inline std::string GetArgument(const std::string directive,
                                   const std::string default_val = "") const
    {
        const DirectiveArgumentMap::const_iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            return default_val;
        else
            return it->second;
    }
    inline int GetArgument(const std::string directive, const int default_val) const
    {
        const DirectiveArgumentMap::const_iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            return default_val;
        else
            return atoi(it->second.c_str());
    }
    inline float GetArgument(const std::string directive,
                             const float default_val) const
    {
        const DirectiveArgumentMap::const_iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            return default_val;
        else
            return float(atof(it->second.c_str()));
    }
    inline double GetArgument(const std::string directive,
                              const double default_val) const
    {
        const DirectiveArgumentMap::const_iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            return default_val;
        else
            return double(atof(it->second.c_str()));
    }

    inline void SetArgument(const std::string directive,
                            const std::string argument)
    {
        DirectiveArgumentMap::iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            m_directiveArgumentMap.insert(
                DirectiveArgumentMap::value_type(directive, argument));
        else
            it->second = argument;
    }
    inline void SetArgument(const std::string directive, const int argument)
    {
        char buf[UT_STRING_MAX_LENGTH];
        sprintf(buf, "%d", argument);
        DirectiveArgumentMap::iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            m_directiveArgumentMap.insert(
                DirectiveArgumentMap::value_type(directive, buf));
        else
            it->second = buf;
    }
    inline void SetArgument(const std::string directive, const float argument)
    {
        char buf[UT_STRING_MAX_LENGTH];
        sprintf(buf, "%f", argument);
        DirectiveArgumentMap::iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            m_directiveArgumentMap.insert(
                DirectiveArgumentMap::value_type(directive, buf));
        else
            it->second = buf;
    }
    inline void SetArgument(const std::string directive, const double argument)
    {
        char buf[UT_STRING_MAX_LENGTH];
        sprintf(buf, "%f", argument);
        DirectiveArgumentMap::iterator it =
            m_directiveArgumentMap.find(directive);
        if (it == m_directiveArgumentMap.end())
            m_directiveArgumentMap.insert(
                DirectiveArgumentMap::value_type(directive, buf));
        else
            it->second = buf;
    }

    inline bool Load(const char *fileName)
    {
        m_directiveArgumentMap.clear();
        FILE *fp = fopen(fileName, "r");
        if (!fp) return false;
        char buf[UT_STRING_MAX_LENGTH];
        while (fgets(buf, UT_STRING_MAX_LENGTH, fp)) {
            int len = int(strlen(buf));
            int i, j;
            for (i = j = 0; i < len; i++) {
                if (buf[i] != 10 && buf[i] != ' ') buf[j++] = buf[i];
            }
            len = j;

            buf[len] = 0;
            if (len < 2 || buf[0] == '/' && buf[1] == '/') continue;
            const std::string directive = strtok(buf, "=");
            const char *argument = strtok(NULL, "=");
            if (!argument) continue;
            m_directiveArgumentMap.insert(
                DirectiveArgumentMap::value_type(directive, argument));
        }
        fclose(fp);
        UT::PrintLoaded(fileName);
        return true;
    }

    void Print() const
    {
        UT::Print("[Configurator]\n");
        for (DirectiveArgumentMap::const_iterator it =
                 m_directiveArgumentMap.begin();
             it != m_directiveArgumentMap.end(); it++)
            UT::Print("  %s = %s\n", it->first.c_str(), it->second.c_str());
    }

  private:
    typedef std::map<std::string, std::string> DirectiveArgumentMap;
    DirectiveArgumentMap m_directiveArgumentMap;
};

#endif
