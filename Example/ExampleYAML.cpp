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

#include <iostream>
#include <memory>
#include <iomanip>
#include <sstream>
#include "BundleAdjustment/BAInterface.h"

int main(int argc, char *argv[]) {
    EIBA::BAInterface interface;
    EIBA::BAParam param;
    param.verbose = 1;
    interface.SetParam(param);

    for (int i = 0; i < 93; ++i) {
        std::stringstream ss;
        ss << argv[1] << "/" << std::setfill('0') << std::setw(5) << i << ".yaml";
        interface.PushFrameInfoFromYAML(ss.str());
        std::vector<EIBA::KeyFrame> KFs;
        interface.Optimize(&KFs);
    }

    return 0;
}
