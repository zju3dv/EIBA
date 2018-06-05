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

#include "Camera.h"

#ifdef CFG_TUNE_PARAMETERS
extern float CAMERA_EPSILON_BIAS_GYROSCOPE = 3.046174198662e-4f;
extern void LOAD_PARAMETERS_CAMERA(const Configurator &cfgor)
{
    CAMERA_EPSILON_BIAS_GYROSCOPE = cfgor.GetArgument(
        "param_camera_epsilone_bias_gyroscope",
        sqrt(CAMERA_EPSILON_BIAS_GYROSCOPE) * UT_FACTOR_RAD_TO_DEG);
    CAMERA_EPSILON_BIAS_GYROSCOPE = CAMERA_EPSILON_BIAS_GYROSCOPE *
                                    CAMERA_EPSILON_BIAS_GYROSCOPE *
                                    UT_FACTOR_DEG_TO_RAD * UT_FACTOR_DEG_TO_RAD;
}
#endif
