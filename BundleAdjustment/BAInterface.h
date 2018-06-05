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

#pragma once

#include <vector>
#include <string>
#include <cfloat>

namespace EIBA {
/// @class Point2
class Point2
{
public:
    float m_x, m_y;
};

/// @class Point3
class Point3 : public Point2
{
public:
    float m_z;
};

/// @class Camera
class Camera
{
public:
    /// rotation
    float m_rotation[3][3];
    /// position
    Point3 m_position;
};

/// @class Feature
class Feature
{
public:
    /// 2D measurement
    Point2 m_pt;
    /// inverse depth, set 0 if invalid
    float m_inv_depth;
};

/// @class Measurement
class Measurement : public Feature
{
public:
    /// matched keyframe index
    int m_matched_kf_idx;
    /// matched source feature local index (in keyframe)
    int m_matched_ftr_idx;
    /// covariance
    float m_cov2[2][2];
};

/// @class KeyFrame
class KeyFrame
{
public:
    /// camera pose
    Camera m_camera;
    /// depth in keyframe
    std::vector<float> m_inv_depths;
    /// mark camera updated
    bool m_camera_updated;
    /// mark depth updated
    std::vector<bool> m_depths_updated;
};

/// @struct BAParam
struct BAParam {
    /// weight of first frame pose prior
    float weight_fix = 1.0e2f;
    /// weight of feature measurements
    float weight_feature = 1.0e-5f;
    /// weight of frame constraints
    float weight_prior = 1.0e-3f;
    /// rotation variance of first frame pose prior
    float variance_fix_rotation = 3.046174198662e-6f; // (0.1*pi/180)^2
    /// position variance of first frame pose prior
    float variance_fix_position = 0.01f; // 0.1^2
    /// variance of inverse depth measurement
    float variance_depth_map = 0.01f;
    /// threshold for update rotation
    float update_rotation = 3.046174198662e-6f;
    /// threshold for update position
    float update_position = 1.0e-4f;
    /// threshold for update depth
    float update_depth = 1.0e-4f;
    /// set verbose level
    int verbose = 0;
    /// width and height, set this to print reprojection error in pixel
    int width = 640, height = 480;
    /// camera intrinsic, set this to print reprojection error in pixel
    float fx_fy_cx_cy[4] = {535.4, 539.2, 320.1, 247.6};
};

/// @class BAInterface
class BAInterface {
public:
    /// @brief constructor of BAInterface
    explicit BAInterface();

    /// @brief destructor of BAInterface
    ~BAInterface();

    /// @brief set parameters
    /// @param param BA parameters
    void SetParam(const BAParam &param);

    /// @brief push keyframe feature observations
    /// @param initial_camera  initial guess of current frame camera pose (rotation + position)
    /// @param initial_depths initial guess of last frame source feature's inverse depth
    /// @param source_features current frame source features(2D + inverse depth measurement)
    /// @oaram measured_features current frame measured feature matches (2D + inverse depth measurement + matched source feature index + covariance)
    void PushKeyframeFeatures(const Camera &initial_camera,
                              const std::vector<float> &initial_depths,
                              const std::vector<Feature> &source_features,
                              const std::vector<Measurement> &measured_features);

    /// @brief push between frame constrains
    /// @param kf_idx1 index of C1
    /// @param kf_idx2 index of C2
    /// @param C12 C2 * C1_{-1} * C12
    /// @param covariance
    void PushBetweenFrameConstraint(int kf_idx1, int kf_idx2,
                                    const Camera &C12, const float cov6[6][6]);

    /// @brief optimize, get updated keyframes and depths
    /// @oaram KFs store optimized Keyframes in optKFs
    void Optimize(std::vector<KeyFrame> *optKFs = nullptr);

#ifdef ENABLE_YAML_IO
    /// @brief push frame info from YAML files
    /// @param filename filename of YAML
    /// @note this would call PushKeyframeFeatures(*) and PushBetweenFrameConstraint(*)
    void PushFrameInfoFromYAML(std::string filename);
#endif
};

}  // namespace EIBA
