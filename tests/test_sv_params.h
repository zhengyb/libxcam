/*
 * test_sv_params.h - parameters for surround view
 *
 *  Copyright (c) 2019 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Yinhang Liu <yinhangx.liu@intel.com>
 */

#include "interface/stitcher.h"

#if HAVE_JSON
#include <calibration_parser.h>
#endif

namespace XCam {

enum StitchScopicMode {
    ScopicMono,
    ScopicStereoLeft,
    ScopicStereoRight
};

static const char *intrinsic_names[] = {
    "intrinsic_camera_front.txt",
    "intrinsic_camera_right.txt",
    "intrinsic_camera_rear.txt",
    "intrinsic_camera_left.txt"
};

static const char *extrinsic_names[] = {
    "extrinsic_camera_front.txt",
    "extrinsic_camera_right.txt",
    "extrinsic_camera_rear.txt",
    "extrinsic_camera_left.txt"
};

#if HAVE_JSON
static const char *camera_calibration_json_names[] = {
    "",
    "",
    "",
    "camera_calibration_CamC3C8K.json",
    "camera_calibration_CamC6C8K.json",
    "k_camera_calibration.json"
};
#endif

uint32_t *
get_fisheye_img_roi_radius (
    CamModel model, StitchScopicMode scopic_mode, uint32_t *roi_radius)
{
    // enable macro XCAM_FISHEYE_IMG_ROI_RADIUS, and fine-tune the roi_radius

    switch (model) {
    case CamA2C1080P: {
        roi_radius[0] = 456;
        roi_radius[1] = 456;
        break;
    }
    case CamC3C4K: {
        switch (scopic_mode) {
        case ScopicStereoLeft: {
            roi_radius[0] = 1787;
            roi_radius[1] = 1787;
            roi_radius[2] = 1787;
            break;
        }
        case ScopicStereoRight: {
            roi_radius[0] = 1787;
            roi_radius[1] = 1787;
            roi_radius[2] = 1787;
            break;
        }
        default:
            XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
            break;
        }
        break;
    }
    case CamC3C8K: {
        switch (scopic_mode) {
        case ScopicStereoLeft: {
            roi_radius[0] = 1787;
            roi_radius[1] = 1787;
            roi_radius[2] = 1787;
            break;
        }
        case ScopicStereoRight: {
            roi_radius[0] = 1787;
            roi_radius[1] = 1787;
            roi_radius[2] = 1787;
            break;
        }
        default:
            XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
            break;
        }
        break;
    }
    case CamC6C8K: {
        roi_radius[0] = 1787;
        roi_radius[1] = 1787;
        roi_radius[2] = 1787;
        roi_radius[3] = 1787;
        roi_radius[4] = 1787;
        roi_radius[5] = 1787;
        break;
    }
    case CamD3C8K: {
        switch (scopic_mode) {
        case ScopicStereoLeft: {
            roi_radius[0] = 1802;
            roi_radius[1] = 1802;
            roi_radius[2] = 1802;
            break;
        }
        case ScopicStereoRight: {
            roi_radius[0] = 1801;
            roi_radius[1] = 1801;
            roi_radius[2] = 1801;
            break;
        }
        default:
            XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
            break;
        }
        break;
    }
    case CamD6C8K: {
        roi_radius[0] = 1802;
        roi_radius[1] = 1802;
        roi_radius[2] = 1802;
        roi_radius[3] = 1802;
        roi_radius[4] = 1802;
        roi_radius[5] = 1802;
        break;
    }
    default:
        XCAM_LOG_ERROR ("unsupported camera model (%d)", model);
        break;
    }

    return roi_radius;
}

/* 根据相机配置生成Bowl。
   FixMe: 对于不同的车型，差宽比差异较大，是否需要生成不同的Bowl.
   TODO： 如何可视化这个Bowl？
*/
BowlDataConfig
bowl_config (CamModel model)
{
    BowlDataConfig bowl;

    switch (model) {
    case CamB4C1080P: {
#if 0
        bowl.a = 6060.0f;
        bowl.b = 4388.0f;
        bowl.c = 3003.4f;
        bowl.angle_start = 0.0f;
        bowl.angle_end = 360.0f;
        bowl.center_z = 1500.0f;
        bowl.wall_height = 1800.0f;
        bowl.ground_length = 3000.0f;
#else
        bowl.a           = 1500.0f;   // ≈ 354.6
        bowl.b           = 800.0f;   // ≈ 163.2
        bowl.c           = 460.0f;   // ≈ 2 * 214
        bowl.angle_start = 0.0f;
        bowl.angle_end   = 360.0f;
        bowl.center_z    = 225.0f;   // 相机高度
        bowl.wall_height = 500.0f;   // 视需要，rig 很矮，可以取 300mm 左右
        bowl.ground_length = 647;//385.0f;
#endif
        break;
    }
    default:
        XCAM_LOG_ERROR ("unsupported camera model (%d)", model);
        break;
    }

    return bowl;
}

void
_log_bowl_data (const BowlDataConfig &bowl)
{
    XCAM_LOG_INFO (
        "Bowl Model Data:"
        " a: %.2f, b: %.2f, c: %.2f,"
        " angle_start: %.2f, angle_end: %.2f,"
        " center_z: %.2f, wall_height: %.2f, ground_length: %.2f",
        bowl.a, bowl.b, bowl.c,
        bowl.angle_start, bowl.angle_end,
        bowl.center_z, bowl.wall_height, bowl.ground_length);
}

BowlDataConfig
cal_bowl_config (PointFloat3 *camera_pos, int camera_num, float x_view_scope, float y_view_scope)
{
    BowlDataConfig bowl;

    if (!camera_pos || camera_num <= 0)
        return bowl;

    float sum_z = 0.0f;
    for (int i = 0; i < camera_num; ++i)
        sum_z += camera_pos[i].z;

    // 椭球中心位于相机位置的平均高度上；
    bowl.center_z = sum_z / camera_num;
    bowl.angle_start = 0.0f;
    bowl.angle_end = 360.0f;

    // c强制为2倍安装高度，使得与地面相交
    bowl.c = 2.0f * bowl.center_z;
    if (bowl.c == 0.0f)
        return bowl;

    const float center_z = bowl.center_z;
    const float c = bowl.c;
    const float r = sqrtf (1.0f - center_z * center_z / (c * c)); // 地面椭圆的半径系数
    const float sqrt2 = 1.41421356237f;

    const int front_idx = 0;
    const int right_idx = (camera_num > 1) ? 1 : 0;
    const int rear_idx  = (camera_num > 2) ? 2 : 0;
    const int left_idx  = (camera_num > 3) ? 3 : 0;

    // 近似车身半长和半宽
    const float half_length = (camera_pos[front_idx].x - camera_pos[rear_idx].x) * 0.5f;
    const float half_width  = (camera_pos[left_idx].y - camera_pos[right_idx].y) * 0.5f;

    // Topview四周预留空间
    const float l_max = (half_length + x_view_scope) * 2.0f; // 为什么要*2.0
    const float w_max = (half_width + y_view_scope) * 2.0f;

    const float denom = sqrt2 * r;
    if (denom != 0.0f) {
        bowl.a = l_max / denom;
        bowl.b = w_max / denom;
    }

    bowl.wall_height = 2.0f * bowl.center_z;
    bowl.ground_length = r * bowl.b - half_width;

    _log_bowl_data(bowl);
    XCAM_LOG_INFO("Topview Lmax = %.2fmm, Wmax = %.2fmm.\n", l_max, w_max);
    return bowl;
}


/* Fixme: 这里硬编码写死了每路相机的视角范围，实际安装角度偏差是否要调整？*/
float *
viewpoints_range (CamModel model, float *range)
{
    switch (model) {
    case CamA2C1080P: {
        range[0] = 202.8f;
        range[1] = 202.8f;
        break;
    }
    case CamB4C1080P: {
        range[0] = 110.0f; //64
        range[1] = 140.0f; //160
        range[2] = 110.0f;
        range[3] = 140.0f; // 160.0f
        break;
    }
    case CamC3C4K: {
        range[0] = 144.0f;
        range[1] = 144.0f;
        range[2] = 144.0f;
        break;
    }
    case CamC3C8K: {
        range[0] = 144.0f;
        range[1] = 144.0f;
        range[2] = 144.0f;
        break;
    }
    case CamC6C8K: {
        range[0] = 72.0f;
        range[1] = 72.0f;
        range[2] = 72.0f;
        range[3] = 72.0f;
        range[4] = 72.0f;
        range[5] = 72.0f;
        break;
    }
    case CamD3C8K: {
        range[0] = 132.0f;
        range[1] = 132.0f;
        range[2] = 132.0f;
        break;
    }
    case CamD6C8K: {
        range[0] = 72.0f;
        range[1] = 72.0f;
        range[2] = 72.0f;
        range[3] = 72.0f;
        range[4] = 72.0f;
        range[5] = 72.0f;
        break;
    }
    default:
        XCAM_LOG_ERROR ("unknown camera model (%d)", model);
        break;
    }

    return range;
}


/* TODO: 特征匹配区域参数没有支持CamB4C1080P? */
FMRegionRatio
fm_region_ratio (CamModel model)
{
    FMRegionRatio ratio;

    switch (model) {
    case CamA2C1080P: {
        ratio.pos_x = 0.0f;
        ratio.width = 1.0f;
        ratio.pos_y = 1.0f / 3.0f;
        ratio.height = 1.0f / 3.0f;
        break;
    }
    case CamC3C4K: {
        ratio.pos_x = 0.0f;
        ratio.width = 1.0f;
        ratio.pos_y = 1.0f / 3.0f;
        ratio.height = 1.0f / 3.0f;
        break;
    }
    case CamC3C8K: {
        ratio.pos_x = 0.0f;
        ratio.width = 1.0f;
        ratio.pos_y = 1.0f / 3.0f;
        ratio.height = 1.0f / 3.0f;
        break;
    }
    case CamC6C8K: {
        ratio.pos_x = 0.0f;
        ratio.width = 1.0f;
        ratio.pos_y = 1.0f / 3.0f;
        ratio.height = 1.0f / 3.0f;
        break;
    }
    case CamD3C8K: {
        ratio.pos_x = 0.0f;
        ratio.width = 1.0f;
        ratio.pos_y = 1.0f / 3.0f;
        ratio.height = 1.0f / 3.0f;
        break;
    }
    case CamD6C8K: {
        ratio.pos_x = 0.0f;
        ratio.width = 1.0f;
        ratio.pos_y = 1.0f / 3.0f;
        ratio.height = 1.0f / 3.0f;
        break;
    }
    default:
        XCAM_LOG_ERROR ("unsupported camera model (%d)", model);
        break;
    }

    return ratio;
}

FMConfig
fm_config (CamModel model)
{
    FMConfig cfg;

    switch (model) {
    case CamA2C1080P: {
        cfg.stitch_min_width = 136;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.9f;
        cfg.delta_mean_offset = 120.0f;
        cfg.recur_offset_error = 8.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 8.0f;
        cfg.max_track_error = 28.0f;
        break;
    }
    case CamB4C1080P: {
        cfg.stitch_min_width = 136;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.8f;
        cfg.delta_mean_offset = 120.0f;
        cfg.recur_offset_error = 8.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 20.0f;
        cfg.max_track_error = 28.0f;
#ifdef ANDROID
        cfg.max_track_error = 3600.0f;
#endif
        break;
    }
    case CamC3C4K: {
        cfg.stitch_min_width = 136;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.95f;
        cfg.delta_mean_offset = 256.0f;
        cfg.recur_offset_error = 4.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 20.0f;
        cfg.max_track_error = 6.0f;
        break;
    }
    case CamC3C8K: {
        cfg.stitch_min_width = 136;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.95f;
        cfg.delta_mean_offset = 256.0f;
        cfg.recur_offset_error = 4.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 20.0f;
        cfg.max_track_error = 6.0f;
        break;
    }
    case CamC6C8K: {
        cfg.stitch_min_width = 136;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.95f;
        cfg.delta_mean_offset = 256.0f;
        cfg.recur_offset_error = 4.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 20.0f;
        cfg.max_track_error = 6.0f;
        break;
    }
    case CamD3C8K: {
        cfg.stitch_min_width = 256;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.6f;
        cfg.delta_mean_offset = 256.0f;
        cfg.recur_offset_error = 2.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 32.0f;
        cfg.max_track_error = 10.0f;
        break;
    }
    case CamD6C8K: {
        cfg.stitch_min_width = 256;
        cfg.min_corners = 4;
        cfg.offset_factor = 0.6f;
        cfg.delta_mean_offset = 256.0f;
        cfg.recur_offset_error = 2.0f;
        cfg.max_adjusted_offset = 24.0f;
        cfg.max_valid_offset_y = 32.0f;
        cfg.max_track_error = 10.0f;
        break;
    }
    default:
        XCAM_LOG_ERROR ("unknown camera model (%d)", model);
        break;
    }

    return cfg;
}

XCamReturn
get_fisheye_info (CamModel model, StitchScopicMode scopic_mode, FisheyeInfo* fisheye_info)
{
    XCamReturn ret = XCAM_RETURN_BYPASS;

#if HAVE_JSON
    CalibrationParser parser;
    StitchInfo info;

    ret = parser.parse_fisheye_camera_param (camera_calibration_json_names[model], info.fisheye_info, XCAM_STITCH_FISHEYE_MAX_NUM);
    if (XCAM_RETURN_NO_ERROR != ret) {
        return ret;
    }

    switch (scopic_mode) {
    case ScopicMono: {
        for (uint32_t i = 0; i < 6; i++) {
            fisheye_info[i].intrinsic = info.fisheye_info[i].intrinsic;
            fisheye_info[i].extrinsic = info.fisheye_info[i].extrinsic;
            fisheye_info[i].cam_model = info.fisheye_info[i].cam_model;
            for (uint32_t j = 0; j < sizeof (FisheyeInfo::distort_coeff) / sizeof(float); j++) {
                fisheye_info[i].distort_coeff[j] = info.fisheye_info[i].distort_coeff[j];
            }
        }
        break;
    }
    case ScopicStereoLeft: {
        for (uint32_t i = 0; i < 3; i++) {
            fisheye_info[i].intrinsic = info.fisheye_info[2 * i].intrinsic;
            fisheye_info[i].extrinsic = info.fisheye_info[2 * i].extrinsic;
            for (uint32_t j = 0; j < sizeof (FisheyeInfo::distort_coeff) / sizeof(float); j++) {
                fisheye_info[i].distort_coeff[j] = info.fisheye_info[2 * i].distort_coeff[j];
            }
        }
        break;
    }
    case ScopicStereoRight: {
        for (uint32_t i = 0; i < 3; i++) {
            fisheye_info[i].intrinsic = info.fisheye_info[2 * i + 1].intrinsic;
            fisheye_info[i].extrinsic = info.fisheye_info[2 * i + 1].extrinsic;
            for (uint32_t j = 0; j < sizeof (FisheyeInfo::distort_coeff) / sizeof(float); j++) {
                fisheye_info[i].distort_coeff[j] = info.fisheye_info[2 * i + 1].distort_coeff[j];
            }
        }
        break;
    }
    default: {
        XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
        break;
    }
    }
#endif
    return ret;
}


/* 仅用于球面贴图场景。Bowl不用该函数。 */
StitchInfo
stitch_info (CamModel model, StitchScopicMode scopic_mode)
{
    StitchInfo info;

    switch (model) {
    case CamA2C1080P: {
        info.fisheye_info[0].intrinsic.cx = 480.0f;
        info.fisheye_info[0].intrinsic.cy = 480.0f;
        info.fisheye_info[0].intrinsic.fov = 202.8f;
        info.fisheye_info[0].radius = 480.0f;
        info.fisheye_info[0].extrinsic.roll = -90.0f;
        info.fisheye_info[1].intrinsic.cx = 1436.0f;
        info.fisheye_info[1].intrinsic.cy = 480.0f;
        info.fisheye_info[1].intrinsic.fov = 202.8f;
        info.fisheye_info[1].radius = 480.0f;
        info.fisheye_info[1].extrinsic.roll = 89.7f;
        break;
    }
    case CamC3C4K: {
        switch (scopic_mode) {
        case ScopicStereoLeft: {
            info.merge_width[0] = 256;
            info.merge_width[1] = 256;
            info.merge_width[2] = 256;

            info.fisheye_info[0].intrinsic.cx = 953.5f;
            info.fisheye_info[0].intrinsic.cy = 720.0f;
            info.fisheye_info[0].intrinsic.fov = 200.0f;
            info.fisheye_info[0].radius = 992.0f;
            info.fisheye_info[0].extrinsic.roll = 90.3f;
            info.fisheye_info[1].intrinsic.cx = 960.0f;
            info.fisheye_info[1].intrinsic.cy = 720.0f;
            info.fisheye_info[1].intrinsic.fov = 200.0f;
            info.fisheye_info[1].radius = 992.0f;
            info.fisheye_info[1].extrinsic.roll = 90.2f;
            info.fisheye_info[2].intrinsic.cx = 960.0f;
            info.fisheye_info[2].intrinsic.cy = 720.0f;
            info.fisheye_info[2].intrinsic.fov = 200.0f;
            info.fisheye_info[2].radius = 992.0f;
            info.fisheye_info[2].extrinsic.roll = 91.2f;
            break;
        }
        case ScopicStereoRight: {
            info.merge_width[0] = 256;
            info.merge_width[1] = 256;
            info.merge_width[2] = 256;

            info.fisheye_info[0].intrinsic.cx = 960.0f;
            info.fisheye_info[0].intrinsic.cy = 720.0f;
            info.fisheye_info[0].intrinsic.fov = 200.0f;
            info.fisheye_info[0].radius = 992.0f;
            info.fisheye_info[0].extrinsic.roll = 90.0f;
            info.fisheye_info[1].intrinsic.cx = 960.0f;
            info.fisheye_info[1].intrinsic.cy = 720.0f;
            info.fisheye_info[1].intrinsic.fov = 200.0f;
            info.fisheye_info[1].radius = 992.0f;
            info.fisheye_info[1].extrinsic.roll = 90.0f;
            info.fisheye_info[2].intrinsic.cx = 957.0f;
            info.fisheye_info[2].intrinsic.cy = 720.0f;
            info.fisheye_info[2].intrinsic.fov = 200.0f;
            info.fisheye_info[2].radius = 992.0f;
            info.fisheye_info[2].extrinsic.roll = 90.1f;
            break;
        }
        default:
            XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
            break;
        }
        break;
    }
    case CamC3C8K: {
        switch (scopic_mode) {
        case ScopicStereoLeft: {
            info.merge_width[0] = 256;
            info.merge_width[1] = 256;
            info.merge_width[2] = 256;

            info.fisheye_info[0].intrinsic.cx = 1907.0f;
            info.fisheye_info[0].intrinsic.cy = 1440.0f;
            info.fisheye_info[0].intrinsic.fov = 200.0f;
            info.fisheye_info[0].radius = 1984.0f;
            info.fisheye_info[0].extrinsic.roll = 90.3f;
            info.fisheye_info[1].intrinsic.cx = 1920.0f;
            info.fisheye_info[1].intrinsic.cy = 1440.0f;
            info.fisheye_info[1].intrinsic.fov = 200.0f;
            info.fisheye_info[1].radius = 1984.0f;
            info.fisheye_info[1].extrinsic.roll = 90.2f;
            info.fisheye_info[2].intrinsic.cx = 1920.0f;
            info.fisheye_info[2].intrinsic.cy = 1440.0f;
            info.fisheye_info[2].intrinsic.fov = 200.0f;
            info.fisheye_info[2].radius = 1984.0f;
            info.fisheye_info[2].extrinsic.roll = 91.2f;
            break;
        }
        case ScopicStereoRight: {
            info.merge_width[0] = 256;
            info.merge_width[1] = 256;
            info.merge_width[2] = 256;

            info.fisheye_info[0].intrinsic.cx = 1920.0f;
            info.fisheye_info[0].intrinsic.cy = 1440.0f;
            info.fisheye_info[0].intrinsic.fov = 200.0f;
            info.fisheye_info[0].radius = 1984.0f;
            info.fisheye_info[0].extrinsic.roll = 90.0f;
            info.fisheye_info[1].intrinsic.cx = 1920.0f;
            info.fisheye_info[1].intrinsic.cy = 1440.0f;
            info.fisheye_info[1].intrinsic.fov = 200.0f;
            info.fisheye_info[1].radius = 1984.0f;
            info.fisheye_info[1].extrinsic.roll = 90.0f;
            info.fisheye_info[2].intrinsic.cx = 1914.0f;
            info.fisheye_info[2].intrinsic.cy = 1440.0f;
            info.fisheye_info[2].intrinsic.fov = 200.0f;
            info.fisheye_info[2].radius = 1984.0f;
            info.fisheye_info[2].extrinsic.roll = 90.1f;
            break;
        }
        default:
            XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
            break;
        }
        break;
    }
    case CamC6C8K: {
        info.merge_width[0] = 256;
        info.merge_width[1] = 256;
        info.merge_width[2] = 256;
        info.merge_width[3] = 256;
        info.merge_width[4] = 256;
        info.merge_width[5] = 256;

        info.fisheye_info[0].intrinsic.cx = 1907.0f;
        info.fisheye_info[0].intrinsic.cy = 1440.0f;
        info.fisheye_info[0].intrinsic.fov = 200.0f;
        info.fisheye_info[0].radius = 1984.0f;
        info.fisheye_info[0].extrinsic.roll = 90.3f;
        info.fisheye_info[1].intrinsic.cx = 1920.0f;
        info.fisheye_info[1].intrinsic.cy = 1440.0f;
        info.fisheye_info[1].intrinsic.fov = 200.0f;
        info.fisheye_info[1].radius = 1984.0f;
        info.fisheye_info[1].extrinsic.roll = 90.0f;
        info.fisheye_info[2].intrinsic.cx = 1920.0f;
        info.fisheye_info[2].intrinsic.cy = 1440.0f;
        info.fisheye_info[2].intrinsic.fov = 200.0f;
        info.fisheye_info[2].radius = 1984.0f;
        info.fisheye_info[2].extrinsic.roll = 90.2f;
        info.fisheye_info[3].intrinsic.cx = 1920.0f;
        info.fisheye_info[3].intrinsic.cy = 1440.0f;
        info.fisheye_info[3].intrinsic.fov = 200.0f;
        info.fisheye_info[3].radius = 1984.0f;
        info.fisheye_info[3].extrinsic.roll = 90.0f;
        info.fisheye_info[4].intrinsic.cx = 1920.0f;
        info.fisheye_info[4].intrinsic.cy = 1440.0f;
        info.fisheye_info[4].intrinsic.fov = 200.0f;
        info.fisheye_info[4].radius = 1984.0f;
        info.fisheye_info[4].extrinsic.roll = 91.2f;
        info.fisheye_info[5].intrinsic.cx = 1914.0f;
        info.fisheye_info[5].intrinsic.cy = 1440.0f;
        info.fisheye_info[5].intrinsic.fov = 200.0f;
        info.fisheye_info[5].radius = 1984.0f;
        info.fisheye_info[5].extrinsic.roll = 90.1f;
        break;
    }
    case CamD3C8K: {
        switch (scopic_mode) {
        case ScopicStereoLeft: {
            info.merge_width[0] = 192;
            info.merge_width[1] = 192;
            info.merge_width[2] = 192;
            info.fisheye_info[0].intrinsic.cx = 1804.0f;
            info.fisheye_info[0].intrinsic.cy = 1532.0f;
            info.fisheye_info[0].intrinsic.fov = 190.0f;
            info.fisheye_info[0].radius = 1900.0f;
            info.fisheye_info[0].extrinsic.roll = 91.5f;
            info.fisheye_info[1].intrinsic.cx = 1836.0f;
            info.fisheye_info[1].intrinsic.cy = 1532.0f;
            info.fisheye_info[1].intrinsic.fov = 190.0f;
            info.fisheye_info[1].radius = 1900.0f;
            info.fisheye_info[1].extrinsic.roll = 92.0f;
            info.fisheye_info[2].intrinsic.cx = 1820.0f;
            info.fisheye_info[2].intrinsic.cy = 1532.0f;
            info.fisheye_info[2].intrinsic.fov = 190.0f;
            info.fisheye_info[2].radius = 1900.0f;
            info.fisheye_info[2].extrinsic.roll = 91.0f;
            break;
        }
        case ScopicStereoRight: {
            info.merge_width[0] = 192;
            info.merge_width[1] = 192;
            info.merge_width[2] = 192;
            info.fisheye_info[0].intrinsic.cx = 1836.0f;
            info.fisheye_info[0].intrinsic.cy = 1532.0f;
            info.fisheye_info[0].intrinsic.fov = 190.0f;
            info.fisheye_info[0].radius = 1900.0f;
            info.fisheye_info[0].extrinsic.roll = 88.0f;
            info.fisheye_info[1].intrinsic.cx = 1852.0f;
            info.fisheye_info[1].intrinsic.cy = 1576.0f;
            info.fisheye_info[1].intrinsic.fov = 190.0f;
            info.fisheye_info[1].radius = 1900.0f;
            info.fisheye_info[1].extrinsic.roll = 90.0f;
            info.fisheye_info[2].intrinsic.cx = 1836.0f;
            info.fisheye_info[2].intrinsic.cy = 1532.0f;
            info.fisheye_info[2].intrinsic.fov = 190.0f;
            info.fisheye_info[2].radius = 1900.0f;
            info.fisheye_info[2].extrinsic.roll = 91.0f;
            break;
        }
        default:
            XCAM_LOG_ERROR ("unsupported scopic mode (%d)", scopic_mode);
            break;
        }
        break;
    }
    case CamD6C8K: {
        info.merge_width[0] = 192;
        info.merge_width[1] = 192;
        info.merge_width[2] = 192;
        info.merge_width[3] = 192;
        info.merge_width[4] = 192;
        info.merge_width[5] = 192;

        info.fisheye_info[0].intrinsic.cx = 1786.0f;
        info.fisheye_info[0].intrinsic.cy = 1530.0f;
        info.fisheye_info[0].intrinsic.fov = 190.0f;
        info.fisheye_info[0].radius = 2150.0f;
        info.fisheye_info[0].extrinsic.roll = 89.6f;
        info.fisheye_info[1].intrinsic.cx = 1774.0f;
        info.fisheye_info[1].intrinsic.cy = 1650.0f;
        info.fisheye_info[1].intrinsic.fov = 190.0f;
        info.fisheye_info[1].radius = 2150.0f;
        info.fisheye_info[1].extrinsic.roll = 90.1f;
        info.fisheye_info[2].intrinsic.cx = 1798.0f;
        info.fisheye_info[2].intrinsic.cy = 1500.0f;
        info.fisheye_info[2].intrinsic.fov = 190.0f;
        info.fisheye_info[2].radius = 2170.0f;
        info.fisheye_info[2].extrinsic.roll = 89.6f;
        info.fisheye_info[3].intrinsic.cx = 1790.0f;
        info.fisheye_info[3].intrinsic.cy = 1487.0f;
        info.fisheye_info[3].intrinsic.fov = 190.0f;
        info.fisheye_info[3].radius = 2150.0f;
        info.fisheye_info[3].extrinsic.roll = 91.1f;
        info.fisheye_info[4].intrinsic.cx = 1790.0f;
        info.fisheye_info[4].intrinsic.cy = 1570.0f;
        info.fisheye_info[4].intrinsic.fov = 190.0f;
        info.fisheye_info[4].radius = 2150.0f;
        info.fisheye_info[4].extrinsic.roll = 90.6f;
        info.fisheye_info[5].intrinsic.cx = 1760.0f;
        info.fisheye_info[5].intrinsic.cy = 1600.0f;
        info.fisheye_info[5].intrinsic.fov = 190.0f;
        info.fisheye_info[5].radius = 2150.0f;
        info.fisheye_info[5].extrinsic.roll = 90.3f;
        break;
    }
    default:
        XCAM_LOG_ERROR ("unsupported camera model (%d)", model);
        break;
    }

    return info;
}

}
