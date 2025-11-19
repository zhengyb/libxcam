/*
 * stitcher.cpp - stitcher base
 *
 *  Copyright (c) 2017 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Author: Wind Yuan <feng.yuan@intel.com>
 * Author: Yinhang Liu <yinhangx.liu@intel.com>
 */

// 本文件实现 Stitcher 基类及其配套的几何模型（BowlModel/CubeMapModel），负责
// 拼接流程中的核心几何设置：相机参数初始化、输出切片划分、重叠区与复制区域计算等。
#include "stitcher.h"
#include "xcam_utils.h"
#include "calibration_parser.h"
#include <string>

// angle to position, output range [-180, 180]
#define OUT_WINDOWS_START 0.0f

#define constraint_margin (2 * _alignment_x)

#define XCAM_GL_RESTART_FIXED_INDEX 0xFFFF

#define XCAM_STITCH_NAME_LEN 256
#define XCAM_CAMERA_POSITION_OFFSET_X 2000

#define FISHEYE_CONFIG_ENV_VAR "FISHEYE_CONFIG_PATH"

namespace XCam {

// 合并相邻 CopyArea 的辅助函数：当两个区域来自同一路输入并且在输入/输出上连续时，将其拼接为更大的块。
static inline bool
merge_neighbor_area (
    const Stitcher::CopyArea &current,
    const Stitcher::CopyArea &next,
    Stitcher::CopyArea &merged)
{
    if (current.in_idx == next.in_idx &&
            current.in_area.pos_x + current.in_area.width == next.in_area.pos_x &&
            current.out_area.pos_x + current.out_area.width == next.out_area.pos_x)
    {
        merged = current;
        merged.in_area.pos_x = current.in_area.pos_x;
        merged.in_area.width = current.in_area.width + next.in_area.width;
        merged.out_area.pos_x = current.out_area.pos_x;
        merged.out_area.width = current.out_area.width + next.out_area.width;
        return true;
    }
    return false;
}

// 在需要跨越环形输出边界时，将 CopyArea 拆分为两段：一段落在末尾，另一段回绕至开头。
static inline bool
split_area_by_out (
    const Stitcher::CopyArea &area, const uint32_t round_width,
    Stitcher::CopyArea &split_a,  Stitcher::CopyArea &split_b)
{
    XCAM_ASSERT (area.out_area.pos_x >= 0 && area.out_area.pos_x < (int32_t)round_width);
    XCAM_ASSERT (area.out_area.width > 0 && area.out_area.width < (int32_t)round_width);
    if (area.out_area.pos_x + area.out_area.width > (int32_t)round_width) {
        split_a = area;
        split_a.out_area.width = round_width - area.out_area.pos_x;
        split_a.in_area.width = split_a.out_area.width;

        split_b = area;
        split_b.in_area.pos_x = area.in_area.pos_x + split_a.in_area.width;
        split_b.in_area.width = area.in_area.width - split_a.in_area.width;
        split_b.out_area.pos_x = 0;
        split_b.out_area.width = split_b.in_area.width;
        XCAM_ASSERT (split_b.out_area.width == area.out_area.pos_x + area.out_area.width - (int32_t)round_width);
        return true;

    }
    XCAM_ASSERT (area.out_area.width == area.in_area.width);
    return false;
}

// 构造函数：指定输出图在 X/Y 方向上的对齐要求，初始化 Stitcher 的内部状态。
Stitcher::Stitcher (uint32_t align_x, uint32_t align_y)
    : _alignment_x (align_x)
    , _alignment_y (align_y)
    , _output_width (0)
    , _output_height (0)
    , _out_start_angle (OUT_WINDOWS_START)
    , _camera_num (0)
    , _is_round_view_set (false)
    , _is_overlap_set (false)
    , _is_crop_set (false)
    , _is_center_marked (false)
    , _dewarp_mode (DewarpBowl)
    , _scale_mode (ScaleSingleConst)
    , _fm_mode (FMNone)
    , _fm_status (FMStatusWholeWay)
    , _fm_frames (100)
    , _fm_frame_count (1)
    , _complete_stitch (true)
    , _need_fm (false)
    , _blend_pyr_levels (2)
{
    XCAM_ASSERT (align_x >= 1);
    XCAM_ASSERT (align_y >= 1);

    xcam_mem_clear (_intr_names);
    xcam_mem_clear (_extr_names);
    xcam_mem_clear (_viewpoints_range);
}

// 析构函数：释放 set_intrinsic_names/set_extrinsic_names 分配的字符串。
Stitcher::~Stitcher ()
{
    for (int idx = 0; idx < XCAM_STITCH_MAX_CAMERAS; ++idx) {
        xcam_free (_intr_names[idx]);
        xcam_free (_extr_names[idx]);
    }
}

// 设置碗面模型参数，供 Bowl 去畸变及顶视图生成使用。
bool
Stitcher::set_bowl_config (const BowlDataConfig &config)
{
    _bowl_config = config;
    return true;
}

// 设置相机数量，需在配置其它相机相关信息前调用。
bool
Stitcher::set_camera_num (uint32_t num)
{
    XCAM_FAIL_RETURN (
        ERROR, num <= XCAM_STITCH_MAX_CAMERAS, false,
        "stitcher: set camera count failed, num(%d) is larger than max value(%d)",
        num, XCAM_STITCH_MAX_CAMERAS);
    _camera_num = num;
    return true;
}

// 写入单路相机的 CalibrationInfo（内外参、角度范围等）。
bool
Stitcher::set_camera_info (uint32_t index, const CameraInfo &info)
{
    XCAM_FAIL_RETURN (
        ERROR, index < _camera_num, false,
        "stitcher: set camera info failed, index(%d) exceed max camera num(%d)",
        index, _camera_num);
    _camera_info[index] = info;
    return true;
}

// 设定输入裁剪窗口：拼接时仅使用有效画面区域。
bool
Stitcher::set_crop_info (uint32_t index, const ImageCropInfo &info)
{
    XCAM_FAIL_RETURN (
        ERROR, index < _camera_num, false,
        "stitcher: set camera info failed, index(%d) exceed max camera num(%d)",
        index, _camera_num);
    _crop_info[index] = info;
    _is_crop_set = true;
    return true;
}

// 读取指定相机的裁剪信息，若尚未设置将返回默认的零裁剪。
bool
Stitcher::get_crop_info (uint32_t index, ImageCropInfo &info) const
{
    XCAM_FAIL_RETURN (
        ERROR, index < _camera_num, false,
        "stitcher: get crop info failed, index(%d) exceed camera num(%d)",
        index, _camera_num);
    info = _crop_info[index];
    return true;
}

// 配置特征匹配所裁剪的重叠区域比例，取值需在 [0,1] 范围。
void
Stitcher::set_fm_region_ratio (const FMRegionRatio &ratio)
{
    if (ratio.pos_x < 0.0f || ratio.width < 0.0f || ratio.pos_y < 0.0f || ratio.height < 0.0f ||
            (ratio.pos_x + ratio.width) > 1.0f || (ratio.pos_y + ratio.height) > 1.0f) {
        XCAM_LOG_ERROR (
            "invalid FM region ratio (%f, %f, %f, %f)",
            ratio.pos_x, ratio.width, ratio.pos_y, ratio.height);
        XCAM_ASSERT (false);
    }

    _fm_region_ratio = ratio;
}

// 更新特征匹配执行状态：根据帧计数、模式决定是否需要运行匹配以及是否允许写出结果。
bool
Stitcher::ensure_stitch_path ()
{
    if (_fm_frame_count > _fm_frames + 1)
        return true;

    _complete_stitch = (
                           _fm_mode == FMNone || _fm_status != FMStatusFMFirst || _fm_frame_count > _fm_frames);

    _need_fm = (
                   _fm_mode != FMNone && (_fm_status == FMStatusWholeWay || _fm_frame_count <= _fm_frames));

    _fm_frame_count++;

    return true;
}

#if 0
bool
Stitcher::set_overlap_info (uint32_t index, const ImageOverlapInfo &info)
{
    XCAM_FAIL_RETURN (
        ERROR, index < _camera_num, false,
        "stitcher: set overlap info failed, index(%d) exceed max camera num(%d)",
        index, _camera_num);
    _overlap_info[index] = info;
    _is_overlap_set = true;
    return true;
}

bool
Stitcher::get_overlap_info (uint32_t index, ImageOverlapInfo &info) const
{
    XCAM_FAIL_RETURN (
        ERROR, index < _camera_num, false,
        "stitcher: get overlap info failed, index(%d) exceed camera num(%d)",
        index, _camera_num);
    info = _overlap_info[index];
    return true;
}
#endif

// 查询指定索引的相机信息，通常供后端在生成 GeoMap/Table 时使用。
bool
Stitcher::get_camera_info (uint32_t index, CameraInfo &info) const
{
    XCAM_FAIL_RETURN (
        ERROR, index < XCAM_STITCH_MAX_CAMERAS, false,
        "stitcher: get camera info failed, index(%d) exceed max camera value(%d)",
        index, XCAM_STITCH_MAX_CAMERAS);
    info = _camera_info[index];
    return true;
}

// 设定各路相机的水平视角范围（以角度为单位），通常来源于配置文件。
bool
Stitcher::set_viewpoints_range (const float *range)
{
    XCAM_FAIL_RETURN (
        ERROR, _camera_num, false,
        "stitcher: set viewpoints range failed, please set camera num(%d) first", _camera_num);

    for(uint32_t i = 0; i < _camera_num; ++i) {
        _viewpoints_range[i] = range[i];
    }

    return true;
}

// 指定各路相机内参文本文件名称，后续 init_camera_info 会在 FISHEYE_CONFIG_PATH 下搜索。
bool
Stitcher::set_intrinsic_names (const char *intr_names[])
{
    XCAM_FAIL_RETURN (
        ERROR, _camera_num, false,
        "stitcher: set intrinsic names failed, please set camera num(%d) first", _camera_num);

    for(uint32_t i = 0; i < _camera_num; ++i) {
        _intr_names[i] = strndup (intr_names[i], XCAM_MAX_STR_SIZE);
    }

    return true;
}

// 指定各路相机外参文本文件名称。
bool
Stitcher::set_extrinsic_names (const char *extr_names[])
{
    XCAM_FAIL_RETURN (
        ERROR, _camera_num, false,
        "stitcher: set extrinsic names failed, please set camera num(%d) first", _camera_num);

    for(uint32_t i = 0; i < _camera_num; ++i) {
        _extr_names[i] = strndup (extr_names[i], XCAM_MAX_STR_SIZE);
    }

    return true;
}

/*
 * 根据去畸变模式初始化每路相机的角度范围与外参：
 *  - Sphere 模式：直接使用 set_viewpoints_range 提供的角度，round_angle_start 依据相机序号均匀分布。
 *  - Bowl 模式：从 FISHEYE_CONFIG_PATH 指定目录读取 intrinsic/extrinsic 文本，填充 CameraInfo.calibration，
 *    并对外参做平移归一化，以方便碗面坐标系的统一处理。
 */
XCamReturn
Stitcher::init_camera_info ()
{
    if (_dewarp_mode == DewarpSphere) {
        for (uint32_t i = 0; i < _camera_num; ++i) {
            CameraInfo &info = _camera_info[i];
            info.angle_range = _viewpoints_range[i];
            info.round_angle_start = (i * 360.0f / _camera_num) - info.angle_range / 2.0f;
        }
    } else {
        const char *env = std::getenv (FISHEYE_CONFIG_ENV_VAR);
        std::string path (env, (env ? strlen (env) : 0));
        XCAM_FAIL_RETURN (
            ERROR, !path.empty (), XCAM_RETURN_ERROR_PARAM,
            "FISHEYE_CONFIG_PATH is empty, export FISHEYE_CONFIG_PATH first");
        XCAM_LOG_INFO ("stitcher calibration config path: %s", path.c_str ());

        CalibrationParser parser;
        char pathname[XCAM_STITCH_NAME_LEN] = {'\0'};
        for (uint32_t i = 0; i < _camera_num; ++i) {
            CameraInfo &info = _camera_info[i];

            snprintf (pathname, XCAM_STITCH_NAME_LEN, "%s/%s", path.c_str (), _intr_names[i]);
            XCamReturn ret = parser.parse_intrinsic_file (pathname, info.calibration.intrinsic);
            XCAM_FAIL_RETURN (
                ERROR, ret == XCAM_RETURN_NO_ERROR, XCAM_RETURN_ERROR_PARAM,
                "stitcher parse intrinsic params(%s) failed", pathname);

            snprintf (pathname, XCAM_STITCH_NAME_LEN, "%s/%s", path.c_str (), _extr_names[i]);
            ret = parser.parse_extrinsic_file (pathname, info.calibration.extrinsic);
            XCAM_FAIL_RETURN (
                ERROR, ret == XCAM_RETURN_NO_ERROR, XCAM_RETURN_ERROR_PARAM,
                "stitcher parse extrinsic params(%s) failed", pathname);

            info.calibration.extrinsic.trans_x += XCAM_CAMERA_POSITION_OFFSET_X;

            info.angle_range = _viewpoints_range[i];
            info.round_angle_start = (i * 360.0f / _camera_num) - info.angle_range / 2.0f;
        }

        centralize_bowl_coord_from_cameras (
            _camera_info[0].calibration.extrinsic, _camera_info[1].calibration.extrinsic,
            _camera_info[2].calibration.extrinsic, _camera_info[3].calibration.extrinsic);
    }

    return XCAM_RETURN_NO_ERROR;
}

XCamReturn
Stitcher::estimate_round_slices ()
{
    /*
     * 功能：根据每路相机的水平视场角（angle_range）以及希望生成的输出分辨率，
     *       计算在环形输出图上对应的切片宽度、角度范围以及起始角度。Soft/GLES/Vulkan
     *       后端都会在生成 GeoMapTable 之前调用该函数。
     *
     * 使用注意事项：
     *  0. 该项目认为每路相机的角度范围都是对称的。由于鱼眼相机在边缘在矫正后较为模糊，如果相机安装的Yaw角较大，这种对称设计可能会让拼接效果不好。
     *  1. 需要在调用之前正确设置 _camera_num、_output_width/_output_height、
     *     每路 CameraInfo 的 angle_range 与 round_angle_start（例如通过 set_viewpoints_range
     *     与 init_camera_info 完成）。否则将触发检查或得到错误切片。
     *  2. 函数会按照 _alignment_x（通常为 8 或 16 像素）对切片宽度做对齐，
     *     以保证后续缓冲区/几何映射的内存对齐要求，因此理论上的角度范围会和输入值有微小差异。
     *  3. 若某路切片起点落在全景图边缘附近（constraint_margin），为了避免超出范围，
     *     会强制把起点设置为 0°，这样可以确保首尾拼接时不会出现空洞。
     *  4. 该函数仅对尚未设置过 round view 的 Stitcher 执行一次；若需要重新计算（比如
     *     输出分辨率发生变化），必须先清除 _is_round_view_set 标志。
     *
     * TODO：
     *  1. start_angle和end_angle改为非对称。
     */

    if (_is_round_view_set)
        return XCAM_RETURN_NO_ERROR;

    XCAM_FAIL_RETURN (
        ERROR, _camera_num && _camera_num <= XCAM_STITCH_MAX_CAMERAS, XCAM_RETURN_ERROR_PARAM,
        "stitcher: camera num was not set, or camera num(%d) exceed max camera value(%d)",
        _camera_num, XCAM_STITCH_MAX_CAMERAS);

    for (uint32_t i = 0; i < _camera_num; ++i) {
        CameraInfo &cam_info = _camera_info[i];
        RoundViewSlice &view_slice = _round_view_slices[i];

        /* _output_width和_output_height分别是平面全景图的宽度和高度分辨率？ */
        view_slice.width = cam_info.angle_range / 360.0f * (float)_output_width;
        view_slice.width = XCAM_ALIGN_UP (view_slice.width, _alignment_x);
        view_slice.height = _output_height;
        view_slice.hori_angle_range = view_slice.width * 360.0f / (float)_output_width; // Because of ALIGN UP op, view_slice.hori_angle_range ~= cam_info.angle_range !!

        uint32_t aligned_start = format_angle (cam_info.round_angle_start) / 360.0f * (float)_output_width;
        aligned_start = XCAM_ALIGN_AROUND (aligned_start, _alignment_x);

        /* 如果相机对应的切片起点很靠近全景图两侧边缘，则将起点设置为全景图起点 */
        if (_output_width <= constraint_margin + aligned_start || aligned_start <= constraint_margin)
            aligned_start = 0;
        view_slice.hori_angle_start = format_angle((float)aligned_start / (float)_output_width * 360.0f);
        if (XCAM_DOUBLE_EQUAL_AROUND (view_slice.hori_angle_start, 0.0001f))
            view_slice.hori_angle_start = 0.0f;

        cam_info.round_angle_start = view_slice.hori_angle_start;
        cam_info.angle_range = view_slice.hori_angle_range;
    }

    _is_round_view_set = true;
    return XCAM_RETURN_NO_ERROR;
}

// 如果外部没有手动提供 crop 信息，则初始化为全幅图像。
XCamReturn
Stitcher::estimate_coarse_crops ()
{
    if (_is_crop_set)
        return XCAM_RETURN_NO_ERROR;

    XCAM_FAIL_RETURN (
        ERROR, _camera_num > 0 && _is_round_view_set, XCAM_RETURN_ERROR_ORDER,
        "stitcher mark_centers failed, need set camera info and round_slices first");

    for (uint32_t i = 0; i < _camera_num; ++i) {
        _crop_info[i].left = 0;
        _crop_info[i].right = 0;
        _crop_info[i].top = 0;
        _crop_info[i].bottom = 0;
    }
    _is_crop_set = true;
    return XCAM_RETURN_NO_ERROR;
}

// 在裁剪信息就绪后，基于 round view 切片计算各路输出中心点位置，供后续拼接和 copy 区使用。
XCamReturn
Stitcher::mark_centers ()
{
    if (_is_center_marked)
        return XCAM_RETURN_NO_ERROR;

    XCAM_FAIL_RETURN (
        ERROR, _camera_num > 0 && _is_round_view_set, XCAM_RETURN_ERROR_ORDER,
        "stitcher mark_centers failed, need set camera info and round_view slices first");

    for (uint32_t i = 0; i < _camera_num; ++i) {
        const RoundViewSlice &slice = _round_view_slices[i];

        //calcuate final output postion
        float center_angle = i * 360.0f / _camera_num;
        uint32_t out_pos = format_angle (center_angle - _out_start_angle) / 360.0f * _output_width;
        XCAM_ASSERT (out_pos < _output_width);
        if (_output_width <= constraint_margin + out_pos || out_pos <= constraint_margin)
            out_pos = 0;

        // get slice center angle
        center_angle = XCAM_ALIGN_AROUND (out_pos, _alignment_x) / (float)_output_width * 360.0f - _out_start_angle;
        center_angle = format_angle (center_angle);

        float center_in_slice = center_angle - slice.hori_angle_start;
        center_in_slice = format_angle (center_in_slice);
        XCAM_FAIL_RETURN (
            ERROR, center_in_slice < slice.hori_angle_range,
            XCAM_RETURN_ERROR_PARAM,
            "stitcher mark center failed, slice:%d  calculated center-angle:%.2f is out of slice angle(start:%.2f, range:%.2f)",
            i, center_angle, slice.hori_angle_start, slice.hori_angle_range);

        uint32_t slice_pos = (uint32_t)(center_in_slice / slice.hori_angle_range * slice.width);
        slice_pos = XCAM_ALIGN_AROUND (slice_pos, _alignment_x);
        XCAM_ASSERT (slice_pos > _crop_info[i].left && slice_pos < slice.width - _crop_info[i].right);

        _center_marks[i].slice_center_x = slice_pos;
        _center_marks[i].out_center_x = out_pos;
    }
    _is_center_marked = true;

    return XCAM_RETURN_NO_ERROR;
}

// 依据中心位置和裁剪结果估算相邻相机之间的重叠窗口（包括输入子区域与输出窗口）。
/*
 * 函数作用：
 *   根据 round view 切片、裁剪信息与中心点，估算相邻两路相机在输入/输出平面上的重叠区域，
 *   将结果写入 _overlap_info[idx]。这些矩形后续会被 Soft/GLES/Vulkan 后端用来执行特征匹配、
 *   融合或直接复制。
 *
 * 使用前置条件：
 *   - 已调用 estimate_round_slices()，设置了切片宽度/角度；
 *   - 已调用 estimate_coarse_crops()，准备好裁剪参数；
 *   - 已调用 mark_centers()，计算好每路切片在输出/输入中的中心位置。
 *   若上述任一未完成，函数会直接返回 XCAM_RETURN_ERROR_ORDER。
 *
 * 计算步骤概述：
 *   1. 对每对相邻相机（左 idx、右 next_idx）：
 *      - 从 CenterMark、CropInfo 获取各自的中心位置与有效裁剪区域；
 *      - 计算两中心在输出上的距离 merge_width；
 *      - 验证左右有效宽度之和是否大于 merge_width，若不是则说明两图无重叠（返回错误）；
 *      - 依据宽度差得到真正的 overlap_width；
 *      - 切出左侧重叠区（valid_left_img 的靠右部份）、右侧重叠区（valid_right_img 的靠左部份）；
 *      - 生成输出平面中的重叠窗口（out_overlap），与输入宽度一致；
 *      - 将 `left/right/out_area` 写入 _overlap_info[idx]。
 *   2. 所有相邻对处理完毕后，设置 _is_overlap_set = true，避免重复计算。
 *
 * 注意事项：
 *   - 若输出宽度设得过小，导致 merge_width >= valid_left + valid_right，会触发错误；
 *   - 若某相机被裁剪过多，也可能导致不满足重叠条件；
 *   - 输出区域目前仅用于水平位置，垂直坐标在 Soft 后端补偿时仍会使用。
 */
XCamReturn
Stitcher::estimate_overlap ()
{
    if (_is_overlap_set)
        return XCAM_RETURN_NO_ERROR;

    XCAM_FAIL_RETURN (
        ERROR, _is_round_view_set && _is_crop_set && _is_center_marked, XCAM_RETURN_ERROR_ORDER,
        "stitcher estimate_coarse_seam failed, need set round_view slices, crop info and mark centers first");

    for (uint32_t idx = 0; idx < _camera_num; ++idx) {
        uint32_t next_idx = (idx + 1) % _camera_num;
        const RoundViewSlice &left = _round_view_slices[idx];
        const RoundViewSlice &right = _round_view_slices[next_idx];
        const CenterMark &left_center = _center_marks[idx];
        const CenterMark &right_center = _center_marks[next_idx];
        const ImageCropInfo &left_img_crop = _crop_info[idx];
        const ImageCropInfo &right_img_crop = _crop_info[next_idx];

#if 0
        XCAM_FAIL_RETURN (
            ERROR,
            (format_angle (right.hori_angle_start - left.hori_angle_start) < left.hori_angle_range)
            XCAM_RETURN_ERROR_UNKNOWN,
            "stitcher estimate_coarse_seam failed and there is no seam between slice %d and slice %d", idx, next_idx);

        float seam_angle_start = right.hori_angle_start;
        float seam_angle_range =
            format_angle (left.hori_angle_start + left.hori_angle_range - right.hori_angle_start);

        XCAM_FAIL_RETURN (
            ERROR, seam_angle_range < right.hori_angle_range, XCAM_RETURN_ERROR_UNKNOWN,
            "stitcher estimate_coarse_seam failed and left slice(%d)over covered right slice(%d)", idx, next_idx);

        XCAM_ASSERT (!XCAM_DOUBLE_EQUAL_AROUND (left.hori_angle_range, 0.0f));
        XCAM_ASSERT (!XCAM_DOUBLE_EQUAL_AROUND (right.hori_angle_range, 0.0f));
#endif
        uint32_t out_right_center_x = right_center.out_center_x;
        if (out_right_center_x == 0)
            out_right_center_x = _output_width;

        Rect valid_left_img, valid_right_img;
        valid_left_img.pos_x = left_center.slice_center_x;
        valid_left_img.width = left.width - left_img_crop.right - valid_left_img.pos_x;
        valid_left_img.pos_y = left_img_crop.top;
        valid_left_img.height = left.height - left_img_crop.top - left_img_crop.bottom;

        valid_right_img.width = right_center.slice_center_x - right_img_crop.left;
        valid_right_img.pos_x = right_center.slice_center_x - valid_right_img.width;
        valid_right_img.pos_y = right_img_crop.top;
        valid_right_img.height = right.height - right_img_crop.top - right_img_crop.bottom;

        uint32_t merge_width = out_right_center_x - left_center.out_center_x;
        XCAM_FAIL_RETURN (
            ERROR,
            valid_left_img.width + valid_right_img.width > (int32_t)merge_width,
            XCAM_RETURN_ERROR_UNKNOWN,
            "stitcher estimate_overlap failed and there is no overlap area between slice %d and slice %d", idx, next_idx);

        uint32_t overlap_width = valid_left_img.width + valid_right_img.width - merge_width;

        Rect left_img_overlap, right_img_overlap;
        left_img_overlap.pos_x = valid_left_img.pos_x + valid_left_img.width - overlap_width;
        left_img_overlap.width = overlap_width;
        left_img_overlap.pos_y = valid_left_img.pos_y;
        left_img_overlap.height = valid_left_img.height;
        XCAM_ASSERT (left_img_overlap.pos_x >= (int32_t)left_center.slice_center_x &&  left_img_overlap.pos_x < (int32_t)left.width);

        right_img_overlap.pos_x = valid_right_img.pos_x;
        right_img_overlap.width = overlap_width;
        right_img_overlap.pos_y = valid_right_img.pos_y;
        right_img_overlap.height = valid_right_img.height;
        XCAM_ASSERT (right_img_overlap.pos_x >= (int32_t)right_img_crop.left && right_img_overlap.pos_x < (int32_t)right_center.slice_center_x);

        Rect out_overlap;
        out_overlap.pos_x = left_center.out_center_x + valid_left_img.width - overlap_width;
        out_overlap.width = overlap_width;
        // out_overlap.pos_y/height not useful by now
        out_overlap.pos_y = valid_left_img.pos_y;
        out_overlap.height = valid_left_img.height;

#if 0
        left_img_seam.pos_x =
            left.width * format_angle (seam_angle_start - left.hori_angle_start) / left.hori_angle_range;
        left_img_seam.pos_y = _crop_info[idx].top;
        left_img_seam.width = left.width * seam_angle_range / left.hori_angle_range;
        left_img_seam.height = left.height - _crop_info[idx].top - _crop_info[idx].bottom;

        //consider crop
        XCAM_ASSERT (left_img_seam.pos_x <  left.width - _crop_info[idx].right);
        if (left_img_seam.pos_x + left_img_seam.width > left.width - _crop_info[idx].right)
            left_img_seam.width = left.width - _crop_info[idx].right;

        right_img_seam.pos_x = 0;
        right_img_seam.pos_y = _crop_info[next_idx].top;
        right_img_seam.width = right.width * (seam_angle_range / right.hori_angle_range);
        right_img_seam.height = right.height - _crop_info[next_idx].top - _crop_info[next_idx].bottom;

        //consider crop
        XCAM_ASSERT (right_img_seam.pos_x + right_img_seam.width >  _crop_info[next_idx].left);
        if (_crop_info[next_idx].left) {
            right_img_seam.pos_x = _crop_info[next_idx].left;
            right_img_seam.width -= _crop_info[next_idx].left;
            left_img_seam.pos_x += _crop_info[next_idx].left;
            left_img_seam.width -= _crop_info[next_idx].left;
        }

        XCAM_ASSERT (abs (left_img_seam.width - right_img_seam.width) < 16);
        left_img_seam.pos_x = XCAM_ALIGN_DOWN (left_img_seam.pos_x, _alignment_x);
        right_img_seam.pos_x = XCAM_ALIGN_DOWN (right_img_seam.pos_x, _alignment_x);

        //find max seam width
        uint32_t seam_width, seam_height;
        seam_width = XCAM_MAX (left_img_seam.width, right_img_seam.width);
        if (left_img_seam.pos_x + seam_width > left.width)
            seam_width = left.width - left_img_seam.pos_x;
        if (right_img_seam.pos_x + seam_width > right.width)
            seam_width = right.width - right_img_seam.pos_x;

        XCAM_FAIL_RETURN (
            ERROR, seam_width >= XCAM_STITCH_MIN_SEAM_WIDTH, XCAM_RETURN_ERROR_UNKNOWN,
            "stitcher estimate_coarse_seam failed, the seam(w:%d) is very narrow between(slice %d and %d)",
            seam_width, idx, next_idx);
        left_img_seam.width = right_img_seam.width = XCAM_ALIGN_DOWN (seam_width, _alignment_x);

        // min height
        uint32_t top = XCAM_MAX (left_img_seam.pos_y, right_img_seam.pos_y);
        uint32_t bottom0 = left_img_seam.pos_y + left_img_seam.height;
        uint32_t bottom1 = right_img_seam.pos_y + right_img_seam.height;
        uint32_t bottom = XCAM_MIN (bottom0, bottom1);
        top = XCAM_ALIGN_UP (top, _alignment_y);
        left_img_seam.pos_y = right_img_seam.pos_y = top;
        left_img_seam.height = right_img_seam.height = XCAM_ALIGN_DOWN (bottom - top, _alignment_y);
#endif
        // set overlap info
        _overlap_info[idx].left = left_img_overlap;
        _overlap_info[idx].right = right_img_overlap;
        _overlap_info[idx].out_area = out_overlap;
    }

    _is_overlap_set = true;

    return XCAM_RETURN_NO_ERROR;
}

// 基于重叠信息生成 copy 区域列表，供非重叠部分直接复制（避免重复重映射）。
XCamReturn
Stitcher::update_copy_areas ()
{
    XCAM_FAIL_RETURN (
        ERROR, _camera_num > 1 && _is_round_view_set && _is_crop_set && _is_overlap_set, XCAM_RETURN_ERROR_ORDER,
        "stitcher update_copy_areas failed, check orders, need"
        "camera_info, round_view slices, crop_info and overlap_info set first.");

    CopyAreaArray tmp_areas;
    uint32_t i = 0;
    uint32_t next_i = 0;
    for (i = 0; i < _camera_num; ++i) {
        next_i = (i + 1 ) % _camera_num;
        const CenterMark &mark_left = _center_marks[i];
        const CenterMark &mark_right = _center_marks[next_i];
        const ImageOverlapInfo  &overlap = _overlap_info[i];

        CopyArea split_a, split_b;

        CopyArea left;
        left.in_idx = i;
        left.in_area.pos_x = mark_left.slice_center_x;
        left.in_area.width = overlap.left.pos_x - left.in_area.pos_x;
        XCAM_ASSERT (left.in_area.width > 0);
        left.in_area.pos_y = _crop_info[i].top;
        left.in_area.height = _round_view_slices[i].height - _crop_info[i].top - _crop_info[i].bottom;
        XCAM_ASSERT (left.in_area.height > 0);

        left.out_area.pos_x = mark_left.out_center_x;
        left.out_area.width = left.in_area.width;
        left.out_area.pos_y = 0;
        left.out_area.height = left.in_area.height;

        if (split_area_by_out (left, _output_width, split_a, split_b)) {
            tmp_areas.push_back (split_a);
            tmp_areas.push_back (split_b);
        } else {
            tmp_areas.push_back (left);
        }

        CopyArea right;
        right.in_idx = next_i;
        right.in_area.pos_x = _overlap_info[i].right.pos_x + _overlap_info[i].right.width;
        right.in_area.width =  (int32_t)mark_right.slice_center_x - right.in_area.pos_x;
        XCAM_ASSERT (right.in_area.width > 0);
        right.in_area.pos_y = _crop_info[next_i].top;
        right.in_area.height = _round_view_slices[next_i].height - _crop_info[next_i].top - _crop_info[next_i].bottom;
        XCAM_ASSERT (right.in_area.height > 0);

        uint32_t out_right_center_x = mark_right.out_center_x;
        if (out_right_center_x == 0)
            out_right_center_x = _output_width;
        right.out_area.width = right.in_area.width;
        right.out_area.pos_x = out_right_center_x - right.out_area.width;
        right.out_area.pos_y = 0;
        right.out_area.height = right.in_area.height;

        if (split_area_by_out (right, _output_width, split_a, split_b)) {
            tmp_areas.push_back (split_a);
            tmp_areas.push_back (split_b);
        } else {
            tmp_areas.push_back (right);
        }
    }
    XCAM_ASSERT (tmp_areas.size () > _camera_num && _camera_num >= 2);

    CopyArea merged;
    int32_t start = 0;
    int32_t end = tmp_areas.size () - 1;
    if (tmp_areas.size () > 2) {
        const CopyArea &first = tmp_areas[0];
        const CopyArea &last = tmp_areas[end];
        // merge first and last
        if (merge_neighbor_area (last, first, merged)) {
            _copy_areas.push_back (merged);
            ++start;
            --end;
        }
    }

    // merge areas
    for (i = (uint32_t)start; (int32_t)i <= end; ) {
        const CopyArea &current = tmp_areas[i];
        if (i == (uint32_t)end) {
            _copy_areas.push_back (current);
            break;
        }

        const CopyArea &next = tmp_areas[i + 1];
        if (merge_neighbor_area (current, next, merged)) {
            _copy_areas.push_back (merged);
            i += 2;
        } else {
            _copy_areas.push_back (current);
            i += 1;
        }
    }

    XCAM_ASSERT (_copy_areas.size() >= _camera_num);

    return XCAM_RETURN_NO_ERROR;
}

// BowlModel 用于把碗面坐标与原始环视图之间互相映射，供顶视图、碗面渲染和 GeoMapper 使用。
BowlModel::BowlModel (const BowlDataConfig &config, const uint32_t image_width, const uint32_t image_height)
    : _config (config)
    , _bowl_img_width (image_width)
    , _bowl_img_height (image_height)
{
    //max area => x/a = y/b
    XCAM_ASSERT (fabs(_config.center_z) < _config.c);
    float mid = sqrt ((1.0f - _config.center_z * _config.center_z / (_config.c * _config.c)) / 2.0f);
    _max_topview_length_mm = mid * _config.a * 2.0f;
    _max_topview_width_mm = mid * _config.b * 2.0f;
}

// 返回可生成的最大顶视图覆盖范围（毫米单位），便于调用方判断裁剪大小。
bool
BowlModel::get_max_topview_area_mm (float &length_mm, float &width_mm)
{
    if (_max_topview_width_mm <= 0.0f || _max_topview_length_mm <= 0.0f)
        return false;
    length_mm = _max_topview_length_mm;
    width_mm = _max_topview_width_mm;
    return true;
}

// 生成顶视图到 ERP 碗面图的查找表：输入顶视图分辨率以及希望覆盖的物理尺寸。
// 依据碗面模型生成顶视图到原始 ERP 图的查找表：输入顶视输出分辨率以及希望覆盖的物理尺寸。
bool
BowlModel::get_topview_rect_map (
    PointMap &texture_points,
    uint32_t res_width, uint32_t res_height,
    float length_mm, float width_mm)
{
    // 若未指定覆盖尺寸，则默认使用模型可支持的最大顶视范围。
    if (XCAM_DOUBLE_EQUAL_AROUND (length_mm, 0.0f) ||
            XCAM_DOUBLE_EQUAL_AROUND (width_mm, 0.0f)) {
        get_max_topview_area_mm (length_mm, width_mm);
    }

    // 检查输入区域是否超过椭球表面可投影的最大范围，避免无效坐标。
    XCAM_FAIL_RETURN (
        ERROR,
        length_mm * length_mm / (_config.a * _config.a) / 4.0f + width_mm * width_mm / (_config.b * _config.b) / 4.0f +
        _config.center_z * _config.center_z / (_config.c * _config.c) <= 1.0f + 0.001f,
        false,
        "bowl model topview input area(L:%.2fmm, W:%.2fmm) is larger than max area", length_mm, width_mm);

    float center_pos_x = res_width / 2.0f;    // 顶视图像素坐标系的中心点
    float center_pos_y = res_height / 2.0f;
    float mm_per_pixel_x = length_mm / res_width;   // 每个像素对应的实际长度（毫米）
    float mm_per_pixel_y = width_mm / res_height;

    texture_points.resize (res_width * res_height);

    for(uint32_t row = 0; row < res_height; row++) {
        for(uint32_t col = 0; col < res_width; col++) {
            // 将顶视图像素转换为碗面上的三维坐标（以车辆中心为原点）。
            PointFloat3 world_pos (
                (col - center_pos_x) * mm_per_pixel_x,
                (center_pos_y - row) * mm_per_pixel_y,
                0.0f);

            // 将世界坐标映射回原始碗面图上的纹理坐标，供 GeoMapper 查表使用。
            PointFloat2 texture_pos = bowl_view_coords_to_image (
                                          _config, world_pos, _bowl_img_width, _bowl_img_height);

            texture_points [res_width * row + col] = texture_pos;
        }
    }
    return true;
}

// 构建用于渲染的顶点/纹理坐标数据，将碗面图像映射到立体几何上。
bool
BowlModel::get_stitch_image_vertex_model (
    VertexMap &vertices, PointMap &texture_points, IndexVector &indeices,
    uint32_t res_width, uint32_t res_height, float vertex_height)
{
    vertices.reserve (2 * (res_width + 1) * (res_height + 1));
    texture_points.reserve (2 * (res_width + 1) * (res_height + 1));
    indeices.reserve (2 * (res_width + 1) * (res_height + 1) + (res_height + 1));

    float step_x = (float)_bowl_img_width / res_width;
    float step_y = vertex_height / res_height;
    float offset_y = (float)_bowl_img_height - vertex_height;

    int32_t indicator = 0;

    for (uint32_t row = 0; row < res_height - 1; row++) {
        PointFloat2 texture_pos0;
        texture_pos0.y = row * step_y + offset_y;

        PointFloat2 texture_pos1;
        texture_pos1.y = (row + 1) * step_y + offset_y;

        for (uint32_t col = 0; col <= res_width; col++) {

            texture_pos0.x = col * step_x;
            texture_pos1.x = col * step_x;

            PointFloat3 world_pos0 =
                bowl_view_image_to_world (
                    _config, _bowl_img_width, _bowl_img_height, texture_pos0);

            vertices.push_back (PointFloat3(world_pos0.x / _config.a, world_pos0.y / _config.b, world_pos0.z / _config.c));
            indeices.push_back (indicator++);
            texture_points.push_back (PointFloat2(texture_pos0.x / _bowl_img_width, texture_pos0.y / _bowl_img_height));

            PointFloat3 world_pos1 =
                bowl_view_image_to_world (
                    _config, _bowl_img_width, _bowl_img_height, texture_pos1);

            vertices.push_back (PointFloat3(world_pos1.x / _config.a, world_pos1.y / _config.b, world_pos1.z / _config.c));
            indeices.push_back (indicator++);
            texture_points.push_back (PointFloat2(texture_pos1.x / _bowl_img_width, texture_pos1.y / _bowl_img_height));
        }
    }
    return true;
}


// 获取完整碗面高度的网格数据，用于碗面渲染。
bool
BowlModel::get_bowlview_vertex_model (
    VertexMap &vertices, PointMap &texture_points, IndexVector &indeices,
    uint32_t res_width, uint32_t res_height)
{
    return get_stitch_image_vertex_model (vertices, texture_points, indeices, res_width, res_height, (float)_bowl_img_height);
}

// 获取仅地面部分的网格数据（墙面保持垂直），用于顶视投影。
bool
BowlModel::get_topview_vertex_model (
    VertexMap &vertices, PointMap &texture_points, IndexVector &indeices,
    uint32_t res_width, uint32_t res_height)
{
    float wall_image_height = _config.wall_height / (float)(_config.wall_height + _config.ground_length) * (float)_bowl_img_height;
    float ground_image_height = (float)_bowl_img_height - wall_image_height;

    return get_stitch_image_vertex_model (vertices, texture_points, indeices, res_width, res_height, ground_image_height);
}

// CubeMapModel 负责把 ERP 图转换为立方体贴图布局，供渲染或纹理输出使用。
CubeMapModel::CubeMapModel (
    const uint32_t image_width, const uint32_t image_height)
    : _erp_img_width(image_width)
    , _erp_img_height(image_height)
{ }

enum CubeSide {
    CubeSideRight = 0,
    CubeSideLeft,
    CubeSideUp,
    CubeSideDown,
    CubeSideFront,
    CubeSideBack,
    CubeSideCount
};

// 工具函数：将三维向量归一化。
static PointFloat3
normalize(const PointFloat3& vec)
{
    const auto sum_square = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    return {vec.x / sum_square, vec.y / sum_square, vec.z / sum_square};
}

// 根据 Cubemap 展开纹理上的像素位置，计算其位于立方体六面上的三维坐标。
static PointFloat3
get_cubemap_world_pos(
    const uint32_t u, const uint32_t v,
    const uint32_t cubemap_width, const uint32_t cubemap_height)
{
    // Side size can be non-integer in case of non 3:2 aspect ratio
    const float side_width  = float(cubemap_width ) / 3.f;
    const float side_height = float(cubemap_height) / 2.f;

    // Get direction
    const uint32_t pos_u = floorf(float(u) / side_width);
    const uint32_t pos_v = floorf(float(v) / side_height);
    const auto cube_side =
        static_cast<CubeSide>(CubeSideRight + pos_u + pos_v * 3);
    XCAM_ASSERT(cube_side < CubeSideCount);

    // Calculate side position
    const int side_left   = ceilf(side_width  * pos_u);
    const int side_right  = ceilf(side_width  * (pos_u + 1));
    const int side_top    = ceilf(side_height * pos_v);
    const int side_bottom = ceilf(side_height * (pos_v + 1));

    // Get position on cube side
    const float side_u = 2.f * (float(u - side_left) + 0.5f) / (side_right  - side_left) - 1.f;
    const float side_v = 2.f * (float(v - side_top ) + 0.5f) / (side_bottom - side_top ) - 1.f;

    switch (cube_side) {
    case CubeSideRight:
        return {1.f, -side_u,  side_v};
    case CubeSideLeft:
        return {-1.f, side_u, side_v};
    case CubeSideUp:
        return {side_u, side_v, -1.f};
    case CubeSideDown:
        return {side_u, -side_v, 1.f};
    case CubeSideFront:
        return {side_u, 1.f, side_v};
    case CubeSideBack:
        return {-side_u, -1.f, side_v};
    default:
        return {-side_u, -1.f, side_v};
    }
}

// 将单位球面坐标转换回 ERP（等距矩形投影）坐标。
static PointFloat2
world_to_erp (const PointFloat3& world_pos, uint32_t width, uint32_t height)
{
    const float phi = atan2f(world_pos.x, world_pos.y);
    const float theta = asinf(world_pos.z);

    return {
        (phi   / XCAM_PI       + 1.f) * width  / 2.f,
        (theta / XCAM_PI * 2.f + 1.f) * height / 2.f
    };
}

// 遍历 Cubemap 输出的每个像素，计算其对应的 ERP 坐标。
bool
CubeMapModel::get_cubemap_rect_map(
    PointMap &texture_points,
    uint32_t res_width, uint32_t res_height)
{
    texture_points.resize (res_width * res_height);

    for(uint32_t row = 0; row < res_height; row++) {
        for(uint32_t col = 0; col < res_width; col++) {
            PointFloat3 world_pos = get_cubemap_world_pos(col, row, res_width, res_height);
            world_pos = normalize(world_pos);

            PointFloat2 texture_pos =
                world_to_erp(world_pos, _erp_img_width, _erp_img_height);

            texture_points [res_width * row + col] = texture_pos;
        }
    }
    return true;
}

}
