# Surround View 特征匹配（Default 模式）处理流程

本文档说明在 `tests/test-surround-view` 中，当命令行参数 `--fm-mode default`
时，libxcam 在环视拼接里的特征匹配处理路径与关键逻辑，重点针对 Soft 后端，
GLES/Vulkan 在整体流程上是一致的。

相关代码入口主要在：

- `tests/test-surround-view.cpp`
- `tests/test_sv_params.h`
- `xcore/interface/stitcher.{h,cpp}`
- `modules/soft/soft_stitcher.cpp`
- `modules/ocv/cv_feature_match.{h,cpp}`
- `xcore/interface/feature_match.{h,cpp}`

## 1. Default 模式的启用方式

命令行层面：

- `--fm-mode default`：选择默认特征匹配实现 `CVFeatureMatch`
- `--fm-frames N`：在需要特征匹配的前 N 帧内运行匹配（视 `--fm-status` 而定）
- `--fm-status {wholeway, halfway, fmfirst}`：
  - `wholeway`：全程启用特征匹配
  - `halfway`：仅前 `--fm-frames` 帧做匹配，且同时输出拼接结果
  - `fmfirst`：前 `--fm-frames` 帧只跑匹配不输出，之后再正常拼接

在 `main()` 中，这些参数会被转换为：

- `stitcher->set_fm_mode (FMDefault);`
- `stitcher->set_fm_frames (fm_frames);`
- `stitcher->set_fm_status (fm_status);`
- `stitcher->set_fm_config (fm_config (cam_model));`
- 若去畸变模式为 `DewarpSphere`，还会设置 `FMRegionRatio` 作为匹配区域比例。

`Stitcher::ensure_stitch_path()` 利用上述配置控制是否需要在当前帧运行特征匹配，
以及当前帧是否允许写出拼接结果（通过 `_complete_stitch` / `_need_fm` 标志）。

## 2. 重叠区域与匹配区域的生成

### 2.1 输入重叠区域 `ImageOverlapInfo`

在运行 `stitcher->stitch_buffers()` 前，`Stitcher` 会依次完成：

1. `estimate_round_slices()`：基于相机视场和输出分辨率切分 round view。
2. `mark_centers()`：为每路相机计算对应输出平面上的中心位置 `CenterMark`。
3. `estimate_coarse_crops()`：设置每路输入图像的有效裁剪区域 `ImageCropInfo`。
4. `estimate_overlap()`：计算相邻相机间的重叠窗口，并写入 `_overlap_info[idx]`：
   - `left` / `right`：输入坐标系下左右相机图中的重叠矩形 `Rect`
   - `out_area`：输出坐标系下拼接结果中的重叠窗口

这些信息在 Soft/GLES/Vulkan 后端初始化时会被读取，用于后续特征匹配和融合。

### 2.2 Default 模式下的匹配裁剪区域

Soft 后端在 `StitcherImpl::init_feature_match (uint32_t idx)` 中为每个重叠对
创建特征匹配器：

- 当 `fm_mode == FMDefault` 时，调用
  `FeatureMatch::create_default_feature_match()`，返回 `CVFeatureMatch`。
- 调用 `_overlaps[idx].matcher->set_config (stitcher->get_fm_config ());`，
  其中 `FMConfig` 来自 `tests/test_sv_params.h:fm_config()`，会随 `CamModel`
  调整各阈值（最小角点数、平滑系数等）。
- 依据去畸变模式选择匹配裁剪区域：
  - `DewarpSphere`：利用 `FMRegionRatio` 只保留重叠区域的中间一条带状区域，
    尽量避开天空/车身等纹理较少区域；
  - `DewarpBowl`（surround-view 默认）：将重叠区域裁剪为上半部分
    （主要覆盖地面），高度约为 `wall_height / (wall_height + ground_length)`
    的比例。

最终通过 `FeatureMatch::set_crop_rect (left_ovlap, right_ovlap);` 把左右匹配区域
固定下来，在每帧匹配时从原图中截取这一小块。

## 3. Default 模式的特征检测与光流计算

Default 模式使用 OpenCV 的 FAST + LK 光流：

1. 在 `CVFeatureMatch::detect_and_match()` 内：
   - 使用 `cv::FastFeatureDetector` 在左侧匹配区域检测角点 `_left_corners`。
   - 若检测不到角点，直接返回，本帧不做任何调整。
2. 调用 `cv::calcOpticalFlowPyrLK()`：
   - 输入为左右匹配区域灰度图 + 左侧角点列表；
   - 输出右侧对应点 `_right_corners`、状态 `status` 与跟踪误差 `error`。
3. 将光流结果传给 `CVFeatureMatch::calc_of_match()` 做后处理。

在调试模式 (`XCAM_CV_FM_DEBUG`) 下，还会将左右重叠区域串接为一张大图，
画出光流向量，并保存到 `fm_optical_flow_*.jpg` 中，便于检查匹配质量。

## 4. 匹配结果过滤与均值偏移估计

`CVFeatureMatch::get_valid_offsets()` 和 `FeatureMatch::get_mean_offset()` 共同
决定哪些光流结果是“可信”的，并从中计算稳定的水平偏移。

### 4.1 单点级过滤

对每一对匹配点 `(corner0[i], corner1[i])`，依次进行以下过滤：

- `status[i]` 必须为真：LK 光流跟踪成功；
- 跟踪误差 `error[i]` 必须小于 `_config.max_track_error`；
- 垂直位移 `|corner0[i].y - corner1[i].y|` 必须小于
  `_config.max_valid_offset_y`，认为重叠区域主要存在水平视差；
- 右图匹配点 `corner1[i].x` 需落在图像宽度范围内。

通过上述过滤后，将水平位移

```text
offset = corner1[i].x - corner0[i].x
```

作为候选样本加入 `offsets` 列表，计入 `sum` 与有效点数 `count`。

### 4.2 多轮均值收敛与离群点剔除

`FeatureMatch::get_mean_offset()` 对 `offsets` 做迭代均值收敛：

1. 若初始有效点数 `count < min_corners`，直接返回失败；
2. 计算初始均值 `mean_offset = sum / count`；
3. 进行最多 3 次迭代：
   - 对所有 `offsets` 重新扫描，仅保留与当前均值差
     小于 `_config.recur_offset_error` 的点；
   - 若新点数仍小于 `min_corners`，则认为不可靠，返回失败；
   - 用新样本重算 `mean_offset`，检查与上一轮均值差是否过大；
   - 若均值基本稳定且点数未变化，则认为收敛成功。

该过程相当于一个简单的“去极值 + 重算均值”的鲁棒估计，以减少噪声匹配对整体偏移的影响。

## 5. 帧间平滑与偏移限制

在 `CVFeatureMatch::calc_of_match()` 中，若成功获得稳定的 `mean_offset`，
会与上一帧均值 `last_mean_offset` 结合进一步滤波：

1. 首先检查帧间偏差：

```text
if (fabs(mean_offset - last_mean_offset) < delta_mean_offset) {
    // 仅在变化不离谱时才采用本帧结果
}
```

2. 若通过检查，则采用指数平滑更新 `_x_offset`：

```text
_x_offset = _x_offset * offset_factor
          + mean_offset * (1.0f - offset_factor);
```

`offset_factor` 越大，历史权重越高，响应越慢但越平滑。

3. 最后对 `_x_offset` 做幅度限制，防止单帧把 seam 拉得过大：

```text
if (fabs(_x_offset) > max_adjusted_offset)
    _x_offset = (_x_offset > 0.0f)
              ?  max_adjusted_offset
              : -max_adjusted_offset;
```

同时会更新 `_mean_offset`、`_valid_count` 等调试统计信息。

## 6. 调整重叠区域（可选）

在 Default 模式中，还可以选择根据匹配结果动态调整重叠窗口大小，
以便让重叠区域更好地覆盖真实“拼接最佳带”。

当上层调用 `FeatureMatch::enable_adjust_crop_area()` 时，
`CVFeatureMatch::detect_and_match()` 在完成匹配后会执行
`adjust_crop_area()`：

1. 若当前水平偏移 `|_x_offset|` 太小（例如 `< 5` 像素），直接返回；
2. 根据当前左右重叠窗口与全局输出宽度 `_dst_width`，计算“原始重叠宽度”；
3. 使用 `_x_offset` 估算“目标重叠宽度”，并对其做对齐与最小宽度约束：
   - 对齐到 8 像素；
   - 保证至少为 `stitch_min_width`；
4. 以目标重叠宽度为依据，重新计算左右窗口的位置与宽度：
   - 使左右输入窗口宽度相同且为 `stitch_min_width`；
   - 保证两者在输出中的拼接位置与最终 overlap 宽度一致。

这样，在标定或长时间运行过程中，即使几何误差发生累积漂移，
也可以通过特征匹配自动微调重叠区域的几何关系，从而保持拼接 seam 的稳定性。

## 7. 匹配结果如何作用到拼接

不同后端将 `_x_offset` 作用到拼接上的方式略有差别，但基本思想一致：

- 在 Soft 后端中，`StitcherImpl::get_and_reset_feature_match_factors()` 会把
  每个重叠区域的 `_x_offset` 转换为左/右两侧 GeoMapper 的缩放因子
  `left_match_factor` / `right_match_factor`，并累积到每路相机的
  `ScaleFactor` 中，驱动 `init_geomap_factors()` 更新重映射 LUT；
- 在 GLES/Vulkan 后端中，亦通过各自的 `*_stitcher.cpp` 将偏移转换为
  纹理坐标或重映射因子，确保重叠区域随时间逐步贴合最佳对齐位置。

简而言之，Default 模式下：

1. 在每帧或前若干帧中，对重叠区域做光流特征匹配；
2. 利用多重阈值与迭代均值估计获取稳定的水平偏移；
3. 通过平滑和幅度限制保证结果连续、不过冲；
4. 将偏移转化为 GeoMapper 的比例调整或重叠窗口调整，从而不断校正拼接 seam。

