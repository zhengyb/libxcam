# 环视（Surround View）拼接与渲染原理

本指南面向使用本仓库环视能力（图像拼接与实时渲染）的开发者，总结核心数据结构、算法流程、三种加速后端差异，以及测试用例与运行方法。涉及的关键源码文件位于：

- 接口与公共模型：`xcore/interface/stitcher.h`, `xcore/interface/stitcher.cpp`, `xcore/interface/data_types.h`
- 去畸变模型：`xcore/fisheye_dewarp.h`, `xcore/fisheye_dewarp.cpp`, `xcore/xcam_utils.{h,cpp}`
- 后端实现：`modules/soft/soft_stitcher.*`, `modules/gles/gl_stitcher.*`, `modules/vulkan/vk_stitcher.*`
- 重映射器（GeoMapper）接口：`xcore/interface/geo_mapper.h`
- 渲染：`modules/render/*`, OSG 相关 `RenderOsg*`
- 测试：`tests/test-surround-view.cpp`, `tests/test-render-surround-view.cpp`

## 1. 背景与目标

环视系统通过多路鱼眼相机（通常 2/3/4/6 路）采集到的图像，经过去畸变、几何对齐与拼接融合，生成连续的环形俯视/全景视图（ERP 或 Bowl 视图）。在此基础上可进行二次重映射得到 Top‑View（顶视图）或 Cubemap（立方体贴图），并可在 3D 场景（如 OSG）中进行实时渲染与叠加车模、检测框等。

## 2. 总体架构

- 统一接口：`Stitcher` 定义了环视拼接的抽象接口，软/GL/Vulkan 三种后端共享同一套外部配置与调用方法：
  - Soft（CPU）：`SoftStitcher`
  - GLES（GPU）：`GLStitcher`
  - Vulkan（GPU）：`VKStitcher`
- 关键数据结构：
  - 去畸变模式 `FisheyeDewarpMode`：`DewarpSphere` 或 `DewarpBowl`（默认环视采用 Bowl）
  - 碗面参数 `BowlDataConfig`：椭球 a/b/c、中心高度、地面/墙高度、角度范围等
  - 拼接信息 `StitchInfo`：每路鱼眼（FOV、中心、半径…）、融合区宽度等
  - 几何重映射器 `GeoMapper`：基于 LUT 的像素坐标重映射

## 3. 拼接算法流水线

下面以 `Stitcher` 为核心描述帧级流水线，三个后端的功能等价，仅实现细节与优化不同：

1) 标定与相机切片（Round View Slices）
- 读取标定（Bowl 模式）：从 `FISHEYE_CONFIG_PATH` 指向的目录加载每路 `intrinsic_*.txt` 与 `extrinsic_*.txt`；将四路相机外参平移到车体坐标系（`stitcher.cpp:init_camera_info`）。
- 设置每路输出角幅（`set_viewpoints_range`），估计每路输出切片宽高与角起点/角幅（`estimate_round_slices`）。
- 在目标输出图上标记每路切片中心位置（`mark_centers`）。

2) 去畸变 LUT（Fisheye Dewarp）
- Bowl 模式：世界坐标（椭球/地面）→ 相机坐标 → 成像平面像素坐标（Scaramuzza 多项式），生成查找表（`BowlFisheyeDewarp::gen_table`）。
- Sphere 模式：球面等距投影到鱼眼像面（`SphereFisheyeDewarp::gen_table`）。
- LUT 分辨率通常小于输出图（如 1/16×1/16），由 `GeoMapper` 进行插值与重映射。

3) 重合区与拷贝区划分
- 邻接两路切片的左右有效区与重合区由几何关系估计（`estimate_overlap`）。
- 非重合区作为 Copy 区段直接复制到输出；环绕边界处会进行首尾段合并（`update_copy_areas`）。

4) 特征匹配（可选）
- 在重合区裁剪中带区域进行特征匹配，估计横向错位，折算成两侧重映射的缩放因子，迭代更新，抑制动态漂移（Soft：`soft_stitcher.cpp`，GL：`gl_stitcher.cpp`）。
- 区域比例由 `FMRegionRatio` 控制；帧内/帧间策略由 `FeatureMatchStatus` 与 `ensure_stitch_path()` 控制。

5) 融合（Blending）与输出
- 对重合区进行金字塔融合（pyr-level 可设为 1/2/...）；其余区域直接 Copy。
- 输出为单帧环视纹理（NV12/其他像素格式），可直接落盘或用于后续渲染。

帧处理伪码（概念化）：

```
for each camera i:
  geomap[i] = GeoMapper(LUT_i)

copy non-overlap areas to output
for each neighbor pair (i, i+1):
  left  = geomap[i](overlap_left)
  right = geomap[i+1](overlap_right)
  if feature_match:
    (sx_left, sx_right) = estimate_scale_factors(left, right)
    geomap[i].update_scale(sx_left); geomap[i+1].update_scale(sx_right)
  blend(left, right) -> output(overlap_window)
```

## 4. 二次重映射：Top‑View 与 Cubemap

- Top‑View：在碗面 ERP 图上选取与车体平面对应的矩形区域，生成查找表并 `remap`：
  - 计算最大可视顶视范围（mm）：`BowlModel::get_max_topview_area_mm`
  - 生成顶视矩形查找表：`BowlModel::get_topview_rect_map`
  - `GeoMapper::set_lookup_table` + `remap()` 得到顶视图

- Cubemap：将 ERP 图按 3×2 立方体展开布局生成查找表：
  - 屏幕坐标 → 立方体面 → 单位球方向 → ERP 坐标：`CubeMapModel::get_cubemap_rect_map`
  - `GeoMapper` 重映射得到 Cubemap 帧

测试用例里均有现成调用：`tests/test-surround-view.cpp`（`create_topview_mapper` / `create_cubemap_mapper`）。

## 5. 实时渲染（OSG）

- 网格生成：使用 `BowlModel::get_bowlview_vertex_model` 生成碗面网格顶点/索引/UV；按 a/b/c 缩放归一到可视空间。
- 纹理：以拼接输出（NV12）作为纹理输入，顶点/片元 Shader 完成 YUV→RGB 采样与上屏。
- 车模叠加：加载 OSGB 车模，绑定简单光照 Shader，设置位姿后与碗面一起绘制。
- 参见：`tests/test-render-surround-view.cpp`（`create_surround_view_model`、`create_car_model`、`RenderOsgViewer`）。

## 6. 三种后端差异

- Soft（CPU）：功能最全面，易调试；性能取决于 CPU。
- GLES（GPU）：支持 NV12 + DMABUF 零拷贝导入/导出，提供 Fast‑map 优化；当前不支持 `ScaleDualCurve`。
- Vulkan（GPU）：当前仅支持 `ScaleSingleConst`，接口风格与 GL 类似。

## 7. 关键参数与开关

- 标定路径：`FISHEYE_CONFIG_PATH` 必须有效（目录包含各路 `intrinsic_*.txt` 与 `extrinsic_*.txt`）。
- 视点角幅：通过 `Stitcher::set_viewpoints_range` 设置，与相机朝向匹配。
- 去畸变模式：`Stitcher::set_dewarp_mode(DewarpBowl|DewarpSphere)`。
- 融合层数：`Stitcher::set_blend_pyr_levels(levels)`；`levels==1` 时 GL 可走 Fast‑map 融合。
- 特征匹配：`set_fm_mode / set_fm_frames / set_fm_status / set_fm_region_ratio`。
- 碗面参数：`Stitcher::set_bowl_config` 可按车型/安装高度微调。

## 8. 构建与运行示例

构建：

```
./autogen.sh --prefix=/usr --enable-gst --enable-libcl --enable-gles --enable-vulkan --enable-debug
make -j$(nproc)
```

运行前环境：

```
export LD_LIBRARY_PATH=/usr/lib
export GST_PLUGIN_PATH=/usr/lib/gstreamer-1.0
export FISHEYE_CONFIG_PATH=/path/to/calibration  # intrinsic_*.txt / extrinsic_*.txt 所在目录
```

生成环视帧（支持 Top‑View / Cubemap）：

```
./tests/test-surround-view \
  --module gles \
  --input front.nv12 --input right.nv12 --input rear.nv12 --input left.nv12 \
  --in-w 1280 --in-h 800 --out-w 1920 --out-h 640 \
  --scale-mode singleconst --fm-mode none \
  --save true --save-topview true --topview-w 1024 --topview-h 768 \
  --save-cubemap true --cubemap-w 1536 --cubemap-h 1024
```

实时渲染（碗面 + 车模叠加）：

```
./tests/test-render-surround-view \
  --module gles \
  --input front.nv12 --input right.nv12 --input rear.nv12 --input left.nv12 \
  --in-w 1280 --in-h 800 --out-w 1920 --out-h 640 \
  --scale-mode singleconst --fm-mode none \
  --car Suv.osgb --loop 1
```

说明：
- GLES 后端在某些平台默认需要 EGL/DMABUF 支持；Vulkan 后端目前仅 `ScaleSingleConst`。
- 特征匹配依赖 OpenCV（构建时需打开相关选项）。

## 9. 代码索引（常用入口）

- Stitcher 抽象与工作流：
  - `xcore/interface/stitcher.h/.cpp`：相机数量/输出尺寸/视点范围/碗面参数/特征匹配等的配置与自动估计（切片、中心、重合区、拷贝区）。
- 后端实现：
  - Soft：LUT 生成、重映射、融合、特征匹配更新（`modules/soft/soft_stitcher.cpp`）
  - GLES：DMABUF 导入/导出，GeoMap/Blender/Fast‑map/特征匹配（`modules/gles/gl_stitcher.cpp`）
  - Vulkan：同构实现（`modules/vulkan/vk_stitcher.*`）
- 二次重映射模型：
  - `BowlModel` Top‑View / Bowl‑View 网格与查找表（`stitcher.cpp` 内定义）
  - `CubeMapModel` 立方体贴图查找表（`stitcher.cpp` 内定义）
- 测试：
  - 图像输出：`tests/test-surround-view.cpp`
  - 渲染演示：`tests/test-render-surround-view.cpp`

## 10. 调试建议

- 首先以 Soft 后端验证标定/几何是否正确，再切换到 GLES/Vulkan。
- 逐层输出：原始输入、各路 GeoMap 输出、融合窗口输入/输出，以定位错位/缝隙问题。
- 适当缩小 Top‑View/Cubemap 分辨率或降低金字塔层数，以便快速迭代。

