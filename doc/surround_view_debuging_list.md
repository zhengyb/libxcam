# Surround View Debuging List


### Topview的视野范围是怎么确定的？ DONE
[Answered by Codex] Topview 的覆盖范围由碗面椭球模型 `BowlDataConfig` 决定：`tests/test-surround-view.cpp:create_topview_mapper` 中先从 `Stitcher::get_bowl_config()` 构造 `BowlModel`，再调用 `BowlModel::get_max_topview_area_mm` 计算当前 Bowl 模型在地面上的最大可视长宽（单位 mm），并把这个物理尺寸传给 `BowlModel::get_topview_rect_map` 生成查找表，最终通过 `GeoMapper::remap` 得到顶视图。因此 Topview 的视野范围实质上等于 Bowl 椭球在 z=0 平面上允许投影的最大矩形区域，修改 `BowlDataConfig`（a/b/c、center_z、wall_height、ground_length 等）即可改变顶视物理范围，而 Topview 的分辨率只影响显示尺度而不改变物理视场。

[Reuben] Bowl椭球体在Z=0处与地面相交，构成碗底平面。碗底平面是一个椭圆形，r=sqrt(1.0 - cz^2/c^2). Lm = sqrt(2)*a*r, Wm = sqrt(2)*b*r. 当bowl.a=bowl.b=bowl.c时，椭球体退化成球体，Lm=Wm=1.22*a. test-surround-view.cpp已经修改为自动计算Bowl模型参数。

### 重叠区域是如何计算得到的？ DONE
[Answered by Codex] 重叠区域由 `Stitcher::estimate_overlap()` 自动计算：在 `stitcher.cpp` 中，先基于 `estimate_round_slices()` 和 `mark_centers()` 得到每路相机对应的 round view 切片宽度与输出中心位置 `CenterMark`，再结合每路的裁剪参数 `ImageCropInfo` 计算各自的“有效宽度”；若两路有效宽度之和大于它们在输出上的中心间距 `merge_width`，则存在重叠，重叠宽度为 `valid_left.width + valid_right.width - merge_width`，并在左图取靠右的一段、右图取靠左的一段作为 `left/right_img_overlap`，同时生成对应输出上的窗口 `out_overlap`，最终写入 `_overlap_info[idx]`，供 Soft/GLES/Vulkan 后端做特征匹配和融合使用。

[Reuben] Bowl View image 中用红色虚线框画出了重叠区域。
### 重叠区域的特征检测与匹配的原理是什么？如何提高标定时的特征匹配效果？ DONE
[Answered by Codex] 本项目在重叠区域内采用“特征点 + 光流”的方案进行匹配：Soft/GLES/Vulkan 三个后端在 `soft_stitcher.cpp`、`gl_stitcher.cpp`、`vk_stitcher.cpp` 中通过 `StitcherImpl::start_overlap_task()` 调用 `FeatureMatch::feature_match()`，默认实现 `CVFeatureMatch`（`modules/ocv/cv_feature_match.cpp`）先用 OpenCV 的 FAST 检测角点，再用金字塔 LK 光流 (`cv::calcOpticalFlowPyrLK`) 跟踪到另一幅重叠图像中；然后根据误差 `error`、纵向偏差 `max_valid_offset_y`、跟踪范围等阈值过滤掉不可靠点，并对剩余点的横向位移做平均与指数平滑（受 `FMConfig::offset_factor/delta_mean_offset/max_adjusted_offset` 控制），得到最终的水平偏移 `_x_offset`。在聚类模式 `CVFeatureMatchCluster`（`modules/ocv/cv_feature_match_cluster.cpp`）下，还会对光流结果按位移进行聚类，选取点数最多的一簇计算 (x,y) 平均偏移，以进一步抑制噪点。匹配得到的偏移在后端的 `start_feature_match()/init_geomap_factors()` 中被转换为左右两侧 GeoMapper 的缩放因子 `left/right_match_factor`，逐帧累乘，从而精细调整重叠区域的几何关系。要提高标定时的匹配效果，代码层面主要通过三类手段：1）合理设置匹配区域比例 `FMRegionRatio`（`tests/test_sv_params.h:fm_region_ratio`），只在纹理丰富、遮挡较少的区域进行匹配；2）根据不同相机模型调节 `FMConfig`（最小角点数、平滑系数、最大纵向偏移、最大光流误差等），以便在噪声多/对比度低时保持稳定；3）在测试程序中利用 `--fm-mode` 选择 `cluster` 或 `capi` 模式，并用 `--fm-frames` + `--fm-status fmfirst/halfway` 把特征匹配集中在开始若干帧或仅在初始化阶段运行，减少动态物体对标定匹配的干扰。

[Reuben] 1. 使用cluster特征匹配法； 2. 树立辅助特征匹配的标靶（工程上不可取）; 3）特征匹配只用墙面图像进行。

### 特征匹配结果如何持久化保存，以便后续快速处理？ TODO
[Answered by Codex] 本项目不支持。
[Reuben] 产品每次开机时使用前面N帧进行特征匹配。

### 多视角的查看车辆周边的原理？
[Answered by Codex] 多视角查看的核心是“先拼接成统一环境贴图，再做二次重映射”：首先通过 `Stitcher` 将多路鱼眼在 Bowl/Sphere 模型下去畸变、对齐并融合成一张环视 ERP/Bowl 纹理；在此基础上，Topview 通过 `BowlModel::get_topview_rect_map` 把碗面地面区域映射到车体上方的俯视平面；Cubemap 通过 `CubeMapModel::get_cubemap_rect_map` 将 ERP 图展开为 3×2 立方体六面视图；OpenCL 样例 `tests/test-image-stitching.cpp` 中还提供 `sample_generate_rectified_view`，可以按给定起止角度生成任意方位的“自由视角”矩形视图。所有这些视角都是用 `GeoMapper::set_lookup_table` 配置查找表，再调用 `remap()` 把同一张环视纹理转换到不同相机虚拟视角，从而实现多视角查看车辆周边。

### 如何使用GPU加速处理？
[Answered by Codex] 本项目已经内置多种加速后端：1）OpenCL：`modules/ocl` 下的 `CLImage360Stitch` 提供基于 CL 的去畸变与拼接，示例程序为 `tests/test-image-stitching`；2）OpenGL ES：`modules/gles/gl_stitcher.*` 实现了基于 GLES 的 GeoMapper 和 Blender，`tests/test-surround-view` 与 `tests/test-render-surround-view` 可通过 `--module gles` 选择该后端，并在有 dmabuf 支持的平台上实现零拷贝；3）Vulkan：`modules/vulkan/vk_stitcher.*` 提供 Vulkan 实现，测试程序通过 `--module vulkan` 选择，目前仅支持 `ScaleSingleConst` 模式。编译时需要在 `./autogen.sh` 中启用对应选项（如 `--enable-libcl --enable-gles --enable-vulkan`），运行时通过命令行 `--module` 即可把几何重映射和融合从 CPU 切换到 GPU/加速器执行。

### 如何让程序直接从系统上的摄像头直接抓取画面，替代nv12输入图像读取。
[Answered by Codex] 目前 `tests/test-surround-view` 与 `tests/test-image-stitching` 等示例是从 NV12 文件读入，但项目本身已经通过 GStreamer 插件 `xcamsrc` 支持直接从 V4L2/USB/ISP 摄像头取流（见 `wrapper/gstreamer/gstxcamsrc.*` 和脚本 `tests/test-xcamsrc-camera.sh`）。构建时开启 `--enable-gst` 并安装插件后，可以使用命令例如：`gst-launch-1.0 xcamsrc sensor-id=0 io-mode=4 ! video/x-raw,format=NV12,width=...,height=...,framerate=... ! xcamfilter ! ...`，由 `xcamsrc` 从系统摄像头抓取 NV12 帧，并交由 `xcamfilter`/libxcam 做实时处理。如果希望复用 `test-surround-view` 的处理逻辑，则需要参考 `wrapper/gstreamer` 中的 `MainDeviceManager` 和 `gstxcamfilter` 实现一个新的应用，从 `xcamsrc` 或下游 filter 输出的 `VideoBuffer` 中直接调用 `Stitcher::stitch_buffers()` 处理，而不是从文件读取。

### 如何根据车辆尺寸信息和相机标定结果自动选择合适的Bowl Model Data，减少人工干预的工作量。
[Answered by Codex] 本项目不支持。

[Reuben] 已经实现根据相机外参自动计算Bowl Model Data的功能。
### 如何添加3D车辆模型。
[Answered by Codex] 环视渲染路径中已经集成了 3D 车模能力（基于 OpenSceneGraph）：`tests/test-render-surround-view.cpp` 的 `create_car_model()` 函数会根据命令行 `--car` 传入的模型名，或默认常量 `CAR_MODEL_NAME`，在 `FISHEYE_CONFIG_PATH` 或其子路径下查找对应的 OSGB 模型文件，并构造一个 `RenderOsgModel`；随后为车模绑定专用着色器 `VtxShaderCar/FrgShaderCar`，并通过 `RenderOsgModel::setup_model_matrix()` 把车模放置在碗面中心上方。要添加新的 3D 车辆模型，只需：1）用外部建模工具导出目标车型的 `.osgb` 文件；2）将文件放入 `FISHEYE_CONFIG_PATH` 指定目录或在运行时通过 `--car` 传入完整路径；3）在需要的示例中（例如 `test-render-surround-view`）启用 `create_car_model` 并把返回的 `RenderOsgModel` 通过 `RenderOsgViewer::add_model()` 加入场景，即可在环视画面上叠加 3D 车辆模型。
