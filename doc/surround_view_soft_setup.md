# 环视测试（Soft 后端）环境搭建与调试指南

本文档说明如何在 Ubuntu 24 环境下，构建并调试 `tests/test-surround-view.cpp` 与 `tests/test-render-surround-view.cpp`（软后端 CPU 版本）。

## 1. 准备依赖
- 更新系统并安装基础工具与软后端所需库：
  ```bash
  sudo apt update
  sudo apt install build-essential autoconf automake libtool pkg-config gawk \
                   libopencv-dev libopenscenegraph-dev libopenthreads-dev \
                   nlohmann-json3-dev
  ```
- 如需自制或转换测试输入，可额外安装 `ffmpeg`。后续若要启用 GPU 后端，再补装 `ocl-icd-opencl-dev`、`mesa-common-dev` 等 GPU 相关依赖。

## 2. 准备标定与输入数据
- 在仓库根目录新建 `calib_params/`，放置四路标定文本：
  - `intrinsic_camera_{front,right,rear,left}.txt`
  - `extrinsic_camera_{front,right,rear,left}.txt`
  标定文件格式与示例可参见 `doc/surround_view.md`。
- 若计划运行 `test-render-surround-view`，同一目录还应包含车模 `Suv.osgb`（或其他 OSGB 模型），以及（可选）`camera_calibration_*.json`。
- 准备四路尺寸一致、格式为 NV12 或 YUV420 的原始输入文件（例如 `front_1280x800.nv12` 等）。可通过：
  ```bash
  ffmpeg -i front.mp4 -pix_fmt nv12 -f rawvideo front_1280x800.nv12
  ```
  将压缩视频转成原始帧。

## 3. 配置与编译（Soft 模块）
在仓库根目录执行：
```bash
./autogen.sh --prefix=/usr \
             --enable-opencv \
             --enable-render \
             --disable-libcl \
             --disable-gles \
             --disable-vulkan \
             --disable-dnn \
             --disable-gst
make -j"$(nproc)"
```
此配置仅构建软后端与 OSG 渲染模块，避免引入 GPU/OpenCL 依赖。

## 4. 运行前环境变量
```bash
PWD=`pwd`
export LD_LIBRARY_PATH=$PWD/xcore/.libs:$PWD/modules/soft/.libs:\
$PWD/modules/ocv/.libs:$PWD/modules/render/.libs:$LD_LIBRARY_PATH
export FISHEYE_CONFIG_PATH=$PWD/calib_params
export OSG_FILE_PATH=$FISHEYE_CONFIG_PATH:${OSG_FILE_PATH:-}
```
- `LD_LIBRARY_PATH` 覆盖运行期所需的本地 `.libs`。
- `FISHEYE_CONFIG_PATH` 指向标定与车模所在目录。
- `OSG_FILE_PATH` 让 OpenSceneGraph 能从同一目录加载车模资源。
- 如在无图形界面环境下运行 `test-render-surround-view`，需提前准备 X11 会话并确保 `DISPLAY` 环境变量有效。

## 5. 运行示例
- 仅生成拼接输出/顶视图：
  ```bash
  ./tests/.libs/test-surround-view \
    --module soft \
    --input ./inputs/usb_cameras_003/front_1280x720.nv12 \
    --input ./inputs/usb_cameras_003/right_1280x720.nv12 \
    --input ./inputs/usb_cameras_003/rear_1280x720.nv12 \
    --input ./inputs/usb_cameras_003/left_1280x720.nv12 \
    --in-w 1280 --in-h 720 \
    --out-w 1920 --out-h 640 \
    --scale-mode singleconst \
    --frame-mode single \
    --fm-mode none \
    --save true \
    --save-topview true --topview-w 720 --topview-h 1280 \
    --save-cubemap false
  ```
  生成的输出文件会以默认前缀保存在当前目录。

- 带渲染窗口（需车模与图形环境）：
  ```bash
  ./tests/.libs/test-render-surround-view \
    --module soft \
    --input front_1280x800.nv12 \
    --input right_1280x800.nv12 \
    --input rear_1280x800.nv12 \
    --input left_1280x800.nv12 \
    --in-w 1280 --in-h 800 \
    --out-w 1920 --out-h 640 \
    --scale-mode singleconst \
    --fm-mode none \
    --car Suv.osgb \
    --loop 1
  ```
  运行时终端会打印拼接参数，OSG 窗口显示碗面视图与车辆模型。

## 6. 调试与故障排查
- 使用 `libtool --mode=execute gdb ./tests/test-surround-view` 或 `lldb` 进入调试，避免手动配置 `.libs`。
- 设置 `export XCAM_LOG_LEVEL=4` 可开启更多调试日志。
- 若输出异常，先确认：
  1. 标定目录与文件名是否匹配（`FISHEYE_CONFIG_PATH` 生效）。
  2. 四路输入分辨率、格式、帧数是否一致。
  3. 运行用户是否拥有访问 X11 的权限（渲染场景）。
- 在软后端验证流程正确后，可启用 `--enable-gles`/`--enable-vulkan` 并补装 GPU 依赖，使用 `--module gles` 或 `--module vulkan` 做性能测试。

## 7. 参考资料
- `doc/surround_view.md`：算法流程、关键模块说明。
- `tests/test-surround-view.cpp`、`tests/test-render-surround-view.cpp`：示例代码与命令行选项。
- `xcore/interface/stitcher.{h,cpp}`：拼接核心接口与参数配置。
