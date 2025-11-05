# 环视标定文件格式说明

libxcam 的环视拼接默认从 `FISHEYE_CONFIG_PATH` 指向的目录按文件名约定载入每路相机的标定结果：

- 内参文件：`intrinsic_camera_<position>.txt`
- 外参文件：`extrinsic_camera_<position>.txt`
- （可选）统一 JSON：`camera_calibration_*.json`

本文基于 `xcore/calibration_parser.cpp` 中的解析流程，总结三种文件的字段含义与排列方式，并在 `doc/calibration_samples/` 下给出示例。

## 1. 内参文件（Intrinsic）

解析函数 `CalibrationParser::parse_intrinsic_param` 逐行读取，忽略空行与以 `#` 开头的注释。数据区必须满足：

> 注：解析实现会忽略第一个非注释数据行，传统标定文件通常在该行记录图像分辨率（例如 `1280 800`）。实际多项式数据需放在下一行开始。本示例按照这一习惯组织。

1. **多项式长度与系数**

   ```
   <poly_length> <poly_coeff[0]> <poly_coeff[1]> ... <poly_coeff[poly_length-1]>
   ```

   - `poly_length` ≤ 16（`XCAM_INTRINSIC_MAX_POLY_SIZE`）。
   - 系数按照 Scaramuzza OcamCalib 多项式顺序，从常数项开始依次列出。

2. **主点坐标（单位：像素）**

   ```
   <cy> <cx>
   ```

   - 注意解析顺序是 `cy` 在前、`cx` 在后。

3. **仿射参数**

   ```
   <c> <d> <e>
   ```

   - 对应公式 `img_coord.x = img_x * c + img_y * d + cx`，`img_coord.y = img_x * e + img_y + cy`。

文件中除上述行以外可以添加注释行，例如 `# polynomial`，解析器会自动跳过。图像分辨率、焦距等字段需要由上层配置或 JSON 文件补充，本格式不包含。

## 2. 外参文件（Extrinsic）

`CalibrationParser::parse_extrinsic_param` 按顺序读取 6 行数值，每行可包含简单注释但必须以数值起始。单位约定如下（与去畸变实现保持一致）：

```
<trans_x>   # 车体坐标系下的 X 平移，单位：毫米
<trans_y>   # 车体坐标系下的 Y 平移，单位：毫米
<trans_z>   # 车体坐标系下的 Z 平移，单位：毫米
<roll>      # 绕 Z 轴旋转角，单位：度
<pitch>     # 绕 X 轴旋转角，单位：度
<yaw>       # 绕 Y 轴旋转角，单位：度
```

解析器允许在数字后继续添加注释内容（使用空格分隔），或在独立行中添加 `# comment`。

## 3. JSON 标定文件（可选）

若在配置阶段调用 `CalibrationParser::parse_fisheye_camera_param`（需启用 `--enable-json` 并提供 `json.hpp`），库可直接读取 OpenCV fisheye 标定导出的 JSON。基本结构：

```json
{
  "model": 3,
  "cameras": {
    "camera": [
      {
        "radius": 1787.0,
        "cx": 640.0,
        "cy": 400.0,
        "w": 1280,
        "h": 800,
        "skew": 0.0,
        "fx": 520.4,
        "fy": 520.1,
        "fov": 190.0,
        "flip": "false",
        "yaw": 0.0,
        "pitch": -1.5,
        "roll": 0.2,
        "K": [ ... 9 entries ... ],
        "D": [ ... 4 entries ... ],
        "R": [ ... 9 entries ... ],
        "t": [ tx, ty, tz ],
        "c": [ c, d, e ]
      }
    ]
  }
}
```

- `model`（可选）：同时写入 `FisheyeInfo::cam_model`，缺省时保持默认值。
- `camera` 数组长度应与使用的鱼眼路数一致（最多 6）。
- `K`/`D`/`R`/`t`/`c` 字段均为可选，缺失时维持默认值。
- `flip` 以字符串 `"true"` 或 `"false"` 表示。

## 4. 示例文件

`doc/calibration_samples/` 目录下提供了一套 CPU 软后端可直接加载的模板：

- `intrinsic_camera_front.txt` 等四个内参示例：匹配解析顺序（多项式长度、主点、仿射参数）。
- `extrinsic_camera_front.txt` 等四个外参示例：采用毫米 + 度单位。
- `camera_calibration_sample.json`：展示 JSON 版多路标定的典型字段。

可复制该目录到 `calib_params/` 并根据实际标定结果替换数字，或作为格式参考编写自有文件。
