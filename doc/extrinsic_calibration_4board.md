# 环视外参标定（四角回字板方案）

本文介绍如何基于四块“回”字形标定板标定 libxcam 环视系统的外参。该方案适用于搭载 200° 鱼眼摄像头的车辆：在车辆四周放置四块与车身平行的标定板，每块板提供 8 个特征角点（外环四角 + 内环四角）。每台摄像头可同时看到两块标定板，共 16 个对应点，可用于估计该摄像头在车辆坐标系下的位姿。

## 1. 坐标系约定

- 车辆坐标系：+X 向右，+Y 向前，+Z 向上；原点位于车辆几何中心（或其他固定点，但四块标定板的位置必须按同一基准测量）。  
- 板坐标系：以标定板中心为原点，保持与车辆坐标轴平行。各角点坐标使用毫米或与车辆一致的单位。
- 输出平移 `trans_x/trans_y/trans_z` 即摄像头光心相对于车辆坐标原点的位置，旋转 `roll/pitch/yaw` 为绕 X/Y/Z 轴的欧拉角，单位度，顺序为 `Rz(yaw) * Ry(pitch) * Rx(roll)`。

## 2. 标定场景搭建

1. 在车辆四个顶角放置回字形标定板，板面与车身平行，四周形成一个近似矩形框。  
2. 精确测量：  
   - 每块板中心相对于车辆坐标原点的平移（X/Y/Z）。  
   - 每块板的姿态（roll/pitch/yaw），通常 roll/pitch≈0，yaw 表示与车身平行方向。  
   - 回字板外环与内环四个角点相对于板心的局部坐标。推荐顺序：外环按左上→右上→右下→左下，内环同理。
3. 若需要提升识别可靠性，可在四个板上贴编号标签或使用不同颜色，方便 ROI 选取。

## 3. 软件流程概览

1. **标定内参**  
   使用 `tools/calibrate_fisheye_intrinsics.py` 及棋盘格图像，获得 fisheye 内参 JSON。

2. **编写板配置（JSON）**  
   在 `board-config` 文件中描述四块板的 3D 位置、朝向和 8 个角点的局部坐标，可附加 ROI（像素区域）帮助脚本在图像中快速定位每块板。

3. **采集外参图像**  
   每台摄像头在最终安装状态下拍摄多张照片，确保画面同时包含两块标定板。建议每块板至少覆盖 16 个角点一次。

4. **运行外参标定脚本**  
   `tools/calibrate_fisheye_extrinsics.py` 会对每张图片检测两块板的角点，结合板配置内的 3D 坐标，通过 `cv2.fisheye.solvePnP` 估计摄像头位姿，并对多帧结果求平均。

5. **输出结果**  
   - `extrinsic_camera_*.txt`：libxcam 读取的文本格式。  
   - 可选：JSON 报告（`--save-json`）与可视化标注图（`--annotated-dir`），用于复核检测结果。

## 4. 板配置 JSON 示例

```json
{
  "boards": [
    {
      "name": "front_right",
      "translation": [850.0, 1800.0, 200.0],
      "rotation_deg": [0.0, 0.0, 0.0],
      "local_points": [
        [-400.0,  300.0, 0.0],  [400.0,  300.0, 0.0],
        [ 400.0, -300.0, 0.0],  [-400.0, -300.0, 0.0],
        [-200.0,  120.0, 0.0],  [200.0,  120.0, 0.0],
        [ 200.0, -120.0, 0.0],  [-200.0, -120.0, 0.0]
      ],
      "roi": [1800, 200, 900, 900]
    },
    {
      "name": "rear_right",
      "translation": [850.0, -1800.0, 200.0],
      "rotation_deg": [0.0, 0.0, 180.0],
      "local_points": [...],
      "roi": [1600, 900, 900, 900]
    }
    // rear_left, front_left ...
  ]
}
```

说明：
- `translation`：板中心的车辆坐标。单位与车辆一致（建议毫米）。  
- `rotation_deg`：板坐标系相对于车辆坐标系的欧拉角，单位度。若板与车身平行，通常仅 yaw 设置 0°/180°。  
- `local_points`：按“外环 4 点 + 内环 4 点”顺序列出，每个点为 `[x, y, z]`（z 多数为 0）。输出顺序必须与图像检测顺序一致。  
- `roi`（可选）：图像中包含该标定板的大致区域 `[x, y, width, height]`，用来限制检测范围，提高稳定性。

## 5. 运行外参标定脚本

```bash
python3 tools/calibrate_fisheye_extrinsics.py \
  --intrinsic-json output/intrinsics_front_cam.json \
  --board-config configs/board_layout.json \
  --image-glob "data/front_cam/*.jpg" \
  --output-txt calib_params/extrinsic_camera_front.txt \
  --save-json output/front_cam_extrinsic_report.json \
  --annotated-dir output/annotated_front_cam \
  --min-boards-per-frame 2
```

常用参数：
- `--binary-threshold`：若场景光照稳定，可指定固定阈值（1–255）；默认 0 使用自适应阈值。  
- `--adaptive-block-size / --adaptive-c`：控制自适应阈值窗口和偏移，可根据板材颜色微调。  
- `--min-quad-area`：剔除过小的伪轮廓。  
- `--annotated-dir`：输出带角点编号的图片，便于人工核验成功率。

处理流程：脚本对每张图进行角点检测，找到 ROI 内最大的外/内嵌套四边形组合（回字），将其顶点排序为左上→右上→右下→左下，并拼接为 8 个点。若单帧检测出的标定板数不足 `--min-boards-per-frame`，该帧会被忽略。

## 6. 输出结果格式

`extrinsic_camera_front.txt` 内容示例：

```
# translation (millimetres)
12.345678
1350.987654
1420.123456
# rotation angles (degrees)
-0.523456
-1.734567
0.456789
```

放入 `calib_params/` 后即可被 `test-surround-view` / `test-render-surround-view` 读取。

`--save-json` 选项会生成报告，包含每帧使用的标定板、求得的位姿以及最终平均值；可在调试时快速识别异常帧。

## 7. 调试建议

- **检测失败**：通过 `--annotated-dir` 查看角点标注；若外环/内环识别错误，可调整 ROI、阈值或在板上增加对比度。  
- **板模型不准**：确保测量的板中心、角点尺寸准确无误；可在 JSON 中使用真实尺寸（mm）。  
- **姿态偏差**：当所有板不在同一平面或存在倾斜时，应在 `rotation_deg` 中填入实际 roll/pitch 值。  
- **多帧平均**：建议选取若干张不同曝光或轻微移动标定板的位置的图像，脚本会自动对所有有效帧求平均，降低噪声。

借助该流程，即可获得满足 libxcam 环视所需格式的外参文件，并快速验证四块回字板方案的标定质量。下一步可将结果拷贝至运行目录，结合 soft 后端测试环视拼接效果。***
