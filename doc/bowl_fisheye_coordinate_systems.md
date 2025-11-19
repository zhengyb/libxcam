# Bowl 模式下的坐标系说明

本文件说明 `BowlFisheyeDewarp::gen_table` 在生成查找表时涉及到的各个坐标系，包括坐标系名称、原点位置和坐标轴方向定义。对应的核心代码位于：

- `xcore/fisheye_dewarp.cpp`
- `xcore/xcam_utils.{h,cpp}` 中的 `bowl_view_image_to_world`、`centralize_bowl_coord_from_cameras`
- `xcore/interface/data_types.h` 中的 `BowlDataConfig`、`ExtrinsicParameter`

代码主流程如下（简化）：

```cpp
for each (row, col) {
    out_pos -> world_coord (bowl_view_image_to_world)
             -> cam_world_coord (cal_cam_world_coord)
             -> cam_coord (world_coord2cam)
             -> img_coord (cal_img_coord)
    map_table[row * tbl_w + col] = img_coord;
}xia m
```

下文按这一链路逐一说明坐标系。

---

## 1. Bowl 视图图像坐标系（输出平面）

对应变量：`out_pos` (`PointFloat2`)，在 `BowlFisheyeDewarp::gen_table` 中由表格索引和缩放系数得到：

```cpp
out_pos.x = col * scale_factor_w;
out_pos.y = row * scale_factor_h;
```

- **名称**：Bowl 视图图像坐标系（Top/Bowl View Image）
- **原点位置**：输出图像左上角像素中心
- **坐标轴方向**：
  - X 轴：向右为正，对应像素列索引增加（`col` 增大）
  - Y 轴：向下为正，对应像素行索引增加（`row` 增大）
- **单位**：像素
- **用途**：查找表的“目标平面”坐标，每个 LUT 采样点先在该坐标系中表示，再通过 `bowl_view_image_to_world` 投影到碗面三维空间。

---

## 2. Bowl 世界坐标系（碗面 / 车体坐标系）

对应变量：`world_coord` (`PointFloat3`)，由 `bowl_view_image_to_world` 计算得到：

```cpp
world_coord = bowl_view_image_to_world (_bowl_cfg, out_w, out_h, out_pos);
```

`bowl_view_image_to_world` 将顶视 / 碗面图上的二维像素坐标映射到碗面三维空间。

- **名称**：Bowl 世界坐标系 / 车体坐标系（Bowl World / Vehicle Frame）
- **原点位置**：
  - 经过 `centralize_bowl_coord_from_cameras` 处理后，原点在四个相机光心在地平面投影的中心，近似为车辆几何中心。
- **坐标轴方向**：
  - X 轴：向前，指向车头方向为正。
  - Y 轴：向右，指向车辆左侧为正。
  - Z 轴：向上为正。
    - 在 `bowl_view_image_to_world` 中：
      - 墙面区域：`world.z` 从 `config.wall_height` 逐渐减小到 `config.center_z`，再过渡到 0。
      - 地面区域：`world.z = 0`，表示地面。
- **单位**：毫米（`BowlDataConfig` 的 a / b / c / ground_length / wall_height / center_z 等参数均用 mm）
- **用途**：
  - 表达车辆周围碗面/地面上的三维点位置；
  - 相机外参 `ExtrinsicParameter.trans_x/y/z` 也在该坐标系下给出，即相机光心相对于车体坐标系的位置。

---

## 3. 以相机为参考的世界坐标（cam_world_coord）

对应变量：`cam_world_coord` (`PointFloat3`)，由 `BowlFisheyeDewarp::cal_cam_world_coord` 计算：

```cpp
Mat4f rotation_mat = generate_rotation_matrix (
    degree2radian (_extr_param.roll),
    degree2radian (_extr_param.pitch),
    degree2radian (_extr_param.yaw));

Mat4f rotation_tran_mat = rotation_mat;
rotation_tran_mat (0, 3) = _extr_param.trans_x;
rotation_tran_mat (1, 3) = _extr_param.trans_y;
rotation_tran_mat (2, 3) = _extr_param.trans_z;

Mat4f world_coord_mat (... world_coord.x/y/z ...);

Mat4f cam_world_coord_mat = rotation_tran_mat.inverse () * world_coord_mat;

cam_world_coord.x = cam_world_coord_mat (0, 3);
cam_world_coord.y = cam_world_coord_mat (1, 3);
cam_world_coord.z = cam_world_coord_mat (2, 3);
```

这里 `_extr_param` 中的 `roll/pitch/yaw/trans_*` 描述的是“相机在 Bowl 世界坐标系中的位姿”，`rotation_tran_mat` 为从相机坐标系到 Bowl 世界坐标系的变换 `[R | t]`。在生成 LUT 时，需要把碗面上的三维点从 Bowl 系变换到相机附近，因此使用了其逆变换：

```text
cam_world_coord = Rᵀ * (world_coord - t)
```

- **名称**：以相机为参考原点的世界点坐标（World-in-Camera-Pose Frame）
- **原点位置**：相机光心（通过平移 t = `trans_x/y/z` 消除）
- **坐标轴方向**：
  - 相比 Bowl 世界坐标系，经过 `_extr_param` 中的旋转（roll/pitch/yaw，见 `generate_rotation_matrix`）对齐到相机朝向，但此时仍然沿用 Bowl 坐标的轴标记，只是原点和平衡姿态发生变化。
  - 可以理解为：“在相机自身位姿下观察到的世界点”，尚未重排到成像模型使用的标准相机坐标系。
- **单位**：毫米
- **用途**：作为 Bowl 世界坐标到真正相机坐标系之间的中间层，为后续轴重排提供统一入口。

---

## 4. 相机坐标系（Camera Frame）

对应变量：`cam_coord` (`PointFloat3`)，由 `BowlFisheyeDewarp::world_coord2cam` 计算：

```cpp
void
BowlFisheyeDewarp::world_coord2cam (const PointFloat3 &cam_world_coord, PointFloat3 &cam_coord)
{
    cam_coord.x = -cam_world_coord.y;
    cam_coord.y = -cam_world_coord.z;
    cam_coord.z = -cam_world_coord.x;
}
```

这一函数只做坐标轴重映射和符号翻转，用于将“以相机为原点的世界点”转换到实际用于成像模型的相机坐标系。

设 Bowl 世界坐标系为 `(X_bowl, Y_bowl, Z_bowl)`，`cam_world_coord` 已经以相机为原点（但仍在 Bowl 轴系标记下），那么上述变换等价于：

```text
X_cam = -Y_bowl
Y_cam = -Z_bowl
Z_cam = -X_bowl
```

- **名称**：相机坐标系（Camera Frame）
- **原点位置**：相机光心
- **坐标轴方向**：
  - Z 轴（光轴方向）：沿 Bowl 系 -X 方向（即车体坐标向后方向）。
  - X 轴：沿 Bowl 系 -Y 方向（向车辆左侧）。
  - Y 轴：沿 Bowl 系 -Z 方向（向下）。
- **单位**：毫米
- **用途**：
  - 这是 Scaramuzza 多项式鱼眼模型的输入坐标系；
  - 后续 `cal_img_coord` / `PolyBowlFisheyeDewarp::cal_img_coord` 中所有几何推导均在此坐标系下进行。

---

## 5. 鱼眼图像平面坐标系（Fisheye Image Plane）

对应变量：`img_coord` (`PointFloat2`)，最终写入 `map_table`：

```cpp
cal_img_coord (cam_coord, img_coord);
map_table[row * tbl_w + col] = img_coord;
```

在 `PolyBowlFisheyeDewarp::cal_img_coord` 中：

1. 计算点到光轴的距离与夹角；
2. 将角度代入多项式 `intr.poly_coeff[]` 得到半径；
3. 用 `(cam_coord.x, cam_coord.y)` 的方向与多项式结果得到像面坐标 `(img_x, img_y)`；
4. 再经由内参仿射参数 `c/d/e` 和主点偏移 `cx/cy` 得到最终像素位置：

```cpp
img_coord.x = img_x * intr.c + img_y * intr.d + intr.cx;
img_coord.y = img_x * intr.e + img_y + intr.cy;
```

- **名称**：鱼眼图像坐标系（Fisheye Image Plane）
- **原点位置**：鱼眼原始图像左上角像素中心（`cx/cy` 在此坐标系下给出主点位置）
- **坐标轴方向**：
  - X 轴：向右为正
  - Y 轴：向下为正
- **单位**：像素
- **用途**：
  - 查找表最终记录的“源图像采样位置”；
  - 在运行时，GeoMapper 根据 `map_table[row, col] = img_coord` 从原始鱼眼图像采样并写入对应 Bowl 视图输出像素。

---

## 6. 与外参标定的关系（简要）

`ExtrinsicParameter` 中的平移与旋转字段：

```cpp
struct ExtrinsicParameter {
    float trans_x, trans_y, trans_z; // mm
    float roll, pitch, yaw;          // degree
};
```

- `trans_x/y/z`：相机光心在 Bowl 世界坐标系下的位置（单位 mm）。
- `roll/pitch/yaw`：通过 `generate_rotation_matrix` 构造旋转矩阵，约定为：
  - 先绕 X 轴旋转 `roll`；
  - 再绕 Y 轴旋转 `pitch`；
  - 最后绕 Z 轴旋转 `yaw`；
  - 得到的 `R` 组合成 `[R | t]`，其逆用于从 Bowl 世界坐标系变换到以相机为原点的坐标。

如果标定工具（例如 OpenCV）输出的是从世界到相机的 `[R_cv | t_cv]`，则需要先做一次变换得到与本实现一致的“相机姿态”表示（即 `[R | t]` 为相机到世界），再分解出对应的欧拉角和平移填入 `ExtrinsicParameter`。

