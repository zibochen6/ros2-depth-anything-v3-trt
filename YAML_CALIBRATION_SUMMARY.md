# YAML 相机标定功能总结

## 修改内容

### 1. 新增文件

#### camera_info_example.yaml
- 包含你的鱼眼相机标定参数
- 格式：标准 YAML 格式
- 参数：fx, fy, cx, cy, k1, k2, p1, p2, k3

#### CAMERA_CALIBRATION_FILE.md
- 完整的使用文档
- 包含使用方法、故障排除、示例等

### 2. 修改的文件

#### depth_anything_v3/src/camera_depth_node.cpp
- 添加 `#include <yaml-cpp/yaml.h>` 头文件
- 添加 `camera_info_file` 参数
- 添加 `loadCalibrationFromFile()` 函数
- 修改构造函数，优先从 YAML 文件加载参数

#### depth_anything_v3/CMakeLists.txt
- 添加 `find_package(yaml-cpp REQUIRED)`
- 在 `camera_depth_node` 链接库中添加 `yaml-cpp`

#### depth_anything_v3/package.xml
- 添加 `libyaml-cpp-dev` 依赖

#### depth_anything_v3/launch/camera_depth_rviz.launch.py
- 添加 `camera_info_file` 启动参数
- 将参数传递给节点

#### run_camera_depth.sh
- 添加对 `CAMERA_INFO_FILE` 环境变量的支持
- 优先使用 YAML 文件，其次使用硬编码参数

## 使用方法

### 方法 1：使用 YAML 文件（推荐）

```bash
# 使用默认的 camera_info_example.yaml
CAMERA_INFO_FILE=camera_info_example.yaml ./run_camera_depth.sh
```

### 方法 2：使用环境变量（旧方法，仍然支持）

```bash
USE_CALIBRATION=1 ./run_camera_depth.sh
```

### 方法 3：不使用标定

```bash
./run_camera_depth.sh
```

## 参数优先级

1. **YAML 文件** - 如果指定了 `CAMERA_INFO_FILE` 且文件存在
2. **环境变量** - 如果设置了 `USE_CALIBRATION=1`
3. **默认值** - 代码中的默认值（估计值）

## YAML 文件格式

```yaml
# 图像分辨率
image_width: 1920
image_height: 1536

# 相机内参
fx: 824.147361
fy: 823.660879
cx: 958.275200
cy: 767.389372

# 鱼眼畸变系数
distortion_model: "fisheye"
k1: 1.486308
k2: -13.386609
p1: 21.409334
p2: 3.817858
k3: 0.0
```

## 验证

启动节点时，会在日志中显示：

```
[camera_depth_node]: Loading calibration from: camera_info_example.yaml
[camera_depth_node]: ✓ Calibration loaded successfully
[camera_depth_node]:   fx=824.15, fy=823.66, cx=958.28, cy=767.39
[camera_depth_node]:   Distortion: k1=1.4863, k2=-13.3866, p1=21.4093, p2=3.8179, k3=0.0000
```

## 优势

1. ✅ **易于管理** - 所有参数集中在一个文件中
2. ✅ **易于共享** - 可以轻松分享标定文件
3. ✅ **版本控制** - 可以用 Git 跟踪标定文件的变化
4. ✅ **多相机支持** - 每个相机可以有独立的标定文件
5. ✅ **无需重新编译** - 修改参数不需要重新编译代码
6. ✅ **向后兼容** - 旧的使用方法仍然有效

## 测试步骤

1. 确保已编译代码：
```bash
colcon build --packages-select depth_anything_v3
source install/setup.bash
```

2. 使用 YAML 文件运行：
```bash
CAMERA_INFO_FILE=camera_info_example.yaml ./run_camera_depth.sh
```

3. 检查日志输出，确认标定参数已从文件加载

4. 在 RViz 中查看点云，确认没有重叠

## 创建自定义标定文件

```bash
# 复制示例文件
cp camera_info_example.yaml my_camera.yaml

# 编辑文件，填入你的标定参数
nano my_camera.yaml

# 使用自定义文件
CAMERA_INFO_FILE=my_camera.yaml ./run_camera_depth.sh
```

## 故障排除

### 文件未找到
- 检查文件路径是否正确
- 使用绝对路径或相对于工作目录的路径

### YAML 格式错误
- 检查 YAML 语法
- 确保所有参数都是数字类型
- 检查缩进是否正确

### 点云仍然重叠
- 确认标定参数正确
- 检查相机分辨率是否匹配
- 验证降采样设置

## 下一步

- 为每个相机创建独立的标定文件
- 在多相机系统中使用不同的标定文件
- 定期重新标定相机以保持精度
