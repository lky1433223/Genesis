# CLAUDE.md

本文档为 Claude Code (claude.ai/code) 在使用本仓库代码时提供指导。
本项目并非开发项目，而是研究项目。我正在研究源码。所以请勿对main分支代码做出任何修改。
我创建了research_with_notes分支，允许在该分支添加注释，撰写文档。但仍然不允许更改业务代码。

## 项目概述

Genesis 是一个通用物理引擎和仿真平台，用于机器人、具身智能和物理 AI 应用。它将 6+ 种物理求解器（刚体、MPM、SPH、FEM、PBD、SF）集成到统一框架中，在 RTX 4090 上模拟机械臂可达到超过 4300 万 FPS。

核心功能：
- **物理仿真**：多求解器架构，支持刚体、可变形体、流体和软材料
- **照片级真实感渲染**：光线追踪和光栅化渲染器
- **可微物理**：MPM 求解器支持梯度计算，适用于机器学习应用
- **跨平台**：支持 Linux、macOS、Windows，提供 CPU/GPU/Metal/CUDA 后端

## 开发命令

### 构建和安装

```bash
# 以开发模式安装（克隆后）
pip install -e ".[dev]"

# 拉取新更改后更新依赖项
pip install -e ".[dev]"

# 构建文档
pip install -e ".[docs]"
cd doc && make html
```

### 代码质量

```bash
# 设置 pre-commit 钩子（提交前必需）
pip install pre-commit
pre-commit install

# 手动运行代码格式化工具
black genesis/
# 注意：外部依赖（genesis/ext）不包含在格式化范围内
```

### 测试

```bash
# 运行所有测试（多进程，自动分布式）
pytest

# 运行最小必需测试集（合并前检查）
pytest -v --forked -m required ./tests

# 运行特定测试文件
pytest tests/test_rigid_physics.py

# 使用特定标记运行测试
pytest -m "not benchmarks"  # 跳过基准测试
pytest -m "not examples"    # 跳过示例

# 在特定后端上运行测试
GS_BACKEND=cpu pytest
GS_BACKEND=gpu pytest
```

测试标记：
- `required`：合并前必须通过的最小测试集
- `examples`：示例脚本验证
- `benchmarks`：性能分析测试
- `field_only`：与 ndarray 后端不兼容的测试

### 运行示例

```bash
# 运行示例脚本
python examples/rigid/urdf_robot.py

# 示例位于：
#   examples/rigid/      - 刚体仿真
#   examples/mpm/        - 物质点法演示
#   examples/sph/        - 流体仿真
#   examples/fem/        - 有限元仿真
#   examples/pbd/        - 基于位置的动态仿真
#   examples/hybrid/     - 多求解器耦合仿真
```

## 架构概述

### 核心引擎结构

```
genesis/
├── __init__.py              # 主 API 入口点
├── engine/
│   ├── scene.py            # 核心场景类 - 一切发生在此
│   ├── simulator.py        # 主仿真器，管理所有求解器
│   ├── solvers/            # 物理求解器（刚体、MPM、SPH、FEM、PBD、SF、工具求解器）
│   ├── entities/           # 物理实体（机器人、无人机等）
│   ├── materials/          # 每个求解器的材料属性
│   ├── couplers/           # 求解器间耦合（如刚体-MPM 交互）
│   ├── sensors/            # 相机、激光雷达、力传感器
│   └── mesh.py             # 网格处理和工具
├── options/                # 所有组件的配置类
├── vis/                    # 可视化和渲染系统
├── grad/                   # 可微物理操作
├── recorders/              # 数据记录和视频导出
└── ext/                    # 外部依赖和渲染器
    ├── LuisaRender/        # 光线追踪渲染器
    ├── pyrender/           # 光栅化渲染器
    └── urdfpy/             # URDF 解析
```

### 关键设计模式

1. **场景为中心的架构**：所有内容都在 `gs.Scene` 内部发生。场景包含：
   - 仿真器（管理所有物理求解器）
   - 实体（物理对象）
   - 可视化器（相机、查看器）
   - 传感器（数据采集）

2. **求解器抽象**：每个物理求解器（刚体、MPM、SPH 等）继承自具有通用接口的基类：
   - `build()` - 初始化求解器
   - `step()` - 推进仿真
   - `couple()` - 处理求解器间交互

3. **实体系统**：实体从形态（MJCF、URDF、基元、网格）创建，并与特定求解器关联。实体维护自身的状态和几何信息。

4. **材料系统**：每个求解器有自己的材料类型（如 `gs.materials.MPM.Elastic`、`gs.materials.Rigid.Frictionless`）。材料定义物理属性。

5. **耦合系统**：启用不同物理求解器间的交互（如刚体机器人操作可变形物体）。由耦合器类处理，传输力和约束。

### 多求解器耦合

Genesis 的独特能力是耦合多个物理求解器。关键耦合类型：
- **刚体-MPM**：机器人操作可变形物体
- **刚体-SPH**：机器人与流体交互
- **刚体-PBD**：机器人与软体交互
- **MPM-SPH**：颗粒介质与流体（湿砂）

耦合器处理：
- 求解器域之间的碰撞检测
- 求解器之间的力传输
- 域分解和边界条件

### Taichi 后端

Genesis 使用 Taichi（gstaichi==3.0.0）作为计算后端，用于：
- GPU/CPU 上的高性能并行内核
- 内存管理和数据结构
- 自动微分（用于可微物理）

关键配置：
- `GS_BACKEND`: cpu/gpu/metal/cuda
- `GS_ENABLE_NDARRAY`: 0/1（数据结构类型）
- `TI_OFFLINE_CACHE`: 启用内核缓存

## 跨平台考虑

- **Linux**：主要开发平台，完整 GPU 支持
- **macOS**：Metal 后端，CPU 回退
- **Windows**：直接计算支持
- **CI 测试**：三个平台在 Python 3.10-3.13 上均进行测试

## 性能优化

本代码库优先考虑极致性能：
- Taichi 中的内核融合以提高计算效率
- 缓存友好的数据布局
- 热循环中最小化 Python 开销
- GPU 内存管理和缓存
- 跨平台 SIMD 利用

进行更改时：
- 在 CPU 和 GPU 后端上进行性能分析
- 考虑内存带宽和缓存局部性
- 测试对基准测试的性能影响
- 在优化内部实现时保持 API 简洁性

## 调试和开发技巧

- 使用 `gs.init(debug=True)` 启用调试模式和更详细的日志记录
- 设置 `GS_LOGGING_LEVEL=DEBUG` 获取详细日志
- 对于渲染问题，使用 `gs.init(backend=gs.cpu)` 测试以隔离 GPU 问题
- 使用 `scene.simulator.profiler.print()` 查看求解器性能分解

## 常见陷阱

1. **后端不匹配**：开发跨平台功能时务必验证后端兼容性
2. **内存管理**：Taichi 内核需要仔细的内存管理 - 检查长时间运行的仿真中的泄漏
3. **耦合顺序**：求解器耦合必须按正确顺序进行 - 刚体 → 可变形体 → 流体
4. **材料属性**：材料参数必须在有效范围内以确保数值稳定性
5. **时间步长**：不同求解器可能需要不同的子步策略以确保稳定性
- 请不要对源码做任何修改