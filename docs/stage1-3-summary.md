# 阶段 1-3 完成总结

## 当前状态

PotreeOSGViewer 已完成从基础 Qt/OSG 点云查看器到 Potree 2.x DEFAULT 数据集异步 LOD 流式显示的主链路。

当前 Viewer 保持 `osgViewer::Viewer::SingleThreaded`。文件读取、hierarchy chunk 解析和点数据解码在后台线程执行；`OctreeNode` 状态修改、OSG 对象创建、节点挂接、显隐和淘汰均在主线程的 update traversal 中执行。

## 阶段 1：基础查看器

阶段 1 建立桌面查看器和普通 PLY 点云显示能力：

- 建立 Qt 5.15.2、osgQt 和 OpenSceneGraph 3.6.5 工程。
- 使用 `QMainWindow` 和 `osgQOpenGLWidget` 提供主窗口及三维视图。
- 建立 `osgViewer::Viewer`、根场景和 Trackball 相机控制。
- 支持打开、关闭 PLY 点云。
- 支持旋转、平移、缩放和 Home 定位。
- 关闭传统光照，显示点云原始颜色。
- 支持调整点大小。
- 状态栏显示文件名和点数。
- CMake 构建后部署 Qt、osgQt 和 OSG 插件运行时文件。

## 阶段 2：Potree 数据层和根节点

阶段 2 建立 Potree 2.x CPU 数据模型和基础读取链路：

- 增加 `PointCloudDataset`、`OctreeNode`、`PointAttributes`、`PointCloudNodeData` 和 Provider 边界。
- 支持从数据集目录或 `metadata.json` 打开 Potree 2.x 数据集。
- 解析版本、encoding、点数、spacing、scale、offset、bounds 和属性定义。
- 解析首个 `hierarchy.bin` chunk 和 22 字节 hierarchy record。
- 识别 Normal、Leaf 和 Proxy 节点。
- 按 hierarchy 中记录的 offset/size 精确读取节点数据。
- 支持 DEFAULT encoding 的 position 和 RGB 解码。
- 使用局部顶点坐标和 OSG 平移保留大世界坐标精度。
- 保留 PLY 加载路径，并增加临时 fixture 自动测试。

阶段 2 的提交基线为 `320a9f9 Implement Potree metadata and root node loading`。

## 阶段 3：LOD、异步加载和多节点显示

### 数据模型与 hierarchy

- 将 hierarchy range 和 point-data range 拆分，避免 Proxy offset 被误作 `octree.bin` 范围。
- 将 hierarchy、点数据和 GPU 驻留状态拆分为独立状态维度。
- 增加 `HierarchyPatch`，后台线程只生成 patch，主线程负责应用到真实八叉树。
- 支持按需读取 Proxy hierarchy chunk 并继续展开子 Proxy。
- 本地样例可递归合成 55 个 hierarchy chunks、522 个节点、最大 level 6 和 1,176,615 点。

### 节点选择

- 增加纯 CPU `LodSelector`。
- 使用屏幕投影半径和最大优先队列执行 best-first 选择。
- 支持视锥剔除、最大层级、最小节点像素尺寸和点预算。
- 根节点使用软预算并作为启动保底节点。
- 预算无法容纳当前节点时继续检查后续候选节点。
- 修复空 Potree layer 无法 Home、根节点无法启动加载的问题。

### 异步加载和线程边界

- 增加不可变 `NodeLoadRequest` 和 `NodeLoadResult`。
- 增加带优先级、并发限制和完成队列的 `NodeLoadScheduler`。
- Proxy hierarchy 展开和节点点数据解码组成一个后台 node-ready 请求。
- 工作线程只读取文件、解析 hierarchy 和解码点数据，不持有或修改真实 `OctreeNode`。
- 完成结果在 update traversal 中按 dataset generation、node ID 和 request generation 验证后应用。
- 关闭或切换数据集后，旧异步结果不会挂入新场景。

### 运行时和场景

- 增加 `PointCloudRuntime` 和 `PointCloudUpdateCallback`，每帧协调完成队列、选择、加载、挂接、显隐和淘汰。
- Viewer 明确保持 `SingleThreaded`。
- 增加单帧挂接节点数和挂接字节数限制。
- 增加基础 resident point limit 和 LRU 淘汰。
- 增加 selected、resident、queued、loading、CpuReady、点数、字节数、层级和耗时统计。
- 将单 Potree 节点场景改为多节点 `PotreeSceneBackend`。
- 多节点使用独立父 Transform，可单独显隐和移除。
- Potree layer 在没有已加载 Geometry 时也提供数据集初始 bound，保证首次 Home 正确。

### 调试显示

- 支持 `Original RGB` 和 `LOD Level` 着色模式切换。
- 每个 LOD 节点使用父 `MatrixTransform` 管理点云和包围盒两个子分支。
- 节点父 Transform 隐藏或释放时，点云和包围盒自动同步处理。
- 支持 `Node Bounds` 全局开关，并通过 Camera CullMask 控制，不逐节点查找包围盒。
- 包围盒使用对应层级颜色，便于检查当前选择的节点范围和 LOD。

## 自动测试和构建验证

阶段 3 完成后已执行：

```powershell
cmake --build build-codex --config Debug --target PotreePointCloudTests
ctest --test-dir build-codex -C Debug --output-on-failure
cmake --build build-codex --config Debug --target PotreeOSGViewer

cmake --build build-codex --config Release --target PotreePointCloudTests
ctest --test-dir build-codex -C Release --output-on-failure
cmake --build build-codex --config Release --target PotreeOSGViewer
```

Debug 和 Release 自动测试均为 1/1 通过，两个配置的 GUI 目标均编译成功。

测试覆盖：

- metadata、hierarchy record 和 DEFAULT 点数据解码。
- 无效文件、range、encoding 和 hierarchy chunk 错误。
- Proxy patch 和本地样例完整 hierarchy 合成。
- 投影权重、视锥、根节点保底、点预算和最小像素阈值。
- 调度器完成队列以及工作线程不修改真实节点状态。
- 多节点挂接、显隐、颜色模式和移除。
- 点云/包围盒父子结构及父节点释放后的子树生命周期。

## 当前限制与后续方向

- BROTLI encoding 尚未实现。
- DEFAULT 目前只将 position 和 RGB 转为渲染数据，其他属性仅保留 metadata 布局。
- 高度、强度、分类等 shader 属性着色尚未实现。
- EDL 屏幕后处理尚未实现。
- PLY 仍为同步加载。
- 当前是基础点数限制和 LRU，尚未实现完整 CPU/GPU 字节级多级缓存。
- 失败节点尚未实现有限次数重试和退避策略。
- 正交相机 LOD、多点云全局预算和共享 OpenGL 上传线程不在阶段 3 范围内。

下一阶段计划建立统一点云 GLSL shader、属性着色和 EDL 渲染链路，BROTLI 支持暂缓。
