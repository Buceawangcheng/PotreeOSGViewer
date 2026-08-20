# PotreeOSGViewer

独立 Qt + osgQt + OpenSceneGraph 点云查看器工程。当前已完成基础 PLY 查看以及 Potree 2.x DEFAULT 数据集的异步 LOD 流式显示链路。

阶段 1-3 的完成内容、测试证据和当前限制见 [`docs/stage1-3-summary.md`](docs/stage1-3-summary.md)。

## 已确认依赖

- Visual Studio 2019 x64
- Qt 5.15.2 MSVC2019 64-bit: `D:/Qt/MyQt/5.15.2/msvc2019_64`
- vcpkg: `D:/wc/vcpkg/vcpkg`
- OpenSceneGraph 3.6.5: `D:/wc/vcpkg/vcpkg/installed/x64-windows`
- osgQt:
  - Debug: `D:/wc/software/AI/osg/osgQt-master/install/debug`
  - Release: `D:/wc/software/AI/osg/osgQt-master/install/release`

## 生成 VS2019 工程

```powershell
& "D:\Qt\MyQt\Tools\CMake_64\bin\cmake.exe" -S . -B build -G "Visual Studio 16 2019" -A x64
```

## 编译

```powershell
& "D:\Qt\MyQt\Tools\CMake_64\bin\cmake.exe" --build build --config Release
& "D:\Qt\MyQt\Tools\CMake_64\bin\cmake.exe" --build build --config Debug
```

## 测试

```powershell
& "D:\Qt\MyQt\Tools\CMake_64\bin\ctest.exe" --test-dir build -C Debug --output-on-failure
```

测试主要使用运行时生成的小型 Potree fixture；如果存在本地 `data/` 样例，还会验证完整 hierarchy 统计。`data/` 不会提交。

## 运行

从 Visual Studio 启动 `PotreeOSGViewer` 时，CMake 已设置调试环境变量：

- `PATH`
- `OSG_LIBRARY_PATH`

命令行运行 Release 示例：

```powershell
.\build\Release\PotreeOSGViewer.exe
```

构建后会把必要的 Qt、osgQt DLL 和 `osgPlugins-3.6.5` 复制到 exe 目录，方便直接从 `build/Release` 或 `build/Debug` 运行。

## 当前功能

- QMainWindow 主窗口
- osgQOpenGLWidget 中央三维窗口
- osgViewer::Viewer + osg::Group 根场景
- 打开/关闭 PLY 点云
- Trackball 鼠标旋转、平移、缩放
- 关闭传统光照，保留点云原始 RGB 显示
- 点大小调整
- 状态栏显示文件名和点数
- 从数据集目录或 `metadata.json` 打开 Potree 2.x 数据
- 解析 metadata、首个 hierarchy chunk，并按需展开 Proxy hierarchy
- 按 hierarchy 的 offset/size 异步读取和解码 `octree.bin` 节点
- best-first LOD 选择、视锥剔除、点预算和最小像素阈值
- 多节点渐进挂接、显隐、基础 LRU 和 generation 失效保护
- 解码 DEFAULT 编码的 position 和 RGB
- 使用局部坐标顶点和场景平移保留大坐标精度
- 原始 RGB / LOD 层级着色切换
- 节点包围盒显示开关
- Viewer 保持 SingleThreaded，OSG 场景修改集中在 update traversal

## 当前限制

- PLY 仍是同步加载，加载大文件时界面可能短暂阻塞。
- 点数统计按加载后 geometry 的 vertex array 数量估算。
- BROTLI 点数据解码尚未实现；BROTLI metadata 和 hierarchy 仍可读取。
- DEFAULT 的其他属性会保留布局信息，但目前只将 position 和 rgb 转成渲染数据。
- 高度、强度、分类属性着色和 EDL 尚未实现。
- 当前仅有基础点数限制和 LRU，尚未实现完整字节级多级缓存。
- 失败节点尚未实现有限次数重试和退避策略。
