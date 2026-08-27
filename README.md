# PotreeOSGViewer

## 项目基础与致谢

本项目的实现建立在以下开源项目和技术工作之上，感谢这些项目的维护者与贡献者：

- [Qt](https://github.com/qt/qt5)：提供桌面界面、事件系统和 OpenGL 上下文。
- [OpenSceneGraph](https://github.com/openscenegraph/OpenSceneGraph)：提供场景图、相机、点云文件读取和 OpenGL 渲染框架。
- [osgQt](https://github.com/openscenegraph/osgQt)：提供 Qt 与 OpenSceneGraph 的集成，本项目使用其中的 `osgQOpenGLWidget`。
- [Potree](https://github.com/potree/potree)：Potree 2.x 数据读取、八叉树节点组织、LOD 选择和流式加载设计的重要参考。
- [PotreeConverter](https://github.com/potree/PotreeConverter)：生成本项目支持的 Potree 2.x `metadata.json`、`hierarchy.bin` 和 `octree.bin` 数据集。
- [CesiumJS](https://github.com/CesiumGS/cesium)：Cesium 风格相机交互的体验参考；本项目不依赖 CesiumJS 运行库。
- [vcpkg](https://github.com/microsoft/vcpkg)：推荐用于安装和管理 OpenSceneGraph 及其依赖。

各项目的版权与许可证归其原作者所有，使用和分发时请遵守对应项目的许可证。

## 编译

### 依赖关系

- Windows x64
- Visual Studio 2019（MSVC v142）
- CMake 3.20 或更高版本
- Qt 5.15.2 MSVC2019 64-bit：提供窗口、控件和 OpenGL 上下文。
- OpenSceneGraph 3.6.5：提供场景图、点云读取和渲染能力。推荐使用 vcpkg 的 `x64-windows` triplet 安装。
- osgQt：连接 Qt 与 OpenSceneGraph。osgQt 必须使用与本项目相同的 Qt、OpenSceneGraph、编译器和 x64 架构构建。

依赖链为 `PotreeOSGViewer -> Qt + osgQt -> OpenSceneGraph`。所有依赖的 Debug/Release 配置必须匹配，不能混用不同配置的库。

推荐先使用 vcpkg 安装 OpenSceneGraph：

```powershell
$env:VCPKG_ROOT = "<vcpkg>"
& "$env:VCPKG_ROOT/vcpkg.exe" install osg:x64-windows
```

随后单独构建 osgQt，并形成以下安装目录：

```text
<osgQt>/install/
|-- debug/
|   |-- bin/
|   |-- include/
|   `-- lib/
`-- release/
    |-- bin/
    |-- include/
    `-- lib/
```

### 生成与构建

将占位路径替换为本机实际安装位置：

```powershell
$env:VCPKG_ROOT = "<vcpkg>"

cmake -S . -B build `
  -G "Visual Studio 16 2019" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" `
  -DQT_ROOT_DIR="<Qt>/5.15.2/msvc2019_64" `
  -DOSGQT_ROOT="<osgQt>/install"

cmake --build build --config Release
```

`CMAKE_TOOLCHAIN_FILE` 未显式传入时，工程也会尝试从 `VCPKG_ROOT` 环境变量自动定位。非标准安装布局还可配置 `OSGQT_DEBUG_ROOT`、`OSGQT_RELEASE_ROOT`、`VCPKG_INSTALLED_DIR`、`OSG_INCLUDE_DIR`、`OSG_PLUGINS_DEBUG` 和 `OSG_PLUGINS_RELEASE`。

如需 Debug 版本：

```powershell
cmake --build build --config Debug
```

构建完成后，Qt、osgQt 运行库以及 `osgPlugins-3.6.5` 会自动复制到可执行文件目录。

## 已实现功能

- 基于 Qt、osgQt 和 OpenSceneGraph 的桌面点云查看器。
- 打开、显示和关闭 PLY 点云。
- 打开 PotreeConverter 2.x `DEFAULT` 编码数据集，支持选择数据集目录或 `metadata.json`。
- 解析 `metadata.json`、`hierarchy.bin` 和 `octree.bin`，按需展开 Proxy hierarchy。
- 持久工作线程异步读取、解析和解码节点，主线程集中完成 OSG 场景更新。
- best-first LOD 选择、视锥剔除、点预算、最小屏幕像素阈值和渐进式节点挂接。
- resident 节点缓存、LRU 淘汰以及 generation 失效保护。
- 解码并渲染位置和 RGB，使用局部坐标顶点与场景平移保留大坐标精度。
- 原始 RGB、LOD 层级和高度三种着色模式。
- 圆形点渲染、点大小调节和节点包围盒显示。
- OSG Trackball 与 Cesium 风格相机控制器切换。
- Cesium 风格旋转、平移、以鼠标位置为中心的缩放，以及基于深度缓冲的拾取。
- 动态 near/far 投影范围和拾取位置调试显示。
- 状态栏显示当前数据集、点数和 FPS。
- Viewer 使用 `SingleThreaded`，后台线程只执行文件读取、hierarchy 解析和点数据解码。

## 未实现功能

- Eye-Dome Lighting（EDL）。
