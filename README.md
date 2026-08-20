# PotreeOSGViewer

独立 Qt + osgQt + OpenSceneGraph 点云查看器工程。当前实现范围是 `规划.txt` 中的阶段0和阶段1。

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

## 当前限制

- PLY 仍是同步加载，加载大文件时界面可能短暂阻塞。
- 点数统计按加载后 geometry 的 vertex array 数量估算。
- 阶段2以后的八叉树、LOD、异步加载、缓存管理尚未实现。
