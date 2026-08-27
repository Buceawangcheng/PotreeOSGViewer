# 第三方软件与资源声明

PotreeOSGViewer 的原创代码采用 BSD-2-Clause 许可证，详见仓库根目录的
[`LICENSE`](LICENSE)。第三方项目和资源仍归各自版权人所有，并继续适用各自
的许可证；PotreeOSGViewer 的 BSD-2-Clause 许可证不会替代这些许可证。

## 直接依赖

### Qt 5.15.2

- 项目地址：https://github.com/qt/qt5
- 使用方式：动态链接 Qt Core、Gui、Widgets 和 OpenGL 模块，并随本地构建结果
  复制相应 DLL 与 Windows platform plugin。
- 许可证：GNU Lesser General Public License v3.0（LGPL-3.0）；Qt 的不同版本和
  模块可能同时提供其他许可选项，应以实际取得的 Qt 软件包为准。
- 版权：Copyright (C) The Qt Company Ltd. 及其他贡献者。
- 随附文本：[`third_party_licenses/Qt-LGPL-3.0.txt`](third_party_licenses/Qt-LGPL-3.0.txt)
  和 [`third_party_licenses/GPL-3.0.txt`](third_party_licenses/GPL-3.0.txt)。

### OpenSceneGraph 3.6.5

- 项目地址：https://github.com/openscenegraph/OpenSceneGraph
- 使用方式：链接 OpenSceneGraph，并在构建结果中复制 OSG 插件。
- 许可证：OpenSceneGraph Public License 1.0，基于 LGPL-2.1，并包含
  wxWindows Library Licence Exception 3.1。
- 版权：Copyright (C) Robert Osfield 及 OpenSceneGraph 贡献者。
- 随附文本：[`third_party_licenses/OpenSceneGraph-LICENSE.txt`](third_party_licenses/OpenSceneGraph-LICENSE.txt)。

### osgQt

- 项目地址：https://github.com/openscenegraph/osgQt
- 使用方式：链接 `osgQOpenGL`，为 Qt 与 OpenSceneGraph 提供集成。
- 许可证：OpenSceneGraph Public License 0.0，包含 wxWindows 例外条款。
- 版权：Copyright (C) Robert Osfield 及 osgQt 贡献者。
- 随附文本：[`third_party_licenses/osgQt-LICENSE.txt`](third_party_licenses/osgQt-LICENSE.txt)。

## 参考项目

### Potree

- 项目地址：https://github.com/potree/potree
- 关系：Potree 2.x 数据读取、八叉树组织、LOD 与流式加载设计的重要参考。
- 许可证：BSD-2-Clause 风格许可证。
- 版权：Copyright (c) 2011-2020, Markus Schütz。
- 随附文本：[`third_party_licenses/Potree-LICENSE.txt`](third_party_licenses/Potree-LICENSE.txt)。

### PotreeConverter

- 项目地址：https://github.com/potree/PotreeConverter
- 关系：生成本项目支持的 Potree 2.x 数据格式，也是格式实现的重要参考。
- 许可证：BSD-2-Clause。
- 版权：Copyright 2020 Markus Schütz。
- 随附文本：[`third_party_licenses/PotreeConverter-LICENSE.txt`](third_party_licenses/PotreeConverter-LICENSE.txt)。

### CesiumJS

- 项目地址：https://github.com/CesiumGS/cesium
- 关系：仅作为 Cesium 风格相机交互体验的设计参考；本项目不链接或分发
  CesiumJS 运行库。
- 许可证：Apache License 2.0。

### vcpkg

- 项目地址：https://github.com/microsoft/vcpkg
- 关系：推荐的 OpenSceneGraph 及其依赖包管理工具；不是本项目运行库的一部分。
- 许可证：MIT License。

## 内置资源

### Google Turbo colormap

- 来源：https://research.google/blog/turbo-an-improved-rainbow-colormap-for-visualization/
- 使用位置：`resources/colormaps/turbo.ppm`。
- 说明：该文件包含 Turbo 色带的采样表示。
- 许可证：Apache License 2.0。
- 版权：Copyright 2019 Google LLC。
- 随附声明：[`third_party_licenses/Turbo-NOTICE.txt`](third_party_licenses/Turbo-NOTICE.txt)。
- 随附许可证：[`third_party_licenses/Apache-2.0.txt`](third_party_licenses/Apache-2.0.txt)。

## 二进制发布说明

本仓库的 CMake 构建会将 Qt、osgQt 以及整个 OpenSceneGraph 插件目录复制到
可执行文件目录。对外发布二进制包前，发布者还需要：

1. 保留本文件、项目 `LICENSE` 和 `third_party_licenses` 目录。
2. 按 LGPL-3.0 要求提供所分发 Qt 版本的对应源代码或有效书面提供方式，允许
   用户替换 Qt 动态库，并且不得禁止用户为了调试其修改版 Qt 而进行必要的
   逆向工程。
3. 根据最终发布包中的 OSG 插件与 DLL，审计并附带全部传递依赖的版权与许可证。
   vcpkg 安装目录下各包的 `share/<package>/copyright` 可作为生成发布清单的来源。

本文件记录的是当前已知的直接依赖、参考项目和内置资源，不是针对任意发布包
的完整法律审计结果。
