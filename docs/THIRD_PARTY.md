# 来源与许可

用户确认将自研功能层 `competition_manual`、`rgbd_grasp`、`raicom_control` 及其中的原生导航插件以 Apache-2.0 发布。各包的 `package.xml`、Python 打包声明和附带 LICENSE 保持一致。原工作区的声明未被修改。

## 保留的第三方内容

| 内容 | 来源与适用许可 |
|---|---|
| bondcpp 3.0.2 补丁中的上游上下文 | [ros/bond_core 3.0.2](https://github.com/ros/bond_core/tree/3.0.2/bondcpp)，保留 `src/raicom_control/compat/bondcpp/LICENSE` 中 BSD 三条款声明 |

`deploy/bondcpp_3_0_2_wait_lock.patch` 的修改范围、ABI 限制和本地构建方法见其兼容层 README。没有附带 `libbondcpp.so`；不能假定本机任意构建与历史摘要一致。

## 外部取得的依赖

厂商 `driver`、`app`、`peripherals`、`landerpi_description` 等包没有复制进首发仓库。原工作区中大量包尚无明确许可正文，用户需要从合法来源安装并遵守其条款。

Aurora930 驱动包 `deptrum-ros-driver-aurora930` 也是外部依赖，当前工作区副本未包含驱动源码。

讯飞 SDK 头文件包含另行许可和保留权利声明。讯飞包、动态库、识别资源和授权缓存没有复制。Vosk 模型、ONNX 模型和反馈音频也不随此版本分发。

`landerpi_lightweight.urdf` 由厂商模型派生，仍保留厂商关节、惯性和 link 信息，因此不公开生成结果及完整尺寸清单。仓库保留生成器和合成数据测试，用户可在本机使用其合法取得的输入生成。

`vendor_*.patch` 和 YOLO Web 补丁中的厂商上下文未复制。SLAM Toolbox heading-gate 工具没有捆绑上游源码或包含其源码上下文的补丁生成器。上游 2.6.8 仓库提供 LGPL 2.1 许可文件，源文件另有原作者声明；本仓库不为其重新授权。历史私有证据目录不在本仓库中，复现前必须另行取得精确上游输入并检查许可和摘要。

此文件记录本次发布边界，不替外部软件重新授予权利。
