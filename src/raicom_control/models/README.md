# 本地生成机器人模型

公开版本不分发厂商派生的 landerpi_lightweight.urdf 及完整尺寸清单。生成器保留在 raicom_control/lightweight_urdf.py，测试使用合成 URDF / STL。

按照仓库 docs/DEPENDENCIES.md，使用自己合法取得的展开厂商模型和 meshes 在本地生成 landerpi_lightweight.urdf，然后构建安装。生成结果与 manifest 不提交。没有模型时不要启动 lightweight_robot_description.launch.py。
