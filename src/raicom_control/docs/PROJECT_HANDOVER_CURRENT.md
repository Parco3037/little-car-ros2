# RAICOM2026 唯一当前交接

## 48. 2026-09-20 恢复次数上限可在工作台修改，默认10次

用户指明截图中的“导航连续恢复超过2次，已停止”规则需要列在界面中，可自行修改，默认10次。本节先于实现写入：将该写死阈值改为工作台可保存的非负整数设置，默认10；按原“超过”语义，10次以内继续，第11次停止，0表示首次恢复反馈即停。显示本目标当前次数与本轮生效上限，进入下一目标重新计数。核对Nav2内部是否有另一层更小恢复预算，不把只改文字当完成。保存、重新加载、开始/继续导航必须使用一致值；保留到点标准、超时、定位/通信失效、人工急停等其他条件，不扩大为所有失败无限重试。新设置与已有地图点位/速度设置兼容，原坐标及历史保留。先本地与Pi隔离验证，再精选安装，不启动实体运动。证据`runtime_evidence/20260920_191401_recovery_limit/`。

实现阶段已完成：新增“导航恢复次数”卡片、输入框及“保存次数”，旧路线无字段时默认10；保存进workspace.max_recoveries及历史，开始/继续时采用，执行中冻结本轮上限。当前次数按Nav2真实number_of_recoveries反馈显示，不把该计数说成车辆成功脱困次数。达到10仍继续、超过10取消，0表示首次恢复即停。修复重新开始且尚无到点记录时残留上一轮失败文案的问题。

同时核验Pi原单目标BT外层NavigateRecovery固定6轮：每个目标派生原树的副本，只将该外层预算设为上限+1，原规划、跟踪、清图/转向/等待动作及内层恢复不改；通过NavigateToPose.behavior_tree指定副本，原厂XML不覆盖。反馈总次数仍由工作台上限判断，其他原生恢复动作失败/超时等仍可提前结束。Humble 1.1.15依据：[RecoveryNode](https://github.com/ros-navigation/navigation2/blob/1.1.15/nav2_behavior_tree/plugins/control/recovery_node.cpp)、[单目标BT加载和反馈](https://github.com/ros-navigation/navigation2/blob/1.1.15/nav2_bt_navigator/src/navigators/navigate_to_pose.cpp)。Context7本轮配额耗尽，改核对官方对应版本源码。

本地86项及21个subtest、Pi原生86项通过。私有DDS87只启动真实bt_navigator、模拟规划/跟踪/恢复服务及虚拟TF，未启动底盘/guard/机械臂或任何速度publisher。实际计数0→1、10→11、17→18分别触发取消；默认10完整观察到1–10继续，第11次CANCELED，17同理至第18次，目标索引不推进、运动许可撤销。候选UI原生Tk预览通过，安装阶段待追加。原启动时序/2cm/10°/stateful=false/速度设置保持。

**第48节最终状态：已安装到Pi并核验生产实际导入，默认10次；部署时工作台未运行，下次正常打开即加载。** boot复核仍为`1b3ea930-0025-4e24-b6ed-666b20b7661a`，部署前后103条底盘输出全零、无活动目标；没有重启任何进程，没有实体目标/速度/舵机命令。56份现场地图、点位、历史及保存速度文件逐字节不变。安装后实际导入均指向production build→source，默认BT派生文件在`/home/ubuntu/raicom_runtime/navigation_bt/`，默认10的XML SHA256为`ae52c8c7035575938dda35dfe917777d217ed45aef95d0fdee2130a2cccd4c6b`。用生产安装模块再次完成0/10/17三组真实BT反馈与取消隔离测试，分别1/11/18取消。这里只验证恢复次数设置和真实BT协议，不是现场脱困/导航精度验收。证据见本节目录`before_read.stdout`、`local_tests.txt`、`recovery_fields.png`、`deployment.stdout`、`installed_check.stdout`、`results.zip`；精选备份在`results/atomic_before.zip`。main/HEAD仍dbac0a6，既有未提交修改保留。

## 47. 2026-09-20 用户要求自主应用速度及减速距离

用户明确要微调0.5m/s、减速距离0.1m可保存并实际应用，其余各阶段数值自行填写，建议值不能作为拒绝应用条件。本节先于实现写入：移除旧fine≤0.08、near≤1、速度必须递减和near_distance=0.20–0.25的强制校验，同步修改原生控制器，而不只改界面。保留此前明确的线/角软件上限2.0、有限数值校验、停车应用/原服务激活与实际回读、原加减速度、倒退0.08、唯一guard、失联急停及2cm/10°/stateful=false。减速距离允许非负值，0表示关闭该接近段；若短于原6cm微调段，微调段取两者较小值。各阶段可独立设置，三节点上限按三种线速度最大值协调，不能悄悄截为巡航值。仅对参数能否应用及隔离执行链验收，不把0.5m/s末端实车精度写成已通过。证据`runtime_evidence/20260920_185201_free_speed/`；相机第46节问题继续处理，不被此项替代。

本地实现已完成并通过66项测试及16个subtest；Pi Humble独立候选编译与C++策略测试已通过。私有DDS87实际写入/回读巡航2、接近1、微调0.5m/s、减速距离0.1m、角速度1rad/s；原生控制器和隔离guard输出均实测达到0.5m/s，取消返回CANCELED且归零。接近速度高于巡航、减速距离0/0.03/3m和小于旧推荐下限的速度均通过三节点回读，随后恢复隔离默认值。当前还在完成基线回归及部署，不能当成生产已加载或实车精度验收。

**第47节最终状态：自主调速修改已安装到Pi，现有GUI PID192642仍为部署前进程，需要停车后正常重开加载。没有自动重开会执行机械臂回位的桌面入口，没有实体行驶或机械臂动作。** 部署前157条底盘输出全零、无活动目标；只结束当前GUI拥有的闲置导航launch（PID192905），AMCL、底盘驱动和唯一guard保持原进程。源文件/原生库精选更新，实际Python导入build→source指向新内容，安装库SHA256 `8645b7f7ae7155b18a25d2e2e9f1561ac071ac6891d5157c50d576ffb499f357`，用新inode原子替换。56份地图/点位/历史等field_tool_data文件逐字节未变，既有保存速度未重写。

完整候选隔离回归通过：2cm/10°基线接近、直行、先转向/再前进/末端修角均SUCCEEDED，模拟最大停车误差1.730cm/8.594°；前进与转向取消均CANCELED且归零，原障碍减速/车体碰撞停车检查仍通过。安装后使用生产实际导入与生产原生库再测0.5/0.1，真实ROS隔离控制器及guard均输出0.5、取消归零，三节点各字段回读一致。Pi原生Tk的无执行器预览已人工看图，所有五项单位/建议/输入/应用按钮完整可见。以上仍不等于末端0.5m/s实车精度或高速运输验收。证据包括本节目录`local_tests.txt`、`speed_fields.png`、`deployment.stdout`、`installed_check.stdout`、`results.zip`及其解压内容；生产修改前完整精选备份`results/atomic_before.zip`。main/HEAD仍dbac0a6，未提交修改全部保留。

## 46. 2026-09-20 用户指出嵌入画面暗紫、无法看清现场

第45节只验证取图/传输/嵌入链路，未验证相机画面可用于现场识别；用户明确指出暗紫画面不可用，本轮承认不能把链路通过当成画面验收。先只读比较原相机8000/api/temp_image、YOLO8080/latest_result.jpg及工作台画面，不自动改曝光、白平衡、模型或重启原容器，不发送底盘/机械臂命令。原相机/YOLO冻结要求继续有效；所有既有修改及现场资产保留。证据`runtime_evidence/20260920_184542_dark_camera/`。

本轮三处均为同样的暗紫、无法辨认场景的图像；原JPEG直接查看同样异常，排除“只有Tk显示变暗”这一解释，尚不能确认镜头遮挡/朝向/光照/相机参数中哪一项是根因。当前原HTTP参数为648×486、曝光30000μs、模拟增益2.5、红/蓝增益1.3/1.5、saturation/contrast=1、preview_rotate=false；实际代码关闭自动曝光和自动白平衡。代码/参数SHA与上一节相同，未读取到实际曝光元数据，不能把参数文件当作传感器已生效回读。SSH boot仍`1b3ea930-0025-4e24-b6ed-666b20b7661a`。已向现场询问接排线单目镜头是否有镜头盖/保护贴、被车体或机械臂挡住，等待反馈；不猜测物理状态。

用户反馈单目镜头无遮挡、正对明亮场景。通过现有ROS适配器新拍一帧仍异常，平均亮度27.74/255、88.87%像素低于32，无检测；JPEG EXIF为ExposureTime=0.029968s、ISO250，与现配置基本一致，不能再把“设置未生效”当已证实根因。相机实际OV5647，输出648×486时选中640×480传感器模式。用户已明确授权两组临时参数对照：1296×972、以及原分辨率下60ms/4倍增益；结束恢复全部原参数，模型/代码/容器仍冻结。对照脚本及写后超时/识别失败恢复测试3项已通过，实机对照结果待追加。

授权临时对照已完成，全部原参数已恢复：API读回九项一致，params.json SHA256仍`9f994eed255a1a64bd5731bdd4a1cee57e14127329d0dbdd2951c5a8692d498c`；相机和推理代码SHA与对照前一致，模型/代码/容器配置未改。三张对照均为空检测。重要观察：对照开始前尚未改参数的baseline已经能看清地砖；1296×972同样可见地面，60ms/4倍增益更明亮，但仍有偏色及暗边。因此不能将画面从暗紫恢复归因于本轮调参，根因尚未证实。恢复后经同一ROS适配器新拍一帧平均亮度49.60/255、9.62%像素低于32，仍无T/G检测，不算分类或园区映射验收。最终对照、原参数、恢复后照片及指纹在本节证据目录`final/`、`camera_trial_final.zip`、`restore_final.json`；初始暗图保留在原`results/fresh.jpg`，未被恢复后帧覆盖。本轮没有底盘或机械臂动作。

## 45. 2026-09-20 识别卡片嵌入带检测框的相机画面

用户明确要求在工作台内查看带检测框的相机画面、类别及园区对应关系。本节先于实现写入：沿用已安装的ROS2视觉适配和原HTTP服务，在同一请求内配对照片与检测结果，增加嵌入预览、类别/置信度、左右园区候选和更新时间；正式映射仍需识别区停车及三次有效一致，独立预览不采用映射、不发导航/夹取。取消、失败、空检测及旧图不能显示为本轮有效识别。相机/YOLO代码、模型、参数和容器配置冻结，原建图/采点/导航和现场资产保留。先离线测试及Pi隔离验证，随后精选部署；不自动重开会触发机械臂动作的工作台。证据目录`runtime_evidence/20260920_182719_embedded_vision/`。当前尚未实现/部署本节功能。

实现/验证阶段：新增嵌入式DetectionPreview、独立“开始/停止画面预览”及自动识别采样显示。两次预览请求之间至少空闲1秒，导航/联动等工作优先并取消独立预览。原推理成功后在同一跨进程锁内GET输入JPEG，通过独立CompressedImage话题传图；JSON只携带时间戳和SHA256，客户端核对当前请求、时间戳及照片哈希后配对，避免JSON/日志携带整张图。配图失败只影响显示，不丢弃原识别结果；未知/blue/空帧不建立候选映射。界面标出候选、正式三次一致结果和图像年龄，超过5秒显示旧帧。原网页/其他不使用适配锁的客户端仍不能与联动同时抓拍，原HTTP服务没有曝光时间戳，未伪称硬件同步。

本地92项及16个subtest、Pi原生92项通过；私有DDS88中两个实际适配节点调用原HTTP服务，成功配对648×486/10368字节JPEG，detections为空，因此只通过取图/传输/显示链路，不是分类验收。Pi原生Tk已检查建图、采点、嵌入画面、T/G左右互换模拟框、真实空检测画面、旧帧和取消；模拟框均标明示例，测试窗口使用Mock节点、禁用操作按钮，不启动原实体GUI/机械臂。Context7配额耗尽后核对Pillow官方ImageTk/Image文档，Pi已有Pillow10.3.0，无需改相机环境。此阶段尚未写生产源文件；后续追加部署回执。

**第45节最终状态：嵌入检测画面已安装到Pi生产工作区，当前工作台仍是原进程，下一次正常重开才显示新卡片。没有自动重启会触发机械臂回位的桌面入口，没有发导航、底盘或夹爪/机械臂命令。** 点击“开始画面预览”会按需启动两个ROS2适配节点并连续逐帧识别；按钮不要求已加载地图，正式映射仍保留原识别区停车与三帧一致条件。

最终本地94项及16个subtest、Pi94项通过，补充覆盖无地图独立预览和取消发送异常不影响后续停车清理。已原子部署5个视觉/界面Python模块、包依赖声明、2个测试、隔离验证脚本及2份文档共12个生产目标（含install/package.xml），逐个保留原件；当前build包目录原已指向source，新模块无需重启或重建底盘服务即可导入。生产实际导入路径/哈希已核验，再次用生产安装模块在私有DDS88完成真实648×486/21762字节JPEG与JSON配对，detections仍为空。测试适配节点结束后退出，未遗留自动抓拍程序。56份现场地图/点位/历史逐字节不变，原GUI/AMCL/底盘驱动/唯一guard进程保留，main/dbac0a6及所有既有修改保留。

证据为本节目录中的`local_tests_final.txt`、`production_deployment.stdout`、`installed_verification.stdout`、`results.zip`及解压内容：包含`production_before.zip`、源/安装指纹、真实两次推理的JSON/JPEG、8张Pi原生Tk截图与回归输出。实物分类及左右标识交换验收仍未完成，不把模拟检测框截图当作实物结果。

最终宿主核验boot仍`1b3ea930-0025-4e24-b6ed-666b20b7661a`；两个原视觉容器ID不变、restart_count均0，9份相机/YOLO代码、模型及参数文件SHA与写前一致。两次真实测试仅按原接口更新临时抓拍照片，没有保存正式照片或改曝光/模型参数。

## 44. 2026-09-20 重开后用户反馈“一下就不行了”

第43节重开并未完成实体导航验收，用户随后反馈立即再次失败。本轮从最新进程、实际加载库/参数、导航返回和原始日志重新诊断，不沿用上一轮“Nav2未启动”的状态，不把模拟成功当作实车通过。保持2cm/10°/stateful=false、实际车体及唯一guard；先只读取证，不自动发送导航、非零底盘或机械臂命令。保留全部现场地图、用户点位和既有修改。证据目录`runtime_evidence/20260920_175503_navigation_failed_again/`。本节在新实施前写入，后续按真实结果追加。

现场重新核验为新版库`b7400907…`、2cm/10°/stateful=false，不能再把本次归因于“旧会话”。90帧雷达窗口平均10.003Hz、中位间隔约0.100s、最大间隔0.218s；controller配置10Hz、局部地图5Hz。日志直接失败原因是实际制动足迹冲突持续超过0.3s→Controller patience exceeded→恢复次数上限，不是已证明的扫描频率配置错误。用户确认附近有空隙、实体转向不会碰物体；该现场反馈已记录，但不能据此删除真实车体或将全部障碍自动当误报。静态快照并非冲突当时几何，尚不足以证明每次拦截均为假障碍。

另捕获到第43节重开后的start返回SUCCEEDED但停车后1.459cm/25.113°（accepted=false），以及用户后续自行试跑start_copy时SUCCEEDED但停车后3.925cm/2.902°。25秒只读运动记录表明返回时底盘/平滑器仍有转动速度，AMCL又继续修正位置；因此“提前成功后再超差”不能单归因于机械过冲。核对Humble SimpleGoalChecker确实不检查速度。曾以双坐标系时戳做另一假设的隔离实验，但未复现预期假成功，该假设未采纳，失败探针和原始回执保留；没有更改目标时戳或坐标。

候选实现（此段写入时尚在构建/隔离测试，未部署）：普通分阶段导航改用SettledGoalChecker，仍2cm/10°且不锁存位置；速度≤0.01m/s、角速度≤0.02rad/s并且两项误差连续满足0.5秒才完成同一个目标，期间超差直接恢复原目标修正。制动预测受阻而当前车体无碰撞时，先在同一Action中输出零并复查，等待上限为当次实际制动预算加0.3秒（0.5–3.5秒有界），不随重规划或降速滑动延期；当前车体碰撞/过期地图仍立即失败，持续无法恢复仍按原0.3秒错误容忍及两次恢复上限退出。没有关闭普通模式碰撞检查、改变guard或放宽精度，也没有增大雷达频率。本轮原固定RPP模式及其独立合同不变。

**第44节最终状态：修复已安装到Pi生产工作区，旧导航子服务已正常退出；原工作台、AMCL、底盘驱动、唯一guard保留。下次用户点击“开始导航”会由工作台原启动路径加载新版，无需重开工作台或再次执行机械臂休息动作。没有自动发送实体目标；这次补丁的实车复跑尚未完成。**

验证：本地相关50项回归通过，Pi原生C++策略测试通过。私有DDS87验证移动中不能提前成功、短暂零速不能成功、停稳窗口定位偏移后在同一Action内恢复位置修正（最终1.537cm）；短时制动冲突保持Action且仅发零、清除后继续，同一段取消归零；持续制动不安全有界ABORTED且全程零速（约2.01秒含结束后观察），车体碰撞立即失败。原5cm微调、0.55m直线、先转向再前进最终修角、前进/转向取消均通过；用户1rad/s转向场景停车误差3.366°。生产安装版（不加载候选overlay）再次在私有DDS实际加载新Goal Checker并通过“移动→定位修正→同目标到达”，停车1.500cm。上述全部为隔离虚拟位姿，不宣称实车连续性/精度已通过。

部署：精选6份源码/配置/测试及生产安装配置、插件描述和原生库原子更新，原生库SHA256 `4bb566e4c8b60351aff6ed7759aa3957ca3d42e4cc05ac448528e774678bb78e`。实际共8个去重写入目标，未在进程内热截断库。写前无活动目标，部署过程187条底盘速度全零，写前后56份现场地图/点位/历史逐字节不变，包含用户本轮自行修正的点位；当前main/dbac0a6和全部既有未提交修改保留。最终50条速度全零、唯一guard；当时工作台PID189781、AMCL189978保留，生产controller已退出，参数回读为null是预期，不能称已经启动新生产导航。

本节证据`runtime_evidence/20260920_175503_navigation_failed_again/`含最新原始日志、9秒运动链、25秒只读运动/AMCL/地图/速度记录、候选及部署回执；`results.zip`与`results/`含原生构建/策略/隔离及生产安装验证、`selected_before.zip`、`atomic_before.zip`、最终文件指纹和运行核验。未复现的目标时戳假设探针保留为调查记录（退出1，不是回归通过）；最终生产候选没有修改目标时戳。普通模式扫描/地图频率、限速输入、实际车体及碰撞检查保持，固定RPP模式合同、视觉容器和原机械臂姿态配置均未改。

参考核验：Context7额度已耗尽，改查Nav2官方Humble源码`nav2_controller/plugins/simple_goal_checker.cpp`与`src/controller_server.cpp`：前者不使用速度参数，后者对持续控制异常按failure_tolerance终止。现场证据与此路径一致；此前全部断续是否还含定位抖动、通信/处理延迟或瞬时障碍，仍需新版本实车数据，不记作已排除。

## 43. 2026-09-20 直线仍断续、start无法到达的继续诊断

用户反馈直线仍断断续续且不能到达start。本轮先重新核对实际boot、加载的新旧控制器/工作台、地图/点位、最新目标结果和运动链采样，不把第42节隔离通过当作现场问题已解决。保持2cm/10°/stateful=false，不改原地图/坐标，不凭猜测调高速度或放宽到点容差。继续采用原Nav2/bridge/唯一guard，先只读取证并在隔离环境复现，再部署有证据支持的最小修复；不主动发送实体导航或机械臂命令。原未提交修改保留。本节在实现前追加，证据目录`runtime_evidence/20260920_172155_straight_start_diagnosis/`。

重新读取发现仍是第42节之前的旧GUI/controller进程（旧库映射、现场20°）；用户已将转弯速度设为1.0rad/s。此次抓到运动中速度链，guard持续转发导航命令，未证实STM32故障；原controller仍有制动足迹冲突/无合法轨迹/控制循环超时。用户明确start是“到附近一直转向”，并回复“默认的固定姿态舵机10改成50，然后已经停车，可以重启”。该最新许可授权在停车后重启工作台及其原有休息姿态流程，本轮把启动/建图休息姿态的10号从500改为50，其余已保存关节不改；仍需原物理读回验收，不把该休息姿态当成运输或放置标定。用户已自行修改room地图的start及start_copy点，保留现场最新路线及全部历史，不恢复旧坐标。

**本轮最终状态：已按用户许可重启原桌面工作台，新GUI/2cm/10°逻辑已加载；启动休息姿态10号目标50、物理读回48，通过原读回校验。底盘未执行本任务发出的导航或非零速度。当前AMCL/Nav2未启动，不能声称生产controller已回读10°或实车断续/转圈已解决。** 下一步由现场选择当前地图并核对真实位置/朝向；若车辆不在记录的起点，须先纠正定位，不能把当前实体位置直接当成start。用户最新点位及历史保持不变。

本轮运动中只读取证：9秒窗口Nav2原始68条（62非零）、平滑器145条（144非零）、bridge到teleop134条（133非零）、guard153条（152非零），表明当时guard在持续转发；不据此断言四轮执行正常或异常。旧controller在平移与纯转向间切换，仍有旧版足迹冲突/恢复记录。用户所设转弯1.0rad/s保留。本轮没有再凭推测改控制器策略，而是使第42节已验证的修复退出“已安装但旧进程仍运行”状态。

重开过程中发现独立故障：原厂机械臂调试窗口`arm_pc/main.py`与ROS底盘驱动同时打开`/dev/rrc`（实际`/dev/ttyACM0`），该工具还会关闭ROS串口接收。正常关闭该窗口、恢复接收开关后仍无IMU/舵机回包；基础服务日志明确记录`recv_task`线程因SerialException退出。于是仅正常终止失去接收线程的驱动，由原launch既有respawn机制重建；没有更改原厂驱动、原厂工具、基础服务或guard代码。恢复后IMU和六关节物理读回正常，再由原桌面入口执行已授权休息动作。串口争用与该故障一致，但其发生时间不能解释此前全部导航断续，不把两个问题合并成一个已证实根因。导航时不要同时打开直接占用该串口的原厂机械臂调试窗口。

休息姿态仅10号500→50，其余目标保持1:454、2:816、3:221、4:52、5:866。最终读回454/818/216/53/866/48，原容差15内全部通过；仍是启动/建图休息姿态，不是运输或放置标定。Pi tools和source两份配置一致。最终4秒回读IMU150帧、scan40帧、底盘62条全部零，唯一发布者仍为cmd_vel_guard。56份地图/路线/历史与重启前精确备份逐字节一致。当前boot仍为`1b3ea930-0025-4e24-b6ed-666b20b7661a`；此次核验GUI PID188884、driver PID188710仅作当时证据，下次须重新发现。

补充验证：本地休息姿态10项测试通过。Pi生产安装版插件在私有DDS87、虚拟位姿/隔离速度出口中按用户转弯1.0rad/s测试：近start接近SUCCEEDED（1.5486cm/4.6657°）、0.55m直线SUCCEEDED（1.7694cm/7.4820°）、原地最终修角SUCCEEDED（0cm/4.9106°），均零速结束。这些不是实车行驶验收；真实直线连续性、start停车误差及提速仍待现场复跑。

证据：本节目录保留原始运动链、失败日志、串口持有者、接收线程异常、驱动恢复和重开全部回执（包括前两次读回失败，未删除失败记录）。`results.zip`及解压`results/`含`before_restart.zip`、生产安装版隔离测试与`final_verification.json`；`restarted_startup_log_final.stdout`含最终`PHYSICAL_READBACK_PASS`及新GUI实际模块路径/指纹。使用说明同步更新。第42节“旧会话尚未重启”由本节最终状态取代，其当时取证结论保留。

## 42. 2026-09-20 地图删除、速度说明、异常停车及10°到点标准

用户本轮要求加入删除地图，说明并改进远距/近距/微调/减速距离的含义和实际生效提示，排查截图的“已停止 / more than two Nav2 recoveries”。随后明确将朝向收紧到10°以内：本轮统一标准为停车后位置≤2cm、最小朝向误差≤10°且stateful=false，覆盖控制器/Goal Checker/执行验收和UI，不改原地图或点位坐标。地图删除采用用户在界面明确选择后移入可恢复回收站，当前任务仅增加功能，不代用户删除任何地图。实际运行问题先读最新日志与目标，不把上一点的通过误差当作失败点误差，不以去掉恢复上限或放宽容差掩盖问题。

本节在实现前写入。重新核对Pi仍为`<ROBOT_OR_HOST_IP>`、boot `1b3ea930-0025-4e24-b6ed-666b20b7661a`，用户当前运行工作台、AMCL与普通分阶段Nav2，加载新地图`20260920_080933_room`。不得使用旧PID/位置，不启动实体运动；先现场只读取证、备份和隔离验证，再精选部署。原视觉容器和guard链不改。修改前证据在`runtime_evidence/20260920_163855_map_speed_stop_fix/`，本地main/dbac0a6及已有未提交修改保留。

用户随后补充“行走总是断断续续”。现场controller日志确认多轮`approach or braking footprint blocked`→约0.3秒后`Controller patience exceeded`→清图重试，部分重试成功，部分超过两次恢复后由工作台停止；不是凭截图推测底盘故障。9秒只读动力链采样时车辆已停，底盘/teleop均连续零速、scan/odom持续更新，尚无运动中STM32/四轮执行证据。截图1.16cm/16.7°属于上一个停车记录，UI失败时错误沿用该大字指标，需明确目标名及上次记录。候选修复包含同目标方向切换缓冲，以及真实足迹和当前实测速制动轨迹通过后才尝试较低的安全指令速度；仍无法通过时保留停止，普通导航碰撞检查不关闭，实际车体不缩小。新实现尚待原生隔离检查与部署，不记作实车改善已验收。

**阶段完成状态：新版本已安装到Pi生产工作区；当前用户旧工作台/Nav2会话未重启，新版本尚未进入这次现场会话，实车行驶未验收。** 最后现场回读旧controller仍为20°、0.25m/s；不要把磁盘已更新写成当前进程已采用10°。用户正常关闭并重开场地工作台后，新的Nav2/控制器/界面使用2cm/10°/stateful=false。未自动重开原入口，因为其既有`prepare_mapping_arm()`会执行机械臂休息姿态，不能把软件更新测试变成未确认的实体动作。

实现内容：采点/纠偏页提供地图删除确认及回收恢复，保留点位/历史、共用图像与同名文件；没有代用户删除任何地图。速度栏改为巡航速度、接近目标速度、最后6cm微调速度、距目标多远开始减速和转弯角速度，单位/范围/输入待应用及真实回读均明确。顶端速度注明“手动移动”。截图的微调0.5m/s、减速距离0.1m不符合原允许范围，未生效；其现场生效值仍为0.25/0.12/0.03m/s、减速距离0.25m、转弯0.35rad/s。失败状态显示实际失败点名，上一点误差单独标注“上次停车记录”。

原生控制器保留普通模式碰撞检查、全部真实footprint和0额外余量。初始对正/大偏差时停车转向，已开始跟踪后的小偏差不再反复进入停车转向；同目标重规划保持该状态。制动检测先核验当前实测速的完整制动尾迹，再选择可通过同一足迹检测的原速或较低速度；当前车体/实际制动范围不安全时仍失败停车。冲突日志新增实际/请求制动、时刻、代价与位姿，便于后续区分真实几何冲突和其他原因；没有删除恢复上限、放宽到点容差或绕过guard。

验证：本地156项加16个subtest通过；Pi Humble原生280项通过，C++策略测试通过。私有DDS87中10°内原地成功且无转动、5cm微调、0.55m跟踪、先转向/前进/末端修角均成功，最大停车误差1.7989cm/8.5944°；前进和转向取消均CANCELED、速度归零。新增障碍隔离测试确认请求0.12m/s时可降至0.09m/s保持Active，不反复abort；障碍进入车体范围时ABORTED且全程零速。控制器/平滑器/guard的2.0m/s与2.0rad/s写入和回读仅在私有DDS进行，随后恢复默认。生产安装版七服务全Active后首个NavigateToPose被接受并SUCCEEDED，误差1.7336cm/0°。上述全部使用虚拟位姿和隔离速度出口，不是实体运动验收。

部署核验：白名单23份源码/配置/原生包文件与生产安装配置、原生库共28条目标路径更新；Python包重建成功，真实导入build→source，新增地图模块可导入。原生库以新inode原子替换，旧controller继续持有旧库映射，未在运行中截断库文件。新库SHA256 `b74009076d23eb6a1bb85d31a885f9f4c7989219612e93ab13ad22388a44189a`。写前118条、最终81条底盘输出均零；唯一guard发布者，未重启guard或更改视觉容器。当前field_tool_data共56份文件逐字节不变，包含用户新room地图；本地main/dbac0a6及所有既有未提交修改保留。Windows9张与Pi原生Tk5张Mock界面预览已检查，未调用实体GUI启动前的机械臂动作。

证据：本节目录内`production_before.zip`、原始导航/工作台日志、`chain_read.stdout`、`results.zip`与解压`results/`，后者含`atomic_targets_before.zip`、构建/测试结果、最终参数与实际模块路径/指纹、Pi预览。最终运行文件清单另存`final_runtime_manifest.json`，初始`candidate_overlay.zip`仅代表首次候选，最终GUI还明确了顶端手动速度标签。已更新使用说明`docs/FIELD_WORKSPACE_CANDIDATE.md`。后续先重开工作台，核验现场运行10°和新控制器；有当前监看/净空后再复跑，记录运动中Nav2→bridge→guard→STM32/四轮，验证停顿是否消除。真实高速、精度、四件/八件运输放置和视觉分类验收仍未完成。

## 41. 2026-09-20 用户确认连接，开始实机候选部署

用户已明确“当前已经连上小车，可以进行实机修改”，此前禁止连接/写Pi的限制解除。先重新核对SSH、boot、实际进程/模块路径及原有未提交修改，保留Pi精确备份；构建第37–40节工作台、导航和视觉候选，完成隔离验证后再精选部署。固定地图、点位坐标和用户现场文件不覆盖；相机/YOLO容器代码、模型、参数及配置继续冻结。保持底盘零速，不把部署许可当作实际运动的现场监看确认，不沿用旧PID或实体位置。软件上限2.0m/s、可调角速度0.05–2.0rad/s，默认0.25m/s/0.35rad/s，2cm/20°/stateful=false，基线与提速实车验收仍分开。

本节始写于部署前；以下按阶段追加真实结果，不把隔离验证当作实车验收。

**本节最终状态：已部署并生效，未进行实体行驶/夹爪/机械臂验收。** 已精选更新41份运行源码、配置、launch和原生包，在`/home/ubuntu/ros2_ws`构建成功；Python实际导入指向production build→source，配置source/install一致，原生包prefix为`/home/ubuntu/ros2_ws/install/raicom_nav2_workspace`。地图、点位与历史等51份field_tool_data文件逐字节不变，现场新增library地图也完整保留。原桌面入口保留；重新打开场地工作台即可加载新界面。默认导航0.25m/s、转弯0.35rad/s，软件可设2.0上限不等于已经完成实车提速。

阶段2原生验证：273项Python回归通过，C++策略测试通过。实测发现自定义RotateToGoal不再替原插件声明`FollowPath.xy_goal_tolerance`，导致GetParameters返回数量不符；已补控制器声明并改善节点名错误提示。修复后私有DDS域87中控制器/平滑器/guard实际写入及回读2.0m/s、2.0rad/s与0.05rad/s，再恢复默认。精确端点规划0.237m不被栅格取整；20°内原地直接成功且无转动；5cm微调、0.55m跟踪、先转向再前进最后修角度均SUCCEEDED，模拟停车最大位置误差1.83cm、最大角误差18.17°；前进/转向取消均CANCELED并零速。新增`deploy/workspace_native_isolated.py`保留复现。生产安装版另经`workspace_launch_isolated.py`验证七个服务全部Active，首个NavigateToPose被接受并SUCCEEDED，停车误差1.7232cm/0°；全部是隔离虚拟位姿，无硬件速度出口。

阶段3写入及运行核验：写前121条底盘速度全零。首次遇Pi launch目录root权限而中止，随后复核原件或本轮精确候选内容，使用sudo仅写白名单文件并保留原所有者，没有整体chown或覆盖其他包。生产两个包编译完成后仅切换原guard服务；旧ExecStop仍匹配历史`python -m`入口，故先停服务、精确终止现场找到的旧guard console进程，再启动，避免并行发布者；service文件/ExecStart未改。重启后回读合同`raicom_guard_speed/v1`和默认guard线/角0.50，79条速度全零、唯一guard。GUI经Pi原生Tk加载实际安装模块，Mock节点/禁用按钮的建图、采点、导航、高级窗截图通过，临时X授权已撤销；没有通过原桌面入口触发其既有建图休息姿态动作。

阶段4视觉并行：私有DDS域88运行新yolo_camera/yolo_infer与既有原厂`deptrum-ros-driver-aurora930 aurora930_launch.py`（仅相机，无夹取或底盘节点）。收到217帧深度、169帧RGB，640×400；18轮双端口GET共36次200，Windows访问8000/8080也200。ROS抓拍成功发布26689字节JPEG，ROS推理成功但detections为空。原视觉容器未改/未重启，测试相机及适配节点已结束。只确认此次短时并行成功，没有复现用户完整深度夹取流程下的超时，也没有完成T/G分类、左右互换或三帧实物映射验收。

证据集中在`runtime_evidence/20260920_155623_pi_deployment/`：`production_before.zip`、全部SSH回执、`deployment_results.zip`及解压`results/`（原生测试、构建、隔离/完整导航、视觉日志、文件指纹与5张Pi界面截图）。原生库SHA256为`046a5d64efd675e5b195eac4527918c40c6d8d67e2a33f673f354345630f131d`。本地main/HEAD仍dbac0a6，所有既有未提交修改保留。剩余现场事项：当前实体位置/监看/净空核验、真实2cm/20°基线与提速、T/G实物识别、同学PICK_READY/A/B实际接线、运输及四套放置姿态标定与4件（再8件）运输放置；没有把上述事项写成完成。

阶段1实查及候选构建：当前SSH `pi@<ROBOT_OR_HOST_IP>`，新boot `1b3ea930-0025-4e24-b6ed-666b20b7661a`。原base和唯一guard运行，未见GUI/Nav2/AMCL/SLAM或深度驱动；使用与生产相同ubuntu身份采到81条底盘速度，全零。root身份第一次探测没有发现生产话题，未据此误判已停车。生产source/install及全部field_tool_data精确备份在Pi `/home/ubuntu/raicom_candidates/20260920_155623/production_before.zip`，已下载到本地证据目录 `runtime_evidence/20260920_155623_pi_deployment/`。生产文件尚未替换。独立候选两个包在Humble构建成功，新增C++插件不再是未编译状态；273项原生Python回归通过，实际控制器加载/隔离执行链仍待下一阶段。当前8000及8080在Pi宿主和MentorPi中GET均200；深度驱动未运行，不能宣称并行故障修复。内核存在Aurora USB断连与一次error -71，现阶段只记录线索，不据此定根因。未重启或修改两个视觉容器。

## 40. 2026-09-20 软件线速度上限2.0m/s与可调转弯速度

本节先于本轮实现写入。用户明确要求把软件线速度上限从0.50改到2.0m/s，并能在工作台手动更改角速度。该决定优先于第39节“软件最高0.50且不改guard限制”的旧安排：需要联动更新控制器、平滑器、唯一guard的软件限速与回读，不能只放开输入框而让下游仍静默截断为0.50。保留默认导航远距0.25m/s、近距0.12、微调0.03；新增导航转弯速度默认0.35rad/s，候选可设0.05–2.0rad/s（约2.9–114.6°/s），应用时要求停车和Nav2 Active。手动驾驶默认值、倒退约束、加速度、停车/失联/超时保护及2cm/20°/stateful=false不放宽。

当前仍没有用户“已连接小车”的确认，只在本地实施和隔离测试，不SSH、不写Pi、不启动实体运动。软件允许2.0不等于底盘实测可达或实车提速验收通过，STM32参数不改；高速度下的完整制动检查范围也需同步审查。真仓库main/HEAD保持`dbac0a67c13acf51541a3e1ea99afc9f74e77ab0`，保留已有未提交修改。本轮修改前副本在`runtime_evidence/20260920_153318_speed_2m_angular/before/`。

本地实现已完成：`speed_limits.py`统一线/角软件上限2.0；工作台新增导航转弯速度框，旧路线没有turn字段时补默认0.35，不重写地图/正式坐标。停车“保存并应用速度”写控制器的直行/角速度、平滑器的线速度及±角速度，再写唯一guard，最后全量回读。guard运行上限取max(0.50,所设导航值)以保留W/S/Q/E原手动默认；低于0.50的导航速度由控制器和平滑器限制。原线加减速度0.5m/s²、角加减速度0.7rad/s²同时核验；倒退平滑器上限仍0.08m/s。旧guard缺少只读`speed_contract_version=raicom_guard_speed/v1`时预检失败，不会只改参数表却仍缓存0.50后宣称已提速。

guard新增参数校验回调：只有运动许可无效、输出/加速器为零时可调速；只校验，不在Humble提交参数之前改变运行缓存，命令处理读取已提交值。调速不清急停锁、不授予运动许可、不更改watchdog或加速度配置。控制器/平滑器/guard任一步拒绝或回读不一致均不发车；三种写入的取消迟到请求均须等旧回复收束再开新事务。已更新手动就绪检查，允许guard上限提高但原手动速度不被隐式提高。

原生分阶段插件增加turn_speed实时读取，关联DWB max_vel_theta；线速度允许2.0。普通模式轮廓检查扩展为0.20秒反应预算+完整制动轨迹，采样间隔限制真实轮廓角点位移不超过半个地图栅格。2m/s、0.5m/s²的制动距离约4m，当前3×3m局部地图/净空可能限制可用速度；可检查范围外拒绝通行，没有用提速来关检测、改地图或缩车体。第36节固定RPP独立模式不变。**C++仍未在Humble编译/加载，完整制动检查和实际速度效果未验收。**

本地回归 **194 passed、2 skipped、16 subtests passed**，包括2.0m/s与0.05/2.0rad/s边界、三节点写入/回读、旧guard版本拒绝、各阶段取消迟到、运动中拒绝调速、参数拒绝不产生运行副作用、真实guard回调的2.0限幅/原加速度/失联归零/急停保持，以及既有导航/视觉/联动回归。两项skip仍为本机无ROS原生模块；离线guard测试执行实际方法但stub消息/时间/发布，不是ROS运行或实车测试。Python语法检查通过；9张Windows Tk离线截图已生成，新的调速框无裁切，建图和采点页保留。证据在本节目录：`local_tests_final.txt`、`speed_settings_local.png`、`candidate_manifest.json`、`local_candidate_files.zip`；文件包仅供本地复核，不是部署器。

后续连接前不进行任何Pi操作。接车后需先验证新guard导入路径/合同标记/参数服务及原生插件编译，再核对Active/停车调参和取消隔离、实际制动净空、线速度/角速度及2cm/20°精度；软件上限不是硬件能力或实车接受回执。

## 39. 2026-09-20 工作台实际调速、ROS2视觉节点与识别映射

用户补充：实机速度要能在工作台手动改变；两个Docker视觉程序（用户称yolo_camera/yolo_infer，历史实际名pi_cam_env/yolo_env）希望通过ROS2节点接入；与深度相机驱动同时启动后8000/8080网页均无法打开或连接超时；识别区增加明确的“两个方块类别→对应园区”映射展示阶段。

本轮先本地实施，仍未获用户“已连接小车”的消息，禁止SSH/写Pi/启动实机。保留原相机及YOLO容器代码、模型、参数、配置，通过独立ROS2 HTTP适配节点接入，不重复打开摄像头。网页均不可访问的根因待运行时证据，不能把ROS2包装当作已修复端口；准备只读的容器/端口/网络/相机占用/资源诊断。默认沿用左侧标识=园区1、右侧=园区2，三次有效且一致才采用，界面显示T/G与原HTTP 1/2标签及所选目的地点。

调速采用停车后应用、控制器和平滑器写入并回读；默认0.25m/s，用户手动调高不等同于实车提速验收通过。最大不超过当前guard回读且不超过软件0.50m/s边界；不把输入1–2m/s静默显示为已经达到该速度，不改guard或STM32硬限。实际效果、两相机并行及20°精度仍待接车验证。

本地实现阶段已完成：新增`vision_http.py`、`vision_ros_nodes.py`、`vision_ros_client.py`及`vision_http.launch.py`，通过yolo_camera/yolo_infer两个ROS2外部节点调用原HTTP服务。请求/结果用String+JSON及task_id/request_id，HTTP在工作线程执行；有界读取、错误/空检测处理、取消/迟到隔离、节点唯一性及同一ROS环境内跨进程抓拍锁。相机JPEG采用实查路径`/api/temp_image`。两个节点仅做请求式抓拍/推理和只读GET健康探测，不重复打开普通相机，不修改Docker。HTTP服务不可达仍会报错；没有宣称已修复8000/8080。

识别区新增独立卡片和停车识别按钮，联动到识别点后复核2cm/20°、三次有效一致才采用T/G→园区A/B→所选点名，发布`/raicom/vision/park_mapping`；task/status保留本轮HTTP标签1/2到A/B映射。每任务重新识别，地图/识别点/目的地改变及取消撤销映射。独立识别不夹取、不导航，也不免除下一任务识别。原手动夹取测试仍T→A/G→B，已在界面与联动映射明确说明区别。同学回报的最终A/B直接作为园区，不进行第二次类别交换。

工作台“保存并应用速度”在停车和Active后写控制器及平滑器，并完整回读；服务未激活只保存并明确标为未应用。两节点不能原子提交，部分写入失败保持停车；取消时未返回的写请求保留到收束，下一次写入/发车需等待旧写入返回并重新全量设置及回读。原生DWB包装补充Humble速度恢复逻辑，避免离开慢段后旧限速未恢复；C++仍未在Humble编译或加载。

本地最终检查 **161 passed、2 skipped、16 subtests passed**；两项skip为本机无ROS的原生模块。验证范围包括双节点调速/guard不变/部分失败/取消迟到写入、唯一视觉节点、当前请求关联、取消后图像不发布、HTTP错误与数据边界、三帧一致/左右互换、四件及八件状态机、20°到点逻辑。全部为离线Mock，不是两相机并行、实物识别或实车验收。9张Windows原生Tk截图已生成，调速及映射卡片实际检查；建图、采点、底部停车保持可访问。第一次预览缺少本地PyYAML路径，补入现有测试依赖后成功，未安装/改动Pi环境。

证据：`runtime_evidence/20260920_150123_vision_nodes_speed_mapping/`内`before/`、`git_before.txt`、`local_tests_final.txt`、`candidate_manifest.json`、`local_candidate_files.zip`及9张截图。清单/ZIP仅作本地复核，不是安装器。新增只读诊断`deploy/camera_coexistence_audit.py`（未在Pi运行），用于连接后分别记录深度驱动启动前及故障态的宿主/容器HTTP、监听端口、网络/IPC关系、相机设备持有者及资源/USB/OOM线索；不启动/停止/修复任何现场服务。用法和消息示例见`docs/VISION_ROS2_INTEGRATION.md`。旧相机/YOLO审计副本SHA与记录一致，仅证明本地副本未修改，不能当作今天Pi运行态。

仍待：用户明确已连接后才重新核验Pi与ROS候选环境；实际诊断8000/8080超时，验证两相机同时可用；原生控制器构建/参数回读/取消隔离；T/G各10帧及左右交换；真实姿态标定、四件试点到八件、实际速度与2cm/20°误差验收。本轮未SSH、未写Pi、未更改原容器或现场深度驱动，保留全部既有未提交修改。

## 38. 2026-09-20 导航与夹取联动新要求（优先于第36/37节旧容差与流程）

**本轮最新修订：用户将 30° 改为 20°。当前统一验收为停车后位置 ≤2 cm、最小朝向误差 ≤20°，stateful=false；本节下文原 30° 表述属于被替代的需求记录。界面、Goal Checker、候选转向判定及联动验收同步修改，仅本地，未写入 Pi。**

用户最新要求：Nav2全部Active后才发首目标，解决首次拒绝；停车后位置≤0.02m、最小朝向误差≤30°同时满足，stateful=false。先面向目标/路径→正向前进→近目标减速→位置到达→最后修朝向，不默认倒车。未达到位置时继续有界低速修正，进入朝向范围后及时结束；DWB RotateToGoal与Goal Checker判定统一，不能只改显示容差。单次失败可在有实际几何和净空验证时退/挪5–10cm后重新接近同一目标，不换业务目标，不绕过guard。

UI增加远距、近距（20–25cm）及最后微调速度设置。用户期望远距约1–2m/s、近距0.5–1m/s；当前源码guard硬限仍0.50m/s、原导航/平滑器0.25m/s，尚无1–2m/s硬件/制动/运输验收，必须明确请求值与实际限制及回读，禁止只改UI数值声称提速。人工急停、失联/定位停车与速度限制继续保留。新的2cm/30°优先于旧3cm/15°，不将旧标准测试重新标为新标准通过。

用户确认真实交接语义：到达夹取点并停车验收通过后发送PICK_READY；同学完成深度夹取和单目园区识别，再返回A/B，本项目收到后才去对应园区。先在识别区建立本轮1/2目标园区映射。到园区记录访问/成功放置次数并顺序使用固定姿态1–4，随后返回夹取区。已确认试点为left_a、right_a，每点两次，共4件；后续启用left_b、right_b后四点各两次，共8件。具体话题未提供，先沿用既有ROS2 String方式并使话题可配置。简单PICK_READY/A/B没有请求编号，不能伪称能辨认跨轮迟到结果；等待阶段只消费一次，停止/超时/非等待阶段丢弃。

原建图、采点/纠偏和第37节新版UI继续完成并保留。第37节UI候选v6已有129项原生Mock回归及四页真实Tk预览，尚未部署到生产；旧导航程序未被本任务重启。当前再次SSH到最后已知192.168.149.1被连接关闭，无法核对新boot、硬件状态或现场位置；继续本地实现和可复核测试，不沿用9月15日运行态。本轮修改前证据 `runtime_evidence/20260920_025304_navigation_delivery/before/`。

用户明确部署限制：当前未连接小车；**仅本地实施/测试，必须等用户明确表示已连接后才允许写入Pi**。收到该限制后停止SSH探测，未部署第37或38节候选。

2026-09-20本地阶段回执（最终20°标准）：已完成工作台三页UI、速度请求与控制器/平滑器/guard回读门禁、七节点Active稳定等待及首拒绝一次同目标重试、接收前不授予运动lease、停车超差有界同目标再次接近、PICK_READY/A/B四件/可扩展八件状态机、同学ROS2示例、运输/四套放置动作采集入口、物理放置人工确认计数及只读动力链观察。当前原生候选包 `native/raicom_nav2_workspace` 包含前向分阶段DWB包装、无锁存RotateToGoal及精确端点规划包装；仍在原Nav2/bridge/guard链内，**C++候选尚未在Humble编译或加载**，不得称已解决实车问题。普通导航候选与第36节固定RPP模式分开；自动退/挪5–10cm未启用，仍需现场几何和净空验证。

本地最终检查 **125 passed、2 skipped、16 subtests passed**；两项skip为本机无ROS的原生消息/GUI测试模块。检查含4/8件顺序、三帧识别衔接、失败/重复/取消/超时、实物确认才计数、2cm/20°边界及速度三段配置；测试输入全部Mock，不是实物或视觉分类验收。Windows原生Tk已检查建图、采点、导航、配送、标定、高级窗和540×760布局；截图明确标“离线预览 / 未连接小车”。首次PrintWindow截取子窗口全黑，改为窗口根句柄并增加非空校验后重新捕获成功，未将黑图当作视觉通过。

证据在 `runtime_evidence/20260920_025304_navigation_delivery/`：`local_tests_final.txt`、`candidate_manifest.json`（43个候选文件指纹，注明未编译/未连接/未实车）、`local_candidate_files.zip`（本地代码文件集，不是安装器）、7张`*_local.png`/`navigation_20deg.png`预览。用法及完整边界见 `docs/FIELD_WORKSPACE_CANDIDATE.md`。本轮更新了固定RPP生成器/执行器、旧JSON配送和提速判定的当前容差；旧3cm/15°固定路线包不能直接冒充新标准，需后续保持同几何重新生成并验证，未改原地图/坐标或自动重生成任何正式路线。

剩余：用户明确连接后重新核验真实boot/路径/进程/实体状态；先在隔离候选环境原生构建和模拟控制/取消回归，再决定生产写入；现场采运输/放置标定，确认同学CANCEL和PICK_FAILED扩展及机械臂控制权交还；核验真实识别、导航误差、夹持和落点。裸A/B缺请求编号的跨轮迟到限制已在说明中显式记录，未伪称完全防重放。0.25m/s基线以上和实车效果均未验收。没有在本次断开期间写Pi、运行ROS示例或发送任何实体运动命令。

## 37. 2026-09-15 场地工具界面优化（本轮最新任务）

用户已在宿舍自行试跑导航并表示满意，这是用户体验反馈，不代替实验室五点/八件实物验收。本轮优先优化现有建图、采点/纠偏、导航UI，保持现有导航控制、地图坐标、到点标准及guard链。普通路线编排嵌入导航页空白区，原入口改为高级路线编排；增加延时开始、点位复制、到点停留、当前机械臂姿态物理读回记录（默认保持当前姿态），以及导航误差、任务进度和夹持状态。移除导航页两点往返测试和重复立即停车按钮，保留下方手动区Space/立即停车及实际取消保护。

T/G操作支持A/B分别从当前地图正式点名选择，T→A、G→B；用户明确夹取动作是舵机10到300，其他关节不得使用猜测脉冲。闭合物理读回和实物夹稳分别记录。高级设置绑定当前地图/路线，不改变原点坐标。当前尚未实现/部署本轮UI；先备份、修改和隔离测试，不为UI验证启动实体导航或机械臂。需重新读取当前Pi状态，不沿用上一轮PID/定位。修改前证据：`runtime_evidence/20260915_032008_field_ui/before/`。此前第36节固定RPP候选仍属独立模式，本轮不擅自切换用户满意的现有导航控制器。

本轮用户补充已确认：夹取后由用户点击“确认夹稳并出发”，不按舵机到位直接发车；建图与采点功能必须完整保留，均已纳入三个页签。修改前后Pi原GUI SHA与本地备份一致；当前仍在宿舍，最新读取boot保持不变。首轮只读81条底盘速度全零，唯一cmd_vel_guard；彼时AMCL/Nav2/旧GUI在运行，后续只读发现这些现场进程已经结束，本任务未关闭它们。预览为Mock节点，未建立任何ROS执行器接口。

实现阶段：新增工作台UI、当前地图A/B绑定、可取消倒计时、到点停留、姿态物理记录/显式启用、T/G闭合→人工确认→再次读回→原导航；保持原field_tool_ros/model/navigation_logic及Nav2配置不改。六关节读姿态默认不创建执行器发布者；夹取仅10号300。记录与动作的物理响应均有超时，界面失联/停车请求/底盘非零会取消动作。54项本地逻辑检查通过；原生Humble消息和原GUI回归129项通过。测试均Mock I/O，没有实际舵机/导航运动。

视觉检查发现短窗口夹取区遮挡、早期自适应滚动区首次打开空白，均在隔离预览中发现并修正；最终采用稳定滚动容器，三个页签及高级窗都有实际Tk截图，普通与540×760尺寸均可访问控件，下方停车始终可见。早期预览失败回执原样保留（未补齐mock mouse_widget、现场X授权已撤销等）；独立预览临时使用已知本地用户X访问，结束撤销，未使用全局xhost放行。

## 36. 2026-09-15 续接执行决定（优先于下方全部历史安排）

本节先于实现写入。真实仓库 `<LOCAL_WORKSPACE>\raicom_control`，main，HEAD `dbac0a67c13acf51541a3e1ea99afc9f74e77ab0`，所有既有未提交修改保留。修改前CURRENT、Git状态及diff在 `runtime_evidence/20260915_015121_fixed_route_resume/`。本轮重新SSH `pi@<ROBOT_OR_HOST_IP>`成功，boot仍为 `ecdad943-b012-4b1e-9be2-8a384015b8dc`；三个容器及基础服务/guard运行。实体位置、定位和监看仍待本轮核验，旧PID/位姿不沿用。

- 用户已授权恢复第35节完整备份中的缺失正式点；保留当前路线精确副本、history及有效pick_A，不再询问删除意图。沿用lab1原地图、原坐标，顺序 `start → shibie → pick → pick_A → pick_B`；pick_A=园区1，pick_B=园区2。
- 原AMCL、Nav2、bridge、唯一guard链内新增RPP/FollowPath固定分段模式，旧DWB模式保留回退。端点、出站点、转弯点和路径绑定地图/正式点指纹；pick按现场几何倒退出站→停车→转向→前进，不能以按键时间估距。
- 用户明确关闭新固定模式前进、倒退、转向执行链的导航实时碰撞检查及自动绕行，保留制作阶段静态几何检查。必须参数回读和隔离测试确认各段关闭范围及取消行为；不通过绕过guard实现，不关闭人工急停、失联/定位失效停车、超时取消及速度限制。
- 到点停车后位置≤0.03m、最小朝向误差≤15°同时满足，goal checker stateful=false。实际车体不缩小，额外padding=0cm。导航生命周期active后才发目标，记录真实返回及停车后误差。不得重复第24—35节失败方法或称固定路径本身已解决末端精度。
- 基线0.25m/s；倒退≤0.08m/s，转弯/接近减速。基线完成才验收0.35m/s；控制器和平滑器同时核对。同路线同运输状态连续两轮满足原精度且耗时降低才保留提速，否则回退0.25m/s。
- 识别区停车后串行调用现有 `POST http://127.0.0.1:8080/api/do_infer`（无body，内部8000抓拍）。HTTP `1→T, 2→G`，blue不作配送类别。左标识=园区1、右标识=园区2，连续三次有效一致才采用本轮映射。pi_cam_env/yolo_env代码、模型、参数及容器配置禁止修改；空detections只证明接口连通。
- pick内部识别/对齐/夹取由同学负责，本项目预留ROS2 String+JSON：`/raicom/pick/request`含任务/请求/轮次及start/cancel；`/raicom/pick/result`含对应编号、成功/失败、T/G、夹持状态、机械臂控制权交还；`/raicom/task/status`含阶段、目标园区、完成数、等待/失败原因。只接纳当前未取消请求的有效成功；重复/迟到/未知类别不发车。提供示例节点。
- 运输夹爪保持夹紧，可抬起/收回到运输姿态。现场采一套运输姿态和四套接近→落位→松爪→撤回动作；两园区复用但分别验证落点，沿用原舵机物理读回。目前无有效标定值，禁止用休息姿态或猜脉冲。
- 实物流程 `start→shibie→pick→对应园区→放置→pick`，8件、4T+4G、打乱顺序，两园区各位置1–4；pick人工装真实方块，测试适配器回报本轮类别。仅确认实物放置成功计数，第8次完成停最后园区。
- 验收依次为五点连续/往返；T/G各10帧、各≥9有效且有效全正确并左右互换；ROS2正常/失败/重复/超时/取消；8次真实运输放置（误差/夹持/舵机读回/落点/耗时）；最后提速。

执行起点：五点尚未恢复，固定控制器尚未部署，实车碰撞检查尚未关闭，八次测试尚未启动。先做资产恢复、实现与隔离验证；实际运动前核验当前实体位置与现场监看。以上已确认选择不再作为阻断问题重复询问。阶段完成后在本节追加事实及证据。

阶段1完成：本轮已从指定备份恢复缺失start/shibie/pick/pick_B，正式顺序为start/shibie/pick/pick_A/pick_B。现有pick_A内容及全部旧history、drafts、start_localization保留，地图两SHA不变；Pi写前副本 `/home/ubuntu/raicom_tools/fixed_route_restore_20260915_015121/`。恢复后route SHA `819dd0d845e991097943d3554ff94771806dbd8716d8483745a158a2568bd7bf`。证据 `restore_five_points.json`、`lab1_restored.route.json`。只读观察99条底盘速度全零、唯一发布者cmd_vel_guard，生产无AMCL/Nav2/GUI；原生Nav2/RPP版本1.1.15。首次资产读取遇隐藏目录而失败，`live_inventory.json.stderr`保留；修正为只取文件后的真实结果是`live_inventory_v2.json`。

用户本轮确认：**小车当前未在实验室**。因此本轮不启动lab1实车导航、不注入实验室初值、不做机械臂动作；继续实现、资产静态检查及私有DDS隔离验证。实车五点/往返、T/G实物20帧与左右互换、运输/放置标定、8件实物及提速验收均待回实验室。Screen Live当前live=false，未取得本轮现场画面。

阶段2原生隔离验证：`native_actions_v3.ndjson` 的9场景PASS。ROS_DOMAIN_ID=87、ROS_LOCALHOST_ONLY=1，原生RPP→平滑器→原bridge候选→原guard，末端仅`/isolated/base_cmd`且没有硬件节点。全黑静态测试网格上，三种动作开启检查均ABORTED(6)，关闭后急停均CANCELED(5)并零速；三个理想位姿积分到点均SUCCEEDED(4)，最终前进2.5306cm/0°、倒退2.6809cm/0°、转向0cm/11.8134°。控制器前进.25/倒退.08、平滑器前进.25/倒退.08、goal checker .03/15°/stateful=false均实际回读。只证明隔离控制/取消行为，未验证lab1全路径或实车。

版本陷阱：原生1.1.15的RPP动态参数回调未处理`use_collision_detection`，设置该参数ACK成功也不会改变内部检测开关。v2因此负对照超时，失败证据保留；v3采用分别configure加载的开/关实例验证。正式固定模式必须启动加载false，不能使用动态ACK伪称已关闭。测试用CheckedForward/CheckedReverse仅负对照，正式配置只有FixedForward/FixedReverse。v1为PYTHONPATH覆盖ROS路径导致导入失败，未启动隔离节点；已修正。当前生产导航未启动，不能称实车实时碰撞检查已经关闭。

阶段3实现与原生回归：新增`fixed_route_assets/builder/profile/runner`、`recognition_station/acceptance`、`delivery_protocol/arm/runner`、`pick_example`、`operator_presence`、`speed_acceptance`。沿用原FieldToolNode与NavigationController的TF/scan、lease、取消及停车后验收；旧队列仅增加可区分FollowPath的错误名称。固定入口只加载两RPP和原bridge，实际速度来源逐级核验；Windows `deploy/fixed_route_presence.py`保留可见Space急停、失联撤销与不自动恢复。`deploy/capture_delivery_pose.py`只采原舵机物理读回，无动作。

最终完整固定配置的costmap空插件数组在Humble初始化时报`No parameter value set`，`native_actions_profile.ndjson.stderr`保留。修正为仅加载**禁用的inflation_layer**，不订阅实时障碍；`native_actions_profile_v2.ndjson`确认controller/smoother激活、两RPP碰撞/成本减速均false、限速与原足迹/0padding回读通过。原生回归最终`native_regression_v4.txt`为127项通过；真实DDS协议六场景`pick_dds_v4.ndjson`均PASS。离线八件乱序4T+4G逻辑、重复放置不计数、实体确认门禁、未标定拒绝、左右互换三帧共识及提速判定均有回归；这些不是实物识别/夹取/放置验收。本地`local_final_regression_v2.txt`为96项+79 subtests通过；v1因Windows无原生ROS launch模块收集失败，不是实车或业务测试失败。

阶段4候选落盘：最终`candidate_deployment_v4.ndjson`核验所有选定source/install SHA，原DWB两份配置不变。Pi备份及manifest在`/home/ubuntu/raicom_tools/fixed_route_candidate_20260915_015121_v4/`，未启动生产定位/导航、未重启任何服务或容器。已有三个文件只作本任务必要小改，其他旧未提交内容保留。`installed_module_imports.json`确认build安装路径实际解析到source的新模块。任务退出时继续保持ROS通信，先取消目标/停止机械臂插值再清理；不卸载扭矩、不猜测脉冲。新增模板`fixed_lab1_survey.template.json`绑定原图/正式点；pick_exit和turning_points为null，`delivery_calibration.template.json`全部未标定。

部署过程失败记录：v1未通过静止门禁、v2看到102条零速后首个目标临时文件写入被目录权限拒绝、v3用root做ROS发现得到0条速度；均未替换生产目标文件。v4改为ubuntu完成ROS只读门禁（108零样本），root只负责既定文件写入后成功；不修改目录权限/所有权，不绕过静止要求。临时before副本保留，不覆盖失败回执。

**当前停止点与下一步：** [完整使用/接口说明](FIXED_ROUTE_DELIVERY.md)。正式五点已恢复；固定控制代码已部署但未在生产激活；lab1可执行路径bundle仍未生成，因为现场pick出站/转弯几何未知。当前实体不在实验室，现场运输姿态/四套放置动作及两个园区落点也未采集。回实验室后先核实实体/定位与监看，采几何并静态编译，再验固定五点/往返，接着T/G各10帧和左右互换，再动作标定和8件真实运输。所有已确认业务选择保持，不再询问是否恢复五点/是否关闭新模式碰撞检测。0.25基线未通过前不切0.35；现有提速函数仅判定保留/回退，0.35现场试验入口/切换留到基线完成后实施。

最后生产读回`final_production_zero.json`：boot仍ecdad943-b012-4b1e-9be2-8a384015b8dc，72条底盘速度全零、唯一cmd_vel_guard，生产无AMCL/地图/Nav2/bridge/场地工具；本图地图与恢复路线三SHA保持。`final_vision_invariance.json`实查视觉代码/模型/参数、三个容器ID/镜像/启动时间/重启计数与之前快照一致；本任务未调用真实YOLO，不新增实物分类声称。Git原567条dirty路径未丢失，main/dbac0a6不变，无提交或清理。

退出处理的最终两文件修订已另行部署并回读：`shutdown_revision_deployment.json`，修改前副本在候选目录`revisions/shutdown_fix/`；写前98条零速且无活动导航会话。fixed_route_runner最终SHA `f6b35bbe4f9d8e1be672fb54013ec556b733118e3dab48e249d3c234d82177e8`，delivery_runner最终SHA `b9f1877cb413118f6fea2f71c5e4930dcb502bc74b54b4f59d23276b61fee12c`。退出信号只置停止请求，主循环先撤许可/取消目标/停止机械臂插值，保留ROS通信等待回执后再退出。完整127项原生回归针对该修订已通过；未启动生产任务。

恢复后的五个正式点以原PGM/16顶点足迹/0padding再次静态核验，5/5标称姿态无碰撞（`restored_points_static_check.json`）。这只覆盖五个保存姿态，不覆盖尚未采集的出站/转弯路线及实车运输姿态。

更新：2026-09-15，已从当前树莓派核实相机与 YOLO 的真实 API，见第35节及 [接口说明](VISION_CONTAINER_API.md)。**当前 SSH 为 pi@<ROBOT_OR_HOST_IP>，boot ecdad943-b012-4b1e-9be2-8a384015b8dc**；旧172.20.10.3已不可用。本轮三个原容器运行，robot_state_publisher进程存在；未完成新boot定位/导航验收。8000相机页面及8080 YOLO页面均200，一次POST /api/do_infer业务成功但detections为空，不能称T/G识别通过。接口标签为"1"/"2"/"blue"，并非直接T/G；两个视觉容器的代码、模型和参数未改。

当前任务是红框比赛区域的固定分段导航，随后接入“蓝色RGBD对齐→T/G识别→抓取→随机目标八件配送”。用户要求额外车体余量0cm已写入Pi source/install，本轮再次核对两份配置SHA一致；3cm/15°同时验收不变。**另发现当前lab1正式路线只剩pick_A**，history记录删除start/shibie/pick/pick_B；五点完整备份仍在，地图PGM/YAML未变。本轮只读取/留存，不擅自恢复路线。此前五点和直接start→pick的DWB软件验证没有全部通过，固定路径与视觉抓取闭环也尚未实施，不得记为实车通过。

上一批 hall1 111↔222已完成4轮完整往返，8段均Nav2 SUCCEEDED且停车后同时满足当时的≤5cm/≤30°。平均位置误差 **2.9363004809cm**，平均最小朝向误差 **22.6782379687°**；最大位置4.6159575556cm，最大朝向26.2276375771°。第9段Nav2成功但停车后5.0713576592cm/23.8754153534°，面板按原阈值判failed并停止；第10段未发，第9段按用户指定不纳入本统计。不能声称5轮完成，也不能把这些旧样本算作新3cm/15°标准通过。上一批独立检查133条底盘速度全零；地图、点位、普通路线指纹均保持。误差为map定位估计，不是外部尺测物理误差。

结果：[前8段统计报告](../runtime_evidence/20260913_dorm_navigation/hall1_first_eight_report.md)、[统计明细JSON](../runtime_evidence/20260913_dorm_navigation/hall1_first_eight_summary.json)、[完整任务原始回执](../runtime_evidence/20260913_dorm_navigation/hall1_repeat_final_receipts.json)。mission `field-repeat-20260913T125848-11288062183491`。当前生产GUI/logic已部署往返入口，102项原生回归通过；代码SHA见第4节，过程与失败回执见第20—21节。RF2O实时检查未开启。Windows与Pi时钟约差30分钟，跨机时间不直接混用。

## 1. 当前目标、主任务、阻断、下一步

- 项目目标：在用户红框的比赛区域内复用lab1与已有正式点，完成导航、识别、对齐抓取和八件随机分类配送；手动区不纳入。
- 当前唯一主任务：先完成既有点之间的固定分段导航（pick出站倒退→停车→转弯→前进），再接入视觉与机械臂步骤。本轮插入项已完成现有容器API实查，尚未改控制代码。
- 已完成：五点历史正式采样及完整备份、原3cm/15°验收逻辑及既有隔离回归保留；0cm额外padding已落盘且新boot读取确认。API合同和真实返回已保存；当前路线实际仅含pick_A，不能再说当前已配置完整五点。
- 当前阻断：正式路线被删到一个点，恢复意图待确认；旧DWB末端停滞/超差未闭合；新boot尚未建立定位与实车导航验收。pick视觉阶段还缺当前RGBD配准、与普通相机同一目标关联及抓取姿态标定，本次空检测未验证T/G实物语义。
- 下一步：以第35节真实API完成控制层标签/目标关联设计；核对当前路线删除意图，再基于已有完整备份继续固定分段导航。不得把当前单点路线当成五点任务，也不从照片猜新坐标或重扫原地图。
- 保持：位置≤.03m、最小角度≤15°同时满足，goal checker stateful=false，DWB xy=.03；实际车体不缩小，额外padding按用户要求为0cm。原guard及低层控制不改。新boot实体位置/监控需新鲜核验，旧boot回到start的确认不代表当前已就位。
- 已排除：旧hall仅vx21仍后退；GoalDist64单独及PathDist64在完整复现触及254格，增加碰撞采样密度亦未解决；未部署。仅min_vel_x=0仍只是未完成验证的候选，不能直接应用。

## 2. 连接与环境（本节旧boot条目均为历史，当前以第35节为准）

- 2026-09-15最新：Windows WLAN192.168.149.201，实际SSH pi@<ROBOT_OR_HOST_IP>，boot ecdad943-b012-4b1e-9be2-8a384015b8dc；三个容器启动约46分钟。项目私钥在真仓库`.ssh/raicom_pi`，父目录没有该文件。`tmp/lab_text_remote.py`仍硬编码旧172.20.10.3，不可直接按其默认连接；本轮只读脚本显式使用.149.1。Pi返回2026-09-14 23:28 +08:00，Windows日期2026-09-15，时间差不擅自同步。

- 换电前最后已核验：SSH pi@<ROBOT_OR_HOST_IP>，boot 025afa9a-b679-41bf-9695-3297f758d410；本次SSH超时，重连后boot/PID全部重读。当前地图lab1。证据根runtime_evidence/20260914_lab_text_navigation，换电检查点lab1_battery_change_checkpoint/checkpoint.json。

- 最新：Pi热点已切回手机热点，SSH pi@<ROBOT_OR_HOST_IP>，Windows WLAN172.20.10.4；MAC2c:cf:67:70:55:6c相符。用户中途再重启，当前boot为97b5cb34-0df3-41af-be6e-a712ef42066f。新GUI9530/ticks15502、keyboard9941/ticks16322、RViz11862/ticks19796、guard1628/ticks3588，仅作本轮观测索引，操作前须再核验。原生104项测试在ROS_DOMAIN_ID87/ROS_LOCALHOST_ONLY=1内存加载候选，不部署或发运动。
- 本轮工具 `tmp/lab_text_remote.py` 固定当前已验证172.20.10.3，证据根 `runtime_evidence/20260914_lab_text_navigation/`；新回执拒绝覆盖。Screen Live已返回实时VNC画面，实时图像不存盘；一次直接grim重启后快照超时，reboot_desktop_v1.png未生成，不当作成功证据。不要改用dorm_remote旧IP。

- 当前SSH **pi@<ROBOT_OR_HOST_IP>**，Windows WLAN192.168.149.201/24连接小车热点HW-6770556A（profile HW-6770556A 2），Pi MAC2c:cf:67:70:55:6c。新boot **2675f6f6-33ae-457f-ab2b-2f809319445c**。此前手机热点172.20.10.3、a269 boot及其PID全部为历史，不能沿用。私钥 `.ssh/raicom_pi` 仅认证，不读取/提交；密码不入仓库/证据。
- 新boot start_app_node、raicom_cmd_vel_guard、wifi active/NRestarts0，MentorPi/pi_cam_env/yolo_env运行；/dev/rrc→/dev/ttyACM0。未重启底盘/guard或重建容器。
- 原入口为Pi桌面场地工具、`/home/pi/raicom_field_tool.sh`，共同锁 `/tmp/raicom_operator_session.lock` 不因空内容而删除。容器先source `/home/ubuntu/ros2_ws/install/setup.bash` 再set -u。build Python链接到src，install配置为独立普通文件。
- 最新只读进程（hall1_repeat_pose_jump_audit.json）：GUI503903/ticks940369、keyboard504311/940789、RViz543255/1007598、map_server543285与AMCL543287/1007712，地图为hall1。操作进程前仍须新鲜核对boot/PID/ticks；此前393595/SLAM等均已过期。
- 当前证据 `runtime_evidence/20260913_dorm_navigation/`。快照 `python tmp/dorm_remote.py screen <新文件名.png>`；只读容器脚本 `python tmp/dorm_remote.py run <脚本> <新回执>`，主机脚本加 `--host`。该wrapper已改新IP，拒绝覆盖。旧field_nav_remote/resample_remote默认旧IP，不直接重用。
- lab_preserved_manifest.json与lab_preserved/保存三份原实验室资产，before/docs/保存本阶段修改前文档。hall_observed_1/为18:28精确快照（含选用install配置），正式点可能随后由用户操作更新，需重新回取。
- 新boot7秒只读89条底盘速度全零、69条scan、最大scan年龄.113944037s，578个有效odom TF、最大年龄.055659815s；仅证明该观察窗口基础链路。电池唯一原发布者7623/7579/7565mV，中位7.579V，静止不低压。
- 初始桌面终端FIELD_TOOL_LOADED四模块全SHA与D一致，归位门禁PHYSICAL_READBACK_PASS、command_sent=false；未另开第二面板。最新快照dorm_state_before_edit.png显示hall已定位、模型出现、手动开启（用户操作），不以初始手动关闭状态冒充当前。
- 以下五点可行性和pick_a重采网络记录为**实验室历史/延期内容**，不能作为当前宿舍坐标或运行状态。
- 最新五点可行性证据 `runtime_evidence/20260913_five_point_feasibility/`，详见该目录README、静态明细和five_point_static_feasibility.png。本轮初查快照/7秒ROS观察仍位于上一行目录，文件five_points_feasibility_start.png、five_points_live_ros_before.json。
- 最新pick_a重采准备证据 `runtime_evidence/20260913_pick_a_resampling/`：精确before/manifest、原生52项测试、GUI关闭与部署、manual_load_observed.json。屏幕仍由field_nav_remote写到前阶段目录，文件resample_gui_loaded.png、resample_manual_map_ready.png。只打开地图，没有替用户画箭头/移动/采点。
- 18:11网络停止点见该目录network_blocked.json；amcl_capability_readback.json实际上为ssh_dispatch_run_fatal错误文本，不是参数回读成功。before_pick_a_seed_live.png未生成，截图进程返回255。一次netsh连接已尝试失败，不重复无间隔重连。未读Wi-Fi密码/私钥，未新建网络profile。
- `tmp/fixed_start_deploy.py snapshot` 和apply均已执行成功，禁止重复。apply仅为磁盘回执；另由 `fixed_start_reopened.png` 的FIELD_TOOL_LOADED四模块全SHA及 `fixed_start_runtime_readback.json` 共同确认新进程加载。

## 3. 实验室阶段事实与验证（历史）

1. 先完成旧 field 的精确PGM/YAML/route备份和独立地图解析，`field_assets_verified_backup.json`；此前关机导致的备份未完成项已关闭。未改变旧room/field点位。
2. 修复 refresh_map_list 总选index0导致显示room而实际field的问题，保留当前map_file；42项原生测试通过。部署GUI6909cc3d…并打开原面板。
3. 实际打开下拉框发现Tk popdown事件widget为Tcl路径字符串，触发AttributeError。修复编辑控件判别及失焦捕获，原生44/44通过；精确关闭空闲GUI后精选部署710c591e…，快照 `panel_after_focus_fix.png` 显示实际加载。
4. 首次本轮打开原入口确实通过原机械臂归位门禁发过归位命令（command_sent=true），PHYSICAL_READBACK_PASS；后续重开command_sent=false。不能笼统写成本轮无机械臂动作。助手未发非零底盘或导航目标。
5. 用户在固定start功能仍为D候选期间自行启动SLAM、手动扫描并记录五个草稿；`new_full_mapping_ros.json` 141条速度中135非零来自用户手动。16:58快照电压6.949V；GUI日志仍有超时停车，不能称卡顿完全根治。
6. 用户17:00:01首次保存失败，17:00:07重试、17:00:12成功。助手随后尝试原保存按钮时widget尺寸断言失败，尚未点击；`full_map_save_receipt.json`是失败回执，实际保存来自用户。SLAM退出，固定地图定位启动等待初始位姿。
7. 本次lab_map地图及路线、Pi修改前三模块六份精确副本在 `full_map_verified_backup.json`、`full_map_artifacts/`、`pi_before_fixed_start/`。新PGM278×214独立解析通过。
8. 固定start候选原生ROS/GUI **49/49**（`native_candidate_fixed_start_v3.txt`），原生model **13/13**（`native_candidate_fixed_start_model_v1.txt`），D导航逻辑 **16/16**。第一版两个错误为旧mock缺新增状态字段，修正fixture后通过；C盘Python缺PyYAML，未安装，改用Pi隔离测试。隔离验证不是实车初始化/导航通过。
9. 校验boot/PID/start ticks、无活动SLAM/保存器/导航控制器且新route未变后，向GUI76436发SIGINT，原清理正常退出；三模块编译与旧SHA预检后原子替换，回执 `fixed_start_disk_apply_receipt.json`。本图route仅加start_localization，其他全部JSON内容相等、九个既有地图/路线资产SHA未变。提示写入后的精确float比较因yaw规范化约4.44e-16rad差异失败；未盲目重写，独立只读回取验证成功，见 `fixed_start_route_hint_verified.json`。`fixed_start_route_hint_receipt.json`为该写后断言失败文本，不是成功回执。
10. 原入口重开后PHYSICAL_READBACK_PASS、command_sent=false，归位阶段62条速度/26条电机零样本；新GUI四模块SHA在终端实际可见。随后7秒只读观察82条底盘速度全零、53条scan、最大scan年龄.387382289s、odom TF最大年龄.324392981s；唯一速度发布者cmd_vel_guard。未发布初始位姿、未加载地图或发送目标。原面板已切到采点页并选中20260913_090007_lab_map；仅选择未点击加载，快照fixed_start_map_selected.png。下拉选择经原X键盘焦点/抓取完成，不重复向父Combobox发Return的失败方法；新面板无该次Tk字符串崩溃。

## 4. 上一批hall1部署与容差历史（最新实际3cm/15°见第24节）

往返功能已经由原入口实际加载，见hall1_repeat_selection_v4.png与repeat_test_disk_deployment.json。归位检查PHYSICAL_READBACK_PASS、command_sent=false。本次从原入口启动Nav2并真实发送9个目标，当前任务已FAILED并停止。hall1_repeat_active_readback.json实际9服务active、原容差回读成功，前8段均停车验收通过。当前是否仍active须后续实时读取。

| 文件 | 当前部署SHA256 |
| --- | --- |
| field_tool_gui.py | `3114fb3e6618a52be5ba3a8dae54a58c6452702415aa1ea5fbf408b7bfa8b960` |
| field_tool_ros.py | `5901c377e12d1e8ed638cb6db1a2d4a3b2728b178629e28dd8313362145d63b0` |
| field_tool_model.py | `b58e3eccd425553ff68c9fa2f5a03f283a9a9561c5e40367b64a8004aaf85410` |
| field_tool_navigation_logic.py | `1b601215cb9d2df63a00abe208d0b502e26e0305917f1796c1d57ba9f93af5a0` |
| nav2_controller_straight.yaml（source/install） | `d06cd433becf152db0ac7d2dcc5d7af7d1e316c3ddcca6da5e255ae06935bcd3` |

- 手动开机等待原TF/激光/控制源/速度参数及零速保持，再自动进入就绪；不发方向键。只允许原keyboard_input单独或加原nav_cmd_bridge，后者必须新鲜inactive；未知、重复或命名空间不符的发布源继续阻止开启。空格/立即停车/点击关闭及持键超时后锁定，必须显式恢复；空闲GUI抖动不再反复锁定，旧持键不会自动续走。
- 96项原生回归通过，见manual_autoready_native_regression_v2.txt；首轮旧fixture缺manual_armed的错误记录保留，不冒充通过。manual_two_owners_ready_verified.json实测88条速度零、100条导航active=false，原双teleop_cmd发布端存在；manual_final_ready.png显示就绪。
- goal checker和DWB的xy_goal_tolerance均.05；yaw_goal_tolerance .5235987756，general_goal_checker.stateful=false。DWB未声明FollowPath.stateful，不设置它。生产已为vx21/vy1/vtheta7，PathDist128/GoalDist64；其余控制参数未改。
- 此已部署版本GUI统一使用POSITION_TOLERANCE_M=.05，model新点默认.05；旧route的历史2cm字段不参与目标构造。旧hall路线SHA ba8371786cfbf1df511721198398ed72190507a8a861c187eb84a32b8502c922属于历史；上一批当前hall1冻结路线SHA为5a43786a41c40d6bcc0cd4b406df99b4d0c264797e3a571b3233a0d05d4f41fb。
- 每目标前7生命周期新鲜active；4项参数设置全部ACK后才发NavigateToPose。返回4后撤许可、等零速保持，再采新鲜map TF同时比较位置和最小朝向误差；曾进入范围不豁免后续偏移。
- trans_stopped_velocity=.02是停车速度阈值，地图resolution=.02、DWB linear_granularity=.02也不属于到点距离，本次均不改。
- 88项原生回归通过，包括4.9cm/29.9°停车后接受、5.1cm漂移拒绝、角度超30°拒绝、未停车/服务未active不推进。
- 固定start初始化和非start加载保持。此前用户回到旧hall start、约(.006,-.009,-4.68°)及11帧激光匹配通过均是历史；随后已后退、导航及切换新SLAM，不能用于当前回位声明或注入初值。before_fivecm_restart_localization.json和before_manual_autoready_restart_localization.json也是旧pose。

## 5. 保留的实验室地图和草稿（宿舍禁用）

数据根 `/home/ubuntu/raicom_tools/field_tool_data/`：

| 文件 | bytes | SHA256 |
| --- | ---: | --- |
| maps/20260913_090007_lab_map.yaml | 141 | `3377095a84e6ccb3678137ae665173bbddb515b8e57c0e5169f49d5a43e0179c` |
| maps/20260913_090007_lab_map.pgm | 59507 | `827cee7b1015d3486d83d88c90aae418a3103c37b055c84fd525f0b2056ce6c3` |
| routes/20260913_090007_lab_map.a08ac14eef3dfc5c.route.json（原始保存） | 1999 | `ada176c08cba3bc82102a009d78593777cd22a89201f7820cc492bfb8238f0d9` |
| 同一路线（当前，仅新增start_localization） | 2481 | `22309d6b74888ac931145003b0c597f5ce1a385ef504d99922b5935be529b6bb` |

points为空、order为空；全部为SLAM草稿，原标签均“启动区”，未擅自改标签或提升为正式点：

| name | x m | y m | yaw deg |
| --- | ---: | ---: | ---: |
| start | -0.001203802 | 0.008701291 | 168.671395 |
| recog | -0.262645544 | 0.074412172 | 79.545795 |
| wait_pick | -1.569308095 | 0.424511734 | 169.848586 |
| pick_b | -1.003204922 | -0.235483461 | -106.595761 |
| pick_a | -0.772261728 | 0.693795076 | 73.884528 |

用户定义start为中转区；识别区、待派送区、园区一/二后续正式核验，pick_a/b与两个园区的具体对应未口头确认。新图开始时仍运行旧GUI，没有自动首帧采集；本图新增初始化提示明确来源为 operator-recorded SLAM start draft; fixed-map point not yet surveyed，不伪称新功能自动采集或SLAM最终优化后正式点。新建图以后才由新功能自动记录停车起点。原始route和当前含提示route精确副本均在full_map_artifacts，草稿/reference/order/history未改。

## 6. 旧资产与排除项

- 旧field `20260913_081136_field`：219×242、2cm、origin[-1.61,-4.67,0]；正式start=[.17772063331052013,-1.256271453804792,-.9437539686227012rad]，order=[start]。原YAML/PGM/route SHA分别41cb6049…、989fb7ab…、539da546…，精确副本在本轮field_artifacts，完整SHA见field_assets_verified_backup.json。保留，不与新lab_map混用。
- 旧room三文件SHA仍为31cd035c7c09ca1d3069ee889844f949e8d1f79264df0c0709c387e2ff8832dd、bb7d94d9e6aedfc49e947193ed8cef0159ad60bc8810c4b346f2efb0e58028bf、bd823b7fab9d12b12e3d53981452a28fa9eb805d7057a3738d6b50417514e17c；本轮inventory核对未变。用户沿用2/3授权仅旧room流程测试，不能推为本次全部五草稿已正式通过。
- room历史导航NOT_PASS：旧5cm/5°条件下1成功，2恢复超限取消返回5，3未发；到2最后约.085574m/84.938°，不是到达。新2cm/30°尚未实车验收。旧map_01_2cm_20260901_0240和机械臂配置继续保护。
- 手动链保持GUI→独立keyboard_input→teleop_cmd→唯一guard→controller；W/S .25m/s、Q/E .50rad/s、A/D横移封存，右前轮卡住未修复。不改STM32/协议/PID/麦轮运动学，不重开IMU标定。
- scan_raw→过滤→scan；EKF30Hz融合命令积分vx/vy和IMU角速度，不是编码器里程计。现有SLAM2.6.8航向门控、queue1、私有超量程修复/细匹配保持；不重试FIFO/多线程堆积，不删alignment critics，不拿离线11档速度冒充生产变更。
- guard20Hz/150ms、keyboard200ms、lease250ms、scan/TF≤.5s、zero hold≥.5s保持。超时停车后不自动续走。用户现场净空/急停持续授权不重复询问；助手非零动作仍需当前可观察的具体阶段。不要第二套底盘/雷达/EKF/guard、厂家整套navigation或~/.stop_ros.sh。

## 7. Git、回退与接手

- 真Git仓库 `<LOCAL_WORKSPACE>\raicom_control`，main，HEAD `dbac0a67c13acf51541a3e1ea99afc9f74e77ab0`，最近dbac0a6/86f32cb/2d82dc4。52跟踪脏文件、大量既有未跟踪，暂存0；本轮无commit/reset/clean/全量同步。tag pre_point_save_resume_20260913仅保护HEAD，不保护脏文件。
- 本轮修改前完整CURRENT/锚点在本轮 `before/docs/`；相机最新16:37记录保留于下节。前序恢复/超时写盘六文件等完整记录见 `runtime_evidence/20260913_lab_mapping_resume/docs_before_camera_replug_update/` 和历史交接。第一轮root写入曾超时后成功，以SHA核对，不重复固定旧SHA apply。
- 先读本文件，再查Pi实时状态、D候选、部署回执和实际进程加载；不能用旧boot/PID操作。每完成关键阶段同步本文件。辅助说明见 [FIELD_TOOL](FIELD_TOOL.md)、[点位锚点](POINT_SAMPLING_ANCHOR.md)。

## 8. 本次相机检查证据（16:33）

证据均位于runtime_evidence/20260913_lab_mapping_resume：camera_replug_host_identity.json记录新boot/IP/USB与服务；camera_replug_host_devices.json记录USB/sysfs、CSI内核节点、三容器状态；camera_replug_ros_streams.json记录3秒发现+6秒只读观察，未见相机驱动/话题/帧；camera_replug_csi_app.json确认camera_app.py为按需拍照程序；camera_replug_app_output.json只读确认stdout/stderr为pipe，未读取活动管道、未把空docker终端日志称为相机无报错。两相机识别通过；实际出图本次尚未验证。旧CURRENT/阶段锚点/FIELD_TOOL精确原件保存在docs_before_camera_replug_update。

## 9. OV5647真实拍照阶段

用户“看是否可以拍到照片”授权后，助手只调用既有camera_app.py端口8000的/api/capture_temp一次；保存前临时照片不存在，无旧预览被覆盖。新原图在`runtime_evidence/20260913_lab_mapping_resume/ov5647_capture_1/ov5647_test_photo.jpg`，648×486、18913bytes、SHA `1149a9c6cb95fb4cfe1d6e468e0c7c3312410a82524f14d96526472659210b8d`。实际看到现场，明显过曝、方向侧转；未修图或调相机参数。

第一版辅助脚本已成功取得JPEG，随后因本地没有Pillow而在可选解码步骤退出，不能把该本地错误记成相机拍照失败。未重拍、未安装依赖，改用已有Windows System.Drawing完整解码、查看原图，并只读回取Pi临时文件SHA/mtime与参数核验。原HTTP响应未落盘，不伪造原始回执；证据是csi_photo_readback.json与ov5647_capture_1/capture_verified.json。辅助脚本已将回执写在可选解码之前，防止以后缺库丢回执，未修改生产拍照程序。

当前相机参数width648/height486、exp_time80000µs、a_gain8、r_gain1.3/b_gain1.5、saturation1/contrast1、preview_rotate=false；未更改。不调用save_temp/discard_temp、不写图库、不重启服务/容器、不启动Aurora、不发底盘或机械臂动作。新照片成功不等于曝光质量或长时稳定性通过。

## 10. 实验室五点可行性核验（17:28，延期任务）

用户“继续确认五点是否可行，有把握一次成功才实测”。本轮只读回取Pi实际map/route/所选install导航参数与launch七份精确文件，map绑定通过；global/local车体footprint及5cm padding一致。

| 点 | 车体占格 | 加原5cm余量占格 | 2cm/30°范围无碰撞样本 | 本轮结论 |
| --- | ---: | ---: | ---: | --- |
| start | 0 | 0 | 2015/2015 | 静态通过 |
| recog | 0 | 0 | 2015/2015 | 静态通过 |
| wait_pick | 0 | 28 | 0/2015 | 余量冲突，不实测 |
| pick_b | 0 | 28 | 0/2015 | 余量冲突，不实测 |
| pick_a | 2 | 34 | 0/2015 | 车体本身冲突，不实测 |

相交格均为occupied，无unknown/越界。采样为65位置（中心、1/2cm半径各32方位）×31角度（±30°每2°）；是有限采样，不写成连续域无解证明。原位姿冲突已足够否决本次五点实测。全朝向每5°的72样本中start/recog全通过，wait_pick/pick_b各66冲突，pick_a72冲突。

start→recog以带原padding车体外接圆半径.21027971284051603m、禁止切角的离线A*找到.29313708498984764m路径；不是实际Nav2规划，更不是整路线/到点通过。原朝向附近平移诊断在wait_pick/pick_b各5cm、pick_a6cm找到无碰撞样本，仅为离障碍更远重新选点的证据，未写回、不是推荐业务点、没有扩大2cm容差。

本轮实车7秒141条速度全零、scan61、最大scan年龄.108449726s；odom TF最大年龄.989974659s，不称全窗通过.5s门槛。固定地图TF仍未建立，无Nav2/controller，未发initialpose/导航目标，最终位置/角度误差与导航返回状态均无。电池原程序读取唯一底盘发布者7370/7368/7375mV，中位7.370V，本次静止不低压；带载供电仍未验收。

地图/route字节与上一阶段一致，正式points/order仍空。本轮无生产代码变更。报告与地图占格放大图见 [五点可行性](../runtime_evidence/20260913_five_point_feasibility/README.md)。Context7额度耗尽后查官方Humble接口和Nav2足迹文档，不把文档/离线算法结论替代Pi实际运行结果。

收尾独立4秒只读复核22条速度全部零、末值[0,0,0]，唯一guard发布者、无map发布者/导航节点；新lab_map YAML/PGM/route三SHA逐一未变，见本阶段final_unchanged_stopped.json。当前没有导航返回状态或到点误差可报告。

## 宿舍hall新阶段（18:29）

用户已确认hall为本轮宿舍地图，不创建dorm_nav。精确快照hall_observed_1/：YAML SHA8d7780815d9498f02ce0e07d053cafb68ed195c079613a280a4d6e6afdc3f845，PGM SHA10ea46ed65ecaa3eca263a95e1162eff12713a175416683625182e033565686a；route `20260913_095722_hall.eb5644e3cb506a29.route.json` SHA0d58db24e4e48815b257c405a60916f04a4e6b8c1d6cf6a18b216d60bfc668ab，1746bytes。地图2cm、origin[-2.49,-1.33,0]。本快照points/order/history为空。

草稿：start=[0,0,-.028451382448213092rad]，2=[.0025488038886203983,.5576426277717502,1.5803448954670185rad]，3=[-.543514022760031,.941068863946182,-.03203297502786677rad]。start来源自动stationary mapping start，start_localization同坐标/方向且requires_same_position_and_heading=true。不能将坐标恰好为0解释为强制重置odom或已正式采点。

新boot地图保存时间按Pi UTC文件名；Windows观测时间与Pi桌面钟约有30分钟差，本阶段以boot、独立时间字段、实际文件SHA和ROS同机时间判别，不按跨机显示时刻倒推因果。未改系统时钟。

## 宿舍第1次单点实车结果与继续点（18:58）

原GUI4ba72ac56b1c0ac3d320766f8723a07c7faafff628decc2326b253689e9b45c6已部署并从终端FIELD_TOOL_LOADED确认；57项原生/mock测试通过。路线窗增加独立草稿列表、选择后填写采点名称、刷新已保存路线按钮；草稿不会自动变正式，刷新不重启定位。新route SHA ba8371786cfbf1df511721198398ed72190507a8a861c187eb84a32b8502c922，只将点2按原坐标加入流程测试队列，保留全部drafts、map和start_localization。start/3原5cm余量各占2个障碍格，未加入测试。

本阶段加载误选room一次，无初始位姿/导航发向room。XTest键盘受RViz焦点影响，End及Home/Down均未真正选择，不再重试；修复为打开原ComboboxPopdown后直接点击其385×128 Listbox第4行hall。首个已加载room的GUI正常关闭后重新打开，先确认选中hall再从原手动纠偏按钮加载，不自动start。当前GUI78861/138003、keyboard79271/138536、map_server88831/155363、amcl88833/155363；仍须使用前重新读取，不直接kill。

重新定位使用重开前实际停车pose，经当前激光3帧重合重新验证后，仅向原30秒/raicom/initialpose_candidate窗口发布一次；原GUI日志确认转发。读取时odom位置不变但航向跨重开阶段变化50.59/53.41°，只记录异常，不把它当物理转动证据；新鲜激光对保存停车pose的误差中位约8.8mm，故不沿用原room窄协方差或伪称odom航向稳定。恢复后9帧地图/激光检查通过，225条速度为零，最终map pose重新收敛在[-.041,-.019,约-1.4°]。

原GUI“开始导航”紧接“停止”仅预热自身Nav2进程且取消待发队列，保留进程供同一面板后续使用。hall_nav_live_parameters_and_path_v3.json实际7生命周期active，SimpleGoalChecker xy.02/yaw.5235987756/stateful=false，DWB xy.02/vx5/vy1/vtheta7，FollowPath.stateful未声明。实际ComputePathToPose返回4、路径.5839607674911709m。预检77条速度为零。v1参数批量请求遇未声明字段返回空，v2新只读监听器TF未发现失败；v3逐项读参并有界等待新鲜TF通过，前三文件不可混称全成功。

本次目标UUID39a271b9ec3148c7a305eb90f0f00f67，mission field-nav-20260913T102411，action2→3→5。最大速度vx.125m/s、vy0、w.35rad/s。起初转到约25°停顿，两次Failed to make progress后原恢复让它继续前进，在距点2约.126m又停顿，第3次恢复触发面板保护并取消。助手原停止按钮尝试因窗口错误文字改变按钮宽度(386×28)触发断言，未点击；实际取消来自原恢复保护。不要把旧observer精确非零计数1017都算实质运动，许多为-2.8e-17残差；独立停车92条速度按1e-6阈值全零。

停车最终误差：位置0.126327425m，最小朝向角13.721756°，Nav2返回5，NOT_PASS。observer末帧导航active=false、速度[0,0,0]、TF年龄0.018212593s。当前只读代价地图中车体加padding在停车点和目标点均无≥254格，重新规划仍返回4；这不等于DWB能生成有效收敛速度。下一步在ROS_DOMAIN_ID87隔离使用原native DWB probe检查真实候选，不直接把旧room的离线11档结果当宿舍实车通过。

## 5cm生效与第2次实车结果（本次最新）

本次只改GUI显示、model默认值、navigation logic统一位置常量和controller两处xy值；goal checker/DWB实际均.05，yaw仍.5235987756、stateful=false。旧route.position_tolerance_m=.02作为历史保留，当前GUI构造NavigationTarget显式用统一常量，实际发目标前回写仍.05，不存在旧点强制2cm。trans_stopped_velocity=.02是速度阈值，resolution/linear_granularity=.02都保持。

部署exact backup在pi_before_fivecm，磁盘回执fivecm_disk_deployment.json，真实GUI加载b0087cab…/ROS48d04288…/logic31cf33db…/modelb58e3ecc…，controller a4baea8a… source/install一致。新GUI首次定位map_server configure响应超时，manager未继续AMCL；原GUI受控关闭后重新从原入口开，第二次正常active。过早重开拒绝和短暂launcher未存活均如实保留，不删锁/不并行第二套服务。最新入口归位门禁PHYSICAL_READBACK_PASS、command_sent=false。

重启前停车pose为[.0198986065,.4429985785,1.7809390204rad]；以当前激光3帧验证中位约9mm、p90约19mm，odom平移不变但航向约-16°差异，未当成物理旋转证据。原30秒initialpose_candidate窗口只发布一次，真实面板确认接受；11帧地图/激光重合通过，226条零速，map cells SHA一致。没有使用start或实验室初值。

fivecm_active_parameters_and_path.json：七生命周期active，实际SimpleGoalChecker/DWB .05、30°、false，341条预检速度零，规划status4，路径.1024621102m；只是规划通过。电池7.888V，原发布者唯一、不低压。

本次第2次目标UUID9b95bc7fb08c48898b6fd4aad5e41c58，mission field-nav-20260913T104739。原GUI发目标后只给小角调整而无前后速度，停滞后一次恢复仍未接近；助手已成功点击原队列Stop（345×28），Nav2返回5/CANCELED。完整120秒只读流hall_point2_trial2_fivecm.ndjson，末帧速度[0,0,0]且navigation_active=false；独立停车检查113条全零，重规划仍成功。最终误差和pose见hall_trial2_fivecm_result.json，NOT_PASS。AMCL地图误差不是外部物理测量。

hall_after_fivecm_trial精确回取YAML/PGM/route三份SHA均未变；fivecm_preserved_assets_verified.json核对之前16份实验室/旧图资产SHA全保留。没有新增第三次目标；导航服务可保持idle供后续同一面板使用，不盲目重启或把停止当已到达。

## 15. 2026-09-13 手动自动就绪实机阶段

原阻断证据manual_ownership_before.json：teleop_cmd原keyboard_input+nav_cmd_bridge同时存在，导航active=false，旧代码要求计数1导致按钮永远不能开。只精选部署GUI/ROS两个文件，exact backup在pi_before_manual_autoready，deployment回执manual_autoready_disk_deployment.json；未全量同步/修改地图点位/控制器容差/底盘。

原入口重启后manual_auto_startup.png已自动就绪，101条速度零。随后通过原非start加载hall，以重启前停车pose经3帧新鲜激光检查、122条零速度后走原一次性纠偏入口恢复，记录manual_restore_prior_applied.json。此旧pose后来已过期，不得重放。

必须区分一次未纳入实时观察器的实际导航：manual_after_preflight_logs.json记录Pi11:06:41启动、11:07:02队列开始，Nav2约11:07:04接受从(.01,-.01)到点2目标，11:07:15取消。助手preflight回执在Windows19:35:33，返回“已有导航请求”后Stop，因此这份回执不能描述为全阶段从未发过目标。启动早于该助手preflight，现有记录不能独立归因；用户后来明确实体已回start。此次没有连续速度/误差流，不填写到点误差或称成功；保留原日志事实，不把短窗零速扩展到整段。

其后独立7秒133条速度零，manual_current_pose_and_processes.json；manual_nav_services_readback.json七生命周期active、goal checker/DWB .05、yaw .5235987756、stateful=false，ComputePathToPose返回4、路径.58m，只是规划检查。原manual_toggle恢复后双发布端共存88条速度零并显示手动就绪。

用户确认回start/同车头后，manual_start_confirmed_alignment.json采11帧、207条速度零：激光端点至地图occupied格中心中位约1.1cm、P90约2.1cm，每帧符合既有静态阈值；不是外部物理位置真值，也不是到点验收。manual_final_hall_assets再次核对hall YAML/PGM/route三份SHA完全保持。

## 16. 2026-09-13 手动修复后续的DWB只读复现

manual_dwb_diagnose.py以当前controller SHA a4baea8a…、hall_trial2_fivecm_stopped_diagnostic.json原始网格/TF/路径和实际footprint，在私有ROS_DOMAIN_ID87/localhost调用安装的DWB 1.1.15插件；没有目标客户端/底盘速度发布器。原5档复现最佳零速，全部正vx候选因ObstacleFootprint非法；21档最佳vx=-.025m/s、w=-.116667rad/s。完整scores保留于hall_trial2_dwb_offline_baseline_vx5.txt和hall_trial2_dwb_offline_candidate_vx21.txt，摘要manual_stage_dwb_findings.json。

只比较真实候选和分项成本，未修改生产参数。新建critic/零速度/冻结网格不能复现历史Oscillation状态或证明闭环到点；不据此开始新实车目标。Context7配额已耗尽，改读Nav2 Humble官方GoalAlign/PathAlign源码辅助解释，最终行为以原生插件结果为依据。手动仍保持原面板就绪，用户已确认start同位置同车头；位置验收仍5cm、朝向30°且停车后同时满足。

## 17. 2026-09-13 新参数部署与第3次实测准备

hall_resume_navigation_state.json同boot/原GUI，112条速度零；hall_start_current_stopped_diagnostic.json实际起点冻结图。hall_start_full_polygon_departure.json完整polygon含5cm padding在当前姿态相交1个254格，后退2cm起至20cm各采样可转至+100°。用户确认后退约5cm，实际hall_after_departure_stopped_diagnostic.json定位x=-.1051680927,y=-.0083176627,yaw=.0736106471，112条速度零；不据人工距离描述替代TF，也未改start资产。

原生DWB持续状态复现工具tmp/hall_dwb_closed_loop.cpp和hall_dwb_convergence.py，不含目标客户端/底盘发布器，使用ROS_DOMAIN_ID87。冻结网格、1ms加速度积分、近似1.3秒重规划、原footprint padding及独立扫掠检查；不覆盖真实传感器更新、物理轮滑、时延或完整实际BT恢复。GoalDist64近点通过，原start直接出发停滞；假设退出5cm及实际退出位置完整轨迹在PathDist32/64下出现扫掠254，细化linear=.005/angular=.01也失败。最终PathDist128/GoalDist64/vx21在实际后退位置及旧近目标两个快照通过；分别hall_dwb_convergence_departure_vx21_g64_p128.txt、hall_dwb_convergence_near_vx21_g64_p128.txt；同名_params.yaml保留候选配置。没有采用失败候选，也未改变碰撞步长。

原配置source/install精确备份pi_before_hall_convergence_controller，hall_controller_exact_backup.json。hall_controller_disk_deployment.json核对仅三项ROS值变化并原子替换；原GUI立即停车后通过原lifecycle_manager_navigation RESET→停用态设置三参数→STARTUP重建加载DWB插件。hall_controller_live_reconfigure.json七节点active，246条速度零、navigation_active=false、无活动目标，参数21/128/64与5cm/30°/false再次回读一致；未重启GUI/AMCL/底盘/guard，未发初始位姿。只修改参数回读而不重新加载plugin不能证明critic生效，因此本轮实际做了生命周期重配置。

旧离线helper里的a4...配置SHA预检是此前快照绑定，当前D盘已变d06...；以后复现须使用对应保存的_params.yaml或精确旧配置，不盲改预检冒充旧快照一致。备份/disk/reconfigure已执行，不重复已有回执。

## 18. 第3次取消与当前hall1非零start核验

hall_trial3_active_params_path.json先核验七节点active、21/128/64、goal checker与DWB均.05、yaw=.5235987756、stateful=false；实际路径.5747726622m。随后先运行120秒只读观察器，再从原路线窗口按钮启动，hall_point2_trial3_start.json。hall_point2_trial3_candidate.ndjson记录UUID238a3daf64cb41d98caedcd72d692114、mission field-nav-20260913T115001，状态2→3→5；两次状态端点相隔73.417秒，峰值速度[vx=.20,vy=0,w=.35]，一次恢复，2362条速度样本中1401条非零。车辆最初接近点2，后来仍未收敛。观察器仅记录、不负责异常取消；其长段输出在完成后读回，不能把连续记录宣称成助手全程实时干预监控。后续运动必须保证实时检查不中断及异常停止可执行。

hall_point2_trial3_result.json记录NOT_PASS；5/CANCELED后首帧旧hall TF位置误差.1957590694m、最小角差13.1716437°，当帧输出零、navigation_active=false。这不是稳定最终验收误差。此次取消早于助手后来的原面板急停；日志显示随后定位/Nav2退出、Pi11:51:24与11:52:21两次新建图，不能把取消归因为该后续急停。当前已是另一会话，禁止用观察器后半段x约-2.2的坐标计算旧hall点2误差，也不能仅凭跨会话坐标跳变断言AMCL漂移。

用户插问start不为000后，直接抓取hall_trial3_current_start_question.png：地图名输入hall1、新SLAM RViz，草稿start=(-2.903,1.252,100.1°)、面板当前=(-2.909,1.239,100.6°)，显示精度下相对距离1.4318cm/角差.5°。hall_trial3_current_mapping_audit.json确认/map仅slam_toolbox、无AMCL/map_server/Nav2、odom x=-2.902408889/y=1.251893604，130条速度全零。当前大坐标来自持续运行的odom及新图没有归零，不代表车辆离自己的start三米。原GUI start_mapping只启动SLAM，_maybe_capture_mapping_start直接保存真实map位姿；online_node.cpp将getOdomPose结果送入addScan，没有减去起点。只记录起点和以后按保存起点初始化，与将新地图原点定义为起点是不同功能；本轮问题解释未改架构或坐标。

hall1_source_inventory.json确认新GUI仍原88bb1c...源码、当前SLAM原online_range_candidate；hall1没有落盘地图/route。hall_trial3_preserved_assets/manifest.json再次证明旧hall YAML/PGM/route三SHA不变，实验室三资产SHA也与第5节一致。控制器磁盘d06...保持，但当前无运行Nav2，且该候选未获实车通过。助手原emergency_stop回执hall_trial3_post_observer_emergency.json锁定手动并维持停车，未停止/保存/覆盖当前新图；用户若继续扫描可从原面板恢复手动。

## 19. 新hall1保存及点位文字显示只读诊断

20:36快照hall1_point3_assets包含当前hall1 YAML/PGM/route精确副本与manifest。YAML SHA53b900c2339e7391c52d5e4026ac294a0c20c0f923b5f5b9a24b213c2daa0e0e，PGM SHA28585a9e10b09b310de2c8921e469edb294591a0217ac2159aa19c37c991dcec，236×175、resolution.02、origin[-5.69,-1.01,0]。route为20260913_115900_hall1.00cdf80690e9edd6.route.json，捕获SHA以manifest为准：只读期间用户仍编辑，初始inventory与稍后的精确捕获不是同一版本。原点2从(-3.9609,1.2278,-173.76°)被纠正到(-2.9303,1.2439,96.20°)，随后Pi12:05:03删除；最新order=[1,3]。不可把两个近乎相同的1/2点称为两个不同的实体目标。

点3reference=approach=(-2.8624530841,.0088993291,-81.0224933°)，offset=[0,0]，Pi12:01:36正式记录。原SLAM草稿3=(-3.0463676211,-.0143259183,-83.4356284°)，两次记录相差18.54cm；缺采点时连续物理/定位证据，不能仅据此归因为漂移或人为移动。正式3距保存start约1.2436m。用户澄清只问“数字2看起来贴障碍”，不是声称实体采点位置错误。

hall1_point3_question_live.png与hall1_point3_marker_audit.json证实publish_points使用整段长文字，TEXT_VIEW_FACING(type9)位姿等于route坐标；reference与approach重叠、中文显示留白使左端数字和右端approach分离，单看数字位置会误认为点贴障碍。另enumerate(order,1)用队列序号而非point name：当前点名3发出的文字确为“2 中转区 reference/approach”。本轮只解释，未修改生产代码。后续若修显示，应短名称加精确位置标记、朝向箭头，使用真实点名，保留现有坐标/地图和两个位姿语义。

hall1_point3_grid_marker_findings.json仅检查标记中心：PGM原始灰度254为白色自由格（不是Nav2 cost254）；点1/已删点2/点3中心均自由。点3中心距最近occupied格中心约16.27cm，不能据中心自由代替完整footprint与padding检查。本轮未做新hall1碰撞/导航验收。实时map_server yaml_filename与hall1一致、AMCL/Nav2节点存在；5秒92条速度非零，未记录来源归属，因此不称已停车或助手发起导航。原图截图的failed/arrival outside tolerance尚需后续读取对应日志，不能嫁接旧hall第3次UUID作为新图结果。

## 20. hall1 两点往返功能部署与实测前恢复

用户选择111↔222自动往返5轮。这测定位与纠偏后的导航重复精度，没有外部尺测，不能写成底盘实际定距精度。222此前已由用户正式保存；错误提示来自定位模式点击“建图草稿”入口。用户后续重采111，最终冻结111=(-2.9286846280331034,.1822211701193912,-77.19814699242217°)，222=(-3.184572620529568,1.1773156529849755,101.24467546675983°)，间距1.0274685856674137m。reference=approach，零offset。精确资产hall1_repeat_frozen_assets_v2/；普通order=[1,3,111,222]不改。

hall1_repeat_preflight_v3.json：地图/route指纹正确，9生命周期active，实际SimpleGoalChecker/DWB .05、yaw .5235987756、stateful=false，原16点footprint/padding .05一致，两端点及当前车体清空、每5°全周转向采样通过；ComputePath双向返回4、约1.04m。仅规划无运动。前两次预检分别因用户手动移动及重采111导致route变化失败，已保留失败回执，没有测试目标。

GUI与navigation_logic两文件新增往返临时序列、独立mission ID、plan/result文件及逐段停车误差。原生102项PASS（repeat_test_native_regression.txt），测试不发运动。精确旧文件pi_before_repeat_test/；部署回执repeat_test_disk_deployment.json，最终新进程加载见hall1_repeat_selection_v4.png。两次原入口重开均机械臂readback通过且command_sent=false。第一次重开误选room，hall1_repeat_restore_check.json按地图指纹拦截，未发初值；正常退出该空闲会话，第二次正确选hall1。Combobox End不选择最后项，不重复该失败操作；切换下拉但已运行旧地图时arm_pose不自动换图。

hall1_repeat_load_nonstart_v2.json加载hall1，hall1_repeat_restore_check_v2.json与hall1_repeat_restore_applied.json以重启前111停车pose经3帧静态匹配、odom平移不变、121条零速后，原一次性候选门只发布一次。hall1_repeat_restored_alignment.json后10帧通过、220条速度零。随后用户明确搬动/转向，画面跳至约(-2.848,-.555,119°)，不是本次导航移动或已证明的自发定位故障。原面板急停hall1_repeat_pose_jump_stop.json，120秒观察器hall1_repeat_pose_jump_observer.ndjson共2400条速度零、无活动导航；旧恢复pose禁止重放。

用户现确认已放回222/同车头。hall1_repeat_222_before_seed.json为新停车odom参考，hall1_repeat_222_seed_check.json初值激光检查失败，候选尚未发布。只读附近匹配脚本tmp/hall1_repeat_local_match.py，结果见新回执；不部署私有匹配器、不开RF2O。RF2O实际核验hall1_rf2o_runtime_audit.json：没有相关节点/进程/话题，/odom_raw唯一odom_publisher、/odom唯一ekf_filter_node。ekf参数服务本次未读到，不把空值写成参数回读成功。配置与README历史记录一致，RF2O私有试验未接入生产。

继续工具：tmp/hall1_repeat_ui.py可dismiss_queue、repeat_dialog、confirm_dialog；最终轮数对话框确认可能启动运动，必须先开tmp/hall1_repeat_observer.py（600秒只读）并实时监控。repeat按钮主窗局部(250,266)，原按钮大小482×28。已执行第一对话框111,222及5轮确认，起点超限拒绝，见hall1_repeat_first_start_state.png；尚无本测试导航目标。任何新回执不能覆盖。

原UI点击辅助脚本tmp/point_stage_gui_click.py的hit曾遍历隐藏的Notebook页面，切过导航页后误命中隐藏502×618页面，arm_pose的尺寸断言正确拦截两次。已加XGetWindowAttributes/map_state==IsViewable过滤，v3命中真实482×28原纠偏按钮。hall1_repeat_matched_seed_applied.json仅发布候选、当时门未开启，不能称initialpose已应用；v2 gate也失败，未再发候选。v3 gate与hall1_repeat_matched_seed_applied_v3.json才恢复成功，后续hall1_repeat_matched_pose_live.json确认真实map位姿改变。此处仅修改Windows传输辅助脚本，生产GUI SHA不变。

用户第一次左移后的证据hall1_repeat_222_reposition_check.json、hall1_repeat_222_reposition_applied.json、hall1_repeat_222_reposition_live.json：新静态匹配通过，单次原候选门初始化，134条零速度、短窗4.8139cm/2.9909°。后续完整观察流收敛到5cm之外，起点门拒绝第一轮测试；未把初始化短窗或临界值挑选当稳定到点。hall1_repeat_preserved_assets_before_test.json核对hall1、旧hall、实验室地图/路线指纹均保持。等待额外左移5cm后，重新核验当前实体位姿，不直接重放已过期pose。

## 21. 前8段统计交付、电量低暂停

用户依次确认放回222、同朝向左移约22cm、再左移约5cm。最后3帧激光中位约8.6mm/P90约13mm，原候选门恢复后进入起点范围；启动初值约(-3.17000,1.16003,103.6374°)。第一对话框起点超限拒绝，第二次45秒冷启动超时均未发目标；第三次在服务已全部active后从原两点往返入口111,222/5轮启动，mission field-repeat-20260913T125848-11288062183491，目标序列[111,222]×5。不得把前两次启动准备算作导航失败样本。

本次真实9个目标均Nav2 status4/SUCCEEDED；前8个停车后通过，第9个到111时停车pose=(-2.881620733970741,.20111182194692345,-101.07356234579198°)，误差5.0713576592cm/23.8754153534°，原面板FAILED，accepted=false，第10个未发送。mission结果文件与完整arrivals保存在hall1_repeat_final_receipts.json；原始连续监视hall1_repeat_attempt1_execution.ndjson，开始前已启动观察器，有原始action UUID、速度、TF、scan、恢复数。前序冷启动观察器为hall1_repeat_attempt1_observer.ndjson，不能混用elapsed起点。截图hall1_repeat_terminal.png显示failed；普通列表仍[1,3,111,222]，临时测试未改保存顺序。

用户报告电量低并指定只用前8段计算平均误差，故统计范围为前4轮，n=8，不含第9段失败或未执行的第10段。到111的4次平均位置2.6046571730cm、平均最小朝向21.8752548396°；到222的4次为3.2679437888cm/23.4812210979°；合计2.9363004809cm/22.6782379687°。合计位置样本标准差1.4351134234cm。各段均为停车后位置欧氏误差和朝向最小夹角绝对值，不进行正负抵消。

独立终止检查hall1_repeat_terminal_stationary_audit.json：133条速度全零，当前map为hall1、唯一底盘发布者cmd_vel_guard。其稍后定位仍会变化，不回改第9段验收结论。hall1_repeat_terminal_assets.json与冻结及测试前全资产manifest比较，hall1、旧hall、实验室等地图路线SHA全部保持。实车暂停，不另发任务或重启底盘；报告hall1_first_eight_report.md和JSON已生成并复核。

## 22. 3cm/15°本地候选完成，实车仍暂停

用户在前8段均值交付后要求平均位置少于3cm、朝向约15°，随后明确“容差也改成3cm”。因此新单次验收条件为位置欧氏误差≤.03m AND 朝向最小夹角≤15°，并继续要求停车后用同一新鲜map位姿复核。原前8段2.9363cm均值已低于3cm，但不能用调参承诺后续真实平均误差，也不把原30°样本重标为新标准通过。

本次只修改D盘四个生产候选文件；面板原已引用统一常量，显示、目标构造及临时往返起点检查随之变为3cm/15°，GUI文件本身未改。model新点默认同步，旧点文件的2cm/5cm/30°历史字段继续可读、保持原内容；目标构造使用统一标准，不受旧metadata覆盖。ROS参数请求改为按输入角度通用转换弧度，去掉旧的“输入30°则套用默认弧度常量”分支，避免新默认15°时误换算旧30°调用。

| 文件 | D盘候选SHA256（未部署） |
| --- | --- |
| field_tool_navigation_logic.py | `53dadac3b5a660aee4dc84c9a37ee95f1004c58cea6e9d7def655a2624bdc84a` |
| field_tool_ros.py | `40ee3661d455c7908fb603f6ccebba404fa389c88db77aa2c51f2c2310940364` |
| field_tool_model.py | `a6a142abf7690dfbbd07c900ab2fcf32b0441aaeccb3300e0e7409bbbec375cb` |
| nav2_controller_straight.yaml | `a5f5ab0e05781a473738e267fc3b5c1881181c39540759c29d5ed1ac63af4f6a` |

配置核对：原field_tool_navigation入口选用nav2_controller_straight.yaml，general_goal_checker仅一个SimpleGoalChecker，xy=.03、yaw=.2617993877991494rad、stateful=false；DWB FollowPath.xy=.03。实际ROS配置仅改变这三个值，速度、加速度、critics、采样数与碰撞设置保持。progress_checker.required_movement_radius=.05、停车速度阈值.02、地图resolution=.02、footprint_padding=.05、路径平滑器迭代tolerance=1e-10各有独立用途，均不是到点距离要求，不能按关键词一并改小。旧点合法范围上限30°仅为历史数据兼容，不是面板当前目标角度阈值。

Context7返回月配额耗尽，改查[Nav2 Humble SimpleGoalChecker源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/plugins/simple_goal_checker.cpp)和[RotateToGoal源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_critics/src/rotate_to_goal.cpp)：stateful=false保持每次检查位置；DWB近点转向阶段仍有内部锁存，但不能豁免goal checker或面板停车后的同步验收。RotateToGoal在onInit读取xy并缓存，因此未来部署必须重新加载原Nav2/DWB插件，单次set_parameters ACK或参数回读不能证明内部阈值已换成3cm。延续既有重配置流程，不重做底层架构或自行添加不存在的FollowPath.stateful。

证据目录 `runtime_evidence/20260913_precision_3cm_15deg_candidate/`：before/及before_manifest.json为本轮修改前11文件精确备份；local_regression_v3.txt为47项离线PASS，覆盖2.9cm/14.9°停车通过、3.1cm或15.1°拒绝、未停车不取验收pose、服务未active不发目标，以及新旧点兼容和配置一致。native_test_availability.txt明确原生ROS/Tk模块跳过；前两份local_regression日志分别是Windows标准库test名称冲突、直接加载原生模块触发SkipTest的运行入口失败，不是通过回执。候选实际参数构造方法另以轻量消息替身离线检查15°、旧30°和6°及服务未就绪不发请求；这不替代原生ROS回归。candidate_audit.json和candidate_changes.diff为完整文件SHA、三项配置差异及本轮精确diff；可用tmp/precision_candidate_audit.py审阅生成逻辑，已有回执拒绝覆盖。

本轮未连接Pi、无部署或实车目标，不能报告新阈值“实际生效”或新导航返回状态。恢复时先做原生回归、精选源码及install配置部署、确认实际模块加载与DWB重建，再读取7个导航生命周期active及四项参数ACK/实际值；原TF/激光新鲜度、零速保持和现场观察条件通过后才开始独立的新测试。地图、点位、普通路线与原失败统计均不改。

## 23. 2026-09-14 实验室新lab_text重扫，原生候选已验证

用户最初要求使用lab_text及截图111=(.210,.446,92.6°)、222=(.259,-.566,-83.7°)连续10次；<ROBOT_OR_HOST_IP>/boot f79fa4ac-9776-438f-8bfa-4ce58a3be6eb初查只有活动SLAM，无已保存lab_text，截图两点为草稿且111有旧同名草稿。initial_desktop.png显示地图保存失败，map_saver.log明确1789372955报Failed to spin map subscription（约2秒超时），未生成正式地图。助手当时仅只读检查，无保存/部署/导航操作。

用户随后明确再次重启并重扫，故前次草稿坐标不能转入本轮。reboot_inventory_v1.json核对新boot 97b5cb34-0df3-41af-be6e-a712ef42066f、原生产五源码/安装controller旧SHA、新GUI9530，ROS仅SLAM建图而无AMCL/Nav2服务。reboot_battery_v1.json为原唯一发布者[7956,7944,7961]mV；电量百分比仅工具估计。Screen Live当前画面显示lab_text新扫描、仅start=(0,0,-98.0°)草稿，用户正在操作原面板。已请用户扫描完成后松键停车再回复，等待中不终止活动SLAM、不关闭未保存面板。

precision_native_regression_v1.txt为104项原生Humble/TF与mock GUI、纯逻辑/model回归全部PASS（0.769秒），由tmp/lab_text_native.py复用原内存加载器并强制私有DDS87/localhost，无生产发布器、部署或目标。3cm/15°配置候选与第22节相同；map_saver既有一次2秒等待失败需要保存时核对，先使用原正常保存入口，不据旧错误盲目重做架构。原始CURRENT/FIELD_TOOL/点位锚点备份在docs_before_rescan/。

下一步保持新图与宿舍/旧实验室隔离。扫描完成后先保存本轮地图和草稿并精确取回，才允许为部署关闭原GUI；完成四文件/安装配置精选部署与实际加载确认，确保新DWB插件以3cm初始化。固定图下核验新111/222，起点同位置同朝向须实证，不能把开机odom零点、旧start或前次截图坐标作为本次定位。连续10段原临时往返入口不改普通路线；每段Nav2结果与停车误差均保存，任一失败按既有保护停止，不把失败段删掉拼成连续通过。


## 24. 2026-09-14 lab_text1 / 222与11111：已部署并完成停车预检

用户再次建图，最终选用 maps/20260914_085137_lab_text1.yaml；正式路线 routes/20260914_085137_lab_text1.6b6e549609696ab5.route.json。lab_text1_frozen_before_test.json及同名目录精确保留28份文件。早先lab_text的一格清除仅在D盘single_cell_cleanup_candidate_v2准备，**从未部署**，已随用户换图作废，不得继续应用到任何新图。

precision_native_regression_v1.txt为104项原生隔离PASS。precision_original_gui_closed.json记录旧GUI9530正常清理；precision_disk_deployed.json记录四源码/安装配置精选替换与旧资产SHA保护。原入口重新打开（precision_original_gui_open.json），实时终端FIELD_TOOL_LOADED与第22节四个SHA完全一致，GUI仍3114fb3e6618a52be5ba3a8dae54a58c6452702415aa1ea5fbf408b7bfa8b960。机械臂原门禁PHYSICAL_READBACK_PASS、command_sent=false。该阶段助手未注入初始位姿，地图由用户保存并定位；lab_text1_save_requested.json是按钮尺寸断言失败，不能冒充保存成功操作。

用户明确222到11111后，lab_text1_test_targets_saved.json通过旧route SHA守卫，仅把两个草稿原值加入points/order并保留来源“建图草稿、未固定地图重采”，drafts/start hint/map均不变。路线新SHA cfe512bd9baae32a9e4affba8653277227cc4834df803dd0eb87763beadbbccd，order=[222,11111]。222=(0.005050374411513293,0.14145230492396715,-2.3511823693162395rad)，11111=(0.67124946859234,0.8625699460610287,0.8091672161780841rad)。111不是本轮目标。原窗口刷新回执lab_text1_queue_refreshed.json及实时画面确认上方正式列表显示两点。

复用原面板“开始导航→立即停止待启动队列”只预热GUI拥有的Nav2，回执lab_text1_services_warmup.json；持续只读观察lab_text1_preflight_observer.ndjson确认未发活动目标、底盘全零。lab_text1_active_preflight_v1.ndjson：9个服务active，实际DWB21样本/PathDist128/GoalDist64，goal checker和DWB均xy=.03、yaw=15°、stateful=false；新插件启动读取新配置，非仅参数ACK。两端及current完整footprint+.05转向72样本均无碰撞，ComputePath双向status4、长度.997207683m/.992750166m，108条速度零。尚未NavigateToPose。

lab_text1_alignment_v2.json有效11帧中6通过5异常，曾口头误称通过，已明确更正；v3重新12s停车检查在TF暖机后8/8有效帧通过，中位8–10mm、p90≤19mm，速度零。初始TF缓冲暖机错误保留，不能算有效通过帧。当前点位偏移从约1.7cm增长至约6cm，不能据旧pose通过往返起点门禁，也不把定位估计视为外部尺测真值。先按用户最新要求普通222→11111，到端点后才进行独立10段，结果须以新任务回执为准。


## 25. 2026-09-14 用户主动关闭，明确暂停（最新停止点）

lab_text1_pair_start_v1.json记录在X11查找原路线窗口时AssertionError: []，发生在任何点击前；屏幕随后显示Pi桌面所有场地工具/RViz终端均退出，雷达年龄持续增加。用户明确回复“我主动关闭，先暂停”，因此没有重开服务、注入定位或重试启动。

lab_text1_preflight_observer.ndjson持续记录从启动起控制速度全零、navigation_active=false、没有NavigateToPose UUID；lab_text1_pair_early_receipts.json没有本日新arrival。普通队列尝试发送0目标，10段测试未开始，不存在本轮平均到点误差。只读观察器停止，最后零速独立回执lab_text1_user_pause_zero_audit.json：123条底盘速度全部为零，scan样本0，map_server/AMCL状态{'map_server': None, 'amcl': None}，没有新鲜地图位姿，所有新证据均在runtime_evidence/20260914_lab_text_navigation。

lab_text1_route_ready_snapshot.json和同名目录精确保存已选222/11111正式路线及地图，route SHA cfe512bd9baae32a9e4affba8653277227cc4834df803dd0eb87763beadbbccd；源草稿坐标未改。关窗前实际容差3cm/15°、goal checker stateful=false，DWB xy=.03，当前磁盘保留。既有lab_text一格清理候选仍未部署，不得恢复时误用。

本轮Git仍main/dbac0a6，工作树存在大量此前修改，未提交或清理。第23节的“尚未部署/等待扫描”和第24节“已active/准备发目标”均为过程历史，最新以本节用户暂停和新运行核验为准。


## 26. 2026-09-14 再重启恢复并实车成功一趟，追加任务失败停止

用户要求立即继续，旧boot原GUI51312为僵尸，首次开启辅助检查误判为活进程，随后绕过僵尸判断仅启动原入口。原入口09:25子进程exit1，根因日志不完整未下结论；随后用户明确“重启了”，旧boot修复不继续。新boot d5d57df3-411c-4c23-b25b-86b479dba3c5与SSH .3已实证。用户自行打开原面板并加载lab_text1和定位，助手新boot开面板请求因已有实例被拦住，没有第二面板，也没有注入旧pose。准备的lab_text1_resume_seed.py和select_map.py均未执行，旧pose/旧boot不得复用。

本次使用Pi直接grim快照观察；Screen Live源窗口停止更新后没有把缓存当当前图。lab_text1_reboot_panel_v1.png确认新面板/地图/模型。用户手动驾驶从11111附近到222附近，observer非零前364样本属于用户手动，不能声称全程静止或全由导航产生。助手提示松键后从原队列启动，不直接发速度。

lab_text1_reboot_preflight.ndjson首次读取9服务里导航仍1，保留失败；v2实证全部3、xy=.03/yaw=.2617993877991494/stateful=false、DWB xy=.03，82条速度零，原footprint+.05和双向路径通过。新boot激光检查初5帧异常，后6帧收敛通过；用户随后重定位/手动到222，以后续现场和新TF为当前依据。

普通任务共有三个mission：093415到222返回4但停车后3.1544716cm/9.2508975°被严格拒绝；093427的222和11111均成功（正式回执lab_text1_pair_success_receipts.json）；093455到222成功0.6710320cm/7.1571027°，随后11111连续无进展，反馈恢复3次，原队列取消后NavigateToPose UUID 2cc9b90780b14134a7da075b93a026c5返回5。原bridge日志OPERATOR SPACE STOPPED CURRENT GOAL是通用停车路径日志，不能据此断言用户实际按了空格。没有更改容差或剔除失败样本。

助手尝试原两点往返入口时已有追加普通任务，GUI拦截“请先停止导航，再修改点位或顺序”，没有成功打开重复测试计划。结果lab_text1_navigation_result.json与lab_text1_navigation_report.md；误差为map定位估计，取消样本只是邻近1Hz观察值，不冒充正式到达。持续观察/最后停车回执lab_text1_reboot_final_zero.json；普通route/map内容需末次SHA核对后记录。3cm/15°生产代码未改，旧lab_text单格候选未应用。


## 27. 继续处理卡住：冻结诊断已取，候选编译期间失联（最新）

已保存lab_text1_stall_snapshot.json：原GetCostmap、TF和ComputePath只读捕获，没有运动发布；lab_text1_stall_footprint.json复用当前已回读原footprint+.05。tmp/lab_text1_dwb_candidate.py复用之前原生Humble DWB持续状态冻结网格工具，改验收为3cm/15°；forward候选仅min_vel_x=0，目的检验反向接近目标是否造成当前局部停滞，其他critics/地图/点位/车体/底层保持。

lab_text1_dwb_forward.txt和console记录仅进行到CMake依赖配置即Connection reset，**没有完成编译或仿真，没有候选通过，未部署生产**。候选参数文件lab_text1_dwb_forward_params.yaml仅离线探针参数。原生临时目录/tmp/raicom_lab_text1_dwb_20260914为私有DDS87/localhost，不是新增生产节点；重连后先检查该编译进程是否仍在，不重复启动并行编译。

原observer截至735.299s记录最新mission field-nav-20260914T094259，goal:1:796947693964，NavigateToPose UUID a372b11a6fb84aff9b75340796a582b6，反馈离目标约1.011m，navigation_active=true，最后底盘输出0，随后SSH reset。VNC画面明确Attempting to reconnect / connection was closed remotely。旧失败093455之外又有093431/094259等任务，正式arrivals尚未回取，不能仅列旧三个mission冒充最终全量。助手此阶段只进行只读诊断/隔离编译，没有再发导航目标。

已向用户说明失联且现场停车，并询问是否再次重启及实体是否停住；回复前不发非零/初值或盲目控制。last-observed摘要lab_text1_stall_disconnect_summary.json。上一批成功结果保留，但不是连续10段验收。网络失联不能归因于编译或电量，尚无证据；重连需查看boot/供电/退出日志。Git、生产配置、本轮地图和点位未改。


## 28. 新lab1五点与换电暂停（2026-09-14最新恢复入口）

用户已重新扫描并完成五个固定地图正式点，明确要求验证五点完备性并执行。之后因低电要求“记录当前状态，我换个电池，然后马上恢复当前工作状态”。该暂停优先于继续定位修复/运动。已尝试SSH只读boot查询，5秒连接超时；Screen Live返回live=false、frame_age_ms=86660。未发送本次停车/重启/目标命令，未改任何生产文件、地图或点位。

检查点：[换电状态JSON](../runtime_evidence/20260914_lab_text_navigation/lab1_battery_change_checkpoint/checkpoint.json)。目录含本图原名PGM/YAML/route及修改前CURRENT。来源为已成功取回的lab1_frozen.json与lab1_frozen/；本次31份备份重新逐项验SHA全部通过，不伪称断线后重新取回Pi文件。

| 资产 | 路径/值 |
| --- | --- |
| 地图 | maps/20260914_101211_lab1.yaml |
| 路线 | routes/20260914_101211_lab1.8a9ee38f24abf6e8.route.json |
| YAML SHA | 51d5bf647afd279f45e23b4411443d05b8f91a66513f91575285978618f01f23 |
| PGM SHA | 51255ed214a4bbf80e13eb7041e9dfee2dd06be90a5509597bed9d8425b4903f |
| Route SHA | 5417b5d1fd580ffe456695bffba5a506bcf86cac701184f2ebd94ceb43981925 |
| 顺序 | start→shibie→pick→pick_B→pick_A |

| 正式点 | x m | y m | yaw deg |
| --- | ---: | ---: | ---: |
| start | -0.8205424272 | -0.4788741020 | 41.28348385 |
| shibie | -0.5417437650 | -0.3202387930 | -51.68130271 |
| pick | 0.3571097671 | 0.5183734949 | 39.26280931 |
| pick_B | -0.3620647166 | 0.4376747665 | 126.69121163 |
| pick_A | 0.1252155559 | -0.2651077706 | -51.27450774 |

以上不是待复核草稿，均原固定地图正式采点，标签当前均中转区，不擅自更名。位置/朝向完整精度以原route为准。原配置source/install SHA a5f5ab0e05781a473738e267fc3b5c1881181c39540759c29d5ed1ac63af4f6a；GUI 3114fb3e…，ROS 40ee3661…，model a6a142ab…，logic 53dadac3…。保留3cm/15°及stateful=false、DWB xy=.03；不把历史第4节5cm/30°写成当前生效值。

最后只读observer完整900秒结束，17793条速度累计795条非零来自该观察期原导航/用户操作，不能写成全窗静止。末次899.583s样本速度[0,0,0]、navigation_active=false；后段非零计数不再增长，精确零速尾窗及最后有效pose见检查点。末次map TF已过时，无新鲜最终误差；不能以反馈distance_remaining=0或以前成功两点任务宣称五点成功。当前五点验收仍未通过。

定位诊断：lab1_restore_pose_check.json与lab1_restore_pose_applied.json曾按最后有效停车pose做3帧激光匹配（183–186点，中位约.0095m、p90约.025–.030m、10cm内约96%），原GUI10:27:29开启纠正门禁，10:27:38确认候选通过并发布。其后lab1_runtime_amcl_now.txt持续出现“AMCL cannot publish a pose or update the transform. Please set the initial pose”，TF再次失效。/request_nomotion_update ACK不等于初值恢复；根因未确诊，下一步查实际/initialpose收发QoS、AMCL生命周期与时间日志。不得只重复点击/发布并宣布成功，也不拿旧odom差值否决用户之后实际运动。

失败证据保留：lab1_preflight_v1超时无有效结果；lab1_preflight_v2.ndjson为controller_server发现断言错误文本，lab1_stall_snapshot及v2也是TF失败文本，不是有效JSON网格。当前五点静态/相邻ComputePath未得完整PASS，DWB候选没有完成编译/仿真/部署。不要继续基于这些失败文件运行探针或把第一步超时后串行命令的产物当通过。tmp/lab1_prepare.py的旋转全角扫描已关闭，不能声称做过该扫描。

换电恢复：先核对新boot/IP和原面板/guard唯一性，旧PID不能复用；核对本图三资产SHA；恢复原面板并选择lab1，实体未确认在start同朝向时不得点自动start定位。旧停车pose仅作激光核验候选，不能跨搬动/重启直接当真值。定位持续有效后等待导航服务active，核验五点车体及原5cm余量和四条相邻路径，再按上述原路线逐点导航、到点停车，分别记录Nav2返回/位置误差/最小角度误差。任一失败停止并保留回执，不扩大3cm/15°，不删障碍格或改点。

Git本次重新核验仍main、HEAD dbac0a6，保留大量既有脏文件，无提交/清理。只新增检查点及更新CURRENT，暂停期间不再进行运动或候选部署。


## 29. 换电后不动车五点预检（最新）

用户已开机但实体不在启动区，要求“先验证导航，通过后再放回”。本轮先静态与隔离软件验证，不运行真实定位或导航。SSH .3首次超时，/28小范围端口发现后恢复；host key/原私钥验证的新boot fdc1977a-f7a5-4f4a-a164-79833e03a7eb。实时13文件精确副本位于runtime_evidence/20260914_lab_text_navigation/lab1_after_battery_preflight，live_snapshot.json记录SHA。三地图资产与第28节完全相等。进程清单仅原guard871（parent18、ticks2977），无原面板、AMCL、controller或planner；不擅自开自动start。

完整结论和范围：[预检报告](../runtime_evidence/20260914_lab_text_navigation/lab1_after_battery_preflight/README.md)、[摘要JSON](../runtime_evidence/20260914_lab_text_navigation/lab1_after_battery_preflight/preflight_summary.json)。static_check.json静态检查五个标称目标均不碰撞，start/shibie/pick的72全转向及3007到达范围样本全部清空；pick_B全转向10/72碰撞、到达范围1759/3007清空，pick_A全转向11/72碰撞、到达范围1552/3007清空。有限采样不是实车成功率。原16顶点足迹、.05m padding、.03m/15°保持。使用既有SAT碰撞逻辑并在五个标称姿态与原实现交叉校验一致，未改地图像素或点。

lab1_isolated_planner_v1.ndjson成功：ROS_DOMAIN_ID87、ROS_LOCALHOST_ONLY=1的独立地图服务与原ThetaStar规划器；仅移除实时obstacle层、robot_base_frame=map以在私有域用恒等变换提供显式起点规划，不发布真实TF/initialpose。map_server和planner均active后，start→shibie、shibie→pick、pick→pick_B、pick_B→pick_A返回4，长度依次.3166124215/1.2345668138/.7298484337/.8687938289m。原地图232×225、.02m，原车体/膨胀和ThetaStar参数实际回读并记录，完整冻结costmap也在回执。无隔离速度发布者；两个拥有进程返回0，最终隔离域仅剩观察节点，后退出。未启动真实controller/bridge或发NavigateToPose；不能把这四个规划返回写成实车到点返回。

native_path_terminal_turns.json按实际路径末段方向在保存目标位置转到目标角，每1°检查：pick_B前进25/55、倒车22/128样本有余量碰撞；pick_A前进2/8、倒车32/175。说明简单末端原地转向存在问题，不能据此证明所有提前调整/绕行方式均不可行。保守外接圆离线A*仅前两段可得，后两段未通过其保守条件；原生规划确有四段路径。尚未有动态控制器成功证据，不允许宣称五点完备性PASS。

tmp/lab1_native_dwb_static.py复用现有hall_dwb_closed_loop.cpp，保持生产DWB所有参数，改验收为当前3cm/15°，按四条原生路径在冻结静态全局costmap内积分，最终误差使用原保存目标而非量化后的路径终点。临时域87无机器人命令。输入完整写在lab1_after_battery_preflight/dwb_probe_inputs，manifest含所有SHA；**lab1_native_dwb_static_v1.ndjson是SSH执行前超时文本，没有远端编译或仿真**，不要视为结果。地图路径与本轮体姿约束不依赖机器人当前位置。探针只是冻结静态地图、理想加速度模型，不能代替实时激光和AMCL实车验收。

之后绑定Windows WLAN172.20.10.4重试SSH仍超时，再查热点网段只见其它设备.6的Ubuntu SSH、没有Pi .3；未尝试其它设备认证。已异步请用户确认小车开机并恢复热点，尚未要求搬回start。当前供电/实体停车未知，本轮没发送运动，不能以没有发送命令替代实时零速观测。恢复后核对boot，确认/tmp/raicom_lab1_dwb_static_20260914是否不存在/未启动，才运行 `python tmp/lab1_native_dwb_static.py lab1_native_dwb_static_v2.ndjson`（若新boot需先核对并更新探针boot守卫，不能删除守卫）。保留失败回执。不要重跑已成功的隔离planner目录或static_check的独占写入。处理控制器/末端几何后才进行用户放回start和实测。

本轮没有生产代码/参数、地图/route修改，没有开启原场地面板或机械臂归位。D盘只新增诊断脚本/证据与本交接更新；先前AMCL持续初值问题未在新boot注入假位姿重试。


## 30. 重连后原生控制器检查完成：NOT_PASS（历史boot，后续见33—35节）

用户回复已连上后核对boot变为f3eb4065-7aaa-445f-adbb-2f7a6c112fd7。lab1_newboot_f3eb_readonly_audit.json重新验证13份实车文件全部SHA一致、83速度零、唯一原guard、无实际AMCL/Nav2/GUI；隔离DWB临时目录此前不存在。候选更新boot守卫并再次在远端逐文件hash预检后运行，没有删除守卫。第一次本地缩进错误发生在执行前，修正后lab1_native_dwb_static_v2.ndjson完成编译和四段；先前v1仍为SSH错误文本，不覆盖。

结果：[控制器验证摘要](../runtime_evidence/20260914_lab_text_navigation/lab1_after_battery_preflight/controller_validation_summary.json)、[最新预检报告](../runtime_evidence/20260914_lab_text_navigation/lab1_after_battery_preflight/README.md)。

| 路段 | 离线停车位置误差 | 最小角度误差 | 3cm/15°验收 |
| --- | ---: | ---: | --- |
| start → shibie | 3.4221 cm | 12.0377° | 未通过 |
| shibie → pick | 3.9842 cm | 4.1987° | 未通过 |
| pick → pick_B | 2.5072 cm | 12.2945° | 通过 |
| pick_B → pick_A | 2.1056 cm | 11.8334° | 通过 |

四段各自从保存源姿态初始化，使用之前原生ThetaStar实际路径、原3cm/15°全套DWB、原footprint+.05。DDS87/localhost内冻结静态全局costmap、理想加速度.5m/s²/.7rad/s²、.1s控制和1ms积分，不启动真实controller/bridge，不模拟实时局部雷达/AMCL，不能算连续五点或实车成功。运行reason前两段outside_saved_target_after_stop，后两段stopped_arrival；末次位置误差使用原保存目标而非ThetaStar量化后路径终点。基线p95计算17–20ms，max≤24.34ms；无254格碰撞。dwb_baseline_polygon_review.json对0.1s记录样本完整SAT检查，75/112/125/142样本均无足迹交叉，仍不等于连续物理安全保证。

额外针对性诊断lab1_exact_endpoint_diagnostic_v1.ndjson：仅在隔离snapshot中将最后路径pose改为同一个原保存目标，原路线文件不变，未修改任何生产参数。前两段在120s模拟上限时距目标.1005191208m/.1020877096m，未成功；后两段.0272559432m/12.2945°、.0284275433m/12.6356°软件通过。dwb_exact_endpoint_polygon_review.json全部记录样本无多边形碰撞。**仅精确路径终点不是已验证修复，列为本轮排除的独立部署方法，不再原样重试**；不得临时扩大容差或把前两段失败剔除拼成五点通过。

证据对第29节结论的细化：pick_B/pick_A保存位置全转向及部分验收区域确有余量限制，但原DWB在允许误差范围内另选位置并完成后两段；不能据原位旋转冲突声称它们实际控制器必失败。前两段基线说明路径终点量化与保存点验收不一致，精确末端诊断则揭示约10cm处的额外局部停滞，critic得分/目标吸引与原alignment作用仍待定位，不盲试已排除的PathDist/GoalDist组合或移除alignment。

最后lab1_fivepoint_final_zero_audit.json：boot同上、13份地图/route/源码/安装配置SHA不变、6秒121条/controller/cmd_vel全零、唯一cmd_vel_guard、实际导航/定位节点空。navigation_active为null（没有发布者），不能写成收到了false。本轮无原GUI开启/归位、无initialpose或NavigateToPose/非零运动发送，没有候选部署。各隔离二进制退出码0仅表示程序正常结束，业务arrived真假以summary为准。

远端隔离输入/编译及各段原日志在/tmp/raicom_lab1_dwb_static_20260914，精确末端诊断在其exact_endpoint_diagnostic/子目录；本地完整基线inputs及SHA在lab1_after_battery_preflight/dwb_probe_inputs和manifest。成功脚本独占目录不要原样重跑。接手继续从已保存轨迹和关键停滞位置做critic诊断；定位没有被初始化，用户实体不在start，预检未通过前不要求搬回或发目标。保持原地图/五正式点、容差和原guard链路。


## 31. 固定路径 / 手动示教录制可行性（用户询问，尚未实施）

用户询问能否固定路径或录制一次后行驶，以减少反复规划与停滞。本次只核对仓库和官方Humble接口，没有开启实车服务/定位、录制或运动，没有决定替换现有架构。

结论：可在现有AMCL + Nav2控制器 + 原bridge/guard框架增加保存路径与示教轨迹模式。Humble FollowPath.action原生接受nav_msgs/Path、controller_id、goal_checker_id；官方ControllerServer在follow_path执行给定路径，依然进行局部闭环控制及到点判断。原场地面板目前只存五点/顺序并发送NavigateToPose，没有轨迹录制/FollowPath客户端。不能告诉用户已有可点录制按钮。Context7解析请求返回Monthly quota exceeded，改核对官方Humble原始源码：
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_msgs/action/FollowPath.action
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_controller/src/controller_server.cpp
- https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_simple_commander/nav2_simple_commander/robot_navigator.py

建议的产品行为：保持当前lab1及五点，在固定地图定位有效时手动示教，记录map坐标下实际位姿轨迹，按五站停车划分四段；保存与地图SHA/端点绑定的轨迹后，以固定全局路径和实时定位闭环跟踪，遇阻停车，不擅自绕行或跳段。轨迹记录用位置和朝向，不是只回放按键持续时间。每段终点必须对齐原保存点、保留原3cm/15°同时验收和停车保护，录制路径需检查抖动/跳变/车体和末端可行性。此为可行性建议，不是已经实现或已采集的新路径。

关键事实：第30节DWB软件检查已经使用一次保存的固定原生ThetaStar路径，只定期裁剪已走部分，没有重新做全局规划；仍前两段超差。仅把路径末端对齐原保存目标的固定路径诊断也在约10cm停滞。因此不得承诺“固定/录制路径必然消除当前停滞”。示教可能改善接近方向和转弯位置，末端控制和定位问题仍需验证。

若后续实施，必须同时接入原目标上下文、活跃状态、取消/急停和停车验收。当前field_tool_nav_bridge.py取消客户端仅指向/navigate_to_pose/_action/cancel_goal；直接改用FollowPath而不更新取消路径会留下未闭合的任务状态。优先保留原导航任务/保护入口，明确路径执行方式后再做最小改动，不绕过现有guard或原型直发底盘速度。本轮用户仍未将车放回start，真实运动未授权恢复。


## 32. 用户质疑现场无障碍：明确地图余量检查与失败原因

用户表示扫描/现场时未看到障碍，询问为何导航验证报冲突。本轮重新读取已冻结PGM、完整足迹和原回执，没有启动真实ROS导航/初值，也没有删格或改图。obstacle_explanation_evidence_v2.json记录两个具体样本（v1的示例yaw_deg沿用源值，v2仅修正该派生元数据，使用的yaw_rad和碰撞结果一致）。

五个保存点原姿态均静态无碰撞。pick_B在保存朝向基础上+13°的样本，未加padding车体无碰撞，原5cm padding与地图(row84,col93)、中心(-.47,.63)的黑格交叉；pick_A在保存朝向-15°样本同样只有padding交叉，格(row136,col131)、中心(.29,-.41)。两格为原PGM灰度0，不是由膨胀层生成的实体障碍；距离各点中心约22.05/21.94cm，分别连接113/121个四邻域occupied格，不能说是已经确认的独立噪点。地图分辨率2cm。

解释需准确：这属于“保存地图中的黑格与额外车体余量，在某些假设朝向发生几何交叉”，不是本轮实时激光确认现场有物体，也不是小车中心点落在障碍上。真实黑格是否对应当前实物、地图残影或误测，尚未完成现场激光对照，不下结论。用户认为现场空旷，不自动授权删除地图格子或放宽保护。

第30节当前基线失败的前两段，完整记录的轨迹采样无碰撞，失败是停车后相对保存点3.4221/3.9842cm超限；后两段软件通过。精确路径末端的诊断仍有约10cm停滞，但不能据此前原位转向余量冲突归因“现场有障碍导致停滞”。应先继续末端控制/critic根因定位；如排查地图疑似障碍，需在真实定位恢复后与新激光对照，不注入假start。此前用“障碍阻断”笼统合并两类问题的表述，以本节和第30节的具体证据为准。

## 33. 补齐后续导航检查与0cm余量落盘状态

用户后续明确要求先start→pick、将额外车体余量改为0cm，并曾确认实体在start；这些确认属于当时boot，不能延用为2026-09-15实体位置事实。此前漏写本索引的真实证据如下：

- `lab1_start_pick_native_plan_v2.ndjson`：隔离ThetaStar路径约1.54493m成功；原保存点间几乎直线，几何清空不能等同控制器完成。
- `lab1_start_pick_control_v1.ndjson`：原5cm padding基线下，原生路径与直接直线路径均在120模拟秒超时，最终约11.248cm/4.26°与11.581cm/0.118°；不是0cm新配置测试，也不是实车到达。
- 用户允许后，`lab1_padding_zero_disk_apply_v2.json`证实Pi source/install的`config/nav2_params_home_straight_low_load.yaml`两处`footprint_padding: 0.0`已写入，磁盘SHA均`8e3f778c184fe758e33c6a58f1fcf0c76b36c1397dbbfd783faa5334ab30fab4`；实际车体与inflation保持。当时Nav2停止，不能把磁盘落盘当作活动节点参数回读。最后61条底盘速度零，地图/点位不变。
- 本地修改文件仍为上述config，第159、207行；Pi备份`/home/ubuntu/raicom_tools/padding_zero_20260914`。第一次`lab1_padding_zero_apply_v1.ndjson`在修改前因Nav2无active状态失败，不算部署成功。
- 2026-09-15 `navigation_disk_readback.json`再次确认Pi source/install SHA及两处0.0，无活动Nav2参数验收。本轮没有启动导航或修改上述文件。

此前原GUI恢复窗口曾因robot_state_publisher缺失导致base FAIL，不能记成完成导航；本轮新boot进程清单中该进程已存在，原“RSP当前缺失”不再成立，但仅进程存在仍不足以证明TF健康。

## 34. 用户最新确定的任务流程（待实现/验收）

- 范围：以用户最后红框照片为比赛区域约束，不把手动区合并，不从透视照片猜测桌台对应点或修改地图坐标。
- 先导航后视觉抓取。采用地图上的出站/转弯中间点，pick到园区先倒退再转弯再前进；允许在保留关键步骤/范围约束下局部避障，倒退遇阻或无路则停车。固定路线不是已解决DWB停滞的证据。
- 点名：pick_A为园区1，pick_B为园区2。识别区同一画面的左标识对应园区1、右标识对应园区2，据本轮识别结果建立T/G去向。
- 共8件，4T+4G，顺序随机；按可达候选从左到右选择，每次运一件。前7次投放后回pick，第8次成功放置后停在最后园区。
- pick顺序最新修订：到点停车→Aurora RGB/深度找蓝色→锁定同一方块并让底盘对齐→停稳→调用原相机/YOLO API识别T/G→按位置/高度选抓取姿态→夹取。蓝色相对纠偏不发布AMCL初值。
- 用户稍后提供2个抓取姿态及4个投放姿态。抓取姿态按位置或高度选择，不按T/G；园区各自依次使用投放位置1..4。未收到标定值，不能编造舵机脉冲或宣布抓取成功。
- 只调用pi_cam_env/yolo_env已有API，不修改它们的代码、模型、配置或容器；任务适配留在ROS控制层。API真实标签与深度边界见第35节。

## 35. 2026-09-15 当前容器API实查及路线变化（最新）

证据根`runtime_evidence/20260915_vision_api_audit/`，接口细节见 [VISION_CONTAINER_API.md](VISION_CONTAINER_API.md)。本轮main/HEAD dbac0a6，保留所有既有脏文件；修改前CURRENT副本保存在该证据目录，没有commit/reset/clean。

实际SSH从旧.3失败后，按当前WLAN已知网段连接.149.1成功；boot ecdad943-b012-4b1e-9be2-8a384015b8dc。MentorPi、pi_cam_env、yolo_env运行，objective_dhawan仍退出。原基础栈/guard、robot_state_publisher进程存在，未启动GUI/定位/Nav2或Aurora驱动，未做新boot的运动验收。

相机进程`/app/camera_app.py`提供8000；YOLO进程`/app/infer_web.py`提供8080，YOLO共享相机容器网络和IPC。实际请求是**POST /api/do_infer，无请求体**，内部自动POST8000的`/api/capture_temp`，从共享内存读取普通相机JPEG后推理。没有外部图像上传/深度查询接口。普通相机框不能直接用作Aurora深度框。

当前CLASSES=["1","2","blue"]；模型索引0/1/2在HTTP中分别输出字符串"1"/"2"/"blue"。按用户此前给的0=T、1=G，控制层应映射"1"→T、"2"→G；本轮没有实物分类证据，不宣称已经验证该语义。蓝色类别只用于目标候选，不能替代T/G分类。

一次真实POST返回HTTP200、status=success、detections=[]、image_size=[648,486]，总约0.954秒，capture558/read36/preprocess31/inference319毫秒。仅证明抓拍推理链通畅，不能证明T/G、深度对齐或抓取成功。结果图已留存，当前画面偏暗且没有清楚的待识别方块。API不返回采集时间/请求ID，适配需处理串行调用及目标一致性。

容器代码/模型/参数SHA和容器ID/镜像ID/StartTime/RestartCount前后完全一致，见`unchanged_check.json`。原接口本身按既有行为更新一次临时照片及latest_result.jpg；没有改配置、重启容器或发送底盘/机械臂动作。

**导航资产新发现：** 本次Pi地图YAML SHA仍`51d5bf647afd279f45e23b4411443d05b8f91a66513f91575285978618f01f23`、PGM仍`51255ed214a4bbf80e13eb7041e9dfee2dd06be90a5509597bed9d8425b4903f`。但同名route SHA现为`23d0d729ba8eda3c23c6449d313c6bf30bcd74954245246facedd2ff4c47fd5f`，order=[pick_A]，points只有pick_A；history含start/shibie/pick/pick_B的delete记录，不能推断操作者或删除意图。本轮只读取得现状，不恢复或覆盖。

删除前完整五点精确副本在`runtime_evidence/20260914_lab_text_navigation/lab1_start_pick_current/20260914_101211_lab1.8a9ee38f24abf6e8.route.json`，SHA `8b3bb9f75eb47e64bad445573e8bd1ba7ebe656e75bb16cfaeafe93c2504bb38`。草稿和start_localization仍保留，不能把它们当成已在当前正式order中。继续固定五点任务前需核对删除意图，禁止以旧备份冒充当前路线已完备。
