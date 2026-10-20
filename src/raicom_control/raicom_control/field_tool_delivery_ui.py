"""Field UI integration for the four/eight-piece signal workflow."""
from copy import deepcopy
import json
import time
import uuid
import tkinter as tk
from tkinter import ttk

from .delivery_arm import PHASES
from .field_tool_delivery import SignalDelivery, geometry_identity, settings as delivery_settings
from .field_tool_workspace_model import save_settings, workspace_settings
from .station_mapping import StationMapping


class DeliveryWorkspaceMixin:
    def _vision(self):
        if getattr(self,'vision_client',None) is None:
            from .vision_ros_client import VisionClient
            self.vision_client=VisionClient(self.node,with_images=True)
        return self.vision_client

    def toggle_vision_preview(self):
        if self.vision_preview_active:
            self._stop_vision_preview('画面预览已停止')
            return
        if self._workspace_busy() or self.navigation_running or self.nav_start_pending:
            self.vision_preview_status.set('请等当前任务结束后再开始独立预览')
            return
        self._clear_station_mapping('画面预览仅供查看，本帧候选不自动用于导航')
        self.mapping_session=None
        self.vision_preview_active=True;self.vision_preview_next=0.
        self.vision_preview_future=None;self.vision_launch_next=0.
        self.vision_preview_task='visual-preview-'+uuid.uuid4().hex
        self.vision_preview_button.configure(text='停止画面预览')
        self.vision_preview_status.set('等待视觉节点 · 每次识别完成后间隔 1 秒刷新')
        if hasattr(self,'detection_preview'):self.detection_preview.clear('正在连接单目相机与识别节点')
        self.start_vision_nodes()

    def _stop_vision_preview(self,reason):
        if not getattr(self,'vision_preview_active',False):return
        self.vision_preview_active=False;self.vision_preview_future=None
        if getattr(self,'vision_client',None) is not None:
            try:self.vision_client.cancel()
            except Exception as exc:reason+='；视觉取消发送失败：'+str(exc)
        self.vision_preview_button.configure(text='开始画面预览')
        self.vision_preview_status.set(reason)
        self.detection_preview.clear(reason)

    def _vision_reply_display(self,reply):
        if not hasattr(self,'detection_preview'):return
        if reply.get('error') or reply.get('preview_error'):
            self.detection_preview.clear('本次画面失败：'+str(reply.get('error') or reply['preview_error']))
        elif not reply.get('preview'):
            self.detection_preview.clear('视觉节点尚未提供配对照片，请重启视觉节点')

    def _poll_vision_preview(self):
        if not hasattr(self,'detection_preview'):return
        client=getattr(self,'vision_client',None)
        if client is not None:
            packet=client.take_preview()
            if packet:self.detection_preview.show_packet(packet,'独立预览 · 未采用映射' if self.vision_preview_active else '识别区采样')
        self.detection_preview.tick()
        if not self.vision_preview_active:return
        if self._workspace_busy() or self.navigation_running or self.nav_start_pending:
            self._stop_vision_preview('已停止独立预览，优先执行当前任务');return
        try:
            if self.vision_preview_future is not None:
                if self.vision_preview_future.done():
                    reply=self.vision_preview_future.result();self._vision_reply_display(reply)
                    self.vision_preview_future=None;self.vision_preview_next=time.monotonic()+1.
                    self.vision_preview_status.set('画面预览中 · 仅识别，不导航、不夹取' if not reply.get('preview_error')
                                                   else '识别已返回，照片暂不可用 · 将自动重试')
            elif time.monotonic()>=self.vision_preview_next:
                if self._vision().ready():
                    self.vision_preview_future=self._vision().request(self.vision_preview_task)
                    self.vision_preview_status.set('正在拍照识别 · 等待新一帧')
                else:
                    if time.monotonic()>=getattr(self,'vision_launch_next',0.):
                        self.vision_launch_next=time.monotonic()+3.
                        self.start_vision_nodes()
                    self.vision_preview_status.set(self._vision().connection_status())
        except (ValueError,RuntimeError,OSError) as exc:
            # HTTP failures are recoverable and must not look like an operator
            # cancellation. Explicit stop/task handoff still cancels this loop.
            self.vision_preview_future=None;self.vision_preview_next=time.monotonic()+3.
            self.detection_preview.clear('本帧未获取：'+str(exc))
            self.vision_preview_status.set('相机暂不可用，3秒后自动重试：'+str(exc))

    def _mapping_progress(self,text):
        if hasattr(self,'mapping_status_var'):self.mapping_status_var.set(text)

    def _clear_station_mapping(self,reason):
        for key,var in getattr(self,'mapping_vars',{}).items():var.set(f'{key} → 待识别')
        self._mapping_progress(reason)
        if getattr(self,'mapping_pub',None) is not None:
            from std_msgs.msg import String
            try:self.mapping_pub.publish(String(data=json.dumps({'valid':False,'reason':reason,'at_unix':time.time()})))
            except Exception as exc:self._set_status('映射撤销发布失败：'+str(exc))

    def _publish_station_mapping(self,result):
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
        if self.mapping_pub is None:
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.mapping_pub=self.node.create_publisher(String,'/raicom/vision/park_mapping',qos)
        self.mapping_pub.publish(String(data=json.dumps(result,ensure_ascii=False)))

    def _observe_station_mapping(self,reply):
        self._vision_reply_display(reply)
        result=self.mapping_session.observe(reply,stopped=True)
        if result:
            for key,var in getattr(self,'mapping_vars',{}).items():
                park=result['category_to_park'][key];number=1 if park=='A' else 2
                var.set(f"{key} → 园区{number}（{park}） · {result['category_to_point'][key]}")
            self._mapping_progress('连续 3 次有效一致 · 本轮映射已采用')
            self._publish_station_mapping(result)
            self._workspace_journal({'kind':'station_mapping_adopted',**result})
        else:
            count=self.mapping_session.consensus.count
            self._mapping_progress(f'有效一致 {count}/3 · '+str(reply.get('error','等待左右各一个 T/G 标识')))
        return result

    def _assert_at_recognition(self,station):
        from .field_tool_navigation_logic import NavigationTarget,arrival_errors,POSITION_TOLERANCE_M,YAW_TOLERANCE_DEG
        if station not in self.route_document['points']:raise ValueError('请先选择已保存的识别区点')
        if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5) or not self.node.zero_velocity_hold_ok():
            raise RuntimeError('识别需要定位有效且底盘停稳')
        p=self.route_document['points'][station]['approach']
        target=NavigationTarget(station,station,p['x'],p['y'],p['yaw_rad'],POSITION_TOLERANCE_M,YAW_TOLERANCE_DEG)
        distance,yaw=arrival_errors(self.node.capture_current_pose(),target)
        if distance>POSITION_TOLERANCE_M or yaw>YAW_TOLERANCE_DEG:raise RuntimeError('请先到识别区并通过 2 cm / 10° 停车验收')

    def start_vision_nodes(self):
        # May also start while stopped in the recognition wait stage. This only
        # starts HTTP adapters; their idle health probes never capture an image.
        try:
            names=list(self.node.get_node_names())
            if any(names.count(name)>1 for name in ('yolo_camera','yolo_infer')):
                raise RuntimeError('存在重复视觉节点，请结束重复实例后重试')
            started=[]
            for name in ('yolo_camera','yolo_infer'):
                if name not in names:
                    self._launch(name,['ros2','run','raicom_control',name])
                    started.append(name)
            text=('正在启动 '+', '.join(started)+' · 等待 8000 / 8080' if started
                  else '相机与识别节点已连接 · 点击“开始画面预览”查看结果')
            self._mapping_progress(text)
            if hasattr(self,'vision_preview_status'):self.vision_preview_status.set(text)
        except (ValueError,RuntimeError,OSError) as exc:
            text='视觉节点未启动：'+str(exc)
            self._mapping_progress(text)
            if hasattr(self,'vision_preview_status'):self.vision_preview_status.set(text)

    def recognize_station_now(self):
        if not self._can_edit_route():return
        try:
            config=self._delivery_form();self._assert_at_recognition(config['recognition'])
            self._stop_vision_preview('开始识别区采样')
            identity=self._workspace_snapshot()
            destinations={k:v.get() for k,v in self.destination_vars.items()}
            if any(v not in self.route_document['points'] for v in destinations.values()):raise ValueError('先选择 A/B 两个正式园区点')
            self.mapping_session=StationMapping('station-preview-'+uuid.uuid4().hex,geometry_identity(self.route_document),config['recognition'],destinations)
            self._clear_station_mapping('开始本次识别 · 连续 3 帧一致后显示映射')
            self._lock_manual_for_task()
            self.manual_mapping={'identity':identity,'deadline':time.monotonic()+60.,'future':None}
        except (ValueError,RuntimeError,OSError) as exc:self._mapping_progress(str(exc))

    def _poll_manual_mapping(self):
        work=getattr(self,'manual_mapping',None)
        if not work:return
        try:
            if time.monotonic()>work['deadline']:raise RuntimeError('识别映射超时，未采用结果')
            if self._workspace_snapshot()!=work['identity']:raise RuntimeError('识别期间地图 / 点位设置已变化')
            self._assert_at_recognition(self.mapping_session.station)
            if work['future'] is None:
                if self._vision().ready():work['future']=self._vision().request(self.mapping_session.task_id)
                else:self._mapping_progress('等待唯一 yolo_infer 节点；不会发导航或夹取')
            elif work['future'].done():
                try:reply=work['future'].result()
                except Exception as exc:reply={'payload':{},'error':str(exc)}
                work['future']=None
                self._workspace_journal({'kind':'station_recognition_frame',**reply})
                if self._observe_station_mapping(reply):self.manual_mapping=None
        except (ValueError,RuntimeError,OSError) as exc:
            self.manual_mapping=None
            try:self._vision().cancel()
            finally:self._clear_station_mapping(str(exc))

    def _delivery_init(self):
        self.delivery = None
        self.delivery_io = None
        self.delivery_point_combos = []
        self.delivery_capture = None
        self.delivery_nav = None
        self.delivery_program = None
        self.delivery_arm_started = False
        self.delivery_internal_dispatch = False
        self.delivery_future = None
        self.delivery_pool = None
        self.vision_client = None
        self.mapping_session = None
        self.manual_mapping = None
        self.mapping_pub = None
        self.mapping_status_var = tk.StringVar(value='尚未识别 · 本轮映射为空')
        self.mapping_vars = {k:tk.StringVar(value=f'{k} → 待识别') for k in ('T','G')}
        self.vision_preview_active=False;self.vision_preview_future=None;self.vision_preview_next=0.
        self.vision_preview_status=tk.StringVar(value='画面预览可独立使用；正式映射需在识别区停车后确认')
        self.delivery_identity = None
        self.delivery_stage_last = None
        self.delivery_status_at = 0.
        self.delivery_var = tk.StringVar(value='试点 4 件 · left_a × 2 → right_a × 2')
        self.delivery_expanded_var = tk.BooleanVar(value=False)
        self.delivery_start_var = tk.StringVar(value='start')
        self.delivery_recognition_var = tk.StringVar(value='shibie')
        self.delivery_request_var = tk.StringVar(value='/raicom/pick/request')
        self.delivery_result_var = tk.StringVar(value='/raicom/pick/result')
        self.delivery_timeout_var = tk.StringVar(value='120')
        self.delivery_pose_slot_var = tk.StringVar(value='运输姿态')
        self.delivery_pose_phase_var = tk.StringVar(value='接近')
        self.delivery_clearance_var = tk.BooleanVar(value=False)
        self.delivery_calibration_var = tk.StringVar(value='尚无有效运输 / 放置标定')

    def _delivery_busy(self):
        task = getattr(self, 'delivery', None)
        return bool(getattr(self, 'manual_mapping', None) or getattr(self, 'delivery_capture', None) or
                    (task and task.stage not in task.TERMINAL
                     and not getattr(self, 'delivery_internal_dispatch', False)))

    def _build_camera_tab(self):
        page = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(page, text='普通相机')
        content = self._scrolling_content(page)
        card = self._card(content, '普通相机 · YOLO 识别预览')
        card.pack(fill='x')
        ttk.Label(card, text='左侧标识 = 园区1（A） · 右侧标识 = 园区2（B）', style='Card.TLabel', wraplength=450).pack(anchor='w')
        from .vision_preview import DetectionPreview
        self.detection_preview = DetectionPreview(card)
        self.detection_preview.pack(fill='x', pady=(7, 5))
        self.vision_preview_button = ttk.Button(card, text='开始画面预览', command=self.toggle_vision_preview)
        self.vision_preview_button.pack(fill='x', pady=3)
        self.vision_preview_status.set('查看检测框、类别与本帧园区候选；不触发导航或夹取。')
        ttk.Label(card, textvariable=self.vision_preview_status, style='Card.TLabel', wraplength=450).pack(anchor='w', pady=5)
        ttk.Button(card, text='连接 / 检查相机节点', command=self.start_vision_nodes).pack(fill='x')
        ttk.Label(card, text='开始预览会自动连接相机。ROS2节点：/yolo_camera、/yolo_infer。',
                  style='Card.TLabel', wraplength=450).pack(anchor='w', pady=5)
        self._bind_workspace_scroll(content)

    def _delivery_refresh(self):
        if not hasattr(self, 'delivery_start_var'): return
        session=getattr(self,'mapping_session',None)
        if session and (not self.route_document or session.geometry!=geometry_identity(self.route_document) or
                        session.destinations!={k:v.get() for k,v in self.destination_vars.items()} or
                        session.station!=delivery_settings(self.route_document)['recognition']):
            self.mapping_session=None
            self._clear_station_mapping('地图 / 目的地已变化，请重新识别')
        names = self.route_document['order'] if self.route_document else []
        for combo in self.delivery_point_combos: combo.configure(values=names)
        config = delivery_settings(self.route_document or {})
        for var, key in [(self.delivery_start_var, 'start'), (self.delivery_recognition_var, 'recognition'),
                         (self.delivery_request_var, 'request_topic'), (self.delivery_result_var, 'result_topic'),
                         (self.delivery_timeout_var, 'timeout_s'), (self.delivery_expanded_var, 'expanded')]:
            var.set(config[key])
        c = config['calibration']
        self.delivery_clearance_var.set(c.get('transport_envelope_verified', False))
        count = sum(len(v) for v in c.get('placements', {}).values())
        current = bool(self.route_document and c.get('geometry_identity') == geometry_identity(self.route_document))
        self.delivery_calibration_var.set(f"运输 {'已记录' if c.get('transport') else '未记录'} · 放置 {count}/16 步 · " + ('坐标匹配' if current else '待标定 / 坐标已变'))

    def _delivery_form(self):
        config = delivery_settings(self.route_document)
        config.update(start=self.delivery_start_var.get(), recognition=self.delivery_recognition_var.get(),
                      expanded=self.delivery_expanded_var.get(), request_topic=self.delivery_request_var.get(),
                      result_topic=self.delivery_result_var.get(), timeout_s=float(self.delivery_timeout_var.get()))
        candidate = deepcopy(self.route_document)
        candidate.setdefault('workspace', {})['delivery'] = config
        return delivery_settings(candidate)

    def show_delivery_setup(self):
        if getattr(self, 'delivery_setup_window', None) is not None and self.delivery_setup_window.winfo_exists():
            self.delivery_setup_window.lift(); return
        window = self.delivery_setup_window = tk.Toplevel(self.root)
        window.title('夹取接口与运输 / 放置标定'); window.geometry('540x500')
        frame = ttk.Frame(window, padding=14); frame.pack(fill='both', expand=True)
        for title, var in [('PICK_READY 请求话题', self.delivery_request_var), ('A/B 返回话题', self.delivery_result_var),
                           ('夹取超时（秒）', self.delivery_timeout_var)]:
            ttk.Label(frame, text=title).pack(anchor='w')
            ttk.Entry(frame, textvariable=var).pack(fill='x', pady=(0, 5))
        ttk.Label(frame, text='同学收到取消后停止；A/B 仅在当前等待阶段消费一次。', wraplength=490).pack(anchor='w', pady=4)
        ttk.Separator(frame).pack(fill='x', pady=7)
        row = ttk.Frame(frame); row.pack(fill='x')
        ttk.Combobox(row, textvariable=self.delivery_pose_slot_var, state='readonly',
                     values=['运输姿态', '放置 1', '放置 2', '放置 3', '放置 4'], width=14).pack(side='left')
        ttk.Combobox(row, textvariable=self.delivery_pose_phase_var, state='readonly',
                     values=['接近', '落位', '松爪', '撤回'], width=14).pack(side='left', padx=8)
        ttk.Button(frame, text='读取当前物理姿态并记录到所选步骤', command=self.capture_delivery_pose).pack(fill='x', pady=7)
        ttk.Label(frame, textvariable=self.delivery_calibration_var, wraplength=490).pack(anchor='w')
        ttk.Checkbutton(frame, text='已在现场核验运输姿态及全车净空', variable=self.delivery_clearance_var).pack(anchor='w', pady=8)
        ttk.Label(frame, text='两园区复用四套动作；落点每次人工确认后才计数。记录只读当前姿态，不移动机械臂。', wraplength=490).pack(anchor='w')
        ttk.Button(frame, text='保存联动设置', style='Accent.TButton', command=self.save_delivery_setup).pack(fill='x', pady=10)

    def save_delivery_setup(self):
        if not self._can_edit_route(): return
        try:
            config = self._delivery_form()
            c = config['calibration']
            if self.delivery_clearance_var.get() and (not c.get('transport') or c.get('geometry_identity') != geometry_identity(self.route_document)):
                raise ValueError('先按当前点位记录运输姿态，再核验净空')
            c['transport_envelope_verified'] = bool(self.delivery_clearance_var.get())
            value = workspace_settings(self.route_document); value['delivery'] = config
            self._workspace_write(save_settings(self.route_document, value))
            self._set_status('联动设置已保存')
        except (ValueError, RuntimeError, OSError) as exc: self._set_status(str(exc))

    def capture_delivery_pose(self):
        if not self._can_edit_route(): return
        try:
            identity = self._workspace_snapshot()
            slot = self.delivery_pose_slot_var.get()
            phase = dict(zip(('接近', '落位', '松爪', '撤回'), PHASES))[self.delivery_pose_phase_var.get()]
            self._lock_manual_for_task(); self._arm().start('capture')
            self.delivery_capture = {'identity': identity, 'slot': slot, 'phase': phase}
        except (ValueError, RuntimeError, OSError, KeyError) as exc: self._set_status(str(exc))

    def start_signal_delivery(self):
        if not self._can_edit_route() or self.node.navigation.cleanup_pending: return
        self._stop_vision_preview('联动任务将自动更新识别画面')
        try:
            config = self._delivery_form()
            task = SignalDelivery(self.route_document, config, {k: v.get() for k, v in self.destination_vars.items()})
            identity = self._workspace_snapshot()
            if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5) or not self.node.zero_velocity_hold_ok():
                raise RuntimeError('开始联动前需要定位有效且车已停稳')
            if self.delivery_io is not None: self.delivery_io.close(); self.delivery_io = None
            from .field_tool_pick_io import PickSignalIO
            from .recognition_station import MarkerConsensus
            self.delivery_io = PickSignalIO(self.node, config['request_topic'], config['result_topic'])
            self.delivery_consensus = MarkerConsensus()
            self.delivery, self.delivery_identity = task, identity
            self.delivery_nav = self.delivery_program = self.delivery_future = None
            self.delivery_arm_started = False
            self.delivery_stage_last = None
            self.mapping_session = None
            self._clear_station_mapping('新任务 · 到识别区后重新建立映射')
            self._lock_manual_for_task()
            self._workspace_journal({'kind': 'delivery_started', **task.status()})
        except (ValueError, RuntimeError, OSError) as exc:
            if self.delivery and self.delivery.stage not in self.delivery.TERMINAL: self._cancel_delivery(str(exc))
            self._set_status(str(exc))

    def _cancel_delivery(self, reason):
        self._stop_vision_preview(reason)
        if hasattr(self,'detection_preview'):self.detection_preview.clear(reason)
        capture = getattr(self, 'delivery_capture', None)
        self.delivery_capture = None
        self.manual_mapping = None
        self.mapping_session = None
        if getattr(self,'vision_client',None) is not None:
            try:self.vision_client.cancel()
            except Exception as exc:self._set_status('视觉取消请求异常：'+str(exc))
        self._clear_station_mapping(reason)
        task = getattr(self, 'delivery', None)
        if task and task.stage not in task.TERMINAL:
            task.cancel(reason)
            self.delivery_nav = self.delivery_program = None
            if self.delivery_io:
                try:
                    self.delivery_io.send('CANCEL')
                    self.delivery_io.publish_status(task.status())
                except Exception as exc:
                    # Do not interrupt the caller's independent arm and chassis
                    # stop, or leave another delayed UI task alive.
                    self._set_status(reason + '；同学端取消信号未送达：' + str(exc))
        if getattr(self, 'delivery_pool', None):
            self.delivery_pool.shutdown(wait=False, cancel_futures=True)
            self.delivery_pool = None
        self.delivery_future = None  # Late HTTP results cannot affect another task.
        if hasattr(self, 'delivery_confirm_button'): self.delivery_confirm_button.configure(state='disabled')
        return capture

    def confirm_delivery_placement(self):
        task = getattr(self, 'delivery', None)
        if not task: return
        try:
            if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5) or not self.node.zero_velocity_hold_ok():
                raise RuntimeError('确认落点时必须定位有效且车已停稳')
            if self._workspace_snapshot() != self.delivery_identity: raise RuntimeError('任务期间点位已变化')
            if task.confirm_placement(time.monotonic(), True):
                self._workspace_journal({'kind': 'delivery_placement_confirmed', **task.status(),
                                         'result': task.placements[-1]})
            self.delivery_confirm_button.configure(state='disabled')
        except (ValueError, RuntimeError, OSError) as exc: self._fail_delivery(str(exc))

    def _fail_delivery(self, reason):
        task = self.delivery
        task.fail(reason)  # Invalidate before any fallible publisher.
        self.delivery_var.set(reason)
        try:
            if self.delivery_io:
                self.delivery_io.send('CANCEL'); self.delivery_io.publish_status(task.status())
        except Exception as exc:
            self._set_status(reason + '；取消信号发布异常：' + str(exc))
        finally:
            self.stop_all_motion()  # Cancels existing arm I/O and drains Nav2.

    def _poll_delivery(self):
        if not hasattr(self, 'delivery'): return
        task = self.delivery
        try:
            if getattr(self,'vision_client',None) is not None:self.vision_client.poll()
            self._poll_vision_preview()
            self._poll_manual_mapping()
            if self.delivery_capture:
                arm = self._arm()
                if arm.state == 'failed': raise RuntimeError(arm.error)
                if arm.state == 'complete':
                    capture = self.delivery_capture
                    if self._workspace_snapshot() != capture['identity']: raise RuntimeError('记录期间点位已改变')
                    config = self._delivery_form(); c = config['calibration']
                    geometry = geometry_identity(self.route_document)
                    if c.get('geometry_identity') != geometry: c = {'geometry_identity': geometry, 'placements': {}}
                    pose = arm.captured_pose(self.session_dir / 'workspace_events.ndjson')
                    if capture['slot'] == '运输姿态': c['transport'] = pose; c['transport_envelope_verified'] = False
                    else: c.setdefault('placements', {}).setdefault(capture['slot'][-1], {})[capture['phase']] = pose
                    config['calibration'] = c
                    value = workspace_settings(self.route_document); value['delivery'] = config
                    self.delivery_capture = None
                    self._workspace_write(save_settings(self.route_document, value))
            if not task or task.stage in task.TERMINAL:
                if self.delivery_io: self.delivery_io.drain()
                if task and hasattr(self, 'delivery_var'):
                    if task.stage == 'complete': self.delivery_var.set(f'全部完成 · {task.completed} 件 · 已停在最后园区')
                    elif task.reason: self.delivery_var.set(task.reason)
                return
            now = time.monotonic(); task.tick(now)
            if task.stage == 'failed': raise RuntimeError(task.reason)
            if self._workspace_snapshot() != self.delivery_identity: raise RuntimeError('联动期间路线已变化')
            if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5): raise RuntimeError('联动期间定位 / 雷达失效')
            if task.stage != 'navigation' and not self.node.zero_velocity_hold_ok(): raise RuntimeError('停车任务期间车体移动')
            for signal, received_at in self.delivery_io.drain():
                accepted = task.receive(signal, received_at, now)
                self._workspace_journal({'kind': 'pick_signal', 'signal': signal, 'accepted': accepted, **task.status()})
            if task.stage == 'failed': raise RuntimeError(task.reason)
            if task.stage == 'navigation':
                if self.delivery_nav is None:
                    if self.navigation_running: return
                    previous = self.node.navigation.mission_id
                    self.delivery_internal_dispatch = True
                    try: self.start_navigation(target_order=[task.target])
                    finally: self.delivery_internal_dispatch = False
                    if not self.nav_start_pending: raise RuntimeError('导航未启动：' + self.status_var.get())
                    self.node.navigation.point_tasks = {}  # Task owns arm control; no per-waypoint replay.
                    self.delivery_nav = {'previous': previous, 'mission': None}
                elif not self.nav_start_pending:
                    nav = self.node.navigation
                    if self.delivery_nav['mission'] is None:
                        if nav.mission_id == self.delivery_nav['previous']: raise RuntimeError('导航请求未建立新任务')
                        self.delivery_nav['mission'] = nav.mission_id
                    if nav.mission_id != self.delivery_nav['mission']: raise RuntimeError('导航任务身份发生变化')
                    if nav.state == nav.SUCCEEDED:
                        task.arrived(nav.arrivals[-1], now); self.delivery_nav = None
                    elif nav.state in (nav.FAILED, nav.STOPPED): raise RuntimeError(nav.last_error)
            elif task.stage == 'recognition':
                self._assert_at_recognition(task.config['recognition'])
                if self.mapping_session is None:
                    self.mapping_session=StationMapping(task.task_id,geometry_identity(self.route_document),task.config['recognition'],task.destinations)
                if self.delivery_future is None:
                    if self._vision().ready():self.delivery_future=self._vision().request(task.task_id)
                    else:self._mapping_progress('等待 yolo_infer ROS2 节点；请先启动视觉节点')
                elif self.delivery_future.done():
                    try: reply = self.delivery_future.result()
                    except Exception as exc: reply = {'payload': {}, 'error': str(exc)}
                    self.delivery_future = None
                    self._workspace_journal({'kind': 'delivery_recognition', **reply})
                    mapping=self._observe_station_mapping(reply)
                    if mapping:task.recognized(mapping['label_to_park'])
            elif task.stage == 'pick_ready':
                if self.delivery_io.request.get_subscription_count() != 1:
                    raise RuntimeError('夹取请求需要唯一接收节点')
                if self.arm_io is not None: self.arm_io.release_control()
                self.delivery_io.drain()
                self.delivery_io.send(task.pick_ready(now))
                self._workspace_journal({'kind': 'pick_ready_sent', **task.status()})
            elif task.stage == 'transport_pose':
                if not self.delivery_arm_started:
                    self._arm().start('pose', task.config['calibration']['transport']); self.delivery_arm_started = True
                elif self._arm().state == 'failed': raise RuntimeError(self._arm().error)
                elif self._arm().state == 'complete':
                    self.delivery_arm_started = False; task.transport_ready(True)
            elif task.stage == 'transport_arrived':
                program = task.begin_placement(now)
                self.delivery_program = [(phase, program[phase]) for phase in PHASES]
                self.delivery_arm_started = False
            elif task.stage == 'placing':
                if not self.delivery_arm_started:
                    phase, pose = self.delivery_program[0]
                    self._arm().start('pose', pose); self.delivery_arm_started = True
                    self._workspace_journal({'kind': 'delivery_placement_phase', 'phase': phase, **task.status()})
                elif self._arm().state == 'failed': raise RuntimeError(self._arm().error)
                elif self._arm().state == 'complete':
                    self.delivery_program.pop(0); self.delivery_arm_started = False
                    if not self.delivery_program: task.placement_readback(now)
            labels = {'navigation': '导航', 'recognition': '识别序列 1/2', 'pick_ready': '到达夹取点',
                      'waiting_ab': '等待同学夹取并返回 A/B', 'transport_pose': '运输姿态读回',
                      'transport_arrived': '已到园区', 'placing': '执行放置动作', 'confirm_placement': '等待实物落点确认',
                      'complete': '全部完成', 'failed': '失败', 'cancelled': '已取消'}
            self.delivery_var.set(f"{labels[task.stage]} · {task.completed}/{len(task.schedule)} 件 · A {task.slots['A']}/4 · B {task.slots['B']}/4")
            if now - self.delivery_status_at > .5 or self.delivery_stage_last != task.stage:
                self.delivery_io.publish_status(task.status()); self.delivery_status_at = now
            if self.delivery_stage_last != task.stage:
                self._workspace_journal({'kind': 'delivery_stage', **task.status()}); self.delivery_stage_last = task.stage
            self.delivery_confirm_button.configure(state='normal' if task.stage == 'confirm_placement' else 'disabled')
        except (ValueError, RuntimeError, OSError) as exc:
            if task and task.stage not in ('complete', 'cancelled'): self._fail_delivery(str(exc))
            else:
                self.delivery_capture = None
                self.manual_mapping = None
                self._clear_station_mapping(str(exc))
                if self.arm_io: self.arm_io.cancel(str(exc))
                self._set_status(str(exc))
