"""One-click frontend for the frozen original RGB-D grasp launch."""
from io import BytesIO
import time
import sys
import tkinter as tk
from tkinter import ttk


class DepthWorkspaceMixin:
    def _depth_init(self):
        self.depth_client = None
        self.depth_status_var = tk.StringVar(value='正在准备抓取接口')
        self.depth_image_var = tk.StringVar(value='尚无画面 · 接口连接后显示后端标注图')
        self.depth_result_var = tk.StringVar(value='本工作台尚未发起抓取')
        self.depth_result_id = None
        self.depth_original = self.depth_photo = None
        self.depth_image_at = None
        self.depth_start_wait = None
        self.depth_stage_history = []
        self.depth_last_stage = None
        self.depth_process_var = tk.StringVar(value='相机和抓取服务提前准备 · 点击后执行本次动作')
        self.depth_preview_var = tk.BooleanVar(value=False)
        self.depth_motion_var = tk.StringVar(value='使用小车当前抓取程序 · 自动执行并报告结果')

    def _build_depth_tab(self):
        page = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(page, text='深度抓取')
        # Reserve the controls first so a tall preview never hides cancel/start.
        state = self._card(page, '抓取状态')
        state.pack(side='bottom', fill='x', pady=(8, 0))
        self.depth_camera_card = self._card(page, '深度相机画面')
        self.depth_camera_card.pack(side='top', fill='x')
        ttk.Checkbutton(self.depth_camera_card, text='打开独立相机画面（不影响抓取）', variable=self.depth_preview_var,
                        command=self._toggle_depth_preview).pack(anchor='w')
        self.depth_camera_window = tk.Toplevel(self.root)
        self.depth_camera_window.withdraw()
        self.depth_camera_window.title('RAICOM · 深度相机画面')
        self.depth_camera_window.geometry('640x480')
        self.depth_camera_window.protocol('WM_DELETE_WINDOW', self._close_depth_preview)
        self.depth_preview_frame = ttk.Frame(self.depth_camera_window, padding=8, style='Card.TFrame')
        self.depth_preview_frame.pack(fill='both', expand=True)
        self.depth_canvas = tk.Canvas(self.depth_preview_frame, height=360, width=600, background='#142633', highlightthickness=0)
        self.depth_canvas.pack(fill='both', expand=True, pady=(6, 0))
        self.depth_canvas.bind('<Configure>', lambda _: self._draw_depth_image())
        ttk.Label(self.depth_preview_frame, textvariable=self.depth_image_var, style='Muted.TLabel', wraplength=450).pack(anchor='w', pady=6)
        self.depth_log_card = self._card(page, '抓取过程')
        self.depth_log_card.pack(fill='both', expand=True, pady=(8, 0))
        ttk.Label(self.depth_log_card, textvariable=self.depth_motion_var, style='Muted.TLabel',
                  wraplength=450).pack(fill='x', pady=(0, 6))
        log = ttk.Frame(self.depth_log_card, style='Card.TFrame')
        log.pack(fill='both', expand=True)
        scroll = ttk.Scrollbar(log, orient='vertical')
        scroll.pack(side='right', fill='y')
        self.depth_log_text = tk.Text(log, height=5, wrap='word', state='disabled',
                                     background='white', foreground='#203747', relief='flat',
                                     font=('TkDefaultFont', 10), yscrollcommand=scroll.set)
        self.depth_log_text.pack(side='left', fill='both', expand=True)
        scroll.configure(command=self.depth_log_text.yview)
        self.depth_process_var.trace_add('write', self._render_depth_log)
        self._render_depth_log()
        ttk.Label(state, textvariable=self.depth_status_var, style='Card.TLabel', wraplength=450).pack(fill='x', pady=4)
        ttk.Label(state, textvariable=self.depth_result_var, style='Muted.TLabel', wraplength=450).pack(fill='x')
        row = ttk.Frame(state, style='Card.TFrame')
        row.pack(fill='x', pady=4)
        self.depth_start_button = ttk.Button(row, text='抓取前方蓝色方块', command=self.start_depth_pick,
                                             style='Accent.TButton')
        self.depth_start_button.pack(side='left', fill='x', expand=True, padx=(0, 4))
        self.depth_stop_button = ttk.Button(row, text='停止本次抓取', command=self.stop_depth_pick,
                                            style='Stop.TButton', state='disabled')
        self.depth_stop_button.pack(side='left', fill='x', expand=True)
        self.depth_empty_button = ttk.Button(state, text='物体已取走，夹爪已清空',
                                             command=self.confirm_depth_empty, state='disabled')
        self.depth_empty_button.pack(fill='x', pady=3)
        ttk.Label(state, text='点击即执行：停车、空爪、动作范围净空，并保持监看。\n自动返回成功或失败；相机画面与过程日志独立显示。',
                  style='Muted.TLabel', wraplength=450).pack(anchor='w', pady=(6, 0))
        self._draw_depth_image()
        # Prewarm perception and the idle native host without issuing arm commands.
        self.root.after(300, self.connect_depth_interface)

    def connect_depth_interface(self):
        if getattr(self, '_closed', False):
            return
        try:
            if self.depth_client is None:
                from .depth_pick_interface import DepthPickClient
                self.depth_client = DepthPickClient(self.node)
                self.depth_client.session.message = '接口已连接 · 等待抓取后端状态'
            if 'raicom_depth_pick_backend' not in self.node.get_node_names():
                self._launch('depth_pick_backend', [sys.executable, '-B', '-m',
                                                   'raicom_control.depth_warm_runner'])
            self._poll_depth()
        except (RuntimeError, ValueError, OSError) as exc:
            self.depth_status_var.set('接口连接失败：' + str(exc))

    def _depth_busy(self):
        client = getattr(self, 'depth_client', None)
        return bool(getattr(self, 'depth_start_wait', None) or (client and client.session.busy(time.monotonic())))

    def _depth_other_work(self):
        return bool(self.navigation_running or self.nav_start_pending or self.mapping_running
                    or getattr(self, 'mapping_start_pending', False) or getattr(self, 'save_pending', False)
                    or getattr(self, 'save_process', None) is not None or self.node.navigation.cleanup_pending
                    or self._workspace_busy(include_depth=False))

    def start_depth_pick(self):
        try:
            if self._depth_busy():
                raise RuntimeError('已有抓取或启动请求；请等待或停止本次抓取')
            if self._depth_other_work() or not self.node.zero_velocity_hold_ok():
                raise RuntimeError('请先停车并结束当前建图、导航或机械臂任务')
            if self.node.external_stop_requested:
                raise RuntimeError('刚收到停车请求，本次未启动；停稳后重新点击')
            if self.depth_client is None:
                self.connect_depth_interface()
            if self.depth_client is None:
                raise RuntimeError('抓取接口创建失败，请重试')
            self.depth_client.poll()
            session = self.depth_client.session
            status = session.status or {}
            if session.busy(time.monotonic()):
                raise RuntimeError('机械臂仍被占用，请等待停止回执')
            if status.get('state') in {'succeeded', 'failed', 'cancelled'}:
                if not self._depth_can_retry():
                    raise RuntimeError('先取走物体并确认夹爪已清空，再开始下一次')
                # This button explicitly says "空爪，重新抓取". Reset is tied
                # to that click and the last matching result, never automatic.
                self.depth_client.confirm_empty()
            self._lock_manual_for_task()
            self.depth_start_wait = {'deadline': time.monotonic() + 20,
                                     'backend_id': status.get('backend_id'), 'retry_at': 0.0}
            self.depth_stage_history = []
            self.depth_last_stage = None
            self.depth_motion_var.set('本次正在准备 · 等待新的保持姿态和夹爪读回')
            self.depth_process_var.set('正在连接抓取程序，停稳后自动开始…')
            self._poll_depth()
        except (RuntimeError, ValueError, OSError) as exc:
            self.depth_status_var.set(str(exc))
            self._set_status(str(exc))

    def stop_depth_pick(self):
        if getattr(self, 'depth_start_wait', None):
            self.depth_start_wait = None
            self.depth_process_var.set('已取消启动 · 未发出抓取动作')
        client = getattr(self, 'depth_client', None)
        if client is not None and client.session.pending:
            try:
                client.cancel()
                self.depth_status_var.set(client.session.message)
            except (RuntimeError, OSError) as exc:
                self.depth_status_var.set('取消发送失败，停止状态未确认：' + str(exc))

    def confirm_depth_holding(self):
        self.depth_status_var.set('已取消人工夹稳确认，请等待自动抓取结果')

    def _render_depth_log(self, *_):
        self.depth_log_text.configure(state='normal')
        self.depth_log_text.delete('1.0', 'end')
        self.depth_log_text.insert('end', self.depth_process_var.get())
        self.depth_log_text.configure(state='disabled')
        self.depth_log_text.see('end')

    def confirm_depth_empty(self):
        try:
            if not self._depth_can_reset():
                raise RuntimeError('当前结果未确认停止或已经过期；等待本次新回执')
            self.depth_client.confirm_empty()
        except (RuntimeError, OSError) as exc:
            self.depth_status_var.set(str(exc))

    def _depth_can_reset(self):
        session = self.depth_client.session
        status, result = session.status or {}, session.completed or {}
        return bool(not session.pending and not self.depth_start_wait and result
                    and session.fresh(time.monotonic()) and status.get('arm_released') is True
                    and status.get('state') in {'succeeded', 'failed', 'cancelled'}
                    and all(status.get(k) == result.get(k) for k in ('backend_id', 'client_id', 'request_id')))

    def _depth_can_retry(self):
        status = self.depth_client.session.status or {}
        return bool(self._depth_can_reset() and status.get('state') in {'failed', 'cancelled'}
                    and status.get('holding') is False)

    def _advance_depth_start(self):
        waiting = self.depth_start_wait
        if not waiting:
            return
        session = self.depth_client.session
        now = time.monotonic()
        if (self.node.external_stop_requested or self._depth_other_work()
                or not self.node.zero_velocity_hold_ok()):
            self.depth_start_wait = None
            raise RuntimeError('启动已取消：收到停车请求、底盘移动或其他任务占用')
        if now >= waiting['deadline']:
            self.depth_start_wait = None
            raise RuntimeError('连接抓取程序超时；本次未发动作，可重新点击连接')
        if session.fresh(now):
            backend = session.status['backend_id']
            if waiting['backend_id'] and waiting['backend_id'] != backend:
                self.depth_start_wait = None
                raise RuntimeError('抓取程序已换实例，本次未发动作；核对空爪后重新点击')
            waiting['backend_id'] = backend
            if session.busy(now):
                self.depth_start_wait = None
                raise RuntimeError('抓取程序被其他任务占用，本次未启动')
            if session.ready(now):
                self.depth_start_wait = None
                self.depth_client.start()
                self._workspace_journal({'kind': 'depth_pick_requested', **session.pending})
                return
        # Only restart a missing idle adapter, never interrupt an active owner.
        if now >= waiting['retry_at']:
            waiting['retry_at'] = now + 2
            if 'raicom_depth_pick_backend' not in self.node.get_node_names():
                self._launch('depth_pick_backend', [sys.executable, '-B', '-m',
                                                  'raicom_control.depth_warm_runner'])

    def _depth_display_status(self):
        session = self.depth_client.session
        status = session.status or {}
        if self.depth_start_wait:
            return '正在准备 · ' + str(status.get('reason') or '连接后自动开始抓取')
        if not session.fresh(time.monotonic()):
            return session.message
        if session.pending:
            return session.message
        state = status.get('state')
        if state in {'succeeded', 'failed', 'cancelled'}:
            label = {'succeeded': '抓取完成', 'failed': '本次失败', 'cancelled': '本次已取消'}[state]
            if status.get('arm_released') is not True:
                return label + ' · 等待机械臂停止与控制权交还'
            return label + ' · ' + str(status.get('reason') or '已夹持') + '；确认空爪后可再抓'
        if session.ready(time.monotonic()):
            return '抓取程序已连接 · 点击即可自动对准、夹取并收拢'
        return '等待抓取条件 · ' + str(status.get('reason') or '停稳、机械臂空闲和程序检查')

    def _poll_depth(self):
        client = getattr(self, 'depth_client', None)
        if client is None:
            return
        try:
            client.poll()
            self._advance_depth_start()
            self.depth_status_var.set(self._depth_display_status())
            status = client.session.status or {}
            motion = status.get('motion_result') or {}
            if not isinstance(motion, dict): motion = {}
            hold = motion.get('hold') or {}
            if not isinstance(hold, dict): hold = {}
            identity = client.session.pending or (client.session.completed if not self.depth_start_wait else None)
            same_request = bool(identity and all(status.get(k) == identity.get(k)
                                for k in ('backend_id', 'client_id', 'request_id')))
            values = hold.get('measured')
            tracking = motion.get('tracking') or {}
            if same_request and isinstance(tracking, dict) and type(tracking.get('model_height_drop_m')) in (int, float):
                self.depth_motion_var.set(f'本段关节读回：末端比计划低 {tracking["model_height_drop_m"] * 100:.1f} cm'
                    ' · 偏低超出补偿余量时停止前伸；不代表已夹稳')
            if same_request and hold.get('hold_pose_reached') is True and isinstance(values, dict):
                confirmation = '已确认夹稳' if status.get('state') == 'succeeded' and status.get('holding') is True else '待夹稳检查'
                self.depth_motion_var.set('保持姿态实际读回：' + ' / '.join(str(values.get(str(i),'—')) for i in range(1,6))
                    + f'；夹爪读回 {hold.get("gripper_readback", "—")}（{confirmation}）')
            if same_request:
                stage = (status.get('request_id'), status.get('state'), status.get('stage'), status.get('reason'))
                if stage != self.depth_last_stage:
                    self.depth_last_stage = stage
                    label = self._depth_display_status()
                    self.depth_stage_history.append(time.strftime('%H:%M:%S') + '  ' + label)
                    self.depth_stage_history = self.depth_stage_history[-6:]
                    self.depth_process_var.set('\n\n'.join(self.depth_stage_history))
                    self._workspace_journal({'kind': 'depth_pick_stage', 'request_id': status.get('request_id'),
                                             'stage': status.get('stage'), 'state': status.get('state'), 'reason': status.get('reason')})
            result = client.session.completed
            if result and result['request_id'] != self.depth_result_id:
                self.depth_result_id = result['request_id']
                self.depth_result_var.set('上次结果：' + ('已夹持（后端回报）' if result['accepted'] else
                                                       '未计成功 · ' + str(result.get('reason') or result['state'])))
                self._workspace_journal({'kind': 'depth_pick_finished', **result})
            retry = self._depth_can_retry()
            terminal = status.get('state') in {'succeeded', 'failed', 'cancelled'}
            allowed = (not self._depth_busy() and not self._depth_other_work()
                       and self.node.zero_velocity_hold_ok() and (not terminal or retry))
            self.depth_start_button.configure(state='normal' if allowed else 'disabled',
                                               text='空爪，重新抓取' if retry else '抓取前方蓝色方块')
            self.depth_stop_button.configure(state='normal' if client.session.pending or self.depth_start_wait else 'disabled')
            status = client.session.status or {}
            can_reset = self._depth_can_reset()
            self.depth_empty_button.configure(state='normal' if can_reset else 'disabled')
        except (RuntimeError, OSError, ValueError) as exc:
            self.depth_status_var.set(str(exc))
            if not client.session.pending:
                self.depth_process_var.set(str(exc))
            # Errors remain retryable after preparation ends; an active request
            # retains its cancel button and all protocol ownership rules.
            allowed = not self._depth_busy() and not self._depth_other_work() and self.node.zero_velocity_hold_ok()
            self.depth_start_button.configure(state='normal' if allowed else 'disabled')
            self.depth_stop_button.configure(state='normal' if client.session.pending or self.depth_start_wait else 'disabled')
        self._poll_depth_image()

    def _poll_depth_image(self):
        try:
            client = self.depth_client
            packet = client.take_image()
            if packet and self.depth_preview_var.get():
                from PIL import Image
                with Image.open(BytesIO(packet[0])) as image:
                    if image.format not in {'JPEG', 'PNG'} or max(image.size) > 1920 or min(image.size) < 1:
                        raise ValueError('后端画面格式或尺寸无效')
                    self.depth_original = image.convert('RGB')
                self.depth_image_at = packet[1]
                self._draw_depth_image()
            if self.depth_image_at is not None:
                age = time.monotonic() - self.depth_image_at
                self.depth_image_var.set(f'后端标注画面 · {age:.0f} 秒前' + (' · 旧帧，非实时画面' if age > 3 else ''))
        except (RuntimeError, OSError, ValueError, ImportError) as exc:
            self.depth_image_var.set('画面暂不可用（抓取状态独立显示）：' + str(exc))

    def _toggle_depth_preview(self):
        if self.depth_preview_var.get():
            self.depth_camera_window.deiconify()
            self._draw_depth_image()
        else:
            self.depth_camera_window.withdraw()

    def _close_depth_preview(self):
        self.depth_preview_var.set(False)
        self.depth_camera_window.withdraw()

    def _draw_depth_image(self):
        canvas = self.depth_canvas
        canvas.delete('all')
        width = max(1, canvas.winfo_width())
        height = max(120, canvas.winfo_height())
        if self.depth_original is None:
            canvas.create_text(width / 2, height / 2, text='等待深度相机画面\n抓取过程在独立日志区显示', justify='center',
                               fill='#b6c9d6', font=('TkDefaultFont', 12))
            return
        from PIL import Image, ImageTk
        image = self.depth_original.copy()
        image.thumbnail((width, height), Image.Resampling.LANCZOS)
        self.depth_photo = ImageTk.PhotoImage(image, master=canvas)
        canvas.create_image(width / 2, height / 2, image=self.depth_photo)
