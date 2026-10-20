"""Tkinter desktop entry for RAICOM field mapping, survey, and navigation."""

from __future__ import annotations

from copy import deepcopy
import json
import hashlib
import math
import os
from pathlib import Path
import signal
import sys
import subprocess
import tempfile
import threading
import time
import tkinter as tk
from tkinter import simpledialog, ttk

import rclpy
from rclpy.executors import SingleThreadedExecutor

from .field_tool_model import (
    atomic_write_route,
    discover_maps,
    migrate_legacy_route,
    navigation_pose,
    new_route,
    next_map_name,
    publish_staged_map,
    read_route_file,
    record_point,
    route_file_for_map,
    route_digest,
    set_approach_offset,
    set_order,
    set_point_label,
    set_start_localization,
    start_localization_pose,
)
from .field_tool_ros import FieldToolNode, NavigationTarget
from .field_tool_navigation_logic import (
    POSITION_TOLERANCE_M, YAW_TOLERANCE_DEG, repeat_target_order,
)


SLAM_RVIZ = '/home/ubuntu/ros2_ws/src/raicom_control/config/slam_low_load.rviz'
NAV_RVIZ = (
    '/home/ubuntu/ros2_ws/src/raicom_control/config/'
    'localization_watch_1fps.rviz'
)


from .field_tool_workspace import WorkspaceMixin


class FieldToolApp(WorkspaceMixin):
    def __init__(
            self,
            node: FieldToolNode,
            executor: SingleThreadedExecutor,
    ) -> None:
        self.node = node
        self.executor = executor
        self.root = tk.Tk()
        self.root.title('RAICOM 场地建图与导航')
        width = 560
        height = min(920, self.root.winfo_screenheight() - 70)
        self.root.geometry(f'{width}x{height}+{max(0, self.root.winfo_screenwidth()-width-12)}+40')
        self.root.minsize(540, 760)
        self.root.configure(bg='#0f172a')
        self.session_dir = Path(
            '/home/ubuntu/raicom_runtime/field_tool_sessions')
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.node.map_dir.mkdir(parents=True, exist_ok=True)
        self.node.route_dir.mkdir(parents=True, exist_ok=True)
        self.processes = {}
        self.mapping_running = False
        self.localization_running = False
        self.navigation_running = False
        self.manual_armed = False
        self.manual_auto_pending = True
        self.map_file = None
        self.route_file = None
        self.route_document = None
        self.draft_points = []
        self.mapping_start_pose = None
        self.mapping_anchor_pending = False
        self.auto_start_pending = False
        self.auto_start_pose = None
        self.available_maps = []
        self.last_map_pose = None
        self.pending_initial_pose = False
        self.pending_initial_pose_at = 0.0
        self.nav_start_pending = False
        self.mapping_start_pending = False
        self.mapping_start_deadline = 0.0
        self.mapping_debug_at = 0.0
        self.save_process = None
        self.save_staged_dir = None
        self.save_stem = None
        self.save_error = ''
        self.save_pending = False
        self.save_deadline = 0.0
        self.held_key = None
        self.release_job = None
        self.mouse_widget = None
        self.last_correction_status = ''
        self.last_manual_tick = time.monotonic()
        self.resume_digest = None
        self.requested_resume_index = 0
        self.nav_start_deadline = 0.0
        self.navigation_order_override = None
        self.repeat_plan = None
        self.repeat_finished_at = None
        self.initial_pose_mode = False
        self._closed = False

        self.status_var = tk.StringVar(value='启动')
        self.pose_var = tk.StringVar(value='x=--  y=--  yaw=--')
        self.tf_var = tk.StringVar(value='TF: --')
        self.speed_var = tk.StringVar(value='速度：等待实际参数读回')
        self.manual_var = tk.StringVar(value='手动等待就绪 · 条件满足后自动开启')
        self.nav_var = tk.StringVar(value='导航: idle')
        self.point_name_var = tk.StringVar(value='start')
        self.point_label_var = tk.StringVar(value='中转区')
        self.point_kind_var = tk.StringVar(value='reference')

        self._workspace_init()
        self._build_ui()
        self.executor_thread = threading.Thread(
            target=self.executor.spin, daemon=True, name='field-tool-ros')
        self.executor_thread.start()
        self.refresh_map_list()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind_all('<KeyPress>', self._key_press)
        self.root.bind_all('<KeyRelease>', self._key_release)
        # Class bindings normally run BEFORE bind_all and Space can invoke a
        # focused button. Emergency Space must never re-arm on key release.
        for button_class in ('Button', 'TButton'):
            self.root.bind_class(button_class, '<KeyPress-space>', self._key_press)
            self.root.bind_class(button_class, '<KeyRelease-space>', lambda _event: 'break')
        self.root.bind_all('<FocusOut>', self._focus_out)
        self.root.bind_all('<FocusIn>', self._focus_in)
        self.root.bind_all('<ButtonRelease-1>', self._mouse_release)
        self.notebook.bind('<<NotebookTabChanged>>', self._tab_changed)
        self._start_keyboard()
        self.root.after(50, self._manual_tick)
        self.root.after(200, self._spin)

    def _build_ui(self) -> None:
        self._build_workspace_ui()

    def _build_mapping_tab(self) -> None:
        self._build_workspace_mapping()

    def _build_point_tab(self) -> None:
        self._build_workspace_points()

    def _build_navigation_tab(self) -> None:
        self._build_workspace_navigation()

    def _launch(self, name, argv) -> None:
        existing = self.processes.get(name)
        if existing is not None and existing.poll() is None:
            if name != 'navigation' or existing.args == argv:
                return
            if (self.navigation_running or self.nav_start_pending
                    or self.node.navigation.cleanup_pending
                    or not self.node.zero_velocity_hold_ok()):
                raise RuntimeError('切换导航模式需要停车并完成上一目标取消')
            # Reuse only an identical launch. In particular, phased speeds
            # cannot be applied to a still-running legacy DWB controller.
            self._stop_process(name)
            if existing.poll() is None:
                self.processes[name] = existing
                raise RuntimeError('旧导航进程尚未退出，未启动新模式')
            self.node.reset_navigation_readiness()
        log_path = self.session_dir / f'{name}.log'
        log = log_path.open('ab')
        self.processes[name] = subprocess.Popen(
            argv,
            stdout=log,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            text=False,
        )
        log.close()

    def _stop_process(self, name) -> None:
        process = self.processes.pop(name, None)
        if process is None or process.poll() is not None:
            return
        try:
            if name.startswith('rviz_'):
                # dbus-run-session can exit before forwarding SIGINT to RViz.
                # _launch creates this private session/process group.
                os.killpg(process.pid, signal.SIGINT)
            else:
                process.send_signal(signal.SIGINT)
            try:
                # The depth adapter must finish its original-node stop
                # handshake before its own process can be terminated.
                process.wait(timeout=12.0 if name == 'depth_pick_backend' else 3.0)
                return
            except subprocess.TimeoutExpired:
                pass
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
                return
            except subprocess.TimeoutExpired:
                pass
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait(timeout=2.0)
        except (OSError, subprocess.TimeoutExpired):
            pass

    def _launch_rviz(self, name: str, config: str) -> None:
        self._launch(
            name,
            [
                'dbus-run-session', '--',
                'nice', '-n', '10', 'rviz2', '-d', config,
            ],
        )

    def show_navigation_queue(self):
        self._release_manual()
        self._advanced_selection()
        width, height = 510, min(650, self.root.winfo_screenheight() - 70)
        x = max(0, self.root.winfo_screenwidth() - width - 580)
        self.queue_window.geometry(f'{width}x{height}+{x}+40')
        self.queue_window.deiconify()
        self.queue_window.lift()

    def _tab_changed(self, event=None):
        self._release_manual()

    @staticmethod
    def _editing_widget(widget):
        # Tk-created combobox popdowns have Tcl paths, not Python widgets.
        if isinstance(widget, str):
            return True
        return widget.winfo_class() in {'Entry', 'TEntry', 'Text', 'TCombobox', 'Spinbox', 'TSpinbox'}

    def _cancel_release(self):
        if self.release_job is not None:
            self.root.after_cancel(self.release_job)
            self.release_job = None

    def _key_press(self, event):
        if event.keysym == 'space':
            self.stop_all_motion(operator=True)
            return 'break'
        if self._editing_widget(event.widget):
            self._release_manual()
            return None
        key = event.keysym.lower()
        if key in ('w', 's', 'a', 'd', 'q', 'e'):
            self._cancel_release()
            self._hold_manual(key)
            return 'break'

    def _key_release(self, event):
        if self.held_key and event.keysym.lower() == self.held_key.lower():
            # X11 auto-repeat queues release/press together; the next press cancels this.
            self._cancel_release()
            self.release_job = self.root.after_idle(self._finish_key_release)

    def _finish_key_release(self):
        self.release_job = None
        self._release_manual()

    def _focus_out(self, event):
        self.root.after_idle(self._check_focus)

    def _check_focus(self):
        if self._closed:
            return
        try:
            focus = self.root.focus_get()
        except (KeyError, tk.TclError):
            self._release_manual()
            return
        if focus is None or self._editing_widget(focus):
            self._release_manual()

    def _focus_in(self, event):
        if self._editing_widget(event.widget):
            self._release_manual()

    def _mouse_press(self, event, key):
        self.root.focus_set()
        event.widget.grab_set()
        self.mouse_widget = event.widget
        self._hold_manual(key)
        return 'break'

    def _mouse_release(self, event=None):
        if self.mouse_widget is not None:
            self._release_manual()
            return 'break'

    def _hold_manual(self, key):
        if key.lower() in ('a', 'd'):
            self._release_manual()
            self._set_status('左右横移已停用，请先转向再前进')
            return
        if self.manual_armed and not self.navigation_running and not self.nav_start_pending:
            self.held_key = key.lower()
            self.send_key(self.held_key)
        else:
            self._set_status('请先开启手动移动；导航期间不能手动驾驶')

    def _release_manual(self, *, task_handoff=False):
        self._cancel_release()
        if self.mouse_widget is not None:
            try:
                self.mouse_widget.grab_release()
            except tk.TclError:
                pass
            self.mouse_widget = None
        if self.held_key is not None:
            self.held_key = None
            self.node.send_key('release')

    def send_key(self, key: str) -> None:
        if key in ('停', ' '):
            self.stop_all_motion(operator=True)
        elif self.manual_armed and not self.navigation_running and not self.nav_start_pending:
            try:
                self.node.send_key(key)
            except (RuntimeError, OSError, ValueError) as exc:
                self.stop_all_motion()
                self._set_status(str(exc))

    def _manual_tick(self):
        if self._closed:
            return
        now = time.monotonic()
        # UI timing is diagnostic only for autonomous navigation. Held-key
        # manual driving retains its own responsiveness watchdog below.
        self.node.navigation.heartbeat.touch_ui(now)
        delayed = now - self.last_manual_tick > .150
        self.last_manual_tick = now
        if delayed and self.manual_armed and self.held_key is not None:
            # The keyboard node's 250 ms command expiry has already stopped
            # motion. Release the held key; a UI scheduling gap is not an E-stop.
            self._release_manual()
            self._set_status('界面短暂停顿，方向已释放；按住方向键可继续')
        self._maybe_enable_manual()
        if self.manual_armed and not self.navigation_running and not self.nav_start_pending:
            backend = self.processes.get('keyboard')
            if (not rclpy.ok() or backend is None or backend.poll() is not None
                    or not self.node.manual_health_ok()):
                reason = self._manual_block_reason()
                self._release_manual(task_handoff=True)
                self.manual_armed = False
                self.manual_auto_pending = True
                self.node.publish_lease(False)
                self._set_status('手动已停车：' + reason + '；恢复后自动开启')
            else:
                self.node.publish_lease(True)
                self.node.publish_manual_suppressed(False)
                if self.held_key is not None:
                    self.send_key(self.held_key)
        self.root.after(50, self._manual_tick)

    def _start_keyboard(self):
        self._launch('keyboard', ['ros2', 'run', 'raicom_control', 'keyboard_input',
            '--ros-args', '-p', 'gui_only:=true', '-p',
            f'linear_speed:={self.node.manual_linear_speed}', '-p',
            f'angular_speed:={self.node.manual_angular_speed}'])

    def _can_edit_route(self):
        if getattr(self, "_workspace_busy", lambda: False)():
            self._set_status("请先取消当前倒计时或夹取/机械臂操作")
            return False
        if self.navigation_running or self.nav_start_pending:
            self._set_status('请先停止导航，再修改点位或顺序')
            return False
        return self.route_document is not None

    def _manual_block_reason(self):
        if getattr(self, "_workspace_busy", lambda: False)():
            return "倒计时或夹取/机械臂操作中"
        if self.mapping_anchor_pending or self.auto_start_pending:
            return '等待起点记录或定位完成'
        if self.mapping_start_pending or self.save_pending or self.save_process is not None:
            return '等待建图准备或地图保存'
        if self.nav_start_pending or self.navigation_running:
            return '导航准备或执行中'
        if self.node.navigation.cleanup_pending:
            return '等待上一目标取消完成'
        if self.node.navigation.state not in {
                'idle', 'succeeded', 'failed', 'stopped'}:
            return '导航中禁止手动移动'
        backend = self.processes.get('keyboard')
        if backend is None or backend.poll() is not None:
            return '等待键盘控制进程'
        if not self.node.manual_health_ok():
            return self.node.manual_health_error()
        return ''

    def _maybe_enable_manual(self):
        if (not self.manual_auto_pending or self.manual_armed
                or self._manual_block_reason()
                or self.node.correction_deadline
                or not self.node.zero_velocity_hold_ok()):
            return
        self.toggle_manual()

    def toggle_manual(self) -> None:
        if self.manual_armed:
            self.stop_all_motion()
            return
        self.manual_auto_pending = True
        reason = self._manual_block_reason()
        if reason:
            self._set_status('手动等待：' + reason + '；恢复后自动开启')
            return
        self.node.cancel_initialpose()
        self._release_manual()
        self.manual_armed = True
        self.manual_auto_pending = False
        self.last_manual_tick = time.monotonic()
        self.node.set_motion_mission('field-manual-' + str(time.time_ns()))
        self.node.publish_lease(True)
        self._set_status('手动已就绪：按住 W/S 前后、Q/E 转向，松开停止')

    def stop_all_motion(self, operator: bool = False) -> None:
        if hasattr(self, "_cancel_workspace"):
            try:
                self._cancel_workspace()
            except Exception as exc:
                self._set_status("机械臂取消异常：" + str(exc))
        self.auto_start_pending = False
        self.save_pending = False
        self.manual_armed = False
        self.manual_auto_pending = False
        self._release_manual()
        self.node.cancel_initialpose()
        self.nav_start_pending = False
        self.mapping_start_pending = False
        self.navigation_running = False
        if operator:
            self.node.emergency_stop()
        else:
            self.node.halt_motion()

    def start_mapping(self) -> None:
        if self.mapping_running or self.save_pending or self.save_process is not None:
            self._set_status('已有建图或地图保存会话')
            return
        self.stop_navigation()
        self._stop_process('localization')
        self._stop_process('navigation')
        self._stop_process('rviz_navigation')
        self.localization_running = False
        self.mapping_start_pending = True
        self.manual_auto_pending = True
        self.mapping_start_deadline = time.monotonic() + 5.0
        self.mapping_debug_at = 0.0
        self._set_status('等待基础 TF、/scan 和零速保持就绪')

    def _maybe_start_mapping(self) -> None:
        if not self.mapping_start_pending:
            return
        base_ok = self.node.base_tf_ok()
        scan_ok = self.node.sensor_ready()
        zero_ok = self.node.zero_velocity_hold_ok()
        if not base_ok or not scan_ok:
            if time.monotonic() - self.mapping_debug_at >= 1.0:
                self.mapping_debug_at = time.monotonic()
                self._set_status(
                    f'建图等待：base_tf={base_ok} scan={scan_ok} '
                    f'zero_hold={zero_ok} details={self.node.base_tf_details()}')
            if time.monotonic() >= self.mapping_start_deadline:
                self.mapping_start_pending = False
                self._set_status(
                    f'建图未启动：base_tf={base_ok} scan={scan_ok} '
                    f'zero_hold={zero_ok} details={self.node.base_tf_details()}')
            return
        if not zero_ok:
            if time.monotonic() >= self.mapping_start_deadline:
                self.mapping_start_pending = False
                self._set_status(
                    f'建图未启动：base_tf={base_ok} scan={scan_ok} '
                    f'zero_hold={zero_ok}')
            return
        self.mapping_start_pending = False
        self._launch(
            'slam', ['ros2', 'launch', 'raicom_control', 'home_slam.launch.py'])
        self._launch_rviz('rviz_mapping', SLAM_RVIZ)
        self.mapping_running = True
        self.draft_points = []
        self.mapping_start_pose = None
        self.mapping_anchor_pending = True
        self.refresh_draft_list()
        self.manual_armed = False
        self.node.reset_map_observation()
        self.node.publish_points()
        self._set_status('建图已启动，请停在中转区并保持朝向，正在记录起点')

    def _maybe_capture_mapping_start(self) -> None:
        if not self.mapping_running or not self.mapping_anchor_pending:
            return
        try:
            pose = self.node.capture_current_pose(require_fixed_map=True)
        except RuntimeError:
            return
        # New SLAM observations only: reset_map_observation cleared the old map/TF.
        self.mapping_start_pose = deepcopy(pose)
        self.mapping_anchor_pending = False
        self.draft_points.append({
            **pose, 'name': 'start', 'label': '中转区',
            'recorded_at': time.time(), 'source': 'stationary mapping start',
        })
        self.refresh_draft_list()
        self.node.publish_points(drafts=self.draft_points)
        self._set_status('中转区 start 和朝向已记录；可开启手动扫描完整地图')

    def stop_mapping(self, keep_localization=False) -> None:
        self.mapping_anchor_pending = False
        self.save_pending = False
        self._release_manual()
        self.mapping_start_pending = False
        self.manual_armed = False
        self.node.publish_lease(False)
        self.node.send_key('release')
        self._stop_process('slam')
        self._stop_process('rviz_mapping')
        self.mapping_running = False
        if not keep_localization:
            self._set_status('建图已停止')

    def record_draft_point(self) -> None:
        if not self.mapping_running:
            self._set_status('草稿点需要建图会话')
            return
        try:
            pose = self.node.capture_current_pose(require_fixed_map=True)
        except RuntimeError as exc:
            self._set_status(str(exc))
            return
        name = self.point_name_var.get().strip()
        label = self.point_label_var.get().strip()
        self.draft_points.append({
            'name': name,
            'label': label,
            **pose,
        })
        self.refresh_draft_list()
        self.node.publish_points(drafts=self.draft_points)
        self._set_status(f'草稿点 {name} 已记录')

    def refresh_draft_list(self) -> None:
        self.draft_list.delete(0, 'end')
        for item in self.draft_points:
            self.draft_list.insert(
                'end',
                f"{item['name']}: x={item['x']:.3f} y={item['y']:.3f} "
                f"yaw={item['yaw_deg']:.1f}",
            )

    def start_save_map(self) -> None:
        if self.save_process is not None or self.save_pending:
            return
        if not self.mapping_running:
            self._set_status('保存需要正在运行的建图会话')
            return
        self.stop_all_motion()
        self.save_pending = True
        self.save_error = ''
        self.save_deadline = time.monotonic() + 5.0
        self._set_status('已停车，等待零速保持及新鲜TF/扫描后保存地图')

    def _begin_map_save(self) -> None:
        if not self.mapping_running:
            self.save_pending = False
            self._set_status('保存需要正在运行的建图会话')
            return
        if self.mapping_anchor_pending or self.mapping_start_pose is None:
            self._set_status('请等待中转区起点记录完成后保存')
            return
        try:
            self.last_map_pose = self.node.capture_current_pose(
                require_fixed_map=True)
        except RuntimeError as exc:
            self.save_error = str(exc)
            self._set_status('等待保存条件：' + self.save_error)
            return
        self.save_pending = False
        self.save_error = ''
        self.manual_armed = False
        self.node.publish_lease(False)
        self.node.send_key('release')
        self.save_stem = next_map_name(
            self.node.map_dir, self.map_name_var.get())
        self.save_staged_dir = Path(tempfile.mkdtemp(
            prefix='.' + self.save_stem + '.', dir=self.node.map_dir))
        log_path = self.session_dir / 'map_saver.log'
        log = log_path.open('ab')
        self.save_process = subprocess.Popen(
            [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(self.save_staged_dir / self.save_stem),
                '--ros-args', '-p', 'map_subscribe_transient_local:=true',
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        log.close()
        self.save_deadline = time.monotonic() + 20.0
        self._set_status('正在保存地图')

    def _poll_save(self) -> None:
        if self.save_pending:
            if time.monotonic() >= self.save_deadline:
                self.save_pending = False
                self._set_status('保存未开始：等待超时；地图及草稿仍保留。' + self.save_error)
            elif self.node.zero_velocity_hold_ok():
                # Each capture uses zero-wait TF lookups. Retry on the next UI
                # poll within the original deadline; never extend freshness.
                self._begin_map_save()
            else:
                self.save_error = 'zero-velocity hold is not satisfied'
        process = self.save_process
        if process is None:
            return
        if process.poll() is None:
            if time.monotonic() > self.save_deadline:
                self.processes['map_saver'] = process
                self._stop_process('map_saver')
                self.save_process = None
                self._set_status('地图保存超时，原始输出已保留')
            return
        self.save_process = None
        if process.returncode != 0:
            self._set_status('地图保存失败，已保留原始输出')
            self.save_staged_dir = None
            return
        try:
            staged = self.save_staged_dir / (self.save_stem + '.yaml')
            self.map_file = publish_staged_map(
                staged, self.node.map_dir)
            self.route_file = route_file_for_map(
                self.map_file, self.node.route_dir)
            self.route_document = new_route(self.map_file)
            # SLAM graph optimization can move earlier draft coordinates.
            # Keep the measurements, but require a fixed-map re-survey for navigation.
            self.route_document['drafts'] = deepcopy(self.draft_points)
            if self.mapping_start_pose is not None:
                set_start_localization(self.route_document, self.mapping_start_pose)
            atomic_write_route(self.route_file, self.route_document)
        except Exception as exc:
            self._set_status(f'地图发布或路线写入失败: {exc}')
            return
        self.stop_mapping(keep_localization=True)
        self.node.publish_points(self.route_document)
        self.start_localization()
        self.pending_initial_pose = False
        self.pending_initial_pose_at = time.monotonic() + 2.0
        self.refresh_map_list()
        self.notebook.select(1)
        self._set_status('地图已保存；车在中转区且朝向一致时，点击加载地图自动定位；其他位置请用 RViz 纠偏。草稿已保留。')

    def start_localization(self) -> None:
        if self.map_file is None:
            self._set_status('请先选择地图')
            return
        self._stop_process('localization')
        self.node.reset_map_observation()
        self._launch(
            'localization',
            [
                'ros2', 'launch', 'raicom_control', 'home_localization.launch.py',
                'map_file:=' + str(self.map_file),
            ],
        )
        self.localization_running = True
        self._launch_rviz('rviz_navigation', NAV_RVIZ)

    def load_selected_map(self, *, initialize_start=True) -> None:
        if self.save_pending or self.save_process is not None:
            self._set_status('请等待地图保存结束')
            return
        self.resume_digest = None
        self.stop_navigation()
        self.stop_mapping()
        self._stop_process('navigation')
        self._stop_process('rviz_navigation')
        self.localization_running = False
        index = self.map_combo.current()
        if index < 0 or index >= len(self.available_maps):
            return
        map_file = self.available_maps[index]
        self.map_file = map_file
        self.route_file = route_file_for_map(map_file, self.node.route_dir)
        if self.route_file.exists():
            self.route_document = read_route_file(self.route_file, map_file)
        else:
            self.route_document = migrate_legacy_route(
                map_file,
                [
                    Path('/home/ubuntu/ros2_ws/src/raicom_control/config'),
                    Path('/home/ubuntu/ros2_ws/install/raicom_control/share/raicom_control/config'),
                ],
            )
            if self.route_document is None:
                self.route_document = new_route(map_file)
            else:
                atomic_write_route(self.route_file, self.route_document)
        self.node.publish_points(self.route_document)
        self.refresh_nav_list()
        self.start_localization()
        self.auto_start_pose = (start_localization_pose(self.route_document)
                                if initialize_start else None)
        self.auto_start_pending = self.auto_start_pose is not None
        self.auto_start_deadline = time.monotonic() + 20.0
        self.manual_auto_pending = True
        self._set_status('地图已加载，等待从中转区 start 初始化；车辆须位于该点且朝向一致'
                         if self.auto_start_pending else
                         '地图已加载；定位服务就绪后，点击“在 RViz 纠正当前位置”并标出当前车位和朝向')

    def _maybe_initialize_start(self) -> None:
        if not self.auto_start_pending:
            return
        if time.monotonic() >= self.auto_start_deadline:
            self.auto_start_pending = False
            self._set_status('自动起点定位未执行：等待定位服务或停车条件超时')
            return
        if not self.node.localization_start_ready(str(self.map_file)):
            return
        try:
            pose = self.auto_start_pose
            self.node.publish_initial_pose(pose['x'], pose['y'], pose['yaw_rad'])
        except RuntimeError:
            return
        self.auto_start_pending = False
        self._set_status('已按中转区 start 初始化，等待激光定位收敛；其他位置请用 RViz 纠偏')

    def _on_map_selected(self, _event=None) -> None:
        pass

    def refresh_map_list(self) -> None:
        roots = [
            self.node.map_dir,
            Path('/home/ubuntu/ros2_ws/src/slam/maps'),
            Path('/home/ubuntu/ros2_ws/src/raicom_control/maps'),
        ]
        seen = set()
        maps = []
        for root in roots:
            for path in discover_maps(root):
                if path in seen:
                    continue
                seen.add(path)
                maps.append(path)
        self.available_maps = maps
        values = [f'{path.name}  ({path.parent})' for path in maps]
        previous = self.map_combo.get()
        selected = (values.index(previous) if previous in values else
                    maps.index(self.map_file) if self.map_file in maps else 0)
        for combo in (self.map_combo, getattr(self, 'mapping_map_combo', None)):
            if combo is None:
                continue
            combo['values'] = values
            if maps:
                combo.current(selected)
            else:
                combo.set('')

    def toggle_initial_pose_mode(self) -> None:
        self.stop_all_motion()
        if not self.localization_running or self.map_file is None:
            if self.mapping_running:
                self._set_status('请先保存当前建图，再进入固定地图定位')
                return
            self.load_selected_map(initialize_start=False)
            return
        try:
            self.node.arm_initialpose(str(self.map_file))
            self.manual_auto_pending = True
            self._set_status(self.node.correction_status)
        except RuntimeError as exc:
            self._set_status(str(exc))

    def record_selected_point(self) -> None:
        if not self._can_edit_route():
            return
        if self.route_document is None:
            self._set_status('请先加载地图')
            return
        try:
            pose = self.node.capture_current_pose(require_fixed_map=True)
        except RuntimeError as exc:
            self._set_status(str(exc))
            return
        name = self.point_name_var.get().strip()
        label = self.point_label_var.get().strip()
        try:
            candidate = deepcopy(self.route_document)
            read_route_file(self.route_file, self.map_file)
            record_point(
                candidate,
                name,
                label,
                pose['x'],
                pose['y'],
                pose['yaw_rad'],
                kind=self.point_kind_var.get(),
            )
            atomic_write_route(self.route_file, candidate)
            self.route_document = candidate
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))
            return
        self.node.publish_points(self.route_document)
        self.refresh_nav_list()
        self._set_status(f'点位 {name} 已记录')

    def refresh_nav_list(self) -> None:
        self.nav_list.delete(0, 'end')
        if hasattr(self, '_workspace_route_refreshed'):
            self._workspace_route_refreshed()
        self.route_draft_list.delete(0, 'end')
        if not self.route_document:
            return
        for draft in self.route_document.get('drafts', []):
            name = draft['name']
            state = '已加入路线' if name in self.route_document['points'] else '待复核'
            self.route_draft_list.insert(
                'end', f"{name}  {draft['label']}  [{state}]  "
                f"{draft['x']:.3f}, {draft['y']:.3f}")
        for name in self.route_document['order']:
            point = self.route_document['points'][name]
            approach = navigation_pose(point)
            self.nav_list.insert(
                'end',
                f"{name} · {point['label']}  {approach['x']:.3f}, "
                f"{approach['y']:.3f}, {approach['yaw_deg']:.1f}",
            )

    def _route_draft_selection(self, _event=None) -> None:
        selection = self.route_draft_list.curselection()
        if not selection or not self.route_document:
            return
        draft = self.route_document.get('drafts', [])[int(selection[0])]
        self.point_name_var.set(draft['name'])
        self.point_label_var.set(draft['label'])
        self.point_kind_var.set('reference')
        self._set_status(f"已选采集点 {draft['name']}；到该点停车后点记录新点")

    def reload_saved_route(self) -> None:
        if not self._can_edit_route() or self.map_file is None:
            return
        try:
            document = read_route_file(self.route_file, self.map_file)
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))
            return
        self.route_document = document
        self.resume_digest = None
        self.node.publish_points(document, drafts=document.get('drafts', []))
        self.refresh_nav_list()
        self._set_status('已刷新当前地图路线；当前位置不变')

    def _selected_route_index(self) -> int | None:
        selection = self.nav_list.curselection()
        return int(selection[0]) if selection else None

    def _nav_press(self, event) -> None:
        self._nav_drag_index = self.nav_list.nearest(event.y)

    def _nav_drag(self, event) -> None:
        if not self._can_edit_route():
            return
        index = self.nav_list.nearest(event.y)
        if index < 0 or not hasattr(self, '_nav_drag_index'):
            return
        if index == self._nav_drag_index:
            return
        order = list(self.route_document['order'])
        order.insert(index, order.pop(self._nav_drag_index))
        try:
            set_order(self.route_document, order)
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))
            return
        atomic_write_route(self.route_file, self.route_document)
        self.refresh_nav_list()
        self._nav_drag_index = index
        self.nav_list.selection_set(index)

    def _nav_selection(self, _event=None) -> None:
        index = self._selected_route_index()
        if index is None:
            return
        name = self.route_document['order'][index]
        point = self.route_document['points'][name]
        self.point_name_var.set(name)
        self.point_label_var.set(point['label'])
        if hasattr(self, '_advanced_selection'):
            self._advanced_selection()

    def move_selected_up(self) -> None:
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None or index == 0:
            return
        order = list(self.route_document['order'])
        order[index - 1], order[index] = order[index], order[index - 1]
        set_order(self.route_document, order)
        atomic_write_route(self.route_file, self.route_document)
        self.refresh_nav_list()
        self.nav_list.selection_set(index - 1)

    def move_selected_down(self) -> None:
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None or index >= len(self.route_document['order']) - 1:
            return
        order = list(self.route_document['order'])
        order[index + 1], order[index] = order[index], order[index + 1]
        set_order(self.route_document, order)
        atomic_write_route(self.route_file, self.route_document)
        self.refresh_nav_list()
        self.nav_list.selection_set(index + 1)

    def delete_selected_point(self) -> None:
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None:
            return
        name = self.route_document['order'][index]
        from .field_tool_model import delete_point
        delete_point(self.route_document, name)
        atomic_write_route(self.route_file, self.route_document)
        self.node.publish_points(self.route_document)
        self.refresh_nav_list()

    def edit_selected_point(self) -> None:
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None:
            return
        self.manual_armed = False
        self._release_manual()
        candidate = deepcopy(self.route_document)
        name = candidate['order'][index]
        point = candidate['points'][name]
        label = simpledialog.askstring(
            '编辑点位', '标签', initialvalue=point['label'], parent=self.root)
        if label is None:
            return
        if label is not None:
            try:
                set_point_label(candidate, name, label)
            except ValueError as exc:
                self._set_status(str(exc))
                return
        reference = point['reference']
        ref_x = simpledialog.askfloat(
            '编辑点位', 'reference X (m)',
            initialvalue=float(reference['x']), minvalue=-100.0,
            maxvalue=100.0, parent=self.root)
        if ref_x is None:
            return
        if ref_x is not None:
            ref_y = simpledialog.askfloat(
                '编辑点位', 'reference Y (m)',
                initialvalue=float(reference['y']), minvalue=-100.0,
                maxvalue=100.0, parent=self.root)
            if ref_y is None:
                return
            ref_yaw_deg = simpledialog.askfloat(
                '编辑点位', 'reference yaw (deg)',
                initialvalue=float(reference['yaw_deg']), minvalue=-180.0,
                maxvalue=180.0, parent=self.root)
            if ref_yaw_deg is None:
                return
            record_point(
                candidate,
                name,
                label if label is not None else self.route_document['points'][name]['label'],
                ref_x, ref_y, math.radians(ref_yaw_deg), kind='reference')
        offset = point['approach']['offset_from_reference_m']
        dx = simpledialog.askfloat(
            '编辑点位', 'approach 前进偏移 (m)',
            initialvalue=float(offset[0]), minvalue=-0.5, maxvalue=0.5,
            parent=self.root)
        if dx is None:
            return
        dy = simpledialog.askfloat(
            '编辑点位', 'approach 横向偏移 (m)',
            initialvalue=float(offset[1]), minvalue=-0.5, maxvalue=0.5,
            parent=self.root)
        if dy is None:
            return
        try:
            set_approach_offset(candidate, name, dx, dy)
            atomic_write_route(self.route_file, candidate)
            self.route_document = candidate
        except ValueError as exc:
            self._set_status(str(exc))
            return
        self.node.publish_points(self.route_document)
        self.refresh_nav_list()
        self._set_status('点位已更新')

    def resume_navigation(self):
        if getattr(self, 'navigation_order_override', None):
            self._set_status('单点任务或重复测试中断后，请重新选择目标开始')
            return
        if self.route_document is None or self.resume_digest != route_digest(self.route_document):
            self._set_status('路线或地图已改变，请从开始导航重新执行')
            return
        if self.node.navigation.state not in {'stopped', 'failed'}:
            self._set_status('没有可继续的中断路线')
            return
        self.start_navigation(resume=True)

    def start_repeat_navigation(self):
        if not self._can_edit_route() or self.node.navigation.cleanup_pending:
            return
        names = self.route_document['order']
        if len(names) < 2:
            self._set_status('请先保存两个不同端点')
            return
        pair = simpledialog.askstring('两点往返精度测试', '输入两个点名，用英文逗号分隔',
                                      initialvalue=','.join(names[-2:]), parent=self.root)
        if pair is None:
            return
        rounds = simpledialog.askinteger('两点往返精度测试', '往返轮数（一次往返为两段）',
                                         initialvalue=5, minvalue=1, maxvalue=20, parent=self.root)
        if rounds is None:
            return
        try:
            first, second = [v.strip() for v in pair.split(',')]
            pose = self.node.capture_current_pose(require_fixed_map=True)
            order = repeat_target_order(self.route_document['points'], first, second, rounds, pose)
            plan = {'kind': 'navigation_repeatability', 'rounds': rounds,
                    'endpoints': [first, second], 'target_order': order,
                    'initial_pose': pose, 'route_digest': route_digest(self.route_document),
                    'map_file': str(self.map_file),
                    'points': {name: deepcopy(self.route_document['points'][name])
                               for name in (first, second)},
                    'position_tolerance_m': POSITION_TOLERANCE_M,
                    'yaw_tolerance_deg': YAW_TOLERANCE_DEG,
                    'measurement': 'map localization estimate; no external ground truth'}
        except (ValueError, RuntimeError) as exc:
            self._set_status(str(exc))
            return
        self.start_navigation(target_order=order)
        if self.nav_start_pending:
            self.repeat_plan = plan
            self.repeat_finished_at = None
            self._set_status(f'等待服务激活，准备{rounds}轮往返；每段停车复核')

    def start_navigation(self, resume=False, *, target_order=None) -> None:
        if getattr(self, "_workspace_busy", lambda: False)():
            self._set_status("请先取消当前倒计时或夹取/机械臂操作")
            return
        if self.navigation_running or self.nav_start_pending or self.node.navigation.cleanup_pending:
            self._set_status('已有导航请求或正在等待取消完成')
            return
        if self.save_pending or self.save_process is not None:
            self._set_status('请等待地图保存完成')
            return
        if self.mapping_running:
            self.stop_mapping()
        if self.route_document is None or not self.route_document['order']:
            self._set_status('导航需要至少一个点位')
            return
        try:
            saved = read_route_file(self.route_file, self.map_file)
            if hasattr(self, '_prepare_workspace_navigation'):
                self._prepare_workspace_navigation()
            if route_digest(saved) != route_digest(self.route_document):
                raise ValueError('路线文件已改变，请重新加载当前地图')
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))
            return
        # Start the services and let the existing bounded readiness wait handle
        # a missing initial TF; a button click must not be discarded on one gap.
        if self.localization_running is False:
            self.start_localization()
        self._release_manual(task_handoff=True)
        self.manual_armed = False
        self.node.publish_lease(False)
        self.node.send_key('release')
        self.node.reset_navigation_readiness()
        try:
            self._launch(
                'navigation',
                [
                    'ros2', 'launch', 'raicom_control',
                    'field_tool_navigation.launch.py',
                ] + getattr(self, '_workspace_navigation_arguments', lambda: [])(),
            )
        except (RuntimeError, OSError) as exc:
            self._set_status(str(exc))
            return
        self._launch_rviz('rviz_navigation', NAV_RVIZ)
        self.requested_resume_index = self.node.navigation.index if resume else 0
        if not resume:
            self.navigation_order_override = list(target_order) if target_order else None
            self.repeat_plan = None
        order = getattr(self, 'navigation_order_override', None) or self.route_document['order']
        if self.requested_resume_index >= len(order):
            self._set_status('路线已经完成')
            return
        self.resume_digest = route_digest(self.route_document)
        self.nav_start_pending = True
        self.nav_inputs_ready_since = None
        self.manual_auto_pending = True
        # A measured cold start needed 25 s before all lifecycle nodes were
        # active. This wait grants no motion lease and keeps the base stopped.
        self.nav_start_deadline = time.monotonic() + 45.0
        self._set_status('正在启动 Nav2')

    def _maybe_start_navigation_queue(self) -> None:
        process = getattr(self, 'processes', {}).get('navigation')
        if self.nav_start_pending and process is not None and process.poll() is not None:
            self.stop_navigation()
            self._set_status(f'导航进程已退出（{process.returncode}），请查看 navigation.log')
            return
        if self.nav_start_pending and time.monotonic() > self.nav_start_deadline:
            detail = self.node.navigation_readiness_detail()
            self.stop_navigation()
            self._set_status('导航服务启动超时：' + detail)
            return
        if (not self.nav_start_pending
                or not self.node.navigation_ready()):
            self.nav_inputs_ready_since = None
            return
        now = time.monotonic()
        input_error = self.node.navigation_input_error()
        if input_error or not self.node.zero_velocity_hold_ok():
            self.nav_inputs_ready_since = None
            self._set_status(input_error or '导航等待底盘停稳')
            return
        if getattr(self, 'nav_inputs_ready_since', None) is None:
            self.nav_inputs_ready_since = now
        if now - self.nav_inputs_ready_since < .5:
            self._set_status('导航服务已激活 · 等待定位和雷达连续就绪')
            return
        try:
            if not getattr(self, '_workspace_navigation_profile_ready', lambda: True)(): return
        except (ValueError, RuntimeError, OSError) as exc:
            self.stop_navigation()
            self._set_status(str(exc))
            return
        targets = []
        for name in (getattr(self, 'navigation_order_override', None) or self.route_document['order']):
            point = self.route_document['points'][name]
            pose = navigation_pose(point)
            targets.append(NavigationTarget(
                name=name,
                label=point['label'],
                x=pose['x'],
                y=pose['y'],
                yaw=pose['yaw_rad'],
                position_tolerance_m=POSITION_TOLERANCE_M,
                yaw_tolerance_deg=YAW_TOLERANCE_DEG,
            ))
        mission_id = 'field-nav-' + time.strftime('%Y%m%dT%H%M%S') + '-' + str(time.monotonic_ns())
        if getattr(self, 'repeat_plan', None):
            mission_id = 'field-repeat-' + time.strftime('%Y%m%dT%H%M%S') + '-' + str(time.monotonic_ns())
        if self.resume_digest != route_digest(self.route_document):
            self.stop_navigation()
            self._set_status('路线已改变，请重新开始')
            return
        if getattr(self, 'workspace_execution_id', None):
            try:
                if self._workspace_snapshot() != self.workspace_execution_id:
                    raise ValueError('高级设置或路线已变化，请重新开始')
            except (ValueError, OSError) as exc:
                self.stop_navigation()
                self._set_status(str(exc))
                return
        if not self.node.zero_velocity_hold_ok():
            return
        if getattr(self, 'repeat_plan', None):
            # Recheck the occupied endpoint after service startup, not just
            # before the dialog. This grants no movement to reach an endpoint.
            try:
                plan = self.repeat_plan
                pose = self.node.capture_current_pose(require_fixed_map=True)
                checked_order = repeat_target_order(plan['points'], *plan['endpoints'], plan['rounds'], pose)
                if checked_order != plan['target_order']:
                    raise ValueError('车辆起始端点改变，请重新开始测试')
                plan.update(mission_id=mission_id, initial_pose=pose, started_at_unix=time.time())
                folder = self.session_dir / 'repeat_tests'
                folder.mkdir(exist_ok=True)
                with (folder / (mission_id + '.plan.json')).open('x', encoding='utf-8') as f:
                    json.dump(plan, f, ensure_ascii=False, indent=2)
            except (ValueError, RuntimeError, OSError) as exc:
                self.stop_navigation()
                self._set_status(str(exc))
                return
        started = self.node.navigation.start(
            targets, mission_id, resume_index=self.requested_resume_index)
        if not started:
            self.nav_start_pending = False
            self._set_status(self.node.navigation.last_error or '导航请求未启动')
            return
        self.nav_start_pending = False
        self.navigation_running = True
        self.manual_armed = False
        self._set_status('导航队列已启动')

    def _record_repeat_result(self):
        plan = getattr(self, 'repeat_plan', None)
        if not plan or not plan.get('mission_id') or self.node.navigation.state not in self.node.navigation.TERMINAL:
            return
        if self.repeat_finished_at is None:
            self.repeat_finished_at = time.monotonic()
        pose, error = None, ''
        try:
            pose = self.node.capture_current_pose(require_fixed_map=True)
        except RuntimeError as exc:
            error = str(exc)
            if time.monotonic() - self.repeat_finished_at < 5.0:
                return
        nav = self.node.navigation
        result = {**plan, 'finished_at_unix': time.time(), 'state': nav.state,
                  'last_error': nav.last_error, 'last_nav_status': nav.last_nav_status,
                  'arrivals': nav.arrivals, 'final_pose': pose, 'final_pose_error': error,
                  'completed_legs': len([a for a in nav.arrivals if a['accepted']])}
        path = self.session_dir / 'repeat_tests' / (plan['mission_id'] + '.result.json')
        try:
            with path.open('x', encoding='utf-8') as f:
                json.dump(result, f, ensure_ascii=False, indent=2)
        except OSError as exc:
            self._set_status('测试结果保存失败：' + str(exc))
            return
        self.repeat_plan = None
        self._set_status(f'重复测试{nav.state}：{result["completed_legs"]}/{2 * plan["rounds"]}段；结果已保存')

    def stop_navigation(self) -> None:
        self.stop_all_motion()
        self._set_status('导航已停止')

    def _set_status(self, text: str) -> None:
        if self.status_var.get() == text:
            return
        self.status_var.set(text)
        try:
            with (self.session_dir / 'field_tool.log').open('a', encoding='utf-8') as stream:
                stream.write(
                    time.strftime('%Y-%m-%dT%H:%M:%S%z') + ' ' + text + '\n')
        except OSError:
            pass

    def show_history(self) -> None:
        if self.route_document is None:
            self._set_status('请先加载地图')
            return
        name = self.point_name_var.get().strip()
        entries = [
            item for item in self.route_document.get('history', [])
            if item.get('point') == name or name == '*'
        ]
        window = tk.Toplevel(self.root)
        window.title('点位历史')
        window.geometry('720x420')
        text = tk.Text(window, bg='#111827', fg='#e5e7eb')
        text.pack(fill='both', expand=True)
        text.insert(
            '1.0',
            json.dumps(entries, ensure_ascii=False, indent=2)
            if entries else '暂无历史记录',
        )
        text.configure(state='disabled')

    @staticmethod
    def _set_display(variable, value):
        # Rewriting unchanged Tk text invalidates layout and queues redraws.
        if variable.get() != value:
            variable.set(value)

    def _refresh_state(self) -> None:
        snapshot = self.node.ui_snapshot
        pose = snapshot.get('pose')
        fresh = time.monotonic() - snapshot.get('at', 0.) < .6
        if pose and fresh:
            x, y, yaw, age = pose
            self._set_display(self.pose_var, f'x={x:.3f}  y={y:.3f}  航向={math.degrees(yaw):.1f}°  age={age:.2f}s')
        else:
            self._set_display(self.pose_var, '当前位置：等待有效 map → base TF')
        self._set_display(self.tf_var, 'TF: base ' + ('OK' if fresh and snapshot.get('base_ok') else 'FAIL')
                        + ' / fixed ' + ('OK' if fresh and snapshot.get('fixed_ok') else 'FAIL'))
        k = self.node.control_parameters.get('keyboard_input')
        g = self.node.control_parameters.get('cmd_vel_guard')
        if k and g:
            self._set_display(self.speed_var, f'手动移动：前后 {k[0]:.2f} m/s · 转向 {k[1]:.2f} rad/s · 上限 {g[0]:.2f} m/s')
        manual_text = ('手动已就绪 · 按住方向移动 · 点击关闭' if self.manual_armed
                       else ('手动等待：' + (self._manual_block_reason() or '停车或定位确认')
                             if self.manual_auto_pending else '手动已锁定 · 点击恢复自动开启'))
        self._set_display(self.manual_var, manual_text)
        self._set_display(self.nav_var, '导航: ' + self.node.navigation.state
                         + f' · 到点 ≤{POSITION_TOLERANCE_M * 100:g} cm / ≤{YAW_TOLERANCE_DEG:g}°'
                         + ((' - ' + self.node.navigation.last_error) if self.node.navigation.last_error else ''))
        if self.node.correction_deadline and time.monotonic() > self.node.correction_deadline:
            self.node.cancel_initialpose()
            self.node.correction_status = 'RViz 纠偏窗口已超时，未发布位姿'
        if self.node.correction_status != self.last_correction_status:
            self.last_correction_status = self.node.correction_status
            if self.last_correction_status:
                self._set_status(self.last_correction_status)

    def _spin(self) -> None:
        if rclpy.ok():
            self._poll_workspace()
            self.node.navigation.poll()
            if self.node.navigation.state in self.node.navigation.TERMINAL:
                self.navigation_running = False
            self._record_repeat_result()
            self._poll_save()
            self._maybe_start_mapping()
            self._maybe_capture_mapping_start()
            self._maybe_initialize_start()
            self._maybe_start_navigation_queue()
        self._refresh_state()
        self._refresh_workspace_metrics()
        self.root.after(200, self._spin)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._cancel_workspace("界面已关闭")
        finally:
            self.node.halt_motion()
        self.node.publish_lease(False)
        self.node.send_key('release')
        if self.save_process is not None and self.save_process.poll() is None:
            try:
                os.killpg(
                    os.getpgid(self.save_process.pid), signal.SIGINT)
            except (OSError, ProcessLookupError):
                pass
        for name in list(self.processes):
            self._stop_process(name)
        if getattr(self,'competition_io',None):self.competition_io.close()
        self.node.stop_health_worker()
        self.executor.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.root.destroy()


def prepare_mapping_arm():
    """Use the existing saved pose BEFORE any keyboard or lease publisher exists."""
    command = [
        '/usr/bin/python3', '-B', '/home/ubuntu/raicom_tools/mapping_arm_rest.py',
        '--profile', '/home/ubuntu/raicom_tools/mapping_rest_pose.json', '--apply', '--enable-torque', '--command-only',
    ]
    print('机械臂上电并回一次 init（夹爪50）；动作结束后保持上电，行驶不重复回位。', flush=True)
    process = subprocess.Popen(command)
    try:
        result = process.wait(timeout=35.0)
    except (subprocess.TimeoutExpired, KeyboardInterrupt):
        for sig, timeout in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            try:
                process.send_signal(sig)
                process.wait(timeout=timeout)
                break
            except subprocess.TimeoutExpired:
                continue
        raise RuntimeError('机械臂避让已中断，建图未启动')
    if result != 0:
        raise RuntimeError('机械臂启动命令未完成，建图未启动；请查看终端原因')


def main() -> None:
    import fcntl
    from rclpy.signals import SignalHandlerOptions
    # Same container lock as the existing mapping and point-survey entries.
    with open('/tmp/raicom_operator_session.lock', 'a') as operator_lock:
        try:
            fcntl.flock(operator_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有建图或导航采点会话，请先正常退出该会话')
        # Normal startup sends init once; ordinary driving never repeats it.
        # Explicit diagnostic entry avoids any startup arm action.
        if '--no-prepare-mapping-arm' not in sys.argv:
            prepare_mapping_arm()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = FieldToolNode()
        loaded = {}
        for name in (__name__, FieldToolNode.__module__,
                     'raicom_control.field_tool_navigation_logic',
                     'raicom_control.field_tool_model',
                     'raicom_control.field_tool_workspace',
                     'raicom_control.field_tool_workspace_model'):
            path = Path(sys.modules[name].__file__)
            loaded[name] = {'path': str(path), 'resolved': str(path.resolve()),
                            'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
        print('FIELD_TOOL_LOADED ' + json.dumps(loaded), flush=True)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        app = FieldToolApp(node, executor)
        if '--depth-grasp' in sys.argv:
            app.notebook.select(3)
        signal.signal(signal.SIGINT, lambda *_: app.root.after(0, app.close))
        signal.signal(signal.SIGTERM, lambda *_: app.root.after(0, app.close))
        if '--start-mapping' in sys.argv or node.get_parameter('start_mapping').value:
            app.root.after(1000, app.start_mapping)
        try:
            app.root.mainloop()
        finally:
            if not app._closed:
                app.close()


if __name__ == '__main__':
    main()
