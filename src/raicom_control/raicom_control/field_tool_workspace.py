"""Field workspace UI and optional tasks layered over the existing Nav2 queue."""
from copy import deepcopy
import json
import math
from pathlib import Path
import time
import tkinter as tk
from tkinter import simpledialog, messagebox, ttk, font as tkfont

from .field_tool_model import atomic_write_route, read_route_file, route_file_for_map
from .field_tool_map_store import archive_map, restore_map, trash_entries
from .field_tool_navigation_logic import NavigationController
from .field_tool_delivery_ui import DeliveryWorkspaceMixin
from .field_tool_depth_ui import DepthWorkspaceMixin
from .field_tool_competition_ui import CompetitionWorkspaceMixin
from .field_tool_speed import speed_settings, ProfileReadback
from .field_tool_recovery import DEFAULT_MAX_RECOVERIES, recovery_limit
from .field_tool_workspace_model import (
    duplicate_point, execution_identity, point_identity, save_settings, workspace_settings,
)


class WorkspaceNavigationController(NavigationController):
    """Default execution is unchanged; optional tasks run only after acceptance."""
    def __init__(self, node, workspace):
        super().__init__(node)
        self.workspace = workspace
        self.point_tasks = {}

    def _verify_stopped_arrival(self, now):
        previous = self.index
        super()._verify_stopped_arrival(now)
        if self.index == previous or not self.arrivals[-1]['accepted']:
            return
        task = self.point_tasks.get(self.targets[previous].name, {})
        if not task.get('dwell_s') and not task.get('apply_arm_pose'):
            return
        self.task = deepcopy(task)
        self.state = 'waypoint_task'
        self.task_until = None
        self.task_deadline = now + float(task.get('dwell_s', 0)) + 30.
        if task.get('apply_arm_pose'):
            self.workspace._arm().start('pose', task['arm_pose'])

    def poll(self):
        if self.state != 'waypoint_task':
            return super().poll()
        try:
            now = time.monotonic()
            if self.node.external_stop_requested:
                self.stop()
                return self.state
            if (not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5)
                    or not self.node.zero_velocity_hold_ok()):
                raise RuntimeError('到点任务期间定位失效或底盘未停稳')
            if now > self.task_deadline:
                raise RuntimeError('到点任务超时')
            if self.task.get('apply_arm_pose'):
                arm = self.workspace._arm()
                if arm.state == 'failed':
                    raise RuntimeError(arm.error)
                if arm.state != 'complete':
                    return self.state
            if self.task_until is None:
                self.task_until = now + float(self.task.get('dwell_s', 0))
            if now >= self.task_until:
                if self.index == len(self.targets):
                    self._finish_success()
                else:
                    self.state, self.hold_until = self.ZERO_HOLD, now + .5
        except Exception as exc:
            self._fail(str(exc))
        return self.state

    def stop(self):
        if self.state == 'waypoint_task':
            # A canceled arrival task must be retried, never skipped by resume.
            self.index = max(0, self.index - 1)
        arm = getattr(self.workspace, 'arm_io', None)
        try:
            if arm is not None:
                arm.cancel('导航任务取消')
        finally:
            super().stop()


class WorkspaceMixin(CompetitionWorkspaceMixin, DepthWorkspaceMixin, DeliveryWorkspaceMixin):
    def _workspace_init(self):
        self._delivery_init()
        self._depth_init()
        self._competition_init()
        self.map_selection_var = tk.StringVar()
        self.destination_combos, self.pick_buttons = {}, []
        self.speed_mode_var = tk.StringVar(value='分阶段前向接近')
        self.navigation_speed_vars = {key: tk.StringVar(value=str(value)) for key, value in
                                     [('far', .25), ('near', .12), ('fine', .03), ('near_distance', .25), ('turn', .35)]}
        self.navigation_speed_var = tk.StringVar(value='基线限速 0.25 m/s · 等待运行配置回读')
        self.recovery_limit_var = tk.StringVar(value=str(DEFAULT_MAX_RECOVERIES))
        self.recovery_status_var = tk.StringVar(value='默认上限 10 次 · 每个目标重新计数')
        self.navigation_profile_gate = None
        self.speed_apply = None
        self.chain_var = tk.StringVar(value='动力链 · 等待运行数据')
        self.arm_io = None
        self.delayed_start = None
        self.pickup = None
        self.pose_capture = None
        self.workspace_execution_id = None
        self.workspace_map_var = tk.StringVar(value='当前地图 · 未加载')
        self.progress_var = tk.StringVar(value='等待路线')
        self.error_var = tk.StringVar(value='位置 — cm    朝向 — °')
        self.arrival_detail_var = tk.StringVar(value='停车后同时 ≤2 cm / ≤10°')
        self.grip_var = tk.StringVar(value='夹爪未读回 · 实物未确认')
        self.task_var = tk.StringVar(value='选择目的地 → 夹取 → 确认夹稳 → 出发')
        self.destination_vars = {k: tk.StringVar() for k in ('A', 'B')}
        self.advanced_delay_var = tk.StringVar(value='0')
        self.advanced_dwell_var = tk.StringVar(value='0')
        self.advanced_note_var = tk.StringVar()
        self.advanced_point_var = tk.StringVar(value='先在路线列表选择一个点')
        self.advanced_pose_var = tk.StringVar(value='默认保持当前姿态；尚未记录')
        self.advanced_apply_var = tk.BooleanVar(value=False)
        self.node.navigation = WorkspaceNavigationController(self.node, self)

    def _style_workspace(self):
        style = ttk.Style(self.root)
        style.theme_use('clam')
        families = set(tkfont.families(self.root))
        family = next((n for n in ('Noto Sans CJK SC', 'Microsoft YaHei UI', 'WenQuanYi Zen Hei') if n in families), 'TkDefaultFont')
        self.ui_font = (family, 10)
        for name in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont'):
            tkfont.nametofont(name).configure(family=family, size=10)
        self.root.configure(bg='#eef2f6')
        style.configure('.', font=self.ui_font, background='#eef2f6', foreground='#233247')
        style.configure('TFrame', background='#eef2f6')
        style.configure('TLabel', background='#eef2f6')
        style.configure('Muted.TLabel', foreground='#69788c')
        style.configure('Title.TLabel', font=(family, 16, 'bold'), foreground='#16334a')
        style.configure('Card.TFrame', background='#ffffff')
        style.configure('Card.TLabel', background='#ffffff')
        style.configure('CardTitle.TLabel', background='#ffffff', foreground='#16334a', font=(family, 10, 'bold'))
        style.configure('Metric.TLabel', background='#ffffff', foreground='#087e8b', font=(family, 13, 'bold'))
        style.configure('TLabelframe', background='#ffffff', bordercolor='#dce4ec', relief='solid', borderwidth=1)
        style.configure('TLabelframe.Label', background='#ffffff', foreground='#16334a')
        style.configure('TButton', padding=(8, 5), borderwidth=0, background='#e6edf4', foreground='#233247')
        style.map('TButton', background=[('active', '#d4e3ef'), ('disabled', '#eef1f5')], foreground=[('disabled', '#8995a5')])
        style.configure('Accent.TButton', background='#087e8b', foreground='white')
        style.map('Accent.TButton', background=[('active', '#086977'), ('disabled', '#b5c6ca')], foreground=[('disabled', '#edf2f3')])
        style.configure('Stop.TButton', background='#fbe9e7', foreground='#a9382e')
        style.map('Stop.TButton', background=[('active', '#f7d5d0')])
        style.configure('TNotebook', borderwidth=0, background='#eef2f6')
        style.configure('TNotebook.Tab', padding=(12, 7), background='#e4eaf0')
        style.map('TNotebook.Tab', background=[('selected', '#ffffff')], foreground=[('selected', '#087e8b')])
        style.configure('TEntry', fieldbackground='#ffffff', padding=5)
        style.configure('TCombobox', padding=4, fieldbackground='#ffffff')
        style.map('TCombobox', fieldbackground=[('readonly', '#ffffff')])
        style.configure('Horizontal.TProgressbar', background='#087e8b', troughcolor='#e7edf2', borderwidth=0, thickness=6)

    def _card(self, parent, title):
        frame = ttk.Frame(parent, style='Card.TFrame', padding=9)
        ttk.Label(frame, text=title, style='CardTitle.TLabel').pack(anchor='w', pady=(0, 5))
        return frame

    def _listbox(self, parent, **kwargs):
        return tk.Listbox(parent, bg='#ffffff', fg='#233247', selectbackground='#d7edf0',
                          selectforeground='#075b68', highlightthickness=0, bd=0,
                          activestyle='none', exportselection=False, **kwargs)

    def _scrolling_content(self, parent):
        viewport = ttk.Frame(parent)
        viewport.pack(fill='both', expand=True)
        bar = ttk.Scrollbar(viewport, orient='vertical')
        bar.pack(side='right', fill='y')
        canvas = tk.Canvas(viewport, bg='#eef2f6', highlightthickness=0, bd=0,
                           width=1, height=1, yscrollcommand=bar.set)
        canvas.pack(side='left', fill='both', expand=True)
        bar.configure(command=canvas.yview)
        content = ttk.Frame(canvas)
        window = canvas.create_window(0, 0, window=content, anchor='nw')
        def resize(_event=None):
            width = canvas.winfo_width()
            height = max(canvas.winfo_height(), content.winfo_reqheight())
            if (int(canvas.itemcget(window, 'width')) != width
                    or int(canvas.itemcget(window, 'height')) != height):
                canvas.itemconfigure(window, width=width, height=height)
            canvas.configure(scrollregion=(0, 0, width, height))
        canvas.bind('<Configure>', resize)
        content.bind('<Configure>', resize)
        content._viewport_canvas = canvas
        return content

    def _bind_workspace_scroll(self, content):
        canvas = content._viewport_canvas
        def bind(widget):
            if widget.winfo_class() in ('Listbox', 'TCombobox', 'TSpinbox'):
                return
            widget.bind('<MouseWheel>', lambda e: canvas.yview_scroll(-1 if e.delta > 0 else 1, 'units'))
            widget.bind('<Button-4>', lambda _: canvas.yview_scroll(-1, 'units'))
            widget.bind('<Button-5>', lambda _: canvas.yview_scroll(1, 'units'))
            for child in widget.winfo_children():
                bind(child)
        bind(content)

    def _build_workspace_ui(self):
        self._style_workspace()
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill='both', expand=True)
        header = ttk.Frame(outer)
        header.pack(fill='x', pady=(0, 5))
        ttk.Label(header, text='RAICOM', style='Title.TLabel').pack(side='left')
        ttk.Label(header, text='场地工作台', style='Muted.TLabel').pack(side='left', padx=10)
        ttk.Label(outer, textvariable=self.status_var, wraplength=485).pack(fill='x', pady=(0, 5))
        telemetry = ttk.Frame(outer, style='Card.TFrame', padding=(9, 6))
        telemetry.pack(fill='x', pady=(0, 8))
        for var in (self.pose_var, self.tf_var, self.speed_var):
            ttk.Label(telemetry, textvariable=var, style='Card.TLabel').pack(anchor='w')
        # Pack the always-visible stop controls before the shrinking notebook.
        controls = ttk.Frame(outer, style='Card.TFrame', padding=8)
        controls.pack(side='bottom', fill='x', pady=(8, 0))
        self.manual_button = ttk.Button(controls, textvariable=self.manual_var, command=self.toggle_manual)
        self.manual_button.pack(fill='x')
        row = ttk.Frame(controls, style='Card.TFrame')
        row.pack(fill='x', pady=5)
        for key, title in [('W', 'W  前进'), ('S', 'S  后退'), ('Q', 'Q  左转'), ('E', 'E  右转')]:
            button = ttk.Button(row, text=title, width=7, takefocus=False)
            button.pack(side='left', expand=True, fill='x', padx=2)
            button.bind('<ButtonPress-1>', lambda e, k=key: self._mouse_press(e, k))
            button.bind('<ButtonRelease-1>', self._mouse_release)
        ttk.Button(controls, text='空格 / 立即停车', style='Stop.TButton', command=self.stop_all_motion).pack(fill='x')
        ttk.Button(controls, text='记录当前位置 / 纠正所选点',
                   command=lambda: self.record_draft_point() if self.mapping_running else self.record_selected_point()).pack(fill='x', pady=(5, 0))
        self.notebook = ttk.Notebook(outer)
        self.notebook.pack(fill='both', expand=True)
        self._build_mapping_tab()
        self._build_point_tab()
        self._build_navigation_tab()
        self._build_depth_tab()
        self._build_camera_tab()
        self._build_competition_tab()

    def _build_workspace_mapping(self):
        page = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(page, text='建图')
        tab = self._scrolling_content(page)
        manager = self._card(tab, '已保存地图')
        manager.pack(fill='x', pady=(0, 10))
        self.mapping_map_combo = ttk.Combobox(manager, textvariable=self.map_selection_var, state='readonly')
        self.mapping_map_combo.pack(fill='x', pady=(0, 6))
        row = ttk.Frame(manager, style='Card.TFrame')
        row.pack(fill='x')
        ttk.Button(row, text='刷新列表', command=self.refresh_map_list).pack(side='left', fill='x', expand=True)
        ttk.Button(row, text='删除所选地图…', command=self.delete_workspace_map).pack(side='left', fill='x', expand=True, padx=4)
        ttk.Button(row, text='恢复已删地图…', command=self.restore_workspace_map).pack(side='left', fill='x', expand=True)
        ttk.Label(manager, text='移入回收站，可恢复；对应点位与历史保留。', style='Muted.TLabel', wraplength=450).pack(anchor='w', pady=(6, 0))
        ttk.Label(tab, text='01  命名地图     02  移动采集     03  保存定位', style='Muted.TLabel').pack(fill='x', pady=(0, 10))
        form = self._card(tab, '新建地图')
        form.pack(fill='x')
        self.map_name_var = tk.StringVar(value='field')
        for title, variable in [('地图名称', self.map_name_var), ('点位名称', self.point_name_var), ('点位标签', self.point_label_var)]:
            row = ttk.Frame(form, style='Card.TFrame')
            row.pack(fill='x', pady=4)
            ttk.Label(row, text=title, width=10, style='Card.TLabel').pack(side='left')
            ttk.Entry(row, textvariable=variable).pack(side='left', fill='x', expand=True)
        row = ttk.Frame(tab)
        row.pack(fill='x', pady=8)
        for title, command in [('开始建图', self.start_mapping), ('保存并切换定位', self.start_save_map), ('结束建图', self.stop_mapping)]:
            ttk.Button(row, text=title, command=command).pack(side='left', fill='x', expand=True, padx=2)
        card = self._card(tab, '采集草稿点')
        card.pack(fill='both', expand=True)
        ttk.Label(card, text='停车后记录；保存地图后逐点复核。', style='Card.TLabel').pack(anchor='w', pady=(0, 6))
        self.draft_list = self._listbox(card, height=6)
        self.draft_list.pack(fill='both', expand=True)
        ttk.Button(card, text='记录草稿点', command=self.record_draft_point, style='Accent.TButton').pack(fill='x', pady=(6, 0))
        self._bind_workspace_scroll(tab)

    def _build_workspace_points(self):
        page = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(page, text='采点 / 纠偏')
        tab = self._scrolling_content(page)
        card = self._card(tab, '地图与定位')
        card.pack(fill='x')
        self.map_combo = ttk.Combobox(card, textvariable=self.map_selection_var, state='readonly')
        self.map_combo.pack(fill='x', pady=(0, 6))
        self.map_combo.bind('<<ComboboxSelected>>', self._on_map_selected)
        ttk.Button(card, text='加载地图 · 使用 start 初始定位', command=self.load_selected_map).pack(fill='x')
        ttk.Label(card, text='加载前：车须停在 start，且与保存朝向一致。', style='Card.TLabel').pack(anchor='w', pady=5)
        ttk.Button(card, text='在 RViz 纠正当前位置', command=self.toggle_initial_pose_mode).pack(fill='x')
        row = ttk.Frame(card, style='Card.TFrame'); row.pack(fill='x', pady=(6, 0))
        ttk.Button(row, text='删除所选地图…', command=self.delete_workspace_map).pack(side='left', fill='x', expand=True)
        ttk.Button(row, text='恢复已删地图…', command=self.restore_workspace_map).pack(side='left', fill='x', expand=True, padx=(5, 0))
        form = self._card(tab, '记录 / 修正点位')
        form.pack(fill='x', pady=8)
        for i, (title, variable) in enumerate([('名称', self.point_name_var), ('标签', self.point_label_var), ('类型', self.point_kind_var)]):
            row = ttk.Frame(form, style='Card.TFrame')
            row.pack(fill='x', pady=3)
            ttk.Label(row, text=title, width=7, style='Card.TLabel').pack(side='left')
            widget = (ttk.Combobox(row, textvariable=variable, state='readonly', values=('reference', 'approach'))
                      if i == 2 else ttk.Entry(row, textvariable=variable))
            widget.pack(side='left', fill='x', expand=True)
        row = ttk.Frame(form, style='Card.TFrame')
        row.pack(fill='x', pady=(7, 0))
        ttk.Button(row, text='记录 / 纠正当前点', command=self.record_selected_point, style='Accent.TButton').pack(side='left', fill='x', expand=True)
        ttk.Button(row, text='点位历史', command=self.show_history).pack(side='left', padx=(6, 0))
        card = self._card(tab, '当前地图点位 · 点击选择')
        card.pack(fill='both', expand=True)
        self.point_list = self._listbox(card, height=3)
        self.point_list.pack(fill='both', expand=True)
        self.point_list.bind('<<ListboxSelect>>', self._point_workspace_selection)
        self._bind_workspace_scroll(tab)

    def _build_workspace_navigation(self):
        page = ttk.Frame(self.notebook, padding=9)
        self.notebook.add(page, text='导航')
        heading = ttk.Frame(page)
        heading.pack(fill='x', pady=(0, 6))
        ttk.Label(heading, textvariable=self.workspace_map_var, style='Muted.TLabel').pack(side='left', fill='x', expand=True)
        ttk.Button(heading, text='高级路线编排', command=self.show_navigation_queue).pack(side='right')
        page = self._scrolling_content(page)
        route = ttk.Frame(page, style='Card.TFrame', padding=8)
        route.pack(fill='both', expand=True)
        ttk.Label(route, text='路线顺序', style='CardTitle.TLabel').pack(anchor='w')
        listrow = ttk.Frame(route, style='Card.TFrame')
        listrow.pack(fill='both', expand=True, pady=5)
        self.nav_list = self._listbox(listrow, height=5, selectmode='single')
        self.nav_list.pack(side='left', fill='both', expand=True)
        scroll = ttk.Scrollbar(listrow, orient='vertical', command=self.nav_list.yview)
        scroll.pack(side='right', fill='y')
        self.nav_list.configure(yscrollcommand=scroll.set)
        self.nav_list.bind('<ButtonPress-1>', self._nav_press)
        self.nav_list.bind('<B1-Motion>', self._nav_drag)
        self.nav_list.bind('<<ListboxSelect>>', self._nav_selection)
        row = ttk.Frame(route, style='Card.TFrame')
        row.pack(fill='x')
        for title, cmd in [('上移', self.move_selected_up), ('下移', self.move_selected_down), ('复制', self.copy_workspace_point), ('编辑', self.edit_selected_point), ('删除', self.delete_selected_point)]:
            ttk.Button(row, text=title, width=4, command=cmd).pack(side='left', expand=True, fill='x', padx=2)
        row = ttk.Frame(route, style='Card.TFrame')
        row.pack(fill='x', pady=(5, 0))
        for title, cmd, style in [('开始导航', self.start_workspace_navigation, 'Accent.TButton'), ('继续未完成', self.resume_navigation, 'TButton'), ('刷新路线', self.reload_saved_route, 'TButton')]:
            ttk.Button(row, text=title, command=cmd, style=style).pack(side='left', expand=True, fill='x', padx=2)
        telemetry = ttk.Frame(page, style='Card.TFrame', padding=8)
        telemetry.pack(fill='x', pady=7)
        ttk.Label(telemetry, textvariable=self.progress_var, style='CardTitle.TLabel').pack(anchor='w')
        progress_row = ttk.Frame(telemetry, style='Card.TFrame', height=6)
        progress_row.pack(fill='x', pady=4)
        progress_row.pack_propagate(False)
        self.route_progress = ttk.Progressbar(progress_row, maximum=100)
        self.route_progress.pack(fill='both', expand=True)
        ttk.Label(telemetry, textvariable=self.error_var, style='Metric.TLabel').pack(anchor='w')
        ttk.Label(telemetry, textvariable=self.arrival_detail_var, style='Card.TLabel', wraplength=475).pack(anchor='w')
        ttk.Label(telemetry, textvariable=self.chain_var, style='Card.TLabel', wraplength=475).pack(anchor='w', pady=(4, 0))
        ttk.Button(telemetry, text='记录动力链快照', command=self.record_command_chain).pack(anchor='e')
        recovery = self._card(page, '导航恢复次数')
        recovery.pack(fill='x', pady=(0, 7))
        row = ttk.Frame(recovery, style='Card.TFrame'); row.pack(fill='x')
        ttk.Label(row, text='每个目标恢复上限 ', style='Card.TLabel').pack(side='left')
        ttk.Entry(row, textvariable=self.recovery_limit_var, width=8).pack(side='left')
        ttk.Label(row, text=' 次', style='Card.TLabel').pack(side='left')
        ttk.Button(row, text='保存次数', command=self.save_navigation_recovery_limit).pack(side='right')
        ttk.Label(recovery, text='默认 10；超过所设次数停止。0 = 首次恢复即停。\n停车后保存；超时、定位失效和急停仍会停止。',
                  style='Card.TLabel', wraplength=450).pack(anchor='w', pady=4)
        ttk.Label(recovery, textvariable=self.recovery_status_var, style='Card.TLabel', wraplength=450).pack(anchor='w')
        speed = self._card(page, '导航速度与减速位置')
        speed.pack(fill='x', pady=(0, 7))
        ttk.Combobox(speed, state='readonly', textvariable=self.speed_mode_var,
                     values=['分阶段前向接近', '原有导航（回退）']).pack(fill='x')
        ttk.Label(speed, text='输入待应用值；实际设置以下方回读为准。', style='Card.TLabel').pack(anchor='w', pady=4)
        for key, title, unit in [
                ('far','巡航速度','m/s · 上限 2.0'),
                ('near','接近目标速度','m/s · 建议低于巡航'),
                ('fine','最后一段微调速度','m/s · 可自行设置'),
                ('near_distance','距目标多远开始减速','m · 0 表示不启用'),
                ('turn','转弯角速度','rad/s · 可自行设置')]:
            row = ttk.Frame(speed, style='Card.TFrame'); row.pack(fill='x', pady=3)
            ttk.Label(row, text=title, width=22, style='Card.TLabel').pack(side='left')
            ttk.Entry(row, textvariable=self.navigation_speed_vars[key], width=7).pack(side='left')
            ttk.Label(row, text=' '+unit, style='Card.TLabel').pack(side='left')
        ttk.Label(speed, text='按所填距离进入接近段；微调段为最后 6 cm（不超过减速距离）。\n前三项单位 m/s；第四项单位 m。1 rad/s ≈ 57°/秒。\n建议：巡航 0.25、接近 0.12、微调 0.03 m/s；减速距离 0.25 m。\n建议不限制应用，可分别设置各段速度及距离。',
                  style='Card.TLabel', wraplength=450).pack(anchor='w', pady=4)
        ttk.Label(speed, textvariable=self.navigation_speed_var, style='Card.TLabel', wraplength=450).pack(anchor='w')
        ttk.Label(speed, text='停车后应用 · 线速度上限 2.0 m/s · 默认 0.25 m/s', style='Card.TLabel').pack(anchor='w')
        ttk.Button(speed, text='保存并应用速度', command=self.save_navigation_speeds).pack(fill='x', pady=(5, 0))
        self._bind_workspace_scroll(page)
        self._build_advanced_window()

    def _build_advanced_window(self):
        self.queue_window = tk.Toplevel(self.root)
        self.queue_window.title('高级路线编排')
        self.queue_window.withdraw()
        self.queue_window.protocol('WM_DELETE_WINDOW', self.queue_window.withdraw)
        tab = ttk.Frame(self.queue_window, padding=14)
        tab.pack(fill='both', expand=True)
        ttk.Label(tab, text='高级路线编排', style='Title.TLabel').pack(anchor='w')
        ttk.Label(tab, text='基础导航参数保持原设置。', style='Muted.TLabel').pack(anchor='w', pady=(0, 10))
        row = ttk.Frame(tab)
        row.pack(fill='x', pady=5)
        ttk.Label(row, text='整条路线延时开始（秒）').pack(side='left')
        ttk.Spinbox(row, textvariable=self.advanced_delay_var, from_=0, to=3600, width=8).pack(side='right')
        card = self._card(tab, '所选点位')
        card.pack(fill='x', pady=7)
        ttk.Label(card, textvariable=self.advanced_point_var, style='Card.TLabel').pack(anchor='w')
        row = ttk.Frame(card, style='Card.TFrame')
        row.pack(fill='x', pady=7)
        ttk.Label(row, text='验收通过后停留（秒）', style='Card.TLabel').pack(side='left')
        ttk.Spinbox(row, textvariable=self.advanced_dwell_var, from_=0, to=3600, width=8).pack(side='right')
        ttk.Entry(card, textvariable=self.advanced_note_var).pack(fill='x')
        ttk.Label(card, text='点位备注（可选）', style='Card.TLabel').pack(anchor='w')
        ttk.Label(card, textvariable=self.advanced_pose_var, wraplength=430, style='Card.TLabel').pack(anchor='w', pady=8)
        ttk.Button(card, text='记录当前机械臂姿态', command=self.capture_workspace_pose).pack(fill='x')
        ttk.Checkbutton(card, text='到点后使用已记录姿态（默认保持当前姿态）', variable=self.advanced_apply_var).pack(anchor='w', pady=7)
        row = ttk.Frame(tab)
        row.pack(fill='x', pady=5)
        ttk.Button(row, text='复制所选点', command=self.copy_workspace_point).pack(side='left', fill='x', expand=True, padx=2)
        ttk.Button(row, text='仅导航到所选点', command=self.navigate_workspace_selected).pack(side='left', fill='x', expand=True, padx=2)
        ttk.Button(tab, text='保存高级设置', style='Accent.TButton', command=self.save_workspace_advanced).pack(fill='x', pady=7)
        ttk.Label(tab, text='建图草稿 · 选中后回到该点停车复核', style='Muted.TLabel').pack(anchor='w')
        self.route_draft_list = self._listbox(tab, height=3)
        self.route_draft_list.pack(fill='both', expand=True, pady=5)
        self.route_draft_list.bind('<<ListboxSelect>>', self._route_draft_selection)
        ttk.Button(tab, text='记录新点 / 复核草稿', command=self.record_selected_point).pack(fill='x')

    def _workspace_busy(self, *, include_depth=True):
        arm = getattr(self, 'arm_io', None)
        return bool(self._competition_busy() or (include_depth and self._depth_busy()) or self._delivery_busy() or getattr(self, 'delayed_start', None) or getattr(self, 'pickup', None)
                    or getattr(self, 'speed_apply', None) or getattr(self, 'pose_capture', None) or (arm and arm.busy))

    def _workspace_journal(self, value):
        value = dict(value, recorded_at_unix=time.time(), map_file=str(self.map_file))
        with (self.session_dir / 'workspace_events.ndjson').open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(value, ensure_ascii=False) + '\n')

    def _arm(self):
        if self.arm_io is None:
            from .field_tool_arm_io import FieldArmIO
            self.arm_io = FieldArmIO(self.node, self._workspace_journal)
        return self.arm_io

    def _workspace_snapshot(self):
        if self.route_document is None or self.map_file is None:
            raise ValueError('请先加载地图与点位')
        saved = read_route_file(self.route_file, self.map_file)
        if execution_identity(saved) != execution_identity(self.route_document):
            raise ValueError('路线已在外部改变，请刷新后重试')
        workspace_settings(saved)
        return execution_identity(saved)

    def _workspace_write(self, candidate):
        self._workspace_snapshot()
        atomic_write_route(self.route_file, candidate)
        self.route_document = candidate
        self.node.publish_points(candidate)
        self.refresh_nav_list()

    def _workspace_route_refreshed(self):
        if not hasattr(self, 'workspace_map_var'):
            return
        names = self.route_document['order'] if self.route_document else []
        self.workspace_map_var.set('当前地图 · ' + (Path(self.map_file).stem if self.map_file else '未加载'))
        self.point_list.delete(0, 'end')
        for name in names:
            p = self.route_document['points'][name]
            self.point_list.insert('end', f"{name}  ·  {p['label']}")
        try:
            settings = workspace_settings(self.route_document) if self.route_document else {'destinations': {}}
        except (ValueError, TypeError, KeyError) as exc:
            settings = {'destinations': {}}
            self._set_status('高级设置需要修复：' + str(exc))
        for key, combo in self.destination_combos.items():
            combo.configure(values=names)
            selected = settings['destinations'].get(key, '')
            self.destination_vars[key].set(selected if selected in names else '')
        self._advanced_selection()
        self._delivery_refresh()
        self._competition_refresh()
        if hasattr(self, 'recovery_limit_var'):
            limit = settings.get('max_recoveries', DEFAULT_MAX_RECOVERIES)
            self.recovery_limit_var.set(str(limit))
            self.recovery_status_var.set(f'已保存上限 {limit} 次 · 下次开始 / 继续导航时采用')
        if hasattr(self, 'navigation_speed_vars'):
            requested = speed_settings(settings.get('speeds'))
            self.speed_mode_var.set('分阶段前向接近' if requested['mode']=='phased' else '原有导航（回退）')
            for key, var in self.navigation_speed_vars.items(): var.set(str(requested[key]))

    def _point_workspace_selection(self, _event=None):
        selected = self.point_list.curselection()
        if selected:
            self.nav_list.selection_clear(0, 'end')
            self.nav_list.selection_set(selected[0])
            self._nav_selection()

    def _advanced_selection(self):
        if not hasattr(self, 'advanced_point_var'):
            return
        index = self._selected_route_index()
        try:
            settings = workspace_settings(self.route_document) if self.route_document else {}
            self.advanced_delay_var.set(str(settings.get('delay_s', 0)))
            if index is None or not self.route_document:
                self.advanced_point_var.set('先在主页面路线列表选择一个点')
                self.advanced_pose_var.set('默认保持当前姿态；尚未记录')
                self.advanced_apply_var.set(False)
                return
            name = self.route_document['order'][index]
            item = settings.get('points', {}).get(name, {})
            self.advanced_point_var.set(name + ' · ' + self.route_document['points'][name]['label'])
            self.advanced_dwell_var.set(str(item.get('dwell_s', 0)))
            self.advanced_note_var.set(item.get('note', ''))
            self.advanced_apply_var.set(bool(item.get('apply_arm_pose', False)))
            pose = item.get('arm_pose')
            self.advanced_pose_var.set(('已记录物理姿态：' + ' / '.join(f'{k}:{v}' for k, v in pose['positions'].items())) if pose else '默认保持当前姿态；尚未记录')
        except (ValueError, KeyError, TypeError) as exc:
            self._set_status(str(exc))

    def _save_destinations(self, _event=None):
        if not self._can_edit_route():
            self._workspace_route_refreshed()
            return
        try:
            settings = workspace_settings(self.route_document)
            settings['destinations'] = {key: var.get() for key, var in self.destination_vars.items()}
            self._workspace_write(save_settings(self.route_document, settings))
            self._set_status('A/B 目的地已保存到当前地图路线')
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))

    def copy_workspace_point(self):
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None:
            self._set_status('先在路线列表选择要复制的点')
            return
        self._release_manual()
        source = self.route_document['order'][index]
        name = simpledialog.askstring('复制点位', '新点名（原坐标完整复制）', initialvalue=source[:58] + '_copy', parent=self.root)
        if not name:
            return
        try:
            self._workspace_write(duplicate_point(self.route_document, source, name.strip()))
            self.nav_list.selection_set(index + 1)
            self._nav_selection()
            self._set_status(f'已复制 {source} → {name}，原点保留')
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))

    def save_workspace_advanced(self):
        if not self._can_edit_route():
            return
        try:
            settings = workspace_settings(self.route_document)
            settings['delay_s'] = float(self.advanced_delay_var.get())
            index = self._selected_route_index()
            if index is not None:
                name = self.route_document['order'][index]
                item = settings['points'].setdefault(name, {})
                item.update(dwell_s=float(self.advanced_dwell_var.get()), note=self.advanced_note_var.get(),
                            apply_arm_pose=bool(self.advanced_apply_var.get()))
            self._workspace_write(save_settings(self.route_document, settings))
            if index is not None:
                self.nav_list.selection_set(index)
            self._advanced_selection()
            self._set_status('高级设置已保存；坐标与导航精度未改变')
        except (ValueError, OSError) as exc:
            self._set_status(str(exc))

    def _lock_manual_for_task(self):
        self.manual_armed = self.manual_auto_pending = False
        self._release_manual(task_handoff=True)
        self.node.publish_lease(False)
        self.node.send_key('release')

    def capture_workspace_pose(self):
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None:
            self._set_status('先在路线列表选择记录姿态的点')
            return
        try:
            identity = self._workspace_snapshot()
            self._lock_manual_for_task()
            self._arm().start('capture')
            self.pose_capture = {'identity': identity, 'name': self.route_document['order'][index]}
            self._set_status('正在读取六个舵机的当前物理姿态…')
        except (ValueError, RuntimeError, OSError) as exc:
            self._set_status(str(exc))

    def start_workspace_navigation(self):
        if not self._can_edit_route():
            return
        try:
            identity = self._workspace_snapshot()
            delay = workspace_settings(self.route_document)['delay_s']
            if delay:
                if not self.node.fixed_tf_ok(.5) or not self.node.zero_velocity_hold_ok():
                    raise RuntimeError('延时开始前需要有效定位且底盘停稳')
                self._lock_manual_for_task()
                self.delayed_start = {'at': time.monotonic() + delay, 'identity': identity}
                self._set_status(f'{delay:g} 秒后开始；空格可取消倒计时')
            else:
                self.start_navigation()
        except (ValueError, RuntimeError, OSError) as exc:
            self._set_status(str(exc))

    def navigate_workspace_selected(self):
        if not self._can_edit_route():
            return
        index = self._selected_route_index()
        if index is None:
            self._set_status('请先选中一个点')
            return
        self.start_navigation(target_order=[self.route_document['order'][index]])

    def _prepare_workspace_navigation(self):
        if not hasattr(self, 'workspace_map_var'):
            return
        self.workspace_execution_id = self._workspace_snapshot()
        settings = workspace_settings(self.route_document, require_current_pose=True)
        self.node.navigation.point_tasks = settings['points']
        self.node.navigation.max_recoveries = settings['max_recoveries']
        self.navigation_speed_request = speed_settings(settings.get('speeds'))
        self.node.navigation.arrival_retry_limit = 2
        if self.navigation_profile_gate: self.navigation_profile_gate.close()
        self.navigation_profile_gate = None

    def save_navigation_recovery_limit(self):
        if not self._can_edit_route(): return
        try:
            text = self.recovery_limit_var.get().strip()
            if not text.isdecimal(): raise ValueError('恢复次数请填写非负整数，例如10')
            limit = recovery_limit(int(text))
            if self.node.navigation.cleanup_pending: raise RuntimeError('请等待上一导航目标取消完成')
            if not self.node.zero_velocity_hold_ok(): raise RuntimeError('请先停车，再保存恢复次数')
            value = workspace_settings(self.route_document); value['max_recoveries'] = limit
            self._workspace_write(save_settings(self.route_document, value))
            self.node.navigation.max_recoveries = limit
            self.recovery_status_var.set(f'已保存上限 {limit} 次 · 下次开始 / 继续导航时采用')
            self._set_status(f'恢复上限已保存为 {limit} 次；超过该次数才因恢复次数停止')
        except (ValueError, RuntimeError, OSError) as exc: self._set_status(str(exc))

    def save_navigation_speeds(self):
        if not self._can_edit_route(): return
        try:
            request = speed_settings(dict({k: float(v.get()) for k,v in self.navigation_speed_vars.items()},
                                          mode='phased' if self.speed_mode_var.get()=='分阶段前向接近' else 'legacy'))
            if self.node.navigation.cleanup_pending: raise RuntimeError('请等待上一导航目标取消完成')
            if not self.node.zero_velocity_hold_ok(): raise RuntimeError('请先停车，再应用速度')
            value = workspace_settings(self.route_document); value['speeds'] = request
            self._workspace_write(save_settings(self.route_document, value))
            process = getattr(self, 'processes', {}).get('navigation')
            if process is not None and process.poll() is None:
                self._lock_manual_for_task()
                self._launch('navigation', [
                    'ros2', 'launch', 'raicom_control', 'field_tool_navigation.launch.py',
                    'workspace_profile:=' + ('true' if request['mode'] == 'phased' else 'false')])
            if request['mode']=='legacy':
                self.navigation_speed_var.set('已选择原有导航；分阶段速度仅在分阶段模式生效')
                return
            self._lock_manual_for_task()
            self.node.reset_navigation_readiness()
            process = getattr(self, 'processes', {}).get('navigation')
            self.speed_apply={'request':request,'gate':None,'deadline':time.monotonic()+(45. if process is not None and process.poll() is None else 3.),
                              'identity':self._workspace_snapshot()}
            self.navigation_speed_var.set('设置已保存 · 等待导航服务激活后应用并回读')
        except (ValueError, RuntimeError, OSError) as exc: self._set_status(str(exc))

    def _poll_speed_apply(self):
        work=getattr(self,'speed_apply',None)
        if not work:return
        if not self.node.zero_velocity_hold_ok():raise RuntimeError('调速期间车体移动，取消应用')
        if self._workspace_snapshot()!=work['identity']:raise RuntimeError('调速期间路线设置已变化')
        if work['gate'] is None:
            if not self.node.navigation_ready():
                if time.monotonic()>work['deadline']:
                    self.speed_apply=None
                    self.navigation_speed_var.set('设置已保存、尚未应用；下次导航服务激活后自动应用并回读')
                return
            work['gate']=ProfileReadback(self.node,work['request'])
        elif not self.node.navigation_ready():
            if time.monotonic()>work['gate'].deadline:raise RuntimeError('调速等待导航服务激活超时，保持停车')
            return
        result=work['gate'].poll()
        if result is not None:
            work['gate'].close();self.speed_apply=None
            self.navigation_speed_var.set(f"已生效回读：巡航 {result['far']:g} / 接近 {result['near']:g} / 微调 {result['fine']:g} m/s；转弯 {result['turn']:g} rad/s（{math.degrees(result['turn']):.1f}°/秒）；距目标 {work['request']['near_distance']*100:g} cm 减速")
            self._workspace_journal({'kind':'manual_speed_applied','requested':work['request'],**result})

    def _workspace_navigation_arguments(self):
        request = getattr(self, 'navigation_speed_request', {'mode': 'legacy'})
        return ['workspace_profile:=' + ('true' if request['mode']=='phased' else 'false')]

    def _workspace_navigation_profile_ready(self):
        request = getattr(self, 'navigation_speed_request', {'mode': 'legacy'})
        if request['mode']=='legacy': return True
        if self.navigation_profile_gate is None: self.navigation_profile_gate = ProfileReadback(self.node, request)
        result = self.navigation_profile_gate.poll()
        if result is None: return False
        self.navigation_speed_var.set(f"已生效回读：巡航 {result['far']:g} / 接近 {result['near']:g} / 微调 {result['fine']:g} m/s；转弯 {result['turn']:g} rad/s；距目标 {request['near_distance']*100:g} cm 减速")
        self._workspace_journal({'kind': 'navigation_speed_readback', 'requested': request, **result})
        return True

    def begin_pickup(self, category):
        if not self._can_edit_route() or self.node.navigation.cleanup_pending:
            return
        try:
            identity = self._workspace_snapshot()
            if category not in ('T', 'G'):
                raise ValueError('未知配送类别')
            destination = self.destination_vars['A' if category == 'T' else 'B'].get()
            if destination not in self.route_document['points']:
                raise ValueError('先选择该类别的 A/B 目的地点名')
            if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5):
                raise RuntimeError('定位或雷达未就绪')
            self._lock_manual_for_task()
            self._arm().start('grip')
            self.pickup = {'category': category, 'destination': destination, 'identity': identity, 'phase': 'gripping'}
            self.grip_var.set('正在按当前已验证标定闭合夹爪')
            self.task_var.set(f'{category} → {destination} · 等待物理读回')
            self._workspace_journal(dict(self.pickup, kind='pickup_requested'))
        except (ValueError, RuntimeError, OSError) as exc:
            self._set_status(str(exc))

    def confirm_pickup(self):
        if not self.pickup or self.pickup['phase'] != 'confirm':
            return
        try:
            if time.monotonic() > self.pickup['deadline']:
                raise RuntimeError('夹稳确认已超时，请重新夹取')
            if self._workspace_snapshot() != self.pickup['identity']:
                raise ValueError('任务开始后路线已改变')
            self._arm().start('verify_grip')
            self.pickup['phase'] = 'verifying'
            self._workspace_journal(dict(self.pickup, kind='object_grip_confirmed_by_operator'))
            self.task_var.set('已人工确认夹稳 · 出发前复核夹爪')
        except (ValueError, RuntimeError, OSError) as exc:
            self._cancel_workspace(str(exc))
            self._set_status(str(exc))

    def _cancel_workspace(self, reason='操作已取消'):
        # Clear queued UI work BEFORE any fallible hardware cancellation. A
        # failed stop publisher must not leave a delayed start/confirmation live.
        was_busy = self._workspace_busy()
        had_pickup = bool(getattr(self, 'pickup', None))
        self.pickup = self.delayed_start = self.pose_capture = None
        if hasattr(self, 'confirm_pick_button'):
            self.confirm_pick_button.configure(state='disabled')
        cancel_error = ''
        self._cancel_competition(reason)
        self.stop_depth_pick()
        speed_work = getattr(self, 'speed_apply', None)
        self.speed_apply = None
        if speed_work is not None:
            self.navigation_speed_var.set('应用已取消；运行配置未确认，须重新应用并回读')
            try:
                if speed_work['gate'] is not None: speed_work['gate'].close()
            except Exception as exc: cancel_error = '调速清理异常：' + str(exc)
        try:
            self._cancel_delivery(reason)
        except Exception as exc:
            cancel_error = '联动取消异常：' + str(exc)
        gate = getattr(self, 'navigation_profile_gate', None)
        self.navigation_profile_gate = None
        if gate is not None:
            try: gate.close()
            except Exception as exc: cancel_error += '；参数请求清理异常：' + str(exc)
        try:
            if getattr(self, 'arm_io', None) is not None:
                self.arm_io.cancel(reason)
        except Exception as exc:
            cancel_error += str(exc)
            # Keep the original chassis emergency-stop chain as a second line
            # of protection; this does not grant any motion permission.
            try:
                self.node.halt_motion()
            except Exception as stop_exc:
                cancel_error += '; 底盘停车请求异常：' + str(stop_exc)
        final_reason = reason
        if cancel_error:
            final_reason += '；机械臂取消异常：' + cancel_error
        if had_pickup:
            self.grip_var.set('配送已取消 · 请核对夹爪状态')
            self.task_var.set(final_reason)
        if was_busy or cancel_error:
            try:
                self._workspace_journal({
                    'kind': 'workspace_cancelled', 'reason': reason,
                    'arm_cancel_error': cancel_error,
                })
            except OSError:
                pass  # Log failure must not re-enable a queued task.
        if cancel_error:
            self._set_status(final_reason)

    def _poll_workspace(self):
        if self.node.external_stop_requested and self._workspace_busy():
            self._cancel_workspace('收到停车请求')
        if self.arm_io is not None:
            self.arm_io.poll()
        self._poll_depth()
        self._poll_competition()
        self._poll_delivery()
        try:
            self._poll_speed_apply()
            if self.delayed_start:
                if (not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5)
                        or not self.node.zero_velocity_hold_ok()):
                    raise RuntimeError('倒计时期间定位失效或车辆移动，已取消')
                remaining = self.delayed_start['at'] - time.monotonic()
                self.progress_var.set(f'延时开始 · 剩余 {max(0, math.ceil(remaining))} 秒')
                if remaining <= 0:
                    identity = self.delayed_start['identity']
                    self.delayed_start = None
                    if self._workspace_snapshot() != identity:
                        raise ValueError('倒计时期间路线已改变')
                    self.start_navigation()
            if self.pose_capture:
                arm = self._arm()
                if arm.state == 'failed':
                    raise RuntimeError(arm.error)
                if arm.state == 'complete':
                    capture = self.pose_capture
                    if self._workspace_snapshot() != capture['identity']:
                        raise ValueError('记录期间路线已改变，未保存姿态')
                    settings = workspace_settings(self.route_document)
                    item = settings['points'].setdefault(capture['name'], {})
                    item.update(arm_pose=arm.captured_pose(self.session_dir / 'workspace_events.ndjson'),
                                pose_fingerprint=point_identity(self.route_document['points'][capture['name']]),
                                apply_arm_pose=False)
                    self.pose_capture = None
                    self._workspace_write(save_settings(self.route_document, settings))
                    self.nav_list.selection_set(self.route_document['order'].index(capture['name']))
                    self._advanced_selection()
                    self._set_status('当前机械臂姿态已物理读回并保存；默认保持当前姿态')
            if self.pickup:
                arm = self._arm()
                task = self.pickup
                if arm.state == 'failed':
                    raise RuntimeError(arm.error)
                if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5) or not self.node.zero_velocity_hold_ok():
                    raise RuntimeError('夹取等待期间定位失效或车辆移动')
                if task['phase'] == 'gripping' and arm.state == 'complete':
                    task.update(phase='confirm', deadline=time.monotonic() + 60.)
                    self.grip_var.set(f"夹爪读回 {arm.positions['10']} · 实物待人工确认")
                    self.task_var.set(f"{task['category']} → {task['destination']} · 请在 60 秒内确认夹稳")
                elif task['phase'] == 'confirm' and time.monotonic() > task['deadline']:
                    raise RuntimeError('夹稳确认超时；未发送导航')
                elif task['phase'] == 'verifying' and arm.state == 'complete':
                    if self._workspace_snapshot() != task['identity']:
                        raise ValueError('夹取期间路线已改变')
                    self.pickup = None  # Release task gate only after matching current confirmation.
                    self.grip_var.set(f"{task['category']} · 人工已确认夹稳 · 出发读回 {arm.positions['10']}")
                    self.task_var.set(f"目的地 {task['destination']} · 保持夹紧运输")
                    self.start_navigation(target_order=[task['destination']])
                    if not self.nav_start_pending:
                        self.task_var.set('导航未启动：' + self.status_var.get())
        except (ValueError, RuntimeError, OSError) as exc:
            self._cancel_workspace(str(exc))
            self._set_status(str(exc))
        can_confirm = bool(self.pickup and self.pickup['phase'] == 'confirm')
        if hasattr(self, 'confirm_pick_button'):
            self.confirm_pick_button.configure(state='normal' if can_confirm else 'disabled')
        busy = self._workspace_busy() or self.navigation_running or self.nav_start_pending
        for widget in self.pick_buttons:
            widget.configure(state='disabled' if busy else 'normal')
        for widget in self.destination_combos.values():
            widget.configure(state='disabled' if busy else 'readonly')

    def _refresh_workspace_metrics(self):
        nav = self.node.navigation
        if hasattr(self, 'recovery_status_var') and getattr(nav, 'goal_id', ''):
            self._set_display(self.recovery_status_var,
                              f"本目标已恢复 {nav.feedback['recoveries']} 次 / 本轮上限 {nav.active_max_recoveries} 次")
        if hasattr(self, 'chain_var'):
            self.chain_var.set(self.node.command_chain.describe(time.monotonic()))
        if not self.delayed_start:
            names = {'idle': '待命', 'waiting_acceptance': '准备目标', 'active': '行进中',
                     'verifying_stop': '停车复核', 'zero_hold': '点间停车', 'succeeded': '路线完成',
                     'failed': '已停止 · 未通过', 'stopped': '已取消', 'waypoint_task': '到点动作 / 停留'}
            total = len(nav.targets)
            target = nav.current_target()
            text = f"{names.get(nav.state, nav.state)} · {len([r for r in nav.arrivals if r['accepted']])}/{total} 点"
            if target and nav.state != nav.SUCCEEDED:
                text += ' · ' + target.name
            self._set_display(self.progress_var, text)
            self.route_progress['value'] = 100 * len([r for r in nav.arrivals if r['accepted']]) / max(1, total)
        if nav.arrivals:
            report = nav.arrivals[-1]
            self._set_display(self.error_var, f"{report['name']} · 位置 {report['position_error_m']*100:.2f} cm / 朝向 {report['yaw_error_deg']:.1f}°")
            result = '通过' if report['accepted'] else '超差'
            self._set_display(self.arrival_detail_var, f"{report['name']} · 停车后{result} · Nav2={report['nav_status']} · {report['elapsed_s']:.1f} s")
        elif not nav.last_error:
            self._set_display(self.error_var, '本目标尚未停车验收')
            self._set_display(self.arrival_detail_var, '停车后同时 ≤2 cm / ≤10°')
        if nav.last_error:
            target = nav.current_target()
            reason = {'more than two Nav2 recoveries': '导航连续恢复超过 2 次，已停止；请检查障碍与动力链记录'}.get(nav.last_error, nav.last_error)
            self._set_display(self.error_var, '当前失败目标：' + (target.name if target else '—'))
            previous = ''
            if nav.arrivals:
                r = nav.arrivals[-1]
                previous = f"\n上次停车记录 {r['name']}：{r['position_error_m']*100:.2f} cm / {r['yaw_error_deg']:.1f}°（{'通过' if r['accepted'] else '未通过'}）"
            self._set_display(self.arrival_detail_var, reason + previous)

    def _map_roots(self):
        return [self.node.map_dir, Path('/home/ubuntu/ros2_ws/src/slam/maps'),
                Path('/home/ubuntu/ros2_ws/src/raicom_control/maps')]

    def _map_change_ready(self):
        if (self._workspace_busy() or self.navigation_running or self.nav_start_pending or
                self.node.navigation.cleanup_pending or self.mapping_running or self.mapping_start_pending or
                self.save_pending or self.save_process is not None or not self.node.zero_velocity_hold_ok()):
            self._set_status('请先停车并结束当前导航、建图或保存，再管理地图')
            return False
        return True

    def delete_workspace_map(self):
        if not self._map_change_ready(): return
        index = self.map_combo.current()
        if not 0 <= index < len(self.available_maps): return
        path = self.available_maps[index]
        if not messagebox.askyesno('删除地图', f'将 {path.name} 移入回收站？\n若已加载，将结束该地图的定位。点位和历史保留，可恢复。', parent=self.root): return
        if not self._map_change_ready(): return
        try:
            route = route_file_for_map(path, self.node.route_dir)
            if self.map_file and path.resolve() == self.map_file.resolve():
                self.stop_all_motion()
                for name in ('navigation', 'localization', 'rviz_navigation'): self._stop_process(name)
                self.localization_running = False
                self.auto_start_pending = self.manual_auto_pending = False
                self.pending_initial_pose = False
                self.map_file = self.route_file = self.route_document = None
                self.resume_digest = None
                self.node.reset_map_observation()
                self.node.publish_points()
                self.refresh_nav_list()
            entry = archive_map(path, self._map_roots(), self.node.map_dir.parent / 'map_trash', route)
            self._workspace_journal({'kind':'map_archived', 'map':str(path), 'backup':str(entry)})
            self.refresh_map_list()
            self._set_status('地图已移入回收站，点位和历史保留：' + path.name)
        except (OSError, ValueError, RuntimeError, KeyError) as exc:
            self.refresh_map_list(); self._set_status('地图删除未完成：' + str(exc))

    def restore_workspace_map(self):
        if not self._map_change_ready(): return
        trash = self.node.map_dir.parent / 'map_trash'
        entries = trash_entries(trash)
        if not entries:
            self._set_status('回收站中没有待恢复地图'); return
        window = tk.Toplevel(self.root); window.title('恢复地图'); window.geometry('500x260')
        listing = tk.Listbox(window, exportselection=False); listing.pack(fill='both', expand=True, padx=8, pady=8)
        for path in entries:
            try: name = json.loads(path.read_text(encoding='utf-8'))['name']
            except (OSError, ValueError, KeyError): name = '无法读取的回收记录'
            listing.insert('end', name + ' · ' + path.parent.name)
        def restore():
            if not listing.curselection() or not self._map_change_ready(): return
            try:
                path = restore_map(entries[listing.curselection()[0]], self._map_roots(), trash)
                self.refresh_map_list(); window.destroy()
                self._set_status('地图已恢复；选择后可手动加载：' + path.name)
            except (OSError, ValueError, KeyError) as exc: self._set_status('恢复未完成：' + str(exc))
        ttk.Button(window, text='恢复所选地图（不覆盖同名文件）', command=restore).pack(fill='x', padx=8, pady=8)

    def record_command_chain(self):
        self._workspace_journal({'kind': 'command_chain_snapshot',
                                 'samples': self.node.command_chain.snapshot(time.monotonic()),
                                 'interpretation': self.node.command_chain.describe(time.monotonic()),
                                 'motor_values_are_commands_not_encoder_feedback': True})
        self._set_status('动力链快照已记录；电机命令不能代替四轮执行证据')
