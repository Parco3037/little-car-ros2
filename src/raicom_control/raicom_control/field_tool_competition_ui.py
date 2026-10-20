"""Independent competition tab; delegates navigation and pickup to existing code."""
from copy import deepcopy
import sys
import subprocess
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

from .competition_model import configuration
from .competition_runtime import CompetitionRuntimeMixin
from .field_tool_workspace_model import workspace_settings, save_settings


class CompetitionWorkspaceMixin(CompetitionRuntimeMixin):
    def connect_depth_interface(self):
        """Prewarm the installed grasp host; never replace a live owner."""
        if getattr(self,'_closed',False): return
        try:
            if self.depth_client is None:
                from .depth_pick_interface import DepthPickClient
                self.depth_client = DepthPickClient(self.node)
                self.depth_client.session.message = '接口已连接 · 等待抓取后端状态'
            if 'raicom_depth_pick_backend' not in self.node.get_node_names():
                self._launch('depth_pick_backend',[sys.executable,'-B','-m','raicom_control.depth_warm_runner'])
            self._poll_depth()
        except (RuntimeError,ValueError,OSError) as exc: self.depth_status_var.set('接口连接失败：'+str(exc))

    def _competition_init(self):
        self.competition = self.competition_io = self.competition_pick = self.competition_vision = None
        self.competition_nav = self.competition_future = self.competition_mapping = None
        self.competition_pick_identity = None
        self.competition_operation = None
        self.competition_reset_sent = False
        self.competition_dispatch = False
        self.competition_ready = False
        self.competition_start_pending = None
        self.competition_prepare_error = ''
        self.competition_profile_ready = False
        self.competition_vision_probe = None
        self.competition_last_stage = None
        self.competition_status_var = tk.StringVar(value='选择点位后点全自动开始 · 自动准备服务并运行本轮')
        self.competition_ready_var = tk.StringVar(value='地图 / 定位 / 导航 / 视觉 / 夹取 / 分类与放置：待检查')
        self.competition_timer_var = tk.StringVar(value='整轮用时 00:00.0')
        self.competition_count_var = tk.StringVar(value='完成 0 / 0 · A园区 0/4 · B园区 0/4')
        self.competition_mapping_var = tk.StringVar(value='本轮映射：尚未识别')
        self.competition_recognition = tk.StringVar()
        self.competition_parks = {p:tk.StringVar() for p in ('A','B')}
        self.competition_transport = tk.StringVar()
        self.competition_actions = {p:[tk.StringVar() for _ in range(4)] for p in ('A','B')}
        self.competition_pick_point = tk.StringVar()
        self.competition_capture_process = None
        self.competition_capture_name = tk.StringVar(value='运输姿态')
        self.competition_repeat = tk.StringVar(value='2')
        self.competition_order = []
        self.competition_point_combos = []
        self.competition_action_combos = []

    def _build_competition_tab(self):
        page=ttk.Frame(self.notebook,padding=9)
        self.notebook.add(page,text='比赛测试')
        controls=self._card(page,'比赛控制')
        controls.pack(side='bottom',fill='x',pady=(7,0))
        ttk.Label(controls,textvariable=self.competition_timer_var,style='Metric.TLabel').pack(anchor='w')
        ttk.Label(controls,textvariable=self.competition_status_var,style='Card.TLabel',wraplength=470).pack(fill='x',pady=3)
        row=ttk.Frame(controls,style='Card.TFrame');row.pack(fill='x')
        ttk.Button(row,text='仅检查准备',command=self.prepare_competition).pack(side='left',fill='x',expand=True,padx=(0,4))
        self.competition_start_button=ttk.Button(row,text='全自动开始',command=self.start_competition,style='Accent.TButton')
        self.competition_start_button.pack(side='left',fill='x',expand=True)
        ttk.Button(row,text='确认空爪·继续比赛',command=self.resume_competition).pack(side='left',fill='x',expand=True,padx=(4,0))
        ttk.Button(row,text='停止本轮',command=lambda:self.stop_all_motion(operator=True),style='Stop.TButton').pack(side='left',fill='x',expand=True,padx=(4,0))
        content=self._scrolling_content(page)
        live=self._card(content,'本轮进度与识别映射');live.pack(fill='x')
        ttk.Label(live,textvariable=self.competition_count_var,style='Card.TLabel').pack(anchor='w')
        ttk.Label(live,textvariable=self.competition_mapping_var,style='Metric.TLabel',wraplength=470).pack(fill='x',pady=5)
        ttk.Label(live,text='识别左右园区 → 深度夹取 → 转识别姿态110并用YOLO选园区 → 夹紧50回init运输 → 按该园区计数放置。',style='Card.TLabel',wraplength=470).pack(fill='x')
        ttk.Label(live,textvariable=self.competition_ready_var,style='Card.TLabel',wraplength=470).pack(fill='x')
        points=self._card(content,'比赛点位');points.pack(fill='x',pady=7)
        for title,var in [('识别区',self.competition_recognition),('A · 园区1',self.competition_parks['A']),('B · 园区2',self.competition_parks['B'])]:
            row=ttk.Frame(points,style='Card.TFrame');row.pack(fill='x',pady=2)
            ttk.Label(row,text=title,width=13,style='Card.TLabel').pack(side='left')
            combo=ttk.Combobox(row,textvariable=var,state='readonly');combo.pack(side='left',fill='x',expand=True)
            self.competition_point_combos.append(combo)
        schedule=self._card(content,'夹取顺序 · 每行一件，按列表执行');schedule.pack(fill='x')
        row=ttk.Frame(schedule,style='Card.TFrame');row.pack(fill='x')
        combo=ttk.Combobox(row,textvariable=self.competition_pick_point,state='readonly',width=15);combo.pack(side='left',fill='x',expand=True)
        self.competition_point_combos.append(combo)
        ttk.Spinbox(row,from_=1,to=8,textvariable=self.competition_repeat,width=3).pack(side='left',padx=3)
        ttk.Button(row,text='次 · 加入',command=self.add_competition_point).pack(side='left')
        self.competition_list=self._listbox(schedule,height=4);self.competition_list.pack(fill='x',pady=5)
        row=ttk.Frame(schedule,style='Card.TFrame');row.pack(fill='x')
        for text,delta in [('上移',-1),('下移',1),('删除',0)]:
            ttk.Button(row,text=text,command=lambda d=delta:self.move_competition_item(d)).pack(side='left',fill='x',expand=True,padx=2)
        actions=self._card(content,'运输姿态与各园区第1–4件的放置动作');actions.pack(fill='x',pady=7)
        ttk.Label(actions,text='默认 auto1、2、3、4，放置后回 init。10号夹取50、松爪350；运输与接近保持夹紧。动作到位后自动计数。',style='Card.TLabel',wraplength=460).pack(fill='x')
        ttk.Label(actions,text='单套放置姿态未到位累计20次，自动尝试下一套未使用姿态；跳过不计成功。',style='Muted.TLabel',wraplength=460).pack(fill='x',pady=4)
        row=ttk.Frame(actions,style='Card.TFrame');row.pack(fill='x',pady=5)
        ttk.Label(row,text='运输姿态',width=10,style='Card.TLabel').pack(side='left')
        self.competition_transport_combo=ttk.Combobox(row,textvariable=self.competition_transport,state='readonly',values=['运输保持'])
        self.competition_transport_combo.pack(side='left',fill='x',expand=True)
        for park in ('A','B'):
            for i,var in enumerate(self.competition_actions[park]):
                row=ttk.Frame(actions,style='Card.TFrame');row.pack(fill='x',pady=2)
                ttk.Label(row,text=f'{park}园区 · 第{i+1}件',width=13,style='Card.TLabel').pack(side='left')
                combo=ttk.Combobox(row,textvariable=var,state='readonly',values=['auto1','2','3','4']);combo.pack(side='left',fill='x',expand=True)
                self.competition_action_combos.append(combo)
        ttk.Button(actions,text='保存比赛设置',command=self.save_competition_config).pack(fill='x',pady=(7,0))
        capture=self._card(content,'现场姿态采集');capture.pack(fill='x',pady=7)
        ttk.Label(capture,text='先在现场摆好姿态，再读取实际舵机位置。此按钮只读取和保存，不移动机械臂；采集值仍须验证动作路径及落点。',style='Card.TLabel',wraplength=460).pack(fill='x')
        ttk.Entry(capture,textvariable=self.competition_capture_name).pack(fill='x',pady=4)
        ttk.Button(capture,text='记录当前姿态（只读）',command=self.capture_competition_pose).pack(fill='x')
        ttk.Label(content,text='一键准备并开始整轮，全程采用自动结果。园区到点、放置并回init后计数；不再识别园区方块。最后一件停在园区。',style='Muted.TLabel',wraplength=470).pack(fill='x')
        self._bind_workspace_scroll(content)

    def capture_competition_pose(self):
        if self._workspace_busy() or self.navigation_running or self.nav_start_pending or self.mapping_running: return
        try:
            if not self.node.zero_velocity_hold_ok(): raise RuntimeError('请先停车并保持机械臂静止')
            name=self.competition_capture_name.get().strip()
            if not name or len(name)>80: raise ValueError('请填写姿态名称，最多80字')
            directory=Path.home()/'raicom_runtime/competition/measurements'
            directory.mkdir(parents=True,exist_ok=True)
            token=str(time.time_ns());output=directory/(token+'.json');log=directory/(token+'.log')
            self._lock_manual_for_task()
            with log.open('xb') as stream:
                flags = {'creationflags':subprocess.CREATE_NO_WINDOW} if sys.platform == 'win32' else {}
                self.competition_capture_process=subprocess.Popen([sys.executable,'-B','-m','raicom_control.competition_capture',
                    '--name',name,'--output',str(output)],stdout=stream,stderr=subprocess.STDOUT,stdin=subprocess.DEVNULL,**flags)
            self.competition_capture_output,self.competition_capture_log=output,log
            self.competition_status_var.set('正在读取两组实际舵机位置 · 不发动作')
        except (OSError,RuntimeError,ValueError) as exc:self.competition_status_var.set(str(exc))

    def _competition_refresh(self):
        if not hasattr(self,'competition_list'):return
        names=list((self.route_document or {}).get('points',{}))
        for combo in self.competition_point_combos:combo.configure(values=names)
        if self._competition_busy():return
        saved=(self.route_document or {}).get('workspace',{}).get('competition',{})
        self.competition_recognition.set(saved.get('recognition','shibie' if 'shibie' in names else ''))
        for park,default in [('A','pick_A'),('B','pick_B')]:
            self.competition_parks[park].set(saved.get('parks',{}).get(park,default if default in names else ''))
            for i,var in enumerate(self.competition_actions[park]):
                actions=saved.get('placements',{}).get(park,[])
                var.set(actions[i] if i<len(actions) and actions[i] in ('auto1','2','3','4') else ('auto1','2','3','4')[i])
        self.competition_transport.set('运输保持')
        self.competition_order=list(saved.get('pick_order',[n for n in ('left_a','right_a') if n in names for _ in range(2)]))
        self._competition_order_display()

    def _competition_order_display(self):
        self.competition_list.delete(0,'end')
        for i,name in enumerate(self.competition_order):self.competition_list.insert('end',f'{i+1}.  {name}')
        if not self.competition:self.competition_count_var.set(f'完成 0 / {len(self.competition_order)} · A园区 0/4 · B园区 0/4')

    def add_competition_point(self):
        if self._competition_busy():return
        try:
            count=int(self.competition_repeat.get());name=self.competition_pick_point.get()
            if name not in (self.route_document or {}).get('points',{}) or not 1<=count<=8 or len(self.competition_order)+count>8:
                raise ValueError('选择有效点位，整轮最多8件')
            self.competition_order.extend([name]*count);self._competition_order_display()
        except ValueError as exc:self.competition_status_var.set(str(exc))

    def move_competition_item(self,delta):
        if self._competition_busy() or not self.competition_list.curselection():return
        i=self.competition_list.curselection()[0]
        if delta==0:self.competition_order.pop(i)
        elif 0<=i+delta<len(self.competition_order):
            self.competition_order[i],self.competition_order[i+delta]=self.competition_order[i+delta],self.competition_order[i]
            i+=delta
        self._competition_order_display()
        if i<len(self.competition_order):self.competition_list.selection_set(i)

    def _competition_form(self, require_complete=True):
        value=dict(recognition=self.competition_recognition.get(),
            parks={p:v.get() for p,v in self.competition_parks.items()},pick_order=list(self.competition_order),
            transport=self.competition_transport.get(),placements={p:[v.get() for v in vs] for p,vs in self.competition_actions.items()})
        if require_complete:return configuration(self.route_document or {},value)
        names=(self.route_document or {}).get('points',{})
        if len(value['pick_order'])>8 or any(p not in names for p in value['pick_order']):
            raise ValueError('夹取顺序含无效点位或超过8件')
        if any(p and p not in names for p in [value['recognition'],*value['parks'].values()]):
            raise ValueError('比赛草稿含已删除的点位，请重新选择')
        return value

    def save_competition_config(self):
        if not self._can_edit_route():return
        try:
            settings=workspace_settings(self.route_document);settings['competition']=self._competition_form(require_complete=False)
            self._workspace_write(save_settings(self.route_document,settings))
            self.competition_status_var.set('比赛设置已保存 · 未选动作可稍后补齐，开始前核验就绪')
        except (ValueError,RuntimeError,OSError,KeyError) as exc:self.competition_status_var.set(str(exc))
