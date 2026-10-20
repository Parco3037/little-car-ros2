"""Competition orchestration. Uses existing Nav2 and the released pick client."""
import time
import sys
import json
import os
from pathlib import Path

from .competition_model import CompetitionRun, validate_catalog
from .field_tool_delivery import geometry_identity
from .station_mapping import StationMapping
from .competition_init import CompetitionInitMixin

LABELS = {'nav_recognition':'前往识别区','recognition':'自动识别左右标识','nav_pick':'前往夹取点',
          'pick':'深度相机对齐夹取','classify':'转识别姿态50 → YOLO判定园区','transport':'夹爪保持50 → 回init运输',
          'nav_park':'前往目标园区','place':'放置：接近 → 落位 → 松爪 → 回init',
          'reset_pick':'放置动作完成 · 复位本件接口',
          'complete':'全部完成 · 停在最后园区','failed':'已停止','cancelled':'已取消'}


class CompetitionRuntimeMixin(CompetitionInitMixin):
    def _competition_checkpoint_path(self):
        return Path.home()/'raicom_runtime/competition/resume_checkpoint.json'

    def _save_competition_checkpoint(self, task):
        path=self._competition_checkpoint_path()
        path.parent.mkdir(parents=True,exist_ok=True)
        value=task.checkpoint()
        value['boot_id']=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        value['task_id']=task.task_id
        temporary=path.with_name(path.name+'.tmp')
        with temporary.open('w',encoding='utf-8') as stream:
            json.dump(value,stream,ensure_ascii=False)
            stream.flush();os.fsync(stream.fileno())
        os.replace(temporary,path)

    def resume_competition(self):
        """Operator acknowledges an empty gripper; re-recognize before retrying an unfinished item."""
        try:
            checkpoint=json.loads(self._competition_checkpoint_path().read_text(encoding='utf-8'))
            boot=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            if checkpoint.get('boot_id')!=boot:
                raise ValueError('小车已重启，不能沿用上次比赛进度')
            config=self._competition_form()
            CompetitionRun(self.route_document,config,time.monotonic()).resume(checkpoint)
            self._start_competition(config,checkpoint)
        except (ValueError,RuntimeError,OSError,KeyError,TypeError) as exc:
            self.competition_status_var.set('无法续赛：'+str(exc))

    def _competition_busy(self):
        task=getattr(self,'competition',None)
        active=task and task.stage not in task.TERMINAL and not getattr(self,'competition_dispatch',False)
        return bool(active or getattr(self,'competition_start_pending',None) or getattr(self,'competition_capture_process',None) or (getattr(self,'competition_io',None) and self.competition_io.session.pending)
                    or (getattr(self,'competition_pick',None) and self.competition_pick.session.pending))

    def _competition_connect(self):
        if self.competition_io is not None:return
        from .competition_io import CompetitionIO
        from .depth_pick_interface import DepthPickClient
        from .vision_ros_client import VisionClient
        self.competition_io=CompetitionIO(self.node)
        self.competition_pick=DepthPickClient(self.node)
        self.competition_vision=VisionClient(self.node,with_images=True)

    def prepare_competition(self):
        if self._workspace_busy() or self.navigation_running or self.nav_start_pending:return
        try:
            if (not self.route_document or self.mapping_running or self.mapping_start_pending or self.save_pending
                    or self.save_process is not None or not self.node.zero_velocity_hold_ok()):
                raise RuntimeError('请加载地图、结束建图/保存并停车')
            self._competition_connect()
            self.connect_depth_interface()
            from pathlib import Path
            from .competition_named_program import save_program
            program_path=Path.home()/'raicom_runtime/competition/named_program.json'
            save_program(program_path,self.route_document,self._competition_form())
            if 'raicom_competition_backend' not in self.node.get_node_names():
                self._launch('competition_backend',[sys.executable,'-B','-m','raicom_control.competition_backend',
                             '--calibration',str(program_path),'--execute-calibrated-actions'])
            self._stop_vision_preview('准备比赛识别')
            self.start_vision_nodes()
            from .competition_vision_preflight import VisionPreflight
            self.competition_vision.cancel()
            self.competition_vision_probe=VisionPreflight()
            self._prepare_workspace_navigation()
            self.node.navigation.point_tasks={}  # Competition owns all arm phases.
            self._lock_manual_for_task()
            self.node.reset_navigation_readiness()
            self._launch('navigation',['ros2','launch','raicom_control','field_tool_navigation.launch.py']+self._workspace_navigation_arguments())
            self.competition_prepare_error=''
            self.competition_profile_ready=False
            self.competition_all_ready_since=None
            self.competition_status_var.set('准备服务中 · 未发送目标，等待Active及后端就绪')
            return True
        except (ValueError,RuntimeError,OSError) as exc:
            self.competition_prepare_error=str(exc);self.competition_status_var.set(str(exc))
            return False

    def _competition_readiness(self):
        now=time.monotonic()
        checks={'地图':bool(self.route_document and self.map_file),
                '定位':bool(self.localization_running and self.node.fixed_tf_ok(.5) and self.node.sensor_ready(.5)),
                '导航':False,'视觉':False,'夹取':False,'分类/放置':False,'动作配置':False}
        detail=''
        if self.competition_io:
            nav_ready=self.node.navigation_ready()
            if nav_ready and not self.competition_profile_ready:
                try:self.competition_profile_ready=self._workspace_navigation_profile_ready()
                except (ValueError,RuntimeError,OSError) as exc:detail=str(exc)
            checks['导航']=bool(nav_ready and self.competition_profile_ready)
            probe=getattr(self,'competition_vision_probe',None)
            checks['视觉']=bool(probe and probe.poll(self.competition_vision,now,
                backend_ready=self.competition_io.vision_ready(now),
                allow_request=not getattr(self,'vision_preview_active',False)
                              and self.node.zero_velocity_hold_ok()))
            if probe:detail=detail or probe.detail
            pick=self.competition_pick
            checks['夹取']=bool(pick.session.ready(now) and self.node.count_subscribers('/raicom/depth_pick/request')==1
                                and self.node.count_publishers('/raicom/depth_pick/status')==1)
            checks['分类/放置']=bool(self.competition_io.session.ready(now) and self.competition_io.unique())
            if not checks['分类/放置']:
                detail += ('\n' if detail else '')+str(self.competition_io.session.health.get('reason') or '')
            catalog=self.competition_io.session.health.get('catalog',{})
            self.competition_transport_combo.configure(values=list(catalog.get('transport',{})) or ['运输保持'])
            for combo in self.competition_action_combos:combo.configure(values=list(catalog.get('placements',{})) or ['auto1','2','3','4'])
            try:
                config=self._competition_form();validate_catalog(self.route_document,config,catalog)
                checks['动作配置']=True
            except (ValueError,KeyError,TypeError) as exc:detail+=('\n' if detail else '')+str(exc)
            if all(checks.values()):
                fallback=[]
                if pick.session.status.get('automatic_holding_available') is not True:fallback.append('自动夹稳观察待接入')
                if fallback:detail+=('\n' if detail else '')+'；'.join(fallback)+'（只采用自动结果）'
                if catalog.get('physical_validation')=='pending':detail+='\n指定动作已载入；放置/松爪完成后回init，回位不作严格到位检查'
        self.competition_ready_var.set(' · '.join(k+(' ✓' if v else ' 待就绪') for k,v in checks.items())
                                       + ('\n'+detail if detail else ''))
        self.competition_ready=all(checks.values()) and self.node.zero_velocity_hold_ok() and not self.competition_prepare_error
        return self.competition_ready

    def start_competition(self):
        """One click arms a bounded preparation; polling dispatches only after ready."""
        try:
            self._start_competition(self._competition_form(),None)
        except (ValueError,RuntimeError,OSError,KeyError) as exc:self.competition_status_var.set(str(exc))

    def _start_competition(self, config, checkpoint):
        try:
            if self._workspace_busy() or self.navigation_running or self.nav_start_pending or self.mapping_running or self.save_pending or self.save_process is not None:
                raise RuntimeError('请先结束其他任务')
            started=time.monotonic()
            stop_generation=getattr(self.node,'external_stop_generation',0)
            if not self.prepare_competition():return
            self.competition_start_pending=dict(config=config,checkpoint=checkpoint,started=started,deadline=started+180.,
                                                identity=self._workspace_snapshot(),stop_generation=stop_generation)
            self.competition=None
            self.competition_status_var.set('一键启动 · 正在准备，服务就绪后自动开始')
            self._workspace_journal({'kind':'competition_preparing','config':config})
        except (ValueError,RuntimeError,OSError,KeyError) as exc:self.competition_status_var.set(str(exc))

    def _poll_competition_start(self, now):
        pending=getattr(self,'competition_start_pending',None)
        if not pending:return
        if getattr(self.node,'external_stop_generation',0)!=pending['stop_generation']:
            raise RuntimeError('准备期间收到急停，本轮不自动开始')
        if self.competition_prepare_error:raise RuntimeError(self.competition_prepare_error)
        if now>=pending['deadline']:raise RuntimeError('比赛准备超时：'+self.competition_ready_var.get())
        if self._workspace_snapshot()!=pending['identity'] or self._competition_form()!=pending['config']:
            raise RuntimeError('准备期间地图、点位或比赛设置已改变，请重新开始')
        if not self.node.zero_velocity_hold_ok():raise RuntimeError('准备期间车体移动，已取消自动开始')
        elapsed=now-pending['started']
        self.competition_timer_var.set(f'整轮用时 {int(elapsed//60):02}:{elapsed%60:04.1f} · 准备中')
        if not self._competition_init_ready(('competition',pending['started'])):
            self.competition_status_var.set('比赛开始前回 init · 完成后自动导航')
            return
        if not self._competition_readiness():
            self.competition_all_ready_since=None
            return
        if getattr(self,'competition_all_ready_since',None) is None:
            self.competition_all_ready_since=now
            return
        if now-self.competition_all_ready_since<1.0:return
        self._begin_competition(pending,now)

    def _begin_competition(self, pending, now):
        try:
            self.competition_pick.poll();self.competition_io.poll()
            if getattr(self.node,'external_stop_generation',0)!=pending['stop_generation']:
                raise RuntimeError('准备期间收到急停，本轮不自动开始')
            if not self._competition_readiness():
                self.competition_all_ready_since=None
                return
            config=pending['config']
            self._stop_vision_preview('比赛运行中')
            self._lock_manual_for_task()
            self.competition=CompetitionRun(self.route_document,config,now)
            if pending.get('checkpoint'):
                self.competition.resume(pending['checkpoint'])
            self._save_competition_checkpoint(self.competition)
            self.competition_vision_probe=None  # Recognition will use a new task-scoped request.
            self.competition.started=pending['started']
            self.competition_start_pending=None
            self.competition_identity=self._workspace_snapshot()
            self.competition_backend=self.competition_io.session.health['backend_id']
            self.competition_catalog=self.competition_io.session.health['catalog']
            self.competition_nav=self.competition_future=self.competition_mapping=None
            self.competition_input_wait=None
            self.competition_stop_generation=pending['stop_generation']
            self.competition_pick_identity=self.competition_operation=None
            self.competition_pick_retries=0
            self.competition_pick_retry_wait=False
            self.competition_reset_sent=False
            self.competition_last_stage=None
            self.competition_mapping_var.set('本轮映射：尚未识别，旧轮结果已清空')
            self._workspace_journal({'kind':'competition_started','task_id':self.competition.task_id,'config':config,
                                     'resumed_completed':self.competition.completed})
        except (ValueError,RuntimeError,OSError,KeyError):
            self.competition_start_pending=None
            raise

    def _cancel_competition(self, reason='人工取消'):
        self._cancel_competition_init(reason)
        self.competition_input_wait=None
        self.competition_vision_probe=None
        if getattr(self,'competition_start_pending',None):
            self.competition_start_pending=None
            self.competition_status_var.set('启动已取消 · '+reason)
        capture=getattr(self,'competition_capture_process',None)
        if capture and capture.poll() is None:capture.terminate()
        task=getattr(self,'competition',None)
        if task and task.stage not in task.TERMINAL:
            task.stop(reason,time.monotonic(),cancelled=True)
            if self.competition_io:self.competition_io.heartbeat(task,False)
            try:self._workspace_journal({'kind':'competition_cancelled','reason':reason,**task.context()})
            except OSError:pass
        for client in (getattr(self,'competition_pick',None),getattr(self,'competition_io',None),getattr(self,'competition_vision',None)):
            if client:
                try:client.cancel()
                except Exception as exc:self.competition_status_var.set('取消发送异常，停止未确认：'+str(exc))
        self.competition_future=None

    def _competition_inputs_ready(self, task, now):
        if getattr(self.node,'external_stop_generation',0)!=getattr(self,'competition_stop_generation',0):
            raise RuntimeError('收到急停，本轮不自动恢复')
        if not task.stage.startswith('nav_'):
            # Navigation already verified station arrival. The stopped arm and
            # camera stages have their own physical admission checks; a GUI TF
            # sample gap must not cancel an independently supervised action.
            if not self.node.zero_velocity_hold_ok():
                raise RuntimeError('停车阶段检测到底盘非零输出或停稳反馈失效')
            return True
        fresh=self.node.fixed_tf_ok(.5) and self.node.sensor_ready(.5)
        self.competition_stationary_input_gap=None
        nav=self.node.navigation
        wait=getattr(self,'competition_input_wait',None)
        prior_sensor_failure=(task.stage.startswith('nav_') and nav.state==nav.FAILED
                              and getattr(nav,'last_failure_kind','') in
                              {'sensor_unavailable','heartbeat_unavailable'})
        owned=getattr(self,'competition_nav',None)
        if task.stage.startswith('nav_') and owned is not None:
            if owned['mission'] is not None and nav.mission_id!=owned['mission']:
                raise RuntimeError('导航任务身份发生变化')
            owns_current=(owned['mission'] is not None or
                          (not self.nav_start_pending and nav.mission_id!=owned['previous']))
            if owns_current and nav.state in (nav.FAILED,nav.STOPPED) and not prior_sensor_failure:
                raise RuntimeError(nav.last_error or '导航已停止')
            if (owns_current and wait is None and nav.state in
                    (nav.WAITING_ACCEPTANCE,nav.ACTIVE,nav.VERIFYING_STOP,nav.ZERO_HOLD,nav.SUCCEEDED)):
                # Navigation owns preflight/arrival checks; Nav2 owns TF during
                # execution. Do not cancel/reissue on this listener's TF gap.
                return True
        if fresh and wait is None and not prior_sensor_failure:return True
        if wait is None:
            # Cancel through the existing controller/guard; never replay the old
            # goal identity or allow a delayed acceptance to start driving.
            failure_reason = nav.last_error or '定位或雷达数据暂不可用'
            self.node.navigation.pause_sensor_input()
            self.nav_start_pending=self.navigation_running=self.manual_auto_pending=False
            self.competition_nav=None
            wait=dict(started=now,stable_since=None,task_id=task.task_id,
                      stage=task.stage,target=task.target,
                      reason=failure_reason)
            self.competition_input_wait=wait
            self._workspace_journal(dict(kind='competition_input_wait',**wait))
        if (wait['task_id'],wait['stage'],wait['target'])!=(task.task_id,task.stage,task.target):
            raise RuntimeError('等待恢复期间任务或目标发生变化')
        self.node.publish_lease(False)
        if now>=wait['started']+30.:
            raise RuntimeError('停车等待导航输入/心跳30秒仍未恢复')
        if not fresh or nav.cleanup_pending or not self.node.zero_velocity_hold_ok():
            wait['stable_since']=None
        elif wait['stable_since'] is None:
            wait['stable_since']=now
        elif now-wait['stable_since']>=1.:
            self.competition_input_wait=None
            self._workspace_journal(dict(kind='competition_input_resumed',wait_s=now-wait['started'],
                                         target=task.target,task_id=task.task_id))
            return True
        self.competition_status_var.set('已停车，等待导航输入/心跳恢复后继续同一目标 · '+task.target)
        return False

    def _competition_navigate(self, task, now):
        if self.competition_nav is None:
            if self.navigation_running or self.nav_start_pending:return
            previous=self.node.navigation.mission_id
            self.competition_dispatch=True
            try:self.start_navigation(target_order=[task.target])
            finally:self.competition_dispatch=False
            self.manual_auto_pending=False
            self.node.navigation.point_tasks={}
            if not self.nav_start_pending:
                # The GUI checks TF again before dispatch. A frame can expire
                # between that check and our preceding readiness check.
                if not self._competition_inputs_ready(task,now):return
                raise RuntimeError('导航未建立：'+self.status_var.get())
            self.competition_nav={'previous':previous,'mission':None}
        elif not self.nav_start_pending:
            nav=self.node.navigation
            if self.competition_nav['mission'] is None:
                if nav.mission_id==self.competition_nav['previous']:raise RuntimeError('导航没有建立新任务')
                self.competition_nav['mission']=nav.mission_id
            if nav.mission_id!=self.competition_nav['mission']:raise RuntimeError('导航任务身份发生变化')
            if nav.state==nav.SUCCEEDED:
                if nav.cleanup_pending:return
                self._workspace_journal({'kind':'competition_arrival','mission_id':nav.mission_id,'report':nav.arrivals[-1],**task.context()})
                task.arrived(nav.arrivals[-1],now);self.competition_nav=None
            elif nav.state in (nav.FAILED,nav.STOPPED):raise RuntimeError(nav.last_error or '导航已停止')

    def _competition_recognize(self, task, now):
        # This stage needs a current map pose to assign left/right markers.
        # Wait for that observation without cancelling the already arrived run.
        if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5):return
        try:
            self._assert_at_recognition(task.config['recognition'])
        except RuntimeError:
            # Input can expire between readiness and pose capture.
            if not self.node.fixed_tf_ok(.5) or not self.node.sensor_ready(.5):return
            raise
        if self.competition_mapping is None:
            self.competition_mapping=StationMapping(task.task_id,task.geometry,task.config['recognition'],task.config['parks'],required_frames=1,allow_cached=True)
        if self.competition_future is None:
            if self.competition_vision.ready():self.competition_future=self.competition_vision.request(task.task_id)
        elif self.competition_future.done():
            try:reply=self.competition_future.result()
            except Exception as exc:reply={'payload':{},'error':str(exc)}
            self.competition_future=None
            mapping=self.competition_mapping.observe(reply,stopped=True)
            from .vision_decision import decision_text
            decision=self.competition_mapping.consensus.last_decision
            self.competition_mapping_var.set(decision_text(decision)+'\n一次有效左右识别即可继续')
            self._workspace_journal({'kind':'competition_recognition_decision','decision':decision,**task.context()})
            if mapping:
                task.recognized(mapping,now)
                self.competition_mapping_var.set('\n'.join(f'{c} → {p}园区（{mapping["category_to_point"][c]}）' for c,p in mapping['category_to_park'].items()))
                if mapping.get('mapping_source')=='last_valid_detection':
                    self.competition_mapping_var.set('当前帧无框 · 沿用上次识别\n'+self.competition_mapping_var.get())
                self._publish_station_mapping(mapping)
                self._workspace_journal({'kind':'competition_mapping',**mapping})

    def _competition_pick_once(self, task, now):
        client=self.competition_pick
        if getattr(self,'competition_pick_retry_wait',False):
            if client.session.ready(now):
                # The old request has been explicitly released and reset by
                # its own backend; the next poll may create a new identity.
                self.competition_pick_retry_wait=False
                self.competition_pick_identity=None
                client.session.completed=None
            return
        if self.competition_pick_identity is None:
            if self.node.navigation.cleanup_pending or not client.session.ready(now):return
            # PICK_READY is the event; translate it into the released start contract.
            from .competition_pick_contract import context_for
            context=context_for(task)
            client.start(competition=context);self.competition_pick_identity=dict(client.session.pending)
            self.competition_io.event('PICK_READY',**task.context(),point=task.target,
                mapping=context['mapping'],pick_request=dict(client.session.pending))
            self._workspace_journal({'kind':'competition_pick_ready',**task.context(),'pick_request':dict(client.session.pending)})
        elif client.session.completed:
            if not client.session.fresh(now):raise RuntimeError('夹取结果已过期')
            result=client.session.completed
            if any(client.session.status.get(k)!=self.competition_pick_identity[k] for k in ('backend_id','client_id','request_id')):
                raise RuntimeError('抓取后端或任务发生变化')
            motion=result.get('motion_result') or {}
            retryable=(result.get('state')=='failed' and result.get('accepted') is False
                and result.get('arm_released') is True and result.get('holding') is False
                and str(result.get('reason','')).startswith('visual centering did not converge:')
                and isinstance(motion,dict) and motion.get('hold') is None
                and motion.get('tracking') is None and motion.get('selection') is None)
            if retryable and self.competition_pick_retries < 1 and now+10 < task.deadline:
                client.confirm_empty()
                self.competition_pick_retries+=1
                self.competition_pick_retry_wait=True
                self._workspace_journal({'kind':'competition_pick_retry','task_id':task.task_id,
                    'item':task.completed+1,'attempt':self.competition_pick_retries+1,
                    'reason':result['reason'],'old_request_id':result['request_id']})
                return
            task.picked(result,self.competition_pick_identity,now)

    def _competition_action(self, task, now):
        session=self.competition_io.session
        if self.competition_operation is None:
            if not session.ready(now):return
            self.competition_io.start(task.stage,task.context(),task.action_fields())
            self.competition_operation=task.stage
            self._workspace_journal({'kind':'competition_action_request',**session.pending})
        elif session.result:
            result=session.result;session.result=None
            if not result['accepted']:raise RuntimeError('后端失败或已取消：'+str(result.get('reason','未完成')))
            action={'classify':task.classified,'transport':task.transported,'place':task.placed}[self.competition_operation]
            action(result,now);self.competition_operation=None
            if task.stage=='reset_pick':self._save_competition_checkpoint(task)
            self._workspace_journal({'kind':'competition_action_result',**result})
        elif session.cancelled:raise RuntimeError('比赛后端失联或操作超时，等待停止回执')

    def _poll_competition(self):
        if not hasattr(self,'competition_start_button'):return
        task=self.competition;now=time.monotonic()
        try:
            capture=getattr(self,'competition_capture_process',None)
            if capture and capture.poll() is not None:
                self.competition_capture_process=None
                self.competition_status_var.set(('实际姿态已保存：'+str(self.competition_capture_output)) if capture.returncode==0 else
                    '姿态采集未通过，请查看：'+str(self.competition_capture_log))
            if self.competition_io:
                self.competition_io.poll();self.competition_pick.poll();self.competition_vision.poll()
            self._poll_competition_start(now)
            task=self.competition
            active=task and task.stage not in task.TERMINAL
            if task:self.competition_io.heartbeat(task,active)
            if not active:
                # Pending startup owns readiness checks. Repeating graph/vision
                # probes here blocks the GUI while its init arm job is active.
                if self.competition_io and not self.competition_start_pending:
                    self._competition_readiness()
                self.competition_start_button.configure(state='normal' if not self._workspace_busy() else 'disabled')
            else:
                self.competition_start_button.configure(state='disabled')
                task.tick(now)
                if task.stage=='failed':raise RuntimeError(task.reason)
                if self._workspace_snapshot()!=self.competition_identity:raise RuntimeError('比赛期间地图、点位或配置改变')
                if not self._competition_inputs_ready(task,now):return
                if (not self.competition_io.session.fresh(now) or self.competition_io.session.health['backend_id']!=self.competition_backend
                        or self.competition_io.session.health.get('catalog')!=self.competition_catalog):raise RuntimeError('比赛适配后端或动作标定已变化/失联')
                if task.stage.startswith('nav_'):self._competition_navigate(task,now)
                else:
                    if not self.node.zero_velocity_hold_ok():raise RuntimeError('停车阶段检测到底盘非零输出')
                    if task.stage=='recognition':self._competition_recognize(task,now)
                    elif task.stage=='pick':self._competition_pick_once(task,now)
                    elif task.stage in {'classify','transport','place'}:self._competition_action(task,now)
                    elif task.stage=='reset_pick':
                        if not self.competition_reset_sent:
                            self.competition_pick.confirm_empty();self.competition_reset_sent=True
                        elif self.competition_pick.session.ready(now):
                            task.reset_done(now);self.competition_reset_sent=False;self.competition_pick_identity=None
                            self.competition_pick_retries=0;self.competition_pick_retry_wait=False
                            if task.stage=='complete':
                                self._competition_checkpoint_path().unlink(missing_ok=True)
                                self._lock_manual_for_task()
            if task:
                elapsed=task.elapsed(now);self.competition_timer_var.set(f'整轮用时 {int(elapsed//60):02}:{elapsed%60:04.1f}')
                self.competition_count_var.set(f'完成 {task.completed} / {len(task.config["pick_order"])} · A园区 {task.counts["A"]}/4 · B园区 {task.counts["B"]}/4'
                    +(f'\n人工介入 {task.manual_interventions} 次' if task.manual_interventions else ''))
                detail=task.reason or str(task.target)
                progress=self.competition_io.session.progress
                if self.competition_operation and progress:detail=str(progress.get('stage') or detail)[:160]
                self.competition_status_var.set(LABELS[task.stage]+' · '+detail)
                if self.competition_last_stage!=task.stage:
                    self._workspace_journal({'kind':'competition_stage','stage':task.stage,'reason':task.reason,'elapsed_s':elapsed,'counts':task.counts,
                        'manual_interventions':task.manual_interventions,'fully_automatic':task.fully_automatic,'confirmations':task.confirmations,**task.context()})
                    self.competition_last_stage=task.stage
        except (ValueError,RuntimeError,OSError,KeyError,TypeError) as exc:
            self._cancel_competition_init(str(exc))
            self.competition_start_pending=None
            self.competition_vision_probe=None
            if getattr(self,'competition_vision',None):self.competition_vision.cancel()
            self.competition_status_var.set(str(exc));self.competition_start_button.configure(state='disabled')
            if task and task.stage not in {'complete','cancelled'}:
                try:
                    self._workspace_journal({'kind':'competition_failure','stage':task.stage,
                        'reason':str(exc),**task.context()})
                finally:
                    task.stop(str(exc),now);self.stop_all_motion()
