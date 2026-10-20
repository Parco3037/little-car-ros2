"""One bounded init subprocess per explicitly requested competition start."""
import json
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid


class CompetitionInitMixin:
    def _competition_init_ready(self, token):
        job=getattr(self,'competition_init_job',None)
        if job and job['token']==token:
            process=job['process']
            if process.poll() is None:
                if time.monotonic()-job['started']>30:
                    self._cancel_competition_init('回init超时')
                    raise RuntimeError('回init超时，等待停止回执')
                return False
            if job.get('cancelled'):raise RuntimeError(job['cancelled'])
            result=json.loads(job['result'].read_text(encoding='utf-8')) if job['result'].is_file() else {}
            if process.returncode!=0 or result.get('completed') is not True:
                raise RuntimeError('比赛前回init未完成：'+str(result.get('reason','回位进程异常退出')))
            return True
        if job and job['process'].poll() is None:return False
        if self._depth_busy():raise RuntimeError('抓取尚未结束，不能回init')
        if not self.node.zero_velocity_hold_ok():return False
        root=Path(self.session_dir)/'competition_init';root.mkdir(exist_ok=True)
        result=root/(uuid.uuid4().hex+'.json')
        with result.with_suffix('.log').open('xb') as log:
            process=subprocess.Popen([sys.executable,'-B','-m','raicom_control.competition_init_runner',
                '--execute-init','--result',str(result)],stdout=log,stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,start_new_session=True)
        self.competition_init_job=dict(token=token,process=process,result=result,started=time.monotonic())
        self._workspace_journal(dict(kind='competition_init_started',pid=process.pid,result=str(result)))
        return False

    def _cancel_competition_init(self, reason):
        job=getattr(self,'competition_init_job',None)
        if job and job['process'].poll() is None:
            job['cancelled']=reason
            try:job['process'].send_signal(signal.SIGINT)
            except ProcessLookupError:pass
