#!/usr/bin/env python3
"""Run the documented headless subset without contacting a ROS graph."""

import json
import os
from pathlib import Path
import subprocess
import sys


if __name__ == '__main__':
    root = Path(__file__).resolve().parents[1]
    suite = json.loads((root / 'tools/offline-suite.json').read_text(encoding='utf-8'))
    paths = [str(root / 'src/raicom_control'), str(root / 'src/rgbd_grasp')]
    if os.environ.get('PYTHONPATH'):
        paths.append(os.environ['PYTHONPATH'])
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join(paths),
                       PYTHONDONTWRITEBYTECODE='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    print(suite['scope'], flush=True)
    command = [sys.executable, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
               '--confcutdir', str(root), *suite['tests'], *sys.argv[1:]]
    raise SystemExit(subprocess.call(command, cwd=root, env=environment))
