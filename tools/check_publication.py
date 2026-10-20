#!/usr/bin/env python3
"""Check publication packaging without importing ROS or starting any node."""

import ast
import hashlib
import json
import os
from pathlib import Path
import re
import runpy
import sys
from types import ModuleType
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'


def check() -> dict:
    files = [p for p in ROOT.rglob('*') if p.is_file() and '.git' not in p.parts]
    for path in files:
        assert not any(x in path.parts for x in ('__pycache__', '.pytest_cache')), path
        if path.suffix == '.py':
            ast.parse(path.read_text(encoding='utf-8-sig'), filename=str(path))
        elif path.suffix == '.json':
            json.loads(path.read_text(encoding='utf-8'))

    packages = list(SRC.rglob('package.xml'))
    assert len(packages) == 4, f'Expected four ROS packages, found {len(packages)}'
    for path in packages:
        xml = ET.parse(path).getroot()
        assert xml.findtext('license') == 'Apache-2.0', path
        assert (path.parent / 'LICENSE').is_file(), path

    # Execute packaging declarations with a stub, never installing or importing nodes.
    entries = []
    old_setuptools = sys.modules.get('setuptools')
    old_cwd = Path.cwd()
    stub = ModuleType('setuptools')
    stub.find_packages = lambda **kwargs: []
    captured = {}
    stub.setup = lambda **kwargs: captured.update(kwargs)
    try:
        sys.modules['setuptools'] = stub
        for package in ('rgbd_grasp', 'competition_manual', 'raicom_control'):
            directory = SRC / package
            captured.clear()
            os.chdir(directory)
            runpy.run_path(str(directory / 'setup.py'), run_name='publication_metadata')
            assert captured['license'] == 'Apache-2.0', package
            for _, sources in captured['data_files']:
                for relative in sources:
                    assert (directory / relative).is_file(), (package, relative)
            for entry in captured.get('entry_points', {}).get('console_scripts', []):
                name, target = entry.split('=', 1)
                module, function = target.strip().split(':', 1)
                path = directory.joinpath(*module.split('.')).with_suffix('.py')
                tree = ast.parse(path.read_text(encoding='utf-8'))
                assert any(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                           and n.name == function for n in tree.body), entry
                entries.append(name.strip())
    finally:
        os.chdir(old_cwd)
        if old_setuptools is None:
            sys.modules.pop('setuptools', None)
        else:
            sys.modules['setuptools'] = old_setuptools

    # Inspect release constants without importing the runtime release verifier.
    release = SRC / 'raicom_control/raicom_control/depth_pick_release.py'
    tree = ast.parse(release.read_text(encoding='utf-8'))
    hashes = next(ast.literal_eval(node.value) for node in tree.body
                  if isinstance(node, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'HASHES' for t in node.targets))
    for name, expected in hashes.items():
        actual = hashlib.sha256((SRC / 'rgbd_grasp/rgbd_grasp' / name).read_bytes()).hexdigest()
        assert actual == expected, f'Fixed runtime hash mismatch: {name}'
    launch = SRC / 'rgbd_grasp/launch/rgbd_grasp.launch.py'
    launch_hash = hashlib.sha256(launch.read_bytes()).hexdigest()
    assert launch_hash == 'af9f8345a944e6fb3452f291c32866b1ac416939967fb3681437fa92f1d8d16a'

    scope = json.loads((ROOT / 'docs/publication-scope.json').read_text(encoding='utf-8'))
    for excluded in scope['excluded']:
        assert not (SRC / excluded['path']).exists(), excluded['path']
    assert not any(p.suffix.lower() in ('.so', '.dll', '.dylib', '.wav', '.pgm')
                   for p in SRC.rglob('*') if p.is_file()), 'Excluded binary/local asset found'

    # A limited signature scan, not proof that all secrets or history are absent.
    patterns = [rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
                rb'\bgh[pousr]_[A-Za-z0-9]{30,}\b',
                rb'\bgithub_pat_[A-Za-z0-9_]{40,}\b',
                rb'\bAKIA[A-Z0-9]{16}\b']
    for path in files:
        data = path.read_bytes()
        assert not any(re.search(pattern, data) for pattern in patterns), (
            'Potential secret signature', str(path.relative_to(ROOT)))

    return {'files': len(files), 'python_files': sum(p.suffix == '.py' for p in files),
            'ros_packages': len(packages), 'console_entries': len(entries),
            'fixed_grasp_hashes': len(hashes) + 1,
            'bytes': sum(p.stat().st_size for p in files)}


if __name__ == '__main__':
    print(json.dumps(check(), indent=2))
