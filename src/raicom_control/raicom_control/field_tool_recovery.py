"""One editable recovery budget for the workbench and its per-goal Nav2 tree."""
import hashlib
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

DEFAULT_MAX_RECOVERIES = 10
DEFAULT_TREE = Path('/home/ubuntu/ros2_ws/src/navigation/config/behavior_trees/navigate_to_pose_without_backup.xml')
TREE_CACHE = Path('/home/ubuntu/raicom_runtime/navigation_bt')


def recovery_limit(value=DEFAULT_MAX_RECOVERIES):
    # The Humble tree and recovery feedback use signed integer counters.
    if type(value) is not int or not 0 <= value <= 2147483646:
        raise ValueError('恢复次数须为非负整数，且不超过2147483646')
    return value


def recovery_tree_xml(source, limit):
    limit = recovery_limit(limit)
    root = ET.fromstring(source)
    nodes = root.findall("./BehaviorTree/RecoveryNode[@name='NavigateRecovery']")
    if len(nodes) != 1 or len(nodes[0]) != 2:
        raise ValueError('导航恢复树结构不匹配，请核对当前导航配置')
    # Keep the original planner, controller and recovery actions. Give the
    # outer tree enough rounds to let the feedback counter enforce > limit;
    # the vendor's fixed six rounds must not preempt an operator's larger cap.
    nodes[0].set('number_of_retries', str(limit + 1))
    return ET.tostring(root, encoding='utf-8', xml_declaration=True)


def materialize_recovery_tree(limit, *, template=DEFAULT_TREE, cache_dir=TREE_CACHE):
    data = recovery_tree_xml(Path(template).read_bytes(), limit)
    cache_dir = Path(cache_dir); cache_dir.mkdir(parents=True, exist_ok=True)
    path = cache_dir / ('recoveries_' + str(limit) + '_' + hashlib.sha256(data).hexdigest()[:16] + '.xml')
    if path.exists() and path.read_bytes() == data:
        return str(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=cache_dir, prefix='.recovery_', delete=False) as stream:
            temporary = Path(stream.name); stream.write(data)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists(): temporary.unlink()
    return str(path)
