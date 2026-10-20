"""Recoverable map removal. Routes/history stay at their original paths."""
import hashlib
import json
from pathlib import Path
import shutil
from datetime import datetime, timezone
from uuid import uuid4
import yaml


def _inside(path, roots):
    path = Path(path).resolve()
    if not any(path.is_relative_to(Path(root).resolve()) for root in roots):
        raise ValueError('地图路径不在允许的地图目录内')
    return path


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _copy_new(source, dest):
    # Never overwrite an existing map, even if it appeared after the precheck.
    with Path(source).open('rb') as src, Path(dest).open('xb') as dst:
        try:
            shutil.copyfileobj(src, dst)
        except Exception:
            dst.close()
            Path(dest).unlink()
            raise


def archive_map(map_file, roots, trash_root, route_file=None):
    source = _inside(map_file, roots)
    if source.suffix != '.yaml' or source.is_symlink():
        raise ValueError('请选择有效的地图 YAML')
    data = yaml.safe_load(source.read_text(encoding='utf-8'))
    image = _inside(source.parent / data['image'], roots)
    if not image.is_file() or image == source:
        raise ValueError('地图图像不存在或路径无效')
    shared = False
    for root in roots:
        for other in Path(root).glob('*.yaml'):
            if other.resolve() == source:
                continue
            try:
                ref = yaml.safe_load(other.read_text(encoding='utf-8'))['image']
                shared |= (other.parent / ref).resolve() == image
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
                # Unknown references: retain the image conservatively.
                shared = True
    entry = Path(trash_root) / (datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S_') + uuid4().hex[:8])
    entry.mkdir(parents=True, exist_ok=False)
    records = []
    for name, original, remove in [('map.yaml', source, True), ('map_image', image, not shared)]:
        _copy_new(original, entry / name)
        if _sha(entry / name) != _sha(original):
            raise RuntimeError('复制时地图发生变化，未删除原件')
        records.append(dict(backup=name, original=str(original), sha256=_sha(original), remove=remove))
    if route_file and Path(route_file).is_file():
        _copy_new(route_file, entry / 'route_backup.json')
    manifest = dict(schema='raicom_map_trash/v1', name=source.name, files=records,
                    route_preserved=str(route_file) if route_file else None)
    (entry / 'manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    removed = []
    try:
        for record in records:
            if record['remove']:
                p = Path(record['original'])
                if _sha(p) != record['sha256']:
                    raise RuntimeError('地图在删除前发生变化，已中止')
                p.unlink()
                removed.append(record)
    except Exception:
        for record in removed:
            _copy_new(entry / record['backup'], record['original'])
        raise
    return entry


def trash_entries(trash_root):
    return sorted(Path(trash_root).glob('*/manifest.json'), reverse=True)


def restore_map(manifest_path, roots, trash_root):
    manifest_path = _inside(manifest_path, [trash_root])
    data = json.loads(manifest_path.read_text(encoding='utf-8'))
    if data.get('schema') != 'raicom_map_trash/v1' or len(data['files']) != 2:
        raise ValueError('地图回收记录无效')
    pending = []
    for record in data['files']:
        dest = _inside(record['original'], roots)
        backup = _inside(manifest_path.parent / record['backup'], [manifest_path.parent])
        if _sha(backup) != record['sha256']:
            raise ValueError('回收站备份校验失败')
        if dest.exists():
            if record['remove'] or _sha(dest) != record['sha256']:
                raise ValueError('原路径已有文件，未覆盖：' + dest.name)
        else:
            pending.append((backup, dest))
    written = []
    try:
        for backup, dest in reversed(pending):  # Image before YAML becomes discoverable.
            dest.parent.mkdir(parents=True, exist_ok=True)
            _copy_new(backup, dest)
            written.append(dest)
    except Exception:
        for path in written:
            path.unlink()
        raise
    manifest_path.rename(manifest_path.with_name('restored.json'))
    return Path(data['files'][0]['original'])
