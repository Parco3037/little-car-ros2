"""Explicit empty-frame fallback; stored detections are never called a new sighting."""
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import time

from .vision_decision import decide_markers


def mapping_evidence_valid(mapping):
    count=mapping.get('confirmed_frames')
    if type(count) is not int or count < 0:return False
    if count >= 1:return True
    return (mapping.get('mapping_source')=='last_valid_detection'
            and mapping.get('current_frame_detected') is False
            and mapping.get('label_to_park') in ({'1':'A','2':'B'},{'1':'B','2':'A'})
            and decide_markers(mapping.get('source_payload'))['label_to_park']==mapping['label_to_park'])


class MarkerCache:
    def __init__(self, path, boot_id, camera_identity):
        self.path, self.boot_id, self.camera_identity = Path(path), boot_id, camera_identity

    def observe(self, payload):
        decision = decide_markers(payload)
        if decision['label_to_park'] is None:
            return
        pair = [decision['left'], decision['right']]
        packet = dict(schema='raicom.marker_cache/v1', boot_id=self.boot_id,
                      camera_identity=self.camera_identity, observed_at_unix=time.time(),
                      payload=dict(status='success', image_size=payload['image_size'], detections=[
                          dict(**{'class':d['label']}, score=d['confidence'], bbox=d['bbox']) for d in pair]))
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_name(self.path.name + f'.{os.getpid()}.tmp')
        temporary.write_text(json.dumps(packet), encoding='utf-8')
        temporary.replace(self.path)

    def fallback(self, payload):
        if (payload.get('status') != 'success' or payload.get('detections') != []):
            return None
        try:
            saved = json.loads(self.path.read_text(encoding='utf-8'))
            if (saved.get('schema') != 'raicom.marker_cache/v1' or saved.get('boot_id') != self.boot_id
                    or saved.get('camera_identity') != self.camera_identity
                    or saved['payload']['image_size'] != payload.get('image_size')
                    or not isinstance(saved.get('observed_at_unix'), (int,float))
                    or not math.isfinite(saved['observed_at_unix'])
                    or saved['observed_at_unix'] > time.time()
                    or decide_markers(saved['payload'])['label_to_park'] is None):
                return None
            return deepcopy(saved)
        except (OSError, ValueError, KeyError, TypeError):
            return None
