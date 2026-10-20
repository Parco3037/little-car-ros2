"""Existing HTTP YOLO adapter and conservative left/right T/G consensus."""
import json
import math
import time
import urllib.request

from .vision_decision import decide_markers

LABELS = {'1': 'T', '2': 'G'}


def parse_markers(payload):
    decision=decide_markers(payload)
    mapping=decision['label_to_park']
    if mapping is None:raise ValueError(decision['reason'])
    return {LABELS[label]:'pick_A' if park=='A' else 'pick_B' for label,park in mapping.items()}


class MarkerConsensus:
    def __init__(self, required_frames=3):
        if type(required_frames)is not int or required_frames not in (1,3):
            raise ValueError('识别次数必须是一次有效或三次一致')
        self.required_frames=required_frames
        self.mapping=None;self.candidate=None;self.count=0;self.last_decision=decide_markers({})

    def observe(self, payload, *, stopped):
        if self.mapping is not None:return dict(self.mapping)
        self.last_decision=decide_markers(payload)
        try:
            if stopped is not True:raise ValueError('Recognition requires a stopped robot')
            mapping=parse_markers(payload)
        except ValueError:
            self.candidate=None;self.count=0
            return None
        self.count=self.count+1 if mapping==self.candidate else 1
        self.candidate=mapping
        if self.count>=self.required_frames:self.mapping=dict(mapping)
        return self.mapping


def infer_once(timeout=5.0):
    """One no-body POST; preserves containers/models/parameters unchanged."""
    started=time.monotonic()
    request=urllib.request.Request('http://127.0.0.1:8080/api/do_infer',method='POST')
    with urllib.request.urlopen(request,timeout=timeout) as response:
        raw=response.read(2_000_001)
        if len(raw)>2_000_000:raise ValueError('Oversized YOLO reply')
        payload=json.loads(raw)
    return {'started_monotonic':started,'finished_monotonic':time.monotonic(),'payload':payload}
