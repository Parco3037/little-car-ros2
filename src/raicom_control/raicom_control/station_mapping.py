"""One stopped recognition session; no stale mapping reuse between tasks."""
import time
import uuid
from .recognition_station import MarkerConsensus


class StationMapping:
    def __init__(self, task_id, geometry, station, destinations, required_frames=3, allow_cached=False):
        if set(destinations)!={'A','B'} or not all(destinations.values()) or destinations['A']==destinations['B']:
            raise ValueError('识别前先选择两个不同的 A/B 园区点')
        self.task_id,self.geometry,self.station=task_id,geometry,station
        self.destinations=dict(destinations);self.consensus=MarkerConsensus(required_frames);self.result=None
        self.allow_cached=allow_cached

    def observe(self, reply, *, stopped):
        if self.result is not None:return self.result
        payload=reply.get('payload',{})
        mapping=self.consensus.observe(payload,stopped=stopped)
        reused=None
        if mapping is None and self.allow_cached and stopped is True and payload.get('status')=='success' and payload.get('detections')==[]:
            cached=reply.get('marker_fallback')
            if isinstance(cached,dict) and cached.get('schema')=='raicom.marker_cache/v1':
                original=cached.get('payload',{})
                if original.get('image_size')==payload.get('image_size'):
                    from .vision_decision import decide_markers
                    from .recognition_station import parse_markers
                    try:mapping=parse_markers(original)
                    except ValueError:pass
                    else:
                        reused=cached
                        self.consensus.last_decision=decide_markers(original)
                        self.consensus.last_decision['reason']='当前帧无检测框，沿用上次识别'
        if mapping is None:return None
        categories={category:'A' if park=='pick_A' else 'B' for category,park in mapping.items()}
        self.result={'schema':'raicom_station_mapping/v1','valid':True,'task_id':self.task_id,
                     'mapping_id':uuid.uuid4().hex,'geometry_identity':self.geometry,'station':self.station,
                     'category_to_park':categories,'category_to_point':{k:self.destinations[v] for k,v in categories.items()},
                     'label_to_park':{label:categories[category] for label,category in [('1','T'),('2','G')]},
                     'confirmed_frames':self.consensus.count,'at_unix':time.time(),'scope':'this_task_only'}
        if reused is not None:
            self.result.update(mapping_source='last_valid_detection',current_frame_detected=False,
                               source_observed_at_unix=reused.get('observed_at_unix'),source_payload=reused['payload'])
        return self.result
