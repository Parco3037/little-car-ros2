"""Deterministic decisions from numeric YOLO boxes, never a supplied A/B string."""
import math


def decide_markers(payload, *, roi=None, expected_size=None):
    result = dict(status='unknown', label=None, confidence=None, side=None,
                  best=None, left=None, right=None, label_to_park=None, reason='', rejected=0)
    if not isinstance(payload, dict) or payload.get('status') != 'success':
        result['reason'] = '相机或识别接口未成功返回'; return result
    size, detections = payload.get('image_size'), payload.get('detections')
    if (not isinstance(size, list) or len(size) != 2 or
            any(type(v) is not int or v <= 0 for v in size) or
            (expected_size is not None and size != expected_size) or not isinstance(detections, list)):
        result['reason'] = '图像尺寸或检测列表无效'; return result
    bounds = roi or [0, 0, *size]
    if (not isinstance(bounds, list) or len(bounds) != 4 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in bounds) or
            not 0 <= bounds[0] < bounds[2] <= size[0] or not 0 <= bounds[1] < bounds[3] <= size[1]):
        result['reason'] = '识别区域无效'; return result
    valid = []
    for item in detections:
        if not isinstance(item,dict) or item.get('class') not in ('1','2'): continue
        box, score = item.get('bbox'), item.get('score')
        if (type(score) not in (int,float) or not math.isfinite(score) or not 0 < score <= 1 or
                not isinstance(box,list) or len(box)!=4 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in box) or
                not bounds[0] <= box[0] < box[2] <= bounds[2] or
                not bounds[1] <= box[1] < box[3] <= bounds[3]):
            result['rejected'] += 1; continue
        cx=(box[0]+box[2])/2
        valid.append(dict(label=item['class'],confidence=float(score),bbox=list(box),center_x=cx,
                          side='left' if cx < size[0]/2 else 'right' if cx > size[0]/2 else 'center'))
    if not valid:
        result['reason'] = '本帧没有有效1/2检测框'; return result
    # Stable tie-breaks make replay independent of the server's detection order.
    valid.sort(key=lambda d:(-d['confidence'],d['center_x'],d['label'],tuple(d['bbox'])))
    best=valid[0]
    result.update(status='selected',best=best,label=best['label'],confidence=best['confidence'],side=best['side'])
    winners={}
    for detection in valid:winners.setdefault(detection['label'],detection)
    if len(winners)==2:
        left,right=sorted(winners.values(),key=lambda d:d['center_x'])
        result.update(left=left,right=right)
        if left['center_x'] < right['center_x']:
            result['label_to_park']={left['label']:'A',right['label']:'B'}
            result['mapping_source']='highest_confidence_centers'
        else:result['reason']='两个最高置信度标识中心重合，不能区分左右'
    else:
        if len(valid)>=2:
            left,right=sorted(valid[:2],key=lambda d:d['center_x'])
            result.update(left=left,right=right)
            result['reason']='已按最高置信度两个框判定左右；两个框类别相同，尚无完整1/2映射'
        else:result['reason']='已判定本帧最高置信度类别，另一标识尚未识别'
    return result


def decide_held_class(payload):
    """Classify the held block from YOLO scores when no usable box remains.

    A class score can select T/G for the current item, but has no image
    position and must never establish the left/right station mapping.
    """
    if not isinstance(payload, dict) or payload.get('status') != 'success':
        return None
    detections = payload.get('detections')
    if not isinstance(detections, list):
        return None
    candidates = []
    for item in detections:
        if not isinstance(item, dict) or item.get('class') not in ('1', '2'):
            continue
        score = item.get('score')
        if type(score) in (int, float) and math.isfinite(score) and 0 < score <= 1:
            candidates.append((float(score), item['class']))
    if not candidates:
        return None
    score, label = min(candidates, key=lambda item: (-item[0], item[1]))
    return dict(label=label, confidence=score, source='highest_yolo_confidence_without_box')


def decision_text(result):
    if result['label'] is None:return result['reason']
    sides={'left':'画面左侧','right':'画面右侧','center':'画面中央'}
    text=f"本帧 {result['label']} · {result['confidence']:.0%} · {sides[result['side']]}"
    if result['label_to_park']:
        text+=f"；左 {result['left']['label']} → A，右 {result['right']['label']} → B"
    elif result['reason']:text+='；'+result['reason']
    return text
