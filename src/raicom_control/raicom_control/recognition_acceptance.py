"""Record exactly ten known-object frames without changing the YOLO service."""
import argparse
import json
from pathlib import Path
from .recognition_station import LABELS,infer_once,parse_markers


def classification(payload):
    # Reuse strict bbox/score validation by pairing the sole object with a
    # synthetic marker is deliberately avoided: a frame stands on its own.
    import math
    if not isinstance(payload,dict) or payload.get('status')!='success':return None
    size=payload.get('image_size');detections=payload.get('detections')
    if not isinstance(size,list) or len(size)!=2 or any(type(x) is not int or x<=0 for x in size):return None
    if not isinstance(detections,list):return None
    classes=[]
    for d in detections:
        if not isinstance(d,dict):return None
        if d.get('class')=='blue':continue
        box=d.get('bbox');score=d.get('score')
        if (d.get('class') not in LABELS or type(score) not in (int,float) or not math.isfinite(score)
                or not .4<score<=1 or not isinstance(box,list) or len(box)!=4
                or any(type(x) not in (int,float) or not math.isfinite(x) for x in box)):return None
        if not (0<=box[0]<box[2]<=size[0] and 0<=box[1]<box[3]<=size[1]):return None
        classes.append(LABELS[d['class']])
    return classes[0] if len(classes)==1 else None


def summarize(records):
    valid=[r for r in records if r['valid']]
    return {'frames':len(records),'valid_frames':len(valid),'correct_valid_frames':sum(r['correct'] for r in valid),
            'passed':len(records)==10 and len(valid)>=9 and all(r['correct'] for r in valid)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--known',choices=['T','G']);group.add_argument('--left',choices=['T','G'])
    p.add_argument('--output',required=True,type=Path);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    records=[]
    with a.output.open('x',encoding='utf-8') as f:
        for i in range(10):
            try:
                reply=infer_once()
                value=classification(reply['payload']) if a.known else parse_markers(reply['payload'])
                valid=value is not None
                correct=value==a.known if a.known else value.get(a.left)=='pick_A'
                record={'index':i+1,'valid':valid,'correct':bool(valid and correct),'observed':value,**reply}
            except Exception as exc:record={'index':i+1,'valid':False,'correct':False,'error':str(exc)}
            records.append(record);f.write(json.dumps(record,ensure_ascii=False)+'\n');f.flush()
        summary=summarize(records);f.write(json.dumps({'summary':summary,'known':a.known,'left':a.left})+'\n')
    print(json.dumps(summary))
    if not summary['passed']:raise SystemExit(2)


if __name__=='__main__':main()
