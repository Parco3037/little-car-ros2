"""Tk-only, request-matched detection preview. Never adopts a delivery mapping."""
from io import BytesIO
import math
import time
import tkinter as tk
from tkinter import ttk

from .recognition_station import LABELS, parse_markers
from .vision_decision import decide_markers, decision_text


def annotations(payload):
    size=payload.get('image_size')
    if (not isinstance(size,list) or len(size)!=2 or
            any(type(v) is not int or not 0<v<=4096 for v in size)):
        raise ValueError('图像尺寸无效，无法对齐检测框')
    detections=payload.get('detections')
    if not isinstance(detections,list) or len(detections)>256:raise ValueError('检测结果无效')
    try:mapping=parse_markers(payload)
    except ValueError:mapping={}
    boxes=[]
    for item in detections:
        if not isinstance(item,dict):continue
        box,score=item.get('bbox'),item.get('score')
        if (not isinstance(box,list) or len(box)!=4 or
                any(type(v) not in (int,float) or not math.isfinite(v) for v in box) or
                type(score) not in (int,float) or not math.isfinite(score) or not 0<=score<=1):continue
        x1,y1,x2,y2=box
        if not 0<=x1<x2<=size[0] or not 0<=y1<y2<=size[1]:continue
        raw=str(item.get('class','?'))
        category=LABELS.get(raw,'blue' if raw=='blue' else '未知')
        park={'pick_A':'园区1 候选','pick_B':'园区2 候选'}.get(mapping.get(category),'')
        boxes.append({'bbox':box,'label':f'{category}  {score:.0%}'+(' · '+park if park else ''),
                      'color':{'T':'#2dd4bf','G':'#fbbf24'}.get(category,'#93c5fd')})
    if not detections:summary='本帧未检测到物品 · 无园区候选'
    elif mapping:
        left=next(k for k,v in mapping.items() if v=='pick_A')
        right=next(k for k,v in mapping.items() if v=='pick_B')
        summary=f'本帧候选：园区1 ← {left}    园区2 ← {right}（尚需三次一致）'
    else:summary='未得到左右各一个有效 T/G 标识 · 不建立园区对应'
    summary=decision_text(decide_markers(payload))+'\n'+summary
    return tuple(size),boxes,summary


class DetectionPreview(ttk.Frame):
    def __init__(self,parent):
        super().__init__(parent,style='Card.TFrame')
        self.canvas=tk.Canvas(self,height=255,width=420,bg='#142633',highlightthickness=0)
        self.canvas.pack(fill='x')
        self.summary=tk.StringVar(value='点击“开始画面预览”，查看检测框与类别')
        self.age=tk.StringVar(value='尚无画面')
        ttk.Label(self,textvariable=self.summary,style='Card.TLabel',wraplength=460).pack(anchor='w',pady=(5,2))
        ttk.Label(self,textvariable=self.age,style='Muted.TLabel',wraplength=460).pack(anchor='w')
        self.original=None;self.photo=None;self.boxes=[];self.received_at=None;self.clock='';self.mode=''
        self.canvas.bind('<Configure>',lambda _:self._draw())
        self.clear('等待相机识别画面')

    def clear(self,reason):
        self.original=self.photo=None;self.boxes=[];self.received_at=None
        self.summary.set(reason);self.age.set('无可用的当前检测画面');self._draw()

    def show_packet(self,packet,mode='识别区采样'):
        from PIL import Image
        try:
            size,boxes,summary=annotations(packet['reply']['payload'])
            with Image.open(BytesIO(packet['jpeg'])) as image:
                if image.format!='JPEG' or image.size!=size:raise ValueError('照片与检测尺寸不一致，未叠加旧框')
                self.original=image.convert('RGB')
            self.boxes=boxes;self.received_at=time.monotonic()
            self.clock=time.strftime('%H:%M:%S');self.mode=mode
            self.summary.set(summary);self._draw();self.tick()
        except (ValueError,OSError,KeyError,TypeError) as exc:self.clear('画面不可用：'+str(exc))

    def tick(self):
        if self.received_at is None:return
        age=max(0.,time.monotonic()-self.received_at)
        value=f'{self.mode} · {self.clock} · {age:.0f} 秒前'+(' · 旧帧，非当前实时画面' if age>5 else '')
        if self.age.get()!=value:self.age.set(value)

    def _draw(self):
        canvas=self.canvas;canvas.delete('all')
        w,h=max(1,canvas.winfo_width()),255
        if self.original is None:
            canvas.create_text(w/2,h/2,text='相机识别画面',fill='#b6c9d6',font=('TkDefaultFont',13))
            return
        from PIL import Image,ImageTk
        im=self.original.copy();im.thumbnail((w,h),Image.Resampling.LANCZOS)
        self.photo=ImageTk.PhotoImage(im,master=canvas)
        ox,oy=(w-im.width)/2,(h-im.height)/2
        canvas.create_image(ox,oy,anchor='nw',image=self.photo)
        sx,sy=im.width/self.original.width,im.height/self.original.height
        for box in self.boxes:
            x1,y1,x2,y2=box['bbox'];x1,x2=ox+x1*sx,ox+x2*sx;y1,y2=oy+y1*sy,oy+y2*sy
            canvas.create_rectangle(x1,y1,x2,y2,outline=box['color'],width=2)
            text=canvas.create_text(x1+3,max(oy+2,y1-19),anchor='nw',text=box['label'],
                                    fill=box['color'],font=('TkDefaultFont',10,'bold'))
            bounds=canvas.bbox(text)
            if bounds[2]>w:canvas.move(text,w-bounds[2]-2,0);bounds=canvas.bbox(text)
            bg=canvas.create_rectangle(bounds,fill='#142633',outline='')
            canvas.tag_lower(bg,text)
