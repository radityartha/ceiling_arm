import os, sys, time
import ultralytics
from ultralytics import YOLOE
img = os.path.join(os.path.dirname(ultralytics.__file__), 'assets', 'bus.jpg')
print('cwd', os.getcwd(), 'mobileclip', os.path.getsize('mobileclip_blt.ts') if os.path.exists('mobileclip_blt.ts') else None, flush=True)
t = time.time()
m = YOLOE('/home/user1/Documents/ceiling_arm/ros2_ws/yoloe-11m-seg.pt')
names = ['person', 'bus']
try:
    m.set_classes(names, m.get_text_pe(names))
except Exception as e:
    print('GAGAL set_classes/get_text_pe:', type(e).__name__, str(e)[:200]); sys.exit(2)
r = m.predict(img, device='cuda:0', conf=0.25, imgsz=512, verbose=False)[0]
cls = [names[int(c)] for c in r.boxes.cls.tolist()]
print('deteksi', {n: cls.count(n) for n in names}, 'mask', None if r.masks is None else len(r.masks), f'{time.time() - t:.1f} s')
