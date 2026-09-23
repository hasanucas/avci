import sys, time, numpy as np
from ultralytics import YOLO
eng, imgsz = sys.argv[1], int(sys.argv[2])
m = YOLO(eng, task="detect")
imgs = [np.random.randint(0, 255, (594, 1056, 3), np.uint8) for _ in range(4)]  # gercek 2x2 tile boyutu
for _ in range(5): m.predict(imgs, imgsz=imgsz, verbose=False)
t0 = time.time(); N = 50
for _ in range(N): m.predict(imgs, imgsz=imgsz, verbose=False)
print(f"{eng} @ imgsz={imgsz}: {(time.time()-t0)/N*1000:.1f} ms (4 tile)")
