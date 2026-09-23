import time, numpy as np, torch
from ultralytics import YOLO

m = YOLO("UAV-YOLOv11m.engine", task="detect")

# A) SAF TENSOR: letterbox/preprocess YOK -> engine + minimal sarmalayici + NMS
t = torch.rand(4, 3, 640, 640, device="cuda")
for _ in range(5):
    m.predict(t, verbose=False)
torch.cuda.synchronize(); t0 = time.time()
N = 50
for _ in range(N):
    m.predict(t, verbose=False)
torch.cuda.synchronize()
a = (time.time() - t0) / N * 1000

# B) NUMPY LISTE: mevcut pipeline yolu (CPU letterbox + normalize + kopyalar dahil)
imgs = [np.random.randint(0, 255, (540, 960, 3), np.uint8) for _ in range(4)]
for _ in range(5):
    m.predict(imgs, imgsz=640, verbose=False)
t0 = time.time()
for _ in range(N):
    m.predict(imgs, imgsz=640, verbose=False)
b = (time.time() - t0) / N * 1000

print(f"\nA) saf-tensor batch4 : {a:6.1f} ms   (~engine gercek hizi)")
print(f"B) numpy+preprocess  : {b:6.1f} ms   (pipeline'in gordugu)")
print(f"   preprocess/ustyapi: {b-a:6.1f} ms")
