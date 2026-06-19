# test_cam.py
import cv2
import subprocess

# Configurar formato via v4l2 antes de abrir
subprocess.run([
    "v4l2-ctl", "--device=/dev/video0",
    "--set-fmt-video=width=640,height=480,pixelformat=YUYV"
])

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f"Abierto: {cap.isOpened()}")
print(f"Ancho: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
print(f"Alto: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")

ret, frame = cap.read()
if ret:
    cv2.imwrite("test.jpg", frame)
    print("✅ Foto guardada en test.jpg")
else:
    print("❌ Error al leer frame")
cap.release()
