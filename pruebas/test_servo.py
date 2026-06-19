import board
import busio
from adafruit_pca9685 import PCA9685
import sys
import tty
import termios


# ---------- lectura de tecla sin ENTER ----------
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
# ------------------------------------------------


# Inicializar I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Inicializar PCA9685
pca = PCA9685(i2c)
pca.frequency = 50

# Valores iniciales (ajustaremos después)
LEFT = 6000
CENTER = 5000
RIGHT = 4000

print("Control servo:")
print("A = izquierda")
print("D = derecha")
print("S = centro")
print("Q = salir")

while True:
    key = getch().lower()

    if key == "a":
        pca.channels[0].duty_cycle = LEFT
        print("Izquierda")

    elif key == "d":
        pca.channels[0].duty_cycle = RIGHT
        print("Derecha")

    elif key == "s":
        pca.channels[0].duty_cycle = CENTER
        print("Centro")

    elif key == "q":
        break

print("Fin")
