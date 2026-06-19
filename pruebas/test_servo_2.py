#!/usr/bin/env python3
import time
import sys
import termios
import tty
import board
import busio
from adafruit_pca9685 import PCA9685



# ====== CONFIG ======
SERVO_CH = 0
ESC_CH   = 1

# PWM en microsegundos
SERVO_LEFT   = 1800
SERVO_CENTER = 1500
SERVO_RIGHT  = 1200

ESC_NEUTRAL  = 1600   # Mantenerse parado
ESC_FWD      = 1800  
ESC_REV      = 1400
# ====================

def pwm_us_to_duty(us: int) -> int:
    # 50 Hz => periodo 20 ms = 20000 us
    return int(us * 65535 / 20000)

def getch() -> str:
    """Lee 1 tecla sin Enter"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def set_channel(channel, us: int):
    channel.duty_cycle = pwm_us_to_duty(us)

def main():
    # I2C + PCA9685
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    servo = pca.channels[SERVO_CH]
    esc   = pca.channels[ESC_CH]

    # Seguridad: centro + neutro
    set_channel(servo, SERVO_CENTER)
    set_channel(esc, ESC_NEUTRAL)
    time.sleep(2)

    print("\nControles:")
    print("  W: avanzar")
    print("  S: Marcha atras")
    print("  A/D: girar izq/der")
    print("  C: centrar dirección")
    print("  Q: salir\n")

    try:
        while True:
            k = getch().lower()

            if k == "w":
                set_channel(esc, ESC_FWD)
                print("AVANZAR 0.1s", flush=True)
                set_channel(esc, ESC_FWD)
                time.sleep(0.2)
                set_channel(esc, ESC_NEUTRAL)
            elif k == "s":
                print("ATRÁS 0.2s", flush=True)
                set_channel(esc, ESC_REV)
                time.sleep(0.2)
                set_channel(esc, ESC_NEUTRAL)
            elif k == "a":
                set_channel(servo, SERVO_LEFT)
                print("IZQ", flush=True)
            elif k == "d":
                set_channel(servo, SERVO_RIGHT)
                print("DER", flush=True)
            elif k == "c":
                set_channel(servo, SERVO_CENTER)
                print("CENTRO", flush=True)
            elif k == "q":
                print("SALIENDO...", flush=True)
                break

    finally:
        # Dejar todo neutro al salir
        set_channel(esc, ESC_NEUTRAL)
        set_channel(servo, SERVO_CENTER)
        time.sleep(0.2)
        pca.deinit()

if __name__ == "__main__":
    main()
