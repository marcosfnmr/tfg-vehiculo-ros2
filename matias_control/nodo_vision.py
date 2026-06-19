#!/usr/bin/env python3
"""
Nodo de seguimiento de objeto por color (rojo).
Detecta el objeto más grande de color rojo en la imagen y publica
consignas para mantenerlo centrado y frenar al aproximarse.
"""
import subprocess
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# El rojo en HSV envuelve alrededor de H=0/180, por eso necesitamos
# dos rangos: uno para la parte baja (0-10) y otro para la alta (170-180)
LIMITE_BAJO  = np.array([0,   120, 80])
LIMITE_ALTO  = np.array([10,  255, 255])
LIMITE_BAJO2 = np.array([170, 120, 80])
LIMITE_ALTO2 = np.array([180, 255, 255])

VELOCIDAD_AVANCE = 0.25
GANANCIA_GIRO    = 1.4
MIN_AREA         = 500      # área mínima para considerar que hay objeto
MAX_AREA         = 25000    # área máxima: si se supera, el objeto está demasiado cerca
DEBUG            = False


class NodoVision(Node):
    def __init__(self):
        super().__init__('nodo_vision')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Usamos MJPEG en lugar de YUV420 porque este nodo necesita
        # la menor latencia posible: MJPEG permite extraer frames
        # completos por sus marcadores de inicio/fin sin conocer
        # el tamaño exacto de cada frame de antemano.
        self.proc = subprocess.Popen(
            ['rpicam-vid', '-t', '0',
             '--width', '320', '--height', '240',
             '--codec', 'mjpeg', '--inline', '-o', '-'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )
        self.buf = b''

        # Hilo de fondo: lee el pipe y guarda siempre el frame mas reciente
        self._ultimo_frame = None
        self._lock  = threading.Lock()
        self._hilo  = threading.Thread(target=self._lector, daemon=True)
        self._hilo.start()

        self.contador_debug = 0
        self.timer = self.create_timer(0.05, self.procesar_frame)
        self.get_logger().info('Nodo de vision iniciado')

    def _lector(self):
        """Hilo de fondo: consume el pipe MJPEG sin parar."""
        while True:
            chunk = self.proc.stdout.read(4096)
            if not chunk:
                break
            self.buf += chunk
            end = self.buf.rfind(b'\xff\xd9')
            if end == -1:
                continue
            start = self.buf.rfind(b'\xff\xd8', 0, end)
            if start == -1:
                continue
            jpg      = self.buf[start:end + 2]
            self.buf = self.buf[end + 2:]
            frame    = cv2.imdecode(
                np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR
            )
            if frame is not None:
                frame = cv2.flip(frame, -1)
                with self._lock:
                    self._ultimo_frame = frame

    def procesar_frame(self):
        # Cogemos el último frame disponible sin bloquear
        with self._lock:
            if self._ultimo_frame is None:
                return
            frame = self._ultimo_frame.copy()

        h, w      = frame.shape[:2]
        cx_imagen = w / 2.0

        hsv      = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mascara1 = cv2.inRange(hsv, LIMITE_BAJO,  LIMITE_ALTO)
        mascara2 = cv2.inRange(hsv, LIMITE_BAJO2, LIMITE_ALTO2)
        mascara  = cv2.bitwise_or(mascara1, mascara2)

        # Apertura para eliminar ruido puntual +
        # cierre para unir partes del objeto que puedan quedar fragmentadas
        kernel  = np.ones((5, 5), np.uint8)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN,  kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)

        if DEBUG:
            self.contador_debug += 1
            if self.contador_debug % 20 == 0:
                cv2.imwrite('/tmp/vision_frame.jpg', frame)
                cv2.imwrite('/tmp/vision_mask.jpg',  mascara)

        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        twist = Twist()

        if contornos:
            c    = max(contornos, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > MAX_AREA:
                self.get_logger().info(
                    f'DEMASIADO CERCA (area={area:.0f}) -> FRENAZO'
                )
                twist.linear.x = -0.25
                self.pub.publish(twist)
                return

            if area > MIN_AREA:
                # El objeto esta demasiado cerca y ademas de parar acelero negativamente
                M               = cv2.moments(c)
                cx              = M['m10'] / M['m00']
                error           = (cx - cx_imagen) / cx_imagen
                twist.linear.x  = VELOCIDAD_AVANCE
                twist.angular.z = -GANANCIA_GIRO * error
                self.get_logger().info(
                    f'Objeto area={area:.0f} cx={cx:.0f} '
                    f'error={error:.2f} w={twist.angular.z:.2f}'
                )
            else:
                # Objeto detectado pero muy pequeño: giramos para buscarlo
                self.get_logger().info('Objeto muy pequeno -> buscando')
                twist.angular.z = 0.4
        else:
            # Sin detección: giramos sobre el sitio para localizar el objeto
            self.get_logger().info('Sin objeto -> buscando')
            twist.angular.z = 0.4

        self.pub.publish(twist)

    def destroy_node(self):
        twist = Twist()
        self.pub.publish(twist)
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        super().destroy_node()


def main():
    rclpy.init()
    node = NodoVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
