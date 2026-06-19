#!/usr/bin/env python3
"""
Nodo ROS 2 de control manual por teclado.
Publica mensajes Twist en /cmd_vel según las teclas pulsadas.
Requiere que cmdvel_to_pca9685 esté en ejecución.

Teclas: W=avanzar  S=atrás  A=izquierda  D=derecha  C=centro  Q=salir
"""
import sys
import termios
import tty
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControlTeclado(Node):
    def __init__(self):
        super().__init__('control_teclado')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Estado actual del comando (protegido por lock para acceso desde hilo)
        self._v = 0.0
        self._w = 0.0
        self._lock = threading.Lock()

        # Publica el último comando a 10 Hz para mantener vivo el timeout
        # del nodo cmdvel_to_pca9685 mientras se mantiene pulsada una tecla
        self.create_timer(0.1, self._republish)

        # Hilo de lectura de teclado: no bloquea el spin de ROS 2
        self._hilo = threading.Thread(target=self._loop_teclado, daemon=True)
        self._hilo.start()

        self.get_logger().info(
            'Control teclado listo — '
            'W=avanzar  S=atrás  A=izq  D=der  C=centro  Q=salir'
        )

    # ── Publicación periódica ─────────────────────────────────────────────────

    def _republish(self):
        """Republica el último comando para evitar el timeout del ESC."""
        with self._lock:
            msg = Twist()
            msg.linear.x  = self._v
            msg.angular.z = self._w
        self.pub.publish(msg)

    # ── Gestión de estado ─────────────────────────────────────────────────────

    def _set(self, v=None, w=None):
        with self._lock:
            if v is not None:
                self._v = float(v)
            if w is not None:
                self._w = float(w)

    # ── Lectura de teclado ────────────────────────────────────────────────────

    @staticmethod
    def _getch() -> str:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def _loop_teclado(self):
        """Hilo de fondo: lee teclas y actualiza el estado del comando."""
        try:
            while rclpy.ok():
                k = self._getch().lower()

                if k == 'w':
                    # Pulso de avance: 0.3 s y vuelve a neutro
                    # No se modifica w para conservar el giro activo
                    self._set(v=0.5)
                    self.get_logger().info('AVANZAR')
                    time.sleep(0.3)
                    self._set(v=0.0)

                elif k == 's':
                    # Pulso de marcha atrás: 0.3 s y vuelve a neutro
                    # No se modifica w para conservar el giro activo
                    self._set(v=-0.5)
                    self.get_logger().info('ATRAS')
                    time.sleep(0.3)
                    self._set(v=0.0)

                elif k == 'a':
                    # Dirección izquierda (se mantiene hasta siguiente tecla)
                    self._set(w=1.0)
                    self.get_logger().info('IZQ')

                elif k == 'd':
                    # Dirección derecha (se mantiene hasta siguiente tecla)
                    self._set(w=-1.0)
                    self.get_logger().info('DER')

                elif k == 'c':
                    # Centro: para todo
                    self._set(v=0.0, w=0.0)
                    self.get_logger().info('CENTRO')

                elif k == 'q':
                    self._set(v=0.0, w=0.0)
                    self.get_logger().info('Saliendo...')
                    rclpy.shutdown()
                    break

        except Exception as exc:
            self.get_logger().error(f'Error en hilo de teclado: {exc}')

    def destroy_node(self):
        # Aseguramos parada completa antes de cerrar
        self._set(v=0.0, w=0.0)
        self._republish()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlTeclado()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
