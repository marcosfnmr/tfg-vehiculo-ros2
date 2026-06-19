# matias_control

Paquete ROS 2 para el control de un vehículo autónomo a escala (1:10) basado en un chasis RC comercial, con percepción mediante visión por computador y actuación mediante PWM sobre un controlador PCA9685.

Desarrollado como parte del Trabajo de Fin de Grado **"Vehículo autónomo a escala con ROS 2 y visión por computador"** — Grado en Ingeniería Informática, Universidad de León.

## Hardware

| Componente | Descripción |
|---|---|
| Chasis RC | WL Toys 144010 (escala 1:10) |
| Unidad de cómputo | Raspberry Pi 5 (8 GB RAM) |
| Sistema operativo | Ubuntu 24.04 LTS |
| Middleware | ROS 2 Jazzy Jalisco |
| Controlador PWM | PCA9685 (I2C) |
| Cámara | OV5647 (interfaz CSI) |
| Alimentación | Batería LiPo 5000 mAh |

## Arquitectura

```
Cámara OV5647 (CSI)
        │
        ▼
  vision_common.py  ──── captura MJPEG vía rpicam-vid + hilo en background
        │
        ▼
  nodo_linea.py / nodo_canny.py / nodo_vision.py
        │
        ▼  (publica /cmd_vel)
  cmdvel_to_pca9685.py
        │
        ▼  (I2C)
     PCA9685 ──► Servo (CH0) + ESC (CH1)
```

## Nodos

| Nodo | Función | Técnica de percepción |
|---|---|---|
| `nodo_linea` | Seguimiento de línea coloreada | Segmentación HSV (con doble rango opcional) |
| `nodo_canny` | Seguimiento de línea por contraste | Detección de bordes (Canny) |
| `nodo_vision` | Seguimiento de objeto rojo | Segmentación HSV + contornos |
| `cmdvel_to_pca9685` | Traducción de `/cmd_vel` a PWM | — |
| `control_teclado` | Control manual por teclado | — |
| `calibrar_hsv` | Herramienta de calibración de rangos HSV | — |

### Módulos compartidos

- **`vision_common.py`** — captura de frames vía `rpicam-vid` (MJPEG) con hilo lector en background, desacoplando la captura del procesamiento.
- **`vision_stream.py`** — servidor HTTP de depuración visual (stream MJPEG en tiempo real, opcional y eliminable sin afectar a los nodos).

## Instalación

```bash
cd ~/ros2_ws/src
git clone https://github.com/marcosfnmr/matias_control.git
cd ~/ros2_ws
colcon build --packages-select matias_control
source install/setup.bash
```

## Uso

Todos los nodos aceptan parámetros opcionales vía `--ros-args -p nombre:=valor` para ajustar calibración HSV, velocidad, ganancia de control, etc. sin recompilar. Ver la sección de parámetros más abajo.

### Control de hardware (siempre necesario)

```bash
ros2 run matias_control cmdvel_to_pca9685
```

### Seguimiento de línea por color

```bash
ros2 run matias_control nodo_linea
```

### Seguimiento de línea por bordes (Canny)

```bash
ros2 run matias_control nodo_canny
```

### Seguimiento de objeto rojo

```bash
ros2 run matias_control nodo_vision
```

### Control manual por teclado

```bash
ros2 run matias_control control_teclado
```
Teclas: `W` avanzar · `S` retroceder · `A` izquierda · `D` derecha · `C` parar · `Q` salir.

### Calibración de rango HSV

```bash
python3 -m matias_control.calibrar_hsv
```
Sugiere un rango HSV a partir de un frame capturado. Modo de verificación:
```bash
python3 -m matias_control.calibrar_hsv h_low s_low v_low h_high s_high v_high
```

### Depuración visual en tiempo real

Cualquier nodo acepta el parámetro `stream`, que expone un stream MJPEG accesible desde el navegador:
```bash
ros2 run matias_control nodo_linea --ros-args -p stream:=true
```
```
http://<IP_RASPBERRY>:8080
```

### Ejemplo de ajuste de parámetros

```bash
ros2 run matias_control nodo_linea \
  --ros-args -p v_high:=100 -p kp:=2.0 -p v_max:=0.08
```

## Limitaciones conocidas

- **Variabilidad cromática del sensor OV5647**: el mismo objeto oscuro puede renderizarse con tonos HSV distintos según el ángulo de incidencia y la distancia, debido al bajo rango dinámico del sensor y a la corrección de color automática del ISP. `nodo_linea` admite un segundo rango HSV opcional para mitigar este efecto.
- **Sensibilidad a la textura del suelo**: superficies con textura marcada pueden generar falsos positivos tanto en segmentación por color como en detección de bordes.
- **Control en lazo abierto**: el chasis RC no dispone de encoders, por lo que no hay realimentación directa de la velocidad o posición real de las ruedas.

## Autor

Marcos Fernández Martínez — Universidad de León
