#!/bin/bash

NODO=$1
DURACION=${2:-60}

if [ -z "$NODO" ]; then
  echo "Uso: ./medir_rendimiento.sh nodo_linea 60"
  echo "Ejemplo: ./medir_rendimiento.sh nodo_canny 60"
  exit 1
fi

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws || exit 1
source install/setup.bash

OUT="rendimiento_${NODO}_$(date +%Y%m%d_%H%M%S).txt"

echo "=== PRUEBA DE RENDIMIENTO: $NODO ===" | tee "$OUT"
echo "Fecha: $(date)" | tee -a "$OUT"
echo "Duración: ${DURACION}s" | tee -a "$OUT"
echo "" | tee -a "$OUT"

echo "=== Temperatura inicial ===" | tee -a "$OUT"
vcgencmd measure_temp | tee -a "$OUT"

echo "=== Throttling inicial ===" | tee -a "$OUT"
vcgencmd get_throttled | tee -a "$OUT"

echo "" | tee -a "$OUT"
echo "=== Procesos relacionados antes de medir ===" | tee -a "$OUT"
ps -eo pid,ppid,pcpu,pmem,cmd | grep -E "$NODO|rpicam-vid" | grep -v grep | tee -a "$OUT"

# Buscar PIDs relevantes:
# 1) Proceso Python del nodo
# 2) Proceso rpicam-vid usado para capturar cámara
PIDS=$(ps -eo pid,cmd | grep -E "$NODO|rpicam-vid" | grep -v grep | awk '{print $1}' | paste -sd, -)

if [ -z "$PIDS" ]; then
  echo "" | tee -a "$OUT"
  echo "ERROR: no se encontraron procesos relacionados con $NODO ni rpicam-vid." | tee -a "$OUT"
  echo "Asegúrate de que el nodo está ejecutándose en otra terminal." | tee -a "$OUT"
  exit 1
fi

echo "" | tee -a "$OUT"
echo "=== PIDs medidos ===" | tee -a "$OUT"
echo "PIDS=$PIDS" | tee -a "$OUT"

echo "" | tee -a "$OUT"
echo "=== CPU y RAM durante ${DURACION}s ===" | tee -a "$OUT"
pidstat -u -r -p "$PIDS" 1 "$DURACION" | tee -a "$OUT" &
PIDSTAT_PID=$!

echo "" | tee -a "$OUT"
echo "=== Frecuencia de /cmd_vel durante ${DURACION}s ===" | tee -a "$OUT"
timeout "$DURACION" ros2 topic hz /cmd_vel | tee -a "$OUT" &
HZ_PID=$!

wait "$PIDSTAT_PID"
wait "$HZ_PID"

echo "" | tee -a "$OUT"
echo "=== Procesos relacionados después de medir ===" | tee -a "$OUT"
ps -eo pid,ppid,pcpu,pmem,cmd | grep -E "$NODO|rpicam-vid" | grep -v grep | tee -a "$OUT"

echo "" | tee -a "$OUT"
echo "=== Temperatura final ===" | tee -a "$OUT"
vcgencmd measure_temp | tee -a "$OUT"

echo "=== Throttling final ===" | tee -a "$OUT"
vcgencmd get_throttled | tee -a "$OUT"

echo "" | tee -a "$OUT"
echo "Resultado guardado en: ~/ros2_ws/$OUT"

