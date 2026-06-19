#!/bin/bash

OUT="pi5_monitor_$(date +%Y%m%d_%H%M%S).csv"
DURATION=300
INTERVAL=10
END=$((SECONDS + DURATION))

echo "timestamp,elapsed_s,temp_c,ext5v_v,ext5v_a,power_w,throttled_hex,undervoltage_now,throttled_now,cap_now,undervoltage_seen,throttled_seen,cap_seen" > "$OUT"

while [ $SECONDS -lt $END ]; do
    TS=$(date '+%Y-%m-%d %H:%M:%S')
    ELAPSED=$((300 - (END - SECONDS)))

    TEMP=$(vcgencmd measure_temp | sed "s/temp=//;s/'C//")

    ADC=$(vcgencmd pmic_read_adc)

    V=$(echo "$ADC" | awk -F= '/EXT5V_V/ {print $2}' | awk '{print $1}')
    A=$(echo "$ADC" | awk -F= '/EXT5V_I/ {print $2}' | awk '{print $1}')

    POWER=$(awk -v v="$V" -v a="$A" 'BEGIN {printf "%.3f", v*a}')

    HEX=$(vcgencmd get_throttled | cut -d= -f2)
    DEC=$((HEX))

    UV_NOW=$(( (DEC & 0x1) != 0 ))
    CAP_NOW=$(( (DEC & 0x2) != 0 ))
    THR_NOW=$(( (DEC & 0x4) != 0 ))

    UV_SEEN=$(( (DEC & 0x10000) != 0 ))
    CAP_SEEN=$(( (DEC & 0x20000) != 0 ))
    THR_SEEN=$(( (DEC & 0x40000) != 0 ))

    echo "$TS,$ELAPSED,$TEMP,$V,$A,$POWER,$HEX,$UV_NOW,$THR_NOW,$CAP_NOW,$UV_SEEN,$THR_SEEN,$CAP_SEEN" >> "$OUT"

    sleep "$INTERVAL"
done

echo "Guardado en: $OUT"
