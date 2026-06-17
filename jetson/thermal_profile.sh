#!/bin/bash
# Thermal & power profiler for KENSAT Jetson Orin Nano.
#
# Samples temperature (CPU, GPU, SOC, Tj) and power (VDD_IN, VDD_CPU_GPU_CV,
# VDD_SOC) at 500ms intervals. Logs to a timestamped CSV in /tmp/.
#
# Run alongside the kensat daemon cycle to capture the full thermal profile
# for CubeSat cooling design.
#
# Usage:
#   bash /opt/kensat/thermal_profile.sh &
#   sudo systemctl restart kensat
#   # Wait for cycle to complete, then Ctrl-C or let it auto-stop
#
# Output: /tmp/thermal_YYYYMMDD_HHMMSS.csv

set -u

# --- Config ---
INTERVAL=0.5          # Sample interval in seconds
TIMEOUT=300           # Auto-stop after 5 minutes (covers full boot+infer+shutdown)
INA="/sys/devices/platform/bus@0/c240000.i2c/i2c-1/1-0040/hwmon/hwmon1"

# --- Output file ---
CSV="/tmp/thermal_$(date +%Y%m%d_%H%M%S).csv"
echo "timestamp_s,elapsed_s,cpu_mC,gpu_mC,soc_mC,tj_mC,vdd_in_mV,vdd_in_mA,vdd_cpugpu_mV,vdd_cpugpu_mA,vdd_soc_mV,vdd_soc_mA,total_power_mW,event" > "$CSV"

echo "[thermal] Logging to $CSV (interval=${INTERVAL}s, timeout=${TIMEOUT}s)" >&2

# --- Journal event tagging ---
# Tail the kensat service journal in background, write events to a regular file
EVENT_FILE="/tmp/thermal_events_$$"
> "$EVENT_FILE"

journalctl -u kensat.service -f --since "now" --no-pager -o cat 2>/dev/null | while IFS= read -r line; do
    case "$line" in
        *"Starting llama-server"*)   echo "MODEL_LOADING" >> "$EVENT_FILE" ;;
        *"Model loaded"*)            echo "MODEL_LOADED"  >> "$EVENT_FILE" ;;
        *"Sent READY"*)              echo "READY"         >> "$EVENT_FILE" ;;
        *"Running inference"*)       echo "INFERENCE"     >> "$EVENT_FILE" ;;
        *"Inference complete"*)      echo "INFER_DONE"    >> "$EVENT_FILE" ;;
        *"Shutting down"*)           echo "SHUTDOWN"      >> "$EVENT_FILE" ;;
    esac
done &
JOURNAL_PID=$!

cleanup() {
    kill "$JOURNAL_PID" 2>/dev/null || true
    rm -f "$EVENT_FILE"
    echo "[thermal] Stopped. CSV at: $CSV" >&2
    echo "[thermal] Total samples: $(( $(wc -l < "$CSV") - 1 ))" >&2
}
trap cleanup EXIT

# Helper: read sysfs file, return 0 on failure
sysfs_read() {
    cat "$1" 2>/dev/null || echo "0"
}

# Helper: consume latest event from event file (non-blocking)
LAST_EVENT_LINE=0
consume_event() {
    local total
    total=$(wc -l < "$EVENT_FILE" 2>/dev/null || echo "0")
    total=$((total + 0))  # ensure numeric
    if (( total > LAST_EVENT_LINE )); then
        LAST_EVENT_LINE=$total
        tail -1 "$EVENT_FILE"
    fi
}

# --- Main sampling loop ---
START=$(date +%s.%N)
SAMPLES=0

while true; do
    NOW=$(date +%s.%N)
    ELAPSED=$(echo "$NOW - $START" | bc)

    # Check timeout
    if (( $(echo "$ELAPSED > $TIMEOUT" | bc -l) )); then
        echo "[thermal] Timeout reached (${TIMEOUT}s), stopping." >&2
        break
    fi

    # Read temperatures (millidegrees C)
    cpu=$(sysfs_read /sys/devices/virtual/thermal/thermal_zone0/temp)
    gpu=$(sysfs_read /sys/devices/virtual/thermal/thermal_zone1/temp)
    soc=$(sysfs_read /sys/devices/virtual/thermal/thermal_zone5/temp)
    tj=$(sysfs_read  /sys/devices/virtual/thermal/thermal_zone8/temp)

    # Read power rails
    vin_mv=$(sysfs_read "$INA/in1_input")
    vin_ma=$(sysfs_read "$INA/curr1_input")
    vcpugpu_mv=$(sysfs_read "$INA/in2_input")
    vcpugpu_ma=$(sysfs_read "$INA/curr2_input")
    vsoc_mv=$(sysfs_read "$INA/in3_input")
    vsoc_ma=$(sysfs_read "$INA/curr3_input")

    # Compute total power (mW)
    power_mw=$(( vin_mv * vin_ma / 1000 ))

    # Check for daemon events (non-blocking)
    event=$(consume_event)

    # Write CSV row
    echo "${NOW},${ELAPSED},${cpu},${gpu},${soc},${tj},${vin_mv},${vin_ma},${vcpugpu_mv},${vcpugpu_ma},${vsoc_mv},${vsoc_ma},${power_mw},${event}" >> "$CSV"

    SAMPLES=$((SAMPLES + 1))
    if (( SAMPLES % 20 == 0 )); then
        echo "[thermal] ${SAMPLES} samples, elapsed=${ELAPSED}s, Tj=${tj}mC, power=${power_mw}mW" >&2
    fi

    sleep "$INTERVAL"
done
