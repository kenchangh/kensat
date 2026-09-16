#!/bin/bash
# KENSAT Jetson inference daemon.
#
# Runs on boot via systemd. Immediately starts llama-server to pre-load
# the model, then sends READY over UART once the server is healthy.
# Waits for RUN:XX (a library prompt) or PROMPT:<text> (a prompt uplinked
# from the ground) from the PIC24, runs inference via HTTP, returns the
# result over UART, then shuts down on OFF command.
#
# The Jetson boots because the PIC24 drives IO.16 HIGH (hardware power
# switch). This daemon does not need UART input to start — it boots
# with the OS and pre-loads the model so it's ready as fast as possible.
#
# UART: /dev/ttyTHS1 at 9600 8N1 (connected to PIC24 UART0 via
#       TXB0102 level shifter on CubeSat Kit bus IO.4/IO.14).

set -euo pipefail

UART="/dev/ttyTHS1"
BAUD=9600
PROMPT_DIR="/opt/kensat/prompts"
MODEL="/opt/kensat/model.gguf"
LLAMA_SERVER="/home/ken/llama.cpp/build/bin/llama-server"
LLAMA_PORT=8080
MAX_TOKENS=128
INFERENCE_TIMEOUT=120
TELEM_POLL_INTERVAL=2
TELEM_PEAK_FILE="/tmp/kensat_peak_temps"

cleanup() {
    if [[ -n "${POLL_PID:-}" ]] && kill -0 "$POLL_PID" 2>/dev/null; then
        kill "$POLL_PID" 2>/dev/null || true
        wait "$POLL_PID" 2>/dev/null || true
    fi
    # Don't kill llama-server — let it persist for fast restarts
    rm -f "$TELEM_PEAK_FILE" 2>/dev/null || true
    exec 3>&- 2>/dev/null || true
}
trap cleanup EXIT

# ---- Telemetry helpers ----

# Find thermal zone path by type name (e.g., "CPU-therm", "GPU-therm")
find_thermal_zone() {
    local target="$1"
    for zone in /sys/devices/virtual/thermal/thermal_zone*; do
        if [[ -f "$zone/type" ]] && [[ "$(cat "$zone/type" 2>/dev/null)" == "$target" ]]; then
            echo "$zone"
            return 0
        fi
    done
    echo ""
    return 1
}

# Read temperature from a thermal zone path, returns degrees C (integer)
read_temp_c() {
    local zone="$1"
    if [[ -n "$zone" ]] && [[ -f "$zone/temp" ]]; then
        local mc
        mc=$(cat "$zone/temp" 2>/dev/null || echo 0)
        echo $(( mc / 1000 ))
    else
        echo 0
    fi
}

# Read total input power in milliwatts from INA3221 (VDD_IN channel)
read_power_mw() {
    local hwmon_dir=""
    for d in /sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*; do
        if [[ -f "$d/in1_input" ]]; then
            hwmon_dir="$d"
            break
        fi
    done
    if [[ -z "$hwmon_dir" ]]; then
        echo 0
        return
    fi
    local mv ma
    mv=$(cat "$hwmon_dir/in1_input" 2>/dev/null || echo 0)
    ma=$(cat "$hwmon_dir/curr1_input" 2>/dev/null || echo 0)
    echo $(( mv * ma / 1000 ))
}

# Read RAM used in MB
read_ram_mb() {
    local total avail
    total=$(awk '/MemTotal/ {print $2}' /proc/meminfo)
    avail=$(awk '/MemAvailable/ {print $2}' /proc/meminfo)
    echo $(( (total - avail) / 1024 ))
}

# Poll CPU/GPU temps in background, writing peak values to a file.
# Usage: start_temp_polling <cpu_zone> <gpu_zone>
# Kill the background PID to stop. Peak values in $TELEM_PEAK_FILE.
start_temp_polling() {
    local cpu_zone="$1" gpu_zone="$2"
    local cpu_peak=0 gpu_peak=0
    rm -f "$TELEM_PEAK_FILE"
    while true; do
        local ct gt
        ct=$(read_temp_c "$cpu_zone")
        gt=$(read_temp_c "$gpu_zone")
        (( ct > cpu_peak )) && cpu_peak=$ct
        (( gt > gpu_peak )) && gpu_peak=$gt
        echo "${cpu_peak},${gpu_peak}" > "$TELEM_PEAK_FILE"
        sleep "$TELEM_POLL_INTERVAL"
    done
}

# ---- Early breadcrumb: open UART so we can yell about progress before READY ----
# This lets us see boot/init progress on the protocol UART even if a later
# step hangs (model load, llama-server crash, etc.).
DBG_UART="/dev/ttyTHS1"
if [[ -e "$DBG_UART" ]]; then
    stty -F "$DBG_UART" 9600 cs8 -cstopb -parenb raw -echo -echoe -echok 2>/dev/null || true
fi
dbg() {
    if [[ -e "$DBG_UART" ]]; then
        printf 'DBG:%s\n' "$1" > "$DBG_UART" 2>/dev/null || true
    fi
    echo "[kensat] DBG:$1" >&2
}

dbg "0-daemon-start"

# ---- Step 0: Set 7W power mode ----
echo "[kensat] Setting 7W power mode..." >&2
echo "${SUDO_PASS:-}" | sudo -S nvpmodel -m 3 2>/dev/null
echo "${SUDO_PASS:-}" | sudo -S jetson_clocks 2>/dev/null || true
echo "[kensat] Power mode: $(echo "${SUDO_PASS:-}" | sudo -S nvpmodel -q 2>&1 | head -1)" >&2
dbg "1-nvpmodel-set"

# ---- Step 1: Start llama-server (or reuse existing) ----
model_load_start=$SECONDS
if curl -sf "http://127.0.0.1:${LLAMA_PORT}/health" >/dev/null 2>&1; then
    echo "[kensat] llama-server already running and healthy — reusing." >&2
    LLAMA_PID=""
    model_load_dur=0
    dbg "2-llama-reused"
else
    echo "[kensat] Starting llama-server (model pre-load)..." >&2
    dbg "2-llama-starting"
    LD_LIBRARY_PATH="${LLAMA_SERVER%/*}" "$LLAMA_SERVER" \
        -m "$MODEL" \
        --port "$LLAMA_PORT" \
        -ngl 99 \
        --ctx-size 512 \
        >/dev/null 2>&1 &
    LLAMA_PID=$!

    # Wait for server health
    echo "[kensat] Waiting for model to load..." >&2
    ready=0
    for attempt in $(seq 1 120); do
        if curl -sf "http://127.0.0.1:${LLAMA_PORT}/health" >/dev/null 2>&1; then
            ready=1
            break
        fi
        sleep 1
    done

    if [[ "$ready" -eq 0 ]]; then
        echo "[kensat] llama-server failed to start within 120s." >&2
        dbg "X-llama-timeout"
        exit 1
    fi
    model_load_dur=$(( SECONDS - model_load_start ))
    echo "[kensat] Model loaded, server ready (${model_load_dur}s)." >&2
    dbg "3-llama-healthy"
fi

# Discover thermal zones once for reuse
CPU_ZONE=$(find_thermal_zone "cpu-thermal") || true
GPU_ZONE=$(find_thermal_zone "gpu-thermal") || true
echo "[kensat] Thermal zones: CPU=${CPU_ZONE:-none} GPU=${GPU_ZONE:-none}" >&2

dbg "4-thermal-zones"

# ---- Step 2: Configure UART and signal READY ----
stty -F "$UART" "$BAUD" cs8 -cstopb -parenb raw -echo -echoe -echok
exec 3<>"$UART"
dbg "5-uart-open"

uart_send() {
    printf '%s\n' "$1" >&3
}

uart_readline() {
    local timeout="${1:-60}"
    read -r -t "$timeout" line <&3 || return 1
    echo "$line"
}

# ---- Background heartbeat: sends HB:<state>:<uptime>s every 10 s ----
# Suppressed during inference (HB lines mid-protocol would corrupt parsing)
# by writing the state to a file; the heartbeat skips when state == "infer".
HB_STATE_FILE="/tmp/kensat_hb_state"
echo "idle" > "$HB_STATE_FILE"

start_heartbeat() {
    (
        while true; do
            sleep 10
            local st
            st=$(cat "$HB_STATE_FILE" 2>/dev/null || echo "?")
            if [[ "$st" != "infer" ]]; then
                printf 'HB:%s:%ds\n' "$st" "$SECONDS" >&3 2>/dev/null || true
            fi
        done
    ) &
    HB_PID=$!
}

set_hb_state() {
    echo "$1" > "$HB_STATE_FILE"
}

# Make sure heartbeat dies with us
_orig_cleanup="$(declare -f cleanup | tail -n +2)"
cleanup() {
    if [[ -n "${HB_PID:-}" ]] && kill -0 "$HB_PID" 2>/dev/null; then
        kill "$HB_PID" 2>/dev/null || true
        wait "$HB_PID" 2>/dev/null || true
    fi
    if [[ -n "${POLL_PID:-}" ]] && kill -0 "$POLL_PID" 2>/dev/null; then
        kill "$POLL_PID" 2>/dev/null || true
        wait "$POLL_PID" 2>/dev/null || true
    fi
    rm -f "$TELEM_PEAK_FILE" "$HB_STATE_FILE" 2>/dev/null || true
    exec 3>&- 2>/dev/null || true
}

start_heartbeat

uart_send "READY"
echo "[kensat] Sent READY, waiting for RUN command..." >&2

# ---- Step 3: Wait for the prompt command ----
#
# Two forms, both answered identically (DONE:<len> + text + TELEM:...):
#
#   RUN:XX            run prompt XX (00-19) from the pre-loaded library
#   PROMPT:<text>     run this free-text prompt, uplinked from the ground
#                     and forwarded verbatim by the OBC
#
# The OBC caps an uplinked prompt at 256 chars (JETSON_PROMPT_MAX in
# firmware/obc/app/task_jetson.h) and reduces it to printable ASCII on one
# line, so the bound below is a belt-and-braces check on a serial line, not
# the primary limit.
line=$(uart_readline 120) || {
    echo "[kensat] Timeout waiting for RUN command, exiting." >&2
    exit 1
}

line="${line%%$'\r'}"

PROMPT_MAX_CHARS=256

if [[ "$line" =~ ^RUN:([0-9]{2})$ ]]; then
    prompt_id="${BASH_REMATCH[1]}"
    prompt_file="$PROMPT_DIR/${prompt_id}.txt"
    echo "[kensat] Running library prompt $prompt_id from $prompt_file" >&2

    if [[ ! -f "$prompt_file" ]]; then
        echo "[kensat] Prompt file not found: $prompt_file" >&2
        uart_send "ERR:NO_PROMPT"
        exit 1
    fi

    prompt_text=$(cat "$prompt_file")

elif [[ "$line" == PROMPT:* ]]; then
    prompt_id="UP"
    prompt_text="${line#PROMPT:}"

    if [[ -z "$prompt_text" ]]; then
        echo "[kensat] Empty uplinked prompt." >&2
        uart_send "ERR:EMPTY_PROMPT"
        exit 1
    fi
    if (( ${#prompt_text} > PROMPT_MAX_CHARS )); then
        echo "[kensat] Uplinked prompt too long (${#prompt_text} chars), truncating." >&2
        prompt_text="${prompt_text:0:PROMPT_MAX_CHARS}"
    fi

    echo "[kensat] Running uplinked prompt (${#prompt_text} chars): $prompt_text" >&2

else
    echo "[kensat] Unexpected command: '$line'" >&2
    exit 1
fi

# ---- Step 4: Run inference (server already loaded) ----
# Sample baseline temperatures before inference
cpu_before=$(read_temp_c "$CPU_ZONE")
gpu_before=$(read_temp_c "$GPU_ZONE")
echo "[kensat] Pre-inference temps: CPU=${cpu_before}C GPU=${gpu_before}C" >&2

# Start background temp polling for peak detection
start_temp_polling "$CPU_ZONE" "$GPU_ZONE" &
POLL_PID=$!

infer_start=$SECONDS
set_hb_state "infer"
echo "[kensat] Running inference (timeout=${INFERENCE_TIMEOUT}s)..." >&2

# Build JSON body via python3 (avoids heredoc quoting issues)
json_body=$(python3 -c "
import json, sys
body = {
    'messages': [{'role': 'user', 'content': sys.argv[1]}],
    'max_tokens': int(sys.argv[2]),
    'temperature': 0.7
}
print(json.dumps(body))
" "$prompt_text" "$MAX_TOKENS")
echo "[kensat] Request body: ${json_body:0:200}..." >&2

# Run inference — guard against set -e with || true
response=$(curl -s --max-time "$INFERENCE_TIMEOUT" \
    "http://127.0.0.1:${LLAMA_PORT}/v1/chat/completions" \
    -H "Content-Type: application/json" \
    -d "$json_body" 2>&1) || true
curl_rc=${PIPESTATUS[0]:-$?}
echo "[kensat] curl exit code: $curl_rc, response length: ${#response}" >&2

if [[ -z "$response" ]] || echo "$response" | grep -q '"error"'; then
    kill "$POLL_PID" 2>/dev/null || true; wait "$POLL_PID" 2>/dev/null || true
    echo "[kensat] Inference failed (curl rc=$curl_rc). Response: ${response:0:500}" >&2
    uart_send "ERR:INFERENCE_FAIL"
    exit 1
fi

reply=$(echo "$response" | python3 -c '
import json, sys
data = json.load(sys.stdin)
# Replace newlines with spaces — PIC24 reads until first \n
# Truncate to 384 chars — PIC24 RX ring buffer is 512 bytes, need headroom.
# The OBC downlinks whatever arrives here in 144-byte AX.25 "TXT" frames,
# one per 10 s beacon cycle, so 384 chars is 3 frames per pass over the set.
text = data["choices"][0]["message"]["content"]
text = text.replace("\n", " ").replace("\r", "")[:384]
print(text)
' 2>/dev/null) || {
    echo "[kensat] Failed to parse response JSON." >&2
    reply=$(echo "$response" | tr '\n\r' '  ' | head -c 384)
}

# Stop temp polling and read peak values
kill "$POLL_PID" 2>/dev/null || true; wait "$POLL_PID" 2>/dev/null || true
infer_dur=$(( SECONDS - infer_start ))
if [[ -f "$TELEM_PEAK_FILE" ]]; then
    IFS=',' read -r cpu_peak gpu_peak < "$TELEM_PEAK_FILE"
    rm -f "$TELEM_PEAK_FILE"
else
    cpu_peak=$cpu_before
    gpu_peak=$gpu_before
fi

# Keep llama-server running for fast restarts

resp_len=${#reply}
echo "[kensat] Inference complete, response length=$resp_len" >&2

# ---- Step 5: Send result over UART ---- (HB stays suppressed through TELEM)
uart_send "DONE:${resp_len}"
printf '%s\n' "$reply" >&3
echo "[kensat] Sent DONE:${resp_len} + response" >&2

# Give PIC24 time to drain ring buffer and transition to JS_READ_TELEM
sleep 5

# ---- Step 5b: Send telemetry ----
power_now=$(read_power_mw)
ram_now=$(read_ram_mb)
telem="${cpu_before},${gpu_before},${cpu_peak},${gpu_peak},${power_now},${ram_now},${infer_dur},${model_load_dur}"
uart_send "TELEM:${telem}"
echo "[kensat] Sent TELEM:${telem}" >&2
set_hb_state "shutdown-wait"

# ---- Step 6: Wait for OK then OFF ----
line=$(uart_readline 30) || true
line="${line%%$'\r'}"
echo "[kensat] Received: '$line'" >&2

line=$(uart_readline 30) || true
line="${line%%$'\r'}"
echo "[kensat] Received: '$line'" >&2

# ---- Step 7: Shutdown ----
echo "[kensat] Shutting down..." >&2
exec 3>&-
# PIC24 controls power via IO.16 — only shutdown if we got the OFF command.
# The `shutdown` call is deliberately left commented out: the OBC cuts the
# rail 10 s after sending OFF either way, and the rootfs is mounted read-only
# so a hard cut is safe. The `:` keeps the branch syntactically valid with
# the call commented out — without it bash fails to parse this block.
if [[ "${line:-}" == "OFF" ]]; then
    :
    # echo "${SUDO_PASS:-}" | sudo -S shutdown -h now
else
    echo "[kensat] No OFF received, exiting without shutdown." >&2
fi
