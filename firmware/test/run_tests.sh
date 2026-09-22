#!/usr/bin/env bash
# Compila e roda os ensaios de host. Recorta os trechos de main/*.c em vez de
# copia-los, para o ensaio nunca divergir do firmware.
set -euo pipefail

cd "$(dirname "$0")"
MAIN=../main
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

CFLAGS="-std=gnu11 -I$MAIN -Wall -Wextra -Wno-unused-function"

# ---- odometria -------------------------------------------------------------
INI=$(grep -n '^static float wrap_pi' "$MAIN/odometry.c" | cut -d: -f1)
FIM=$(($(grep -n '^static void odometry_task' "$MAIN/odometry.c" | cut -d: -f1) - 2))
{
    echo '#include <math.h>'
    echo '#include <stdbool.h>'
    echo '#include <stdio.h>'
    echo '#include "config.h"'
    sed -n "${INI},${FIM}p" "$MAIN/odometry.c"
    cat test_odom_main.c
} > "$OUT/test_odom.c"
gcc $CFLAGS -o "$OUT/test_odom" "$OUT/test_odom.c" -lm

# ---- payload de telemetria -------------------------------------------------
INI=$(grep -n '^#define APPEND' "$MAIN/telemetry.c" | cut -d: -f1)
FIM=$(($(grep -n '^static void telemetry_task' "$MAIN/telemetry.c" | cut -d: -f1) - 1))
{
    cat <<'STUB'
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "state.h"
static int64_t s_now = 24000000;
static int64_t esp_timer_get_time(void) { return s_now; }
static uint32_t s_seq = 120;
STUB
    sed -n "${INI},${FIM}p" "$MAIN/telemetry.c"
    cat test_payload_main.c
} > "$OUT/test_payload.c"
gcc $CFLAGS -o "$OUT/test_payload" "$OUT/test_payload.c"

# ---- execucao --------------------------------------------------------------
echo "== cinematica diferencial =="
"$OUT/test_odom"

echo
echo "== payload de telemetria =="
"$OUT/test_payload" > "$OUT/out.json"
python3 - "$OUT/out.json" <<'PY'
import json, sys
for i, linha in enumerate(open(sys.argv[1]), 1):
    json.loads(linha)
    print(f"amostra {i}: JSON valido, {len(linha)-1} bytes")
PY
echo
echo "todos os ensaios passaram"
