#!/usr/bin/env bash
# Extracts code from main/*.c instead of copying it, so the tests never drift
# from the firmware.
set -euo pipefail

cd "$(dirname "$0")"
MAIN=../main
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

CFLAGS="-std=gnu11 -I$MAIN -Wall -Wextra -Wno-unused-function"

START=$(grep -n '^static float wrap_pi' "$MAIN/odometry.c" | cut -d: -f1)
END=$(($(grep -n '^static void odometry_task' "$MAIN/odometry.c" | cut -d: -f1) - 2))
{
    echo '#include <math.h>'
    echo '#include <stdbool.h>'
    echo '#include <stdio.h>'
    echo '#include "config.h"'
    sed -n "${START},${END}p" "$MAIN/odometry.c"
    cat test_odom_main.c
} > "$OUT/test_odom.c"
gcc $CFLAGS -o "$OUT/test_odom" "$OUT/test_odom.c" -lm

START=$(grep -n '^#define APPEND' "$MAIN/telemetry.c" | cut -d: -f1)
END=$(($(grep -n '^static void telemetry_task' "$MAIN/telemetry.c" | cut -d: -f1) - 1))
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
    sed -n "${START},${END}p" "$MAIN/telemetry.c"
    cat test_payload_main.c
} > "$OUT/test_payload.c"
gcc $CFLAGS -o "$OUT/test_payload" "$OUT/test_payload.c"

echo "== differential kinematics =="
"$OUT/test_odom"

echo
echo "== telemetry payload =="
"$OUT/test_payload" > "$OUT/out.json"
python3 - "$OUT/out.json" <<'PY'
import json, sys
for i, line in enumerate(open(sys.argv[1]), 1):
    json.loads(line)
    print(f"sample {i}: valid JSON, {len(line)-1} bytes")
PY

if [ -f ../../docs/wiring.md ]; then
echo
echo "== wiring doc pins =="
python3 - "$MAIN/config.h" ../../docs/wiring.md <<'PY'
import re, sys
cfg, doc = (open(a).read() for a in sys.argv[1:3])
in_cfg = {int(p) for n, v in re.findall(r'#define (PIN_\w+)\s+(\{[^}]*\}|\d+)', cfg)
          for p in re.findall(r'\d+', v)}
in_doc = {int(m) for m in re.findall(r'GPIO (\d+)', doc)}
if in_cfg != in_doc:
    print("MISMATCH between config.h and docs/wiring.md:", in_cfg ^ in_doc)
    sys.exit(1)
print(f"{len(in_cfg)} pins match")
PY
fi

echo
echo "all tests passed"
