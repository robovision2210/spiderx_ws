#!/usr/bin/env bash
# S4e harness template: the S2-S4d drivers running now, as a JSON list (H1 needs none at the start
# and at the end of the session). Usage: scan.sh <run dir> <start|end>
RUN=$1; H=$(dirname "$(realpath "$0")"); source "$H/env.sh"
pgrep -af -- "$DRIVER_PATTERN" | grep -v -E "pgrep|scan.sh" | python3 -c '
import json, sys
print(json.dumps([{"pid": int(ln.split(None, 1)[0]), "cmd": ln.split(None, 1)[1].strip()}
                  for ln in sys.stdin if ln.strip()]))' > "$RUN/process_scan_$2.json"
echo "process scan $2: $(cat "$RUN/process_scan_$2.json")"
