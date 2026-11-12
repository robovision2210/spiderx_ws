#!/usr/bin/env bash
# The S4d-M shadow run with cleanup armed BEFORE launch (as the S4c level-2 shadow's guarded
# wrapper): cleanup.sh (owned processes only) runs on success, failure or interruption.
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools
mkdir -p "$RUN"
trap 'bash "$EVT/cleanup.sh" "$RUN" >> "$RUN/cleanup.log" 2>&1' EXIT INT TERM
bash "$EVT/s4d_shadow_run.sh" "$RUN" "$BAG"
