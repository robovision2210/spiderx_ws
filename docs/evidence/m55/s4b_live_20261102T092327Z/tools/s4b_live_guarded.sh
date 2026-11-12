#!/usr/bin/env bash
# The S4b live run with cleanup armed BEFORE launch (as phase_c_guarded.sh): cleanup.sh runs on
# success, failure or interruption.
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools
mkdir -p "$RUN"
trap 'bash "$EVT/cleanup.sh" "$RUN" >> "$RUN/cleanup.log" 2>&1' EXIT INT TERM
bash "$EVT/s4b_live_run.sh" "$RUN" "$BAG"
