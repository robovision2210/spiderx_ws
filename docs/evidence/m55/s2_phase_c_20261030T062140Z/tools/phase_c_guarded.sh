#!/usr/bin/env bash
# The Phase C run with cleanup armed BEFORE launch (as the S2 s2_run_guarded.sh): cleanup.sh runs
# on success, failure or interruption.
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools
mkdir -p "$RUN"
trap 'bash "$EVT/cleanup.sh" "$RUN" >> "$RUN/cleanup.log" 2>&1' EXIT INT TERM
bash "$EVT/phase_c_run.sh" "$RUN" "$BAG"
