#!/usr/bin/env bash
# The S2 run (frozen harness s2_rehearsal.sh with the current driver as tools/s2_driver.py),
# with cleanup armed BEFORE launch: it runs on success, failure or interruption.
RUN=$1; BAG=$2; EVT=$(dirname "$RUN")/tools
mkdir -p "$RUN"
trap 'bash "$EVT/cleanup.sh" "$RUN" >> "$RUN/cleanup.log" 2>&1' EXIT INT TERM
bash "$EVT/s2_rehearsal.sh" "$RUN" "$BAG"
