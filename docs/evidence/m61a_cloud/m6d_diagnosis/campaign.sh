#!/bin/bash
# usage: campaign.sh PREFIX STACK BURNERS N
DG="$(cd "$(dirname "$0")" && pwd)"
for i in $(seq -w 1 "$4"); do "$DG/run_diag.sh" "${1}_$i" "$2" "$3"; done
echo CAMPAIGN_DONE
