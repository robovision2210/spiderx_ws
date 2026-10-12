#!/usr/bin/env bash
# Plan section 4 E (confirmation): wait for the tool's prompt (readiness #1 passed) or its exit.
# Write the word ONCE only if the prompt is present and no refusal or error is shown; otherwise
# close the pipe (EOF refuses: nothing is sent). Records what was seen and what was done.
RUN=$(cd "$1" && pwd)
PROMPT="Type SEND-ONE-TROT-CYCLE to send ONE trot-cycle goal"
for i in $(seq 1 240); do                     # up to 120 s
  if grep -q "$PROMPT" $RUN/live.txt 2>/dev/null; then break; fi
  if grep -qE "^exit=|REFUSED|final state" $RUN/live.txt 2>/dev/null; then break; fi
  sleep 0.5
done
date -u +%FT%T.%NZ > $RUN/confirm_decision_utc.txt
cp $RUN/live.txt $RUN/live_at_decision.txt
if grep -q "$PROMPT" $RUN/live.txt && ! grep -qE "REFUSED|Traceback|Error|final state|^exit=" $RUN/live.txt; then
  printf 'SEND-ONE-TROT-CYCLE\n' > $RUN/confirm
  echo "decision: prompt present, no refusal or error: confirmation word written once" > $RUN/confirm_decision.txt
else
  echo "decision: prompt absent or a refusal/error shown: pipe closed, nothing confirmed" > $RUN/confirm_decision.txt
fi
kill $(cat $RUN/confirm_holder.pid) 2>/dev/null      # close the write end: EOF after the word
cat $RUN/confirm_decision.txt
