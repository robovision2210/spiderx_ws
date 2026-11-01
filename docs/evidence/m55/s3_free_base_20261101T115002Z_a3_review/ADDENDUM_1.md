# Addendum 1 to the A3 review: rounding of the key-write bounds (2026-11-01)

This addendum corrects wording in `REVIEW.md` (commit `2859c8c`), which stays byte-identical, and in the docstring of `scripts/s3/review_a3.py` (commit `ea0203f`). It is arithmetic on the recorded values only: no bag reread, no new simulation. It **does not change the assessment**.

## The rounded records
The driver logs `time.monotonic()` at `log()` entry, rounded to 4 decimals (0.1 ms). The true times therefore lie in these intervals:

| Record | Logged (s) | True log-entry time (s) |
|---|---|---|
| `reached: estop exercise: goal in flight` (before `key('x')`, command thread) | 1753.4301 | in [1753.43005, 1753.43015) |
| `x` key record (after `os.write` returned) | 1753.4316 | in [1753.43155, 1753.43165) |

## Two different quantities

1. **Conservative outer interval for the write: approximately 1753.43005–1753.43165 s.** These are inferred bounds, not a measured write time or duration.
   - The write started after the `reached` record was logged, so no earlier than 1753.43005.
   - It completed before the `x` record was logged, so no later than 1753.43165.
2. **The stricter acceptance comparison: fault > pre + 0.05 ms = 1753.43015 s.** S3-r1's checks (`estop_fault_after_bracket_start`, `cancel_decision_at_the_fault_and_early`) require the node's reaction after the *latest* possible true time of the `reached` record. That is stricter than the outer interval requires.

**Correction.** `REVIEW.md` and the tool's docstring call pre + 0.05 ms (1753.43015) the earliest possible write start, and its `times.bracket_start` field reports that value. That label is wrong: 1753.43015 is the stricter comparison point, not the earliest possible write start, which is just after 1753.43005. Likewise, where `REVIEW.md` says the write completed "no later than 1753.4316", the conservative bound is 1753.43165.

## The node event lies inside both
The node's estop fault and cancel decision were stamped at **1753.431099 s**.
- **Outer interval.** It lies inside it: 1.049 ms after 1753.43005 and 0.551 ms before 1753.43165.
- **Stricter comparison.** It is 0.949 ms after 1753.43015, so it satisfies that too.
- **Post-write record.** It is 0.451–0.551 ms before the `x` record's true log time, in [1753.43155, 1753.43165). `REVIEW.md` rounded this to 0.5 ms.

**Effect on the assessment: none.** The S3-r1 checks hold under the stricter comparison, and the event is also inside the conservative outer interval. Every other statement in the review is unchanged.
