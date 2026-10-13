# Can the battery's data predict combinations it never ran?

Asked of the 2026-09-21 tiered run (`battery-2026-09-21.md`). The hypothesis was
reasonable and I endorsed it too readily: tier 1 covers every single failure and
tier 2 covers every pair exhaustively, so higher-order behaviour should follow by
composition and the whole space becomes predictable without running it.

Tested directly. Learn **only** from tier 1 and tier 2; hold tiers 3, 4 and 5 out
entirely and measure against them.

    reach(S) ~= U reach(e)  U  U added(a,b)  \  U removed(a,b)
                e in S        {a,b} ⊆ S        {a,b} ⊆ S

    added(a,b)   = reach(a,b) \ (reach(a) U reach(b))
    removed(a,b) = (reach(a) U reach(b)) \ reach(a,b)

Learned from 81,951 explicit pairs: 2,519 carry additions, 1,964 carry masking.
The 58 variables that move with no fault armed (18/20/27/42 per preset, measured
by tier 1's own empty cases) are subtracted from both sides — without that the
exact-match figures roughly halve and the comparison is unfair.

## Result

| held-out tier | n | exact | pure-union baseline | mean set overlap |
| --- | --- | --- | --- | --- |
| tier 3 (triples, **chosen from tier 2's interactions**) | 13,560 | 31.1 % | 24.7 % | 85.4 % |
| tier 4 (fourth-order, likewise chosen) | 17,715 | 10.0 % | 8.3 % | 85.9 % |
| tier 5 (**random** 5–12 faults) | 1,000 | **92.6 %** | 90.4 % | **97.3 %** |

The pairwise model beats naive union everywhere, but only slightly — the
interaction terms are worth 2–6 points. What actually separates the rows is how
the cases were **selected**.

Tiers 3 and 4 are not random samples. The battery builds them from combinations
whose members were already observed to interact in tier 2 — they are adversarial
by construction. Tier 5 draws 5-to-12-fault combinations at random.

So the answer is split, and the split is the finding:

- **A combination drawn at random is predictable.** Exact reach 92.6 % of the
  time, 97.3 % set overlap. For the overwhelming bulk of the 2.7 billion triples
  and beyond, you do not need to run anything.
- **A combination chosen because its parts interact is not.** Exact reach 10 % at
  fourth order. And those are precisely the combinations anyone would care about.

Worse for the hypothesis: masking learned from a pair does not survive a third
element. Before noise filtering the pairwise model was *worse* than naive union
on tier 5 (55.6 % against 68.5 %) — the learned `removed(a,b)` sets were being
subtracted in contexts where they no longer applied. Real higher-order structure
exists; pairwise data does not capture it.

## Correction to the earlier report

`battery-2026-09-21.md` calls the tier-by-tier prediction accuracy "the strongest
result" — each tier forecast from the ones below it, 100 % right every time. That
claim is true and nearly empty, and this document supersedes that framing.

Read `TIERS.txt` exactly: *"tier 3 predicted before running: 100.00 % right
(0 failures foreseen, 13560 passes foreseen, 0 failures missed, 0 false alarms)"*.
The prediction is **pass/fail only**. Nothing in 139,072 cases ever failed, so
"predict pass" scores 100 % and carries no information. It does not show that
interactions compose; it shows that nothing breaks.

The substantive question — *which variables a combination moves* — was never what
that figure measured, and the answer to it is the table above.

## What this is good for

- **Triage, not proof.** Predict a combination's reach, and treat a large
  disagreement with the prediction as the signal worth running. That inverts the
  brute-force approach: compute everywhere, run where the model is least certain.
- **Bounding.** Mean overlap 85–97 % means the predicted set is a good estimate
  of scope even when not exact — enough to answer "does this combination touch
  the hydraulics" without a run.
- **Not for certifying a combination safe.** 10 % exact at fourth order on
  interacting cases is nowhere near that bar, and no amount of tier-2 data fixes
  it, because the missing structure is higher-order by nature.

## Reproducing

`tools/battery_extrapolate.py`; it reads the dumps at
`E:\fbw-battery\run-20260921-2220\dumps` and prints the table above. It learns
strictly from tier 1 and tier 2 and never looks at the held-out tiers except to
score itself.

One bug found while writing it, worth noting because it inflates results
silently: `tier_2` and `tier_2_split` number their cases from 0 independently, so
keying them by case index alone drops 5,188 pairs. Key by `(tier, index)`.
