# Sim-to-real transfer

The whole point of SkySim is that an algorithm developed and tested here works on
a real drone. That claim is only credible with a **demonstrated transfer
result** — and that result cannot come from the simulator. It comes from flying a
real vehicle. This document is the framework to produce and document one; the
result itself is contributed by whoever flies it.

> **Status: no verified transfer result yet.** SkySim provides the tools to
> measure transfer; the first documented result is an open, high-value
> contribution. Do not claim transfer the sim hasn't demonstrated.

## Why it's the hard part

Data collected in sim only improves a real drone if the simulated inputs are
close enough to reality, or if the policy was trained to be robust to the gap.
Naive "train in sim, deploy real" often fails — especially for vision. Closing
the gap is the real work, and these are the levers SkySim gives you:

- **Ground-truth labels** (perfect pose/depth/segmentation you can't get from
  real flights) for supervised training and evaluation.
- **Domain randomization** (`DomainRandomizer`): vary mass, wind, spawn, sensor
  noise per episode so the real world looks like one more sample.
- **The shared interface**: the same observation/action contract in sim and
  on-vehicle, so the same policy binary runs in both.

## Methodology

1. **Train / tune** the policy against a benchmark task in SkySim, with domain
   randomization on. Record runs (`RecordRun`) for reproducibility.
2. **Freeze** the policy and its exact SkySim scorecard + determinism check.
3. **Deploy** the *same* policy to the real vehicle behind the *same* interface
   (observation → action). No sim-only inputs.
4. **Fly** a matched real-world version of the task; log real telemetry.
5. **Compare** sim vs real: success rate, trajectory error, and the gap in the
   metrics that matter for the task.

## Results template

Copy to `docs/transfer/<date>-<task>.md`:

```
## Transfer result: <task>, <date>

- Policy: <arch / training method / commit>
- SkySim scorecard: <link to scorecard.json>  (build: <deterministic? flags>)
- Domain randomization: <ranges used>
- Vehicle: <frame, flight controller, companion computer>
- Real task: <how the real-world task matched the sim task>

| Metric | Sim | Real |
|--------|-----|------|
| success rate | | |
| mean trajectory error (m) | | |
| <task metric> | | |

Notes: <what transferred, what didn't, what closed the gap>
Media: <video / logs>
```

A single honest transfer result — even a modest one — is the most valuable thing
the project can publish. It's what turns "should transfer" into "does."
