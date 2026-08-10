# SkySim benchmarks

A small, stable set of seeded tasks so algorithms can be compared and results
cited. A benchmark's value is in *not changing* — new tasks get added, existing
ones stay fixed so numbers remain comparable across versions.

## The suite (v1)

| Task | Observation | Goal |
|------|-------------|------|
| `hover` | full state | reach and hold 3 m altitude |
| `waypoint` | full state | fly a 3-waypoint square leg |
| `gps_denied_nav` | state + depth (vision-only) | reach a goal 40 m ahead through obstacles |

Each runs across 5 fixed seeds; the scorecard reports mean ± std reward and
collision rate.

## Running

```bash
# start the server for the task's scene, then:
python examples/run_benchmark.py --port 5557 --out scorecard.json
```

Bring your own policy by editing `make_policy()` in that script (a callable
`policy(obs) -> action`, optional `policy.reset()`).

## Submitting results

Open a PR adding your `scorecard.json` under `benchmarks/results/<name>/`, with a
one-line description of the method and a link to the code. Include the output of
`check_determinism.py` for the build you used.

## Reproducibility

Benchmark numbers are only meaningful if the build reproduces. Check it:

```bash
python examples/check_determinism.py --port 5557
```

The default `-ffast-math` build is fast but **not guaranteed bit-identical**
across machines/architectures. For reference results, build the extension with
`-DDRONE_SIM_DETERMINISTIC=ON` (strict floating point) and note that in your
submission.
