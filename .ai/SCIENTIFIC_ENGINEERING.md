<!-- managed-by: jam-coding-role; file: SCIENTIFIC_ENGINEERING.md -->
# Scientific Software and Robotics Extension

Read this file only for ML/RL、simulation、robotics、benchmark、causal probe、formal evaluation or hardware work. It extends `ROLE.md`; it does not make every implementation a formal experiment.

## 1. Claim-matched registration

Before a formal run, state the minimum needed to interpret it:

```text
QUESTION / CLAIM CLASS
INTERVENTION / BASELINE
UNIT / TIMEBASE / EVENT
DIRECT METRIC
POPULATION / DENOMINATOR
ADMISSION / STOPPING
SOURCE / CONFIG / CHECKPOINT / ARTIFACTS
```

A small smoke or temporary QA does not need a full experimental registry. It still must report what it actually proves.

## 2. Intent、realization、outcome

Separate intended config、realized telemetry and outcome. Bucket and causal interpretation should use realized exposure in consistent units, not only the intended setting.

## 3. Measure at the correct event

If behavior is required at crossing, do not measure only release. If sustained contact matters, do not report only a peak. When success saturates, use the direct mechanics/quality variable rather than inferring equivalence from a ceiling metric.

## 4. Admission gate must be valid

- reproduce known baseline vitals;
- validate magnitude anchors and units;
- prove derived metrics are monotone/identifiable enough for the claim;
- do not let admission structurally exclude samples where the hypothesis is true;
- do not change gate/reducer after seeing the outcome.

## 5. Causal boundary

State exact restore/matched prefix、intervention horizon、policy/history、randomization、sample size、missing data and the population to which the result applies. Short simulation interventions do not automatically prove long-term adaptation or hardware capability.

## 6. Source lock and provenance

Formal evidence must identify repository revision、resolved config、checkpoint lineage、command、seed/device、output root、evaluator/reducer and completion status. Do not infer existence from filenames or fill missing artifacts with fallback data.

## 7. Timebase and units

Distinguish policy/control step、physics substep、sensor sample and seconds. Declare angle、torque、length、frame、handedness and in/out sign conversions at module boundaries.

## 8. Typed outcomes

Use `SUPPORTED`、`NOT_SUPPORTED`、`INCONCLUSIVE`、`NOT_ADMITTED`、`UNRESOLVED`、`NOT_RUN` accurately. Negative results may be durable memory when they rule out a reusable hypothesis.

## 9. Sim-to-real and hardware

Simulation evidence and hardware evidence are separate. Nominal effort limits、simulated forces and controller commands do not establish real safety margin. Hardware work follows device manuals、risk assessment、workspace isolation and emergency-stop procedures.
