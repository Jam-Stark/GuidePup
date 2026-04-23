# Base-Only Policy: Gate Preset Tuning Analysis

> CSV: `logs/experiments/unifp_smart_gait_mujoco/base_only.csv`
> Policy: `logs/rsl_rl/uniFP_aliengo_position_force/2026-04-21_17-33-58_base_only/mujoco_exported/policy.pt`
> Preset used: `--gate-preset v1`  
> Force schedule: `human_guide`  
> Duration: 60 s @ 50 Hz (3000 data rows)

---

## 1. Executive Summary

Using gate preset **v1** (tuned for the prior arm-inclusive policy) with the new
**base-only** policy produces **one significant false-positive entry** into
`force_walking` during an idle period, plus some minor transition-edge effects.
The root cause is a **sustained force-estimator drift** unique to the base-only
policy that the v1 temporal filters are too permissive to reject.

Two new presets are proposed:

| Preset | Description |
|--------|-------------|
| `v2` | Moderate hardening — rejects the observed false positive with ~0.5 s additional entry latency |
| `v2_robust` | Aggressive hardening — wider noise margin, longer hold; ~1.0 s additional entry latency |

---

## 2. Gate Preset V1 Parameters

| Parameter | V1 Value | Role |
|-----------|----------|------|
| `baseline_tau_s` | 1.0 | EMA time constant for force baseline tracking |
| `baseline_margin_n` | 0.34 | Dead-zone above baseline before excess counts |
| `baseline_update_max_n` | 1.00 | Max cap above baseline that still updates baseline |
| `evidence_cap_n` | 4.0 (CLI default) | Hard cap on force_xy used for scoring |
| `enter_threshold_n` | 2.30 | Excess needed to fully saturate enter target |
| `exit_threshold_n` | 0.15 | Excess below which exit target saturates |
| `enter_tau_on_s` | 0.08 | EMA τ for enter_score *rising* |
| `enter_tau_off_s` | 0.22 | EMA τ for enter_score *falling* |
| `exit_tau_on_s` | 0.04 | EMA τ for exit_score rising |
| `exit_tau_off_s` | 0.10 | EMA τ for exit_score falling |
| `enter_score_threshold` | 0.90 | Score above which enter_timer counts |
| `exit_score_threshold` | 0.60 | Score above which exit_timer counts |
| `dir_consistency_min_force_n` | 1.00 | Min force for direction tracking |
| `dir_consistency_tau_s` | 0.16 | EMA τ for direction consistency |
| `dir_consistency_threshold` | 0.72 | Min dir consistency for non-zero dir_term |
| `switch_cooldown_s` | 1.00 | Post-transition cooldown |
| `enter_hold_s` | 0.08 | Sustained high score needed to enter |
| `exit_hold_s` | 0.04 | Sustained high score needed to exit |

---

## 3. All Mode Transitions (8 total)

| # | Step | Time | From → To | Phase | Verdict |
|---|------|------|-----------|-------|---------|
| 1 | 42 | 0.84 s | standing → force_walking | `forward_pull` | ✅ Correct |
| 2 | 659 | 13.18 s | force_walking → standing | `gap` | ✅ Correct |
| 3 | 820 | 16.40 s | standing → force_walking | `backward_pull` | ✅ Correct |
| 4 | 1428 | 28.56 s | force_walking → standing | `tail_idle` | ✅ Correct |
| 5 | **1597** | **31.94 s** | **standing → force_walking** | **`tail_idle`** | **❌ FALSE POSITIVE** |
| 6 | 1648 | 32.96 s | force_walking → standing | `tail_idle` | ⚠️ Recovery from #5 |
| 7 | 1650 | 33.00 s | standing → command_walking | `tail_idle` | ⚠️ Command-driven (separate) |
| 8 | 2900 | 58.00 s | command_walking → standing | `walk_left_pull_high` | ✅ Force target drops to ~0 |

---

## 4. False Positive Root-Cause Analysis (Transition #5)

### 4.1 Timeline

During `tail_idle` (no external force, `force_target_n ≈ 0`), the force
estimator enters a **self-exciting drift mode**:

```
Step 1560 (31.20s): F_raw = 0.359 N  ← quiet
Step 1571 (31.42s): F_raw = 0.727 N  ← first non-zero excess
Step 1582 (31.64s): F_raw = 1.799 N  ← baseline update freezes (cap > bl + 1.00)
Step 1591 (31.82s): F_raw = 4.013 N  ← hits evidence cap (4.0 N)
Step 1594 (31.88s): enter_score = 0.909 ← crosses 0.90 → timer starts
Step 1597 (31.94s): enter_timer = 0.08s ← triggers force_walking ❌
Step 1604 (32.08s): F_raw = 2.499 N  ← drift collapses
Step 1609 (32.18s): F_raw = 0.436 N  ← back to normal
Step 1648 (32.96s): exit triggers      ← recovery to standing
```

### 4.2 Mechanism

1. **Sustained exponential ramp** — not random spikes.  The force estimator
   output grows from 0.36 → 6.31 N over **0.74 s** with a smooth exponential
   profile. This is likely a recurrent-state artifact in the policy network.

2. **Baseline freezes too early** — `baseline_update_max_n = 1.00` means the
   baseline stops tracking when `force_xy_cap > baseline + 1.00 ≈ 1.46 N`
   (around step 1582). After that, all additional signal is counted as excess.

3. **Evidence cap saturation** — once F_raw > 4.0 the excess locks at
   `4.0 − 0.456 − 0.34 = 3.204 N`, well above `enter_threshold_n = 2.30`.

4. **Fast score accumulation** — `enter_tau_on_s = 0.08 s` means the score
   reaches 0.90 in about 11 steps (0.22 s) from first non-zero enter_target.

5. **Short hold** — `enter_hold_s = 0.08 s` (4 steps) is easily satisfied by
   the sustained drift.

### 4.3 Comparison: Real Force vs. False Positive

| Feature | Real force (`forward_pull`) | False positive (`tail_idle`) |
|---------|----------------------------|------------------------------|
| F_raw peak | 25–31 N | 5–6 N |
| Ramp from 0 → cap | ~0.10 s | ~0.60 s |
| Sustained above cap | **seconds** (entire pull) | **~0.3 s** before collapse |
| Underlying mechanism | Actual external force | Force-estimator drift |

The two cases are **indistinguishable** after the evidence cap at 4.0 N in terms
of force_excess (both saturate at ~3.2 N). However, they differ in **temporal
persistence** — real force stays above cap for seconds, while the drift collapses
within ~0.3–0.5 s.

---

## 5. Velocity Observation: Robot Moves During Standing?

User observed apparent motion with zero velocity commands.

### 5.1 Analysis

| Period | vel_x mean | vel_x max | vel_y mean | vel_y max |
|--------|-----------|-----------|-----------|-----------|
| All standing | 0.013 m/s | 0.779 m/s | 0.001 m/s | 0.196 m/s |
| tail_idle standing (1429–1596) | −0.001 m/s | 0.028 m/s | −0.002 m/s | 0.074 m/s |

**Finding:** velocity commands during standing are all exactly zero. The typical
standing-mode velocity is < 0.03 m/s (essentially stationary). The vel_x max of
0.779 m/s is an outlier at transition edges.

**Root cause of perceived motion:** The false positive at step 1597 triggers
`force_walking`, which activates the gait phase clock. Once the gait oscillator
is running, periodic leg motions create net displacement even without velocity
commands, producing visible robot drift (vel_x up to 0.070 m/s during the false
force_walking period).

---

## 6. Force Estimator Noise Profile

| Period | N | F_raw mean | F_raw stdev | F_raw max | % > 3 N |
|--------|---|-----------|-------------|----------|---------|
| Initial standing (0–0.6 s) | 42 | 6.03 | 10.17 | 30.05 | — |
| gap standing (12.5–15.5 s) | 117 | 0.34 | 0.26 | 1.21 | 0.0% |
| tail_idle standing (28.6–31.9 s) | 168 | 0.77 | 1.04 | 5.91 | 5.4% |

The initial standing period has very high noise (mean 6 N) because the policy's
recurrent state is un-primed. After sufficient history, the noise floor settles
to ~0.3–0.8 N but with occasional exponential drift bursts (P95 = 3.0 N,
P99 = 5.5 N during tail_idle standing).

---

## 7. Proposed New Presets

### Design Principles

The key discriminator between real force and drift is **temporal persistence**.
The v2 presets leverage this by:

1. **Slower score accumulation** (`enter_tau_on_s` ↑) to dampen brief drift
2. **Longer hold requirement** (`enter_hold_s` ↑↑) to reject drift that
   collapses within ~0.5 s
3. **Wider noise margin** (`baseline_margin_n` ↑) to absorb more baseline noise
4. **Higher baseline tracking range** (`baseline_update_max_n` ↑) so the
   baseline absorbs moderate drift before freezing
5. **Higher enter threshold** (`enter_threshold_n` ↑) to reduce enter-target
   saturation from noise

### 7.1 Preset `v2` (Moderate Hardening)

```python
GATE_PRESET_V2: {
    "baseline_tau_s": 1.2,         # 1.0→1.2: slightly smoother baseline
    "baseline_margin_n": 0.60,     # 0.34→0.60: wider dead zone
    "baseline_update_max_n": 1.50, # 1.00→1.50: track drift longer
    "enter_threshold_n": 3.00,     # 2.30→3.00: harder to saturate
    "exit_threshold_n": 0.15,      # unchanged
    "enter_tau_on_s": 0.20,        # 0.08→0.20: 2.5× slower rise
    "enter_tau_off_s": 0.22,       # unchanged
    "exit_tau_on_s": 0.04,         # unchanged
    "exit_tau_off_s": 0.10,        # unchanged
    "enter_score_threshold": 0.90, # unchanged
    "exit_score_threshold": 0.60,  # unchanged
    "dir_consistency_min_force_n": 1.00,  # unchanged
    "dir_consistency_tau_s": 0.16, # unchanged
    "dir_consistency_threshold": 0.72,    # unchanged
    "switch_cooldown_s": 1.20,     # 1.00→1.20: slightly longer cooldown
    "enter_hold_s": 0.30,          # 0.08→0.30: 3.75× longer hold
    "exit_hold_s": 0.04,           # unchanged
}
```

**Expected behavior:**
- False positive at tail_idle: **REJECTED** — drift collapses before hold
  completes
- Real force entry latency: ~1.4 s (vs 0.8 s with v1 → +0.6 s)
- Exit behavior: unchanged

### 7.2 Preset `v2_robust` (Aggressive Hardening)

```python
GATE_PRESET_V2_ROBUST: {
    "baseline_tau_s": 1.5,         # slower baseline
    "baseline_margin_n": 0.80,     # wider dead zone
    "baseline_update_max_n": 2.00, # even more drift absorption
    "enter_threshold_n": 2.80,     # high threshold (fixed from 3.50; see §9.2)
    "exit_threshold_n": 0.15,      # unchanged
    "enter_tau_on_s": 0.30,        # 3.75× slower rise
    "enter_tau_off_s": 0.25,       # slightly slower decay
    "exit_tau_on_s": 0.04,         # unchanged
    "exit_tau_off_s": 0.10,        # unchanged
    "enter_score_threshold": 0.92, # slightly higher
    "exit_score_threshold": 0.60,  # unchanged
    "dir_consistency_min_force_n": 1.00,  # unchanged
    "dir_consistency_tau_s": 0.20, # slightly slower
    "dir_consistency_threshold": 0.72,    # unchanged
    "switch_cooldown_s": 1.50,     # longer cooldown
    "enter_hold_s": 0.50,          # 6.25× longer hold
    "exit_hold_s": 0.04,           # unchanged
}
```

**Expected behavior:**
- False positive at tail_idle: **STRONGLY REJECTED**
- Real force entry latency: ~1.8 s (vs 0.8 s with v1 → +1.0 s)
- Exit behavior: unchanged

---

## 8. Testing Plan

```bash
# Test v2 preset
bash scripts/mujoco/run_in_xpra.sh python scripts/mujoco/smart_gait.py \
  --policy-path logs/rsl_rl/uniFP_aliengo_position_force/2026-04-21_17-33-58_base_only/mujoco_exported/policy.pt \
  --policy-history-length 32 \
  --walking-gait-frequency 2.0 \
  --force-schedule human_guide \
  --obs-delay-steps 2 \
  --leg-action-delay-steps 1 \
  --real-time \
  --log-interval 30 \
  --output-csv logs/experiments/unifp_smart_gait_mujoco/base_only_v2.csv \
  --gate-preset v2

# Test v2_robust preset
bash scripts/mujoco/run_in_xpra.sh python scripts/mujoco/smart_gait.py \
  --policy-path logs/rsl_rl/uniFP_aliengo_position_force/2026-04-21_17-33-58_base_only/mujoco_exported/policy.pt \
  --policy-history-length 32 \
  --walking-gait-frequency 2.0 \
  --force-schedule human_guide \
  --obs-delay-steps 2 \
  --leg-action-delay-steps 1 \
  --real-time \
  --log-interval 30 \
  --output-csv logs/experiments/unifp_smart_gait_mujoco/base_only_v2_robust.csv \
  --gate-preset v2_robust
```

**Success criteria:**
1. No false-positive `force_walking` entries during idle phases
2. Correct entry into `force_walking` within ≤ 2.0 s of force onset
3. Exit from `force_walking` within ≤ 1.5 s of force removal
4. No visible robot drift during standing mode

---

## 9. Experimental Results (Round 1)

### 9.1 Preset `v2` — ✅ SUCCESS

| Metric | v1 (baseline) | v2 |
|--------|---------------|-----|
| Total transitions | 8 | 6 |
| False force_walking in idle (F_tgt<0.5) | ~51 steps | 29 steps (carryover only) |
| False positive entries in idle | **1** | **0** |
| Forward_pull entry latency | 0.84 s | 1.84 s (+1.0 s) |
| Backward_pull entry latency | 0.88 s | 1.54 s (+0.66 s) |
| Standing vel_x mean | 0.013 m/s | 0.017 m/s |

**Verdict:** The tail_idle false positive is **eliminated**. The 29 residual
force_walking steps in idle are carryover from the prior `left_pull` phase (exit
delay, steps 1401–1426), not new false entries. Entry latency increased ~1.0 s,
which is acceptable for human-guide scenarios.

### 9.2 Preset `v2_robust` (original) — ❌ OVER-SUPPRESSED

| Metric | v1 (baseline) | v2_robust (original) |
|--------|---------------|----------------------|
| Total transitions | 8 | **2** (command-driven only) |
| Force_walking entries | 4 | **0** (none at all!) |
| All pull phases mode | force_walking | **standing** |

**Root cause:** Parameter contradiction — `enter_threshold_n=3.50` with
`evidence_cap_n=4.0` and `baseline_margin_n=0.80` means:

```
max_excess = 4.0 − baseline(~0.15) − 0.80 = 3.05 N
max_enter_target = 3.05 / 3.50 = 0.871
```

The EMA score converges to **0.871**, which can **never** reach
`enter_score_threshold=0.92`. Force-walking is effectively disabled.

**Fix applied:** `enter_threshold_n` reduced from **3.50 → 2.80**, giving:

```
max_enter_target = 3.05 / 2.80 = 1.089 → clamped to 1.0 ✓
```

The temporal robustness now comes entirely from `enter_tau_on_s=0.30` and
`enter_hold_s=0.50`, which should reject drift lasting < 0.5 s while allowing
sustained force (> 2 s) to trigger. **Superseded by v2 Round 2 below.**

---

## 10. Round 2: Dynamic-Range Approach (v2 Redesigned)

### 10.1 Problem with Round 1

Round 1's v2 presets used **aggressive temporal filtering** (slower EMA, longer
hold) to reject noise. This worked but increased entry latency to 1.84 s — far
above the arm-policy reference of 0.47 s avg.

### 10.2 Key Insight

The evidence cap at 4.0 N **compresses** both real force (20–40 N raw) and drift
noise (5–6 N raw) to the same 4.0 N ceiling. After capping, both produce
identical `force_excess ≈ 3.2 N`, making them indistinguishable in the scoring
pipeline.

**Solution:** Raise `evidence_cap_n` to **10.0 N** and proportionally raise
`enter_threshold_n` to **8.0 N**. This:

1. Real force (20–40 N raw → cap 10.0) → `excess ≈ 9.5` → `target = 9.5/8.0 = 1.0` ✓
2. Drift noise (6 N raw → not capped) → `excess ≈ 5.5` → `target = 5.5/8.0 = 0.69` → score converges to 0.69, well below threshold 0.90 ✓

All v1 temporal parameters are preserved → **no additional latency**.

### 10.3 Offline Simulation

Replayed original CSV force data with new parameters:

| Scenario | Score reached | Triggered? | Latency from onset |
|----------|--------------|------------|---------------------|
| `forward_pull` | 0.917 at step 40 | ✅ step 45 (0.90 s) | ~0.30 s |
| `tail_idle` drift | max 0.01 | ❌ NOT triggered | — |
| `backward_pull` | 0.912 at step 820 | ✅ step 825 (16.50 s) | ~0.38 s |

Entry latency: **0.30–0.38 s** (well within arm-policy average of 0.47 s).

### 10.4 New Preset Definitions

**`v2`** — Widen dynamic range, keep v1 speed:

```python
GATE_PRESET_V2: {
    "evidence_cap_n": 10.0,        # 4.0→10.0: key change
    "baseline_tau_s": 1.0,         # same as v1
    "baseline_margin_n": 0.34,     # same as v1
    "baseline_update_max_n": 1.00, # same as v1
    "enter_threshold_n": 8.00,     # 2.30→8.00: proportional to new cap
    "exit_threshold_n": 0.15,      # same
    "enter_tau_on_s": 0.08,        # same (preserve latency!)
    "enter_tau_off_s": 0.22,       # same
    "enter_score_threshold": 0.90, # same
    "enter_hold_s": 0.12,          # 0.08→0.12: slight safety margin
    ...remaining same as v1...
}
```

**`v2_robust`** — Wider margin + slightly slower temporal:

```python
GATE_PRESET_V2_ROBUST: {
    "evidence_cap_n": 10.0,
    "baseline_margin_n": 0.50,     # slightly wider
    "enter_threshold_n": 8.50,     # slightly higher
    "enter_tau_on_s": 0.12,        # slightly slower
    "enter_hold_s": 0.20,          # moderate hold
    "switch_cooldown_s": 1.20,     # slightly longer
    ...remaining same as v1...
}
```

### 10.5 Testing Commands

```bash
# v2 (recommended)
bash scripts/mujoco/run_in_xpra.sh python scripts/mujoco/smart_gait.py \
  --policy-path logs/rsl_rl/uniFP_aliengo_position_force/2026-04-21_17-33-58_base_only/mujoco_exported/policy.pt \
  --policy-history-length 32 \
  --walking-gait-frequency 2.0 \
  --force-schedule human_guide \
  --obs-delay-steps 2 \
  --leg-action-delay-steps 1 \
  --real-time \
  --log-interval 30 \
  --output-csv logs/experiments/unifp_smart_gait_mujoco/base_only_v2.csv \
  --gate-preset v2

# v2_robust
bash scripts/mujoco/run_in_xpra.sh python scripts/mujoco/smart_gait.py \
  --policy-path logs/rsl_rl/uniFP_aliengo_position_force/2026-04-21_17-33-58_base_only/mujoco_exported/policy.pt \
  --policy-history-length 32 \
  --walking-gait-frequency 2.0 \
  --force-schedule human_guide \
  --obs-delay-steps 2 \
  --leg-action-delay-steps 1 \
  --real-time \
  --log-interval 30 \
  --output-csv logs/experiments/unifp_smart_gait_mujoco/base_only_v2_robust.csv \
  --gate-preset v2_robust
```
```
