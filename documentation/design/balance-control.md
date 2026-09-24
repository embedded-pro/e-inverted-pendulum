---
title: "Balance Control Design"
type: design
status: draft
version: 0.2.0
component: "balance-control"
date: 2026-09-24
---

| Field     | Value                  |
|-----------|------------------------|
| Title     | Balance Control Design |
| Type      | design                 |
| Status    | draft                  |
| Version   | 0.2.0                  |
| Component | balance-control        |
| Date      | 2026-09-24             |

> **The control law is a runtime choice, not an architectural commitment.** This document
> designs the seam that makes that true: one strategy interface, several interchangeable
> implementations, and a selection mechanism confined to a state where swapping is safe.

---

## Responsibilities

**Is responsible for:**
- Defining the single interface every control strategy implements.
- Converting the estimated state and the operator setpoints into a per-wheel effort command.
- Hosting the available strategies and tracking which one is active.
- Publishing each strategy's tunable parameter set in a self-describing form, so a client
  can tune a strategy it has no built-in knowledge of.
- Enforcing that a strategy change or a parameter write happens only while not ARMED.
- Resetting the active strategy's internal state on every transition into ARMED.
- Saturating the effort command and preventing internal state from growing while saturated.

**Is NOT responsible for:**
- Estimating attitude or wheel motion — it consumes both.
- Deciding whether the drive may be energised — the safety supervisor owns that.
- Mapping effort onto bridge duty cycle and direction — that is motion actuation.
- Transporting parameters over the link — connectivity carries them; this component
  defines what they mean and validates them.
- Persisting parameters — it supplies and accepts values; the platform stores them.

---

## Component Details

### Part A — The strategy interface

Every strategy is a pure function of state plus memory: it receives the estimated pitch and
pitch rate, the measured chassis velocity and yaw rate, and the commanded velocity and yaw
setpoints; it returns a signed effort for each wheel. Alongside that it offers three
lifecycle operations — reset its internal state, describe its parameters, and get or set a
parameter by index.

The interface is deliberately narrow. Nothing in it mentions gains, error terms, state
vectors or matrices, because those are properties of particular laws. This is what allows
the requirements to be written as outcomes and to be satisfied by any conforming strategy.

### Part B — Cascaded PID strategy

An outer loop compares commanded and measured chassis velocity and produces a *pitch
setpoint* — to go faster, lean further forward. An inner loop drives the estimated pitch to
that setpoint and produces a common-mode effort. A third loop drives yaw rate to its
setpoint and produces a differential effort. The two are summed per wheel.

The nesting is what makes it work: the inner loop must be substantially faster than the
outer, or the robot chases a pitch target that is still moving. The inner loop runs every
balance iteration; the velocity and yaw loops run at the outer rate. Its parameters are the
proportional, integral and derivative terms of the three loops.

Each loop is an incremental PID whose output is held within its own limits: the pitch
setpoint within the lean limit, the differential effort within the differential limit, and the
common-mode effort within the actuator range. Gains are expressed in continuous units — integral
per second, derivative in seconds — and are discretised with the measured interval of every run, so
a stretched iteration does not silently change the tuning. The inner loop takes its derivative
from the measured pitch rate rather than from the change in pitch error: the gyroscope already
measures the rate directly, and differentiating the error would kick every time the outer loop
moves the pitch setpoint.

The initial gains are conservative placeholders to be tuned on the bench.

| Index | Parameter   | Unit                      | Range    | Initial |
|-------|-------------|---------------------------|----------|---------|
| 0     | pitch.kp    | effort per radian         | 0 to 50  | 2.0     |
| 1     | pitch.ki    | effort per radian-second  | 0 to 100 | 0.0     |
| 2     | pitch.kd    | effort per radian/second  | 0 to 5   | 0.1     |
| 3     | velocity.kp | radian per metre/second   | 0 to 2   | 0.05    |
| 4     | velocity.ki | radian per metre          | 0 to 5   | 0.0     |
| 5     | velocity.kd | radian per metre/second²  | 0 to 1   | 0.0     |
| 6     | yaw.kp      | effort per radian/second  | 0 to 2   | 0.1     |
| 7     | yaw.ki      | effort per radian         | 0 to 5   | 0.0     |
| 8     | yaw.kd      | effort per radian/second² | 0 to 1   | 0.0     |

### Part C — Full-state feedback (LQR) strategy

A single gain vector maps the deviation of the state — pitch, pitch rate, wheel position,
wheel velocity — onto a common-mode effort, with a separate yaw term for the differential
part. The gains come from solving the linearised regulator problem offline; the firmware
evaluates one inner product per iteration. Its parameters are the elements of that gain
vector, which is why the parameter descriptor must be per-strategy rather than a fixed
layout of named gains.

### Part D — Strategy registry and selection

The registry holds the compiled-in strategies, exposes their identifiers, and tracks the
active one. The cascaded PID strategy is the default. Selection is refused while ARMED. This is not a convenience restriction: the
strategies have different internal state, and swapping mid-flight would apply a
freshly-reset controller to a robot already in motion. Refusal is silent to the drive — the
running controller is not disturbed by a rejected request.

### Part E — Setpoint arbitration and saturation

Setpoints reach the controller from the operator, but the controller does not trust them
unconditionally. They are range-checked, decayed to zero on operator silence or link loss,
and forced to zero whenever the system is not ARMED. Until the link exists the operator commands
them from the terminal; a command is accepted only while ARMED and within range, and holds for one
second before it falls back to zero. Every engagement and disengagement also clears them, so a
command from a previous session never carries into the next. Effort is clamped to the actuator
range on output, and the active strategy is told that it saturated so that any accumulating
internal term stops growing — otherwise a robot held against a wall builds up a correction
it discharges violently on release. The strategy is handed the effort that was actually applied,
and restarts its accumulation from that value rather than from what it asked for.

### Part F — Engagement

The supervisor engages balance control on every transition into ARMED and disengages it on every
transition out. Engaging resets the active strategy and clears the setpoints before the drive is
permitted; disengaging stops the strategy and clears the setpoints again. Balance control uses the
same two events to decide when it is configurable, so it never has to ask the supervisor for its
mode. An invalid estimate reaching an engaged controller produces zero effort.

---

## Interfaces

### Provided

| Interface            | Purpose                                                  | Contract                                                                                                                    |
|----------------------|----------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------|
| Control strategy     | The common interface every law implements                | Consumes estimated state and setpoints, produces per-wheel effort; bounded execution, no allocation, no recursion           |
| Effort command       | Per-wheel signed effort for actuation                    | Within the configured actuator range; zero whenever not ARMED                                                               |
| Strategy registry    | Enumerate available strategies and report the active one | At least two strategies available; identifiers stable across builds                                                         |
| Strategy selection   | Change the active strategy                               | Accepted only while not ARMED; a rejected request leaves the active strategy and its state untouched                        |
| Parameter descriptor | Describe the active strategy's tunable parameters        | Reports count, order, identity and permitted range; changes when the active strategy changes                                |
| Parameter access     | Read and write parameters by index                       | Writes accepted only while not ARMED; out-of-range values rejected with the stored value unchanged                          |
| Strategy lifecycle   | Engage and disengage balance control                     | Engaged by the supervisor on every transition into ARMED, before the drive is permitted; disengaged on every transition out |
| Motion setpoint      | Command forward velocity and yaw rate                    | Accepted only while engaged and within range; falls back to zero after 1 s without a new command                            |

### Required

| Interface         | Purpose                                 | Contract                                                                |
|-------------------|-----------------------------------------|-------------------------------------------------------------------------|
| Attitude estimate | Pitch and pitch rate with validity      | An invalid estimate must not produce a non-zero effort                  |
| Chassis motion    | Measured forward velocity and yaw rate  | Updated at the control-loop rate                                        |
| Motion setpoints  | Commanded forward velocity and yaw rate | Already range-checked and decayed by connectivity; zero when not ARMED  |
| Drive permission  | Whether the drive may be energised      | Signalled by engagement; while disengaged no effort is produced         |
| Timebase          | Loop period for rate-dependent terms    | Nominal period is known; actual jitter stays within the specified bound |

---

## Data Model

| Entity               | Field                   | Type / Unit                | Range                           | Notes                                           |
|----------------------|-------------------------|----------------------------|---------------------------------|-------------------------------------------------|
| Estimated state      | pitch                   | radians                    | -0.61 to 0.61                   | Beyond this the supervisor has already disarmed |
| Estimated state      | pitchRate               | radians per second         | -8.7 to 8.7                     | Bias-corrected                                  |
| Estimated state      | chassisVelocity         | metres per second          | -1.5 to 1.5                     | Derived from both wheels                        |
| Estimated state      | yawRate                 | radians per second         | -3.1 to 3.1                     | Derived from the wheel difference               |
| Setpoints            | velocitySetpoint        | metres per second          | -1.0 to 1.0                     | Decays to zero on operator silence              |
| Setpoints            | yawRateSetpoint         | radians per second         | -1.6 to 1.6                     | Decays to zero on operator silence              |
| Output               | effortLeft, effortRight | normalised effort          | -1.0 to 1.0                     | Mapped to duty and direction by actuation       |
| Parameter descriptor | index                   | count                      | 0 to parameterCount-1           | Position is the identity used over the link     |
| Parameter descriptor | minimum, maximum        | same unit as the parameter | strategy-defined                | Writes outside the range are rejected           |
| Registry             | activeStrategy          | identifier                 | one of the available strategies | Changeable only while not ARMED                 |

---

## State Machine

The controller's own lifecycle, driven entirely by the supervisor's drive permission.

```mermaid
stateDiagram-v2
    [*] --> Configurable
    Configurable --> Configurable : Select strategy / write parameter
    Configurable --> Running : Drive permitted (strategy state reset first)
    Running --> Running : Compute effort each period
    Running --> Saturated : Effort clamped at the actuator range
    Saturated --> Running : Effort within range
    Saturated --> Saturated : Internal accumulation held
    Running --> Configurable : Drive permission withdrawn
    Saturated --> Configurable : Drive permission withdrawn
    Configurable --> Configurable : Selection or write rejected while Running
```

---

## Sequence Diagrams

One balance iteration.

```mermaid
sequenceDiagram
    participant Est as Attitude estimation
    participant Odom as Wheel odometry
    participant Ctrl as Balance control
    participant Strat as Active strategy
    participant Act as Motion actuation

    Est->>Ctrl: Pitch, pitch rate, valid
    Odom->>Ctrl: Chassis velocity, yaw rate
    Ctrl->>Ctrl: Apply current setpoints
    Ctrl->>Strat: Compute(state, setpoints)
    Strat-->>Ctrl: Effort left, effort right
    Ctrl->>Ctrl: Saturate to actuator range
    Ctrl->>Strat: Report saturation
    Ctrl->>Act: Per-wheel effort
```

Selecting a different strategy, and being refused while flying.

```mermaid
sequenceDiagram
    participant Op as Operator
    participant Link as Connectivity
    participant Ctrl as Balance control
    participant Sup as Safety supervisor

    Op->>Link: Select strategy B
    Link->>Ctrl: Selection request
    Note over Ctrl: Disengaged since the last disarm
    Ctrl->>Ctrl: Activate strategy B
    Ctrl-->>Link: Accepted, descriptor changed
    Link-->>Op: New parameter descriptor

    Note over Op,Sup: Later, while balancing
    Op->>Link: Select strategy A
    Link->>Ctrl: Selection request
    Note over Ctrl,Sup: Engaged by the supervisor on arming
    Ctrl-->>Link: Rejected
    Note over Ctrl: Strategy B keeps running, undisturbed
```

---

## Block Diagram

The cascade, drawn for the PID strategy. A full-state strategy replaces the two nested
blocks with a single gain block; everything outside the dashed boundary is unchanged.

```mermaid
graph LR
    VSP[Velocity setpoint] -->|error| OUTER[Outer velocity loop]
    VMEAS[Measured velocity] -->|feedback| OUTER
    OUTER -->|pitch setpoint| INNER[Inner pitch loop]
    PITCH[Estimated pitch and rate] -->|feedback| INNER
    INNER -->|common-mode effort| SUM[Per-wheel mixer]
    YSP[Yaw rate setpoint] -->|error| YAW[Yaw loop]
    YMEAS[Measured yaw rate] -->|feedback| YAW
    YAW -->|differential effort| SUM
    SUM -->|effort left, effort right| SAT[Saturation]
    SAT --> OUT[To actuation]
    SAT -->|saturation flag| INNER
    SAT -->|saturation flag| OUTER
```

---

## Constraints & Limitations

| Constraint             | Value / Description                                                                                                                                                                                                                         |
|------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Iteration budget       | The complete iteration — read state, dispatch to the strategy, saturate, publish effort — fits within the 500 Hz period and the 3 ms sensor-to-actuator latency                                                                             |
| Strategy dispatch      | One indirect call per iteration, in task context. Not reachable from an interrupt handler, so the project's prohibition on virtual dispatch in interrupt paths is not engaged. The cost is fixed once ARMED because selection cannot change |
| Memory                 | Strategies are constructed once at startup. No allocation on the control path; the registry has a compile-time fixed capacity                                                                                                               |
| Inner-outer separation | The cascade assumes the inner pitch loop is materially faster than the outer velocity loop; violating that produces a slow instability rather than an obvious failure                                                                       |
| Small-angle validity   | Strategies are tuned against a model linearised about upright; behaviour degrades as the body approaches the fall threshold                                                                                                                 |
| Level ground           | No slope compensation. On an incline the robot holds a pitch offset and drifts unless the operator commands against it                                                                                                                      |
| Parameter identity     | Parameters are addressed by index, not by name, to keep the link payload bounded. Reordering a strategy's parameters is a breaking change for stored values                                                                                 |

---

## Open Questions

| # | Question                                                                                                         | Options                                                                      | Status                       |
|---|------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------|------------------------------|
| 1 | Which strategy is the factory default?                                                                           | Cascaded PID; LQR                                                            | decided: cascaded PID        |
| 2 | Should stored parameters be invalidated when a strategy's descriptor changes between firmware versions?          | Version the descriptor and reject stale values; always fall back to defaults | open                         |
| 3 | Should the outer velocity loop limit the pitch setpoint it may request, independently of effort saturation?      | Rely on effort saturation; add an explicit pitch setpoint clamp              | decided: clamp at 10 degrees |
| 4 | Should a third strategy exist for bench testing, commanding zero effort while reporting what it would have done? | Not needed; add an observing strategy                                        | open                         |
| 5 | How is the yaw term handled by a full-state strategy — inside the gain vector or as a separate loop?             | Separate yaw loop for both strategies; per-strategy choice                   | open                         |
