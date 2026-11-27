---
title: "Wheel Odometry Design"
type: design
status: draft
version: 0.3.2
component: "wheel-odometry"
date: 2026-11-27
---

| Field     | Value                 |
|-----------|-----------------------|
| Title     | Wheel Odometry Design |
| Type      | design                |
| Status    | draft                 |
| Version   | 0.3.2                 |
| Component | wheel-odometry        |
| Date      | 2026-11-27            |

> Turns two sixteen-bit hardware counters that wrap every revolution into a signed wheel
> position that never wraps, a wheel angular velocity, and the chassis forward velocity and
> yaw rate the outer control loop steers on.

---

## Responsibilities

- Sampling both encoder counters on a fixed cadence that it owns.
- Accumulating a signed per-wheel position that survives hardware counter wrap-around
  without losing or duplicating counts.
- Deriving a signed angular velocity per wheel from the counts observed in the last interval.
- Deriving chassis forward velocity and yaw rate from the two wheel velocities and the
  documented wheel geometry.
- Holding the wheel geometry — radius, track width, gear ratio — in exactly one place.

---

## Component Details

### Part A — The hardware decodes, this component accumulates

Each wheel has a hardware quadrature decoder. The decoder counts quadrature edges on both
channels and presents a counter in the range zero to one below the configured resolution.
It wraps in both directions and gives no indication that it has done so.

This component adds one thing over the raw counter. It turns that wrapping, resolution-bounded
number into a wide signed position. That position only changes by the amount the wheel actually
turned. On each sample the component reads the counter and forms the difference against the
previous reading. It folds that difference into the shorter of the two ways around the counter.
A difference of more than half the resolution is reinterpreted as the same journey taken the
other way. A wheel that runs forwards past the counter maximum therefore reads as a small
positive step, not as a revolution backwards. The accumulated position continues monotonically.

A difference of *exactly* half the resolution is the one case no rule can get right. A half-turn
forwards and a half-turn backwards produce the identical pair of counter readings. The
information needed to tell them apart is not present. The tie is broken deterministically, not
arbitrarily. The raw difference is kept as it stands, so the reported sign follows the order of
the two readings. Reading 0 then half the resolution is reported as forward. Reading half the
resolution then 0 is reported as reverse. Part B says why the cadence keeps the robot well clear
of this point. The alternative is to refuse to run when a wheel reaches it. That is the wrong
failure mode for a machine that falls over when it stops balancing.

The first sample after construction only seeds the previous reading. It cannot produce a
displacement, because there is nothing to difference against. Inventing one would attribute the
counter's arbitrary power-on value to motion that never happened. Position and velocity both stay
at zero until the second sample.

### Part B — The sampling cadence is a correctness constraint, not a tuning knob

Shortest-path reconstruction is only correct while the wheel turns by less than half the
resolution between two samples. Past that, the reconstruction silently chooses the wrong
direction. The error is not a small one: it is a whole revolution, in the wrong sign.

That makes the sample period part of the component's contract, not something a caller may set
freely. For this reason the component owns its own periodic sampling. It does not wait to be
driven by whatever loop consumes it. A control loop that stalls, runs slow, or is not yet written
cannot corrupt the accumulated position.

With the default resolution of 4096 counts per revolution and a 20 millisecond period, the
ceiling is just under half a revolution per sample. Half itself is the ambiguous case described
in Part A, so the last unambiguous count is one below it:

| Quantity                       | Value                      |
|--------------------------------|----------------------------|
| Counts per revolution          | 4096                       |
| Sample period                  | 20 ms                      |
| Unambiguous counts per sample  | 2047                       |
| Maximum trackable wheel rate   | under 25 rev/s ≈ 1499 rpm  |
| Maximum trackable ground speed | ≈ 5.3 m/s at a 34 mm wheel |

The chassis is designed for at most 1.5 m/s, so the margin is better than a factor of three.
The figure is recorded because it moves. A finer encoder, a longer period, or a gearbox between
encoder and wheel all lower it. A change to any of the three needs a check against this table.

### Part C — Velocity is differenced position, not the driver's speed reading

The encoder driver also offers a speed reading of its own. It is deliberately not used. That
reading is an unsigned magnitude sampled on the driver's own period, so it carries no direction.
It resolves wrap by trusting the counter's instantaneous direction flag. That flag is the
direction of the last edge, not of the interval. A balance controller needs the sign far more
than it needs the magnitude. Velocity is therefore the displacement of the last interval divided
by the interval. It is signed by construction and consistent with the position it comes from.

The divisor is the interval that was *measured*, not the one that was configured. The platform
abstraction states this as a contract: "the timebase is monotonic and measured intervals, not
nominal ones, are what callers receive". It matters because the repeating timer re-anchors to its
original grid and does not drift. A callback that arrives late does not shift the schedule. It
skips a period. Dividing two periods of counts by one period would report twice the true
velocity. That error would come at exactly the moment the system is already under strain. Each
sample therefore reads the clock and divides by the time that actually passed.

Velocity therefore inherits the counting resolution directly. One count per interval is
0.077 rad/s at the wheel, about 2.6 mm/s of ground speed. Faster sampling would lower the
aliasing ceiling in Part B *and* make velocity noisier. The same one-count quantisation is then
divided by a shorter interval. Twenty milliseconds matches the outer control loop, which is the
only consumer of velocity. The inner balance loop runs on attitude, not on odometry.

### Part D — Chassis motion

The two wheel velocities are combined into the chassis motion the outer loop actually steers on.
Forward velocity is the mean of the two wheel velocities scaled by the wheel radius. Yaw rate is
their difference scaled by the radius over the track width. Driving both wheels equally gives a
pure forward velocity and no yaw. Driving them equally and oppositely gives a pure yaw and no
forward velocity. The forward kinematics come from the differential-drive model in the robotics
toolbox. The model's wheel speed limit is set to the trackable rate of Part B. The forward
kinematics do not use it.

Yaw is positive when the right wheel runs ahead of the left, which is the robot turning to its
left. Both are derived here, beside the geometry they depend on. A consumer therefore never needs
the wheel radius or the track width to interpret what it receives.

### Part E — Sign convention: settled in hardware, overridable at run time

Requirement REQ-ODOM-003 asks that forward robot motion be positive on both wheels. The two
wheels face opposite ways on the chassis, so one encoder would otherwise count down while the
other counts up.

The platform implementation, one layer below, resolves this. It inverts one encoder's
phase-A polarity in hardware. By the time this component reads a counter, both already count
up for forward motion. This component therefore applies no sign correction by default.

The hardware correction assumes how the encoders are wired. If the built robot differs, the
operator inverts one wheel's counts at run time ([calibration](calibration.md)). The setting is
off by default, so it never cancels the hardware correction unless someone sets it. The wheel
geometry is set at run time in the same way.

---

## Interfaces

### Provided

| Interface      | Purpose                                        | Contract                                                                                            |
|----------------|------------------------------------------------|-----------------------------------------------------------------------------------------------------|
| Wheel motion   | Signed position and angular velocity per wheel | Lossless across counter wrap; forward motion positive on both wheels; zero until the second sample  |
| Chassis motion | Forward velocity and yaw rate                  | Derived from both wheel velocities and the documented geometry; refreshed on every sample           |
| Calibration    | Encoder inversion per wheel and the geometry   | Inversion applies from the next sample; a geometry outside the ranges in the data model is rejected |

Apart from calibration, the provided interface is read-only. Consumers observe the latest
estimate. They do not drive the sampling and cannot advance, reset or reseed the accumulated
position.

### Required

| Interface         | Purpose                                    | Contract                                                                                                                                                                                        |
|-------------------|--------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Wheel encoders    | Counter value and resolution per wheel     | Counter strictly below the resolution; resolution at least two and constant                                                                                                                     |
| Periodic timebase | Drive the sampling cadence, and measure it | Monotonic; fires on the configured period and reports the interval that actually elapsed. A skipped sample does not corrupt velocity, but it does halve the ceiling in Part B for that interval |

Boundaries with the neighbouring components:

- The hardware decodes the quadrature edges. This component only reads the resulting counter.
- The platform layer corrects the mirrored wheel mounting. It inverts one encoder phase, so both
  counters already count up for forward robot motion. A second correction here would cancel it.
- The consumer filters, smooths or fuses the estimate. The published estimate is a raw
  difference, and only the consumer knows its own bandwidth.
- A pose integrator, if any, derives heading and floor position. This component reports rates,
  not where the robot is.

---

## Data Model

| Entity         | Field            | Type / Unit        | Range             | Notes                                                                                                                         |
|----------------|------------------|--------------------|-------------------|-------------------------------------------------------------------------------------------------------------------------------|
| Wheel motion   | position         | encoder counts     | full signed range | Accumulated across hardware wrap; zero at construction                                                                        |
| Wheel motion   | angular velocity | radians per second | -105 to 105       | Displacement of the last interval over the sample period                                                                      |
| Chassis motion | forward velocity | metres per second  | -1.5 to 1.5       | Mean of both wheel velocities times the wheel radius                                                                          |
| Chassis motion | yaw rate         | radians per second | -3.1 to 3.1       | Wheel difference times radius over track width; positive left                                                                 |
| Configuration  | sample period    | microseconds       | greater than zero | Sets the cadence and bounds the trackable wheel rate — see Part B. Not used as a divisor; velocity uses the measured interval |
| Configuration  | wheel radius     | metres             | 0.01 to 0.2       | Fitted value; documented in one place only                                                                                    |
| Configuration  | track width      | metres             | 0.05 to 0.5       | Distance between the wheel contact patches                                                                                    |
| Configuration  | gear ratio       | encoder per wheel  | 0.1 to 100        | Encoder revolutions per wheel revolution; one when direct                                                                     |

---

## State Machine

Each wheel carries one bit of state beyond its accumulated position: whether it has ever been
sampled. The bit makes the arbitrary counter value at power-on an origin, not a displacement.

```mermaid
stateDiagram-v2
    [*] --> Unsampled
    Unsampled --> Tracking : first sample, counter stored as the origin
    Tracking --> Tracking : sample, shortest-path difference accumulated
```

There is no path back to Unsampled. Nothing reseeds a wheel once it is tracking. In particular
the index pulse does not, which is what REQ-ODOM-005 protects.

---

## Sequence Diagrams

Steady-state sampling, showing that consumers are decoupled from the cadence:

```mermaid
sequenceDiagram
    participant Timebase
    participant Odometry
    participant Encoders
    participant Consumer

    Timebase->>Odometry: period elapsed
    Odometry->>Encoders: read left counter and resolution
    Encoders-->>Odometry: counter value
    Odometry->>Odometry: shortest-path difference, accumulate, divide by period
    Odometry->>Encoders: read right counter and resolution
    Encoders-->>Odometry: counter value
    Odometry->>Odometry: shortest-path difference, accumulate, divide by period

    Consumer->>Odometry: read chassis motion
    Odometry-->>Consumer: forward velocity, yaw rate
```

Start-up, showing that the first sample yields no motion:

```mermaid
sequenceDiagram
    participant Timebase
    participant Odometry
    participant Encoders

    Timebase->>Odometry: first period elapsed
    Odometry->>Encoders: read counter
    Encoders-->>Odometry: arbitrary power-on value
    Odometry->>Odometry: store as origin, report zero position and zero velocity

    Timebase->>Odometry: second period elapsed
    Odometry->>Encoders: read counter
    Encoders-->>Odometry: counter value
    Odometry->>Odometry: first displacement is produced here
```

---

## Block Diagram

```mermaid
graph LR
    LC[Left counter] --> LD[Shortest-path difference]
    RC[Right counter] --> RD[Shortest-path difference]

    LD -->|counts| LA[Left accumulator]
    RD -->|counts| RA[Right accumulator]
    LD -->|counts per period| LV[Left angular velocity]
    RD -->|counts per period| RV[Right angular velocity]

    LA --> LP[Left position]
    RA --> RP[Right position]

    LV --> CM[Chassis combiner]
    RV --> CM
    GEO[Wheel radius, track width] --> CM
    CM --> FV[Forward velocity]
    CM --> YR[Yaw rate]
```

---

## Constraints & Limitations

| Constraint                   | Value / Description                                                                                                                                                         |
|------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Maximum trackable wheel rate | Just under half the resolution per sample — 2047 counts, under 25 rev/s at 4096 counts and 20 ms. Beyond it the position is wrong by a revolution, silently                 |
| Exactly half the resolution  | Inherently ambiguous; the tie falls to the raw difference, so the sign follows the order the readings were taken in — see Part A                                            |
| Missed sample                | Velocity stays correct — the measured interval is the divisor, so two periods of counts are divided by two periods — but the wrap ceiling above is halved for that interval |
| Velocity quantisation        | One count per period: 0.077 rad/s at the wheel, about 2.6 mm/s of ground speed                                                                                              |
| Velocity is unfiltered       | A single-interval difference; consumers needing smoothness filter at their own bandwidth                                                                                    |
| Geometry is nominal          | Wheel radius, track width and gear ratio are configured values, not measured ones; nothing here detects a wrong one. The defaults are placeholders                          |
| Slip is invisible            | A wheel that spins without the chassis moving reports motion that did not happen. Odometry is dead reckoning                                                                |
| Accumulator range            | Signed 32-bit counts — about 524 288 revolutions at the default resolution, far beyond any run length                                                                       |
| No heap, no recursion        | Fixed-size state, two wheels, no dynamic allocation on any path                                                                                                             |
| No index event               | REQ-ODOM-005 is unmet: the component reports no once-per-revolution index event. See Decisions, row 1                                                                       |


---

## Decisions

| # | Decision                                                                            | Options                                                                                                                                       | Status                                                               |
|---|-------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------|
| 1 | How is the once-per-revolution index event exposed (REQ-ODOM-005)?                  | Extend the portable encoder interface with an index accessor; add an interrupt-driven index latch in the hardware layer; drop the requirement | open — see the note below (#96)                                      |
| 2 | Are the wheel radius, track width and gear ratio defaults the built robot's values? | Measure on the assembled chassis; fit from a straight-line and a spin-in-place run                                                            | open — defaults are placeholders (#97)                               |
| 3 | Should velocity be filtered before the outer loop sees it?                          | Leave raw and let balance control filter; add a configurable first-order filter here                                                          | open — raw for now, decided by the balance loop's noise budget (#98) |

**On decision 1.** The requirement cannot be met through the interface this component consumes.
The portable encoder abstraction exposes counter, resolution, direction and speed. It has no
index accessor at all. The hardware layer does have one, but it is a bare level read of the index
pin. No latch and no interrupt stand behind it. Polling it at the sampling cadence catches the
pulse only occasionally, because the pulse is present for a fraction of a revolution. An index
event that is missed most of the time is worse than none.

A fix needs a change one layer down. That change is an index accessor on the portable
interface, backed by a latch or an edge interrupt in the hardware layer. It needs no change here. Until then
REQ-ODOM-005 is explicitly unmet. It is recorded as unmet, not quietly approximated. The half of
the requirement that *is* met is the half that matters for safety. Nothing in this component lets
an index pulse disturb the accumulated count, because the count never consults it.
