---
title: "Inertial Sensing Design"
type: design
status: draft
version: 0.2.1
component: "imu-sensing"
date: 2026-11-26
---

| Field     | Value                   |
|-----------|-------------------------|
| Title     | Inertial Sensing Design |
| Type      | design                  |
| Status    | draft                   |
| Version   | 0.2.1                   |
| Component | imu-sensing             |
| Date      | 2026-11-26              |

> Turns a six-axis part on a serial bus into bias-corrected angular rate and acceleration in a
> fixed body frame, each sample carrying the instant it was taken and an explicit statement of
> whether it may be believed.

---

## Responsibilities

- Estimating the gyroscope bias of each axis while the robot is held still, and subtracting it
  from every subsequent sample.
- Refusing to calibrate when the robot is not still, and reporting that refusal.
- Marking its output invalid when no sample has arrived for more than three sampling periods.
- Marking its output invalid when a sample is flagged as a failed transfer, rather than
  publishing a value. On the target nothing raises that flag today — see **Constraints**.
- Publishing a validity indication, and the reason for invalidity, with every measurement.

---

## Component Details

### Part A — The part, and what was already written

The fitted part is a six-axis inertial sensor: a three-axis gyroscope and a three-axis
accelerometer, with a magnetometer in the same package. The magnetometer is **not used**. It sits
behind the part's auxiliary bus, and the vendored driver has no support for it. The estimation
theory also establishes that yaw has no absolute reference in this robot and is not integrated.
The magnetometer is present on the die and absent from the design.

The driver itself was not written for this project. The embedded infrastructure library vendors a
complete, tested driver for the part. It has a bus-agnostic core, adapters for both serial buses,
a data-ready interrupt path, self-test and FIFO extensions, and an identification read. This
component adds everything around it: the bus and pins, the frame and units, calibration, and
validity.

### Part B — Sampling is driven by the sensor, not by a loop

The part asserts a data-ready line when a new sample is latched. That line is wired to an
interrupt and the sample is read in response.

The sample is timestamped when the driver delivers it. That is one burst later than the edge
that latched it: about 120 µs at the bus rate below, plus event-dispatch latency. The offset is
near-constant, not jitter. It is stated here so that the design does not claim a capture instant
where it has a delivery instant. Closing the gap means timestamping inside the driver's interrupt
handler. That is a change to the vendored driver, not to this component.

The alternative, polling on a periodic timer, was rejected. Both the estimator's required
interface and the platform timebase contract say consumers get the *measured* interval, never the
nominal one. The estimation theory names sampling-interval jitter as its third largest error
source, because Δt enters the integration directly. Polling at a rate unrelated to the sensor's
own cadence produces samples of varying and unknown age. It also reports the poller's interval,
not the sensor's. That was the exact defect corrected in wheel odometry. Repeating it here would
be worse, because the pitch estimate integrates this signal.

This settles the platform abstraction's decision on whether the inertial role exposes raw samples
or a board-driven callback. It is a callback. The board owns the cadence because the
part owns the cadence.

### Part C — The body frame

The platform abstraction declares the axis convention binding but never states it. It is fixed
here, and the estimation theory forces the choice. With the accelerometer model
`[a_x; a_z] = [-g sinθ; -g cosθ]` and `θ = atan2(-a_x, -a_z)`, upright must read `-g` on z.

| Axis | Direction |
|------|-----------|
| x    | Forward   |
| y    | Left      |
| z    | Up        |

Right-handed. The accelerometer reports the gravity vector, so at rest and upright the z axis
reads about -9.81 m/s². Positive pitch is nose-up, a rotation about y.

The mapping from the part's own axes to these is a signed permutation. The board implementation
holds it, with identity as the default. **The correct permutation depends on how the part is
physically mounted. Nothing in this repository can establish that.** It is a value to set when the
sensor is fitted and to verify by tilting the robot. The wheel geometry is a fitted value in the
same way, not a measured one.

The board keeps an identity map. This component applies a second signed permutation that the
operator sets at run time ([calibration](calibration.md)). The component rejects a map that is
not a permutation of the three axes with unit signs. A new map clears the gyroscope bias and the last sample, so the
component reports itself uncalibrated until the next calibration. A change during a calibration
fails that attempt, so the safety supervisor sees the outcome.

### Part D — Units are converted at the board boundary

The driver reports milli-degrees per second and milli-metres per second squared as integers.
Every specification in this project is SI: radians per second and metres per second squared. The
estimator that consumes this is SI too. The conversion happens in the board implementation,
beside the axis mapping. Both are properties of the part, not of the robot.

### Part E — Calibration belongs here, not to the estimator

The gyroscope's bias is a slowly varying offset that integration turns into unbounded drift. The
theory puts an uncalibrated bias of 0.02 rad/s at 34 degrees of error after thirty seconds. It
must be removed, and it is removed here.

This was a contradiction in the specifications. The requirements assign bias estimation and
subtraction to the sensing component. The attitude estimation design listed it among the
estimator's responsibilities. The requirements win, and the estimation design document is
corrected to match. There is a reason beyond precedence. The architecture already describes this
component as producing *calibrated* measurements. An estimator that receives a corrected rate is
a pure fusion filter, without a calibration mode of its own.

Calibration averages the angular rate over a window while it checks two things:

- Every sample stays within a stillness threshold. A sample beyond it abandons the attempt. An
  average taken while the robot moves is not a bias but motion. Subtracting it would bias the
  estimate permanently.
- No gap between consecutive samples exceeds the bound that decides staleness. This check
  matters because the window closes on elapsed time, not on a count of samples. Without it, one
  sample, a stall, and a second sample a second later would pass as a bias from two readings.
  The whole point of a window is that averaging several hundred samples shrinks the noise.

A failed attempt leaves the previous bias untouched.

The operator can set the stillness threshold at run time, from 0.01 to 1.0 rad/s
([calibration](calibration.md)). A new threshold clears the bias, as a new axis map does.

### Part F — Validity has a cause

A consumer that learns only that a measurement is unusable cannot tell a sensor that has never
been read from one that has stopped answering. Each measurement therefore carries a validity flag
and the reason. The reasons are: never sampled, stale, transfer failed, and not yet calibrated.

Staleness is evaluated when the measurement is read, not when a sample arrives. This matters. If
the sensor stops entirely, no callback arrives to notice. A validity computed only on arrival
would then stay true forever, at exactly the moment it stopped being true.

---

## Interfaces

### Provided

| Interface            | Purpose                                          | Contract                                                                                                 |
|----------------------|--------------------------------------------------|----------------------------------------------------------------------------------------------------------|
| Inertial measurement | Bias-corrected angular rate and acceleration     | Body frame, SI units, carries the sample instant; invalid unless fresh, calibrated and read successfully |
| Validity and cause   | Whether the measurement may be believed, and why | Invalid is never merely degraded; the cause distinguishes stale from failed from uncalibrated            |
| Calibration control  | Begin bias calibration and report its outcome    | Failure leaves the previous bias unchanged; a new attempt may be started at any time                     |
| Frame and stillness  | Set the axis map and the stillness threshold     | Invalid values rejected; a change clears the bias and fails an attempt in progress                       |

### Required

| Interface       | Purpose                                      | Contract                                                                          |
|-----------------|----------------------------------------------|-----------------------------------------------------------------------------------|
| Inertial sensor | Six-axis samples in the body frame           | Pushed as they are produced, timestamped at delivery, flagged on transfer failure |
| Timebase        | Judging staleness and the calibration window | Monotonic; the measured instant is used, never a nominal one                      |

Boundaries with the neighbouring components:

- The driver and the board implementation talk to the part. This component consumes a portable
  role.
- Attitude estimation fuses the measurements into pitch. It consumes this component.
- The safety supervisor decides what an invalid measurement means for the drive.
- The sensor's own data-ready output sets the sampling cadence.

---

## Data Model

| Entity        | Field               | Type / Unit               | Range                                                     | Notes                                                                          |
|---------------|---------------------|---------------------------|-----------------------------------------------------------|--------------------------------------------------------------------------------|
| Measurement   | angular rate        | radians per second        | -8.7 to 8.7                                               | Body frame, bias-corrected                                                     |
| Measurement   | acceleration        | metres per second squared | -39 to 39                                                 | Body frame, uncorrected                                                        |
| Measurement   | sample instant      | time point                | monotonic                                                 | Taken when the driver delivers the sample, one burst after the data-ready edge |
| Measurement   | valid               | boolean                   | true or false                                             | False means unusable, not degraded                                             |
| Measurement   | cause               | enumeration               | none, never sampled, stale, transfer failed, uncalibrated | Why an invalid measurement is invalid                                          |
| Calibration   | gyroscope bias      | radians per second        | -0.17 to 0.17                                             | One per axis, re-estimated each power-on                                       |
| Calibration   | window              | milliseconds              | 500 to 2000                                               | Averaging window                                                               |
| Calibration   | stillness threshold | radians per second        | 0.01 to 1.0                                               | Exceeding it during the window fails the attempt                               |
| Configuration | sample period       | microseconds              | greater than zero                                         | Expected cadence; only staleness is judged by it                               |
| Configuration | stale periods       | count                     | greater than zero                                         | Periods without a sample before invalidity                                     |

---

## State Machine

Calibration is the only state this component carries beyond the last sample.

```mermaid
stateDiagram-v2
    [*] --> Uncalibrated
    Uncalibrated --> Calibrating : start calibration
    Calibrating --> Calibrated : window elapsed, every sample still
    Calibrating --> Failed : motion beyond the threshold, or a failed transfer
    Failed --> Calibrating : start calibration
    Calibrated --> Calibrating : start calibration
    Calibrated --> Uncalibrated : axis map or threshold changed
    Failed --> Uncalibrated : axis map or threshold changed
    Calibrating --> Failed : axis map or threshold changed
```

Measurements are invalid in every state but Calibrated.

---

## Sequence Diagrams

Steady-state sampling:

```mermaid
sequenceDiagram
    participant Part as Inertial sensor
    participant Board as Board implementation
    participant Sensing as Inertial sensing
    participant Consumer

    Part->>Board: data ready asserted
    Board->>Part: burst read, accelerometer then gyroscope
    Part-->>Board: six axes
    Board->>Board: timestamp, map axes, convert to SI
    Board->>Sensing: sample
    Sensing->>Sensing: subtract gyroscope bias

    Consumer->>Sensing: read measurement
    Sensing-->>Consumer: rate, acceleration, validity and cause
```

Calibration:

```mermaid
sequenceDiagram
    participant Operator
    participant Sensing as Inertial sensing
    participant Board as Board implementation

    Operator->>Sensing: start calibration
    loop until the window elapses
        Board->>Sensing: sample
        alt still
            Sensing->>Sensing: accumulate
        else moving
            Sensing->>Sensing: abandon, previous bias kept
        end
    end
    Sensing->>Sensing: bias is the mean of the accumulated rates
```

---

## Block Diagram

```mermaid
graph LR
    PART[Inertial sensor] -->|data ready| INT[Interrupt]
    INT --> READ[Burst read]
    PART -->|six axes| READ
    READ --> STAMP[Timestamp at delivery]
    STAMP --> MAP[Axis map and SI conversion]
    MAP --> BIAS[Subtract gyroscope bias]
    MAP --> CAL[Calibration accumulator]
    CAL --> BIAS
    BIAS --> OUT[Measurement]
    FRESH[Staleness and transfer state] --> VALID[Validity and cause]
    VALID --> OUT
```

---

## Constraints & Limitations

| Constraint                                  | Value / Description                                                                                                                                                                                                                                                                                                                                                        |
|---------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Sample rate                                 | 1 kHz, set by the part's own divider. The requirement asks for at least 500 Hz; a nominal 500 would sit below it whenever the internal oscillator runs slow, so the rate is doubled rather than left on the boundary                                                                                                                                                       |
| Bus rate                                    | The part permits a high bus rate only for sensor and interrupt data and a low rate elsewhere. The adapter does not switch rates per transaction, so the whole bus runs at the low rate                                                                                                                                                                                     |
| Transfer cost                               | About 120 µs per sample at the low bus rate for fifteen bytes, roughly 12% of one core at 1 kHz. Measured on paper, not on hardware                                                                                                                                                                                                                                        |
| Bias is estimated once                      | Per power-on, and not tracked against temperature. A long run that warms up will drift                                                                                                                                                                                                                                                                                     |
| Stillness is judged by rate                 | Only angular rate is checked; a robot translating smoothly at constant velocity would pass                                                                                                                                                                                                                                                                                 |
| Magnetometer unused                         | The magnetometer in the package is not read, so there is no absolute heading reference                                                                                                                                                                                                                                                                                     |
| Transfer failure is invisible on the target | The bus interface this driver sits on reports completion with no status, so a failed transfer cannot be told from a slow one. In practice a failure means no sample arrives and the output goes stale. REQ-IMU-006 is therefore only partly met on the target:     no measurement is propagated, but the condition surfaces as staleness rather than as a transfer failure |
| Timestamp offset                            | The sample instant is the delivery instant, about 120 µs after the data-ready edge — see Part B                                                                                                                                                                                                                                                                            |
| Single interrupt line                       | Interrupt lines are keyed by pin number across all ports on the target, so the data-ready pin's number must stay unique                                                                                                                                                                                                                                                    |
| No heap, no recursion                       | Fixed-size state, no dynamic allocation on any path                                                                                                                                                                                                                                                                                                                        |

---

## Decisions

| # | Decision                                                                                    | Options                                                                                                      | Status                                                                                                                       |
|---|---------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------|
| 1 | What is the sensor-to-body axis permutation for the fitted part?                            | Determine by tilting the assembled robot; read it off the mechanical drawing                                 | open — identity until hardware exists                                                                                        |
| 2 | Is the stillness threshold right, and should it also bound acceleration?                    | Fit from a bench recording of the robot at rest; add an acceleration bound                                   | open — placeholder value                                                                                                     |
| 3 | Should a failed identification read raise a fault rather than simply never sampling?        | Raise it through the safety supervisor once that component exists                                            | decided — no samples means the supervisor self-test fails after 1 second ([safety supervisor](safety-supervisor.md), Part F) |
| 4 | Does 12% of a core for transfers survive contact with the control loop?                     | Measure on target; drop to 500 Hz; switch bus clocks per transaction                                         | open — unmeasured                                                                                                            |
| 5 | How should a failed bus transfer become visible, given the bus interface reports no status? | Extend the bus interface with a completion status; time out a transfer in the adapter; leave it as staleness | open — staleness only, today                                                                                                 |
| 6 | Should the timestamp be taken at the data-ready edge inside the driver?                     | Change the vendored driver to stamp in its handler; accept the constant delivery offset                      | open — offset accepted and documented                                                                                        |
