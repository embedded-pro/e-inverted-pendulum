---
title: "Use Cases"
type: use-cases
status: draft
version: 0.3.0
component: "system"
date: 2026-11-26
---

| Field     | Value      |
|-----------|------------|
| Title     | Use Cases  |
| Type      | use-cases  |
| Status    | draft      |
| Version   | 0.3.0      |
| Component | system     |
| Date      | 2026-11-26 |

> **Specification scenarios.** The `.feature` files in this directory are the
> executable-shaped statement of what the robot must do. They are the bridge between
> the requirements in `documentation/requirements/` and the integration suite in
> `integration_tests/`: every requirement is traced to at least one scenario here.

---

## Why these live in `documentation/`

`integration_tests/features/` is *executed* — by `ctest --preset host` and by the SIL
job in CI. A scenario placed there without a matching step definition fails the build.
A scenario stays here, as specification, until it has step definitions.

They are not inert. The release pipeline generates the requirements traceability matrix
from this directory, so coverage is measured on every release.

**Migration path**: when a component gains an implementation, move its `.feature` file
into `integration_tests/features/`, add the step definitions under
`integration_tests/steps/`, and register them in that directory's `CMakeLists.txt`.
A scenario that the host cannot run yet stays here, in a file of the same name.
The release job copies both directories into one before it makes the matrix.

---

## Actors

| Actor       | Description                                                                      |
|-------------|----------------------------------------------------------------------------------|
| Operator    | The person driving the robot, acting through a BLE client application            |
| BLE client  | The host application that connects, pairs, sends commands and renders telemetry  |
| Robot       | The firmware under specification                                                 |
| Environment | Ground surface, disturbances and the physical handling the robot is subjected to |

---

## Use cases

| #    | Use case                           | Actor    | Precondition                  | Main flow                                                                                       | Alternate flows                                                                                    | File                              |
|------|------------------------------------|----------|-------------------------------|-------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|-----------------------------------|
| UC-1 | Bring the robot into service       | Operator | Robot powered off, held still | Power on, self-test, calibrate gyroscope bias, settle in IDLE                                   | Sensor absent, or robot moved during calibration                                                   | `startup-and-calibration.feature` |
| UC-2 | Measure attitude and wheel motion  | Robot    | Calibration complete          | Sample the IMU, fuse into pitch and pitch rate, decode both encoders                            | Sensor stale or transfer failed; counter wrap                                                      | `sensing-and-odometry.feature`    |
| UC-3 | Balance and follow motion commands | Operator | Robot in IDLE, upright        | Arm, hold upright, track velocity and yaw setpoints                                             | Arm refused when tilted; disturbance; actuator saturation                                          | `balancing.feature`               |
| UC-4 | Shut down safely                   | Robot    | Robot ARMED                   | Detect fall, driver fault, invalid estimate or stalled loop; tri-state the bridges; latch FAULT | Recoverable tilt does not trip; clear-fault then explicit re-arm                                   | `safety.feature` (both)           |
| UC-5 | Drive the robot remotely           | Operator | Robot advertising             | Connect, pair, arm, write velocity and yaw setpoints                                            | Second client refused; unpaired write refused; command timeout; link lost                          | `teleoperation.feature`           |
| UC-6 | Observe and tune                   | Operator | Paired client connected       | Subscribe to telemetry; read the active strategy and its parameter descriptor; write parameters | Strategy change or parameter write refused while ARMED; out-of-range write refused; congested link | `telemetry-and-tuning.feature`    |

---

## Traceability

Each scenario carries the requirement IDs it verifies as **Gherkin tags** on the line
immediately above `Scenario:`:

```gherkin
  @REQ-CTRL-002
  Scenario: The robot recovers from a push
```

A scenario verifying several requirements carries one tag per requirement, separated by
spaces:

```gherkin
  @REQ-CTRL-005 @REQ-CTRL-006 @REQ-TUNE-001
  Scenario: The robot reports the strategies it offers
```

Tags rather than comments, because a tag is part of the Gherkin language rather than
inert text. Cucumber can filter on them, so once a feature file moves into
`integration_tests/features/` the scenarios verifying one requirement become a runnable
selection — `--tags @REQ-CTRL-002` — instead of something only the matrix generator reads.

The matrix is generated by the `embedded-pro/project-documentation` action, which
scans one flat directory for `REQ-[A-Z]+-[0-9]+` tags. The "Collect feature files" step in
`.github/workflows/release-please.yml` copies both feature directories into one first.

**Current coverage: 88 of 88 requirements.** A new requirement without a tagged
scenario shows up under "Uncovered Requirements" — treat that as a specification defect.

Two mechanical constraints on this directory, both imposed by the generator:

- Feature files must sit **flat** in each directory; the generator does not recurse into
  subdirectories.
- A requirement ID is matched by exact text, so a typo in a tag silently drops
  coverage rather than failing loudly.
- The generator matches the ID anywhere on the line, so it reads `@REQ-…` tags and bare
  `REQ-…` text alike; the tag form is the convention here.
