---
title: "LQR Design Tool"
type: design
status: draft
version: 0.1.1
component: "lqr-design-tool"
date: 2026-11-26
---

| Field     | Value           |
|-----------|-----------------|
| Title     | LQR Design Tool |
| Type      | design          |
| Status    | draft           |
| Version   | 0.1.1           |
| Component | lqr-design-tool |
| Date      | 2026-11-26      |

> A host console tool that turns the measured robot parameters into the four state gains of the
> LQR strategy ([balance control](balance-control.md)). It prints them as console commands for the robot. This
> gives roadmap M8 step 1 a first gain set.

---

## Responsibilities

- Building the linear pitch plant from the measured parameters.
- Discretising the plant at the control loop period and solving the regulator problem.
- Converting the gains to the firmware state and effort conventions.
- Checking that the closed loop settles and that each gain fits its parameter range.

---

## Component Details

### Part A — Sources

The plant comes from the wheeled inverted pendulum model in robotics-toolbox-cpp
(its wheeled inverted pendulum reference document). The zero-order-hold discretisation and the
discrete Riccati solution come from numerical-toolbox. robotics-toolbox becomes a submodule of this
repository. Wheel odometry uses it too ([wheel odometry](wheel-odometry.md), Part D).

### Part B — Inputs

| Input         | Unit  | Default        | Notes                                                |
|---------------|-------|----------------|------------------------------------------------------|
| Body mass     | kg    | required       | Everything that pivots about the axle                |
| Wheels mass   | kg    | required       | Both wheels                                          |
| Height        | m     | required       | Axle to the body centre of mass                      |
| Body inertia  | kg·m² | required       | About the body centre of mass                        |
| Wheel inertia | kg·m² | required       | One wheel, with the rotor through the gear ratio     |
| Wheel radius  | m     | 0.034          | As `cal wheel`                                       |
| Wheel torque  | N·m   | required       | Torque at one wheel at full effort                   |
| Loop period   | s     | 0.002          | The 500 Hz control loop                              |
| State limits  | SI    | 0.1, 1, 0.5, 1 | Pitch, pitch rate, position, velocity; Bryson's rule |

Bryson's rule sets each state weight to one over the square of its limit. The input weight is
one over the square of the common torque at full effort, two wheel torques.

### Part C — Conventions

The model takes pitch positive with the top forward and the common wheel torque as its input. The
firmware takes pitch positive nose-up ([attitude estimation](attitude-estimation.md), Part G) and effort per wheel,
where full effort gives the wheel torque. The tool therefore negates the two pitch gains and
divides each gain by two wheel torques. With effort equal to minus the gains times the state, the
pitch gains come out positive and the wheel gains negative, as [balance control](balance-control.md) expects.

### Part D — Output

The tool prints the plant, the gains and the checks. When every check passes, it prints the
commands to type: `strategy lqr` and one `param` command per state gain. Otherwise it prints the
failed check instead and stops with a failure status. The checks: the Riccati solution converges,
the closed loop settles, and each gain fits its parameter range.

The tool rejects these inputs:

- A value that is not a finite number.
- A limit that is not positive.
- A loop period outside 0.1 ms to 100 ms.

The closed-loop check runs the discrete closed loop from a 0.05 rad pitch for 5 seconds. It passes
when the state ends below 1 % of its start. Tight limits on one state can leave another too slow
to settle; the remedy is a looser limit.

---

## Interfaces

### Provided

| Interface    | Purpose                             | Contract                                                       |
|--------------|-------------------------------------|----------------------------------------------------------------|
| Command line | Parameters in, gains and checks out | Exit status 0 only when every check passes and the gains print |

### Required

| Interface              | Purpose                          | Contract                                                        |
|------------------------|----------------------------------|-----------------------------------------------------------------|
| Pendulum model         | Continuous pitch plant           | robotics-toolbox, model pitch positive top forward              |
| Discretisation and LQR | Zero-order hold, discrete gains  | numerical-toolbox; the Riccati iteration reports convergence    |
| LQR strategy           | Parameter names and ranges       | See [balance control](balance-control.md)                       |
| Operator               | Measure and enter the parameters | Types the printed commands; the tool does not talk to the robot |
| Yaw loop               | Yaw gains                        | A PID that the operator tunes on the robot; not designed here   |

---

## Constraints & Limitations

| Constraint   | Value / Description                                                                                  |
|--------------|------------------------------------------------------------------------------------------------------|
| Host only    | The tool is not part of the firmware build                                                           |
| Linear model | Valid near upright only; see the model's limits                                                      |
| Torque       | Effort maps to torque open-loop ([motion actuation](motion-actuation.md)); the wheel torque is a fit |
| No state     | The tool keeps no settings and writes no files, so it needs no log or safe start                     |
