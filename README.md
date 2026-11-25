# e-inverted-pendulum

Firmware for a self-balancing inverted pendulum robot on the ST NUCLEO-WB55RG, written in
C++20 with strict real-time and memory constraints, with a host build for tests. The
repository started from [`embedded-pro/embedded-scaffold`](https://github.com/embedded-pro/embedded-scaffold).

> **Status**: the robot's components (inertial sensing, attitude estimation, balance
> control, motion actuation, wheel odometry, control loop, safety supervisor, telemetry,
> BLE link) are in `core/`. `integration_tests/` runs the safety scenarios on the host.

## Overview

The project separates **portable logic** from **hardware** through one key seam: an
abstract `platform::Platform` interface in `core/platform_abstraction/`. Application
logic in `core/` depends only on that interface; each board provides a concrete
`PlatformImpl` in `targets/platform_implementations/`. The same application is
therefore compiled once and runs on the host (for testing) and on the
microcontroller.

The firmware entry point is **`targets/cli`**: it wires the components to the
platform and serves a UART command-line interface to inspect and drive the robot
(sensor readout, calibration, filter and control-strategy selection, arming, motion
commands, loop timing, BLE state; `help` lists them). It builds for the host and the
**ST NUCLEO-WB55RG**.

## Features

- **Platform abstraction**: application logic depends on the `platform::Platform`
  interface, not on an MCU — so it is built once and unit-tested on the host against
  a `PlatformMock`, then run on real hardware.
- **Real firmware output**: `cli` builds to a flashable `.elf`/`.hex` for the
  NUCLEO-WB55RG.
- **Selectable estimation and control**: complementary or Kalman pitch filter; cascaded
  PID or LQR balance control, tunable over the CLI or BLE.
- **Safety supervisor**: self-test, calibration, arming and disarm conditions.
- **No heap allocation** in runtime/embedded code — bounded containers from
  `infra/embedded-infra-lib` (`infra::BoundedVector`, `infra::BoundedString`).
- **Unit tests** (GoogleTest) and **BDD integration tests** (cucumber-cpp / Gherkin).
- **Dev container** with the full toolchain (CMake, Ninja, ccache, ARM GCC, Qt6).
- **CI** for build, linting/formatting, static analysis, documentation and
  requirements validation, and release-please versioning.

## Getting Started

### Prerequisites

- Docker (the project is developed inside a Dev Container) **or** a local toolchain
  with CMake ≥ 3.24, Ninja, a C++20 compiler, and (for embedded) ARM GCC.
- VS Code with the Dev Containers extension is the recommended workflow.

### Quick Start

1. Clone with submodules:
   ```bash
   git clone --recursive https://github.com/embedded-pro/e-inverted-pendulum.git
   cd e-inverted-pendulum
   ```

2. Configure & build for the host:
   ```bash
   cmake --preset host
   cmake --build --preset host-Debug
   ```

3. Run unit + integration tests:
   ```bash
   ctest --preset host
   ```

4. Build the `cli` firmware for the board (produces `.elf`/`.hex`):
   ```bash
   cmake --preset NUCLEO-WB55RG          # ST Nucleo-68
   cmake --build --preset NUCLEO-WB55RG-Debug
   ```
   On hardware the status LED (LD2, green, PB0) blinks and the UART command-line
   interface runs at 115200 8N1. The CLI is on USART1
   (PB6 TX / PB7 RX), which the NUCLEO-WB55RG routes to the on-board ST-LINK virtual
   COM port — no USB-UART adapter needed.

All presets are defined in `CMakePresets.json` (`host`, `host-single-Debug`,
`windows`, `coverage`, `NUCLEO-WB55RG`).

## Project Structure

```
e-inverted-pendulum/
├── core/                      # Reusable libraries only — no entry points
│   ├── platform_abstraction/  #   platform::Platform interface (+ mocks) — the seam
│   ├── inertial_sensing/      #   IMU samples, gyroscope bias calibration
│   ├── attitude_estimation/   #   pitch: complementary or Kalman filter
│   ├── balance_control/       #   cascaded PID or LQR, setpoints
│   ├── motion_actuation/      #   DRV8711 motor driver, effort mapping
│   ├── wheel_odometry/        #   wheel velocity and chassis motion
│   ├── control_loop/          #   pacing of estimation, balance and outer loops
│   ├── safety_supervisor/     #   self-test, calibration, arming, disarm
│   ├── persistence/           #   tuning record in EEPROM
│   ├── telemetry/             #   telemetry recorder
│   ├── ble_link/              #   GATT service: teleoperation, telemetry, tuning
│   └── cli/                   #   UART command-line interface
├── targets/                   # Application entry points + platform implementations
│   ├── cli/                   #   the firmware: one Main.cpp for host and st
│   └── platform_implementations/
│       ├── host/              #   PlatformImpl: host stubs
│       └── st/                #   PlatformImpl: NUCLEO-WB55RG
├── tools/                     # Host-side developer tools
├── integration_tests/         # BDD integration tests (cucumber-cpp / Gherkin)
├── infra/                     # Infrastructure submodules
│   ├── embedded-infra-lib/    #   bounded containers, build helpers, toolchains
│   ├── numerical-toolbox/     #   PID, filters, fixed-point algorithms
│   └── hal/st/                #   STM32 hardware abstraction layer
├── documentation/             # Architecture/design/theory/requirements (+ templates)
├── scripts/                   # Build and utility scripts
└── build/                     # Build artifacts (generated, not committed)
```

Each top-level folder has its own `README.md` describing its conventions and how to
add new components.

## Key Design Principles

- **No heap in runtime code**: all memory statically allocated; bounded containers
  instead of STL containers. Host tools and tests may use the heap.
- **Interface-driven design + dependency injection**: hardware is injected via the
  `platform::Platform` interface through constructors, never global state.
- **Documentation-first**: update the relevant `documentation/` doc before or
  alongside behavioural changes. Diagrams use Mermaid or ASCII art only.
- **SOLID / DRY**: reuse `infra/numerical-toolbox/` algorithms; do not duplicate.

## Documentation

| Document                                                                           | Description                                                                       |
|------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------|
| [System Architecture](documentation/architecture/system.md)                        | Components, interfaces and cross-cutting concerns of the robot                    |
| [Use Cases](documentation/use-cases/README.md)                                     | Actors, flows and the Gherkin scenarios every requirement traces to               |
| [Requirements](documentation/requirements/)                                        | Requirement sets, validated against `documentation/tools/requirement.schema.json` |
| [Safety Supervisor](documentation/design/safety-supervisor.md)                     | Mode state machine: self-test, calibration, arming and the disarm conditions      |
| [Balance Control](documentation/design/balance-control.md)                         | The selectable control-strategy interface and setpoint handling                   |
| [Attitude Estimation](documentation/design/attitude-estimation.md)                 | Pitch estimation with a selectable complementary or Kalman filter                 |
| [Control Loop](documentation/design/control-loop.md)                               | Sample-driven pacing of estimation, balance and outer loops, and loop timing      |
| [Motion Actuation](documentation/design/motion-actuation.md)                       | Motor driver, effort mapping and the bridge disable states                        |
| [Wheel Odometry Design](documentation/design/wheel-odometry.md)                    | Wrap-safe count accumulation, wheel velocity and chassis motion                   |
| [Inertial Sensing Design](documentation/design/imu-sensing.md)                     | MPU9250 wiring, body frame, gyroscope bias calibration and sample validity        |
| [BLE Service](documentation/design/ble-service.md)                                 | GATT layout for teleoperation, telemetry and tuning                               |
| [Platform Abstraction](documentation/design/platform-abstraction.md)               | The peripheral roles each board must supply                                       |
| [Pendulum Dynamics](documentation/theory/pendulum-dynamics.md)                     | Equations of motion, linearisation and the fall time constant                     |
| [Control Laws](documentation/theory/control-laws.md)                               | Cascaded PID and LQR derived from the shared plant model                          |
| [Pitch Estimation Theory](documentation/theory/attitude-estimation.md)             | Complementary and Kalman formulations, drift and noise                            |
| [Wheel Odometry Theory](documentation/theory/wheel-odometry.md)                    | Quadrature decoding and differential-drive kinematics                             |
| [Documentation Templates](documentation/templates/)                                | Starting points for new architecture/design/theory/requirements docs              |
| [Performance Optimization Guide](documentation/performance-optimization/README.md) | Embedded performance techniques, assembly analysis, cycle budgets                 |
| [AI Agent Instructions](AGENTS.md)                                                 | Development guidelines, patterns, and constraints                                 |

## License

This project is licensed under the terms in the [LICENSE](LICENSE) file.

Third-party components and submodules keep their own license terms as documented in
their respective directories.
