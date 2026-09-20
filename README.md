# inverted-pendulum-bot

Firmware for a self-balancing inverted pendulum robot, written in C++20 with strict
real-time and memory constraints. The repository was bootstrapped from
[`embedded-pro/embedded-scaffold`](https://github.com/embedded-pro/embedded-scaffold)
and carries its structure, build system, dev container, CI, and documentation
scaffolding, so the project starts from a working, testable baseline.

> **Status**: the structure is in place; the `example_*` components and `cli`
> are the scaffold's worked examples and are expected to be replaced by the robot's
> own components (sensing, control, actuation).

## Overview

The project separates **portable logic** from **hardware** through one key seam: an
abstract `platform::Platform` interface in `core/platform_abstraction/`. Application
logic in `core/` depends only on that interface; each board provides a concrete
`PlatformImpl` in `targets/platform_implementations/`. The same application is
therefore compiled once and runs on the host (for testing) and on the
microcontroller.

The current headline example is **`cli`**: blink a status LED and serve a tiny
UART command-line interface (`ping`, `id`). It is written once against the interface
and builds for the host and the **ST NUCLEO-WB55RG**.

## Features

- **Platform abstraction**: application logic depends on the `platform::Platform`
  interface, not on an MCU — so it is built once and unit-tested on the host against
  a `PlatformMock`, then run on real hardware.
- **Real firmware output**: `cli` builds to a flashable `.elf`/`.hex` for the
  NUCLEO-WB55RG (LED blink + UART CLI).
- **No heap allocation** in runtime/embedded code — bounded containers from
  `infra/embedded-infra-lib` (`infra::BoundedVector`, `infra::BoundedString`).
- **Worked examples** in every top-level folder showing the conventions to follow.
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
   git clone --recursive https://github.com/embedded-pro/e-Inverted-pendulum-bot.git
   cd e-Inverted-pendulum-bot
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

4. Run the example tool / app on the host:
   ```bash
   ./build/host/bin/Debug/inverted_pendulum_bot.tool.example 2 3 4   # -> total = 9
   ./build/host/bin/Debug/inverted_pendulum_bot.example_app          # -> accumulator total = 5
   ```

5. Build the `cli` firmware for the board (produces `.elf`/`.hex`):
   ```bash
   cmake --preset NUCLEO-WB55RG          # ST Nucleo-68
   cmake --build --preset NUCLEO-WB55RG-Debug
   ```
   On hardware the status LED (LD2, green, PB0) blinks and a UART command-line
   interface (115200 8N1) accepts `ping` and `id`. The CLI is on USART1
   (PB6 TX / PB7 RX), which the NUCLEO-WB55RG routes to the on-board ST-LINK virtual
   COM port — no USB-UART adapter needed.

All presets are defined in `CMakePresets.json` (`host`, `host-single-Debug`,
`windows`, `coverage`, `NUCLEO-WB55RG`).

## Project Structure

```
e-Inverted-pendulum-bot/
├── core/                      # Reusable libraries only — no entry points
│   ├── platform_abstraction/  #   platform::Platform interface (+ mock) — the seam
│   ├── cli/            #   portable app: LED blink + UART CLI (+ unit test)
│   └── example_component/     #   trivial interfaces/ + implementations/ (+ unit test)
├── targets/                   # Application entry points + platform implementations
│   ├── cli/            #   one Main.cpp reused across host/st
│   ├── example_app/           #   a trivial host-only entry point
│   └── platform_implementations/
│       ├── host/              #   PlatformImpl: stubs + loopback serial (host build)
│       └── st/                #   PlatformImpl: NUCLEO-WB55RG (LED PB0, USART1)
├── tools/                     # Host-side developer tools
│   └── example_tool/          #   a CLI reusing a core library
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
| [Safety Supervisor](documentation/design/safety-supervisor.md)                     | Arm/disarm/fault state machine and the disarm conditions                          |
| [Balance Control](documentation/design/balance-control.md)                         | The selectable control-strategy interface and setpoint handling                   |
| [Attitude Estimation](documentation/design/attitude-estimation.md)                 | Pitch estimation, bias calibration and estimate validity                          |
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
| [AI Agent Instructions](CLAUDE.md)                                                 | Development guidelines, patterns, and constraints                                 |

## License

This project is licensed under the terms in the [LICENSE](LICENSE) file.

Third-party components and submodules keep their own license terms as documented in
their respective directories.
