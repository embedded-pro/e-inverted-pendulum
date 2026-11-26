# integration_tests

BDD integration tests using [cucumber-cpp](https://github.com/philips-software/amp-cucumber-cpp-runner)
(Gherkin features + C++ step definitions). Built only when `INVERTED_PENDULUM_BOT_BUILD_TESTS`
is on (the `host` preset enables it) and run via CTest.

```
integration_tests/
├── features/        # Gherkin .feature files that run
│   └── safety.feature
├── support/         # The host system under test and its fakes
├── hooks/           # Before/After scenario hooks
├── steps/           # C++ step definitions binding Gherkin to the fixture
└── main/            # Test executable (cucumber_cpp.runner provides main)
```

Run them:

```bash
cmake --preset host
cmake --build --preset host-Debug
ctest --preset host -R inverted_pendulum_bot.integration_tests
```

## The host system

`support/SafetyFixture` builds the same components as `targets/cli/Main.cpp`. It replaces the
hardware with fakes and the clock with `infra::ClockFixture`, so a step moves time forward on demand.

```text
InertialSensorFake ─► InertialSensingImpl ─► ControlLoopImpl ─► SafetySupervisorImpl ─► BalanceControlImpl
                                              AttitudeEstimationImpl                          │
WheelEncodersStub ─► WheelOdometryImpl ──────────────────────────────────────────────────────┤
MotorDriverFake ◄── MotionActuationImpl ◄────────────────────────────────────────────────────┘
       └─ Drv8711Emulator (register read-back)
```

| Fake                  | Steps can                                                                                |
|-----------------------|------------------------------------------------------------------------------------------|
| `InertialSensorFake`  | Tilt the body at a set rate; stop the samples; mark samples as failed transfers          |
| `MotorDriverFake`     | Read the last bridge inputs; see energised bridges; raise a fault                        |
| `WheelEncodersStub`   | Nothing: the wheels stay still (host stub from `targets/platform_implementations/host/`) |
| `NoStoredCalibration` | Nothing: the start-up condition holds at once, as with no saved calibration              |

The IMU fake sends a sample every 1 ms. It reports the gravity vector and pitch rate of its
body pitch in the body frame of `documentation/design/imu-sensing.md`.

The fixture has no model of the robot's motion. A scenario that needs the robot to move by
itself stays staged in `documentation/use-cases/`.

## How it fits together

- A **fixture** in `support/` holds the system under test and is stored in the
  cucumber `context` (`context.Emplace<T>()` / `context.Get<T>()`).
- **Steps** in `steps/` use `GIVEN` / `WHEN` / `THEN` / `STEP` macros whose
  cucumber expression matches lines in the `.feature` files.
- The **feature** files are the human-readable spec. Each scenario carries the tags of the
  requirements it verifies; see `documentation/use-cases/README.md`.

## Add a scenario

1. Move the scenario from `documentation/use-cases/` to the feature file of the same name here.
2. Implement any new `Given/When/Then` lines as steps in `steps/`.
3. Extend the fixture in `support/` if the steps need new hooks into the system.
