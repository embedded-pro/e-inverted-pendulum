# Changelog

## 1.0.0 (2026-11-27)


### ⚠ BREAKING CHANGES

* **motion:** platform::MotorDriver replaces MotorBridge with Drive() and ConfigurationChannel(); MotionActuationImpl takes a DriverConfiguration.

### Features

* Add comprehensive documentation for balancing robot system ([7a27fae](https://github.com/embedded-pro/e-inverted-pendulum/commit/7a27fae2b47524fa7461c5c9e7e19a22977776f7))
* Attitude estimation, sample-driven control loop and safety supervisor ([#16](https://github.com/embedded-pro/e-inverted-pendulum/issues/16)) ([1f4a0b2](https://github.com/embedded-pro/e-inverted-pendulum/commit/1f4a0b271bb9bf019d24fb34894195e98cfbbdc9))
* **balance:** Balance control with a cascaded PID strategy ([#18](https://github.com/embedded-pro/e-inverted-pendulum/issues/18)) ([81f5236](https://github.com/embedded-pro/e-inverted-pendulum/commit/81f5236e572c79b108a9b9ada8f5c3bb112a1e5f))
* **balance:** LQR balance strategy ([#19](https://github.com/embedded-pro/e-inverted-pendulum/issues/19)) ([9de6d34](https://github.com/embedded-pro/e-inverted-pendulum/commit/9de6d34915c30532ef1e0d87d49509df7dd734b8))
* **balance:** Read and write any strategy's parameters by index ([#57](https://github.com/embedded-pro/e-inverted-pendulum/issues/57)) ([bc3244f](https://github.com/embedded-pro/e-inverted-pendulum/commit/bc3244fd35c7a4d46d08dac8e3efa9e2f0496b81))
* **ble:** Bring up the STM32WB55 radio ([#21](https://github.com/embedded-pro/e-inverted-pendulum/issues/21)) ([5b88987](https://github.com/embedded-pro/e-inverted-pendulum/commit/5b88987bd8cbc1002cf62545389ef0a906dd4e63))
* **ble:** Read and change the calibration over Bluetooth ([#101](https://github.com/embedded-pro/e-inverted-pendulum/issues/101)) ([3bc2de4](https://github.com/embedded-pro/e-inverted-pendulum/commit/3bc2de40fa5f7e0558166cb74f66b90d01066230))
* **ble:** Robot control service, telemetry recorder and setpoint decay ([#20](https://github.com/embedded-pro/e-inverted-pendulum/issues/20)) ([dfef43f](https://github.com/embedded-pro/e-inverted-pendulum/commit/dfef43fd3eb6e04262306c879165ab669a895620))
* Bootstrap project structure from embedded-scaffold ([d3816b1](https://github.com/embedded-pro/e-inverted-pendulum/commit/d3816b1549721309f99a07e05d7185d35e630f34))
* **calibration:** Keep the calibration values in EEPROM ([#70](https://github.com/embedded-pro/e-inverted-pendulum/issues/70)) ([88f60dc](https://github.com/embedded-pro/e-inverted-pendulum/commit/88f60dcff45dd0b027fc11036f3bdc717d772e32))
* **calibration:** Set motor and encoder signs and the wheel geometry at run time ([#67](https://github.com/embedded-pro/e-inverted-pendulum/issues/67)) ([c349fd7](https://github.com/embedded-pro/e-inverted-pendulum/commit/c349fd7d596f26b5ee90dcfccf64c21bf3643bec))
* **calibration:** Set the IMU axis map, stillness limit and upright offset at run time ([#65](https://github.com/embedded-pro/e-inverted-pendulum/issues/65)) ([d626c21](https://github.com/embedded-pro/e-inverted-pendulum/commit/d626c213dea53a74a0d4bc813c0559367b3b2bc7))
* **cli:** Add the cal command ([#71](https://github.com/embedded-pro/e-inverted-pendulum/issues/71)) ([54fde1d](https://github.com/embedded-pro/e-inverted-pendulum/commit/54fde1d01402d1a12165da4f46e1a5b64fff7536))
* **motion:** Drive both motors from the CLI ([88840a0](https://github.com/embedded-pro/e-inverted-pendulum/commit/88840a057e26f6f94ac3f210a469303768e401a5))
* **motion:** Drive both motors from the CLI ([79fb38e](https://github.com/embedded-pro/e-inverted-pendulum/commit/79fb38ec2bbab7df7ac68bf270e6c32cea8adba0))
* **motion:** Drive both motors from TIM1 through one DRV8711, bump submodules ([#15](https://github.com/embedded-pro/e-inverted-pendulum/issues/15)) ([1c0b0ad](https://github.com/embedded-pro/e-inverted-pendulum/commit/1c0b0adbf5ae38cd39b160259c226c6c28a8fb85))
* **odometry:** Accumulate wheel position and derive chassis motion ([#11](https://github.com/embedded-pro/e-inverted-pendulum/issues/11)) ([7a0939b](https://github.com/embedded-pro/e-inverted-pendulum/commit/7a0939bb10d3d390b41719539a93efbeb3b8008f))
* **persistence:** Keep tuning in an external EEPROM across a reset ([#60](https://github.com/embedded-pro/e-inverted-pendulum/issues/60)) ([a779228](https://github.com/embedded-pro/e-inverted-pendulum/commit/a77922895c65de5d18634d1994960a74fa4ec4f8))
* **platform:** Add an EEPROM role ([#58](https://github.com/embedded-pro/e-inverted-pendulum/issues/58)) ([9be24e7](https://github.com/embedded-pro/e-inverted-pendulum/commit/9be24e7bf2727313f8db19a815ee9e7660164b4a))
* **platform:** Decode both wheel encoders in hardware ([#10](https://github.com/embedded-pro/e-inverted-pendulum/issues/10)) ([54df174](https://github.com/embedded-pro/e-inverted-pendulum/commit/54df174d0d5318cfa6eb55ac00524e296878a4fc))
* **sensing:** Acquire bias-corrected inertial samples from an MPU9250 ([#12](https://github.com/embedded-pro/e-inverted-pendulum/issues/12)) ([44b32cb](https://github.com/embedded-pro/e-inverted-pendulum/commit/44b32cb0c1265d5fbee87d4e1ee30cab2e1d0e62))
* **st:** Reset the part when the event loop stalls ([#55](https://github.com/embedded-pro/e-inverted-pendulum/issues/55)) ([572ff42](https://github.com/embedded-pro/e-inverted-pendulum/commit/572ff42a7b6c036afe4ba4037056ff988270f489))
* **tools:** Add lqr_design with robotics-toolbox ([#72](https://github.com/embedded-pro/e-inverted-pendulum/issues/72)) ([2d78eb7](https://github.com/embedded-pro/e-inverted-pendulum/commit/2d78eb7142619a366c2e2d87df1b55e5e15cb6da))


### Bug Fixes

* **cli:** Parse decimals without strtof to keep newlib's heap out of the firmware ([#17](https://github.com/embedded-pro/e-inverted-pendulum/issues/17)) ([4e2cfc4](https://github.com/embedded-pro/e-inverted-pendulum/commit/4e2cfc4e41c1bf83eb2a5571b8c8e3ce1f63db95))
* **cli:** Reject malformed drive arguments ([f9fa429](https://github.com/embedded-pro/e-inverted-pendulum/commit/f9fa4295cae586b5c62d144123adacc60c0646be))
* **motion:** Make the MotionActuationImpl constructor explicit ([dd6d127](https://github.com/embedded-pro/e-inverted-pendulum/commit/dd6d1276d39b956c68895d773052c3c8dd9d4b23))
* **motion:** One DRV8711 drives two motors from four PWM lines ([91bede1](https://github.com/embedded-pro/e-inverted-pendulum/commit/91bede1ec60131ba61b1be89244915a65772dafc))
