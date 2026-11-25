Feature: Safety supervision and shutdown
  The safety supervisor owns the decision to energise the motors. These scenarios are
  staged: the host fixture has no model of the robot's motion, and the motor rating is
  not recorded. The scenarios that run are in integration_tests/features/safety.feature.

  Background:
    Given the robot is calibrated
    And the robot is armed and balancing

  @REQ-SAFE-002
  Scenario: A large but recoverable tilt does not trip the fall detector
    When the body is tipped 30 degrees from upright and released
    Then the mode remains ARMED
    And the robot returns upright

  @REQ-DRIVE-005
  Scenario: Motor current is limited
    Given the motor driver has been configured
    When the configured per-motor current limit is read back
    Then it does not exceed the continuous current rating of the fitted motors
