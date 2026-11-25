Feature: Safety supervision and shutdown
  The safety supervisor owns the decision to energise the motors. It disarms on a
  fall, on a driver fault, on loss of a usable attitude estimate, and on a stalled
  control loop. Every fault latches, and recovery is always an explicit operator act.

  Background:
    Given the robot is calibrated
    And the robot is armed and balancing

  @REQ-SAFE-002 @REQ-SAFE-003
  Scenario: A fall disarms the drive
    When the body is tipped 40 degrees from upright
    Then the mode becomes FAULT within 20 milliseconds
    And both motor bridges are tri-stated
    And the bridges are not actively braked

  @REQ-SAFE-006 @REQ-DRIVE-004
  Scenario: A motor driver fault escalates to the supervisor
    When the motor driver asserts its fault output
    Then both motor bridges are disabled
    And the mode becomes FAULT
    And the reported fault cause identifies the motor driver

  @REQ-SAFE-007
  Scenario: Losing the attitude estimate disarms the robot
    When the attitude estimate becomes invalid
    Then the mode becomes FAULT
    And both motor bridges are tri-stated

  @REQ-SAFE-008
  Scenario: A stalled control loop is detected
    When the balance control loop is not serviced for 3 consecutive periods
    Then the mode becomes FAULT
    And both motor bridges are tri-stated

  @REQ-SAFE-004
  Scenario: Faults latch after the cause has cleared
    Given the robot has entered FAULT after a fall
    When the body is returned to upright
    Then the mode remains FAULT
    And both motor bridges remain disabled

  @REQ-SAFE-005
  Scenario: Recovery requires an explicit arm command
    Given the robot is in FAULT
    And the body is upright
    When a clear-fault command is issued
    Then the mode becomes IDLE
    And the motors remain de-energised until an arm command is issued

  @REQ-DRIVE-006
  Scenario: Both tri-state and brake are available, and safety uses the tri-state
    Given the actuation component is available
    When a tri-state disable and a braked disable are each requested
    Then both disable states are supported
    And every safety-initiated disable uses the tri-state
