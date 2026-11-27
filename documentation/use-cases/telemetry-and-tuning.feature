Feature: Telemetry and runtime tuning over Bluetooth Low Energy
  The operator observes the robot's state and tunes it without rebuilding firmware.
  Because the control law is not fixed by the architecture, tuning is generic: the
  robot describes the parameters of whichever strategy is active, and the client
  tunes it without prior knowledge of that strategy.

  Background:
    Given the robot is calibrated
    And a paired client is connected

  @REQ-TELEM-003
  Scenario: Telemetry flows only to a subscriber
    Given the client has not subscribed to telemetry
    When 1 second elapses
    Then no telemetry notification has been emitted

  @REQ-TELEM-001 @REQ-TELEM-002
  Scenario: Subscribed clients receive the robot state
    Given the robot is ARMED and balancing
    When the client subscribes to telemetry and observes for 1 second
    Then 25 telemetry notifications are received
    And each notification reports the estimated pitch, pitch rate, chassis velocity, yaw rate, per-wheel effort and active mode

  @REQ-TELEM-005
  Scenario: A telemetry update is internally consistent
    Given the robot is ARMED and the client is subscribed
    When a telemetry notification is received
    Then all reported values originate from a single control-loop iteration

  @REQ-TELEM-004
  Scenario: A congested link never stalls the control loop
    Given the robot is ARMED and the client is subscribed
    When the link cannot accept further notifications for 200 milliseconds
    Then the balance loop continues to execute at its specified rate
    And the undeliverable updates are dropped rather than queued without bound

  @REQ-TELEM-003
  Scenario: Unsubscribing stops the stream
    Given the client is subscribed to telemetry
    When the client unsubscribes
    Then no further telemetry notifications are emitted

  @REQ-TELEM-006
  Scenario: Faults are visible in telemetry
    Given the robot has entered FAULT after a fall
    When a telemetry notification is received
    Then it reports the mode as FAULT
    And it reports the latched fault cause

  @REQ-CTRL-005 @REQ-CTRL-006 @REQ-TUNE-001
  Scenario: The robot reports the strategies it offers
    Given the robot is in IDLE
    When the client requests the available control strategies
    Then at least two interchangeable strategies are reported
    And the identifier of the active strategy is reported

  @REQ-TUNE-002
  Scenario: The operator selects a different control strategy
    Given the robot is in IDLE
    And the active strategy is the first available strategy
    When the client selects the second available strategy
    Then the active strategy becomes the second available strategy

  @REQ-CTRL-007
  Scenario: The control strategy cannot be swapped in flight
    Given the robot is ARMED and balancing
    When the client selects a different control strategy
    Then the request is rejected
    And the active strategy is unchanged
    And the robot continues to balance undisturbed

  @REQ-TUNE-003
  Scenario: The active strategy describes its own parameters
    Given the robot is in IDLE
    When the client requests the parameter descriptor of the active strategy
    Then the number, order and identity of the tunable parameters are reported
    And a permitted range is reported for each parameter

  @REQ-TUNE-003
  Scenario: Selecting another strategy changes the parameter descriptor
    Given the robot is in IDLE
    When the client selects a different control strategy
    And the client requests the parameter descriptor of the active strategy
    Then the descriptor matches the newly selected strategy

  @REQ-TUNE-004
  Scenario: Parameters are readable
    Given the robot is in IDLE
    When the client reads the parameters of the active strategy
    Then the current value of every tunable parameter is reported

  @REQ-TUNE-005
  Scenario: Parameters are writable while not armed
    Given the robot is in IDLE
    When the client writes a new in-range value to a tunable parameter
    Then the write is accepted
    And reading the parameter back returns the new value

  @REQ-TUNE-005
  Scenario: Parameters cannot be written while armed
    Given the robot is ARMED and balancing
    When the client writes a new value to a tunable parameter
    Then the write is rejected
    And the parameter value is unchanged

  @REQ-TUNE-006
  Scenario: Out-of-range parameter writes are refused
    Given the robot is in IDLE
    When the client writes a value outside the published range of a parameter
    Then the write is rejected
    And the stored parameter value is unchanged

  @REQ-LINK-011
  Scenario: The client reads and changes the calibration
    Given the robot is in IDLE
    When the client sets the left motor sign to -1
    Then the change is accepted
    And reading every calibration value reports the left motor sign as -1

  @REQ-LINK-011
  Scenario: Calibration changes are refused while ARMED
    Given the robot is ARMED and balancing
    When the client sets the wheel geometry
    Then the change is refused because the drive is armed
    And the wheel geometry is unchanged
