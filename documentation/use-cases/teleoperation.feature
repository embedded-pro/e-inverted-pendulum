Feature: Remote teleoperation over Bluetooth Low Energy
  The robot is a BLE peripheral. An operator connects, pairs, and drives it by
  writing velocity and yaw-rate setpoints. The link is assumed unreliable: losing it
  must never drop the robot, and must never leave it running away on a stale command.

  @REQ-LINK-001
  Scenario: The robot advertises while unconnected
    Given the robot is in IDLE with no client connected
    When the advertising state is observed
    Then the robot is advertising a connectable service

  @REQ-LINK-002
  Scenario: The control service is exposed
    Given a client has connected to the robot
    When the client discovers the GATT database
    Then the robot control service is present
    And it exposes teleoperation, telemetry, tuning and mode-control characteristics

  @REQ-LINK-003
  Scenario: Only one operator at a time
    Given a client is already connected
    When a second client attempts to connect
    Then the second connection is refused

  @REQ-LINK-003
  Scenario: Advertising resumes after disconnection
    Given a client is connected
    When the client disconnects
    Then the robot resumes advertising

  @REQ-LINK-008
  Scenario: Unpaired clients cannot command motion
    Given a client has connected but has not paired
    When the client writes a motion command
    Then the write is rejected
    And the commanded setpoints are unchanged

  @REQ-LINK-007
  Scenario: The robot is armed over the link
    Given a paired client is connected
    And the robot is upright and in IDLE
    When the client issues an arm command
    Then the mode becomes ARMED

  @REQ-LINK-004
  Scenario: Motion setpoints are accepted
    Given a paired client is connected and the robot is ARMED
    When the client writes a forward velocity of 0.2 metres per second and a yaw rate of 30 degrees per second
    Then the commanded setpoints are applied

  @REQ-LINK-004
  Scenario: Out-of-range setpoints are refused rather than clamped
    Given a paired client is connected and the robot is ARMED
    When the client writes a forward velocity beyond the documented range
    Then the write is rejected
    And the previously commanded setpoints are unchanged

  @REQ-LINK-005
  Scenario: A silent operator stops the robot
    Given the robot is ARMED and driving forwards on operator command
    When no motion command is received for 500 milliseconds
    Then the commanded velocity and yaw-rate setpoints decay to zero
    And the robot continues to balance

  @REQ-LINK-006
  Scenario: Losing the link does not drop the robot
    Given the robot is ARMED and driving forwards on operator command
    When the client connection is lost
    Then the commanded setpoints are zeroed
    And the robot continues to balance in place
    And the mode remains ARMED

  @REQ-LINK-009
  Scenario: Each robot advertises under its own address
    Given two robots with different factory identities
    When both robots are advertising
    Then their public addresses differ

  @REQ-LINK-009
  Scenario: The address survives a reset
    Given the robot is advertising under a public address
    When the robot is reset
    Then the robot advertises under the same public address

  @REQ-LINK-010
  Scenario: The console reports the link before the radio starts
    Given the radio has not reported its stack running
    When the operator asks the console for the link status
    Then the console reports that the radio is starting

  @REQ-LINK-010
  Scenario: The console reports a connected link
    Given a client is connected and has negotiated an attribute MTU of 247 bytes
    When the operator asks the console for the link status
    Then the console reports that the radio is connected
    And the console reports the public address
    And the console reports an attribute MTU of 247 bytes
