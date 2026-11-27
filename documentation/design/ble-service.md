---
title: "BLE Service Design"
type: design
status: draft
version: 0.3.2
component: "ble-service"
date: 2026-11-26
---

| Field     | Value              |
|-----------|--------------------|
| Title     | BLE Service Design |
| Type      | design             |
| Status    | draft              |
| Version   | 0.3.2              |
| Component | ble-service        |
| Date      | 2026-11-26         |

> The robot's only window on the world. Designed on the assumption that the link will
> fail at the worst moment, and that losing it must never drop a balancing robot.

---

## Responsibilities

- Presenting the robot control service as a GATT server and managing advertising and
  connection lifecycle.
- Carrying motion setpoints inward, and range-checking them before they reach the controller.
- Decaying setpoints to zero on operator silence, and zeroing them on link loss.
- Carrying mode commands — arm, disarm, clear fault, calibrate — inward to the safety supervisor.
- Streaming telemetry outward to a subscribed client without ever blocking the control loop.
- Carrying strategy selection and parameter access between the client and balance control.
- Enforcing that no commanding write is honoured before pairing.

---

## Component Details

### Part A — Service layout

One service carries four characteristic groups, separated by direction and lifetime rather
than by feature, so that a congested or dropped link degrades predictably:

- **Motion** — written frequently by the client while driving. The only high-rate inbound path.
- **Mode** — written rarely, with high consequence. Arm, disarm, clear fault, calibrate, plus the
  readable mode and latched fault cause.
- **Telemetry** — notified outward at a fixed rate while subscribed. Never written.
- **Tuning** — the active strategy, the list of available strategies, the parameter
  descriptor, and indexed parameter access.

### Part B — The command timeout

A balancing robot driving forward on a stale setpoint will keep going until it hits
something. The inbound motion path therefore carries a deadline: if no motion command
arrives within the timeout, the setpoints decay towards zero.

Decay, not a hard cut. Instantaneously zeroing the velocity setpoint of a leaning robot demands
a violent pitch correction. Ramping it down lets the balance loops bring the body upright as the
robot slows. The ramp is a fixed deceleration: 0.5 metres per second squared and 1.6 radians per
second squared. Braking therefore feels the same from any speed. From full speed the robot stops
in about two seconds. The robot keeps balancing throughout. The timeout removes the *motion*
command, not the *balancing* function.

### Part C — Link loss is not a fault

Losing the connection while ARMED zeroes the setpoints and leaves the robot balancing in
place. This is deliberate. Disarming on disconnection would drop the robot every time the
operator walked out of range. Dropping a robot is strictly worse than leaving it standing. The operator reconnects and takes control again, or disarms it when they arrive.

### Part D — Telemetry without backpressure

Telemetry is produced on the control loop and consumed by a link that may stall for tens of
milliseconds. The rule is that the producer never waits. A single most-recent sample is held for
transmission. If the link cannot take it before the next one is ready, the newer sample
overwrites the older one, which is lost. Dropping telemetry is free. Delaying the control loop is
not.

Each published sample is captured from a single control iteration. The client therefore never
sees a pitch from one moment beside an effort from another. A recorder observes every balance
iteration after the supervisor and balance control have run. It keeps these as one sample:

- the estimate,
- the measured chassis motion,
- the effort actually applied,
- the mode and fault cause.

The link reads that sample when it is time to notify.

A connected client is treated as subscribed. The link updates the telemetry value at the fixed
rate for as long as a client is connected. The Bluetooth stack only turns an update into a
notification for a client that has enabled notifications. The link therefore never tracks the
subscription itself.

### Part E — Generic tuning

The controller supplies a descriptor of the active strategy's parameters. This component
transports it. Parameters are addressed by index, and the descriptor supplies the count,
identity and permitted range. A client can therefore tune a strategy that did not exist when
the client was written, and selecting a different strategy simply changes the descriptor.

### Part F — Pairing

Any write that commands motion, changes mode, or modifies tuning requires a paired
connection. An unpaired client may discover the service and read, but cannot move the robot.

Pairing is Just Works with encryption. It needs no input on either side. Every commanding
characteristic demands an encrypted link, and the Bluetooth stack enforces this before a write
reaches the application. Just Works does not protect the pairing exchange itself against a man
in the middle. That is accepted for an operator robot that is paired within arm's reach. Bonds
are held in volatile memory until non-volatile storage exists, so a client pairs again after
every reset. The
tuning control point is itself a commanding characteristic, so tuning queries also need pairing.

### Part G — Attribute MTU

Every value is carried in little-endian IEEE 754 single precision, which a client decodes directly.
The larger values do not fit the default 23-byte attribute MTU. The robot is a GATT server only, so
it does not request an MTU itself. The stack is configured for up to 251 bytes. The client
negotiates the MTU after connecting, as current phones and browsers do. The GATT server reports
the agreed value. Until the exchange has raised the MTU enough, telemetry is not sent and tuning
responses carry only their status.

### Part H — Wire format

All UUIDs share the base `c7a1xxxx-5f6e-4d2b-9a3c-8e1f4b6d2a70`.

| Characteristic | UUID prefix | Properties             | Value                                                                                    |
|----------------|-------------|------------------------|------------------------------------------------------------------------------------------|
| Service        | c7a10001    | —                      | —                                                                                        |
| Motion         | c7a10002    | write without response | velocity float32 (m/s), yaw rate float32 (rad/s)                                         |
| Mode           | c7a10003    | read, write, notify    | written: one command byte; read: mode, cause, last command, last outcome                 |
| Telemetry      | c7a10004    | read, notify           | pitch, pitch rate, velocity, yaw rate, effort left, effort right as float32; mode, cause |
| Tuning         | c7a10005    | write, notify          | request and response, below                                                              |

| Code | Mode        | Code | Latched cause      | Code | Command     | Code | Outcome  |
|------|-------------|------|--------------------|------|-------------|------|----------|
| 0    | INIT        | 0    | None               | 1    | Arm         | 0    | None yet |
| 1    | CALIBRATING | 1    | Fall               | 2    | Disarm      | 1    | Accepted |
| 2    | IDLE        | 2    | Driver fault       | 3    | Clear fault | 2    | Refused  |
| 3    | ARMED       | 3    | Estimate invalid   | 4    | Calibrate   |      |          |
| 4    | FAULT       | 4    | Loop stalled       |      |             |      |          |
|      |             | 5    | Self-test failed   |      |             |      |          |
|      |             | 6    | Calibration failed |      |             |      |          |

Tuning requests start with an operation byte and an index byte. Every response repeats both and
adds a status: 0 accepted, 1 refused by balance control, 2 invalid request, 3 MTU too small.

| Operation              | Request                 | Response after the status                                              |
|------------------------|-------------------------|------------------------------------------------------------------------|
| 1 — describe strategy  | index                   | strategy count, active index, name length, name                        |
| 2 — select strategy    | index                   | nothing                                                                |
| 3 — describe parameter | index                   | parameter count, value, minimum, maximum as float32, name length, name |
| 4 — write parameter    | index, value as float32 | stored value as float32                                                |

A request that arrives while the previous response is still being sent is ignored. The client
waits for each response before it sends the next request.

### Part I — Device identity

Each robot advertises under a public address derived from the target's factory identity. Two
robots in one room therefore never collide, and a client recognises the same robot after a reset.
The address follows the vendor's scheme:

- the vendor's company identifier,
- the device identifier,
- the low bytes of the factory-programmed unique device number.

A part whose unique device number was never programmed falls back to a fold of its 96-bit unique
identifier in those low bytes.

The stack derives every bond's keys from the identity and encryption root keys. These root keys
come from the same 96-bit unique identifier, with a different constant per key. The two keys
therefore differ from each other and from every other robot. They are not secret. Anyone who can
read the part can recompute them. That matches Just Works, which already accepts an unauthenticated pairing exchange.

### Part J — Radio bring-up and status

The radio starts after the rest of the robot. The board boots the wireless coprocessor. Only
when the coprocessor reports that its stack is running does the robot build the service and
begin advertising. Until then the robot balances and answers the command line as usual.

The command line reports the link without a phone:

- whether the radio is still starting, advertising or connected,
- the public address,
- the negotiated attribute MTU.

---

## Interfaces

### Provided

| Interface              | Purpose                                                                        | Contract                                                                                                  |
|------------------------|--------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------|
| Robot control service  | The GATT service exposing all four characteristic groups                       | Discoverable while connected; single client                                                               |
| Advertising lifecycle  | Advertise while unconnected, resume after disconnection                        | At most one concurrent connection; a second is refused                                                    |
| Motion command intake  | Accept velocity and yaw-rate setpoints                                         | Pairing required; out-of-range values rejected, not clamped; setpoints decay to zero after the timeout    |
| Mode command intake    | Accept arm, disarm, clear-fault and calibrate                                  | Pairing required; forwarded to the supervisor, which applies its own preconditions                        |
| Telemetry notification | Publish robot state to a subscriber                                            | Fixed rate while subscribed; silent otherwise; never blocks the control loop; samples internally coherent |
| Tuning access          | Strategy list, active strategy, parameter descriptor, indexed parameter access | Pairing required for writes; the controller decides acceptance and this component reports the outcome     |
| Link status            | Report radio state, address and attribute MTU                                  | Readable at any time, including before the radio has started                                              |

### Required

| Interface            | Purpose                                                  | Contract                                                                  |
|----------------------|----------------------------------------------------------|---------------------------------------------------------------------------|
| Bluetooth peripheral | Advertising, connection, pairing, GATT database          | Started asynchronously; connection loss and MTU changes are observable    |
| Safety supervisor    | Forward mode commands; read mode and latched fault cause | The supervisor may reject any command; rejection is reported, not retried |
| Balance control      | Deliver setpoints; access strategies and parameters      | Rejected writes leave stored values unchanged                             |
| Telemetry source     | Obtain a coherent state sample                           | Sampled from one control iteration; non-blocking                          |
| Timebase             | Telemetry cadence and command timeout                    | Monotonic                                                                 |

Boundaries with the neighbouring components:

- The supervisor and the controller decide whether the current mode permits a command. This
  component forwards the command and reports the outcome.
- Balance control interprets parameters. This component transports opaque values against the
  descriptor the controller supplies.
- Persistence stores values. This component persists nothing.
- Balance control balances. A link failure is not a control event.

---

## Data Model

| Entity         | Field                      | Type / Unit                           | Range                                                                                          | Notes                                    |
|----------------|----------------------------|---------------------------------------|------------------------------------------------------------------------------------------------|------------------------------------------|
| Motion command | velocity setpoint          | metres per second                     | -1.0 to 1.0                                                                                    | Out-of-range rejected                    |
| Motion command | yaw rate setpoint          | radians per second                    | -1.6 to 1.6                                                                                    | Out-of-range rejected                    |
| Motion command | timeout                    | milliseconds                          | 500                                                                                            | Silence beyond this decays the setpoints |
| Mode           | active mode                | enumeration                           | INIT, CALIBRATING, IDLE, ARMED, FAULT                                                          | Read-only to the client                  |
| Mode           | latched cause              | enumeration                           | None, Fall, driver fault, estimate invalid, loop stalled, self-test failed, calibration failed | Meaningful in FAULT                      |
| Mode command   | command                    | enumeration                           | Arm, Disarm, Clear fault, Calibrate                                                            | Write-only; outcome reported             |
| Telemetry      | pitch, pitch rate          | radians, radians per second           | as estimated                                                                                   | From one control iteration               |
| Telemetry      | forward velocity, yaw rate | metres per second, radians per second | as measured                                                                                    | From the same iteration                  |
| Telemetry      | left and right effort      | normalised effort                     | -1.0 to 1.0                                                                                    | From the same iteration                  |
| Telemetry      | rate                       | notifications per second              | 25                                                                                             | While subscribed                         |
| Tuning         | strategy list              | list of identifiers                   | at least 2 entries                                                                             | Fixed at build time                      |
| Tuning         | active strategy            | identifier                            | one of the strategy list                                                                       | Writable only while not ARMED            |
| Tuning         | parameter descriptor       | count, identity and range per index   | strategy-defined                                                                               | Changes when the active strategy changes |
| Tuning         | parameter value            | value at an index                     | within the published range                                                                     | Writable only while not ARMED            |

---

## State Machine

```mermaid
stateDiagram-v2
    [*] --> Advertising
    Advertising --> Connected : Client connected
    Connected --> Advertising : Client disconnected
    Connected --> Paired : Pairing completed
    Paired --> Advertising : Client disconnected (setpoints zeroed)
    Paired --> Commanding : Motion command received
    Commanding --> Commanding : Further motion commands within the timeout
    Commanding --> Decaying : No motion command for the timeout
    Decaying --> Commanding : Motion command received
    Decaying --> Paired : Setpoints reached zero
    Paired --> Streaming : Telemetry subscribed
    Streaming --> Paired : Unsubscribed
```

---

## Sequence Diagrams

Connect, pair, arm, drive.

```mermaid
sequenceDiagram
    participant Op as Operator client
    participant Ble as BLE service
    participant Sup as Safety supervisor
    participant Ctrl as Balance control

    Op->>Ble: Connect
    Op->>Ble: Pair
    Ble-->>Op: Paired
    Op->>Ble: Subscribe to telemetry
    Op->>Ble: Arm
    Ble->>Sup: Arm request
    Sup-->>Ble: Mode = ARMED
    Ble-->>Op: Mode notification
    loop While driving
        Op->>Ble: Velocity and yaw setpoint
        Ble->>Ctrl: Setpoints
        Ctrl-->>Ble: State sample
        Ble-->>Op: Telemetry notification
    end
```

Operator falls silent, then the link drops.

```mermaid
sequenceDiagram
    participant Op as Operator client
    participant Ble as BLE service
    participant Ctrl as Balance control

    Note over Op,Ble: No motion command for 500 ms
    Ble->>Ctrl: Decay setpoints towards zero
    Note over Ctrl: Robot slows, keeps balancing
    Note over Op,Ble: Connection lost
    Ble->>Ctrl: Setpoints = zero
    Note over Ctrl: Mode remains ARMED, robot balances in place
    Ble->>Ble: Resume advertising
```

---

## Block Diagram

```mermaid
graph LR
    CLIENT[Operator client] -->|motion writes| MOTION[Motion intake]
    CLIENT -->|mode writes| MODE[Mode intake]
    CLIENT -->|tuning writes and reads| TUNE[Tuning access]
    MOTION -->|range check| WATCH[Timeout and decay]
    WATCH -->|setpoints| CTRL[Balance control]
    MODE --> SUP[Safety supervisor]
    TUNE --> CTRL
    CTRL -->|state sample| SLOT[Most-recent-sample slot]
    SLOT -->|notifications| CLIENT
    SUP -->|mode and fault cause| SLOT
    PAIR[Pairing state] -->|gates| MOTION
    PAIR -->|gates| MODE
    PAIR -->|gates| TUNE
```

---

## Constraints & Limitations

| Constraint                  | Value / Description                                                                                     |
|-----------------------------|---------------------------------------------------------------------------------------------------------|
| Radio start                 | The service exists only after the wireless coprocessor reports its stack running                        |
| Single client               | One concurrent connection; a second is refused rather than queued                                       |
| Command timeout             | 500 ms of silence begins setpoint decay                                                                 |
| Telemetry rate              | 25 notifications per second while subscribed                                                            |
| Non-blocking                | Telemetry production must never delay the balance loop; stale samples are dropped                       |
| Bounded buffers             | One most-recent sample is held; there is no unbounded transmit queue                                    |
| Link is not safety-critical | Loss of the link zeroes motion but never disarms; no safety function depends on it                      |
| Range, not latency          | Connection interval bounds how quickly a command takes effect; the control loops are unaffected by it   |
| Parameters by index         | The link addresses parameters by index; stored values are keyed by name ([persistence](persistence.md)) |

---

## Decisions

| # | Decision                                                                                 | Options                                                                                      | Status                              |
|---|------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------|-------------------------------------|
| 1 | Which pairing association model?                                                         | Just-works — simplest, no authenticated protection; passkey — needs an input or display path | decided: Just Works with encryption |
| 2 | Should the setpoint decay be a fixed ramp time or a fixed deceleration?                  | Fixed ramp; fixed deceleration matched to the balance envelope                               | decided: fixed deceleration         |
| 3 | Should telemetry rate be negotiable by the client?                                       | Fixed at 25 per second; client-selectable within bounds                                      | decided: fixed at 25 per second     |
| 4 | Should motion and mode commands share one characteristic to save a round trip?           | Separate — different rates and consequences; combined                                        | decided: separate                   |
| 5 | Should the service expose a diagnostic log, or is telemetry plus fault cause sufficient? | Telemetry only; add a fault history                                                          | decided: telemetry only             |
