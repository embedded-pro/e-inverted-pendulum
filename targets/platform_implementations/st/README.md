# platform_implementations/st

STM32 implementation of `platform::Platform`, shared by all supported ST board
presets: **NUCLEO-WB55RG**.

- **StatusLed** — LD2 (green) on PB0.
- **Communication** — `hal::UartStm` on USART1 (TX = PB6, RX = PB7), 115200 8N1.
  On the NUCLEO-WB55RG this UART is routed to the on-board ST-LINK virtual COM
  port, so no USB-UART adapter is needed.
- **Tracer** — `services::TracerToStream` over the UART.
- **Bluetooth** — `hal::TracingSystemTransportLayerWb` is a `PlatformImpl` member, so the
  wireless coprocessor (CPU2) boots when the board is constructed. Bonds are held in EMIL's
  `services::VolatileBondStorage`, synchronised with `hal::BondStorageSt`. Once CPU2 reports
  its stack running and `StartBluetooth` has been called, `BluetoothPeripheralStm` is built
  from hal-st only: `hal::TracingGapPeripheralSt` with Just Works and encryption, and
  `hal::TracingGattServerSt`, which reports the MTU the client negotiates (up to 251 bytes).
  The address and root keys come from the part's factory identity (see
  `documentation/design/ble-service.md`). Every HCI command and event is traced on the
  console.
- **Eeprom** — a 24C04 (512 bytes, address 0x50) through `drivers::Eeprom24xx` on I2C3:
  SCL = PB10, SDA = PB11. It holds the tuning record; see `documentation/design/persistence.md`.
- **Watchdog** — `hal::WatchDogStm` (WWDG) supervises the event loop through
  `services::EventDispatcherWithWeakPtrAndWatchdog`. An event that runs for 129–258 ms or more
  resets the part; see `documentation/design/platform-abstraction.md`, Part H. A debugger halt
  freezes the watchdog.
- **Run** — runs the supervised event dispatcher. A first member (`ClockInit`) calls
  `HAL_Init()` + the board's default clock configuration function before any
  peripheral is constructed (32 MHz HSE on the NUCLEO-WB55RG).

The default clock header/init function is selected per board in
`CMakeLists.txt` (via `INVERTED_PENDULUM_BOT_ST_CLOCK_HEADER` / `INVERTED_PENDULUM_BOT_ST_CLOCK_INIT`,
keyed off `TARGET_MCU`). To support another STM32 board, add a `TARGET_MCU`
case there pointing at the matching `hal_st` clock header/function, and add a
preset in `CMakePresets.json`.

## Peripheral and pin allocation (NUCLEO-WB55RG)

The design reason is in `documentation/design/platform-abstraction.md`, Part E.

The selected part has two general-purpose timers with an encoder mode, one low-power timer with
an encoder mode, and one timer with four independent output channels and a break input. The
motor driver's four bridge inputs want that four-channel timer, so that both motors switch in
phase and a single break input releases every input in hardware. That leaves the second
general-purpose timer and the low-power timer for the wheels.

| Timer  | Use                                                                     |
|--------|-------------------------------------------------------------------------|
| TIM1   | Motor driver bridge inputs, four channels, break input on the fault net |
| TIM2   | Right wheel encoder, x4 quadrature                                      |
| LPTIM1 | Left wheel encoder, x4 quadrature                                       |
| TIM16  | Free                                                                    |
| TIM17  | Free                                                                    |

The low-power timer's counter is 16 bits, which is ample for the configured encoder
resolution, and it has no per-phase polarity; the left wheel is the unmirrored one, so it does
not need any.

The serial buses are just as constrained. This part has two SPIs. Every I2C1 pin option
collides with the console UART, an encoder phase or a motor output. I2C3 is free on PA7/PC1
and on PB10/PB11; its other pins collide with SPI1 or SPI2. So:

| Bus  | Use                                                                                        |
|------|--------------------------------------------------------------------------------------------|
| SPI1 | Motor driver configuration and read-back, on PB3/PB4/PB5 with an active-high select on PA4 |
| SPI2 | Inertial sensor, with data-ready on PC6                                                    |

The reservation matters: the motor driver's configuration channel is the only other thing on
this board that needs a bus, and spending SPI1 on the sensor would have left it nowhere to go.

| Signal                       | Pins                                 |
|------------------------------|--------------------------------------|
| Bridge inputs A1, A2, B1, B2 | PA8, PA9, PA10, PA11 (TIM1 CH1–CH4)  |
| Driver fault                 | PA6 (TIM1 break) and PC4 (interrupt) |
| Driver sleep, reset          | PB8, PB9                             |
| Left encoder A, B, index     | PC0, PC2 (LPTIM1 IN1, IN2), PC5      |
| Right encoder A, B, index    | PA0, PA1 (TIM2 CH1, CH2), PC3        |

This is a board-level allocation, not part of the abstraction — the roles above say nothing
about timers or buses, and a board with more of them is free to spend them differently. It is
recorded here because it is the constraint that shaped three component designs, and
rediscovering it from the pinout tables is expensive.

## Wireless coprocessor

CPU2 must run ST's **full** BLE stack, flashed once per board with STM32CubeProgrammer.
The application only occupies the first 512 KB of flash, so it never overlaps the stack.

| Item    | Value                                                                                             |
|---------|---------------------------------------------------------------------------------------------------|
| Image   | `infra/hal/st/hal_st/middlewares/STM32_WPAN/STM32CubeWB/binaries/stm32wb5x_BLE_Stack_full_fw.bin` |
| Release | STM32CubeWB V1.17.0                                                                               |
| Address | `0x080CE000` (STM32WB55xG, 1 MB)                                                                  |

```bash
STM32_Programmer_CLI -c port=swd -fwdelete
STM32_Programmer_CLI -c port=swd -fwupgrade stm32wb5x_BLE_Stack_full_fw.bin 0x080CE000 firstinstall=1
STM32_Programmer_CLI -c port=swd -startwirelessstack
```

If the upgrade is refused, update the FUS first with the FUS image from the same
STM32CubeWB release, then repeat the stack upgrade.

### Bring-up checklist

1. After reset the console traces the CPU2 ready event and the GAP and GATT set-up, then
   advertising starts. `ble` reports `advertising` and the public address.
2. A client (nRF Connect, or a browser with Web Bluetooth) sees `inverted-pendulum`
   advertising the robot control service `c7a10001-5f6e-4d2b-9a3c-8e1f4b6d2a70`.
3. On connection `ble` reports `connected` and, once the exchange completes, an MTU above 23.
4. A mode write before pairing is refused by the stack; after Just Works pairing it is
   accepted.
5. Subscribing to telemetry delivers 25 notifications per second.
6. Disconnecting while driving slows the robot to a stop and advertising resumes.
