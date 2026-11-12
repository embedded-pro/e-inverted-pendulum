# platform_implementations/st

STM32 implementation of `platform::Platform`, shared by all supported ST board
presets: **NUCLEO-WB55RG**.

- **StatusLed** — LD2 (green) on PB0.
- **Communication** — `hal::UartStm` on USART1 (TX = PB6, RX = PB7), 115200 8N1.
  On the NUCLEO-WB55RG this UART is routed to the on-board ST-LINK virtual COM
  port, so no USB-UART adapter is needed.
- **Tracer** — `services::TracerToStream` over the UART.
- **Bluetooth** — `hal::TracingSystemTransportLayerWb` is created once the bond store has
  recovered from flash (see [Persistence](#persistence)), so CPU2 boots with its bonds in
  place. Bonds are held in EMIL's `services::PersistentBondStorage`, synchronised with
  `hal::BondStorageSt`. Once CPU2 reports its stack running and `StartBluetooth` has been
  called, `BluetoothPeripheralStm` is built from hal-st only: `hal::TracingGapPeripheralSt`
  with Just Works and encryption, and `hal::TracingGattServerSt`, which reports the MTU the
  client negotiates (up to 251 bytes). The address and root keys come from the part's factory
  identity (see `documentation/design/ble-service.md`). Every HCI command and event is traced
  on the console.
- **ParameterStorage** — two 4 KB flash pages for the tuning store (see
  [Persistence](#persistence)).
- **Watchdog** — `hal::WatchDogStm` (WWDG), fed by the event loop. If the loop stalls for
  about 1.5 s the part resets, and the drive comes back up disabled.
- **Run** — runs `main_::StmEventInfrastructure`. A first member (`ClockInit`) calls
  `HAL_Init()` + the board's default clock configuration function before any
  peripheral is constructed (32 MHz HSE on the NUCLEO-WB55RG).

The default clock header/init function is selected per board in
`CMakeLists.txt` (via `INVERTED_PENDULUM_BOT_ST_CLOCK_HEADER` / `INVERTED_PENDULUM_BOT_ST_CLOCK_INIT`,
keyed off `TARGET_MCU`). To support another STM32 board, add a `TARGET_MCU`
case there pointing at the matching `hal_st` clock header/function, and add a
preset in `CMakePresets.json`.

## Persistence

Four flash pages between the end of the application's 512 KB and the CPU2 stack hold two
`services::ConfigurationStoreImpl` stores. Each store keeps two copies, and each copy is
verified by `services::Sha256Software`.

| Page | Address      | Content                                                               |
|------|--------------|-----------------------------------------------------------------------|
| 128  | `0x08080000` | Tuning, copy A                                                        |
| 129  | `0x08081000` | Tuning, copy B                                                        |
| 130  | `0x08082000` | Bonds (stack record and bonded addresses, `BondRecord.proto`), copy A |
| 131  | `0x08083000` | Bonds, copy B                                                         |

One `hal::FlashHomogeneousInternalStm` spans the whole flash, because pages are erased by
their absolute index. It is wrapped in `hal::FlashCoordinatedWithWirelessStack`, which
coordinates every write and erase with CPU2. `services::FlashMultipleAccess` shares it
between the two stores, and each copy is a `services::FlashRegion` of one page.

Boot order:
1. The flash starts in the `stopped` state, so both stores recover and may erase a stale
   page before CPU2 runs.
2. Once the bond store has recovered, the board calls `WirelessStackStarting()` and creates
   the transport. From then on, flash writes are held.
3. When CPU2 reports ready, `WirelessStackReady()` releases the held writes.

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
7. Change a parameter and select the other strategy, wait more than two seconds, then reset.
   `strategy` and `param` show the changed values.
8. After a reset, a paired client reconnects encrypted without pairing again.
