# targets

Application **entry points** (`main`) and **platform-specific implementations**.
Libraries with no `main` belong in [`core/`](../core).

```
targets/
├── cli/                   # Headline app: one Main.cpp reused on every platform
│   └── Main.cpp                  #   includes PLATFORM_IMPL_HEADER, constructs Cli
└── platform_implementations/
    ├── host/                     # PlatformImpl: stubs + loopback serial (host preset)
    └── st/                       # PlatformImpl: NUCLEO-WB55RG
```

## How platform selection works

`INVERTED_PENDULUM_BOT_TARGET_PLATFORM` (set per preset in `CMakePresets.json`) picks which
`platform_implementations/<platform>` subdirectory is compiled. Each provides a
`PlatformImpl : platform::Platform` (see `core/platform_abstraction/`) exposing the
peripherals the application needs.

`targets/CMakeLists.txt` builds an interface library `inverted_pendulum_bot.platform_impl`
that (a) defines `PLATFORM_IMPL_HEADER` pointing at the selected platform's
`PlatformImpl.hpp` and (b) links its library. A target's `Main.cpp` includes that
macro and constructs `application::PlatformImpl`, so the entry point stays
platform-agnostic — the same `cli/Main.cpp` builds for host and ST.

- **host** builds under `cmake --preset host`.
- **st** builds the `NUCLEO-WB55RG` firmware.

## Add a new application

1. Put the portable logic in `core/` against `platform::Platform` (see `core/cli/`).
2. Copy `cli/` to `targets/<your_app>/`; keep `Main.cpp` platform-agnostic.
3. Add `add_subdirectory(<your_app>)` in `targets/CMakeLists.txt`.
