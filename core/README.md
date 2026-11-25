# core

Reusable libraries only — **no application entry points** live here (those go in
[`targets/`](../targets)).

```
core/
├── platform_abstraction/     # platform::Platform interface — the hardware seam
│   ├── Platform.hpp          #   peripherals exposed to application logic
│   └── test_doubles/         #   PlatformMock (gmock) for host unit tests
├── balance_control/          # A component with the interfaces/implementations split
│   ├── interfaces/           #   Abstract INTERFACE libraries
│   └── implementations/      #   Compiled libraries + test/
└── cli/                      # Portable app: UART command-line interface
    ├── Cli.{hpp,cpp}         #   depends ONLY on platform::Platform
    └── test/                 #   unit-tested on the host against PlatformMock
```

**The key pattern**: `platform_abstraction/` defines an abstract interface; the
concrete board wiring lives in [`targets/platform_implementations/`](../targets/platform_implementations).
Application logic such as `cli` depends only on the interface, so it compiles
once and runs on the host (tested with `PlatformMock`) and on every board.

## Conventions (copy these for new components)

- **Interfaces vs implementations**: keep pure interfaces in `interfaces/` as
  `INTERFACE` CMake libraries; put compiled code in `implementations/`.
- **Library naming**: `inverted_pendulum_bot.<area>.<layer>` (e.g.
  `inverted_pendulum_bot.balance_control.implementations`).
- **Include paths** are rooted at the repository root, so headers are included as
  `#include "core/balance_control/implementations/BalanceControlImpl.hpp"`.
- **No heap in runtime/embedded code**: use `infra::BoundedVector`/`infra::BoundedString`
  from `infra/embedded-infra-lib`. Host tools and tests may use the heap.
- **Tests** live in a `test/` subfolder and register with `emil_add_test`.

## Add a new component

1. Write its design in `documentation/design/` first.
2. Lay it out like `balance_control/`: `interfaces/` and `implementations/`.
3. Add `add_subdirectory(<your_component>)` to `core/CMakeLists.txt`.
