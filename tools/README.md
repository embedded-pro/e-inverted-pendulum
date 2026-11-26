# tools

Host-side developer tools — CLI utilities, simulators, and bridges that help you
develop and validate the firmware. They build only for the host platform.

A tool links the same `core/` libraries the firmware uses. It does not duplicate them.

## Tools

| Tool         | Purpose                                        | Design                                    |
|--------------|------------------------------------------------|-------------------------------------------|
| `lqr_design` | LQR state gains from the measured robot values | `documentation/design/lqr-design-tool.md` |

## Qt GUI tools

The devcontainer ships Qt6, so GUI tools (e.g. a simulator) are supported. See
the commented `find_package(Qt6 ...)` block in `CMakeLists.txt` for how to enable
them, and the README's "How to Run the Simulator" section for X-server setup.

## Add a new tool

1. Make `tools/<your_tool>/` with a `Main.cpp` and a `CMakeLists.txt`.
2. In the `CMakeLists.txt`, add an executable `inverted_pendulum_bot.tool.<your_tool>`.
   Link the `core/` libraries it uses.
3. Add `add_subdirectory(<your_tool>)` to `tools/CMakeLists.txt`.
