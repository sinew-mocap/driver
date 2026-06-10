# driver — sinew-mocap

A hexagon cluster (`core/` + `ports/` + `adapters/`) of the Sinew mocap stack:
the rebocap dongle driver — reads 36-byte serial frames and emits the /sinew OSC protocol; a terminal UI monitors it.

## Build

```
cmake -B build && cmake --build build
# tests: cmake -B build -DSINEW_TESTS=ON && cmake --build build && ctest --test-dir build
```

## Dependencies

See `ports/sibling-repos.txt` — clone the listed `sinew-mocap` repos side-by-side in `~/`.
