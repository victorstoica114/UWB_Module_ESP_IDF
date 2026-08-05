# UWB protocol boundaries

The firmware exposes three independent positioning protocols over one DW3000
radio driver.  The split is deliberately based on **state ownership**, because
stale cross-frame state was the source of protocol-switch regressions.

## Shared radio layer

`uwb_dw3000.c` owns only hardware-facing resources that cannot exist three
times on one transceiver: SPI/register access, IRQ/RX buffering, timestamps,
delayed TX primitives, timers and the runtime-mode router.  Its FlexTDOA and
Passive DS-TWR schedule adapters may call their own runtime facade, but may not
call a solver implementation directly.

`uwb_anchor_range_cache.c` implements generic cache mechanics.  It contains no
global cache.  A cache instance is always supplied by one protocol runtime.

## FlexTDOA

- `uwb_flex_tdoa_runtime.c`: Flex-only cache and solver lifecycle.
- `flextdoa_solver_service`: Flex observation/geometry solver task.
- The radio adapter submits only Flex observations through the runtime facade.

## Native DS-TWR

- `uwb_native_ds_twr.c`: complete POLL/RESP/FINAL state machine.
- `uwb_native_ds_runtime.c`: Native position solver and telemetry bridge.
- `uwb_native_ds_position_solver.c`: Native geometry and position math.

Native DS-TWR receives only generic radio callbacks from `uwb_dw3000.c`; it has
no dependency on either receive-only protocol.

## Passive DS-TWR

- `uwb_passive_ds_protocol.c`: PDS2 packet codec, rotating star plan, timing
  helpers and full anchor DS-TWR equation.
- `uwb_passive_ds_observation.c`: passive POLL/RESPONSE CFO correction and
  fixed-baseline range-difference equation.
- `uwb_passive_ds_runtime.c`: Passive-only anchor diagnostic cache, counters
  and raw solver lifecycle.
- `passive_ds_solver_service`: coherent-star raw AlgMin solver using fixed RTK
  ENU geometry.

The driver cannot obtain a mutable pointer to Passive runtime state.  It uses
explicit submit, record and snapshot operations instead.

## Switching invariant

Entering a protocol resets only that protocol's runtime.  FlexTDOA never reads
or resets the Passive cache; Passive DS-TWR never falls back to Flex ranges;
Native DS-TWR constructs a fresh stack runtime for every entry.  Shared radio
hardware is stopped/reconfigured by the router, independently of solver state.

The architectural guard is a dependency of the UWB component and therefore
runs automatically during every firmware build. It can also be run directly:

```sh
python3 tools/check_uwb_protocol_boundaries.py
idf.py build
```

The guard fails if the common driver calls a protocol solver directly, if one
runtime references another, if private state accessors reappear, or if an
isolated source file drops out of the ESP-IDF build.
