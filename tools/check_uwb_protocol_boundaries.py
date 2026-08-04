#!/usr/bin/env python3
"""Fail when one UWB protocol reaches into another protocol's runtime.

This is intentionally a source-boundary test, not a behavioral radio test.
It protects the ownership rules that make hot protocol switching safe:
protocol-private caches, solvers and cross-frame state must not be shared.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
COMPONENT = ROOT / "components" / "uwb_dw3000"


def read(relative: str) -> str:
    return (COMPONENT / relative).read_text(encoding="utf-8")


def require_absent(
    failures: list[str], relative: str, patterns: tuple[str, ...]
) -> None:
    source = read(relative)
    for pattern in patterns:
        if re.search(pattern, source, flags=re.MULTILINE):
            failures.append(f"{relative}: forbidden pattern {pattern!r}")


def require_present(
    failures: list[str], relative: str, patterns: tuple[str, ...]
) -> None:
    source = read(relative)
    for pattern in patterns:
        if not re.search(pattern, source, flags=re.MULTILINE):
            failures.append(f"{relative}: required pattern {pattern!r} is missing")


def main() -> int:
    failures: list[str] = []

    # The common radio adapter may route events to a protocol facade, but it
    # must never bypass that facade and invoke a solver implementation.
    require_absent(
        failures,
        "uwb_dw3000.c",
        (
            r'#include\s+"flextdoa_solver_service\.h"',
            r'#include\s+"passive_ds_solver_service\.h"',
            r"\bflextdoa_solver_service_",
            r"\bpassive_ds_solver_service_",
            r"uwb_passive_ds_runtime_(?:stats|completed_exchange)\s*\(",
        ),
    )

    # Each facade may depend on the generic cache and its own solver only.
    for relative in ("uwb_flex_tdoa_runtime.c", "uwb_flex_tdoa_runtime.h"):
        require_absent(failures, relative, (r"passive_ds", r"native_ds"))
    for relative in ("uwb_passive_ds_runtime.c", "uwb_passive_ds_runtime.h"):
        require_absent(failures, relative, (r"flex_tdoa", r"native_ds"))
    for relative in (
        "uwb_native_ds_runtime.c",
        "uwb_native_ds_runtime.h",
        "uwb_native_ds_twr.c",
        "uwb_native_ds_twr.h",
        "uwb_native_ds_position_solver.c",
        "include/uwb_native_ds_position_solver.h",
    ):
        require_absent(failures, relative, (r"flex_tdoa", r"passive_ds"))

    # The two receive-only protocols must own distinct cache instances.
    require_present(
        failures,
        "uwb_flex_tdoa_runtime.c",
        (r"static struct uwb_anchor_range_cache s_anchor_range_cache;",),
    )
    require_present(
        failures,
        "uwb_passive_ds_runtime.c",
        (r"struct uwb_passive_ds_runtime_state", r"anchor_range_cache;"),
    )

    # Every isolated translation unit must remain part of the firmware build.
    require_present(
        failures,
        "CMakeLists.txt",
        (
            r'"uwb_flex_tdoa_runtime\.c"',
            r'"uwb_native_ds_runtime\.c"',
            r'"uwb_passive_ds_runtime\.c"',
            r'"uwb_anchor_range_cache\.c"',
        ),
    )

    # Extraction must preserve the old worst-case cache depth: one directed
    # observation for every configured anchor pair. A smaller fixed cache can
    # look correct with four anchors and silently regress larger deployments.
    require_present(
        failures,
        "uwb_anchor_range_cache.h",
        (
            r"UWB_ANCHOR_RANGE_CACHE_CAPACITY\s+\\\s*\n\s*"
            r"\(APP_RUNTIME_CONFIG_MAX_ANCHORS\s*\*\s*"
            r"\(APP_RUNTIME_CONFIG_MAX_ANCHORS\s*-\s*1U\)\)",
        ),
    )

    if failures:
        print("UWB protocol boundary check failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print("UWB protocol boundaries: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
