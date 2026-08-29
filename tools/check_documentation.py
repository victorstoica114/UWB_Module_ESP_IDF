#!/usr/bin/env python3
"""Audit the repository README against selected firmware interfaces."""

from __future__ import annotations

import pathlib
import re
import sys
import urllib.parse


ROOT = pathlib.Path(__file__).resolve().parents[1]
README_PATH = ROOT / "README.md"
OTA_SOURCE_PATH = ROOT / "components" / "ota_service" / "ota_service.c"
APP_CONFIG_PATH = ROOT / "components" / "config" / "include" / "app_config.h"


def github_anchor(heading: str) -> str:
    """Return the common GitHub-style slug used by README fragment links."""
    slug = heading.strip().lower()
    slug = re.sub(r"[^\w\- ]", "", slug, flags=re.UNICODE)
    return re.sub(r"[ ]+", "-", slug)


def markdown_links(text: str) -> list[str]:
    return [
        match.group(1).strip().split(maxsplit=1)[0].strip("<>")
        for match in re.finditer(r"!?\[[^\]]*\]\(([^)]+)\)", text)
    ]


def markdown_section(text: str, heading: str) -> str:
    start_match = re.search(
        rf"^##\s+{re.escape(heading)}\s*$", text, re.MULTILINE
    )
    if start_match is None:
        raise RuntimeError(f"README section not found: {heading}")
    next_match = re.search(r"^##\s+", text[start_match.end() :], re.MULTILINE)
    end = len(text) if next_match is None else start_match.end() + next_match.start()
    return text[start_match.start() : end]


def audit_links(readme: str, errors: list[str]) -> None:
    if readme.count("```") % 2 != 0:
        errors.append("README contains an unclosed fenced code block")

    anchors = {
        github_anchor(match.group(2))
        for match in re.finditer(r"^(#{1,6})\s+(.+?)\s*$", readme, re.MULTILINE)
    }
    for target in markdown_links(readme):
        if not target:
            continue
        if target.startswith("#"):
            if urllib.parse.unquote(target[1:]).lower() not in anchors:
                errors.append(f"README fragment does not exist: {target}")
            continue
        parsed = urllib.parse.urlsplit(target)
        if parsed.scheme or target.startswith("//"):
            continue
        relative = urllib.parse.unquote(parsed.path)
        if relative and not (ROOT / relative).exists():
            errors.append(f"README local link does not exist: {target}")


def handler_body(source: str, handler_name: str) -> str:
    start_match = re.search(
        rf"static esp_err_t\s+{re.escape(handler_name)}\s*\(", source
    )
    if start_match is None:
        raise RuntimeError(f"handler not found: {handler_name}")
    next_match = re.search(r"\nstatic esp_err_t\s+\w+\s*\(", source[start_match.end() :])
    end = len(source) if next_match is None else start_match.end() + next_match.start()
    return source[start_match.start() : end]


def query_keys(body: str) -> set[str]:
    patterns = (
        r'httpd_query_key_value\s*\(\s*query\s*,\s*"([^"]+)"',
        r'ota_query_(?:option_enabled|has_key)\s*\(\s*query\s*,\s*"([^"]+)"',
        r'APPLY_(?:BOOL|U8|U32)_PARAM\s*\(\s*"([^"]+)"',
    )
    return {
        match.group(1)
        for pattern in patterns
        for match in re.finditer(pattern, body, re.DOTALL)
    }


def audit_http_and_commands(readme: str, source: str, errors: list[str]) -> None:
    http_section = markdown_section(readme, "HTTP Control Surface")
    routes = set(re.findall(r'\.uri\s*=\s*"([^"]+)"', source))
    for route in sorted(routes):
        if route not in http_section:
            errors.append(f"registered HTTP route is not documented: {route}")

    scoped_handlers = {
        "charger_config_post_handler": (None, "BQ25792 Battery Charger"),
        "max77958_config_post_handler": (None, "MAX77958 USB-C PD Controller"),
        "runtime_config_post_handler": ("bno085", "BNO085 IMU"),
    }
    for handler, (prefix, section_heading) in scoped_handlers.items():
        keys = query_keys(handler_body(source, handler))
        if prefix is not None:
            keys = {key for key in keys if key.startswith(prefix)}
        section = markdown_section(readme, section_heading)
        for key in sorted(keys):
            documented = re.search(
                rf"`[^`]*\b{re.escape(key)}\b[^`]*`", section
            )
            if documented is None:
                errors.append(f"{handler} query key is not documented: {key}")


def audit_components(readme: str, errors: list[str]) -> None:
    architecture = markdown_section(readme, "Firmware Architecture")
    component_names = sorted(
        path.parent.name
        for path in (ROOT / "components").glob("*/CMakeLists.txt")
    )
    for component in component_names:
        if f"`{component}`" not in architecture:
            errors.append(f"compiled component is missing from architecture table: {component}")


def macro_value(config: str, name: str) -> str:
    match = re.search(rf"^#define\s+{re.escape(name)}\s+(.+?)\s*$", config, re.MULTILINE)
    if match is None:
        raise RuntimeError(f"configuration macro not found: {name}")
    return match.group(1)


def audit_selected_defaults(readme: str, config: str, errors: list[str]) -> None:
    bno_section = markdown_section(readme, "BNO085 IMU")
    macros = (
        "APP_BNO085_ACCEL_TEST_ENABLED",
        "APP_BNO085_I2C_ADDRESS",
        "APP_BNO085_I2C_CLOCK_HZ",
        "APP_BNO085_ACCEL_INTERVAL_MS",
        "APP_BNO085_LOG_INTERVAL_MS",
        "APP_BNO085_INT_WAIT_TIMEOUT_MS",
    )
    for name in macros:
        expected = f"#define {name} {macro_value(config, name)}"
        if expected not in bno_section:
            errors.append(f"README does not show the current value: {expected}")

    required_protocol_markers = (
        "| `22` acceleration | `26` bytes |",
        "| `23` orientation | `32` bytes |",
        "| `24` clock anchor | `20` bytes |",
    )
    for marker in required_protocol_markers:
        if marker not in bno_section:
            errors.append(f"current BNO085 telemetry format is missing: {marker}")


def main() -> int:
    readme = README_PATH.read_text(encoding="utf-8")
    ota_source = OTA_SOURCE_PATH.read_text(encoding="utf-8")
    app_config = APP_CONFIG_PATH.read_text(encoding="utf-8")
    errors: list[str] = []

    audit_links(readme, errors)
    audit_http_and_commands(readme, ota_source, errors)
    audit_components(readme, errors)
    audit_selected_defaults(readme, app_config, errors)

    if errors:
        print(f"Documentation audit failed with {len(errors)} issue(s):", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print("Documentation audit passed: local links, HTTP routes, hardware commands, components, and selected defaults are synchronized.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
