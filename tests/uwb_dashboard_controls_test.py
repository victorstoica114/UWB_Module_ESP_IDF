#!/usr/bin/env python3

import pathlib
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import INDEX_HTML  # noqa: E402


def javascript_block(start: str, end: str) -> str:
    start_index = INDEX_HTML.index(start)
    end_index = INDEX_HTML.index(end, start_index)
    return INDEX_HTML[start_index:end_index]


class DashboardControlStateTests(unittest.TestCase):
    def test_accelerometer_refresh_does_not_overwrite_unsaved_edit(self) -> None:
        function = javascript_block(
            "function updateAccelEnabledControl()",
            "function moduleHttpOnline",
        )
        dirty_guard = function.index(
            "if (state.accelSettingsDirty || state.accelApplyInFlight)"
        )
        live_checkbox_write = function.index("checkbox.checked = enabled === statuses.length")
        self.assertLess(dirty_guard, live_checkbox_write)
        self.assertIn('state.accelApplyInFlight ? "applying..." : "not applied"', function)

    def test_accelerometer_apply_waits_for_live_confirmation(self) -> None:
        apply_handler = javascript_block(
            'document.getElementById("applyAccelSample")',
            'document.getElementById("applyUwbSettings")',
        )
        self.assertIn("state.accelPendingApply = {", apply_handler)
        self.assertIn("const data = await postConfig", apply_handler)
        self.assertIn("if (!apiResponseOk(data))", apply_handler)
        self.assertIn("setAccelControlsDisabled(true)", apply_handler)

    def test_accelerometer_changes_mark_form_dirty(self) -> None:
        listeners = javascript_block(
            'document.getElementById("accelTargets")',
            'document.getElementById("applyUwbSettings")',
        )
        self.assertIn(
            'document.getElementById("accelSampleHz").addEventListener("input"',
            listeners,
        )
        self.assertIn(
            'document.getElementById("accelEnabled").addEventListener("change"',
            listeners,
        )
        self.assertGreaterEqual(listeners.count("state.accelSettingsDirty = true"), 2)


if __name__ == "__main__":
    unittest.main()
