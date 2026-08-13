#!/usr/bin/env python3

import json
import pathlib
import shutil
import subprocess
import sys
import unittest


sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))

from uwb_dashboard import INDEX_HTML  # noqa: E402


def javascript_block(start: str, end: str) -> str:
    start_index = INDEX_HTML.index(start)
    end_index = INDEX_HTML.index(end, start_index)
    return INDEX_HTML[start_index:end_index]


def run_gps_rtk_javascript(body: str) -> dict:
    node = shutil.which("node")
    if node is None:
        raise unittest.SkipTest("node is required for dashboard JavaScript tests")
    implementation = javascript_block(
        "const gpsRtkAnchorIds", "function gpsRtkEcef("
    ) + javascript_block(
        "function gpsRtkEcef(", "function gpsRtkToUwbAlignment("
    )
    harness = r"""
const state = {
  gpsRtkSamples: new Map(),
  gpsRtkLastTokens: new Map(),
  gpsRtkPendingJumps: new Map(),
  gpsRtkStatusByModule: new Map(),
  gpsRtkAnchorIdsKey: "",
  gpsRtkGeometryGenerationKey: "",
};
function statusIsFresh(item) { return Boolean(item?.http_status_online); }
function gpsRxIsFresh(item) {
  return Boolean(item?.runtime_gps_enabled && item?.gps_powered &&
    item?.gps_task_running && item?.gps_uart_ready &&
    Number(item?.gps_last_rx_age_ms) >= 0 &&
    Number(item?.gps_last_rx_age_ms) < 3000);
}
""" + implementation + body
    completed = subprocess.run(
        [node, "-e", harness],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(completed.stdout)


def run_gps_rtk_alignment_javascript(body: str) -> dict:
    node = shutil.which("node")
    if node is None:
        raise unittest.SkipTest("node is required for dashboard JavaScript tests")
    implementation = javascript_block(
        "function gpsRtkToUwbAlignment(",
        "function gpsRtkTagTrack(",
    )
    completed = subprocess.run(
        [node, "-e", implementation + body],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(completed.stdout)


class DashboardControlStateTests(unittest.TestCase):
    def test_high_rate_gps_events_drive_dashboard_without_full_snapshot(self) -> None:
        gps_stream = javascript_block(
            "function gpsEventStatusPatch(",
            "function snapshotPollDelayMs(",
        )
        self.assertIn('/api/gps-events?after=${state.gpsEventAfterId}', gps_stream)
        self.assertIn("estimated_measurement_wall_ns", gps_stream)
        self.assertIn("recordGpsRtkSamples(state.statuses)", gps_stream)
        self.assertIn("requestAnimationFrame", gps_stream)
        self.assertIn("schedulePositionStreamRender()", gps_stream)

        position_stream = javascript_block(
            "function renderPositionStreamFrame()",
            "function schedulePositionStreamRender()",
        )
        self.assertIn("positionKnownReference(", position_stream)

    def test_gps_rtk_recorder_retains_all_eight_hz_samples(self) -> None:
        result = run_gps_rtk_javascript(r"""
const base = {
  module_id: 1,
  http_status_online: true,
  runtime_gps_enabled: true,
  gps_powered: true,
  gps_task_running: true,
  gps_uart_ready: true,
  gps_last_rx_age_ms: 0,
  gps_last_fix_age_ms: 0,
  gps_fix_valid: true,
  gps_fix_quality: 4,
  gps_latitude_deg: 44.33617448,
  gps_longitude_deg: 25.94750246,
  gps_altitude_m: 76.72,
  gps_speed_mps: 0,
  uptime_ms: 10000,
  boot_guard_boot_count: 1,
};
const now = Date.now();
for (let sequence = 1; sequence <= 8; sequence += 1) {
  recordGpsRtkSamples([{
    ...base,
    gps_gga_count: sequence,
    gps_utc_ms_of_day: 43200000 + sequence * 125,
    _gps_event_captured_at_ms: now - (8 - sequence) * 125,
  }]);
}
console.log(JSON.stringify({
  count: state.gpsRtkSamples.get(1)?.length || 0,
}));
""")
        self.assertEqual(result["count"], 8)

    def test_gps_rtk_dedup_uses_binary_gga_sequence(self) -> None:
        recorder = javascript_block(
            "function recordGpsRtkSamples(",
            "function gpsRtkEcef(",
        )
        self.assertIn("item.gps_utc_ms_of_day", recorder)
        self.assertIn("item.gps_gga_count", recorder)
        self.assertIn("_gps_event_captured_at_ms", recorder)

    def test_native_position_bootstraps_without_gps_rtk(self) -> None:
        geometry = javascript_block(
            "function paperAnchorGeometry(",
            "function freshTdoaObservations(",
        )
        self.assertGreaterEqual(
            geometry.count("persistedModuleAnchorGeometry(anchorIds)"), 2
        )
        self.assertNotIn('protocol === "native_ds"\n        ? null', geometry)

        readout = javascript_block(
            "function renderPositionReadout(",
            "function trimPositionRateWindow(",
        )
        self.assertNotIn("Waiting for GPS RTK anchor geometry", readout)
        self.assertNotIn("every anchor reports RTK Fixed", readout)
        self.assertIn("GPS/RTK is optional", readout)

    def test_rtk_target_is_visible_for_every_protocol_even_when_fit_is_poor(self) -> None:
        reference = javascript_block(
            "function positionKnownReference(",
            "function percentile(",
        )
        self.assertIn("const comparisonValid", reference)
        self.assertIn("RTK target shown", reference)
        self.assertIn("comparisonValid,", reference)
        self.assertIn("comparisonWarning,", reference)
        self.assertNotIn(
            "GPS RTK reference blocked: rigid anchor fit RMS", reference
        )
        # This reference path is shared by FlexTDOA, Native DS-TWR and
        # Passive DS-TWR and must never branch on the selected protocol.
        self.assertNotIn("settings.solver", reference)

        metrics = javascript_block(
            "function positionReferenceErrorStats(",
            "function selectedPositionModuleIds(",
        )
        self.assertIn(
            "if (reference.comparisonValid === false) return null", metrics
        )

        drawing = javascript_block(
            "function drawPosition(model)",
            "function renderPositionGeometryPanel(",
        )
        self.assertIn("if (model.reference)", drawing)
        self.assertIn("ctx.arc(x, y, 9", drawing)
        self.assertIn('ctx.fillText("RTK target"', drawing)

    def test_rtk_overlay_accepts_mirrored_uwb_local_frame(self) -> None:
        result = run_gps_rtk_alignment_javascript(r"""
const geometry = {points: new Map([
  [2, {east: 0.0, north: 0.0}],
  [3, {east: 4.917, north: 3.526}],
  [4, {east: 1.081, north: 8.664}],
  [5, {east: -3.901, north: 5.673}],
])};
const anchors = {
  2: {x: 0.0, y: 0.0},
  3: {x: 0.0, y: 6.838},
  4: {x: 7.036, y: 5.980},
  5: {x: 7.410, y: 0.221},
};
const alignment = gpsRtkToUwbAlignment(geometry, anchors, [2, 3, 4, 5]);
console.log(JSON.stringify({
  reflected: alignment.reflected,
  anchorCount: alignment.anchorCount,
  scale: alignment.scale,
  rms: alignment.anchorFitRmsM,
}));
""")
        self.assertTrue(result["reflected"], result)
        self.assertEqual(result["anchorCount"], 4)
        self.assertGreater(result["scale"], 0)
        self.assertLess(result["rms"], 0.5)

    def test_rtk_overlay_fits_uniform_scale_without_moving_uwb_geometry(self) -> None:
        result = run_gps_rtk_alignment_javascript(r"""
const geometry = {points: new Map([
  [2, {east: 0.0, north: 0.0}],
  [3, {east: 0.0, north: 5.0}],
  [4, {east: 4.0, north: 5.0}],
  [5, {east: 4.0, north: 0.0}],
])};
const anchors = {
  2: {x: 0.0, y: 0.0},
  3: {x: 0.0, y: 6.0},
  4: {x: 4.8, y: 6.0},
  5: {x: 4.8, y: 0.0},
};
const alignment = gpsRtkToUwbAlignment(geometry, anchors, [2, 3, 4, 5]);
console.log(JSON.stringify({
  scale: alignment.scale,
  rms: alignment.anchorFitRmsM,
}));
""")
        self.assertAlmostEqual(result["scale"], 1.2, places=9)
        self.assertLess(result["rms"], 1e-9)

        tag_track = javascript_block(
            "function gpsRtkTagTrack(",
            "function positionGpsReferenceErrorStats(",
        )
        self.assertIn(
            "scale * (cosine * point.east - sine * point.north)", tag_track
        )
        self.assertIn(
            "scale * (sine * point.east + cosine * point.north)", tag_track
        )

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

    def test_position_trail_is_stable_until_operator_reset(self) -> None:
        append = javascript_block(
            "function appendPositionTrailPoint(",
            "function resetPositionTagTrail(",
        )
        draw = javascript_block(
            "function positionTrailDrawSamples(",
            "function localPositionAge(",
        )
        self.assertNotIn("positionTrailMaxAgeSec", append)
        self.assertNotIn("Math.ceil", draw)
        self.assertIn("return Array.isArray(trail) ? trail : []", draw)

    def test_position_viewport_is_frozen_from_complete_anchor_geometry(self) -> None:
        bounds = javascript_block(
            "function positionViewportKey(",
            "function positionTransform(",
        )
        self.assertNotIn("model.reference", bounds)
        self.assertNotIn("state.positionTrail", bounds)
        self.assertNotIn("state.positionRawTrail", bounds)
        self.assertIn(
            "if (state.positionViewport?.key === key) return state.positionViewport",
            bounds,
        )
        self.assertIn("if (completeAnchorGeometry) state.positionViewport = viewport", bounds)
        self.assertNotIn("viewport.minX =", bounds)
        reset = javascript_block(
            "function resetPositionTagTrails()",
            "function applyImuFusionSnapshot(",
        )
        self.assertIn("state.positionViewport = null", reset)

    def test_position_stream_rejects_stale_event_before_epoch_reset(self) -> None:
        ingest = javascript_block(
            "function ingestPositionStreamSample(",
            "function startPositionStream(",
        )
        stale_guard = ingest.index(
            "Number(previous?.position_event_id || 0) >= eventId"
        )
        epoch_update = ingest.index("ensurePositionTrailEpoch(key, item);", stale_guard)
        self.assertLess(stale_guard, epoch_update)

    def test_passive_raw_trail_records_valid_overlapping_solves(self) -> None:
        ingest = javascript_block(
            "function ingestPositionStreamSample(",
            "function startPositionStream(",
        )
        self.assertIn('item.tdoa_protocol === "passive_ds"', ingest)
        self.assertIn("passiveRawSolve || item.independent_frame !== false", ingest)
        self.assertIn("positionTrailTimestamp(item)", ingest)
        metrics = javascript_block(
            "function updatePositionStreamMetrics()",
            "function positionTdoaProtocolMatches(",
        )
        self.assertIn("all valid raw solves trail", metrics)

    def test_position_kalman_is_a_local_display_toggle(self) -> None:
        self.assertIn('id="positionKalmanEnabled"', INDEX_HTML)
        self.assertIn("Adaptive EKF (position-only, no IMU)", INDEX_HTML)
        settings = javascript_block(
            "function positionSettings()",
            "function synchronizePositionAnchorsFromRuntime",
        )
        self.assertIn("kalmanEnabled", settings)
        self.assertIn("positionKalmanEnabled", settings)
        wiring = javascript_block("function wireSettings()", "wireSettings();")
        self.assertIn('"positionKalmanEnabled"', wiring)
        self.assertIn("renderPosition();", wiring)

    def test_position_trail_overlays_are_independent_and_persisted(self) -> None:
        self.assertIn('id="positionShowRawTrail" type="checkbox" checked', INDEX_HTML)
        self.assertIn('id="positionShowEkfTrail" type="checkbox"', INDEX_HTML)
        settings = javascript_block(
            "function positionSettings()",
            "function synchronizePositionAnchorsFromRuntime",
        )
        self.assertIn("showRawTrail", settings)
        self.assertIn("showEkfTrail", settings)
        draw = javascript_block(
            "function drawPosition(model)",
            "function renderPositionGeometryPanel",
        )
        self.assertIn("if (model.settings.showRawTrail)", draw)
        self.assertIn("if (model.settings.showEkfTrail)", draw)
        persistence = javascript_block(
            "function persistedSettingIds()",
            "function restoreSettings()",
        )
        self.assertIn('"positionShowRawTrail"', persistence)
        self.assertIn('"positionShowEkfTrail"', persistence)
        restore = javascript_block(
            "function restoreSettings()",
            "function migrateRangingProfileDefaults",
        )
        self.assertIn("rawTrail.checked = true", restore)
        self.assertIn("positionKalmanEnabled", restore)

    def test_passive_kalman_reset_breaks_only_the_ekf_trail(self) -> None:
        append = javascript_block(
            "function appendPositionTrailPoint(",
            "function resetPositionTagTrail(",
        )
        self.assertIn("breakOnFilterReset = true", append)
        self.assertIn(
            "(breakOnFilterReset && filterReset)",
            append,
        )
        record = javascript_block(
            "function recordPositionTrailPoint(",
            "function positionTrailTimestamp(",
        )
        self.assertIn(
            "state.positionTrail, key, position, trailTimestamp, trailMetrics, false",
            record,
        )
        self.assertIn(
            "state.positionRawTrail, key, rawPosition, trailTimestamp, trailMetrics, false",
            record,
        )
        self.assertIn("state.positionKalmanTrail", record)

    def test_passive_correlated_window_holds_the_ekf_marker(self) -> None:
        held = javascript_block(
            "function heldPassiveKalmanPosition(",
            "function rawPositionFromSample(",
        )
        self.assertIn('sample?.kalman_reason !== "correlated_window_raw_bypass"', held)
        self.assertIn("state.positionKalmanTrail", held)
        stream_model = javascript_block(
            "function applyStreamPositionToModel(",
            "function updatePositionLiveMetrics(",
        )
        self.assertIn("const displayKalmanPosition = kalmanPosition || heldKalmanPosition", stream_model)
        self.assertIn("tag.heldPosition", stream_model)
        self.assertIn('"adaptive_ekf_position_only"', stream_model)
        live_metrics = javascript_block(
            "function updatePositionLiveMetrics(",
            "function renderPositionStreamFrame(",
        )
        self.assertIn('tag.heldPosition ? " held', live_metrics)
        self.assertIn("raw stream live", live_metrics)

    def test_position_stream_canvas_is_capped_without_decimating_ingest(self) -> None:
        constants = javascript_block("const accelLineRe", "const rangingProfileFields")
        self.assertIn("positionStreamRenderIntervalMs = 1000 / 30", constants)
        scheduler = javascript_block(
            "function schedulePositionStreamRender()",
            "function ingestPositionStreamSample(",
        )
        self.assertIn("positionStreamRenderIntervalMs", scheduler)
        self.assertIn("setTimeout(requestFrame, delayMs)", scheduler)
        self.assertIn("requestAnimationFrame(renderPositionStreamFrame)", scheduler)
        ingest = javascript_block(
            "function ingestPositionStreamSample(",
            "function startPositionStream(",
        )
        self.assertIn("recordPositionTrailPoint(", ingest)
        self.assertNotIn("positionTrailDrawSamples", ingest)

    def test_protocol_change_clears_trail_but_reboot_only_breaks_segment(self) -> None:
        epoch = javascript_block(
            "function ensurePositionTrailEpoch(",
            "function recordPositionTrailPoint(",
        )
        self.assertIn("previous.protocol !== protocol", epoch)
        self.assertIn("resetPositionTagTrail(key)", epoch)
        self.assertIn("if (rebooted)", epoch)
        self.assertIn("state.positionTrailEpochBreaks[key] = true", epoch)
        record = javascript_block(
            "function recordPositionTrailPoint(",
            "function positionTrailTimestamp(",
        )
        self.assertIn("trail_epoch_break: true", record)

    def test_position_kalman_keeps_separate_raw_and_filtered_trails(self) -> None:
        state = javascript_block("const state = {", "const accelLineRe")
        self.assertIn("positionKalmanTrail", state)
        record = javascript_block(
            "function recordPositionTrailPoint(",
            "function positionTrailTimestamp(",
        )
        self.assertIn("state.positionTrail", record)
        self.assertIn("state.positionKalmanTrail", record)
        reset = javascript_block(
            "function resetPositionTagTrail(",
            "function ensurePositionTrailEpoch(",
        )
        self.assertIn("delete state.positionKalmanTrail[key]", reset)

    def test_position_rtk_metrics_use_wall_time_not_esp_uptime(self) -> None:
        timestamp = javascript_block(
            "function positionTrailTimestamp(",
            "function resetPositionTagTrails()",
        )
        self.assertIn("sample?.received_at", timestamp)
        self.assertNotIn("uptimeMs / 1000", timestamp)
        gps_metrics = javascript_block(
            "function positionGpsReferenceErrorStats(",
            "function renderGpsRtkGeometryStatus()",
        )
        self.assertIn("nearestAge <= 0.25", gps_metrics)

    def test_position_rtk_metrics_do_not_block_live_rendering(self) -> None:
        live_metrics = javascript_block(
            "function updatePositionLiveMetrics(",
            "function renderPositionStreamFrame(",
        )
        self.assertIn("positionMetricsUpdateIntervalMs", live_metrics)
        self.assertIn("state.positionMetricsLastUpdateMs", live_metrics)
        gps_metrics = javascript_block(
            "function positionGpsReferenceErrorStats(",
            "function renderGpsRtkGeometryStatus()",
        )
        self.assertIn("let gpsIndex = 0", gps_metrics)
        self.assertIn("gpsIndex + 1 < gpsSamples.length", gps_metrics)
        self.assertNotIn("for (const gps of gpsSamples)", gps_metrics)
        snapshot_poll = javascript_block(
            "function snapshotPollDelayMs()",
            "function scheduleSnapshotPoll()",
        )
        self.assertIn('state.activeTab === "position") return 1000', snapshot_poll)

    def test_dynamic_rtk_reference_requires_current_fixed_reacquisition(self) -> None:
        result = run_gps_rtk_javascript(r"""
function fixedStatus(moduleId, ggaCount, quality = 4, fixAgeMs = 0) {
  return {
    module_id: moduleId,
    http_status_online: true,
    runtime_gps_enabled: true,
    gps_powered: true,
    gps_task_running: true,
    gps_uart_ready: true,
    gps_last_rx_age_ms: 0,
    gps_last_fix_age_ms: fixAgeMs,
    gps_fix_valid: true,
    gps_fix_quality: quality,
    gps_gga_count: ggaCount,
    boot_guard_boot_count: 7,
    gps_utc_date: "120826",
    gps_utc_time: String(ggaCount),
    gps_latitude_deg: 44.336 + moduleId * 0.00001,
    gps_longitude_deg: 25.947 + moduleId * 0.00001,
    gps_altitude_m: 76 + moduleId,
    gps_speed_mps: 0,
  };
}
const anchors = [2, 3, 4];
const statuses = count => anchors.map(id => fixedStatus(id, count));
recordGpsRtkSamples(statuses(1));
recordGpsRtkSamples(statuses(2));
const beforeThree = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) === null;
recordGpsRtkSamples(statuses(3));
const readyAfterThree = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) !== null;
recordGpsRtkSamples([
  fixedStatus(2, 4), fixedStatus(3, 4), fixedStatus(4, 4, 5),
]);
const floatBlocked = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) === null;
const floatCleared = (state.gpsRtkSamples.get(4) || []).length === 0;
recordGpsRtkSamples(statuses(5));
const firstFixedStillBlocked = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) === null;
recordGpsRtkSamples(statuses(6));
recordGpsRtkSamples(statuses(7));
const reacquiredAfterThree = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) !== null;
recordGpsRtkSamples([
  fixedStatus(2, 8), fixedStatus(3, 8), fixedStatus(4, 8, 4, 2000),
]);
const staleBlocked = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) === null;
console.log(JSON.stringify({beforeThree, readyAfterThree, floatBlocked,
  floatCleared, firstFixedStillBlocked, reacquiredAfterThree, staleBlocked}));
""")
        self.assertTrue(all(result.values()), result)

    def test_dynamic_rtk_reference_reacquires_after_gga_backstep(self) -> None:
        result = run_gps_rtk_javascript(r"""
function fixedStatus(moduleId, ggaCount) {
  return {
    module_id: moduleId,
    http_status_online: true,
    runtime_gps_enabled: true,
    gps_powered: true,
    gps_task_running: true,
    gps_uart_ready: true,
    gps_last_rx_age_ms: 0,
    gps_last_fix_age_ms: 0,
    gps_fix_valid: true,
    gps_fix_quality: 4,
    gps_gga_count: ggaCount,
    boot_guard_boot_count: 9,
    gps_utc_date: "120826",
    gps_utc_time: String(ggaCount),
    gps_latitude_deg: 44.336 + moduleId * 0.00001,
    gps_longitude_deg: 25.947 + moduleId * 0.00001,
    gps_altitude_m: 76 + moduleId,
    gps_speed_mps: 0,
  };
}
const anchors = [2, 3, 4];
const statuses = count => anchors.map(id => fixedStatus(id, count));
recordGpsRtkSamples(statuses(101));
recordGpsRtkSamples(statuses(102));
recordGpsRtkSamples(statuses(103));
const readyBeforeBackstep = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) !== null;
recordGpsRtkSamples(statuses(1));
const resetToOneSample = anchors.every(id =>
  (state.gpsRtkSamples.get(id) || []).length === 1);
const blockedAfterBackstep = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) === null;
recordGpsRtkSamples(statuses(2));
recordGpsRtkSamples(statuses(3));
const readyAfterReacquire = gpsRtkGeometryModel({
  latestOnly: true, requiredAnchorIds: anchors, minimumAnchorCount: 3,
}) !== null;
console.log(JSON.stringify({readyBeforeBackstep, resetToOneSample,
  blockedAfterBackstep, readyAfterReacquire}));
""")
        self.assertTrue(all(result.values()), result)


if __name__ == "__main__":
    unittest.main()
