"""Generate the IMU-ablation figures used by the 2026-08-11 report."""

from __future__ import annotations

import csv
import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))

from uwb_dynamic_static_report import pdf_bar_chart, svg_bar_chart  # noqa: E402


SOURCE_DIR = pathlib.Path(__file__).resolve().parent
BUNDLE_DIR = SOURCE_DIR.parent
ANALYSIS_DIR = BUNDLE_DIR / "analysis"
FIGURES_DIR = BUNDLE_DIR / "figures"


def load_ablation() -> list[dict[str, str]]:
    with (ANALYSIS_DIR / "ablation_metrics.csv").open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def main() -> None:
    rows = load_ablation()
    labels = ["Native DS-TWR", "Passive DS-TWR"]
    protocol_keys = ["native_ds", "passive_ds"]
    configurations = [
        ("Raw UWB", "raw", "#64748b"),
        ("Accel disabled", "accel_disabled", "#2563eb"),
        ("Current fusion", "current", "#0f766e"),
        ("Relaxed gates", "accel_relaxed", "#f59e0b"),
    ]

    def values(metric: str, configuration: str) -> list[float]:
        result = []
        for protocol in protocol_keys:
            row = next(
                item
                for item in rows
                if item["protocol"] == protocol and item["configuration"] == configuration
            )
            result.append(float(row[metric]))
        return result

    rmse_series = [
        (label, values("rmse_cm", key), color)
        for label, key, color in configurations
    ]
    pdf_bar_chart(
        FIGURES_DIR / "09_imu_ablation_rmse.pdf",
        "Controlled replay ablation: dynamic RMSE",
        labels,
        rmse_series,
        "centimetres",
    )
    svg_bar_chart(
        FIGURES_DIR / "09_imu_ablation_rmse.svg",
        "Controlled replay ablation: dynamic RMSE",
        labels,
        rmse_series,
        "centimetres",
    )

    p95_series = [
        (label, values("p95_cm", key), color)
        for label, key, color in configurations
    ]
    pdf_bar_chart(
        FIGURES_DIR / "10_imu_ablation_p95.pdf",
        "Controlled replay ablation: dynamic P95",
        labels,
        p95_series,
        "centimetres",
    )
    svg_bar_chart(
        FIGURES_DIR / "10_imu_ablation_p95.svg",
        "Controlled replay ablation: dynamic P95",
        labels,
        p95_series,
        "centimetres",
    )

    propagation_labels = ["Native DS-TWR", "FlexTDOA", "Passive DS-TWR"]
    propagation = [18.84, 0.0, 0.0]
    pdf_bar_chart(
        FIGURES_DIR / "11_accel_propagation_coverage.pdf",
        "Valid IMU samples used for acceleration propagation",
        propagation_labels,
        [("current gates", propagation, "#dc2626")],
        "percent",
    )
    svg_bar_chart(
        FIGURES_DIR / "11_accel_propagation_coverage.svg",
        "Valid IMU samples used for acceleration propagation",
        propagation_labels,
        [("current gates", propagation, "#dc2626")],
        "percent",
    )


if __name__ == "__main__":
    main()
