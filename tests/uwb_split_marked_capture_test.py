import json
import pathlib
import tempfile
import unittest

from tools.uwb_split_marked_capture import split_capture


class SplitMarkedCaptureTest(unittest.TestCase):
    def test_splits_and_normalizes_protocol_envelope(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            source = root / "mixed.passive_ds.jsonl"
            records = [
                {"kind": "capture_start", "protocol": "passive_ds", "collector_wall_ns": 1_000_000_000},
                {"kind": "gps_fix", "protocol": "passive_ds", "collector_wall_ns": 2_100_000_000},
                {"kind": "position", "protocol": "native_ds", "tdoa_protocol": "native_ds", "collector_wall_ns": 2_200_000_000},
                {"kind": "position", "protocol": "passive_ds", "tdoa_protocol": "passive_ds", "collector_wall_ns": 2_300_000_000},
                {"kind": "capture_end", "protocol": "passive_ds", "collector_wall_ns": 4_000_000_000},
            ]
            source.write_text("".join(json.dumps(item) + "\n" for item in records), encoding="utf-8")
            markers = root / "markers.json"
            markers.write_text(
                json.dumps({"segments": [{"protocol": "native_ds", "start_unix_ms": 2000, "stop_unix_ms": 2500}]}),
                encoding="utf-8",
            )
            outputs = split_capture(source, markers, root / "out")
            payloads = [json.loads(line) for line in outputs[0].read_text(encoding="utf-8").splitlines()]
            self.assertEqual(payloads[0]["kind"], "capture_start")
            self.assertEqual(payloads[-1]["kind"], "capture_end")
            self.assertEqual([item["kind"] for item in payloads[1:-1]], ["gps_fix", "position"])
            self.assertTrue(all(item["protocol"] == "native_ds" for item in payloads))


if __name__ == "__main__":
    unittest.main()
