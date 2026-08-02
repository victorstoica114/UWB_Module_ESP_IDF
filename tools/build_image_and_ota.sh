#!/usr/bin/env bash
set -euo pipefail

umask 077

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "${script_dir}/.." && pwd)"
output_root="${UWB_FIRMWARE_IMAGE_DIR:-${project_root}/firmware_images}"
target_list="${project_root}/tools/ota_targets.local.txt"
parallel=5
skip_ota=0
keep_build=0
build_dir=""

usage() {
  printf '%s\n' \
    "Usage: tools/build_image_and_ota.sh [options]" \
    "" \
    "Builds in /tmp, creates a versioned full-flash image and OTA binary," \
    "then uploads the OTA binary to all configured targets in parallel." \
    "" \
    "Options:" \
    "  --skip-ota              Build and archive only" \
    "  --target-list PATH      OTA target list (default: tools/ota_targets.local.txt)" \
    "  --output-root PATH      Image archive root (default: firmware_images)" \
    "  --parallel COUNT        Simultaneous OTA uploads (default: 5)" \
    "  --keep-build            Keep the temporary build directory" \
    "  -h, --help              Show this help"
}

while (($#)); do
  case "$1" in
    --skip-ota)
      skip_ota=1
      shift
      ;;
    --target-list)
      target_list="$2"
      shift 2
      ;;
    --output-root)
      output_root="$2"
      shift 2
      ;;
    --parallel)
      parallel="$2"
      shift 2
      ;;
    --keep-build)
      keep_build=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$parallel" =~ ^[1-9][0-9]*$ ]]; then
  printf 'Invalid --parallel value: %s\n' "$parallel" >&2
  exit 2
fi

if [[ "$target_list" != /* ]]; then
  target_list="${project_root}/${target_list}"
fi
if [[ "$output_root" != /* ]]; then
  output_root="${project_root}/${output_root}"
fi

idf_export="${IDF_EXPORT:-${IDF_PATH:-/home/pi/esp/esp-idf}/export.sh}"
if [[ ! -f "$idf_export" ]]; then
  printf 'ESP-IDF export script not found: %s\n' "$idf_export" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$idf_export" >/dev/null 2>&1

cleanup() {
  if [[ "$keep_build" -eq 1 || -z "$build_dir" ]]; then
    return
  fi
  case "$build_dir" in
    /tmp/uwb-release-build.*)
      find "$build_dir" -depth -delete 2>/dev/null || true
      ;;
    *)
      printf 'Refusing to clean unexpected build path: %s\n' "$build_dir" >&2
      ;;
  esac
}
trap cleanup EXIT

build_dir="$(mktemp -d /tmp/uwb-release-build.XXXXXX)"
git_commit="$(git -C "$project_root" rev-parse HEAD)"
git_short="$(git -C "$project_root" rev-parse --short=12 HEAD)"
git_describe="$(git -C "$project_root" describe --always --dirty)"
dirty_suffix=""
if [[ -n "$(git -C "$project_root" status --porcelain --untracked-files=normal)" ]]; then
  dirty_suffix="-dirty"
fi
timestamp="$(date +%Y%m%d_%H%M%S)"
release_name="${timestamp}_${git_short}${dirty_suffix}"
release_dir="${output_root}/${release_name}"

mkdir -p "$release_dir"

printf 'Building firmware in %s\n' "$build_dir"
idf.py -C "$project_root" -B "$build_dir" build

printf 'Creating merged full-flash image\n'
idf.py -C "$project_root" -B "$build_dir" merge-bin \
  -o "$release_dir/uwb_full_flash.bin"

install -m 0600 "$build_dir/uwb_esp_idf.bin" \
  "$release_dir/uwb_ota.bin"
install -m 0600 "$build_dir/bootloader/bootloader.bin" \
  "$release_dir/bootloader.bin"
install -m 0600 "$build_dir/partition_table/partition-table.bin" \
  "$release_dir/partition-table.bin"
install -m 0600 "$build_dir/ota_data_initial.bin" \
  "$release_dir/ota_data_initial.bin"
install -m 0600 "$build_dir/flasher_args.json" \
  "$release_dir/flasher_args.json"
install -m 0600 "$project_root/sdkconfig" \
  "$release_dir/sdkconfig"

idf_version="$(idf.py --version)"
built_at="$(date --iso-8601=seconds)"
cat >"$release_dir/README.txt" <<EOF
UWB ESP32-S3 firmware image

Source commit: ${git_commit}
Source state:  ${git_describe}
ESP-IDF:       ${idf_version}
Built at:      ${built_at}

The binaries contain the compiled device configuration and must be treated as
sensitive. This directory is intentionally excluded from Git.

Clean full restore over USB:
  esptool.py --chip esp32s3 erase_flash
  esptool.py --chip esp32s3 write_flash 0x0 uwb_full_flash.bin

Normal network update:
  python3 tools/ota_upload.py --firmware "${release_dir}/uwb_ota.bin" --target-list "${target_list}" --parallel ${parallel}
EOF

(
  cd "$release_dir"
  sha256sum \
    uwb_full_flash.bin uwb_ota.bin bootloader.bin partition-table.bin \
    ota_data_initial.bin flasher_args.json sdkconfig README.txt \
    >SHA256SUMS
)

python3 - "$release_dir" "$git_commit" "$git_describe" \
  "$idf_version" "$built_at" "$target_list" <<'PY'
import hashlib
import json
import pathlib
import sys

release_dir = pathlib.Path(sys.argv[1])
files = {}
for path in sorted(release_dir.iterdir()):
    if not path.is_file() or path.name in {"manifest.json", "SHA256SUMS"}:
        continue
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    files[path.name] = {"bytes": path.stat().st_size, "sha256": digest}

manifest = {
    "format": 1,
    "chip": "esp32s3",
    "source_commit": sys.argv[2],
    "source_state": sys.argv[3],
    "esp_idf": sys.argv[4],
    "built_at": sys.argv[5],
    "ota_target_list": str(pathlib.Path(sys.argv[6]).resolve()),
    "full_restore": [
        "esptool.py --chip esp32s3 erase_flash",
        "esptool.py --chip esp32s3 write_flash 0x0 uwb_full_flash.bin",
    ],
    "files": files,
}
(release_dir / "manifest.json").write_text(
    json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
)
PY

(
  cd "$release_dir"
  sha256sum manifest.json >>SHA256SUMS
)

printf 'Firmware archive: %s\n' "$release_dir"

if [[ "$skip_ota" -eq 0 ]]; then
  if [[ ! -f "$target_list" ]]; then
    printf 'OTA target list not found: %s\n' "$target_list" >&2
    exit 1
  fi
  printf 'Uploading OTA image to all targets in parallel\n'
  python3 "$project_root/tools/ota_upload.py" \
    --firmware "$release_dir/uwb_ota.bin" \
    --target-list "$target_list" \
    --parallel "$parallel"
else
  printf 'OTA skipped by request\n'
fi

printf 'Build, image archive and OTA workflow completed successfully\n'
