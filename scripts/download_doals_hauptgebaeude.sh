#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
package_dir="$(cd -- "$script_dir/.." && pwd)"
target_root="${1:-$package_dir/data/doals}"
scene_dir="$target_root/hauptgebaeude"
archive="$target_root/hauptgebaeude.zip"

# Current ETH Research Collection bitstream (DOI: 10.3929/ethz-c-000788171).
# Override without editing this script if ETH moves the file again:
#   DOALS_HG_URL=https://mirror.example/hauptgebaeude.zip ./download_doals_hauptgebaeude.sh
default_url="https://www.research-collection.ethz.ch/server/api/core/bitstreams/71c3fdb6-1260-4994-a591-f8d0ddd066e9/content?download=true"
download_url="${DOALS_HG_URL:-$default_url}"
expected_size=7462858321

usage() {
  cat <<EOF
Usage: $(basename "$0") [target_root]

Downloads and extracts the DOALS Hauptgebaeude scene (about 7.5 GB archive).
Default target: $package_dir/data/doals

Environment variables:
  DOALS_HG_URL       Override the archive URL (useful for a mirror).
  DOALS_KEEP_ARCHIVE Keep the zip after extraction when set to 1.
EOF
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  usage
  exit 0
fi
if [[ $# -gt 1 ]]; then
  usage >&2
  exit 2
fi

for command in curl unzip; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Missing required command: $command" >&2
    exit 1
  fi
done

sequence_1="$scene_dir/sequence_1/bag.bag"
sequence_2="$scene_dir/sequence_2/bag.bag"
if [[ -s "$sequence_1" && -s "$sequence_2" ]]; then
  echo "DOALS Hauptgebaeude is already available at: $scene_dir"
  exit 0
fi

mkdir -p "$target_root"
echo "Downloading DOALS Hauptgebaeude (~7.5 GB)"
echo "Source: $download_url"
echo "Target: $archive"

# --continue-at enables restart after interruption. The explicit browser user
# agent avoids overly aggressive filters on repository/CDN endpoints.
if ! curl --fail --location --continue-at - \
    --retry 20 --retry-all-errors --retry-delay 5 \
    --connect-timeout 30 --speed-time 120 --speed-limit 1024 \
    --user-agent "Mozilla/5.0 (X11; Linux x86_64) DOALS-downloader/1.0" \
    --output "$archive" "$download_url"; then
  cat >&2 <<EOF
Download failed. The legacy DOALS server is known to be unreliable.
Open the dataset landing page and copy a working Hauptgebaeude URL:
  https://doi.org/10.3929/ethz-c-000788171
Then retry with:
  DOALS_HG_URL='<url-or-mirror>' $0 '$target_root'
The partial archive was kept for resume.
EOF
  exit 1
fi

actual_size="$(stat -c '%s' "$archive")"
if [[ "$download_url" == "$default_url" && "$actual_size" -ne "$expected_size" ]]; then
  echo "Unexpected archive size: $actual_size bytes (expected $expected_size)." >&2
  echo "Keeping $archive for inspection; refusing to extract it." >&2
  exit 1
fi
if ! unzip -tq "$archive" >/dev/null; then
  echo "Archive integrity check failed: $archive" >&2
  echo "Delete the corrupt file or rerun the script to resume it." >&2
  exit 1
fi

echo "Extracting archive..."
unzip -n "$archive" -d "$target_root"

raw_1="$scene_dir/sequence_1/2020-02-20-11-58-45.bag"
raw_2="$scene_dir/sequence_2/2020-02-20-12-04-48.bag"
if [[ ! -s "$sequence_1" && -s "$raw_1" ]]; then
  mv "$raw_1" "$sequence_1"
fi
if [[ ! -s "$sequence_2" && -s "$raw_2" ]]; then
  mv "$raw_2" "$sequence_2"
fi
if [[ ! -s "$sequence_1" || ! -s "$sequence_2" ]]; then
  echo "Extraction completed, but the expected sequence bags were not found." >&2
  echo "Inspect: $scene_dir" >&2
  exit 1
fi

if [[ ${DOALS_KEEP_ARCHIVE:-0} != 1 ]]; then
  rm -f -- "$archive"
fi

echo "DOALS Hauptgebaeude is ready:"
echo "  $sequence_1"
echo "  $sequence_2"
