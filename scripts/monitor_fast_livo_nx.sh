#!/usr/bin/env bash
# Lightweight FAST-LIVO2 process monitor for Jetson Xavier NX.
# Usage:
#   bash scripts/monitor_fast_livo_nx.sh | tee /tmp/fast_livo_mem.log
#   bash scripts/monitor_fast_livo_nx.sh <PID> 1 | tee /tmp/fast_livo_mem.log
set -u

PID="${1:-}"
INTERVAL="${2:-1}"

if [[ -z "${PID}" ]]; then
  PID="$(pgrep -n -f 'fastlivo_mapping' || true)"
fi

if [[ -z "${PID}" || ! -r "/proc/${PID}/status" ]]; then
  echo "fastlivo_mapping PID not found. Start FAST-LIVO2 first or pass PID explicitly." >&2
  exit 1
fi

echo "# monitoring PID=${PID}, interval=${INTERVAL}s"
echo "# timestamp rss_kb vm_size_kb vm_swap_kb threads cpu_percent"

while kill -0 "${PID}" 2>/dev/null; do
  TS="$(date '+%F %T')"
  RSS="$(awk '/^VmRSS:/{print $2}' /proc/${PID}/status)"
  VSZ="$(awk '/^VmSize:/{print $2}' /proc/${PID}/status)"
  SWAP="$(awk '/^VmSwap:/{print $2}' /proc/${PID}/status)"
  THREADS="$(awk '/^Threads:/{print $2}' /proc/${PID}/status)"
  CPU="$(ps -p "${PID}" -o %cpu= | xargs)"
  echo "${TS} ${RSS:-0} ${VSZ:-0} ${SWAP:-0} ${THREADS:-0} ${CPU:-0}"
  sleep "${INTERVAL}"
done

echo "# process ${PID} exited"
