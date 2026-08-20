#!/usr/bin/env bash
set -euo pipefail

# Release development-tool memory without touching ROS, SSH, the camera,
# the lidar, the base driver, or this terminal's ancestor process chain.

MODE="safe"
DRY_RUN=false
for arg in "$@"; do
  case "$arg" in
    --full-vscode) MODE="full" ;;
    --dry-run) DRY_RUN=true ;;
    -h|--help)
      cat <<'EOF'
Usage: prepare_competition_memory.sh [--dry-run] [--full-vscode]

Default safe mode stops only heavy development helpers:
  - Pylance language servers
  - C/C++ IntelliSense (cpptools and cpptools-srv)
  - GitHub Copilot language server

--full-vscode also stops VS Code extension hosts. Use it only after closing
the remote VS Code windows; the current SSH process chain is still protected.
No ROS, navigation, sensor, camera, base-driver, SSH or Codex process is
selected by this script.
EOF
      exit 0
      ;;
    *)
      echo "[ERROR] Unknown option: $arg" >&2
      exit 2
      ;;
  esac
done

declare -A PROTECTED=()
pid=$$
while [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )); do
  PROTECTED["$pid"]=1
  pid="$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
done

available_mb() {
  awk '/^MemAvailable:/ {printf "%d", $2 / 1024}' /proc/meminfo
}

print_memory() {
  local label="$1"
  echo "[INFO] $label: available=$(available_mb)MB"
  free -h | sed -n '1,2p'
}

collect_pattern() {
  local label="$1"
  local pattern="$2"
  local pid command
  while read -r pid; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    [[ -n "${PROTECTED[$pid]:-}" ]] && continue
    [[ "$pid" -eq "$$" ]] && continue
    command="$(ps -o args= -p "$pid" 2>/dev/null || true)"
    [[ -n "$command" ]] || continue
    TARGET_PIDS["$pid"]="$label"
    TARGET_COMMANDS["$pid"]="$command"
  done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
}

declare -A TARGET_PIDS=()
declare -A TARGET_COMMANDS=()

collect_pattern "Pylance" '/\.vscode-server/extensions/ms-python\.vscode-pylance-.*/dist/server\.bundle\.js'
collect_pattern "C/C++ IntelliSense" '/\.vscode-server/extensions/ms-vscode\.cpptools-.*/bin/cpptools([[:space:]]|$)'
collect_pattern "C/C++ IntelliSense helper" 'cpptools-srv'
collect_pattern "GitHub Copilot" '/\.vscode-server/.*@github/copilot'

if [[ "$MODE" == "full" ]]; then
  collect_pattern "VS Code extension host" '/\.vscode-server/.*/out/bootstrap-fork --type=extensionHost'
fi

print_memory "memory before cleanup"

if (( ${#TARGET_PIDS[@]} == 0 )); then
  echo "[INFO] No matching development helper is running."
  exit 0
fi

echo "[INFO] Selected development processes (mode=$MODE):"
for pid in "${!TARGET_PIDS[@]}"; do
  rss_kb="$(ps -o rss= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  rss_mb=$(( ${rss_kb:-0} / 1024 ))
  printf '  pid=%s rss=%sMB type=%s\n' "$pid" "$rss_mb" "${TARGET_PIDS[$pid]}"
  printf '    %s\n' "${TARGET_COMMANDS[$pid]}"
done

if [[ "$DRY_RUN" == "true" ]]; then
  echo "[INFO] Dry run only; no process was stopped."
  exit 0
fi

for pid in "${!TARGET_PIDS[@]}"; do
  kill -TERM "$pid" 2>/dev/null || true
done

deadline=$((SECONDS + 4))
while (( SECONDS < deadline )); do
  alive=false
  for pid in "${!TARGET_PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      alive=true
      break
    fi
  done
  [[ "$alive" == "false" ]] && break
  sleep 0.2
done

for pid in "${!TARGET_PIDS[@]}"; do
  if kill -0 "$pid" 2>/dev/null; then
    echo "[WARN] Development helper pid=$pid ignored SIGTERM; sending SIGKILL."
    kill -KILL "$pid" 2>/dev/null || true
  fi
done

sleep 0.5
print_memory "memory after cleanup"

available="$(available_mb)"
if (( available < 1800 )); then
  echo "[WARN] Less than 1800MB remains available. Close remote VS Code windows" >&2
  echo "[WARN] and rerun with --full-vscode before starting the competition." >&2
  exit 3
fi

echo "[INFO] Competition memory preparation complete; ROS and hardware processes were untouched."
