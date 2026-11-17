#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
result_dir="${1:-$(pwd)/results/matrix}"
mode="${2:-blocked}"
case "$mode" in
  clear) ;;
  blocked) ;;
  *) echo "Mode: blocked or clear" >&2; exit 2 ;;
esac
mkdir -p "$result_dir"
export RESULT_DIR="$(realpath "$result_dir")"
export MODE="$mode"
for start in 21 22 23 25 26 27; do
    for goal in 21 22 23 25 26 27; do
        if [[ "$start" != "$goal" ]]; then
            printf '%s %s\n' "$start" "$goal"
        fi
    done
done | xargs -P "${JOBS:-4}" -n 2 bash -c '
    set -e
    start=$0 goal=$1 name="$0-$1"
    extra=()
    [[ "$MODE" != clear ]] || extra+=(--clear)
    rm -f "$RESULT_DIR/$name.json"
    if docker run --rm --init --user "$(id -u):$(id -g)" -e HOME=/tmp -e USER=robot \
          -v "$RESULT_DIR:/results" turtle-courier:humble \
          --start "$start" --goal "$goal" --seed "$start$goal" "${extra[@]}" \
          --result "/results/$name.json" > "$RESULT_DIR/$name.log" 2>&1; then
        printf "PASS %s\n" "$name"
    else
        printf "FAIL %s\n" "$name" >&2
        exit 1
    fi
'
