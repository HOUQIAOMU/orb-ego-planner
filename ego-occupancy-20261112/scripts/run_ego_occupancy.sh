#!/usr/bin/env bash
if [ -z "${BASH_VERSION:-}" ]; then exec bash "$0" "$@"; fi
set -euo pipefail
tool_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
project_root=$(cd -- "$tool_root/.." && pwd)
sensor_root=${EGO_SENSOR_ROOT:-$project_root}
binary=${EGO_OCCUPANCY_BINARY:-"$tool_root/build/release/bin/ego-rgbd-map"}
if [ ! -x "$binary" ]; then printf 'Missing ARM mapper: %s\n' "$binary" >&2; exit 1; fi
tag=$(TZ=Asia/Shanghai date +%Y%m%d-%H%M%S-%N)
output="$sensor_root/data/ego-occupancy-$tag"
camera=''
camera_backend=''
mapping_hz=''
dataset=''
check=false
viewer=true
forward=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --dataset|--output|--camera|--camera-backend|--camera-end|--mapping-hz)
            key=$1
            shift
            if [ "$#" -eq 0 ] || [ -z "$1" ]; then
                printf 'Missing value for %s\n' "$key" >&2
                exit 2
            fi
            case "$key" in
                --dataset) dataset=$1 ;;
                --output) output=$1 ;;
                --camera) camera=$1 ;;
                --camera-backend|--camera-end) camera_backend=$1 ;;
                --mapping-hz) mapping_hz=$1 ;;
            esac
            ;;
        --check) check=true ;;
        --no-viewer) viewer=false; forward+=("$1") ;;
        --live) ;; # Default mode; dataset switches to replay.
        --help|-h)
            printf '%s\n' 'Orbbec live defaults: SV1301S_U3 60 fps Rectified YAML and --mapping-hz 20.' \
                'Use --camera-backend (or --camera-end), --camera, and --mapping-hz to override.'
            exec "$binary" --help
            ;;
        *) forward+=("$1") ;;
    esac
    shift
done
# Resolve defaults after parsing so an explicit YAML or rate always wins.
default_camera="$sensor_root/config/D435i_042222071908_640x480_15.yaml"
if [ -z "$dataset" ]; then
    camera_backend=${camera_backend:-orbbec}
    case "$camera_backend" in
        orbbec)
            default_camera="$tool_root/config/cameras/SV1301S_U3_RGBD_640x480_60fps_rectified.yaml"
            mapping_hz=${mapping_hz:-20}
            ;;
        realsense) ;;
        *)
            printf 'Unknown camera backend: %s\n' "$camera_backend" >&2
            exit 2
            ;;
    esac
elif [ -n "$camera_backend" ]; then
    printf 'Camera backend selection requires live mode. Use --camera to select replay calibration.\n' >&2
    exit 2
fi

args=(--vocabulary "$sensor_root/ORB_SLAM3/Vocabulary/ORBvoc.txt"
      --camera "${camera:-$default_camera}" --map-config "$tool_root/config/ego-occupancy.yaml"
      --output "$output")
if [ -n "$dataset" ]; then
    args+=(--dataset "$dataset")
else
    args+=(--live --lock-root "$sensor_root" --camera-backend "$camera_backend")
fi
if [ -n "$mapping_hz" ]; then args+=(--mapping-hz "$mapping_hz"); fi
args+=("${forward[@]}")
"$binary" "${args[@]}" --check
if [ "$check" = true ]; then exit 0; fi
if [ "$viewer" = true ]; then
    export DISPLAY=${DISPLAY:-:0}
    export XAUTHORITY=${XAUTHORITY:-/home/lckfb/.Xauthority}
    if ! xwininfo -root >/dev/null 2>&1; then printf 'Cannot access the RK desktop; use --no-viewer or log in to the desktop.\n' >&2; exit 1; fi
fi
run_dir="$sensor_root/logs/ego/ego-occupancy-$tag"
mkdir -p "$sensor_root/logs/ego"
mkdir "$run_dir"
printf 'DATA_DIR=%s\nRUN_DIR=%s\n' "$output" "$run_dir"
printf '%q ' "$binary" "${args[@]}" > "$run_dir/command.txt"
printf '\n' >> "$run_dir/command.txt"
set +e
"$binary" "${args[@]}" 2>&1 | tee "$run_dir/run.log"
code=${PIPESTATUS[0]}
printf '%s\n' "$code" > "$run_dir/exit-code.txt"
exit "$code"
