#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"
export ROBOCUP_WS="${ROBOCUP_WS:-$WORKSPACE_ROOT}"

DEFAULT_CONFIG="${REPO_ROOT}/config/default_jt128.yaml"
DEFAULT_MAP="${REPO_ROOT}/data/new_map"
DEFAULT_EXTERNAL_MAP="${REPO_ROOT}/data/external_map"
DEFAULT_PANGOLIN_PREFIX="${HOME}/.local/pangolin-0.9.3"
DEFAULT_BAG_LIDAR_TOPIC="/lidar_points"
DEFAULT_BAG_IMU_TOPIC="/lidar_imu"
DEFAULT_NAV_SCRIPT="${WORKSPACE_ROOT}/bringup/scripts/run_go2w_superloc_navigation.sh"
DEFAULT_UNITREE_PARAMS="${WORKSPACE_ROOT}/unitree_adapter/config/unitree_adapter.yaml"
DEFAULT_RVIZ_CONFIG="${WORKSPACE_ROOT}/bringup/rviz/robocup_navigation.rviz"

MODE=""
BAG_PATH=""
PCD_PATH=""
PRIOR_MAP_PCD=""
CONFIG_PATH="${DEFAULT_CONFIG}"
MAP_PATH=""
MAP_SOURCE_PCD=""
MAP_CONVERT_NEEDED=0
MAP_WAS_PCD=0
RUN_NAME=""
WAIT_SECONDS="2"
SOURCE_SETUP=1
BAG_LIDAR_TOPIC="${DEFAULT_BAG_LIDAR_TOPIC}"
BAG_IMU_TOPIC="${DEFAULT_BAG_IMU_TOPIC}"
AUTO_BAG_TOPIC_OVERRIDE=1
CONVERT_OVERWRITE=0
VIS_MODE="${JT128_VIZ_MODE:-rviz}"
LOC_ENABLE_RVIZ=1
RVIZ_CONFIG="${DEFAULT_RVIZ_CONFIG}"
NAV_SCRIPT="${DEFAULT_NAV_SCRIPT}"
NAV_SKIP_BUILD=0
NAV_ENABLE_RVIZ=1
NAV_DRY_RUN=0
NAV_BOUNDARY_FILE=""
NAV_BOUNDARY_DISABLED=0
NAV_UNITREE_BACKEND="${UNITREE_BACKEND_TYPE:-sdk2}"
NAV_ROBOT_MODEL="${UNITREE_ROBOT_MODEL:-a2}"
NAV_NETWORK_INTERFACE="${UNITREE_NETWORK_INTERFACE:-}"
NAV_ADAPTER_ENABLED_ON_START=0
NAV_ODOM_TIMEOUT="${EXTERNAL_LOCALIZATION_ODOM_TIMEOUT_SEC:-30}"
NAV_UNITREE_PARAMS_FILE="${UNITREE_ADAPTER_PARAMS_FILE:-${DEFAULT_UNITREE_PARAMS}}"
APP_PID=""
RVIZ_PID=""
TMP_CONFIG=""
STARTED_PID=""
BAG_PLAY_ARGS=()
EXTRA_ARGS=()

usage() {
    cat <<'USAGE'
Usage:
  scripts/run_jt128.sh lio-bag --bag BAG [--config CFG] [-- BAG_PLAY_ARGS...]
  scripts/run_jt128.sh loc-bag --bag BAG [--map MAP_DIR_OR_PCD] [--config CFG] [-- BAG_PLAY_ARGS...]
  scripts/run_jt128.sh lio-live [--config CFG]
  scripts/run_jt128.sh loc-live [--map MAP_DIR_OR_PCD] [--config CFG]
  scripts/run_jt128.sh nav-live [--map MAP_DIR_OR_PCD] [--prior-map-pcd PCD] [--config CFG] [NAV_OPTIONS]
  scripts/run_jt128.sh lio-offline --bag BAG [--config CFG]
  scripts/run_jt128.sh loc-offline --bag BAG [--map MAP_DIR_OR_PCD] [--config CFG]
  scripts/run_jt128.sh convert-map --pcd INPUT.pcd [--map OUT_DIR] [--overwrite] [-- EXTRA_ARGS...]

Modes:
  lio-bag       Start online LIO/SLAM and play a ROS 2 bag.
  loc-bag       Start online localization, load a map, and play a ROS 2 bag.
  lio-live      Start online LIO/SLAM for real sensors.
  loc-live      Start online localization for real sensors and load a map.
  nav-live      Start Lightning localization, wait for RViz initial pose, then
                start TRG/follower/collision guard/Unitree adapter with
                external localization topics.
  lio-offline   Run the built-in offline bag reader for LIO/SLAM.
  loc-offline   Run the built-in offline bag reader for localization.
  convert-map   Convert an external PCD into a Lightning-LM tiled map directory.

Options:
  -b, --bag BAG       ROS 2 bag directory or db3 file.
      --pcd PCD       External PCD used by convert-map.
  -c, --config CFG    Config file. Default: config/default_jt128.yaml
  -m, --map MAP       Localization map directory, or an external PCD.
                      If MAP is a PCD, the script reuses a same-stem converted
                      Lightning map directory when present, otherwise converts first.
                      Default: data/new_map for loc, data/external_map for convert-map.
      --prior-map-pcd PCD
                      TRG prior map PCD for nav-live. Default: MAP/global.pcd.
      --run-name NAME Output/run name for nav-live.
      --skip-build    Reuse existing navigation install trees in nav-live.
      --viz MODE      Visualization mode for loc-bag/loc-live:
                      rviz, pangolin, or none. Default: rviz.
                      rviz waits for /initialpose; pangolin/none start from identity.
      --pangolin      Shortcut for --viz pangolin.
      --rviz          Shortcut for --viz rviz.
      --no-rviz       Do not launch RViz in loc-bag/loc-live/nav-live.
      --rviz-config RVIZ
                      RViz config for loc-bag/loc-live/nav-live.
      --dry-run       Do not launch Unitree adapter in nav-live.
      --network-interface IFACE
                      Unitree SDK2 network interface for nav-live.
      --unitree-backend TYPE
                      Unitree adapter backend for nav-live. Default: sdk2.
      --robot-model MODEL
                      Unitree model for nav-live. Default: a2.
      --unitree-params-file YAML
                      Unitree adapter params for nav-live.
      --adapter-enabled-on-start
                      Start Unitree adapter enabled. Default is disabled.
      --boundary-file YAML
                      TRG allowed-area polygon for nav-live.
      --no-boundary   Disable TRG boundary in nav-live.
      --external-odom-timeout SEC
                      Wait timeout for first /laser_odometry before navigation. Default: 30.
      --overwrite     Replace convert-map output directory if it already exists.
      --bag-lidar-topic TOPIC
                      Bag lidar topic used when the bag does not match CFG.
                      Default: /lidar_points
      --bag-imu-topic TOPIC
                      Bag IMU topic used when the bag does not match CFG.
                      Default: /lidar_imu
      --no-auto-bag-topics
                      Do not generate a temporary config for detected bag topics.
      --wait SEC      Delay before ros2 bag play in *-bag modes. Default: 2
      --no-source     Do not source ROS/workspace setup files.
  -h, --help          Show this help.

Arguments after -- are passed to "ros2 bag play"; in convert-map they go to
"convert_pcd_to_map"; in nav-live they are forwarded to the navigation script.
Example:
  scripts/run_jt128.sh loc-bag --bag ~/bags/jt128 --map ./data/new_map -- --clock --rate 0.5
  scripts/run_jt128.sh loc-live --map ~/maps/site.pcd
  scripts/run_jt128.sh convert-map --pcd ~/maps/site.pcd --map ./data/site_map --overwrite -- --voxel_size 0.1
  scripts/run_jt128.sh nav-live --map ./data/site_map --skip-build --network-interface enp3s0
Note:
  For --map ~/maps/site.pcd, the auto-converted Lightning map directory is
  ~/maps/site by default. If ~/maps/site already contains index.txt, it is reused.
  In --viz rviz, loc-bag starts ros2 bag play paused. Set the initial pose in
  RViz, then press space in the bag-play terminal.
  In --viz pangolin, Lightning's Pangolin window is enabled and localization
  starts from identity; use this only when the map/bag frames are already aligned.
USAGE
}

die() {
    echo "error: $*" >&2
    exit 1
}

note() {
    echo "[jt128] $*" >&2
}

start_background() {
    if command -v setsid >/dev/null 2>&1; then
        setsid "$@" &
    else
        "$@" &
    fi
    STARTED_PID=$!
}

start_background_logged() {
    local log_file="$1"
    shift

    if command -v setsid >/dev/null 2>&1; then
        setsid "$@" >"${log_file}" 2>&1 &
    else
        "$@" >"${log_file}" 2>&1 &
    fi
    STARTED_PID=$!
}

terminate_process_group() {
    local pid="$1"
    [[ -n "${pid}" ]] || return 0

    kill -TERM -- "-${pid}" 2>/dev/null || true
    kill -TERM "${pid}" 2>/dev/null || true
    sleep 1
    kill -KILL -- "-${pid}" 2>/dev/null || true
    kill -KILL "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
}

cleanup() {
    local rc=$?
    trap - EXIT INT TERM

    terminate_process_group "${APP_PID}"
    APP_PID=""

    terminate_process_group "${RVIZ_PID}"
    RVIZ_PID=""

    if [[ -n "${TMP_CONFIG}" && -f "${TMP_CONFIG}" ]]; then
        rm -f "${TMP_CONFIG}"
    fi

    exit "${rc}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

abs_path() {
    local path="$1"
    if command -v realpath >/dev/null 2>&1; then
        realpath -m "${path}"
    else
        printf '%s\n' "${path}"
    fi
}

abs_path_preserve_final_component() {
    local path="$1"
    local dir
    local base

    dir="$(dirname "${path}")"
    base="$(basename "${path}")"
    printf '%s/%s\n' "$(abs_path "${dir}")" "${base}"
}

yaml_double_quote_escape() {
    local value="$1"
    value="${value//\\/\\\\}"
    value="${value//\"/\\\"}"
    printf '%s' "${value}"
}

yaml_section_value() {
    local src="$1"
    local section="$2"
    local key="$3"

    awk -v section="${section}" -v key="${key}" '
        /^[^[:space:]#][^:]*:[[:space:]]*($|#)/ {
            current = $0
            sub(/:.*/, "", current)
            in_section = (current == section)
        }

        in_section && $0 ~ "^[[:space:]]+" key ":[[:space:]]*" {
            value = $0
            sub(/^[^:]*:[[:space:]]*/, "", value)
            sub(/[[:space:]]*#.*$/, "", value)
            sub(/^[[:space:]]*/, "", value)
            sub(/[[:space:]]*$/, "", value)
            sub(/^"/, "", value)
            sub(/"$/, "", value)
            print value
            exit
        }
    ' "${src}"
}

is_pcd_path() {
    local path="$1"
    [[ "${path,,}" == *.pcd ]]
}

same_stem_map_dir_for_pcd() {
    local pcd="$1"
    local dir
    local base

    dir="$(dirname "${pcd}")"
    base="$(basename "${pcd}")"
    base="${base%.*}"
    printf '%s/%s\n' "${dir}" "${base}"
}

is_lightning_map_dir() {
    local map_dir="$1"
    [[ -d "${map_dir}" && -f "${map_dir}/index.txt" ]]
}

resolve_localization_map_input() {
    local input="$1"
    local pcd
    local same_stem_dir
    local legacy_dir

    if [[ -f "${input}" ]] && is_pcd_path "${input}"; then
        pcd="$(abs_path_preserve_final_component "${input}")"
        same_stem_dir="$(same_stem_map_dir_for_pcd "${pcd}")"
        legacy_dir="${same_stem_dir}_lightning_map"

        MAP_SOURCE_PCD="${pcd}"
        MAP_WAS_PCD=1

        if [[ "${CONVERT_OVERWRITE}" -eq 0 ]] && is_lightning_map_dir "${same_stem_dir}"; then
            MAP_PATH="${same_stem_dir}"
            MAP_CONVERT_NEEDED=0
            note "using existing converted Lightning map: ${MAP_PATH}"
        elif [[ "${CONVERT_OVERWRITE}" -eq 0 ]] && is_lightning_map_dir "${legacy_dir}"; then
            MAP_PATH="${legacy_dir}"
            MAP_CONVERT_NEEDED=0
            note "using existing converted Lightning map: ${MAP_PATH}"
        elif [[ "${CONVERT_OVERWRITE}" -eq 0 && -e "${same_stem_dir}" ]]; then
            die "same-stem path exists but is not a Lightning map: ${same_stem_dir}; pass --overwrite to replace it"
        else
            MAP_PATH="${same_stem_dir}"
            MAP_CONVERT_NEEDED=1
            note "will convert PCD to Lightning map: ${MAP_SOURCE_PCD} -> ${MAP_PATH}"
        fi
        return
    fi

    MAP_PATH="$(abs_path "${input}")"
    if [[ ! -d "${MAP_PATH}" ]]; then
        die "map path not found: ${MAP_PATH}"
    fi
    if ! is_lightning_map_dir "${MAP_PATH}"; then
        die "map directory is not a Lightning map (missing index.txt): ${MAP_PATH}; pass the source .pcd to auto-convert"
    fi
}

auto_convert_map_if_needed() {
    if [[ "${MAP_CONVERT_NEEDED}" -eq 0 ]]; then
        return
    fi

    local convert_args=(--input_pcd "${MAP_SOURCE_PCD}" --output_map "${MAP_PATH}")
    if [[ "${CONVERT_OVERWRITE}" -eq 1 ]]; then
        convert_args+=(--overwrite)
    fi

    note "+ ros2 run lightning convert_pcd_to_map ${convert_args[*]}"
    ros2 run lightning convert_pcd_to_map "${convert_args[@]}"

    if ! is_lightning_map_dir "${MAP_PATH}"; then
        die "conversion finished but Lightning map index was not created: ${MAP_PATH}/index.txt"
    fi
}

bag_metadata_path() {
    if [[ -d "${BAG_PATH}" ]]; then
        printf '%s\n' "${BAG_PATH}/metadata.yaml"
    else
        printf '%s\n' "$(dirname "${BAG_PATH}")/metadata.yaml"
    fi
}

bag_has_topic() {
    local metadata="$1"
    local topic="$2"
    [[ -f "${metadata}" ]] || return 1
    awk -v topic="${topic}" '$1 == "name:" && $2 == topic { found = 1 } END { exit(found ? 0 : 1) }' "${metadata}"
}

config_with_overrides() {
    local src="$1"
    local map="${2:-}"
    local lidar_topic="${3:-}"
    local imu_topic="${4:-}"
    local with_ui="${5:-}"
    local with_2dui="${6:-}"
    local enable_rviz="${7:-}"
    local auto_start="${8:-}"
    local out
    local escaped_map
    local escaped_lidar_topic
    local escaped_imu_topic

    out="$(mktemp "${TMPDIR:-/tmp}/lightning-jt128.XXXXXX.yaml")"
    escaped_map="$(yaml_double_quote_escape "${map}")"
    escaped_lidar_topic="$(yaml_double_quote_escape "${lidar_topic}")"
    escaped_imu_topic="$(yaml_double_quote_escape "${imu_topic}")"

    awk -v map="${escaped_map}" -v lidar_topic="${escaped_lidar_topic}" -v imu_topic="${escaped_imu_topic}" \
        -v with_ui="${with_ui}" -v with_2dui="${with_2dui}" -v enable_rviz="${enable_rviz}" \
        -v auto_start="${auto_start}" '
        function finish_section() {
            if (section == "common") {
                if (lidar_topic != "" && !replaced_lidar) {
                    print "  lidar_topic: \"" lidar_topic "\""
                    replaced_lidar = 1
                }
                if (imu_topic != "" && !replaced_imu) {
                    print "  imu_topic: \"" imu_topic "\""
                    replaced_imu = 1
                }
            }
            if (section == "system") {
                if (map != "" && !replaced_map) {
                    print "  map_path: \"" map "\""
                    replaced_map = 1
                }
                if (with_ui != "" && !replaced_with_ui) {
                    print "  with_ui: " with_ui
                    replaced_with_ui = 1
                }
                if (with_2dui != "" && !replaced_with_2dui) {
                    print "  with_2dui: " with_2dui
                    replaced_with_2dui = 1
                }
                if (enable_rviz != "" && !replaced_enable_rviz) {
                    print "  enable_lidar_loc_rviz: " enable_rviz
                    replaced_enable_rviz = 1
                }
                if (auto_start != "" && !replaced_auto_start) {
                    print "  auto_start_from_identity: " auto_start
                    replaced_auto_start = 1
                }
            }
        }

        BEGIN {
            section = ""
            saw_common = 0
            saw_system = 0
            replaced_lidar = 0
            replaced_imu = 0
            replaced_map = 0
            replaced_with_ui = 0
            replaced_with_2dui = 0
            replaced_enable_rviz = 0
            replaced_auto_start = 0
        }

        /^[^[:space:]#][^:]*:[[:space:]]*($|#)/ {
            finish_section()
            section = $0
            sub(/:.*/, "", section)
            if (section == "common") {
                saw_common = 1
            }
            if (section == "system") {
                saw_system = 1
            }
        }

        section == "common" && lidar_topic != "" && /^[[:space:]]+lidar_topic:[[:space:]]*/ {
            print "  lidar_topic: \"" lidar_topic "\""
            replaced_lidar = 1
            next
        }

        section == "common" && imu_topic != "" && /^[[:space:]]+imu_topic:[[:space:]]*/ {
            print "  imu_topic: \"" imu_topic "\""
            replaced_imu = 1
            next
        }

        section == "system" && map != "" && /^[[:space:]]+map_path:[[:space:]]*/ {
            print "  map_path: \"" map "\""
            replaced_map = 1
            next
        }

        section == "system" && with_ui != "" && /^[[:space:]]+with_ui:[[:space:]]*/ {
            print "  with_ui: " with_ui
            replaced_with_ui = 1
            next
        }

        section == "system" && with_2dui != "" && /^[[:space:]]+with_2dui:[[:space:]]*/ {
            print "  with_2dui: " with_2dui
            replaced_with_2dui = 1
            next
        }

        section == "system" && enable_rviz != "" && /^[[:space:]]+enable_lidar_loc_rviz:[[:space:]]*/ {
            print "  enable_lidar_loc_rviz: " enable_rviz
            replaced_enable_rviz = 1
            next
        }

        section == "system" && auto_start != "" && /^[[:space:]]+auto_start_from_identity:[[:space:]]*/ {
            print "  auto_start_from_identity: " auto_start
            replaced_auto_start = 1
            next
        }

        { print }

        END {
            finish_section()
            if (!saw_common && (lidar_topic != "" || imu_topic != "")) {
                print ""
                print "common:"
                if (lidar_topic != "") {
                    print "  lidar_topic: \"" lidar_topic "\""
                }
                if (imu_topic != "") {
                    print "  imu_topic: \"" imu_topic "\""
                }
            }
            if (!saw_system && (map != "" || with_ui != "" || with_2dui != "" || enable_rviz != "" || auto_start != "")) {
                print ""
                print "system:"
                if (map != "") {
                    print "  map_path: \"" map "\""
                }
                if (with_ui != "") {
                    print "  with_ui: " with_ui
                }
                if (with_2dui != "") {
                    print "  with_2dui: " with_2dui
                }
                if (enable_rviz != "") {
                    print "  enable_lidar_loc_rviz: " enable_rviz
                }
                if (auto_start != "") {
                    print "  auto_start_from_identity: " auto_start
                }
            }
        }
    ' "${src}" > "${out}"

    TMP_CONFIG="${out}"
    printf '%s\n' "${out}"
}

detect_bag_topic_override() {
    local config_lidar_topic
    local config_imu_topic
    local metadata

    [[ "${AUTO_BAG_TOPIC_OVERRIDE}" -eq 1 ]] || return

    metadata="$(bag_metadata_path)"
    [[ -f "${metadata}" ]] || return

    config_lidar_topic="$(yaml_section_value "${CONFIG_PATH}" common lidar_topic)"
    config_imu_topic="$(yaml_section_value "${CONFIG_PATH}" common imu_topic)"

    if bag_has_topic "${metadata}" "${config_lidar_topic}" && bag_has_topic "${metadata}" "${config_imu_topic}"; then
        return
    fi

    if bag_has_topic "${metadata}" "${BAG_LIDAR_TOPIC}" && bag_has_topic "${metadata}" "${BAG_IMU_TOPIC}"; then
        CONFIG_LIDAR_TOPIC_OVERRIDE="${BAG_LIDAR_TOPIC}"
        CONFIG_IMU_TOPIC_OVERRIDE="${BAG_IMU_TOPIC}"
        note "bag topics: ${BAG_LIDAR_TOPIC}, ${BAG_IMU_TOPIC} (temporary config override)"
    fi
}

bag_play_arg_present() {
    local expected="$1"
    local arg
    for arg in "${BAG_PLAY_ARGS[@]}"; do
        [[ "${arg}" == "${expected}" ]] && return 0
    done
    return 1
}

normalize_visualization_mode() {
    VIS_MODE="${VIS_MODE,,}"
    case "${VIS_MODE}" in
        rviz|pangolin|none)
            ;;
        off|headless)
            VIS_MODE="none"
            ;;
        pango)
            VIS_MODE="pangolin"
            ;;
        *)
            die "unknown --viz mode: ${VIS_MODE}; expected rviz, pangolin, or none"
            ;;
    esac
}

visualization_overrides() {
    CONFIG_WITH_UI_OVERRIDE=""
    CONFIG_WITH_2DUI_OVERRIDE=""
    CONFIG_ENABLE_RVIZ_OVERRIDE=""
    CONFIG_AUTO_START_OVERRIDE=""

    case "${MODE}" in
        loc-bag|loc-live)
            case "${VIS_MODE}" in
                rviz)
                    CONFIG_WITH_UI_OVERRIDE="false"
                    CONFIG_WITH_2DUI_OVERRIDE="false"
                    CONFIG_ENABLE_RVIZ_OVERRIDE="true"
                    CONFIG_AUTO_START_OVERRIDE="false"
                    note "visualization mode: RViz (/initialpose gate)"
                    ;;
                pangolin)
                    CONFIG_WITH_UI_OVERRIDE="true"
                    CONFIG_WITH_2DUI_OVERRIDE="false"
                    CONFIG_ENABLE_RVIZ_OVERRIDE="false"
                    CONFIG_AUTO_START_OVERRIDE="true"
                    LOC_ENABLE_RVIZ=0
                    note "visualization mode: Pangolin (identity initial pose)"
                    ;;
                none)
                    CONFIG_WITH_UI_OVERRIDE="false"
                    CONFIG_WITH_2DUI_OVERRIDE="false"
                    CONFIG_ENABLE_RVIZ_OVERRIDE="false"
                    CONFIG_AUTO_START_OVERRIDE="true"
                    LOC_ENABLE_RVIZ=0
                    note "visualization mode: none (identity initial pose)"
                    ;;
            esac
            ;;
        lio-bag|lio-live)
            case "${VIS_MODE}" in
                pangolin)
                    CONFIG_WITH_UI_OVERRIDE="true"
                    CONFIG_WITH_2DUI_OVERRIDE="false"
                    note "visualization mode: Pangolin"
                    ;;
                rviz|none)
                    CONFIG_WITH_UI_OVERRIDE="false"
                    CONFIG_WITH_2DUI_OVERRIDE="false"
                    ;;
            esac
            ;;
        nav-live)
            if [[ "${VIS_MODE}" != "rviz" ]]; then
                die "nav-live requires --viz rviz because navigation startup is gated by RViz /initialpose"
            fi
            ;;
    esac

    case "${MODE}" in
        loc-bag|loc-live)
            if [[ "${VIS_MODE}" == "rviz" && "${LOC_ENABLE_RVIZ}" -eq 0 ]]; then
                note "RViz mode is active but RViz launch is disabled; publish /initialpose from an external RViz or script"
            fi
            ;;
    esac
}

check_visualization_requirements() {
    case "${VIS_MODE}" in
        pangolin)
            if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
                die "Pangolin selected but DISPLAY/WAYLAND_DISPLAY is not set; run from a graphical terminal or use --viz rviz/none"
            fi
            ;;
    esac
}

configure_loc_bag_manual_init_playback() {
    local auto_start
    auto_start="$(yaml_section_value "${RUN_CONFIG}" system auto_start_from_identity)"

    case "${auto_start}" in
        false|False|FALSE|0)
            if ! bag_play_arg_present "--start-paused" && ! bag_play_arg_present "-p"; then
                BAG_PLAY_ARGS=(--start-paused "${BAG_PLAY_ARGS[@]}")
                note "loc-bag uses RViz initial pose; ros2 bag play will start paused"
            fi
            ;;
    esac
}

source_setup_files() {
    if [[ "${SOURCE_SETUP}" -eq 0 ]]; then
        return
    fi

    local ros_distro="${ROS_DISTRO:-humble}"
    set +u
    if [[ -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
        # shellcheck disable=SC1090
        source "/opt/ros/${ros_distro}/setup.bash"
    fi

    if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${WORKSPACE_ROOT}/install/setup.bash"
    fi

    # Source the repository overlay last so this script uses the Lightning-LM
    # binaries rebuilt in the current checkout instead of an older parent
    # workspace install.
    if [[ -f "${REPO_ROOT}/install/setup.bash" ]]; then
        # shellcheck disable=SC1091
        source "${REPO_ROOT}/install/setup.bash"
    fi
    set -u

    if [[ -d "${DEFAULT_PANGOLIN_PREFIX}/lib" ]]; then
        export LD_LIBRARY_PATH="${DEFAULT_PANGOLIN_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
    fi
}

run_ros2() {
    note "+ ros2 $*"
    set +e
    ros2 "$@"
    local rc=$?
    set -e
    return "${rc}"
}

run_node_foreground() {
    local executable="$1"
    shift
    run_ros2 run lightning "${executable}" "$@"
}

maybe_start_rviz() {
    case "${MODE}" in
        loc-bag|loc-live)
            ;;
        *)
            return 0
            ;;
    esac

    [[ "${LOC_ENABLE_RVIZ}" -eq 1 ]] || return 0

    if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
        note "DISPLAY/WAYLAND_DISPLAY is not set; skip RViz. Open another desktop terminal and run: rviz2 -d ${RVIZ_CONFIG}"
        return 0
    fi

    if [[ ! -f "${RVIZ_CONFIG}" ]]; then
        note "RViz config not found; skip RViz: ${RVIZ_CONFIG}"
        return 0
    fi

    note "+ rviz2 -d ${RVIZ_CONFIG}"
    start_background_logged /tmp/lightning-jt128-rviz.log rviz2 -d "${RVIZ_CONFIG}"
    RVIZ_PID="${STARTED_PID}"
    return 0
}

run_node_with_bag_play() {
    local executable="$1"
    shift

    maybe_start_rviz

    note "starting ${executable}"
    start_background ros2 run lightning "${executable}" "$@"
    APP_PID="${STARTED_PID}"

    sleep "${WAIT_SECONDS}"
    if ! kill -0 "${APP_PID}" 2>/dev/null; then
        set +e
        wait "${APP_PID}"
        local rc=$?
        set -e
        die "${executable} exited before bag playback, exit code ${rc}"
    fi

    note "+ ros2 bag play ${BAG_PATH} ${BAG_PLAY_ARGS[*]}"
    set +e
    ros2 bag play "${BAG_PATH}" "${BAG_PLAY_ARGS[@]}"
    local rc=$?
    set -e

    terminate_process_group "${APP_PID}"
    APP_PID=""

    return "${rc}"
}

run_nav_live() {
    local nav_args=()

    note "starting Lightning-LM localization; it will wait for RViz /initialpose"
    start_background ros2 run lightning run_loc_online --config "${RUN_CONFIG}"
    APP_PID="${STARTED_PID}"

    sleep "${WAIT_SECONDS}"
    if ! kill -0 "${APP_PID}" 2>/dev/null; then
        set +e
        wait "${APP_PID}"
        local rc=$?
        set -e
        die "run_loc_online exited before navigation startup, exit code ${rc}"
    fi

    nav_args+=(--external-localization --external-odom-timeout "${NAV_ODOM_TIMEOUT}")
    if [[ "${NAV_SKIP_BUILD}" -eq 1 ]]; then
        nav_args+=(--skip-build)
    fi
    if [[ "${NAV_ENABLE_RVIZ}" -eq 0 ]]; then
        nav_args+=(--no-rviz)
    else
        nav_args+=(--rviz-config "${RVIZ_CONFIG}")
    fi
    if [[ "${NAV_DRY_RUN}" -eq 1 ]]; then
        nav_args+=(--dry-run)
    fi
    if [[ "${NAV_ADAPTER_ENABLED_ON_START}" -eq 1 ]]; then
        nav_args+=(--adapter-enabled-on-start)
    fi
    if [[ "${NAV_BOUNDARY_DISABLED}" -eq 1 ]]; then
        nav_args+=(--no-boundary)
    elif [[ -n "${NAV_BOUNDARY_FILE}" ]]; then
        nav_args+=(--boundary-file "${NAV_BOUNDARY_FILE}")
    fi
    if [[ -n "${NAV_NETWORK_INTERFACE}" ]]; then
        nav_args+=(--network-interface "${NAV_NETWORK_INTERFACE}")
    fi
    nav_args+=(--unitree-backend "${NAV_UNITREE_BACKEND}")
    nav_args+=(--robot-model "${NAV_ROBOT_MODEL}")

    note "starting navigation after manual initialization gate"
    note "+ UNITREE_ADAPTER_PARAMS_FILE=${NAV_UNITREE_PARAMS_FILE} bash ${NAV_SCRIPT} ${nav_args[*]} ${EXTRA_ARGS[*]} ${PRIOR_MAP_PCD} ${RUN_NAME}"
    UNITREE_ADAPTER_PARAMS_FILE="${NAV_UNITREE_PARAMS_FILE}" \
        bash "${NAV_SCRIPT}" "${nav_args[@]}" "${EXTRA_ARGS[@]}" "${PRIOR_MAP_PCD}" "${RUN_NAME}"
    local rc=$?

    terminate_process_group "${APP_PID}"
    APP_PID=""

    return "${rc}"
}

normalize_mode() {
    case "$1" in
        lio-bag|slam-bag|bag-lio|bag-slam)
            MODE="lio-bag"
            ;;
        loc-bag|bag-loc)
            MODE="loc-bag"
            ;;
        lio-live|slam-live|live-lio|live-slam|lio-online|slam-online)
            MODE="lio-live"
            ;;
        loc-live|live-loc|loc-online)
            MODE="loc-live"
            ;;
        nav-live|live-nav|navigation-live|nav-online)
            MODE="nav-live"
            ;;
        lio-offline|slam-offline|offline-lio|offline-slam)
            MODE="lio-offline"
            ;;
        loc-offline|offline-loc)
            MODE="loc-offline"
            ;;
        convert-map|map-convert|pcd-to-map|convert-pcd)
            MODE="convert-map"
            ;;
        -h|--help|help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            die "unknown mode: $1"
            ;;
    esac
}

if [[ $# -eq 0 ]]; then
    usage
    exit 2
fi

normalize_mode "$1"
shift

while [[ $# -gt 0 ]]; do
    case "$1" in
        -b|--bag)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            BAG_PATH="$2"
            shift 2
            ;;
        --pcd)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            PCD_PATH="$2"
            shift 2
            ;;
        --prior-map-pcd|--prior-map)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            PRIOR_MAP_PCD="$2"
            shift 2
            ;;
        -c|--config)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            CONFIG_PATH="$2"
            shift 2
            ;;
        -m|--map)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            MAP_PATH="$2"
            shift 2
            ;;
        --wait)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            WAIT_SECONDS="$2"
            shift 2
            ;;
        --run-name)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            RUN_NAME="$2"
            shift 2
            ;;
        --nav-script)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_SCRIPT="$2"
            shift 2
            ;;
        --skip-build|--skip-nav-build)
            NAV_SKIP_BUILD=1
            shift
            ;;
        --viz|--visualization)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            VIS_MODE="$2"
            shift 2
            ;;
        --pangolin)
            VIS_MODE="pangolin"
            shift
            ;;
        --rviz)
            VIS_MODE="rviz"
            shift
            ;;
        --no-rviz)
            LOC_ENABLE_RVIZ=0
            NAV_ENABLE_RVIZ=0
            shift
            ;;
        --rviz-config)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            RVIZ_CONFIG="$2"
            shift 2
            ;;
        --dry-run|--no-unitree-adapter)
            NAV_DRY_RUN=1
            shift
            ;;
        --network-interface)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_NETWORK_INTERFACE="$2"
            shift 2
            ;;
        --unitree-backend)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_UNITREE_BACKEND="$2"
            shift 2
            ;;
        --robot-model)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_ROBOT_MODEL="$2"
            shift 2
            ;;
        --unitree-params-file)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_UNITREE_PARAMS_FILE="$2"
            shift 2
            ;;
        --adapter-enabled-on-start)
            NAV_ADAPTER_ENABLED_ON_START=1
            shift
            ;;
        --boundary-file)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_BOUNDARY_FILE="$2"
            shift 2
            ;;
        --no-boundary)
            NAV_BOUNDARY_DISABLED=1
            shift
            ;;
        --external-odom-timeout)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            NAV_ODOM_TIMEOUT="$2"
            shift 2
            ;;
        --bag-lidar-topic)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            BAG_LIDAR_TOPIC="$2"
            shift 2
            ;;
        --bag-imu-topic)
            [[ $# -ge 2 ]] || die "$1 requires a value"
            BAG_IMU_TOPIC="$2"
            shift 2
            ;;
        --overwrite)
            CONVERT_OVERWRITE=1
            shift
            ;;
        --no-auto-bag-topics)
            AUTO_BAG_TOPIC_OVERRIDE=0
            shift
            ;;
        --no-source)
            SOURCE_SETUP=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            case "${MODE}" in
                convert-map|nav-live)
                    EXTRA_ARGS+=("$@")
                    ;;
                *)
                    BAG_PLAY_ARGS+=("$@")
                    ;;
            esac
            break
            ;;
        *)
            die "unknown argument: $1"
            ;;
    esac
done

CONFIG_PATH="$(abs_path "${CONFIG_PATH}")"
RVIZ_CONFIG="$(abs_path "${RVIZ_CONFIG}")"
[[ -f "${CONFIG_PATH}" ]] || die "config file not found: ${CONFIG_PATH}"
normalize_visualization_mode
visualization_overrides

case "${MODE}" in
    loc-bag|loc-live|loc-offline|nav-live)
        if [[ -z "${MAP_PATH}" && -n "${PCD_PATH}" ]]; then
            MAP_PATH="${PCD_PATH}"
        fi
        ;;
esac

case "${MODE}" in
    lio-bag|loc-bag|lio-offline|loc-offline)
        [[ -n "${BAG_PATH}" ]] || die "${MODE} requires --bag"
        [[ -e "${BAG_PATH}" ]] || die "bag path not found: ${BAG_PATH}"
        BAG_PATH="$(abs_path "${BAG_PATH}")"
        ;;
    convert-map)
        [[ -n "${PCD_PATH}" ]] || die "convert-map requires --pcd"
        [[ -f "${PCD_PATH}" ]] || die "PCD file not found: ${PCD_PATH}"
        PCD_PATH="$(abs_path "${PCD_PATH}")"
        MAP_PATH="${MAP_PATH:-${DEFAULT_EXTERNAL_MAP}}"
        MAP_PATH="$(abs_path "${MAP_PATH}")"
        ;;
esac

RUN_CONFIG="${CONFIG_PATH}"
CONFIG_LIDAR_TOPIC_OVERRIDE=""
CONFIG_IMU_TOPIC_OVERRIDE=""
case "${MODE}" in
    lio-bag|loc-bag|lio-offline|loc-offline)
        detect_bag_topic_override
        ;;
esac

if [[ "${MODE}" == "convert-map" ]]; then
    :
else
case "${MODE}" in
    loc-bag|loc-live|loc-offline|nav-live)
        MAP_PATH="${MAP_PATH:-${DEFAULT_MAP}}"
        resolve_localization_map_input "${MAP_PATH}"
        note "localization map: ${MAP_PATH}"
        ;;
esac

if [[ "${MODE}" == "nav-live" ]]; then
    if [[ -z "${PRIOR_MAP_PCD}" && -n "${MAP_SOURCE_PCD}" ]]; then
        PRIOR_MAP_PCD="${MAP_SOURCE_PCD}"
    fi
    PRIOR_MAP_PCD="${PRIOR_MAP_PCD:-${MAP_PATH}/global.pcd}"
    PRIOR_MAP_PCD="$(abs_path "${PRIOR_MAP_PCD}")"
    NAV_SCRIPT="$(abs_path "${NAV_SCRIPT}")"
    NAV_UNITREE_PARAMS_FILE="$(abs_path "${NAV_UNITREE_PARAMS_FILE}")"

    [[ -f "${PRIOR_MAP_PCD}" ]] || die "prior map PCD not found: ${PRIOR_MAP_PCD}"
    [[ -f "${NAV_SCRIPT}" ]] || die "navigation script not found: ${NAV_SCRIPT}"
    [[ -f "${NAV_UNITREE_PARAMS_FILE}" ]] || die "Unitree params file not found: ${NAV_UNITREE_PARAMS_FILE}"
    if [[ -n "${NAV_BOUNDARY_FILE}" ]]; then
        NAV_BOUNDARY_FILE="$(abs_path "${NAV_BOUNDARY_FILE}")"
        [[ -f "${NAV_BOUNDARY_FILE}" ]] || die "boundary file not found: ${NAV_BOUNDARY_FILE}"
    fi
    RUN_NAME="${RUN_NAME:-jt128_lightning_nav_$(date +%Y%m%d_%H%M%S)}"
fi

if [[ -n "${MAP_PATH}" || -n "${CONFIG_LIDAR_TOPIC_OVERRIDE}" || -n "${CONFIG_IMU_TOPIC_OVERRIDE}" || \
      -n "${CONFIG_WITH_UI_OVERRIDE}" || -n "${CONFIG_WITH_2DUI_OVERRIDE}" || \
      -n "${CONFIG_ENABLE_RVIZ_OVERRIDE}" || -n "${CONFIG_AUTO_START_OVERRIDE}" ]]; then
    RUN_CONFIG="$(config_with_overrides "${CONFIG_PATH}" "${MAP_PATH}" "${CONFIG_LIDAR_TOPIC_OVERRIDE}" \
        "${CONFIG_IMU_TOPIC_OVERRIDE}" "${CONFIG_WITH_UI_OVERRIDE}" "${CONFIG_WITH_2DUI_OVERRIDE}" \
        "${CONFIG_ENABLE_RVIZ_OVERRIDE}" "${CONFIG_AUTO_START_OVERRIDE}")"
fi
fi

if [[ "${MODE}" == "loc-bag" ]]; then
    configure_loc_bag_manual_init_playback
fi

source_setup_files
check_visualization_requirements
command -v ros2 >/dev/null 2>&1 || die "ros2 not found; source ROS 2 or remove --no-source"
ros2 pkg prefix lightning >/dev/null 2>&1 || die "ROS package 'lightning' not found; build and source the workspace first"
auto_convert_map_if_needed

case "${MODE}" in
    lio-bag)
        run_node_with_bag_play run_slam_online --config "${RUN_CONFIG}"
        ;;
    loc-bag)
        run_node_with_bag_play run_loc_online --config "${RUN_CONFIG}"
        ;;
    lio-live)
        run_node_foreground run_slam_online --config "${RUN_CONFIG}"
        ;;
    loc-live)
        maybe_start_rviz

        note "starting run_loc_online"
        start_background ros2 run lightning run_loc_online --config "${RUN_CONFIG}"
        APP_PID="${STARTED_PID}"

        sleep "${WAIT_SECONDS}"
        if ! kill -0 "${APP_PID}" 2>/dev/null; then
            set +e
            wait "${APP_PID}"
            rc=$?
            set -e
            die "run_loc_online exited after startup, exit code ${rc}"
        fi

        wait "${APP_PID}"
        ;;
    nav-live)
        run_nav_live
        ;;
    lio-offline)
        run_node_foreground run_slam_offline --config "${RUN_CONFIG}" --input_bag "${BAG_PATH}"
        ;;
    loc-offline)
        run_node_foreground run_loc_offline --config "${RUN_CONFIG}" --map_path "${MAP_PATH}" --input_bag "${BAG_PATH}"
        ;;
    convert-map)
        convert_args=(--input_pcd "${PCD_PATH}" --output_map "${MAP_PATH}")
        if [[ "${CONVERT_OVERWRITE}" -eq 1 ]]; then
            convert_args+=(--overwrite)
        fi
        run_node_foreground convert_pcd_to_map "${convert_args[@]}" "${EXTRA_ARGS[@]}"
        ;;
esac
