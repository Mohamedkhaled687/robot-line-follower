#!/usr/bin/env bash
#
# run_experiments.sh — Automated Mandatory Experiments (E1–E4)
#
# Patches Makefile.client0/1/2 sim-target command lines with
# experiment-specific arguments, runs `make sim`, and restores
# the original Makefiles after each run.
#
# Usage (from the project root, i.e. robot-line-follower/):
#   chmod +x run_experiments.sh
#   ./run_experiments.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/src/lineFollowerDT"

MK0="${BUILD_DIR}/Makefile.client0"
MK1="${BUILD_DIR}/Makefile.client1"
MK2="${BUILD_DIR}/Makefile.client2"

mkdir -p "${SCRIPT_DIR}/results/figures"

backup() {
    cp "$MK0" "${MK0}.bak"
    cp "$MK1" "${MK1}.bak"
    cp "$MK2" "${MK2}.bak"
}

restore() {
    mv -f "${MK0}.bak" "$MK0"
    mv -f "${MK1}.bak" "$MK1"
    mv -f "${MK2}.bak" "$MK2"
}

patch_and_run() {
    local SIM_ARGS="$1"
    local CTRL_ARGS="$2"
    local VIS_ARGS="$3"
    local TAG="$4"

    echo ""
    echo "============================================================"
    echo "  EXPERIMENT: ${TAG}"
    echo "============================================================"
    echo "  Simulator:   ${SIM_ARGS}"
    echo "  Controller:  ${CTRL_ARGS}"
    echo "  Visualizer:  ${VIS_ARGS}"
    echo "============================================================"
    echo ""

    backup

    sed -i 's|^\t*python3 \./\$(TB).*|'"$(printf '\t')python3 ./\$(TB) --domain=\"AF_UNIX\" --server-url=\"localhost\" ${SIM_ARGS}|" "$MK0"
    sed -i 's|^\t*python3 \./\$(TB).*|'"$(printf '\t')python3 ./\$(TB) --domain=\"AF_UNIX\" --server-url=\"localhost\" ${CTRL_ARGS}|" "$MK1"
    sed -i 's|^\t*python3 \./\$(TB).*|'"$(printf '\t')python3 ./\$(TB) --domain=\"AF_UNIX\" --server-url=\"localhost\" ${VIS_ARGS}|" "$MK2"

    (cd "$BUILD_DIR" && make sim) || true

    restore

    echo ""
    echo "[DONE] Experiment ${TAG} complete."
    echo ""
}

# ===================================================================
#  E1 — PID Gain Sweep (straight path, spawn offset 0.3)
# ===================================================================
SIM_E1='--path-type="straight" --spawn-offset=0.3 --noise-x=0.0 --noise-y=0.0 --noise-theta=0.0'

patch_and_run \
    "$SIM_E1" \
    "--kp=1.0 --ki=0.0 --kd=2.0 --v-const=0.5" \
    '--output-tag="E1_A"' \
    "E1_A  (kp=1.0 ki=0.0 kd=2.0)"

patch_and_run \
    "$SIM_E1" \
    "--kp=2.0 --ki=0.0 --kd=3.0 --v-const=0.5" \
    '--output-tag="E1_B"' \
    "E1_B  (kp=2.0 ki=0.0 kd=3.0)"

patch_and_run \
    "$SIM_E1" \
    "--kp=2.0 --ki=0.1 --kd=3.0 --v-const=0.5" \
    '--output-tag="E1_C"' \
    "E1_C  (kp=2.0 ki=0.1 kd=3.0) — baseline"

patch_and_run \
    "$SIM_E1" \
    "--kp=3.0 --ki=0.1 --kd=4.0 --v-const=0.5" \
    '--output-tag="E1_D"' \
    "E1_D  (kp=3.0 ki=0.1 kd=4.0)"

patch_and_run \
    "$SIM_E1" \
    "--kp=4.0 --ki=0.2 --kd=5.0 --v-const=0.5" \
    '--output-tag="E1_E"' \
    "E1_E  (kp=4.0 ki=0.2 kd=5.0)"

# ===================================================================
#  E2 — Curved Path Robustness (best gains = set C)
# ===================================================================
patch_and_run \
    '--path-type="curved" --spawn-offset=0.3 --noise-x=0.0 --noise-y=0.0 --noise-theta=0.0' \
    "--kp=2.0 --ki=0.1 --kd=3.0 --v-const=0.5" \
    '--output-tag="E2_curved"' \
    "E2_curved  (curved path, best gains)"

# ===================================================================
#  E3 — Noise Rejection (straight path, best gains, 3 noise levels)
# ===================================================================
SIM_E3_BASE='--path-type="straight" --spawn-offset=0.3'
CTRL_E3="--kp=2.0 --ki=0.1 --kd=3.0 --v-const=0.5"

patch_and_run \
    "${SIM_E3_BASE} --noise-x=0.001 --noise-y=0.001 --noise-theta=0.005" \
    "$CTRL_E3" \
    '--output-tag="E3_low"' \
    "E3_low   (noise: x=0.001 y=0.001 theta=0.005)"

patch_and_run \
    "${SIM_E3_BASE} --noise-x=0.005 --noise-y=0.005 --noise-theta=0.01" \
    "$CTRL_E3" \
    '--output-tag="E3_med"' \
    "E3_med   (noise: x=0.005 y=0.005 theta=0.01)"

patch_and_run \
    "${SIM_E3_BASE} --noise-x=0.01 --noise-y=0.01 --noise-theta=0.02" \
    "$CTRL_E3" \
    '--output-tag="E3_high"' \
    "E3_high  (noise: x=0.01  y=0.01  theta=0.02)"

# ===================================================================
#  E4 — PD vs PID Ablation (curved path + medium noise)
# ===================================================================
SIM_E4='--path-type="curved" --spawn-offset=0.3 --noise-x=0.005 --noise-y=0.005 --noise-theta=0.01'

patch_and_run \
    "$SIM_E4" \
    "--kp=2.0 --ki=0.0 --kd=3.0 --v-const=0.5" \
    '--output-tag="E4_PD"' \
    "E4_PD   (PD: ki=0.0)"

patch_and_run \
    "$SIM_E4" \
    "--kp=2.0 --ki=0.1 --kd=3.0 --v-const=0.5" \
    '--output-tag="E4_PID"' \
    "E4_PID  (PID: ki=0.1)"

echo ""
echo "============================================================"
echo "  ALL EXPERIMENTS COMPLETE"
echo "  Results in: ${SCRIPT_DIR}/results/"
echo "  Figures in: ${SCRIPT_DIR}/results/figures/"
echo "============================================================"
