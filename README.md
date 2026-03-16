## Line-Following Robot Digital Twin (Siemens VSI)

This repository contains a complete **line-following robot digital twin** implemented on the **Siemens Innexis Virtual System Interconnect (VSI)** platform.  
The system models a differential-drive robot that tracks a path using a PID controller, with all components connected via a virtual CAN bus.

### 1. Architecture Overview

- **FabricServer (SystemC)**: Central VSI coordinator that advances simulation time and routes CAN frames.
- **Client 0 – Simulator (Plant + Environment)**:
  - Differential-drive robot kinematics (`simulator.py`).
  - Path generation (straight line and cubic Bézier curves).
  - Publishes robot pose and nearest path reference over CAN.
- **Client 1 – PID Controller**:
  - Computes lateral and heading errors from robot pose vs. reference.
  - PID steering law using *derivative on measurement* (heading error as D-term).
  - Sends `v_cmd` and `omega_cmd` over CAN.
- **Client 2 – Visualizer / Logger**:
  - Passive observer of all CAN signals.
  - Real-time 2D view (pygame) + PyQtGraph plots.
  - Logs CSV data, computes KPIs, and saves figures.

The CAN routing and simulation parameters are defined in `vsiBuildCommands` and generated into the VSI project (`src/lineFollowerDT`).

### 2. Repository Layout

- `vsiBuildCommands` – VSI TCL configuration (components, signals, frames, timing).
- `src/lineFollowerDT/`
  - `Makefile`, `Makefile.client0/1/2` – build and run targets for each client.
  - `src/simulator/simulator.py` – plant + environment client (Client 0).
  - `src/controller/controller.py` – PID controller client (Client 1).
  - `src/visualizer/visualizer.py` – visualizer/logger client (Client 2).
  - `src/visualizer/pygameVisualizer.py` – top‑down 2D visualization.
  - `src/visualizer/realTimePlotter.py` – PyQtGraph real‑time dashboard.
- `results/`
  - `*_kpis.txt` – overshoot, settling time, steady‑state error for each run.
  - `*.csv` – logged time series for each experiment.
  - `figures/*.png` – trajectory, error, and command plots (per experiment).
- `docs/`
  - `report.pdf` – **final capstone report** (LaTeX, 18 pages).
  - Other intermediate build artifacts and helper scripts (ignored by git).
- `EXPERIMENTS_GUIDE.md` – step‑by‑step instructions to reproduce all mandatory experiments.

### 3. Prerequisites

On the VSI server (or local Linux machine with VSI installed):

- **Siemens VSI** 2025.x (or project‑compatible version) with examples.
- **Python 3** (used by all three clients).
- Python packages:
  - `numpy`
  - `matplotlib`
  - `pyqtgraph`
  - `pygame`

The exact versions can follow your standard VSI training environment.

### 4. Building the VSI Project

From the VSI examples root (e.g. `.../vsiTutorials/robot-line-follower-digital-twin`):

```bash
# 1) Generate the digital twin project from vsiBuildCommands
vsiBuild -f vsiBuildCommands

# 2) Build all server + client binaries
cd src/lineFollowerDT
make clean
make compile
make build
```

This produces the FabricServer executable and the three Python client launchers (via `Makefile.client0/1/2`).

### 5. Running a Single Simulation

From `src/lineFollowerDT`:

```bash
# Example: baseline straight‑line experiment (E1_C)
make sim
```

The `sim` target in `Makefile.client0/1/2` controls:

- Path type (straight vs curved),
- Noise levels,
- PID gains (`--kp`, `--ki`, `--kd`),
- Output tag used for logging (e.g. `E1_C`, `E2_curved`, `E3_HighNoise`, ...).

During a run:

- The **pygame** window shows a top‑down view of the robot and path.
- The **PyQtGraph** dashboard (if available) displays live time‑series.
- After the run, the visualizer writes:
  - `results/<tag>.csv`
  - `results/<tag>_kpis.txt`
  - `results/figures/<tag>_*.png`

### 6. Mandatory Experiments

All mandatory experiments from the capstone specification are documented in detail in `EXPERIMENTS_GUIDE.md`:

- **E1 – PID Gain Sweep**: E1_A, E1_B, E1_C, E1_D on a straight path.
- **E2 – Curved Path**: baseline gains on a cubic Bézier path.
- **E3 – Noise Rejection**: E3_LowNoise and E3_HighNoise.
- **E4 – PD vs PID**: structure ablation between PD and full PID.

For each experiment, the guide specifies:

- What to change in `Makefile.client0/1/2`,
- Which output tag to use,
- Expected high‑level behaviour.

### 7. Final Report

A comprehensive LaTeX report is available at `docs/report.pdf`.  
It includes:

- Cover page with Siemens branding and project details,
- VSI gateway architecture (with a dedicated diagram),
- Modeling equations (all math in LaTeX),
- Controller design and tuning rationale,
- Detailed description of each experiment,
- KPI tables and embedded plots from `results/figures/`,
- Conclusions summarising controller performance and digital‑twin fidelity.

### 8. Notes on Version Control

The repository’s `.gitignore` is configured so that:

- All intermediate files under `docs/` are ignored,
- **Only** `docs/report.pdf` is tracked for the report,
- Raw experimental results remain under `results/` for traceability.

This keeps the repository clean while preserving the key artefacts needed for assessment and future work.

