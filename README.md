# 🛰️ UAV Tracking Using Kalman, Extended Kalman, and Unscented Kalman Filters (FPGA Implementation)

## 📘 Overview
This repository contains three FPGA-ready implementations of state estimation filters — **Kalman Filter (KF)**, **Extended Kalman Filter (EKF)**, and **Unscented Kalman Filter (UKF)** — designed for **3D UAV tracking in XYZ space**.  
Each filter uses a MATLAB-generated trajectory and has its own trajectory generator, HLS implementation, and testbench.

---

## 📂 Repository Structure

| Directory | Filter | Description |
|------------|---------|-------------|
| `NAV_KF` | Kalman Filter | Linear KF implementation and trajectory |
| `NAV_EKF` | Extended Kalman Filter | Nonlinear EKF with process model and trajectory |
| `NAV_UKF` | Unscented Kalman Filter | UKF version using sigma points and covariance propagation |

Each filter directory contains:
- `trajectory_*.m` – MATLAB trajectory and measurement generator  
- `kalman_filter_*.cpp` – HLS C++ implementation  
- `kalman_filter_*.h` – Header definitions  
- `kalman_filter_*_tb.cpp` – HLS testbench for verification  

All filters are implemented in **single-precision floating point (`float`)** and are **Vivado HLS compatible**.

---

## ⚙️ FPGA Target
| Device | Family | Package | Speed |
|---------|---------|----------|--------|
| XC7K410T-2FFG900 | Xilinx Kintex-7 | FFG900 | -2 |

---

## 🧩 Project Highlights
- Fully synthesizable **HLS C++** implementations  
- MATLAB trajectories provide identical input for all three filters  
- Direct comparison between KF, EKF, and UKF outputs  
- Easily extensible for new sensor models or motion dynamics  

---

### FPGA Resource Utilization and RMSE Comparison

| Filter | Latency | BRAM | DSP | FF  | LUT  | RMSE (Position) | RMSE (Velocity) |
|:-------|:--------|:-----|:----|:----|:----:|:----------------|:----------------|
| **KF**  | 2 µs  | 5 | 146 | 11k | 15k | **0.59** | 1.34 |
| **EKF** | 14 ms | 8 | 89  | 20k | 20k | 1.18 | 0.98 |
| **UKF** | 17 ms | 5 | 39  | 7k  | 10k | 0.70 | **0.86** |

🛰️ Comparison of Kalman Filter, Extended Kalman Filter, and Unscented Kalman Filter

This project compares three filtering algorithms — Kalman Filter (KF), Extended Kalman Filter (EKF), and Unscented Kalman Filter (UKF) — using the same trajectory data.
The goal is to evaluate accuracy, stability, and robustness in the presence of nonlinear motion and measurement noise.


![compare](https://github.com/user-attachments/assets/ce1be26c-e6da-490e-8a94-a53015c8610a)
🔹 Simulation Results

Below is the comparison between the three filters when applied to the same target trajectory:

🔹 Interpretation

KF (Red) → Performs well when the system is nearly linear. It maintains a stable and accurate estimate.

EKF (Blue) → Shows divergence and high error accumulation. This happens because EKF linearizes the nonlinear model using Jacobians, which can introduce significant approximation errors if the motion model is strongly nonlinear or the initial estimate is poor.

UKF (Green) → Produces the best overall performance. By using sigma points to capture the true mean and covariance, UKF handles nonlinearities much better and remains stable even under noisy or curved trajectories.

### 3D Trajectory Comparison
The figure below shows the real trajectory (black) and the estimated ones by UKF.
![compare1](https://github.com/user-attachments/assets/ea3e79f3-2977-4a0b-aebb-e774b005cf6d)



> **Notes:**
> - All implementations were tested on **Xilinx Kintex-7 XC7K410T-FFG900-2**.
> - Latency is measured after C/RTL co-simulation.
> - RMSE is computed by comparing MATLAB simulation results with HLS outputs.


---

## 🧰 Tools and Environment
- **Vivado HLS 2020.1 or newer**
- **MATLAB R2023a+** (for trajectory generation)
- Tested on: **Kintex-7 XC7K410T-FFG900-2**

---

## 👤 Author
**Abbas Fadavi**  
FPGA Engineer — Specializing in signal processing, radar, and UAV tracking systems  
📍 Tehran  
