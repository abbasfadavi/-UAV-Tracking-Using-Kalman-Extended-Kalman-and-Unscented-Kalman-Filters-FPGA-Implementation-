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

## 🧠 Future Work
1. **Unified Comparative Framework**  
   Create a single MATLAB and HLS environment that runs all three filters on the same dataset, enabling direct performance comparison (accuracy, RMSE, latency).

2. **Performance and Resource Reports**  
   Generate and publish Vivado HLS reports for each filter, showing resource utilization (DSPs, LUTs, FFs, BRAM) and latency metrics on **XC7K410T-FFG900-2**.

---

## 📈 Example Output
Example comparison table (expected after unified framework integration):

| Filter | RMSE (Position) | RMSE (Velocity) | Latency (cycles) | DSPs Used |
|--------|----------------|----------------|------------------|------------|
| KF | 0.35 | 0.28 | 14,000 | 210 |
| EKF | 0.29 | 0.25 | 18,500 | 280 |
| UKF | 0.20 | 0.19 | 22,000 | 310 |

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
