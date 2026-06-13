# IMU Drift Analysis Report

**Source:** `data/drift_2.5min_20260612_231647.csv`  
**Generated:** 2026-06-12 23:23:37  
**Samples:** 1503 | **Duration:** 150.2s (2.5 min) | **Rate:** 10.0 Hz

---

## 1. Gyroscope Bias

| Parameter | Value | Unit |
|---|---|---|
| GyroZ bias (raw) | -0.152 | LSB |
| GyroZ noise (1σ) | 1.325 | LSB |
| GyroZ bias | -0.009259 | °/s |
| GyroZ noise | 0.080882 | °/s |
| GyroX bias | +0.202 | LSB |
| GyroY bias | +0.496 | LSB |
| Spec ZRO limit | ±1.0 | °/s |
| **ZRO margin** | **108x** | below spec |

## 2. Yaw Drift

| Parameter | Value | Unit |
|---|---|---|
| CF total drift | -1.2160 | ° |
| Kal total drift | -1.2100 | ° |
| CF drift rate | -0.008097 | °/s |
| Drift per minute | -0.486 | °/min |
| Kalman suppression | 0% | — |

## 3. Temperature

| Parameter | Value | Unit |
|---|---|---|
| Min | 25.94 | °C |
| Max | 26.09 | °C |
| Mean | 26.01 | °C |
| ΔT | 0.15 | °C |
| GyroZ-Temp r | -0.0137 | Pearson |
| Spec TCO | ±0.015 | °/s/K |

## 4. Allan Variance

| Parameter | Value | Unit |
|---|---|---|
| ARW | 1.2232 | °/√h |
| BI | 19.8925 | °/h |
| BI τ | 25.6 | s |
| ADEV @ 1s | 0.020387 | °/s |

## 5. Per-Minute Drift

| Min | Samples | Drift (°) | Rate (°/s) | Temp (°C) |
|---|---|---|---|---|
| 1 | 600 | -0.4360 | -0.007267 | 26.01 |
| 2 | 600 | -0.5030 | -0.008383 | 26.00 |

## 6. Recommendations

- ✅ Drift 0.486°/min — no urgent action needed
