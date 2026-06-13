# IMU Drift Analysis Report

**Source:** `data/drift_10min.csv`  
**Generated:** 2026-06-12 20:18:38  
**Samples:** 6000 | **Duration:** 599.9s (10.0 min) | **Rate:** 10.0 Hz

---

## 1. Gyroscope Bias

| Parameter | Value | Unit |
|---|---|---|
| GyroZ bias (raw) | +0.116 | LSB |
| GyroZ noise (1σ) | 0.870 | LSB |
| GyroZ bias | +0.007100 | °/s |
| GyroZ noise | 0.053085 | °/s |
| GyroX bias | -0.281 | LSB |
| GyroY bias | +0.801 | LSB |
| Spec ZRO limit | ±1.0 | °/s |
| **ZRO margin** | **141x** | below spec |

## 2. Yaw Drift

| Parameter | Value | Unit |
|---|---|---|
| CF total drift | +4.0880 | ° |
| Kal total drift | +4.0800 | ° |
| CF drift rate | +0.006815 | °/s |
| Drift per minute | +0.409 | °/min |
| Kalman suppression | 0% | — |

## 3. Temperature

| Parameter | Value | Unit |
|---|---|---|
| Min | 25.74 | °C |
| Max | 26.20 | °C |
| Mean | 26.04 | °C |
| ΔT | 0.46 | °C |
| GyroZ-Temp r | -0.0123 | Pearson |
| Spec TCO | ±0.015 | °/s/K |

## 4. Allan Variance

| Parameter | Value | Unit |
|---|---|---|
| ARW | 0.7576 | °/√h |
| BI | 4.5241 | °/h |
| BI τ | 102.4 | s |
| ADEV @ 1s | 0.012626 | °/s |

## 5. Per-Minute Drift

| Min | Samples | Drift (°) | Rate (°/s) | Temp (°C) |
|---|---|---|---|---|
| 1 | 600 | +0.4240 | +0.007067 | 26.14 |
| 2 | 600 | +0.3270 | +0.005450 | 26.08 |
| 3 | 601 | +0.3990 | +0.006650 | 26.12 |
| 4 | 600 | +0.3990 | +0.006650 | 26.13 |
| 5 | 599 | +0.2960 | +0.004933 | 26.11 |
| 6 | 600 | +0.4180 | +0.006967 | 26.05 |
| 7 | 600 | +0.4360 | +0.007267 | 26.01 |
| 8 | 601 | +0.3890 | +0.006483 | 25.97 |
| 9 | 600 | +0.5330 | +0.008883 | 25.97 |

## 6. Recommendations

- ✅ Drift 0.409°/min — no urgent action needed
