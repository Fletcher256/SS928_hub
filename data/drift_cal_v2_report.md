# IMU Drift Analysis Report

**Source:** `data/drift_10min.csv`  
**Generated:** 2026-06-12 22:13:28  
**Samples:** 2999 | **Duration:** 299.7s (5.0 min) | **Rate:** 10.0 Hz

---

## 1. Gyroscope Bias

| Parameter | Value | Unit |
|---|---|---|
| GyroZ bias (raw) | -0.359 | LSB |
| GyroZ noise (1σ) | 0.778 | LSB |
| GyroZ bias | -0.021939 | °/s |
| GyroZ noise | 0.047476 | °/s |
| GyroX bias | -0.163 | LSB |
| GyroY bias | +0.426 | LSB |
| Spec ZRO limit | ±1.0 | °/s |
| **ZRO margin** | **46x** | below spec |

## 2. Yaw Drift

| Parameter | Value | Unit |
|---|---|---|
| CF total drift | -6.7710 | ° |
| Kal total drift | -6.7700 | ° |
| CF drift rate | -0.022591 | °/s |
| Drift per minute | -1.355 | °/min |
| Kalman suppression | 0% | — |

## 3. Temperature

| Parameter | Value | Unit |
|---|---|---|
| Min | 26.70 | °C |
| Max | 26.93 | °C |
| Mean | 26.81 | °C |
| ΔT | 0.23 | °C |
| GyroZ-Temp r | +0.0038 | Pearson |
| Spec TCO | ±0.015 | °/s/K |

## 4. Allan Variance

| Parameter | Value | Unit |
|---|---|---|
| ARW | 0.6801 | °/√h |
| BI | 5.4459 | °/h |
| BI τ | 51.2 | s |
| ADEV @ 1s | 0.011336 | °/s |

## 5. Per-Minute Drift

| Min | Samples | Drift (°) | Rate (°/s) | Temp (°C) |
|---|---|---|---|---|
| 1 | 601 | -1.3310 | -0.022183 | 26.86 |
| 2 | 600 | -1.3980 | -0.023300 | 26.82 |
| 3 | 600 | -1.3670 | -0.022783 | 26.78 |
| 4 | 600 | -1.3510 | -0.022517 | 26.78 |

## 6. Recommendations

- △ Drift 1.355°/min — static bias subtraction recommended
