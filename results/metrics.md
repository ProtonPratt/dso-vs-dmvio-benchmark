### TUM-VI, full motion-capture ground truth

Sim(3)-aligned, lever-arm-compensated ATE RMSE of the position.

| Sequence | Length | DM-VIO ATE | DSO ATE | DM-VIO coverage | DSO coverage |
| :-- | --: | --: | --: | --: | --: |
| `calib-cam1` | 109 s | 0.22 cm | 1.51 cm | 99 % | 97 % |
| `calib-cam5` | 94 s | 0.20 cm | 26.10 cm | 99 % | 98 % |
| `calib-cam8` | 50 s | 1.48 cm | 0.26 cm | 99 % | 92 % |
| `calib-imu2` | 53 s | 0.22 cm | 9.92 cm | 100 % | 97 % |

### Effect of the camera-IMU lever arm on the ATE

| Sequence | DM-VIO raw | DM-VIO compensated | DSO raw | DSO compensated |
| :-- | --: | --: | --: | --: |
| `calib-cam1` | 3.94 cm | 0.22 cm | 4.06 cm | 1.51 cm |
| `calib-cam5` | 4.05 cm | 0.20 cm | 23.51 cm | 26.10 cm |
| `calib-cam8` | 4.42 cm | 1.48 cm | 3.90 cm | 0.26 cm |
| `calib-imu2` | 3.02 cm | 0.22 cm | 8.70 cm | 9.92 cm |

### TUM-VI, ground truth at start and end only

| Sequence | Method | Scale drift | End error (start-aligned) | e_align | Coverage |
| :-- | :-- | --: | --: | --: | --: |
| `magistrale5` | DM-VIO | 1.02x | 1.41 m | 2.24 m | 100 % |
| `magistrale5` | DSO | 6.68x | 22.35 m | 57.43 m | 100 % |

### DM-VIO metric-scale output (calib-cam5 (run 2))

| Alignment | ATE RMSE | Residual scale error |
| :-- | --: | --: |
| SE(3), no scale correction | 1.56 cm | 4.5 % |
| Sim(3) | 0.32 cm | - |

### DSO on TUM monoVO (mid-term)

| Sequence | Length | Keyframes | Scale drift | e_align | Coverage |
| :-- | --: | --: | --: | --: | --: |
| `sequence_14` | 62 s | 502 | 1.040x | 0.72 m | 98 % |
| `sequence_40` | 174 s | 1164 | 1.088x | 1.44 m | 100 % |
