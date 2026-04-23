Sensor self-consistency audit — source=blackbox
  path: flight-results/flight-20260422/blackbox_log_2026-04-22_175203.01.csv
  samples: 7738, span: 130.3s, median dt: 17ms

Pass/fail policy: sign-inversion gate (slope sign wrong = FAIL).
Correlation 'r' reported but not gated.

  1. Position ↔ velocity integration: PASS
                 N  slope=+0.986  r=+0.926  n=7738
                 E  slope=+0.962  r=+0.997  n=7738
                 D  slope=+0.962  r=+0.987  n=7738
  2. Gyro ↔ quat-delta: PASS
                 p  slope=+1.004  r=+0.999  n=7737
                 q  slope=+0.990  r=+0.999  n=7737
                 r  slope=+1.003  r=+0.998  n=7737
  3. Euler(quat) ↔ attitude[]: SKIP (no euler or quat)
  4. Accel ↔ gravity (quasi-steady): PASS
                ax  slope=+0.359  r=+0.320  n=314
                ay  slope=+0.078  r=+0.129  n=314
                az  slope=+0.607  r=+0.159  n=314
      filtered_samples: 314
  5. Heading ↔ ground track: PASS
               yaw  slope=+0.285  r=+0.434  n=7121
      filtered_samples: 7121
  6. Mag ↔ heading: PASS
      mean_offset_deg: -17.220388976234748
      stddev_deg: 68.48473874656568
      n: 7738
  7. Attitude vector ↔ velocity dir: FAIL
                 N  slope=+0.861  r=+0.907  n=7296
                 E  slope=+0.669  r=+0.782  n=7296
                 D  slope=-0.346  r=-0.320  n=7296
      filtered_samples: 7296
  8. Cmd ↔ attitude change (rate): SKIP (no cmd or quat)
