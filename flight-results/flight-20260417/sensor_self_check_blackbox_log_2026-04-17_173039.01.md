Sensor self-consistency audit — source=blackbox
  path: blackbox_log_2026-04-17_173039.01.csv
  samples: 9593, span: 161.5s, median dt: 17ms

Pass/fail policy: sign-inversion gate (slope sign wrong = FAIL).
Correlation 'r' reported but not gated.

  1. Position ↔ velocity integration: PASS
                 N  slope=+1.070  r=+0.970  n=9593
                 E  slope=+1.105  r=+0.991  n=9593
                 D  slope=+0.922  r=+0.989  n=9593
  2. Gyro ↔ quat-delta: PASS
                 p  slope=+1.005  r=+0.999  n=9592
                 q  slope=+0.985  r=+0.999  n=9592
                 r  slope=+1.000  r=+0.997  n=9592
  3. Euler(quat) ↔ attitude[]: SKIP (no euler or quat)
  4. Accel ↔ gravity (quasi-steady): PASS
                ax  slope=+0.781  r=+0.816  n=1234
                ay  slope=+0.094  r=+0.251  n=1234
                az  slope=+0.756  r=+0.290  n=1234
      filtered_samples: 1234
  5. Heading ↔ ground track: FAIL
               yaw  slope=-0.632  r=-0.671  n=8407
      filtered_samples: 8407
  6. Mag ↔ heading: PASS
      mean_offset_deg: -42.598749653588285
      stddev_deg: 63.28511152199289
      n: 9593
  7. Attitude vector ↔ velocity dir: FAIL
                 N  slope=+0.811  r=+0.904  n=9019
                 E  slope=-0.835  r=-0.931  n=9019
                 D  slope=-0.773  r=-0.910  n=9019
      filtered_samples: 9019
  8. Cmd ↔ attitude change (rate): SKIP (no cmd or quat)
