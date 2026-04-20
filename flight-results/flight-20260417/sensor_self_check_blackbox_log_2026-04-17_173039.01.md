Sensor self-consistency audit — source=blackbox
  path: blackbox_log_2026-04-17_173039.01.csv
  samples: 9593, span: 161.5s, median dt: 17ms

Pass/fail policy: sign-inversion gate (slope sign wrong = FAIL).
Correlation 'r' reported but not gated.

  1. Position ↔ velocity integration: PASS
                 N  slope=+1.070  r=+0.970  n=9593
                 E  slope=+1.105  r=+0.991  n=9593
                 D  slope=+0.922  r=+0.989  n=9593
  2. Gyro ↔ quat-delta: FAIL
                 p  slope=-0.166  r=-0.202  n=9592
                 q  slope=-0.235  r=-0.237  n=9592
                 r  slope=+1.053  r=+0.599  n=9592
  3. Euler(quat) ↔ attitude[]: SKIP (no euler or quat)
  4. Accel ↔ gravity (quasi-steady): FAIL
                ax  slope=-4.825  r=-0.709  n=1234
                ay  slope=-0.250  r=-0.082  n=1234
                az  slope=+6.055  r=+0.290  n=1234
      filtered_samples: 1234
  5. Heading ↔ ground track: PASS
               yaw  slope=+1.165  r=+0.808  n=8407
      filtered_samples: 8407
  6. Mag ↔ heading: PASS
      mean_offset_deg: -39.92105374724732
      stddev_deg: 98.75310629686761
      n: 9593
  7. Attitude vector ↔ velocity dir: PASS
                 N  slope=+0.811  r=+0.904  n=9019
                 E  slope=+0.608  r=+0.735  n=9019
                 D  slope=+0.076  r=+0.083  n=9019
      filtered_samples: 9019
  8. Cmd ↔ attitude change (rate): SKIP (no cmd or quat)
