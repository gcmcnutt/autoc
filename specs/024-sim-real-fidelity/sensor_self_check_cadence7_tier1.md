Sensor self-consistency audit — source=sim
  path: eval-results/2026-04-21T16:24:41Z/tier1-aeroStandard/data.dat
  samples: 44202, span: 23.7s, median dt: 100ms

Pass/fail policy: sign-inversion gate (slope sign wrong = FAIL).
Correlation 'r' reported but not gated.

  1. Position ↔ velocity integration: FAIL
                 N  slope=+1.349  r=+0.003  n=44202
                 E  slope=-20.115  r=-0.312  n=44202
                 D  slope=+19.450  r=+0.239  n=44202
  2. Gyro ↔ quat-delta: PASS
                 p  slope=+1.175  r=+0.925  n=44201
                 q  slope=+0.870  r=+0.917  n=44201
                 r  slope=+1.024  r=+0.994  n=44201
  3. Euler(quat) ↔ attitude[]: SKIP (no euler or quat)
  4. Accel ↔ gravity (quasi-steady): SKIP (no accel or quat)
  5. Heading ↔ ground track: PASS
               yaw  slope=+0.926  r=+0.998  n=43931
      filtered_samples: 43931
  6. Mag ↔ heading: SKIP (no mag or quat)
  7. Attitude vector ↔ velocity dir: PASS
                 N  slope=+0.932  r=+0.974  n=44202
                 E  slope=+0.939  r=+0.946  n=44202
                 D  slope=+0.935  r=+0.891  n=44202
      filtered_samples: 44202
  8. Cmd ↔ attitude change (rate): PASS
          pitch->q  slope=+0.091  r=+0.046  n=44201
           roll->p  slope=+0.006  r=+0.002  n=44201
