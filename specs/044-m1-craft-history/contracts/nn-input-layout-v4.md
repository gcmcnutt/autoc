# Contract — NN input layout v4 (044)

Consumers: autoc gather (sim), xiao gather (generated program), nn2cpp, dmp-dump, data.dat writer, renderer, flight-log writer and
both decoders (`src/analytics/flightlog_decode.py`, `xiao/web/flight_logger.html`), input mask, analytics.

1. Slot order is the `PathgenInput` / `TrackerInput` enum order; the shared craft tail is the 28 slots listed in data-model.md, last
   in both vectors (M1 index 25–52, M2 46–73).
2. History slots: for each of gyro p, q, r and accel z, three consecutive slots `_TM2` (100 ms), `_TM1` (50 ms), current. Display
   names `gyrP-2, gyrP-1, gyrP`, … `acZ-2, acZ-1, acZ`.
3. Scaling per signal: gyro ÷ 6 rad/s, accel ÷ 8 g — identical for history and current.
4. Pre-fill: after reset/engage every history slot equals the current reading.
5. Sim and xiao produce identical vectors from identical state (test).
6. Layout marker v4; readers of v3 artifacts fail loud.
