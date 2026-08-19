# Contract — raw recording container

**Consumers**: `src_replay.c`, `tools/oracle`, `tools/inject`, `tools/score`, any analysis script.
**Constitution V applies.**

## Header

```
offset  size  field
0       4     magic            0x42434E52  ("BCNR")
4       2     format_version   starts at 1
6       2     header_bytes
8       2     width
10      2     height
12      2     bits_per_pixel   8 (10 reserved)
14      2     sensor_mode      enum: 640x400 stock, 640x200 patched, ...
16      4     nominal_fps
20      8     start_t_us
28      8     build_id
36      8     config_hash
44      4     mode             0=continuous 1=ring 2=burst
48      4     reserved
```

## Frame records

Length-prefixed, in capture order:

```
4   record_bytes
4   seq                monotonic; GAPS ARE EXPLICIT and legal in ring/burst mode
8   t_us
4   exposure_us
2   gain_q8
2   flags              bit0 = burst_start, bit1 = trigger_dump
-   payload            width*height*bpp/8
```

## Rules

- **Gaps are explicit, never implied.** In `ring` and `burst` modes `seq` jumps; a reader MUST treat a jump
  as a discontinuity and MUST NOT correlate across it. This is the one thing that makes decimated/burst
  captures safe to hand to the same tools as continuous ones.
- **`burst_start` marks a correlatable boundary.** Each burst is ≥1 full word of contiguous frames, so tools
  may correlate *within* a burst and never across.
- Writers preallocate, use `O_DIRECT` and large aligned writes (R9) so IO jitter stays off the §11.1
  deadline.
- Readers fail loudly on `magic`/`format_version` mismatch, naming both versions.
