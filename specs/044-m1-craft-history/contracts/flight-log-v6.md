# Contract — xiao flight log format v6 (044)

1. `FileHeader.format_version` = 6. Decoders accept exactly 6 for the new layout; a v5 file still decodes with the v5 path; a decoder
   that knows only ≤ 5 refuses a v6 file with a version error (Constitution V).
2. The input block is the whole v4 input vector (53 slots), in enum order; scale entries are indexed by enum NAME (8 new entries for the
   history slots, equal to their signal's scale); the scale table travels in the header, CRC-guarded.
2a. The header also carries the **ordered slot-name table** and the **layout signature**; decoders take column names and the input
   count from the header (no hard-coded list or count) and refuse a signature they cannot verify against the table.
3. Outputs and the v2 telemetry triples follow the input block unchanged.
4. Round-trip test: write a bench log with known values, decode with both decoders, compare all 53 inputs.
