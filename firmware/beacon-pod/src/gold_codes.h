// N=31 Gold preferred-pair codes, matching the FPGA correlator (experiments/s3.v CODE0/CODE1), MSB-first:
// emitted chip i = bit (N-1-i) = the i-th character of the bit string left-to-right — bit-identical to the FPGA
// emitter, so the correlator's matching channel locks on this emitter unchanged.
#pragma once
#include <stdint.h>
#define GOLD_N        31
#define GOLD_NCODES   2
extern const uint32_t GOLD_CODE[GOLD_NCODES];   // [0]=CODE0 (A), [1]=CODE1 (B); bit 30 = chip 0
