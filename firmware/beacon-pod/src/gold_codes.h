// N=31 Gold preferred-pair codes. CODE0 matches the FPGA correlator (experiments/s3.v CODE0), MSB-first:
// emitted chip i = bit (N-1-i), i.e. the i-th character of the bit string left-to-right — bit-identical to the
// FPGA emitter `codeA = CODE0[N-1-ech]`. So the correlator locks on this emitter unchanged.
#pragma once
#include <stdint.h>
#define GOLD_N      31
extern const uint32_t GOLD_CODE0;       // bit 30 = chip 0
