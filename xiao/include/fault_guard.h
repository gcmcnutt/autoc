#pragma once

// 039 bench hardening (post 2026-07-12 alignment-fault incident): watchdog +
// fault breadcrumb. A HardFault used to leave the board playing dead —
// interrupts masked in mbed's spin handler, USB and LED gone — until a power
// cycle, with no trace of why. Two guarantees now:
//
//  1. WATCHDOG: the nRF52840 WDT reboots a wedged/faulted firmware within
//     kWdtTimeoutS. Armed at the END of setup() — i.e., sequenced AFTER the
//     8 s console-reconnect delay, which must stay ahead of it — fed once per
//     loop() and inside the long QSPI erase waits. CONFIG.HALT=0 so the dog
//     pauses while a debugger has the core halted. nRF52 gotcha: once started
//     the WDT keeps counting through soft reset (incl. the UF2 bootloader
//     during DFU — the Adafruit-lineage bootloader feeds a running WDT).
//
//  2. BREADCRUMB: fault vectors are re-pointed (RAM vector table — mbed owns
//     the flash handlers) at a capture handler that stores pc/lr/cfsr/hfsr/
//     bfar + IPSR into .noinit RAM (survives soft reset) and SystemResets.
//     faultGuardReport() prints it on the next boot, after the console is up.

#include <stdint.h>

// Watchdog period. Control loop is 50 ms; worst legitimate stall between
// feeds is a 64 KB QSPI block erase (~2 s spec max), fed per-operation.
constexpr uint32_t kWdtTimeoutS = 4;

// FIRST line of setup(): install capture vectors, snapshot + clear the
// .noinit fault record and POWER->RESETREAS. Must run before anything that
// can fault; does not print (console isn't up yet).
void faultGuardInstall();

// After the console is usable: print why the previous run rebooted, if bad.
void faultGuardReport();

// END of setup(): start the WDT (unstoppable from then on).
void faultGuardArmWatchdog();

// Feed. Called once per loop() and from inside long blocking flash ops.
void faultGuardFeed();
