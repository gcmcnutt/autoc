#include "fault_guard.h"

#include <string.h>

#include "main.h"
#include "nrf.h"

// ---------------------------------------------------------------------------
// .noinit fault record — survives NVIC_SystemReset (nRF52840 RAM is retained
// through soft/pin/watchdog reset; only POR/brownout scrambles it, and the
// checksum rejects scrambled garbage).
// ---------------------------------------------------------------------------

namespace {

constexpr uint32_t kFaultMagic = 0xFA173CA9u;

struct FaultRecord {
  uint32_t magic;
  uint32_t pc, lr, xpsr;             // from the stacked exception frame
  uint32_t cfsr, hfsr, bfar, mmfar;  // SCB fault status at capture
  uint32_t ipsr;                     // which fault vector ran
  uint32_t sp;                       // frame address (stack pointer at fault)
  uint32_t check;                    // ~(sum of all words above)
};

uint32_t recordSum(const FaultRecord& r) {
  return r.magic + r.pc + r.lr + r.xpsr + r.cfsr + r.hfsr + r.bfar + r.mmfar +
         r.ipsr + r.sp;
}

__attribute__((section(".noinit"))) FaultRecord s_faultRec;

// RAM vector table (nRF52840: 16 system + 48 peripheral = 64 vectors).
__attribute__((aligned(256))) uint32_t s_ramVectors[64];

// Snapshot of the previous run's evidence, taken once at install.
FaultRecord s_prevFault;  // valid iff s_prevFaultValid
bool s_prevFaultValid = false;
uint32_t s_resetReas = 0;

}  // namespace

// ---------------------------------------------------------------------------
// Capture handler
// ---------------------------------------------------------------------------

extern "C" void faultGuardCaptureC(uint32_t* frame, uint32_t exc_return) {
  (void)exc_return;
  s_faultRec.magic = kFaultMagic;
  // Only dereference the stacked frame if it plausibly sits in SRAM — a
  // corrupted SP must not double-fault us into a lockup.
  const uint32_t f = reinterpret_cast<uint32_t>(frame);
  if (f >= 0x20000000u && f <= 0x2003FFC0u) {
    s_faultRec.pc = frame[6];
    s_faultRec.lr = frame[5];
    s_faultRec.xpsr = frame[7];
  } else {
    s_faultRec.pc = s_faultRec.lr = s_faultRec.xpsr = 0xDEADBEEFu;
  }
  s_faultRec.cfsr = SCB->CFSR;
  s_faultRec.hfsr = SCB->HFSR;
  s_faultRec.bfar = SCB->BFAR;
  s_faultRec.mmfar = SCB->MMFAR;
  s_faultRec.ipsr = __get_IPSR();
  s_faultRec.sp = f;
  s_faultRec.check = ~recordSum(s_faultRec);
  NVIC_SystemReset();
}

extern "C" __attribute__((naked)) void faultGuardHandler(void) {
  __asm volatile(
      "tst lr, #4        \n"  // EXC_RETURN bit2: which SP held the frame
      "ite eq            \n"
      "mrseq r0, msp     \n"
      "mrsne r0, psp     \n"
      "mov r1, lr        \n"
      "b faultGuardCaptureC\n");
}

// ---------------------------------------------------------------------------
// Install / report / watchdog
// ---------------------------------------------------------------------------

void faultGuardInstall() {
  // Snapshot + clear the reset reason (cumulative register, W1C).
  s_resetReas = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = s_resetReas;

  // Snapshot + invalidate the previous run's fault record.
  if (s_faultRec.magic == kFaultMagic && s_faultRec.check == ~recordSum(s_faultRec)) {
    s_prevFault = s_faultRec;
    s_prevFaultValid = true;
  }
  s_faultRec.magic = 0;
  s_faultRec.check = 0;

  // Re-point the fault vectors at the capture handler. mbed's table lives in
  // flash; copy it to RAM once and retarget VTOR (patch in place if some
  // layer already moved it to RAM).
  uint32_t* activeTable = reinterpret_cast<uint32_t*>(SCB->VTOR);
  const uint32_t handler = reinterpret_cast<uint32_t>(&faultGuardHandler);
  if (SCB->VTOR < 0x20000000u) {
    memcpy(s_ramVectors, activeTable, sizeof(s_ramVectors));
    s_ramVectors[3] = handler;  // HardFault
    s_ramVectors[4] = handler;  // MemManage
    s_ramVectors[5] = handler;  // BusFault
    s_ramVectors[6] = handler;  // UsageFault
    __DSB();
    SCB->VTOR = reinterpret_cast<uint32_t>(s_ramVectors);
  } else {
    activeTable[3] = activeTable[4] = activeTable[5] = activeTable[6] = handler;
    __DSB();
  }
  __ISB();
}

void faultGuardReport() {
  if (s_prevFaultValid) {
    logPrint(ERROR,
             "*** PREVIOUS RUN CRASHED: ipsr=%lu pc=%08lx lr=%08lx cfsr=%08lx "
             "hfsr=%08lx bfar=%08lx sp=%08lx ***",
             (unsigned long)s_prevFault.ipsr, (unsigned long)s_prevFault.pc,
             (unsigned long)s_prevFault.lr, (unsigned long)s_prevFault.cfsr,
             (unsigned long)s_prevFault.hfsr, (unsigned long)s_prevFault.bfar,
             (unsigned long)s_prevFault.sp);
  }
  if (s_resetReas & POWER_RESETREAS_DOG_Msk) {
    logPrint(ERROR, "*** WATCHDOG REBOOT (previous run wedged) resetreas=%08lx ***",
             (unsigned long)s_resetReas);
  } else if (s_resetReas != 0) {
    logPrint(INFO, "Reset reason: %08lx", (unsigned long)s_resetReas);
  }
  logPrint(INFO, "Fault guard: WDT %lus arms at end of setup", (unsigned long)kWdtTimeoutS);
}

void faultGuardArmWatchdog() {
  // SLEEP=Run (counts while WFE/WFI), HALT=Pause (frozen under debugger halt).
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV = kWdtTimeoutS * 32768u - 1u;
  NRF_WDT->RREN = WDT_RREN_RR0_Msk;
  NRF_WDT->TASKS_START = 1;
}

void faultGuardFeed() {
  if (NRF_WDT->RUNSTATUS) {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload << WDT_RR_RR_Pos;
  }
}
