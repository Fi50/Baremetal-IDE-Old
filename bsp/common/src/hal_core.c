/**
 * @file hal_core.c
 * @author -T.K.- / t_k_233@outlook.com
 * @brief 
 * @version 0.1
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "hal_core.h"


// void HAL_CORE_disableGlobalInterrupt() {
//   // Clear MPIE
//   uint32_t mask = (1U << 3U);
//   asm volatile("csrc mstatus, %0" :: "r"(mask));
// }

// void HAL_CORE_enableGlobalInterrupt() {
//   // Set MPIE
//   uint32_t mask = (1U << 3U);
//   asm volatile("csrs mstatus, %0" :: "r"(mask));
// }

// void HAL_CORE_disableIRQ(IRQn_Type IRQn) {
//   uint32_t mask = (1U << (uint32_t)IRQn);
//   asm volatile("csrc mie, %0" :: "r"(mask));
// }

// void HAL_CORE_enableIRQ(IRQn_Type IRQn) {
//   uint32_t mask = (1U << (uint32_t)IRQn);
//   asm volatile("csrs mie, %0" :: "r"(mask));
// }

// void HAL_CORE_clearIRQ(IRQn_Type IRQn) {
//   uint32_t mask = (1U << (uint32_t)IRQn);
//   asm volatile("csrc mip, %0" :: "r"(mask));
// }

uint64_t HAL_getTick() {
  return HAL_CLINT_getTime(CLINT);
}

void HAL_delay(uint64_t time) {
  uint64_t target_tick = HAL_getTick() + (time * MTIME_FREQ);
  while (HAL_getTick() < target_tick) {
    // asm("nop");
  }
}


/* ==== Exception Handlers ==== */
__attribute__((weak)) void HAL_instructionAddressMisalignedExceptionHandler() {
  while (1) {
    printf("Exception: instruction address misaligned\n");
  }
}

__attribute__((weak)) void HAL_instructionAccessFaultExceptionHandler() {
  while (1) {
    printf("Exception: instruction access fault\n");
  }
}

__attribute__((weak)) void HAL_illegalInstructionExceptionHandler() {
  while (1) {
    printf("Exception: illegal instruction\n");
  }
}

__attribute__((weak)) void HAL_breakpointExceptionHandler() {
  while (1) {
    printf("Exception: breakpoint\n");
  }
}

__attribute__((weak)) void HAL_loadAddressMisalignedExceptionHandler() {
  while (1) {
    printf("Exception: load address misaligned\n");
  }
}

__attribute__((weak)) void HAL_loadAccessFaultExceptionHandler() {
  while (1) {
    printf("Exception: load access fault\n");
  }
}

__attribute__((weak)) void HAL_storeAMOAddressMisalignedExceptionHandler() {
  while (1) {
    printf("Exception: store/AMO address misaligned\n");
  }
}

__attribute__((weak)) void HAL_storeAMOAccessFaultExceptionHandler() {
  while (1) {
    printf("Exception: store/AMO access fault\n");
  }
}

__attribute__((weak)) void HAL_environmentCallUModeExceptionHandler() {
  // TODO: implement mode transfer
  while (1) {
    printf("Exception: environment call from U-mode\n");
  }
}

__attribute__((weak)) void HAL_environmentCallMModeExceptionHandler() {
  // TODO: implement mode transfer
  while (1) {
    printf("Exception: environment call from M-mode\n");
  }
}

__attribute__((weak)) void HAL_instructionPageFaultExceptionHandler() {
  while (1) {
    printf("Exception: instruction page fault\n");
  }
}

/* ==== Interrupt Callbacks ==== */
__attribute__((weak)) void HAL_userSoftwareInterruptCallback() {
  printf("user software irq\n");
  // HAL_CORE_clearIRQ(UserSoftware_IRQn);
}

__attribute__((weak)) void HAL_supervisorSoftwareInterruptCallback() {
  printf("supervisor software irq\n");
  // HAL_CORE_clearIRQ(SupervisorSoftware_IRQn);
}

__attribute__((weak)) void HAL_hypervisorSoftwareInterruptCallback() {
  printf("hypervisor software irq\n");
  // HAL_CORE_clearIRQ(HypervisorSoftware_IRQn);
}

__attribute__((weak)) void HAL_machineSoftwareInterruptCallback() {
  printf("machine software irq\n");
  uint32_t hartid = HAL_CORE_getHartId();
  HAL_CLINT_clearSoftwareInterrupt(CLINT, hartid);
  // HAL_CORE_clearIRQ(MachineSoftware_IRQn);
}

__attribute__((weak)) void HAL_userTimerInterruptCallback() {
  printf("user timer irq\n");
  // HAL_CORE_clearIRQ(UserTimer_IRQn);
}

__attribute__((weak)) void HAL_supervisorTimerInterruptCallback() {
  printf("supervisor timer irq\n");
  // HAL_CORE_clearIRQ(SupervisorTimer_IRQn);
}

__attribute__((weak)) void HAL_hypervisorTimerInterruptCallback() {
  printf("hypervisor timer irq\n");
  // HAL_CORE_clearIRQ(HypervisorTimer_IRQn);
}

__attribute__((weak)) void HAL_machineTimerInterruptCallback() {
  printf("machine timer irq\n");
  uint32_t hartid = HAL_CORE_getHartId();
  HAL_CLINT_setTimerInterruptTarget(CLINT, hartid, 0xFFFFFFFFFFFFFFFF);
}

__attribute__((weak)) void HAL_machineExternalInterruptCallback() {
  printf("machine external irq\n");
  // uint32_t irq_source = HAL_PLIC_claimIRQ(0);

  // { // debug message
//     char str[32];
//     sprintf(str, "machine external irq: %lu\n", irq_source);
//     HAL_UART_transmit(UART0, (uint8_t *)str, strlen(str), 0);
//   }
//   switch (irq_source) {
//     case 2:                 // GPIO interrupt
//       HAL_GPIO_Callback();
//       GPIOA->HIGH_IE = 0b0;
//       break;
//   }
  // HAL_PLIC_completeIRQ(0, irq_source);
  // HAL_CORE_clearIRQ(MachineExternal_IRQn);
}

void __init_tls(void) {
  register char *__thread_self __asm__ ("tp");
  extern char __tdata_start[];
  extern char __tbss_offset[];
  extern char __tdata_size[];
  extern char __tbss_size[];

  memcpy(__thread_self, __tdata_start, (size_t)__tdata_size);
  memset(__thread_self + (size_t)__tbss_offset, 0, (size_t)__tbss_size);
}

uintptr_t trap_handler(uintptr_t m_epc, uintptr_t m_cause, uintptr_t m_tval, uintptr_t regs[32]) {
  switch (m_cause) {
    case 0x00000000UL:      // instruction address misaligned
      HAL_instructionAddressMisalignedExceptionHandler();
      break;
    case 0x00000001UL:      // instruction access fault
      HAL_instructionAccessFaultExceptionHandler();
      break;
    case 0x00000002UL:      // illegal instruction
      HAL_illegalInstructionExceptionHandler();
      break;
    case 0x00000003UL:      // breakpoint
      HAL_breakpointExceptionHandler();
      break;
    case 0x00000004UL:      // load address misaligned
      HAL_loadAddressMisalignedExceptionHandler();
      break;
    case 0x00000005UL:      // load access fault
      HAL_loadAccessFaultExceptionHandler();
      break;
    case 0x00000006UL:      // store/AMO address misaligned
      HAL_storeAMOAddressMisalignedExceptionHandler();
      break;
    case 0x00000007UL:      // store/AMO access fault
      HAL_storeAMOAccessFaultExceptionHandler();
      break;
    case 0x00000008UL:      // environment call from U-mode
      HAL_environmentCallUModeExceptionHandler();
      break;
    case 0x0000000BUL:      // environment call from M-mode
      HAL_environmentCallMModeExceptionHandler();
      break;
    case 0x0000000CUL:      // instruction page fault
      HAL_instructionPageFaultExceptionHandler();
      break;
    case (1UL << (RISCV_XLEN-1)) | 0x00000003UL:      // machine software interrupt
      HAL_machineSoftwareInterruptCallback();
      break;
    case (1UL << (RISCV_XLEN-1)) | 0x00000007UL:      // machine timer interrupt
      HAL_machineTimerInterruptCallback();
      break;
    case (1UL << (RISCV_XLEN-1)) | 0x0000000BUL:      // machine external interrupt
      HAL_machineExternalInterruptCallback();
      break;
    default:
      break;
  }
  return m_epc;
}

