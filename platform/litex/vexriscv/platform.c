/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Dolu1990 <charles.papon.90@gmail.com>
 *
 */

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>
#include <sbi/riscv_io.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_console.h>
#include <sbi_utils/fdt/fdt_helper.h>
#include <sbi_utils/fdt/fdt_fixup.h>
#include <sbi_utils/ipi/aclint_mswi.h>
#include <sbi_utils/irqchip/plic.h>
#include <sbi_utils/serial/litex-uart.h>
#include <sbi_utils/timer/aclint_mtimer.h>

#define VEX_DEFAULT_HART_COUNT	8
#define VEX_DEFAULT_PLATFORM_FEATURES	SBI_PLATFORM_HAS_MFAULTS_DELEGATION
#define VEX_DEFAULT_UART_ADDR	0xf0001000
#define VEX_DEFAULT_PLIC_ADDR	0xf0c00000
#define VEX_DEFAULT_PLIC_NUM_SOURCES	4
#define VEX_DEFAULT_CLINT_ADDR	0xF0010000
#define VEX_DEFAULT_CLIC_ADDR	0xf0d00000  /* Default CLIC base address */
#define VEX_DEFAULT_ACLINT_MTIMER_FREQ	100000000
#define VEX_DEFAULT_ACLINT_MSWI_ADDR	\
		(VEX_DEFAULT_CLINT_ADDR + CLINT_MSWI_OFFSET)
#define VEX_DEFAULT_ACLINT_MTIMER_ADDR	\
		(VEX_DEFAULT_CLINT_ADDR + CLINT_MTIMER_OFFSET)
#define VEX_DEFAULT_HART_STACK_SIZE	8192

/* clang-format on */

/* CLIC mode detection flag */
static bool clic_mode_detected = false;

/* Simple CLIC initialization structure */
struct clic_data {
	unsigned long addr;
	u32 num_interrupts;
	u32 num_harts;
};

static struct clic_data clic = {
	.addr = VEX_DEFAULT_CLIC_ADDR,
	.num_interrupts = 64,
	.num_harts = VEX_DEFAULT_HART_COUNT,
};

static struct plic_data plic = {
	.addr = VEX_DEFAULT_PLIC_ADDR,
	.num_src = VEX_DEFAULT_PLIC_NUM_SOURCES,
};

static struct aclint_mswi_data mswi = {
	.addr = VEX_DEFAULT_ACLINT_MSWI_ADDR,
	.size = ACLINT_MSWI_SIZE,
	.first_hartid = 0,
	.hart_count = VEX_DEFAULT_HART_COUNT,
};

static struct aclint_mtimer_data mtimer = {
	.mtime_freq = VEX_DEFAULT_ACLINT_MTIMER_FREQ,
	.mtime_addr = VEX_DEFAULT_ACLINT_MTIMER_ADDR +
			  ACLINT_DEFAULT_MTIME_OFFSET,
	.mtime_size = ACLINT_DEFAULT_MTIME_SIZE,
	.mtimecmp_addr = VEX_DEFAULT_ACLINT_MTIMER_ADDR +
			  ACLINT_DEFAULT_MTIMECMP_OFFSET,
	.mtimecmp_size = ACLINT_DEFAULT_MTIMECMP_SIZE,
	.first_hartid = 0,
	.hart_count = VEX_DEFAULT_HART_COUNT,
	.has_64bit_mmio = true,
};

/*
 * Detect if CLIC is present and configured
 */
static bool vex_detect_clic(void)
{
	unsigned long mtvec, mtvec_orig;
	bool clic_supported = false;
	
	/* Save original mtvec */
	mtvec_orig = csr_read(CSR_MTVEC);
	
	/* Try to set CLIC mode (mode = 3) in mtvec.mode bits [1:0] */
	mtvec = (mtvec_orig & ~0x3UL) | 0x3UL;
	csr_write(CSR_MTVEC, mtvec);
	
	/* Read back to see if CLIC mode stuck */
	mtvec = csr_read(CSR_MTVEC);
	
	/* Check if mode bits are 3 (CLIC mode) */
	if ((mtvec & 0x3UL) == 0x3UL) {
		clic_supported = true;
		/* Keep CLIC mode active */
		sbi_printf("VexRiscv: CLIC hardware detected (mtvec.mode=3 supported)\n");
	} else {
		/* Restore original mode */
		csr_write(CSR_MTVEC, mtvec_orig);
		sbi_printf("VexRiscv: No CLIC support detected (mtvec.mode=%ld)\n", 
			   mtvec & 0x3UL);
	}
	
	return clic_supported;
}

/*
 * VexRiscv platform early initialization.
 */
static int vex_early_init(bool cold_boot)
{
	if (cold_boot) {
		/* Detect if CLIC mode is available */
		clic_mode_detected = vex_detect_clic();
	}
	
	return 0;
}

/*
 * VexRiscv platform final initialization.
 */
static int vex_final_init(bool cold_boot)
{
	void *fdt;

	if (!cold_boot)
		return 0;

	fdt = fdt_get_address();
	fdt_fixups(fdt);
	
	/* Report interrupt controller mode */
	if (clic_mode_detected) {
		sbi_printf("VexRiscv: Running in CLIC mode with %d interrupts\n", 
			   clic.num_interrupts);
	} else {
		sbi_printf("VexRiscv: Running in PLIC/CLINT mode\n");
	}

	return 0;
}

/*
 * Initialize the vexRiscv console.
 */
static int vex_console_init(void)
{
	return litex_uart_init(VEX_DEFAULT_UART_ADDR);
}

/* CLIC CSR addresses - these may vary by implementation */
#define CSR_MCLICBASE     0x307  /* Base address for CLIC memory-mapped registers */

/*
 * Initialize CLIC for current HART
 */
static int vex_clic_init(bool cold_boot)
{
	u32 hartid = current_hartid();
	unsigned long mtvec_val;
	
	if (cold_boot) {
		/* Basic CLIC initialization */
		sbi_printf("VexRiscv: Initializing CLIC at 0x%lx\n", clic.addr);
		
		/* Set CLIC mode in mtvec (mode = 3) */
		mtvec_val = csr_read(CSR_MTVEC);
		mtvec_val = (mtvec_val & ~0x3UL) | 0x3UL;
		csr_write(CSR_MTVEC, mtvec_val);
		
		/* Verify CLIC mode is active */
		mtvec_val = csr_read(CSR_MTVEC);
		if ((mtvec_val & 0x3UL) != 0x3UL) {
			sbi_printf("VexRiscv: WARNING: Failed to set CLIC mode in mtvec\n");
			return -1;
		}
		
		/* Clear all pending interrupts if accessible via MMIO */
		/* This would require memory-mapped access to CLIC registers */
		/* which depends on the specific implementation */
		
		/* Set interrupt threshold to 0 to allow all interrupts */
		/* The exact CSR number for mintthresh varies by implementation */
		/* Some implementations use custom CSRs in the 0x7XX range */
	}
	
	/* Per-hart CLIC configuration */
	sbi_printf("VexRiscv: CLIC initialized for hart %d in mode 0x%lx\n", 
		   hartid, csr_read(CSR_MTVEC) & 0x3);
	
	/* Enable machine interrupts */
	csr_set(CSR_MSTATUS, MSTATUS_MIE);
	
	/* Enable all interrupt sources in MIE */
	csr_write(CSR_MIE, -1UL);
	
	return 0;
}

/*
 * Initialize the vexRiscv interrupt controller for current HART.
 */
static int vex_irqchip_init(bool cold_boot)
{
	int rc;
	u32 hartid = current_hartid();

	/* Use CLIC if detected, otherwise fall back to PLIC */
	if (clic_mode_detected) {
		return vex_clic_init(cold_boot);
	}

	/* Traditional PLIC initialization */
	if (cold_boot) {
		rc = plic_cold_irqchip_init(&plic);
		if (rc)
			return rc;
	}

	return plic_warm_irqchip_init(&plic, hartid * 2, hartid * 2 + 1);

}

/*
 * Initialize IPI for current HART.
 */
static int vex_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = aclint_mswi_cold_init(&mswi);
		if (rc)
			return rc;
	}

	return aclint_mswi_warm_init();
}

/*
 * Initialize vexRiscv timer for current HART.
 */
static int vex_timer_init(bool cold_boot)
{
	int rc;
	if (cold_boot) {
		rc = aclint_mtimer_cold_init(&mtimer, NULL); /* Timer has no reference */
		if (rc)
			return rc;
	}

	return aclint_mtimer_warm_init();
}

/*
 * Platform descriptor.
 */
const struct sbi_platform_operations platform_ops = {
	.early_init = vex_early_init,
	.final_init = vex_final_init,
	.console_init = vex_console_init,
	.irqchip_init = vex_irqchip_init,
	.ipi_init = vex_ipi_init,
	.timer_init = vex_timer_init
};

const struct sbi_platform platform = {
	.opensbi_version = OPENSBI_VERSION,
	.platform_version = SBI_PLATFORM_VERSION(0x0, 0x01),
	.name = "LiteX / VexRiscv-SMP",
	.features = VEX_DEFAULT_PLATFORM_FEATURES,
	.hart_count = VEX_DEFAULT_HART_COUNT,
	.hart_stack_size = VEX_DEFAULT_HART_STACK_SIZE,
	.heap_size =
		SBI_PLATFORM_DEFAULT_HEAP_SIZE(VEX_DEFAULT_HART_COUNT),
	.platform_ops_addr = (unsigned long)&platform_ops
};
