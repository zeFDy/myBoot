/*
 *  Copyright (C) 2012 Altera Corporation <www.altera.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

/*
#include <common.h>
#include <asm/io.h>
#include <asm/pl310.h>
#include <asm/u-boot.h>
#include <asm/utils.h>
#include <image.h>
#include <asm/arch/reset_manager.h>
#include <spl.h>
#include <asm/arch/system_manager.h>
#include <asm/arch/freeze_controller.h>
#include <asm/arch/clock_manager.h>
#include <asm/arch/scan_manager.h>
#include <asm/arch/sdram.h>
#include <asm/arch/scu.h>
#include <asm/arch/nic301.h>

DECLARE_GLOBAL_DATA_PTR;

static struct pl310_regs *const pl310 =
	(struct pl310_regs *)CONFIG_SYS_PL310_BASE;
static struct scu_registers *scu_regs =
	(struct scu_registers *)SOCFPGA_MPUSCU_ADDRESS;
static struct socfpga_system_manager *sysmgr_regs =
	(struct socfpga_system_manager *)SOCFPGA_SYSMGR_ADDRESS;
*/

//u32 spl_boot_device(void)
//{
//	const u32 bsel = readl(&sysmgr_regs->bootinfo);
//
//	switch (bsel & 0x7) {
//	case 0x1:	/* FPGA (HPS2FPGA Bridge) */
//		return BOOT_DEVICE_RAM;
//	case 0x2:	/* NAND Flash (1.8V) */
//	case 0x3:	/* NAND Flash (3.0V) */
//		socfpga_per_reset(SOCFPGA_RESET(NAND), 0);
//		return BOOT_DEVICE_NAND;
//	case 0x4:	/* SD/MMC External Transceiver (1.8V) */
//	case 0x5:	/* SD/MMC Internal Transceiver (3.0V) */
//		socfpga_per_reset(SOCFPGA_RESET(SDMMC), 0);
//		socfpga_per_reset(SOCFPGA_RESET(DMA), 0);
//		return BOOT_DEVICE_MMC1;
//	case 0x6:	/* QSPI Flash (1.8V) */
//	case 0x7:	/* QSPI Flash (3.0V) */
//		socfpga_per_reset(SOCFPGA_RESET(QSPI), 0);
//		return BOOT_DEVICE_SPI;
//	default:
//		printf("Invalid boot device (bsel=%08x)!\n", bsel);
//		hang();
//	}
//}

void preloader_console_init(void);

//#ifdef CONFIG_SPL_MMC_SUPPORT
//u32 spl_boot_mode(const u32 boot_device)
//{
//#if defined(CONFIG_SPL_FAT_SUPPORT) || defined(CONFIG_SPL_EXT_SUPPORT)
//	return MMCSD_MODE_FS;
//#else
//	return MMCSD_MODE_RAW;
//#endif
//}
//#endif

// if LE
# define cpu_to_le32(x)		(x)
# define le32_to_cpu(x)		(x)

typedef unsigned char	uint8_t;
typedef unsigned short	uint16_t;
typedef unsigned int	uint32_t;
typedef unsigned int	u32;
typedef unsigned int	ulong;

//#if __LINUX_ARM_ARCH__ >= 7
#define ISB	asm volatile ("isb sy" : : : "memory")
#define DSB	asm volatile ("dsb sy" : : : "memory")
#define DMB	asm volatile ("dmb sy" : : : "memory")
//#elif __LINUX_ARM_ARCH__ == 6
//#define ISB	CP15ISB
//#define DSB	CP15DSB
//#define DMB	CP15DMB
//#else
//#define ISB	asm volatile ("" : : : "memory")
//#define DSB	CP15DSB
//#define DMB	asm volatile ("" : : : "memory")
//#endif

#define isb()	ISB
#define dsb()	DSB
#define dmb()	DMB


/*
 * Generic virtual read/write.  Note that we don't support half-word
 * read/writes.  We define __arch_*[bl] here, and leave __arch_*w
 * to the architecture specific code.
 */
#define __arch_getb(a)			(*(volatile unsigned char *)(a))
#define __arch_getw(a)			(*(volatile unsigned short *)(a))
#define __arch_getl(a)			(*(volatile unsigned int *)(a))
#define __arch_getq(a)			(*(volatile unsigned long long *)(a))

#define __arch_putb(v,a)		(*(volatile unsigned char *)(a) = (v))
#define __arch_putw(v,a)		(*(volatile unsigned short *)(a) = (v))
#define __arch_putl(v,a)		(*(volatile unsigned int *)(a) = (v))
#define __arch_putq(v,a)		(*(volatile unsigned long long *)(a) = (v))

static inline void __raw_writesb(unsigned long addr, const void *data,
				 int bytelen)
{
	uint8_t *buf = (uint8_t *)data;
	while(bytelen--)
		__arch_putb(*buf++, addr);
}

static inline void __raw_writesw(unsigned long addr, const void *data,
				 int wordlen)
{
	uint16_t *buf = (uint16_t *)data;
	while(wordlen--)
		__arch_putw(*buf++, addr);
}

static inline void __raw_writesl(unsigned long addr, const void *data,
				 int longlen)
{
	uint32_t *buf = (uint32_t *)data;
	while(longlen--)
		__arch_putl(*buf++, addr);
}

static inline void __raw_readsb(unsigned long addr, void *data, int bytelen)
{
	uint8_t *buf = (uint8_t *)data;
	while(bytelen--)
		*buf++ = __arch_getb(addr);
}

static inline void __raw_readsw(unsigned long addr, void *data, int wordlen)
{
	uint16_t *buf = (uint16_t *)data;
	while(wordlen--)
		*buf++ = __arch_getw(addr);
}

static inline void __raw_readsl(unsigned long addr, void *data, int longlen)
{
	uint32_t *buf = (uint32_t *)data;
	while(longlen--)
		*buf++ = __arch_getl(addr);
}

#define __raw_writeb(v,a)	__arch_putb(v,a)
#define __raw_writew(v,a)	__arch_putw(v,a)
#define __raw_writel(v,a)	__arch_putl(v,a)
#define __raw_writeq(v,a)	__arch_putq(v,a)

#define __raw_readb(a)		__arch_getb(a)
#define __raw_readw(a)		__arch_getw(a)
#define __raw_readl(a)		__arch_getl(a)
#define __raw_readq(a)		__arch_getq(a)

/*
 * TODO: The kernel offers some more advanced versions of barriers, it might
 * have some advantages to use them instead of the simple one here.
 */
//#define mb()		dsb()
#define __iormb()	dmb()
#define __iowmb()	dmb()

#define writeb(v,c)	({ u8  __v = v; __iowmb(); __arch_putb(__v,c); __v; })
#define writew(v,c)	({ u16 __v = v; __iowmb(); __arch_putw(__v,c); __v; })
#define writel(v,c)	({ u32 __v = v; __iowmb(); __arch_putl(__v,c); __v; })
#define writeq(v,c)	({ u64 __v = v; __iowmb(); __arch_putq(__v,c); __v; })

#define readb(c)	({ u8  __v = __arch_getb(c); __iormb(); __v; })
#define readw(c)	({ u16 __v = __arch_getw(c); __iormb(); __v; })
#define readl(c)	({ u32 __v = __arch_getl(c); __iormb(); __v; })
#define readq(c)	({ u64 __v = __arch_getq(c); __iormb(); __v; })

/*
 * The compiler seems to be incapable of optimising constants
 * properly.  Spell it out to the compiler in some cases.
 * These are only valid for small values of "off" (< 1<<12)
 */
#define __raw_base_writeb(val,base,off)	__arch_base_putb(val,base,off)
#define __raw_base_writew(val,base,off)	__arch_base_putw(val,base,off)
#define __raw_base_writel(val,base,off)	__arch_base_putl(val,base,off)

#define __raw_base_readb(base,off)	__arch_base_getb(base,off)
#define __raw_base_readw(base,off)	__arch_base_getw(base,off)
#define __raw_base_readl(base,off)	__arch_base_getl(base,off)

/*
 * Clear and set bits in one shot. These macros can be used to clear and
 * set multiple bits in a register using a single call. These macros can
 * also be used to set a multiple-bit bit pattern using a mask, by
 * specifying the mask in the 'clear' parameter and the new bit pattern
 * in the 'set' parameter.
 */

#define out_arch(type,endian,a,v)	__raw_write##type(cpu_to_##endian(v),a)
#define in_arch(type,endian,a)		endian##_to_cpu(__raw_read##type(a))

#define out_le64(a,v)	out_arch(q,le64,a,v)
#define out_le32(a,v)	out_arch(l,le32,a,v)
#define out_le16(a,v)	out_arch(w,le16,a,v)

#define in_le64(a)	in_arch(q,le64,a)
#define in_le32(a)	in_arch(l,le32,a)
#define in_le16(a)	in_arch(w,le16,a)

#define out_be32(a,v)	out_arch(l,be32,a,v)
#define out_be16(a,v)	out_arch(w,be16,a,v)

#define in_be32(a)	in_arch(l,be32,a)
#define in_be16(a)	in_arch(w,be16,a)

#define out_8(a,v)	__raw_writeb(v,a)
#define in_8(a)		__raw_readb(a)

#define clrbits(type, addr, clear) \
	out_##type((addr), in_##type(addr) & ~(clear))

#define setbits(type, addr, set) \
	out_##type((addr), in_##type(addr) | (set))

#define clrsetbits(type, addr, clear, set) \
	out_##type((addr), (in_##type(addr) & ~(clear)) | (set))

#define clrbits_be32(addr, clear) clrbits(be32, addr, clear)
#define setbits_be32(addr, set) setbits(be32, addr, set)
#define clrsetbits_be32(addr, clear, set) clrsetbits(be32, addr, clear, set)

#define clrbits_le32(addr, clear) clrbits(le32, addr, clear)
#define setbits_le32(addr, set) setbits(le32, addr, set)
#define clrsetbits_le32(addr, clear, set) clrsetbits(le32, addr, clear, set)

#define clrbits_be16(addr, clear) clrbits(be16, addr, clear)
#define setbits_be16(addr, set) setbits(be16, addr, set)
#define clrsetbits_be16(addr, clear, set) clrsetbits(be16, addr, clear, set)

#define clrbits_le16(addr, clear) clrbits(le16, addr, clear)
#define setbits_le16(addr, set) setbits(le16, addr, set)
#define clrsetbits_le16(addr, clear, set) clrsetbits(le16, addr, clear, set)

#define clrbits_8(addr, clear) clrbits(8, addr, clear)
#define setbits_8(addr, set) setbits(8, addr, set)
#define clrsetbits_8(addr, clear, set) clrsetbits(8, addr, clear, set)

// ac5
#define SOCFPGA_MPUL2_ADDRESS		0xfffef000
// a10
//#define SOCFPGA_MPUL2_ADDRESS		0xfffff000

#define SOCFPGA_L3REGS_ADDRESS		0xff800000
#define SOCFPGA_MPUSCU_ADDRESS		0xfffec000
#define SOCFPGA_SYSMGR_ADDRESS		0xffd08000
#define CONFIG_SYS_PL310_BASE		SOCFPGA_MPUL2_ADDRESS
#define SOCFPGA_RSTMGR_ADDRESS		0xffd05000

#define SOCFPGA_FPGAMGRREGS_ADDRESS	0xff706000


struct scu_registers {
	u32	ctrl;			/* 0x00 */
	u32	cfg;
	u32	cpsr;
	u32	iassr;
	u32	_pad_0x10_0x3c[12];	/* 0x10 */
	u32	fsar;			/* 0x40 */
	u32	fear;
	u32	_pad_0x48_0x50[2];
	u32	acr;			/* 0x54 */
	u32	sacr;
};

struct nic301_registers {
	u32	remap;				/* 0x0 */
	/* Security Register Group */
	u32	_pad_0x4_0x8[1];
	u32	l4main;
	u32	l4sp;
	u32	l4mp;				/* 0x10 */
	u32	l4osc1;
	u32	l4spim;
	u32	stm;
	u32	lwhps2fpgaregs;			/* 0x20 */
	u32	_pad_0x24_0x28[1];
	u32	usb1;
	u32	nanddata;
	u32	_pad_0x30_0x80[20];
	u32	usb0;				/* 0x80 */
	u32	nandregs;
	u32	qspidata;
	u32	fpgamgrdata;
	u32	hps2fpgaregs;			/* 0x90 */
	u32	acp;
	u32	rom;
	u32	ocram;
	u32	sdrdata;			/* 0xA0 */
	u32	_pad_0xa4_0x1fd0[1995];
	/* ID Register Group */
	u32	periph_id_4;			/* 0x1FD0 */
	u32	_pad_0x1fd4_0x1fe0[3];
	u32	periph_id_0;			/* 0x1FE0 */
	u32	periph_id_1;
	u32	periph_id_2;
	u32	periph_id_3;
	u32	comp_id_0;			/* 0x1FF0 */
	u32	comp_id_1;
	u32	comp_id_2;
	u32	comp_id_3;
	u32	_pad_0x2000_0x2008[2];
	/* L4 MAIN */
	u32	l4main_fn_mod_bm_iss;
	u32	_pad_0x200c_0x3008[1023];
	/* L4 SP */
	u32	l4sp_fn_mod_bm_iss;
	u32	_pad_0x300c_0x4008[1023];
	/* L4 MP */
	u32	l4mp_fn_mod_bm_iss;
	u32	_pad_0x400c_0x5008[1023];
	/* L4 OSC1 */
	u32	l4osc_fn_mod_bm_iss;
	u32	_pad_0x500c_0x6008[1023];
	/* L4 SPIM */
	u32	l4spim_fn_mod_bm_iss;
	u32	_pad_0x600c_0x7008[1023];
	/* STM */
	u32	stm_fn_mod_bm_iss;
	u32	_pad_0x700c_0x7108[63];
	u32	stm_fn_mod;
	u32	_pad_0x710c_0x8008[959];
	/* LWHPS2FPGA */
	u32	lwhps2fpga_fn_mod_bm_iss;
	u32	_pad_0x800c_0x8108[63];
	u32	lwhps2fpga_fn_mod;
	u32	_pad_0x810c_0xa008[1983];
	/* USB1 */
	u32	usb1_fn_mod_bm_iss;
	u32	_pad_0xa00c_0xa044[14];
	u32	usb1_ahb_cntl;
	u32	_pad_0xa048_0xb008[1008];
	/* NANDDATA */
	u32	nanddata_fn_mod_bm_iss;
	u32	_pad_0xb00c_0xb108[63];
	u32	nanddata_fn_mod;
	u32	_pad_0xb10c_0x20008[21439];
	/* USB0 */
	u32	usb0_fn_mod_bm_iss;
	u32	_pad_0x2000c_0x20044[14];
	u32	usb0_ahb_cntl;
	u32	_pad_0x20048_0x21008[1008];
	/* NANDREGS */
	u32	nandregs_fn_mod_bm_iss;
	u32	_pad_0x2100c_0x21108[63];
	u32	nandregs_fn_mod;
	u32	_pad_0x2110c_0x22008[959];
	/* QSPIDATA */
	u32	qspidata_fn_mod_bm_iss;
	u32	_pad_0x2200c_0x22044[14];
	u32	qspidata_ahb_cntl;
	u32	_pad_0x22048_0x23008[1008];
	/* FPGAMGRDATA */
	u32	fpgamgrdata_fn_mod_bm_iss;
	u32	_pad_0x2300c_0x23040[13];
	u32	fpgamgrdata_wr_tidemark;	/* 0x23040 */
	u32	_pad_0x23044_0x23108[49];
	u32	fn_mod;
	u32	_pad_0x2310c_0x24008[959];
	/* HPS2FPGA */
	u32	hps2fpga_fn_mod_bm_iss;
	u32	_pad_0x2400c_0x24040[13];
	u32	hps2fpga_wr_tidemark;		/* 0x24040 */
	u32	_pad_0x24044_0x24108[49];
	u32	hps2fpga_fn_mod;
	u32	_pad_0x2410c_0x25008[959];
	/* ACP */
	u32	acp_fn_mod_bm_iss;
	u32	_pad_0x2500c_0x25108[63];
	u32	acp_fn_mod;
	u32	_pad_0x2510c_0x26008[959];
	/* Boot ROM */
	u32	bootrom_fn_mod_bm_iss;
	u32	_pad_0x2600c_0x26108[63];
	u32	bootrom_fn_mod;
	u32	_pad_0x2610c_0x27008[959];
	/* On-chip RAM */
	u32	ocram_fn_mod_bm_iss;
	u32	_pad_0x2700c_0x27040[13];
	u32	ocram_wr_tidemark;		/* 0x27040 */
	u32	_pad_0x27044_0x27108[49];
	u32	ocram_fn_mod;
	u32	_pad_0x2710c_0x42024[27590];
	/* DAP */
	u32	dap_fn_mod2;
	u32	dap_fn_mod_ahb;
	u32	_pad_0x4202c_0x42100[53];
	u32	dap_read_qos;			/* 0x42100 */
	u32	dap_write_qos;
	u32	dap_fn_mod;
	u32	_pad_0x4210c_0x43100[1021];
	/* MPU */
	u32	mpu_read_qos;			/* 0x43100 */
	u32	mpu_write_qos;
	u32	mpu_fn_mod;
	u32	_pad_0x4310c_0x44028[967];
	/* SDMMC */
	u32	sdmmc_fn_mod_ahb;
	u32	_pad_0x4402c_0x44100[53];
	u32	sdmmc_read_qos;			/* 0x44100 */
	u32	sdmmc_write_qos;
	u32	sdmmc_fn_mod;
	u32	_pad_0x4410c_0x45100[1021];
	/* DMA */
	u32	dma_read_qos;			/* 0x45100 */
	u32	dma_write_qos;
	u32	dma_fn_mod;
	u32	_pad_0x4510c_0x46040[973];
	/* FPGA2HPS */
	u32	fpga2hps_wr_tidemark;		/* 0x46040 */
	u32	_pad_0x46044_0x46100[47];
	u32	fpga2hps_read_qos;		/* 0x46100 */
	u32	fpga2hps_write_qos;
	u32	fpga2hps_fn_mod;
	u32	_pad_0x4610c_0x47100[1021];
	/* ETR */
	u32	etr_read_qos;			/* 0x47100 */
	u32	etr_write_qos;
	u32	etr_fn_mod;
	u32	_pad_0x4710c_0x48100[1021];
	/* EMAC0 */
	u32	emac0_read_qos;			/* 0x48100 */
	u32	emac0_write_qos;
	u32	emac0_fn_mod;
	u32	_pad_0x4810c_0x49100[1021];
	/* EMAC1 */
	u32	emac1_read_qos;			/* 0x49100 */
	u32	emac1_write_qos;
	u32	emac1_fn_mod;
	u32	_pad_0x4910c_0x4a028[967];
	/* USB0 */
	u32	usb0_fn_mod_ahb;
	u32	_pad_0x4a02c_0x4a100[53];
	u32	usb0_read_qos;			/* 0x4A100 */
	u32	usb0_write_qos;
	u32	usb0_fn_mod;
	u32	_pad_0x4a10c_0x4b100[1021];
	/* NAND */
	u32	nand_read_qos;			/* 0x4B100 */
	u32	nand_write_qos;
	u32	nand_fn_mod;
	u32	_pad_0x4b10c_0x4c028[967];
	/* USB1 */
	u32	usb1_fn_mod_ahb;
	u32	_pad_0x4c02c_0x4c100[53];
	u32	usb1_read_qos;			/* 0x4C100 */
	u32	usb1_write_qos;
	u32	usb1_fn_mod;
};

struct socfpga_system_manager {
	/* System Manager Module */
	u32	siliconid1;			/* 0x00 */
	u32	siliconid2;
	u32	_pad_0x8_0xf[2];
	u32	wddbg;				/* 0x10 */
	u32	bootinfo;
	u32	hpsinfo;
	u32	parityinj;
	/* FPGA Interface Group */
	u32	fpgaintfgrp_gbl;		/* 0x20 */
	u32	fpgaintfgrp_indiv;
	u32	fpgaintfgrp_module;
	u32	_pad_0x2c_0x2f;
	/* Scan Manager Group */
	u32	scanmgrgrp_ctrl;		/* 0x30 */
	u32	_pad_0x34_0x3f[3];
	/* Freeze Control Group */
	u32	frzctrl_vioctrl;		/* 0x40 */
	u32	_pad_0x44_0x4f[3];
	u32	frzctrl_hioctrl;		/* 0x50 */
	u32	frzctrl_src;
	u32	frzctrl_hwctrl;
	u32	_pad_0x5c_0x5f;
	/* EMAC Group */
	u32	emacgrp_ctrl;			/* 0x60 */
	u32	emacgrp_l3master;
	u32	_pad_0x68_0x6f[2];
	/* DMA Controller Group */
	u32	dmagrp_ctrl;			/* 0x70 */
	u32	dmagrp_persecurity;
	u32	_pad_0x78_0x7f[2];
	/* Preloader (initial software) Group */
	u32	iswgrp_handoff[8];		/* 0x80 */
	u32	_pad_0xa0_0xbf[8];		/* 0xa0 */
	/* Boot ROM Code Register Group */
	u32	romcodegrp_ctrl;		/* 0xc0 */
	u32	romcodegrp_cpu1startaddr;
	u32	romcodegrp_initswstate;
	u32	romcodegrp_initswlastld;
	u32	romcodegrp_bootromswstate;	/* 0xd0 */
	u32	__pad_0xd4_0xdf[3];
	/* Warm Boot from On-Chip RAM Group */
	u32	romcodegrp_warmramgrp_enable;	/* 0xe0 */
	u32	romcodegrp_warmramgrp_datastart;
	u32	romcodegrp_warmramgrp_length;
	u32	romcodegrp_warmramgrp_execution;
	u32	romcodegrp_warmramgrp_crc;	/* 0xf0 */
	u32	__pad_0xf4_0xff[3];
	/* Boot ROM Hardware Register Group */
	u32	romhwgrp_ctrl;			/* 0x100 */
	u32	_pad_0x104_0x107;
	/* SDMMC Controller Group */
	u32	sdmmcgrp_ctrl;
	u32	sdmmcgrp_l3master;
	/* NAND Flash Controller Register Group */
	u32	nandgrp_bootstrap;		/* 0x110 */
	u32	nandgrp_l3master;
	/* USB Controller Group */
	u32	usbgrp_l3master;
	u32	_pad_0x11c_0x13f[9];
	/* ECC Management Register Group */
	u32	eccgrp_l2;			/* 0x140 */
	u32	eccgrp_ocram;
	u32	eccgrp_usb0;
	u32	eccgrp_usb1;
	u32	eccgrp_emac0;			/* 0x150 */
	u32	eccgrp_emac1;
	u32	eccgrp_dma;
	u32	eccgrp_can0;
	u32	eccgrp_can1;			/* 0x160 */
	u32	eccgrp_nand;
	u32	eccgrp_qspi;
	u32	eccgrp_sdmmc;
	u32	_pad_0x170_0x3ff[164];
	/* Pin Mux Control Group */
	u32	emacio[20];			/* 0x400 */
	u32	flashio[12];			/* 0x450 */
	u32	generalio[28];			/* 0x480 */
	u32	_pad_0x4f0_0x4ff[4];
	u32	mixed1io[22];			/* 0x500 */
	u32	mixed2io[8];			/* 0x558 */
	u32	gplinmux[23];			/* 0x578 */
	u32	gplmux[71];			/* 0x5d4 */
	u32	nandusefpga;			/* 0x6f0 */
	u32	_pad_0x6f4;
	u32	rgmii1usefpga;			/* 0x6f8 */
	u32	_pad_0x6fc_0x700[2];
	u32	i2c0usefpga;			/* 0x704 */
	u32	sdmmcusefpga;			/* 0x708 */
	u32	_pad_0x70c_0x710[2];
	u32	rgmii0usefpga;			/* 0x714 */
	u32	_pad_0x718_0x720[3];
	u32	i2c3usefpga;			/* 0x724 */
	u32	i2c2usefpga;			/* 0x728 */
	u32	i2c1usefpga;			/* 0x72c */
	u32	spim1usefpga;			/* 0x730 */
	u32	_pad_0x734;
	u32	spim0usefpga;			/* 0x738 */
};


struct pl310_regs {
	u32 pl310_cache_id;
	u32 pl310_cache_type;
	u32 pad1[62];
	u32 pl310_ctrl;
	u32 pl310_aux_ctrl;
	u32 pl310_tag_latency_ctrl;
	u32 pl310_data_latency_ctrl;
	u32 pad2[60];
	u32 pl310_event_cnt_ctrl;
	u32 pl310_event_cnt1_cfg;
	u32 pl310_event_cnt0_cfg;
	u32 pl310_event_cnt1_val;
	u32 pl310_event_cnt0_val;
	u32 pl310_intr_mask;
	u32 pl310_masked_intr_stat;
	u32 pl310_raw_intr_stat;
	u32 pl310_intr_clear;
	u32 pad3[323];
	u32 pl310_cache_sync;
	u32 pad4[15];
	u32 pl310_inv_line_pa;
	u32 pad5[2];
	u32 pl310_inv_way;
	u32 pad6[12];
	u32 pl310_clean_line_pa;
	u32 pad7[1];
	u32 pl310_clean_line_idx;
	u32 pl310_clean_way;
	u32 pad8[12];
	u32 pl310_clean_inv_line_pa;
	u32 pad9[1];
	u32 pl310_clean_inv_line_idx;
	u32 pl310_clean_inv_way;
	u32 pad10[64];
	u32 pl310_lockdown_dbase;
	u32 pl310_lockdown_ibase;
	u32 pad11[190];
	u32 pl310_addr_filter_start;
	u32 pl310_addr_filter_end;
	u32 pad12[190];
	u32 pl310_test_operation;
	u32 pad13[3];
	u32 pl310_line_data;
	u32 pad14[7];
	u32 pl310_line_tag;
	u32 pad15[3];
	u32 pl310_debug_ctrl;
	u32 pad16[7];
	u32 pl310_prefetch_ctrl;
	u32 pad17[7];
	u32 pl310_power_ctrl;
};

struct socfpga_fpga_manager {
	/* FPGA Manager Module */
	u32	stat;			/* 0x00 */
	u32	ctrl;
	u32	dclkcnt;
	u32	dclkstat;
	u32	gpo;			/* 0x10 */
	u32	gpi;
	u32	misci;			/* 0x18 */
	u32	_pad_0x1c_0x82c[517];

	/* Configuration Monitor (MON) Registers */
	u32	gpio_inten;		/* 0x830 */
	u32	gpio_intmask;
	u32	gpio_inttype_level;
	u32	gpio_int_polarity;
	u32	gpio_intstatus;		/* 0x840 */
	u32	gpio_raw_intstatus;
	u32	_pad_0x848;
	u32	gpio_porta_eoi;
	u32	gpio_ext_porta;		/* 0x850 */
	u32	_pad_0x854_0x85c[3];
	u32	gpio_1s_sync;		/* 0x860 */
	u32	_pad_0x864_0x868[2];
	u32	gpio_ver_id_code;
	u32	gpio_config_reg2;	/* 0x870 */
	u32	gpio_config_reg1;
};

#define FPGAMGRREGS_STAT_MODE_MASK		0x7
#define FPGAMGRREGS_STAT_MSEL_MASK		0xf8
#define FPGAMGRREGS_STAT_MSEL_LSB		3

#define FPGAMGRREGS_CTRL_CFGWDTH_MASK		0x200
#define FPGAMGRREGS_CTRL_AXICFGEN_MASK		0x100
#define FPGAMGRREGS_CTRL_NCONFIGPULL_MASK	0x4
#define FPGAMGRREGS_CTRL_NCE_MASK		0x2
#define FPGAMGRREGS_CTRL_EN_MASK		0x1
#define FPGAMGRREGS_CTRL_CDRATIO_LSB		6

#define FPGAMGRREGS_MON_GPIO_EXT_PORTA_CRC_MASK	0x8
#define FPGAMGRREGS_MON_GPIO_EXT_PORTA_ID_MASK	0x4
#define FPGAMGRREGS_MON_GPIO_EXT_PORTA_CD_MASK	0x2
#define FPGAMGRREGS_MON_GPIO_EXT_PORTA_NS_MASK	0x1

/* FPGA Mode */
#define FPGAMGRREGS_MODE_FPGAOFF		0x0
#define FPGAMGRREGS_MODE_RESETPHASE		0x1
#define FPGAMGRREGS_MODE_CFGPHASE		0x2
#define FPGAMGRREGS_MODE_INITPHASE		0x3
#define FPGAMGRREGS_MODE_USERMODE		0x4
#define FPGAMGRREGS_MODE_UNKNOWN		0x5


#define SYSMGR_ROMCODEGRP_CTRL_WARMRSTCFGPINMUX	(1 << 0)
#define SYSMGR_ROMCODEGRP_CTRL_WARMRSTCFGIO		(1 << 1)
#define SYSMGR_ECC_OCRAM_EN						(1 << 0)
#define SYSMGR_ECC_OCRAM_SERR					(1 << 3)
#define SYSMGR_ECC_OCRAM_DERR					(1 << 4)
#define SYSMGR_FPGAINTF_USEFPGA						0x1
#define SYSMGR_FPGAINTF_SPIM0					(1 << 0)
#define SYSMGR_FPGAINTF_SPIM1					(1 << 1)
#define SYSMGR_FPGAINTF_EMAC0					(1 << 2)
#define SYSMGR_FPGAINTF_EMAC1					(1 << 3)
#define SYSMGR_FPGAINTF_NAND					(1 << 4)
#define SYSMGR_FPGAINTF_SDMMC					(1 << 5)

#define L3REGS_REMAP_LWHPS2FPGA_MASK	0x10
#define L3REGS_REMAP_HPS2FPGA_MASK	0x08
#define L3REGS_REMAP_OCRAM_MASK		0x01

#define RSTMGR_BANK_OFFSET	8
#define RSTMGR_BANK_MASK	0x7
#define RSTMGR_RESET_OFFSET	0
#define RSTMGR_RESET_MASK	0x1f
#define RSTMGR_DEFINE(_bank, _offset)		\
	((_bank) << RSTMGR_BANK_OFFSET) | ((_offset) << RSTMGR_RESET_OFFSET)

/* Extract reset ID from the reset identifier. */
#define RSTMGR_RESET(_reset)			\
	(((_reset) >> RSTMGR_RESET_OFFSET) & RSTMGR_RESET_MASK)

/* Extract bank ID from the reset identifier. */
#define RSTMGR_BANK(_reset)			\
	(((_reset) >> RSTMGR_BANK_OFFSET) & RSTMGR_BANK_MASK)

/*
 * SocFPGA Cyclone V/Arria V reset IDs, bank mapping is as follows:
 * 0 ... mpumodrst
 * 1 ... permodrst
 * 2 ... per2modrst
 * 3 ... brgmodrst
 * 4 ... miscmodrst
 */
#define RSTMGR_EMAC0		RSTMGR_DEFINE(1, 0)
#define RSTMGR_EMAC1		RSTMGR_DEFINE(1, 1)
#define RSTMGR_NAND		RSTMGR_DEFINE(1, 4)
#define RSTMGR_QSPI		RSTMGR_DEFINE(1, 5)
#define RSTMGR_L4WD0		RSTMGR_DEFINE(1, 6)
#define RSTMGR_OSC1TIMER0	RSTMGR_DEFINE(1, 8)
#define RSTMGR_UART0		RSTMGR_DEFINE(1, 16)
#define RSTMGR_SPIM0		RSTMGR_DEFINE(1, 18)
#define RSTMGR_SPIM1		RSTMGR_DEFINE(1, 19)
#define RSTMGR_SDMMC		RSTMGR_DEFINE(1, 22)
#define RSTMGR_DMA		RSTMGR_DEFINE(1, 28)
#define RSTMGR_SDR		RSTMGR_DEFINE(1, 29)

#define SOCFPGA_OSC1TIMER0_ADDRESS	0xffd00000

/* Create a human-readable reference to SoCFPGA reset. */
#define SOCFPGA_RESET(_name)	RSTMGR_##_name
#define TIMER_LOAD_VAL		0xFFFFFFFF
#define CONFIG_SYS_TIMERBASE		SOCFPGA_OSC1TIMER0_ADDRESS

static const struct socfpga_timer *timer_base = (void *)CONFIG_SYS_TIMERBASE;


static struct pl310_regs *const pl310 =
	(struct pl310_regs *)CONFIG_SYS_PL310_BASE;
static struct scu_registers *scu_regs =
	(struct scu_registers *)SOCFPGA_MPUSCU_ADDRESS;
static struct nic301_registers *nic301_regs =
	(struct nic301_registers *)SOCFPGA_L3REGS_ADDRESS;
static struct socfpga_system_manager *sysmgr_regs =
	(struct socfpga_system_manager *)SOCFPGA_SYSMGR_ADDRESS;
static const struct socfpga_reset_manager *reset_manager_base =
		(void *)SOCFPGA_RSTMGR_ADDRESS;

static struct socfpga_fpga_manager *fpgamgr_regs =
	(struct socfpga_fpga_manager *)SOCFPGA_FPGAMGRREGS_ADDRESS;

struct socfpga_reset_manager {
	u32	status;
	u32	ctrl;
	u32	counts;
	u32	padding1;
	u32	mpu_mod_reset;
	u32	per_mod_reset;
	u32	per2_mod_reset;
	u32	brg_mod_reset;
	u32	misc_mod_reset;
	u32	padding2[12];
	u32	tstscratch;
};

struct socfpga_freeze_controller {
	u32	vioctrl;
	u32	padding[3];
	u32	hioctrl;
	u32	src;
	u32	hwctrl;
};

struct socfpga_timer {
	u32	load_val;
	u32	curr_val;
	u32	ctrl;
	u32	eoi;
	u32	int_stat;
};

#define FREEZE_CHANNEL_NUM		(4)

typedef enum {
	FREEZE_CTRL_FROZEN = 0,
	FREEZE_CTRL_THAWED = 1
} FREEZE_CTRL_CHAN_STATE;

#define SYSMGR_FRZCTRL_ADDRESS 0x40
#define SYSMGR_FRZCTRL_SRC_VIO1_ENUM_SW 0x0
#define SYSMGR_FRZCTRL_SRC_VIO1_ENUM_HW 0x1
#define SYSMGR_FRZCTRL_VIOCTRL_SLEW_MASK 0x00000010
#define SYSMGR_FRZCTRL_VIOCTRL_WKPULLUP_MASK 0x00000008
#define SYSMGR_FRZCTRL_VIOCTRL_TRISTATE_MASK 0x00000004
#define SYSMGR_FRZCTRL_VIOCTRL_BUSHOLD_MASK 0x00000002
#define SYSMGR_FRZCTRL_VIOCTRL_CFG_MASK 0x00000001
#define SYSMGR_FRZCTRL_HIOCTRL_SLEW_MASK 0x00000010
#define SYSMGR_FRZCTRL_HIOCTRL_WKPULLUP_MASK 0x00000008
#define SYSMGR_FRZCTRL_HIOCTRL_TRISTATE_MASK 0x00000004
#define SYSMGR_FRZCTRL_HIOCTRL_BUSHOLD_MASK 0x00000002
#define SYSMGR_FRZCTRL_HIOCTRL_CFG_MASK 0x00000001
#define SYSMGR_FRZCTRL_HIOCTRL_REGRST_MASK 0x00000080
#define SYSMGR_FRZCTRL_HIOCTRL_OCTRST_MASK 0x00000040
#define SYSMGR_FRZCTRL_HIOCTRL_OCT_CFGEN_CALSTART_MASK 0x00000100
#define SYSMGR_FRZCTRL_HIOCTRL_DLLRST_MASK 0x00000020
#define SYSMGR_FRZCTRL_HWCTRL_VIO1REQ_MASK 0x00000001
#define SYSMGR_FRZCTRL_HWCTRL_VIO1STATE_ENUM_FROZEN 0x2
#define SYSMGR_FRZCTRL_HWCTRL_VIO1STATE_ENUM_THAWED 0x1

#define DIV_ROUND_UP(n, d)	(((n) + (d) - 1) / (d))

#define CONFIG_HPS_CLK_OSC1_HZ 25000000
#define CONFIG_HPS_CLK_OSC2_HZ 25000000

// next block to define


void udelay(unsigned long usec)
{

}

const unsigned int cm_get_osc_clk_hz(const int osc)
{
	if (osc == 1)
		return CONFIG_HPS_CLK_OSC1_HZ;
	else if (osc == 2)
		return CONFIG_HPS_CLK_OSC2_HZ;
	else
		return 0;
}

const struct cm_config *const cm_get_default_config(void)
{

}

static const struct socfpga_freeze_controller *freeze_controller_base =
		(void *)(SOCFPGA_SYSMGR_ADDRESS + SYSMGR_FRZCTRL_ADDRESS);

/*
 * Default state from cold reset is FREEZE_ALL; the global
 * flag is set to TRUE to indicate the IO banks are frozen
 */
static uint32_t frzctrl_channel_freeze[FREEZE_CHANNEL_NUM]
	= { FREEZE_CTRL_FROZEN, FREEZE_CTRL_FROZEN,
	FREEZE_CTRL_FROZEN, FREEZE_CTRL_FROZEN};

/* Freeze HPS IOs */
void sys_mgr_frzctrl_freeze_req(void)
{
	u32 ioctrl_reg_offset;
	u32 reg_value;
	u32 reg_cfg_mask;
	u32 channel_id;

	/* select software FSM */
	writel(SYSMGR_FRZCTRL_SRC_VIO1_ENUM_SW,	&freeze_controller_base->src);

	/* Freeze channel 0 to 2 */
	for (channel_id = 0; channel_id <= 2; channel_id++) {
		ioctrl_reg_offset = (u32)(
			&freeze_controller_base->vioctrl + channel_id);

		/*
		 * Assert active low enrnsl, plniotri
		 * and niotri signals
		 */
		reg_cfg_mask =
			SYSMGR_FRZCTRL_VIOCTRL_SLEW_MASK
			| SYSMGR_FRZCTRL_VIOCTRL_WKPULLUP_MASK
			| SYSMGR_FRZCTRL_VIOCTRL_TRISTATE_MASK;
		clrbits_le32(ioctrl_reg_offset,	reg_cfg_mask);

		/*
		 * Note: Delay for 20ns at min
		 * Assert active low bhniotri signal and de-assert
		 * active high csrdone
		 */
		reg_cfg_mask
			= SYSMGR_FRZCTRL_VIOCTRL_BUSHOLD_MASK
			| SYSMGR_FRZCTRL_VIOCTRL_CFG_MASK;
		clrbits_le32(ioctrl_reg_offset,	reg_cfg_mask);

		/* Set global flag to indicate channel is frozen */
		frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_FROZEN;
	}

	/* Freeze channel 3 */
	/*
	 * Assert active low enrnsl, plniotri and
	 * niotri signals
	 */
	reg_cfg_mask
		= SYSMGR_FRZCTRL_HIOCTRL_SLEW_MASK
		| SYSMGR_FRZCTRL_HIOCTRL_WKPULLUP_MASK
		| SYSMGR_FRZCTRL_HIOCTRL_TRISTATE_MASK;
	clrbits_le32(&freeze_controller_base->hioctrl, reg_cfg_mask);

	/*
	 * assert active low bhniotri & nfrzdrv signals,
	 * de-assert active high csrdone and assert
	 * active high frzreg and nfrzdrv signals
	 */
	reg_value = readl(&freeze_controller_base->hioctrl);
	reg_cfg_mask
		= SYSMGR_FRZCTRL_HIOCTRL_BUSHOLD_MASK
		| SYSMGR_FRZCTRL_HIOCTRL_CFG_MASK;
	reg_value
		= (reg_value & ~reg_cfg_mask)
		| SYSMGR_FRZCTRL_HIOCTRL_REGRST_MASK
		| SYSMGR_FRZCTRL_HIOCTRL_OCTRST_MASK;
	writel(reg_value, &freeze_controller_base->hioctrl);

	/*
	 * assert active high reinit signal and de-assert
	 * active high pllbiasen signals
	 */
	reg_value = readl(&freeze_controller_base->hioctrl);
	reg_value
		= (reg_value &
		~SYSMGR_FRZCTRL_HIOCTRL_OCT_CFGEN_CALSTART_MASK)
		| SYSMGR_FRZCTRL_HIOCTRL_DLLRST_MASK;
	writel(reg_value, &freeze_controller_base->hioctrl);

	/* Set global flag to indicate channel is frozen */
	frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_FROZEN;
}

/* Unfreeze/Thaw HPS IOs */
void sys_mgr_frzctrl_thaw_req(void)
{
	u32 ioctrl_reg_offset;
	u32 reg_cfg_mask;
	u32 reg_value;
	u32 channel_id;
	unsigned long eosc1_freq;

	/* select software FSM */
	writel(SYSMGR_FRZCTRL_SRC_VIO1_ENUM_SW,	&freeze_controller_base->src);

	/* Thaw channel 0 to 2 */
	for (channel_id = 0; channel_id <= 2; channel_id++) {
		ioctrl_reg_offset
			= (u32)(&freeze_controller_base->vioctrl + channel_id);

		/*
		 * Assert active low bhniotri signal and
		 * de-assert active high csrdone
		 */
		reg_cfg_mask
			= SYSMGR_FRZCTRL_VIOCTRL_BUSHOLD_MASK
			| SYSMGR_FRZCTRL_VIOCTRL_CFG_MASK;
		setbits_le32(ioctrl_reg_offset,	reg_cfg_mask);

		/*
		 * Note: Delay for 20ns at min
		 * de-assert active low plniotri and niotri signals
		 */
		reg_cfg_mask
			= SYSMGR_FRZCTRL_VIOCTRL_WKPULLUP_MASK
			| SYSMGR_FRZCTRL_VIOCTRL_TRISTATE_MASK;
		setbits_le32(ioctrl_reg_offset,	reg_cfg_mask);

		/*
		 * Note: Delay for 20ns at min
		 * de-assert active low enrnsl signal
		 */
		setbits_le32(ioctrl_reg_offset,
			SYSMGR_FRZCTRL_VIOCTRL_SLEW_MASK);

		/* Set global flag to indicate channel is thawed */
		frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_THAWED;
	}

	/* Thaw channel 3 */
	/* de-assert active high reinit signal */
	clrbits_le32(&freeze_controller_base->hioctrl,
		SYSMGR_FRZCTRL_HIOCTRL_DLLRST_MASK);

	/*
	 * Note: Delay for 40ns at min
	 * assert active high pllbiasen signals
	 */
	setbits_le32(&freeze_controller_base->hioctrl,
		SYSMGR_FRZCTRL_HIOCTRL_OCT_CFGEN_CALSTART_MASK);

	/* Delay 1000 intosc cycles. The intosc is based on eosc1. */
	eosc1_freq = cm_get_osc_clk_hz(1) / 1000;	/* kHz */
	
	//udelay(DIV_ROUND_UP(1000000, eosc1_freq));
	udelay(100);

	/*
	 * de-assert active low bhniotri signals,
	 * assert active high csrdone and nfrzdrv signal
	 */
	reg_value = readl(&freeze_controller_base->hioctrl);
	reg_value = (reg_value
		| SYSMGR_FRZCTRL_HIOCTRL_BUSHOLD_MASK
		| SYSMGR_FRZCTRL_HIOCTRL_CFG_MASK)
		& ~SYSMGR_FRZCTRL_HIOCTRL_OCTRST_MASK;
	writel(reg_value, &freeze_controller_base->hioctrl);

	/*
	 * Delay 33 intosc
	 * Use worst case which is fatest eosc1=50MHz, delay required
	 * is 1/50MHz * 33 = 660ns ~= 1us
	 */
	udelay(1);

	/* de-assert active low plniotri and niotri signals */
	reg_cfg_mask
		= SYSMGR_FRZCTRL_HIOCTRL_WKPULLUP_MASK
		| SYSMGR_FRZCTRL_HIOCTRL_TRISTATE_MASK;

	setbits_le32(&freeze_controller_base->hioctrl, reg_cfg_mask);

	/*
	 * Note: Delay for 40ns at min
	 * de-assert active high frzreg signal
	 */
	clrbits_le32(&freeze_controller_base->hioctrl,
		SYSMGR_FRZCTRL_HIOCTRL_REGRST_MASK);

	/*
	 * Note: Delay for 40ns at min
	 * de-assert active low enrnsl signal
	 */
	setbits_le32(&freeze_controller_base->hioctrl,
		SYSMGR_FRZCTRL_HIOCTRL_SLEW_MASK);

	/* Set global flag to indicate channel is thawed */
	frzctrl_channel_freeze[channel_id] = FREEZE_CTRL_THAWED;
}



void socfpga_per_reset_all(void)
{
	const u32 l4wd0 = 1 << RSTMGR_RESET(SOCFPGA_RESET(L4WD0));

	writel(~l4wd0, &reset_manager_base->per_mod_reset);
	writel(0xffffffff, &reset_manager_base->per2_mod_reset);
}

void socfpga_per_reset(u32 reset, int set)
{
	const void *reg;

	if (RSTMGR_BANK(reset) == 0)
		reg = &reset_manager_base->mpu_mod_reset;
	else if (RSTMGR_BANK(reset) == 1)
		reg = &reset_manager_base->per_mod_reset;
	else if (RSTMGR_BANK(reset) == 2)
		reg = &reset_manager_base->per2_mod_reset;
	else if (RSTMGR_BANK(reset) == 3)
		reg = &reset_manager_base->brg_mod_reset;
	else if (RSTMGR_BANK(reset) == 4)
		reg = &reset_manager_base->misc_mod_reset;
	else	/* Invalid reset register, do nothing */
		return;

	if (set)
		setbits_le32(reg, 1 << RSTMGR_RESET(reset));
	else
		clrbits_le32(reg, 1 << RSTMGR_RESET(reset));
}

/* Check whether FPGA Init_Done signal is high */
static int is_fpgamgr_initdone_high(void)
{
	unsigned long val;

	val = readl(&fpgamgr_regs->gpio_ext_porta);
	return val & FPGAMGRREGS_MON_GPIO_EXT_PORTA_ID_MASK;
}

/* Get the FPGA mode */
int fpgamgr_get_mode(void)
{
	unsigned long val;

	val = readl(&fpgamgr_regs->stat);
	return val & FPGAMGRREGS_STAT_MODE_MASK;
}

/* Check whether FPGA is ready to be accessed */
int fpgamgr_test_fpga_ready(void)
{
	/* Check for init done signal */
	if (!is_fpgamgr_initdone_high())
		return 0;

	/* Check again to avoid false glitches */
	if (!is_fpgamgr_initdone_high())
		return 0;

	if (fpgamgr_get_mode() != FPGAMGRREGS_MODE_USERMODE)
		return 0;

	return 1;
}

void socfpga_bridges_reset(int enable)
{
	const uint32_t l3mask = L3REGS_REMAP_LWHPS2FPGA_MASK |
				L3REGS_REMAP_HPS2FPGA_MASK |
				L3REGS_REMAP_OCRAM_MASK;

	if (enable) {
		/* brdmodrst */
		writel(0xffffffff, &reset_manager_base->brg_mod_reset);
	} else {
		writel(0, &sysmgr_regs->iswgrp_handoff[0]);
		writel(l3mask, &sysmgr_regs->iswgrp_handoff[1]);

		/* Check signal from FPGA. */
		if (!fpgamgr_test_fpga_ready()) {
			/* FPGA not ready, do nothing. */
			//printf("%s: FPGA not ready, aborting.\n", __func__);
			return;
		}

		/* brdmodrst */
		writel(0, &reset_manager_base->brg_mod_reset);

		/* Remap the bridges into memory map */
		writel(l3mask, SOCFPGA_L3REGS_ADDRESS);
	}
}

int timer_init(void)
{
	writel(TIMER_LOAD_VAL, &timer_base->load_val);
	writel(TIMER_LOAD_VAL, &timer_base->curr_val);
	writel(readl(&timer_base->ctrl) | 0x3, &timer_base->ctrl);
	return 0;
}


void hang(void)
{
	//puts("### ERROR ### Please RESET the board ###\n");
	//bootstage_error(BOOTSTAGE_ID_NEED_RESET);
	for (;;)
		;
}


void cm_basic_init(const struct cm_config *const cfg)
{

}

void sysmgr_config_warmrstcfgio(int enable)
{

}

int scan_mgr_configure_iocsr(void)
{

}

void sysmgr_pinmux_init(void)
{

}

int sdram_mmr_init_full(unsigned int sdr_phy_reg)
{

}

int sdram_calibration_full(void)
{

} 

unsigned long sdram_calculate_size(void)
{

}

long get_ram_size(long *pointer, long size)
{

}

void reset_deassert_peripherals_handoff(void)
{
	writel(0, &reset_manager_base->per_mod_reset);
}



// end of block


static void socfpga_nic301_slave_ns(void)
{
	writel(0x1, &nic301_regs->lwhps2fpgaregs);
	writel(0x1, &nic301_regs->hps2fpgaregs);
	writel(0x1, &nic301_regs->acp);
	writel(0x1, &nic301_regs->rom);
	writel(0x1, &nic301_regs->ocram);
	writel(0x1, &nic301_regs->sdrdata);
}

void board_init_f(ulong dummy)
{
	const struct cm_config *cm_default_cfg = cm_get_default_config();

	unsigned long sdram_size;
	unsigned long reg;

	/*
	 * First C code to run. Clear fake OCRAM ECC first as SBE
	 * and DBE might triggered during power on
	 */

	reg = readl(&sysmgr_regs->eccgrp_ocram);
	if (reg & SYSMGR_ECC_OCRAM_SERR)
		writel(SYSMGR_ECC_OCRAM_SERR | SYSMGR_ECC_OCRAM_EN,
		       &sysmgr_regs->eccgrp_ocram);
	if (reg & SYSMGR_ECC_OCRAM_DERR)
		writel(SYSMGR_ECC_OCRAM_DERR  | SYSMGR_ECC_OCRAM_EN,
		       &sysmgr_regs->eccgrp_ocram);


	//memset(__bss_start, 0, __bss_end - __bss_start);

	socfpga_nic301_slave_ns();

	/* Configure ARM MPU SNSAC register. */
	setbits_le32(&scu_regs->sacr, 0xfff);

	/* Remap SDRAM to 0x0 */
	writel(0x1, &nic301_regs->remap);	/* remap.mpuzero */
	writel(0x1, &pl310->pl310_addr_filter_start);

	//debug("Freezing all I/O banks\n");
	/* freeze all IO banks */
	sys_mgr_frzctrl_freeze_req();

	/* Put everything into reset but L4WD0. */
	socfpga_per_reset_all();
	/* Put FPGA bridges into reset too. */
	socfpga_bridges_reset(1);

	socfpga_per_reset(SOCFPGA_RESET(SDR), 0);
	socfpga_per_reset(SOCFPGA_RESET(UART0), 0);
	socfpga_per_reset(SOCFPGA_RESET(OSC1TIMER0), 0);

	timer_init();

	//debug("Reconfigure Clock Manager\n");
	/* reconfigure the PLLs */
	cm_basic_init(cm_default_cfg);

	/* Enable bootrom to configure IOs. */
	sysmgr_config_warmrstcfgio(1);

	/* configure the IOCSR / IO buffer settings */
	if (scan_mgr_configure_iocsr())
		hang();

	sysmgr_config_warmrstcfgio(0);

	/* configure the pin muxing through system manager */
	sysmgr_config_warmrstcfgio(1);
	sysmgr_pinmux_init();
	sysmgr_config_warmrstcfgio(0);

	/* De-assert reset for peripherals and bridges based on handoff */
	reset_deassert_peripherals_handoff();
	socfpga_bridges_reset(0);

	//debug("Unfreezing/Thaw all I/O banks\n");
	/* unfreeze / thaw all IO banks */
	sys_mgr_frzctrl_thaw_req();

	/* enable console uart printing */
	preloader_console_init();

	if (sdram_mmr_init_full(0xffffffff) != 0) {
		//puts("SDRAM init failed.\n");
		hang();
	}

	//debug("SDRAM: Calibrating PHY\n");
	/* SDRAM calibration */
	if (sdram_calibration_full() == 0) {
		//puts("SDRAM calibration failed.\n");
		hang();
	}

	sdram_size = sdram_calculate_size();
	//debug("SDRAM: %ld MiB\n", sdram_size >> 20);

	/* Sanity check ensure correct SDRAM size specified */
	if (get_ram_size(0, sdram_size) != sdram_size) {
		//puts("SDRAM size check failed!\n");
		hang();
	}

	socfpga_bridges_reset(1);

	/* Configure simple malloc base pointer into RAM. */
	//gd->malloc_base = CONFIG_SYS_TEXT_BASE + (1024 * 1024);
}
