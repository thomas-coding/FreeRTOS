/*
 * Copyright (c) 2021-2031, Jinping Wu. All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef GICV2_H
#define GICV2_H

/* following defines should be used for structure members */
#define     __IM     volatile const      /*! Defines 'read only' structure member permissions */
#define     __OM     volatile            /*! Defines 'write only' structure member permissions */
#define     __IOM    volatile            /*! Defines 'read / write' structure member permissions */
#define RESERVED(N, T) T RESERVED##N;    // placeholder struct members used for "reserved" areas

#ifndef uint32_t
typedef unsigned int uint32_t;
#endif

#ifndef uint8_t
typedef unsigned char uint8_t;
#endif

#ifndef uint64_t
typedef unsigned long long int uint64_t;
#endif
    
/** \brief  Structure type to access the Generic Interrupt Controller Distributor (GICD)
*/
typedef struct
{
  __IOM uint32_t CTLR;                 /*!< \brief  Offset: 0x000 (R/W) Distributor Control Register */
  __IM  uint32_t TYPER;                /*!< \brief  Offset: 0x004 (R/ ) Interrupt Controller Type Register */
  __IM  uint32_t IIDR;                 /*!< \brief  Offset: 0x008 (R/ ) Distributor Implementer Identification Register */
        RESERVED(0, uint32_t)
  __IOM uint32_t STATUSR;              /*!< \brief  Offset: 0x010 (R/W) Error Reporting Status Register, optional */
        RESERVED(1[11], uint32_t)
  __OM  uint32_t SETSPI_NSR;           /*!< \brief  Offset: 0x040 ( /W) Set SPI Register */
        RESERVED(2, uint32_t)
  __OM  uint32_t CLRSPI_NSR;           /*!< \brief  Offset: 0x048 ( /W) Clear SPI Register */
        RESERVED(3, uint32_t)
  __OM  uint32_t SETSPI_SR;            /*!< \brief  Offset: 0x050 ( /W) Set SPI, Secure Register */
        RESERVED(4, uint32_t)
  __OM  uint32_t CLRSPI_SR;            /*!< \brief  Offset: 0x058 ( /W) Clear SPI, Secure Register */
        RESERVED(5[9], uint32_t)
  __IOM uint32_t IGROUPR[32];          /*!< \brief  Offset: 0x080 (R/W) Interrupt Group Registers */
  __IOM uint32_t ISENABLER[32];        /*!< \brief  Offset: 0x100 (R/W) Interrupt Set-Enable Registers */
  __IOM uint32_t ICENABLER[32];        /*!< \brief  Offset: 0x180 (R/W) Interrupt Clear-Enable Registers */
  __IOM uint32_t ISPENDR[32];          /*!< \brief  Offset: 0x200 (R/W) Interrupt Set-Pending Registers */
  __IOM uint32_t ICPENDR[32];          /*!< \brief  Offset: 0x280 (R/W) Interrupt Clear-Pending Registers */
  __IOM uint32_t ISACTIVER[32];        /*!< \brief  Offset: 0x300 (R/W) Interrupt Set-Active Registers */
  __IOM uint32_t ICACTIVER[32];        /*!< \brief  Offset: 0x380 (R/W) Interrupt Clear-Active Registers */
  __IOM uint32_t IPRIORITYR[255];      /*!< \brief  Offset: 0x400 (R/W) Interrupt Priority Registers */
        RESERVED(6, uint32_t)
  __IOM uint32_t  ITARGETSR[255];      /*!< \brief  Offset: 0x800 (R/W) Interrupt Targets Registers */
        RESERVED(7, uint32_t)
  __IOM uint32_t ICFGR[64];            /*!< \brief  Offset: 0xC00 (R/W) Interrupt Configuration Registers */
  __IOM uint32_t IGRPMODR[32];         /*!< \brief  Offset: 0xD00 (R/W) Interrupt Group Modifier Registers */
        RESERVED(8[32], uint32_t)
  __IOM uint32_t NSACR[64];            /*!< \brief  Offset: 0xE00 (R/W) Non-secure Access Control Registers */
  __OM  uint32_t SGIR;                 /*!< \brief  Offset: 0xF00 ( /W) Software Generated Interrupt Register */
        RESERVED(9[3], uint32_t)
  __IOM uint32_t CPENDSGIR[4];         /*!< \brief  Offset: 0xF10 (R/W) SGI Clear-Pending Registers */
  __IOM uint32_t SPENDSGIR[4];         /*!< \brief  Offset: 0xF20 (R/W) SGI Set-Pending Registers */
        RESERVED(10[5236], uint32_t)
  __IOM uint64_t IROUTER[988];         /*!< \brief  Offset: 0x6100(R/W) Interrupt Routing Registers */
}  GICDistributor_Type;

/* From qemu hw/arm/virt.c */
#define GIC_DISTRIBUTOR_BASE	(0x08000000UL)
#define GICDistributor      ((GICDistributor_Type      *)     GIC_DISTRIBUTOR_BASE ) /*!< \brief GIC Distributor register set access pointer */

/** \brief  Structure type to access the Generic Interrupt Controller Interface (GICC)
*/
typedef struct
{
  __IOM uint32_t CTLR;                 /*!< \brief  Offset: 0x000 (R/W) CPU Interface Control Register */
  __IOM uint32_t PMR;                  /*!< \brief  Offset: 0x004 (R/W) Interrupt Priority Mask Register */
  __IOM uint32_t BPR;                  /*!< \brief  Offset: 0x008 (R/W) Binary Point Register */
  __IM  uint32_t IAR;                  /*!< \brief  Offset: 0x00C (R/ ) Interrupt Acknowledge Register */
  __OM  uint32_t EOIR;                 /*!< \brief  Offset: 0x010 ( /W) End Of Interrupt Register */
  __IM  uint32_t RPR;                  /*!< \brief  Offset: 0x014 (R/ ) Running Priority Register */
  __IM  uint32_t HPPIR;                /*!< \brief  Offset: 0x018 (R/ ) Highest Priority Pending Interrupt Register */
  __IOM uint32_t ABPR;                 /*!< \brief  Offset: 0x01C (R/W) Aliased Binary Point Register */
  __IM  uint32_t AIAR;                 /*!< \brief  Offset: 0x020 (R/ ) Aliased Interrupt Acknowledge Register */
  __OM  uint32_t AEOIR;                /*!< \brief  Offset: 0x024 ( /W) Aliased End Of Interrupt Register */
  __IM  uint32_t AHPPIR;               /*!< \brief  Offset: 0x028 (R/ ) Aliased Highest Priority Pending Interrupt Register */
  __IOM uint32_t STATUSR;              /*!< \brief  Offset: 0x02C (R/W) Error Reporting Status Register, optional */
        RESERVED(1[40], uint32_t)
  __IOM uint32_t APR[4];               /*!< \brief  Offset: 0x0D0 (R/W) Active Priority Register */
  __IOM uint32_t NSAPR[4];             /*!< \brief  Offset: 0x0E0 (R/W) Non-secure Active Priority Register */
        RESERVED(2[3], uint32_t)
  __IM  uint32_t IIDR;                 /*!< \brief  Offset: 0x0FC (R/ ) CPU Interface Identification Register */
        RESERVED(3[960], uint32_t)
  __OM  uint32_t DIR;                  /*!< \brief  Offset: 0x1000( /W) Deactivate Interrupt Register */
}  GICInterface_Type;

#define GIC_INTERFACE_BASE	(0x08010000UL)
#define GICInterface        ((GICInterface_Type        *)     GIC_INTERFACE_BASE )   /*!< \brief GIC Interface register set access pointer */


#define INTR_ENTRY_NUM	151

typedef void (*isr_handler_t)(uint32_t, void *);

enum isr_type_t {
	ISR_TYPE_IRQ = 0,
	ISR_TYPE_FIQ = 1
};

/* struct israction_s - per interrupt action descriptor */
struct israction_s {
	isr_handler_t handler;	/* interrupt handler function */
	enum isr_type_t	isr_type;	/* interrupt type */
	void *dev_id;	/* used to identify the device */
};

void gicv2_init(void);
void request_irq(uint32_t irq_num, enum isr_type_t isr_type,
	isr_handler_t cur_src_isr, void *dev_id);
#endif
