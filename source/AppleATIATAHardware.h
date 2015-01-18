/*
 * Copyright (c) 2004 Apple Computer, Inc. All rights reserved.
 *
 * @APPLE_LICENSE_HEADER_START@
 * 
 * The contents of this file constitute Original Code as defined in and
 * are subject to the Apple Public Source License Version 1.1 (the
 * "License").  You may not use this file except in compliance with the
 * License.  Please obtain a copy of the License at
 * http://www.apple.com/publicsource and read it before using this file.
 * 
 * This Original Code and all software distributed under the License are
 * distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND APPLE HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE OR NON-INFRINGEMENT.  Please see the
 * License for the specific language governing rights and limitations
 * under the License.
 * 
 * @APPLE_LICENSE_HEADER_END@
 */

#ifndef _APPLEATIATAHARDWARE_H
#define _APPLEATIATAHARDWARE_H

#define kMaxDriveCount      2
#define kMaxChannelCount    2

/*
 * I/O port addresses for primary and secondary channels.
 */
#define PRI_CMD_ADDR        0x1f0
#define PRI_CTR_ADDR        0x3f4
#define SEC_CMD_ADDR        0x170
#define SEC_CTR_ADDR        0x374

/*
 * IRQ assigned to primary and secondary channels.
 */
#define PRI_ISA_IRQ         14
#define SEC_ISA_IRQ         15

/*
 * Two ATA channels max.
 */
#define PRI_CHANNEL_ID      0
#define SEC_CHANNEL_ID      1

/*
 * PCI ATA config space registers.
 * Register size (bits) in parenthesis.
 */
#define PCI_CFID            0x00    // (32) PCI Device/Vendor ID
#define PCI_PCICMD          0x04    // (16) PCI command register
#define PCI_PCISTS          0x06    // (16) PCI device status register
#define PCI_RID             0x08    // (8)  Revision ID register
#define PCI_PI              0x09    // (8)  Programming interface
#define PCI_MLT             0x0d    // (8)  Master latency timer register
#define PCI_HEDT            0x0e    // (8)  Header type register
#define PCI_BMIBA           0x20    // (32) Bus-Master base address

#define PCI_BMIBA_RTE       0x01    // resource type indicator (I/O)
#define PCI_BMIBA_MASK      0xfff0  // base address mask

#define PCI_PCICMD_IOSE     0x01    // I/O space enable
#define PCI_PCICMD_BME      0x04    // bus-master enable

/*
 * ATI specific PCI config space registers.
 */
#define PCI_PIO_TIMING      0x40
#define PCI_DMA_TIMING      0x44
#define PCI_PIO_ENABLE      0x48
#define PCI_PIO_MODE        0x4a
#define PCI_ULTRA_ENABLE    0x54
#define PCI_ULTRA_MODE      0x56

/*
 * Supported devices.
 */
#define ATA_ATI_ID          0x1002
#define ATA_ATI_IXP200      0x43491002
#define ATA_ATI_IXP300      0x43691002
#define ATA_ATI_IXP300_S1   0x436e1002
#define ATA_ATI_IXP400      0x43761002
#define ATA_ATI_IXP400_S1   0x43791002
#define ATA_ATI_IXP400_S2   0x437a1002
#define ATA_ATI_IXP600      0x438c1002
#define ATA_ATI_IXP600_S1   0x43801002
#define ATA_ATI_IXP700      0x439c1002
#define ATA_ATI_IXP700_S1   0x43901002

/*
 * Bus master registers are located in I/O space.
 * Register size (bits) indicated in parenthesis.
 *
 * Note:
 * For the primary channel, the base address is stored in PCI_BMIBA.
 * For the secondary channel, an offset (BM_SEC_OFFSET) is added to
 * the value stored in PCI_BMIBA.
 */
#define BM_COMMAND          0x00    // (8) Bus master command register
#define BM_STATUS           0x02    // (8) Bus master status register
#define BM_PRD_TABLE        0x04    // (32) Descriptor table register

#define BM_SEC_OFFSET       0x08    // offset to channel 1 registers
#define BM_ADDR_MASK        0xfff0  // BMIBA mask to get I/O base address
#define BM_STATUS_INT       0x04    // IDE device asserted its interrupt

/*
 * Enumeration of ATI hardware types.
 */
enum {
    ATI_HW_UDMA_NONE = 0,
    ATI_HW_UDMA_33,
    ATI_HW_UDMA_66,
    ATI_HW_UDMA_100,
    ATI_HW_UDMA_133,
    ATI_HW_SATA,
    ATI_HW_COUNT
};

/*
 * Property keys
 */
#define kChannelNumberKey         "Channel Number"
#define kCommandBlockAddressKey   "Command Block Address"
#define kControlBlockAddressKey   "Control Block Address"
#define kInterruptVectorKey       "Interrupt Vector"
#define kSerialATAKey             "Serial ATA"
#define kHardwareNameKey          "Hardware Name"
#define kSelectedPIOModeKey       "PIO Mode"
#define kSelectedDMAModeKey       "DMA Mode"
#define kSelectedUltraDMAModeKey  "Ultra DMA Mode"
#define kISABridgeMatchingKey     "ISA Bridge Matching"

#endif /* !_APPLEATIATAHARDWARE_H */
