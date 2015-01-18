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

#include <sys/systm.h>    // snprintf
#include <IOKit/assert.h>
#include <IOKit/IOMessage.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/storage/IOStorageProtocolCharacteristics.h>
#include "AppleATIATADriver.h"

#define super IOPCIATA
OSDefineMetaClassAndStructors( AppleATIATADriver, IOPCIATA )

#if     0
#define DEBUG_LOG(fmt, args...) kprintf(fmt, ## args)
#define ERROR_LOG(fmt, args...) kprintf(fmt, ## args)
#else
#define DEBUG_LOG(fmt, args...)
#define ERROR_LOG(fmt, args...) IOLog(fmt, ## args)
#endif

#ifdef FEDE_DEBUG
#define FEDE_LOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#define FEDE_RELOG(fmt,args...) IOLog(fmt, ## args); kprintf(fmt, ## args);
#else
#define FEDE_LOG(fmt,args...) 
#define FEDE_RELOG(fmt,args...)
#endif


#define kPIOModeMask   ((1 << kPIOModeCount) - 1)
#define kDMAModeMask   ((1 << kDMAModeCount) - 1)
#define kUDMAModeMask  (fProvider->getUltraDMAModeMask())

#define DRIVE_IS_PRESENT(u) \
        (_devInfo[u].type != kUnknownATADeviceType)

#define TIMING_PARAM_IS_VALID(p) \
        ((p) != 0)

// Increase the PRD table size to one full page or 4096 descriptors for
// large transfers via DMA.  2048 are required for 1 megabyte transfers
// assuming no fragmentation and no alignment issues on the buffer.  We
// allocate twice that since there are more issues than simple alignment
// for this DMA engine.

#define kATAXferDMADesc  512
#define kATAMaxDMADesc   kATAXferDMADesc

// up to 2048 ATA sectors per transfer

#define kMaxATAXfer      512 * 2048

/*---------------------------------------------------------------------------
 *
 * Start the single-channel ATI ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

bool AppleATIATADriver::start( IOService * provider )
{
    bool superStarted = false;

    DEBUG_LOG("%s: %s( %p, %p )\n", getName(), __FUNCTION__, this, provider);

    // Our provider is a 'nub' that represents a single channel PCI ATA
    // controller, and not an IOPCIDevice.

    fProvider = OSDynamicCast( AppleATIATAChannel, provider );
    if ( fProvider == 0 )
        goto fail;

    // Retain and open our provider.

    fProvider->retain();

    if ( fProvider->open( this ) != true )
    {
        DEBUG_LOG("%s: provider open failed\n", getName());
        goto fail;
    }

    // Create a work loop.

    fWorkLoop = IOWorkLoop::workLoop();
    if ( fWorkLoop == 0 )
    {
        DEBUG_LOG("%s: new work loop failed\n", getName());
        goto fail;
    }

    // Cache static controller properties.

    fChannelNumber = fProvider->getChannelNumber();
    if ( fChannelNumber > SEC_CHANNEL_ID )
    {
        DEBUG_LOG("%s: bad ATA channel number %ld\n", getName(),
                  fChannelNumber);
        goto fail;
    }

    // Probe for 80-pin conductors on drive 0 and 1.

    f80PinCable[0] = true;
    f80PinCable[1] = true;

    // Get the base address for the bus master registers in I/O space.

    if ( getBMBaseAddress( fChannelNumber, &fBMBaseAddr ) != true )
    {
        DEBUG_LOG("%s: invalid bus-master base address\n", getName());
        goto fail;
    }

    // Must setup these variables inherited from IOPCIATA before it is started.

    _bmCommandReg   = IOATAIOReg8::withAddress( fBMBaseAddr + BM_COMMAND );
    _bmStatusReg    = IOATAIOReg8::withAddress( fBMBaseAddr + BM_STATUS );
    _bmPRDAddresReg = IOATAIOReg32::withAddress( fBMBaseAddr + BM_PRD_TABLE );

    // Reset bus timings for both drives.

    initializeHardware();
    resetBusTimings();

    // Override P-ATA reporting in IOATAController::start()
    // for SystemProfiler.

    if (fProvider->getHardwareType() == ATI_HW_SATA)
    {
        setProperty( kIOPropertyPhysicalInterconnectTypeKey,
                     kIOPropertyPhysicalInterconnectTypeSerialATA );
    }


    // Now we are ready to call super::start

    if ( super::start(_provider) == false )
    {
        goto fail;
    }
    superStarted = true;

    // This driver will handle interrupts using a work loop.
    // Create interrupt event source that will signal the
    // work loop (thread) when a device interrupt occurs.

    if ( fProvider->getInterruptVector() == 14 ||
         fProvider->getInterruptVector() == 15 )
    {
        // Legacy IRQ are never shared, no need for an interrupt filter.

        fInterruptSource = IOInterruptEventSource::interruptEventSource(
                           this, &interruptOccurred,
                           fProvider, 0 );
    }
    else
    {
        fInterruptSource = IOFilterInterruptEventSource::filterInterruptEventSource(
                           this, &interruptOccurred, &interruptFilter,
                           fProvider, 0 );
    }

    if ( !fInterruptSource ||
         (fWorkLoop->addEventSource(fInterruptSource) != kIOReturnSuccess) )
    {
        DEBUG_LOG("%s: interrupt registration error\n", getName());
        goto fail;
    }
    fInterruptSource->enable();

    // Attach to power management.

    initForPM( provider );

    // For each device discovered on the ATA bus (by super),
    // create a nub for that device and call registerService() to
    // trigger matching against that device.

    for ( UInt32 i = 0; i < kMaxDriveCount; i++ )
    {
        if ( _devInfo[i].type != kUnknownATADeviceType )
        {
            ATADeviceNub * nub;

            nub = ATADeviceNub::ataDeviceNub( (IOATAController*) this,
                                              (ataUnitID) i,
                                              _devInfo[i].type );

            if ( nub )
            {
                if ( _devInfo[i].type == kATAPIDeviceType )
                {
                    nub->setProperty( kIOMaximumSegmentCountReadKey,
                                      kATAMaxDMADesc / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentCountWriteKey,
                                      kATAMaxDMADesc / 2, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountReadKey,
                                      0x10000, 64 );

                    nub->setProperty( kIOMaximumSegmentByteCountWriteKey,
                                      0x10000, 64 );
                }

                if ( nub->attach( this ) )
                {
                    _nub[i] = (IOATADevice *) nub;
                    _nub[i]->retain();
                    _nub[i]->registerService();
                }
                nub->release();
            }
        }
    }

    // Successful start, announce useful properties.

    IOLog("%s: ATI %s (CMD 0x%x, CTR 0x%x, IRQ %u, BM 0x%x)\n", getName(),
          fProvider->getHardwareName(),
          fProvider->getCommandBlockAddress(),
          fProvider->getControlBlockAddress(),
          (unsigned int)fProvider->getInterruptVector(),
          fBMBaseAddr);

    return true;

fail:
    if ( fProvider )
        fProvider->close( this );

    if ( superStarted )
        super::stop( provider );

    return false;
}

/*---------------------------------------------------------------------------
 *
 * Stop the single-channel ATI ATA controller driver.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::stop( IOService * provider )
{
    PMstop();
    super::stop( provider );
}

/*---------------------------------------------------------------------------
 *
 * Release resources before this driver is destroyed.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::free( void )
{
#define RELEASE(x) do { if(x) { (x)->release(); (x) = 0; } } while(0)

    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    // Release resources created by start().

    if (fInterruptSource && fWorkLoop)
    {
        fWorkLoop->removeEventSource(fInterruptSource);
    }

    RELEASE( fProvider        );
    RELEASE( fInterruptSource );
    RELEASE( fWorkLoop        );
    RELEASE( _nub[0]          );
    RELEASE( _nub[1]          );
    RELEASE( _bmCommandReg    );
    RELEASE( _bmStatusReg     );
    RELEASE( _bmPRDAddresReg  );

    // Release registers created by configureTFPointers().

    RELEASE( _tfDataReg       );
    RELEASE( _tfFeatureReg    );
    RELEASE( _tfSCountReg     );
    RELEASE( _tfSectorNReg    );
    RELEASE( _tfCylLoReg      );
    RELEASE( _tfCylHiReg      );
    RELEASE( _tfSDHReg        );
    RELEASE( _tfStatusCmdReg  );
    RELEASE( _tfAltSDevCReg   );

    // IOATAController should release this.

	if( _DMACursor != NULL )
	{
		_DMACursor->release();
		_DMACursor = NULL;
	}
	
	if( _workLoop != NULL )
	{
		_workLoop->release();
	}
	
    // What about _cmdGate, and _timer in the superclass?

    super::free();
}

/*---------------------------------------------------------------------------
 *
 * Return the driver's work loop
 *
 ---------------------------------------------------------------------------*/

IOWorkLoop * AppleATIATADriver::getWorkLoop( void ) const
{
    return fWorkLoop;
}

/*---------------------------------------------------------------------------
 *
 * Override IOATAController::synchronousIO()
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleATIATADriver::synchronousIO( void )
{
    IOReturn ret;
    
    // IOATAController::synchronousIO() asserts nIEN bit in order to disable
    // drive interrupts during polled mode command execution. The problem is
    // that this will float the INTRQ line and put it in high impedance state,
    // which on certain systems has the undesirable effect of latching a false
    // interrupt on the interrupt controller. Perhaps those systems lack a
    // strong pull down resistor on the INTRQ line. Experiment shows that the
    // interrupt event source is signalled, and its producerCount incremented
    // after every synchronousIO() call. This false interrupt can become
    // catastrophic after reverting to async operations since software can
    // issue a command, handle the false interrupt, and issue another command
    // to the drive before the actual completion of the first command, leading
    // to a irrecoverable bus hang. This function is called after an ATA bus
    // reset. Waking from system sleep will exercise this path.
    // The workaround is to mask the interrupt line while the INTRQ line is
    // floating (or bouncing).

    if (fInterruptSource) fInterruptSource->disable();
    ret = super::synchronousIO();
    if (fInterruptSource) fInterruptSource->enable();

    return ret;
}

/*---------------------------------------------------------------------------
 *
 * Determine the start of the I/O mapped Bus-Master registers.
 *
 ---------------------------------------------------------------------------*/

bool AppleATIATADriver::getBMBaseAddress( UInt32   channel,
                                          UInt16 * baseAddr )
{
    UInt32 bmiba;

    DEBUG_LOG("%s::%s( %p, %ld, %p )\n", getName(), __FUNCTION__,
              this, channel, baseAddr);

    bmiba = fProvider->pciConfigRead32( PCI_BMIBA );

    if ((bmiba & PCI_BMIBA_RTE) == 0)
    {
        DEBUG_LOG("%s: PCI BAR 0x%02x (0x%08lx) is not an I/O range\n",
                  getName(), PCI_BMIBA, bmiba);
        return false;
    }

    bmiba &= PCI_BMIBA_MASK;  // get the address portion
    if (bmiba == 0)
    {
        DEBUG_LOG("%s: BMIBA is zero\n", getName());
        return false;
    }

    if (channel == SEC_CHANNEL_ID)
        bmiba += BM_SEC_OFFSET;

    *baseAddr = (UInt16) bmiba;
    DEBUG_LOG("%s: BMBaseAddr = %04x\n", getName(), *baseAddr);

    return true;
}

/*---------------------------------------------------------------------------
 *
 * Reset all timing registers to the slowest (most compatible) timing.
 * DMA modes are disabled.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::resetBusTimings( void )
{
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    memset(&fBusTimings[0], 0, sizeof(fBusTimings));

    fBusTimings[0].pioTiming = &PIOTimingTable[0];
    fBusTimings[1].pioTiming = &PIOTimingTable[0];

    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Setup the location of the task file registers.
 *
 ---------------------------------------------------------------------------*/

bool AppleATIATADriver::configureTFPointers( void )
{
    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    UInt16 cmdBlockAddr = fProvider->getCommandBlockAddress();
    UInt16 ctrBlockAddr = fProvider->getControlBlockAddress();

    _tfDataReg      = IOATAIOReg16::withAddress( cmdBlockAddr + 0 );
    _tfFeatureReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 1 );
    _tfSCountReg    = IOATAIOReg8::withAddress(  cmdBlockAddr + 2 );
    _tfSectorNReg   = IOATAIOReg8::withAddress(  cmdBlockAddr + 3 );
    _tfCylLoReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 4 );
    _tfCylHiReg     = IOATAIOReg8::withAddress(  cmdBlockAddr + 5 );
    _tfSDHReg       = IOATAIOReg8::withAddress(  cmdBlockAddr + 6 );
    _tfStatusCmdReg = IOATAIOReg8::withAddress(  cmdBlockAddr + 7 );
    _tfAltSDevCReg  = IOATAIOReg8::withAddress(  ctrBlockAddr + 2 );

    if ( !_tfDataReg || !_tfFeatureReg || !_tfSCountReg ||
         !_tfSectorNReg || !_tfCylLoReg || !_tfCylHiReg ||
         !_tfSDHReg || !_tfStatusCmdReg || !_tfAltSDevCReg )
    {
        return false;
    }

    return true;
}

/*---------------------------------------------------------------------------
 *
 * Filter interrupts that are not originated by our hardware. This will help
 * prevent waking up our work loop thread when sharing a interrupt line with
 * another driver.
 *
 ---------------------------------------------------------------------------*/

bool AppleATIATADriver::interruptFilter( OSObject * owner,
                                         IOFilterInterruptEventSource * src )
{
    AppleATIATADriver * self = (AppleATIATADriver *) owner;

    if ( *(self->_bmStatusReg) & BM_STATUS_INT )
        return true;   // signal the work loop
    else
        return false;  // ignore this spurious interrupt
}

/*---------------------------------------------------------------------------
 *
 * The work loop based interrupt handler called by our interrupt event
 * source.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::interruptOccurred( OSObject *               owner,
                                           IOInterruptEventSource * source,
                                           int                      count )
{
    AppleATIATADriver * self = (AppleATIATADriver *) owner;

    // Clear interrupt latch

    *(self->_bmStatusReg) = BM_STATUS_INT;

    // Let our superclass handle the interrupt to advance to the next state
    // in the state machine.

    self->handleDeviceInterrupt();
}

/*---------------------------------------------------------------------------
 *
 * Extend the implementation of scanForDrives() from IOATAController
 * to issue a soft reset before scanning for ATA/ATAPI drive signatures.
 *
 ---------------------------------------------------------------------------*/

UInt32 AppleATIATADriver::scanForDrives( void )
{
    UInt32 unitsFound;

    DEBUG_LOG("%s::%s( %p )\n", getName(), __FUNCTION__, this);

    *_tfAltSDevCReg = mATADCRReset;

    IODelay( 100 );

    *_tfAltSDevCReg = 0x0;

    IOSleep( 10 );

    unitsFound = super::scanForDrives();

    *_tfSDHReg = 0x00;  // Initialize device selection to device 0.

    return unitsFound;
}

/*---------------------------------------------------------------------------
 *
 * Provide information on the ATA bus capability.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleATIATADriver::provideBusInfo( IOATABusInfo * infoOut )
{
    DEBUG_LOG("%s::%s( %p, %p )\n", getName(), __FUNCTION__, this, infoOut);

    if ( infoOut == 0 )
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    infoOut->zeroData();

    if (fProvider->getHardwareType() == ATI_HW_SATA)
        infoOut->setSocketType( kInternalSATA );
    else
        infoOut->setSocketType( kInternalATASocket );

    infoOut->setPIOModes( kPIOModeMask );
    infoOut->setDMAModes( kDMAModeMask );
    infoOut->setUltraModes( kUDMAModeMask );
    infoOut->setExtendedLBA( true );
    infoOut->setMaxBlocksExtended( 0x0800 );  // 2048 sectors for ext LBA

    UInt8 units = 0;
    if ( _devInfo[0].type != kUnknownATADeviceType ) units++;
    if ( _devInfo[1].type != kUnknownATADeviceType ) units++;
    infoOut->setUnits( units );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Returns the currently configured timings for the drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleATIATADriver::getConfig( IOATADevConfig * configOut,
                                       UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %ld )\n", getName(), __FUNCTION__,
              this, configOut, unit);

    if ((configOut == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    configOut->setPIOMode( 0 );
    configOut->setDMAMode( 0 );
    configOut->setUltraMode( 0 );

    // Note that we need to report the bitmap of each mode,
    // not its mode number.

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].pioTiming))
    {
        configOut->setPIOMode( 1 << fBusTimings[unit].pioModeNumber );
        configOut->setPIOCycleTime( fBusTimings[unit].pioTiming->cycleTimeNS );
    }

    if (TIMING_PARAM_IS_VALID(fBusTimings[unit].dmaTiming))
    {
        configOut->setDMAMode( 1 << fBusTimings[unit].dmaModeNumber );
        configOut->setDMACycleTime( fBusTimings[unit].dmaTiming->cycleTimeNS );
    }

    if (fBusTimings[unit].ultraEnabled)
    {
        configOut->setUltraMode( 1 << fBusTimings[unit].ultraModeNumber );
    }

    configOut->setPacketConfig( _devInfo[unit].packetSend );

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Select the bus timings for a given drive unit.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleATIATADriver::selectConfig( IOATADevConfig * configRequest,
                                          UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %p, %ld )\n", getName(), __FUNCTION__,
              this, configRequest, unit);

    if ((configRequest == 0) || (unit > kATADevice1DeviceID))
    {
        DEBUG_LOG("%s: %s bad argument\n", getName(), __FUNCTION__);
        return -1;
    }

    // All config requests must include a supported PIO mode

    if ((configRequest->getPIOMode() & kPIOModeMask) == 0)
    {
        DEBUG_LOG("%s: PIO mode unsupported\n", getName());
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() & ~kDMAModeMask)
    {
        DEBUG_LOG("%s: DMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getDMAMode());
        return kATAModeNotSupported;
    }

    if (configRequest->getUltraMode() & ~kUDMAModeMask)
    {
        DEBUG_LOG("%s: UDMA mode unsupported (0x%x)\n",
                  getName(), configRequest->getUltraMode());
        return kATAModeNotSupported;
    }

    if (configRequest->getDMAMode() && configRequest->getUltraMode())
    {
        DEBUG_LOG("%s: multiple DMA mode selection error\n", getName());
        return kATAModeNotSupported;
    }

    _devInfo[unit].packetSend = configRequest->getPacketConfig();

    selectTimingParameter( configRequest, unit );

    return getConfig( configRequest, unit );
}

/*---------------------------------------------------------------------------
 *
 * Select timing parameters based on config request.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::selectTimingParameter( IOATADevConfig * configRequest,
                                               UInt32           unit )
{
    DEBUG_LOG("%s::%s( %p, %d )\n", getName(), __FUNCTION__, this, unit);

    // Reset existing parameters for this unit.

    fBusTimings[unit].pioTiming = 0;
    fBusTimings[unit].dmaTiming = 0;
    fBusTimings[unit].ultraEnabled = false;

    if ( configRequest->getPIOMode() )
    {
        UInt32  pioModeNumber;
        UInt32  pioCycleTime;
        UInt32  pioTimingEntry = 0;

        pioModeNumber = bitSigToNumeric( configRequest->getPIOMode() );
        pioModeNumber = min(pioModeNumber, kPIOModeCount - 1);

        // Use a default cycle time if the device didn't report a time to use.
    
        pioCycleTime = configRequest->getPIOCycleTime();
        pioCycleTime = max(pioCycleTime, PIOTimingTable[pioModeNumber].cycleTimeNS);

        // Look for the fastest entry in the PIOTimingTable with a cycle time
        // which is larger than or equal to pioCycleTime.
    
        for (int i = kPIOModeCount - 1; i > 0; i--)
        {
            if ( PIOTimingTable[i].cycleTimeNS >= pioCycleTime )
            {
                pioTimingEntry = i;
                break;
            }
        }

        fBusTimings[unit].pioTiming = &PIOTimingTable[pioTimingEntry];
        fBusTimings[unit].pioModeNumber = pioModeNumber;
        DEBUG_LOG("%s: selected PIO mode %d\n", getName(), pioModeNumber);
        setDriveProperty(unit, kSelectedPIOModeKey, pioModeNumber, 8);
    }

    if ( configRequest->getDMAMode() )
    {
        UInt32  dmaModeNumber;
        UInt32  dmaCycleTime;
        UInt32  dmaTimingEntry = 0;

        dmaModeNumber = bitSigToNumeric( configRequest->getDMAMode() );
        dmaModeNumber = min(dmaModeNumber, kDMAModeCount - 1);

        dmaCycleTime = configRequest->getDMACycleTime();
        dmaCycleTime = max(dmaCycleTime, DMATimingTable[dmaModeNumber].cycleTimeNS);

        // Look for the fastest entry in the DMATimingTable with a cycle time
        // which is larger than or equal to dmaCycleTime.
    
        for (int i = kDMAModeCount - 1; i > 0; i--)
        {
            if ( DMATimingTable[i].cycleTimeNS >= dmaCycleTime )
            {
                dmaTimingEntry = i;
                break;
            }
        }
        
        fBusTimings[unit].dmaTiming = &DMATimingTable[dmaTimingEntry];
        fBusTimings[unit].dmaModeNumber = dmaModeNumber;
        DEBUG_LOG("%s: selected DMA mode %d\n", getName(), dmaModeNumber);
        setDriveProperty(unit, kSelectedDMAModeKey, dmaModeNumber, 8);
    }

    if ( configRequest->getUltraMode() )
    {
        UInt32  ultraModeNumber;

        ultraModeNumber = bitSigToNumeric( configRequest->getUltraMode() );
        ultraModeNumber = min(ultraModeNumber, kUDMAModeCount - 1);

        // For Ultra DMA mode 3 or higher, 80 pin cable must be present.
        // Otherwise, the drive will be limited to UDMA mode 2.

        if ( fProvider->getHardwareType() != ATI_HW_SATA &&
             ultraModeNumber > 2 )
        {
            if ( f80PinCable[unit] == false )
            {
                DEBUG_LOG("%s: 80-conductor cable not detected\n", getName());
                ultraModeNumber = 2;
            }
        }

        fBusTimings[unit].ultraEnabled = true;
        fBusTimings[unit].ultraModeNumber = ultraModeNumber;
        DEBUG_LOG("%s: selected Ultra mode %d\n", getName(), ultraModeNumber);
        setDriveProperty(unit, kSelectedUltraDMAModeKey, ultraModeNumber, 8);
    }

    programTimingRegisters();
}

/*---------------------------------------------------------------------------
 *
 * Program timing registers for both drives.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::programTimingRegisters( void )
{
    if (fProvider->getHardwareType() == ATI_HW_SATA) return;

    UInt8   ultraEnable   = 0;
    UInt8   ultraMode     = 0;
    UInt8   pioMode       = 0;
    UInt32  dmaTiming     = 0;
    UInt32  pioTiming     = 0;

    for (int unit = 0; unit < kMaxDriveCount; unit++)
    {
        UInt32 dmaTimingValue;
        int    globalDriveIndex = unit + (fChannelNumber * 2);

        if (fBusTimings[unit].ultraEnabled)
        {
            ultraEnable |= (1 << globalDriveIndex);
            ultraMode |= (fBusTimings[unit].ultraModeNumber << (4 * unit));
            dmaTimingValue = DMATimingTable[2].timingValue;            
        }
        else if (TIMING_PARAM_IS_VALID(fBusTimings[unit].dmaTiming))
        {
            dmaTimingValue = fBusTimings[unit].dmaTiming->timingValue;
        }
        else
        {
            dmaTimingValue = DMATimingTable[0].timingValue;
        }

        dmaTiming |= (dmaTimingValue <<
                      (((1 ^ unit) + (2 * fChannelNumber)) * 8));

        pioMode |= (fBusTimings[unit].pioTiming->modeNumber << (4 * unit));
        pioTiming |= ((UInt32)(fBusTimings[unit].pioTiming->timingValue) <<
                      (((1 ^ unit) + (2 * fChannelNumber)) * 8));
    }

    // Apply the timing changes for this channel only.

    fProvider->pciConfigWrite8( PCI_PIO_MODE + fChannelNumber, pioMode );

    fProvider->pciConfigWrite32( PCI_PIO_TIMING, pioTiming,
                                 fChannelNumber ? 0xFFFF0000 : 0x0000FFFF );

    fProvider->pciConfigWrite32( PCI_DMA_TIMING, dmaTiming,
                                 fChannelNumber ? 0xFFFF0000 : 0x0000FFFF );

    fProvider->pciConfigWrite8( PCI_ULTRA_ENABLE,
                                 ultraEnable, fChannelNumber ? 0x0C : 0x03 );

    fProvider->pciConfigWrite8( PCI_ULTRA_MODE + fChannelNumber, ultraMode );
}

/*---------------------------------------------------------------------------
 *
 * Hardware initialization.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::initializeHardware( void )
{

}

/*---------------------------------------------------------------------------
 *
 * Dynamically select the bus timings for a drive unit.
 *
 ---------------------------------------------------------------------------*/

void AppleATIATADriver::selectIOTiming( ataUnitID unit )
{
    /* Timings was already applied by selectConfig() */
}

/*---------------------------------------------------------------------------
 *
 * Flush the outstanding commands in the command queue.
 * Implementation borrowed from MacIOATA in IOATAFamily.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleATIATADriver::handleQueueFlush( void )
{
    UInt32 savedQstate = _queueState;

    DEBUG_LOG("%s::%s()\n", getName(), __FUNCTION__);

    _queueState = IOATAController::kQueueLocked;

    IOATABusCommand * cmdPtr = 0;

    while ( (cmdPtr = dequeueFirstCommand()) )
    {
        cmdPtr->setResult( kIOReturnError );
        cmdPtr->executeCallback();
    }

    _queueState = savedQstate;

    return kATANoErr;
}

/*---------------------------------------------------------------------------
 *
 * Handle termination notification from the provider.
 *
 ---------------------------------------------------------------------------*/

IOReturn AppleATIATADriver::message( UInt32      type,
                                      IOService * provider,
                                      void *      argument )
{
    if ( ( provider == fProvider ) &&
         ( type == kIOMessageServiceIsTerminated ) )
    {
        fProvider->close( this );
        return kIOReturnSuccess;
    }

    return super::message( type, provider, argument );
}

/*---------------------------------------------------------------------------
 *
 * Publish a numeric property pertaining to a drive to the registry.
 *
 ---------------------------------------------------------------------------*/

bool AppleATIATADriver::setDriveProperty( UInt32       driveUnit,
                                          const char * key,
                                          UInt32       value,
                                          UInt32       numberOfBits)
{
    char keyString[40];
    
    snprintf(keyString, 40, "Drive %u %s", (unsigned int)driveUnit, key);
    
    return super::setProperty( keyString, value, numberOfBits );
}

//---------------------------------------------------------------------------

IOReturn AppleATIATADriver::createChannelCommands( void )
{
	FEDE_LOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);
	FEDE_RELOG("FEDE - %s: %s( %p)\n", getName(), __FUNCTION__, this);

	FEDE_RELOG("FEDE Casteo a IOAABUSCommand64\n");
	IOATABusCommand64* currentCommand64 = OSDynamicCast( IOATABusCommand64, _currentCommand );
	FEDE_RELOG("FEDE Obtengo el comaqndo DMA\n ");
	IODMACommand* currentDMACmd = currentCommand64->GetDMACommand();
    IODMACommand::Segment32 elSegmento;
	IOReturn DMAStatus = 0;
	

	if ( NULL == currentDMACmd
		|| currentDMACmd->getMemoryDescriptor() == NULL)
    {
        FEDE_RELOG("FEDE DMA BUFFER NOT SET ON COMMAND\n ");
		IOLog("%s: DMA buffer not set on command\n", getName());
		return -1;
    }


	FEDE_RELOG("FEDE obtengo el descriptor de memoria\n ");
    IOMemoryDescriptor* descriptor = _currentCommand->getBuffer();
    IOMemoryCursor::PhysicalSegment physSegment;
    UInt32 index = 0;
    UInt8  *xferDataPtr, *ptr2EndData, *next64KBlock, *starting64KBlock;
    UInt32 xferCount, count2Next64KBlock;
    
    if ( !descriptor )
    {
        return -1;
    }

	
	   // This form of DMA engine can only do 1 pass.
    // It cannot execute multiple chains.

	UInt32 numSegmentos = 1;

    IOByteCount bytesRemaining = _currentCommand->getByteCount() ;
    IOByteCount xfrPosition    = _currentCommand->getPosition() ;
    UInt64  transferSize  = 0; 

    // There's a unique problem with pci-style controllers, in that each
    // dma transaction is not allowed to cross a 64K boundary. This leaves
    // us with the yucky task of picking apart any descriptor segments that
    // cross such a boundary ourselves.  
//
//    while ( _DMACursor->getPhysicalSegments(
//                           /* descriptor */ descriptor,
//							/* position   */ xfrPosition,
//                           /* segments   */ &physSegment,
//                           /* max segs   */ 1,
//                           /* max xfer   */ bytesRemaining,
//                           /* xfer size  */ &transferSize) )
//
	FEDE_RELOG("FEDE entro al ciclo\n");
	while ( bytesRemaining )
	{
		FEDE_RELOG("FEDE genero 32IOVMSegments\n");
		DMAStatus = currentDMACmd->gen32IOVMSegments( &transferSize, &elSegmento, &numSegmentos);
		if ( ( DMAStatus != kIOReturnSuccess ) || ( numSegmentos != 1 ) || ( elSegmento.fLength == 0 ) )
		{
			
			panic ( "AppleATIATA::createChannelCommands [%d] status %x segs %u phys %x:%x \n", __LINE__, DMAStatus, (unsigned int)numSegmentos, (unsigned int)elSegmento.fIOVMAddr, (unsigned int)elSegmento.fLength );
		    break;
		    
		}

        xferDataPtr = (UInt8 *) ((UInt64)elSegmento.fIOVMAddr);
        xferCount   = elSegmento.fLength;

        if ( (uintptr_t) xferDataPtr & 0x01 )
        {
            IOLog("%s: DMA buffer %p not 2 byte aligned\n",
                  getName(), xferDataPtr);
            return kIOReturnNotAligned;        
        }

        if ( xferCount & 0x01 )
        {
            IOLog("%s: DMA buffer length %u is odd\n",
                  getName(), (unsigned int)xferCount);
        }

        // Update bytes remaining count after this pass.
        bytesRemaining -= xferCount;
        xfrPosition += xferCount;
            
        // Examine the segment to see whether it crosses (a) 64k boundary(s)
        starting64KBlock = (UInt8*) ( (uintptr_t) xferDataPtr & 0xffff0000);
        ptr2EndData  = xferDataPtr + xferCount;
        next64KBlock = starting64KBlock + 0x10000;

        // Loop until this physical segment is fully accounted for.
        // It is possible to have a memory descriptor which crosses more
        // than one 64K boundary in a single span.
        
        while ( xferCount > 0 )
        {
            if (ptr2EndData > next64KBlock)
            {
                count2Next64KBlock = next64KBlock - xferDataPtr;
                if ( index < kATAMaxDMADesc )
                {
                    setPRD( xferDataPtr, (UInt16)count2Next64KBlock,
                            &_prdTable[index], kContinue_PRD);
                    
                    xferDataPtr = next64KBlock;
                    next64KBlock += 0x10000;
                    xferCount -= count2Next64KBlock;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 1\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
            else
            {
                if (index < kATAMaxDMADesc)
                {
                    setPRD( xferDataPtr, (UInt16) xferCount,
                            &_prdTable[index],
                            (bytesRemaining == 0) ? kLast_PRD : kContinue_PRD);
                    xferCount = 0;
                    index++;
                }
                else
                {
                    IOLog("%s: PRD table exhausted error 2\n", getName());
                    _dmaState = kATADMAError;
                    return -1;
                }
            }
        }
    } // end of segment counting loop.

    if (index == 0)
    {
        IOLog("%s: rejected command with zero PRD count (0x%x bytes)\n",
              getName(), (unsigned int)_currentCommand->getByteCount());
        return kATADeviceError;
    }

    // Transfer is satisfied and only need to check status on interrupt.
    _dmaState = kATADMAStatus;
    
    // Chain is now ready for execution.
    return kATANoErr;
}
//---------------------------------------------------------------------------

bool AppleATIATADriver::allocDMAChannel( void )
{
	_prdBuffer = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(
		kernel_task,
		kIODirectionInOut | kIOMemoryPhysicallyContiguous,
		sizeof(PRD) * kATAMaxDMADesc,
		0xFFFF0000UL );
    
    if ( !_prdBuffer )
    {
        IOLog("%s: PRD buffer allocation failed\n", getName());
        return false;
    }

	_prdBuffer->prepare ( );
	
	_prdTable			= (PRD *) _prdBuffer->getBytesNoCopy();
	_prdTablePhysical	= _prdBuffer->getPhysicalAddress();

    _DMACursor = IONaturalMemoryCursor::withSpecification(
                          /* max segment size  */ 0x10000,
                          /* max transfer size */ kMaxATAXfer );
    
    if ( !_DMACursor )
    {
        freeDMAChannel();
        IOLog("%s: Memory cursor allocation failed\n", getName());
        return false;
    }

    // fill the chain with stop commands to initialize it.    
    initATADMAChains( _prdTable );

    return true;
}

//---------------------------------------------------------------------------

bool AppleATIATADriver::freeDMAChannel( void )
{
    if ( _prdBuffer )
    {
        // make sure the engine is stopped.
        stopDMA();

        // free the descriptor table.
		_prdBuffer->complete();
        _prdBuffer->release();
        _prdBuffer = NULL;
        _prdTable = NULL;
        _prdTablePhysical = 0;
    }

    return true;
}

//---------------------------------------------------------------------------

void AppleATIATADriver::initATADMAChains( PRD * descPtr )
{
    UInt32 i;

    /* Initialize the data-transfer PRD channel command descriptors. */

    for (i = 0; i < kATAMaxDMADesc; i++)
    {
        descPtr->bufferPtr = 0;
        descPtr->byteCount = 1;
        descPtr->flags = OSSwapHostToLittleConstInt16( kLast_PRD );
        descPtr++;
    }
}

//---------------------------------------------------------------------------

enum {
    kATIPowerStateOff = 0,
    kATIPowerStateDoze,
    kATIPowerStateOn,
    kATIPowerStateCount
};

void AppleATIATADriver::initForPM( IOService * provider )
{
    static const IOPMPowerState powerStates[ kATIPowerStateCount ] =
    {
        { 1, 0, 0,             0,             0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMSoftSleep, IOPMSoftSleep, 0, 0, 0, 0, 0, 0, 0, 0 },
        { 1, 0, IOPMPowerOn,   IOPMPowerOn,   0, 0, 0, 0, 0, 0, 0, 0 }
    };

    PMinit();

    registerPowerDriver( this, (IOPMPowerState *) powerStates,
                         kATIPowerStateCount );

    provider->joinPMtree( this );
}

//---------------------------------------------------------------------------

IOReturn AppleATIATADriver::setPowerState( unsigned long stateIndex,
                                           IOService *   whatDevice )
{
    if ( stateIndex == kATIPowerStateOff )
    {
        fHardwareLostContext = true;
    }
    else if ( fHardwareLostContext )
    {
        initializeHardware();
        programTimingRegisters();
        fHardwareLostContext = false;
    }

    return IOPMAckImplied;
}
