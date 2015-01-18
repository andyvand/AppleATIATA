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

#ifndef _APPLEATIATADRIVER_H
#define _APPLEATIATADRIVER_H

#define UnsignedWide UInt64

#include <IOKit/IOFilterInterruptEventSource.h>
#include <IOKit/IOBufferMemoryDescriptor.h>
#include <IOKit/ata/IOATATypes.h>
#include <IOKit/ata/IOPCIATA.h>
#include <IOKit/ata/IOATAController.h>
#include <IOKit/ata/ATADeviceNub.h>
#include "AppleATIATAHardware.h"
#include "AppleATIATAChannel.h"
#include "AppleATIATATiming.h"

struct ATABusTimings
{
    UInt8                      pioModeNumber;
    const TimingParameter *    pioTiming;
    UInt8                      dmaModeNumber;
    const TimingParameter *    dmaTiming;
    UInt8                      ultraModeNumber;
    bool                       ultraEnabled;
};

class AppleATIATADriver : public IOPCIATA
{
    OSDeclareDefaultStructors( AppleATIATADriver )

protected:
    AppleATIATAChannel *       fProvider;
    IOInterruptEventSource *   fInterruptSource;
    IOWorkLoop *               fWorkLoop;
    ATABusTimings              fBusTimings[ kMaxDriveCount ];
    bool                       f80PinCable[ kMaxDriveCount ];
    UInt16                     fBMBaseAddr;
    UInt32                     fChannelNumber;
    bool                       fHardwareLostContext;
	IOBufferMemoryDescriptor*  _prdBuffer;

    /* Interrupt event source action */

    static void      interruptOccurred( OSObject *               owner,
                                        IOInterruptEventSource * src,
                                        int                      count );

    /* Interrupt event source filter */

    static bool      interruptFilter( OSObject * owner,
                                      IOFilterInterruptEventSource * src );

    /* Driver functions */

    virtual bool     getBMBaseAddress( UInt32   channel,
                                       UInt16 * baseAddr );

    virtual void     resetBusTimings( void );

    virtual void     selectIOTiming( ataUnitID unit );

    virtual bool     setDriveProperty( UInt32       driveUnit,
                                       const char * key,
                                       UInt32       value,
                                       UInt32       numberOfBits);

    virtual IOReturn synchronousIO( void );

    virtual void     initForPM( IOService * provider );

    virtual void     selectTimingParameter( IOATADevConfig * configRequest,
                                            UInt32           unitNumber );

    virtual void     programTimingRegisters( void );

    virtual void     initializeHardware( void );

public:
    /* IOService overrides */

    virtual bool     start( IOService * provider );

    virtual void     stop( IOService * provider );

    virtual void     free( void );

    virtual IOWorkLoop * getWorkLoop( void ) const;

    virtual IOReturn message( UInt32      type,
                              IOService * provider,
                              void *      argument );

    virtual IOReturn setPowerState( unsigned long stateIndex,
                                    IOService *   whatDevice );

    /* Mandatory IOATAController overrides */

    virtual bool     configureTFPointers( void );

    virtual IOReturn provideBusInfo( IOATABusInfo * infoOut );

    virtual IOReturn getConfig( IOATADevConfig * configOut,
                                UInt32           unit );

    virtual IOReturn selectConfig( IOATADevConfig * config,
                                   UInt32           unit );

    /* Optional IOATAController overrides */

    virtual UInt32   scanForDrives( void );

    virtual IOReturn handleQueueFlush( void );

    /* Optional IOPCIATA overrides to support large transfers */

    virtual bool     allocDMAChannel( void );

    virtual void     initATADMAChains( PRD * descPtr );

    virtual IOReturn createChannelCommands( void );

    virtual bool     freeDMAChannel( void );
};

#endif /* !_APPLEATIATADRIVER_H */
