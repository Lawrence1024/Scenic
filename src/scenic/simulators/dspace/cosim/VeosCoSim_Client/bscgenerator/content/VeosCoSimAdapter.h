// Copyright dSPACE GmbH. All rights reserved.

#pragma once

#ifdef HAS_CAN
#include <DsCan.h>
#endif
#if defined(HAS_ETH) && !defined(OLD_ETH_IF)
#include <DsPcap.h>
#endif
#ifdef HAS_LIN
#include <dslin.h>
#endif
#include <stdbool.h>

#include "VeosCoSim.h"

typedef void (*VoidVoidCallback)(void);

#ifdef HAS_ETH
#ifdef OLD_ETH_IF
typedef uint32_t EthAddress;
#else
typedef DsEthInterfaceHandle EthAddress;
#endif
#endif

typedef union AddressType {
#ifdef HAS_CAN
    DsTCanCh_Address* canAddress;
#endif
#ifdef HAS_ETH
    EthAddress* ethAddress;
#endif
#ifdef HAS_LIN
    dslin_channel_address_t* linAddress;
#endif
    void* dummy;  // So that this struct will not be empty
} AddressType;

VEOSCOSIM_EXTERN bool VeosCoSim_Adapter_Load(uint16_t port,
                                             const char* serverName,
                                             VeosCoSim_Time sampleTime,
                                             bool clientOptional,
                                             uint32_t ioSignalsCount,
                                             const VeosCoSim_IoSignalInfo* ioSignals,
                                             bool noLoopBackMessages,
                                             uint32_t busControllersCount,
                                             const VeosCoSim_BusChannelInfo* busControllers,
                                             AddressType busAddresses[],
                                             VoidVoidCallback requestTerminate,
                                             VoidVoidCallback requestStop);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_Unload(void);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_Start(void);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_Stop(void);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_PreStep(void);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_Step(void);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_OutputSignalRead(VeosCoSim_IoSignalId id, uint32_t* length, void* value);
VEOSCOSIM_EXTERN void VeosCoSim_Adapter_InputSignalWrite(VeosCoSim_IoSignalId id, uint32_t length, const void* value);
