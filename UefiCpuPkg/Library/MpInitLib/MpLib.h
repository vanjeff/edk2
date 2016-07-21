/** @file
  Common header file for MP Initialize Library.

  Copyright (c) 2016, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _MP_LIB_H_
#define _MP_LIB_H_

#include <PiPei.h>

#include <Register/Cpuid.h>
#include <Register/Msr.h>
#include <Register/LocalApic.h>
#include <Register/Microcode.h>

#include <Library/MpInitLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/LocalApicLib.h>
#include <Library/CpuLib.h>
#include <Library/UefiCpuLib.h>
#include <Library/TimerLib.h>
#include <Library/SynchronizationLib.h>
#include <Library/MtrrLib.h>
#include <Library/HobLib.h>

#define WAKEUP_AP_SIGNAL SIGNATURE_32 ('S', 'T', 'A', 'P')

//
//  The MP data for switch BSP
//
#define CPU_SWITCH_STATE_IDLE   0
#define CPU_SWITCH_STATE_STORED 1
#define CPU_SWITCH_STATE_LOADED 2

//
// CPU exchange information for switch BSP
//
typedef struct {
  UINT8             State;        // offset 0
  UINTN             StackPointer; // offset 4 / 8
  IA32_DESCRIPTOR   Gdtr;         // offset 8 / 16
  IA32_DESCRIPTOR   Idtr;         // offset 14 / 26
} CPU_EXCHANGE_ROLE_INFO;

typedef enum {
  ApInHltLoop   = 1,
  ApInMwaitLoop = 2,
  ApInRunLoop   = 3
} AP_LOOP_MODE;

typedef enum {
  ApInitConfig   = 1,
  ApInitReconfig = 2,
  ApInitDone     = 3
} AP_INIT_STATE;

//
// AP state
//
typedef enum {
  CpuStateIdle,
  CpuStateReady,
  CpuStateBusy,
  CpuStateFinished,
  CpuStateDisabled
} CPU_STATE;

typedef struct {
  UINTN                          Cr0;
  UINTN                          Cr3;
  UINTN                          Cr4;
  UINTN                          Dr0;
  UINTN                          Dr1;
  UINTN                          Dr2;
  UINTN                          Dr3;
  UINTN                          Dr6;
  UINTN                          Dr7;
} CPU_VOLATILE_REGISTERS;

typedef struct {
  SPIN_LOCK                      ApLock;
  volatile UINT32                *StartupApSignal;
  volatile UINTN                 ApFunction;
  volatile UINTN                 ApFunctionArgument;
  UINT32                         InitialApicId;
  UINT32                         ApicId;
  UINT32                         Health;
  BOOLEAN                        CpuHealthy;
  volatile CPU_STATE             State;
  CPU_VOLATILE_REGISTERS         VolatileRegisters;
  BOOLEAN                        Waiting;
  BOOLEAN                        *Finished;
  UINT64                         ExpectedTime;
  UINT64                         CurrentTime;
  UINT64                         TotalTime;
  EFI_EVENT                      WaitEvent;
} CPU_AP_DATA;

//
// Basic CPU information saved in Guided HOB
//
typedef struct {
  UINT32                         InitialApicId;
  UINT32                         ApicId;
  UINT32                         Health;
} CPU_INFO_IN_HOB;

//
// AP reset code information
//
typedef struct {
  UINT8             *RendezvousFunnelAddress;
  UINTN             ModeEntryOffset;
  UINTN             RendezvousFunnelSize;
  UINT8             *RellocateApLoopFuncAddress;
  UINTN             RellocateApLoopFuncSize;
} MP_ASSEMBLY_ADDRESS_MAP;

typedef struct _CPU_MP_DATA  CPU_MP_DATA;

#pragma pack(1)

//
// MP CPU exchange information for AP reset code
// This structure is required to be packed because fixed field offsets
// into this structure are used in assembly code in this module
//
typedef struct {
  UINTN                 Lock;
  UINTN                 StackStart;
  UINTN                 StackSize;
  UINTN                 CFunction;
  IA32_DESCRIPTOR       GdtrProfile;
  IA32_DESCRIPTOR       IdtrProfile;
  UINTN                 BufferStart;
  UINTN                 ModeOffset;
  UINTN                 NumApsExecuting;
  UINTN                 CodeSegment;
  UINTN                 DataSegment;
  UINTN                 EnableExecuteDisable;
  UINTN                 Cr3;
  CPU_MP_DATA           *CpuMpData;
} MP_CPU_EXCHANGE_INFO;

#pragma pack()

//
// CPU MP Data save in memory
//
struct _CPU_MP_DATA {
  UINT64                         CpuInfoInHob;
  UINT32                         CpuCount;
  UINT32                         BspNumber;
  //
  // The above fields data will be passed from PEI to DXE
  //
  SPIN_LOCK                      MpLock;
  UINTN                          Buffer;
  UINTN                          CpuApStackSize;
  MP_ASSEMBLY_ADDRESS_MAP        AddressMap;
  UINTN                          WakeupBuffer;
  UINTN                          BackupBuffer;
  UINTN                          BackupBufferSize;
  BOOLEAN                        EndOfPeiFlag;

  volatile UINT32                StartCount;
  volatile UINT32                FinishedCount;
  volatile UINT32                RunningCount;
  BOOLEAN                        SingleThread;
  EFI_AP_PROCEDURE               Procedure;
  VOID                           *ProcArguments;
  BOOLEAN                        *Finished;
  UINT64                         ExpectedTime;
  UINT64                         CurrentTime;
  UINT64                         TotalTime;
  EFI_EVENT                      WaitEvent;
  UINTN                          **FailedCpuList;

  AP_INIT_STATE                  InitFlag;
  BOOLEAN                        X2ApicEnable;
  BOOLEAN                        SwitchBspFlag;
  CPU_EXCHANGE_ROLE_INFO         BSPInfo;
  CPU_EXCHANGE_ROLE_INFO         APInfo;
  MTRR_SETTINGS                  MtrrTable;
  UINT8                          ApLoopMode;
  UINT8                          ApTargetCState;
  UINT16                         PmCodeSegment;
  CPU_AP_DATA                    *CpuData;
  volatile MP_CPU_EXCHANGE_INFO  *MpCpuExchangeInfo;
};

extern EFI_GUID mCpuInitMpLibHobGuid;

/**
  Assembly code to place AP into safe loop mode.

  Place AP into targeted C-State if mwait-monitor is supported, otherwise
  place AP into hlt state.
  Place AP in protected mode if the current is long mode. Due to AP maybe
  wakeup by some hardware event. It could avoid accessing page table that
  may not available during booting to OS.

  @param[in] MwaitSupport    TRUE indicates mwait-monitor is supported.
                             FALSE indicates mwait-monitor is not supported.
  @param[in] ApTargetCState  Target C-State value.
  @param[in] PmCodeSegment   Proteced mode code segement value.
**/
typedef
VOID
(EFIAPI * ASM_RELLOCATE_AP_LOOP) (
  IN BOOLEAN                 MwaitSupport,
  IN UINTN                   ApTargetCState,
  IN UINTN                   PmCodeSegment
  );

/**
  Assembly code to get starting address and size of the rendezvous entry for APs.
  Information for fixing a jump instruction in the code is also returned.

  @param[out] AddressMap  Output buffer for address map information.
**/
VOID
EFIAPI
AsmGetAddressMap (
  OUT MP_ASSEMBLY_ADDRESS_MAP    *AddressMap
  );

/**
  This function is called by both the BSP and the AP which is to become the BSP to
  Exchange execution context including stack between them. After return from this
  function, the BSP becomes AP and the AP becomes the BSP.

  @param[in] MyInfo      Pointer to buffer holding the exchanging information for the executing processor.
  @param[in] OthersInfo  Pointer to buffer holding the exchanging information for the peer.

**/
VOID
EFIAPI
AsmExchangeRole (
  IN CPU_EXCHANGE_ROLE_INFO    *MyInfo,
  IN CPU_EXCHANGE_ROLE_INFO    *OthersInfo
  );

/**
  Get the pointer to CPU MP Data structure.

  @return  The pointer to CPU MP Data structure.
**/
CPU_MP_DATA *
GetCpuMpData (
  VOID
  );

/**
  Save the pointer to CPU MP Data structure.

  @param[in] CpuMpData  The pointer to CPU MP Data structure will be saved.
**/
VOID
SaveCpuMpData (
  IN CPU_MP_DATA   *CpuMpData
  );

/**
  Allocate reset vector buffer.

  @param[in, out]  CpuMpData  The pointer to CPU MP Data structure.
**/
VOID
AllocateResetVector (
  IN OUT CPU_MP_DATA          *CpuMpData
  );

/**
  Free AP reset vector buffer.

  @param[in]  CpuMpData  The pointer to CPU MP Data structure.
**/
VOID
FreeResetVector (
  IN CPU_MP_DATA              *CpuMpData
  );

/**
  This function will be called by BSP to wakeup AP.

  @param[in] CpuMpData          Pointer to CPU MP Data
  @param[in] Broadcast          TRUE:  Send broadcast IPI to all APs
                                FALSE: Send IPI to AP by ApicId
  @param[in] ProcessorNumber    The handle number of specified processor
  @param[in] Procedure          The function to be invoked by AP
  @param[in] ProcedureArgument  The argument to be passed into AP function
**/
VOID
WakeUpAP (
  IN CPU_MP_DATA               *CpuMpData,
  IN BOOLEAN                   Broadcast,
  IN UINTN                     ProcessorNumber,
  IN EFI_AP_PROCEDURE          Procedure,              OPTIONAL
  IN VOID                      *ProcedureArgument      OPTIONAL
  );

/**
  Initialize global data for MP support.

  @param[in] CpuMpData  The pointer to CPU MP Data structure.
**/
VOID
InitMpGlobalData (
  IN CPU_MP_DATA               *CpuMpData
  );

/**
  Worker function to execute a caller provided function on all enabled APs.

  @param[in]  Procedure               A pointer to the function to be run on
                                      enabled APs of the system. See type
                                      EFI_AP_PROCEDURE.
  @param[in]  SingleThread            If TRUE, then all the enabled APs execute
                                      the function specified by Procedure one by
                                      one, in ascending order of processor handle
                                      number.  If FALSE, then all the enabled APs
                                      execute the function specified by Procedure
                                      simultaneously.
  @param[in]  WaitEvent               The event created by the caller with CreateEvent()
                                      service.  If it is NULL, then execute in
                                      blocking mode. BSP waits until all APs finish
                                      or TimeoutInMicroSeconds expires.  If it's
                                      not NULL, then execute in non-blocking mode.
                                      BSP requests the function specified by
                                      Procedure to be started on all the enabled
                                      APs, and go on executing immediately. If
                                      all return from Procedure, or TimeoutInMicroSeconds
                                      expires, this event is signaled. The BSP
                                      can use the CheckEvent() or WaitForEvent()
                                      services to check the state of event.  Type
                                      EFI_EVENT is defined in CreateEvent() in
                                      the Unified Extensible Firmware Interface
                                      Specification.
  @param[in]  TimeoutInMicrosecsond   Indicates the time limit in microseconds for
                                      APs to return from Procedure, either for
                                      blocking or non-blocking mode. Zero means
                                      infinity.  If the timeout expires before
                                      all APs return from Procedure, then Procedure
                                      on the failed APs is terminated. All enabled
                                      APs are available for next function assigned
                                      by MpInitLibStartupAllAPs() or
                                      MPInitLibStartupThisAP().
                                      If the timeout expires in blocking mode,
                                      BSP returns EFI_TIMEOUT.  If the timeout
                                      expires in non-blocking mode, WaitEvent
                                      is signaled with SignalEvent().
  @param[in]  ProcedureArgument       The parameter passed into Procedure for
                                      all APs.
  @param[out] FailedCpuList           If NULL, this parameter is ignored. Otherwise,
                                      if all APs finish successfully, then its
                                      content is set to NULL. If not all APs
                                      finish before timeout expires, then its
                                      content is set to address of the buffer
                                      holding handle numbers of the failed APs.
                                      The buffer is allocated by MP Initialization
                                      library, and it's the caller's responsibility to
                                      free the buffer with FreePool() service.
                                      In blocking mode, it is ready for consumption
                                      when the call returns. In non-blocking mode,
                                      it is ready when WaitEvent is signaled.  The
                                      list of failed CPU is terminated by
                                      END_OF_CPU_LIST.

  @retval EFI_SUCCESS             In blocking mode, all APs have finished before
                                  the timeout expired.
  @retval EFI_SUCCESS             In non-blocking mode, function has been dispatched
                                  to all enabled APs.
  @retval EFI_UNSUPPORTED         A non-blocking mode request was made after the
                                  UEFI event EFI_EVENT_GROUP_READY_TO_BOOT was
                                  signaled.
  @retval EFI_UNSUPPORTED         WaitEvent is not NULL if non-blocking mode is not
                                  supported.
  @retval EFI_DEVICE_ERROR        Caller processor is AP.
  @retval EFI_NOT_STARTED         No enabled APs exist in the system.
  @retval EFI_NOT_READY           Any enabled APs are busy.
  @retval EFI_NOT_READY           MP Initialize Library is not initialized.
  @retval EFI_TIMEOUT             In blocking mode, the timeout expired before
                                  all enabled APs have finished.
  @retval EFI_INVALID_PARAMETER   Procedure is NULL.

**/
EFI_STATUS
StartupAllAPsWorker (
  IN  EFI_AP_PROCEDURE          Procedure,
  IN  BOOLEAN                   SingleThread,
  IN  EFI_EVENT                 WaitEvent               OPTIONAL,
  IN  UINTN                     TimeoutInMicroseconds,
  IN  VOID                      *ProcedureArgument      OPTIONAL,
  OUT UINTN                     **FailedCpuList         OPTIONAL
  );

/**
  Worker function to let the caller get one enabled AP to execute a caller-provided
  function.

  @param[in]  Procedure               A pointer to the function to be run on
                                      enabled APs of the system. See type
                                      EFI_AP_PROCEDURE.
  @param[in]  ProcessorNumber         The handle number of the AP. The range is
                                      from 0 to the total number of logical
                                      processors minus 1. The total number of
                                      logical processors can be retrieved by
                                      MpInitLibGetNumberOfProcessors().
  @param[in]  WaitEvent               The event created by the caller with CreateEvent()
                                      service.  If it is NULL, then execute in
                                      blocking mode. BSP waits until all APs finish
                                      or TimeoutInMicroSeconds expires.  If it's
                                      not NULL, then execute in non-blocking mode.
                                      BSP requests the function specified by
                                      Procedure to be started on all the enabled
                                      APs, and go on executing immediately. If
                                      all return from Procedure or TimeoutInMicroSeconds
                                      expires, this event is signaled. The BSP
                                      can use the CheckEvent() or WaitForEvent()
                                      services to check the state of event.  Type
                                      EFI_EVENT is defined in CreateEvent() in
                                      the Unified Extensible Firmware Interface
                                      Specification.
  @param[in]  TimeoutInMicrosecsond   Indicates the time limit in microseconds for
                                      APs to return from Procedure, either for
                                      blocking or non-blocking mode. Zero means
                                      infinity.  If the timeout expires before
                                      all APs return from Procedure, then Procedure
                                      on the failed APs is terminated. All enabled
                                      APs are available for next function assigned
                                      by MpInitLibStartupAllAPs() or
                                      MpInitLibStartupThisAP().
                                      If the timeout expires in blocking mode,
                                      BSP returns EFI_TIMEOUT.  If the timeout
                                      expires in non-blocking mode, WaitEvent
                                      is signaled with SignalEvent().
  @param[in]  ProcedureArgument       The parameter passed into Procedure for
                                      all APs.
  @param[out] Finished                If NULL, this parameter is ignored.  In
                                      blocking mode, this parameter is ignored.
                                      In non-blocking mode, if AP returns from
                                      Procedure before the timeout expires, its
                                      content is set to TRUE. Otherwise, the
                                      value is set to FALSE. The caller can
                                      determine if the AP returned from Procedure
                                      by evaluating this value.

  @retval EFI_SUCCESS             In blocking mode, specified AP finished before
                                  the timeout expires.
  @retval EFI_SUCCESS             In non-blocking mode, the function has been
                                  dispatched to specified AP.
  @retval EFI_UNSUPPORTED         A non-blocking mode request was made after the
                                  UEFI event EFI_EVENT_GROUP_READY_TO_BOOT was
                                  signaled.
  @retval EFI_UNSUPPORTED         WaitEvent is not NULL if non-blocking mode is not
                                  supported.
  @retval EFI_DEVICE_ERROR        The calling processor is an AP.
  @retval EFI_TIMEOUT             In blocking mode, the timeout expired before
                                  the specified AP has finished.
  @retval EFI_NOT_READY           The specified AP is busy.
  @retval EFI_NOT_READY           MP Initialize Library is not initialized.
  @retval EFI_NOT_FOUND           The processor with the handle specified by
                                  ProcessorNumber does not exist.
  @retval EFI_INVALID_PARAMETER   ProcessorNumber specifies the BSP or disabled AP.
  @retval EFI_INVALID_PARAMETER   Procedure is NULL.

**/
EFI_STATUS
StartupThisAPWorker (
  IN  EFI_AP_PROCEDURE          Procedure,
  IN  UINTN                     ProcessorNumber,
  IN  EFI_EVENT                 WaitEvent               OPTIONAL,
  IN  UINTN                     TimeoutInMicroseconds,
  IN  VOID                      *ProcedureArgument      OPTIONAL,
  OUT BOOLEAN                   *Finished               OPTIONAL
  );

/**
  Worker function to switch the requested AP to be the BSP from that point onward.

  @param[in] ProcessorNumber   The handle number of AP that is to become the new
                               BSP. The range is from 0 to the total number of
                               logical processors minus 1. The total number of
                               logical processors can be retrieved by
                               MpInitLibGetNumberOfProcessors().
  @param[in] EnableOldBSP      If TRUE, then the old BSP will be listed as an
                               enabled AP. Otherwise, it will be disabled.

  @retval EFI_SUCCESS             BSP successfully switched.
  @retval EFI_UNSUPPORTED         Switching the BSP cannot be completed prior to
                                  this service returning.
  @retval EFI_UNSUPPORTED         Switching the BSP is not supported.
  @retval EFI_SUCCESS             The calling processor is an AP.
  @retval EFI_NOT_FOUND           The processor with the handle specified by
                                  ProcessorNumber does not exist.
  @retval EFI_INVALID_PARAMETER   ProcessorNumber specifies the current BSP or
                                  a disabled AP.
  @retval EFI_NOT_READY           The specified AP is busy.

**/
EFI_STATUS
SwitchBspWorker (
  IN UINTN                     ProcessorNumber,
  IN BOOLEAN                   EnableOldBSP
  );

/**
  Worker function to let the caller enable or disable an AP from this point onward.
  This service may only be called from the BSP.

  @param[in] ProcessorNumber   The handle number of AP that is to become the new
                               BSP. The range is from 0 to the total number of
                               logical processors minus 1. The total number of
                               logical processors can be retrieved by
                               MpInitLibGetNumberOfProcessors().
  @param[in] EnableAP          Specifies the new state for the processor for
                               enabled, FALSE for disabled.
  @param[in] HealthFlag        If not NULL, a pointer to a value that specifies
                               the new health status of the AP. This flag
                               corresponds to StatusFlag defined in
                               EFI_MP_SERVICES_PROTOCOL.GetProcessorInfo(). Only
                               the PROCESSOR_HEALTH_STATUS_BIT is used. All other
                               bits are ignored.  If it is NULL, this parameter
                               is ignored.

  @retval EFI_SUCCESS             The specified AP was enabled or disabled successfully.
  @retval EFI_UNSUPPORTED         Enabling or disabling an AP cannot be completed
                                  prior to this service returning.
  @retval EFI_UNSUPPORTED         Enabling or disabling an AP is not supported.
  @retval EFI_DEVICE_ERROR        The calling processor is an AP.
  @retval EFI_NOT_FOUND           Processor with the handle specified by ProcessorNumber
                                  does not exist.
  @retval EFI_INVALID_PARAMETER   ProcessorNumber specifies the BSP.
  @retval EFI_NOT_READY           MP Initialize Library is not initialized.

**/
EFI_STATUS
EnableDisableApWorker (
  IN  UINTN                     ProcessorNumber,
  IN  BOOLEAN                   EnableAP,
  IN  UINT32                    *HealthFlag OPTIONAL
  );

/**
  Get pointer to CPU MP Data structure from GUIDed HOB.

  @return  The pointer to CPU MP Data structure.
**/
CPU_MP_DATA *
GetCpuMpDataFromGuidedHob (
  VOID
  );

/** Checks status of specified AP.

  This function checks whether the specified AP has finished the task assigned
  by StartupThisAP(), and whether timeout expires.

  @param[in]  ProcessorNumber       The handle number of processor.

  @retval EFI_SUCCESS           Specified AP has finished task assigned by StartupThisAPs().
  @retval EFI_TIMEOUT           The timeout expires.
  @retval EFI_NOT_READY         Specified AP has not finished task and timeout has not expired.
**/
EFI_STATUS
CheckThisAP (
  IN UINTN        ProcessorNumber
  );

/**
  Checks status of all APs.

  This function checks whether all APs have finished task assigned by StartupAllAPs(),
  and whether timeout expires.

  @retval EFI_SUCCESS           All APs have finished task assigned by StartupAllAPs().
  @retval EFI_TIMEOUT           The timeout expires.
  @retval EFI_NOT_READY         APs have not finished task and timeout has not expired.
**/
EFI_STATUS
CheckAllAPs (
  VOID
  );

/**
  Checks APs status and updates APs status if needed.

**/
VOID
CheckAndUpdateAPsStatus (
  VOID
  );

/**
  Detect whether specified processor can find matching microcode patch and load it.

  @param[in] PeiCpuMpData        Pointer to PEI CPU MP Data
**/
VOID
MicrocodeDetect (
  IN CPU_MP_DATA             *CpuMpData
  );

/**
  Detect whether Mwait-monitor feature is supported.

  @retval TRUE    Mwait-monitor feature is supported.
  @retval FALSE   Mwait-monitor feature is not supported.
**/
BOOLEAN
IsMwaitSupport (
  VOID
  );

/**
  Notify function on End Of PEI PPI.

  On S3 boot, this function will restore wakeup buffer data.
  On normal boot, this function will flag wakeup buffer to be un-used type.

  @param[in]  PeiServices        The pointer to the PEI Services Table.
  @param[in]  NotifyDescriptor   Address of the notification descriptor data structure.
  @param[in]  Ppi                Address of the PPI that was installed.

  @retval EFI_SUCCESS        When everything is OK.
**/
EFI_STATUS
EFIAPI
CpuMpEndOfPeiCallback (
  IN EFI_PEI_SERVICES             **PeiServices,
  IN EFI_PEI_NOTIFY_DESCRIPTOR    *NotifyDescriptor,
  IN VOID                         *Ppi
  );

#endif

