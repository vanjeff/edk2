;------------------------------------------------------------------------------ ;
; Copyright (c) 2015 - 2016, Intel Corporation. All rights reserved.<BR>
; This program and the accompanying materials
; are licensed and made available under the terms and conditions of the BSD License
; which accompanies this distribution.  The full text of the license may be found at
; http://opensource.org/licenses/bsd-license.php.
;
; THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
; WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
;
; Module Name:
;
;   MpFuncs.nasm
;
; Abstract:
;
;   This is the assembly code for MP support
;
;-------------------------------------------------------------------------------

%include "MpEqu.inc"
extern ASM_PFX(InitializeFloatingPointUnits)

DEFAULT REL

SECTION .text

;-------------------------------------------------------------------------------------
;RendezvousFunnelProc  procedure follows. All APs execute their procedure. This
;procedure serializes all the AP processors through an Init sequence. It must be
;noted that APs arrive here very raw...ie: real mode, no stack.
;ALSO THIS PROCEDURE IS EXECUTED BY APs ONLY ON 16 BIT MODE. HENCE THIS PROC
;IS IN MACHINE CODE.
;-------------------------------------------------------------------------------------
global ASM_PFX(RendezvousFunnelProc)
ASM_PFX(RendezvousFunnelProc):
RendezvousFunnelProcStart:
; At this point CS = 0x(vv00) and ip= 0x0.
; Save BIST information to ebp firstly

BITS 16
    mov        ebp, eax                        ; Save BIST information

    mov        ax, cs
    mov        ds, ax
    mov        es, ax
    mov        ss, ax
    xor        ax, ax
    mov        fs, ax
    mov        gs, ax

    mov        si,  BufferStartLocation
    mov        ebx, [si]

    mov        di,  ModeOffsetLocation
    mov        eax, [di]
    mov        di,  CodeSegmentLocation
    mov        edx, [di]
    mov        di,  ax
    sub        di,  02h
    mov        [di],dx                         ; Patch long mode CS
    sub        di,  04h
    add        eax, ebx
    mov        [di],eax                        ; Patch address

    mov        si, GdtrLocation
o32 lgdt       [cs:si]

    mov        si, IdtrLocation
o32 lidt       [cs:si]

    mov        si, EnableExecuteDisableLocation
    cmp        byte [si], 0
    jz         SkipEnableExecuteDisableBit

    ;
    ; Enable execute disable bit
    ;
    mov        ecx, 0c0000080h             ; EFER MSR number
    rdmsr                                  ; Read EFER
    bts        eax, 11                     ; Enable Execute Disable Bit
    wrmsr                                  ; Write EFER

SkipEnableExecuteDisableBit:

    mov        di,  DataSegmentLocation
    mov        edi, [di]                   ; Save long mode DS in edi

    mov        si, Cr3Location             ; Save CR3 in ecx
    mov        ecx, [si]

    xor        ax,  ax
    mov        ds,  ax                     ; Clear data segment

    mov        eax, cr0                    ; Get control register 0
    or         eax, 000000003h             ; Set PE bit (bit #0) & MP
    mov        cr0, eax

    mov        eax, cr4
    bts        eax, 5
    mov        cr4, eax

    mov        cr3, ecx                    ; Load CR3

    mov        ecx, 0c0000080h             ; EFER MSR number
    rdmsr                                  ; Read EFER
    bts        eax, 8                      ; Set LME=1
    wrmsr                                  ; Write EFER

    mov        eax, cr0                    ; Read CR0
    bts        eax, 31                     ; Set PG=1
    mov        cr0, eax                    ; Write CR0

    jmp        0:strict dword 0  ; far jump to long mode
BITS 64
LongModeStart:
    mov        eax, edi
    mov        ds,  ax
    mov        es,  ax
    mov        ss,  ax

    mov        esi, ebx
    mov        edi, esi
    add        edi, LockLocation
    mov        rax, NotVacantFlag

TestLock:
    xchg       qword [edi], rax
    cmp        rax, NotVacantFlag
    jz         TestLock

    mov        edi, esi
    add        edi, NumApsExecutingLocation
    inc        dword [edi]
    mov        ebx, [edi]

ProgramStack:
    mov        edi, esi
    add        edi, StackSizeLocation
    mov        rax, qword [edi]
    mov        edi, esi
    add        edi, StackStartAddressLocation
    add        rax, qword [edi]
    mov        rsp, rax
    mov        qword [edi], rax

Releaselock:
    mov        rax, VacantFlag
    mov        edi, esi
    add        edi, LockLocation
    xchg       qword [edi], rax

CProcedureInvoke:
    push       rbp               ; Push BIST data at top of AP stack
    xor        rbp, rbp          ; Clear ebp for call stack trace
    push       rbp
    mov        rbp, rsp

    mov        rax, ASM_PFX(InitializeFloatingPointUnits)
    sub        rsp, 20h
    call       rax               ; Call assembly function to initialize FPU per UEFI spec
    add        rsp, 20h

    mov        edx, ebx          ; edx is NumApsExecuting
    mov        ecx, esi
    add        ecx, LockLocation ; rcx is address of exchange info data buffer

    mov        edi, esi
    add        edi, ApProcedureLocation
    mov        rax, qword [edi]

    sub        rsp, 20h
    call       rax               ; Invoke C function
    add        rsp, 20h
    jmp        $                 ; Should never reach here

RendezvousFunnelProcEnd:

