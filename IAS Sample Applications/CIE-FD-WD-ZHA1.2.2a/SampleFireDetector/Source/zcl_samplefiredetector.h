/******************************************************************************

@file zcl_samplefiredetector.h

@brief Z-Stack Home 1.2.2a Sample Application

@date 2017-02-15 18:24:32

Group: CMCU LPC
Target Devices: CC2530/CC2531/CC2538

******************************************************************************

Copyright (c) 2017, Texas Instruments Incorporated
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

*  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

*  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

*  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/

#ifndef ZCL_SAMPLEFIREDETECTOR_H
#define ZCL_SAMPLEFIREDETECTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "zcl_ss.h"

/*********************************************************************
 * CONSTANTS
 */
#define SAMPLEFIREDETECTOR_ENDPOINT            17

#define SAMPLEFIREDETECTOR_MAX_ATTRIBUTES      14

#define SAMPLEFIREDETECTOR_ALARM_MAX_DURATION    0x00F0

#define SQUAWKBLINKS 4

// Application Events
#define SAMPLEFIREDETECTOR_IDENTIFY_TIMEOUT_EVT		      0x0001
#define SAMPLEFIREDETECTOR_EZMODE_TIMEOUT_EVT           0x0002
#define SAMPLEFIREDETECTOR_EZMODE_NEXTSTATE_EVT         0x0004
#define SAMPLEFIREDETECTOR_MAIN_SCREEN_EVT              0x0008
#define RESET_EVT                                       0x0010
  
// Application Display Modes
#define FIREDETECTOR_MAINMODE         0x00

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
extern SimpleDescriptionFormat_t zclSampleFireDetector_SimpleDesc;

extern CONST zclAttrRec_t zclSampleFireDetector_Attrs[];



extern uint16 zclSampleFireDetector_IdentifyTime;

extern uint8 zclSampleFireDetector_ZoneState;
extern uint16 zclSampleFireDetector_ZoneType;
extern uint16 zclSampleFireDetector_ZoneStatus;
extern uint8 zclSampleFireDetector_IAS_CIE_Address[8];

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleFireDetector_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleFireDetector_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLEFIREDETECTOR_H */
