/******************************************************************************

@file zcl_samplewarnigndevice.h

@brief Z-Stack 3.0 Sample Application

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

#ifndef ZCL_SAMPLEWARNINGDEVICE_H
#define ZCL_SAMPLEWARNINGDEVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "zcl_ss.h"

// Added to include ZLL Target functionality
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  #include "zcl_general.h"
  #include "bdb_tlCommissioning.h"
#endif

/*********************************************************************
 * CONSTANTS
 */
 
// Added to include ZLL Target functionality
#define SAMPLEWARNINGDEVICE_NUM_GRPS            2

#define SAMPLEWARNINGDEVICE_ENDPOINT            20

#define SAMPLEWARNINGDEVICE_MAX_ATTRIBUTES      11

#define SAMPLEWARNINGDEVICE_ALARM_MAX_DURATION    0x00F0

#define SQUAWKBLINKS 4

// Application Events
#define SAMPLEWARNINGDEVICE_IDENTIFY_TIMEOUT_EVT		     0x0001
#define SAMPLEWARNINGDEVICE_EZMODE_TIMEOUT_EVT           0x0002
#define SAMPLEWARNINGDEVICE_EZMODE_NEXTSTATE_EVT         0x0004
#define SAMPLEWARNINGDEVICE_MAIN_SCREEN_EVT              0x0008
#define SAMPLEWARNINGDEVICE_END_DEVICE_REJOIN_EVT         0x0010
#define SAMPLEWARNINGDEVICE_CLEAR_SQUAWK_NOTIFICATION_EVT    0x0020

// Application Display Modes
#define WARNINGDEVICE_MAINMODE         0x00
  
#define SAMPLEWARNINGDEVICE_END_DEVICE_REJOIN_DELAY 10000

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * VARIABLES
 */
 
// Added to include ZLL Target functionality 
#if defined ( BDB_TL_INITIATOR ) || defined ( BDB_TL_TARGET )
  extern bdbTLDeviceInfo_t tlSampleFireDetector_DeviceInfo;
#endif

extern SimpleDescriptionFormat_t zclSampleWarningDevice_SimpleDesc;

extern CONST zclAttrRec_t zclSampleWarningDevice_Attrs[];



extern uint16 zclSampleWarningDevice_IdentifyTime;

extern uint8 zclSampleWarningDevice_ZoneState;
extern uint16 zclSampleWarningDevice_ZoneType;
extern uint16 zclSampleWarningDevice_ZoneStatus;
extern uint8 zclSampleWarningDevice_IAS_CIE_Address[8];

// Groups attributes
extern uint8 zclSampleWarningDevice_GroupsNameSupport;

// Scenes attributes
extern uint8        zclSampleWarningDevice_ScenesSceneCount;
extern uint8        zclSampleWarningDevice_ScenesCurrentScene;
extern uint16       zclSampleWarningDevice_ScenesCurrentGroup;
extern uint8        zclSampleWarningDevice_ScenesSceneValid;
extern CONST uint8  zclSampleWarningDevice_ScenesNameSupport;

/*********************************************************************
 * FUNCTIONS
 */

 /*
  * Initialization for the task
  */
extern void zclSampleWarningDevice_Init( byte task_id );

/*
 *  Event Process for the task
 */
extern UINT16 zclSampleWarningDevice_event_loop( byte task_id, UINT16 events );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLEWARNINGDEVICE_H */
