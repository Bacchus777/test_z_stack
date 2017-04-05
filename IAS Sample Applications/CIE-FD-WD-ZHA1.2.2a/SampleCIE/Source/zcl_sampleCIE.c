/******************************************************************************

@file zcl_sampleCIE.c

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

/*********************************************************************
  This device will act as CIE

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Start EZ-Mode Commissioning
    - SW2: [unassigned]
    - SW3: Write CIE IEEE Address Attribute on last commissioned IAS Zone device
    - SW4: Send Squawk to all Warning Devices in network
    - SW5: Reset to factory new
  ----------------------------------------
*********************************************************************/
/*********************************************************************
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"
#include "MT_SYS.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ezmode.h"


#include "zcl_sampleCIE.h"

#include "onboard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "zcl_ss.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
byte zclSampleCIE_TaskID;

//keeps count of items in WD table
uint8 wdCount = 0;
#ifdef ZCL_WD
zclSS_WDItem_t *wdTable = (zclSS_WDItem_t *)NULL;
#endif //ZCL_WD

uint8 zclSampleCIESeqNum;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleCIE_DstAddr;
uint8 zoneid ;

#ifdef ZCL_EZMODE
static void zclSampleCIE_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleCIE_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleCIE_RegisterEZModeData =
{
  &zclSampleCIE_TaskID,
  SAMPLECIE_EZMODE_NEXTSTATE_EVT,
  SAMPLECIE_EZMODE_TIMEOUT_EVT,
  &zclSampleCIESeqNum,
  zclSampleCIE_EZModeCB
};

// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_SS_IAS_ZONE,
  ZCL_CLUSTER_ID_SS_IAS_WD
};
#define ZCLSAMPLECIE_BINDINGLIST        2 
#endif

devStates_t zclSampleCIE_NwkState = DEV_INIT;

uint8 gCIEScreenMode = CIE_MAINMODE;   // display main screen mode first

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleCIE_TestEp =
{
  SAMPLECIE_ENDPOINT,       // Test endpoint
  &zclSampleCIE_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

static afAddrType_t afDstAddr;


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleCIE_HandleKeys( byte shift, byte keys );
static void zclSampleCIE_BasicResetCB( void );
static void zclSampleCIE_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleCIE_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSampleCIE_ProcessIdentifyTimeChange( void );

// app display functions
void zclSampleCIE_LcdDisplayUpdate(void);
void zclSampleCIE_LcdDisplayMainMode(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleCIE_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleCIE_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleCIE_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleCIE_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleCIE_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleCIE_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleCIE_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif // ZCL_DISCOVER


#ifdef ZCL_ZONE
static void zclSampleCIE_WriteIAS_CIE_Address(afAddrType_t *dstAddr);
//SS cluster callback functions
//Zone Cluster
static ZStatus_t zclSampleCIE_ChangeNotificationCB(zclZoneChangeNotif_t *pCmd,afAddrType_t *srcAddr );
static ZStatus_t zclSampleCIE_EnrollRequestCB(zclZoneEnrollReq_t *pReq, uint8 endpoint );
#endif

#ifdef ZCL_WD
static void zclSendSquawkToAllWD(zclWDSquawk_t *squawk);
static void zclSendWarningToAllWD(zclWDStartWarning_t *alarm);
#endif


#ifdef ZCL_ACE
static uint8 zclSampleCIE_ArmCB(zclACEArm_t* armMode);
static ZStatus_t zclSampleCIE_BypassCB(zclACEBypass_t *pCmd);
static ZStatus_t zclSampleCIE_EmergencyCB(void);
static ZStatus_t zclSampleCIE_FireCB(void);
static ZStatus_t zclSampleCIE_PanicCB(void);
#endif

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = "  CIE";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwSquawk[]= "SW3: Squawk";
const char sSwHelp[]       = "SW5: Help";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleCIE_CmdCallbacks =
{
  zclSampleCIE_BasicResetCB,        // Basic Cluster Reset command
  zclSampleCIE_IdentifyCB,          // Identify command
#ifdef ZCL_EZMODE
  NULL,                                           // Identify EZ-Mode Invoke command
  NULL,                                           // Identify Update Commission State command
#endif
  NULL,                                           // Identify Trigger Effect command
  zclSampleCIE_IdentifyQueryRspCB,  // Identify Query Response command
  NULL,             				                      // On/Off cluster command
  NULL,                                           // On/Off cluster enhanced command Off with Effect
  NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                           // On/Off cluster enhanced command On with Timed Off
#ifdef ZCL_LEVEL_CTRL
  NULL,                                           // Level Control Move to Level command
  NULL,                                           // Level Control Move command
  NULL,                                           // Level Control Step command
  NULL,                                           // Level Control Stop command
#endif
#ifdef ZCL_GROUPS
  NULL,                                           // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                           // Scene Store Request command
  NULL,                                           // Scene Recall Request command
  NULL,                                           // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                           // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                           // Get Event Log command
  NULL,                                           // Publish Event Log command
#endif
  NULL,                                           // RSSI Location command
  NULL                                            // RSSI Location Response command
};

/*********************************************************************
* ZCL SS Profile Callback table
*/


static zclSS_AppCallbacks_t zclSampleCIE_SSCmdCallbacks =
{
  zclSampleCIE_ChangeNotificationCB,      // Change Notification command
  zclSampleCIE_EnrollRequestCB,           // Enroll Request command
  NULL,                                   // Enroll Reponse command
  NULL,                                   // Initiate Normal Operation Mode command
  NULL,                                   // Initiate Test Mode command
  zclSampleCIE_ArmCB,                     // Arm command
  zclSampleCIE_BypassCB,                  // Bypass command
  zclSampleCIE_EmergencyCB,               // Emergency command
  zclSampleCIE_FireCB,                    // Fire command
  zclSampleCIE_PanicCB,                   // Panic command
  NULL,                                   // Get Zone ID Map command
  NULL,                                   // Get Zone Information Command
  NULL,                                   // Get Panel Status Command
  NULL,                                   // Get Bypassed Zone List Command
  NULL,                                   // Get Zone Status Command
  NULL,                                   // ArmResponse command
  NULL,                                   // Get Zone ID Map Response command
  NULL,                                   // Get Zone Information Response command
  NULL,                                   // Zone Status Changed command
  NULL,                                   // Panel Status Changed command
  NULL,                                   // Get Panel Status Response command
  NULL,                                   // Set Bypassed Zone List command
  NULL,                                   // Bypass Response command
  NULL,                                   // Get Zone Status Response command
  NULL,                                   // Start Warning command
  NULL                                    // Squawk command 
};


/*********************************************************************
 * @fn          zclSampleCIE_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleCIE_Init( byte task_id )
{
  zclSampleCIE_TaskID = task_id;

  // Set destination address to indirect
  zclSampleCIE_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleCIE_DstAddr.endPoint = 0;
  zclSampleCIE_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleCIE_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLECIE_ENDPOINT, &zclSampleCIE_CmdCallbacks );

  // Register the ZCL Security and Safety Cluster Library callback functions
  zclSS_RegisterCmdCallbacks( SAMPLECIE_ENDPOINT, &zclSampleCIE_SSCmdCallbacks );
    
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLECIE_ENDPOINT, SAMPLECIE_MAX_ATTRIBUTES, zclSampleCIE_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleCIE_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleCIE_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleCIE_TaskID );

  // Register for a test endpoint
  afRegister( &sampleCIE_TestEp );

#ifdef LCD_SUPPORTED
  // display the device name
#ifdef HAL_BOARD_CC2538 
    HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_4 );
#endif
#ifdef HAL_MCU_CC2530
    HalLcdWriteString( (char *)sDeviceName, HAL_LCD_LINE_3 );
#endif  
#endif
}

/*********************************************************************
 * @fn          zclSample_event_loop
 *
 * @brief       Event Loop Processor for zclGeneral.
 *
 * @param       none
 *
 * @return      none
 */
uint16 zclSampleCIE_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleCIE_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleCIE_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleCIE_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleCIE_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleCIE_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( (zclSampleCIE_NwkState == DEV_ZB_COORD) ||
               (zclSampleCIE_NwkState == DEV_ROUTER)   ||
               (zclSampleCIE_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            gCIEScreenMode = CIE_MAINMODE;
            zclSampleCIE_LcdDisplayUpdate();
#endif
#ifdef ZCL_EZMODE
            zcl_EZModeAction( EZMODE_ACTION_NETWORK_STARTED, NULL );
#endif // ZCL_EZMODE
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SAMPLECIE_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleCIE_IdentifyTime > 0 )
      zclSampleCIE_IdentifyTime--;
    zclSampleCIE_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLECIE_IDENTIFY_TIMEOUT_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SAMPLECIE_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLECIE_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLECIE_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLECIE_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

  if ( events & SAMPLECIE_MAIN_SCREEN_EVT )
  {
    gCIEScreenMode = CIE_MAINMODE;
    zclSampleCIE_LcdDisplayUpdate();

    return ( events ^ SAMPLECIE_MAIN_SCREEN_EVT );
  }
  
  if ( events & RESET_EVT )
  {
    zgWriteStartupOptions(ZG_STARTUP_SET, (ZCD_STARTOPT_DEFAULT_NETWORK_STATE | ZCD_STARTOPT_DEFAULT_CONFIG_STATE) );
   
    // no return from this call
    SystemReset();
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleCIE_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_5
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void zclSampleCIE_HandleKeys( byte shift, byte keys )
{
  HalLcdWriteString ("", HAL_LCD_LINE_2);
  HalLcdWriteString ("", HAL_LCD_LINE_3);

  if ( keys & HAL_KEY_SW_1 )
  {
#ifdef ZCL_EZMODE
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_SS_IAS_ZONE, ZCL_CLUSTER_ID_SS_IAS_WD };

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLECIE_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclSampleCIE_NwkState == DEV_ZB_COORD ) ||
           ( zclSampleCIE_NwkState == DEV_ROUTER )   ||
           ( zclSampleCIE_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;
      ezModeData.numActiveInClusters = 0;
      ezModeData.pActiveInClusterIDs = NULL;
      ezModeData.numActiveOutClusters = 2;   // active output cluster
      ezModeData.pActiveOutClusterIDs = clusterIDs;
      zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

#endif // ZCL_EZMODE
  }

  if( keys & HAL_KEY_SW_2 )
  {

  }

  if( keys & HAL_KEY_SW_3 )
  {
    // send write CIE IEEE attribute request to IAS Zone Server (e.g. fire detector) in network 
    zclSampleCIE_WriteIAS_CIE_Address(&afDstAddr);
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    
    // build the squawk msg
    zclWDSquawk_t squawk;
    squawk.squawkbits.squawkMode = SS_IAS_SQUAWK_SQUAWK_MODE_SYSTEM_ALARMED_NOTIFICATION_SOUND;
    squawk.squawkbits.strobe = SS_IAS_SQUAWK_STROBE_USE_STROBE_BLINK_IN_PARALLEL_TO_SQUAWK;
    squawk.squawkbits.squawkLevel = SS_IAS_SQUAWK_SQUAWK_LEVEL_VERY_HIGH_LEVEL_SOUND;
    
    // send squawk to all warning devices in the network
    zclSendSquawkToAllWD(&squawk);
      
    HalLcdWriteString ("Squawks Sent", HAL_LCD_LINE_2);
  
  }

  if ( keys & HAL_KEY_SW_5 )
  {
    // factory new reset
    zclSampleCIE_BasicResetCB();
  }

}

/*********************************************************************
 * @fn      zclSampleCIE_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleCIE_LcdDisplayUpdate( void )
{    
  zclSampleCIE_LcdDisplayMainMode();
}

/*********************************************************************
 * @fn      zclSampleCIE_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleCIE_LcdDisplayMainMode( void )
{
  if ( zclSampleCIE_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSampleCIE_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSampleCIE_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }

}

/*********************************************************************
 * @fn      zclSampleCIE_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleCIE_ProcessIdentifyTimeChange( void )
{ 
  
  if ( zclSampleCIE_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleCIE_TaskID, SAMPLECIE_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleCIE_DeviceEnable )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclSampleCIE_TaskID, SAMPLECIE_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleCIE_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleCIE_BasicResetCB( void )
{

  if(ZG_BUILD_COORDINATOR_TYPE && ZG_DEVICE_COORDINATOR_TYPE)
  {
    // sending leave does nothing for ZC
  }
  else
  {
    // remove self from network if ZR or ZED
    NLME_LeaveReq_t req;
    req.extAddr = NULL;
    req.removeChildren = 0;
    req.rejoin = 0;
    req.silent = 0;
    NLME_LeaveReq(&req);
  }

  // restart device after 3 seconds (wait for leave)
  osal_start_timerEx( zclSampleCIE_TaskID, RESET_EVT, 3000);
}

/*********************************************************************
 * @fn      zclSampleCIE_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleCIE_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleCIE_IdentifyTime = pCmd->identifyTime;
  zclSampleCIE_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleCIE_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleCIE_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp )
{
  (void)pRsp;
#ifdef ZCL_EZMODE
  {
    zclEZMode_ActionData_t data;
    data.pIdentifyQueryRsp = pRsp;
    zcl_EZModeAction ( EZMODE_ACTION_IDENTIFY_QUERY_RSP, &data );
  }
#endif
}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleCIE_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleCIE_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleCIE_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleCIE_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleCIE_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleCIE_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleCIE_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleCIE_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleCIE_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleCIE_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleCIE_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleCIE_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleCIE_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleCIE_ProcessInDiscAttrsExtRspCmd( pInMsg );
      break;
#endif
    default:
      break;
  }

  if ( pInMsg->attrCmd )
  {
    osal_mem_free( pInMsg->attrCmd );
  }
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSampleCIE_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleCIE_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8 i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSampleCIE_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleCIE_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8 i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSampleCIE_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleCIE_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleCIE_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleCIE_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleCIE_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleCIE_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSampleCIE_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleCIE_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8 i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER

#ifdef ZCL_EZMODE

/*********************************************************************
 * @fn      zclSampleCIE_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleCIE_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
{
  zclEZMode_ActionData_t data;
  ZDO_MatchDescRsp_t *pMatchDescRsp;

  // Let EZ-Mode know of the Match Descriptor Response
  if ( pMsg->clusterID == Match_Desc_rsp )
  {
    pMatchDescRsp = ZDO_ParseEPListRsp( pMsg );
    data.pMatchDescRsp = pMatchDescRsp;
    zcl_EZModeAction( EZMODE_ACTION_MATCH_DESC_RSP, &data );
    osal_mem_free( pMatchDescRsp );

    // remember address of last device commissioned for writing CIE IEEE Addr attribute
    afDstAddr.addrMode = afAddr16Bit;
    afDstAddr.addr.shortAddr = pMatchDescRsp->nwkAddr;
    afDstAddr.endPoint = pMatchDescRsp->epList[0];
    afDstAddr.panId = 0xFFFF;
  }
}

/*********************************************************************
 * @fn      zclSampleCIE_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleCIE_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSampleCIE_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    zclSampleCIE_ProcessIdentifyTimeChange();
  }

  // autoclosing, show what happened (success, cancelled, etc...)
  if( state == EZMODE_STATE_AUTOCLOSE )
  {
#ifdef LCD_SUPPORTED
    pStr = NULL;
    err = pData->sAutoClose.err;
    if ( err == EZMODE_ERR_SUCCESS )
    {
      pStr = "EZMode: Success";
    }
    else if ( err == EZMODE_ERR_NOMATCH )
    {
      pStr = "EZMode: NoMatch"; // not a match made in heaven
    }
    if ( pStr )
    {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif
  }

  // finished, either show DstAddr/EP, or nothing (depending on success or not)
  if( state == EZMODE_STATE_FINISH )
  {
    // turn off identify mode
    zclSampleCIE_IdentifyTime = 0;
    zclSampleCIE_ProcessIdentifyTimeChange();

#ifdef LCD_SUPPORTED
    // if successful, inform user which nwkaddr/ep we bound to
    pStr = NULL;
    err = pData->sFinish.err;
    if( err == EZMODE_ERR_SUCCESS )
    {
      // "EZDst:1234 EP:34"
      osal_memcpy( szLine, "EZDst:", 6 );
      zclHA_uint16toa( pData->sFinish.nwkaddr, &szLine[6] );
      osal_memcpy( &szLine[10], " EP:", 4 );
      _ltoa( pData->sFinish.ep, (void *)(&szLine[14]), 16 );  // _ltoa NULL terminates
      pStr = szLine;
            
      //zclAddtoWDList( pData->sFinish.nwkaddr,  pData->sFinish.ep); 
      
    }
    else if ( err == EZMODE_ERR_BAD_PARAMETER )
    {
      pStr = "EZMode: BadParm";
    }
    else if ( err == EZMODE_ERR_CANCELLED )
    {
      pStr = "EZMode: Cancel";
    }
    else
    {
      pStr = "EZMode: TimeOut";
    }
    if ( pStr )
    {
        HalLcdWriteString ( pStr, HAL_LCD_LINE_2 );
    }
#endif  // LCD_SUPPORTED

    // show main UI screen 3 seconds after joining network
    osal_start_timerEx( zclSampleCIE_TaskID, SAMPLECIE_MAIN_SCREEN_EVT, 3000 );

  }
}
#endif // ZCL_EZMODE

#ifdef ZCL_ZONE
/*******************************************************************************
* @fn      static void zclSampleCIE_ChangeNotificationCB
*
* @brief   Process incoming Change Notification Command
*
* @param   Zone Status Change Notification command
*
* @return  none
*/
static ZStatus_t zclSampleCIE_ChangeNotificationCB(zclZoneChangeNotif_t *pCmd, afAddrType_t *srcAddr )
{
  
  //building the warning msg
  zclWDStartWarning_t alarm;
  alarm.warningDuration = SS_IAS_WD_ALARM_DEFAULT_DURATION;
  
  //displaying messages
  switch(pCmd->zoneStatus)
  {
    //Here only implementing fire alarm (1st bit), other bits not checked
  case 0://since zoneStatus is a bit mask
    {
      HalLcdWriteString ("No Fires @", HAL_LCD_LINE_2);
      HalLcdWriteStringValue ("Address: ", (uint16)(srcAddr->addr.shortAddr), 16, HAL_LCD_LINE_3);

      //setting params for the Warning Message
      alarm.warningmessage.warningbits.warnMode = SS_IAS_START_WARNING_WARNING_MODE_STOP;
      alarm.warningmessage.warningbits.warnStrobe = SS_IAS_START_WARNING_STROBE_NO_STROBE_WARNING;   
    }
    break;
    
  case SS_IAS_ZONE_STATUS_ALARM1_ALARMED:
    {
      HalLcdWriteStringValue( "Alarm @: ", (uint16)(srcAddr->addr.shortAddr), 16, HAL_LCD_LINE_2);
      HalLcdWriteString ("Alerting WDs", HAL_LCD_LINE_3);
      
      //setting params for the Warning Message
      alarm.warningmessage.warningbits.warnMode = SS_IAS_START_WARNING_WARNING_MODE_FIRE;
      alarm.warningmessage.warningbits.warnStrobe = SS_IAS_START_WARNING_STROBE_USE_STPOBE_IN_PARALLEL_TO_WARNING;
    }
    break;
    
  default:
    break;
  }
  
  zclSendWarningToAllWD(&alarm);
  return ZSuccess ;
}



/*******************************************************************************
* @fn      static void zclSampleCIE_EnrollRequestCB
*
* @brief   Process incoming Zone Enroll Request Command
*
* @param   Zone Enroll Request command
*
* @return  none
*/
static ZStatus_t zclSampleCIE_EnrollRequestCB(zclZoneEnrollReq_t *pReq, uint8 endpoint )
{
  //create temporary placeholder for IEEE address
  uint8 extAddr[Z_EXTADDR_LEN];
  
  //copy the IEEE Address to the temp var
  if(APSME_LookupExtAddr(pReq->srcAddr->addr.shortAddr, extAddr))
  {    
    //fill in the enrolling Device IEEE Address in the zone record
    zclSS_UpdateZoneAddress( pReq->srcAddr->endPoint, pReq->zoneID, extAddr );
    
    HalLcdWriteString ("Zone Added", HAL_LCD_LINE_2);
    HalLcdWriteString ("Succesfully", HAL_LCD_LINE_3);
  }

  return ZSuccess ;
}

/*******************************************************************************
* @fn      static void zclSampleCIE_WriteIAS_CIE_Address
*
* @brief   Write to the IAS_CIE_Addr attribute on the Zone server
*
* @param   none
*
* @return  none
*/

static void zclSampleCIE_WriteIAS_CIE_Address(afAddrType_t *dstAddr)
{
  //only one attrib to write
  uint8 numAttr = 1;
  uint8 hdrLen;
  uint8 dataLen = zclGetDataTypeLength(ZCL_DATATYPE_IEEE_ADDR);
  
  // calculate the length of the cmd
  hdrLen = sizeof( zclWriteCmd_t ) + ( numAttr * sizeof( zclWriteRec_t ) );
  zclWriteCmd_t *writeCmd = (zclWriteCmd_t *)osal_mem_alloc( hdrLen + dataLen );
  
  //Building the command and sending it
  if ( writeCmd != NULL )
  {
    writeCmd->numAttr = numAttr;
    zclWriteRec_t *attrRec = &(writeCmd->attrList[0]);
    attrRec->attrID = ATTRID_SS_IAS_CIE_ADDRESS;
    attrRec->dataType = ZCL_DATATYPE_IEEE_ADDR;
    attrRec->attrData = NLME_GetExtAddr();
    //sending the write attrib command
    zcl_SendWrite(SAMPLECIE_ENDPOINT, dstAddr,ZCL_CLUSTER_ID_SS_IAS_ZONE,
                  writeCmd,ZCL_FRAME_CLIENT_SERVER_DIR,0,1 );
    
    osal_mem_free( writeCmd );
  }
}

#endif //ZCL_ZONE


#ifdef ZCL_ACE

/*********************************************************************
* @fn      zclSampleCIE_ArmCB
*
* @brief   Process in the received Arm Command.
*
* @param   armMode
*
*/
static uint8 zclSampleCIE_ArmCB( zclACEArm_t* payload )
{
  //Process the ACE Arm command
  switch(payload->armMode)
  {
  case SS_IAS_ACE_ARM_DISARM:
    //set the CIE to disarmed
    break;
  case SS_IAS_ACE_ARM_DAY_HOME_ZONES_ONLY:
    
    break;
  case SS_IAS_ACE_ARM_NIGHT_SLEEP_ZONES_ONLY:
    
    break;
  case SS_IAS_ACE_ARM_ALL_ZONES:
    
    break;
  }
  
  return (ZSuccess);
}

/*********************************************************************
* @fn      zclSampleCIE_BypassCB
*
* @brief   Process in the received Bypass Command.
*
* @param   zclACEBypass_t *pCmd
*
*/
static ZStatus_t zclSampleCIE_BypassCB(zclACEBypass_t *pCmd)
{
  //Process the ACE Bypass cmd
  return ZSuccess ;
}


/*******************************************************************************
* @fn      static void zclSampleCIE_FireCB
*
* @brief   Process incoming Fire Command
*
* @param   none
*
* @return  none
*/
static ZStatus_t zclSampleCIE_FireCB( void)
{
  //Make the lcd display the information about a fire being detected
  return ZSuccess ;
  
}

/*******************************************************************************
* @fn      static void zclSampleCIE_EmergencyCB
*
* @brief   Process incoming Emergency Command
*
* @param   none
*
* @return  none
*/
static ZStatus_t zclSampleCIE_EmergencyCB(void)
{
  //display emergency state on LCD
  return ZSuccess ;
}

/*******************************************************************************
* @fn      static void zclSampleCIE_PanicCB
*
* @brief   Process incoming Panic Command
*
* @param   none
*
* @return  none
*/
static ZStatus_t zclSampleCIE_PanicCB(void)
{
  //display Panic state on LCD
  return ZSuccess ;
}



#endif //ZCL_ACE

#ifdef ZCL_WD
/*******************************************************************************

* @fn      static void zclSendSquawkToAllWD
*
* @brief   Send Squawk Signal to all Warning Devices in the table
*
* @param   none
*
* @return  none
*/
static void zclSendSquawkToAllWD(zclWDSquawk_t *squawk)
{ 
  // use the binding table as dstAddr
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  dstAddr.endPoint = 0;
  dstAddr.addr.shortAddr = 0;

  zclSS_Send_IAS_WD_SquawkCmd(SAMPLECIE_ENDPOINT, &dstAddr, squawk, 0, 1);
}

/*******************************************************************************

* @fn      static void zclSendWarningToAllWD
*
* @brief   sends warning message to all WD in WD table
*
* @param   uint16 network addr of alarm origininator
*
* @return  none
*/
static void zclSendWarningToAllWD(zclWDStartWarning_t *alarm)
{
  // use the binding table as dstAddr
  afAddrType_t dstAddr;
  dstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  dstAddr.endPoint = 0;
  dstAddr.addr.shortAddr = 0;
  
  zclSS_Send_IAS_WD_StartWarningCmd(SAMPLECIE_ENDPOINT, &dstAddr, alarm, 1, 1);
}

#endif //ZCL_WD

/****************************************************************************
****************************************************************************/