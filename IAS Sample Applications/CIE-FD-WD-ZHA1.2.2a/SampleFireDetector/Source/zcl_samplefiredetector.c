/******************************************************************************

@file zcl_samplefiredetector.c

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
  This device will act as a fire detector.

  SCREEN MODES
  ----------------------------------------
    - SW1: Start EZ-Mode Commissioning
    - SW2: [unassigned]
    - SW3: Send IAS Zone Enrollment request to CIE 
    - SW4: Send change notification, i.e. start detecting a fire
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


#include "zcl_samplefiredetector.h"

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
byte zclSampleFireDetector_TaskID;

uint8 zclSampleFireDetectorSeqNum;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleFireDetector_DstAddr;
static afAddrType_t zclCIE_IEEE_Addr;
uint8 zoneid ;

#ifdef ZCL_EZMODE
static void zclSampleFireDetector_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg );
static void zclSampleFireDetector_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData );

static const zclEZMode_RegisterData_t zclSampleFireDetector_RegisterEZModeData =
{
  &zclSampleFireDetector_TaskID,
  SAMPLEFIREDETECTOR_EZMODE_NEXTSTATE_EVT,
  SAMPLEFIREDETECTOR_EZMODE_TIMEOUT_EVT,
  &zclSampleFireDetectorSeqNum,
  zclSampleFireDetector_EZModeCB
};

// NOT ZCL_EZMODE, Use EndDeviceBind
#else

static cId_t bindingOutClusters[] =
{
  ZCL_CLUSTER_ID_SS_IAS_ZONE,
  ZCL_CLUSTER_ID_SS_IAS_WD
};
#define ZCLSAMPLEFIREDETECTOR_BINDINGLIST        2
#endif

devStates_t zclSampleFireDetector_NwkState = DEV_INIT;

uint8 giFireDetectorScreenMode = FIREDETECTOR_MAINMODE;   // display main screen mode first

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleFireDetector_TestEp =
{
  SAMPLEFIREDETECTOR_ENDPOINT,                                 // Test endpoint
  &zclSampleFireDetector_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleFireDetector_HandleKeys( byte shift, byte keys );
static void zclSampleFireDetector_BasicResetCB( void );
static void zclSampleFireDetector_IdentifyCB( zclIdentify_t *pCmd );
static void zclSampleFireDetector_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp );
static void zclSampleFireDetector_ProcessIdentifyTimeChange( void );

// app display functions
void zclSampleFireDetector_LcdDisplayUpdate(void);
void zclSampleFireDetector_LcdDisplayMainMode(void);
void zclSampleFireDetector_LcdDisplayHelpMode(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleFireDetector_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleFireDetector_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleFireDetector_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleFireDetector_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleFireDetector_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleFireDetector_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleFireDetector_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif // ZCL_DISCOVER

#ifdef ZCL_ZONE
static void zclSampleFireDetector_SendChangeNotification(void);
static ZStatus_t zclSampleFireDetector_EnrollResponseCB(zclZoneEnrollRsp_t *rsp);
#endif

/*********************************************************************
 * STATUS STRINGS
 */
#ifdef LCD_SUPPORTED
const char sClearLine[]    = " ";
const char sDeviceName[]   = "  FireDetector";
const char sSwFireDetector[]= "SW1: Fire Toggle";
const char sSwEZMode[]     = "SW2: EZ-Mode";
const char sSwHelp[]       = "SW5: Help";
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleFireDetector_CmdCallbacks =
{
  zclSampleFireDetector_BasicResetCB,        // Basic Cluster Reset command
  zclSampleFireDetector_IdentifyCB,          // Identify command
#ifdef ZCL_EZMODE
  NULL,                                           // Identify EZ-Mode Invoke command
  NULL,                                           // Identify Update Commission State command
#endif
  NULL,                                           // Identify Trigger Effect command
  zclSampleFireDetector_IdentifyQueryRspCB,  // Identify Query Response command
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
static zclSS_AppCallbacks_t zclSampleFireDetector_SSCmdCallbacks =
{
  NULL,                                         // Change Notification command
  NULL,                                         // Enroll Request command
  zclSampleFireDetector_EnrollResponseCB,       // Enroll Reponse command
  NULL,                                         // Initiate Normal Operation Mode command
  NULL,                                         // Initiate Test Mode command
  NULL,                                         // Arm command
  NULL,                                         // Bypass command
  NULL,                                         // Emergency command
  NULL,                                         // Fire command
  NULL,                                         // Panic command
  NULL,                                         // Get Zone ID Map command
  NULL,                                         // Get Zone Information Command
  NULL,                                         // Get Panel Status Command
  NULL,                                         // Get Bypassed Zone List Command
  NULL,                                         // Get Zone Status Command
  NULL,                                         // ArmResponse command
  NULL,                                         // Get Zone ID Map Response command     
  NULL,                                         // Get Zone Information Response command     
  NULL,                                         // Zone Status Changed command     
  NULL,                                         // Panel Status Changed command     
  NULL,                                         // Get Panel Status Response command     
  NULL,                                         // Set Bypassed Zone List command     
  NULL,                                         // Bypass Response command     
  NULL,                                         // Get Zone Status Response command     
  NULL,                                         // Start Warning command
  NULL                                          // Squawk command                              
};

/*********************************************************************
 * @fn          zclSampleFireDetector_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleFireDetector_Init( byte task_id )
{
  zclSampleFireDetector_TaskID = task_id;

  // Set destination address to indirect
  zclSampleFireDetector_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleFireDetector_DstAddr.endPoint = 0;
  zclSampleFireDetector_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  zclHA_Init( &zclSampleFireDetector_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLEFIREDETECTOR_ENDPOINT, &zclSampleFireDetector_CmdCallbacks );

  // Register the ZCL Security and Safety Cluster Library callback functions
  zclSS_RegisterCmdCallbacks( SAMPLEFIREDETECTOR_ENDPOINT, &zclSampleFireDetector_SSCmdCallbacks );
    
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLEFIREDETECTOR_ENDPOINT, SAMPLEFIREDETECTOR_MAX_ATTRIBUTES, zclSampleFireDetector_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleFireDetector_TaskID );

#ifdef ZCL_EZMODE
  // Register EZ-Mode
  zcl_RegisterEZMode( &zclSampleFireDetector_RegisterEZModeData );

  // Register with the ZDO to receive Match Descriptor Responses
  ZDO_RegisterForZDOMsg(task_id, Match_Desc_rsp);
#endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleFireDetector_TaskID );

  // Register for a test endpoint
  afRegister( &sampleFireDetector_TestEp );

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
uint16 zclSampleFireDetector_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleFireDetector_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
#ifdef ZCL_EZMODE
        case ZDO_CB_MSG:
          zclSampleFireDetector_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
#endif

        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleFireDetector_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleFireDetector_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleFireDetector_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( (zclSampleFireDetector_NwkState == DEV_ZB_COORD) ||
               (zclSampleFireDetector_NwkState == DEV_ROUTER)   ||
               (zclSampleFireDetector_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giFireDetectorScreenMode = FIREDETECTOR_MAINMODE;
            zclSampleFireDetector_LcdDisplayUpdate();
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

  if ( events & SAMPLEFIREDETECTOR_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleFireDetector_IdentifyTime > 0 )
      zclSampleFireDetector_IdentifyTime--;
    zclSampleFireDetector_ProcessIdentifyTimeChange();

    return ( events ^ SAMPLEFIREDETECTOR_IDENTIFY_TIMEOUT_EVT );
  }

#ifdef ZCL_EZMODE
  // going on to next state
  if ( events & SAMPLEFIREDETECTOR_EZMODE_NEXTSTATE_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_PROCESS, NULL );   // going on to next state
    return ( events ^ SAMPLEFIREDETECTOR_EZMODE_NEXTSTATE_EVT );
  }

  // the overall EZMode timer expired, so we timed out
  if ( events & SAMPLEFIREDETECTOR_EZMODE_TIMEOUT_EVT )
  {
    zcl_EZModeAction ( EZMODE_ACTION_TIMED_OUT, NULL ); // EZ-Mode timed out
    return ( events ^ SAMPLEFIREDETECTOR_EZMODE_TIMEOUT_EVT );
  }
#endif // ZLC_EZMODE

  if ( events & SAMPLEFIREDETECTOR_MAIN_SCREEN_EVT )
  {
    giFireDetectorScreenMode = FIREDETECTOR_MAINMODE;
    zclSampleFireDetector_LcdDisplayUpdate();

    return ( events ^ SAMPLEFIREDETECTOR_MAIN_SCREEN_EVT );
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
 * @fn      zclSampleFireDetector_HandleKeys
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
static void zclSampleFireDetector_HandleKeys( byte shift, byte keys )
{
  
  HalLcdWriteString ("", HAL_LCD_LINE_2);
  HalLcdWriteString ("", HAL_LCD_LINE_3);

  if ( keys & HAL_KEY_SW_1 )
  {
#ifdef ZCL_EZMODE
      zclEZMode_InvokeData_t ezModeData;
      static uint16 clusterIDs[] = { ZCL_CLUSTER_ID_SS_IAS_ZONE };

      // Invoke EZ-Mode
      ezModeData.endpoint = SAMPLEFIREDETECTOR_ENDPOINT; // endpoint on which to invoke EZ-Mode
      if ( ( zclSampleFireDetector_NwkState == DEV_ZB_COORD ) ||
           ( zclSampleFireDetector_NwkState == DEV_ROUTER )   ||
           ( zclSampleFireDetector_NwkState == DEV_END_DEVICE ) )
      {
        ezModeData.onNetwork = TRUE;      // node is already on the network
      }
      else
      {
        ezModeData.onNetwork = FALSE;     // node is not yet on the network
      }
      ezModeData.initiator = TRUE;        //  initiator
      ezModeData.numActiveInClusters = 1;
      ezModeData.pActiveInClusterIDs = clusterIDs;
      ezModeData.numActiveOutClusters = 0;   // active output cluster
      ezModeData.pActiveOutClusterIDs = NULL;
      zcl_InvokeEZMode( &ezModeData );

#ifdef LCD_SUPPORTED
      HalLcdWriteString( "EZMode", HAL_LCD_LINE_2 );
#endif

#endif // ZCL_EZMODE
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    
  }

  if ( keys & HAL_KEY_SW_3 )
  {
    // zone enrollment request to CIE, only perform after writing cie address attribute value
    afAddrType_t zclCIE_Addr;

    zclCIE_Addr.addrMode = (afAddrMode_t)(Addr64Bit);
    osal_memcpy(zclCIE_Addr.addr.extAddr, zclSampleFireDetector_IAS_CIE_Address, Z_EXTADDR_LEN);
    zclCIE_Addr.endPoint = AF_BROADCAST_ENDPOINT;

    zclSS_IAS_Send_ZoneStatusEnrollRequestCmd( SAMPLEFIREDETECTOR_ENDPOINT,
                                               &zclCIE_Addr,
                                               zclSampleFireDetector_ZoneType,
                                               22, 0, 1);
  }

  if ( keys & HAL_KEY_SW_4 )
  {
    // Fire Alarm
    zclSampleFireDetector_SendChangeNotification() ;
  }
  
  if ( keys & HAL_KEY_SW_5 )
  {
    // factory reset
    zclSampleFireDetector_BasicResetCB();
  }
}

/*********************************************************************
 * @fn      zclSampleFireDetector_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleFireDetector_LcdDisplayUpdate( void )
{
 
    zclSampleFireDetector_LcdDisplayMainMode();
  
}

/*********************************************************************
 * @fn      zclSampleFireDetector_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleFireDetector_LcdDisplayMainMode( void )
{
  if ( zclSampleFireDetector_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSampleFireDetector_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSampleFireDetector_NwkState == DEV_END_DEVICE )
  {
    zclHA_LcdStatusLine1( 2 );
  }
}

/*********************************************************************
 * @fn      zclSampleFireDetector_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleFireDetector_ProcessIdentifyTimeChange( void )
{ 
  
  if ( zclSampleFireDetector_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleFireDetector_TaskID, SAMPLEFIREDETECTOR_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleFireDetector_ZoneStatus )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclSampleFireDetector_TaskID, SAMPLEFIREDETECTOR_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleFireDetector_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleFireDetector_BasicResetCB( void )
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
  osal_start_timerEx( zclSampleFireDetector_TaskID, RESET_EVT, 3000);
}

/*********************************************************************
 * @fn      zclSampleFireDetector_IdentifyCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Command for this application.
 *
 * @param   srcAddr - source address and endpoint of the response message
 * @param   identifyTime - the number of seconds to identify yourself
 *
 * @return  none
 */
static void zclSampleFireDetector_IdentifyCB( zclIdentify_t *pCmd )
{
  zclSampleFireDetector_IdentifyTime = pCmd->identifyTime;
  zclSampleFireDetector_ProcessIdentifyTimeChange();
}

/*********************************************************************
 * @fn      zclSampleFireDetector_IdentifyQueryRspCB
 *
 * @brief   Callback from the ZCL General Cluster Library when
 *          it received an Identity Query Response Command for this application.
 *
 * @param   srcAddr - requestor's address
 * @param   timeout - number of seconds to identify yourself (valid for query response)
 *
 * @return  none
 */
static void zclSampleFireDetector_IdentifyQueryRspCB( zclIdentifyQueryRsp_t *pRsp )
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
 * @fn      zclSampleFireDetector_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleFireDetector_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleFireDetector_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleFireDetector_ProcessInWriteRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_REPORT
    // See ZCL Test Applicaiton (zcl_testapp.c) for sample code on Attribute Reporting
    case ZCL_CMD_CONFIG_REPORT:
      //zclSampleFireDetector_ProcessInConfigReportCmd( pInMsg );
      break;

    case ZCL_CMD_CONFIG_REPORT_RSP:
      //zclSampleFireDetector_ProcessInConfigReportRspCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG:
      //zclSampleFireDetector_ProcessInReadReportCfgCmd( pInMsg );
      break;

    case ZCL_CMD_READ_REPORT_CFG_RSP:
      //zclSampleFireDetector_ProcessInReadReportCfgRspCmd( pInMsg );
      break;

    case ZCL_CMD_REPORT:
      //zclSampleFireDetector_ProcessInReportCmd( pInMsg );
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSampleFireDetector_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleFireDetector_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleFireDetector_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleFireDetector_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleFireDetector_ProcessInDiscAttrsExtRspCmd( pInMsg );
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
 * @fn      zclSampleFireDetector_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleFireDetector_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleFireDetector_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleFireDetector_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleFireDetector_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleFireDetector_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleFireDetector_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleFireDetector_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleFireDetector_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleFireDetector_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleFireDetector_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleFireDetector_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleFireDetector_ProcessZDOMsgs
 *
 * @brief   Called when this node receives a ZDO/ZDP response.
 *
 * @param   none
 *
 * @return  status
 */
static void zclSampleFireDetector_ProcessZDOMsgs( zdoIncomingMsg_t *pMsg )
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
  }
}

/*********************************************************************
 * @fn      zclSampleFireDetector_EZModeCB
 *
 * @brief   The Application is informed of events. This can be used to show on the UI what is
*           going on during EZ-Mode steering/finding/binding.
 *
 * @param   state - an
 *
 * @return  none
 */
static void zclSampleFireDetector_EZModeCB( zlcEZMode_State_t state, zclEZMode_CBData_t *pData )
{
#ifdef LCD_SUPPORTED
  char szLine[20];
  char *pStr;
  uint8 err;
#endif

  // time to go into identify mode
  if ( state == EZMODE_STATE_IDENTIFYING )
  {
    zclSampleFireDetector_IdentifyTime = ( EZMODE_TIME / 1000 );  // convert to seconds
    zclSampleFireDetector_ProcessIdentifyTimeChange();
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
    zclSampleFireDetector_IdentifyTime = 0;
    zclSampleFireDetector_ProcessIdentifyTimeChange();

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
    osal_start_timerEx( zclSampleFireDetector_TaskID, SAMPLEFIREDETECTOR_MAIN_SCREEN_EVT, 3000 );

  }
}
#endif // ZCL_EZMODE

#ifdef ZCL_ZONE
/*******************************************************************************
* @fn      static void zclSampleFireDetector_EnrollResponseCB
*
* @brief   Process incoming Enroll Response Command
*
* @param   Zone Enroll Response command
*
* @return  none
*/
static ZStatus_t zclSampleFireDetector_EnrollResponseCB(zclZoneEnrollRsp_t *rsp)
{
  switch(rsp->responseCode)
  {
  case SS_IAS_ZONE_STATUS_ENROLL_RESPONSE_CODE_SUCCESS:
    
    //Must set the Zone State attribute to Enrolled if succesful
    zclSampleFireDetector_ZoneState = SS_IAS_ZONE_STATE_ENROLLED;
    zoneid = rsp->zoneID;
    
    HalLcdWriteString ("Zone", HAL_LCD_LINE_2);
    HalLcdWriteString ("Enrolled", HAL_LCD_LINE_3); 
    
    //must authentice received messages by checking address of sender
    //against IAS_CIE_ADDRESS
    break;
    
    //The rest of the cases indicate that the enrollment was not succesful
    //dealt by printing error message
  default:
    break;
  }
  
  return (ZStatus_t)(0)  ;
}


/*******************************************************************************
* @fn      static void zclSampleFireDetector_SendChangeNotification
*
* @brief   Process incoming Enroll Response Command
*
* @param   Zone Enroll Response command
*
* @return  none
*/
static void zclSampleFireDetector_SendChangeNotification( void)
{
  // Using this as the "Alarm Switch"
  //Toggle the zone status attrib with button press
  zclSampleFireDetector_ZoneStatus ^= SS_IAS_ZONE_STATUS_ALARM1_ALARMED;
  
  //generates a Zone Status Change Notification Command
  zclSS_IAS_Send_ZoneStatusChangeNotificationCmd(SAMPLEFIREDETECTOR_ENDPOINT,
                                                 &zclCIE_IEEE_Addr,
                                                 zclSampleFireDetector_ZoneStatus, 0, zoneid,0,
                                                 1, 1 );

  if(zclSampleFireDetector_ZoneStatus == SS_IAS_ZONE_STATUS_ALARM1_ALARMED)
  {
    HalLcdWriteString ("Fire Detected", HAL_LCD_LINE_2);
  }
  else
  {
    HalLcdWriteString ("Alarm Stopped", HAL_LCD_LINE_2);   
  }
    
}

#endif //ZCL_ZONE

/****************************************************************************
****************************************************************************/


