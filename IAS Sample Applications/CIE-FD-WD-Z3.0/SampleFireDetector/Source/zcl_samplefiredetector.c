/******************************************************************************

@file zcl_samplefiredetector.c

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

/*********************************************************************
  This device will act as a fire detector.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Start BDB Commissioning
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

#include "bdb.h"
#include "bdb_interface.h"
#include "gp_interface.h"

#if defined ( INTER_PAN )
#if defined ( BDB_TL_INITIATOR )
  #include "bdb_touchlink_initiator.h"
#endif // BDB_TL_INITIATOR
#if defined ( BDB_TL_TARGET )
  #include "bdb_touchlink_target.h"
#endif // BDB_TL_TARGET
#endif // INTER_PAN

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

uint8 zoneid ;

#define ZCLSAMPLEFIREDETECTOR_BINDINGLIST        2

devStates_t zclSampleFireDetector_NwkState = DEV_INIT;

uint8 giFireDetectorScreenMode = FIREDETECTOR_MAINMODE;   // display main screen mode first

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleFireDetector_TestEp =
{
  SAMPLEFIREDETECTOR_ENDPOINT,                                 // Test endpoint
  0,
  &zclSampleFireDetector_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

const char sDeviceName[]   = "  FireDetector";


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleFireDetector_HandleKeys( byte shift, byte keys );
static void zclSampleFireDetector_BasicResetCB( void );
static void zclSampleFireDetector_ProcessIdentifyTimeChange( uint8 endpoint );

// app display functions
void zclSampleFireDetector_LcdDisplayUpdate(void);
void zclSampleFireDetector_LcdDisplayMainMode(void);

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

static void zclSampleFireDetector_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);
#if (ZG_BUILD_JOINING_TYPE)
static void zclSampleFireDetector_CBKE(void);
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleFireDetector_CmdCallbacks =
{
  zclSampleFireDetector_BasicResetCB,        // Basic Cluster Reset command
  NULL,                                           // Identify Trigger Effect command
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
  bdb_RegisterSimpleDescriptor( &zclSampleFireDetector_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLEFIREDETECTOR_ENDPOINT, &zclSampleFireDetector_CmdCallbacks );

  // Register the ZCL Security and Safety Cluster Library callback functions
  zclSS_RegisterCmdCallbacks( SAMPLEFIREDETECTOR_ENDPOINT, &zclSampleFireDetector_SSCmdCallbacks );
    
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLEFIREDETECTOR_ENDPOINT, SAMPLEFIREDETECTOR_MAX_ATTRIBUTES, zclSampleFireDetector_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleFireDetector_TaskID );
  
  bdb_RegisterCommissioningStatusCB(zclSampleFireDetector_ProcessCommissioningStatus);
  bdb_RegisterIdentifyTimeChangeCB(zclSampleFireDetector_ProcessIdentifyTimeChange );
  #if (ZG_BUILD_JOINING_TYPE)
  bdb_RegisterCBKETCLinkKeyExchangeCB(zclSampleFireDetector_CBKE);
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
    zclSampleFireDetector_ProcessIdentifyTimeChange(SAMPLEFIREDETECTOR_ENDPOINT);

    return ( events ^ SAMPLEFIREDETECTOR_IDENTIFY_TIMEOUT_EVT );
  }

  if ( events & SAMPLEFIREDETECTOR_MAIN_SCREEN_EVT )
  {
    giFireDetectorScreenMode = FIREDETECTOR_MAINMODE;
    zclSampleFireDetector_LcdDisplayUpdate();

    return ( events ^ SAMPLEFIREDETECTOR_MAIN_SCREEN_EVT );
  }

#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SAMPLEFIREDETECTOR_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ SAMPLEFIREDETECTOR_END_DEVICE_REJOIN_EVT );
  }
#endif

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
    // network formation + steering, i.e. form or join a network
    // finding & binding, find matching clusters + make binds
    bdb_StartCommissioning( BDB_COMMISSIONING_MODE_NWK_FORMATION | BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING );
    HalLcdWriteString( "Commissioning...", HAL_LCD_LINE_2 );
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
    // start a fire
    zclSampleFireDetector_SendChangeNotification();
  }

  if ( keys & HAL_KEY_SW_5 )
  {
    // leave (if applicable) + factory new reset
    bdb_resetLocalAction();
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
 * @fn      zclSampleLight_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSampleFireDetector_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
{
  switch(bdbCommissioningModeMsg->bdbCommissioningMode)
  {
    case BDB_COMMISSIONING_FORMATION:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //After formation, perform nwk steering again plus the remaining commissioning modes that has not been process yet
        bdb_StartCommissioning(BDB_COMMISSIONING_MODE_NWK_STEERING | bdbCommissioningModeMsg->bdbRemainingCommissioningModes);
      }
      else
      {
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_NWK_STEERING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
        //We are on the nwk, what now?
      }
      else
      {
        //See the possible errors for nwk steering procedure
        //No suitable networks found
        //Want to try other channels?
        //try with bdb_setChannelAttribute
      }
    break;
    case BDB_COMMISSIONING_FINDING_BINDING:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
      {
        //YOUR JOB:
      }
      else
      {
        //YOUR JOB:
        //retry?, wait for user interaction?
      }
    break;
    case BDB_COMMISSIONING_INITIALIZATION:
      //Initialization notification can only be successful. Failure on initialization
      //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification

      //YOUR JOB:
      //We are on a network, what now?

    break;
#if ZG_BUILD_ENDDEVICE_TYPE    
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        osal_start_timerEx(zclSampleFireDetector_TaskID, SAMPLEFIREDETECTOR_END_DEVICE_REJOIN_EVT, SAMPLEFIREDETECTOR_END_DEVICE_REJOIN_DELAY);
      }
    break;
#endif 
  }
}

#if (ZG_BUILD_JOINING_TYPE)

/*********************************************************************
 * @fn      zclSampleLight_CBKE
 *
 * @brief   Callback function in which application performs Certificate Based Key Exchange.
 *          The result from this operation must be notified to using the 
 *          bdb_CBKETCLinkKeyExchangeAttempt interface.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleFireDetector_CBKE(void)
{
  bdb_CBKETCLinkKeyExchangeAttempt(FALSE);
}
#endif

/*********************************************************************
 * @fn      zclSampleFireDetector_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleFireDetector_ProcessIdentifyTimeChange( uint8 endpoint )
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
    case ZCL_CMD_CONFIG_REPORT:
    case ZCL_CMD_CONFIG_REPORT_RSP:
    case ZCL_CMD_READ_REPORT_CFG:
    case ZCL_CMD_READ_REPORT_CFG_RSP:
    case ZCL_CMD_REPORT:
      //bdb_ProcessIncomingReportingMsg( pInMsg );
      break;
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
    if(writeRspCmd->attrList[i].attrID == ATTRID_SS_IAS_CIE_ADDRESS)
    {
      afAddrType_t zclCIE_Addr;

      zclCIE_Addr.addrMode = (afAddrMode_t)(Addr64Bit);
      osal_memcpy(zclCIE_Addr.addr.extAddr, zclSampleFireDetector_IAS_CIE_Address, Z_EXTADDR_LEN);
      zclCIE_Addr.endPoint = AF_BROADCAST_ENDPOINT;

      zclSS_IAS_Send_ZoneStatusEnrollRequestCmd( SAMPLEFIREDETECTOR_ENDPOINT,
                                                 &zclCIE_Addr,
                                                 zclSampleFireDetector_ZoneType,
                                                 22, 
                                                 0, 
                                                 1 );
    }
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
                                                 &zclSampleFireDetector_DstAddr,
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


