/******************************************************************************

@file zcl_samplewarningdevice.c

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
  This device will act as a warning Device.

  SCREEN MODES
  ----------------------------------------
  Main:
    - SW1: Start BDB Commissioning
    - SW2: [unassigned] 
    - SW3: [unassigned] 
    - SW4: [unassigned] 
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
#include "zcl_samplewarningdevice.h"

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
#endif // 

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
byte zclSampleWarningDevice_TaskID;

uint8 zclSampleWarningDeviceSeqNum;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
afAddrType_t zclSampleWarningDevice_DstAddr;

uint8 zoneid ;

#define ZCLSAMPLEWARNINGDEVICE_BINDINGLIST        2

devStates_t zclSampleWarningDevice_NwkState = DEV_INIT;

uint8 giWarningDeviceScreenMode = WARNINGDEVICE_MAINMODE;   // display main screen mode first

// Test Endpoint to allow SYS_APP_MSGs
static endPointDesc_t sampleWarningDevice_TestEp =
{
  SAMPLEWARNINGDEVICE_ENDPOINT,                                 // Test endpoint
  0,
  &zclSampleWarningDevice_TaskID,
  (SimpleDescriptionFormat_t *)NULL,  // No Simple description for this test endpoint
  (afNetworkLatencyReq_t)0            // No Network Latency req
};

const char sDeviceName[]   = "  WarningDevice";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSampleWarningDevice_HandleKeys( byte shift, byte keys );
static void zclSampleWarningDevice_BasicResetCB( void );
static void zclSampleWarningDevice_ProcessIdentifyTimeChange( uint8 endpoint );

// app display functions
void zclSampleWarningDevice_LcdDisplayUpdate(void);
void zclSampleWarningDevice_LcdDisplayMainMode(void);

// Functions to process ZCL Foundation incoming Command/Response messages
static void zclSampleWarningDevice_ProcessIncomingMsg( zclIncomingMsg_t *msg );
#ifdef ZCL_READ
static uint8 zclSampleWarningDevice_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8 zclSampleWarningDevice_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg );
#endif
static uint8 zclSampleWarningDevice_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8 zclSampleWarningDevice_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleWarningDevice_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg );
static uint8 zclSampleWarningDevice_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg );
#endif // ZCL_DISCOVER

#ifdef ZCL_WD
//WD Cluster
static ZStatus_t zclSampleWarningDevice_StartWarningCB(zclWDStartWarning_t *pCmd);
static ZStatus_t zclSampleWarningDevice_SquawkCB(zclWDSquawk_t *squawk);
#endif

static void zclSampleWarningDevice_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);
#if (ZG_BUILD_JOINING_TYPE)
static void zclSampleWarningDevice_CBKE(void);
#endif

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSampleWarningDevice_CmdCallbacks =
{
  zclSampleWarningDevice_BasicResetCB,        // Basic Cluster Reset command
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
static zclSS_AppCallbacks_t zclSampleWarningDevice_SSCmdCallbacks =
{
  NULL,                                         // Change Notification command
  NULL,                                         // Enroll Request command
  NULL,                                         // Enroll Reponse command
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
  zclSampleWarningDevice_StartWarningCB,        // Start Warning command
  zclSampleWarningDevice_SquawkCB               // Squawk command                  
};

/*********************************************************************
 * @fn          zclSampleWarningDevice_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */
void zclSampleWarningDevice_Init( byte task_id )
{
  zclSampleWarningDevice_TaskID = task_id;

  // Set destination address to indirect
  zclSampleWarningDevice_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  zclSampleWarningDevice_DstAddr.endPoint = 0;
  zclSampleWarningDevice_DstAddr.addr.shortAddr = 0;

  // This app is part of the Home Automation Profile
  bdb_RegisterSimpleDescriptor( &zclSampleWarningDevice_SimpleDesc );

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SAMPLEWARNINGDEVICE_ENDPOINT, &zclSampleWarningDevice_CmdCallbacks );

  // Register the ZCL Security and Safety Cluster Library callback functions
  zclSS_RegisterCmdCallbacks( SAMPLEWARNINGDEVICE_ENDPOINT, &zclSampleWarningDevice_SSCmdCallbacks );
    
  // Register the application's attribute list
  zcl_registerAttrList( SAMPLEWARNINGDEVICE_ENDPOINT, SAMPLEWARNINGDEVICE_MAX_ATTRIBUTES, zclSampleWarningDevice_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zcl_registerForMsg( zclSampleWarningDevice_TaskID );
  
  bdb_RegisterCommissioningStatusCB(zclSampleWarningDevice_ProcessCommissioningStatus);
  bdb_RegisterIdentifyTimeChangeCB(zclSampleWarningDevice_ProcessIdentifyTimeChange );
  #if (ZG_BUILD_JOINING_TYPE)
  bdb_RegisterCBKETCLinkKeyExchangeCB(zclSampleWarningDevice_CBKE);
  #endif

  // Register for all key events - This app will handle all key events
  RegisterForKeys( zclSampleWarningDevice_TaskID );

  // Register for a test endpoint
  afRegister( &sampleWarningDevice_TestEp );

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
uint16 zclSampleWarningDevice_event_loop( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    while ( (MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( zclSampleWarningDevice_TaskID )) )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZCL_INCOMING_MSG:
          // Incoming ZCL Foundation command/response messages
          zclSampleWarningDevice_ProcessIncomingMsg( (zclIncomingMsg_t *)MSGpkt );
          break;

        case KEY_CHANGE:
          zclSampleWarningDevice_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case ZDO_STATE_CHANGE:
          zclSampleWarningDevice_NwkState = (devStates_t)(MSGpkt->hdr.status);


          // now on the network
          if ( (zclSampleWarningDevice_NwkState == DEV_ZB_COORD) ||
               (zclSampleWarningDevice_NwkState == DEV_ROUTER)   ||
               (zclSampleWarningDevice_NwkState == DEV_END_DEVICE) )
          {
#ifndef HOLD_AUTO_START
            giWarningDeviceScreenMode = WARNINGDEVICE_MAINMODE;
            zclSampleWarningDevice_LcdDisplayUpdate();
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

  if ( events & SAMPLEWARNINGDEVICE_IDENTIFY_TIMEOUT_EVT )
  {
    if ( zclSampleWarningDevice_IdentifyTime > 0 )
      zclSampleWarningDevice_IdentifyTime--;
    zclSampleWarningDevice_ProcessIdentifyTimeChange(SAMPLEWARNINGDEVICE_ENDPOINT);

    return ( events ^ SAMPLEWARNINGDEVICE_IDENTIFY_TIMEOUT_EVT );
  }


  if ( events & SAMPLEWARNINGDEVICE_MAIN_SCREEN_EVT )
  {
    giWarningDeviceScreenMode = WARNINGDEVICE_MAINMODE;
    zclSampleWarningDevice_LcdDisplayUpdate();

    return ( events ^ SAMPLEWARNINGDEVICE_MAIN_SCREEN_EVT );
  }

  if ( events & SAMPLEWARNINGDEVICE_CLEAR_SQUAWK_NOTIFICATION_EVT )
  {
    HalLcdWriteString ("", HAL_LCD_LINE_2);
    HalLcdWriteString ("", HAL_LCD_LINE_3);

    return ( events ^ SAMPLEWARNINGDEVICE_CLEAR_SQUAWK_NOTIFICATION_EVT );
  }

#if ZG_BUILD_ENDDEVICE_TYPE    
  if ( events & SAMPLEWARNINGDEVICE_END_DEVICE_REJOIN_EVT )
  {
    bdb_ZedAttemptRecoverNwk();
    return ( events ^ SAMPLEWARNINGDEVICE_END_DEVICE_REJOIN_EVT );
  }
#endif

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      zclSampleWarningDevice_HandleKeys
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
static void zclSampleWarningDevice_HandleKeys( byte shift, byte keys )
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
	  
  }
  if ( keys & HAL_KEY_SW_4 )
  {
	  
  }
  if ( keys & HAL_KEY_SW_5 )
  {
    // leave (if applicable) + factory new reset
	  bdb_resetLocalAction();
  }
}

/*********************************************************************
 * @fn      zclSampleWarningDevice_LcdDisplayUpdate
 *
 * @brief   Called to update the LCD display.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleWarningDevice_LcdDisplayUpdate( void )
{
  zclSampleWarningDevice_LcdDisplayMainMode();
}

/*********************************************************************
 * @fn      zclSampleWarningDevice_LcdDisplayMainMode
 *
 * @brief   Called to display the main screen on the LCD.
 *
 * @param   none
 *
 * @return  none
 */
void zclSampleWarningDevice_LcdDisplayMainMode( void )
{
  if ( zclSampleWarningDevice_NwkState == DEV_ZB_COORD )
  {
    zclHA_LcdStatusLine1( 0 );
  }
  else if ( zclSampleWarningDevice_NwkState == DEV_ROUTER )
  {
    zclHA_LcdStatusLine1( 1 );
  }
  else if ( zclSampleWarningDevice_NwkState == DEV_END_DEVICE )
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
static void zclSampleWarningDevice_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg)
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
        osal_start_timerEx(zclSampleWarningDevice_TaskID, SAMPLEWARNINGDEVICE_END_DEVICE_REJOIN_EVT, SAMPLEWARNINGDEVICE_END_DEVICE_REJOIN_DELAY);
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
void zclSampleWarningDevice_CBKE(void)
{
  bdb_CBKETCLinkKeyExchangeAttempt(FALSE);
}
#endif

/*********************************************************************
 * @fn      zclSampleWarningDevice_ProcessIdentifyTimeChange
 *
 * @brief   Called to process any change to the IdentifyTime attribute.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleWarningDevice_ProcessIdentifyTimeChange( uint8 endpoint )
{ 
  
  if ( zclSampleWarningDevice_IdentifyTime > 0 )
  {
    osal_start_timerEx( zclSampleWarningDevice_TaskID, SAMPLEWARNINGDEVICE_IDENTIFY_TIMEOUT_EVT, 1000 );
    HalLedBlink ( HAL_LED_4, 0xFF, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
  }
  else
  {
    if ( zclSampleWarningDevice_ZoneStatus )
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_ON );
    else
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
    osal_stop_timerEx( zclSampleWarningDevice_TaskID, SAMPLEWARNINGDEVICE_IDENTIFY_TIMEOUT_EVT );
  }
}

/*********************************************************************
 * @fn      zclSampleWarningDevice_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSampleWarningDevice_BasicResetCB( void )
{

}

/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/

/*********************************************************************
 * @fn      zclSampleWarningDevice_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  none
 */
static void zclSampleWarningDevice_ProcessIncomingMsg( zclIncomingMsg_t *pInMsg)
{
  switch ( pInMsg->zclHdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSampleWarningDevice_ProcessInReadRspCmd( pInMsg );
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSampleWarningDevice_ProcessInWriteRspCmd( pInMsg );
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
      zclSampleWarningDevice_ProcessInDefaultRspCmd( pInMsg );
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSampleWarningDevice_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSampleWarningDevice_ProcessInDiscCmdsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSampleWarningDevice_ProcessInDiscAttrsRspCmd( pInMsg );
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSampleWarningDevice_ProcessInDiscAttrsExtRspCmd( pInMsg );
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
 * @fn      zclSampleWarningDevice_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleWarningDevice_ProcessInReadRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleWarningDevice_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleWarningDevice_ProcessInWriteRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleWarningDevice_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleWarningDevice_ProcessInDefaultRspCmd( zclIncomingMsg_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSampleWarningDevice_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleWarningDevice_ProcessInDiscCmdsRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleWarningDevice_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleWarningDevice_ProcessInDiscAttrsRspCmd( zclIncomingMsg_t *pInMsg )
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
 * @fn      zclSampleWarningDevice_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8 zclSampleWarningDevice_ProcessInDiscAttrsExtRspCmd( zclIncomingMsg_t *pInMsg )
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

#ifdef ZCL_WD
/*********************************************************************
* @fn      zclSampleWarningDevice_StartWarningCB
*
* @brief   Process in the received StartWarning Command.
*
* @param   pCmd - pointer to the incoming command
*
*/
static ZStatus_t zclSampleWarningDevice_StartWarningCB(zclWDStartWarning_t *pCmd)
{
  
  //terminate the effect of previous Squawk command if still current
  HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
  
  //Only blinks if the alarm is set
  if(((zclWDStartWarning_t *)pCmd->warningmessage.warningbits.warnMode != \
    SS_IAS_START_WARNING_WARNING_MODE_STOP)
     ||((zclWDStartWarning_t *)pCmd->warningmessage.warningbits.warnStrobe != \
       SS_IAS_START_WARNING_STROBE_NO_STROBE_WARNING))
  {
    //Blink both led1 and led2 to signal an alarm
    HalLedBlink ( (HAL_LED_1 | HAL_LED_2), pCmd->warningDuration, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME ); 

    HalLcdWriteString ("Alarm in", HAL_LCD_LINE_2);
    HalLcdWriteString ("Progress", HAL_LCD_LINE_3);
  }
  else //if the alarm is not present
  {
    //turn off the blinking when alarm not present
    HalLedSet ( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

    HalLcdWriteString ("No Warnings", HAL_LCD_LINE_2);
    HalLcdWriteString ("", HAL_LCD_LINE_3);
  }

   return (ZStatus_t)(0)  ;
  
}


ZStatus_t zclSampleWarningDevice_SquawkCB(zclWDSquawk_t* squawk)
{
  zclWDSquawk_t squawks = *squawk;
  //Squawk is only valid if there is not alarm in progress
  if(zclSampleWarningDevice_ZoneStatus != SS_IAS_ZONE_STATUS_ALARM1_ALARMED)
  {
    if((squawks.squawkbits.squawkMode == SS_IAS_SQUAWK_SQUAWK_MODE_SYSTEM_ALARMED_NOTIFICATION_SOUND)
       || (squawks.squawkbits.strobe == SS_IAS_SQUAWK_STROBE_USE_STROBE_BLINK_IN_PARALLEL_TO_SQUAWK))
    {
      HalLedBlink ( (HAL_LED_4), SQUAWKBLINKS, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME );
      
      HalLcdWriteString ("Squawk", HAL_LCD_LINE_2);
      HalLcdWriteString ("Received", HAL_LCD_LINE_3);

      osal_start_timerEx(zclSampleWarningDevice_TaskID, SAMPLEWARNINGDEVICE_CLEAR_SQUAWK_NOTIFICATION_EVT, 2000);

    }
  }
   return (ZStatus_t)(0)  ;
}
#endif

/****************************************************************************
****************************************************************************/


