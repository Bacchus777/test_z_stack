/******************************************************************************

@file zcl_samplefiredetector_data.c

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
 * INCLUDES
 */
#include "ZComDef.h"
#include "OSAL.h"
#include "AF.h"
#include "ZDConfig.h"

#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"
#include "zcl_ezmode.h"

#include "zcl_samplefiredetector.h"

/*********************************************************************
 * CONSTANTS
 */

#define SAMPLEFIREDETECTOR_DEVICE_VERSION     0
#define SAMPLEFIREDETECTOR_FLAGS              0

#define SAMPLEFIREDETECTOR_HWVERSION          1
#define SAMPLEFIREDETECTOR_ZCLVERSION         1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Basic Cluster
const uint8 zclSampleFireDetector_HWRevision = SAMPLEFIREDETECTOR_HWVERSION;
const uint8 zclSampleFireDetector_ZCLVersion = SAMPLEFIREDETECTOR_ZCLVERSION;
const uint8 zclSampleFireDetector_ManufacturerName[] = { 16, 'T','e','x','a','s','I','n','s','t','r','u','m','e','n','t','s' };
const uint8 zclSampleFireDetector_ModelId[] = { 16, 'T','I','0','0','0','1',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSampleFireDetector_DateCode[] = { 16, '2','0','0','6','0','8','3','1',' ',' ',' ',' ',' ',' ',' ',' ' };
const uint8 zclSampleFireDetector_PowerSource = POWER_SOURCE_MAINS_1_PHASE;

uint8 zclSampleFireDetector_LocationDescription[17] = { 16, ' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ' };
uint8 zclSampleFireDetector_PhysicalEnvironment = 0;
uint8 zclSampleFireDetector_DeviceEnable = DEVICE_ENABLED;

// Identify Cluster
uint16 zclSampleFireDetector_IdentifyTime = 0;

uint16 zclSampleFireDetector_MaxDuration = SAMPLEFIREDETECTOR_ALARM_MAX_DURATION;

// IAS ZONE Cluster
uint8 zclSampleFireDetector_ZoneState = SS_IAS_ZONE_STATE_NOT_ENROLLED;
uint16 zclSampleFireDetector_ZoneType = SS_IAS_ZONE_TYPE_FIRE_SENSOR;
uint16 zclSampleFireDetector_ZoneStatus = 0;
uint8 zclSampleFireDetector_IAS_CIE_Address[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };


// Fire Detector Sensor Cluster
int16 zclSampleFireDetector_MeasuredValue = 2200;  // 22.00C
const int16 zclSampleFireDetector_MinMeasuredValue = 1700;   // 17.00C
const uint16 zclSampleFireDetector_MaxMeasuredValue = 2700;  // 27.00C

/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */
CONST zclAttrRec_t zclSampleFireDetector_Attrs[SAMPLEFIREDETECTOR_MAX_ATTRIBUTES] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GEN_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSampleFireDetector_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleFireDetector_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleFireDetector_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_ID,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleFireDetector_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DATE_CODE,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSampleFireDetector_DateCode
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleFireDetector_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_LOCATION_DESC,
      ZCL_DATATYPE_CHAR_STR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSampleFireDetector_LocationDescription
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENV,
      ZCL_DATATYPE_UINT8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleFireDetector_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GEN_BASIC,
    { // Attribute record
      ATTRID_BASIC_DEVICE_ENABLED,
      ZCL_DATATYPE_BOOLEAN,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleFireDetector_DeviceEnable
    }
  },

  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GEN_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSampleFireDetector_IdentifyTime
    }
  },
  // *** Zone Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_SS_IAS_ZONE,
    { // Attribute record
      ATTRID_SS_IAS_ZONE_STATE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleFireDetector_ZoneState
    }
  },
  
   {
    ZCL_CLUSTER_ID_SS_IAS_ZONE,
    { // Attribute record
      ATTRID_SS_IAS_ZONE_TYPE,
      ZCL_DATATYPE_ENUM16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleFireDetector_ZoneType
    }
  },
  
  {
    ZCL_CLUSTER_ID_SS_IAS_ZONE,
    { // Attribute record
      ATTRID_SS_IAS_ZONE_STATUS,
      ZCL_DATATYPE_BITMAP16,
      ACCESS_CONTROL_READ,
      (void *)&zclSampleFireDetector_ZoneStatus
    }
  },
  
  {
    ZCL_CLUSTER_ID_SS_IAS_ZONE,
    { // Attribute record
      ATTRID_SS_IAS_CIE_ADDRESS,
      ZCL_DATATYPE_IEEE_ADDR,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)zclSampleFireDetector_IAS_CIE_Address
    }
  }
  
  
};

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#define ZCLSAMPLEFIREDETECTOR_MAX_INCLUSTERS       3
const cId_t zclSampleFireDetector_InClusterList[ZCLSAMPLEFIREDETECTOR_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_BASIC,
  ZCL_CLUSTER_ID_GEN_IDENTIFY,
  ZCL_CLUSTER_ID_SS_IAS_ZONE
};

#define ZCLSAMPLEFIREDETECTOR_MAX_OUTCLUSTERS       1
const cId_t zclSampleFireDetector_OutClusterList[ZCLSAMPLEFIREDETECTOR_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GEN_IDENTIFY
};

SimpleDescriptionFormat_t zclSampleFireDetector_SimpleDesc =
{
  SAMPLEFIREDETECTOR_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                     //  uint16 AppProfId[2];
  ZCL_HA_DEVICEID_IAS_WARNING_DEVICE,        //  uint16 AppDeviceId[2];
  SAMPLEFIREDETECTOR_DEVICE_VERSION,            //  int   AppDevVer:4;
  SAMPLEFIREDETECTOR_FLAGS,                     //  int   AppFlags:4;
  ZCLSAMPLEFIREDETECTOR_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSampleFireDetector_InClusterList, //  byte *pAppInClusterList;
  ZCLSAMPLEFIREDETECTOR_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSampleFireDetector_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/****************************************************************************
****************************************************************************/

