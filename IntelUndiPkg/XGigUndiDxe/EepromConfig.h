/**************************************************************************

Copyright (c) 2016, Intel Corporation

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Intel Corporation nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/
#ifndef EEPROM_CONFIG_H_
#define EEPROM_CONFIG_H_

#include "Xgbe.h"

/* Compatibility Word in EEPROM
 The only thing we need this define for it to determine if the device is a LOM */
#define EEPROM_COMPATIBILITY_WORD          0x03
#define EEPROM_COMPATIBILITY_WORD_OPLIN    0x10
#define EEPROM_COMPATABILITY_LOM_BIT       0x0800

#define XGBE_EEPROM_CORE0                 0x9
#define XGBE_EEPROM_CORE1                 0xA

#define IOV_CONTROL_WORD_1_OFFSET                   0x0C
#define IOV_CONTROL_WORD_IOVENABLE_SHIFT            0
#define IOV_CONTROL_WORD_IOVENABLE_MASK             0x0001
#define IOV_CONTROL_WORD_MAXVFS_SHIFT               5
#define IOV_CONTROL_WORD_MAXVFS_MASK                0x07E0
#define IOV_CONTROL_WORD_MAXVFS_MAX                 63

// Sageville PCIe Capabilities Support
#define PCI_CAPSUP_L_OFFSET         0x000A
#define PCI_CAPSUP_H_OFFSET         0x000B
#define PCI_CAPSUP_IOVENABLE_SHIFT  0x0005
#define PCI_CAPSUP_IOVENABLE_MASK   0x01 << PCI_CAPSUP_IOVENABLE_SHIFT /* 0x0020 */
#define PCI_CNF2_L_OFFSET           0x0000
#define PCI_CNF2_H_OFFSET           0x0001
#define PCI_CNF2_NUM_VFS_SHIFT      0x0008
#define PCI_CNF2_NUM_VFS_MASK       0x7F << PCI_CNF2_NUM_VFS_SHIFT     /* 0x7F00 */
#define PCI_CNF2_NUM_VFS_MAX        64

// EEPROM power management bit definitions
#define XGBE_EEPROM_CONTROL_WORD3         0x38
#define XGBE_EEPROM_APM_ENABLE_PORT1      0x2
#define XGBE_EEPROM_APM_ENABLE_PORT0      0x1
#define XGBE_PCIE_CONFIG0_PTR             0x07
#define XGBE_PCIE_CONFIG1_PTR             0x08
#define XGBE_FLASH_DISABLE_BIT            0x0100 /* bit 8 */

typedef enum {
  LOCATION_DIRECT,  // direct offset
  LOCATION_POINTER, // pointer
} LOCATION_TYPE;

#define  NVM_DIRECT(Offset1)           {LOCATION_DIRECT, Offset1}
#define  NVM_POINTER(Offset1, Offset2) {LOCATION_POINTER, Offset1, Offset2}

typedef struct {
  LOCATION_TYPE Type;
  UINT32        Offset1;
  UINT32        Offset2;
} NVM_LOCATION;

#define MAX_EXCLUDED_FIELDS_RECORD_COUNT  40

typedef struct {
  NVM_LOCATION Location;          // location in NVM
  UINT32       Length;
  UINT32       ModuleStartOffset;
  UINT16       BitMask;           // bits mask - applies to all words pointed by Size
} EXCLUDED_FIELDS_RECORD;


/** Gets LAN speed setting for port

   @param[in]   UndiPrivateData    Pointer to adapter structure

   @retval   LINK_SPEED_AUTO_NEG   We do not support speed settings
   @retval   LINK_SPEED_AUTO_NEG   Default Auto-Negotiation settings
   @retval   LINK_SPEED_1000FULL   LAN speed 1 GBit Full duplex
   @retval   LINK_SPEED_100FULL    LAN speed 100 MBit Full duplex
**/
UINTN
EepromGetLanSpeedStatus (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  );

/** Sets LAN speed setting for port

   @param[in]   UndiPrivateData    Driver private data structure
   @param[in]   LanSpeed           Desired LAN speed

   @retval   EFI_SUCCESS    LAN speed set successfully
**/
EFI_STATUS
EepromSetLanSpeed (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  UINT8              LanSpeed
  );

/** Restores the factory default MAC address.

   @param[in]   UndiPrivateData   Driver private data structure

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Alternate MAC Address feature not enabled
**/
EFI_STATUS
EepromMacAddressDefault (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  );



/** Programs the port with an alternate MAC address, and (in 82580-like case)
   backs up the factory default MAC address.

   @param[in]   UndiPrivateData   Pointer to driver private data structure
   @param[in]   NewMacAddress        Value to set the MAC address to.

   @retval   EFI_UNSUPPORTED   Alternate MAC Address feature not enabled
   @retval   EFI_SUCCESS       Default MAC address set successfully
**/
EFI_STATUS
EepromMacAddressSet (
  IN UNDI_PRIVATE_DATA *UndiPrivateData,
  IN UINT16 *           NewMacAddress
  );

/** Reads the currently assigned MAC address and factory default MAC address.

   @param[in]    UndiPrivateData   Driver private data structure
   @param[out]   DefaultMacAddress   Factory default MAC address of the adapter
   @param[out]   AssignedMacAddress  CLP Assigned MAC address of the adapter,
                                     or the factory MAC address if an alternate MAC
                                     address has not been assigned.

   @retval   EFI_SUCCESS   MAC addresses successfully read.
**/
EFI_STATUS
EepromMacAddressGet (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData,
  OUT UINT16 *           DefaultMacAddress,
  OUT UINT16 *           AssignedMacAddress
  );


#define EEPROM_CAPABILITIES_WORD 0x33
#define EEPROM_CAPABILITIES_SIG  0x4000

/** Returns EEPROM capabilities word (0x33) for current adapter

   @param[in]    UndiPrivateData   Points to the driver instance private data
   @param[out]   CapabilitiesWord   EEPROM capabilities word (0x33) for current adapter

   @retval   EFI_SUCCESS   Function completed successfully,
   @retval   !EFI_SUCCESS  Failed to read EEPROM capabilities word
**/
EFI_STATUS
EepromGetCapabilitiesWord (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData,
  OUT UINT16 *           CapabilitiesWord
  );

/** Checks if it is LOM device

   @param[in]   UndiPrivateData   Points to the driver instance private data

   @retval   TRUE     It is LOM device
   @retval   FALSE    It is not LOM device
   @retval   FALSE    Failed to read NVM word
**/
BOOLEAN
EepromIsLomDevice (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  );

/** Updates NVM checksum

   @param[in]   UndiPrivateData   Pointer to driver private data structure

   @retval      EFI_SUCCESS       Checksum successfully updated
   @retval      EFI_DEVICE_ERROR  Failed to update NVM checksum
**/
EFI_STATUS
EepromUpdateChecksum (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  );


#endif /* EEPROM_CONFIG_H_ */

