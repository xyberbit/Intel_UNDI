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
#include "ixgbe_common.h"
#include "Xgbe.h"
#include "EepromConfig.h"
#include "wol.h"
#include "DeviceSupport.h"


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
  )
{
  UINTN Active        = LINK_SPEED_AUTO_NEG;

  //  Speed settings are currently not supported for 10 Gig driver. It's always set to autoneg to
  //  allow operation with the highest possible speed
  DEBUGPRINT (HII, ("EepromGetLanSpeedStatus\n"));
  return Active;
}

/** Sets LAN speed setting for port

   @param[in]   UndiPrivateData    Driver private data structure
   @param[in]   LanSpeed           Desired LAN speed

   @retval   EFI_SUCCESS    LAN speed set successfully
**/
EFI_STATUS
EepromSetLanSpeed (
  UNDI_PRIVATE_DATA *UndiPrivateData,
  UINT8              LanSpeed
  )
{

  return EFI_SUCCESS;
}

/** Reads the currently assigned MAC address and factory default MAC address.

   @param[in]    UndiPrivateData     Driver private data structure
   @param[in]    LanFunction         LAN function number
   @param[out]   DefaultMacAddress   Factory default MAC address of the adapter
   @param[out]   AssignedMacAddress  CLP Assigned MAC address of the adapter,
                                     or the factory MAC address if an alternate MAC
                                     address has not been assigned.

   @retval   EFI_SUCCESS   MAC addresses successfully read.
**/
EFI_STATUS
_EepromMacAddressGet (
  IN UNDI_PRIVATE_DATA *UndiPrivateData,
  IN  UINT32            LanFunction,
  OUT UINT16 *          DefaultMacAddress,
  OUT UINT16 *          AssignedMacAddress
  )
{
  UINT16 BackupMacOffset;
  UINT16 FactoryMacOffset;
  UINT16 BackupMacAddress[3];

  XGBE_DRIVER_DATA *XgbeAdapter;

  XgbeAdapter = &UndiPrivateData->NicInfo;

  // Read in the currently assigned address from the default MAC address location
  if (LanFunction == 0) {
    ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_CORE0_PTR, &FactoryMacOffset);
  } else {
    ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_CORE1_PTR, &FactoryMacOffset);
  }

  // The first word of the EEPROM CORE structure is the size field.  The MAC address
  // starts after the size field.  Adjust here:
  FactoryMacOffset += 1;
  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));

  ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset, &AssignedMacAddress[0]);
  ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 1, &AssignedMacAddress[1]);
  ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 2, &AssignedMacAddress[2]);

  // Check to see if the backup MAC address location is being used, otherwise the
  // factory MAC address location will be the default.
  ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_ALT_MAC_ADDR_PTR, &BackupMacOffset);

  if (BackupMacOffset == 0xFFFF
    || BackupMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset, &DefaultMacAddress[0]);
    ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 1, &DefaultMacAddress[1]);
    ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 2, &DefaultMacAddress[2]);
  } else {

    // Adjust the MAC address offset if this is the second port (function 1)
    BackupMacOffset = BackupMacOffset + (UINT16) (3 * XgbeAdapter->Function);
    DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", BackupMacOffset));

    // Check if MAC address is backed up
    ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset, &BackupMacAddress[0]);
    if (BackupMacAddress[0] == 0xFFFF) {

      // In this case the factory MAC address is not in the backup location, so the factory
      // default MAC address is the same as the address we read in from the EEPROM CORE 0/1
      // locations.
      DefaultMacAddress[0] = AssignedMacAddress[0];
      DefaultMacAddress[1] = AssignedMacAddress[1];
      DefaultMacAddress[2] = AssignedMacAddress[2];
    } else {

      // Read in the factory default Mac address.
      ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset, &DefaultMacAddress[0]);
      ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset + 1, &DefaultMacAddress[1]);
      ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset + 2, &DefaultMacAddress[2]);
    }
  }

  return EFI_SUCCESS;
}

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
  )
{
  return _EepromMacAddressGet (
           UndiPrivateData,
           UndiPrivateData->NicInfo.LanFunction,
           DefaultMacAddress,
           AssignedMacAddress
         );
}



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
  )
{
  UINT16 BackupMacOffset;
  UINT16 FactoryMacOffset;
  UINT16 BackupMacAddress[3];

  XGBE_DRIVER_DATA *XgbeAdapter;

  XgbeAdapter = &UndiPrivateData->NicInfo;

  // Read the address where the override MAC address is stored.
  ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_ALT_MAC_ADDR_PTR, &BackupMacOffset);

  if (BackupMacOffset == 0xFFFF
    || BackupMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    return EFI_UNSUPPORTED;
  }

  // Adjust the MAC address offset if this is the second port (function 1)
  BackupMacOffset = BackupMacOffset + (UINT16) (3 * XgbeAdapter->Function);
  DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", BackupMacOffset));

  if (XgbeAdapter->LanFunction == 0) {
    ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_CORE0_PTR, &FactoryMacOffset);
  } else {
    ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_CORE1_PTR, &FactoryMacOffset);
  }

  // The first word of the EEPROM CORE structure is the size field.  The MAC address
  // starts after the size field.  Adjust here:
  FactoryMacOffset += 1;
  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));


  if ((UndiPrivateData->NicInfo.Hw.mac.type != ixgbe_mac_X550)
    && (UndiPrivateData->NicInfo.Hw.mac.type != ixgbe_mac_X550EM_x)
    && (UndiPrivateData->NicInfo.Hw.mac.type != ixgbe_mac_X550EM_a)
    )
  {

  // Check if MAC address is backed up

  ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset, &BackupMacAddress[0]);
  if (BackupMacAddress[0] == 0xFFFF) {

    // Read in the factory MAC address
    ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset, &BackupMacAddress[0]);
    ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 1, &BackupMacAddress[1]);
    ixgbe_read_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 2, &BackupMacAddress[2]);

    // Now back it up
    ixgbe_write_eeprom (&XgbeAdapter->Hw, BackupMacOffset, BackupMacAddress[0]);
    ixgbe_write_eeprom (&XgbeAdapter->Hw, BackupMacOffset + 1, BackupMacAddress[1]);
    ixgbe_write_eeprom (&XgbeAdapter->Hw, BackupMacOffset + 2, BackupMacAddress[2]);
  }
  }

  // At this point the factory MAC address should be in the backup location.  Now
  // write the new CLP assigned MAC address into the original factory location.
  ixgbe_write_eeprom (&XgbeAdapter->Hw, FactoryMacOffset, NewMacAddress[0]);
  ixgbe_write_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 1, NewMacAddress[1]);
  ixgbe_write_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 2, NewMacAddress[2]);

  ixgbe_update_eeprom_checksum (&XgbeAdapter->Hw);

  return EFI_SUCCESS;
}

/** Restores the factory default MAC address.

   @param[in]   UndiPrivateData   Driver private data structure

   @retval   EFI_UNSUPPORTED   Invalid offset for alternate MAC address
   @retval   EFI_SUCCESS       Alternate MAC Address feature not enabled
**/
EFI_STATUS
EepromMacAddressDefault (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  UINT16            BackupMacOffset;
  UINT16            FactoryMacOffset;
  UINT16            BackupMacAddress[3];
  XGBE_DRIVER_DATA *XgbeAdapter;

  XgbeAdapter = &UndiPrivateData->NicInfo;

  // Read the address where the override MAC address is stored.
  ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_ALT_MAC_ADDR_PTR, &BackupMacOffset);

  if (BackupMacOffset == 0xFFFF
    || BackupMacOffset == 0x0000)
  {
    DEBUGPRINT (CLP, ("Alt MAC Address feature not enabled.\n"));
    return EFI_UNSUPPORTED;
  }

  // Adjust the MAC address offset if this is the second port (function 1)
  BackupMacOffset = BackupMacOffset + (UINT16) (3 * XgbeAdapter->Function);
  DEBUGPRINT (CLP, ("MAC addresses at offset %X\n", BackupMacOffset));

  // Check if MAC address is backed up
  ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset, &BackupMacAddress[0]);
  ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset + 1, &BackupMacAddress[1]);
  ixgbe_read_eeprom (&XgbeAdapter->Hw, BackupMacOffset + 2, &BackupMacAddress[2]);
  if (BackupMacAddress[0] == 0xFFFF) {
    DEBUGPRINT (CRITICAL, ("No backup MAC addresses\n"));
    return EFI_SUCCESS;
  }

  // Restore the factory MAC address
  if (XgbeAdapter->LanFunction == 0) {
    ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_CORE0_PTR, &FactoryMacOffset);
  } else {
    ixgbe_read_eeprom (&XgbeAdapter->Hw, IXGBE_CORE1_PTR, &FactoryMacOffset);
  }

  // The first word of the EEPROM CORE structure is the size field.  The MAC address
  // starts after the size field.  Adjust here:
  FactoryMacOffset += 1;
  DEBUGPRINT (CLP, ("Factory MAC address at offset %X\n", FactoryMacOffset));

  ixgbe_write_eeprom (&XgbeAdapter->Hw, FactoryMacOffset, BackupMacAddress[0]);
  ixgbe_write_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 1, BackupMacAddress[1]);
  ixgbe_write_eeprom (&XgbeAdapter->Hw, FactoryMacOffset + 2, BackupMacAddress[2]);

  ixgbe_update_eeprom_checksum (&XgbeAdapter->Hw);

  return EFI_SUCCESS;
}

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
  )
{
  UINT16     Word;
  EFI_STATUS Status;

  Status = ixgbe_read_eeprom (&UndiPrivateData->NicInfo.Hw, EEPROM_CAPABILITIES_WORD, &Word);
  Word &= ~EEPROM_CAPABILITIES_SIG;
  *CapabilitiesWord = Word;

  return Status;
}

/** Checks if it is LOM device

   @param[in]   UndiPrivateData   Points to the driver instance private data

   @retval   TRUE     It is LOM device
   @retval   FALSE    It is not LOM device
   @retval   FALSE    Failed to read NVM word
**/
BOOLEAN
EepromIsLomDevice (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  EFI_STATUS Status;
  UINT16     SetupWord;

  if (UndiPrivateData->NicInfo.Hw.mac.type == ixgbe_mac_82598EB) {
    Status = ixgbe_read_eeprom (&UndiPrivateData->NicInfo.Hw, EEPROM_COMPATIBILITY_WORD_OPLIN, &SetupWord);
    if (SetupWord == 0xffff) {
      SetupWord = 0;
    }
  } else {
    Status = ixgbe_read_eeprom (&UndiPrivateData->NicInfo.Hw, EEPROM_COMPATIBILITY_WORD, &SetupWord);
  }
  if (Status != IXGBE_SUCCESS) {
    return FALSE;
  }

  if ((SetupWord & EEPROM_COMPATABILITY_LOM_BIT) == EEPROM_COMPATABILITY_LOM_BIT) {
    return TRUE;
  }
  return FALSE;

}

/** Updates NVM checksum

   @param[in]   UndiPrivateData   Pointer to driver private data structure

   @retval      EFI_SUCCESS       Checksum successfully updated
   @retval      EFI_DEVICE_ERROR  Failed to update NVM checksum
**/
EFI_STATUS
EepromUpdateChecksum (
  IN  UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  if (ixgbe_update_eeprom_checksum (&UndiPrivateData->NicInfo.Hw) != IXGBE_SUCCESS) {
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

