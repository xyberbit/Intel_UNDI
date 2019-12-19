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


#include "I40e.h"
#include "i40e_diag.h"
#include "DriverDiagnostics.h"

// tentative definition needed by RunDiagnostics() function
EFI_DRIVER_DIAGNOSTICS_PROTOCOL    gUndiDriverDiagnostics;

/* Global Variables */

UINT8  mPacket[MAX_ETHERNET_SIZE]; // our loopback test packet

/* Function definitions */

/** Build a packet to transmit in the phy loopback test.

  @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering on so that we can
                             get the MAC address

  @return      Sets the global array Packet[] with the packet to send out during PHY loopback.
**/
VOID
_BuildPacket (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  ETHERNET_HDR  *EthernetHdr;
  UINT16        Length;
  UINT16        i;

  EthernetHdr = NULL;
  Length      = 0;
  i           = 0;

  ZeroMem ((CHAR8 *) mPacket, MAX_ETHERNET_SIZE);

  // First copy the source and destination addresses
  EthernetHdr = (ETHERNET_HDR *) mPacket;
  CopyMem (
    (CHAR8 *) &EthernetHdr->SourceAddr,
    (CHAR8 *) AdapterInfo->Hw.mac.perm_addr,
    ETH_ALEN
  );
  CopyMem (
    (CHAR8 *) &EthernetHdr->DestAddr,
    (CHAR8 *) AdapterInfo->BroadcastNodeAddress,
    ETH_ALEN
  );

  // Source address must be different than the station address
  // otherwise packet will be dropped
  EthernetHdr->SourceAddr[5] = 255 - EthernetHdr->SourceAddr[5];

  // Calculate the data segment size and store it in the header Big Endian style
  Length                  = TEST_PACKET_SIZE - sizeof (ETHERNET_HDR);
  EthernetHdr->Length[0]  = (UINT8) (Length >> 8);
  EthernetHdr->Length[1]  = (UINT8) Length;

  // Generate Packet data
  for (i = 0; i < Length; i++) {
    mPacket[i + sizeof (ETHERNET_HDR)] = (UINT8) i;
  }
}

#if (DBG_LVL & DIAG)
/** Helper debug function to display Rx descriptors, and respective packet and
   header addresses

  @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering

  @retval      None
**/
VOID
_DisplayBuffersAndDescriptors (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  union i40e_16byte_rx_desc  *ReceiveDesc;

  UINT32                    j;

  DEBUGDUMP (DIAG, ("Receive Descriptor\n"));
  DEBUGDUMP (DIAG, ("I40E_QRX_TAIL=%X ", rd32 (&AdapterInfo->Hw, I40E_QRX_TAIL (0))));
  DEBUGDUMP (DIAG, ("RxRing.NextToUse=%X\n", AdapterInfo->Vsi.RxRing.NextToUse));

  for (j = 0; j < AdapterInfo->Vsi.RxRing.Count; j++) {
    ReceiveDesc = I40E_RX_DESC (&AdapterInfo->Vsi.RxRing, j);
    DEBUGDUMP (DIAG, ("QWORD1=%p, ", ReceiveDesc));
    DEBUGDUMP (DIAG, ("QWORD1=%LX, ", ReceiveDesc->read.pkt_addr));
    DEBUGDUMP (DIAG, ("QWORD2=%LX\n", ReceiveDesc->read.hdr_addr));
  }
}
#endif /* (DBG_LVL & DIAG) */

/** Obtains allowed link speed on port as i40e_aq_link_speed enum value.

  @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             which the UNDI driver is layering

  @retval      I40E_LINK_SPEED_10GB     Allowed link speed is 10 GB
  @retval      I40E_LINK_SPEED_40GB     Allowed link speed is 40 GB
  @retval      I40E_LINK_SPEED_UNKNOWN  Link speed on port is not known,
                                        possible error
**/
enum i40e_aq_link_speed
_I40eGetMacLoopbackSpeed (
  IN I40E_DRIVER_DATA  *AdapterInfo
  )
{
  enum i40e_aq_link_speed  Speed;
  struct i40e_hw           *Hw;
  UINT8                    AllowedLinkSpeeds;
  EFI_STATUS               Status;

  Hw = &AdapterInfo->Hw;

  // Read current link speed. If no link select 25G for 25G-capable devices and
  // the smallest supported speed among 10G and 40G for other devices.
  Speed = i40e_get_link_speed (Hw);
  DEBUGPRINT (DIAG, ("i40e_get_link_speed = %x\n", Speed));

  if (Speed == I40E_LINK_SPEED_UNKNOWN) {

    // No link, see current phy speed capabilities
    Status = GetLinkSpeedCapability (AdapterInfo, &AllowedLinkSpeeds);
    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("GetLinkSpeedCapability returned %r\n", Status));
      return I40E_LINK_SPEED_UNKNOWN;
    }
    DEBUGPRINT (DIAG, ("GetLinkSpeedCapability = %x\n", AllowedLinkSpeeds));
    if ((AllowedLinkSpeeds & I40E_LINK_SPEED_25GB) != 0) {
      Speed = I40E_LINK_SPEED_25GB;
    } else if ((AllowedLinkSpeeds & I40E_LINK_SPEED_10GB) != 0) {
      Speed = I40E_LINK_SPEED_10GB;
    } else if ((AllowedLinkSpeeds & I40E_LINK_SPEED_40GB) != 0) {
      Speed = I40E_LINK_SPEED_40GB;
    } else if ((AllowedLinkSpeeds & I40E_LINK_SPEED_5GB) != 0) {
      Speed = I40E_LINK_SPEED_5GB;
    } else if ((AllowedLinkSpeeds & I40E_LINK_SPEED_2_5GB) != 0) {
      Speed = I40E_LINK_SPEED_2_5GB;
    } else {
      Speed = I40E_LINK_SPEED_UNKNOWN;
    }
  }
  return Speed;
}

/** Puts adapter into the loopback mode on given port.

   @param[in]    AdapterInfo   Pointer to the NIC data structure information
                               which the UNDI driver is layering

   @param[out]   PcsLinkCtrl   Pointer to variable that will save old link control
                               register (auto_neg, speed_sel settings etc.) value
                               between SetMacLoopback() and ClearMacLoopback() calls.

   @retval      EFI_SUCCESS          Loopback settings successfully applied
   @retval      EFI_DEVICE_ERROR     Unsupported link speed on adapter
   @retval      EFI_DEVICE_ERROR     Error reading or writing values from/to registers
**/
EFI_STATUS
I40eSetMacLoopbackWorkaround (
  IN  I40E_DRIVER_DATA        *AdapterInfo,
  OUT UINT32                  *PcsLinkCtrl,
  IN  enum i40e_aq_link_speed Speed
  )
{
  UINT64                      Reg;
  UINT64                      RegC;
  UINT64                      Reg8;
  struct i40e_hw              *Hw;
  struct i40e_asq_cmd_details CmdDetails;
  UINT8                       Port;
  UINT8                       WaIterNum;
  enum i40e_status_code       Status;

  Hw = &AdapterInfo->Hw;

  // Disable Link Management
  Status = i40e_aq_set_phy_debug(Hw, I40E_AQ_PHY_DEBUG_DISABLE_LINK_FW, NULL);
  RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

  if (Speed == I40E_LINK_SPEED_1GB
    || Speed == I40E_LINK_SPEED_10GB
    || Speed == I40E_LINK_SPEED_5GB
    || Speed == I40E_LINK_SPEED_2_5GB)
  {

    Status = i40e_aq_debug_read_register (Hw, 0x001E2000 + (4 * Hw->port), &Reg, NULL); // I40E_PRTMAC_HLCTL
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

    Reg |= (0x1ULL << 15); // I40E_PRTMAC_HLCTL_LPBK_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x001E2000 + (4 * Hw->port), Reg, NULL);
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

    Status = i40e_aq_debug_read_register (Hw, 0x0008C260 + (4 * Hw->port), &Reg, NULL); // I40E_PRTMAC_PCS_LINK_CTRL
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

    *PcsLinkCtrl = Reg;

    Reg |=  (0x1ULL << 30); // I40E_PRTMAC_PCS_LINK_CTRL_FORCE_LINK_UP_MASK
    Reg &= ~(0x1ULL << 29); // I40E_PRTMAC_PCS_LINK_CTRL_AUTO_NEG_ENABLE_MASK
    Reg &= ~(0x7ULL << 8);  // I40E_PRTMAC_PCS_LINK_CTRL_SPEED_SELECTION_MASK
    Reg |= (0x4ULL << 8);   // I40E_PRTMAC_PCS_LINK_CTRL_SPEED_SELECTION_SHIFT
    Reg &= ~(0x1ULL << 23); // I40E_PRTMAC_PCS_LINK_CTRL_POWER_DOWN_MASK
    Reg |= (0x1ULL << 31);  // I40E_PRTMAC_PCS_LINK_CTRL_RESTART_AUTO_NEG_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x0008C260 + (4 * Hw->port), Reg, NULL);
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

  } else if (Speed == I40E_LINK_SPEED_25GB || Speed == I40E_LINK_SPEED_40GB) {
    Status = i40e_aq_debug_read_register (Hw, 0x001E3050 + (4 * Hw->port), &Reg, NULL); // I40E_PRTMAC_HSEC_CTL_TX_TO_RX_LOOPBACK
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

    Reg |= (0x1ULL << 0); // I40E_PRTMAC_HSEC_CTL_TX_TO_RX_LOOPBACK_HSEC_CTL_TX_TO_RX_LOOPBACK_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x001E3050 + (4 * Hw->port), Reg, NULL);
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

    Status = i40e_aq_debug_read_register (Hw, 0x0008C260 + (4 * Hw->port), &Reg, NULL); // I40E_PRTMAC_PCS_LINK_CTRL
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

    *PcsLinkCtrl = Reg;

    // Configure Loopback
    Reg &= ~(0x1ULL << 29); // I40E_PRTMAC_PCS_LINK_CTRL_AUTO_NEG_ENABLE_MASK
    Reg |= (0x1ULL << 30);  // I40E_PRTMAC_PCS_LINK_CTRL_FORCE_LINK_UP_MASK
    Reg &= ~(0x7ULL << 8);  // I40E_PRTMAC_PCS_LINK_CTRL_SPEED_SELECTION_MASK
    Reg &= ~(0x3ULL << 0);  // I40E_PRTMAC_PCS_LINK_CTRL_PMD_40G_R_TYPE_SELECTION_MASK
    Reg |= (0x2ULL << 0);   // I40E_PRTMAC_PCS_LINK_CTRL_PMD_40G_R_TYPE_SELECTION_SHIFT
    Reg &= ~(0x1ULL << 16); // I40E_PRTMAC_PCS_LINK_CTRL_KX_ABILITY_MASK
    Reg &= ~(0x1ULL << 17); // I40E_PRTMAC_PCS_LINK_CTRL_KX4_ABILITY_MASK
    Reg &= ~(0x1ULL << 18); // I40E_PRTMAC_PCS_LINK_CTRL_KR_ABILITY_MASK
    Reg |= (0x1ULL << 19);  // I40E_PRTMAC_PCS_LINK_CTRL_KR4_ABILITY_MASK
    Reg &= ~(0x1ULL << 20); // I40E_PRTMAC_PCS_LINK_CTRL_CR4_ABILITY_MASK
    Reg &= ~(0x1ULL << 21); // I40E_PRTMAC_PCS_LINK_CTRL_KR2_ABILITY_MASK
    Reg |= (0x5ULL << 8);   // I40E_PRTMAC_PCS_LINK_CTRL_SPEED_SELECTION_SHIFT
    Reg &= ~(0x1ULL << 23); // I40E_PRTMAC_PCS_LINK_CTRL_POWER_DOWN_MASK
    Reg |= (0x1ULL << 31);  // I40E_PRTMAC_PCS_LINK_CTRL_RESTART_AUTO_NEG_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x0008C260 + (4 * Hw->port), Reg, NULL);
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

  } else {
    DEBUGPRINT (CRITICAL, ("Unsupported link speed %d\n", Speed));
    return EFI_DEVICE_ERROR;
  }

  ZeroMem (&CmdDetails, sizeof (CmdDetails));
  CmdDetails.async = TRUE;
  CmdDetails.postpone = TRUE;

  WaIterNum = 0;

  Status = i40e_aq_debug_read_register (Hw, 0x000B8000 + (4 * Hw->pf_id), &Reg, NULL); // I40E_PFGEN_PORTNUM_CAR
  RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);

  Port = Reg & (0x3 << 0); // I40E_PFGEN_PORTNUM_CAR_PORT_NUM_MASK
  switch (Port) {
  case 0:
    Status = i40e_aq_debug_write_register (Hw, 0x8E040, 0x7, NULL);
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
    break;
  case 1:
    Status = i40e_aq_debug_write_register (Hw, 0x8E044, 0x7, NULL);
    RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
    break;
  case 2:
    do {
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &Reg, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Reg |= 0x06;
      Status = i40e_aq_debug_write_register (Hw, 0xA4038, 0xF0009038, &CmdDetails);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Status = i40e_aq_debug_write_register (Hw, 0xA403C, Reg, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &RegC, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Status = i40e_aq_debug_read_register (Hw, 0xA4038, &Reg8, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      WaIterNum++;
    } while ((!((RegC & 0x06) && (Reg8 & 0x9038)))
      && (WaIterNum <= 5));
    break;
  case 3:
    do {
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &Reg, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Reg |= 0x30;
      Status = i40e_aq_debug_write_register (Hw, 0xA4038, 0xF0009038, &CmdDetails);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Status = i40e_aq_debug_write_register (Hw, 0xA403C, Reg, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &RegC, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      Status = i40e_aq_debug_read_register (Hw, 0xA4038, &Reg8, NULL);
      RETURN_DEVICE_ERROR_ON_ERROR_STATUS (Status);
      WaIterNum++;
    } while ((!((RegC & 0x30) && (Reg8 & 0x9038)))
      && (WaIterNum <= 5));
    break;
  default:
    break;
  }

  return EFI_SUCCESS;
}

/** Clears loopback mode settings on given port.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              which the UNDI driver is layering

   @param[in]   PcsLinkCtrl   Variable where old link control
                              register (auto_neg, speed_sel settings etc.) value
                              was saved on SetMacLoopback() call.

   @retval      EFI_SUCCESS          Loopback settings successfully cleared
   @retval      EFI_DEVICE_ERROR     Unsupported link speed on adapter
   @retval      EFI_DEVICE_ERROR     Error reading or writing values from/to registers
**/
EFI_STATUS
I40eClearMacLoopbackWorkaround (
  IN  I40E_DRIVER_DATA        *AdapterInfo,
  IN  UINT32                  PcsLinkCtrl,
  IN  enum i40e_aq_link_speed Speed
  )
{
  UINT64                      Reg;
  UINT64                      Reg8;
  UINT64                      RegC;
  struct i40e_hw              *Hw;
  struct i40e_asq_cmd_details CmdDetails;
  UINT8                       Port;
  UINT8                       WaIterNum;
  enum i40e_status_code       Status;
  enum i40e_status_code       AqStatus;

  Hw = &AdapterInfo->Hw;

  if (Speed == I40E_LINK_SPEED_1GB
    || Speed == I40E_LINK_SPEED_10GB
    || Speed == I40E_LINK_SPEED_5GB
    || Speed == I40E_LINK_SPEED_2_5GB)
  {

    Status = i40e_aq_debug_read_register (Hw, 0x001E2000 + (4 * Hw->port), &Reg, NULL); // I40E_PRTMAC_HLCTL
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
    Reg &= ~(0x1ULL << 15); // I40E_PRTMAC_HLCTL_LPBK_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x001E2000 + (4 * Hw->port), Reg, NULL);
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);

    Reg = PcsLinkCtrl;
    Reg &= ~(0x1ULL << 30); // I40E_PRTMAC_PCS_LINK_CTRL_FORCE_LINK_UP_MASK
    Reg |= (0x1ULL << 31);  // I40E_PRTMAC_PCS_LINK_CTRL_RESTART_AUTO_NEG_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x0008C260 + (4 * Hw->port), Reg, NULL); // I40E_PRTMAC_PCS_LINK_CTRL
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);

  } else if (Speed == I40E_LINK_SPEED_25GB || Speed == I40E_LINK_SPEED_40GB) {
    Status = i40e_aq_debug_read_register (
               Hw,
               0x001E3050 + (4 * Hw->port), // I40E_PRTMAC_HSEC_CTL_TX_TO_RX_LOOPBACK
               &Reg,
               NULL
             );
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
    Reg &= ~(0x1ULL << 0); // I40E_PRTMAC_HSEC_CTL_TX_TO_RX_LOOPBACK_HSEC_CTL_TX_TO_RX_LOOPBACK_MASK
    Status = i40e_aq_debug_write_register (Hw, 0x001E3050 + (4 * Hw->port), Reg, NULL);
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);

    Reg = PcsLinkCtrl;
    Reg &= ~(0x1ULL << 30); // I40E_PRTMAC_PCS_LINK_CTRL_FORCE_LINK_UP_MASK
    Reg &= ~(0x1ULL << 29); // I40E_PRTMAC_PCS_LINK_CTRL_AUTO_NEG_ENABLE_MASK
    Reg &= ~(0x7ULL << 8);  // I40E_PRTMAC_PCS_LINK_CTRL_SPEED_SELECTION_MASK
    Reg |= (0x1ULL << 31);  // I40E_PRTMAC_PCS_LINK_CTRL_RESTART_AUTO_NEG_MASK

    Status = i40e_aq_debug_write_register (Hw, 0x0008C260 + (4 * Hw->port), Reg, NULL); // I40E_PRTMAC_PCS_LINK_CTRL
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
  } else {
    DEBUGPRINT (CRITICAL, ("Unsupported link speed %d\n", Speed));
    return EFI_DEVICE_ERROR;
  }

  ZeroMem (&CmdDetails, sizeof (CmdDetails));
  CmdDetails.async = TRUE;
  CmdDetails.postpone = TRUE;

  WaIterNum = 0;

  Status = i40e_aq_debug_read_register (Hw, 0x000B8000 + (4 * Hw->pf_id), &Reg, NULL); // I40E_PFGEN_PORTNUM_CAR
  GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);

  Port = Reg & (0x3 << 0); // I40E_PFGEN_PORTNUM_CAR_PORT_NUM_MASK
  switch (Port) {
  case 0:
    Status = i40e_aq_debug_write_register (Hw, 0x8E040, 0x1, NULL);
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
    break;
  case 1:
    Status = i40e_aq_debug_write_register (Hw, 0x8E044, 0x1, NULL);
    GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
    break;
  case 2:
    do {
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &Reg, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Reg &= 0xFFFFFFF9;
      Status = i40e_aq_debug_write_register (Hw, 0xA4038, 0xF0009038, &CmdDetails);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Status = i40e_aq_debug_write_register (Hw, 0xA403C, Reg, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &RegC, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Status = i40e_aq_debug_read_register (Hw, 0xA4038, &Reg8, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      WaIterNum++;
    } while ((!(!(RegC & 0x06) && (Reg8 & 0x9038)))
      && (WaIterNum <= 5));
    break;
  case 3:
    do {
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &Reg, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Reg &= 0xFFFFFFCF;
      Status = i40e_aq_debug_write_register (Hw, 0xA4038, 0xF0009038, &CmdDetails);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Status = i40e_aq_debug_write_register (Hw, 0xA403C, Reg, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Status = i40e_aq_debug_read_register (Hw, 0xA403C, &RegC, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      Status = i40e_aq_debug_read_register (Hw, 0xA4038, &Reg8, NULL);
      GOTO_LABEL_ON_ERROR_STATUS (Status, ClearLoopBackError);
      WaIterNum++;
    } while ((!(!(RegC & 0x30) && (Reg8 & 0x9038)))
      && (WaIterNum <= 5));
    break;
  default:
    break;
  }

ClearLoopBackError:
  // Enable Link Management
  AqStatus = i40e_aq_set_phy_debug(Hw, 0, NULL);
  RETURN_DEVICE_ERROR_ON_ERROR_STATUS (AqStatus);

  return Status;
}

#ifdef X722_SUPPORT
/** Puts adapter into the switch loopback mode on given port.

   @param[in]    AdapterInfo   Pointer to the NIC data structure information
                               which the UNDI driver is layering

   @retval      EFI_SUCCESS          Loopback settings successfully applied
   @retval      EFI_DEVICE_ERROR     Unsupported link speed on adapter
   @retval      EFI_DEVICE_ERROR     Error reading or writing values from/to registers
**/
EFI_STATUS
I40eSetSwitchLoopback (
  IN  I40E_DRIVER_DATA  *AdapterInfo
  )
{
  struct i40e_hw              *Hw;
  enum i40e_status_code       I40eStatus;
  EFI_STATUS                  Status;
  struct i40e_vsi_context     VsiCtx;
  Hw = &AdapterInfo->Hw;
  ZeroMem (&VsiCtx, sizeof (VsiCtx));

  DEBUGPRINT(CRITICAL, ("I40eSetSwitchLoopback\n"));

  Status = I40eGetVsiParams (AdapterInfo, &VsiCtx);

  DEBUGPRINT(
    DIAG, ("i40e_aq_get_vsi_params VsiCtx.uplink_seid %d, AdapterInfo->MainVsiSeid %d %\n",
    VsiCtx.uplink_seid,
    AdapterInfo->MainVsiSeid)
  );

  if (EFI_ERROR(Status)) {
    DEBUGPRINT (CRITICAL, ("I40eGetVsiParams returned %r\n", Status));
    return EFI_DEVICE_ERROR;
  }
  I40eStatus = i40e_aq_add_veb(
                 Hw,
                 VsiCtx.uplink_seid,
                 AdapterInfo->MainVsiSeid,
                 1,
                 Hw->pf_id,
                 &AdapterInfo->VebSeid,
                 FALSE,
                 NULL
               );
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("i40e_aq_add_veb returned %d, aq_err %d\n",
      Status, AdapterInfo->Hw.aq.asq_last_status)
    );
    return EFI_DEVICE_ERROR;
  }

  VsiCtx.info.valid_sections = I40E_AQ_VSI_PROP_SECURITY_VALID;
  VsiCtx.info.sec_flags |= I40E_AQ_VSI_SEC_FLAG_ALLOW_DEST_OVRD;
  VsiCtx.info.valid_sections |= I40E_AQ_VSI_PROP_SWITCH_VALID;
  VsiCtx.info.switch_id |= I40E_AQ_VSI_SW_ID_FLAG_LOCAL_LB;
  VsiCtx.info.switch_id |= I40E_AQ_VSI_SW_ID_FLAG_ALLOW_LB;

  I40eStatus = i40e_aq_update_vsi_params(Hw, &VsiCtx, NULL);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("i40e_aq_update_vsi_params returned %d, aq_err %d\n",
      I40eStatus, AdapterInfo->Hw.aq.asq_last_status)
    );
    Status = EFI_DEVICE_ERROR;
  }

  return Status;
}

/** Puts adapter out from switch loopback mode on given port.

   @param[in]    AdapterInfo   Pointer to the NIC data structure information
                               which the UNDI driver is layering

   @retval      EFI_SUCCESS          Loopback settings successfully applied
   @retval      EFI_DEVICE_ERROR     Unsupported link speed on adapter
   @retval      EFI_DEVICE_ERROR     Error reading or writing values from/to registers
**/
EFI_STATUS
I40eClearSwitchLoopback (
  IN  I40E_DRIVER_DATA  *AdapterInfo
  )
{
  struct i40e_hw              *Hw;
  enum i40e_status_code       I40eStatus;
  EFI_STATUS                  Status;
  struct i40e_vsi_context     VsiCtx;
  Hw = &AdapterInfo->Hw;
  ZeroMem (&VsiCtx, sizeof (VsiCtx));

  DEBUGPRINT(CRITICAL, ("I40eClearSwitchLoopback\n"));

  Status = I40eGetVsiParams (AdapterInfo, &VsiCtx);

  DEBUGPRINT(
    DIAG, ("i40e_aq_get_vsi_params VsiCtx.uplink_seid %d, AdapterInfo->MainVsiSeid %d %\n",
    VsiCtx.uplink_seid, AdapterInfo->MainVsiSeid )
  );

  if (EFI_ERROR(Status)) {
    DEBUGPRINT (CRITICAL, ("I40eGetVsiParams returned %r\n", Status));
    return EFI_DEVICE_ERROR;
  }

  VsiCtx.info.valid_sections = I40E_AQ_VSI_PROP_SECURITY_VALID;
  VsiCtx.info.sec_flags &= ~I40E_AQ_VSI_SEC_FLAG_ALLOW_DEST_OVRD;
  VsiCtx.info.valid_sections |= I40E_AQ_VSI_PROP_SWITCH_VALID;
  VsiCtx.info.switch_id &= ~I40E_AQ_VSI_SW_ID_FLAG_LOCAL_LB;

  I40eStatus = i40e_aq_update_vsi_params(Hw, &VsiCtx, NULL);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("i40e_aq_update_vsi_params returned %d, aq_err %d\n",
      I40eStatus, AdapterInfo->Hw.aq.asq_last_status)
    );
    Status = EFI_DEVICE_ERROR;
  }

  if (AdapterInfo->VebSeid) {
    I40eStatus = i40e_aq_delete_element(Hw, AdapterInfo->VebSeid, NULL);
    if (I40eStatus != I40E_SUCCESS) {
      Status = EFI_DEVICE_ERROR;
    }
  }

  return Status;
}
#endif /* X722_SUPPORT */

/** Cleans up the receive queue if not empty.

   This routine repeatedly calls I40eReceive() until there is no data left in the queue.

   @param[in]   AdapterInfo      Pointer to the NIC data structure.

   @retval    EFI_SUCCESS            Receive queue is cleaned.
   @retval    EFI_OUT_OF_RESOURCES   No memory left to allocate Cpb receive buffer.
**/
EFI_STATUS
_CleanUpReceiveQueue (
  IN  I40E_DRIVER_DATA   *AdapterInfo
  )
{
  PXE_DB_RECEIVE  DbReceive;
  PXE_CPB_RECEIVE CpbReceive;
  UINTN           PxeStatCode;

  // Wait a little, then check to see if the packet has arrived
  CpbReceive.BufferAddr = (PXE_UINT64) (UINTN) AllocateZeroPool (I40E_RXBUFFER_2048);
  if (CpbReceive.BufferAddr == (PXE_UINT64) (UINTN) NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  CpbReceive.BufferLen = I40E_RXBUFFER_2048;

  do {
    PxeStatCode = I40eReceive (
                    AdapterInfo,
                    &CpbReceive,
                    &DbReceive
                  );
  } while (PxeStatCode != PXE_STATCODE_NO_DATA);

  FreePool ((VOID *) (UINTN) CpbReceive.BufferAddr);

  return EFI_SUCCESS;
}

/** Run the PHY loopback test for N iterations.

   This routine transmits a packet, waits a bit, and then checks to see if it was received.
   If any of the packets are not received then it will be interpreted as a failure.

   @param[in]   AdapterInfo      Pointer to the NIC data structure the PHY loopback test will be run on.
   @param[in]   PxeCpbTransmit   Pointer to the packet to transmit.


   @retval      EFI_SUCCESS        All packets were received successfully
   @retval      EFI_DEVICE_ERROR   Transmitting packet failed.
   @retval      EFI_DEVICE_ERROR   Receiving packet failed.
   @retval      EFI_DEVICE_ERROR   Transmitted and received packet data do not match.
**/
EFI_STATUS
I40eUndiRunPhyLoopback (
  IN I40E_DRIVER_DATA   *AdapterInfo,
  IN PXE_CPB_TRANSMIT   PxeCpbTransmit
  )
{
  EFI_STATUS      Status = EFI_SUCCESS;
  PXE_DB_RECEIVE  DbReceive;
  PXE_CPB_RECEIVE CpbReceive;
  UINTN           PxeStatCode;
  UINT32          j;
  UINT32          i;
  UINT64         *FreeTxBuffer = NULL;

  FreeTxBuffer = (UINT64 *) AllocateZeroPool (sizeof (*FreeTxBuffer) * AdapterInfo->TxRxDescriptorCount);
  if (FreeTxBuffer == NULL) {
    DEBUGPRINT (CRITICAL, ("AllocateZeroPool returned %X\n", Status));
    DEBUGWAIT (CRITICAL);
    return EFI_DEVICE_ERROR;
  }

  // Clean up Rx/Tx queues
  _CleanUpReceiveQueue (AdapterInfo);
  I40eFreeTxBuffers (
    AdapterInfo,
    AdapterInfo->TxRxDescriptorCount,
    FreeTxBuffer
  );

  CpbReceive.BufferAddr = (PXE_UINT64) (UINTN) AllocateZeroPool (I40E_RXBUFFER_2048);
  if (CpbReceive.BufferAddr == (PXE_UINT64) (UINTN) NULL) {
    DEBUGPRINT (CRITICAL, ("AllocateZeroPool returned %X\n", Status));
    DEBUGWAIT (CRITICAL);
    return EFI_DEVICE_ERROR;
  }

  j = 0;
  while (j < PHY_LOOPBACK_ITERATIONS) {
    ZeroMem ((VOID *) CpbReceive.BufferAddr, I40E_RXBUFFER_2048);

    PxeStatCode = I40eTransmit (
                    AdapterInfo,
                    (UINT64) (UINTN) &PxeCpbTransmit,
                    PXE_OPFLAGS_TRANSMIT_WHOLE | PXE_OPFLAGS_TRANSMIT_BLOCK
                  );

    if (PxeStatCode != PXE_STATCODE_SUCCESS) {
      DEBUGPRINT (CRITICAL, ("I40eTransmit returned error code %X\n", PxeStatCode));
      DEBUGWAIT (CRITICAL);
      Status = EFI_DEVICE_ERROR;
      break;
    }

    // Wait a little, then check to see if the packet has arrived

    DEBUGPRINT (DIAG, ("CpbReceive.BufferAddr allocated at %x\n", (UINTN) CpbReceive.BufferAddr));
    DEBUGWAIT (DIAG);
    CpbReceive.BufferLen = I40E_RXBUFFER_2048;

    for (i = 0; i <= 100000; i++) {
      Status = I40eReceive (
                 AdapterInfo,
                 &CpbReceive,
                 &DbReceive
               );
      gBS->Stall (10);
      if (Status == PXE_STATCODE_NO_DATA) {
        continue;
      } else if (Status != PXE_STATCODE_SUCCESS) {
        break;
      }

      // Packets from NCSI may be received even though internal PHY loopback
      // is set.
      // Test for packet we have just sent. If received something else, ignore
      // and continue polling for packets.
      if (CompareMem ((VOID *) (UINTN) CpbReceive.BufferAddr, (VOID *) (UINTN) mPacket, TEST_PACKET_SIZE) == 0) {

        // Coming out with PXE_STATCODE_SUCCESS
        break;
      }
    }

#if (DBG_LVL & DIAG)
    _DisplayBuffersAndDescriptors (AdapterInfo);
#endif /* (DBG_LVL & DIAG) */

    if (i > 100000) {
      DEBUGPRINT (CRITICAL, ("ERROR: Receive timeout on iteration %d\n", i));
      Status = EFI_DEVICE_ERROR;
      break;
    } else if (Status != PXE_STATCODE_SUCCESS) {
      DEBUGPRINT (CRITICAL, ("ERROR: Receive failed with status %X\n", Status));
      Status = EFI_DEVICE_ERROR;
      break;
    }

    I40eFreeTxBuffers (
      AdapterInfo,
      AdapterInfo->TxRxDescriptorCount,
      FreeTxBuffer
    );
    j++;

  }

  FreePool ((VOID *) ((UINTN) CpbReceive.BufferAddr));
  FreePool ((VOID *) FreeTxBuffer);
  return Status;
}

/** Sets up the adapter to run the Phy loopback test and then calls
   the loop which will iterate through the test.

   @param[in]   UndiPrivateData   Pointer to adapter data.

   @retval      EFI_SUCCESS             The Phy loopback test passed.
   @retval      EFI_DEVICE_ERROR        Phy loopback test failed
   @retval      EFI_INVALID_PARAMETER   Some other error occured.
**/
EFI_STATUS
I40eExecutePhyLoopbackDiagnostics (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  PXE_CPB_TRANSMIT        PxeCpbTransmit;
  EFI_STATUS              DiagnosticsStatus = EFI_DEVICE_ERROR;
  EFI_STATUS              Status;
  UINTN                   i;
  UINT32                  SaveReg = 0;
  enum i40e_aq_link_speed Speed;

  // Initialize and start the UNDI driver if it has not already been done
  if (I40eInitialize (&UndiPrivateData->NicInfo) != PXE_STATCODE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("I40eInitialize returned error\n"));
    return EFI_DEVICE_ERROR;
  }

  // No returns within this do-while loop because we need to take the nic out of
  // PHY loopback before exiting.
  do {

    UndiPrivateData->NicInfo.DriverBusy = TRUE;

    // The algorithm depends on the link speed so figure out current link speed.
    Speed = _I40eGetMacLoopbackSpeed (&UndiPrivateData->NicInfo);

    // Enable mac loopback mode
#ifdef X722_SUPPORT
    if (UndiPrivateData->NicInfo.Hw.mac.type == I40E_MAC_X722) {
      Status = I40eSetSwitchLoopback (&UndiPrivateData->NicInfo);
    } else {
      Status = I40eSetMacLoopbackWorkaround (
                 &UndiPrivateData->NicInfo,
                 &SaveReg,
                 Speed
               );
    }
#else /* NOT X722_SUPPORT */
    Status = I40eSetMacLoopbackWorkaround (
               &UndiPrivateData->NicInfo,
               &SaveReg,
               Speed
             );
#endif /* X722_SUPPORT */
    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("I40eSetMacLoopbackWorkaround returned %r\n", Status));
      break;
    }


    // Test uses broadcast packets
    i40e_aq_set_vsi_broadcast (
      &UndiPrivateData->NicInfo.Hw,
      UndiPrivateData->NicInfo.Vsi.Seid,
      TRUE,
      NULL
    );

    // Build our packet, and send it out the door.
    DEBUGPRINT (DIAG, ("Building Packet\n"));
    _BuildPacket (&UndiPrivateData->NicInfo);

    PxeCpbTransmit.MediaheaderLen = sizeof (ETHERNET_HDR);
    PxeCpbTransmit.DataLen        = TEST_PACKET_SIZE - sizeof (ETHERNET_HDR);
    PxeCpbTransmit.FrameAddr      = (UINTN) mPacket;
    PxeCpbTransmit.reserved       = 0;
    DEBUGPRINT (DIAG, ("Packet length = %d\n", PxeCpbTransmit.DataLen));
    DEBUGPRINT (DIAG, ("Packet = %X FrameAddr = %X\n", (UINTN) mPacket, PxeCpbTransmit.FrameAddr));
    DEBUGPRINT (DIAG, ("Packet data:\n"));
    for (i = 0; i < 40; i++) {
      DEBUGDUMP (DIAG, ("%d: %x ", i, ((UINT8 *) ((UINTN) PxeCpbTransmit.FrameAddr))[i]));
    }

    DEBUGWAIT (DIAG);

    DiagnosticsStatus = I40eUndiRunPhyLoopback (&UndiPrivateData->NicInfo, PxeCpbTransmit);
    DEBUGPRINT (DIAG, ("PHY Loopback test returns %r\n", DiagnosticsStatus));

  } while (0);

#ifdef X722_SUPPORT
  if (UndiPrivateData->NicInfo.Hw.mac.type == I40E_MAC_X722) {
    Status = I40eClearSwitchLoopback (&UndiPrivateData->NicInfo);
  } else {
    Status = I40eClearMacLoopbackWorkaround (&UndiPrivateData->NicInfo, SaveReg, Speed);
  }
#else /* NOT X722_SUPPORT */
  Status = I40eClearMacLoopbackWorkaround (&UndiPrivateData->NicInfo, SaveReg, Speed);
#endif /* X722_SUPPORT */

  UndiPrivateData->NicInfo.DriverBusy = FALSE;

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eClearMacLoopbackWorkaround returned %r\n", Status));
    return Status;
  }

  return DiagnosticsStatus;
}

/** Runs diagnostics on a controller.

    @param[in]   This               A pointer to the EFI_DRIVER_DIAGNOSTICS_PROTOCOL instance.
    @param[in]   ControllerHandle   The handle of the controller to run diagnostics on.
    @param[in]   ChildHandle        The handle of the child controller to run diagnostics on
                                    This is an optional parameter that may be NULL.  It will
                                    be NULL for device drivers.  It will also be NULL for a
                                    bus drivers that wish to run diagnostics on the bus
                                    controller.  It will not be NULL for a bus driver that
                                    wishes to run diagnostics on one of its child controllers.
    @param[in]   DiagnosticType     Indicates type of diagnostics to perform on the controller
                                    specified by ControllerHandle and ChildHandle.   See
                                    "Related Definitions" for the list of supported types.
    @param[in]   Language           A pointer to a three character ISO 639-2 language
                                    identifier.  This is the language in which the optional
                                    error message should be returned in Buffer, and it must
                                    match one of the languages specified in SupportedLanguages.
                                    The number of languages supported by a driver is up to
                                    the driver writer.
    @param[out]  ErrorType          A GUID that defines the format of the data returned in
                                    Buffer.
    @param[out]  BufferSize         The size, in bytes, of the data returned in Buffer.
    @param[out]  Buffer             A buffer that contains a Null-terminated Unicode string
                                    plus some additional data whose format is defined by
                                    ErrorType.  Buffer is allocated by this function with
                                    AllocatePool(), and it is the caller's responsibility
                                    to free it with a call to FreePool().

    @retval      EFI_SUCCESS             The controller specified by ControllerHandle and
                                         ChildHandle passed the diagnostic.
    @retval      EFI_INVALID_PARAMETER   ControllerHandle is not a valid EFI_HANDLE.
    @retval      EFI_INVALID_PARAMETER   ChildHandle is not NULL and it is not a valid
                                         EFI_HANDLE.
    @retval      EFI_INVALID_PARAMETER   Language is NULL.
    @retval      EFI_INVALID_PARAMETER   ErrorType is NULL.
    @retval      EFI_INVALID_PARAMETER   BufferType is NULL.
    @retval      EFI_INVALID_PARAMETER   Buffer is NULL.
    @retval      EFI_UNSUPPORTED         The driver specified by This does not support
                                         running diagnostics for the controller specified
                                         by ControllerHandle and ChildHandle.
    @retval      EFI_UNSUPPORTED         The driver specified by This does not support the
                                         type of diagnostic specified by DiagnosticType.
    @retval      EFI_UNSUPPORTED         The driver specified by This does not support the
                                         language specified by Language.
    @retval      EFI_OUT_OF_RESOURCES    There are not enough resources available to complete
                                         the diagnostics.
    @retval      EFI_OUT_OF_RESOURCES    There are not enough resources available to return
                                         the status information in ErrorType, BufferSize,
                                         and Buffer.
    @retval      EFI_DEVICE_ERROR        The controller specified by ControllerHandle and
                                         ChildHandle did not pass the diagnostic.
**/
EFI_STATUS
EFIAPI
I40eUndiDriverDiagnosticsRunDiagnostics (
  IN EFI_DRIVER_DIAGNOSTICS_PROTOCOL                         *This,
  IN EFI_HANDLE                                              ControllerHandle,
  IN EFI_HANDLE                                              ChildHandle OPTIONAL,
  IN EFI_DRIVER_DIAGNOSTIC_TYPE                              DiagnosticType,
  IN CHAR8                                                   *Language,
  OUT EFI_GUID                                               **ErrorType,
  OUT UINTN                                                  *BufferSize,
  OUT CHAR16                                                 **Buffer
  )
{
  EFI_DEVICE_PATH_PROTOCOL  *UndiDevicePath;
  UNDI_PRIVATE_DATA         *UndiPrivateData;
  EFI_NII_POINTER_PROTOCOL  *NiiPointerProtocol;
  EFI_STATUS                Status;
  CHAR8                     *SupportedLanguages;
  BOOLEAN                   Iso639Language;
  BOOLEAN                   Found;
  UINTN                     Index;

  Status = EFI_SUCCESS;
  UndiPrivateData = NULL;

  // Validate input parameters

  // Check against invalid NULL parameters
  if ((NULL == Language)
    || (NULL == ErrorType)
    || (NULL == BufferSize)
    || (NULL == Buffer)
    || (NULL == ControllerHandle))
  {
    return EFI_INVALID_PARAMETER;
  }

  // The following implementation is taken from UDK2014
  SupportedLanguages = This->SupportedLanguages;
  Iso639Language = (BOOLEAN) (This == &gUndiDriverDiagnostics);

  // Make sure Language is in the set of Supported Languages
  Found = FALSE;

  while (*SupportedLanguages != 0) {
    if (Iso639Language) {
      if (CompareMem (Language, SupportedLanguages, 3) == 0) {
        Found = TRUE;
        break;
      }
      SupportedLanguages += 3;
    } else {
      for (Index = 0; SupportedLanguages[Index] != 0
        && SupportedLanguages[Index] != ';'; Index++)
      {
        ;
      }
      if ((AsciiStrnCmp (SupportedLanguages, Language, Index) == 0)
        && (Language[Index] == 0))
      {
        Found = TRUE;
        break;
      }
      SupportedLanguages += Index;
      for (; *SupportedLanguages != 0
        && *SupportedLanguages == ';'; SupportedLanguages++)
      {
        ;
      }
    }
  }

  // If Language is not a member of SupportedLanguages, then return EFI_UNSUPPORTED
  if (!Found) {
    DEBUGPRINT (CRITICAL, ("Driver Diagnostics: Unsupported Language: %a\n", Language));
    return EFI_UNSUPPORTED;
  }

  // Make sure this driver is currently managing ControllerHandle
  // This satisfies the ControllerHandle validation requirement in scope of detection of invalid EFI handle
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiDevicePathProtocolGuid,
                  (VOID * *) &UndiDevicePath,
                  gUndiDriverBinding.DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_TEST_PROTOCOL
                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (DIAG, (" OpenProtocol Status = %8X\n", Status));
    return Status;
  }

  //  Open an instance for the gEfiPro1000Comp protocol so we can check
  //  if the child handle interface is actually supported and calculate the pointer to GigUndiPrivateData.
  DEBUGPRINT (DIAG, ("Open an instance for the gEfiPro1000Com Protocol\n"));
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiNiiPointerGuid,
                  (VOID * *) &NiiPointerProtocol,
                  gUndiDriverBinding.DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("OpenProtocol error Status %X\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  UndiPrivateData = UNDI_PRIVATE_DATA_FROM_THIS (NiiPointerProtocol->NiiProtocol31);

  // ChildHandle input parameter can be NULL. If it is not NULL we have to validate it.
  if (ChildHandle != NULL) {

    // Make sure this ChildHandle is a valid EFI handle with NII protocol support
    // This satisfies the ChildHandle validation requirement in scope of detecion of invalid EFI handle
    Status = gBS->OpenProtocol (
                    ChildHandle,
                    &gEfiNetworkInterfaceIdentifierProtocolGuid_31,
                    NULL,
                    gUndiDriverBinding.DriverBindingHandle,
                    ControllerHandle,
                    EFI_OPEN_PROTOCOL_TEST_PROTOCOL
                  );

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (DIAG, (" OpenProtocol returned %r\n", Status));
      return Status;
    }

    // Now we know the ChildHandle is a valid EFI handle.
    // Let's check if current ControllerHandle supports ChildHandle
    if (ChildHandle != UndiPrivateData->DeviceHandle) {
      DEBUGPRINT (CRITICAL, ("Driver Diagnostics: Unsupported Child handle: %x\n", ChildHandle));
      DEBUGPRINT (CRITICAL, ("UndiPrivateData->DeviceHandle: %x\n", UndiPrivateData->DeviceHandle));
      return EFI_UNSUPPORTED;
    }
  }

  if (!UndiPrivateData->NicInfo.FwSupported) {
    return EFI_UNSUPPORTED;
  }

  // Perform required type of diagnostics
  switch (DiagnosticType) {
  case EfiDriverDiagnosticTypeStandard:
    if (i40e_diag_eeprom_test (&UndiPrivateData->NicInfo.Hw) != I40E_SUCCESS) {
      Status = EFI_DEVICE_ERROR;
    }
    break;
  case EfiDriverDiagnosticTypeExtended:
    if ((UndiPrivateData->NicInfo.UndiEnabled)
      && (UndiPrivateData->IsChildInitialized)

      // Loppback mode diagnostics is not supported in MFP mode.
      && (!UndiPrivateData->NicInfo.Hw.func_caps.flex10_enable)
      && (!UndiPrivateData->NicInfo.Hw.func_caps.npar_enable))
    {
      Status = I40eExecutePhyLoopbackDiagnostics (UndiPrivateData);
    } else {
      Status = EFI_UNSUPPORTED;
    }
    break;
  case EfiDriverDiagnosticTypeManufacturing:
    DEBUGPRINT (CRITICAL, ("Driver Diagnostics: EfiDriverDiagnosticTypeManufacturing not supported\n"));
    DEBUGWAIT (CRITICAL);
    Status = EFI_UNSUPPORTED;
    break;
  default:
    DEBUGPRINT (DIAG, ("Unsupported diagnostic mode %x\n", DiagnosticType));
    DEBUGWAIT (DIAG);
    Status = EFI_UNSUPPORTED;
    break;
  }

  return Status;
}

// Diagnostics protocols structures definition and initialization

EFI_DRIVER_DIAGNOSTICS_PROTOCOL gUndiDriverDiagnostics = {
  I40eUndiDriverDiagnosticsRunDiagnostics,
  "eng"
};

EFI_DRIVER_DIAGNOSTICS2_PROTOCOL gUndiDriverDiagnostics2 = {
  (EFI_DRIVER_DIAGNOSTICS2_RUN_DIAGNOSTICS) I40eUndiDriverDiagnosticsRunDiagnostics,
  "en-US"
};

