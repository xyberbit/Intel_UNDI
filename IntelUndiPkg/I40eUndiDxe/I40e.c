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
#include "EepromConfig.h"
#include "DeviceSupport.h"

/* Global variables for blocking IO*/
STATIC BOOLEAN  mInitializeLock = TRUE;
STATIC EFI_LOCK gLock;


/** Blocking function called to assure that we are not swapped out from
   the queue while moving TX ring tail pointer.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on
   @param[in]   Flag         Block flag

   @return   According to Flag setting (TRUE/FALSE) we're acquiring or releasing EFI lock
**/
VOID
I40eBlockIt (
  IN  I40E_DRIVER_DATA *AdapterInfo,
  IN  UINT32            Flag
  )
{
  if (AdapterInfo->Block != NULL) {
    (*AdapterInfo->Block) (AdapterInfo->UniqueId, Flag);
  } else {
    if (mInitializeLock) {
      EfiInitializeLock (&gLock, TPL_NOTIFY);
      mInitializeLock = FALSE;
    }

    if (Flag != 0) {
      EfiAcquireLock (&gLock);
    } else {
      EfiReleaseLock (&gLock);
    }
  }
}

/** This is the drivers copy function so it does not need to rely on the
  BootServices copy which goes away at runtime.

  This copy function allows 64-bit or 32-bit copies depending on platform
  architecture. On Itanium we must check that both addresses
  are naturally aligned before attempting a 64-bit copy.

  @param[in]  Dest    Destination memory pointer.
  @param[in]  Source  Source memory pointer.
  @param[in]  Count   Number of bytes to copy.

  @return     Count bytes from Source copied to Dest
**/
VOID
I40eMemCopy (
  IN  UINT8 *Dest,
  IN  UINT8 *Source,
  IN  UINT32 Count
  )
{
  UINT32 BytesToCopy;
  UINT32 IntsToCopy;
  UINTN *SourcePtr;
  UINTN *DestPtr;
  UINT8 *SourceBytePtr;
  UINT8 *DestBytePtr;

  IntsToCopy  = Count / sizeof (UINTN);
  BytesToCopy = Count % sizeof (UINTN);

  SourcePtr = (UINTN *) Source;
  DestPtr   = (UINTN *) Dest;

  while (IntsToCopy > 0) {
    *DestPtr = *SourcePtr;
    SourcePtr++;
    DestPtr++;
    IntsToCopy--;
  }

  // Copy the leftover bytes.
  SourceBytePtr = (UINT8 *) SourcePtr;
  DestBytePtr   = (UINT8 *) DestPtr;
  while (BytesToCopy > 0) {
    *DestBytePtr = *SourceBytePtr;
    SourceBytePtr++;
    DestBytePtr++;
    BytesToCopy--;
  }
}

/** Dumps LAN context and HMC related info

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on
   @param[in]   QueueNumber   Number of queue
   @param[in]   HmcType       Type of HMC (only LAN_TX and LAN_RX matters)

   @return   Information dumped to standard output
**/
EFI_STATUS
HmcDump (
  IN I40E_DRIVER_DATA           *AdapterInfo,
  IN UINT16                      QueueNumber,
  IN enum i40e_hmc_lan_rsrc_type HmcType
  )
{
  UINT32 ControlReg;
  UINT32 ByteLength = 0;
  UINT32 QueueType  = 0;
  UINT32 SubLine    = 0;

  switch (HmcType) {
  case I40E_HMC_LAN_RX:
    ByteLength = I40E_HMC_OBJ_SIZE_RXQ;
    QueueType  = LANCTXCTL_QUEUE_TYPE_RX;
    break;
  case I40E_HMC_LAN_TX:
    ByteLength = I40E_HMC_OBJ_SIZE_TXQ;
    QueueType  = LANCTXCTL_QUEUE_TYPE_TX;
    break;
  default:
    return EFI_DEVICE_ERROR;
    break;
  }


  for (SubLine = 0; SubLine < (ByteLength / SUB_LINE_LENGTH); SubLine++)
  {
    ControlReg = ((UINT32) QueueNumber << I40E_PFCM_LANCTXCTL_QUEUE_NUM_SHIFT) |
                 ((UINT32) QueueType << I40E_PFCM_LANCTXCTL_QUEUE_TYPE_SHIFT) |
                 ((UINT32) SubLine << I40E_PFCM_LANCTXCTL_SUB_LINE_SHIFT) |
                 ((UINT32) 0 << I40E_PFCM_LANCTXCTL_OP_CODE_SHIFT);

    I40eWrite32 (AdapterInfo, I40E_PFCM_LANCTXCTL, ControlReg);
    Print (L"I40E_PFCM_LANCTXCTL = %x\n", ControlReg);
    while ((I40eRead32 (
              AdapterInfo,
              I40E_PFCM_LANCTXSTAT
            ) & I40E_PFCM_LANCTXSTAT_CTX_DONE_MASK) == 0)
    {
      ;
    }
    Print (
      L"HMC function %d, Queue %d, Type %d, SubLine %x: %x %x %x %x\n",
      AdapterInfo->Function,
      QueueNumber,
      QueueType,
      SubLine,
      I40eRead32 (AdapterInfo, I40E_PFCM_LANCTXDATA (0)),
      I40eRead32 (AdapterInfo, I40E_PFCM_LANCTXDATA (1)),
      I40eRead32 (AdapterInfo, I40E_PFCM_LANCTXDATA (2)),
      I40eRead32 (AdapterInfo, I40E_PFCM_LANCTXDATA (3))
    );
  }


  return EFI_SUCCESS;
}

/** Copies the frame from one of the Rx buffers to the command block
  passed in as part of the cpb parameter.

  The flow:  Ack the interrupt, setup the pointers, find where the last
  block copied is, check to make sure we have actually received something,
  and if we have then we do a lot of work. The packet is checked for errors,
  adjust the amount to copy if the buffer is smaller than the packet,
  copy the packet to the EFI buffer, and then figure out if the packet was
  targetted at us, broadcast, multicast or if we are all promiscuous.
  We then put some of the more interesting information (protocol, src and dest
  from the packet) into the db that is passed to us.  Finally we clean up
  the frame, set the return value to _SUCCESS, and inc the index, watching
  for wrapping.  Then with all the loose ends nicely wrapped up,
  fade to black and return.

  @param[in]  AdapterInfo  Pointer to the NIC data structure information which
                           the UNDI driver is layering on
  @param[in]  CpbReceive  Pointer (Ia-64 friendly) to the command parameter block.
                          The frame will be placed inside of it.
  @param[in]  DbReceive   The data buffer.  The out of band method of passing
                          pre-digested information to the protocol.

  @retval     PXE_STATCODE_NO_DATA  There is no data to receive
  @retval     PXE_STATCODE_SUCCESS  Received data passed to the protocol.
**/
UINTN
I40eReceive (
  IN  I40E_DRIVER_DATA *AdapterInfo,
  PXE_CPB_RECEIVE      *CpbReceive,
  PXE_DB_RECEIVE       *DbReceive
  )
{
  PXE_FRAME_TYPE             PacketType;
  union i40e_16byte_rx_desc *ReceiveDescriptor;
  ETHER_HEADER              *EtherHeader;
  PXE_STATCODE               StatCode;
  UINT16                     i;
  UINT16                     TempLen;
  UINT8                     *PacketPtr;

  UINT32 RxStatus;
  UINT32 RxError;
  UINT16 RxPacketLength;
  UINT16 RxHeaderLength;
  UINT16 RxSph;
  UINT16 RxPType;

  UINT64 DescQWord;

  PacketType  = PXE_FRAME_TYPE_NONE;
  StatCode    = PXE_STATCODE_NO_DATA;
  i           = 0;


  // Get a pointer to the buffer that should have a rx in it, IF one is really there.
  ReceiveDescriptor = I40E_RX_DESC (&AdapterInfo->Vsi.RxRing, AdapterInfo->Vsi.RxRing.NextToUse);

  DescQWord = ReceiveDescriptor->wb.qword1.status_error_len;
  RxStatus = (UINT32) ((DescQWord & I40E_RXD_QW1_STATUS_MASK) >> I40E_RXD_QW1_STATUS_SHIFT);

  if ((RxStatus & (1 << I40E_RX_DESC_STATUS_DD_SHIFT)) != 0) {
    RxPacketLength = (UINT16) ((DescQWord & I40E_RXD_QW1_LENGTH_PBUF_MASK) >> I40E_RXD_QW1_LENGTH_PBUF_SHIFT);
    RxHeaderLength = (UINT16) ((DescQWord & I40E_RXD_QW1_LENGTH_HBUF_MASK) >> I40E_RXD_QW1_LENGTH_HBUF_SHIFT);
    RxSph          = (UINT16) ((DescQWord & I40E_RXD_QW1_LENGTH_SPH_MASK) >> I40E_RXD_QW1_LENGTH_SPH_SHIFT);
    RxError        = (UINT32) ((DescQWord & I40E_RXD_QW1_ERROR_MASK) >> I40E_RXD_QW1_ERROR_SHIFT);
    RxPType        = (UINT16) ((DescQWord & I40E_RXD_QW1_PTYPE_MASK) >> I40E_RXD_QW1_PTYPE_SHIFT);

    // Just to make sure we don't try to copy a zero length, only copy a positive sized packet.
    if ((RxPacketLength != 0)
      && (RxError == 0))
    {
      // If the buffer passed us is smaller than the packet, only copy the size of the buffer.
      TempLen = RxPacketLength;
      if (RxPacketLength > (INT16) CpbReceive->BufferLen)
        TempLen = (UINT16) CpbReceive->BufferLen;

      // Copy the packet from our list to the EFI buffer.
      I40eMemCopy (
        (UINT8 *) (UINTN) CpbReceive->BufferAddr,
        AdapterInfo->Vsi.RxRing.BufferAddresses[AdapterInfo->Vsi.RxRing.NextToUse],
        TempLen
      );

      PacketPtr = (UINT8 *) (UINTN) CpbReceive->BufferAddr;
      DEBUGDUMP (
        RX, ("%02x:%02x:%02x:%02x:%02x:%02x %02x:%02x:%02x:%02x:%02x:%02x %02x%02x %02x %02x\n",
        PacketPtr[0x0], PacketPtr[0x1], PacketPtr[0x2], PacketPtr[0x3], PacketPtr[0x4], PacketPtr[0x5],
        PacketPtr[0x6], PacketPtr[0x7], PacketPtr[0x8], PacketPtr[0x9], PacketPtr[0xA], PacketPtr[0xB],
        PacketPtr[0xC], PacketPtr[0xD], PacketPtr[0xE], PacketPtr[0xF])
      );

      // Fill the DB with needed information
      DbReceive->FrameLen = RxPacketLength;  // includes header
      DbReceive->MediaHeaderLen = PXE_MAC_HEADER_LEN_ETHER;

      EtherHeader = (ETHER_HEADER *) (UINTN) PacketPtr;

      // Figure out if the packet was meant for us, was a broadcast, multicast or we
      // recieved a frame in promiscuous mode.
      for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
        if (EtherHeader->DestAddr[i] != AdapterInfo->Hw.mac.perm_addr[i]) {
          break;
        }
      }

      // if we went the whole length of the header without breaking out then the packet is
      // directed at us.
      if (i >= PXE_HWADDR_LEN_ETHER) {
        PacketType = PXE_FRAME_TYPE_UNICAST;
      } else {

        // Compare it against our broadcast node address
        for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
          if (EtherHeader->DestAddr[i] != AdapterInfo->BroadcastNodeAddress[i]) {
            break;
          }
        }

        // If we went the whole length of the header without breaking out
        // then the packet is directed at us via broadcast
        if (i >= PXE_HWADDR_LEN_ETHER) {
          PacketType = PXE_FRAME_TYPE_BROADCAST;
        } else {

          // That leaves multicast or we must be in promiscuous mode. Check for the
          // Mcast bit in the address. Otherwise its a promiscuous receive.
          if ((EtherHeader->DestAddr[0] & 1) == 1) {
            PacketType = PXE_FRAME_TYPE_MULTICAST;
          } else {
            PacketType = PXE_FRAME_TYPE_PROMISCUOUS;
          }
        }
      }

      DEBUGPRINT (RX, ("Status %x, Length %d, PacketType = %d\n", RxStatus, RxPacketLength, PacketType));

      DbReceive->Type = PacketType;

      // Put the protocol (UDP, TCP/IP) in the data buffer.
      DbReceive->Protocol = EtherHeader->Type;

      for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
        DbReceive->SrcAddr[i]   = EtherHeader->SrcAddr[i];
        DbReceive->DestAddr[i]  = EtherHeader->DestAddr[i];
      }

      DEBUGDUMP (
        RX, ("RxRing.NextToUse: %x, BufAddr: %x\n",
        AdapterInfo->Vsi.RxRing.NextToUse,
        (UINT64) (AdapterInfo->Vsi.RxRing.BufferAddresses[AdapterInfo->Vsi.RxRing.NextToUse]))
      );

      StatCode = PXE_STATCODE_SUCCESS;
    } else {
      DEBUGPRINT (CRITICAL, ("ERROR: RxPacketLength: %x, RxError: %x \n", RxPacketLength, RxError));
    }

    // Clean up the packet and restore the buffer address
    ReceiveDescriptor->wb.qword1.status_error_len = 0;
    ReceiveDescriptor->read.pkt_addr = (UINT64)
                                       (AdapterInfo->Vsi.RxRing.BufferAddresses[AdapterInfo->Vsi.RxRing.NextToUse]);

    // Move the current cleaned buffer pointer, being careful to wrap it as needed.  Then update the hardware,
    // so it knows that an additional buffer can be used.
    I40eWrite32 (AdapterInfo, I40E_QRX_TAIL (0), AdapterInfo->Vsi.RxRing.NextToUse);

    AdapterInfo->Vsi.RxRing.NextToUse++;
    if (AdapterInfo->Vsi.RxRing.NextToUse == AdapterInfo->Vsi.RxRing.Count) {
      AdapterInfo->Vsi.RxRing.NextToUse = 0;
    }
  }
  return StatCode;
}

/** Takes a command block pointer (cpb) and sends the frame.

  Takes either one fragment or many and places them onto the wire.
  Cleanup of the send happens in the function UNDI_Status in Decode.c

  @param[in]  AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on
  @param[in]  Cpb           The command parameter block address.
                            64 bits since this is Itanium(tm) processor friendly
  @param[in]  OpFlags       The operation flags, tells if there is any special
                            sauce on this transmit

  @retval     PXE_STATCODE_SUCCESS        The frame goes out
  @retval     PXE_STATCODE_DEVICE_FAILURE The frame does not go out
  @retval     PXE_STATCODE_BUSY           Need to call again later
**/
UINTN
I40eTransmit (
  IN I40E_DRIVER_DATA *AdapterInfo,
  IN UINT64            Cpb,
  IN UINT16            OpFlags
  )
{
  PXE_CPB_TRANSMIT_FRAGMENTS  *TxFrags;
  PXE_CPB_TRANSMIT            *TxBuffer;
  I40E_RING                   *TxRing;
  EFI_STATUS                  Status;

  struct i40e_tx_desc *TransmitDescriptor;
  UINT16               Size;

  UINT32 TdCommand = 0;
  UINT32 TdOffset = 0;
  UINT32 TdTag = 0;

  UINT32 i;
  INT32  WaitMsec;

  TxRing = &AdapterInfo->Vsi.TxRing;

  // Transmit buffers must be freed by the upper layer before we can transmit any more.
  if (TxRing->TxBufferMappings[TxRing->NextToUse].PhysicalAddress != 0) {
    DEBUGPRINT (CRITICAL, ("TX buffers have all been used!\n"));
    DEBUGWAIT (CRITICAL);
    return PXE_STATCODE_QUEUE_FULL;
  }

  // Make some short cut pointers so we don't have to worry about typecasting later.
  // If the TX has fragments we will use the
  // tx_tpr_f pointer, otherwise the tx_ptr_l (l is for linear)
  TxBuffer  = (PXE_CPB_TRANSMIT *) (UINTN) Cpb;
  TxFrags   = (PXE_CPB_TRANSMIT_FRAGMENTS *) (UINTN) Cpb;


  // quicker pointer to the next available Tx descriptor to use.
  TransmitDescriptor = I40E_TX_DESC (TxRing, TxRing->NextToUse);

  // Opflags will tell us if this Tx has fragments
  // So far the linear case (the no fragments case, the else on this if) is the majority
  // of all frames sent.
  if (OpFlags & PXE_OPFLAGS_TRANSMIT_FRAGMENTED) {

    // this count cannot be more than 8;
    DEBUGPRINT (TX, ("Fragments %x\n", TxFrags->FragCnt));

    // for each fragment, give it a descriptor, being sure to keep track of the number used.
    for (i = 0; i < TxFrags->FragCnt; i++) {

      // Put the size of the fragment in the descriptor
      TransmitDescriptor->buffer_addr = TxFrags->FragDesc[i].FragAddr;
      Size = (UINT16) TxFrags->FragDesc[i].FragLen;

      TransmitDescriptor->cmd_type_offset_bsz = I40E_TX_DESC_DTYPE_DATA
                                                | ((UINT64) TdCommand << I40E_TXD_QW1_CMD_SHIFT)
                                                | ((UINT64) TdOffset << I40E_TXD_QW1_OFFSET_SHIFT)
                                                | ((UINT64) Size << I40E_TXD_QW1_TX_BUF_SZ_SHIFT)
                                                | ((UINT64) TdTag << I40E_TXD_QW1_L2TAG1_SHIFT);

      TxRing->TxBufferMappings[TxRing->NextToUse].PhysicalAddress = TxFrags->FragDesc[i].FragAddr;

      // If this is the last fragment we must also set the EOP bit
      if ((i + 1) == TxFrags->FragCnt) {
        TransmitDescriptor->cmd_type_offset_bsz |= (UINT64) I40E_TXD_CMD << I40E_TXD_QW1_CMD_SHIFT;
      }

      // move our software counter passed the frame we just used, watching for wrapping
      DEBUGPRINT (TX, ("Advancing TX pointer %x\n", AdapterInfo->Vsi.TxRing.NextToUse));
      AdapterInfo->Vsi.TxRing.NextToUse++;
      if (AdapterInfo->Vsi.TxRing.NextToUse == AdapterInfo->Vsi.TxRing.Count) {
        AdapterInfo->Vsi.TxRing.NextToUse = 0;
      }

      TransmitDescriptor = I40E_TX_DESC (&AdapterInfo->Vsi.TxRing, AdapterInfo->Vsi.TxRing.NextToUse);
    }
  } else {
    Size = (UINT16) ((UINT16) TxBuffer->DataLen + TxBuffer->MediaheaderLen);

    TxRing->TxBufferMappings[TxRing->NextToUse].UnmappedAddress = TxBuffer->FrameAddr;
    TxRing->TxBufferMappings[TxRing->NextToUse].Size = Size;

    Status = UndiDmaMapMemoryRead (
               AdapterInfo->PciIo,
               &TxRing->TxBufferMappings[TxRing->NextToUse]
               );

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("Failed to map Tx buffer: %r\n", Status));
      DEBUGWAIT (CRITICAL);
      return PXE_STATCODE_DEVICE_FAILURE;
    }

    TransmitDescriptor->buffer_addr = TxRing->TxBufferMappings[TxRing->NextToUse].PhysicalAddress;

    TransmitDescriptor->cmd_type_offset_bsz = I40E_TX_DESC_DTYPE_DATA
                                              | ((UINT64) TdCommand << I40E_TXD_QW1_CMD_SHIFT)
                                              | ((UINT64) TdOffset << I40E_TXD_QW1_OFFSET_SHIFT)
                                              | ((UINT64) Size << I40E_TXD_QW1_TX_BUF_SZ_SHIFT)
                                              | ((UINT64) TdTag << I40E_TXD_QW1_L2TAG1_SHIFT);
    TransmitDescriptor->cmd_type_offset_bsz |= (UINT64) I40E_TXD_CMD << I40E_TXD_QW1_CMD_SHIFT;

    // Move our software counter passed the frame we just used, watching for wrapping
    TxRing->NextToUse++;
    if (TxRing->NextToUse == TxRing->Count) {
      TxRing->NextToUse = 0;
    }
    DEBUGDUMP (
      TX, ("Length = %d, Buffer addr %x, cmd_type_offset_bsz %x \n",
      Size,
      TransmitDescriptor->buffer_addr,
      TransmitDescriptor->cmd_type_offset_bsz)
    );

#if (DBG_LVL & TX)
    UINT8 * PacketPtr = (UINT8 *) (UINTN) TransmitDescriptor->buffer_addr;
    DEBUGDUMP (
      TX, ("%02x:%02x:%02x:%02x:%02x:%02x %02x:%02x:%02x:%02x:%02x:%02x %02x%02x %02x %02x\n",
      PacketPtr[0x0], PacketPtr[0x1], PacketPtr[0x2], PacketPtr[0x3], PacketPtr[0x4], PacketPtr[0x5],
      PacketPtr[0x6], PacketPtr[0x7], PacketPtr[0x8], PacketPtr[0x9], PacketPtr[0xA], PacketPtr[0xB],
      PacketPtr[0xC], PacketPtr[0xD], PacketPtr[0xE], PacketPtr[0xF])
    );
#endif /* (DBG_LVL & TX) */
  }

  // Turn on the blocking function so we don't get swapped out
  // Then move the Tail pointer so the HW knows to start processing the TX we just setup.
  I40eBlockIt (AdapterInfo, TRUE);
  I40eWrite32 (AdapterInfo, I40E_QTX_TAIL (0), TxRing->NextToUse);
  I40eBlockIt (AdapterInfo, FALSE);

  // If the OpFlags tells us to wait for the packet to hit the wire, we will wait.
  if ((OpFlags & PXE_OPFLAGS_TRANSMIT_BLOCK) != 0) {
    WaitMsec = 10000;

    while ((TransmitDescriptor->cmd_type_offset_bsz & I40E_TX_DESC_DTYPE_DESC_DONE) == 0) {
      DelayInMicroseconds (AdapterInfo, 10);
      WaitMsec -= 10;
      if (WaitMsec <= 0) {
        break;
      }
    }

    // If we waited for a while, and it didn't finish then the HW must be bad.
    if ((TransmitDescriptor->cmd_type_offset_bsz & I40E_TX_DESC_DTYPE_DESC_DONE) == 0) {
      DEBUGPRINT (CRITICAL, ("Device failure\n"));
      return PXE_STATCODE_DEVICE_FAILURE;
    } else {
      DEBUGPRINT (TX, ("Transmit success\n"));
    }
  }

  return PXE_STATCODE_SUCCESS;
}

/** Free TX buffers that have been transmitted by the hardware.

  @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                           the UNDI driver is layering on.
  @param[in]   NumEntries   Number of entries in the array which can be freed.
  @param[out]  TxBuffer     Array to pass back free TX buffer

  @return      Number of TX buffers written.
**/
UINT16
I40eFreeTxBuffers (
  I40E_DRIVER_DATA *AdapterInfo,
  IN UINT16         NumEntries,
  OUT UINT64       *TxBuffer
  )
{
  struct i40e_tx_desc   *TransmitDescriptor;
  I40E_RING             *TxRing;
  UINT16                i;
  EFI_STATUS            Status;

  i = 0;
  TxRing = &AdapterInfo->Vsi.TxRing;

  do {
    if (i >= NumEntries) {
      DEBUGPRINT (TX, ("Exceeded number of DB entries, i=%d, NumEntries=%d\n", i, NumEntries));
      break;
    }

    TransmitDescriptor = I40E_TX_DESC (TxRing, TxRing->NextToClean);

    DEBUGPRINT (
      TX, ("TXDesc:%d Addr:%x, ctob: %x\n",
      TxRing->NextToClean,
      TransmitDescriptor->buffer_addr,
      TransmitDescriptor->cmd_type_offset_bsz)
    );

    if ((TransmitDescriptor->cmd_type_offset_bsz & I40E_TX_DESC_DTYPE_DESC_DONE) != 0) {
      if (TxRing->TxBufferMappings[TxRing->NextToClean].PhysicalAddress == 0) {
        DEBUGPRINT (CRITICAL, ("ERROR: TX buffer complete without being marked used!\n"));
        break;
      }

      DEBUGPRINT (TX, ("Cleaning buffer address %d, %x\n", i, TxBuffer[i]));

      Status = UndiDmaUnmapMemory (
                 AdapterInfo->PciIo,
                 &TxRing->TxBufferMappings[TxRing->NextToClean]
                 );

      if (EFI_ERROR (Status)) {
        DEBUGPRINT (CRITICAL, ("Failed to unmap Tx buffer: %r\n", Status));
        DEBUGWAIT (CRITICAL);
        break;
      }

      TxBuffer[i] = TxRing->TxBufferMappings[TxRing->NextToClean].UnmappedAddress;
      i++;

      TxRing->TxBufferMappings[TxRing->NextToClean].UnmappedAddress = 0;
      TxRing->TxBufferMappings[TxRing->NextToClean].Size = 0;
      TransmitDescriptor->cmd_type_offset_bsz &= ~((UINT64)I40E_TXD_QW1_DTYPE_MASK);

      TxRing->NextToClean++;
      if (TxRing->NextToClean >= TxRing->Count) {
        TxRing->NextToClean = 0;
      }
    } else {
      DEBUGPRINT (TX, ("TX Descriptor %d not done\n", TxRing->NextToClean));
      break;
    }
  } while (TxRing->NextToUse != TxRing->NextToClean);

  return i;
}

/** Sets receive filters.

  @param[in]  AdapterInfo  Pointer to the adapter structure
  @param[in]  NewFilter    A PXE_OPFLAGS bit field indicating what filters to use.

  @return     Broad/Multicast and promiscous settings are set according to NewFilter
**/
VOID
I40eSetFilter (
  IN I40E_DRIVER_DATA  *AdapterInfo,
  IN UINT16             NewFilter
  )
{
  BOOLEAN                 ChangedPromiscuousFlag;
  BOOLEAN                 ChangedMulticastPromiscuousFlag;
  BOOLEAN                 ChangedBroadcastFlag;
  enum i40e_status_code   I40eStatus;

  DEBUGPRINT (RXFILTER, ("NewFilter %x= \n", NewFilter));

  ChangedPromiscuousFlag = FALSE;
  ChangedMulticastPromiscuousFlag = FALSE;
  ChangedBroadcastFlag = FALSE;

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
    if (!AdapterInfo->Vsi.EnablePromiscuous) {
      ChangedPromiscuousFlag = TRUE;
    }
    AdapterInfo->Vsi.EnablePromiscuous = TRUE;
    DEBUGPRINT (RXFILTER, ("  Promiscuous\n"));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
    if (!AdapterInfo->Vsi.EnableBroadcast) {
      ChangedBroadcastFlag = TRUE;
    }
    AdapterInfo->Vsi.EnableBroadcast = TRUE;
    DEBUGPRINT (RXFILTER, ("  Broadcast\n"));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
    if (!AdapterInfo->Vsi.EnableMulticastPromiscuous) {
      ChangedMulticastPromiscuousFlag = TRUE;
    }
    AdapterInfo->Vsi.EnableMulticastPromiscuous = TRUE;
    DEBUGPRINT (RXFILTER, ("  MulticastPromiscuous\n"));
  }

  if (!AdapterInfo->DriverBusy) {
    if (ChangedPromiscuousFlag
      || ChangedMulticastPromiscuousFlag
      || ChangedBroadcastFlag)
    {
      I40eStatus = i40e_aq_set_vsi_unicast_promiscuous(
                     &AdapterInfo->Hw, AdapterInfo->Vsi.Seid,
                     AdapterInfo->Vsi.EnablePromiscuous,
                     NULL,
                     TRUE
                     );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_set_vsi_unicast_promiscuous returned %d\n", I40eStatus));
      }

      I40eStatus = i40e_aq_set_vsi_multicast_promiscuous(
                     &AdapterInfo->Hw, AdapterInfo->Vsi.Seid,
                     AdapterInfo->Vsi.EnablePromiscuous || AdapterInfo->Vsi.EnableMulticastPromiscuous,
                     NULL
                     );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_set_vsi_multicast_promiscuous returned %d\n", I40eStatus));
      }

      I40eStatus = i40e_aq_set_vsi_broadcast(
                     &AdapterInfo->Hw,
                     AdapterInfo->Vsi.Seid,
                     AdapterInfo->Vsi.EnablePromiscuous || AdapterInfo->Vsi.EnableBroadcast,
                     NULL
                     );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_set_vsi_broadcast returned %d\n", I40eStatus));
      }
    }
  }

  AdapterInfo->RxFilter |= NewFilter;
}

/** Clears receive filters.

  @param[in]  AdapterInfo  Pointer to the adapter structure
  @param[in]  NewFilter    A PXE_OPFLAGS bit field indicating what filters to clear.

  @return     Broad/Multicast and promiscous settings are cleared according to NewFilter
**/
VOID
I40eClearFilter (
  I40E_DRIVER_DATA   *AdapterInfo,
  UINT16             NewFilter
)
{
  BOOLEAN                 ChangedPromiscuousFlag;
  BOOLEAN                 ChangedMulticastPromiscuousFlag;
  BOOLEAN                 ChangedBroadcastFlag;
  enum i40e_status_code   I40eStatus;

  ChangedPromiscuousFlag = FALSE;
  ChangedMulticastPromiscuousFlag = FALSE;
  ChangedBroadcastFlag = FALSE;

  DEBUGPRINT (RXFILTER, ("NewFilter %x= \n", NewFilter));

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
    if (AdapterInfo->Vsi.EnablePromiscuous) {
      ChangedPromiscuousFlag = TRUE;
    }
    AdapterInfo->Vsi.EnablePromiscuous = FALSE;
    DEBUGPRINT (RXFILTER, ("  Promiscuous\n"));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
    if (AdapterInfo->Vsi.EnableBroadcast) {
      ChangedBroadcastFlag = TRUE;
    }
    AdapterInfo->Vsi.EnableBroadcast = FALSE;
    DEBUGPRINT (RXFILTER, ("  Broadcast\n"));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
    if (AdapterInfo->Vsi.EnableMulticastPromiscuous) {
      ChangedMulticastPromiscuousFlag = TRUE;
    }
    AdapterInfo->Vsi.EnableMulticastPromiscuous = FALSE;
    DEBUGPRINT (RXFILTER, ("  MulticastPromiscuous\n"));
  }

  if (!AdapterInfo->DriverBusy) {
    if (ChangedPromiscuousFlag
      || ChangedMulticastPromiscuousFlag
      || ChangedBroadcastFlag)
    {
      I40eStatus = i40e_aq_set_vsi_unicast_promiscuous(
                     &AdapterInfo->Hw, AdapterInfo->Vsi.Seid,
                     AdapterInfo->Vsi.EnablePromiscuous,
                     NULL,
                     TRUE
                   );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_set_vsi_unicast_promiscuous returned %d\n", I40eStatus));
      }

      I40eStatus = i40e_aq_set_vsi_multicast_promiscuous(
                     &AdapterInfo->Hw, AdapterInfo->Vsi.Seid,
                     AdapterInfo->Vsi.EnableMulticastPromiscuous || AdapterInfo->Vsi.EnablePromiscuous,
                     NULL
                   );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_set_vsi_multicast_promiscuous returned %d\n", I40eStatus));
      }

      I40eStatus = i40e_aq_set_vsi_broadcast(
                     &AdapterInfo->Hw,
                     AdapterInfo->Vsi.Seid,
                     AdapterInfo->Vsi.EnableBroadcast || AdapterInfo->Vsi.EnablePromiscuous,
                     NULL
                   );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_set_vsi_broadcast returned %d\n", I40eStatus));
      }
    }
  }

  AdapterInfo->RxFilter &= ~NewFilter;
}

/** Adds MAC/VLAN elements to multicast list

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @return  MAC/VLAN elements from adapter VSI structure are added to list
**/
VOID
I40eSetMcastList (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  struct i40e_aqc_remove_macvlan_element_data MacVlanElementsToRemove[MAX_MCAST_ADDRESS_CNT];
  struct i40e_aqc_add_macvlan_element_data    MacVlanElementsToAdd[MAX_MCAST_ADDRESS_CNT];
  enum i40e_status_code                       I40eStatus = I40E_SUCCESS;
  UINTN                                       i;
  UINT32                                      Reg = 0;

  DEBUGDUMP(
    INIT, ("SM(%d,%d):",
    AdapterInfo->Vsi.McastListToProgram.Length, AdapterInfo->Vsi.CurrentMcastList.Length)
  );

  DEBUGPRINT (
    RXFILTER, ("McastListToProgram.Length = %d\n",
    AdapterInfo->Vsi.McastListToProgram.Length)
  );
  DEBUGPRINT (
    RXFILTER, ("CurrentMcastList.Length = %d\n",
    AdapterInfo->Vsi.CurrentMcastList.Length)
  );

  if (!AdapterInfo->DriverBusy) {

    // Remove existing elements from the Forwarding Table
    if (AdapterInfo->Vsi.CurrentMcastList.Length > 0) {
      for (i = 0; i < AdapterInfo->Vsi.CurrentMcastList.Length; i++) {
        DEBUGPRINT (
          RXFILTER, ("Remove MAC %d, %x:%x:%x:%x:%x:%x\n",
          i,
          AdapterInfo->Vsi.CurrentMcastList.McAddr[i][0],
          AdapterInfo->Vsi.CurrentMcastList.McAddr[i][1],
          AdapterInfo->Vsi.CurrentMcastList.McAddr[i][2],
          AdapterInfo->Vsi.CurrentMcastList.McAddr[i][3],
          AdapterInfo->Vsi.CurrentMcastList.McAddr[i][4],
          AdapterInfo->Vsi.CurrentMcastList.McAddr[i][5])
        );
        CopyMem(
          MacVlanElementsToRemove[i].mac_addr,
          &AdapterInfo->Vsi.CurrentMcastList.McAddr[i],
          6
        );
        MacVlanElementsToRemove[i].vlan_tag = 0;
        MacVlanElementsToRemove[i].flags = I40E_AQC_MACVLAN_DEL_IGNORE_VLAN | I40E_AQC_MACVLAN_DEL_PERFECT_MATCH;
      }

      // For FPK - switch Rx drop policy when removing macvlan filter
      if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
        Reg = I40eRead32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB);
        I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, 0);
      }

      I40eStatus = i40e_aq_remove_macvlan(
                     &AdapterInfo->Hw,
                     AdapterInfo->Vsi.Seid,
                     MacVlanElementsToRemove,
                     AdapterInfo->Vsi.CurrentMcastList.Length,
                     NULL
                   );

      if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
        I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, Reg);
      }

      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (
          CRITICAL, ("i40e_aq_remove_macvlan returned %d, aq error = %d\n",
          I40eStatus,
          AdapterInfo->Hw.aq.asq_last_status)
        );
        for (i = 0; i < AdapterInfo->Vsi.CurrentMcastList.Length; i++) {
          DEBUGPRINT (CRITICAL, ("i40e_aq_remove_macvlan %d, %d\n", i, MacVlanElementsToRemove[i].error_code));
        }
      }
    }

    // Add new elements to the Forwarding Table
    if (AdapterInfo->Vsi.McastListToProgram.Length > 0) {
      for (i = 0; i < AdapterInfo->Vsi.McastListToProgram.Length; i++) {
        DEBUGPRINT (RXFILTER, ("Add MAC %d, %x:%x:%x:%x:%x:%x\n",
          i,
          AdapterInfo->Vsi.McastListToProgram.McAddr[i][0],
          AdapterInfo->Vsi.McastListToProgram.McAddr[i][1],
          AdapterInfo->Vsi.McastListToProgram.McAddr[i][2],
          AdapterInfo->Vsi.McastListToProgram.McAddr[i][3],
          AdapterInfo->Vsi.McastListToProgram.McAddr[i][4],
          AdapterInfo->Vsi.McastListToProgram.McAddr[i][5])
        );
        CopyMem(
          MacVlanElementsToAdd[i].mac_addr,
          &AdapterInfo->Vsi.McastListToProgram.McAddr[i],
          6
        );
        MacVlanElementsToAdd[i].vlan_tag = 0;
        MacVlanElementsToAdd[i].flags = I40E_AQC_MACVLAN_ADD_IGNORE_VLAN | I40E_AQC_MACVLAN_ADD_PERFECT_MATCH;
      }

      I40eStatus = i40e_aq_add_macvlan(
                     &AdapterInfo->Hw,
                     AdapterInfo->Vsi.Seid,
                     MacVlanElementsToAdd,
                     AdapterInfo->Vsi.McastListToProgram.Length,
                     NULL
                   );
      if (I40eStatus != I40E_SUCCESS) {
        DEBUGPRINT (
          CRITICAL, ("i40e_aq_add_macvlan returned %d, aq error = %d\n",
          I40eStatus,
          AdapterInfo->Hw.aq.asq_last_status)
        );
        for (i = 0; i < AdapterInfo->Vsi.McastListToProgram.Length; i++) {
          DEBUGPRINT (RXFILTER, ("i40e_aq_add_macvlan %d, %d\n", i, MacVlanElementsToAdd[i].match_method ));
        }
      }
    }
  }

  // Update CurrentMcastList
  CopyMem(
    AdapterInfo->Vsi.CurrentMcastList.McAddr,
    AdapterInfo->Vsi.McastListToProgram.McAddr,
    AdapterInfo->Vsi.McastListToProgram.Length * PXE_MAC_LENGTH
  );
  AdapterInfo->Vsi.CurrentMcastList.Length = AdapterInfo->Vsi.McastListToProgram.Length;
}


/** Starts Rx and Tx rings.

   Enable rings by using Queue enable registers

   @param[in]  AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval     I40E_SUCCESS      TX/RX rings started successfully
   @retval     I40E_ERR_TIMEOUT  Waiting for TX queue status timed out
   @retval     I40E_ERR_TIMEOUT  Waiting for RX queue status timed out
**/
enum i40e_status_code
I40eReceiveStart (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  struct i40e_hw *Hw;
  UINTN           j;

  Hw = &AdapterInfo->Hw;

  // Tell HW that we intend to enable the Tx queue
  i40e_pre_tx_queue_cfg (Hw, 0, TRUE);

  // Enable both Tx and Rx queues by setting proper bits in I40E_QTX_ENA
  // and I40E_QRX_ENA registers. Wait and check if status bits are changed.
  wr32 (Hw, I40E_QTX_ENA (0), (rd32 (Hw, I40E_QTX_ENA (0)) | I40E_QTX_ENA_QENA_REQ_MASK));
  wr32 (Hw, I40E_QRX_ENA (0), (rd32 (Hw, I40E_QRX_ENA (0)) | I40E_QRX_ENA_QENA_REQ_MASK));

  for (j = 0; j < START_RINGS_TIMEOUT; j++) {
    if (rd32 (Hw, I40E_QTX_ENA (0)) & I40E_QTX_ENA_QENA_STAT_MASK) {
      break;
    }
    gBS->Stall (10);
  }
  if (j >= START_RINGS_TIMEOUT) {
    DEBUGPRINT (
      CRITICAL, ("Tx ring enable timed out, value %x\n",
      rd32 (Hw, I40E_QTX_ENA (0)))
    );
    return I40E_ERR_TIMEOUT;
  } else {
    DEBUGPRINT (INIT, ("Tx ring enabled\n"));
  }

  // Wait for the Rx queue status
  for (; j < START_RINGS_TIMEOUT; j++) {
    if (rd32 (Hw, I40E_QRX_ENA (0)) & I40E_QRX_ENA_QENA_STAT_MASK) {
      break;
    }
    gBS->Stall (10);
  }
  if (j >= START_RINGS_TIMEOUT) {
    DEBUGPRINT (
      CRITICAL, ("Rx ring enable timed out, value %x\n",
      rd32 (Hw, I40E_QRX_ENA (0)))
    );
    return I40E_ERR_TIMEOUT;
  } else {
    DEBUGPRINT (INIT, ("Rx ring enabled\n"));
  }

  AdapterInfo->ReceiveStarted = TRUE;

  return I40E_SUCCESS;
}

/** Stops Rx and Tx rings.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @retval     I40E_SUCCESS      TX/RX rings started successfully
   @retval     I40E_ERR_TIMEOUT  Waiting for TX queue status timed out
   @retval     I40E_ERR_TIMEOUT  Waiting for RX queue status timed out
**/
enum i40e_status_code
I40eReceiveStop (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code      Status;
  struct i40e_hw  *Hw;
  UINTN           j;
  UINT32          Reg = 0;

  // For FPK - switch Rx drop policy when stopping Rx rings
  if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
    Reg = I40eRead32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB);
    I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, 0);
  }

  Hw = &AdapterInfo->Hw;

  // Tell HW that we intend to disable the Tx queue
  i40e_pre_tx_queue_cfg (Hw, 0, FALSE);

  // Disable both Tx and Rx queues by setting proper bits in I40E_QTX_ENA
  // and I40E_QRX_ENA registers. Wait and check if status bits are changed.
  wr32 (Hw, I40E_QTX_ENA (0), (rd32 (Hw, I40E_QTX_ENA (0)) & ~I40E_QTX_ENA_QENA_REQ_MASK));
  wr32 (Hw, I40E_QRX_ENA (0), (rd32 (Hw, I40E_QRX_ENA (0)) & ~I40E_QRX_ENA_QENA_REQ_MASK));

  for (j = 0; j < STOP_RINGS_TIMEOUT; j++) {
    if (!(rd32 (Hw, I40E_QTX_ENA (0)) & I40E_QTX_ENA_QENA_STAT_MASK)) {
      break;
    }
    gBS->Stall (10);
  }
  if (j >= STOP_RINGS_TIMEOUT) {
    DEBUGPRINT (
      CRITICAL, ("Tx ring disable timed out, value %x\n",
      rd32 (Hw, I40E_QTX_ENA (0)))
    );
    Status = I40E_ERR_TIMEOUT;
    goto ON_EXIT;
  }
  DEBUGPRINT (INIT, ("Tx ring disabled\n"));

  for (j = 0; j < STOP_RINGS_TIMEOUT; j++) {
    if (!(rd32 (Hw, I40E_QRX_ENA (0)) & I40E_QRX_ENA_QENA_STAT_MASK)) {
      break;
    }
    gBS->Stall (10);
  }
  if (j >= STOP_RINGS_TIMEOUT) {
    DEBUGPRINT (
      CRITICAL, ("Rx ring disable timed out, value %x\n",
      rd32 (Hw, I40E_QRX_ENA (0)))
    );
    Status = I40E_ERR_TIMEOUT;
    goto ON_EXIT;
  }
  DEBUGPRINT (INIT, ("Rx ring disabled\n"));

  gBS->Stall (50000);

  AdapterInfo->ReceiveStarted = FALSE;

  Status = I40E_SUCCESS;

ON_EXIT:
  if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
    I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, Reg);
  }
  return Status;
}

/** Sets base queue in VSI structure to first queue from PF queue allocation
   register

   @param[in]   AdapterInfo  Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval    EFI_SUCCESS   Base queue set successfully (always returned)
**/
EFI_STATUS
I40eSetupPfQueues (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32 Reg;
  UINT16 FirstQueue;
#if (DBG_LVL & INIT)
  UINT16 LastQueue;
#endif /* (DBG_LVL & INIT) */

  Reg = i40e_read_rx_ctl (&AdapterInfo->Hw, I40E_PFLAN_QALLOC);
  FirstQueue = (Reg & I40E_PFLAN_QALLOC_FIRSTQ_MASK) >> I40E_PFLAN_QALLOC_FIRSTQ_SHIFT;

#if (DBG_LVL & INIT)
  LastQueue = (Reg & I40E_PFLAN_QALLOC_LASTQ_MASK) >> I40E_PFLAN_QALLOC_LASTQ_SHIFT;
  DEBUGPRINT (INIT, ("PF Queues - first: %x, last: %x\n", FirstQueue, LastQueue));
#endif /* (DBG_LVL & INIT) */

  AdapterInfo->Vsi.BaseQueue = FirstQueue;
  return EFI_SUCCESS;
}

/** Configure transmit and receive descriptor rings in HMC context

  @param[in]  AdapterInfo  Pointer to the NIC data structure information
                           the UNDI driver is layering on

  @retval     EFI_STATUS        TX/RX queues configured successfully
  @retval     EFI_DEVICE_ERROR  Failed to clear LAN Tx queue context on Tx ring
  @retval     EFI_DEVICE_ERROR  Failed to set LAN Tx queue context on Tx ring
  @retval     EFI_DEVICE_ERROR  Failed to clear LAN Rx queue context on Rx ring
  @retval     EFI_DEVICE_ERROR  Failed to set LAN Rx queue context on Rx ring
  @retval     EFI_OUT_OF_RESOURCES  Could not allocate buffer for Rx Ring
 **/
EFI_STATUS
I40eConfigureTxRxQueues (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum   i40e_status_code    I40eStatus;
  struct i40e_hw            *Hw;
  struct i40e_hmc_obj_txq    TxHmcContext;
  struct i40e_hmc_obj_rxq    RxHmcContext;
  UINT32                     QTxCtrl;
  union i40e_16byte_rx_desc *ReceiveDescriptor;
  UINTN                      i;
  EFI_STATUS                 Status;
  I40E_RING                  *RxRing;
  I40E_RING                  *TxRing;
  UNDI_DMA_MAPPING           *RxBufferMapping;
  EFI_PHYSICAL_ADDRESS       RxBufferPhysicalAddress;

  DEBUGPRINT (INIT, ("\n"));

  Hw              = &AdapterInfo->Hw;
  I40eStatus      = I40E_SUCCESS;
  Status          = EFI_DEVICE_ERROR;
  RxRing          = &AdapterInfo->Vsi.RxRing;
  TxRing          = &AdapterInfo->Vsi.TxRing;
  RxBufferMapping = &RxRing->RxBufferMapping;

  // Now associate the queue with the PCI function
  QTxCtrl = I40E_QTX_CTL_PF_QUEUE;
  QTxCtrl |= ((Hw->pf_id << I40E_QTX_CTL_PF_INDX_SHIFT) & I40E_QTX_CTL_PF_INDX_MASK);
  wr32 (Hw, I40E_QTX_CTL (0), QTxCtrl);

  // Prepare LAN context structures for our tx and rx queues and setup the HMC

  // Clear the context structure before use
  ZeroMem (&TxHmcContext, sizeof (struct i40e_hmc_obj_txq));

  TxHmcContext.new_context = 1;
  TxHmcContext.base = (UINT64) TxRing->Mapping.PhysicalAddress / 128;
  TxHmcContext.qlen = TxRing->Count;

  // Disable FCoE
  TxHmcContext.fc_ena = 0;

  TxHmcContext.timesync_ena = 0;
  TxHmcContext.fd_ena = 0;
  TxHmcContext.alt_vlan_ena = 0;

  // By default all traffic is assigned to TC0
  TxHmcContext.rdylist = AdapterInfo->Vsi.Info.qs_handle[0];
  TxHmcContext.rdylist_act = 0;


#ifndef DIRECT_QUEUE_CTX_PROGRAMMING

  // Clear the context in the HMC
  I40eStatus = i40e_clear_lan_tx_queue_context (Hw, 0);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Failed to clear LAN Tx queue context on Tx ring, error: %d\n",
      I40eStatus)
    );
    return EFI_DEVICE_ERROR;
  }
#endif /* DIRECT_QUEUE_CTX_PROGRAMMING */

  // Set the context in the HMC
#ifdef DIRECT_QUEUE_CTX_PROGRAMMING
  I40eStatus = i40e_set_lan_tx_queue_context_directly (Hw, AdapterInfo->Vsi.BaseQueue, &TxHmcContext);
  //HmcDump(AdapterInfo, 0, I40E_HMC_LAN_TX);
#else /* NOT DIRECT_QUEUE_CTX_PROGRAMMING */
  I40eStatus = i40e_set_lan_tx_queue_context (Hw, 0, &TxHmcContext);
#endif /* DIRECT_QUEUE_CTX_PROGRAMMING */
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Failed to set LAN Tx queue context on Tx ring, error: %d\n",
      I40eStatus)
    );
    return EFI_DEVICE_ERROR;
  }

  // Clear the context structure first
  ZeroMem (&RxHmcContext, sizeof (struct i40e_hmc_obj_rxq));

  RxRing->RxBufLen = I40E_RXBUFFER_2048;

  // No packet split
  RxRing->RxHdrLen = 0;

  RxHmcContext.head = 0;
  RxHmcContext.cpuid = 0;

  RxHmcContext.dbuff = (UINT8) (RxRing->RxBufLen >> I40E_RXQ_CTX_DBUFF_SHIFT);
  RxHmcContext.hbuff = (UINT8) (RxRing->RxHdrLen >> I40E_RXQ_CTX_HBUFF_SHIFT);

  RxHmcContext.base = (UINT64) RxRing->Mapping.PhysicalAddress / 128;
  RxHmcContext.qlen = RxRing->Count;

  // 16 byte descriptors in use
  RxHmcContext.dsize = 0;

  RxHmcContext.dtype = I40E_RX_DTYPE_NO_SPLIT;
  RxHmcContext.hsplit_0 = I40E_HMC_OBJ_RX_HSPLIT_0_NO_SPLIT;

  RxHmcContext.rxmax = 0x600;
  RxHmcContext.tphrdesc_ena = 0;
  RxHmcContext.tphwdesc_ena = 0;
  RxHmcContext.tphdata_ena = 0;
  RxHmcContext.tphhead_ena = 0;
  RxHmcContext.lrxqthresh = 0;
  RxHmcContext.crcstrip = 1;

  // No FCoE
  RxHmcContext.fc_ena = 0;


#ifndef DIRECT_QUEUE_CTX_PROGRAMMING

  // Clear the context in the HMC
  I40eStatus = i40e_clear_lan_rx_queue_context (Hw, 0);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Failed to clear LAN Rx queue context on Rx ring, error: %d\n",
      I40eStatus)
    );
    return EFI_DEVICE_ERROR;
  }
#endif /* DIRECT_QUEUE_CTX_PROGRAMMING */

  // Set the context in the HMC
#ifdef DIRECT_QUEUE_CTX_PROGRAMMING
  I40eStatus = i40e_set_lan_rx_queue_context_directly (Hw, AdapterInfo->Vsi.BaseQueue, &RxHmcContext);
  //HmcDump(AdapterInfo, 0, I40E_HMC_LAN_RX);
#else /* NOT DIRECT_QUEUE_CTX_PROGRAMMING */
  I40eStatus = i40e_set_lan_rx_queue_context (Hw, 0, &RxHmcContext);
#endif /* DIRECT_QUEUE_CTX_PROGRAMMING */

  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Failed to set LAN Rx queue context on Rx ring, error: %d\n",
      I40eStatus)
    );
    return EFI_DEVICE_ERROR;
  }

  // Initialize tail register
  I40eWrite32 (AdapterInfo, I40E_QRX_TAIL (0), 0);
  I40eWrite32 (AdapterInfo, I40E_QRX_TAIL (0), RxRing->Count - 1);
  RxRing->NextToUse = 0;

#if (DBG_LVL & RX)
  UINT32 QRxTail = I40eRead32 (AdapterInfo, I40E_QRX_TAIL (0));
  DEBUGPRINT (INIT, ("QRXTail %d\n", QRxTail));
#endif /* (DBG_LVL & RX) */

  // Determine the overall size of memory needed for receive buffers and allocate memory
  RxBufferMapping->Size = ALIGN (RxRing->Count * RxRing->RxBufLen, 4096);

  //
  // Allocate a common buffer for Rx buffers
  //
  Status = UndiDmaAllocateCommonBuffer (AdapterInfo->PciIo, RxBufferMapping);

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Failed to allocate memory for Rx buffers: %r\n", Status));
    return Status;
  }

  // Link the RX Descriptors to the receive buffers and cleanup descriptors
  for (i = 0; i < RxRing->Count; i++) {
    ReceiveDescriptor = I40E_RX_DESC (RxRing, i);

    RxRing->BufferAddresses[i] = (UINT8*) (RxBufferMapping->UnmappedAddress +
                                           i * RxRing->RxBufLen);

    RxBufferPhysicalAddress = RxBufferMapping->PhysicalAddress +
                                        i * RxRing->RxBufLen;

    ReceiveDescriptor->read.pkt_addr = RxBufferPhysicalAddress;

    ReceiveDescriptor->read.hdr_addr = 0;

    ReceiveDescriptor->wb.qword1.status_error_len = 0;
  }

  return EFI_SUCCESS;
}

/** Free resources allocated for transmit and receive descriptor rings and remove
  HMC contexts

  @param[in]  AdapterInfo  Pointer to the NIC data structure information
                             the UNDI driver is layering on

  @retval     EFI_SUCCESS       TX/RX resources freed successfully
  @retval     EFI_DEVICE_ERROR  Failed to clear LAN Tx queue context on Tx ring
  @retval     EFI_DEVICE_ERROR  Failed to clear LAN Rx queue context on Rx ring
**/
EFI_STATUS
I40eFreeTxRxQueues (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum   i40e_status_code I40eStatus;
  struct i40e_hw         *Hw;
  EFI_STATUS              Status;

  DEBUGPRINT (INIT, ("\n"));

  Hw = &AdapterInfo->Hw;
  I40eStatus = I40E_SUCCESS;
  Status = EFI_DEVICE_ERROR;

#ifndef DIRECT_QUEUE_CTX_PROGRAMMING

  // Clear the context in the HMC
  I40eStatus = i40e_clear_lan_tx_queue_context (Hw, 0);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Failed to clear LAN Tx queue context on Tx ring, error: %d\n",
      I40eStatus)
    );
    return EFI_DEVICE_ERROR;
  }

  // Clear the context in the HMC
  I40eStatus = i40e_clear_lan_rx_queue_context (Hw, 0);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Failed to clear LAN Rx queue context on Rx ring, error: %d\n",
      I40eStatus)
    );
    return EFI_DEVICE_ERROR;
  }
#endif /* DIRECT_QUEUE_CTX_PROGRAMMING */
  Status = UndiDmaFreeCommonBuffer (
             AdapterInfo->PciIo,
             &AdapterInfo->Vsi.RxRing.RxBufferMapping
             );

  return Status;
}

/** Allocate memory resources for the Tx and Rx descriptors

  @param[in]  AdapterInfo  Pointer to the NIC data structure information
                             the UNDI driver is layering on

  @retval     EFI_SUCCESS           Resources allocated succesfully
  @retval     EFI_OUT_OF_RESOURCES  Could not allocate buffer for Tx descriptor ring
  @retval     EFI_OUT_OF_RESOURCES  Could not allocate buffer for Rx descriptor ring
**/
EFI_STATUS
I40eSetupTxRxResources (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  EFI_STATUS          Status;
  UINTN               i;
  I40E_RING           *TxRing;
  I40E_RING           *RxRing;

  TxRing = &AdapterInfo->Vsi.TxRing;
  RxRing = &AdapterInfo->Vsi.RxRing;

  TxRing->Count = AdapterInfo->Vsi.NumDesc;
  TxRing->Size = 0;

  RxRing->Count = AdapterInfo->Vsi.NumDesc;
  RxRing->Size = 0;

  //
  // This block is for Tx descriptiors
  // Round up to nearest 4K
  //
  TxRing->Size = ALIGN (TxRing->Count * sizeof (struct i40e_tx_desc), 4096);
  TxRing->Mapping.Size = TxRing->Size;

  Status = UndiDmaAllocateCommonBuffer (AdapterInfo->PciIo, &TxRing->Mapping);

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Failed to allocate memory for Tx desc ring: %r\n", Status));
    return Status;
  }

  TxRing->NextToUse = 0;
  TxRing->NextToClean = 0;

  // All available transmit descriptors are free by default
  for (i = 0; i < TxRing->Count; i++) {
    ZeroMem (&TxRing->TxBufferMappings[i], sizeof (UNDI_DMA_MAPPING));
  }

  //  This block is for Rx descriptors
  //  Use 16 byte descriptors as we are in PXE MODE.

  // Round up to nearest 4K
  RxRing->Size = ALIGN (RxRing->Count * sizeof (union i40e_16byte_rx_desc), 4096);
  RxRing->Mapping.Size = RxRing->Size;

  Status = UndiDmaAllocateCommonBuffer (AdapterInfo->PciIo, &RxRing->Mapping);

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Failed to allocate memory for Rx desc ring: %r\n", Status));

    UndiDmaFreeCommonBuffer (AdapterInfo->PciIo, &TxRing->Mapping);
    return Status;
  }

  RxRing->NextToClean = 0;
  RxRing->NextToUse = 0;

  return EFI_SUCCESS;
}

/** Free memory resources for the Tx and Rx descriptors

 @param[in]  AdapterInfo  Pointer to the NIC data structure information
                             the UNDI driver is layering on

 @retval     EFI_SUCCESS            Tx/Rx ring descriptor resources freed successfully
 @retval     EFI_INVALID_PARAMETER  Memory pages count was not allocated with Allocate
                                    Buffer() on specified Tx desc. address
 @retval     EFI_INVALID_PARAMETER  Memory pages count was not allocated with Allocate
                                    Buffer() on specified Rx desc. address
**/
EFI_STATUS
I40eFreeTxRxResources (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  EFI_STATUS Status;

  Status = UndiDmaFreeCommonBuffer (
             AdapterInfo->PciIo,
             &AdapterInfo->Vsi.TxRing.Mapping
             );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (
      CRITICAL, ("Unable to free memory for the Tx descriptor ring: %r\n",
      Status)
    );
    return Status;
  }

  Status = UndiDmaFreeCommonBuffer (
             AdapterInfo->PciIo,
             &AdapterInfo->Vsi.RxRing.Mapping
             );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (
      CRITICAL, ("Unable to free memory for the Rx descriptor ring: %r\n",
      Status)
    );
    return Status;
  }

  return Status;

}

/** Get the current switch configuration from the device and
 extract a few useful SEID values.

 @param[in]  AdapterInfo  Pointer to the NIC data structure information
                          the UNDI driver is layering on

 @retval     EFI_SUCCESS            Switch configuration read successfully
 @retval     EFI_INVALID_PARAMETER  AdapterInfo is NULL
 @retval     EFI_DEVICE_ERROR       get_switch_config AQ cmd failed
 @retval     EFI_DEVICE_ERROR       No data returned in Switch Config
**/
EFI_STATUS
I40eReadSwitchConfiguration (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  struct i40e_aqc_get_switch_config_resp *SwitchConfig;
  UINT8                                   AqBuffer[I40E_AQ_LARGE_BUF];
  enum i40e_status_code                   I40eStatus;
  UINTN                                   i;
  UINT16                                  StartSeid;

  if (AdapterInfo == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  SwitchConfig = (struct i40e_aqc_get_switch_config_resp *) AqBuffer;
  StartSeid = 0;
  I40eStatus = i40e_aq_get_switch_config (
                 &AdapterInfo->Hw,
                 SwitchConfig,
                 sizeof (AqBuffer),
                 &StartSeid,
                 NULL
               );
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("get switch config failed %d aq_err=%x\n",
      I40eStatus, AdapterInfo->Hw.aq.asq_last_status)
    );
    return EFI_DEVICE_ERROR;
  }

  if (SwitchConfig->header.num_reported == 0) {
    DEBUGPRINT (CRITICAL, ("No data returned in the SwitchConfig \n"));
    return EFI_DEVICE_ERROR;
  }

  for (i = 0; i < SwitchConfig->header.num_reported; i++) {
    DEBUGPRINT (
      INIT, ("type=%d seid=%d uplink=%d downlink=%d\n",
      SwitchConfig->element[i].element_type,
      SwitchConfig->element[i].seid,
      SwitchConfig->element[i].uplink_seid,
      SwitchConfig->element[i].downlink_seid)
    );

    switch (SwitchConfig->element[i].element_type) {
    case I40E_SWITCH_ELEMENT_TYPE_MAC:
      AdapterInfo->MacSeid = SwitchConfig->element[i].seid;
      break;
    case I40E_SWITCH_ELEMENT_TYPE_VEB:
      AdapterInfo->VebSeid = SwitchConfig->element[i].seid;
      break;
    case I40E_SWITCH_ELEMENT_TYPE_PF:
      AdapterInfo->PfSeid = SwitchConfig->element[i].seid;
      AdapterInfo->MainVsiSeid = SwitchConfig->element[i].uplink_seid;
      break;
    case I40E_SWITCH_ELEMENT_TYPE_VSI:
      AdapterInfo->MainVsiSeid = SwitchConfig->element[i].seid;
      AdapterInfo->PfSeid = SwitchConfig->element[i].downlink_seid;
      AdapterInfo->MacSeid = SwitchConfig->element[i].uplink_seid;
      break;

    // ignore these for now
    case I40E_SWITCH_ELEMENT_TYPE_VF:
    case I40E_SWITCH_ELEMENT_TYPE_EMP:
    case I40E_SWITCH_ELEMENT_TYPE_BMC:
    case I40E_SWITCH_ELEMENT_TYPE_PE:
    case I40E_SWITCH_ELEMENT_TYPE_PA:
      break;
    default:
      DEBUGPRINT (
        CRITICAL, ("Unknown element type=%d seid=%d\n",
        SwitchConfig->element[i].element_type,
        SwitchConfig->element[i].seid)
      );
      break;
    }
  }

  return EFI_SUCCESS;
}

/** Turn off VLAN stripping for the VSI

  @param[in]  AdapterInfo  Pointer to the NIC data structure information
                             the UNDI driver is layering on

  @retval    EFI_SUCCESS       VLAN stripping successfully disabled
  @retval    EFI_SUCCESS       VLAN stripping already disabled
  @retval    EFI_DEVICE_ERROR  Failed to update VSI param
**/
EFI_STATUS
I40eDisableVlanStripping (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  struct i40e_vsi_context VsiContext;
  enum i40e_status_code   I40eStatus;

  ZeroMem (&VsiContext, sizeof (VsiContext));


  if ((AdapterInfo->Vsi.Info.valid_sections & I40E_AQ_VSI_PROP_VLAN_VALID) == I40E_AQ_VSI_PROP_VLAN_VALID) {
    if ((AdapterInfo->Vsi.Info.port_vlan_flags & I40E_AQ_VSI_PVLAN_EMOD_MASK) == I40E_AQ_VSI_PVLAN_EMOD_MASK) {
      DEBUGPRINT (INIT, ("VLAN stripping already disabled\n"));
      return EFI_SUCCESS;
    }
  }

  AdapterInfo->Vsi.Info.valid_sections |= I40E_AQ_VSI_PROP_VLAN_VALID;
  AdapterInfo->Vsi.Info.port_vlan_flags = I40E_AQ_VSI_PVLAN_MODE_ALL |
                                          I40E_AQ_VSI_PVLAN_EMOD_NOTHING;

  VsiContext.seid = AdapterInfo->MainVsiSeid;
  CopyMem (&VsiContext.info, &AdapterInfo->Vsi.Info, sizeof (AdapterInfo->Vsi.Info));

  I40eStatus = i40e_aq_update_vsi_params (&AdapterInfo->Hw, &VsiContext, NULL);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("Update vsi failed, aq_err=%d\n",
      AdapterInfo->Hw.aq.asq_last_status)
    );
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}


/** Setup the initial LAN and VMDq switch.

   This adds the VEB into the internal switch, makes sure the main
   LAN VSI is connected correctly, allocates and connects all the
   VMDq VSIs, and sets the base queue index for each VSI.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval  EFI_SUCCESS        Successfull LAN and VMDq setup
   @retval  EFI_DEVICE_ERROR   Failed to get VSI params
   @retval  EFI_DEVICE_ERROR   VSI has not enough queue pairs
   @retval  EFI_DEVICE_ERROR   Failed to set filter control settings
   @retval  EFI_DEVICE_ERROR   add_macvlan AQ cmd failed
   @retval  EFI_DEVICE_ERROR   Failed to disable VLAN stripping
**/
EFI_STATUS
I40eSetupPFSwitch (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  EFI_STATUS                               Status;
  struct i40e_vsi_context                  VsiCtx;
  struct i40e_aqc_add_macvlan_element_data MacVlan;
  enum i40e_status_code                    I40eStatus;

  I40eStatus = I40E_SUCCESS;
  Status = EFI_SUCCESS;
  ZeroMem (&VsiCtx, sizeof (VsiCtx));

  // Read the default switch configuration
  Status = I40eReadSwitchConfiguration (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eReadSwitchConfiguration returned %r\n", Status));
    return Status;
  }

  // Get main VSI parameters
  I40eStatus = I40eGetVsiParams (AdapterInfo, &VsiCtx);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("I40e_aq_get_vsi_params returned %d, aq_err %d\n",
      I40eStatus, AdapterInfo->Hw.aq.asq_last_status)
    );
    return EFI_DEVICE_ERROR;
  }

  DEBUGPRINT (
    INIT, ("VSI params: vsi_number=%d, VsiCtx.info.qs_handle[0]=%d\n",
    VsiCtx.vsi_number,
    VsiCtx.info.qs_handle[0])
  );

  AdapterInfo->Vsi.Id = VsiCtx.vsi_number;

  // Determnine which queues are used by this PF
  I40eSetupPfQueues (AdapterInfo);

  //  Set number of queue paires we use to 1
  AdapterInfo->NumLanQps = 1;

  // Check if VSI has enough queue pairs
  if ((AdapterInfo->Hw.func_caps.num_tx_qp < AdapterInfo->NumLanQps)
    || (AdapterInfo->Hw.func_caps.num_rx_qp < AdapterInfo->NumLanQps))
  {
    DEBUGPRINT (CRITICAL, ("Not enough qps available\n"));
    return EFI_DEVICE_ERROR;
  }

  // Store VSI parameters in VSI structure
  AdapterInfo->Vsi.Type = I40E_VSI_MAIN;
  AdapterInfo->Vsi.Flags = 0;
  AdapterInfo->Vsi.NumQueuePairs = AdapterInfo->NumLanQps;
  AdapterInfo->Vsi.NumDesc = AdapterInfo->TxRxDescriptorCount;
  AdapterInfo->Vsi.Seid = AdapterInfo->MainVsiSeid;
  CopyMem (&AdapterInfo->Vsi.Info, &VsiCtx.info, sizeof (VsiCtx.info));

  {
    struct i40e_filter_control_settings FilterControlSettings;

    ZeroMem (&FilterControlSettings, sizeof (FilterControlSettings));

    FilterControlSettings.hash_lut_size = I40E_HASH_LUT_SIZE_128;
    FilterControlSettings.enable_ethtype = TRUE;
    FilterControlSettings.enable_macvlan = TRUE;
    I40eStatus = i40e_set_filter_control (&AdapterInfo->Hw, &FilterControlSettings);
    if (I40eStatus != I40E_SUCCESS) {
      DEBUGPRINT (
        CRITICAL, ("i40e_set_filter_control returned %d, aq_err %d\n",
        I40eStatus, AdapterInfo->Hw.aq.asq_last_status)
      );
      return EFI_DEVICE_ERROR;
    }
  }

  SetMem (&MacVlan, sizeof (struct i40e_aqc_add_macvlan_element_data), 0);
  MacVlan.flags = I40E_AQC_MACVLAN_ADD_IGNORE_VLAN | I40E_AQC_MACVLAN_ADD_PERFECT_MATCH;
  CopyMem (MacVlan.mac_addr, &AdapterInfo->Hw.mac.addr, sizeof (MacVlan.mac_addr));

  I40eStatus = i40e_aq_add_macvlan (
                 &AdapterInfo->Hw,
                 AdapterInfo->MainVsiSeid,
                 &MacVlan,
                 1,
                 NULL
               );
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("i40e_aq_add_macvlan returned %d, aq_err %d\n",
      I40eStatus,
      AdapterInfo->Hw.aq.asq_last_status)
    );
    return EFI_DEVICE_ERROR;
  }

  // Configure VLAN stripping on Rx packets
  Status = I40eDisableVlanStripping (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eDisableVlanStripping returned %r\n", Status));
    return Status;
  }

  return Status;
}

/** This function performs PCI-E initialization for the device.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval   EFI_SUCCESS            PCI-E initialized successfully
   @retval   EFI_UNSUPPORTED        Failed to get original PCI attributes to save locally
   @retval   EFI_UNSUPPORTED        Failed to get supported PCI command options
   @retval   EFI_UNSUPPORTED        Failed to set PCI command options
**/
EFI_STATUS
I40ePciInit (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  EFI_STATUS Status;
  UINT64     NewCommand;
  UINT64     Result;
  BOOLEAN    PciAttributesSaved;

  NewCommand = 0;
  Result = 0;

  PciAttributesSaved = FALSE;

  // Save original PCI attributes
  Status = AdapterInfo->PciIo->Attributes (
                                 AdapterInfo->PciIo,
                                 EfiPciIoAttributeOperationGet,
                                 0,
                                 &AdapterInfo->OriginalPciAttributes
                               );

  if (EFI_ERROR (Status)) {
    goto Error;
  }
  PciAttributesSaved = TRUE;

  // Get the PCI Command options that are supported by this controller.
  Status = AdapterInfo->PciIo->Attributes (
                                 AdapterInfo->PciIo,
                                 EfiPciIoAttributeOperationSupported,
                                 0,
                                 &Result
                               );

  DEBUGPRINT (INIT, ("Attributes supported %x\n", Result));

  if (!EFI_ERROR (Status)) {

    // Set the PCI Command options to enable device memory mapped IO,
    // port IO, and bus mastering.
    Status = AdapterInfo->PciIo->Attributes (
                                   AdapterInfo->PciIo,
                                   EfiPciIoAttributeOperationEnable,
                                   Result & (EFI_PCI_DEVICE_ENABLE | EFI_PCI_IO_ATTRIBUTE_DUAL_ADDRESS_CYCLE),
                                   &NewCommand
                                 );
  }
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("PciIo->Attributes returned %r\n", Status));
    goto Error;
  }

  AdapterInfo->PciIo->GetLocation (
                        AdapterInfo->PciIo,
                        &AdapterInfo->Segment,
                        &AdapterInfo->Bus,
                        &AdapterInfo->Device,
                        &AdapterInfo->Function
                      );

  // Read all the registers from the device's PCI Configuration space
  AdapterInfo->PciIo->Pci.Read (
                            AdapterInfo->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            MAX_PCI_CONFIG_LEN,
                            AdapterInfo->PciConfig
                          );
  return Status;

Error:
  if (PciAttributesSaved) {

    // Restore original PCI attributes
    AdapterInfo->PciIo->Attributes (
                          AdapterInfo->PciIo,
                          EfiPciIoAttributeOperationSet,
                          AdapterInfo->OriginalPciAttributes,
                          NULL
                        );
  }

  return Status;
}

/** Reads and prints adapter MAC address

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval    EFI_SUCCESS       MAC address read successfully
   @retval    EFI_DEVICE_ERROR  Failed to get MAC address
**/
EFI_STATUS
I40eReadMacAddress (
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code I40eStatus;
  struct i40e_hw       *Hw;

  Hw = &AdapterInfo->Hw;

  // Get current MAC Address using the shared code function
  I40eStatus = i40e_get_mac_addr (Hw, Hw->mac.addr);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_get_mac_addr returned %d\n", I40eStatus));
    return EFI_DEVICE_ERROR;
  }

  // Assume this is also a permanent address and save it for the future
  CopyMem (Hw->mac.perm_addr, Hw->mac.addr, ETHER_MAC_ADDR_LEN);

  DEBUGPRINT (
    INIT, ("MAC Address = %02x:%02x:%02x:%02x:%02x:%02x\n",
    AdapterInfo->Hw.mac.addr[0],
    AdapterInfo->Hw.mac.addr[1],
    AdapterInfo->Hw.mac.addr[2],
    AdapterInfo->Hw.mac.addr[3],
    AdapterInfo->Hw.mac.addr[4],
    AdapterInfo->Hw.mac.addr[5])
  );

  return EFI_SUCCESS;
}


/** Performs HW initialization from child side

   Initializes HMC structure, sets flow control, setups PF switch,
   setups and configures Tx/Rx resources and queues, enables Tx/Rx rings

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @retval    EFI_SUCCESS       HW initialized successfully
   @retval    EFI_DEVICE_ERROR  Failed to initialize HMC structure for LAN function
   @retval    EFI_DEVICE_ERROR  Failed to configure HMC
   @retval    EFI_DEVICE_ERROR  Failed to setup PF switch
   @retval    EFI_OUT_OF_RESOURCES  Failed to setup Tx/Rx resources
   @retval    EFI_DEVICE_ERROR  Failed to configure Tx/Rx queues
   @retval    EFI_OUT_OF_RESOURCES  Failed to configure Tx/Rx queues
**/
EFI_STATUS
I40eInitHw (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code I40eStatus;
  EFI_STATUS            Status;
  struct i40e_hw *      Hw;
  UINT32                TmpReg0 = 0;

  Hw = &AdapterInfo->Hw;


  //  Initialize HMC structure for this Lan function. We need 1 Tx and 1 Rx queue.
  //  FCoE parameters are zeroed
#ifdef DIRECT_QUEUE_CTX_PROGRAMMING
  UNREFERENCED_1PARAMETER (I40eStatus);
#else /* NOT DIRECT_QUEUE_CTX_PROGRAMMING */
  I40eStatus = i40e_init_lan_hmc (Hw, 1, 1, 0, 0);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_init_lan_hmc returned %d\n", I40eStatus));
    return EFI_DEVICE_ERROR;
  }

  I40eStatus = i40e_configure_lan_hmc (Hw, I40E_HMC_MODEL_DIRECT_ONLY);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_configure_lan_hmc returned %d\n", I40eStatus));
    return EFI_DEVICE_ERROR;
  }
#endif /* DIRECT_QUEUE_CTX_PROGRAMMING */

  AdapterInfo->WaitingForLinkUp = IsLinkUp (AdapterInfo);

  Status = I40eSetupPFSwitch (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eSetupPFSwitch returned %r\n", Status));
    return Status;
  }

  Status = I40eSetupTxRxResources (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eSetupTxRxResources returned %r\n", Status));
    return Status;
  }

  Status = I40eConfigureTxRxQueues (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eConfigureTxRxQueues returned %r\n", Status));
    return Status;
  }

  // Enable interrupt causes.
  I40eConfigureInterrupts (AdapterInfo);

  if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
    TmpReg0 = I40eRead32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB);
    I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, 0);
  }

  I40eReceiveStart (AdapterInfo);

  if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
    I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, TmpReg0);
  }


  AdapterInfo->HwInitialized = TRUE;

  return Status;
}

/** Performs I40eInitHw function for UNDI interface

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval    PXE_STATCODE_SUCCESS   HW initialized successfully
   @retval    PXE_STATCODE_NOT_STARTED  Failed to initialize HW
**/
PXE_STATCODE
I40eInitialize (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
#ifndef AVOID_HW_REINITIALIZATION
  EFI_STATUS Status;
#endif /* AVOID_HW_REINITIALIZATION */
  PXE_STATCODE PxeStatcode;

  DEBUGPRINT (INIT, ("Entering I40eInitialize\n"));

  PxeStatcode = PXE_STATCODE_SUCCESS;

  // Do not try to initialize hw again when it is already initialized
  if (AdapterInfo->HwInitialized == FALSE) {
    DEBUGPRINT (INIT, ("Hw is not initialized, calling I40eInitHw\n"));
#ifndef AVOID_HW_REINITIALIZATION
    Status = I40eInitHw (AdapterInfo);
    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("I40eInitHw returns %r\n", Status));
      PxeStatcode = PXE_STATCODE_NOT_STARTED;
    }
#endif /* AVOID_HW_REINITIALIZATION */
  }
  AdapterInfo->DriverBusy = FALSE;
  return PxeStatcode;
}

/** Reverts the operations performed in I40eInitHw. Stops HW from child side

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @retval   PXE_STATCODE_SUCCESS   HW is already not initialized
   @retval   PXE_STATCODE_SUCCESS   HW successfully stopped
**/
PXE_STATCODE
I40eShutdown (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_STATCODE PxeStatcode;
#ifndef AVOID_HW_REINITIALIZATION
  enum i40e_status_code I40eStatus = I40E_SUCCESS;
#endif  /* AVOID_HW_REINITIALIZATION */
  DEBUGPRINT (INIT, ("Entering I40eShutdown\n"));

#if (0)
  if (AdapterInfo->Hw.bus.func == 1) {
    DumpInternalFwHwData (AdapterInfo);
  }

#endif /* (0) */

  if (!AdapterInfo->HwInitialized) {
    PxeStatcode = PXE_STATCODE_SUCCESS;
    return PxeStatcode;
  }

#ifndef AVOID_HW_REINITIALIZATION
  I40eStatus = I40eReceiveStop (AdapterInfo);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("I40eReceiveStop returned %d\n", I40eStatus));
  }

  I40eStatus = I40eFreeTxRxQueues (AdapterInfo);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("I40eFreeTxRxQueues returned %d\n", I40eStatus));
  }

  I40eStatus = I40eFreeTxRxResources (AdapterInfo);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("I40eFreeTxRxResources returned %d\n", I40eStatus));
  }

#ifndef DIRECT_QUEUE_CTX_PROGRAMMING
  I40eStatus = i40e_shutdown_lan_hmc (&AdapterInfo->Hw);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_shutdown_lan_hmc returned %d\n", I40eStatus));
  }

#endif  /* DIRECT_QUEUE_CTX_PROGRAMMING */
  // Disable all interrupt causes.
  I40eDisableInterrupts (AdapterInfo);

  AdapterInfo->HwInitialized = FALSE;
#endif  /* AVOID_HW_REINITIALIZATION */
  PxeStatcode = PXE_STATCODE_SUCCESS;
  return PxeStatcode;
}

/** Performs HW reset by reinitialization

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @retval   PXE_STATCODE_SUCCESS      Successfull HW reset
   @retval   PXE_STATCODE_NOT_STARTED  Failed to initialize HW
**/
PXE_STATCODE
I40eReset (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_STATCODE PxeStatcode;
#ifndef AVOID_HW_REINITIALIZATION
  EFI_STATUS Status;
#endif /* AVOID_HW_REINITIALIZATION */

  DEBUGPRINT (INIT, ("Entering I40eReset\n"));

  // Do not reinitialize the adapter when it has already been initialized
  // This saves the time required for initialization
  if (!AdapterInfo->HwInitialized) {
#ifndef AVOID_HW_REINITIALIZATION
    Status = I40eInitHw (AdapterInfo);
    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("I40eInitHw returns %r\n", Status));
      return PXE_STATCODE_NOT_STARTED;
    }
#endif /* AVOID_HW_REINITIALIZATION */
  } else {
    DEBUGPRINT (I40E, ("Skipping adapter reset\n"));
  }

  PxeStatcode = PXE_STATCODE_SUCCESS;
  return PxeStatcode;
}

/** Configures internal interrupt causes on current PF.

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @return   Interrupt causes are configured for current PF
**/
VOID
I40eConfigureInterrupts (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32  RegVal = 0;

  I40eWrite32 (AdapterInfo, I40E_PFINT_ITR0(0), 0);
  I40eWrite32 (AdapterInfo, I40E_PFINT_ITR0(1), 0);
  I40eWrite32 (AdapterInfo, I40E_PFINT_LNKLST0, 0);

  RegVal = I40E_PFINT_ICR0_ENA_ADMINQ_MASK | I40E_PFINT_ICR0_ENA_LINK_STAT_CHANGE_MASK;
  I40eWrite32 (AdapterInfo, I40E_PFINT_ICR0_ENA, RegVal);

  // Enable Queue 0 for receive interrupt
  RegVal = I40E_QINT_RQCTL_CAUSE_ENA_MASK | I40E_QINT_RQCTL_ITR_INDX_MASK |
    0x1 << I40E_QINT_RQCTL_NEXTQ_INDX_SHIFT |
    I40E_QUEUE_TYPE_TX << I40E_QINT_RQCTL_NEXTQ_TYPE_SHIFT;
  I40eWrite32 (AdapterInfo, I40E_QINT_RQCTL(0), RegVal);

  // Enable Queue 1 for transmit interrupt
  RegVal = I40E_QINT_TQCTL_CAUSE_ENA_MASK | I40E_QINT_TQCTL_ITR_INDX_MASK |
    0x1 << I40E_QINT_RQCTL_MSIX0_INDX_SHIFT | I40E_QINT_TQCTL_NEXTQ_INDX_MASK;
  I40eWrite32 (AdapterInfo, I40E_QINT_TQCTL(0), RegVal);
}

/** Disables internal interrupt causes on current PF.

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @return   Interrupt causes are disabled for current PF
**/
VOID
I40eDisableInterrupts (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  // Disable all non-queue interrupt causes
  I40eWrite32 (AdapterInfo, I40E_PFINT_ICR0_ENA, 0);

  // Disable receive queue interrupt causes
  I40eWrite32 (AdapterInfo, I40E_QINT_RQCTL(0), 0);

  // Disable transmit queue interrupt
  I40eWrite32 (AdapterInfo, I40E_QINT_TQCTL(0), 0);
}

/** Read function capabilities using AQ command.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval   EFI_INVALID_PARAMETER   Failed to allocate memory for function
                                     capabilities buffer
   @retval   EFI_OUT_OF_RESOURCES    Failed to allocate memory for function
                                     capabilities buffer
   @retval   EFI_DEVICE_ERROR        Discover capabilities AQ cmd failed
**/
EFI_STATUS
I40eDiscoverCapabilities (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code                           I40eStatus;
  UINT16                                          BufferSize;
  UINT16                                          BufferSizeNeeded;
  struct i40e_aqc_list_capabilities_element_resp *CapabilitiesBuffer;
  UINT32                                          i = 0;

  BufferSize = 40 * sizeof (struct i40e_aqc_list_capabilities_element_resp);

  do {
    i++;

    CapabilitiesBuffer = AllocateZeroPool (BufferSize);
    if (CapabilitiesBuffer == NULL) {
      return EFI_OUT_OF_RESOURCES;
    }

    I40eStatus = i40e_aq_discover_capabilities (
                   &AdapterInfo->Hw,
                   CapabilitiesBuffer,
                   BufferSize,
                   &BufferSizeNeeded,
                   i40e_aqc_opc_list_func_capabilities,
                   NULL
                 );

    // Free memory that was required only internally
    // in i40e_aq_discover_capabilities
    gBS->FreePool (CapabilitiesBuffer);

    if (AdapterInfo->Hw.aq.asq_last_status == I40E_AQ_RC_ENOMEM) {

      // Buffer passed was to small, use buffer size returned by the function
      BufferSize = BufferSizeNeeded;
    } else if (AdapterInfo->Hw.aq.asq_last_status != I40E_AQ_RC_OK) {
      return EFI_DEVICE_ERROR;
    }

    // Infinite loop protection
    if (i > 100) {
      DEBUGPRINT (CRITICAL, ("i40e_aq_discover_capabilities returns %x\n", I40eStatus));
      return EFI_DEVICE_ERROR;
    }
  } while (I40eStatus != I40E_SUCCESS);

  i = 0;
  do {
      i++;
      CapabilitiesBuffer = AllocateZeroPool (BufferSize);
      if (CapabilitiesBuffer == NULL) {
        return EFI_OUT_OF_RESOURCES;
      }

      I40eStatus = i40e_aq_discover_capabilities (
                     &AdapterInfo->Hw,
                     CapabilitiesBuffer,
                     BufferSize,
                     &BufferSizeNeeded,
                     i40e_aqc_opc_list_dev_capabilities,
                     NULL
                   );

      // Free memory that was required only internally
      // in i40e_aq_discover_capabilities
      gBS->FreePool (CapabilitiesBuffer);

      if (AdapterInfo->Hw.aq.asq_last_status == I40E_AQ_RC_ENOMEM) {

        // Buffer passed was to small, use buffer size returned by the function
        BufferSize = BufferSizeNeeded;
      } else if (AdapterInfo->Hw.aq.asq_last_status != I40E_AQ_RC_OK) {
        return EFI_DEVICE_ERROR;
      }

      // Endless loop protection
      if (i > 100) {
        DEBUGPRINT (CRITICAL, ("i40e_aq_discover_capabilities returns %x\n", I40eStatus));
        return EFI_DEVICE_ERROR;
      }
    } while (I40eStatus != I40E_SUCCESS);

  return EFI_SUCCESS;
}

/** Checks if reset is done.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on
   @param[in]   ResetMask     Mask to compare with read reg. value if reset was done

   @retval    EFI_SUCCESS       Function ended successfully
   @retval    EFI_DEVICE_ERROR  Timeout when waiting for device to become active
   @retval    EFI_DEVICE_ERROR  Timeout when waiting for reset done
**/
EFI_STATUS
I40eCheckResetDone (
  I40E_DRIVER_DATA *AdapterInfo,
  UINT32            ResetMask
  )
{
  EFI_STATUS       Status;
  UINT32           Reg;
  UINTN            i = 0;
  struct  i40e_hw *Hw;

  Hw = &AdapterInfo->Hw;
  Status = EFI_SUCCESS;


  // First wait until device becomes active.
  while (1) {
    Reg = rd32 (Hw, I40E_GLGEN_RSTAT);
    if ((Reg & I40E_GLGEN_RSTAT_DEVSTATE_MASK) == 0) {
      break;
    }
    DelayInMicroseconds (AdapterInfo, 100);
    if (i++ > I40E_GLNVM_ULD_TIMEOUT) {
      DEBUGPRINT (CRITICAL, ("Device activation error\n"));
      Status = EFI_DEVICE_ERROR;
      break;
    }
  }

  i = 0;

  // Now wait for reset done indication.
  Reg = rd32 (Hw, I40E_GLNVM_ULD);
  while (1) {
    Reg = rd32 (Hw, I40E_GLNVM_ULD);
    if ((Reg & ResetMask) == ResetMask) {
      break;
    }
    DelayInMicroseconds (AdapterInfo, 100);
    if (i++ > I40E_GLNVM_ULD_TIMEOUT) {
      DEBUGPRINT (CRITICAL, ("Timeout waiting for reset done\n"));
      Status = EFI_DEVICE_ERROR;
      break;
    }
  }

  return Status;
}

/** Triggers global reset

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @retval   EFI_SUCCESS   Reset triggered successfully
**/
EFI_STATUS
I40eTriggerGlobalReset (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32           Reg;
  struct i40e_hw  *Hw;

  Hw = &AdapterInfo->Hw;

  Reg = 0x1 << I40E_GLGEN_RTRIG_GLOBR_SHIFT;
  wr32 (Hw, I40E_GLGEN_RTRIG, Reg);


  return EFI_SUCCESS;
}


/** This function checks if any  other instance of driver is loaded on this PF by
   reading PFGEN_DRUN register.

   If not it writes the bit in the register to let know other components that
   the PF is in use.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @retval   TRUE   The PF is free to use for Tx/Rx
   @retval   FALSE  The PF cannot be used for Tx/Rx
**/
BOOLEAN
I40eAquireControllerHw (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32 RegValue;

  RegValue = rd32 (&AdapterInfo->Hw, I40E_PFGEN_DRUN);

  if (RegValue & I40E_PFGEN_DRUN_DRVUNLD_MASK) {

    // bit set means other driver is loaded on this pf
    return FALSE;
  }

  RegValue |= I40E_PFGEN_DRUN_DRVUNLD_MASK;
  wr32 (&AdapterInfo->Hw, I40E_PFGEN_DRUN, RegValue);
  return TRUE;
}

/** Release this PF by clearing the bit in PFGEN_DRUN register.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on

   @return   PFGEN_DRUN driver unload bit is cleared
**/
VOID
I40eReleaseControllerHw (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32 RegValue;

  RegValue = rd32 (&AdapterInfo->Hw, I40E_PFGEN_DRUN);
  RegValue &= ~I40E_PFGEN_DRUN_DRVUNLD_MASK;
  wr32 (&AdapterInfo->Hw, I40E_PFGEN_DRUN, RegValue);
}

/** This function is called as early as possible during driver start to ensure the
   hardware has enough time to autonegotiate when the real SNP device initialize call
   is made.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              the UNDI driver is layering on

   @retval   EFI_SUCCESS            First time init end up successfully
   @retval   EFI_INVALID_PARAMETER  Firmware version is newer than expected
   @retval   EFI_DEVICE_ERROR       Failed to init shared code
   @retval   EFI_DEVICE_ERROR       PF reset failed
   @retval   EFI_DEVICE_ERROR       Init Admin Queue failed
   @retval   EFI_NOT_FOUND          Failed reading MFP configuration
   @retval   EFI_DEVICE_ERROR       Failed reading MFP configuration
   @retval   EFI_INVALID_PARAMETER  Failed to discover (read) capabilities
   @retval   EFI_OUT_OF_RESOURCES   Failed to discover (read) capabilities
   @retval   EFI_DEVICE_ERROR       Failed to discover (read) capabilities
   @retval   EFI_DEVICE_ERROR       Failed to read MAC address
   @retval   EFI_ACCESS_DENIED      UNDI is not enabled
**/
EFI_STATUS
I40eFirstTimeInit (
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  PCI_CONFIG_HEADER     *PciConfigHeader;
  UINT8                  PciBarConfiguration;
  BOOLEAN                Pci64Bit;
  enum  i40e_status_code I40eStatus;
  EFI_STATUS             Status;
  UINT32                 Reg;
  struct  i40e_hw       *Hw;
  UINTN                  AllocationSize = 0;

  Hw = &AdapterInfo->Hw;
  Hw->back = AdapterInfo;

  AdapterInfo->DriverBusy = FALSE;
  AdapterInfo->LastMediaStatus = FALSE;
  AdapterInfo->MediaStatusChecked = FALSE;


  Hw->bus.device = (UINT16) AdapterInfo->Device;
  Hw->bus.func =   (UINT16) AdapterInfo->Function;

  PciConfigHeader     = (PCI_CONFIG_HEADER *) &AdapterInfo->PciConfig[0];
  PciBarConfiguration = PciConfigHeader->BaseAddressReg0 & PCI_BAR_MEM_MASK;
  Pci64Bit            = (PciBarConfiguration & 0x6) == PCI_BAR_MEM_64BIT;

  if (Pci64Bit) {
    // On 64-bit BAR, device address claims two slots in the PCI header.
    Hw->hw_addr = (UINT8 *) (UINTN) ((UINT64) (PciConfigHeader->BaseAddressReg0 & PCI_BAR_MEM_BASE_ADDR_M) +
                                    (((UINT64) PciConfigHeader->BaseAddressReg1) << 32));
    DEBUGPRINT (INIT, ("PCI Base Address Register (64-bit) = %8X:%8X\n",
                       PciConfigHeader->BaseAddressReg1,
                       PciConfigHeader->BaseAddressReg0));
  } else {
    Hw->hw_addr = (UINT8 *) (UINTN) (PciConfigHeader->BaseAddressReg0 & PCI_BAR_MEM_BASE_ADDR_M);
    DEBUGPRINT (INIT, ("PCI Base Address Register (32-bit) = %8X\n", PciConfigHeader->BaseAddressReg0));
  }

  if (Hw->hw_addr == NULL) {
    DEBUGPRINT (CRITICAL, ("NIC Hardware Address is NULL - expect issues!\n"));
    DEBUGPRINT (CRITICAL, ("Basic networking should work, advanced features will fail.\n"));
    DEBUGWAIT (CRITICAL);
  }

  DEBUGPRINT (INIT, ("PCI Command Register = %X\n", PciConfigHeader->Command));
  DEBUGPRINT (INIT, ("PCI Status Register = %X\n", PciConfigHeader->Status));
  DEBUGPRINT (INIT, ("PCI VendorID = %X\n", PciConfigHeader->VendorId));
  DEBUGPRINT (INIT, ("PCI DeviceID = %X\n", PciConfigHeader->DeviceId));
  DEBUGPRINT (INIT, ("PCI SubVendorID = %X\n", PciConfigHeader->SubVendorId));
  DEBUGPRINT (INIT, ("PCI SubSystemID = %X\n", PciConfigHeader->SubSystemId));
  // DEBUGPRINT (INIT, ("PCI Segment = %X\n", AdapterInfo->Segment));
  DEBUGPRINT (INIT, ("PCI Bus = %X\n", AdapterInfo->Bus));
  DEBUGPRINT (INIT, ("PCI Device = %X\n", AdapterInfo->Device));
  DEBUGPRINT (INIT, ("PCI Function = %X\n", AdapterInfo->Function));

  ZeroMem (AdapterInfo->BroadcastNodeAddress, PXE_MAC_LENGTH);
  SetMem (AdapterInfo->BroadcastNodeAddress, PXE_HWADDR_LEN_ETHER, 0xFF);
  ZeroMem (&AdapterInfo->Vsi.CurrentMcastList, sizeof (AdapterInfo->Vsi.CurrentMcastList));

  // Initialize all parameters needed for the shared code
  Hw->vendor_id              = PciConfigHeader->VendorId;
  Hw->device_id              = PciConfigHeader->DeviceId;
  Hw->subsystem_vendor_id    = PciConfigHeader->SubVendorId;
  Hw->subsystem_device_id    = PciConfigHeader->SubSystemId;
  Hw->revision_id            = PciConfigHeader->RevId;

  Hw->adapter_stopped        = TRUE;

  AdapterInfo->PciClass       = PciConfigHeader->ClassIdMain;
  AdapterInfo->PciSubClass    = PciConfigHeader->ClassIdSubclass;
  AdapterInfo->PciClassProgIf = PciConfigHeader->ClassIdProgIf;

  if (Hw->subsystem_device_id == 0) {
    // Read Subsystem ID from PFPCI_SUBSYSID
    Hw->subsystem_device_id = (UINT16) (rd32 (Hw, 0x000BE100) & 0xFFFF);
  }


  // Find out if this function is already used by legacy component
  AdapterInfo->UndiEnabled = I40eAquireControllerHw (AdapterInfo);
  DEBUGPRINT (INIT, ("I40eAquireControllerHw returned %d\n", AdapterInfo->UndiEnabled));

  // Setup AQ initialization parameters: 32 descriptor rings, 4kB buffers
  Hw->aq.num_arq_entries = 32;
  Hw->aq.num_asq_entries = 32;
  Hw->aq.arq_buf_size = 4096;
  Hw->aq.asq_buf_size = 4096;

  I40eStatus = i40e_init_shared_code (Hw);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_init_shared_code returned %d\n", I40eStatus));
    Status = EFI_DEVICE_ERROR;
    goto ErrorReleaseController;
  }

  DEBUGPRINT (INIT, ("Initializing PF %d, PCI Func %d\n", Hw->pf_id, Hw->bus.func));

  AdapterInfo->FwSupported = TRUE;

  if (IsRecoveryMode (AdapterInfo)) {
    // Firmware is in recovery mode. Refrain from further initialization
    // and report error status thru the Driver Health Protocol
    DEBUGPRINT (CRITICAL, ("FW is in recovery mode, skip further initialization\n"));
    AdapterInfo->FwSupported = FALSE;
    return EFI_UNSUPPORTED;
  }

  if (AdapterInfo->UndiEnabled) {
    i40e_clear_hw (Hw);

    if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
      Reg = I40eRead32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB);
      I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, 0);
    }
    I40eStatus = i40e_pf_reset (Hw);

    if (AdapterInfo->Hw.mac.type == I40E_MAC_X722) {
      I40eWrite32 (AdapterInfo, I40E_PRTDCB_TC2PFC_RCB, Reg);
    }

    if (I40eStatus != I40E_SUCCESS) {
      DEBUGPRINT (CRITICAL, ("i40e_pf_reset failed %d\n", I40eStatus));
      Status = EFI_DEVICE_ERROR;
      goto ErrorReleaseController;
    }
  }

  I40eStatus = i40e_init_adminq (Hw);
  if (I40eStatus == I40E_ERR_FIRMWARE_API_VERSION) {

    // Firmware version is newer then expected. Refrain from further initialization
    // and report error status thru the Driver Health Protocol
    AdapterInfo->FwSupported = FALSE;
    return EFI_UNSUPPORTED;
  }
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_init_adminq returned %d\n", I40eStatus));
    Status = EFI_DEVICE_ERROR;
    goto ErrorReleaseController;
  }

  DEBUGPRINT (INIT, ("FW API Info: api_maj_ver: %x, api_min_ver: %x\n", Hw->aq.api_maj_ver, Hw->aq.api_min_ver));

  Status = I40eDiscoverCapabilities (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eDiscoverCapabilities failed: %r\n", Status));
    goto ErrorReleaseController;
  }

  AdapterInfo->TxRxDescriptorCount = I40eGetTxRxDescriptorsCount (Hw);
  DEBUGPRINT (INIT, ("I40eGetTxRxDescriptorsCount: %d\n", AdapterInfo->TxRxDescriptorCount));

  AllocationSize = sizeof (*AdapterInfo->Vsi.TxRing.BufferAddresses) * AdapterInfo->TxRxDescriptorCount;
  AdapterInfo->Vsi.TxRing.BufferAddresses = AllocateZeroPool (AllocationSize);
  if (AdapterInfo->Vsi.TxRing.BufferAddresses == NULL) {
    DEBUGPRINT (CRITICAL, ("AllocateZeroPool failed to allocate TxRing.BufferAddresses!\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorReleaseController;
  }

  AllocationSize = sizeof (*AdapterInfo->Vsi.RxRing.BufferAddresses) * AdapterInfo->TxRxDescriptorCount;
  AdapterInfo->Vsi.RxRing.BufferAddresses = AllocateZeroPool (AllocationSize);
  if (AdapterInfo->Vsi.RxRing.BufferAddresses == NULL) {
    DEBUGPRINT (CRITICAL, ("AllocateZeroPool failed to allocate RxRing.BufferAddresses!\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorReleaseController;
  }

  AllocationSize = sizeof (*AdapterInfo->Vsi.TxRing.TxBufferMappings) * AdapterInfo->TxRxDescriptorCount;
  AdapterInfo->Vsi.TxRing.TxBufferMappings = AllocateZeroPool (AllocationSize);
  if (AdapterInfo->Vsi.TxRing.TxBufferMappings == NULL) {
    DEBUGPRINT (CRITICAL, ("AllocateZeroPool failed to allocate TxRing.TxBufferMappings!\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorReleaseController;
  }

  gBS->Stall (100 * 1000);

  AdapterInfo->QualificationResult = GetModuleQualificationResult (AdapterInfo);

  // Determine existing PF/Port configuration
  // This is to correctly match partitions, pfs and ports
  Status = ReadMfpConfiguration (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("ReadMfpConfiguration returned %r\n", Status));
    goto ErrorReleaseController;
  }

  // Read MAC address.
  Status = I40eReadMacAddress (AdapterInfo);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("I40eReadMacAddress failed with %r\n", Status));
    goto ErrorReleaseController;
  }

  if (AdapterInfo->UndiEnabled) {
    return EFI_SUCCESS;
  } else {
    return EFI_ACCESS_DENIED;
  }

ErrorReleaseController:
  if (AdapterInfo->Vsi.TxRing.BufferAddresses != NULL) {
    gBS->FreePool ((VOID *) AdapterInfo->Vsi.TxRing.BufferAddresses);
  }
  if (AdapterInfo->Vsi.RxRing.BufferAddresses != NULL) {
    gBS->FreePool ((VOID *) AdapterInfo->Vsi.RxRing.BufferAddresses);
  }
  if (AdapterInfo->Vsi.TxRing.TxBufferMappings != NULL) {
    gBS->FreePool ((VOID *) AdapterInfo->Vsi.TxRing.TxBufferMappings);
  }

  I40eReleaseControllerHw (AdapterInfo);
  return Status;
}


/** This function calls the MemIo callback to read a dword from the device's
   address space

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on
   @param[in]   Port         Address to read from

   @return      The data read from the port.
**/
UINT32
I40eRead32 (
  IN I40E_DRIVER_DATA *AdapterInfo,
  IN UINT32            Port
  )
{
  UINT32 Results;

#ifdef CONFIG_ACCESS_TO_CSRS
  {
    UINT32 ConfigSpaceAddress;

    ConfigSpaceAddress = Port | 0x80000000;
    AdapterInfo->PciIo->Pci.Write (
                              AdapterInfo->PciIo,
                              EfiPciIoWidthUint32,
                              IOADDR,
                              1,
                              &ConfigSpaceAddress
                            );
    AdapterInfo->PciIo->Pci.Read (
                              AdapterInfo->PciIo,
                              EfiPciIoWidthUint32,
                              IODATA,
                              1,
                              &Results
                            );
    ConfigSpaceAddress = 0;
    AdapterInfo->PciIo->Pci.Write (
                              AdapterInfo->PciIo,
                              EfiPciIoWidthUint32,
                              IOADDR,
                              1,
                              &ConfigSpaceAddress
                            );
  }
#else /* NOT CONFIG_ACCESS_TO_CSRS */
  MemoryFence ();

  AdapterInfo->PciIo->Mem.Read (
                            AdapterInfo->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            Port,
                            1,
                            (VOID *) (&Results)
                          );
  MemoryFence ();
#endif  /* CONFIG_ACCESS_TO_CSRS */
  return Results;
}

/** This function calls the MemIo callback to write a word from the device's
   address space

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on
   @param[in]   Port         Address to write to
   @param[in]   Data         Data to write to Port

   @return    Data written to address in device's space
**/
VOID
I40eWrite32 (
  IN I40E_DRIVER_DATA *AdapterInfo,
  IN UINT32            Port,
  IN UINT32            Data
  )
{
  UINT32 Value;

  Value = Data;

#ifdef CONFIG_ACCESS_TO_CSRS
  {
    UINT32 ConfigSpaceAddress;

    ConfigSpaceAddress = Port | 0x80000000;
    AdapterInfo->PciIo->Pci.Write (
                              AdapterInfo->PciIo,
                              EfiPciIoWidthUint32,
                              IOADDR,
                              1,
                              &ConfigSpaceAddress
                            );
    AdapterInfo->PciIo->Pci.Write (
                              AdapterInfo->PciIo,
                              EfiPciIoWidthUint32,
                              IODATA,
                              1,
                              &Value
                            );
    ConfigSpaceAddress = 0;
    AdapterInfo->PciIo->Pci.Write (
                              AdapterInfo->PciIo,
                              EfiPciIoWidthUint32,
                              IOADDR,
                              1,
                              &ConfigSpaceAddress
                            );
  }
#else /* NOT CONFIG_ACCESS_TO_CSRS */
  MemoryFence ();

  AdapterInfo->PciIo->Mem.Write (
                            AdapterInfo->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            Port,
                            1,
                            (VOID *) (&Value)
                          );

  MemoryFence ();
#endif /* CONFIG_ACCESS_TO_CSRS */
}

/** Delays execution of next instructions for MicroSeconds microseconds

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                             the UNDI driver is layering on
   @param[in]   MicroSeconds   Time to delay in Microseconds.

   @retval   NONE
**/
VOID
DelayInMicroseconds (
  IN I40E_DRIVER_DATA *AdapterInfo,
  UINT32               MicroSeconds
  )
{
  if (AdapterInfo->Delay != NULL) {
    (*AdapterInfo->Delay) (AdapterInfo->UniqueId, MicroSeconds);
  } else {
    gBS->Stall (MicroSeconds);
  }
}

/** OS specific memory alloc for shared code

   @param[in]   Hw    pointer to the HW structure
   @param[out]  Mem   ptr to mem struct to fill out
   @param[in]   size  size of memory requested

   @retval   I40E_SUCCESS        Memory allocated successfully
   @retval   I40E_ERR_NO_MEMORY  Failed to allocate memory
**/
enum i40e_status_code
I40eAllocateMem (
  struct i40e_hw          *Hw,
  struct i40e_virt_mem    *Mem,
  UINT32                   Size
  )
{
  if (Mem == NULL) {
    return I40E_ERR_PARAM;
  }

  Mem->size = Size;
  Mem->va = AllocateZeroPool (Size);
  if (Mem->va == NULL) {
    DEBUGPRINT (CRITICAL, ("Error: Requested: %d, Allocated size: %d\n", Size, Mem->size));
    return I40E_ERR_NO_MEMORY;
  }

  return I40E_SUCCESS;
}

/** OS specific memory free for shared code

   @param[in]   Hw    pointer to the HW structure
   @param[in]   Mem   ptr to mem struct to free

   @retval   I40E_SUCCESS    There is nothing to free
   @retval   I40E_SUCCESS    Memory successfully freed
   @retval   I40E_ERR_PARAM  Mem is NULL
**/
enum i40e_status_code
I40eFreeMem (
  struct i40e_hw       *Hw,
  struct i40e_virt_mem *Mem
  )
{
  if (Mem == NULL) {
    return I40E_ERR_PARAM;
  }

  if (Mem->va == NULL) {
    // Nothing to free
    return I40E_SUCCESS;
  }

  if (!mExitBootServicesTriggered) {
    gBS->FreePool ((VOID *) Mem->va);
  }

  // Always return I40E_SUCCESS, no need for enhanced error handling here
  return I40E_SUCCESS;
}

/** OS specific spinlock init for shared code

   @param[in]   Sp   pointer to a spinlock declared in driver space

   @return    Spinlock pointed by Sp is initialized
**/
VOID
I40eInitSpinLock (struct i40e_spinlock *Sp)
{
  EfiInitializeLock (&Sp->SpinLock, TPL_NOTIFY);
}

/** OS specific spinlock acquire for shared code

   @param[in]   Sp   pointer to a spinlock declared in driver space

   @return    Spinlock pointed by Sp is acquired
**/
VOID
I40eAcquireSpinLock (struct i40e_spinlock *Sp)
{
  EfiAcquireLockOrFail (&Sp->SpinLock);
}

/** OS specific spinlock release for shared code

   @param[in]   Sp   pointer to a spinlock declared in driver space

   @return    Spinlock pointed by Sp is released
**/
VOID
I40eReleaseSpinLock (struct i40e_spinlock *Sp)
{
  EfiReleaseLock (&Sp->SpinLock);
}

/** OS specific spinlock destroy for shared code

   @param[in]   Sp   pointer to a spinlock declared in driver space

   @return    Nothing is done
**/
VOID
I40eDestroySpinLock (struct i40e_spinlock *Sp)
{
}

/** OS specific DMA memory alloc for shared code

   @param[in]   Hw         pointer to the HW structure
   @param[out]  Mem        ptr to mem struct to fill out
   @param[in]   Size       size of memory requested
   @param[in]   Alignment  byte boundary to which we must align

   @retval   I40E_SUCCESS        Memory allocated successfully
   @retval   I40E_ERR_NO_MEMORY  Failed to allocate memory
**/
enum i40e_status_code
I40eAllocateDmaMem (
  struct i40e_hw      *Hw,
  struct i40e_dma_mem *Mem,
  UINT64               Size,
  UINT32               Alignment
  )
{
  EFI_STATUS        Status;
  I40E_DRIVER_DATA *AdapterInfo = (I40E_DRIVER_DATA *) Hw->back;

  if (Mem == NULL) {
    return I40E_ERR_PARAM;
  }

  Mem->Mapping.Size = (UINT32) ALIGN (Size, Alignment);

  Status = UndiDmaAllocateCommonBuffer (AdapterInfo->PciIo, &Mem->Mapping);

  Mem->va     = (VOID*) Mem->Mapping.UnmappedAddress;
  Mem->pa     = Mem->Mapping.PhysicalAddress;
  Mem->size   = Mem->Mapping.Size;

  if ((Mem->va != NULL)
    && (Status == EFI_SUCCESS))
  {
    return I40E_SUCCESS;
  } else {
    DEBUGPRINT (
      CRITICAL, ("Error: Requested: %d, Allocated size: %d\n",
      Size, Mem->Mapping.Size)
    );
    return I40E_ERR_NO_MEMORY;
  }
}

/** OS specific DMA memory free for shared code

   @param[in]   Hw         pointer to the HW structure
   @param[out]  Mem        ptr to mem struct to free

   @retval  I40E_SUCCESS     Memory successfully freed
   @retval  I40E_ERR_BAD_PTR Failed to free buffer
   @retval  I40E_ERR_PARAM   Mem is NULL
**/
enum i40e_status_code
I40eFreeDmaMem (
  struct i40e_hw *     Hw,
  struct i40e_dma_mem *Mem
  )
{
  EFI_STATUS        Status;
  I40E_DRIVER_DATA *AdapterInfo = (I40E_DRIVER_DATA *) Hw->back;

  if (NULL == Mem) {
    return I40E_ERR_PARAM;
  }

  // Free memory allocated for transmit and receive resources.
  Status = UndiDmaFreeCommonBuffer (AdapterInfo->PciIo, &Mem->Mapping);

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Failed to free I40E DMA buffer: %r\n", Status));
    return I40E_ERR_BAD_PTR;
  }
  return I40E_SUCCESS;
}

/** Checks if Firmware is in recovery mode.

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @retval   TRUE   Firmware is in recovery mode
   @retval   FALSE  Firmware is not in recovery mode
**/
BOOLEAN
IsRecoveryMode (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32  RegVal;

  RegVal = I40eRead32 (AdapterInfo, I40E_GL_FWSTS);
  RegVal &= I40E_GL_FWSTS_FWS1B_MASK;
  RegVal >>= I40E_GL_FWSTS_FWS1B_SHIFT;

  if ((RegVal == I40E_FW_RECOVERY_MODE_CORER)
      || (RegVal == I40E_FW_RECOVERY_MODE_CORER_LEGACY)
      || (RegVal == I40E_FW_RECOVERY_MODE_GLOBR)
      || (RegVal == I40E_FW_RECOVERY_MODE_GLOBR_LEGACY)
      || (RegVal == I40E_FW_RECOVERY_MODE_TRANSITION)
      || (RegVal == I40E_FW_RECOVERY_MODE_NVM))
  {
    return TRUE;
  }

  return FALSE;
}

/** Gets link state (up/down)

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @retval   TRUE   Link is up
   @retval   FALSE  Link is down
   @retval   FALSE  get_link_info AQ cmd failed
**/
BOOLEAN
IsLinkUp (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code I40eStatus;

  I40eStatus = i40e_aq_get_link_info (&AdapterInfo->Hw, TRUE, NULL, NULL);
  if (I40eStatus != I40E_SUCCESS) {
    return FALSE;
  }
  return AdapterInfo->Hw.phy.link_info.link_info & I40E_AQ_LINK_UP;
}

/** Gets link speed capability

   @param[in]    AdapterInfo        Pointer to the NIC data structure information which
                                    the UNDI driver is layering on
   @param[out]   AllowedLinkSpeeds  Pointer to resulting link spedd capability

   @retval    EFI_SUCCESS       Link speed capability setting successfully read
   @retval    EFI_DEVICE_ERROR  Get phy capabilities AQ cmd failed
**/
EFI_STATUS
GetLinkSpeedCapability (
  IN  I40E_DRIVER_DATA *AdapterInfo,
  OUT UINT8            *AllowedLinkSpeeds
  )
{
  enum i40e_status_code                 I40eStatus;
  struct i40e_aq_get_phy_abilities_resp PhyAbilites;

  ZeroMem (&PhyAbilites, sizeof (PhyAbilites));

  // Use AQ command to retrieve phy capabilities
  I40eStatus = i40e_aq_get_phy_capabilities (
                 &AdapterInfo->Hw,
                 FALSE,
                 FALSE,
                 &PhyAbilites,
                 NULL
               );

  if (I40eStatus != I40E_SUCCESS) {
    return EFI_DEVICE_ERROR;
  }

  *AllowedLinkSpeeds = PhyAbilites.link_speed;

  return EFI_SUCCESS;
}

/** Gets link speed

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @retval   LINK_SPEED_UNKNOWN     Link speed is unknown
   @retval   I40E_LINK_SPEED_100MB  Link speed is 100 MB
   @retval   I40E_LINK_SPEED_1GB    Link speed is 1 GB
   @retval   I40E_LINK_SPEED_10GB   Link speed is 10 GB
   @retval   I40E_LINK_SPEED_20GB   Link speed is 20 GB
   @retval   I40E_LINK_SPEED_40GB   Link speed is 40 GB
**/
UINT8
GetLinkSpeed (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum   i40e_aq_link_speed Speed;
  UINT8                     LinkSpeed;

  LinkSpeed = LINK_SPEED_UNKNOWN;

  Speed = i40e_get_link_speed (&AdapterInfo->Hw);

  switch (Speed) {
  case I40E_LINK_SPEED_100MB:
    LinkSpeed = LINK_SPEED_100FULL;
    break;
  case I40E_LINK_SPEED_1GB:
    LinkSpeed = LINK_SPEED_1000FULL;
    break;
  case I40E_LINK_SPEED_2_5GB:
    LinkSpeed = LINK_SPEED_2500;
    break;
  case I40E_LINK_SPEED_5GB:
    LinkSpeed = LINK_SPEED_5000;
    break;
  case I40E_LINK_SPEED_10GB:
    LinkSpeed = LINK_SPEED_10000FULL;
    break;
  case I40E_LINK_SPEED_20GB:
    LinkSpeed = LINK_SPEED_20000;
    break;
  case I40E_LINK_SPEED_25GB:
    LinkSpeed = LINK_SPEED_25000;
    break;
  case I40E_LINK_SPEED_40GB:
    LinkSpeed = LINK_SPEED_40000;
    break;
  default:
    LinkSpeed = LINK_SPEED_UNKNOWN;
    break;
  }

  return LinkSpeed;
}

/** Gets FVL chip type

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @retval   I40E_CHIP_X710      Chip type is X710
   @retval   I40E_CHIP_XL710     Chip type is XL710
   @retval   I40e_CHIP_UNKNOWN   Chip is unknown
**/
I40E_CHIP_TYPE
I40eGetFortvilleChipType (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  switch (AdapterInfo->Hw.device_id) {

  // X710 10 Gig devices
  case I40E_DEV_ID_SFP_XL710:
  case I40E_DEV_ID_KX_C:
  case I40E_DEV_ID_10G_BASE_T_BC:
  case I40E_DEV_ID_10G_B:
  case I40E_DEV_ID_10G_SFP:
  case I40E_DEV_ID_10G_BASE_T:
  case I40E_DEV_ID_10G_BASE_T4:
    return I40E_CHIP_X710;
    break;

  // XL710 40 Gig devices
  case I40E_DEV_ID_25G_SFP28:
  case I40E_DEV_ID_KX_B:
  case I40E_DEV_ID_QSFP_A:
  case I40E_DEV_ID_QSFP_B:
  case I40E_DEV_ID_QSFP_C:
    return I40E_CHIP_XL710;
    break;
  default:
    return I40E_CHIP_UNKNOWN;
    break;
  }
}

/** Gets EEE capability

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on

   @retval   TRUE    EEE capability present
   @retval   FALSE   EEE capability not present
   @retval   FALSE   Get phy capabilities AQ command failed
**/
BOOLEAN
GetEeeCapability (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code                 I40eStatus;
  struct i40e_aq_get_phy_abilities_resp PhyAbilites;

  ZeroMem (&PhyAbilites, sizeof (PhyAbilites));

  // Use AQ command to retrieve phy capabilities
  I40eStatus = i40e_aq_get_phy_capabilities (
                 &AdapterInfo->Hw,
                 FALSE,
                 FALSE,
                 &PhyAbilites,
                 NULL
               );

  if (I40eStatus != I40E_SUCCESS) {
    return FALSE;
  }

  if ((PhyAbilites.eee_capability &
      (I40E_AQ_EEE_100BASE_TX  | I40E_AQ_EEE_1000BASE_T |
       I40E_AQ_EEE_10GBASE_T   | I40E_AQ_EEE_1000BASE_KX |
       I40E_AQ_EEE_10GBASE_KX4 | I40E_AQ_EEE_10GBASE_KR)) != 0)
  {
    return TRUE;
  }
  return FALSE;
}

/** Check if current  module used for this port is qualified module

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              which the UNDI driver is layering on..

   @retval   MODULE_UNSUPPORTED Get link info AQ cmd failed
   @retval   MODULE_SUPPORTED   Link is up - module is qualified
   @retval   MODULE_THERMAL_UNSUPPORTED   Link is down - module is thermal unqualified
   @retval   MODULE_UNSUPPORTED  Link is down - module is unqualified and module
                                 qualification is enabled on the port
   @retval   MODULE_SUPPORTED   Link is down - module is qualified, or qualification
                                process is not available
**/
MODULE_QUALIFICATION_STATUS
GetModuleQualificationResult (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  enum i40e_status_code       I40eStatus;
  UINTN                       i = 0;

  do {

    I40eStatus = i40e_aq_get_link_info (&AdapterInfo->Hw, TRUE, NULL, NULL);

    if (I40eStatus != I40E_SUCCESS) {
      gBS->Stall (50 * 1000);
      i++;
    } else {
      break;
    }
  } while (i < 10);

  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("i40e_aq_get_link_info returned %d\n", I40eStatus));
    return MODULE_UNSUPPORTED;
  }

  if ((AdapterInfo->Hw.phy.link_info.link_info & I40E_AQ_LINK_UP)) {
    DEBUGPRINT (INIT, ("Link is up \n"));
    // Link is up. Module is qualified
    return MODULE_SUPPORTED;
  } else {
    // Need to check for qualified module here
    if ((AdapterInfo->Hw.phy.link_info.link_info & I40E_AQ_MEDIA_AVAILABLE) &&
      (!(AdapterInfo->Hw.phy.link_info.an_info & I40E_AQ_QUALIFIED_MODULE)))
    {
      // Unqualified module was detected
      return MODULE_UNSUPPORTED;
    }
  }
  // Link is down
  // Module is qualified or qualification process is not available
  return MODULE_SUPPORTED;
}

/** Blinks leds on given port

   @param[in]   AdapterInfo  Pointer to the NIC data structure information which
                             the UNDI driver is layering on
   @param[in]   Time         Time in seconds

   @return   LED on port is set blinking for given Time
**/
VOID
BlinkLeds (
  IN  I40E_DRIVER_DATA *AdapterInfo,
  IN UINT32             Time
  )
{
  UINT32 led_status;

  if (AdapterInfo->Hw.device_id == I40E_DEV_ID_10G_BASE_T
    || AdapterInfo->Hw.device_id == I40E_DEV_ID_10G_BASE_T4)
  {
    i40e_blink_phy_link_led (&AdapterInfo->Hw, Time, 500);
  } else {

    // Get current LED state
    led_status = i40e_led_get (&AdapterInfo->Hw);

    // Turn on blinking LEDs and wait required ammount of time
    i40e_led_set (&AdapterInfo->Hw, 0xF, TRUE);
    DelayInMicroseconds (AdapterInfo, Time * 1000 * 1000);

    // Restore LED state
    i40e_led_set (&AdapterInfo->Hw, led_status, FALSE);
  }
}

/** This functions initialize table containing PF numbers for partitions
   related to this port.

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              which the UNDI driver is layering on..

   @retval      EFI_SUCCESS       PF/partition relations saved successfully.
   @retval      EFI_NOT_FOUND     EepromGetMaxPfPerPortNumber failed
   @retval      EFI_DEVICE_ERROR  EepromGetMaxPfPerPortNumber failed
**/
EFI_STATUS
ReadMfpConfiguration (
  IN  I40E_DRIVER_DATA *AdapterInfo
  )
{
  UINT32     i;
  UINT32     PortNumber;
  UINT8      PartitionCount = 0;
  UINT32     PortNum;
  UINT32     FunctionStatus;
  UINT16     PortNumValues[16];
  EFI_STATUS Status;

  Status = EepromGetMaxPfPerPortNumber (
             AdapterInfo,
             &AdapterInfo->PfPerPortMaxNumber
           );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("EepromGetMaxPfPerPortNumber returned %r\n", Status));
    return Status;
  }

  // Read port number for current PF and save its value
  PortNumber = I40eRead32 (
                 AdapterInfo,
                 I40E_PFGEN_PORTNUM
               );
  PortNumber &= I40E_PFGEN_PORTNUM_PORT_NUM_MASK;

  AdapterInfo->PhysicalPortNumber = PortNumber;

  DEBUGPRINT (INIT, ("PhysicalPortNumber: %d\n", PortNumber));

  EepromReadPortnumValues (AdapterInfo, &PortNumValues[0], 16);

  // Run through all pfs and collect information on pfs (partitions) associated
  // to the same port as our base partition
  for (i = 0; i < I40E_MAX_PF_NUMBER; i++) {
    PortNum = PortNumValues[i];

    PortNum &= I40E_PFGEN_PORTNUM_PORT_NUM_MASK;

    if (AdapterInfo->Hw.func_caps.valid_functions & (1 << i)) {
      FunctionStatus = 1;
    } else {
      FunctionStatus = 0;
    }

    if (PortNum == PortNumber) {

      // Partition is connected to the same port as the base partition
      // Save information on the status of this partition
      if (FunctionStatus != 0) {
        AdapterInfo->PartitionEnabled[PartitionCount] = TRUE;
      } else {
        AdapterInfo->PartitionEnabled[PartitionCount] = FALSE;
      }

      // Save PCI function number
      AdapterInfo->PartitionPfNumber[PartitionCount] = i;
      DEBUGPRINT (
        INIT, ("Partition %d, PF:%d, enabled:%d\n",
        PartitionCount,
        AdapterInfo->PartitionPfNumber[PartitionCount],
        AdapterInfo->PartitionEnabled[PartitionCount])
      );

      PartitionCount++;
    }
  }
  return EFI_SUCCESS;
}

/** Read VSI parameters

  @param[in]    AdapterInfo         Pointer to the NIC data structure information
                                    which the UNDI driver is layerin

  @param[out]   VsiCtx             resulting VSI context

  @retval       EFI_SUCCESS         VSI context successfully read
  @retval       EFI_DEVICE_ERROR    VSI context read error
**/
EFI_STATUS
I40eGetVsiParams (
  IN  I40E_DRIVER_DATA        *AdapterInfo,
  OUT struct i40e_vsi_context *VsiCtx
  )
{
  EFI_STATUS I40eStatus;

  ZeroMem (VsiCtx, sizeof (struct i40e_vsi_context));

  VsiCtx->seid = AdapterInfo->MainVsiSeid;
  VsiCtx->pf_num = AdapterInfo->Hw.pf_id;
  VsiCtx->uplink_seid = AdapterInfo->MacSeid;
  VsiCtx->vf_num = 0;

  I40eStatus = i40e_aq_get_vsi_params (&AdapterInfo->Hw, VsiCtx, NULL);
  if (I40eStatus != I40E_SUCCESS) {
    DEBUGPRINT (
      CRITICAL, ("i40e_aq_get_vsi_params returned %d, aq_err %d\n",
      I40eStatus, AdapterInfo->Hw.aq.asq_last_status)
    );
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}




/** Get supported Tx/Rx descriptor count for a given device

   @param[in]    Hw         Pointer to the HW Structure

   @return       Supported Tx/RX descriptors count
**/
UINT16
I40eGetTxRxDescriptorsCount (
  IN struct i40e_hw *Hw
  )
{
  // I40eDiscoverCapabilities must be called prior to this function.

  if (Hw->num_ports != 0 && Hw->num_partitions != 0) {
    return I40E_TOTAL_NUM_TX_RX_DESCRIPTORS / (Hw->num_ports * Hw->num_partitions);
  } else {
    return I40E_DEF_NUM_TX_RX_DESCRIPTORS;
  }
}
