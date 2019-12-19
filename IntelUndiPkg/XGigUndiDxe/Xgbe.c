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
#include "Xgbe.h"
#include "DeviceSupport.h"

/* Global variables for blocking IO */
STATIC BOOLEAN  gInitializeLock = TRUE;
STATIC EFI_LOCK gLock;


/** Display the buffer and descriptors for Transmit/Receive queues.

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information
                            which the UNDI driver is layering on so that we can
                            get the MAC address

   @return   Buffers and descriptors displayed
**/
VOID
_DisplayBuffersAndDescriptors (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  struct ixgbe_legacy_rx_desc *ReceiveDesc;
  struct ixgbe_legacy_tx_desc *TransmitDesc;
  UINT32                       j;
  UINT32 *                     Hi;
  DEBUGPRINT (DIAG, ("IXGBE_RXDCTL=%X\n", IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXDCTL (0))));
  DEBUGPRINT (DIAG, ("RDH0 = %x  ", (UINT16) IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RDH (0))));
  DEBUGPRINT (DIAG, ("RDT0 = %x  ", (UINT16) IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RDT (0))));

  DEBUGPRINT (DIAG, ("Receive Descriptor\n"));
  ReceiveDesc = XGBE_RX_DESC (&XgbeAdapter->RxRing, 0);
  for (j = 0; j < DEFAULT_RX_DESCRIPTORS; j++) {
    Hi = (UINT32 *) &ReceiveDesc->buffer_addr;
    Hi++; // Point to the HI dword of the buffer address
    DEBUGPRINT (DIAG, ("Buff=%x %x,", *Hi, ReceiveDesc->buffer_addr));
    DEBUGPRINT (DIAG, ("Len=%x,", ReceiveDesc->length));
    DEBUGPRINT (DIAG, ("Stat=%x,", ReceiveDesc->status));
    DEBUGPRINT (DIAG, ("errors=%x,", ReceiveDesc->errors));
    DEBUGPRINT (DIAG, ("Csum=%x\n", ReceiveDesc->csum));
    ReceiveDesc++;
  }

  DEBUGWAIT (DIAG);
  DEBUGPRINT (DIAG, ("Transmit Descriptor\n"));
  TransmitDesc = XGBE_TX_DESC (&XgbeAdapter->TxRing, 0);
  for (j = 0; j < DEFAULT_TX_DESCRIPTORS; j++) {
    Hi = (UINT32 *) &TransmitDesc->buffer_addr;
    Hi++; // Point to the HI dword of the buffer address
    DEBUGPRINT (DIAG, ("Buff=%x %x,", *Hi, TransmitDesc->buffer_addr));
    DEBUGPRINT (DIAG, ("Cmd=%x,", TransmitDesc->lower.flags.cmd));
    DEBUGPRINT (DIAG, ("Cso=%x,", TransmitDesc->lower.flags.cso));
    DEBUGPRINT (DIAG, ("Length=%x,", TransmitDesc->lower.flags.length));
    DEBUGPRINT (DIAG, ("Status= %x,", TransmitDesc->upper.fields.status));
    DEBUGPRINT (DIAG, ("Css=%x\n", TransmitDesc->upper.fields.css));
    TransmitDesc++;
  }

  DEBUGWAIT (DIAG);
}

/** Checks if alternate MAC address is supported

   @param[in]   UndiPrivateData    Driver instance private data structure

   @retval   TRUE    Alternate MAC address is supported
   @retval   FALSE   Alternate MAC address is not supported
**/
BOOLEAN
IsAltMacAddrSupported (
  IN UNDI_PRIVATE_DATA *UndiPrivateData
  )
{
  UINT16 BackupMacPointer;

  // Check to see if the backup MAC address location pointer is set
  ixgbe_read_eeprom (&UndiPrivateData->NicInfo.Hw, IXGBE_ALT_MAC_ADDR_PTR, &BackupMacPointer);

  if (BackupMacPointer == 0xFFFF
    || BackupMacPointer == 0x0000)
  {

    //  Alternate Mac Address not supported if 0x37 pointer is not initialized to a value
    //  other than 0x0000 or 0xffff
    return FALSE;
  } else {
    return TRUE;
  }
}




/** Iterate over list of multicast MAC addresses, and gets the current
   MAC address from the first address in the list

   @param[in]   Hw          Shared code hardware structure
   @param[in]   McAddrPtr   Pointer to table of multicast addresses
   @param[in]   Vmdq        VMDQ pointer

   @retval   Pointer to current MAC address
**/
UINT8 *
_XgbeIterateMcastMacAddr (
  IN struct ixgbe_hw *Hw,
  IN UINT8 **         McAddrPtr,
  IN UINT32 *         Vmdq
  )
{
  UINT8 *CurrentMac;

  CurrentMac = *McAddrPtr;
  *McAddrPtr += PXE_MAC_LENGTH;

  DEBUGPRINT (
    XGBE, ("Current MC MAC Addr = %02x %02x %02x %02x %02x %02x",
    CurrentMac[0], CurrentMac[1], CurrentMac[2], CurrentMac[3], CurrentMac[4], CurrentMac[5])
  );
  DEBUGWAIT (XGBE);

  return CurrentMac;
}

/** Implements IO blocking when reading DMA memory.

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information
                             which the UNDI driver is layering on.
   @param[in]   Flag         Block flag

   @return    Lock is acquired or released according to Flag
**/
VOID
XgbeBlockIt (
  IN XGBE_DRIVER_DATA *XgbeAdapter,
  IN UINT32            Flag
  )
{
  if (XgbeAdapter->Block != NULL) {
    (*XgbeAdapter->Block)(XgbeAdapter->UniqueId, Flag);
  } else {
    if (gInitializeLock) {
      EfiInitializeLock (&gLock, TPL_NOTIFY);
      gInitializeLock = FALSE;
    }

    if (Flag != 0) {
      EfiAcquireLock (&gLock);
    } else {
      EfiReleaseLock (&gLock);
    }
  }
}

/** This is the drivers copy function so it does not need to rely on the BootServices
   copy which goes away at runtime.

   This copy function allows 64-bit or 32-bit copies
   depending on platform architecture.  On Itanium we must check that both addresses
   are naturally aligned before attempting a 64-bit copy.

   @param[in]   Dest     Destination memory pointer to copy data to.
   @param[in]   Source   Source memory pointer.
   @param[in]   Count    Number of bytes to copy

   @return    Memory copied from source to destination
**/
VOID
XgbeMemCopy (
  IN UINT8 *Dest,
  IN UINT8 *Source,
  IN UINT32 Count
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

/** Copies the stats from our local storage to the protocol storage.

   It means it will read our read and clear numbers, so some adding is required before
   we copy it over to the protocol.

   @param[in]   XgbeAdapter  Pointer to the NIC data structure information
                             which the UNDI driver is layering on..
   @param[in]   DbAddr   The data Block address
   @param[in]   DbSize   The data Block size

   @retval   PXE_STATCODE_SUCCESS  Statistics copied successfully
**/
UINTN
XgbeStatistics (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT64            DbAddr,
  UINT16            DbSize
  )
{
  PXE_DB_STATISTICS *    DbPtr;
  struct ixgbe_hw *      Hw;
  struct ixgbe_hw_stats *St;
  UINTN                  Stat;

  Hw  = &XgbeAdapter->Hw;
  St  = &XgbeAdapter->Stats;

  UPDATE_OR_RESET_STAT (tpr, IXGBE_TPR);
  UPDATE_OR_RESET_STAT (gprc, IXGBE_GPRC);
  UPDATE_OR_RESET_STAT (ruc, IXGBE_RUC);
  UPDATE_OR_RESET_STAT (ruc, IXGBE_ROC);
  UPDATE_OR_RESET_STAT (rnbc[0], IXGBE_RNBC (0));
  UPDATE_OR_RESET_STAT (bprc, IXGBE_BPRC);
  UPDATE_OR_RESET_STAT (mprc, IXGBE_MPRC);
  UPDATE_OR_RESET_STAT (tpt, IXGBE_TPT);
  UPDATE_OR_RESET_STAT (gptc, IXGBE_GPTC);
  UPDATE_OR_RESET_STAT (bptc, IXGBE_BPTC);
  UPDATE_OR_RESET_STAT (mptc, IXGBE_MPTC);
  UPDATE_OR_RESET_STAT (crcerrs, IXGBE_CRCERRS);

  if (!DbAddr) {
    return PXE_STATCODE_SUCCESS;
  }

  DbPtr = (PXE_DB_STATISTICS *) (UINTN) DbAddr;

  // Fill out the OS statistics structure
  // To Add/Subtract stats, include/delete the lines in pairs.
  // E.g., adding a new stat would entail adding these two lines:
  // stat = PXE_STATISTICS_NEW_STAT_XXX;         SET_SUPPORT;
  //     DbPtr->Data[stat] = st->xxx;
  DbPtr->Supported = 0;

  UPDATE_EFI_STAT (RX_TOTAL_FRAMES, tpr);
  UPDATE_EFI_STAT (RX_GOOD_FRAMES, gprc);
  UPDATE_EFI_STAT (RX_UNDERSIZE_FRAMES, ruc);
  UPDATE_EFI_STAT (RX_OVERSIZE_FRAMES, roc);
  UPDATE_EFI_STAT (RX_DROPPED_FRAMES, rnbc[0]);
  SET_SUPPORT (RX_UNICAST_FRAMES);
  DbPtr->Data[Stat] = (St->gprc - St->bprc - St->mprc);
  UPDATE_EFI_STAT (RX_BROADCAST_FRAMES, bprc);
  UPDATE_EFI_STAT (RX_MULTICAST_FRAMES, mprc);
  UPDATE_EFI_STAT (RX_CRC_ERROR_FRAMES, crcerrs);
  UPDATE_EFI_STAT (TX_TOTAL_FRAMES, tpt);
  UPDATE_EFI_STAT (TX_GOOD_FRAMES, gptc);
  SET_SUPPORT (TX_UNICAST_FRAMES);
  DbPtr->Data[Stat] = (St->gptc - St->bptc - St->mptc);
  UPDATE_EFI_STAT (TX_BROADCAST_FRAMES, bptc);
  UPDATE_EFI_STAT (TX_MULTICAST_FRAMES, mptc);

  return PXE_STATCODE_SUCCESS;
}

/** Takes a command Block pointer (cpb) and sends the frame.  Takes either one fragment or many
   and places them onto the wire.  Cleanup of the send happens in the function UNDI_Status in DECODE.C

   @param[in]   XgbeAdapter   Pointer to the instance data
   @param[in]   Cpb       The command parameter Block address.  64 bits since this is Itanium(tm)
                          processor friendly
   @param[in]   OpFlags   The operation flags, tells if there is any special sauce on this transmit

   @retval   PXE_STATCODE_SUCCESS        If the frame goes out
   @retval   PXE_STATCODE_QUEUE_FULL     Transmit buffers aren't freed by upper layer
   @retval   PXE_STATCODE_DEVICE_FAILURE Frame failed to go out
   @retval   PXE_STATCODE_BUSY           If they need to call again later
**/
UINTN
XgbeTransmit (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT64            Cpb,
  UINT16            OpFlags
  )
{
  PXE_CPB_TRANSMIT_FRAGMENTS * TxFrags;
  PXE_CPB_TRANSMIT *           TxBuffer;
  struct ixgbe_legacy_tx_desc *TransmitDescriptor;
  UINT32                       i;
  INT16                        WaitMsec;
  EFI_STATUS                   Status;
  UNDI_DMA_MAPPING             *TxBufMapping;

  TxBufMapping = &XgbeAdapter->TxBufferMappings[XgbeAdapter->CurTxInd];

  // Transmit buffers must be freed by the upper layer before we can transmit any more.
  if (TxBufMapping->PhysicalAddress != 0) {
    DEBUGPRINT (CRITICAL, ("TX buffers have all been used!\n"));
    return PXE_STATCODE_QUEUE_FULL;
  }

  // Make some short cut pointers so we don't have to worry about typecasting later.
  // If the TX has fragments we will use the
  // tx_tpr_f pointer, otherwise the tx_ptr_l (l is for linear)
  TxBuffer  = (PXE_CPB_TRANSMIT *) (UINTN) Cpb;
  TxFrags   = (PXE_CPB_TRANSMIT_FRAGMENTS *) (UINTN) Cpb;

  // quicker pointer to the next available Tx descriptor to use.
  TransmitDescriptor = XGBE_TX_DESC (&XgbeAdapter->TxRing, XgbeAdapter->CurTxInd);

  // Opflags will tell us if this Tx has fragments
  // So far the linear case (the no fragments case, the else on this if) is the majority
  // of all frames sent.
  if (OpFlags & PXE_OPFLAGS_TRANSMIT_FRAGMENTED) {

    // this count cannot be more than 8;
    DEBUGPRINT (TX, ("Fragments %x\n", TxFrags->FragCnt));

    // for each fragment, give it a descriptor, being sure to keep track of the number used.
    for (i = 0; i < TxFrags->FragCnt; i++) {

      // Put the size of the fragment in the descriptor
      TransmitDescriptor->buffer_addr        = TxFrags->FragDesc[i].FragAddr;
      TransmitDescriptor->lower.flags.length = (UINT16) TxFrags->FragDesc[i].FragLen;
      TransmitDescriptor->lower.data         = (IXGBE_TXD_CMD_IFCS | IXGBE_TXD_CMD_RS);

      XgbeAdapter->TxBufferMappings[XgbeAdapter->CurTxInd].PhysicalAddress  = TxFrags->FragDesc[i].FragAddr;

      // If this is the last fragment we must also set the EOP bit
      if ((i + 1) == TxFrags->FragCnt) {
        TransmitDescriptor->lower.data |= IXGBE_TXD_CMD_EOP;
      }

      // move our software counter passed the frame we just used, watching for wrapping
      DEBUGPRINT (TX, ("Advancing TX pointer %x\n", XgbeAdapter->CurTxInd));
      XgbeAdapter->CurTxInd++;
      if (XgbeAdapter->CurTxInd == DEFAULT_TX_DESCRIPTORS) {
        XgbeAdapter->CurTxInd = 0;
      }

      TransmitDescriptor = XGBE_TX_DESC (&XgbeAdapter->TxRing, XgbeAdapter->CurTxInd);
    }
  } else {
    TxBufMapping->UnmappedAddress = TxBuffer->FrameAddr;
    TxBufMapping->Size = TxBuffer->DataLen + TxBuffer->MediaheaderLen;

    //
    // Make the Tx buffer accessible for adapter over DMA
    //
    Status = UndiDmaMapMemoryRead (
               XgbeAdapter->PciIo,
               TxBufMapping
               );

    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("Failed to map Tx buffer: %r\n", Status));
      DEBUGWAIT (CRITICAL);
      return PXE_STATCODE_DEVICE_FAILURE;
    }

    TransmitDescriptor->buffer_addr = TxBufMapping->PhysicalAddress;
    DEBUGPRINT (TX, ("Packet buffer at %x\n", TransmitDescriptor->buffer_addr));

    // Set the proper bits to tell the chip that this is the last descriptor in the send,
    // and be sure to tell us when its done.
    // EOP - End of packet
    // IFCs - Insert FCS (Ethernet CRC)
    // RS - Report Status
    TransmitDescriptor->lower.data          = (IXGBE_TXD_CMD_EOP |
                                               IXGBE_TXD_CMD_IFCS |
                                               IXGBE_TXD_CMD_RS);
    TransmitDescriptor->upper.fields.status = 0;
    TransmitDescriptor->lower.flags.length  = (UINT16) ((UINT16) TxBuffer->DataLen +
                                                                 TxBuffer->MediaheaderLen);

    DEBUGPRINT (TX, ("BuffAddr=%x, ", TransmitDescriptor->buffer_addr));
    DEBUGPRINT (TX, ("Cmd=%x,", TransmitDescriptor->lower.flags.cmd));
    DEBUGPRINT (TX, ("Cso=%x,", TransmitDescriptor->lower.flags.cso));
    DEBUGPRINT (TX, ("Len=%x,", TransmitDescriptor->lower.flags.length));
    DEBUGPRINT (TX, ("Status=%x,", TransmitDescriptor->upper.fields.status));
    DEBUGPRINT (TX, ("Css=%x\n", TransmitDescriptor->upper.fields.css));

    // Move our software counter passed the frame we just used, watching for wrapping
    XgbeAdapter->CurTxInd++;
    if (XgbeAdapter->CurTxInd == DEFAULT_TX_DESCRIPTORS) {
      XgbeAdapter->CurTxInd = 0;
    }

    DEBUGPRINT (TX, ("Packet length = %d\n", TransmitDescriptor->lower.flags.length));
    DEBUGPRINT (TX, ("Packet data:\n"));
    for (i = 0; i < 32; i++) {
      DEBUGPRINT (TX, ("%x ", ((UINT16 *) ((UINTN) TransmitDescriptor->buffer_addr))[i]));
    }
  }

  // Turn on the blocking function so we don't get swapped out
  // Then move the Tail pointer so the HW knows to start processing the TX we just setup.
  XgbeBlockIt (XgbeAdapter, TRUE);
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_TDT (0), XgbeAdapter->CurTxInd);
  XgbeBlockIt (XgbeAdapter, FALSE);

  // If the opflags tells us to wait for the packet to hit the wire, we will wait.
  if ((OpFlags & PXE_OPFLAGS_TRANSMIT_BLOCK) != 0) {
    WaitMsec = 1000;

    while ((TransmitDescriptor->upper.fields.status & IXGBE_TXD_STAT_DD) == 0) {
      DELAY_IN_MILLISECONDS (10);
      WaitMsec -= 10;
      if (WaitMsec <= 0) {
        break;
      }
    }

    // If we waited for a while, and it didn't finish then the HW must be bad.
    if ((TransmitDescriptor->upper.fields.status & IXGBE_TXD_STAT_DD) == 0) {
      DEBUGPRINT (CRITICAL, ("Device failure\n"));
      return PXE_STATCODE_DEVICE_FAILURE;
    } else {
      DEBUGPRINT (TX, ("Transmit success\n"));
    }
  }

  return PXE_STATCODE_SUCCESS;
}

/** Copies the frame from our internal storage ring (As pointed to by XgbeAdapter->rx_ring)
   to the command Block passed in as part of the cpb parameter.

   The flow:
   Ack the interrupt, setup the pointers, find where the last Block copied is, check to make
   sure we have actually received something, and if we have then we do a lot of work.
   The packet is checked for errors, size is adjusted to remove the CRC, adjust the amount
   to copy if the buffer is smaller than the packet, copy the packet to the EFI buffer,
   and then figure out if the packet was targetted at us, broadcast, multicast
   or if we are all promiscuous.  We then put some of the more interesting information
   (protocol, src and dest from the packet) into the db that is passed to us.
   Finally we clean up the frame, set the return value to _SUCCESS, and inc the cur_rx_ind, watching
   for wrapping.  Then with all the loose ends nicely wrapped up, fade to black and return.

   @param[in]   XgbeAdapter   pointer to the driver data
   @param[in]   CpbReceive    Pointer (Ia-64 friendly) to the command parameter Block.
                              The frame will be placed inside of it.
   @param[out]  DbReceive     The data buffer.  The out of band method of passing pre-digested
                              information to the protocol.

   @retval   PXE_STATCODE_NO_DATA If there is no data
   @retval   PXE_STATCODE_SUCCESS If we passed the goods to the protocol.
**/
UINTN
XgbeReceive (
  XGBE_DRIVER_DATA *XgbeAdapter,
  PXE_CPB_RECEIVE * CpbReceive,
  PXE_DB_RECEIVE *  DbReceive
  )
{
  PXE_FRAME_TYPE               PacketType;
  struct ixgbe_legacy_rx_desc *ReceiveDescriptor;
  ETHER_HEADER *               EtherHeader;
  PXE_STATCODE                 StatCode;
  UINT16                       i;
  UINT16                       TempLen;
  UINT8 *                      PacketPtr;

  PacketType  = PXE_FRAME_TYPE_NONE;
  StatCode    = PXE_STATCODE_NO_DATA;
  i           = 0;

  DEBUGPRINT (RX, ("RCTL=%X\n", IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXDCTL (0))));

  // acknowledge the interrupts
  DEBUGPRINT (RX, ("XgbeReceive  "));
  IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_EICR);
  DEBUGPRINT (RX, ("RDH0 = %x  ", (UINT16) IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RDH (0))));
  DEBUGPRINT (RX, ("RDT0 = %x  ", (UINT16) IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RDT (0))));

  // Get a pointer to the buffer that should have a rx in it, IF one is really there.
  ReceiveDescriptor = XGBE_RX_DESC (&XgbeAdapter->RxRing, XgbeAdapter->CurRxInd);

  if ((ReceiveDescriptor->status & (IXGBE_RXD_STAT_EOP | IXGBE_RXD_STAT_DD)) != 0) {
    DEBUGPRINT (RX, ("XgbeReceive Packet Data at address %0x \n", CpbReceive->BufferAddr));

    // Just to make sure we don't try to copy a zero length, only copy a positive sized packet.
    if ((ReceiveDescriptor->length != 0)
      && (ReceiveDescriptor->errors == 0))
    {

      // If the buffer passed us is smaller than the packet, only copy the size of the buffer.
      TempLen = ReceiveDescriptor->length;
      if (ReceiveDescriptor->length > (INT16) CpbReceive->BufferLen) {
        TempLen = (UINT16) CpbReceive->BufferLen;
      }

      // Copy the packet from our list to the EFI buffer.
      XgbeMemCopy (
        (UINT8 *) (UINTN) CpbReceive->BufferAddr,
        (UINT8 *) (UINTN) ReceiveDescriptor->buffer_addr,
        TempLen
      );

      PacketPtr = (UINT8 *) (UINTN) CpbReceive->BufferAddr;
      DEBUGPRINT (RX, ("XgbeReceive Packet Data at address %0x \n", CpbReceive->BufferAddr));
      for (i = 0; i < 40; i++) {
        DEBUGPRINT (RX, ("%x ", PacketPtr[i]));
      }

      DEBUGPRINT (RX, ("\n"));
      DEBUGWAIT (RX);

      // Fill the DB with needed information
      DbReceive->FrameLen       = ReceiveDescriptor->length;  // includes header
      DbReceive->MediaHeaderLen = PXE_MAC_HEADER_LEN_ETHER;

      EtherHeader               = (ETHER_HEADER *) (UINTN) ReceiveDescriptor->buffer_addr;

      // Figure out if the packet was meant for us, was a broadcast, multicast or we
      // recieved a frame in promiscuous mode.
      for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
        if (EtherHeader->DestAddr[i] != XgbeAdapter->Hw.mac.perm_addr[i]) {
          DEBUGPRINT (RX, ("Packet is not specifically for us\n"));
          break;
        }
      }

      // if we went the whole length of the header without breaking out then the packet is
      // directed at us.
      if (i >= PXE_HWADDR_LEN_ETHER) {
        DEBUGPRINT (RX, ("Packet is for us\n"));
        PacketType = PXE_FRAME_TYPE_UNICAST;
      } else {

        // Compare it against our broadcast node address
        for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
          if (EtherHeader->DestAddr[i] != XgbeAdapter->BroadcastNodeAddress[i]) {
            DEBUGPRINT (RX, ("Packet is not our broadcast\n"));
            break;
          }
        }

        // If we went the whole length of the header without breaking out
        // then the packet is directed at us via broadcast
        if (i >= PXE_HWADDR_LEN_ETHER) {
          PacketType = PXE_FRAME_TYPE_BROADCAST;
        } else {

          // That leaves multicast or we must be in promiscuous mode.
          // Check for the Mcast bit in the address.
          // otherwise its a promiscuous receive.
          if ((EtherHeader->DestAddr[0] & 1) == 1) {
            PacketType = PXE_FRAME_TYPE_MULTICAST;
          } else {
            PacketType = PXE_FRAME_TYPE_PROMISCUOUS;
          }
        }
      }

      DbReceive->Type = PacketType;
      DEBUGPRINT (RX, ("PacketType %x\n", PacketType));

      // Put the protocol (UDP, TCP/IP) in the data buffer.
      DbReceive->Protocol = EtherHeader->Type;
      DEBUGPRINT (RX, ("protocol %x\n", EtherHeader->Type));

      DEBUGPRINT (RX, ("Dest Address: "));
      for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
        DEBUGPRINT (RX, ("%x", (UINT32) EtherHeader->DestAddr[i]));
        DbReceive->SrcAddr[i]   = EtherHeader->SrcAddr[i];
        DbReceive->DestAddr[i]  = EtherHeader->DestAddr[i];
      }

      DEBUGPRINT (RX, ("\n"));
      StatCode = PXE_STATCODE_SUCCESS;
    } else {
      DEBUGPRINT (
        RX, ("ERROR: ReceiveDescriptor->length=%x, ReceiveDescriptor->errors=%x \n",
        ReceiveDescriptor->length, ReceiveDescriptor->errors)
      );

      // Go through all the error bits - these are only valid when EOP and DD are set
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_CE) {
        DEBUGPRINT (CRITICAL, ("CE Error\n"));
      }
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_LE) {
        DEBUGPRINT (CRITICAL, ("LE Error\n"));
      }
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_PE) {
        DEBUGPRINT (CRITICAL, ("PE Error\n"));
      }
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_OSE) {
        DEBUGPRINT (CRITICAL, ("OSE Error\n"));
      }
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_USE) {
        DEBUGPRINT (CRITICAL, ("USE Error\n"));
      }
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_TCPE) {
        DEBUGPRINT (CRITICAL, ("TCP Error\n"));
      }
      if (ReceiveDescriptor->errors & IXGBE_RXD_ERR_IPE) {
        DEBUGPRINT (CRITICAL, ("IP Error\n"));
      }
    }

    // Clean up the packet
    ReceiveDescriptor->status = 0;
    ReceiveDescriptor->length = 0;
    ReceiveDescriptor->errors = 0;

    // Move the current cleaned buffer pointer, being careful to wrap it as needed.
    // Then update the hardware, so it knows that an additional buffer can be used.
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDT (0), XgbeAdapter->CurRxInd);
    XgbeAdapter->CurRxInd++;
    if (XgbeAdapter->CurRxInd == DEFAULT_RX_DESCRIPTORS) {
      XgbeAdapter->CurRxInd = 0;
    }
  }

  return StatCode;
}

/** Allows the protocol to control our interrupt behaviour.

   @param[in]   XgbeAdapter   Pointer to the driver structure

   @retval   PXE_STATCODE_SUCCESS   Interrupt state set successfully
**/
UINTN
XgbeSetInterruptState (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  UINT32 SetIntMask;

  SetIntMask = 0;

  DEBUGPRINT (XGBE, ("XgbeSetInterruptState\n"));

  // Mask the RX interrupts
  if ((XgbeAdapter->IntMask & PXE_OPFLAGS_INTERRUPT_RECEIVE) != 0) {
    SetIntMask |= IXGBE_EICR_RTX_QUEUE_0_MASK;
    DEBUGPRINT (XGBE, ("Mask the RX interrupts\n"));
  }

  // Mask the TX interrupts
  if ((XgbeAdapter->IntMask & PXE_OPFLAGS_INTERRUPT_TRANSMIT) != 0) {
    SetIntMask |= IXGBE_EICR_RTX_QUEUE_1_MASK;
    DEBUGPRINT (XGBE, ("Mask the TX interrupts\n"));
  }

  // Now we have all the Ints we want, so let the hardware know.
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_EIMS, SetIntMask);

  return PXE_STATCODE_SUCCESS;
}

/** Stop the hardware and put it all (including the PHY) into a known good state.

   @param[in]   XgbeAdapter   Pointer to the driver structure

   @retval   PXE_STATCODE_SUCCESS    Hardware stopped
**/
UINTN
XgbeShutdown (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  DEBUGPRINT (XGBE, ("XgbeShutdown - adapter stop\n"));

  XgbeReceiveStop (XgbeAdapter);

  XgbeClearRegBits (XgbeAdapter, IXGBE_TXDCTL (0), IXGBE_TXDCTL_ENABLE);

  XgbeAdapter->RxFilter = 0;

  XgbeDisableInterrupts (XgbeAdapter);

  return PXE_STATCODE_SUCCESS;
}

/** Resets the hardware and put it all (including the PHY) into a known good state.

   @param[in]   XgbeAdapter   The pointer to our context data
   @param[in]   OpFlags       The information on what else we need to do.

   @retval   PXE_STATCODE_SUCCESS        Successful hardware reset
   @retval   PXE_STATCODE_NOT_STARTED    Hardware init failed
**/
UINTN
XgbeReset (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT16            OpFlags
  )
{
  EFI_STATUS Status;

  // Put the XGBE into a known state by resetting the transmit
  // and receive units of the XGBE and masking/clearing all
  // interrupts.
  // If the hardware has already been started then don't bother with a reset.
  if (!XgbeAdapter->HwInitialized) {

    // Now that the structures are in place, we can configure the hardware to use it all.
    Status = XgbeInitHw (XgbeAdapter);
    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("XgbeInitHw returns %r\n", Status));
      return PXE_STATCODE_NOT_STARTED;
    }
  } else {
    DEBUGPRINT (XGBE, ("Skipping adapter reset\n"));
  }

  if ((OpFlags & PXE_OPFLAGS_RESET_DISABLE_FILTERS) == 0) {
    UINT16 SaveFilter;

    SaveFilter = XgbeAdapter->RxFilter;

    // if we give the filter same as RxFilter, this routine will not set mcast list
    // (it thinks there is no change)
    // to force it, we will reset that flag in the RxFilter
    XgbeAdapter->RxFilter &= (~PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST);
    XgbeSetFilter (XgbeAdapter, SaveFilter);
  }

  if (OpFlags & PXE_OPFLAGS_RESET_DISABLE_INTERRUPTS) {
    XgbeAdapter->IntMask = 0; // disable the interrupts
  }

  XgbeSetInterruptState (XgbeAdapter);

  return PXE_STATCODE_SUCCESS;
}

/** Configures internal interrupt causes on current PF.

   @param[in]   XgbeAdapter   The pointer to our context data

   @return   Interrupt causes are configured for current PF
**/
VOID
XgbeConfigureInterrupts (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  UINT32  RegVal = 0;

  // Map Rx queue 0 interrupt to EICR bit0 and Tx queue 0interrupt to EICR bit1
  switch (XgbeAdapter->Hw.mac.type) {
  case ixgbe_mac_82598EB:
    RegVal = IXGBE_IVAR_ALLOC_VAL;
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_IVAR (0), RegVal);
    RegVal = IXGBE_IVAR_ALLOC_VAL | 0x01;
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_IVAR (16), RegVal);
    break;
  case ixgbe_mac_82599EB:
  case ixgbe_mac_X540:
  case ixgbe_mac_X550:
  case ixgbe_mac_X550EM_x:
  case ixgbe_mac_X550EM_a:
    RegVal = ((IXGBE_IVAR_ALLOC_VAL | 0x01) << 8) | IXGBE_IVAR_ALLOC_VAL;
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_IVAR (0), RegVal);
    break;
  default:
    break;
  }

}

/** Disables internal interrupt causes on current PF.

   @param[in]   XgbeAdapter   The pointer to our context data

   @return   Interrupt causes are disabled for current PF
**/
VOID
XgbeDisableInterrupts (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{

  // Deconfigure Interrupt Vector Allocation Register
  switch (XgbeAdapter->Hw.mac.type) {
  case ixgbe_mac_82598EB:
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_IVAR (0), 0);
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_IVAR (16), 0);
    break;
  case ixgbe_mac_82599EB:
  case ixgbe_mac_X540:
  case ixgbe_mac_X550:
  case ixgbe_mac_X550EM_x:
  case ixgbe_mac_X550EM_a:
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_IVAR (0), 0);
    break;
  default:
    break;
  }
}

/** PCIe function to LAN port mapping.

   @param[in,out]   XgbeAdapter   Pointer to adapter structure

   @return   LAN function set accordingly
**/
VOID
XgbeLanFunction (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  XgbeAdapter->LanFunction = (IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_STATUS) & IXGBE_STATUS_LAN_ID)
                             >> IXGBE_STATUS_LAN_ID_SHIFT;
  DEBUGPRINT (INIT, ("PCI function %d is LAN port %d \n", XgbeAdapter->Function, XgbeAdapter->LanFunction));
  DEBUGWAIT (INIT);
}

/** This function performs PCI-E initialization for the device.

   @param[in]   XgbeAdapter   Pointer to adapter structure

   @retval   EFI_SUCCESS            PCI-E initialized successfully
   @retval   EFI_UNSUPPORTED        Failed to get supported PCI command options
   @retval   EFI_UNSUPPORTED        Failed to set PCI command options
   @retval   EFI_OUT_OF_RESOURCES   The memory pages for transmit and receive resources could
                                    not be allocated
**/
EFI_STATUS
XgbePciInit (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  EFI_STATUS Status;
  UINTN      Seg;
  UINT64     Result;
  BOOLEAN    PciAttributesSaved = FALSE;

  Result = 0;

  // Save original PCI attributes
  Status = XgbeAdapter->PciIo->Attributes (
                                 XgbeAdapter->PciIo,
                                 EfiPciIoAttributeOperationGet,
                                 0,
                                 &XgbeAdapter->OriginalPciAttributes
                               );

  if (EFI_ERROR (Status)) {
    goto PciIoError;
  }
  PciAttributesSaved = TRUE;

  // Get the PCI Command options that are supported by this controller.
  Status = XgbeAdapter->PciIo->Attributes (
                                 XgbeAdapter->PciIo,
                                 EfiPciIoAttributeOperationSupported,
                                 0,
                                 &Result
                               );

  DEBUGPRINT (XGBE, ("Attributes supported %x\n", Result));

  if (!EFI_ERROR (Status)) {

    // Set the PCI Command options to enable device memory mapped IO,
    // port IO, and bus mastering.
    Status = XgbeAdapter->PciIo->Attributes (
                                   XgbeAdapter->PciIo,
                                   EfiPciIoAttributeOperationEnable,
                                   Result & (EFI_PCI_DEVICE_ENABLE |
                                             EFI_PCI_IO_ATTRIBUTE_DUAL_ADDRESS_CYCLE),
                                   NULL
                                 );
  }

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("Attributes returns %r\n", Status));
    goto PciIoError;
  }

  XgbeAdapter->PciIo->GetLocation (
                        XgbeAdapter->PciIo,
                        &Seg,
                        &XgbeAdapter->Bus,
                        &XgbeAdapter->Device,
                        &XgbeAdapter->Function
                      );

  // Read all the registers from the device's PCI Configuration space
  XgbeAdapter->PciIo->Pci.Read (
                            XgbeAdapter->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            MAX_PCI_CONFIG_LEN,
                            XgbeAdapter->PciConfig
                          );

  //
  // Allocate common DMA buffer for Tx descriptors
  //
  XgbeAdapter->TxRing.Size = TX_RING_SIZE;

  Status = UndiDmaAllocateCommonBuffer (
             XgbeAdapter->PciIo,
             &XgbeAdapter->TxRing
             );

  if (EFI_ERROR (Status)) {
    goto OnAllocError;
  }

  //
  // Allocate common DMA buffer for Rx descriptors
  //
  XgbeAdapter->RxRing.Size = RX_RING_SIZE;

  Status = UndiDmaAllocateCommonBuffer (
             XgbeAdapter->PciIo,
             &XgbeAdapter->RxRing
             );

  if (EFI_ERROR (Status)) {
    goto OnAllocError;
  }

  //
  // Allocate common DMA buffer for Rx buffers
  //
  XgbeAdapter->RxBufferMapping.Size = RX_BUFFERS_SIZE;

  Status = UndiDmaAllocateCommonBuffer (
             XgbeAdapter->PciIo,
             &XgbeAdapter->RxBufferMapping
             );

  if (EFI_ERROR (Status)) {
    goto OnAllocError;
  }

  return EFI_SUCCESS;

OnAllocError:
  if (XgbeAdapter->TxRing.Mapping != NULL) {
    UndiDmaFreeCommonBuffer (XgbeAdapter->PciIo, &XgbeAdapter->TxRing);
  }

  if (XgbeAdapter->RxRing.Mapping != NULL) {
    UndiDmaFreeCommonBuffer (XgbeAdapter->PciIo, &XgbeAdapter->RxRing);
  }

  if (XgbeAdapter->RxBufferMapping.Mapping != NULL) {
    UndiDmaFreeCommonBuffer (XgbeAdapter->PciIo, &XgbeAdapter->RxBufferMapping);
  }

PciIoError:
  if (PciAttributesSaved) {

      // Restore original PCI attributes
    XgbeAdapter->PciIo->Attributes (
                          XgbeAdapter->PciIo,
                          EfiPciIoAttributeOperationSet,
                          XgbeAdapter->OriginalPciAttributes,
                          NULL
                        );
  }

  return Status;
}

/** This function is called as early as possible during driver start to ensure the
   hardware has enough time to autonegotiate when the real SNP device initialize call is made.

   @param[in]   XgbePrivate   Pointer to driver private data

   @retval   EFI_SUCCESS        Hardware init success
   @retval   EFI_UNSUPPORTED    Shared code initialization failed
   @retval   EFI_UNSUPPORTED    Could not read MAC address
   @retval   EFI_ACCESS_DENIED  iSCSI Boot detected on port
   @retval   EFI_DEVICE_ERROR   Hardware init failed
**/
EFI_STATUS
XgbeFirstTimeInit (
  UNDI_PRIVATE_DATA *XgbePrivate
  )
{
  PCI_CONFIG_HEADER *PciConfigHeader;
  EFI_STATUS         Status;
  XGBE_DRIVER_DATA * XgbeAdapter;
  INT32              ScStatus;
  UINT32             Reg;
  UINT16             i;

  DEBUGPRINT (CRITICAL, ("XgbeFirstTimeInit\n"));

  XgbeAdapter             = &XgbePrivate->NicInfo;

  XgbeAdapter->DriverBusy = FALSE;
  XgbeAdapter->ReceiveStarted = FALSE;
  PciConfigHeader         = (PCI_CONFIG_HEADER *) &XgbeAdapter->PciConfig[0];

  ZeroMem (XgbeAdapter->BroadcastNodeAddress, PXE_MAC_LENGTH);
  SetMem (XgbeAdapter->BroadcastNodeAddress, PXE_HWADDR_LEN_ETHER, 0xFF);

  DEBUGPRINT (XGBE, ("PciConfigHeader->VendorId = %X\n", PciConfigHeader->VendorId));
  DEBUGPRINT (XGBE, ("PciConfigHeader->DeviceId = %X\n", PciConfigHeader->DeviceId));


  // Initialize all parameters needed for the shared code
  XgbeAdapter->Hw.hw_addr                       = 0;
  XgbeAdapter->Hw.back                          = XgbeAdapter;
  XgbeAdapter->Hw.vendor_id                     = PciConfigHeader->VendorId;
  XgbeAdapter->Hw.device_id                     = PciConfigHeader->DeviceId;
  XgbeAdapter->Hw.revision_id                   = (UINT8) PciConfigHeader->RevId;
  XgbeAdapter->Hw.subsystem_vendor_id           = PciConfigHeader->SubVendorId;
  XgbeAdapter->Hw.subsystem_device_id           = PciConfigHeader->SubSystemId;
  XgbeAdapter->Hw.revision_id                   = (UINT8) PciConfigHeader->RevId;
  XgbeAdapter->Hw.adapter_stopped               = TRUE;
  XgbeAdapter->Hw.fc.requested_mode             = ixgbe_fc_default;

  XgbeAdapter->PciClass    = (UINT8) ((PciConfigHeader->ClassId & PCI_CLASS_MASK) >> 8);
  XgbeAdapter->PciSubClass = (UINT8) (PciConfigHeader->ClassId) & PCI_SUBCLASS_MASK;

  ScStatus = ixgbe_init_shared_code (&XgbeAdapter->Hw);
  if (ScStatus != IXGBE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("Error initializing shared code! %d\n", -ScStatus));

    // This is the only condition where we will fail.  We need to support SFP hotswap
    // which may produce an error if the SFP module is missing.
    if (ScStatus == IXGBE_ERR_DEVICE_NOT_SUPPORTED ||
      ScStatus == IXGBE_ERR_SFP_NOT_SUPPORTED)
    {
      return EFI_UNSUPPORTED;
    }
  }

  XgbeLanFunction (XgbeAdapter);

  DEBUGPRINT (XGBE, ("Calling ixgbe_get_mac_addr\n"));
  if (ixgbe_get_mac_addr (&XgbeAdapter->Hw, XgbeAdapter->Hw.mac.perm_addr) != IXGBE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("Could not read MAC address\n"));
    return EFI_UNSUPPORTED;
  }

  // Copy perm_addr to addr. Needed in HII. Print it if requested.
  DEBUGPRINT (INIT, ("MAC Address: "));
  for (i = 0; i < 6; i++) {
    XgbeAdapter->Hw.mac.addr[i] = XgbeAdapter->Hw.mac.perm_addr[i];
    DEBUGPRINT (INIT, ("%2x ", XgbeAdapter->Hw.mac.perm_addr[i]));
  }
  DEBUGPRINT (INIT, ("\n"));

  if (ixgbe_fw_recovery_mode (&XgbeAdapter->Hw)) {
    // Firmware is in recovery mode - we CANNOT touch the NIC a lot from now on.
    // Report this via the Driver Health Protocol.

    DEBUGPRINT (CRITICAL, ("NIC firmware is in Recovery Mode, skip further initialization.\n"));
    XgbeAdapter->FwSupported = FALSE;
    return EFI_UNSUPPORTED;
  } else {
    XgbeAdapter->FwSupported = TRUE;
  }

  Reg = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_CTRL_EXT);
  if ((Reg & IXGBE_CTRL_EXT_DRV_LOAD) != 0) {
    DEBUGPRINT (CRITICAL, ("XgbeFirstTimeInit: iSCSI Boot detected on port!\n"));
    return EFI_ACCESS_DENIED;
  }

  // Clear the Wake-up status register in case there has been a power management event
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_WUS, 0);

  Status = XgbeInitHw (XgbeAdapter);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("XgbeInitHw returns %r\n", Status));
    return Status;
  }

  XgbeSetRegBits (XgbeAdapter, IXGBE_CTRL_EXT, IXGBE_CTRL_EXT_DRV_LOAD);

  return EFI_SUCCESS;
}


/** Initializes the hardware and sets up link.

   @param[in]   XgbeAdapter   Pointer to adapter structure

   @retval   EFI_SUCCESS        Hardware initialized, link set up
   @retval   EFI_DEVICE_ERROR   Failed to initialize hardware
   @retval   EFI_DEVICE_ERROR   Failed to set up link
   @retval   EFI_DEVICE_ERROR   Failed to set the PCI bus info in ixgbe_hw structure
**/
EFI_STATUS
XgbeInitHw (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  INT32            ScStatus;
  ixgbe_link_speed Speed;

  // Now that the structures are in place, we can configure the hardware to use it all.

  ScStatus = ixgbe_init_hw (&XgbeAdapter->Hw);
  if (ScStatus == 0) {
    DEBUGPRINT (XGBE, ("ixgbe_init_hw success\n"));
    XgbeAdapter->HwInitialized = TRUE;
  } else if (ScStatus == IXGBE_ERR_SFP_NOT_PRESENT) {
    DEBUGPRINT (CRITICAL, ("ixgbe_init_hw returns IXGBE_ERR_SFP_NOT_PRESENT\n"));
    XgbeAdapter->HwInitialized = TRUE;
  } else {
    DEBUGPRINT (CRITICAL, ("Hardware Init failed = %d\n", -ScStatus));
    XgbeAdapter->HwInitialized = FALSE;
    return EFI_DEVICE_ERROR;
  }
  DEBUGWAIT (XGBE);

  ScStatus = ixgbe_set_phy_power (&XgbeAdapter->Hw, TRUE);
  if (ScStatus != IXGBE_SUCCESS
    && ScStatus != IXGBE_NOT_IMPLEMENTED)
  {
    DEBUGPRINT (CRITICAL, ("ixgbe_set_phy_power failed with Status %X\n", ScStatus));
  }

  // Set the PCI bus info in ixgbe_hw structure
  ScStatus = ixgbe_get_bus_info (&XgbeAdapter->Hw);
  if (ScStatus != IXGBE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("ixgbe_get_bus_info fails\n"));
    return EFI_DEVICE_ERROR;
  }

  // 82599+ silicons support 100Mb autonegotiation which is not supported
  // with 82598. This is why we initialize Speed parameter in different way.
  if (XgbeAdapter->Hw.mac.type == ixgbe_mac_82598EB) {
    Speed = IXGBE_LINK_SPEED_82598_AUTONEG;
  } else if (XgbeAdapter->Hw.device_id == IXGBE_DEV_ID_X550EM_X_KX4) {
    Speed = IXGBE_LINK_SPEED_1GB_FULL;
  } else if ((XgbeAdapter->Hw.device_id == IXGBE_DEV_ID_X550EM_A_KR_L ||
    XgbeAdapter->Hw.device_id == IXGBE_DEV_ID_X550EM_A_KR) &&
    XgbeAdapter->Hw.phy.nw_mng_if_sel & IXGBE_NW_MNG_IF_SEL_PHY_SPEED_2_5G) {
    Speed = IXGBE_LINK_SPEED_2_5GB_FULL;
  } else if (XgbeAdapter->Hw.device_id == IXGBE_DEV_ID_X550EM_A_SGMII ||
             XgbeAdapter->Hw.device_id == IXGBE_DEV_ID_X550EM_A_SGMII_L) {
    Speed = IXGBE_LINK_SPEED_1GB_FULL |
            IXGBE_LINK_SPEED_100_FULL |
            IXGBE_LINK_SPEED_10_FULL;
  } else {
    Speed = IXGBE_LINK_SPEED_82599_AUTONEG;
  }

  XgbeAdapter->QualificationResult = GetModuleQualificationResult (XgbeAdapter);

  ScStatus = ixgbe_setup_link (&XgbeAdapter->Hw, Speed, FALSE);
  if (ScStatus != IXGBE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("ixgbe_setup_link fails\n"));
    return EFI_DEVICE_ERROR;
  }

  // Enable interrupt causes.
  XgbeConfigureInterrupts (XgbeAdapter);

  return EFI_SUCCESS;
}

/** Initializes the transmit and receive resources for the adapter.

   @param[in]   XgbeAdapter   Pointer to adapter structure

   @return   TX/RX resources configured and initialized
**/
VOID
XgbeTxRxConfigure (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  UINT32  TempReg;
  UINT64  MemAddr;
  UINT32 *MemPtr;
  UINT16  i;
  struct ixgbe_legacy_rx_desc   *RxDesc;
  LOCAL_RX_BUFFER *             RxBuffer;

  DEBUGPRINT (XGBE, ("XgbeTxRxConfigure\n"));

  XgbeReceiveStop (XgbeAdapter);

  DEBUGPRINT (
    XGBE, ("Rx Ring %x Tx Ring %X  RX size %X \n",
    XGBE_RX_DESC (&XgbeAdapter->RxRing, 0),
    XGBE_TX_DESC (&XgbeAdapter->TxRing, 0),
    (sizeof (struct ixgbe_legacy_rx_desc) * DEFAULT_RX_DESCRIPTORS))
  );

  ZeroMem (XgbeAdapter->TxBufferMappings, sizeof (XgbeAdapter->TxBufferMappings));

  RxBuffer = (LOCAL_RX_BUFFER *) (UINTN) XgbeAdapter->RxBufferMapping.PhysicalAddress;

  DEBUGPRINT (
    XGBE, ("Local Rx Buffer %X size %X\n",
    RxBuffer,
    (sizeof (struct ixgbe_legacy_tx_desc) * DEFAULT_TX_DESCRIPTORS))
  );

  //
  // Now to link the RX Ring to the local buffers
  // Write physical addresses of Rx buffers to Rx descriptors
  //
  for (i = 0; i < DEFAULT_RX_DESCRIPTORS; i++) {
    RxDesc = XGBE_RX_DESC (&XgbeAdapter->RxRing, i);
    RxDesc->buffer_addr = (UINT64) ((UINTN) RxBuffer[i].RxBuffer);
    DEBUGPRINT (XGBE, ("Rx Local Buffer %X\n", RxDesc->buffer_addr));
  }

  _DisplayBuffersAndDescriptors (XgbeAdapter);

  //
  // Setup the RDBA, RDLEN
  // Write physical address of Rx descriptor buffer for HW to use
  //
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDBAL (0), (UINT32) (UINTN) (XgbeAdapter->RxRing.PhysicalAddress));

  MemAddr = (UINT64) (UINTN) XgbeAdapter->RxRing.PhysicalAddress;
  MemPtr  = (UINT32 *) &MemAddr;
  MemPtr++;
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDBAH (0), *MemPtr);
  DEBUGPRINT (XGBE, ("Rdbal0 %X\n", (UINT32) IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RDBAL (0))));
  DEBUGPRINT (XGBE, ("RdBah0 %X\n", (UINT32) IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RDBAH (0))));
  DEBUGPRINT (XGBE, ("Rx Ring %X\n", XgbeAdapter->RxRing.PhysicalAddress));

  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDH (0), 0);
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_TDH (0), 0);
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_TDT (0), 0);

  // We must wait for the receive unit to be enabled before we move
  // the tail descriptor or the hardware gets confused.
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDT (0), 0);

  IXGBE_WRITE_REG (
    &XgbeAdapter->Hw,
    IXGBE_RDLEN (0),
    (sizeof (struct ixgbe_legacy_rx_desc) * DEFAULT_RX_DESCRIPTORS)
  );

  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {

    // Init receive buffer size (BSIZEPACKET field) to 2kB
    // Setup descriptor type to legacy (bits 27:25 to 0)
    TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_SRRCTL (0));
    TempReg &= ~IXGBE_SRRCTL_BSIZEPKT_MASK;
    TempReg |= 0x2;
    TempReg &= ~IXGBE_SRRCTL_DESCTYPE_MASK;
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_SRRCTL (0), TempReg);

    if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
      || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
      || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
      || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
      )
    {
      XgbeAdapter->Hw.fc.pause_time = 1;
      XgbeAdapter->Hw.fc.requested_mode = ixgbe_fc_none;
      XgbeAdapter->Hw.fc.disable_fc_autoneg = TRUE;
      ixgbe_fc_enable (&XgbeAdapter->Hw);
    }

    XgbeSetRegBits (XgbeAdapter, IXGBE_RXDCTL (0), IXGBE_RXDCTL_ENABLE);
    i = 0;
    do {
      TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXDCTL (0));
      i++;
      if ((TempReg & IXGBE_RXDCTL_ENABLE) != 0) {
        DEBUGPRINT (XGBE, ("RX queue enabled, after attempt i = %d\n", i));
        break;
      }

      DelayInMicroseconds (XgbeAdapter, 1);
    } while (i < 1000);

    if (i >= 1000) {
      DEBUGPRINT (CRITICAL, ("Enable RX queue failed!\n"));
    }
  }

  // Point the current working rx buffer to the top of the list
  XgbeAdapter->CurRxInd     = 0;
  XgbeAdapter->CurTxInd     = 0;  // currently usable buffer
  XgbeAdapter->XmitDoneHead = 0;  // the last cleaned buffer
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_TDBAL (0), (UINT32) (XgbeAdapter->TxRing.PhysicalAddress));
  MemAddr = (UINT64) XgbeAdapter->TxRing.PhysicalAddress;
  MemPtr  = (UINT32 *) &MemAddr;
  MemPtr++;
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_TDBAH (0), *MemPtr);
  DEBUGPRINT (XGBE, ("TdBah0 %X\n", *MemPtr));
  DEBUGWAIT (XGBE);
  IXGBE_WRITE_REG (
    &XgbeAdapter->Hw,
    IXGBE_TDLEN (0),
    (sizeof (struct ixgbe_legacy_tx_desc) * DEFAULT_TX_DESCRIPTORS)
  );
  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {
    XgbeSetRegBits (XgbeAdapter, IXGBE_DMATXCTL, IXGBE_DMATXCTL_TE);
  }

  XgbeSetRegBits (XgbeAdapter, IXGBE_TXDCTL (0), IXGBE_TXDCTL_ENABLE | IXGBE_TX_PAD_ENABLE);


  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {
    i = 0;
    do {
      TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_TXDCTL (0));
      i++;
      if ((TempReg & IXGBE_TXDCTL_ENABLE) != 0) {
        DEBUGPRINT (XGBE, ("TX queue enabled, after attempt i = %d\n", i));
        break;
      }

      DelayInMicroseconds (XgbeAdapter, 1);
    } while (i < 1000);
    if (i >= 1000) {
      DEBUGPRINT (CRITICAL, ("Enable TX queue failed!\n"));
    }
  }

  XgbePciFlush (XgbeAdapter);
}

/** Initializes the gigabit adapter, setting up memory addresses, MAC Addresses,
   Type of card, etc.

   @param[in]   XgbeAdapter   Pointer to adapter structure

   @retval   PXE_STATCODE_SUCCESS       Initialization succeeded
   @retval   PXE_STATCODE_NOT_STARTED   Hardware initialization failed
**/
PXE_STATCODE
XgbeInitialize (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  EFI_STATUS   Status;
  PXE_STATCODE PxeStatcode = PXE_STATCODE_SUCCESS;

  ZeroMem (
    (VOID *) (UINTN) XgbeAdapter->RxRing.UnmappedAddress,
    RX_RING_SIZE
    );

  ZeroMem (
    (VOID *) (UINTN) XgbeAdapter->TxRing.UnmappedAddress,
    TX_RING_SIZE
    );

  ZeroMem (
    (VOID *) (UINTN) XgbeAdapter->RxBufferMapping.UnmappedAddress,
    RX_BUFFERS_SIZE
    );

  // If the hardware has already been started then don't bother with a reset
  // We want to make sure we do not have to restart link negotiation.
  if (!XgbeAdapter->HwInitialized) {

    // Now that the structures are in place, we can configure the hardware to use it all.
    Status = XgbeInitHw (XgbeAdapter);
    if (EFI_ERROR (Status)) {
      DEBUGPRINT (CRITICAL, ("XgbeInitHw returns %r\n", Status));
      PxeStatcode = PXE_STATCODE_NOT_STARTED;
    }
  } else {
    DEBUGPRINT (XGBE, ("Skipping adapter reset\n"));
    PxeStatcode = PXE_STATCODE_SUCCESS;
  }

  // If we reset the adapter then reinitialize the TX and RX rings
  // and reconfigure interrupt causes.
  if (PxeStatcode == PXE_STATCODE_SUCCESS) {
    XgbeTxRxConfigure (XgbeAdapter);
    XgbeConfigureInterrupts (XgbeAdapter);
  }

  return PxeStatcode;
}

/** Disable Rx unit. Use the Shared Code implementation to
   make sure all WAs are in place.

   @param[in]   XgbeAdapter   Pointer to the adapter structure

   @return   RX unit disabled
**/
VOID
RxDisable (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  UINT32 RxCtrl;

  RxCtrl = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXCTRL);
  RxCtrl &= ~IXGBE_RXCTRL_RXEN;
  ixgbe_enable_rx_dma (&XgbeAdapter->Hw, RxCtrl);
}

/** Enable Rx unit. Use the Shared Code implementation to
   make sure all WAs are in place.

   @param[in]   XgbeAdapter   Pointer to the adapter structure

   @return   RX unit enabled
**/
VOID
RxEnable (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  UINT32 RxCtrl;

  RxCtrl = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXCTRL);
  RxCtrl |= IXGBE_RXCTRL_RXEN;
  ixgbe_enable_rx_dma (&XgbeAdapter->Hw, RxCtrl);
}

/** Changes filter settings

   @param[in]   XgbeAdapter  Pointer to the NIC data structure information which the
                            UNDI driver is layering on..
   @param[in]   NewFilter   A PXE_OPFLAGS bit field indicating what filters to use.

   @return   Filters changed according to NewFilter settings
**/
VOID
XgbeSetFilter (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT16            NewFilter
  )
{
  UINT32 Fctrl;
  UINT32 FctrlInitial;

  DEBUGPRINT (RXFILTER, ("XgbeSetFilter: "));

  Fctrl = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_FCTRL);
  FctrlInitial = Fctrl;

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
    Fctrl |= IXGBE_FCTRL_UPE;
    DEBUGPRINT (RXFILTER, ("IXGBE_FCTRL_UPE "));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
    Fctrl |= IXGBE_FCTRL_BAM;
    DEBUGPRINT (RXFILTER, ("IXGBE_FCTRL_BAM "));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
    Fctrl |= IXGBE_FCTRL_MPE;
    DEBUGPRINT (RXFILTER, ("IXGBE_FCTRL_MPE "));
  }

  XgbeAdapter->RxFilter |= NewFilter;
  DEBUGPRINT (RXFILTER, (", RxFilter=%08x, FCTRL=%08x\n", XgbeAdapter->RxFilter, Fctrl));

  if (Fctrl != FctrlInitial) {

    // Filter has changed - write the new value
    // Receiver must be disabled during write to IXGBE_FCTRL
    if (XgbeAdapter->ReceiveStarted) {
      if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
        (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x))
      {
        RxDisable (XgbeAdapter);
      }
    }

    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_FCTRL, Fctrl);

    if (XgbeAdapter->ReceiveStarted) {
      if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
        (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x))
      {
        RxEnable (XgbeAdapter);
      }
    }
  }

  // Start/Stop Rx unit based on the updated Rx filters
  if (XgbeAdapter->RxFilter != 0) {
    XgbeReceiveStart (XgbeAdapter);
  } else {
    XgbeReceiveStop (XgbeAdapter);
  }

  DEBUGWAIT (XGBE);
}

/** Clears receive filters.

   @param[in]   XgbeAdapter   Pointer to the adapter structure
   @param[in]   NewFilter   A PXE_OPFLAGS bit field indicating what filters to clear.

   @retval   0   Filters cleared according to NewFilter settings
**/
UINTN
XgbeClearFilter (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT16            NewFilter
  )
{
  UINT32 Fctrl;
  UINT32 FctrlInitial;

  DEBUGPRINT (RXFILTER, ("XgbeClearFilter %x: ", NewFilter));

  Fctrl = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_FCTRL);
  FctrlInitial = Fctrl;

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
    Fctrl &= ~IXGBE_FCTRL_UPE;
    DEBUGPRINT (RXFILTER, ("IXGBE_FCTRL_UPE "));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
    Fctrl &= ~IXGBE_FCTRL_BAM;
    DEBUGPRINT (RXFILTER, ("IXGBE_FCTRL_BAM "));
  }

  if (NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {

    // add the MPE bit to the variable to be written to the RCTL
    Fctrl &= ~IXGBE_FCTRL_MPE;
    DEBUGPRINT (RXFILTER, ("IXGBE_FCTRL_MPE "));
  }

  XgbeAdapter->RxFilter &= ~NewFilter;
  DEBUGPRINT (RXFILTER, (", RxFilter=%08x, FCTRL=%08x\n", XgbeAdapter->RxFilter, Fctrl));

  if (Fctrl != FctrlInitial) {

    // Filter has changed - write the new value
    // Receiver must be disabled during write to IXGBE_FCTRL
    if (XgbeAdapter->ReceiveStarted) {
    	if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
        (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x))
      {
        RxDisable (XgbeAdapter);
      }
    }

    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_FCTRL, Fctrl);

    if (XgbeAdapter->ReceiveStarted) {
      if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
        (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x))
      {
        RxEnable (XgbeAdapter);
      }
    }
  }

  // Start/Stop Rx unit based on the updated Rx filters
  if (XgbeAdapter->RxFilter != 0) {
    XgbeReceiveStart (XgbeAdapter);
  } else {
    XgbeReceiveStop (XgbeAdapter);
  }

  DEBUGPRINT (XGBE, ("XgbeClearFilter done.\n"));
  DEBUGWAIT (XGBE);
  return 0;
}


/** Updates multicast filters, updates MAC address list and enables multicast

   @param[in]   XgbeAdapter   Pointer to the adapter structure

   @return   All operations in description completed
**/
VOID
XgbeSetMcastList (
  XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  // Updating Mcast filters requires disabling Rx unit
  if (XgbeAdapter->ReceiveStarted) {
    if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
      (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x)) {
      RxDisable (XgbeAdapter);
    }
  }

  if (XgbeAdapter->McastList.Length == 0) {
    DEBUGPRINT (RXFILTER, ("Resetting multicast list\n"));
    XgbeAdapter->RxFilter &= ~PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST;
    ixgbe_disable_mc (&XgbeAdapter->Hw);
    if (XgbeAdapter->ReceiveStarted) {
      if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
        (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x))
      {
        RxEnable (XgbeAdapter);
      }
    }
    return;
  }

  XgbeAdapter->RxFilter |= PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST;

  DEBUGPRINT (RXFILTER, ("Update multicast list, count=%d\n", XgbeAdapter->McastList.Length));

  ixgbe_update_mc_addr_list (
    &XgbeAdapter->Hw,
    (UINT8 *) &XgbeAdapter->McastList.McAddr[0][0],
    XgbeAdapter->McastList.Length,
    _XgbeIterateMcastMacAddr,
    true
  );

  ixgbe_enable_mc (&XgbeAdapter->Hw);

  // Assume that if we are updating the MC list that we want to also
  // start the receiver.
  if (XgbeAdapter->ReceiveStarted) {
    if ((XgbeAdapter->Hw.mac.type != ixgbe_mac_X550) &&
      (XgbeAdapter->Hw.mac.type != ixgbe_mac_X550EM_x))
    {
      RxEnable (XgbeAdapter);
    }
  } else {
    XgbeReceiveStart (XgbeAdapter);
  }
}

/** Stops the receive unit. Receive queue is also reset and all existing packets are dropped.

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information
                              which the UNDI driver is layering on..

   @return   Receive unit stopped
**/
VOID
XgbeReceiveStop (
  IN XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  struct ixgbe_legacy_rx_desc *ReceiveDesc;
  UINTN                        i;
  UINT32                       RxdCtl;

  DEBUGPRINT (XGBE, ("XgbeReceiveStop\n"));

  if (!XgbeAdapter->ReceiveStarted) {
    DEBUGPRINT (CRITICAL, ("Receive unit already disabled!\n"));
    return;
  }

  XgbeClearRegBits (XgbeAdapter, IXGBE_RXDCTL (0), IXGBE_RXDCTL_ENABLE);
  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {
    do {
      gBS->Stall (1);
      RxdCtl = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXDCTL (0));
    } while ((RxdCtl & IXGBE_RXDCTL_ENABLE) != 0);
    DEBUGPRINT (XGBE, ("Receiver Disabled\n"));
  }

  ixgbe_disable_rx (&XgbeAdapter->Hw);

  // Reset the transmit and receive descriptor rings
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDH (0), 0);
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDT (0), 0);
  XgbeAdapter->CurRxInd = 0;

  // Clean up any left over packets
  ReceiveDesc = XGBE_RX_DESC (&XgbeAdapter->RxRing, 0);
  for (i = 0; i < DEFAULT_RX_DESCRIPTORS; i++) {
    ReceiveDesc->length = 0;
    ReceiveDesc->status = 0;
    ReceiveDesc->errors = 0;
    ReceiveDesc++;
  }

  XgbeAdapter->ReceiveStarted = FALSE;
}

/** Starts the receive unit.

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information which
                              the UNDI driver is layering on..

   @return   Receive unit started
**/
VOID
XgbeReceiveStart (
  IN XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  UINT32 TempReg;

  DEBUGPRINT (XGBE, ("XgbeReceiveStart\n"));

  if (XgbeAdapter->ReceiveStarted) {
    DEBUGPRINT (CRITICAL, ("Receive unit already started!\n"));
    return;
  }

  XgbeSetRegBits (XgbeAdapter, IXGBE_RXDCTL (0), IXGBE_RXDCTL_ENABLE);
  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {
    do {
      gBS->Stall (1);
      TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_RXDCTL (0));
    } while ((TempReg & IXGBE_RXDCTL_ENABLE) == 0);
  }

  // Advance the tail descriptor to tell the hardware it can use the descriptors
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_RDT (0), DEFAULT_RX_DESCRIPTORS - 1);

  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {
    DEBUGPRINT (XGBE, ("Disabling SECRX.\n"));
    XgbeSetRegBits (XgbeAdapter, IXGBE_SECRXCTRL, IXGBE_SECRXCTRL_RX_DIS);
    do {
      TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_SECRXSTAT);
      DEBUGPRINT (XGBE, ("SEC_RX = %x\n", TempReg));
    } while ((TempReg & IXGBE_SECRXSTAT_SECRX_RDY) == 0);
    DEBUGPRINT (XGBE, ("SEC_RX has been disabled.\n"));
  }

  ixgbe_enable_rx (&XgbeAdapter->Hw);

  if ((XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x)
    || (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a)
    )
  {
    XgbeClearRegBits (XgbeAdapter, IXGBE_SECRXCTRL, IXGBE_SECRXCTRL_RX_DIS);
  }

  XgbeAdapter->ReceiveStarted = TRUE;
}

/** Delay a specified number of microseconds

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information
                              which the UNDI driver is layering on..
   @param[in]   MicroSeconds   Time to delay in Microseconds.

   @return   Execution of code delayed
**/
VOID
DelayInMicroseconds (
  IN XGBE_DRIVER_DATA *XgbeAdapter,
  IN UINT32               MicroSeconds
  )
{
  if (XgbeAdapter->Delay != NULL) {
    (*XgbeAdapter->Delay)(XgbeAdapter->UniqueId, MicroSeconds);
  } else {
    gBS->Stall (MicroSeconds);
  }
}

/** Swaps the bytes from machine order to network order (Big Endian)

   @param[in]   Dword   32-bit input value

   @return    Big Endian swapped value
**/
UINT32
IxgbeHtonl (
  IN UINT32 Dword
  )
{
  UINT8   Buffer[4];
  UINT32 *Result;

  DEBUGPRINT (XGBE, ("IxgbeHtonl = %x\n", Dword));

  Buffer[3] = (UINT8) Dword;
  Dword     = Dword >> 8;
  Buffer[2] = (UINT8) Dword;
  Dword     = Dword >> 8;
  Buffer[1] = (UINT8) Dword;
  Dword     = Dword >> 8;
  Buffer[0] = (UINT8) Dword;

  Result    = (UINT32 *) Buffer;
  DEBUGPRINT (XGBE, ("IxgbeHtonl result %x\n", *Result));
  DEBUGWAIT (XGBE);

  return *Result;
}

/** This function calls the MemIo callback to read a dword from the device's
   address space

   @param[in]   XgbeAdapter   Adapter structure
   @param[in]   Port          Address to read from

   @retval    The data read from the port.
**/
UINT32
XgbeInDword (
  IN XGBE_DRIVER_DATA *XgbeAdapter,
  IN UINT32            Port
  )
{
  UINT32 Results;

  MemoryFence ();
  XgbeAdapter->PciIo->Mem.Read (
                            XgbeAdapter->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            Port,
                            1,
                            (VOID *) (&Results)
                          );
  MemoryFence ();
  return Results;
}

/** This function calls the MemIo callback to write a word from the device's
   address space

   @param[in]   XgbeAdapter   Adapter structure
   @param[in]   Port          Address to write to
   @param[in]   Data          Data to write to Port

   @return   Word written
**/
VOID
XgbeOutDword (
  IN XGBE_DRIVER_DATA *XgbeAdapter,
  IN UINT32            Port,
  IN UINT32            Data
  )
{
  UINT32 Value;

  Value = Data;

  MemoryFence ();

  XgbeAdapter->PciIo->Mem.Write (
                            XgbeAdapter->PciIo,
                            EfiPciIoWidthUint32,
                            0,
                            Port,
                            1,
                            (VOID *) (&Value)
                          );

  MemoryFence ();
  return;
}

/** Sets specified bits in a device register

   @param[in]   XgbeAdapter   Pointer to the device instance
   @param[in]   Register      Register to write
   @param[in]   BitMask       Bits to set

   @return    Returns the value read from the PCI register with BitMask applied.
**/
UINT32
XgbeSetRegBits (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT32            Register,
  UINT32            BitMask
  )
{
  UINT32 TempReg;

  TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, Register);
  TempReg |= BitMask;
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, Register, TempReg);

  return TempReg;
}

/** Clears specified bits in a device register

   @param[in]   XgbeAdapter   Pointer to the device instance
   @param[in]   Register      Register to write
   @param[in]   BitMask       Bits to clear

   @return    Returns the value read from the PCI register with ~BitMask applied.
**/
UINT32
XgbeClearRegBits (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT32            Register,
  UINT32            BitMask
  )
{
  UINT32 TempReg;

  TempReg = IXGBE_READ_REG (&XgbeAdapter->Hw, Register);
  TempReg &= ~BitMask;
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, Register, TempReg);

  return TempReg;
}


/** This function calls the EFI PCI IO protocol to read a value from the device's PCI
   register space.

   @param[in]   XgbeAdapter   Pointer to the shared code hw structure.
   @param[in]   Offset        Which register to read from.

   @return     The value read from the PCI register.
**/
UINT16
XgbeReadPci16 (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT32            Offset
  )
{
  UINT16 Data;

  MemoryFence ();

  XgbeAdapter->PciIo->Pci.Read (
                            XgbeAdapter->PciIo,
                            EfiPciIoWidthUint16,
                            Offset,
                            1,
                            (VOID *) (&Data)
                          );
  MemoryFence ();
  return Data;
}

/** This function calls the EFI PCI IO protocol to write a value to the device's PCI
   register space.

   @param[in]   XgbeAdapter   Pointer to the adapter structure.
   @param[in]   Offset        Which register to read from.
   @param[in]   Data          Returns the value read from the PCI register.

   @return    Value present in Data was written
**/
VOID
XgbeWritePci16 (
  XGBE_DRIVER_DATA *XgbeAdapter,
  UINT32            Offset,
  UINT16            Data
  )
{
  MemoryFence ();

  XgbeAdapter->PciIo->Pci.Write (
                            XgbeAdapter->PciIo,
                            EfiPciIoWidthUint16,
                            Offset,
                            1,
                            (VOID *) (&Data)
                          );
  MemoryFence ();
}

/** Flushes a PCI write transaction to system memory.

   @param[in]   XgbeAdapter   Pointer to the adapter structure.

   @return   Write transaction flushed
**/
VOID
XgbePciFlush (
  IN XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  MemoryFence ();

  XgbeAdapter->PciIo->Flush (XgbeAdapter->PciIo);

  MemoryFence ();

  return;
}

/** Free TX buffers that have been transmitted by the hardware.

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information which
                              the UNDI driver is layering on.
   @param[in]   NumEntries    Number of entries in the array which can be freed.
   @param[out]  TxBuffer      Array to pass back free TX buffer

   @return   Number of TX buffers written.
**/
UINT16
XgbeFreeTxBuffers (
  IN XGBE_DRIVER_DATA *XgbeAdapter,
  IN UINT16            NumEntries,
  OUT UINT64 *         TxBuffer
  )
{
  struct ixgbe_legacy_tx_desc *TransmitDescriptor;
  UINT32                       Tdh;
  UINT16                       i;
  UNDI_DMA_MAPPING             *TxBufMapping;

  //  Read the TX head posistion so we can see which packets have been sent out on the wire.
  Tdh = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_TDH (0));
  DEBUGPRINT (XGBE, ("TDH = %d, XgbeAdapter->XmitDoneHead = %d\n", Tdh, XgbeAdapter->XmitDoneHead));

  //  If Tdh does not equal xmit_done_head then we will fill all the transmitted buffer
  // addresses between Tdh and xmit_done_head into the completed buffers array
  i = 0;
  do {
    if (i >= NumEntries) {
      DEBUGPRINT (XGBE, ("Exceeded number of DB entries, i=%d, NumEntries=%d\n", i, NumEntries));
      break;
    }

    TransmitDescriptor = XGBE_TX_DESC (&XgbeAdapter->TxRing, XgbeAdapter->XmitDoneHead);
    TxBufMapping = &XgbeAdapter->TxBufferMappings[XgbeAdapter->XmitDoneHead];

    if ((TransmitDescriptor->upper.fields.status & IXGBE_TXD_STAT_DD) != 0) {

      if (TxBufMapping->UnmappedAddress == 0) {
        DEBUGPRINT (CRITICAL, ("ERROR: TX buffer complete without being marked used!\n"));
        break;
      }

      DEBUGPRINT (XGBE, ("Writing buffer address %d, %x\n", i, TxBuffer[i]));
      UndiDmaUnmapMemory (XgbeAdapter->PciIo, TxBufMapping);

      TxBuffer[i] = TxBufMapping->UnmappedAddress;
      i++;

      ZeroMem (TxBufMapping, sizeof (UNDI_DMA_MAPPING));
      TransmitDescriptor->upper.fields.status                 = 0;

      XgbeAdapter->XmitDoneHead++;
      if (XgbeAdapter->XmitDoneHead >= DEFAULT_TX_DESCRIPTORS) {
        XgbeAdapter->XmitDoneHead = 0;
      }
    } else {
      DEBUGPRINT (XGBE, ("TX Descriptor %d not done\n", XgbeAdapter->XmitDoneHead));
      break;
    }
  } while (Tdh != XgbeAdapter->XmitDoneHead);
  return i;
}

/** Checks if link is up

   @param[in]   XgbeAdapter   Pointer to the NIC data structure information
                             which the UNDI driver is layering on.

   @retval   TRUE   Link is up
   @retval   FALSE  Link is down
**/
BOOLEAN
IsLinkUp (
  IN XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  ixgbe_link_speed Speed;
  BOOLEAN          LinkUp;

  if (XgbeAdapter->QualificationResult != MODULE_SUPPORTED) {
    return FALSE;
  }

  ixgbe_check_link (&XgbeAdapter->Hw, &Speed, &LinkUp, FALSE);
  return LinkUp;
}

/** Gets current link speed and duplex from shared code and converts it to UNDI
   driver format

   @param[in]   XgbeAdapter   Pointer to the device instance

   @retval    LINK_SPEED_100FULL    100 MBit full duplex
   @retval    LINK_SPEED_1000FULL   1   GBit full duplex
   @retval    LINK_SPEED_10000FULL  10  GBit full duplex
   @retval    LINK_SPEED_UNKNOWN    Unknown link speed
**/
UINT8
GetLinkSpeed (
  IN XGBE_DRIVER_DATA *XgbeAdapter
  )
{
  ixgbe_link_speed Speed     = 0;
  BOOLEAN          LinkUp;
  UINT8            LinkSpeed = LINK_SPEED_UNKNOWN;

  if (XgbeAdapter->QualificationResult != MODULE_SUPPORTED) {
    LinkSpeed = LINK_SPEED_UNKNOWN;
    return LinkSpeed;
  }

  ixgbe_check_link (&XgbeAdapter->Hw, &Speed, &LinkUp, FALSE);
  switch (Speed) {
  case IXGBE_LINK_SPEED_10_FULL:
    LinkSpeed = LINK_SPEED_10FULL;
    break;
  case IXGBE_LINK_SPEED_100_FULL:
    LinkSpeed = LINK_SPEED_100FULL;
    break;
  case IXGBE_LINK_SPEED_1GB_FULL:
    LinkSpeed = LINK_SPEED_1000FULL;
    break;
  case IXGBE_LINK_SPEED_10GB_FULL:
    LinkSpeed = LINK_SPEED_10000FULL;
    break;
  default:
    LinkSpeed = LINK_SPEED_UNKNOWN;
    break;
  }
  DEBUGPRINT (HII, ("Link Speed Status %x\n", LinkSpeed));
  return LinkSpeed;
}

/** Blinks LED on a port for time given in seconds

   @param[in]   XgbeAdapter   Pointer to the device instance
   @param[in]   Time         Seconds to blink

   @return    LED is blinking for Time seconds
**/
VOID
BlinkLeds (
  IN XGBE_DRIVER_DATA *XgbeAdapter,
  IN UINT32            Time
  )
{
  UINT32  LedCtl;
  BOOLEAN LedOn = FALSE;
  UINT32  i = 0;
  UINT32  LedIndex = 2;
  UINT16  PhyRegVal = 0;

  // IXGBE shared code doesn't save/restore the LEDCTL register when blinking used.
  LedCtl = IXGBE_READ_REG (&XgbeAdapter->Hw, IXGBE_LEDCTL);

  if (XgbeAdapter->Hw.mac.type == ixgbe_mac_X540) {

    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_LEDCTL, (LedCtl & ~0xFF) | 0x4E);
  } else if (XgbeAdapter->Hw.mac.type == ixgbe_mac_82599EB &&
    XgbeAdapter->Hw.device_id == 0x154A)
  {
    IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_LEDCTL, (LedCtl & ~0xFF00) | 0x4E00);
  }

  if (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_x) {
    LedIndex = 0;
  }
  else if (XgbeAdapter->Hw.mac.type == ixgbe_mac_X550EM_a) {
    LedIndex = XgbeAdapter->Hw.mac.led_link_act;
  }

  switch (XgbeAdapter->Hw.device_id) {
  case IXGBE_DEV_ID_X550EM_X_10G_T:
  case IXGBE_DEV_ID_X550EM_A_10G_T:
    {
      ixgbe_read_phy_reg (
        &XgbeAdapter->Hw,
        IXGBE_X557_LED_PROVISIONING + LedIndex,
        IXGBE_MDIO_VENDOR_SPECIFIC_1_DEV_TYPE,
        &PhyRegVal
      );

      ixgbe_write_phy_reg (
        &XgbeAdapter->Hw,
        IXGBE_X557_LED_PROVISIONING + LedIndex,
        IXGBE_MDIO_VENDOR_SPECIFIC_1_DEV_TYPE,
        0
      );
    }
  }
  if (Time > 0) {
    for (i = 0; i < Time * 1000; i += BLINK_INTERVAL) {
      LedOn = !LedOn;
      if (LedOn) {
        ixgbe_led_on (&XgbeAdapter->Hw, LedIndex);
      } else {
        ixgbe_led_off (&XgbeAdapter->Hw, LedIndex);
      }
      DelayInMicroseconds (XgbeAdapter, BLINK_INTERVAL * 1000);
    }
  }
  IXGBE_WRITE_REG (&XgbeAdapter->Hw, IXGBE_LEDCTL, LedCtl);
  IXGBE_WRITE_FLUSH (&XgbeAdapter->Hw);

  switch (XgbeAdapter->Hw.device_id) {
  case IXGBE_DEV_ID_X550EM_X_10G_T:
  case IXGBE_DEV_ID_X550EM_A_10G_T:
    {
      ixgbe_write_phy_reg (
        &XgbeAdapter->Hw,
        IXGBE_X557_LED_PROVISIONING + LedIndex,
        IXGBE_MDIO_VENDOR_SPECIFIC_1_DEV_TYPE,
        PhyRegVal
      );
    }
  }
}

/** Reads PBA string from NVM

   @param[in]       XgbeAdapter     Pointer to the device instance
   @param[in,out]   PbaNumber      Pointer to buffer for PBA string
   @param[in]       PbaNumberSize  Size of PBA string

   @retval   EFI_SUCCESS            PBA string successfully read
   @retval   EFI_DEVICE_ERROR       Failed to read PBA string
**/
EFI_STATUS
ReadPbaString (
  IN     XGBE_DRIVER_DATA *XgbeAdapter,
  IN OUT UINT8 *           PbaNumber,
  IN     UINT32            PbaNumberSize
  )
{
  if (ixgbe_read_pba_string (&XgbeAdapter->Hw, PbaNumber, PbaNumberSize) == IXGBE_SUCCESS) {
    return EFI_SUCCESS;
  } else {
    return EFI_DEVICE_ERROR;
  }
}

/** Reverse bytes of a word (endianness change)

   @param[in]   Word   Value to be modified

   @return   Word reversed
**/
UINT16
IxgbeReverseWord (
  IN UINT16 Word
  )
{
  UINT8  SwapBuf;
  UINT8 *Ptr;

  Ptr = (UINT8 *) &Word;
  SwapBuf = Ptr[0];
  Ptr[0] = Ptr[1];
  Ptr[1] = SwapBuf;

  return Word;
}

/** Reverse bytes of a double word (endianness change)

   @param[in]   DWord   Value to be modified

   @return   DWord reversed
**/
UINT32
IxgbeReverseDword (
  IN UINT32 Dword
  )
{
  UINT16  SwapBuf;
  UINT16 *Ptr;

  Ptr = (UINT16 *) &Dword;
  SwapBuf = Ptr[0];
  Ptr[0] = IxgbeReverseWord (Ptr[1]);
  Ptr[1] = IxgbeReverseWord (SwapBuf);

  return Dword;
}

/** Check if current module used for this port is qualified module

   @param[in]   AdapterInfo   Pointer to the NIC data structure information
                              which the UNDI driver is layering on..

   @retval   MODULE_SUPPORTED    Module is qualified
   @retval   MODULE_UNSUPPORTED  Module is unqualified and module
                                 qualification is enabled on the port
**/
MODULE_QUALIFICATION_STATUS
GetModuleQualificationResult (
  IN  XGBE_DRIVER_DATA *AdapterInfo
  )
{
  INT32 ScStatus;

  ScStatus = AdapterInfo->Hw.phy.ops.identify_sfp (&AdapterInfo->Hw);
  DEBUGPRINT (HEALTH, ("identify_sfp returns = %d\n", ScStatus));
  if (ScStatus != IXGBE_SUCCESS
    && ScStatus != IXGBE_ERR_SFP_NOT_PRESENT)
  {
    return MODULE_UNSUPPORTED;
  }
  // Module is qualified or qualification process is not enabled
  return MODULE_SUPPORTED;
}


