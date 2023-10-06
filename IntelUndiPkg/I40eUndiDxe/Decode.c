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
#include "Decode.h"

// Forward declarations for UNDI function table
/** This routine determines the operational state of the UNDI.  It updates the state flags in the
   Command Descriptor Block based on information derived from the AdapterInfo instance data.

   To ensure the command has completed successfully, CdbPtr->StatCode will contain the result of
   the command execution. The CdbPtr->StatFlags will contain a STOPPED, STARTED, or INITIALIZED
   state once the command has successfully completed. Keep in mind the AdapterInfo->State is the
   active state of the adapter (based on software interrogation), and the CdbPtr->StateFlags is
   the passed back information that is reflected to the caller of the UNDI API.


   @param[in]   CdbPtr       Pointer to the command descriptor block.
   @param[in]   AdapterInfo  Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval      None
**/
VOID
UndiGetState (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to change the operational state of the 10-Gigabit UNDI
   from stopped to started.

   It will do this as long as the adapter's state is PXE_STATFLAGS_GET_STATE_STOPPED, otherwise
   the CdbPtr->StatFlags will reflect a command failure, and the CdbPtr->StatCode will reflect the
   UNDI as having already been started.
   This routine is modified to reflect the UNDI 1.1 specification changes. The
   changes in the spec. are mainly in the callback routines, the new spec. adds
   3 more callbacks and a unique id. Since this UNDI supports both old and new UNDI specifications,
   The NIC's data structure is filled in with the callback routines (depending
   on the version) pointed to in the caller's CpbPtr.  This seeds the Delay,
   Virt2Phys, Block, and Mem_IO for old and new versions and Map_Mem, UnMap_Mem
   and Sync_Mem routines and a unique id variable for the new version.
   This is the function which an external entity (SNP, O/S, etc) would call
   to provide it's I/O abstraction to the UNDI.
   It's final action is to change the AdapterInfo->State to PXE_STATFLAGS_GET_STATE_STARTED.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiStart (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to change the operational state of the UNDI from started to stopped.

   It will not do this if the adapter's state is PXE_STATFLAGS_GET_STATE_INITIALIZED, otherwise
   the CdbPtr->StatFlags will reflect a command failure, and the CdbPtr->StatCode will reflect the
   UNDI as having already not been shut down.
   The NIC's data structure will have the Delay, Virt2Phys, and Block, pointers zero'd out..
   It's final action is to change the AdapterInfo->State to PXE_STATFLAGS_GET_STATE_STOPPED.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiStop (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to retrieve the initialization information that is
   needed by drivers and applications to initialize the UNDI.

   This will fill in data in the Data Block structure that is pointed to by the
   caller's CdbPtr->DBaddr.  The fields filled in are as follows:
   MemoryRequired, FrameDataLen, LinkSpeeds[0-3], NvCount, NvWidth, MediaHeaderLen, HWaddrLen,
   MCastFilterCnt, TxBufCnt, TxBufSize, RxBufCnt, RxBufSize, IFtype, Duplex, and LoopBack.
   In addition, the CdbPtr->StatFlags ORs in that this NIC supports cable detection.  (APRIORI knowledge)

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiGetInitInfo (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to retrieve the configuration information about the NIC being controlled by
  this driver.

  This will fill in data in the Data Block structure that is pointed to by the caller's CdbPtr->DBaddr.
  The fields filled in are as follows:
  DbPtr->pci.BusType, DbPtr->pci.Bus, DbPtr->pci.Device, and DbPtr->pci.
  In addition, the DbPtr->pci.Config.Dword[0-63] grabs a copy of this NIC's PCI configuration space.

  @param[in]   CdbPtr        Pointer to the command descriptor block.
  @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

  @retval      None
**/
VOID
UndiGetConfigInfo (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine resets the network adapter and initializes the 10-Gigabit UNDI using the parameters
   supplied in the CPB.

   This command must be issued before the network adapter can be setup to transmit and receive packets.
   Once the memory requirements of the UNDI are obtained by using the GetInitInfo command, a block
   of non-swappable memory may need to be allocated.  The address of this memory must be passed to
   UNDI during the Initialize in the CPB.  This memory is used primarily for transmit and receive buffers.
   The fields CableDetect, LinkSpeed, Duplex, LoopBack, MemoryPtr, and MemoryLength are set with
   information that was passed in the CPB and the NIC is initialized.
   If the NIC initialization fails, the CdbPtr->StatFlags are updated with PXE_STATFLAGS_COMMAND_FAILED
   Otherwise, AdapterInfo->State is updated with PXE_STATFLAGS_GET_STATE_INITIALIZED showing the state of
   the UNDI is now initialized.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval      None
-**/
VOID
UndiInitialize (
  IN  PXE_CDB      *CdbPtr,
  I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine resets the network adapter and initializes the 10-Gigabit UNDI using the
   parameters supplied in the CPB.

   The transmit and receive queues are emptied and any pending interrupts are cleared.
   If the NIC reset fails, the CdbPtr->StatFlags are updated with PXE_STATFLAGS_COMMAND_FAILED

   @param[in]   CdbPtr         Pointer to the command descriptor block.
   @param[in]   AdapterInfo    Pointer to the NIC data structure information which the
                               UNDI driver is layering on..

   @retval      None
**/
VOID
UndiReset (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine resets the network adapter and leaves it in a safe state for another
   driver to initialize.

   Any pending transmits or receives are lost.  Receive filters and external
   interrupt enables are disabled.  Once the UNDI has been shutdown, it can then be stopped
   or initialized again.
   If the NIC reset fails, the CdbPtr->StatFlags are updated with PXE_STATFLAGS_COMMAND_FAILED
   Otherwise, AdapterInfo->State is updated with PXE_STATFLAGS_GET_STATE_STARTED showing
   the state of the NIC as being started.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval      None
**/
VOID
UndiShutdown (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine can be used to read and/or change the current external interrupt enable
   settings.

   Disabling an external interrupt enable prevents and external (hardware)
   interrupt from being signalled by the network device.  Internally the interrupt events
   can still be polled by using the UNDI_GetState command.
   The resulting information on the interrupt state will be passed back in the CdbPtr->StatFlags.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on.

   @retval      None
**/
VOID
UndiInterrupt (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to read and change receive filters and, if supported, read
   and change multicast MAC address filter list.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval     None
**/
VOID
UndiRecFilter (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to get the current station and broadcast MAC addresses,
   and to change the current station MAC address.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on.

   @retval      None
**/
VOID
UndiStnAddr (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to read and clear the NIC traffic statistics.  This command is supported
   only if the !PXE structure's Implementation flags say so.

   Results will be parsed out in the following manner:
   CdbPtr->DBaddr.Data[0]   R  Total Frames (Including frames with errors and dropped frames)
   CdbPtr->DBaddr.Data[1]   R  Good Frames (All frames copied into receive buffer)
   CdbPtr->DBaddr.Data[2]   R  Undersize Frames (Frames below minimum length for media <64 for ethernet)
   CdbPtr->DBaddr.Data[4]   R  Dropped Frames (Frames that were dropped because receive buffers were full)
   CdbPtr->DBaddr.Data[8]   R  CRC Error Frames (Frames with alignment or CRC errors)
   CdbPtr->DBaddr.Data[A]   T  Total Frames (Including frames with errors and dropped frames)
   CdbPtr->DBaddr.Data[B]   T  Good Frames (All frames copied into transmit buffer)
   CdbPtr->DBaddr.Data[C]   T  Undersize Frames (Frames below minimum length for media <64 for ethernet)
   CdbPtr->DBaddr.Data[E]   T  Dropped Frames (Frames that were dropped because of collisions)
   CdbPtr->DBaddr.Data[14]  T  Total Collision Frames (Total collisions on this subnet)

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiStatistics (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to translate a multicast IP address to a multicast MAC address.

   This results in a MAC address composed of 25 bits of fixed data with the upper 23 bits of the IP
   address being appended to it.  Results passed back in the equivalent of CdbPtr->DBaddr->MAC[0-5].

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiIp2Mac (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to read and write non-volatile storage on the NIC (if supported).  The NVRAM
   could be EEPROM, FLASH, or battery backed RAM.

   This is an optional function according to the UNDI specification  (or will be......)

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiNvData (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine returns the current interrupt status and/or the transmitted buffer addresses.

   If the current interrupt status is returned, pending interrupts will be acknowledged by this
   command.  Transmitted buffer addresses that are written to the DB are removed from the transmit
   buffer queue.
   Normally, this command would be polled with interrupts disabled.
   The transmit buffers are returned in CdbPtr->DBaddr->TxBufer[0 - NumEntries].
   The interrupt status is returned in CdbPtr->StatFlags.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the 10-Gigabit
                             UNDI driver is layering on..

   @retval   None
**/
VOID
UndiStatus (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to fill media header(s) in transmit packet(s).

   Copies the MAC address into the media header whether it is dealing
   with fragmented or non-fragmented packets.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on.

   @retval      None
**/
VOID
UndiFillHeader (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** This routine is used to place a packet into the transmit queue.

   The data buffers given to this command are to be considered locked and the application or
   network driver loses ownership of these buffers and must not free or relocate them until
   the ownership returns.
   When the packets are transmitted, a transmit complete interrupt is generated (if interrupts
   are disabled, the transmit interrupt status is still set and can be checked using the UNDI_Status
   command.
   Some implementations and adapters support transmitting multiple packets with one transmit
   command.  If this feature is supported, the transmit CPBs can be linked in one transmit
   command.
   All UNDIs support fragmented frames, now all network devices or protocols do.  If a fragmented
   frame CPB is given to UNDI and the network device does not support fragmented frames
   (see !PXE.Implementation flag), the UNDI will have to copy the fragments into a local buffer
   before transmitting.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiTransmit (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/** When the network adapter has received a frame, this command is used to copy the frame
   into the driver/application storage location.

   Once a frame has been copied, it is removed from the receive queue.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiReceive (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  );

/* Global variables */

UNDI_CALL_TABLE gUndiApiTable[PXE_OPCODE_LAST_VALID + 1] = {
  {
    PXE_CPBSIZE_NOT_USED,
    PXE_DBSIZE_NOT_USED,
    0,
    (UINT16) (ANY_STATE),
    UndiGetState
  },
  {
    (UINT16) (DONT_CHECK),
    PXE_DBSIZE_NOT_USED,
    0,
    (UINT16) (ANY_STATE),
    UndiStart
  },
  {
    PXE_CPBSIZE_NOT_USED,
    PXE_DBSIZE_NOT_USED,
    0,
    MUST_BE_STARTED,
    UndiStop
  },
  {
    PXE_CPBSIZE_NOT_USED,
    sizeof (PXE_DB_GET_INIT_INFO),
    0,
    MUST_BE_STARTED,
    UndiGetInitInfo
  },
  {
    PXE_CPBSIZE_NOT_USED,
    sizeof (PXE_DB_GET_CONFIG_INFO),
    0,
    MUST_BE_STARTED,
    UndiGetConfigInfo
  },
  {
    sizeof (PXE_CPB_INITIALIZE),
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    MUST_BE_STARTED,
    UndiInitialize
  },
  {
    PXE_CPBSIZE_NOT_USED,
    PXE_DBSIZE_NOT_USED,
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiReset
  },
  {
    PXE_CPBSIZE_NOT_USED,
    PXE_DBSIZE_NOT_USED,
    0,
    MUST_BE_INITIALIZED,
    UndiShutdown
  },
  {
    PXE_CPBSIZE_NOT_USED,
    PXE_DBSIZE_NOT_USED,
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiInterrupt
  },
  {
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiRecFilter
  },
  {
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiStnAddr
  },
  {
    PXE_CPBSIZE_NOT_USED,
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiStatistics
  },
  {
    sizeof (PXE_CPB_MCAST_IP_TO_MAC),
    sizeof (PXE_DB_MCAST_IP_TO_MAC),
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiIp2Mac
  },
  {
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiNvData
  },
  {
    PXE_CPBSIZE_NOT_USED,
    (UINT16) (DONT_CHECK),
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiStatus
  },
  {
    (UINT16) (DONT_CHECK),
    PXE_DBSIZE_NOT_USED,
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiFillHeader
  },
  {
    (UINT16) (DONT_CHECK),
    PXE_DBSIZE_NOT_USED,
    (UINT16) (DONT_CHECK),
    MUST_BE_INITIALIZED,
    UndiTransmit
  },
  {
    sizeof (PXE_CPB_RECEIVE),
    sizeof (PXE_DB_RECEIVE),
    0,
    MUST_BE_INITIALIZED,
    UndiReceive
  }
};

/* Function definitions */

/** This routine determines the operational state of the UNDI.  It updates the state flags in the
   Command Descriptor Block based on information derived from the AdapterInfo instance data.

   To ensure the command has completed successfully, CdbPtr->StatCode will contain the result of
   the command execution. The CdbPtr->StatFlags will contain a STOPPED, STARTED, or INITIALIZED
   state once the command has successfully completed. Keep in mind the AdapterInfo->State is the
   active state of the adapter (based on software interrogation), and the CdbPtr->StateFlags is
   the passed back information that is reflected to the caller of the UNDI API.


   @param[in]   CdbPtr       Pointer to the command descriptor block.
   @param[in]   AdapterInfo  Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval      None
**/
VOID
UndiGetState (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  CdbPtr->StatFlags |= AdapterInfo->State;
  CdbPtr->StatFlags |= PXE_STATFLAGS_COMMAND_COMPLETE;

  CdbPtr->StatCode = PXE_STATCODE_SUCCESS;

  return;
}

/** This routine is used to change the operational state of the 10-Gigabit UNDI
   from stopped to started.

   It will do this as long as the adapter's state is PXE_STATFLAGS_GET_STATE_STOPPED, otherwise
   the CdbPtr->StatFlags will reflect a command failure, and the CdbPtr->StatCode will reflect the
   UNDI as having already been started.
   This routine is modified to reflect the UNDI 1.1 specification changes. The
   changes in the spec. are mainly in the callback routines, the new spec. adds
   3 more callbacks and a unique id. Since this UNDI supports both old and new UNDI specifications,
   The NIC's data structure is filled in with the callback routines (depending
   on the version) pointed to in the caller's CpbPtr.  This seeds the Delay,
   Virt2Phys, Block, and Mem_IO for old and new versions and Map_Mem, UnMap_Mem
   and Sync_Mem routines and a unique id variable for the new version.
   This is the function which an external entity (SNP, O/S, etc) would call
   to provide it's I/O abstraction to the UNDI.
   It's final action is to change the AdapterInfo->State to PXE_STATFLAGS_GET_STATE_STARTED.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiStart (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{

  PXE_CPB_START_31  *CpbPtr31;

  DEBUGPRINT (DECODE, ("UndiStart\n"));
  DEBUGWAIT (DECODE);

  // check if it is already started.
  if (AdapterInfo->State != PXE_STATFLAGS_GET_STATE_STOPPED) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiStart called when driver state not stopped\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_ALREADY_STARTED;
    return;
  }

  if (CdbPtr->CPBsize != sizeof (PXE_CPB_START_30)
    && CdbPtr->CPBsize != sizeof (PXE_CPB_START_31))
  {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiStart CPD size incorrect\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    return;
  }

  CpbPtr31               = (PXE_CPB_START_31 *) (UINTN) (CdbPtr->CPBaddr);

  AdapterInfo->Delay      = (BS_PTR) (UINTN) CpbPtr31->Delay;
  AdapterInfo->Virt2Phys  = (VIRT_PHYS) (UINTN) CpbPtr31->Virt2Phys;
  AdapterInfo->Block      = (BLOCK) (UINTN) CpbPtr31->Block;
  AdapterInfo->MemIo     = (MEM_IO) (UINTN) CpbPtr31->Mem_IO;

  AdapterInfo->MapMem    = (MAP_MEM) (UINTN) CpbPtr31->Map_Mem;
  AdapterInfo->UnMapMem  = (UNMAP_MEM) (UINTN) CpbPtr31->UnMap_Mem;
  AdapterInfo->SyncMem   = (SYNC_MEM) (UINTN) CpbPtr31->Sync_Mem;
  AdapterInfo->UniqueId  = CpbPtr31->Unique_ID;

  AdapterInfo->State  = PXE_STATFLAGS_GET_STATE_STARTED;

  CdbPtr->StatFlags   = PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode    = PXE_STATCODE_SUCCESS;
}

/** This routine is used to change the operational state of the UNDI from started to stopped.

   It will not do this if the adapter's state is PXE_STATFLAGS_GET_STATE_INITIALIZED, otherwise
   the CdbPtr->StatFlags will reflect a command failure, and the CdbPtr->StatCode will reflect the
   UNDI as having already not been shut down.
   The NIC's data structure will have the Delay, Virt2Phys, and Block, pointers zero'd out..
   It's final action is to change the AdapterInfo->State to PXE_STATFLAGS_GET_STATE_STOPPED.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiStop (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{

  DEBUGPRINT (DECODE, ("UndiStop\n"));
  DEBUGWAIT (DECODE);
  if (AdapterInfo->State == PXE_STATFLAGS_GET_STATE_INITIALIZED) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiStop called when driver not initialized\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_NOT_SHUTDOWN;
    return;
  }

  AdapterInfo->Delay        = 0;
  AdapterInfo->Virt2Phys    = 0;
  AdapterInfo->Block        = 0;

  AdapterInfo->MapMem      = 0;
  AdapterInfo->UnMapMem    = 0;
  AdapterInfo->SyncMem     = 0;

  AdapterInfo->State        = PXE_STATFLAGS_GET_STATE_STOPPED;

  CdbPtr->StatFlags         = PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode          = PXE_STATCODE_SUCCESS;
}

/** This routine is used to retrieve the initialization information that is
   needed by drivers and applications to initialize the UNDI.

   This will fill in data in the Data Block structure that is pointed to by the
   caller's CdbPtr->DBaddr.  The fields filled in are as follows:
   MemoryRequired, FrameDataLen, LinkSpeeds[0-3], NvCount, NvWidth, MediaHeaderLen, HWaddrLen,
   MCastFilterCnt, TxBufCnt, TxBufSize, RxBufCnt, RxBufSize, IFtype, Duplex, and LoopBack.
   In addition, the CdbPtr->StatFlags ORs in that this NIC supports cable detection.  (APRIORI knowledge)

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the UNDI driver is layering on..

   @retval      None
**/
VOID
UndiGetInitInfo (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_DB_GET_INIT_INFO  *DbPtr;

  DEBUGPRINT (INIT, ("UndiGetInitInfo\n"));
  DEBUGWAIT (DECODE);
  DbPtr                 = (PXE_DB_GET_INIT_INFO *) (UINTN) (CdbPtr->DBaddr);

  DbPtr->MemoryRequired = 0;
  DbPtr->FrameDataLen   = PXE_MAX_TXRX_UNIT_ETHER;

  DbPtr->LinkSpeeds[0]  = 10000;
  DbPtr->LinkSpeeds[1]  = 0;
  DbPtr->LinkSpeeds[2]  = 0;
  DbPtr->LinkSpeeds[3]  = 0;

  DbPtr->NvCount        = MAX_EEPROM_LEN;
  DbPtr->NvWidth        = 4;
  DbPtr->MediaHeaderLen = PXE_MAC_HEADER_LEN_ETHER;
  DbPtr->HWaddrLen      = PXE_HWADDR_LEN_ETHER;
  if (AdapterInfo->PartitionPfNumber[0] == AdapterInfo->Function) {
    DbPtr->MCastFilterCnt = MAX_MCAST_ADDRESS_CNT;
  } else {
    DbPtr->MCastFilterCnt = 0;
  }

  DbPtr->TxBufCnt       = AdapterInfo->TxRxDescriptorCount;
  DbPtr->TxBufSize      = 0;
  DbPtr->RxBufCnt       = AdapterInfo->TxRxDescriptorCount;
  DbPtr->RxBufSize      = 0;

  DbPtr->IFtype = PXE_IFTYPE_ETHERNET;
  DbPtr->SupportedDuplexModes = PXE_DUPLEX_ENABLE_FULL_SUPPORTED;
  DbPtr->SupportedLoopBackModes = 0;

  CdbPtr->StatFlags |= (PXE_STATFLAGS_CABLE_DETECT_SUPPORTED |
                        PXE_STATFLAGS_GET_STATUS_NO_MEDIA_SUPPORTED);

  CdbPtr->StatFlags |= PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode = PXE_STATCODE_SUCCESS;
}

/** This routine is used to retrieve the configuration information about the NIC being controlled by
  this driver.

  This will fill in data in the Data Block structure that is pointed to by the caller's CdbPtr->DBaddr.
  The fields filled in are as follows:
  DbPtr->pci.BusType, DbPtr->pci.Bus, DbPtr->pci.Device, and DbPtr->pci.
  In addition, the DbPtr->pci.Config.Dword[0-63] grabs a copy of this NIC's PCI configuration space.

  @param[in]   CdbPtr        Pointer to the command descriptor block.
  @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

  @retval      None
**/
VOID
UndiGetConfigInfo (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_DB_GET_CONFIG_INFO  *DbPtr;

  DEBUGPRINT (DECODE, ("UndiGetConfigInfo\n"));
  DEBUGWAIT (DECODE);
  DbPtr               = (PXE_DB_GET_CONFIG_INFO *) (UINTN) (CdbPtr->DBaddr);

  DbPtr->pci.BusType  = PXE_BUSTYPE_PCI;
  DbPtr->pci.Bus      = (UINT16) AdapterInfo->Bus;
  DbPtr->pci.Device   = (UINT8) AdapterInfo->Device;
  DbPtr->pci.Function = (UINT8) AdapterInfo->Function;
  DEBUGPRINT (
    DECODE,
    ("Bus %x, Device %x, Function %x\n",
    AdapterInfo->Bus,
    AdapterInfo->Device,
    AdapterInfo->Function)
  );

  CopyMem (
    DbPtr->pci.Config.Dword,
    &AdapterInfo->PciConfig,
    MAX_PCI_CONFIG_LEN * sizeof (UINT32)
  );

  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode  = PXE_STATCODE_SUCCESS;
}

/** This routine resets the network adapter and initializes the 10-Gigabit UNDI using the parameters
   supplied in the CPB.

   This command must be issued before the network adapter can be setup to transmit and receive packets.
   Once the memory requirements of the UNDI are obtained by using the GetInitInfo command, a block
   of non-swappable memory may need to be allocated.  The address of this memory must be passed to
   UNDI during the Initialize in the CPB.  This memory is used primarily for transmit and receive buffers.
   The fields CableDetect, LinkSpeed, Duplex, LoopBack, MemoryPtr, and MemoryLength are set with
   information that was passed in the CPB and the NIC is initialized.
   If the NIC initialization fails, the CdbPtr->StatFlags are updated with PXE_STATFLAGS_COMMAND_FAILED
   Otherwise, AdapterInfo->State is updated with PXE_STATFLAGS_GET_STATE_INITIALIZED showing the state of
   the UNDI is now initialized.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval      None
-**/
VOID
UndiInitialize (
  IN  PXE_CDB      *CdbPtr,
  I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_CPB_INITIALIZE  *CpbPtr;
  PXE_DB_INITIALIZE   *DbPtr;

  UINT8                i = 0;

  if (AdapterInfo->DriverBusy) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiInitialize called when driver busy\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_BUSY;
    return;
  }

  DEBUGPRINT (DECODE, ("UndiInitialize\n"));
  DEBUGWAIT (DECODE);

  if ((CdbPtr->OpFlags != PXE_OPFLAGS_INITIALIZE_DETECT_CABLE)
    && (CdbPtr->OpFlags != PXE_OPFLAGS_INITIALIZE_DO_NOT_DETECT_CABLE))
  {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiInitialize invalid CDB\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    return;
  }

  // check if it is already initialized
  if (AdapterInfo->State == PXE_STATFLAGS_GET_STATE_INITIALIZED) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiInitialize already initialized\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_ALREADY_INITIALIZED;
    return;
  }

  CpbPtr  = (PXE_CPB_INITIALIZE *) (UINTN) CdbPtr->CPBaddr;
  DbPtr   = (PXE_DB_INITIALIZE *) (UINTN) CdbPtr->DBaddr;

  if (CdbPtr->OpFlags == (UINT16) PXE_OPFLAGS_INITIALIZE_DO_NOT_DETECT_CABLE) {
    AdapterInfo->CableDetect = (UINT8) 0;
  } else {
    AdapterInfo->CableDetect = (UINT8) 1;
  }
  DEBUGPRINT (DECODE, ("CdbPtr->OpFlags = %X\n", CdbPtr->OpFlags));
  AdapterInfo->LinkSpeed  = (UINT16) CpbPtr->LinkSpeed;
  AdapterInfo->DuplexMode = CpbPtr->DuplexMode;
  AdapterInfo->LoopBack   = CpbPtr->LoopBackMode;

  DEBUGPRINT (DECODE, ("CpbPtr->TxBufCnt = %X\n", CpbPtr->TxBufCnt));
  DEBUGPRINT (DECODE, ("CpbPtr->TxBufSize = %X\n", CpbPtr->TxBufSize));
  DEBUGPRINT (DECODE, ("CpbPtr->RxBufCnt = %X\n", CpbPtr->RxBufCnt));
  DEBUGPRINT (DECODE, ("CpbPtr->RxBufSize = %X\n", CpbPtr->RxBufSize));

  CdbPtr->StatCode = (PXE_STATCODE) I40eInitialize (AdapterInfo);
  CdbPtr->StatCode = PXE_STATCODE_SUCCESS;


  // Fill in the DpPtr with how much memory we used to conform to the UNDI spec.
  DbPtr->MemoryUsed = 0;
  DbPtr->TxBufCnt   = AdapterInfo->TxRxDescriptorCount;
  DbPtr->RxBufCnt   = AdapterInfo->TxRxDescriptorCount;

  if (CdbPtr->StatCode != PXE_STATCODE_SUCCESS) {
    DEBUGPRINT (DECODE, ("Initialize failed! Statcode = %X\n", CdbPtr->StatCode));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  } else {
    CdbPtr->StatFlags   = PXE_STATFLAGS_COMMAND_COMPLETE;
    AdapterInfo->State  = PXE_STATFLAGS_GET_STATE_INITIALIZED;
  }

  if (AdapterInfo->CableDetect != 0) {
    BOOLEAN           LinkUp;
    UINT8             Timeout = 10;
#define I40E_GLGEN_GPIO_STAT_GPIO_VALUE_6        0x40
#define I40E_GLGEN_GPIO_STAT_GPIO_VALUE_10       0x400

    if (AdapterInfo->Hw.device_id == I40E_DEV_ID_25G_B) {
      AdapterInfo->WaitingForLinkUp = TRUE;
      Timeout = 40;
    } else if (AdapterInfo->Hw.device_id == I40E_DEV_ID_25G_SFP28) {
      UINT32 GpioRegValue;
      Timeout = 40;
      GpioRegValue = rd32 (&AdapterInfo->Hw, I40E_GLGEN_GPIO_STAT);
      if (AdapterInfo->Function == 0) {
        AdapterInfo->WaitingForLinkUp = !(I40E_GLGEN_GPIO_STAT_GPIO_VALUE_10 & GpioRegValue);
      } else if (AdapterInfo->Function == 1) {
        AdapterInfo->WaitingForLinkUp = !(I40E_GLGEN_GPIO_STAT_GPIO_VALUE_6 & GpioRegValue);
      }
    }
    switch (AdapterInfo->WaitingForLinkUp) {
    case TRUE:
      for (i = 0; i < Timeout; i++)
      {
        if (IsLinkUp (AdapterInfo)) {
          AdapterInfo->WaitingForLinkUp = FALSE;
          break;
        }
        DelayInMicroseconds (AdapterInfo, 500 * 1000);
      }
    case FALSE:
      LinkUp = IsLinkUp (AdapterInfo);
      DEBUGPRINT (I40E, ("Link Up:%X\n", LinkUp));
      if (!LinkUp) {
        DEBUGPRINT (I40E, ("Link down\n"));
        CdbPtr->StatFlags |= PXE_STATFLAGS_INITIALIZED_NO_MEDIA;
        CdbPtr->StatCode    = PXE_STATCODE_NOT_STARTED;
        AdapterInfo->State  = PXE_STATFLAGS_GET_STATE_STARTED;
      }
      break;
    default:
      break;
    }
  }
}

/** This routine resets the network adapter and initializes the 10-Gigabit UNDI using the
   parameters supplied in the CPB.

   The transmit and receive queues are emptied and any pending interrupts are cleared.
   If the NIC reset fails, the CdbPtr->StatFlags are updated with PXE_STATFLAGS_COMMAND_FAILED

   @param[in]   CdbPtr         Pointer to the command descriptor block.
   @param[in]   AdapterInfo    Pointer to the NIC data structure information which the
                               UNDI driver is layering on..

   @retval      None
**/
VOID
UndiReset (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  DEBUGPRINT (DECODE, ("UndiReset\n"));
  DEBUGWAIT (DECODE);

  if (AdapterInfo->DriverBusy) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiReset called when driver busy\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_BUSY;
    return;
  }

  if (CdbPtr->OpFlags != PXE_OPFLAGS_NOT_USED
    && CdbPtr->OpFlags != PXE_OPFLAGS_RESET_DISABLE_INTERRUPTS
    && CdbPtr->OpFlags != PXE_OPFLAGS_RESET_DISABLE_FILTERS)
  {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiReset, invalid CDB\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    return;
  }

  CdbPtr->StatCode = (UINT16) I40eReset (AdapterInfo);

  if (CdbPtr->StatCode != PXE_STATCODE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiReset failed\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  } else {
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  }
}

/** This routine resets the network adapter and leaves it in a safe state for another
   driver to initialize.

   Any pending transmits or receives are lost.  Receive filters and external
   interrupt enables are disabled.  Once the UNDI has been shutdown, it can then be stopped
   or initialized again.
   If the NIC reset fails, the CdbPtr->StatFlags are updated with PXE_STATFLAGS_COMMAND_FAILED
   Otherwise, AdapterInfo->State is updated with PXE_STATFLAGS_GET_STATE_STARTED showing
   the state of the NIC as being started.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval      None
**/
VOID
UndiShutdown (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  // do the shutdown stuff here
  DEBUGPRINT (DECODE, ("UndiShutdown\n"));
  DEBUGWAIT (DECODE);

  if (AdapterInfo->DriverBusy) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiShutdown called when driver busy\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_BUSY;
    return;
  }

  CdbPtr->StatCode = (UINT16) I40eShutdown (AdapterInfo);

  if (CdbPtr->StatCode != PXE_STATCODE_SUCCESS) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiShutdown failed\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  } else {
    AdapterInfo->State  = PXE_STATFLAGS_GET_STATE_STARTED;
    CdbPtr->StatFlags   = PXE_STATFLAGS_COMMAND_COMPLETE;
  }
}

/** This routine can be used to read and/or change the current external interrupt enable
   settings.

   Disabling an external interrupt enable prevents and external (hardware)
   interrupt from being signaled by the network device.  Internally the interrupt events
   can still be polled by using the UNDI_GetState command.
   The resulting information on the interrupt state will be passed back in the CdbPtr->StatFlags.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on.

   @retval      None
**/
VOID
UndiInterrupt (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  CdbPtr->StatCode  = PXE_STATCODE_UNSUPPORTED;
}

#if (DBG_LVL & DECODE)
/** Debug function to read receive filter flags and multicast addresses.

   @param[in]   CdbPtr        Pointer to the command descriptor block.

   @retval      None
**/
VOID
DebugRcvFilter (
  IN PXE_CDB          *CdbPtr
  )
{
  UINTN i;
  UINT8* Byte;

  DEBUGPRINT (DECODE, ("OPFLAGS = %04x\n", CdbPtr->OpFlags));

  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ENABLE) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_ENABLE\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_DISABLE) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_DISABLE\n"));
  }
  if ((CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_OPMASK) == PXE_OPFLAGS_RECEIVE_FILTER_READ) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_READ\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_UNICAST) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_UNICAST\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_BROADCAST\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_PROMISCUOUS\n"));
  }
  if (CdbPtr->OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST\n"));
  }
  DEBUGWAIT (DECODE);

  Byte = (UINT8 *) (UINTN) CdbPtr->CPBaddr;

  DEBUGPRINT (DECODE, ("\nFound %d multicast addresses\n", CdbPtr->CPBsize / PXE_MAC_LENGTH));
  for (i = 0; i < CdbPtr->CPBsize; i++) {
    if (i % PXE_MAC_LENGTH == 0) {
      DEBUGPRINT (DECODE, ("\nMcast Addr %d:", i / PXE_MAC_LENGTH));
    }
    DEBUGPRINT (DECODE, ("%02x ", Byte[i]));
  }
}
#endif /* (DBG_LVL & DECODE) */

/** This routine is used to read and change receive filters and, if supported, read
   and change multicast MAC address filter list.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on..

   @retval     None
**/
VOID
UndiRecFilter (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{

  UINT16                  NewFilter;
  UINT16                  OpFlags;

  DEBUGPRINT (DECODE, ("UndiRecFilter\n"));

  OpFlags   = CdbPtr->OpFlags;
  NewFilter = (UINT16) (OpFlags & 0x1F);

#if (DBG_LVL & DECODE)
  DebugRcvFilter (CdbPtr);
#endif /* (DBG_LVL & DECODE) */

  switch (OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_OPMASK) {
  case PXE_OPFLAGS_RECEIVE_FILTER_READ:
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_READ\n"));

    // not expecting a cpb, not expecting any filter bits
    if ((NewFilter != 0) || (CdbPtr->CPBsize != 0)) {
      goto BadCdb;
    }

    if (CdbPtr->DBsize >= AdapterInfo->Vsi.CurrentMcastList.Length * PXE_MAC_LENGTH) {
      DEBUGPRINT (DECODE, ("Copying Mcast ADDR list to DBAddr\n"));
      CopyMem (
        (VOID *) (UINTN) CdbPtr->DBaddr,
        AdapterInfo->Vsi.CurrentMcastList.McAddr,
        AdapterInfo->Vsi.CurrentMcastList.Length * PXE_MAC_LENGTH
      );
    } else {
      DEBUGPRINT (CRITICAL, ("DBsize too small for MC List: %d\n", CdbPtr->DBaddr));
    }
    break;
  case PXE_OPFLAGS_RECEIVE_FILTER_ENABLE:

    // there should be at least one other filter bit set.
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_ENABLE\n"));

    if (NewFilter == 0) {

      // nothing to enable
      goto BadCdb;
    }

    if (CdbPtr->CPBsize != 0) {

      // this must be a multicast address list!
      // don't accept the list unless selective_mcast is set
      // don't accept confusing mcast settings with this
      if (((NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) == 0)
        || ((NewFilter & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) != 0))
      {
        goto BadCdb;
      }
    }

    // check selective mcast case enable case
    if ((OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_FILTERED_MULTICAST) != 0) {
      if (((OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST) != 0)
        || ((OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_ALL_MULTICAST) != 0))
      {
        goto BadCdb;
      }

      // if no cpb, make sure we have an old list
      if ((CdbPtr->CPBsize == 0)
        && (AdapterInfo->Vsi.CurrentMcastList.Length == 0))
      {
        goto BadCdb;
      }
    }
    NewFilter |= PXE_OPFLAGS_RECEIVE_FILTER_UNICAST;
    I40eSetFilter (AdapterInfo, NewFilter);

    ZeroMem (AdapterInfo->Vsi.McastListToProgram.McAddr, MAX_MCAST_ADDRESS_CNT * PXE_MAC_LENGTH);
    CopyMem (
      AdapterInfo->Vsi.McastListToProgram.McAddr,
      (VOID *) (UINTN) CdbPtr->CPBaddr,
      CdbPtr->CPBsize
    );
    AdapterInfo->Vsi.McastListToProgram.Length = CdbPtr->CPBsize / PXE_MAC_LENGTH;
    I40eSetMcastList (AdapterInfo);
    break;

  case PXE_OPFLAGS_RECEIVE_FILTER_DISABLE:

    // mcast list not expected, i.e. no cpb here!
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_DISABLE\n"));

    if (CdbPtr->CPBsize != PXE_CPBSIZE_NOT_USED) {
      goto BadCdb;    // db with all_multi??
    }
    I40eClearFilter (AdapterInfo, NewFilter);
    break;

  default:
    goto BadCdb;
    break;
  }

  if ((OpFlags & PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST) != 0) {
    DEBUGPRINT (DECODE, ("PXE_OPFLAGS_RECEIVE_FILTER_RESET_MCAST_LIST\n"));

    // Setting Mcast list length to 0 will disable receive of multicast packets
    AdapterInfo->Vsi.McastListToProgram.Length = 0;
    I40eSetMcastList (AdapterInfo);
  }

  DEBUGPRINT (DECODE, ("AdapterInfo->RxFilter = %04x\n", AdapterInfo->RxFilter));
  DEBUGWAIT (DECODE);

  if (AdapterInfo->ReceiveStarted) {
    CdbPtr->StatFlags |= (AdapterInfo->RxFilter | PXE_STATFLAGS_COMMAND_COMPLETE);
    DEBUGPRINT (RXFILTER, ("Returning CdbPtr->StatFlags = %x\n", CdbPtr->StatFlags));
  } else {
    DEBUGPRINT (CRITICAL, ("Returning CdbPtr->StatFlags = 0\n"));
  }

  return;

BadCdb:
  DEBUGPRINT (CRITICAL, ("ERROR: UndiRecFilter: invalid CDB\n"));
  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
}

/** This routine is used to get the current station and broadcast MAC addresses,
   and to change the current station MAC address.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on.

   @retval      None
**/
VOID
UndiStnAddr (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{

  PXE_CPB_STATION_ADDRESS *CpbPtr;
  PXE_DB_STATION_ADDRESS  *DbPtr;

  DbPtr = NULL;
  DEBUGPRINT (DECODE, ("\n"));

  if (CdbPtr->OpFlags == PXE_OPFLAGS_STATION_ADDRESS_RESET) {

    // Check if there is a need to reset the address
    if (CompareMem (
          AdapterInfo->Hw.mac.addr,
          AdapterInfo->Hw.mac.perm_addr,
          PXE_HWADDR_LEN_ETHER
        ) != 0)
    {
      CopyMem (
        AdapterInfo->Hw.mac.addr,
        AdapterInfo->Hw.mac.perm_addr,
        PXE_HWADDR_LEN_ETHER
      );
      i40e_aq_mac_address_write (
        &AdapterInfo->Hw,
        I40E_AQC_WRITE_TYPE_LAA_ONLY,
        AdapterInfo->Hw.mac.perm_addr,
        NULL
      );
    }
  }

  if (CdbPtr->CPBaddr != (UINT64) 0) {
    CpbPtr                        = (PXE_CPB_STATION_ADDRESS *) (UINTN) (CdbPtr->CPBaddr);
    AdapterInfo->MacAddrOverride  = TRUE;

    // configure the new address
    CopyMem (AdapterInfo->Hw.mac.addr, CpbPtr->StationAddr, PXE_HWADDR_LEN_ETHER);

    DEBUGPRINT (
      DECODE,
      ("CpbPtr->StationAddr = %02x:%02x:%02x:%02x:%02x:%02x\n",
      CpbPtr->StationAddr[0],
      CpbPtr->StationAddr[1],
      CpbPtr->StationAddr[2],
      CpbPtr->StationAddr[3],
      CpbPtr->StationAddr[4],
      CpbPtr->StationAddr[5])
    );

    i40e_aq_mac_address_write (
      &AdapterInfo->Hw,
      I40E_AQC_WRITE_TYPE_LAA_ONLY,
      AdapterInfo->Hw.mac.addr,
      NULL
    );
  }

  if (CdbPtr->DBaddr != (UINT64) 0) {
    DbPtr = (PXE_DB_STATION_ADDRESS *) (UINTN) (CdbPtr->DBaddr);

    // fill it with the new values
    ZeroMem (DbPtr->StationAddr, PXE_MAC_LENGTH);
    ZeroMem (DbPtr->PermanentAddr, PXE_MAC_LENGTH);
    ZeroMem (DbPtr->BroadcastAddr, PXE_MAC_LENGTH);
    CopyMem (DbPtr->StationAddr, AdapterInfo->Hw.mac.addr, PXE_HWADDR_LEN_ETHER);
    CopyMem (DbPtr->PermanentAddr, AdapterInfo->Hw.mac.perm_addr, PXE_HWADDR_LEN_ETHER);
    CopyMem (DbPtr->BroadcastAddr, AdapterInfo->BroadcastNodeAddress, PXE_MAC_LENGTH);
  }

  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode  = PXE_STATCODE_SUCCESS;
}

/** This routine is used to read and clear the NIC traffic statistics.  This command is supported
   only if the !PXE structure's Implementation flags say so.

   Results will be parsed out in the following manner:
   CdbPtr->DBaddr.Data[0]   R  Total Frames (Including frames with errors and dropped frames)
   CdbPtr->DBaddr.Data[1]   R  Good Frames (All frames copied into receive buffer)
   CdbPtr->DBaddr.Data[2]   R  Undersize Frames (Frames below minimum length for media <64 for ethernet)
   CdbPtr->DBaddr.Data[4]   R  Dropped Frames (Frames that were dropped because receive buffers were full)
   CdbPtr->DBaddr.Data[8]   R  CRC Error Frames (Frames with alignment or CRC errors)
   CdbPtr->DBaddr.Data[A]   T  Total Frames (Including frames with errors and dropped frames)
   CdbPtr->DBaddr.Data[B]   T  Good Frames (All frames copied into transmit buffer)
   CdbPtr->DBaddr.Data[C]   T  Undersize Frames (Frames below minimum length for media <64 for ethernet)
   CdbPtr->DBaddr.Data[E]   T  Dropped Frames (Frames that were dropped because of collisions)
   CdbPtr->DBaddr.Data[14]  T  Total Collision Frames (Total collisions on this subnet)

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiStatistics (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  DEBUGPRINT (DECODE, ("ERROR: UndiStatistics called, but unsupported\n"));
  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  CdbPtr->StatCode  = PXE_STATCODE_UNSUPPORTED;
}

/** This routine is used to translate a multicast IP address to a multicast MAC address.

   This results in a MAC address composed of 25 bits of fixed data with the upper 23 bits of the IP
   address being appended to it.  Results passed back in the equivalent of CdbPtr->DBaddr->MAC[0-5].

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiIp2Mac (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_CPB_MCAST_IP_TO_MAC *CpbPtr;
  PXE_DB_MCAST_IP_TO_MAC  *DbPtr;
  UINT32                  IpAddr;
  UINT8                   *TmpPtr;

  DEBUGPRINT (DECODE, ("UndiIp2Mac\n"));

  CpbPtr  = (PXE_CPB_MCAST_IP_TO_MAC *) (UINTN) CdbPtr->CPBaddr;
  DbPtr   = (PXE_DB_MCAST_IP_TO_MAC *) (UINTN) CdbPtr->DBaddr;

  if ((CdbPtr->OpFlags & PXE_OPFLAGS_MCAST_IPV6_TO_MAC) != 0) {
    UINT8 *IpV6Ptr;

    IpV6Ptr    = (UINT8 *) &CpbPtr->IP.IPv6;

    DbPtr->MAC[0] = 0x33;
    DbPtr->MAC[1] = 0x33;
    DbPtr->MAC[2] = *(IpV6Ptr + 12);
    DbPtr->MAC[3] = *(IpV6Ptr + 13);
    DbPtr->MAC[4] = *(IpV6Ptr + 14);
    DbPtr->MAC[5] = *(IpV6Ptr + 15);
    return;
  }

  IpAddr        = CpbPtr->IP.IPv4;
  TmpPtr        = (UINT8 *) (&IpAddr);

  DbPtr->MAC[0] = 0x01;
  DbPtr->MAC[1] = 0x00;
  DbPtr->MAC[2] = 0x5e;
  DbPtr->MAC[3] = (UINT8) (TmpPtr[1] & 0x7f);
  DbPtr->MAC[4] = (UINT8) TmpPtr[2];
  DbPtr->MAC[5] = (UINT8) TmpPtr[3];
}

/** This routine is used to read and write non-volatile storage on the NIC (if supported).  The NVRAM
   could be EEPROM, FLASH, or battery backed RAM.

   This is an optional function according to the UNDI specification  (or will be......)

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiNvData (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  DEBUGPRINT (DECODE, ("ERROR: UndiNvData called, but unsupported\n"));
  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  CdbPtr->StatCode  = PXE_STATCODE_UNSUPPORTED;
}

/** This routine returns the current interrupt status and/or the transmitted buffer addresses.

   If the current interrupt status is returned, pending interrupts will be acknowledged by this
   command.  Transmitted buffer addresses that are written to the DB are removed from the transmit
   buffer queue.
   Normally, this command would be polled with interrupts disabled.
   The transmit buffers are returned in CdbPtr->DBaddr->TxBufer[0 - NumEntries].
   The interrupt status is returned in CdbPtr->StatFlags.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the 10-Gigabit
                             UNDI driver is layering on..

   @retval   None
**/
VOID
UndiStatus (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_DB_GET_STATUS           *DbPtr;
  UINT16                      NumEntries;
  union i40e_16byte_rx_desc   *RxPtr;
  UINT32                      RxStatus;
  UINT64                      DescQWord;
  UINT16                      RxPacketLength;
  UINT32                      RegVal;

  RegVal = 0;
  // The function is called in timer interrupt.
  // Keep this in mind when you touch Hw resources like AQ, registers, e.t.c.

  DEBUGPRINT (DECODE, ("UndiStatus\n"));

  if (AdapterInfo->DriverBusy) {

    //DEBUGPRINT (CRITICAL, ("ERROR: UndiStatus called when driver busy\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_BUSY;
    return;
  }

  // Cache interrupt status in driver's private structure.
  RegVal = I40eRead32 (AdapterInfo, I40E_PFINT_ICR0);
  AdapterInfo->IntStatus |= RegVal;

  // Check whether Link Status Event occurred since last call.
  if ((RegVal & I40E_PFINT_ICR0_ADMINQ_MASK) != 0) {
    AdapterInfo->MediaStatusChecked = FALSE;
  }

  // If the size of the DB is not large enough to store at least one 64 bit
  // complete transmit buffer address and size of the next available receive
  // packet we will return an error.  Per E.4.16 of the EFI spec the DB should
  // have enough space for at least 1 completed transmit buffer.
  if (CdbPtr->DBsize < (sizeof (UINT64) * 2)) {
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    DEBUGPRINT (CRITICAL, ("ERROR: UndiStatus invalid CDB\n"));
    if ((CdbPtr->OpFlags & PXE_OPFLAGS_GET_TRANSMITTED_BUFFERS) != 0) {
      CdbPtr->StatFlags |= PXE_STATFLAGS_GET_STATUS_NO_TXBUFS_WRITTEN;
    }

    return;
  }

  DbPtr = (PXE_DB_GET_STATUS *) (UINTN) CdbPtr->DBaddr;

  // Fill in size of next available receive packet and
  // reserved field in caller's DB storage.
  RxPtr = I40E_RX_DESC (&AdapterInfo->Vsi.RxRing, AdapterInfo->Vsi.RxRing.NextToUse);

  DescQWord = RxPtr->wb.qword1.status_error_len;
  RxStatus = (UINT32) ((DescQWord & I40E_RXD_QW1_STATUS_MASK) >> I40E_RXD_QW1_STATUS_SHIFT);

  if ((RxStatus & (1 << I40E_RX_DESC_STATUS_DD_SHIFT)) != 0) {
    RxPacketLength = (UINT16) ((DescQWord & I40E_RXD_QW1_LENGTH_PBUF_MASK) >> I40E_RXD_QW1_LENGTH_PBUF_SHIFT);
    DEBUGPRINT (DECODE, ("Get Status->We have a Rx Frame at %x\n", AdapterInfo->Vsi.RxRing.NextToUse));
    DEBUGPRINT (DECODE, ("Frame length = %X  ", RxPacketLength));
    DbPtr->RxFrameLen = RxPacketLength;
  } else {
    DbPtr->RxFrameLen = 0;
  }

  // Fill in the completed transmit buffer addresses so they can be freed by
  // the calling application or driver
  if ((CdbPtr->OpFlags & PXE_OPFLAGS_GET_TRANSMITTED_BUFFERS) != 0) {

    // Calculate the number of entries available in the DB to save the addresses
    // of completed transmit buffers.
    NumEntries = (UINT16) ((CdbPtr->DBsize - sizeof (UINT64)) / sizeof (UINT64));
    DEBUGPRINT (DECODE, ("CdbPtr->DBsize = %d\n", CdbPtr->DBsize));
    DEBUGPRINT (DECODE, ("NumEntries in DbPtr = %d\n", NumEntries));

    NumEntries = I40eFreeTxBuffers (AdapterInfo, NumEntries, DbPtr->TxBuffer);
    if (NumEntries == 0) {
      CdbPtr->StatFlags |= PXE_STATFLAGS_GET_STATUS_NO_TXBUFS_WRITTEN;
    }

    // The receive buffer size and reserved fields take up the first 64 bits of the DB
    // The completed transmit buffers take up the rest
    CdbPtr->DBsize = (UINT16) (sizeof (UINT64) + NumEntries * sizeof (UINT64));
    DEBUGPRINT (DECODE, ("Return DBsize = %d\n", CdbPtr->DBsize));
  }


  if ((CdbPtr->OpFlags & PXE_OPFLAGS_GET_INTERRUPT_STATUS) != 0) {
    if (AdapterInfo->IntStatus & I40E_PFINT_ICR0_QUEUE_0_MASK) {
      CdbPtr->StatFlags |= PXE_STATFLAGS_GET_STATUS_RECEIVE;
    }
    if (AdapterInfo->IntStatus & I40E_PFINT_ICR0_QUEUE_1_MASK) {
      CdbPtr->StatFlags |= PXE_STATFLAGS_GET_STATUS_TRANSMIT;
    }

    // Clear interrupt cached status
    AdapterInfo->IntStatus = 0;
  }

  // Return current media status
  if ((CdbPtr->OpFlags & PXE_OPFLAGS_GET_MEDIA_STATUS) != 0) {

    if (!AdapterInfo->MediaStatusChecked) {
      // Skip checking if AQ is currently in use
      if (EfiAcquireLockOrFail (&AdapterInfo->Hw.aq.asq_spinlock.SpinLock) == EFI_SUCCESS) {
        enum i40e_admin_queue_err LastStatus = AdapterInfo->Hw.aq.asq_last_status;

        EfiReleaseLock (&AdapterInfo->Hw.aq.asq_spinlock.SpinLock);
        AdapterInfo->MediaStatusChecked = TRUE;
        if (!IsLinkUp (AdapterInfo)) {
          AdapterInfo->LastMediaStatus = FALSE;
          CdbPtr->StatFlags |= PXE_STATFLAGS_GET_STATUS_NO_MEDIA;
        } else {
          AdapterInfo->LastMediaStatus = TRUE;
        }

        // Restore the last status
        AdapterInfo->Hw.aq.asq_last_status = LastStatus;
      }
    }

    // Return previous media status
    if (!AdapterInfo->LastMediaStatus) {
      CdbPtr->StatFlags |= PXE_STATFLAGS_GET_STATUS_NO_MEDIA;
    }
  }

  CdbPtr->StatFlags |= PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode = PXE_STATCODE_SUCCESS;
}

/** This routine is used to fill media header(s) in transmit packet(s).

   Copies the MAC address into the media header whether it is dealing
   with fragmented or non-fragmented packets.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                             UNDI driver is layering on.

   @retval      None
**/
VOID
UndiFillHeader (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  PXE_CPB_FILL_HEADER             *Cpb;
  PXE_CPB_FILL_HEADER_FRAGMENTED  *Cpbf;
  ETHER_HEADER                    *MacHeader;
  UINTN                           i;

  DEBUGPRINT (DECODE, ("UndiFillHeader\n"));
  if (CdbPtr->CPBsize == PXE_CPBSIZE_NOT_USED) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiFillHeader invalid CDB\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    return;
  }

  if ((CdbPtr->OpFlags & PXE_OPFLAGS_FILL_HEADER_FRAGMENTED) != 0) {
    Cpbf = (PXE_CPB_FILL_HEADER_FRAGMENTED *) (UINTN) CdbPtr->CPBaddr;

    // assume 1st fragment is big enough for the mac header
    if ((Cpbf->FragCnt == 0)
      || (Cpbf->FragDesc[0].FragLen < PXE_MAC_HEADER_LEN_ETHER))
    {
      DEBUGPRINT (CRITICAL, ("ERROR: UndiFillHeader, fragment too small for MAC header\n"));
      CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
      CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
      return;
    }

    MacHeader = (ETHER_HEADER *) (UINTN) Cpbf->FragDesc[0].FragAddr;

    // we don't swap the protocol bytes
    MacHeader->Type = Cpbf->Protocol;

    DEBUGPRINT (DECODE, ("MacHeader->SrcAddr = "));
    for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
      MacHeader->DestAddr[i] = Cpbf->DestAddr[i];
      MacHeader->SrcAddr[i] = Cpbf->SrcAddr[i];
      DEBUGPRINT (DECODE, ("%x ", MacHeader->SrcAddr[i]));
    }

    DEBUGPRINT (DECODE, ("\n"));
  } else {
    Cpb       = (PXE_CPB_FILL_HEADER *) (UINTN) CdbPtr->CPBaddr;

    MacHeader = (ETHER_HEADER *) (UINTN) Cpb->MediaHeader;

    // we don't swap the protocol bytes
    MacHeader->Type = Cpb->Protocol;

    DEBUGPRINT (DECODE, ("MacHeader->SrcAddr = "));
    for (i = 0; i < PXE_HWADDR_LEN_ETHER; i++) {
      MacHeader->DestAddr[i] = Cpb->DestAddr[i];
      MacHeader->SrcAddr[i]  = Cpb->SrcAddr[i];
      DEBUGPRINT (DECODE, ("%x ", MacHeader->SrcAddr[i]));
    }

    DEBUGPRINT (DECODE, ("\n"));
  }

  DEBUGWAIT (DECODE);
}

/** This routine is used to place a packet into the transmit queue.

   The data buffers given to this command are to be considered locked and the application or
   network driver loses ownership of these buffers and must not free or relocate them until
   the ownership returns.
   When the packets are transmitted, a transmit complete interrupt is generated (if interrupts
   are disabled, the transmit interrupt status is still set and can be checked using the UNDI_Status
   command.
   Some implementations and adapters support transmitting multiple packets with one transmit
   command.  If this feature is supported, the transmit CPBs can be linked in one transmit
   command.
   All UNDIs support fragmented frames, now all network devices or protocols do.  If a fragmented
   frame CPB is given to UNDI and the network device does not support fragmented frames
   (see !PXE.Implementation flag), the UNDI will have to copy the fragments into a local buffer
   before transmitting.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiTransmit (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  if (CdbPtr->CPBsize == PXE_CPBSIZE_NOT_USED) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiTransmit invalid CDB\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    return;
  }

  if (AdapterInfo->DriverBusy) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiTransmit called when driver busy\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_BUSY;
    return;
  }

  CdbPtr->StatCode = (PXE_STATCODE) I40eTransmit (AdapterInfo, CdbPtr->CPBaddr, CdbPtr->OpFlags);

  if (CdbPtr->StatCode == PXE_STATCODE_SUCCESS) {
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  } else {
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  }
}

/** When the network adapter has received a frame, this command is used to copy the frame
   into the driver/application storage location.

   Once a frame has been copied, it is removed from the receive queue.

   @param[in]   CdbPtr        Pointer to the command descriptor block.
   @param[in]   AdapterInfo   Pointer to the NIC data structure information which the
                              UNDI driver is layering on..

   @retval      None
**/
VOID
UndiReceive (
  IN PXE_CDB          *CdbPtr,
  IN I40E_DRIVER_DATA *AdapterInfo
  )
{
  // check if RU has started...
  if (!AdapterInfo->ReceiveStarted) {
    DEBUGPRINT (CRITICAL, ("ERROR: UndiReceive called, but receive not started\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_NOT_INITIALIZED;
    return;
  }

  if (AdapterInfo->DriverBusy) {
    DEBUGPRINT (CRITICAL, ("Driver busy\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_BUSY;
    return;
  }

  CdbPtr->StatCode = (UINT16) I40eReceive (
                                AdapterInfo,
                                (PXE_CPB_RECEIVE *) (UINTN) CdbPtr->CPBaddr,
                                (PXE_DB_RECEIVE *) (UINTN) CdbPtr->DBaddr
                              );

  if (CdbPtr->StatCode == PXE_STATCODE_SUCCESS) {
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  } else {
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  }
}

/** This is the main SW UNDI API entry using the newer NII protocol.

   The parameter passed in is a 64 bit flat model virtual address of the Cdb.  We then
   jump into the service routine pointed to by the Api_Table[OpCode].

   @param[in]   Cdb    Pointer to the command descriptor block.

   @retval      None
**/
VOID
EFIAPI
UndiApiEntry (
  IN UINT64 Cdb
  )
{
  PXE_CDB           *CdbPtr;
  I40E_DRIVER_DATA  *AdapterInfo;
  UNDI_CALL_TABLE   *TabPtr;

  if (Cdb == (UINT64) 0) {
    DEBUGPRINT (CRITICAL, ("ERROR: ApiEntry invalid CDB\n"));
    return;
  }

  CdbPtr = (PXE_CDB *) (UINTN) Cdb;

  if (CdbPtr->IFnum > mPxe31->IFcnt) {
    DEBUGPRINT (CRITICAL, ("Invalid IFnum %d\n", CdbPtr->IFnum));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
    return;
  }

  AdapterInfo               = &(mUndi32DeviceList[CdbPtr->IFnum]->NicInfo);

  // Check if InitUndiNotifyExitBs was called before
  if (mExitBootServicesTriggered) {
    DEBUGPRINT (CRITICAL, ("Exit Boot Services triggered prior to entering UNDI API entry.\n"));
    CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
    CdbPtr->StatCode  = PXE_STATCODE_NOT_INITIALIZED;
    return;
  }

  AdapterInfo->VersionFlag  = 0x31; // entering from new entry point

  // check the OPCODE range
  if ((CdbPtr->OpCode > PXE_OPCODE_LAST_VALID)
    || (CdbPtr->StatCode != PXE_STATCODE_INITIALIZE)
    || (CdbPtr->StatFlags != PXE_STATFLAGS_INITIALIZE))
  {
    DEBUGPRINT (DECODE, ("Invalid StatCode, OpCode, or StatFlags.\n", CdbPtr->IFnum));
    goto BadCdb;
  }

  if (CdbPtr->CPBsize == PXE_CPBSIZE_NOT_USED) {
    if (CdbPtr->CPBaddr != PXE_CPBADDR_NOT_USED) {
      goto BadCdb;
    }
  } else if (CdbPtr->CPBaddr == PXE_CPBADDR_NOT_USED) {
    goto BadCdb;
  }

  if (CdbPtr->DBsize == PXE_DBSIZE_NOT_USED) {
    if (CdbPtr->DBaddr != PXE_DBADDR_NOT_USED) {
      goto BadCdb;
    }
  } else if (CdbPtr->DBaddr == PXE_DBADDR_NOT_USED) {
    goto BadCdb;
  }

  // check if CpbSize and DbSize are as needed
  // check if OpFlags are as expected
  TabPtr = &gUndiApiTable[CdbPtr->OpCode];

  if ((TabPtr->CpbSize != (UINT16) (DONT_CHECK))
    && (TabPtr->CpbSize != CdbPtr->CPBsize))
  {
    goto BadCdb;
  }

  if ((TabPtr->DbSize != (UINT16) (DONT_CHECK))
    && (TabPtr->DbSize != CdbPtr->DBsize))
  {
    goto BadCdb;
  }

  if ((TabPtr->OpFlags != (UINT16) (DONT_CHECK))
    && (TabPtr->OpFlags != CdbPtr->OpFlags))
  {
    goto BadCdb;
  }

  // check if UNDI_State is valid for this call
  if (TabPtr->State != (UINT16) (-1)) {

    // should atleast be started
    if (AdapterInfo->State == PXE_STATFLAGS_GET_STATE_STOPPED) {
      CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
      CdbPtr->StatCode  = PXE_STATCODE_NOT_STARTED;
      return;
    }

    // check if it should be initialized
    if (TabPtr->State == 2) {
      if (AdapterInfo->State != PXE_STATFLAGS_GET_STATE_INITIALIZED) {
        CdbPtr->StatCode  = PXE_STATCODE_NOT_INITIALIZED;
        CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
        return;
      }
    }
  }

  // set the return variable for success case here
  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_COMPLETE;
  CdbPtr->StatCode  = PXE_STATCODE_SUCCESS;

  TabPtr->ApiPtr (CdbPtr, AdapterInfo);
  return;

BadCdb:
  DEBUGPRINT (CRITICAL, ("ERROR: Invalid CDB\n"));
  CdbPtr->StatFlags = PXE_STATFLAGS_COMMAND_FAILED;
  CdbPtr->StatCode  = PXE_STATCODE_INVALID_CDB;
}

