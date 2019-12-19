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

#ifndef INIT_H_
#define INIT_H_

#define EFI_NII_POINTER_PROTOCOL_GUID \
  { 0xE3161450, 0xAD0F, 0x11D9, { 0x96, 0x69, 0x08, 0x00, 0x20, 0x0c, 0x9a, 0x66 } }

#define MAC_ADDRESS_SIZE_IN_BYTES 6

#define MAX_FIRMWARE_COMPATIBILITY_STRING 150
#define MAX_DRIVER_HEALTH_ERROR_STRING    200
typedef enum {
  FW_COMPATIBLE = 0,
  FW_NEWER_THAN_EXPECTED,
  FW_OLDER_THAN_EXPECTED,
  FW_INCOMPATIBLE,
  FW_RECOVERY_MODE
} FW_COMPATIBILITY_LEVEL;

/** Return the health status of the controller.

   If there is a message status that is related to the
   current health status it is also prepared and returned
   by this function

   @param[in]    UndiPrivateData      controller data
   @param[out]   DriverHealthStatus   controller status to be returned
   @param[out]   MessageList      pointer to the message list to be returned

   @retval   EFI_SUCCESS              Health status retrieved successfully or driver is healthy
   @retval   EFI_SUCCESS              MessageList is NULL or HII is not supported on this port
   @retval   EFI_INVALID_PARAMETER    Invalid parameter passed
   @retval   EFI_OUT_OF_RESOURCES     We are out of resources either for allocating MessageList
                                      or setting HII string
**/
EFI_STATUS
UndiGetControllerHealthStatus (
  IN  UNDI_PRIVATE_DATA              *UndiPrivateData,
  OUT EFI_DRIVER_HEALTH_STATUS       *DriverHealthStatus,
  OUT EFI_DRIVER_HEALTH_HII_MESSAGE **MessageList
  );

/** Return the cumulative health status of all controllers managed by the driver

   @param[out]   DriverHealthStatus   controller status to be returned

   @retval   EFI_SUCCESS             Procedure returned successfully
   @retval   EFI_INVALID_PARAMETER   DriverHealthStatus is NULL
   @retval   EFI_DEVICE_ERROR        Failed to get controller health status
**/
EFI_STATUS
UndiGetDriverHealthStatus (
  OUT EFI_DRIVER_HEALTH_STATUS *DriverHealthStatus
  );

#endif /* INIT_H_ */
