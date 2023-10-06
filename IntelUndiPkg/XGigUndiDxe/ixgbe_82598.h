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
#ifndef _IXGBE_82598_H_
#define _IXGBE_82598_H_

#ifdef NO_API_SUPPORT
s32 ixgbe_init_ops_82598(struct ixgbe_hw *hw);
#endif
u32 ixgbe_get_pcie_msix_count_82598(struct ixgbe_hw *hw);
s32 ixgbe_fc_enable_82598(struct ixgbe_hw *hw);
s32 ixgbe_reset_hw_rev_0_82598(struct ixgbe_hw *hw);
s32 ixgbe_start_hw_82598(struct ixgbe_hw *hw);
#ifndef NO_RELAX_ORDERING_EN_SUPPORT
void ixgbe_enable_relaxed_ordering_82598(struct ixgbe_hw *hw);
#endif /* NO_RELAX_ORDERING_EN_SUPPORT */
s32 ixgbe_set_vmdq_82598(struct ixgbe_hw *hw, u32 rar, u32 vmdq);
s32 ixgbe_set_vfta_82598(struct ixgbe_hw *hw, u32 vlan, u32 vind, bool vlan_on,
			 bool vlvf_bypass);
s32 ixgbe_read_analog_reg8_82598(struct ixgbe_hw *hw, u32 reg, u8 *val);
s32 ixgbe_write_analog_reg8_82598(struct ixgbe_hw *hw, u32 reg, u8 val);
s32 ixgbe_read_i2c_eeprom_82598(struct ixgbe_hw *hw, u8 byte_offset,
				u8 *eeprom_data);
u64 ixgbe_get_supported_physical_layer_82598(struct ixgbe_hw *hw);
s32 ixgbe_init_phy_ops_82598(struct ixgbe_hw *hw);
void ixgbe_set_lan_id_multi_port_pcie_82598(struct ixgbe_hw *hw);
void ixgbe_set_pcie_completion_timeout(struct ixgbe_hw *hw);
s32 ixgbe_enable_rx_dma_82598(struct ixgbe_hw *hw, u32 regval);
#endif /* _IXGBE_82598_H_ */
