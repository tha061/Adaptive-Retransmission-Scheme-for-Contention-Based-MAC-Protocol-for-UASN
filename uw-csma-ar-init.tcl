#
# Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the University of Padova (SIGNET lab) nor the 
#    names of its contributors may be used to endorse or promote products 
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
# TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Federico Guerra, Saiful Azad and Federico Favaro
# Version: 1.0.0

PacketHeaderManager set tab_(PacketHeader/CSMAAR_DATA) 1
PacketHeaderManager set tab_(PacketHeader/CSMAAR_ACK) 1
#Module/UW/CSMA_AR set HDR_size_ 			0
Module/UW/CSMA_AR set ACK_size_  			10
Module/UW/CSMA_AR set max_tx_tries_		5
Module/UW/CSMA_AR set wait_costant_		0.1
Module/UW/CSMA_AR set debug_			0
Module/UW/CSMA_AR set backoff_tuner_   		1
Module/UW/CSMA_AR set max_payload_			125
Module/UW/CSMA_AR set ACK_timeout_			5.0
Module/UW/CSMA_AR set alpha_			0.8
Module/UW/CSMA_AR set buffer_pkts_			-1
Module/UW/CSMA_AR set max_backoff_counter_   	4
Module/UW/CSMA_AR set listen_time_ 		0.5
Module/UW/CSMA_AR set MAC_addr_ 		0
#Seting for CSMA-ARS
Module/UW/CSMA_AR set HDR_size_ 		10
Module/UW/CSMA_AR set number_of_flow_ar_       10   
Module/UW/CSMA_AR set p_req_ar_                0.95 
Module/UW/CSMA_AR set bit_data_rate_ar_        14000
Module/UW/CSMA_AR set object_ar_               1
Module/UW/CSMA_AR set delta_ar_                 0.001
Module/UW/CSMA_AR set epsilon1_ar_              0.01
Module/UW/CSMA_AR set epsilon2_ar_              0.01
Module/UW/CSMA_AR set packet_period_ar_        5
Module/UW/CSMA_AR set x_upper_              5
Module/UW/CSMA_AR set time_to_check_bf_     10000
Module/UW/CSMA_AR set boff_factor_          0.06
Module/UW/CSMA_AR set beta_                  1
Module/UW/CSMA_AR set alpha_t_                  0.5
Module/UW/CSMA_AR set print_sink_            0
Module/UW/CSMA_AR set print_px_              0
Module/UW/CSMA_AR set print_process_              0
