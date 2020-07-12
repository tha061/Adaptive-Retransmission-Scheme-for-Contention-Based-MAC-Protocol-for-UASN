
# Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
# All rights reserved.

# Author: Tham Nguyen
# Version: 1.0.0

PacketHeaderManager set tab_(PacketHeader/CSMAAR_DATA) 1
PacketHeaderManager set tab_(PacketHeader/CSMAAR_ACK) 1
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
