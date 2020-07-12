#ifndef HDR_CSMAAR_ACK_H 
#define HDR_CSMAAR_ACK_H

#include <mac.h>
#include <iostream>
#include <string>
#include <map>
#include <set>
#include <queue>
#include <fstream>
#include <packet.h>
#include <iostream>
#include <string>
#include <sstream>
#define HDR_CSMAAR_ACK(P)      (hdr_csmaar_ack::access(P))

using namespace std;

extern packet_t PT_CSMAAR_ACK;
// definition of CSMA_AR header

typedef struct hdr_csmaar_ack {
	MacFrameType ftype_;	// frame type
	int macSA_;		// source MAC address
	int macDA_;		// destination MAC address
	u_int16_t hdr_type_;     // mac_hdr type

        /* For CSMA_AR from DESERT, add by Tham Ng */
        u_int16_t num_tran_within_time_window_ar_; 
        float    rate_of_traffic_gen_by_all_nodes_;
        float   delay_ar_;   //delay from sink node
        /* */ 

	double txtime_;		// transmission time
	double sstime_;		// slot start time

	int padding_;

	inline void set(MacFrameType ft, int sa, int da=-1) {
		ftype_ = ft;
		macSA_ = sa;
		if (da != -1)  macDA_ = da;
	}
	inline MacFrameType& ftype() { return ftype_; }
	inline int& macSA() { return macSA_; }
	inline int& macDA() { return macDA_; }
	inline u_int16_t& hdr_type() {return hdr_type_; }
        // for CSMA_AR from Desert, by Tham Ng
        inline u_int16_t& num_tran_within_time_window_ar() {return num_tran_within_time_window_ar_; }
        inline float& rate_of_traffic_gen_by_all_nodes() { return rate_of_traffic_gen_by_all_nodes_;}
        inline float&  delay_ar() {return delay_ar_;}
        ///
	inline double& txtime() { return txtime_; }
	inline double& sstime() { return sstime_; }

	// Header access methods
	static int offset_;
	inline static int& offset() { return offset_; }
	inline static hdr_csmaar_ack* access(const Packet* p) {
		return (hdr_csmaar_ack*) p->access(offset_);
	}
} hdr_csmaar_ack;

#endif /* HDR_CSMAAR_ACK_H */
