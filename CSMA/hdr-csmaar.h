#ifndef HDR_CSMAAR_DATA_H 
#define HDR_CSMAAR_DATA_H


#include <iostream>
#include <string>
#include <map>
#include <set>
#include <queue>
#include <fstream>
#include <packet.h>

#define HDR_CSMAAR_DATA(P)      (hdr_csmaar::access(P))

using namespace std;

extern packet_t PT_CSMAAR_DATA;
// definition of CSMA_AR header
typedef struct hdr_csmaar {
	//MacFrameType ftype_;	// frame type
        u_int8_t    num_trans_;    // number of packets transmitted in time_window
	int macSA_;		// source MAC address
	int macDA_;		// destination MAC address
	//u_int16_t hdr_type_;     // mac_hdr type
	//double txtime_;		// transmission time
	//double sstime_;		// slot start time
	//int padding_;

	//inline void set(MacFrameType ft, int sa, int da=-1) {
	//	ftype_ = ft;
	//	macSA_ = sa;
	//	if (da != -1)  macDA_ = da;
	//}
	//inline MacFrameType& ftype() { return ftype_; }
	inline int& macSA() { return macSA_; }
	inline int& macDA() { return macDA_; }
	//inline u_int16_t& hdr_type() {return hdr_type_; }
        inline u_int8_t&  num_trans() {return num_trans_;}
        
	//inline double& txtime() { return txtime_; }
	//inline double& sstime() { return sstime_; }

	// Header access methods
	static int offset_;
	inline static int& offset() { return offset_; }
	inline static struct hdr_csmaar* access(const Packet* p) {
		return (struct hdr_csmaar*) p->access(offset_);
	}
  
} hdr_csmaar;
#endif /* HDR_CSMAAR_DATA_H */
