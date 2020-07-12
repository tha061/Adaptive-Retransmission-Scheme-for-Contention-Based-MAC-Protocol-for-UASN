// Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
// All rights reserved.


/**
 * @file   uw-csma-ar.cpp
 * @author Tham Nguyen
 * @version 1.0.0
 * 
 * \brief Provides the implementation of Csma-AR Protocol
 * 
 */


#include "uw-csma-ar.h"
#include <mac.h>
#include <cmath>
#include <climits>
#include <iomanip>
#include <rng.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <rng.h>
#include "hdr-csmaar.h"
#include "hdr-csmaar-ack.h"
using namespace std;

enum {
    NOT_SET = -1, SESSION_DISTANCE_NOT_SET = 0
};


/**
 * Class that represents the binding with the tcl configuration script 
 */
static class CSMAARModuleClass : public TclClass {
public:
  /**
   * Constructor of the class
   */
  CSMAARModuleClass() : TclClass("Module/UW/CSMA_AR") {}
  /**
   * Creates the TCL object needed for the tcl language interpretation
   * @return Pointer to an TclObject
   */
  TclObject* create(int, const char*const*) {
    return (new CsmaAr());
  }
} class_module_csmaar;

//-----Timer-----------------------
void CsmaAr::AckTimer::expire(Event *e) {
  timer_status = CSMA_EXPIRED;
  if (module->curr_state == CSMA_STATE_WAIT_ACK) {


        if (module->debug_) cout << NOW << "  CsmaAr("<< module->addr << ") timer expire() current state = " 
                         << module->status_info[module->curr_state] << "; ACK not received, next state = " 
                         << module->status_info[CSMA_STATE_BACKOFF] << endl;

        module->refreshReason(CSMA_REASON_ACK_TIMEOUT);
        module->stateBackoff(); 
  }
  else {
    if (module->debug_ ) cout << NOW << "  CsmaAr("<< module->addr << ")::AckTimer::expired() " << endl;
  }  
}


void CsmaAr::BackOffTimer::expire(Event *e) {
  timer_status = CSMA_EXPIRED;
  if (module->curr_state == CSMA_STATE_BACKOFF ) {
    
    if (module->debug_) cout << NOW << "  CsmaAr("<< module->addr << ") timer expire() current state = " 
                             << module->status_info[module->curr_state] << "; backoff expired, next state = " 
                             << module->status_info[CSMA_STATE_IDLE] << endl;

    module->refreshReason(CSMA_REASON_BACKOFF_TIMEOUT);
    module->exitBackoff();
    module->stateIdle();
  }
  else {
    if (module->debug_ ) cout << NOW << "  CsmaAr("<< module->addr << ")::BackoffTimer::expired() " << endl;
  }  
}


void CsmaAr::ListenTimer::expire(Event *e) {   
  timer_status = CSMA_EXPIRED;

  if (module->curr_state == CSMA_STATE_LISTEN ) {

    if (module->debug_) cout << NOW << "  CsmaAr("<< module->addr << ") timer expire() current state = " 
                      << module->status_info[module->curr_state] << "; listening period expired, next state = " 
                      << module->status_info[CSMA_STATE_TX_DATA] << endl;

    module->refreshReason(CSMA_REASON_LISTEN_TIMEOUT);
    module->stateTxData();
  }
  else {
    if (module->debug_ ) cout << NOW << "  CsmaAr("<< module->addr << ")::ListenTimer::expired() " << endl;
  }  
}

/*  Window Timer for csma-ar expire: added by Tham Nguyen UOU 2012 */

void CsmaAr::WindowTimer_Ar::expire(Event *e) {   
   timer_status = CSMA_EXPIRED;
  // printf("%f CsmaAr::WindowTimer_Ar::expire\n",NOW);
   module->getTrafficWithinTimeWd_Ar(); // for modify 2
  // module->sinkProcess();
}

/*  Sink Timer for ARS scheme : added by Tham Nguyen UOU 2012 */
void CsmaAr::SinkTimer::expire(Event *e) {
  timer_status = CSMA_EXPIRED;
 //if (module->print_sink) printf("%f CsmaAr::SinkTimer::expire\n",NOW );
// printf("%f CsmaAr::SinkTimer::expire\n",NOW );
  module->sinkProcess();
 }
//--------------------------------------
 
const double CsmaAr::prop_speed = 1500.0;
int CsmaAr::u_pkt_id;
bool CsmaAr::initialized = false;

//----Tham's adding---------------------
int CsmaAr::max_tx_tries;
int CsmaAr::total_orig = 0;
int CsmaAr::recv_frame = 0;
int CsmaAr::total_retrans = 0;
int CsmaAr::ack_rcv = 0;
int CsmaAr::recv_first_frame = 0;
int CsmaAr::drop_pkt = 0;
double CsmaAr::time_window_ar;
double CsmaAr::time_tr_ar;
//--------------------------------------

map< CsmaAr::CSMA_STATUS , string> CsmaAr::status_info;
map< CsmaAr::CSMA_REASON_STATUS, string> CsmaAr::reason_info;
map< CsmaAr::CSMA_PKT_TYPE, string> CsmaAr::pkt_type_info;

CsmaAr::CsmaAr() 
: ack_timer(this),
  listen_timer(this),
  backoff_timer(this),
  window_timer_ar(this), //added by Tham Ng, UOU korea 2012
  sink_timer(this),

  u_data_id(0),
  session_distance(SESSION_DISTANCE_NOT_SET),
  curr_data_pkt(0),
  last_data_id_rx(NOT_SET),
  curr_tx_rounds(0),
  TxActive(false),
  RxActive(false),
  session_active(false),
  print_transitions(false),
  has_buffer_queue(false),
  curr_state(CSMA_STATE_IDLE), 
  prev_state(CSMA_STATE_IDLE),
  prev_prev_state(CSMA_STATE_IDLE),
  ack_mode(CSMA_ACK_MODE),
  last_reason(CSMA_REASON_NOT_SET),
  start_tx_time(0),       
  srtt(0),      
  sumrtt(0),      
  sumrtt2(0),     
  rttsamples(0),
//---Tham's adding-------------------------
  num_trans_current_ar(0),//added by Tham
  re_trans_no(0),  
  e(2.71828),
  //x_upper(5),
  //recv_frame(0),
  //packet_length(80),
  ack_sent(0),
  //recv_first_frame(0),
  total_packet_trans_ar(0),
  total_packet_recev_ar(0),
  original_current_ar(0),
  average_num_trans_ar(0),
  Px_curr(0),
  pdr_curr(0),
  Rm(0),
  Ot_m(0),
  n1(0),
  n2(0),
  n3(0),
  n4(0),
  n5(0),
  n6(0),
  n7(0),
  n8(0),
  n9(0),
  n10(0),
//----------------------------------------
  last_sent_data_id(-1)
{ 
  u_pkt_id = 0;
  mac2phy_delay_ = 1e-19;
  
  bind("HDR_size_", (int*)& HDR_size);
  bind("ACK_size_", (int*)& ACK_size);
  bind("max_tx_tries_", (int*)& max_tx_tries);
  bind("wait_costant_", (double*)& wait_costant);
  bind("debug_", (double*)& debug_);
  bind("max_payload_", (int*)& max_payload);
  bind("ACK_timeout_", (double*)& ACK_timeout);
  bind("alpha_", (double*)& alpha_);
  bind("backoff_tuner_", (double*)& backoff_tuner);
  bind("buffer_pkts_", (int*)& buffer_pkts);
  bind("listen_time_", & listen_time);
 // bind("max_backoff_counter_", (int*)& max_backoff_counter);
  bind("max_backoff_counter_", (double*)& max_backoff_counter); //corrected by tham

//---Addition for CSMA_ARS : Tham Ng 2012--------------
  // bind("time_window_ar_",(double*)& time_window_ar);//time_window, added by Tham Ng UOU Korea 2012  
  bind("number_of_flow_ar_", (int*)& number_of_flow_ar);
  bind("p_req_ar_", (double*)& p_req_ar);
  bind("bit_data_rate_ar_", (double*)& bit_data_rate_ar);
  bind("object_ar_",(int*)& object_ar);  
  bind("epsilon1_ar_", (double*)& epsilon1_ar);  
  bind("epsilon2_ar_", (double*)& epsilon2_ar);  
  bind("packet_period_ar_", (double*)& packet_period_ar); 
  bind("x_upper_", (int*)& x_upper); 
  bind("time_to_check_bf_", (double*)& time_to_check_bf);    
  bind("boff_factor_", (double*)& boff_factor);  
  bind("beta_",(double*)& beta); 
  bind("alpha_t_", (double*)& alpha_t); 
  bind("print_sink_", (double*)& print_sink);
  bind("print_px_", (double*)& print_px);
  bind("print_process_", (double*)& print_process);
  
  /* window timer start */
  time_window_ar = 10*packet_period_ar;
  sink_time_window = 10*packet_period_ar;
  window_timer_ar.schedule(time_window_ar); //added by Tham Ng, UOU Korea 2012
  
  sum_of_traffic_ar = max_tx_tries * number_of_flow_ar;
  total_background_traffic_ar = 1;
  total_original_traffic_ar = 1;
  node_background_traffic_ar = 0;
  node_original_traffic_ar = 0;
  max_tx_tries_curr = max_tx_tries;
  max_starting = max_tx_tries;
//----------------------------------------------------     
  if (max_tx_tries <= 0) max_tx_tries = INT_MAX;
  if (buffer_pkts > 0) has_buffer_queue = true;
  if ( listen_time <= 0.0 ) listen_time = 1e-19;
}


CsmaAr::~CsmaAr()
{

}

// TCL command interpreter
int CsmaAr::command(int argc, const char*const* argv)
{
  Tcl& tcl = Tcl::instance();
  if (argc==2)
    {
      if(strcasecmp(argv[1], "setAckMode") == 0)
	{
	  ack_mode = CSMA_ACK_MODE;
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "setNoAckMode") == 0)	
	{
          ack_mode = CSMA_NO_ACK_MODE;
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "initialize") == 0)	
	{
          if (initialized == false) initInfo();          
          if (print_transitions) fout.open("/tmp/CSMAstateTransitions.txt",ios_base::app);
         // For Csma-Ar          
          initArrayOfNode_Ar();
          initArrayBackgroundTrafficOfEachNode_Ar(); // modify 2
          initArrayOriginalTrafficOfEachNode_Ar();   // modify 2
          initArrayMaxTransOfNode_Ar(); // For modify 2
          initMatrixPacketRecv_Ar();
          sink_timer.schedule(sink_time_window); // Sink Timer
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "printTransitions") == 0)	
	{
          print_transitions = true;
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "getQueueSize") == 0)	
	{
	  tcl.resultf("%d",Q.size());
      	  return TCL_OK;
        }
} 
    else if(argc==3){
		if(strcasecmp(argv[1],"setMacAddr") == 0)
		{
			addr = atoi(argv[2]);
			if(debug_) cout << "Csma_Ar MAC address of current node is " << addr <<endl;
			return TCL_OK;
		}
	}
  return MMac::command(argc, argv);
}



int CsmaAr::crLayCommand(ClMessage* m)
{
  switch (m->type()) 
    {
    
    
    default:
      return Module::crLayCommand(m);    
    }  
}


void CsmaAr::initInfo()
{

  initialized = true;

  if ( (print_transitions) && (system(NULL)) ) {
      system("rm -f /tmp/CSMAstateTransitions.txt");
      system("touch /tmp/CSMAstateTransitions.txt");
  }

  status_info[CSMA_STATE_IDLE] = "Idle state";
  status_info[CSMA_STATE_BACKOFF] = "Backoff state"; 
  status_info[CSMA_STATE_TX_DATA] = "Transmit DATA state"; 
  status_info[CSMA_STATE_TX_ACK] = "Transmit ACK state";
  status_info[CSMA_STATE_WAIT_ACK] = "Wait for ACK state"; 
  status_info[CSMA_STATE_DATA_RX] = "DATA received state"; 
  status_info[CSMA_STATE_ACK_RX] = "ACK received state"; 
  status_info[CSMA_STATE_LISTEN] = "Listening channel state";
  status_info[CSMA_STATE_RX_IDLE] = "Start rx Idle state";
  status_info[CSMA_STATE_RX_BACKOFF] = "Start rx Backoff state";
  status_info[CSMA_STATE_RX_LISTEN] = "Start rx Listen state";
  status_info[CSMA_STATE_RX_WAIT_ACK] = "Start rx Wait ACK state";
  status_info[CSMA_STATE_CHK_LISTEN_TIMEOUT] = "Check Listen timeout state";
  status_info[CSMA_STATE_CHK_BACKOFF_TIMEOUT] = "Check Backoff timeout state";
  status_info[CSMA_STATE_CHK_ACK_TIMEOUT] = "Check Wait ACK timeout state";
  status_info[CSMA_STATE_WRONG_PKT_RX] = "Wrong Pkt Rx state";
  
  reason_info[CSMA_REASON_DATA_PENDING] = "DATA pending from upper layers"; 
  reason_info[CSMA_REASON_DATA_RX] = "DATA received";
  reason_info[CSMA_REASON_DATA_TX] = "DATA transmitted"; 
  reason_info[CSMA_REASON_ACK_TX] = "ACK tranmsitted";
  reason_info[CSMA_REASON_ACK_RX] = "ACK received"; 
  reason_info[CSMA_REASON_BACKOFF_TIMEOUT] = "Backoff expired"; 
  reason_info[CSMA_REASON_ACK_TIMEOUT] = "ACK timeout"; 
  reason_info[CSMA_REASON_DATA_EMPTY] = "DATA queue empty";
  reason_info[CSMA_REASON_MAX_TX_TRIES] = "DATA dropped due to max tx rounds";
  reason_info[CSMA_REASON_LISTEN] = "DATA pending, listening to channel";
  reason_info[CSMA_REASON_LISTEN_TIMEOUT] = "DATA pending, end of listening period";
  reason_info[CSMA_REASON_START_RX] = "Start rx pkt";
  reason_info[CSMA_REASON_PKT_NOT_FOR_ME] = "Received an erroneous pkt";
  reason_info[CSMA_REASON_BACKOFF_PENDING] = "Backoff timer pending";
  reason_info[CSMA_REASON_WAIT_ACK_PENDING] = "Wait for ACK timer pending";
  reason_info[CSMA_REASON_LISTEN_PENDING] = "Listen to channel pending";
  reason_info[CSMA_REASON_PKT_ERROR] = "Erroneous pkt";
  
  pkt_type_info[CSMA_ACK_PKT] = "ACK pkt";
  pkt_type_info[CSMA_DATA_PKT] = "DATA pkt"; 
  pkt_type_info[CSMA_DATAMAX_PKT] = "MAX payload DATA pkt";
}

void CsmaAr::resetSession()
{   
  session_distance = SESSION_DISTANCE_NOT_SET; 
}
//=======modified by Tham Ng, UOU Korea 2012-- use fixed ACK timeout, not update
/*void CsmaAloha::updateRTT(double curr_rtt)
{
  srtt = alpha_ * srtt + (1-alpha_) * curr_rtt;
  sumrtt += curr_rtt;
  sumrtt2 += curr_rtt*curr_rtt;
  rttsamples++;
  ACK_timeout = ( srtt / rttsamples );
}

void CsmaAloha::updateAckTimeout(double rtt) {
  updateRTT(rtt);

  if (debug_) cout << NOW << "  CsmaAloha(" << addr << ")::updateAckTimeout() curr ACK_timeout = " 
                   << ACK_timeout << endl;
}*/
//============
bool CsmaAr::keepDataPkt(int serial_number) {
  bool keep_packet;
  if (serial_number > last_data_id_rx) {
    keep_packet = true;
    last_data_id_rx = serial_number;
  }
  else keep_packet = false;
  return keep_packet;
}

double CsmaAr::computeTxTime(CSMA_PKT_TYPE type)
{
  double duration;
  Packet* temp_data_pkt;

  if (type == CSMA_DATA_PKT) {
     if (!Q.empty()) {
        temp_data_pkt = (Q.front())->copy();  
        hdr_cmn *ch = HDR_CMN(temp_data_pkt);
        ch->size() = HDR_size + ch->size();
     }
     else { 
        temp_data_pkt = Packet::alloc();
        hdr_cmn *ch = HDR_CMN(temp_data_pkt);
        ch->size() = HDR_size + max_payload;
     }
  }
  else if (type == CSMA_ACK_PKT) {
        temp_data_pkt = Packet::alloc();  
        hdr_cmn *ch = HDR_CMN(temp_data_pkt);
        ch->size() = ACK_size;
  }
  duration = Mac2PhyTxDuration(temp_data_pkt );
  Packet::free(temp_data_pkt);
  return(duration);
}
//-----------------------------------------
// Get the transmission time: time_tr_ar : Tham Nguyen 2012
int CsmaAr::getTransTime_Ar(int size_ar)
  {
   packet_size_ar = size_ar;
   time_tr_ar = (packet_size_ar*8)/bit_data_rate_ar;
  // time_tr_ar = (packet_size_ar*8)/14000;
   return (time_tr_ar);
   }
// --- modify 2 ---
double CsmaAr::getTrafficWithinTimeWd_Ar()
 {
   window_timer_ar.resched(time_window_ar);

   num_tran_within_time_window_ar = num_trans_current_ar;
   num_trans_current_ar = 0;
   original_packet_within_time_wd_ar = original_current_ar;
   original_current_ar = 0; //reset no_of_sent 
   node_background_traffic_ar = (num_tran_within_time_window_ar /time_window_ar); // lambda_i
   node_original_traffic_ar = (original_packet_within_time_wd_ar/time_window_ar); //original traffic of node i
   /*
   printf("node %d at time %f num trans: %d\n",addr,NOW, num_tran_within_time_window_ar);
   printf("node %d at time %f num of orriginal packet: %d\n",addr,NOW,original_packet_within_time_wd_ar);
   printf("node %d at time %f background traffic: %f\n",addr,NOW, node_background_traffic_ar);
   printf("node %d at time %f original traffic: %f\n",addr,NOW,node_original_traffic_ar);
    */
  
   return(node_background_traffic_ar,node_original_traffic_ar);
}

// Init the array to store the packet's sourse addr at sink: Tham Ng 2012
void CsmaAr::initArrayOfNode_Ar()
 {
   for (int i = 0, n = 99; i<=n; i++)
    { 
        Node_Array[i] = -1;
     }
  }
void CsmaAr::initArrayMaxTransOfNode_Ar()
{
  for (int i = 0, n = 99; i<=n; i++)
  {
       MaxTrans_Array[i] = max_tx_tries;
     // cout << MaxTrans_Array[i] << " ";
  } 
     // cout << "\n";
}

// -- modify 2 --
void CsmaAr::initArrayBackgroundTrafficOfEachNode_Ar()
 {
   for (int i=0,n=99; i<=n; i++) 
     {
        BackgroundTraffic_Array[i] = 0;
     }
  }
void CsmaAr::initArrayOriginalTrafficOfEachNode_Ar()
 {
  for (int i=0,n=99; i<=n; i++)
  {
    OriginalTraffic_Array[i] = 0;
  }
}

void CsmaAr::initMatrixPacketRecv_Ar() //For calculate instaneous PDR
 {

for (int i = 0; i < 100; i++)
    { 
     for (int j = 0; j < 500; j++)
         { 
           matrix_t[i][j] = -1;
         //  printf("%d ",matrix_t[i][j]);
          }
   // printf("\n");
   } 

 }
//-----------------------------------

void CsmaAr::sinkProcess() 
 {
   sink_timer.resched(sink_time_window);

 //--new-idea
    //double alpha_t = 0.9;
     Rs = total_retrans/sink_time_window;
     Ot_curr = total_orig/sink_time_window;
 if (total_orig != 0)    
   {  //rc = Rs/Os;
     if (Rm == 0) Rm = Rs;
     if (Ot_m == 0) Ot_m = Ot_curr;
   
     Rm = alpha_t*Rm + (1 - alpha_t)*Rs;
     Ot_m = alpha_t*Ot_m + (1 - alpha_t)*Ot_curr;

     rc = Rm/Ot_m;
     double pdr = (double)(recv_first_frame)/total_orig;
     double pdp = (double)(recv_frame)/total_retrans;
    //  printf("	====================\n");
     // printf("	total_retrans = %d\n",total_retrans);
    //  printf("	total_recv = %d\n",recv_frame);
     // printf("	total_orig = %d\n",total_orig);
     // printf("	total_recv_first = %d\n",recv_first_frame);     
     // printf("	rc = %f\n",rc);
    //  printf("	Rs = %f\n",Rs);
     // printf("	Rm = %f\n",Rm);
     // printf("	Ot_curr = %f\n",Ot_curr);
     // printf("	Ot_m = %f\n", Ot_m);
      if (print_sink) 
    {  printf("Px_curr = %f\n", Px_curr);
       printf("max_tx_tries_curr = %d\n",max_tx_tries_curr);
    }
      if (print_sink)  printf("PDR =%f\n",pdr);
      if (print_process) printf("%f ",pdr);
       // printf("pdp = %f\n",pdp);
    //  printf("	====================\n");
      
      total_retrans = 0;
      total_orig = 0;
      recv_first_frame = 0;
      recv_frame = 0;
      for (int i = 0; i < 100; i++)
       { 
      for (int j = 0; j < 500; j++)
         { 
           matrix_t[i][j] = -1;
        
          }
       } 

// For AR-MAC
//=========================================================================
             //--- Calculate Ps, Px(rc,X), E(Y,X)

  //  double lambda_b = Rs*(number_of_flow_ar - 1)/number_of_flow_ar;
    double lambda_b = Rm*(number_of_flow_ar - 1)/number_of_flow_ar;
    double t_ack = (ACK_size*8)/bit_data_rate_ar;   
    double p_s = pow(e, (-2 *lambda_b*time_tr_ar));          //Ps1
   
    double p_f = 1 - p_s;
    double Px_rc_x = (1 - pow(p_f, max_tx_tries));
    if (print_process) printf("%f ",Px_rc_x);
    if (print_sink) printf("Px(rc,X) =%f\n",Px_rc_x);
    if (print_px) printf("%f %f\n",pdr,Px_rc_x);
    double E_y_x = p_s + max_tx_tries*pow(p_f,(max_tx_tries - 1));
    for (int i = 1; i <= (max_tx_tries - 2); i++)
       {
          E_y_x = E_y_x + (i + 1)*p_s*pow(p_f, i);
       }

//================================================================================    
                //-- Calculate Px(rc+delta_r, X+1), E(Y,X+1)

   if (print_sink)  printf("	X+1 = %d\n",max_tx_tries + 1);
    double E_y_x1 = p_s + (max_tx_tries+1)*pow(p_f,(max_tx_tries));
    
    for (int i = 1; i <= (max_tx_tries - 1); i++)
       {
          E_y_x1 = E_y_x1 + (i + 1)*p_s*pow(p_f, i);
       }
  
    double delta_r1 = E_y_x1 - E_y_x;
  
    double lambda_b1 = Ot_m*(rc + delta_r1*beta)*(number_of_flow_ar - 1)/number_of_flow_ar; 
    double p_s_delta_r1 = pow(e, (-2*lambda_b1*time_tr_ar)); // (option1)
  
    double p_f_delta_r1 = 1 - p_s_delta_r1;
    int x1 = max_tx_tries + 1;
    double Px_rc1_x1 = 1 - pow(p_f_delta_r1, max_tx_tries + 1);
    if (print_process) printf("%f ",Px_rc1_x1);
    if (print_sink) printf("	Px(rc+delta_r, X+1) = %f\n",Px_rc1_x1); 

//================================================================================
              //--Calculate Px(rc-delta_r, X-1), E(Y,X-1)
   
   if (print_sink)  printf("	X-1 = %d\n", max_tx_tries - 1 );
    double E_y_x2 = p_s + (max_tx_tries-1)*pow(p_f,(max_tx_tries - 2));
    for (int i = 1; i <= (max_tx_tries - 3); i++)
       {
          E_y_x2 = E_y_x2 + (i + 1)*p_s*pow(p_f, i);
       }
  
    double delta_r2 = E_y_x2 - E_y_x;
 
    double lambda_b2 = Ot_m*(rc + delta_r2*beta)*(number_of_flow_ar - 1)/number_of_flow_ar;
   
    double p_s_delta_r2 = pow(e, (-2*lambda_b2*time_tr_ar)); // option1
   
    double p_f_delta_r2 = 1 - p_s_delta_r2;
    int x2 = max_tx_tries - 1;
    double Px_rc2_x2 = 1 - pow(p_f_delta_r2, max_tx_tries - 1);
    if (print_process) printf ("%f ",Px_rc2_x2);
    if (print_sink)  printf("	Px(rc-delta_r, X-1) = %f\n",Px_rc2_x2);
 
//=================================================================================
             // Change the max_tx_tries if the gain > epsilon


/*
  if (Px_rc_x > Px_curr) 
    {
        Px_curr = Px_rc_x; 
        max_tx_tries_curr = max_tx_tries;
       if (print_sink)   printf("	Current max_tx_tries = %d\n", max_tx_tries_curr);
           
    }


  if (max_tx_tries > max_tx_tries_curr && Px_rc_x < Px_curr)  
    {
        max_tx_tries = max_tx_tries_curr;
        if (print_process) printf ("%d\n",max_tx_tries);
         if (print_sink)   printf("	Return to curr the max_tx_tries: %d\n", max_tx_tries);
    }
   else
   { 
     if (Px_rc1_x1 - Px_rc_x >= epsilon_ar && (max_tx_tries < x_upper)) 
    // if (Px_rc1_x1 - Px_rc_x >= 0.1 && (max_tx_tries < x_upper)) 
       {
          max_tx_tries++;
         // max_tx_tries_curr = max_tx_tries - 1;
          if (print_process) printf ("%d\n",max_tx_tries);
          if (print_sink)   printf("	Increase the max_tx_tries: X = %d\n", max_tx_tries);
       }
     else if (Px_rc2_x2 - Px_rc_x >= epsilon_ar && (max_tx_tries > 1))
          {
            max_tx_tries = max_tx_tries - 1;
          //  max_tx_tries_curr = max_tx_tries + 1;
            if (print_process) printf ("%d\n",max_tx_tries);
            if (print_sink)  printf("	Decrease the max_tx_tries: X = %d\n", max_tx_tries);
          }     

    else if (print_process) printf ("%d\n",max_tx_tries);
   } 
  */ 
                //----------------------------------------
                        // Modified algorithm
  
        double test1 = abs (Px_rc1_x1 - Px_rc_x);
        double test2 = abs (Px_rc2_x2 - Px_rc_x);  
        if (print_sink)   printf("	test1 = %f\n", test1);
        if (print_sink)   printf("	test2 = %f\n", test2);

   if (Px_rc1_x1 > Px_rc2_x2 )
      {
          if (print_sink)   printf ("	Px+1 > Px-1\n");
          if (Px_rc1_x1 - Px_rc_x >= epsilon1_ar && (max_tx_tries < x_upper))
          {   
               max_tx_tries = max_tx_tries++;
               if (print_process) printf ("%d\n",max_tx_tries);
               if (print_sink)   printf("	Increase the max_tx_tries: X = %d\n", max_tx_tries);
           } 
          else  
              {
                  if (print_process) printf ("%d\n",max_tx_tries);   
                  if (print_sink)   printf("	Remain the max_tx_tries: X = %d\n", max_tx_tries); 
               }

       }
   else
       { 
          if (print_sink)  printf("	Px-1 > Px+1 \n");  
          if ((test2 <= epsilon2_ar || Px_rc2_x2 - Px_rc_x >= epsilon2_ar) && (max_tx_tries > 1))
            {
                 max_tx_tries = max_tx_tries - 1;
                 if (print_process) printf ("%d\n",max_tx_tries);
                 if (print_sink)  printf("	Decrease the max_tx_tries: X = %d\n", max_tx_tries);
            } 
          else 
              {
                  if (print_process) printf ("%d\n",max_tx_tries);   
                  if (print_sink)   printf("	Remain the max_tx_tries: X = %d\n", max_tx_tries); 
               }

       }
 
   }
 }


//=========================================================================
  
//-------------------------------
void CsmaAr::exitBackoff()
{
  backoff_timer.stop();
  // backoff_duration = 0;
}


double CsmaAr::getBackoffTime()
{
  incrTotalBackoffTimes();
 
  double random = RNG::defaultrng()->uniform_double();

  backoff_timer.incrCounter();
  double counter = backoff_timer.getCounter();

  if ( counter > max_backoff_counter ) counter = max_backoff_counter;

  // backoff_duration = backoff_tuner * random * 2.0 * ACK_timeout * pow( 2.0, counter );
  // backoff_duration = backoff_tuner * random * 1.0 * ACK_timeout * pow( 2.0, counter ); //modified by tham
     backoff_duration = backoff_tuner * random * boff_factor * pow( 2.0, counter ); //ARS paper
     backoffSumDuration(backoff_duration);

  if (debug_){
       cout << NOW << "  CsmaAr("<< addr <<")::getBackoffTime() backoff time = " 
            << backoff_duration << " s " << " counter = " << counter  
            << " total backoff_times " << backoff_times_no << endl; //added by tham
             }

  if (debug_) printf ("%f CsmaAloha (%d)::getBackoffTime =%f \n ", NOW, addr , backoff_duration);// added by THAM 
  return backoff_duration;
}

void CsmaAr::recvFromUpperLayers(Packet* p)
{
  //printf("%f CsmaAr(%d) generate packet\n",NOW,addr); //Tham 
  if ( ((has_buffer_queue == true) && (Q.size() < buffer_pkts)) || (has_buffer_queue == false) ) 
  {
     initPkt(p , CSMA_DATA_PKT);
     Q.push(p);
     incrUpperDataRx();
     waitStartTime();

     if ( curr_state == CSMA_STATE_IDLE ) 
       {
         refreshReason(CSMA_REASON_DATA_PENDING);
         stateListen();
       }
  }
  else {
     incrDiscardedPktsTx();
     drop(p, 1, CSMA_DROP_REASON_BUFFER_FULL);
     drop_pkt++; // Tham
    // printf("%f Total dropped packets = %d\n",NOW,drop_pkt); // Tham
    // printf("%f Node(%d) drop pkt because bf full\n",NOW,addr);
  }
  
}

 void CsmaAr::initPkt( Packet* p, CSMA_PKT_TYPE type, int dest_addr) {
 //void CsmaAr::initPkt(Packet* p, CSMA_PKT_TYPE type, int dest_addr, int max_trans) { // modify 2
  hdr_cmn* ch = hdr_cmn::access(p);
  hdr_mac* mach = HDR_MAC(p);
  

  int curr_size = ch->size();

  switch(type) {
  
    case(CSMA_DATA_PKT): {
      ch->size() = curr_size + HDR_size;
      data_sn_queue.push(u_data_id);
      u_data_id++;
    } 
    break;
/* -- modify 2 --- */
    case(CSMA_ACK_PKT): {
      ch->ptype() = PT_MMAC_ACK;
      ch->size() = ACK_size;
      ch->uid() = u_pkt_id++;
      mach->macSA() = addr;
      mach->macDA() = dest_addr;
    //  mach->max_trans() = max_trans;
      }
    break;
  }

}


void CsmaAr::Mac2PhyStartTx(Packet* p) {
  if (debug_) cout << NOW << "  CsmaAr("<< addr <<")::Mac2PhyStartTx() start tx packet" << endl;
  
  MMac::Mac2PhyStartTx(p);
}


void CsmaAr::Phy2MacEndTx(const Packet* p) { 

  if (debug_) cout << NOW << "  CsmaAr("<< addr <<")::Phy2MacEndTx() end tx packet" << endl;

  switch(curr_state) {

    case(CSMA_STATE_TX_DATA): {
      refreshReason(CSMA_REASON_DATA_TX);
      if (ack_mode == CSMA_ACK_MODE) {

        if (debug_) cout << NOW << "  CsmaAr("<< addr <<")::Phy2MacEndTx() DATA sent,from "
                        << status_info[curr_state] << " to " 
                        << status_info[CSMA_STATE_WAIT_ACK] << endl;

        stateWaitAck(); 
      }
      else{

        if (debug_) cout << NOW << "  CsmaAr("<< addr <<")::Phy2MacEndTx() DATA sent, from " 
                        << status_info[curr_state] << " to " << status_info[CSMA_STATE_IDLE] << endl;
    
        stateIdle();
      }
    }
    break;

    case(CSMA_STATE_TX_ACK): {
      refreshReason(CSMA_REASON_ACK_TX);

      if ( prev_prev_state == CSMA_STATE_RX_BACKOFF ) {
        if (debug_) cout << NOW  << "  CsmaAr("<< addr <<")::Phy2MacEndTx() ack sent, from " 
                          << status_info[curr_state] << " to " << status_info[CSMA_STATE_CHK_BACKOFF_TIMEOUT] << endl;
          
        stateCheckBackoffExpired();
      }
      else if ( prev_prev_state == CSMA_STATE_RX_LISTEN ) {
        if (debug_) cout << NOW  << "  CsmaAr("<< addr <<")::Phy2MacEndTx() ack sent, from " 
                          << status_info[curr_state] << " to " << status_info[CSMA_STATE_CHK_LISTEN_TIMEOUT] << endl;
        
        stateCheckListenExpired();
      }
      else if ( prev_prev_state == CSMA_STATE_RX_IDLE ) {

        if (debug_) cout << NOW  << "  CsmaAr("<< addr <<")::Phy2MacEndTx() ack sent, from " 
                         << status_info[curr_state] << " to " << status_info[CSMA_STATE_IDLE] << endl;

        stateIdle();
      } 
      else {
      
        cout << NOW << "  CsmaAr("<< addr <<")::Phy2MacEndTx() logical error in timers, current state = " 
              << status_info[curr_state] << endl;
        stateIdle();
      }
    }
    break;

    default: {
        cout << NOW << "  CsmaAr("<< addr <<")::Phy2MacEndTx() logical error, current state = " 
             << status_info[curr_state] << endl;
       stateIdle(); 
    }
    break;

  }

}


void CsmaAr::Phy2MacStartRx(const Packet* p) {
  if (debug_) cout << NOW << "  CsmaAr("<< addr <<")::Phy2MacStartRx() rx Packet " << endl; 


  refreshReason(CSMA_REASON_START_RX);

  switch(curr_state) { 
    
    case(CSMA_STATE_IDLE): 
      stateRxIdle();
    break;
    
    case(CSMA_STATE_LISTEN): 
      stateRxListen();
    break;
      
    case(CSMA_STATE_BACKOFF): 
      stateRxBackoff();
    break;
      
    case(CSMA_STATE_WAIT_ACK): 
      stateRxWaitAck();
    break;
    
    default: {
      cerr << NOW << "  CsmaAr("<< addr << ")::Phy2MacStartRx() logical warning, current state = " 
           << status_info[curr_state] << endl;
      stateIdle();
    }
    
  }
}


void CsmaAr::Phy2MacEndRx(Packet* p) {

 
  hdr_cmn* ch = HDR_CMN(p);
  packet_t rx_pkt_type = ch->ptype();
  hdr_mac* mach = HDR_MAC(p);
  hdr_MPhy* ph = HDR_MPHY(p);

  int source_mac = mach->macSA();
  int dest_mac = mach->macDA();

  double gen_time = ph->txtime;
  double received_time = ph->rxtime;
  double diff_time = received_time - gen_time;

  double distance = diff_time * prop_speed;

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::Phy2MacEndRx() " 
                   << status_info[curr_state] << ", received a pkt type = " 
                   << ch->ptype() << ", src addr = " << mach->macSA() 
                   << " dest addr = " << mach->macDA() 
                   << ", estimated distance between nodes = " << distance << " m " << endl;

  if ( ch->error() ) {

    if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::Phy2MacEndRx() dropping corrupted pkt " << endl;
    incrErrorPktsRx();

    refreshReason(CSMA_REASON_PKT_ERROR);
    drop(p, 1, CSMA_DROP_REASON_ERROR);
    stateRxPacketNotForMe(NULL);
  }
  else {
    if ( dest_mac == addr || dest_mac == MAC_BROADCAST ) {
       if ( rx_pkt_type == PT_MMAC_ACK ) {
      // if (rx_pkt_type == PT_CSMAAR_ACK) {
        refreshReason(CSMA_REASON_ACK_RX);
        stateRxAck(p);
      }
      else if ( curr_state != CSMA_STATE_RX_WAIT_ACK ) {
        refreshReason(CSMA_REASON_DATA_RX);
        stateRxData(p);
      }
      else {
        refreshReason(CSMA_REASON_PKT_NOT_FOR_ME);
        stateRxPacketNotForMe(p);
      }
    }
    else {
      refreshReason(CSMA_REASON_PKT_NOT_FOR_ME);
      stateRxPacketNotForMe(p);
    }
  }
}

void CsmaAr::txData()
{ 
  Packet* data_pkt = curr_data_pkt->copy();  
 
  if ( (ack_mode == CSMA_NO_ACK_MODE) ) {
     queuePop();
   }
  total_retrans++;                                          
  incrDataPktsTx();
  incrCurrTxRounds();
  Mac2PhyStartTx(data_pkt); 
  /* Increase the number of transmission at current time by 1 //added by Tham Ng, UOU Korea 2012 */
  num_trans_current_ar++;
  total_packet_trans_ar++; // calculate total of packet is transmitted in MAC layer (trans and re-trans)
 // printf("Node %d total packet trans %d \n", addr, total_packet_trans_ar);
 // printf("time %f node %d, num curr: %d\n",NOW,addr, num_trans_current_ar); //Tham Ng
}

 void CsmaAr::txAck( int dest_addr )
 // void CsmaAr::txAck(int dest_addr, int max_trans)
{
  Packet* ack_pkt = Packet::alloc();
  initPkt( ack_pkt , CSMA_ACK_PKT, dest_addr);
  //initPkt(ack_pkt, CSMA_ACK_PKT, dest_addr, max_trans); // modify 2
  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::time-sending-ACK " << NOW << endl; //added by tham
  incrAckPktsTx();
  ack_sent++;
  Mac2PhyStartTx(ack_pkt);
}

void CsmaAr::stateRxPacketNotForMe(Packet* p) {
  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateRxPacketNotForMe() pkt for another address. Dropping pkt" << endl;
  if ( p != NULL ) Packet::free(p);
  
  refreshState( CSMA_STATE_WRONG_PKT_RX );
  
  switch( prev_state ) {
  
    case CSMA_STATE_RX_IDLE:
      stateIdle();
      break;
      
    case CSMA_STATE_RX_LISTEN:
      stateCheckListenExpired();
      break;
      
    case CSMA_STATE_RX_BACKOFF:
      stateCheckBackoffExpired();
      break;
      
    case CSMA_STATE_RX_WAIT_ACK:
      stateCheckAckExpired();
      break;
      
    default:
      cerr << NOW << "  CsmaAr("<< addr << ")::stateRxPacketNotForMe() logical error, current state = " 
           << status_info[curr_state] << endl;
      stateIdle();
      
  }
}


void CsmaAr::stateCheckListenExpired() { 
  refreshState(CSMA_STATE_CHK_LISTEN_TIMEOUT);

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateCheckListenExpired()" << endl;
  if (print_transitions) printStateInfo();
  if ( listen_timer.isActive() ) {
    refreshReason( CSMA_REASON_LISTEN_PENDING );
    refreshState( CSMA_STATE_LISTEN );
  }
  else if ( listen_timer.isExpired() ) {
    refreshReason( CSMA_REASON_LISTEN_TIMEOUT );
    if ( !( prev_state == CSMA_STATE_TX_ACK || prev_state == CSMA_STATE_WRONG_PKT_RX
         || prev_state == CSMA_STATE_ACK_RX || prev_state == CSMA_STATE_DATA_RX ) ) stateTxData();
    else stateListen();
  }
  else {
    cerr << NOW << "  CsmaAr("<< addr << ")::stateCheckListenExpired() listen_timer logical error, current timer state = " 
         << status_info[curr_state] << endl;
    stateIdle();  
  }
}


void CsmaAr::stateCheckAckExpired() {
  refreshState(CSMA_STATE_CHK_ACK_TIMEOUT);

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateCheckAckExpired()" << endl;
  if (print_transitions) printStateInfo();
  if ( ack_timer.isActive() ) {
    refreshReason( CSMA_REASON_WAIT_ACK_PENDING );
    refreshState( CSMA_STATE_WAIT_ACK );
  }
  else if ( ack_timer.isExpired() ) {
    refreshReason( CSMA_REASON_ACK_TIMEOUT );
    stateBackoff();
  }
  else {
    cerr << NOW << "  CsmaAr("<< addr << ")::stateCheckAckExpired() ack_timer logical error, current timer state = " 
         << status_info[curr_state] << endl;
    stateIdle();  
  }
}


void CsmaAr::stateCheckBackoffExpired() {
  refreshState(CSMA_STATE_CHK_BACKOFF_TIMEOUT);

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateCheckBackoffExpired()" << endl;
  if (print_transitions) printStateInfo();
  if ( backoff_timer.isActive() ) {
    refreshReason( CSMA_REASON_BACKOFF_PENDING );
    stateBackoff();
  }
  else if ( backoff_timer.isExpired() ) {
    refreshReason( CSMA_REASON_BACKOFF_TIMEOUT );
    exitBackoff();
    stateIdle();
  }
  else {
    cerr << NOW << "  CsmaAr("<< addr << ")::stateCheckBackoffExpired() backoff_timer logical error, current timer state = " 
         << status_info[curr_state] << endl;
    stateIdle();  
  }
}
  
  
void CsmaAr::stateIdle() {
  ack_timer.stop();
  backoff_timer.stop();
  listen_timer.stop();
  resetSession();
  
  refreshState(CSMA_STATE_IDLE);

  if (print_transitions) printStateInfo();

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateIdle() queue size = " << Q.size() << endl;

  if ( !Q.empty() ) {
    refreshReason(CSMA_REASON_LISTEN);
    stateListen();
  }
}


void CsmaAr::stateRxIdle() {
  refreshState(CSMA_STATE_RX_IDLE);

  if (print_transitions) printStateInfo();
}


void CsmaAr::stateListen() {
  listen_timer.stop();
  refreshState(CSMA_STATE_LISTEN);

  listen_timer.incrCounter();
  
  double time = listen_time * RNG::defaultrng()->uniform_double() + wait_costant;

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateListen() listen time = " << time << endl;

  if (print_transitions) printStateInfo();

  listen_timer.schedule( time );
}


void CsmaAr::stateRxListen() {
  refreshState(CSMA_STATE_RX_LISTEN);

  if (print_transitions) printStateInfo();
}


void CsmaAr::stateBackoff() {
  refreshState(CSMA_STATE_BACKOFF);

  if ( backoff_timer.isFrozen() ) 
                backoff_timer.unFreeze();

  else backoff_timer.schedule( getBackoffTime() );

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateBackoff() " << endl;
  if (print_transitions) printStateInfo(backoff_timer.getDuration());
}


void CsmaAr::stateRxBackoff() {
  backoff_timer.freeze();
  refreshState(CSMA_STATE_RX_BACKOFF);

  if (print_transitions) printStateInfo();
}


/* --- modify 2 ------*/

void CsmaAr::stateTxData()
{
 // if (NOW > time_to_check_bf)
 // printf("%f Node(%d) has buffer queue = %d packet\n",NOW,addr,Q.size()); // for check buffer

  refreshState(CSMA_STATE_TX_DATA);
  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateTxData() " << endl;
  if (print_transitions) printStateInfo();

  curr_data_pkt = Q.front();
  hdr_cmn *ch = HDR_CMN(curr_data_pkt);
  hdr_mac* mach = HDR_MAC(curr_data_pkt);

  if (data_sn_queue.front() != last_sent_data_id) {
     resetCurrTxRounds();
     ack_timer.resetCounter();
     listen_timer.resetCounter();
     backoff_timer.resetCounter(); 
     original_current_ar++; // in time window
     total_orig++;
     mach->first() = 1;
    // printf("Node %d Number of packet sent in Time window %d\n", addr, original_current_ar);
  }
 else mach->first() = 0;
  //printf("%f CsmaAr(%d)::stateTxData(): max_tx_tries = %d\n", NOW, addr,max_tx_tries);
  if ( curr_tx_rounds < max_tx_tries ) { 
    // hdr_mac* mach = HDR_MAC(curr_data_pkt);
     mach->macSA() = addr;
     start_tx_time = NOW;

     mach->node_background_traffic_ar() = node_background_traffic_ar; // traffic include retransmission
     mach->node_original_traffic_ar() = node_original_traffic_ar; // original traffic

     if ( data_sn_queue.front() == last_sent_data_id) {
          re_trans_no = re_trans_no++;//added by Tham 
         // printf (" %f CsmaAloha Node (%d): total re-trans-times =%d \n ", NOW, addr , re_trans_no);// added by THAM  
     }

     last_sent_data_id = data_sn_queue.front();
     txData();
     
  }
  else {
   
    queuePop(false);
    incrDroppedPktsTx();
     
    refreshReason(CSMA_REASON_MAX_TX_TRIES);

    if (debug_) cout << NOW << "CsmaAr("<< addr << ")::stateTxData() curr_tx_rounds " << curr_tx_rounds
                     << " > max_tx_tries = " << max_tx_tries << endl;

    stateIdle();
  }

 //----colect the total number of retrans-----
  //printf (" %f CsmaAr Node (%d): total re-trans-times =%d\n ", NOW, addr , re_trans_no);// added by THAM 
   
}
void CsmaAr::stateWaitAck() {
  ack_timer.stop();
  refreshState(CSMA_STATE_WAIT_ACK);

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateWaitAck() " ;
  if (print_transitions) printStateInfo();
  
  ack_timer.incrCounter();
  ack_timer.schedule(ACK_timeout + 2*wait_costant); 
  acktimeout = (ACK_timeout + 2*wait_costant); 
  if (debug_) cout << "ack_timer = " << acktimeout << endl;//added by THAM
}


void CsmaAr::stateRxWaitAck() {
  refreshState(CSMA_STATE_RX_WAIT_ACK);

  if (print_transitions) printStateInfo();
}


 void CsmaAr::stateTxAck( int dest_addr) { // initial
 //oid CsmaAr::stateTxAck(int dest_addr, int max_trans) {  // modify 2
  refreshState(CSMA_STATE_TX_ACK);

  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateTxAck() dest addr " << dest_addr << endl;
  if (print_transitions) printStateInfo();
 
   txAck( dest_addr );
 
    //txAck(dest_addr, max_trans); // modify 2

}

/* ------- modify 2nd ------------*/

void CsmaAr::stateRxData(Packet* data_pkt) {
  refreshState( CSMA_STATE_DATA_RX );
  
  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateRxData() in state " << status_info[curr_state] << endl;
  refreshReason( CSMA_REASON_DATA_RX );
  hdr_mac* mach = HDR_MAC(data_pkt);
  hdr_cmn *ch = HDR_CMN(data_pkt);//add by tham
  dst_addr = mach->macSA();

// --- add by tham ----
   //if (mach->first() == 1) recv_first_frame++;
  recv_frame++;
  
  size_ar = ch->size(); // Tham Nguyen 2012
 if (debug_) printf(" Packet size at MAC %d\n",ch->size());
  getTransTime_Ar(size_ar); //Tham Nguyen 2012      
    double background_traffic = mach->node_background_traffic_ar(); //tham
    double original_traffic = mach->node_original_traffic_ar(); //tham

   for (int i=0,n=99; i < number_of_flow_ar; i++) {
      if ((dst_addr == i) && (background_traffic != 0) && (original_traffic != 0) ) {
         Node_Array[i] = dst_addr;
         BackgroundTraffic_Array[i] = background_traffic;
         OriginalTraffic_Array[i] = original_traffic;
       }
    }

//======== Count the packet recv at sink (do not take acount duplicate pkts)
   int packet_id = ch->uid();
   int i = dst_addr;
//for (int i= 0 ; i < 50; i++) {
 //if (i == dst_addr) {
   for (int j=0 ; j < 500; j++) 
      {
       if (matrix_t[i][j] == -1) 
       {
         matrix_t[i][j] = packet_id;
         recv_first_frame++;
       // printf("%d \n",matrix_t[i][j]);
         break;
       } 
       else if (matrix_t[i][j] == packet_id)
              {
                break;
              }                               
      }

 //  break;
 //}
//} 
  // printf("recv_frame = %d\n",recv_frame);
//===============================

 /*      
  for (int i=0,n=99; i < number_of_node_ar; i++) {
      cout<< Node_Array[i] << " ";
    }
      cout<< "\n";
   for (int i=0,n=99; i < number_of_node_ar; i++) {
      cout<< BackgroundTraffic_Array[i] << " ";
    }
      cout << "\n";
   for (int i=0,n=99; i < number_of_node_ar; i++) {
      cout<< OriginalTraffic_Array[i] << " ";
    }
      cout << "\n";

 */

//-----------
  switch( prev_state ) {  
    case CSMA_STATE_RX_IDLE: {
      hdr_cmn* ch = hdr_cmn::access(data_pkt);
      ch->size() = ch->size() - HDR_size;
      incrDataPktsRx();
      sendUp(data_pkt);

      if (ack_mode == CSMA_ACK_MODE) stateTxAck(dst_addr);
        
      // if (ack_mode == CSMA_ACK_MODE) stateTxAck(dst_addr, MaxTrans_Array[dst_addr]); //tham
      else stateIdle();
    }
    break;
      
    case CSMA_STATE_RX_LISTEN: {
      hdr_cmn* ch = hdr_cmn::access(data_pkt);
      ch->size() = ch->size() - HDR_size;
      incrDataPktsRx();
      sendUp(data_pkt);

      if (ack_mode == CSMA_ACK_MODE) stateTxAck(dst_addr);
       //if (ack_mode == CSMA_ACK_MODE) stateTxAck(dst_addr, MaxTrans_Array[dst_addr]); //tham
      else stateCheckListenExpired();
    }
    break;
   
    case CSMA_STATE_RX_BACKOFF: {
      hdr_cmn* ch = hdr_cmn::access(data_pkt);
      ch->size() = ch->size() - HDR_size;
      incrDataPktsRx();
      sendUp(data_pkt);       
      if (ack_mode == CSMA_ACK_MODE) stateTxAck(dst_addr);
       // if (ack_mode == CSMA_ACK_MODE) stateTxAck(dst_addr, MaxTrans_Array[dst_addr]); //tham
      else stateCheckBackoffExpired();
    }
    break;
            
    default: 

      cerr << NOW << " CsmaAr("<< addr << ")::stateRxData() logical error, prev state = " << status_info[prev_state]
           << endl;

  }
 
}

/* ------------- modify 2nd -------------------*/

void CsmaAr::stateRxAck(Packet* p) {
  ack_timer.stop();
  refreshState(CSMA_STATE_ACK_RX);
  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::stateRxAck() " << endl;
  //----
 // hdr_mac* mach = HDR_MAC(p); //Tham Ng
 // max_tx_tries = mach->max_trans();
  //printf("Node %d: maximum number of trans %d\n",addr,mach->max_trans());
  //----
  Packet::free(p);
  if (debug_) cout << NOW << "  CsmaAr("<< addr << ")::time-rec-ACK "<< NOW << endl; //added by tham
  refreshReason(CSMA_REASON_ACK_RX);

  ack_rcv++;

  switch( prev_state ) {
    
    case CSMA_STATE_RX_IDLE:
      stateIdle();
      break;
      
    case CSMA_STATE_RX_LISTEN:
      stateCheckListenExpired();
      break;
      
    case CSMA_STATE_RX_BACKOFF:
      stateCheckBackoffExpired();
      break;
      
    case CSMA_STATE_RX_WAIT_ACK:
      queuePop();
     //updateAckTimeout(NOW - start_tx_time); //modified by THAM
      //double time_rtt = NOW - start_tx_time;//added by tham
     // printf ("time: %f \n", time); //added by tham
      incrAckPktsRx();
      stateIdle();      
      break;
      
    default: 

      cerr << NOW << " CsmaAr("<< addr << ")::stateRxData() logical error, prev state = " << status_info[prev_state]
           << endl;
  }
}
void CsmaAr::printStateInfo(double delay)
{
  if (debug_) cout << NOW << " CsmaAr("<< addr << ")::printStateInfo() " << "from " << status_info[prev_state] 
                   << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] << endl;

  if (curr_state == CSMA_STATE_BACKOFF) {
      fout <<left << setw(10) << NOW << "  CsmaAr("<< addr << ")::printStateInfo() " 
           << "from " << status_info[prev_state] 
           << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] 
           << ". Backoff duration = " << delay << endl;
  }
  else {
      fout << left << setw(10) << NOW << "  CsmaAr("<< addr << ")::printStateInfo() " 
           << "from " << status_info[prev_state] 
           << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] << endl;
  }
}

void CsmaAr::waitForUser()
{
  std::string response;
  std::cout << "Press Enter to continue";
  std::getline(std::cin, response);
} 

