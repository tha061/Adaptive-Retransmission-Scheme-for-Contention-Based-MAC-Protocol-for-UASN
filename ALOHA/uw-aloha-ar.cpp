/* -*- Mode:C++ -*- */

/*
 * Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University of Padova (SIGNET lab) nor the 
 *    names of its contributors may be used to endorse or promote products 
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file   uwaloha.cpp
 * @author Saiful Azad
 * @version 1.0.0
 * 
 * @brief Class that provide the implementation of ALOHA protocol
 */


#include "uw-aloha-ar.h"
#include <mac.h>
#include <cmath>
#include <climits>
#include <iomanip>
#include <rng.h>


enum {
    NOT_SET = -1, SESSION_DISTANCE_NOT_SET = 0
};

/**
 * Class that describe the binding with tcl scripting language
 */
static class ALOHAARModuleClass : public TclClass 
{
public:
  /**
   * Constructor of the class
   */
  ALOHAARModuleClass() : TclClass("Module/UW/ALOHA_AR") {}
  TclObject* create(int, const char*const*) {
    return (new AlohaAr ());
  }
} class_module_alohaar;


void AlohaAr ::AckTimer::expire(Event *e) 
{
  
  if (module->curr_state == UWALOHA_STATE_WAIT_ACK) {


        if (module->alohaar_debug) cout << NOW << "  AlohaAr ("<< module->addr << ") timer expire() current state = " 
                         << module->status_info[module->curr_state] << "; ACK not received, next state = " 
                        << module->status_info[UWALOHA_STATE_BACKOFF] << endl;

        module->refreshReason(UWALOHA_REASON_ACK_TIMEOUT);
        module->stateBackoff(); 
  }
  else {
    if (module->alohaar_debug ) cout << NOW << "  AlohaAr ("<< module->addr << ")::AckTimer::expired() " << endl;
  }  
}


void AlohaAr ::BackOffTimer::expire(Event *e) 
{
  if (module->curr_state == UWALOHA_STATE_BACKOFF) {
        if (module->alohaar_debug) cout << NOW << "  AlohaAr ("<< module->addr << ") timer expire() current state = " 
                         << module->status_info[module->curr_state] << "; backoff expired, next state = " 
                         << module->status_info[UWALOHA_STATE_IDLE] << endl;

        module->refreshReason(UWALOHA_REASON_BACKOFF_TIMEOUT);
	module->exitBackoff();
        module->stateIdle(); 
  }
  else {
   if (module->alohaar_debug ) cout << NOW << "  AlohaAr ("<< module->addr << ")::BackOffTimer::expired() " << endl;
  }  
}

//--- Timer adding for ALOHA-AR (Tham Nguyen 2013/Aug)-----------------------
/*

void AlohaAr::WindowTimer_Ar::expire(Event *e) {   
   timer_status = UWALOHA_EXPIRED;
  // printf("%f AlohaAr::WindowTimer_Ar::expire\n",NOW);
//   module->getTrafficWithinTimeWd_Ar(); // for modify 2
  // module->sinkProcess();
}
*/

void AlohaAr::SinkTimer::expire(Event *e) {
  timer_status = UWALOHA_EXPIRED;
  int sink_id = module->number_of_node_ar;
  if (module->addr == sink_id) 
  module->sinkProcess();
 }
//---------------------------------------------------------------------------


const double AlohaAr ::prop_speed = 1500.0;
bool AlohaAr ::initialized = false;

//----Tham's adding---------------------
int AlohaAr::max_tx_tries;
int AlohaAr::total_orig = 0;
int AlohaAr::recv_frame = 0;
int AlohaAr::total_retrans = 0;
int AlohaAr::ack_rcv = 0;
int AlohaAr::recv_first_frame = 0;
int AlohaAr::drop_pkt = 0;
double AlohaAr::time_window_ar;
double AlohaAr::time_tr_ar;
//--------------------------------------

map< AlohaAr ::UWALOHA_STATUS , string> AlohaAr ::status_info;
map< AlohaAr ::UWALOHA_REASON_STATUS, string> AlohaAr ::reason_info;
map< AlohaAr ::UWALOHA_PKT_TYPE, string> AlohaAr ::pkt_type_info;

AlohaAr ::AlohaAr () 
: ack_timer(this),
  backoff_timer(this),
 // window_timer_ar(this), //added by Tham Ng, UOU korea 2012
  sink_timer(this),
  last_sent_data_id(-1),
  txsn(1),
  curr_data_pkt(0),
  last_data_id_rx(NOT_SET),
  curr_tx_rounds(0),
  print_transitions(false),
  curr_state(UWALOHA_STATE_IDLE), 
  prev_state(UWALOHA_STATE_IDLE),
  prev_prev_state(UWALOHA_STATE_IDLE),
  ack_mode(UWALOHA_NO_ACK_MODE),
  last_reason(UWALOHA_REASON_NOT_SET),
  start_tx_time(0),     
  recv_data_id(-1),
  srtt(0),      
  sumrtt(0),      
  sumrtt2(0),     
  rttsamples(0),
//---Tham's adding---------------
  re_trans_no(0),  
  total_packet_trans(0),
  total_packet_recv(0),
  has_buffer_queue(false), //added by Tham
  mac_pkt_recv(0), //tham
  mac_pkt_recv_of_spec_node(0), //tham
  num_trans_current_ar(0),//added by Tham
  e(2.71828),
  ack_sent(0),
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
  n10(0)
//-----------------------------
// last_recv_data_id(-1)
{ 
  mac2phy_delay_ = 1e-19;
  
  bind("HDR_size_", (int*)& HDR_size);
  bind("ACK_size_", (int*)& ACK_size);
  bind("max_tx_tries_", (int*)& max_tx_tries);
  bind("wait_constant_", (double*)& wait_constant);
  bind("alohaar_debug_", (int*)& alohaar_debug); //debug mode
  bind("max_payload_", (int*)& max_payload);
  bind("ACK_timeout_", (double*)& ACK_timeout);
  bind("alpha_", (double*)&alpha_);
  bind("buffer_pkts_", (int*)&buffer_pkts);
  bind("backoff_tuner_", (double*)&backoff_tuner);
  bind("max_backoff_counter_", (int*)&max_backoff_counter);
  //----Tham's adding-----------
 // bind("spec_node_id_", (int*)& spec_node_id);
  bind("stop_time_", (double*)& stop_time);
  bind("tham_debug_", (int*)& tham_debug); // added by Tham
  bind("number_of_node_ar_",(int*)&number_of_node_ar); //tham
  bind("bit_rate_",(double*)&bit_rate); // tham
  bind("interval_",(double*)&interval);  // Tham

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
 // window_timer_ar.schedule(time_window_ar); //added by Tham Ng, UOU Korea 2012
 // printf("time window = %f",time_window_ar);
  sum_of_traffic_ar = max_tx_tries * number_of_flow_ar;
  total_background_traffic_ar = 1;
  total_original_traffic_ar = 1;
  node_background_traffic_ar = 0;
  node_original_traffic_ar = 0;
  max_tx_tries_curr = max_tx_tries;
  max_starting = max_tx_tries;
//--------------------------                          
                     
  if (max_tx_tries <= 0) max_tx_tries = INT_MAX;
  if (buffer_pkts > 0) has_buffer_queue = true;
}

AlohaAr ::~AlohaAr ()
{

}

// TCL command interpreter
int AlohaAr ::command(int argc, const char*const* argv)
{
  Tcl& tcl = Tcl::instance();
  if (argc==2)
    {
      if(strcasecmp(argv[1], "setAckMode") == 0)
	{
	  ack_mode = UWALOHA_ACK_MODE;
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "setNoAckMode") == 0)	
	{
          ack_mode = UWALOHA_NO_ACK_MODE;
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "initialize") == 0)	
	{
          if (initialized == false) initInfo();          
          if (print_transitions) fout.open("/tmp/ALOHAARstateTransitions.txt",ios_base::app);
 // Initiate for Aloha-Ar
          //initArrayOfNode_Ar();
         // initArrayBackgroundTrafficOfEachNode_Ar(); // modify 2
         // initArrayOriginalTrafficOfEachNode_Ar();   // modify 2
         // initArrayMaxTransOfNode_Ar(); // For modify 2
           initMatrixPacketRecv_Ar();
           sink_timer.schedule(sink_time_window); // Sink Timer
         // window_timer_ar.schedule(time_window_ar);
	  return TCL_OK;
	}
      else if(strcasecmp(argv[1], "printTransitions") == 0)	
	{
          print_transitions = true;
	  return TCL_OK;
	}
      // stats functions
      else if(strcasecmp(argv[1], "getQueueSize") == 0)	
	{
	  tcl.resultf("%d",mapPacket.size());
      	  return TCL_OK;
        }
    }
    else if(argc==3){
		if(strcasecmp(argv[1],"setMacAddr") == 0)
		{
			addr = atoi(argv[2]);
			if(alohaar_debug) cout << "AlohaAr MAC address of current node is " << addr <<endl;
			return TCL_OK;
		}
	}
  return MMac::command(argc, argv);
}

void AlohaAr ::initInfo()
{

  initialized = true;

  if ( (print_transitions) && (system(NULL)) ) {
      system("rm -f /tmp/ALOHAARstateTransitions.txt");
      system("touch /tmp/ALOHAARstateTransitions.txt");
  }

  status_info[UWALOHA_STATE_IDLE] = "Idle state";
  status_info[UWALOHA_STATE_TX_DATA] = "Transmit DATA state"; 
  status_info[UWALOHA_STATE_TX_ACK] = "Transmit ACK state";
  status_info[UWALOHA_STATE_WAIT_ACK] = "Wait for ACK state"; 
  status_info[UWALOHA_STATE_DATA_RX] = "DATA received state"; 
  status_info[UWALOHA_STATE_ACK_RX] = "ACK received state"; 
  status_info[UWALOHA_STATE_RX_IDLE] = "Start rx Idle state";
  status_info[UWALOHA_STATE_RX_WAIT_ACK] = "Start rx Wait ACK state";
  status_info[UWALOHA_STATE_CHK_ACK_TIMEOUT] = "Check Wait ACK timeout state";
  status_info[UWALOHA_STATE_WRONG_PKT_RX] = "Wrong Pkt Rx state";
  status_info[UWALOHA_STATE_BACKOFF] = "Backoff state";
  status_info[UWALOHA_STATE_RX_BACKOFF] = "Start rx Backoff state";
  status_info[UWALOHA_STATE_CHK_BACKOFF_TIMEOUT] = "Check Backoff timeout state";
  status_info[UWALOHA_STATE_TX_MAC_PKT] = "Transmit MAC pkt state"; // tham
  
  reason_info[UWALOHA_REASON_DATA_PENDING] = "DATA pending from upper layers"; 
  reason_info[UWALOHA_REASON_DATA_RX] = "DATA received";
  reason_info[UWALOHA_REASON_DATA_TX] = "DATA transmitted"; 
  reason_info[UWALOHA_REASON_ACK_TX] = "ACK tranmsitted";
  reason_info[UWALOHA_REASON_ACK_RX] = "ACK received"; 
  reason_info[UWALOHA_REASON_ACK_TIMEOUT] = "ACK timeout"; 
  reason_info[UWALOHA_REASON_DATA_EMPTY] = "DATA queue empty";
  reason_info[UWALOHA_REASON_MAX_TX_TRIES] = "DATA dropped due to max tx rounds";
  reason_info[UWALOHA_REASON_START_RX] = "Start rx pkt";
  reason_info[UWALOHA_REASON_PKT_NOT_FOR_ME] = "Received an erroneous pkt";
  reason_info[UWALOHA_REASON_WAIT_ACK_PENDING] = "Wait for ACK timer pending";
  reason_info[UWALOHA_REASON_PKT_ERROR] = "Erroneous pkt";
  reason_info[UWALOHA_REASON_BACKOFF_TIMEOUT] = "Backoff expired";
  reason_info[UWALOHA_REASON_BACKOFF_PENDING] = "Backoff timer pending";
  reason_info[UWALOHA_REASON_MAC_PENDING] = "MAC pending"; // tham
  reason_info[UWALOHA_REASON_MAC_TX] = "MAC pkt transmitted";// tham
  
  pkt_type_info[UWALOHA_ACK_PKT] = "ACK pkt";
  pkt_type_info[UWALOHA_DATA_PKT] = "DATA pkt"; 
  pkt_type_info[UWALOHA_DATAMAX_PKT] = "MAX payload DATA pkt";
  pkt_type_info[UWALOHA_MAC_PKT] = "MAC pkt"; // tham
}
//=====modified by THAM
/*void UWAloha ::updateRTT(double curr_rtt)
{
  srtt = alpha_ * srtt + (1-alpha_) * curr_rtt;
  sumrtt += curr_rtt;
  sumrtt2 += curr_rtt*curr_rtt;
  rttsamples++;
  ACK_timeout = (sumrtt / rttsamples);
}

void UWAloha ::updateAckTimeout(double rtt) {
  updateRTT(rtt);
//   double curr_rtt = getRTT();

//   if (curr_rtt > 0) ACK_timeout = min(ACK_timeout, getRTT() );

   if (uwaloha_debug) cout << NOW << "  UWAloha (" << addr << ")::updateAckTimeout() curr ACK_timeout = " 
                   << ACK_timeout << endl;
//   waitForUser();
}*/
//===============

  /*
// For reschedule Packet interval - tham
void UWAloha::reschedule_MacTimer()
 {
  mac_traffic_timer_ar.resched(T);
 }

//--For initiate Jitter (reschedule) - tham
void UWAloha::initJitter_Ar()
 {
 // r_t = T/2;
  //random_ar = RNG::defaultrng()->uniform(r_t);
  //transmission_time = T - r_t/2 + random_ar;
  random_ar = RNG::defaultrng()->uniform(T);
  transmission_time = random_ar;
  pkt_init_timer.resched(transmission_time);
}
//----
   */

//----- Adding functions for Aloha-AR--------------------------------------------------------


// Get the transmission time: time_tr_ar : Tham Nguyen 2012
int AlohaAr::getTransTime_Ar(int size_ar)
  {
   packet_size_ar = size_ar;
  // printf("packet size = %d\n",packet_size_ar);
   time_tr_ar = (packet_size_ar*8)/bit_data_rate_ar;
  // printf("time_tr = %f\n",time_tr_ar);
   return (time_tr_ar);
   }


/*
//Get traffic at each node
double AlohaAr::getTrafficWithinTimeWd_Ar()
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
    
  
   return(node_background_traffic_ar,node_original_traffic_ar);
}

// Init the array to store the packet's sourse addr at sink: Tham Ng 2012
void AlohaAr::initArrayOfNode_Ar()
 {
   for (int i = 0, n = 99; i<=n; i++)
    { 
        Node_Array[i] = -1;
     }
  }
void AlohaAr::initArrayMaxTransOfNode_Ar()
{
  for (int i = 0, n = 99; i<=n; i++)
  {
       MaxTrans_Array[i] = max_tx_tries;
     // cout << MaxTrans_Array[i] << " ";
  } 
     // cout << "\n";
}

void AlohaAr::initArrayBackgroundTrafficOfEachNode_Ar()
 {
   for (int i=0,n=99; i<=n; i++) 
     {
        BackgroundTraffic_Array[i] = 0;
     }
  }
void AlohaAr::initArrayOriginalTrafficOfEachNode_Ar()
 {
  for (int i=0,n=99; i<=n; i++)
  {
    OriginalTraffic_Array[i] = 0;
  }
}
*/


void AlohaAr::initMatrixPacketRecv_Ar() //For calculate instaneous PDR
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

// Sink process
void AlohaAr::sinkProcess() 
 {
   //printf("%f AlohaAr(%d)::SinkTimer::expire\n",NOW,addr);
   sink_timer.resched(sink_time_window);
 //--new-idea
    //double alpha_t = 0.9;
     Rs = total_retrans/sink_time_window;
     Ot_curr = total_orig/sink_time_window;
    // printf("total_orig = %d\n",total_orig);
   //  printf("time_trans = %f\n",time_tr_ar);
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
     // printf("	total_recv = %d\n",recv_frame);
     // printf("	total_orig = %d\n",total_orig);
     // printf("	total_recv_first = %d\n",recv_first_frame);     
     // printf("	rc = %f\n",rc);
    //  printf("	Rs = %f\n",Rs);
     // printf("	Rm = %f\n",Rm);
     // printf("	Ot_curr = %f\n",Ot_curr);
     // printf("	Ot_m = %f\n", Ot_m);
      if (print_sink) 
    { // printf("Px_curr = %f\n", Px_curr);
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
    if (print_process) printf("%f ",Px_rc2_x2);
    if (print_sink)  printf("	Px(rc-delta_r, X-1) = %f\n",Px_rc2_x2);
 
//=================================================================================
             // Change the max_tx_tries if the gain > epsilon

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
               if (print_process) printf("%d\n",max_tx_tries);
               if (print_sink)   printf("	Increase the max_tx_tries: X = %d\n", max_tx_tries);
           } 
          else  
              {
                  if (print_process) printf("%d\n",max_tx_tries);   
                  if (print_sink)   printf("	Remain the max_tx_tries: X = %d\n", max_tx_tries); 
               }

       }
   else
       { 
          if (print_sink)  printf("	Px-1 > Px+1 \n");  
          if ((test2 <= epsilon2_ar || Px_rc2_x2 - Px_rc_x >= epsilon2_ar) && (max_tx_tries > 1))
            {
                 max_tx_tries = max_tx_tries - 1;
                 if (print_process) printf("%d\n",max_tx_tries);
                 if (print_sink)  printf("	Decrease the max_tx_tries: X = %d\n", max_tx_tries);
            } 
          else 
              {
                  if (print_process) printf("%d\n",max_tx_tries);   
                  if (print_sink)   printf("	Remain the max_tx_tries: X = %d\n", max_tx_tries); 
               }

       }
 
   }
 }

//-------------------------------------------------------------------------------------------

void AlohaAr ::exitBackoff()
{
  backoff_timer.stop();
}


double AlohaAr ::getBackoffTime()
{
  incrTotalBackoffTimes();
  double random = RNG::defaultrng()->uniform_double();

  backoff_timer.incrCounter();
  double counter = backoff_timer.getCounter();
  if ( counter > max_backoff_counter ) counter = max_backoff_counter;

 // double backoff_duration = backoff_tuner * random * 2.0 * ACK_timeout * pow( 2.0, counter ); //original code
 // double backoff_duration = backoff_tuner * random * 1.0 * ACK_timeout * pow( 2.0, counter ); // modified 1st by Tham
           backoff_duration = backoff_tuner * random * boff_factor * pow( 2.0, counter ); //modified 1st by tham

  backoffSumDuration(backoff_duration);

  if (alohaar_debug){
       cout << NOW << "  AlohaAr ("<< addr <<")::getBackoffTime() backoff time = " 
            << backoff_duration << " s" << " counter = " << counter<< " total backoff_times = " 
            << backoff_times_no<< endl; 
 }

 // printf (" %f UWAloha (%d)::getBackoffTime: total backoff_times"\n);// added by THAM 
  return backoff_duration;
}


double AlohaAr ::computeTxTime(UWALOHA_PKT_TYPE type)
{
  double duration;
  Packet* temp_data_pkt;
  map< pktSeqNum,Packet*> :: iterator it_p;

  if (type == UWALOHA_DATA_PKT) {
     if (!mapPacket.empty()) {
	it_p = mapPacket.begin();
        temp_data_pkt = ((*it_p).second)->copy();  
        //temp_data_pkt = (Q.front())->copy();  
        hdr_cmn *ch = HDR_CMN(temp_data_pkt);
        ch->size() = HDR_size + ch->size();
     }
     else { 
        temp_data_pkt = Packet::alloc();
        hdr_cmn *ch = HDR_CMN(temp_data_pkt);
        ch->size() = HDR_size + max_payload;
     }
  }
  else if (type == UWALOHA_ACK_PKT) {
        temp_data_pkt = Packet::alloc();  
        hdr_cmn *ch = HDR_CMN(temp_data_pkt);
        ch->size() = ACK_size;
  }
  duration = Mac2PhyTxDuration(temp_data_pkt );
  Packet::free(temp_data_pkt);
  //if (uwaloha_debug)  cout << "computed-txtime: " << duration << endl; //added by tham
 // printf("computeTxTime()\n");
  return(duration);
}


void AlohaAr ::recvFromUpperLayers(Packet* p)
{ 
   //printf("%f AlohaAr(%d) generate packet\n",NOW,addr); //Tham 
  if ( ((has_buffer_queue == true) && (mapPacket.size() < buffer_pkts)) || (has_buffer_queue == false) ) {
     initPkt(p , UWALOHA_DATA_PKT);
     //Q.push(p);
     putPktInQueue(p);
     incrUpperDataRx();
     waitStartTime();

     if ( curr_state == UWALOHA_STATE_IDLE ) 
       {
         refreshReason(UWALOHA_REASON_DATA_PENDING);
         stateTxData();

//          /* if (uwaloha_debug) 
// 	   cerr << showpoint << NOW << " " <<  __PRETTY_FUNCTION__ 
// 	       << " mac busy => enqueueing packet" << endl;
       }
     else
       {
//          /* if (uwaloha_debug) 
// 	   cerr << showpoint << NOW << " " <<  __PRETTY_FUNCTION__ 
// 	        << " transmitting packet" << endl;
//       
       }
  }
  else {
     incrDiscardedPktsTx();
     drop(p, 1, UWALOHA_DROP_REASON_BUFFER_FULL);
  }
} 

             
void AlohaAr ::initPkt( Packet* p, UWALOHA_PKT_TYPE type, int dest_addr ) 
{
  hdr_cmn* ch = hdr_cmn::access(p);
  hdr_mac* mach = HDR_MAC(p);

  int curr_size = ch->size();

  switch(type) {
  
    case(UWALOHA_DATA_PKT): {
      ch->size() = curr_size + HDR_size;
    } 
    break;

    case(UWALOHA_ACK_PKT): {
      ch->ptype() = PT_MMAC_ACK;
      ch->size() = ACK_size;
      mach->uwaloha_pkt_type() = 2; // tham
      ch->uid() = recv_data_id;
      mach->macSA() = addr;
      mach->macDA() = dest_addr;
    }
    break;

  }

}

void AlohaAr ::Mac2PhyStartTx(Packet* p) 
{
   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr <<")::Mac2PhyStartTx() start tx packet" << endl;
  MMac::Mac2PhyStartTx(p);
}


void AlohaAr ::Phy2MacEndTx(const Packet* p) 
{ 

  if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr <<")::Phy2MacEndTx() end tx packet" << endl;

  switch(curr_state) {

    case(UWALOHA_STATE_TX_DATA): {
      refreshReason(UWALOHA_REASON_DATA_TX);
      if (ack_mode == UWALOHA_ACK_MODE) {

         if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr <<")::Phy2MacEndTx() DATA sent,from "
                        << status_info[curr_state] << " to " 
                        << status_info[UWALOHA_STATE_WAIT_ACK] << endl;

        stateWaitAck(); 
      }
      else{

         if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr <<")::Phy2MacEndTx() DATA sent, from " 
                        << status_info[curr_state] << " to " << status_info[UWALOHA_STATE_IDLE] << endl;
    
        stateIdle();
      }
    }
    break;

    case(UWALOHA_STATE_TX_ACK): {
      refreshReason(UWALOHA_REASON_ACK_TX);

	if (alohaar_debug) cout << NOW  << "  AlohaAr ("<< addr <<")::Phy2MacEndTx() ack sent, from " 
                          << status_info[curr_state] << " to " << status_info[UWALOHA_STATE_IDLE] << endl;
	stateIdle();
    }
    break;
    default: {
        cout << NOW << "  AlohaAr ("<< addr <<")::Phy2MacEndTx() logical error, current state = " 
             << status_info[curr_state] << endl;
        stateIdle();
        //exit(1);
    }
    break;

  }

}


void AlohaAr ::Phy2MacStartRx(const Packet* p) 
{
   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr <<")::Phy2MacStartRx() rx Packet " << endl; 
   //check packet size - tham
   hdr_cmn* ch = HDR_CMN(p);
   int size = ch->size();
  // printf("size of packet at Phy: %d\n",size);
 
}


void AlohaAr ::Phy2MacEndRx(Packet* p) {
  hdr_cmn* ch = HDR_CMN(p);
  packet_t rx_pkt_type = ch->ptype();
  hdr_mac* mach = HDR_MAC(p);
  hdr_MPhy* ph = HDR_MPHY(p);

  int source_mac = mach->macSA();
  int dest_mac = mach->macDA();
  
  double gen_time = ph->txtime;
  double received_time = ph->rxtime;
  double time_trans = ph->duration; //tham
 // printf("duration: %f\n",time_trans); //tham
  double diff_time = received_time - gen_time;

  double distance = diff_time * prop_speed;

   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::Phy2MacEndRx() " 
                   << status_info[curr_state] << ", received a pkt type = " 
                   << mach->uwaloha_pkt_type() << ", src addr = " << mach->macSA()  //
                   << " dest addr = " << mach->macDA()                              //
                   << ", estimated distance between nodes = " << distance << " m " << endl; //tham

  if ( ch->error() ) {

     if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::Phy2MacEndRx() error pkt " << endl;
    incrErrorPktsRx();

    refreshReason(UWALOHA_REASON_PKT_ERROR);
    drop(p, 1, UWALOHA_DROP_REASON_ERROR);
    return;
  }
  else {
    if ( dest_mac == addr || dest_mac == MAC_BROADCAST ) {
    
      if ( rx_pkt_type == PT_MMAC_ACK ) {
  
          if (mach->uwaloha_pkt_type() == 2) {
             refreshReason(UWALOHA_REASON_ACK_RX);
             stateRxAck(p);

           }
             /* else if (mach->uwaloha_pkt_type() == 1 && (curr_state != UWALOHA_STATE_WAIT_ACK)) {
               refreshReason(UWALOHA_REASON_MAC_PKT_RX);
               stateRxMacPkt(p);               
              } */

      }  else if ( curr_state != UWALOHA_STATE_WAIT_ACK ) {
                refreshReason(UWALOHA_REASON_DATA_RX);
                stateRxData(p);
   
                }
               else { 
                  refreshReason(UWALOHA_REASON_PKT_NOT_FOR_ME);
	          Packet::free(p);
                 // printf("UWAloha ::Phy2MacEndRx(): Node %d Rx pkt not for me\n");
	          return;
                }
   }
   else {
      refreshReason(UWALOHA_REASON_PKT_NOT_FOR_ME);
      Packet::free(p);
    if (tham_debug)
     printf("UWAloha ::Phy2MacEndRx(): Node %d Rx Pkt not for me\n",addr);
      return;
    }
  }
}


void AlohaAr ::txData()
{ 
  Packet* data_pkt = curr_data_pkt->copy();    
 
  if ( (ack_mode == UWALOHA_NO_ACK_MODE) ) {
     eraseItemFromPktQueue(getPktSeqNum(data_pkt));
  }                                           
  
  total_retrans++; // for ARS
  incrDataPktsTx();
  incrCurrTxRounds();
  Mac2PhyStartTx(data_pkt); 
  num_trans_current_ar++;  // ARS
  total_packet_trans_ar++; // ARS: calculate total of packet is transmitted in MAC layer (trans and re-trans)

}


void AlohaAr ::txAck( int dest_addr )
{
  Packet* ack_pkt = Packet::alloc();
  initPkt( ack_pkt , UWALOHA_ACK_PKT, dest_addr );

  incrAckPktsTx();
  Mac2PhyStartTx(ack_pkt);

}


void AlohaAr ::stateCheckAckExpired() 
{
  refreshState(UWALOHA_STATE_CHK_ACK_TIMEOUT);

   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::stateCheckAckExpired()" << endl;
  
  map< pktSeqNum,AckTimer> :: iterator it_a;
  it_a = mapAckTimer.begin();
  
  
  if (print_transitions) printStateInfo();
  if (((*it_a).second).isActive() ) {
    refreshReason( UWALOHA_REASON_WAIT_ACK_PENDING );
    refreshState( UWALOHA_STATE_WAIT_ACK );
  }
  else if (((*it_a).second).isExpired() ) {
    refreshReason( UWALOHA_REASON_ACK_TIMEOUT );
    stateBackoff();
  }
  else {
    cerr << NOW << "  AlohaAr ("<< addr << ")::stateCheckAckExpired() ack_timer logical error, current timer state = " 
         << status_info[curr_state] << endl;
    stateIdle();
    //exit(1);  
  }
}


void AlohaAr ::stateCheckBackoffExpired() 
{
  refreshState(UWALOHA_STATE_CHK_BACKOFF_TIMEOUT);

  if (alohaar_debug) cout << NOW << "  AlohaAr ("<< addr << ")::stateCheckBackoffExpired()" << endl;
  if (print_transitions) printStateInfo();
  if ( backoff_timer.isActive() ) {
    refreshReason( UWALOHA_REASON_BACKOFF_PENDING );
    stateBackoff();
  }
  else if ( backoff_timer.isExpired() ) {
    refreshReason( UWALOHA_REASON_BACKOFF_TIMEOUT );
    exitBackoff();
    stateIdle();
  }
  else {
    cerr << NOW << "  UWAloha ("<< addr << ")::stateCheckBackoffExpired() backoff_timer logical error, current timer state = " 
         << status_info[curr_state] << endl;
    stateIdle();
    //exit(1);  
  }
}
  

void AlohaAr ::stateIdle() {
  mapAckTimer.clear();
  backoff_timer.stop();
 
  if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::stateIdle() queue size = " << mapPacket.size() << endl;
    
  refreshState(UWALOHA_STATE_IDLE);

  if (print_transitions) printStateInfo();

  if ( !mapPacket.empty() ) {
      stateTxData();
     // stateSendMacpkt(); 
  
  } 
}


void AlohaAr ::stateRxIdle() {
  refreshState(UWALOHA_STATE_RX_IDLE);

  if (print_transitions) printStateInfo();
}


void AlohaAr ::stateBackoff() {
  backoff_timer.force_cancel();
  refreshState(UWALOHA_STATE_BACKOFF);
  if (alohaar_debug) cout << NOW << "  AlohaAr ("<< addr << ")::stateBackoff() " << endl;
  if ( backoff_timer.isFrozen() ) backoff_timer.unFreeze();
  else backoff_timer.schedule( getBackoffTime() );

  if (print_transitions) printStateInfo(backoff_timer.getDuration());
}


void AlohaAr ::stateRxBackoff() {
  backoff_timer.freeze();
  refreshState(UWALOHA_STATE_RX_BACKOFF);

  if (print_transitions) printStateInfo();
}


void AlohaAr ::stateTxData()
{
  refreshState(UWALOHA_STATE_TX_DATA);
 
   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::stateTxData() at :";
  if (print_transitions) printStateInfo();

  map< pktSeqNum,Packet*> :: iterator it_p;
  it_p = mapPacket.begin();
  curr_data_pkt = (*it_p).second;
  int seq_num;
  seq_num = getPktSeqNum(curr_data_pkt);
  map< pktSeqNum,AckTimer> :: iterator it_a;

  if ( seq_num != last_sent_data_id) {
     putAckTimerInMap(seq_num);
     it_a = mapAckTimer.find(seq_num);
     resetCurrTxRounds();
     backoff_timer.resetCounter();
     ((*it_a).second).resetCounter();
     hdr_mac* mach = HDR_MAC(curr_data_pkt);
     start_tx_time = NOW; // we set curr RTT
     last_sent_data_id = seq_num;
     original_current_ar++; // in time window
     total_orig++; //Tham
     txData();  
  }
  
  else { 
    
    if (mapAckTimer.size() == 0) {
      putAckTimerInMap(seq_num);
      it_a = mapAckTimer.find(seq_num);
      ((*it_a).second).resetCounter();
      //incrCurrTxRounds(); //modified by THAM
      backoff_timer.incrCounter();
      if ( curr_tx_rounds < max_tx_tries ) { 
	hdr_mac* mach = HDR_MAC(curr_data_pkt);
        mach->macSA() = addr;
	start_tx_time = NOW; // we set curr RTT
	last_sent_data_id = seq_num;
	txData();
        re_trans_no = re_trans_no++;//added by Tham

      //  mach->node_background_traffic_ar() = node_background_traffic_ar; // traffic include retransmission
      //  mach->node_original_traffic_ar() = node_original_traffic_ar; // original traffic

      }
      
      else {
       
	eraseItemFromPktQueue(seq_num);
	incrDroppedPktsTx();

	refreshReason(UWALOHA_REASON_MAX_TX_TRIES);

       if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::statePreTxData() curr_tx_rounds " << curr_tx_rounds
                     << " > max_tx_tries = " << max_tx_tries << endl;
	stateIdle();
           }
      }
    
   else {
      stateCheckAckExpired();
    }
  }
 }



void AlohaAr ::stateWaitAck() {
  
  map< pktSeqNum,AckTimer> :: iterator it_a;
  it_a = mapAckTimer.begin();
  
  ((*it_a).second).stop();
  refreshState(UWALOHA_STATE_WAIT_ACK);

   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::stateWaitAck() " << endl;
   if (print_transitions) printStateInfo();
  
  ((*it_a).second).incrCounter();
  ((*it_a).second).schedule(ACK_timeout + 2 * wait_constant); 
   if (alohaar_debug)  cout << "ack-timeout: " << ACK_timeout + 2* wait_constant << endl;//added by tham
}


void AlohaAr ::stateRxWaitAck() {
  refreshState(UWALOHA_STATE_RX_WAIT_ACK);

  if (print_transitions) printStateInfo();
}


void AlohaAr ::stateTxAck( int dest_addr ) {
  refreshState(UWALOHA_STATE_TX_ACK);

   if (alohaar_debug)  cout << NOW << "  AlohaAr ("<< addr << ")::stateTxAck() dest addr " << dest_addr << endl;
   if (print_transitions) printStateInfo();
 
  txAck( dest_addr );
}


void AlohaAr ::stateRxData(Packet* data_pkt) {
  ack_timer.stop();
  refreshState( UWALOHA_STATE_DATA_RX );
  
   if (alohaar_debug) cout << NOW << "  AlohaAr ("<< addr << ")::stateRxData() at: " << NOW << endl;//modified by tham
  refreshReason( UWALOHA_REASON_DATA_RX );

  hdr_cmn* ch = hdr_cmn::access(data_pkt);
  hdr_mac* mach = HDR_MAC(data_pkt);
  int dst_addr = mach->macSA();

  //-------- Added by Tham (2013, Aug) ----------------------------------------------------

  recv_frame++;
  size_ar = ch->size(); // Tham Nguyen 2013
  if (alohaar_debug) printf(" Packet size at MAC %d\n",ch->size());
  getTransTime_Ar(size_ar); //Tham Nguyen 2013
      
   /* double background_traffic = mach->node_background_traffic_ar(); //tham
    double original_traffic = mach->node_original_traffic_ar(); //tham
  
   for (int i=0,n=99; i < number_of_flow_ar; i++) {
      if ((dst_addr == i) && (background_traffic != 0) && (original_traffic != 0) ) {
         Node_Array[i] = dst_addr;
         BackgroundTraffic_Array[i] = background_traffic;
         OriginalTraffic_Array[i] = original_traffic;
       }
    } */ 

 // Count the packet recv at sink (do not take acount duplicate pkts)
   int packet_id = ch->uid();
   int i = dst_addr;
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
//-------------------------------------------------------------------------------------------
 
  
  recv_data_id = ch->uid();
  ch->size() = ch->size() - HDR_size;
  incrDataPktsRx();
  sendUp(data_pkt); 
      
  if (ack_mode == UWALOHA_ACK_MODE) stateTxAck(dst_addr);
  else stateIdle();

}


void AlohaAr ::stateRxAck(Packet* p) {
  
  map< pktSeqNum,AckTimer> :: iterator it_a;
  it_a = mapAckTimer.begin();
  
  ((*it_a).second).stop();
  refreshState(UWALOHA_STATE_ACK_RX);
   if (alohaar_debug)  cout << NOW << " AlohaAr ("<< addr << ")::stateRxAck() " << endl; //modified by tham
  
  int seq_num;
  seq_num = getPktSeqNum(p);

  Packet::free(p);

  refreshReason(UWALOHA_REASON_ACK_RX);

  eraseItemFromPktQueue(seq_num);
  eraseItemFrommapAckTimer(seq_num);
 // updateAckTimeout(NOW - start_tx_time); //modified by THAM
  incrAckPktsRx();
  stateIdle(); 

 /*switch( prev_state ) {
    
    case UWALOHA_STATE_RX_IDLE: {
      
      stateIdle();
    }
    break;
      
    case UWALOHA_STATE_RX_BACKOFF: {

      printf("%f Node(%d) go to state check backoff expire\n",NOW,addr);
      
      stateCheckBackoffExpired();
    }
    break;     
    default: 

      cerr << NOW << " UWAloha("<< addr << ")::stateRx-ack-pkt() logical error, prev state = " << status_info[prev_state]
           << endl;
  }
  */
       
  
}

void AlohaAr ::printStateInfo(double delay)
{
   if (alohaar_debug)  cout << NOW << " AlohaAr ("<< addr << ")::printStateInfo() " << "from " << status_info[prev_state] 
                   << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] << endl;
  
  if (curr_state == UWALOHA_STATE_BACKOFF) {
      fout <<left << setw(10) << NOW << "  AlohaAr ("<< addr << ")::printStateInfo() " 
           << "from " << status_info[prev_state] 
           << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] 
           << ". Backoff duration = " << delay << endl;
  }
  else {	   
  fout << left << setw(10) << NOW << "  AlohaAr ("<< addr << ")::printStateInfo() " 
           << "from " << status_info[prev_state] 
           << " to " << status_info[curr_state] << ". Reason: " << reason_info[last_reason] << endl;
  }
}
void AlohaAr ::waitForUser()
{
  std::string response;
  std::cout << "Press Enter to continue";
  std::getline(std::cin, response);
} 


