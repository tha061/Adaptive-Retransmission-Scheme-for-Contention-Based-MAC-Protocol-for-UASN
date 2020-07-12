/* -*- Mode:C++ -*- */

/*
 * Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
 * All rights reserved.
 */

 
/**
 * @file   uw-aloha-ar.h
 * @author Tham Nguyen
 * @version 1.0.0
 */

#ifndef UWALOHA_H_
#define UWALOHA_H_

#include <mmac.h>
#include <iostream>
#include <string>
#include <map>
#include <set>
#include <queue>
#include <fstream>

#include <mphy.h>

#define UWALOHA_DROP_REASON_WRONG_STATE "WST"
#define UWALOHA_DROP_REASON_WRONG_RECEIVER "WRCV"
#define UWALOHA_DROP_REASON_UNKNOWN_TYPE "UPT"
#define UWALOHA_DROP_REASON_BUFFER_FULL "DBF"
#define UWALOHA_DROP_REASON_ERROR "ERR"

extern packet_t PT_MMAC_ACK;

typedef int pktSeqNum;

/**
*@brief This is the base class of UWAloha protocol, which is a derived class of MMac.
*/

class AlohaAr : public MMac {
  
  public:

  /**
  *Constructor of UWAloha Class
  */
  AlohaAr();
  
  /**
  *Destructor of UWAloha Class
  */
  virtual ~AlohaAr();

  
  /**
  * TCL command interpreter. It implements the following OTcl methods:
  * @param argc number of arguments in <i>argv</i>
  * @param argv array of strings which are the command parameters (Note that argv[0] is the name of the object)
  * @return TCL_OK or TCL_ERROR whether the command has been dispatched succesfully or not
  */
  virtual int command(int argc, const char*const* argv);

  
  protected:
  
  /**
  *Enumeration class of UWAloha status. First enumerator is given value 1.
  */
  enum UWALOHA_STATUS {
    UWALOHA_STATE_IDLE = 1, UWALOHA_STATE_BACKOFF, UWALOHA_STATE_TX_DATA, UWALOHA_STATE_TX_ACK, UWALOHA_STATE_WAIT_ACK, 
    UWALOHA_STATE_DATA_RX, UWALOHA_STATE_ACK_RX, UWALOHA_STATE_NOT_SET, UWALOHA_STATE_CHK_ACK_TIMEOUT, UWALOHA_STATE_RX_IDLE,
    UWALOHA_STATE_RX_WAIT_ACK, UWALOHA_STATE_CHK_BACKOFF_TIMEOUT, UWALOHA_STATE_RX_BACKOFF, UWALOHA_STATE_WRONG_PKT_RX, 
    UWALOHA_STATE_TX_MAC_PKT, UWALOHA_STATE_MAC_RX
  };

  /**
  *Enumeration class which tells the nodes the reason why it is in this state. First enumerator is given value 1.
  */
  enum UWALOHA_REASON_STATUS {
    UWALOHA_REASON_DATA_PENDING, UWALOHA_REASON_DATA_RX, UWALOHA_REASON_DATA_TX, UWALOHA_REASON_ACK_TX, 
    UWALOHA_REASON_ACK_RX, UWALOHA_REASON_ACK_TIMEOUT, UWALOHA_REASON_DATA_EMPTY, UWALOHA_REASON_NOT_SET,
    UWALOHA_REASON_MAX_TX_TRIES, 
    UWALOHA_REASON_START_RX, UWALOHA_REASON_PKT_NOT_FOR_ME, UWALOHA_REASON_WAIT_ACK_PENDING,  
    UWALOHA_REASON_PKT_ERROR, UWALOHA_REASON_BACKOFF_TIMEOUT,UWALOHA_REASON_BACKOFF_PENDING, UWALOHA_REASON_MAC_PENDING,
    UWALOHA_REASON_MAC_TX, UWALOHA_REASON_MAC_PKT_RX
  };
  
  /**
  *Enumeration class of UWAloha packet type. First enumerator is given value 1. Three kinds of packets are supported by UWAloha protocol.
  */
  enum UWALOHA_PKT_TYPE {
    UWALOHA_ACK_PKT = 1, UWALOHA_DATA_PKT, UWALOHA_DATAMAX_PKT, UWALOHA_MAC_PKT
  };
  
  /**
  *Enumeration class of UWAloha acknowledgement mode. First enumerator is given value 1. This protocol supports both acknowledgement and 
  * non-acknowledgement technique. If Acknowledgement is set, it uses Stop-And-Wait ARQ technique.
  */
  enum UWALOHA_ACK_MODES {
    UWALOHA_ACK_MODE = 1, UWALOHA_NO_ACK_MODE
  };

  /**
  *Enumeration class of UWAloha timer status. First enumerator is given value 1. It is employed to know the current status of a timer.
  */
  enum UWALOHA_TIMER_STATUS {
    UWALOHA_IDLE = 1, UWALOHA_RUNNING, UWALOHA_FROZEN, UWALOHA_EXPIRED
  };

  /**
  * Base class of all the timer used in this protocol. This is a derived class of TimerHandler.
  */
  class UWAlohaTimer  : public TimerHandler {
    
    
    public:

    /**
    * Constructor of UWAlohaTimer Class.
    */
 UWAlohaTimer (AlohaAr *m) : TimerHandler(), start_time(0.0), left_duration(0.0), counter(0), module(m), timer_status(UWALOHA_IDLE) 
     { 
      assert(m != NULL); }
      
    /**
    * Destructor of UWAlohaTimer Class.
    */
    virtual ~UWAlohaTimer () { }
    
    /**
    * It freezes or in another word, it stops the timer for some time. Suppose, for some reason we want to stop 
    * a timer for some period and we want to run this timer from where it was stopped. This function stops the timer and 
    * save the left time duration it must run.
    */
/*
    virtual void freeze() { assert(timer_status == UWALOHA_RUNNING); left_duration = NOW - start_time; 
                            if (left_duration <= 0.0) left_duration = module->mac2phy_delay_; force_cancel();
                            timer_status = UWALOHA_FROZEN; }
*/
   virtual void freeze() { assert(timer_status == UWALOHA_RUNNING); 
                            left_duration -= (NOW - start_time); // Corrected and confirmed to DESERT authors (2013/05)
                            if (left_duration <= 0.0) left_duration = module->mac2phy_delay_; 
                            force_cancel();
                            timer_status = UWALOHA_FROZEN; }
    /**
    * It starts the timer from where it was stopped. To run any freeze timer, we can use unfreeze method.   
    */
    virtual void unFreeze() { assert(timer_status == UWALOHA_FROZEN); 
                              start_time = NOW; 
                              assert(left_duration > 0);
                              sched(left_duration); 
                              timer_status = UWALOHA_RUNNING; }
   
    /**
    * Stop the timer any way.
    */
    virtual void stop() { timer_status = UWALOHA_IDLE; force_cancel(); }
     
    /**
    * Schedule the time, i.e., how long a timer is going to run. 
    * @param double time
    */
    virtual void schedule( double val ) { start_time = NOW; left_duration = val; timer_status = UWALOHA_RUNNING; resched(val); }
   
    /**
    * It tells whether the timer is in Idle state or not.
    * @return 1 if the timer is idle and 0 if it is not.
    */
    bool isIdle() { return ( timer_status == UWALOHA_IDLE ); }
    
    /**
    * This method tells whether the timer is in Running state or not.
    * @return 1 if the timer is running and 0 if it is not.
    */
    bool isRunning() { return (timer_status == UWALOHA_RUNNING); }
    
    /**
    * Tells whether the timer is expired or not.
    * @return 1 if the timer expired and 0 if it is not.
    */
    bool isExpired() { return (timer_status == UWALOHA_EXPIRED); }
    
    /**
    * It tells whether the timer is in freeze mode or not.
    * @return 1 if the timer is in freeze mode and 0 if it is not.
    */
    bool isFrozen() { return (timer_status == UWALOHA_FROZEN); }
       
    /**
    * It tells whether the timer is active or not.
    * @return 1 if the timer is active and 0 if it is not.
    */
    bool isActive() { return (timer_status == UWALOHA_FROZEN || timer_status == UWALOHA_RUNNING ); }
    
    /**
    * Reset the timer counter.
    */
    void resetCounter() { counter = 0; }
    
    /**
    * Increment the timer counter. It helps to know the statics of the timer.
    */
    void incrCounter() { ++counter; }
    
    /**
    * It provides, how many times a timer ran.
    * @return number of times a timer ran (int).
    */
    int getCounter() { return counter; }
    
    /**
    * This methods provide the duration of a timer.
    * @return left time duration of a timer (double).
    */
    double getDuration() { return left_duration; }
    
    
    protected:

    double start_time;     /**< Start time of a timer. */
    
    double left_duration;     /**< How long a timer is going to run more. */
       
    int counter;     /**< How many times a timer ran. */
    
    AlohaAr* module;     /**< Pointer of UWAloha module. */
    
    UWALOHA_TIMER_STATUS timer_status;       /**< Set the status of the timer. */
    
  };
  
  /**
  * Base class of AckTimer, which is a derived class of UWAlohaTimer.
  */
  class AckTimer : public UWAlohaTimer  { 
  
    
    public:
    
    /**
    * Constructor of AckTimer Class.
    */
    AckTimer(AlohaAr* m) : UWAlohaTimer (m) { }
    
    /**
    * Destructor of AckTimer Class.
    */
    virtual ~AckTimer() { }
    
      
    protected:

    /**
    * What a node is going to do when a timer expire.
    * @param Event
    */
    virtual void expire(Event *e);
    

  };
  
  /**
  * Base class of BackoffTimer. It is derived class of UWAlohaTimer.
  */
  class BackOffTimer : public UWAlohaTimer  { 
  
    
    public:
    
    /**
    * Constructor of BackOffTimer Class.
    */
    BackOffTimer(AlohaAr* m) : UWAlohaTimer (m) { }
    
    /**
    * Destructor of BackOffTimer.
    */
    virtual ~BackOffTimer() { }
    
      
    protected:

    /**
    * What a node is going to do when a timer expire.
    * @param Event
    */      
    virtual void expire(Event *e);
    

  };

//------------------------------------------------------------------------------------------------
      
           /*   Class used to handle the Window Timer. The duration of the timer is given by time window value (default = 20 s)
           Added by Tham Nguyen UOU Korea 2012
           */
    class WindowTimer_Ar : public UWAlohaTimer { 
  
    public:
      
	//
	//  Conscructor of WindowTimer class 
	//  @param CsmaAr* pointer to an object of type CsmaAloha
	
    WindowTimer_Ar(AlohaAr* m) : UWAlohaTimer(m) { }
    
	// Destructor of AckTimer class 
	
    virtual ~WindowTimer_Ar() { }
    
      
    protected:

	 //* Method called when the timer expire
	 //* @param Eevent*  pointer to an object of type Event 
	
    virtual void expire(Event *e);
    
    
  };

//Sink calculate parameter whenever SinkTimer expire

 class SinkTimer : public UWAlohaTimer { 
  
    public:
      
	//*
	 //* Conscructor of WindowTimer class 
	 //* @param CsmaAr* pointer to an object of type CsmaAloha
	
    SinkTimer(AlohaAr* m) : UWAlohaTimer(m) { }
    
	//**
	 //* Destructor of AckTimer class : // corrected by Tham 2013/04
	
    virtual ~SinkTimer() { }
    
      
    protected:

	//*
	// * Method called when the timer expire
	// * @param Eevent*  pointer to an object of type Event 
	
    virtual void expire(Event *e);
    
    
  };

/////
// Get packet size take acount into headers, Tham Ng 2012
   virtual int getTransTime_Ar(int size_ar);

// Get background traffic (include retransmission) and original traffic of each node
   // virtual double getTrafficWithinTimeWd_Ar();

// Sink process
   virtual void sinkProcess();

// create the array store the node addr when sink receive Data packet, Tham Ng 2012

    //virtual void initArrayOfNode_Ar();

// Create the array store the number of transmission of each node in array of node addr, Tham Ng 2012

   //virtual void initArrayBackgroundTrafficOfEachNode_Ar();
  // virtual void initArrayOriginalTrafficOfEachNode_Ar();

// Create the array store the max_trans of each node
   //virtual void initArrayMaxTransOfNode_Ar();

//Create the matrix to store the packet-id recve at sink
  virtual void initMatrixPacketRecv_Ar();

//-------------------------------------------------------------------------------------------------------------
  /**
  * This function receives the packet from upper layer and save it in the queue.
  * @param Packet pointer
  */
  virtual void recvFromUpperLayers(Packet* p);

  /**
  * It informs that a packet transmission started.
  * @param Packet pointer
  */
  virtual void Mac2PhyStartTx(Packet* p);
  
  /**
  * It infroms that a packet transmission end.
  * @param Packet pointer
  */
  virtual void Phy2MacEndTx(const Packet* p);
  
  /**
  * PHY layer informs the MAC layer that it is receiving a packet.
  * @Param Packet pointer (constant)
  */
  virtual void Phy2MacStartRx(const Packet* p);
  
  /**
  * PHY layer informs the MAC layer that the reception of the packet is over.
  * @param Packet pointer.
  */
  virtual void Phy2MacEndRx(Packet* p);

  /**
  * Compute the transmission time of a packet. It uses a cross-layer message to calculate the duration of that packet. 
  * @param type is a UWALOHA_PKT_TYPE
  * @return tranmission time of a packet which is a double data type.
  */
  virtual double computeTxTime(UWALOHA_PKT_TYPE type);
  
  /**
  * This method, initialize the packet. If the packet is received from the upper layer, it adds the header (if any). In case of UWAloha with ARQ 
  * technique, it set the fields of ACK packet.
  * @param Packet pointer P. The packet can be <i>Data</i> packet or <i>ACK</i> packet.
  * @param pkt_type is an UWALOHA_PKT_TYPE. Packet can be either <i>Data</i> packet or <i>ACK</i> packet.
  * @param dest_addr is a integer data type. It is initialized as 0.
  */
  virtual void initPkt( Packet* p, UWALOHA_PKT_TYPE pkt_type, int dest_addr = 0);
  
  /**
  * This function calculates the backoff duration and return the backoff time.It employs the exponential backoff algorithm.
  * @return backoff duration which is a double data type.
  */
  virtual double getBackoffTime();
  
  /**
  * This method transmits <i>Data</i> packets from MAC layer to PHY layer.
  */
  virtual void txData();
  

  /**
  * This methods transmits <i>ACK</i> packet from MAC layer to PHY layer.
  * @param dest_addr which is an integer data type.
  */
  virtual void txAck(int dest_addr);
  
  /**
  * Node is in Idle state. It only changes its state if it has packet(s) to transmit or it receives a packet.
  */
  virtual void stateIdle();
  
  /**
  * If a node start receiving a packet in Idle state.
  */
  virtual void stateRxIdle();
  
  /**
  * If a node has packet to transmits. In such case, it moves from Idle state to data transmits state.
  */
  virtual void stateTxData();
  
  /**
  * If the protocl uses ARQ technique, in that case, after receiving a <i>Data</i> packet the node sends an <i>ACK</i> packet.
  */
  virtual void stateTxAck(int dest_addr);
  
  /**
  * After transmitting a <i>Data</i> packet, a node waits for the <i>ACK</i> packet.
  */
  virtual void stateWaitAck();
  
  /**
  * If a node receives any packet while it was waiting for <i>ACK</i> packet, it moves to this state. The packet it is receiving can be a <i>Data</i> packet
  * from another node or <i>ACK</i> packet.
  */
  virtual void stateRxWaitAck();
  
  /**
  * If <i>ACK</i> packet is not received within the acknowledgement expire time.
  */
  virtual void stateBackoff();
  
  /**
  * If a node start receiving a packet when it is in backoff state. The node first freeze (or another word, hold) the backoff timer and start receiving 
  * the packet. 
  */
  virtual void stateRxBackoff();
  
  /**
  * It checks whether the ack timer is already expired while it was busy with other activities.
  */
  virtual void stateCheckAckExpired();
  
  /**
  * It checks whether the backoff timer is already expired while it was busy with other activities.
  */
  virtual void stateCheckBackoffExpired();
  
  /**
  * It process the packet which is received. After receiving a packet it changes it states according to the previously stored status information.
  * @param <i>Data</i> packet pointer
  */
  virtual void stateRxData(Packet* p);
  
  /**
  * The node comes to this state if it receives an <i>ACK</i> packet. After receiving an <i>ACK</i> packet it changes it states according 
  * to the previously stored status information.
  */
  virtual void stateRxAck(Packet* p);
  
  /**
  * It stops the backoff timer.
  */
  virtual void exitBackoff();

  /**
  * This methods print the state information of the nodes.
  * @param delay is a double data type.
  */
  virtual void printStateInfo(double delay = 0);
  
  /**
  * This function is used to initialize the UWAloha protocol.
  */
  virtual void initInfo();
  
  /**
  * Refreshes the states of the node. The node save the information of three states, they are: previous to previous state, previous state and 
  * current state of the node.
  * @param state which is an UWALOHA_STATUS type.
  */
  virtual void refreshState(UWALOHA_STATUS state) { prev_prev_state = prev_state; prev_state = curr_state; curr_state = state; }
  
  /**
  * To know the reason why a node is in this current state. 
  * @param reason is an UWALOHA_REASON_STATUS type.
  */
  virtual void refreshReason(UWALOHA_REASON_STATUS reason) { last_reason = reason; }
  
  /**
  * Increments the current transmission round of a packet. It keeps track of the number of retransmition of a packet.
  */
  virtual void incrCurrTxRounds() { curr_tx_rounds++; }
  
  /**
  * If a node is going to transmit a new packet, it resets the tx counter.
  */
  virtual void resetCurrTxRounds() { curr_tx_rounds = 0; }
  
  /**
  * Update the Round Trip Time (RTT) which is necessary to compute the acknowledgement duration as well as backoff duration.
  * @param rtt is a double data type.
  */
  //virtual void updateRTT(double rtt);
  
  /**
  * This method is used to get the average RTT over all the receives RTT.
  * @return average RTT time which is a double data type.
  */
 // virtual double getRTT() { return (rttsamples>0) ? sumrtt/rttsamples : 0 ; }
  
  /**
  * Like updateRTT() function.
  */
 // virtual void updateAckTimeout(double rtt);
  
  /**
  * It updates the sequence number of the last data packet rx.
  * @param id is an integer data type.
  */
  virtual void updateLastDataIdRx(int id) { last_data_id_rx = id; }
  virtual void waitForUser();

  /**
  * This method is used to get the sequence number from a packet. 
  * @param packet pointer
  * @return it returns sequence number which is an integer data type.
  */
  inline int getPktSeqNum(Packet* p) { int seq_num; hdr_cmn* ch = hdr_cmn::access(p); seq_num = ch->uid(); return seq_num; }
  
  /**
  * A node receives packet(s) from upper layer and store them in the container.
  * @param packet pointer
  */
  inline void putPktInQueue(Packet *p) { mapPacket.insert(pair<pktSeqNum,Packet*> (getPktSeqNum(p),p)); }
  
  /**
  * It erases the packet from the container.
  * @param seq_num which is an integer data type.
  */
  inline void eraseItemFromPktQueue(int seq_num){ map< pktSeqNum,Packet*> :: iterator it_p; it_p = mapPacket.find(seq_num); mapPacket.erase((*it_p).first); }
  
  /**
  * Put acknowledgement timer in the container.
  * @param seq_num which is an integer data type.
  */
  inline void putAckTimerInMap(int seq_num) { mapAckTimer.insert(pair<pktSeqNum,AckTimer> (seq_num,ack_timer)); }
  
  /**
  * Erase an item from acknowledgement stored container.
  * @param seq_num which is an integer data type.
  */
  inline void eraseItemFrommapAckTimer(int seq_num) { map< pktSeqNum,AckTimer> :: iterator it_a; it_a = mapAckTimer.find(seq_num); mapAckTimer.erase((*it_a).first); }
  
  /**
  * Number of packets which MAC layer receives form upper layer(s) but were not transmitted.
  * @return an integer value. 
  */
  virtual int getRemainingPkts() { return(up_data_pkts_rx - mapPacket.size()); }
  
  /**
  * Increment the number of <i>Data</i> packet receive for the upper layer.
  */
  virtual void incrUpperDataRx() {up_data_pkts_rx++;}
 
  // increment the number of mac pkt (added by Tham)
 // virtual void incrMacPktSend() {mac_pkts_send++;}

//--------------Input added by tham----------------------
 
  static int total_orig; //tham
  static int recv_frame; //tham
  static int total_retrans;
  static int ack_rcv; // Number of ACK packet received at nodes
  static double time_window_ar;
  static int max_tx_tries;
  static int recv_first_frame;
  static int drop_pkt;
  double sink_time_window;
  double ps_ack;
  double rc_sim;
  double Ps; //tham
  int ack_sent;
  int number_of_node_ar;
  int number_of_flow_ar;
  int packet_length;
  int re_trans_no; //number of re-trans - added by Tham
  double backoff_duration;
  double packet_period_ar;
  int    num_trans_current_ar;//number of transmissions current for CSMA_AR, Tham Nguyen UOU Korea 2012 
  int    num_tran_within_time_window_ar; //number of transmission within time_window, Tham Nguyen UOU Korea 2012 
  int    original_packet_within_time_wd_ar;
  int    sum_of_traffic_ar;
  double node_background_traffic_ar;
  double node_original_traffic_ar;
  double delta_d_ar;
  double epsilon1_ar;
  double epsilon2_ar;
  double Px_curr;
  int    max_tx_tries_curr;
  double print_sink;
  double print_px;
  double print_process;
  int    max_starting;
  double p_rc_x;
  double p_rc_x_dec;
  double p_rc_x_inc;
  double val;
  double val1;
  int    x_inc;
  double rc_inc;
  double Rs;
  double rc;
  double Rm;
  double Ot_curr;
  double Ot_m;
  double lambda_t_inc;
  double lambda_i_inc;
  double lambda_m_inc;
  double p_s_inc;
  double p_f_inc;
  int    x_dec;
  double rc_dec;
  double lambda_t_dec;
  double lambda_i_dec;
  double lambda_m_dec;
  double p_s_dec;
  double p_f_dec;
  int    dst_addr;
  int    x;
  double e;
  int    Node_Array[100];
  int    NumTrans_Array[100];
  double BackgroundTraffic_Array[100];
  double OriginalTraffic_Array[100];
  int    MaxTrans_Array[100];
  int    matrix_t[100][500]; // For count the packet recv at sink (inogoze duplicate)
  double time_to_check_bf;  //check buffer
  double boff_factor;       // back-off factor
  double beta;
  double alpha_t;
  double pdr_curr;
  double total_background_traffic_ar;
  double total_original_traffic_ar;
  double average_num_trans_ar;
  int    x_upper;
  double p_req_ar;
  double bit_data_rate_ar;
  int    size_ar;
  static double time_tr_ar;
  int    packet_size_ar;
  int    object_ar;
  double delta_ar;
  int    total_packet_trans_ar;
  int    total_packet_recev_ar;
  int    original_current_ar;
  //////////////////////////////
  double T;
  double r_t;
  double curr_t;
  double interval;
 // int mac_packet_size;
  double bit_rate;
 // int spec_node_id;
  int mac_pkts_send;
  int mac_pkt_recv;
  double time_expire;
  double random_ar;
  double transmission_time;
  int last_recv_pkt_id;
  int total_packet_trans; // Tham
  int total_mac_packet_trans;  // Tham
  int total_packet_recv;   // Tham
  int node_sent_to_medium; // Tham
  double tham_debug;      // Debuging flag
  double stop_time;
  int mac_pkt_recv_of_spec_node;
  int n1;
  int n2;
  int n3;
  int n4;
  int n5;
  int n6;
  int n7;
  int n8;
  int n9;
  int n10; 
  /////////////////////////////////

 // WindowTimer_Ar window_timer_ar;      //**< Object that represents the window timer for 
                                          //  counting number of transmission in time window
  SinkTimer sink_timer;               //Timer for sink process added by Tham Nguyen, UOU Korea 2012

//------------------------------------

// Input of original code
//  int max_tx_tries;                 /**< Maximum number of retransmissions attempt. */
  double wait_constant;            /**< This fixed time is used to componsate different time variations. */
  double backoff_tuner;           /**< Tunes the backoff duration. */
  int max_payload;                /**< Maximum number of payload in a packet. */
  int HDR_size;                   /**< Size of the HDR if any */
  int ACK_size;                  /**< Size of the ACK, if the node uses ARQ technique */
  double ACK_timeout;           /**< ACK timeout for the initial packet */
  int buffer_pkts;              /**< Number of packets a node can store in the container */
  double alpha_;               /**< This variable is used to tune the RTT */ 
  int max_backoff_counter;     /**< Maximum number of backoff it will consider while it increases the backoff exponentially */
  int alohaar_debug;           /**< Debuging Flag */
 
  static bool initialized;    /**< It checks whether UWAloha protocol is initialized or not. 
                                If <i>FALSE</i> means, not initialized and if <i>TRUE</i> * means it is initialized */

  int last_sent_data_id;      /**< sequence number of the last sent packet */

  bool print_transitions;    /**< Whether to print the state of the nodes */
  bool has_buffer_queue;     /**< Whether the node has buffer to store data or not */

  double start_tx_time;     /**< Time when a packet start transmitting */       
  double srtt;              /**< Smoothed Round Trip Time, calculated as for TCP */
  double sumrtt;            /**< Sum of RTT samples */
  double sumrtt2;           /**< Sum of (RTT^2) */
  int rttsamples;           /**< Number of RTT samples */  
  
  int curr_tx_rounds;       /**< How many times a packet is transmitted */
  int last_data_id_rx;      /**< The sequence number of last received packet */
  int recv_data_id;         /**< The sequence number of the packet which is received */

  Packet* curr_data_pkt;    /**< Pointer of the latest selected data packet. */

  int txsn;                         /**< Sequence number of the packet which is transmitted */
  static const double prop_speed; /**< Speed of the sound signal*/

  AckTimer ack_timer;              /**< An object of the AckTimer class */
  BackOffTimer backoff_timer;      /**< An object of the backoff timer class */
  
  UWALOHA_REASON_STATUS last_reason; /**< Enum variable which stores the last reason why a node changes its state */
  UWALOHA_STATUS curr_state;         /**< Enum variable. It stores the current state of a node */
  UWALOHA_STATUS prev_state;        /**< Enum variable. It stores the previous state of a node */
  UWALOHA_STATUS prev_prev_state;   /**< Enum variable. It stores the previous to previous state of a node */
  
  UWALOHA_ACK_MODES ack_mode;       /**< Enum variable. It tells the node whether to use ARQ technique or not. */
  static map< UWALOHA_STATUS , string > status_info;      /**< Container which stores all the status information */
  static map< UWALOHA_REASON_STATUS, string> reason_info; /**< Container which stores all the reason information */
  static map< UWALOHA_PKT_TYPE, string> pkt_type_info;    /**< Container which stores all the packet type information of UWAloha*/
  
  map< pktSeqNum,Packet*> mapPacket;     /**< Container where <i>Data</i> packets are stored */
  map< pktSeqNum,AckTimer> mapAckTimer; /**< Container where acknowledgement timer(s) is stored */

  ofstream fout; /**< An object of ofstream class */
};

#endif /* UWUWALOHA_H_ */
