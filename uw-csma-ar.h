 //
 // Copyright (c) 2012 Regents of the SIGNET lab, University of Padova.
 // All rights reserved.
 //
 // Redistribution and use in source and binary forms, with or without
 // modification, are permitted provided that the following conditions
 // are met:
 // 1. Redistributions of source code must retain the above copyright
 //    notice, this list of conditions and the following disclaimer.
 // 2. Redistributions in binary form must reproduce the above copyright
 //    notice, this list of conditions and the following disclaimer in the
 //    documentation and/or other materials provided with the distribution.
 // 3. Neither the name of the University of Padova (SIGNET lab) nor the 
 //    names of its contributors may be used to endorse or promote products 
 //    derived from this software without specific prior written permission.
 //
 // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 // "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 // TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 // PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
 // CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 // EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 // PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 // OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 // WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 // OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
 // ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 //

 
/**
 * @file   uw-csma-aloha.h
 * @author Federico Guerra, Saiful Azad and Federico Favaro
 * @version 1.0.0
 * 
 * \brief Provides the description of CsmaAloha Class
 * 
 */


#ifndef CSMA_H 
#define CSMA_H

#include <mmac.h>
#include <iostream>
#include <string>
#include <map>
#include <set>
#include <queue>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <mphy.h>
#include <module.h>
#include <packet.h>
#include "hdr-csmaar-ack.h"
#define CSMA_DROP_REASON_WRONG_STATE "WST"		/**< The protocol cannot receive this kind of packet in this state */
#define CSMA_DROP_REASON_WRONG_RECEIVER "WRCV"		/**< The packet is for another node */
#define CSMA_DROP_REASON_UNKNOWN_TYPE "UPT"		/**< The type of the packet is unknown */
#define CSMA_DROP_REASON_BUFFER_FULL "DBF"		/**< The Buffer of DATA packets is full */
#define CSMA_DROP_REASON_ERROR "ERR"			/**< Packet corrupted */


extern packet_t PT_MMAC_ACK; 
/**
 * Class that describes a CsmaAr module 
*/
class CsmaAr : public MMac {

  
  public:
  //static double time_window_ar; 

  /**
   * Constructor of the CsmaAr class
   */
  CsmaAr();
   /**
	 * Destructor of the CsmaAr class
   */
  virtual ~CsmaAr();
	/**
	 * TCL command interpreter. It implements the following OTcl methods:
	 * 
	 * @param argc Number of arguments in <i>argv</i>.
	 * @param argv Array of strings which are the command parameters (Note that <i>argv[0]</i> is the name of the object).
	 * @return TCL_OK or TCL_ERROR whether the command has been dispatched successfully or not.
	 * 
	 **/
  virtual int command(int argc, const char*const* argv);
	/**
	 * Cross-Layer messages interpreter
	 * 
	 * @param ClMessage* an instance of ClMessage that represent the message received
	 * @return <i>0</i> if successful.
	 */
  virtual int crLayCommand(ClMessage* m);
 // virtual int getPacketSize_Ar(int size_ar);
 // virtual void stateRxData(Packet* p);

  
  protected:


  static const double prop_speed; /**< Typical sound propagation speed in underwater enviroment */
  /**< Variable that rapresent the status of the protocol machine state */
  enum CSMA_STATUS {
    CSMA_STATE_IDLE = 1, CSMA_STATE_BACKOFF, CSMA_STATE_TX_DATA, CSMA_STATE_TX_ACK, CSMA_STATE_WAIT_ACK, 
    CSMA_STATE_DATA_RX, CSMA_STATE_ACK_RX, CSMA_STATE_NOT_SET, CSMA_STATE_LISTEN,
    CSMA_STATE_CHK_ACK_TIMEOUT, CSMA_STATE_CHK_LISTEN_TIMEOUT, CSMA_STATE_CHK_BACKOFF_TIMEOUT, CSMA_STATE_RX_IDLE,
    CSMA_STATE_RX_LISTEN, CSMA_STATE_RX_BACKOFF, CSMA_STATE_RX_WAIT_ACK, CSMA_STATE_WRONG_PKT_RX
  };
  /**< Reason for the changing of the state */
  enum CSMA_REASON_STATUS {
    CSMA_REASON_DATA_PENDING, CSMA_REASON_DATA_RX, CSMA_REASON_DATA_TX, CSMA_REASON_ACK_TX, 
    CSMA_REASON_ACK_RX, CSMA_REASON_BACKOFF_TIMEOUT, CSMA_REASON_ACK_TIMEOUT, CSMA_REASON_DATA_EMPTY, 
    CSMA_REASON_NOT_SET, CSMA_REASON_MAX_TX_TRIES, CSMA_REASON_BACKOFF_PENDING, CSMA_REASON_LISTEN,
    CSMA_REASON_LISTEN_TIMEOUT, CSMA_REASON_LISTEN_PENDING, CSMA_REASON_START_RX, 
    CSMA_REASON_PKT_NOT_FOR_ME, CSMA_REASON_WAIT_ACK_PENDING, CSMA_REASON_PKT_ERROR
  };
  /**< Type of the packet */
  enum CSMA_PKT_TYPE {
    CSMA_ACK_PKT = 1, CSMA_DATA_PKT, CSMA_DATAMAX_PKT
  };
	/**< ACK modes of the protocol */
  enum CSMA_ACK_MODES {
    CSMA_ACK_MODE = 1, CSMA_NO_ACK_MODE
  };
  /**< Status of the timer */
  enum CSMA_TIMER_STATUS {
    CSMA_IDLE = 1, CSMA_RUNNING, CSMA_FROZEN, CSMA_EXPIRED
  };
 

	/**
	 * Class that describes the timers in the node
	 */
  class AlohaTimer : public TimerHandler {
    
    
    public:
	  
	  /**
	   * Constructor of the AlohaTimer class
	   * @param CsmaAr* a pointer to an object of type CsmaAr*
	   */  
    AlohaTimer(CsmaAr *m) : TimerHandler(), start_time(0.0), left_duration(0.0), counter(0), module(m), timer_status(CSMA_IDLE) { 
      assert(m != NULL); }
	/**
	 * Destructor of the AlohaTimer class
	*/
    virtual ~AlohaTimer() { }
    
	/**
	* Freezes the timer
	*/

  // original code ----
 /*
    virtual void freeze() { assert(timer_status == CSMA_RUNNING); left_duration = NOW - start_time; 
                            if (left_duration <= 0.0) left_duration = module->mac2phy_delay_; force_cancel();
                            timer_status = CSMA_FROZEN; }
	
	 // unFreezes is used to resume the timer starting from the point where it was freezed
	
    virtual void unFreeze() { assert(timer_status == CSMA_FROZEN); start_time = NOW; assert(left_duration > 0);
                              sched(left_duration); timer_status = CSMA_RUNNING; }
  */

  //--------- corrected back-off operation 
    virtual void freeze() { 
              if (module->debug_)  printf("%f CsmaAloha (%d) is in freeze()\n",NOW,module->addr); 
        //if (left_duration == 0) left_duration = module->backoff_duration; // added By Tham to correct left_duration (not necessary)
              if (module->debug_) printf("%f CsmaAloha (%d) left_duration = %f\n",NOW,module->addr,left_duration); //Tham 
                      assert(timer_status == CSMA_RUNNING); //left_duration = NOW - start_time; (wrong)
                      left_duration -= (NOW - start_time); // Fixed by THAM (2013/05/07) - confirmed to Desert author
                      if (left_duration <= 0.0) { left_duration = module->mac2phy_delay_; }
                      force_cancel();
                      timer_status = CSMA_FROZEN; 
               if (module->debug_)  printf("%f CsmaAloha (%d) left_duration = %f\n",NOW,module->addr,left_duration);              
                             }

//* unFreezes is used to resume the timer starting from the point where it was freezed
 
    virtual void unFreeze() { assert(timer_status == CSMA_FROZEN); 
                               start_time = NOW; assert(left_duration > 0);
                               sched(left_duration); 
                               timer_status = CSMA_RUNNING; 
                   if (module->debug_) printf("%f CsmaAloha (%d) is in unfreeze()\n",NOW,module->addr);                               
                              } // Fixed by THAM (2013/05/08)
    
  //-----------------------
	/**
	 * stops the timer
	*/
    virtual void stop() { timer_status = CSMA_IDLE; force_cancel(); }
	/**
	 * schedule a timer
	 * @param double the duration of the timer
	*/
    virtual void schedule( double val ) { start_time = NOW; left_duration = val; timer_status = CSMA_RUNNING; resched(val); }
	  /**
	   * checks if the timer is IDLE
	   * @return bool <i>true</i> or <i>false</i>
	   */
    bool isIdle() { return ( timer_status == CSMA_IDLE ); }
	  /**
	   * checks if the timer is RUNNING
	   * @return <i>true</i> or <i>false</i>
	   */
    bool isRunning() { return (timer_status == CSMA_RUNNING); }
	  /**
	   * Checks if the timer is EXPIRED
	   * @return <i>true</i> or <i>false</i>
	   */
    
    bool isExpired() { return (timer_status == CSMA_EXPIRED); }
	  /**
	   * Checks if the timer is FROZEN
	   * @return <i>true</i> or <i>false</i>
	   */
    bool isFrozen() { return (timer_status == CSMA_FROZEN); }
	/**
	   * Checks if the timer is ACTIVE
	   * @return <i>true</i> or <i>false</i>
	*/
       
    bool isActive() { return (timer_status == CSMA_FROZEN || timer_status == CSMA_RUNNING ); }
	  /**
	   * Resets the counter of the timer
	   */
    
    void resetCounter() { counter = 0; }
	  /**
	   * Increments the counter of the timer
	   */
	  
    void incrCounter() { ++counter; }
	  /**
	   * Returns the counter of the timer
	   * @return the value of the counter of the timer
	   */
	  
    
    int getCounter() { return counter; }
    
	  /**
	   * Returns the left duration of the timer
	   * @return left duration of the timer
	   */
    double getDuration() { return left_duration; }
    
    
    protected:

      
    double start_time; /**< Start Time of the timer */
    
    double left_duration; /**< Left duration of the timer */

    int counter; /**< counter of the timer */
    
    CsmaAr* module; /**< Pointer to an object of type CsmaAr */    
    CSMA_TIMER_STATUS timer_status; /**< Timer status */
    
    
  };
  
  /**
   * Class used to handle the timer of the backoff period.
   */
  class BackOffTimer : public AlohaTimer { 
  
    
    public: 
    
	/**
	 * Conscructor of BackOffTimer class 
	 * @param CsmaAloha* pointer to an object of type CsmaAloha
	*/  
    BackOffTimer(CsmaAr* m) : AlohaTimer(m) { }
	/**
	 * Destructor of DataTimer class 
	*/
    virtual ~BackOffTimer() { }
    
    
    protected:
      
	/**
	 * Method called when the timer expire
	 * @param Eevent*  pointer to an object of type Event
	*/  
    virtual void expire(Event *e);

  
  };
  
    /**
	 * Class used to handle the timer for waiting the ACK.
	 */
  class AckTimer : public AlohaTimer { 
  
    
    public:
    
	/**
	 * Conscructor of AckTimer class 
	 * @param CsmaAloha* pointer to an object of type CsmaAloha
	*/
    AckTimer(CsmaAr* m) : AlohaTimer(m) { }
	/**
	 * Destructor of AckTimer class 
	*/
    virtual ~AckTimer() { }
    
      
    protected:

	/**
	 * Method called when the timer expire
	 * @param Event*  pointer to an object of type Event
	*/ 
    virtual void expire(Event *e);
   // 
    

  };
  
   /**
	 * Class used to handle the Listen Timer. The duration of the timer is equal to the duration
	 * duration of the sense of the channel
   */
  class ListenTimer : public AlohaTimer { 
  
    
    public:
    
	/**
	 * Conscructor of ListenTimer class 
	 * @param CsmaAloha* pointer to an object of type CsmaAloha
	*/
    ListenTimer(CsmaAr* m) : AlohaTimer(m) { }
	/**
	 * Destructor of AckTimer class 
	*/
    virtual ~ListenTimer() { }
    
      
    protected:

	/**
	 * Method called when the timer expire
	 * @param Eevent*  pointer to an object of type Event
	*/ 
    virtual void expire(Event *e);

    
  };

//-----------------------------------Adding function for AR---------------------------------------------------------------
      /*   Class used to handle the Window Timer. The duration of the timer is given by time window value (default = 20 s)
           Added by Tham Nguyen UOU Korea 2012
           */
  class WindowTimer_Ar : public AlohaTimer { 
  
    public:
      
	/*
	 * Conscructor of WindowTimer class 
	 * @param CsmaAr* pointer to an object of type CsmaAloha*/
	
    WindowTimer_Ar(CsmaAr* m) : AlohaTimer(m) { }
    
	/**
	 * Destructor of AckTimer class */
	
    virtual ~WindowTimer_Ar() { }
    
      
    protected:

	/*
	 * Method called when the timer expire
	 * @param Eevent*  pointer to an object of type Event */
	
    virtual void expire(Event *e);
    
    
  };

//Sink calculate parameter whenever SinkTimer expire
 class SinkTimer : public AlohaTimer { 
  
    public:
      
	/*
	 * Conscructor of WindowTimer class 
	 * @param CsmaAr* pointer to an object of type CsmaAloha*/
	
    SinkTimer(CsmaAr* m) : AlohaTimer(m) { }
    
	//**
	 //* Destructor of AckTimer class : // corrected by Tham 2013/04
	
    virtual ~SinkTimer() { }
    
      
    protected:

	/*
	 * Method called when the timer expire
	 * @param Eevent*  pointer to an object of type Event */
	
    virtual void expire(Event *e);
    
    
  };

/////
// Get packet size take acount into headers, Tham Ng 2012
    virtual int getTransTime_Ar(int size_ar);

// Get background traffic (include retransmission) and original traffic of each node
    virtual double getTrafficWithinTimeWd_Ar();

// Sink process
   virtual void sinkProcess();

// create the array store the node addr when sink receive Data packet, Tham Ng 2012

    virtual void initArrayOfNode_Ar();

// Create the array store the number of transmission of each node in array of node addr, Tham Ng 2012

   virtual void initArrayBackgroundTrafficOfEachNode_Ar();
   virtual void initArrayOriginalTrafficOfEachNode_Ar();

// Create the array store the max_trans of each node
   virtual void initArrayMaxTransOfNode_Ar();

//Create the matrix to store the packet-id recve at sink
  virtual void initMatrixPacketRecv_Ar();

//---------------------------------------------------------------------------------------------------------

  /**
	 * Receives the packet from the upper layer (e.g. IP)
	 * @param Packet* pointer to the packet received
	 *
  */
  virtual void recvFromUpperLayers(Packet* p);
  /**
   * Pass the packet to the PHY layer
   * @param Packet* Pointer to an object of type Packet that rapresent the Packet to transmit
  */	
  virtual void Mac2PhyStartTx(Packet* p);
  /**
   * Method called when the PHY layer finish to transmit the packet.
   * @param Packet* Pointer to an object of type Packet that rapresent the Packet transmitted
  */
  virtual void Phy2MacEndTx(const Packet* p);
  /**
   * Method called when the Phy Layer start to receive a Packet 
   * @param const Packet* Pointer to an object of type Packet that rapresent the Packet that is in reception
  */
  virtual void Phy2MacStartRx(const Packet* p);
  /**
   * Method called when the Phy Layer finish to receive a Packet 
   * @param Packet* Pointer to an object of type Packet that rapresent the Packet received
  */
  virtual void Phy2MacEndRx(Packet* p);
  /**
   * Compute the time needed to transmit the packet (using a CrLayerMessage to ask the PHY to perform the computation)
   * @param CMSA_PKT_TYPE Type of the packet 
  */
  virtual double computeTxTime(CSMA_PKT_TYPE type);

  /**
   * Init the packet with the MAC address of the receiver and the sender,
   * the size of the packet and the type
   * @param UWPOLLING_PKT_TYPE the type of the packet
  */
   virtual void initPkt( Packet* p, CSMA_PKT_TYPE pkt_type, int dest_addr = 0);
  // virtual void initPkt( Packet* p, CSMA_PKT_TYPE type, int dest_addr = 0, int max_trans = 0); // For modify 2 Tham Ng
  /**
   * compute the BackOff time as backoff = backoff_tuner*random*2*ACK_timeout*2^(counter)
   * where counter is a value incremented each time this method is called and backoff_tuner is 
   * a multiplier factor chosen by the user
  */
  virtual double getBackoffTime();
  /**
   * Transmits the DATA packet (calling Mac2PhyStartTx) and increment the counter of transmitted data packets
   */
  virtual void txData();
  /**
   * Transmits the ACK packet (calling Mac2PhyStartTx) and increment the counter of transmitted ACK packets
   * @param in MAC address of the destination of ACK packet 
   */
  virtual void txAck(int dest_addr);
   //virtual void txAck(int dest_addr, int max_trans); //For modify 2
  /**
   * IDLE state. Each variable is resetted
  */
  virtual void stateIdle();
  /**
   * A reception is occuring while the protocol is in IDLE state
  */
  virtual void stateRxIdle();
  /**
   * State in which the protocol allows the node to transmit a data packet
  */
  virtual void stateTxData();
  /**
   * State in which the protocol set-up the ACK packet to transmit
   * @param int MAC address of the destination of the ACK packet
   */
  virtual void stateTxAck(int dest_addr);
   // virtual void stateTxAck(int dest_addr, int max_trans); // For modify 2
  /**
   * BackOff STATE. An ACK packet is lost. A backoff timer is set up. When the timer expire 
   * the protocol will re-send the data packet 
  */
  virtual void stateBackoff();
  /**
   * State in which a reception is occurring while the protocol is in the backoff state
  */
  virtual void stateRxBackoff();
  /**
   * State in which a DATA packet is sent. The time-out for receiving a ACK is set-up
  */
  virtual void stateWaitAck();
  /**
   * State in which a reception is occuring while the node is waiting an ACK 
   */
  virtual void stateRxWaitAck();
  /**
    * State in which the node is listening the channel
   */
  virtual void stateListen();
  /**
   * State in which a reception is occuring while the node is listening the channel
  */
  virtual void stateRxListen();
  /**
   * Checks if the Listen period is expired
   */
  virtual void stateCheckListenExpired();
  /**
   * Checks if the ACK reception timer is expired
  */
  virtual void stateCheckAckExpired();
  /**
   * Checks if the Backoff period is expired
  */
  virtual void stateCheckBackoffExpired();
  /**
   * State in which a DATA packet is received 
  */
  virtual void stateRxData(Packet* p);
  /**
   * state in which an ACK packet is received
   */
  virtual void stateRxAck(Packet* p);
	/**
	 * state in which a wrong Packet is received
	 */
  virtual void stateRxPacketNotForMe(Packet* p);
  /**
   * Prints in a file the textual information of the current state and the transitions reasons
   * @param double time lapse from the call of the method and the effective write process in the file (setted to zero by default)
   */

  virtual void printStateInfo(double delay = 0);
  /**
   * Initializes the protocol at the beginning of the simulation. This method is called by
   * a command in tcl. 
   * @param double delay
   * @see command method
   */
  virtual void initInfo();
	/**
	 * Refresh the State of the protocol
	 */
  virtual void refreshState(CSMA_STATUS state) { prev_prev_state = prev_state; prev_state = curr_state; curr_state = state; }
	/**
	 * Refresh the reason for the change of state
	 */
  virtual void refreshReason(CSMA_REASON_STATUS reason) { last_reason = reason; }
  /**
   * Method called when the Backoff timer is expired
   */
  virtual void exitBackoff();
  /**
   * Set the distance between the sender and the receiver
   * @param double Distance between the sender and the receiver
   */
  virtual void setSessionDistance(double distance) { session_distance = distance; }
  /**
   * Checks if the data packet received is a double packet (using the serial number of the packet)
   * @param int serial number of the received packet
   * @return <i>true</i> if the serial number of the last data pacekt received is greater than the
   * serial number of the last data packet received. <i>false</i> otherwise
   */
  virtual bool keepDataPkt(int serial_number);
  /**
   * Increases the number of times a packet is re-transmitted
   */
  virtual void incrCurrTxRounds() { curr_tx_rounds++; }
	/**
	 * Reset the number of times a data pacekt is re-transmitted
	 */
  virtual void resetCurrTxRounds() { curr_tx_rounds = 0; }
  /**
   * Update the RTT increasing the number of RTT samples and calculating the smoothed RTT using
   * the formula srtt = alpha*srtt + (1-alpha) * curr_rtt where alpha is a value chosen by the user
   * @param double current value of RTT
   */
 // virtual void updateRTT(double rtt);
  /**
   * get the value of RTT as mean of all the rttsamples
   * @return RTT value
   */
 // virtual double getRTT() { return (rttsamples>0) ? sumrtt/rttsamples : 0 ; }
  /**
   * Updates the AckTimeout calling getRTT, where the ACK timeout is computed as srtt/rttsamples using the smooth RTT
   * @see updateRTT
   * @param double RTT value
  */
 // virtual void updateAckTimeout(double rtt);
  /**
   * Updates the ID of the last DATA packet received
   */
  virtual void updateLastDataIdRx(int id) { last_data_id_rx = id; }
  /**
	* Pop the first element of the data packet queue
  */
  virtual void queuePop(bool flag = true) { Packet::free(Q.front()); Q.pop(); waitEndTime(flag); data_sn_queue.pop(); }
  /**
   * Resets the current session (e.g. the session distance)
   */ 
  virtual void resetSession();
  /**
   * Used for debug purposes. (Permit to have a "step by step" behaviour of the protocol)
  */
  virtual void waitForUser();

  // stats functions
  /**
   * Return the number of packets not transmitted (remained in the protocol queue) at the end 
   * of the simulation
   * @return the size of the queue at the end of the simulation
  */
  virtual int getRemainingPkts() { return(up_data_pkts_rx - Q.size()); }
  /**
   * Increase the number of Data packet Received from the Upper layers
   */
  virtual void incrUpperDataRx() {up_data_pkts_rx++;}

//---------original input----------------------

  static int max_tx_tries;		/**< Maximum number of re-transmissions for one packet */
  double backoff_tuner;		/**< Multiplier factor in the calculation of the backoff */
  double wait_costant;			/**< Adding factor in the calculation of the listen time */
  int max_payload;			/**< Maximum dimension of the data payload in bytes */
  int HDR_size;		        /**< Size (in bytes) of the header added by the protocol */
  int ACK_size;		        /**< Size of the ACK message */
  double ACK_timeout;			/**< Duration of the ACK waiting time */
  int buffer_pkts;			/**< Length of the data buffer in number of packets */
  double alpha_;			/**< smooth factor in the calculation of the RTT */
  double max_backoff_counter;	        /**< Number of times a backoff is calculated */
  double listen_time;	                /**< Time in which the node sense the channel */
  int counter;

  std::queue<Packet*> Q;		/**< Packet queue */	
  std::queue<int> data_sn_queue;       /**< Queue of the sequence number of the packets */

  static bool initialized;		/**< <i>true</i> if the protocol is initialized */ 
  static int u_pkt_id;	        /**< simulation-unique packet ID */

  int u_data_id;			/**< DATA packete ID */
  int last_sent_data_id;		/**< ID of the last sent packet */

  bool TxActive;			/**< flag that indicates if a transmission is occuring */
  bool RxActive;			/**< flag that indicates if a reception is occuring */
  bool session_active;			/**< flag that indicates if a session (i.e. a transmission/reception activity is occuring)
                                             between two nodes */
  bool print_transitions;		/**< flag that indicates if the protocol is enabled to print its state transitions on a file*/
  bool has_buffer_queue;		/**< flag that indicates if a node has a buffer where store DATA packets */

  double start_tx_time;		/**< timestamp in which the node stars to transmit a packet */
  double srtt;				/**< Smoothed Round Trip Time, calculated as for TCP */
  double sumrtt;			/**< sum of RTT samples */
  double sumrtt2;			/**< sum of (RTT^2) */
  int rttsamples;                      /**< num of RTT samples */  
  double acktimeout;		       /**< ack timeout> */
  int curr_tx_rounds;		       /**< Number of current transmission of the same packet */
  int last_data_id_rx;		       /**< ID of the last DATA packet received */
  
  Packet* curr_data_pkt;		/**< Pointer to the current data packet */

  double session_distance;		/**< Distance between sender and the receiver involved in the current session */

//---Addition for CSMA_ARS : Tham Nguyen 2012---------------

  static int total_orig; //tham
  static int recv_frame; //tham
  static int total_retrans;
  static int ack_rcv; // Number of ACK packet received at nodes
  static double time_window_ar;
  static int recv_first_frame;
  static int drop_pkt;

  double sink_time_window;
  double ps_ack;
  double rc_sim;
  double Ps; //tham
  int ack_sent;
  int number_of_flow_ar;
  int packet_length;
  int re_trans_no; //Number of re-trans - added by Tham Nguyen UOU Korea 2012
  int n1;
  int n2;
  int n3;
  int n4;
  int n5;
  int n6;
  int n7;
  int n8;
  int n9;
  int n10; //added by tham 
  double backoff_duration; // added by THAM (2013-05-04) 
 // double time_window_ar; //time window added by Tham Nguyen UOU, Korea 2012
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
 // double Os;
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

  WindowTimer_Ar window_timer_ar;      /**< Object that represents the window timer for 
                                            counting number of transmission in time window*/
  SinkTimer sink_timer;               //Timer for sink process added by Tham Nguyen, UOU Korea 2012

//--------------------------------------

  AckTimer ack_timer;			/**< Object that represents the ack timer */
  BackOffTimer backoff_timer;	        /**< Object that represents the backoff timer */
  ListenTimer listen_timer;		/**< Object that represents the listen timer */

  CSMA_REASON_STATUS last_reason;	/**< Reason for the state transitions */
  CSMA_STATUS curr_state;		/**< Current state of the protocol */
  CSMA_STATUS prev_state;		/**< Previous state of the protocol */
  CSMA_STATUS prev_prev_state;		/**< Previous previous state of the protocol */
  
  CSMA_ACK_MODES ack_mode;		/**< Variable that indicates if the protocol is in ACK or NO_ACK mode */

  static map< CSMA_STATUS , string > status_info;	/**< Textual description of the protocol states */
  static map< CSMA_REASON_STATUS, string> reason_info; /**< Textual description of the protocol reason for the change of the state*/
  static map< CSMA_PKT_TYPE, string> pkt_type_info;	/**< Textual description of the packet type */ 
														 
  ofstream fout;         /**< Object that handles the output file where the protocol writes the state transistions */
  
};

#endif /* CSMA_H */
