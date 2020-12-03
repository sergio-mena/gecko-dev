/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_
#define MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_

#include <deque>

namespace webrtc {

#define NADA_RMODE_ACCELERATED_RAMPUP 0
#define NADA_RMODE_GRADUAL_UPDATE 1

constexpr int kNadaDefaultBitrate =  600000;          // Default initial rate: 600Kbps 
constexpr int64_t kNadaDefaultFBIntervalMs = 100;     // Default feedback interval in ms 
constexpr int kNadaMinFilterWin = 5;                  // # of taps for minimum filtering on one-way-delay 

class NadaCore {
 public:
    NadaCore();
    virtual ~NadaCore();

    int GetMinBitrate() const; 
    void SetMinMaxBitrate(int min_bitrate, 
						              int max_bitrate); 

    void UpdateDelta(int64_t delta);      
    void UpdateOwdStats(int64_t now_ms, int64_t dfwd);
    void UpdateRttStats(int64_t now_ms, int64_t rtt); 
    void UpdatePlrStats(int64_t now_ms, int nloss, int npkts); 

  	void SetRecvRate(const uint32_t rrate); 
  	void CalcRecvRate(uint64_t curr_ts, 
  					          uint64_t last_ts, 
  					          int nbytes); 

    int UpdateNadaRate(const int64_t now_ms, 
                       const bool use_rtt);

    void LogUpdate(const char * algo, const int64_t ts); 

 private: 

    // Maintains and updates timestamped history of: 
    // -- long term baseline rtt/owd
    // -- max rtt/owd
    // -- max plr    
    // 
    // After the update, xxx_history_.front().second contains
    // the min/max value used during last logging window.
    void UpdateRttHistory(int64_t now_ms, int64_t rtt); 
    void UpdatePlrHistory(int64_t now_ms, float plr); 
    void UpdateOwdHistory(int64_t now_ms, float dfwd);  
    std::deque<std::pair<int64_t, int64_t> > min_rtt_history_;
    std::deque<std::pair<int64_t, int64_t> > max_rtt_history_;
    std::deque<std::pair<int64_t, float> > min_owd_history_;
    std::deque<std::pair<int64_t, float> > max_owd_history_;
    std::deque<std::pair<int64_t, float> > max_plr_history_;

    // Supporting functions for NADA Rate Calculation
    void UpdateCongestion(const bool use_rtt);   // Update composite congestion signal (x_curr) 
                                                 // based on OWD or RTT

    void GetRampUpMode(const bool use_rtt);
    
    void AcceleratedRampUp(const int64_t now_ms); 

    void GradualRateUpdate(const int64_t now_ms); 

    void ClipBitrate();

    int64_t delta_;  // feedback interval used for rate calculation | delta in draft

    // per-interval packet statistics
 	  float nada_dfwd_;       // one-way forward delay
 	  float nada_dq_;         // queuing delay
 	  int64_t nada_rtt_;      // instantaneous RTT
 	  int64_t nada_relrtt_;   // relative RTT
 	  int   nada_nloss_;      // # of losses
 	  float nada_plr_; 	      // packet loss ratio:  XXX in draft
 	  float nada_rrate_;      // receiving rate

    // NADA rate calculation
  	float nada_x_curr_;     // current congestion level  | x_curr in draft
  	float nada_x_prev_;     // previous congestion level | x_prev in draft

    int nada_rmode_;        // 0: Accelerated Ramp Up | 1: Gradual Rate Update
  	int nada_rate_in_bps_;  // key variable holding calculated bandwidth: r_ref in draft
  	int nada_rmin_in_bps_;  // configured minimum rate: RMIN in draft
  	int nada_rmax_in_bps_;  // configured maximum rate: RMAX in draft}
};


}  // namespace webrtc

#endif  // MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_
