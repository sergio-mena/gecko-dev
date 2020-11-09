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

class NadaCore {
 public:
    NadaCore();
    virtual ~NadaCore();

    void TestFunction(const char * from) const;

    int GetMinBitrate() const; 

    void SetMinMaxBitrate(int min_bitrate, 
						  int max_bitrate); 

    void UpdateDelta(int64_t delta); 
    void UpdateRttStats(int64_t now_ms, int64_t rtt); 
    void UpdatePlrStats(int64_t now_ms, 
                        int nloss, 
                        int npkts); 
    void UpdatePktStats(int64_t now_ms, float dfwd, float rtt, int nloss, int npkts);

    // Updates history of:
  	// -- min bitrates 
  	// -- max rtt/owd
  	// -- max plr
  	// -- long term baseline rtt/owd
  	//
  	// After this method returns xxx_history_.front().second contains the
  	// min/max value used during last logging window Logwin.
  	//
  	std::deque<std::pair<int64_t, int64_t> > max_rtt_history_;
  	std::deque<std::pair<int64_t, int64_t> > min_rtt_history_;
  	std::deque<std::pair<int64_t, float> > max_plr_history_;
  	std::deque<std::pair<int64_t, float> > min_del_history_;
  	std::deque<std::pair<int64_t, float> > max_del_history_;

    int64_t GetRttmin(); 
    float GetDmin(); 

  	void UpdateRttHistory(int64_t now_ms, int64_t rtt); 
  	void UpdatePlrHistory(int64_t now_ms, float plr); 
    void UpdateDminHistory(int64_t now_ms, float dtmp);
    void UpdateDelHistory(int64_t now_ms, float dfwd);

  	// Set/get congestion signal value
  	float GetCongestion(); 
  	void UpdateRttCongestion();  
  	void UpdateOwdCongestion(); 

  	void SetRecvRate(const uint32_t rrate); 
  	float CalcRecvRate(uint64_t curr_ts, 
  					   uint64_t last_ts, 
  					   int nbytes); 

  	// Core calculations for NADA algorithm
  	int GetRampUpMode(int use_rtt);
  	uint32_t AcceleratedRampUp(const int64_t now_ms, 
							                  uint32_t rate_curr); 
	uint32_t GradualRateUpdate(const int64_t now_ms, 
							               uint32_t rate_curr); 
	int ClipBitrate(int bitrate);

	void LogUpdate(const char * algo, int ts); 

 private: 

 	int64_t delta_;  // update interval used for rate calculation | delta in draft

 	// per-interval stats
 	float nada_dfwd_; 
 	float nada_dq_; 
 	float nada_rtt_; 
 	float nada_relrtt_; 
 	int   nada_nloss_; 
 	float nada_plr_; 	// packet loss ratio:  XXX in draft
 	float nada_rrate_;  // receiving rate

 	int nada_rmode_; 
  	float nada_x_curr_;   // current congestion level  | x_curr in draft
  	float nada_x_prev_;   // previous congestion level | x_prev in draft

  	int nada_rate_in_bps_;  // key variable holding calculated bandwidth: r_ref in draft
  	int nada_rmin_in_bps_;  // configured minimum rate: RMIN in draft
  	int nada_rmax_in_bps_;  // configured maximum rate: RMAX in draft};
};


}  // namespace webrtc

#endif  // MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_
