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

    void UpdateDelta(int64_t delta); 

    // Updates history of:
  	// -- min bitrates (to be depreciated for NADA)
  	// -- max rtt/owd
  	// -- max plr
  	//
  	// After this method returns xxx_history_.front().second contains the
  	// min/max value used during last logging window Logwin.
  	//
  	void ClearRminHistory(); 
  	void UpdateRminHistory(int64_t now_ms, uint32_t rate_curr); 
  	void UpdateRttHistory(int64_t now_ms, int64_t rtt); 
  	void UpdatePlrHistory(int64_t now_ms, uint8_t plr); 
  	std::deque<std::pair<int64_t, uint32_t> > min_bitrate_history_;
  	std::deque<std::pair<int64_t, int64_t> > max_rtt_history_;
  	std::deque<std::pair<int64_t, uint8_t> > max_plr_history_;

  	// Set/get congestion signal value
  	float GetCongestion(); 
  	void UpdateCongestion(int64_t val);  

  	// Core calculations for NADA algorithm
  	int getRampUpMode(int64_t rtt_min);
  	uint32_t AcceleratedRampUp(const int64_t now_ms, 
							   const int64_t rtt_curr, 
							   uint32_t rate_curr); 
	uint32_t GradualRateUpdate(const int64_t now_ms, 
							   const uint32_t rate_max, 
							   uint32_t rate_curr); 

	void LogUpdate(); 

 private: 
 	int64_t delta_;  // update interval used for rate calculation | delta in draft
  	float nada_x_curr_;   // current congestion level  | x_curr in draft
  	float nada_x_prev_;   // previous congestion level | x_prev in draft

};

}  // namespace webrtc

#endif  // MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_
