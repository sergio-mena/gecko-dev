/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_CONGESTION_CONTROLLER_NADA_OWD_BWE_H_
#define WEBRTC_MODULES_CONGESTION_CONTROLLER_NADA_OWD_BWE_H_

#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include "webrtc/base/checks.h"
#include "webrtc/base/constructormagic.h"
#include "webrtc/base/rate_statistics.h"
#include "webrtc/base/thread_checker.h"
#include "webrtc/modules/congestion_controller/delay_based_bwe.h"  // for Result struct

namespace webrtc {

class NadaOwdBwe {
 public:

  // static const int64_t kStreamTimeOutMs = 2000;
  /*
  struct Result {
    Result() : updated(false), probe(false), target_bitrate_bps(0) {}
    Result(bool probe, uint32_t target_bitrate_bps)
        : updated(true), probe(probe), target_bitrate_bps(target_bitrate_bps) {}
    bool updated;
    bool probe;
    uint32_t target_bitrate_bps;
  };

  */

  explicit NadaOwdBwe(Clock* clock);
  virtual ~NadaOwdBwe() {}

  // Triggers BW estimation upon receving a new packet FB vector 
  DelayBasedBwe::Result IncomingPacketFeedbackVector(
      const std::vector<PacketInfo>& packet_feedback_vector);

  // Update local variables fed by others:  RTT, R_min
  void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms);
  void SetMinBitrate(int min_bitrate_bps);

  // Answer queries: 
  bool LatestEstimate(std::vector<uint32_t>* ssrcs,
                      uint32_t* bitrate_bps) const;

  int64_t GetProbingIntervalMs() const;

 private:

  // Computes a bayesian estimate of the throughput given acks containing
  // the arrival time and payload size. Samples which are far from the current
  // estimate or are based on few packets are given a smaller weight, as they
  // are considered to be more likely to have been caused by, e.g., delay spikes
  // unrelated to congestion.
  class BitrateEstimator {
   public:
    BitrateEstimator();
    void Update(int64_t now_ms, int bytes);
    // rtc::Optional<uint32_t> bitrate_bps() const;
    uint32_t bitrate_bps() const;

   private:
    float UpdateWindow(int64_t now_ms, int bytes, int rate_window_ms);
    int sum_;
    int64_t current_win_ms_;
    int64_t prev_time_ms_;
    float bitrate_estimate_;
    float bitrate_estimate_var_;
    RateStatistics old_estimator_;
    const bool in_experiment_;
  };

  // Core NADA BW Estimation Calculations
   int GetRampUpMode();
  void AcceleratedRampUp(int64_t now_ms);
  void GradualRateUpdate(int64_t now_ms);
  void ClipBitrate();  // Clip bitrate_ between [R_min, R_max]
 
  // DelayBasedBwe::Result IncomingPacketInfo(const PacketInfo& info);

  // Updates the current remote rate estimate and returns true if a valid
  // estimate exists.
  // bool UpdateEstimate(int64_t packet_arrival_time_ms,
  //                    int64_t now_ms,
  //                     rtc::Optional<uint32_t> acked_bitrate_bps,
  //                     uint32_t* target_bitrate_bps);
  // const bool in_trendline_experiment_;
  // const bool in_median_slope_experiment_;

  rtc::ThreadChecker network_thread_;
  Clock* const clock_;

  BitrateEstimator receiver_incoming_bitrate_;  // for estimating recevied rate, used for Accelerated Ramp Up calculation 
  int64_t last_update_ms_;			// timestamp for last rate update: t_last in draft 
  int64_t first_update_ms_;			// timestamp for first rate update: t_init
  int64_t last_seen_packet_ms_;			// timestamp for last seen packet: t_last in draft (?)

  // history of plr and dfwd 
  void UpdateDminHistory(int64_t now_ms, float dtmp);
  void UpdateDelHistory(int64_t now_ms);
  void UpdatePlrHistory(int64_t now_ms);
  std::deque<std::pair<int64_t, float> > dmin_history_;
  std::deque<std::pair<int64_t, int64_t> > max_del_history_;
  std::deque<std::pair<int64_t, uint8_t> > max_plr_history_;


// variables for NADA BW estimation
  uint32_t nada_rate_in_bps_;  // key variable holding calculated bandwidth: r_ref in draft 
  uint32_t nada_rmin_in_bps_;  // configured minimum rate: RMIN in draft 
  uint32_t nada_rmax_in_bps_;  // configured maximum rate: RMAX in draft
  uint32_t nada_recv_in_bps_;  // estimated receiving rate based on FB reports

  float  nada_rtt_in_ms_;    // measured RTT used for Accelerated Ramp Up calculation
  float  nada_rtt_base_in_ms_;   // baseline RTT  
  float  nada_rtt_rel_in_ms_;    // relative RTT  
  float  nada_rtt_avg_in_ms_;    // average RTT fed by others  
     
  float nada_x_curr_;   // current congestion level:  x_curr in draft 
  float nada_x_prev_;   // previous congestion level: x_prev in draft 
  float nada_delta_; 	// update interval:  delta in draft
  float nada_d_fwd_;    // current forward one-way-delay:  d_fwd in draft
  float nada_d_base_;   // baseline forward one-way-delay along path: d_base in draft
  float nada_d_queue_;  // queuing delay: d_queue in draft  
  float nada_plr_; 	// packet loss ratio:  XXX in draft 

  int64_t probing_interval_in_ms_; 

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(NadaOwdBwe);
};

}  // namespace webrtc

#endif  // WEBRTC_MODULES_CONGESTION_CONTROLLER_NADA_OWD_BWE_H_
