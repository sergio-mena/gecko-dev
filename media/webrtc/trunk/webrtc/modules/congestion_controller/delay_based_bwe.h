/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_DELAY_BASED_BWE_H_
#define MODULES_CONGESTION_CONTROLLER_DELAY_BASED_BWE_H_

#include <memory>
#include <utility>
#include <vector>

#include "modules/congestion_controller/delay_based_bwe_interface.h"
#include "modules/congestion_controller/median_slope_estimator.h"
#include "modules/congestion_controller/probe_bitrate_estimator.h"
#include "modules/congestion_controller/trendline_estimator.h"
#include "modules/remote_bitrate_estimator/aimd_rate_control.h"
#include "modules/remote_bitrate_estimator/include/remote_bitrate_estimator.h"
#include "modules/remote_bitrate_estimator/inter_arrival.h"
#include "modules/remote_bitrate_estimator/overuse_detector.h"
#include "modules/remote_bitrate_estimator/overuse_estimator.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/race_checker.h"

namespace webrtc {

class RtcEventLog;

class DelayBasedBwe: public DelayBasedBweInterface {
 public:
  static const int64_t kStreamTimeOutMs = 2000;

  DelayBasedBwe(RtcEventLog* event_log, const Clock* clock);
  virtual ~DelayBasedBwe();

  virtual DelayBasedBweInterface::Result IncomingPacketFeedbackVector(
      const std::vector<PacketFeedback>& packet_feedback_vector,
      rtc::Optional<uint32_t> acked_bitrate_bps) override;
  virtual void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) override;
  virtual bool LatestEstimate(std::vector<uint32_t>* ssrcs,
                      uint32_t* bitrate_bps) const override;
  virtual void SetStartBitrate(int start_bitrate_bps) override;
  virtual void SetMinBitrate(int min_bitrate_bps) override;
  virtual int64_t GetExpectedBwePeriodMs() const override;

 private:
  void IncomingPacketFeedback(const PacketFeedback& packet_feedback);
  Result OnLongFeedbackDelay(int64_t arrival_time_ms);
  Result MaybeUpdateEstimate(bool overusing,
                             rtc::Optional<uint32_t> acked_bitrate_bps,
                             bool request_probe);
  // Updates the current remote rate estimate and returns true if a valid
  // estimate exists.
  bool UpdateEstimate(int64_t now_ms,
                      rtc::Optional<uint32_t> acked_bitrate_bps,
                      bool overusing,
                      uint32_t* target_bitrate_bps);

  rtc::RaceChecker network_race_;
  RtcEventLog* const event_log_;
  const Clock* const clock_;
  std::unique_ptr<InterArrival> inter_arrival_;
  std::unique_ptr<TrendlineEstimator> trendline_estimator_;
  OveruseDetector detector_;
  int64_t first_seen_packet_ms_;    // [XZ 2019-10-21 logging of relative time]
  int64_t last_seen_packet_ms_;     // [XZ 2019-10-21 logging of FB interval]
  int64_t feedback_interval_ms_;    // [XZ 2019-10-21 logging of FB interval]
  int64_t last_seen_seqno_;         // [XZ 2019-10-21 logging of pkt seq #]
  int default_bwe_npkts_;           // [XZ 2019-10-21 logging of # of ACKed pkts]
  int default_bwe_ploss_;           // [XZ 2019-10-21 logging of pkt loss count]
  double default_bwe_plr_;          // [XZ 2019-10-21 logging of pkt loss ratio]
  uint64_t default_bwe_rtt_ms_;     // [XZ 2019-10-21 logging of per-pkt RTT]
  uint64_t default_bwe_dqel_ms_;    // [XZ 2019-10-21 logging of queuing delay] 
  int64_t default_bwe_dbase_ms_;   // [XZ 2019-10-21 logging of baseline one-way delay]
  int default_bwe_nbytes_;        // [XZ 2019-10-21 logging of receiving rate]
  double default_bwe_rrate_;      // [XZ 2019-10-21 logging of receiving rate]
  int64_t last_arrival_time_ms_;   // [XZ 2019-10-21 logging of receiving rate]
  uint64_t curr_arrival_time_ms_;   // [XZ 2019-10-21 logging of receiving rate]

  bool uma_recorded_;
  AimdRateControl rate_control_;
  ProbeBitrateEstimator probe_bitrate_estimator_;
  size_t trendline_window_size_;
  double trendline_smoothing_coeff_;
  double trendline_threshold_gain_;
  int consecutive_delayed_feedbacks_;
  uint32_t prev_bitrate_;
  BandwidthUsage prev_state_;
  bool in_sparse_update_experiment_;

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(DelayBasedBwe);
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_DELAY_BASED_BWE_H_
