/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_NADA_OWD_BWE_H_
#define MODULES_CONGESTION_CONTROLLER_NADA_OWD_BWE_H_

#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include "modules/congestion_controller/delay_based_bwe_interface.h"
#include "modules/remote_bitrate_estimator/include/nada_core.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/race_checker.h"

namespace webrtc {

class RtcEventLog;

class NadaOwdBwe: public DelayBasedBweInterface {
 public:

  explicit NadaOwdBwe(const Clock* clock);
  virtual ~NadaOwdBwe();

  // Triggers BW estimation upon receiving a new packet FB vector
  virtual DelayBasedBweInterface::Result IncomingPacketFeedbackVector(
      const std::vector<PacketFeedback>& packet_feedback_vector,
      rtc::Optional<uint32_t> acked_bitrate_bps) override;

  // Update local variables fed by others:  RTT, R_min
  virtual void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) override;
  
  // Getters and setters
  virtual bool LatestEstimate(std::vector<uint32_t>* ssrcs,
                      uint32_t* bitrate_bps) const override;
  virtual void SetStartBitrate(int start_bitrate_bps) override;
  virtual void SetMinBitrate(int min_bitrate_bps) override;
  virtual int64_t GetExpectedBwePeriodMs() const override;

 private:

  rtc::RaceChecker network_race_;
  const Clock* const clock_;

  int64_t last_update_ms_;           // timestamp for last rate update: t_last in draft
  int64_t first_update_ms_;          // timestamp for first rate update: t_init
  int64_t last_seen_packet_ms_;      // timestamp for last seen packet: t_last in draft (?)
  int64_t last_seen_seqno_;          // sequence number for last seen packet, for plr estimation

  uint64_t last_arrival_time_ms_;

  // for NADA BW estimation
  int nada_rate_in_bps_;  // local cache of calculated bandwidth | r_ref in draft
  NadaCore core_;          // core calculations for NADA algorithm

  RTC_DISALLOW_IMPLICIT_CONSTRUCTORS(NadaOwdBwe);
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_NADA_OWD_BWE_H_
