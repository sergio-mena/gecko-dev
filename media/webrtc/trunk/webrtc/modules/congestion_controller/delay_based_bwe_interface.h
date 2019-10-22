/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_CONGESTION_CONTROLLER_DELAY_BASED_BWE_INTERFACE_H_
#define MODULES_CONGESTION_CONTROLLER_DELAY_BASED_BWE_INTERFACE_H_

#include <vector>

#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "rtc_base/constructormagic.h"

namespace webrtc {

class RtcEventLog;

class DelayBasedBweInterface {
 public:
  struct Result {
    Result();
    Result(bool probe, uint32_t target_bitrate_bps);
    ~Result();
    bool updated;
    bool probe;
    uint32_t target_bitrate_bps;  
    bool recovered_from_overuse;
  };

  DelayBasedBweInterface();
  virtual ~DelayBasedBweInterface();

  virtual Result IncomingPacketFeedbackVector(
      const std::vector<PacketFeedback>& packet_feedback_vector,
      rtc::Optional<uint32_t> acked_bitrate_bps) = 0;
  virtual void OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) = 0;
  virtual bool LatestEstimate(std::vector<uint32_t>* ssrcs,
                      uint32_t* bitrate_bps) const = 0;
  virtual void SetStartBitrate(int start_bitrate_bps) = 0;
  virtual void SetMinBitrate(int min_bitrate_bps) = 0;
  virtual int64_t GetExpectedBwePeriodMs() const = 0;

  RTC_DISALLOW_COPY_AND_ASSIGN(DelayBasedBweInterface);
};

}  // namespace webrtc

#endif  // MODULES_CONGESTION_CONTROLLER_DELAY_BASED_BWE_INTERFACE_H_
