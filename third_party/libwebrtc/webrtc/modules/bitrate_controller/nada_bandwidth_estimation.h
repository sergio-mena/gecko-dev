/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 *  FEC and NACK added bitrate is handled by outside class
 *
 *  
 *  Invokes the NADA congestion control algorithm (implemented by the 
 *  NadaCore module) using RTT as the congestion signal; modified following
 *  send_side_bandwidth_estimation.cc/h as the initial example
 *
*/

#ifndef MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_
#define MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_

#include <utility>
#include <vector>

#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"
#include "modules/remote_bitrate_estimator/include/nada_core.h"
#include "send_side_bandwidth_estimation_interface.h"

namespace webrtc {

class RtcEventLog;

class NADABandwidthEstimation: public SendSideBandwidthEstimationInterface {
 public:
  explicit NADABandwidthEstimation();
  virtual ~NADABandwidthEstimation();

  // Retrieve current estimate
  virtual void CurrentEstimate(int* bitrate, uint8_t* loss, int64_t* rtt) const override;

  // Call periodically to update estimate.
  virtual void UpdateEstimate(int64_t now_ms) override;

  // Call when we receive a RTCP message with TMMBR or REMB.
  virtual void UpdateReceiverEstimate(int64_t now_ms, uint32_t bandwidth) override;

  // Call when a new delay-based estimate is available.
  virtual void UpdateDelayBasedEstimate(int64_t now_ms, uint32_t bitrate_bps) override;

  // Call when we receive a RTCP message with a ReceiveBlock.
  virtual void UpdateReceiverBlock(uint8_t fraction_loss,
                                   int64_t rtt,
                                   int number_of_packets,
                                   int64_t now_ms) override;

  // Setters and Getters
  virtual void SetBitrates(int send_bitrate,
                           int min_bitrate,
                           int max_bitrate) override;
  virtual void SetSendBitrate(int bitrate) override;
  virtual void SetMinMaxBitrate(int min_bitrate, int max_bitrate) override;
  virtual int  GetMinBitrate() const override;

 private:

  int64_t first_report_time_ms_;        // for calculating relative time stamps for logging

  int64_t last_feedback_ms_;            // last time receiving a feedback (in ms) | t_last in draft
  int64_t feedback_interval_ms_;        // previous feedback interval | delta = t_curr - t_last

  uint8_t last_fraction_loss_;          // local cache of PLR obtained from UpdateReceiverBlock 
  int64_t last_round_trip_time_ms_;     // local cache of RTT obtained from UpdateReceiverBlock

  uint32_t bwe_incoming_;               // receiver-estimated bandwidth | r_recv in draft 
  uint32_t delay_based_bitrate_bps_;    // delay-based bandwidth estimation, 
                                        // reported by NadaOwdBwe when NADA-OWD mode is on

  int bitrate_;                         // local cache of calculated bandwidth | r_ref in draft

  NadaCore core_;                       // core calculations for NADA algorithm

  bool use_delay_based_;                // whether to overwrite rate calculation with delay-based estimation
};

}  // namespace webrtc

#endif  // MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_
