/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 *  FEC and NACK added bitrate is handled outside class
 *
 *  Implementation of the NADA congestion control algorithm
 *  as described in: https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 *
 *  Xiaoqing Zhu | 2018/12/20
 *
*/



#ifndef MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_
#define MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_

#include <deque>
#include <utility>
#include <vector>

#include "modules/rtp_rtcp/include/rtp_rtcp_defines.h"

namespace webrtc {

class RtcEventLog;

class NADABandwidthEstimation {
 public:

  NADABandwidthEstimation() = delete;
  explicit NADABandwidthEstimation(RtcEventLog* event_log);
  virtual ~NADABandwidthEstimation();

  // Retrieve current estimate
  void CurrentEstimate(int* bitrate, uint8_t* loss, int64_t* rtt) const;

  // Call periodically to update estimate.
  void UpdateEstimate(int64_t now_ms);

  // Call when we receive a RTCP message with TMMBR or REMB.
  void UpdateReceiverEstimate(int64_t now_ms, uint32_t bandwidth);

  // Call when a new delay-based estimate is available.
  void UpdateDelayBasedEstimate(int64_t now_ms, uint32_t bitrate_bps);

  // Call when we receive a RTCP message with a ReceiveBlock.
  void UpdateReceiverBlock(uint8_t fraction_loss,
                           int64_t rtt,
                           int number_of_packets,
                           int64_t now_ms);

  void SetBitrates(int send_bitrate,
                   int min_bitrate,
                   int max_bitrate);

  void SetSendBitrate(int bitrate);
  void SetMinMaxBitrate(int min_bitrate, int max_bitrate);
  int GetMinBitrate() const;

 private:

  int getRampUpMode();
  void AcceleratedRampUp(int64_t now_ms);
  void GradualRateUpdate(int64_t now_ms);

  //
  // [XZ 2018-12-20]  save for now ...
  //
  // Returns the input bitrate capped to the range
  // between min and max bandwidth.
  //
  // uint32_t CapBitrateToThresholds(int64_t now_ms, uint32_t bitrate);

  void ClipBitrate();  // Clip bitrate_ between [R_min, R_max] //TODO Sergio: check new method CapBitrateToThresholds

  // Updates history of:
  // -- min bitrates (to be depreciated for NADA)
  // -- max rtt/owd
  // -- max plr
  //
  // After this method returns xxx_history_.front().second contains the
  // min/max value used during last logging window Logwin.
  //
  void UpdateMinHistory(int64_t now_ms);
  void UpdateRttHistory(int64_t now_ms);
  void UpdatePlrHistory(int64_t now_ms);
  std::deque<std::pair<int64_t, uint32_t> > min_bitrate_history_;
  std::deque<std::pair<int64_t, int64_t> > max_rtt_history_;
  std::deque<std::pair<int64_t, uint8_t> > max_plr_history_;

  // incoming filters for calculating packet loss ratio
  int lost_packets_since_last_loss_update_Q8_;
  int expected_packets_since_last_loss_update_;

  //
  // key variables for NADA rate calculation
  //
  // rates: r_ref, RMIN, RMAX
  uint32_t bitrate_;                    // key variable holding calculated bandwidth: r_ref in draft
  uint32_t min_bitrate_configured_;     // min rate: RMIN in draft
  uint32_t max_bitrate_configured_;     // max rate: RMAX in draft

  // intervals: delta
  // int64_t last_rate_update_ms_;      // last time updating the rate (in ms) | t_last in draft
  int64_t last_feedback_ms_;            // last time receiving a feedback (in ms) | t_last in draft
  // int64_t rate_update_interval_ms_;  // previous rate update interval | delta = t_curr - t_last
  int64_t feedback_interval_ms_;        // previous feedback interval | delta = t_curr - t_last
  int64_t delta_;                       // update interval used for rate calculation | delta in draft

  // congestion level
  float nada_x_curr_;   // current congestion level  | x_curr in draft
  float nada_x_prev_;   // previous congestion level | x_prev in draft

  //
  // inherited from SenderSideBandwidthEstimation
  //
//  bool has_decreased_since_last_fraction_loss_;
//  int64_t last_feedback_ms_;		// last time receiving a feedback
//  int64_t last_packet_report_ms_;	//
//  int64_t last_timeout_ms_;
  uint8_t last_fraction_loss_;
//  uint8_t last_logged_fraction_loss_;
  int64_t last_round_trip_time_ms_;
  int64_t min_round_trip_time_ms_;

  uint32_t bwe_incoming_; 		// receiver-estimated bandwidth, not used
  uint32_t delay_based_bitrate_bps_;	// delay-based bandwidth estimation, not used

  // int64_t time_last_decrease_ms_;
  int64_t first_report_time_ms_;
  // int initially_lost_packets_;

  RtcEventLog* event_log_;
  int64_t last_rtc_event_log_ms_;

  bool in_timeout_experiment_;
};

}  // namespace webrtc

#endif  // MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_
