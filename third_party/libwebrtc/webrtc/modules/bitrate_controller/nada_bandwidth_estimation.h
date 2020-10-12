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

  virtual void SetBitrates(int send_bitrate,
                           int min_bitrate,
                           int max_bitrate) override;
  virtual void SetSendBitrate(int bitrate) override;
  virtual void SetMinMaxBitrate(int min_bitrate, int max_bitrate) override;
  virtual int GetMinBitrate() const override;

 private:

  int getRampUpMode();
  void AcceleratedRampUp(int64_t now_ms);
  void GradualRateUpdate(int64_t now_ms);

  void ClipBitrate();  // Clip bitrate_ between [R_min, R_max]

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
  int lost_packets_since_last_loss_update_Q8_; //TODO: Sergio's question: what does "Q8" mean?
  int expected_packets_since_last_loss_update_;

  //
  // key variables for NADA rate calculation
  //
  // rates: r_ref, RMIN, RMAX
  uint32_t bitrate_;                    // key variable holding calculated bandwidth: r_ref in draft
  uint32_t min_bitrate_configured_;     // min rate: RMIN in draft
  uint32_t max_bitrate_configured_;     // max rate: RMAX in draft

  // intervals: delta
  int64_t last_feedback_ms_;            // last time receiving a feedback (in ms) | t_last in draft
  int64_t feedback_interval_ms_;        // previous feedback interval | delta = t_curr - t_last
  int64_t delta_;                       // update interval used for rate calculation | delta in draft

  // congestion level
  float nada_x_curr_;   // current congestion level  | x_curr in draft
  float nada_x_prev_;   // previous congestion level | x_prev in draft
  uint64_t nada_relrtt_;  // relative RTT 
  uint8_t last_fraction_loss_;
  int64_t last_round_trip_time_ms_;
  int64_t min_round_trip_time_ms_;

  uint32_t bwe_incoming_;               // receiver-estimated bandwidth, not used
  uint32_t delay_based_bitrate_bps_;    // delay-based bandwidth estimation, not used

  int64_t first_report_time_ms_;

  NadaCore core_;  // core calculations for NADA algorithm
};

}  // namespace webrtc

#endif  // MODULES_BITRATE_CONTROLLER_NADA_BANDWIDTH_ESTIMATION_H_
