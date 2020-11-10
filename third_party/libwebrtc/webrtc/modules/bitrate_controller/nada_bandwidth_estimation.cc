/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/bitrate_controller/nada_bandwidth_estimation.h"

#include <algorithm>
#include <cmath>

#include "rtc_base/checks.h"
#include "rtc_base/logging.h"

#define USE_DELAY_BASED 1

namespace webrtc {

namespace {

constexpr int kNADAParamRateBps =  600000;  // Default rate: 600Kbps
// constexpr int kNADALimitNumPackets = 20;    // Number of packets before packet loss calculation is
                                        // considered as valid (outside the scope of NADA draft)

}  // namespace

NADABandwidthEstimation::NADABandwidthEstimation()
    : SendSideBandwidthEstimationInterface(),
      first_report_time_ms_(-1),
      last_feedback_ms_(-1),
      feedback_interval_ms_(0),
      last_fraction_loss_(0),
      last_round_trip_time_ms_(0),
      bwe_incoming_(0),
      delay_based_bitrate_bps_(kNADAParamRateBps),
      bitrate_(kNADAParamRateBps),
      core_() {

  printf("Initializing the RTT-based NADA BW Estimation Module\n");

  RTC_LOG(LS_INFO) << "Initializing the RTT-based NADA BW Estimation Module"<< std::endl;
}

NADABandwidthEstimation::~NADABandwidthEstimation() {

}

/*
 * Existing APIs for setting/getting bitrates.
 *
 */
void NADABandwidthEstimation::SetBitrates(int send_bitrate,
                                          int min_bitrate,
                                          int max_bitrate) {
  if (send_bitrate > 0)
    SetSendBitrate(send_bitrate);

  SetMinMaxBitrate(min_bitrate, max_bitrate);

  RTC_LOG(LS_INFO) << "NADA SetBitrates: bitrate_ = " << bitrate_/1000
                   << "Kbps."  << std::endl;
}

void NADABandwidthEstimation::SetSendBitrate(int bitrate) {

  RTC_DCHECK_GT(bitrate, 0);
  bitrate_ = bitrate;

  RTC_LOG(LS_INFO) << "NADA SetSendBitrate: bitrate_ = " << bitrate_/1000
                   << "Kbps." << std::endl;
}

void NADABandwidthEstimation::SetMinMaxBitrate(int min_bitrate,
                                               int max_bitrate) {

  // lower bound by default min rate in congestion controller
  min_bitrate = std::max(min_bitrate, congestion_controller::GetMinBitrateBps());
  core_.SetMinMaxBitrate(min_bitrate, max_bitrate); 

}

int NADABandwidthEstimation::GetMinBitrate() const {

  return core_.GetMinBitrate(); 
}

//
// [XZ 2019-03-07 Report query of currently estimated bitrate]
//
void NADABandwidthEstimation::CurrentEstimate(int* bitrate,
                                              uint8_t* loss,
                                              int64_t* rtt) const {

// TODO (sergio): Convince Xiaoqing to remove delay_based_bitrate_bps_ altogether: 
// it's the same as bitrate_ !!
// 
// [Notes: 2020-11-08]
// Xiaoqing's observation on Mozilla behavior when USE_DELAY_BASED is active (1): 
// 
// In NADA-RTT mode (use_transport_cc as false), the delay_based_bitrate_bps_ is 
// chosen instead of bitrate_, but UpdateDelayBasedEstimate() is never called so 
// delay_based_bitrate_bps_ is stuck at default rate -- so this is undesired
// 
// In NADA-OWD mode (use_transport_cc as true), the delay_based_bitrate_bps_ is
// chosen based on rate calculation from NadaOwdBwe whereas bitrate_ is 
// still periodically updated. So reporting the bitrate as delay_based_bitrate_bps_
// is desired.
// 
// Question: can we switch the flag of USE_DELAY_BASED according to the
// configuration flag use_transport_cc?   
//
// Proposed alternative modification: simply override bitrate_ with input 
// rate of the UpdateDelayBasedEstimate() function whenever it is invoked
//  

#ifdef USE_DELAY_BASED
  *bitrate = delay_based_bitrate_bps_;
#else
  *bitrate = bitrate_;
#endif

  *loss = last_fraction_loss_;
  *rtt = last_round_trip_time_ms_;

  printf("NADA CurrentEstimate: bitrate_ = %8.2f, delay_based_bitrate_bps_ = %8.2f Kbps | rate = %.2f Kbps, loss = %d, rtt = %lld ms\n",
        bitrate_/1000., delay_based_bitrate_bps_/1000.,
       *bitrate/1000.,  *loss, *rtt);

  RTC_LOG(LS_INFO) << "NADA CurrentEstimate: " 
                   << " | bitrate_: " << bitrate_/1000. << " Kbps"
                   << " | delay_based_bitrate_bps_:  " << delay_based_bitrate_bps_/1000 << " Kbps"
                   << " | reported rate: " << *bitrate/1000. << " Kbps"
                   << " | loss: " << *loss*100 << " %"
                   << " | rtt: "  << *rtt << " ms"  << std::endl;
}

// [XZ 2020-11-08]  
// Currently receiver estimated rate is unused for reference rate calculation; 
// passed along to NadaCore for the purpose of stats reporting/logging only 
void NADABandwidthEstimation::UpdateReceiverEstimate(
    int64_t now_ms, uint32_t bandwidth) {

  bwe_incoming_ = bandwidth;

  core_.SetRecvRate(bwe_incoming_); // pass along to NadaCore

  RTC_LOG(LS_INFO) << "NADA UpdateReceiverEstimate: now = " << now_ms-first_report_time_ms_
                   << " ms, bwe_incoming_ = " << bandwidth/1000
                   << " Kbps" << std::endl;
}

void NADABandwidthEstimation::UpdateDelayBasedEstimate(
    int64_t now_ms,
    uint32_t bitrate_bps) {

  delay_based_bitrate_bps_ = bitrate_bps;

  printf("NADA UpdateDelayBasedEstimate: %.2f Kbps at %lld ms\n",
         delay_based_bitrate_bps_/1000.,
         now_ms-first_report_time_ms_);

  RTC_LOG(LS_INFO) << "NADA UpdateDelayBasedEstimate: now = " << now_ms-first_report_time_ms_
                   << " ms, delay_based_rate_ = " << bitrate_bps/1000
                   << " Kbps" << std::endl;
}

/*
 * Upon receiving a Receiver Feedback Report, update stats for
 * packet loss and RTT measures
 *
 */
// TODO: what's the unit for the input fraction_loss variable? 
void NADABandwidthEstimation::UpdateReceiverBlock(uint8_t fraction_loss,
                                                  int64_t rtt,
                                                  int number_of_packets,
                                                  int64_t now_ms) {

  if (last_feedback_ms_ == -1) {
      feedback_interval_ms_ = 0;
      last_feedback_ms_ = now_ms;
  } else {
      feedback_interval_ms_ = now_ms - last_feedback_ms_;
      last_feedback_ms_ = now_ms;
  }
    
  if (first_report_time_ms_ == -1) {
    first_report_time_ms_ = now_ms;
  }

  printf("NADA UpdateReceiverBlock at %lld ms...rtt = %lld, loss = %d, npkts = %d\n",
         now_ms-first_report_time_ms_, rtt, fraction_loss, number_of_packets);

  RTC_LOG(LS_INFO) << "NADA UpdateReceiverBlock: now = " << now_ms-first_report_time_ms_
                   << " ms, fb_interval = " << feedback_interval_ms_
                   << " ms, loss = " << int(fraction_loss)
                   << " , rtt = " << rtt
                   << " ms, npackets = " << number_of_packets << std::endl;

  // Only update loss/rtt stats and rate when feedback containts more than 1 packet
  if (number_of_packets > 0) {

    // save local copy to serve queries
    last_round_trip_time_ms_ = rtt;
    last_fraction_loss_ = fraction_loss; 
    
    core_.UpdateDelta(feedback_interval_ms_); // update feedback interval delta_
    
    // Update RTT and base-RTT
    core_.UpdateRttStats(now_ms, rtt);    // update RTT stats

    int nloss = fraction_loss * number_of_packets;
    core_.UpdatePlrStats(now_ms, nloss, number_of_packets); 

    // call rate update calculation
    // core_.UpdateRttCongestion();   // update aggregate congestion stats accordingly
    UpdateEstimate(now_ms);
  }

}

// call calculations in NadaCore for reference rate update
void NADABandwidthEstimation::UpdateEstimate(int64_t now_ms) {

    int64_t ts = now_ms-first_report_time_ms_; 
    
    if (last_feedback_ms_ == -1) {

      // no feedback message yet: staying with current rate
      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: no feedback yet -- "
                   << "ts: "	<< ts  << " ms "
                   << "rate: " << bitrate_/1000 << " Kbps." << std::endl;

    }  else if (now_ms == last_feedback_ms_) {

      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: triggered by feedback -- "
                          << "ts: " << ts  << " ms "
                          << "rate: " << bitrate_/1000 << " Kbps "
                          << "feedback interval: " << feedback_interval_ms_ << " ms. " << std::endl;

    // calculate reference rate via NadaCore, in rtt-based mode (use_rtt=true)
    bitrate_ = core_.UpdateNadaRate(now_ms, true);
    core_.LogUpdate("nada_rtt", ts); 

    } else {

      // [XZ 2018-12-20] Currently, no rate update in between feedback reports
      // TODO: trigger timeout when needed
      printf("NADA UpdateEstimate triggered by local timer: ts = %lld, rate = %6d Kbps\n",
            ts, bitrate_/1000);

      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: triggered by sender local timer -- "
                          << "ts: "   << ts  << " ms "
                          << "rate: " << bitrate_/1000 << " Kbps. " << std::endl;
    }
}

}  // namespace webrtc
