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
constexpr int kNADALimitNumPackets = 20;    // Number of packets before packet loss calculation is
                                        // considered as valid (outside the scope of NADA draft)

}  // namespace

NADABandwidthEstimation::NADABandwidthEstimation()
    : SendSideBandwidthEstimationInterface(),
      lost_packets_since_last_loss_update_Q8_(0),
      expected_packets_since_last_loss_update_(0),
      bitrate_(kNADAParamRateBps),
      last_feedback_ms_(-1),
      feedback_interval_ms_(0),
      last_fraction_loss_(0),
      last_round_trip_time_ms_(0),
      min_round_trip_time_ms_(-1),
      relative_rtt_(0),
      bwe_incoming_(0),
      delay_based_bitrate_bps_(kNADAParamRateBps),
      first_report_time_ms_(-1),
      core_() {

  printf("Initializing the RTT-based NADA BW Estimation Module\n");

  RTC_LOG(LS_INFO) << "Initializing the RTT-based NADA BW Estimation Module" ;
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

  // Clear last sent bitrate history so the new value can be used directly
  // and not capped.
  // min_bitrate_history_.clear();
  core_.ClearRminHistory(); 

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

#ifdef USE_DELAY_BASED
  *bitrate = delay_based_bitrate_bps_;
#else
  *bitrate = bitrate_;
#endif

  *loss = last_fraction_loss_;
  *rtt = last_round_trip_time_ms_;

printf("NADA CurrentEstimate: rtt-based: %8.2f, owd-based: %8.2f | rate = %.2f Kbps, loss = %d, rtt = %lld ms\n",
       bitrate_/1000., delay_based_bitrate_bps_/1000.,
       *bitrate/1000.,  *loss, *rtt);

  RTC_LOG(LS_INFO) << "NADA CurrentEstimate: " 
                   << " | delay_based_rate:  " << delay_based_bitrate_bps_/1000 << " Kbps"
                   << " | sender_estimated_rate: " << bitrate_/1000. << " Kbps"
                   << " | reported rate: " << *bitrate/1000. << " Kbps"
                   << " | loss: " << *loss*100 << " %"
                   << " | rtt: "  << *rtt << " ms"  << std::endl;
}


// [XZ 2018-12-20]  Currently receiver estimated rate is unused
// in the NADA rate calculation; passed along to NadaCore for stats reporting
// purpose only 
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
    min_round_trip_time_ms_ = rtt;
  }

  // Update RTT and base-RTT
  last_round_trip_time_ms_ = rtt;
  if (rtt < min_round_trip_time_ms_) min_round_trip_time_ms_ = rtt;
  relative_rtt_ = rtt-min_round_trip_time_ms_; 

  core_.UpdateCongestion(relative_rtt_); 


  printf("NADA UpdateReceiverBlock at %lld ms...rtt = %lld, npkts = %d\n",
         now_ms-first_report_time_ms_, rtt, number_of_packets);

  RTC_LOG(LS_INFO) << "NADA UpdateReceiverBlock: now = " << now_ms-first_report_time_ms_
                   << " ms, fb_interval = " << feedback_interval_ms_
                   << " ms, loss = " << int(fraction_loss)
                   << " , rtt = " << rtt
                   << " ms, rttmin = " << min_round_trip_time_ms_
                   << " ms, npackets = " << number_of_packets << std::endl;

  // Check sequence number diff and weight loss report
  if (number_of_packets > 0) {
    // Calculate number of lost packets.
    const int num_lost_packets_Q8 = fraction_loss * number_of_packets;
    // Accumulate reports.
    lost_packets_since_last_loss_update_Q8_ += num_lost_packets_Q8;
    expected_packets_since_last_loss_update_ += number_of_packets;

    // Don't generate a loss rate or update rate until it can be based on enough packets.
    if (expected_packets_since_last_loss_update_ < kNADALimitNumPackets)
      return;

    last_fraction_loss_ = lost_packets_since_last_loss_update_Q8_ /
                          expected_packets_since_last_loss_update_;

    // Reset accumulators.
    lost_packets_since_last_loss_update_Q8_ = 0;
    expected_packets_since_last_loss_update_ = 0;

    // call rate update calculation
    UpdateEstimate(now_ms);
  }
}

// call calculations in NadaCore for reference rate update
void NADABandwidthEstimation::UpdateEstimate(int64_t now_ms) {

    if (last_feedback_ms_ == -1) {
      // no feedback message yet: staying with current rate
      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: no feedback yet -- "
                   << "ts: "	<< now_ms-first_report_time_ms_  << " ms "
                   << "rate: " << bitrate_/1000 << " Kbps." << std::endl;
    }  else if (now_ms == last_feedback_ms_) {
      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: triggered by feedback -- "
                          << "ts: " << now_ms-first_report_time_ms_  << " ms "
                          << "rate: " << bitrate_/1000 << " Kbps "
                          << "feedback interval: " << feedback_interval_ms_ << " ms. " << std::endl;

    // // triggered by feedback update
    core_.UpdateDelta(feedback_interval_ms_); 

    /*
     * TODO (Xiaoqing: worth it?): re-visit the logic of how feedback intervals are logged/updated
     *
        // update feedback interval
        if (last_rate_update_ms_ == -1) {
    	// first rate update
            last_rate_update_ms_ = now_ms;
    	rate_update_interval_ms_ = 0;
            delta_ = kNADAParamDeltaMs;
        } else {
    	rate_update_interval_ms_ = now_ms - last_rate_update_ms_;
            last_rate_update_ms_ = now_ms;
    	if ((rate_update_interval_ms_ > kNADAParamMinDeltaMs)
    	    && (rate_update_interval_ms_ < kNADAParamMaxDeltaMs) ) {
    	    delta_ = rate_update_interval_ms_;
    	}
        }
    */

    // update history logs of R_min | RTT | PlR
    core_.UpdateRminHistory(now_ms, bitrate_);
    core_.UpdateRttHistory(now_ms, last_round_trip_time_ms_);
    core_.UpdatePlrHistory(now_ms, float(last_fraction_loss_));

    // switch between Accelerated-Ramp-Up mode and
    // Gradual-Update mode based on loss/delay observations
    int rmode = core_.GetRampUpMode(min_round_trip_time_ms_);

    if (rmode == 0)
      // AcceleratedRampUp(now_ms);
      bitrate_ = core_.AcceleratedRampUp(now_ms, 
                                         last_round_trip_time_ms_, 
                                         bitrate_);
    else
      bitrate_ = core_.GradualRateUpdate(now_ms, bitrate_);

    bitrate_ = core_.ClipBitrate(bitrate_);

    float xcurr = core_.GetCongestion(); 
    int64_t ts = now_ms-first_report_time_ms_; 
    printf("NADA UpdateEstimate triggered by FB: ts = %lld, fbint = %lld ms, rmode = %d, xcurr = %4.2f, rate = %6d Kbps\n",
            ts, feedback_interval_ms_, rmode, xcurr, bitrate_/1000);

    // core_.TestFunction("NADA rtt");
    core_.LogUpdate("nada_rtt", ts); 

    std::ostringstream os;
    os << std::fixed;
    os.precision(2);
    RTC_LOG(LS_INFO) << " NADA UpdateEstimate | algo:nada_rtt "                     // 1) CC algorithm flavor
                     << " | ts: "     << now_ms-first_report_time_ms_ << " ms"      // 2) timestamp
                     << " | fbint: "  << feedback_interval_ms_ << " ms"             // 3) feedback interval
                     << " | relrtt: " << relative_rtt_ << " ms"                      // 4) relative RTT
                     << " | rtt: "    << last_round_trip_time_ms_ << " ms"          // 5) RTT
                     << " | ploss: "  << lost_packets_since_last_loss_update_Q8_    // 6) packet loss count
                     << " | plr: "     << std::fixed << float(last_fraction_loss_)*100.  << " %"  // 7) temporally smoothed packet loss ratio
                     << " | rmode: "   << rmode                                      // 8) rate update mode: accelerated ramp-up or gradual 
                     << " | xcurr: "   << std::fixed << xcurr << " ms"       // 9) aggregated congestion signal
                     << " | rrate: "  << bwe_incoming_/1000. << " Kbps"             // 10) receiving rate 
                     << " | srate: "  << bitrate_/1000. << " Kbps"                  // 11) sending rate
                     // << " | rmin: "    << min_bitrate_configured_/1000. << " Kbps"  // 12) minimum rate
                     // << " | rmax: "    << max_bitrate_configured_/1000. << " Kbps"  // 13) maximum rate
                     << std::endl;

    } else {

      // [XZ 2018-12-20] Currently, no rate update in between feedback reports
     printf("NADA UpdateEstimate triggered by local timer: ts = %lld, rate = %6d Kbps\n",
            now_ms-first_report_time_ms_, bitrate_/1000);

      // TODO: trigger timeout when needed
      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: triggered by sender local timer -- "
                          << "ts: "   << now_ms-first_report_time_ms_  << " ms "
                          << "rate: " << bitrate_/1000 << " Kbps. " << std::endl;
    }
}

}  // namespace webrtc
