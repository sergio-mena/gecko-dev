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
#include "logging/rtc_event_log/rtc_event_log.h"
#include "system_wrappers/include/field_trial.h"
#include "system_wrappers/include/metrics.h"

#define USE_DELAY_BASED 1

namespace webrtc {

namespace {

/*
 *
 * Default parameter values for the NADA algorithm.
 * See Fig. 3 in https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 * for more details.
 *
*/
const float kNADAParamPrio  = 1.0;   // weight of priority for the flow [PRIO: dimensionless]
const float kNADAParamXref  = 10.0;  // reference congestion level  [XREF: in ms]
const float kNADAParamXDefault = 20.0;  // default congestion level [in ms]
const float kNADAParamKappa = 0.5;   // scaling parameter for gradual rate update [KAPPA: dimensionless]
const float kNADAParamEta  = 2.0;  // scaling parameter for gradual rate update [ETA: dimensionless]
const float kNADAParamTau = 500.;  // Upper bound of RTT for gradual rate update [TAU: in ms]

const int64_t kNADAParamDeltaMs = 100; // Target interval for feedback and/or rate update [DELTA: in ms]
const int64_t kNADAParamMinDeltaMs =  20; // Minimum value of delta for rate calculation [in ms]
const int64_t kNADAParamMaxDeltaMs = 500; // Maximum value of delta for rate calculation [in ms]

const int64_t kNADAParamLogwinMs = 500; // Observation time window for calculating
					// packet summary statistics [LOGWIN: in ms]
const int64_t kNADAParamQepsMs = 10; 	// Threshold for determining queueing delay build-up [QEPS: in ms]
const int64_t kNADAParamQboundMs = 50;  // Upper bound on self-inflicted queuing delay [QBOUND: in ms]
const int64_t kNADAParamDfiltMs = 120;  // Bound on filtering delay [DFILT: in ms]
const float   kNADAParamGammaMax =0.2;  // Upper bound on rate increase ratio for accelerated ramp-up
					// [GAMMA_MAX: dimensionless]
const int kNADAParamRateBps =  800000;  // Default rate: 800Kbps
const int kNADAParamRminBps =  250000;  // Min rate: 250Kbps
const int kNADAParamRmaxBps = 2500000;  // Max rate: 2.5Mbps

const int kNADALimitNumPackets = 20;    // Number of packets before packet loss calculation is
					// considered as valid (outside the scope of NADA draft)

}  // namespace


//
// TO-TRY: pass in min/max rates from external modules (e.g., about:config)
//
NADABandwidthEstimation::NADABandwidthEstimation(RtcEventLog* event_log)
    : lost_packets_since_last_loss_update_Q8_(0),
      expected_packets_since_last_loss_update_(0),
      bitrate_(kNADAParamRateBps),
//      min_bitrate_configured_(congestion_controller::GetMinBitrateBps()),
//      max_bitrate_configured_(congestion_controller::GetMaxBitrateBps()),  // [XZ 2018-12-20] currently not supported
      min_bitrate_configured_(kNADAParamRminBps),
      max_bitrate_configured_(kNADAParamRmaxBps),
      // last_rate_update_ms_(-1),
      last_feedback_ms_(-1),
      // rate_update_interval_ms_(0),
      feedback_interval_ms_(0),
      delta_(kNADAParamDeltaMs),
      nada_x_curr_(kNADAParamXDefault),
      nada_x_prev_(kNADAParamXDefault),
      // last_packet_report_ms_(-1),
      // last_timeout_ms_(-1),
      last_fraction_loss_(0),
      // last_logged_fraction_loss_(0),
      last_round_trip_time_ms_(0),
      min_round_trip_time_ms_(0),
      bwe_incoming_(0),
      delay_based_bitrate_bps_(kNADAParamRateBps),
      // time_last_decrease_ms_(0),
      first_report_time_ms_(-1),
      // initially_lost_packets_(0),
      event_log_(event_log),			// [XZ 2018-12-20] currently not utilized by NADA estimator
      last_rtc_event_log_ms_(-1),		// [XZ 2018-12-20] currently not utilized by NADA estimator
      in_timeout_experiment_(webrtc::field_trial::FindFullName(
                                 "WebRTC-FeedbackTimeout") == "Enabled") {

  RTC_DCHECK(event_log);

  printf("Initializing the NADA BW Estimation Module\n");

  RTC_LOG(LS_INFO) << "Initializing the NADA BW Estimation Module" ;
  RTC_LOG(LS_INFO) << "NADA initial stats: delta = " << delta_
                   << "ms, x_curr = " << nada_x_curr_
                   << "ms, rate = "   << bitrate_/1000
                   << "Kbps, rmin = " << min_bitrate_configured_/1000
                   << "Kbps, rmax = " << max_bitrate_configured_/1000
                   << "Kbps" << std::endl;

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
  min_bitrate_history_.clear();

  RTC_LOG(LS_INFO) << "NADA SetSendBitrate: bitrate_ = " << bitrate_/1000
                   << "Kbps." << std::endl;
}

void NADABandwidthEstimation::SetMinMaxBitrate(int min_bitrate,
                                               int max_bitrate) {
  RTC_DCHECK_GE(min_bitrate, 0);
  min_bitrate_configured_ =
      std::max(min_bitrate, congestion_controller::GetMinBitrateBps());
  if (max_bitrate > 0) {
    max_bitrate_configured_ =
        std::max<uint32_t>(min_bitrate_configured_, max_bitrate);
  } else {
    max_bitrate_configured_ = kNADAParamRmaxBps;
  }


  RTC_LOG(LS_INFO)  << "NADA SetMinMaxBitrate: updating min/max rate:"
                    << " rmin = " << min_bitrate/1000
                    << " => " << min_bitrate_configured_/1000 << " Kbps"
                    << " rmax = " << max_bitrate/1000
                    << " => " << max_bitrate_configured_/1000 << " Kbps"
                    << std::endl;
}

int NADABandwidthEstimation::GetMinBitrate() const {
  return min_bitrate_configured_;
}



// [XZ 2019-03-07 Report query of currently estimated bitrate]
//
void NADABandwidthEstimation::CurrentEstimate(int* bitrate,
                                              uint8_t* loss,
                                              int64_t* rtt) const {


#ifdef USE_DELAY_BASED
  *bitrate = delay_based_bitrate_bps_;
#else
  *bitrate = bitrate_;
#endif

  *loss = last_fraction_loss_;
  *rtt = last_round_trip_time_ms_;

/*
printf("NADA CurrentEstimate: rtt-based: %8.2f, owd-based: %8.2f | rate = %.2f Kbps, loss = %d, rtt = %ld ms\n",
	bitrate_/1000., delay_based_bitrate_bps_/1000.,
	*bitrate/1000.,  *loss, *rtt);

*/
}


// [XZ 2018-12-20]  Currently receiver estimated rate is ignored
// completely by the NADA estimator
void NADABandwidthEstimation::UpdateReceiverEstimate(
    int64_t now_ms, uint32_t bandwidth) {
  bwe_incoming_ = bandwidth;

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


  printf("NADA UpdateReceiverBlock at %lld ms...rtt = %lld, npkts = %d\n",
         now_ms-first_report_time_ms_, rtt, number_of_packets);

  RTC_LOG(LS_INFO) << "NADA UpdateReceiverBlock: now = " << now_ms-first_report_time_ms_
                   << " ms, fb_interval = " << feedback_interval_ms_
                   << " ms, loss = " << int(fraction_loss)
                   << " , rtt = " << rtt
                   << " ms, npackets = " << number_of_packets << std::endl;

  if (first_report_time_ms_ == -1)
  {
    first_report_time_ms_ = now_ms;
    min_round_trip_time_ms_ = rtt;
  }

  // Update RTT and base-RTT
  last_round_trip_time_ms_ = rtt;
  if (rtt < min_round_trip_time_ms_) min_round_trip_time_ms_ = rtt;

  // TODO: switch how congestion signal (x_curr) is calculated
  // based on a macro-defined mode:
  // -- absolute RTT
  // -- relative RTT
  // -- absolute OWD
  // -- relative OWD (i.e., queuing delay)
  // -- compound (warped OWD + loss)
  // ...
  //
  // Update x_curr_ as RTT value
  nada_x_curr_ = rtt;
//  nada_x_curr_ = rtt-min_round_trip_time_ms_+1;


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

    // has_decreased_since_last_fraction_loss_ = false;
    last_fraction_loss_ = lost_packets_since_last_loss_update_Q8_ /
                          expected_packets_since_last_loss_update_;

    // Reset accumulators.
    lost_packets_since_last_loss_update_Q8_ = 0;
    expected_packets_since_last_loss_update_ = 0;
    // last_packet_report_ms_ = now_ms;

    // call rate update calculation
    UpdateEstimate(now_ms);
  }
}


/*
 *
 * Xiaoqing 2018-07-02
 *
 * Implementation of core NADA congestion control algorithm
 *
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 *
 */

/*
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 *
 * The criteria for operating in accelerated ramp-up mode are:
 *
 * o  No recent packet losses within the observation window LOGWIN; and
 *
 * o  No build-up of queuing delay: d_fwd-d_base < QEPS for all previous
 *    delay samples within the observation window LOGWIN.
 *
 *
 */
int NADABandwidthEstimation::getRampUpMode() {

    uint32_t rate_min = min_bitrate_history_.front().second;
    int64_t rtt_max = max_rtt_history_.front().second;
    uint8_t plr_max = max_plr_history_.front().second;


    RTC_LOG(LS_VERBOSE) << "NADA getRampUpMode: rmin = " << rate_min/1000
                        << " Kbps, rtt_min = " << min_round_trip_time_ms_
                        << " ms, rtt_max = "   << rtt_max
                        << " ms, plr_max = "   << int(plr_max) << std::endl;

    if (plr_max > 0.)	  return 1;  // loss exists, gradual-update

    if (rtt_max - min_round_trip_time_ms_ > kNADAParamQepsMs) return 1;

    return 0;
}

/*
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 *
 * In accelerated ramp-up mode, the rate r_ref is updated as follows:
 *
 *                             QBOUND
 * gamma = min(GAMMA_MAX, ------------------)     (3)
 *                        rtt+DELTA+DFILT
 *
 *
 * r_ref = max(r_ref, (1+gamma) r_recv)           (4)
 *
 */
void NADABandwidthEstimation::AcceleratedRampUp(const int64_t now_ms) {

	float rtt = float(last_round_trip_time_ms_);

	float gamma = kNADAParamQboundMs /(rtt + kNADAParamDeltaMs + kNADAParamDfiltMs);
	if (gamma > kNADAParamGammaMax) gamma = kNADAParamGammaMax;

	// float rnew = (1+gamma)* bwe_incoming_;
	// if (rnew > bitrate_)	bitrate_ = rnew;

	bitrate_ = (1+gamma)*bitrate_;
}

/*
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 *
 *
 * In gradual update mode, the rate r_ref is updated as:
 *
 *
 * x_offset = x_curr - PRIO*XREF*RMAX/r_ref          (5)
 *
 * x_diff   = x_curr - x_prev                        (6)
 *
 *                       delta    x_offset
 * r_ref = r_ref - KAPPA*-------*------------*r_ref
 *                        TAU       TAU
 *
 *                             x_diff
 *               - KAPPA*ETA*---------*r_ref         (7)
 *                             TAU
 *
 */
void NADABandwidthEstimation::GradualRateUpdate(const int64_t now_ms) {

    double x_ratio = float(max_bitrate_configured_)/float(bitrate_);
    double x_target = kNADAParamPrio * kNADAParamXref * x_ratio;
    double x_offset = nada_x_curr_ - x_target;
    double x_diff = nada_x_curr_ - nada_x_prev_;

    // cache current observation as x_prev
    nada_x_prev_ = nada_x_curr_;

    // calculate updated rate per equations above
    double w1 = float(delta_)/kNADAParamTau;
    w1 = w1*x_offset/kNADAParamTau;

    double w2 = kNADAParamEta*x_diff/kNADAParamTau;

    bitrate_ = bitrate_*(1-kNADAParamKappa*(w1+w2));

//	bitrate_ = 800000;  // for debugging
}


void NADABandwidthEstimation::UpdateEstimate(int64_t now_ms) {


//    printf("NADA invoking UpdateEstimate at %d ms...\n", now_ms-first_report_time_ms_);

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

    // triggered by feedback update
    delta_ = feedback_interval_ms_;
    if (delta_ < kNADAParamMinDeltaMs) delta_ = kNADAParamMinDeltaMs;
    if (delta_ > kNADAParamMaxDeltaMs) delta_ = kNADAParamMaxDeltaMs;

/*
 * TODO: re-visit the logic of how feedback intervals are logged/updated
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
    UpdateMinHistory(now_ms);
    UpdateRttHistory(now_ms);
    UpdatePlrHistory(now_ms);

    // switch between Accelerated-Ramp-Up mode and
    // Gradual-Update mode based on loss/delay observations
    int rmode = getRampUpMode();

    if (rmode == 0)
      AcceleratedRampUp(now_ms);
    else
      GradualRateUpdate(now_ms);

    ClipBitrate();

    RTC_LOG(LS_INFO) << "NADA UpdateEstimate | "
                     << " ts: " << now_ms-first_report_time_ms_
                     << " fbint: " << feedback_interval_ms_
                     << " delta: " << delta_
                     << " rmode: " << rmode
                     << " xcurr: " << nada_x_curr_
                     << " rtt_min: " <<  min_round_trip_time_ms_
                     << " srate: " << bitrate_/1000
                     << " rmin: "  << min_bitrate_configured_/1000
                     << " rmax: "  << max_bitrate_configured_/1000
                     << std::endl;

     printf("NADA UpdateEstimate triggered by FB: ts = %lld, fbint = %lld ms, rmode = %d, xcurr = %4.2f, rate = %6d Kbps\n",
            now_ms-first_report_time_ms_, feedback_interval_ms_, rmode, nada_x_curr_, bitrate_/1000);

    } else {

      // [XZ 2018-12-20] Currently, no rate update in between feedback reports

      // TODO: trigger timeout when needed
      RTC_LOG(LS_VERBOSE) << "NADA UpdateEstimate: triggered by sender local timer -- "
                          << "ts: "   << now_ms-first_report_time_ms_  << " ms "
                          << "rate: " << bitrate_/1000 << " Kbps. " << std::endl;
    }
}

/*
 *
 * Update history records for Rmin, RTT, PLR
 *
 */
void NADABandwidthEstimation::UpdateMinHistory(int64_t now_ms) {

  // Remove old data points from history.
  // Since history precision is in ms, add one so it is able to increase
  // bitrate if it is off by as little as 0.5ms.
  while (!min_bitrate_history_.empty() &&
         now_ms - min_bitrate_history_.front().first + 1 > kNADAParamLogwinMs) {

    min_bitrate_history_.pop_front();
  }

  // Typical minimum sliding-window algorithm: Pop values higher than current
  // bitrate before pushing it.
  while (!min_bitrate_history_.empty() &&
         bitrate_ <= min_bitrate_history_.back().second) {
    min_bitrate_history_.pop_back();
  }

  min_bitrate_history_.push_back(std::make_pair(now_ms, bitrate_));
}

void NADABandwidthEstimation::UpdateRttHistory(int64_t now_ms) {

  // Remove expired data points from history.
  while (!max_rtt_history_.empty() &&
         now_ms - max_rtt_history_.front().first > kNADAParamLogwinMs) {

      max_rtt_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current RTT before pushing the current RTT.
  while (!max_rtt_history_.empty() &&
         last_round_trip_time_ms_ >= max_rtt_history_.back().second) {
    max_rtt_history_.pop_back();
  }

  max_rtt_history_.push_back(std::make_pair(now_ms, last_round_trip_time_ms_));
}

void NADABandwidthEstimation::UpdatePlrHistory(int64_t now_ms) {

  // Remove expired data points from history.
  while (!max_plr_history_.empty() &&
         now_ms - max_plr_history_.front().first > kNADAParamLogwinMs) {

    max_plr_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current PLR before pushing the current PLR
  while (!max_plr_history_.empty() &&
         last_fraction_loss_ >= max_plr_history_.back().second) {
    max_plr_history_.pop_back();
  }

  max_plr_history_.push_back(std::make_pair(now_ms, last_fraction_loss_));
}


void NADABandwidthEstimation::ClipBitrate() {

  if (bitrate_ < min_bitrate_configured_)
    bitrate_ = min_bitrate_configured_;

  if (bitrate_ > max_bitrate_configured_) {
    bitrate_ = max_bitrate_configured_;
  }
}

/*
 * [XZ 2018-12-20]  Inherited from the SenderSideBandwidthEstimation
 * module.  We may need it later ...
 *
uint32_t NADABandwidthEstimation::CapBitrateToThresholds(
    int64_t now_ms, uint32_t bitrate) {
  if (bwe_incoming_ > 0 && bitrate > bwe_incoming_) {
    bitrate = bwe_incoming_;
  }
  if (delay_based_bitrate_bps_ > 0 && bitrate > delay_based_bitrate_bps_) {
    bitrate = delay_based_bitrate_bps_;
  }
  if (bitrate > max_bitrate_configured_) {
    bitrate = max_bitrate_configured_;
  }
  if (bitrate < min_bitrate_configured_) {
    if (last_low_bitrate_log_ms_ == -1 ||
        now_ms - last_low_bitrate_log_ms_ > kLowBitrateLogPeriodMs2) {
      LOG(LS_WARNING) << "Estimated available bandwidth " << bitrate / 1000
                      << " kbps is below configured min bitrate "
                      << min_bitrate_configured_ / 1000 << " kbps.";
      last_low_bitrate_log_ms_ = now_ms;
    }
    bitrate = min_bitrate_configured_;
  }
  return bitrate;
}  */

}  // namespace webrtc
