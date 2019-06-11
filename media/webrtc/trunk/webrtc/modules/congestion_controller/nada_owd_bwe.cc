/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/nada_owd_bwe.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "rtc_base/checks.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/logging.h"
#include "rtc_base/thread_annotations.h"
//#include "webrtc/modules/congestion_controller/include/congestion_controller.h"

#include "system_wrappers/include/field_trial.h"
#include "system_wrappers/include/metrics.h"
#include "typedefs.h"

namespace {

/*
 *
 * Default parameter values for the NADA algorithm.
 * See Fig. 3 in https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
 * for more details.
 *
*/
constexpr float kNadaBweParamPrio  = 1.0;   // weight of priority for the flow [PRIO: dimensionless]
constexpr float kNadaBweParamXref  = 10.0;  // reference congestion level  [XREF: in ms]
constexpr float kNadaBweParamXDefault = 20.0;  // default congestion level [in ms]
constexpr float kNadaBweParamKappa = 0.5;   // scaling parameter for gradual rate update [KAPPA: dimensionless]
constexpr float kNadaBweParamEta  = 2.0;  // scaling parameter for gradual rate update [ETA: dimensionless]
constexpr float kNadaBweParamTau = 500.;  // Upper bound of RTT for gradual rate update [TAU: in ms]

constexpr uint32_t kNadaBweDefaultBitrate = 600000;  // 600 Kbps
constexpr uint32_t kNadaBweDefaultMinBitrate =  300000;  //  200 Kbps
constexpr uint32_t kNadaBweDefaultMaxBitrate = 3000000;  // 2000 Kbps

constexpr int64_t kNadaBweParamDeltaMs = 100; // Target interval for feedback and/or rate update [DELTA: in ms]
constexpr int64_t kNadaBweParamMinDeltaMs =  20; // Minimum value of delta for rate calculation [in ms]
constexpr int64_t kNadaBweParamMaxDeltaMs = 200; // Maximum value of delta for rate calculation [in ms]
constexpr int64_t kNadaBweParamLogwinMs  = 500; 	 // Observation time window for calculating
constexpr int64_t kNadaBweParamLogwinMs2 = 3000; 	 // Observation time window for long-term minimal baseline forward delay value [in ms]
constexpr int64_t  kNadaBweDefaultRttMs = 50.;   	// in ms
constexpr int64_t  kNadaBweParamQepsMs = 10;   	// Threshold for determining queueing delay build-up [QEPS: in ms]
constexpr int64_t  kNadaBweParamQboundMs = 50;  // Upper bound on self-inflicted queuing delay [QBOUND: in ms]
constexpr int64_t  kNadaBweParamDfiltMs = 120;  // Bound on filtering delay [DFILT: in ms]
constexpr float    kNadaBweParamGammaMax =0.2;  // Upper bound on rate increase ratio for accelerated ramp-up
                                                // [GAMMA_MAX: dimensionless]

constexpr int64_t kDefaultProbingIntervalinMs = 100;  // in ms
constexpr int64_t kStreamTimeOutMs = 2000;   	// in ms

// This ssrc is used to fulfill the current API but will be removed
// after the API has been changed.
constexpr uint32_t kNadaOwdFixedSsrc = 0;

// for Bitrate estimation in BitrateEstimator subclass
constexpr int kInitialRateEstmWindowMs = 500;
constexpr int kRateEstmWindowMs = 150;

}  // namespace

namespace webrtc {

///// start of BitrateEstimator subclass /////
NadaOwdBwe::BitrateEstimator::BitrateEstimator()
    : sum_(0),
      current_win_ms_(0),
      prev_time_ms_(-1),
      bitrate_estimate_(-1.0f),
      bitrate_estimate_var_(50.0f),
      old_estimator_(kBitrateWindowMs, 8000) {}

void NadaOwdBwe::BitrateEstimator::Update(int64_t now_ms, int bytes) {

  old_estimator_.Update(bytes, now_ms);
  rtc::Optional<uint32_t> rate = old_estimator_.Rate(now_ms);
  bitrate_estimate_ = -1.0f;
  if (rate)
    bitrate_estimate_ = *rate / 1000.0f;
  return;

  int rate_window_ms = kRateEstmWindowMs;

  // We use a larger window at the beginning to get a more stable sample that
  // we can use to initialize the estimate.
  if (bitrate_estimate_ < 0.f)
    rate_window_ms = kInitialRateEstmWindowMs;
  float bitrate_sample = UpdateWindow(now_ms, bytes, rate_window_ms);
  if (bitrate_sample < 0.0f)
    return;
  if (bitrate_estimate_ < 0.0f) {
    // This is the very first sample we get. Use it to initialize the estimate.
    bitrate_estimate_ = bitrate_sample;
    return;
  }
  // Define the sample uncertainty as a function of how far away it is from the
  // current estimate.
  float sample_uncertainty =
      10.0f * std::abs(bitrate_estimate_ - bitrate_sample) / bitrate_estimate_;
  float sample_var = sample_uncertainty * sample_uncertainty;
  // Update a bayesian estimate of the rate, weighting it lower if the sample
  // uncertainty is large.
  // The bitrate estimate uncertainty is increased with each update to model
  // that the bitrate changes over time.
  float pred_bitrate_estimate_var = bitrate_estimate_var_ + 5.f;
  bitrate_estimate_ = (sample_var * bitrate_estimate_ +
                       pred_bitrate_estimate_var * bitrate_sample) /
                      (sample_var + pred_bitrate_estimate_var);
  bitrate_estimate_var_ = sample_var * pred_bitrate_estimate_var /
                          (sample_var + pred_bitrate_estimate_var);
}

float NadaOwdBwe::BitrateEstimator::UpdateWindow(int64_t now_ms,
                                                    int bytes,
                                                    int rate_window_ms) {
  // Reset if time moves backwards.
  if (now_ms < prev_time_ms_) {
    prev_time_ms_ = -1;
    sum_ = 0;
    current_win_ms_ = 0;
  }
  if (prev_time_ms_ >= 0) {
    current_win_ms_ += now_ms - prev_time_ms_;
    // Reset if nothing has been received for more than a full window.
    if (now_ms - prev_time_ms_ > rate_window_ms) {
      sum_ = 0;
      current_win_ms_ %= rate_window_ms;
    }
  }
  prev_time_ms_ = now_ms;
  float bitrate_sample = -1.0f;
  if (current_win_ms_ >= rate_window_ms) {
    bitrate_sample = 8.0f * sum_ / static_cast<float>(rate_window_ms);
    current_win_ms_ -= rate_window_ms;
    sum_ = 0;
  }
  sum_ += bytes;
  return bitrate_sample;
}

// rtc::Optional<uint32_t> NadaOwdBwe::BitrateEstimator::bitrate_bps() const {
uint32_t NadaOwdBwe::BitrateEstimator::bitrate_bps() const {

  if (bitrate_estimate_ < 0.f)
    // return rtc::Optional<uint32_t>();
    return 0;

  return (uint32_t)(bitrate_estimate_ * 1000);

  //return rtc::Optional<uint32_t>(bitrate_estimate_ * 1000);
}

///// end of BitrateEstimator subclass /////



///  start of NadaOwdBwe (NADA One-way-delay BW Estimation) ///

NadaOwdBwe::NadaOwdBwe(const Clock* clock)
    : clock_(clock),
// in_trendline_experiment_(TrendlineFilterExperimentIsEnabled()),
//      in_median_slope_experiment_(MedianSlopeFilterExperimentIsEnabled()),
//      inter_arrival_(),
//      kalman_estimator_(),
//      trendline_estimator_(),
//      detector_(),
      receiver_incoming_bitrate_(),
      last_update_ms_(-1),
      first_update_ms_(-1),
      last_seen_packet_ms_(-1),
//      uma_recorded_(false),
//      probe_bitrate_estimator_(event_log),
//      trendline_window_size_(kDefaultTrendlineWindowSize),
//      trendline_smoothing_coeff_(kDefaultTrendlineSmoothingCoeff),
//      trendline_threshold_gain_(kDefaultTrendlineThresholdGain),
      nada_rate_in_bps_(kNadaBweDefaultBitrate),
      nada_rmin_in_bps_(kNadaBweDefaultMinBitrate),
      nada_rmax_in_bps_(kNadaBweDefaultMaxBitrate),
      nada_recv_in_bps_(kNadaBweDefaultBitrate),
      nada_rtt_in_ms_(kNadaBweDefaultRttMs),
      nada_rtt_base_in_ms_(-1.),
      nada_rtt_rel_in_ms_(0.),
      nada_rtt_avg_in_ms_(0.),
      nada_x_curr_(kNadaBweParamXDefault),
      nada_x_prev_(kNadaBweParamXDefault),
      nada_delta_(kNadaBweParamDeltaMs),
      nada_d_fwd_(-1.),
      nada_d_base_(-1.),
      nada_d_queue_(-1.),
      nada_plr_(0.),
      probing_interval_in_ms_(kDefaultProbingIntervalinMs) {
}

NadaOwdBwe::~NadaOwdBwe() {}

// Main API for BW estimation, triggered by receiving transport FB vectors
DelayBasedBwe::Result NadaOwdBwe::IncomingPacketFeedbackVector(
    const std::vector<PacketFeedback>& packet_feedback_vector,
    rtc::Optional<uint32_t> acked_bitrate_bps) {
  RTC_DCHECK(std::is_sorted(packet_feedback_vector.begin(),
                            packet_feedback_vector.end(),
                            PacketFeedbackComparator()));

  // [XZ 2019-03-07  add time stamp info]
  int64_t now_ms = clock_->TimeInMilliseconds();
  int nfb = int(packet_feedback_vector.size());
  if (last_update_ms_ > 0)
    nada_delta_ = now_ms - last_update_ms_;
  if (first_update_ms_ < 0)
    first_update_ms_ = now_ms;

  printf("NadaOwdBwe::IncomingPacketFeedbackVector | t=%lld (%lld) ms, nfb = %d, fbint = %6.2f ms\n",
         now_ms, now_ms - first_update_ms_,
         nfb, nada_delta_);

  // [XZ 2019-03-07]

  RTC_DCHECK_RUNS_SERIALIZED(&network_race_);


  // TODO: replace loop below with NADA calculation:
  // Step 1)  FB Vector => per-pkt <seqno, owd> info *
  //
  // Step 2)  Rate calculation update based on OWD/loss/RTT/recv_rate **
  // 		2.a: accelerated ramp-up (probing)
  // 		2.b: gradual update
  //
  // Step 3)  Save to result struct

  DelayBasedBwe::Result result;
  double dtmp = 0.;
  double dmin = -1.;
  int ipkt = 0;
  int nwin = 5;  // TODO: should make this dynamically tunable but how? [XZ-2019-03-08]
  double rtt = 0.;
  double dq  = 0.;
  for (const auto& packet_feedback : packet_feedback_vector) {

    receiver_incoming_bitrate_.Update(packet_feedback.arrival_time_ms, packet_feedback.payload_size);
    nada_recv_in_bps_ = receiver_incoming_bitrate_.bitrate_bps();

    dtmp = packet_feedback.arrival_time_ms - packet_feedback.send_time_ms;
    rtt = now_ms - packet_feedback.send_time_ms;

    // update d_base and rtt_base
    // if (nada_d_base_ < 0 || nada_d_base_ > dtmp) nada_d_base_ = dtmp;
    UpdateDminHistory(now_ms, dtmp);
    nada_d_base_ = dmin_history_.front().second;

    if (nada_rtt_base_in_ms_ < 0 || nada_rtt_base_in_ms_ > rtt) nada_rtt_base_in_ms_ = rtt;

    // calculate relative RTT and OWD accordingly
    nada_rtt_in_ms_ = rtt;
    nada_rtt_rel_in_ms_ = nada_rtt_in_ms_ - nada_rtt_base_in_ms_;

    dq = dtmp - nada_d_base_;

    printf("\t pktinfo | seqno=%6d, pktsize=%6d | creatts=%8lld, sendts=%8lld, recvts=%8lld, ackts=%8lld | d_fwd=%6.1f, dbase=%6.1f, dqueue=%6.1f ms | rtt=%6.1f, rtt_base=%6.1f, rtt_rel=%6.1fms\n",
           packet_feedback.sequence_number,
           int(packet_feedback.payload_size),
           packet_feedback.creation_time_ms,
           packet_feedback.send_time_ms,
           packet_feedback.arrival_time_ms,
           now_ms,
           dtmp, nada_d_base_, dq, rtt,
           nada_rtt_base_in_ms_,
           nada_rtt_rel_in_ms_);

    // minimum-filtering of past [15] FB reports
    if (ipkt > nfb - nwin) {
        if (dmin < 0  || dmin > dtmp) dmin = dtmp;
    }
    ipkt ++;
  }

  // update delay measurements
  nada_d_fwd_ = dmin;
  nada_d_queue_ = nada_d_fwd_ - nada_d_base_;
  nada_x_curr_ = nada_d_queue_;

  printf("\t pktstats | delta=%6.2f d_fwd=%6.2f, d_base=%6.2f, d_queue=%6.2f ms, rtt=%6.2f, rtt_b=%6.2f, rtt_rel=%6.2f, x_curr=%6.2f\n",
         nada_delta_, nada_d_fwd_, nada_d_base_, nada_d_queue_,
         nada_rtt_in_ms_, nada_rtt_base_in_ms_, nada_rtt_rel_in_ms_,
         nada_x_curr_);

  // switch between Accelerated-Ramp-Up mode and
  // Gradual-Update mode based on loss/delay observations
   UpdateDelHistory(now_ms);
   UpdatePlrHistory(now_ms);

  int rmode = GetRampUpMode();

  if (rmode == 0)
      AcceleratedRampUp(now_ms);
  else
      GradualRateUpdate(now_ms);

  // clip the updated rate between [rmin, rmax]
  ClipBitrate();

  printf("NadaOwdBwe: t = %lld ms mode = %d | r_curr = %6.2f => [%6.2f, %6.2f] Kbps | rtt_rel = %6.2f ms, x_curr = %6.2f ms | r_recv = %6.2f Kbps\n",
         now_ms - first_update_ms_,
         rmode,
         nada_rate_in_bps_/1000.,
         nada_rmin_in_bps_/1000.,
         nada_rmax_in_bps_/1000.,
         nada_rtt_rel_in_ms_,
         nada_x_curr_,
         nada_recv_in_bps_/1000.);

  last_seen_packet_ms_ = now_ms;
  last_update_ms_ = now_ms;

  result.target_bitrate_bps = nada_rate_in_bps_;
  result.updated = true;
  return result;
}

///////////////// Core Rate Update Calculations /////////////////
/*
 *
 * Xiaoqing 2019-03-08
 *
 * Implementation of core NADA congestion control algorithm
 *
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
 *
 */

/*
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
 *
 * The criteria for operating in accelerated ramp-up mode are:
 *
 * o  No recent packet losses within the observation window LOGWIN; and
 *
 * o  No build-up of queuing delay: d_fwd-d_base < QEPS for all previous
 *    delay samples within the observation window LOGWIN.
 *
 */

int NadaOwdBwe::GetRampUpMode() {

//    uint32_t rate_min = min_bitrate_history_.front().second;
    int64_t del_max = max_del_history_.front().second;
    uint8_t plr_max = max_plr_history_.front().second;

//    LOG(LS_VERBOSE) << "NADA getRampUpMode: rmin = " << rate_min/1000
//                  << " Kbps, rtt_min = " << min_round_trip_time_ms_
//                  << " ms, rtt_max = "   << rtt_max
//                  << " ms, plr_max = "   << int(plr_max) << std::endl;

    if (plr_max > 0.)     return 1;  // loss exists, gradual-update

    if (del_max - nada_d_base_ > kNadaBweParamQepsMs) return 1;

    return 0;
}

/*
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
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
void NadaOwdBwe::AcceleratedRampUp(const int64_t now_ms) {

        // float rtt = float(nada_rtt_in_ms_);

	// calculate multiplicative ramp-up ratio
        float gamma = kNadaBweParamQboundMs /(nada_rtt_in_ms_ + kNadaBweParamDeltaMs + kNadaBweParamDfiltMs);
        if (gamma > kNadaBweParamGammaMax) gamma = kNadaBweParamGammaMax;

	printf("\t NadaOwdBwe::AcceleratedRampUp:  ramp-up ratio gamma = %4.2f, r_curr = %6.2f Kbps\n", gamma, nada_rate_in_bps_/1000.);

        // float rnew = (1+gamma)* bwe_incoming_;
        // if (rnew > bitrate_) bitrate_ = rnew;

        nada_rate_in_bps_ = (1+gamma)*nada_rate_in_bps_;
}



/*
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
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
void NadaOwdBwe::GradualRateUpdate(const int64_t now_ms) {

    double x_ratio = float(nada_rmax_in_bps_)/float(nada_rate_in_bps_);
    double x_target = kNadaBweParamPrio * kNadaBweParamXref * x_ratio;
    double x_offset = nada_x_curr_ - x_target;
    double x_diff = nada_x_curr_ - nada_x_prev_;


    // calculate updated rate per equations above
    double delta0 = nada_delta_;
    if (delta0 > kNadaBweParamMaxDeltaMs)  delta0 = kNadaBweParamMaxDeltaMs;

    double w1 = delta0/kNadaBweParamTau;
    w1 = w1*x_offset/kNadaBweParamTau;

    double w2 = kNadaBweParamEta*x_diff/kNadaBweParamTau;

    // avoid numerical "overflow" with uint type
    if (kNadaBweParamKappa*(w1+w2) < 1.0)
        nada_rate_in_bps_ = nada_rate_in_bps_*(1-kNadaBweParamKappa*(w1+w2));
    else
	nada_rate_in_bps_ = 0;

    printf("\t NadaOwdBwe::GradualRateUpdate:  x_curr = %6.2f ms | r_curr = %6.2f Kbps\n", nada_x_curr_, nada_rate_in_bps_/1000.);

    // cache current observation as x_prev
    nada_x_prev_ = nada_x_curr_;
}


//////////////////// Auxilliary Functions ///////////////

void NadaOwdBwe::ClipBitrate() {

    if (nada_rate_in_bps_ < nada_rmin_in_bps_)
        nada_rate_in_bps_ = nada_rmin_in_bps_;

    if (nada_rate_in_bps_ > nada_rmax_in_bps_) {
        nada_rate_in_bps_ = nada_rmax_in_bps_;
  }
}

// maintain long-term of recent pkt delay (dfwd) records
void NadaOwdBwe::UpdateDminHistory(int64_t now_ms, float dtmp) {

  // Remove expired data points from history.
  while (!dmin_history_.empty() &&
         now_ms - dmin_history_.front().first > kNadaBweParamLogwinMs2) {
        dmin_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging minimum values:
  // Pop values highter than current delay value before pushing the current delay value.
  while (!dmin_history_.empty() &&
         dtmp < dmin_history_.back().second) {
    dmin_history_.pop_back();
  }

  dmin_history_.push_back(std::make_pair(now_ms, dtmp));
}


// maintain history of recent pkt delay (dfwd) records
void NadaOwdBwe::UpdateDelHistory(int64_t now_ms) {

  // Remove expired data points from history.
  while (!max_del_history_.empty() &&
         now_ms - max_del_history_.front().first > kNadaBweParamLogwinMs) {
        max_del_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current delay value before pushing the current delay value.
  while (!max_del_history_.empty() &&
         nada_d_fwd_ >= max_del_history_.back().second) {
    max_del_history_.pop_back();
  }

  max_del_history_.push_back(std::make_pair(now_ms, nada_d_fwd_));
}

void NadaOwdBwe::UpdatePlrHistory(int64_t now_ms) {

  // Remove expired data points from history.
  while (!max_plr_history_.empty() &&
         now_ms - max_plr_history_.front().first > kNadaBweParamLogwinMs) {

        max_plr_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current PLR before pushing the current PLR
  while (!max_plr_history_.empty() &&
         nada_plr_ >= max_plr_history_.back().second) {
    max_plr_history_.pop_back();
  }

  max_plr_history_.push_back(std::make_pair(now_ms, nada_plr_));
}


//////////////////// Set Local Variables Upon Request ///////////////

void NadaOwdBwe::OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) {

   //  if (nada_rtt_base_in_ms_ < 0 || avg_rtt_ms < nada_rtt_base_in_ms_)
   //	    nada_rtt_base_in_ms_ = avg_rtt_ms;
   //
   // nada_rtt_in_ms_ = avg_rtt_ms;

   nada_rtt_avg_in_ms_ = avg_rtt_ms;
}

void NadaOwdBwe::SetStartBitrate(int start_bitrate_bps) {
  // Sergio: by-pass for the time being...
}

void NadaOwdBwe::SetMinBitrate(int min_bitrate_bps) {

  // Called from both the configuration thread and the network thread. Shouldn't
  // be called from the network thread in the future.


    // [XZ 2019-04-09: for now by pass the pass-along of Rmin, use hardcoded default instead]
    // printf("NadaOwdBwe: minimum rate set to %d bps\n", min_bitrate_bps);
    // nada_rmin_in_bps_ = min_bitrate_bps;

}


////////////// Anwsering Queries ///////////////////

//
// [XZ 2019-03-07]  TODO: check where & why this is called
//
bool NadaOwdBwe::LatestEstimate(std::vector<uint32_t>* ssrcs,
                                uint32_t* bitrate_bps) const {

  // Currently accessed from both the process thread (see
  // ModuleRtpRtcpImpl::Process()) and the configuration thread (see
  // Call::GetStats()). Should in the future only be accessed from a single
  // thread.
  RTC_DCHECK(ssrcs);
  RTC_DCHECK(bitrate_bps);

//  if (!rate_control_.ValidEstimate())
//    return false;

  *ssrcs = {kNadaOwdFixedSsrc};
  *bitrate_bps = nada_rate_in_bps_;
  // *bitrate_bps = rate_control_.LatestEstimate();
  return true;
}

//
// [XQ 2019-03-07]  figure out where & why this is called
//
int64_t NadaOwdBwe::GetExpectedBwePeriodMs() const {

  return probing_interval_in_ms_;
  //return rate_control_.GetExpectedBandwidthPeriodMs();
}

}  // namespace webrtc
