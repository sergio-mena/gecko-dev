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

#include "system_wrappers/include/field_trial.h"
#include "system_wrappers/include/metrics.h"
#include "typedefs.h"

namespace {

/*
 * Default parameter values for the NADA algorithm.
 * See Fig. 3 in https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
 * for more details.
 */
constexpr float kNadaBweParamPrio  = 1.0;   // weight of priority for the flow [PRIO: dimensionless]
constexpr float kNadaBweParamXref  = 10.0;  // reference congestion level  [XREF: in ms]
constexpr float kNadaBweParamXDefault = 20.0;  // default congestion level [in ms]
constexpr float kNadaBweParamKappa = 0.5;   // scaling parameter for gradual rate update [KAPPA: dimensionless]
constexpr float kNadaBweParamEta  = 2.0;  // scaling parameter for gradual rate update [ETA: dimensionless]
constexpr float kNadaBweParamTau = 500.;  // Upper bound of RTT for gradual rate update [TAU: in ms]

constexpr uint32_t kNadaBweDefaultBitrate = 600000;      // 600 Kbps
constexpr uint32_t kNadaBweDefaultMinBitrate =  300000;  //  300 Kbps
constexpr uint32_t kNadaBweDefaultMaxBitrate = 3000000;  // 3000 Kbps

constexpr int64_t kNadaBweParamDeltaMs = 100; // Target interval for feedback and/or rate update [DELTA: in ms]
constexpr int64_t kNadaBweParamMinDeltaMs =  20; // Minimum value of delta for rate calculation [in ms]
constexpr int64_t kNadaBweParamMaxDeltaMs = 200; // Maximum value of delta for rate calculation [in ms]
constexpr int64_t kNadaBweParamLogwinMs  = 500;  // Observation time window for calculating
constexpr int64_t kNadaBweParamLogwinMs2 = 3000; // Observation time window for long-term minimal baseline forward delay value [in ms]
constexpr int64_t  kNadaBweDefaultRttMs = 50.;   // in ms
constexpr int64_t  kNadaBweParamQepsMs = 10;    // Threshold for determining queueing delay build-up [QEPS: in ms]
constexpr int64_t  kNadaBweParamQboundMs = 50;  // Upper bound on self-inflicted queuing delay [QBOUND: in ms]
constexpr int64_t  kNadaBweParamDfiltMs = 120;  // Bound on filtering delay [DFILT: in ms]
constexpr float    kNadaBweParamGammaMax = 0.2;  // Upper bound on rate increase ratio for accelerated ramp-up
                                                // [GAMMA_MAX: dimensionless]
constexpr float    kNadaBweParamPlrSmooth = 0.1;  // Smoothing factor for EMA of packet loss ratio

constexpr float    kNadaBweParamQthMs =50.;    // Delay threshold for invoking non-linear warping (QTH: in ms)
constexpr float    kNadaBweParamLambda =0.5;   // Scaling parameter in the exponent of non-linear warping (LAMBDA: dimensionless)
constexpr float    kNadaBweParamPlrRef =0.01;  // Reference packet loss ratio (PLRREF: dimensionless)
constexpr float    kNadaBweParamDLossMs =10.;  // Reference delay penalty for loss when packet loss ratio is at PLRREF 
                                               // (DLOSS: dimensionless)

constexpr int64_t kDefaultProbingIntervalinMs = 100;  // in ms
constexpr int64_t kStreamTimeOutMs = 2000;            // in ms

// This ssrc is used to fulfill the current API but will be removed
// after the API has been changed.
constexpr uint32_t kNadaOwdFixedSsrc = 0;

// for Bitrate estimation in BitrateEstimator subclass
constexpr int kInitialRateEstmWindowMs = 500;
constexpr int kRateEstmWindowMs = 150;

}  // namespace

namespace webrtc {

///  start of NadaOwdBwe (NADA One-way-delay BW Estimation) ///
NadaOwdBwe::NadaOwdBwe(const Clock* clock)
    : DelayBasedBweInterface(),
      clock_(clock),
      last_update_ms_(-1),
      first_update_ms_(-1),
      last_seen_packet_ms_(-1),
      last_seen_seqno_(-1),
      last_arrival_time_ms_(-1),
      nada_rate_in_bps_(kNadaBweDefaultBitrate),
      nada_rmin_in_bps_(kNadaBweDefaultMinBitrate),
      nada_rmax_in_bps_(kNadaBweDefaultMaxBitrate),
      nada_rtt_in_ms_(kNadaBweDefaultRttMs),
      nada_rtt_base_in_ms_(-1.),
      nada_rtt_rel_in_ms_(0.),
      nada_x_curr_(kNadaBweParamXDefault),
      nada_x_prev_(kNadaBweParamXDefault),
      nada_delta_(kNadaBweParamDeltaMs),
      nada_d_fwd_(-1.),
      nada_d_base_(-1.),
      nada_d_queue_(-1.),
      nada_plr_(0.) {
}

NadaOwdBwe::~NadaOwdBwe() {}

// TODO:  An alternative way of handling packet delay logs and duplicate
// FB vectors would be:
// * keep a local log (list) of "raw" pkt one-way-delay info along with seqno, etc.
// * decouple rate calculation from per-pkt stats update

// Main API for BW estimation, triggered by receiving transport FB vectors
DelayBasedBwe::Result NadaOwdBwe::IncomingPacketFeedbackVector(
    const std::vector<PacketFeedback>& packet_feedback_vector,
    rtc::Optional<uint32_t> acked_bitrate_bps) {
  RTC_DCHECK(std::is_sorted(packet_feedback_vector.begin(),
                            packet_feedback_vector.end(),
                            PacketFeedbackComparator()));

  int64_t now_ms = clock_->TimeInMilliseconds();
  int nfb = int(packet_feedback_vector.size());
  if (last_update_ms_ > 0)
    nada_delta_ = now_ms - last_update_ms_;
  if (first_update_ms_ < 0)
    first_update_ms_ = now_ms;

  printf("NadaOwdBwe::IncomingPacketFeedbackVector | t=%lld (%lld) ms, nfb = %d, fbint = %6.2f ms\n",
         now_ms, now_ms - first_update_ms_,
         nfb, nada_delta_);

  RTC_LOG(LS_INFO) << "NADA IncomingPacketFBVector" 
                   << " | t = " << now_ms 
                   << " | t_rel = " <<  now_ms-first_update_ms_
                   << " | nfb = " << nfb
                   << " | fbint = " << nada_delta_ << "ms" << std::endl; 


  RTC_DCHECK_RUNS_SERIALIZED(&network_race_);


  // [TODO Xiaoqing to decide if this is worth doing]
  // TODO: replace loop below with NADA calculation:
  // Step 1)  FB Vector => per-pkt <seqno, owd> info *
  //
  // Step 2)  Rate calculation update based on OWD/loss/RTT/recv_rate **
  //          2.a: accelerated ramp-up (probing)
  //          2.b: gradual update
  //
  // Step 3)  Save to result struct

  DelayBasedBwe::Result result;

  double dtmp = 0.;
  double dmin = -1.;
  int ipkt = 0;
  int nwin = 5;  // TODO: should make this dynamically tunable but how? [XZ-2019-03-08]
  double rtt = 0.;
  double dq  = 0.;
  int nloss = 0;
  int npkts = 0; 
  int nbytes = 0;
  uint64_t curr_arrival_time_ms = 0;
  double tmpplr = 0.0; 
  for (const auto& packet_feedback : packet_feedback_vector) {
    if (packet_feedback.send_time_ms < 0) {
      // Handle duplicate FB vectors, possibly due to duplicated packets.
      // Bypass rest of delay statistics update
      RTC_LOG(LS_VERBOSE) << "NADA IncomingPacketFBVector | Detected duplicate FB vectors for pkt no. " 
                          << packet_feedback.sequence_number << std::endl; 
      result.updated = false;
      return result;
    }

    // update delay info: d_fwd, d_base, d_queue, rtt
    dtmp = packet_feedback.arrival_time_ms - packet_feedback.send_time_ms;
    rtt = now_ms - packet_feedback.send_time_ms;

    // update d_base and rtt_base
    UpdateDminHistory(now_ms, dtmp);
    nada_d_base_ = dmin_history_.front().second;

    if (nada_rtt_base_in_ms_ < 0 || nada_rtt_base_in_ms_ > rtt) nada_rtt_base_in_ms_ = rtt;

    // calculate relative RTT and OWD accordingly
    nada_rtt_in_ms_ = rtt;
    nada_rtt_rel_in_ms_ = nada_rtt_in_ms_ - nada_rtt_base_in_ms_;

    dq = dtmp - nada_d_base_;

    npkts ++;
    nbytes += packet_feedback.payload_size;
    curr_arrival_time_ms = packet_feedback.arrival_time_ms;
    // update pkt loss info
    if (last_seen_seqno_ > 0 && packet_feedback.sequence_number > last_seen_seqno_ + 1)
    {
      if (packet_feedback.sequence_number-last_seen_seqno_ < 1000) // avoid wrap-around
        nloss += packet_feedback.sequence_number-last_seen_seqno_-1; 
    }
    last_seen_seqno_ = packet_feedback.sequence_number;

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


    RTC_LOG(LS_INFO) << " NADA IncomingPacketFBVector | pktinfo " 
                      << " | seqno: " <<  packet_feedback.sequence_number
                      << " | pktsize: " <<  packet_feedback.payload_size << " bytes"
                      << " | creatts: " << packet_feedback.creation_time_ms << " ms"
                      << " | sendts: "  << packet_feedback.send_time_ms << " ms"
                      << " | recvts: "  << packet_feedback.arrival_time_ms << " ms"
                      << " | ackts: "   << now_ms << " ms"
                      << " | d_fwd: " << dtmp << " ms"
                      << " | dbase: " << nada_d_base_ << " ms"
                      << " | dqueue: " << dq << " ms" 
                      << " | rtt: " << rtt  << " ms"
                      << " | rtt_base: " << nada_rtt_base_in_ms_ << " ms"
                      << " | rtt_rel: " << nada_rtt_rel_in_ms_ << " ms" 
                      << " | ploss: " << nloss 
                      << " | plr: " << std::fixed << nada_plr_*100. << " %"
                      << std::endl; 

    // minimum-filtering of past [15] FB reports
    if (ipkt > nfb - nwin) {
        if (dmin < 0  || dmin > dtmp) dmin = dtmp;
    }
    ipkt ++;
  }

  // update delay measurements
  nada_d_fwd_ = dmin;
  nada_d_queue_ = nada_d_fwd_ - nada_d_base_;
  tmpplr = double(nloss)/(double(npkts+nloss));

  nada_plr_ += kNadaBweParamPlrSmooth * (tmpplr - nada_plr_);  // exponential smoothing

  // non-linear delay warping
  double d_tilde = nada_d_queue_;
  if (nloss > 0)
  {
    if (nada_d_queue_ > kNadaBweParamQthMs)
   {
     double beta = kNadaBweParamLambda*(nada_d_queue_-kNadaBweParamQthMs)/kNadaBweParamQthMs;
     d_tilde = kNadaBweParamQthMs*exp(-beta);
   }
  }
  nada_x_curr_ = d_tilde +  kNadaBweParamDLossMs * (nada_plr_/kNadaBweParamPlrRef)*(nada_plr_/kNadaBweParamPlrRef);

  float rrate = 0.;
  if (last_arrival_time_ms_ > 0) {
    uint64_t dt = curr_arrival_time_ms - last_arrival_time_ms_;
    rrate = float(nbytes) * 8000. / double(dt);
    RTC_LOG(LS_INFO) << "dt: " << dt << ", nbytes: " << nbytes << std::endl;
  }

  last_arrival_time_ms_ = curr_arrival_time_ms;

  printf("\t pktstats | delta=%6.2f d_fwd=%6.2f, d_base=%6.2f, d_queue=%6.2f ms, rtt=%6.2f, rtt_b=%6.2f, rtt_rel=%6.2f, plr = %6.2f, %6.2f, r_recv = %6.2f, x_curr=%6.2f\n",
         nada_delta_, nada_d_fwd_, nada_d_base_, nada_d_queue_,
         nada_rtt_in_ms_, nada_rtt_base_in_ms_, nada_rtt_rel_in_ms_,
         tmpplr*100, nada_plr_*100,
         rrate / 1000.0,
         nada_x_curr_);

  RTC_LOG(LS_INFO) << " NADA IncomingPacketFBVector | pktstats "
                   << "| delta="    << nada_delta_
                   << ", d_fwd="    << nada_d_fwd_
                   << ", d_base="   << nada_d_base_
                   << ", d_queue="  << nada_d_queue_
                   << ", d_tilde"   << d_tilde << " ms"
                   << ", rtt="      << nada_rtt_in_ms_
                   << ", rtt_b="    << nada_rtt_base_in_ms_
                   << ", rtt_rel="  << nada_rtt_rel_in_ms_
                   << ", tmpplr="   << tmpplr*100  // instantaneous PLR
                   << ", plr="      << nada_plr_*100 // time-smoothed PLR
                   << ", r_recv="   << rrate / 1000.0 << " Kbps"
                   << ", x_curr="   << nada_x_curr_ << " ms" << std::endl;

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
         rrate / 1000.);

  std::ostringstream os;
  os << std::fixed;
  os.precision(2);
  RTC_LOG(LS_INFO) << " NADA IncomingPacketFBVector | algo:nada_owd "     // 1) CC algorithm flavor
               << " | ts: "     << now_ms - first_update_ms_ << " ms"     // 2) timestamp
               << " | fbint: "  << nada_delta_ << " ms"                   // 3) feedback interval
               << " | qdel: "   << nada_d_queue_ << " ms"                 // 4) queuing delay
               << " | rtt: "    << nada_rtt_in_ms_ << " ms"               // 5) RTT
               << " | ploss: "  << nloss                                  // 6) packet loss count
               << " | plr: "    << std::fixed << nada_plr_*100.  << " %"  // 7) temporallysmoothed packet loss ratio
               << " | rmode: "  << rmode                                  // 8) rate update mode: accelerated ramp-up or gradual
               << " | xcurr: "   << std::fixed << nada_x_curr_ << " ms"   // 9) aggregated congestion signal
               << " | rrate: "  << rrate / 1000. << " Kbps"               // 10) receiving rate
               << " | srate: "  << nada_rate_in_bps_ / 1000. << " Kbps"   // 11) sending rate
               << " | rmin: "    << nada_rmin_in_bps_ / 1000. << " Kbps"  // 12) minimum rate
               << " | rmax: "    << nada_rmax_in_bps_ / 1000. << " Kbps"  // 13) maximum rate
               << std::endl;

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

  // calculate multiplicative ramp-up ratio
  float gamma = kNadaBweParamQboundMs /(nada_rtt_in_ms_ + kNadaBweParamDeltaMs + kNadaBweParamDfiltMs);
  if (gamma > kNadaBweParamGammaMax) gamma = kNadaBweParamGammaMax;
  printf("\t NadaOwdBwe::AcceleratedRampUp:  ramp-up ratio gamma = %4.2f, r_curr = %6.2f Kbps\n", gamma, nada_rate_in_bps_/1000.);

  RTC_LOG(LS_VERBOSE) << "NADA AcceleratedRampUp "
                      << "| ramp-up ratio gamma=" << gamma
                      << ", r_curr=" << nada_rate_in_bps_/1000. << " Kbps" << std::endl; 

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
    if (kNadaBweParamKappa*(w1+w2) < 1.0) {
        nada_rate_in_bps_ = nada_rate_in_bps_*(1-kNadaBweParamKappa*(w1+w2));
    } else {
        nada_rate_in_bps_ = 0;
    }

    printf("\t NadaOwdBwe::GradualRateUpdate:  x_curr = %6.2f ms | r_curr = %6.2f Kbps\n", nada_x_curr_, nada_rate_in_bps_/1000.);


    RTC_LOG(LS_VERBOSE) << "NADA GradualRateUpdate "
                        << "| x_curr=" << nada_x_curr_ << " ms " 
                        << "| r_curr=" << nada_rate_in_bps_/1000. << " Kbps" << std::endl; 

    // cache current observation as x_prev
    nada_x_prev_ = nada_x_curr_;
}


//////////////////// Auxiliary Functions ///////////////
void NadaOwdBwe::ClipBitrate() {

  if (nada_rate_in_bps_ < nada_rmin_in_bps_) {
    nada_rate_in_bps_ = nada_rmin_in_bps_;
  }
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
  // Pop values higher than current delay value before pushing the current delay value.
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

// We ignore updates to the RTT as one way delay (OWD) information is
// more accurate for this version of the algorithm
void NadaOwdBwe::OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) {
}

// This method can be implemented if need be, but current version of the
// algorithm works well without it
void NadaOwdBwe::SetStartBitrate(int start_bitrate_bps) {
}

// This method can be implemented if need be, but current version of the
// algorithm works well without it
void NadaOwdBwe::SetMinBitrate(int min_bitrate_bps) {
}

bool NadaOwdBwe::LatestEstimate(std::vector<uint32_t>* ssrcs,
                                uint32_t* bitrate_bps) const {

  // Currently accessed from both the process thread (see
  // ModuleRtpRtcpImpl::Process()) and the configuration thread (see
  // Call::GetStats()). Should in the future only be accessed from a single
  // thread.
  RTC_DCHECK(ssrcs);
  RTC_DCHECK(bitrate_bps);

  *ssrcs = {kNadaOwdFixedSsrc};
  *bitrate_bps = nada_rate_in_bps_;
  return true;
}

int64_t NadaOwdBwe::GetExpectedBwePeriodMs() const {
  return kDefaultProbingIntervalinMs;
}

}  // namespace webrtc
