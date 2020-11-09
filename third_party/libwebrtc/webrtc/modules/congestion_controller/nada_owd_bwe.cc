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


constexpr int kNadaBweDefaultBitrate = 600000;      // 600 Kbps
constexpr int kNadaBweMinFilterWin = 5;      // # of taps for minimum filter 

constexpr int64_t kDefaultProbingIntervalinMs = 100;  // in ms

// This ssrc is used to fulfill the current API but will be removed
// after the API has been changed.
constexpr uint32_t kNadaOwdFixedSsrc = 0;

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
      core_() {

      printf("Initializing the OWD-based NADA BW Estimation Module\n");
      RTC_LOG(LS_INFO) << "Initializing the OWD-based NADA BW Estimation Module" ;
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
  float fbint = 100;  // default to 100ms
  if (last_update_ms_ > 0)
    fbint = now_ms - last_update_ms_;
  if (first_update_ms_ < 0)
    first_update_ms_ = now_ms;

  core_.UpdateDelta(fbint); 

  printf("NadaOwdBwe::IncomingPacketFeedbackVector | t=%lld (%lld) ms, nfb = %d, fbint = %6.2f ms\n",
         now_ms, now_ms - first_update_ms_,
         nfb, fbint);

  RTC_LOG(LS_INFO) << "NADA IncomingPacketFBVector" 
                   << " | t = " << now_ms 
                   << " | t_rel = " <<  now_ms-first_update_ms_
                   << " | nfb = " << nfb
                   << " | fbint = " << fbint << "ms" << std::endl; 


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
  // int nwin = 5;  // TODO: should make this dynamically tunable but how? [XZ-2019-03-08]
  double rtt = 0.;
  int64_t rtt_base = 0; 
  int64_t rtt_rel = 0; 
  double dq  = 0.;
  int nloss = 0;
  int npkts = 0; 
  int nbytes = 0;
  uint64_t curr_arrival_time_ms = 0;
  float d_base = 0.0; 
  for (const auto& packet_feedback : packet_feedback_vector) {

    if (packet_feedback.send_time_ms < 0) {
      // Handle duplicate FB vectors, possibly due to duplicated packets.
      // Bypass rest of delay statistics update
      RTC_LOG(LS_VERBOSE) << "NADA IncomingPacketFBVector | Detected duplicate FB vectors for pkt no. " 
                          << packet_feedback.sequence_number << std::endl; 
      result.updated = false;
      return result;
    }

    // update instantaneous, baseline and relative OWD 
    // dtmp = packet_feedback.arrival_time_ms - packet_feedback.send_time_ms;
    dtmp = packet_feedback.arrival_time_ms - packet_feedback.creation_time_ms; // [XZ-2020-011-05]

    core_.UpdateDminHistory(now_ms, dtmp);
    d_base = core_.GetDmin(); 
    dq = dtmp - d_base;

    // update instantaneous baseline and relative RTT 
    // rtt = now_ms - packet_feedback.send_time_ms;
    rtt = now_ms - packet_feedback.creation_time_ms;  // [XZ-2020-011-05]
    core_.UpdateRttHistory(now_ms, rtt); 
    rtt_base = core_.GetRttmin(); 
    rtt_rel = rtt - rtt_base; 

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

    // Log per-pkt info
    printf("\t pktinfo | seqno=%6d, pktsize=%6d | creatts=%8lld, sendts=%8lld, recvts=%8lld, ackts=%8lld | d_fwd=%6.1f, dbase=%6.1f, dqueue=%6.1f ms | rtt=%6.1f, rtt_base=%8lld, rtt_rel=%8lld ms\n",
           packet_feedback.sequence_number,
           int(packet_feedback.payload_size),
           packet_feedback.creation_time_ms,
           packet_feedback.send_time_ms,
           packet_feedback.arrival_time_ms,
           now_ms,
           dtmp, d_base, dq, 
           rtt,
           rtt_base,
           rtt_rel);

    RTC_LOG(LS_INFO) << " NADA IncomingPacketFBVector | pktinfo " 
                      << " | seqno: " <<  packet_feedback.sequence_number
                      << " | pktsize: " <<  packet_feedback.payload_size << " bytes"
                      << " | creatts: " << packet_feedback.creation_time_ms << " ms"
                      << " | sendts: "  << packet_feedback.send_time_ms << " ms"
                      << " | recvts: "  << packet_feedback.arrival_time_ms << " ms"
                      << " | ackts: "   << now_ms << " ms"
                      << " | d_fwd: " << dtmp << " ms"
                      << " | dbase: " << d_base << " ms"
                      << " | dqueue: " << dq << " ms" 
                      << " | rtt: " << rtt  << " ms"
                      << " | rtt_base: " << rtt_base << " ms"
                      << " | rtt_rel: " << rtt_rel << " ms" 
                      << " | ploss: " << nloss 
                      // << " | plr: " << std::fixed << nada_plr_*100. << " %"
                      << std::endl; 

    // minimum-filtering of past [15] FB reports on one-way-delay
    if (ipkt > nfb - kNadaBweMinFilterWin) {
        if (dmin < 0  || dmin > dtmp) dmin = dtmp;
    }
    ipkt ++;
  }

  // // update delay measurements
  float d_queue = dmin - d_base;
  core_.UpdatePktStats(now_ms, dmin, rtt, nloss, npkts); 

  // non-linear delay warping + loss penalty
  core_.UpdateOwdCongestion(); 

  float rrate = core_.CalcRecvRate(curr_arrival_time_ms, last_arrival_time_ms_, nbytes); 
  last_arrival_time_ms_ = curr_arrival_time_ms;

  // core_.TestFunction("NADA owd");
  float xcurr = core_.GetCongestion();  

  printf("\t pktstats | delta=%6.2f d_fwd=%6.2f, d_base=%6.2f, d_queue=%6.2f ms, rtt=%6.2f, rtt_b=%8lld, rtt_rel=%8lld, r_recv = %6.2f, x_curr=%6.2f\n",
         fbint, dmin, d_base, d_queue,
         rtt, rtt_base, rtt_rel,
         // tmpplr*100, nada_plr_*100,
         rrate / 1000.0,
         xcurr);

  RTC_LOG(LS_INFO) << " NADA IncomingPacketFBVector | pktstats "
                   << "| delta="    << fbint
                   << ", d_fwd="    << dmin
                   << ", d_base="   << d_base
                   << ", d_queue="  << d_queue
                   // << ", d_tilde"   << d_tilde << " ms"
                   << ", rtt="      << rtt
                   << ", rtt_b="    << rtt_base
                   << ", rtt_rel="  << rtt_rel
                   // << ", tmpplr="   << tmpplr*100  // instantaneous PLR
                   // << ", plr="      << nada_plr_*100 // time-smoothed PLR
                   << ", r_recv="   << rrate / 1000.0 << " Kbps"
                   << ", x_curr="   << xcurr << " ms" << std::endl;

  // switch between Accelerated-Ramp-Up mode and
  // Gradual-Update mode based on loss/delay observations
  // core_.UpdateDelHistory(now_ms, dmin);
  // core_.UpdatePlrHistory(now_ms, nada_plr_);

  int rmode = core_.GetRampUpMode(0); // use_rtt = 0
  if (rmode == 0)
    nada_rate_in_bps_ = core_.AcceleratedRampUp(now_ms, nada_rate_in_bps_); 
  else
    nada_rate_in_bps_ = core_.GradualRateUpdate(now_ms, nada_rate_in_bps_); 

  // clip the updated rate between [rmin, rmax]
  nada_rate_in_bps_ = core_.ClipBitrate(nada_rate_in_bps_);

  int64_t ts = now_ms - first_update_ms_; 
  core_.LogUpdate("nada_owd", ts); 

  printf("NadaOwdBwe: t = %lld ms mode = %d | r_curr = %6.2f Kbps | rtt_rel = %8lld ms, x_curr = %6.2f ms | r_recv = %6.2f Kbps\n",
         ts,
         rmode,
         nada_rate_in_bps_/1000.,
         rtt_rel,
         xcurr,
         rrate / 1000.);


  last_seen_packet_ms_ = now_ms;
  last_update_ms_ = now_ms;

  result.target_bitrate_bps = nada_rate_in_bps_;
  result.updated = true;
  return result;
}

//////////////////// Auxiliary Functions ///////////////
// We ignore updates to the RTT as one way delay (OWD) information is
// more accurate for this version of the algorithm
void NadaOwdBwe::OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) {

  printf("[DEBUG]: inside OnRttUpdate: rtt_avg = %d ms, rtt_max = %d ms\n", 
         avg_rtt_ms, max_rtt_ms); 

  RTC_LOG(LS_INFO) << "[DEBUG]  inside OnRttUpdate: "
                   << "  rtt_avg =" << avg_rtt_ms 
                   << " ms , rtt_max = " << max_rtt_ms
                   << " ms" << std::endl; 
}

// This method can be implemented if need be, but current version of the
// algorithm works well without it
void NadaOwdBwe::SetStartBitrate(int start_bitrate_bps) {

  printf("[DEBUG]: inside SetStartBitrate: start_bitrate = %d Kbps\n", 
         start_bitrate_bps/1000); 

  RTC_LOG(LS_INFO) << " [DEBUG]  inside SetStartBitrate: start_bitrate ="
                   << start_bitrate_bps/1000 << " Kbps" << std::endl; 
}

// This method can be implemented if need be, but current version of the
// algorithm works well without it
void NadaOwdBwe::SetMinBitrate(int min_bitrate_bps) {

  printf("[DEBUG]: inside SetMinBitrate: incoming min_bitrate = %d Kbps\n", 
         min_bitrate_bps/1000); 

  RTC_LOG(LS_INFO) << " [DEBUG]  inside SetMinBitrate: incoming min_bitrate ="
                   << min_bitrate_bps/1000 << " Kbps" << std::endl; 
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

  printf("[DEBUG]: inside LatestEstimate: reporting bitrate as %d Kbps\n", 
         nada_rate_in_bps_/1000); 

  RTC_LOG(LS_INFO) << " [DEBUG]  inside LatestEstimate: reporting bitrate as"
                   << nada_rate_in_bps_/1000 << " Kbps" << std::endl; 

  return true;
}

int64_t NadaOwdBwe::GetExpectedBwePeriodMs() const {

  return kDefaultProbingIntervalinMs;
}

}  // namespace webrtc
