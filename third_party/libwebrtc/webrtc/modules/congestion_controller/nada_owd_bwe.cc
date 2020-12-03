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

// Default values to fulfill the external API calls 
constexpr int64_t kDefaultProbingIntervalinMs = 100; 
constexpr uint32_t kDefaultFixedSsrc = 0;

}  // namespace

namespace webrtc {

NadaOwdBwe::NadaOwdBwe(const Clock* clock)
    : DelayBasedBweInterface(),
      clock_(clock),
      first_update_ms_(-1),
      last_update_ms_(-1),
      last_seen_seqno_(-1),
      last_arrival_time_ms_(-1),
      nada_rate_in_bps_(kNadaDefaultBitrate),
      core_() {

      printf("Initializing the OWD-based NADA BW Estimation Module\n");
      RTC_LOG(LS_INFO) << "Initializing the OWD-based NADA BW Estimation Module" ;
}

NadaOwdBwe::~NadaOwdBwe() {}

// Main API for BW estimation, triggered by receiving transport FB vectors
DelayBasedBwe::Result NadaOwdBwe::IncomingPacketFeedbackVector(
    const std::vector<PacketFeedback>& packet_feedback_vector,
    rtc::Optional<uint32_t> acked_bitrate_bps) {

  RTC_DCHECK(std::is_sorted(packet_feedback_vector.begin(),
                            packet_feedback_vector.end(),
                            PacketFeedbackComparator()));

  int64_t now_ms = clock_->TimeInMilliseconds();
  int nfb = int(packet_feedback_vector.size());

  int64_t fbint = kNadaDefaultFBIntervalMs;  
  if (last_update_ms_ > 0)
    fbint = now_ms - last_update_ms_;

  int64_t ts = 0;  // relative timestamp
  if (first_update_ms_ < 0)
    first_update_ms_ = now_ms;
  else
    ts = now_ms - first_update_ms_; 

  core_.UpdateDelta(fbint); 

  printf("NadaOwdBwe::IncomingPacketFeedbackVector | t=%lld (%lld) ms, nfb = %d, fbint = %6lld ms\n",
         now_ms, ts,  nfb, fbint);

  RTC_LOG(LS_INFO) << "NADA IncomingPacketFBVector" 
                   << " | t = " << now_ms 
                   << " | t_rel = " <<  ts
                   << " | nfb = " << nfb
                   << " | fbint = " << fbint << "ms" << std::endl; 

  RTC_DCHECK_RUNS_SERIALIZED(&network_race_);

  DelayBasedBwe::Result result;

  int64_t dtmp = 0.;
  int64_t dmin = -1;  // locally min-filtered version of one-way delay
  int64_t rtt = 0;
  int nloss = 0;
  int npkts = 0; 
  int nbytes = 0;
  uint64_t curr_arrival_time_ms = 0;

  int ipkt = 0;  // pkt index in feedback vector
  for (const auto& packet_feedback : packet_feedback_vector) {

    if (packet_feedback.send_time_ms < 0) {
      // Handle duplicate FB vectors, possibly due to duplicated packets.
      // Bypass rest of delay statistics update
      RTC_LOG(LS_VERBOSE) << "NADA IncomingPacketFBVector | Detected duplicate FB vectors for pkt no. " 
                          << packet_feedback.sequence_number << std::endl; 
      result.updated = false;
      return result;
    }

    // [XZ-2020-11-05] Update instantaneous OWD and RTT: 
    // switch to use creation_time_ms as sending ts since send_time_ms follows a different clock
    // added RTC_CHECK_LE to make sure this subtraction doesn't wrap

    RTC_CHECK_LE(packet_feedback.creation_time_ms, packet_feedback.arrival_time_ms);
    dtmp = packet_feedback.arrival_time_ms - packet_feedback.creation_time_ms; 

    RTC_CHECK_LE(packet_feedback.creation_time_ms, now_ms);
    rtt = now_ms - packet_feedback.creation_time_ms; 
    core_.UpdateRttStats(now_ms, rtt);

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

    printf("\t pktinfo | seqno=%6d, pktsize=%6d | creatts=%8lld, sendts=%8lld, recvts=%8lld, ackts=%8lld | d_fwd=%8lld  ms | rtt=%8lld ms\n",
           packet_feedback.sequence_number,
           int(packet_feedback.payload_size),
           packet_feedback.creation_time_ms,
           packet_feedback.send_time_ms,
           packet_feedback.arrival_time_ms,
           now_ms,
           dtmp, 
           rtt); 

    RTC_LOG(LS_INFO) << " NADA IncomingPacketFBVector | pktinfo " 
                      << " | seqno: "   <<  packet_feedback.sequence_number
                      << " | pktsize: " <<  packet_feedback.payload_size << " bytes"
                      << " | creatts: " << packet_feedback.creation_time_ms << " ms"
                      << " | sendts: "  << packet_feedback.send_time_ms << " ms"
                      << " | recvts: "  << packet_feedback.arrival_time_ms << " ms"
                      << " | ackts: "   << now_ms << " ms"
                      << " | d_fwd: "   << dtmp << " ms"
                      << " | rtt: "     << rtt  << " ms"
                      << " | nloss: "   << nloss 
                      << std::endl; 

    // minimum-filtering of past per-packet one-way-delay
    if (ipkt > nfb - kNadaMinFilterWin) {
        if (dmin < 0  || dmin > dtmp) dmin = dtmp;
    }
    ipkt ++;
  }

  // per feedback interval
  if (last_arrival_time_ms_>0) {
    core_.CalcRecvRate(curr_arrival_time_ms, last_arrival_time_ms_, nbytes); 
    last_arrival_time_ms_ = curr_arrival_time_ms; 
  }

  // update per-interval packet stats (delay/PLR)
  core_.UpdateOwdStats(now_ms, dmin);
  core_.UpdatePlrStats(now_ms, nloss, npkts); 

  // non-linear delay warping + loss penalty
  nada_rate_in_bps_ = core_.UpdateNadaRate(now_ms, false); 
  core_.LogUpdate("nada_owd", ts); 

  // update local cache and return updated rate
  last_update_ms_ = now_ms;
  result.target_bitrate_bps = nada_rate_in_bps_;
  result.updated = true;
  return result;
}

//////////////////// Auxiliary Functions ///////////////
// We ignore updates to the RTT as one way delay (OWD) information is
// more accurate for this version of the algorithm
void NadaOwdBwe::OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) {

  printf("[DEBUG]: inside OnRttUpdate: rtt_avg = %8lld ms, rtt_max = %8lld ms\n", 
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

  RTC_LOG(LS_INFO) << " [DEBUG] inside SetMinBitrate: incoming min_bitrate ="
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

  *ssrcs = {kDefaultFixedSsrc};
  *bitrate_bps = nada_rate_in_bps_;

  printf("[DEBUG] inside LatestEstimate: reporting bitrate as %d Kbps\n", 
         nada_rate_in_bps_/1000); 

  RTC_LOG(LS_INFO) << " [DEBUG] inside LatestEstimate: reporting bitrate as"
                   << nada_rate_in_bps_/1000 << " Kbps" << std::endl; 

  return true;
}

int64_t NadaOwdBwe::GetExpectedBwePeriodMs() const {

  printf("[DEBUG]: inside GetExpectedBwePeriodMs: reporting probing interval as %8lld ms\n", 
         kDefaultProbingIntervalinMs); 

  RTC_LOG(LS_INFO) << "[DEBUG] inside GetExpectedBwePeriodMs: reporting probing interval as "
                   << kDefaultProbingIntervalinMs << " ms" << std::endl; 

  return kDefaultProbingIntervalinMs;
}

}  // namespace webrtc
