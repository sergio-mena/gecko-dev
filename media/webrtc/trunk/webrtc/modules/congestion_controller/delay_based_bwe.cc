/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/delay_based_bwe.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "logging/rtc_event_log/events/rtc_event_bwe_update_delay_based.h"
#include "logging/rtc_event_log/rtc_event_log.h"
#include "modules/pacing/paced_sender.h"
#include "modules/remote_bitrate_estimator/include/remote_bitrate_estimator.h"
#include "modules/remote_bitrate_estimator/test/bwe_test_logging.h"
#include "rtc_base/checks.h"
#include "rtc_base/constructormagic.h"
#include "rtc_base/logging.h"
#include "rtc_base/ptr_util.h"
#include "rtc_base/thread_annotations.h"
#include "system_wrappers/include/field_trial.h"
#include "system_wrappers/include/metrics.h"
#include "typedefs.h"  // NOLINT(build/include)

namespace {
constexpr int kTimestampGroupLengthMs = 5;
constexpr int kAbsSendTimeFraction = 18;
constexpr int kAbsSendTimeInterArrivalUpshift = 8;
constexpr int kInterArrivalShift =
    kAbsSendTimeFraction + kAbsSendTimeInterArrivalUpshift;
constexpr double kTimestampToMs =
    1000.0 / static_cast<double>(1 << kInterArrivalShift);
// This ssrc is used to fulfill the current API but will be removed
// after the API has been changed.
constexpr uint32_t kFixedSsrc = 0;

// Parameters for linear least squares fit of regression line to noisy data.
constexpr size_t kDefaultTrendlineWindowSize = 20;
constexpr double kDefaultTrendlineSmoothingCoeff = 0.9;
constexpr double kDefaultTrendlineThresholdGain = 4.0;

constexpr int kMaxConsecutiveFailedLookups = 5;

const char kBweSparseUpdateExperiment[] = "WebRTC-BweSparseUpdateExperiment";
const char kBweWindowSizeInPacketsExperiment[] =
    "WebRTC-BweWindowSizeInPackets";

size_t ReadTrendlineFilterWindowSize() {
  std::string experiment_string =
      webrtc::field_trial::FindFullName(kBweWindowSizeInPacketsExperiment);
  size_t window_size;
  int parsed_values =
      sscanf(experiment_string.c_str(), "Enabled-%zu", &window_size);
  if (parsed_values == 1) {
    if (window_size > 1)
      return window_size;
    RTC_LOG(WARNING) << "Window size must be greater than 1.";
  }
  RTC_LOG(LS_WARNING) << "Failed to parse parameters for BweTrendlineFilter "
                         "experiment from field trial string. Using default.";
  return kDefaultTrendlineWindowSize;
}
}  // namespace

namespace webrtc {

DelayBasedBwe::DelayBasedBwe(RtcEventLog* event_log, const Clock* clock)
    : DelayBasedBweInterface(),
      event_log_(event_log),
      clock_(clock),
      inter_arrival_(),
      trendline_estimator_(),
      detector_(),
      first_seen_packet_ms_(-1),
      last_seen_packet_ms_(-1),
      feedback_interval_ms_(0),
      last_seen_seqno_(-1),
      default_bwe_npkts_(0),
      default_bwe_ploss_(0),
      default_bwe_plr_(0.),
      default_bwe_rtt_ms_(0),
      default_bwe_dqel_ms_(0),
      default_bwe_dbase_ms_(-1),
      default_bwe_nbytes_(0),
      default_bwe_rrate_(0.),
      last_arrival_time_ms_(-1),
      curr_arrival_time_ms_(0),
      uma_recorded_(false),
      probe_bitrate_estimator_(event_log),
      trendline_window_size_(
          webrtc::field_trial::IsEnabled(kBweWindowSizeInPacketsExperiment)
              ? ReadTrendlineFilterWindowSize()
              : kDefaultTrendlineWindowSize),
      trendline_smoothing_coeff_(kDefaultTrendlineSmoothingCoeff),
      trendline_threshold_gain_(kDefaultTrendlineThresholdGain),
      consecutive_delayed_feedbacks_(0),
      prev_bitrate_(0),
      prev_state_(BandwidthUsage::kBwNormal),
      in_sparse_update_experiment_(
          webrtc::field_trial::IsEnabled(kBweSparseUpdateExperiment)) {
  RTC_LOG(LS_INFO)
      << "Using Trendline filter for delay change estimation with window size "
      << trendline_window_size_;
}

DelayBasedBwe::~DelayBasedBwe() {}

DelayBasedBwe::Result DelayBasedBwe::IncomingPacketFeedbackVector(
    const std::vector<PacketFeedback>& packet_feedback_vector,
    rtc::Optional<uint32_t> acked_bitrate_bps) {
  RTC_DCHECK(std::is_sorted(packet_feedback_vector.begin(),
                            packet_feedback_vector.end(),
                            PacketFeedbackComparator()));

  // [XZ 2019-10-21  added logging of time stamp, # of FB pkts, etc]
  int64_t now_ms = clock_->TimeInMilliseconds();
  int nfb = int(packet_feedback_vector.size());
  if (last_seen_packet_ms_ > 0)
    feedback_interval_ms_ = now_ms - last_seen_packet_ms_; 

  printf("DelayBasedBwe::IncomingPktFBVector | t=%lld (%lld) ms, nfb = %d, fbint = %d\n",
  //  acked_rate = %d bps\n",
         now_ms, 
         now_ms - first_seen_packet_ms_, 
         nfb, 
         feedback_interval_ms_); 
         // acked_bitrate_bps);

  RTC_LOG(LS_INFO) << "DelayBasedBwe IncomingPacketFBVector" 
                   << " | t = "     << now_ms 
                   << " | t_rel = " <<  now_ms-first_seen_packet_ms_
                   << " | nfb = " << nfb 
                   // << " | acked_rate = "  << acked_bitrate_bps << " bps" 
                   << std::endl; 
  // [XZ 2019-10-21]

  RTC_DCHECK_RUNS_SERIALIZED(&network_race_);

  // TOOD(holmer): An empty feedback vector here likely means that
  // all acks were too late and that the send time history had
  // timed out. We should reduce the rate when this occurs.
  if (packet_feedback_vector.empty()) {
    RTC_LOG(LS_WARNING) << "Very late feedback received.";
    return DelayBasedBwe::Result();
  }

  if (!uma_recorded_) {
    RTC_HISTOGRAM_ENUMERATION(kBweTypeHistogram,
                              BweNames::kSendSideTransportSeqNum,
                              BweNames::kBweNamesMax);
    uma_recorded_ = true;
  }

  bool overusing = false;
  bool delayed_feedback = true;
  bool recovered_from_overuse = false;
  BandwidthUsage prev_detector_state = detector_.State();
  for (const auto& packet_feedback : packet_feedback_vector) {
    if (packet_feedback.send_time_ms < 0)
      continue;
    delayed_feedback = false;
    IncomingPacketFeedback(packet_feedback);

    //  TODO Sergio: interface of IncomingPacketFeedback is now "void"
    //  printf("\t\t Inside DelayBasedBwe | update based on pkt %d, result = %d\n",
    //      packet_info.sequence_number, result.updated);

    if (!in_sparse_update_experiment_)
      overusing |= (detector_.State() == BandwidthUsage::kBwOverusing);
    if (prev_detector_state == BandwidthUsage::kBwUnderusing &&
        detector_.State() == BandwidthUsage::kBwNormal) {
      recovered_from_overuse = true;
    }
    prev_detector_state = detector_.State();
  }

  // [XZ 2019-10-21  added logging of pkt loss ratio, etc]
  if (default_bwe_npkts_>0)  {
    double tmpplr = double(default_bwe_ploss_)/(double(default_bwe_npkts_+default_bwe_ploss_));
    default_bwe_plr_ += 0.1*(tmpplr - default_bwe_plr_);  // exponential smoothing
    printf("DelayBasedBwe: plr = %.2f | %.2f\n", tmpplr, default_bwe_plr_);
  }

  if (last_arrival_time_ms_>0) {
    uint64_t dt = curr_arrival_time_ms_ - last_arrival_time_ms_; 
    default_bwe_rrate_ = float(default_bwe_nbytes_)*8000./double(dt); 
    RTC_LOG(LS_INFO) << "dt: " << dt << ", nbytes: " << default_bwe_nbytes_ << std::endl; 
  } 

  last_arrival_time_ms_ = curr_arrival_time_ms_; 

  // [XZ 2019-10-21]

  if (in_sparse_update_experiment_)
    overusing = (detector_.State() == BandwidthUsage::kBwOverusing);

  if (delayed_feedback) {
    ++consecutive_delayed_feedbacks_;
    if (consecutive_delayed_feedbacks_ >= kMaxConsecutiveFailedLookups) {
      consecutive_delayed_feedbacks_ = 0;
      return OnLongFeedbackDelay(packet_feedback_vector.back().arrival_time_ms);
    }
  } else {
    consecutive_delayed_feedbacks_ = 0;
    return MaybeUpdateEstimate(overusing, acked_bitrate_bps,
                               recovered_from_overuse);
  }


  // reset
  default_bwe_ploss_ = 0; 
  default_bwe_npkts_ = 0; 
  default_bwe_nbytes_ = 0.;
  
  return Result();
}

DelayBasedBwe::Result DelayBasedBwe::OnLongFeedbackDelay(
    int64_t arrival_time_ms) {
  // Estimate should always be valid since a start bitrate always is set in the
  // Call constructor. An alternative would be to return an empty Result here,
  // or to estimate the throughput based on the feedback we received.
  RTC_DCHECK(rate_control_.ValidEstimate());
  rate_control_.SetEstimate(rate_control_.LatestEstimate() / 2,
                            arrival_time_ms);
  Result result;
  result.updated = true;
  result.probe = false;
  result.target_bitrate_bps = rate_control_.LatestEstimate();
  RTC_LOG(LS_WARNING) << "Long feedback delay detected, reducing BWE to "
                      << result.target_bitrate_bps;
  return result;
}

void DelayBasedBwe::IncomingPacketFeedback(
    const PacketFeedback& packet_feedback) {

  // [XZ 2019-10-21 added logging of FB interval, etc.]
  int64_t now_ms = clock_->TimeInMilliseconds();
  if (first_seen_packet_ms_ == -1) first_seen_packet_ms_ = now_ms; 
  // [XZ 2019-10-21]

  // Reset if the stream has timed out.
  if (last_seen_packet_ms_ == -1 ||
      now_ms - last_seen_packet_ms_ > kStreamTimeOutMs) {
    inter_arrival_.reset(
        new InterArrival((kTimestampGroupLengthMs << kInterArrivalShift) / 1000,
                         kTimestampToMs, true));
    trendline_estimator_.reset(new TrendlineEstimator(
        trendline_window_size_, trendline_smoothing_coeff_,
        trendline_threshold_gain_));
  }
  last_seen_packet_ms_ = now_ms;

  uint32_t send_time_24bits =
      static_cast<uint32_t>(
          ((static_cast<uint64_t>(packet_feedback.send_time_ms)
            << kAbsSendTimeFraction) +
           500) /
          1000) &
      0x00FFFFFF;
  // Shift up send time to use the full 32 bits that inter_arrival works with,
  // so wrapping works properly.
  uint32_t timestamp = send_time_24bits << kAbsSendTimeInterArrivalUpshift;

  uint32_t ts_delta = 0;
  int64_t t_delta = 0;
  int size_delta = 0;
  if (inter_arrival_->ComputeDeltas(timestamp, packet_feedback.arrival_time_ms,
                                    now_ms, packet_feedback.payload_size,
                                    &ts_delta, &t_delta, &size_delta)) {
    double ts_delta_ms = (1000.0 * ts_delta) / (1 << kInterArrivalShift);
    trendline_estimator_->Update(t_delta, ts_delta_ms,
                                 packet_feedback.arrival_time_ms);
    detector_.Detect(trendline_estimator_->trendline_slope(), ts_delta_ms,
                     trendline_estimator_->num_of_deltas(),
                     packet_feedback.arrival_time_ms);
  }
  if (packet_feedback.pacing_info.probe_cluster_id !=
      PacedPacketInfo::kNotAProbe) {
    probe_bitrate_estimator_.HandleProbeAndEstimateBitrate(packet_feedback);
  }

  // [XZ 2019-10-21 added logging of per-pkt loss/delay, etc.]
  if (packet_feedback.send_time_ms > 0) {
    // update delay info: d_fwd, d_base, d_queue, rtt
    default_bwe_rtt_ms_ = now_ms - packet_feedback.send_time_ms;
    uint64_t dtmp = packet_feedback.arrival_time_ms - packet_feedback.send_time_ms;
    if (default_bwe_dbase_ms_ < 0 || default_bwe_dbase_ms_ > dtmp) default_bwe_dbase_ms_ = dtmp;
    default_bwe_dqel_ms_ = dtmp - default_bwe_dbase_ms_; 

    // update loss info: ploss, plr
    default_bwe_npkts_ ++; 
    default_bwe_nbytes_ += packet_feedback.payload_size; 
    curr_arrival_time_ms_ = packet_feedback.arrival_time_ms; 
    if (last_seen_seqno_>0 && packet_feedback.sequence_number > last_seen_seqno_+1)
    {
      if (packet_feedback.sequence_number-last_seen_seqno_ < 1000) // avoid wrap-around
        default_bwe_ploss_ += packet_feedback.sequence_number-last_seen_seqno_-1; 
    }
    last_seen_seqno_ = packet_feedback.sequence_number;

    RTC_LOG(LS_INFO) << " DelayBWE IncomingPktFB | pktinfo "
                   << " | seqno: " <<  packet_feedback.sequence_number
                   << " | pktsize: " <<  packet_feedback.payload_size << " bytes"
                   << " | creatts: " << packet_feedback.creation_time_ms << " ms"
                   << " | sendts: "  << packet_feedback.send_time_ms << " ms"
                   << " | recvts: "  << packet_feedback.arrival_time_ms << " ms"
                   << " | ackts: "   << now_ms << " ms"
                   << " | dqel: " << default_bwe_dqel_ms_ << " ms"
                   << " | rtt: " << default_bwe_rtt_ms_  << " ms" 
                   << " | ploss: " << default_bwe_ploss_ 
                   << " | plr: " << std::fixed << default_bwe_plr_*100. << " %"
                   << std::endl; 

  }
  // [XZ 2019-10-21]
}

DelayBasedBwe::Result DelayBasedBwe::MaybeUpdateEstimate(
    bool overusing,
    rtc::Optional<uint32_t> acked_bitrate_bps,
    bool recovered_from_overuse) {
  Result result;
  int64_t now_ms = clock_->TimeInMilliseconds();

  rtc::Optional<int> probe_bitrate_bps =
      probe_bitrate_estimator_.FetchAndResetLastEstimatedBitrateBps();
  // Currently overusing the bandwidth.
  if (overusing) {
    if (acked_bitrate_bps &&
        rate_control_.TimeToReduceFurther(now_ms, *acked_bitrate_bps)) {
      result.updated = UpdateEstimate(now_ms, acked_bitrate_bps, overusing,
                                      &result.target_bitrate_bps);
    } else if (!acked_bitrate_bps && rate_control_.ValidEstimate() &&
               rate_control_.TimeToReduceFurther(
                   now_ms, rate_control_.LatestEstimate() / 2 - 1)) {
      // Overusing before we have a measured acknowledged bitrate. We check
      // TimeToReduceFurther (with a fake acknowledged bitrate) to avoid
      // reducing too often.
      // TODO(tschumim): Improve this and/or the acknowledged bitrate estimator
      // so that we (almost) always have a bitrate estimate.
      rate_control_.SetEstimate(rate_control_.LatestEstimate() / 2, now_ms);
      result.updated = true;
      result.probe = false;
      result.target_bitrate_bps = rate_control_.LatestEstimate();
    }
  } else {
    if (probe_bitrate_bps) {
      result.probe = true;
      result.updated = true;
      result.target_bitrate_bps = *probe_bitrate_bps;
      rate_control_.SetEstimate(*probe_bitrate_bps, now_ms);
    } else {
      result.updated = UpdateEstimate(now_ms, acked_bitrate_bps, overusing,
                                      &result.target_bitrate_bps);
      result.recovered_from_overuse = recovered_from_overuse;
    }
  }
  if ((result.updated && prev_bitrate_ != result.target_bitrate_bps) ||
      detector_.State() != prev_state_) {
    uint32_t bitrate_bps =
        result.updated ? result.target_bitrate_bps : prev_bitrate_;

    BWE_TEST_LOGGING_PLOT(1, "target_bitrate_bps", now_ms, bitrate_bps);

    if (event_log_) {
      event_log_->Log(rtc::MakeUnique<RtcEventBweUpdateDelayBased>(
          bitrate_bps, detector_.State()));
    }

    prev_bitrate_ = bitrate_bps;
    prev_state_ = detector_.State();
  }

  // [XZ 2019-10-21  added logging of BW estimation result]
  std::ostringstream os;
  os << std::fixed;
  os.precision(2);

  RTC_LOG(LS_INFO) << " DelayBasedBwe::MaybeUpdateEstimate | algo:default "       // 1) CC algorithm flavor
                     << " | ts: "     << now_ms - first_seen_packet_ms_ << " ms"  // 2) timestamp
                     << " | fbint: "  << feedback_interval_ms_ << " ms"              // 3) feedback interval
                     << " | qdel: "   << default_bwe_dqel_ms_ << " ms"                 // 4) queuing delay 
                     << " | rtt: "    << default_bwe_rtt_ms_ << " ms"               // 5) RTT
                     << " | ploss: "  << default_bwe_ploss_                                  // 6) packet loss count
                     << " | plr: "    << std::fixed << default_bwe_plr_*100.  << " %"                 // 7) temporally packet loss ratio
                     << " | update: "  << result.updated                  // 9) aggregated congestion signal 
                     << " | probe: "   << result.probe                           // 8) rate update mode: accelerated ramp-up or gradual 
                     // << " | state: "   << prev_state_                       // 
                     << " | rrate: "   << default_bwe_rrate_/1000. << " Kbps"     // 10) receiving rate 
                     << " | srate: "    << result.target_bitrate_bps/1000. << " Kbps"     // 11) sending rate
                     << std::endl;

  // [XZ 2019-10-21]

  return result;
}

bool DelayBasedBwe::UpdateEstimate(int64_t now_ms,
                                   rtc::Optional<uint32_t> acked_bitrate_bps,
                                   bool overusing,
                                   uint32_t* target_bitrate_bps) {
  // TODO(terelius): RateControlInput::noise_var is deprecated and will be
  // removed. In the meantime, we set it to zero.
  const RateControlInput input(
      overusing ? BandwidthUsage::kBwOverusing : detector_.State(),
      acked_bitrate_bps, 0);
  *target_bitrate_bps = rate_control_.Update(&input, now_ms);
  return rate_control_.ValidEstimate();
}

void DelayBasedBwe::OnRttUpdate(int64_t avg_rtt_ms, int64_t max_rtt_ms) {
  rate_control_.SetRtt(avg_rtt_ms);
}

bool DelayBasedBwe::LatestEstimate(std::vector<uint32_t>* ssrcs,
                                   uint32_t* bitrate_bps) const {
  // Currently accessed from both the process thread (see
  // ModuleRtpRtcpImpl::Process()) and the configuration thread (see
  // Call::GetStats()). Should in the future only be accessed from a single
  // thread.
  RTC_DCHECK(ssrcs);
  RTC_DCHECK(bitrate_bps);
  if (!rate_control_.ValidEstimate())
    return false;

  *ssrcs = {kFixedSsrc};
  *bitrate_bps = rate_control_.LatestEstimate();
  return true;
}

void DelayBasedBwe::SetStartBitrate(int start_bitrate_bps) {
  RTC_LOG(LS_WARNING) << "BWE Setting start bitrate to: " << start_bitrate_bps;
  rate_control_.SetStartBitrate(start_bitrate_bps);
}

void DelayBasedBwe::SetMinBitrate(int min_bitrate_bps) {
  // Called from both the configuration thread and the network thread. Shouldn't
  // be called from the network thread in the future.
  rate_control_.SetMinBitrate(min_bitrate_bps);
}

int64_t DelayBasedBwe::GetExpectedBwePeriodMs() const {
  return rate_control_.GetExpectedBandwidthPeriodMs();
}
}  // namespace webrtc
