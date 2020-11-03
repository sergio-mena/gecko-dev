/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/remote_bitrate_estimator/include/nada_core.h"
#include "rtc_base/logging.h"

namespace webrtc {
	namespace {

/*
 * Default parameter values for the NADA algorithm.
 * See Fig. 3 in https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
 * for more details.
 */
  
constexpr float kNADAParamPrio  = 1.0;   // weight of priority for the flow [PRIO: dimensionless]
constexpr float kNADAParamXref  = 10.0;  // reference congestion level  [XREF: in ms]
constexpr float kNADAParamXDefault = 20.0;  // default congestion level [in ms]
constexpr float kNADAParamKappa = 0.5;   // scaling parameter for gradual rate update [KAPPA: dimensionless]
constexpr float kNADAParamEta  = 2.0;  // scaling parameter for gradual rate update [ETA: dimensionless]
constexpr float kNADAParamTau = 500.;  // Upper bound of RTT for gradual rate update [TAU: in ms]

constexpr int64_t kNADAParamLogwinMs = 500; 	// Observation time window for maintaining 
                                                // short-term stats of rate/delay/plr [in ms]
constexpr int64_t kNADAParamLogwinMs2 = 3000; 	// Observation time window for maintaining
                                                // long-term minimal baseline forward delay value 
                                                // [in ms]

constexpr int64_t kNADAParamDeltaMs = 100; 		// Target interval for feedback and/or rate update [DELTA: in ms]
constexpr int64_t kNADAParamMinDeltaMs =  20; 	// Minimum value of delta for rate calculation [in ms]
constexpr int64_t kNADAParamMaxDeltaMs = 500; 	// Maximum value of delta for rate calculation [in ms]

constexpr int64_t kNADAParamQepsMs = 10; 		// Threshold for determining queueing delay build-up [QEPS: in ms]
constexpr int64_t kNADAParamQboundMs = 50;  	// Upper bound on self-inflicted queuing delay [QBOUND: in ms]
constexpr int64_t kNADAParamDfiltMs = 120;  	// Bound on filtering delay [DFILT: in ms]
constexpr float   kNADAParamGammaMax =0.2;  	// Upper bound on rate increase ratio for accelerated ramp-up
                                            	// [GAMMA_MAX: dimensionless]
// constexpr float   kNADAParamPlrSmooth = 0.1;  	// Smoothing factor for EMA of packet loss ratio


constexpr float    kNADAParamQthMs =50.;    	// Delay threshold for invoking non-linear warping 
												// [QTH: in ms]
constexpr float    kNADAParamLambda =0.5;   	// Scaling parameter in the exponent of non-linear warping
												// [LAMBDA: dimensionless]
constexpr float    kNADAParamPlrRef =0.01;  	// Reference packet loss ratio [PLRREF: dimensionless]
constexpr float    kNADAParamDLossMs =10.;  	// Reference delay penalty for loss when packet loss ratio
												// is at PLRREF [DLOSS: dimensionless]
}

NadaCore::NadaCore()
    : delta_(kNADAParamDeltaMs),
      nada_x_curr_(kNADAParamXDefault),
      nada_x_prev_(kNADAParamXDefault) {}

NadaCore::~NadaCore() {}

void NadaCore::TestFunction (const char * from) const {
    printf("[%s]: TestFunction__TestFunction__TestFunction__TestFunction__TestFunction__TestFunction\n", from);

      RTC_LOG(LS_INFO) << "Initializing the NADA BW Estimation Module for " <<
      						from << "  mode" << std::endl ;


}

/*
 * Update feedback interval 
 */
void NadaCore::UpdateDelta(int64_t delta) {

	// triggered by feedback update
    delta_ = delta;
    if (delta_ < kNADAParamMinDeltaMs) delta_ = kNADAParamMinDeltaMs;
    if (delta_ > kNADAParamMaxDeltaMs) delta_ = kNADAParamMaxDeltaMs;
}


/*
 * Update congestion signal (x_curr) as relative RTT
 */

void NadaCore::UpdateCongestion(int64_t val) {

  nada_x_curr_ = float(val);

}


void NadaCore::UpdateCongestion(double dq, double plr, int nloss) {

  // non-linear delay warping
  double d_tilde = dq;
  if (nloss > 0)  {
    if (dq > kNADAParamQthMs)  {
     double beta = kNADAParamLambda*(dq-kNADAParamQthMs)/kNADAParamQthMs;
     d_tilde = kNADAParamQthMs*exp(-beta);
   }
  }

  nada_x_curr_ = d_tilde +  kNADAParamDLossMs * (plr/kNADAParamPlrRef)*(plr/kNADAParamPlrRef);

}

float NadaCore::GetCongestion() {
	return nada_x_curr_; 
}

float NadaCore::GetDmin() {
	return dmin_history_.front().second;
}


/////////// Core NADA Calculations /////////////
/*
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
 */
int NadaCore::GetRampUpMode(int64_t rtt_min) {

    uint32_t rate_min = min_bitrate_history_.front().second;
    int64_t rtt_max = max_rtt_history_.front().second;
    float plr_max = max_plr_history_.front().second;


    RTC_LOG(LS_VERBOSE) << "NADA getRampUpMode: rmin = " << rate_min/1000
                        << " Kbps, rtt_min = " << rtt_min
                        << " ms, rtt_max = "   << rtt_max
                        << " ms, plr_max = "   << plr_max * 100. << std::endl;

    if (plr_max > 0.)	  return 1;  // loss exists, gradual-update

    if (rtt_max - rtt_min > kNADAParamQepsMs) return 1;

    return 0;
}

int NadaCore::GetRampUpMode() {

	float d_base = dmin_history_.front().second; 
    float del_max = max_del_history_.front().second;
    float plr_max = max_plr_history_.front().second;

    uint32_t rate_min = min_bitrate_history_.front().second;

    RTC_LOG(LS_VERBOSE) << "NADA getRampUpMode: rmin = " << rate_min/1000
                    << " Kbps, del_max = " << del_max
                    << " ms, plr_max = "   << plr_max * 100. << std::endl;

    if (plr_max > 0.)     return 1;  // loss exists, gradual-update

    if (del_max - d_base > kNADAParamQepsMs) return 1;

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
uint32_t NadaCore::AcceleratedRampUp(const int64_t now_ms, 
									const int64_t rtt_curr, 
									uint32_t rate_curr) {

    float rtt = float(rtt_curr);

    float gamma = kNADAParamQboundMs /(rtt + kNADAParamDeltaMs + kNADAParamDfiltMs);

    if (gamma > kNADAParamGammaMax) gamma = kNADAParamGammaMax;

  	RTC_LOG(LS_VERBOSE) << "NADA AcceleratedRampUp "
                      	<< "| ramp-up ratio gamma=" << gamma
                      	<< ", r_curr=" << rate_curr/1000 << " Kbps" << std::endl; 


    rate_curr = (1+gamma)*rate_curr;

    return rate_curr; 
}

// void NadaOwdBwe::AcceleratedRampUp(const int64_t now_ms) {

//   // calculate multiplicative ramp-up ratio
//   float gamma = kNadaBweParamQboundMs /(nada_rtt_in_ms_ + kNadaBweParamDeltaMs + kNadaBweParamDfiltMs);
  
//   if (gamma > kNadaBweParamGammaMax) gamma = kNadaBweParamGammaMax;
  
//   printf("\t NadaOwdBwe::AcceleratedRampUp:  ramp-up ratio gamma = %4.2f, r_curr = %6.2f Kbps\n", gamma, nada_rate_in_bps_/1000.);

//   RTC_LOG(LS_VERBOSE) << "NADA AcceleratedRampUp "
//                       << "| ramp-up ratio gamma=" << gamma
//                       << ", r_curr=" << nada_rate_in_bps_/1000. << " Kbps" << std::endl; 

//   nada_rate_in_bps_ = (1+gamma)*nada_rate_in_bps_;
// }

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
uint32_t NadaCore::GradualRateUpdate(const int64_t now_ms, 
									 const uint32_t rate_max, 
									 uint32_t rate_curr) {

    double x_ratio = float(rate_max)/float(rate_curr);
    double x_target = kNADAParamPrio * kNADAParamXref * x_ratio;
    double x_offset = nada_x_curr_ - x_target;
    double x_diff = nada_x_curr_ - nada_x_prev_;

    // cache current observation as x_prev
    nada_x_prev_ = nada_x_curr_;

    // calculate updated rate per equations above
    double w1 = float(delta_)/kNADAParamTau;
    w1 = w1*x_offset/kNADAParamTau;

    double w2 = kNADAParamEta*x_diff/kNADAParamTau;

    rate_curr = rate_curr*(1-kNADAParamKappa*(w1+w2));
    return rate_curr; 
}


// /*
//  * https://tools.ietf.org/html/draft-ietf-rmcat-nada-10
//  *
//  *
//  * In gradual update mode, the rate r_ref is updated as:
//  *
//  *
//  * x_offset = x_curr - PRIO*XREF*RMAX/r_ref          (5)
//  *
//  * x_diff   = x_curr - x_prev                        (6)
//  *
//  *                       delta    x_offset
//  * r_ref = r_ref - KAPPA*-------*------------*r_ref
//  *                        TAU       TAU
//  *
//  *                             x_diff
//  *               - KAPPA*ETA*---------*r_ref         (7)
//  *                             TAU
//  *
//  */
// void NadaOwdBwe::GradualRateUpdate(const int64_t now_ms) {

//     double x_ratio = float(nada_rmax_in_bps_)/float(nada_rate_in_bps_);
//     double x_target = kNadaBweParamPrio * kNadaBweParamXref * x_ratio;
//     double x_offset = nada_x_curr_ - x_target;
//     double x_diff = nada_x_curr_ - nada_x_prev_;


//     // calculate updated rate per equations above
//     double delta0 = nada_delta_;
//     if (delta0 > kNadaBweParamMaxDeltaMs)  delta0 = kNadaBweParamMaxDeltaMs;

//     double w1 = delta0/kNadaBweParamTau;
//     w1 = w1*x_offset/kNadaBweParamTau;

//     double w2 = kNadaBweParamEta*x_diff/kNadaBweParamTau;

//     // avoid numerical "overflow" with uint type
//     if (kNadaBweParamKappa*(w1+w2) < 1.0) {
//         nada_rate_in_bps_ = nada_rate_in_bps_*(1-kNadaBweParamKappa*(w1+w2));
//     } else {
//         nada_rate_in_bps_ = 0;
//     }

//     printf("\t NadaOwdBwe::GradualRateUpdate:  x_curr = %6.2f ms | r_curr = %6.2f Kbps\n", nada_x_curr_, nada_rate_in_bps_/1000.);


//     RTC_LOG(LS_VERBOSE) << "NADA GradualRateUpdate "
//                         << "| x_curr=" << nada_x_curr_ << " ms " 
//                         << "| r_curr=" << nada_rate_in_bps_/1000. << " Kbps" << std::endl; 

//     // cache current observation as x_prev
//     nada_x_prev_ = nada_x_curr_;
// }




/////////// Auxiliary Functions /////////////

/*
 *
 * Update history records for min value of bitrate
 *
 */

void NadaCore::ClearRminHistory() {
	min_bitrate_history_.clear(); 
}

void NadaCore::UpdateRminHistory(int64_t now_ms, uint32_t rate_curr) {

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
         rate_curr <= min_bitrate_history_.back().second) {
    min_bitrate_history_.pop_back();
  }

  min_bitrate_history_.push_back(std::make_pair(now_ms, rate_curr));
}




/*
 *
 * Update history records for max value of RTT
 *
 */
void NadaCore::UpdateRttHistory(int64_t now_ms, int64_t rtt) {

  // Remove expired data points from history.
  while (!max_rtt_history_.empty() &&
         now_ms - max_rtt_history_.front().first > kNADAParamLogwinMs) {

      max_rtt_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current RTT before pushing the current RTT.
  while (!max_rtt_history_.empty() &&
         rtt >= max_rtt_history_.back().second) {
    max_rtt_history_.pop_back();
  }

  max_rtt_history_.push_back(std::make_pair(now_ms, rtt));
}



// maintain long-term of recent pkt delay (dfwd) records
void NadaCore::UpdateDminHistory(int64_t now_ms, float dtmp) {

  // Remove expired data points from history.
  while (!dmin_history_.empty() &&
         now_ms - dmin_history_.front().first > kNADAParamLogwinMs2) {
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
void NadaCore::UpdateDelHistory(int64_t now_ms, float dfwd) {

  // Remove expired data points from history.
  while (!max_del_history_.empty() &&
         now_ms - max_del_history_.front().first > kNADAParamLogwinMs) {
    max_del_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current delay value before pushing the current delay value.
  while (!max_del_history_.empty() &&
         dfwd >= max_del_history_.back().second) {
    max_del_history_.pop_back();
  }

  max_del_history_.push_back(std::make_pair(now_ms, dfwd));
}


/*
 *
 * Update history records for max value of PLR
 *
 */
void NadaCore::UpdatePlrHistory(int64_t now_ms, float plr) {

  // Remove expired data points from history.
  while (!max_plr_history_.empty() &&
         now_ms - max_plr_history_.front().first > kNADAParamLogwinMs) {

    max_plr_history_.pop_front();
  }

  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current PLR before pushing the current PLR
  while (!max_plr_history_.empty() &&
         plr >= max_plr_history_.back().second) {
    max_plr_history_.pop_back();
  }

  max_plr_history_.push_back(std::make_pair(now_ms, plr));
}



}  // namespace webrtc
