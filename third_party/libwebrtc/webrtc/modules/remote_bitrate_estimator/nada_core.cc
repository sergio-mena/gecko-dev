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
  
constexpr int kNADAParamRateBps =  600000;  // Default rate: 600Kbps
constexpr int kNADAParamRminBps =  300000;  // Min rate: 300Kbps
constexpr int kNADAParamRmaxBps = 3000000;  // Max rate: 3Mbps

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

constexpr int64_t kNADAParamDeltaMs = 100; 		 // Target interval for feedback and/or rate update [DELTA: in ms]
constexpr int64_t kNADAParamMinDeltaMs =  20; 	// Minimum value of delta for rate calculation [in ms]
constexpr int64_t kNADAParamMaxDeltaMs = 500; 	// Maximum value of delta for rate calculation [in ms]

constexpr int64_t kNADAParamQepsMs = 10; 		   // Threshold for determining queueing delay build-up [QEPS: in ms]
constexpr int64_t kNADAParamQboundMs = 50;  	// Upper bound on self-inflicted queuing delay [QBOUND: in ms]
constexpr int64_t kNADAParamDfiltMs = 120;  	// Bound on filtering delay [DFILT: in ms]
constexpr float   kNADAParamGammaMax =0.2;  	// Upper bound on rate increase ratio for accelerated ramp-up
                                            	// [GAMMA_MAX: dimensionless]
constexpr float   kNADAParamPlrSmooth = 0.1;  	// Smoothing factor for EMA of packet loss ratio


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
      nada_dfwd_(0.),
 	    nada_dq_(0.),  
 	    nada_rtt_(0),  
 	    nada_relrtt_(0), 
 	    nada_nloss_(0), 
 	    nada_plr_(0.), 
 	    nada_rrate_(0.), 
      nada_x_curr_(kNADAParamXDefault),
      nada_x_prev_(kNADAParamXDefault), 
      nada_rmode_(NADA_RMODE_ARAMPUP),  
      nada_rate_in_bps_(kNADAParamRateBps),
      nada_rmin_in_bps_(kNADAParamRminBps),
      nada_rmax_in_bps_(kNADAParamRmaxBps)  {}

NadaCore::~NadaCore() {}

void NadaCore::TestFunction (const char * from) const {
	
    printf("[%s]: TestFunction__TestFunction__TestFunction__TestFunction__TestFunction__TestFunction\n", from);

      RTC_LOG(LS_INFO) << "Initializing the NADA BW Estimation Module for " <<
      						from << "  mode" << std::endl ;

}

int NadaCore::GetMinBitrate() const {

	return nada_rmin_in_bps_; 
}

/* 
 * Update delay/loss states per feedback interval 
 */
void NadaCore::UpdateDelStats(int64_t now_ms, 
                              int64_t dfwd) {

    UpdateDelHistory(now_ms, dfwd); 

    float d_base = min_del_history_.front().second; 
    nada_dfwd_ = dfwd; 
    nada_dq_ = dfwd - d_base; 
}

void NadaCore::UpdateRttStats(int64_t now_ms, 
                              int64_t rtt) {

    UpdateRttHistory(now_ms, rtt); 

    int64_t rtt_base = min_rtt_history_.front().second; 
    nada_rtt_ = rtt; 
    nada_relrtt_ = rtt = rtt_base; 
}

void NadaCore::UpdatePlrStats(int64_t now_ms, 
                              int nloss, 
                              int npkts) {

  nada_nloss_ = nloss; 
  float tmpplr = float(nloss)/(float(npkts+nloss));
  nada_plr_ += kNADAParamPlrSmooth * (tmpplr - nada_plr_);  // exponential smoothing

  UpdatePlrHistory(now_ms, nada_plr_); 

}

void NadaCore::SetRecvRate(const uint32_t rrate) {

	nada_rrate_ = float(rrate); 
}

void NadaCore::CalcRecvRate(uint64_t curr_ts, 
  					   		          uint64_t last_ts, 
  					   		          int nbytes) {

  if (last_ts > 0) {
      uint64_t dt = curr_ts - last_ts;
      nada_rrate_ = float(nbytes) * 8000. / float(dt);

      RTC_LOG(LS_INFO) << "Updating RecvRate: dt: " << dt 
                        << ", nbytes: " << nbytes  
                        << ", rrate: " << nada_rrate_ /1000
                        << " Kbps"<<  std::endl;
  }
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
void NadaCore::UpdateRttCongestion() {

  nada_x_curr_ = float(nada_relrtt_); 
}

/*
 * Update congestion signal (x_curr) as the composite of 
 * warped relative one-way delay (d_tilde) and loss penalty (d_loss)
 */
void NadaCore::UpdateOwdCongestion() {

    RTC_LOG(LS_INFO) << "Updating Congestion" << std::endl;
    printf("[DEBUG] UpdateCongestion\n");


  // non-linear delay warping
  double d_tilde = nada_dq_;
  if (nada_nloss_ > 0)  {

    if (nada_dq_ > kNADAParamQthMs)  {

     double beta = kNADAParamLambda*(nada_dq_-kNADAParamQthMs)/kNADAParamQthMs;
     d_tilde = kNADAParamQthMs*exp(-beta);
   
   }
  
  }

  // add loss-based penalty
  double relplr = nada_plr_/kNADAParamPlrRef;  // relative plr
  nada_x_curr_ = d_tilde +  kNADAParamDLossMs * relplr * relplr;

}


/////////// Core NADA Calculations /////////////
/*
 * Implementation of core NADA congestion control algorithm
 *
 * https://tools.ietf.org/html/draft-ietf-rmcat-nada-09
 *
 */
int NadaCore::UpdateNadaRate(const int64_t now_ms, 
                             const bool use_rtt) {

  // update composite congestion signal x_curr
  if (use_rtt)
    UpdateRttCongestion(); 
  else 
    UpdateOwdCongestion(); 
  
  // obtain rate update mode (ramp-up vs. gradual update)
  GetRampUpMode(use_rtt); 
  if (nada_rmode_ == NADA_RMODE_ARAMPUP)
    AcceleratedRampUp(now_ms); 
  else
    GradualRateUpdate(now_ms); 

  // clip to min/max range
  ClipBitrate();

  return nada_rate_in_bps_; 
}

void NadaCore::GetRampUpMode(const bool use_rtt) {


  int drel = 0; 
  if (use_rtt) {
    int64_t rtt_base = min_rtt_history_.front().second;
    int64_t rtt_max = max_rtt_history_.front().second;
    drel = rtt_max - rtt_base; 
  } else {
    float d_base = min_del_history_.front().second; 
    float d_max = max_del_history_.front().second;
    drel = d_max - d_base; 
  }
  
  float plr_max = max_plr_history_.front().second;

  RTC_LOG(LS_INFO) << "[DEBUG] NADA GetRampUpMode: " 
                     << " drel = " << drel
                     << " ms, plr_max = "   << plr_max * 100. << std::endl;

  if (plr_max > 0.)     
    nada_rmode_ = NADA_RMODE_GUPDATE;   // loss exists, gradual-update
  else if (drel > kNADAParamQepsMs) 
    nada_rmode_ = NADA_RMODE_GUPDATE; 
  else
    nada_rmode_ = NADA_RMODE_ARAMPUP; 

  printf("DEBUG] NADA GetRampUpMode: rmode = %d\n", nada_rmode_);
  fflush(stdout);

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

void NadaCore::AcceleratedRampUp(const int64_t now_ms) {

    float gamma = kNADAParamQboundMs /float(nada_rtt_ + kNADAParamDeltaMs + kNADAParamDfiltMs);

    if (gamma > kNADAParamGammaMax) gamma = kNADAParamGammaMax;
    nada_rate_in_bps_ = (1+gamma)*nada_rate_in_bps_;

  	RTC_LOG(LS_INFO) << "[DEBUG] NADA AcceleratedRampUp "
                      	<< "| ramp-up ratio gamma=" << gamma
                      	<< ", rate =" << nada_rate_in_bps_/1000 << " Kbps" << std::endl; 


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
void NadaCore::GradualRateUpdate(const int64_t now_ms) {

    double x_ratio = float(nada_rmax_in_bps_)/float(nada_rate_in_bps_);
    double x_target = kNADAParamPrio * kNADAParamXref * x_ratio;
    double x_offset = nada_x_curr_ - x_target;
    double x_diff = nada_x_curr_ - nada_x_prev_;

    // cache current observation as x_prev
    nada_x_prev_ = nada_x_curr_;

    // calculate updated rate per equations above
    double w1 = float(delta_)/kNADAParamTau;
    w1 = w1*x_offset/kNADAParamTau;

    double w2 = kNADAParamEta*x_diff/kNADAParamTau;

  // avoid numerical "overflow" with uint type
    double tmp_ratio = kNADAParamKappa*(w1+w2); 
    if ( tmp_ratio < 1.0) {
        nada_rate_in_bps_ = nada_rate_in_bps_*(1-tmp_ratio);
    } else {

        RTC_LOG(LS_INFO) << "WARNING: avoided potential overflow in GradualRateUpdate: "
                         << "tmp_ratio = " << tmp_ratio << std::endl; 

        nada_rate_in_bps_ = 0;
    }

    RTC_LOG(LS_INFO) << "[DEBUG] NADA GradualRateUpdate "
                        << "| x_curr=" << nada_x_curr_ << " ms " 
                        << "| r_curr=" << nada_rate_in_bps_/1000. << " Kbps" << std::endl; 

    // return rate_curr; 
}

void NadaCore::ClipBitrate() {

  int bitrate = nada_rate_in_bps_; // save pre-clipping rate 

  if (nada_rate_in_bps_ < nada_rmin_in_bps_)
    nada_rate_in_bps_ = nada_rmin_in_bps_;

  if (nada_rate_in_bps_ > nada_rmax_in_bps_) {
    nada_rate_in_bps_ = nada_rmax_in_bps_;
  }

  RTC_LOG(LS_INFO) << "[DEBUG] NADA ClipBitrate "
                         << "| rate_in  =" << bitrate/1000 << " Kbps" 
                         << "| rate_out =" << nada_rate_in_bps_/1000. << " Kbps" << std::endl; 

  // return nada_rate_in_bps_; 
}

/////////// Auxiliary Functions /////////////
void NadaCore::LogUpdate(const char * algo, 
						             const int64_t ts) {

  printf("NADA Update | algo: %s | ts = %lld, fbint = %lld ms, rmode = %d, xcurr = %4.2f, rate = %6d Kbps\n",
            algo, ts, delta_, nada_rmode_, nada_x_curr_, nada_rate_in_bps_/1000);

  std::ostringstream os;
  os << std::fixed;
  os.precision(2);

  RTC_LOG(LS_INFO) << " NADA Update | algo: "  << algo 					      // 1) CC algorithm flavor
               << " | ts: "     << ts << " ms"     						         // 2) relative timestamp (now - t0)
               << " | fbint: "  << delta_ << " ms"                   	  // 3) feedback interval
               << " | qdel: "   << nada_dq_ << " ms"                 	  // 4) queuing delay
               << " | dfwd: " 	<< nada_dfwd_ << " ms"						  // 5) one-way delay 
               << " | relrtt: " << nada_relrtt_ << " ms"                 // 6) relative RTT
               << " | rtt: "    << nada_rtt_ << " ms"               			  // 7) RTT
               << " | nloss: "  << nada_nloss_                                  // 8) packet loss count
               << " | plr: "     << std::fixed << nada_plr_*100.  << " %"  // 9) temporally smoothed packet loss ratio
               << " | rmode: "   << nada_rmode_                                  // 10) rate update mode: accelerated ramp-up or gradual
               << " | xcurr: "   << std::fixed << nada_x_curr_ << " ms"   // 11) aggregated congestion signal
               << " | rrate: "   << nada_rrate_ / 1000. << " Kbps"               // 12) receiving rate
               << " | srate: "   << nada_rate_in_bps_ / 1000. << " Kbps"   // 13) sending rate
               << " | rmin: "    << nada_rmin_in_bps_ / 1000. << " Kbps"  // 14) minimum rate
               << " | rmax: "    << nada_rmax_in_bps_ / 1000. << " Kbps"  // 15) maximum rate
               << std::endl;
}

void NadaCore::SetMinMaxBitrate(int min_bitrate, 
								                int max_bitrate) {

  nada_rmin_in_bps_ = std::max(min_bitrate, 0); 

  if (max_bitrate > 0) {
    nada_rmax_in_bps_ = std::max(nada_rmin_in_bps_, max_bitrate);
  } else {
    nada_rmax_in_bps_ = kNADAParamRmaxBps; // stay with default Rmax
  }

  RTC_LOG(LS_INFO)  << "NADA SetMinMaxBitrate: updating min/max rate:"
                    << " rmin = " << min_bitrate/1000
                    << " => " << nada_rmin_in_bps_/1000 << " Kbps"
                    << " rmax = " << max_bitrate/1000
                    << " => " << nada_rmax_in_bps_/1000 << " Kbps"
                    << std::endl;
}

/*
 *
 * Update RTT history records:
 * -- max_rtt_history_:  short-term max values
 * -- min_rtt_history_:  long-term min (baseline) values
 *
 */
void NadaCore::UpdateRttHistory(int64_t now_ms, int64_t rtt) {

  // Remove expired data points from max_rtt history history.
  while (!max_rtt_history_.empty() &&
         now_ms - max_rtt_history_.front().first > kNADAParamLogwinMs) {
      max_rtt_history_.pop_front();
  }

  // Remove expired data points from min_rtt history history.
  while (!min_rtt_history_.empty() &&
         now_ms - min_rtt_history_.front().first > kNADAParamLogwinMs2) {
      min_rtt_history_.pop_front();
  }

 
  // Sliding-window algorithm for logging short-term maximum values: 
  // Pop values lower than current RTT before pushing the current RTT.
  while (!max_rtt_history_.empty() &&
         rtt >= max_rtt_history_.back().second) {
    max_rtt_history_.pop_back();
  }
  max_rtt_history_.push_back(std::make_pair(now_ms, rtt));


  // Sliding-window algorithm for logging long-term minimum/baseline values:
  // Pop values lower than current RTT before pushing the current RTT.
  while (!min_rtt_history_.empty() &&
         rtt <= min_rtt_history_.back().second) {
    min_rtt_history_.pop_back();
  }
  min_rtt_history_.push_back(std::make_pair(now_ms, rtt));

}


/*
 *
 * Update OWD history records:
 * -- max_del_history_:  short-term max values
 * -- min_del_history_:  long-term min (baseline) values
 *
 */
void NadaCore::UpdateDelHistory(int64_t now_ms, float dfwd) {

  // Remove expired data points from maximum delay (max_del) history.
  while (!max_del_history_.empty() &&
         now_ms - max_del_history_.front().first > kNADAParamLogwinMs) {
    max_del_history_.pop_front();
  }

  // Remove expired data points from baseline delay (min_del) history 
  while (!min_del_history_.empty() &&
         now_ms - min_del_history_.front().first > kNADAParamLogwinMs2) {
    min_del_history_.pop_front();
  }


  // Typical sliding-window algorithm for logging maximum values:
  // Pop values lower than current delay value before pushing the current delay value.
  while (!max_del_history_.empty() &&
         dfwd >= max_del_history_.back().second) {
    max_del_history_.pop_back();
  }
  max_del_history_.push_back(std::make_pair(now_ms, dfwd));

  // Typical sliding-window algorithm for logging minimum values:
  // Pop values higher than current delay value before pushing the current delay value.
  while (!min_del_history_.empty() &&
         dfwd < min_del_history_.back().second) {
    min_del_history_.pop_back();
  }
  min_del_history_.push_back(std::make_pair(now_ms, dfwd));
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
