/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 *
 *  FEC and NACK added bitrate is handled outside class
 */

#ifndef MODULES_BITRATE_CONTROLLER_SEND_SIDE_BANDWIDTH_ESTIMATION_INTERFACE_H_
#define MODULES_BITRATE_CONTROLLER_SEND_SIDE_BANDWIDTH_ESTIMATION_INTERFACE_H_

namespace webrtc {

class RtcEventLog;

class SendSideBandwidthEstimationInterface {
 public:
  SendSideBandwidthEstimationInterface();
  virtual ~SendSideBandwidthEstimationInterface();

  virtual void CurrentEstimate(int* bitrate, uint8_t* loss, int64_t* rtt) const = 0;

  // Call periodically to update estimate.
  virtual void UpdateEstimate(int64_t now_ms) = 0;

  // Call when we receive a RTCP message with TMMBR or REMB.
  virtual void UpdateReceiverEstimate(int64_t now_ms, uint32_t bandwidth) = 0;

  // Call when a new delay-based estimate is available.
  virtual void UpdateDelayBasedEstimate(int64_t now_ms, uint32_t bitrate_bps) = 0;

  // Call when we receive a RTCP message with a ReceiveBlock.
  virtual void UpdateReceiverBlock(uint8_t fraction_loss,
                                   int64_t rtt,
                                   int number_of_packets,
                                   int64_t now_ms) = 0;

  virtual void SetBitrates(int send_bitrate,
                           int min_bitrate,
                           int max_bitrate) = 0;
  virtual void SetSendBitrate(int bitrate) = 0;
  virtual void SetMinMaxBitrate(int min_bitrate, int max_bitrate) = 0;
  virtual int GetMinBitrate() const = 0;
};
}  // namespace webrtc
#endif  // MODULES_BITRATE_CONTROLLER_SEND_SIDE_BANDWIDTH_ESTIMATION_INTERFACE_H_
