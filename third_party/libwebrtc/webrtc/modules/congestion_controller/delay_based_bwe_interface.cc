/*
 *  Copyright (c) 2019 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "modules/congestion_controller/delay_based_bwe_interface.h"

namespace webrtc {

DelayBasedBweInterface::Result::Result()
    : updated(false),
      probe(false),
      target_bitrate_bps(0),
      recovered_from_overuse(false) {}

DelayBasedBweInterface::Result::Result(bool probe, uint32_t target_bitrate_bps)
    : updated(true),
      probe(probe),
      target_bitrate_bps(target_bitrate_bps),
      recovered_from_overuse(false) {}

DelayBasedBweInterface::Result::~Result() {}

DelayBasedBweInterface::DelayBasedBweInterface() {}

DelayBasedBweInterface::~DelayBasedBweInterface() {}

}  // namespace webrtc
