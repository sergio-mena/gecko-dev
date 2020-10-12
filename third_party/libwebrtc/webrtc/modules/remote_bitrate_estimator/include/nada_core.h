/*
 *  Copyright (c) 2020 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_
#define MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_

namespace webrtc {

class NadaCore {
 public:
    NadaCore();
    virtual ~NadaCore();

    void TestFunction(const char * from) const;
};

}  // namespace webrtc

#endif  // MODULES_REMOTE_BITRATE_ESTIMATOR_INCLUDE_NADA_CORE_H_
