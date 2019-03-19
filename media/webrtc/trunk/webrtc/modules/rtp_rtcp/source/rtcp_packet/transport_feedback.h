/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef WEBRTC_MODULES_RTP_RTCP_SOURCE_RTCP_PACKET_TRANSPORT_FEEDBACK_H_
#define WEBRTC_MODULES_RTP_RTCP_SOURCE_RTCP_PACKET_TRANSPORT_FEEDBACK_H_

#include <memory>
#include <vector>
#include <map>

#include "webrtc/base/constructormagic.h"
#include "webrtc/modules/rtp_rtcp/source/rtcp_packet/rtpfb.h"
#include "webrtc/modules/rtp_rtcp/include/rtp_rtcp_defines.h"

namespace webrtc {
namespace rtcp {
class CommonHeader;

class TransportFeedback : public Rtpfb {
public:
  ~TransportFeedback() override;

  virtual void SetBase(uint16_t base_sequence,         // Seq# of first packet in this msg.
                       int64_t ref_timestamp_us) = 0;  // Reference timestamp for this msg.
  virtual void SetFeedbackSequenceNumber(uint8_t feedback_sequence) = 0;
  virtual int64_t GetBaseTimeUs() const = 0;
  virtual bool AddReceivedPacket(uint16_t sequence_number, int64_t timestamp_us) = 0;
  virtual bool IsConsistent() const = 0;
  virtual bool Parse(const CommonHeader& packet) = 0;
  virtual std::vector<PacketInfo> GetFeedbackVector(int64_t base_offset_ms) const = 0;
  virtual uint16_t GetBaseSequence() const;

protected:
  TransportFeedback(size_t site_bytes);
  size_t BlockLength() const override;

  size_t size_bytes_;
  uint16_t base_seq_no_;
  int64_t last_timestamp_us_;
};

class TransportCCFeedback : public TransportFeedback {
 public:
  // TODO(sprang): IANA reg?
  static constexpr uint8_t kFeedbackMessageType = 15;
  // Convert to multiples of 0.25ms.
  static constexpr int kDeltaScaleFactor = 250;
  // Maximum number of packets (including missing) TransportFeedback can report.
  static constexpr size_t kMaxReportedPackets = 0xffff;

  TransportCCFeedback();
  ~TransportCCFeedback() override;

  void SetBase(uint16_t base_sequence,              // Seq# of first packet in this msg.
               int64_t ref_timestamp_us) override;  // Reference timestamp for this msg.
  void SetFeedbackSequenceNumber(uint8_t feedback_sequence) override;
  // NOTE: This method requires increasing sequence numbers (excepting wraps).
  bool AddReceivedPacket(uint16_t sequence_number, int64_t timestamp_us) override;

  enum class StatusSymbol {
    kNotReceived,
    kReceivedSmallDelta,
    kReceivedLargeDelta,
  };

  std::vector<TransportCCFeedback::StatusSymbol> GetStatusVector() const;
  std::vector<int16_t> GetReceiveDeltas() const;

  // Get the reference time in microseconds, including any precision loss.
  int64_t GetBaseTimeUs() const override;
  // Convenience method for getting all deltas as microseconds. The first delta
  // is relative the base time.
  std::vector<int64_t> GetReceiveDeltasUs() const;

  bool Parse(const CommonHeader& packet) override;
  static std::unique_ptr<TransportCCFeedback> ParseFrom(const uint8_t* buffer,
                                                        size_t length);
  // Pre and postcondition for all public methods. Should always return true.
  // This function is for tests.
  bool IsConsistent() const override;
  std::vector<PacketInfo> GetFeedbackVector(int64_t base_offset_ms) const override;

 protected:
  bool Create(uint8_t* packet,
              size_t* position,
              size_t max_length,
              PacketReadyCallback* callback) const override;

 private:
  // Size in bytes of a delta time in rtcp packet.
  // Valid values are 0 (packet wasn't received), 1 or 2.
  using DeltaSize = uint8_t;
  // Keeps DeltaSizes that can be encoded into single chunk if it is last chunk.
  class LastChunk;
  struct ReceivedPacket {
    ReceivedPacket(uint16_t sequence_number, int16_t delta_ticks)
        : sequence_number(sequence_number), delta_ticks(delta_ticks) {}
    uint16_t sequence_number;
    int16_t delta_ticks;
  };

  // Reset packet to consistent empty state.
  void Clear();

  bool AddDeltaSize(DeltaSize delta_size);

  uint16_t num_seq_no_;
  int32_t base_time_ticks_;
  uint8_t feedback_seq_;

  std::vector<ReceivedPacket> packets_;
  // All but last encoded packet chunks.
  std::vector<uint16_t> encoded_chunks_;
  const std::unique_ptr<LastChunk> last_chunk_;

  RTC_DISALLOW_COPY_AND_ASSIGN(TransportCCFeedback);
};

class CcfbFeedback : public TransportFeedback {
public:
  // TODO(semena): IANA reg?
  static constexpr uint8_t kFeedbackMessageType = 14;

  class MetricBlock {
    public:
      static constexpr uint16_t m_overrange = 0x1FFE;
      static constexpr uint16_t m_unavailable = 0x1FFF;
      uint8_t m_ecn;
      uint64_t m_timestampUs;
      uint16_t m_ato;
    };

  typedef std::map<uint16_t /* sequence */, MetricBlock> ReportBlock_t;

  CcfbFeedback();
  ~CcfbFeedback() override;

  void SetBase(uint16_t base_sequence,              // Seq# of first packet in this msg.
               int64_t ref_timestamp_us) override;  // Reference timestamp for this msg.
  void SetFeedbackSequenceNumber(uint8_t feedback_sequence) override;
  bool AddReceivedPacket(uint16_t sequence_number, int64_t timestamp_us) override;

  int64_t GetBaseTimeUs() const override;

  bool Parse(const CommonHeader& packet) override;
  static std::unique_ptr<CcfbFeedback> ParseFrom(const uint8_t* buffer,
                                                         size_t length);
  bool IsConsistent() const override;
  std::vector<PacketInfo> GetFeedbackVector(int64_t base_offset_ms) const override;

protected:
  bool Create(uint8_t* packet,
              size_t* position,
              size_t max_length,
              PacketReadyCallback* callback) const override;

 private:
  void Clear();
  static std::pair<uint16_t, uint16_t> CalculateBeginStopSeq(uint16_t baseSeq, const ReportBlock_t& rb);
  static uint64_t NtpToUs(uint32_t ntp);
  static uint32_t UsToNtp(uint64_t tsUs);
  static uint16_t NtpToAto(uint32_t ntp, uint32_t ntpRef);
  static uint32_t AtoToNtp(uint16_t ato, uint32_t ntpRef);
  bool UpdateLength();
  std::map<uint32_t /* SSRC */, ReportBlock_t> m_reportBlocks;

  RTC_DISALLOW_COPY_AND_ASSIGN(CcfbFeedback);
};


}  // namespace rtcp
}  // namespace webrtc
#endif  // WEBRTC_MODULES_RTP_RTCP_SOURCE_RTCP_PACKET_TRANSPORT_FEEDBACK_H_
