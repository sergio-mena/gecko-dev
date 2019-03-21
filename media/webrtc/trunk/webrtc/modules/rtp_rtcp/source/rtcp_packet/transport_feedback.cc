/*
 *  Copyright (c) 2015 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include "webrtc/modules/rtp_rtcp/source/rtcp_packet/transport_feedback.h"

#include <algorithm>
#include <cstdint>
#include <cinttypes>

#include "webrtc/base/checks.h"
#include "webrtc/base/logging.h"
#include "webrtc/modules/include/module_common_types.h"
#include "webrtc/modules/rtp_rtcp/source/byte_io.h"
#include "webrtc/modules/rtp_rtcp/source/rtcp_packet/common_header.h"

namespace webrtc {
namespace rtcp {
namespace {

// Header size:
// * 4 bytes Common RTCP Packet Header
// * 8 bytes Common Packet Format for RTCP Feedback Messages
// * 8 bytes FeedbackPacket header
constexpr size_t kTransportCCFeedbackHeaderSizeBytes = 4 + 8 + 8;
constexpr size_t kChunkSizeBytes = 2;
// TODO(sprang): Add support for dynamic max size for easier fragmentation,
// eg. set it to what's left in the buffer or IP_PACKET_SIZE.
// Size constraint imposed by RTCP common header: 16bit size field interpreted
// as number of four byte words minus the first header word.
constexpr size_t kMaxSizeBytes = (1 << 16) * 4;
// Payload size:
// * 8 bytes Common Packet Format for RTCP Feedback Messages
// * 8 bytes FeedbackPacket header.
// * 2 bytes for one chunk.
constexpr size_t kMinPayloadSizeBytes = 8 + 8 + 2;
constexpr size_t kBaseScaleFactor =
    TransportCCFeedback::kDeltaScaleFactor * (1 << 8);
constexpr int64_t kTimeWrapPeriodUs = (1ll << 24) * kBaseScaleFactor;
}  // namespace

TransportFeedback::TransportFeedback(size_t size_bytes)
    : Rtpfb(),
      size_bytes_(size_bytes),
      base_seq_no_(0),
      last_timestamp_us_(0) {}

TransportFeedback::~TransportFeedback() {}

size_t TransportFeedback::BlockLength() const {
  // Round size_bytes_ up to multiple of 32bits.
  return (size_bytes_ + 3) & (~static_cast<size_t>(3));
}

uint16_t TransportFeedback::GetBaseSequence() const {
  return base_seq_no_;
}


//    Message format
//
//     0                   1                   2                   3
//     0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |V=2|P|  FMT=15 |    PT=205     |           length              |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  0 |                     SSRC of packet sender                     |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  4 |                      SSRC of media source                     |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  8 |      base sequence number     |      packet status count      |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// 12 |                 reference time                | fb pkt. count |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// 16 |          packet chunk         |         packet chunk          |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    .                                                               .
//    .                                                               .
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |         packet chunk          |  recv delta   |  recv delta   |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    .                                                               .
//    .                                                               .
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//    |           recv delta          |  recv delta   | zero padding  |
//    +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

constexpr uint8_t TransportCCFeedback::kFeedbackMessageType;
constexpr size_t TransportCCFeedback::kMaxReportedPackets;

// Keep delta_sizes that can be encoded into single chunk if it is last chunk.
class TransportCCFeedback::LastChunk {
 public:
  using DeltaSize = TransportCCFeedback::DeltaSize;

  LastChunk();

  bool Empty() const;
  void Clear();
  // Return if delta sizes still can be encoded into single chunk with added
  // |delta_size|.
  bool CanAdd(DeltaSize delta_size) const;
  // Add |delta_size|, assumes |CanAdd(delta_size)|,
  void Add(DeltaSize delta_size);

  // Encode chunk as large as possible removing encoded delta sizes.
  // Assume CanAdd() == false for some valid delta_size.
  uint16_t Emit();
  // Encode all stored delta_sizes into single chunk, pad with 0s if needed.
  uint16_t EncodeLast() const;

  // Decode up to |max_size| delta sizes from |chunk|.
  void Decode(uint16_t chunk, size_t max_size);
  // Appends content of the Lastchunk to |deltas|.
  void AppendTo(std::vector<DeltaSize>* deltas) const;

 private:
  static constexpr size_t kMaxRunLengthCapacity = 0x1fff;
  static constexpr size_t kMaxOneBitCapacity = 14;
  static constexpr size_t kMaxTwoBitCapacity = 7;
  static constexpr size_t kMaxVectorCapacity = kMaxOneBitCapacity;
  static constexpr DeltaSize kLarge = 2;

  uint16_t EncodeOneBit() const;
  void DecodeOneBit(uint16_t chunk, size_t max_size);

  uint16_t EncodeTwoBit(size_t size) const;
  void DecodeTwoBit(uint16_t chunk, size_t max_size);

  uint16_t EncodeRunLength() const;
  void DecodeRunLength(uint16_t chunk, size_t max_size);

  DeltaSize delta_sizes_[kMaxVectorCapacity];
  uint16_t size_;
  bool all_same_;
  bool has_large_delta_;
};
constexpr size_t TransportCCFeedback::LastChunk::kMaxRunLengthCapacity;
constexpr size_t TransportCCFeedback::LastChunk::kMaxOneBitCapacity;
constexpr size_t TransportCCFeedback::LastChunk::kMaxTwoBitCapacity;
constexpr size_t TransportCCFeedback::LastChunk::kMaxVectorCapacity;

TransportCCFeedback::LastChunk::LastChunk() {
  Clear();
}

bool TransportCCFeedback::LastChunk::Empty() const {
  return size_ == 0;
}

void TransportCCFeedback::LastChunk::Clear() {
  size_ = 0;
  all_same_ = true;
  has_large_delta_ = false;
}

bool TransportCCFeedback::LastChunk::CanAdd(DeltaSize delta_size) const {
  RTC_DCHECK_LE(delta_size, 2);
  if (size_ < kMaxTwoBitCapacity)
    return true;
  if (size_ < kMaxOneBitCapacity && !has_large_delta_ && delta_size != kLarge)
    return true;
  if (size_ < kMaxRunLengthCapacity && all_same_ &&
      delta_sizes_[0] == delta_size)
    return true;
  return false;
}

void TransportCCFeedback::LastChunk::Add(DeltaSize delta_size) {
  RTC_DCHECK(CanAdd(delta_size));
  if (size_ < kMaxVectorCapacity)
    delta_sizes_[size_] = delta_size;
  size_++;
  all_same_ = all_same_ && delta_size == delta_sizes_[0];
  has_large_delta_ = has_large_delta_ || delta_size == kLarge;
}

uint16_t TransportCCFeedback::LastChunk::Emit() {
  RTC_DCHECK(!CanAdd(0) || !CanAdd(1) || !CanAdd(2));
  if (all_same_) {
    uint16_t chunk = EncodeRunLength();
    Clear();
    return chunk;
  }
  if (size_ == kMaxOneBitCapacity) {
    uint16_t chunk = EncodeOneBit();
    Clear();
    return chunk;
  }
  RTC_DCHECK_GE(size_, kMaxTwoBitCapacity);
  uint16_t chunk = EncodeTwoBit(kMaxTwoBitCapacity);
  // Remove |kMaxTwoBitCapacity| encoded delta sizes:
  // Shift remaining delta sizes and recalculate all_same_ && has_large_delta_.
  size_ -= kMaxTwoBitCapacity;
  all_same_ = true;
  has_large_delta_ = false;
  for (size_t i = 0; i < size_; ++i) {
    DeltaSize delta_size = delta_sizes_[kMaxTwoBitCapacity + i];
    delta_sizes_[i] = delta_size;
    all_same_ = all_same_ && delta_size == delta_sizes_[0];
    has_large_delta_ = has_large_delta_ || delta_size == kLarge;
  }

  return chunk;
}

uint16_t TransportCCFeedback::LastChunk::EncodeLast() const {
  RTC_DCHECK_GT(size_, 0);
  if (all_same_)
    return EncodeRunLength();
  if (size_ <= kMaxTwoBitCapacity)
    return EncodeTwoBit(size_);
  return EncodeOneBit();
}

// Appends content of the Lastchunk to |deltas|.
void TransportCCFeedback::LastChunk::AppendTo(
    std::vector<DeltaSize>* deltas) const {
  if (all_same_) {
    deltas->insert(deltas->end(), size_, delta_sizes_[0]);
  } else {
    deltas->insert(deltas->end(), delta_sizes_, delta_sizes_ + size_);
  }
}

void TransportCCFeedback::LastChunk::Decode(uint16_t chunk, size_t max_size) {
  if ((chunk & 0x8000) == 0) {
    DecodeRunLength(chunk, max_size);
  } else if ((chunk & 0x4000) == 0) {
    DecodeOneBit(chunk, max_size);
  } else {
    DecodeTwoBit(chunk, max_size);
  }
}

//  One Bit Status Vector Chunk
//
//  0                   1
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |T|S|       symbol list         |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  T = 1
//  S = 0
//  Symbol list = 14 entries where 0 = not received, 1 = received 1-byte delta.
uint16_t TransportCCFeedback::LastChunk::EncodeOneBit() const {
  RTC_DCHECK(!has_large_delta_);
  RTC_DCHECK_LE(size_, kMaxOneBitCapacity);
  uint16_t chunk = 0x8000;
  for (size_t i = 0; i < size_; ++i)
    chunk |= delta_sizes_[i] << (kMaxOneBitCapacity - 1 - i);
  return chunk;
}

void TransportCCFeedback::LastChunk::DecodeOneBit(uint16_t chunk,
                                                size_t max_size) {
  RTC_DCHECK_EQ(chunk & 0xc000, 0x8000);
  size_ = std::min(kMaxOneBitCapacity, max_size);
  has_large_delta_ = false;
  all_same_ = false;
  for (size_t i = 0; i < size_; ++i)
    delta_sizes_[i] = (chunk >> (kMaxOneBitCapacity - 1 - i)) & 0x01;
}

//  Two Bit Status Vector Chunk
//
//  0                   1
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |T|S|       symbol list         |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  T = 1
//  S = 1
//  symbol list = 7 entries of two bits each.
uint16_t TransportCCFeedback::LastChunk::EncodeTwoBit(size_t size) const {
  RTC_DCHECK_LE(size, size_);
  uint16_t chunk = 0xc000;
  for (size_t i = 0; i < size; ++i)
    chunk |= delta_sizes_[i] << 2 * (kMaxTwoBitCapacity - 1 - i);
  return chunk;
}

void TransportCCFeedback::LastChunk::DecodeTwoBit(uint16_t chunk,
                                                size_t max_size) {
  RTC_DCHECK_EQ(chunk & 0xc000, 0xc000);
  size_ = std::min(kMaxTwoBitCapacity, max_size);
  has_large_delta_ = true;
  all_same_ = false;
  for (size_t i = 0; i < size_; ++i)
    delta_sizes_[i] = (chunk >> 2 * (kMaxTwoBitCapacity - 1 - i)) & 0x03;
}

//  Run Length Status Vector Chunk
//
//  0                   1
//  0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |T| S |       Run Length        |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  T = 0
//  S = symbol
//  Run Length = Unsigned integer denoting the run length of the symbol
uint16_t TransportCCFeedback::LastChunk::EncodeRunLength() const {
  RTC_DCHECK(all_same_);
  RTC_DCHECK_LE(size_, kMaxRunLengthCapacity);
  return (delta_sizes_[0] << 13) | size_;
}

void TransportCCFeedback::LastChunk::DecodeRunLength(uint16_t chunk,
                                                   size_t max_count) {
  RTC_DCHECK_EQ(chunk & 0x8000, 0);
  size_ = std::min<size_t>(chunk & 0x1fff, max_count);
  size_t delta_size = (chunk >> 13) & 0x03;
  has_large_delta_ = delta_size >= kLarge;
  all_same_ = true;
  // To make it consistent with Add function, populate delta_sizes_ beyound 1st.
  for (size_t i = 0; i < std::min<size_t>(size_, kMaxVectorCapacity); ++i)
    delta_sizes_[i] = delta_size;
}

TransportCCFeedback::TransportCCFeedback()
    : TransportFeedback(kTransportCCFeedbackHeaderSizeBytes),
      num_seq_no_(0),
      base_time_ticks_(0),
      feedback_seq_(0),
      last_chunk_(new LastChunk()) {}

TransportCCFeedback::~TransportCCFeedback() {}

void TransportCCFeedback::SetBase(uint16_t base_sequence,
                                  int64_t ref_timestamp_us) {
  RTC_DCHECK_EQ(num_seq_no_, 0);
  RTC_DCHECK_GE(ref_timestamp_us, 0);
  base_seq_no_ = base_sequence;
  base_time_ticks_ = (ref_timestamp_us % kTimeWrapPeriodUs) / kBaseScaleFactor;
  last_timestamp_us_ = GetBaseTimeUs();
}

void TransportCCFeedback::SetFeedbackSequenceNumber(uint8_t feedback_sequence) {
  feedback_seq_ = feedback_sequence;
}

bool TransportCCFeedback::AddReceivedPacket(uint16_t sequence_number,
                                          int64_t timestamp_us) {
  // Convert to ticks and round.
  int64_t delta_full = (timestamp_us - last_timestamp_us_) % kTimeWrapPeriodUs;
  if (delta_full > kTimeWrapPeriodUs / 2)
    delta_full -= kTimeWrapPeriodUs;
  delta_full +=
      delta_full < 0 ? -(kDeltaScaleFactor / 2) : kDeltaScaleFactor / 2;
  delta_full /= kDeltaScaleFactor;

  int16_t delta = static_cast<int16_t>(delta_full);
  // If larger than 16bit signed, we can't represent it - need new fb packet.
  if (delta != delta_full) {
    LOG(LS_WARNING) << "Delta value too large ( >= 2^16 ticks )";
    return false;
  }

  uint16_t next_seq_no = base_seq_no_ + num_seq_no_;
  if (sequence_number != next_seq_no) {
    uint16_t last_seq_no = next_seq_no - 1;
    if (!IsNewerSequenceNumber(sequence_number, last_seq_no))
      return false;
    for (; next_seq_no != sequence_number; ++next_seq_no)
      if (!AddDeltaSize(0))
        return false;
  }

  DeltaSize delta_size = (delta >= 0 && delta <= 0xff) ? 1 : 2;
  if (!AddDeltaSize(delta_size))
    return false;

  packets_.emplace_back(sequence_number, delta);
  last_timestamp_us_ += delta * kDeltaScaleFactor;
  size_bytes_ += delta_size;
  return true;
}

std::vector<TransportCCFeedback::StatusSymbol>
TransportCCFeedback::GetStatusVector() const {
  std::vector<TransportCCFeedback::StatusSymbol> symbols;
  uint16_t seq_no = GetBaseSequence();
  for (const auto& packet : packets_) {
    for (; seq_no != packet.sequence_number; ++seq_no)
      symbols.push_back(StatusSymbol::kNotReceived);
    if (packet.delta_ticks >= 0x00 && packet.delta_ticks <= 0xff) {
      symbols.push_back(StatusSymbol::kReceivedSmallDelta);
    } else {
      symbols.push_back(StatusSymbol::kReceivedLargeDelta);
    }
    ++seq_no;
  }
  return symbols;
}

std::vector<int16_t> TransportCCFeedback::GetReceiveDeltas() const {
  std::vector<int16_t> deltas;
  for (const auto& packet : packets_)
    deltas.push_back(packet.delta_ticks);
  return deltas;
}

int64_t TransportCCFeedback::GetBaseTimeUs() const {
  return static_cast<int64_t>(base_time_ticks_) * kBaseScaleFactor;
}

std::vector<int64_t> TransportCCFeedback::GetReceiveDeltasUs() const {
  std::vector<int64_t> us_deltas;
  for (const auto& packet : packets_)
    us_deltas.push_back(packet.delta_ticks * kDeltaScaleFactor);
  return us_deltas;
}

// De-serialize packet.
bool TransportCCFeedback::Parse(const CommonHeader& packet) {
  RTC_DCHECK_EQ(packet.type(), kPacketType);
  RTC_DCHECK_EQ(packet.fmt(), kFeedbackMessageType);

  if (packet.payload_size_bytes() < kMinPayloadSizeBytes) {
    LOG(LS_WARNING) << "Buffer too small (" << packet.payload_size_bytes()
                    << " bytes) to fit a "
                       "FeedbackPacket. Minimum size = "
                    << kMinPayloadSizeBytes;
    return false;
  }

  const uint8_t* const payload = packet.payload();
  ParseCommonFeedback(payload);

  base_seq_no_ = ByteReader<uint16_t>::ReadBigEndian(&payload[8]);
  size_t status_count = ByteReader<uint16_t>::ReadBigEndian(&payload[10]);
  base_time_ticks_ = ByteReader<int32_t, 3>::ReadBigEndian(&payload[12]);
  feedback_seq_ = payload[15];
  Clear();
  size_t index = 16;
  const size_t end_index = packet.payload_size_bytes();

  if (status_count == 0) {
    LOG(LS_WARNING) << "Empty feedback messages not allowed.";
    return false;
  }

  std::vector<uint8_t> delta_sizes;
  delta_sizes.reserve(status_count);
  while (delta_sizes.size() < status_count) {
    if (index + kChunkSizeBytes > end_index) {
      LOG(LS_WARNING) << "Buffer overflow while parsing packet.";
      Clear();
      return false;
    }

    uint16_t chunk = ByteReader<uint16_t>::ReadBigEndian(&payload[index]);
    index += kChunkSizeBytes;
    encoded_chunks_.push_back(chunk);
    last_chunk_->Decode(chunk, status_count - delta_sizes.size());
    last_chunk_->AppendTo(&delta_sizes);
  }
  // Last chunk is stored in the |last_chunk_|.
  encoded_chunks_.pop_back();
  RTC_DCHECK_EQ(delta_sizes.size(), status_count);
  num_seq_no_ = status_count;

  uint16_t seq_no = base_seq_no_;
  for (size_t delta_size : delta_sizes) {
    if (index + delta_size > end_index) {
      LOG(LS_WARNING) << "Buffer overflow while parsing packet.";
      Clear();
      return false;
    }
    switch (delta_size) {
      case 0:
        break;
      case 1: {
        int16_t delta = payload[index];
        packets_.emplace_back(seq_no, delta);
        last_timestamp_us_ += delta * kDeltaScaleFactor;
        index += delta_size;
        break;
      }
      case 2: {
        int16_t delta = ByteReader<int16_t>::ReadBigEndian(&payload[index]);
        packets_.emplace_back(seq_no, delta);
        last_timestamp_us_ += delta * kDeltaScaleFactor;
        index += delta_size;
        break;
      }
      case 3:
        Clear();
        LOG(LS_WARNING) << "Invalid delta_size for seq_no " << seq_no;
        return false;
      default:
        RTC_NOTREACHED();
        break;
    }
    ++seq_no;
  }
  size_bytes_ = RtcpPacket::kHeaderLength + index;
  RTC_DCHECK_LE(index, end_index);
  return true;
}

std::unique_ptr<TransportCCFeedback> TransportCCFeedback::ParseFrom(
    const uint8_t* buffer,
    size_t length) {
  CommonHeader header;
  if (!header.Parse(buffer, length))
    return nullptr;
  if (header.type() != kPacketType || header.fmt() != kFeedbackMessageType)
    return nullptr;
  std::unique_ptr<TransportCCFeedback> parsed(new TransportCCFeedback);
  if (!parsed->Parse(header))
    return nullptr;
  return parsed;
}

bool TransportCCFeedback::IsConsistent() const {
  size_t packet_size = kTransportCCFeedbackHeaderSizeBytes;
  std::vector<DeltaSize> delta_sizes;
  LastChunk chunk_decoder;
  for (uint16_t chunk : encoded_chunks_) {
    chunk_decoder.Decode(chunk, kMaxReportedPackets);
    chunk_decoder.AppendTo(&delta_sizes);
    packet_size += kChunkSizeBytes;
  }
  if (!last_chunk_->Empty()) {
    last_chunk_->AppendTo(&delta_sizes);
    packet_size += kChunkSizeBytes;
  }
  if (num_seq_no_ != delta_sizes.size()) {
    LOG(LS_ERROR) << delta_sizes.size() << " packets encoded. Expected "
                  << num_seq_no_;
    return false;
  }
  int64_t timestamp_us = base_time_ticks_ * kBaseScaleFactor;
  auto packet_it = packets_.begin();
  uint16_t seq_no = base_seq_no_;
  for (DeltaSize delta_size : delta_sizes) {
    if (delta_size > 0) {
      if (packet_it == packets_.end()) {
        LOG(LS_ERROR) << "Failed to find delta for seq_no " << seq_no;
        return false;
      }
      if (packet_it->sequence_number != seq_no) {
        LOG(LS_ERROR) << "Expected to find delta for seq_no " << seq_no
                      << ". Next delta is for " << packet_it->sequence_number;
        return false;
      }
      if (delta_size == 1 &&
          (packet_it->delta_ticks < 0 || packet_it->delta_ticks > 0xff)) {
        LOG(LS_ERROR) << "Delta " << packet_it->delta_ticks << " for seq_no "
                      << seq_no << " doesn't fit into one byte";
        return false;
      }
      timestamp_us += packet_it->delta_ticks * kDeltaScaleFactor;
      ++packet_it;
    }
    packet_size += delta_size;
    ++seq_no;
  }
  if (packet_it != packets_.end()) {
    LOG(LS_ERROR) << "Unencoded delta for seq_no "
                  << packet_it->sequence_number;
    return false;
  }
  if (timestamp_us != last_timestamp_us_) {
    LOG(LS_ERROR) << "Last timestamp mismatch. Calculated: " << timestamp_us
                  << ". Saved: " << last_timestamp_us_;
    return false;
  }
  if (size_bytes_ != packet_size) {
    LOG(LS_ERROR) << "Rtcp packet size mismatch. Calculated: " << packet_size
                  << ". Saved: " << size_bytes_;
    return false;
  }
  return true;
}

std::vector<PacketInfo> TransportCCFeedback::GetFeedbackVector(int64_t base_offset_ms) const {
  std::vector<PacketInfo> packet_feedback_vector;
  uint16_t sequence_number = GetBaseSequence();
  std::vector<int64_t> delta_vec = GetReceiveDeltasUs();
  auto delta_it = delta_vec.begin();
  packet_feedback_vector.reserve(delta_vec.size());

  int64_t offset_us = 0;
  for (auto symbol : GetStatusVector()) {
    // TODO: (semena) But then, if initial packet was lost, info is discarded
    // This logic is taken from TransportFeedbackAdapter
    if (symbol != StatusSymbol::kNotReceived) {
      RTC_DCHECK(delta_it != delta_vec.end());
      offset_us += *(delta_it++);
      int64_t timestamp_ms = base_offset_ms + (offset_us / 1000);
      PacketInfo info(timestamp_ms, sequence_number);
      packet_feedback_vector.push_back(info);
    }
    ++sequence_number;
  }
  RTC_DCHECK(delta_it == delta_vec.end());
  return packet_feedback_vector;
}

// Serialize packet.
bool TransportCCFeedback::Create(uint8_t* packet,
                               size_t* position,
                               size_t max_length,
                               PacketReadyCallback* callback) const {
  if (num_seq_no_ == 0)
    return false;

  while (*position + BlockLength() > max_length) {
    if (!OnBufferFull(packet, position, callback))
      return false;
  }
  const size_t position_end = *position + BlockLength();

  CreateHeader(kFeedbackMessageType, kPacketType, HeaderLength(), packet,
               position);
  CreateCommonFeedback(packet + *position);
  *position += kCommonFeedbackLength;

  ByteWriter<uint16_t>::WriteBigEndian(&packet[*position], base_seq_no_);
  *position += 2;

  ByteWriter<uint16_t>::WriteBigEndian(&packet[*position], num_seq_no_);
  *position += 2;

  ByteWriter<int32_t, 3>::WriteBigEndian(&packet[*position], base_time_ticks_);
  *position += 3;

  packet[(*position)++] = feedback_seq_;

  for (uint16_t chunk : encoded_chunks_) {
    ByteWriter<uint16_t>::WriteBigEndian(&packet[*position], chunk);
    *position += 2;
  }
  if (!last_chunk_->Empty()) {
    uint16_t chunk = last_chunk_->EncodeLast();
    ByteWriter<uint16_t>::WriteBigEndian(&packet[*position], chunk);
    *position += 2;
  }

  for (const auto& received_packet : packets_) {
    int16_t delta = received_packet.delta_ticks;
    if (delta >= 0 && delta <= 0xFF) {
      packet[(*position)++] = delta;
    } else {
      ByteWriter<int16_t>::WriteBigEndian(&packet[*position], delta);
      *position += 2;
    }
  }

  while ((*position % 4) != 0)
    packet[(*position)++] = 0;

  RTC_DCHECK_EQ(*position, position_end);
  return true;
}

void TransportCCFeedback::Clear() {
  num_seq_no_ = 0;
  last_timestamp_us_ = GetBaseTimeUs();
  packets_.clear();
  encoded_chunks_.clear();
  last_chunk_->Clear();
  size_bytes_ = kTransportCCFeedbackHeaderSizeBytes;
}

bool TransportCCFeedback::AddDeltaSize(DeltaSize delta_size) {
  if (num_seq_no_ == kMaxReportedPackets)
    return false;
  size_t add_chunk_size = last_chunk_->Empty() ? kChunkSizeBytes : 0;
  if (size_bytes_ + delta_size + add_chunk_size > kMaxSizeBytes)
    return false;

  if (last_chunk_->CanAdd(delta_size)) {
    size_bytes_ += add_chunk_size;
    last_chunk_->Add(delta_size);
    ++num_seq_no_;
    return true;
  }
  if (size_bytes_ + delta_size + kChunkSizeBytes > kMaxSizeBytes)
    return false;

  encoded_chunks_.push_back(last_chunk_->Emit());
  size_bytes_ += kChunkSizeBytes;
  last_chunk_->Add(delta_size);
  ++num_seq_no_;
  return true;
}





//-- RCTP CCFB HEADER (draft-ietf-avtcore-cc-feedback-message-02) -//
//   0                   1                   2                   3
//   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |V=2|P| FMT=CCFB| PT=RTPFB=205  |          length               |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |                 SSRC of RTCP packet sender                    |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |                   SSRC of 1st RTP Stream                      |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |          begin_seq            |             end_seq + 1       |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |L|ECN|  Arrival time offset    | ...                           .
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  .                                                               .
//  .                                                               .
//  .                                                               .
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |                   SSRC of nth RTP Stream                      |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |          begin_seq            |             end_seq + 1       |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |L|ECN|  Arrival time offset    | ...                           |
//  .                                                               .
//  .                                                               .
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//  |                        Report Timestamp                       |
//  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

constexpr uint8_t CcfbFeedback::kFeedbackMessageType;
constexpr uint16_t CcfbFeedback::MetricBlock::overrange_;
constexpr uint16_t CcfbFeedback::MetricBlock::unavailable_;

// RTCP common header + SSRC of packet sender + report timestamp
static constexpr size_t kCcfbMinPayloadSizeBytes = 4 + 4 + 4;

CcfbFeedback::CcfbFeedback()
    : TransportFeedback(kCcfbMinPayloadSizeBytes),
      report_blocks_() {}

CcfbFeedback::~CcfbFeedback() {}

template <typename T>
T ReadSequential(const uint8_t* const payload, size_t* index) {
  auto data = ByteReader<T>::ReadBigEndian(&payload[*index]);
  *index += sizeof(T);
  return data;
}

template <typename T>
void WriteSequential(uint8_t* payload, size_t* index, T data) {
  ByteWriter<T>::WriteBigEndian(&payload[*index], data);
  *index += sizeof(T);
}

bool RtpHdrGetBit (uint8_t val, uint8_t pos)
{
    return bool(val & (1u << pos));
}

void RtpHdrSetBit (uint8_t& val, uint8_t pos, bool bit)
{
  RTC_DCHECK_LT(pos, 8);
  if (bit) {
    val |= (1u << pos);
  } else {
    val &= (~(1u << pos));
  }
}

bool CcfbFeedback::UpdateLength()
{
  if (report_blocks_.empty()) {
    return false;
  }
  size_t len = 8; // common header + SSRC of packet sender
  for (auto& rb : report_blocks_) {
    len+=4; // SSRC
    len+=4; // begin & end seq
    const auto beginStop = CalculateBeginStopSeq(base_seq_no_, rb.second);
    const uint16_t beginSeq = beginStop.first;
    const uint16_t stopSeq = beginStop.second;
    const uint16_t nMetricBlocks = stopSeq - beginSeq; //this wraps properly
    const uint16_t nPaddingBlocks = nMetricBlocks % 2;
    len += (nMetricBlocks + nPaddingBlocks) * 2; // metric blocks are 16 bits long
  }
  len+=4; // report timestamp field
  if (len > 0xffff) {
    return false;
  }
  size_bytes_ = len;
  return true;
}

void CcfbFeedback::SetBase(uint16_t base_sequence,
                           int64_t ref_timestamp_us) {
  base_seq_no_ = base_sequence;
  last_timestamp_us_ = ref_timestamp_us;
}

void CcfbFeedback::SetFeedbackSequenceNumber(uint8_t feedback_sequence) {
  // CCFB format does not contain a sequence number for feedback packets
}

bool CcfbFeedback::AddReceivedPacket(uint16_t sequence_number,
                                     int64_t timestamp_us) {
  printf("\t\t\t\t\t\t\t\t\t\t\t\tReceived packet %d %" PRId64 "\n", sequence_number, timestamp_us);
  //TODO: Another difference with NS3: Only one media SSRC supported (in the packet format)
  uint32_t ssrc = media_ssrc();
  //TODO: No ecn support at the moment
  uint8_t ecn = 0;
  RTC_DCHECK_GE(timestamp_us, 0); //TODO Remove this and check type all over
  if (ecn > 0x03) {
      //log: bad ecn
      return false;
  }
  auto& rb = report_blocks_[ssrc];
  if (rb.find(sequence_number) != rb.end()) {
    //log: duplicate
    return false;
  }
  auto& mb = rb[sequence_number];
  mb.timestamp_us_ = timestamp_us;
  mb.ecn_ = ecn;
  if (!UpdateLength()) {
      rb.erase(sequence_number);
      if (rb.empty()) {
          report_blocks_.erase(ssrc);
      }
      //log: too long
      return false;
  }
  last_timestamp_us_ = std::max(last_timestamp_us_, timestamp_us);
  return true;
}

std::vector<PacketInfo> CcfbFeedback::GetFeedbackVector(int64_t base_offset_ms) const {
  std::vector<PacketInfo> packet_feedback_vector;
  RTC_DCHECK(IsConsistent()); // Exactly one SSRC

  if (report_blocks_.empty()) {
    return packet_feedback_vector;
  }
  const auto& rb_it = report_blocks_.find(media_ssrc());
  if(rb_it == report_blocks_.end()) {
    FATAL();
  }
  const auto& rb = rb_it->second;
  const auto beginStop = CalculateBeginStopSeq(base_seq_no_, rb);
  const uint16_t beginSeq = beginStop.first;
  const uint16_t stopSeq = beginStop.second;
  for (uint16_t i = beginSeq; i != stopSeq; ++i) {
    const auto& mb_it = rb.find(i);
    const bool received = (mb_it != rb.end());
    if (received) {
      const auto& mb = mb_it->second;
      const int64_t mb_timestampUs = int64_t(mb.timestamp_us_);
      const int64_t timestamp_ms = base_offset_ms + (mb_timestampUs / 1000);
      PacketInfo info(timestamp_ms, i);
      packet_feedback_vector.push_back(info);
    }
  }
  return packet_feedback_vector;
}

int64_t CcfbFeedback::GetBaseTimeUs() const {
  return last_timestamp_us_;
}

// De-serialize packet.
bool CcfbFeedback::Parse(const CommonHeader& packet) {
  printf("\t\t\t\t\t\t+Begin deserialize CCFB packet\n");
  RTC_DCHECK_EQ(packet.type(), kPacketType);
  RTC_DCHECK_EQ(packet.fmt(), kFeedbackMessageType);

  const size_t total_length = packet.payload_size_bytes();
  if (total_length < kCcfbMinPayloadSizeBytes - 4 /* Common header */) {
    LOG(LS_WARNING) << "Buffer too small (" << total_length
                    << " bytes) to fit a "
                       "FeedbackPacket. Minimum size = "
                    << (kCcfbMinPayloadSizeBytes - 4);
    return false;
  }
  if (total_length % 4 != 0) {
    LOG(LS_WARNING) << "Malformed buffer, length of data + padding ("
                    << total_length << " bytes) should be divisible by 4";
    return false;
  }

  const uint8_t* const payload = packet.payload();
  ParseCommonFeedback(payload);
  // Start at the second SSRC as that gets read again in the loop below
  size_t index = 4;
  Clear();

  //length of all report blocks in 16-bit words (also including RTP timestamp)
  size_t len_left = (total_length - index) / 2;
  uint16_t base_seq = 0;
  bool base_seq_set = false;
  // exit when the only the timestamp is left
  while (len_left > 2) {
    RTC_DCHECK_GE(len_left, 4); // SSRC + begin & end
    const auto ssrc = ReadSequential<uint32_t>(payload, &index);
    auto& rb = report_blocks_[ssrc];
    const uint16_t beginSeq = ReadSequential<uint16_t>(payload, &index);
    const uint16_t stopSeq = ReadSequential<uint16_t>(payload, &index);
    if (!base_seq_set) {
      base_seq_set = true;
      base_seq = beginSeq;
    }
    len_left -= 4;
    const uint16_t nMetricBlocks = stopSeq - beginSeq; //this wraps properly
    const uint16_t nPaddingBlocks = nMetricBlocks % 2;
    RTC_DCHECK_GE(len_left, nMetricBlocks + nPaddingBlocks);
    uint16_t seq = beginSeq;
    printf("\t\t\t\t\t\tB %d, S %d\n", beginSeq, stopSeq);
    for (auto i = 0u; i < nMetricBlocks; ++i) {
      const auto octet1 = payload[index++];
      const auto octet2 = payload[index++];
      if (RtpHdrGetBit(octet1, 7)) {
        uint16_t ato = (uint16_t (octet1) << 8) & 0x1f00;
        ato |= uint16_t(octet2);
        // 'Unavailable' treated as a lost packet
        if (ato != MetricBlock::unavailable_) {
          auto &mb = rb[seq];
          mb.ecn_ = (octet1 >> 5) & 0x03;
          mb.ato_ = ato;
          printf("\t\t\t\t\t\t    ecn %d, ato %d\n", mb.ecn_, mb.ato_);
        } else {
          printf("\t\t\t\t\t\t    LOST2\n");
        }
      } else {
        printf("\t\t\t\t\t\t    LOST\n");
      }
      ++seq;
    }
    len_left -= nMetricBlocks;
    if (nPaddingBlocks == 1) {
      (void)ReadSequential<uint16_t>(payload, &index); //skip padding
        --len_left;
    }
    RTC_DCHECK_EQ(2, len_left); // Only one SSRC supported at the moment
  }
  const uint32_t ntpRef = ReadSequential<uint32_t>(payload, &index);
  printf("\t\t\t\t\t\t    ntpRef %u us %" PRIu64 "\n", ntpRef, NtpToUs(ntpRef));
  // Populate all timestamps once Report Timestamp is known
  for (auto& rb : report_blocks_) {
    for (auto& mb : rb.second) {
      const uint32_t ntp = AtoToNtp(mb.second.ato_, ntpRef);
      mb.second.timestamp_us_ = NtpToUs(ntp);
      printf("\t\t\t\t\t\t    timestamp %" PRIu64 "\n", mb.second.timestamp_us_);
    }
  }

  if (!base_seq_set || report_blocks_.empty()) {
    LOG(LS_WARNING) << "Empty reports currently not supported";
    Clear();
    return false;
  }

  size_bytes_ = total_length + 4 /* Common header */;

  if (report_blocks_.size() > 1) {
    LOG(LS_WARNING) << "More than one SSRC (" << report_blocks_.size()
                    << ") not currently supported";
    Clear();
    return false;
  }

  //TODO This "base data" should be per-SSRC!!
  SetBase(base_seq, NtpToUs(ntpRef));
  printf("\t\t\t\t\t\t-Deserialized CCFB packet with %zu bytes\n", size_bytes_);
  return true;
}

std::unique_ptr<CcfbFeedback> CcfbFeedback::ParseFrom(
    const uint8_t* buffer,
    size_t length) {
  CommonHeader header;
  if (!header.Parse(buffer, length))
    return nullptr;
  if (header.type() != kPacketType || header.fmt() != kFeedbackMessageType)
    return nullptr;
  std::unique_ptr<CcfbFeedback> parsed(new CcfbFeedback);
  if (!parsed->Parse(header))
    return nullptr;
  return parsed;
}

bool CcfbFeedback::IsConsistent() const {
  return (size_bytes_ >= kCcfbMinPayloadSizeBytes)
      && (size_bytes_% 4 == 0)
      && (report_blocks_.size() == 0 ||
            (report_blocks_.size() == 1 &&
             report_blocks_.find(media_ssrc()) != report_blocks_.end()));
}

// Serialize packet.
bool CcfbFeedback::Create(uint8_t* packet,
                                  size_t* position,
                                  size_t max_length,
                                  PacketReadyCallback* callback) const {
  printf("+Begin serialize CCFB packet\n");
  RTC_DCHECK_EQ(1, report_blocks_.size()); // Only one SSRC supported

  /* TODO(drno): does this number need to grow like it does in the existing
   * code, because it doesn't right now and prevents feedback packets getting
   * send
   */
  RTC_DCHECK_GE(size_bytes_, kCcfbMinPayloadSizeBytes);
  RTC_DCHECK_EQ(0, size_bytes_ % 4);
  const size_t position_end = *position + size_bytes_;
  while (position_end > max_length) {
    if (!OnBufferFull(packet, position, callback))
      return false;
  }

  if (report_blocks_.empty()) { // TODO (authors): 0 report blocks should be allowed
    return false;
  }

  CreateHeader(kFeedbackMessageType, kPacketType, HeaderLength(), packet,
               position);
  WriteSequential<uint32_t>(packet, position, sender_ssrc());

  for (const auto& rb : report_blocks_) {
    WriteSequential<uint32_t>(packet, position, rb.first);
    const auto beginStop = CalculateBeginStopSeq(base_seq_no_, rb.second);
    const uint16_t beginSeq = beginStop.first;
    const uint16_t stopSeq = beginStop.second;
    WriteSequential<uint16_t>(packet, position, beginSeq);
    // TODO(drno): does it really make sense that in the case of a single report
    // block we write the same number as stopSeq and the beginSeq?
    // (Sergio): this is the main difference between versions -01 and -02. I just corrected it
    // In version -02, if both fields are equal, it means no metrics (i.e., empty report for that SSRC)
    WriteSequential<uint16_t>(packet, position, stopSeq);
    RTC_DCHECK(!rb.second.empty()); // at least one metric block
    printf("B %d, S %d\n", beginSeq, stopSeq);
    for (uint16_t i = beginSeq; i != stopSeq; ++i) {
      const auto& mb_it = rb.second.find(i);
      uint8_t octet1 = 0;
      uint8_t octet2 = 0;
      const bool received = (mb_it != rb.second.end());
      RtpHdrSetBit(octet1, 7, received);
      if (received) {
        const auto& mb = mb_it->second;
        RTC_DCHECK_LE(mb.ecn_, 0x03);
        octet1 |= uint8_t((mb.ecn_ & 0x03) << 5);
        const uint32_t ntp = UsToNtp(mb.timestamp_us_);
        const uint32_t ntpRef = UsToNtp(last_timestamp_us_);
        const uint16_t ato = NtpToAto(ntp, ntpRef);
        RTC_DCHECK_LE(ato, 0x1fff);
        octet1 |= uint8_t(ato >> 8);
        octet2 |= uint8_t(ato & 0xff);
        printf("    ecn %d, ts %" PRIu64 "\n", mb.ecn_, mb.timestamp_us_);
      } else {
        printf("    LOST\n");
      }
      WriteSequential<uint8_t>(packet, position, octet1);
      WriteSequential<uint8_t>(packet, position, octet2);
    }
    if (uint16_t(stopSeq - beginSeq) % 2 == 1) {
      WriteSequential<uint16_t>(packet, position, 0); //padding
    }
  }
  const uint32_t ntpTs = UsToNtp(last_timestamp_us_);
  printf(" ntpRef %u us %" PRId64 " usu %" PRIu64 "\n", ntpTs, last_timestamp_us_,  last_timestamp_us_);
  printf(" back and forth us %" PRId64 "\n", NtpToUs(ntpTs));
  WriteSequential<uint32_t>(packet, position, ntpTs);

  /* TODO(drno): see above
  */
  printf("-Serialized CCFB packet with %zu bytes\n", size_bytes_);
  RTC_DCHECK_EQ(*position, position_end);
  return true;
}

void CcfbFeedback::Clear() {
  size_bytes_ = kCcfbMinPayloadSizeBytes;
  base_seq_no_ = 0;
  last_timestamp_us_ = 0;
  report_blocks_.clear();
}

std::pair<uint16_t, uint16_t>
CcfbFeedback::CalculateBeginStopSeq(uint16_t baseSeq, const ReportBlock_t& rb)
{
  //TODO: This would be more efficient to maintain in AddReceivedPacket
  uint16_t endSeq = baseSeq - 1;
  for (auto& mb : rb) {
    uint16_t currSeq = mb.first;
    if (uint16_t(baseSeq - currSeq) >= 16384 && uint16_t(currSeq - baseSeq) >= 16384) {
      // This discards all sequences more than 16k away from base sequence
      // We need both conditions to account for wrapping
      continue;
    }
    if (uint16_t(currSeq - endSeq) < 16384) // Means currSeq > endSeq with wrapping
      endSeq = currSeq;
  }
  return std::make_pair(baseSeq, endSeq + 1);
}

uint16_t CcfbFeedback::NtpToAto(uint32_t ntp, uint32_t ntpRef) {
  RTC_DCHECK_LE(ntp, ntpRef);
  // ato contains offset measured in 1/1024 seconds
  const uint32_t atoNtp = ntpRef - ntp;
  const uint32_t atoNtpRounded = atoNtp + (1 << 5);
  const uint16_t ato = uint16_t(atoNtpRounded >> 6); // i.e., * 0x400 / 0x10000
  return std::min(ato, MetricBlock::overrange_);
}

uint32_t CcfbFeedback::AtoToNtp(uint16_t ato, uint32_t ntpRef) {
  RTC_DCHECK_LT(ato, MetricBlock::unavailable_);
  // ato contains offset measured in 1/1024 seconds
  const uint32_t atoNtp = uint32_t(ato) << 6; // i.e., * 0x10000 / 0x400
  RTC_DCHECK_LE(atoNtp, ntpRef);
  return ntpRef - atoNtp;
}

uint64_t CcfbFeedback::NtpToUs(uint32_t ntp) {
  const double tsSeconds = double(ntp) / double(0x10000);
  return uint64_t(tsSeconds * 1000. * 1000.);
}
uint32_t CcfbFeedback::UsToNtp (uint64_t tsUs) {
  const double tsSeconds = double(tsUs) / 1000. / 1000.;
  return uint32_t(tsSeconds * double (0x10000));
}


}  // namespace rtcp
}  // namespace webrtc
