// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <oxrsys/protocol/FecCodec.h>
#include <oxrsys/protocol/Protocol.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <vector>

namespace oxr
{
namespace streaming
{

// Reassembles UDP video packets into complete NAL units, recovering losses from XOR FEC parity.
// Shared by every UDP video client (android-vr, steam-frame) so the reassembly rules -- parity
// matched by totalPackets as well as frameIndex, negotiated group layouts, recovery on parity
// arrival -- exist and are tested exactly once; the receivers own sockets, NACKs, logging, and
// stats, and drive this from their receive thread. Single-threaded by design.
//
// Retransmit wait (SetRetransmitWaitUs > 0): a NAL still incomplete after FEC when the next one
// starts is parked rather than abandoned. OnFrameIncomplete fires at that moment so the receiver
// can NACK; retransmitted packets (which carry the original frameIndex/packetIndex/totalPackets)
// fill the parked NAL, and it is delivered first, then the newer NALs buffered behind it, in
// order: nothing newer is delivered while an older NAL is parked, because the decoder must see
// NALs in stream order. The parked NAL is abandoned when the wait expires (caller's clock) or
// when more than SetMaxBufferedFrames newer frames have started; OnFrameAbandoned fires then,
// and by default (SetDiscardUntilKeyframe) everything after it is dropped until a keyframe
// starts, since decoding P-frames against a missing reference is the visible smear. A wait of 0
// (the default) is the original single-frame path, unchanged.
class VideoFrameAssembler
{
public:
    struct Frame
    {
        const uint8_t* data = nullptr;
        size_t size = 0;
        uint32_t frameIndex = 0;
        uint32_t totalPackets = 0;
        int64_t timestampNs = 0;
        uint8_t flags = 0;
        uint8_t codec = 0;
    };

    struct AbandonedFrame
    {
        uint32_t frameIndex = 0;
        uint32_t receivedPackets = 0;
        uint32_t totalPackets = 0;
        // Per-packet received flags (totalPackets entries), so the caller can NACK exactly the
        // missing ones. Valid only for the duration of the callback.
        const uint8_t* packetReceived = nullptr;
    };

    // Frame::data is valid only for the duration of the callback.
    using OnFrameCallback = std::function<void(const Frame&)>;
    using OnFrameAbandonedCallback = std::function<void(const AbandonedFrame&)>;
    using OnFecRecoveryCallback =
        std::function<void(uint32_t packetIndex, uint32_t totalPackets, uint32_t frameIndex)>;
    // Same payload as an abandonment: the NAL and which of its packets arrived.
    using OnFrameIncompleteCallback = std::function<void(const AbandonedFrame&)>;

    // One 90 Hz period plus margin; the measured Wi-Fi NACK round trip is about 5 ms.
    static constexpr int64_t kRecommendedRetransmitWaitUs = 12000;

    void SetOnFrame(OnFrameCallback callback) { onFrame_ = std::move(callback); }
    void SetOnFrameAbandoned(OnFrameAbandonedCallback callback)
    {
        onFrameAbandoned_ = std::move(callback);
    }
    void SetOnFecRecovery(OnFecRecoveryCallback callback)
    {
        onFecRecovery_ = std::move(callback);
    }
    // Retransmit-wait mode only: a NAL closed incomplete after FEC and parked. NACK from here.
    void SetOnFrameIncomplete(OnFrameIncompleteCallback callback)
    {
        onFrameIncomplete_ = std::move(callback);
    }

    // How long a parked NAL waits for retransmits, from the moment it was parked. 0 = the
    // original behaviour (abandon at once when the next NAL starts; OnFrameIncomplete never
    // fires). Change it only before the first packet or right after Reset().
    void SetRetransmitWaitUs(int64_t waitUs) { retransmitWaitUs_ = waitUs > 0 ? waitUs : 0; }
    int64_t RetransmitWaitUs() const { return retransmitWaitUs_; }
    // Newer frames (distinct frameIndex values) held behind a parked NAL; one more abandons it.
    void SetMaxBufferedFrames(uint32_t frames) { maxBufferedFrames_ = frames; }
    // After an abandonment in retransmit-wait mode, drop everything until a keyframe starts.
    void SetDiscardUntilKeyframe(bool discard) { discardUntilKeyframe_ = discard; }

    // Counters (retransmit-wait mode): parked NALs that were completed and delivered, NALs
    // dropped behind an abandonment, and retransmits that arrived after their NAL was settled.
    uint32_t ParkedRecoveries() const { return parkedRecoveries_; }
    uint32_t DiscardedNals() const { return discardedNals_; }
    uint32_t LateRetransmits() const { return lateRetransmits_; }

    // Selects the FEC group layout for frames that begin after this call; a frame in flight
    // keeps the layout it started with, so sender and receiver never mix layouts mid-frame.
    // Must match the server, which decides from CLIENT_CAPABILITY_FEC_INTERLEAVED.
    void SetInterleaved(bool interleaved) { interleaved_ = interleaved; }

    // Feed one video packet (data or FEC parity; not render-pose packets). In retransmit-wait
    // mode this overload reads std::chrono::steady_clock.
    void ProcessPacket(const protocol::VideoPacketHeader& header,
                       const uint8_t* payload, size_t payloadSize)
    {
        if (retransmitWaitUs_ > 0)
        {
            ProcessPacketWaiting(header, payload, payloadSize, SteadyNowUs());
            return;
        }
        ProcessPacketImmediate(header, payload, payloadSize);
    }

    // Same, with the caller's clock (microseconds, any epoch, monotonic). The clock is ignored
    // when the wait is 0.
    void ProcessPacket(const protocol::VideoPacketHeader& header,
                       const uint8_t* payload, size_t payloadSize, int64_t nowUs)
    {
        if (retransmitWaitUs_ > 0)
        {
            ProcessPacketWaiting(header, payload, payloadSize, nowUs);
            return;
        }
        ProcessPacketImmediate(header, payload, payloadSize);
    }

    // Retransmit-wait mode: apply the deadline without a packet (call on a receive timeout).
    void Poll(int64_t nowUs)
    {
        if (retransmitWaitUs_ > 0)
        {
            Drain(nowUs);
        }
    }
    void Poll() { Poll(SteadyNowUs()); }

    // Drop any partial frame, e.g. between connections.
    void Reset()
    {
        pending_.totalPackets = 0;
        pending_.frameIndex = 0;
        while (!active_.empty())
        {
            Recycle();
        }
        discarding_ = false;
        haveNewest_ = false;
        settledCount_ = 0;
    }

private:
    // The original single-frame path (retransmit wait 0).
    void ProcessPacketImmediate(const protocol::VideoPacketHeader& header,
                                const uint8_t* payload, size_t payloadSize)
    {
        // FEC parity packet - store it separately. The server reuses one frameIndex for every
        // NAL of a frame, each with its own packet numbering, so frameIndex alone does not
        // identify the NAL this parity belongs to: totalPackets must match too, or a parity
        // packet from a small NAL (SPS/PPS) reordered behind the next NAL's data would occupy
        // that NAL's parity slot and XOR-recover garbage.
        if (header.flags & protocol::VIDEO_FLAG_FEC)
        {
            if (header.frameIndex == pending_.frameIndex && pending_.totalPackets > 0 &&
                header.totalPackets == pending_.totalPackets)
            {
                if (AcceptParity(pending_, header, payload, payloadSize))
                {
                    DeliverPending(pending_);
                }
            }
            return;
        }

        // A different frameIndex - or a different totalPackets under the same frameIndex,
        // meaning another NAL of the same frame - starts a new reassembly.
        if (header.frameIndex != pending_.frameIndex ||
            header.totalPackets != pending_.totalPackets || pending_.totalPackets == 0)
        {
            AbandonOrRecoverPending();
            StartFrame(pending_, header);
        }

        if (StoreData(pending_, header, payload, payloadSize) &&
            pending_.receivedPackets == pending_.totalPackets)
        {
            DeliverPending(pending_);
        }
    }

    struct PendingFrame
    {
        uint32_t frameIndex = 0;
        uint32_t totalPackets = 0;
        uint32_t receivedPackets = 0;
        int64_t timestampNs = 0;
        uint8_t flags = 0;
        uint8_t codec = static_cast<uint8_t>(protocol::VideoCodec::H265);
        bool interleaved = false;
        std::vector<uint8_t> data;
        std::vector<uint8_t> packetReceived;
        std::vector<uint16_t> packetSizes;  // Actual size of each packet's payload
        std::vector<uint8_t> compactedData;

        // FEC parity packets indexed by group number
        uint32_t fecGroupCount = 0;
        uint32_t fecHeldCount = 0;
        std::vector<uint8_t> fecReceived;
        std::vector<uint8_t> fecData;  // fecGroupCount * MAX_PACKET_PAYLOAD
        std::vector<uint16_t> fecGroupLastPacketSizes;

        // Retransmit-wait mode: a newer NAL has started (closed), and when it was parked
        // incomplete (closedAtUs, parked).
        bool closed = false;
        bool parked = false;
        int64_t closedAtUs = 0;

        bool Complete() const { return totalPackets > 0 && receivedPackets == totalPackets; }
    };

    static int64_t SteadyNowUs()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    // Serial-number order for frameIndex (wraps at 2^32).
    static bool IsNewer(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) > 0; }

    // Stores one data packet; false if it was out of range or a duplicate.
    static bool StoreData(PendingFrame& f, const protocol::VideoPacketHeader& header,
                          const uint8_t* payload, size_t payloadSize)
    {
        if (header.packetIndex >= header.totalPackets || f.packetReceived[header.packetIndex])
        {
            return false;
        }
        const size_t offset = header.packetIndex * protocol::MAX_PACKET_PAYLOAD;
        memcpy(f.data.data() + offset, payload, payloadSize);
        f.packetReceived[header.packetIndex] = 1;
        f.packetSizes[header.packetIndex] = static_cast<uint16_t>(payloadSize);
        f.receivedPackets++;
        return true;
    }

    // Stores one parity packet already matched to f; true if it completed f by recovery.
    bool AcceptParity(PendingFrame& f, const protocol::VideoPacketHeader& header,
                      const uint8_t* payload, size_t payloadSize)
    {
        const uint32_t groupIdx = header.packetIndex;
        if (groupIdx >= f.fecGroupCount || f.fecReceived[groupIdx])
        {
            return false;
        }
        const size_t fecOffset = groupIdx * protocol::MAX_PACKET_PAYLOAD;
        memcpy(f.fecData.data() + fecOffset, payload,
               std::min(payloadSize, protocol::MAX_PACKET_PAYLOAD));
        f.fecGroupLastPacketSizes[groupIdx] = header.fecGroupLastPacketPayloadSize;
        f.fecReceived[groupIdx] = 1;
        f.fecHeldCount++;

        // Each parity packet can recover at most one loss (in its own group), so recovery is
        // worth attempting as soon as the parity held could cover everything missing - not
        // only when a single packet is missing, which would defer every burst recovery to the
        // next frame's arrival.
        const uint32_t missing = f.totalPackets - f.receivedPackets;
        return missing > 0 && f.fecHeldCount >= missing && TryFecRecovery(f);
    }

    static void StartFrame(PendingFrame& f, const protocol::VideoPacketHeader& header,
                           bool interleaved)
    {
        f.frameIndex = header.frameIndex;
        f.totalPackets = header.totalPackets;
        f.receivedPackets = 0;
        f.timestampNs = header.presentationTimeNs;
        f.flags = header.flags;
        f.codec = header.codec;
        // Latched per frame: a layout change mid-frame (never expected from a well-behaved
        // server) must not mix layouts within one frame's recovery.
        f.interleaved = interleaved;
        const size_t dataBytes = header.totalPackets * protocol::MAX_PACKET_PAYLOAD;
        if (f.data.size() < dataBytes)
        {
            f.data.resize(dataBytes);
        }
        f.packetReceived.assign(header.totalPackets, 0);
        f.packetSizes.assign(header.totalPackets, 0);

        f.fecGroupCount = fec::GroupCount(header.totalPackets);
        f.fecHeldCount = 0;
        f.fecReceived.assign(f.fecGroupCount, 0);
        const size_t fecBytes = f.fecGroupCount * protocol::MAX_PACKET_PAYLOAD;
        if (f.fecData.size() < fecBytes)
        {
            f.fecData.resize(fecBytes);
        }
        f.fecGroupLastPacketSizes.assign(f.fecGroupCount, 0);
        f.closed = false;
        f.parked = false;
        f.closedAtUs = 0;
    }

    void StartFrame(PendingFrame& f, const protocol::VideoPacketHeader& header)
    {
        StartFrame(f, header, interleaved_);
    }

    void AbandonOrRecoverPending()
    {
        if (pending_.totalPackets == 0 || pending_.receivedPackets == 0 ||
            pending_.receivedPackets >= pending_.totalPackets)
        {
            return;
        }
        if (TryFecRecovery(pending_))
        {
            DeliverPending(pending_);
            return;
        }
        if (onFrameAbandoned_)
        {
            onFrameAbandoned_({pending_.frameIndex, pending_.receivedPackets,
                               pending_.totalPackets, pending_.packetReceived.data()});
        }
    }

    bool TryFecRecovery(PendingFrame& f)
    {
        const uint32_t totalPackets = f.totalPackets;
        if (totalPackets == 0)
        {
            return false;
        }

        bool recovered = false;
        // Must match the layout the server used; negotiated via
        // CLIENT_CAPABILITY_FEC_INTERLEAVED.
        const fec::GroupLayout layout{totalPackets, f.interleaved};
        const uint32_t groupCount = layout.Count();

        for (uint32_t g = 0; g < groupCount; g++)
        {
            if (!f.fecReceived[g])
            {
                continue;  // No FEC parity for this group
            }

            const uint32_t members = layout.MemberCount(g);
            if (members == 0)
            {
                continue;
            }

            // Count missing packets in this group
            uint32_t missingIdx = UINT32_MAX;
            uint32_t missingCount = 0;
            for (uint32_t k = 0; k < members; k++)
            {
                const uint32_t i = layout.Member(g, k);
                if (!f.packetReceived[i])
                {
                    missingIdx = i;
                    missingCount++;
                }
            }

            if (missingCount != 1)
            {
                continue;  // FEC can only recover exactly 1 missing packet per group
            }

            // Gather present packets for XOR recovery
            const uint32_t presentCount = members - 1;
            std::array<const uint8_t*, protocol::FEC_GROUP_SIZE> presentPtrs = {};
            std::array<uint16_t, protocol::FEC_GROUP_SIZE> presentSizes = {};
            uint32_t p = 0;
            for (uint32_t k = 0; k < members; k++)
            {
                const uint32_t i = layout.Member(g, k);
                if (i != missingIdx)
                {
                    presentPtrs[p] = f.data.data() + i * protocol::MAX_PACKET_PAYLOAD;
                    presentSizes[p] = f.packetSizes[i];
                    p++;
                }
            }

            const uint8_t* fecPayload =
                f.fecData.data() + g * protocol::MAX_PACKET_PAYLOAD;
            uint8_t* recoveredSlot =
                f.data.data() + missingIdx * protocol::MAX_PACKET_PAYLOAD;
            fec::Decode(presentPtrs.data(), presentSizes.data(), presentCount, fecPayload,
                        recoveredSlot);

            uint16_t recoveredSize = static_cast<uint16_t>(protocol::MAX_PACKET_PAYLOAD);
            if (missingIdx == layout.Member(g, members - 1))
            {
                const uint16_t groupLastPacketSize = f.fecGroupLastPacketSizes[g];
                if (groupLastPacketSize > 0 &&
                    groupLastPacketSize <= protocol::MAX_PACKET_PAYLOAD)
                {
                    recoveredSize = groupLastPacketSize;
                }
            }

            f.packetReceived[missingIdx] = 1;
            f.packetSizes[missingIdx] = recoveredSize;
            f.receivedPackets++;
            recovered = true;

            if (onFecRecovery_)
            {
                onFecRecovery_(missingIdx, totalPackets, f.frameIndex);
            }
        }

        return recovered && (f.receivedPackets == f.totalPackets);
    }

    void DeliverPending(PendingFrame& f)
    {
        size_t totalSize = 0;
        for (uint32_t i = 0; i < f.totalPackets; i++)
        {
            totalSize += f.packetSizes[i];
        }

        Frame frame;
        frame.size = totalSize;
        frame.frameIndex = f.frameIndex;
        frame.totalPackets = f.totalPackets;
        frame.timestampNs = f.timestampNs;
        frame.flags = f.flags;
        frame.codec = f.codec;

        // Compact the data (remove gaps from fixed-size slots) into a reused buffer.
        if (f.totalPackets > 1)
        {
            f.compactedData.resize(totalSize);
            size_t dstOffset = 0;
            for (uint32_t i = 0; i < f.totalPackets; i++)
            {
                const size_t srcOffset = i * protocol::MAX_PACKET_PAYLOAD;
                memcpy(f.compactedData.data() + dstOffset,
                       f.data.data() + srcOffset,
                       f.packetSizes[i]);
                dstOffset += f.packetSizes[i];
            }
            frame.data = f.compactedData.data();
        }
        else
        {
            frame.data = f.data.data();
        }

        if (onFrame_)
        {
            onFrame_(frame);
        }

        f.totalPackets = 0;  // Mark as consumed
    }

    // ---- Retransmit-wait mode ----------------------------------------------------------

    // A frameIndex this far behind the newest one seen is a sender restart, not a late packet.
    static constexpr uint32_t kStaleWindowFrames = 256;
    // Bound on NALs held at once (a keyframe is several NALs under one frameIndex).
    static constexpr size_t kMaxActiveNals = 32;

    // The incomplete NAL a packet belongs to, if any. Complete NALs are not matched: a
    // single-packet NAL (VPS/SPS/PPS) shares frameIndex and totalPackets with its neighbours.
    PendingFrame* FindIncomplete(uint32_t frameIndex, uint32_t totalPackets)
    {
        for (PendingFrame& f : active_)
        {
            if (f.frameIndex == frameIndex && f.totalPackets == totalPackets && !f.Complete())
            {
                return &f;
            }
        }
        return nullptr;
    }

    bool MatchesCompleteMultiPacket(uint32_t frameIndex, uint32_t totalPackets) const
    {
        if (totalPackets <= 1)
        {
            return false;
        }
        for (const PendingFrame& f : active_)
        {
            if (f.frameIndex == frameIndex && f.totalPackets == totalPackets && f.Complete())
            {
                return true;
            }
        }
        return false;
    }

    bool IsSettled(uint32_t frameIndex, uint32_t totalPackets) const
    {
        const uint32_t n = std::min(settledCount_, kSettledKeys);
        for (uint32_t i = 0; i < n; i++)
        {
            if (settled_[i].frameIndex == frameIndex && settled_[i].totalPackets == totalPackets)
            {
                return true;
            }
        }
        return false;
    }

    void ProcessPacketWaiting(const protocol::VideoPacketHeader& header,
                              const uint8_t* payload, size_t payloadSize, int64_t nowUs)
    {
        if (header.flags & protocol::VIDEO_FLAG_FEC)
        {
            // Parity for a parked NAL arriving late still completes it.
            if (PendingFrame* f = FindIncomplete(header.frameIndex, header.totalPackets))
            {
                if (AcceptParity(*f, header, payload, payloadSize) && f->parked)
                {
                    parkedRecoveries_++;
                }
            }
            Drain(nowUs);
            return;
        }

        PendingFrame* target = FindIncomplete(header.frameIndex, header.totalPackets);
        if (target == nullptr)
        {
            if (IsSettled(header.frameIndex, header.totalPackets) ||
                MatchesCompleteMultiPacket(header.frameIndex, header.totalPackets))
            {
                lateRetransmits_++;
                Drain(nowUs);
                return;
            }

            if (haveNewest_ && IsNewer(newestFrameIndex_, header.frameIndex))
            {
                if (newestFrameIndex_ - header.frameIndex <= kStaleWindowFrames)
                {
                    // An older frame nobody is waiting for: a late duplicate or a retransmit
                    // for something already settled. Never let it restart assembly.
                    lateRetransmits_++;
                    Drain(nowUs);
                    return;
                }
                // The sender restarted its numbering: forget everything held.
                Reset();
            }
            if (!haveNewest_ || IsNewer(header.frameIndex, newestFrameIndex_))
            {
                newestFrameIndex_ = header.frameIndex;
                haveNewest_ = true;
            }

            if (discarding_)
            {
                if (!(header.flags & protocol::VIDEO_FLAG_KEYFRAME) ||
                    !IsNewer(header.frameIndex, discardAfterFrameIndex_))
                {
                    Drain(nowUs);
                    return;
                }
                discarding_ = false;
            }

            if (!active_.empty())
            {
                CloseNewest(nowUs);
            }
            active_.emplace_back();
            if (!spare_.empty())
            {
                active_.back() = std::move(spare_.back());
                spare_.pop_back();
            }
            StartFrame(active_.back(), header);
            target = &active_.back();
        }

        if (StoreData(*target, header, payload, payloadSize) && target->Complete() &&
            target->parked)
        {
            parkedRecoveries_++;
        }
        Drain(nowUs);
    }

    // The newest NAL is no longer the one being assembled: recover it from parity if it can
    // be, else park it and tell the receiver which packets to NACK.
    void CloseNewest(int64_t nowUs)
    {
        PendingFrame& f = active_.back();
        if (f.closed)
        {
            return;
        }
        f.closed = true;
        if (f.Complete() || f.receivedPackets == 0)
        {
            return;
        }
        if (TryFecRecovery(f))
        {
            return;
        }
        f.parked = true;
        f.closedAtUs = nowUs;
        if (onFrameIncomplete_)
        {
            onFrameIncomplete_({f.frameIndex, f.receivedPackets, f.totalPackets,
                                f.packetReceived.data()});
        }
    }

    uint32_t NewerFramesBehindHead() const
    {
        uint32_t count = 0;
        uint32_t last = active_.front().frameIndex;
        for (size_t i = 1; i < active_.size(); i++)
        {
            if (active_[i].frameIndex != last)
            {
                count++;
                last = active_[i].frameIndex;
            }
        }
        return count;
    }

    void Recycle()
    {
        PendingFrame& f = active_.front();
        if (f.parked)
        {
            settled_[settledCount_ % kSettledKeys] = {f.frameIndex, f.totalPackets};
            settledCount_++;
        }
        f.totalPackets = 0;
        spare_.push_back(std::move(f));
        active_.pop_front();
    }

    // Deliver from the head while it is complete; abandon a parked head whose wait is over.
    void Drain(int64_t nowUs)
    {
        while (!active_.empty())
        {
            PendingFrame& head = active_.front();
            if (head.Complete())
            {
                DeliverPending(head);
                head.totalPackets = head.receivedPackets;  // DeliverPending zeroes it
                Recycle();
                continue;
            }
            if (!head.closed)
            {
                return;  // Still being assembled.
            }
            const bool expired = nowUs - head.closedAtUs >= retransmitWaitUs_;
            const bool crowded = NewerFramesBehindHead() > maxBufferedFrames_ ||
                                 active_.size() > kMaxActiveNals;
            if (!expired && !crowded)
            {
                return;
            }
            AbandonHead();
        }
    }

    void AbandonHead()
    {
        PendingFrame& head = active_.front();
        if (onFrameAbandoned_)
        {
            onFrameAbandoned_({head.frameIndex, head.receivedPackets, head.totalPackets,
                               head.packetReceived.data()});
        }
        const uint32_t abandonedFrameIndex = head.frameIndex;
        Recycle();
        if (!discardUntilKeyframe_)
        {
            return;
        }
        // Everything after a lost NAL decodes against a missing reference until a keyframe
        // (of a later frame: the rest of a keyframe whose own NAL was lost is no reset).
        while (!active_.empty())
        {
            const PendingFrame& f = active_.front();
            if ((f.flags & protocol::VIDEO_FLAG_KEYFRAME) &&
                IsNewer(f.frameIndex, abandonedFrameIndex))
            {
                return;
            }
            discardedNals_++;
            Recycle();
        }
        discarding_ = true;
        discardAfterFrameIndex_ = abandonedFrameIndex;
    }

    bool interleaved_ = false;
    PendingFrame pending_;
    OnFrameCallback onFrame_;
    OnFrameAbandonedCallback onFrameAbandoned_;
    OnFecRecoveryCallback onFecRecovery_;
    OnFrameIncompleteCallback onFrameIncomplete_;

    // Retransmit-wait state. active_ holds NALs in arrival order: zero or more closed ones
    // (complete and waiting behind a parked one, or parked) and at most one open one at the
    // back. spare_ keeps released buffers for reuse.
    int64_t retransmitWaitUs_ = 0;
    uint32_t maxBufferedFrames_ = 2;
    bool discardUntilKeyframe_ = true;
    std::deque<PendingFrame> active_;
    std::vector<PendingFrame> spare_;
    bool discarding_ = false;
    uint32_t discardAfterFrameIndex_ = 0;
    bool haveNewest_ = false;
    uint32_t newestFrameIndex_ = 0;
    // NALs that were parked (so NACKed) and have since been delivered or dropped: their
    // retransmits may still be in flight and must not start a new reassembly.
    struct SettledKey
    {
        uint32_t frameIndex = 0;
        uint32_t totalPackets = 0;
    };
    static constexpr uint32_t kSettledKeys = 16;
    std::array<SettledKey, kSettledKeys> settled_ = {};
    uint32_t settledCount_ = 0;
    uint32_t parkedRecoveries_ = 0;
    uint32_t discardedNals_ = 0;
    uint32_t lateRetransmits_ = 0;
};

} // namespace streaming
} // namespace oxr
