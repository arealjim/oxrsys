// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include <oxrsys/streaming/VideoFrameAssembler.h>

#include <oxrsys/protocol/FecCodec.h>
#include <oxrsys/protocol/Protocol.h>

#include <array>
#include <cstring>
#include <vector>

using namespace oxr;
using oxr::streaming::VideoFrameAssembler;

namespace
{

// A synthetic NAL: per-packet payloads with deterministic bytes and a short last packet, plus
// the parity packets a server emitting with the given layout would produce. Feeding these to
// the assembler in various orders and with various losses exercises the receiver half of the
// FEC contract without sockets.
struct TestNal
{
    uint32_t frameIndex;
    uint32_t totalPackets;
    std::vector<std::vector<uint8_t>> payloads;
    std::vector<std::vector<uint8_t>> parity;
    std::vector<uint16_t> parityLastMemberSize;

    std::vector<uint8_t> ExpectedBytes() const
    {
        std::vector<uint8_t> all;
        for (const auto& p : payloads)
        {
            all.insert(all.end(), p.begin(), p.end());
        }
        return all;
    }
};

TestNal MakeNal(uint32_t frameIndex, uint32_t totalPackets, bool interleaved,
                size_t lastPacketSize = 313)
{
    TestNal nal;
    nal.frameIndex = frameIndex;
    nal.totalPackets = totalPackets;
    for (uint32_t i = 0; i < totalPackets; i++)
    {
        const size_t size =
            (i == totalPackets - 1) ? lastPacketSize : protocol::MAX_PACKET_PAYLOAD;
        std::vector<uint8_t> payload(size);
        for (size_t j = 0; j < size; j++)
        {
            payload[j] = static_cast<uint8_t>((frameIndex * 31 + i * 7 + j) % 251);
        }
        nal.payloads.push_back(std::move(payload));
    }

    const fec::GroupLayout layout{totalPackets, interleaved};
    for (uint32_t g = 0; g < layout.Count(); g++)
    {
        const uint32_t members = layout.MemberCount(g);
        std::array<const uint8_t*, protocol::FEC_GROUP_SIZE> ptrs = {};
        std::array<uint16_t, protocol::FEC_GROUP_SIZE> sizes = {};
        for (uint32_t k = 0; k < members; k++)
        {
            const uint32_t idx = layout.Member(g, k);
            ptrs[k] = nal.payloads[idx].data();
            sizes[k] = static_cast<uint16_t>(nal.payloads[idx].size());
        }
        std::vector<uint8_t> parity(protocol::MAX_PACKET_PAYLOAD);
        fec::Encode(ptrs.data(), sizes.data(), members, parity.data());
        nal.parity.push_back(std::move(parity));
        nal.parityLastMemberSize.push_back(sizes[members - 1]);
    }
    return nal;
}

protocol::VideoPacketHeader DataHeader(const TestNal& nal, uint32_t packetIndex)
{
    protocol::VideoPacketHeader header = {};
    header.frameIndex = nal.frameIndex;
    header.packetIndex = static_cast<uint16_t>(packetIndex);
    header.totalPackets = static_cast<uint16_t>(nal.totalPackets);
    header.payloadSize = static_cast<uint16_t>(nal.payloads[packetIndex].size());
    header.flags = protocol::VIDEO_FLAG_STEREO;
    header.presentationTimeNs = 1000 + nal.frameIndex;
    return header;
}

protocol::VideoPacketHeader ParityHeader(const TestNal& nal, uint32_t groupIndex)
{
    protocol::VideoPacketHeader header = {};
    header.frameIndex = nal.frameIndex;
    header.packetIndex = static_cast<uint16_t>(groupIndex);
    header.totalPackets = static_cast<uint16_t>(nal.totalPackets);
    header.payloadSize = static_cast<uint16_t>(protocol::MAX_PACKET_PAYLOAD);
    header.flags = protocol::VIDEO_FLAG_FEC | protocol::VIDEO_FLAG_STEREO;
    header.fecGroupLastPacketPayloadSize = nal.parityLastMemberSize[groupIndex];
    header.presentationTimeNs = 1000 + nal.frameIndex;
    return header;
}

struct Harness
{
    VideoFrameAssembler assembler;
    std::vector<std::vector<uint8_t>> deliveredFrames;
    std::vector<uint32_t> deliveredIndices;
    std::vector<VideoFrameAssembler::AbandonedFrame> abandoned;
    std::vector<uint32_t> incompleteIndices;
    std::vector<uint32_t> incompleteMissing;
    uint32_t fecRecoveries = 0;

    explicit Harness(bool interleaved)
    {
        assembler.SetInterleaved(interleaved);
        assembler.SetOnFrame([this](const VideoFrameAssembler::Frame& frame) {
            deliveredFrames.emplace_back(frame.data, frame.data + frame.size);
            deliveredIndices.push_back(frame.frameIndex);
        });
        assembler.SetOnFrameAbandoned(
            [this](const VideoFrameAssembler::AbandonedFrame& frame) {
                abandoned.push_back(frame);
            });
        assembler.SetOnFecRecovery(
            [this](uint32_t, uint32_t, uint32_t) { fecRecoveries++; });
        assembler.SetOnFrameIncomplete(
            [this](const VideoFrameAssembler::AbandonedFrame& frame) {
                incompleteIndices.push_back(frame.frameIndex);
                incompleteMissing.push_back(frame.totalPackets - frame.receivedPackets);
            });
    }

    // Retransmit-wait mode with the test's clock; keyframe marks the packet VIDEO_FLAG_KEYFRAME.
    void Data(const TestNal& nal, uint32_t packetIndex, int64_t nowUs, bool keyframe = false)
    {
        protocol::VideoPacketHeader header = DataHeader(nal, packetIndex);
        if (keyframe)
        {
            header.flags |= protocol::VIDEO_FLAG_KEYFRAME;
        }
        assembler.ProcessPacket(header, nal.payloads[packetIndex].data(),
                                nal.payloads[packetIndex].size(), nowUs);
    }

    void Parity(const TestNal& nal, uint32_t groupIndex, int64_t nowUs)
    {
        assembler.ProcessPacket(ParityHeader(nal, groupIndex), nal.parity[groupIndex].data(),
                                nal.parity[groupIndex].size(), nowUs);
    }

    void All(const TestNal& nal, int64_t nowUs, bool keyframe = false)
    {
        for (uint32_t i = 0; i < nal.totalPackets; i++)
        {
            Data(nal, i, nowUs, keyframe);
        }
    }

    void Data(const TestNal& nal, uint32_t packetIndex)
    {
        assembler.ProcessPacket(DataHeader(nal, packetIndex),
                                nal.payloads[packetIndex].data(),
                                nal.payloads[packetIndex].size());
    }

    void Parity(const TestNal& nal, uint32_t groupIndex)
    {
        assembler.ProcessPacket(ParityHeader(nal, groupIndex),
                                nal.parity[groupIndex].data(),
                                nal.parity[groupIndex].size());
    }
};

} // namespace

TEST_CASE("Lossless frames deliver once with intact bytes under both layouts",
          "[fec][assembler]")
{
    for (const bool interleaved : {false, true})
    {
        Harness harness(interleaved);
        const TestNal nal = MakeNal(7, 25, interleaved);
        for (uint32_t i = 0; i < nal.totalPackets; i++)
        {
            harness.Data(nal, i);
        }
        REQUIRE(harness.deliveredFrames.size() == 1);
        REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
        REQUIRE(harness.abandoned.empty());
        REQUIRE(harness.fecRecoveries == 0);
    }
}

TEST_CASE("An adjacent burst is recovered when the parity arrives, not a frame later",
          "[fec][assembler]")
{
    Harness harness(true);
    const TestNal nal = MakeNal(9, 25, true);
    // Adjacent losses: the headline case interleaving exists for. With the old
    // "almost complete" gate this recovery was deferred to the next frame's first packet.
    for (uint32_t i = 0; i < nal.totalPackets; i++)
    {
        if (i == 10 || i == 11)
        {
            continue;
        }
        harness.Data(nal, i);
    }
    REQUIRE(harness.deliveredFrames.empty());
    const fec::GroupLayout layout{nal.totalPackets, true};
    harness.Parity(nal, layout.GroupOf(10));
    REQUIRE(harness.deliveredFrames.empty());  // one parity covers only one of the two losses
    harness.Parity(nal, layout.GroupOf(11));

    // Delivered by the parity itself: no packet of any later frame was needed.
    REQUIRE(harness.deliveredFrames.size() == 1);
    REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
    REQUIRE(harness.fecRecoveries == 2);
    REQUIRE(harness.abandoned.empty());
}

TEST_CASE("A burst as long as the group count is recovered; one longer is not",
          "[fec][assembler]")
{
    const uint32_t totalPackets = 30;
    const fec::GroupLayout layout{totalPackets, true};
    const uint32_t groupCount = layout.Count();

    {
        Harness harness(true);
        const TestNal nal = MakeNal(11, totalPackets, true);
        for (uint32_t i = 0; i < totalPackets; i++)
        {
            if (i >= 5 && i < 5 + groupCount)
            {
                continue;
            }
            harness.Data(nal, i);
        }
        for (uint32_t g = 0; g < groupCount; g++)
        {
            harness.Parity(nal, g);
        }
        REQUIRE(harness.deliveredFrames.size() == 1);
        REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
        REQUIRE(harness.fecRecoveries == groupCount);
    }

    {
        Harness harness(true);
        const TestNal nal = MakeNal(12, totalPackets, true);
        for (uint32_t i = 0; i < totalPackets; i++)
        {
            if (i >= 5 && i < 5 + groupCount + 1)
            {
                continue;
            }
            harness.Data(nal, i);
        }
        for (uint32_t g = 0; g < groupCount; g++)
        {
            harness.Parity(nal, g);
        }
        REQUIRE(harness.deliveredFrames.empty());

        // The next frame's first packet abandons the unrecoverable frame (NACK path) but must
        // still be stored for its own frame: losing it would turn one lost frame into two.
        const TestNal next = MakeNal(13, 3, true);
        harness.Data(next, 0);
        REQUIRE(harness.abandoned.size() == 1);
        REQUIRE(harness.abandoned[0].frameIndex == 12);
        harness.Data(next, 1);
        harness.Data(next, 2);
        REQUIRE(harness.deliveredFrames.size() == 1);
        REQUIRE(harness.deliveredFrames[0] == next.ExpectedBytes());
        REQUIRE(harness.deliveredIndices[0] == 13);
    }
}

TEST_CASE("Contiguous groups recover a single loss as soon as their parity arrives",
          "[fec][assembler]")
{
    Harness harness(false);
    const TestNal nal = MakeNal(21, 25, false);
    for (uint32_t i = 0; i < nal.totalPackets; i++)
    {
        if (i == 4)
        {
            continue;
        }
        harness.Data(nal, i);
        // Inline schedule: parity follows its group's last data packet.
        const uint32_t next = i + 1;
        if (next % protocol::FEC_GROUP_SIZE == 0 || next == nal.totalPackets)
        {
            harness.Parity(nal, i / protocol::FEC_GROUP_SIZE);
        }
    }
    REQUIRE(harness.deliveredFrames.size() == 1);
    REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
    REQUIRE(harness.fecRecoveries == 1);
}

TEST_CASE("A recovered short last packet keeps its true size", "[fec][assembler]")
{
    for (const bool interleaved : {false, true})
    {
        Harness harness(interleaved);
        const TestNal nal = MakeNal(31, 25, interleaved, 47);
        const uint32_t last = nal.totalPackets - 1;
        for (uint32_t i = 0; i < nal.totalPackets; i++)
        {
            if (i != last)
            {
                harness.Data(nal, i);
            }
        }
        const fec::GroupLayout layout{nal.totalPackets, interleaved};
        harness.Parity(nal, layout.GroupOf(last));
        REQUIRE(harness.deliveredFrames.size() == 1);
        REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
    }
}

TEST_CASE("Parity from another NAL sharing the frameIndex is rejected", "[fec][assembler]")
{
    // The server reuses one frameIndex across a frame's NALs (SPS/PPS then IDR), each numbering
    // packets and groups from zero. A reordered parity packet from the small NAL must not
    // occupy the large NAL's parity slot; totalPackets is the discriminator.
    Harness harness(true);
    const TestNal small = MakeNal(40, 1, true, 21);
    const TestNal large = MakeNal(40, 25, true);

    harness.Data(small, 0);
    REQUIRE(harness.deliveredFrames.size() == 1);

    // Large NAL starts; drop packet 0 (a group-0 member under both layouts).
    for (uint32_t i = 1; i < large.totalPackets; i++)
    {
        harness.Data(large, i);
    }
    // The small NAL's parity arrives late, reordered behind the large NAL's data.
    harness.Parity(small, 0);
    REQUIRE(harness.deliveredFrames.size() == 1);  // must not "recover" from foreign parity

    // The large NAL's own group-0 parity must still be accepted and recover correctly.
    const fec::GroupLayout layout{large.totalPackets, true};
    harness.Parity(large, layout.GroupOf(0));
    REQUIRE(harness.deliveredFrames.size() == 2);
    REQUIRE(harness.deliveredFrames[1] == large.ExpectedBytes());
}

TEST_CASE("A new NAL with the same frameIndex but different totalPackets flushes the previous",
          "[fec][assembler]")
{
    Harness harness(false);
    const TestNal first = MakeNal(50, 25, false);
    const TestNal second = MakeNal(50, 3, false, 99);

    harness.Data(first, 0);
    harness.Data(first, 1);
    harness.Data(second, 0);
    REQUIRE(harness.abandoned.size() == 1);
    REQUIRE(harness.abandoned[0].frameIndex == 50);
    REQUIRE(harness.abandoned[0].totalPackets == 25);

    harness.Data(second, 1);
    harness.Data(second, 2);
    REQUIRE(harness.deliveredFrames.size() == 1);
    REQUIRE(harness.deliveredFrames[0] == second.ExpectedBytes());
}

TEST_CASE("Duplicate data and parity packets are ignored", "[fec][assembler]")
{
    Harness harness(true);
    const TestNal nal = MakeNal(60, 12, true);
    for (uint32_t i = 0; i + 1 < nal.totalPackets; i++)
    {
        harness.Data(nal, i);
        harness.Data(nal, i);
    }
    const fec::GroupLayout layout{nal.totalPackets, true};
    const uint32_t lastGroup = layout.GroupOf(nal.totalPackets - 1);
    harness.Parity(nal, lastGroup);
    harness.Parity(nal, lastGroup);
    REQUIRE(harness.deliveredFrames.size() == 1);
    REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
    REQUIRE(harness.fecRecoveries == 1);
}

// ---- Retransmit-wait mode ------------------------------------------------------------------
// Measured on the live chain (2026-10-18): a frame missing 7-46 packets when the next frame
// starts was abandoned at once, its NACK retransmit arrived ~5 ms later to an assembler that had
// moved on, and a keyframe followed. These cases pin the parked-NAL path that replaces that.

namespace
{
constexpr int64_t kWait = VideoFrameAssembler::kRecommendedRetransmitWaitUs;

// A 40-packet NAL (one ~50 kB P-frame) losing an 8-packet burst: two losses in each of its four
// interleaved FEC groups, so parity alone cannot recover any of it.
constexpr uint32_t kBurstFirst = 10;
constexpr uint32_t kBurstLast = 17;

void SendWithBurstLoss(Harness& harness, const TestNal& nal, int64_t nowUs)
{
    for (uint32_t i = 0; i < nal.totalPackets; i++)
    {
        if (i < kBurstFirst || i > kBurstLast)
        {
            harness.Data(nal, i, nowUs);
        }
    }
    for (uint32_t g = 0; g < nal.parity.size(); g++)
    {
        harness.Parity(nal, g, nowUs);
    }
}

void Retransmit(Harness& harness, const TestNal& nal, int64_t nowUs)
{
    for (uint32_t i = kBurstFirst; i <= kBurstLast; i++)
    {
        harness.Data(nal, i, nowUs);
    }
}
} // namespace

TEST_CASE("A burst retransmitted within the wait delivers the parked NAL first, then the newer "
          "frames in order",
          "[assembler][nack]")
{
    Harness harness(true);
    harness.assembler.SetRetransmitWaitUs(kWait);
    const TestNal lossy = MakeNal(100, 40, true);
    const TestNal next = MakeNal(101, 40, true);
    const TestNal after = MakeNal(102, 40, true);

    SendWithBurstLoss(harness, lossy, 0);
    REQUIRE(harness.incompleteIndices.empty());

    harness.All(next, 11000);
    REQUIRE(harness.incompleteIndices == std::vector<uint32_t>{100});
    REQUIRE(harness.incompleteMissing == std::vector<uint32_t>{kBurstLast - kBurstFirst + 1});
    REQUIRE(harness.deliveredIndices.empty());  // 101 is complete but must wait behind 100

    harness.Data(after, 0, 15000);
    Retransmit(harness, lossy, 16000);  // 5 ms after the NACK, inside the 12 ms wait
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{100, 101});
    REQUIRE(harness.deliveredFrames[0] == lossy.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[1] == next.ExpectedBytes());

    for (uint32_t i = 1; i < after.totalPackets; i++)
    {
        harness.Data(after, i, 17000);
    }
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{100, 101, 102});
    REQUIRE(harness.abandoned.empty());
    REQUIRE(harness.assembler.ParkedRecoveries() == 1);
}

TEST_CASE("Single-packet NALs sharing a frameIndex queue behind a parked NAL without colliding",
          "[assembler][nack]")
{
    // A keyframe's VPS, SPS and PPS are each one packet under one frameIndex: same frameIndex,
    // same totalPackets. Held behind a parked NAL they must stay three NALs.
    Harness harness(true);
    harness.assembler.SetRetransmitWaitUs(kWait);
    const TestNal lossy = MakeNal(500, 40, true);
    const TestNal vps = MakeNal(501, 1, true, 20);
    const TestNal sps = MakeNal(501, 1, true, 30);
    const TestNal pps = MakeNal(501, 1, true, 40);
    const TestNal idr = MakeNal(501, 60, true);

    SendWithBurstLoss(harness, lossy, 0);
    harness.All(vps, 1000, true);
    harness.All(sps, 1000, true);
    harness.All(pps, 1000, true);
    harness.All(idr, 2000, true);
    REQUIRE(harness.deliveredIndices.empty());

    Retransmit(harness, lossy, 6000);
    REQUIRE(harness.deliveredFrames.size() == 5);
    REQUIRE(harness.deliveredFrames[0] == lossy.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[1] == vps.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[2] == sps.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[3] == pps.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[4] == idr.ExpectedBytes());
    REQUIRE(harness.abandoned.empty());
}

TEST_CASE("A parked NAL whose wait expires is abandoned and newer frames are dropped until a "
          "keyframe",
          "[assembler][nack]")
{
    Harness harness(true);
    harness.assembler.SetRetransmitWaitUs(kWait);
    const TestNal lossy = MakeNal(200, 40, true);
    const TestNal p1 = MakeNal(201, 40, true);
    const TestNal p2 = MakeNal(202, 40, true);
    const TestNal p3 = MakeNal(203, 40, true);
    const TestNal vps = MakeNal(204, 1, true, 20);
    const TestNal idr = MakeNal(204, 60, true);
    const TestNal p5 = MakeNal(205, 40, true);

    SendWithBurstLoss(harness, lossy, 0);
    harness.All(p1, 1000);  // 200 parked at t=1000
    harness.All(p2, 12000);
    REQUIRE(harness.abandoned.empty());  // 11 ms in: still waiting

    harness.assembler.Poll(1000 + kWait);  // the wait is over, no packet needed
    REQUIRE(harness.abandoned.size() == 1);
    REQUIRE(harness.abandoned[0].frameIndex == 200);
    REQUIRE(harness.deliveredIndices.empty());
    REQUIRE(harness.assembler.DiscardedNals() == 2);  // 201 and 202 reference the lost NAL

    // The retransmit that missed the deadline must not restart assembly or abandon anything.
    Retransmit(harness, lossy, 14000);
    harness.All(p3, 15000);
    REQUIRE(harness.deliveredIndices.empty());
    REQUIRE(harness.abandoned.size() == 1);
    REQUIRE(harness.incompleteIndices == std::vector<uint32_t>{200});
    REQUIRE(harness.assembler.LateRetransmits() == kBurstLast - kBurstFirst + 1);

    harness.All(vps, 20000, true);
    harness.All(idr, 20000, true);
    harness.All(p5, 30000);
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{204, 204, 205});
    REQUIRE(harness.deliveredFrames[0] == vps.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[1] == idr.ExpectedBytes());
    REQUIRE(harness.deliveredFrames[2] == p5.ExpectedBytes());
}

TEST_CASE("A third newer frame abandons the parked NAL before the wait expires",
          "[assembler][nack]")
{
    for (const bool discard : {true, false})
    {
        Harness harness(true);
        harness.assembler.SetRetransmitWaitUs(kWait);
        harness.assembler.SetDiscardUntilKeyframe(discard);
        const TestNal lossy = MakeNal(300, 40, true);
        const TestNal p1 = MakeNal(301, 30, true);
        const TestNal p2 = MakeNal(302, 30, true);
        const TestNal p3 = MakeNal(303, 30, true);

        SendWithBurstLoss(harness, lossy, 0);
        harness.All(p1, 100);
        harness.All(p2, 200);
        REQUIRE(harness.abandoned.empty());
        harness.Data(p3, 0, 300);  // 3 newer frames > the 2 that may be buffered
        REQUIRE(harness.abandoned.size() == 1);
        REQUIRE(harness.abandoned[0].frameIndex == 300);
        for (uint32_t i = 1; i < p3.totalPackets; i++)
        {
            harness.Data(p3, i, 400);
        }
        if (discard)
        {
            REQUIRE(harness.deliveredIndices.empty());
        }
        else
        {
            REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{301, 302, 303});
            REQUIRE(harness.deliveredFrames[2] == p3.ExpectedBytes());
        }
    }
}

TEST_CASE("A wait of 0 is the original path: abandon at once, deliver the next, no park",
          "[assembler][nack]")
{
    for (const bool explicitZero : {false, true})
    {
        Harness harness(true);
        if (explicitZero)
        {
            harness.assembler.SetRetransmitWaitUs(kWait);
            harness.assembler.SetRetransmitWaitUs(0);
        }
        REQUIRE(harness.assembler.RetransmitWaitUs() == 0);
        const TestNal lossy = MakeNal(100, 40, true);
        const TestNal next = MakeNal(101, 40, true);

        SendWithBurstLoss(harness, lossy, 0);
        harness.Data(next, 0, 11000);
        REQUIRE(harness.abandoned.size() == 1);
        REQUIRE(harness.abandoned[0].frameIndex == 100);
        REQUIRE(harness.abandoned[0].receivedPackets == 40 - (kBurstLast - kBurstFirst + 1));
        REQUIRE(harness.incompleteIndices.empty());
        for (uint32_t i = 1; i < next.totalPackets; i++)
        {
            harness.Data(next, i, 11000);
        }
        REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{101});
        harness.assembler.Poll(1000000);  // no-op at wait 0
        REQUIRE(harness.abandoned.size() == 1);
    }
}

TEST_CASE("FEC still recovers in retransmit-wait mode without parking", "[assembler][nack][fec]")
{
    Harness harness(true);
    harness.assembler.SetRetransmitWaitUs(kWait);
    const TestNal nal = MakeNal(600, 40, true);
    const fec::GroupLayout layout{nal.totalPackets, true};
    const uint32_t groupCount = layout.Count();
    // One loss per group: exactly what the parity covers.
    for (uint32_t i = 0; i < nal.totalPackets; i++)
    {
        if (i >= 20 && i < 20 + groupCount)
        {
            continue;
        }
        harness.Data(nal, i, 0);
    }
    for (uint32_t g = 0; g < groupCount; g++)
    {
        harness.Parity(nal, g, 0);
    }
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{600});
    REQUIRE(harness.deliveredFrames[0] == nal.ExpectedBytes());
    REQUIRE(harness.fecRecoveries == groupCount);

    const TestNal next = MakeNal(601, 10, true);
    harness.All(next, 1000);
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{600, 601});
    REQUIRE(harness.incompleteIndices.empty());
    REQUIRE(harness.abandoned.empty());
}

TEST_CASE("Parity for a parked NAL arriving late still recovers it; its retransmit is then "
          "ignored",
          "[assembler][nack][fec]")
{
    Harness harness(true);
    harness.assembler.SetRetransmitWaitUs(kWait);
    const TestNal lossy = MakeNal(400, 40, true);
    const TestNal next = MakeNal(401, 40, true);
    const fec::GroupLayout layout{lossy.totalPackets, true};
    const uint32_t lostIndex = 5;
    const uint32_t lostGroup = layout.GroupOf(lostIndex);

    for (uint32_t i = 0; i < lossy.totalPackets; i++)
    {
        if (i != lostIndex)
        {
            harness.Data(lossy, i, 0);
        }
    }
    for (uint32_t g = 0; g < layout.Count(); g++)
    {
        if (g != lostGroup)
        {
            harness.Parity(lossy, g, 0);
        }
    }
    harness.All(next, 1000);  // parks 400 (its parity is still in flight) and NACKs it
    REQUIRE(harness.incompleteIndices == std::vector<uint32_t>{400});
    REQUIRE(harness.incompleteMissing == std::vector<uint32_t>{1});
    REQUIRE(harness.deliveredIndices.empty());

    harness.Parity(lossy, lostGroup, 2000);
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{400, 401});
    REQUIRE(harness.deliveredFrames[0] == lossy.ExpectedBytes());
    REQUIRE(harness.assembler.ParkedRecoveries() == 1);

    // The NACKed packet now arrives: settled, so it must not start a phantom NAL.
    harness.Data(lossy, lostIndex, 6000);
    const TestNal after = MakeNal(402, 10, true);
    harness.All(after, 11000);
    REQUIRE(harness.deliveredIndices == std::vector<uint32_t>{400, 401, 402});
    REQUIRE(harness.abandoned.empty());
    REQUIRE(harness.incompleteIndices.size() == 1);
    REQUIRE(harness.assembler.LateRetransmits() == 1);
}
