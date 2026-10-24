// SPDX-License-Identifier: BSL-1.0
// Decodes the bundled SBS test clip and checks the output is well-formed and
// genuinely stereo (left and right halves differ).
#include "test_util.h"
#include "video_decoder.h"

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <set>
#include <string>
#include <vector>

#ifndef TEST_ASSET_DIR
#define TEST_ASSET_DIR "assets"
#endif

// Mean absolute per-channel difference between the two SBS halves of an RGBA frame.
static double halvesDiffer(const std::vector<uint8_t>& rgba, int w, int h)
{
    const int half = w / 2;
    long long acc = 0;
    long long n = 0;
    for (int y = 0; y < h; y += 8) {
        for (int x = 0; x < half; x += 8) {
            const uint8_t* l = &rgba[((size_t)y * w + x) * 4];
            const uint8_t* r = &rgba[((size_t)y * w + (x + half)) * 4];
            for (int c = 0; c < 3; c++) { acc += std::abs((int)l[c] - (int)r[c]); ++n; }
        }
    }
    return n ? (double)acc / n : 0.0;
}

void test_video_decoder()
{
    SECTION("video_decoder: file mode decodes SBS HEVC");
    VideoDecoder dec;
    const std::string path = std::string(TEST_ASSET_DIR) + "/stereo_test.hevc";
    CHECK(dec.Open(path));

    std::vector<uint8_t> rgba;
    int w = 0, h = 0;
    bool got = dec.NextFrameRGBA(rgba, w, h);
    CHECK(got);
    CHECK(w == 1920);
    CHECK(h == 1080);
    CHECK(rgba.size() == (size_t)w * h * 4);

    // The generated clip has distinct left/right halves — prove it decoded real
    // content (not a blank/constant frame) and is actually side-by-side stereo.
    if (got && w > 0) {
        double diff = halvesDiffer(rgba, w, h);
        CHECK(diff > 5.0); // clearly different halves, not noise-level
    }

    // Looping: many NextFrame calls keep returning frames past end-of-stream.
    int frames = 1;
    for (int i = 0; i < 120; i++)
        if (dec.NextFrameRGBA(rgba, w, h)) ++frames;
    CHECK(frames >= 100);
    dec.Close();
}

// Split an Annex-B stream into NALs, each with its start code (the receiver's delivery unit).
static std::vector<std::vector<uint8_t>> splitNals(const std::vector<uint8_t>& es)
{
    std::vector<size_t> starts;
    for (size_t i = 0; i + 3 <= es.size(); i++)
        if (es[i] == 0 && es[i + 1] == 0 && es[i + 2] == 1) { starts.push_back(i > 0 && es[i - 1] == 0 ? i - 1 : i); i += 2; }
    std::vector<std::vector<uint8_t>> out;
    for (size_t k = 0; k < starts.size(); k++) {
        const size_t end = k + 1 < starts.size() ? starts[k + 1] : es.size();
        out.emplace_back(es.begin() + starts[k], es.begin() + end);
    }
    return out;
}

static int hevcType(const std::vector<uint8_t>& nal)
{
    const size_t sc = nal.size() > 3 && nal[2] == 1 ? 3 : 4;
    return nal.size() > sc ? (nal[sc] >> 1) & 0x3f : -1;
}

void test_access_units()
{
    SECTION("video_decoder: access units assembled from NALs, slice completes");
    {
        std::vector<uint8_t> au;
        const uint8_t vps[] = {0, 0, 0, 1, 0x40, 0x01, 0xAA};
        const uint8_t sps[] = {0, 0, 1, 0x42, 0x01, 0xBB};
        const uint8_t bare[] = {0x44, 0x01, 0xCC};          // PPS without a start code
        const uint8_t slice[] = {0, 0, 0, 1, 0x26, 0x01, 0xDD};  // IDR_W_RADL (type 19)
        CHECK(!VideoDecoder::AppendNalToAccessUnit(au, vps, sizeof(vps)));
        CHECK(!VideoDecoder::AppendNalToAccessUnit(au, sps, sizeof(sps)));
        CHECK(!VideoDecoder::AppendNalToAccessUnit(au, bare, sizeof(bare)));
        CHECK(au.size() == sizeof(vps) + sizeof(sps) + 4 + sizeof(bare));   // start code added
        CHECK(VideoDecoder::AppendNalToAccessUnit(au, slice, sizeof(slice)));
        CHECK(au.size() == sizeof(vps) + sizeof(sps) + 4 + sizeof(bare) + sizeof(slice));
        const uint8_t empty[] = {0, 0, 0, 1};
        CHECK(!VideoDecoder::AppendNalToAccessUnit(au, empty, sizeof(empty)));   // a start code alone adds nothing
    }

    SECTION("video_decoder: stream mode decodes the clip NAL by NAL, every frame with a submitted pts");
    std::ifstream f(std::string(TEST_ASSET_DIR) + "/stereo_test.hevc", std::ios::binary);
    std::vector<uint8_t> es((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    CHECK(!es.empty());
    const auto nals = splitNals(es);
    CHECK(nals.size() > 90);
    VideoDecoder dec;
    CHECK(dec.OpenStream(VideoDecoder::Codec::H265));
    std::set<int64_t> submitted;
    std::vector<int64_t> outPts;
    int64_t pts = 1000000;
    std::vector<uint8_t> rgba;
    int w = 0, h = 0;
    for (const auto& nal : nals) {
        dec.SubmitNal(nal.data(), nal.size(), pts);
        if (hevcType(nal) >= 0 && hevcType(nal) < 32) { submitted.insert(pts); pts += 33333; }
        int64_t got = 0;
        if (dec.TakeLatestRGBA(rgba, w, h, &got)) outPts.push_back(got);
    }
    CHECK(outPts.size() >= 80);   // 90 frames; the clip's B-frame reorder holds a few back
    bool allSubmitted = true;
    for (int64_t p : outPts) if (!submitted.count(p)) allSubmitted = false;
    CHECK(allSubmitted);
    CHECK(w == 1920 && h == 1080);
    dec.Close();
}

// Decode the clip NAL by NAL; after every `takeEvery` slices the render loop runs `takes` display
// periods, one take each (drain: take everything ready). Returns the pts taken, in order.
static std::vector<int64_t> decodeTaking(VideoDecoder& dec, const std::vector<std::vector<uint8_t>>& nals,
                                         int takeEvery, bool drain, int& slicesOut, int takes = 1)
{
    std::vector<int64_t> taken;
    std::vector<uint8_t> rgba;
    int w = 0, h = 0, slices = 0;
    int64_t pts = 1000000;
    for (const auto& nal : nals) {
        dec.SubmitNal(nal.data(), nal.size(), pts);
        if (hevcType(nal) >= 0 && hevcType(nal) < 32) {
            pts += 33333;
            if (++slices % takeEvery == 0) {
                int64_t got = 0;
                for (int t = 0; drain || t < takes; t++) {
                    if (!dec.TakeLatestRGBA(rgba, w, h, &got)) break;
                    taken.push_back(got);
                }
            }
        }
    }
    int64_t got = 0;
    while (drain && dec.TakeLatestRGBA(rgba, w, h, &got)) taken.push_back(got);
    slicesOut = slices;
    return taken;
}

static bool isSubsequence(const std::vector<int64_t>& sub, const std::vector<int64_t>& seq)
{
    size_t j = 0;
    for (int64_t v : seq) if (j < sub.size() && sub[j] == v) j++;
    return j == sub.size();
}

void test_frame_queue()
{
    std::ifstream f(std::string(TEST_ASSET_DIR) + "/stereo_test.hevc", std::ios::binary);
    std::vector<uint8_t> es((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const auto nals = splitNals(es);
    int slices = 0;

    // Reference: every frame the decoder puts out, in its output order (the clip has B-frames,
    // so that is not submit order). Depth 3, drained after every submit, never cuts a backlog.
    std::vector<int64_t> all;
    {
        VideoDecoder dec;
        CHECK(dec.OpenStream(VideoDecoder::Codec::H265));
        dec.SetQueueDepth(3);
        all = decodeTaking(dec, nals, 1, true, slices);
        dec.Close();
    }

    SECTION("video_decoder: two frames in one display period both reach the screen");
    {
        VideoDecoder dec;
        CHECK(dec.OpenStream(VideoDecoder::Codec::H265));
        CHECK(dec.QueueDepth() == 2);
        CHECK(all.size() + 6 >= (size_t)slices);
        // Frames land in pairs (the sender's pacing jitter across the pickup boundary) and the
        // render loop takes one per period: a frame that lands while another waits queues
        // behind it instead of replacing it. The latest-only slot loses half of them here.
        const auto taken = decodeTaking(dec, nals, 2, false, slices, 2);
        std::vector<int64_t> rest = taken;
        std::vector<uint8_t> rgba; int w = 0, h = 0; int64_t p = 0;
        while (dec.TakeLatestRGBA(rgba, w, h, &p)) rest.push_back(p);
        CHECK(isSubsequence(rest, all));
        CHECK(rest.size() + 2 >= all.size());
        dec.Close();
    }

    SECTION("video_decoder: a backlog is cut to the newest `depth` frames");
    {
        VideoDecoder dec;
        CHECK(dec.OpenStream(VideoDecoder::Codec::H265));
        // One take per three submits: the queue catches up instead of falling further behind.
        const auto taken = decodeTaking(dec, nals, 3, false, slices);
        CHECK(isSubsequence(taken, all));
        CHECK(taken.size() + 2 >= (size_t)slices / 3);
        // The last frame taken is within depth (+ the reorder delay) of the newest decoded.
        size_t idx = 0;
        for (size_t i = 0; i < all.size(); i++) if (!taken.empty() && all[i] == taken.back()) idx = i;
        CHECK(idx + 2 + 6 >= all.size());
        dec.Close();
    }

    SECTION("video_decoder: depth 0 keeps the latest-only slot");
    {
        VideoDecoder dec;
        CHECK(dec.OpenStream(VideoDecoder::Codec::H265));
        dec.SetQueueDepth(0);
        const auto taken = decodeTaking(dec, nals, 3, false, slices);
        CHECK(isSubsequence(taken, all));
        std::vector<uint8_t> rgba; int w = 0, h = 0;
        dec.TakeLatestRGBA(rgba, w, h);
        CHECK(!dec.TakeLatestRGBA(rgba, w, h));   // one slot: nothing left after one take
        dec.Close();
    }
}
