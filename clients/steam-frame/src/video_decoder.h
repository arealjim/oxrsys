// Frame VR client — video decode layer
// SPDX-License-Identifier: BSL-1.0
//
// FFmpeg HEVC/H264 -> RGBA, portable across Mac (dev) and Linux (Frame).
// Two feed modes behind one narrow seam:
//   - File mode (V1):   Open(path) + NextFrameRGBA()          [pull, one thread]
//   - Stream mode (V2):  OpenStream(codec) + SubmitNal()      [push, net thread]
//                        + TakeLatestRGBA()                    [render thread]
// The guts (software avcodec now) can be swapped for Vulkan Video / VAAPI
// hardware decode later without touching either seam.

#pragma once
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

class VideoDecoder {
public:
    enum class Codec { H265, H264 };

    VideoDecoder() = default;
    ~VideoDecoder();
    VideoDecoder(const VideoDecoder&) = delete;
    VideoDecoder& operator=(const VideoDecoder&) = delete;

    // ---- File mode ----
    bool Open(const std::string& path);
    bool NextFrameRGBA(std::vector<uint8_t>& out, int& width, int& height);

    // ---- Stream mode ----
    // FRAME_CLIENT_DECODER names a libavcodec decoder to use instead of the default for the
    // codec (hevc_v4l2m2m on the Steam Frame). Hardware wrappers size their buffers at open
    // and refuse a 0x0 context; for them the open is deferred to ReconfigureStream.
    bool OpenStream(Codec codec);
    // Called once the server's announce gives the encoded size, before the first NAL: reopens
    // the decoder with that size preset (a no-op if it is already open at that size).
    bool ReconfigureStream(int width, int height);
    // Called from the network thread with a complete encoded access unit. ptsUs is the
    // frame's presentation timestamp; it rides through parser and codec delays so the
    // decoded output can be matched to its per-frame metadata (render pose, foveation
    // centre) rather than to whatever arrived most recently.
    // HEVC: the receiver delivers one NAL per call (VPS, SPS, PPS, SEI, then the slice, all with the
    // frame's pts). They are gathered into one access unit, sent to the decoder as one packet with
    // that pts when the slice arrives, and the decoder is polled briefly for that frame's own output.
    void SubmitNal(const uint8_t* data, size_t size, int64_t ptsUs);
    // Called from the render thread; true if a newer decoded frame was taken.
    bool TakeLatestRGBA(std::vector<uint8_t>& out, int& width, int& height,
                        int64_t* outPtsUs = nullptr);

    void Close();
    int Width() const { return width_; }
    int Height() const { return height_; }
    double LastDecodeMs() const;  // wall-clock of the last decode+convert (stream mode)

    // True (and cleared) if a decode error occurred since the last call — the
    // caller should ask the server for a keyframe (IDR) to resync the decoder.
    bool TakeNeedKeyframe();

    // Appends one Annex-B NAL (a start code is added if it has none) to an HEVC access unit
    // being assembled. True when the NAL is a slice, i.e. the access unit is complete.
    static bool AppendNalToAccessUnit(std::vector<uint8_t>& au, const uint8_t* data, size_t size);
    // Non-slice NALs accumulate until a slice arrives; a run without one is dropped at this size
    // (about 3x the largest IDR at 50 Mbit/s).
    static constexpr size_t kMaxAccessUnitBytes = 2u << 20;

private:
    bool openInternal();     // file
    bool openCodecOnly(int avCodecId);
    void closeInternal();
    void convertToLatest();  // frame_ -> latest_ (RGBA), stream mode
    void sendAccessUnit(uint8_t* data, int size, int64_t ptsUs);

    std::string path_;
    int width_ = 0, height_ = 0;
    int videoStream_ = -1;
    bool streamMode_ = false;

    struct AVFormatContext* fmt_ = nullptr;
    struct AVCodecContext* codec_ = nullptr;
    struct AVCodecParserContext* parser_ = nullptr;
    struct SwsContext* sws_ = nullptr;
    struct AVFrame* frame_ = nullptr;
    struct AVPacket* packet_ = nullptr;

    // Stream mode: latest decoded frame handed net thread -> render thread.
    std::mutex latestMutex_;
    std::vector<uint8_t> latest_;
    int latestW_ = 0, latestH_ = 0;
    int64_t latestPtsUs_ = 0;
    int streamCodecId_ = 0;
    int expectedW_ = 0, expectedH_ = 0;
    std::vector<uint8_t> au_;      // HEVC access unit being assembled (stream mode)
    uint64_t noPtsFrames_ = 0;
    bool latestFresh_ = false;
    double lastDecodeMs_ = 0.0;
    bool needKeyframe_ = false;
};
