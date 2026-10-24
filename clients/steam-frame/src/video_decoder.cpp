// SPDX-License-Identifier: BSL-1.0
#include "video_decoder.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#define LOG_ERR(fmt, ...) fprintf(stderr, "[ERROR] VideoDecoder: " fmt "\n", ##__VA_ARGS__)
#define LOG_INF(fmt, ...) fprintf(stderr, "[INFO] VideoDecoder: " fmt "\n", ##__VA_ARGS__)

VideoDecoder::~VideoDecoder() { Close(); }

// ---- shared helpers --------------------------------------------------------

// Convert the current decoded frame_ to tightly-packed RGBA into `out`.
static bool frameToRGBA(SwsContext*& sws, AVFrame* frame, std::vector<uint8_t>& out)
{
    sws = sws_getCachedContext(sws, frame->width, frame->height,
                               (AVPixelFormat)frame->format,
                               frame->width, frame->height, AV_PIX_FMT_RGBA,
                               SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (!sws) return false;
    out.resize((size_t)frame->width * frame->height * 4);
    uint8_t* dst[4] = {out.data(), nullptr, nullptr, nullptr};
    int dstStride[4] = {frame->width * 4, 0, 0, 0};
    sws_scale(sws, frame->data, frame->linesize, 0, frame->height, dst, dstStride);
    return true;
}

// ---- file mode -------------------------------------------------------------

bool VideoDecoder::openInternal()
{
    if (avformat_open_input(&fmt_, path_.c_str(), nullptr, nullptr) != 0) {
        LOG_ERR("cannot open %s", path_.c_str());
        return false;
    }
    if (avformat_find_stream_info(fmt_, nullptr) < 0) { LOG_ERR("no stream info"); return false; }
    videoStream_ = -1;
    for (unsigned i = 0; i < fmt_->nb_streams; i++)
        if (fmt_->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_VIDEO) { videoStream_ = (int)i; break; }
    if (videoStream_ < 0) { LOG_ERR("no video stream"); return false; }

    AVCodecParameters* par = fmt_->streams[videoStream_]->codecpar;
    const AVCodec* dec = avcodec_find_decoder(par->codec_id);
    if (!dec) { LOG_ERR("no decoder for codec %d", par->codec_id); return false; }
    codec_ = avcodec_alloc_context3(dec);
    if (!codec_ || avcodec_parameters_to_context(codec_, par) < 0) { LOG_ERR("ctx init"); return false; }
    if (avcodec_open2(codec_, dec, nullptr) < 0) { LOG_ERR("avcodec_open2"); return false; }

    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    return frame_ && packet_;
}

bool VideoDecoder::Open(const std::string& path)
{
    streamMode_ = false;
    path_ = path;
    if (!openInternal()) { closeInternal(); return false; }
    LOG_INF("opened %s (%s)", path.c_str(), avcodec_get_name(codec_->codec_id));
    return true;
}

bool VideoDecoder::NextFrameRGBA(std::vector<uint8_t>& out, int& width, int& height)
{
    for (int attempt = 0; attempt < 2; attempt++) {
        while (av_read_frame(fmt_, packet_) >= 0) {
            if (packet_->stream_index != videoStream_) { av_packet_unref(packet_); continue; }
            int sret = avcodec_send_packet(codec_, packet_);
            av_packet_unref(packet_);
            if (sret < 0) continue;
            int rret = avcodec_receive_frame(codec_, frame_);
            if (rret == AVERROR(EAGAIN)) continue;
            if (rret < 0) return false;
            width_ = width = frame_->width;
            height_ = height = frame_->height;
            if (!frameToRGBA(sws_, frame_, out)) { LOG_ERR("sws"); return false; }
            return true;
        }
        closeInternal();
        if (!openInternal()) return false; // loop at EOF
    }
    return false;
}

// ---- stream mode -----------------------------------------------------------

bool VideoDecoder::openCodecOnly(int avCodecId)
{
    // The Steam Frame's video decoder is a V4L2 memory-to-memory device, which the system FFmpeg
    // exposes as hevc_v4l2m2m (about 1 ms per frame there; software hevc 7.8 ms on one thread).
    const AVCodec* dec = nullptr;
    if (const char* name = getenv("FRAME_CLIENT_DECODER")) {
        dec = avcodec_find_decoder_by_name(name);
        if (!dec) LOG_ERR("decoder '%s' not found; falling back to the default", name);
    }
    if (!dec) dec = avcodec_find_decoder((AVCodecID)avCodecId);
    if (!dec) { LOG_ERR("no decoder for codec id %d", avCodecId); return false; }
    codec_ = avcodec_alloc_context3(dec);
    if (!codec_) { LOG_ERR("ctx alloc"); return false; }
    if (expectedW_ > 0 && expectedH_ > 0) {
        // v4l2m2m allocates its queues at open (VIDIOC_REQBUFS fails at 0x0).
        codec_->width = codec_->coded_width = expectedW_;
        codec_->height = codec_->coded_height = expectedH_;
    }
    codec_->flags |= AV_CODEC_FLAG_LOW_DELAY;
    // Packet pts are microseconds. hevc_v4l2m2m converts them through the context's time base and
    // returns every frame with AV_NOPTS_VALUE when it is unset (3,600 of 3,600 on the Frame).
    codec_->time_base = AVRational{1, 1000000};
    codec_->pkt_timebase = AVRational{1, 1000000};
    if (avcodec_open2(codec_, dec, nullptr) < 0) { LOG_ERR("avcodec_open2 (stream)"); return false; }
    LOG_INF("decoder %s (%dx%d at open)", dec->name, expectedW_, expectedH_);
    parser_ = av_parser_init(avCodecId);
    if (!parser_) { LOG_ERR("parser init"); return false; }
    frame_ = av_frame_alloc();
    packet_ = av_packet_alloc();
    return frame_ && packet_;
}

bool VideoDecoder::OpenStream(Codec codec)
{
    streamMode_ = true;
    if (const char* q = getenv("FRAME_CLIENT_QUEUE")) SetQueueDepth(atoi(q));
    int id = (codec == Codec::H264) ? AV_CODEC_ID_H264 : AV_CODEC_ID_HEVC;
    streamCodecId_ = id;
    au_.clear();
    if (!openCodecOnly(id)) {
        closeInternal();
        if (getenv("FRAME_CLIENT_DECODER")) {
            // SubmitNal is a no-op until ReconfigureStream opens it at the announced size.
            LOG_INF("stream decoder open deferred until the encoded size is known");
            return true;
        }
        return false;
    }
    LOG_INF("stream decoder open (%s)", avcodec_get_name((AVCodecID)id));
    return true;
}

bool VideoDecoder::ReconfigureStream(int width, int height)
{
    if (!streamMode_ || width <= 0 || height <= 0) return false;
    if (codec_ && width == expectedW_ && height == expectedH_) return true;
    expectedW_ = width;
    expectedH_ = height;
    closeInternal();
    if (!openCodecOnly(streamCodecId_)) { closeInternal(); return false; }
    LOG_INF("stream decoder reopened for %dx%d", width, height);
    return true;
}

void VideoDecoder::convertToLatest()
{
    if (!frameToRGBA(sws_, frame_, scratch_)) return;
    const int64_t ptsUs = frame_->pts != AV_NOPTS_VALUE ? frame_->pts : frame_->best_effort_timestamp;
    std::lock_guard<std::mutex> lk(latestMutex_);
    if (queueDepth_ > 0) {
        if (ringCount_ == kRing) {   // the render loop stopped taking: drop the oldest
            ringHead_ = (ringHead_ + 1) % kRing;
            ringCount_--;
        }
        Ready& r = ring_[(ringHead_ + ringCount_) % kRing];
        r.buf.swap(scratch_);
        r.w = frame_->width;
        r.h = frame_->height;
        r.ptsUs = ptsUs;
        ringCount_++;
        return;
    }
    latest_.swap(scratch_);
    latestW_ = frame_->width;
    latestH_ = frame_->height;
    // Never borrow the latest submit's pts for a frame without one: with a decoder that
    // returns frames late it belongs to a later frame, whose render pose would then be
    // stamped on these pixels. Unmatched is better than mismatched.
    latestPtsUs_ = ptsUs;
    if (frame_->pts == AV_NOPTS_VALUE && (noPtsFrames_++ % 300) == 0)
        LOG_ERR("decoded frame without pts (%llu so far): its render pose cannot be matched",
                (unsigned long long)noPtsFrames_);
    latestFresh_ = true;
}

bool VideoDecoder::AppendNalToAccessUnit(std::vector<uint8_t>& au, const uint8_t* data, size_t size)
{
    size_t sc = 0;
    if (size >= 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1) sc = 4;
    else if (size >= 3 && data[0] == 0 && data[1] == 0 && data[2] == 1) sc = 3;
    if (size <= sc) return false;
    if (sc == 0) {
        static const uint8_t start[4] = {0, 0, 0, 1};
        au.insert(au.end(), start, start + 4);
    }
    au.insert(au.end(), data, data + size);
    const int nalType = (data[sc] >> 1) & 0x3f;
    return nalType < 32;   // VCL (a slice); VPS/SPS/PPS/AUD/SEI ride along with the next slice
}

// Send one access unit and collect what the decoder gives back. hevc_v4l2m2m returns a frame
// about 2 ms after its own submit, never in a receive call straight after it; without the
// poll, each frame waited for the next submit and came out one frame late.
void VideoDecoder::sendAccessUnit(uint8_t* data, int size, int64_t ptsUs)
{
    constexpr double kPollMs = 8.0;
    packet_->data = data;
    packet_->size = size;
    packet_->pts = ptsUs;
    const auto t0 = std::chrono::steady_clock::now();
    int sret = avcodec_send_packet(codec_, packet_);
    if (sret == AVERROR(EAGAIN)) {
        // The decoder's input queue is full: take what it has finished, then offer the packet
        // once more. Only a second refusal drops the access unit.
        while (avcodec_receive_frame(codec_, frame_) >= 0) {
            width_ = frame_->width;
            height_ = frame_->height;
            convertToLatest();
        }
        sret = avcodec_send_packet(codec_, packet_);
    }
    if (sret < 0) {
        std::lock_guard<std::mutex> lk(latestMutex_);
        needKeyframe_ = true;
        return;
    }
    bool gotOne = false;
    for (;;) {
        int rret = avcodec_receive_frame(codec_, frame_);
        if (rret == AVERROR(EAGAIN) && !gotOne &&
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() < kPollMs) {
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            continue;
        }
        if (rret == AVERROR(EAGAIN) || rret == AVERROR_EOF) break;
        gotOne = true;
        if (rret < 0) {  // decode error — a reference frame was likely lost
            std::lock_guard<std::mutex> lk(latestMutex_);
            needKeyframe_ = true;
            break;
        }
        width_ = frame_->width;
        height_ = frame_->height;
        convertToLatest();
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        std::lock_guard<std::mutex> lk(latestMutex_);
        lastDecodeMs_ = ms;
    }
}

void VideoDecoder::SubmitNal(const uint8_t* data, size_t size, int64_t ptsUs)
{
    if (!codec_ || !parser_ || size == 0) return;
    if (streamCodecId_ == AV_CODEC_ID_HEVC) {
        // av_parser_parse2 can only close an access unit when the NEXT one's first bytes arrive,
        // so every frame sat in the parser for a frame interval (0 of 5,400 submits got their own
        // picture out in the same call). The receiver already delivers whole NALs with their
        // frame's pts, so the access unit is assembled here instead.
        if (au_.size() + size > kMaxAccessUnitBytes) {
            LOG_ERR("access unit over %zu bytes without a slice; dropped, keyframe requested",
                    kMaxAccessUnitBytes);
            au_.clear();
            std::lock_guard<std::mutex> lk(latestMutex_);
            needKeyframe_ = true;
            return;
        }
        if (!AppendNalToAccessUnit(au_, data, size)) return;
        au_.insert(au_.end(), AV_INPUT_BUFFER_PADDING_SIZE, 0);
        sendAccessUnit(au_.data(), (int)(au_.size() - AV_INPUT_BUFFER_PADDING_SIZE), ptsUs);
        au_.clear();
        return;
    }
    const uint8_t* p = data;
    size_t remaining = size;
    while (remaining > 0) {
        uint8_t* outData = nullptr;
        int outSize = 0;
        int used = av_parser_parse2(parser_, codec_, &outData, &outSize,
                                    p, (int)remaining, ptsUs, AV_NOPTS_VALUE, 0);
        if (used < 0) return;
        p += used;
        remaining -= (size_t)used;
        if (outSize == 0) continue;

        packet_->data = outData;
        packet_->size = outSize;
        packet_->pts = parser_->pts;
        auto t0 = std::chrono::steady_clock::now();
        if (avcodec_send_packet(codec_, packet_) < 0) {
            std::lock_guard<std::mutex> lk(latestMutex_);
            needKeyframe_ = true;
            continue;
        }
        for (;;) {
            int rret = avcodec_receive_frame(codec_, frame_);
            if (rret == AVERROR(EAGAIN) || rret == AVERROR_EOF) break;
            if (rret < 0) {  // decode error — a reference frame was likely lost
                std::lock_guard<std::mutex> lk(latestMutex_);
                needKeyframe_ = true;
                break;
            }
            width_ = frame_->width;
            height_ = frame_->height;
            convertToLatest();
            double ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
            std::lock_guard<std::mutex> lk(latestMutex_);
            lastDecodeMs_ = ms;
        }
    }
}

void VideoDecoder::SetQueueDepth(int depth)
{
    std::lock_guard<std::mutex> lk(latestMutex_);
    queueDepth_ = depth < 0 ? 0 : (depth > kRing - 1 ? kRing - 1 : depth);
    ringHead_ = ringCount_ = 0;
}

bool VideoDecoder::TakeLatestRGBA(std::vector<uint8_t>& out, int& width, int& height,
                                  int64_t* outPtsUs)
{
    std::lock_guard<std::mutex> lk(latestMutex_);
    if (queueDepth_ > 0) {
        if (ringCount_ == 0) return false;
        while (ringCount_ > queueDepth_) {   // backlog: catch up to the newest `depth` frames
            ringHead_ = (ringHead_ + 1) % kRing;
            ringCount_--;
        }
        Ready& r = ring_[ringHead_];
        ringHead_ = (ringHead_ + 1) % kRing;
        ringCount_--;
        out.swap(r.buf);   // the taken buffer goes back into the ring for reuse
        width = r.w;
        height = r.h;
        if (outPtsUs != nullptr) *outPtsUs = r.ptsUs;
        return true;
    }
    if (!latestFresh_ || latest_.empty()) return false;
    out.swap(latest_);   // rotate buffers: no per-frame copy
    width = latestW_;
    height = latestH_;
    if (outPtsUs != nullptr) {
        *outPtsUs = latestPtsUs_;
    }
    latestFresh_ = false;
    return true;
}

// ---- teardown --------------------------------------------------------------

void VideoDecoder::closeInternal()
{
    if (sws_) { sws_freeContext(sws_); sws_ = nullptr; }
    if (parser_) { av_parser_close(parser_); parser_ = nullptr; }
    if (frame_) av_frame_free(&frame_);
    if (packet_) av_packet_free(&packet_);
    if (codec_) avcodec_free_context(&codec_);
    if (fmt_) avformat_close_input(&fmt_);
    au_.clear();
    videoStream_ = -1;
}

void VideoDecoder::Close() { closeInternal(); }

double VideoDecoder::LastDecodeMs() const
{
    return lastDecodeMs_;
}

bool VideoDecoder::TakeNeedKeyframe()
{
    std::lock_guard<std::mutex> lk(latestMutex_);
    bool v = needKeyframe_;
    needKeyframe_ = false;
    return v;
}
