// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <string_view>

// Is a VideoToolbox compression session hardware-accelerated?
//
// kVTCompressionPropertyKey_UsingHardwareAcceleratedVideoEncoder answers for the
// classic Apple hardware encoder ("com.apple.videotoolbox.videoencoder.ave.hevc"),
// but a session created with EnableLowLatencyRateControl is served by a different
// hardware encoder ("com.apple.videotoolbox.videoencoder.hevc.rtvc") whose session
// returns kVTPropertyNotSupportedErr (-12900) for that key. Trusting the failed
// query as "software" (seen on an M5 Pro) made the runtime warn about a software
// session. So: the property when it answers, the encoder id otherwise.
namespace oxrsys
{

enum class EncoderSessionKind
{
    Hardware,
    Software,
    Unknown,
};

// hardwareQueryAnswered / hardwareQueryValue: the UsingHardwareAcceleratedVideoEncoder
// property (answered = status noErr and a value was returned).
// encoderId: kVTCompressionPropertyKey_EncoderID, or empty when unavailable.
inline EncoderSessionKind ClassifyEncoderSession(bool hardwareQueryAnswered, bool hardwareQueryValue,
                                                 std::string_view encoderId)
{
    if (hardwareQueryAnswered)
    {
        return hardwareQueryValue ? EncoderSessionKind::Hardware : EncoderSessionKind::Software;
    }
    // Apple's hardware encoder ids: ".ave." (Apple Video Encoder, the classic path)
    // and ".rtvc" (the real-time encoder behind low-latency rate control). The
    // software encoders carry neither token.
    if (encoderId.find(".ave.") != std::string_view::npos ||
        encoderId.find(".rtvc") != std::string_view::npos)
    {
        return EncoderSessionKind::Hardware;
    }
    return encoderId.empty() ? EncoderSessionKind::Unknown : EncoderSessionKind::Software;
}

// Does this session write a limited-range (video-range) stream when fed the BGRA
// compose surface? Only the classic hardware encoder (".ave.") does. The software
// encoders and the low-latency rtvc hardware encoder write FULL range for a BGRA
// source (the stream's VUI flipped to full range the moment the rtvc session was
// fed BGRA), so every other session is fed the video-range 4:2:0
// conversion (EncoderSessionColor.h) and keeps the stream limited-range. That
// conversion costs about 1 ms of submit time per 2272x1264 frame.
inline bool SessionWritesVideoRangeFromBGRA(EncoderSessionKind kind, std::string_view encoderId)
{
    return kind == EncoderSessionKind::Hardware &&
           encoderId.find(".ave.") != std::string_view::npos;
}

} // namespace oxrsys
