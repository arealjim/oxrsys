// SPDX-License-Identifier: MPL-2.0

#include <catch2/catch_test_macros.hpp>

#include "EncoderSessionKind.h"

using oxrsys::ClassifyEncoderSession;
using oxrsys::EncoderSessionKind;
using oxrsys::SessionWritesVideoRangeFromBGRA;

TEST_CASE("Encoder session kind: the hardware property wins when it answers", "[encoder-session]")
{
    CHECK(ClassifyEncoderSession(true, true, "com.apple.videotoolbox.videoencoder.ave.hevc") ==
          EncoderSessionKind::Hardware);
    CHECK(ClassifyEncoderSession(true, false, "com.apple.videotoolbox.videoencoder.hevc") ==
          EncoderSessionKind::Software);
    // Even an id that looks like hardware does not override an explicit "no".
    CHECK(ClassifyEncoderSession(true, false, "com.apple.videotoolbox.videoencoder.ave.hevc") ==
          EncoderSessionKind::Software);
}

TEST_CASE("Encoder session kind: low-latency rtvc session is hardware although the property fails",
          "[encoder-session]")
{
    // Measured 2026-10-17: EnableLowLatencyRateControl -> encoder id hevc.rtvc, and the
    // UsingHardwareAcceleratedVideoEncoder query returns kVTPropertyNotSupportedErr.
    CHECK(ClassifyEncoderSession(false, false, "com.apple.videotoolbox.videoencoder.hevc.rtvc") ==
          EncoderSessionKind::Hardware);
    CHECK(ClassifyEncoderSession(false, false, "com.apple.videotoolbox.videoencoder.ave.avc") ==
          EncoderSessionKind::Hardware);
}

TEST_CASE("Encoder session kind: no answer and a software id is software; no id at all is unknown",
          "[encoder-session]")
{
    CHECK(ClassifyEncoderSession(false, false, "com.apple.videotoolbox.videoencoder.h264.gva") ==
          EncoderSessionKind::Software);
    CHECK(ClassifyEncoderSession(false, false, "") == EncoderSessionKind::Unknown);
}

TEST_CASE("Encoder session kind: only the classic ave hardware encoder keeps video range from BGRA",
          "[encoder-session]")
{
    CHECK(SessionWritesVideoRangeFromBGRA(EncoderSessionKind::Hardware,
                                          "com.apple.videotoolbox.videoencoder.ave.hevc"));
    CHECK(SessionWritesVideoRangeFromBGRA(EncoderSessionKind::Hardware,
                                          "com.apple.videotoolbox.videoencoder.ave.avc"));
    // The low-latency rtvc session is hardware but writes full range for BGRA: convert.
    CHECK_FALSE(SessionWritesVideoRangeFromBGRA(EncoderSessionKind::Hardware,
                                                "com.apple.videotoolbox.videoencoder.hevc.rtvc"));
    CHECK_FALSE(SessionWritesVideoRangeFromBGRA(EncoderSessionKind::Software,
                                                "com.apple.videotoolbox.videoencoder.hevc"));
    CHECK_FALSE(SessionWritesVideoRangeFromBGRA(EncoderSessionKind::Unknown, ""));
}
