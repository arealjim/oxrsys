// SPDX-License-Identifier: MPL-2.0
// Render poses arrive from the network and go straight into xrEndFrame: the receiver's bounds.
#include "test_util.h"
#include "NetworkReceiver.h"

#include <cmath>
#include <limits>

void test_receiver()
{
    SECTION("receiver: render pose sanity (finite, within 100 m, unit quaternion)");
    using oxr::NetworkReceiver;
    {
        float p[3] = {0.1f, 1.6f, -0.2f};
        float q[4] = {0.0f, 0.0f, 0.0f, 1.0f};
        CHECK(NetworkReceiver::SanitizePose(p, q));
        CHECK(q[3] == 1.0f);
    }
    {   // a near-unit quaternion is normalised
        float p[3] = {0, 0, 0};
        float q[4] = {0.0f, 0.0f, 0.0f, 1.05f};
        CHECK(NetworkReceiver::SanitizePose(p, q));
        CHECK(std::fabs(q[3] - 1.0f) < 1e-6f);
    }
    {
        float p[3] = {std::numeric_limits<float>::quiet_NaN(), 0, 0};
        float q[4] = {0, 0, 0, 1};
        CHECK(!NetworkReceiver::SanitizePose(p, q));
    }
    {
        float p[3] = {0, 0, 0};
        float q[4] = {0, 0, std::numeric_limits<float>::infinity(), 1};
        CHECK(!NetworkReceiver::SanitizePose(p, q));
    }
    {
        float p[3] = {0, 150.0f, 0};
        float q[4] = {0, 0, 0, 1};
        CHECK(!NetworkReceiver::SanitizePose(p, q));
    }
    {   // all zero: not a rotation
        float p[3] = {0, 0, 0};
        float q[4] = {0, 0, 0, 0};
        CHECK(!NetworkReceiver::SanitizePose(p, q));
    }
    {
        float p[3] = {0, 0, 0};
        float q[4] = {0, 0, 0, 1.2f};
        CHECK(!NetworkReceiver::SanitizePose(p, q));
    }
}
