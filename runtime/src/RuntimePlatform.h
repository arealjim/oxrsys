// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace oxrsys::runtime_platform
{

struct EnvironmentPaths
{
    std::string home;
};

EnvironmentPaths CurrentEnvironmentPaths();
std::string ConfigRootForEnvironment(const EnvironmentPaths& environment);
std::string StateRootForEnvironment(const EnvironmentPaths& environment);
std::string ConfigRoot();
std::string StateRoot();
std::string ModuleDirectory(const void* symbolAddress);
uint64_t ProcessId();

// Raise the calling thread's scheduling priority so time-sensitive per-frame
// work is not delayed behind default-priority threads. Best effort on every
// macOS: failures just keep the default policy.
void SetCurrentThreadTimeSensitive();

// Give the calling thread a Mach time-constraint (realtime) policy for work that repeats every
// period: computation = a third of the period, constraint = half of it, preemptible. A thread that
// overruns its declared computation is demoted by the scheduler, not stalled. Returns false if the
// kernel refused the policy (the thread keeps its old one).
bool SetCurrentThreadRealtime(std::chrono::nanoseconds period);

} // namespace oxrsys::runtime_platform
