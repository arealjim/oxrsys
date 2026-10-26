// SPDX-License-Identifier: MPL-2.0

#include "RuntimePlatform.h"

#include <cstdlib>
#include <filesystem>

#include <dlfcn.h>
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread/qos.h>
#include <unistd.h>

namespace oxrsys::runtime_platform
{

namespace
{

std::string GetEnvironment(const char* name)
{
    const char* value = std::getenv(name);
    if (value != nullptr && value[0] != '\0')
    {
        return value;
    }
    return {};
}

std::string JoinPath(const std::string& root, const char* suffix)
{
    if (root.empty())
    {
        return {};
    }
    return (std::filesystem::path(root) / suffix).generic_string();
}

} // namespace

EnvironmentPaths CurrentEnvironmentPaths()
{
    EnvironmentPaths paths = {};
    paths.home = GetEnvironment("HOME");
    return paths;
}

std::string ConfigRootForEnvironment(const EnvironmentPaths& environment)
{
    return JoinPath(environment.home, "Library/Application Support/OXRSys");
}

std::string StateRootForEnvironment(const EnvironmentPaths& environment)
{
    return ConfigRootForEnvironment(environment);
}

std::string ConfigRoot()
{
    return ConfigRootForEnvironment(CurrentEnvironmentPaths());
}

std::string StateRoot()
{
    return StateRootForEnvironment(CurrentEnvironmentPaths());
}

std::string ModuleDirectory(const void* symbolAddress)
{
    Dl_info info = {};
    if (dladdr(symbolAddress, &info) && info.dli_fname != nullptr)
    {
        return std::filesystem::path(info.dli_fname).parent_path().string();
    }
    return ".";
}

uint64_t ProcessId()
{
    return static_cast<uint64_t>(getpid());
}

void SetCurrentThreadTimeSensitive()
{
    pthread_set_qos_class_self_np(QOS_CLASS_USER_INTERACTIVE, 0);
}

bool SetCurrentThreadRealtime(std::chrono::nanoseconds period)
{
    mach_timebase_info_data_t timebase{};
    mach_timebase_info(&timebase);
    const auto toTicks = [&](long long ns)
    { return static_cast<uint32_t>(ns * timebase.denom / timebase.numer); };

    thread_time_constraint_policy_data_t policy{};
    policy.period = toTicks(period.count());
    policy.computation = toTicks(period.count() / 3);
    policy.constraint = toTicks(period.count() / 2);
    policy.preemptible = 1;

    const thread_port_t self = mach_thread_self();
    const kern_return_t result = thread_policy_set(self, THREAD_TIME_CONSTRAINT_POLICY,
                                                   reinterpret_cast<thread_policy_t>(&policy),
                                                   THREAD_TIME_CONSTRAINT_POLICY_COUNT);
    mach_port_deallocate(mach_task_self(), self);
    return result == KERN_SUCCESS;
}

} // namespace oxrsys::runtime_platform
