/*
 * Copyright (c) 2024 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 */

#ifndef LEOSIM_TASK_PROFILER_H
#define LEOSIM_TASK_PROFILER_H

#include "ns3/simulator.h"

#include <chrono>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace ns3
{

class LeoSimTaskProfiler
{
  public:
    class ScopedEvent
    {
      public:
        explicit ScopedEvent(const std::string& taskName)
            : m_output(LeoSimTaskProfiler::GetOutputStream()),
              m_active(m_output != nullptr || LeoSimTaskProfiler::IsAggregationEnabled()),
              m_taskName(taskName),
              m_startWallTime(std::chrono::steady_clock::now()),
              m_startSimTimeSeconds(Simulator::Now().GetSeconds())
        {
        }

        ~ScopedEvent()
        {
            if (!m_active)
            {
                return;
            }

            const auto endWallTime = std::chrono::steady_clock::now();
            const double elapsedMs =
                std::chrono::duration<double, std::milli>(endWallTime - m_startWallTime).count();

            LeoSimTaskProfiler::Record(m_taskName, elapsedMs);
            if (m_output)
            {
                (*m_output) << m_taskName << "," << std::fixed << std::setprecision(6)
                            << m_startSimTimeSeconds << "," << Simulator::Now().GetSeconds() << ","
                            << elapsedMs << std::endl;
            }
        }

      private:
        std::ostream* m_output;
        bool m_active;
        std::string m_taskName;
        std::chrono::steady_clock::time_point m_startWallTime;
        double m_startSimTimeSeconds;
    };

    static void SetOutputStream(std::ostream* output)
    {
        GetOutputStreamStorage() = output;
    }

    static std::ostream* GetOutputStream()
    {
        return GetOutputStreamStorage();
    }

    static void EnableAggregation(bool enabled = true)
    {
        IsAggregationEnabledStorage() = enabled;
    }

    static bool IsAggregationEnabled()
    {
        return IsAggregationEnabledStorage();
    }

    static void Reset()
    {
        GetMeasurements().clear();
    }

    static void PrintSummary(double simulatorWallSeconds, std::ostream& output = std::cout)
    {
        struct RankedMeasurement
        {
            std::string name;
            double milliseconds;
            uint64_t calls;
        };

        std::vector<RankedMeasurement> ranked;
        double measuredMs = 0.0;
        for (const auto& [name, measurement] : GetMeasurements())
        {
            ranked.push_back({name, measurement.first, measurement.second});
            measuredMs += measurement.first;
        }
        std::sort(ranked.begin(),
                  ranked.end(),
                  [](const auto& lhs, const auto& rhs) {
                      return lhs.milliseconds > rhs.milliseconds;
                  });

        output << "\n[timing] Simulator::Run breakdown (wall-clock, slowest first)" << std::endl;
        for (const auto& measurement : ranked)
        {
            const double percent =
                simulatorWallSeconds > 0.0
                    ? measurement.milliseconds / (simulatorWallSeconds * 10.0)
                    : 0.0;
            output << "[timing]   " << measurement.name << ": " << std::fixed
                   << std::setprecision(3) << measurement.milliseconds / 1000.0 << "s ("
                   << percent << "%), calls=" << measurement.calls << std::endl;
        }

        const double unclassifiedMs =
            std::max(0.0, simulatorWallSeconds * 1000.0 - measuredMs);
        output << "[timing]   other ns-3 events/overhead: " << std::fixed
               << std::setprecision(3) << unclassifiedMs / 1000.0 << "s ("
               << (simulatorWallSeconds > 0.0
                       ? unclassifiedMs / (simulatorWallSeconds * 10.0)
                       : 0.0)
               << "%)" << std::endl;
        if (!ranked.empty())
        {
            output << "[timing] SLOWEST SIMULATOR PROCESS: " << ranked.front().name
                   << " consumed " << ranked.front().milliseconds / 1000.0 << "s" << std::endl;
        }
    }

  private:
    static void Record(const std::string& taskName, double elapsedMs)
    {
        if (!IsAggregationEnabled())
        {
            return;
        }
        auto& measurement = GetMeasurements()[taskName];
        measurement.first += elapsedMs;
        ++measurement.second;
    }

    static std::map<std::string, std::pair<double, uint64_t>>& GetMeasurements()
    {
        static std::map<std::string, std::pair<double, uint64_t>> measurements;
        return measurements;
    }

    static bool& IsAggregationEnabledStorage()
    {
        static bool enabled = false;
        return enabled;
    }

    static std::ostream*& GetOutputStreamStorage()
    {
        static std::ostream* output = nullptr;
        return output;
    }
};

} // namespace ns3

#endif /* LEOSIM_TASK_PROFILER_H */
