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
#include <iomanip>
#include <ostream>
#include <string>

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
              m_active(m_output != nullptr),
              m_taskName(taskName),
              m_startWallTime(std::chrono::steady_clock::now()),
              m_startSimTimeSeconds(Simulator::Now().GetSeconds())
        {
        }

        ~ScopedEvent()
        {
            if (!m_active || m_output == nullptr)
            {
                return;
            }

            const auto endWallTime = std::chrono::steady_clock::now();
            const double elapsedMs =
                std::chrono::duration<double, std::milli>(endWallTime - m_startWallTime).count();

            (*m_output) << m_taskName << "," << std::fixed << std::setprecision(6)
                        << m_startSimTimeSeconds << "," << Simulator::Now().GetSeconds() << ","
                        << elapsedMs << std::endl;
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

  private:
    static std::ostream*& GetOutputStreamStorage()
    {
        static std::ostream* output = nullptr;
        return output;
    }
};

} // namespace ns3

#endif /* LEOSIM_TASK_PROFILER_H */
