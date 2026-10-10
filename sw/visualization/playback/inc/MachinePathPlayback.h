#pragma once
#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

#include "MachineKinematicsIf.h"
#include "PathGenerationContext.h"
#include "millingScene.h"

namespace transmuter::visualization
{
/** Playback timing, machine-axis interpolation and displayed stock removal, without rendering. */
class MachinePathPlayback
{
   public:
    MachinePathPlayback(const geometry::MillingScene& scene, const pathgenerator::PathGenerationContext& pathGeneration, double playbackDurationSeconds);

   private:
    friend class Visualizer;

    void start();
    void update();
    void calculatePlaybackTimes();
    void updateMove(double targetTravelTime);
    void removeTraversedMaterial(double targetTravelTime);
    void finishPlayback();

    double calculatePlaybackProgress() const;
    pathgenerator::MachineMove interpolateMove(size_t upperIndex, double travelTime) const;
    double segmentTravel(const pathgenerator::MachineMove& from, const pathgenerator::MachineMove& to, double reach) const;
    bool hasStarted() const;
    const pathgenerator::MachineMove& currentMove() const;
    std::shared_ptr<geometry::GeometryIf> m_workpiece;
    std::shared_ptr<const pathgenerator::MachinePathIf> m_sourcePath;
    std::shared_ptr<const geometry::GeometryIf> m_tool;
    std::shared_ptr<const geometry::GeometryIf> m_assembly;
    std::shared_ptr<const pathgenerator::MachineKinematicsIf> m_machineKinematics;
    double m_playbackDurationSeconds;

    std::chrono::steady_clock::time_point m_startTime;
    std::vector<pathgenerator::MachineMove> m_moves;
    std::vector<double> m_cumulativeTravelTimes;
    pathgenerator::MachineMove m_currentMove;
    double m_pathTravelTime = 0.0;
    size_t m_processedPointCount = 0;
    bool m_started = false;
    bool m_active = false;
};
}  // namespace transmuter::visualization
