#include "BridgeportMachineVisualization.h"

#include <algorithm>
#include <numbers>
#include <utility>

#include "MachineMeshBuilder.h"
namespace transmuter::visualization
{
BridgeportMachineVisualization::BridgeportMachineVisualization(std::shared_ptr<const pathgenerator::MachineKinematicsIf> machineKinematics, MachineDrawingConfig config)
    : m_machineKinematics(std::move(machineKinematics)), m_config(config), m_box(MachineMeshBuilder::box()), m_cylinder(MachineMeshBuilder::cylinder())
{
}
MachineDrawing BridgeportMachineVisualization::getMachineDrawing(const geometry::MachineAxes& axes) const
{
    MachineDrawing drawing;
    const auto frames = framesFor(axes);
    addFrame(drawing);
    addTable(drawing, frames);
    addTrunnion(drawing, frames);
    addSpindle(drawing, frames);
    return drawing;
}
BridgeportMachineVisualization::Frames BridgeportMachineVisualization::framesFor(const geometry::MachineAxes& axes) const
{
    Frames frames;
    frames.table.translate(Eigen::Vector3d(axes.x, axes.y, 0.0));
    frames.tilt = frames.table * Eigen::Translation3d(m_config.jointPositionInTable) * Eigen::AngleAxisd(axes.a, Eigen::Vector3d::UnitY());
    frames.rotary = frames.tilt * Eigen::AngleAxisd(axes.b, Eigen::Vector3d::UnitZ());
    const auto display = m_machineKinematics->displayPose(axes);
    frames.workpiece = display.workpiece;
    frames.tool = display.tool;
    return frames;
}
void BridgeportMachineVisualization::addPart(MachineDrawing& drawing, const std::shared_ptr<const MachineMesh>& mesh, const Eigen::Affine3d& frame, const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const Eigen::Vector3f& color) const
{
    Eigen::Affine3d transform = frame * Eigen::Translation3d(center);
    transform.scale(dimensions);
    drawing.push_back({mesh, transform, color});
}
void BridgeportMachineVisualization::addFrame(MachineDrawing& drawing) const
{
    const auto frame = Eigen::Affine3d::Identity();
    const Eigen::Vector3f casting{0.28f, 0.34f, 0.38f};
    addPart(drawing, m_box, frame, {0, 80, -160}, {380, 440, 100}, casting);
    addPart(drawing, m_box, frame, {0, 230, 140}, {160, 140, 540}, casting);
    addPart(drawing, m_box, frame, {0, 100, 430}, {130, 390, 85}, casting);
    addPart(drawing, m_box, frame, {0, 35, -70}, {240, 270, 100}, casting);
}
void BridgeportMachineVisualization::addTable(MachineDrawing& drawing, const Frames& frames) const
{
    Eigen::Affine3d saddle = Eigen::Affine3d::Identity();
    saddle.translation().y() = frames.table.translation().y();
    addPart(drawing, m_box, saddle, {0, 0, -35}, {260, 200, 30}, {0.36f, 0.4f, 0.43f});
    addPart(drawing, m_box, frames.table, {0, 0, -10}, {600, 180, 20}, {0.58f, 0.61f, 0.64f});
    for (double y : {-55.0, 0.0, 55.0})
        addPart(drawing, m_box, frames.table, {0, y, 0.5}, {580, 5, 1}, {0.12f, 0.13f, 0.14f});
}
void BridgeportMachineVisualization::addTrunnion(MachineDrawing& drawing, const Frames& frames) const
{
    const Eigen::Vector3d joint = (frames.table.inverse() * frames.tilt).translation();
    for (double side : {-1.0, 1.0})
        addPart(drawing, m_box, frames.table, joint + Eigen::Vector3d(0, side * 105, -joint.z() * 0.5),
            {70, 30, std::max(10.0, joint.z())}, {0.3f, 0.45f, 0.55f});
    Eigen::Affine3d bearing = frames.table * Eigen::Translation3d(joint) * Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitX());
    addPart(drawing, m_cylinder, bearing, {0, 0, 0}, {24, 24, 225}, {0.45f, 0.5f, 0.54f});
    const Eigen::Vector3d mounting = (frames.rotary.inverse() * frames.workpiece).translation() + Eigen::Vector3d(0, 0, m_config.workpieceBaseZ - 8.0);
    addPart(drawing, m_box, frames.tilt, {0, 0, mounting.z() - 18}, {150, 180, 20}, {0.3f, 0.45f, 0.55f});
    addPart(drawing, m_cylinder, frames.rotary, mounting, {75, 75, 16}, {0.65f, 0.67f, 0.68f});
}
void BridgeportMachineVisualization::addSpindle(MachineDrawing& drawing, const Frames& frames) const
{
    addPart(drawing, m_cylinder, frames.tool, {0, 0, m_config.toolLength + 25}, {22, 22, 50}, {0.68f, 0.7f, 0.72f});
    addPart(drawing, m_box, frames.tool, {0, 20, m_config.toolLength + 100}, {90, 100, 110}, {0.38f, 0.43f, 0.47f});
}
}  // namespace transmuter::visualization
