#pragma once
#include "MachineVisualizationIf.h"
namespace transmuter::visualization
{
class BridgeportMachineVisualization final : public MachineVisualizationIf
{
   public:
    BridgeportMachineVisualization(std::shared_ptr<const pathgenerator::MachineKinematicsIf> machineKinematics, MachineDrawingConfig config);
    MachineDrawing getMachineDrawing(const geometry::MachineAxes& axes) const override;

   private:
    struct Frames
    {
        Eigen::Affine3d table = Eigen::Affine3d::Identity();
        Eigen::Affine3d tilt = Eigen::Affine3d::Identity();
        Eigen::Affine3d rotary = Eigen::Affine3d::Identity();
        Eigen::Affine3d workpiece = Eigen::Affine3d::Identity();
        Eigen::Affine3d tool = Eigen::Affine3d::Identity();
    };
    Frames framesFor(const geometry::MachineAxes& axes) const;
    void addFrame(MachineDrawing& drawing) const;
    void addTable(MachineDrawing& drawing, const Frames& frames) const;
    void addTrunnion(MachineDrawing& drawing, const Frames& frames) const;
    void addSpindle(MachineDrawing& drawing, const Frames& frames) const;
    void addPart(MachineDrawing& drawing, const std::shared_ptr<const MachineMesh>& mesh, const Eigen::Affine3d& frame, const Eigen::Vector3d& center, const Eigen::Vector3d& dimensions, const Eigen::Vector3f& color) const;
    std::shared_ptr<const pathgenerator::MachineKinematicsIf> m_machineKinematics;
    MachineDrawingConfig m_config;
    std::shared_ptr<const MachineMesh> m_box;
    std::shared_ptr<const MachineMesh> m_cylinder;
};
}  // namespace transmuter::visualization
