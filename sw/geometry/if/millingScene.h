#ifndef TRANSMUTER_GEOMETRY_MILLING_SCENE_H
#define TRANSMUTER_GEOMETRY_MILLING_SCENE_H

#include <Eigen/Geometry>
#include <memory>

#include "geometryIf.h"

namespace transmuter::geometry
{
class MillingScene
{
   public:
    MillingScene(std::shared_ptr<const GeometryIf> stock, std::shared_ptr<const GeometryIf> model, std::shared_ptr<GeometryIf> workpiece, std::shared_ptr<const GeometryIf> toolModel, std::shared_ptr<const GeometryIf> toolAssemblyModel = {}, Eigen::Affine3d stockToWorkpiece = Eigen::Affine3d::Identity());

    const std::shared_ptr<const GeometryIf>& stock() const;
    const std::shared_ptr<const GeometryIf>& model() const;
    const std::shared_ptr<GeometryIf>& workpiece() const;
    const std::shared_ptr<const GeometryIf>& toolModel() const;
    const std::shared_ptr<const GeometryIf>& toolAssemblyModel() const;
    const Eigen::Affine3d& stockToWorkpiece() const;

    void setStock(std::shared_ptr<const GeometryIf> stock, const Eigen::Affine3d& stockToWorkpiece = Eigen::Affine3d::Identity());
    void setWorkpiece(std::shared_ptr<GeometryIf> workpiece);

   private:
    void validate() const;

    std::shared_ptr<const GeometryIf> m_stock;
    std::shared_ptr<const GeometryIf> m_model;
    std::shared_ptr<GeometryIf> m_workpiece;
    std::shared_ptr<const GeometryIf> m_toolModel;
    std::shared_ptr<const GeometryIf> m_toolAssemblyModel;
    Eigen::Affine3d m_stockToWorkpiece;
};
}  // namespace transmuter::geometry

#endif  // TRANSMUTER_GEOMETRY_MILLING_SCENE_H
