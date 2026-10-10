#include "millingScene.h"

#include <utility>

#include "trace.h"

namespace transmuter::geometry
{
MillingScene::MillingScene(std::shared_ptr<const GeometryIf> stock, std::shared_ptr<const GeometryIf> model, std::shared_ptr<GeometryIf> workpiece, std::shared_ptr<const GeometryIf> toolModel, std::shared_ptr<const GeometryIf> toolAssemblyModel, Eigen::Affine3d stockToWorkpiece)
    : m_stock(std::move(stock)), m_model(std::move(model)), m_workpiece(std::move(workpiece)), m_toolModel(std::move(toolModel)), m_toolAssemblyModel(std::move(toolAssemblyModel)), m_stockToWorkpiece(std::move(stockToWorkpiece))
{
    validate();
}

void MillingScene::validate() const
{
    if (!m_stock || !m_model || !m_workpiece || !m_toolModel)
    {
        ERROR("Milling scene requires stock, target, workpiece and tool geometry.");
    }
    if (!m_stock->bounds().isValid || !m_model->bounds().isValid || !m_workpiece->bounds().isValid ||
        !m_toolModel->bounds().isValid || (m_toolAssemblyModel && !m_toolAssemblyModel->bounds().isValid) ||
        !m_stockToWorkpiece.matrix().allFinite())
    {
        ERROR("Milling scene geometry must have valid bounds.");
    }
}

const std::shared_ptr<const GeometryIf>& MillingScene::stock() const
{
    return m_stock;
}

const std::shared_ptr<const GeometryIf>& MillingScene::model() const
{
    return m_model;
}

const std::shared_ptr<GeometryIf>& MillingScene::workpiece() const
{
    return m_workpiece;
}

const std::shared_ptr<const GeometryIf>& MillingScene::toolModel() const
{
    return m_toolModel;
}

const std::shared_ptr<const GeometryIf>& MillingScene::toolAssemblyModel() const
{
    return m_toolAssemblyModel;
}

const Eigen::Affine3d& MillingScene::stockToWorkpiece() const
{
    return m_stockToWorkpiece;
}

void MillingScene::setStock(std::shared_ptr<const GeometryIf> stock, const Eigen::Affine3d& stockToWorkpiece)
{
    m_stock = std::move(stock);
    m_stockToWorkpiece = stockToWorkpiece;
    validate();
}

void MillingScene::setWorkpiece(std::shared_ptr<GeometryIf> workpiece)
{
    m_workpiece = std::move(workpiece);
    validate();
}

}  // namespace transmuter::geometry
