#include "Renderer.h"

#include <GL/gl.h>
#include <GL/glu.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <string_view>
#include <utility>

#include "geometryFactory.h"
#include "trace.h"

namespace transmuter::visualization
{

namespace
{
constexpr double MillingPanelWidth = 230.0;
constexpr double MinimumSelectionSize = 0.5;
constexpr double MinimumSelectionPixels = 6.0;
constexpr double MinimumSelectionDepth = 20.0;
constexpr double MaximumSelectionDepth = 400.0;

std::array<unsigned char, 7> glyph(char character)
{
    switch (character)
    {
        case 'A':
            return {14, 17, 17, 31, 17, 17, 17};
        case 'B':
            return {30, 17, 17, 30, 17, 17, 30};
        case 'C':
            return {14, 17, 16, 16, 16, 17, 14};
        case 'D':
            return {30, 17, 17, 17, 17, 17, 30};
        case 'E':
            return {31, 16, 16, 30, 16, 16, 31};
        case 'F':
            return {31, 16, 16, 30, 16, 16, 16};
        case 'G':
            return {14, 17, 16, 23, 17, 17, 15};
        case 'H':
            return {17, 17, 17, 31, 17, 17, 17};
        case 'I':
            return {14, 4, 4, 4, 4, 4, 14};
        case 'J':
            return {7, 2, 2, 2, 2, 18, 12};
        case 'K':
            return {17, 18, 20, 24, 20, 18, 17};
        case 'L':
            return {16, 16, 16, 16, 16, 16, 31};
        case 'M':
            return {17, 27, 21, 21, 17, 17, 17};
        case 'N':
            return {17, 25, 21, 19, 17, 17, 17};
        case 'O':
            return {14, 17, 17, 17, 17, 17, 14};
        case 'P':
            return {30, 17, 17, 30, 16, 16, 16};
        case 'Q':
            return {14, 17, 17, 17, 21, 18, 13};
        case 'R':
            return {30, 17, 17, 30, 20, 18, 17};
        case 'S':
            return {15, 16, 16, 14, 1, 1, 30};
        case 'T':
            return {31, 4, 4, 4, 4, 4, 4};
        case 'U':
            return {17, 17, 17, 17, 17, 17, 14};
        case 'V':
            return {17, 17, 17, 17, 17, 10, 4};
        case 'W':
            return {17, 17, 17, 21, 21, 21, 10};
        case 'X':
            return {17, 17, 10, 4, 10, 17, 17};
        case 'Y':
            return {17, 17, 10, 4, 4, 4, 4};
        case 'Z':
            return {31, 1, 2, 4, 8, 16, 31};
        case '0':
            return {14, 17, 19, 21, 25, 17, 14};
        case '1':
            return {4, 12, 4, 4, 4, 4, 14};
        case '2':
            return {14, 17, 1, 2, 4, 8, 31};
        case '3':
            return {30, 1, 1, 14, 1, 1, 30};
        case '4':
            return {2, 6, 10, 18, 31, 2, 2};
        case '5':
            return {31, 16, 16, 30, 1, 1, 30};
        case '6':
            return {14, 16, 16, 30, 17, 17, 14};
        case '7':
            return {31, 1, 2, 4, 8, 8, 8};
        case '8':
            return {14, 17, 17, 14, 17, 17, 14};
        case '9':
            return {14, 17, 17, 15, 1, 1, 14};
        case '-':
            return {0, 0, 0, 31, 0, 0, 0};
        case '.':
            return {0, 0, 0, 0, 0, 12, 12};
        case '/':
            return {1, 2, 2, 4, 8, 8, 16};
        case ':':
            return {0, 12, 12, 0, 12, 12, 0};
        default:
            return {};
    }
}

void drawRectangle(double left, double bottom, double right, double top, float red, float green, float blue, float alpha = 1.0f)
{
    glColor4f(red, green, blue, alpha);
    glBegin(GL_QUADS);
    glVertex2d(left, bottom);
    glVertex2d(right, bottom);
    glVertex2d(right, top);
    glVertex2d(left, top);
    glEnd();
}

void drawText(std::string_view text, double x, double top, double scale, float red, float green, float blue)
{
    glColor3f(red, green, blue);
    const double pixelSize = scale;
    glBegin(GL_QUADS);
    for (const char character : text)
    {
        const auto rows = glyph(character);
        for (size_t row = 0; row < rows.size(); ++row)
        {
            for (size_t column = 0; column < 5; ++column)
            {
                if ((rows[row] & (1u << (4 - column))) == 0)
                {
                    continue;
                }
                const double left = x + column * scale;
                const double bottom = top - (row + 1) * scale;
                glVertex2d(left, bottom);
                glVertex2d(left + pixelSize, bottom);
                glVertex2d(left + pixelSize, bottom + pixelSize);
                glVertex2d(left, bottom + pixelSize);
            }
        }
        x += 6.0 * scale;
    }
    glEnd();
}
}  // namespace

Renderer::Renderer(int width, int height, const std::string& title)
    : m_title(title), m_width(width), m_height(height), m_selectedAlgorithm(pathgenerator::PathAlgorithmType::THREE_AXIS_RASTER), m_pathConfig{.pointSpacing = 5.0, .passSpacing = 10.0, .layerSpacing = 10.0, .geometryTolerance = 0.01}
{
    if (!glfwInit())
    {
        ERROR("Renderer: GLFW Init Failed");
    }

    m_window = glfwCreateWindow(m_width, m_height, title.c_str(), nullptr, nullptr);
    if (!m_window)
    {
        glfwTerminate();
        ERROR("Renderer: Window Creation Failed");
    }

    glfwSetWindowUserPointer(m_window, this);
    glfwSetScrollCallback(m_window, Renderer::scrollCallback);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1);
    glClearColor(0.12f, 0.12f, 0.14f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    updateWindowTitle();

    INFO("Renderer: window created (" + std::to_string(width) + "x" + std::to_string(height) + ")");
}

Renderer::~Renderer()
{
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

bool Renderer::processEvents()
{
    glfwWaitEventsTimeout(0.01);
    if (glfwWindowShouldClose(m_window))
    {
        return false;
    }
    handleInput();
    return true;
}

std::optional<UserMillingRequest> Renderer::takeMillingRequest()
{
    auto request = std::move(m_pendingMillingRequest);
    m_pendingMillingRequest.reset();
    return request;
}

bool Renderer::machineViewSelected() const
{
    return m_camera.machineView;
}

void Renderer::setMachineView(bool selected)
{
    if (selected != m_camera.machineView)
    {
        m_camera.zoom = selected ? -1300.0 : -300.0;
        m_camera.thetaDegrees = selected ? 65.0 : 55.0;
        m_camera.phiDegrees = selected ? 0.0 : 45.0;
    }
    m_camera.machineView = selected;
    if (selected)
    {
        m_selectingArea = false;
        m_selectionDragging = false;
    }
    updateWindowTitle();
}

void Renderer::beginFrame()
{
    const int viewWidth = std::max(1, m_width - static_cast<int>(MillingPanelWidth));
    glViewport(0, 0, viewWidth, m_height);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, static_cast<double>(viewWidth) / m_height, 1.0, 5000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslated(0.0, 0.0, m_camera.zoom);
    // Compose polar tilt with azimuth about scene Z, without camera roll.
    glRotated(-m_camera.thetaDegrees, 1.0, 0.0, 0.0);
    glRotated(m_camera.phiDegrees, 0.0, 0.0, 1.0);
    if (m_camera.machineView)
    {
        glTranslated(0.0, 0.0, -120.0);
    }
}

void Renderer::drawMachine(const MachineDrawing& drawing)
{
    glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_NORMALIZE);
    const GLfloat light[] = {0.4f, -0.6f, 1.0f, 0.0f};
    glLightfv(GL_LIGHT0, GL_POSITION, light);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);
    for (const auto& part : drawing) drawMachinePart(part);
    glPopClientAttrib();
    glPopAttrib();
}

void Renderer::drawStock(const geometry::GeometryIf& stock, const Eigen::Affine3d& transform)
{
    glPushMatrix();
    glMultMatrixd(transform.data());
    stock.copyVisualizationVertices(m_pointCloudSnapshot);

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glPointSize(1.0f);
    glBegin(GL_POINTS);

    glColor4f(0.55f, 0.55f, 0.55f, 0.025f);

    // The stock may be volumetric, so draw a sparse reference instead of obscuring the workpiece.
    for (size_t index = 0; index < m_pointCloudSnapshot.size(); index += 8)
    {
        const auto& v = m_pointCloudSnapshot[index];
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();
}

void Renderer::drawWorkpiece(const geometry::GeometryIf& workpiece, const Eigen::Affine3d& transform)
{
    glPushMatrix();
    glMultMatrixd(transform.data());
    workpiece.copyVisualizationVertices(m_pointCloudSnapshot);
    const bool highlightSelection = m_selectionDisplay == SelectionDisplay::STOCK_HIGHLIGHT && m_selectionVolume;
    const Eigen::Affine3d workpieceToSelection = m_selectionVolumeToWorkpiece.inverse();

    glDisable(GL_LIGHTING);
    glPointSize(3.0f);
    glBegin(GL_POINTS);

    for (const auto& v : m_pointCloudSnapshot)
    {
        if (highlightSelection && m_selectionVolume->isInside(workpieceToSelection * v.pos))
        {
            glColor3d(1.0, 0.48, 0.08);
            glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
            continue;
        }
        // Keep the changing workpiece as the visually dominant shape.
        glColor3d(std::abs(v.normal.x()), std::abs(v.normal.y()), std::abs(v.normal.z()));
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
}

void Renderer::drawUserSelection(const Eigen::Affine3d& workpieceFrame)
{
    if (m_selectionDisplay != SelectionDisplay::BOX || !m_selectionVolume)
    {
        return;
    }

    m_selectionVolume->copyVisualizationVertices(m_pointCloudSnapshot);
    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT | GL_COLOR_BUFFER_BIT | GL_POINT_BIT | GL_CURRENT_BIT);
    const Eigen::Affine3d selectionFrame = workpieceFrame * m_selectionVolumeToWorkpiece;
    glMultMatrixd(selectionFrame.data());
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glPointSize(5.0f);
    glColor4f(0.10f, 0.95f, 1.0f, 0.9f);
    glBegin(GL_POINTS);
    for (const auto& vertex : m_pointCloudSnapshot)
    {
        glVertex3d(vertex.pos.x(), vertex.pos.y(), vertex.pos.z());
    }
    glEnd();
    glPopAttrib();
    glPopMatrix();
}

void Renderer::drawToolPath(const std::vector<geometry::Vertex>& points, const Eigen::Affine3d& transform)
{
    glPushMatrix();
    glMultMatrixd(transform.data());

    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glLineWidth(1.5f);
    glBegin(GL_LINE_STRIP);

    for (const auto& point : points)
    {
        glColor4f(0.35f, 0.75f, 0.85f, 0.55f);
        glVertex3d(point.pos.x(), point.pos.y(), point.pos.z());
    }
    glEnd();
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glPopMatrix();
}

void Renderer::drawTool(const geometry::GeometryIf& toolModel, const Eigen::Vector3d& pos, const Eigen::Vector3d& normal)
{
    toolModel.copyVisualizationVertices(m_pointCloudSnapshot);

    glPushMatrix();
    glTranslated(pos.x(), pos.y(), pos.z());

    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(),
        normal.normalized());
    const Eigen::AngleAxisd angleAxis(rotation);
    const double angleDegrees = angleAxis.angle() * 180.0 / std::numbers::pi;
    glRotated(angleDegrees, angleAxis.axis().x(), angleAxis.axis().y(), angleAxis.axis().z());

    glBegin(GL_POINTS);
    glColor3f(1.0f, 0.1f, 0.1f);  // Solid Red for the tool as requested

    for (const auto& v : m_pointCloudSnapshot)
    {
        glVertex3d(v.pos.x(), v.pos.y(), v.pos.z());
    }
    glEnd();
    glPopMatrix();
}

void Renderer::endFrame()
{
    drawMillingPanel();
    glfwSwapBuffers(m_window);
}

void Renderer::handleInput()
{
    if (glfwGetKey(m_window, GLFW_KEY_M) == GLFW_PRESS)
    {
        setMachineView(true);
    }
    if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS)
    {
        setMachineView(false);
    }
    double currentX = 0.0;
    double currentY = 0.0;
    glfwGetCursorPos(m_window, &currentX, &currentY);

    const bool leftMouseDown = glfwGetMouseButton(m_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    const bool mousePressed = m_mousePressPending || (leftMouseDown && !m_leftMouseDown);
    const bool mouseReleased = m_mouseReleasePending || (!leftMouseDown && m_leftMouseDown);
    const double panelLeft = m_width - MillingPanelWidth;
    const double pressX = m_mousePressPending ? m_mousePressX : currentX;
    const double pressY = m_mousePressPending ? m_mousePressY : currentY;

    if (mousePressed && pressX >= panelLeft)
    {
        handleMillingPanelClick(pressX, pressY);
    }
    else if (mousePressed && m_selectingArea && !m_camera.machineView)
    {
        m_selectionDragging = true;
        m_selectionStartX = pressX;
        m_selectionStartY = pressY;
        m_selectionEndX = pressX;
        m_selectionEndY = pressY;
        m_hasSelection = false;
        m_selectionVolume.reset();
        m_selectionDisplay = SelectionDisplay::NONE;
    }

    if (m_selectionDragging && leftMouseDown)
    {
        m_selectionEndX = currentX;
        m_selectionEndY = currentY;
    }
    if (m_selectionDragging && mouseReleased)
    {
        updateSelection(currentX, currentY);
        m_selectionDragging = false;
        m_selectingArea = false;
        updateWindowTitle();
    }

    if (m_activeSlider >= 0 && leftMouseDown)
    {
        updateMillingSlider(currentX);
    }
    if (mouseReleased)
    {
        m_activeSlider = -1;
    }

    const bool cameraDragging = leftMouseDown && m_leftMouseDown && !m_selectionDragging && currentX < panelLeft;
    if (cameraDragging)
    {
        const double deltaX = currentX - m_lastMouseX;
        const double deltaY = currentY - m_lastMouseY;
        m_camera.phiDegrees = std::remainder(m_camera.phiDegrees + deltaX * 0.5, 360.0);
        m_camera.thetaDegrees = std::clamp(m_camera.thetaDegrees - deltaY * 0.5, 1.0, 179.0);
    }

    m_leftMouseDown = leftMouseDown;
    m_mousePressPending = false;
    m_mouseReleasePending = false;
    m_lastMouseX = currentX;
    m_lastMouseY = currentY;
}

void Renderer::handleMillingPanelClick(double x, double y)
{
    m_activeSlider = -1;
    // GLFW cursor coordinates are measured down from the top of the window.
    const double panelTop = y;
    if (panelTop >= 28.0 && panelTop <= 78.0)
    {
        switch (m_selectedAlgorithm)
        {
            case pathgenerator::PathAlgorithmType::THREE_AXIS_RASTER:
                m_selectedAlgorithm = pathgenerator::PathAlgorithmType::THREE_AXIS_SPIRAL;
                break;
            case pathgenerator::PathAlgorithmType::THREE_AXIS_SPIRAL:
                m_selectedAlgorithm = pathgenerator::PathAlgorithmType::FIVE_AXIS_NORMAL;
                break;
            case pathgenerator::PathAlgorithmType::FIVE_AXIS_NORMAL:
                m_selectedAlgorithm = pathgenerator::PathAlgorithmType::THREE_AXIS_RASTER;
                break;
        }
    }
    else if (panelTop >= 105.0 && panelTop <= 137.0)
    {
        m_activeSlider = 0;
        updateMillingSlider(x);
    }
    else if (panelTop >= 150.0 && panelTop <= 182.0)
    {
        m_activeSlider = 1;
        updateMillingSlider(x);
    }
    else if (panelTop >= 195.0 && panelTop <= 227.0)
    {
        m_activeSlider = 2;
        updateMillingSlider(x);
    }
    else if (panelTop >= 240.0 && panelTop <= 272.0)
    {
        m_activeSlider = 3;
        updateMillingSlider(x);
    }
    else if (panelTop >= 300.0 && panelTop <= 342.0)
    {
        m_selectingArea = !m_camera.machineView;
    }
    else if (panelTop >= 360.0 && panelTop <= 402.0 && m_hasSelection)
    {
        m_selectionDisplay = SelectionDisplay::STOCK_HIGHLIGHT;
        m_pendingMillingRequest = UserMillingRequest{
            .selectedVolume = m_selectionVolume,
            .volumeToWorkpiece = m_selectionVolumeToWorkpiece,
            .algorithm = m_selectedAlgorithm,
            .config = m_pathConfig,
        };
    }

    updateWindowTitle();
}

void Renderer::updateSelection(double x, double y)
{
    m_selectionEndX = x;
    m_selectionEndY = y;
    std::array<Eigen::Vector3d, 8> corners;
    const double left = std::min(m_selectionStartX, m_selectionEndX);
    const double right = std::max(m_selectionStartX, m_selectionEndX);
    const double top = std::min(m_selectionStartY, m_selectionEndY);
    const double bottom = std::max(m_selectionStartY, m_selectionEndY);
    if (right - left < MinimumSelectionPixels || bottom - top < MinimumSelectionPixels)
    {
        m_hasSelection = false;
        m_selectionVolume.reset();
        m_selectionDisplay = SelectionDisplay::NONE;
        return;
    }
    const double screenCorners[4][2] = {{left, top}, {right, top}, {left, bottom}, {right, bottom}};
    GLdouble modelView[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
    const Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> viewTransform(modelView);
    const Eigen::Matrix3d cameraToWorkpiece = viewTransform.block<3, 3>(0, 0).transpose();
    const Eigen::Vector3d viewRight = cameraToWorkpiece.col(0).normalized();
    const Eigen::Vector3d viewUp = cameraToWorkpiece.col(1).normalized();
    const Eigen::Vector3d viewDepth = cameraToWorkpiece.col(2).normalized();

    for (size_t index = 0; index < 4; ++index)
    {
        Eigen::Vector3d nearPoint;
        Eigen::Vector3d farPoint;
        if (!screenToWorkpieceRay(screenCorners[index][0], screenCorners[index][1], nearPoint, farPoint))
        {
            m_hasSelection = false;
            m_selectionDisplay = SelectionDisplay::NONE;
            return;
        }
        const Eigen::Vector3d ray = farPoint - nearPoint;
        const double rayDepth = ray.dot(viewDepth);
        if (std::abs(rayDepth) <= 1e-9)
        {
            m_hasSelection = false;
            m_selectionDisplay = SelectionDisplay::NONE;
            return;
        }
        for (size_t depthIndex = 0; depthIndex < 2; ++depthIndex)
        {
            const double planeDepth = depthIndex == 0 ? -m_selectionDepth * 0.5 : m_selectionDepth * 0.5;
            const double rayDistance = (planeDepth - nearPoint.dot(viewDepth)) / rayDepth;
            if (rayDistance < 0.0 || rayDistance > 1.0)
            {
                m_hasSelection = false;
                m_selectionDisplay = SelectionDisplay::NONE;
                return;
            }
            corners[index + depthIndex * 4] = nearPoint + rayDistance * ray;
        }
    }

    double minimumRight = corners[0].dot(viewRight);
    double maximumRight = minimumRight;
    double minimumUp = corners[0].dot(viewUp);
    double maximumUp = minimumUp;
    for (const auto& corner : corners)
    {
        const double right = corner.dot(viewRight);
        const double up = corner.dot(viewUp);
        minimumRight = std::min(minimumRight, right);
        maximumRight = std::max(maximumRight, right);
        minimumUp = std::min(minimumUp, up);
        maximumUp = std::max(maximumUp, up);
    }
    const double width = maximumRight - minimumRight;
    const double height = maximumUp - minimumUp;
    const double depth = m_selectionDepth;
    const Eigen::Vector3d dimensions(width, height, depth);
    m_hasSelection = width >= MinimumSelectionSize && height >= MinimumSelectionSize;
    m_selectionVolume.reset();
    m_selectionDisplay = SelectionDisplay::NONE;
    if (m_hasSelection)
    {
        m_selectionVolume = geometry::GeometryFactory::createBox(dimensions.x(), dimensions.y(),
            dimensions.z(), geometry::GeometryRepresentation::ANALYTIC, 1.0);
        m_selectionVolumeToWorkpiece = Eigen::Affine3d::Identity();
        m_selectionVolumeToWorkpiece.linear().col(0) = viewRight;
        m_selectionVolumeToWorkpiece.linear().col(1) = viewUp;
        m_selectionVolumeToWorkpiece.linear().col(2) = viewDepth;
        m_selectionVolumeToWorkpiece.translation() = viewRight * (0.5 * (minimumRight + maximumRight)) + viewUp * (0.5 * (minimumUp + maximumUp));
        m_selectionDisplay = SelectionDisplay::BOX;
    }
}

void Renderer::drawMillingPanel()
{
    GLint previousViewport[4];
    glGetIntegerv(GL_VIEWPORT, previousViewport);
    glViewport(0, 0, m_width, m_height);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, m_width, 0.0, m_height, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    const double left = m_width - MillingPanelWidth;
    drawRectangle(left, 0.0, m_width, m_height, 0.08f, 0.09f, 0.11f, 0.96f);
    drawRectangle(left + 16.0, m_height - 78.0, m_width - 16.0, m_height - 28.0, 0.20f, 0.35f, 0.48f);
    drawRectangle(left + 18.0, m_height - 132.0, m_width - 18.0, m_height - 110.0, 0.24f, 0.25f, 0.28f);
    drawRectangle(left + 18.0, m_height - 177.0, m_width - 18.0, m_height - 155.0, 0.24f, 0.25f, 0.28f);
    drawRectangle(left + 18.0, m_height - 222.0, m_width - 18.0, m_height - 200.0, 0.24f, 0.25f, 0.28f);
    drawRectangle(left + 18.0, m_height - 130.0,
        left + 18.0 + (MillingPanelWidth - 36.0) * ((m_pathConfig.pointSpacing - 0.5) / 9.5), m_height - 112.0, 0.25f, 0.62f, 0.78f);
    drawRectangle(left + 18.0, m_height - 175.0,
        left + 18.0 + (MillingPanelWidth - 36.0) * ((m_pathConfig.passSpacing - 1.0) / 19.0), m_height - 157.0, 0.25f, 0.62f, 0.78f);
    drawRectangle(left + 18.0, m_height - 220.0,
        left + 18.0 + (MillingPanelWidth - 36.0) * ((m_pathConfig.layerSpacing - 1.0) / 39.0), m_height - 202.0, 0.25f, 0.62f, 0.78f);
    drawRectangle(left + 18.0, m_height - 267.0, m_width - 18.0, m_height - 245.0, 0.24f, 0.25f, 0.28f);
    drawRectangle(left + 18.0, m_height - 265.0, left + 18.0 + (MillingPanelWidth - 36.0) * ((m_selectionDepth - MinimumSelectionDepth) / (MaximumSelectionDepth - MinimumSelectionDepth)), m_height - 247.0, 0.25f, 0.62f, 0.78f);
    const bool selectionAvailable = !m_camera.machineView;
    drawRectangle(left + 16.0, m_height - 342.0, m_width - 16.0, m_height - 300.0, selectionAvailable ? (m_selectingArea ? 0.70f : 0.25f) : 0.18f, selectionAvailable ? (m_selectingArea ? 0.48f : 0.34f) : 0.19f, 0.20f);
    drawRectangle(left + 16.0, m_height - 402.0, m_width - 16.0, m_height - 360.0, m_hasSelection ? 0.20f : 0.22f, m_hasSelection ? 0.58f : 0.23f, m_hasSelection ? 0.35f : 0.24f);

    const char* algorithmLabel = "RASTER";
    if (m_selectedAlgorithm == pathgenerator::PathAlgorithmType::THREE_AXIS_SPIRAL)
    {
        algorithmLabel = "SPIRAL";
    }
    else if (m_selectedAlgorithm == pathgenerator::PathAlgorithmType::FIVE_AXIS_NORMAL)
    {
        algorithmLabel = "FIVE AXIS";
    }
    drawText("MILLING SETUP", left + 16.0, m_height - 12.0, 2.0, 0.90f, 0.93f, 0.97f);
    drawText("ALGORITHM", left + 26.0, m_height - 39.0, 2.0, 0.90f, 0.93f, 0.97f);
    drawText(algorithmLabel, left + 26.0, m_height - 59.0, 2.0, 1.0f, 1.0f, 1.0f);
    drawText("POINT SPACING", left + 18.0, m_height - 91.0, 2.0, 0.90f, 0.93f, 0.97f);
    drawText("PASS SPACING", left + 18.0, m_height - 136.0, 2.0, 0.90f, 0.93f, 0.97f);
    drawText("LAYER SPACING", left + 18.0, m_height - 181.0, 2.0, 0.90f, 0.93f, 0.97f);
    drawText("VOLUME DEPTH", left + 18.0, m_height - 226.0, 2.0, 0.90f, 0.93f, 0.97f);
    drawText(!selectionAvailable ? "USE W VIEW" : (m_selectingArea ? "DRAG AREA" : "SELECT AREA"), left + 26.0, m_height - 316.0, 2.0, 1.0f, 1.0f, 1.0f);
    drawText(m_hasSelection ? "GENERATE PATH" : "GENERATE", left + 26.0, m_height - 376.0, 2.0, 1.0f, 1.0f, 1.0f);
    drawText("W WORKPIECE M MACHINE", left + 16.0, 30.0, 1.5, 0.72f, 0.76f, 0.82f);

    if (m_selectionDragging)
    {
        glColor3f(0.15f, 0.85f, 1.0f);
        glLineWidth(2.0f);
        glBegin(GL_LINE_LOOP);
        glVertex2d(m_selectionStartX, m_height - m_selectionStartY);
        glVertex2d(m_selectionEndX, m_height - m_selectionStartY);
        glVertex2d(m_selectionEndX, m_height - m_selectionEndY);
        glVertex2d(m_selectionStartX, m_height - m_selectionEndY);
        glEnd();
    }

    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glViewport(previousViewport[0], previousViewport[1], previousViewport[2], previousViewport[3]);
}

void Renderer::scrollCallback(GLFWwindow* window, double, double yOffset)
{
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    renderer->handleScroll(yOffset);
}

void Renderer::mouseButtonCallback(GLFWwindow* window, int button, int action, int)
{
    if (button != GLFW_MOUSE_BUTTON_LEFT)
    {
        return;
    }
    auto* renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
    double cursorX = 0.0;
    double cursorY = 0.0;
    glfwGetCursorPos(window, &cursorX, &cursorY);
    if (action == GLFW_PRESS)
    {
        renderer->m_mousePressPending = true;
        renderer->m_mousePressX = cursorX;
        renderer->m_mousePressY = cursorY;
    }
    else if (action == GLFW_RELEASE)
    {
        renderer->m_mouseReleasePending = true;
    }
}

void Renderer::handleScroll(double yOffset)
{
    m_camera.zoom = std::clamp(m_camera.zoom + yOffset * 10.0, -3000.0, -10.0);
}

void Renderer::updateMillingSlider(double x)
{
    const double panelLeft = m_width - MillingPanelWidth;
    const double position = std::clamp((x - panelLeft - 18.0) / (MillingPanelWidth - 36.0), 0.0, 1.0);
    if (m_activeSlider == 0)
    {
        m_pathConfig.pointSpacing = 0.5 + 9.5 * position;
    }
    else if (m_activeSlider == 1)
    {
        m_pathConfig.passSpacing = 1.0 + 19.0 * position;
    }
    else if (m_activeSlider == 2)
    {
        m_pathConfig.layerSpacing = 1.0 + 39.0 * position;
    }
    else if (m_activeSlider == 3)
    {
        m_selectionDepth = MinimumSelectionDepth + (MaximumSelectionDepth - MinimumSelectionDepth) * position;
        if (m_hasSelection)
        {
            updateSelection(m_selectionEndX, m_selectionEndY);
        }
    }
    updateWindowTitle();
}

void Renderer::updateWindowTitle()
{
    const char* algorithmName = "Raster";
    if (m_selectedAlgorithm == pathgenerator::PathAlgorithmType::THREE_AXIS_SPIRAL)
    {
        algorithmName = "Spiral";
    }
    else if (m_selectedAlgorithm == pathgenerator::PathAlgorithmType::FIVE_AXIS_NORMAL)
    {
        algorithmName = "Five-axis normal";
    }
    const std::string coordinates = m_camera.machineView ? "Machine view" : "Workpiece view";
    const std::string status = m_selectingArea  ? "Drag a box"
                               : m_hasSelection ? "Area selected; Generate"
                                                : "Select an area, then Generate";
    const std::string title = m_title + " | " + coordinates + " | " + status +
                              " | panel top: algorithm; sliders: point/pass/layer; then Select "
                              "area / Generate | " +
                              algorithmName + " " + std::to_string(m_pathConfig.pointSpacing) + "/" +
                              std::to_string(m_pathConfig.passSpacing) + "/" +
                              std::to_string(m_pathConfig.layerSpacing) + " mm | selection depth " + std::to_string(m_selectionDepth) +
                              " mm";
    glfwSetWindowTitle(m_window, title.c_str());
}

bool Renderer::screenToWorkpieceRay(double x, double y, Eigen::Vector3d& nearPoint, Eigen::Vector3d& farPoint) const
{
    GLdouble modelView[16];
    GLdouble projection[16];
    GLint viewport[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    GLdouble nearX, nearY, nearZ, farX, farY, farZ;
    if (gluUnProject(x, m_height - y, 0.0, modelView, projection, viewport, &nearX, &nearY, &nearZ) != GL_TRUE || gluUnProject(x, m_height - y, 1.0, modelView, projection, viewport, &farX, &farY, &farZ) != GL_TRUE)
    {
        return false;
    }
    nearPoint = Eigen::Vector3d(nearX, nearY, nearZ);
    farPoint = Eigen::Vector3d(farX, farY, farZ);
    return nearPoint.allFinite() && farPoint.allFinite();
}

void Renderer::drawMachinePart(const MachineDrawingPart& part)
{
    if (!part.mesh || part.mesh->vertices.empty())
    {
        return;
    }
    glPushMatrix();
    glMultMatrixd(part.transform.data());
    glColor3fv(part.color.data());
    glVertexPointer(3, GL_FLOAT, sizeof(MachineMeshVertex), part.mesh->vertices.front().position.data());
    glNormalPointer(GL_FLOAT, sizeof(MachineMeshVertex), part.mesh->vertices.front().normal.data());
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(part.mesh->triangleIndices.size()), GL_UNSIGNED_INT,
        part.mesh->triangleIndices.data());
    glPopMatrix();
}
}  // namespace transmuter::visualization
