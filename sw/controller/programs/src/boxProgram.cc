#include "boxProgram.h"

#include "box.h"
#include "visualizer.h"

namespace transmuter::controller::programs
{

void BoxProgram::execute()
{
    using namespace transmuter::geometry;
    using namespace transmuter::visualization;

    // 1. Local Service Setup
    Visualizer viz(1024, 768, "Bernardo: BoxProgram Milling Program");

    // 2. Geometry Generation
    Eigen::Vector3f dims(100.0f, 100.0f, 100.0f);
    auto workPiece = shapes::Box::generate(dims, 5.0f);

    float rotation = 0.0f;

    // 3. The Autonomous Loop
    // The program stays here until the user closes the window
    while (!viz.shouldClose())
    {
        viz.beginFrame();

        // All the logic stays encapsulated right here
        viz.drawPointCloud(workPiece, rotation);
        rotation += 0.4f;

        viz.endFrame();
    }

    // Once the loop breaks, the function returns,
    // and 'viz' is automatically destroyed (RAII).
}

}  // namespace transmuter::controller::programs