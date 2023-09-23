#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "iostream"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

std::unique_ptr<SurfaceMesh> mesh_ptr;
std::unique_ptr<VertexPositionGeometry> geometry_ptr;
SurfaceMesh* mesh;
VertexPositionGeometry* geometry;

int main(int argc, char const* argv[])
{
    /* code */
    std::tie(mesh_ptr, geometry_ptr) = readSurfaceMesh("../../../input/cowhead.obj");
    mesh = mesh_ptr.release();
    geometry = geometry_ptr.release();
    polyscope::init();

    // todo
    auto psMesh = polyscope::registerSurfaceMesh("mesh", geometry->inputVertexPositions, mesh->getFaceVertexList());

    polyscope::show();
    return 0;
}
