#include "algorithm.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "iostream"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/point_cloud_vector_quantity.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

int main(int argc, char const* argv[])
{
    std::unique_ptr<SurfaceMesh> mesh_ptr;
    std::unique_ptr<VertexPositionGeometry> geometry_ptr;

    std::tie(mesh_ptr, geometry_ptr) = readSurfaceMesh("../../../input/bunny.obj");
    SurfaceMesh* mesh = mesh_ptr.release();
    VertexPositionGeometry* geometry = geometry_ptr.release();

    std::cout << "num of vertices: " << mesh->nVertices() << std::endl;
    std::cout << "please input(source and dist vertex id): ";
    size_t bgn, end;
    std::cin >> bgn >> end;
    std::vector<std::array<size_t, 2>> edges;
    std::vector<Vector3> points;
    Dijkstra(mesh, geometry, bgn, end, points, edges);

    polyscope::init();
    polyscope::registerCurveNetwork("net work", points, edges);
    polyscope::registerSurfaceMesh("my mesh1", geometry->vertexPositions, mesh->getFaceVertexList());

    polyscope::show();

    // psMesh->edge
    // polyscope::registerGroup()
    // psMesh->setEdgeColor({1.0, 1.0, 0.0});

    // polyscope::registerStructure
    // psMesh->addEdgeScalarQuantity
    // polyscope::show();

    // polyscope::init();

    // // 创建网格
    // std::vector<std::array<size_t, 3>> triangles = {{0, 1, 2}};
    // polyscope::registerSurfaceMesh("my mesh", points, triangles);

    // // 设置点云的颜色和点大小
    // // polyscope::getPointCloud("my points")->pointColor = {1.0, 0.0, 0.0};
    // // polyscope::getPointCloud("my points")->pointRadius = 0.02;

    // // 设置网格的颜色
    // polyscope::getSurfaceMesh("my mesh")->setSurfaceColor({0.0, 0.0, 1.0});

    // // 取消地板背景
    // // polyscope::view::bgColor({0.0, 0.0, 0.0, 0.0});
    // // polyscope::view::
    // polyscope::show();
    // std::cout << "hello, world!" << std::endl;
    return 0;
}
