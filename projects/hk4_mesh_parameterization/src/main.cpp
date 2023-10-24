#include <cstddef>
#include "geometrycentral/numerical/linear_solvers.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "iostream"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

std::unique_ptr<ManifoldSurfaceMesh> mesh_ptr;
std::unique_ptr<VertexPositionGeometry> geometry_ptr;
ManifoldSurfaceMesh* mesh;
VertexPositionGeometry* geometry;

int main(int argc, char const* argv[])
{
    /* code */
    std::tie(mesh_ptr, geometry_ptr) = readManifoldSurfaceMesh("../../../input/cowhead.obj");
    mesh = mesh_ptr.release();
    geometry = geometry_ptr.release();
    polyscope::init();

    // todo
    auto psMesh = polyscope::registerSurfaceMesh("mesh", geometry->inputVertexPositions, mesh->getFaceVertexList());

    // 边界点放在一个凸多边形上
    Vector<double> xr(mesh->nVertices());
    Vector<double> yr(mesh->nVertices());
    int cnt = 0;
    for (auto loop : mesh->boundaryLoops())
    {
        int numOfBoundary = 0;
        for (auto v : loop.adjacentVertices())
        {
            numOfBoundary++;
        }
        std::cout << "numOfBoundary = " << numOfBoundary << std::endl;
        for (auto v : loop.adjacentVertices())
        {
            cnt++;
            double x = std::cos(cnt * PI * 2 / static_cast<double>(numOfBoundary));
            double y = std::sin(cnt * PI * 2 / static_cast<double>(numOfBoundary));
            std::cout << "v.index() = " << v.getIndex() << std::endl;
            std::cout << "cnt = " << cnt << std::endl;
            std::cout << "x = " << x << std::endl;
            std::cout << "y = " << y << std::endl;
            xr(v.getIndex()) = x;
            yr(v.getIndex()) = y;
        }
    }
    SparseMatrix<double> mat(mesh->nVertices(), mesh->nVertices());
    for (auto v : mesh->vertices())
    {
        // 如果不是边界点
        if (!v.isBoundary())
        {
            int cnt = 0;
            size_t i = v.getIndex();
            // int n = v.adjacentVertices().end() - v.adjacentVertices().begin();
            for (auto adjv : v.adjacentVertices())
            {
                // 计算出边界点的数量
                cnt++;
            }
            double coeff = -1.0 / cnt;
            mat.insert(i, i) = 1.0;
            for (auto adjv : v.adjacentVertices())
            {
                size_t j = adjv.getIndex();
                mat.insert(i, j) = coeff;
            }
        }
        // 边界点
        else
        {
            size_t i = v.getIndex();
            mat.insert(i, i) = 1.0;
        }
    }
    // std::cout << "mat = " << mat << std::endl;

    Solver<double> solver(mat);
    Vector<double> xl = solver.solve(xr);
    std::cout << "solve x ok" << std::endl;
    Vector<double> yl = solver.solve(yr);
    std::cout << "solve y ok" << std::endl;


    std::vector<Vector2> pts(mesh->nVertices());
    for (auto v : mesh->vertices())
    {
        int index = v.getIndex();
        std::cout << "x = " << xl(index) << std::endl;
        std::cout << "y = " << yl(index) << std::endl;
        pts[index].x = xl(index);
        pts[index].y = yl(index);
    }
    auto psParameterization = polyscope::registerSurfaceMesh2D("parameterization", pts, mesh->getFaceVertexList());

    // std::cout << "num of exterior halfedges = " << mesh->nExteriorHalfedges() << std::endl;
    // if (mesh->hasBoundary())
    // {
    //     std::cout << "mesh has boundary" << std::endl;
    // }
    // mesh->boundaryLoop(0);
    // mesh->boundary
    // std::cout << "num of boundary = " << mesh->nBoundaryLoops() << std::endl;
    // std::vector<Vertex> bndVertices;
    // int cnt = 0;
    // std::vector<Vector3> points;
    // std::vector<std::array<size_t, 2>> edges;
    // for (auto bdloop : mesh->boundaryLoops())
    // {
    //     std::cout << "loop" << std::endl;
    //     for (auto v : bdloop.adjacentVertices())
    //     {
    //         cnt++;
    //         if (cnt == 25)
    //         {
    //             break;
    //         }
    //         points.push_back(geometry->inputVertexPositions[v.getIndex()]);
    //         bndVertices.push_back(v);
    //     }
    // }
    // polyscope::registerPointCloud("pointcloud", points);
    // std::cout << "n bndvertices = " << bndVertices.size() << std::endl;

    polyscope::show();
    return 0;
}
