#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "iostream"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

std::unique_ptr<ManifoldSurfaceMesh> mesh_ptr;
std::unique_ptr<VertexPositionGeometry> geometry_ptr;
SurfaceMesh* mesh;
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

    Vector<double> xr(mesh->nVertices());
    Vector<double> yr(mesh->nVertices());
    int cnt = 0;
    for (auto loop : mesh->boundaryLoops())
    {
        for (auto v : loop.adjacentVertices())
        {
            cnt++;
            xr(v.getIndex()) = std::cos(cnt * PI * 2 / static_cast<double>(mesh->nVertices()));
            yr(v.getIndex()) = std::sin(cnt * PI * 2 / static_cast<double>(mesh->nVertices()));
        }
    }
    SparseMatrix<double> mat(mesh->nVertices(), mesh->nVertices());
    for (auto v : mesh->vertices())
    {
        if (!v.isBoundary())
        {
            size_t i = v.getIndex();
            // int n = v.adjacentVertices().end() - v.adjacentVertices().begin();
            for (auto adjv : v.adjacentVertices())
            {
                cnt++;
            }
            double coeff = -1.0 / cnt;
        }
    }



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
