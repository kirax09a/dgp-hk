#include "geometrycentral/numerical/linear_solvers.h"
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

SparseMatrix<double> massMatrix(SurfaceMesh* mesh, VertexPositionGeometry* geometry)
{
    std::vector<Eigen::Triplet<double>> tripList;

    for (auto v : mesh->vertices())
    {
        tripList.push_back(Eigen::Triplet<double>(v.getIndex(), v.getIndex(), geometry->vertexDualArea(v)));
    }

    SparseMatrix<double> result(mesh->nVertices(), mesh->nVertices());
    result.setFromTriplets(tripList.begin(), tripList.end());

    return result;
}

SparseMatrix<double> laplaceMatrix(SurfaceMesh* mesh, VertexPositionGeometry* geometry)
{
    std::vector<Eigen::Triplet<double>> tripList;

    for (auto v : mesh->vertices())
    {
        double ii_value = 0.0;
        for (auto he : v.outgoingHalfedges())
        {
            double cot = geometry->edgeCotanWeight(he.edge());
            ii_value += cot;
            tripList.push_back(Eigen::Triplet<double>(he.tailVertex().getIndex(), he.tipVertex().getIndex(), -cot));
        }
        tripList.push_back(Eigen::Triplet<double>(v.getIndex(), v.getIndex(), ii_value + 1e-8));
    }

    SparseMatrix<double> result(mesh->nVertices(), mesh->nVertices());
    result.setFromTriplets(tripList.begin(), tripList.end());

    return result;
}

SparseMatrix<double>
buildFlowOperator(SurfaceMesh* mesh, VertexPositionGeometry* geometry, const SparseMatrix<double>& M, double h)
{
    auto L = laplaceMatrix(mesh, geometry);
    auto result = M + h * L;
    return result;
}

void updateNewPositions(SurfaceMesh* mesh, VertexPositionGeometry* geometry, double h)
{
    DenseMatrix<double> f0(mesh->nVertices(), 3);
    auto M = massMatrix(mesh, geometry);
    auto A = buildFlowOperator(mesh, geometry, M, h);

    for (Vertex v : mesh->vertices())
    {
        auto& p = geometry->inputVertexPositions[v];
        f0.row(v.getIndex()) << p.x, p.y, p.z;
    }

    Vector<double> x0 = M * f0.col(0);
    Vector<double> y0 = M * f0.col(1);
    Vector<double> z0 = M * f0.col(2);

    Vector<double> x_1 = geometrycentral::solvePositiveDefinite(A, x0);
    Vector<double> y_1 = geometrycentral::solvePositiveDefinite(A, y0);
    Vector<double> z_1 = geometrycentral::solvePositiveDefinite(A, z0);

    for (auto v : mesh->vertices())
    {
        auto& pos = geometry->inputVertexPositions[v];
        pos.x = x_1[v.getIndex()];
        pos.y = y_1[v.getIndex()];
        pos.z = z_1[v.getIndex()];
    }
    // 矩阵求解
}

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
