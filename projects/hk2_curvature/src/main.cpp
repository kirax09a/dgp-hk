// #include "colormap.h"
// #include "geometrycentral/surface/manifold_surface_mesh.h"
// #include "geometrycentral/surface/meshio.h"
// #include "geometrycentral/surface/surface_mesh.h"
// #include "iostream"
// #include "polyscope/polyscope.h"
// #include "polyscope/surface_mesh.h"

// using namespace geometrycentral;
// using namespace geometrycentral::surface;

// std::unique_ptr<SurfaceMesh> mesh_ptr;
// std::unique_ptr<VertexPositionGeometry> geometry_ptr;
// SurfaceMesh* mesh;
// VertexPositionGeometry* geometry;

// std::vector<std::array<double, 3>> K_colors;
// std::vector<std::array<double, 3>> H_colors;

// double cotan(Halfedge he)
// {
//     auto he1 = he.next();
//     auto he2 = he1.next();

//     Vector3 v1 = geometry->inputVertexPositions[he1.tailVertex()] - geometry->inputVertexPositions[he1.tipVertex()];
//     Vector3 v2 = geometry->inputVertexPositions[he2.tipVertex()] - geometry->inputVertexPositions[he2.tailVertex()];

//     Vector3 v_cross = cross(v1, v2);
//     double v_dot = dot(v1, v2);

//     return v_dot / v_cross.norm();
// }

// double angle(Corner c)
// {
//     Vector3 v1 = geometry->inputVertexPositions[c.halfedge().tipVertex()] -
//         geometry->inputVertexPositions[c.halfedge().tailVertex()];
//     auto nextedge = c.halfedge().next().next();
//     Vector3 v2 =
//         geometry->inputVertexPositions[nextedge.tailVertex()] - geometry->inputVertexPositions[nextedge.tipVertex()];

//     return std::acos(dot(v1, v2) / (v1.norm() * v2.norm()));
// }

// double vertexDualArea(Vertex v)
// {
//     double totalDualArea = 0.0;
//     for (Corner c : v.adjacentCorners())
//     {
//         Halfedge h1 = c.halfedge();
//         Halfedge h2 = h1.next().next();

//         double edgeLen1 = geometry->edgeLength(h1.edge());
//         double edgeLen2 = geometry->edgeLength(h2.edge());

//         totalDualArea += cotan(h1) * edgeLen1 * edgeLen1 + cotan(h2) * edgeLen2 * edgeLen2;
//     }
//     return totalDualArea * 0.125;
// }

// double vertexMeanCurvature(Vertex v)
// {
//     double meanCurvature = 0.;
//     for (Edge e : v.adjacentEdges())
//     {
//         meanCurvature += geometry->edgeLength(e) * geometry->edgeDihedralAngle(e);
//     }
//     meanCurvature /= 4.;
//     return meanCurvature;
// }

// double vertexGaussianCurvature(Vertex v)
// {
//     double totalInterAngle = 0.0;
//     for (Corner c : v.adjacentCorners())
//     {
//         totalInterAngle += angle(c);
//     }

//     return (PI * 2 - totalInterAngle);
// }

// int main(int argc, char const* argv[])
// {
//     /* code */
//     std::tie(mesh_ptr, geometry_ptr) = readSurfaceMesh("../../../input/cowhead.obj");
//     mesh = mesh_ptr.release();
//     geometry = geometry_ptr.release();

//     Vector<double> K(mesh->nVertices());
//     Vector<double> H(mesh->nVertices());
//     double K_max = 0;
//     double H_max = 0;
//     for (Vertex v : mesh->vertices())
//     {
//         size_t i = v.getIndex();
//         // K[i] = vertexGaussianCurvature(v);
//         K[i] = geometry->vertexGaussianCurvature(v);
//         // H[i] = vertexMeanCurvature(v);
//         H[i] = geometry->vertexMeanCurvature(v);
//         K_max = std::max(abs(K[i]), K_max);
//         H_max = std::max(abs(H[i]), H_max);
//     }

//     if (K_max == 0)
//     {
//         K_max = 1e-3;
//     }

//     if (H_max == 0)
//     {
//         H_max = 1e-3;
//     }

//     for (size_t i = 0; i < mesh->nVertices(); i++)
//     {
//         K_colors.push_back(mapToColor(K[i], -K_max, K_max, "seismic"));
//         H_colors.push_back(mapToColor(H[i], -H_max, H_max, "seismic"));
//     }

//     polyscope::init();
//     auto psMesh = polyscope::registerSurfaceMesh("mesh", geometry->inputVertexPositions, mesh->getFaceVertexList());
//     auto vertexColors = psMesh->addVertexColorQuantity("color", H_colors);
//     vertexColors->setEnabled(true);
//     polyscope::show();

//     // geometry->edgeDihedralAngle

//     return 0;
// }


//-------------------------homework3-----------------------------
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
    std::cout << "----update----" << std::endl;
    DenseMatrix<double> f0(mesh->nVertices(), 3);
    auto M = massMatrix(mesh, geometry);
    std::cout << "----mass matrix done----" << std::endl;
    auto A = buildFlowOperator(mesh, geometry, M, h);
    std::cout << "----flow operator done----" << std::endl;

    for (Vertex v : mesh->vertices())
    {
        auto& p = geometry->inputVertexPositions[v];
        f0.row(v.getIndex()) << p.x, p.y, p.z;
    }

    Vector<double> x0 = M * f0.col(0);
    Vector<double> y0 = M * f0.col(1);
    Vector<double> z0 = M * f0.col(2);

    Vector<double> x_1 = geometrycentral::solveSquare(A, x0);
    Vector<double> y_1 = geometrycentral::solveSquare(A, y0);
    Vector<double> z_1 = geometrycentral::solveSquare(A, z0);
    
    std::cout << "done";
    
    for (auto v : mesh->vertices())
    {
        auto index = v.getIndex();
        geometry->inputVertexPositions[v].x = x_1[index];
        geometry->inputVertexPositions[v].y = y_1[index];
        geometry->inputVertexPositions[v].z = z_1[index];
    }
    // 矩阵求解
}

int main(int argc, char const* argv[])
{
    /* code */
    std::tie(mesh_ptr, geometry_ptr) = readSurfaceMesh("../../../input/bunny.obj");
    mesh = mesh_ptr.release();
    geometry = geometry_ptr.release();
    polyscope::init();
    
    const double h = 0.0001;
    int iteNum;
    std::cout << "input num: ";
    std::cin >> iteNum;
    for (int i = 0; i < iteNum; i++)
    {
        updateNewPositions(mesh, geometry, h);
    }

    // todo
    auto psMesh = polyscope::registerSurfaceMesh("mesh", geometry->inputVertexPositions, mesh->getFaceVertexList());

    polyscope::show();
    return 0;
}
