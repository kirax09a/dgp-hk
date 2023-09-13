#pragma once
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "queue"
#include "iostream"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/point_cloud_vector_quantity.h"
#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;

struct node
{
    int id;
    double dis;
    node(int id, double d)
    {
        this->id = id;
        this->dis = d;
    }
    bool operator<(const node& rhs) const
    {
        return dis > rhs.dis;
    }
};

void Dijkstra(SurfaceMesh* mesh,
              VertexPositionGeometry* geometry,
              size_t s_p,
              size_t e_p,
              std::vector<Vector3>& points,
              std::vector<std::array<size_t, 2>>& edges);
